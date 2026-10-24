// Mechanical drivetrain model: what makes driving feel like the real robot instead of an arcade car.
//
// Per side: V5 motors (voltage minus back-EMF, current limit, heat), the gear train's slop
// (backlash: a clunk when the drive reverses), the wheels (they can spin out when the load on
// them drops), and the foam tiles. All four wheels are omnis, so the robot has little sideways grip:
// fast turns drift and other robots can shove it sideways. The chassis also pitches and rolls on
// its wheels: hard stops make it rock, with the cascade up it wheelies and can tip over.
// Everything runs in small substeps, then the net change is handed to the Rapier body.
const IN = 0.0254;

const G = 9.81;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

export const DRIVE = {
	motorsPerSide: 2,
	stallNm100: 2.1,          // V5 11W motor stall torque with the 100 rpm (red) cartridge
	motorRpm: 600,            // blue cartridge free speed
	eff: 0.82,                // gear train + bearings
	rotorMass: 0.55,          // motor + cartridge inertia per side, as an equivalent mass at the wheel (kg)
	wheelMass: 0.12,          // wheels + gears per side (kg)
	slop: 0.07 * IN,          // gear train backlash per side, at the wheel surface (5 red gears in a row)
	slopK: 40000, slopC: 36,  // gear mesh stiffness (N/m) and damping once the slop is taken up
	gearDrag: 0.04,           // gearbox drag as a fraction of stall force (always on)
	viscous: 0.07,
	coastDrag: 0.2,           // extra drag of a coasting (unpowered) motor: the robot rolls ~20" from full speed            // speed-proportional drag, fraction of stall force at free speed
	muS: 1.0, muK: 0.85,      // omni wheel grip on foam tiles: static / spinning
	muLat: 0.11,              // sideways: the rollers roll, so omnis slide sideways easily (fast turns drift, robots get shoved)
	latEps: 0.03,             // m/s: sideways speed at which the rollers' grip is fully used
	latViscous: 1.4,          // 1/s: roller bearing / rubber drag on sideways motion (still there while spinning, when the Coulomb parts cancel)
	rolling: 0.018,           // rolling resistance, fraction of weight
	mismatch: 0.035,          // one side's motors are a bit stronger (worn motors / friction): it pulls to one side
	substeps: 12,
};

export class Drivetrain {
	// geo: { trackWidth, wheelDia, driveRpm, wheelbase (inches between axles), mass (kg),
	//        cog(): CoG height in inches (changes with the lift), length, width }
	constructor(geo, opts = {}) {
		this.geo = geo;
		this.o = { ...DRIVE, ...opts };
		this.reset();
		// Which side is stronger and by how much is fixed for this robot (like real motors).
		const s = this.o.mismatch;
		this.strength = Math.random() < 0.5 ? { L: 1, R: 1 - s } : { L: 1 - s, R: 1 };
	}

	reset() {
		const side = () => ({ vm: 0, vw: 0, slop: 0, slip: false, temp: 32, F: 0, cmd: 0 });
		this.L = side(); this.R = side();
		this.pitch = { th: 0, w: 0, susp: 0, suspW: 0 }; this.roll = { th: 0, w: 0, susp: 0, suspW: 0 };
		this.tipped = 0;            // 0 upright; otherwise +-1 = fell nose up / nose down, +-2 = on its side
		this.prevV = null; this.acc = { f: 0, l: 0 };
		this.encL = 0; this.encR = 0;   // motor encoder distance (m at the wheel), slop and slip included
	}

	vfree() { const g = this.geo; return (g.driveRpm / 60) * 2 * Math.PI * (g.wheelDia / 2) * IN; }
	fstall() {   // ground force from one side's motors at stall (N)
		const g = this.geo, o = this.o, r = (g.wheelDia / 2) * IN;
		return (o.motorsPerSide * (o.stallNm100 * 100 / o.motorRpm) * (o.motorRpm / g.driveRpm) * o.eff) / r;
	}

	// V5 motors cut their current as they heat up (55 C: half, 60 C: quarter, 65 C+: off-ish).
	currentLimit(temp) { return temp < 55 ? 1 : temp < 60 ? 0.5 : temp < 65 ? 0.25 : 0.1; }

	// One physics step. cmd: { L, R } in -127..127, brake: 'coast' | 'brake' | 'hold'.
	// v: { f, l, w } robot-local forward / left-right (m/s, +right) / yaw rate (rad/s, CCW+).
	// Returns the new { f, l, w } the body should have (before collisions).
	step(dt, cmd, brake, v) {
		const g = this.geo, o = this.o, m = g.mass;
		const tw = g.trackWidth * IN, xw = (g.wheelbase / 2) * IN;
		const I = m * ((g.length * IN) ** 2 + (g.width * IN) ** 2) / 12 * 0.85;
		const vfree = this.vfree(), Fst = this.fstall();
		const n = o.substeps, h = dt / n;
		let { f, l, w } = v;
		const tipped = this.tipped !== 0;
		// Weight on each side / axle: shifts with the measured acceleration (and goes to zero on a wheelie).
		const cog = g.cog() * IN;
		const lift = (a, half) => clamp((m * a * cog) / (2 * half) / (m * G), -0.5, 0.5);   // fraction moved
		const pitchOff = Math.abs(this.pitch.th) > 0.002, rollOff = Math.abs(this.roll.th) > 0.002;
		const sideLoad = { L: 0.5 - (rollOff ? (this.roll.th > 0 ? 0.5 : -0.5) : lift(this.acc.l, tw / 2)), R: 0 };
		sideLoad.R = 1 - sideLoad.L;
		const front = pitchOff ? (this.pitch.th > 0 ? 0 : 1) : 0.5 - lift(this.acc.f, xw);
		for (const k of ['L', 'R']) { this[k].cmd = tipped ? 0 : clamp(cmd[k] || 0, -127, 127); }
		let enc = { L: 0, R: 0 };
		for (let i = 0; i < n; i++) {
			let Ff = 0;
			for (const k of ['L', 'R']) {
				const s = this[k];
				const N = m * G * sideLoad[k];
				// Motor: voltage minus back-EMF, current limited (heat), plus gearbox drag.
				const u = s.vm / vfree, lim = this.currentLimit(s.temp) * this.strength[k];
				let tn = 0;   // coasting with no command: the motor just spins along
				if (s.cmd !== 0) tn = clamp(s.cmd / 127 - u, -lim, lim);
				else if (brake !== 'coast') tn = clamp(-u * (brake === 'hold' ? 3 : 1), -lim, lim);   // shorted windings
				const drag = o.gearDrag + (s.cmd === 0 && brake === 'coast' ? o.coastDrag : 0);
				const Fm = tn * Fst - drag * Fst * Math.tanh(s.vm / 0.02) - o.viscous * Fst * u;
				// Gear train slop: no force until it's taken up, then a stiff mesh.
				let Fc = 0;
				if (s.slop > o.slop) Fc = Math.max(0, o.slopK * (s.slop - o.slop) + o.slopC * (s.vm - s.vw));
				else if (s.slop < -o.slop) Fc = Math.min(0, o.slopK * (s.slop + o.slop) + o.slopC * (s.vm - s.vw));
				// Wheel on the tile: sticks until the force needed is more than the grip, then spins.
				const vgr = k === 'L' ? f - w * tw / 2 : f + w * tw / 2;
				let Fg = 0;
				if (N <= 0) s.slip = true;
				if (!s.slip) {
					if (Math.abs(Fc) <= o.muS * N) { Fg = Fc; s.vw = vgr; }
					else s.slip = true;
				}
				if (s.slip) {
					const dv = s.vw - vgr, dir = dv !== 0 ? Math.sign(dv) : Math.sign(Fc);
					Fg = N > 0 ? dir * o.muK * N : 0;
					let vwn = s.vw + ((Fc - Fg) / o.wheelMass) * h;
					if (N > 0 && dv !== 0 && Math.sign(vwn - vgr) !== Math.sign(dv)) {   // wheel speed caught up with the ground
						vwn = vgr;
						if (Math.abs(Fc) <= o.muS * N) { s.slip = false; Fg = Fc; }
					}
					s.vw = vwn;
				}
				s.vm += ((Fm - Fc) / o.rotorMass) * h;
				s.slop = clamp(s.slop + (s.vm - s.vw) * h, -o.slop * 8, o.slop * 8);   // (safety only: the mesh spring carries the load)
				s.F = Fg;
				enc[k] += s.vm * h;
				// Heat: I^2 R in, cooling toward the room (a stalled motor gets hot in about a minute).
				s.temp += (tn * tn * 0.55 - (s.temp - 30) * 0.004) * h;
				Ff += Fg;
			}
			// Yaw torque from the two sides: +z (right) pushing forward turns CCW (left).
			const Tdrive = (this.R.F - this.L.F) * tw / 2;
			// Omni rollers: sideways friction at each wheel. Wheels at x = +-xw; their sideways speed
			// is l - w*x. Normal loads per wheel from the axle / side load split.
			let Fl = 0, Tl = 0;
			for (const [x, fr] of [[xw, front], [-xw, 1 - front]]) {
				for (const k of ['L', 'R']) {
					const Nw = m * G * fr * sideLoad[k];
					const vlat = l - w * x;
					const fl = -o.muLat * Nw * clamp(vlat / o.latEps, -1, 1);
					Fl += fl; Tl += -x * fl;
				}
			}
			// Rolling resistance (always against the motion, never reverses it).
			const roll = o.rolling * m * G;
			const fRoll = -Math.sign(f) * Math.min(roll, (Math.abs(f) * m) / h);
			f += ((Ff + fRoll) / m) * h;
			l += (Fl / m) * h;
			l *= 1 - o.latViscous * h;
			w += ((Tdrive + Tl) / I) * h;
		}
		this.encL += enc.L; this.encR += enc.R;
		this.driveForce = this.L.F + this.R.F;
		return { f, l, w };
	}

	// After the physics step: measured accelerations (collisions included) drive pitch and roll.
	// a: { f, l } m/s^2 robot-local. Returns true the moment the robot tips over.
	stepTilt(dt, a) {
		const g = this.geo, m = g.mass;
		// Filter: the solver's contact pushes come in spikes.
		const k = 1 - Math.exp(-dt / 0.025);
		this.acc.f += (clamp(a.f, -40, 40) - this.acc.f) * k;
		this.acc.l += (clamp(a.l, -40, 40) - this.acc.l) * k;
		if (this.tipped) return false;
		const h = g.cog() * IN, xw = (g.wheelbase / 2) * IN, yw = (g.trackWidth / 2 + 0.6) * IN;
		const Ip = m * (h * h + xw * xw) + m * ((g.length * IN) ** 2 + (g.height * IN) ** 2) / 12 * 0.6;
		const Ir = m * (h * h + yw * yw) + m * ((g.width * IN) ** 2 + (g.height * IN) ** 2) / 12 * 0.6;
		let fell = false;
		// th > 0: nose up (pivot on the rear axle). Pitch is driven by forward acceleration.
		const axis = (st, aIn, half, Iax, sign) => {
			const sub = 8, hh = dt / sub;
			for (let i = 0; i < sub; i++) {
				const th = st.th, piv = th > 0 || (th === 0 && aIn * sign > 0) ? 1 : -1;
				const c = Math.cos(th), s = Math.sin(th);
				// Torque about the axle it's rocking on (see derivation in the file header):
				const rx = piv * half;   // CoG is `half` in front of the rear axle (piv +1), behind the front one (-1)
				const tq = piv > 0
					? -m * G * (rx * c - h * s) + m * aIn * sign * (rx * s + h * c)
					: m * G * (half * c + h * s) + m * aIn * sign * (h * c - half * s);
				if (th === 0 && ((piv > 0 && tq <= 0) || (piv < 0 && tq >= 0))) { st.w = 0; continue; }   // all wheels down
				st.w += (tq / Iax) * hh;
				st.w *= 1 - 0.6 * hh;
				const next = th + st.w * hh;
				if (th !== 0 && Math.sign(next) !== Math.sign(th)) { st.th = 0; st.w = Math.abs(st.w) < 0.4 ? 0 : -st.w * 0.15; }   // wheels land, small bounce
				else st.th = next;
				if (Math.abs(st.th) > 1.25) { fell = true; break; }
			}
			// Visible rock on the tiles / frame flex even when nothing lifts: a stiff, lightly damped spring.
			const target = clamp((m * aIn * sign * h) / 350, -0.05, 0.05);
			const wn = 2 * Math.PI * 6, z = 0.22;
			st.suspW += (wn * wn * (target - st.susp) - 2 * z * wn * st.suspW) * dt;
			st.susp += st.suspW * dt;
		};
		axis(this.pitch, this.acc.f, xw, Ip, 1);
		if (fell) { this.tipped = this.pitch.th > 0 ? 1 : -1; return true; }
		axis(this.roll, this.acc.l, yw, Ir, 1);
		if (fell) { this.tipped = this.roll.th > 0 ? 2 : -2; return true; }
		return false;
	}

	// Visual tilt (rad): pitch nose-up +, roll right-side-up +, and the axle each pivots on.
	tilt() {
		const g = this.geo, xw = g.wheelbase / 2, yw = g.trackWidth / 2 + 0.6;
		if (this.tipped) {
			const onEnd = Math.abs(this.tipped) === 1, sg = Math.sign(this.tipped);
			return onEnd ? { pitch: sg * Math.PI / 2, roll: 0, px: -sg * xw, pz: 0 } : { pitch: 0, roll: sg * Math.PI / 2, px: 0, pz: -sg * yw };
		}
		const p = this.pitch.th, r = this.roll.th;
		// Nose up pivots on the rear axle, nose down on the front one; right side up on the left wheels.
		return { pitch: p + this.pitch.susp, roll: r + this.roll.susp, px: -Math.sign(p) * xw, pz: -Math.sign(r) * yw };
	}

	maxTemp() { return Math.max(this.L.temp, this.R.temp); }
}
