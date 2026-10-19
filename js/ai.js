// AI-controlled robots for 2v2 matches. Each one uses the same drive, lift, claw and intake
// physics as the Titanium robot, but has a simpler body and a small state machine that
// picks up a pin, carries it to a goal and drops it on.
import * as THREE from 'three';
import { IN, W, toField, PIN_H, PIN_COLLAR, CUP_H, COLORS, GROUP, groups, TOGGLE } from './field.js';
import { Robot, CFG } from './robot.js';

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const DEG = Math.PI / 180;
const colsOf = (o) => (o.collider ? [o.collider, ...o.extra] : Array.from({ length: o.body.numColliders() }, (_, i) => o.body.collider(i)));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const CLAW_REACH = 6.5 + CFG.clawArm;          // robot centre -> held pin with the claw at 90 deg
const LIFT_FOR = { alliance: CFG.LIFT_ALLIANCE_GOAL_DEG, short: CFG.LIFT_SHORT_GOAL_DEG, tall: CFG.LIFT_TALL_GOAL_DEG };

// Strategies the AI robots can play, and the kind of robot each one is (from early-season Override
// designs: intake + flip claw, cascade + claw, low wedge / plow bots for Toggles and defense,
// DR4B lift with a claw).
export const STRATEGIES = {
	scorer: { name: 'Scorer', style: 'intake' },              // alliance-colour pins on any goal
	stacker: { name: 'Yellow stacker', style: 'cascade' },    // tall cup + yellow pin stacks on goals we own
	attacker: { name: 'Toggle attacker', style: 'wedge' },    // flips the other alliance's Toggles to our colour
	defender: { name: 'Toggle defender', style: 'dr4b' },     // keeps our Toggles our colour and blocks robots going for them
	pusher: { name: 'Pusher (defense)', style: 'wedge' },     // shoves opponents around, mostly the one carrying a piece
};

function aiMesh(style = 'intake') {
	const g = new THREE.Group(), s = IN;
	const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, metalness: 0.4, ...o });
	const metal = mat(0xb8bcc2, { metalness: 0.8, roughness: 0.35 }), dark = mat(0x222428), bump = mat(0x888888, { metalness: 0.1 });
	const grey = mat(0x8c9096, { metalness: 0.1, roughness: 0.8 });
	const box = (w, h, d, x, y, z, m, parent = g) => {
		const b = new THREE.Mesh(new THREE.BoxGeometry(w * s, h * s, d * s), m);
		b.position.set(x * s, y * s, z * s); b.castShadow = true; parent.add(b); return b;
	};
	const bar = (a, b, w, m, parent = g) => {   // a box from point a to point b (side view, x/y), w thick
		const dx = b[0] - a[0], dy = b[1] - a[1], o = box(Math.hypot(dx, dy), w, w, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2], m, parent);
		o.rotation.z = Math.atan2(dy, dx); return o;
	};
	const L = CFG.length, Wd = CFG.width, hl = L / 2, hw = Wd / 2;
	box(L, 1, Wd - 3, 0, 1.5, 0, metal);                                    // base plate
	box(L, 2.5, 1.2, 0, 2.2, -hw + 0.6, bump); box(L, 2.5, 1.2, 0, 2.2, hw - 0.6, bump);   // side bumpers
	box(1.2, 2.5, Wd, -hl + 0.6, 2.2, 0, bump);                             // back bumper
	box(4, 2, Wd - 4, 3, 3, 0, dark);                                       // battery / Brain block
	const wheels = [];
	for (const x of [-hl + 3, hl - 3]) for (const z of [-hw + 1.6, hw - 1.6]) {
		const axle = new THREE.Group(); axle.position.set(x * s, 1.625 * s, z * s); axle.rotation.x = Math.PI / 2; g.add(axle);
		const w = new THREE.Mesh(new THREE.CylinderGeometry(1.625 * s, 1.625 * s, 1 * s, 16), dark); w.castShadow = true; axle.add(w);
		wheels.push({ w, side: z < 0 ? 'L' : 'R' });
	}
	const intake = [], top = new THREE.Group(), stages = [{ g: top, frac: 1 }];
	g.add(top);
	const pivot = new THREE.Group(); pivot.position.set(-6.5 * s, (CFG.pinBottomAtHorizontal + PIN_H - 1) * s, 0); top.add(pivot);
	if (style === 'intake') {
		// Front roller intake, two-stage lift towers at the back, claw arm hanging from the top stage.
		for (const y of [2, 5]) {
			const r = new THREE.Mesh(new THREE.CylinderGeometry(0.8 * s, 0.8 * s, (Wd - 4) * s, 10), mat(0x3a3d42));
			r.rotation.x = Math.PI / 2; r.position.set((hl - 0.5) * s, y * s, 0); g.add(r); intake.push(r);
		}
		for (const z of [-hw + 2, hw - 2]) box(1, 14, 1, -hl + 2.5, 9, z, metal);
		for (const z of [-hw + 2.8, hw - 2.8]) box(1, 13, 1, -hl + 2.5, 10, z, metal, top);
		box(1.2, 1, Wd - 4, -hl + 2.5, 16, 0, metal, top);
		for (const z of [-3, 3]) box(1, CFG.clawArm, 0.8, 0, -CFG.clawArm / 2, z, metal, pivot);
		box(1.4, 1.4, 7, 0, -CFG.clawArm, 0, dark, pivot);
	} else if (style === 'cascade') {
		// Tall three-stage cascade in the middle, flex-wheel claw out the back (like ours).
		for (let k = 0; k < 3; k++) {
			const st = k === 2 ? top : new THREE.Group();
			if (k < 2) { g.add(st); stages.push({ g: st, frac: k / 2 }); }
			for (const z of [-hw + 1.5 + k * 0.6, hw - 1.5 - k * 0.6]) box(1, 20 - k, 1.6, -1 + k * 0.3, 12 + k * 0.5, z, mat(0x2c62c9), st);
		}
		box(1.2, 1, Wd - 4, -0.6, 22.5, 0, mat(0x2c62c9), top);
		box(6, 1, 7, -hl + 2, 7, 0, mat(0x2c62c9), top);                          // claw plate
		for (const z of [-2.5, 2.5]) for (const y of [1.3, 3.1, 4.9]) {
			const w = new THREE.Mesh(new THREE.CylinderGeometry(1.2 * s, 1.2 * s, 1 * s, 14), grey);
			w.position.set((-hl + 2) * s, y * s, z * s); w.castShadow = true; top.add(w);
		}
		box(1, 1, 1, 0, 0, 0, dark, pivot).visible = false;
	} else if (style === 'wedge') {
		// Low and heavy: a steel plow across the front (rams Toggles, pushes robots), no lift to speak of.
		const plow = mat(0x9aa1a8, { metalness: 0.9, roughness: 0.25 });
		for (const z of [-1, 1]) bar([hl - 3, 6.5, z * (hw - 1)], [hl + 1.5, 1, z * (hw - 1)], 0.8, metal);
		const face = box(0.4, 6.5, Wd + 0.6, hl - 0.6, 3.6, 0, plow); face.rotation.z = -0.65;
		box(L - 6, 1, Wd - 4, -1, 6.5, 0, metal);                               // top deck
		box(1, 6, 1, -hl + 3, 9, 0, metal); box(6, 1.6, 2.5, -hl + 4, 12, 0, dark);   // short mast + small claw
		for (const z of [-hw + 0.6, hw - 0.6]) box(L, 1.2, 1.2, 0, 4.4, z, bump);
		box(1, 1, 1, 0, 0, 0, dark, pivot).visible = false;
	} else {
		// DR4B: two crossed four-bar stages on each side, claw on the top.
		const red = mat(0xc8202a, { roughness: 0.45 });
		for (const z of [-hw + 1.4, hw - 1.4]) {
			box(1, 8, 1, -hl + 4, 5, z, metal); box(1, 8, 1, hl - 4, 5, z, metal);   // towers
			bar([-hl + 4, 8, z], [hl - 4, 15, z], 0.8, red, top); bar([-hl + 4, 15, z], [hl - 4, 8, z], 0.8, red, top);
			bar([-hl + 4, 15, z], [hl - 4, 22, z], 0.8, red, top); bar([-hl + 4, 22, z], [hl - 4, 15, z], 0.8, red, top);
		}
		box(L - 6, 1, Wd - 2, 0, 22.5, 0, metal, top);
		for (const z of [-2.5, 2.5]) box(1.2, 5, 1, -hl + 1, 20, z, dark, top);   // claw fingers off the back of the top
		box(1, 1, 1, 0, 0, 0, dark, pivot).visible = false;
	}
	return { group: g, wheels, intake, stages, clawPivot: pivot, bump };
}

// The AI robots keep the old Titanium design: a front intake that hands pieces up a ramp to a claw
// that flips out the back (the Titanium robot itself now has the flex-wheel claw, robot.js).
export class AIRobot extends Robot {
	constructor(R, world, scene, field, alliance, label) {
		super(R, world, scene, field);
		this.alliance = alliance; this.label = label;
		this.bumpMat.color.setHex(COLORS[alliance]);
		this.ai = { state: 'fetch', target: null, t: 0, stuckT: 0, lastPos: null, ban: new Map() };
	}

	// Pick a strategy (STRATEGIES key); the robot is rebuilt as the kind of robot that plays it.
	setStrategy(key) {
		this.strategy = STRATEGIES[key] ? key : 'scorer';
		const style = STRATEGIES[this.strategy].style;
		if (style !== this.style) {
			const vis = this.mesh.visible;
			this.scene.remove(this.mesh);
			this.style = style;
			this.mesh = this.buildMesh();
			this.mesh.visible = vis;
			this.scene.add(this.mesh);
			this.bumpMat.color.setHex(COLORS[this.alliance]);
		}
		this.resetState();
	}

	buildMesh() {
		const m = aiMesh(this.style);
		Object.assign(this, { wheels: m.wheels, intakeRollers: m.intake, clawRollers: [], pistons: [], stages: m.stages,
			clawPivot: m.clawPivot, driveGears: [], liftGears: [], liftLeds: [], bumpMat: m.bump });
		return m.group;
	}

	resetState() {
		super.resetState();
		this.clawDeg = 0; this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.clawVel = CFG.CLAW_TURN_DOWN_VELOCITY;
		this.intakeCmd = 0; this.clawTurnHorizontal = false;
		if (this.ai) Object.assign(this.ai, { state: 'fetch', target: null, t: 0, stuckT: 0, lastPos: null });
	}

	// ---- Old claw + intake mechanics ----------------------------------------------------
	build() {
		super.build();
		const { R, world } = this, hw = CFG.width / 2;
		// End channel of the claw frame: with the claw horizontal it's what hits the Toggle. Checked
		// geometrically (clawTouches) rather than as a collider: with the robot locked to the floor, a
		// collider pressed by the turning Toggle makes the solver fling the robot.
		const cw = hw - 1.6;
		this.clawSeg = [[0, -CFG.clawArm, -cw - 0.5], [0, -CFG.clawArm, cw + 0.5]];
		// Claw frame (side arms + end channel) as colliders that swing with the claw, so it can't pass
		// through pins and cups. They skip the Toggle, walls and goals (see GROUP.fixed in field.js).
		const arm = CFG.clawArm;
		const clawBox = (hx, hy, hz) => world.createCollider(R.ColliderDesc.cuboid(hx * IN, hy * IN, hz * IN).setDensity(0).setFriction(0)
			.setCollisionGroups(groups(GROUP.claw, 0xffff & ~(GROUP.toggle | GROUP.wall | GROUP.fixed))), this.body);
		this.clawCols = [
			{ c: clawBox(1.0, 0.5, (cw + 0.5 - 2.2) / 2), p: [0, -arm, -(cw + 0.5 + 2.2) / 2] },
			{ c: clawBox(1.0, 0.5, (cw + 0.5 - 2.2) / 2), p: [0, -arm, (cw + 0.5 + 2.2) / 2] },
			{ c: clawBox(0.5, arm / 2, 0.5), p: [0, -arm / 2, -cw] },
			{ c: clawBox(0.5, arm / 2, 0.5), p: [0, -arm / 2, cw] },
		];
	}

	clawArmAngle() { return this.clawDeg * CFG.clawDegPerMotorDeg * DEG; }   // 0 = hanging down, 90 = pointing back
	// Held pin pose: lying flat with the claw down, hanging straight down at 90 deg.
	clawPinLocal() {
		const a = this.clawArmAngle();
		const pivotY = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		const gx = -6.5 - CFG.clawArm * Math.sin(a), gy = pivotY - CFG.clawArm * Math.cos(a);
		const off = PIN_H / 2 - 1, angle = Math.PI / 2 - a;
		const pos = new THREE.Vector3(gx + Math.cos(a) * off, gy - Math.sin(a) * off, 0);
		const lowest = pos.y - Math.abs(Math.cos(angle)) * (PIN_H / 2) - Math.abs(Math.sin(angle)) * 1.58;
		if (lowest < 0.3) pos.y += 0.3 - lowest;
		return { pos, angle };
	}
	// Which way round a piece goes in (the end that comes in first stays first).
	decideFlip(o, angle) {
		const def = new THREE.Vector3(0, 1, 0).applyQuaternion(this.quat().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), angle)));
		const r = o.body.rotation(), act = new THREE.Vector3(0, 1, 0).applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w));
		o.flipped = def.dot(act) < 0;
	}
	// Is the end channel of the claw frame touching this Toggle?
	clawTouches(tg) {
		const a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		const c = tg.body.translation(), [p0, p1] = this.clawSeg;
		for (let i = 0; i <= 8; i++) {
			const x = p0[0], y = p0[1], z = p0[2] + ((p1[2] - p0[2]) * i) / 8;
			const d = this.toWorld(-6.5 + x * ca + y * sa, py - x * sa + y * ca, z).sub(new THREE.Vector3(c.x, c.y, c.z));
			const along = d.dot(tg.axis);
			if (Math.abs(along) < (TOGGLE.len / 2) * IN && d.addScaledVector(tg.axis, -along).length() < 1.9 * IN) return true;
		}
		return false;
	}
	// The claw frame against a Toggle or sticking out over a wall stops the robot going that way.
	holdAtToggle() {
		const [p0] = this.clawSeg, a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		for (const z of [p0[2], 0, -p0[2]]) {
			const f = toField(this.toWorld(-6.5 + p0[0] * ca + p0[1] * sa, py - p0[0] * sa + p0[1] * ca, z));
			if (f.h > 12.5) continue;
			const v = this.body.linvel();
			if (Math.abs(f.fx) > 71.4 && Math.sign(v.x) === Math.sign(f.fx)) this.body.setLinvel({ x: 0, y: 0, z: v.z }, true);
			if (Math.abs(f.fy) > 71.4 && Math.sign(-v.z) === Math.sign(f.fy)) this.body.setLinvel({ x: this.body.linvel().x, y: 0, z: 0 }, true);
		}
		const tg = this.clawToggle;
		if (!tg) { this.toggleHitV = 0; return; }
		const v = this.body.linvel(), n = tg.nWorld, vn = v.x * n.x + v.z * n.z;
		this.toggleHitV = Math.max(this.toggleHitV || 0, -vn);
		if (vn < 0) this.body.setLinvel({ x: v.x - vn * n.x, y: 0, z: v.z - vn * n.z }, true);
	}

	stepMechanisms(dt) {
		super.stepMechanisms(dt);
		// The claw motor stalls with the frame pressed on a Toggle: it can't swing further that way.
		this.clawToggle = this.field.toggles.find((tg) => this.clawTouches(tg)) || null;
		const step = this.clawVel * 6 * dt, e = this.clawTarget - this.clawDeg;
		const move = this.clawToggle && Math.sign(e) === this.clawDir ? 0 : clamp(e, -step, step);
		if (move) this.clawDir = Math.sign(move);
		this.clawDeg += move;
		const a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		const rq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -a);
		for (const k of this.clawCols) {
			const [x, y, z] = k.p;
			k.c.setTranslationWrtParent({ x: (-6.5 + x * ca + y * sa) * IN, y: (py - x * sa + y * ca) * IN, z: z * IN });
			k.c.setRotationWrtParent({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		}
	}

	stepPins(dt) {
		for (const r of this.justReleased || []) if ((r.t -= dt) <= 0 && !r.pin.held && (this.field.pins.includes(r.pin) || this.field.cups.includes(r.pin))) for (const c of colsOf(r.pin)) c.setCollisionGroups(0xffffffff);
		if (this.justReleased) this.justReleased = this.justReleased.filter((r) => r.t > 0);
		// Intake: one piece at a time, only with the claw down.
		if (this.intakeCmd > 30 && !this.hopper && !this.claw && this.clawDeg < 150) {
			for (const pin of [...this.field.pins, ...this.field.cups]) {
				if (pin.held) continue;
				const f = toField(pin.body.translation());
				if (this.field.goals.some((g) => Math.hypot(f.fx - g.fx, f.fy - g.fy) < 2.5 && f.h > g.base)) continue;   // seated on a post
				const l = this.toLocal(new THREE.Vector3().copy(pin.body.translation()));
				if (l.x > CFG.length / 2 - 3 && l.x < CFG.length / 2 + 4.5 && Math.abs(l.z) < 5 && l.y < 8) { this.decideFlip(pin, Math.PI / 2); this.attachPin(pin, 'hopper'); break; }
			}
		}
		// Hand-off up the deflector to the claw: claw down, lift down, claw spinning in.
		if (this.hopper && !this.claw && this.clawSpin < -30 && this.clawDeg < 150 && this.liftDeg < CFG.CASCADE_INTAKE_DEG + 60) {
			this.transferT += dt;
			if (this.transferT > 0.25) { const p = this.hopper; this.hopper = null; p.held = null; this.attachPin(p, 'claw'); this.transferT = 0; }
		} else this.transferT = 0;
		if (this.claw && this.clawSpin > 30) {
			this.releaseT += dt;
			if (this.releaseT > 0.12) { this.releaseClaw(true, true); this.releaseT = 0; }
		} else this.releaseT = 0;
		// Move held pieces.
		const q = this.quat(), Z = new THREE.Vector3(0, 0, 1);
		const pose = (pin, l, angle) => {
			const rq = q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Z, angle));
			pin.body.setNextKinematicTranslation(this.toWorld(l.x, l.y, l.z));
			pin.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		};
		const flip = (o) => (o.flipped ? Math.PI : 0);
		if (this.hopper) pose(this.hopper, new THREE.Vector3(2.5, 4.5, 0), Math.PI / 2 + flip(this.hopper));   // lying flat on the ramp
		if (this.claw) {
			const l = this.clawPinLocal(); l.angle += flip(this.claw); pose(this.claw, l.pos, l.angle);
			const e = this.clawExtraPiece, rel = this.clawRel;
			if (e && rel) {
				const mq = q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Z, l.angle)), mp = this.toWorld(l.pos.x, l.pos.y, l.pos.z);
				const p = rel.pos.clone().applyQuaternion(mq).add(mp), rq = mq.clone().multiply(rel.quat);
				e.body.setNextKinematicTranslation(p);
				e.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
			}
		}
	}

	syncMesh(dt) {
		super.syncMesh(dt);
		this.clawPivot.rotation.z = -this.clawArmAngle();
		for (const rr of this.intakeRollers) rr.rotation.z += (this.intakeCmd / 127) * 25 * dt;
	}

	fpos() { return toField(this.pos()); }
	heading() { const f = new THREE.Vector3(1, 0, 0).applyQuaternion(this.quat()); return Math.atan2(-f.z, f.x); }

	// If a goal (other than the one we're going to) sits on the straight line to (fx, fy), aim for
	// a point beside it instead, on the side we're already on. Returns the point to drive at.
	avoid(fx, fy, except) {
		const p = this.fpos(), dx = fx - p.fx, dy = fy - p.fy, L2 = dx * dx + dy * dy;
		if (L2 < 1) return { fx, fy };
		let best = null, bestT = 2;
		for (const g of this.field.goals) {
			if (g === except || Math.hypot(g.fx - fx, g.fy - fy) < 6) continue;
			const t = ((g.fx - p.fx) * dx + (g.fy - p.fy) * dy) / L2;
			if (t <= 0 || t >= 1) continue;
			const cx = p.fx + dx * t, cy = p.fy + dy * t, off = Math.hypot(g.fx - cx, g.fy - cy);
			const clear = g.type === 'tall' ? 22 : 18;   // goal base + the robot's half-diagonal + a margin
			if (off < clear && t < bestT) { bestT = t; best = { g, cx, cy, off, clear }; }
		}
		if (!best) return { fx, fy };
		const { g, cx, cy, off, clear } = best;
		let nx = cx - g.fx, ny = cy - g.fy;   // push the waypoint out from the goal, our side of it
		if (off < 0.5) { nx = -dy; ny = dx; }
		const n = Math.hypot(nx, ny);
		return { fx: clamp(g.fx + (nx / n) * (clear + 3), -62, 62), fy: clamp(g.fy + (ny / n) * (clear + 3), -62, 62) };
	}

	// Arcade drive toward a field point (steering around goals in the way); returns the distance
	// left to the real target (inches).
	driveTo(fx, fy, speed = 0.8, reverse = false, except = this.ai?.target) {
		const real = Math.hypot(fx - this.fpos().fx, fy - this.fpos().fy);
		if (!reverse && real > 8) ({ fx, fy } = this.avoid(fx, fy, except));
		const p = this.fpos(), d = Math.hypot(fx - p.fx, fy - p.fy);
		const err = wrap(Math.atan2(fy - p.fy, fx - p.fx) + (reverse ? Math.PI : 0) - this.heading());
		const turn = this.steer(err) * Math.min(1, (this.skill || 0.8) / 0.8);
		const fwd = Math.cos(err) > 0.8 ? clamp(d / 14, 0.25, 1) * speed * (this.skill || 0.8) / 0.8 * Math.cos(err) * (reverse ? -1 : 1) : 0;
		this.arcade(fwd, turn);
		return real;
	}
	turnTo(angle) { const err = wrap(angle - this.heading()); this.arcade(0, this.steer(err) * 0.8); return Math.abs(err); }
	// PD on heading: the spin-rate term brakes the turn before it overshoots.
	steer(err) {
		const w = this.body.angvel().y, out = clamp(1.1 * err - 0.32 * w, -0.85, 0.85);
		return Math.abs(err) > 0.04 && Math.abs(out) < 0.22 ? Math.sign(err) * 0.22 : out;   // enough to beat wheel drag
	}
	setState(st) { Object.assign(this.ai, { state: st, t: 0, stuckT: 0, lastPos: null, aligning: false }); }
	arcade(fwd, turn) { this.leftCmd = clamp(fwd - turn, -1, 1) * 127; this.rightCmd = clamp(fwd + turn, -1, 1) * 127; }

	// Called every physics step. `others` are all robots (to avoid chasing the same pin, and to find
	// opponents to block or push). this.clockLeft = seconds of driver control left (null otherwise).
	think(dt, others) {
		const ai = this.ai;
		ai.t += dt;
		for (const [k, until] of ai.ban) if (until < this.t) ai.ban.delete(k);
		this.liftMode = 'pos'; this.brake = 'coast';
		// Endgame: everyone parks in our half of the Midfield for the 8 points.
		if (this.clockLeft != null && this.clockLeft < 9) {
			if (this.claw && !this.field.cups.includes(this.claw)) this.releaseClaw(true);
			this.intakeCmd = 0; this.clawSpin = 0; this.park(this.fpos()); return;
		}
		// Stuck check: barely moved in 1.5 s while trying to drive -> back off and pick something else.
		// Not while it means to be pressed against something (ramming a Toggle, pushing a robot).
		const p = this.fpos();
		if (ai.state !== 'unstick' && !ai.aligning && !ai.pressing) {
			ai.stuckT += dt;
			if (ai.stuckT > 1.5) {
				if (ai.lastPos && Math.hypot(p.fx - ai.lastPos.fx, p.fy - ai.lastPos.fy) < 2.5) {
					if (ai.target) ai.ban.set(ai.target, this.t + 8);
					if (ai.tgt) ai.ban.set(ai.tgt, this.t + 10);   // can't get to that Toggle (robot in the way): try another for a while
					ai.prev = ai.state; ai.state = 'unstick'; ai.t = 0; ai.target = null; ai.tgt = null;
				}
				ai.stuckT = 0; ai.lastPos = p;
			}
		}
		ai.pressing = false;
		if (ai.state === 'unstick') {
			this.arcade(-0.6, ai.t < 0.5 ? 0.5 : 0);
			if (ai.t > 0.9) { ai.state = ai.prev === 'unstick' ? 'fetch' : ai.prev; ai.t = 0; ai.stuckT = 0; ai.lastPos = null; }
			return;
		}
		const mine = this.alliance, theirs = mine === 'red' ? 'blue' : 'red';
		const toggles = this.field.toggles;
		if (this.strategy === 'attacker') {
			// Their Toggles first (the one guarding the most yellow pins), then ours if they've been
			// turned; all set: sit by their Toggles and shove away anyone coming to turn them back.
			const want = toggles.filter((t) => t.state !== mine && !ai.ban.has(t)).sort((a, b) => (b.quad.startsWith(theirs) - a.quad.startsWith(theirs)) || this.yellowIn(b.quad) - this.yellowIn(a.quad));
			if (want.length) { this.flipToggle(want[0], mine, dt); return; }
			this.guard(toggles.filter((t) => t.quad.startsWith(theirs)), others, dt);
			return;
		}
		if (this.strategy === 'defender') {
			// Keep our two Toggles our colour; otherwise stand in front of the one worth more and block.
			const own = toggles.filter((t) => t.quad.startsWith(mine));
			const bad = own.find((t) => t.state !== mine && !ai.ban.has(t));
			if (bad) { this.flipToggle(bad, mine, dt); return; }
			this.guard(own, others, dt);
			return;
		}
		if (this.strategy === 'pusher') {
			const opp = this.opponent(others);
			if (opp) { this.push(opp); return; }
			this.park(p); return;
		}
		this.scoreCycle(dt, others, p);
	}

	// Yellow pin halves already placed in a quadrant (what its Toggle is worth).
	yellowIn(quad) {
		let n = 0;
		for (const pin of this.field.pins) {
			if (pin.held) continue;
			const t = toField(pin.body.translation()), g = this.field.goals.find((g) => Math.hypot(t.fx - g.fx, t.fy - g.fy) < 3 && t.h > g.base);
			if (g && g.quad === quad) n += (pin.top === 'yellow') + (pin.bottom === 'yellow');
		}
		return n;
	}

	// Turn a Toggle until it shows `color`: line up square in front of it, ram it, back off, ram again
	// (one face per hit, see Field.stepToggles). Done when it has settled on the colour.
	flipToggle(tg, color, dt) {
		const ai = this.ai, n = { x: tg.n[0], y: tg.n[1] }, hl = CFG.length / 2;
		if (ai.tgt !== tg) { ai.tgt = tg; ai.tph = 'approach'; ai.tt = 0; }
		ai.tt += dt;
		if (this.claw) this.releaseClaw(true);   // both hands free to ram
		this.intakeCmd = 0; this.clawSpin = 0; this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.liftTarget = 0;
		const face = Math.atan2(-n.y, -n.x), p = this.fpos();
		const off = (p.fx - tg.fx) * n.x + (p.fy - tg.fy) * n.y;   // distance out from the wall
		if (ai.tph === 'approach') {
			const stand = { fx: tg.fx + n.x * (hl + 12), fy: tg.fy + n.y * (hl + 12) };
			if (this.driveTo(stand.fx, stand.fy, 0.85, false, null) > 3) return;
			ai.aligning = true;
			if (this.turnTo(face) < 0.07) { ai.tph = 'ram'; ai.tt = 0; ai.aligning = false; }
			return;
		}
		if (ai.tph === 'ram') {
			if (off > hl + 22) { ai.tph = 'approach'; return; }   // pushed away / not lined up: start over
			ai.pressing = true;
			this.arcade(1, this.steer(wrap(face - this.heading())) * 0.6);
			if (tg.flipped || ai.tt > 1.8) { ai.tph = 'back'; ai.tt = 0; }
			return;
		}
		if (ai.tph === 'back') {
			ai.pressing = true;
			this.arcade(-0.8, 0);
			if (off > hl + 9 || ai.tt > 0.9) { ai.tph = 'settle'; ai.tt = 0; }
			return;
		}
		// settle: wait for it to stop turning, then hit again if it isn't our colour yet.
		ai.pressing = true;
		this.arcade(0, this.steer(wrap(face - this.heading())) * 0.6);
		if (tg.set || ai.tt > 0.8) { ai.tph = tg.state === color || off > hl + 16 ? 'approach' : 'ram'; ai.tt = 0; if (tg.state === color) ai.tgt = null; }
	}

	// Stand ~20" out in front of the Toggle worth the most; push away an opponent that comes near it.
	guard(list, others, dt) {
		const ai = this.ai, hl = CFG.length / 2;
		this.intakeCmd = 0; this.clawSpin = 0; this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.liftTarget = 0;
		if (!list.length) { this.park(this.fpos()); return; }
		const tg = list.slice().sort((a, b) => this.yellowIn(b.quad) - this.yellowIn(a.quad))[0];
		const threat = others.filter((r) => r.alliance !== this.alliance && r.mesh.visible)
			.map((r) => ({ r, d: Math.hypot(toField(r.pos()).fx - tg.fx, toField(r.pos()).fy - tg.fy) }))
			.filter((o) => o.d < 40).sort((a, b) => a.d - b.d)[0];
		if (threat) { this.push(threat.r); return; }
		const stand = { fx: tg.fx + tg.n[0] * (hl + 14), fy: tg.fy + tg.n[1] * (hl + 14) };
		// A partner already sitting there (e.g. lined up on the Toggle): stand beside it instead.
		if (others.some((r) => r !== this && r.alliance === this.alliance && Math.hypot(toField(r.pos()).fx - stand.fx, toField(r.pos()).fy - stand.fy) < 16)) {
			stand.fx += -tg.n[1] * 20; stand.fy += tg.n[0] * 20;
		}
		if (this.driveTo(stand.fx, stand.fy, 0.8, false, null) < 3) {
			ai.aligning = true;
			if (this.turnTo(Math.atan2(tg.n[1], tg.n[0])) < 0.1) this.arcade(0, 0);   // facing out, ready to meet them
		} else ai.aligning = false;
		ai.stuckT = 0;
	}

	// Nearest opponent, preferring one that's carrying a piece.
	opponent(others) {
		const p = this.fpos();
		return others.filter((r) => r.alliance !== this.alliance && r.mesh.visible)
			.map((r) => { const q = toField(r.pos()); return { r, c: Math.hypot(q.fx - p.fx, q.fy - p.fy) - (r.claw ? 30 : 0) }; })
			.sort((a, b) => a.c - b.c)[0]?.r || null;
	}

	// Drive straight into a robot at full power (stays out of the opponents' Load Zones).
	push(r) {
		const q = toField(r.pos());
		this.intakeCmd = 0; this.clawSpin = 0; this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.liftTarget = 0;
		if (Math.abs(q.fx) > 56 && Math.abs(q.fy) > 44) { this.park(this.fpos()); return; }   // Load Zones are protected
		const d = this.driveTo(q.fx, q.fy, 1, false, r);
		if (d < 22) this.ai.pressing = true;
	}

	// Scorer and stacker: fetch a piece, carry it to a goal, drop it on.
	scoreCycle(dt, others, p) {
		const ai = this.ai, field = this.field, stack = this.strategy === 'stacker';
		if (this.claw && ai.state === 'fetch') { this.setState('score'); ai.target = null; }
		if (ai.state !== 'fetch' && ai.state !== 'score' && ai.state !== 'leave') ai.state = this.claw ? 'score' : 'fetch';

		if (ai.state === 'fetch') {
			this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.liftTarget = 0;
			this.intakeCmd = 127; this.clawSpin = this.hopper ? -127 : 0;
			if (this.hopper) { this.arcade(0, 0); return; }   // wait for the hand-off to the claw
			const valid = (o) => o && !o.held && (field.pins.includes(o) || field.cups.includes(o)) && !this.isScored(o);
			if (ai.target && ai.t > 7) { ai.ban.set(ai.target, this.t + 15); ai.target = null; }   // can't get it: try another
			if (!valid(ai.target)) ai.t = 0;
			// Stacker: a cup whenever one of its goals has a pin on top, otherwise a yellow pin.
			if (!valid(ai.target)) ai.target = stack
				? (this.pickGoal('cup') ? this.pickCup(others) : this.pickPin(others)) || (this.pickGoal('pin') ? this.pickPin(others) : null)
				: this.pickGoal('pin') ? this.pickPin(others) : this.pickGoal('cup') ? this.pickCup(others) : null;
			if (!ai.target) { this.park(p); return; }
			const t = toField(ai.target.body.translation());
			this.driveTo(t.fx, t.fy, 0.75);
			// Cups: grabbed straight into the claw when they're in front of the intake.
			if (field.cups.includes(ai.target) && !this.claw) {
				const l = this.toLocal(new THREE.Vector3().copy(ai.target.body.translation()));
				if (l.x > CFG.length / 2 - 3 && l.x < CFG.length / 2 + 5 && Math.abs(l.z) < 5 && l.y < 9) this.attachPin(ai.target, 'claw');
			}
		} else if (ai.state === 'score') {
			this.intakeCmd = 0; this.clawSpin = 0;
			const kind = field.cups.includes(this.claw) ? 'cup' : 'pin';
			if (!ai.target || !field.goals.includes(ai.target) || this.goalTop(ai.target, this.claw, kind) == null) ai.target = this.pickGoal(kind);
			const g = ai.target;
			if (!g) {
				// Every goal has a pin on top: put this one down and go stack a cup first.
				if (kind === 'pin' && this.pickGoal('cup')) { const pin = this.claw; this.releaseClaw(true); ai.ban.set(pin, this.t + 12); this.setState('fetch'); ai.target = null; }
				else this.park(p);
				return;
			}
			// Stand CLAW_REACH in front of the goal, back to it, so the claw hangs the pin over the post.
			const u = new THREE.Vector2(p.fx - g.fx, p.fy - g.fy); if (u.lengthSq() < 1) u.set(1, 0); u.normalize();
			const P = { fx: clamp(g.fx + u.x * CLAW_REACH, -62, 62), fy: clamp(g.fy + u.y * CLAW_REACH, -62, 62) };
			const d = Math.hypot(P.fx - p.fx, P.fy - p.fy);
			const near = d < 14;
			this.clawTarget = near ? CFG.CLAW_TURN_HORIZONTAL_DEG : CFG.CLAW_TURN_VERTICAL_DEG;
			this.liftTarget = near ? LIFT_FOR[g.type] : 0;
			if (d > 2.5 && !ai.aligning) { this.driveTo(P.fx, P.fy, d < 14 ? 0.45 : 0.75, d < 8 && Math.cos(wrap(Math.atan2(P.fy - p.fy, P.fx - p.fx) - this.heading())) < 0); return; }
			if (!ai.aligning) ai.alignT = 0;
			ai.aligning = true; ai.alignT += dt;
			const facing = Math.atan2(P.fy - g.fy, P.fx - g.fx);
			if (d > 5) { ai.aligning = false; return; }
			if (this.turnTo(facing) > 0.08) return;
			this.arcade(0, 0);
			const pin = toField(this.claw.body.translation());
			if (Math.abs(this.clawDeg - CFG.CLAW_TURN_HORIZONTAL_DEG) < 20 && Math.abs(this.liftDeg - this.liftTarget) < 30 && Math.hypot(pin.fx - g.fx, pin.fy - g.fy) < 3.5) {
				ai.aligning = false;
				if (this.dropOn(g)) this.setState('leave');
			} else if (ai.alignT > 4) { ai.ban.set(g, this.t + 10); ai.target = null; ai.aligning = false; }
		} else if (ai.state === 'leave') {
			this.arcade(0.5, 0); this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.liftTarget = 0;
			if (ai.t > 0.7) { this.setState('fetch'); ai.target = null; }
		}
	}

	// Nothing to score: sit in our half of the Midfield (8 points while touching it).
	park(p) {
		this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.liftTarget = 0;
		const s = this.alliance === 'red' ? -1 : 1, slot = this.label.endsWith('1') ? 1 : -1;
		if (this.driveTo(s * 12, slot * 8, 0.6) < 3) this.arcade(0, 0);
		this.ai.stuckT = 0;
	}

	stop() { this.leftCmd = this.rightCmd = this.intakeCmd = this.clawSpin = 0; this.liftTarget = this.liftDeg; }

	isScored(pin) {
		const t = toField(pin.body.translation());
		return this.field.goals.some((g) => Math.hypot(t.fx - g.fx, t.fy - g.fy) < 4);
	}

	// Most useful free pin nearby: own-colour halves are worth the most, yellow less, never the other colour.
	pickPin(others) {
		const p = this.fpos(), mine = this.alliance;
		const taken = new Set(others.filter((r) => r !== this && r.ai).map((r) => r.ai.target));
		let best = null, bestCost = Infinity;
		for (const pin of this.field.pins) {
			if (pin.held || taken.has(pin) || this.ai.ban.has(pin)) continue;
			// Stacker wants yellow (worth 10 a half on a goal we own); everyone else their own colour.
			const value = this.strategy === 'stacker'
				? [pin.top, pin.bottom].reduce((a, c) => a + (c === 'yellow' ? 1 : c === mine ? 0.3 : -1), 0) - (pin.top !== 'yellow' && pin.bottom !== 'yellow' ? 9 : 0)
				: [pin.top, pin.bottom].reduce((a, c) => a + (c === mine ? 1 : c === 'yellow' ? 0.4 : -1), 0);
			if (value <= 0) continue;
			const t = toField(pin.body.translation());
			if (Math.abs(t.fx) > 64 || Math.abs(t.fy) > 64 || this.isScored(pin)) continue;
			const cost = Math.hypot(t.fx - p.fx, t.fy - p.fy) / 24 - value;
			if (cost < bestCost) { bestCost = cost; best = pin; }
		}
		return best;
	}

	// What's stacked on a goal: the height a new pin would sit on, or null if a pin is on top
	// (a pin can only go on an empty goal or into a cup, <SC3>).
	// kind 'cup': where a cup would sit (only on top of a pin), null if it can't go there.
	goalTop(g, skip, kind = 'pin') {
		let top = { h: -1, surface: g.H, cupSeat: null };
		for (const o of [...this.field.pins, ...this.field.cups]) {
			if (o === skip || o.held) continue;
			const t = toField(o.body.translation());
			const r = o.body.rotation(), upright = 1 - 2 * (r.x * r.x + r.z * r.z) > 0.85;
			if (upright && Math.hypot(t.fx - g.fx, t.fy - g.fy) < 2.2 && t.h > g.base && t.h > top.h) {
				const cup = this.field.cups.includes(o);
				top = { h: t.h, surface: cup ? t.h + CUP_H / 2 : null, cupSeat: cup ? null : t.h - PIN_H / 2 + PIN_COLLAR };
			}
		}
		const tall = this.strategy === 'stacker' ? 14 : 0;   // the stacker builds them taller
		if (kind === 'cup') return top.cupSeat != null && top.cupSeat < 14 + tall ? top.cupSeat : null;
		return top.surface != null && top.surface < 16 + tall ? top.surface : null;   // keep stacks short enough to stay up
	}

	// Pieces already on a goal.
	stackSize(g) {
		return [...this.field.pins, ...this.field.cups].filter((o) => { if (o.held) return false; const t = toField(o.body.translation()); return Math.hypot(t.fx - g.fx, t.fy - g.fy) < 2.5 && t.h > g.base; }).length;
	}

	// Nearest free cup that isn't already part of a stack.
	pickCup(others) {
		const p = this.fpos(), taken = new Set(others.filter((r) => r !== this && r.ai).map((r) => r.ai.target));
		let best = null, bestD = Infinity;
		for (const c of this.field.cups) {
			if (c.held || taken.has(c) || this.ai.ban.has(c) || this.isScored(c)) continue;
			const t = toField(c.body.translation());
			if (Math.abs(t.fx) > 64 || Math.abs(t.fy) > 64 || t.h > 8) continue;
			const d = Math.hypot(t.fx - p.fx, t.fy - p.fy);
			if (d < bestD) { bestD = d; best = c; }
		}
		return best;
	}

	// Nearest goal this alliance can use (its own alliance goals or any neutral one) that has room.
	pickGoal(kind = 'pin') {
		const p = this.fpos();
		let best = null, bestD = Infinity;
		const stack = this.strategy === 'stacker', owner = Object.fromEntries(this.field.toggles.map((t) => [t.quad, t.state]));
		for (const g of this.field.goals) {
			if (g.type === 'tall' || (g.color !== 'neutral' && g.color !== this.alliance) || this.ai.ban.has(g) || this.goalTop(g, this.claw, kind) == null) continue;
			// Stacker: only goals whose yellow pins score for us (our quadrant's Toggle is our colour,
			// or our own alliance goals), and keeps building the stack it already started.
			if (stack && g.color !== this.alliance && owner[g.quad] !== this.alliance) continue;
			const d = Math.hypot(g.fx - p.fx, g.fy - p.fy) - (stack ? 25 * this.stackSize(g) : 0);
			if (d < bestD) { bestD = d; best = g; }
		}
		return best;
	}

	// Line the pin up over the post and let go (stands in for the driver's fine adjustment).
	dropOn(g) {
		const pin = this.claw, cup = this.field.cups.includes(pin), surface = this.goalTop(g, pin, cup ? 'cup' : 'pin');
		if (surface == null) { this.ai.target = null; return false; }
		// Pin: collar lands on the rim. Cup: slides over the pin's top and rests on its collar.
		const pos = W(g.fx, g.fy, cup ? surface + 0.3 + CUP_H / 2 : surface - PIN_COLLAR + 0.3 + PIN_H / 2);
		pin.body.setTranslation({ x: pos.x, y: pos.y, z: pos.z }, true);
		pin.body.setRotation({ x: 0, y: 0, z: 0, w: 1 }, true);
		this.releaseClaw(false, false);
		this.field.assistDrop(pin);   // guided down onto the post (same goal-top funnel as the driver's drops)
		return true;
	}
}
