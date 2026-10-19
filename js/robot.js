// The Titanium robot: drivetrain physics, cascade lift, the flex-wheel claw (10/19: two stacks of
// flex wheels on vertical shafts that grab a cup with a pin in it, no intake, no claw flip), and a
// port of src/opcontrol.cpp. Robot-local frame (inches): +x = motor forward, +y up, +z right.
// The claw is on the back (-x); the driver drives claw first (opcontrol starts with inverse = -1).
import * as THREE from 'three';
import { IN, W, PIN_H, CUP_H, MID, GROUP, groups, toField, LOADER } from './field.js';
import { buildRobotMesh } from './robotmodel.js';

// ---- Constants mirrored from the real code (include/mechanisms.h, robot-config.cpp) ----
export const CFG = {
	// Size: PLACEHOLDER until measured on the robot.
	length: 17, width: 15, height: 44.4 / 2.54,   // team: robot top is 45 cm with the lift all the way down (frame sits 0.6 cm above H)
	trackWidth: 11.02, wheelDia: 3.25, mass: 6.5 /* kg */,
	driveRpm: 384,                      // wheel rpm (robot-config.cpp 10/15: 600 rpm motors geared down; a 24" push read 37.5")
	motorRpm: 600,                      // blue cartridge: the motor encoders count motor turns
	motorsPerSide: 2,                   // left {1,4}, right {-2,-3}
	motorStallNm: 2.1,                  // V5 11W stall torque with the 100 rpm cartridge (VEX spec)
	driveEff: 0.8,                      // gear/bearing losses
	rollingLoss: 0.04,                  // rolling resistance as a fraction of weight
	coastDrag: 1.5,                     // coasting motors still drag (gearbox + cartridge friction), as a fraction of brake
	yawScrub: 4.0,                      // N*m per rad/s resisting spin when not turning
	turnScrub: 0.5,                     // N*m per rad/s omni scrub while turning (tops out ~300 deg/s)
	sideGrip: 1.0,                      // how hard the wheels resist sliding sideways (1 = full tile grip)
	mu: 1.0,                            // omni wheel grip on foam tiles
	// Claw (10/19): stack centre sits this far in from the back edge, between the two roller stacks.
	clawIn: 2.6, clawWheelR: 1.25,
	CLAW_GRAB_SPEED: -127, CLAW_RELEASE_SPEED: 85,   // opcontrol.cpp: R1 grab spin, R2 / A release spin
	// The old flip claw + intake, still used by the AI robots (ai.js).
	CLAW_TURN_VERTICAL_DEG: 25, CLAW_TURN_HORIZONTAL_DEG: 900, CLAW_LIFT_FRACTION: 0.25,
	CASCADE_INTAKE_DEG: 180,            // R1 sends the cascade here to intake (claw side), at velocity 45
	CLAW_TURN_VELOCITY: 200, CLAW_TURN_DOWN_VELOCITY: 100,
	CLAW_DROP_SPEED: 127, CLAW_DROP_MS: 700,
	clawDegPerMotorDeg: 0.1,            // 900 motor deg = 90 deg of arm swing (assumed)
	clawArm: 9,                         // pivot -> grip, inches (from photos)
	pinBottomAtHorizontal: 25 / 2.54 - 5.5,   // team (2026-10-15): claw lowered, pivot 25 cm off the floor with the lift down
	// mechanisms.h: lift rotation sensor reads 10 (down) to -1444 (up); claw scoring point 107 cm up at the top.
	liftMaxIn: 42.1,
	liftInPerDeg: 42.1 / 1454,
	liftDegPerSec: 1200,                // 3 green motors at full command (assumed)
	LIFT_ALLIANCE_GOAL_DEG: 400, LIFT_SHORT_GOAL_DEG: 700, LIFT_TALL_GOAL_DEG: 1100, LIFT_VELOCITY: 200,
};
CFG.CLAW_TURN_LIFT_DEG = 2 * CFG.CLAW_TURN_HORIZONTAL_DEG * CFG.CLAW_LIFT_FRACTION;     // 450: slanted (claw_up / claw_lift_off)
CFG.CLAW_TURN_HIGHEST_DEG = CFG.CLAW_TURN_HORIZONTAL_DEG + CFG.CLAW_TURN_LIFT_DEG;      // 1350: past horizontal, up high
CFG.liftMaxDeg = CFG.liftMaxIn / CFG.liftInPerDeg;
CFG.clawX = -CFG.length / 2 + CFG.clawIn;   // robot-local x of a held stack's centre

const DEG = Math.PI / 180;
// All colliders of a pin or cup.
const colsOf = (o) => (o.collider ? [o.collider, ...o.extra] : Array.from({ length: o.body.numColliders() }, (_, i) => o.body.collider(i)));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const upright = (o) => { const r = o.body.rotation(); return Math.abs(1 - 2 * (r.x * r.x + r.z * r.z)) > 0.85; };

export class Robot {
	constructor(R, world, scene, field) {
		this.R = R; this.world = world; this.scene = scene; this.field = field;
		this.alliance = 'red';
		this.build();
		this.resetState();
	}

	build() {
		const { R, world } = this;
		const L = CFG.length, Wd = CFG.width, H = CFG.height;
		this.body = world.createRigidBody(R.RigidBodyDesc.dynamic().setCanSleep(false).setLinearDamping(0.1).setAngularDamping(0.1));
		this.body.setEnabledRotations(false, true, false, true);
		this.body.setEnabledTranslations(true, false, true, true);   // stays on the floor: can't be pushed under it or lifted
		const box = (x0, x1, y0, y1, z0, z1, density = 800) => {
			const d = R.ColliderDesc.cuboid(((x1 - x0) / 2) * IN, ((y1 - y0) / 2) * IN, ((z1 - z0) / 2) * IN)
				.setTranslation(((x0 + x1) / 2) * IN, ((y0 + y1) / 2) * IN, ((z0 + z1) / 2) * IN)
				.setFriction(0).setFrictionCombineRule(R.CoefficientCombineRule.Min).setDensity(density)   // slides along goals/walls/robots instead of sticking
				.setCollisionGroups(groups(GROUP.robot, 0xffff));
			return world.createCollider(d, this.body);
		};
		const hl = L / 2, hw = Wd / 2;
		// U-shaped base, open at the back so a cup + pin stack drives in between the claw rollers;
		// closed front with the crossbar that hits the Toggle; corner towers.
		const inner = CFG.clawX + 1.575 + 0.6;   // just behind a held stack
		this.colliders = [
			box(-hl, hl, 0, 4, -hw, -hw + 2), box(-hl, hl, 0, 4, hw - 2, hw),
			box(hl - 1.5, hl, 0, 4, -hw + 2, hw - 2),
			box(hl - 2, hl, 4, H, -hw, -hw + 1.5), box(hl - 2, hl, 4, H, hw - 1.5, hw),
			box(-hl, -hl + 2, 4, H, -hw, -hw + 1.5), box(-hl, -hl + 2, 4, H, hw - 1.5, hw),
			box(hl - 1.5, hl, H - 5, H, -hw, hw),
			// The inside of the robot (motors, Brain, battery, cascade) up to the back of the claw.
			// Solid so goals and other robots can't drive into it. Massless: the robot's weight is already set.
			box(inner, hl - 1.5, 1.2, 10, -hw + 2, hw - 2, 0),
			// Side panels (towers, motors, cascade): nothing pokes into the robot above the rails.
			box(-hl + 2, hl - 2, 4, 12, -hw, -hw + 1.5, 0), box(-hl + 2, hl - 2, 4, 12, hw - 1.5, hw, 0),
		];
		this.mesh = this.buildMesh();
		this.scene.add(this.mesh);
	}

	buildMesh() {
		const m = buildRobotMesh(CFG);
		Object.assign(this, { wheels: m.wheels, clawRollers: m.clawRollers, stages: m.stages, driveGears: m.driveGears, liftGears: m.liftGears, liftLeds: m.liftLeds });
		return m.group;
	}

	resetState() {
		// Motors / mechanisms
		this.leftCmd = 0; this.rightCmd = 0; this.brake = 'coast';
		this.liftDeg = 0; this.liftMode = 'volt'; this.liftCmd = 0; this.liftTarget = 0; this.liftVel = CFG.LIFT_VELOCITY;
		this.clawSpin = 0; this.clawToggle = null;
		// Sensors
		this.leftPos = 0; this.rightPos = 0; this.imuDeg = 0; this.t = 0;
		// Pieces. hopper stays null on this robot (no intake); the AI robots still use it.
		this.hopper = null; this.claw = null; this.clawExtraPiece = null; this.clawRel = null; this.clawHold = null;
		this.releaseT = 0; this.releaseCool = 0; this.transferT = 0;
		// opcontrol state. inverse starts at -1 like opcontrol.cpp: stick up drives claw first.
		this.op = { inverse: -1, inverseScale: -1, clawIn: false, clawOut: false, prev: {} };
	}

	// Place at a pose: field inches, heading in degrees (0 = facing +fx, CCW positive).
	place(fx, fy, headingDeg) {
		const p = W(fx, fy, 0.02);
		const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), headingDeg * DEG);
		this.body.setTranslation({ x: p.x, y: p.y, z: p.z }, true);
		this.body.setRotation(q, true);
		this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
		this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
		for (const pin of [this.hopper, this.clawExtraPiece, this.claw]) if (pin && (this.field.pins.includes(pin) || this.field.cups.includes(pin))) this.releasePin(pin, false);
		this.resetState();
	}

	// A pin sitting in a cup, or a cup sitting on a pin: grabbing one takes the other along (the claw
	// holds the pair, e.g. pin + cup off the field together). Kept in place relative to the grabbed piece.
	grabPartner(o) {
		const isCup = this.field.cups.includes(o), to = o.body.translation(), ro = o.body.rotation();
		const qo = new THREE.Quaternion(ro.x, ro.y, ro.z, ro.w);
		let best = null, bd = Infinity;
		for (const k of isCup ? this.field.pins : this.field.cups) {
			if (k.held) continue;
			const t = k.body.translation(), r = k.body.rotation();
			const upright = Math.abs(1 - 2 * (r.x * r.x + r.z * r.z)) > 0.8;
			const flat = Math.hypot(t.x - to.x, t.z - to.z) / IN, dy = Math.abs(t.y - to.y) / IN;
			if (upright && flat < 1.3 && dy > 0.5 && dy < 5.5 && dy < bd) { bd = dy; best = k; }
		}
		if (!best) return;
		const t = best.body.translation(), r = best.body.rotation(), inv = qo.clone().invert();
		this.clawRel = {
			pos: new THREE.Vector3(t.x - to.x, t.y - to.y, t.z - to.z).applyQuaternion(inv),
			quat: inv.multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)),
		};
		this.attachPin(best, 'clawExtraPiece');
	}

	// The lower of the pieces in the claw (what has to fit the goal when placing).
	clawBottom() {
		const e = this.clawExtraPiece;
		return e && e.body.translation().y < this.claw.body.translation().y ? e : this.claw;
	}

	// Let go of everything in the claw. assist: line it up on a goal it's over (see Field.assistDrop),
	// moving a held pair together.
	releaseClaw(keepVel = true, assist = false) {
		const main = this.claw, extra = this.clawExtraPiece;
		if (!main) return;
		const bottom = this.clawBottom(), top = bottom === main ? extra : main;
		const off = top ? new THREE.Vector3().copy(top.body.translation()).sub(bottom.body.translation()) : null;
		if (extra) this.releasePin(extra, keepVel);
		this.releasePin(main, keepVel);
		this.clawRel = null;
		if (!assist || !this.field.assistDrop(bottom) || !top) return;
		if (top.kind === 'pin' && bottom.kind === 'pin') return;   // two pins side by side: both let go, the lower one seats
		const b = bottom.body.translation(), h = off.length();   // stack straight up over the seated bottom piece
		top.body.setTranslation({ x: b.x, y: b.y + h, z: b.z }, true);
		top.body.setRotation(bottom.body.rotation(), true);
		top.body.setLinvel({ x: 0, y: 0, z: 0 }, true); top.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
		top.funnel = { ...bottom.funnel };
	}

	// No claw frame to hit the Toggle any more: only the robot body flips it.
	// The Loader tubes are solid (field.js), but the robot's back is open between the corner towers:
	// stop the back edge at the front of a tube instead of letting the tube into the robot.
	holdAtToggle() {
		this.toggleHitV = 0;
		const hl = CFG.length / 2, hw = CFG.width / 2, tw = LOADER.inner / 2 + 0.12;
		for (const L of this.field.loaders || []) {
			const l = this.toLocal(W(L.px, L.fy, LOADER.base));
			if (Math.abs(l.z) > hw + tw || l.x > -hl + 3 || l.x < -hl - tw - 0.3) continue;
			const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(this.quat()), v = this.body.linvel();
			const vx = v.x * fwd.x + v.z * fwd.z;
			if (vx < 0) this.body.setLinvel({ x: v.x - vx * fwd.x, y: 0, z: v.z - vx * fwd.z }, true);
		}
	}

	// The Loader just behind the robot's back, if any: { L, l (tube centre at its base, robot-local
	// inches), ok }. ok = it can be taken now: a stack waiting, robot square to the wall (within
	// 10 deg), the stack centred on the claw (within 1.2"), the lift at the base and the back
	// within the rollers' reach (2.5"). Otherwise the robot just bumps the tube.
	loaderReach() {
		const hl = CFG.length / 2, hw = CFG.width / 2;
		for (const L of this.field.loaders || []) {
			const l = this.toLocal(W(L.px, L.fy, LOADER.base));
			if (Math.abs(l.z) > hw + 2.2 || l.x > -hl + 0.5 || l.x < -hl - 6) continue;
			const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(this.quat());
			const e = this.liftIn() - LOADER.base;
			const ok = L.ready && fwd.x * L.n > Math.cos(10 * DEG) && Math.abs(l.z) < 1.2 && e > -3 && e < 1.5 && l.x > -hl - 2.5;
			return { L, l, ok };
		}
		return null;
	}

	// Safety net against solver kicks (e.g. the turning Toggle shoving the claw): the robot can't
	// gain speed faster than its wheels could push it (~1 g), and never past 2.6 m/s / 12 rad/s.
	// Slowing down (hitting things) is never limited.
	clampVel() {
		// Last line of defence: if the solver ever produces NaN, put the robot back where it last was.
		const tr = this.body.translation(), lv = this.body.linvel(), av = this.body.angvel();
		if ([tr.x, tr.z, lv.x, lv.z, av.y].some((v) => !Number.isFinite(v))) {
			if (this.lastGood) { this.body.setTranslation(this.lastGood.t, true); this.body.setRotation(this.lastGood.r, true); }
			this.body.setLinvel({ x: 0, y: 0, z: 0 }, true); this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
			if (this.clawDeg !== undefined && !Number.isFinite(this.clawDeg)) this.clawDeg = this.clawTarget || 0;
			if (!Number.isFinite(this.liftDeg)) this.liftDeg = 0;
			return;
		}
		this.lastGood = { t: { ...tr }, r: { ...this.body.rotation() } };
		const v = this.body.linvel(), sp = Math.hypot(v.x, v.z), prev = this.prevSp ?? sp;
		const cap = Math.min(2.6, prev + 9.8 / 120);
		if (sp > cap) this.body.setLinvel({ x: (v.x * cap) / sp, y: 0, z: (v.z * cap) / sp }, true);
		const w = this.body.angvel().y, pw = this.prevW ?? w, wcap = Math.min(12, Math.abs(pw) + 60 / 120);
		if (Math.abs(w) > wcap) this.body.setAngvel({ x: 0, y: Math.sign(w) * wcap, z: 0 }, true);
		this.prevSp = Math.min(sp, cap); this.prevW = Math.sign(w) * Math.min(Math.abs(w), wcap);
	}

	quat() { const r = this.body.rotation(); return new THREE.Quaternion(r.x, r.y, r.z, r.w); }

	pos() { const t = this.body.translation(); return new THREE.Vector3(t.x, t.y, t.z); }

	toWorld(x, y, z) { return new THREE.Vector3(x * IN, y * IN, z * IN).applyQuaternion(this.quat()).add(this.pos()); }

	toLocal(v) { return v.clone().sub(this.pos()).applyQuaternion(this.quat().invert()).divideScalar(IN); }

	// Robot footprint touches the Midfield?
	inMidfield() {
		const hl = CFG.length / 2, hw = CFG.width / 2;
		for (const [x, z] of [[hl, hw], [hl, -hw], [-hl, hw], [-hl, -hw], [0, 0]]) {
			const f = toField(this.toWorld(x, 0, z));
			if (Math.abs(f.fx) + Math.abs(f.fy) <= MID) return true;
		}
		return false;
	}

	liftIn() { return clamp(this.liftDeg * CFG.liftInPerDeg, 0, CFG.liftMaxIn); }

	attachPin(pin, slot) {
		pin.held = slot;
		for (const c of colsOf(pin)) c.setCollisionGroups(0);   // no contacts while held (disabling would zero the mass)
		pin.body.setBodyType(this.R.RigidBodyType.KinematicPositionBased, true);
		this[slot] = pin;
	}

	releasePin(pin, keepVel = true, pushFwd = 0) {
		const slot = pin.held;
		if (slot) this[slot] = null;
		pin.held = null;
		pin.flipped = false;
		pin.body.setBodyType(this.R.RigidBodyType.Dynamic, true);
		// Briefly ignore the claw frame so a piece let go from the grip can drop out of it.
		for (const c of colsOf(pin)) c.setCollisionGroups(groups(0xffff & ~GROUP.claw, 0xffff & ~GROUP.claw));
		(this.justReleased ||= []).push({ pin, t: 0.5 });
		const v = keepVel ? this.body.linvel() : { x: 0, y: 0, z: 0 };
		const f = new THREE.Vector3(1, 0, 0).applyQuaternion(this.quat()).multiplyScalar(pushFwd);
		pin.body.setLinvel({ x: v.x + f.x, y: v.y + f.y, z: v.z + f.z }, true);
		pin.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
	}

	// Start the match holding the preload upright in the claw.
	loadPreload() {
		const pin = this.field.makePin('yellow', 'yellow', this.toWorld(CFG.clawX, PIN_H / 2 + 0.3, 0));   // preload is yellow/yellow
		this.grab(pin);
	}

	// Take an upright piece into the claw (plus whatever is stacked with it), keeping it as it
	// stands; it's centred between the rollers and rides with the lift.
	grab(o) {
		const t = o.body.translation(), r = o.body.rotation();
		const l = this.toLocal(new THREE.Vector3(t.x, t.y, t.z));
		const half = (this.field.cups.includes(o) ? CUP_H : PIN_H) / 2;
		this.clawHold = {
			y: Math.max(l.y - this.liftIn(), half + 0.3),
			q: this.quat().invert().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)),
		};
		this.attachPin(o, 'claw');
		this.grabPartner(o);
	}

	// ---- Physics step ------------------------------------------------------------------
	step(dt) {
		this.t += dt;
		this.stepDrive(dt);
		this.stepMechanisms(dt);
		this.stepPins(dt);
		this.holdAtToggle();
	}

	// DC motor model (same approach as RobotPy / FRC drivetrain sims): each motor's torque
	// falls linearly from stall at 0 rpm to zero at free speed, scaled by the commanded
	// voltage. V5 11W motor: 2.1 N*m stall with the 100 rpm cartridge -> 0.35 N*m with blue
	// (600 rpm). Forces are capped by wheel traction; rolling/chain losses slow it a bit.
	stepDrive(dt) {
		const g = 9.81, tw = CFG.trackWidth * IN, r = (CFG.wheelDia / 2) * IN;
		const q = this.quat();
		const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(q), right = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
		const lv = this.body.linvel(), v = new THREE.Vector3(lv.x, 0, lv.z);
		const vf = v.dot(fwd), vl = v.dot(right), w = this.body.angvel().y;
		const m = this.body.mass(), I = (m * ((CFG.length * IN) ** 2 + (CFG.width * IN) ** 2)) / 12;
		const vfree = (CFG.driveRpm / 60) * 2 * Math.PI * r;
		const Fstall = (CFG.motorsPerSide * CFG.motorStallNm * (100 / CFG.driveRpm) * CFG.driveEff) / r;
		const grip = (CFG.mu * m * g) / 2;
		const side = (cmd, vw) => {
			const coast = cmd === 0 && this.brake === 'coast';
			let F = coast ? -Fstall * CFG.coastDrag * (vw / vfree) : Fstall * (clamp(cmd, -127, 127) / 127 - vw / vfree);   // brake = shorted motor
			const roll = (CFG.rollingLoss * m * g) / 2 * (coast ? 2 : 1);              // coasting also drags the chain
			F -= Math.sign(vw) * Math.min(roll, (Math.abs(vw) * m) / 2 / dt);         // never reverses the wheel
			return clamp(F, -grip, grip);
		};
		const FL = side(this.leftCmd, vf - (w * tw) / 2), FR = side(this.rightCmd, vf + (w * tw) / 2);
		this.driveForce = FL + FR;   // N, forward; the Toggle uses it to tell a shove from a bump
		const dvF = ((FL + FR) / m) * dt;
		// Omni rollers + chain scrub resist spinning; with no turn command the robot settles instead of
		// carrying on rotating (that coasting spin felt like drifting).
		const noTurn = Math.abs(this.leftCmd - this.rightCmd) < 8;
		const scrub = clamp(-w * (noTurn ? CFG.yawScrub : CFG.turnScrub), -Math.abs(w) * I / dt, Math.abs(w) * I / dt);
		const dW = ((((FR - FL) * tw) / 2 + scrub) / I) * dt;                          // CCW positive
		const dvL = clamp(-vl, -CFG.mu * g * dt, CFG.mu * g * dt);                     // omnis resist sliding sideways
		const imp = fwd.multiplyScalar(dvF * m).addScaledVector(right, dvL * m * CFG.sideGrip);
		this.body.applyImpulse({ x: imp.x, y: 0, z: imp.z }, true);
		// Set the spin directly with this inertia: Rapier's own (from the collider boxes) is ~3x bigger,
		// which made turns sluggish and overshoot. Collisions still change it inside world.step().
		this.body.setAngvel({ x: 0, y: w + dW, z: 0 }, true);
		// Encoders (deg) and IMU (deg, clockwise positive, unwrapped like get_rotation()).
		const circ = Math.PI * CFG.wheelDia * IN;
		const gear = CFG.motorRpm / CFG.driveRpm;   // motor turns per wheel turn
		this.leftPos += ((vf - (w * tw) / 2) / circ) * 360 * gear * dt;
		this.rightPos += ((vf + (w * tw) / 2) / circ) * 360 * gear * dt;
		this.imuDeg -= (w * dt) / DEG;
		this.wheelSpin = { L: (vf - (w * tw) / 2) / (1.625 * IN), R: (vf + (w * tw) / 2) / (1.625 * IN) };
	}

	stepMechanisms(dt) {
		const liftBefore = this.liftDeg;
		if (this.liftMode === 'volt') this.liftDeg += (this.liftCmd / 127) * CFG.liftDegPerSec * dt;
		else {
			const step = this.liftVel * 6 * dt, e = this.liftTarget - this.liftDeg;
			this.liftDeg += clamp(e, -step, step);
		}
		this.liftDeg = clamp(this.liftDeg, 0, CFG.liftMaxDeg);   // cascade_move() stops at both ends
		this.liftRate = (this.liftDeg - liftBefore) / dt;   // motor deg/s, for the spinning gears
	}

	stepPins(dt) {
		for (const r of this.justReleased || []) if ((r.t -= dt) <= 0 && !r.pin.held && (this.field.pins.includes(r.pin) || this.field.cups.includes(r.pin))) for (const c of colsOf(r.pin)) c.setCollisionGroups(0xffffffff);
		if (this.justReleased) this.justReleased = this.justReleased.filter((r) => r.t > 0);
		this.releaseCool = Math.max(0, this.releaseCool - dt);
		// Out of a Loader (see loaderReach): the rollers have to be spinning in to pull it out.
		if (!this.claw && this.clawSpin < -30 && this.releaseCool <= 0) {
			const r = this.loaderReach();
			if (r && r.ok) {
				const st = r.L.stack, o = this.field.takeLoader(r.L);
				this.grab(o);
				for (const k of st) if (!k.held) this.field.freePiece(k);
				this.op.clawOut = false;
			}
		}
		// Grab: an upright cup (with its pin) or pin whose bottom is at the rollers, driven in between
		// them. The claw doesn't have to be spinning: driving into the stack squeezes it in. With R1's
		// grab spin on, the rollers reach out and pull it in from the mouth. Not while spinning to let go.
		if (!this.claw && this.clawSpin <= 30 && this.releaseCool <= 0) {
			const spin = this.clawSpin < -30, lift = this.liftIn(), hl = CFG.length / 2;
			for (const o of [...this.field.pins, ...this.field.cups]) {
				if (o.held || !upright(o)) continue;
				const l = this.toLocal(new THREE.Vector3().copy(o.body.translation()));
				const bottom = l.y - (this.field.cups.includes(o) ? CUP_H : PIN_H) / 2;
				if (bottom < lift - 1.5 || bottom > lift + 3) continue;   // a pin sitting in a cup is too high: the cup is what's gripped
				if (l.x < (spin ? -hl - 2.5 : CFG.clawX - 1.2) || l.x > CFG.clawX + 1.5 || Math.abs(l.z) > (spin ? 3.5 : 2.5)) continue;
				this.grab(o);
				this.op.clawOut = false;   // optical sensor sees it: R1's grab spin stops by itself
				break;
			}
		}
		// Scoring by going down: holding a stack over a goal, lowering the lift until it seats lets go.
		// Only while actually lowering, and not right after picking it up.
		this.clawHeldT = this.claw && this.claw === this.clawHeldObj ? this.clawHeldT + dt : 0;
		this.clawHeldObj = this.claw;
		if (this.claw && this.clawHeldT > 0.5 && (this.liftRate || 0) < -1) {
			const o = this.clawBottom(), s = this.field.seatFor(o), h = o.body.translation().y / IN;
			const half = (this.field.cups.includes(o) ? CUP_H : PIN_H) / 2;
			if (s && h - half <= s.seatBottom + 0.4) {
				const both = [this.claw, this.clawExtraPiece].filter(Boolean);
				this.releaseClaw(false, true);
				for (const p of both) (this.justPlaced ||= []).push({ o: p, t: 2 });
				this.releaseCool = 1;
			}
		}
		if (this.justPlaced) this.justPlaced = this.justPlaced.filter((r) => (r.t -= dt) > 0);
		// R2 / A spin lets go of the stack (lined up on a goal it's over).
		if (this.claw && this.clawSpin > 30) {
			this.releaseT += dt;
			if (this.releaseT > 0.12) { this.releaseClaw(true, true); this.releaseT = 0; this.releaseCool = 0.8; }
		} else this.releaseT = 0;
		// Move the held stack.
		if (this.claw) {
			const h = this.clawHold, mq = this.quat().multiply(h.q), mp = this.toWorld(CFG.clawX, h.y + this.liftIn(), 0);
			this.claw.body.setNextKinematicTranslation(mp);
			this.claw.body.setNextKinematicRotation({ x: mq.x, y: mq.y, z: mq.z, w: mq.w });
			const e = this.clawExtraPiece, rel = this.clawRel;
			if (e && rel) {   // stacked partner (pin in the cup) rides along rigidly
				const p = rel.pos.clone().applyQuaternion(mq).add(mp), rq = mq.clone().multiply(rel.quat);
				e.body.setNextKinematicTranslation(p);
				e.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
			}
		}
	}

	syncMesh(dt) {
		const t = this.body.translation(), r = this.body.rotation();
		this.mesh.position.set(t.x, t.y, t.z);
		this.mesh.quaternion.set(r.x, r.y, r.z, r.w);
		for (const s of this.stages) s.g.position.y = this.liftIn() * s.frac * IN;
		const ws = this.wheelSpin || { L: 0, R: 0 };
		for (const w of this.wheels) w.w.rotation.y += (w.side === 'C' ? (ws.L + ws.R) / 2 * 1.6 : ws[w.side]) * dt;
		// Cascade motor gears spin with the lift; their LEDs light while driving.
		const lr = this.liftRate || 0;
		for (const lg of this.liftGears) lg.g.rotation.z += lg.dir * (lr * DEG) * 0.5 * dt;
		for (const led of this.liftLeds) led.material.color.setHex(Math.abs(lr) > 1 ? 0xff3322 : 0x331010);
		for (const dg of this.driveGears) dg.g.rotation.z -= (ws[dg.side] || 0) * dt;
		for (const rr of this.clawRollers) rr.g.rotation.y += rr.dir * (this.clawSpin / 127) * 25 * dt;   // the two stacks turn opposite ways
	}

	// ---- Driver control: port of src/opcontrol.cpp (one 20 ms loop per call) -----------
	// c = { ly, rx, L1, L2, R1, R2, A, B, X, Y, Up, Down, Left, Right }
	opcontrol(c, onRunAuton) {
		const o = this.op, newPress = (k) => c[k] && !o.prev[k];
		if (newPress('Down') && onRunAuton) onRunAuton();
		// 1. Drivetrain. Starts claw first (inverse = -1); D-pad Up flips it. Turning stays the same.
		// Ramped over ~160 ms.
		if (newPress('Up')) { o.inverse = -o.inverse; this.flash?.(o.inverse < 0 ? 'CLAW FIRST' : 'CLAW BACK'); }
		o.inverseScale = o.inverseScale < o.inverse ? Math.min(o.inverseScale + 0.25, o.inverse) : Math.max(o.inverseScale - 0.25, o.inverse);
		let dir = c.ly, turn = c.rx;
		if (Math.abs(dir) < 10) dir = 0;
		if (Math.abs(turn) < 10) turn = 0;
		dir = Math.round(dir * o.inverseScale);
		this.brake = 'coast';
		this.leftCmd = clamp(dir + turn, -127, 127);
		this.rightCmd = clamp(dir - turn, -127, 127);
		const estop = c.L1 && c.R1;
		// R1 toggles the grab spin; R2 or A toggles the release spin. Each cancels the other.
		if (newPress('R1') && !estop) { o.clawOut = !o.clawOut; o.clawIn = false; }
		if ((newPress('R2') || newPress('A')) && !estop) { o.clawIn = !o.clawIn; o.clawOut = false; }
		// Cascade: L1 up, L2 down.
		this.liftMode = 'volt';
		this.liftCmd = estop ? 0 : c.L1 ? 127 : c.L2 ? -85 : 0;
		this.clawSpin = o.clawIn ? CFG.CLAW_RELEASE_SPEED : o.clawOut ? CFG.CLAW_GRAB_SPEED : 0;
		o.prev = { ...c };
	}

	// After an auton test run from driver control (D-pad DOWN), reset like the real code does.
	afterAutonTest() {
		this.op.clawIn = false; this.op.clawOut = false;
	}
}
