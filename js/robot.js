// The Titanium robot (4787T): drivetrain (drivetrain.js), cascade lift, the flex-wheel claw on the
// back with the goal guide under it, and a port of src/opcontrol.cpp + src/mechanisms.cpp from
// will12liu/4787T-Titanium-Vex-Override. Robot-local frame (inches): +x = motor forward (the
// Toggle bumper end), +y up, +z right. The claw is on the back (-x); the driver drives claw first
// (opcontrol starts with drive_direction = -1).
import * as THREE from 'three';
import { IN, W, PIN_H, CUP_H, MID, GROUP, groups, toField, LOADER, TOGGLE } from './field.js';
import { buildRobotMesh } from './robotmodel.js';
import { Drivetrain } from './drivetrain.js';

// ---- Constants mirrored from the robot code (include/mechanisms.h, src/robot-config.cpp) ----
export const CFG = {
	// Size: PLACEHOLDER until measured on the robot.
	length: 17, width: 15, height: 44.4 / 2.54,   // team: robot top is 45 cm with the lift all the way down
	trackWidth: 11.02, wheelDia: 3.25, mass: 6.6 /* kg */,
	wheelbase: 12,                      // inches between the front and back axles
	driveRpm: 384,                      // robot-config.cpp: measured wheel rpm (600 rpm motors geared down)
	motorRpm: 600,                      // blue cartridge: the motor encoders count motor turns
	// Claw (10/19): stack centre sits this far in from the back edge, between the two roller stacks.
	clawIn: 2.6, clawWheelR: 1.25,
	CLAW_GRAB_SPEED: -127, CLAW_RELEASE_SPEED: 85,   // mechanisms.h
	// mechanisms.h: lift rotation sensor reads 0 (down, 10 = bottom limit) to -1444 (top limit).
	LIFT_LIMIT_BOTTOM_DEG: 10, LIFT_LIMIT_TOP_DEG: -1444, LIFT_LIMIT_SLOW_DEG: 80, LIFT_LIMIT_MIN_POWER: 30,
	LIFT_HOLD_BAND_DEG: 6, LIFT_RECORRECT_DEG: 14, LIFT_UP_MIN_POWER: 40, LIFT_DOWN_MIN_POWER: 30, LIFT_DOWN_MAX_POWER: 100,
	LIFT_SLOW_START_FRAC: 0.35, LIFT_SLOW_MIN_SCALE: 0.4,
	R1_LIFT_ROT: -70,                   // opcontrol.cpp: R1 also sends the cascade to -70 (intake height)
	liftMaxIn: 42.1,                    // claw scoring point 107 cm up at the top
	liftDegPerSec: 1250,                // 3 green motors at full power (assumed)
	// The grab: driving into a stack with R1 spinning pulls it in, but only slowly (team: too fast and it
	// knocks the stack over). Relative speed in m/s.
	grabMaxSpeed: 0.24,                 // ~9.5 in/s: under a fifth of full speed
	// Old flip claw + intake geometry, used by the AI robots' designs (ai.js).
	CLAW_TURN_VERTICAL_DEG: 25, CLAW_TURN_HORIZONTAL_DEG: 900, CLAW_TURN_DOWN_VELOCITY: 100,
	CASCADE_INTAKE_DEG: 180, clawDegPerMotorDeg: 0.1, clawArm: 9, pinBottomAtHorizontal: 25 / 2.54 - 5.5,
	LIFT_ALLIANCE_GOAL_DEG: 400, LIFT_SHORT_GOAL_DEG: 700, LIFT_TALL_GOAL_DEG: 1100, LIFT_VELOCITY: 200,
};
CFG.liftMaxDeg = -CFG.LIFT_LIMIT_TOP_DEG + 10;   // the cascade's hard stop, a little past the code's top limit
CFG.liftInPerDeg = CFG.liftMaxIn / -CFG.LIFT_LIMIT_TOP_DEG;
CFG.clawX = -CFG.length / 2 + CFG.clawIn;   // robot-local x of a held stack's centre
// Goal guide (from the team's photos): two arms under the claw, hinged on the inside of the drive
// rails, folded into a V that opens toward the back with rubber feet on the tips. Backing onto a goal,
// its base slides into the V and stops with the goal right under the claw. It also catches pins that
// get knocked over on the floor: one stuck in the V blocks the claw (and the Loader) until it's shaken out.
CFG.guide = { y0: 1.1, y1: 3.0, root: [CFG.clawX + 0.7, 2.2], tip: [-CFG.length / 2 - 0.35, 5.6], t: 0.5 };
// Toggle arms (front, from the photos): hinged blue arms with a black rubber puck under each tip. The
// pucks sit at the Toggle's axle height, above the 11.5" wall, and reach past the bumper, so they are
// what hits the Toggle (square on its face) and the bumper never touches the wall.
CFG.puck = { x: CFG.length / 2 + 1.7, z: CFG.width / 2 + 0.9, y: TOGGLE.axisH, r: 0.75, h: 1.0 };

const DEG = Math.PI / 180;
// Claw mode colours (light bar on the claw + HUD): R1 grab running, stopped (closed), R2 release running (open).
export const CLAW_LIGHT = { GRAB: 0x22e05a, STOPPED: 0xf2f4f6, RELEASE: 0xff9a1a };
const colsOf = (o) => (o.collider ? [o.collider, ...o.extra] : Array.from({ length: o.body.numColliders() }, (_, i) => o.body.collider(i)));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const upright = (o) => { const r = o.body.rotation(); return Math.abs(1 - 2 * (r.x * r.x + r.z * r.z)) > 0.85; };

export class Robot {
	constructor(R, world, scene, field) {
		this.R = R; this.world = world; this.scene = scene; this.field = field;
		this.alliance = 'red';
		this.build();
		this.drive = new Drivetrain({ ...CFG, cog: () => this.cogHeight() });
		this.resetState();
	}

	// ---- Body -------------------------------------------------------------------------------
	build() {
		const { R, world } = this;
		const L = CFG.length, Wd = CFG.width, H = CFG.height, hl = L / 2, hw = Wd / 2;
		const m = CFG.mass, I = (m * ((L * IN) ** 2 + (Wd * IN) ** 2)) / 12 * 0.85;
		this.body = world.createRigidBody(R.RigidBodyDesc.dynamic().setCanSleep(false).setLinearDamping(0).setAngularDamping(0)
			.setAdditionalMassProperties(m, { x: 0, y: 0.15, z: 0 }, { x: I, y: I, z: I }, { x: 0, y: 0, z: 0, w: 1 }));
		this.body.setEnabledRotations(false, true, false, true);
		this.body.setEnabledTranslations(true, false, true, true);   // stays on the floor: can't be pushed under it or lifted
		const box = (x0, x1, y0, y1, z0, z1, fr = 0.2) => {
			const d = R.ColliderDesc.cuboid(((x1 - x0) / 2) * IN, ((y1 - y0) / 2) * IN, ((z1 - z0) / 2) * IN)
				.setTranslation(((x0 + x1) / 2) * IN, ((y0 + y1) / 2) * IN, ((z0 + z1) / 2) * IN)
				.setFriction(fr).setFrictionCombineRule(R.CoefficientCombineRule.Min).setDensity(0)   // metal slides along goals / walls / robots
				.setCollisionGroups(groups(GROUP.robot, 0xffff));
			return world.createCollider(d, this.body);
		};
		// U-shaped base, open at the back so a cup + pin stack (or a goal) drives in under the claw;
		// closed front with the crossbar that hits the Toggle; corner towers.
		const inner = CFG.clawX + 2.9;   // in front of a goal seated under the claw
		this.colliders = [
			box(-hl, hl, 0.3, 4, -hw, -hw + 2), box(-hl, hl, 0.3, 4, hw - 2, hw),
			box(hl - 1.5, hl + 0.6, 0.3, 4, -hw + 2, hw - 2),
			box(hl - 2, hl, 4, H, -hw, -hw + 1.5), box(hl - 2, hl, 4, H, hw - 1.5, hw),
			box(-hl, -hl + 2, 4, H, -hw, -hw + 1.5), box(-hl, -hl + 2, 4, H, hw - 1.5, hw),
			box(hl - 1.5, hl, H - 5, H, -hw, hw),
			// The inside (motors, Brain, battery, cascade): solid so goals and robots can't drive into it.
			box(inner, hl - 1.5, 1.2, 11, -hw + 2, hw - 2),
			box(-hl + 2, hl - 2, 4, 12, -hw, -hw + 1.5), box(-hl + 2, hl - 2, 4, 12, hw - 1.5, hw),
		];
		// Goal guide: the two V arms + their rubber feet (fixed to the chassis, hit everything).
		const gd = CFG.guide;
		for (const s of [-1, 1]) {
			const [x0, z0] = gd.root, [x1, z1] = gd.tip;
			const len = Math.hypot(x1 - x0, z1 - z0), a = Math.atan2(s * (z1 - z0), x1 - x0);
			const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -a);
			const d = R.ColliderDesc.cuboid((len / 2) * IN, ((gd.y1 - gd.y0) / 2) * IN, (gd.t / 2) * IN)
				.setTranslation(((x0 + x1) / 2) * IN, ((gd.y0 + gd.y1) / 2) * IN, (s * (z0 + z1) / 2 + s * gd.t / 2) * IN)
				.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w })
				.setFriction(0.45).setDensity(0).setCollisionGroups(groups(GROUP.robot, 0xffff));
			this.colliders.push(world.createCollider(d, this.body));
			const foot = R.ColliderDesc.cylinder(1.4 * IN, 0.4 * IN).setTranslation(x1 * IN, 1.9 * IN, s * z1 * IN)
				.setFriction(0.9).setDensity(0).setCollisionGroups(groups(GROUP.robot, 0xffff));
			this.colliders.push(world.createCollider(foot, this.body));
		}
		// Toggle pucks: rubber, grippy.
		for (const s of [-1, 1]) {
			const pk = CFG.puck;
			this.colliders.push(world.createCollider(R.ColliderDesc.cylinder((pk.h / 2) * IN, pk.r * IN).setTranslation(pk.x * IN, pk.y * IN, s * pk.z * IN)
				.setFriction(0.9).setDensity(0).setCollisionGroups(groups(GROUP.robot, 0xffff)), this.body));
		}
		// The claw: two roller stacks + the brace behind the cup. They ride up with the lift
		// (moved every step) and only touch pins, cups and robots.
		const cg = groups(GROUP.claw, 0xffff & ~(GROUP.toggle | GROUP.wall | GROUP.fixed));
		const X = CFG.clawX, zs = 1.575 + CFG.clawWheelR - 0.3;
		this.clawCols = [];
		for (const s of [-1, 1]) {
			// Flex wheels squash: a stack centred in the mouth slides in between them, an off-centre one hits one.
			const c = world.createCollider(R.ColliderDesc.cylinder(2.3 * IN, (CFG.clawWheelR - 0.25) * IN).setFriction(0.8).setDensity(0).setCollisionGroups(cg), this.body);
			this.clawCols.push({ c, p: [X, 3.1, s * zs] });
		}
		const brace = world.createCollider(R.ColliderDesc.cuboid(0.25 * IN, 1.0 * IN, 3.2 * IN).setFriction(0.3).setDensity(0).setCollisionGroups(cg), this.body);
		this.clawCols.push({ c: brace, p: [X + 2.05, 3.4, 0] });
		this.mesh = this.buildMesh();
		this.scene.add(this.mesh);
	}

	buildMesh() {
		const m = buildRobotMesh(CFG);
		Object.assign(this, { parts: m });
		return m.group;
	}

	// CoG height (inches) for tipping: heavy base low, stages + claw ride up with the lift.
	cogHeight() { return 4.7 + 0.185 * this.liftIn() + (this.claw ? 0.02 * (this.liftIn() + 6) : 0); }

	resetState() {
		this.leftCmd = 0; this.rightCmd = 0; this.brake = 'coast';
		// Cascade: sensor degrees (0 down, -1444 up) like lift_rot; liftDeg = -sensor.
		this.liftDeg = 0; this.liftVel = 0; this.liftPower = 0; this.liftRate = 0;
		this.liftTask = { active: false, goal: 0, holding: false, t: 0 };
		this.liftMode = 'power';   // 'pos' = simple position mode (AI robots, old auton commands)
		this.liftTarget = 0; this.liftVelCmd = CFG.LIFT_VELOCITY;
		this.clawMode = 'STOPPED'; this.clawSpin = 0;
		this.leftPos = 0; this.rightPos = 0; this.imuDeg = 0; this.t = 0;
		this.hopper = null; this.claw = null; this.clawExtraPiece = null; this.clawRel = null; this.clawHold = null;
		this.pulling = null; this.releaseT = 0; this.releaseCool = 0; this.knocked = new Map();
		this.op = { inverse: -1, inverseScale: -1, preset: false, prev: {} };
		this.prevVel = null; this.toggleHitV = 0;
		this.drive?.reset();
		this.tippedNotified = false;
	}

	// Place at a pose: field inches, heading in degrees (0 = facing +fx, CCW positive).
	place(fx, fy, headingDeg) {
		const p = W(fx, fy, 0.02);
		const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), headingDeg * DEG);
		this.body.setTranslation({ x: p.x, y: p.y, z: p.z }, true);
		this.body.setRotation(q, true);
		this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
		this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
		for (const pin of [this.hopper, this.clawExtraPiece, this.claw]) if (pin && this.field.has(pin)) this.releasePin(pin, false);
		this.resetState();
		this.updateClawColliders();
	}

	quat() { const r = this.body.rotation(); return new THREE.Quaternion(r.x, r.y, r.z, r.w); }
	pos() { const t = this.body.translation(); return new THREE.Vector3(t.x, t.y, t.z); }
	toWorld(x, y, z) { return new THREE.Vector3(x * IN, y * IN, z * IN).applyQuaternion(this.quat()).add(this.pos()); }
	toLocal(v) { return v.clone().sub(this.pos()).applyQuaternion(this.quat().invert()).divideScalar(IN); }
	fwd() { return new THREE.Vector3(1, 0, 0).applyQuaternion(this.quat()); }
	liftIn() { return clamp(this.liftDeg * CFG.liftInPerDeg, 0, CFG.liftMaxIn + 0.5); }
	tipped() { return this.drive.tipped !== 0; }
	// How far the front reaches past the centre (inches): the Toggle pucks.
	frontReach() { return CFG.puck.x + CFG.puck.r; }

	// Robot footprint touches the Midfield?
	inMidfield() {
		const hl = CFG.length / 2, hw = CFG.width / 2;
		for (const [x, z] of [[hl, hw], [hl, -hw], [-hl, hw], [-hl, -hw], [0, 0]]) {
			const f = toField(this.toWorld(x, 0, z));
			if (Math.abs(f.fx) + Math.abs(f.fy) <= MID) return true;
		}
		return false;
	}

	// ---- Pieces in the claw ---------------------------------------------------------------------
	attachPin(pin, slot) {
		pin.held = slot;
		for (const c of colsOf(pin)) c.setCollisionGroups(0);   // no contacts while held (disabling would zero the mass)
		pin.body.setBodyType(this.R.RigidBodyType.KinematicPositionBased, true);
		this[slot] = pin;
	}

	releasePin(pin, keepVel = true, push = null) {
		const slot = pin.held;
		if (slot && this[slot] === pin) this[slot] = null;
		pin.held = null;
		pin.body.setBodyType(this.R.RigidBodyType.Dynamic, true);
		// Briefly ignore the claw so a piece let go from the grip can drop out of it.
		for (const c of colsOf(pin)) c.setCollisionGroups(groups(0xffff & ~GROUP.claw, 0xffff & ~GROUP.claw));
		(this.justReleased ||= []).push({ pin, t: 0.5 });
		const v = keepVel ? this.body.linvel() : { x: 0, y: 0, z: 0 };
		const f = push || { x: 0, y: 0, z: 0 };
		pin.body.setLinvel({ x: v.x + f.x, y: v.y + f.y, z: v.z + f.z }, true);
		pin.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
	}

	// A pin sitting in a cup, or a cup sitting on a pin: the claw takes the pair. Kept in place
	// relative to the grabbed piece.
	grabPartner(o) {
		const isCup = o.kind === 'cup', to = o.body.translation(), ro = o.body.rotation();
		const qo = new THREE.Quaternion(ro.x, ro.y, ro.z, ro.w);
		let best = null, bd = Infinity;
		for (const k of isCup ? this.field.pins : this.field.cups) {
			if (k.held) continue;
			const t = k.body.translation(), r = k.body.rotation();
			const up = Math.abs(1 - 2 * (r.x * r.x + r.z * r.z)) > 0.8;
			const flat = Math.hypot(t.x - to.x, t.z - to.z) / IN, dy = (t.y - to.y) / IN;
			if (up && flat < 1.3 && (isCup ? dy > 0.5 && dy < 5.5 : dy < -0.5 && dy > -5.5) && Math.abs(dy) < bd) { bd = Math.abs(dy); best = k; }
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

	// Let go of everything in the claw. assist: line it up on a goal it's right over (the goal-top
	// chamfer funnels it, see Field.assistDrop), moving a held pair together.
	releaseClaw(keepVel = true, assist = false, push = null) {
		const main = this.claw, extra = this.clawExtraPiece;
		if (!main) return;
		this.pulling = null;
		const bottom = this.clawBottom(), top = bottom === main ? extra : main;
		const off = top ? new THREE.Vector3().copy(top.body.translation()).sub(bottom.body.translation()) : null;
		if (extra) this.releasePin(extra, keepVel, push);
		this.releasePin(main, keepVel, push);
		this.clawRel = null;
		if (!assist || !this.field.assistDrop(bottom, 1.3) || !top) return;
		if (top.kind === 'pin' && bottom.kind === 'pin') return;
		const b = bottom.body.translation(), h = off.length();   // stack straight up over the seated bottom piece
		top.body.setTranslation({ x: b.x, y: b.y + h, z: b.z }, true);
		top.body.setRotation(bottom.body.rotation(), true);
		top.body.setLinvel({ x: 0, y: 0, z: 0 }, true); top.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
		top.funnel = { ...bottom.funnel };
	}

	// Start the match holding the preload (<SG5>: red/yellow for red, blue/yellow for blue) in the claw.
	loadPreload() {
		const pin = this.field.makePin(this.alliance, 'yellow', this.toWorld(CFG.clawX, PIN_H / 2 + 0.5, 0));
		this.grab(pin, true);
	}

	// Take an upright piece into the claw (plus whatever is stacked with it). instant: snap it in;
	// otherwise the rollers pull it in over ~0.2 s from where it stands.
	grab(o, instant = false) {
		const t = o.body.translation(), r = o.body.rotation();
		const l = this.toLocal(new THREE.Vector3(t.x, t.y, t.z));
		const half = (o.kind === 'cup' ? CUP_H : PIN_H) / 2;
		this.clawHold = {
			y: Math.max(l.y - this.liftIn(), half + 0.3 - this.liftIn()),
			q: this.quat().invert().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)),
		};
		this.attachPin(o, 'claw');
		this.grabPartner(o);
		this.pulling = instant ? null : { t: 0, dur: 0.2, x: l.x, z: l.z };
	}

	// ---- Physics step ------------------------------------------------------------------
	step(dt) {
		this.t += dt;
		this.stepDrive(dt);
		this.stepMechanisms(dt);
		this.stepPins(dt);
		this.holdAtToggle();
	}

	stepDrive(dt) {
		const q = this.quat();
		const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(q), right = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
		const lv = this.body.linvel(), v = new THREE.Vector3(lv.x, 0, lv.z);
		const vf = v.dot(fwd), vl = v.dot(right), w = this.body.angvel().y;
		const nv = this.drive.step(dt, { L: this.leftCmd, R: this.rightCmd }, this.brake, { f: vf, l: vl, w });
		const m = CFG.mass;
		const imp = fwd.multiplyScalar((nv.f - vf) * m).addScaledVector(right, (nv.l - vl) * m);
		this.body.applyImpulse({ x: imp.x, y: 0, z: imp.z }, true);
		this.body.setAngvel({ x: 0, y: nv.w, z: 0 }, true);
		this.driveForce = this.drive.driveForce;   // N, forward; the Toggle uses it to tell a shove from a bump
		// Encoders (motor deg, slop and wheel spin included) and IMU (deg, clockwise +, unwrapped).
		const circ = Math.PI * CFG.wheelDia * IN, gear = CFG.motorRpm / CFG.driveRpm;
		this.leftPos = (this.drive.encL / circ) * 360 * gear;
		this.rightPos = (this.drive.encR / circ) * 360 * gear;
		this.imuDeg -= (w * dt) / DEG;
		this.wheelSpin = { L: this.drive.L.vw / (CFG.wheelDia / 2 * IN), R: this.drive.R.vw / (CFG.wheelDia / 2 * IN) };
	}

	// ---- Cascade + claw: port of src/mechanisms.cpp ----------------------------------------------
	liftPos() { return -this.liftDeg; }
	// cascade_move(): power [-127, 127], positive = up; stops at the travel limits and eases off near them.
	cascadeMove(power) {
		power = clamp(Math.round(power), -127, 127);
		if (power !== 0) {
			const pos = this.liftPos(), upSign = -1;
			const room = power > 0 ? (CFG.LIFT_LIMIT_TOP_DEG - pos) * upSign : (pos - CFG.LIFT_LIMIT_BOTTOM_DEG) * upSign;
			if (room <= 0) power = 0;
			else if (room < CFG.LIFT_LIMIT_SLOW_DEG) {
				const cap = Math.max(CFG.LIFT_LIMIT_MIN_POWER, Math.trunc((127 * room) / CFG.LIFT_LIMIT_SLOW_DEG));
				power = clamp(power, -cap, cap);
			}
		}
		this.liftPower = power;
		this.liftMode = 'power';
	}
	liftTo(rot) { this.liftTask.goal = rot; this.liftTask.active = true; this.liftMode = 'power'; }
	liftRelease() { this.liftTask.active = false; }
	// lift_drive_scale(): anti-tip, the higher the lift the less drive power (40% at the top).
	liftDriveScale() {
		let f = (this.liftPos() - CFG.LIFT_LIMIT_BOTTOM_DEG) / (CFG.LIFT_LIMIT_TOP_DEG - CFG.LIFT_LIMIT_BOTTOM_DEG);
		if (f <= CFG.LIFT_SLOW_START_FRAC) return 1;
		f = Math.min(f, 1);
		return 1 - ((f - CFG.LIFT_SLOW_START_FRAC) / (1 - CFG.LIFT_SLOW_START_FRAC)) * (1 - CFG.LIFT_SLOW_MIN_SCALE);
	}
	clawGrabToggle() { this.clawMode = this.clawMode === 'GRAB' ? 'STOPPED' : 'GRAB'; }
	clawReleaseToggle() { this.clawMode = this.clawMode === 'RELEASE' ? 'STOPPED' : 'RELEASE'; }
	clawDriverReset() { this.clawMode = 'STOPPED'; }
	mechStopAll() { this.liftRelease(); this.clawDriverReset(); this.cascadeMove(0); }

	// lift_task_fn(): runs every 10 ms on the robot.
	liftTaskStep() {
		const T = this.liftTask;
		if (!T.active) return;
		const err = (T.goal - this.liftPos()) * -1, dist = Math.abs(err);
		if (dist < CFG.LIFT_HOLD_BAND_DEG) T.holding = true;
		else if (dist > CFG.LIFT_RECORRECT_DEG) T.holding = false;
		if (T.holding) { this.cascadeMove(0); return; }
		let p = Math.trunc(3 * err);
		p = err > 0 ? clamp(p, CFG.LIFT_UP_MIN_POWER, 127) : clamp(p, -CFG.LIFT_DOWN_MAX_POWER, -CFG.LIFT_DOWN_MIN_POWER);
		this.cascadeMove(p);
	}

	// The most the lift can rise right now (inches): under a Loader tube the claw hits its top.
	liftCeiling() {
		for (const L of this.field.loaders || []) {
			const l = this.toLocal(W(L.px, L.fy, 0));
			if (Math.abs(l.z) < 4.5 && l.x > CFG.clawX - 3.6 && l.x < CFG.clawX + 3.6) return LOADER.slotTop - 9.4;
		}
		return Infinity;
	}

	stepMechanisms(dt) {
		this.clawSpin = this.clawMode === 'RELEASE' ? CFG.CLAW_RELEASE_SPEED : this.clawMode === 'GRAB' ? CFG.CLAW_GRAB_SPEED : 0;
		if ((this.liftTask.tick = (this.liftTask.tick || 0) + dt) >= 0.01) { this.liftTask.tick -= 0.01; this.liftTaskStep(); }
		const before = this.liftDeg;
		if (this.liftMode === 'pos') {   // simple position mode (AI robots / old auton commands)
			const step = this.liftVelCmd * 6 * dt, e = this.liftTarget - this.liftDeg;
			this.liftDeg += clamp(e, -step, step);
		} else {
			// Cascade motors: speed follows power (a little slower going up against gravity); brake mode holds.
			const p = this.tipped() ? 0 : this.liftPower;
			const want = (p / 127) * CFG.liftDegPerSec * (p > 0 ? 0.92 : 1.05);
			this.liftVel += (want - this.liftVel) * (1 - Math.exp(-dt / (p === 0 ? 0.03 : 0.07)));
			this.liftDeg += this.liftVel * dt;
		}
		const ceil = this.liftCeiling();
		if (this.liftIn() > ceil && this.liftDeg > before) this.liftDeg = Math.max(before, ceil / CFG.liftInPerDeg);
		if (this.liftDeg < 0 || this.liftDeg > CFG.liftMaxDeg) this.liftVel = 0;
		this.liftDeg = clamp(this.liftDeg, 0, CFG.liftMaxDeg);
		this.liftRate = (this.liftDeg - before) / dt;
		this.updateClawColliders();
	}

	updateClawColliders() {
		const lift = this.liftIn();
		for (const k of this.clawCols || []) k.c.setTranslationWrtParent({ x: k.p[0] * IN, y: (k.p[1] + lift) * IN, z: k.p[2] * IN });
	}

	// The mouth of the claw (robot-local inches): where a stack has to be for the rollers to pull it in.
	inMouth(l, slack = 0) { return Math.abs(l.z) < 1.35 + slack && l.x > -CFG.length / 2 - 0.4 - slack && l.x < CFG.clawX + 0.8 + slack; }

	// Speed of a piece relative to the claw mouth (m/s).
	relSpeed(o) {
		const v = this.body.linvel(), w = this.body.angvel().y, c = this.pos(), p = o.body.translation(), pv = o.body.linvel();
		const rx = p.x - c.x, rz = p.z - c.z;
		const vx = v.x + w * rz, vz = v.z - w * rx;   // robot velocity at the piece (w x r)
		return Math.hypot(vx - pv.x, vz - pv.z);
	}

	// Foreign pieces in the mouth (a pin knocked over and caught in the guide, a second stack...).
	mouthBlocker(except) {
		for (const o of [...this.field.pins, ...this.field.cups]) {
			if (o.held || except.includes(o)) continue;
			const l = this.toLocal(new THREE.Vector3().copy(o.body.translation()));
			if (l.y < this.liftIn() + 8 && Math.abs(l.z) < 2.6 && l.x > -CFG.length / 2 - 0.6 && l.x < CFG.clawX + 2.2) return o;
		}
		return null;
	}

	// Pieces knocked by a too-fast claw: tip them over away from the robot.
	knock(o, speed) {
		const now = this.t;
		if ((this.knocked.get(o) || -9) > now - 0.6) return;
		this.knocked.set(o, now);
		const p = o.body.translation(), c = this.pos(), v = this.body.linvel();
		let dx = v.x, dz = v.z;
		if (Math.hypot(dx, dz) < 0.02) { dx = p.x - c.x; dz = p.z - c.z; }
		const n = Math.hypot(dx, dz) || 1;
		dx /= n; dz /= n;
		// Tip about the horizontal axis perpendicular to the push (up x dir).
		const k = clamp(speed * 9, 2.5, 7);
		const partner = this.field.partnerOf(o);
		for (const q of [o, partner].filter(Boolean)) {
			q.body.applyImpulse({ x: dx * 0.02 * speed, y: 0, z: dz * 0.02 * speed }, true);
			q.body.setAngvel({ x: dz * k, y: 0, z: -dx * k }, true);
		}
		if (partner?.kind === 'pin') partner.body.setLinvel({ x: dx * speed * 1.2, y: 0.4, z: dz * speed * 1.2 }, true);
		this.flash?.('TOO FAST');
	}

	stepPins(dt) {
		for (const r of this.justReleased || []) if ((r.t -= dt) <= 0 && !r.pin.held && this.field.has(r.pin)) for (const c of colsOf(r.pin)) c.setCollisionGroups(0xffffffff);
		if (this.justReleased) this.justReleased = this.justReleased.filter((r) => r.t > 0);
		this.releaseCool = Math.max(0, this.releaseCool - dt);
		const spinIn = this.clawSpin < -30, lift = this.liftIn();
		if (!this.claw && !this.tipped()) {
			// Out of a Loader: R1 spinning, cascade low enough to fit the slot, square to the wall, slow.
			const r = this.loaderReach();
			if (r && r.inMouth) {
				const blocker = this.mouthBlocker(r.L.stack || []);
				const sp = Math.hypot(this.body.linvel().x, this.body.linvel().z);
				if (blocker) this.warn('CLAW BLOCKED', 'Something is stuck in the guide: shake it out before loading');
				else if (spinIn && this.releaseCool <= 0 && r.ok && sp < CFG.grabMaxSpeed * 1.3) {
					const st = r.L.stack, o = this.field.takeLoader(r.L);
					this.grab(o);
					for (const k of st) if (!k.held) this.field.freePiece(k);
				}
			}
			// Off the floor or a wall: an upright cup (with its pin) or pin, bottom near the rollers.
			for (const o of [...this.field.pins, ...this.field.cups]) {
				if (this.claw || o.held || !upright(o)) continue;
				const l = this.toLocal(new THREE.Vector3().copy(o.body.translation()));
				const bottom = l.y - (o.kind === 'cup' ? CUP_H : PIN_H) / 2;
				if (bottom < lift - 3.4 || bottom > lift + 2.2 || !this.inMouth(l)) continue;
				if (o.kind === 'pin' && this.field.partnerOf(o)?.kind === 'cup' && this.field.partnerOf(o).body.translation().y < o.body.translation().y) continue;   // pin in a cup: the cup is what's gripped
				if (this.field.onGoal(o)) continue;   // stacked on a goal: not ours to pull off (<SG10>)
				// The speed it hits the rollers at decides: once knocked it's falling, not grabbable.
				if ((this.knocked.get(o) ?? -9) > this.t - 1.2) continue;
				const sp = this.relSpeed(o);
				if (sp > CFG.grabMaxSpeed) { this.knock(o, sp); continue; }
				if (!spinIn || this.releaseCool > 0) continue;
				const partner = this.field.partnerOf(o);
				if (this.mouthBlocker([o, partner].filter(Boolean))) { this.warn('CLAW BLOCKED', 'Something is stuck in the guide'); continue; }
				this.grab(o);
			}
		}
		// Scoring by going down: holding a stack over a goal, lowering the lift until it seats lets go.
		this.clawHeldT = this.claw && this.claw === this.clawHeldObj ? this.clawHeldT + dt : 0;
		this.clawHeldObj = this.claw;
		if (this.claw && !this.pulling && this.clawHeldT > 0.5 && (this.liftRate || 0) < -1) {
			const o = this.clawBottom(), s = this.field.seatFor(o, 1.3), h = o.body.translation().y / IN;
			const half = (o.kind === 'cup' ? CUP_H : PIN_H) / 2;
			if (s && h - half <= s.seatBottom + 0.4) {
				const both = [this.claw, this.clawExtraPiece].filter(Boolean);
				this.releaseClaw(false, true);
				for (const p of both) (this.justPlaced ||= []).push({ o: p, t: 2 });
				this.releaseCool = 1;
			}
		}
		if (this.justPlaced) this.justPlaced = this.justPlaced.filter((r) => (r.t -= dt) > 0);
		// R2 / A spin pushes the stack out the back (lined up on a goal it's right over).
		if (this.claw && this.clawSpin > 30) {
			this.releaseT += dt;
			if (this.releaseT > 0.15) {
				const back = this.fwd().multiplyScalar(-0.22);
				this.releaseClaw(true, true, { x: back.x, y: 0, z: back.z });
				this.releaseT = 0; this.releaseCool = 0.8;
			}
		} else this.releaseT = 0;
		// Move the held stack (being pulled in from the mouth, then riding with the lift).
		if (this.claw) {
			const h = this.clawHold, mq = this.quat().multiply(h.q);
			let x = CFG.clawX, z = 0;
			if (this.pulling) {
				const P = this.pulling, k = Math.min(1, (P.t += dt) / P.dur), e = k * k * (3 - 2 * k);
				x = P.x + (CFG.clawX - P.x) * e; z = P.z * (1 - e);
				if (k >= 1) { this.pulling = null; this.clawMode = this.clawMode === 'GRAB' ? 'STOPPED' : this.clawMode; }   // optical sensor sees it: R1 stops by itself
			}
			// The floor stops a stack the claw lowers onto it (the rollers slide down the cup instead).
			const half = (this.claw.kind === 'cup' ? CUP_H : PIN_H) / 2, rel0 = this.clawRel;
			const e0 = this.clawExtraPiece, below = Math.max(half, e0 && rel0 && rel0.pos.y < 0 ? -rel0.pos.y / IN + (e0.kind === 'cup' ? CUP_H : PIN_H) / 2 : 0);
			const mp = this.toWorld(x, Math.max(h.y + this.liftIn(), below + 0.12), z);
			this.claw.body.setNextKinematicTranslation(mp);
			this.claw.body.setNextKinematicRotation({ x: mq.x, y: mq.y, z: mq.z, w: mq.w });
			const e = this.clawExtraPiece, rel = this.clawRel;
			if (e && rel) {
				const p = rel.pos.clone().applyQuaternion(mq).add(mp), rq = mq.clone().multiply(rel.quat);
				e.body.setNextKinematicTranslation(p);
				e.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
			}
		}
	}

	warn(code, msg) {
		if (this.lastWarn === code && this.t - this.lastWarnT < 2.5) return;
		this.lastWarn = code; this.lastWarnT = this.t;
		this.flash?.(code, msg);
	}

	// The Loader just behind the robot's back, if any: { L, l (stack centre at the floor, robot-local
	// inches), inMouth, ok }. ok = a stack waiting, the robot within 18 deg of square to the wall and
	// the claw low enough to fit the slot. Otherwise the robot just bumps the tube.
	loaderReach() {
		for (const L of this.field.loaders || []) {
			const l = this.toLocal(W(L.px, L.fy, 0));
			if (Math.abs(l.z) > 3 || l.x > CFG.clawX + 2 || l.x < -CFG.length / 2 - 3) continue;
			const fwd = this.fwd();
			const square = fwd.x * L.n > Math.cos(18 * DEG);
			return { L, l, inMouth: !!L.stack?.length && this.inMouth(l), ok: L.ready && square && this.liftIn() + 9.4 <= LOADER.slotTop + 0.2 };
		}
		return null;
	}

	// The Loader tube's upper part (above its slot): with the cascade up, the claw hits it and the
	// robot can't back in under it.
	holdAtToggle() {
		this.toggleHitV = 0;
		const top = this.liftIn() + 9.4;
		if (top <= LOADER.slotTop + 0.2) return;
		const hw = 4.5;
		for (const L of this.field.loaders || []) {
			const l = this.toLocal(W(L.px, L.fy, 0));
			if (Math.abs(l.z) > hw || l.x > CFG.clawX + 6 || l.x < CFG.clawX - 4) continue;
			if (l.x > CFG.clawX + 2.6) continue;   // the tube would have to be inside the claw: stop
			const fwd = this.fwd(), v = this.body.linvel(), vx = v.x * fwd.x + v.z * fwd.z;
			if (vx < 0) this.body.setLinvel({ x: v.x - vx * fwd.x, y: 0, z: v.z - vx * fwd.z }, true);
		}
	}

	// Safety net against solver kicks: the robot can't gain speed faster than its wheels could push it,
	// and never past 2.6 m/s / 12 rad/s. Also feeds the pitch / roll model with the real acceleration.
	clampVel() {
		const tr = this.body.translation(), lv = this.body.linvel(), av = this.body.angvel();
		if ([tr.x, tr.z, lv.x, lv.z, av.y].some((v) => !Number.isFinite(v))) {
			if (this.lastGood) { this.body.setTranslation(this.lastGood.t, true); this.body.setRotation(this.lastGood.r, true); }
			this.body.setLinvel({ x: 0, y: 0, z: 0 }, true); this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
			if (this.clawDeg !== undefined && !Number.isFinite(this.clawDeg)) this.clawDeg = this.clawTarget || 0;
			if (!Number.isFinite(this.liftDeg)) this.liftDeg = 0;
			this.drive.reset();
			return;
		}
		this.lastGood = { t: { ...tr }, r: { ...this.body.rotation() } };
		const v = this.body.linvel(), sp = Math.hypot(v.x, v.z), prev = this.prevSp ?? sp;
		const cap = Math.min(2.6, prev + 14 / 120);
		if (sp > cap) this.body.setLinvel({ x: (v.x * cap) / sp, y: 0, z: (v.z * cap) / sp }, true);
		const w = this.body.angvel().y, pw = this.prevW ?? w, wcap = Math.min(12, Math.abs(pw) + 70 / 120);
		if (Math.abs(w) > wcap) this.body.setAngvel({ x: 0, y: Math.sign(w) * wcap, z: 0 }, true);
		this.prevSp = Math.min(sp, cap); this.prevW = Math.sign(w) * Math.min(Math.abs(w), wcap);
		// Acceleration for pitch / roll (robot-local).
		const nv = this.body.linvel(), dt = 1 / 120;
		if (this.prevVel) {
			const ax = (nv.x - this.prevVel.x) / dt, az = (nv.z - this.prevVel.z) / dt, q = this.quat();
			const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(q), right = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
			if (this.drive.stepTilt(dt, { f: ax * fwd.x + az * fwd.z, l: ax * right.x + az * right.z })) this.onTipped();
		}
		this.prevVel = { x: nv.x, z: nv.z };
	}

	onTipped() {
		if (this.claw) this.releaseClaw(true);
		this.leftCmd = this.rightCmd = 0; this.liftPower = 0; this.clawMode = 'STOPPED';
		this.flash?.('TIPPED OVER', 'Too hard a stop / hit with the cascade up. Press T to reset');
	}

	syncMesh(dt) {
		const t = this.body.translation(), r = this.body.rotation();
		const tl = this.drive.tilt();
		this.mesh.position.set(t.x, t.y, t.z);
		this.mesh.quaternion.set(r.x, r.y, r.z, r.w);
		// Pitch / roll about the axle (or wheel line) it's rocking on.
		const P = this.parts;
		P.tilt.position.set(tl.px * IN, 0, tl.pz * IN);
		P.tilt.rotation.set(-tl.roll, 0, tl.pitch, 'ZXY');
		P.body.position.set(-tl.px * IN, 0, -tl.pz * IN);
		for (const s of P.stages) s.g.position.y = this.liftIn() * s.frac * IN;
		const ws = this.wheelSpin || { L: 0, R: 0 };
		for (const w of P.wheels) w.w.rotation.z -= ws[w.side] * dt;
		for (const g of P.driveGears) g.g.rotation.z -= ws[g.side] * g.ratio * dt;
		const lr = this.liftRate || 0;
		for (const lg of P.liftGears) lg.g.rotation.z += lg.dir * (lr * DEG) * 0.5 * dt;
		for (const led of P.liftLeds) led.material.color.setHex(Math.abs(lr) > 1 ? 0xff3322 : 0x331010);
		for (const rr of P.clawRollers) rr.g.rotation.y += rr.dir * (this.clawSpin / 127) * 22 * dt;
		for (const g of P.clawGears) g.g.rotation.y += g.dir * (this.clawSpin / 127) * 22 * dt;
		if (P.clawLight) {   // running modes pulse so "on" reads at a glance
			const pulse = 0.75 + 0.25 * Math.sin(performance.now() / 90);
			P.clawLight.color.setHex(CLAW_LIGHT[this.clawMode] ?? CLAW_LIGHT.STOPPED);
			if (this.clawMode !== 'STOPPED') P.clawLight.color.multiplyScalar(pulse);
		}
		P.setAlliance?.(this.alliance);
	}

	// ---- Driver control: port of src/opcontrol.cpp (one 20 ms loop per call) -----------
	// c = { ly, rx, L1, L2, R1, R2, A, B, X, Y, Up, Down, Left, Right }
	// hooks: { practiceAuton(), macro(), side(dir) } (null in a real match)
	opcontrol(c, hooks = {}) {
		const o = this.op, newPress = (k) => c[k] && !o.prev[k];
		if (this.tipped()) { this.leftCmd = this.rightCmd = 0; this.cascadeMove(0); o.prev = { ...c }; return; }
		// D-pad Down (no field control): 15 s practice autonomous. B + Y: the routine as a macro.
		if (newPress('Down') && hooks.practiceAuton) { o.prev = { ...c }; hooks.practiceAuton(); return; }
		if (c.B && newPress('Y') && hooks.macro) { o.prev = { ...c }; hooks.macro(); return; }
		// D-pad Left / Right picks the auton side (auton_selector.cpp).
		if (newPress('Left')) hooks.side?.('left');
		if (newPress('Right')) hooks.side?.('right');
		// D-pad Up reverses the driving direction; turning stays the same. Ramped over ~160 ms.
		if (newPress('Up')) { o.inverse = -o.inverse; this.flash?.(o.inverse < 0 ? 'CLAW FIRST' : 'CLAW BACK'); }
		o.inverseScale = o.inverseScale < o.inverse ? Math.min(o.inverseScale + 0.25, o.inverse) : Math.max(o.inverseScale - 0.25, o.inverse);
		let dir = c.ly, turn = c.rx;
		if (Math.abs(dir) < 10) dir = 0;
		if (Math.abs(turn) < 10) turn = 0;
		dir = Math.round(dir * o.inverseScale);
		const tip = this.liftDriveScale();   // anti-tip
		dir = Math.round(dir * tip); turn = Math.round(turn * tip);
		this.leftCmd = clamp(dir + turn, -127, 127);
		this.rightCmd = clamp(dir - turn, -127, 127);
		const estop = c.X && c.A;
		if (newPress('R1') && !estop) { this.clawGrabToggle(); this.liftTo(CFG.R1_LIFT_ROT); o.preset = true; }
		if ((newPress('R2') || newPress('A')) && !estop) this.clawReleaseToggle();
		if (!estop && c.L1) { this.liftRelease(); o.preset = false; this.cascadeMove(127); }
		else if (!estop && c.L2) { this.liftRelease(); o.preset = false; this.cascadeMove(-85); }
		else if (estop || !o.preset) { this.liftRelease(); o.preset = false; this.cascadeMove(0); }
		o.prev = { ...c };
	}

	// ---- HUD / controls page (each robot design has its own) ---------------------------------
	// [mode for the colour, text] of the claw pill: green = grab running, white = stopped, amber = release.
	clawHud() {
		return { GRAB: ['grab', 'CLAW R1 GRAB · closed, spinning in'], RELEASE: ['release', 'CLAW R2 OPEN · spinning out'] }[this.clawMode] || ['stopped', 'CLAW CLOSED · stopped'];
	}
	driveName() { return this.op.inverse < 0 ? 'claw first' : 'claw back'; }
	dirTag() { return this.op.inverse > 0 ? ' · CLAW BACK' : ''; }
	heldText(name) {
		return this.claw ? name(this.claw) + (this.clawExtraPiece ? ' + ' + name(this.clawExtraPiece) : '') : '—';
	}
	// Mechanism rows of the controls page: [action, V5, Xbox, keyboard, note].
	static controlRows(k, x) {
		return [
			['Claw grab spin + intake height', k('R1'), k('RB'), k('E'), 'Toggle; also sends the cascade to -70 (2" up), like opcontrol.cpp. Drive claw first into a standing stack SLOWLY (under ~6 in/s): the rollers pull it in and the spin stops by itself. Faster and you knock it over. Something stuck in the guide (a knocked-over pin) blocks the claw'],
			['Claw release spin', k('R2') + ' / ' + k('A'), k('RT') + ' / ' + k(x.A), k('Q') + ' / ' + k('F'), 'Toggle. Pushes the stack out the back (onto a goal if it\'s right over one). Lowering the stack onto a goal also lets go'],
			['Take a match load', k('R1'), k('RB'), k('E'), 'Cascade at R1 height (it must fit under the tube), back in square to the wall, slow. Your human player drops loads in the top (10 Cups + 10 Pins per alliance, driver period only)'],
			['Hit a Toggle', 'Drive', 'Drive', 'Drive', 'The black rubber pucks on the front arms hit it at its height. One hit = one face: back off and hit again'],
			['Flip drive direction', k('↑'), 'D-pad ↑', k('↑'), 'Starts claw first (stick up drives claw first), like opcontrol.cpp; turning unchanged'],
		];
	}

	// After a practice auton / macro (claw_driver_reset(), preset cleared), like opcontrol.cpp.
	afterAutonTest() { this.clawDriverReset(); this.op.preset = false; this.liftRelease(); }
}
