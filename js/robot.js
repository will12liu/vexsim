// The Titanium robot: drivetrain physics, cascade lift, 3-position claw flip, claw spin,
// front intake, aux pistons, and a port of src/opcontrol.cpp.
// Robot-local frame (inches): +x forward (intake side), +y up, +z right. Claw is on the back.
import * as THREE from 'three';
import { IN, W, PIN_H, MID, GROUP, groups, toField, TOGGLE } from './field.js';
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
	// mechanisms.h (10/15): B cycles vertical -> highest -> lift -> horizontal -> vertical
	CLAW_TURN_VERTICAL_DEG: 25, CLAW_TURN_HORIZONTAL_DEG: 900, CLAW_LIFT_FRACTION: 0.25,
	CASCADE_INTAKE_DEG: 180,            // R1 sends the cascade here to intake (claw side), at velocity 45
	CLAW_TURN_VELOCITY: 200, CLAW_TURN_DOWN_VELOCITY: 100,
	CLAW_DROP_SPEED: 127, CLAW_DROP_MS: 700,
	clawDegPerMotorDeg: 0.1,            // 900 motor deg = 90 deg of arm swing (assumed)
	clawArm: 9,                         // pivot -> grip, inches (from photos)
	pinBottomAtHorizontal: 25 / 2.54 - 5.5,   // team (2026-10-15): claw lowered, pivot 25 cm off the floor with the lift down
	// Goal guide (team addition 10/17): a V of channel on the back. Backing into a goal, the post
	// slides into the V and stops where the claw at 90 deg holds the pin right over the goal.
	guide: { r: 2.15, t: 0.5, armLen: 5.5, y0: 2.4, y1: 3.2 },   // goal radius it seats on, channel thickness, length of each V arm (11 holes), height band
	liftInPerDeg: 0.03,                 // lift inches per cascade motor degree (assumed)
	liftMaxIn: 62.1 / 2.54,             // team: fully up the top of the cascade is 105 cm (top stage starts 2.1 cm below the frame)
	liftDegPerSec: 1200,                // 200 rpm green at full command
	LIFT_ALLIANCE_GOAL_DEG: 400, LIFT_SHORT_GOAL_DEG: 700, LIFT_TALL_GOAL_DEG: 1100, LIFT_VELOCITY: 200,
};
CFG.CLAW_TURN_LIFT_DEG = 2 * CFG.CLAW_TURN_HORIZONTAL_DEG * CFG.CLAW_LIFT_FRACTION;     // 450: slanted (claw_up / claw_lift_off)
CFG.CLAW_TURN_HIGHEST_DEG = CFG.CLAW_TURN_HORIZONTAL_DEG + CFG.CLAW_TURN_LIFT_DEG;      // 1350: past horizontal, up high
CFG.liftMaxDeg = CFG.liftMaxIn / CFG.liftInPerDeg;
// Guide layout (robot-local inches): the goal centre seats under the claw grip at 90 deg.
{
	const g = CFG.guide, seat = -6.5 - CFG.clawArm;               // grip x with the claw at 90
	g.seatX = seat;
	g.apexX = seat + (g.r + g.t / 2) / Math.SQRT1_2;   // V half-angle 45 deg
}

const DEG = Math.PI / 180;
// All colliders of a pin or cup.
const colsOf = (o) => (o.collider ? [o.collider, ...o.extra] : Array.from({ length: o.body.numColliders() }, (_, i) => o.body.collider(i)));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

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
		// U-shaped base (open at the front so pins can reach the intake), corner towers,
		// and the front top crossbar that hits the Toggle.
		this.colliders = [
			box(-hl, hl, 0, 4, -hw, -hw + 2), box(-hl, hl, 0, 4, hw - 2, hw),
			box(-hl, -hl + 1.5, 0, 4, -hw + 2, hw - 2),
			box(hl - 2, hl, 4, H, -hw, -hw + 1.5), box(hl - 2, hl, 4, H, hw - 1.5, hw),
			box(-hl, -hl + 2, 4, H, -hw, -hw + 1.5), box(-hl, -hl + 2, 4, H, hw - 1.5, hw),
			box(hl - 1.5, hl, H - 5, H, -hw, hw),
			// Intake rollers + bumper across the front, and the inside of the robot (ramp,
			// motors, Brain, battery). Solid so goals and other robots can't drive into it;
			// pins are still grabbed because the intake reaches ~3" past the front.
			box(hl - 2.5, hl + 0.8, 0.6, 7.5, -hw + 2, hw - 2, 0),   // massless: the robot's weight is already set
			box(-hl + 1.5, hl - 2.5, 1.2, 10, -hw + 2, hw - 2, 0),
			// Side and back panels (towers, motors, cascade): nothing pokes into the robot above the rails.
			box(-hl + 2, hl - 2, 4, 12, -hw, -hw + 1.5, 0), box(-hl + 2, hl - 2, 4, 12, hw - 1.5, hw, 0),
			box(-hl, -hl + 1.5, 4, 10, -hw + 1.5, hw - 1.5, 0),
		];
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
			// End channel (2 holes wide, like the mesh), open at the grip: a pin/cup there is in the
			// gripper (grabbed, or just placed and the claw lowering past it), not hitting the frame.
			{ c: clawBox(1.0, 0.5, (cw + 0.5 - 2.2) / 2), p: [0, -arm, -(cw + 0.5 + 2.2) / 2] },
			{ c: clawBox(1.0, 0.5, (cw + 0.5 - 2.2) / 2), p: [0, -arm, (cw + 0.5 + 2.2) / 2] },
			{ c: clawBox(0.5, arm / 2, 0.5), p: [0, -arm / 2, -cw] },
			{ c: clawBox(0.5, arm / 2, 0.5), p: [0, -arm / 2, cw] },
		];
		// Goal guide: two channel arms in a V opening backward (see CFG.guide). Our robot only.
		if (!this.noGuide) {
			const g = CFG.guide, len = g.armLen, hy = (g.y1 - g.y0) / 2;
			for (const s of [-1, 1]) {
				const dir = new THREE.Vector3(-Math.SQRT1_2, 0, s * Math.SQRT1_2);
				const c = new THREE.Vector3(g.apexX, (g.y0 + g.y1) / 2, 0).addScaledVector(dir, len / 2);
				const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0), dir);
				this.colliders.push(world.createCollider(R.ColliderDesc.cuboid((len / 2) * IN, hy * IN, (g.t / 2) * IN)
					.setTranslation(c.x * IN, c.y * IN, c.z * IN).setRotation({ x: q.x, y: q.y, z: q.z, w: q.w })
					.setFriction(0).setFrictionCombineRule(R.CoefficientCombineRule.Min).setDensity(0)
					.setCollisionGroups(groups(GROUP.robot, 0xffff)), this.body));
			}
		}
		this.mesh = this.buildMesh();
		this.scene.add(this.mesh);
	}

	buildMesh() {
		const m = buildRobotMesh(CFG);
		Object.assign(this, { wheels: m.wheels, intakeRollers: m.intakeRollers, clawRollers: m.clawRollers, pistons: m.pistons, stages: m.stages, clawPivot: m.clawPivot, driveGears: m.driveGears, liftGears: m.liftGears, liftLeds: m.liftLeds });
		return m.group;
	}

	resetState() {
		// Motors / mechanisms
		this.leftCmd = 0; this.rightCmd = 0; this.brake = 'coast';
		this.liftDeg = 0; this.liftMode = 'volt'; this.liftCmd = 0; this.liftTarget = 0; this.liftVel = CFG.LIFT_VELOCITY;
		this.clawDeg = 0; this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.clawVel = CFG.CLAW_TURN_DOWN_VELOCITY;   // claw tared at 0 at boot; driver control holds it at 25
		this.clawSpin = 0; this.intakeCmd = 0; this.aux = false;
		this.clawTurnHorizontal = false;
		// Sensors
		this.leftPos = 0; this.rightPos = 0; this.imuDeg = 0; this.t = 0;
		// Pins
		this.hopper = null; this.claw = null; this.clawExtraPiece = null; this.clawRel = null; this.transferT = 0; this.releaseT = 0;
		// opcontrol state
		this.op = { inverse: 1, inverseScale: 1, intakeRunning: false, intakeReversed: false, cascadeIntake: false, clawIn: false, clawOut: false, r1: 0, clawState: 0, prev: {} };
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

	// Which way round a piece goes in: pins and cups aren't symmetric (pin colours, the cup's
	// opaque half), so the end that comes in first stays first. flipped = held the other way
	// round from the default pose (the pose at robot-local rotation `angle` about Z).
	decideFlip(o, angle) {
		const def = new THREE.Vector3(0, 1, 0).applyQuaternion(this.quat().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), angle)));
		const r = o.body.rotation(), act = new THREE.Vector3(0, 1, 0).applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w));
		o.flipped = def.dot(act) < 0;
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
		const b = bottom.body.translation(), h = off.length();   // stack straight up over the seated bottom piece
		top.body.setTranslation({ x: b.x, y: b.y + h, z: b.z }, true);
		top.body.setRotation(bottom.body.rotation(), true);
		top.body.setLinvel({ x: 0, y: 0, z: 0 }, true); top.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
		top.funnel = { ...bottom.funnel };
	}

	// Grip point (end of the claw frame) in world coordinates.
	clawGripWorld() {
		const a = this.clawArmAngle(), py = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		return this.toWorld(-6.5 - CFG.clawArm * Math.sin(a), py - CFG.clawArm * Math.cos(a), 0);
	}

	// Is the end channel of the claw frame touching this Toggle? (Points along it vs the Toggle's prism.)
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

	// With the claw frame against a Toggle the robot can't keep going toward the wall (it's solid).
	holdAtToggle() {
		// The claw frame sticking out at wall height stops against the perimeter (it isn't a collider
		// there: see clawCols). Block motion that would push its end past a wall.
		const [p0, p1] = this.clawSeg, a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		for (const z of [p0[2], 0, p1[2]]) {
			const f = toField(this.toWorld(-6.5 + p0[0] * ca + p0[1] * sa, py - p0[0] * sa + p0[1] * ca, z));
			if (f.h > 12.5) continue;
			const v = this.body.linvel();
			if (Math.abs(f.fx) > 71.4 && Math.sign(v.x) === Math.sign(f.fx)) this.body.setLinvel({ x: 0, y: 0, z: v.z }, true);
			if (Math.abs(f.fy) > 71.4 && Math.sign(-v.z) === Math.sign(f.fy)) this.body.setLinvel({ x: this.body.linvel().x, y: 0, z: 0 }, true);
		}
		const tg = this.clawToggle;
		if (!tg) { this.toggleHitV = 0; return; }
		const v = this.body.linvel(), n = tg.nWorld, vn = v.x * n.x + v.z * n.z;
		this.toggleHitV = Math.max(this.toggleHitV || 0, -vn);   // impact speed, for the Toggle's shove check
		if (vn < 0) this.body.setLinvel({ x: v.x - vn * n.x, y: 0, z: v.z - vn * n.z }, true);
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
			if (!Number.isFinite(this.clawDeg)) this.clawDeg = this.clawTarget || 0;
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

	// ---- Claw geometry -----------------------------------------------------------------
	clawArmAngle() { return this.clawDeg * CFG.clawDegPerMotorDeg * DEG; }   // 0 = hanging down, 90 = pointing back
	liftIn() { return clamp(this.liftDeg * CFG.liftInPerDeg, 0, CFG.liftMaxIn); }
	// Held pin pose. The pin is fixed across the end of the claw frame: lying flat (pointing
	// forward, as it comes off the intake) with the claw down, hanging straight down at 90 deg.
	clawPinLocal() {
		const a = this.clawArmAngle();
		const pivotY = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		const gx = -6.5 - CFG.clawArm * Math.sin(a), gy = pivotY - CFG.clawArm * Math.cos(a);
		const off = PIN_H / 2 - 1, angle = Math.PI / 2 - a;
		const pos = new THREE.Vector3(gx + Math.cos(a) * off, gy - Math.sin(a) * off, 0);
		// The claw hangs down to near the floor: keep the held piece resting on top of the floor
		// (and the base plate) instead of sinking into it.
		const lowest = pos.y - Math.abs(Math.cos(angle)) * (PIN_H / 2) - Math.abs(Math.sin(angle)) * 1.58;
		if (lowest < 0.3) pos.y += 0.3 - lowest;
		return { pos, angle };
	}

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

	// Start the match holding the preload in the claw.
	loadPreload() {
		const pin = this.field.makePin('yellow', 'yellow', this.toWorld(0, 10, 0));   // preload is yellow/yellow
		this.attachPin(pin, 'claw');
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
		this.liftDeg = clamp(this.liftDeg, 0, CFG.liftMaxDeg);
		this.liftRate = (this.liftDeg - liftBefore) / dt;   // motor deg/s, for the spinning gears
		// The claw motor stalls with the frame pressed on a Toggle: it can't swing further that way.
		this.clawToggle = this.field.toggles.find((tg) => this.clawTouches(tg)) || null;
		const step = this.clawVel * 6 * dt, e = this.clawTarget - this.clawDeg;
		const move = this.clawToggle && Math.sign(e) === this.clawDir ? 0 : clamp(e, -step, step);
		if (move) this.clawDir = Math.sign(move);
		this.clawDeg += move;
		const a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = CFG.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn();
		const rq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -a);
		for (const k of this.clawCols) {
			const [x, y, z] = k.p;   // pivot-local, same transform as the claw mesh (rotation.z = -a)
			k.c.setTranslationWrtParent({ x: (-6.5 + x * ca + y * sa) * IN, y: (py - x * sa + y * ca) * IN, z: z * IN });
			k.c.setRotationWrtParent({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		}
	}

	stepPins(dt) {
		for (const r of this.justReleased || []) if ((r.t -= dt) <= 0 && !r.pin.held && (this.field.pins.includes(r.pin) || this.field.cups.includes(r.pin))) for (const c of colsOf(r.pin)) c.setCollisionGroups(0xffffffff);
		if (this.justReleased) this.justReleased = this.justReleased.filter((r) => r.t > 0);
		// Intake: grab a pin or cup sitting in front of the rollers. One piece at a time: nothing is
		// picked up while the robot already holds one (in the intake or the claw).
		// The claw has to be at its starting position (down) or the piece jams: nothing is taken in.
		if (this.intakeCmd > 30 && !this.hopper && !this.claw && this.clawDeg < 150) {
			for (const pin of [...this.field.pins, ...this.field.cups]) {   // cups go through the same way as pins
				if (pin.held) continue;
				const f = toField(pin.body.translation());
				if (this.field.goals.some((g) => Math.hypot(f.fx - g.fx, f.fy - g.fy) < 2.5 && f.h > g.base)) continue;   // seated on a post
				const l = this.toLocal(new THREE.Vector3().copy(pin.body.translation()));
				if (l.x > CFG.length / 2 - 3 && l.x < CFG.length / 2 + 4.5 && Math.abs(l.z) < 5 && l.y < 8) { this.decideFlip(pin, Math.PI / 2); this.attachPin(pin, 'hopper'); break; }
			}
		}
		if (this.intakeCmd < -30 && this.hopper) this.releasePin(this.hopper, true, 1.2);
		// Hand-off up the deflector to the claw: claw down, lift down, claw spinning in.
		if (this.hopper && !this.claw && this.clawSpin < -30 && this.clawDeg < 150 && this.liftDeg < CFG.CASCADE_INTAKE_DEG + 60) {   // R1 holds the cascade at the intake height
			this.transferT += dt;
			if (this.transferT > 0.25) { const p = this.hopper; this.hopper = null; p.held = null; this.attachPin(p, 'claw'); this.transferT = 0; }
		} else this.transferT = 0;
		// The claw as an intake: swung out and spinning the grab way, it picks up an upright pin or
		// cup whose top is at the grip (off the floor, a goal or a stack). One piece at a time.
		if (!this.claw && !this.hopper && this.clawSpin < -30 && this.clawDeg > 100) {
			const g = this.clawGripWorld(), recent = new Set((this.justPlaced || []).map((r) => r.o));
			for (const o of [...this.field.pins, ...this.field.cups]) {
				if (o.held || recent.has(o)) continue;
				const t = o.body.translation(), r = o.body.rotation();
				const ax = new THREE.Vector3(0, 1, 0).applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w));
				// Upright pieces are gripped near the top; lying ones (on the floor) round the middle.
				const upright = Math.abs(ax.y) > 0.8;
				if (!upright && Math.abs(ax.y) > 0.5) continue;
				const grab = new THREE.Vector3(t.x, t.y, t.z).addScaledVector(ax, upright ? Math.sign(ax.y) * (PIN_H / 2 - 1) * IN : 0);
				if (Math.hypot(grab.x - g.x, grab.z - g.z) < 2 * IN && Math.abs(grab.y - g.y) < 2.5 * IN) { this.decideFlip(o, this.clawPinLocal().angle); this.attachPin(o, 'claw'); this.grabPartner(o); break; }   // rollers reach ~2" around
			}
		}
		// Scoring by going down: holding a piece out over a goal, lowering the lift until it seats
		// lets go. Only while actually lowering, and not right after picking the piece up.
		this.clawHeldT = this.claw && this.claw === this.clawHeldObj ? this.clawHeldT + dt : 0;
		this.clawHeldObj = this.claw;
		if (this.claw && this.clawHeldT > 0.5 && (this.liftRate || 0) < -1 && Math.abs(this.clawDeg - CFG.CLAW_TURN_HORIZONTAL_DEG) < 200) {
			const o = this.clawBottom(), s = this.field.seatFor(o), h = o.body.translation().y / IN;
			if (s && h - PIN_H / 2 <= s.seatBottom + 0.4) {
				const both = [this.claw, this.clawExtraPiece].filter(Boolean);
				this.releaseClaw(false, true);
				for (const p of both) (this.justPlaced ||= []).push({ o: p, t: 2 });
			}
		}
		if (this.justPlaced) this.justPlaced = this.justPlaced.filter((r) => (r.t -= dt) > 0);
		// Spinning the claw the drop direction lets go of the pin.
		if (this.claw && this.clawSpin > 30) {
			this.releaseT += dt;
			if (this.releaseT > 0.12) { this.releaseClaw(true, true); this.releaseT = 0; }
		} else this.releaseT = 0;
		// Move held pins.
		const q = this.quat();
		const Z = new THREE.Vector3(0, 0, 1);
		const pose = (pin, l, angle) => {
			const rq = q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Z, angle));
			pin.body.setNextKinematicTranslation(this.toWorld(l.x, l.y, l.z));
			pin.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		};
		const flip = (o) => (o.flipped ? Math.PI : 0);   // held the other way round (bottom end first)
		if (this.hopper) pose(this.hopper, new THREE.Vector3(2.5, 4.5, 0), Math.PI / 2 + flip(this.hopper));   // lying flat on the ramp
		if (this.claw) {
			const l = this.clawPinLocal(); l.angle += flip(this.claw); pose(this.claw, l.pos, l.angle);
			const e = this.clawExtraPiece, rel = this.clawRel;
			if (e && rel) {   // stacked partner (cup under the pin / cup on top) rides along rigidly
				const mq = q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Z, l.angle)), mp = this.toWorld(l.pos.x, l.pos.y, l.pos.z);
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
		this.clawPivot.rotation.z = -this.clawArmAngle();
		const ws = this.wheelSpin || { L: 0, R: 0 };
		for (const w of this.wheels) w.w.rotation.y += (w.side === 'C' ? (ws.L + ws.R) / 2 * 1.6 : ws[w.side]) * dt;
		// Cascade motor gears spin with the lift; their LEDs light while driving.
		const lr = this.liftRate || 0;
		for (const lg of this.liftGears) lg.g.rotation.z += lg.dir * (lr * DEG) * 0.5 * dt;
		for (const led of this.liftLeds) led.material.color.setHex(Math.abs(lr) > 1 ? 0xff3322 : 0x331010);
		for (const dg of this.driveGears) dg.g.rotation.z -= (ws[dg.side] || 0) * dt;
		for (const rr of this.intakeRollers) rr.rotation.z += (this.intakeCmd / 127) * 25 * dt;
		for (const rr of this.clawRollers) rr.rotation.z += (this.clawSpin / 127) * 25 * dt;
		for (const p of this.pistons) p.position.y = p.userData.y0 - (this.aux ? 1.5 * IN : 0);
	}

	// ---- Driver control: port of src/opcontrol.cpp (one 20 ms loop per call) -----------
	// c = { ly, rx, L1, L2, R1, R2, A, B, X, Y, Up, Down, Left, Right }
	opcontrol(c, onRunAuton) {
		const o = this.op, newPress = (k) => c[k] && !o.prev[k];
		if (newPress('Down') && onRunAuton) onRunAuton();
		// 1. Drivetrain. D-pad Up toggles inverted driving: the claw side becomes the front, so stick
		// up drives claw first. Turning stays the same. Ramped over ~160 ms.
		if (newPress('Up')) { o.inverse = -o.inverse; this.flash?.(o.inverse < 0 ? 'INVERTED' : 'normal'); }
		o.inverseScale = o.inverseScale < o.inverse ? Math.min(o.inverseScale + 0.25, o.inverse) : Math.max(o.inverseScale - 0.25, o.inverse);
		let dir = c.ly, turn = c.rx;
		if (Math.abs(dir) < 10) dir = 0;
		if (Math.abs(turn) < 10) turn = 0;
		dir = Math.round(dir * o.inverseScale);
		this.brake = 'coast';
		this.leftCmd = clamp(dir + turn, -127, 127);
		this.rightCmd = clamp(dir - turn, -127, 127);
		const estop = c.L1 && c.R1;
		// R1 cycles: intake + claw spin out + cascade to the intake height -> claw spin out only -> off.
		if (newPress('R1') && !estop) {
			o.r1 = (o.r1 + 1) % 3;
			o.intakeRunning = o.r1 === 1;
			o.intakeReversed = false;
			o.clawOut = o.r1 !== 0;
			o.clawIn = false;
			o.cascadeIntake = o.r1 === 1;
		}
		// R2 toggles both motors opposite their R1 directions; press again to stop.
		if (newPress('R2') && !estop) {
			o.intakeReversed = !o.intakeReversed;
			o.intakeRunning = false; o.clawIn = false; o.clawOut = false;
		}
		this.intakeCmd = o.intakeReversed ? -127 : o.intakeRunning ? 127 : 0;
		// Cascade: L1 up, L2 down; R1's intake height otherwise. Manual controls cancel that target.
		if (estop) { o.cascadeIntake = false; this.liftMode = 'volt'; this.liftCmd = 0; }
		else if (c.L1) { o.cascadeIntake = false; this.liftMode = 'volt'; this.liftCmd = 127; }
		else if (c.L2) { o.cascadeIntake = false; this.liftMode = 'volt'; this.liftCmd = -85; }
		else if (o.cascadeIntake) { this.liftMode = 'pos'; this.liftTarget = CFG.CASCADE_INTAKE_DEG; this.liftVel = 45; }
		else { this.liftMode = 'volt'; this.liftCmd = 0; }
		if (newPress('A')) { o.clawIn = !o.clawIn; o.clawOut = false; }
		if (newPress('Y')) { o.clawOut = !o.clawOut; o.clawIn = false; }
		this.clawSpin = o.intakeReversed ? 127 : o.clawIn ? 45 : o.clawOut ? -127 : 0;
		// Claw turn (opcontrol.cpp): B cycles vertical -> highest -> lift -> horizontal -> vertical.
		if (newPress('B')) {
			o.clawState = (o.clawState + 1) % 4;
			this.clawTarget = [CFG.CLAW_TURN_VERTICAL_DEG, CFG.CLAW_TURN_HIGHEST_DEG, CFG.CLAW_TURN_LIFT_DEG, CFG.CLAW_TURN_HORIZONTAL_DEG][o.clawState];
			this.clawTurnHorizontal = o.clawState !== 0;
		}
		// X: one flip straight to 90, X again back down to rest (same state as B: 3 = horizontal).
		if (newPress('X')) {
			o.clawState = o.clawState !== 3 ? 3 : 0;
			this.clawTarget = o.clawState === 3 ? CFG.CLAW_TURN_HORIZONTAL_DEG : CFG.CLAW_TURN_VERTICAL_DEG;
			this.clawTurnHorizontal = o.clawState !== 0;
		}
		this.clawVel = this.clawTurnHorizontal ? CFG.CLAW_TURN_VELOCITY : CFG.CLAW_TURN_DOWN_VELOCITY;
		o.prev = { ...c };
	}


	// After an auton test run from driver control (D-pad DOWN), reset like the real code does.
	afterAutonTest() {
		const o = this.op;
		o.clawState = 0; this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.clawTurnHorizontal = false;
		o.intakeRunning = false; o.intakeReversed = false; o.cascadeIntake = false; o.clawIn = false; o.clawOut = false; o.r1 = 0;
	}
}
