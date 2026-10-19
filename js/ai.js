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

function aiMesh() {
	const g = new THREE.Group(), s = IN;
	const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, metalness: 0.4, ...o });
	const metal = mat(0xb8bcc2, { metalness: 0.8, roughness: 0.35 }), dark = mat(0x222428), bump = mat(0x888888, { metalness: 0.1 });
	const box = (w, h, d, x, y, z, m, parent = g) => {
		const b = new THREE.Mesh(new THREE.BoxGeometry(w * s, h * s, d * s), m);
		b.position.set(x * s, y * s, z * s); b.castShadow = true; parent.add(b); return b;
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
	const intake = [];
	for (const y of [2, 5]) {
		const r = new THREE.Mesh(new THREE.CylinderGeometry(0.8 * s, 0.8 * s, (Wd - 4) * s, 10), mat(0x3a3d42));
		r.rotation.x = Math.PI / 2; r.position.set((hl - 0.5) * s, y * s, 0); g.add(r); intake.push(r);
	}
	// Two-stage lift towers at the back, claw arm hanging from the top stage.
	for (const z of [-hw + 2, hw - 2]) box(1, 14, 1, -hl + 2.5, 9, z, metal);
	const top = new THREE.Group(); g.add(top);
	for (const z of [-hw + 2.8, hw - 2.8]) box(1, 13, 1, -hl + 2.5, 10, z, metal, top);
	box(1.2, 1, Wd - 4, -hl + 2.5, 16, 0, metal, top);
	const pivot = new THREE.Group(); pivot.position.set(-6.5 * s, (CFG.pinBottomAtHorizontal + PIN_H - 1) * s, 0); top.add(pivot);
	for (const z of [-3, 3]) box(1, CFG.clawArm, 0.8, 0, -CFG.clawArm / 2, z, metal, pivot);
	box(1.4, 1.4, 7, 0, -CFG.clawArm, 0, dark, pivot);
	return { group: g, wheels, intake, stages: [{ g: top, frac: 1 }], clawPivot: pivot, bump };
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

	buildMesh() {
		const m = aiMesh();
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
			const clear = g.type === 'tall' ? 20 : 15;   // goal base + half the robot
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

	// Called every physics step. `others` are all robots (to avoid chasing the same pin).
	think(dt, others) {
		const ai = this.ai, field = this.field;
		ai.t += dt;
		for (const [k, until] of ai.ban) if (until < this.t) ai.ban.delete(k);
		this.liftMode = 'pos'; this.brake = 'coast';
		// Stuck check: barely moved in 2.5 s while trying to drive -> back off and pick something else.
		const p = this.fpos();
		if (ai.state !== 'unstick' && !ai.aligning) {
			ai.stuckT += dt;
			if (ai.stuckT > 1.5) {
				if (ai.lastPos && Math.hypot(p.fx - ai.lastPos.fx, p.fy - ai.lastPos.fy) < 2.5) {
					if (ai.target) ai.ban.set(ai.target, this.t + 8);
					ai.state = 'unstick'; ai.t = 0; ai.prev = this.claw ? 'score' : 'fetch'; ai.target = null;
				}
				ai.stuckT = 0; ai.lastPos = p;
			}
		}
		if (ai.state === 'unstick') {
			this.arcade(-0.6, ai.t < 0.5 ? 0.5 : 0);
			if (ai.t > 0.9) { ai.state = ai.prev; ai.t = 0; ai.stuckT = 0; ai.lastPos = null; }
			return;
		}
		if (this.claw && ai.state === 'fetch') { this.setState('score'); ai.target = null; }

		if (ai.state === 'fetch') {
			this.clawTarget = CFG.CLAW_TURN_VERTICAL_DEG; this.liftTarget = 0;
			this.intakeCmd = 127; this.clawSpin = this.hopper ? -127 : 0;
			if (this.hopper) { this.arcade(0, 0); return; }   // wait for the hand-off to the claw
			const valid = (o) => o && !o.held && (field.pins.includes(o) || field.cups.includes(o)) && !this.isScored(o);
			if (ai.target && ai.t > 7) { ai.ban.set(ai.target, this.t + 15); ai.target = null; }   // can't get it: try another
			if (!valid(ai.target)) ai.t = 0;
			if (!valid(ai.target)) ai.target = this.pickGoal('pin') ? this.pickPin(others) : this.pickGoal('cup') ? this.pickCup(others) : null;
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
			const value = [pin.top, pin.bottom].reduce((a, c) => a + (c === mine ? 1 : c === 'yellow' ? 0.4 : -1), 0);
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
		if (kind === 'cup') return top.cupSeat != null && top.cupSeat < 14 ? top.cupSeat : null;
		return top.surface != null && top.surface < 16 ? top.surface : null;   // keep stacks short enough to stay up
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
		for (const g of this.field.goals) {
			if (g.type === 'tall' || (g.color !== 'neutral' && g.color !== this.alliance) || this.ai.ban.has(g) || this.goalTop(g, this.claw, kind) == null) continue;
			const d = Math.hypot(g.fx - p.fx, g.fy - p.fy);
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
