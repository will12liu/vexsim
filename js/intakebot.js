// The first Titanium robot (Sept-Oct 2026): ground intake on the front feeding a hopper, cascade lift,
// and a big claw frame on the back that flips through 4 positions (vertical, highest, lift, 90 deg)
// and spins to grab / drop. Brought back from the sim's history (commit 9472eea) as a robot you can
// pick on the title menu. It drives on the same drivetrain physics as the current robot (motors,
// gear slop, omni slide, tipping) and the same cascade code; the claw / intake / hopper and the
// controls are the old opcontrol.cpp (10/15). Robot-local inches: +x = intake (front), +y up, +z right.
import * as THREE from 'three';
import { IN, W, WALL, PIN_H, GROUP, groups, toField, TOGGLE } from './field.js';
import { Robot, CFG } from './robot.js';
import { buildIntakeMesh } from './intakemodel.js';

const DEG = Math.PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const PAIR_GAP = 3.4;   // inches between two pins held side by side in the claw

// mechanisms.h (10/15) and the old robot's measurements.
export const ICFG = {
	...CFG,
	liftMaxIn: 62.1 / 2.54,             // team: fully up, the top of the cascade is 105 cm
	CLAW_TURN_VERTICAL_DEG: 25, CLAW_TURN_HORIZONTAL_DEG: 900, CLAW_LIFT_FRACTION: 0.25,
	CASCADE_INTAKE_DEG: 180,            // R1 sends the cascade here to intake (rotation sensor degrees)
	CLAW_TURN_VELOCITY: 200, CLAW_TURN_DOWN_VELOCITY: 100,
	clawDegPerMotorDeg: 0.1,            // 900 motor deg = 90 deg of arm swing
	clawArm: 9,                         // pivot -> grip, inches
	pinBottomAtHorizontal: 25 / 2.54 - 5.5,   // claw lowered, pivot 25 cm off the floor with the lift down
	// Goal guide: a V of channel on the back. Backing onto a goal, the post slides into the V and stops
	// where the claw at 90 deg holds the pin right over it.
	guide: { r: 2.15, t: 0.5, armLen: 4, y0: 2.4, y1: 3.2 },
};
ICFG.CLAW_TURN_LIFT_DEG = 2 * ICFG.CLAW_TURN_HORIZONTAL_DEG * ICFG.CLAW_LIFT_FRACTION;   // 450
ICFG.CLAW_TURN_HIGHEST_DEG = ICFG.CLAW_TURN_HORIZONTAL_DEG + ICFG.CLAW_TURN_LIFT_DEG;    // 1350
ICFG.guide.seatX = -6.5 - ICFG.clawArm;
ICFG.guide.apexX = ICFG.guide.seatX + (ICFG.guide.r + ICFG.guide.t / 2) / Math.SQRT1_2;
const C = ICFG;

export class IntakeBot extends Robot {
	build() {
		const { R, world } = this;
		const L = C.length, Wd = C.width, H = C.height, hl = L / 2, hw = Wd / 2;
		const m = C.mass, I = (m * ((L * IN) ** 2 + (Wd * IN) ** 2)) / 12 * 0.85;
		this.body = world.createRigidBody(R.RigidBodyDesc.dynamic().setCanSleep(false).setLinearDamping(0).setAngularDamping(0)
			.setAdditionalMassProperties(m, { x: 0, y: 0.15, z: 0 }, { x: I, y: I, z: I }, { x: 0, y: 0, z: 0, w: 1 }));
		this.body.setEnabledRotations(false, true, false, true);
		this.body.setEnabledTranslations(true, false, true, true);
		const box = (x0, x1, y0, y1, z0, z1, fr = 0.2) => world.createCollider(R.ColliderDesc.cuboid(((x1 - x0) / 2) * IN, ((y1 - y0) / 2) * IN, ((z1 - z0) / 2) * IN)
			.setTranslation(((x0 + x1) / 2) * IN, ((y0 + y1) / 2) * IN, ((z0 + z1) / 2) * IN)
			.setFriction(fr).setFrictionCombineRule(R.CoefficientCombineRule.Min).setDensity(0)
			.setCollisionGroups(groups(GROUP.robot, 0xffff)), this.body);
		// U-shaped base (open at the front so pieces reach the intake), corner towers, the front top
		// crossbar that hits the Toggle, the intake + bumper, the inside, side and back panels.
		this.colliders = [
			box(-hl, hl, 0.3, 4, -hw, -hw + 2), box(-hl, hl, 0.3, 4, hw - 2, hw),
			box(-hl, -hl + 1.5, 0.3, 4, -hw + 2, hw - 2),
			box(hl - 2, hl, 4, H, -hw, -hw + 1.5), box(hl - 2, hl, 4, H, hw - 1.5, hw),
			box(-hl, -hl + 2, 4, H, -hw, -hw + 1.5), box(-hl, -hl + 2, 4, H, hw - 1.5, hw),
			box(hl - 1.5, hl, H - 5, H, -hw, hw),
			box(hl - 2.5, hl + 0.8, 0.6, 7.5, -hw + 2, hw - 2),
			box(-hl + 1.5, hl - 2.5, 1.2, 10, -hw + 2, hw - 2),
			box(-hl + 2, hl - 2, 4, 12, -hw, -hw + 1.5), box(-hl + 2, hl - 2, 4, 12, hw - 1.5, hw),
			box(-hl, -hl + 1.5, 4, 10, -hw + 1.5, hw - 1.5),
		];
		// End channel of the claw frame: with the claw at 90 it's what hits the Toggle (checked
		// geometrically in clawTouches: a collider pressed by the turning Toggle flings the robot).
		const cw = hw - 1.6, arm = C.clawArm;
		this.clawSeg = [[0, -arm, -cw - 0.5], [0, -arm, cw + 0.5]];
		// Claw frame colliders that swing with the claw (pins and cups only).
		const clawBox = (hx, hy, hz) => world.createCollider(R.ColliderDesc.cuboid(hx * IN, hy * IN, hz * IN).setDensity(0).setFriction(0)
			.setCollisionGroups(groups(GROUP.claw, 0xffff & ~(GROUP.toggle | GROUP.wall | GROUP.fixed))), this.body);
		this.clawCols = [
			{ c: clawBox(1.0, 0.5, (cw + 0.5 - 2.2) / 2), p: [0, -arm, -(cw + 0.5 + 2.2) / 2] },
			{ c: clawBox(1.0, 0.5, (cw + 0.5 - 2.2) / 2), p: [0, -arm, (cw + 0.5 + 2.2) / 2] },
			{ c: clawBox(0.5, arm / 2, 0.5), p: [0, -arm / 2, -cw] },
			{ c: clawBox(0.5, arm / 2, 0.5), p: [0, -arm / 2, cw] },
		];
		// Goal guide V on the back.
		const g = C.guide, hy = (g.y1 - g.y0) / 2;
		for (const s of [-1, 1]) {
			const dir = new THREE.Vector3(-Math.SQRT1_2, 0, s * Math.SQRT1_2);
			const c = new THREE.Vector3(g.apexX, (g.y0 + g.y1) / 2, 0).addScaledVector(dir, g.armLen / 2);
			const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0), dir);
			this.colliders.push(world.createCollider(R.ColliderDesc.cuboid((g.armLen / 2) * IN, hy * IN, (g.t / 2) * IN)
				.setTranslation(c.x * IN, c.y * IN, c.z * IN).setRotation({ x: q.x, y: q.y, z: q.z, w: q.w })
				.setFriction(0).setFrictionCombineRule(R.CoefficientCombineRule.Min).setDensity(0)
				.setCollisionGroups(groups(GROUP.robot, 0xffff)), this.body));
		}
		this.mesh = this.buildMesh();
		this.scene.add(this.mesh);
	}

	buildMesh() {
		// The old model has no pitch / roll rig: wrap it so the drivetrain's rocking still shows.
		const m = buildIntakeMesh(C), root = new THREE.Group(), tilt = new THREE.Group(), body = new THREE.Group();
		root.add(tilt); tilt.add(body); body.add(m.group);
		this.parts = { ...m, group: root, tilt, body };
		return root;
	}

	frontReach() { return C.length / 2 + 0.8; }

	resetState() {
		super.resetState();
		this.clawDeg = 0; this.clawTarget = C.CLAW_TURN_VERTICAL_DEG; this.clawVel = C.CLAW_TURN_DOWN_VELOCITY; this.clawDir = 0;
		this.spinCmd = 0; this.intakeCmd = 0; this.clawTurnHorizontal = false; this.transferT = 0; this.clawToggle = null;
		this.op = { inverse: 1, inverseScale: 1, intakeRunning: false, intakeReversed: false, cascadeIntake: false, clawIn: false, clawOut: false, r1: 0, clawState: 0, prev: {} };
	}

	// ---- Claw geometry --------------------------------------------------------------------------
	clawArmAngle() { return this.clawDeg * C.clawDegPerMotorDeg * DEG; }   // 0 = hanging down, 90 = pointing back
	pivotY() { return C.pinBottomAtHorizontal + PIN_H - 1 + this.liftIn(); }
	clawPinLocal() {
		const a = this.clawArmAngle(), gx = -6.5 - C.clawArm * Math.sin(a), gy = this.pivotY() - C.clawArm * Math.cos(a);
		const off = PIN_H / 2 - 1, angle = Math.PI / 2 - a;
		const pos = new THREE.Vector3(gx + Math.cos(a) * off, gy - Math.sin(a) * off, 0);
		const lowest = pos.y - Math.abs(Math.cos(angle)) * (PIN_H / 2) - Math.abs(Math.sin(angle)) * 1.58;
		if (lowest < 0.3) pos.y += 0.3 - lowest;
		return { pos, angle };
	}
	clawGripWorld() { const a = this.clawArmAngle(); return this.toWorld(-6.5 - C.clawArm * Math.sin(a), this.pivotY() - C.clawArm * Math.cos(a), 0); }
	canPair() { return !!this.claw && this.claw.kind === 'pin' && !this.clawExtraPiece && this.liftPos() > -(C.CASCADE_INTAKE_DEG + 60); }
	decideFlip(o, angle) {
		const def = new THREE.Vector3(0, 1, 0).applyQuaternion(this.quat().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), angle)));
		const r = o.body.rotation(), act = new THREE.Vector3(0, 1, 0).applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w));
		o.flipped = def.dot(act) < 0;
	}
	clawTouches(tg) {
		const a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = this.pivotY();
		const c = tg.body.translation(), [p0, p1] = this.clawSeg;
		for (let i = 0; i <= 8; i++) {
			const x = p0[0], y = p0[1], z = p0[2] + ((p1[2] - p0[2]) * i) / 8;
			const d = this.toWorld(-6.5 + x * ca + y * sa, py - x * sa + y * ca, z).sub(new THREE.Vector3(c.x, c.y, c.z));
			const along = d.dot(tg.axis);
			if (Math.abs(along) < (TOGGLE.len / 2) * IN && d.addScaledVector(tg.axis, -along).length() < 1.9 * IN) return true;
		}
		return false;
	}
	updateClawColliders() {
		if (this.clawDeg === undefined) return;
		const a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = this.pivotY();
		const rq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -a);
		for (const k of this.clawCols) {
			const [x, y, z] = k.p;
			k.c.setTranslationWrtParent({ x: (-6.5 + x * ca + y * sa) * IN, y: (py - x * sa + y * ca) * IN, z: z * IN });
			k.c.setRotationWrtParent({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		}
	}

	// ---- Mechanisms -------------------------------------------------------------------------
	liftCeiling() { return C.liftMaxIn; }
	cogHeight() { return 4.9 + 0.2 * this.liftIn(); }
	clawGrabToggle() { this.spinCmd = this.spinCmd < 0 ? 0 : -127; }   // auton claw_grab: claw spin out (grabs)
	clawReleaseToggle() { this.spinCmd = this.spinCmd > 0 ? 0 : 127; }
	clawDriverReset() { this.spinCmd = 0; this.intakeCmd = 0; }

	stepMechanisms(dt) {
		super.stepMechanisms(dt);   // cascade (lift task, limits, motors)
		this.clawSpin = this.spinCmd;
		this.clawToggle = this.field.toggles.find((tg) => this.clawTouches(tg)) || null;
		const step = this.clawVel * 6 * dt, e = this.clawTarget - this.clawDeg;
		const move = this.clawToggle && Math.sign(e) === this.clawDir ? 0 : clamp(e, -step, step);
		if (move) this.clawDir = Math.sign(move);
		this.clawDeg += move;
		this.updateClawColliders();
	}

	stepPins(dt) {
		for (const r of this.justReleased || []) if ((r.t -= dt) <= 0 && !r.pin.held && this.field.has(r.pin)) for (let i = 0; i < r.pin.body.numColliders(); i++) r.pin.body.collider(i).setCollisionGroups(0xffffffff);
		if (this.justReleased) this.justReleased = this.justReleased.filter((r) => r.t > 0);
		const hl = C.length / 2;
		// Intake: anything low in front of the rollers goes up the ramp into the hopper.
		if (this.intakeCmd > 30 && !this.hopper && (!this.claw || this.canPair()) && this.clawDeg < 150 && !this.tipped()) {
			let took = false;
			for (const L of this.field.loaders || []) {   // out of a Loader: the bottom piece of its stack
				if (!L.ready || !L.stack?.length || (this.claw && L.stack.every((o) => o.kind !== 'pin'))) continue;
				const l = this.toLocal(W(L.px, L.fy, 0));
				if (l.x < hl - 1 || l.x > hl + 4.5 || Math.abs(l.z) > 4) continue;
				const st = L.stack, o = this.field.takeLoader(L);
				this.decideFlip(o, Math.PI / 2); this.attachPin(o, 'hopper');
				for (const k of st) if (!k.held) this.field.freePiece(k);
				took = true; break;
			}
			for (const pin of took ? [] : [...this.field.pins, ...this.field.cups]) {
				if (pin.held || (this.claw && pin.kind !== 'pin')) continue;
				const f = toField(pin.body.translation());
				if (this.field.goals.some((g) => Math.hypot(f.fx - g.fx, f.fy - g.fy) < 2.5 && f.h > g.base)) continue;   // seated on a post
				const l = this.toLocal(new THREE.Vector3().copy(pin.body.translation()));
				if (l.x > hl - 3 && l.x < hl + 4.5 && Math.abs(l.z) < 5 && l.y < 8) { this.decideFlip(pin, Math.PI / 2); this.attachPin(pin, 'hopper'); break; }
			}
		}
		if (this.intakeCmd < -30 && this.hopper) {   // reversed: spit it out the front
			const p = this.hopper, f = this.fwd().multiplyScalar(1.2);
			p.flipped = false;
			this.releasePin(p, true, { x: f.x, y: 0, z: f.z });
		}
		// Hopper -> claw: the claw (down, spinning the grab way) takes it with the cascade at intake height.
		const pairing = this.hopper && this.hopper.kind === 'pin' && this.canPair();
		if (this.hopper && (!this.claw || pairing) && this.clawSpin < -30 && this.clawDeg < 150 && this.liftPos() > -(C.CASCADE_INTAKE_DEG + 60)) {
			this.transferT += dt;
			if (this.transferT > 0.25) {
				const p = this.hopper; this.hopper = null; p.held = null;
				if (pairing) { this.clawRel = { pos: new THREE.Vector3(0, 0, PAIR_GAP), quat: new THREE.Quaternion() }; this.attachPin(p, 'clawExtraPiece'); }
				else this.attachPin(p, 'claw');
				this.transferT = 0;
			}
		} else this.transferT = 0;
		// Claw swung out and spinning: grabs a piece at the grip (off a goal, a standing pin...).
		if (!this.claw && !this.hopper && this.clawSpin < -30 && this.clawDeg > 100) {
			const g = this.clawGripWorld(), recent = new Set((this.justPlaced || []).map((r) => r.o));
			for (const o of [...this.field.pins, ...this.field.cups]) {
				if (o.held || recent.has(o)) continue;
				const t = o.body.translation(), r = o.body.rotation();
				const ax = new THREE.Vector3(0, 1, 0).applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w));
				const up = Math.abs(ax.y) > 0.8;
				if (!up && Math.abs(ax.y) > 0.5) continue;
				const grab = new THREE.Vector3(t.x, t.y, t.z).addScaledVector(ax, up ? Math.sign(ax.y) * (PIN_H / 2 - 1) * IN : 0);
				if (Math.hypot(grab.x - g.x, grab.z - g.z) < 2 * IN && Math.abs(grab.y - g.y) < 2.5 * IN) { this.decideFlip(o, this.clawPinLocal().angle); this.attachPin(o, 'claw'); this.grabPartner(o); break; }
			}
		}
		// Scoring: claw at 90 over a goal, lowering the lift until the piece seats lets go.
		this.clawHeldT = this.claw && this.claw === this.clawHeldObj ? this.clawHeldT + dt : 0;
		this.clawHeldObj = this.claw;
		if (this.claw && this.clawHeldT > 0.5 && (this.liftRate || 0) < -1 && Math.abs(this.clawDeg - C.CLAW_TURN_HORIZONTAL_DEG) < 200) {
			const o = this.clawBottom(), s = this.field.seatFor(o), h = o.body.translation().y / IN;
			if (s && h - PIN_H / 2 <= s.seatBottom + 0.4) {
				const both = [this.claw, this.clawExtraPiece].filter(Boolean);
				this.releaseClaw(false, true);
				for (const p of both) { p.flipped = false; (this.justPlaced ||= []).push({ o: p, t: 2 }); }
			}
		}
		if (this.justPlaced) this.justPlaced = this.justPlaced.filter((r) => (r.t -= dt) > 0);
		// Spinning the drop way lets go of what's in the claw.
		if (this.claw && this.clawSpin > 30) {
			this.releaseT += dt;
			if (this.releaseT > 0.12) { for (const p of [this.claw, this.clawExtraPiece]) if (p) p.flipped = false; this.releaseClaw(true, true); this.releaseT = 0; }
		} else this.releaseT = 0;
		// Pose what's held: lying on the ramp, or in the claw at its angle.
		const q = this.quat(), Z = new THREE.Vector3(0, 0, 1);
		const pose = (pin, l, angle) => {
			const rq = q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Z, angle));
			pin.body.setNextKinematicTranslation(this.toWorld(l.x, l.y, l.z));
			pin.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		};
		const flip = (o) => (o.flipped ? Math.PI : 0);
		if (this.hopper) pose(this.hopper, new THREE.Vector3(2.5, 4.5, 0), Math.PI / 2 + flip(this.hopper));
		if (this.claw) {
			const l = this.clawPinLocal(); l.angle += flip(this.claw);
			if (this.clawExtraPiece && this.clawExtraPiece.kind === 'pin' && this.claw.kind === 'pin') l.pos.z -= PAIR_GAP / 2;
			pose(this.claw, l.pos, l.angle);
			const e = this.clawExtraPiece, rel = this.clawRel;
			if (e && rel) {
				const mq = q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Z, l.angle)), mp = this.toWorld(l.pos.x, l.pos.y, l.pos.z);
				const p = rel.pos.clone().applyQuaternion(mq).add(mp), rq = mq.clone().multiply(rel.quat);
				e.body.setNextKinematicTranslation(p);
				e.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
			}
		}
	}

	// The claw frame stops at the perimeter wall; pressed on a Toggle it can flip it.
	holdAtToggle() {
		const [p0, p1] = this.clawSeg, a = this.clawArmAngle(), ca = Math.cos(a), sa = Math.sin(a), py = this.pivotY();
		for (const z of [p0[2], 0, p1[2]]) {
			const f = toField(this.toWorld(-6.5 + p0[0] * ca + p0[1] * sa, py - p0[0] * sa + p0[1] * ca, z));
			if (f.h > 12.5) continue;
			const v = this.body.linvel();
			if (Math.abs(f.fx) > WALL && Math.sign(v.x) === Math.sign(f.fx)) this.body.setLinvel({ x: 0, y: 0, z: v.z }, true);
			if (Math.abs(f.fy) > WALL && Math.sign(-v.z) === Math.sign(f.fy)) this.body.setLinvel({ x: this.body.linvel().x, y: 0, z: 0 }, true);
		}
		const tg = this.clawToggle;
		if (!tg) { this.toggleHitV = 0; return; }
		const v = this.body.linvel(), n = tg.nWorld, vn = v.x * n.x + v.z * n.z;
		this.toggleHitV = Math.max(this.toggleHitV || 0, -vn);
		if (vn < 0) this.body.setLinvel({ x: v.x - vn * n.x, y: 0, z: v.z - vn * n.z }, true);
	}

	loadPreload() {
		const pin = this.field.makePin(this.alliance, 'yellow', this.toWorld(0, 10, 0));
		this.attachPin(pin, 'claw');
	}

	syncMesh(dt) {
		const t = this.body.translation(), r = this.body.rotation(), tl = this.drive.tilt(), P = this.parts;
		this.mesh.position.set(t.x, t.y, t.z);
		this.mesh.quaternion.set(r.x, r.y, r.z, r.w);
		P.tilt.position.set(tl.px * IN, 0, tl.pz * IN);
		P.tilt.rotation.set(-tl.roll, 0, tl.pitch, 'ZXY');
		P.body.position.set(-tl.px * IN, 0, -tl.pz * IN);
		for (const s of P.stages) s.g.position.y = this.liftIn() * s.frac * IN;
		P.clawPivot.rotation.z = -this.clawArmAngle();
		const ws = this.wheelSpin || { L: 0, R: 0 };
		for (const w of P.wheels) w.w.rotation.y += ws[w.side] * dt;
		for (const dg of P.driveGears) dg.g.rotation.z -= (ws[dg.side] || 0) * dt;
		const lr = this.liftRate || 0;
		for (const lg of P.liftGears) lg.g.rotation.z += lg.dir * (lr * DEG) * 0.5 * dt;
		for (const led of P.liftLeds) led.material.color.setHex(Math.abs(lr) > 1 ? 0xff3322 : 0x331010);
		for (const rr of P.intakeRollers) rr.rotation.z += (this.intakeCmd / 127) * 25 * dt;
		for (const rr of P.clawRollers) rr.rotation.z += (this.clawSpin / 127) * 25 * dt;
	}

	// ---- HUD / controls page ---------------------------------------------------------------
	clawHud() {
		const pos = ['VERTICAL', 'HIGHEST', 'LIFT', '90°'][this.op.clawState];
		const spin = this.clawSpin < -30 ? 'spinning in' : this.clawSpin > 30 ? 'spinning out (drop)' : 'stopped';
		const intake = this.intakeCmd > 30 ? 'INTAKE ON · ' : this.intakeCmd < -30 ? 'INTAKE REVERSED · ' : '';
		return [this.clawSpin < -30 || this.intakeCmd > 30 ? 'grab' : this.clawSpin > 30 || this.intakeCmd < -30 ? 'release' : 'stopped', `${intake}CLAW ${pos} · ${spin}`];
	}
	loaderHint() { return this.hopper ? 'hopper full' : 'intake on (R1), drive the intake in'; }
	driveName() { return this.op.inverse > 0 ? 'intake first' : 'claw first'; }
	dirTag() { return this.op.inverse < 0 ? ' · CLAW FIRST' : ''; }
	heldText(name) {
		const h = this.hopper ? 'hopper ' + name(this.hopper) : '';
		return [h, super.heldText(name)].filter((s) => s && s !== '—').join(' · ') || '—';
	}
	static controlRows(k, x) {
		return [
			['Intake cycle', k('R1'), k('RB'), k('E'), '1st: intake + claw grab spin + cascade to intake height · 2nd: claw spin only · 3rd: off. Drive intake first over a pin or cup; it goes up the ramp and the claw takes it'],
			['Reverse intake + claw spin', k('R2'), k('RT'), k('Q'), 'Toggle: spits out of the intake and lets go of the claw'],
			['Claw flip', k('B'), k(x.B), k('B'), '1st highest (1350) · 2nd lift (450) · 3rd 90° (900, the pin hangs over the goal) · 4th down'],
			['Claw spin in / drop pin', k('A'), k(x.A), k('F'), 'Toggle. Spinning this way lets go of the pin'],
			['Claw spin out', k('Y'), k(x.Y), k('Y'), 'Toggle. With the claw at 90° it grabs a pin at the grip'],
			['Claw straight to 90° / back down', k('X'), k(x.X), k('X'), 'One press flips to 90°, again back to rest. Score: claw at 90° over a goal, lower the lift'],
			['Take a match load', k('R1'), k('RB'), k('E'), 'Intake on, drive the intake into the Loader slowly: it takes the bottom piece'],
			['Flip drive direction', k('↑'), 'D-pad ↑', k('↑'), 'Starts intake first; turning unchanged'],
		];
	}

	// ---- Driver control: the old opcontrol.cpp (10/15) ---------------------------------------
	opcontrol(c, hooks = {}) {
		const o = this.op, newPress = (k) => c[k] && !o.prev[k];
		if (this.tipped()) { this.leftCmd = this.rightCmd = 0; this.cascadeMove(0); o.prev = { ...c }; return; }
		if (newPress('Down') && hooks.practiceAuton) { o.prev = { ...c }; hooks.practiceAuton(); return; }
		if (newPress('Left')) hooks.side?.('left');
		if (newPress('Right')) hooks.side?.('right');
		if (newPress('Up')) { o.inverse = -o.inverse; this.flash?.(o.inverse < 0 ? 'CLAW FIRST' : 'INTAKE FIRST'); }
		o.inverseScale = o.inverseScale < o.inverse ? Math.min(o.inverseScale + 0.25, o.inverse) : Math.max(o.inverseScale - 0.25, o.inverse);
		let dir = c.ly, turn = c.rx;
		if (Math.abs(dir) < 10) dir = 0;
		if (Math.abs(turn) < 10) turn = 0;
		dir = Math.round(dir * o.inverseScale);
		this.leftCmd = clamp(dir + turn, -127, 127);
		this.rightCmd = clamp(dir - turn, -127, 127);
		const estop = c.L1 && c.R1;
		if (newPress('R1') && !estop) {
			o.r1 = (o.r1 + 1) % 3;
			o.intakeRunning = o.r1 === 1; o.intakeReversed = false;
			o.clawOut = o.r1 !== 0; o.clawIn = false;
			o.cascadeIntake = o.r1 === 1;
			if (o.cascadeIntake) this.liftTo(-C.CASCADE_INTAKE_DEG);
		}
		if (newPress('R2') && !estop) { o.intakeReversed = !o.intakeReversed; o.intakeRunning = false; o.clawIn = false; o.clawOut = false; }
		this.intakeCmd = o.intakeReversed ? -127 : o.intakeRunning ? 127 : 0;
		if (estop) { o.cascadeIntake = false; this.liftRelease(); this.cascadeMove(0); }
		else if (c.L1) { o.cascadeIntake = false; this.liftRelease(); this.cascadeMove(127); }
		else if (c.L2) { o.cascadeIntake = false; this.liftRelease(); this.cascadeMove(-85); }
		else if (!o.cascadeIntake) { this.liftRelease(); this.cascadeMove(0); }
		if (newPress('A')) { o.clawIn = !o.clawIn; o.clawOut = false; }
		if (newPress('Y')) { o.clawOut = !o.clawOut; o.clawIn = false; }
		this.spinCmd = o.intakeReversed ? 127 : o.clawIn ? 45 : o.clawOut ? -127 : 0;
		if (newPress('B')) {
			o.clawState = (o.clawState + 1) % 4;
			this.clawTarget = [C.CLAW_TURN_VERTICAL_DEG, C.CLAW_TURN_HIGHEST_DEG, C.CLAW_TURN_LIFT_DEG, C.CLAW_TURN_HORIZONTAL_DEG][o.clawState];
			this.clawTurnHorizontal = o.clawState !== 0;
		}
		if (newPress('X')) {
			o.clawState = o.clawState !== 3 ? 3 : 0;
			this.clawTarget = o.clawState === 3 ? C.CLAW_TURN_HORIZONTAL_DEG : C.CLAW_TURN_VERTICAL_DEG;
			this.clawTurnHorizontal = o.clawState !== 0;
		}
		this.clawVel = this.clawTurnHorizontal ? C.CLAW_TURN_VELOCITY : C.CLAW_TURN_DOWN_VELOCITY;
		o.prev = { ...c };
	}

	afterAutonTest() {
		const o = this.op;
		o.clawState = 0; this.clawTarget = C.CLAW_TURN_VERTICAL_DEG; this.clawTurnHorizontal = false;
		o.intakeRunning = o.intakeReversed = o.cascadeIntake = o.clawIn = o.clawOut = false; o.r1 = 0;
		this.spinCmd = 0; this.intakeCmd = 0; this.liftRelease();
	}
}
