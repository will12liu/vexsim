// The DR4B robot (from the team's Onshape CAD, "main" assembly): 6-wheel drive (2.75" omni,
// traction, omni a side; 4 blue motors, 36T -> 48T = 450 rpm), a double reverse four-bar lift
// (2 green motors through 12T -> 60T, rotation sensor on the arm axle) carrying the flex-wheel
// claw at the back, and a pneumatic gear pincer on the front that clamps one cup / pin stack.
// No robot code yet, so the controls follow 4787T's opcontrol (L1 / L2 lift, R1 grab, R2 / A
// release) with B for the pincer. Robot-local inches: +x = front (pincer), +y up, +z right.
import * as THREE from 'three';
import { IN, PIN_H, CUP_H, GROUP, groups } from './field.js';
import { Robot, CFG } from './robot.js';
import { buildDr4bMesh } from './dr4bmodel.js';

const DEG = Math.PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const upright = (o) => { const r = o.body.rotation(); return Math.abs(1 - 2 * (r.x * r.x + r.z * r.z)) > 0.85; };

// Lift geometry (measured off the CAD renders): arms pivot on the top of the towers; the claw's
// mount sits END_REST off the floor with the lift down and rises 2 * ARM * (sin th - sin th0).
const ARM = 12, PIVOT_Y = 15.8, END_REST = 4.5, BAR_GAP = 1.5;
const TH0 = Math.asin((END_REST - PIVOT_Y) / (2 * ARM));   // about -28 deg: arms angled down at rest

export const DCFG = {
	...CFG,
	length: 14.6, width: 13.7, height: 17.6, mass: 7.0,
	trackWidth: 11.3, wheelDia: 2.75, wheelbase: 9.85,
	driveRpm: 450, motorRpm: 600,                         // blue motors, 36T -> 48T
	wheelX: [[-5.1, 'omni'], [-1.7, 'traction'], [4.75, 'omni']], motorX: [-3.4, 1.5],
	// The middle traction wheels grip sideways: much harder to shove than the all-omni robots.
	driveOpts: { muLat: 0.36, slop: 0.05 * IN },
	backBarX: -3.4,                                       // crossbar in front of the claw (the back is open)
	clawIn: 1.0, clawWheelR: 1.0, clawTop: 7.4,
	claw: { h: 5.0, housingR: 2.35, rollerZ: 1.99 },
	dr4b: { towerX: -2.2, towerZ: 4.6, pivotY: PIVOT_Y, arm: ARM, barGap: BAR_GAP, stage1Z: 5.15, stage2Z: 4.1, endZ: 3.6, endY: END_REST, th0: TH0 },
	// Pincer: ring centre past the front, jaw pivots just behind it, plates at two heights.
	pincer: { pivotZ: 0.95, ri: 1.68, ro: 2.35, plateY: [1.0, 3.0], openRad: 0.55, closeS: 0.12 },
	// Lift: rotation sensor on the arm axle, degrees of arm swing (reads negative going up, like lift_rot).
	LIFT_LIMIT_BOTTOM_DEG: 1, LIFT_LIMIT_TOP_DEG: -88, LIFT_LIMIT_SLOW_DEG: 6, LIFT_LIMIT_MIN_POWER: 30,
	LIFT_HOLD_BAND_DEG: 1.5, LIFT_RECORRECT_DEG: 3.5, LIFT_UP_MIN_POWER: 40, LIFT_DOWN_MIN_POWER: 25, LIFT_DOWN_MAX_POWER: 90, LIFT_KP: 12,
	R1_LIFT_ROT: -1,                                      // R1: lift all the way down (the claw grabs off the floor)
	liftDegPerSec: 170,                                   // 2 green motors, 5:1: 240 deg/s free, loaded
};
const C = DCFG, D = C.dr4b, P = C.pincer;
C.liftMaxDeg = -C.LIFT_LIMIT_TOP_DEG + 2;
C.liftMaxIn = 2 * ARM * (Math.sin(TH0 + C.liftMaxDeg * DEG) - Math.sin(TH0));
C.liftInPerDeg = C.liftMaxIn / C.liftMaxDeg;          // average, for code that wants a rough number
C.clawX = -C.length / 2 + C.clawIn;
C.claw.rollerX = C.clawX - 1.1;
P.x = C.length / 2 + 2.3; P.pivotX = C.length / 2 + 0.4;

export class Dr4bBot extends Robot {
	get C() { return DCFG; }

	build() {
		const { R, world } = this;
		const L = C.length, Wd = C.width, hl = L / 2, hw = Wd / 2;
		const m = C.mass, I = (m * ((L * IN) ** 2 + (Wd * IN) ** 2)) / 12 * 0.85;
		this.body = world.createRigidBody(R.RigidBodyDesc.dynamic().setCanSleep(false).setLinearDamping(0).setAngularDamping(0)
			.setAdditionalMassProperties(m, { x: 0, y: 0.15, z: 0 }, { x: I, y: I, z: I }, { x: 0, y: 0, z: 0, w: 1 }));
		this.body.setEnabledRotations(false, true, false, true);
		this.body.setEnabledTranslations(true, false, true, true);
		const box = (x0, x1, y0, y1, z0, z1) => world.createCollider(R.ColliderDesc.cuboid(((x1 - x0) / 2) * IN, ((y1 - y0) / 2) * IN, ((z1 - z0) / 2) * IN)
			.setTranslation(((x0 + x1) / 2) * IN, ((y0 + y1) / 2) * IN, ((z0 + z1) / 2) * IN)
			.setFriction(0.2).setFrictionCombineRule(R.CoefficientCombineRule.Min).setDensity(0)
			.setCollisionGroups(groups(GROUP.robot, 0xffff)), this.body);
		const TX = D.towerX, bx = C.backBarX, side = hw - 1.9;
		this.colliders = [
			box(-hl, hl, 0.3, 3.2, -hw, -side), box(-hl, hl, 0.3, 3.2, side, hw),        // drive rails + wheels
			box(hl - 1.0, hl, 0.3, 3.2, -side, side),                                    // front crossbar
			box(bx - 0.5, bx + 0.5, 0.3, 3.2, -side, side),                              // crossbar in front of the claw
			box(bx + 0.5, hl - 1.0, 0.8, 4.0, -side, side),                              // Brain, battery, air tank
			box(TX - 0.6, TX + 0.6, 3.2, D.pivotY + 0.8, -D.towerZ - 0.5, -D.towerZ + 0.5),   // lift towers
			box(TX - 0.6, TX + 0.6, 3.2, D.pivotY + 0.8, D.towerZ - 0.5, D.towerZ + 0.5),
		];
		// Claw (rides with the lift; touches pins, cups and robots only): the roller columns at its back
		// corners and the housing around the front half of the stack.
		const cg = groups(GROUP.claw, 0xffff & ~(GROUP.toggle | GROUP.wall | GROUP.fixed));
		const col = (desc) => world.createCollider(desc.setDensity(0).setCollisionGroups(cg), this.body);
		const X = C.clawX, cy = 0.4 + C.claw.h / 2;
		this.clawCols = [];
		for (const s of [-1, 1]) this.clawCols.push({ c: col(R.ColliderDesc.cylinder((C.claw.h / 2) * IN, 0.6 * IN).setFriction(0.8)), p: [C.claw.rollerX, cy, s * C.claw.rollerZ], roller: true });
		this.clawCols.push({ c: col(R.ColliderDesc.cuboid(0.2 * IN, (C.claw.h / 2) * IN, 2.3 * IN).setFriction(0.3)), p: [X + 2.15, cy, 0] });
		for (const s of [-1, 1]) this.clawCols.push({ c: col(R.ColliderDesc.cuboid(1.0 * IN, (C.claw.h / 2) * IN, 0.2 * IN).setFriction(0.3)), p: [X + 0.9, cy, s * 2.45] });
		// Pincer jaws (pins and cups only): swing out when open.
		this.jawCols = [-1, 1].map((s) => ({ s, c: col(R.ColliderDesc.cuboid(1.3 * IN, 1.1 * IN, 0.33 * IN).setFriction(0.5)) }));
		this.mesh = this.buildMesh();
		this.scene.add(this.mesh);
	}

	buildMesh() {
		const m = buildDr4bMesh(C);
		this.parts = m;
		return m.group;
	}

	resetState() {
		super.resetState();
		this.pincer = null; this.pincerExtra = null; this.pincerHold = null;
		this.pincerClosed = false; this.pincerT = 0;   // 0 open .. 1 closed
	}

	// ---- Lift: DR4B arm angle from the rotation sensor ------------------------------------------
	armAngle() { return D.th0 + this.liftDeg * DEG; }
	liftIn() { return clamp(2 * D.arm * (Math.sin(this.armAngle()) - Math.sin(D.th0)), 0, C.liftMaxIn + 0.5); }
	liftDegFor(inches) { return (Math.asin(clamp(inches / (2 * D.arm) + Math.sin(D.th0), -1, 1)) - D.th0) / DEG; }
	// Tall and top-heavy with the lift up; the claw's load rides at the end of the arms.
	cogHeight() { return 6.2 + 0.28 * this.liftIn() + (this.claw ? 0.03 * (this.liftIn() + 6) : 0); }
	// The claw's mouth: the roller columns stick out behind the frame.
	mouthBack() { return C.clawX - 3.0; }
	frontReach() { return P.x + P.ro; }
	heldPieces() { return [...super.heldPieces(), this.pincerExtra, this.pincer]; }

	// ---- Pincer (pneumatic) ----------------------------------------------------------------
	togglePincer() {
		this.pincerClosed = !this.pincerClosed;
		if (this.pincerClosed) this.pincerGrab();
		else this.pincerRelease();
	}
	// Closing on a standing stack in the jaws clamps it (and whatever is stacked with it).
	pincerGrab() {
		let best = null, bl = null;
		for (const o of [...this.field.pins, ...this.field.cups]) {
			if (o.held || !upright(o) || this.field.onGoal(o)) continue;
			const l = this.toLocal(new THREE.Vector3().copy(o.body.translation()));
			const bottom = l.y - (o.kind === 'cup' ? CUP_H : PIN_H) / 2;
			if (Math.abs(l.z) > 1.1 || Math.abs(l.x - P.x) > 1.1 || bottom > 1.5) continue;
			if (!best || l.y < bl.y) { best = o; bl = l; }
		}
		if (!best) return;
		const q = this.quat().invert(), rel = (o) => {
			const t = o.body.translation(), r = o.body.rotation(), l = this.toLocal(new THREE.Vector3(t.x, t.y, t.z));
			return { pos: new THREE.Vector3(P.x, l.y, 0), raw: l, q: q.clone().multiply(new THREE.Quaternion(r.x, r.y, r.z, r.w)) };
		};
		const partner = this.field.partnerOf(best);
		const hb = rel(best);
		this.pincerHold = { main: hb };
		this.attachPin(best, 'pincer');
		if (partner && !partner.held && upright(partner)) {
			const hp = rel(partner);
			hp.pos = new THREE.Vector3(P.x + (hp.raw.x - hb.raw.x), hp.raw.y, hp.raw.z - hb.raw.z);   // keep it where it sits on / in the main piece
			this.pincerHold.extra = hp;
			this.attachPin(partner, 'pincerExtra');
		}
	}
	pincerRelease() {
		for (const p of [this.pincerExtra, this.pincer]) if (p) this.releasePin(p, true);
		this.pincerHold = null;
	}
	onTippedExtra() { this.pincerRelease(); }

	stepMechanisms(dt) {
		super.stepMechanisms(dt);
		const want = this.pincerClosed ? 1 : 0;
		this.pincerT += clamp(want - this.pincerT, -dt / P.closeS, dt / P.closeS);
		const open = P.openRad * (1 - this.pincerT);
		for (const j of this.jawCols || []) {
			const s = j.s, a = -s * open;   // jaw yaw about its pivot (front tips swing outward when open)
			const lx = P.x - P.pivotX + 0.2, lz = -s * P.pivotZ + s * (P.ri + P.ro) / 2;   // side of the ring, pivot frame
			const x = P.pivotX + lx * Math.cos(a) + lz * Math.sin(a), z = s * P.pivotZ - lx * Math.sin(a) + lz * Math.cos(a);
			const rq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), a);
			j.c.setTranslationWrtParent({ x: x * IN, y: 2.0 * IN, z: z * IN });
			j.c.setRotationWrtParent({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		}
	}

	stepPins(dt) {
		super.stepPins(dt);
		const h = this.pincerHold;
		if (!h) return;
		const q = this.quat();
		const pose = (o, k) => {
			if (!o || !k) return;
			const rq = q.clone().multiply(k.q);
			o.body.setNextKinematicTranslation(this.toWorld(k.pos.x, k.pos.y, k.pos.z));
			o.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		};
		pose(this.pincer, h.main); pose(this.pincerExtra, h.extra);
	}

	syncMesh(dt) {
		super.syncMesh(dt);
		const P2 = this.parts, th = this.armAngle(), TX = D.towerX;
		const mx = TX + D.arm * Math.cos(th), my = D.pivotY + D.arm * Math.sin(th);
		for (const a of P2.arms) {
			const dy = -a.i * D.barGap;
			if (a.stage === 1) { a.g.position.set(TX * IN, (D.pivotY + dy) * IN, a.g.position.z); a.g.rotation.z = th; }
			else { a.g.position.set(mx * IN, (my + dy) * IN, a.g.position.z); a.g.rotation.z = Math.PI - th; }
		}
		for (const m of P2.mids) {
			m.g.position.x = mx * IN; m.g.position.y = my * IN;
			m.gears[0].rotation.z = th * 1.0; m.gears[1].rotation.z = -th * 1.0;
		}
		const open = P.openRad * (1 - this.pincerT);
		for (const j of P2.jaws) j.g.rotation.y = -j.s * open;
		if (P2.pincerRod) P2.pincerRod.position.z = (-0.75 + 0.6 * (1 - this.pincerT)) * IN;
	}

	// ---- HUD / controls --------------------------------------------------------------------
	clawHud() {
		const [mode, text] = super.clawHud();
		return [mode, `${text} · PINCER ${this.pincerClosed ? 'CLOSED' : 'OPEN'}`];
	}
	heldText(name) {
		const p = this.pincer ? 'pincer ' + name(this.pincer) + (this.pincerExtra ? ' + ' + name(this.pincerExtra) : '') : '';
		return [super.heldText(name), p].filter((s) => s && s !== '—').join(' · ') || '—';
	}
	static hasMacro = false;   // B is the pincer
	static controlRows(k, x) {
		return [
			['DR4B lift', k('L1') + ' / ' + k('L2'), k('LB') + ' / ' + k('LT'), k('R') + ' / ' + k('G'), 'No robot code yet: same buttons as 4787T. Drive power drops with the lift up (anti-tip); it is top-heavy, so ease off before stopping'],
			['Claw grab spin', k('R1'), k('RB'), k('E'), 'Toggle; also drops the lift to the floor. Back the claw SLOWLY into a standing stack: the flex wheels pull it in and stop by themselves'],
			['Claw release spin', k('R2') + ' / ' + k('A'), k('RT') + ' / ' + k(x.A), k('Q') + ' / ' + k('F'), 'Toggle. Pushes the stack out the back; lowering it onto a goal also lets go'],
			['Pincer open / close', k('B'), k(x.B), k('B'), 'Pneumatic. Drive the front over a standing cup / pin stack, close: it is clamped and carried. Open to drop it'],
			['Take a match load', k('R1'), k('RB'), k('E'), 'Lift down, back the claw into the Loader square and slow'],
			['Flip drive direction', k('↑'), 'D-pad ↑', k('↑'), 'Starts claw first; turning unchanged'],
		];
	}

	opcontrol(c, hooks = {}) {
		const pincerPress = c.B && !this.op.prev.B;
		super.opcontrol(c, { ...hooks, macro: null });
		if (pincerPress && !this.tipped()) { this.togglePincer(); this.flash?.(this.pincerClosed ? 'PINCER CLOSED' : 'PINCER OPEN'); }
	}
}
