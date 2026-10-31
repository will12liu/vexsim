// AI robots for 2v2. They drive through the same drivetrain physics as the Titanium robot (motors,
// gear slop, omni slide, tipping), plan paths around goals and robots (A* on a 3" grid, then pure
// pursuit), and pick what to do by points per second: stacking with the real scoring rules (which
// halves end up visible, who owns the quadrant's yellow), match-load cycles from their Loaders,
// Toggle control, defense, and the Midfield in the last 10 s. Each strategy is its own kind of robot.
import * as THREE from 'three';
import { IN, W, WALL, toField, PIN_H, CUP_H, PIN_COLLAR, COLORS, GROUP, groups, TOGGLE, LOADER, MID, quadrantOf } from './field.js';
import { Robot, CFG } from './robot.js';

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const DEG = Math.PI / 180;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const HL = CFG.length / 2, HW = CFG.width / 2;
const HOLD_OUT = 5.2;   // the grip sits this far past the bumper (front intake or back claw), clear of a goal's base

export const STRATEGIES = {
	scorer: { name: 'Scorer', style: 'intake' },              // picks up pins and cups anywhere, stacks them where they're worth most
	stacker: { name: 'Stacker', style: 'cascade' },           // match-load cycles: cup + pin stacks from its Loaders onto goals
	attacker: { name: 'Toggle attacker', style: 'wedge' },    // flips Toggles to its colour (theirs first), then defends them
	defender: { name: 'Toggle defender', style: 'dr4b' },     // keeps its Toggles, scores when they're safe
	pusher: { name: 'Pusher (defense)', style: 'wedge' },     // shoves opponents, mostly the one carrying a stack
};
const CAN_HOLD = { intake: true, cascade: true, dr4b: true, wedge: false };
const COMMITTED = new Set(['ram', 'back', 'settle', 'final', 'lower', 'leave']);

// ---- Look ---------------------------------------------------------------------------------------
function aiMesh(style) {
	const root = new THREE.Group(), tilt = new THREE.Group(), g = new THREE.Group(); root.add(tilt); tilt.add(g);
	const s = IN;
	const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, metalness: 0.4, ...o });
	const metal = mat(0xb8bcc2, { metalness: 0.85, roughness: 0.32 }), dark = mat(0x1d1f23), bump = mat(0x888888, { metalness: 0.1 });
	const grey = mat(0x9ca1a7, { metalness: 0.1, roughness: 0.75 }), team = mat(0x2c62c9, { metalness: 0.55, roughness: 0.35 });
	const box = (w, h, d, x, y, z, m, parent = g) => { const b = new THREE.Mesh(new THREE.BoxGeometry(w * s, h * s, d * s), m); b.position.set(x * s, y * s, z * s); b.castShadow = true; parent.add(b); return b; };
	const bar = (a, b, w, m, parent = g) => { const dx = b[0] - a[0], dy = b[1] - a[1], o = box(Math.hypot(dx, dy), w, w, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2], m, parent); o.rotation.z = Math.atan2(dy, dx); return o; };
	const L = CFG.length, Wd = CFG.width, hl = L / 2, hw = Wd / 2;
	// Drive base: rails, crossbars, motors, 4 wheels.
	for (const z of [-hw + 0.6, hw - 0.6]) box(L, 2, 1, 0, 1.6, z, metal);
	box(1, 2, Wd - 1, hl - 0.5, 1.6, 0, metal); box(1, 2, Wd - 1, -hl + 0.5, 1.6, 0, metal);
	box(L - 4, 0.2, Wd - 4, 0, 0.9, 0, metal);
	box(5, 1.2, 4, 2, 2.0, 0, dark);   // Brain
	const wheels = [];
	for (const x of [-hl + 2.5, hl - 2.5]) for (const z of [-hw + 1.9, hw - 1.9]) {
		const p = new THREE.Group(); p.position.set(x * s, 1.625 * s, z * s); g.add(p);
		const w = new THREE.Mesh(new THREE.CylinderGeometry(1.625 * s, 1.625 * s, 1.1 * s, 18), dark); w.rotation.x = Math.PI / 2; w.castShadow = true; p.add(w);
		for (let k = 0; k < 8; k++) { const a = (k / 8) * Math.PI * 2, r = new THREE.Mesh(new THREE.BoxGeometry(0.35 * s, 0.35 * s, 1.15 * s), grey); r.position.set(Math.cos(a) * 1.5 * s, Math.sin(a) * 1.5 * s, 0); p.add(r); }
		wheels.push({ w: p, side: z < 0 ? 'L' : 'R' });
		box(2.4, 2.0, 1.8, x * 0.55, 3.0, z * 0.7, dark);   // motor
	}
	for (const z of [-hw - 0.05, hw + 0.05]) { const p = box(5, 2, 0.1, 0, 4.4, z, bump); void p; }   // license plates (alliance colour)
	const stages = [], top = new THREE.Group(); g.add(top);
	if (style === 'intake') {
		// Front roller intake, two-stage lift with a little claw up top.
		for (const y of [2.2, 5.0]) { const r = new THREE.Mesh(new THREE.CylinderGeometry(0.9 * s, 0.9 * s, (Wd - 3) * s, 12), mat(0x30343a)); r.rotation.x = Math.PI / 2; r.position.set((hl + 1.2) * s, y * s, 0); g.add(r); }
		for (const z of [-hw + 1.5, hw - 1.5]) { bar([hl - 2, 2, z], [hl + 1.2, 6, z], 0.8, team); box(1, 13, 1, -hl + 3, 8, z, team); box(1, 12, 1, -hl + 3.5, 9, z * 0.85, team, top); }
		box(3, 1, Wd - 4, -hl + 3.5, 15.5, 0, team, top); box(2, 3, 5, hl + 2, 3, 0, dark, top);
		stages.push({ g: top, frac: 1 });
	} else if (style === 'cascade') {
		for (let k = 0; k < 3; k++) {
			const st = k === 2 ? top : new THREE.Group(); if (k < 2) g.add(st);
			stages.push({ g: st, frac: k / 2 });
			for (const z of [-hw + 1.4 + k * 0.5, hw - 1.4 - k * 0.5]) box(2, 18 - k, 1, -1 + k * 0.3, 11.5 + k * 0.5, z, team, st);
		}
		box(1.2, 1, Wd - 4, -0.6, 21, 0, team, top);
		box(6, 0.8, 8, -hl - 1, 7.3, 0, team, top);
		for (const z of [-2.4, 2.4]) for (const y of [1.6, 3.4, 5.2]) { const w = new THREE.Mesh(new THREE.CylinderGeometry(1.2 * s, 1.2 * s, 1 * s, 16), grey); w.position.set((-hl - HOLD_OUT + 0.3) * s, y * s, z * s); w.castShadow = true; top.add(w); }
	} else if (style === 'wedge') {
		const plow = mat(0x9aa1a8, { metalness: 0.9, roughness: 0.25 });
		for (const z of [-1, 1]) bar([hl - 3, 6.5, z * (hw - 1)], [hl + 1.5, 1, z * (hw - 1)], 0.8, metal);
		const face = box(0.4, 6.5, Wd + 0.6, hl - 0.6, 3.6, 0, plow); face.rotation.z = -0.65;
		box(L - 6, 1, Wd - 4, -1, 6.5, 0, team);
		box(1, 9, 1, -hl + 3, 11, 0, metal); box(1.5, 7, Wd - 3, -hl + 3, 13.5, 0, team);   // Toggle bar up top
		for (const z of [-hw + 0.6, hw - 0.6]) box(L, 1.2, 1.2, 0, 4.4, z, bump);
		stages.push({ g: top, frac: 1 });
	} else {
		const red = mat(0xc8202a, { roughness: 0.45 });
		for (const z of [-hw + 1.4, hw - 1.4]) {
			box(1, 8, 1, -hl + 4, 5, z, metal); box(1, 8, 1, hl - 4, 5, z, metal);
			bar([-hl + 4, 8, z], [hl - 4, 15, z], 0.8, red, top); bar([-hl + 4, 15, z], [hl - 4, 8, z], 0.8, red, top);
			bar([-hl + 4, 15, z], [hl - 4, 22, z], 0.8, red, top); bar([-hl + 4, 22, z], [hl - 4, 15, z], 0.8, red, top);
		}
		box(L - 6, 1, Wd - 2, 0, 22.5, 0, metal, top);
		for (const z of [-2.5, 2.5]) box(1.2, 5, 1, -hl - HOLD_OUT + 0.6, 4, z, dark, top);
		stages.push({ g: top, frac: 1 });
	}
	return { group: root, tilt, body: g, wheels, stages, bump };
}

// ---- Navigation: A* on a 3" grid ---------------------------------------------------------------
const CELL = 3, NG = Math.ceil((2 * WALL) / CELL);
const cellOf = (v) => clamp(Math.floor((v + WALL) / CELL), 0, NG - 1);
const centerOf = (i) => -WALL + (i + 0.5) * CELL;
function plan(from, to, cost) {
	const sx = cellOf(from.fx), sy = cellOf(from.fy), tx = cellOf(to.fx), ty = cellOf(to.fy);
	const N = NG * NG, g = new Float32Array(N).fill(Infinity), came = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
	const h = (x, y) => Math.hypot(x - tx, y - ty);
	const open = [[h(sx, sy), sx + sy * NG]];
	g[sx + sy * NG] = 0;
	const push = (f, i) => { open.push([f, i]); let k = open.length - 1; while (k > 0) { const p = (k - 1) >> 1; if (open[p][0] <= open[k][0]) break; [open[p], open[k]] = [open[k], open[p]]; k = p; } };
	const pop = () => { const top = open[0], last = open.pop(); if (open.length) { open[0] = last; let k = 0; for (;;) { const l = 2 * k + 1, r = l + 1; let m = k; if (l < open.length && open[l][0] < open[m][0]) m = l; if (r < open.length && open[r][0] < open[m][0]) m = r; if (m === k) break; [open[m], open[k]] = [open[k], open[m]]; k = m; } } return top; };
	open.length = 0; push(h(sx, sy), sx + sy * NG);
	let found = false, iter = 0;
	while (open.length && iter++ < 6000) {
		const [, i] = pop();
		if (closed[i]) continue;
		closed[i] = 1;
		const x = i % NG, y = (i / NG) | 0;
		if (x === tx && y === ty) { found = true; break; }
		for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
			if (!dx && !dy) continue;
			const nx = x + dx, ny = y + dy;
			if (nx < 0 || ny < 0 || nx >= NG || ny >= NG) continue;
			const j = nx + ny * NG;
			if (closed[j]) continue;
			const c = cost(nx, ny, nx === tx && ny === ty);
			if (c === Infinity) continue;
			const ng = g[i] + Math.hypot(dx, dy) * (1 + c);
			if (ng < g[j]) { g[j] = ng; came[j] = i; push(ng + h(nx, ny), j); }
		}
	}
	if (!found) return null;
	const pts = [];
	for (let i = tx + ty * NG; i >= 0; i = came[i]) pts.push({ fx: centerOf(i % NG), fy: centerOf((i / NG) | 0) });
	pts.reverse();
	pts[pts.length - 1] = { fx: to.fx, fy: to.fy };
	return pts;
}

export class AIRobot extends Robot {
	constructor(R, world, scene, field, alliance, label) {
		super(R, world, scene, field);
		this.alliance = alliance; this.label = label;
		this.ai = { task: null, path: null, planT: -9, stuckT: 0, lastP: null, ban: new Map(), t: 0, phase: '', decideT: -9 };
		this.skill = 0.8;
	}

	frontReach() { return HL; }

	// Solid body (no claw mouth): the AI's manipulators reach out past the bumper.
	build() {
		const { R, world } = this;
		const m = CFG.mass, I = (m * ((CFG.length * IN) ** 2 + (CFG.width * IN) ** 2)) / 12 * 0.85;
		this.body = world.createRigidBody(R.RigidBodyDesc.dynamic().setCanSleep(false)
			.setAdditionalMassProperties(m, { x: 0, y: 0.15, z: 0 }, { x: I, y: I, z: I }, { x: 0, y: 0, z: 0, w: 1 }));
		this.body.setEnabledRotations(false, true, false, true);
		this.body.setEnabledTranslations(true, false, true, true);
		const box = (x0, x1, y0, y1, z0, z1) => world.createCollider(R.ColliderDesc.cuboid(((x1 - x0) / 2) * IN, ((y1 - y0) / 2) * IN, ((z1 - z0) / 2) * IN)
			.setTranslation(((x0 + x1) / 2) * IN, ((y0 + y1) / 2) * IN, ((z0 + z1) / 2) * IN).setFriction(0.2).setFrictionCombineRule(R.CoefficientCombineRule.Min).setDensity(0)
			.setCollisionGroups(groups(GROUP.robot, 0xffff)), this.body);
		this.colliders = [box(-HL, HL, 0.3, 7, -HW, HW), box(HL - 1.5, HL, 7, CFG.height, -HW, HW), box(-HL, -HL + 2, 7, 14, -HW, HW)];
		this.clawCols = [];
		this.mesh = this.buildMesh();
		this.scene.add(this.mesh);
	}

	buildMesh() {
		const m = aiMesh(this.style || 'intake');
		this.parts = { tilt: m.tilt, body: m.body, wheels: m.wheels, stages: m.stages, driveGears: [], liftGears: [], liftLeds: [], clawRollers: [], clawGears: [] };
		this.bumpMat = m.bump;
		return m.group;
	}

	paint() { this.bumpMat.color.setHex(COLORS[this.alliance]); }

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
		}
		this.holdX = this.style === 'intake' ? HL + HOLD_OUT : -(HL + HOLD_OUT);   // where held pieces sit
		this.paint();
		this.resetState();
	}

	resetState() {
		super.resetState();
		this.liftMode = 'pos'; this.liftTarget = 0; this.liftVelCmd = 260; this.brake = 'brake';
		if (this.ai) Object.assign(this.ai, { task: null, path: null, planT: -9, stuckT: 0, lastP: null, phase: '', t: 0, hits: 0, decideT: -9 });
	}

	// ---- Holding pieces ------------------------------------------------------------------------
	holdPose() { return { x: this.holdX, y: this.liftIn() + (this.hold?.y ?? 0) }; }
	aiGrab(o) {
		const t = o.body.translation(), l = this.toLocal(new THREE.Vector3(t.x, t.y, t.z));
		this.hold = { y: l.y - this.liftIn(), q: this.quat().invert().multiply(new THREE.Quaternion().copy(o.body.rotation())) };
		this.attachPin(o, 'claw');
		this.grabPartner(o);
	}
	loadPreload() {
		const pin = this.field.makePin(this.alliance, 'yellow', this.toWorld(this.holdX ?? -(HL + HOLD_OUT), PIN_H / 2 + 0.6, 0));
		this.aiGrab(pin);
	}
	stepPins(dt) {
		for (const r of this.justReleased || []) if ((r.t -= dt) <= 0 && !r.pin.held && this.field.has(r.pin)) for (let i = 0; i < r.pin.body.numColliders(); i++) r.pin.body.collider(i).setCollisionGroups(0xffffffff);
		if (this.justReleased) this.justReleased = this.justReleased.filter((r) => r.t > 0);
		if (!this.claw) return;
		const mq = this.quat().multiply(this.hold.q), hp = this.holdPose();
		const mp = this.toWorld(hp.x, Math.max(hp.y, (this.claw.kind === 'cup' ? CUP_H : PIN_H) / 2 + 0.1), 0);
		this.claw.body.setNextKinematicTranslation(mp);
		this.claw.body.setNextKinematicRotation({ x: mq.x, y: mq.y, z: mq.z, w: mq.w });
		const e = this.clawExtraPiece, rel = this.clawRel;
		if (e && rel) {
			const p = rel.pos.clone().applyQuaternion(mq).add(mp), rq = mq.clone().multiply(rel.quat);
			e.body.setNextKinematicTranslation(p);
			e.body.setNextKinematicRotation({ x: rq.x, y: rq.y, z: rq.z, w: rq.w });
		}
	}
	syncMesh(dt) {
		super.syncMesh(dt);
		this.paint();
	}

	// ---- Helpers --------------------------------------------------------------------------------
	fpos() { return toField(this.pos()); }
	heading() { const f = this.fwd(); return Math.atan2(-f.z, f.x); }
	holdField() { return toField(this.toWorld(this.holdX, 0, 0)); }
	arcade(fwd, turn) { this.leftCmd = clamp(fwd - turn, -1, 1) * 127; this.rightCmd = clamp(fwd + turn, -1, 1) * 127; }
	stop() { this.leftCmd = this.rightCmd = 0; this.liftTarget = this.liftDeg; }
	theirs() { return this.alliance === 'red' ? 'blue' : 'red'; }
	speed() { const v = this.body.linvel(); return Math.hypot(v.x, v.z); }
	onOurSide(f) { return this.alliance === 'red' ? f.fx + f.fy < -2 : f.fx + f.fy > 2; }
	pace() { return 0.62 + 0.4 * clamp((this.skill - 0.6) / 0.4, 0, 1); }   // rookie drives slower and less precisely

	// Value of one visible pin half of `color` in quadrant `quad` for this alliance.
	halfValue(color, quad) {
		const mine = this.alliance, theirs = this.theirs();
		if (color === mine) return 5;
		if (color === theirs) return -5;
		const own = quad === 'mid' ? (this.clockLeft != null && this.clockLeft < 25 ? mine : null) : this.field.toggles.find((t) => t.quad === quad)?.state;
		return own === mine ? 10 : own === theirs ? -10 : 3;   // unowned: worth something once we flip the Toggle
	}

	// What's stacked on goal g, bottom to top (upright pieces only).
	goalStack(g) {
		const out = [];
		for (const o of [...this.field.pins, ...this.field.cups]) {
			if (o.held) continue;
			const t = toField(o.body.translation()), r = o.body.rotation();
			if (Math.hypot(t.fx - g.fx, t.fy - g.fy) < 2.4 && t.h > g.base && 1 - 2 * (r.x * r.x + r.z * r.z) > 0.8) out.push({ o, h: t.h });
		}
		return out.sort((a, b) => a.h - b.h).map((x) => x.o);
	}
	upHalves(o) {   // pin: [lower colour, upper colour]; cup: { lowerOpaque, upperOpaque }
		const r = o.body.rotation(), up = 1 - 2 * (r.x * r.x + r.z * r.z) >= 0;
		if (o.kind === 'pin') return up ? [o.bottom, o.top] : [o.top, o.bottom];
		return { lowerOpaque: up !== o.opaqueUp, upperOpaque: up === o.opaqueUp };
	}

	// Points this alliance gains by putting what we hold on goal g (null if it can't go there).
	placeValue(g) {
		if (!this.claw) return null;
		if (g.color !== 'neutral' && g.color !== this.alliance) return null;   // <SG9>
		if (g.type === 'tall' && this.style !== 'cascade' && this.style !== 'dr4b') return null;
		const st = this.goalStack(g), top = st[st.length - 1], quad = g.quad;
		const pieces = [this.claw, this.clawExtraPiece].filter(Boolean).sort((a, b) => a.body.translation().y - b.body.translation().y);
		const bottom = pieces[0], pin = pieces.find((p) => p.kind === 'pin'), cup = pieces.find((p) => p.kind === 'cup');
		const seatH = this.seatHeight(g, st);
		if (seatH > 34) return null;   // too tall to reach / stay up
		let v = 0;
		if (bottom.kind === 'pin') {
			if (top && top.kind === 'pin') return null;   // <SC2>: a pin goes on an empty goal or into a cup
			const [lo, hi] = this.upHalves(pin);
			if (!top) v = this.halfValue(hi, quad);       // its lower half sits in the goal
			else v = (this.upHalves(top).upperOpaque ? 0 : this.halfValue(lo, quad)) + this.halfValue(hi, quad);
		} else {
			if (!top || top.kind !== 'pin') return null;  // a cup only goes over a pin
			const c = this.upHalves(cup), [, qHi] = this.upHalves(top);
			v -= c.lowerOpaque ? this.halfValue(qHi, quad) : 0;
			if (pin) { const [lo, hi] = this.upHalves(pin); v += (c.upperOpaque ? 0 : this.halfValue(lo, quad)) + this.halfValue(hi, quad); }
			else v += 1;   // a bare cup on top: sets up the next pin
		}
		if (g.color === this.alliance) v += 1.5;   // protected from the other alliance (<SG9>)
		return v;
	}
	seatHeight(g, st = this.goalStack(g)) {
		const top = st[st.length - 1];
		if (!top) return g.H;
		const h = toField(top.body.translation()).h;
		return top.kind === 'cup' ? h + CUP_H / 2 : h - PIN_H / 2 + PIN_COLLAR;
	}

	// ---- Driving --------------------------------------------------------------------------------
	costFn(others, ignore) {
		const goals = this.field.goals, loaders = this.field.loaders, pieces = this.field.cups;
		const bots = others.filter((r) => r !== this && r !== ignore && r.mesh.visible).map((r) => r.fpos ? r.fpos() : toField(r.pos()));
		const near = this.ai.task?.near;
		return (x, y, isGoal) => {
			const fx = centerOf(x), fy = centerOf(y);
			let c = 0;
			const wallD = WALL - Math.max(Math.abs(fx), Math.abs(fy));
			if (wallD < 7.6 && !isGoal) return Infinity;
			if (wallD < 11) c += (11 - wallD) * 0.15;
			for (const g of goals) {
				const d = Math.hypot(fx - g.fx, fy - g.fy), r = g.type === 'tall' ? 13 : 12;
				if (near === g && d > 6) continue;
				if (d < r && !isGoal) return Infinity;
				if (d < r + 6) c += (r + 6 - d) * 0.25;
			}
			for (const L of loaders) if (Math.hypot(fx - L.px, fy - L.fy) < 9 && !isGoal) return Infinity;
			for (const b of bots) { const d = Math.hypot(fx - b.fx, fy - b.fy); if (d < 17 && !isGoal) c += 30; else if (d < 26) c += (26 - d) * 0.5; }
			if (this.claw === null) for (const p of pieces) { if (p.held) continue; const t = toField(p.body.translation()); const d = Math.hypot(fx - t.fx, fy - t.fy); if (d < 6) c += 0.6; }
			return c;
		};
	}

	// Drive along a planned path to `to` (field point for the robot centre). Returns distance left.
	// opts: { reverse: force driving backwards / forwards, slow: speed cap 0..1, tol: stop radius, ignore: robot }
	navigate(to, others, opts = {}) {
		const ai = this.ai, p = this.fpos();
		const dist = Math.hypot(to.fx - p.fx, to.fy - p.fy);
		if (dist < (opts.tol ?? 2.5)) { this.arcade(0, 0); return dist; }
		if (!ai.path || this.t - ai.planT > 0.45 || !ai.goal || Math.hypot(ai.goal.fx - to.fx, ai.goal.fy - to.fy) > 2) {
			ai.path = dist < 10 ? [to] : plan(p, to, this.costFn(others, opts.ignore)) || [to];
			if (!ai.goal || Math.hypot(ai.goal.fx - to.fx, ai.goal.fy - to.fy) > 2) ai.rev = null;
			ai.planT = this.t; ai.goal = { ...to };
		}
		// Pure pursuit: aim at the first path point at least `look` away.
		while (ai.path.length > 1 && Math.hypot(ai.path[0].fx - p.fx, ai.path[0].fy - p.fy) < 4) ai.path.shift();
		const look = clamp(dist * 0.6, 4, 14);
		let aim = ai.path[ai.path.length - 1];
		for (const q of ai.path) if (Math.hypot(q.fx - p.fx, q.fy - p.fy) >= look) { aim = q; break; }
		const want = Math.atan2(aim.fy - p.fy, aim.fx - p.fx), h = this.heading();
		const errF = wrap(want - h), errR = wrap(want + Math.PI - h);
		// Forward or backward, with hysteresis so it doesn't flip-flop near the target.
		if (opts.reverse != null) ai.rev = opts.reverse;
		else if (ai.rev == null) ai.rev = Math.abs(errR) < Math.abs(errF) - 0.4 && dist < 48;
		else if (ai.rev && Math.abs(errR) > 2.2 && dist > 18) ai.rev = false;
		else if (!ai.rev && Math.abs(errF) > 2.4 && dist < 30) ai.rev = true;
		const err = ai.rev ? errR : errF;
		// Speed: turn first if way off, then a braking profile into the target (in/s -> command).
		const full = 55, pace = this.pace() * (opts.slow ?? 1);
		const v = Math.min(full * pace, Math.sqrt(2 * 28 * Math.max(0, dist - 1)) + 4);
		const near = dist < 14, turnFirst = Math.abs(err) > (near ? 0.35 : 0.65);   // close in: point-turn rather than orbit the spot
		let fwd = turnFirst ? 0 : (v / full) * Math.cos(err) ** (near ? 3 : 1);
		let turn = this.steer(err);
		if (fwd > 0.45) turn *= 0.7;
		this.arcade(ai.rev ? -fwd : fwd, turn);
		return dist;
	}
	// PD on heading; the spin-rate term brakes the turn before it overshoots.
	steer(err) {
		const w = this.body.angvel().y, k = 0.6 + 0.5 * this.skill;
		const out = clamp(k * 1.25 * err - 0.2 * w, -0.8, 0.8);
		return Math.abs(err) > 0.05 && Math.abs(out) < 0.13 && Math.abs(w) < 0.6 ? Math.sign(err) * 0.13 : out;   // enough to get it turning, not to buzz
	}
	turnTo(a) { const err = wrap(a - this.heading()); this.arcade(0, this.steer(err)); return Math.abs(err); }

	// Final approach: put the grip (hold point) on `spot` coming in along heading `a` (the robot
	// faces `a` for a front grip, the opposite way for a back grip). Slow once close.
	approachGrip(spot, a, others, done) {
		const ai = this.ai, front = this.holdX > 0;
		const off = Math.abs(this.holdX), dirx = Math.cos(a), diry = Math.sin(a);
		// The robot centre sits `off` short of the spot along the approach (it faces the spot with a
		// front grip, backs onto it with a back grip).
		const center = { fx: spot.fx - dirx * off, fy: spot.fy - diry * off };
		const stage = { fx: center.fx - dirx * 16, fy: center.fy - diry * 16 };
		const p = this.fpos(), face = front ? a : a + Math.PI;
		if (ai.phase !== 'final') {
			if (ai.phase !== 'align' && this.navigate(stage, others, { tol: 3 }) < 3) ai.phase = 'align';
			if (ai.phase === 'align') {
				ai.at = (ai.at || 0) + 1 / 120;
				if (this.turnTo(face) < 0.1 || ai.at > 1.6) { ai.phase = 'final'; ai.ft = 0; ai.at = 0; } else return false;
			}
			if (ai.phase !== 'final') return false;
		}
		// Straight in, slowly, steering the grip onto the spot. Blocked or slipping past it: go round again.
		ai.ft = (ai.ft || 0) + 1 / 120;
		if (ai.ft > 4) { ai.phase = ''; ai.ft = 0; ai.retries = (ai.retries || 0) + 1; if (ai.retries > 2 && ai.task) { ai.ban.set(ai.task.target, this.t + 10); ai.task.done = true; ai.retries = 0; } return false; }
		const g = this.holdField(), dx = spot.fx - g.fx, dy = spot.fy - g.fy;
		const along = dx * dirx + dy * diry, lat = -dx * diry + dy * dirx;
		if (along < -3 || Math.abs(lat) > 4 || Math.hypot(center.fx - p.fx, center.fy - p.fy) > 26) { ai.phase = ''; return false; }   // missed: go round again
		const herr = wrap(face - this.heading()) + clamp(lat * 0.06, -0.3, 0.3);   // steer the grip toward the spot (same sign driving either way)
		// Creep in and stop on it (back up a touch if it slid past).
		let v = clamp(along / 12, -0.14, 0.3) * (0.8 + 0.2 * this.skill);
		if (Math.abs(along) < 0.35) v = 0; else if (Math.abs(v) < 0.075) v = Math.sign(v) * 0.075;
		this.arcade(front ? v : -v, Math.abs(along) < 2 ? 0 : this.steer(herr) * 0.7);
		return done(along, lat);
	}

	// ---- Brain ----------------------------------------------------------------------------------
	think(dt, others) {
		const ai = this.ai;
		ai.t += dt;
		for (const [k, until] of ai.ban) if (until < this.t) ai.ban.delete(k);
		if (this.tipped()) { this.arcade(0, 0); return; }
		this.liftMode = 'pos';
		// Stuck: trying to drive but not moving -> back off, ban what we were going for.
		const p = this.fpos(), trying = Math.abs(this.leftCmd) + Math.abs(this.rightCmd) > 40;
		if (ai.phase !== 'unstick' && trying && !ai.pressing) {
			ai.stuckT += dt;
			if (ai.stuckT > 1.4) {
				if (ai.lastP && Math.hypot(p.fx - ai.lastP.fx, p.fy - ai.lastP.fy) < 2) {
					if (ai.task?.target) ai.ban.set(ai.task.target, this.t + 10);
					ai.phase = 'unstick'; ai.ut = 0; ai.udir = Math.sign(this.leftCmd + this.rightCmd) || 1;
				}
				ai.stuckT = 0; ai.lastP = p;
			}
		} else if (!trying) { ai.stuckT = 0; ai.lastP = p; }
		ai.pressing = false;
		if (ai.phase === 'unstick') {
			ai.ut += dt;
			this.arcade(-0.5 * ai.udir, ai.ut < 0.5 ? 0.45 : 0);
			if (ai.ut > 0.9) { ai.phase = ''; ai.task = null; ai.path = null; }
			return;
		}
		// Endgame (last ~10 s, less if it's far): Midfield for the 8 points and the yellow there.
		if (this.clockLeft != null && this.clockLeft < 10.5) { this.endgame(others); return; }
		if (this.period === 'auton') { this.auton(dt, others); return; }
		// Re-decide every so often (faster for better drivers), or when the task is done / gone.
		// Mid-ram on a Toggle or in the last inches of a grab / placement: finish it first (those phases time out).
		const committed = ai.task && !ai.task.done && COMMITTED.has(ai.phase);
		if (!ai.task || ai.task.done || (!committed && this.t - ai.decideT > 2.5 - this.skill)) { this.decide(others); ai.decideT = this.t; }
		const T = ai.task;
		if (!T) { this.park(others); return; }
		this.arcade(0, 0);   // a task that's finished this step leaves the drive stopped, not on its last command
		this['t_' + T.kind](T, others, dt);
	}

	// Candidate tasks scored by points per second.
	decide(others) {
		const ai = this.ai, cand = [], p = this.fpos(), mine = this.alliance;
		const time = (q) => Math.hypot(q.fx - p.fx, q.fy - p.fy) / (34 * this.pace()) + 1.2;
		const busy = new Set(others.filter((r) => r !== this && r.ai && r.alliance === mine).map((r) => r.ai.task?.target).filter(Boolean));
		const canHold = CAN_HOLD[this.style];
		if (this.claw) {
			for (const g of this.field.goals) {
				if (ai.ban.has(g)) continue;
				const v = this.placeValue(g);
				if (v == null || v <= 0) continue;
				cand.push({ kind: 'place', target: g, near: g, score: v / (time(g) + 1.5) });
			}
			if (!cand.length) cand.push({ kind: 'drop', score: 0.1 });
		} else if (canHold) {
			// Match loads from our Loaders (the cascade / claw robots are built for them).
			for (const L of this.field.loaders) {
				if (L.alliance !== mine || !L.ready || busy.has(L) || ai.ban.has(L)) continue;
				const best = this.bestGoalFor({ cup: true, pin: true });
				const v = (best ? best.v : 4) * (this.style === 'cascade' ? 1.35 : 0.8);
				cand.push({ kind: 'load', target: L, score: v / (time({ fx: L.px, fy: L.fy }) + 2.5 + (best ? 2.5 : 0)) });
			}
			// Pieces on the field we could pick up.
			for (const o of [...this.field.pins, ...this.field.cups]) {
				if (o.held || ai.ban.has(o) || busy.has(o) || this.field.onGoal(o)) continue;
				const t = toField(o.body.translation());
				if (Math.abs(t.fx) > WALL - 2 || Math.abs(t.fy) > WALL - 2) continue;
				const r = o.body.rotation(), up = Math.abs(1 - 2 * (r.x * r.x + r.z * r.z)) > 0.85;
				if (!up && this.style !== 'intake') continue;   // only the intake bot picks pins off the floor
				if (o.kind === 'pin' && this.field.partnerOf(o)) continue;   // the cup it's in is what gets picked
				const partner = o.kind === 'cup' ? this.field.partnerOf(o) : null;
				const what = { cup: o.kind === 'cup', pin: o.kind === 'pin' || !!partner, colors: o.kind === 'pin' ? [o.bottom, o.top] : partner ? [partner.bottom, partner.top] : null };
				if (what.colors && what.colors.every((c) => c === this.theirs())) continue;
				const best = this.bestGoalFor(what);
				if (!best) continue;
				cand.push({ kind: 'pick', target: o, score: best.v / (time(t) + 1.5 + best.t) });
			}
		}
		// Toggles: flip ones that aren't ours (worth the yellow already in that quarter, doubled if theirs).
		// The attacker / defender live on them; the scorers only go when it's worth a lot.
		const partnerOnToggles = others.some((r) => r !== this && r.ai && r.alliance === mine && r.ai.task?.kind === 'toggle');
		for (const tg of this.field.toggles) {
			if (ai.ban.has(tg) || busy.has(tg)) continue;
			const yellow = this.yellowIn(tg.quad), cur = tg.state;
			if (cur === mine) continue;
			const hits = this.hitsTo(tg, mine), own = tg.quad.startsWith(mine);
			let v = yellow * (cur === 'yellow' ? 10 : 20) + 4;
			v *= { attacker: own ? 1.6 : 2.2, defender: own ? 2.4 : 0.7, pusher: 0.6 }[this.strategy] ?? (yellow >= 2 ? 0.7 : 0.35);
			if (this.claw) v *= 0.3;
			if (partnerOnToggles) v *= 0.5;
			cand.push({ kind: 'toggle', target: tg, score: v / (time(tg) + 1.8 * hits) });
		}
		// Defense.
		if (this.strategy === 'pusher' || this.strategy === 'defender' || this.strategy === 'attacker') {
			const opp = this.opponent(others);
			if (opp) cand.push({ kind: 'push', target: opp, score: (this.strategy === 'pusher' ? 6 : 1.5) * (opp.claw ? 1.6 : 1) / (time(opp.fpos ? opp.fpos() : toField(opp.pos())) + 2) });
		}
		if (this.period === 'auton') {   // <SG7>: only things on our side of the Autonomous Line, no defense
			const where = (c) => (c.target?.px != null ? { fx: c.target.px, fy: c.target.fy } : c.target?.body ? toField(c.target.body.translation()) : c.target);
			for (let i = cand.length - 1; i >= 0; i--) { const w = where(cand[i]); if (cand[i].kind === 'push' || (w && w.fx != null && !this.onOurSide(w))) cand.splice(i, 1); }
		}
		cand.sort((a, b) => b.score - a.score);
		const pick = cand[0] || null;
		if (pick && ai.task && ai.task.target === pick.target && ai.task.kind === pick.kind) return;   // keep going
		// Don't switch for a small gain once committed (stops dithering).
		if (ai.task && !ai.task.done && pick && ai.task.score && pick.score < ai.task.score * 1.3 && ai.task.kind !== 'drop') return;
		ai.task = pick; ai.phase = ''; ai.path = null; ai.tt = 0;
	}

	bestGoalFor(what) {
		let best = null;
		const p = this.fpos();
		for (const g of this.field.goals) {
			if (g.color !== 'neutral' && g.color !== this.alliance) continue;
			const st = this.goalStack(g), top = st[st.length - 1];
			if (this.seatHeight(g, st) > 30) continue;
			let v;
			if (what.cup) {
				if (!top || top.kind !== 'pin') continue;
				const [, qHi] = this.upHalves(top);
				v = -this.halfValue(qHi, g.quad) * 0.6 + (what.colors ? what.colors.reduce((a, c) => a + this.halfValue(c, g.quad), 0) : what.pin ? 15 : 1);
			} else {
				if (top && top.kind === 'pin') continue;
				v = what.colors ? this.halfValue(what.colors[1], g.quad) + (top ? this.halfValue(what.colors[0], g.quad) * 0.7 : 0) : 5;
			}
			if (g.color === this.alliance) v += 1.5;
			const t = Math.hypot(g.fx - p.fx, g.fy - p.fy) / 34 + 2;
			if (v > 0 && (!best || v / t > best.v / best.t)) best = { g, v, t };
		}
		return best;
	}

	yellowIn(quad) {
		let n = 0;
		for (const g of this.field.goals) {
			if (g.quad !== quad) continue;
			const st = this.goalStack(g);
			st.forEach((o, i) => {
				if (o.kind !== 'pin') return;
				const [lo, hi] = this.upHalves(o), above = st[i + 1];
				if (hi === 'yellow' && !(above && above.kind === 'cup' && this.upHalves(above).lowerOpaque)) n++;
				if (lo === 'yellow' && i > 0 && st[i - 1].kind === 'cup' && !this.upHalves(st[i - 1]).upperOpaque) n++;
			});
		}
		return n;
	}
	hitsTo(tg, color) {
		const faces = tg.quad.startsWith('red') ? ['yellow', 'blue', 'red'] : ['yellow', 'red', 'blue'];
		const cur = ((tg.index % 3) + 3) % 3;
		for (let k = 1; k <= 3; k++) if (faces[(cur + k) % 3] === color) return k;
		return 3;
	}
	opponent(others) {
		const p = this.fpos();
		return others.filter((r) => r.alliance !== this.alliance && r.mesh.visible && !this.inTheirLoadZone(toField(r.pos())))
			.map((r) => { const q = toField(r.pos()); return { r, c: Math.hypot(q.fx - p.fx, q.fy - p.fy) - (r.claw ? 30 : 0) }; })
			.sort((a, b) => a.c - b.c)[0]?.r || null;
	}
	inTheirLoadZone(q) { return Math.abs(q.fy) > 44 && (this.alliance === 'red' ? q.fx > 56 : q.fx < -56); }   // <SG13>: theirs are protected

	// ---- Tasks ------------------------------------------------------------------------------------
	t_pick(T, others) {
		const o = T.target;
		if (!this.field.has(o) || o.held || this.field.onGoal(o)) { T.done = true; return; }
		this.liftTarget = 40;
		const t = toField(o.body.translation()), p = this.fpos();
		// Come in from the open side (away from the wall it may be against).
		const out = Math.abs(t.fx) > WALL - 8 || Math.abs(t.fy) > WALL - 8
			? (Math.abs(t.fx) > Math.abs(t.fy) ? Math.atan2(0, -Math.sign(t.fx)) : Math.atan2(-Math.sign(t.fy), 0)) : Math.atan2(t.fy - p.fy, t.fx - p.fx) + Math.PI;
		const a = out + Math.PI;   // approach heading: toward the piece
		if (this.approachGrip(t, a, others, (along, lat) => {
			if (Math.abs(lat) < 1.2 && along < 0.8 && this.relSpeed(o) < CFG.grabMaxSpeed) { this.arcade(0, 0); this.aiGrab(o); T.done = true; return true; }
			return false;
		})) return;
		if ((this.ai.tt += 1 / 120) > 9) { this.ai.ban.set(o, this.t + 12); T.done = true; }
	}

	t_load(T, others) {
		const L = T.target;
		if (this.claw) { T.done = true; return; }
		this.liftTarget = 40;
		const spot = { fx: L.px, fy: L.fy }, a = Math.atan2(0, -L.n);   // toward the wall
		if (!L.ready) { this.navigate({ fx: L.px + L.n * 26, fy: L.fy * 0.92 }, others); return; }
		if (this.approachGrip(spot, a, others, (along, lat) => {
			if (Math.abs(lat) < 1.0 && along < 0.8 && this.speed() < 0.12 && L.ready) {
				this.arcade(0, 0);
				const st = L.stack, o = this.field.takeLoader(L);
				this.aiGrab(o);
				for (const k of st) if (!k.held) this.field.freePiece(k);
				T.done = true; return true;
			}
			return false;
		})) return;
		if ((this.ai.tt += 1 / 120) > 10) { this.ai.ban.set(L, this.t + 8); T.done = true; }
	}

	t_place(T, others) {
		const g = T.target;
		if (!this.claw) { T.done = true; return; }
		const v = this.placeValue(g);
		if (v == null) { T.done = true; return; }
		const seat = this.seatHeight(g);
		const bottom = this.claw && this.clawBottom(), bh = bottom ? toField(bottom.body.translation()).h - (bottom.kind === 'cup' ? CUP_H : PIN_H) / 2 : 0;
		const seatBottom = bottom.kind === 'cup' ? seat : seat - PIN_COLLAR;
		const p = this.fpos(), a = Math.atan2(g.fy - p.fy, g.fx - p.fx);
		const holdLow = bottom ? bh - this.liftIn() : 0;   // piece bottom relative to the lift
		const clearDeg = (seatBottom + 1.2 - holdLow) / CFG.liftInPerDeg;
		const dGoal = Math.hypot(g.fx - p.fx, g.fy - p.fy);
		this.liftTarget = dGoal < 36 ? clamp(clearDeg, 0, CFG.liftMaxDeg) : Math.min(clamp(clearDeg, 0, CFG.liftMaxDeg), 300);
		const ai = this.ai;
		if (ai.phase === 'lower') {
			this.arcade(0, 0);
			this.liftTarget = clamp((seatBottom - 0.2 - holdLow) / CFG.liftInPerDeg, 0, CFG.liftMaxDeg);
			if (bh <= seatBottom + 0.35 || Math.abs(this.liftDeg - this.liftTarget) < 8) {
				const pieces = [this.claw, this.clawExtraPiece].filter(Boolean);
				this.releaseClaw(false, true);
				void pieces;
				ai.phase = 'leave'; ai.lt = 0;
			}
			return;
		}
		if (ai.phase === 'leave') {
			ai.lt += 1 / 120;
			this.arcade(this.holdX > 0 ? -0.35 : 0.35, 0);
			if (ai.lt > 0.6) T.done = true;
			return;
		}
		this.approachGrip({ fx: g.fx, fy: g.fy }, a, others, (along, lat) => {
			const ready = Math.abs(this.liftDeg - this.liftTarget) < 25;
			if (!ready) { this.arcade(0, 0); return false; }
			const tol = 1.3 - 0.5 * this.skill;
			if (Math.abs(lat) < tol && Math.abs(along) < tol && this.speed() < 0.08) { ai.phase = 'lower'; return true; }
			return false;
		});
		if ((ai.tt += 1 / 120) > 12) { ai.ban.set(g, this.t + 10); T.done = true; }
	}

	t_drop(T) {   // nothing useful to do with what we hold: set it down out of the way
		this.releaseClaw(true); T.done = true;
	}

	// Line up square on the Toggle, ram it, back off, ram again (one face per hit, Field.stepToggles).
	t_toggle(T, others, dt) {
		const tg = T.target, ai = this.ai, n = { x: tg.n[0], y: tg.n[1] };
		if (tg.state === this.alliance && tg.set) { T.done = true; ai.ban.set(tg, this.t + 5); return; }
		this.liftTarget = 0;
		ai.tt += dt;
		const face = Math.atan2(-n.y, -n.x), p = this.fpos();
		const off = (p.fx - tg.fx) * n.x + (p.fy - tg.fy) * n.y;
		const front = 1;   // only the front flips a Toggle (Field.stepToggles)
		const faceDir = front > 0 ? face : face + Math.PI;
		const stand = { fx: tg.fx + n.x * (HL + 13), fy: tg.fy + n.y * (HL + 13) };
		if (!ai.phase || ai.phase === 'approach') {
			const d = this.navigate(stand, others, { tol: 3 });
			if (d >= 3 && !(d < 7 && this.speed() < 0.15)) { ai.phase = 'approach'; return; }
			ai.phase = 'face';
		}
		if (ai.phase === 'face') {
			if (Math.hypot(stand.fx - p.fx, stand.fy - p.fy) > 9) { ai.phase = 'approach'; return; }
			if (this.turnTo(faceDir) < 0.07) { ai.phase = 'ram'; ai.rt = 0; }
			return;
		}
		ai.pressing = true;
		if (ai.phase === 'ram') {
			ai.rt += dt;
			this.arcade(front, this.steer(wrap(faceDir - this.heading())) * 0.5);
			if (tg.flipped || ai.rt > 1.6) { ai.phase = 'back'; ai.rt = 0; }
			if (off > HL + 22) ai.phase = 'approach';
			return;
		}
		if (ai.phase === 'back') {
			ai.rt += dt;
			// Hung up on the Toggle or a piece: twist while backing to come free.
			this.arcade(-0.7 * front, ai.rt > 0.4 && this.speed() < 0.1 ? 0.5 : 0);
			if (off > HL + 8 || ai.rt > 0.8) { ai.phase = 'settle'; ai.rt = 0; }
			return;
		}
		ai.rt += dt;   // settle: let it finish turning, then hit again if needed
		this.arcade(0, this.steer(wrap(faceDir - this.heading())) * 0.5);
		if (tg.set || ai.rt > 0.7) { if (tg.state === this.alliance) T.done = true; ai.phase = 'ram'; ai.rt = 0; }
		if (ai.tt > 14) { ai.ban.set(tg, this.t + 10); T.done = true; }
	}

	t_push(T, others) {
		const r = T.target, q = toField(r.pos());
		this.liftTarget = 0;
		if (!r.mesh.visible || this.inTheirLoadZone(q)) { T.done = true; return; }
		const d = this.navigate(q, others, { ignore: r });
		if (d < 24) { this.ai.pressing = true; const a = Math.atan2(q.fy - this.fpos().fy, q.fx - this.fpos().fx); this.arcade(Math.cos(wrap(a - this.heading())) > 0.6 ? 1 : 0, this.steer(wrap(a - this.heading()))); }
		if ((this.ai.tt += 1 / 120) > 6) T.done = true;
	}

	park(others) {
		this.liftTarget = 0;
		const s = this.alliance === 'red' ? -1 : 1, slot = this.label.endsWith('1') ? 1 : -1;
		if (this.navigate({ fx: s * 30, fy: slot * 14 }, others) < 4) this.arcade(0, 0);
	}

	endgame(others) {
		this.liftTarget = 0;   // <SG12>: back down to starting height in the Midfield
		if (this.claw && this.liftIn() > 3) { this.releaseClaw(true); }
		const s = this.alliance === 'red' ? -1 : 1, slot = this.label.endsWith('1') ? 1 : -1;
		const spot = { fx: s * 12, fy: slot * 7 };
		const d = this.navigate(spot, others, { slow: 1 });
		if (this.inMidfield() && d < 9) {
			// In: hold position, square up against anyone shoving.
			const opp = others.find((r) => r.alliance !== this.alliance && r.mesh.visible && Math.hypot(toField(r.pos()).fx - spot.fx, toField(r.pos()).fy - spot.fy) < 22);
			if (opp) { this.ai.pressing = true; const q = toField(opp.pos()), a = Math.atan2(q.fy - this.fpos().fy, q.fx - this.fpos().fx); this.arcade(0.6, this.steer(wrap(a - this.heading()))); }
			else this.arcade(0, 0);
		}
	}

	// Autonomous: on our side of the line only (<SG7>). Flip our Toggle, then score the preload and
	// whatever else is on our side.
	auton(dt, others) {
		const ai = this.ai;
		if (!ai.autonPlan) {
			const p = this.fpos();
			const near = this.field.toggles.filter((t) => this.onOurSide(t)).sort((a, b) => Math.hypot(a.fx - p.fx, a.fy - p.fy) - Math.hypot(b.fx - p.fx, b.fy - p.fy))[0];
			ai.autonPlan = near ? [{ kind: 'toggle', target: near, score: 1 }] : [];
		}
		if (!ai.task || ai.task.done) {
			const T = ai.autonPlan.shift();
			if (T) { ai.task = T; ai.phase = ''; ai.tt = 0; } else this.decide(others);
		}
		const T = ai.task;
		if (!T) { this.arcade(0, 0); return; }
		this['t_' + T.kind](T, others, dt);
		// Never cross the Autonomous Line.
		const ahead = toField(this.toWorld(Math.sign(this.leftCmd + this.rightCmd) * (HL + 4), 0, 0));
		if (!this.onOurSide(ahead)) this.arcade(0, this.steer(0));
	}
}

void TOGGLE; void LOADER; void MID; void quadrantOf; void W;
