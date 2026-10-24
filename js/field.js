// Override field: perimeter, goals, toggles, pins, cups, and the scoring rules (<SC2>-<SC6>).
// Field coordinates are inches from the field centre: +fx = right (blue side),
// +fy = up (Blue 2 / far wall). World (three.js / Rapier) is metres, Y up, Z = -fy.
import * as THREE from 'three';

export const IN = 0.0254;
export const W = (fx, fy, h = 0) => new THREE.Vector3(fx * IN, h * IN, -fy * IN);
export const toField = (v) => ({ fx: v.x / IN, fy: -v.z / IN, h: v.y / IN });

export const FIELD_HALF = 72;                // tiles: 6 x 6 of 24", they run under the walls
export const WALL = 70.2;                    // inside face of the perimeter (Appendix A: 140.4" wall to wall)
export const WALL_H = 11.54;
export const MID = 24;                       // Midfield is |fx| + |fy| <= MID
// Pin dimensions from the official drawing: 6.50" tall, collar Ø3.16, the bottom colour
// runs from the base to 2.93" (where the collar starts).
export const PIN_R = 1.58, PIN_H = 6.5, PIN_COLLAR = 2.93;
export const CUP_R = 1.575, CUP_H = 6.5;     // manual: 3.15" x 6.5"
const GOAL_R = 1.7, GOAL_INNER = 1.27, GOAL_DEPTH = 3.0;   // round hole Ø2.54, pin collar rests on the rim
const GOAL_HEIGHT = { alliance: 3.25, short: 5.8, tall: 8.7 };
export const TOGGLE = { axisH: WALL_H + 1.19, inset: 0.3, len: 25.8, face: 2.03 };   // on top of the wall
// Face 0 faces the field at rest; each hit shows the next face. Each Toggle is mounted so
// the Quadrant's own colour comes up on the 2nd hit (per the team).
// Shove needed to flip a Toggle one face, in N*s (momentum + push). Raise to make it harder.
export const TOGGLE_EFFORT = 14;
export const toggleFaces = (quad) => (quad.startsWith('red') ? ['yellow', 'blue', 'red'] : ['yellow', 'red', 'blue']);
export const COLORS = { red: 0xd62828, blue: 0x1e6fd9, yellow: 0xf5c518, neutral: 0x2b2b2b };
export const GROUP = { robot: 0x0002, toggle: 0x0004, wall: 0x0008, claw: 0x0010, fixed: 0x0020 };
export const groups = (member, filter) => ((member & 0xffff) << 16) | (filter & 0xffff);

const GOALS = [
	{ fx: 0, fy: 0, type: 'tall', color: 'neutral' },
	{ fx: 24, fy: 48, type: 'alliance', color: 'blue' },
	{ fx: -24, fy: 48, type: 'short', color: 'neutral' },
	{ fx: -48, fy: 24, type: 'short', color: 'neutral' },
	{ fx: -48, fy: -24, type: 'alliance', color: 'red' },
	{ fx: 48, fy: 24, type: 'alliance', color: 'blue' },
	{ fx: 48, fy: -24, type: 'short', color: 'neutral' },
	{ fx: -24, fy: -48, type: 'alliance', color: 'red' },
	{ fx: 24, fy: -48, type: 'short', color: 'neutral' },
];

// Loaders (Appendix A "Loader specifications"): a clear 3.74"-square tube hung on the inside of each
// alliance-station wall near the corner, 23" tall, with a slot at the bottom (up to 11.85") open to the
// field and both sides. Match loads stand on the tiles inside it, one nested on the next (gravity fed,
// two cup + pin stacks fit). The human player drops them in through the top (<SG11>); a robot of
// that alliance takes the bottom one out through the slot (the claw has to fit under the tube).
// n = which way the slot faces (+1 = +fx).
export const LOADER = { inner: 3.74, outer: 4.02, h: 23, slotTop: 11.85, off: 2.26, cap: 2, stackStep: 6.95 };
export const MATCH_LOADS = { cups: 10, pins: 10, yellow: 1 };   // per alliance: 10 Cups, 10 alliance/yellow Pins, 1 yellow/yellow
const LOADERS = [
	{ alliance: 'red', fx: -WALL, fy: 60.7, n: 1 }, { alliance: 'red', fx: -WALL, fy: -60.7, n: 1 },
	{ alliance: 'blue', fx: WALL, fy: 60.7, n: -1 }, { alliance: 'blue', fx: WALL, fy: -60.7, n: -1 },
];

// Toggles: one per Quadrant, centred on each wall. n = inward normal (field coords).
export const TOGGLES = [
	{ quad: 'red1', fx: -WALL, fy: 0, n: [1, 0] },
	{ quad: 'blue1', fx: WALL, fy: 0, n: [-1, 0] },
	{ quad: 'blue2', fx: 0, fy: WALL, n: [0, -1] },
	{ quad: 'red2', fx: 0, fy: -WALL, n: [0, 1] },
];

// Foam tile texture: one 24" tile with interlocking edges, repeated 6 x 6.
function tileTexture() {
	const c = document.createElement('canvas'), N = 512;
	c.width = c.height = N;
	const g = c.getContext('2d');
	g.fillStyle = '#6d7175'; g.fillRect(0, 0, N, N);
	const img = g.getImageData(0, 0, N, N);
	for (let i = 0; i < img.data.length; i += 4) { const n = (Math.random() - 0.5) * 14; img.data[i] += n; img.data[i + 1] += n; img.data[i + 2] += n; }
	g.putImageData(img, 0, 0);
	g.strokeStyle = 'rgba(30,32,36,0.75)'; g.lineWidth = 3;
	for (const horiz of [true, false]) {
		g.beginPath();
		for (let k = 0; k <= 16; k++) {
			const t = (k / 16) * N, d = k % 2 ? 10 : 0;
			horiz ? g.lineTo(t, 2 + d) : g.lineTo(2 + d, t);
		}
		g.stroke();
	}
	const tex = new THREE.CanvasTexture(c);
	tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
	tex.repeat.set(6, 6);
	tex.colorSpace = THREE.SRGBColorSpace;
	tex.anisotropy = 8;
	return tex;
}
// Pin shape from the manual (Figure P-1): octagonal, a tall taper on top, a wide collar,
// then a short taper underneath. The colour split is at the bottom of the collar.
export const PIN_SPLIT = PIN_COLLAR - PIN_H / 2;   // inches from the pin centre
// Profiles (radius, height from the pin bottom), from the drawing.
const PIN_BOTTOM = [[0.0001, 0], [0.62, 0], [0.7, 0.06], [0.7, 0.64], [1.3, PIN_COLLAR]];
const PIN_COLLAR_P = [[1.3, PIN_COLLAR], [1.52, PIN_COLLAR], [1.58, 3.03], [1.58, 3.27], [1.45, 3.4], [1.175, 3.47]];
const PIN_TOP = [[1.175, 3.47], [0.62, 5.72], [0.6, 6.42], [0.5, 6.5], [0.0001, 6.5]];
let PIN_GEO;
function pinGeo() {
	if (PIN_GEO) return PIN_GEO;
	const lathe = (pts, seg) => {
		const g = new THREE.LatheGeometry(pts.map(([r, y]) => new THREE.Vector2(r * IN, (y - PIN_H / 2) * IN)), seg);
		g.rotateY(Math.PI / seg); g.computeVertexNormals(); return g;
	};
	return (PIN_GEO = { top: lathe([...PIN_COLLAR_P, ...PIN_TOP.slice(1)], 8), bottom: lathe(PIN_BOTTOM, 8) });
}
// Convex hull points for a profile segment (8-sided), centred on the pin centre.
function hullPts(profile, seg = 8) {
	const out = [];
	for (const [r, y] of profile) for (let k = 0; k < seg; k++) {
		const a = ((k + 0.5) / seg) * Math.PI * 2;
		out.push(Math.cos(a) * Math.max(r, 0.05) * IN, (y - PIN_H / 2) * IN, Math.sin(a) * Math.max(r, 0.05) * IN);
	}
	return new Float32Array(out);
}
const PIN_MATS = Object.fromEntries(['red', 'blue', 'yellow'].map((c) => [c, new THREE.MeshStandardMaterial({ color: COLORS[c], roughness: 0.55 })]));
// Cup (from the photo): clear half flares out to a thin rim with inner ribs; the dark
// opaque half flares down to a wider foot. Each lathe runs centre -> end.
let CUP_GEO;
function cupGeo() {
	if (CUP_GEO) return CUP_GEO;
	const L = (pts, n) => new THREE.LatheGeometry(pts.map(([r, y]) => new THREE.Vector2(r * IN, y * IN)), n);
	const clear = L([[1.2, 0], [1.24, 0.8], [1.38, 2.0], [1.53, 3.0], [CUP_R, CUP_H / 2]], 32);
	const opaque = L([[1.2, 0], [1.22, 0.8], [1.32, 2.2], [1.45, 2.95], [1.5, 3.05], [1.5, CUP_H / 2]], 32);
	const rib = new THREE.BoxGeometry(0.05 * IN, 2.6 * IN, 0.22 * IN);
	return (CUP_GEO = { clear, opaque, rib });
}
const CUP_MATS = {
	opaque: new THREE.MeshStandardMaterial({ color: 0x45494e, roughness: 0.6, side: THREE.DoubleSide }),
	clear: new THREE.MeshPhysicalMaterial({ color: 0xf2f7fb, transparent: true, opacity: 0.3, roughness: 0.05, side: THREE.DoubleSide, depthWrite: false }),
	rim: new THREE.MeshStandardMaterial({ color: 0x6b7178, roughness: 0.4 }),
	rib: new THREE.MeshStandardMaterial({ color: 0xdfe8f0, transparent: true, opacity: 0.45, roughness: 0.1, depthWrite: false }),
};

// Stop anything getting launched: a pushed pin or cup never moves faster than the robot can.
// Pins and cups can't be shoved faster than a robot drives (~1.6 m/s), and nothing on the field can
// throw them upward. Without this, a piece wedged between the robot and a cup or goal gets
// spat out by the solver and goes flying.
export function clampVelocities(objs) {
	for (const o of objs) {
		if (o.held) { o.funnel = null; continue; }
		if (o.funnel) {   // sliding down the goal's chamfer: held over the post, falling gently, not tumbling
			const f = o.funnel, c = o.body.translation(), v0 = o.body.linvel();
			o.body.setLinvel({ x: (f.x - c.x) * 12, y: Math.max(v0.y, -1.2), z: (f.z - c.z) * 12 }, true);
			o.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
			if ((f.t -= 1 / 120) <= 0) o.funnel = null;
		}
		const v = o.body.linvel(), hs = Math.hypot(v.x, v.z), k = hs > 1.8 ? 1.8 / hs : 1;
		const vy = Math.min(v.y, 0.6);
		if (k < 1 || vy !== v.y) o.body.setLinvel({ x: v.x * k, y: vy, z: v.z * k }, true);
		const w = o.body.angvel(), ws = Math.hypot(w.x, w.y, w.z);
		if (ws > 8) o.body.setAngvel({ x: (w.x * 8) / ws, y: (w.y * 8) / ws, z: (w.z * 8) / ws }, true);   // a pin tipping over turns ~5-8 rad/s
	}
}

export function quadrantOf(fx, fy) {
	if (Math.abs(fx) + Math.abs(fy) <= MID) return 'mid';
	if (fy >= Math.abs(fx)) return 'blue2';
	if (fy <= -Math.abs(fx)) return 'red2';
	return fx < 0 ? 'red1' : 'blue1';
}

// Thin boxes in a ring: the walls of a goal cavity or a cup.
function ringColliders(R, desc, rInner, thick, h, yCenter, extra) {
	const n = 8, side = 2 * (rInner + thick) * Math.tan(Math.PI / n);
	const out = [];
	for (let j = 0; j < n; j++) {
		const a = (j / n) * Math.PI * 2, rm = rInner + thick / 2;
		const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -a);
		let d = R.ColliderDesc.cuboid((thick / 2) * IN, (h / 2) * IN, (side / 2) * IN)
			.setTranslation(Math.cos(a) * rm * IN, yCenter * IN, Math.sin(a) * rm * IN)
			.setRotation(q);
		out.push(extra ? extra(d) : d);
	}
	return out;
}

// Text tag that always faces the camera, with a little arrow pointing down at the thing.
export function textSprite(text, color, width = 18) {
	const c = document.createElement('canvas');
	c.width = 640; c.height = 160;
	const g = c.getContext('2d');
	g.font = 'bold 64px system-ui, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
	const w = g.measureText(text).width + 60;
	g.fillStyle = 'rgba(10,12,16,0.78)';
	g.beginPath(); g.roundRect((640 - w) / 2, 8, w, 96, 22); g.fill();
	g.fillStyle = color; g.fillText(text, 320, 58);
	g.beginPath(); g.moveTo(300, 108); g.lineTo(340, 108); g.lineTo(320, 150); g.closePath(); g.fill();
	const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), transparent: true, depthTest: false }));
	sp.scale.set(width * IN, ((width * 160) / 640) * IN, 1);
	sp.renderOrder = 10;
	return sp;
}

export class Field {
	constructor(R, world, scene) {
		this.R = R; this.world = world; this.scene = scene;
		this.goals = []; this.pins = []; this.cups = []; this.toggles = []; this.loaders = [];
		this.dynamicMeshes = [];
		this.buildStatic();
		// Floor, goals etc. (fixed bodies with default groups): the claw frame passes over them. Only
		// pins and cups collide with it; a fixed object pressing on the claw of a floor-locked robot
		// makes the solver fling the robot.
		world.forEachCollider((c) => {
			if (c.parent()?.isFixed() && c.collisionGroups() === 0xffffffff) c.setCollisionGroups(groups(GROUP.fixed, 0xffff & ~GROUP.claw));
		});
		this.reset();
	}

	buildStatic() {
		const { R, world, scene } = this;
		// Floor: 6x6 tiles.
		const floor = world.createRigidBody(R.RigidBodyDesc.fixed());
		world.createCollider(R.ColliderDesc.cuboid(80 * IN, 1 * IN, 80 * IN).setTranslation(0, -1 * IN, 0).setFriction(0.7), floor);
		const tiles = new THREE.Mesh(new THREE.PlaneGeometry(144 * IN, 144 * IN),
			new THREE.MeshStandardMaterial({ map: tileTexture(), roughness: 0.97, metalness: 0 }));
		tiles.rotation.x = -Math.PI / 2;
		tiles.receiveShadow = true;
		scene.add(tiles);
		// Arena floor + alliance stations outside the field.
		const arena = new THREE.Mesh(new THREE.PlaneGeometry(14, 14), new THREE.MeshStandardMaterial({ color: 0x2b3038, roughness: 0.9 }));
		arena.rotation.x = -Math.PI / 2; arena.position.y = -0.002; arena.receiveShadow = true;
		scene.add(arena);
		for (const [fx, col] of [[-110, COLORS.red], [110, COLORS.blue]]) {
			const st = new THREE.Mesh(new THREE.PlaneGeometry(60 * IN, 140 * IN), new THREE.MeshStandardMaterial({ color: col, roughness: 0.8, transparent: true, opacity: 0.55 }));
			st.rotation.x = -Math.PI / 2; st.position.copy(W(fx, 0, 0.01));
			scene.add(st);
		}
		this.buildTape();
		// Perimeter walls.
		const poly = new THREE.MeshStandardMaterial({ color: 0xe6f0fa, transparent: true, opacity: 0.18, roughness: 0.05, metalness: 0.1, depthWrite: false });
		const alu = new THREE.MeshStandardMaterial({ color: 0xc8ccd1, metalness: 0.85, roughness: 0.3 });
		const wallBody = world.createRigidBody(R.RigidBodyDesc.fixed());
		const Wc = WALL + 1, Wl = 2 * WALL + 4;   // 2"-thick portable perimeter
		for (const [fx, fy, sx, sy] of [[-Wc, 0, 2, Wl], [Wc, 0, 2, Wl], [0, Wc, Wl, 2], [0, -Wc, Wl, 2]]) {
			world.createCollider(R.ColliderDesc.cuboid((sx / 2) * IN, (WALL_H / 2) * IN, (sy / 2) * IN)
				.setTranslation(fx * IN, (WALL_H / 2) * IN, -fy * IN).setFriction(0.3)
				.setCollisionGroups(groups(GROUP.wall, 0xffff & ~GROUP.toggle)), wallBody);
			const panel = new THREE.Mesh(new THREE.BoxGeometry((sx > 2 ? sx : 0.3) * IN, (WALL_H - 3) * IN, (sy > 2 ? sy : 0.3) * IN), poly);
			panel.position.copy(W(fx, fy, 1.5 + (WALL_H - 3) / 2));
			scene.add(panel);
			for (const [h, t] of [[1, 2], [WALL_H - 0.75, 1.5]]) {
				const rail = new THREE.Mesh(new THREE.BoxGeometry(Math.max(sx, 2) * IN, t * IN, Math.max(sy, 2) * IN), alu);
				rail.position.copy(W(fx, fy, h)); rail.castShadow = rail.receiveShadow = true;
				scene.add(rail);
			}
			for (let k = -WALL - 1; k <= WALL + 1.01; k += (2 * WALL + 2) / 6) {
				const post = new THREE.Mesh(new THREE.BoxGeometry(2 * IN, WALL_H * IN, 2 * IN), alu);
				post.position.copy(W(sx > 2 ? k : fx, sx > 2 ? fy : k, WALL_H / 2)); post.castShadow = true;
				scene.add(post);
			}
		}
		for (const L of LOADERS) this.buildLoader(L);
		for (const g of GOALS) this.buildGoal(g);
		for (const t of TOGGLES) this.buildToggle(t);
	}

	// ---- Loaders ---------------------------------------------------------------
	// Clear tube on an aluminium hanger hooked over the wall. Tag + floor glow are switched on for
	// the driver's own loaders by setMine().
	buildLoader(L) {
		const { R, world, scene } = this, c = COLORS[L.alliance], o = LOADER.outer / 2;
		const px = L.fx + L.n * LOADER.off, grp = new THREE.Group();
		grp.position.copy(W(px, L.fy, 0));
		grp.rotation.y = L.n > 0 ? 0 : Math.PI;   // local +x = the slot side, toward the field
		scene.add(grp);
		const box = (sx, sy, sz, x, y, z, mat, parent = grp) => {
			const m = new THREE.Mesh(new THREE.BoxGeometry(sx * IN, sy * IN, sz * IN), mat);
			m.position.set(x * IN, y * IN, z * IN); m.castShadow = true; parent.add(m);
			return m;
		};
		const team = new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, emissive: c, emissiveIntensity: 0 });
		const alu = new THREE.MeshStandardMaterial({ color: 0xc8ccd1, metalness: 0.8, roughness: 0.35 });
		const clear = new THREE.MeshPhysicalMaterial({ color: 0xeef6ff, transparent: true, opacity: 0.26, roughness: 0.05, side: THREE.DoubleSide, depthWrite: false });
		const H = LOADER.h, st = LOADER.slotTop, t = 0.14;
		// Upper tube (four clear walls) above the slot; below it only the back wall and two corner posts.
		box(t, H - st, 2 * o, -o + t / 2, (H + st) / 2, 0, clear);
		box(t, H - st, 2 * o, o - t / 2, (H + st) / 2, 0, clear);
		for (const z of [-1, 1]) box(2 * o, H - st, t, 0, (H + st) / 2, z * (o - t / 2), clear);
		box(t, st, 2 * o, -o + t / 2, st / 2, 0, clear);
		for (const z of [-1, 1]) box(0.35, st, 0.35, -o + 0.2, st / 2, z * (o - 0.2), alu);
		// Coloured bands: at the top, round the slot's top edge, and the base of the back.
		for (const y of [H - 0.4, st + 0.35]) {
			box(0.3, 0.7, 2 * o + 0.2, -o, y, 0, team); box(0.3, 0.7, 2 * o + 0.2, o, y, 0, team);
			for (const z of [-1, 1]) box(2 * o + 0.2, 0.7, 0.3, 0, y, z * o, team);
		}
		box(0.3, 0.7, 2 * o, -o, 0.4, 0, team);
		// Hanger: plate behind the tube, a hook over the 2" wall top, a lip down the outside.
		const wf = -LOADER.off;   // the wall's inside face (local x)
		box(0.2, H - 0.5, 3, (wf + -o) / 2, H / 2, 0, alu);
		box(2.6, 0.3, 3, wf - 1.1, WALL_H + 0.15, 0, alu);
		box(0.2, 3, 3, wf - 2.15, WALL_H - 1.4, 0, alu);
		const body = world.createRigidBody(R.RigidBodyDesc.fixed());
		const col = (sx, sy, sz, x, y, z) => world.createCollider(R.ColliderDesc.cuboid(sx * IN / 2, sy * IN / 2, sz * IN / 2)
			.setTranslation((px + L.n * x) * IN, y * IN, -L.fy * IN).setFriction(0.3).setCollisionGroups(groups(GROUP.fixed, 0xffff & ~GROUP.claw)), body);
		col(2 * o, H - st, 2 * o, 0, (H + st) / 2, 0);    // upper tube: solid
		col(0.4, st, 2 * o, -o + 0.2, st / 2, 0);         // back wall of the slot
		// Load Zone glow and the floating tag (ours only).
		const zoneMat = new THREE.MeshBasicMaterial({ color: c, transparent: true, opacity: 0.2, depthWrite: false });
		const zone = new THREE.Mesh(new THREE.PlaneGeometry((WALL - 58) * IN, (WALL - 46) * IN), zoneMat);
		zone.rotation.x = -Math.PI / 2;
		zone.position.copy(W(L.fx + L.n * (WALL - 58) / 2, Math.sign(L.fy) * (WALL + 46) / 2, 0.03));
		scene.add(zone);
		const tag = textSprite('YOUR LOADER', L.alliance === 'red' ? '#ff6b6b' : '#6bb0ff');
		tag.position.set(0, (H + 5) * IN, 0);
		grp.add(tag);
		this.loaders.push({ ...L, px, grp, team, zone, zoneMat, tag, mine: false, stacks: [], stack: null, ready: false, drop: null });
	}

	has(o) { return this.pins.includes(o) || this.cups.includes(o); }

	removePiece(o) {
		this.world.removeRigidBody(o.body); this.scene.remove(o.mesh);
		this.pins = this.pins.filter((p) => p !== o); this.cups = this.cups.filter((p) => p !== o);
	}

	// Loader contents: L.stacks = [[cup, pin], ...] bottom first; L.stack = the bottom one (what a robot takes).
	syncLoaderStack(L) {
		L.stacks = L.stacks.filter((st) => st.every((o) => o.held === 'loader' && this.has(o)));
		L.stack = L.stacks[0] || null;
		L.ready = !!L.stack && !L.drop;
	}

	// Place loader pieces: each stack stands on the one below (nested), `fall` inches still to drop.
	placeLoaderStacks(L) {
		let base = 0;
		for (const st of L.stacks) {
			const fall = st.fall || 0;
			for (const o of st) o.body.setNextKinematicTranslation(W(L.px, L.fy, base + o.loadY + fall));
			base += st.step;
		}
	}
	loaderHeight(L) { return L.stacks.reduce((a, st) => a + st.step, 0); }

	// The claw takes loader L's bottom stack: hand over its bottom piece (the claw grabs it and its
	// partner); the ones above drop down the tube.
	takeLoader(L) {
		const st = L.stacks.shift();
		for (const o of st) o.held = null;
		for (const up of L.stacks) up.fall = (up.fall || 0) + st.step;
		this.syncLoaderStack(L);
		return st.reduce((a, o) => (o.body.translation().y < a.body.translation().y ? o : a));
	}

	// A piece the loader was holding that nothing picked up: back to a normal, solid piece.
	freePiece(o) {
		o.body.setBodyType(this.R.RigidBodyType.Dynamic, true);
		for (let i = 0; i < o.body.numColliders(); i++) o.body.collider(i).setCollisionGroups(0xffffffff);
	}

	// The driver's own loaders (the ones their human player uses) get the tag and glow.
	setMine(fn) { for (const L of this.loaders) { L.mine = fn(L); L.tag.visible = L.zone.visible = L.mine; } }

	syncLoaders() {
		const pulse = 0.5 + 0.5 * Math.sin(performance.now() / 250);
		for (const L of this.loaders) {
			L.team.emissiveIntensity = L.mine ? 0.3 + 0.5 * pulse : 0;
			L.zoneMat.opacity = 0.12 + 0.12 * pulse;
			L.tag.material.opacity = L.ready ? 1 : 0.5;
		}
	}

	// Gravity feed: stacks still falling down the tube settle (called every physics step).
	stepLoaders(dt) {
		for (const L of this.loaders) {
			this.syncLoaderStack(L);
			let moving = false;
			for (const st of L.stacks) if (st.fall > 0) { st.v = (st.v || 0) + 9.81 / IN * dt; st.fall = Math.max(0, st.fall - st.v * dt); moving = true; if (!st.fall) st.v = 0; }
			L.drop = moving ? L.drop || true : null;
			this.placeLoaderStacks(L);
			this.syncLoaderStack(L);
		}
	}

	// A pin sitting in a cup <-> that cup.
	partnerOf(o) {
		const to = o.body.translation();
		for (const k of o.kind === 'cup' ? this.pins : this.cups) {
			if (k === o || k.held) continue;
			const t = k.body.translation(), flat = Math.hypot(t.x - to.x, t.z - to.z) / IN, dy = (t.y - to.y) / IN;
			if (flat < 1.3 && (o.kind === 'cup' ? dy > 0.5 && dy < 5.5 : dy < -0.5 && dy > -5.5)) return k;
		}
		return null;
	}

	// Sitting on a goal (in its hole or stacked on it)?
	onGoal(o) {
		const t = toField(o.body.translation());
		return this.goals.some((g) => Math.hypot(t.fx - g.fx, t.fy - g.fy) < 2.5 && t.h > g.base);
	}

	buildTape() {
		const mat = new THREE.MeshBasicMaterial({ color: 0xffffff });
		const line = (x1, y1, x2, y2, w = 1, m = mat) => {
			const len = Math.hypot(x2 - x1, y2 - y1);
			const s = new THREE.Mesh(new THREE.PlaneGeometry(len * IN, w * IN), m);
			s.rotation.x = -Math.PI / 2;
			s.rotation.z = Math.atan2(y2 - y1, x2 - x1);
			s.position.copy(W((x1 + x2) / 2, (y1 + y2) / 2, 0.02));
			this.scene.add(s);
		};
		// Midfield square (a diamond in field coords).
		line(0, MID, MID, 0); line(MID, 0, 0, -MID); line(0, -MID, -MID, 0); line(-MID, 0, 0, MID);
		// Autonomous Line: doubled diagonal top-left -> bottom-right, single lines on the other diagonal.
		const E = WALL - 1.5;
		for (const o of [-1.2, 1.2]) { line(-E + o, E + o, -MID / 2 + o, MID / 2 + o); line(MID / 2 + o, -MID / 2 + o, E + o, -E + o); }
		line(-E, -E, -MID / 2, -MID / 2); line(MID / 2, MID / 2, E, E);
		// Load Zone tape.
		const red = new THREE.MeshBasicMaterial({ color: COLORS.red }), blue = new THREE.MeshBasicMaterial({ color: COLORS.blue });
		for (const sy of [1, -1]) {
			line(-58, WALL * sy, -58, 46 * sy, 1, red); line(-WALL, 46 * sy, -58, 46 * sy, 1, red);
			line(58, WALL * sy, 58, 46 * sy, 1, blue); line(WALL, 46 * sy, 58, 46 * sy, 1, blue);
		}
	}

	// Goals, modelled on the team's printed ones. Small goals (alliance + neutral short):
	// flanged octagonal base flaring up into an octagonal block with a round hole and
	// gusset slots. Centre goal: two-tier black octagon. The pin collar rests on the rim.
	buildGoal(g) {
		const { R, world, scene } = this;
		const H = GOAL_HEIGHT[g.type], base = H - GOAL_DEPTH, tall = g.type === 'tall';
		const prof = tall
			? [[3.5, 0], [3.5, 0.3], [3.3, 0.35], [3.3, H * 0.5], [2.35, H * 0.5 + 0.7], [2.35, H - 0.2], [2.15, H]]
			: [[3.1, 0], [3.1, 0.25], [2.85, 0.3], [2.2, H * 0.55], [2.2, H - 0.18], [2.02, H]];
		const body = world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(g.fx * IN, 0, -g.fy * IN));
		// Colliders: solid convex base up to the hole floor, then walls around the hole.
		const hull = [];
		for (const [r, y] of prof) {
			if (y > base) continue;
			for (let k = 0; k < 8; k++) { const a = ((k + 0.5) / 8) * Math.PI * 2; hull.push(Math.cos(a) * r * IN, y * IN, Math.sin(a) * r * IN); }
		}
		for (let k = 0; k < 8; k++) { const a = ((k + 0.5) / 8) * Math.PI * 2, r = tall ? 3.3 : 2.4; hull.push(Math.cos(a) * r * IN, Math.max(base, 0.05) * IN, Math.sin(a) * r * IN); }
		world.createCollider(R.ColliderDesc.convexHull(new Float32Array(hull)), body);
		const outerR = tall ? 2.1 : 2.0;
		for (const d of ringColliders(R, null, GOAL_INNER, outerR - GOAL_INNER, GOAL_DEPTH, base + GOAL_DEPTH / 2)) world.createCollider(d, body);
		if (!tall) for (const d of ringColliders(R, null, outerR, 0.6, H * 0.5, H * 0.25)) world.createCollider(d, body);
		// Mesh
		const color = g.color === 'neutral' ? (tall ? 0x1c1c1e : 0x2a2b2e) : COLORS[g.color];
		const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.05, flatShading: true });
		const grp = new THREE.Group();
		grp.position.copy(W(g.fx, g.fy, 0));
		const outer = new THREE.Mesh(new THREE.LatheGeometry(prof.map(([r, y]) => new THREE.Vector2(r * IN, y * IN)), 8), mat);
		outer.rotation.y = Math.PI / 8;
		outer.castShadow = outer.receiveShadow = true;
		// Top face: octagon with the round hole.
		const top = new THREE.Shape(), rTop = prof[prof.length - 1][0] * IN;
		for (let k = 0; k <= 8; k++) { const a = ((k + 0.5) / 8) * Math.PI * 2; k ? top.lineTo(Math.cos(a) * rTop, Math.sin(a) * rTop) : top.moveTo(Math.cos(a) * rTop, Math.sin(a) * rTop); }
		const hole = new THREE.Path(); hole.absarc(0, 0, GOAL_INNER * IN, 0, Math.PI * 2, true); top.holes.push(hole);
		const topM = new THREE.Mesh(new THREE.ShapeGeometry(top, 24), mat);
		topM.rotation.x = -Math.PI / 2; topM.position.y = H * IN;
		const holeWall = new THREE.Mesh(new THREE.CylinderGeometry(GOAL_INNER * IN, GOAL_INNER * IN, GOAL_DEPTH * IN, 32, 1, true),
			new THREE.MeshStandardMaterial({ color: new THREE.Color(color).multiplyScalar(0.55), roughness: 0.8, side: THREE.BackSide }));
		holeWall.position.y = (base + GOAL_DEPTH / 2) * IN;
		const floor = new THREE.Mesh(new THREE.CircleGeometry(GOAL_INNER * IN, 32), new THREE.MeshStandardMaterial({ color: 0x0c0c0c }));
		floor.rotation.x = -Math.PI / 2; floor.position.y = base * IN + 0.0005;
		grp.add(outer, topM, holeWall, floor);
		// Gusset slots on the flared faces of the small goals.
		if (!tall) for (let k = 0; k < 8; k += 2) {
			const a = (k / 8) * Math.PI * 2 + Math.PI / 8, rr = 2.55;
			const slot = new THREE.Mesh(new THREE.BoxGeometry(0.12 * IN, H * 0.32 * IN, 0.05 * IN), new THREE.MeshBasicMaterial({ color: 0x050505 }));
			slot.position.set(Math.cos(a) * rr * IN, H * 0.3 * IN, Math.sin(a) * rr * IN);
			slot.rotation.y = -a + Math.PI / 2; slot.rotation.x = 0.35;
			grp.add(slot);
		}
		scene.add(grp);
		const quad = quadrantOf(g.fx, g.fy);
		this.goals.push({ ...g, H, base, quad });
		if (quad !== 'mid') {   // floor ring in the colour that owns yellow pins on this goal
			const ringMat = new THREE.MeshBasicMaterial({ color: 0x555555, transparent: true, opacity: 0.85 });
			const ring = new THREE.Mesh(new THREE.RingGeometry(5 * IN, 6.4 * IN, 40), ringMat);
			ring.rotation.x = -Math.PI / 2;
			ring.position.copy(W(g.fx, g.fy, 0.06));
			this.scene.add(ring);
			(this.ownerRings ||= []).push({ quad, mat: ringMat });
		}
	}

	// Triangular prism on the wall top. Only rotates about its long axis, which is
	// always a world axis, so the body just has its other DOFs locked (no joint needed).
	buildToggle(t) {
		const { R, world, scene } = this;
		const n = new THREE.Vector3(t.n[0], 0, -t.n[1]);
		const up = new THREE.Vector3(0, 1, 0);
		const axis = new THREE.Vector3().crossVectors(n, up);       // +rotation turns n toward up
		const centre = W(t.fx + t.n[0] * TOGGLE.inset, t.fy + t.n[1] * TOGGLE.inset, TOGGLE.axisH);
		const r = (TOGGLE.face / Math.sqrt(3)) * IN, half = (TOGGLE.len / 2) * IN;
		const pts = [];
		for (let k = 0; k < 3; k++) {
			const th = ((-60 + 120 * k) * Math.PI) / 180;
			const v = n.clone().multiplyScalar(Math.cos(th) * r).addScaledVector(up, Math.sin(th) * r);
			for (const s of [-1, 1]) { const p = v.clone().addScaledVector(axis, s * half); pts.push(p.x, p.y, p.z); }
		}
		const body = world.createRigidBody(R.RigidBodyDesc.kinematicPositionBased().setTranslation(centre.x, centre.y, centre.z));
		const col = world.createCollider(R.ColliderDesc.convexHull(new Float32Array(pts)).setDensity(400).setFriction(0.4)
			.setCollisionGroups(groups(GROUP.toggle, 0xffff & ~GROUP.wall)), body);
		// Mesh in a canonical frame (X = axis, Y = up, Z = -n), oriented by `base`.
		const grp = new THREE.Group();
		for (let i = 0; i < 3; i++) {
			const th = (120 * i * Math.PI) / 180;      // face i normal angle in the (n, up) plane
			const face = new THREE.Mesh(new THREE.BoxGeometry(TOGGLE.len * IN, TOGGLE.face * IN, 0.08 * IN),
				new THREE.MeshStandardMaterial({ color: COLORS[toggleFaces(t.quad)[i]], roughness: 0.5 }));
			const d = r * Math.cos(Math.PI / 3);         // apothem
			face.position.set(0, Math.sin(th) * d, -Math.cos(th) * d);
			face.rotation.x = -th;
			face.castShadow = true;
			grp.add(face);
		}
		const base = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(axis, up, n.clone().negate()));
		scene.add(grp);
		// Owner light: a bar along the wall above the Toggle, lit in the colour that owns this quarter.
		const lightMat = new THREE.MeshStandardMaterial({ color: 0x555555, emissive: 0x000000, roughness: 0.4 });
		const light = new THREE.Mesh(new THREE.BoxGeometry(TOGGLE.len * IN, 1.2 * IN, 1.2 * IN), lightMat);
		light.position.copy(W(t.fx - t.n[0] * 1.2, t.fy - t.n[1] * 1.2, TOGGLE.axisH + 3.4));
		light.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), axis.clone().normalize());
		scene.add(light);
		this.toggles.push({ ...t, body, col, grp, base, axis, nWorld: n, alpha: 0, index: 0, touching: false, state: 'yellow', set: true, ownerMats: [lightMat] });
	}

	// Each separate hit from the robot (driving toward the wall) advances the Toggle one
	// face. A flat face pushed head-on gives almost no torque, so a pure physics hinge
	// would never turn; this ratchet matches "hit it, back off, hit again".
	stepToggles(dt, robots) {
		const third = (2 * Math.PI) / 3;
		robots = [].concat(robots);
		for (const tg of this.toggles) {
			let touching = false, robot = robots[0];
			for (const r of robots) {
				for (const c of r.colliders) this.world.contactPair(c, tg.col, (m) => { if (m.numContacts() > 0) touching = true; });
				if (r.clawToggle === tg) touching = true;
				if (touching) { robot = r; break; }
			}
			const v = robot.body.linvel();
			const approach = -(v.x * tg.nWorld.x + v.z * tg.nWorld.z);   // m/s toward the wall
			// A flip needs a real shove: impact momentum + drive force pushed into it over time.
			// A full-speed ram or ~0.5 s of full-power pushing does it; a light bump doesn't.
			// One flip per contact: back off before it can turn again.
			if (touching) {
				const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(robot.quat());
				const push = (robot.driveForce || 0) * -(fwd.x * tg.nWorld.x + fwd.z * tg.nWorld.z);
				tg.effort = tg.touching ? tg.effort + Math.max(0, push) * dt : robot.body.mass() * Math.max(0, approach, robot.toggleHitV || 0);
				// Flipped by the front of the robot or the end of the claw frame (claw horizontal);
				// the back of the robot or the lifted claw (higher) don't flip it.
				const frontOn = fwd.x * tg.nWorld.x + fwd.z * tg.nWorld.z < -0.5 || robot.clawToggle === tg;
				if (!tg.flipped && frontOn && tg.effort >= TOGGLE_EFFORT) { tg.index++; tg.flipped = true; }
			} else { tg.effort = 0; tg.flipped = false; }
			tg.touching = touching;
			const target = -tg.index * third;
			tg.alpha += Math.max(-10 * dt, Math.min(10 * dt, target - tg.alpha));
			const q = new THREE.Quaternion().setFromAxisAngle(tg.axis, tg.alpha);
			tg.body.setNextKinematicRotation({ x: q.x, y: q.y, z: q.z, w: q.w });
			tg.set = Math.abs(target - tg.alpha) < 0.02 && !touching;
			// Ownership changes only once the Toggle has settled on a new face; touching it or the turn
			// itself doesn't flash it back to yellow (that made the score jump around).
			if (tg.set) tg.state = toggleFaces(tg.quad)[((tg.index % 3) + 3) % 3];
		}
	}

	// ---- Scoring objects ---------------------------------------------------------
	clearObjects() {
		for (const o of [...this.pins, ...this.cups]) { this.world.removeRigidBody(o.body); this.scene.remove(o.mesh); }
		this.pins = []; this.cups = [];
	}

	makePin(top, bottom, pos, quat) {
		const { R, world } = this;
		const body = world.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(pos.x, pos.y, pos.z)
			.setRotation(quat || { x: 0, y: 0, z: 0, w: 1 }).setCcdEnabled(true).setLinearDamping(0.5).setAngularDamping(3));   // tips over, doesn't twirl
		// Compound shape matching the real pin: narrow tapered top, wide collar, short taper below.
		// ~60 g total so the robot pushes it instead of launching it.
		// Mass is set by hand: Rapier computes these thin hulls' centre of mass ~0.4" off the axis,
		// which tipped standing pins over by themselves. 65 g, centre of mass on the axis just
		// below mid-height, inertia of a 2"-wide, 6.5"-tall solid.
		const m = 0.065, r = 1.0 * IN, h = PIN_H * IN;
		const part = (profile, massive) => {
			const d = R.ColliderDesc.convexHull(hullPts(profile)).setFriction(0.6).setRestitution(0);
			if (massive) d.setMassProperties(m, { x: 0, y: -0.15 * IN, z: 0 }, { x: (m * (3 * r * r + h * h)) / 12, y: 0.5 * m * r * r, z: (m * (3 * r * r + h * h)) / 12 }, { x: 0, y: 0, z: 0, w: 1 });
			else d.setDensity(0);
			return world.createCollider(d, body);
		};
		const collider = part(PIN_COLLAR_P, true);
		const extra = [part(PIN_BOTTOM), part(PIN_TOP)];
		const mesh = new THREE.Group();
		for (const [c, geo] of [[top, pinGeo().top], [bottom, pinGeo().bottom]]) {
			const h = new THREE.Mesh(geo, PIN_MATS[c]);
			h.castShadow = true;
			mesh.add(h);
		}
		this.scene.add(mesh);
		const pin = { kind: 'pin', top, bottom, body, collider, extra, mesh, held: null };
		this.pins.push(pin);
		return pin;
	}

	makeCup(pos, opaqueUp) {
		const { R, world } = this;
		const body = world.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(pos.x, pos.y, pos.z)
			.setCcdEnabled(true).setLinearDamping(0.3).setAngularDamping(2.5));
		const wall = 0.1;
		for (const d of ringColliders(R, null, CUP_R - wall, wall, CUP_H, 0, (d) => d.setDensity(500).setFriction(0.5)))
			world.createCollider(d, body);
		world.createCollider(R.ColliderDesc.cylinder(0.05 * IN, (CUP_R - wall) * IN).setDensity(500), body);
		// Real cup: clear half flares out to a rim, opaque dark-grey half flares to a base.
		const mesh = new THREE.Group();
		for (const s of [1, -1]) {
			const isOpaque = (s > 0) === opaqueUp;
			const h = new THREE.Mesh(isOpaque ? cupGeo().opaque : cupGeo().clear, isOpaque ? CUP_MATS.opaque : CUP_MATS.clear);
			if (s < 0) h.rotation.x = Math.PI;
			h.castShadow = isOpaque;
			mesh.add(h);
			if (!isOpaque) {
				for (let k = 0; k < 8; k++) {   // internal ribs
					const a = (k / 8) * Math.PI * 2, rb = new THREE.Mesh(cupGeo().rib, CUP_MATS.rib);
					rb.position.set(Math.cos(a) * 1.3 * IN, s * 1.75 * IN, Math.sin(a) * 1.3 * IN);
					rb.rotation.y = -a; rb.rotation.z = s * 0.1;
					mesh.add(rb);
				}
				const rim = new THREE.Mesh(new THREE.TorusGeometry(CUP_R * IN, 0.05 * IN, 6, 24), CUP_MATS.rim);
				rim.rotation.x = Math.PI / 2; rim.position.y = s * (CUP_H / 2) * IN;
				mesh.add(rim);
			}
		}
		const mid = new THREE.Mesh(new THREE.CircleGeometry(1.2 * IN, 16), CUP_MATS.opaque);
		mid.rotation.x = -Math.PI / 2;
		mesh.add(mid);
		this.scene.add(mesh);
		const cup = { kind: 'cup', opaqueUp, body, mesh };
		this.cups.push(cup);
		return cup;
	}

	// Starting configuration (Figure FO-2), read off the overhead diagram.
	reset() {
		this.clearObjects();
		this.loads = { red: { ...MATCH_LOADS }, blue: { ...MATCH_LOADS } };
		for (const L of this.loaders) { L.stacks = []; L.stack = null; L.ready = false; L.drop = null; }
		for (const tg of this.toggles) {
			tg.body.setRotation({ x: 0, y: 0, z: 0, w: 1 }, true);
			tg.index = 0; tg.alpha = 0; tg.touching = false;
		}
		const upright = (fx, fy, h) => W(fx, fy, h);
		const cupWithPin = (fx, fy, opaqueUp, top, bottom) => {
			this.makeCup(upright(fx, fy, CUP_H / 2), opaqueUp);
			if (top) this.makePin(top, bottom, upright(fx, fy, CUP_H - PIN_COLLAR + 0.04 + PIN_H / 2));   // collar on the rim
		};
		// Yellow/yellow Pins already sitting in the 5 neutral Goals.
		for (const g of this.goals) if (g.color === 'neutral') this.makePin('yellow', 'yellow', W(g.fx, g.fy, g.H - PIN_COLLAR + 0.04 + PIN_H / 2));
		// Along each wall: [cup, cup + yellow pin, cup] twice.
		for (const [ax, ay] of [[-1, 0], [1, 0], [0, 1], [0, -1]]) {
			for (const off of [-23, 23]) {
				for (const d of [-8, 0, 8]) {
					const along = off + d, wallPos = WALL - CUP_R - 0.3;
					const fx = ax ? ax * wallPos : along, fy = ay ? ay * wallPos : along;
					cupWithPin(fx, fy, true, d === 0 ? 'yellow' : null, 'yellow');
				}
			}
		}
		// Cups with yellow Pins on the diagonal.
		for (const [fx, fy] of [[48, 48], [24, 24], [-24, -24], [-48, -48]]) cupWithPin(fx, fy, false, 'yellow', 'yellow');
		// Red/blue Pins in cups at the Midfield corners.
		for (const [fx, fy] of [[0, MID], [0, -MID], [MID, 0], [-MID, 0]]) cupWithPin(fx, fy, false, 'red', 'blue');
		// "Stars": an empty cup with 4 horizontal pins around it (alliance colour on the outer tip).
		for (const [cx, cy] of [[-48, 48], [-24, 24], [24, -24], [48, -48]]) {
			cupWithPin(cx, cy, false, null);
			for (const [dx, dy, col] of [[0, 1, 'blue'], [-1, 0, 'red'], [1, 0, 'blue'], [0, -1, 'red']]) {
				const off = CUP_R + 0.4 + PIN_H / 2;
				const p = W(cx + dx * off, cy + dy * off, PIN_R);
				// Pin local +Y (the 'top' half) points outward along (dx, dy).
				const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(dx, 0, -dy));
				this.makePin(col, 'yellow', p, q);
			}
		}
	}

	// The goal tops are chamfered and the pins/cups taper, so a piece dropped a little off-centre
	// funnels onto the post. Within 2.5" of a goal it is lined up and drops straight down — only
	// where it can legally go: a pin onto an empty goal or into a cup, a cup onto a pin (<SC3>).
	// Where piece o (pin or cup) could legally go on a goal it's over: { g, top, surface, seatBottom }
	// (seatBottom = height of the piece's bottom once seated), or null.
	seatFor(o, tol = 2.5) {
		const t = toField(o.body.translation()), cup = this.cups.includes(o);
		for (const g of this.goals) {
			if (Math.hypot(t.fx - g.fx, t.fy - g.fy) > tol) continue;
			let top = null, topH = -1;
			for (const k of [...this.pins, ...this.cups]) {
				if (k === o || k.held) continue;
				const q = toField(k.body.translation()), r = k.body.rotation();
				if (1 - 2 * (r.x * r.x + r.z * r.z) > 0.85 && Math.hypot(q.fx - g.fx, q.fy - g.fy) < 2.2 && q.h > g.base && q.h > topH) { top = k; topH = q.h; }
			}
			const topIsCup = top && this.cups.includes(top);
			if (cup ? !top || topIsCup : top && !topIsCup) return null;
			const surface = !top ? g.H : topIsCup ? topH + CUP_H / 2 : topH - PIN_H / 2 + PIN_COLLAR;
			return { g, top, surface, seatBottom: cup ? surface : surface - PIN_COLLAR };
		}
		return null;
	}

	assistDrop(o, tol = 2.5) {
		const t = toField(o.body.translation()), cup = this.cups.includes(o);
		for (const g of this.goals) {
			if (Math.hypot(t.fx - g.fx, t.fy - g.fy) > tol) continue;
			let top = null, topH = -1;
			for (const k of [...this.pins, ...this.cups]) {
				if (k === o || k.held) continue;
				const q = toField(k.body.translation()), r = k.body.rotation();
				if (1 - 2 * (r.x * r.x + r.z * r.z) > 0.85 && Math.hypot(q.fx - g.fx, q.fy - g.fy) < 2.2 && q.h > g.base && q.h > topH) { top = k; topH = q.h; }
			}
			const topIsCup = top && this.cups.includes(top);
			if (cup ? !top || topIsCup : top && !topIsCup) return false;
			const surface = !top ? g.H : topIsCup ? topH + CUP_H / 2 : topH - PIN_H / 2 + PIN_COLLAR;
			if (t.h - PIN_H / 2 < (cup ? surface : surface - PIN_COLLAR) - 0.5) return false;   // already below its seat: too late
			// Aim at the top of the stack (a cup may sit a little off-centre), then guide it down.
			const aim = top ? toField(top.body.translation()) : g;
			const p = W(aim.fx, aim.fy, Math.max(t.h, surface + PIN_H / 2));
			o.body.setTranslation({ x: p.x, y: p.y, z: p.z }, true);
			o.funnel = { x: p.x, z: p.z, t: 1.5 };
			// Stand it up straight but keep it the way round it was (upside-down stays upside-down).
			const rr = o.body.rotation(), upY = 1 - 2 * (rr.x * rr.x + rr.z * rr.z);
			o.body.setRotation(upY >= 0 ? { x: 0, y: 0, z: 0, w: 1 } : { x: 1, y: 0, z: 0, w: 0 }, true);
			o.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
			o.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
			return true;
		}
		return false;
	}

	// Light bars + goal rings show who owns each quarter (yellow face = nobody: grey).
	syncOwners() {
		const col = { red: 0xd62828, blue: 0x1e6fd9 };
		for (const tg of this.toggles) {
			const c = col[tg.state];
			for (const m of tg.ownerMats) { m.color.setHex(c ?? 0x555555); m.emissive.setHex(c ?? 0x000000); m.emissiveIntensity = c ? 0.9 : 0; }
		}
		const owner = Object.fromEntries(this.toggles.map((t) => [t.quad, t.state]));
		for (const r of this.ownerRings || []) r.mat.color.setHex(col[owner[r.quad]] ?? 0x5a5a5a);
	}

	syncMeshes() {
		this.syncOwners();
		this.syncLoaders();
		for (const o of [...this.pins, ...this.cups]) {
			const t = o.body.translation(), r = o.body.rotation();
			o.mesh.position.set(t.x, t.y, t.z);
			o.mesh.quaternion.set(r.x, r.y, r.z, r.w);
		}
		for (const tg of this.toggles) {
			const t = tg.body.translation(), r = tg.body.rotation();
			tg.grp.position.set(t.x, t.y, t.z);
			tg.grp.quaternion.set(r.x, r.y, r.z, r.w).multiply(tg.base);
		}
	}

	// ---- Scoring ---------------------------------------------------------------
	// robotsInMid: { red: n, blue: n }
	score(robotsInMid) {
		const up = new THREE.Vector3(0, 1, 0);
		const axisOf = (o) => { const r = o.body.rotation(); return up.clone().applyQuaternion(new THREE.Quaternion(r.x, r.y, r.z, r.w)); };
		const posOf = (o) => { const t = o.body.translation(); return new THREE.Vector3(t.x, t.y, t.z); };
		const pins = this.pins.filter((p) => !p.held).map((p) => {
			const c = posOf(p), a = axisOf(p);
			const topEnd = c.clone().addScaledVector(a, (PIN_H / 2) * IN), botEnd = c.clone().addScaledVector(a, -(PIN_H / 2) * IN);
			const topIsHigh = topEnd.y >= botEnd.y;
			return { p, high: topIsHigh ? topEnd : botEnd, low: topIsHigh ? botEnd : topEnd,
				highColor: topIsHigh ? p.top : p.bottom, lowColor: topIsHigh ? p.bottom : p.top,
				placed: false, hideHigh: false, hideLow: false, goal: null };
		});
		const cups = this.cups.filter((c) => !c.held).map((c) => {
			const a = axisOf(c); const upAxis = a.y >= 0 ? a : a.clone().negate();
			return { c, centre: posOf(c), upAxis, upright: Math.abs(a.y) > 0.7,
				upperOpaque: (a.y >= 0) === c.opaqueUp, placed: false, goal: null };
		});
		// Point e is inside half `side` (+1 upper, -1 lower) of cup k?
		const inCupHalf = (k, e, side) => {
			if (!k.upright) return false;
			const d = e.clone().sub(k.centre), s = d.dot(k.upAxis) / IN;
			const radial = d.addScaledVector(k.upAxis, -s * IN).length() / IN;
			return radial < CUP_R && (side > 0 ? s > -0.2 && s < CUP_H / 2 + 0.2 : s < 0.2 && s > -CUP_H / 2 - 0.2);
		};
		for (const pi of pins) for (const g of this.goals) {
			const e = pi.low, ef = toField(e);
			if (Math.hypot(ef.fx - g.fx, ef.fy - g.fy) < GOAL_R && ef.h < g.H + 0.1 && ef.h > g.base - 0.4) {
				pi.placed = true; pi.hideLow = true; pi.goal = g; break;
			}
		}
		for (let changed = true; changed;) {
			changed = false;
			for (const k of cups) if (!k.placed) for (const pi of pins) if (pi.placed && inCupHalf(k, pi.high, -1)) {
				k.placed = true; k.goal = pi.goal; changed = true;
				if (!k.upperOpaque) pi.hideHigh = true;   // lower half of the cup is the opaque one
				break;
			}
			for (const pi of pins) if (!pi.placed) for (const k of cups) if (k.placed && inCupHalf(k, pi.low, 1)) {
				pi.placed = true; pi.goal = k.goal; changed = true;
				if (k.upperOpaque) pi.hideLow = true;
				break;
			}
		}
		const toggleOf = Object.fromEntries(this.toggles.map((t) => [t.quad, t.state]));
		const midOwner = robotsInMid.red > robotsInMid.blue ? 'red' : robotsInMid.blue > robotsInMid.red ? 'blue' : null;
		const s = { red: 0, blue: 0, placedPins: 0, halves: { red: 0, blue: 0, yellowRed: 0, yellowBlue: 0 } };
		for (const pi of pins) {
			if (!pi.placed) continue;
			s.placedPins++;
			const owner = pi.goal.quad === 'mid' ? midOwner : (toggleOf[pi.goal.quad] === 'yellow' ? null : toggleOf[pi.goal.quad]);
			for (const [col, hidden] of [[pi.highColor, pi.hideHigh], [pi.lowColor, pi.hideLow]]) {
				if (hidden) continue;
				if (col === 'red' || col === 'blue') { s[col] += 5; s.halves[col]++; }
				else if (owner) { s[owner] += 10; s.halves[owner === 'red' ? 'yellowRed' : 'yellowBlue']++; }
			}
		}
		s.red += 8 * robotsInMid.red; s.blue += 8 * robotsInMid.blue;
		s.toggles = toggleOf;
		return s;
	}
}
