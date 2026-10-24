// Visual model of 4787T, from the team's photos: silver aluminium drive with four 3.25" omnis
// driven through a row of red gears on each side, black V5 motors, nested blue cascade stages
// rigged with yellow string, a top frame with the distance sensor, the flex-wheel claw on the
// back (two roller stacks counter-rotating through red gears), the goal guide (two hinged arms
// with rubber feet) under it, grey Toggle wheels on the front, license plates. Visual only.
// Robot-local inches: +x = motor forward, +y up, +z right; the claw is at -x.
import * as THREE from 'three';
import { IN } from './field.js';

// ---- Textures / materials -------------------------------------------------------------
// VEX channel face: square holes on a 0.5" grid, a little brushed sheen.
function holeTexture(base, hole) {
	const c = document.createElement('canvas');
	c.width = 64; c.height = 64;
	const g = c.getContext('2d');
	g.fillStyle = base; g.fillRect(0, 0, 64, 64);
	for (let i = 0; i < 64; i += 2) { g.fillStyle = `rgba(255,255,255,${Math.random() * 0.05})`; g.fillRect(i, 0, 1, 64); }
	g.fillStyle = hole; g.fillRect(20, 20, 24, 24);
	g.fillStyle = 'rgba(255,255,255,0.18)'; g.fillRect(20, 44, 24, 2); g.fillRect(44, 20, 2, 26);
	const t = new THREE.CanvasTexture(c);
	t.wrapS = t.wrapT = THREE.RepeatWrapping;
	t.colorSpace = THREE.SRGBColorSpace;
	t.anisotropy = 4;
	return t;
}
const TEX = {};
const KINDS = { blue: ['#2d63cc', '#0a1426', 0.5, 0.34], silver: ['#dfe3e7', '#3a3f45', 0.7, 0.32] };
const MATS = {};
// One material per (kind, length, width) so the hole pattern has the right count along the part.
function channelMat(kind, lengthIn, widthIn) {
	const key = `${kind}_${Math.round(lengthIn * 2)}_${Math.round(widthIn * 2)}`;
	if (MATS[key]) return MATS[key];
	TEX[kind] ||= holeTexture(KINDS[kind][0], KINDS[kind][1]);
	const t = TEX[kind].clone();
	t.needsUpdate = true;
	t.repeat.set(Math.max(1, Math.round(lengthIn * 2)), Math.max(1, Math.round(widthIn * 2)));
	return (MATS[key] = new THREE.MeshStandardMaterial({ map: t, metalness: KINDS[kind][2], roughness: KINDS[kind][3] }));
}
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.5, ...o });
const M = {
	red: mat(0xd01f28, { roughness: 0.42 }), black: mat(0x121212, { roughness: 0.62 }), cap: mat(0x2a2a2a, { roughness: 0.5 }),
	steel: mat(0xa9afb5, { metalness: 0.92, roughness: 0.25 }), gray: mat(0xb8bcc1, { roughness: 0.7 }), grayDark: mat(0x6f7479, { roughness: 0.6 }),
	hub: mat(0x2c2d30, { metalness: 0.2, roughness: 0.55 }), string: mat(0xe7d24a, { roughness: 0.8 }), white: mat(0xf2f2ee, { roughness: 0.55 }),
	chain: mat(0x222222, { metalness: 0.6, roughness: 0.4 }), led: new THREE.MeshBasicMaterial({ color: 0xff2a1f }),
	screen: new THREE.MeshStandardMaterial({ color: 0x0d1a33, emissive: 0x1b4fb0, emissiveIntensity: 0.6, roughness: 0.2 }),
	rubber: mat(0x0d0d0d, { roughness: 0.95 }), motorGrey: mat(0x8d9297, { roughness: 0.55 }), sticker: mat(0xd9dde2, { roughness: 0.8 }),
	cable: mat(0x151515, { roughness: 0.7 }), green: mat(0x2f9d4a, { roughness: 0.5 }), blueCart: mat(0x2b6fdc, { roughness: 0.5 }),
};

// ---- Part builders ---------------------------------------------------------------------
const mesh = (geo, m, parent, x = 0, y = 0, z = 0) => {
	const o = new THREE.Mesh(geo, m);
	o.position.set(x * IN, y * IN, z * IN);
	o.castShadow = o.receiveShadow = true;
	parent?.add(o);
	return o;
};
const basis = (o, A, B, up) => {
	const x = B.clone().sub(A).normalize(), y = new THREE.Vector3(...up).normalize();
	const z = new THREE.Vector3().crossVectors(x, y).normalize();
	y.crossVectors(z, x);
	o.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
};
// C-channel from a to b: a web `rows` inches wide (1 or 2) whose face normal is `up`, with two
// 0.5" flanges standing off it (toward +up), like VEX 1x2x1 / 1x5x1 channel.
function channel(parent, kind, a, b, rows = 1, up = [0, 1, 0], flange = 0.5) {
	const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b), len = A.distanceTo(B);
	const g = new THREE.Group();
	g.position.copy(A.clone().add(B).multiplyScalar(0.5 * IN));
	basis(g, A, B, up);
	mesh(new THREE.BoxGeometry(len * IN, 0.065 * IN, rows * IN), channelMat(kind, len, rows), g);
	for (const s of [-1, 1]) mesh(new THREE.BoxGeometry(len * IN, flange * IN, 0.065 * IN), channelMat(kind, len, flange), g, 0, flange / 2, (s * rows) / 2);
	parent.add(g);
	return g;
}
function rod(parent, m, a, b, r, seg = 8) {
	const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
	const o = mesh(new THREE.CylinderGeometry(r * IN, r * IN, A.distanceTo(B) * IN, seg), m, parent);
	o.position.copy(A.clone().add(B).multiplyScalar(0.5 * IN));
	o.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize());
	return o;
}
// Square VEX shaft.
function shaft(parent, a, b) {
	const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
	const o = mesh(new THREE.BoxGeometry(0.125 * IN, A.distanceTo(B) * IN, 0.125 * IN), M.steel, parent);
	o.position.copy(A.clone().add(B).multiplyScalar(0.5 * IN));
	o.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize());
	return o;
}
const collar = (parent, x, y, z, axis = 'z') => {
	const o = mesh(new THREE.CylinderGeometry(0.28 * IN, 0.28 * IN, 0.3 * IN, 12), M.black, parent, x, y, z);
	if (axis === 'z') o.rotation.x = Math.PI / 2; else if (axis === 'x') o.rotation.z = Math.PI / 2;
	return o;
};
const screw = (parent, x, y, z, axis = 'z') => {
	const o = mesh(new THREE.CylinderGeometry(0.11 * IN, 0.11 * IN, 0.12 * IN, 6), M.steel, parent, x, y, z);
	if (axis === 'z') o.rotation.x = Math.PI / 2; else if (axis === 'x') o.rotation.z = Math.PI / 2;
	return o;
};
const zip = (parent, x, y, z, ry = 0) => {
	const o = mesh(new THREE.TorusGeometry(0.42 * IN, 0.035 * IN, 4, 12), M.white, parent, x, y, z);
	o.rotation.y = ry; return o;
};

const GEARS = {};
function gearGeo(r, teeth, thick = 0.25) {
	const key = `${r}_${teeth}_${thick}`;
	if (GEARS[key]) return GEARS[key];
	const s = new THREE.Shape(), ro = r * IN, ri = (r - 0.1) * IN;
	const n = teeth * 4;
	for (let i = 0; i <= n; i++) {
		const a = (i / n) * Math.PI * 2, rr = i % 4 < 2 ? ro : ri;
		i ? s.lineTo(Math.cos(a) * rr, Math.sin(a) * rr) : s.moveTo(Math.cos(a) * rr, Math.sin(a) * rr);
	}
	const holes = r > 0.9 ? 6 : r > 0.6 ? 4 : 0;
	for (let k = 0; k < holes; k++) {   // lightening holes like the real red gears
		const a = (k / holes) * Math.PI * 2, h = new THREE.Path();
		h.absarc(Math.cos(a) * r * 0.56 * IN, Math.sin(a) * r * 0.56 * IN, r * 0.17 * IN, 0, Math.PI * 2, true);
		s.holes.push(h);
	}
	const sq = new THREE.Path(), q = 0.07 * IN;
	sq.moveTo(-q, -q); sq.lineTo(-q, q); sq.lineTo(q, q); sq.lineTo(q, -q); sq.lineTo(-q, -q); s.holes.push(sq);
	const g = new THREE.ExtrudeGeometry(s, { depth: thick * IN, bevelEnabled: false, curveSegments: 4 });
	g.translate(0, 0, (-thick / 2) * IN);
	return (GEARS[key] = g);
}
// Gear in the x-y plane (axle along z) or flat (axle along y).
const gear = (parent, r, x, y, z, flat = false) => {
	const o = mesh(gearGeo(r, Math.max(12, Math.round(r * 24))), M.red, parent, x, y, z);
	if (flat) o.rotation.x = Math.PI / 2;
	return o;
};

// 3.25" omni: dark hub with two offset rows of grey rollers; axle along z. Returns the spinning group.
function omni(dia) {
	const g = new THREE.Group(), r = dia / 2;
	const hub = mesh(new THREE.CylinderGeometry((r - 0.42) * IN, (r - 0.42) * IN, 1.0 * IN, 24), M.hub, g);
	hub.rotation.x = Math.PI / 2;
	for (const side of [-0.24, 0.24]) {
		const plate = mesh(new THREE.CylinderGeometry((r - 0.25) * IN, (r - 0.25) * IN, 0.08 * IN, 24), M.grayDark, g, 0, 0, side * 1.6);
		plate.rotation.x = Math.PI / 2;
		for (let i = 0; i < 10; i++) {
			const a = ((i + (side > 0 ? 0.5 : 0)) / 10) * Math.PI * 2;
			const rl = mesh(new THREE.CapsuleGeometry(0.21 * IN, 0.36 * IN, 3, 8), M.gray, g, Math.cos(a) * (r - 0.21), Math.sin(a) * (r - 0.21), side);
			rl.rotation.z = a;   // roller axis tangent to the wheel
		}
	}
	return g;
}
// Grey flex / traction wheel with spokes (axle along `axis`).
function flexWheel(parent, r, w, x, y, z, axis = 'y') {
	const g = new THREE.Group(); g.position.set(x * IN, y * IN, z * IN);
	const tire = mesh(new THREE.CylinderGeometry(r * IN, r * IN, w * IN, 24), M.gray, g);
	const face = new THREE.MeshStandardMaterial({ color: 0xd6d9dc, roughness: 0.65 });
	for (const s of [-1, 1]) {
		const d = mesh(new THREE.CylinderGeometry((r - 0.18) * IN, (r - 0.18) * IN, 0.02 * IN, 24), face, g, 0, s * (w / 2 + 0.005), 0);
		void d;
		for (let k = 0; k < 6; k++) {   // spoke windows
			const a = (k / 6) * Math.PI * 2, win = mesh(new THREE.BoxGeometry(r * 0.42 * IN, 0.03 * IN, r * 0.3 * IN), M.grayDark, g, Math.cos(a) * r * 0.55, s * (w / 2 + 0.01), Math.sin(a) * r * 0.55);
			win.rotation.y = -a;
		}
	}
	mesh(new THREE.CylinderGeometry(0.32 * IN, 0.32 * IN, (w + 0.1) * IN, 8), M.hub, g);
	void tire;
	if (axis === 'z') g.rotation.x = Math.PI / 2; else if (axis === 'x') g.rotation.z = Math.PI / 2;
	parent.add(g);
	return g;
}
// V5 Smart Motor (11W): black body, grey cap, cartridge colour dot, the team's round sticker.
// `face` = direction the output shaft points (local axis, sign).
function motor(parent, x, y, z, rotY = 0, cart = M.blueCart, tall = false) {
	const g = new THREE.Group();
	g.position.set(x * IN, y * IN, z * IN); g.rotation.y = rotY;
	const [sx, sy] = tall ? [1.9, 2.6] : [2.6, 2.2];
	mesh(new THREE.BoxGeometry(sx * IN, sy * IN, 1.9 * IN), M.black, g);
	mesh(new THREE.BoxGeometry((sx + 0.02) * IN, 0.4 * IN, 1.92 * IN), M.cap, g, 0, sy / 2 - 0.2, 0);
	for (let i = -2; i <= 2; i++) mesh(new THREE.BoxGeometry(0.08 * IN, 0.9 * IN, 0.04 * IN), M.cap, g, i * 0.38, -0.3, 0.97);
	mesh(new THREE.CircleGeometry(0.32 * IN, 16), M.sticker, g, -0.5, 0.25, 0.97);
	mesh(new THREE.CylinderGeometry(0.28 * IN, 0.28 * IN, 0.08 * IN, 12), cart, g, 0.7, 0.3, -1.0).rotation.x = Math.PI / 2;
	parent.add(g);
	return g;
}
// The team's license plate: alliance-coloured plastic with "4787T".
function plateTexture(alliance) {
	const c = document.createElement('canvas'); c.width = 512; c.height = 200;
	const g = c.getContext('2d');
	g.fillStyle = alliance === 'red' ? '#d8262f' : '#2a8ae0'; g.fillRect(0, 0, 512, 200);
	g.fillStyle = 'rgba(255,255,255,0.15)'; g.fillRect(0, 0, 512, 14);
	g.fillStyle = '#f4f8ff'; g.font = 'bold 150px system-ui, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
	g.fillText('4787T', 256, 108);
	g.font = 'bold 26px system-ui'; g.fillText('VEX', 470, 30);
	const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}

// ---- The robot -------------------------------------------------------------------------
const cx0Top = () => -2.6 + 3.8;   // front edge of the top frame (cascade x + 3.8)
export function buildRobotMesh(CFG) {
	const L = CFG.length, Wd = CFG.width, H = CFG.height, hl = L / 2, hw = Wd / 2;
	const root = new THREE.Group(), tilt = new THREE.Group(), body = new THREE.Group();
	root.add(tilt); tilt.add(body);
	const out = { group: root, tilt, body, wheels: [], clawRollers: [], clawGears: [], stages: [], driveGears: [], liftGears: [], liftLeds: [] };
	const R = body;

	// ---- Drive base: silver channel, omnis, a row of red gears per side ---------------------------
	const track = CFG.trackWidth / 2, wr = CFG.wheelDia / 2, ax = CFG.wheelbase / 2;
	const rw = 1.25, ri = (2 * ax - 2 * rw) / 10;   // wheel gears + 5 idlers in a row between the axles
	for (const s of [-1, 1]) {
		const side = s < 0 ? 'L' : 'R';
		channel(R, 'silver', [-hl, 1.2, s * (hw - 0.55)], [hl, 1.2, s * (hw - 0.55)], 2, [0, 0, -s]);        // outer rail (2" web, flanges in)
		channel(R, 'silver', [-hl + 0.5, 1.2, s * (track - 1.05)], [hl - 0.5, 1.2, s * (track - 1.05)], 2, [0, 0, s]);   // inner rail
		for (const x of [-hl + 0.6, hl - 0.6]) rod(R, M.steel, [x, 0.55, s * (hw - 0.55)], [x, 0.55, s * (track - 1.05)], 0.09);   // standoffs tying the rails
		const gz = s * (track + 0.75);
		for (const x of [-ax, ax]) {
			const pivot = new THREE.Group();
			pivot.position.set(x * IN, wr * IN, s * track * IN);
			pivot.add(omni(CFG.wheelDia));
			R.add(pivot);
			out.wheels.push({ w: pivot, side });
			shaft(R, [x, wr, s * (track - 1.2)], [x, wr, s * (hw - 0.3)]);
			collar(R, x, wr, s * (hw - 0.35));
		}
		// gear train: wheel gear, 5 idlers, wheel gear; every other one turns the other way
		const xs = [-ax];
		for (let k = 0; k < 5; k++) xs.push(-ax + rw + ri * (2 * k + 1));
		xs.push(ax);
		xs.forEach((x, k) => {
			const r = k === 0 || k === 6 ? rw : ri - 0.03;
			const g = gear(R, r, x, wr + 0.0, gz);
			g.rotation.z = k * 0.4;
			out.driveGears.push({ g, side, ratio: (k % 2 ? -1 : 1) * (rw / r) });   // wheel gears turn with the wheels
			if (k > 0 && k < 6) { shaft(R, [x, wr, s * (track - 1.2)], [x, wr, s * (hw - 0.3)]); screw(R, x, wr, s * (hw - 0.2)); }
		});
		// drive motors: two per side, standing on the inner rail, shafts into idlers 2 and 4
		for (const k of [2, 4]) {
			const m = motor(R, xs[k], 3.7, s * (track - 2.1), s < 0 ? Math.PI / 2 : -Math.PI / 2, M.blueCart, true);
			m.rotation.z = 0;
			zip(R, xs[k], 3.3, s * (track - 1.05), Math.PI / 2);
		}
	}
	channel(R, 'silver', [hl - 0.55, 1.2, -hw + 1.1], [hl - 0.55, 1.2, hw - 1.1], 2, [-1, 0, 0]);   // front crossbar (the back is open)
	channel(R, 'silver', [1.8, 0.25, -track + 1.0], [1.8, 0.25, track - 1.0], 5, [0, 1, 0]);        // base plate under the electronics
	// Bumper on the front: 2" blue channel, the face that hits the Toggle (with the wheels above)
	channel(R, 'blue', [hl + 0.3, 1.2, -hw + 0.4], [hl + 0.3, 1.2, hw - 0.4], 2, [-1, 0, 0]);
	// Front crossbar at Toggle height (what hits it; collider in robot.js) on two light uprights.
	channel(R, 'blue', [hl - 0.5, H - 4.6, -hw + 0.9], [hl - 0.5, H - 4.6, hw - 0.9], 1, [-1, 0, 0]);
	for (const s of [-1, 1]) channel(R, 'blue', [hl - 0.5, 2.4, s * (hw - 0.9)], [hl - 0.5, H - 4.1, s * (hw - 0.9)], 1, [-1, 0, 0]);
	for (const s of [-1, 1]) rod(R, M.steel, [hl - 0.5, H - 4.6, s * (hw - 0.9)], [cx0Top(), H - 0.6, s * (hw - 1.3)], 0.09);   // braces back to the tower top
	// ---- Toggle wheels (front left corner): free grey wheels up a diagonal channel ---------------
	const tw0 = [hl - 0.4, 3.0, -hw + 1.0], tw1 = [hl + 0.9, H - 4.2, -hw + 1.0];
	channel(R, 'blue', tw0, tw1, 1, [0, 0, 1]);
	for (let k = 0; k < 4; k++) {
		const f = (k + 0.5) / 4, x = tw0[0] + (tw1[0] - tw0[0]) * f + 1.0, y = tw0[1] + (tw1[1] - tw0[1]) * f;
		rod(R, M.steel, [x - 1.0, y, -hw + 1.0], [x, y, -hw - 0.9], 0.08);
		flexWheel(R, 1.35, 0.9, x, y, -hw - 0.25, 'z');
	}

	// ---- Electronics: Brain, battery, radio, cables -----------------------------------------------
	const brain = new THREE.Group();
	brain.position.set(3.2 * IN, 1.9 * IN, 0);
	mesh(new THREE.BoxGeometry(4.4 * IN, 0.9 * IN, 5.8 * IN), M.black, brain);
	mesh(new THREE.BoxGeometry(3.1 * IN, 0.02 * IN, 4.4 * IN), M.screen, brain, 0.1, 0.46, 0);
	for (let k = 0; k < 7; k++) mesh(new THREE.BoxGeometry(0.25 * IN, 0.2 * IN, 0.4 * IN), M.cap, brain, -2.0, 0, -2.4 + k * 0.8);
	R.add(brain);
	mesh(new THREE.BoxGeometry(5.0 * IN, 1.6 * IN, 2.3 * IN), M.black, R, 3.0, 3.25, -2.2);              // battery
	mesh(new THREE.BoxGeometry(2.4 * IN, 0.6 * IN, 2.3 * IN), M.green, R, 3.0, 4.35, -2.2).material = mat(0x1f1f1f);
	mesh(new THREE.BoxGeometry(1.6 * IN, 2.6 * IN, 0.9 * IN), mat(0x262626), R, 5.5, 4.3, 2.6);           // radio
	mesh(new THREE.BoxGeometry(0.6 * IN, 0.6 * IN, 0.4 * IN), mat(0x3b8f2a, { emissive: 0x2a7a1a, emissiveIntensity: 0.6 }), R, 5.5, 5.4, 2.6);
	const cable = (pts) => {
		const c = new THREE.CatmullRomCurve3(pts.map(([x, y, z]) => new THREE.Vector3(x * IN, y * IN, z * IN)));
		mesh(new THREE.TubeGeometry(c, 24, 0.09 * IN, 6), M.cable, R);
	};
	cable([[1.4, 2.4, -2.4], [0.2, 4.5, -4.5], [-1.5, 6, -track + 1.6]]);
	cable([[1.4, 2.4, 2.4], [0.2, 4.5, 4.5], [-1.5, 6, track - 1.6]]);
	cable([[4.6, 2.4, 0.8], [5.5, 3.5, 2.4]]);
	cable([[1.3, 2.4, 1.0], [-1.0, 9, 3.0], [-3.5, 14, hw - 1.6]]);

	// ---- Cascade: 4 nested 2" C-channel stages per side, rigged with yellow string ---------------
	// Stage 0 is fixed; stage k rises k/3 of the lift travel; the claw rides on stage 3.
	const cx = -2.6, h0 = 2.4, len = H - h0 - 0.4;
	for (let k = 0; k < 4; k++) {
		const g = new THREE.Group();
		R.add(g);
		out.stages.push({ g, frac: k / 3 });
		for (const s of [-1, 1]) {
			const z = s * (hw - 1.1 - k * 0.32), x = cx + k * 0.22;
			channel(g, 'blue', [x, h0 + k * 0.5, z], [x, h0 + k * 0.5 + len - k * 0.7, z], 2, [0, 0, -s]);
			const top = h0 + len - k * 0.2 - 0.6;
			const p = mesh(new THREE.CylinderGeometry(0.45 * IN, 0.45 * IN, 0.22 * IN, 14), M.white, g, x + 0.2, top, z - s * 0.35);
			p.rotation.x = Math.PI / 2;
			for (const dz of [0.12, -0.12]) rod(g, M.string, [x + 0.65, h0 + 0.8 + k * 0.45, z - s * (0.35 + dz)], [x + 0.65, top, z - s * (0.35 + dz)], 0.035, 5);
			for (let j = 1; j < 4; j++) screw(g, x - 0.5, h0 + k * 0.5 + (len - k * 0.7) * (j / 4), z + s * 0.05);
			if (k === 1 || k === 3) zip(g, x, h0 + len * 0.55, z, Math.PI / 2);
		}
	}
	// Top frame on the fixed stage (the box in the photos) + distance sensor + rods + zip ties.
	const top = H - 0.3, fx0 = cx - 1.8, fx1 = cx + 3.8;
	for (const s of [-1, 1]) channel(R, 'blue', [fx0, top, s * (hw - 1.1)], [fx1, top, s * (hw - 1.1)], 1, [0, -1, 0]);
	for (const x of [fx0, fx1]) channel(R, 'blue', [x, top, -hw + 1.1], [x, top, hw - 1.1], 1, [0, -1, 0]);
	for (const s of [-1, 1]) channel(R, 'blue', [fx1, h0, s * (hw - 1.1)], [fx1, top, s * (hw - 1.1)], 1, [-1, 0, 0]);
	for (const z of [-2.2, 2.2]) rod(R, M.steel, [fx0 + 0.2, top + 0.4, z], [fx1 - 0.2, top + 0.4, z], 0.08);
	for (const x of [fx0 + 1.4, fx1 - 1.2]) zip(R, x, top + 0.4, -2.2, Math.PI / 2);
	mesh(new THREE.BoxGeometry(1.2 * IN, 1.0 * IN, 3.0 * IN), M.black, R, fx1 + 0.3, top - 0.9, 0);            // distance sensor
	out.liftLeds.push(mesh(new THREE.BoxGeometry(0.06 * IN, 0.3 * IN, 0.3 * IN), new THREE.MeshBasicMaterial({ color: 0xff2a1f }), R, fx1 + 0.95, top - 0.8, 0.9));
	// Lift motors (3, green cartridge): two stacked on the left tower, one on the right; red output gears.
	for (const [x, y, s] of [[cx - 1.2, 6.2, -1], [cx - 1.2, 9.0, -1], [cx + 1.6, 7.5, 1]]) {
		motor(R, x, y, s * (hw + 0.5), s < 0 ? 0 : Math.PI, M.green);
		const gr = gear(R, 0.75, x + 0.9, y - 0.2, s * (hw + 1.6));
		out.liftGears.push({ g: gr, dir: s });
		shaft(R, [x + 0.9, y - 0.2, s * (hw + 1.3)], [x + 0.9, y - 0.2, s * (hw + 1.8)]);
		const led = mesh(new THREE.BoxGeometry(0.3 * IN, 0.12 * IN, 0.05 * IN), new THREE.MeshBasicMaterial({ color: 0x331010 }), R, x - 0.8, y + 0.85, s * (hw + 1.47));
		out.liftLeds.push(led);
	}
	// License plates on both sides.
	const plateMat = new THREE.MeshStandardMaterial({ map: plateTexture('red'), roughness: 0.45 });
	const plateGeo = new THREE.BoxGeometry(5.2 * IN, 2.1 * IN, 0.06 * IN);
	for (const s of [-1, 1]) {
		const p = mesh(plateGeo, plateMat, R, 1.8, 6.2, s * (hw + 0.08));
		if (s < 0) p.rotation.y = Math.PI;
		channel(R, 'blue', [0.1, 3.2, s * (hw - 0.05)], [0.1, 8.6, s * (hw - 0.05)], 1, [0, 0, -s]);
	}
	const plates = { red: plateMat.map, blue: plateTexture('blue') };
	let shownAlliance = 'red';
	out.setAlliance = (a) => { if (a !== shownAlliance && plates[a]) { plateMat.map = plates[a]; plateMat.needsUpdate = true; shownAlliance = a; } };

	// ---- Claw (on the top stage): two roller stacks that counter-rotate through red gears ---------
	const car = out.stages[3].g, X = CFG.clawX, cwr = CFG.clawWheelR, zs = 1.575 + cwr - 0.3, ct = 6.6;
	for (const s of [-1, 1]) {
		channel(car, 'blue', [X - 1.3, ct, s * (zs - 0.7)], [X + 2.4, ct, s * (zs - 0.7)], 2, [0, 1, 0]);   // top plate over each stack (open in the middle for the pin)
		channel(car, 'blue', [X + 2.4, ct + 0.3, s * (zs + 1.3)], [cx + 1.2, ct + 0.3, s * (zs + 1.3)], 1, [0, 0, s]);   // arms back to the stage
		channel(car, 'blue', [cx + 1.2, ct + 0.3, s * (zs + 1.3)], [cx + 1.2, H - 3.2, s * (zs + 1.3)], 1, [1, 0, 0]);
		channel(car, 'blue', [X + 2.3, 1.5, s * (zs + 0.9)], [X + 2.3, ct, s * (zs + 0.9)], 1, [1, 0, 0]);
		channel(car, 'blue', [X + 2.4, ct, s * (zs + 0.2)], [X + 2.4, ct, s * (zs + 1.4)], 1, [0, 1, 0]);
		const sh = new THREE.Group();
		sh.position.set(X * IN, 0, s * zs * IN);
		shaft(sh, [0, 0.4, 0], [0, ct + 1.0, 0]);
		for (const y of [1.3, 3.1, 4.9]) flexWheel(sh, cwr, 1.0, 0, y, 0, 'y');
		collar(sh, 0, 0.35, 0, 'y');
		gear(sh, zs - 1.18, 0, ct + 0.55, 0, true);
		car.add(sh);
		out.clawRollers.push({ g: sh, dir: s });
	}
	// Idler pair between the shaft gears (so the stacks turn opposite ways), driven by the claw motor.
	for (const s of [-1, 1]) {
		const ig = new THREE.Group(); ig.position.set(X * IN, (ct + 0.55) * IN, s * 0.6 * IN);
		const gm = mesh(gearGeo(0.58, 14), M.red, ig); gm.rotation.x = Math.PI / 2;
		shaft(ig, [0, -0.2, 0], [0, 0.9, 0]);
		car.add(ig);
		out.clawGears.push({ g: ig, dir: -s * ((zs - 1.18) / 0.58) });
	}
	channel(car, 'blue', [X - 1.2, ct + 1.15, -zs - 0.9], [X - 1.2, ct + 1.15, zs + 0.9], 1, [0, 1, 0]);      // brace over the gears
	const cm = new THREE.Group(); cm.position.set((X + 0.2) * IN, (ct + 2.2) * IN, 0);                     // claw motor (5.5W, grey)
	mesh(new THREE.BoxGeometry(2.0 * IN, 1.6 * IN, 1.7 * IN), M.motorGrey, cm);
	mesh(new THREE.BoxGeometry(2.02 * IN, 0.35 * IN, 1.72 * IN), mat(0xbfc3c7), cm, 0, 0.62, 0);
	mesh(new THREE.CircleGeometry(0.3 * IN, 16), M.sticker, cm, -0.4, 0, 0.86);
	car.add(cm);
	channel(car, 'blue', [X + 2.05, 3.4, -3.2], [X + 2.05, 3.4, 3.2], 1, [1, 0, 0]);                     // brace behind the cup
	mesh(new THREE.BoxGeometry(0.6 * IN, 0.9 * IN, 0.9 * IN), M.black, car, X + 1.75, 2.4, 0);          // optical sensor
	mesh(new THREE.BoxGeometry(0.04 * IN, 0.25 * IN, 0.25 * IN), M.led, car, X + 1.43, 2.4, 0);
	for (const s of [-1, 1]) zip(car, X + 2.3, ct - 0.8, s * (zs + 0.9), 0);

	// ---- Goal guide: two hinged arms under the claw, rubber feet on the tips ----------------------
	const gd = CFG.guide, gy = (gd.y0 + gd.y1) / 2;
	channel(R, 'silver', [CFG.clawX + 2.9, 0.9, -track + 1.0], [CFG.clawX + 2.9, 0.9, track - 1.0], 1, [-1, 0, 0]);   // crossbar the arms hang from
	for (const s of [-1, 1]) {
		const [x0, z0] = gd.root, [x1, z1] = gd.tip;
		channel(R, 'blue', [x0, gy - 0.5, s * (z0 + 0.25)], [x1, gy - 0.5, s * (z1 + 0.25)], 1, [0, 0, -s]);
		channel(R, 'blue', [x0, gy - 0.5, s * (z0 + 0.3)], [CFG.clawX + 2.9, gy - 0.5, s * (z0 + 0.3)], 1, [0, 0, -s]);
		// hinge: shoulder screw + rubber band keeping the arm folded in
		rod(R, M.steel, [x0, gy - 1.2, s * (z0 + 0.4)], [x0, gy + 0.8, s * (z0 + 0.4)], 0.1);
		const band = mesh(new THREE.TorusGeometry(0.3 * IN, 0.06 * IN, 4, 12), M.rubber, R, x0 + 0.5, gy, s * (z0 + 0.5));
		band.rotation.x = Math.PI / 2;
		// tip: standoff with the black rubber foot
		rod(R, M.steel, [x1, gy - 0.4, s * (z1 + 0.1)], [x1, gy + 1.0, s * (z1 + 0.1)], 0.12);
		mesh(new THREE.CylinderGeometry(0.42 * IN, 0.46 * IN, 1.4 * IN, 16), M.rubber, R, x1, 1.3, s * (z1 + 0.1));
		screw(R, x1, gy + 1.05, s * (z1 + 0.1), 'y');
	}
	return out;
}
