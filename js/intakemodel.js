// Visual model of the first Titanium robot (ground intake, Sept-Oct 2026): silver chain-driven base,
// wide flex-wheel intake behind a blue bumper, nested blue C-channel cascade stages on both
// sides that telescope with the lift, a top frame with the distance sensor, a big
// rectangular braced claw frame at the back, and the Brain mounted low at the back.
// Visual only (no physics). Robot-local inches: +x forward (intake), +y up, +z right.
import * as THREE from 'three';
import { IN } from './field.js';

// ---- Textures / materials -------------------------------------------------------------
function holeTexture(base, hole, rows) {
	const c = document.createElement('canvas');
	c.width = 64; c.height = 64 * rows;
	const g = c.getContext('2d');
	g.fillStyle = base; g.fillRect(0, 0, c.width, c.height);
	const grad = g.createLinearGradient(0, 0, 64, 0);
	grad.addColorStop(0, 'rgba(255,255,255,0.10)'); grad.addColorStop(0.5, 'rgba(255,255,255,0)'); grad.addColorStop(1, 'rgba(255,255,255,0.08)');
	g.fillStyle = grad; g.fillRect(0, 0, c.width, c.height);
	g.fillStyle = hole;
	for (let r = 0; r < rows; r++) g.fillRect(19, 19 + r * 64, 26, 26);
	g.fillStyle = 'rgba(0,0,0,0.25)'; g.fillRect(0, 0, 64, 3); g.fillRect(0, c.height - 3, 64, 3);
	const t = new THREE.CanvasTexture(c);
	t.wrapS = t.wrapT = THREE.RepeatWrapping;
	t.colorSpace = THREE.SRGBColorSpace;
	t.anisotropy = 4;
	return t;
}
const TEX = {};
const KINDS = {
	blue: ['#2c62c9', '#0b1730', 0.35], silver: ['#c5c9ce', '#33373c', 0.75],
};
function channelMat(kind, lengthIn, rows) {
	const key = kind + rows;
	TEX[key] ||= holeTexture(KINDS[kind][0], KINDS[kind][1], rows);
	const t = TEX[key].clone();
	t.needsUpdate = true;
	t.repeat.set(lengthIn / 0.5, 1);
	return new THREE.MeshStandardMaterial({ map: t, metalness: KINDS[kind][2], roughness: 0.36 });
}
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.5, ...o });
const M = {
	red: mat(0xd3202a, { roughness: 0.42 }), black: mat(0x141414, { roughness: 0.65 }), cap: mat(0x2b2b2b),
	steel: mat(0xa3a9af, { metalness: 0.9, roughness: 0.28 }), gray: mat(0x8c9096, { roughness: 0.75 }),
	hub: mat(0x3a3a3a, { metalness: 0.2 }), string: mat(0xe9d64a, { roughness: 0.7 }),
	poly: mat(0xf2f5f8, { transparent: true, opacity: 0.55, roughness: 0.2, side: THREE.DoubleSide }),
	chain: mat(0x222222, { metalness: 0.6, roughness: 0.4 }), led: new THREE.MeshBasicMaterial({ color: 0xff2a1f }),
	screen: new THREE.MeshBasicMaterial({ color: 0x2a6fd6 }), rubber: mat(0x0e0e0e, { roughness: 0.9 }),
	zip: mat(0xf4f4f0, { roughness: 0.6 }),
};

// ---- Part builders ---------------------------------------------------------------------
const mesh = (geo, m, parent, x = 0, y = 0, z = 0) => {
	const o = new THREE.Mesh(geo, m);
	o.position.set(x * IN, y * IN, z * IN);
	o.castShadow = o.receiveShadow = true;
	parent?.add(o);
	return o;
};
// C-channel from a to b. rows = 1 (1" wide) or 2 (2" wide). `up` = which way the wide face points.
function channel(parent, kind, a, b, rows = 1, up = [0, 1, 0]) {
	const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b), len = A.distanceTo(B);
	const o = mesh(new THREE.BoxGeometry(len * IN, 0.12 * IN, rows * IN), channelMat(kind, len, rows), parent);
	// two flanges so it reads as a C-channel, not a plank
	for (const s of [-1, 1]) mesh(new THREE.BoxGeometry(len * IN, 0.5 * IN, 0.08 * IN), channelMat(kind, len, 1), o, 0, 0.25, (s * rows) / 2);
	o.position.copy(A.clone().add(B).multiplyScalar(0.5 * IN));
	const x = B.clone().sub(A).normalize(), y = new THREE.Vector3(...up).normalize();
	const z = new THREE.Vector3().crossVectors(x, y).normalize();
	y.crossVectors(z, x);
	o.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
	return o;
}
function rod(parent, m, a, b, r) {
	const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
	const o = mesh(new THREE.CylinderGeometry(r * IN, r * IN, A.distanceTo(B) * IN, 8), m, parent);
	o.position.copy(A.clone().add(B).multiplyScalar(0.5 * IN));
	o.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize());
	return o;
}
const GEARS = {};
function gearGeo(r, teeth, thick = 0.25) {
	const key = `${r}_${teeth}`;
	if (GEARS[key]) return GEARS[key];
	const s = new THREE.Shape(), ro = r * IN, ri = (r - 0.13) * IN;
	for (let i = 0; i <= teeth * 2; i++) {
		const a = (i / (teeth * 2)) * Math.PI * 2, rr = i % 2 ? ri : ro;
		i ? s.lineTo(Math.cos(a) * rr, Math.sin(a) * rr) : s.moveTo(Math.cos(a) * rr, Math.sin(a) * rr);
	}
	for (let k = 0; k < 6; k++) {   // lightening holes like the real red gears
		const a = (k / 6) * Math.PI * 2, h = new THREE.Path();
		h.absarc(Math.cos(a) * r * 0.58 * IN, Math.sin(a) * r * 0.58 * IN, r * 0.17 * IN, 0, Math.PI * 2, true);
		s.holes.push(h);
	}
	const g = new THREE.ExtrudeGeometry(s, { depth: thick * IN, bevelEnabled: false });
	g.translate(0, 0, (-thick / 2) * IN);
	return (GEARS[key] = g);
}
const gear = (parent, r, teeth, x, y, z) => mesh(gearGeo(r, teeth), M.red, parent, x, y, z);   // axle along z

function omni(dia) {
	const g = new THREE.Group(), r = dia / 2;
	mesh(new THREE.CylinderGeometry((r - 0.45) * IN, (r - 0.45) * IN, 0.9 * IN, 20), M.hub, g);
	for (const side of [-0.22, 0.22]) for (let i = 0; i < 10; i++) {
		const a = ((i + (side > 0 ? 0.5 : 0)) / 10) * Math.PI * 2;
		const rl = mesh(new THREE.CapsuleGeometry(0.23 * IN, 0.42 * IN, 2, 6), M.gray, g, Math.cos(a) * (r - 0.23), side, Math.sin(a) * (r - 0.23));
		rl.rotation.y = -a; rl.rotation.x = Math.PI / 2;
	}
	return g;
}
function motor(parent, x, y, z, rotY = 0) {
	const g = new THREE.Group();
	g.position.set(x * IN, y * IN, z * IN);
	g.rotation.y = rotY;
	mesh(new THREE.BoxGeometry(2.6 * IN, 2.2 * IN, 1.9 * IN), M.black, g);
	mesh(new THREE.BoxGeometry(2.62 * IN, 0.45 * IN, 1.92 * IN), M.cap, g, 0, 0.9, 0);
	for (let i = -2; i <= 2; i++) mesh(new THREE.BoxGeometry(0.1 * IN, 1.2 * IN, 0.05 * IN), M.cap, g, i * 0.45, -0.2, 0.97);
	mesh(new THREE.CircleGeometry(0.22 * IN, 12), M.zip, g, 0.7, 0.1, 0.98);
	parent.add(g);
	return g;
}

// ---- The robot -------------------------------------------------------------------------
export function buildIntakeMesh(CFG) {
	const L = CFG.length, Wd = CFG.width, H = CFG.height, hl = L / 2, hw = Wd / 2;
	const root = new THREE.Group();
	const out = { group: root, wheels: [], intakeRollers: [], clawRollers: [], pistons: [], stages: [], driveGears: [], liftGears: [], liftLeds: [] };

	// ---- Drive base: silver channels, 4 omnis with red gears, chain along each side -------
	const track = CFG.trackWidth / 2;
	for (const s of [-1, 1]) {
		channel(root, 'silver', [-hl, 1.8, s * (hw - 0.4)], [hl - 0.5, 1.8, s * (hw - 0.4)], 1, [0, 0, s]);
		channel(root, 'silver', [-hl + 1, 1.8, s * (track - 1.2)], [hl - 1, 1.8, s * (track - 1.2)], 1, [0, 0, s]);
		for (const x of [-hl + 2.5, hl - 2.5]) {
			const w = omni(CFG.wheelDia);
			w.position.set(x * IN, (CFG.wheelDia / 2) * IN, s * track * IN);
			w.rotation.x = Math.PI / 2;
			root.add(w);
			out.wheels.push({ w, side: s < 0 ? 'L' : 'R' });
			out.driveGears.push({ g: gear(root, 1.45, 36, x, CFG.wheelDia / 2, s * (track + 0.75)), side: s < 0 ? 'L' : 'R' });
			rod(root, M.steel, [x, CFG.wheelDia / 2, s * (hw - 0.3)], [x, CFG.wheelDia / 2, s * (track - 1.4)], 0.09);
		}
		// chain run between the two wheel gears
		for (const dy of [-1.4, 1.4]) mesh(new THREE.BoxGeometry((L - 5) * IN, 0.14 * IN, 0.2 * IN), M.chain, root, 0, CFG.wheelDia / 2 + dy, s * (track + 0.75));
		motor(root, 0, 2.1, s * (track - 2.6), 0);
	}
	channel(root, 'silver', [-hl + 0.5, 3.3, -hw + 0.4], [-hl + 0.5, 3.3, hw - 0.4], 1, [1, 0, 0]);

	// ---- Front: blue 2" bumper + wide flex-wheel intake + deflector ramp -------------------
	channel(root, 'blue', [hl + 0.3, 1.6, -hw], [hl + 0.3, 1.6, hw], 2, [1, 0, 0]);
	for (const [x, y, n] of [[hl - 1.4, 2.4, 6], [hl - 3, 5.6, 4]]) {
		const axle = new THREE.Group();
		axle.position.set(x * IN, y * IN, 0);
		rod(axle, M.steel, [0, 0, -hw + 0.8], [0, 0, hw - 0.8], 0.09);
		for (let i = 0; i < n; i++) {
			const z = -hw + 2.2 + (i * (Wd - 4.4)) / (n - 1);
			mesh(new THREE.CylinderGeometry(1.0 * IN, 1.0 * IN, 1.0 * IN, 14), M.gray, axle, 0, 0, z).rotation.x = Math.PI / 2;
		}
		for (const z of [-hw + 1.1, hw - 1.1]) gear(axle, 0.95, 24, 0, 0, z);
		root.add(axle);
		out.intakeRollers.push(axle);
	}
	const ramp = mesh(new THREE.PlaneGeometry(9 * IN, (Wd - 4) * IN), M.poly, root, 1.5, 6.2, 0);
	ramp.rotateX(-Math.PI / 2); ramp.rotateY(0.42);

	// ---- Cascade lift: 4 nested 2" C-channel stages per side, telescoping -----------------
	// Stage 0 is fixed; stage k rises k/3 of the lift travel; the claw rides on stage 3.
	const cx = -2.5, h0 = 3.5, len = H - h0;
	for (let k = 0; k < 4; k++) {
		const g = new THREE.Group();
		root.add(g);
		out.stages.push({ g, frac: k / 3 });
		for (const s of [-1, 1]) {
			const z = s * (hw - 0.9 - k * 0.55);
			channel(g, 'blue', [cx + k * 0.25, h0 + k * 0.4, z], [cx + k * 0.25, h0 + k * 0.4 + len - k * 0.6, z], 2, [0, 0, s]);
			mesh(new THREE.CylinderGeometry(0.35 * IN, 0.35 * IN, 0.3 * IN, 10), M.zip, g, cx + 1.1, h0 + len - 0.8 - k * 0.2, z).rotation.x = Math.PI / 2;   // pulley
			for (const dz of [0.15, -0.25]) rod(g, M.string, [cx + 1.1, h0 + 1 + k * 0.4, z + s * dz], [cx + 1.1, h0 + len - 1 - k * 0.2, z + s * dz], 0.04);
			gear(g, 1.1, 30, cx - 0.9, h0 + 1.5 + k * 0.4, z - s * 0.4);
		}
	}
	// Top frame on the fixed stage (the "box" in the photos) + distance sensor.
	const top = H - 0.3, fx0 = cx - 3, fx1 = cx + 4;
	for (const s of [-1, 1]) channel(root, 'blue', [fx0, top, s * (hw - 0.9)], [fx1, top, s * (hw - 0.9)], 1);
	for (const x of [fx0, fx1]) channel(root, 'blue', [x, top, -hw + 0.9], [x, top, hw - 0.9], 1);
	for (const s of [-1, 1]) channel(root, 'blue', [fx1, h0, s * (hw - 0.9)], [fx1, top, s * (hw - 0.9)], 1, [1, 0, 0]);
	rod(root, M.steel, [fx1 - 0.3, 12, -hw + 0.9], [fx1 - 0.3, 12, hw - 0.9], 0.09);
	rod(root, M.steel, [fx0 + 0.3, 15, -hw + 0.9], [fx0 + 0.3, 15, hw - 0.9], 0.09);
	mesh(new THREE.BoxGeometry(1.2 * IN, 1.4 * IN, 3.2 * IN), M.black, root, fx1 + 0.2, top - 1, 0);
	mesh(new THREE.BoxGeometry(0.05 * IN, 0.3 * IN, 0.3 * IN), M.led, root, fx1 + 0.82, top - 0.8, 1);
	// Lift motors on the tower sides, near the top.
	// Each has a red output gear on the outside that spins whenever the cascade moves,
	// and a status LED that lights up while it is driving.
	for (const [x, y, s] of [[cx - 1, 15.2, -1], [cx + 2.2, 9.5, 1], [cx + 2.2, 13.2, 1]]) {
		motor(root, x, y, s * (hw + 1.1), s < 0 ? 0 : Math.PI);
		const gr = gear(root, 0.75, 12, x + 0.8, y - 0.2, s * (hw + 2.2));
		out.liftGears.push({ g: gr, dir: s });
		rod(root, M.steel, [x + 0.8, y - 0.2, s * (hw + 2.1)], [x + 0.8, y - 0.2, s * (hw + 2.5)], 0.09);
		const led = mesh(new THREE.BoxGeometry(0.3 * IN, 0.14 * IN, 0.06 * IN), new THREE.MeshBasicMaterial({ color: 0x331010 }), root, x - 0.9, y + 0.9, s * (hw + 2.08));
		out.liftLeds.push(led);
	}

	// ---- Brain + battery low at the back, screen tilted up --------------------------------
	const brain = new THREE.Group();
	brain.position.set((-hl - 0.4) * IN, 4.8 * IN, 0);
	brain.rotation.z = -0.95;
	mesh(new THREE.BoxGeometry(0.9 * IN, 4.4 * IN, 5.8 * IN), M.black, brain);
	mesh(new THREE.PlaneGeometry(3.4 * IN, 4.4 * IN), M.screen, brain, -0.46, 0.2, 0).rotation.y = -Math.PI / 2;
	root.add(brain);
	mesh(new THREE.BoxGeometry(5.2 * IN, 1.5 * IN, 2.3 * IN), M.black, root, -hl + 4, 4.2, -hw + 3.2);

	// ---- Claw: big rectangular braced frame, pivots at the back of the top stage ----------
	// 0 deg hangs straight down inside the robot; 90 deg sticks straight out the back.
	const car = out.stages[3].g;
	const pivot = new THREE.Group();
	pivot.position.set(-6.5 * IN, (CFG.pinBottomAtHorizontal + 5.5) * IN, 0);   // same pivot height robot.js uses
	car.add(pivot);
	out.carriage = car;
	out.clawPivot = pivot;
	const arm = CFG.clawArm, cw = hw - 1.6;
	rod(pivot, M.steel, [0, 0, -hw + 0.6], [0, 0, hw - 0.6], 0.1);
	for (const s of [-1, 1]) {
		channel(pivot, 'blue', [0, 0.5, s * cw], [0, -arm, s * cw], 1, [1, 0, 0]);
		channel(pivot, 'blue', [0, -arm * 0.45, s * cw], [0, -arm, s * cw * 0.25], 1, [1, 0, 0]);   // diagonal brace
		gear(pivot, 1.25, 36, 0, 0, s * (cw + 0.7));
	}
	channel(pivot, 'blue', [0, -arm, -cw - 0.5], [0, -arm, cw + 0.5], 2, [1, 0, 0]);
	// Grip at the end of the frame: two small rollers either side of the pin.
	for (const dx of [-1.1, 1.1]) {
		const r = new THREE.Group();
		r.position.set(dx * IN, (-arm - 0.3) * IN, 0);
		mesh(new THREE.CylinderGeometry(0.55 * IN, 0.55 * IN, 2.2 * IN, 12), M.rubber, r).rotation.x = Math.PI / 2;
		gear(r, 0.7, 18, 0, 0, 1.3);
		pivot.add(r);
		out.clawRollers.push(r);
	}
	// Piston on the claw frame (does nothing yet).
	mesh(new THREE.CylinderGeometry(0.42 * IN, 0.42 * IN, 3.6 * IN, 14), M.steel, pivot, 0.8, -arm * 0.4, cw - 1.2);
	const pr = mesh(new THREE.CylinderGeometry(0.12 * IN, 0.12 * IN, 2.6 * IN, 8), M.steel, pivot, 0.8, -arm * 0.4 - 2.6, cw - 1.2);
	pr.userData.y0 = pr.position.y;
	out.pistons.push(pr);
	// Goal guide on the back: a channel out from the base, then a V of channel that the goal post
	// slides into (CFG.guide; the colliders are in robot.js).
	const gd = CFG.guide, gy = (gd.y0 + gd.y1) / 2, back = -hl;
	channel(root, 'silver', [back, gy, 0], [gd.apexX, gy, 0], 1, [0, 1, 0]);
	for (const s of [-1, 1]) {
		const len = gd.armLen;
		channel(root, 'silver', [gd.apexX, gy, 0], [gd.apexX - len * Math.SQRT1_2, gy, s * len * Math.SQRT1_2], 1, [0, 1, 0]);
	}
	return out;
}
