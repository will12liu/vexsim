// Visual model of the DR4B robot, from the team's Onshape CAD (main assembly): silver drive with
// three 2.75" wheels a side (omni, traction in the middle, omni) on 48T gears and four blue motors,
// a double reverse four-bar on two towers with 60T gears and purple gussets at the middle joint,
// the flex-wheel claw (grey housing, roller columns at its back corners, two 5.5W motors on top)
// riding on the end of the lift at the back, and the pneumatic gear pincer (blue + white jaws) with
// its red air tank on the front. Visual only. Robot-local inches: +x = front (pincer), +y up,
// +z right; the claw is at -x.
import * as THREE from 'three';
import { IN } from './field.js';
import { M, mat, mesh, channel, rod, shaft, collar, screw, zip, gear, gearGeo, omni, motor, flexWheel, plateTexture } from './robotmodel.js';

const PURPLE = mat(0x8a3fb0, { roughness: 0.5 });
const PLASTIC = mat(0x5e6166, { roughness: 0.55, side: THREE.DoubleSide });
const JAW = { blue: mat(0x2f5fb8, { roughness: 0.45 }), white: mat(0xe8eaec, { roughness: 0.5 }) };

// 2.75" traction wheel: dark tread, grey hub (axle along z).
function traction(dia) {
	const g = new THREE.Group(), r = dia / 2;
	const tire = mesh(new THREE.CylinderGeometry(r * IN, r * IN, 1.0 * IN, 28), M.rubber, g);
	tire.rotation.x = Math.PI / 2;
	for (const s of [-1, 1]) {
		const hub = mesh(new THREE.CylinderGeometry((r - 0.3) * IN, (r - 0.3) * IN, 0.04 * IN, 20), M.grayDark, g, 0, 0, s * 0.52);
		hub.rotation.x = Math.PI / 2;
	}
	for (let k = 0; k < 14; k++) {   // tread blocks
		const a = (k / 14) * Math.PI * 2, b = mesh(new THREE.BoxGeometry(0.12 * IN, 0.06 * IN, 0.95 * IN), M.black, g, Math.cos(a) * (r + 0.01), Math.sin(a) * (r + 0.01), 0);
		b.rotation.z = a;
	}
	return g;
}

// A flat jaw of the pincer: a C-ring around the stack plus a geared hub at its pivot. Built in the
// pivot's frame (x-z plane), side s = +1 covers +z. Returns the group to rotate about y.
function jawGeo(s, c, ri, ro) {
	const sh = new THREE.Shape(), n = 18, a0 = s > 0 ? 0.12 : -0.12, a1 = s > 0 ? Math.PI * 0.96 : -Math.PI * 0.96;
	// The shape lives in x / v (v = -z once laid flat).
	const P = (r, a) => [c[0] + Math.cos(a) * r, -(c[1] + Math.sin(a) * r)];
	for (let i = 0; i <= n; i++) { const [x, v] = P(ro, a0 + ((a1 - a0) * i) / n); i ? sh.lineTo(x * IN, v * IN) : sh.moveTo(x * IN, v * IN); }
	for (let i = n; i >= 0; i--) { const [x, v] = P(ri, a0 + ((a1 - a0) * i) / n); sh.lineTo(x * IN, v * IN); }
	const g = new THREE.ExtrudeGeometry(sh, { depth: 0.125 * IN, bevelEnabled: false, curveSegments: 6 });
	g.rotateX(-Math.PI / 2);
	return g;
}

export function buildDr4bMesh(C) {
	const L = C.length, Wd = C.width, hl = L / 2, hw = Wd / 2;
	const root = new THREE.Group(), tilt = new THREE.Group(), body = new THREE.Group();
	root.add(tilt); tilt.add(body);
	const out = { group: root, tilt, body, wheels: [], clawRollers: [], clawGears: [], stages: [], driveGears: [], liftGears: [], liftLeds: [], arms: [], mids: [], jaws: [] };
	const R = body;

	// ---- Drive: two rails a side with the wheels between them, 48T gears on every axle -----------
	const track = C.trackWidth / 2, wr = C.wheelDia / 2, wy = wr;
	for (const s of [-1, 1]) {
		const side = s < 0 ? 'L' : 'R';
		channel(R, 'silver', [-hl, 1.2, s * (hw - 0.5)], [hl, 1.2, s * (hw - 0.5)], 2, [0, 0, -s]);
		channel(R, 'silver', [-hl + 0.3, 1.2, s * (track - 0.75)], [hl - 0.3, 1.2, s * (track - 0.75)], 2, [0, 0, s]);
		for (const x of [-hl + 0.4, hl - 0.4]) rod(R, M.steel, [x, 0.5, s * (hw - 0.5)], [x, 0.5, s * (track - 0.75)], 0.09);
		for (const [x, kind] of C.wheelX) {
			const pivot = new THREE.Group();
			pivot.position.set(x * IN, wy * IN, s * track * IN);
			pivot.add(kind === 'omni' ? omni(C.wheelDia) : traction(C.wheelDia));
			R.add(pivot);
			out.wheels.push({ w: pivot, side });
			shaft(R, [x, wy, s * (track - 1.0)], [x, wy, s * (hw - 0.2)]);
			collar(R, x, wy, s * (hw - 0.25));
			const g = gear(R, 1.0, x, wy, s * (track - 1.25));   // 48T HS on each wheel
			out.driveGears.push({ g, side, ratio: 1 });
		}
		// Idler 48Ts between the middle and front wheels, and the motors' 36T pinions above the row.
		for (const [x, k] of [[0.45, -1], [2.6, 1]]) out.driveGears.push({ g: gear(R, 1.0, x, wy, s * (track - 1.25)), side, ratio: k });
		for (const x of C.motorX) {
			out.driveGears.push({ g: gear(R, 0.75, x, wy + 1.55, s * (track - 1.25)), side, ratio: -1.33 });
			motor(R, x, wy + 1.55, s * (track - 2.45), s < 0 ? Math.PI / 2 : -Math.PI / 2, M.blueCart, false);
			zip(R, x, wy + 1.0, s * (track - 1.6), Math.PI / 2);
		}
	}
	channel(R, 'silver', [hl - 0.5, 1.2, -track + 0.75], [hl - 0.5, 1.2, track - 0.75], 2, [-1, 0, 0]);        // front crossbar
	channel(R, 'silver', [C.backBarX, 1.2, -track + 0.75], [C.backBarX, 1.2, track - 0.75], 2, [1, 0, 0]);   // crossbar in front of the claw (the back is open)
	for (const s of [-1, 1]) channel(R, 'silver', [C.backBarX + 0.3, 2.4, s * (track - 1.2)], [hl - 1.0, 2.4, -s * (track - 1.2)], 1, [0, 1, 0]);   // X brace
	// Air tank across the front, Brain + battery in the middle.
	const tank = mesh(new THREE.CylinderGeometry(0.72 * IN, 0.72 * IN, 8.6 * IN, 20), M.red, R, hl - 2.6, 2.9, 0);
	tank.rotation.x = Math.PI / 2;
	for (const s of [-1, 1]) { const cap = mesh(new THREE.CylinderGeometry(0.74 * IN, 0.74 * IN, 0.35 * IN, 20), M.steel, R, hl - 2.6, 2.9, s * 4.4); cap.rotation.x = Math.PI / 2; }
	mesh(new THREE.BoxGeometry(0.35 * IN, 0.8 * IN, 0.35 * IN), mat(0xc9a227, { metalness: 0.8, roughness: 0.3 }), R, hl - 2.6, 3.8, 3.6);   // fitting
	const brain = new THREE.Group(); brain.position.set(0.9 * IN, 3.0 * IN, 0);
	mesh(new THREE.BoxGeometry(4.4 * IN, 0.9 * IN, 5.8 * IN), M.black, brain);
	mesh(new THREE.BoxGeometry(3.1 * IN, 0.02 * IN, 4.4 * IN), M.screen, brain, 0.1, 0.46, 0);
	R.add(brain);
	mesh(new THREE.BoxGeometry(2.3 * IN, 1.4 * IN, 5.0 * IN), M.black, R, -0.6, 1.4, 0);   // battery
	const plateMat = new THREE.MeshStandardMaterial({ map: plateTexture('red'), roughness: 0.45 });
	for (const s of [-1, 1]) {
		const p = mesh(new THREE.BoxGeometry(5.2 * IN, 2.1 * IN, 0.06 * IN), plateMat, R, 1.0, 3.4, s * (hw + 0.08));
		if (s < 0) p.rotation.y = Math.PI;
		channel(R, 'silver', [-1.4, 2.4, s * (hw - 0.05)], [3.4, 2.4, s * (hw - 0.05)], 1, [0, 1, 0]);
	}
	const plates = { red: plateMat.map, blue: plateTexture('blue') };
	let shown = 'red';
	out.setAlliance = (a) => { if (a !== shown && plates[a]) { plateMat.map = plates[a]; plateMat.needsUpdate = true; shown = a; } };

	// ---- DR4B: towers, lift motors + 60T gears at the top pivots ----------------------------------
	const D = C.dr4b, TX = D.towerX;
	for (const s of [-1, 1]) {
		const z = s * D.towerZ;
		channel(R, 'silver', [TX, 1.2, z], [TX, D.pivotY + 0.8, z], 2, [0, 0, -s]);
		channel(R, 'silver', [TX + 0.2, 1.6, z], [TX + 5.5, 1.6, z], 1, [0, 1, 0]);
		rod(R, M.steel, [TX + 5.2, 1.8, z], [TX + 0.3, D.pivotY - 3.5, z], 0.1);    // front brace
		rod(R, M.steel, [TX - 2.8, 1.8, z], [TX - 0.3, D.pivotY - 6, z], 0.1);      // back brace
		for (const dy of [0, -D.barGap]) {
			const g = gear(R, 1.25, TX, D.pivotY + dy, s * (D.towerZ + 0.5));
			out.liftGears.push({ g, dir: s * (dy ? -1 : 1), ratio: 1 });
		}
		motor(R, TX - 1.6, D.pivotY - 3.6, s * (D.towerZ - 1.2), s < 0 ? 0 : Math.PI, M.green);
		const pg = gear(R, 0.3, TX - 0.9, D.pivotY - 2.6, s * (D.towerZ + 0.5));
		out.liftGears.push({ g: pg, dir: -s * 4, ratio: 1 });
		const led = mesh(new THREE.BoxGeometry(0.3 * IN, 0.12 * IN, 0.05 * IN), new THREE.MeshBasicMaterial({ color: 0x331010 }), R, TX - 2.3, D.pivotY - 2.5, s * (D.towerZ - 0.2));
		out.liftLeds.push(led);
	}
	mesh(new THREE.BoxGeometry(0.9 * IN, 0.9 * IN, 0.5 * IN), M.black, R, TX, D.pivotY, -(D.towerZ + 1.0));   // rotation sensor on the arm axle
	channel(R, 'silver', [TX, 1.4, -D.towerZ], [TX, 1.4, D.towerZ], 2, [0, 1, 0]);

	// Arms: four bars per side (two per stage), each built along +x from its pivot; syncMesh places them.
	for (const s of [-1, 1]) for (const stage of [1, 2]) for (const i of [0, 1]) {
		const g = new THREE.Group(), z = s * (stage === 1 ? D.stage1Z : D.stage2Z);
		channel(g, 'silver', [0, 0, 0], [D.arm, 0, 0], 1, [0, 0, s]);
		for (const x of [0, D.arm]) { const c = mesh(new THREE.CylinderGeometry(0.22 * IN, 0.22 * IN, 0.6 * IN, 10), M.steel, g, x, 0, 0); c.rotation.x = Math.PI / 2; }
		g.position.z = z * IN;
		R.add(g);
		out.arms.push({ g, s, stage, i });
	}
	// Middle joint: a purple gusset each side with two meshing 60T gears (they keep the stages mirrored).
	for (const s of [-1, 1]) {
		const g = new THREE.Group();
		const gs = new THREE.Shape([[-0.6, 0.8], [0.8, 0.8], [0.9, -D.barGap - 0.9], [-0.7, -D.barGap - 0.9]].map(([x, y]) => new THREE.Vector2(x * IN, y * IN)));
		const pl = mesh(new THREE.ExtrudeGeometry(gs, { depth: 0.1 * IN, bevelEnabled: false }), PURPLE, g, 0, 0, s * 0.55);
		pl.position.z -= 0.05 * IN;
		const g1 = gear(g, 0.75, 0, 0, s * 0.85), g2 = gear(g, 0.75, 0, -D.barGap, s * 0.85);
		channel(g, 'silver', [0, 1.0, 0], [0, -D.barGap - 1.0, 0], 1, [-1, 0, 0]);
		g.position.z = s * ((D.stage1Z + D.stage2Z) / 2) * IN;
		R.add(g);
		out.mids.push({ g, gears: [g1, g2] });
	}

	// ---- Claw on the end of the lift: rides up and down with it (stage frac 1) ---------------------
	const car = new THREE.Group();
	R.add(car);
	out.stages.push({ g: car, frac: 1 });
	const X = C.clawX, cr = C.claw;
	const housing = mesh(new THREE.CylinderGeometry(cr.housingR * IN, (cr.housingR + 0.15) * IN, cr.h * IN, 8, 1, true, Math.PI / 2 - 1.95, 3.9), PLASTIC, car, X, 0.4 + cr.h / 2, 0);
	void housing;
	const topRing = new THREE.RingGeometry(1.75 * IN, (cr.housingR + 0.1) * IN, 8, 1, -1.95, 3.9);
	const tr = mesh(topRing, PLASTIC, car, X, 0.4 + cr.h, 0);
	tr.rotation.x = -Math.PI / 2;
	for (const s of [-1, 1]) {
		const sh = new THREE.Group();
		sh.position.set(cr.rollerX * IN, 0, s * cr.rollerZ * IN);
		shaft(sh, [0, 0.4, 0], [0, cr.h + 1.4, 0]);
		flexWheel(sh, 0.81, 1.0, 0, 1.0, 0, 'y');
		flexWheel(sh, 1.0, 1.0, 0, 2.5, 0, 'y');
		flexWheel(sh, 0.81, 1.0, 0, 4.0, 0, 'y');
		gear(sh, 0.75, 0, 5.0, 0, true);
		collar(sh, 0, 0.35, 0, 'y');
		car.add(sh);
		out.clawRollers.push({ g: sh, dir: s });
		// 5.5W motor standing on the top plate over each roller column
		const cm = new THREE.Group(); cm.position.set((cr.rollerX - 0.2) * IN, (cr.h + 1.9) * IN, s * (cr.rollerZ + 0.2) * IN);
		cm.rotation.y = s * 0.5;
		mesh(new THREE.BoxGeometry(1.7 * IN, 1.5 * IN, 1.5 * IN), M.black, cm);
		mesh(new THREE.BoxGeometry(1.72 * IN, 0.3 * IN, 1.52 * IN), M.cap, cm, 0, 0.6, 0);
		car.add(cm);
		channel(car, 'silver', [cr.rollerX - 0.5, cr.h + 0.6, s * cr.rollerZ], [X + 1.8, cr.h + 0.6, s * 2.0], 1, [0, 1, 0]);   // top bracket
		channel(car, 'silver', [cr.rollerX - 0.5, 0.5, s * cr.rollerZ], [X + 1.8, 0.5, s * 2.0], 1, [0, -1, 0]);              // bottom bracket
		// out to the end of the lift
		channel(car, 'silver', [X + 1.9, D.endY - 0.75, s * 2.3], [TX, D.endY - 0.75, s * D.endZ], 1, [0, 1, 0]);
		channel(car, 'silver', [TX, D.endY - D.barGap - 0.9, s * D.endZ], [TX, D.endY + 0.9, s * D.endZ], 1, [-s, 0, 0]);
		for (const dy of [0, -D.barGap]) collar(car, TX, D.endY + dy, s * (D.endZ + 0.3));
	}
	out.clawLight = new THREE.MeshBasicMaterial({ color: 0xffffff });
	mesh(new THREE.BoxGeometry(0.5 * IN, 0.35 * IN, 3.2 * IN), out.clawLight, car, X + 1.6, 0.4 + cr.h + 0.25, 0);
	for (const s of [-1, 1]) zip(car, X + 1.2, cr.h - 0.8, s * 2.3, 0);

	// ---- Pincer on the front: two geared jaws on a 25 mm pneumatic cylinder ----------------------
	const P = C.pincer;
	for (const s of [-1, 1]) {
		const g = new THREE.Group();
		g.position.set(P.pivotX * IN, 0, s * P.pivotZ * IN);
		const c = [P.x - P.pivotX, -s * P.pivotZ];
		const geo = jawGeo(s, c, P.ri, P.ro), m = s < 0 ? JAW.blue : JAW.white;
		for (const y of P.plateY) mesh(geo, m, g, 0, y, 0);
		const hub = mesh(gearGeo(0.62, 15, 0.12), m, g, 0, P.plateY[0] + 0.06, 0); hub.rotation.x = Math.PI / 2;
		const a = s > 0 ? Math.PI * 0.55 : -Math.PI * 0.55;
		for (const k of [0.25, 0.75]) rod(g, M.steel, [c[0] + Math.cos(a * k * 1.6) * (P.ri + P.ro) / 2, P.plateY[0], c[1] + Math.sin(a * k * 1.6) * (P.ri + P.ro) / 2], [c[0] + Math.cos(a * k * 1.6) * (P.ri + P.ro) / 2, P.plateY[1] + 0.12, c[1] + Math.sin(a * k * 1.6) * (P.ri + P.ro) / 2], 0.12);
		rod(g, M.steel, [0, P.plateY[0] - 0.2, 0], [0, P.plateY[1] + 0.3, 0], 0.1);
		R.add(g);
		out.jaws.push({ g, s });
	}
	channel(R, 'silver', [hl - 0.2, P.plateY[1] + 0.45, -2.2], [hl - 0.2, P.plateY[1] + 0.45, 2.2], 1, [0, -1, 0]);   // jaw mount
	const cyl = mesh(new THREE.CylinderGeometry(0.32 * IN, 0.32 * IN, 2.6 * IN, 14), M.red, R, hl + 0.1, P.plateY[0] + 0.9, -2.6);
	cyl.rotation.x = Math.PI / 2;
	out.pincerRod = rod(R, M.steel, [hl + 0.1, P.plateY[0] + 0.9, -1.3], [hl + 0.1, P.plateY[0] + 0.9, -0.2], 0.08);
	return out;
}
