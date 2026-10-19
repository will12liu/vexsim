// Human players standing behind the wall at each Loader (Override: Drive Team Members put match
// loads in through their alliance's Loaders). Ours keep our Loaders filled with a cup with a pin in
// it (or just a pin, picked before the match: `load`), at both Loaders or only the left / right one
// as seen from our station (`loader`). Loading: slide the clear tube up, set the stack on the base,
// slide the tube back down (~1 s). The robot backs in square and the claw pulls the stack out
// (robot.js). Only our alliance's human players load; theirs just stand there, greyed out.
import * as THREE from 'three';
import { IN, W, CUP_H, PIN_H, PIN_COLLAR, LOADER, COLORS } from './field.js';

const UP_S = 0.35, DROP_S = 0.7, DOWN_S = 1.05;   // tube up, stack down onto the base, tube down
const COOLDOWN_S = 1.0;                            // after one is taken, before the next goes in
const SCALE = 0.8;                                 // a bit smaller than life so they don't fill the view

export class HumanPlayers {
	constructor(scene, field, toast) {
		this.field = field; this.toast = toast;
		this.t = 0; this.load = 'cupOwn'; this.loader = 'both';
		this.people = field.loaders.map((L) => {
			const shirt = new THREE.MeshStandardMaterial({ color: COLORS[L.alliance], roughness: 0.8, transparent: true });
			const pants = new THREE.MeshStandardMaterial({ color: 0x23262d, roughness: 0.9, transparent: true });
			const skin = new THREE.MeshStandardMaterial({ color: 0xe0b48f, roughness: 0.8, transparent: true });
			const g = new THREE.Group();
			const part = (geo, m, x, y, z, parent = g) => {
				const o = new THREE.Mesh(geo, m);
				o.position.set(x * IN, y * IN, z * IN); parent.add(o); return o;
			};
			const box = (w, h, d) => new THREE.BoxGeometry(w * IN, h * IN, d * IN);
			for (const z of [-3, 3]) part(box(4.5, 31, 5), pants, 0, 15.5, z);            // legs
			part(box(7, 21, 14), shirt, 0, 41.5, 0);                                     // torso
			part(new THREE.SphereGeometry(4.6 * IN, 16, 12), skin, 0, 57, 0);              // head
			part(box(4.5, 22, 4.5), shirt, 0, 41, -9.5);                                  // left arm
			const arm = new THREE.Group();                                               // right arm, pivots at the shoulder
			arm.position.set(0, 51 * IN, 9.5 * IN);
			part(box(4.5, 23, 4.5), shirt, 0, -11.5, 0, arm);
			part(new THREE.SphereGeometry(2.4 * IN, 10, 8), skin, 0, -24, 0, arm);        // hand
			g.add(arm);
			// Glowing spot on the floor under our own human players.
			const ring = new THREE.Mesh(new THREE.RingGeometry(9 * IN, 12 * IN, 32), new THREE.MeshBasicMaterial({ color: COLORS[L.alliance], transparent: true, opacity: 0.7, depthWrite: false }));
			ring.rotation.x = -Math.PI / 2; ring.position.y = 0.01;
			g.add(ring);
			// Behind the wall at the tube, facing the field (world +x is field +fx); right hand toward the tube.
			g.position.copy(W(L.fx - L.n * 13, L.fy + Math.sign(L.fy) * 4, 0));
			g.rotation.y = L.n > 0 ? 0 : Math.PI;
			g.scale.setScalar(SCALE);
			scene.add(g);
			return { L, s: Math.sign(L.fx), sy: Math.sign(L.fy), g, arm, ring, mats: [shirt, pants, skin], phase: -1, readyAt: 0 };
		});
	}

	// Loaders this human player uses. Red stands at -fx looking +fx (left = +fy); blue the other way.
	uses(p, s) {
		if (this.load === 'off' || p.s !== s) return false;
		if (this.loader === 'both') return true;
		const left = p.sy === -s;
		return this.loader === 'left' ? left : !left;
	}

	// The stack still sitting in the tube (not taken, not cleared by a field reset)?
	stocked(L) {
		const f = this.field;
		return !!L.stack && L.stack.every((o) => o.held === 'loader' && (f.pins.includes(o) || f.cups.includes(o)));
	}

	// Make the stack (held by the loader: no contacts, moved by hand) standing `above` the base.
	makeStack(L, alliance, above) {
		const f = this.field, top = this.load.endsWith('Yellow') ? 'yellow' : alliance, b = LOADER.base + above;
		const pieces = this.load.startsWith('cup')
			? [f.makeCup(W(L.px, L.fy, b + CUP_H / 2), false), f.makePin(top, 'yellow', W(L.px, L.fy, b + CUP_H - PIN_COLLAR + 0.06 + PIN_H / 2))]
			: [f.makePin(top, 'yellow', W(L.px, L.fy, b + PIN_H / 2))];
		for (const o of pieces) {
			o.held = 'loader';
			o.loadY = o.body.translation().y / IN - b;   // height above the base
			for (let i = 0; i < o.body.numColliders(); i++) o.body.collider(i).setCollisionGroups(0);
			o.body.setBodyType(f.R.RigidBodyType.KinematicPositionBased, true);
		}
		return pieces;
	}

	step(dt, robot, live) {
		this.t += dt;
		const s = robot.alliance === 'red' ? -1 : 1, ease = (x) => 0.5 - 0.5 * Math.cos(Math.PI * Math.min(1, Math.max(0, x)));
		this.field.setMine((L) => this.people.some((p) => p.L === L && this.uses(p, s)));
		for (const p of this.people) {
			const L = p.L, active = this.uses(p, s);
			// Ours in full colour on a glowing spot; the others faded.
			for (const m of p.mats) m.opacity = active ? 0.85 : 0.3;
			p.ring.visible = active;
			if (!this.stocked(L)) { L.ready = false; if (L.stack && L.stack.some((o) => o.held === 'loader')) L.stack = null; }
			if (p.phase >= 0) {
				const t = (p.phase += dt);
				L.slideY = LOADER.slide * (t < UP_S ? ease(t / UP_S) : t < DROP_S ? 1 : 1 - ease((t - DROP_S) / (DOWN_S - DROP_S)));
				p.arm.rotation.z = 1.3 * Math.min(1, t / 0.25) * (t > DOWN_S - 0.2 ? (DOWN_S - t) / 0.2 : 1);
				if (t >= UP_S && !L.stack) L.stack = this.makeStack(L, robot.alliance, LOADER.slide);
				if (L.stack && this.stocked(L)) {
					const above = LOADER.slide * (1 - ease((t - UP_S) / (DROP_S - UP_S)));
					for (const o of L.stack) o.body.setTranslation(W(L.px, L.fy, LOADER.base + above + o.loadY), true);
				}
				if (t >= DOWN_S) { p.phase = -1; L.slideY = 0; p.arm.rotation.z = 0; L.ready = this.stocked(L); }
				continue;
			}
			L.slideY = 0; p.arm.rotation.z = 0;
			if (!active) {   // not ours (any more): take back anything left in the tube
				if (L.stack) { for (const o of L.stack) if (o.held === 'loader') this.field.removePiece(o); L.stack = null; L.ready = false; }
				continue;
			}
			if (this.stocked(L)) { L.ready = true; p.readyAt = this.t + COOLDOWN_S; continue; }
			if (live && this.t >= p.readyAt) { p.phase = 0; L.stack = null; }
		}
	}

	// For the HUD: near one of our Loaders, is a stack waiting, and is the lift at the right height?
	status(robot) {
		const r = robot.pos(), L = this.field.loaders.find((q) => q.mine && Math.hypot(r.x / IN - q.px, -r.z / IN - q.fy) < 30);
		if (!L) return '';
		if (!L.ready) return 'Loader: refilling';
		const e = robot.liftIn() - LOADER.base;
		return `Loader: ready · ${e < -3 ? `lift up to ${LOADER.base}"` : e > 1.5 ? `lift down to ${LOADER.base}"` : 'lift OK, R1 + back in straight'}`;
	}
}
