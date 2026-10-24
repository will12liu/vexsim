// Human players (Drive Team Members) behind the wall at each Loader. <SG11>: during the Driver
// Controlled Period they drop match loads in through the top of their alliance's Loaders: a single
// Pin, a single Cup, or a Cup with a Pin nested in it. Each alliance only has 10 Cups and 10
// alliance/yellow Pins (+1 yellow/yellow) for the whole match, shared by both Loaders and both robots.
// The tube holds two nested stacks; the bottom one is what a robot pulls out through the slot.
// Ours load what was picked on the title menu, at both Loaders or only the left / right one as seen
// from our station. In 2v2 the other alliance's human players keep their Loaders stocked too.
import * as THREE from 'three';
import { IN, W, CUP_H, PIN_H, PIN_COLLAR, LOADER, COLORS } from './field.js';

const REACH_S = 0.55, DONE_S = 1.0;    // pick up -> let go over the tube top, arm back down
const COOLDOWN_S = 0.9;                // before the next one
const SCALE = 0.85;

export class HumanPlayers {
	constructor(scene, field, toast) {
		this.field = field; this.toast = toast;
		this.t = 0; this.load = 'cupOwn'; this.loader = 'both'; this.opponents = false; this.unlimited = false;
		this.people = field.loaders.map((L) => this.buildPerson(scene, L));
	}

	buildPerson(scene, L) {
		const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.8, transparent: true, ...o });
		const shirt = mat(COLORS[L.alliance]), pants = mat(0x2a2f38), skin = mat(0xd9a982), shoe = mat(0xf1f1f1), hair = mat(0x1d1a17), glass = mat(0x99ccee, { roughness: 0.1, opacity: 0.6 });
		const g = new THREE.Group();
		const part = (geo, m, x, y, z, parent = g) => { const o = new THREE.Mesh(geo, m); o.position.set(x * IN, y * IN, z * IN); o.castShadow = true; parent.add(o); return o; };
		const cap = (r, len) => new THREE.CapsuleGeometry(r * IN, len * IN, 4, 10);
		for (const z of [-2.6, 2.6]) { part(cap(2.3, 27), pants, 0, 17, z); part(new THREE.BoxGeometry(5.5 * IN, 2 * IN, 3.6 * IN), shoe, 0.8, 1, z); }
		part(cap(6.2, 13), shirt, 0, 40, 0).scale.set(0.62, 1, 1);           // torso
		part(new THREE.CylinderGeometry(1.7 * IN, 1.9 * IN, 3 * IN, 10), skin, 0, 51.5, 0);   // neck
		const head = part(new THREE.SphereGeometry(4.3 * IN, 18, 14), skin, 0, 56.5, 0);
		part(new THREE.SphereGeometry(4.45 * IN, 18, 10, 0, Math.PI * 2, 0, Math.PI * 0.45), hair, -0.3, 57.2, 0);
		part(new THREE.BoxGeometry(1 * IN, 1.4 * IN, 6.4 * IN), glass, 3.9, 57, 0);   // safety glasses (<S5>)
		void head;
		part(cap(1.6, 19), shirt, 0, 40, -7.6);                                 // left arm, hanging
		const arm = new THREE.Group();                                          // right arm, pivots at the shoulder
		arm.position.set(0, 49 * IN, 7.6 * IN);
		part(cap(1.6, 19), shirt, 0, -10.5, 0, arm);
		part(new THREE.SphereGeometry(1.8 * IN, 10, 8), skin, 0, -21.5, 0, arm);
		g.add(arm);
		const carry = new THREE.Group(); carry.position.set(0, -23 * IN, 0); arm.add(carry);   // where a held load sits
		const ring = new THREE.Mesh(new THREE.RingGeometry(9 * IN, 12 * IN, 32), new THREE.MeshBasicMaterial({ color: COLORS[L.alliance], transparent: true, opacity: 0.7, depthWrite: false }));
		ring.rotation.x = -Math.PI / 2; ring.position.y = 0.01;
		g.add(ring);
		// Behind the wall at the tube, facing the field (local +x), right hand toward the tube.
		const sy = Math.sign(L.fy);
		g.position.copy(W(L.fx - L.n * 15, L.fy + sy * 5, 0));
		g.rotation.y = L.n > 0 ? 0 : Math.PI;
		if ((L.n > 0) === (sy > 0)) { arm.position.z *= -1; }   // the arm nearer the tube does the loading
		g.scale.setScalar(SCALE);
		scene.add(g);
		return { L, s: Math.sign(L.fx), sy, g, arm, ring, mats: [shirt, pants, skin, shoe, hair], phase: -1, readyAt: 0, held: null };
	}

	// Does this person load? Ours per the menu; the other alliance's only in 2v2.
	// Red stands at -fx looking +fx (left = +fy); blue the other way.
	uses(p, mine) {
		if (p.L.alliance !== mine) return this.opponents;
		if (this.load === 'off') return false;
		if (this.loader === 'both') return true;
		const left = p.sy === -p.s;
		return this.loader === 'left' ? left : !left;
	}
	loadFor(p, mine) { return p.L.alliance === mine ? this.load : 'cupOwn'; }

	// What the next drop is, given what's left: { cup, pin: 'own' | 'yellow' | null } or null if nothing.
	nextLoad(kind, left) {
		const cup = kind.startsWith('cup');
		if (cup && left.cups <= 0) return null;
		const wantYellow = kind.endsWith('Yellow');
		const pin = wantYellow && left.yellow > 0 ? 'yellow' : left.pins > 0 ? 'own' : left.yellow > 0 && wantYellow ? 'yellow' : null;
		if (!cup && !pin) return null;
		return { cup, pin };
	}

	// Build the stack (held by the loader: kinematic, solid to robots) hanging above the tube.
	makeStack(L, what) {
		const f = this.field, top = what.pin === 'yellow' ? 'yellow' : L.alliance, b = LOADER.h + 1.5;
		const pieces = [];
		if (what.cup) pieces.push(f.makeCup(W(L.px, L.fy, b + CUP_H / 2), false));   // clear half up
		if (what.pin) pieces.push(f.makePin(top, 'yellow', W(L.px, L.fy, b + (what.cup ? CUP_H - PIN_COLLAR + 0.06 : 0) + PIN_H / 2)));
		for (const o of pieces) {
			o.held = 'loader';
			o.loadY = o.body.translation().y / IN - b;
			o.body.setBodyType(f.R.RigidBodyType.KinematicPositionBased, true);
		}
		pieces.step = what.cup ? LOADER.stackStep : PIN_H - 0.4;
		return pieces;
	}

	step(dt, mineAlliance, live) {
		this.t += dt;
		const f = this.field, ease = (x) => 0.5 - 0.5 * Math.cos(Math.PI * Math.min(1, Math.max(0, x)));
		f.setMine((L) => L.alliance === mineAlliance && this.people.some((p) => p.L === L && this.uses(p, mineAlliance)));
		for (const p of this.people) {
			const L = p.L, active = this.uses(p, mineAlliance);
			for (const m of p.mats) m.opacity = active ? 0.92 : 0.3;
			p.ring.visible = active && L.alliance === mineAlliance;
			if (p.phase >= 0) {
				const t = (p.phase += dt);
				p.arm.rotation.z = 2.5 * (t < REACH_S ? ease(t / REACH_S) : 1 - ease((t - REACH_S) / (DONE_S - REACH_S)));
				if (t >= REACH_S && p.pending) {
					// Let go over the top: it falls down the tube onto whatever is already in it.
					const st = this.makeStack(L, p.pending); p.pending = null;
					st.fall = LOADER.h + 1.5 - f.loaderHeight(L);
					L.stacks.push(st);
				}
				if (t >= DONE_S) { p.phase = -1; p.arm.rotation.z = 0; p.readyAt = this.t + COOLDOWN_S; }
				continue;
			}
			if (!active) {   // not loading (any more): take back anything left in the tube
				if (L.stacks.length) { for (const st of L.stacks) for (const o of st) if (o.held === 'loader' && f.has(o)) f.removePiece(o); L.stacks = []; f.syncLoaderStack(L); }
				continue;
			}
			if (!live || this.t < p.readyAt || L.stacks.length >= LOADER.cap || f.loaderHeight(L) + 7 > LOADER.h) continue;
			const left = this.unlimited ? { cups: 99, pins: 99, yellow: 99 } : f.loads[L.alliance];
			const what = this.nextLoad(this.loadFor(p, mineAlliance), left);
			if (!what) continue;
			if (!this.unlimited) {
				if (what.cup) left.cups--;
				if (what.pin === 'yellow') left.yellow--; else if (what.pin) left.pins--;
			}
			p.pending = what;
			p.phase = 0;
		}
	}

	// For the HUD: near one of our Loaders, what's in it and is the claw set to take it?
	status(robot) {
		const r = robot.pos(), L = this.field.loaders.find((q) => q.mine && Math.hypot(r.x / IN - q.px, -r.z / IN - q.fy) < 30);
		if (!L) return '';
		if (!L.ready) return L.stacks.length ? 'Loader: dropping' : 'Loader: empty';
		const high = robot.liftIn() + 9.4 > LOADER.slotTop + 0.2;
		return `Loader: ${L.stacks.length} in · ${high ? 'cascade too high for the slot' : 'R1, back in square, SLOW'}`;
	}
}
