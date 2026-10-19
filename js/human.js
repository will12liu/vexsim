// Human players standing behind the wall at each corner Loader (Override: Drive Team Members put
// match loads in through their alliance's Loaders, driver period only). Ours keep a cup with a pin
// in it (or just a pin, picked before the match: `load`) standing on the floor in front of our
// Loaders (both, or only the left / right one as seen from our station: `loader`). When the spot is
// empty they put a new one there. The robot drives claw first into it and the claw takes it.
import * as THREE from 'three';
import { IN, W, toField, CUP_H, PIN_H, PIN_COLLAR } from './field.js';
import { CFG } from './robot.js';

const HAND_S = 0.35;       // arm swing before the stack is down
const SWING_S = 0.8;
const COOLDOWN_S = 1.5;    // after one is taken, before the next goes down
const SPOT = 69;           // Loader post position (field inches, |fx| and |fy|)
const LOAD = SPOT - 6.5;   // where the stack stands, in front of the post

export class HumanPlayers {
	constructor(scene, toast) {
		this.toast = toast;
		this.t = 0; this.load = 'cupOwn'; this.loader = 'both';
		const skin = new THREE.MeshStandardMaterial({ color: 0xe0b48f, roughness: 0.8 });
		const pants = new THREE.MeshStandardMaterial({ color: 0x23262d, roughness: 0.9 });
		this.people = [];
		for (const s of [-1, 1]) for (const sy of [-1, 1]) {
			const shirt = new THREE.MeshStandardMaterial({ color: s < 0 ? 0xd62828 : 0x1e6fd9, roughness: 0.8 });
			const g = new THREE.Group();
			const part = (geo, m, x, y, z, parent = g) => {
				const o = new THREE.Mesh(geo, m);
				o.position.set(x * IN, y * IN, z * IN); o.castShadow = true; parent.add(o); return o;
			};
			const box = (w, h, d) => new THREE.BoxGeometry(w * IN, h * IN, d * IN);
			for (const z of [-3.5, 3.5]) part(box(5, 31, 6), pants, 0, 15.5, z);          // legs
			part(box(8, 22, 16), shirt, 0, 42, 0);                                         // torso
			part(new THREE.SphereGeometry(5.2 * IN, 16, 12), skin, 0, 59, 0);              // head
			part(box(5.5, 24, 5), shirt, 0, 42, -10.5);                                    // left arm
			const arm = new THREE.Group();                                                 // right arm, pivots at the shoulder
			arm.position.set(0, 52 * IN, 10.5 * IN);
			part(box(5, 25, 5), shirt, 0, -12.5, 0, arm);
			part(new THREE.SphereGeometry(2.7 * IN, 10, 8), skin, 0, -26, 0, arm);          // hand
			g.add(arm);
			// Faces the field; world +x is field +fx.
			g.position.copy(W(s * 80, sy * SPOT, 0));
			g.rotation.y = s > 0 ? Math.PI : 0;
			scene.add(g);
			this.people.push({ s, sy, g, arm, swing: -1 });
		}
	}

	spot(p) { return { fx: p.s * LOAD, fy: p.sy * LOAD }; }

	// A piece still standing at this Loader's spot?
	stocked(p, field) {
		const sp = this.spot(p);
		return [...field.pins, ...field.cups].some((o) => {
			if (o.held) return false;
			const f = toField(o.body.translation());
			return Math.hypot(f.fx - sp.fx, f.fy - sp.fy) < 3.5 && f.h < 12;
		});
	}

	// Room to put a stack down: nothing there, and the robot isn't parked on the spot (unless it's
	// sitting with its claw mouth right over it, then it goes straight between the rollers).
	clear(p, robot) {
		if (this.stocked(p, robot.field)) return false;
		const sp = this.spot(p), l = robot.toLocal(W(sp.fx, sp.fy, 0));
		const onRobot = Math.abs(l.x) < CFG.length / 2 + 1.8 && Math.abs(l.z) < CFG.width / 2 + 1.8;
		const inMouth = !robot.claw && l.x < CFG.clawX + 1 && Math.abs(l.z) < 1.5;
		return !onRobot || inMouth;
	}

	// Loaders this human player uses. Red stands at -fx looking +fx (left = +fy); blue the other way.
	uses(p, s) {
		if (this.load === 'off' || p.s !== s) return false;
		if (this.loader === 'both') return true;
		const left = p.sy === -s;
		return this.loader === 'left' ? left : !left;
	}

	place(p, robot) {
		const sp = this.spot(p), f = robot.field;
		const top = this.load.endsWith('Yellow') ? 'yellow' : robot.alliance;
		if (this.load.startsWith('cup')) {
			f.makeCup(W(sp.fx, sp.fy, CUP_H / 2 + 0.02), false);   // opaque half down, like the photos
			f.makePin(top, 'yellow', W(sp.fx, sp.fy, CUP_H - PIN_COLLAR + 0.06 + PIN_H / 2));
		} else f.makePin(top, 'yellow', W(sp.fx, sp.fy, PIN_H / 2 + 0.02));
	}

	step(dt, robot, live) {
		this.t += dt;
		const s = robot.alliance === 'red' ? -1 : 1;
		for (const p of this.people) {
			if (p.swing >= 0) {
				p.swing += dt;
				if (!p.placed && p.swing >= HAND_S) { p.placed = true; if (this.clear(p, robot)) this.place(p, robot); p.readyAt = this.t + COOLDOWN_S; }
				p.arm.rotation.z = 1.25 * Math.sin(Math.PI * Math.min(1, p.swing / SWING_S));
				if (p.swing >= SWING_S) p.swing = -1;
				continue;
			}
			p.arm.rotation.z = 0;
			if (!this.uses(p, s) || !live || this.t < (p.readyAt || 0)) continue;
			if (this.stocked(p, robot.field)) { p.readyAt = this.t + COOLDOWN_S; continue; }   // wait a moment after it's taken
			if (this.clear(p, robot)) { p.swing = 0; p.placed = false; }
		}
	}

	// For the HUD: near one of our Loaders, is a stack waiting?
	status(robot) {
		const s = robot.alliance === 'red' ? -1 : 1, f = toField(robot.pos());
		const p = this.people.find((q) => this.uses(q, s) && Math.hypot(f.fx - q.s * LOAD, f.fy - q.sy * LOAD) < 26);
		if (!p) return '';
		return this.stocked(p, robot.field) ? 'Loader: ready' : 'Loader: refilling';
	}
}
