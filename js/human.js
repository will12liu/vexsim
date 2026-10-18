// Human players standing behind the wall at each corner Loader (Override <SG> match loads: Drive
// Team Members put Pins in through the top/back of their alliance's Loader, driver period only).
// Ours loads straight into the robot, two Pins at a time (one onto the intake ramp, one into the
// claw), by themselves whenever the robot sits at its Loader with room for them.
import * as THREE from 'three';
import { IN, W, toField } from './field.js';

const REACH = 20;          // robot centre within this many inches of the Loader post
const HAND_S = 0.35;       // arm swing before the pins arrive
const SWING_S = 0.8;
const COOLDOWN_S = 1.5;
const SPOT = 69;           // Loader post position (field inches, |fx| and |fy|)

export class HumanPlayers {
	constructor(scene, toast) {
		this.toast = toast;
		this.t = 0; this.job = null; this.readyAt = 0;
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

	// The alliance's Loader nearest the robot, if it's close enough.
	near(robot) {
		const s = robot.alliance === 'red' ? -1 : 1, f = toField(robot.pos());
		let best = null, bd = REACH;
		for (const p of this.people) {
			if (p.s !== s) continue;
			const d = Math.hypot(f.fx - s * SPOT, f.fy - p.sy * SPOT);
			if (d < bd) { bd = d; best = p; }
		}
		return best;
	}

	// Something the robot could take right now: the intake ramp, or the claw when it's down.
	room(robot) { return !robot.hopper || (!robot.claw && robot.clawDeg < 150); }

	// Starts a load when the robot is stopped at its Loader with room, in driver period.
	auto(robot, live) {
		if (this.job || !live || this.t < this.readyAt || !this.room(robot)) return;
		const v = robot.body.linvel();
		if (Math.hypot(v.x, v.z) > 0.35) return;   // still driving in
		const p = this.near(robot);
		if (!p) return;
		this.job = { robot, p, t: 0, loaded: false };
		p.swing = 0;
	}

	// Two pins straight into the robot: the claw (if it's down) and the intake ramp.
	load(robot) {
		const field = robot.field, color = robot.alliance, at = robot.toWorld(0, 10, 0);
		let n = 0;
		const give = (slot) => {
			const pin = field.makePin(color, 'yellow', at);
			robot.attachPin(pin, slot);
			pin.flipped = false;
			n++;
		};
		if (!robot.claw && robot.clawDeg < 150) give('claw');
		if (!robot.hopper) give('hopper');
		this.toast(`HUMAN LOAD: ${n} pin${n > 1 ? 's' : ''}`);
	}

	step(dt, robot, live) {
		this.t += dt;
		this.auto(robot, live);
		for (const p of this.people) {
			if (p.swing < 0) { p.arm.rotation.z = 0; continue; }
			p.swing += dt;
			p.arm.rotation.z = 1.25 * Math.sin(Math.PI * Math.min(1, p.swing / SWING_S));
			if (p.swing >= SWING_S) p.swing = -1;
		}
		const j = this.job;
		if (!j) return;
		j.t += dt;
		if (!j.loaded && j.t >= HAND_S) {
			j.loaded = true;
			if (this.near(j.robot) === j.p) this.load(j.robot);
			this.readyAt = this.t + COOLDOWN_S;
		}
		if (j.t >= SWING_S) this.job = null;
	}

	// For the HUD.
	status(robot) {
		if (!this.near(robot)) return '';
		return this.t < this.readyAt ? 'Loader: reloading' : 'Loader: ready';
	}
}
