// Online play. One browser hosts: it runs the physics as usual, the other players' robots take
// AI slots and are driven by the controller input their browsers send. The host streams back
// where every robot, pin, cup and Toggle is, and the joiners just draw that (smoothed).
// PeerJS (WebRTC, peer to peer) is loaded from a CDN only when someone goes online, so the
// rest of the sim still works with no internet.
import * as THREE from 'three';

const PEER_URL = 'https://cdn.jsdelivr.net/npm/peerjs@1.5.4/+esm';
const PREFIX = 'titanium-sim-v1-';
const CODE_CHARS = 'ABCDEFGHJKLMNPQRSTUVWXYZ23456789';
let PeerLib = null;
async function loadPeer() {
	const url = PEER_URL;   // a variable, so the offline bundler leaves this import alone
	return (PeerLib ||= (await import(/* @vite-ignore */ url)).Peer);
}
const opened = (peer) => new Promise((res, rej) => { peer.on('open', res); peer.on('error', rej); });

export class Net {
	// on: { data(conn, msg), leave(conn) }
	constructor(on) { this.on = on; this.role = null; this.conns = []; this.conn = null; this.code = null; }

	async host() {
		const Peer = await loadPeer();
		for (let tries = 0; ; tries++) {
			this.code = Array.from({ length: 4 }, () => CODE_CHARS[(Math.random() * CODE_CHARS.length) | 0]).join('');
			this.peer = new Peer(PREFIX + this.code);
			try { await opened(this.peer); break; }
			catch (e) { this.peer.destroy(); if (e.type !== 'unavailable-id' || tries > 4) throw e; }   // code taken: pick another
		}
		this.role = 'host';
		this.peer.on('connection', (c) => {
			c.on('open', () => this.conns.push(c));
			c.on('data', (d) => this.on.data(c, d));
			c.on('close', () => { this.conns = this.conns.filter((k) => k !== c); this.on.leave(c); });
		});
		return this.code;
	}

	async join(code) {
		const Peer = await loadPeer();
		this.peer = new Peer();
		await opened(this.peer);
		const c = this.peer.connect(PREFIX + code.trim().toUpperCase(), { reliable: true });
		await new Promise((res, rej) => {
			c.on('open', res);
			this.peer.on('error', (e) => rej(e.type === 'peer-unavailable' ? new Error('No game with that code') : e));
			setTimeout(() => rej(new Error('No answer from that code')), 10000);
		});
		this.role = 'client'; this.conn = c; this.code = code.toUpperCase();
		c.on('data', (d) => this.on.data(c, d));
		c.on('close', () => this.on.leave(c));
		return c;
	}

	id() { return this.peer?.id; }
	send(msg) { if (this.conn?.open) this.conn.send(msg); }
	broadcast(msg) { for (const c of this.conns) if (c.open) c.send(msg); }
	close() { this.peer?.destroy(); this.peer = null; this.role = null; this.conns = []; this.conn = null; this.code = null; }
}

// ---- Snapshots --------------------------------------------------------------------------------
const r4 = (v) => Math.round(v * 1e4) / 1e4;
const pose = (b) => { const t = b.translation(), q = b.rotation(); return [r4(t.x), r4(t.y), r4(t.z), r4(q.x), r4(q.y), r4(q.z), r4(q.w)]; };
const isPiece = (v) => v && typeof v === 'object' && v.body && (v.kind === 'pin' || v.kind === 'cup');

// Everything a robot's mesh and the HUD read: its plain number / text / true-false fields, and
// which pieces it holds (by id). shortcut: nested state (drivetrain wheel spin etc.) isn't sent,
// so joiners see wheels still; send it too if that matters.
function robotState(r) {
	const st = {};
	for (const [k, v] of Object.entries(r)) {
		const t = typeof v;
		if (t === 'number' || t === 'boolean' || t === 'string') st[k] = t === 'number' ? r4(v) : v;
		else if (v === null) st[k] = null;
		else if (isPiece(v)) st[k] = { pid: v.id };
	}
	return st;
}

// Host: what changed since the last call (all of it when full).
export function snapshot(field, robots, sim, full) {
	const seen = (field.netSeen ||= new Map());
	const pieces = [], born = [];
	const all = [...field.pins, ...field.cups];
	for (const o of all) {
		const p = pose(o.body), key = p.join();
		if (!seen.has(o.id)) born.push(o.kind === 'pin' ? [o.id, 'p', o.top, o.bottom] : [o.id, 'c', o.opaqueUp]);
		else if (!full && seen.get(o.id) === key) continue;
		seen.set(o.id, key);
		pieces.push([o.id, ...p]);
	}
	if (full) for (const o of all) if (!born.some((b) => b[0] === o.id)) born.push(o.kind === 'pin' ? [o.id, 'p', o.top, o.bottom] : [o.id, 'c', o.opaqueUp]);
	const ids = new Set(all.map((o) => o.id));
	const dead = [...seen.keys()].filter((id) => !ids.has(id));
	for (const id of dead) seen.delete(id);
	return {
		t: 's', full: !!full, born, dead, pieces,
		robots: robots.map((r) => [...pose(r.body), robotState(r)]),
		toggles: field.toggles.map((tg) => [tg.index, tg.state, ...pose(tg.body).slice(3)]),
		sim: { mode: sim.mode, clock: r4(sim.clock), final: sim.final || null },
	};
}

// Joiner: take in a snapshot. Bodies get a target pose that smooth() eases them to each frame.
export function applySnapshot(s, field, robots, sim) {
	const byId = new Map([...field.pins, ...field.cups].map((o) => [o.id, o]));
	if (s.full) for (const o of byId.values()) if (!s.born.some((b) => b[0] === o.id)) { field.removePiece(o); byId.delete(o.id); }
	for (const id of s.dead) { const o = byId.get(id); if (o) { field.removePiece(o); byId.delete(id); } }
	for (const b of s.born) {
		if (byId.has(b[0])) continue;
		const o = b[1] === 'p' ? field.makePin(b[2], b[3], new THREE.Vector3(0, -5, 0)) : field.makeCup(new THREE.Vector3(0, -5, 0), b[2]);
		o.id = b[0]; o.body.setBodyType(field.R.RigidBodyType.KinematicPositionBased, true);
		for (let i = 0; i < o.body.numColliders(); i++) o.body.collider(i).setCollisionGroups(0);   // drawn only: the host does the physics
		byId.set(o.id, o);
	}
	for (const [id, ...p] of s.pieces) { const o = byId.get(id); if (o) target(o, p, s.full); }
	s.robots.forEach(([...a], i) => {
		const r = robots[i];
		if (!r) return;
		const st = a.pop();
		for (const [k, v] of Object.entries(st)) r[k] = v && v.pid != null ? byId.get(v.pid) || null : v;
		target(r, a, s.full);
	});
	s.toggles.forEach(([index, state, x, y, z, w], i) => {
		const tg = field.toggles[i];
		if (!tg) return;
		tg.index = index; tg.state = state;
		tg.body.setRotation({ x, y, z, w }, true);
	});
	Object.assign(sim, s.sim);
}

function target(o, p, snap) {
	o.netT = p;
	if (snap || !o.netHad) { o.body.setTranslation({ x: p[0], y: p[1], z: p[2] }, true); o.body.setRotation({ x: p[3], y: p[4], z: p[5], w: p[6] }, true); o.netHad = true; }
}

const qa = new THREE.Quaternion(), qb = new THREE.Quaternion();
// Joiner, every frame: ease bodies toward the last pose the host sent (it sends ~30 a second).
export function smooth(objs, dt) {
	const k = 1 - Math.exp(-dt * 18);
	for (const o of objs) {
		const p = o.netT;
		if (!p) continue;
		const t = o.body.translation();
		const far = Math.hypot(p[0] - t.x, p[1] - t.y, p[2] - t.z) > 0.4;   // teleports (resets, loads) snap
		const f = far ? 1 : k;
		o.body.setTranslation({ x: t.x + (p[0] - t.x) * f, y: t.y + (p[1] - t.y) * f, z: t.z + (p[2] - t.z) * f }, true);
		const q = o.body.rotation();
		qa.set(q.x, q.y, q.z, q.w).slerp(qb.set(p[3], p[4], p[5], p[6]), f);
		o.body.setRotation({ x: qa.x, y: qa.y, z: qa.z, w: qa.w }, true);
	}
}
