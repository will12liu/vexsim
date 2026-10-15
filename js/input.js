// Turns an Xbox (or any standard) gamepad and/or the keyboard into a V5 controller state:
// { ly, rx (-127..127), L1, L2, R1, R2, A, B, X, Y, Up, Down, Left, Right }.

// V5 face buttons sit A = right, B = bottom, X = top, Y = left. 'position' keeps your thumb
// where it is on the V5 controller; 'letter' matches the printed letters on the Xbox pad.
const FACE = {
	position: { A: 1, B: 0, X: 3, Y: 2 },
	letter: { A: 0, B: 1, X: 2, Y: 3 },
};
// Keyboard matches the controller letters where it can: R/G = lift (L1/L2), E = R1,
// Q = R2, and F / B / X / Y = the V5 A / B / X / Y buttons.
const KEYS = {
	L1: ['KeyR'], L2: ['KeyG'], R1: ['KeyE'], R2: ['KeyQ'],
	A: ['KeyF'], B: ['KeyB'], X: ['KeyX'], Y: ['KeyY'],
	Up: ['ArrowUp'], Down: ['ArrowDown'], Left: ['ArrowLeft'], Right: ['ArrowRight'],
};

// Xbox sticks don't rest exactly at 0 (worn ones sit 5-15% off), which made the robot creep and
// wander. Ignore the first 12% of travel, then scale so full stick is still 127.
const DEAD = 0.12;
const dz = (a = 0) => (Math.abs(a) < DEAD ? 0 : (Math.sign(a) * (Math.abs(a) - DEAD)) / (1 - DEAD));

export class Input {
	constructor() {
		this.keys = new Set();
		this.layout = 'position';
		this.external = null;           // filled by the V5 serial bridge later
		addEventListener('keydown', (e) => {
			if (e.target.closest?.('textarea, input, select')) return;
			this.keys.add(e.code);
			if (e.code.startsWith('Arrow') || e.code === 'Space') e.preventDefault();
		});
		addEventListener('keyup', (e) => this.keys.delete(e.code));
		addEventListener('blur', () => this.keys.clear());
	}

	gamepad() {
		for (const g of navigator.getGamepads?.() || []) if (g && g.connected) return g;
		return null;
	}

	rumble(ms) {
		this.gamepad()?.vibrationActuator?.playEffect?.('dual-rumble', { duration: ms, strongMagnitude: 0.6, weakMagnitude: 0.6 }).catch?.(() => {});
	}

	poll() {
		const c = { ly: 0, rx: 0 };
		for (const k of Object.keys(KEYS)) c[k] = KEYS[k].some((code) => this.keys.has(code));
		// Keys are on/off, so the stick value ramps: a tap moves a little, holding goes full speed
		// (~0.4 s to full for driving, ~0.25 s for turning). Releasing stops right away.
		const has = (code) => this.keys.has(code), now = performance.now();
		const dt = Math.min(0.1, (now - (this.lastPoll || now)) / 1000);
		this.lastPoll = now;
		const ramp = (cur, want, perSec) => (want === 0 || Math.sign(want) !== Math.sign(cur) ? Math.sign(want) * Math.min(Math.abs(want), perSec * dt)
			: Math.sign(want) * Math.min(Math.abs(want), Math.abs(cur) + perSec * dt));
		const half = has('ShiftLeft') || has('ShiftRight') ? 0.5 : 1;
		this.kly = ramp(this.kly || 0, ((has('KeyW') ? 127 : 0) - (has('KeyS') ? 127 : 0)) * half, 320);
		this.krx = ramp(this.krx || 0, ((has('KeyD') ? 127 : 0) - (has('KeyA') ? 127 : 0)) * half, 500);
		c.ly = Math.round(this.kly); c.rx = Math.round(this.krx);
		const g = this.gamepad();
		this.source = this.external ? 'V5 controller (USB)' : g ? `Gamepad: ${g.id.slice(0, 40)}` : 'Keyboard';
		const merge = (s) => {
			if (Math.abs(s.ly) > Math.abs(c.ly)) c.ly = s.ly;
			if (Math.abs(s.rx) > Math.abs(c.rx)) c.rx = s.rx;
			for (const k of Object.keys(KEYS)) c[k] = c[k] || s[k];
		};
		if (g) {
			const b = (i) => !!(g.buttons[i] && (g.buttons[i].pressed || g.buttons[i].value > 0.5));
			const face = FACE[this.layout];
			merge({
				ly: Math.round(-dz(g.axes[1]) * 127), rx: Math.round(dz(g.axes[2]) * 127),
				L1: b(4), R1: b(5), L2: b(6), R2: b(7),
				A: b(face.A), B: b(face.B), X: b(face.X), Y: b(face.Y),
				Up: b(12), Down: b(13), Left: b(14), Right: b(15),
			});
		}
		if (this.external) merge(this.external);
		return c;
	}
}
