// Switch-Pro-style USB controllers (incl. knockoffs) through WebHID. Chrome's own gamepad support
// does a handshake with these that many knockoffs fail, so the pad never shows up; this talks to
// the controller directly instead. Chrome / Edge only. Fills input.hidPad with a V5 controller
// state, the same shape as the V5 bridge and the gamepad reader.
const NINTENDO = 0x057e;

export function hidSupported() { return 'hid' in navigator; }

export async function connectSwitchPad(input, onStatus, { ask = true } = {}) {
	if (!hidSupported()) { onStatus?.('Use Chrome or Edge for this'); return false; }
	let dev = (await navigator.hid.getDevices()).find((d) => d.vendorId === NINTENDO);
	if (!dev && ask) {
		// Gamepads (usage page 1, usage 5) of any brand, plus anything Nintendo.
		const picked = await navigator.hid.requestDevice({ filters: [{ vendorId: NINTENDO }, { usagePage: 1, usage: 5 }] }).catch(() => []);
		dev = picked[0];
	}
	if (!dev) { if (ask) onStatus?.('No controller picked'); return false; }
	if (!dev.opened) await dev.open();
	const st = { cal: null, reports: 0, kind: '' };
	input.hidName = dev.productName || 'USB controller';
	dev.addEventListener('inputreport', (e) => {
		const s = parse(e.reportId, e.data, st);
		if (s) { input.hidPad = s; st.reports++; }
	});
	// Switch Pro over USB: handshake, then "USB only" so it keeps sending, then full input reports.
	const send = (id, bytes) => dev.sendReport(id, new Uint8Array(bytes)).catch(() => {});
	await send(0x80, [0x02]);
	await send(0x80, [0x04]);
	let pkt = 0;
	const sub = (cmd, arg) => send(0x01, [pkt++ & 0xf, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40, cmd, arg]);
	await sub(0x03, 0x30);   // input report mode: standard full (0x30)
	await sub(0x48, 0x01);   // vibration on (harmless if ignored)
	setTimeout(() => onStatus?.(st.reports ? `Controller connected: ${input.hidName}` : 'Connected, but no data yet: press a button'), 600);
	navigator.hid.addEventListener('disconnect', (e) => { if (e.device === dev) { input.hidPad = null; onStatus?.('Controller unplugged'); } });
	return true;
}

// Sticks come in raw; centre comes from the first report and each axis scales to its own range
// as the stick is pushed, so worn or cheap sticks still reach full speed.
function axis(st, i, raw, flip) {
	st.cal ||= {};
	const c = (st.cal[i] ||= { mid: raw, lo: raw - 1200, hi: raw + 1200 });
	c.lo = Math.min(c.lo, raw); c.hi = Math.max(c.hi, raw);
	const v = raw >= c.mid ? (raw - c.mid) / (c.hi - c.mid) : (raw - c.mid) / (c.mid - c.lo);
	return Math.round(Math.max(-1, Math.min(1, v)) * 127 * (flip ? -1 : 1));
}

// -> { ly, rx, L1, L2, R1, R2, A, B, X, Y, Up, Down, Left, Right }. Switch face buttons sit where the
// V5 letters do (A right, B bottom, X top, Y left), so they map straight across.
function parse(id, d, st) {
	if (id === 0x30 || id === 0x21) {   // full report: 3 button bytes, then 12-bit sticks
		if (d.byteLength < 11) return null;
		const r = d.getUint8(2), l = d.getUint8(4);
		const lx = d.getUint8(5) | ((d.getUint8(6) & 0xf) << 8), lyRaw = (d.getUint8(6) >> 4) | (d.getUint8(7) << 4);
		const rx = d.getUint8(8) | ((d.getUint8(9) & 0xf) << 8);
		void lx;
		st.kind = 'full';
		return {
			ly: axis(st, 'ly', lyRaw, false), rx: axis(st, 'rx', rx, false),
			Y: !!(r & 0x01), X: !!(r & 0x02), B: !!(r & 0x04), A: !!(r & 0x08), R1: !!(r & 0x40), R2: !!(r & 0x80),
			Down: !!(l & 0x01), Up: !!(l & 0x02), Right: !!(l & 0x04), Left: !!(l & 0x08), L1: !!(l & 0x40), L2: !!(l & 0x80),
		};
	}
	if (id === 0x3f) {   // simple HID report: 2 button bytes, hat, then 16-bit sticks
		if (d.byteLength < 11) return null;
		const b0 = d.getUint8(0), hat = d.getUint8(2);
		const ly = d.getUint16(5, true), rx = d.getUint16(7, true);
		st.kind = 'simple';
		const h = (...v) => v.includes(hat);
		return {
			ly: axis(st, 'ly', ly, true), rx: axis(st, 'rx', rx, false),
			B: !!(b0 & 0x01), A: !!(b0 & 0x02), Y: !!(b0 & 0x04), X: !!(b0 & 0x08),
			L1: !!(b0 & 0x10), R1: !!(b0 & 0x20), L2: !!(b0 & 0x40), R2: !!(b0 & 0x80),
			Up: h(7, 0, 1), Right: h(1, 2, 3), Down: h(3, 4, 5), Left: h(5, 6, 7),
		};
	}
	return null;
}
