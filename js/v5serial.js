// Reads the bridge program (bridge/main.cpp) over Web Serial and feeds Input.external.
const NAMES = ['L1', 'L2', 'R1', 'R2', 'A', 'B', 'X', 'Y', 'Up', 'Down', 'Left', 'Right'];

export async function connectV5(input, onStatus) {
	if (!('serial' in navigator)) { onStatus('Web Serial needs Chrome or Edge'); return; }
	let port;
	try { port = await navigator.serial.requestPort(); } catch { onStatus('No port picked'); return; }
	await port.open({ baudRate: 115200 });
	onStatus('V5 connected — waiting for data…');
	const reader = port.readable.pipeThrough(new TextDecoderStream()).getReader();
	let buf = '', stale;
	try {
		for (;;) {
			const { value, done } = await reader.read();
			if (done) break;
			buf += value;
			const lines = buf.split('\n');
			buf = lines.pop();
			for (const line of lines) {
				const m = line.match(/V5 (-?\d+) (-?\d+) (-?\d+) (-?\d+) (\d+)/);
				if (!m) continue;
				const bits = +m[5], s = { ly: +m[1], rx: +m[2] };
				NAMES.forEach((n, i) => { s[n] = !!(bits & (1 << i)); });
				input.external = s;
				clearTimeout(stale);
				stale = setTimeout(() => { input.external = null; }, 500);   // bridge stopped
			}
		}
	} catch (e) {
		onStatus('V5 disconnected: ' + e.message);
	} finally {
		input.external = null;
		reader.releaseLock();
		await port.close().catch(() => {});
	}
}
