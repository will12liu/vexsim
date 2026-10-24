// Offline cache for the sim (the installable app and the website both use it).
// Online: always fetch fresh files (so updates show up) and keep a copy.
// Offline / slow network: serve the copy. Every file is cached on the first visit.
const CACHE = 'titanium-sim-v1';
const FILES = [
	'./', 'index.html', 'routine.cpp', 'manifest.webmanifest', 'TitaniumSim.html',
	'icons/icon-192.png', 'icons/icon-512.png', 'icons/icon-maskable-512.png', 'icons/apple-touch-icon.png',
	'vendor/three/three.module.min.js', 'vendor/three/addons/controls/OrbitControls.js',
	'vendor/three/addons/environments/RoomEnvironment.js', 'vendor/rapier/rapier.es.js',
	'js/ai.js', 'js/auton.js', 'js/cpp.js', 'js/drivetrain.js', 'js/field.js', 'js/human.js',
	'js/input.js', 'js/main.js', 'js/robot.js', 'js/robotmodel.js', 'js/switchhid.js', 'js/v5serial.js',
];
const TIMEOUT_MS = 3500;   // flaky plane / venue wifi: don't wait on it, use the copy
// Hosts like Cloudflare Pages redirect index.html -> / ; a redirected response can't answer a page load, so store a plain copy.
const plain = async (r) => (r.redirected ? new Response(await r.blob(), { status: r.status, statusText: r.statusText, headers: r.headers }) : r);

self.addEventListener('install', (e) => {
	e.waitUntil(caches.open(CACHE).then((c) => Promise.all(FILES.map((f) =>
		fetch(f, { cache: 'reload' }).then(async (r) => (r.ok ? c.put(f, await plain(r)) : null)).catch(() => null)))).then(() => self.skipWaiting()));
});

self.addEventListener('activate', (e) => {
	e.waitUntil(caches.keys().then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)))).then(() => self.clients.claim()));
});

self.addEventListener('fetch', (e) => {
	const req = e.request;
	if (req.method !== 'GET' || new URL(req.url).origin !== location.origin) return;
	e.respondWith((async () => {
		const cache = await caches.open(CACHE);
		const net = fetch(req).then(async (r) => { if (r.ok) cache.put(req, await plain(r.clone())); return r; });
		const timeout = new Promise((res) => setTimeout(res, TIMEOUT_MS));
		const first = await Promise.race([net.catch(() => null), timeout]);
		if (first) return first;
		const copy = await cache.match(req, { ignoreSearch: true });
		if (copy) { net.catch(() => {}); return copy; }
		return net;   // nothing cached: keep waiting on the network
	})());
});
