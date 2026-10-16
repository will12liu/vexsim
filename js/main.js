import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import RAPIER from '@dimforge/rapier3d-compat';
import { Field, TOGGLES, IN, W, toField, clampVelocities } from './field.js';
import { Robot, CFG } from './robot.js';
import { AIRobot } from './ai.js';
import { Input } from './input.js';
import { makeAuton } from './auton.js';
import { connectV5 } from './v5serial.js';
import { connectSwitchPad, hidSupported } from './switchhid.js';

await RAPIER.init();
const $ = (id) => document.getElementById(id);
const DT = 1 / 120, AUTON_S = 15, DRIVER_S = 105;

// ---- Scene ---------------------------------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
$('view').appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x15181d);
scene.fog = new THREE.Fog(0x15181d, 6, 16);
scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.55;
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x30343a, 0.9));
const sun = new THREE.DirectionalLight(0xfff6e8, 2.2);
sun.position.set(1.2, 4.5, 1.6);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
sun.shadow.bias = -0.0004;
sun.shadow.normalBias = 0.01;
Object.assign(sun.shadow.camera, { left: -2.2, right: 2.2, top: 2.2, bottom: -2.2, near: 0.5, far: 8 });
scene.add(sun);
const camera = new THREE.PerspectiveCamera(52, 1, 0.02, 50);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
camera.position.copy(W(-120, -40, 90));
function resize() {
	const r = $('view').getBoundingClientRect();
	renderer.setSize(r.width, r.height);
	camera.aspect = r.width / r.height;
	camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

// ---- World ---------------------------------------------------------------------------
const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
world.timestep = DT;
if (world.integrationParameters) world.integrationParameters.numSolverIterations = 8;
const field = new Field(RAPIER, world, scene);
const robot = new Robot(RAPIER, world, scene, field);
const input = new Input();
// AI robots for 2v2: partner + two opponents. Built once, switched off outside 2v2.
const ais = [0, 1, 2].map(() => new AIRobot(RAPIER, world, scene, field, 'red', ''));
const active = () => (sim.game === 'ai' ? [robot, ...ais] : [robot]);

const sim = { game: 'free', menu: true, mode: 'practice', clock: 0, auton: null, gen: null, autonTest: false, final: null };
const sel = () => ({ color: $('alliance').value, side: $('side').value });

function toast(msg) {
	const t = $('toast');
	t.textContent = msg; t.classList.add('show');
	clearTimeout(toast.h); toast.h = setTimeout(() => t.classList.remove('show'), 1200);
}
robot.flash = (msg) => { toast(msg); input.rumble(msg === 'normal' ? 80 : 300); };
function log(msg) {
	const el = $('log');
	el.textContent = (msg + '\n' + el.textContent).slice(0, 2000);
}

// Robot centred on the chosen Toggle, front facing the Toggle wall, a few inches off it.
function placeAt(r, quad) {
	const t = TOGGLES.find((x) => x.quad === quad);
	const d = 72 - 3 - CFG.length / 2;
	r.place(-t.n[0] * d, -t.n[1] * d, (Math.atan2(-t.n[1], -t.n[0]) * 180) / Math.PI);
	r.loadPreload();
}
function placeAtStart() {
	placeAt(robot, $('start').value);
	robot.alliance = $('alliance').value;
	// The other three starts go to the AI robots (2v2 only).
	const others = ['red1', 'red2', 'blue1', 'blue2'].filter((q) => q !== $('start').value);
	const on = sim.game === 'ai';
	ais.forEach((a, i) => {
		const q = others[i];
		a.alliance = q.startsWith('red') ? 'red' : 'blue';
		a.bumpMat.color.setHex(a.alliance === 'red' ? 0xd62828 : 0x1e6fd9);
		a.label = (a.alliance === robot.alliance ? 'AI partner' : 'AI opponent') + ' · ' + q.replace('red', 'R').replace('blue', 'B');
		a.skill = +$('mSkill').value;
		a.body.setEnabled(on); a.mesh.visible = on;
		if (on) placeAt(a, q);
		else a.place(200 + i * 40, 200, 0);   // parked far away
	});
}

function resetField() {
	stopAuton();
	for (const r of [robot, ...ais]) r.hopper = r.claw = null;
	field.reset();
	placeAtStart();
	sim.final = null;
}

function startAuton(test) {
	sim.auton = makeAuton(robot, sel(), log);
	try { sim.gen = sim.auton.compile($('routine').value); }
	catch (e) { log('Routine error: ' + e.message); sim.auton = null; return false; }
	sim.autonTest = test;
	log(`--- auton start (${sel().color} ${sel().side}) ---`);
	return true;
}
function stopAuton() {
	if (!sim.auton) return;
	sim.auton.stop(); sim.auton.finish();
	if (sim.autonTest) robot.afterAutonTest();
	sim.auton = sim.gen = null;
	log('--- auton end ---');
}

function startMatch() {
	if (sim.game === 'free') sim.game = 'solo';
	resetField();
	sim.mode = 'auton'; sim.clock = AUTON_S;
	startAuton(false);
}
function practice() { stopAuton(); sim.mode = 'practice'; sim.final = null; }

// ---- Title menu ----------------------------------------------------------------------
function startGame(game) {
	sim.game = game; sim.menu = false; $('menu').hidden = true; document.body.classList.remove('menu-open');
	$('start').value = $('mStart').value;
	$('alliance').value = $('start').value.startsWith('red') ? 'red' : 'blue';
	if (game === 'free') { resetField(); practice(); } else startMatch();
	toast({ free: 'FREE DRIVE', solo: 'SOLO MATCH', ai: '2v2 vs AI' }[game]);
}
function showMenu() { stopAuton(); sim.menu = true; $('menu').hidden = false; document.body.classList.add('menu-open'); $('mStart').value = $('start').value; menuSel = -1; markMenu(); }
const modeBtns = [...document.querySelectorAll('.mode')];
let menuSel = -1;
const markMenu = () => modeBtns.forEach((b, i) => b.classList.toggle('sel', i === menuSel));
modeBtns.forEach((b) => (b.onclick = () => startGame(b.dataset.game)));
$('mControls').onclick = () => showControls(true);
$('btnMenu').onclick = () => { $('btnMenu').blur(); showMenu(); };
addEventListener('keydown', (e) => {
	if (!sim.menu || e.target.closest?.('select')) return;
	const i = ['Digit1', 'Digit2', 'Digit3'].indexOf(e.code);
	if (i >= 0) startGame(modeBtns[i].dataset.game);
	if (e.code === 'Enter' && menuSel >= 0) startGame(modeBtns[menuSel].dataset.game);
});
// Controller on the menu: D-pad / stick moves the highlight, A (either layout) picks.
function menuPad(c) {
	const p = sim.menuPrev || {};
	const left = c.Left || c.Up || c.rx < -60, right = c.Right || c.Down || c.rx > 60;
	if (left && !p.left) { menuSel = (menuSel + 2) % 3; markMenu(); }
	if (right && !p.right) { menuSel = (menuSel + 1) % 3; markMenu(); }
	const pick = c.A || c.B;
	if (pick && !p.pick && menuSel >= 0 && $('controls').hidden) startGame(modeBtns[menuSel].dataset.game);
	sim.menuPrev = { left, right, pick };
}

// ---- UI ------------------------------------------------------------------------------
$('btnMatch').onclick = () => { startMatch(); $('btnMatch').blur(); };
$('btnPractice').onclick = () => { if (sim.game !== 'free') { sim.game = 'free'; resetField(); } practice(); $('btnPractice').blur(); };
$('btnReset').onclick = () => { resetField(); $('btnReset').blur(); };
$('btnAuton').onclick = () => { if (sim.mode === 'practice') { stopAuton(); startAuton(true); } $('btnAuton').blur(); };
$('btnStop').onclick = () => stopAuton();
$('btnV5').onclick = () => { $('btnV5').blur(); connectV5(input, toast); };
// Controllers Chrome doesn't pick up by itself (e.g. Switch-Pro-style knockoffs): talk to them directly.
for (const id of ['btnPad', 'mPad']) $(id).onclick = () => { $(id).blur(); connectSwitchPad(input, toast); };
if (hidSupported()) connectSwitchPad(input, toast, { ask: false }).catch(() => {});   // reconnect one picked before
else for (const id of ['btnPad', 'mPad']) $(id).hidden = true;
$('layout').onchange = (e) => { input.layout = e.target.value; };
$('alliance').onchange = () => resetField();
$('start').onchange = () => { $('alliance').value = $('start').value.startsWith('red') ? 'red' : 'blue'; resetField(); };
const CAMS = ['driver', 'chase', 'orbit', 'top'];
let cam = 'driver';
const look = new THREE.Vector3();
function setCam(c) {
	cam = c; controls.enabled = cam === 'orbit';
	$('btnCam').textContent = 'View: ' + cam;
	if (cam === 'orbit') { controls.target.set(0, 0, 0); camera.position.copy(W(-120, -40, 90)); }
}
// ---- Controls page -------------------------------------------------------------------
const XBOX = { position: { A: 'B', B: 'A', X: 'Y', Y: 'X' }, letter: { A: 'A', B: 'B', X: 'X', Y: 'Y' } };
function renderControls() {
	const x = XBOX[input.layout], k = (...a) => a.map((c) => `<kbd>${c}</kbd>`).join(' ');
	const rows = [
		['Drive forward / back', 'Left stick ↕', 'Left stick ↕', k('W', 'S'), 'Sticks are proportional: push a little = go slow. Keys ramp up: tap = slow, hold = full. Shift = half speed'],
		['Turn', 'Right stick ↔', 'Right stick ↔', k('A', 'D')],
		['Lift up (cascade)', k('L1'), k('LB'), k('R')],
		['Lift down', k('L2'), k('LT'), k('G')],
		['Intake cycle', k('R1'), k('RB'), k('E'), '1st: intake + claw grabs + cascade to intake height · 2nd: claw spin only · 3rd: off'],
		['Reverse intake + claw spin', k('R2'), k('RT'), k('Q'), 'Toggle: spits out of the intake and lets go of the claw'],
		['Claw flip', k('B'), k(x.B), k('B'), '1st: slanted · 2nd: 90° (pin hangs over the goal) · 3rd: down'],
		['Claw spin in / drop pin', k('A'), k(x.A), k('F'), 'Toggle. Spinning this way lets go of the pin'],
		['Claw spin out', k('Y'), k(x.Y), k('Y'), 'Toggle'],
		['X (not used yet)', k('X'), k(x.X), k('X')],
		['Invert drive (claw side = front)', k('↑'), 'D-pad ↑', k('↑'), 'Stick up drives claw first; turning unchanged. Shows INVERTED on the scoreboard'],
		['Run auton (practice only)', k('↓'), 'D-pad ↓', k('↓')],
		['E-stop a running auton test', k('L1') + '+' + k('R1'), k('LB') + '+' + k('RB'), k('R') + '+' + k('E')],
	];
	$('ctrlRows').innerHTML = rows.map(([a, v, xb, kb, note]) => `<tr><td>${a}${note ? `<small>${note}</small>` : ''}</td><td>${v}</td><td>${xb}</td><td>${kb}</td></tr>`).join('');
}
function showControls(on) { renderControls(); $('controls').hidden = !on; }
$('btnHelp').onclick = () => { $('btnHelp').blur(); showControls(true); };
$('btnHelpClose').onclick = () => { input.cancelSetup(); showControls(false); };
$('btnPadSetup').onclick = () => { $('btnPadSetup').blur(); input.startSetup((m) => { $('padSetupMsg').textContent = m; }, () => toast('CONTROLLER SET')); };
$('btnPadReset').onclick = () => { $('btnPadReset').blur(); input.clearMap(); $('padSetupMsg').textContent = 'Back to the default layout.'; };
$('controls').onclick = (e) => { if (e.target.id === 'controls') showControls(false); };
addEventListener('keydown', (e) => {
	if (e.target.closest?.('textarea, input, select')) return;
	if (e.key === '?' || e.code === 'F1') { e.preventDefault(); showControls($('controls').hidden); }
	if (e.code === 'Escape' && input.setup) { input.cancelSetup(); return; }
	if (e.code === 'Escape') { if (!$('controls').hidden) showControls(false); else if (sim.menu && sim.game) { sim.menu = false; $('menu').hidden = true; document.body.classList.remove('menu-open'); } else showMenu(); }
});
setCam('driver');
$('btnCam').onclick = () => { setCam(CAMS[(CAMS.indexOf(cam) + 1) % CAMS.length]); $('btnCam').blur(); };
addEventListener('keydown', (e) => {
	if (e.target.closest?.('textarea, input, select')) return;
	if (sim.menu) return;
	if (e.code === 'KeyV') setCam(CAMS[(CAMS.indexOf(cam) + 1) % CAMS.length]);
	if (e.code === 'KeyT') resetField();
	if (e.code === 'KeyP') $('btnPractice').onclick();
	if (e.code === 'KeyM') startMatch();
});

function updateCamera() {
	const p = robot.pos();
	if (sim.menu) {   // slow fly-around behind the title
		const a = performance.now() / 9000;
		camera.position.set(Math.cos(a) * 3.1, 1.7, Math.sin(a) * 3.1);
		camera.lookAt(0, 0, 0);
		return;
	}
	if (cam === 'chase') {
		const back = new THREE.Vector3(-1, 0, 0).applyQuaternion(robot.quat());
		const want = p.clone().addScaledVector(back, 0.9).add(new THREE.Vector3(0, 0.6, 0));
		camera.position.lerp(want, 0.12);
		camera.lookAt(p.x, p.y + 0.2, p.z);
	} else if (cam === 'driver') {
		// Standing in the alliance station: ~30" behind the wall, eyes at 64", head follows the robot.
		const red = $('alliance').value === 'red', s = red ? 1 : -1;
		const spot = { left: 30, centre: 0, right: -30 }[$('spot').value] * s;
		camera.position.copy(W(red ? -102 : 102, spot, 64));
		const want = p.clone().lerp(W(0, 0, 0), 0.25);
		look.lerp(want, look.lengthSq() ? 0.08 : 1);
		camera.lookAt(look);
	} else if (cam === 'top') {
		camera.position.set(0, 4.6, 0.001);
		camera.lookAt(0, 0, 0);
	} else controls.update();
}

const fmtClock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
const dot = (c) => `<span class="dot ${c}"></span>`;
// The scoreboard only changes once a new score has held for half a second, so a pin wobbling on
// the edge of a goal (or a piece mid-bounce) doesn't make the numbers flicker.
let shown = null, pending = null, pendingAt = 0;
function steadyScore(sc) {
	const now = performance.now(), same = (a, b) => a && a.red === b.red && a.blue === b.blue;
	if (!shown || sim.menu) { shown = sc; return sc; }
	if (same(shown, sc)) { pending = null; shown = sc; return sc; }
	if (!same(pending, sc)) { pending = sc; pendingAt = now; }
	if (now - pendingAt > 500) { shown = sc; pending = null; }
	return { ...sc, red: shown.red, blue: shown.blue };
}
function updateHud(score) {
	const label = { practice: 'PRACTICE', auton: 'AUTONOMOUS', driver: 'DRIVER', ended: 'MATCH OVER' }[sim.mode];
	$('mode').textContent = label + (sim.auton && sim.mode === 'practice' ? ' · AUTON TEST' : '') + (robot.op.inverse < 0 ? ' · INVERTED' : '');
	$('clock').textContent = sim.mode === 'practice' ? '--:--' : fmtClock(Math.max(0, sim.clock));
	const s = sim.final || steadyScore(score);
	$('red').textContent = s.red; $('blue').textContent = s.blue;
	// Who owns each quarter's yellow pins (same colours as the light bars on the wall and the goal rings).
	$('toggles').innerHTML = ['red1', 'red2', 'blue1', 'blue2'].map((q) => {
		const own = s.toggles[q], name = q.replace(/(\d)/, ' $1').toUpperCase();
		return `<span class="tg own-${own}" title="${name} Toggle: yellow pins in this quarter score for ${own === 'yellow' ? 'nobody' : own}">${name}<b>${own === 'yellow' ? '—' : own.toUpperCase()}</b></span>`;
	}).join('');
	$('detail').textContent = `Pins placed ${s.placedPins} · halves R${s.halves.red} B${s.halves.blue} Y→R${s.halves.yellowRed} Y→B${s.halves.yellowBlue} `;
	const clawState = robot.clawDeg < 450 ? 'down' : robot.clawDeg < 1120 ? '90°' : 'slanted';
	$('robot').textContent = `Lift ${robot.liftIn().toFixed(1)}" · Claw ${clawState} · Claw ${robot.claw ? (robot.claw.kind === 'cup' ? 'cup' : robot.claw.top + '/' + robot.claw.bottom + ' pin') : '—'} · Intake ${robot.hopper ? (robot.hopper.kind === 'cup' ? 'cup' : 'pin') : '—'} · ` +
		`Drive ${robot.op.inverse > 0 ? 'normal' : 'INVERTED'} · IMU ${robot.imuDeg.toFixed(1)}° · Midfield ${robot.inMidfield() ? 'YES' : 'no'}`;
	$('source').textContent = input.source;
}

// ---- Loop ----------------------------------------------------------------------------
resetField();
let last = performance.now(), acc = 0, frame = 0, score = field.score({ red: 0, blue: 0 });
function control(input) {
	const ctrl = { ...input };   // don't flip the caller's copy (a reused state would flip back and forth)
	if ($('invY').checked) ctrl.ly = -ctrl.ly;
	if ($('invX').checked) ctrl.rx = -ctrl.rx;
	const driverMode = sim.mode === 'practice' || sim.mode === 'driver';
	if (sim.auton && sim.autonTest && ctrl.L1 && ctrl.R1) { log('E-STOP'); stopAuton(); }
	if (driverMode && !sim.auton) robot.opcontrol(ctrl, sim.mode === 'practice' ? () => startAuton(true) : null);
	else robot.op.prev = { ...ctrl };
}
function physicsStep() {
	if (sim.gen) { const r = sim.gen.next(); if (r.done) stopAuton(); }
	if (sim.mode === 'auton' || sim.mode === 'driver') {
		sim.clock -= DT;
		if (sim.clock <= 0 && sim.mode === 'auton') {
			stopAuton();
			score = field.score(mid());
			log(`Auton score: red ${score.red} / blue ${score.blue}`);
			sim.mode = 'driver'; sim.clock = DRIVER_S;
		} else if (sim.clock <= 0 && sim.mode === 'driver') {
			sim.mode = 'ended'; sim.endT = robot.t;
			robot.leftCmd = robot.rightCmd = robot.intakeCmd = robot.clawSpin = robot.liftCmd = 0;
			for (const a of ais) a.stop();
		}
	}
	// Scores are taken once things settle (<SC1>), here 3 s after the buzzer.
	if (sim.mode === 'ended' && !sim.final && robot.t - sim.endT > 3) { sim.final = field.score(mid()); log(`FINAL red ${sim.final.red} / blue ${sim.final.blue}`); }
	const live = sim.mode === 'auton' || sim.mode === 'driver';
	for (const a of ais) if (sim.game === 'ai') { if (live) a.think(DT, active()); else a.stop(); }
	for (const r of active()) r.step(DT);
	field.stepToggles(DT, active());
	world.step();
	clampVelocities(field.pins); clampVelocities(field.cups);
	for (const r of active()) { r.clampVel(); r.holdAtToggle(); }
}
function tick(now) {
	const frameDt = Math.min(0.1, (now - last) / 1000);
	last = now; acc += frameDt;
	const c = input.poll();
	if (sim.menu) { menuPad(c); acc = 0; }
	else { control(c); while (acc >= DT) { acc -= DT; physicsStep(); } }
	field.syncMeshes();
	for (const r of active()) r.syncMesh(frameDt);
	updateTags();
	updateCamera();
	// A bug in the scoreboard must never stop the field from being drawn.
	if (frame++ % 6 === 0) { try { score = field.score(mid()); updateHud(score); } catch (e) { if (!tick.warned) { console.error(e); tick.warned = true; } } }
	renderer.render(scene, camera);
	requestAnimationFrame(tick);
}
// Headless testing: run `seconds` of sim time right now with a fixed controller state.
function advance(seconds, ctrl = {}) {
	const c = { ly: 0, rx: 0, ...ctrl };
	for (let i = 0; i < Math.round(seconds / DT); i++) { if (i % 2 === 0) control(c); physicsStep(); }
	field.syncMeshes(); for (const r of active()) r.syncMesh(0);
	return field.score(mid());
}
function mid() { const m = { red: 0, blue: 0 }; for (const r of active()) if (r.inMidfield()) m[r.alliance]++; return m; }
// Name tags over the AI robots.
const tagEls = ais.map(() => { const d = document.createElement('div'); d.className = 'tag'; $('tags').appendChild(d); return d; });
function updateTags() {
	ais.forEach((a, i) => {
		const el = tagEls[i], v = a.pos().add(new THREE.Vector3(0, 24 * IN, 0)).project(camera);
		const show = sim.game === 'ai' && !sim.menu && v.z < 1;
		el.style.display = show ? 'block' : 'none';
		if (!show) return;
		const r = $('view').getBoundingClientRect();
		el.className = 'tag ' + a.alliance; el.textContent = a.label;
		el.style.left = ((v.x + 1) / 2) * r.width + 'px'; el.style.top = ((1 - v.y) / 2) * r.height + 'px';
	});
}
requestAnimationFrame(tick);
$('loading').remove();
// Auton routine: routine.cpp is copied from the robot code (tools/sync-robot-code.sh). Load its
// run_routine() into the editor unless it's already been edited here.
const routineDefault = $('routine').value;
fetch('routine.cpp', { cache: 'no-cache' }).then((r) => (r.ok ? r.text() : null)).then((src) => {
	const i = src?.indexOf('void run_routine()') ?? -1;
	if (i < 0 || $('routine').value !== routineDefault) return;
	let d = 0, k = src.indexOf('{', i);
	for (; k < src.length; k++) { if (src[k] === '{') d++; else if (src[k] === '}' && --d === 0) break; }
	$('routine').value = src.slice(i, k + 1) + '\n';
}).catch(() => {});
window.sim = { robot, ais, startGame, showMenu, input, field, world, sim, toField, IN, advance, resetField, startAuton, $, camera, controls, setCam };   // for debugging in the console
