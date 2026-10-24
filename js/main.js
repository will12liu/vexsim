import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import RAPIER from '@dimforge/rapier3d-compat';
import { Field, TOGGLES, TOGGLE, IN, W, WALL, toField, clampVelocities } from './field.js';
import { Robot, CFG } from './robot.js';
import { AIRobot, STRATEGIES } from './ai.js';
import { Input } from './input.js';
import { HumanPlayers } from './human.js';
import { makeAuton } from './auton.js';
import { connectV5 } from './v5serial.js';
import { connectSwitchPad, hidSupported } from './switchhid.js';

await RAPIER.init();
const $ = (id) => document.getElementById(id);
const DT = 1 / 120, AUTON_S = 15, DRIVER_S = 105;
const LOOP_S = 0.02;      // opcontrol.cpp runs every 20 ms
const LATENCY_S = 0.025;  // controller -> VEXnet radio -> Brain

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
scene.fog = new THREE.Fog(0x15181d, 7, 18);
scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.6;
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x30343a, 0.85));
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
const humans = new HumanPlayers(scene, field, (m) => toast(m));
const humanOpts = () => { humans.load = $('mHuman').value; humans.loader = $('mLoader').value; humans.unlimited = $('mLoads').value === 'unlimited'; };
for (const id of ['mHuman', 'mLoader', 'mLoads']) $(id).onchange = humanOpts;
// AI robots for 2v2: partner + two opponents. Built once, switched off outside 2v2.
const ais = [0, 1, 2].map(() => new AIRobot(RAPIER, world, scene, field, 'red', ''));
const active = () => (sim.game === 'ai' ? [robot, ...ais] : [robot]);

const sim = { game: 'free', menu: true, mode: 'practice', clock: 0, auton: null, autonKind: null, autonT: 0, final: null, loopAcc: 0, hist: [] };
const sel = () => ({ color: $('alliance').value, side: $('side').value });

function toast(msg, sub) {
	const t = $('toast');
	t.innerHTML = msg + (sub ? `<small>${sub}</small>` : ''); t.classList.add('show');
	clearTimeout(toast.h); toast.h = setTimeout(() => t.classList.remove('show'), sub ? 2200 : 1200);
}
robot.flash = (msg, sub) => { toast(msg, sub); input.rumble(msg === 'CLAW FIRST' ? 300 : msg === 'TIPPED OVER' ? 700 : 90); };
function log(msg) {
	const el = $('log');
	el.textContent = (msg + '\n' + el.textContent).slice(0, 3000);
}

// Robot centred on the chosen Toggle, front facing it, 0.7" off it (routine.cpp: WALL_Y = 0.7).
function placeAt(r, quad) {
	const t = TOGGLES.find((x) => x.quad === quad);
	const face = WALL - TOGGLE.inset - TOGGLE.face / (2 * Math.sqrt(3));
	const d = face - 0.7 - CFG.length / 2;
	r.place(-t.n[0] * d, -t.n[1] * d, (Math.atan2(-t.n[1], -t.n[0]) * 180) / Math.PI);
	r.loadPreload();
}
function placeAtStart() {
	robot.alliance = $('alliance').value;
	placeAt(robot, $('start').value);
	// The other three starts go to the AI robots (2v2 only).
	const others = ['red1', 'red2', 'blue1', 'blue2'].filter((q) => q !== $('start').value);
	const on = sim.game === 'ai';
	let opp = 0;
	ais.forEach((a, i) => {
		const q = others[i];
		a.alliance = q.startsWith('red') ? 'red' : 'blue';
		const partner = a.alliance === robot.alliance;
		let st = $(partner ? 'mStratP' : opp++ ? 'mStratO2' : 'mStratO1').value;
		if (st === 'auto') { const keys = Object.keys(STRATEGIES); st = keys[Math.floor(Math.random() * keys.length)]; }
		a.setStrategy(st);
		a.label = (partner ? 'AI partner' : 'AI opponent') + ' · ' + STRATEGIES[a.strategy].name + ' · ' + q.replace('red', 'R').replace('blue', 'B');
		a.skill = +$('mSkill').value;
		a.body.setEnabled(on); a.mesh.visible = on;
		if (on) placeAt(a, q);
		else a.place(200 + i * 40, 200, 0);   // parked far away
		a.paint?.();
	});
	humans.opponents = on;
}

function resetField() {
	stopAuton();
	for (const r of [robot, ...ais]) r.hopper = r.claw = r.clawExtraPiece = null;
	field.reset();
	placeAtStart();
	sim.final = null; sim.hist = [];
}

// kind: 'match' (the 15 s autonomous period), 'practice' (D-pad Down: 15 s cutoff), 'macro' (B + Y).
function startAuton(kind) {
	const a = makeAuton(robot, sel(), log);
	try { a.compile($('routine').value); }
	catch (e) { log('Routine error: ' + e.message); return false; }
	sim.auton = a; sim.autonKind = kind; sim.autonT = 0;
	robot.mechStopAll();
	input.rumble(80);
	log(`--- auton ${kind} (${sel().color} ${sel().side}${sel().color === 'blue' ? ', blue swaps the path' : ''}) ---`);
	return true;
}
function stopAuton(reason) {
	if (!sim.auton) return;
	const a = sim.auton;
	a.stop(); a.finish();
	if (sim.autonKind !== 'match') robot.afterAutonTest();
	log(`--- auton ${reason || 'end'}: last step "${a.lastStep}" at ${(a.lastStepMs / 1000).toFixed(1)}s ---`);
	sim.auton = null; sim.autonKind = null;
}

function startMatch() {
	if (sim.game === 'free') sim.game = 'solo';
	resetField();
	sim.mode = 'auton'; sim.clock = AUTON_S;
	startAuton('match');
}
function practice() { stopAuton(); sim.mode = 'practice'; sim.final = null; }

// ---- Title menu ----------------------------------------------------------------------
function startGame(game) {
	humanOpts();
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
$('btnAuton').onclick = () => { if (sim.mode === 'practice') { stopAuton(); startAuton('practice'); } $('btnAuton').blur(); };
$('btnStop').onclick = () => stopAuton('stopped');
$('btnV5').onclick = () => { $('btnV5').blur(); connectV5(input, toast); };
// Controllers Chrome doesn't pick up by itself (e.g. Switch-Pro-style knockoffs): talk to them directly.
for (const id of ['btnPad', 'mPad']) $(id).onclick = () => { $(id).blur(); connectSwitchPad(input, toast); };
if (hidSupported()) connectSwitchPad(input, toast, { ask: false }).catch(() => {});   // reconnect one picked before
else for (const id of ['btnPad', 'mPad']) $(id).hidden = true;
$('layout').onchange = (e) => { input.layout = e.target.value; };
$('alliance').onchange = () => resetField();
$('start').onchange = () => { $('alliance').value = $('start').value.startsWith('red') ? 'red' : 'blue'; resetField(); };
const setSide = (side) => { $('side').value = side; markSide(); toast(`AUTON: ${sel().color.toUpperCase()} ${side.toUpperCase()}`, sideNote()); };
const sideNote = () => (sel().color === 'blue' ? `Blue swaps the path (routine.cpp): runs the ${sel().side === 'right' ? 'LEFT' : 'RIGHT'} path` : `Runs the ${sel().side.toUpperCase()} path`);
const markSide = () => { $('sideNote').textContent = sideNote(); };
$('side').onchange = markSide;
const CAMS = ['driver', 'standing', 'chase', 'orbit', 'top'];
let cam = 'driver';
const look = new THREE.Vector3();
function setCam(c) {
	cam = c; controls.enabled = cam === 'orbit';
	$('btnCam').textContent = 'View: ' + { driver: 'driver (field photo)', standing: 'driver standing', chase: 'chase', orbit: 'orbit', top: 'top' }[cam];
	camera.fov = cam === "driver" ? 46 : cam === 'standing' ? 50 : 52; camera.updateProjectionMatrix();
	if (cam === 'orbit') { controls.target.set(0, 0, 0); camera.position.copy(W(-120, -40, 90)); }
	look.set(0, 0, 0);
}
// ---- Controls page -------------------------------------------------------------------
const XBOX = { position: { A: 'B', B: 'A', X: 'Y', Y: 'X' }, letter: { A: 'A', B: 'B', X: 'X', Y: 'Y' } };
function renderControls() {
	const x = XBOX[input.layout], k = (...a) => a.map((c) => `<kbd>${c}</kbd>`).join(' ');
	const rows = [
		['Drive forward / back', 'Left stick ↕', 'Left stick ↕', k('W', 'S'), 'The drive is the real thing: V5 motors, gear slop, omni wheels that slide sideways, coast after you let go. Feather the sticks. With the cascade up it rocks and can tip if you stop hard. Keys ramp: tap = slow, hold = full, Shift = half'],
		['Turn', 'Right stick ↔', 'Right stick ↔', k('A', 'D')],
		['Lift up (cascade)', k('L1'), k('LB'), k('R')],
		['Lift down', k('L2'), k('LT'), k('G')],
		['Claw grab spin + intake height', k('R1'), k('RB'), k('E'), 'Toggle; also sends the cascade to -70 (2" up), like opcontrol.cpp. Drive claw first into a standing stack SLOWLY (under ~6 in/s): the rollers pull it in and the spin stops by itself. Faster and you knock it over. Something stuck in the guide (a knocked-over pin) blocks the claw'],
		['Claw release spin', k('R2') + ' / ' + k('A'), k('RT') + ' / ' + k(x.A), k('Q') + ' / ' + k('F'), 'Toggle. Pushes the stack out the back (onto a goal if it\'s right over one). Lowering the stack onto a goal also lets go'],
		['Take a match load', k('R1'), k('RB'), k('E'), 'Cascade at R1 height (it must fit under the tube), back in square to the wall, slow. Your human player drops loads in the top (10 Cups + 10 Pins per alliance, driver period only)'],
		['Flip drive direction', k('↑'), 'D-pad ↑', k('↑'), 'Starts claw first (stick up drives claw first), like opcontrol.cpp; turning unchanged'],
		['Auton side', k('←') + ' ' + k('→'), 'D-pad ← →', k('←') + ' ' + k('→'), 'Like the Brain\'s RED L / RED R / BLUE L / BLUE R buttons. Blue swaps the path in routine.cpp'],
		['Practice auton (15 s cutoff)', k('↓'), 'D-pad ↓', k('↓'), 'Free drive only, like autonomous_match_sim()'],
		['Run the routine as a macro', 'Hold ' + k('B') + ', press ' + k('Y'), 'Hold ' + k(x.B) + ', press ' + k(x.Y), 'Hold ' + k('B') + ', press ' + k('Y')],
		['E-stop a practice run / macro', k('L1') + '+' + k('R1'), k('LB') + '+' + k('RB'), k('R') + '+' + k('E'), k('X') + '+' + k('A') + ' blocks the lift and claw buttons while held'],
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

// Driver views. 'driver' matches the team's photo from behind the alliance-station wall (camera
// fitted to it: 26" back from the wall, eyes 44" up, looking 32 degrees down, ~44 degree vertical
// field of view); the head turns a little toward the robot. 'standing' = eyes at 64".
const EYE = { back: 26, h: 44, pitch: 30 };
function updateCamera() {
	const p = robot.pos();
	if (sim.menu) {   // slow fly-around behind the title
		const a = performance.now() / 9000;
		camera.position.set(Math.cos(a) * 3.1, 1.7, Math.sin(a) * 3.1);
		camera.lookAt(0, 0, 0);
		return;
	}
	const red = $('alliance').value === 'red', s = red ? 1 : -1;
	const spot = { left: 20, centre: 0, right: -20 }[$('spot').value] * s;
	if (cam === 'chase') {
		const back = new THREE.Vector3(-1, 0, 0).applyQuaternion(robot.quat());
		const want = p.clone().addScaledVector(back, 0.9).add(new THREE.Vector3(0, 0.6, 0));
		camera.position.lerp(want, 0.12);
		camera.lookAt(p.x, p.y + 0.2, p.z);
	} else if (cam === 'driver') {
		const back = WALL + 2 + EYE.back;
		camera.position.copy(W(-s * back, spot, EYE.h));
		const base = W(-s * (back - EYE.h / Math.tan((EYE.pitch * Math.PI) / 180)), spot * 0.7, 0);
		const want = base.clone().lerp(p, 0.14);
		look.lerp(want, look.lengthSq() ? 0.06 : 1);
		camera.lookAt(look);
	} else if (cam === 'standing') {
		camera.position.copy(W(-s * (WALL + 2 + 28), spot, 64));
		const want = p.clone().lerp(W(0, 0, 0), 0.25);
		look.lerp(want, look.lengthSq() ? 0.08 : 1);
		camera.lookAt(look);
	} else if (cam === 'top') {
		camera.position.set(0, 4.6, 0.001);
		camera.lookAt(0, 0, 0);
	} else controls.update();
}

const fmtClock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
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
	const run = sim.auton && sim.autonKind !== 'match' ? (sim.autonKind === 'macro' ? ' · MACRO' : ' · AUTON TEST') : '';
	$('mode').textContent = label + run + (robot.op.inverse > 0 ? ' · CLAW BACK' : '');
	const pc = sim.autonKind === 'practice' ? AUTON_S - sim.autonT : null;
	$('clock').textContent = pc != null ? fmtClock(Math.max(0, pc)) : sim.mode === 'practice' ? '--:--' : fmtClock(Math.max(0, sim.clock));
	const s = sim.final || steadyScore(score);
	$('red').textContent = s.red; $('blue').textContent = s.blue;
	for (const a of ['red', 'blue']) {
		const L = field.loads[a];
		$(a + 'Loads').textContent = humans.unlimited ? 'LOADS ∞' : `${L.cups} CUPS · ${L.pins} PINS${L.yellow ? ' +Y' : ''}`;
		$(a + 'Loads').title = 'Match loads left for this alliance (10 Cups, 10 alliance/yellow Pins, 1 yellow/yellow)';
	}
	// Who owns each quarter's yellow pins (same colours as the light bars on the wall and the goal rings).
	$('toggles').innerHTML = ['red1', 'red2', 'blue1', 'blue2'].map((q) => {
		const own = s.toggles[q], name = q.replace(/(\d)/, ' $1').toUpperCase();
		return `<span class="tg own-${own}" title="${name} Toggle: yellow pins in this quarter score for ${own === 'yellow' ? 'nobody' : own}">${name}<b>${own === 'yellow' ? '—' : own.toUpperCase()}</b></span>`;
	}).join('');
	$('detail').textContent = `Pins placed ${s.placedPins} · halves R${s.halves.red} B${s.halves.blue} Y→R${s.halves.yellowRed} Y→B${s.halves.yellowBlue} `;
	const spin = robot.clawMode === 'GRAB' ? 'grab' : robot.clawMode === 'RELEASE' ? 'release' : 'off';
	const held = robot.claw ? (robot.claw.kind === 'cup' ? 'cup' : robot.claw.top + '/' + robot.claw.bottom + ' pin') + (robot.clawExtraPiece ? ' + ' + (robot.clawExtraPiece.kind === 'cup' ? 'cup' : robot.clawExtraPiece.top + ' pin') : '') : '—';
	const temp = robot.drive.maxTemp(), hot = temp >= 55 ? ` (HOT: ${temp >= 60 ? 'quarter' : 'half'} power)` : '';
	const tip = robot.tipped() ? 'TIPPED · ' : Math.abs(robot.drive.pitch.th) > 0.03 ? 'WHEELIE · ' : '';
	$('robot').textContent = `${tip}Lift ${robot.liftIn().toFixed(1)}" (${robot.liftPos().toFixed(0)}) · Claw ${held} (spin ${spin}) · ` +
		`${humans.status(robot) ? humans.status(robot) + ' · ' : ''}Drive ${robot.op.inverse < 0 ? 'claw first' : 'claw back'} ${robot.brake} · motors ${temp.toFixed(0)}°C${hot} · Midfield ${robot.inMidfield() ? 'YES' : 'no'}`;
	$('source').textContent = input.source;
}

// ---- Loop ----------------------------------------------------------------------------
resetField();
let last = performance.now(), acc = 0, frame = 0, score = field.score({ red: 0, blue: 0 });
let latest = { ly: 0, rx: 0 };
// Controller states as the Brain sees them: LATENCY_S late, read once per 20 ms loop.
function delayedInput() {
	const h = sim.hist, want = robot.t - LATENCY_S;
	while (h.length > 2 && h[1].t <= want) h.shift();
	return h.length ? h[0].c : latest;
}
const isPractice = () => sim.mode === 'practice';
function control(c) {
	const ctrl = { ...c };   // don't flip the caller's copy
	if ($('invY').checked) ctrl.ly = -ctrl.ly;
	if ($('invX').checked) ctrl.rx = -ctrl.rx;
	const driverMode = sim.mode === 'practice' || sim.mode === 'driver';
	if (sim.auton && sim.autonKind !== 'match' && ctrl.L1 && ctrl.R1) { log('E-STOP (L1+R1)'); sim.auton.estop(); stopAuton('E-STOPPED'); robot.op.prev = { ...ctrl }; return; }
	if (driverMode && !sim.auton) robot.opcontrol(ctrl, {
		practiceAuton: isPractice() ? () => startAuton('practice') : null,
		macro: () => startAuton('macro'),
		side: (sd) => { if (!sim.auton) setSide(sd); },
	});
	else robot.op.prev = { ...ctrl };
}
function physicsStep() {
	sim.hist.push({ t: robot.t, c: latest });
	if ((sim.loopAcc += DT) >= LOOP_S - 1e-9) { sim.loopAcc -= LOOP_S; control(delayedInput()); }
	if (sim.auton) {
		sim.autonT += DT;
		const done = sim.auton.tick();
		if (done) stopAuton();
		else if (sim.autonKind === 'practice' && sim.autonT >= AUTON_S) { log('CUT OFF at 15s'); stopAuton('CUT OFF at 15s'); input.rumble(400); }
	}
	if (sim.mode === 'auton' || sim.mode === 'driver') {
		sim.clock -= DT;
		if (sim.clock <= 0 && sim.mode === 'auton') {
			stopAuton('time');
			score = field.score(mid(), true);
			log(`Auton score: red ${score.red} / blue ${score.blue}`);
			sim.mode = 'driver'; sim.clock = DRIVER_S;
			input.rumble(250);
		} else if (sim.clock <= 0 && sim.mode === 'driver') {
			sim.mode = 'ended'; sim.endT = robot.t;
			robot.leftCmd = robot.rightCmd = 0; robot.mechStopAll();
			for (const a of ais) a.stop();
		}
	}
	// <SC1>: scored once things settle, here 3 s after the buzzer.
	if (sim.mode === 'ended' && !sim.final && robot.t - sim.endT > 3) { sim.final = field.score(mid()); log(`FINAL red ${sim.final.red} / blue ${sim.final.blue}`); }
	const live = sim.mode === 'auton' || sim.mode === 'driver';
	for (const a of ais) if (sim.game === 'ai') { a.clockLeft = sim.mode === 'driver' ? sim.clock : null; a.period = sim.mode; if (live) a.think(DT, active()); else a.stop(); }
	for (const r of active()) r.step(DT);
	humans.step(DT, robot.alliance, sim.mode === 'practice' || sim.mode === 'driver');
	field.stepLoaders(DT);
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
	else { latest = c; while (acc >= DT) { acc -= DT; physicsStep(); } }
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
	latest = { ly: 0, rx: 0, ...ctrl };
	for (let i = 0; i < Math.round(seconds / DT); i++) physicsStep();
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
markSide();
requestAnimationFrame(tick);
$('loading').remove();
// Auton routine: routine.cpp is copied from the robot code (tools/sync-robot-code.sh). Load the whole
// file (constants, helpers and run_routine()) into the editor unless it's already been edited here.
const routineDefault = $('routine').value;
// The single-file build (TitaniumSim.html) carries routine.cpp inside it, since file:// can't fetch.
const routineSrc = window.ROUTINE_CPP != null ? Promise.resolve(window.ROUTINE_CPP)
	: fetch('routine.cpp', { cache: 'no-cache' }).then((r) => (r.ok ? r.text() : null));
routineSrc.then((src) => {
	if (!src || !/run_routine\s*\(\s*\)/.test(src) || $('routine').value !== routineDefault) return;
	$('routine').value = src.replace(/^\s*#include.*\n/gm, '').replace(/^\n+/, '');
}).catch(() => {});
// Offline app: cache every file so the site opens with no internet once it's been visited / installed.
if ('serviceWorker' in navigator && location.protocol.startsWith('http') && !window.ROUTINE_CPP) navigator.serviceWorker.register('sw.js').catch(() => {});
window.sim = { robot, ais, startGame, showMenu, input, field, world, sim, toField, IN, W, WALL, advance, resetField, startAuton, stopAuton, $, camera, controls, setCam, humans, setSide };   // for debugging in the console
