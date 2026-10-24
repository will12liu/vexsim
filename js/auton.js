// The robot's autonomous code, running in the sim. routine.cpp (C++) is translated by cpp.js; the
// commands it calls are ports of src/auton_cmds.cpp, src/mechanisms.cpp and the LemLib chassis calls
// (moveToPoint, turnToHeading, moveToPose, ...) with the PID gains from src/robot-config.cpp.
// Odometry is like the robot's: drive motor encoders + the IMU (no tracking wheels), so wheel slip,
// gear slop and getting shoved all throw it off the same way they would on the field.
// Blocking commands are generators that advance one physics step (1/120 s) per yield.
import { CFG } from './robot.js';
import { translateCpp, sprintf } from './cpp.js';

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const DEG = Math.PI / 180;
const wrap180 = (a) => ((((a + 180) % 360) + 360) % 360) - 180;
// robot-config.cpp: lateral kP 13 kD 3, angular kP 2 kD 7; exits 1 / 0 ms small, 3 / 500 ms large.
const LATERAL = { kP: 13, kI: 0, kD: 3, small: 1, smallT: 0, large: 3, largeT: 500 };
const ANGULAR = { kP: 2, kI: 0, kD: 7, small: 1, smallT: 0, large: 3, largeT: 500 };
const DRIVE_TIMEOUT_MS = 2000, TURN_TIMEOUT_MS = 1000, STRAIGHT_KP = 3, AUTON_BUDGET_MS = 14800;

// Commands that block (the translator wraps calls to them in yield*).
export const BLOCKING = ['wait', 'delay', 'drive_time', 'drive_to', 'back_dist', 'back_dist_score', 'face_heading', 'claw_grab', 'claw_release',
	'chassis.moveToPoint', 'chassis.turnToHeading', 'chassis.turnToPoint', 'chassis.moveToPose', 'chassis.swingToHeading', 'chassis.waitUntilDone', 'chassis.waitUntil',
	// older sim commands
	'move', 'turn', 'turn_to', 'lift_wait', 'claw_in', 'claw_out', 'claw_up', 'claw_drop', 'claw_drop_up', 'score',
	'back_to', 'back_past', 'lift_secs', 'face', 'drive_to_pose', 'back_to_pose'];

function pid(g) {
	let prev = null, integ = 0;
	return { update(e) { integ += e; const d = prev === null ? 0 : e - prev; prev = e; return g.kP * e + g.kI * integ + g.kD * d; }, reset() { prev = null; integ = 0; } };
}
// LemLib ExitCondition: true once the error has stayed inside `range` for `ms`.
function exitCond(range, ms) {
	let start = -1;
	return (err, now) => {
		if (Math.abs(err) > range) { start = -1; return false; }
		if (start < 0) start = now;
		return now - start >= ms;
	};
}

export function makeAuton(robot, sel /* { side, color } */, log, opts = {}) {
	const s = { stopped: false, estop: false, start: robot.t, lastStep: '', lastStepMs: 0, motion: null, moveKp: 8, moveKd: 0.4, turnKp: 1.6, turnKd: 0.12, liftTarget: 0 };
	const millis = () => Math.round((robot.t - s.start) * 1000);
	const status = (msg) => { s.lastStep = String(msg).slice(0, 23); s.lastStepMs = millis(); log?.(`${(millis() / 1000).toFixed(2)}s ${msg}`); };
	const stopped = () => s.stopped || s.estop;
	const setLR = (l, r) => { robot.leftCmd = clamp(Math.trunc(l), -127, 127); robot.rightCmd = clamp(Math.trunc(r), -127, 127); };
	const stopDrive = () => setLR(0, 0);
	robot.brake = 'brake';   // auton_cmds_reset(): drive motors to BRAKE (they stay that way in driver control)

	// ---- Odometry (LemLib, no tracking wheels): motor encoders + IMU ---------------------------
	const IN_PER_M = 1 / 0.0254;
	const od = { x: 0, y: 0, th: 0, encL: robot.drive.encL, encR: robot.drive.encR, imu: robot.imuDeg, ok: true };
	const odomStep = () => {
		const dL = (robot.drive.encL - od.encL) * IN_PER_M, dR = (robot.drive.encR - od.encR) * IN_PER_M;
		od.encL = robot.drive.encL; od.encR = robot.drive.encR;
		const dth = robot.imuDeg - od.imu; od.imu = robot.imuDeg;
		const ds = (dL + dR) / 2, mid = (od.th + dth / 2) * DEG;
		od.x += ds * Math.sin(mid); od.y += ds * Math.cos(mid); od.th += dth;
	};
	const getPose = () => ({ x: od.x, y: od.y, theta: od.th });
	const setPose = (x, y, th) => { od.x = +x; od.y = +y; od.th = +th; };

	// ---- Generators used everywhere -------------------------------------------------------------
	function* delay(ms) { const end = robot.t + ms / 1000; while (robot.t < end - 1e-9) yield; }
	function* wait(ms) { const end = robot.t + ms / 1000; while (robot.t < end - 1e-9 && !stopped()) yield; }
	// Run `fn` every 10 ms (LemLib / robot loop rate) until it returns true or the timeout passes.
	function* every10(timeout, fn) {
		const t0 = millis(); let next = -1;
		while (!stopped() && millis() - t0 < timeout) {
			if (millis() >= next) { next = millis() + 10; if (fn(millis())) return true; }
			yield;
		}
		return false;
	}

	// ---- LemLib chassis -----------------------------------------------------------------------------
	const angleTo = (p, x, y) => Math.atan2(x - p.x, y - p.y) / DEG;   // clockwise from +y
	function* motion(gen, async) {
		yield* chassis.waitUntilDone();
		if (async) { s.motion = gen; return; }
		yield* gen;
	}
	function* moveToPointGen(x, y, timeout, p) {
		const fwd = p.forwards !== false, maxS0 = clamp(p.maxSpeed ?? 127, 0, 127), minS = Math.abs(p.minSpeed ?? 0), early = p.earlyExitRange ?? 0;
		const lat = pid(LATERAL), ang = pid(ANGULAR), small = exitCond(LATERAL.small, LATERAL.smallT), large = exitCond(LATERAL.large, LATERAL.largeT);
		let close = false, maxS = maxS0, prevLat = 0;
		yield* every10(timeout, (now) => {
			const q = getPose(), d = Math.hypot(x - q.x, y - q.y);
			if (d < 7.5 && !close) { close = true; maxS = Math.max(Math.abs(prevLat), 60); }
			const robotTh = fwd ? q.theta : q.theta + 180;
			const aErr = wrap180(angleTo(q, x, y) - robotTh);
			const latErr = d * Math.cos(wrap180(angleTo(q, x, y) - q.theta) * DEG);
			if (early > 0 && d < early) return true;
			if (minS !== 0 && close && latErr * (fwd ? 1 : -1) < 0) return true;   // went past it with minSpeed set: chain into the next move
			if (small(latErr, now) || large(latErr, now)) return true;
			let lo = lat.update(latErr), ao = close ? 0 : ang.update(aErr);
			lo = clamp(lo, -maxS, maxS); ao = clamp(ao, -maxS0, maxS0);
			if (fwd && !close) lo = Math.max(lo, 0); else if (!fwd && !close) lo = Math.min(lo, 0);
			if (fwd && lo < minS && lo > 0) lo = minS;
			if (!fwd && -lo < minS && lo < 0) lo = -minS;
			const sum = Math.abs(lo) + Math.abs(ao);
			if (sum > maxS0) { lo *= maxS0 / sum; ao *= maxS0 / sum; }
			prevLat = lo;
			setLR(lo + ao, lo - ao);
			return false;
		});
		stopDrive();
	}
	function* turnToHeadingGen(target, timeout, p) {
		const maxS = clamp(p.maxSpeed ?? 127, 0, 127), minS = Math.abs(p.minSpeed ?? 0), early = p.earlyExitRange ?? 0;
		const dir = String(p.direction || 'AUTO');
		const a = pid(ANGULAR), small = exitCond(ANGULAR.small, ANGULAR.smallT), large = exitCond(ANGULAR.large, ANGULAR.largeT);
		let forced = dir !== 'AUTO';
		yield* every10(timeout, (now) => {
			const th = getPose().theta;
			let err = wrap180(target - th);
			if (forced) {
				if (Math.abs(err) < 30) forced = false;
				else if (dir.startsWith('CW')) err = ((target - th) % 360 + 360) % 360;
				else err = -(((th - target) % 360 + 360) % 360);
			}
			if (early > 0 && Math.abs(err) < early) return true;
			if (small(err, now) || large(err, now)) return true;
			let o = clamp(a.update(err), -maxS, maxS);
			if (minS && Math.abs(o) < minS) o = Math.sign(o || err) * minS;
			setLR(o, -o);
			return false;
		});
		stopDrive();
	}
	function* moveToPoseGen(x, y, th, timeout, p) {
		const fwd = p.forwards !== false, lead = p.lead ?? 0.6, maxS = clamp(p.maxSpeed ?? 127, 0, 127);
		const lat = pid(LATERAL), ang = pid(ANGULAR), small = exitCond(LATERAL.small, LATERAL.smallT);
		const hx = Math.sin(th * DEG), hy = Math.cos(th * DEG), dirS = fwd ? 1 : -1;
		let close = false;
		yield* every10(timeout, (now) => {
			const q = getPose(), d = Math.hypot(x - q.x, y - q.y);
			if (d < 7.5) close = true;
			const c = close ? { x, y } : { x: x - hx * dirS * d * lead, y: y - hy * dirS * d * lead };   // boomerang carrot
			const robotTh = fwd ? q.theta : q.theta + 180;
			const aErr = close ? wrap180(th - q.theta) : wrap180(angleTo(q, c.x, c.y) - robotTh);
			const latErr = Math.hypot(c.x - q.x, c.y - q.y) * Math.cos(wrap180(angleTo(q, c.x, c.y) - q.theta) * DEG);
			if (close && small(d * Math.sign(latErr), now) && Math.abs(wrap180(th - q.theta)) < 3) return true;
			let lo = clamp(lat.update(latErr), -maxS, maxS), ao = clamp(ang.update(aErr), -maxS, maxS);
			const sum = Math.abs(lo) + Math.abs(ao);
			if (sum > maxS) { lo *= maxS / sum; ao *= maxS / sum; }
			setLR(lo + ao, lo - ao);
			return false;
		});
		stopDrive();
	}
	const chassis = {
		getPose, setPose: (x, y, t) => setPose(x, y, t),
		*moveToPoint(x, y, timeout, p = {}, async = true) { yield* motion(moveToPointGen(x, y, timeout, p), async); },
		*turnToHeading(t, timeout, p = {}, async = true) { yield* motion(turnToHeadingGen(t, timeout, p), async); },
		*turnToPoint(x, y, timeout, p = {}, async = true) {
			const q = getPose(); let t = angleTo(q, x, y); if (p.forwards === false) t += 180;
			yield* motion(turnToHeadingGen(q.theta + wrap180(t - q.theta), timeout, p), async);
		},
		*swingToHeading(t, side, timeout, p = {}, async = true) { yield* motion(turnToHeadingGen(t, timeout, p), async); },
		*moveToPose(x, y, t, timeout, p = {}, async = true) { yield* motion(moveToPoseGen(x, y, t, timeout, p), async); },
		*waitUntilDone() { while (s.motion && !stopped()) yield; },
		*waitUntil() { yield* this.waitUntilDone(); },
		cancelAllMotions() { s.motion = null; stopDrive(); },
		cancelMotion() { s.motion = null; stopDrive(); },
		isInMotion: () => !!s.motion,
		calibrate() {},
	};

	// ---- src/auton_cmds.cpp ----------------------------------------------------------------------
	const odomOk = (step) => { status(step); return !stopped(); };
	function* drive_time(speed, ms) {
		status('drive_time');
		const head = robot.imuDeg, end = robot.t + ms / 1000;
		while (robot.t < end - 1e-9 && !stopped()) {
			const turn = Math.trunc(clamp(STRAIGHT_KP * (head - robot.imuDeg), -40, 40));
			setLR(speed + turn, speed - turn);
			yield* delay(10);
		}
		stopDrive();
	}
	function* alignDrive(t) {
		const q = getPose(), dx = Math.abs(t.x - q.x), dy = Math.abs(t.y - q.y);
		let h;
		if (dx < 1.5 && dy >= dx) { t.x = q.x; h = t.y > q.y ? 0 : 180; }
		else if (dy < 1.5 && dx > dy) { t.y = q.y; h = t.x > q.x ? 90 : 270; }
		else return;
		if (Math.max(dx, dy) < 3) return;
		const err = ((h - q.theta + 540) % 360 + 360) % 360 - 180;
		if (Math.abs(err) > 1.5 && !stopped()) yield* chassis.turnToHeading(h, TURN_TIMEOUT_MS, { maxSpeed: 100 }, false);
	}
	function* drive_to(x, y, speed = 100, timeout = DRIVE_TIMEOUT_MS, align = true) {
		if (!odomOk('drive_to')) return;
		const t = { x, y };
		if (align) yield* alignDrive(t);
		yield* chassis.moveToPoint(t.x, t.y, timeout, { forwards: true, maxSpeed: clamp(speed, 0, 127) }, false);
	}
	function* back_dist(inches, speed = 100, timeout = DRIVE_TIMEOUT_MS) {
		if (!odomOk('back_dist')) return;
		const p0 = getPose(), head = robot.imuDeg, t0 = millis();
		while (millis() - t0 < timeout && !stopped()) {
			const q = getPose(), rem = inches - Math.hypot(q.x - p0.x, q.y - p0.y);
			if (rem <= 0.5) break;
			let pw = clamp(speed, 0, 127);
			if (rem < 6) pw = Math.max(35, Math.trunc((pw * rem) / 6));
			const turn = Math.trunc(clamp(STRAIGHT_KP * (head - robot.imuDeg), -40, 40));
			setLR(-pw + turn, -pw - turn);
			yield* delay(10);
		}
		stopDrive();
	}
	function* back_dist_score(inches, speed, timeout, slowdown, minSpeed) {
		if (!odomOk('back_dist_score')) return 'FAILED';
		const p0 = getPose(), rad = p0.theta * DEG, head = robot.imuDeg, maxP = clamp(speed, 0, 127), minP = clamp(minSpeed, 0, maxP);
		const t0 = millis();
		let arrived = false, stalled = false, lastD = 0, lastT = t0;
		while (millis() - t0 < timeout && !stopped()) {
			const q = getPose(), dist = -((q.x - p0.x) * Math.sin(rad) + (q.y - p0.y) * Math.cos(rad)), rem = inches - dist;
			if (rem <= 0.5) { arrived = true; break; }
			if (dist >= lastD + 0.1) { lastD = dist; lastT = millis(); }
			if (rem <= slowdown && millis() - lastT >= 250) { stalled = true; break; }
			let pw = maxP;
			if (slowdown > 0 && rem < slowdown) pw = minP + Math.trunc(((maxP - minP) * rem) / slowdown);
			const turn = Math.trunc(clamp(STRAIGHT_KP * (head - robot.imuDeg), -40, 40));
			setLR(-pw + turn, -pw - turn);
			yield* delay(10);
		}
		stopDrive();
		if (!arrived && !stopped()) status(stalled ? 'score stalled' : 'reverse timeout');
		if (stopped()) return 'FAILED';
		return arrived ? 'REACHED' : stalled ? 'STALLED_NEAR_GOAL' : 'FAILED';
	}
	function* face_heading(h, speed = 100, timeout = TURN_TIMEOUT_MS) {
		if (odomOk('face_heading')) yield* chassis.turnToHeading(h, timeout, { maxSpeed: clamp(speed, 0, 127) }, false);
	}
	function* claw_grab(ms = 0) { status('claw_grab'); if (stopped()) return; robot.clawGrabToggle(); if (ms <= 0) return; yield* wait(ms); robot.clawDriverReset(); }
	function* claw_release(ms = 0) { status('claw_release'); if (stopped()) return; robot.clawReleaseToggle(); if (ms <= 0) return; yield* wait(ms); robot.clawDriverReset(); }
	const timeLeft = () => Math.max(0, AUTON_BUDGET_MS - millis());

	// ---- Older sim commands (routines from before 10/21 still run) ------------------------------
	const posIn = () => (((robot.leftPos + robot.rightPos) / 2) / 360) * Math.PI * CFG.wheelDia * (CFG.driveRpm / CFG.motorRpm);
	let goalHeading = robot.imuDeg;
	const startHeading = robot.imuDeg;
	function* move(inches, speed = 100, kP = -1, kD = -1, timeout = 0) {
		status(`move ${inches}`);
		if (kP < 0) kP = s.moveKp; if (kD < 0) kD = s.moveKd;
		speed = clamp(speed, 0, 127); if (timeout <= 0) timeout = 1000 + Math.abs(inches) * 60;
		const start = posIn(), t0 = millis();
		let last = inches, prevOut = 0, settled = 0;
		yield* every10(timeout, () => {
			const err = inches - (posIn() - start), der = (err - last) / 0.01; last = err;
			let out = clamp(kP * err + kD * der, -speed, speed);
			out = clamp(out, prevOut - 25, prevOut + 25);
			if (Math.abs(err) > 0.5 && Math.abs(out) < 15) out = err > 0 ? 15 : -15;
			prevOut = out;
			const turn = clamp(STRAIGHT_KP * (goalHeading - robot.imuDeg), -40, 40);
			setLR(out + turn, out - turn);
			settled = Math.abs(err) < 0.5 && Math.abs(der) < 3 ? settled + 10 : 0;
			return settled >= 100;
		});
		void t0;
		stopDrive();
	}
	function* turnImu(target, speed, kP, kD, timeout) {
		goalHeading = target; if (kP < 0) kP = s.turnKp; if (kD < 0) kD = s.turnKd;
		if (timeout <= 0) timeout = 1500 + Math.abs(target - robot.imuDeg) * 10;
		let last = target - robot.imuDeg, settled = 0;
		yield* every10(timeout, () => {
			const err = target - robot.imuDeg, der = (err - last) / 0.01; last = err;
			let out = clamp(kP * err + kD * der, -speed, speed);
			if (Math.abs(err) > 1.5 && Math.abs(out) < 18) out = err > 0 ? 18 : -18;
			setLR(out, -out);
			settled = Math.abs(err) < 1.5 && Math.abs(der) < 10 ? settled + 10 : 0;
			return settled >= 150;
		});
		stopDrive();
	}
	function* turn(deg, speed = 100, kP = -1, kD = -1, timeout = 0) { status(`turn ${deg}`); yield* turnImu(goalHeading + deg, speed, kP, kD, timeout); }
	function* turn_to(deg, speed = 100, kP = -1, kD = -1, timeout = 0) { status(`turn_to ${deg}`); yield* turnImu(startHeading + deg, speed, kP, kD, timeout); }
	const liftToDeg = (d) => { s.liftTarget = d; robot.liftTo(-d); };
	function* lift_wait(timeout = 1500) { status('lift_wait'); const t0 = millis(); while (!stopped() && Math.abs(robot.liftDeg - s.liftTarget) > 20 && millis() - t0 < timeout) yield; }
	function* lift_secs(ms, up) { status(`lift_secs ${ms}`); robot.liftRelease(); robot.cascadeMove(up ? 127 : -85); yield* wait(ms); robot.cascadeMove(0); }
	function* claw_drop(speed = 1000, ms = 0) { status('claw_drop'); robot.clawMode = 'RELEASE'; yield* wait(ms > 0 ? ms : 700); robot.clawMode = 'STOPPED'; }
	function* skip(name) { status(`${name} (no claw flip: skipped)`); }
	function* chase(x, y, fwd, speed, timeout) { yield* chassis.moveToPoint(x, y, timeout, { forwards: fwd, maxSpeed: speed }, false); }
	const bearing = (x, y) => angleTo(getPose(), x, y);

	const mirror = (d) => ((sel.side === 'right') !== (sel.color === 'blue') ? -d : d);   // blue swaps the side, like run_routine()
	const lemlib = { AngularDirection: {} };
	const motors = (side) => ({
		move: (v) => { if (side === 'L') robot.leftCmd = clamp(v, -127, 127); else robot.rightCmd = clamp(v, -127, 127); },
		move_velocity: (v) => { const c = clamp((v / 600) * 127, -127, 127); if (side === 'L') robot.leftCmd = c; else robot.rightCmd = c; },
		set_brake_mode_all: (m) => { robot.brake = /BRAKE/.test(String(m)) ? 'brake' : /HOLD/.test(String(m)) ? 'hold' : 'coast'; },
		set_brake_mode: (m) => { robot.brake = /BRAKE/.test(String(m)) ? 'brake' : /HOLD/.test(String(m)) ? 'hold' : 'coast'; },
		brake: () => {}, get_position: () => (side === 'L' ? robot.leftPos : robot.rightPos), tare_position: () => {},
	});
	const api = {
		// auton_cmds.h
		set_pose: (x, y, h) => { if (odomOk('set_pose')) setPose(x, y, h); },
		drive_time, drive_to, back_dist, back_dist_score, face_heading, wait, delay, claw_grab, claw_release,
		lift_to_rot: (rot) => { status('lift_to_rot'); if (!stopped()) robot.liftTo(rot); },
		on_right: () => sel.side === 'right', on_left: () => sel.side !== 'right',
		is_blue: () => sel.color === 'blue', is_red: () => sel.color !== 'blue',
		auton_time_left: timeLeft, auton_out_of_time: () => false, auton_stopped: stopped,
		auton_status: status, auton_odom_ready: () => true, auton_imu_ready: () => true,
		auton_last_step: () => s.lastStep, auton_last_step_ms: () => s.lastStepMs,
		DRIVE_TIMEOUT_MS, TURN_TIMEOUT_MS,
		// mechanisms.h
		lift_to: (rot) => robot.liftTo(rot), lift_release: () => robot.liftRelease(), lift_pos: () => robot.liftPos(),
		cascade_move: (p) => robot.cascadeMove(p), lift_drive_scale: () => robot.liftDriveScale(),
		claw_grab_toggle: () => robot.clawGrabToggle(), claw_release_toggle: () => robot.clawReleaseToggle(),
		claw_driver_update: () => {}, claw_driver_reset: () => robot.clawDriverReset(), claw_auton_start: () => {},
		mech_stop_all: () => robot.mechStopAll(), lift_task_start: () => {},
		CLAW_GRAB_SPEED: CFG.CLAW_GRAB_SPEED, CLAW_RELEASE_SPEED: CFG.CLAW_RELEASE_SPEED,
		LIFT_LIMIT_BOTTOM_DEG: CFG.LIFT_LIMIT_BOTTOM_DEG, LIFT_LIMIT_TOP_DEG: CFG.LIFT_LIMIT_TOP_DEG, LIFT_HOLD_BAND_DEG: CFG.LIFT_HOLD_BAND_DEG, LIFT_RECORRECT_DEG: CFG.LIFT_RECORRECT_DEG,
		// robot-config.h devices
		chassis, lemlib, imu: { get_rotation: () => robot.imuDeg, get_heading: () => ((robot.imuDeg % 360) + 360) % 360, reset: () => {} },
		left_motors: motors('L'), right_motors: motors('R'),
		cascade: { move: (p) => robot.cascadeMove(p), get_position: () => robot.liftPos(), tare_position_all: () => {}, set_brake_mode_all: () => {} },
		claw_spin: { move: (v) => { robot.clawMode = v > 30 ? 'RELEASE' : v < -30 ? 'GRAB' : 'STOPPED'; } },
		lift_rot: { get_position: () => robot.liftPos() * 100, reset_position: () => {} },
		millis, sprintf, printf: (...a) => log?.(sprintf(...a).trim()),
		clamp, fmod: (a, b) => a % b,
		// older sim commands
		move, turn, turn_to, lift_wait, lift_secs, claw_drop, claw_drop_up: claw_drop, score: function* () { liftToDeg(CFG.LIFT_ALLIANCE_GOAL_DEG); yield* lift_wait(); yield* claw_drop(); },
		claw_in: () => skip('claw_in'), claw_out: () => skip('claw_out'), claw_up: () => skip('claw_up'),
		claw_out_start: () => status('claw_out_start (skipped)'), claw_up_start: () => status('claw_up_start (skipped)'),
		move_pid: (p, d) => { s.moveKp = p; s.moveKd = d; }, turn_pid: (p, d) => { s.turnKp = p; s.turnKd = d; },
		lift_goal: () => { status('lift_goal'); liftToDeg(CFG.LIFT_ALLIANCE_GOAL_DEG); }, lift_down: () => { status('lift_down'); liftToDeg(0); },
		intake: () => status('intake (no intake: skipped)'), spin_claw: (v) => { robot.clawMode = v > 30 ? 'RELEASE' : v < -30 ? 'GRAB' : 'STOPPED'; },
		mirror,
		back_to: (x, y, sp = 100, to = 3000) => chase(x, y, false, sp, to),
		back_past: (x, y, sp = 100, to = 3000) => chassis.moveToPoint(x, y, to, { forwards: false, maxSpeed: sp, minSpeed: 30, earlyExitRange: 2 }, false),
		face: (x, y, sp = 100, to = 1500) => face_heading(bearing(x, y), sp, to),
		drive_to_pose: (x, y, h, sp = 100, to = 4000) => chassis.moveToPose(x, y, h, to, { maxSpeed: sp }, false),
		back_to_pose: (x, y, h, sp = 100, to = 4000) => chassis.moveToPose(x, y, h, to, { forwards: false, maxSpeed: sp }, false),
	};

	let gen = null;
	return {
		state: s, pose: getPose,
		compile(src) {
			const { code } = translateCpp(src, BLOCKING);
			const names = Object.keys(api);
			const factory = new Function(...names, `return function* () {\n${code}\n};`);
			gen = factory(...names.map((n) => api[n]))();
			return gen;
		},
		// One physics step: odometry, a running async motion, then the routine. Returns true when done.
		tick() {
			odomStep();
			if (s.motion) { const r = s.motion.next(); if (r.done) s.motion = null; }
			if (!gen) return true;
			const r = gen.next();
			return r.done;
		},
		estop() { s.estop = true; robot.mechStopAll(); stopDrive(); },
		stop() { s.stopped = true; },
		// autonomous() epilogue: stop the drive + mech_stop_all().
		finish() { s.motion = null; stopDrive(); robot.mechStopAll(); },
		get lastStep() { return s.lastStep; }, get lastStepMs() { return s.lastStepMs; },
	};
}
