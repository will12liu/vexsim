// Port of src/auton_cmds.cpp. Your run_routine() code runs almost unchanged: blocking
// commands become generator calls that advance one physics step (1/120 s) per yield.
import { CFG } from './robot.js';
import { toField } from './field.js';

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const BLOCKING = ['move', 'turn_to', 'turn', 'drive_time', 'wait', 'lift_wait', 'claw_in', 'claw_out', 'claw_up', 'claw_drop', 'score', 'claw_drop_up',
	'drive_to', 'back_to', 'face', 'face_heading', 'drive_to_pose', 'back_to_pose'];

// C++ routine text -> generator function source.
export function translate(src) {
	let code = src;
	const m = code.match(/run_routine\s*\(\s*\)\s*\{/);
	if (m) code = code.slice(m.index + m[0].length, code.lastIndexOf('}'));
	code = code
		.replace(/\((int|float|double)\)\s*/g, '')
		.replace(/(\d+\.\d*|\.\d+|\d+)f\b/g, '$1')
		.replace(/\b(const\s+)?(int|float|double|bool|auto)\s+(?=[A-Za-z_])/g, 'let ')
		.replace(new RegExp(`\\b(${BLOCKING.join('|')})\\s*\\(`, 'g'), 'yield* $1(');
	return code;
}

export function makeAuton(robot, sel /* { side: 'left'|'right', color: 'red'|'blue' } */, log) {
	const s = {
		moveKp: 8, moveKd: 0.4, turnKp: 1.6, turnKd: 0.12,
		startHeading: robot.imuDeg, goalHeading: robot.imuDeg, liftTarget: 0, stopped: false,
	};
	const STRAIGHT_KP = 3, ACCEL_STEP = 25, MOVE_MIN = 15, TURN_MIN = 18, MOVE_TOL = 0.5, TURN_TOL = 1.5;
	const now = () => robot.t;
	let lastT = robot.t;
	const dt = () => { const d = Math.max(robot.t - lastT, 1e-4); lastT = robot.t; return d; };
	const posIn = () => (((robot.leftPos + robot.rightPos) / 2) / 360) * Math.PI * CFG.wheelDia;
	const steer = () => clamp(STRAIGHT_KP * (s.goalHeading - robot.imuDeg), -40, 40);
	const driveLR = (base, t) => { robot.leftCmd = clamp(Math.trunc(base + t), -127, 127); robot.rightCmd = clamp(Math.trunc(base - t), -127, 127); };
	const stop = () => { robot.leftCmd = 0; robot.rightCmd = 0; };
	const status = (msg) => log?.(`${(now() - t0).toFixed(2)}s ${msg}`);
	const t0 = now();
	robot.brake = 'brake';

	function* wait(ms) { const end = now() + ms / 1000; while (now() < end && !s.stopped) yield; }

	function* drive_time(speed, ms) {
		status('drive_time');
		const end = now() + ms / 1000;
		while (now() < end && !s.stopped) { driveLR(speed, steer()); yield; }
		stop();
	}

	function* move(inches, speed = 100, kP = -1, kD = -1, timeout = 0) {
		status(`move ${inches}`);
		if (kP < 0) kP = s.moveKp; if (kD < 0) kD = s.moveKd;
		speed = clamp(speed, 0, 127);
		if (timeout <= 0) timeout = 1000 + Math.abs(inches) * 60;
		const start = posIn(), t0 = now();
		let lastErr = inches, outPrev = 0, settled = 0;
		dt();
		yield;
		while (!s.stopped && (now() - t0) * 1000 < timeout) {
			const d = dt();
			const err = inches - (posIn() - start), deriv = (err - lastErr) / d;
			lastErr = err;
			let out = clamp(kP * err + kD * deriv, -speed, speed);
			const acc = ACCEL_STEP * (d / 0.01);
			out = clamp(out, outPrev - acc, outPrev + acc);
			if (Math.abs(err) > MOVE_TOL && Math.abs(out) < MOVE_MIN) out = err > 0 ? MOVE_MIN : -MOVE_MIN;
			outPrev = out;
			driveLR(out, steer());
			settled = Math.abs(err) < MOVE_TOL && Math.abs(deriv) < 3 ? settled + d : 0;
			if (settled >= 0.1) break;
			yield;
		}
		stop();
	}

	function* turnToHeading(target, speed, kP, kD, timeout) {
		s.goalHeading = target;
		if (kP < 0) kP = s.turnKp; if (kD < 0) kD = s.turnKd;
		speed = clamp(speed, 0, 127);
		if (timeout <= 0) timeout = 1500 + Math.abs(target - robot.imuDeg) * 10;
		let lastErr = target - robot.imuDeg, settled = 0;
		const t0 = now();
		dt();
		yield;
		while (!s.stopped && (now() - t0) * 1000 < timeout) {
			const d = dt();
			const err = target - robot.imuDeg, deriv = (err - lastErr) / d;
			lastErr = err;
			let out = clamp(kP * err + kD * deriv, -speed, speed);
			if (Math.abs(err) > TURN_TOL && Math.abs(out) < TURN_MIN) out = err > 0 ? TURN_MIN : -TURN_MIN;
			robot.leftCmd = clamp(Math.trunc(out), -127, 127); robot.rightCmd = clamp(Math.trunc(-out), -127, 127);
			settled = Math.abs(err) < TURN_TOL && Math.abs(deriv) < 10 ? settled + d : 0;
			if (settled >= 0.15) break;
			yield;
		}
		stop();
	}
	function* turn(deg, speed = 100, kP = -1, kD = -1, timeout = 0) { status(`turn ${deg}`); yield* turnToHeading(s.goalHeading + deg, speed, kP, kD, timeout); }
	function* turn_to(deg, speed = 100, kP = -1, kD = -1, timeout = 0) { status(`turn_to ${deg}`); yield* turnToHeading(s.startHeading + deg, speed, kP, kD, timeout); }

	const liftTo = (deg) => { s.liftTarget = deg; robot.liftMode = 'pos'; robot.liftTarget = deg; robot.liftVel = CFG.LIFT_VELOCITY; };
	function* lift_wait(timeout = 1500) {
		status('lift_wait');
		const t0 = now();
		while (!s.stopped && Math.abs(robot.liftDeg - s.liftTarget) > 20 && (now() - t0) * 1000 < timeout) yield;
	}
	function* clawTo(deg, vel, timeout, tol = 30) {
		robot.clawTarget = deg; robot.clawVel = vel;
		const t0 = now();
		while (!s.stopped && Math.abs(robot.clawDeg - deg) > tol && (now() - t0) * 1000 < timeout) yield;
	}
	function* claw_in() { status('claw_in'); robot.clawTurnHorizontal = false; yield* clawTo(CFG.CLAW_TURN_VERTICAL_DEG, CFG.CLAW_TURN_DOWN_VELOCITY, 2000); }
	function* claw_out() { status('claw_out'); robot.clawTurnHorizontal = true; yield* clawTo(CFG.CLAW_TURN_HORIZONTAL_DEG, CFG.CLAW_TURN_VELOCITY, 200); }   // waits at most 200 ms (10/15)
	// Start the flip and keep going (no wait), so it flips while the robot drives.
	const claw_out_start = () => { status('claw_out_start'); robot.clawTurnHorizontal = true; robot.clawTarget = CFG.CLAW_TURN_HORIZONTAL_DEG; robot.clawVel = CFG.CLAW_TURN_VELOCITY; };
	const claw_up_start = () => { status('claw_up_start'); robot.clawTurnHorizontal = true; robot.clawTarget = CFG.CLAW_TURN_LIFT_DEG; robot.clawVel = CFG.CLAW_TURN_VELOCITY; };
	// Drop the pin and lift the claw slightly up at the same time; waits for a started flip first.
	function* claw_drop_up(speed = 1000, ms = 0) {
		status('claw_drop_up');
		if (speed === 1000) speed = CFG.CLAW_DROP_SPEED;
		if (ms <= 0) ms = CFG.CLAW_DROP_MS;
		yield* clawTo(robot.clawTarget, robot.clawVel, 1000);
		if (s.stopped) return;
		robot.clawSpin = speed; robot.clawTurnHorizontal = true; robot.clawTarget = CFG.CLAW_TURN_LIFT_DEG; robot.clawVel = CFG.CLAW_TURN_VELOCITY;
		yield* wait(ms); robot.clawSpin = 0;
	}
	function* claw_up() { status('claw_up'); if (s.stopped) return; robot.clawTurnHorizontal = true; yield* clawTo(CFG.CLAW_TURN_LIFT_DEG, CFG.CLAW_TURN_VELOCITY, 1500); }
	function* claw_drop(speed = 1000, ms = 0) {
		status('claw_drop');
		if (speed === 1000) speed = CFG.CLAW_DROP_SPEED;
		if (ms <= 0) ms = CFG.CLAW_DROP_MS;
		robot.clawSpin = speed; yield* wait(ms); robot.clawSpin = 0;
	}
	// ---- LemLib odometry (drive_to, face, ...): the sim knows exactly where the robot is, so this
	// is perfect odometry. Pose frame like the robot code: (0, 0) = robot centre at auton start,
	// +y = the way it faced at the start, +x = to its right, heading 0 = start direction, clockwise +.
	const DEG = Math.PI / 180, wrap180 = (a) => ((((a + 180) % 360) + 360) % 360) - 180;
	const fieldPos = () => toField(robot.pos());
	const fieldHeading = () => { const q = robot.quat(); const fx = 1 - 2 * (q.y * q.y + q.z * q.z), fz = 2 * (q.x * q.z - q.w * q.y); return Math.atan2(-fz, fx); };
	const frame = { ox: 0, oy: 0, th: 0 };   // field origin + field angle (CCW rad) of the pose +y axis
	const setFrame = (x, y, h) => {
		const p = fieldPos(); frame.th = fieldHeading() + h * DEG;
		const u = [Math.cos(frame.th), Math.sin(frame.th)], r = [Math.sin(frame.th), -Math.cos(frame.th)];
		frame.ox = p.fx - x * r[0] - y * u[0]; frame.oy = p.fy - x * r[1] - y * u[1];
	};
	setFrame(0, 0, 0);
	const pose = () => {
		const p = fieldPos(), dx = p.fx - frame.ox, dy = p.fy - frame.oy;
		const u = [Math.cos(frame.th), Math.sin(frame.th)], r = [Math.sin(frame.th), -Math.cos(frame.th)];
		return { x: dx * r[0] + dy * r[1], y: dx * u[0] + dy * u[1], h: wrap180((frame.th - fieldHeading()) / DEG) };
	};
	const bearing = (p, x, y) => Math.atan2(x - p.x, y - p.y) / DEG;   // clockwise from +y
	const LIN_KP = 8, ANG_KP = 2.2;
	// Drive toward (x, y); `carrot` (optional) gives an intermediate point each step (boomerang).
	function* chase(x, y, fwd, speed, timeout, carrot, exitIn = 1) {
		const t0 = now(), max = clamp(speed, 0, 127);
		dt(); yield;
		while (!s.stopped && (now() - t0) * 1000 < timeout) {
			const p = pose(), dist = Math.hypot(x - p.x, y - p.y);
			if (dist < exitIn) break;
			const c = carrot ? carrot(p, dist) : { x, y };
			let err = wrap180(bearing(p, c.x, c.y) + (fwd ? 0 : 180) - p.h);
			let lin = clamp(LIN_KP * dist, -max, max) * Math.max(0, Math.cos(err * DEG)) * (fwd ? 1 : -1);
			if (dist > exitIn && Math.abs(lin) < MOVE_MIN && Math.abs(err) < 60) lin = fwd ? MOVE_MIN : -MOVE_MIN;
			const ang = clamp(ANG_KP * err, -max, max);
			driveLR(lin, dist < 2.5 ? 0 : ang);
			yield;
		}
		stop();
		s.goalHeading = robot.imuDeg;
	}
	function* faceDeg(target, speed, timeout) {   // pose heading -> IMU target
		yield* turnToHeading(robot.imuDeg + wrap180(target - pose().h), speed, -1, -1, timeout);
	}
	const set_pose = (x, y, h) => { status(`set_pose ${x} ${y} ${h}`); setFrame(x, y, h); };
	function* drive_to(x, y, speed = 100, timeout = 3000) { status(`drive_to ${x} ${y}`); yield* chase(x, y, true, speed, timeout); }
	function* back_to(x, y, speed = 100, timeout = 3000) { status(`back_to ${x} ${y}`); yield* chase(x, y, false, speed, timeout); }
	function* face(x, y, speed = 100, timeout = 1500) { status(`face ${x} ${y}`); yield* faceDeg(bearing(pose(), x, y), speed, timeout); }
	function* face_heading(h, speed = 100, timeout = 1500) { status(`face_heading ${h}`); yield* faceDeg(h, speed, timeout); }
	function* toPose(x, y, h, fwd, speed, timeout) {
		const t0 = now();
		const front = [Math.sin(h * DEG), Math.cos(h * DEG)], dir = fwd ? 1 : -1;   // travel direction = front * dir
		// Boomerang: aim at a point behind the target along the final direction, so it curves in.
		yield* chase(x, y, fwd, speed, timeout, (p, d) => (d < 6 ? { x, y } : { x: x - front[0] * dir * d * 0.6, y: y - front[1] * dir * d * 0.6 }), 2);
		if (!s.stopped) yield* faceDeg(h, speed, Math.max(800, timeout - (now() - t0) * 1000));
	}
	function* drive_to_pose(x, y, h, speed = 100, timeout = 4000) { status(`drive_to_pose ${x} ${y} ${h}`); yield* toPose(x, y, h, true, speed, timeout); }
	function* back_to_pose(x, y, h, speed = 100, timeout = 4000) { status(`back_to_pose ${x} ${y} ${h}`); yield* toPose(x, y, h, false, speed, timeout); }

	function* score() { yield* lift_goal_wait(); if (s.stopped) return; yield* claw_out(); if (s.stopped) return; yield* claw_drop(); if (s.stopped) return; yield* claw_up(); }
	function* lift_goal_wait() { api.lift_goal(); yield* lift_wait(); }

	const api = {
		move, turn, turn_to, drive_time, wait, lift_wait, claw_in, claw_out, claw_up, claw_drop, score, claw_out_start, claw_up_start, claw_drop_up,
		set_pose, drive_to, back_to, face, face_heading, drive_to_pose, back_to_pose,
		move_pid: (p, d) => { s.moveKp = p; s.moveKd = d; },
		turn_pid: (p, d) => { s.turnKp = p; s.turnKd = d; },
		lift_goal: () => { status('lift_goal'); liftTo(CFG.LIFT_ALLIANCE_GOAL_DEG); },
		lift_down: () => { status('lift_down'); liftTo(0); },
		intake: (v) => { robot.intakeCmd = v; },
		spin_claw: (v) => { robot.clawSpin = v; },
		on_right: () => sel.side === 'right', on_left: () => sel.side !== 'right',
		is_red: () => sel.color === 'red', is_blue: () => sel.color !== 'red',
		mirror: (d) => (sel.side === 'right' ? -d : d),
		printf: (...a) => log?.(a.join(' ')),
	};

	return {
		state: s,
		compile(src) {
			const names = Object.keys(api);
			const factory = new Function(...names, `return function* run_routine() {\n${translate(src)}\n};`);
			return factory(...names.map((n) => api[n]))();
		},
		stop() { s.stopped = true; },
		// autonomous() epilogue: stop drive + mech_stop_all().
		finish() {
			stop();
			robot.intakeCmd = 0; robot.clawSpin = 0;
			robot.liftMode = 'volt'; robot.liftCmd = 0;
			robot.clawTarget = robot.clawDeg;
		},
	};
}
