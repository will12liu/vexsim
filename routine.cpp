#include "auton.h"
#include "auton_cmds.h"
#include "mechanisms.h"
#include "robot-config.h"
#include <cmath>

namespace {

// Distances are inches; headings are degrees clockwise from the starting direction.
constexpr int DRIVE_SPEED = AUTON_DRIVE_POWER;
constexpr int PAUSE_MS = 50;
constexpr int SCORE_ACTION_PAUSE_MS = 100;  // After raising the pin, before the second scoring turn.
constexpr float SCORE_APPROACH_Y = -13.3f;

// Toggle: reverse to this y, drive into the wall, then correct the robot's known position.
constexpr float TOGGLE_BACK_Y = -6.4f;
constexpr float SECOND_TOGGLE_BACK_Y = TOGGLE_BACK_Y - 4.0f;
constexpr float WALL_Y = 0.7f;
constexpr int TOGGLE_HIT_POWER = 127;
constexpr int TOGGLE_HIT_MS = 900;
constexpr int SECOND_TOGGLE_HIT_MS = 900;
constexpr int SECOND_HIT_MIN_TIME_MS = 9000;

// Scoring: a more negative lift rotation means a higher lift.
constexpr double SCORE_LIFT_ROT = -250.0;
constexpr float SCORE_Y = -14.4f;
constexpr float SCORE_EXTRA_IN = 4.0f;
constexpr float SCORE_SLOWDOWN_IN = 8.0f;  // Ease off over the last 8 inches into either goal.
constexpr int SCORE_MIN_SPEED = 25;
constexpr int SCORE_DRIVE_TIMEOUT_MS = auton_move_time(4000);
constexpr int SCORE_SETTLE_MS = 200;
constexpr int SCORE_RELEASE_PAUSE_MS = 500;  // Let the pin clear before pulling away after either score.
constexpr int CLAW_RELEASE_MS = 1500;
constexpr int CLAMP_DOWN_POWER = 60;
constexpr int CLAMP_PUSH_MS = 900;
constexpr int CLAMP_HOLD_POWER = 30;

// Exit: pull forward from the goal, turn 60 degrees, then intake backward toward -y.
constexpr int EXIT_LOWER_TIMEOUT_MS = 4000;
constexpr float EXIT_TURN_DEG = 60.0f;
constexpr int EXIT_TURN_TIMEOUT_MS = auton_move_time(2000);
constexpr float EXIT_Y_CHANGE = -16.0f;
constexpr float EXIT_DISTANCE_ADJUST = 1.0f;
constexpr float EXIT_SLOW_BACK_IN = 12.75f;
constexpr int EXIT_SLOW_BACK_SPEED = 35;
constexpr int EXIT_SLOW_BACK_TIMEOUT_MS = 3000;

// After intake: raise, drive backward, apply the side's heading change, then reverse to score.
constexpr float SECOND_SCORE_PRE_TURN_BACK_IN = 3.5f;
constexpr int SECOND_SCORE_TURN_TIMEOUT_MS = auton_move_time(2000);
constexpr float SECOND_SCORE_BACK_IN = 26.0f;
constexpr double SECOND_SCORE_LIFT_ROT = -550.0;
constexpr int SECOND_SCORE_SETTLE_MS = 200;
constexpr double SECOND_SCORE_RELEASE_ROT = -250.0;
constexpr int SECOND_SCORE_RELEASE_WAIT_MS = 250;
constexpr int LIFT_WAIT_TIMEOUT_MS = 2000;

// After the final release: lift clear, move away, lower fully, then continue forward.
constexpr double AFTER_SCORE_RAISE_DEG = 60.0;
constexpr int AFTER_SCORE_LOWER_TIMEOUT_MS = 4000;
constexpr float AFTER_SCORE_CLEARANCE_IN = 7.0f;
constexpr float AFTER_SCORE_FORWARD_IN = 41.0f;
constexpr int AFTER_SCORE_DRIVE_TIMEOUT_MS = auton_move_time(6000);

struct Side {
	float score_heading;
	float score_x;
	float pull_away_x;
	float second_score_turn_deg;
};

constexpr Side RIGHT_SIDE = {
	.score_heading = -87.1f,
	.score_x = 21.2f,
	.pull_away_x = 3.4f,  // Pull 2 inches farther from the first goal before turning.
	.second_score_turn_deg = -135.0f  // Red right / blue left: -45 -> -180 is a -135-degree change.
};
constexpr Side LEFT_SIDE = {
	.score_heading = -RIGHT_SIDE.score_heading,
	.score_x = -RIGHT_SIDE.score_x,
	.pull_away_x = -RIGHT_SIDE.pull_away_x,
	.second_score_turn_deg = 130.0f  // Red left / blue right: 70 -> 200 is a +130-degree change.
};

void hit_toggle(float from_y, float back_y, int hit_ms) {
	back_dist(from_y - back_y, DRIVE_SPEED);
	drive_time(TOGGLE_HIT_POWER, hit_ms);
	set_pose(0, WALL_Y, 0);
}

void approach_goal(const Side& side) {
	if (!auton_odom_ready() || auton_stopped()) return;

	// Reverse farther than the toggle backup before turning toward the first goal.
	auton_status("goal approach");
	chassis.moveToPoint(chassis.getPose().x, SCORE_APPROACH_Y, DRIVE_TIMEOUT_MS,
	                    {.forwards = false, .maxSpeed = DRIVE_SPEED}, false);
	printf("DBG goal approach end target_y=%.1f actual_y=%.1f\n", SCORE_APPROACH_Y, chassis.getPose().y);
	if (auton_stopped()) return;
	face_heading(side.score_heading, DRIVE_SPEED);
	wait(PAUSE_MS);
	if (auton_stopped()) return;
	const float distance = std::fabs(side.score_x - chassis.getPose().x) + SCORE_EXTRA_IN;
	back_dist_score(distance, DRIVE_SPEED, SCORE_DRIVE_TIMEOUT_MS, SCORE_SLOWDOWN_IN, SCORE_MIN_SPEED);
}

bool clamp_down() {
	auton_status("clamp_down");
	lift_release();  // Give manual commands control of the lift.
	const uint32_t start = pros::millis();
	while (pros::millis() - start < CLAMP_PUSH_MS && !auton_stopped()) {
		cascade_move(-CLAMP_DOWN_POWER);  // Recheck the travel limits every 10 ms.
		pros::delay(10);
	}
	if (auton_stopped()) {
		cascade_move(0);
		return false;
	}
	cascade_move(-CLAMP_HOLD_POWER);
	return true;
}

bool wait_for_lift(double target, int timeout_ms = LIFT_WAIT_TIMEOUT_MS) {
	char step[24];
	snprintf(step, sizeof(step), "lift to %.0f", target);
	auton_status(step);
	const uint32_t start = pros::millis();
	// The controller can hold anywhere inside its restart band after settling.
	while (std::fabs(lift_pos() - target) > LIFT_RECORRECT_DEG) {
		if (auton_stopped()) return false;
		if (pros::millis() - start >= (uint32_t)timeout_ms) {
			printf("DBG lift timeout target=%.1f actual=%.1f limit=%dms\n", target, lift_pos(), timeout_ms);
			auton_fail("lift timeout");
			return false;
		}
		pros::delay(10);
	}
	return !auton_stopped();
}

bool wait_for_bottom(uint32_t lower_start_ms) {
	auton_status("lift to bottom");
	while (lift_pos() < LIFT_LIMIT_BOTTOM_DEG) {
		if (auton_stopped()) return false;
		if (pros::millis() - lower_start_ms >= EXIT_LOWER_TIMEOUT_MS) {
			printf("DBG lift bottom timeout actual=%.1f limit=%dms\n", lift_pos(), EXIT_LOWER_TIMEOUT_MS);
			auton_fail("lift timeout");
			return false;
		}
		pros::delay(10);
	}
	lift_release();
	cascade_move(0);
	return !auton_stopped();
}

} // namespace

void run_routine() {
	claw_grab(0);  // Start grabbing immediately and keep running until the first release.

	// Red right / blue left start on +x; red left / blue right mirror their path.
	bool use_right = on_right();
	if (is_blue()) use_right = !use_right;
	const Side& side = use_right ? RIGHT_SIDE : LEFT_SIDE;
	const float turn_sign = use_right ? 1.0f : -1.0f;

	set_pose(0, 0, 0);
	lift_to_rot(SCORE_LIFT_ROT);  // Raise the lift while driving.

	// 1. Hit the toggle. Repeat only if enough time remains to score.
	hit_toggle(0, TOGGLE_BACK_Y, TOGGLE_HIT_MS);
	if (auton_time_left() > SECOND_HIT_MIN_TIME_MS) {
		hit_toggle(WALL_Y, SECOND_TOGGLE_BACK_Y, SECOND_TOGGLE_HIT_MS);
	}

	// 2. Approach the goal, push the lift down, and release the pin.
	approach_goal(side);
	wait(SCORE_SETTLE_MS);
	if (!clamp_down()) return;
	claw_release(0, CLAW_RELEASE_POWER);  // Keep opening through the pull-away and intake turn.
	cascade_move(0);
	wait(SCORE_RELEASE_PAUSE_MS);

	// 3. Pull forward from the goal, then turn right/left 60 degrees for intake.
	// Start moving away without an automatic alignment turn at the goal.
	drive_to(side.pull_away_x, SCORE_Y, DRIVE_SPEED, DRIVE_TIMEOUT_MS, false);
	lift_to_rot(FIRST_SCORE_RETURN_LIFT_ROT);  // Return to intake height once clear of the first goal.
	if (!wait_for_lift(FIRST_SCORE_RETURN_LIFT_ROT)) return;
	claw_grab(0);  // Start intake only after reaching -70; keep grabbing until the second release.
	wait(PAUSE_MS);
	if (auton_stopped()) return;
	if (!auton_odom_ready()) {
		auton_fail("NO ODOM");
		return;
	}
	const float exit_heading = chassis.getPose().theta + turn_sign * EXIT_TURN_DEG;
	face_heading(exit_heading, DRIVE_SPEED, EXIT_TURN_TIMEOUT_MS,
	             use_right ? TurnDirection::RIGHT : TurnDirection::LEFT);
	claw_grab(0);  // Close after the intake turn; keep grabbing until the second release.
	// Reverse travel follows (-sin heading, -cos heading); both mirrors move toward -y.
	const float exit_radians = exit_heading * 3.14159265f / 180.0f;
	const float exit_distance = EXIT_Y_CHANGE / -std::cos(exit_radians) + EXIT_DISTANCE_ADJUST;
	// Aim beyond the bottom so the controller's hold band cannot stop short.
	// cascade_move still enforces the configured bottom limit on every update.
	const uint32_t lower_start_ms = pros::millis();
	lift_to_rot(LIFT_LIMIT_BOTTOM_DEG + 2 * LIFT_RECORRECT_DEG);
	back_dist(exit_distance, DRIVE_SPEED);
	if (!wait_for_bottom(lower_start_ms)) return;

	/* Temporarily disabled: slow intake and the remainder of the routine.
	// 4. Keep R1 running during the slow intake and second scoring approach.
	back_dist(EXIT_SLOW_BACK_IN, EXIT_SLOW_BACK_SPEED, EXIT_SLOW_BACK_TIMEOUT_MS);

	// 5. Raise the pin before turning toward the second score.
	if (auton_stopped()) return;
	if (!auton_odom_ready()) {
		auton_fail("NO ODOM");
		return;
	}
	lift_to_rot(SECOND_SCORE_LIFT_ROT);
	if (!wait_for_lift(SECOND_SCORE_LIFT_ROT)) return;
	wait(SCORE_ACTION_PAUSE_MS);
	back_dist(SECOND_SCORE_PRE_TURN_BACK_IN, DRIVE_SPEED, DRIVE_TIMEOUT_MS);
	if (auton_stopped()) return;
	// Use the measured heading plus the example's difference, not a fixed 200/-180 target.
	face_heading(chassis.getPose().theta + side.second_score_turn_deg,
	             DRIVE_SPEED, SECOND_SCORE_TURN_TIMEOUT_MS,
	             side.second_score_turn_deg > 0 ? TurnDirection::RIGHT : TurnDirection::LEFT);
	if (auton_stopped()) return;

	// Reverse with the pin raised, slowing as the robot reaches the goal.
	const auto approach_result = back_dist_score(SECOND_SCORE_BACK_IN, DRIVE_SPEED, SCORE_DRIVE_TIMEOUT_MS,
	                                            SCORE_SLOWDOWN_IN, SCORE_MIN_SPEED);
	if (approach_result == ScoreApproachResult::FAILED) {
		if (!auton_stopped()) auton_fail("reverse timeout");
		return;
	}

	// At the target or stopped near it: settle, lower the cascade, then release.
	wait(SECOND_SCORE_SETTLE_MS);
	lift_to_rot(SECOND_SCORE_RELEASE_ROT);
	wait(SECOND_SCORE_RELEASE_WAIT_MS);
	claw_release(CLAW_RELEASE_MS, CLAW_RELEASE_POWER);
	wait(SCORE_RELEASE_PAUSE_MS);

	// 6. Lift clear, move forward 7 inches, lower fully, then move forward 41 inches.
	if (auton_stopped()) return;
	const double raised_target = lift_pos() - AFTER_SCORE_RAISE_DEG;
	lift_to_rot(raised_target);
	if (!wait_for_lift(raised_target)) return;
	if (!forward_dist(AFTER_SCORE_CLEARANCE_IN, DRIVE_SPEED, AFTER_SCORE_DRIVE_TIMEOUT_MS)) return;
	lift_to_rot(LIFT_LIMIT_BOTTOM_DEG);
	if (!wait_for_lift(LIFT_LIMIT_BOTTOM_DEG, AFTER_SCORE_LOWER_TIMEOUT_MS)) return;
	forward_dist(AFTER_SCORE_FORWARD_IN, DRIVE_SPEED, AFTER_SCORE_DRIVE_TIMEOUT_MS);
}
