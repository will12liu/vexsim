#include "auton.h"
#include "auton_cmds.h"
#include "mechanisms.h"
#include "robot-config.h"
#include <cmath>

namespace {

// Distances are inches; headings are degrees clockwise from the starting direction.
constexpr int DRIVE_SPEED = 100;
constexpr int PAUSE_MS = 50;
constexpr int SCORE_ACTION_PAUSE_MS = 100;  // After intake and before the final release.

constexpr float SCORE_Y = -14.4f;

// Toggle: back up to the scoring Y, then drive into the wall at maximum power.
constexpr float TOGGLE_BACK_Y = SCORE_Y;
constexpr float WALL_Y = 0.7f;
constexpr int TOGGLE_FORWARD_POWER = 127;
constexpr int TOGGLE_EXTRA_PUSH_MS = 500;  // Keep pushing beyond the estimated wall arrival.
// Estimate from the original 6.1-inch return at power 100 for 900 ms.
constexpr int TOGGLE_HIT_MS = static_cast<int>(
	900.0f * (WALL_Y - TOGGLE_BACK_Y) / 6.1f * 100.0f / TOGGLE_FORWARD_POWER) + TOGGLE_EXTRA_PUSH_MS;
constexpr int SECOND_HIT_MIN_TIME_MS = 9000;

// Scoring: a more negative lift rotation means a higher lift.
constexpr double SCORE_LIFT_ROT = -300.0;
constexpr float SCORE_APPROACH_Y = -11.8f;
constexpr float SCORE_EXTRA_IN = 4.0f;
constexpr float SCORE_SLOWDOWN_IN = 8.0f;  // Ease off over the last 8 inches into either goal.
constexpr int SCORE_MIN_SPEED = 25;
constexpr int SCORE_DRIVE_TIMEOUT_MS = 4000;
constexpr int SCORE_SETTLE_MS = 200;
constexpr int FIRST_RELEASE_PAUSE_MS = 500;  // Let the claw release before pulling away.
constexpr double FIRST_SCORE_RETURN_LIFT_ROT = -70.0;
constexpr int CLAW_RELEASE_MS = 1500;
constexpr int CLAMP_DOWN_POWER = 60;
constexpr int CLAMP_PUSH_MS = 900;
constexpr int CLAMP_HOLD_POWER = 30;

// Leave the first approach move early so the turn starts while the robot is moving.
constexpr float APPROACH_MIN_SPEED = 50.0f;
constexpr float APPROACH_EARLY_EXIT_IN = 4.0f;

// Exit: complete the original reverse move, then reverse slowly while grabbing.
constexpr float EXIT_Y_CHANGE = -16.0f;
constexpr float EXIT_HEADING_ADJUST = 3.0f;
constexpr float EXIT_DISTANCE_ADJUST = -1.0f;
constexpr float EXIT_SLOW_BACK_IN = 11.75f;  // Extend the intake approach by 2 inches at slow speed.
constexpr int EXIT_SLOW_BACK_SPEED = 35;
constexpr int EXIT_SLOW_BACK_TIMEOUT_MS = 3000;

// After intake: raise first, turn and reverse, then lower onto the goal and release.
constexpr float SECOND_SCORE_TURN_DEG = 133.0f;
constexpr int SECOND_SCORE_TURN_TIMEOUT_MS = 2000;
constexpr float SECOND_SCORE_BACK_IN = 23.0f;
constexpr double SECOND_SCORE_LIFT_ROT = -450.0;
constexpr double SECOND_SCORE_RELEASE_ROT = -365.0;
constexpr int SECOND_SCORE_LIFT_WAIT_MS = 2000;

struct Side {
	float score_heading;
	float score_x;
	float pull_away_x;
	float exit_heading;
};

constexpr Side RIGHT_SIDE = {
	.score_heading = -87.1f,
	.score_x = 21.2f,
	.pull_away_x = 5.4f,
	.exit_heading = 316.3f
};
constexpr Side LEFT_SIDE = {
	.score_heading = 87.1f,
	.score_x = -21.2f,
	.pull_away_x = -5.4f,
	.exit_heading = 43.7f
};

void hit_toggle(float from_y) {
	back_dist(from_y - TOGGLE_BACK_Y, DRIVE_SPEED);
	wait(PAUSE_MS);
	drive_time(TOGGLE_FORWARD_POWER, TOGGLE_HIT_MS);
	set_pose(0, WALL_Y, 0);
}

void approach_goal(const Side& side) {
	if (!auton_odom_ready() || auton_stopped()) return;

	// Reverse along y, turn the claw toward the goal, then reverse along x.
	chassis.moveToPoint(chassis.getPose().x, SCORE_APPROACH_Y, DRIVE_TIMEOUT_MS,
	                    {.forwards = false, .maxSpeed = DRIVE_SPEED,
	                     .minSpeed = APPROACH_MIN_SPEED, .earlyExitRange = APPROACH_EARLY_EXIT_IN}, false);
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

bool wait_for_lift(double target) {
	char step[24];
	snprintf(step, sizeof(step), "lift to %.0f", target);
	auton_status(step);
	const uint32_t start = pros::millis();
	while (std::fabs(lift_pos() - target) > LIFT_RECORRECT_DEG) {
		if (auton_stopped()) return false;
		if (pros::millis() - start >= SECOND_SCORE_LIFT_WAIT_MS) {
			auton_status("lift wait timeout");
			return false;
		}
		pros::delay(10);
	}
	return !auton_stopped();
}

} // namespace

void run_routine() {
	// Keep the current field setup: blue swaps the selected left/right path.
	bool use_right = on_right();
	if (is_blue()) use_right = !use_right;
	const Side& side = use_right ? RIGHT_SIDE : LEFT_SIDE;
	// Mirror relative turns as well as the field coordinates and base headings.
	// const float turn_mirror = use_right ? 1.0f : -1.0f;

	set_pose(0, 0, 0);
	lift_to_rot(SCORE_LIFT_ROT);  // Raise the lift while driving.

	// Current sequence: bump twice, finish backing away, then turn toward the goal.
	hit_toggle(0);
	if (auton_stopped()) return;
	hit_toggle(WALL_Y);
	if (!auton_odom_ready() || auton_stopped()) return;
	back_dist(chassis.getPose().y - SCORE_APPROACH_Y, DRIVE_SPEED);
	if (auton_stopped()) return;
	wait(PAUSE_MS);
	face_heading(side.score_heading, DRIVE_SPEED);

	// Original sequence preserved below:
	// // 1. Hit the toggle. Repeat only if enough time remains to score.
	// hit_toggle(0);
	// if (auton_time_left() > SECOND_HIT_MIN_TIME_MS) {
	// 	wait(PAUSE_MS);
	// 	hit_toggle(WALL_Y);
	// }
	// wait(PAUSE_MS);
	//
	// // 2. Approach the goal, push the lift down, and release the pin.
	// approach_goal(side);
	// wait(SCORE_SETTLE_MS);

	// Finish the first scoring approach after the backward move and turn above.
	wait(PAUSE_MS);
	if (auton_stopped()) return;
	const float score_distance = std::fabs(side.score_x - chassis.getPose().x) + SCORE_EXTRA_IN;
	back_dist_score(score_distance, DRIVE_SPEED, SCORE_DRIVE_TIMEOUT_MS, SCORE_SLOWDOWN_IN, SCORE_MIN_SPEED);
	wait(SCORE_SETTLE_MS);
	if (!clamp_down()) return;
	claw_release(0);  // One R2 press; keep opening while pulling away.
	cascade_move(0);
	wait(FIRST_RELEASE_PAUSE_MS);

	// 3. Pull away, turn, and complete the original backward drive.
	// Start moving away without an automatic alignment turn at the goal.
	drive_to(side.pull_away_x, SCORE_Y, DRIVE_SPEED, DRIVE_TIMEOUT_MS, false);
	lift_to_rot(FIRST_SCORE_RETURN_LIFT_ROT);  // Return to intake height once clear of the first goal.
	if (!wait_for_lift(FIRST_SCORE_RETURN_LIFT_ROT)) return;
	claw_grab(0);  // Start intake only after reaching -70; keep grabbing until the second release.
	wait(PAUSE_MS);
	if (use_right) {
		face_heading(side.exit_heading + EXIT_HEADING_ADJUST, DRIVE_SPEED);
	} else {
		if (!auton_odom_ready() || auton_stopped()) return;
		auton_status("exit turn left");
		// Force the front to turn left after pulling away from the first goal.
		chassis.turnToHeading(side.exit_heading - EXIT_HEADING_ADJUST, TURN_TIMEOUT_MS,
		                      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
		                       .maxSpeed = DRIVE_SPEED}, false);
	}
	wait(PAUSE_MS);
	const float exit_radians = (side.exit_heading + 180.0f) * 3.14159265f / 180.0f;
	const float exit_distance = EXIT_Y_CHANGE / std::cos(exit_radians) + EXIT_DISTANCE_ADJUST;
	back_dist(exit_distance, DRIVE_SPEED);

	/* Temporarily disabled: slow intake and the remainder of the routine.
	// 4. Keep R1 running during the slow intake and second scoring approach.
	back_dist(EXIT_SLOW_BACK_IN, EXIT_SLOW_BACK_SPEED, EXIT_SLOW_BACK_TIMEOUT_MS);

	// 5. Raise the pin before turning toward the second score.
	if (!auton_odom_ready() || auton_stopped()) return;
	lift_to_rot(SECOND_SCORE_LIFT_ROT);
	if (!wait_for_lift(SECOND_SCORE_LIFT_ROT)) return;
	wait(SCORE_ACTION_PAUSE_MS);
	face_heading(chassis.getPose().theta - turn_mirror * SECOND_SCORE_TURN_DEG,
	             DRIVE_SPEED, SECOND_SCORE_TURN_TIMEOUT_MS);
	wait(PAUSE_MS);
	if (auton_stopped()) return;

	// Reverse with the pin raised, slowing as the robot reaches the goal.
	const auto approach_result = back_dist_score(SECOND_SCORE_BACK_IN, DRIVE_SPEED, SCORE_DRIVE_TIMEOUT_MS,
	                                            SCORE_SLOWDOWN_IN, SCORE_MIN_SPEED);
	if (approach_result == ScoreApproachResult::FAILED) return;

	// At the target or stopped near it: lower onto the goal before releasing.
	lift_to_rot(SECOND_SCORE_RELEASE_ROT);
	if (!wait_for_lift(SECOND_SCORE_RELEASE_ROT)) return;
	wait(SCORE_ACTION_PAUSE_MS);
	claw_release(0);  // Keep opening until autonomous cleanup stops the mechanisms.
	wait(CLAW_RELEASE_MS);
	*/
}
