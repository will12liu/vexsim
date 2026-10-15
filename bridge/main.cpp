// V5 controller -> simulator bridge. Upload this as its OWN PROS project in a spare
// program slot (not your competition code). It never drives a motor; it only prints the
// controller state over the Brain's USB port 50 times a second for the simulator to read.
//
//   pros conductor new-project vexsim-bridge
//   copy this file over vexsim-bridge/src/main.cpp
//   pros mu --slot 8
//
// Then: Brain USB -> computer, controller paired to the Brain as normal, run the program,
// and press "Connect V5" in the simulator (Chrome or Edge; Safari has no Web Serial).
#include "main.h"
#include "pros/apix.h"   // serctl()

void initialize() {}
void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
	// Plain text instead of PROS's framed (COBS) output, so the browser can read it directly.
	pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	const pros::controller_digital_e_t buttons[] = {
		pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2,
		pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2,
		pros::E_CONTROLLER_DIGITAL_A, pros::E_CONTROLLER_DIGITAL_B,
		pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_Y,
		pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN,
		pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT,
	};
	pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Sim bridge running");
	while (true) {
		int bits = 0;
		for (int i = 0; i < 12; i++) bits |= (master.get_digital(buttons[i]) ? 1 : 0) << i;
		// "V5 <leftY> <rightX> <leftX> <rightY> <button bits>"
		printf("V5 %d %d %d %d %d\n",
		       master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X),
		       master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), bits);
		fflush(stdout);
		pros::delay(20);
	}
}
