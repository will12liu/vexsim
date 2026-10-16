
#include "main.h"
#include "auton_cmds.h"

// =============================================================================
//                       WRITE YOUR AUTON ROUTINE HERE
//
// Commands run top to bottom, one after another. See include/auton_cmds.h for
// the full list. The main ones:
//
//   move(inches, speed)          drive straight, negative = backwards
//   turn(degrees, speed)         turn from where you face now, + = right, - = left
//   turn_to(degrees, speed)      turn to a heading measured from the auton start
//   drive_time(speed, ms)        drive straight for some time (for pushing things)
//   wait(ms)                     pause
//   move_pid(kP, kD)             change the drive PID for every move() after this
//   turn_pid(kP, kD)             change the turn PID for every turn() after this
//   lift_goal()  lift_down()  lift_wait()                (lift moves don't wait; add lift_wait())
//   claw_in()  claw_out()  claw_up()  claw_drop()          (claw: vertical / horizontal / slightly up / spin to drop)
//   intake(speed)  spin_claw(speed)
//   claw_out_start()             start flipping the claw out, doesn't wait (flip while driving)
//   claw_drop_up()               drop the pin and lift the claw up at the same time
//   score()   = lift_goal, lift_wait, claw_out, claw_drop, claw_up in one go
//
// LemLib (go to a spot on the field, see include/auton_cmds.h for the coordinates):
//   drive_to(x, y, speed)   back_to(x, y, speed)     drive there front / back first
//   face(x, y)   face_heading(degrees)               turn in place
//   drive_to_pose(x, y, heading)   back_to_pose(...) curve in, arrive facing heading
//   set_pose(x, y, heading)                           "you are here" (e.g. against a wall)
//
// Speed is 0-127 (for drive_time it also sets the direction: negative = backwards).
// You can also set the PID for a single command: move(24, 100, 6.0, 0.3)
//
// LEFT / RIGHT: write the LEFT version. Wrap turns in mirror() and they flip
// automatically for RIGHT:   turn(mirror(-85));
// Or check on_left() / on_right() / is_red() / is_blue() yourself.
//
// START: robot centred on the Toggle, FRONT facing the Toggle wall, a few inches
// from it. Left / right = which side our goal is on, standing at the Toggle wall
// with your back to it looking out into the field.
// =============================================================================

void run_routine() {
	// 1. Hit the Toggle twice 
	back_to(0, -8.4, 60);        // back up to y = -8.4
	drive_time(127, 900);//forward
	set_pose(0, 0.7, 0);   
	wait(150); //wait
	back_to(0, -8.4, 60);        // back up to y = -8.4
	drive_time(127, 950);        // hit 2: drive in
	set_pose(0, 0.7, 0);         // pressed flat on the wall: exact spot (measured 10/13), wipes out wheel slip from the hits
	wait(150);

	//2. score preloaded pin (LemLib). Spots measured on the TUNE screen 10/13, LEFT side:
	//   score spot (19.2, -12.9, -87.1)
	back_to(0, -17.4, 60);                 // 1) straight back off the Toggle to the score spot's y
	face_heading(mirror(-87.1), 100);      // 2) turn so the BACK (claw side) faces our goal
	claw_out();                            // claw vertical -> horizontal
	back_to(mirror(21.2), -17.4, 60);        // 3) straight back to the score spot's x
	claw_drop();                           // claw spin (A button) to drop the pin

	//3. claw up while driving to the next spot (-16.9, -35.8, -130.1)
	//claw_up_start();                       // B once: horizontal -> slanted, lifts while we drive
	//face_heading(mirror(-120.1), 100);     // turn to the angle first
	//drive_to(mirror(-16.9), -35.8, 100);   // then drive front first to the spot
}
