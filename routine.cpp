
#include "main.h"
#include "auton_cmds.h"
#include <cmath>

void run_routine() {
	// 1. Hit the Toggle twice 
	back_to(0.0, -8.4,100);      // back up to y = -8.4 (no settle: the hit below lines it up)
	drive_time(127, 900);//forward
	set_pose(0, 0.7, 0);   
	wait(50); //wait
	back_to(0.0, -8.4, 100);      // back up to y = -8.4
	drive_time(127, 900);        // hit 2: drive in
	set_pose(0, 0.7, 0);         // pressed flat on the wall: exact spot (measured 10/13), wipes out wheel slip from the hits
	wait(50);
// 2. score pin 
	back_to(0, -16.8, 100);                // 1) straight back off the Toggle to the score spot's y
	face_heading(mirror(-87.1), 100);      // 2) turn so the BACK (claw side) faces our goal
	back_to(mirror(21.2), -17.4, 100);     // 3) straight back to the score spot's x
	claw_drop();    
	wait(50);                       // claw spin (A button) to drop the pin
	lift_secs(300,true);
	const float FACE_H = 316.3f;
    // y the move must reach (-39.9 overshot by ~14" along the line)
	const float back_dir = (FACE_H + 180.0f) * 3.14159265f / 180.0f;
	const float run = (-16.0) / std::cos(back_dir);          // distance along that heading to reach END_Y
	drive_to(mirror(5.4), -17.4, 100);              // 1) pull away from the goal to x = 5.4
	lift_secs(300,false);
	face_heading(mirror(FACE_H), 100);              // 2) turn
	back_dist(run, 100);                            // 3) straight line down that heading, IMU-held so it can't curve
	spin_claw(100);
	spin_claw(-127);                                // Y button: claw spin OUT (stays on until set to 0)
}
