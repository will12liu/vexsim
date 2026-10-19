# Titanium Sim

3D physics simulator of team Titanium's VEX V5RC **Override** (2026–27) robot. Runs in the browser (three.js + Rapier), no build step.

- **Modes:** Free Drive, Solo Match (15 s auton + 1:45 driver), 2v2 vs AI
- **Input:** keyboard, Xbox / any gamepad, or a real V5 controller over USB (Chrome/Edge, see `bridge/`)
- **Robot (10/19):** flex-wheel claw on the back that grabs a cup + pin stack (drive claw first into it, or R1 grab spin; R2 / A lets go), cascade lift, no intake. The human player keeps a stack standing at each of your corner Loaders.
- Press **?** in the sim for the full controls

Run locally: `python3 -m http.server 8765` in this folder, then open http://localhost:8765.

## Auton routine from the robot code

`routine.cpp` is a copy of `src/routine.cpp` from [will12liu/titaniumvex](https://github.com/will12liu/titaniumvex) (main). The sim loads its `run_routine()` into the auton editor. To update it:

- **By hand:** `sh tools/sync-robot-code.sh` (needs the robot repo checked out next to this one), then commit and push.
- **Automatically:** `.github/workflows/sync-robot-code.yml` checks hourly, once a `TITANIUMVEX_TOKEN` secret (fine-grained token, read-only Contents on titaniumvex) is added to this repo.

Supported commands: `move turn turn_to drive_time wait move_pid turn_pid lift_goal lift_down lift_wait lift_secs claw_drop spin_claw score mirror on_left on_right is_red is_blue`, plus the LemLib ones `set_pose drive_to back_to back_dist back_past face face_heading drive_to_pose back_to_pose` (the sim gives them perfect odometry). The old `claw_in claw_out claw_up intake` still run but do nothing (no claw flip or intake on the robot now).
