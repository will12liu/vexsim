# Titanium Sim

3D physics simulator of team 4787T Titanium's VEX V5RC **Override** (2026–27) robot. Runs in the browser (three.js + Rapier), no build step.

Press **?** in the sim for the full controls.

## Running it (works offline)

Everything it needs (three.js, Rapier, the routine) is in this repo, so nothing loads from the internet.

- **One file, no internet, no setup:** `TitaniumSim.html` is the whole sim in one file. Copy it to a laptop and double-click it. (The menu has a download link for it.) Double-clicking `index.html` in a copied folder opens it too.
- **As an app:** open the website once, then install it. Chrome / Edge: the install icon in the address bar. Safari on Mac: File → Add to Dock. iPad / iPhone: Share → Add to Home Screen. It opens in its own window and keeps working with no internet; when you're online it picks up updates by itself.
- **Local server:** `python3 -m http.server 8765` in this folder, then open http://localhost:8765.

`TitaniumSim.html` is generated: `npm install && npm run build` (`tools/build-offline.mjs`). The `build-offline` workflow rebuilds it on every push to main, and the routine sync rebuilds it when `routine.cpp` changes. Don't edit it by hand.

## What it simulates

- **Modes:** Free Drive (D-pad ↓ runs a 15 s practice auton), Solo Match (15 s auton + 1:45 driver), 2v2 vs AI.
- **Drivetrain:** not a velocity setter. Each side is two V5 motors (torque curve, back-EMF, current limit, heat throttling) through gears with backlash, into wheels that can stick or slip. Omni wheels slide sideways, the robot coasts when you let go, and the two sides are slightly mismatched. With the cascade up the robot rocks, can wheelie, and tips if you slam the stick (the opcontrol anti-tip scale is ported). Smooth stick work is the whole point.
- **Robot (from the photos):** silver drive with red gears, cascade lift, flex-wheel claw on the back with the V-shaped guide below it. The rollers, guide arms and rubber feet are real colliders, so a pin knocked over into the guide stays there and blocks the claw ("CLAW BLOCKED") until you drive it out.
- **Ground pickups:** drive claw first into a standing cup + pin with R1 spinning, slowly (under about 6 in/s). Faster and the stack is knocked over.
- **Wall stacks:** the claw only takes a wall stack if you come in square through the gap behind the goal. Come in crooked and you knock a neighbour over or grab the wrong cup, then you have to clear the pins with the drivetrain.
- **Match loads:** 10 Cups + 10 Pins (+ yellow) per alliance, as in the game manual, loaded by the human player only during driver control. The Loader is a gravity tube with a slot: the cascade must be at R1 height to fit, back in square (within about 18°), slowly, with nothing stuck in the guide. A practice "unlimited" option is on the title menu.
- **AI robots (2v2):** Scorer, Stacker, Toggle attacker, Toggle defender, Pusher, or Random. They drive the same motor/omni physics as you, plan with A* plus pure pursuit, pick tasks by points per second using the real scoring, take match loads, fight over Toggles and park at the end. "Pro" skill is tuned to be harder than average.
- **Driver view:** matches the alliance-station photo (eye height, distance, pitch). You can turn your head and walk along the station like at a real match: drag the mouse (or a finger) to look, for example at your human player at the Loader; scroll or two-finger swipe to walk; double-click to look ahead again. Keys: J L / I K look, U O walk, H reset.
- **Input:** keyboard, Xbox / any gamepad, or a real V5 controller over USB (Chrome/Edge, see `bridge/`). Mapping follows `src/opcontrol.cpp`.

## Auton from the robot code

`routine.cpp` is a copy of `src/routine.cpp` from [will12liu/4787T-Titanium-Vex-Override](https://github.com/will12liu/4787T-Titanium-Vex-Override) (main). The sim translates the C++ (`js/cpp.js`) and runs `run_routine()` in the auton editor.

- **LemLib:** `chassis.moveToPoint / turnToHeading / moveToPose / turnToPoint` use the robot's PID gains and exit conditions. Odometry comes from simulated encoders + IMU, so wheel slip and gear slop make it drift like the real robot. The sim does not give perfect odometry.
- **auton_cmds:** `drive_time drive_to back_dist back_dist_score face_heading claw_grab claw_release lift_to_rot wait delay`, plus the mechanism functions from `mechanisms.h`.
- **Alliance side:** pick RED L / RED R / BLUE L / BLUE R with D-pad ← →. Blue swaps the path the same way `run_routine()` does on the robot (`if (is_blue()) use_right = !use_right;`, so BLUE R runs `LEFT_SIDE`).

Updating `routine.cpp`:

- **By hand:** `sh tools/sync-robot-code.sh` (uses a checkout next to this one, or clones the public repo), then commit and push.
- **Automatically:** `.github/workflows/sync-robot-code.yml` checks hourly. The robot repo is public, so no token is needed.

## Files

| File | What |
|---|---|
| `js/drivetrain.js` | Motor, gear slop, wheel friction and tip model |
| `js/robot.js` | Robot body, claw, guide, cascade, opcontrol |
| `js/robotmodel.js` | Robot 3D model |
| `js/field.js` | Field, goals, Toggles, Loaders, scoring |
| `js/human.js` | Human players and match-load counts |
| `js/auton.js` | Auton runtime, LemLib port, odometry |
| `js/cpp.js` | C++ to JS translator for `routine.cpp` |
| `js/ai.js` | AI robots |
| `js/main.js` | Game loop, cameras, HUD, menus |
| `vendor/` | three.js 0.160 (MIT) and Rapier 0.12 (Apache-2.0), kept here for offline use |
| `sw.js`, `manifest.webmanifest`, `icons/` | Installable offline app |
| `tools/build-offline.mjs` | Builds `TitaniumSim.html` |
