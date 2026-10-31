// Robots you can drive, picked on the title menu. To add one (e.g. the new robot once it's built):
// write a class extending Robot (robot.js) with its colliders, mesh, mechanisms and opcontrol, like
// intakebot.js, and add it here.
import { Robot } from './robot.js';
import { IntakeBot } from './intakebot.js';

export const ROBOTS = {
	titanium: { name: '4787T Titanium', sub: 'Current robot: flex-wheel claw on the back, cascade, Toggle arms with rubber pucks', cls: Robot },
	intake: { name: 'Ground intake (first build)', sub: 'Sept to Oct 2026: front intake into a hopper, flip claw on the back, cascade', cls: IntakeBot },
};
export const DEFAULT_ROBOT = 'titanium';
