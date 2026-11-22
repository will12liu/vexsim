// Robots you can drive, picked on the title menu. To add one: write a class extending Robot
// (robot.js) with its colliders, mesh, mechanisms and opcontrol, like dr4bbot.js, and add it here.
import { Robot } from './robot.js';
import { IntakeBot } from './intakebot.js';
import { Dr4bBot } from './dr4bbot.js';

export const ROBOTS = {
	dr4b: { name: 'DR4B (new robot)', sub: 'From the Onshape CAD: 6-wheel drive, DR4B lift with the flex-wheel claw on the back, pneumatic pincer on the front. No robot code yet', cls: Dr4bBot },
	titanium: { name: '4787T Titanium', sub: 'Flex-wheel claw on the back, cascade, Toggle arms with rubber pucks', cls: Robot },
	intake: { name: 'Ground intake (first build)', sub: 'Sept to Oct 2026: front intake into a hopper, flip claw on the back, cascade', cls: IntakeBot },
};
export const DEFAULT_ROBOT = 'dr4b';
