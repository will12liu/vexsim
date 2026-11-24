// Robots you can drive, picked on the title menu. To add one: write a class extending Robot
// (robot.js) with its colliders, mesh, mechanisms and opcontrol, like dr4bbot.js, and add it here.
import { Robot, CFG } from './robot.js';
import { IntakeBot } from './intakebot.js';
import { Dr4bBot, DCFG } from './dr4bbot.js';
import { IN, PIN_H, CUP_H, toField } from './field.js';

// Other teams' robots, from their public Override reveals. shortcut: built on our robot classes
// (same frames and mechanisms) with their gearing, lift speed and a different frame colour, not
// their CAD; give one its own model if it becomes a regular sparring partner.
function variant(Base, cfg, metal) {
	return class extends Base {
		get C() { return cfg || super.C; }
		buildMesh() {
			const g = super.buildMesh(), done = new Map();
			g.traverse((m) => {   // repaint the grey metal frame (materials are shared, so swap in copies)
				const mat = m.material;
				if (!mat || Array.isArray(mat) || !(mat.metalness >= 0.5) || !mat.color) return;
				const { r, g: gg, b } = mat.color;
				if (Math.max(r, gg, b) - Math.min(r, gg, b) > 0.08) return;   // coloured parts stay
				if (!done.has(mat)) { const c = mat.clone(); c.color.setHex(metal); done.set(mat, c); }
				m.material = done.get(mat);
			});
			return g;
		}
	};
}

// 40000A Apogee: one-button autostack. X turns it on; while on, the lift rises so the held stack
// clears whatever goal stack the claw is backing toward, and L2 lowers it on (which lets go).
class ApogeeBot extends variant(Robot, null, 0x2e7d4f) {
	static controlRows(k, x) {
		return [...super.controlRows(k, x), ['Autostack on / off', k('X'), k(x.X), k('X'), 'Holding a stack, back toward a goal: the lift rises to just clear it by itself. L1 / L2 take over; lower onto the goal to score']];
	}
	opcontrol(c, hooks = {}) {
		const press = c.X && !this.op.prev.X && !c.A;
		super.opcontrol(c, hooks);
		if (press) { this.autoStack = !this.autoStack; this.flash?.(this.autoStack ? 'AUTOSTACK ON' : 'AUTOSTACK OFF'); }
		if (!this.autoStack || !this.claw || c.L1 || c.L2 || this.tipped()) return;
		const o = this.clawBottom(), spot = toField(this.toWorld(this.C.clawX - 3, 0, 0));   // a little behind the claw
		const s = this.field.seatFor(o, 6, spot);
		if (!s) return;
		const half = (k) => (k.kind === 'cup' ? CUP_H : PIN_H) / 2;
		const bottom = o.body.translation().y / IN - half(o);
		const top = s.top ? toField(s.top.body.translation()).h + half(s.top) : s.g.H;   // highest point to clear
		const want = Math.min(this.C.liftMaxIn, this.liftIn() + top + 1 - bottom);
		if (want > this.liftIn() + 0.3) { this.liftTo(-this.liftDegFor(want)); this.op.preset = true; }
	}
}

export const ROBOTS = {
	dr4b: { name: 'DR4B (new robot)', sub: 'From the Onshape CAD: 6-wheel drive, DR4B lift with the flex-wheel claw on the back, pneumatic pincer on the front. No robot code yet', cls: Dr4bBot },
	titanium: { name: '4787T Titanium', sub: 'Flex-wheel claw on the back, cascade, Toggle arms with rubber pucks', cls: Robot },
	intake: { name: 'Ground intake (first build)', sub: 'Sept to Oct 2026: front intake into a hopper, flip claw on the back, cascade', cls: IntakeBot },
	'40000a': { name: '40000A Apogee', sub: 'Great Planes 2026 reveal: claw bot with one-button autostack. X turns it on; the lift rises to clear the goal you back toward. Our claw bot\'s frame', cls: ApogeeBot },
	'8059a': { name: '8059A Blank.', sub: 'Early-season reveal: standoff-linkage DR4B, motors at the bottom so the lift is light and fast. Our DR4B\'s frame with a faster lift', cls: variant(Dr4bBot, { ...DCFG, liftDegPerSec: 240 }, 0x34373c) },
	'8059y': { name: '8059Y Blank.', sub: 'Early-season reveal: DR4B with a rubber-band intake on a pivoting chamber. Approximated with our ground-intake build', cls: variant(IntakeBot, null, 0x2f5fb8) },
	'8059s': { name: '8059S Blank.', sub: 'First-year team\'s starter bot: drivetrain, 6-bar and a Toggle hitter. Our claw bot\'s frame, slower green-motor drive', cls: variant(Robot, { ...CFG, driveRpm: 300, liftDegPerSec: 800 }, 0x9b3a2f) },
};
export const DEFAULT_ROBOT = 'dr4b';
