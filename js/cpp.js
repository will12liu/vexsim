// Turns the robot's routine.cpp (C++) into a JS generator the sim can step one physics tick at a time.
// Handles the subset our routines use: constexpr constants, structs with designated initializers,
// helper functions (void / bool / int / float / double / enum return types), LemLib calls with
// {.option = value} parameter structs, casts, std:: maths, enums (A::B::C -> "C"), snprintf.
// Every helper function becomes a generator, and every call to one (or to a blocking command)
// becomes (yield* f(...)), so a C++ call that blocks the robot blocks the routine here too.

const KEYWORDS = new Set(['if', 'else', 'while', 'for', 'switch', 'return', 'do', 'case', 'sizeof', 'catch']);
const TYPE = String.raw`(?:static\s+|inline\s+|constexpr\s+|const\s+|volatile\s+|unsigned\s+|signed\s+)*(?:[A-Za-z_][\w]*(?:::[A-Za-z_]\w*)*)(?:\s*<[^<>;(){}]*>)?\s*(?:const\s*)?[&*]*`;

function stripComments(src) {
	let out = '', i = 0;
	while (i < src.length) {
		const c = src[i], n = src[i + 1];
		if (c === '"' || c === "'") {   // copy string / char literals untouched
			let j = i + 1;
			while (j < src.length && src[j] !== c) j += src[j] === '\\' ? 2 : 1;
			out += src.slice(i, j + 1); i = j + 1;
		} else if (c === '/' && n === '/') { while (i < src.length && src[i] !== '\n') i++; }
		else if (c === '/' && n === '*') { const j = src.indexOf('*/', i + 2); out += ' '; i = j < 0 ? src.length : j + 2; }
		else { out += c; i++; }
	}
	return out;
}

// Index of the bracket matching the one at `open` (same kind), skipping strings.
function matchBracket(s, open) {
	const o = s[open], c = { '(': ')', '{': '}', '[': ']' }[o];
	let d = 0;
	for (let i = open; i < s.length; i++) {
		const ch = s[i];
		if (ch === '"' || ch === "'") { let j = i + 1; while (j < s.length && s[j] !== ch) j += s[j] === '\\' ? 2 : 1; i = j; continue; }
		if (ch === o) d++;
		else if (ch === c && --d === 0) return i;
	}
	return -1;
}

// Wrap every call `name(` (name in the set, not a definition, not a method call) as (yield* name(...)).
function wrapCalls(s, names) {
	if (!names.size) return s;
	const re = new RegExp(String.raw`(?<![\w.$])(${[...names].map((n) => n.replace('.', '\\.')).join('|')})\s*\(`, 'g');
	let out = '', last = 0, m;
	while ((m = re.exec(s))) {
		const before = s.slice(Math.max(0, m.index - 12), m.index);
		if (/function\*\s*$/.test(before)) continue;   // the definition itself
		const open = m.index + m[0].length - 1, close = matchBracket(s, open);
		if (close < 0) continue;
		const inner = wrapCalls(s.slice(open + 1, close), names);
		out += s.slice(last, m.index) + `(yield* ${m[1]}(${inner}))`;
		last = close + 1; re.lastIndex = close + 1;
	}
	return out + s.slice(last);
}

// C++ source -> { code, functions } where code is the body of a generator that defines everything
// and finally runs run_routine().
export function translateCpp(src, blocking = []) {
	let s = stripComments(src.replace(/\r/g, ''));
	s = s.replace(/^\s*#.*$/gm, '');                                            // #include, #pragma, #define
	// Park string literals so nothing below rewrites their contents ("%.0f" stays "%.0f").
	const strs = [];
	s = s.replace(/"(?:[^"\\\n]|\\.)*"|'(?:[^'\\\n]|\\.)*'/g, (m) => `__STR${strs.push(m) - 1}__`);
	s = s.replace(/\busing\s+namespace\s+[\w:]+\s*;/g, '');
	// Anonymous / named namespaces: drop the wrapper, keep the contents.
	for (let m; (m = /\bnamespace\s*\w*\s*\{/.exec(s));) {
		const open = m.index + m[0].length - 1, close = matchBracket(s, open);
		s = s.slice(0, m.index) + s.slice(open + 1, close) + s.slice(close + 1);
	}
	// struct / enum definitions: JS needs none of them. Remember the struct names as types.
	const structs = [];
	for (let m; (m = /\b(struct|enum\s+class|enum)\s+(\w+)\s*\{/.exec(s));) {
		structs.push(m[2]);
		const open = m.index + m[0].length - 1, close = matchBracket(s, open);
		const semi = s.indexOf(';', close);
		s = s.slice(0, m.index) + s.slice(semi + 1);
	}
	// Function definitions -> generator functions. Prototypes (ending in ';') are dropped.
	const fns = new Set();
	const defRe = new RegExp(String.raw`(^|[;}\n])\s*${TYPE}\s+([A-Za-z_]\w*)\s*\(([^()]*)\)\s*(\{|;)`, 'g');
	s = s.replace(defRe, (all, pre, name, params, end) => {
		if (KEYWORDS.has(name)) return all;
		const typeWord = all.slice(pre.length).trim().split(/\s+/)[0];
		if (KEYWORDS.has(typeWord) || typeWord === 'return' || typeWord === 'else') return all;
		if (end === ';') return pre;   // forward declaration
		fns.add(name);
		const ps = params.trim() === 'void' ? '' : params.split(',').map((p) => {
			p = p.trim(); if (!p) return '';
			const def = p.match(/=\s*(.+)$/);
			const id = p.replace(/=.*$/, '').trim().match(/([A-Za-z_]\w*)\s*(\[\s*\d*\s*\])?$/);
			return id ? id[1] + (def ? ' = ' + def[1] : '') : '';
		}).filter(Boolean).join(', ');
		return `${pre}\nfunction* ${name}(${ps}) {`;
	});
	// Casts and literals.
	s = s.replace(/\bstatic_cast\s*<\s*(?:int|long|int32_t|uint32_t|short)\s*>\s*\(/g, 'Math.trunc(')
		.replace(/\bstatic_cast\s*<[^>]*>\s*\(/g, '(')
		.replace(/\((?:int|long|int32_t|short)\)\s*(?=[\w(])/g, '__INT__')
		.replace(/\((?:float|double|uint32_t|size_t|unsigned|bool)\)\s*/g, '')
		.replace(/\b(\d+\.\d*|\.\d+|\d+)[fF]\b/g, '$1')
		.replace(/\b(\d+)[uUlL]+\b/g, '$1')
		.replace(/\bnullptr\b/g, 'null');
	// (int) applied to a call or a name: Math.trunc(...)
	for (let i; (i = s.indexOf('__INT__')) >= 0;) {
		let j = i + 7;
		const m = s.slice(j).match(/^[\w.:]+/);
		if (m) j += m[0].length;
		if (s[j] === '(') j = matchBracket(s, j) + 1;
		s = s.slice(0, i) + 'Math.trunc(' + s.slice(i + 7, j) + ')' + s.slice(j);
	}
	// Maths.
	s = s.replace(/\bstd::(fabs|abs)\b/g, 'Math.abs').replace(/\bstd::lround\b/g, 'Math.round').replace(/\bstd::round\b/g, 'Math.round')
		.replace(/\bstd::(min|max|sin|cos|tan|atan2|atan|asin|acos|sqrt|hypot|floor|ceil|pow|exp|log|trunc)\b/g, 'Math.$1')
		.replace(/\bstd::clamp\b/g, 'clamp').replace(/\bstd::fmod\b/g, 'fmod').replace(/\bstd::/g, '')
		.replace(/\bpros::millis\b/g, 'millis').replace(/\bpros::delay\b/g, 'delay').replace(/\bpros::(\w+)::(\w+)\b/g, '"$2"');
	// Enum constants: A::B or a::B::C -> "C".
	s = s.replace(/\b[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*::([A-Za-z_]\w*)\b/g, '"$1"');
	// Designated initializers {.a = 1, .b = x} -> {a: 1, b: x}.
	s = s.replace(/([{,]\s*)\.([A-Za-z_]\w*)\s*=(?!=)/g, '$1$2:');
	// snprintf(buf, sizeof(buf), fmt, ...) -> buf = sprintf(fmt, ...)
	s = s.replace(/\bsnprintf\s*\(\s*([A-Za-z_]\w*)\s*,\s*sizeof\s*\(?\s*\w+\s*\)?\s*,/g, '$1 = sprintf(');
	// Declarations: [static|const|constexpr] Type [&*] name -> let name. char arrays -> ''.
	s = s.replace(/\bchar\s+([A-Za-z_]\w*)\s*\[\s*\w*\s*\]\s*(=\s*__STR\d+__)?\s*;/g, (a, n, init) => `let ${n} = ${init ? init.slice(1).trim() : "''"};`);
	const typeNames = ['int', 'float', 'double', 'bool', 'auto', 'char', 'long', 'short', 'uint32_t', 'int32_t', 'uint8_t', 'int8_t', 'size_t', 'std::string', 'string',
		'lemlib::Pose', 'Pose', ...structs, 'ScoreApproachResult', 'AutonSide', 'AutonColor'];
	const declRe = new RegExp(String.raw`(^|[;{}(\n]|\bfor\s*\()\s*(?:static\s+|inline\s+|constexpr\s+|const\s+|volatile\s+|unsigned\s+|signed\s+)*(?:${typeNames.map((t) => t.replace(/:/g, '\\:')).join('|')})\b(?:\s*const)?\s*[&*]?\s*(?=[A-Za-z_]\w*\s*(?:=|;|,|\[|\{|\)))`, 'g');
	s = s.replace(declRe, (a, pre) => `${pre} let `);
	// Remaining `(let x` from for-loops are fine; `let x{...}` brace init -> = {...}
	s = s.replace(/\blet\s+([A-Za-z_]\w*)\s*\{/g, 'let $1 = {');
	// lemlib::Pose(x, y, theta) style constructors are rare; leave them.
	// Calls that block: helper functions and blocking commands / chassis motions.
	const wrap = new Set([...fns, ...blocking]);
	s = wrapCalls(s, wrap);
	s = s.replace(/__STR(\d+)__/g, (m, i) => strs[+i]);
	const hasMain = fns.has('run_routine');
	const code = hasMain ? `${s}\nyield* run_routine();` : s;
	return { code, functions: [...fns] };
}

// printf-style formatting for snprintf / printf (%d %i %f %.Nf %s %c %%, widths and '-').
export function sprintf(fmt, ...args) {
	let k = 0;
	return String(fmt).replace(/%([-0 +]*)(\d*)(?:\.(\d+))?([difsuxXc%])/g, (m, flags, width, prec, t) => {
		if (t === '%') return '%';
		let v = args[k++], s;
		if (t === 'd' || t === 'i' || t === 'u') s = String(Math.trunc(+v || 0));
		else if (t === 'f') s = (+v || 0).toFixed(prec === undefined ? 6 : +prec);
		else if (t === 'x' || t === 'X') s = (Math.trunc(+v || 0) >>> 0).toString(16)[t === 'X' ? 'toUpperCase' : 'toLowerCase']();
		else if (t === 'c') s = typeof v === 'number' ? String.fromCharCode(v) : String(v ?? '');
		else s = String(v ?? '');
		if (t === 's' && prec !== undefined) s = s.slice(0, +prec);
		const w = +width || 0;
		if (s.length < w) s = flags.includes('-') ? s.padEnd(w) : flags.includes('0') && t !== 's' ? s.padStart(w, '0') : s.padStart(w);
		return s;
	});
}
