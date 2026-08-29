/**
 * Headless entry point.
 *
 * The SAME tests suite.html renders visually, run here in node with no renderer. One test definition
 * (suite()/test() + ctx.simulate/ctx.expect), two ways to run it, so the picture and the report cannot
 * disagree.
 *
 *   node tests/run_headless.js                run everything
 *   node tests/run_headless.js vector3        only groups whose name contains "vector3"
 *   node tests/run_headless.js --suite=math   only one suite
 *   node tests/run_headless.js --logs         print every criterion, not just failures
 */
var fs = require('fs');
var path = require('path');

var HERE = __dirname;
var Runner = require('./js/runner.js');

// Which build is under test. A stale build silently producing wrong results has bitten this project
// before, so the stamp goes out before anything runs.
var buildFile = path.join(HERE, '..', 'build', 'actionphysics.js');
if (!fs.existsSync(buildFile)) {
	console.error('No build found. Run: node build.js');
	process.exit(2);
}
var stampLine = fs.readFileSync(buildFile, 'utf8').split(/\r?\n/)[0];
console.log(stampLine.replace(/^\/\/\s*/, '=== ') + ' ===');

// Skipped from headless runs: the 385-box pyramid (1200 ticks x 385 bodies) is slow and would
// dominate every run's signal during perf iteration. The two perf-settle scenes are pure timing
// benchmarks, not pass/fail correctness tests, and dominate wall-clock time; skipped by explicit
// standing instruction until told otherwise. The files are untouched, just unloaded here; remove
// an entry to fold it back into every run.
var SKIP_FILES = {
	'pyramid.js': true,
	'perf-settle-scene.js': true,
	'perf-settle-scene-compound.js': true,
};

// Suite files. A leading _ marks a shared helper, loaded first and never treated as a suite.
var suitesDir = path.join(HERE, 'js', 'suites');
var utilFile = path.join(suitesDir, '_util.js');
if (fs.existsSync(utilFile)) require(utilFile);
fs.readdirSync(suitesDir)
	.filter(function (f) { return f.endsWith('.js') && f.charAt(0) !== '_' && !SKIP_FILES[f]; })
	.sort()
	.forEach(function (f) { require(path.join(suitesDir, f)); });

// Tom's suite: a separate directory (not folded into js/suites/) of scene-style FPS/physics
// regression tests, kept apart because it has its own shared helpers (_util.js, _util_fps.js) and
// naming conventions distinct from the base engine suite. Loaded the same way - _-prefixed files
// first (as plain requires, not test files), then everything else.
var tomDir = path.join(HERE, 'js', 'tom');
if (fs.existsSync(tomDir)) {
	fs.readdirSync(tomDir)
		.filter(function (f) { return f.endsWith('.js') && f.charAt(0) === '_'; })
		.sort()
		.forEach(function (f) { require(path.join(tomDir, f)); });
	fs.readdirSync(tomDir)
		.filter(function (f) { return f.endsWith('.js') && f.charAt(0) !== '_' && !SKIP_FILES[f]; })
		.sort()
		.forEach(function (f) { require(path.join(tomDir, f)); });
}

var onlySuite = null, only = null, showLogs = false;
process.argv.slice(2).forEach(function (a) {
	if (a === '--logs') showLogs = true;
	else if (a.indexOf('--suite=') === 0) onlySuite = a.slice(8);
	else only = a;
});
var filter = function (t) {
	if (onlySuite && t.suite !== onlySuite) return false;
	if (only && t.group.indexOf(only) === -1) return false;
	return true;
};

var curSuite = null, curGroup = null;
var summary = Runner.run(filter, function (r) {
	if (r.suite !== curSuite) { curSuite = r.suite; curGroup = null; console.log('\n##### ' + curSuite + ' #####'); }
	if (r.group !== curGroup) { curGroup = r.group; console.log('  [' + curGroup + ']'); }
	console.log('    ' + (r.ok ? 'ok  ' : 'FAIL') + '  ' + r.name + (r.ok ? '' : '   -> ' + r.error));
	if (showLogs && r.logs && r.logs.length) {
		r.logs.forEach(function (e) {
			if (e.type === 'criterion') console.log('        [' + e.status + '] ' + e.label + (e.detail ? '  (' + e.detail + ')' : ''));
			else if (e.type === 'log') console.log('        ' + e.msg);
		});
	}
});

console.log('\n=== ' + summary.pass + ' passed, ' + summary.fail + ' failed (' + summary.total + ' total) ===');
process.exit(summary.fail > 0 ? 1 : 0);
