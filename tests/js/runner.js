/**
 * ActionPhysics test runner - the dependency-free framework the whole suite is built on.
 *
 * No mocha, no chai. Owns: test registration, the per-test context (assertions + world/body
 * helpers), the runner, and suite ordering (suite keys in SUITE_ORDER, unlisted suites last).
 * Holds no tests itself - suite files require/use this and register into it.
 *
 * Every test is fully STANDALONE: it builds its own world and bodies, sets full state explicitly,
 * and asserts. No shared state between tests - re-running one test alone reproduces its result.
 *
 * Loadable in both node (require) and the browser (global APRunner).
 */
(function (root, factory) {
	var AP = (typeof module !== 'undefined' && module.exports)
		? require('../../build/actionphysics.js')
		: root.ActionPhysics;
	var api = factory(AP);
	if (typeof module !== 'undefined' && module.exports) module.exports = api;
	else root.APRunner = api;
})(typeof self !== 'undefined' ? self : this, function (AP) {
	'use strict';

	var tests = [];

	// Suite currently being registered. Test files call Runner.suite(key) at the top, then register
	// their tests. Drives run order and the frontend's suite picker.
	var _currentSuite = 'math';
	function suite(key) { _currentSuite = key; }

	// Display order. An unlisted suite still runs, it just sorts last.
	var SUITE_ORDER = ['math', 'shapes', 'collision', 'solver', 'character', 'tom'];
	var SUITE_NAMES = {
		math: 'Math',
		shapes: 'Shapes',
		collision: 'Collision',
		solver: 'Solver',
		character: 'Character',
		tom: "Tom's Suite"
	};

	// A failed assertion throws this; a real code error is anything else. The runner distinguishes them.
	function AssertionError(message) { this.message = message; this.name = 'AssertionError'; }
	AssertionError.prototype = Object.create(Error.prototype);

	/**
	 * Register a test.
	 *   test(group, name, fn)          simple test; fn(t) throws (via t.check*) to fail.
	 *   test(group, name, fn, meta)    meta = { visual:bool, steps:int } for the browser viewer.
	 *
	 * meta.visual      — can this be watched in the browser?
	 * meta.steps       — for animated (dynamics) visual tests, how many frames the viewer animates. A
	 *                    visual test with steps===0 is a STATIC diagram (geometry: gjk / raytracing).
	 * meta.page        — the collapsible section this test lives under in the viewer.
	 *                    Defaults to the group with any "/suffix" stripped.
	 * meta.description — plain-English idea + pass criteria, shown when the row's "?" is expanded.
	 */
	function test(group, name, fn, meta) {
		meta = meta || {};
		tests.push({
			suite: _currentSuite, group: group, name: name, fn: fn,
			visual: meta.visual === true, steps: meta.steps || 0,
			page: meta.page || group.split('/')[0],
			description: meta.description || ''
		});
	}

	// The per-test context. Fresh for every test — no carryover. Gives assertions + world/body helpers
	// so a test never imports anything. In the browser viewer, the same ctx is used to capture what to
	// draw (ctx.world, ctx.bodies, and optionally ctx.ray for raytracing tests).
	function makeContext() {
		var ctx = {
			world: null,     // set by makeWorld(); the viewer renders this if present
			bodies: [],      // every body created via the helpers, in creation order (each has _color)
			ray: null,       // raytracing tests set { start:[3], stop:[3], hit:[3]|null } for the viewer
			support: null,   // support-point tests set { dir:[3], point:[3] } for the viewer
			swept: null,     // sweptshape tests set { shape, start:[3], end:[3] } for the viewer

			// timeline: the ordered stream the browser replays as a live checklist. Entries are either
			//   { type:'log', msg }                                   — narration ("why")
			//   { type:'criterion', label, status, detail }           — a pass/fail check
			// CRITERIA-FIRST MODEL (like mocha): a test declares each criterion up front with
			// t.criterion(label) — it renders immediately as a PENDING line — then verifies it later via
			// the returned handle's .check()/.checkEqual()/.checkTrue(), which flips it to pass/fail in
			// place. This lets the console show the full list of what's being checked before results land.
			timeline: [],
			get logs() { return ctx.timeline; }, // back-compat alias

			// ---- narration ----
			log: function (msg) { ctx.timeline.push({ type: 'log', kind: 'info', msg: String(msg) }); },

			// ---- criteria ----
			// Declare a criterion (renders pending). Returns a handle to verify it. On failure the handle's
			// verify throws (stopping the test, as before) after marking the line failed.
			// A failed criterion does NOT throw — it records the failure and the test keeps going, so the
			// whole checklist resolves (you see every criterion's ✓/✗, like mocha, not just the first
			// failure). The test's overall ok is derived from whether any criterion failed (see runOne).
			criterion: function (label) {
				var entry = { type: 'criterion', label: label, status: 'pending', detail: '' };
				ctx.timeline.push(entry);
				function fail(detail) { entry.status = 'fail'; entry.detail = detail; }
				function pass(detail) { entry.status = 'pass'; entry.detail = detail; }
				return {
					entry: entry,
					check: function (actual, expected, eps) {
						eps = (eps == null) ? 1e-6 : eps;
						var ok = typeof actual === 'number' && !isNaN(actual) && Math.abs(actual - expected) <= eps;
						ok ? pass(actual + ' ≈ ' + expected + ' (±' + eps + ')') : fail('got ' + actual + ', expected ' + expected + ' (±' + eps + ')');
						return ok;
					},
					checkEqual: function (actual, expected) {
						var ok = actual === expected;
						ok ? pass(actual + ' === ' + expected) : fail('got ' + actual + ', expected ' + expected);
						return ok;
					},
					checkTrue: function (cond, detail) {
						cond ? pass(detail || 'true') : fail(detail || 'expected true');
						return !!cond;
					}
				};
			},

			// ---- shorthand assertions (auto-create an unlabeled criterion) — for tests that don't need
			// named criteria (e.g. the pure-math checks). Still produces a timeline line.
			check: function (actual, expected, eps, label) { ctx.criterion(label || (actual + ' ≈ ' + expected)).check(actual, expected, eps); },
			checkEqual: function (actual, expected, label) { ctx.criterion(label || (actual + ' === ' + expected)).checkEqual(actual, expected); },
			checkTrue: function (cond, msg) { ctx.criterion(msg || 'condition true').checkTrue(cond, msg); },

			// ---- live expectations (for dynamics tests that animate) ----
			// t.expect(label, predicate) declares a condition that is CHECKED EVERY TICK against the live
			// world. It flips green the first tick predicate(world) returns true — the moment the physics
			// makes it so, not a tick we picked. If it's never true by the end of the run, it flips red.
			// No hardcoded ticks, nothing pre-computed: the criterion genuinely doesn't know its result
			// until the simulation produces it, on screen.
			//
			// predicate returns either a boolean, or { ok, detail } to show a value alongside the ✓/✗.
			//
			// t.simulate(world, totalTicks) drives it. HEADLESS: steps synchronously, evaluating every
			// expectation each tick. LIVE (browser): the renderer owns the stepping and calls the same
			// per-tick evaluator, so the same code decides pass/fail either way.
			expectations: [],  // { entry, predicate, met:false }
			expect: function (label, predicate) {
				var entry = { type: 'criterion', label: label, status: 'pending', detail: '' };
				ctx.timeline.push(entry);
				ctx.expectations.push({ entry: entry, predicate: predicate, met: false });
			},
			// Evaluate all not-yet-met expectations against the world at the current tick. Any that
			// become true this tick flip to pass (recording the tick). Returns true if all are met.
			evalTick: function (world, tick) {
				var allMet = true;
				for (var i = 0; i < ctx.expectations.length; i++) {
					var ex = ctx.expectations[i];
					if (ex.met) continue;
					var res = ex.predicate(world);
					var ok = res === true || (res && res.ok === true);
					if (ok) {
						ex.met = true;
						ex.entry.status = 'pass';
						ex.entry.detail = (res && res.detail ? res.detail + '  ' : '') + '(at tick ' + tick + ')';
					} else {
						allMet = false;
						if (res && res.detail) ex.entry.detail = res.detail; // live value while pending
					}
				}
				return allMet;
			},
			// Mark every still-unmet expectation as failed (called when the run ends without them met).
			failUnmet: function () {
				for (var i = 0; i < ctx.expectations.length; i++) {
					var ex = ctx.expectations[i];
					if (!ex.met) { ex.entry.status = 'fail'; ex.entry.detail = (ex.entry.detail ? ex.entry.detail + ' — ' : '') + 'never satisfied'; }
				}
			},
			// Scripted mid-sim events. ctx.onTick(fn) registers fn(world, tick) to run each tick
			// BEFORE the expectations are evaluated — used for deterministic, tick-driven disturbances
			// (a shove, a jiggle) that must behave identically headless and in-browser. When any tick
			// hooks are registered the run does NOT early-out, so a two-phase test (settle -> shove ->
			// re-settle) always runs its full tick budget instead of stopping the instant phase 1 passes.
			tickHooks: [],
			// A test may set ctx.stepWorld = fn(world) to advance the sim with a different lifecycle each
			// tick (e.g. a host controller's beginStep/fixed_update/endStep). Null = default.
			stepWorld: null,
			onTick: function (fn) { ctx.tickHooks.push(fn); },
			runTickHooks: function (world, tick) {
				for (var i = 0; i < ctx.tickHooks.length; i++) ctx.tickHooks[i](world, tick);
			},
			simulate: function (world, totalTicks) {
				ctx.simTicks = totalTicks;
				var canEarlyOut = ctx.tickHooks.length === 0;
				// One tick: default world.step(1/60), or ctx.stepWorld for another engine lifecycle
				// (e.g. beginStep/fixed_update/endStep). Live loop uses the SAME ctx.stepWorld, so
				// headless and on-screen agree tick-for-tick.
				var stepOnce = (typeof ctx.stepWorld === 'function') ? ctx.stepWorld : function (w) { w.step(1 / 60); };
				for (var tk = 1; tk <= totalTicks; tk++) {
					ctx.runTickHooks(world, tk);
					stepOnce(world);
					var allMet = ctx.evalTick(world, tk);
					if (allMet && canEarlyOut) break; // early-out only when nothing is scripted
				}
				ctx.failUnmet();
			},

			// ---- world + bodies ----
			// makeWorld({ gravity }) — omit gravity for the engine default (-9.8), pass 0 for zero-g.
			makeWorld: function (opts) {
				opts = opts || {};
				var w = new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), new AP.Solver());
				if (opts.gravity != null) w.gravity = new AP.Vector3(0, opts.gravity, 0);
				ctx.world = w;
				return w;
			},
			// Add a body. opts: { pos:[x,y,z], vel:[x,y,z], rot:[x,y,z,w] (normalized), restitution,
			// linear_damping, angular_damping, color }. Returns the raw RigidBody.
			sphere: function (w, radius, mass, opts) { return add(w, new AP.SphereShape(radius), mass, opts); },
			box: function (w, hx, hy, hz, mass, opts) { return add(w, new AP.BoxShape(hx, hy, hz), mass, opts); },
			cylinder: function (w, radius, halfHeight, mass, opts) { return add(w, new AP.CylinderShape(radius, halfHeight), mass, opts); },
			cone: function (w, radius, halfHeight, mass, opts) { return add(w, new AP.ConeShape(radius, halfHeight), mass, opts); },
			capsule: function (w, radius, totalHeight, mass, opts) { return add(w, new AP.CapsuleShape(radius, totalHeight), mass, opts); },
			// mesh(w, verts, faces, mass, opts) — verts: [[x,y,z],...], faces: flat index triples.
			mesh: function (w, verts, faces, mass, opts) {
				var vs = verts.map(function (v) { return new AP.Vector3(v[0], v[1], v[2]); });
				return add(w, new AP.MeshShape(vs, faces), mass, opts);
			},
			plane: function (w, orientation, halfW, halfL, mass, opts) { return add(w, new AP.PlaneShape(orientation, halfW, halfL), mass, opts); },
			convex: function (w, verts, mass, opts) {
				var vs = verts.map(function (v) { return new AP.Vector3(v[0], v[1], v[2]); });
				return add(w, new AP.ConvexShape(vs), mass, opts);
			},
			// A lone body NOT added to a world (for geometry queries: findSupportPoint / rayIntersect).
			// Applies pos/rot and updateDerived; also pushed to ctx.bodies so the viewer can draw it.
			loneBody: function (shape, opts) {
				opts = opts || {};
				var b = new AP.RigidBody(shape, opts.mass != null ? opts.mass : 1);
				applyOpts(b, opts);
				b.updateDerived();
				b._color = opts.color || '#4af';
				ctx.bodies.push(b);
				return b;
			},

			// ---- helpers ----
			vec: function (x, y, z) { return new AP.Vector3(x, y, z); },
			quat: function (x, y, z, w) { return new AP.Quaternion(x, y, z, w).normalize(); },
			step: function (w, n) { for (var i = 0; i < n; i++) w.step(1 / 60); },
			AP: AP,
			DT: 1 / 60
		};

		function applyOpts(b, opts) {
			if (opts.pos) b.position.set(opts.pos[0] || 0, opts.pos[1] || 0, opts.pos[2] || 0);
			if (opts.rot) b.rotation = new AP.Quaternion(opts.rot[0], opts.rot[1], opts.rot[2], opts.rot[3]).normalize();
			if (opts.vel) b.linear_velocity.set(opts.vel[0] || 0, opts.vel[1] || 0, opts.vel[2] || 0);
			if (opts.avel) b.angular_velocity.set(opts.avel[0] || 0, opts.avel[1] || 0, opts.avel[2] || 0);
			if (opts.friction != null) b.friction = opts.friction;
			if (opts.angular_friction != null) b.angular_friction = opts.angular_friction;
			if (opts.restitution != null) b.restitution = opts.restitution;
			if (opts.linear_damping != null) b.linear_damping = opts.linear_damping;
			if (opts.angular_damping != null) b.angular_damping = opts.angular_damping;
			if (opts.noGravity) b.setGravity(0, 0, 0);
		}
		function add(w, shape, mass, opts) {
			opts = opts || {};
			var b = new AP.RigidBody(shape, mass);
			applyOpts(b, opts);
			w.addRigidBody(b);
			b._color = opts.color || '#4af';
			ctx.bodies.push(b);
			return b;
		}
		return ctx;
	}

	// Tests in run order: SUITE_ORDER first, registration order within a suite.
	function orderedTests() {
		var out = [], s, i;
		for (s = 0; s < SUITE_ORDER.length; s++)
			for (i = 0; i < tests.length; i++) if (tests[i].suite === SUITE_ORDER[s]) out.push(tests[i]);
		for (i = 0; i < tests.length; i++) if (SUITE_ORDER.indexOf(tests[i].suite) === -1) out.push(tests[i]);
		return out;
	}

	// Run a single test in a FRESH context. Returns { suite, group, name, ..., ok, error, ctx, timeline }.
	// ok is derived from the criteria: a test passes iff every criterion passed AND nothing threw. ctx is
	// returned so the browser viewer can draw exactly the state this run produced and replay its timeline.
	function runOne(t) {
		var ctx = makeContext();
		var r = { suite: t.suite, group: t.group, name: t.name, page: t.page, description: t.description,
			visual: t.visual, steps: t.steps, ok: true, error: null, ctx: ctx, timeline: ctx.timeline, logs: ctx.timeline };
		try {
			t.fn(ctx);
		} catch (e) {
			// A real thrown error (not an assertion — criteria no longer throw) fails the test outright.
			r.ok = false;
			r.error = (e && e.message) ? e.message : String(e);
			if (!(e instanceof AssertionError)) r.error = 'ERROR: ' + r.error;
		}
		// Derive ok/error from the criteria checklist.
		var failed = [];
		for (var i = 0; i < ctx.timeline.length; i++) {
			var e = ctx.timeline[i];
			if (e.type === 'criterion' && e.status === 'fail') failed.push(e.label + ' (' + e.detail + ')');
		}
		if (failed.length) { r.ok = false; if (!r.error) r.error = failed.join('; '); }
		return r;
	}

	// Run all (or a filtered subset), in suite order. onResult(result) called per test. Returns summary.
	// opts.bail — stop the whole run at the first failing test (summary still reflects what ran).
	function run(filter, onResult, opts) {
		opts = opts || {};
		var results = [], pass = 0, fail = 0, ordered = orderedTests(), i;
		for (i = 0; i < ordered.length; i++) {
			if (filter && !filter(ordered[i])) continue;
			var r = runOne(ordered[i]);
			r.ok ? pass++ : fail++;
			results.push(r);
			if (onResult) onResult(r);
			if (!r.ok && opts.bail) break;
		}
		return { results: results, pass: pass, fail: fail, total: pass + fail };
	}

	// Run the whole suite for one key, sequentially, in registration order. This is the "run a category
	// as a unit" primitive: same as run() with a suite filter, but returns a per-suite summary and (when
	// onSuiteDone is given) reports it as a block. opts.bail stops at the first failing test in the suite.
	function runSuite(suiteKey, onResult, opts) {
		var s = run(function (t) { return t.suite === suiteKey; }, onResult, opts);
		s.suite = suiteKey;
		s.name = SUITE_NAMES[suiteKey] || suiteKey;
		return s;
	}

	// Run every suite in turn, each as its own block. onSuiteStart(key,name)/onResult(r)/onSuiteDone(sum)
	// fire around each suite. Returns { suites:[perSuiteSummary...], pass, fail, total }. opts.bail stops
	// the current suite at its first failure AND stops starting further suites.
	function runBySuite(onSuiteStart, onResult, onSuiteDone, opts) {
		opts = opts || {};
		var keys = [], seen = {}, ordered = orderedTests(), i;
		for (i = 0; i < ordered.length; i++) {
			if (!seen[ordered[i].suite]) { seen[ordered[i].suite] = 1; keys.push(ordered[i].suite); }
		}
		var out = { suites: [], pass: 0, fail: 0, total: 0 };
		for (i = 0; i < keys.length; i++) {
			if (onSuiteStart) onSuiteStart(keys[i], SUITE_NAMES[keys[i]] || keys[i]);
			var s = runSuite(keys[i], onResult, opts);
			out.suites.push(s); out.pass += s.pass; out.fail += s.fail; out.total += s.total;
			if (onSuiteDone) onSuiteDone(s);
			if (opts.bail && s.fail > 0) break;
		}
		return out;
	}

	// Find one test by identity (suite/group/name) and run it fresh — used by the viewer's watch button.
	function runByName(suiteKey, group, name) {
		for (var i = 0; i < tests.length; i++) {
			var t = tests[i];
			if (t.suite === suiteKey && t.group === group && t.name === name) return { test: t, result: runOne(t) };
		}
		return null;
	}

	function groups(suiteKey) {
		var seen = {}, out = [];
		for (var i = 0; i < tests.length; i++) {
			if (suiteKey && tests[i].suite !== suiteKey) continue;
			if (!seen[tests[i].group]) { seen[tests[i].group] = 1; out.push(tests[i].group); }
		}
		return out;
	}

	// Every suite that has at least one registered test, in run order: SUITE_ORDER first, then any
	// unlisted key in first-seen order (matching orderedTests). Unlisted keys used to be invisible here.
	function suites() {
		var counts = {}, order = [], i;
		for (i = 0; i < SUITE_ORDER.length; i++) { counts[SUITE_ORDER[i]] = 0; order.push(SUITE_ORDER[i]); }
		for (i = 0; i < tests.length; i++) {
			var k = tests[i].suite;
			if (!(k in counts)) { counts[k] = 0; order.push(k); }
			counts[k]++;
		}
		var out = [];
		for (i = 0; i < order.length; i++) {
			if (counts[order[i]] > 0) out.push({ key: order[i], name: SUITE_NAMES[order[i]] || order[i], count: counts[order[i]] });
		}
		return out;
	}

	return {
		suite: suite, test: test,
		run: run, runSuite: runSuite, runBySuite: runBySuite, runOne: runOne, runByName: runByName,
		groups: groups, suites: suites, orderedTests: orderedTests, tests: tests
	};
});
