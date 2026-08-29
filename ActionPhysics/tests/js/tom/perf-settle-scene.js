// Tom's Suite — PERF SETTLE SCENE (visual, always passes; for watching, not asserting).
// The exact scenario the perf benchmark (tests/bench/scene-perf.js) measures, built as a watchable
// test: a 200m x 200m static map mesh with 1,000,000 triangles, and 500 mixed convex props (boxes,
// cylinders, cones) dropped onto it. Narrates the two phases live — falling (cheap, no contacts yet)
// and settled (expensive, every resting body re-walks the BVH and re-runs GJK every frame even though
// nothing is moving) — and logs real per-step timing at intervals so the cost is visible directly,
// not just described. No pass/fail claim beyond "the scene ran" — this is for seeing it, not testing it.
(function (Runner, U) {
	Runner.suite('tom');

	var MAP_SIZE = 200;       // meters, matches the real target scene
	var TARGET_TRIS = 1000000;
	var NUM_BODIES = 500;
	var TOTAL_TICKS = 260;    // ~4.3s: long enough to fall, land, and settle
	var LOG_EVERY = 20;       // ticks between timing log lines

	// Deterministic PRNG so the drop pattern is identical every run (headless == browser).
	function mulberry32(seed) {
		return function () {
			seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
			var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
			t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
			return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
		};
	}

	// Builds the same map-mesh generator the perf benchmark uses: a rolling heightfield sized to
	// MAP_SIZE with enough subdivisions to hit ~targetTris triangles, plus scattered raised bumps.
	function buildMapMesh(targetTris, mapSize, rand) {
		var quads = Math.max(1, Math.round(targetTris / 2));
		var res = Math.max(2, Math.round(Math.sqrt(quads)));
		var halfExtent = mapSize / 2;
		var verts = [], faces = [];
		var step = (halfExtent * 2) / res;
		for (var z = 0; z <= res; z++) {
			for (var x = 0; x <= res; x++) {
				var wx = -halfExtent + x * step;
				var wz = -halfExtent + z * step;
				var h = Math.sin(wx * 0.05) * 0.6 + Math.cos(wz * 0.05) * 0.6;
				if (rand() < 0.01) h += 1.5 + rand() * 2;
				verts.push([wx, h, wz]);
			}
		}
		function idx(x, z) { return z * (res + 1) + x; }
		for (z = 0; z < res; z++) {
			for (x = 0; x < res; x++) {
				var a = idx(x, z), b = idx(x + 1, z), c = idx(x + 1, z + 1), d = idx(x, z + 1);
				faces.push(a, b, c);
				faces.push(a, c, d);
			}
		}
		return { verts: verts, faces: faces };
	}

	Runner.test('perf', '500 props settling on a 1,000,000-triangle map', function (t) {
		var rand = mulberry32(1234);
		var w = t.makeWorld({ gravity: -9.8 });

		t.log('Building the map: ' + MAP_SIZE + 'm x ' + MAP_SIZE + 'm, target ' + TARGET_TRIS.toLocaleString() + ' triangles. This is the exact geometry tests/bench/scene-perf.js measures.');

		var mapData = buildMapMesh(TARGET_TRIS, MAP_SIZE, rand);
		var actualTris = mapData.faces.length / 3;
		var buildStart = (typeof performance !== 'undefined' ? performance.now() : Date.now());
		var mapBody = t.mesh(w, mapData.verts, mapData.faces, 0, U.withMat({ pos: [0, 0, 0], color: '#3a4a3a' }));
		var buildMs = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - buildStart;

		t.log('Map built: ' + actualTris.toLocaleString() + ' actual triangles. BVH construction took ' + buildMs.toFixed(0) + 'ms (one-time load cost, not per-frame).');
		t.log('Dropping ' + NUM_BODIES + ' mixed props (boxes / cylinders / cones) from 2-8m up, scattered across the map.');

		var halfExtent = MAP_SIZE / 2;
		var bodies = [];
		for (var i = 0; i < NUM_BODIES; i++) {
			var kind = i % 3;
			var body;
			var x = (rand() * 2 - 1) * halfExtent * 0.9;
			var z = (rand() * 2 - 1) * halfExtent * 0.9;
			var y = 2 + rand() * 6;
			var rot = [rand() - 0.5, rand() - 0.5, rand() - 0.5, 1];
			var opts = U.withMat({ pos: [x, y, z], rot: rot, color: kind === 0 ? '#c98' : (kind === 1 ? '#8ac' : '#c88') });
			if (kind === 0) body = t.box(w, 0.4 + rand() * 0.3, 0.4 + rand() * 0.3, 0.4 + rand() * 0.3, 5 + rand() * 10, opts);
			else if (kind === 1) body = t.cylinder(w, 0.3 + rand() * 0.2, 0.4 + rand() * 0.3, 5 + rand() * 10, opts);
			else body = t.cone(w, 0.3 + rand() * 0.2, 0.5 + rand() * 0.3, 5 + rand() * 10, opts);
			bodies.push(body);
		}

		t.log('All ' + NUM_BODIES + ' bodies spawned. Stepping the world — watch step time climb as bodies land, then plateau once everything is at rest.');

		var phase = 'falling';
		var settleDeclaredAt = null;

		// t.simulate's loop calls onTick(world, tick) THEN world.step for that same tick, so this
		// hook can't time "this" tick's step — instead wrap world.step once to record each step's own
		// duration, and the hook (running just before the NEXT step) reports the PRIOR tick's time.
		// That's a one-tick lag on the logged number, immaterial at a 20-tick log interval.
		var lastStepMs = 0;
		var now = function () { return (typeof performance !== 'undefined' ? performance.now() : Date.now()); };
		var origStep = w.step.bind(w);
		var lastTick = 0;
		w.step = function (dt) {
			var t0 = now();
			var r = origStep(dt);
			lastStepMs = now() - t0;
			return r;
		};

		t.onTick(function (world, tick) {
			lastTick = tick;
			if (tick === 1) { t.log('Tick 1: bodies released.'); }

			// SAPBroadphase.computePairs() is called fresh each World.step and keeps no retained pair
			// count of its own (see SAPBroadphase.js) - manifolds (real detected contact, not just
			// broadphase candidates) are what this test actually cares about anyway: "first contact with
			// the map" reads more truthfully as "first real touch," not "first pair that might touch."
			var manifoldCount = world.narrowphase.manifolds.size;
			if (phase === 'falling' && manifoldCount > 0) {
				phase = 'landing';
				t.log('Tick ' + tick + ': first contact with the map (manifolds=' + manifoldCount + '). Entering the landing/settling phase — this is where cost ramps.');
			}

			if (tick % LOG_EVERY === 0) {
				t.log('Tick ' + tick + ' [' + phase + ']: step=' + lastStepMs.toFixed(1) + 'ms  manifolds=' + manifoldCount);
			}

			if (phase === 'landing' && settleDeclaredAt == null) {
				var allSlow = true;
				for (var bi = 0; bi < bodies.length; bi++) {
					if (U.speed(bodies[bi]) > 0.1 || U.spin(bodies[bi]) > 0.1) { allSlow = false; break; }
				}
				if (allSlow && tick > 30) {
					settleDeclaredAt = tick;
					phase = 'settled';
					t.log('Tick ' + tick + ': every body is effectively at rest (|v|<0.1, |w|<0.1). Entering the settled phase — this is the steady-state cost tests/bench/scene-perf.js reports, and it never drops from here: every resting body still gets a full BVH walk + GJK pass, every frame, forever.');
				}
			}
		});

		t.expect('scene ran to completion (this test is for watching, not asserting)', function (world) {
			if (lastTick < TOTAL_TICKS) return false; // keep pending until the run actually ends
			return { ok: true, detail: 'phase=' + phase + (settleDeclaredAt ? (' settledAtTick=' + settleDeclaredAt) : ' (never had every body under threshold at once — see step-time plateau in the log instead)') + '  final step=' + lastStepMs.toFixed(1) + 'ms' };
		});

		t.simulate(w, TOTAL_TICKS);
	}, {
		visual: true, steps: TOTAL_TICKS, page: 'perf',
		description: 'The real perf-benchmark scene, built as something to watch: a 200m x 200m map with 1,000,000 static triangles, and 500 mixed props (boxes/cylinders/cones) dropped onto it. Narrates the fall -> first-contact -> settled transition live via the log. No hard pass/fail — it always passes; the point is to see the scene and its cost pattern directly, matching tests/bench/scene-perf.js and tests/bench/narrowphase-breakdown.js.'
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
   typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil);
