(function (Runner, U) {
	Runner.suite('tom');

	var NUM_CHILDREN = 3000;
	var NUM_PROPS = 500;
	var MAP_SIZE = 200;
	var TOTAL_TICKS = 260;
	var LOG_EVERY = 20;

	function mulberry32(seed) {
		return function () {
			seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
			var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
			t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
			return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
		};
	}

	function buildCompoundGround(t, w, rand) {
		var perSide = Math.max(1, Math.round(Math.sqrt(NUM_CHILDREN)));
		var half = MAP_SIZE / 2;
		var tileSize = (half * 2) / perSide;

		var AP = t.AP;
		var compound = new AP.CompoundShape();
		var ident = new AP.Quaternion(0, 0, 0, 1);
		var zero = new AP.Vector3(0, 0, 0);

		for (var gz = 0; gz < perSide; gz++) {
			for (var gx = 0; gx < perSide; gx++) {
				var cx = -half + tileSize / 2 + gx * tileSize;
				var cz = -half + tileSize / 2 + gz * tileSize;
				var th = tileSize / 2;
				var h0 = Math.sin(cx * 0.05) * 0.6 + Math.cos(cz * 0.05) * 0.6;
				var v = [
					new AP.Vector3(cx - th, h0, cz - th),
					new AP.Vector3(cx + th, h0, cz - th),
					new AP.Vector3(cx + th, h0, cz + th),
					new AP.Vector3(cx - th, h0, cz + th)
				];
				var f = [0, 2, 1, 0, 3, 2];
				compound.addChild(new AP.MeshShape(v, f), zero, ident);
			}
		}

		var groundBody = new AP.RigidBody(compound, 0);
		groundBody._color = '#3a4a3a';
		w.addRigidBody(groundBody);
		t.bodies.push(groundBody);
		return { body: groundBody, actualChildren: perSide * perSide };
	}

	Runner.test('perf', NUM_PROPS + ' props settling on a ' + NUM_CHILDREN + '-child CompoundShape ground', function (t) {
		var rand = mulberry32(1234);
		var w = t.makeWorld({ gravity: -9.8 });

		t.log('Building the ground: a CompoundShape of ~' + NUM_CHILDREN.toLocaleString() + ' small MeshShape tiles over a ' + MAP_SIZE + 'm x ' + MAP_SIZE + 'm map. This is the exact geometry tests/bench/compound-children-perf.js measures — many small children combined for broadphase, not one giant mesh.');

		var buildStart = (typeof performance !== 'undefined' ? performance.now() : Date.now());
		var ground = buildCompoundGround(t, w, rand);
		var buildMs = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - buildStart;

		t.log('Ground built: ' + ground.actualChildren.toLocaleString() + ' actual children. Build took ' + buildMs.toFixed(0) + 'ms (one-time load cost, not per-frame).');
		t.log('Dropping ' + NUM_PROPS + ' mixed props (boxes / cylinders / cones) from 2-8m up, scattered across the map.');

		var halfExtent = MAP_SIZE / 2;
		var bodies = [];
		for (var i = 0; i < NUM_PROPS; i++) {
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

		t.log('All ' + NUM_PROPS + ' bodies spawned. Stepping the world — watch step time (and the solver split) climb as bodies land, then plateau once everything is at rest.');

		var phase = 'falling';
		var settleDeclaredAt = null;

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

			var manifoldCount = world.narrowphase.manifolds.size;
			if (phase === 'falling' && manifoldCount > 0) {
				phase = 'landing';
				t.log('Tick ' + tick + ': first contact with the ground (manifolds=' + manifoldCount + '). Entering the landing/settling phase — this is where cost ramps.');
			}

			if (tick % LOG_EVERY === 0) {
				t.log('Tick ' + tick + ' [' + phase + ']: step=' + lastStepMs.toFixed(2) + 'ms  manifolds=' + manifoldCount);
			}

			if (phase === 'landing' && settleDeclaredAt == null) {
				var allSlow = true;
				for (var bi = 0; bi < bodies.length; bi++) {
					if (U.speed(bodies[bi]) > 0.1 || U.spin(bodies[bi]) > 0.1) { allSlow = false; break; }
				}
				if (allSlow && tick > 30) {
					settleDeclaredAt = tick;
					phase = 'settled';
					t.log('Tick ' + tick + ': every body is effectively at rest (|v|<0.1, |w|<0.1). Entering the settled phase — this is the steady-state cost tests/bench/compound-children-perf.js reports, and it never drops from here (no sleep system): every resting body still gets a full solve pass, every frame, forever.');
				}
			}
		});

		t.expect('scene ran to completion (this test is for watching, not asserting)', function (world) {
			if (lastTick < TOTAL_TICKS) return false;
			return { ok: true, detail: 'phase=' + phase + (settleDeclaredAt ? (' settledAtTick=' + settleDeclaredAt) : ' (never had every body under threshold at once — see step-time plateau in the log instead)') + '  final step=' + lastStepMs.toFixed(2) + 'ms' };
		});

		t.simulate(w, TOTAL_TICKS);
	}, {
		visual: true, steps: TOTAL_TICKS, page: 'perf',
		description: 'The real perf-benchmark scene, built as something to watch: a ' + MAP_SIZE + 'm x ' + MAP_SIZE + 'm CompoundShape ground made of ~' + NUM_CHILDREN.toLocaleString() + ' small MeshShape tiles, with ' + NUM_PROPS + ' mixed props (boxes/cylinders/cones) dropped onto it. Narrates the fall -> first-contact -> settled transition live via the log, with per-tick step time and manifold count. No hard pass/fail — it always passes; the point is to see the scene and its cost pattern directly.'
	});
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
   typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil);
