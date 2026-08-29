// Tom's Suite — SLEEP (island-based body sleeping).
// Sleep parks a body that has come to rest: the island manager (src/solver/IslandManager.js) groups
// coupled bodies into islands and, once every member of an island has stayed below the sleep
// thresholds for TIME_TO_SLEEP, zeroes their velocity and clears isAwake so the solver skips them.
// Bodies wake as a whole island the instant any member is disturbed (a new moving contact, or a
// force/impulse through the body's own API).
//
// These are LIVE predicates like every other Tom scene: they read body.isAwake and the body's live
// velocity every tick against the running world, identical headless and in-browser. The value of
// having them here (rather than in a throwaway script) is exactly that they are watchable and you see
// the same numbers the engine does.
(function (Runner, U) {

	var TOTAL = 240;   // ~4s: long enough to settle (drop) and cross TIME_TO_SLEEP (0.5s) with margin

	// A live predicate that passes once `b` is asleep AND has been zeroed (a sleeping body must hold
	// still by definition). Held for `hold` ticks so a single frame does not pass. maxY guards against
	// a body that launches instead of settling (can never legitimately sleep).
	function sleepsAndParks(b, opts) {
		opts = opts || {};
		var hold = opts.hold != null ? opts.hold : 20;
		var maxY = opts.maxY != null ? opts.maxY : 50;
		var run = 0, blown = false;
		return function () {
			var sv = U.speed(b), sw = U.spin(b);
			if (b.position.y > maxY) blown = true;
			if (blown) return { ok: false, detail: 'y=' + b.position.y.toFixed(2) + ' LAUNCHED' };
			// Asleep implies the manager zeroed velocity; assert both so a "sleep without park" bug
			// (isAwake cleared but velocity left nonzero) cannot slip through.
			var parked = !b.isAwake && sv === 0 && sw === 0;
			if (parked) run++; else run = 0;
			return { ok: run >= hold, detail: 'awake=' + b.isAwake + ' |v|=' + sv.toFixed(3) + ' |w|=' + sw.toFixed(3) + ' slept=' + run + '/' + hold };
		};
	}

	// ---- a dropped sphere settles and goes to sleep ----
	Runner.test('sleep', 'a dropped sphere settles and then goes to sleep', function (t) {
		t.log('Drop a sphere; once it has rested past the sleep time it should park (isAwake=false, velocity zeroed).');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		var ball = t.sphere(w, 0.5, 1, U.withMat({ pos: [0, 0.5, 0], color: '#F4D35E' }));
		t.expect('sphere is asleep and parked (isAwake=false, |v|=|w|=0) for 20 ticks', sleepsAndParks(ball));
		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'sleep',
		description: 'A sphere is dropped onto the ground. PASS: after it settles and stays below the sleep ' +
			'thresholds for the sleep time, the island manager parks it — isAwake goes false and its velocity ' +
			'is zeroed. A sphere that never settles, launches, or stays awake forever fails.'
	});

	// ---- a cylinder on its side settles and sleeps (the shape that jittered forever) ----
	Runner.test('sleep', 'a cylinder resting on its side goes to sleep instead of jittering forever', function (t) {
		t.log('A cylinder on its side has a small residual spin that never fully decays; sleep should still park it.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		// 90 deg about Z: axis horizontal, barrel resting on the floor.
		var cyl = t.cylinder(w, 0.4, 1, 1, U.withMat({ pos: [0, 0.4, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 2), color: '#45B7D1' }));
		t.expect('cylinder is asleep and parked (isAwake=false, |v|=|w|=0) for 20 ticks', sleepsAndParks(cyl));
		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'sleep',
		description: 'A cylinder is laid on its side. Its barrel-on-flat contact leaves a small residual spin ' +
			'that does not fully decay on its own, but it stays under the angular sleep threshold — so PASS is ' +
			'the island manager parking it once the sleep time elapses. This is the case per-body settling alone ' +
			'could never satisfy.'
	});

	// ---- a stack sleeps as one island, and no body sinks into another ----
	Runner.test('sleep', 'a settled three-box stack sleeps as one island', function (t) {
		t.log('Three stacked boxes must sleep TOGETHER (one island) and none may sink into the one below.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		var boxes = [];
		for (var i = 0; i < 3; i++) {
			boxes.push(t.box(w, 0.5, 0.5, 0.5, 1, U.withMat({ pos: [0, 0.5 + i * 1.0, 0], color: '#8367C7' })));
		}
		// The whole stack asleep at once (island coupling), with the resting heights preserved (no sink).
		t.expect('all three boxes are asleep together AND resting at ~0.5/1.5/2.5 for 20 ticks', (function () {
			var run = 0;
			return function () {
				var allAsleep = boxes.every(function (b) { return !b.isAwake; });
				var heightsOk = boxes.every(function (b, k) { return Math.abs(b.position.y - (0.5 + k)) < 0.1; });
				if (allAsleep && heightsOk) run++; else run = 0;
				var ys = boxes.map(function (b) { return b.position.y.toFixed(3); }).join(',');
				var awake = boxes.map(function (b) { return b.isAwake ? 1 : 0; }).join(',');
				return { ok: run >= 20, detail: 'awake=[' + awake + '] ys=[' + ys + '] held=' + run + '/20' };
			};
		})());
		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'sleep',
		description: 'Three unit boxes stacked on the ground. PASS: they settle and sleep as a single island ' +
			'(all isAwake=false in the same window), with each box holding its resting height — proving a bottom ' +
			'box never sleeps out from under the one above it. NOTE: this currently exposes a pre-existing ' +
			'resting-stack residual-velocity bug (the bottom box reads a phantom ~0.15 m/s while its position is ' +
			'frozen), so it is expected to FAIL until that is fixed — the failure is real signal, not a sleep bug.'
	});

	// ---- a sleeping body wakes when disturbed ----
	Runner.test('sleep', 'a sleeping body wakes when an impulse is applied', function (t) {
		t.log('Let a sphere sleep, then apply an upward impulse; it must wake and start moving again.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		var ball = t.sphere(w, 0.5, 1, U.withMat({ pos: [0, 0.5, 0], color: '#EE964B' }));

		var WAKE_AT = 120, fired = false, sleptBeforeWake = false, wokeAndMoved = false;
		t.onTick(function (world, tick) {
			if (tick === WAKE_AT && !fired) {
				fired = true;
				sleptBeforeWake = !ball.isAwake;               // must have actually been asleep first
				ball.applyImpulse(t.vec(0, 5, 0));             // the disturbance
			}
			if (fired && ball.isAwake && ball.linear_velocity.y > 0.5) wokeAndMoved = true;
		});

		t.expect('the ball is asleep before the impulse, then wakes and moves upward after it', function () {
			if (!fired) return { ok: false, detail: 'waiting for impulse @tick ' + WAKE_AT };
			return {
				ok: sleptBeforeWake && wokeAndMoved,
				detail: 'sleptBeforeImpulse=' + sleptBeforeWake + ' wokeAndMoved=' + wokeAndMoved + ' awake=' + ball.isAwake
			};
		});
		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'sleep',
		description: 'A sphere is dropped and allowed to sleep, then at a scripted tick an upward impulse is ' +
			'applied through its own API (applyImpulse). PASS: it was genuinely asleep beforehand, and the ' +
			'impulse wakes it (isAwake=true) and sends it moving upward. A body that was never asleep, or that ' +
			'ignores the impulse, fails.'
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
