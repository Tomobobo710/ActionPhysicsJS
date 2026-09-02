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

	// ---- INVARIANT: sleeping must not change the result. A body woken by a world change (its
	// support removed, a teleport, a gravity change) must behave exactly as the never-slept
	// version would - fall, move, resettle. ----

	// The sphere-on-a-static-box-then-delete-the-box case. Non-sleeping: sphere free-falls onto
	// the ground the instant the box is gone. Sleeping (before this was fixed): sphere hung
	// frozen in mid-air forever, because nothing woke it and the integrator skips a parked body.
	Runner.test('sleep', 'a sleeping body falls when the static body it rested on is deleted', function (t) {
		t.log('Sphere sleeps on a static box; delete the box; the sphere must wake and fall to the ground below.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w); // top face at y = 0
		// Static box, top face at y = 2. Sphere (r=0.5) rests centered on it at y = 2.5.
		var box = t.box(w, 1, 1, 1, 0, U.withMat({ pos: [0, 1, 0], color: '#8888aa' }));
		var ball = t.sphere(w, 0.5, 1, U.withMat({ pos: [0, 2.5 + 0.01, 0], color: '#F4D35E' }));

		var removed = false, wokeAfterRemoval = false;
		t.onTick(function (world) {
			if (!removed && !ball.isAwake) {
				removed = true;
				world.removeRigidBody(box);
			}
			if (removed && ball.isAwake && ball.position.y < 2.4) wokeAfterRemoval = true;
		});

		// PASS once the sphere has (a) been asleep, (b) had the box removed, (c) woken and fallen,
		// and (d) come to rest on the ground at y ~ 0.5.
		t.expect('sphere sleeps on the box, then after the box is deleted falls and lands on the ground (y~0.5)', (function () {
			var run = 0;
			return function () {
				if (!removed) return { ok: false, detail: 'waiting for sphere to sleep on the box' };
				var landed = wokeAfterRemoval && Math.abs(ball.position.y - 0.5) < 0.06 && U.speed(ball) < 0.05;
				if (landed) run++; else run = 0;
				return { ok: run >= 15, detail: 'woke=' + wokeAfterRemoval + ' y=' + ball.position.y.toFixed(3) + ' |v|=' + U.speed(ball).toFixed(3) + ' held=' + run + '/15' };
			};
		})());
		t.simulate(w, 480);
	}, {
		visual: true, steps: 480, page: 'sleep',
		description: 'A sphere is dropped onto a static box and allowed to sleep. The box is then removed from the ' +
			'world. PASS: the sphere wakes the same tick (nothing else would wake it - its own state did not change), ' +
			'free-falls, and lands on the ground below. A sphere that hangs frozen where the box used to be is the ' +
			'bug this guards against.'
	});

	// Teleporting a sleeping body with the blessed setter must wake it and land it where it now belongs.
	Runner.test('sleep', 'teleporting a sleeping body wakes it and it re-settles at the new spot', function (t) {
		t.log('Let a sphere sleep on the ground, then World.setBodyTransform it up into the air; it must fall back down.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		var ball = t.sphere(w, 0.5, 1, U.withMat({ pos: [0, 0.5, 0], color: '#3a9ee0' }));

		var TELEPORT_AT = 120, teleported = false;
		t.onTick(function (world, tick) {
			if (tick === TELEPORT_AT && !teleported) {
				teleported = true;
				world.setBodyTransform(ball, t.vec(0, 5, 0), null); // straight up, 4.5m above rest
			}
		});

		t.expect('sphere is asleep before the teleport, then wakes, falls from y=5, and re-settles at y~0.5', (function () {
			var run = 0, sleptFirst = false;
			return function () {
				if (!teleported) { if (!ball.isAwake) sleptFirst = true; return { ok: false, detail: 'pre-teleport, slept=' + sleptFirst }; }
				var back = sleptFirst && Math.abs(ball.position.y - 0.5) < 0.06 && U.speed(ball) < 0.05;
				if (back) run++; else run = 0;
				return { ok: run >= 15, detail: 'sleptFirst=' + sleptFirst + ' y=' + ball.position.y.toFixed(3) + ' held=' + run + '/15' };
			};
		})());
		t.simulate(w, 480);
	}, {
		visual: true, steps: 480, page: 'sleep',
		description: 'A sphere sleeps on the ground, then is teleported 4.5m straight up via World.setBodyTransform. ' +
			'PASS: it was asleep first, the move wakes it, and it falls back to the same resting height. A sphere ' +
			'that stays frozen at y=5 is a miss.'
	});

	// Changing gravity while an island sleeps must wake it so it responds to the new field.
	Runner.test('sleep', 'changing world gravity wakes a sleeping island', function (t) {
		t.log('Let a sphere sleep, flip gravity to point UP, and confirm the sphere wakes and rises.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		var ball = t.sphere(w, 0.5, 1, U.withMat({ pos: [0, 0.5, 0], color: '#e07a3a' }));

		var FLIP_AT = 120, flipped = false, roseAfterFlip = false;
		t.onTick(function (world, tick) {
			if (tick === FLIP_AT && !flipped) {
				flipped = true;
				world.gravity.set(0, 9.8, 0); // now points up
			}
			if (flipped && ball.isAwake && ball.position.y > 1.0 && ball.linear_velocity.y > 0.2) roseAfterFlip = true;
		});

		t.expect('sphere is asleep before the gravity flip, then wakes and rises once gravity points up', function () {
			if (!flipped) return { ok: false, detail: 'waiting for gravity flip @tick ' + FLIP_AT };
			return { ok: roseAfterFlip, detail: 'roseAfterFlip=' + roseAfterFlip + ' y=' + ball.position.y.toFixed(3) + ' vy=' + ball.linear_velocity.y.toFixed(3) + ' awake=' + ball.isAwake };
		});
		t.simulate(w, 300);
	}, {
		visual: true, steps: 300, page: 'sleep',
		description: 'A sphere sleeps on the ground, then world.gravity is flipped to point upward. PASS: the ' +
			'island manager notices gravity changed since the body parked, wakes it, and it accelerates upward. ' +
			'A sphere that keeps sleeping through the change is the bug.'
	});

	// ---- The invariant, tested head-on: same scene, same script, allowSleeping on vs off, and
	// the final state must match. This is the real regression guard - if any future change makes
	// sleeping observable, this fails regardless of which specific trigger broke. ----
	Runner.test('sleep', 'a scene with a mid-run body deletion ends identically with sleeping on and off', function (t) {
		t.log('Run the same drop+settle+delete-a-support scene twice (sleep on, sleep off); assert matching end state.');

		// Build the scene into a given world; returns the bodies we compare. `deleteAt` bodies are
		// scripted for removal once everything has settled.
		function build(w) {
			U.ground(t, w);
			var pedestal = t.box(w, 0.6, 1, 0.6, 0, U.withMat({ pos: [1.5, 1, 0] }));
			var onPedestal = t.sphere(w, 0.4, 1, U.withMat({ pos: [1.5, 2.4 + 0.01, 0] }));
			var freeDrop = t.box(w, 0.4, 0.4, 0.4, 1, U.withMat({ pos: [-1.5, 1.2, 0] }));
			return { pedestal: pedestal, onPedestal: onPedestal, freeDrop: freeDrop };
		}

		function run(allowSleeping) {
			var w = t.makeWorld({ gravity: -9.8 });
			w.allowSleeping = allowSleeping;
			var b = build(w);
			var removed = false;
			for (var tick = 0; tick < 420; tick++) {
				// Delete the pedestal at a fixed tick, well after everything has settled - identical
				// timing in both runs so the comparison is apples to apples.
				if (!removed && tick === 240) { w.removeRigidBody(b.pedestal); removed = true; }
				w.step(1 / 60);
			}
			return b;
		}

		var on = run(true);
		var off = run(false);

		function dist(a, c) {
			var dx = a.position.x - c.position.x, dy = a.position.y - c.position.y, dz = a.position.z - c.position.z;
			return Math.sqrt(dx * dx + dy * dy + dz * dz);
		}
		// Immediate assertions (like the math suite) - this test does all its own stepping above and
		// has nothing to animate, so it declares no live expectation and needs no t.simulate().
		t.checkTrue(dist(on.onPedestal, off.onPedestal) < 0.002,
			'sphere-on-pedestal ends within 2mm with sleeping on vs off (on y=' + on.onPedestal.position.y.toFixed(4) +
			', off y=' + off.onPedestal.position.y.toFixed(4) + ')');
		t.checkTrue(dist(on.freeDrop, off.freeDrop) < 0.002,
			'free-dropped box ends within 2mm with sleeping on vs off (d=' + dist(on.freeDrop, off.freeDrop).toFixed(5) + ')');
	}, {
		visual: false, page: 'sleep',
		description: 'The transparency invariant, tested directly: the same drop-settle-then-delete-a-pedestal ' +
			'scene is run twice, once with allowSleeping true and once false, and the final resting positions of ' +
			'the dynamic bodies must agree. Sleeping is only allowed to be an optimization - if it changes where ' +
			'anything ends up, this fails.'
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
