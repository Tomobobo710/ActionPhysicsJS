// XPBD solver core mechanic in isolation: a single position correction resolves a known overlap
// exactly, warm start carries lambda across ticks, velocity derives from the position delta - split
// by how much of it the body's own real velocity can explain (see Solver.js's own header). A body
// SPAWNED deep in overlap, with zero real velocity behind it, settles instantly with zero derived
// velocity rather than launching. Every test is one tick unless noted otherwise.
(function (Runner) {
	Runner.suite('solver');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "XPBD solver: substepped position-constraint solve for contacts, with velocity " +
		"DERIVED from the position delta and used raw (no clamp). Verified here: single-correction " +
		"exactness, warm-start persistence, velocity derivation.";
	function test(group, name, fn) {
		Runner.test(group, name, fn, { page: 'solver', description: DESC, visual: true, steps: 1 });
	}

	// ---- core mechanic: a single substep resolves a known overlap exactly ----

	// Sphere-on-box, not box-on-box: this pins the SOLVER's one-shot exactness for a single contact
	// point. A box on a much bigger box is a genuine 4-point face manifold (BoxBox's own SAT+clip
	// contact), and 4 coincident-depth points solved Gauss-Seidel in one unconverged substep
	// (iterations=1) legitimately overshoot slightly - each point's correction is computed against
	// the position the previous point already moved, a well-known property of multi-point Gauss-
	// Seidel position solving, not a bug. A sphere always contacts a box at exactly one point, so it
	// isolates the single-point mechanic this test actually means to check.
	test('solver/core', 'a single substep resolves a sphere-on-ground overlap to exactly zero penetration', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var sphere = t.sphere(world, 0.5, 1, { pos: [0, 0.4, 0], color: '#4af' }); // overlapping by exactly 0.1

		t.expect('exactly resolved to zero penetration in one substep', function () {
			return { ok: Math.abs(sphere.position.y - 0.5) < 1e-9, detail: 'y=' + sphere.position.y.toFixed(10) };
		});
		t.simulate(world, 1);
	});

	test('solver/core', 'a static body never moves, regardless of overlap', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		var ground = t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.4, 0], color: '#4af' });

		t.expect('the static ground does not move even though it is bodyA in the constraint', function () {
			return { ok: Math.abs(ground.position.y - (-0.5)) < 1e-9, detail: 'y=' + ground.position.y.toFixed(10) };
		});
		t.simulate(world, 1);
	});

	// Two spheres, not two boxes: two axis-aligned equal boxes overlapping face-to-face is itself a
	// 4-point BoxBox manifold (see the sphere-on-ground test above for why that legitimately
	// overshoots a single unconverged Gauss-Seidel substep) - two spheres always contact at exactly
	// one point, isolating the single-point equal-mass split this test means to check.
	test('solver/core', 'two equal-mass dynamic bodies split a correction evenly', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		var a = t.sphere(world, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var b = t.sphere(world, 0.5, 1, { pos: [0.8, 0, 0], color: '#f84' }); // overlapping by 0.2 (radius sum 1.0)

		t.expect('the pair separates to exactly the half-extent sum', function () {
			var gap = b.position.x - a.position.x;
			return { ok: Math.abs(gap - 1.0) < 1e-6, detail: 'gap=' + gap.toFixed(7) };
		});
		t.expect('equal mass: A moves half the correction one way', function () {
			return { ok: Math.abs(a.position.x - (-0.1)) < 1e-6, detail: 'x=' + a.position.x.toFixed(7) };
		});
		t.expect('equal mass: B moves half the correction the other way', function () {
			return { ok: Math.abs(b.position.x - 0.9) < 1e-6, detail: 'x=' + b.position.x.toFixed(7) };
		});
		t.simulate(world, 1);
	});

	// ---- normal direction / sign convention (the actual bug found and fixed while building this) ----

	// Sphere-on-box, not box-on-box: same single-point rationale as the exactness test above - this
	// one pins the normal-sign convention end-to-end (either half signed backwards sends the body
	// THROUGH the ground), which needs exactly one contact point to check exactly, not BoxBox's
	// legitimate 4-point overshoot.
	test('solver/core', 'a sphere resting on top of the ground is pushed UP, never down through it', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var sphere = t.sphere(world, 0.5, 1, { pos: [0, 0.3, 0], color: '#4af' }); // overlapping by 0.2

		t.expect('the sphere moved UP, out of the ground, not further down through it', function () {
			return { ok: sphere.position.y > 0.3, detail: 'y=' + sphere.position.y.toFixed(6) };
		});
		t.expect('resolved to exactly the correct resting height', function () {
			return { ok: Math.abs(sphere.position.y - 0.5) < 1e-9, detail: 'y=' + sphere.position.y.toFixed(10) };
		});
		t.simulate(world, 1);
	});

	// ---- velocity derivation ----

	test('solver/core', 'velocity derives from the position delta, matching (x - x_prev) / h', function (t) {
		var world = t.makeWorld({ gravity: -10 }); world.solver.substeps = 1; // simple, known gravity for an easy closed-form check
		var box = t.sphere(world, 0.5, 1, { pos: [0, 10, 0], linear_damping: 0, angular_damping: 0, color: '#4af' }); // far from anything, no contacts to solve

		var dt = 1 / 60;
		// No contacts: v_new = v_old + g*h first, then x_new = x + v_new*h (Solver._substep order).
		var expectedV = -10 * dt;
		var expectedY = 10 + expectedV * dt;
		t.expect('velocity after one free-falling substep matches g*h', function () {
			return { ok: Math.abs(box.linear_velocity.y - expectedV) < 1e-9, detail: 'vy=' + box.linear_velocity.y.toFixed(10) };
		});
		t.expect('position matches the predicted (no contact to correct)', function () {
			return { ok: Math.abs(box.position.y - expectedY) < 1e-9, detail: 'y=' + box.position.y.toFixed(10) };
		});
		t.simulate(world, 1);
	});

	// ---- warm start across ticks ----

	test('solver/core', 'a settled contact\'s accumulated lambda is available for warm-starting the next tick', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], color: '#4af' }); // exactly resting, no correction needed

		// At exactly resting height C <= 0, so no correction runs; this checks only that manifold
		// machinery survives a full step without losing its points - what warm start depends on.
		// A flat box resting on a much bigger box is a genuine 4-point face contact (BoxBox's own
		// SAT+clip manifold), not the single GJK/EPA point this pair used to fall through to.
		t.expect('a manifold exists for the resting pair, with its 4 contact points', function () {
			var manifold = Array.from(world.narrowphase.manifolds.values())[0];
			return { ok: !!manifold && manifold.pointCount === 4, detail: manifold ? ('pointCount=' + manifold.pointCount) : 'no manifold' };
		});
		t.simulate(world, 1);
	});

	// ---- deep-spawn overlap: velocity is only what the body's own real motion can explain ----

	// PRIOR BEHAVIOR (no longer correct - kept only as history): a body constructed already deep in
	// overlap got the ENTIRE overlap corrected in one undamped substep, producing derived velocity =
	// correction/h with no bound - a real, confirmed bug (see the settling test below: this exact
	// setup used to launch the body to y=53+ under gravity). THE FIX: _solvePoint splits each
	// correction by how much of it the body's own real, already-measured closing velocity could
	// explain - a spawn/teleport overlap has ZERO real velocity behind it, so almost none of a large
	// correction is recognized as explainable, and it resolves as a pure position edit instead of
	// becoming derived velocity. See Solver.js's own header for the full reasoning and the two
	// narrower approaches (a flat magnitude cap, a fixed Baumgarte fraction) that were tried first and
	// failed for different reasons.
	// Sphere-on-box, not box-on-box: same single-point rationale as the exactness test above.
	test('solver/core', 'a body spawned deep in overlap settles instantly with zero fabricated velocity', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var sphere = t.sphere(world, 0.5, 1, { pos: [0, 0.4, 0], color: '#f55' }); // overlapping by 0.1, zero real velocity behind it

		t.expect('resolved to exactly the correct rest height', function () {
			return { ok: Math.abs(sphere.position.y - 0.5) < 1e-9, detail: 'y=' + sphere.position.y.toFixed(10) };
		});
		t.expect('zero derived velocity - the correction was entirely non-explainable, so entirely bias', function () {
			return { ok: Math.abs(sphere.linear_velocity.y) < 1e-9, detail: 'vy=' + sphere.linear_velocity.y.toExponential(3) };
		});
		t.simulate(world, 1);
	});

	// ---- deep-spawn overlap: does the one-shot correction LAUNCH the body, or settle it? ----
	// Flat ground, no rotation, nothing incidental - isolates exactly one thing: a body spawned
	// significantly overlapping another gets ONE undamped position correction (see the small-overlap
	// test above), and that correction becomes derived velocity with no bound on its size. A large
	// enough spawn depth turns that into a real launch, not a bounce: traced directly on a box
	// spawned 0.15m into a rotated ramp, this produced vy=26 m/s on the very first tick and the box
	// was still climbing (y=13+, still rising) 40 ticks later under normal gravity - it does not fall
	// back down in any reasonable window, it leaves. This test reproduces the same depth on FLAT
	// ground (no ramp, no rotation) to isolate the mechanism from any ramp-specific detail, and
	// asserts the physically sane outcome directly: the body should settle at rest on the surface,
	// not be thrown away from it.
	// Registered directly (not via this file's own `test()` helper, which hardcodes steps:1 for the
	// single-tick checks above) - this one needs the full window watchable, not just frame 1.
	Runner.test('solver/core', 'a large deep-spawn overlap does not launch the body - it settles', function (t) {
		var world = t.makeWorld({ gravity: -9.81 });
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		// Overlapping by 0.15 - the same depth traced from the real ramp-spawn bug this isolates.
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.35, 0], color: '#f55' });

		// t.expect LATCHES the first tick its predicate reads true and never re-checks (see
		// tests/js/runner.js's evalTick: `if (ex.met) continue`) - correct for "eventually becomes
		// true and STAYS true" (a settle), wrong for "must hold the WHOLE run", which reads trivially
		// true at tick 1 before anything has happened. maxY tracked across every tick, but the
		// predicate itself only reports once the run has actually finished (tick0 >= total) - the
		// same finalGate pattern already established elsewhere in this suite for exactly this class
		// of bug (see tests/js/tom/fps-knockback.js's own finalGate, which exists because this
		// project has directly caught assertions that read green on tick 1 while the real behavior
		// they were supposed to catch happened later in the run).
		var maxY = -Infinity, tick0 = 0;
		t.onTick(function (world, tick) {
			tick0 = tick;
			if (box.position.y > maxY) maxY = box.position.y;
		});
		t.expect('the body does not launch away from the surface (stays near rest height)', function () {
			if (tick0 < 300) return { ok: false, detail: 'running… maxY so far=' + maxY.toFixed(3) };
			// Rest height is 0.5 (box half-extent 0.5 on top of ground half-extent 0.5). A real launch
			// from this depth measured vy=35.8 on tick 1 and y=6.17 by tick 10, still rising - 1.0 is
			// a generous ceiling a correct settle never approaches, while a launch blows through it
			// almost immediately.
			return { ok: maxY < 1.0, detail: 'maxY=' + maxY.toFixed(3) + ' (rest height is 0.5)' + (maxY >= 1.0 ? ' LAUNCHED' : '') };
		});
		t.simulate(world, 300);
	}, { page: 'solver', description: DESC, visual: true, steps: 300 });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
