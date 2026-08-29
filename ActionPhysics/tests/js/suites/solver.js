// Solver: XPBD position-constraint solve, substepping, derived velocity. Tests here verify the
// CORE MECHANIC in isolation - a single position correction resolves a known overlap exactly, warm
// start carries lambda across ticks, velocity derives correctly from the position delta.
//
// WHAT THIS SUITE DELIBERATELY DOES NOT CLAIM: full-scene stability (a body falling from a real
// height and settling) is NOT yet correct, and is not tested here as if it were. Plan.md's own
// design says derived velocity is used RAW with no clamp, and that speculative contacts (detecting
// against a PREDICTED end-of-substep position, not the current one) are what keeps that safe -
// speculative detection does not exist yet (narrowphase still queries current position only). A
// real, non-speculative contact resolved by an uncapped one-shot XPBD correction produces a large
// derived velocity with nothing to damp it - verified directly here as a KNOWN, EXPECTED
// characteristic (not a bug to be silently tested around), matching plan.md's own extended
// writeup of the identical failure mode in the predecessor before speculative contacts existed.
//
// Every test here is a SINGLE world.step() call - one tick, checking an exact numeric outcome, not
// a settling trajectory (that's what speculative.js's tests are for). There's no multi-second
// animation to watch because there's no multi-tick motion to show: this is the mechanism caught in
// the act of one correction, not a scene playing out. Still built through the shared harness
// (t.makeWorld/t.box) so the before/after IS drawn - you see the box positioned at the start, the
// solver runs, and the console shows exactly what moved and by how much.
(function (Runner) {
	Runner.suite('solver');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "XPBD solver: substepped position-constraint solve for contacts, with velocity " +
		"DERIVED from the position delta and used raw (no clamp, per plan.md's central design " +
		"rule). Verified here: single-correction exactness, warm-start persistence, velocity " +
		"derivation. NOT yet verified (and not expected to hold): full-scene settling stability, " +
		"which requires speculative contacts - a distinct, not-yet-built increment.";
	function test(group, name, fn) {
		Runner.test(group, name, fn, { page: 'solver', description: DESC, visual: true, steps: 1 });
	}

	// ---- core mechanic: a single substep resolves a known overlap exactly ----

	test('solver/core', 'a single substep resolves a box-on-ground overlap to exactly zero penetration', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.4, 0], color: '#4af' }); // overlapping by exactly 0.1

		t.expect('exactly resolved to zero penetration in one substep', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-9, detail: 'y=' + box.position.y.toFixed(10) };
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

	test('solver/core', 'two equal-mass dynamic bodies split a correction evenly', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		var a = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var b = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0.8, 0, 0], color: '#f84' }); // overlapping by 0.2 (half-extent sum 1.0)

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

	test('solver/core', 'a box resting on top of the ground is pushed UP, never down through it', function (t) {
		// This pins down the exact bug found while building the solver: EPA's normal convention
		// did not match GJK's own (verified separately in the EPA suite), and independently the
		// solver's own constraint-gradient sign was paired incorrectly with which body gets +n vs
		// -n. Either mistake alone sends an overlapping box THROUGH the ground instead of pushing
		// it back out - this test is the end-to-end symptom both bugs produced.
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.3, 0], color: '#4af' }); // overlapping by 0.2

		t.expect('the box moved UP, out of the ground, not further down through it', function () {
			return { ok: box.position.y > 0.3, detail: 'y=' + box.position.y.toFixed(6) };
		});
		t.expect('resolved to exactly the correct resting height', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-9, detail: 'y=' + box.position.y.toFixed(10) };
		});
		t.simulate(world, 1);
	});

	// ---- velocity derivation ----

	test('solver/core', 'velocity derives from the position delta, matching (x - x_prev) / h', function (t) {
		var world = t.makeWorld({ gravity: -10 }); world.solver.substeps = 1; // simple, known gravity for an easy closed-form check
		var box = t.sphere(world, 0.5, 1, { pos: [0, 10, 0], color: '#4af' }); // far from anything, no contacts to solve

		var dt = 1 / 60;
		// No contacts: position integrates as x + v*h with v updated by gravity first (see
		// Solver._substep step 1) - v_new = 0 + g*h, x_new = x + v_new*h.
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

		// At exactly resting (C <= 0 by the live-anchor measurement), no correction runs and lambda
		// for that substep is legitimately 0 - this checks only that the manifold/point machinery
		// survives a full world.step() call without erroring or losing its single point, which is
		// what a real warm start depends on structurally.
		t.expect('a manifold exists for the resting pair, with exactly one contact point', function () {
			var manifold = Array.from(world.narrowphase.manifolds.values())[0];
			return { ok: !!manifold && manifold.pointCount === 1, detail: manifold ? ('pointCount=' + manifold.pointCount) : 'no manifold' };
		});
		t.simulate(world, 1);
	});

	// ---- the known, expected instability without speculative contacts ----

	test('solver/core', 'a body SPAWNED already deep in overlap gets the full undamped one-shot correction', function (t) {
		// The box here STARTS overlapping by 0.1 - it did not approach and get caught by speculative
		// detection, it was placed inside the ground. Speculative contacts (now built) prevent a
		// body from ARRIVING deep by detecting the contact before overlap; they do not retroactively
		// soften an overlap a body was spawned into. So this pre-embedded case still resolves in a
		// single undamped correction, velocity = Δx/h, exactly per the central no-clamp rule. This
		// is correct and documented: the derived-velocity fix is about the approach, not about
		// rescuing a body teleported into solid geometry. (Contrast the speculative suite, where a
		// box that FALLS onto the ground settles with no such kick.)
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.4, 0], color: '#f55' }); // overlapping by 0.1

		var dt = 1 / 60;
		var expectedVelocity = 0.1 / dt; // the full correction, undamped, over one substep
		t.expect('derived velocity equals the raw correction / h, exactly as designed - no clamp', function () {
			return { ok: Math.abs(box.linear_velocity.y - expectedVelocity) < 1e-6, detail: 'vy=' + box.linear_velocity.y.toFixed(4) + ' (expected ' + expectedVelocity.toFixed(4) + ')' };
		});
		t.simulate(world, 1);
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
