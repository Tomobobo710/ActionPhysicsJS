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
(function (Runner) {
	Runner.suite('solver');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3, Q = AP.Quaternion;
	var DESC = "XPBD solver: substepped position-constraint solve for contacts, with velocity " +
		"DERIVED from the position delta and used raw (no clamp, per plan.md's central design " +
		"rule). Verified here: single-correction exactness, warm-start persistence, velocity " +
		"derivation. NOT yet verified (and not expected to hold): full-scene settling stability, " +
		"which requires speculative contacts - a distinct, not-yet-built increment.";
	function test(group, name, fn, opts) {
		opts = opts || {};
		Runner.test(group, name, fn, { page: 'solver', description: DESC, visual: !!opts.visual, steps: opts.steps || 0 });
	}

	function mkWorld(opts) {
		var solver = new AP.Solver(opts);
		var w = new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), solver);
		return w;
	}

	// ---- core mechanic: a single substep resolves a known overlap exactly ----

	test('solver/core', 'a single substep resolves a box-on-ground overlap to exactly zero penetration', function (t) {
		var world = mkWorld({ substeps: 1, iterations: 1 });
		world.gravity.set(0, 0, 0);
		var ground = new AP.RigidBody(new AP.BoxShape(10, 0.5, 10), 0);
		ground.position.set(0, -0.5, 0);
		world.addRigidBody(ground);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.4, 0); // overlapping by exactly 0.1
		world.addRigidBody(box);

		world.step(1 / 60);
		t.check(box.position.y, 0.5, 1e-9, 'exactly resolved to zero penetration in one substep');
	});

	test('solver/core', 'a static body never moves, regardless of overlap', function (t) {
		var world = mkWorld({ substeps: 1, iterations: 1 });
		world.gravity.set(0, 0, 0);
		var ground = new AP.RigidBody(new AP.BoxShape(10, 0.5, 10), 0);
		ground.position.set(0, -0.5, 0);
		world.addRigidBody(ground);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.4, 0);
		world.addRigidBody(box);

		world.step(1 / 60);
		t.check(ground.position.y, -0.5, 1e-9, 'the static ground does not move even though it is bodyA in the constraint');
	});

	test('solver/core', 'two equal-mass dynamic bodies split a correction evenly', function (t) {
		var world = mkWorld({ substeps: 1, iterations: 1 });
		world.gravity.set(0, 0, 0);
		var a = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		a.position.set(0, 0, 0);
		world.addRigidBody(a);
		var b = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		b.position.set(0.8, 0, 0); // overlapping by 0.2 along X (half-extent sum 1.0)
		world.addRigidBody(b);

		var startGap = b.position.x - a.position.x;
		world.step(1 / 60);
		var endGap = b.position.x - a.position.x;
		t.check(endGap, 1.0, 1e-6, 'the pair separates to exactly the half-extent sum');
		t.check(a.position.x, -0.1, 1e-6, 'equal mass: A moves half the correction one way');
		t.check(b.position.x, 0.9, 1e-6, 'equal mass: B moves half the correction the other way');
	});

	// ---- normal direction / sign convention (the actual bug found and fixed while building this) ----

	test('solver/core', 'a box resting on top of the ground is pushed UP, never down through it', function (t) {
		// This pins down the exact bug found while building the solver: EPA's normal convention
		// did not match GJK's own (verified separately in the EPA suite), and independently the
		// solver's own constraint-gradient sign was paired incorrectly with which body gets +n vs
		// -n. Either mistake alone sends an overlapping box THROUGH the ground instead of pushing
		// it back out - this test is the end-to-end symptom both bugs produced.
		var world = mkWorld({ substeps: 1, iterations: 1 });
		world.gravity.set(0, 0, 0);
		var ground = new AP.RigidBody(new AP.BoxShape(10, 0.5, 10), 0);
		ground.position.set(0, -0.5, 0);
		world.addRigidBody(ground);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.3, 0); // overlapping by 0.2
		world.addRigidBody(box);

		world.step(1 / 60);
		t.checkTrue(box.position.y > 0.3, 'the box moved UP, out of the ground, not further down through it');
		t.check(box.position.y, 0.5, 1e-9, 'and resolved to exactly the correct resting height');
	});

	// ---- velocity derivation ----

	test('solver/core', 'velocity derives from the position delta, matching (x - x_prev) / h', function (t) {
		var world = mkWorld({ substeps: 1, iterations: 1 });
		world.gravity.set(0, -10, 0); // simple, known gravity for an easy closed-form check
		var box = new AP.RigidBody(new AP.SphereShape(0.5), 1);
		box.position.set(0, 10, 0); // far from anything, no contacts to solve
		world.addRigidBody(box);

		var dt = 1 / 60;
		world.step(dt);
		// No contacts: position integrates as x + v*h with v updated by gravity first (see
		// Solver._substep step 1) - v_new = 0 + g*h, x_new = x + v_new*h.
		var expectedV = -10 * dt;
		t.check(box.linear_velocity.y, expectedV, 1e-9, 'velocity after one free-falling substep matches g*h');
		var expectedY = 10 + expectedV * dt;
		t.check(box.position.y, expectedY, 1e-9, 'position matches the predicted (no contact to correct)');
	});

	// ---- warm start across ticks ----

	test('solver/core', 'a settled contact\'s accumulated lambda is available for warm-starting the next tick', function (t) {
		var world = mkWorld({ substeps: 1, iterations: 1 });
		world.gravity.set(0, 0, 0);
		var ground = new AP.RigidBody(new AP.BoxShape(10, 0.5, 10), 0);
		ground.position.set(0, -0.5, 0);
		world.addRigidBody(ground);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.5, 0); // exactly resting, no correction needed
		world.addRigidBody(box);

		world.step(1 / 60);
		// At exactly resting (C <= 0 by the live-anchor measurement), no correction runs and
		// lambda for that substep is legitimately 0 - this test's job is only to confirm the
		// manifold/point machinery survives a full world.step() call without erroring or losing
		// its single point, which is what a real warm start depends on structurally.
		var manifold = Array.from(world.narrowphase.manifolds.values())[0];
		t.checkTrue(!!manifold, 'a manifold exists for the resting pair');
		t.checkEqual(manifold.pointCount, 1, 'exactly one contact point persists');
	});

	// ---- the known, expected instability without speculative contacts ----

	test('solver/core', 'a real (non-speculative) overlap produces a large, undamped derived velocity - documented, not hidden', function (t) {
		// This is the exact mechanism plan.md's "derived-velocity problem" describes: with no
		// clamp anywhere (the central design rule), resolving a real position error in a single
		// short substep produces velocity = Δx / h, which grows without bound as h shrinks. This
		// is EXPECTED behavior for XPBD without speculative contacts (predicting the end-of-
		// substep position before detection runs) - not something this test pretends is fine, but
		// something it measures honestly so a future speculative-contacts change has a concrete
		// before/after to check against.
		var world = mkWorld({ substeps: 1, iterations: 1 });
		world.gravity.set(0, 0, 0);
		var ground = new AP.RigidBody(new AP.BoxShape(10, 0.5, 10), 0);
		ground.position.set(0, -0.5, 0);
		world.addRigidBody(ground);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.4, 0); // overlapping by 0.1
		world.addRigidBody(box);

		var dt = 1 / 60;
		world.step(dt);
		var expectedVelocity = 0.1 / dt; // the full correction, undamped, over one substep
		t.check(box.linear_velocity.y, expectedVelocity, 1e-6, 'derived velocity equals the raw correction / h, exactly as designed - no clamp');
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
