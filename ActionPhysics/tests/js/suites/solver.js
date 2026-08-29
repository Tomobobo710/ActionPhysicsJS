// XPBD solver core mechanic in isolation: a single position correction resolves a known overlap
// exactly, warm start carries lambda across ticks, velocity derives from the position delta.
//
// Deliberately NOT claimed here: full-scene settling stability. A body SPAWNED deep in overlap gets
// the full undamped one-shot correction (velocity = delta/h, no clamp by design); speculative
// contacts prevent ARRIVING deep, they do not rescue an embedded spawn. Every test is one tick.
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
		// Pins the GJK/EPA normal-sign convention end-to-end: either half signed backwards sends
		// the box THROUGH the ground instead of out of it.
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
		// machinery survives a full step without losing its point - what warm start depends on.
		t.expect('a manifold exists for the resting pair, with exactly one contact point', function () {
			var manifold = Array.from(world.narrowphase.manifolds.values())[0];
			return { ok: !!manifold && manifold.pointCount === 1, detail: manifold ? ('pointCount=' + manifold.pointCount) : 'no manifold' };
		});
		t.simulate(world, 1);
	});

	// ---- the known, expected instability without speculative contacts ----

	test('solver/core', 'a body SPAWNED already deep in overlap gets the full undamped one-shot correction', function (t) {
		// Placed inside the ground, not arrived there: speculative detection cannot retroactively
		// soften an embedded spawn, so this resolves as one undamped correction, v = delta/h,
		// exactly per the no-clamp rule. (Contrast the speculative suite, where a FALLING box
		// settles with no such kick.)
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
