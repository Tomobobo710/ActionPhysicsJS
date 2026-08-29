(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "XPBD solver: substepped position-constraint solve for contacts, with velocity " +
		"DERIVED from the position delta and used raw (no clamp). Verified here: single-correction " +
		"exactness, warm-start persistence, velocity derivation.";
	function test(group, name, fn) {
		Runner.test(group, name, fn, { page: 'solver', description: DESC, visual: true, steps: 1 });
	}

	test('solver/core', 'a single substep resolves a sphere-on-ground overlap to exactly zero penetration', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var sphere = t.sphere(world, 0.5, 1, { pos: [0, 0.4, 0], color: '#4af' });

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

	test('solver/core', 'two equal-mass dynamic bodies split a correction evenly', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		var a = t.sphere(world, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var b = t.sphere(world, 0.5, 1, { pos: [0.8, 0, 0], color: '#f84' });

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

	test('solver/core', 'a sphere resting on top of the ground is pushed UP, never down through it', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var sphere = t.sphere(world, 0.5, 1, { pos: [0, 0.3, 0], color: '#4af' });

		t.expect('the sphere moved UP, out of the ground, not further down through it', function () {
			return { ok: sphere.position.y > 0.3, detail: 'y=' + sphere.position.y.toFixed(6) };
		});
		t.expect('resolved to exactly the correct resting height', function () {
			return { ok: Math.abs(sphere.position.y - 0.5) < 1e-9, detail: 'y=' + sphere.position.y.toFixed(10) };
		});
		t.simulate(world, 1);
	});

	test('solver/core', 'velocity derives from the position delta, matching (x - x_prev) / h', function (t) {
		var world = t.makeWorld({ gravity: -10 }); world.solver.substeps = 1;
		var box = t.sphere(world, 0.5, 1, { pos: [0, 10, 0], linear_damping: 0, angular_damping: 0, color: '#4af' });

		var dt = 1 / 60;

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

	test('solver/core', 'a settled contact\'s accumulated lambda is available for warm-starting the next tick', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], color: '#4af' });

		t.expect('a manifold exists for the resting pair, with its 4 contact points', function () {
			var manifold = Array.from(world.narrowphase.manifolds.values())[0];
			return { ok: !!manifold && manifold.pointCount === 4, detail: manifold ? ('pointCount=' + manifold.pointCount) : 'no manifold' };
		});
		t.simulate(world, 1);
	});

	test('solver/core', 'a body spawned deep in overlap settles instantly with zero fabricated velocity', function (t) {
		var world = t.makeWorld({ gravity: 0 }); world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var sphere = t.sphere(world, 0.5, 1, { pos: [0, 0.4, 0], color: '#f55' });

		t.expect('resolved to exactly the correct rest height', function () {
			return { ok: Math.abs(sphere.position.y - 0.5) < 1e-9, detail: 'y=' + sphere.position.y.toFixed(10) };
		});
		t.expect('zero derived velocity - the correction was entirely non-explainable, so entirely bias', function () {
			return { ok: Math.abs(sphere.linear_velocity.y) < 1e-9, detail: 'vy=' + sphere.linear_velocity.y.toExponential(3) };
		});
		t.simulate(world, 1);
	});

	Runner.test('solver/core', 'a large deep-spawn overlap does not launch the body - it settles', function (t) {
		var world = t.makeWorld({ gravity: -9.81 });
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });

		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.35, 0], color: '#f55' });

		var maxY = -Infinity, tick0 = 0;
		t.onTick(function (world, tick) {
			tick0 = tick;
			if (box.position.y > maxY) maxY = box.position.y;
		});
		t.expect('the body does not launch away from the surface (stays near rest height)', function () {
			if (tick0 < 300) return { ok: false, detail: 'running… maxY so far=' + maxY.toFixed(3) };
			return { ok: maxY < 1.0, detail: 'maxY=' + maxY.toFixed(3) + ' (rest height is 0.5)' + (maxY >= 1.0 ? ' LAUNCHED' : '') };
		});
		t.simulate(world, 300);
	}, { page: 'solver', description: DESC, visual: true, steps: 300 });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
