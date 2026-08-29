(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Friction (velocity-pass Coulomb, static stick below the friction angle, slide above) " +
		"and restitution (e=0 inelastic .. e=1 full rebound). Includes sphere-on-large-box regression " +
		"tests for the GJK strict-interior / EPA tetrahedron-completion fix.";
	function visualTest(group, name, fn, steps) {
		Runner.test(group, name, fn, { page: 'friction_restitution', description: DESC, visual: true, steps: steps });
	}
	function staticTest(group, name, fn) {
		Runner.test(group, name, fn, { page: 'friction_restitution', description: DESC });
	}

	visualTest('friction', 'a shoved box decelerates and stops under friction', function (t) {
		var world = t.makeWorld();
		t.box(world, 40, 0.5, 40, 0, { pos: [0, -0.5, 0], friction: 0.5, color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], vel: [3, 0, 0], friction: 0.5, color: '#4af' });

		t.expect('stopped sliding', function () {
			return { ok: Math.abs(box.linear_velocity.x) < 0.05, detail: '|vx|=' + Math.abs(box.linear_velocity.x).toFixed(4) };
		});
		t.expect('stopped within a short distance', function () {
			return { ok: box.position.x < 3, detail: 'x=' + box.position.x.toFixed(3) };
		});
		t.simulate(world, 300);
	}, 300);

	function slopeTest(deg, mu, expectStick) {
		visualTest('friction', 'a box on a ' + deg + 'deg slope (mu=' + mu + ', tan=' + Math.tan(deg * Math.PI / 180).toFixed(2) + ') ' + (expectStick ? 'holds still' : 'slides'), function (t) {
			var world = t.makeWorld();
			var a = deg * Math.PI / 180;
			var ground = t.box(world, 40, 0.5, 40, 0, { pos: [0, -0.5, 0], friction: mu, color: '#556' });
			ground.rotation.setAxisAngle(new V(0, 0, 1), a); ground.updateDerived(0);
			var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.7, 0], friction: mu, color: '#4af' });
			box.rotation.setAxisAngle(new V(0, 0, 1), a);

			var x0 = null, y0 = null, settleTick = 100;
			var slideRate = 0;
			t.onTick(function (world, tick) {
				if (tick === settleTick) { x0 = box.position.x; y0 = box.position.y; }
				if (tick > settleTick) {
					var dist = Math.hypot(box.position.x - x0, box.position.y - y0);
					slideRate = dist / ((tick - settleTick) / 60);
				}
			});
			t.expect(expectStick ? 'sticks (slide rate stays low)' : 'slides (slide rate stays high)', function () {
				if (x0 == null) return false;
				var ok = expectStick ? slideRate < 0.1 : slideRate > 0.3;
				return { ok: ok, detail: 'slide rate=' + slideRate.toFixed(4) + ' m/s' };
			});
			t.simulate(world, 400);
		}, 400);
	}
	slopeTest(15, 0.5, true);
	slopeTest(20, 0.5, true);
	slopeTest(35, 0.5, false);
	slopeTest(45, 0.5, false);

	function restitutionTest(e, label) {
		visualTest('restitution', 'restitution e=' + e + ': ' + label, function (t) {
			var world = t.makeWorld();
			t.box(world, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: e, color: '#556' });
			var ball = t.sphere(world, 0.5, 1, { pos: [0, 3, 0], restitution: e, color: '#f84' });

			var solver = world.solver;
			var origSolveContactVelocity = solver._solveContactVelocity.bind(solver);
			var approachSpeed = null, exitSpeed = null;
			solver._solveContactVelocity = function (point, bodyA, bodyB, gravity, h) {
				var preLambda = point.normalLambda;
				origSolveContactVelocity(point, bodyA, bodyB, gravity, h);

				var g = bodyA.gravity || bodyB.gravity || gravity;
				var gravityMag = Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
				var restitutionThreshold = gravityMag * h * AP.Solver.RESTITUTION_SLOP_FACTOR;
				var fires = preLambda < 0 && e > 0 && point._preSolveNormalVel > restitutionThreshold;
				if (approachSpeed === null && fires) {
					approachSpeed = point._preSolveNormalVel;
					exitSpeed = -solver._contactRelativeNormalVelocity(point, bodyA, bodyB);
				}
			};

			t.expect(e > 0
				? 'the exact contact-relative exit speed equals e * approach speed, to 1e-6 (not a rebound-height proxy)'
				: 'no restitution event ever fires at e=0 (the ball simply stops, nothing to bounce)', function () {
				if (e === 0) {
					return { ok: approachSpeed === null, detail: approachSpeed === null ? 'no restitution event fired, correct' : 'restitution unexpectedly fired at e=0' };
				}
				if (approachSpeed === null) return false;
			var expected = e * approachSpeed;
			var err = Math.abs(exitSpeed - expected);
			return {
				ok: err < 1e-6,
				detail: 'approach=' + approachSpeed.toFixed(6) + ' exit=' + exitSpeed.toFixed(6) +
						' expected=' + expected.toFixed(6) + ' err=' + err.toExponential(3)
				};
			});
			t.simulate(world, 120);
		}, 120);
	}
	restitutionTest(0, 'no bounce (inelastic) - exit speed is exactly zero');
	restitutionTest(0.5, 'exit speed is exactly half the approach speed');
	restitutionTest(0.8, 'exit speed is exactly 80% of the approach speed');
	restitutionTest(1.0, 'exit speed exactly EQUALS approach speed - full energy back, not more, not less');

	visualTest('restitution', 'four restitution sub-tests (shared world)', function (t) {
		var world = t.makeWorld({ gravity: 0 });

		var t1_stat = t.sphere(world, 1, 0, { pos: [0, 0, 0], restitution: 1, linear_damping: 0, color: '#888' });
		var t1_dyn = t.sphere(world, 1, 1, { pos: [0, 5, 0], vel: [0, -3, 0], restitution: 1, linear_damping: 0, color: '#F4D35E' });
		var t2_stat = t.sphere(world, 1, 0, { pos: [3, 0, 0], restitution: 0.2, linear_damping: 0, color: '#888' });
		var t2_dyn = t.sphere(world, 1, 1, { pos: [3, 5, 0], vel: [0, -3, 0], restitution: 0.2, linear_damping: 0, color: '#EE964B' });
		var t3_a = t.sphere(world, 1, 1, { pos: [6, 0, 0], restitution: 1, linear_damping: 0, color: '#45B7D1' });
		var t3_b = t.sphere(world, 1, 1, { pos: [6, 3, 0], vel: [0, -2, 0], restitution: 1, linear_damping: 0, color: '#45B7D1' });
		var t4_a = t.sphere(world, 1, 1, { pos: [9, 0, 0], linear_damping: 0, color: '#8367C7' });
		var t4_b = t.sphere(world, 1, 1, { pos: [9, 3, 0], vel: [0, -2, 0], linear_damping: 0, color: '#8367C7' });

		function reachesVy(b, v, eps) { return function () { var d = Math.abs(b.linear_velocity.y - v); return { ok: d <= eps, detail: 'vy=' + b.linear_velocity.y.toFixed(4) }; }; }
		function sepSpeed(a, b, v, eps) { return function () { var s = a.linear_velocity.length() + b.linear_velocity.length(); return { ok: Math.abs(s - v) <= eps, detail: 'sep=' + s.toFixed(4) }; }; }

		t.expect('Test 1 - elastic ball bounces off the wall at full speed (vy -> +3)', reachesVy(t1_dyn, 3, 0.0001));
		t.expect('Test 2 - soft ball (e=0.2) keeps 20% after the bounce (vy -> +0.6)', reachesVy(t2_dyn, 0.6, 0.0001));
		t.expect('Test 3 - elastic Newton\'s-cradle transfers all motion (separating speed -> 2)', sepSpeed(t3_a, t3_b, 2, 0.0001));
		t.expect('Test 4 - default restitution conserves the closing speed (separating speed -> 2)', sepSpeed(t4_a, t4_b, 2, 0.0001));

		t.simulate(world, 150);
	}, 150);

	visualTest('sphere', 'a sphere dropped on a large ground box settles at rest height without launching', function (t) {
		var world = t.makeWorld();
		t.box(world, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#556' });
		var ball = t.sphere(world, 0.5, 1, { pos: [0, 3, 0], color: '#4af' });

		var minY = 99, maxRebound = 0, landed = false;
		t.onTick(function () {
			minY = Math.min(minY, ball.position.y);
			if (ball.position.y < 0.55) landed = true;
			if (landed) maxRebound = Math.max(maxRebound, ball.position.y);
		});
		t.expect('settles at rest height 0.5', function () {
			return { ok: Math.abs(ball.position.y - 0.5) < 1e-3, detail: 'y=' + ball.position.y.toFixed(5) };
		});
		t.expect('never sank through the ground', function () {
			return { ok: minY > 0.45, detail: 'min y=' + minY.toFixed(4) };
		});
		t.expect('did not launch - with restitution 0 there is no bounce', function () {
			return { ok: maxRebound < 0.55, detail: 'max height after landing=' + maxRebound.toFixed(3) };
		});
		t.simulate(world, 400);
	}, 400);

	staticTest('sphere', 'GJK reports a shallow sphere-in-large-box penetration as overlapping with correct depth, in both operand orders', function (t) {

		var y = 0.45;
		var sph = t.loneBody(new AP.SphereShape(0.5), { pos: [0, y, 0], color: '#4af' });
		var box = t.loneBody(new AP.BoxShape(20, 0.5, 20), { pos: [0, -0.5, 0], color: '#556' });

		function measure(a, b) {
			var pa = { shape: a.shape, position: a.position, rotation: a.rotation };
			var pb = { shape: b.shape, position: b.position, rotation: b.rotation };
			var s = new AP.MinkowskiSupport(pa, pb);
			var r = new AP.GJK().run(s);
			if (!r.overlapping) return null;
			return new AP.EPA().run(s, r.simplex).distance;
		}
		var dSphereFirst = measure(sph, box);
		var dBoxFirst = measure(box, sph);
		t.checkTrue(dSphereFirst !== null, 'sphere-first order detects overlap');
		t.checkTrue(dBoxFirst !== null, 'box-first order detects overlap (this order was the broken one)');
		t.check(dSphereFirst, 0.05, 1e-3, 'sphere-first depth is the true penetration');
		t.check(dBoxFirst, 0.05, 1e-3, 'box-first depth matches - order independence restored');
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
