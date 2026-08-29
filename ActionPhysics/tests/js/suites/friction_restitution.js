// Friction and restitution (plan.md build step 7), plus the sphere-vs-large-box GJK/EPA robustness
// fix these exercised.
//
// FRICTION is a velocity-pass contact constraint (Muller et al. 2020 sec 3.6): after the position
// solve sets velocities, the tangential contact-relative velocity is driven toward zero, capped by
// the Coulomb limit mu*|normal impulse|/h. Below the cap the contact sticks (a box holds on a slope
// shallower than its friction angle); at the cap it slides. This is NOT the derived-velocity clamp
// plan.md forbids - that governed a whole body's velocity to hide a detection bug; this acts only on
// the contact-relative tangential velocity, which is what friction physically IS.
//
// RESTITUTION is the normal half of the same pass: the contact-relative approach speed is captured
// before the position solve zeroes it, and a fraction e of it is restored as a separating velocity.
// e=0 is inelastic (no bounce), e=1 returns all the energy (rebounds to the drop height).
//
// The sphere tests are regressions for a real GJK/EPA bug these surfaced: a small sphere shallowly
// penetrating a much larger box was mis-classified by GJK as separated (its seed tetrahedra could
// not enclose the origin for that size ratio, and the incremental walk then stalled at the contact
// face), so EPA never ran and the penetration went uncorrected until the body launched. GJK now
// disambiguates a boundary-origin case with an explicit strict-interior probe, and EPA completes a
// degenerate (<4-point) simplex into a real tetrahedron instead of reading past its vertex count.
//
// Every dynamics test here is built through the shared harness (t.makeWorld/t.box/t.sphere) and
// asserts LIVE against the ticking world via t.expect + t.simulate, and is marked visual so it's
// watchable (a slide-to-stop, a slope hold/slide, a bounce). The two GJK/EPA depth-classification
// tests are static geometry queries with no motion (same category as the AABB tests elsewhere) - no
// t.simulate for those, but the shapes are still drawn via t.loneBody so the configuration is visible.
(function (Runner) {
	Runner.suite('friction_restitution');
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

	// ---- friction: slide vs stop ----

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

	// ---- friction: static stick vs slide on an inclined plane, at the Coulomb boundary ----
	// tan(15deg)=0.27 and tan(20deg)=0.36, both below mu=0.5 -> must stick (only a tiny settle creep).
	// tan(35deg)=0.70 and tan(45deg)=1.0, both above mu=0.5 -> must slide.

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
					slideRate = dist / ((tick - settleTick) / 60); // m/s over the window since settling
				}
			});
			t.expect(expectStick ? 'sticks (slide rate stays low)' : 'slides (slide rate stays high)', function () {
				if (x0 == null) return false; // hasn't reached the settle tick yet
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

	// ---- restitution: e controls rebound height ----
	// Drop from y=3, rest height 0.5, so fall height 2.5. Energy-conserving rebound height above rest
	// scales with e^2: e=0 -> 0, e=1 -> 2.5 (back to y=3). Generous tolerances (a discrete solver is
	// not lossless), but the ORDERING and the endpoints are the physics.

	function restitutionTest(e, expectMin, expectMax, label) {
		visualTest('restitution', 'restitution e=' + e + ': ' + label, function (t) {
			var world = t.makeWorld();
			t.box(world, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: e, color: '#556' });
			var ball = t.sphere(world, 0.5, 1, { pos: [0, 3, 0], restitution: e, color: '#f84' });

			var maxRebound = 0, landed = false;
			t.onTick(function () {
				if (ball.position.y < 0.55) landed = true;
				if (landed) maxRebound = Math.max(maxRebound, ball.position.y);
			});
			t.expect(label, function () {
				return { ok: maxRebound >= expectMin && maxRebound <= expectMax, detail: 'max rebound height=' + maxRebound.toFixed(3) };
			});
			t.simulate(world, 400);
		}, 400);
	}
	restitutionTest(0, 0, 0.55, 'no bounce (inelastic)');
	restitutionTest(0.5, 0.9, 1.4, 'rebounds ~0.6 above rest');
	restitutionTest(0.8, 1.6, 2.4, 'rebounds higher than e=0.5');
	restitutionTest(1.0, 2.5, 3.2, 'rebounds nearly back to the drop height y=3');

	// ---- sphere-on-large-box GJK/EPA robustness (the bug friction/restitution surfaced) ----

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
		// The exact bug: order-dependent mis-classification. Assert both orders now agree and give the
		// true penetration depth (sphere radius 0.5, center at y, ground top at 0 -> pen = 0.5 - y).
		var y = 0.45; // pen = 0.05
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
