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
(function (Runner) {
	Runner.suite('friction_restitution');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Friction (velocity-pass Coulomb, static stick below the friction angle, slide above) " +
		"and restitution (e=0 inelastic .. e=1 full rebound). Includes sphere-on-large-box regression " +
		"tests for the GJK strict-interior / EPA tetrahedron-completion fix.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'friction_restitution', description: DESC }); }
	function mkWorld(sub, it) {
		return new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), new AP.Solver({ substeps: sub || 8, iterations: it || 8 }));
	}
	var DT = 1 / 60;

	// ---- friction: slide vs stop ----

	test('friction', 'a shoved box decelerates and stops under friction', function (t) {
		var w = mkWorld();
		var g = new AP.RigidBody(new AP.BoxShape(40, 0.5, 40), 0); g.position.set(0, -0.5, 0); g.friction = 0.5; w.addRigidBody(g);
		var b = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1); b.position.set(0, 0.5, 0); b.friction = 0.5;
		b.linear_velocity.set(3, 0, 0); w.addRigidBody(b);
		for (var i = 0; i < 300; i++) w.step(DT);
		t.checkTrue(Math.abs(b.linear_velocity.x) < 0.05, 'stopped sliding (|vx| ' + Math.abs(b.linear_velocity.x).toFixed(4) + ')');
		t.checkTrue(b.position.x < 3, 'stopped within a short distance (x ' + b.position.x.toFixed(3) + ')');
	});

	// ---- friction: static stick vs slide on an inclined plane, at the Coulomb boundary ----

	function slopeSlideRate(deg, mu) {
		var w = mkWorld();
		var a = deg * Math.PI / 180;
		var g = new AP.RigidBody(new AP.BoxShape(40, 0.5, 40), 0);
		g.position.set(0, -0.5, 0); g.rotation.setAxisAngle(new V(0, 0, 1), a); g.friction = mu; g.updateDerived(0);
		w.addRigidBody(g);
		var b = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		b.rotation.setAxisAngle(new V(0, 0, 1), a); b.position.set(0, 0.7, 0); b.friction = mu;
		w.addRigidBody(b);
		for (var i = 0; i < 100; i++) w.step(DT); // land + settle
		var x0 = b.position.x, y0 = b.position.y;
		for (var j = 0; j < 300; j++) w.step(DT);
		return Math.hypot(b.position.x - x0, b.position.y - y0) / (300 * DT); // m/s over the late window
	}

	test('friction', 'a box holds still on a slope shallower than its friction angle (static friction)', function (t) {
		// tan(15deg)=0.27 and tan(20deg)=0.36, both below mu=0.5 -> must stick (only a tiny settle creep).
		t.checkTrue(slopeSlideRate(15, 0.5) < 0.1, '15deg (tan 0.27 < mu 0.5): sticks');
		t.checkTrue(slopeSlideRate(20, 0.5) < 0.1, '20deg (tan 0.36 < mu 0.5): sticks');
	});

	test('friction', 'a box slides down a slope steeper than its friction angle (dynamic friction)', function (t) {
		// tan(35deg)=0.70 and tan(45deg)=1.0, both above mu=0.5 -> must slide.
		t.checkTrue(slopeSlideRate(35, 0.5) > 0.3, '35deg (tan 0.70 > mu 0.5): slides');
		t.checkTrue(slopeSlideRate(45, 0.5) > 0.3, '45deg (tan 1.0 > mu 0.5): slides');
	});

	// ---- restitution: e controls rebound height ----

	function reboundHeight(e) {
		var w = mkWorld();
		var g = new AP.RigidBody(new AP.BoxShape(20, 0.5, 20), 0); g.position.set(0, -0.5, 0); g.restitution = e; w.addRigidBody(g);
		var b = new AP.RigidBody(new AP.SphereShape(0.5), 1); b.position.set(0, 3, 0); b.restitution = e; w.addRigidBody(b);
		var maxRebound = 0, landed = false;
		for (var i = 0; i < 400; i++) {
			w.step(DT);
			if (b.position.y < 0.55) landed = true;
			if (landed) maxRebound = Math.max(maxRebound, b.position.y);
		}
		return maxRebound;
	}

	test('restitution', 'e=0 does not bounce; higher e rebounds higher, e=1 returns to the drop height', function (t) {
		// Drop from y=3, rest height 0.5, so fall height 2.5. Energy-conserving rebound height above
		// rest scales with e^2: e=0 -> 0, e=1 -> 2.5 (back to y=3). Generous tolerances (a discrete
		// solver is not lossless), but the ORDERING and the endpoints are the physics.
		var e0 = reboundHeight(0), e05 = reboundHeight(0.5), e08 = reboundHeight(0.8), e1 = reboundHeight(1.0);
		t.checkTrue(e0 < 0.55, 'e=0: no bounce (max height ' + e0.toFixed(3) + ')');
		t.checkTrue(e05 > 0.9 && e05 < 1.4, 'e=0.5: rebounds ~0.6 above rest (max height ' + e05.toFixed(3) + ')');
		t.checkTrue(e08 > e05, 'e=0.8 rebounds higher than e=0.5 (' + e08.toFixed(3) + ' > ' + e05.toFixed(3) + ')');
		t.checkTrue(e1 > 2.5, 'e=1: rebounds nearly back to the drop height y=3 (max height ' + e1.toFixed(3) + ')');
	});

	// ---- sphere-on-large-box GJK/EPA robustness (the bug friction/restitution surfaced) ----

	test('sphere', 'a sphere dropped on a large ground box settles at rest height without launching', function (t) {
		var w = mkWorld();
		var g = new AP.RigidBody(new AP.BoxShape(20, 0.5, 20), 0); g.position.set(0, -0.5, 0); w.addRigidBody(g);
		var b = new AP.RigidBody(new AP.SphereShape(0.5), 1); b.position.set(0, 3, 0); w.addRigidBody(b);
		var minY = 99, maxRebound = 0, landed = false;
		for (var i = 0; i < 400; i++) {
			w.step(DT);
			minY = Math.min(minY, b.position.y);
			if (b.position.y < 0.55) landed = true;
			if (landed) maxRebound = Math.max(maxRebound, b.position.y);
		}
		t.check(b.position.y, 0.5, 1e-3, 'settles at rest height 0.5');
		t.checkTrue(minY > 0.45, 'never sank through the ground (min y ' + minY.toFixed(4) + ')');
		t.checkTrue(maxRebound < 0.55, 'did not launch (max height after landing ' + maxRebound.toFixed(3) + ') - with restitution 0 there is no bounce');
	});

	test('sphere', 'GJK reports a shallow sphere-in-large-box penetration as overlapping with correct depth, in both operand orders', function (t) {
		// The exact bug: order-dependent mis-classification. Assert both orders now agree and give the
		// true penetration depth (sphere radius 0.5, center at y, ground top at 0 -> pen = 0.5 - y).
		var sph = new AP.SphereShape(0.5), box = new AP.BoxShape(20, 0.5, 20);
		function measure(shapeA, posA, shapeB, posB) {
			var pa = { shape: shapeA, position: posA, rotation: new AP.Quaternion() };
			var pb = { shape: shapeB, position: posB, rotation: new AP.Quaternion() };
			var s = new AP.MinkowskiSupport(pa, pb);
			var r = new AP.GJK().run(s);
			if (!r.overlapping) return null;
			return new AP.EPA().run(s, r.simplex).distance;
		}
		var y = 0.45; // pen = 0.05
		var sphPos = new V(0, y, 0), boxPos = new V(0, -0.5, 0);
		var dSphereFirst = measure(sph, sphPos, box, boxPos);
		var dBoxFirst = measure(box, boxPos, sph, sphPos);
		t.checkTrue(dSphereFirst !== null, 'sphere-first order detects overlap');
		t.checkTrue(dBoxFirst !== null, 'box-first order detects overlap (this order was the broken one)');
		t.check(dSphereFirst, 0.05, 1e-3, 'sphere-first depth is the true penetration');
		t.check(dBoxFirst, 0.05, 1e-3, 'box-first depth matches - order independence restored');
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
