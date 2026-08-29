(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3, Q = AP.Quaternion;
	var DESC = "GJK over the Minkowski difference of two placed shapes. Returns overlapping (hand " +
		"to EPA) or separated (exact distance + witness points + normal, used directly for " +
		"speculative contacts). Exact touching (distance exactly 0) is the hard boundary case - " +
		"resolved via several diverse seed tetrahedra rather than growing one incrementally, which " +
		"gets stuck in the touching plane.";
	function test(group, name, fn, opts) {
		opts = opts || {};
		Runner.test(group, name, fn, { page: 'gjk', description: DESC, visual: !!opts.visual, steps: 0 });
	}

	function placed(shape, pos, rot) {
		return { shape: shape, position: pos || new V(0, 0, 0), rotation: rot || new Q() };
	}

	function run(a, b) {
		return new AP.GJK().run(new AP.MinkowskiSupport(a, b));
	}

	test('collision/gjk', 'two far-apart spheres report the exact gap and correct normal', function (t) {
		var r = run(placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.SphereShape(1), new V(5, 0, 0)));
		t.checkTrue(!r.overlapping, 'separated');
		t.check(r.distance, 3, 1e-9, 'distance = 5 - 1 - 1');
		t.check(r.normal.x, -1, 1e-9, 'normal points from B to A: A is in the -X direction from B');
	});

	test('collision/gjk', 'sphere and box separated along an axis report the true gap', function (t) {
		var r = run(placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(3, 0, 0)));
		t.checkTrue(!r.overlapping, 'separated');
		t.check(r.distance, 1, 1e-9, 'sphere surface at x=1, box surface at x=2');
	});

	test('collision/gjk', 'diagonal box separation matches the true corner-to-corner distance', function (t) {
		var r = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(5, 5, 5)));
		t.checkTrue(!r.overlapping, 'separated');
		t.check(r.distance, Math.sqrt(27), 1e-6, 'corner (1,1,1) to corner (4,4,4)');
	});

	test('collision/gjk', 'boxes flush on X report separated at distance 0, never NaN', function (t) {
		var r = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(2, 0, 0)));
		t.checkTrue(!r.overlapping, 'separated, not falsely overlapping');
		t.check(r.distance, 0, 1e-9, 'exact zero gap');
		t.checkTrue(r.normal.isFinite(), 'normal is never NaN even at the degenerate zero-distance boundary');
	});

	test('collision/gjk', 'boxes flush on Y report separated at distance 0 (axis-independence check)', function (t) {
		var r = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(0, 2, 0)));
		t.checkTrue(!r.overlapping, 'separated on the Y axis too, not just X');
		t.check(r.distance, 0, 1e-9, 'exact zero gap');
	});

	test('collision/gjk', 'boxes flush on Z report separated at distance 0 (axis-independence check)', function (t) {
		var r = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 2)));
		t.checkTrue(!r.overlapping, 'separated on the Z axis too');
		t.check(r.distance, 0, 1e-9, 'exact zero gap');
	});

	test('collision/gjk', 'two spheres touching exactly report separated at distance 0', function (t) {
		var r = run(placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.SphereShape(1), new V(2, 0, 0)));
		t.checkTrue(!r.overlapping, 'separated');
		t.check(r.distance, 0, 1e-9, 'radius sum exactly equals the center distance');
	});

	test('collision/gjk', 'a capsule and a sphere touching exactly report separated at distance 0', function (t) {
		var r = run(placed(new AP.CapsuleShape(0.5, 3), new V(0, 0, 0)), placed(new AP.CapsuleShape(0.5, 3), new V(1, 0, 0)));
		t.checkTrue(!r.overlapping, 'separated');
		t.check(r.distance, 0, 1e-9, 'capsules touching side-by-side at exactly the radius sum');
	});

	test('collision/gjk', 'penetrating boxes report overlapping, not a false touch', function (t) {
		var r = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(1.5, 0, 0)));
		t.checkTrue(r.overlapping, '0.5 units of real penetration must report overlapping');
	});

	test('collision/gjk', 'overlapping spheres report overlapping', function (t) {
		var r = run(placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.SphereShape(1), new V(1, 0, 0)));
		t.checkTrue(r.overlapping, 'centers 1 apart, radius sum 2: real overlap');
	});

	test('collision/gjk', 'exactly coincident shapes report overlapping (the deepest possible case)', function (t) {
		var r1 = run(placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.SphereShape(1), new V(0, 0, 0)));
		t.checkTrue(r1.overlapping, 'coincident spheres');
		var r2 = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)));
		t.checkTrue(r2.overlapping, 'coincident boxes');
	});

	test('collision/gjk', 'sphere vs box overlap is detected', function (t) {
		var r = run(placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(1.5, 0, 0)));
		t.checkTrue(r.overlapping, 'sphere surface at x=1 vs box surface at x=0.5: 0.5 overlap');
	});

	test('collision/gjk', 'deep box penetration is detected', function (t) {
		var r = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(0.5, 0, 0)));
		t.checkTrue(r.overlapping, '1.5 units of penetration out of a 2-unit half-extent sum');
	});

	test('collision/gjk', 'sphere-sphere matches the closed-form overlap test across many random pairs', function (t) {
		var s = 999; function rand() { s = (s * 1103515245 + 12345) & 0x7fffffff; return s / 0x7fffffff; }
		var mismatches = 0, trials = 300;
		for (var i = 0; i < trials; i++) {
			var r1 = 0.5 + rand() * 1.5, r2 = 0.5 + rand() * 1.5;
			var px = rand() * 8 - 4, py = rand() * 8 - 4, pz = rand() * 8 - 4;
			var d = Math.sqrt(px * px + py * py + pz * pz);
			var res = run(placed(new AP.SphereShape(r1), new V(0, 0, 0)), placed(new AP.SphereShape(r2), new V(px, py, pz)));
			var expect = d < r1 + r2 - 1e-9;
			var isTouch = Math.abs(d - (r1 + r2)) < 1e-7;
			if (!isTouch && res.overlapping !== expect) mismatches++;
		}
		t.checkEqual(mismatches, 0, trials + ' random sphere pairs, checked against |centers| vs radius sum');
	});

	test('collision/gjk', 'box-box matches an AABB ground truth across many random offsets, including forced touches', function (t) {
		var s = 12345; function rand() { s = (s * 1103515245 + 12345) & 0x7fffffff; return s / 0x7fffffff; }
		function bruteOverlap(p) { return Math.abs(p[0]) < 2 && Math.abs(p[1]) < 2 && Math.abs(p[2]) < 2; }
		function bruteTouch(p) {
			var dx = Math.abs(p[0]), dy = Math.abs(p[1]), dz = Math.abs(p[2]);
			return (Math.abs(dx - 2) < 1e-9 && dy <= 2 && dz <= 2) ||
				(Math.abs(dy - 2) < 1e-9 && dx <= 2 && dz <= 2) ||
				(Math.abs(dz - 2) < 1e-9 && dx <= 2 && dy <= 2);
		}
		var mismatches = 0, trials = 400;
		for (var i = 0; i < trials; i++) {
			var p = [rand() * 6 - 3, rand() * 6 - 3, rand() * 6 - 3];
			if (rand() < 0.15) { var axis = Math.floor(rand() * 3); p[axis] = p[axis] >= 0 ? 2 : -2; }
			var res = run(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(p[0], p[1], p[2])));
			var expect = bruteOverlap(p) && !bruteTouch(p);
			if (res.overlapping !== expect) mismatches++;
		}
		t.checkEqual(mismatches, 0, trials + ' random box pairs (15% forced to exact touch), checked against AABB overlap');
	});

	test('collision/gjk', 'witness points sit on each shape\'s own surface for a separated pair', function (t) {
		var r = run(placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.SphereShape(1), new V(5, 0, 0)));
		t.check(r.pointA.x, 1, 1e-9, 'witness on A is its surface point facing B');
		t.check(r.pointB.x, 4, 1e-9, 'witness on B is its surface point facing A');
	});

	test('collision/gjk', 'closest points and separating normal between two spheres', function (t) {
		var a = placed(new AP.SphereShape(1), new V(-2, 0, 0));
		var b = placed(new AP.SphereShape(1), new V(2.5, 0.5, 0));
		var r = run(a, b);
		t.bodies.push({ shape: a.shape, position: a.position, rotation: a.rotation, _color: '#4af' });
		t.bodies.push({ shape: b.shape, position: b.position, rotation: b.rotation, _color: '#f55' });
		t.support = { dir: [r.normal.x, r.normal.y, r.normal.z], point: [r.pointA.x, r.pointA.y, r.pointA.z] };
		var centerDist = Math.sqrt(4.5 * 4.5 + 0.5 * 0.5);
		t.checkTrue(!r.overlapping, 'separated');
		t.check(r.distance, centerDist - 2, 1e-6, 'gap between the two sphere surfaces');
	}, { visual: true });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
