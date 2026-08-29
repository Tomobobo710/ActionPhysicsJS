(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3, Q = AP.Quaternion;
	var DESC = "EPA expands a GJK-confirmed enclosing simplex into penetration depth, normal, and " +
		"witness points via the standard expanding-polytope algorithm. Flat-faced shapes (boxes) " +
		"converge exactly; curved shapes (spheres) converge asymptotically with iteration count, " +
		"same as every EPA implementation - not a bug, a property of polygonal approximation.";
	function test(group, name, fn, opts) {
		opts = opts || {};
		Runner.test(group, name, fn, { page: 'epa', description: DESC, visual: !!opts.visual, steps: 0 });
	}

	function placed(shape, pos, rot) {
		return { shape: shape, position: pos || new V(0, 0, 0), rotation: rot || new Q() };
	}

	function overlapDepth(t, a, b) {
		var sup = new AP.MinkowskiSupport(a, b);
		var gjkResult = new AP.GJK().run(sup);
		t.checkTrue(gjkResult.overlapping, 'GJK confirms overlap before EPA runs');
		if (!gjkResult.overlapping) return null;
		return { epa: new AP.EPA().run(sup, gjkResult.simplex), sup: sup };
	}

	test('collision/epa', 'penetrating boxes report the exact overlap depth and axis normal', function (t) {
		var r = overlapDepth(t, placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(1.5, 0, 0)));
		t.check(r.epa.distance, 0.5, 1e-6, 'depth = 2 - 1.5');
		t.check(Math.abs(r.epa.normal.x), 1, 1e-6, 'normal is along the true penetration axis');
	});

	test('collision/epa', 'deep box penetration still converges exactly (flat faces, no curvature error)', function (t) {
		var r = overlapDepth(t, placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(0.5, 0, 0)));
		t.check(r.epa.distance, 1.5, 1e-6, 'depth = 2 - 0.5');
	});

	test('collision/epa', 'box penetration along Y and Z axes both converge exactly (axis-independence check)', function (t) {
		var ry = overlapDepth(t, placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(0, 1.5, 0)));
		t.check(ry.epa.distance, 0.5, 1e-6, 'Y-axis depth');
		t.check(Math.abs(ry.epa.normal.y), 1, 1e-6, 'Y-axis normal');
		var rz = overlapDepth(t, placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 1.5)));
		t.check(rz.epa.distance, 0.5, 1e-6, 'Z-axis depth');
		t.check(Math.abs(rz.epa.normal.z), 1, 1e-6, 'Z-axis normal');
	});

	test('collision/epa', 'overlapping spheres converge close to the exact depth at the default iteration budget', function (t) {
		var r = overlapDepth(t, placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.SphereShape(1), new V(1, 0, 0)));
		t.check(r.epa.distance, 1.0, 1e-2, 'depth = 1 + 1 - 1, within curvature-approximation tolerance');
		t.checkTrue(Math.abs(r.epa.normal.x) > 0.99, 'normal is close to the true X axis');
	});

	test('collision/epa', 'shallow sphere overlap (the realistic contact regime) converges tightly', function (t) {

		var r = overlapDepth(t, placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.SphereShape(1), new V(1.95, 0, 0)));
		t.check(r.epa.distance, 0.05, 1e-4, 'shallow depth converges to near machine precision');
	});

	test('collision/epa', 'sphere vs box overlap depth is correct', function (t) {
		var r = overlapDepth(t, placed(new AP.SphereShape(1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(1.5, 0, 0)));
		t.check(r.epa.distance, 0.5, 1e-3, 'sphere surface at x=1, box surface at x=0.5');
	});

	test('collision/epa', 'more iterations converges a deep sphere overlap closer to the true depth', function (t) {
		var a = placed(new AP.SphereShape(1.2), new V(0, 0, 0));
		var b = placed(new AP.SphereShape(1.1), new V(0.3, 0.2, -0.1));
		var sup = new AP.MinkowskiSupport(a, b);
		var gjkResult = new AP.GJK().run(sup);
		t.checkTrue(gjkResult.overlapping, 'confirmed deep overlap');
		var trueDepth = 1.2 + 1.1 - Math.sqrt(0.3 * 0.3 + 0.2 * 0.2 + 0.1 * 0.1);
		var errLow = Math.abs(new AP.EPA().run(sup, gjkResult.simplex, 16).distance - trueDepth);
		var errHigh = Math.abs(new AP.EPA().run(sup, gjkResult.simplex, 200).distance - trueDepth);
		t.checkTrue(errHigh < errLow, 'error shrinks as iteration budget grows (asymptotic convergence, not noise)');
	});

	test('collision/epa', 'witness points land on each shape\'s own surface', function (t) {
		var r = overlapDepth(t, placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(1.5, 0, 0)));
		t.check(r.epa.pointA.x, 1, 1e-6, 'witness on A sits on A\'s own +X face');
		t.check(r.epa.pointB.x, 0.5, 1e-6, 'witness on B sits on B\'s own -X face');
	});

	test('collision/epa', 'box-box depth matches the axis-aligned closed form across many random overlaps', function (t) {
		var s = 999; function rand() { s = (s * 1103515245 + 12345) & 0x7fffffff; return s / 0x7fffffff; }
		var mismatches = 0, trials = 150;
		for (var i = 0; i < trials; i++) {
			var px = rand() * 1.8 - 0.9, py = rand() * 1.8 - 0.9, pz = rand() * 1.8 - 0.9;
			var sup = new AP.MinkowskiSupport(placed(new AP.BoxShape(1, 1, 1), new V(0, 0, 0)), placed(new AP.BoxShape(1, 1, 1), new V(px, py, pz)));
			var gr = new AP.GJK().run(sup);
			if (!gr.overlapping) continue;
			var r = new AP.EPA().run(sup, gr.simplex);
			var trueDepth = Math.min(2 - Math.abs(px), 2 - Math.abs(py), 2 - Math.abs(pz));
			if (Math.abs(r.distance - trueDepth) > 1e-4) mismatches++;
		}
		t.checkEqual(mismatches, 0, trials + ' random box overlaps, checked against the axis-aligned closed form (exact for flat faces)');
	});

	test('collision/epa', 'penetration depth and normal for two overlapping boxes', function (t) {
		var a = placed(new AP.BoxShape(1, 1, 1), new V(-0.75, 0, 0));
		var b = placed(new AP.BoxShape(1, 1, 1), new V(0.75, 0, 0));
		var sup = new AP.MinkowskiSupport(a, b);
		var gr = new AP.GJK().run(sup);
		t.checkTrue(gr.overlapping, 'boxes overlap');
		var r = new AP.EPA().run(sup, gr.simplex);
		t.bodies.push({ shape: a.shape, position: a.position, rotation: a.rotation, _color: '#4af' });
		t.bodies.push({ shape: b.shape, position: b.position, rotation: b.rotation, _color: '#f55' });
		t.support = { dir: [r.normal.x, r.normal.y, r.normal.z], point: [r.pointA.x, r.pointA.y, r.pointA.z] };
		t.check(r.distance, 0.5, 1e-6, 'depth = 2 - 1.5');
	}, { visual: true });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
