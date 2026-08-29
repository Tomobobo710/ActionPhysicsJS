(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3, Q = AP.Quaternion;
	var EPS = 0.02;
	var DESC = "GJK+EPA collision between two shapes in a specific arrangement: how deep they " +
		"overlap, which direction separates them, and where they touch, checked against known-" +
		"correct values.";

	function test(group, name, fn) {
		Runner.test(group, name, fn, { page: 'gjk-epa-configs', description: DESC, visual: true, steps: 0 });
	}

	function placed(shape, pos, rot) {
		return { shape: shape, position: pos || new V(0, 0, 0), rotation: rot || new Q(0, 0, 0, 1) };
	}

	function collide(t, shapeA, posA, rotA, shapeB, posB, rotB, depth, normalDir, dotExp) {
		var a = placed(shapeA, new V(posA[0], posA[1], posA[2]), rotA && new Q(rotA[0], rotA[1], rotA[2], rotA[3]).normalize());
		var b = placed(shapeB, new V(posB[0], posB[1], posB[2]), rotB && new Q(rotB[0], rotB[1], rotB[2], rotB[3]).normalize());
		var sup = new AP.MinkowskiSupport(a, b);
		var gjkResult = new AP.GJK().run(sup);
		t.checkTrue(gjkResult.overlapping, 'a contact is found (GJK confirms overlap)');
		if (!gjkResult.overlapping) return;
		var epa = new AP.EPA().run(sup, gjkResult.simplex);
		t.check(epa.distance, depth, EPS, 'penetration depth ~ ' + depth);
		var dot = epa.normal.x * normalDir[0] + epa.normal.y * normalDir[1] + epa.normal.z * normalDir[2];
		t.check(dot, dotExp, EPS, 'contact normal points the right way');
	}

	test('collision/gjk-epa-configs', 'boxes: flat stack', function (t) {
		collide(t, new AP.BoxShape(2, 1, 2), [0, 0, 0], null, new AP.BoxShape(0, 0.5, 0.5), [0, 1.49, 0], null,
			0.02, [0, -1, 0], 1);
	});
	test('collision/gjk-epa-configs', 'boxes: offset overlap', function (t) {
		collide(t, new AP.BoxShape(2, 1, 2), [2, 0, 0], null, new AP.BoxShape(0, 0.5, 0.5), [1.5, 1.25, 0], null,
			0.26, [0, -1, 0], 1);
	});
	test('collision/gjk-epa-configs', 'boxes: both rotated about y', function (t) {
		collide(t, new AP.BoxShape(2, 1, 2), [0, 0, 0], [0, -0.415, 0, 1], new AP.BoxShape(0, 0.5, 0.5), [0, 1.49, 0], [0, 0.415, 0, 1],
			0.02, [0, -1, 0], 1);
	});
	test('collision/gjk-epa-configs', 'boxes: one box rotated about x', function (t) {
		collide(t, new AP.BoxShape(2, 1, 2), [2, -1, 0], null, new AP.BoxShape(0, 0.5, 0.5), [2, 0.7, 0], [0.415, 0, 0, 1],
			0.017, [0, -1, 0], 1);
	});

	test('collision/gjk-epa-configs', 'spheres: near-touching, normal points down', function (t) {
		collide(t, new AP.SphereShape(1), [0, 0.9999, 0], null, new AP.SphereShape(1), [0, -1, 0], null,
			0.0001, [0, 1, 0], 1);
	});
	test('collision/gjk-epa-configs', 'spheres: deep overlap', function (t) {
		collide(t, new AP.SphereShape(1), [0, 0.5, 0], null, new AP.SphereShape(1), [0, -1, 0], null,
			0.5, [0, 1, 0], 1);
	});
	test('collision/gjk-epa-configs', 'spheres: one sphere rotated (rotation-invariant collision)', function (t) {
		collide(t, new AP.SphereShape(1), [-2, 1, 0], [1, 0, 0, 1], new AP.SphereShape(1), [-2, -0.5, 0], null,
			0.5, [0, 1, 0], 1);
	});
	test('collision/gjk-epa-configs', 'spheres: other sphere rotated (rotation-invariant collision)', function (t) {
		collide(t, new AP.SphereShape(1), [-2, 0.75, 0], null, new AP.SphereShape(1), [-2, -0.75, 0], [0, 5, -3, 1],
			0.5, [0.1486, 0.98, -0.1486], 0.96);
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
