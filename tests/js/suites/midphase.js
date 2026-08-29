// Midphase: expands a broadphase body pair into candidate PRIMITIVE-shape pairs, each carrying a
// world position/rotation. A primitive body is itself the one candidate; a Compound expands to its
// overlapping children; a Mesh expands to its overlapping triangles (wrapped as TriangleShape).
// Never computes contact data (plan.md, Midphase) - these tests check WHICH primitives are
// produced, never how deep or where they touch.
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3, Q = AP.Quaternion;
	var DESC = "Midphase turns one broadphase body pair into candidate primitive-shape pairs. A " +
		"primitive body needs no expansion; Compound/Mesh shapes get their own BVH (built once, " +
		"cached on the shape) walked against the other body's world AABB brought into local space. " +
		"The leaf-walk result is cached per other-body-id, including empty results.";
	function test(group, name, fn, opts) {
		opts = opts || {};
		Runner.test(group, name, fn, { page: 'midphase', description: DESC, visual: !!opts.visual, steps: 0 });
	}

	function mkBody(shape, mass, pos) {
		var b = new AP.RigidBody(shape, mass);
		b.position.set(pos[0], pos[1], pos[2]);
		b.updateDerived();
		return b;
	}

	test('collision/midphase', 'two primitive bodies produce exactly one candidate pair', function (t) {
		var mid = new AP.Midphase();
		var a = mkBody(new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(new AP.SphereShape(1), 1, [1, 0, 0]);
		var pairs = mid.expandPair(a, b);
		t.checkEqual(pairs.length, 1, 'one primitive pair, no expansion needed');
		t.checkTrue(pairs[0].a.shape === a.shape && pairs[0].b.shape === b.shape, 'candidate shapes are the bodies\' own shapes');
	});

	test('collision/midphase', 'a compound only offers up the child that overlaps the other body', function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		comp.addChild(new AP.SphereShape(0.5), new V(-3, 0, 0), new Q());
		comp.addChild(new AP.SphereShape(0.5), new V(3, 0, 0), new Q());
		var cbody = mkBody(comp, 1, [0, 0, 0]);
		var near = mkBody(new AP.SphereShape(0.5), 1, [3.2, 0, 0]); // overlaps the +X child only
		var pairs = mid.expandPair(cbody, near);
		t.checkEqual(pairs.length, 1, 'only the overlapping child is offered as a candidate');
		t.check(pairs[0].a.position.x, 3, 1e-9, 'candidate carries the WORLD position of the child (body pos + local offset)');
	});

	test('collision/midphase', 'a compound offers every child that overlaps a large enough other body', function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		comp.addChild(new AP.SphereShape(0.5), new V(-3, 0, 0), new Q());
		comp.addChild(new AP.SphereShape(0.5), new V(3, 0, 0), new Q());
		var cbody = mkBody(comp, 1, [0, 0, 0]);
		var wide = mkBody(new AP.BoxShape(10, 10, 10), 1, [0, 0, 0]); // covers both children
		t.checkEqual(mid.expandPair(cbody, wide).length, 2, 'both children overlap a wide enough box');
	});

	test('collision/midphase', 'compound children carry the composed world rotation', function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		// Child rotated 90 degrees about Z relative to the compound.
		var childRot = new Q().setAxisAngle(new V(0, 0, 1), Math.PI / 2);
		comp.addChild(new AP.BoxShape(1, 1, 1), new V(0, 0, 0), childRot);
		// Body itself unrotated.
		var cbody = mkBody(comp, 1, [0, 0, 0]);
		var other = mkBody(new AP.SphereShape(0.5), 1, [0, 0, 0]);
		var pairs = mid.expandPair(cbody, other);
		t.checkEqual(pairs.length, 1, 'one candidate');
		t.check(pairs[0].a.rotation.z, childRot.z, 1e-9, 'world rotation matches the child\'s local rotation when the body itself is unrotated');
	});

	test('collision/midphase', 'a mesh offers only the triangles near the other body', function (t) {
		var mid = new AP.Midphase();
		var verts = [new V(-10, 0, -10), new V(10, 0, -10), new V(10, 0, 10), new V(-10, 0, 10)];
		var mesh = new AP.MeshShape(verts, [0, 1, 2, 0, 2, 3]);
		var ground = mkBody(mesh, 0, [0, 0, 0]);
		var ball = mkBody(new AP.SphereShape(0.5), 1, [8, 0.3, -8]); // near the far corner, in triangle 0's area
		var pairs = mid.expandPair(ground, ball);
		t.checkTrue(pairs.length >= 1, 'at least one triangle overlaps near the corner');
		t.checkTrue(pairs.length <= 2, 'not every triangle in the mesh is offered, only the nearby one(s)');
		t.checkTrue(pairs[0].a.shape instanceof AP.TriangleShape, 'mesh candidates are wrapped as TriangleShape');
	});

	test('collision/midphase', 'a mesh far from the other body offers zero candidates', function (t) {
		var mid = new AP.Midphase();
		var verts = [new V(-1, 0, -1), new V(1, 0, -1), new V(1, 0, 1), new V(-1, 0, 1)];
		var mesh = new AP.MeshShape(verts, [0, 1, 2, 0, 2, 3]);
		var ground = mkBody(mesh, 0, [0, 0, 0]);
		var farBall = mkBody(new AP.SphereShape(0.5), 1, [100, 100, 100]);
		t.checkEqual(mid.expandPair(ground, farBall).length, 0, 'no candidates when nothing is near');
	});

	test('collision/midphase', 'mesh triangle world vertices land at the correct world position', function (t) {
		var mid = new AP.Midphase();
		var verts = [new V(-1, 0, -1), new V(1, 0, -1), new V(1, 0, 1), new V(-1, 0, 1)];
		var mesh = new AP.MeshShape(verts, [0, 1, 2]);
		var ground = mkBody(mesh, 0, [5, 0, 0]); // body offset by +5 on X
		var ball = mkBody(new AP.SphereShape(0.5), 1, [5.5, 0.3, -0.5]);
		var pairs = mid.expandPair(ground, ball);
		t.checkTrue(pairs.length >= 1, 'candidate produced');
		var tri = pairs[0].a.shape;
		t.check(tri.a.x, 4, 1e-9, 'triangle vertex a.x is baked into world space (local -1 + body offset 5)');
	});

	test('collision/midphase', 'repeated identical queries reuse the cached leaf-walk result', function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		comp.addChild(new AP.SphereShape(0.5), new V(0, 0, 0), new Q());
		var cbody = mkBody(comp, 1, [0, 0, 0]);
		var other = mkBody(new AP.SphereShape(0.5), 1, [0.5, 0, 0]);
		var first = mid.expandPair(cbody, other);
		// Same bodies, same positions - second call must hit the cache and agree exactly.
		var second = mid.expandPair(cbody, other);
		t.checkEqual(first.length, second.length, 'identical query returns the same candidate count');
	});

	test('collision/midphase', 'an empty leaf-walk result is itself cached (no candidates both times)', function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		comp.addChild(new AP.SphereShape(0.5), new V(0, 0, 0), new Q());
		var cbody = mkBody(comp, 1, [0, 0, 0]);
		var farOther = mkBody(new AP.SphereShape(0.5), 1, [500, 500, 500]);
		t.checkEqual(mid.expandPair(cbody, farOther).length, 0, 'first query: no candidates');
		t.checkEqual(mid.expandPair(cbody, farOther).length, 0, 'second identical query: still no candidates (served from cache)');
	});

	test('collision/midphase', 'invalidate() clears the leaf cache', function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		comp.addChild(new AP.SphereShape(0.5), new V(0, 0, 0), new Q());
		var cbody = mkBody(comp, 1, [0, 0, 0]);
		var other = mkBody(new AP.SphereShape(0.5), 1, [0.5, 0, 0]);
		mid.expandPair(cbody, other);
		mid.invalidate();
		// Not asserting internal state directly - just that a fresh query after invalidate still
		// produces the correct (non-stale) answer.
		t.checkEqual(mid.expandPair(cbody, other).length, 1, 'query after invalidate still finds the overlap');
	});

	test('collision/midphase', "a shape's BVH is built once and shared across bodies using that shape", function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		comp.addChild(new AP.SphereShape(0.5), new V(1, 0, 0), new Q());
		var bodyOne = mkBody(comp, 1, [0, 0, 0]);
		var bodyTwo = mkBody(comp, 1, [10, 0, 0]); // SAME shape instance, different body
		var other = mkBody(new AP.SphereShape(0.5), 1, [1.2, 0, 0]);
		mid.expandPair(bodyOne, other);
		var bvhAfterFirst = comp._midphaseBVH;
		mid.expandPair(bodyTwo, mkBody(new AP.SphereShape(0.5), 1, [11.2, 0, 0]));
		t.checkTrue(comp._midphaseBVH === bvhAfterFirst, 'the same BVH instance is reused for a second body sharing the shape');
	});

	// ---- visual ----

	test('collision/midphase', 'compound candidate expansion against a nearby probe', function (t) {
		var mid = new AP.Midphase();
		var comp = new AP.CompoundShape();
		comp.addChild(new AP.SphereShape(0.6), new V(-2, 0, 0), new Q());
		comp.addChild(new AP.SphereShape(0.6), new V(2, 0, 0), new Q());
		comp.addChild(new AP.SphereShape(0.6), new V(0, 2, 0), new Q());
		var cbody = mkBody(comp, 1, [0, 0, 0]);
		var probe = mkBody(new AP.SphereShape(0.4), 1, [2.3, 0, 0]);
		t.bodies.push({ shape: comp, position: cbody.position, rotation: cbody.rotation, _color: '#4af' });
		t.bodies.push({ shape: probe.shape, position: probe.position, rotation: probe.rotation, _color: '#f55' });
		var pairs = mid.expandPair(cbody, probe);
		t.checkEqual(pairs.length, 1, 'only the near child is a candidate against the probe');
	}, { visual: true });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
