(function (Runner) {
	Runner.suite('math');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "The Vector3 the whole engine is built on: construction, arithmetic, dot and cross " +
		"products, length and normalization. The shared API sits above the additions marker so its class " +
		"body can be pasted over ours; the physics-only additions below it never allocate.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'vector3', description: DESC }); }

	function vecIs(t, v, x, y, z, label) {
		t.check(v.x, x, 1e-12, label + '.x');
		t.check(v.y, y, 1e-12, label + '.y');
		t.check(v.z, z, 1e-12, label + '.z');
	}

	test('math/vector3', 'instantiate (no values / with values)', function (t) {
		var a = new V();
		t.checkEqual(a.x, 0, 'default x'); t.checkEqual(a.y, 0, 'default y'); t.checkEqual(a.z, 0, 'default z');
		var b = new V(5, 3, 2.5);
		t.checkEqual(b.x, 5, 'x'); t.checkEqual(b.y, 3, 'y'); t.checkEqual(b.z, 2.5, 'z');
	});

	test('math/vector3', 'copy leaves the source intact', function (t) {
		var a = new V(1, 3, 5.5), b = new V().copy(a);
		a.set(9, 9, 9);
		vecIs(t, b, 1, 3, 5.5, 'copied');
	});

	test('math/vector3', 'set accepts three numbers or another vector', function (t) {
		vecIs(t, new V().set(1, 2, 3), 1, 2, 3, 'from numbers');
		vecIs(t, new V().set(new V(4, 5, 6)), 4, 5, 6, 'from vector');
	});

	test('math/vector3', 'add / addInPlace / addInto', function (t) {
		vecIs(t, new V(1, 3, 5.5).add(new V(-2, 0.5, 4)), -1, 3.5, 9.5, 'add');
		vecIs(t, new V(1, 3, 5.5).addInPlace(new V(-2, 0.5, 4)), -1, 3.5, 9.5, 'addInPlace');
		vecIs(t, V.addInto(new V(), new V(1, 3, 5.5), new V(-2, 0.5, 4)), -1, 3.5, 9.5, 'addInto');
	});

	test('math/vector3', 'sub / subInPlace / subInto', function (t) {
		vecIs(t, new V(1, 3, 5.5).sub(new V(-2, 0.5, 4)), 3, 2.5, 1.5, 'sub');
		vecIs(t, new V(1, 3, 5.5).subInPlace(new V(-2, 0.5, 4)), 3, 2.5, 1.5, 'subInPlace');
		vecIs(t, V.subInto(new V(), new V(1, 3, 5.5), new V(-2, 0.5, 4)), 3, 2.5, 1.5, 'subInto');
	});

	test('math/vector3', 'scale / scaleInPlace / scaleInto', function (t) {
		vecIs(t, new V(1, 2, 3).scale(2), 2, 4, 6, 'scale');
		vecIs(t, new V(1, 2, 3).scaleInPlace(2), 2, 4, 6, 'scaleInPlace');
		vecIs(t, V.scaleInto(new V(), new V(1, 2, 3), 3), 3, 6, 9, 'scaleInto');
	});

	test('math/vector3', 'addScaledInPlace (the integration primitive)', function (t) {
		vecIs(t, new V(1, 1, 1).addScaledInPlace(new V(2, 4, 6), 0.5), 2, 3, 4, 'addScaled');
	});

	test('math/vector3', 'component-wise multiply', function (t) {
		vecIs(t, new V(2, 3, 4).multiplyInPlace(new V(5, 6, 7)), 10, 18, 28, 'multiplyInPlace');
	});

	test('math/vector3', 'dot', function (t) {
		t.checkEqual(new V(1, 2, 3).dot(new V(4, 5, 6)), 32, 'dot');
		t.checkEqual(new V(1, 0, 0).dot(new V(0, 1, 0)), 0, 'perpendicular');
	});

	test('math/vector3', 'cross follows the right-hand rule', function (t) {
		vecIs(t, new V(1, 0, 0).cross(new V(0, 1, 0)), 0, 0, 1, 'x cross y');
		vecIs(t, new V(0, 1, 0).cross(new V(0, 0, 1)), 1, 0, 0, 'y cross z');
		vecIs(t, new V(0, 0, 1).cross(new V(1, 0, 0)), 0, 1, 0, 'z cross x');
	});

	test('math/vector3', 'cross is anticommutative', function (t) {
		var a = new V(1, 2, 3), b = new V(4, 5, 6);
		var ab = V.crossInto(new V(), a, b), ba = V.crossInto(new V(), b, a);
		vecIs(t, ab, -ba.x, -ba.y, -ba.z, 'a x b === -(b x a)');
	});

	test('math/vector3', 'crossInto is correct when out aliases an input', function (t) {
		var a = new V(1, 2, 3), b = new V(4, 5, 6);
		var expected = V.crossInto(new V(), a, b);

		var aliasA = new V(1, 2, 3);
		V.crossInto(aliasA, aliasA, b);
		vecIs(t, aliasA, expected.x, expected.y, expected.z, 'out === a');

		var aliasB = new V(4, 5, 6);
		V.crossInto(aliasB, a, aliasB);
		vecIs(t, aliasB, expected.x, expected.y, expected.z, 'out === b');
	});

	test('math/vector3', 'crossInPlace of a vector with itself gives zero', function (t) {
		var v = new V(1, 2, 3);
		v.crossInPlace(v);
		vecIs(t, v, 0, 0, 0, 'v x v');
	});

	test('math/vector3', 'length / lengthSquared', function (t) {
		t.checkEqual(new V(3, 4, 0).lengthSquared(), 25, 'lengthSquared');
		t.checkEqual(new V(3, 4, 0).length(), 5, 'length');
	});

	test('math/vector3', 'normalize produces unit length', function (t) {
		var v = new V(3, 4, 0).normalizeInPlace();
		t.check(v.length(), 1, 1e-15, 'unit length');
		t.check(v.x, 0.6, 1e-15, 'x'); t.check(v.y, 0.8, 1e-15, 'y');
		vecIs(t, V.normalizeInto(new V(), new V(3, 4, 0)), 0.6, 0.8, 0, 'normalizeInto');
	});

	test('math/vector3', 'normalizing a zero vector yields zero, not NaN', function (t) {
		var v = new V(0, 0, 0).normalizeInPlace();
		t.checkTrue(v.isFinite(), 'stays finite');
		t.checkTrue(v.isZero(), 'stays zero');
	});

	test('math/vector3', 'distanceTo / distanceSquared', function (t) {
		t.checkEqual(new V(0, 0, 0).distanceTo(new V(3, 4, 0)), 5, 'distanceTo');
		t.checkEqual(new V(0, 0, 0).distanceSquared(new V(3, 4, 0)), 25, 'distanceSquared');
	});

	test('math/vector3', 'findOrthogonal is perpendicular and unit for any input', function (t) {
		var inputs = [
			new V(1, 0, 0), new V(0, 1, 0), new V(0, 0, 1),
			new V(-1, 0, 0), new V(0, -1, 0), new V(0, 0, -1),
			new V(1, 1, 1), new V(0.001, 5, 0.002), new V(-3, 0.0001, 7)
		];
		for (var i = 0; i < inputs.length; i++) {
			var n = V.normalizeInto(new V(), inputs[i]);
			var o = new V().findOrthogonal(n);
			t.check(o.dot(n), 0, 1e-15, 'perpendicular to ' + inputs[i].toString());
			t.check(o.length(), 1, 1e-15, 'unit for ' + inputs[i].toString());
		}
	});

	test('math/vector3', 'shared methods allocate, leaving the receiver untouched', function (t) {
		var v = new V(1, 2, 3), o = new V(1, 1, 1);
		t.checkTrue(v.add(o) !== v, 'add returns new');
		t.checkTrue(v.sub(o) !== v, 'sub returns new');
		t.checkTrue(v.scale(2) !== v, 'scale returns new');
		t.checkTrue(v.normalize() !== v, 'normalize returns new');
		t.checkTrue(v.cross(o) !== v, 'cross returns new');
		vecIs(t, v, 1, 2, 3, 'receiver untouched');
	});

	test('math/vector3', 'additions never allocate', function (t) {
		var v = new V(1, 2, 3), o = new V(1, 1, 1), out = new V();
		t.checkEqual(v.addInPlace(o), v, 'addInPlace');
		t.checkEqual(v.subInPlace(o), v, 'subInPlace');
		t.checkEqual(v.scaleInPlace(2), v, 'scaleInPlace');
		t.checkEqual(v.normalizeInPlace(), v, 'normalizeInPlace');
		t.checkEqual(v.crossInPlace(o), v, 'crossInPlace');
		t.checkEqual(v.addScaledInPlace(o, 0.5), v, 'addScaledInPlace');
		t.checkEqual(V.addInto(out, o, o), out, 'addInto');
		t.checkEqual(V.crossInto(out, o, o), out, 'crossInto');
		t.checkEqual(V.normalizeInto(out, o), out, 'normalizeInto');
	});

	test('math/vector3', 'accepts any object with x/y/z', function (t) {
		var plain = { x: 4, y: 5, z: 6 };
		vecIs(t, new V(1, 2, 3).addInPlace(plain), 5, 7, 9, 'addInPlace plain');
		t.checkEqual(new V(1, 2, 3).dot(plain), 32, 'dot plain');
	});
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner);
