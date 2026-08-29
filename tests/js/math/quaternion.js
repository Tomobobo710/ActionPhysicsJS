(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var Q = AP.Quaternion, V = AP.Vector3, S = AP.Scalar;
	var DESC = "Unit quaternions representing rotation: construction from axis-angle, multiplication, " +
		"inversion, applying a rotation to a vector, and angle measurement. Includes the drift tests " +
		"that justify normalize() existing at all.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'quaternion', description: DESC }); }

	function vecIs(t, v, x, y, z, label) {
		t.check(v.x, x, 1e-14, label + '.x');
		t.check(v.y, y, 1e-14, label + '.y');
		t.check(v.z, z, 1e-14, label + '.z');
	}

	test('math/quaternion', 'default is identity', function (t) {
		var q = new Q();
		t.checkEqual(q.x, 0, 'x'); t.checkEqual(q.y, 0, 'y');
		t.checkEqual(q.z, 0, 'z'); t.checkEqual(q.w, 1, 'w');
	});

	test('math/quaternion', 'identity rotation leaves a vector untouched', function (t) {
		var v = new V(1, 2, 3);
		new Q().transformVectorInPlace(v);
		vecIs(t, v, 1, 2, 3, 'unchanged');
	});

	test('math/quaternion', 'setAxisAngle produces a unit quaternion', function (t) {
		t.check(new Q().setAxisAngle(new V(0, 1, 0), 1.234).length(), 1, 1e-15, 'unit length');
	});

	test('math/quaternion', 'setAxisAngle normalizes a non-unit axis', function (t) {
		var a = new Q().setAxisAngle(new V(0, 5, 0), 0.7);
		var b = new Q().setAxisAngle(new V(0, 1, 0), 0.7);
		t.check(a.x, b.x, 1e-15, 'x'); t.check(a.y, b.y, 1e-15, 'y');
		t.check(a.z, b.z, 1e-15, 'z'); t.check(a.w, b.w, 1e-15, 'w');
	});

	test('math/quaternion', 'a zero axis gives identity rather than NaN', function (t) {
		var q = new Q().setAxisAngle(new V(0, 0, 0), 1.0);
		t.checkTrue(q.isFinite(), 'finite');
		t.checkEqual(q.w, 1, 'identity');
	});

	test('math/quaternion', '90 degrees about Y takes +X to -Z', function (t) {
		var v = new V(1, 0, 0);
		new Q().setAxisAngle(new V(0, 1, 0), S.HALF_PI).transformVectorInPlace(v);
		vecIs(t, v, 0, 0, -1, 'x -> -z');
	});

	test('math/quaternion', '90 degrees about X takes +Y to +Z', function (t) {
		var v = new V(0, 1, 0);
		new Q().setAxisAngle(new V(1, 0, 0), S.HALF_PI).transformVectorInPlace(v);
		vecIs(t, v, 0, 0, 1, 'y -> z');
	});

	test('math/quaternion', 'rotation preserves length', function (t) {
		var v = new V(4, -5, 6), before = v.length();
		new Q().setAxisAngle(new V(1, 2, 3), 2.4).transformVectorInPlace(v);
		t.check(v.length(), before, 1e-14, 'length preserved');
	});

	test('math/quaternion', 'rotating by q then q-inverse returns the original', function (t) {
		var q = new Q().setAxisAngle(new V(0.3, 0.5, -0.8), 1.7);
		var inv = new Q().invertQuaternion(q);
		var v = new V(1, 2, 3);
		q.transformVectorInPlace(v);
		inv.transformVectorInPlace(v);
		vecIs(t, v, 1, 2, 3, 'round trip');
	});

	test('math/quaternion', 'four 90-degree turns return to the start', function (t) {
		var step = new Q().setAxisAngle(new V(0, 1, 0), S.HALF_PI), acc = new Q();
		for (var i = 0; i < 4; i++) acc.multiplyInPlace(step);
		var v = new V(1, 2, 3);
		acc.transformVectorInPlace(v);
		vecIs(t, v, 1, 2, 3, 'full turn');
	});

	test('math/quaternion', 'transformVector3Into leaves the input untouched', function (t) {
		var q = new Q().setAxisAngle(new V(0, 1, 0), S.HALF_PI);
		var src = new V(1, 0, 0), out = new V();
		q.transformVectorInto(src, out);
		vecIs(t, src, 1, 0, 0, 'source unchanged');
		vecIs(t, out, 0, 0, -1, 'output rotated');
	});

	test('math/quaternion', 'multiplyQuaternions is correct when the output aliases an input', function (t) {
		var a = new Q().setAxisAngle(new V(0, 1, 0), 0.6);
		var b = new Q().setAxisAngle(new V(1, 0, 0), 0.9);
		var expected = new Q().multiplyQuaternions(a, b);
		var alias = a.clone();
		alias.multiplyQuaternions(alias, b);
		t.check(alias.x, expected.x, 1e-15, 'x'); t.check(alias.y, expected.y, 1e-15, 'y');
		t.check(alias.z, expected.z, 1e-15, 'z'); t.check(alias.w, expected.w, 1e-15, 'w');
	});

	test('math/quaternion', 'normalize restores unit length', function (t) {
		t.check(new Q(0.6, 0.8, 0, 0).normalize().length(), 1, 1e-15, 'unit');
	});

	test('math/quaternion', 'a zero quaternion normalizes to identity, not NaN', function (t) {
		var q = new Q(0, 0, 0, 0).normalize();
		t.checkTrue(q.isFinite(), 'finite');
		t.checkEqual(q.w, 1, 'identity');
	});

	test('math/quaternion', 'length drifts without normalization and holds with it', function (t) {
		var step = new Q().setAxisAngle(new V(0.267, 0.535, 0.802), 0.01);
		var i;

		var drifting = new Q();
		for (i = 0; i < 10000; i++) drifting.multiplyInPlace(step);
		var driftErr = Math.abs(drifting.length() - 1);

		var kept = new Q();
		for (i = 0; i < 10000; i++) kept.multiplyInPlace(step).normalize();
		var keptErr = Math.abs(kept.length() - 1);

		t.checkTrue(keptErr < 1e-14, 'normalized stays unit (err ' + keptErr.toExponential(2) + ')');
		t.checkTrue(keptErr <= driftErr, 'normalizing is never worse (drift err ' + driftErr.toExponential(2) + ')');
	});

	test('math/quaternion', 'a non-unit quaternion scales the vectors it rotates', function (t) {
		var q = new Q().setAxisAngle(new V(0, 1, 0), 0.5);
		q.x *= 1.01; q.y *= 1.01; q.z *= 1.01; q.w *= 1.01;
		var v = new V(1, 0, 0);
		q.transformVectorInPlace(v);
		t.checkTrue(Math.abs(v.length() - 1) > 1e-3, 'length changed: ' + v.length().toFixed(6));

		q.normalize();
		var v2 = new V(1, 0, 0);
		q.transformVectorInPlace(v2);
		t.check(v2.length(), 1, 1e-15, 'length preserved after normalize');
	});

	test('math/quaternion', 'angleBetween', function (t) {
		var a = new Q(), b = new Q().setAxisAngle(new V(0, 1, 0), 1.0);
		t.check(a.angleBetween(b), 1.0, 1e-14, 'identity to 1 rad');
		var q = new Q().setAxisAngle(new V(1, 1, 0), 0.9);
		t.check(q.angleBetween(q), 0, 1e-7, 'self angle is zero');
	});

	test('math/quaternion', 'angleBetween handles double cover', function (t) {
		var q = new Q().setAxisAngle(new V(0, 1, 0), 1.0);
		var neg = new Q(-q.x, -q.y, -q.z, -q.w);
		t.check(q.angleBetween(neg), 0, 1e-7, 'q vs -q');
	});

	test('math/quaternion', 'signedAngleBetween reports direction', function (t) {
		var axis = new V(0, 1, 0), base = new Q();
		var sp = base.signedAngleBetween(new Q().setAxisAngle(axis, 0.7), axis);
		var sm = base.signedAngleBetween(new Q().setAxisAngle(axis, -0.7), axis);
		t.check(Math.abs(sp), 0.7, 1e-13, 'magnitude +');
		t.check(Math.abs(sm), 0.7, 1e-13, 'magnitude -');
		t.checkTrue(sp * sm < 0, 'opposite signs: ' + sp.toFixed(4) + ' vs ' + sm.toFixed(4));
	});
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner);
