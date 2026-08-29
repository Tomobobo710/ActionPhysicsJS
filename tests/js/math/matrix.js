(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var M3 = AP.Matrix3, M4 = AP.Matrix4, Q = AP.Quaternion, V = AP.Vector3, S = AP.Scalar;
	var DESC = "Matrix3 is the inertia-tensor type; Matrix4 is the rigid transform (rotation plus " +
		"translation, never scale). Checked for agreement with the quaternions they are built from, and " +
		"for exact round-trips through inversion.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'matrix', description: DESC }); }

	var M3_KEYS = ['e00', 'e01', 'e02', 'e10', 'e11', 'e12', 'e20', 'e21', 'e22'];

	function vecIs(t, v, x, y, z, label) {
		t.check(v.x, x, 1e-13, label + '.x');
		t.check(v.y, y, 1e-13, label + '.y');
		t.check(v.z, z, 1e-13, label + '.z');
	}
	function m3Is(t, a, b, tol, label) {
		for (var i = 0; i < M3_KEYS.length; i++) t.check(a[M3_KEYS[i]], b[M3_KEYS[i]], tol, label + '.' + M3_KEYS[i]);
	}

	test('math/matrix3', 'default is identity', function (t) {
		var v = new V(1, 2, 3);
		new M3().transformVector3(v);
		vecIs(t, v, 1, 2, 3, 'identity transform');
	});

	test('math/matrix3', 'setDiagonal builds a scaling matrix', function (t) {
		var v = new V(1, 1, 1);
		new M3().setDiagonal(new V(2, 3, 4)).transformVector3(v);
		vecIs(t, v, 2, 3, 4, 'diagonal scale');
	});

	test('math/matrix3', 'fromQuaternion agrees with the quaternion it came from', function (t) {
		var q = new Q().setAxisAngle(new V(0.3, 0.7, -0.5), 1.1);
		var m = new M3().fromQuaternion(q);
		var srcs = [new V(1, 0, 0), new V(0, 1, 0), new V(0, 0, 1), new V(1, 2, 3)];
		for (var i = 0; i < srcs.length; i++) {
			var byQuat = srcs[i].clone(); q.transformVectorInPlace(byQuat);
			var byMat = srcs[i].clone(); m.transformVector3(byMat);
			vecIs(t, byMat, byQuat.x, byQuat.y, byQuat.z, 'agreement for ' + srcs[i].toString());
		}
	});

	test('math/matrix3', 'a rotation matrix has determinant 1', function (t) {
		var m = new M3().fromQuaternion(new Q().setAxisAngle(new V(1, 2, 3), 0.9));
		t.check(m.determinant(), 1, 1e-14, 'determinant');
	});

	test('math/matrix3', 'transpose of a rotation is its inverse', function (t) {
		var m = new M3().fromQuaternion(new Q().setAxisAngle(new V(0.2, -0.9, 0.4), 2.1));
		var product = new M3().multiplyFrom(m, new M3().transposeInto(m));
		m3Is(t, product, new M3(), 1e-14, 'R * R^T');
	});

	test('math/matrix3', 'invert round-trips', function (t) {
		var m = new M3().setDiagonal(new V(2, 4, 8));
		m.e01 = 0.3; m.e12 = -0.7;
		var inv = new M3().copy(m);
		t.checkTrue(inv.invert(), 'invert reports success');
		m3Is(t, new M3().multiplyFrom(m, inv), new M3(), 1e-13, 'M * M^-1');
	});

	test('math/matrix3', 'a singular matrix reports failure instead of producing Infinity', function (t) {
		var inv = new M3().setDiagonal(new V(1, 0, 1));
		t.checkEqual(inv.invert(), false, 'reports singular');
		t.checkTrue(inv.isFinite(), 'result stays finite');
	});

	test('math/matrix3', 'multiply is associative', function (t) {
		var a = new M3().fromQuaternion(new Q().setAxisAngle(new V(1, 0, 0), 0.4));
		var b = new M3().fromQuaternion(new Q().setAxisAngle(new V(0, 1, 0), 0.8));
		var c = new M3().fromQuaternion(new Q().setAxisAngle(new V(0, 0, 1), 1.2));
		var left = new M3().multiplyFrom(new M3().multiplyFrom(a, b), c);
		var right = new M3().multiplyFrom(a, new M3().multiplyFrom(b, c));
		m3Is(t, left, right, 1e-14, '(ab)c === a(bc)');
	});

	test('math/matrix3', 'multiplyFrom is correct when the output aliases an input', function (t) {
		var a = new M3().fromQuaternion(new Q().setAxisAngle(new V(1, 1, 0), 0.5));
		var b = new M3().fromQuaternion(new Q().setAxisAngle(new V(0, 1, 1), 1.3));
		var expected = new M3().multiplyFrom(a, b);
		var alias = new M3().copy(a);
		alias.multiplyFrom(alias, b);
		m3Is(t, alias, expected, 1e-15, 'out === a');
	});

	function xform(m, x, y, z) {
		var out = [0, 0, 0, 0];
		M4.multiplyVector(out, m, [x, y, z, 1]);
		return { x: out[0], y: out[1], z: out[2] };
	}

	function xformDir(m, x, y, z) {
		return M4.transformNormal({ x: x, y: y, z: z }, m);
	}

	test('math/matrix4', 'fromRotationTranslation places the translation', function (t) {
		var m = M4.createPrecise();
		M4.fromRotationTranslation(m, new Q(), new V(5, 6, 7));
		vecIs(t, xform(m, 0, 0, 0), 5, 6, 7, 'origin maps to translation');
	});

	test('math/matrix4', 'fromRotationTranslation applies rotation then translation', function (t) {
		var m = M4.createPrecise();
		M4.fromRotationTranslation(m, new Q().setAxisAngle(new V(0, 1, 0), S.HALF_PI), new V(10, 0, 0));
		vecIs(t, xform(m, 1, 0, 0), 10, 0, -1, 'rotate then translate');
	});

	test('math/matrix4', 'transformNormal ignores the translation', function (t) {
		var m = M4.createPrecise();
		M4.fromRotationTranslation(m, new Q(), new V(100, 200, 300));
		vecIs(t, xformDir(m, 1, 0, 0), 1, 0, 0, 'direction untranslated');
	});

	test('math/matrix4', 'invert round-trips a point', function (t) {
		var m = M4.createPrecise(), inv = M4.createPrecise();
		M4.fromRotationTranslation(m, new Q().setAxisAngle(new V(0.5, 0.2, -0.8), 1.4), new V(3, -4, 5));
		M4.invert(inv, m);
		var p = xform(m, 7, 8, 9);
		vecIs(t, xform(inv, p.x, p.y, p.z), 7, 8, 9, 'round trip');
	});

	test('math/matrix4', 'M * M^-1 is the identity transform', function (t) {
		var m = M4.createPrecise(), inv = M4.createPrecise(), product = M4.createPrecise();
		M4.fromRotationTranslation(m, new Q().setAxisAngle(new V(1, 1, 1), 2.0), new V(-2, 6, 1));
		M4.invert(inv, m);
		M4.multiply(product, m, inv);
		vecIs(t, xform(product, 4, 5, 6), 4, 5, 6, 'identity');
	});

	test('math/matrix4', 'composed transforms apply in the right order', function (t) {
		var rot = M4.createPrecise(), trans = M4.createPrecise(), composed = M4.createPrecise();
		M4.fromRotationTranslation(rot, new Q().setAxisAngle(new V(0, 1, 0), S.HALF_PI), new V(0, 0, 0));
		M4.fromRotationTranslation(trans, new Q(), new V(10, 0, 0));
		M4.multiply(composed, trans, rot);
		vecIs(t, xform(composed, 1, 0, 0), 10, 0, -1, 'rotate then translate');
	});

	test('math/matrix4', 'the translation is readable from the matrix', function (t) {
		var m = M4.createPrecise();
		M4.fromRotationTranslation(m, new Q().setAxisAngle(new V(0, 0, 1), 0.6), new V(1, 2, 3));
		vecIs(t, { x: m[12], y: m[13], z: m[14] }, 1, 2, 3, 'translation column');
	});

	test('math/matrix4', 'a rigid transform preserves distances', function (t) {
		var m = M4.createPrecise();
		M4.fromRotationTranslation(m, new Q().setAxisAngle(new V(0.1, 0.9, 0.4), 1.9), new V(5, -3, 2));
		var a = new V(1, 2, 3), b = new V(-4, 0, 7), before = a.distanceTo(b);
		var ta = xform(m, a.x, a.y, a.z), tb = xform(m, b.x, b.y, b.z);
		var after = Math.sqrt(Math.pow(ta.x - tb.x, 2) + Math.pow(ta.y - tb.y, 2) + Math.pow(ta.z - tb.z, 2));
		t.check(after, before, 1e-13, 'distance preserved');
	});

	test('math/matrix4', 'fromQuat agrees with the quaternion it came from', function (t) {
		var q = new Q().setAxisAngle(new V(0.3, 0.7, -0.5), 1.1);
		var m = M4.createPrecise();
		M4.fromQuat(m, q);

		var srcs = [new V(1, 0, 0), new V(0, 1, 0), new V(0, 0, 1), V.normalizeInto(new V(), new V(1, 2, 3))];
		for (var i = 0; i < srcs.length; i++) {
			var byQuat = srcs[i].clone();
			q.transformVectorInPlace(byQuat);
			var byMat = xformDir(m, srcs[i].x, srcs[i].y, srcs[i].z);
			vecIs(t, byMat, byQuat.x, byQuat.y, byQuat.z, 'agreement for ' + srcs[i].toString());
		}
	});
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner);
