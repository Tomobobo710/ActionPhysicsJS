(function (Runner) {
	Runner.suite('math');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var S = AP.Scalar;
	var DESC = "The deterministic sin/cos/tan/asin/acos/atan/atan2 the engine uses instead of Math.*, " +
		"which the JS spec leaves implementation-defined and which differ in low bits between browsers. " +
		"Checked for accuracy against Math.* and for bit-exact symmetry.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'scalar', description: DESC }); }

	// Sweep a function against a reference; returns the worst absolute error and where it occurred.
	function worst(ours, ref, lo, hi, samples) {
		var w = 0, at = lo;
		for (var i = 0; i <= samples; i++) {
			var x = lo + (hi - lo) * i / samples;
			var e = Math.abs(ours(x) - ref(x));
			if (e > w) { w = e; at = x; }
		}
		return { err: w, at: at };
	}

	test('math/scalar', 'sin matches Math.sin across several turns', function (t) {
		var r = worst(S.sin, Math.sin, -4 * Math.PI, 4 * Math.PI, 100000);
		t.checkTrue(r.err < 1e-11, 'worst err ' + r.err.toExponential(2) + ' at x=' + r.at.toFixed(4));
	});

	test('math/scalar', 'cos matches Math.cos across several turns', function (t) {
		var r = worst(S.cos, Math.cos, -4 * Math.PI, 4 * Math.PI, 100000);
		t.checkTrue(r.err < 1e-11, 'worst err ' + r.err.toExponential(2) + ' at x=' + r.at.toFixed(4));
	});

	test('math/scalar', 'atan matches Math.atan over a wide range', function (t) {
		var r = worst(S.atan, Math.atan, -50, 50, 100000);
		t.checkTrue(r.err < 1e-14, 'worst err ' + r.err.toExponential(2) + ' at x=' + r.at.toFixed(4));
	});

	test('math/scalar', 'asin / acos match Math', function (t) {
		var a = worst(S.asin, Math.asin, -0.9999, 0.9999, 100000);
		t.checkTrue(a.err < 1e-13, 'asin worst err ' + a.err.toExponential(2));
		var c = worst(S.acos, Math.acos, -0.9999, 0.9999, 100000);
		t.checkTrue(c.err < 1e-13, 'acos worst err ' + c.err.toExponential(2));
	});

	test('math/scalar', 'atan2 matches Math.atan2 in all four quadrants', function (t) {
		var pts = [[1, 1], [1, -1], [-1, 1], [-1, -1], [0, 1], [0, -1], [1, 0], [-1, 0], [3, -7], [-0.001, 5]];
		for (var i = 0; i < pts.length; i++) {
			var y = pts[i][0], x = pts[i][1];
			t.check(S.atan2(y, x), Math.atan2(y, x), 1e-14, 'atan2(' + y + ',' + x + ')');
		}
	});

	// Range reduction is where naive implementations fall apart.
	test('math/scalar', 'sin/cos stay accurate for large angles', function (t) {
		var xs = [100, 1000, -5000, 12345.6789];
		for (var i = 0; i < xs.length; i++) {
			t.check(S.sin(xs[i]), Math.sin(xs[i]), 1e-9, 'sin(' + xs[i] + ')');
			t.check(S.cos(xs[i]), Math.cos(xs[i]), 1e-9, 'cos(' + xs[i] + ')');
		}
	});

	test('math/scalar', 'sin is exactly odd, cos exactly even', function (t) {
		var xs = [0.1, 0.7, 1.3, 2.9, 5.5, 11.2, 100.4];
		for (var i = 0; i < xs.length; i++) {
			t.checkEqual(S.sin(-xs[i]), -S.sin(xs[i]), 'sin(-' + xs[i] + ') === -sin(' + xs[i] + ')');
			t.checkEqual(S.cos(-xs[i]), S.cos(xs[i]), 'cos(-' + xs[i] + ') === cos(' + xs[i] + ')');
		}
	});

	test('math/scalar', 'atan and asin are exactly odd', function (t) {
		var xs = [0.1, 0.7, 1.3, 2.9, 5.5];
		for (var i = 0; i < xs.length; i++) {
			t.checkEqual(S.atan(-xs[i]), -S.atan(xs[i]), 'atan(-' + xs[i] + ')');
		}
		var ys = [0.1, 0.35, 0.7, 0.95, 0.999];
		for (var j = 0; j < ys.length; j++) {
			t.checkEqual(S.asin(-ys[j]), -S.asin(ys[j]), 'asin(-' + ys[j] + ')');
		}
	});

	// Callers reach acos/asin through dot products of unit vectors, where |x| can exceed 1 by a few ULP
	// from rounding alone. That is not an error and must not produce NaN.
	test('math/scalar', 'acos and asin clamp instead of returning NaN', function (t) {
		t.checkEqual(S.acos(1.0000000002), 0, 'acos just above 1');
		t.checkEqual(S.acos(-1.0000000002), S.PI, 'acos just below -1');
		t.checkEqual(S.asin(1.0000000002), S.HALF_PI, 'asin just above 1');
		t.checkEqual(S.asin(-1.0000000002), -S.HALF_PI, 'asin just below -1');
	});

	test('math/scalar', 'exact values at the cardinal angles', function (t) {
		t.checkEqual(S.sin(0), 0, 'sin(0)');
		t.check(S.sin(S.HALF_PI), 1, 1e-15, 'sin(PI/2)');
		t.checkEqual(S.cos(0), 1, 'cos(0)');
		t.check(S.cos(S.PI), -1, 1e-15, 'cos(PI)');
		t.checkEqual(S.atan(0), 0, 'atan(0)');
		t.checkEqual(S.atan2(0, 1), 0, 'atan2(0,1)');
	});

	test('math/scalar', 'wrapAngle brings any angle into [-PI, PI] without changing it', function (t) {
		var xs = [0, 3, -3, 7, -7, 100, -100, 12345.678];
		for (var i = 0; i < xs.length; i++) {
			var w = S.wrapAngle(xs[i]);
			t.checkTrue(w >= -S.PI - 1e-12 && w <= S.PI + 1e-12, 'wrap(' + xs[i] + ') = ' + w.toFixed(6));
			t.check(S.sin(w), Math.sin(xs[i]), 1e-9, 'sin preserved through wrap(' + xs[i] + ')');
		}
	});

	test('math/scalar', 'clamp', function (t) {
		t.checkEqual(S.clamp(5, 0, 1), 1);
		t.checkEqual(S.clamp(-5, 0, 1), 0);
		t.checkEqual(S.clamp(0.5, 0, 1), 0.5);
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner);
