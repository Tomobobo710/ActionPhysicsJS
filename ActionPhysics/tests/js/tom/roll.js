(function (Runner, U) {
	Runner.suite('tom');

	var SHOVE_AT = 90, TOTAL = 420;

	function rollTest(name, page, build, axis, desc) {
		Runner.test('roll to rest', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);
			var b = build(t, w);
			var shoved = false;
			t.onTick(function (world, tick) {
				if (tick === SHOVE_AT && !shoved) { shoved = true; b.angular_velocity.set(axis[0] * 6, axis[1] * 6, axis[2] * 6); }
			});
			var run = 0, HOLD = 20;
			t.expect('after the shove, rolls to a stop within ' + TOTAL + ' ticks', function () {
				if (!shoved) return { ok: false, detail: 'waiting for shove @tick ' + SHOVE_AT };
				var sv = U.speed(b), sw = U.spin(b);
				if (sv < 0.05 && sw < 0.05) run++; else run = 0;
				return { ok: run >= HOLD, detail: '|v|=' + sv.toFixed(3) + ' |w|=' + sw.toFixed(3) + ' rest=' + run + '/' + HOLD };
			});
			t.simulate(w, TOTAL);
		}, { visual: true, steps: TOTAL, page: page, description: desc });
	}

	rollTest('sphere — shoved, rolls to rest', 'sphere',
		function (t, w) { return t.sphere(w, 0.4, 1, { pos: [0, 0.4, 0], friction: U.MAT.friction, angular_friction: U.MAT.angular_friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#F4D35E' }); },
		[1, 0, 0],
		"A resting sphere is shoved into a roll (spun about a horizontal axis). PASS: rolling resistance " +
		"brings |w| back to ~0 and it stops creeping. A sphere that rolls forever is the regression.");

	rollTest('cylinder on side — shoved, rolls to rest', 'cylinder',
		function (t, w) { return t.cylinder(w, 0.4, 1, 1, { pos: [0, 0.4, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 2), friction: U.MAT.friction, angular_friction: U.MAT.angular_friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#FFAA00' }); },
		[1, 0, 0],
		"A cylinder lying on its side is shoved into a roll along its barrel. PASS: it rolls, then comes " +
		"to rest within the budget rather than trundling on indefinitely.");

	rollTest('capsule on side — shoved, rolls to rest', 'capsule',
		function (t, w) { return t.capsule(w, 0.4, 2, 1, { pos: [0, 0.4, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 2), friction: U.MAT.friction, angular_friction: U.MAT.angular_friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#45B7D1' }); },
		[1, 0, 0],
		"A capsule lying on its side is shoved into a roll along its barrel. PASS: it rolls, then comes to " +
		"rest rather than rolling or wobbling indefinitely.");
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
