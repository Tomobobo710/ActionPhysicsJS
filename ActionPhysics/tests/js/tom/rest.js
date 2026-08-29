(function (Runner, U) {
	Runner.suite('tom');

	var R = 0.4, HH = 0.5;
	var SHOVE_AT = 120;
	var TOTAL = 420;

	function tipAndRim(b, mk) {
		var tf = b.getTransform(), tmp = mk(0, 0, 0), out = mk(0, 0, 0);
		tmp.set(0, HH, 0); tf.transformPointInto(tmp, out); var tipY = out.y;
		var lowRim = Infinity;
		for (var k = 0; k < 64; k++) {
			var a = (k / 64) * Math.PI * 2;
			tmp.set(Math.cos(a) * R, -HH, Math.sin(a) * R);
			tf.transformPointInto(tmp, out);
			if (out.y < lowRim) lowRim = out.y;
		}
		return { tipY: tipY, rimY: lowRim };
	}

	Runner.test('cone', 'cone rolls to rest flush on its slant (tip & rim coplanar)', function (t) {
		t.log('Drop the cone on its side, let it settle, then shove it and watch it come back to rest.');

		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);

		var cone = t.cone(w, R, HH, 1, { pos: [0, 0.6, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 2), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#FF8C42' });

		var shoved = false;
		t.onTick(function (world, tick) {
			if (tick === SHOVE_AT && !shoved) {
				shoved = true;
				cone.angular_velocity.set(6, 0, 0);
				cone.linear_velocity.z += 0.3;
			}
		});

		var TOL = 0.02, run = 0, HOLD = 20;
		t.expect('after the shove, settles flush on its slant (|tipY − rimY| < ' + TOL + ')', function () {
			if (!shoved) return { ok: false, detail: 'waiting for shove @tick ' + SHOVE_AT };
			var g = tipAndRim(cone, t.vec), sw = U.spin(cone), sv = U.speed(cone);
			var flush = Math.abs(g.tipY - g.rimY) < TOL;
			var onFloor = Math.abs(g.rimY) < 0.05 && Math.abs(g.tipY) < 0.05;
			var still = sv < 0.05 && sw < 0.05;
            if (flush && onFloor && still) run++; else run = 0;
			return { ok: run >= HOLD, detail: 'tipY=' + g.tipY.toFixed(3) + ' rimY=' + g.rimY.toFixed(3) + ' |v|=' + sv.toFixed(3) + ' |w|=' + sw.toFixed(3) + ' rest=' + run + '/' + HOLD };
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'cone',
		description:
			"A cone is dropped on its side and settles lying on its slant. At tick " + SHOVE_AT + " it gets a " +
			"deliberate angular shove (rolled about a horizontal axis). PASS: it rolls, then comes back to REST " +
			"flush on its slant — proven geometrically by its tip and the lowest point of its base rim ending at " +
			"the SAME height on the floor (within " + 0.02 + "). A cone that never re-settles, or settles in a " +
			"cocked pose (tip and rim at different heights), fails."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
