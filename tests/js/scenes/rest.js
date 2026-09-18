(function (Runner, U) {

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

		// The cone must not be able to PARK anywhere but on its slant. Dropped on its side it lands on one
		// point of its base rim, with its tip a whole base radius (0.4 here) in the air and its centre of
		// mass half a metre out to the side of that point - a pose the solver must not be able to hold, so
		// gravity tips it onto its slant within the first few ticks. Counted over the WHOLE run rather
		// than judged at the end, because a proportion cannot be satisfied by being briefly right:
		// balancing on the rim for the hundred ticks before the shove reads as ~70%, and the fixed
		// engine reads 95%.
		var TIP_NEAR = 0.05, MIN_SHARE = 0.9;
		var shoved = false;
		var nearCount = 0, samples = 0, firstNear = 0;
		t.onTick(function (world, tick) {
			var g0 = tipAndRim(cone, t.vec);
			samples++;
			if (Math.abs(g0.tipY) < TIP_NEAR) { nearCount++; if (!firstNear) firstNear = tick; }
			if (tick === SHOVE_AT && !shoved) {
				shoved = true;
				cone.angular_velocity.set(6, 0, 0);
				cone.linear_velocity.z += 0.3;
			}
		});

		t.expect('lies on the floor (tip within ' + TIP_NEAR + ') for at least ' + (MIN_SHARE * 100) +
			'% of the run', function () {
			if (samples < TOTAL) return { ok: false, detail: 'tick ' + samples + '/' + TOTAL +
				'  tip at the floor for ' + nearCount + '/' + samples + ' so far' };
			var share = nearCount / samples;
			return {
				ok: share >= MIN_SHARE,
				detail: 'tip at the floor for ' + nearCount + '/' + samples + ' ticks (' +
					(100 * share).toFixed(1) + '%, need ' + (MIN_SHARE * 100) + '%), first @tick ' + firstNear
			};
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
		visual: true, steps: TOTAL, page: 'cone',			description:
			"A cone is dropped on its side and settles lying on its slant. At tick " + SHOVE_AT + " it gets a " +
			"deliberate angular shove (rolled about a horizontal axis). PASS: it lies on the floor with its tip " +
			"down (within 0.05) for at least 90% of the whole run — a cone balanced on its base rim has its tip " +
			"a full radius (0.4) in the air and cannot count — and after the shove it rolls and comes back to " +
			"REST flush on its slant, proven geometrically by its tip and the lowest point of its base rim " +
			"ending at the SAME height on the floor (within " + 0.02 + "). A cone that balances on its rim, " +
			"never re-settles, or settles in a cocked pose (tip and rim at different heights) fails."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
