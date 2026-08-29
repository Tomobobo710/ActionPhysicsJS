// Tom's Suite — ENTANGLEMENT.
// A tall upright box with a sphere resting on top. Once settled, a tiny tap-wait-tap lateral jiggle is
// fired on the box (the shape of a 2-frame movement tap). A correct contact solver lets the sphere just
// settle back down; the regression is the sphere developing its own sideways drift that it never had,
// eventually dragging the box off balance. Oracle: the sphere reseats onto the box soon after the jiggle
// AND shows no self-driven lateral velocity in the late tail.
//
// Fully scripted from tick hooks (deterministic, headless == browser).
(function (Runner, U) {
	Runner.suite('tom');

	var BOX_H = 1.8, BOX_HALF = BOX_H / 2, RADIUS = 0.4;
	var JIGGLE_AT = 40;             // fire the jiggle once the sphere has seated
	var TOTAL = 340;                // JIGGLE_AT + 250 tail + margin

	Runner.test('entanglement', 'sphere on a jiggled box does not self-drift', function (t) {
		t.log('Settle a sphere on a tall box, jiggle the box, watch the sphere for invented drift.');

		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		var box = t.box(w, 0.3, BOX_HALF, 0.3, 10, U.withMat({ pos: [0, BOX_HALF, 0], color: '#3355ff' }));
		var sphere = t.sphere(w, RADIUS, 2, U.withMat({ pos: [0, BOX_H + 0.2 + RADIUS, 0], color: '#0ff' }));

		// tap-wait-tap on the box: 2 ticks +vx, 1 tick idle, 2 ticks -vx, once.
		var phase = 'idle', pt = 0, lastTick = 0;
		t.onTick(function (world, tick) {
			lastTick = tick;
			if (tick === JIGGLE_AT && phase === 'idle') { phase = 'right'; pt = 0; }
			if (phase === 'right') { box.linear_velocity.x = 4.6; if (++pt >= 2) { phase = 'wait'; pt = 0; } }
			else if (phase === 'wait') { if (++pt >= 1) { phase = 'left'; pt = 0; } }
			else if (phase === 'left') { box.linear_velocity.x = -4.6; if (++pt >= 2) { phase = 'done'; } }
		});

		// Metric A: the sphere reseats — its bottom returns near the box top within 60 ticks of the jiggle.
		var landedTick = -1;
		t.expect('sphere reseats within 60 ticks of the jiggle', function (world) {
			// bottom of sphere vs top of box
			var bottom = sphere.position.y - RADIUS, top = box.position.y + BOX_HALF;
			if (landedTick < 0 && lastTick >= JIGGLE_AT && Math.abs(bottom - top) < 0.06 && Math.abs(sphere.linear_velocity.y) < 0.3) landedTick = lastTick;
			var ok = landedTick > 0 && (landedTick - JIGGLE_AT) <= 60;
			return { ok: ok, detail: 'gap=' + (bottom - top).toFixed(3) + (landedTick > 0 ? ' reseat@+' + (landedTick - JIGGLE_AT) : '') };
		});

		// Metric B: no self-driven lateral drift in the late tail (>= 250 ticks after the jiggle).
		var maxLateVx = 0, decided = false;
		t.expect('no self-driven sphere drift in the late tail (|vx| < 0.5)', function (world) {
			var late = lastTick >= JIGGLE_AT + 250;
			if (late) { var a = Math.abs(sphere.linear_velocity.x); if (a > maxLateVx) maxLateVx = a; decided = true; }
			return { ok: decided && maxLateVx < 0.5, detail: 'maxLate|vx|=' + maxLateVx.toFixed(3) + (late ? '' : ' (measuring…)') };
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'entanglement',
		description:
			"A sphere rests on a tall box; the box gets a tiny tap-wait-tap jiggle. A correct solver " +
			"lets the sphere settle back. PASS: the sphere reseats within 60 ticks and shows no self-driven " +
			"sideways velocity (|vx| < 0.5) in the late tail. The regression is the sphere inventing lateral " +
			"drift that slowly topples the box."
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
