// Tom's Suite — SMEAR / FLIP.
// A "box hangs frozen against a surface" contact-solver bug in its portable form: a small box, released
// ALREADY TILTED just above the top corner of a static, upright support box, free-falls, catches the top
// edge, and — on a healthy solver — tips over the corner and slides/rolls off to the floor. The bug is
// the box HANGING frozen up at head height (tiny |vy| and tiny spin, held by a persistent multi-point
// contact whose normal points nearly straight down) for many frames instead of coming down.
//
// Oracle: the box must NOT freeze mid-air (no long stuck run up high) and must reach the floor by the end.
(function (Runner, U) {
	Runner.suite('tom');

	var BOX = 0.8, HALF = BOX / 2, TOTAL = 300;
	// Release orientation that reproduces the hang: box already tilted (~104°), dead still.
	var SEED_Q = [0.2625, 0.5063, 0.7427, -0.3509];

	Runner.test('smear', 'tilted box on a support does not hang frozen mid-air', function (t) {
		t.log('Drop a tilted box onto the top corner of a support box; it must tip off, not stick.');

		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		// Static tall support: center y=0.9, top y=1.8.
		t.box(w, 0.3, 0.9, 0.3, 0, { pos: [0, 0.9, 0], friction: 0.4, restitution: 0, color: '#3355ff' });
		// Falling box released tilted, hanging off the support's top corner (grippy material: friction 3).
		// x=0.4 puts the box's inner face level with the pillar's outer edge (pillar half-width 0.3, box
		// half-extent 0.4), so it catches the corner clearly overhanging and must tumble off rather than
		// balancing on top. The old 0.169 sat close enough to centred that a solver which lands the box
		// squarely - rather than knocking it off - leaves it resting on the pillar, which reads as a
		// failure without being one. Both PGS and XPBD tumble off across x=0.32-0.48.
		var box = t.box(w, HALF, HALF, HALF, 2, {
			pos: [0.4, 0.9 + 2.92, 0.017], rot: SEED_Q, friction: 3, restitution: 0.33, color: '#e0b020'
		});

		// Track the worst "stuck up high" run: box frozen (tiny |vy|, tiny spin) while still above y=0.9,
		// held by a live multi-point contact. A long stuck run is the smear.
		var stuckRun = 0, worstStuck = 0, reachedFloor = false;
		t.onTick(function (world) {
			var vy = Math.abs(box.linear_velocity.y), sw = U.spin(box);
			var pts = contactPoints(world, box);
			var stuck = box.position.y > 0.9 && vy < 0.1 && sw < 0.15 && pts >= 2;
			if (stuck) { stuckRun++; if (stuckRun > worstStuck) worstStuck = stuckRun; } else stuckRun = 0;
			if ((box.position.y - HALF) < 0.06) reachedFloor = true;
		});

		// Count contact-manifold points the box is part of, this tick.
		function contactPoints(world, b) {
			var np = world.narrowphase; if (!np) return 0;
			var iter = np.manifolds.values();
			for (var r = iter.next(); !r.done; r = iter.next()) {
				var m = r.value;
				if ((m.bodyA === b || m.bodyB === b) && m.points.length) return m.points.length;
			}
			return 0;
		}

		t.expect('box does not hang frozen at head height (no long stuck run)', function () {
			// fails the moment a long freeze appears; otherwise green
			return { ok: worstStuck <= 30, detail: 'worstStuckRun=' + worstStuck + ' (limit 30)' };
		});
		t.expect('box comes down to the floor', function () {
			return { ok: reachedFloor, detail: 'y=' + box.position.y.toFixed(3) + (reachedFloor ? ' reached floor' : '') };
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'smear',
		description:
			"A box, released already tilted just above the top corner of a static support box, free-falls and " +
			"catches the edge. PASS: it tips over the corner and comes down to the floor. FAIL: it hangs frozen " +
			"up at head height (tiny velocity and spin, held by a persistent multi-point contact whose normal points nearly straight down) instead of sliding off — a contact-solver hang."
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
