// Ported from Goblin's tests/js/chandler/constraint-point.js. An 11-plank bridge linked end-to-end
// by paired PointConstraints (front + back), anchored at both ends, over a ground plane. A sphere
// is dropped onto it every 60 ticks. Watch-only scene (matching Goblin's own), strengthened here to
// check the bridge stays numerically stable for the whole run.
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "An 11-plank bridge is linked end-to-end with paired PointConstraints (breaking_threshold " +
		"8), anchored at both ends over a ground plane, and a sphere is dropped onto it every 60 ticks.";

	Runner.test('collision/constraint-point-scene', 'plank bridge held together by point constraints', function (t) {
		var world = t.makeWorld();
		t.plane(world, 'y', 20, 20, 0, { pos: [0, -5, 0], color: '#243B2A' });

		var previousPlank = null,
			plankCount = 11,
			plankSeparation = 0.2,
			plankWidth = 2,
			plankHeight = 0.4,
			plankLength = 6,
			plankMass = 1,
			plankSpace = plankWidth + plankSeparation,
			i, constraint,
			rightFront = t.vec(plankSpace / 2, 0, plankLength / -6),
			leftFront = t.vec(plankSpace / -2, 0, plankLength / -6),
			rightBack = t.vec(plankSpace / 2, 0, plankLength / 6),
			leftBack = t.vec(plankSpace / -2, 0, plankLength / 6);

		for (i = 0; i < plankCount; i++) {
			var isAnchor = (i === 0 || i + 1 === plankCount);
			var plank = t.box(world, plankWidth / 2, plankHeight / 2, plankLength / 2, isAnchor ? 0 : plankMass, {
				pos: [i * plankSpace - (plankCount / 2 * plankSpace) + plankWidth / 2, 3, 0],
				color: isAnchor ? '#888' : '#B08968'
			});

			if (previousPlank) {
				constraint = new AP.PointConstraint(previousPlank, plank, rightBack, leftBack);
				constraint.breaking_threshold = 8;
				world.addConstraint(constraint);

				constraint = new AP.PointConstraint(previousPlank, plank, rightFront, leftFront);
				constraint.breaking_threshold = 8;
				world.addConstraint(constraint);
			}

			previousPlank = plank;
		}

		var ticks = 0, everNonFinite = false;
		t.onTick(function (world, tick) {
			ticks = tick;
			if (tick % 60 === 1) t.sphere(world, 1, 1, { pos: [Math.random() * 4 - 2, 8, 0], color: '#4af' });
			for (var i = 0; i < world.bodies.length; i++) {
				var p = world.bodies[i].position;
				if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) { everNonFinite = true; break; }
			}
		});
		t.expect('the bridge stays numerically finite for the WHOLE run (checked every tick, not just the end)', function () {
			if (ticks < 600) return false;
			return { ok: !everNonFinite, detail: everNonFinite ? 'went non-finite at some point' : 'all bodies finite' };
		});

		t.simulate(world, 600);
	}, { visual: true, steps: 600, page: 'constraint-point-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
