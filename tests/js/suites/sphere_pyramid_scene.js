(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "A 5x5 static sphere base with 4x4/3x3/2x2 dynamic levels stacked above, then a heavy " +
		"sphere dropped from y=20 with a downward impulse.";

	Runner.test('collision/sphere-pyramid-scene', 'sphere pyramid with a heavy sphere dropped on top', function (t) {
		var world = t.makeWorld();
		var i, j;
		for (i = 0; i < 5; i++) for (j = 0; j < 5; j++) t.sphere(world, 1, 0, { pos: [i * 2 - 2.5, 0, j * 2 - 2.5], color: '#8899AA' });
		for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) t.sphere(world, 1, 10, { pos: [i * 2 - 1.5, 2, j * 2 - 1.5], color: '#8899AA' });
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) t.sphere(world, 1, 10, { pos: [i * 2 - 0.5, 4, j * 2 - 0.5], color: '#8899AA' });
		for (i = 0; i < 2; i++) for (j = 0; j < 2; j++) t.sphere(world, 1, 10, { pos: [i * 2 + 0.5, 6, j * 2 + 0.5], color: '#8899AA' });

		var heavy = t.sphere(world, 1.5, 300, { pos: [1.5, 20, 1.5], color: '#E85D4D' });
		heavy.applyImpulse(t.vec(0, -10, 0));

		var ticks = 0, everNonFinite = false;
		t.onTick(function (world, tick) {
			ticks = tick;
			if (!isFinite(heavy.position.x) || !isFinite(heavy.position.y) || !isFinite(heavy.position.z)) everNonFinite = true;
		});
		t.expect('the pile stays numerically finite for the WHOLE run (checked every tick, not just the end)', function () {
			if (ticks < 600) return false;
			return { ok: !everNonFinite, detail: everNonFinite ? 'went non-finite at some point' : 'heavy.y=' + heavy.position.y.toFixed(2) };
		});

		t.simulate(world, 600);
	}, { visual: true, steps: 600, page: 'sphere-pyramid-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
