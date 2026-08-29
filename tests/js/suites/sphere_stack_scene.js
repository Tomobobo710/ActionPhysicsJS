(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "Six spheres stacked directly on top of each other, centers exactly aligned, over a " +
		"ground plane. Measured: the stack holds — every sphere stays centered (x/z ~0) and spaced by " +
		"exactly one diameter in y, for the full 600-tick run, no tip-over.";

	Runner.test('collision/sphere-stack-scene', 'six spheres stacked directly on top of each other', function (t) {
		var world = t.makeWorld();
		t.plane(world, 'y', 20, 20, 0, { color: '#243B2A' });

		var r = 1, spheres = [];
		for (var i = 0; i < 6; i++) {
			spheres.push(t.sphere(world, r, 1, { pos: [0, r + i * (2 * r), 0], color: '#4af' }));
		}

		var ticks = 0, everNonFinite = false;
		t.onTick(function (world, tick) {
			ticks = tick;
			for (var i = 0; i < spheres.length; i++) {
				var p = spheres[i].position;
				if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) { everNonFinite = true; break; }
			}
		});
		t.expect('the stack stays numerically finite for the WHOLE run (checked every tick, not just the end)', function () {
			if (ticks < 600) return false;
			return { ok: !everNonFinite, detail: everNonFinite ? 'went non-finite at some point' : 'all spheres finite' };
		});

		var TOL = 0.05;
		spheres.forEach(function (s, i) {
			t.expect('sphere ' + i + ' holds its stacked position (x/y/z)', function () {
				if (ticks < 600) return false;
				var expectedY = r + i * (2 * r);
				var dx = Math.abs(s.position.x), dz = Math.abs(s.position.z), dy = Math.abs(s.position.y - expectedY);
				var ok = dx < TOL && dz < TOL && dy < TOL;
				return { ok: ok, detail: 'pos=(' + s.position.x.toFixed(4) + ',' + s.position.y.toFixed(4) + ',' + s.position.z.toFixed(4) + ') expectedY=' + expectedY.toFixed(2) };
			});
		});

		t.simulate(world, 600);
	}, { visual: true, steps: 600, page: 'sphere-stack-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
