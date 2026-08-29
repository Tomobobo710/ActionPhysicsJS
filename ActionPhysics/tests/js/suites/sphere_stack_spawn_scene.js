(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "Six spheres stacked one at a time: each new sphere spawns above the current top of the " +
		"stack once the one before it has settled, dropping onto the pile in turn.";

	Runner.test('collision/sphere-stack-spawn-scene', 'six spheres stack up one at a time', function (t) {
		var world = t.makeWorld();
		t.plane(world, 'y', 20, 20, 0, { color: '#243B2A' });

		var r = 1, spheres = [], spawned = 0, SPAWN_INTERVAL = 90, SPHERE_COUNT = 6;
		var ticks = 0, everNonFinite = false;
		t.onTick(function (world, tick) {
			ticks = tick;
			if (spawned < SPHERE_COUNT && (tick - 1) % SPAWN_INTERVAL === 0) {
				var topY = spawned === 0 ? 0 : spheres[spawned - 1].position.y + 2 * r;
				spheres.push(t.sphere(world, r, 1, { pos: [0, topY + r + 3, 0], color: '#4af' }));
				spawned++;
			}
			for (var i = 0; i < spheres.length; i++) {
				var p = spheres[i].position;
				if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) { everNonFinite = true; break; }
			}
		});
		t.expect('the stack stays numerically finite for the WHOLE run (checked every tick, not just the end)', function () {
			if (ticks < SPHERE_COUNT * SPAWN_INTERVAL) return false;
			return { ok: !everNonFinite, detail: everNonFinite ? 'went non-finite at some point' : 'all spheres finite' };
		});

		t.simulate(world, SPHERE_COUNT * SPAWN_INTERVAL + 60);
	}, { visual: true, steps: 600, page: 'sphere-stack-spawn-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
