(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "A base box and a top box are welded face-to-face by a WeldConstraint and dropped " +
		"onto a ground plane - a correctly working weld holds them as one rigid unit.";

	Runner.test('collision/constraint-weld-scene', 'two boxes welded together fall as one unit', function (t) {
		var world = t.makeWorld();
		t.box(world, 20, 0.5, 20, 0, { color: '#243B2A' });
		var base = t.box(world, 1, 1, 1, 10, { pos: [0, 2, 0], color: '#B08968' });
		var top = t.box(world, 1, 1, 1, 10, { pos: [0, 4, 0], color: '#4af' });

		var weld = new AP.WeldConstraint(base, top, new AP.Vector3(0, 1, 0), new AP.Vector3(0, -1, 0));
		world.addConstraint(weld);

		var ticks = 0, worstGapDeviation = 0, everNonFinite = false;
		t.onTick(function (world, tick) {
			ticks = tick;
			if (!isFinite(base.position.x) || !isFinite(base.position.y) || !isFinite(top.position.x) || !isFinite(top.position.y)) { everNonFinite = true; return; }
			var dx = top.position.x - base.position.x, dy = top.position.y - base.position.y, dz = top.position.z - base.position.z;
			var gap = Math.abs(Math.hypot(dx, dy, dz) - 2);
			if (gap > worstGapDeviation) worstGapDeviation = gap;
		});
		t.expect('the welded pair stays finite and the weld never separates, for the WHOLE run (checked every tick)', function () {
			if (ticks < 300) return false;
			return { ok: !everNonFinite && worstGapDeviation < 0.1, detail: everNonFinite ? 'went non-finite' : 'worst gap deviation=' + worstGapDeviation.toFixed(4) };
		});

		t.simulate(world, 300);
	}, { visual: true, steps: 300, page: 'constraint-weld-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
