// Top spawns FLUSH with base (top's bottom face = base's top face), never overlapping: a
// joint-held pair that also starts interpenetrating takes a one-shot contact correction on top of
// the joint lock and diverges.
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "A tall base box and a smaller top box are linked by a SliderConstraint along the " +
		"world Y axis and dropped onto a ground plane, sliding freely relative to each other.";

	Runner.test('collision/constraint-slider-scene', 'two boxes linked by a slider constraint stay stable', function (t) {
		var world = t.makeWorld();
		t.box(world, 20, 0.5, 20, 0, { pos: [0, -3, 0], color: '#243B2A' });
		var base = t.box(world, 1, 5, 1, 10, { pos: [0, 5, 0], color: '#B08968' });
		var top = t.box(world, 1, 2, 1, 10, { pos: [0, 12, 0], color: '#4af' });

		var slider = new AP.SliderConstraint(base, new AP.Vector3(0, 1, 0), new AP.Vector3(0, 0, 0), top, new AP.Vector3(0, 0, 0));
		world.addConstraint(slider);

		var ticks = 0, everNonFinite = false, worstPerpError = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			if (!isFinite(base.position.y) || !isFinite(top.position.y) || !isFinite(base.position.x) || !isFinite(top.position.x)) { everNonFinite = true; return; }
			var perpError = Math.hypot(top.position.x - base.position.x, top.position.z - base.position.z);
			if (perpError > worstPerpError) worstPerpError = perpError;
		});
		t.expect('the scene stays numerically finite for the WHOLE run (checked every tick, not just the end)', function () {
			if (ticks < 300) return false;
			return { ok: !everNonFinite, detail: everNonFinite ? 'went non-finite at some point' : 'base.y=' + base.position.y.toFixed(2) + ' top.y=' + top.position.y.toFixed(2) };
		});
		t.expect('the slider holds base and top aligned perpendicular to the slide axis (X/Z stay coincident)', function () {
			if (ticks < 300) return false;
			return { ok: worstPerpError < 0.01, detail: 'worst perpendicular drift=' + worstPerpError.toFixed(5) };
		});

		t.simulate(world, 300);
	}, { visual: true, steps: 300, page: 'constraint-slider-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
