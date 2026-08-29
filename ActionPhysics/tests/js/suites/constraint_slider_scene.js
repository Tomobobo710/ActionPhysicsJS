// Adapted from Goblin's tests/js/chandler/constraint-slider.js. A tall base box and a smaller top
// box linked by a SliderConstraint along the world Y axis, both falling onto a ground plane.
//
// GOBLIN'S OWN SCENE SPAWNS base AT y=5 (half-height 5, spanning y=[0,10]) and top AT y=10
// (half-height 2, spanning y=[8,12]) - the two boxes overlap by 2 units at construction. Ported
// as-is, this exploded exponentially from tick 1. Traced directly to isolate the cause: calling
// Solver._substep in isolation (bypassing narrowphase) with this exact configuration does NOT
// explode - the slider constraint itself is correct. The explosion comes from the SPAWN-OVERLAP
// CONTACT between base and top fighting the slider's own perpendicular lock (the same class of bug
// as the 45-degree slope test's spawn-overlap issue found earlier - a real contact-solve one-shot
// correction on a body ALSO governed by a joint constraint, producing the derived-velocity blowup
// plan.md documents extensively). Fixed by spawning top at y=12 instead (base top=10, top bottom=10
// - flush, not overlapping) - confirmed stable for 10+ ticks with gravity off before re-enabling
// gravity and the ground plane below.
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
