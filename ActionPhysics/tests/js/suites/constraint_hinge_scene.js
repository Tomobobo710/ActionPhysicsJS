// Ported from Goblin's tests/js/chandler/constraint-hinge.js. A plank hinged to a fixed world point
// (single-body HingeConstraint, no bodyB) with a limit and a motor, over a ground plane. A sphere is
// dropped onto it every 240 ticks. Watch-only scene (matching Goblin's own), strengthened here to
// check the hinge stays numerically stable and respects its own limit for the whole run.
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "A plank is hinged to a fixed world point (limit -PI/8..0, motor target 40 rad/s) over a " +
		"ground plane, and a sphere is dropped onto it every 240 ticks.";

	Runner.test('collision/constraint-hinge-scene', 'motorized hinge plank catches dropped spheres', function (t) {
		var world = t.makeWorld();
		t.plane(world, 'y', 20, 20, 0, { pos: [0, -10, 0], color: '#243B2A' });

		var plank = t.box(world, 5, 0.3, 2, 1, { color: '#B08968' });

		var constraint = new AP.HingeConstraint(plank, new V(0, 0, 1), new V(-4, 0, 0));
		constraint.limit.set(-Math.PI / 8, 0);
		// Goblin's own torque=1 does not carry over unchanged: its ConstraintMotor is a PGS
		// velocity-impulse row (accumulated across solver iterations, clamped to the same total
		// impulse across the whole step), while this engine's motor is an XPBD position correction
		// applied fresh each substep - a genuinely different mechanism, so the numeric scale that
		// produces the SAME real behavior (holds flat under gravity, visibly yields to the sphere's
		// impact, then recovers) is different too. 0.25 is that number here, tuned directly against
		// the described behavior, not copied from Goblin's own value.
		constraint.motor.set(40, 0.25);
		world.addConstraint(constraint);

		var ticks = 0, everNonFinite = false, worstLimitViolation = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			if (tick % 240 === 0) t.sphere(world, 1, 1, { pos: [-3, 5, 0], color: '#4af' });
			var p = plank.position;
			if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) { everNonFinite = true; return; }
			var angle = constraint._swingAngle();
			var over = Math.max(constraint.limit.min - angle, angle - constraint.limit.max, 0);
			if (over > worstLimitViolation) worstLimitViolation = over;
		});
		t.expect('the hinge stays numerically finite for the WHOLE run (checked every tick, not just the end)', function () {
			if (ticks < 600) return false;
			return { ok: !everNonFinite, detail: everNonFinite ? 'went non-finite at some point' : 'plank.y=' + plank.position.y.toFixed(2) };
		});
		t.expect('the swing angle never exceeds its own limit by more than 0.01 rad', function () {
			if (ticks < 600) return false;
			return { ok: worstLimitViolation < 0.01, detail: 'worst limit violation=' + worstLimitViolation.toFixed(5) };
		});

		t.simulate(world, 600);
	}, { visual: true, steps: 600, page: 'constraint-hinge-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
