// Expected values are derived from the engine's own per-substep integration formulas - different
// substep counts and integration order produce different discrete results for the same physics.
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Damping is drag that bleeds a body's velocity away over time (0 = none, 0.5 = " +
		"moderate, 0.9 = strong). Nine spheres isolate every combination: linear vs angular motion, " +
		"driven by gravity / an impulse / a spin.";
	function test(group, name, fn) {
		Runner.test(group, name, fn, { page: 'damping', description: DESC, visual: true, steps: 130 });
	}

	var SUBSTEPS = 4;
	var DT = 1 / 60;
	var H = DT / SUBSTEPS;
	var TICKS = 130;
	var TOTAL_SUBSTEPS = TICKS * SUBSTEPS;
	var GRAVITY = -9.81;
	var TOLERANCE = 1e-6;

	function decayOnly(v0, damping) {
		var v = v0;
		var factor = Math.max(0, 1 - damping * H);
		for (var i = 0; i < TOTAL_SUBSTEPS; i++) v *= factor;
		return v;
	}

	function gravityWithDamping(g, damping) {
		var v = 0;
		var factor = damping > 0 ? Math.max(0, 1 - damping * H) : 1;
		for (var i = 0; i < TOTAL_SUBSTEPS; i++) {
			v += g * H;
			if (damping > 0) v *= factor;
		}
		return v;
	}

	var EXPECT_GRAVITY_NONE = gravityWithDamping(GRAVITY, 0);
	var EXPECT_GRAVITY_0_5 = gravityWithDamping(GRAVITY, 0.5);
	var EXPECT_GRAVITY_0_9 = gravityWithDamping(GRAVITY, 0.9);
	var EXPECT_IMPULSE_NONE = -10;
	var EXPECT_IMPULSE_0_5 = decayOnly(-10, 0.5);
	var EXPECT_IMPULSE_0_9 = decayOnly(-10, 0.9);
	var EXPECT_SPIN_NONE = 5;
	var EXPECT_SPIN_0_5 = decayOnly(5, 0.5);
	var EXPECT_SPIN_0_9 = decayOnly(5, 0.9);

	test('collision/damping', 'nine damping cases (gravity/impulse/spin x none/0.5/0.9)', function (t) {
		var world = t.makeWorld();
		var b1 = t.sphere(world, 1, 1, { pos: [-12, 0, 0], color: '#F4D35E' });
		var b2 = t.sphere(world, 1, 1, { pos: [-9, 0, 0], noGravity: true, color: '#F4D35E' }); b2.applyImpulse(t.vec(0, -10, 0));
		var b3 = t.sphere(world, 1, 1, { pos: [-6, 0, 0], noGravity: true, avel: [0, 0, 5], color: '#F4D35E' });
		var b4 = t.sphere(world, 1, 1, { pos: [-3, 0, 0], linear_damping: 0.5, color: '#EE964B' });
		var b5 = t.sphere(world, 1, 1, { pos: [0, 0, 0], noGravity: true, linear_damping: 0.5, color: '#EE964B' }); b5.applyImpulse(t.vec(0, -10, 0));
		var b6 = t.sphere(world, 1, 1, { pos: [3, 0, 0], noGravity: true, angular_damping: 0.5, avel: [0, 0, 5], color: '#EE964B' });
		var b7 = t.sphere(world, 1, 1, { pos: [6, 0, 0], linear_damping: 0.9, color: '#45B7D1' });
		var b8 = t.sphere(world, 1, 1, { pos: [9, 0, 0], noGravity: true, linear_damping: 0.9, color: '#45B7D1' }); b8.applyImpulse(t.vec(0, -10, 0));
		var b9 = t.sphere(world, 1, 1, { pos: [12, 0, 0], noGravity: true, angular_damping: 0.9, avel: [0, 0, 5], color: '#45B7D1' });

		function vy(b, expected, label) {
			t.expect(label, function () {
				var d = Math.abs(b.linear_velocity.y - expected);
				return { ok: d < TOLERANCE, detail: 'vy=' + b.linear_velocity.y.toFixed(6) + ' expected=' + expected.toFixed(6) + ' err=' + d.toExponential(3) };
			});
		}
		function wz(b, expected, label) {
			t.expect(label, function () {
				var d = Math.abs(b.angular_velocity.z - expected);
				return { ok: d < TOLERANCE, detail: 'wz=' + b.angular_velocity.z.toFixed(6) + ' expected=' + expected.toFixed(6) + ' err=' + d.toExponential(3) };
			});
		}

		vy(b1, EXPECT_GRAVITY_NONE, 'no damping - gravity integrates to the engine\'s own per-substep value');
		vy(b2, EXPECT_IMPULSE_NONE, 'no damping - an impulse holds its velocity');
		wz(b3, EXPECT_SPIN_NONE, 'no damping - a torque-free spin conserves angular velocity');
		vy(b4, EXPECT_GRAVITY_0_5, '0.5 damping - gravity vs drag settles at the coupled recurrence value');
		vy(b5, EXPECT_IMPULSE_0_5, '0.5 damping - an impulse decays by (1 - 0.5*h) every substep');
		wz(b6, EXPECT_SPIN_0_5, '0.5 damping - a spin decays by (1 - 0.5*h) every substep');
		vy(b7, EXPECT_GRAVITY_0_9, '0.9 strong damping - gravity vs drag settles near zero');
		vy(b8, EXPECT_IMPULSE_0_9, '0.9 strong damping - an impulse decays almost to zero');
		wz(b9, EXPECT_SPIN_0_9, '0.9 strong damping - a spin decays almost to zero');

		t.simulate(world, TICKS);
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
