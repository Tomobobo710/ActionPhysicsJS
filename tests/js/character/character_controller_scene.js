(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "A spring-based CharacterController is dropped above a ground box and left to fall, land, " +
		"and settle under its own ground spring - no input, no jump. Not a behavioral pass/fail; the only " +
		"assertion is that nothing goes non-finite. Watch it in the visual bench.";

	Runner.test('character/controller-drop', 'CharacterController drops onto a ground box and settles', function (t) {
		var world = t.makeWorld();
		var ground = t.box(world, 25, 1, 25, 0, { pos: [0, -1, 0], color: '#243B2A' });

		var cc = new AP.CharacterController(world, {
			radius: 1, height: 4, mass: 5, rideHeight: 2, moveSpeed: 10
		});
		cc.body.position.set(0, 10, 0);
		world.addRigidBody(cc.body);
		cc.body._color = '#e0b040';
		t.bodies.push(cc.body);

		t.stepWorld = function (w) {
			cc.update(t.DT);
			w.step(t.DT);
		};

		var ticks = 0, everNonFinite = false;
		t.onTick(function (w, tick) {
			ticks = tick;
			var p = cc.body.position, v = cc.body.linear_velocity;
			if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z) ||
				!isFinite(v.x) || !isFinite(v.y) || !isFinite(v.z)) {
				everNonFinite = true;
			}
		});

		t.expect('the character stays numerically finite for the WHOLE run (checked every tick)', function () {
			if (ticks < 300) return false;
			return { ok: !everNonFinite, detail: everNonFinite ? 'went non-finite at some point' : 'stayed finite throughout' };
		});

		t.simulate(world, 300);
	}, { visual: true, steps: 300, page: 'character-controller-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
