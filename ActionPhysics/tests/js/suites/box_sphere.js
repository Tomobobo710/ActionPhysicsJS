(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "A sphere of radius 1 is dropped from y=5 onto a fixed box whose top is at y=0.5. " +
		"Sphere-on-box contact should catch it and hold it: its center comes to rest at y = 1.5.";

	Runner.test('collision/box-sphere', 'sphere rests on a static box at y ~ 1.5', function (t) {
		var world = t.makeWorld();
		t.box(world, 0.5, 0.5, 0.5, 0, { color: '#888' });
		var sphere = t.sphere(world, 1, 1, { pos: [0, 5, 0], color: '#45B7D1' });

		t.expect('sphere lands and rests on the box (center y ~ 1.5)', function () {
			var atRest = Math.abs(sphere.position.y - 1.5) <= 0.05 && Math.abs(sphere.linear_velocity.y) < 0.1;
			return { ok: atRest, detail: 'y=' + sphere.position.y.toFixed(3) + ' vy=' + sphere.linear_velocity.y.toFixed(2) };
		});

		t.simulate(world, 150);
	}, { visual: true, steps: 150, page: 'box-sphere', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
