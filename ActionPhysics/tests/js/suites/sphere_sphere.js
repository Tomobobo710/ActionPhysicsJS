(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "Two equal balls fly straight at each other at 2 u/s, each with restitution 0.2. A " +
		"head-on collision at e=0.2 keeps only 20% of the closing speed.";

	Runner.test('collision/sphere-sphere', 'e=0.2 head-on: s1 -> -0.4, s2 -> +0.4', function (t) {
		var world = t.makeWorld({ gravity: 0 });
		var s1 = t.sphere(world, 1, 1, { pos: [0, 0, 0], vel: [0, 2, 0], restitution: 0.2, linear_damping: 0, angular_damping: 0, color: '#F4D35E' });
		var s2 = t.sphere(world, 1, 1, { pos: [0, 3, 0], vel: [0, -2, 0], restitution: 0.2, linear_damping: 0, angular_damping: 0, color: '#45B7D1' });

		function reachesVy(b, v) {
			return function () { return { ok: Math.abs(b.linear_velocity.y - v) <= 0.001, detail: 'vy=' + b.linear_velocity.y.toFixed(3) }; };
		}
		t.expect('bottom ball ends moving down slowly (vy -> -0.4)', reachesVy(s1, -0.4));
		t.expect('top ball ends moving up slowly (vy -> +0.4)', reachesVy(s2, 0.4));

		t.simulate(world, 120);
	}, { visual: true, steps: 120, page: 'sphere-sphere', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
