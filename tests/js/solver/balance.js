(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "Two balls are dropped straight down onto a fixed ball. A correct solver stacks them " +
		"into a steady column (centers at y=0, 2, 4) instead of letting them slip off or sink through.";

	Runner.test('collision/balance', 'two spheres stack on a static one (y=2, y=4)', function (t) {
		var world = t.makeWorld();
		var stat = t.sphere(world, 1, 0, { pos: [0, 0, 0], color: '#888' });
		var d1 = t.sphere(world, 1, 1, { pos: [0, 3, 0], color: '#F4D35E' });
		var d2 = t.sphere(world, 1, 1, { pos: [0, 7, 0], color: '#45B7D1' });

		function restingAt(b, h) {
			return function () {
				var dy = Math.abs(b.position.y - h), slow = Math.abs(b.linear_velocity.y) < 0.1;
				return { ok: dy <= 0.25 && slow, detail: 'y=' + b.position.y.toFixed(2) + ' vy=' + b.linear_velocity.y.toFixed(2) };
			};
		}
		t.expect('middle ball comes to rest on the static one (y ~ 2)', restingAt(d1, 2));
		t.expect('top ball comes to rest on the middle one (y ~ 4)', restingAt(d2, 4));

		t.simulate(world, 180);
	}, { visual: true, steps: 180, page: 'balance', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
