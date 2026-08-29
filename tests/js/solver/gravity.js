(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "Four bodies of very different mass fall under gravity g=-10. Gravity accelerates " +
		"everything equally, so after 2 seconds every body - heavy or light - is falling at exactly " +
		"vy = -20.";

	Runner.test('collision/gravity', 'four bodies free-fall to vy=-20 (g=-10, t=2s)', function (t) {
		var world = t.makeWorld({ gravity: -10 });
		var sphere1 = t.sphere(world, 1, 1, { pos: [0, 0, 0], linear_damping: 0, angular_damping: 0, color: '#F4D35E' });
		var sphere2 = t.sphere(world, 1, 10, { pos: [3, 0, 0], linear_damping: 0, angular_damping: 0, color: '#EE964B' });
		var box1 = t.box(world, 1, 1, 1, 1, { pos: [6, 0, 0], linear_damping: 0, angular_damping: 0, color: '#45B7D1' });
		var box2 = t.box(world, 1, 1, 1, 0.01, { pos: [9, 0, 0], linear_damping: 0, angular_damping: 0, color: '#8367C7' });

		function reachesMinus20(b) {
			return function () { return { ok: Math.abs(b.linear_velocity.y + 20) <= 1e-5, detail: 'vy=' + b.linear_velocity.y.toFixed(3) }; };
		}
		t.expect('sphere (mass 1) reaches vy = -20', reachesMinus20(sphere1));
		t.expect('sphere (mass 10) reaches vy = -20 - same as mass 1', reachesMinus20(sphere2));
		t.expect('box (mass 1) reaches vy = -20', reachesMinus20(box1));
		t.expect('box (mass 0.01) reaches vy = -20 - mass does not matter', reachesMinus20(box2));

		t.simulate(world, 130);
	}, { visual: true, steps: 130, page: 'gravity', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
