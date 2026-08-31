(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "A KINEMATIC body (new RigidBody(shape, mass, { kinematic: true })) is code-driven: " +
		"the integrator advances its transform from its own velocity every tick, but contacts and " +
		"forces never move it. A dynamic rider on a moving kinematic platform is carried along; a " +
		"dynamic body shoved into a kinematic wall stops and the wall does not budge; a kinematic " +
		"body driven by direct position writes still collides correctly.";

	function test(name, fn, steps) {
		Runner.test('solver/kinematic', name, fn, { page: 'kinematic', description: DESC, visual: true, steps: steps || 120 });
	}

	function kinematicBox(w, hx, hy, hz, opts) {
		opts = opts || {};
		var b = new AP.RigidBody(new AP.BoxShape(hx, hy, hz), 0, { kinematic: true });
		if (opts.pos) b.position.set(opts.pos[0], opts.pos[1], opts.pos[2]);
		if (opts.vel) b.linear_velocity.set(opts.vel[0], opts.vel[1], opts.vel[2]);
		b.updateDerived();
		b._color = opts.color || '#c4c';
		w.addRigidBody(b);
		return b;
	}

	test('a kinematic body reports bodyType KINEMATIC and zero inverse mass', function (t) {
		var w = t.makeWorld();
		var k = kinematicBox(w, 1, 0.25, 1, { pos: [0, 0, 0] });
		t.expect('bodyType === RigidBody.KINEMATIC', function () {
			return { ok: k.bodyType === AP.RigidBody.KINEMATIC, detail: 'bodyType=' + k.bodyType };
		});
		t.expect('_mass_inverted === 0 (infinite effective mass)', function () {
			return { ok: k._mass_inverted === 0, detail: 'inv=' + k._mass_inverted };
		});
		t.expect('is_static === false (it is not static, it moves)', function () {
			return { ok: k.is_static === false, detail: 'is_static=' + k.is_static };
		});
		t.simulate(w, 1);
	}, 1);

	test('a velocity-driven kinematic platform advances by v*t and is unaffected by a rider', function (t) {
		var w = t.makeWorld();
		var plat = kinematicBox(w, 2, 0.25, 2, { pos: [0, 0, 0], vel: [1, 0, 0] });
		var box = t.box(w, 0.4, 0.4, 0.4, 1, { pos: [0, 0.66, 0], friction: 1, color: '#4af' });

		t.expect('after 1.0s the platform has moved ~1.0 in x (v=1)', function () {
			return { ok: Math.abs(plat.position.x - 1.0) < 0.02, detail: 'plat.x=' + plat.position.x.toFixed(4) };
		});
		t.expect('the platform never picked up any y or z drift (contacts did not move it)', function () {
			return { ok: Math.abs(plat.position.y) < 1e-9 && Math.abs(plat.position.z) < 1e-9,
				detail: 'plat=(' + plat.position.y.toExponential(2) + ',' + plat.position.z.toExponential(2) + ')' };
		});
		t.expect('the rider was carried along and stayed on the platform (x tracks within 0.25)', function () {
			return { ok: Math.abs(box.position.x - plat.position.x) < 0.25 && box.position.y > 0.4,
				detail: 'box.x=' + box.position.x.toFixed(3) + ' plat.x=' + plat.position.x.toFixed(3) + ' box.y=' + box.position.y.toFixed(3) };
		});
		t.simulate(w, 60);
	}, 60);

	test('a dynamic body shoved into a kinematic wall stops flush and the wall does not move', function (t) {
		var w = t.makeWorld({ gravity: 0 });
		var wall = kinematicBox(w, 0.5, 2, 2, { pos: [3, 0, 0] });
		var b = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], vel: [8, 0, 0], color: '#4af', restitution: 0 });

		t.expect('the body stops before passing through the wall (x stays below ~2.0)', function () {
			return { ok: b.position.x < 2.05, detail: 'b.x=' + b.position.x.toFixed(4) };
		});
		t.expect('the wall never moved', function () {
			return { ok: Math.abs(wall.position.x - 3) < 1e-9 && Math.abs(wall.position.y) < 1e-9 && Math.abs(wall.position.z) < 1e-9,
				detail: 'wall.x=' + wall.position.x.toFixed(6) };
		});
		t.simulate(w, 90);
	}, 90);

	test('a position-driven kinematic body (no velocity set) still collides correctly', function (t) {
		var w = t.makeWorld({ gravity: 0 });
		var k = kinematicBox(w, 0.5, 2, 2, { pos: [-5, 0, 0] });
		var b = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], color: '#4af', restitution: 0 });
		var driver = -5;

		t.expect('the kinematic body drives into the resting box and pushes it (box.x climbs past 0.5)', function () {
			return { ok: b.position.x > 0.4, detail: 'box.x=' + b.position.x.toFixed(4) + ' k.x=' + k.position.x.toFixed(3) };
		});
		t.expect('the box is never left overlapping the kinematic body by more than a small margin', function () {
			var gap = b.position.x - k.position.x;
			return { ok: gap > 0.9, detail: 'gap=' + gap.toFixed(4) };
		});

		// Drive by direct position write each tick, no velocity.
		for (var i = 0; i < 100; i++) {
			driver += 0.06;
			if (driver > 2) driver = 2;
			k.position.x = driver;
			k.updateDerived();
			w.step(1 / 60);
		}
	}, 100);

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
