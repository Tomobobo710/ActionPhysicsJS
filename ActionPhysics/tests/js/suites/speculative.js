(function (Runner) {
	Runner.suite('speculative');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Speculative contacts: contact detected before overlap, geometry re-measured each " +
		"substep so corner/edge contacts stay stable, solver stops the body at touch. Verified: flat " +
		"drop settles exactly, high-speed drop never tunnels, resting is stable, stacks settle with " +
		"rotation free, a tilted box tips flat, friction resists sliding.";
	function test(group, name, fn, steps) {
		Runner.test(group, name, fn, { page: 'speculative', description: DESC, visual: true, steps: steps || 0 });
	}
	var DT = 1 / 60;

	Runner.test('speculative/aabb', 'getBroadphaseAABB fattens the tight AABB by the speculative margin, getAABB stays tight', function (t) {
		var b = t.loneBody(new AP.BoxShape(1, 1, 1), { pos: [0, 0, 0] });
		b.updateDerived(0);
		var tight = b.getAABB(), fat = b.getBroadphaseAABB();
		var m = AP.RigidBody.SPECULATIVE_MARGIN;
		t.check(tight.max.x, 1, 1e-9, 'tight AABB is the exact half-extent, no margin');
		t.check(fat.max.x, 1 + m, 1e-9, 'broadphase AABB adds the speculative margin');
		t.check(fat.min.y, -1 - m, 1e-9, 'margin applies on the min side too');
	}, { page: 'speculative', description: DESC });

	Runner.test('speculative/aabb', 'a downward velocity sweeps the broadphase AABB downward, not upward', function (t) {
		var b = t.loneBody(new AP.BoxShape(1, 1, 1), { vel: [0, -6, 0] });
		b.updateDerived(DT);
		var fat = b.getBroadphaseAABB();
		var m = AP.RigidBody.SPECULATIVE_MARGIN, sweep = 6 * DT;
		t.check(fat.min.y, -1 - m - sweep, 1e-9, 'the leading (downward) face grows by margin + velocity*dt');
		t.check(fat.max.y, 1 + m, 1e-9, 'the trailing (upward) face grows by the margin only - sweep is directional');
	}, { page: 'speculative', description: DESC });

	test('speculative/settle', 'a box dropped from height settles at exactly rest height, no bounce, no penetration', function (t) {
		var world = t.makeWorld({ gravity: 0 });
		world.gravity.set(0, -9.81, 0);
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], color: '#4af' });

		var maxPen = 0, maxSettledSpeed = 0;
		t.onTick(function (world, tick) {
			var pen = 0.5 - box.position.y;
			if (pen > maxPen) maxPen = pen;
			if (tick > 150) maxSettledSpeed = Math.max(maxSettledSpeed, Math.abs(box.linear_velocity.y));
		});
		t.expect('settles at exactly rest height', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-4, detail: 'y=' + box.position.y.toFixed(6) };
		});
		t.expect('never penetrates the ground meaningfully', function () {
			return { ok: maxPen < 1e-3, detail: 'max penetration ' + maxPen.toFixed(6) };
		});
		t.expect('no residual bounce once settled', function () {
			return { ok: maxSettledSpeed < 1e-4, detail: 'max settled speed ' + maxSettledSpeed.toFixed(6) };
		});
		t.simulate(world, 400);
	}, 400);

	test('speculative/tunnel', 'a box hurled straight down at 50 m/s is stopped at the surface, never tunnels through', function (t) {
		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 5, 0], vel: [0, -50, 0], color: '#f55' });

		var minY = Infinity;
		t.onTick(function () { if (box.position.y < minY) minY = box.position.y; });
		t.expect('the box never sank below the surface - no tunnelling', function () {
			return { ok: minY > 0.45, detail: 'min y ' + minY.toFixed(4) };
		});
		t.expect('comes to rest at the correct height', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-3, detail: 'y=' + box.position.y.toFixed(6) };
		});
		t.simulate(world, 400);
	}, 400);

	test('speculative/rest', 'a box placed exactly at rest height stays put - the exact-touch normal does not flip and eject it', function (t) {

		var world = t.makeWorld();
		world.solver.substeps = 1;
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], color: '#4af' });

		var maxSteadyDrift = 0;
		t.onTick(function (world, tick) { if (tick >= 5) maxSteadyDrift = Math.max(maxSteadyDrift, Math.abs(box.position.y - 0.5)); });
		t.expect('y holds at rest in steady state - no penetrate-then-eject buzz', function () {
			return { ok: maxSteadyDrift < 1e-5, detail: 'max steady drift ' + maxSteadyDrift.toFixed(8) };
		});
		t.expect('ends exactly at rest', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 0.02, detail: 'y=' + box.position.y.toFixed(8) };
		});
		t.simulate(world, 300);
	}, 300);

	test('speculative/stack', 'a 3-box stack settles at exact rest heights with rotation free', function (t) {

		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var boxes = [];
		var colors = ['#4af', '#6c8', '#fc4'];
		for (var i = 0; i < 3; i++) boxes.push(t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5 + i * 1.0 + 0.02, 0], color: colors[i] }));

		var maxHeightErr = 0;
		t.onTick(function (world, tick) {
			if (tick >= 300) for (var j = 0; j < 3; j++) maxHeightErr = Math.max(maxHeightErr, Math.abs(boxes[j].position.y - (0.5 + j)));
		});
		t.expect('bottom box at 0.5', function () { return { ok: Math.abs(boxes[0].position.y - 0.5) < 1e-3, detail: 'y=' + boxes[0].position.y.toFixed(5) }; });
		t.expect('middle box at 1.5', function () { return { ok: Math.abs(boxes[1].position.y - 1.5) < 1e-3, detail: 'y=' + boxes[1].position.y.toFixed(5) }; });
		t.expect('top box at 2.5', function () { return { ok: Math.abs(boxes[2].position.y - 2.5) < 1e-3, detail: 'y=' + boxes[2].position.y.toFixed(5) }; });
		t.expect('heights hold steady once settled - no drift', function () { return { ok: maxHeightErr < 2e-3, detail: 'max error ' + maxHeightErr.toFixed(6) }; });
		for (var k = 0; k < 3; k++) {
			(function (idx) {
				t.expect('box ' + idx + ' is at rest, not diverging', function () {
					var b = boxes[idx];
					var speed = Math.hypot(b.linear_velocity.x, b.linear_velocity.y, b.linear_velocity.z);
					return { ok: speed < 0.1, detail: '|v|=' + speed.toFixed(6) };
				});
			})(k);
		}
		t.simulate(world, 800);
	}, 800);

	test('speculative/tilt', 'a tilted box dropped onto the ground tips flat and settles, no launch', function (t) {
		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 1.2, 0], color: '#f84' });
		box.rotation.setAxisAngle(new V(0, 0, 1), 0.15);

		var maxW = 0;
		t.onTick(function () { maxW = Math.max(maxW, Math.hypot(box.angular_velocity.x, box.angular_velocity.y, box.angular_velocity.z)); });
		t.expect('settles flat at rest height (not launched)', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 5e-3, detail: 'y=' + box.position.y.toFixed(5) };
		});
		t.expect('angular velocity stays bounded through the tip - no runaway spin', function () {
			return { ok: maxW < 15, detail: 'max |w| ' + maxW.toFixed(3) + ' rad/s' };
		});
		t.expect('comes to angular rest', function () {
			var w = Math.hypot(box.angular_velocity.x, box.angular_velocity.y, box.angular_velocity.z);
			return { ok: w < 0.2, detail: '|w|=' + w.toFixed(4) };
		});
		t.simulate(world, 1200);
	}, 1200);

	test('speculative/friction', 'a box slid across the ground is brought to rest by friction, not left sliding forever', function (t) {
		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], friction: 0.5, color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], vel: [3, 0, 0], friction: 0.5, color: '#4af' });

		t.expect('the box has stopped sliding', function () {
			return { ok: Math.abs(box.linear_velocity.x) < 0.05, detail: '|vx|=' + Math.abs(box.linear_velocity.x).toFixed(4) };
		});
		t.expect('stopped within a short distance, not the ~15m a frictionless slide would cover', function () {
			return { ok: box.position.x < 3, detail: 'x=' + box.position.x.toFixed(3) };
		});
		t.simulate(world, 300);
	}, 300);

	test('speculative/friction', 'zero friction lets a box slide freely - friction is actually doing the work above', function (t) {

		var world = t.makeWorld();
		t.box(world, 40, 0.5, 40, 0, { pos: [0, -0.5, 0], friction: 0, color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], vel: [3, 0, 0], friction: 0, linear_damping: 0, angular_damping: 0, color: '#f84' });

		t.expect('frictionless: velocity is essentially unchanged', function () {
			return { ok: Math.abs(box.linear_velocity.x - 3) < 0.02, detail: 'vx=' + box.linear_velocity.x.toFixed(4) };
		});
		t.expect('frictionless: the box slid a long way', function () {
			return { ok: box.position.x > 9, detail: 'x=' + box.position.x.toFixed(2) };
		});
		t.expect('and stayed on the ground (did not fall off or sink)', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-2, detail: 'y=' + box.position.y.toFixed(4) };
		});
		t.simulate(world, 200);
	}, 200);

	test('speculative/pyramid', 'a 10-box pyramid drops, settles, and holds its shape without ejecting bodies', function (t) {
		var world = t.makeWorld();
		t.box(world, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#556' });
		var boxes = [], rows = 4;
		var layerColor = ['#4af', '#6c8', '#fc4', '#f66'];
		for (var row = 0; row < rows; row++) {
			var n = rows - row, y = 0.5 + row * 1.0 + 0.02;
			for (var k = 0; k < n; k++) {
				boxes.push(t.box(world, 0.5, 0.5, 0.5, 1, { pos: [(k - (n - 1) / 2) * 1.05, y, 0], color: layerColor[row] }));
			}
		}

		t.expect('no box was ejected from the pyramid', function () {
			var ejected = 0;
			for (var i = 0; i < boxes.length; i++) {
				var b = boxes[i];
				if (b.position.y > 5 || b.position.y < 0 || Math.abs(b.position.x) > 8) ejected++;
			}
			return { ok: ejected === 0, detail: ejected + '/' + boxes.length + ' ejected' };
		});
		t.expect('the pyramid is settled, not exploding', function () {
			var maxSpeed = 0;
			for (var i = 0; i < boxes.length; i++) {
				var b = boxes[i];
				maxSpeed = Math.max(maxSpeed, Math.hypot(b.linear_velocity.x, b.linear_velocity.y, b.linear_velocity.z));
			}
			return { ok: maxSpeed < 1, detail: 'max |v| ' + maxSpeed.toFixed(4) };
		});
		for (var m = 0; m < boxes.length; m++) {
			(function (idx) {
				t.expect('box ' + idx + ' rests at a valid layer height', function () {
					var yy = boxes[idx].position.y, layer = Math.round(yy - 0.5);
					var ok = Math.abs(yy - (0.5 + layer)) < 0.1 && layer >= 0 && layer < 4;
					return { ok: ok, detail: 'y=' + yy.toFixed(3) };
				});
			})(m);
		}
		t.simulate(world, 1200);
	}, 1200);

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
