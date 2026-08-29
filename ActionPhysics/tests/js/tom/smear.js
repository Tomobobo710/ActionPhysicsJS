(function (Runner, U, AP) {
	Runner.suite('tom');

	var BOX = 0.8, HALF = BOX / 2, TOTAL = 300;

	var SEED_Q = [0.2625, 0.5063, 0.7427, -0.3509];

	var LOCAL_AXES = [
		new AP.Vector3(1, 0, 0), new AP.Vector3(-1, 0, 0),
		new AP.Vector3(0, 1, 0), new AP.Vector3(0, -1, 0),
		new AP.Vector3(0, 0, 1), new AP.Vector3(0, 0, -1)
	];
	function mostDownwardLocalAxis(rotQuat) {
		var q = new AP.Quaternion(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w);
		var best = null, bestY = Infinity;
		LOCAL_AXES.forEach(function (axis) {
			var world = q.transformVector(axis);
			if (world.y < bestY) { bestY = world.y; best = axis; }
		});
		return best;
	}

	Runner.test('smear', 'tilted box on a support does not hang frozen mid-air', function (t) {
		t.log('Drop a tilted box onto the top corner of a support box; it must tip off, not stick.');

		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);

		t.box(w, 0.3, 0.9, 0.3, 0, { pos: [0, 0.9, 0], friction: 0.4, restitution: 0, color: '#3355ff' });

		var box = t.box(w, HALF, HALF, HALF, 2, {
			pos: [0.4, 0.9 + 2.92, 0.017], rot: SEED_Q, friction: 3, restitution: 0.33, color: '#e0b020'
		});

		var bottomAxis = mostDownwardLocalAxis(box.rotation);

		var stuckRun = 0, worstStuck = 0, reachedFloor = false;
		t.onTick(function (world) {
			var vy = Math.abs(box.linear_velocity.y), sw = U.spin(box);
			var pts = contactPoints(world, box);
			var stuck = box.position.y > 0.9 && vy < 0.1 && sw < 0.15 && pts >= 2;
			if (stuck) { stuckRun++; if (stuckRun > worstStuck) worstStuck = stuckRun; } else stuckRun = 0;
			if ((box.position.y - HALF) < 0.06) reachedFloor = true;
		});

		function contactPoints(world, b) {
			var np = world.narrowphase; if (!np) return 0;
			var iter = np.manifolds.values();
			for (var r = iter.next(); !r.done; r = iter.next()) {
				var m = r.value;
				if ((m.bodyA === b || m.bodyB === b) && m.points.length) return m.points.length;
			}
			return 0;
		}

		t.expect('box does not hang frozen at head height (no long stuck run)', function () {
			return { ok: worstStuck <= 30, detail: 'worstStuckRun=' + worstStuck + ' (limit 30)' };
		});
		t.expect('box comes down to the floor', function () {
			return { ok: reachedFloor, detail: 'y=' + box.position.y.toFixed(3) + (reachedFloor ? ' reached floor' : '') };
		});
		t.expect('Physics correctness check', function () {
			if (!reachedFloor) return false;
			var q = new AP.Quaternion(box.rotation.x, box.rotation.y, box.rotation.z, box.rotation.w);
			var worldY = q.transformVector(bottomAxis).y;
			return { ok: worldY > 0.8, detail: 'bottom-face-at-spawn now points world Y=' + worldY.toFixed(3) + ' (expect >0.8, i.e. facing up)' };
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'smear',
		description:
			"A box, released already tilted just above the top corner of a static support box, free-falls and " +
			"catches the edge. PASS: it tips over the corner, rolls, and comes down to the floor with the face " +
			"that caught the corner now facing up (a real ~180° roll completed, not just a settle). FAIL: it " +
			"hangs frozen up at head height (tiny velocity and spin, held by a persistent multi-point contact " +
			"whose normal points nearly straight down) instead of rolling off — a contact-solver hang."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
