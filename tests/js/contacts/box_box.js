(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;

	function totalRotationDegrees(body) {
		var wq = Math.abs(body.rotation.w);
		if (wq > 1) wq = 1;
		return 2 * Math.acos(wq) * 180 / Math.PI;
	}
	function speed(b) { var v = b.linear_velocity; return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
	function spin(b) { var a = b.angular_velocity; return Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

	function leanDegrees(body) {
		var up = { x: 0, y: 1, z: 0 };
		body.rotation.transformVectorInPlace(up);
		return {
			x: Math.atan2(up.x, up.y) * 180 / Math.PI,
			z: Math.atan2(up.z, up.y) * 180 / Math.PI
		};
	}

	function isSane(body, floorY) {
		var p = body.position, v = body.linear_velocity, w = body.angular_velocity;
		if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) return false;
		if (!isFinite(v.x) || !isFinite(v.y) || !isFinite(v.z)) return false;
		if (!isFinite(w.x) || !isFinite(w.y) || !isFinite(w.z)) return false;
		if (p.y < floorY - 5) return false;
		if (p.y > floorY + 100) return false;
		return true;
	}

	function makeTracker(t, boxes, totalTicks, floorY) {
		var worst = { x: 0, z: 0, y: 0, lx: 0, lz: 0, rot: 0, v: 0, w: 0 };
		var atHalf = null;
		var sane = true, insaneDetail = '';
		var half = Math.floor(totalTicks / 2);
		t.onTick(function (world, tick) {
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], body = o.body;
				if (!isSane(body, floorY)) { sane = false; insaneDetail = 'box ' + n + ' went insane: pos=' + JSON.stringify(body.position); continue; }
				var lean = leanDegrees(body);
				worst.x = Math.max(worst.x, Math.abs(body.position.x - o.x0));
				worst.z = Math.max(worst.z, Math.abs(body.position.z - o.z0));
				worst.y = Math.max(worst.y, Math.abs(body.position.y - o.y0));
				worst.lx = Math.max(worst.lx, Math.abs(lean.x));
				worst.lz = Math.max(worst.lz, Math.abs(lean.z));
				worst.rot = Math.max(worst.rot, totalRotationDegrees(body));
				worst.v = Math.max(worst.v, speed(body));
				worst.w = Math.max(worst.w, spin(body));
			}
			if (tick === half) atHalf = { x: worst.x, z: worst.z, y: worst.y, lx: worst.lx, lz: worst.lz, rot: worst.rot };
		});
		return {
			isSane: function () { return sane; },
			insaneDetail: function () { return insaneDetail; },
			worst: function () { return worst; },
			atHalf: function () { return atHalf; }
		};
	}

	Runner.test('box-box/single', 'a single box on a much bigger box settles dead still (no residual torque)', function (t) {

		var TICKS = 240;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: 0, color: '#243B2A' });
		var box = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], color: '#4af' });
		var tracker = makeTracker(t, [{ body: box, x0: 0, y0: 0.5, z0: 0 }], TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('stays sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'sane throughout' : tracker.insaneDetail() };
		});
		t.expect('settles at exactly rest height (y = 0.5)', function () {
			if (tick0 < TICKS) return false;
			var y = box.position.y;
			return { ok: Math.abs(y - 0.5) < 1e-3, detail: 'y=' + y.toFixed(5) };
		});
		t.expect('ends with negligible linear speed', function () {
			if (tick0 < TICKS) return false;
			return { ok: speed(box) < 1e-3, detail: '|v|=' + speed(box).toFixed(6) };
		});
		t.expect('ends with negligible angular speed (no fabricated spin)', function () {
			if (tick0 < TICKS) return false;
			return { ok: spin(box) < 1e-3, detail: '|w|=' + spin(box).toFixed(6) };
		});
		t.expect('x/z drift never exceeded a millimetre (perfectly flat drop, nothing should push it sideways)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			var lat = Math.sqrt(worst.x * worst.x + worst.z * worst.z);
			return { ok: lat < 1e-3, detail: 'worst lateral drift=' + lat.toFixed(6) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 240, page: 'box-box-single' });

	Runner.test('box-box/stack2', 'two boxes spawned already stacked, exactly touching, hold both positions', function (t) {

		var TICKS = 180;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: 0, color: '#243B2A' });
		var b0 = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], color: '#4af' });
		var b1 = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 1.5, 0], color: '#f84' });
		var tracker = makeTracker(t, [{ body: b0, x0: 0, y0: 0.5, z0: 0 }, { body: b1, x0: 0, y0: 1.5, z0: 0 }], TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('stays sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'sane throughout' : tracker.insaneDetail() };
		});
		t.expect('bottom box holds at y = 0.5', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(b0.position.y - 0.5) < 1e-3, detail: 'y=' + b0.position.y.toFixed(5) };
		});
		t.expect('top box holds at y = 1.5', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(b1.position.y - 1.5) < 1e-3, detail: 'y=' + b1.position.y.toFixed(5) };
		});
		t.expect('neither box ever drifted laterally', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			var lat = Math.sqrt(worst.x * worst.x + worst.z * worst.z);
			return { ok: lat < 1e-3, detail: 'worst lateral drift=' + lat.toFixed(6) };
		});
		t.expect('neither box ever picked up rotation', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 0.5, detail: 'worst rotation=' + worst.rot.toFixed(3) + ' degrees' };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 180, page: 'box-box-stack2' });

	Runner.test('box-box/bridge', 'a box bridging two separate supports stays flat, does not tip', function (t) {

		var TICKS = 400;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: 0, color: '#243B2A' });
		t.box(w, 1, 1, 1, 1, { pos: [-1.3, 1, 0], color: '#4af' });
		t.box(w, 1, 1, 1, 1, { pos: [1.3, 1, 0], color: '#4af' });
		var bridge = t.box(w, 1, 1, 1, 1, { pos: [0, 3.2, 0], color: '#f84' });
		var tracker = makeTracker(t, [{ body: bridge, x0: 0, y0: 3, z0: 0 }], TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('stays sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'sane throughout' : tracker.insaneDetail() };
		});
		t.expect('the bridge settles resting on both supports (y ~ 3)', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(bridge.position.y - 3) < 0.05, detail: 'y=' + bridge.position.y.toFixed(4) };
		});
		t.expect('the bridge never tipped (total rotation stayed under 2 degrees, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 2, detail: 'worst rotation=' + worst.rot.toFixed(3) + ' degrees' };
		});
		t.expect('the bridge never slid off its supports (x drift stayed under 0.1, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.x < 0.1, detail: 'worst x drift=' + worst.x.toFixed(4) };
		});

		t.expect('position genuinely stopped moving (second half added < 0.01 to y and x drift)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var addedY = worst.y - half.y, addedX = worst.x - half.x;
			return { ok: addedY < 0.01 && addedX < 0.01, detail: 'second-half added dy=' + addedY.toFixed(5) + ', dx=' + addedX.toFixed(5) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 400, page: 'box-box-bridge' });

	Runner.test('box-box/offset-stack', 'a 4-box offset stack (each box inset from the one below, on BOTH axes) holds its shape', function (t) {

		var TICKS = 500;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: 0, color: '#243B2A' });

		var boxes = [];
		// GAP = 2.0 == box height (half-extent 1): each layer spawns already resting on the one below,
		// inset horizontally on both axes. A larger gap would drop each box 0.2 onto its neighbour, and
		// the "stayed within 0.05 of its spawn height" checks below (which measure against the SPAWN y)
		// could never pass - the boxes would settle 0.2/0.4/0.6 below where they started, by design.
		var LAYERS = 4, GAP = 2.0, INSET = 0.3;
		for (var i = 0; i < LAYERS; i++) {
			var x = i * INSET, z = i * INSET * 0.6, y = i * GAP + 1;
			var b = t.box(w, 1, 1, 1, 1, { pos: [x, y, z], color: '#B08968' });
			boxes.push({ body: b, x0: x, y0: y, z0: z });
		}
		var tracker = makeTracker(t, boxes, TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('every box stayed sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'all ' + boxes.length + ' boxes finite and in bounds, the whole run' : tracker.insaneDetail() };
		});
		t.expect('every box stayed within 0.05 of its expected height (spacing held, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.y < 0.05, detail: 'worst height error=' + worst.y.toFixed(4) };
		});
		t.expect('every box stayed within 0.1 of its expected x (no creep along the x inset, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.x < 0.1, detail: 'worst x drift=' + worst.x.toFixed(4) };
		});
		t.expect('every box stayed within 0.1 of its expected z (no creep along the z inset, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.z < 0.1, detail: 'worst z drift=' + worst.z.toFixed(4) };
		});
		t.expect('nothing ever leaned on the X axis past 3 degrees, the whole run', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.lx < 3, detail: 'worst X-lean=' + worst.lx.toFixed(3) + ' degrees' };
		});
		t.expect('nothing ever leaned on the Z axis past 3 degrees, the whole run', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.lz < 3, detail: 'worst Z-lean=' + worst.lz.toFixed(3) + ' degrees' };
		});
		t.expect('nothing flipped or rolled over (absolute backstop: total rotation stayed under 90 degrees)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 90, detail: 'worst total rotation=' + worst.rot.toFixed(2) + ' degrees' };
		});
		t.expect('the whole stack is at rest by the end', function () {
			if (tick0 < TICKS) return false;
			var worstV = 0, worstW = 0;
			for (var n = 0; n < boxes.length; n++) {
				worstV = Math.max(worstV, speed(boxes[n].body));
				worstW = Math.max(worstW, spin(boxes[n].body));
			}
			return { ok: worstV < 0.01 && worstW < 0.01, detail: 'final |v|=' + worstV.toFixed(4) + ', final |w|=' + worstW.toFixed(4) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 500, page: 'box-box-offset-stack' });

	Runner.test('box-box/corner-drop', 'a box dropped corner-first onto flat ground tips onto a face and settles', function (t) {

		var TICKS = 400;
		var qx = t.quat(Math.sin(Math.PI / 8), 0, 0, Math.cos(Math.PI / 8));
		var halfZ = Math.atan(1 / Math.sqrt(2)) / 2;
		var qz = t.quat(0, 0, Math.sin(halfZ), Math.cos(halfZ));
		var qx2 = t.quat(Math.sin(0.05), 0, 0, Math.cos(0.05));
		var rot = AP.Quaternion.multiply(qx2, AP.Quaternion.multiply(qz, qx)).normalize();

		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: 0, color: '#243B2A' });
		var box = t.box(w, 0.5, 0.5, 0.5, 1, {
			pos: [0, 3, 0], rot: [rot.x, rot.y, rot.z, rot.w],
			friction: 0.5, restitution: 0, color: '#4af'
		});

		var minY = Infinity;
		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; if (box.position.y < minY) minY = box.position.y; });

		t.expect('settles onto a face (SOME local axis lands within 1 degree of world-up, not corner/edge-balanced)', function () {
			if (tick0 < TICKS) return false;
			var best = 180;
			var localAxes = [t.vec(1, 0, 0), t.vec(0, 1, 0), t.vec(0, 0, 1)];
			for (var i = 0; i < 3; i++) {
				var axis = localAxes[i];
				box.rotation.transformVectorInPlace(axis);
				var d = Math.max(-1, Math.min(1, Math.abs(axis.y)));
				var deg = Math.acos(d) * 180 / Math.PI;
				if (deg < best) best = deg;
			}
			return { ok: best < 1, detail: 'nearest-to-vertical local axis is ' + best.toFixed(3) + ' degrees off' };
		});
		t.expect('comes to rest (no perpetual rocking)', function () {
			if (tick0 < TICKS) return false;
			return { ok: speed(box) < 0.01 && spin(box) < 0.01, detail: '|v|=' + speed(box).toFixed(4) + ' |w|=' + spin(box).toFixed(4) };
		});
		t.expect('never tunnels through the ground, the whole run', function () {
			if (tick0 < TICKS) return false;
			return { ok: minY > -1, detail: 'lowest y reached=' + minY.toFixed(4) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 400, page: 'box-box-corner-drop' });

	Runner.test('box-box/fast-approach', 'a box falling fast onto flat ground lands flush, no single-corner torque kick', function (t) {

		var TICKS = 200;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: 0, color: '#243B2A' });
		var box = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], vel: [0, -15, 0], friction: 0.5, restitution: 0, color: '#4af' });

		var worstLateralV = 0, worstSpin = 0, landed = false, minY = Infinity, tick0 = 0;
		t.onTick(function (world, tick) {
			tick0 = tick;
			if (box.position.y < minY) minY = box.position.y;
			if (box.position.y < 0.55) landed = true;
			if (landed) {
				var lv = Math.sqrt(box.linear_velocity.x * box.linear_velocity.x + box.linear_velocity.z * box.linear_velocity.z);
				if (lv > worstLateralV) worstLateralV = lv;
				if (spin(box) > worstSpin) worstSpin = spin(box);
			}
		});
		t.expect('never tunnels through the ground, the whole run', function () {
			if (tick0 < TICKS) return false;
			return { ok: minY > -1, detail: 'lowest y reached=' + minY.toFixed(4) };
		});
		t.expect('no sideways velocity kick from landing (worst lateral speed under 0.05, the whole run)', function () {
			if (tick0 < TICKS) return false;
			return { ok: worstLateralV < 0.05, detail: 'worst lateral |v|=' + worstLateralV.toFixed(4) };
		});
		t.expect('no spin kick from landing (worst angular speed under 0.05, the whole run)', function () {
			if (tick0 < TICKS) return false;
			return { ok: worstSpin < 0.05, detail: 'worst |w|=' + worstSpin.toFixed(4) };
		});
		t.expect('settles at exactly rest height', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(box.position.y - 0.5) < 1e-2, detail: 'y=' + box.position.y.toFixed(5) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 200, page: 'box-box-fast-approach' });

	Runner.test('box-box/long-rest', 'a 3-box offset stack (inset on BOTH axes) shows no slow drift or tilt accumulation over 1000 ticks', function (t) {

		var TICKS = 1000;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], restitution: 0, color: '#243B2A' });

		var boxes = [];
		var LAYERS = 3, GAP = 2.2, INSET = 0.3;
		for (var i = 0; i < LAYERS; i++) {
			var x = i * INSET, z = i * INSET * 0.6, y = i * GAP + 1;
			var b = t.box(w, 1, 1, 1, 1, { pos: [x, y, z], color: '#B08968' });
			boxes.push({ body: b, x0: x, y0: y, z0: z });
		}
		var tracker = makeTracker(t, boxes, TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });

		t.expect('every box stayed sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'all ' + boxes.length + ' boxes finite and in bounds, the whole run' : tracker.insaneDetail() };
		});
		t.expect('x drift did not keep growing once settled (second half added < 0.02)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.x - half.x;
			return { ok: added < 0.02, detail: added.toFixed(4) + ' added since half-time (was ' + half.x.toFixed(4) + ', is ' + worst.x.toFixed(4) + ')' };
		});
		t.expect('z drift did not keep growing once settled (second half added < 0.02)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.z - half.z;
			return { ok: added < 0.02, detail: added.toFixed(4) + ' added since half-time (was ' + half.z.toFixed(4) + ', is ' + worst.z.toFixed(4) + ')' };
		});
		t.expect('y (height) drift did not keep growing once settled (second half added < 0.02)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.y - half.y;
			return { ok: added < 0.02, detail: added.toFixed(4) + ' added since half-time (was ' + half.y.toFixed(4) + ', is ' + worst.y.toFixed(4) + ')' };
		});
		t.expect('X-lean did not keep growing once settled (second half added < 0.5 degrees)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.lx - half.lx;
			return { ok: added < 0.5, detail: added.toFixed(3) + ' degrees added since half-time (was ' + half.lx.toFixed(3) + ', is ' + worst.lx.toFixed(3) + ')' };
		});
		t.expect('Z-lean did not keep growing once settled (second half added < 0.5 degrees)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.lz - half.lz;
			return { ok: added < 0.5, detail: added.toFixed(3) + ' degrees added since half-time (was ' + half.lz.toFixed(3) + ', is ' + worst.lz.toFixed(3) + ')' };
		});
		t.expect('total rotation did not keep growing once settled (second half added < 0.5 degrees)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.rot - half.rot;
			return { ok: added < 0.5, detail: added.toFixed(3) + ' degrees added since half-time (was ' + half.rot.toFixed(3) + ', is ' + worst.rot.toFixed(3) + ')' };
		});
		t.expect('nothing ever flipped or rolled over (absolute backstop: total rotation stayed under 90 degrees, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 90, detail: 'worst total rotation=' + worst.rot.toFixed(2) + ' degrees' };
		});
		t.expect('every box is still at rest at the end (no late-breaking instability)', function () {
			if (tick0 < TICKS) return false;
			var worstV = 0, worstW = 0;
			for (var n = 0; n < boxes.length; n++) {
				worstV = Math.max(worstV, speed(boxes[n].body));
				worstW = Math.max(worstW, spin(boxes[n].body));
			}
			return { ok: worstV < 0.01 && worstW < 0.01, detail: 'final |v|=' + worstV.toFixed(4) + ', final |w|=' + worstW.toFixed(4) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 1000, page: 'box-box-long-rest' });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
