(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	function wallWorld() {
		var w = PBF.flatWorld();
		w.addRigidBody(PBF.staticBox(10, 2, 0.3, { x: 0, y: 2, z: 3 }));
		return w;
	}

	PBF.scaleTest('fps/walls', 'W1', 'head-on wall stop (no penetrate)', function (t, S) {
		var w = wallWorld();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var maxFront = 0;
		PBF.drive(t, p, function () {
			var front = p.body.position.z + p.depth / 2;
			if (front > maxFront) maxFront = front;
			return { forward: 1, sprint: true };
		});
		t.log('Sprint straight into the wall (face z=2.70). The front of the collider must never pass 2.72.');
		t.expect('front never passes 2.72 (face 2.70)', function () {
			return { ok: maxFront < 2.72, detail: 'maxFront=' + maxFront.toFixed(3) + ' (face 2.70)' };
		});
		t.simulate(w, 120);
	}, { page: 'fps/walls', steps: 120, description: 'Sprint head-on into a thin wall. The collider should stop flush at the face and never penetrate.' });

	PBF.scaleTest('fps/walls', 'W2', 'wall contact no jitter', function (t, S) {
		var w = wallWorld();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var maxSwing = 0, prev = null, tick0 = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			if (tick > 60) {
				if (prev == null) prev = p.body.position.z;
				var swing = Math.abs(p.body.position.z - prev);
				if (swing > maxSwing) maxSwing = swing;
				prev = p.body.position.z;
			}
			return { forward: 1, sprint: true };
		});
		t.log('Hold against the wall; watch for in/out spring. Peak per-tick z swing over the tail must stay < 0.02.');
		t.expect('max per-tick swing < 0.02', function () {
			var measuring = tick0 > 61;
			return { ok: measuring && maxSwing < 0.02, detail: 'maxPerTickSwing=' + maxSwing.toFixed(4) };
		});
		t.simulate(w, 120);
	}, { page: 'fps/walls', steps: 120, description: 'Press into the wall and hold. The collider must sit dead still — no per-tick in/out jitter.' });

	PBF.scaleTest('fps/walls', 'W3', 'sprint into thick wall no enter', function (t, S) {
		var w = S.flat();
		w.addRigidBody(PBF.staticBox(10, 2, 2, { x: 0, y: 2, z: 5 }));
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var maxFront = 0;
		PBF.drive(t, p, function () {
			var front = p.body.position.z + p.depth / 2;
			if (front > maxFront) maxFront = front;
			return { forward: 1, sprint: true };
		});
		t.log('Sprint into a thick slab (front face z=3.0). The front must never reach 3.05.');
		t.expect('front never passes 3.05 (face 3.0)', function () {
			return { ok: maxFront < 3.05, detail: 'maxFront=' + maxFront.toFixed(3) + ' (face 3.0)' };
		});
		t.simulate(w, 200);
	}, { page: 'fps/walls', steps: 200, description: 'Sprint into a thick wall. Swept sub-stepping should catch the face so the collider never tunnels in.' });

	PBF.scaleTest('fps/walls', 'W4', 'buried recovery walks out', function (t, S) {
		var w = S.flat();
		w.addRigidBody(PBF.staticBox(10, 2, 2, { x: 0, y: 2, z: 5 }));
		var buryZ = 3 + 0.6 * S.SC;
		var p = S.spawn(w, { x: 0, y: 1, z: buryZ }, {});
		p.yaw = Math.PI / 2;
		PBF.renderables(t, p);
		PBF.drive(t, p, function () {
			return { forward: 1 };
		});
		var limit = 3.0 + S.sc(0.15);
		t.log('Spawned buried inside the wall; walking parallel should nudge the front out to the 3.0 face.');
		t.expect('front recovers to face + sc(0.15)', function () {
			var front = p.body.position.z + p.depth / 2;
			return { ok: front <= limit, detail: 'front=' + front.toFixed(3) + ' (face 3.0) limit=' + limit.toFixed(2) };
		});
		t.simulate(w, 160);
	}, { page: 'fps/walls', steps: 160, description: 'Spawn the character buried inside a thick wall and walk parallel to the face. The recovery nudge should walk it back out.' });

	PBF.scaleTest('fps/walls', 'W5', 'wall stop preserves tangential', function (t, S) {
		var w = wallWorld();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var frontLimit = 2.70 + S.sc(0.06);
		var contactBand = 2.70 - S.sc(0.25);
		var reached = false, maxFrontWhileOn = 0, tangentialWhileOn = 0, everOnWall = false;
		PBF.drive(t, p, function () {
			var front = p.body.position.z + p.depth / 2;
			var onFace = front > contactBand && Math.abs(p.body.position.x) < 9;
			if (front > contactBand) reached = true;
			if (onFace) {
				everOnWall = true;
				if (front > maxFrontWhileOn) maxFrontWhileOn = front;
				tangentialWhileOn = Math.abs(p.body.linear_velocity.x);
			}
			return { forward: 1, right: 1, sprint: true, yaw: 0 };
		});
		t.log('Run into the wall at 45deg. Must actually REACH it, then the into-wall z stops flush while tangential x speed keeps you sliding along the face (>sc(3)), no push-through.');
		t.expect('reaches the wall', function () {
			return { ok: reached, detail: 'reachedWall=' + reached };
		});
		t.expect('actually slides along the face (not just a corner graze)', function () {
			if (!reached) return { ok: false, detail: 'never reached the wall' };
			return { ok: everOnWall, detail: 'everOnWall=' + everOnWall };
		});
		t.expect('tangential speed while on the face > sc(3)', function () {
			if (!everOnWall) return { ok: false, detail: 'never on the face' };
			return { ok: tangentialWhileOn > S.sc(3), detail: 'tangentialOnFace=' + tangentialWhileOn.toFixed(2) + ' expect>' + S.sc(3).toFixed(1) };
		});
		t.expect('no push-through while on the face', function () {
			if (!everOnWall) return { ok: false, detail: 'never on the face' };
			return { ok: maxFrontWhileOn < frontLimit, detail: 'maxFrontOnFace=' + maxFrontWhileOn.toFixed(3) + ' limit=' + frontLimit.toFixed(2) };
		});
		t.simulate(w, 90);
	}, { page: 'fps/walls', steps: 90, description: 'Hit the wall at 45deg. Must actually reach the wall, then slide along its face keeping tangential speed while the into-wall component stops flush.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
