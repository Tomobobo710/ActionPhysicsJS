/**
 * Tom's Suite — FPSCharacterController STATE-CHANGE tests (S1-S3).
 */
(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	// rampWorld(deg): 20x2x30 slab at y=10, rotated -deg about X. NOT scaled.
	function rampWorld(deg) {
		var w = PBF.makeWorld();
		var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
		return w;
	}

	// ---- S1: crouch-while-sprint continuity ----
	PBF.scaleTest('fps/state', 'S1', 'crouch-while-sprint continuity', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);

		var before = null, after = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 26) before = PBF.hsp(p);
			return tick >= 26 ? { forward: 1, sprint: true, crouch: true }
			                  : { forward: 1, sprint: true };
		});

		t.log('Sprint on flat, then crouch mid-sprint on tick 26; horizontal speed should stay > 80% of the pre-crouch speed.');
		t.expect('crouch keeps >80% of sprint speed', function () {
			if (before == null) return { ok: false, detail: 'sprinting…' };
			after = PBF.hsp(p);
			return { ok: after > before * 0.8, detail: 'before=' + before.toFixed(2) + ' after=' + after.toFixed(2) };
		});
		t.simulate(w, 26);
	}, { page: 'fps/state', steps: 26, description: 'Sprint on flat ground, then crouch while still sprinting. Watch the collider keep moving (>80% of its pre-crouch speed) rather than stalling.' });

	// ---- S2: scale mid-motion continuity ----
	PBF.scaleTest('fps/state', 'S2', 'scale mid-motion continuity', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);

		var before = null, scaled = false;
		PBF.drive(t, p, function (tick) {
			if (tick === 26 && !scaled) { before = PBF.hsp(p); p.setScale(1.5); scaled = true; }
			return { forward: 1, sprint: true };
		});

		t.log('Sprint on flat, then setScale(1.5) mid-motion on tick 26; horizontal speed should stay > 80% of the pre-scale speed.');
		t.expect('scale keeps >80% of sprint speed', function () {
			if (before == null) return { ok: false, detail: 'sprinting…' };
			var after = PBF.hsp(p);
			return { ok: after > before * 0.8, detail: 'before=' + before.toFixed(2) + ' after=' + after.toFixed(2) };
		});
		t.simulate(w, 26);
	}, { page: 'fps/state', steps: 26, description: 'Sprint on flat ground, then scale the controller to 1.5x mid-motion. Watch that speed is preserved (>80%) across the scale change.' });

	// ---- S3: crouch on ramp no drift ----
	PBF.scaleTest('fps/state', 'S3', 'crouch on ramp no drift', function (t, S) {
		var w = rampWorld(30);
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);

		var z0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 120) z0 = p.body.position.z;
			return tick >= 121 ? { crouch: true } : {};
		});

		t.log('Settle on a 30° ramp, then crouch and hold; z-position must not drift (< 0.2 units) from where it settled.');
		t.expect('crouch on ramp drift < 0.2', function () {
			if (z0 == null) return { ok: false, detail: 'settling…' };
			var drift = Math.abs(p.body.position.z - z0);
			return { ok: drift < 0.2, detail: 'drift=' + drift.toFixed(3) };
		});
		t.simulate(w, 241);
	}, { page: 'fps/state', steps: 241, description: 'Park on a 30° ramp, let it settle, then crouch and hold. Watch that the collider stays put (< 0.2u drift) instead of sliding down.' });

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
