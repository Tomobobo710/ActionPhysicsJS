/**
 * Tom's Suite — FPSCharacterController GHOST mechanism tests (G1-G7).
 */
(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	// ---- G1: ghost exists & is registered in the solver ----
	PBF.scaleTest('fps/ghost', 'G1', 'ghost exists & in solver', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		PBF.drive(t, p, function () { return {}; });

		t.log('The controller should own a ghost body that is registered in the physics solver.');
		t.expect('ghost body exists', function () {
			var has = !!p._ghost && !!p._ghostObject;
			return { ok: has, detail: 'ghost=' + has };
		});
		t.expect('ghost is registered in the solver', function () {
			var has = !!p._ghost && !!p._ghostObject;
			if (!has) return { ok: false, detail: 'no ghost body to check' };
			var inWorld = w.bodies.indexOf(p._ghost) !== -1;
			return { ok: inWorld, detail: 'inWorld=' + inWorld };
		});
		t.simulate(w, 10);
	}, { page: 'fps/ghost', steps: 10, description: 'The controller must own a ghost body registered in the solver.' });

	// ---- G2: teleporting the ghost never moves the player ----
	PBF.scaleTest('fps/ghost', 'G2', 'ghost never moves player position', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var px0 = null, moved = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) { px0 = p.body.position.x; p._ghost.position.set(50, 1, 50); }
			if (tick > 30 && px0 != null) moved = Math.abs(p.body.position.x - px0);
			return {};
		});

		t.log('At tick 30 the ghost is teleported to (50,1,50); the player position must not follow.');
		t.expect('player not dragged by ghost (moved < 0.01)', function () {
			if (moved == null) return { ok: false, detail: 'settling…' };
			return { ok: moved < 0.01, detail: 'playerMovedByGhost=' + moved.toFixed(4) };
		});
		t.simulate(w, 40);
	}, { page: 'fps/ghost', steps: 40, description: 'Teleport the ghost far away mid-run; the player must stay put.' });

	// ---- G3: drive accumulates onto ghost velocity, does not overwrite ----
	PBF.scaleTest('fps/ghost', 'G3', 'drive accumulates not overwrites', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var before = null, after = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 10) {
				p._ghost.linear_velocity.set(20, 0, 0);
				before = p._ghost.linear_velocity.x;
				p._syncGhost(PBF.DT);
				after = p._ghost.linear_velocity.x;
			}
			return {};
		});

		t.log('Seed ghost vx=20 then _syncGhost; the drive must accumulate (keep/boost), not overwrite to a lower value.');
		t.expect('drive accumulates (vxAfter>1 or <before)', function () {
			if (after == null) return { ok: false, detail: 'settling…' };
			return { ok: after > 1 || after < before, detail: 'vxBefore=' + before.toFixed(2) + ' vxAfter=' + after.toFixed(2) };
		});
		t.simulate(w, 20);
	}, { page: 'fps/ghost', steps: 20, description: 'The ghost drive accumulates onto existing velocity rather than clobbering it.' });

	// ---- G4: teleport threshold beams the ghost back to the player ----
	PBF.scaleTest('fps/ghost', 'G4', 'teleport threshold beams ghost', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var gap = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 10) {
				p._ghost.position.set(0, 1, 80);
				p._syncGhost(PBF.DT);
				gap = Math.abs(p._ghost.position.z - p.body.position.z);
			}
			return {};
		});

		t.log('Fling the ghost 80 units away, then _syncGhost; the threshold should beam it back onto the player (gap<1).');
		t.expect('ghost beamed back (gap < 1)', function () {
			if (gap == null) return { ok: false, detail: 'settling…' };
			return { ok: gap < 1, detail: 'gapAfterSync=' + gap.toFixed(3) };
		});
		t.simulate(w, 20);
	}, { page: 'fps/ghost', steps: 20, description: 'A ghost flung past the teleport threshold beams straight back onto the player.' });

	// ---- G5: no oscillation at rest over 10s ----
	PBF.scaleTest('fps/ghost', 'G5', 'no oscillation at rest 10s', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var maxGap = 0, maxGhostSpeed = 0;
		PBF.drive(t, p, function (tick) {
			if (tick > 60) {
				var g = Math.hypot(p.body.position.x - p._ghost.position.x, p.body.position.z - p._ghost.position.z);
				maxGap = Math.max(maxGap, g);
				maxGhostSpeed = Math.max(maxGhostSpeed, Math.hypot(p._ghost.linear_velocity.x, p._ghost.linear_velocity.z));
			}
			return {};
		});

		t.log('Stand idle 10 seconds; the ghost must not oscillate (maxGap<0.05, maxGhostSpeed<0.5).');
		t.expect('ghost stays glued to the player (gap < 0.05)', function () {
			return { ok: maxGap < 0.05, detail: 'maxGap=' + maxGap.toFixed(4) };
		});
		t.expect('ghost stays quiet at rest (speed < 0.5)', function () {
			return { ok: maxGhostSpeed < 0.5, detail: 'maxGhostSpeed=' + maxGhostSpeed.toFixed(4) };
		});
		t.simulate(w, 660);
	}, { page: 'fps/ghost', steps: 660, description: 'Idle for 10 seconds; the ghost stays glued to the player with no oscillation.' });

	// ---- G6: crouch height-swap does not strand the ghost ----
	PBF.scaleTest('fps/ghost', 'G6', 'crouch swap no ghost strand', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var worstGap = 0, tick0 = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			if (tick <= 25) return { forward: 1, sprint: true };
			if (tick >= 26 && tick <= 35) {
				if (tick >= 30) {
					var g = Math.hypot(p.body.position.x - p._ghost.position.x, p.body.position.z - p._ghost.position.z);
					if (g > worstGap) worstGap = g;
				}
				return { crouch: true };
			}
			return {};
		});

		t.log('Sprint 25 ticks, then crouch (height swap) and hold it standing still ~10 frames; the ghost must stay glued (gap<1.0) — no strand from the swap.');
		t.expect('ghost stays with player through the crouch hold (gap<1.0)', function () {
			if (tick0 < 31) return { ok: false, detail: 'running…' };
			return { ok: worstGap < 1.0, detail: 'worstGapWhileCrouched=' + worstGap.toFixed(3) };
		});
		t.simulate(w, 40);
	}, { page: 'fps/ghost', steps: 40, description: 'A crouch height-swap mid-sprint must not strand the ghost.' });

	// ---- G7: ghost live on a 25deg ramp — zero drift over ~5s ----
	PBF.scaleTest('fps/ghost', 'G7', 'ghost live: ramp holds still', function (t, S) {
		var w = PBF.makeWorld();
		var rot = PBF.axisAngleQuat(1, 0, 0, 25 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var z0 = null, y0 = null, drift = 0;
		PBF.drive(t, p, function (tick) {
			if (tick === 120) { z0 = p.body.position.z; y0 = p.body.position.y; }
			if (tick > 120 && z0 != null) {
				drift = Math.hypot(p.body.position.z - z0, p.body.position.y - y0);
			}
			return {};
		});

		t.log('Settle on a 25° ramp then idle (~5s); with the ghost live the character must hold DEAD STILL — any drift means the ghost is destabilizing it.');
		t.expect('holds dead still on the ramp (zero drift)', function () {
			if (z0 == null) return { ok: false, detail: 'settling…' };
			return { ok: drift === 0, detail: 'drift=' + drift.toFixed(6) + ' (any drift = ghost slip)' };
		});
		t.simulate(w, 420);
	}, { page: 'fps/ghost', steps: 420, description: 'Rest on a sub-limit ramp with the ghost live; the character must hold DEAD STILL (zero drift). Any drift means the ghost is destabilizing it.' });

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
