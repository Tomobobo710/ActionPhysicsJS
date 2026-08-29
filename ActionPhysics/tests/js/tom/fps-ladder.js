/**
 * Tom's Suite — FPSCharacterController LADDER tests (L1-L6).
 *
 * One controller per test, all 3 scales, ladder sized off a square pillar (1.2x6x1.2 pre-scale,
 * climbable on every face) via S.pillarLadder.
 */
(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	var G = 'fps/ladder', P = 'fps/ladder';

	// ---- L1: walk up to the ladder, climb it fully, reach the top, dismount, fall, land ----
	PBF.scaleTest(G, 'L1', 'walk up, climb to the top, dismount, and land', function (t, S) {
		var w = S.flat();
		S.pillarLadder(w);
		var p = S.spawn(w, { x: 0, y: 0.9 * S.SC + 0.001, z: S.sc(-6) }, {});
		PBF.renderables(t, p);
		var mountedAt = -1, leftTopAt = -1, landedAfterTop = false;
		PBF.drive(t, p, function (tick) {
			if (p._onLadder && mountedAt < 0) mountedAt = tick;
			if (mountedAt > 0 && leftTopAt < 0 && !p._onLadder && p.body.position.y > 5 * S.SC) leftTopAt = tick;
			if (leftTopAt > 0 && p.grounded && p.body.position.y < 1 * S.SC) landedAfterTop = true;
			return { forward: 1 };
		});
		t.log('Walk up to the ladder from a distance, mount it, climb all the way to the top, dismount ' +
			'once the ladder ends, then fall and land back on the real floor.');
		t.expect('mounted the ladder from a walk-up approach', function () {
			return { ok: mountedAt > 0, detail: 'mountedAt=' + mountedAt };
		});
		t.expect('climbed to the top and dismounted there', function () {
			return { ok: leftTopAt > 0, detail: 'leftTopAt=' + leftTopAt + ' y=' + p.body.position.y.toFixed(2) };
		});
		t.expect('fell and landed back on the floor', function () {
			return { ok: landedAfterTop, detail: 'landedAfterTop=' + landedAfterTop + ' y=' + p.body.position.y.toFixed(2) };
		});
		t.simulate(w, 400);
	}, { page: P, steps: 400, description: 'Walking up to the pillar ladder from a distance mounts it, climbs it fully, dismounts at the top, and lands back on the floor.' });

	// ---- L2: same as L1, but sprinting the whole time ----
	PBF.scaleTest(G, 'L2', 'sprint up, climb to the top, dismount, and land', function (t, S) {
		var w = S.flat();
		S.pillarLadder(w);
		var p = S.spawn(w, { x: 0, y: 0.9 * S.SC + 0.001, z: S.sc(-6) }, {});
		PBF.renderables(t, p);
		var mountedAt = -1, leftTopAt = -1, landedAfterTop = false;
		PBF.drive(t, p, function (tick) {
			if (p._onLadder && mountedAt < 0) mountedAt = tick;
			if (mountedAt > 0 && leftTopAt < 0 && !p._onLadder && p.body.position.y > 5 * S.SC) leftTopAt = tick;
			if (leftTopAt > 0 && p.grounded && p.body.position.y < 1 * S.SC) landedAfterTop = true;
			return { forward: 1, sprint: true };
		});
		t.log('Sprint up to the ladder from a distance, mount it, climb all the way to the top, ' +
			'dismount, fall, and land.');
		t.expect('mounted the ladder from a sprint approach', function () {
			return { ok: mountedAt > 0, detail: 'mountedAt=' + mountedAt };
		});
		t.expect('climbed to the top and dismounted there', function () {
			return { ok: leftTopAt > 0, detail: 'leftTopAt=' + leftTopAt + ' y=' + p.body.position.y.toFixed(2) };
		});
		t.expect('fell and landed back on the floor', function () {
			return { ok: landedAfterTop, detail: 'landedAfterTop=' + landedAfterTop + ' y=' + p.body.position.y.toFixed(2) };
		});
		t.simulate(w, 400);
	}, { page: P, steps: 400, description: 'Sprinting up to the pillar ladder mounts, climbs, dismounts at the top, and lands, same as a walk approach.' });

	// ---- L3: strafing into the ladder (pure sideways input, no forward/back) mounts and climbs it ----
	PBF.scaleTest(G, 'L3', 'strafing into the ladder mounts and climbs it', function (t, S) {
		var w = S.flat();
		S.pillarLadder(w);
		// Face ALONG the ladder's wall so "right" strafes straight into its face — a pure sideways
		// approach, never any forward/back input.
		var yaw = Math.PI / 2;
		var p = S.spawn(w, { x: 0, y: 0.9 * S.SC + 0.001, z: S.sc(-6) }, { yaw: yaw });
		PBF.renderables(t, p);
		var mountedAt = -1, leftTopAt = -1, landedAfterTop = false;
		PBF.drive(t, p, function (tick) {
			if (p._onLadder && mountedAt < 0) mountedAt = tick;
			if (mountedAt > 0 && leftTopAt < 0 && !p._onLadder && p.body.position.y > 5 * S.SC) leftTopAt = tick;
			if (leftTopAt > 0 && p.grounded && p.body.position.y < 1 * S.SC) landedAfterTop = true;
			return { right: 1, yaw: yaw };
		});
		t.log('Strafe (pure sideways input, no forward/back) up to the ladder from a distance, mount it, ' +
			'climb all the way to the top, dismount, fall, and land — same journey as L1/L2, driven ' +
			'entirely by strafe.');
		t.expect('strafing alone mounted the ladder from a distance', function () {
			return { ok: mountedAt > 0, detail: 'mountedAt=' + mountedAt };
		});
		t.expect('climbed to the top and dismounted there', function () {
			return { ok: leftTopAt > 0, detail: 'leftTopAt=' + leftTopAt + ' y=' + p.body.position.y.toFixed(2) };
		});
		t.expect('fell and landed back on the floor', function () {
			return { ok: landedAfterTop, detail: 'landedAfterTop=' + landedAfterTop + ' y=' + p.body.position.y.toFixed(2) };
		});
		t.simulate(w, 400);
	}, { page: P, steps: 400, description: 'Strafing sideways into the ladder face (no forward/back input) from a distance mounts, climbs fully, dismounts at the top, and lands, same journey as L1/L2.' });

	// ---- L4: look direction determines climb direction ----
	PBF.scaleTest(G, 'L4', 'look direction determines climb direction', function (t, S) {
		var w = S.flat();
		S.pillarLadder(w);
		// Face the ladder (yaw=PI, forward=-z toward the face at z=0).
		var yaw = Math.PI;
		var p = S.spawn(w, { x: 0, y: 3 * S.SC, z: S.sc(0.6) }, { yaw: yaw });
		PBF.renderables(t, p);
		var vyLevel = null, vyDown = null, vyUp = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 20) vyLevel = p.body.linear_velocity.y;
			if (tick === 70) vyDown = p.body.linear_velocity.y;
			if (tick === 120) vyUp = p.body.linear_velocity.y;
			var pitch = tick < 50 ? 0 : (tick < 100 ? -1.0 : 1.0);
			return { forward: 1, pitch: pitch, yaw: yaw };
		});
		t.log('Mounted, holding forward the whole time: looking level climbs UP, looking DOWN reverses ' +
			'to climb DOWN, looking UP climbs UP again — look direction steers climb.');
		t.expect('level look + forward climbs up', function () {
			if (vyLevel == null) return { ok: false, detail: 'settling…' };
			return { ok: vyLevel > S.sc(0.5), detail: 'vyLevel=' + vyLevel.toFixed(3) };
		});
		t.expect('down look + forward climbs down', function () {
			if (vyDown == null) return { ok: false, detail: 'settling…' };
			return { ok: vyDown < -S.sc(0.5), detail: 'vyDown=' + vyDown.toFixed(3) };
		});
		t.expect('up look + forward climbs up again', function () {
			if (vyUp == null) return { ok: false, detail: 'settling…' };
			return { ok: vyUp > S.sc(0.5), detail: 'vyUp=' + vyUp.toFixed(3) };
		});
		t.simulate(w, 150);
	}, { page: P, steps: 150, description: 'While mounted, look pitch determines climb direction — level or up climbs up, down reverses to climb down.' });

	// ---- L5: no jitter/flicker at the top of the ladder ----
	PBF.scaleTest(G, 'L5', 'no jitter at the top of the ladder', function (t, S) {
		var w = S.flat();
		S.pillarLadder(w);
		var yaw = Math.PI;
		var p = S.spawn(w, { x: 0, y: 5 * S.SC, z: S.sc(0.6) }, { yaw: yaw }); // spawn near the top already
		PBF.renderables(t, p);
		var transitions = 0, lastOnLadder = null;
		PBF.drive(t, p, function () {
			if (lastOnLadder !== null && p._onLadder !== lastOnLadder) transitions++;
			lastOnLadder = p._onLadder;
			return { forward: 1, yaw: yaw };
		});
		t.log('Climb to the top of the ladder and dismount; onLadder should flip from true to false ONCE, ' +
			'not flicker back and forth several times before settling.');
		t.expect('at most one mount/dismount transition at the top (no flicker)', function () {
			return { ok: transitions <= 1, detail: 'transitions=' + transitions };
		});
		t.simulate(w, 120);
	}, { page: P, steps: 120, description: 'Regression: dismounting at the top of the ladder must not flicker between mounted and dismounted several times before settling.' });

	// ---- L6: jump-off dismount feel ----
	PBF.scaleTest(G, 'L6', 'jump-off dismount feel', function (t, S) {
		var w = S.flat();
		S.pillarLadder(w);
		var yaw = Math.PI;
		var p = S.spawn(w, { x: 0, y: 3 * S.SC, z: S.sc(0.6) }, { yaw: yaw });
		PBF.renderables(t, p);
		var mounted = false, offAt = -1, dismountPos = null, awayAfter = false, vyAtDismount = null;
		PBF.drive(t, p, function (tick) {
			if (p._onLadder) mounted = true;
			if (mounted && !p._onLadder && offAt < 0) {
				offAt = tick;
				dismountPos = { x: p.body.position.x, z: p.body.position.z };
				vyAtDismount = p.body.linear_velocity.y;
			}
			if (offAt >= 0 && tick === offAt + 15 && dismountPos) {
				var dx = p.body.position.x - dismountPos.x, dz = p.body.position.z - dismountPos.z;
				awayAfter = Math.hypot(dx, dz) > S.sc(0.2);
			}
			return tick < 40 ? { forward: 1, yaw: yaw } : { jumpPressed: tick === 40, yaw: yaw };
		});
		t.log('Mount, then press jump once; the controller should dismount immediately with a purely ' +
			'horizontal shove away from the ladder face — no upward kick, no phantom jump firing later ' +
			'when gravity brings the player back down.');
		t.expect('mounted before jumping off', function () { return { ok: mounted, detail: 'mounted=' + mounted }; });
		t.expect('dismounted immediately on jump press', function () {
			return { ok: offAt >= 0 && offAt <= 41, detail: 'offAt=' + offAt };
		});
		t.expect('shoved away from the ladder face', function () {
			return { ok: awayAfter, detail: 'awayAfter=' + awayAfter };
		});
		t.expect('no vertical kick on dismount', function () {
			if (vyAtDismount == null) return { ok: false, detail: 'settling…' };
			return { ok: vyAtDismount < 0.05 && vyAtDismount > -0.3, detail: 'vy=' + vyAtDismount.toFixed(4) };
		});
		t.simulate(w, 90);
	}, { page: P, steps: 90, description: 'Pressing jump while on a ladder dismounts with a horizontal-only shove and no phantom jump on landing.' });

	// ---- L7: sliding into a ladder stops the slide ----
	PBF.scaleTest(G, 'L7', 'sliding into a ladder stops the slide', function (t, S) {
		var w = S.flat();
		S.pillarLadder(w);
		// Further back than L1/L2's approach spawn, so there's room to sprint up to slide speed
		// before crouching, then still cover ground into the ladder while sliding.
		var p = S.spawn(w, { x: 0, y: 0.9 * S.SC + 0.001, z: S.sc(-14) }, {});
		PBF.renderables(t, p);
		var enteredSlide = false, mountedAt = -1, slidingAtMount = null, slidingAfterMount = false;
		PBF.drive(t, p, function (tick) {
			if (p.sliding) { enteredSlide = true; }
			if (p._onLadder && mountedAt < 0) {
				mountedAt = tick;
				slidingAtMount = p.sliding;
			}
			if (mountedAt > 0 && p._onLadder && p.sliding) { slidingAfterMount = true; }
			if (tick <= 15) return { forward: 1, sprint: true };
			return { forward: 1, sprint: true, crouch: true };
		});
		t.log('Sprint into a slide, then straight into a ladder. Expect: the slide is entered before reaching the ladder, and mounting clears it immediately — no lingering slide state while climbing.');
		t.expect('entered a slide before reaching the ladder', function () {
			return { ok: enteredSlide, detail: 'enteredSlide=' + enteredSlide };
		});
		t.expect('mounted the ladder', function () {
			return { ok: mountedAt > 0, detail: 'mountedAt=' + mountedAt };
		});
		t.expect('not sliding on the mount tick', function () {
			return { ok: slidingAtMount === false, detail: 'slidingAtMount=' + slidingAtMount };
		});
		t.expect('never reads as sliding while on the ladder', function () {
			return { ok: !slidingAfterMount, detail: 'slidingAfterMount=' + slidingAfterMount };
		});
		t.simulate(w, 120);
	}, { page: P, steps: 120, description: 'Sprint into a slide, then into a ladder — mounting must clear the slide flag immediately, not carry it in stale while climbing.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
