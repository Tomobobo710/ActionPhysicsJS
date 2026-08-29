(function (Runner, PBF, ActionPhysics) {

	var G = 'fps/movement', P = 'fps/movement';

	PBF.scaleTest(G, 'M1', 'idle no-jitter', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var mx = 0;
		PBF.drive(t, p, function (tick) {
			if (tick > 30) {
				var v = p.body.linear_velocity;
				var s = Math.hypot(v.x, v.y, v.z);
				mx = Math.max(mx, s);
			}
			return {};
		});
		t.log('No input on flat ground: the collider should sit dead still, speed under 0.01.');
		t.expect('idle speed stays < 0.01', function () {
			return { ok: mx < 0.01, detail: 'max=' + mx.toFixed(4) };
		});
		t.simulate(w, 120);
	}, { page: P, steps: 120, description: 'Stand idle on flat ground; the collider should not drift or jitter (max speed < 0.01).' });

	PBF.scaleTest(G, 'M2', 'sprint holds 11.5', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var s = 0;
		PBF.drive(t, p, function () {
			s = PBF.hsp(p);
			return { forward: 1, sprint: true };
		});
		t.log('Sprint forward: horizontal speed should settle at ' + S.sc(11.5).toFixed(2) + ' u/s at this scale.');
		t.expect('sprint speed holds ' + S.sc(11.5).toFixed(2), function () {
			return { ok: Math.abs(s - S.sc(11.5)) < S.sc(0.5), detail: 's=' + s.toFixed(3) + ' expect=' + S.sc(11.5).toFixed(2) };
		});
		t.simulate(w, 40);
	}, { page: P, steps: 40, description: 'Sprint forward; steady horizontal speed should hold at 11.5*scale u/s.' });

	PBF.scaleTest(G, 'M3', 'jump showcase (golden baseline)', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var y0 = null, preJumpSpeed = null, jumped = false, jumpTick = null;
		var peakY = -Infinity, peakTick = null, minAirSpeed = Infinity, maxAirSpeed = 0;
		var landedTick = null, landedSpeed = null;
		PBF.drive(t, p, function (tick) {
			if (tick <= 20) return { forward: 1, sprint: true };
			if (y0 === null) { y0 = p.body.position.y; preJumpSpeed = PBF.hsp(p); }
			var j = !jumped;
			if (j) { jumped = true; jumpTick = tick; }
			if (!p.grounded) {
				if (p.body.position.y > peakY) { peakY = p.body.position.y; peakTick = tick; }
				var s = PBF.hsp(p);
				minAirSpeed = Math.min(minAirSpeed, s);
				maxAirSpeed = Math.max(maxAirSpeed, s);
			} else if (jumped && landedTick === null && tick > jumpTick + 2) {
				landedTick = tick; landedSpeed = PBF.hsp(p);
			}
			return { forward: 1, sprint: true, jumpPressed: j };
		});
		t.log('Sprint to speed, jump: watch rise, apex timing, air time, and horizontal speed held rock-solid through the whole arc, then a clean landing.');
		t.expect('rises at least half a jumpSpeed-scaled unit', function () {
			if (y0 === null || peakTick === null) return { ok: false, detail: 'jumping…' };
			var rise = peakY - y0;
			return { ok: rise > S.sc(0.3), detail: 'rise=' + rise.toFixed(3) };
		});
		t.expect('reaches apex within a sane window (not instant, not endless)', function () {
			if (peakTick === null) return { ok: false, detail: 'jumping…' };
			var apexTicks = peakTick - jumpTick;
			return { ok: apexTicks > 3 && apexTicks < 60, detail: 'apexTicks=' + apexTicks };
		});
		t.expect('horizontal speed never bleeds off mid-air (stays within 5% of pre-jump speed)', function () {
			if (landedTick === null) return { ok: false, detail: 'jumping…' };
			var lowOk = minAirSpeed > preJumpSpeed * 0.95;
			var highOk = maxAirSpeed < preJumpSpeed * 1.05;
			return { ok: lowOk && highOk,
				detail: 'preJump=' + preJumpSpeed.toFixed(2) + ' airMin=' + minAirSpeed.toFixed(2) + ' airMax=' + maxAirSpeed.toFixed(2) };
		});
		t.expect('lands cleanly (grounded, no horizontal speed lost on touchdown)', function () {
			if (landedTick === null) return { ok: false, detail: 'jumping…' };
			var keptSpeed = landedSpeed > preJumpSpeed * 0.9;
			return { ok: p.grounded && keptSpeed,
				detail: 'grounded=' + p.grounded + ' landedSpeed=' + landedSpeed.toFixed(2) + ' preJump=' + preJumpSpeed.toFixed(2) };
		});
		t.simulate(w, 100);
	}, { page: P, steps: 100, description: 'Sprint then jump: rise, apex timing, air time, and horizontal speed held through the whole flight, then a clean landing. Golden regression baseline.' });

	PBF.scaleTest(G, 'M4', 'ground slide', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var slid = false;
		PBF.drive(t, p, function (tick) {
			if (tick <= 25) return { forward: 1, sprint: true };
			if (p.sliding) slid = true;
			return { forward: 1, sprint: true, crouch: true };
		});
		t.log('Sprint, then crouch: the controller should enter a slide (_sliding true).');
		t.expect('enters a slide', function () {
			return { ok: slid, detail: 'slid=' + slid };
		});
		t.simulate(w, 50);
	}, { page: P, steps: 50, description: 'Sprint then crouch on flat ground to trigger a slide.' });

	PBF.scaleTest(G, 'M5', 'jump-into-slide', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var slid = false, sp = 0;
		PBF.drive(t, p, function (tick) {
			var t0 = tick - 1;
			var j = (t0 === 15), c = (t0 >= 18);
			if (p.sliding && !slid) { slid = true; sp = PBF.hsp(p); }
			return { forward: 1, sprint: true, jumpPressed: j, crouch: c };
		});
		t.log('Sprint, jump, then crouch in the air to land into a fast slide (>' + S.sc(9).toFixed(2) + ' u/s at slide start).');
		t.expect('lands into slide with speed > ' + S.sc(9).toFixed(2), function () {
			return { ok: slid && sp > S.sc(9), detail: 'slid=' + slid + ' sp=' + sp.toFixed(2) + ' expect>' + S.sc(9).toFixed(2) };
		});

		t.simulate(w, 300);
	}, { page: P, steps: 300, description: 'Sprint, jump, then crouch mid-air to land into a slide carrying speed (>9*scale u/s). Airborne crouch adds real jump height, so this needs a longer window than a plain jump would.' });

	PBF.scaleTest(G, 'M6', 'strafe-turn no jitter', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var jit = 0, prev = null;
		PBF.drive(t, p, function (tick) {
			var t0 = tick - 1;
			var s = PBF.hsp(p);
			if (prev !== null && t0 > 10) jit = Math.max(jit, Math.abs(s - prev));
			prev = s;
			return { forward: 1, sprint: true, yaw: t0 * 0.05 };
		});
		t.log('Sprint while continuously turning (yaw ramp): speed should not spike tick-to-tick (<1.0).');
		t.expect('per-tick speed delta < 1.0', function () {
			return { ok: jit < 1.0, detail: 'maxDelta=' + jit.toFixed(3) };
		});
		t.simulate(w, 120);
	}, { page: P, steps: 120, description: 'Sprint while turning continuously; horizontal speed stays smooth (per-tick delta < 1.0).' });

	PBF.scaleTest(G, 'M7', 'wall-slide preserves tangential', function (t, S) {
		var w = S.flat();

		w.addRigidBody(PBF.staticBox(0.5, 2, 20, { x: -3, y: 2, z: 0 }));
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var reached = false, tangOnWall = 0;
		PBF.drive(t, p, function () {
			var frontX = p.body.position.x - p.width / 2;
			if (frontX < -2.5 + S.sc(0.15) && Math.abs(p.body.position.z) < 19) {
				reached = true;
				tangOnWall = Math.abs(p.body.linear_velocity.z);
			}
			return { forward: 1, right: 1, sprint: true, yaw: 0 };
		});
		t.log('Sprint diagonally into a long wall: the character must REACH it and slide along the face keeping tangential speed (>' + S.sc(3).toFixed(1) + ').');
		t.expect('reaches the wall', function () {
			return { ok: reached, detail: 'reachedWall=' + reached };
		});
		t.expect('tangential speed on the wall > sc(3)', function () {
			if (!reached) return { ok: false, detail: 'never reached the wall' };
			return { ok: tangOnWall > S.sc(3), detail: 'tangential=' + tangOnWall.toFixed(2) + ' expect>' + S.sc(3).toFixed(1) };
		});
		t.simulate(w, 60);
	}, { page: P, steps: 60, description: 'Sprint diagonally into a wall; slide along the face keeping tangential speed (>3).' });

	PBF.scaleTest(G, 'M8a', 'step up onto a step below stepHeight', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var boxH = 0.6 * p.stepHeight;
		w.addRigidBody(PBF.staticBox(4, boxH / 2, 10, { x: 0, y: boxH / 2, z: 13 }));
		var tick0 = 0;
		PBF.renderables(t, p);
		var peakFeetGrounded = -9, onTop = false;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			var feet = p.body.position.y - p.height / 2;
			if (p.grounded) { peakFeetGrounded = Math.max(peakFeetGrounded, feet); if (feet > 0.6 * boxH) onTop = true; }
			var cf = p.body.position.z + p.depth / 2;
			return cf < 3.5 ? { forward: 1, sprint: true } : { forward: 0 };
		});
		t.log('Sprint at a step 0.6× stepHeight tall (below it) then stop: the char should STEP UP and stand on top (feet reach the step top, grounded).');
		t.expect('reached the step top at some point', function () {
			return { ok: onTop, detail: 'peakFeet=' + peakFeetGrounded.toFixed(2) + ' stepTop=' + boxH.toFixed(2) };
		});
		t.expect('ends grounded', function () {
			if (tick0 < 180) return { ok: false, detail: 'climbing…' };
			return { ok: p.grounded, detail: 'grounded=' + p.grounded };
		});
		t.expect('feet settle at the step top height', function () {
			if (tick0 < 180) return { ok: false, detail: 'climbing…' };
			var feet = p.body.position.y - p.height / 2;
			return { ok: Math.abs(feet - boxH) < S.sc(0.1), detail: 'feetEnd=' + feet.toFixed(2) + ' stepTop=' + boxH.toFixed(2) };
		});
		t.simulate(w, 180);
	}, { page: P, steps: 180, description: 'Sprint at a step 0.6× the character\'s scaled stepHeight (below it), then stop; the char steps up and stands on top.' });

	PBF.scaleTest(G, 'M8b', 'stop at a step just above stepHeight', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var boxH = 1.4 * p.stepHeight;
		var faceZ = 3;
		w.addRigidBody(PBF.staticBox(4, boxH / 2, 10, { x: 0, y: boxH / 2, z: 13 }));
		var tick0 = 0;
		PBF.renderables(t, p);
		var maxFeet = -9, maxFront = -9, tookAndHeldTop = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			var feet = p.body.position.y - p.height / 2;
			maxFeet = Math.max(maxFeet, feet);
			maxFront = Math.max(maxFront, p.body.position.z + p.depth / 2);
			if (p.grounded && feet > 0.6 * boxH && p.body.position.z > faceZ) tookAndHeldTop++;
			return { forward: 1, sprint: true };
		});
		t.log('Sprint into a step 1.4× stepHeight tall (just above it): the char must be BLOCKED — it never ends up standing on the step, and never gets past the face.');
		t.expect('never mounts the step (no sustained stand on top)', function () {
			if (tick0 < 150) return { ok: false, detail: 'pushing… maxFeet=' + maxFeet.toFixed(2) + ' stepTop=' + boxH.toFixed(2) };
			var mounted = tookAndHeldTop >= 10;
			return { ok: !mounted, detail: 'heldTopTicks=' + tookAndHeldTop + ' (mount-pop peakFeet=' + maxFeet.toFixed(2) + ' stepTop=' + boxH.toFixed(2) + ')' };
		});
		t.expect('never advances past the step face', function () {
			if (tick0 < 150) return { ok: false, detail: 'pushing…' };
			var passedFace = maxFront > faceZ + S.sc(0.3);
			return { ok: !passedFace, detail: 'maxFront=' + maxFront.toFixed(2) + ' faceZ=' + faceZ };
		});
		t.simulate(w, 150);
	}, { page: P, steps: 150, description: 'Sprint into a step 1.4× the character\'s scaled stepHeight (just above it); the char is blocked and never mounts the step.' });

	PBF.scaleTest(G, 'M9', 'head-bonk no stick', function (t, S) {

		function freeJump() {
			var pw = S.flat();
			var pp = S.feetSpawn(pw, 0, 0, {});
			var h0 = null, peak = -9, jumped = false;
			for (var s = 0; s < 20; s++) { pp.beginStep({}, PBF.DT); pw.step(PBF.DT); pp.endStep(PBF.DT); }
			h0 = pp.body.position.y + pp.height / 2; peak = h0;
			for (var i = 0; i < 90; i++) {
				var cmd = {}; if (!jumped && pp.grounded) { cmd.jumpPressed = true; jumped = true; }
				pp.beginStep(cmd, PBF.DT); pw.step(PBF.DT); pp.endStep(PBF.DT);
				peak = Math.max(peak, pp.body.position.y + pp.height / 2);
			}
			return { h0: h0, apex: peak, rise: peak - h0 };
		}
		var fj = freeJump();
		var clr = 0.5 * fj.rise;

		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var headTop = null, ceilBottom = null;
		var jumped = false, peakHead = -9, stuck = false, tick0 = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			if (tick <= 20) return {};
			if (headTop === null) {
				headTop = p.body.position.y + p.height / 2;
				ceilBottom = headTop + clr;
				var th = S.sc(0.3);
				w.addRigidBody(PBF.staticBox(S.sc(4), th / 2, S.sc(4), { x: 0, y: ceilBottom + th / 2, z: 0 }));
				return {};
			}
			var head = p.body.position.y + p.height / 2;
			peakHead = Math.max(peakHead, head);

			if (jumped && head > headTop + clr * 0.5 && Math.abs(p.body.linear_velocity.y) < 0.01) stuck = true;
			var cmd = {};
			if (!jumped && p.grounded) { cmd.jumpPressed = true; jumped = true; }
			return cmd;
		});

		t.log('Feet-planted, jump into a ceiling placed within jump reach: the char must actually BONK it (jump cut short of its free apex) and then FALL BACK — never stick hanging up high.');
		t.expect('reaches near the ceiling underside', function () {
			if (ceilBottom == null) return { ok: false, detail: 'jumping…' };
			var reachedCeil = peakHead > ceilBottom - S.sc(0.2);
			return { ok: reachedCeil, detail: 'peakHead=' + peakHead.toFixed(2) + ' ceilBottom=' + ceilBottom.toFixed(2) };
		});
		t.expect('the ceiling truncates the jump (peak well short of the free apex)', function () {
			if (tick0 < 120) return { ok: false, detail: 'jumping…' };
			var truncated = (fj.apex - peakHead) > (fj.rise - clr) * 0.5;
			return { ok: truncated, detail: 'peakHead=' + peakHead.toFixed(2) + ' freeApex=' + fj.apex.toFixed(2) };
		});
		t.expect('never sticks hanging at the ceiling', function () {
			if (tick0 < 120) return { ok: false, detail: 'jumping…' };
			return { ok: !stuck, detail: 'stuck=' + stuck };
		});
		t.expect('falls back down after the bonk', function () {
			if (tick0 < 120) return { ok: false, detail: 'jumping…' };
			return { ok: p.grounded, detail: 'grounded=' + p.grounded };
		});
		t.simulate(w, 120);
	}, { page: P, steps: 120, description: 'Feet-planted, jump into a ceiling placed within the jump\'s real reach (self-calibrated from a free-jump probe); the char must actually bonk it and fall back, not stick.' });

	PBF.scaleTest(G, 'M10', 'tall static ledge blocks (no climb)', function (t, S) {
		var w = S.flat();
		var lw = S.sc(4), lh = S.sc(2), ld = S.sc(4);
		w.addRigidBody(PBF.staticBox(lw / 2, lh / 2, ld / 2, { x: 0, y: lh / 2, z: S.sc(4) }));
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var y0 = null, peak = -Infinity;
		PBF.drive(t, p, function (tick) {
			if (tick <= 20) return {};
			if (y0 === null) { y0 = p.body.position.y; peak = y0; }
			peak = Math.max(peak, p.body.position.y);
			return { forward: 1, sprint: true };
		});
		t.log('Sprint into a 2-tall static ledge (taller than stepHeight): must NOT climb it (rise < ' + S.sc(0.3).toFixed(2) + ').');
		t.expect('does not climb tall ledge (rise < ' + S.sc(0.3).toFixed(2) + ')', function () {
			if (y0 === null) return { ok: false, detail: 'settling…' };
			var rose = peak - y0;
			return { ok: rose < S.sc(0.3), detail: 'peakRose=' + rose.toFixed(2) + ' limit=' + S.sc(0.3).toFixed(2) };
		});
		t.simulate(w, 140);
	}, { page: P, steps: 140, description: 'Sprint into a tall static ledge; it should block the character, not be climbable.' });

	PBF.scaleTest(G, 'M11', 'tall dynamic object not climbable', function (t, S) {
		var w = S.flat();
		S.scaledObject(w, 1.2, 8, 3, '#cc4444');
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var y0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick <= 20) return {};
			if (y0 === null) y0 = p.body.position.y;
			return { forward: 1, sprint: true };
		});
		t.log('Sprint into a tall dynamic object: you push it, you do not climb it (net rise < ' + S.sc(0.3).toFixed(2) + ').');
		t.expect('net rise stays < ' + S.sc(0.3).toFixed(2), function () {
			if (y0 === null) return { ok: false, detail: 'settling…' };
			var netRise = p.body.position.y - y0;
			return { ok: netRise < S.sc(0.3), detail: 'netRise=' + netRise.toFixed(2) + ' limit=' + S.sc(0.3).toFixed(2) };
		});
		t.simulate(w, 140);
	}, { page: P, steps: 140, description: 'Sprint into a tall dynamic object; shove it forward rather than climbing on top.' });

	PBF.scaleTest(G, 'M12', 'jump onto low box lands and stays', function (t, S) {
		var w = S.flat();
		var bs = S.sc(0.8);
		var boxZ = S.sc(6);
		var box = PBF.object(w, bs, 2 * S.SC * S.SC * S.SC, { x: 0, y: bs / 2, z: boxZ }, '#0f0');
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var boxFront = boxZ - bs / 2;

		var jumpGap = S.sc(S.SC <= 0.5 ? 0.6 : (S.SC >= 2 ? 1.0 : 0.8));
		var jumped = false, jumpTick = -1, boxStart = null, boxLateSpeed = 0, tick0 = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			if (tick <= 25) return {};
			var cf = p.body.position.z + p.depth / 2;
			if (!jumped) {
				if (boxFront - cf <= jumpGap) { jumped = true; jumpTick = tick; return { forward: 1, jumpPressed: true }; }
				return { forward: 1 };
			}
			var since = tick - jumpTick;
			if (since < 3) return { forward: 1 };
			if (boxStart === null) boxStart = { x: box.position.x, z: box.position.z };
			if (since > 40) boxLateSpeed = Math.max(boxLateSpeed, box.linear_velocity.length());
			return {};
		});
		t.log('Sprint at a low box, jump onto it (jump timed by distance so the arc clears the box), then stand: feet rest on the box top and the box stays put.');
		t.expect('feet rest at the box top height', function () {
			if (tick0 < 150) return { ok: false, detail: 'running…' };
			var feet = p.body.position.y - p.height / 2, boxTop = box.position.y + bs / 2;
			return { ok: Math.abs(feet - boxTop) < S.sc(0.2), detail: 'feet=' + feet.toFixed(2) + ' boxTop=' + boxTop.toFixed(2) };
		});
		t.expect('centered over the box (not hanging off an edge)', function () {
			if (tick0 < 150) return { ok: false, detail: 'running…' };
			var dist = Math.hypot(p.body.position.x - box.position.x, p.body.position.z - box.position.z);
			return { ok: dist < S.sc(0.7), detail: 'dist=' + dist.toFixed(2) };
		});
		t.expect('box does not drift from the landing', function () {
			if (tick0 < 150) return { ok: false, detail: 'running…' };
			var boxDrift = boxStart === null ? 0 :
				Math.hypot(box.position.x - boxStart.x, box.position.z - boxStart.z);
			return { ok: boxDrift < S.sc(1.0), detail: 'boxDrift=' + boxDrift.toFixed(2) };
		});
		t.expect('box settles still (no residual sliding)', function () {
			if (tick0 < 150) return { ok: false, detail: 'running…' };
			return { ok: boxLateSpeed < S.sc(0.3), detail: 'boxSettledSpeed=' + boxLateSpeed.toFixed(2) };
		});
		t.expect('character ends grounded', function () {
			if (tick0 < 150) return { ok: false, detail: 'running…' };
			return { ok: p.grounded, detail: 'grounded=' + p.grounded };
		});
		t.simulate(w, 150);
	}, { page: P, steps: 150, description: 'Sprint at a low box and jump onto it; land and rest on top while the box stays put.' });

	PBF.scaleTest(G, 'M13', 'flat-ground slide reversal is straight, not a U-turn', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var enteredSlide = false, reversed = false, worstLateral = 0, everLeftSlideDuringHold = false;
		PBF.drive(t, p, function (tick) {
			if (tick <= 15) return { forward: 1, sprint: true, yaw: 0 };
			if (tick <= 25) return { forward: 1, sprint: true, crouch: true, yaw: 0 };

			if (p.sliding) { enteredSlide = true; }
			if (enteredSlide) {
				if (!p.sliding) { everLeftSlideDuringHold = true; }
				var v = p.body.linear_velocity;
				worstLateral = Math.max(worstLateral, Math.abs(v.x));
				if (v.z < 0) { reversed = true; }
			}
			return { forward: -1, crouch: true, yaw: 0 };
		});
		t.log('Sprint into a flat-ground slide, then hold straight backward (same yaw) through the whole reversal. Expect: still sliding throughout (no longer exits on backward), velocity reverses along Z, and X (lateral) never drifts far — a straight brake-and-reverse, not an arcing U-turn.');
		t.expect('entered a slide before the reversal hold began', function () {
			return { ok: enteredSlide, detail: 'enteredSlide=' + enteredSlide };
		});
		t.expect('never exits the slide while holding a straight reversal', function () {
			return { ok: !everLeftSlideDuringHold, detail: 'everLeftSlideDuringHold=' + everLeftSlideDuringHold };
		});
		t.expect('velocity actually reverses by the end', function () {
			return { ok: reversed, detail: 'reversed=' + reversed };
		});
		t.expect('lateral drift stays tight through the reversal (< ' + S.sc(1.0).toFixed(2) + ' u/s)', function () {
			return { ok: worstLateral < S.sc(1.0), detail: 'worstLateral=' + worstLateral.toFixed(3) + ' expect<' + S.sc(1.0).toFixed(2) };
		});
		t.simulate(w, 120);
	}, { page: P, steps: 120, description: 'Sprint into a flat-ground slide, then hold straight backward through the reversal — must brake-and-reverse in a straight line (tight lateral drift), staying in the slide the whole time, not U-turn or exit early.' });

	PBF.scaleTest(G, 'M14', 'airborne slide continuation off a ledge', function (t, S) {

		var w = PBF.makeWorld();
		var edgeZ = S.sc(10), dropH = S.sc(3), platformTop = 0, platformHalfH = S.sc(0.5);
		var platform = PBF.staticBox(S.sc(20), platformHalfH, edgeZ / 2 + S.sc(0.01), { x: 0, y: platformTop - platformHalfH, z: edgeZ / 2 }, '#665544');
		w.addRigidBody(platform);
		var lowGround = PBF.staticBox(S.sc(20), platformHalfH, S.sc(30), { x: 0, y: platformTop - dropH - platformHalfH, z: edgeZ + S.sc(15) }, '#444444');
		w.addRigidBody(lowGround);
		var p = S.spawn(w, { x: 0, y: platformTop + 0.9 * S.SC + 0.001, z: 0 }, {});
		PBF.renderables(t, p);
		var enteredSlide = false, wentAirborne = false, slidWhileAirborne = false, droppedSlideMidair = false,
			landed = false, slidingAfterLanding = false;
		PBF.drive(t, p, function (tick) {
			if (tick <= 15) return { forward: 1, sprint: true, yaw: 0 };
			if (tick <= 25) return { forward: 1, sprint: true, crouch: true, yaw: 0 };
			if (p.sliding) { enteredSlide = true; }
			if (enteredSlide && !p.grounded) {
				wentAirborne = true;
				if (p.sliding) { slidWhileAirborne = true; } else { droppedSlideMidair = true; }
			}
			if (wentAirborne && p.grounded && !landed) {
				landed = true;
				slidingAfterLanding = p.sliding;
			}
			return { forward: 1, crouch: true, yaw: 0 };
		});
		t.log('Sprint into a slide, then off a ledge. Expect: still sliding through the airborne phase (not dropped to ordinary air-control), and still sliding immediately on landing (fast enough on flat ground to sustain it).');
		t.expect('entered a slide before reaching the ledge', function () {
			return { ok: enteredSlide, detail: 'enteredSlide=' + enteredSlide };
		});
		t.expect('actually went airborne off the ledge', function () {
			return { ok: wentAirborne, detail: 'wentAirborne=' + wentAirborne };
		});
		t.expect('stayed sliding through the airborne phase (never dropped to ordinary air-control)', function () {
			return { ok: slidWhileAirborne && !droppedSlideMidair,
				detail: 'slidWhileAirborne=' + slidWhileAirborne + ' droppedSlideMidair=' + droppedSlideMidair };
		});
		t.expect('landed', function () {
			return { ok: landed, detail: 'landed=' + landed };
		});
		t.expect('still sliding immediately on landing', function () {
			return { ok: slidingAfterLanding, detail: 'slidingAfterLanding=' + slidingAfterLanding };
		});
		t.simulate(w, 220);
	}, { page: P, steps: 220, description: 'Sprint into a slide, off a ledge, and confirm the slide persists through the airborne phase and continues immediately on landing (fast enough on flat ground below).' });

	PBF.scaleTest(G, 'M15', 'slideBoost launches speed at slide entry', function (t, S) {
		var boost = 1.3;
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, { slideBoost: boost });
		PBF.renderables(t, p);
		var preEntrySpeed = -1, entrySpeed = -1, oneTickLaterSpeed = -1, wasSliding = false;
		PBF.drive(t, p, function (tick) {
			if (!p.sliding) { preEntrySpeed = PBF.hsp(p); }
			var cmd = (tick <= 15) ? { forward: 1, sprint: true } : { forward: 1, sprint: true, crouch: true };
			if (!wasSliding && p.sliding) { entrySpeed = PBF.hsp(p); }
			if (entrySpeed >= 0 && oneTickLaterSpeed < 0 && wasSliding) { oneTickLaterSpeed = PBF.hsp(p); }
			wasSliding = p.sliding;
			return cmd;
		});
		t.log('Sprint then crouch into a slide (slideBoost=' + boost + '). Expect: speed right at the slide-entry tick is measurably boosted above pre-entry speed (roughly x' + boost + ', modulo the same tick\'s own friction/slope decay) — not applied again on later ticks.');
		t.expect('entry speed is boosted above pre-entry speed', function () {
			return { ok: entrySpeed > preEntrySpeed * 1.05,
				detail: 'preEntrySpeed=' + preEntrySpeed.toFixed(2) + ' entrySpeed=' + entrySpeed.toFixed(2) + ' expect entry > preEntry*1.05' };
		});
		t.expect('boost is close to the configured multiplier (within 15%, allowing for same-tick decay)', function () {
			var ratio = entrySpeed / preEntrySpeed;
			return { ok: ratio > boost * 0.85 && ratio <= boost * 1.02,
				detail: 'ratio=' + ratio.toFixed(3) + ' expect~' + boost };
		});
		t.simulate(w, 40);
	}, { page: P, steps: 40, description: 'Sprint then crouch into a slide with slideBoost configured; the entry tick\'s speed should be boosted roughly by that multiplier over the pre-entry speed, applied once.' });

	function ledgeClearanceScene(w, S) {
		var topY = S.sc(1.6), hy = topY / 2, hx = S.sc(2), hz = S.sc(2);
		var ledge = PBF.staticBox(hx, hy, hz, { x: 0, y: hy, z: hz }, '#7a6a52');
		w.addRigidBody(ledge);
		var spawnZ = -(hz + S.sc(1));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		return { p: p, topY: topY, hx: hx, hz: hz };
	}

	PBF.scaleTest(G, 'M16', 'crouch-jump clears a ledge a plain jump cannot', function (t, S) {
		var w = S.flat();
		var scene = ledgeClearanceScene(w, S);
		var p = scene.p;
		PBF.renderables(t, p);
		var landedOnTop = false;
		PBF.drive(t, p, function (tick) {
			var onLedgeFootprint = Math.abs(p.body.position.x) < scene.hx &&
				p.body.position.z > 0 && p.body.position.z < 2 * scene.hz;
			if (p.grounded && onLedgeFootprint && p.body.position.y > scene.topY) { landedOnTop = true; }
			return {
				forward: tick < 40 ? 1 : 0,
				jumpPressed: tick === 8,
				crouch: tick >= 9 && tick <= 50
			};
		});
		t.log('Jump and crouch mid-air (top-anchored collider shrink) toward a ledge (1.6×SC — the same height as the 4th stair step in-game) too tall for a plain jump to clear; the crouch-jump should land on top.');
		t.expect('landed on top of the ledge', function () {
			return { ok: landedOnTop, detail: 'landedOnTop=' + landedOnTop + ' pos=(' + p.body.position.x.toFixed(2) + ',' + p.body.position.y.toFixed(2) + ',' + p.body.position.z.toFixed(2) + ')' };
		});
		t.simulate(w, 90);
	}, { page: P, steps: 90, description: 'Jumping and crouching mid-air toward a 1.6×SC ledge (same height as the 4th in-game stair step) clears it and lands on top.' });

	PBF.scaleTest(G, 'M17', 'plain jump (no crouch) does not clear the same ledge', function (t, S) {
		var w = S.flat();
		var scene = ledgeClearanceScene(w, S);
		var p = scene.p;
		PBF.renderables(t, p);
		var landedOnTop = false;
		PBF.drive(t, p, function (tick) {
			var onLedgeFootprint = Math.abs(p.body.position.x) < scene.hx &&
				p.body.position.z > 0 && p.body.position.z < 2 * scene.hz;
			if (p.grounded && onLedgeFootprint && p.body.position.y > scene.topY) { landedOnTop = true; }
			return { forward: tick < 40 ? 1 : 0, jumpPressed: tick === 8 };
		});
		t.log('Control for M16: identical jump and approach at the SAME ledge, but no crouch — should bounce off the face and land back on the floor, not on top.');
		t.expect('did NOT land on top (control case)', function () {
			return { ok: !landedOnTop, detail: 'landedOnTop=' + landedOnTop + ' pos=(' + p.body.position.x.toFixed(2) + ',' + p.body.position.y.toFixed(2) + ',' + p.body.position.z.toFixed(2) + ')' };
		});
		t.simulate(w, 90);
	}, { page: P, steps: 90, description: 'Control for M16: the same jump at the same ledge WITHOUT crouching does not clear it — proves M16\'s clearance comes from the crouch, not the ledge being trivially jumpable.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
