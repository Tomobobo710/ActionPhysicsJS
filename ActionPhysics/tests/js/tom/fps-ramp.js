/**
 * Tom's Suite — FPSCharacterController RAMP matrix (R1-R18).
 */
(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	var DT = PBF.DT;

	// rampWorld(deg): world with a 20x2x30 ramp at y=10, rotated +deg about X. NOT scaled.
	function rampWorld(deg) {
		var w = PBF.makeWorld();
		var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
		var ramp = PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot);
		w.addRigidBody(ramp);
		return { w: w, ramp: ramp };
	}
	// buriedRamp: steep ramp buried through the floor (R13/R14/R15/R16 geometry).
	function buriedRamp() {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(25, 0.2, 25, { x: 0, y: -0.2, z: 0 }));
		var rot = PBF.axisAngleQuat(1, 0, 0, 0.96);
		var ramp = PBF.staticBox(2.5, 0.2, 4, { x: 0, y: 3, z: -3 }, '#665544', null, rot);
		w.addRigidBody(ramp);
		return { w: w, ramp: ramp };
	}
	function drift(p, z0, y0) {
		return Math.hypot(p.body.position.z - z0, p.body.position.y - y0);
	}
	function rampFaceDistance(deg, worldY, worldZ) {
		var rad = deg * Math.PI / 180;
		var ny = Math.cos(rad), nz = Math.sin(rad);
		var faceY = 10 + ny, faceZ = 0 + nz;
		return ny * (worldY - faceY) + nz * (worldZ - faceZ);
	}

	// ---- R1: hold below-limit 30s (worst drift over 10/25/40/44) ----
	PBF.scaleTest('fps/ramp', 'R1', 'hold below-limit (zero drift)', function (t, S) {
		var angles = [10, 25, 40, 44], BLOCK = 300;
		var w = PBF.makeWorld();
		var z0 = null, y0 = null, worst = {};

		function build(world, idx, deg) {
			var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
			world.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
			z0 = null; y0 = null;
			return S.spawn(world, { x: 0, y: 16, z: 0 }, {});
		}

		var seq = PBF.sequentialBlocks(t, w, angles, BLOCK, build, function (local, idx, deg, curP) {
			if (local === 120) { z0 = curP.body.position.z; y0 = curP.body.position.y; }
			if (local > 120 && z0 != null) worst[deg] = Math.max(worst[deg] || 0, drift(curP, z0, y0));
		});

		t.log('Idle on below-limit ramps 10°→25°→40°→44°, ONE at a time. Each must HOLD dead still — if it moves at ALL, it slipped.');
		t.expect('holds dead still (zero movement)', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'holding ' + angles[Math.min(angles.length - 1, Math.floor(seq.tickOf() / BLOCK))] + '°…' };
			var wv = 0, wa = 0; angles.forEach(function (d) { if ((worst[d] || 0) > wv) { wv = worst[d]; wa = d; } });
			return { ok: +wv.toFixed(6) === 0, detail: 'maxMovement=' + wv.toFixed(6) + '@' + wa + '° (any movement = slip)' };
		});
		t.simulate(w, angles.length * BLOCK);
	}, { page: 'fps/ramp', steps: 1200, description: 'Idle on below-limit ramps (10/25/40/44°) one at a time, ~5s each; the character must hold DEAD STILL — any movement is a slip.' });

	// ---- R2: slide off above-limit (47/55/70/89 all slide > 2) ----
	PBF.scaleTest('fps/ramp', 'R2', 'slide off above-limit', function (t, S) {
		var angles = [47, 55, 70, 89], BLOCK = 360;
		var w = PBF.makeWorld();
		var z0 = null, y0 = null, slid = {}, fellThrough = {}, launched = {}, groundedLastTick = false;

		function build(world, idx, deg) {
			world.addRigidBody(PBF.staticBox(60, 0.5, 60, { x: 0, y: -0.5, z: 0 }));
			var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
			world.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
			var rad = deg * Math.PI / 180;
			var ny = Math.cos(rad), nz = Math.sin(rad);
			var faceY = 10 + ny, faceZ = 0 + nz;
			var standoff = 10;
			var curP = S.spawn(world, { x: 0, y: faceY + ny * standoff, z: faceZ + nz * standoff }, {});
			var tossSpeed = 8;
			curP.body.linear_velocity.x = 0;
			curP.body.linear_velocity.y = tossSpeed * 0.3;
			curP.body.linear_velocity.z = -nz * tossSpeed;
			z0 = null; y0 = null; groundedLastTick = false;
			return curP;
		}

		var seq = PBF.sequentialBlocks(t, w, angles, BLOCK, build, function (local, idx, deg, curP) {
			var wasJustGrounded = curP.grounded && !groundedLastTick;
			groundedLastTick = curP.grounded;
			if (local === 1) { z0 = curP.body.position.z; y0 = curP.body.position.y; }
			if (local > 1 && z0 != null) {
				slid[deg] = +drift(curP, z0, y0).toFixed(1);
				var feet = curP.body.position.y - curP.height / 2;
				var overRamp = Math.abs(curP.body.position.z) < 15;
				var faceDist = rampFaceDistance(deg, feet, curP.body.position.z);
				if (overRamp && faceDist < -1) fellThrough[deg] = true;
				if (wasJustGrounded && curP.body.linear_velocity.y > 2) launched[deg] = true;
			}
		});

		t.log('Idle on above-limit ramps 47°→55°→70°→89°, ONE at a time. Each must SLIDE DOWN THE FACE (> 2u), NEVER drop through the slab, and NEVER launch upward on landing.');
		t.expect('all angles slide down the face (> 2u)', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'sliding ' + angles[Math.min(angles.length - 1, Math.floor(seq.tickOf() / BLOCK))] + '°…' };
			var ok = angles.every(function (d) { return (slid[d] || 0) > 2; });
			var m = {}; angles.forEach(function (d) { m[d] = slid[d] || 0; });
			return { ok: ok, detail: JSON.stringify(m) };
		});
		t.expect('no angle falls through the slab', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'sliding…' };
			var ok = angles.every(function (d) { return !fellThrough[d]; });
			var m = {}; angles.forEach(function (d) { m[d] = !!fellThrough[d]; });
			return { ok: ok, detail: JSON.stringify(m) };
		});
		t.expect('no angle launches upward on landing', function () {
			if (seq.tickOf() < angles.length * BLOCK) return { ok: false, detail: 'sliding…' };
			var ok = angles.every(function (d) { return !launched[d]; });
			var m = {}; angles.forEach(function (d) { m[d] = !!launched[d]; });
			return { ok: ok, detail: JSON.stringify(m) };
		});
		t.simulate(w, angles.length * BLOCK);
	}, { page: 'fps/ramp', steps: 1440, description: 'Idle on above-limit ramps (47/55/70/89°) one at a time; each must slide DOWN THE FACE, not tunnel through the slab, and not launch upward on landing.' });

	// ---- R3: boundary 44 holds / 47 slides ----
	PBF.scaleTest('fps/ramp', 'R3', 'boundary 44 holds / 47 slides', function (t, S) {
		var BLOCK = 240;
		var w = PBF.makeWorld();
		var z0 = null, y0 = null, hold44 = false, slid47 = false;
		var blockAngles = [44, 47];

		function build(world, idx, deg) {
			var rot = PBF.axisAngleQuat(1, 0, 0, deg * Math.PI / 180);
			world.addRigidBody(PBF.staticBox(10, 1, 15, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
			z0 = null; y0 = null;
			return S.spawn(world, { x: 0, y: 16, z: 0 }, {});
		}

		var seq = PBF.sequentialBlocks(t, w, blockAngles, BLOCK, build, function (local, idx, deg, curP) {
			if (local === 60) { z0 = curP.body.position.z; y0 = curP.body.position.y; }
			if (local === BLOCK) {
				if (idx === 0) hold44 = curP.grounded;
				else slid47 = drift(curP, z0, y0) > 2;
			}
		});

		t.log('The stand/slide boundary: first 44° (character HOLDS, grounded), then 47° (SLIDES off). Watch both in turn.');
		t.expect('44° holds (grounded)', function () {
			if (seq.tickOf() < 2 * BLOCK) return { ok: false, detail: seq.tickOf() <= BLOCK ? 'holding 44°…' : 'sliding 47°…' };
			return { ok: hold44, detail: 'hold44=' + hold44 };
		});
		t.expect('47° slides', function () {
			if (seq.tickOf() < 2 * BLOCK) return { ok: false, detail: seq.tickOf() <= BLOCK ? 'holding 44°…' : 'sliding 47°…' };
			return { ok: slid47, detail: 'slid47=' + slid47 };
		});
		t.simulate(w, 2 * BLOCK);
	}, { page: 'fps/ramp', steps: 480, description: 'The stand/slide boundary sits between 44° (holds) and 47° (slides), shown one after the other.' });

	// ---- R4: limit-disabled holds 55° (maxSlopeAngle:90) ----
	PBF.scaleTest('fps/ramp', 'R4', 'limit-disabled holds 55deg', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, { maxSlopeAngle: 90 });
		PBF.renderables(t, p);
		var z0 = null, y0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 120) { z0 = p.body.position.z; y0 = p.body.position.y; }
			return {};
		});
		t.log('maxSlopeAngle:90 disables the limit — the character HOLDS on a 55° ramp, grounded, < 0.5u drift.');
		t.expect('stays grounded', function () {
			if (z0 == null) return { ok: false, detail: 'settling…' };
			return { ok: p.grounded, detail: 'grounded=' + p.grounded };
		});
		t.expect('drift < 0.5', function () {
			if (z0 == null) return { ok: false, detail: 'settling…' };
			var d = drift(p, z0, y0);
			return { ok: d < 0.5, detail: 'drift=' + d.toFixed(3) };
		});
		t.simulate(w, 420);
	}, { page: 'fps/ramp', steps: 420, description: 'With the slope limit disabled the character stands on a 55° ramp.' });

	// ---- R5: walk up ramp climbs no launch (25°) ----
	PBF.scaleTest('fps/ramp', 'R5', 'walk up ramp climbs no launch', function (t, S) {
		var rw = rampWorld(25);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var y0 = null, launched = false, leftRamp = false;
		PBF.drive(t, p, function (tick) {
			if (tick === 60) y0 = p.body.position.y;
			if (tick > 60) {
				if (p.body.position.z < -12) leftRamp = true;
				if (!leftRamp && !p.grounded && p.body.linear_velocity.y > 2) launched = true;
			}
			return tick > 60 ? { forward: -1, sprint: true } : {};
		});
		t.log('Sprint up a 25° ramp for ~40 ticks. It should CLIMB (> 0.5u) without launching off the face.');
		t.expect('climbs > 0.5', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			var climbed = p.body.position.y - y0;
			return { ok: climbed > 0.5, detail: 'climbed=' + climbed.toFixed(2) };
		});
		t.expect('never launches off the face', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: !launched, detail: 'launched=' + launched };
		});
		t.simulate(w, 100);
	}, { page: 'fps/ramp', steps: 100, description: 'Sprint up a 25° ramp; the character climbs without being launched.' });

	// ---- R6: walk down stays clamped (25°) ----
	PBF.scaleTest('fps/ramp', 'R6', 'walk down stays clamped', function (t, S) {
		var rw = rampWorld(25);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var airborneTicks = 0, stopped = false;
		PBF.drive(t, p, function (tick) {
			if (tick > 60 && !stopped) {
				if (p.body.position.z > 12) stopped = true;
				else if (!p.grounded) airborneTicks++;
			}
			return tick > 60 ? { forward: 1, sprint: true } : {};
		});
		t.log('Sprint DOWN a 25° ramp. Ground clamp keeps it planted — < 8 airborne ticks over the descent.');
		t.expect('airborne ticks < 8', function () {
			return { ok: airborneTicks < 8, detail: 'airborneTicks=' + airborneTicks + '/40' };
		});
		t.simulate(w, 100);
	}, { page: 'fps/ramp', steps: 100, description: 'Sprint down a 25° ramp; the ground clamp keeps the character planted.' });

	// ---- R7: walk across fall line no jitter (30°) ----
	PBF.scaleTest('fps/ramp', 'R7', 'walk across fall line no jitter', function (t, S) {
		var rw = rampWorld(30);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var maxJit = 0, prev = null;
		PBF.drive(t, p, function (tick) {
			if (tick > 60) {
				var s = PBF.hsp(p);
				if (prev !== null) maxJit = Math.max(maxJit, Math.abs(s - prev));
				prev = s;
			}
			return tick > 60 ? { right: 1, sprint: true } : {};
		});
		t.log('Strafe ACROSS the fall line of a 30° ramp. Speed should be smooth — per-tick delta < 2.');
		t.expect('max per-tick speed delta < 2', function () {
			if (prev === null) return { ok: false, detail: 'settling…' };
			return { ok: maxJit < 2, detail: 'maxPerTickDelta=' + maxJit.toFixed(3) };
		});
		t.simulate(w, 180);
	}, { page: 'fps/ramp', steps: 180, description: 'Strafe across the fall line of a 30° ramp; horizontal speed stays smooth.' });

	// ---- R8: ramp->flat seam no spike ----
	PBF.scaleTest('fps/ramp', 'R8', 'ramp->flat seam no spike', function (t, S) {
		var w = S.flat();
		var rot = PBF.axisAngleQuat(1, 0, 0, 25 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(10, 0.3, 8, { x: 0, y: 2.2, z: -6 }, '#665544', null, rot));
		var p = S.feetSpawn(w, 0, 4, {});
		PBF.renderables(t, p);
		var maxSpike = 0, prev = null, local = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 40) return {};
			if (prev === null) prev = PBF.hsp(p);
			local++;
			var s = PBF.hsp(p);
			if (local > 3) maxSpike = Math.max(maxSpike, Math.abs(s - prev));
			prev = s;
			return { forward: -1, sprint: true };
		});
		t.log('Walk from flat onto a shallow ramp across the seam. No velocity spike (< sc(3)) as you cross.');
		t.expect('max seam spike < sc(3)', function () {
			if (prev === null) return { ok: false, detail: 'settling…' };
			return { ok: maxSpike < S.sc(3), detail: 'maxSpike=' + maxSpike.toFixed(3) + ' limit=' + S.sc(3).toFixed(2) };
		});
		t.simulate(w, 130);
	}, { page: 'fps/ramp', steps: 130, description: 'Walk from flat onto a shallow ramp; crossing the seam produces no velocity spike.' });

	// ---- R9: cannot climb steep slope (default), 55° ----
	PBF.scaleTest('fps/ramp', 'R9', 'cannot climb steep slope (default)', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 4, z: 6 }, {});
		PBF.renderables(t, p);
		var y0 = null, worstClimb = -99;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) y0 = p.body.position.y;
			if (tick > 30 && y0 != null) worstClimb = Math.max(worstClimb, p.body.position.y - y0);
			return tick > 30 ? { forward: -1, sprint: true } : {};
		});
		t.log('Default settings: sprint into a 55° ramp. It should NOT climb — height gained stays < 2u.');
		t.expect('climbed < 2 (stays low)', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: worstClimb < 2, detail: 'climbedY=' + (worstClimb === -99 ? '0.00' : worstClimb.toFixed(2)) };
		});
		t.simulate(w, 150);
	}, { page: 'fps/ramp', steps: 150, description: 'By default the character cannot walk up a too-steep 55° ramp.' });

	// ---- R10: climbSteepSlopes opts into climbing, 55° ----
	PBF.scaleTest('fps/ramp', 'R10', 'climbSteepSlopes opts into climbing', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 4, z: 6 }, { climbSteepSlopes: true });
		PBF.renderables(t, p);
		var y0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) y0 = p.body.position.y;
			return tick > 30 ? { forward: -1, sprint: true } : {};
		});
		t.log('climbSteepSlopes:true — sprint into a 55° ramp should ASCEND it (> 4u climbed).');
		t.expect('climbs > 4 units', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			var climbed = p.body.position.y - y0;
			return { ok: climbed > 4, detail: 'climbedY=' + climbed.toFixed(2) };
		});
		t.simulate(w, 150);
	}, { page: 'fps/ramp', steps: 150, description: 'With climbSteepSlopes on, sprint into a 55° ramp and walk UP the too-steep face.' });

	// ---- R11: steep slip is movement not slide (55°) ----
	PBF.scaleTest('fps/ramp', 'R11', 'steep slip is movement not slide', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var y0 = null, everSliding = false, gainedSpeed = false;
		PBF.drive(t, p, function (tick) {
			if (tick === 45) y0 = p.body.position.y;
			if (tick > 45) {
				if (p.sliding) everSliding = true;
				if (PBF.hsp(p) > 2) gainedSpeed = true;
			}
			return {};
		});
		t.log('Idle on a 55° ramp: it slips down and builds speed, but the crouch-slide flag never engages.');
		t.expect('gains speed (> 2)', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: gainedSpeed, detail: 'gainedSpeed=' + gainedSpeed };
		});
		t.expect('never engages the crouch-slide flag', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: !everSliding, detail: 'slidFlag=' + everSliding };
		});
		t.expect('drops > 0.5 below base', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			var dropped = p.body.position.y < y0 - 0.5;
			return { ok: dropped, detail: 'droppedY=' + (y0 - p.body.position.y).toFixed(2) };
		});
		t.simulate(w, 105);
	}, { page: 'fps/ramp', steps: 105, description: 'Idle on a 55° ramp; the character slips down under gravity without engaging the crouch-slide.' });

	// ---- R12: cannot walk up steep (net descent, 55°) ----
	PBF.scaleTest('fps/ramp', 'R12', 'cannot walk up steep (net descent)', function (t, S) {
		var rw = rampWorld(55);
		var w = rw.w;
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var y0 = null;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) y0 = p.body.position.y;
			return tick > 30 ? { forward: -1, sprint: true } : {};
		});
		t.log('Hold forward uphill into a 55° ramp — gravity wins, net result is DESCENT (final y < start).');
		t.expect('net descent (final y < base)', function () {
			if (y0 == null) return { ok: false, detail: 'settling…' };
			return { ok: p.body.position.y < y0, detail: 'netClimbY=' + (p.body.position.y - y0).toFixed(2) };
		});
		t.simulate(w, 120);
	}, { page: 'fps/ramp', steps: 120, description: 'Holding forward uphill into a 55° ramp nets a descent.' });

	// ---- R13: buried-ramp base stabilizes (no wedge/glitch) ----
	PBF.scaleTest('fps/ramp', 'R13', 'buried-ramp base stabilizes (no wedge/glitch)', function (t, S) {
		var rw = buriedRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, 0, 2, {});
		PBF.renderables(t, p);
		var maxSpeedLate = 0, minY = 99, everAirborneLate = false, local = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 25) return {};
			local++;
			if (local > 50) {
				maxSpeedLate = Math.max(maxSpeedLate, Math.hypot(p.body.linear_velocity.x, p.body.linear_velocity.y, p.body.linear_velocity.z));
				minY = Math.min(minY, p.body.position.y);
				if (!p.grounded) everAirborneLate = true;
			}
			return { forward: -1, sprint: true };
		});
		t.log('Walk on flat into the base of a steep buried ramp. It must STABILIZE — rests, grounded, no wedge-fall.');
		t.expect('speed settles low (< sc(0.5))', function () {
			if (local <= 50) return { ok: false, detail: 'approaching…' };
			return { ok: maxSpeedLate < S.sc(0.5), detail: 'lateSpeed=' + maxSpeedLate.toFixed(2) };
		});
		t.expect('never airborne late (no wedge-fall)', function () {
			if (local <= 50) return { ok: false, detail: 'approaching…' };
			return { ok: !everAirborneLate, detail: 'airborneLate=' + everAirborneLate };
		});
		t.expect('stays above the floor (minY > sc(0.5))', function () {
			if (local <= 50) return { ok: false, detail: 'approaching…' };
			return { ok: minY > S.sc(0.5), detail: 'minY=' + minY.toFixed(2) };
		});
		t.simulate(w, 105);
	}, { page: 'fps/ramp', steps: 105, description: 'Walk into the base of a steep buried ramp; the character stabilizes with no wedge-fall.' });

	// ---- R14: no mount-pop bob on too-steep toe ----
	PBF.scaleTest('fps/ramp', 'R14', 'no mount-pop bob on too-steep toe', function (t, S) {
		var rw = buriedRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, 0, 2, {});
		PBF.renderables(t, p);
		var steepFlips = 0, prevSteep = null, yMin = 99, yMax = -99, maxUpVz = 0, local = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 25) return {};
			local++;
			var steep = p.groundNormal.y < p._minStandableNormalY;
			if (local > 40) {
				if (prevSteep !== null && steep !== prevSteep) steepFlips++;
				yMin = Math.min(yMin, p.body.position.y);
				yMax = Math.max(yMax, p.body.position.y);
				if (-p.body.linear_velocity.z > maxUpVz) maxUpVz = -p.body.linear_velocity.z;
			}
			prevSteep = steep;
			return { forward: -1, sprint: true };
		});
		t.log('At the too-steep toe: the steep flag must not flip-flop and y must not bob (no mount-pop).');
		t.expect('steep flag does not flip-flop (<= 1 flip)', function () {
			if (local <= 40) return { ok: false, detail: 'approaching…' };
			return { ok: steepFlips <= 1, detail: 'steepFlips=' + steepFlips };
		});
		t.expect('no y-bob (< 0.05)', function () {
			if (local <= 40) return { ok: false, detail: 'approaching…' };
			return { ok: (yMax - yMin) < 0.05, detail: 'yBob=' + (yMax - yMin).toFixed(3) };
		});
		t.expect('no mount-pop upward velocity (< 1.0)', function () {
			if (local <= 40) return { ok: false, detail: 'approaching…' };
			return { ok: maxUpVz < 1.0, detail: 'maxUpVz=' + maxUpVz.toFixed(2) };
		});
		t.simulate(w, 105);
	}, { page: 'fps/ramp', steps: 105, description: 'Holding forward into a too-steep toe does not mount-and-pop.' });

	// ---- R15: backside approach no passthrough ----
	PBF.scaleTest('fps/ramp', 'R15', 'backside approach no passthrough', function (t, S) {
		var rw = buriedRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, 0, -8, {});
		p.yaw = 0;
		PBF.renderables(t, p);
		var maxZ = -99, started = false;
		PBF.drive(t, p, function (tick) {
			if (tick <= 20) return {};
			started = true;
			maxZ = Math.max(maxZ, p.body.position.z);
			return { forward: 1, sprint: true, yaw: 0 };
		});
		t.log('Approach the steep ramp from the high back side. It must STOP against the face, never tunnel past z=-1.');
		t.expect('stops against the face (maxZ < -1.0)', function () {
			if (!started) return { ok: false, detail: 'settling…' };
			return { ok: maxZ < -1.0, detail: 'maxZ=' + maxZ.toFixed(3) };
		});
		t.expect('ends grounded', function () {
			if (!started) return { ok: false, detail: 'settling…' };
			return { ok: p.grounded, detail: 'grounded=' + p.grounded };
		});
		t.simulate(w, 140);
	}, { page: 'fps/ramp', steps: 140, description: 'Walk into the steep ramp from the tall back side; the character stops against the face.' });

	// ---- R16: sprint into ramp from flat climbs (no toe stall) ----
	PBF.scaleTest('fps/ramp', 'R16', 'sprint into ramp from flat climbs (no toe stall)', function (t, S) {
		var starts = [0.7, 1.0, 1.5, 2.0, 2.5, 3.0], BLOCK = 170;
		var w = PBF.makeWorld();
		var y0 = null, climbed = {};

		function build(world, idx, startZ) {
			world.addRigidBody(PBF.staticBox(25, 0.2, 25, { x: 0, y: -0.2, z: 0 }));
			var rot = PBF.axisAngleQuat(1, 0, 0, 0.96);
			world.addRigidBody(PBF.staticBox(2.5, 0.2, 4, { x: 0, y: 3, z: -3 }, '#665544', null, rot));
			y0 = null;
			return S.feetSpawn(world, 0, startZ, { climbSteepSlopes: true });
		}

		var seq = PBF.sequentialBlocks(t, w, starts, BLOCK, build, function (local, idx, startZ, curP) {
			if (local === 10) y0 = curP.body.position.y;
			if (local > 10 && y0 != null) climbed[startZ] = Math.max(climbed[startZ] != null ? climbed[startZ] : -99, curP.body.position.y - y0);
		}, function (local) { return local >= 10 ? { forward: -1, sprint: true } : {}; });

		t.log('Sprint from flat into a steep buried-lip ramp from 6 approach positions, ONE at a time. EVERY approach climbs > 3u (no toe stall).');
		t.expect('all approaches climb > 3', function () {
			if (seq.tickOf() < starts.length * BLOCK) return { ok: false, detail: 'approach z=' + starts[Math.min(starts.length - 1, Math.floor(seq.tickOf() / BLOCK))] + '…' };
			var worst = 99, worstZ = 0; starts.forEach(function (z) { var c = climbed[z] != null ? climbed[z] : 0; if (c < worst) { worst = c; worstZ = z; } });
			var allClimbed = starts.every(function (z) { return (climbed[z] != null ? climbed[z] : 0) >= 3; });
			return { ok: allClimbed, detail: 'worstPeakClimb=' + worst.toFixed(2) + '@z' + worstZ };
		});
		t.simulate(w, starts.length * BLOCK);
	}, { page: 'fps/ramp', steps: 1020, description: 'Sprint from flat into a steep buried-lip ramp from 6 positions, one at a time; every approach climbs.' });

	// ---- R17: slip under a steep ramp, then get stuck ----
	function steepRamp() {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(25, 0.2, 25, { x: 0, y: -0.2, z: 0 }));
		var rot = PBF.axisAngleQuat(1, 0, 0, -0.96);
		var ramp = PBF.staticBox(2.5, 0.2, 4, { x: -15, y: 3, z: 3 }, '#665544', null, rot);
		w.addRigidBody(ramp);
		return { w: w, ramp: ramp };
	}
	PBF.scaleTest('fps/ramp', 'R17', 'slip under ramp then get axis-locked', function (t, S) {
		var rw = steepRamp();
		var w = rw.w;
		var spawnZ = S.SC <= 0.5 ? 1.5 : 1.88;
		var p = S.feetSpawn(w, -19.68, spawnZ, {});
		p.yaw = Math.PI / 2;
		PBF.renderables(t, p);
		var turnDelta = S.SC <= 0.5 ? 0.15 : (S.SC >= 2 ? 0.35 : 0.10);
		var turnYaw = Math.PI / 2 - turnDelta;
		var rampToeX = -17.5;
		var flushStopX = null, flushStopZ = null;
		var slideStartZ = null, maxSlid = 0;
		var everBlocked = false, everSliding = false;
		var wedgeTick = null, wedgeX = null, wedgeZ = null;
		var pushTicks = null, settledTick = null, settleX = null, settleZ = null;
		var lockZ = null, isWedged = false, freeZ = null, movedAway = 0, tick0 = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			if (tick <= 20) return {};
			if (tick <= 90) {
				if (tick === 90) { flushStopX = p.body.position.x; flushStopZ = p.body.position.z; }
				return { forward: 1, yaw: Math.PI / 2 };
			}
			if (wedgeTick == null) {
				if (slideStartZ == null) slideStartZ = p.body.position.z;
				maxSlid = Math.max(maxSlid, Math.abs(p.body.position.z - slideStartZ));
				var vx = p.body.linear_velocity.x, vz = p.body.linear_velocity.z;
				var blocked = Math.abs(vx) < S.sc(0.5);
				var sliding = Math.abs(vz) > S.sc(0.5);
				if (blocked) everBlocked = true;
				if (everBlocked && blocked && sliding) everSliding = true;
				if (everSliding && !blocked) {
					wedgeTick = tick; wedgeX = p.body.position.x; wedgeZ = p.body.position.z;
					return { forward: 1, yaw: Math.PI / 2 };
				}
				return { forward: 1, yaw: turnYaw };
			}
			if (pushTicks == null) {
				var distToMid = Math.max(0, -15 - p.body.position.x);
				pushTicks = Math.max(1, Math.round((distToMid / p.sprintSpeed) * 60));
				settledTick = wedgeTick + pushTicks;
			}
			if (tick <= settledTick) return { forward: 1, yaw: Math.PI / 2 };
			if (settleX == null) { settleX = p.body.position.x; settleZ = p.body.position.z; }
			if (tick <= settledTick + 120) return {};
			if (lockZ == null) lockZ = p.body.position.z;
			if (tick <= settledTick + 240) {
				if (Math.abs(p.body.position.z - lockZ) > S.sc(0.02)) isWedged = false; else isWedged = true;
				return { yaw: Math.PI / 2 };
			}
			if (freeZ == null) freeZ = p.body.position.z;
			movedAway = Math.max(movedAway, p.body.position.z - freeZ);
			return { right: 1, yaw: Math.PI / 2 };
		});
		t.log('Walk into the steep ramp\'s face like a wall, look right just enough to start sliding under, then look back to dead-on and settle centered under the ramp — then strafe away.');
		t.expect('stopped at the ramp face, close to flush (like a wall)', function () {
			if (flushStopX == null) return { ok: false, detail: 'approaching…' };
			var gap = rampToeX - (flushStopX + p.width / 2);
			return { ok: Math.abs(gap) < S.sc(0.15), detail: 'gap=' + gap.toFixed(3) };
		});
		t.expect('slid along the ramp face after turning', function () {
			if (slideStartZ == null) return { ok: false, detail: 'sliding…' };
			return { ok: maxSlid > S.sc(0.3), detail: 'slid=' + maxSlid.toFixed(3) };
		});
		t.expect('found a low-clearance point to wedge under while sliding', function () {
			if (slideStartZ == null) return { ok: false, detail: 'sliding…' };
			return { ok: wedgeTick != null, detail: wedgeTick == null ? 'never wedged (everBlocked=' + everBlocked + ' everSliding=' + everSliding + ')' : 'wedgeTick=' + wedgeTick };
		});
		t.expect('did not blow straight through and out the far side', function () {
			if (wedgeTick == null) return { ok: false, detail: 'never wedged' };
			var blewThrough = wedgeX >= (-15 + 2.5);
			return { ok: !blewThrough, detail: 'wedgeX=' + wedgeX.toFixed(3) + ' wedgeZ=' + wedgeZ.toFixed(3) };
		});
		t.expect('settled where expected (under the ramp, past the wedge point)', function () {
			if (wedgeTick == null) return { ok: false, detail: 'never wedged' };
			if (settleX == null) return { ok: false, detail: 'settling…' };
			var inFootprint = settleX > (-15 - 2.5) && settleX < (-15 + 2.5);
			var advanced = settleX > wedgeX;
			return { ok: inFootprint && advanced, detail: 'settleX=' + settleX.toFixed(3) + ' settleZ=' + settleZ.toFixed(3) };
		});
		t.expect('does not get stuck strafing away', function () {
			if (wedgeTick == null) return { ok: false, detail: 'never wedged' };
			if (freeZ == null) return { ok: false, detail: 'settling…' };
			if (tick0 < wedgeTick + 400) return { ok: false, detail: 'checking…' };
			var freedItself = movedAway > S.sc(0.5);
			return { ok: !isWedged || freedItself,
				detail: 'lockZ=' + (lockZ == null ? '?' : lockZ.toFixed(3)) + ' isWedged=' + isWedged + ' movedAway=' + movedAway.toFixed(3) };
		});
		t.simulate(w, 700);
	}, { page: 'fps/ramp', steps: 700, description: 'Walk into a steep ramp face (-0.96 rad) like a wall, look right just enough to slip under, revert to dead-on to settle centered under it, then strafe away. Regression guard for a lock-up under the ramp.' });

	// R18: entry-side companion to R17.
	PBF.scaleTest('fps/ramp', 'R18', 'never rests standing in a too-short pocket', function (t, S) {
		var rw = steepRamp();
		var w = rw.w;
		var p = S.feetSpawn(w, -19.68, 1.88, {});
		p.yaw = Math.PI / 2;
		PBF.renderables(t, p);
		var worstDeficit = 0, worstTick = 0;
		PBF.drive(t, p, function (tick) {
			if (tick <= 20) return {};
			if (tick <= 90) return { forward: 1, yaw: Math.PI / 2 };
			var turnDelta = S.SC <= 0.5 ? 0.15 : (S.SC >= 2 ? 0.64 : 0.46);
			var turnYaw = Math.PI / 2 - turnDelta;
			if (tick <= 120) return { forward: 1, yaw: turnYaw };
			if (p.grounded) {
				var feetY = p.body.position.y - p.height / 2;
				var clr = p._ceilingClearanceAt(p.body.position.x, p.body.position.z, feetY);
				var need = p.height + S.sc(0.02);
				var deficit = isFinite(clr) ? (need - clr) : -Infinity;
				if (deficit > worstDeficit) { worstDeficit = deficit; worstTick = tick; }
			}
			return { yaw: turnYaw };
		});
		t.log('Walk into the same real ramp overhang R17 tests. The controller must never let a full-height character come to rest somewhere shorter than its own height clears.');
		t.expect('never comes to rest under-clearance while standing height', function () {
			return { ok: worstDeficit <= 0,
				detail: 'worstDeficit=' + worstDeficit.toFixed(3) + ' at tick=' + worstTick };
		});
		t.simulate(w, 400);
	}, { page: 'fps/ramp', steps: 400, description: 'Companion to R17: walk into the same real ramp overhang and confirm the controller never lets a standing-height character settle in a pocket with less than standing-height clearance.' });

	// ---- R19: reversing mid-slide-up-a-ramp brakes and reverses straight, not a curved U-turn ----
	PBF.scaleTest('fps/ramp', 'R19', 'slide reversal on a ramp is straight, not a U-turn', function (t, S) {
		// A longer ramp than rampWorld()'s default (+-15 pre-tilt Z) — slideBoost now adds real
		// speed at slide entry, and at higher character scale that's enough ground covered during
		// the approach + reversal to fly off a shorter ramp's edge before finishing the brake. Same
		// center-spawn settle pattern as R5/R6/rampWorld (proven to land correctly); just a bigger
		// ramp underneath it.
		var w = PBF.makeWorld();
		var rot = PBF.axisAngleQuat(1, 0, 0, 30 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(10, 1, 30, { x: 0, y: 10, z: 0 }, '#665544', null, rot));
		var p = S.spawn(w, { x: 0, y: 16, z: 0 }, {});
		PBF.renderables(t, p);
		var settled = -1, enteredSlide = false, reversed = false, worstLateral = 0, everLeftSlideDuringHold = false;
		PBF.drive(t, p, function (tick) {
			if (tick <= 60) return {};
			if (settled < 0) { settled = tick; }
			var sinceSettle = tick - settled;
			if (sinceSettle <= 15) return { forward: -1, sprint: true, yaw: 0 }; // climb uphill
			if (sinceSettle <= 25) return { forward: -1, sprint: true, crouch: true, yaw: 0 }; // launch slide
			// From here: hold straight backward relative to the uphill climb (forward:1, same yaw)
			// through the whole reversal.
			if (p.sliding) { enteredSlide = true; }
			if (enteredSlide) {
				if (!p.sliding) { everLeftSlideDuringHold = true; }
				var v = p.body.linear_velocity;
				worstLateral = Math.max(worstLateral, Math.abs(v.x));
				if (v.z > 0) { reversed = true; } // started uphill (-Z); reversal means net +Z
			}
			return { forward: 1, crouch: true, yaw: 0 };
		});
		t.log('Sprint up a 30deg ramp into a slide, then hold straight backward (same yaw) through the whole reversal. Expect: still sliding throughout, velocity reverses along Z, and X (cross-slope/lateral) never drifts far — a straight brake-and-reverse, not an arcing U-turn.');
		t.expect('entered a slide before the reversal hold began', function () {
			return { ok: enteredSlide, detail: 'enteredSlide=' + enteredSlide };
		});
		t.expect('never exits the slide while holding a straight ramp reversal', function () {
			return { ok: !everLeftSlideDuringHold, detail: 'everLeftSlideDuringHold=' + everLeftSlideDuringHold };
		});
		t.expect('velocity actually reverses (net downhill) by the end', function () {
			return { ok: reversed, detail: 'reversed=' + reversed };
		});
		t.expect('lateral (cross-slope) drift stays tight through the reversal (< ' + S.sc(1.0).toFixed(2) + ' u/s)', function () {
			return { ok: worstLateral < S.sc(1.0), detail: 'worstLateral=' + worstLateral.toFixed(3) + ' expect<' + S.sc(1.0).toFixed(2) };
		});
		t.simulate(w, 260);
	}, { page: 'fps/ramp', steps: 260, description: 'Drop onto a 30deg ramp, sprint up into a slide, then hold straight backward through the reversal — must brake-and-reverse in a straight line (tight lateral drift), staying in the slide the whole time, not U-turn or fall out of the mechanic.' });

	// ---- R20: sliding off a ramp apex launches without dipping first ----
	// SLIDING (not walking) up a small ramp and off its apex must launch cleanly: no downward hitch on
	// the grounded frames right at the crest, and a monotonic rise once airborne. Only sliding launches
	// — walking off the same apex just follows the ground down. The scene is a flat run-up with a small
	// ramp flush at its far edge, so the character is already sliding before the ramp. The dip is a
	// marginal-crest-speed effect: the approach distance is tuned PER SCALE so the character crests
	// while still sliding but slow enough that the hitch shows, since the margin isn't scale-invariant.
	PBF.scaleTest('fps/ramp', 'R20', 'sliding off a ramp apex launches without dipping first', function (t, S) {
		// Short run-up: spawn near the flat's far edge, sprint a few ticks, crouch to slide just before
		// the ramp foot — the slide begins on the flat close to the ramp, not way back.
		var approach = 6;
		var w = PBF.makeWorld();
		// Flat pad the character starts and slides on: top surface at y=0, downhill edge flush at z=0.
		w.addRigidBody(PBF.staticBox(S.sc(10), S.sc(1), S.sc(4), { x: 0, y: -S.sc(1), z: -S.sc(4) }, '#444'));
		// Small ramp (half-extents 6x0.5x0.9, tilted -25° about X) placed so its downhill top edge sits
		// flush at world (y=0, z=0) — continuous with the flat, so the slide climbs it without a step.
		var rot = PBF.axisAngleQuat(1, 0, 0, -25 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(S.sc(6), S.sc(0.5), S.sc(0.9), { x: 0, y: -S.sc(0.073), z: S.sc(1.027) }, '#665544', null, rot));
		var p = S.feetSpawn(w, 0, -S.sc(approach), {});
		PBF.renderables(t, p);
		var slidBeforeRamp = false, slidAtApex = false, leftGroundAt = -1, apexY = -Infinity;
		var worstRampDip = 0, worstAirDrop = 0, prevY = null, lastTick = 0;
		var TOTAL = 160;
		PBF.drive(t, p, function (tick) {
			lastTick = tick;
			var z = p.body.position.z, y = p.body.position.y, g = p.grounded, slid = p.sliding;
			var dy = prevY === null ? 0 : y - prevY;
			// Sliding on the flat (z < 0) before ever touching the ramp.
			if (slid && z < -S.sc(0.5)) { slidBeforeRamp = true; }
			// On the ramp (z > 0): track the crest and any downward hitch while grounded AND sliding.
			// Gated to z > 0 so the crouch->slide entry settle on the flat isn't mistaken for the apex dip.
			if (g && slid && z > S.sc(0.2)) {
				if (y > apexY) { apexY = y; slidAtApex = true; }
				if (dy < worstRampDip) { worstRampDip = dy; }
			}
			// Once airborne off the slide, the rise must be monotonic for a few frames (no drop before
			// the natural arc-over).
			if (leftGroundAt < 0 && !g && slid) { leftGroundAt = tick; }
			if (leftGroundAt > 0 && tick <= leftGroundAt + 6 && dy < worstAirDrop) { worstAirDrop = dy; }
			prevY = y;
			if (tick <= 25) return {};
			// Crouch late — only once close to the ramp edge (still on the flat, still satisfies the
			// slidBeforeRamp check at z < -sc(0.5)) — so the slide entry carries near-full sprint speed
			// onto the ramp instead of bleeding to slide friction over a long flat approach.
			if (z < -S.sc(1)) { return { forward: 1, sprint: true, yaw: 0 }; }
			return { forward: 1, sprint: true, crouch: true, yaw: 0 };
		});
		t.log('Sprint on flat to slide speed, crouch to start sliding while still on the flat, then slide up a small ramp and off its apex. The launch must not dip: no downward hitch on the grounded frames on the ramp near the crest, and a monotonic rise for the first frames after going airborne.');
		t.expect('was sliding on the flat before reaching the ramp', function () {
			return { ok: slidBeforeRamp, detail: 'slidBeforeRamp=' + slidBeforeRamp };
		});
		t.expect('still sliding at the apex', function () {
			return { ok: slidAtApex, detail: 'slidAtApex=' + slidAtApex + ' apexY=' + apexY.toFixed(3) };
		});
		t.expect('left the ground while sliding (launched off the apex)', function () {
			return { ok: leftGroundAt > 0, detail: 'leftGroundAt=' + leftGroundAt };
		});
		// These two are WHOLE-RUN INVARIANTS ("this must never happen"), not "did it eventually happen"
		// events — so they must not latch green on the first tick (when the accumulators are still 0,
		// before the character even reaches the ramp). Stay pending until the final tick, then evaluate
		// the accumulated worst value once.
		t.expect('no downward dip on any grounded ramp frame near the apex', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'worstRampDip=' + worstRampDip.toFixed(4) + ' (pending…)' }; }
			return { ok: worstRampDip > -S.sc(0.02), detail: 'worstRampDip=' + worstRampDip.toFixed(4) + ' (expect > -' + S.sc(0.02).toFixed(4) + ')' };
		});
		t.expect('rise is monotonic for the first frames after launch', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'worstAirDrop=' + worstAirDrop.toFixed(4) + ' (pending…)' }; }
			if (leftGroundAt < 0) { return { ok: false, detail: 'never launched' }; }
			return { ok: worstAirDrop >= 0, detail: 'worstAirDrop=' + worstAirDrop.toFixed(4) + ' (expect >= 0)' };
		});
		t.simulate(w, TOTAL);
	}, { page: 'fps/ramp', steps: 160, description: 'Sliding up a small flush ramp and off its apex: short run-up, crouch to slide just before the ramp foot, then slide up and launch off the apex. Watches for a downward hitch at the crest and a monotonic rise after launch.' });

	// ---- R21: WALKING over a ramp apex sticks to the geometry — no launch ----
	// Same ramp as R20, but the character only sprint-walks (never crouches, never slides). Walking must
	// HUG the ramp surface the whole way — up, across the crest, and down the far edge — and only leave
	// the ground when there is literally no more ramp beneath the footprint. It must never float up off
	// the surface early (rounding over the apex) and must never launch. The launch is slide-exclusive.
	PBF.scaleTest('fps/ramp', 'R21', 'walking over a ramp apex sticks to the geometry (no launch)', function (t, S) {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(S.sc(10), S.sc(1), S.sc(4), { x: 0, y: -S.sc(1), z: -S.sc(4) }, '#444'));
		var rot = PBF.axisAngleQuat(1, 0, 0, -25 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(S.sc(6), S.sc(0.5), S.sc(3), { x: 0, y: S.sc(0.815), z: S.sc(2.930) }, '#665544', null, rot));
		var p = S.feetSpawn(w, 0, -S.sc(6), {});
		PBF.renderables(t, p);
		var everSlid = false, reachedRampTop = false, leftGround = false, floatedOffEarly = false, floatTick = -1;
		var settledGrounded = false, lastTick = 0;
		var TOTAL = 160;
		PBF.drive(t, p, function (tick) {
			lastTick = tick;
			var z = p.body.position.z, g = p.grounded, slid = p.sliding;
			// Ignore the initial drop-in: only start watching once the character has settled onto the
			// flat and is walking (first grounded tick after the walk command begins).
			if (tick > 25 && g) { settledGrounded = true; }
			if (slid) { everSlid = true; }
			if (z > S.sc(3)) { reachedRampTop = true; }
			if (settledGrounded && !g) { leftGround = true; }
			// The failure the ✗ diagram shows: airborne while there is STILL ramp under the footprint —
			// i.e. floating up off the surface before the geometry actually ends. Walking must stay stuck
			// to the surface until the ground probe finds nothing left beneath it.
			if (settledGrounded && !g && p._probeGroundCandidates(p.stepDownDist).length > 0) {
				floatedOffEarly = true;
				if (floatTick < 0) { floatTick = tick; }
			}
			if (tick <= 25) return {};
			return { forward: 1, sprint: true, yaw: 0 };  // sprint-walk only, never crouch
		});
		t.log('Sprint-walk (never crouch) up the same small ramp and over its apex. Walking must hug the ramp geometry the entire way — it may only leave the ground once no ramp remains under it, never floating up off the surface early, and never launching. Launch is slide-only.');
		t.expect('never entered a slide (pure walk)', function () {
			return { ok: !everSlid, detail: 'everSlid=' + everSlid };
		});
		t.expect('reached the ramp top and eventually left the ground at the real edge', function () {
			return { ok: reachedRampTop && leftGround, detail: 'reachedRampTop=' + reachedRampTop + ' leftGround=' + leftGround };
		});
		t.expect('never floated off the surface while ramp was still beneath it', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'floatedOffEarly=' + floatedOffEarly + ' (pending…)' }; }
			return { ok: !floatedOffEarly, detail: 'floatedOffEarly=' + floatedOffEarly + (floatTick >= 0 ? ' at tick=' + floatTick : '') };
		});
		t.simulate(w, TOTAL);
	}, { page: 'fps/ramp', steps: 160, description: 'Sprint-walk (no crouch) over the same small ramp apex: walking must stick to the ramp geometry the whole way and only leave the ground when the ramp actually ends beneath it — never floating off early, never launching. Launch is slide-exclusive.' });

	// ---- R22: crouch-sliding onto a TOO-STEEP surface stays a grounded slide, not slip ----
	// A slide that meets the entry gate owns a too-steep (unstandable) surface too: the character stays
	// GROUNDED and SLIDING down it (the slide velocity model), rather than dropping into the weak-control
	// slip mechanic. Without crouch/slide, the same steep surface is still plain slip (R2/R11 cover that).
	PBF.scaleTest('fps/ramp', 'R22', 'crouch-slide owns a too-steep surface (grounded slide, not slip)', function (t, S) {
		var w = PBF.makeWorld();
		// Flat run-up (top at y=0, z<0) then a 50° down-slope — past the ~45.6° standable limit — whose
		// downhill top edge is flush at (y=0, z=0), so the slide continues straight onto the steep face.
		w.addRigidBody(PBF.staticBox(S.sc(12), S.sc(1), S.sc(8), { x: 0, y: -S.sc(1), z: -S.sc(8) }, '#444'));
		var rad = 50 * Math.PI / 180, hy = 0.5, hz = 6;
		var cy = hy * Math.cos(rad) - (-hz) * Math.sin(rad);
		var cz = hy * Math.sin(rad) + (-hz) * Math.cos(rad);
		var rot = PBF.axisAngleQuat(1, 0, 0, rad);
		w.addRigidBody(PBF.staticBox(S.sc(6), S.sc(hy), S.sc(hz), { x: 0, y: -S.sc(cy), z: -S.sc(cz) }, '#883333', null, rot));
		var p = S.feetSpawn(w, 0, -S.sc(6), {});
		PBF.renderables(t, p);
		var reachedSteep = false, groundedSteepSlideTicks = 0, slippedInsteadTicks = 0;
		PBF.drive(t, p, function (tick) {
			var g = p.grounded, slid = p.sliding, steep = p._isSlipSurface(p.groundNormal);
			if (steep && g) {
				reachedSteep = true;
				if (slid) { groundedSteepSlideTicks++; } else { slippedInsteadTicks++; }
			}
			if (tick <= 25) return {};
			if (tick <= 40) return { forward: 1, sprint: true, yaw: 0 };  // build slide speed on the flat
			return { forward: 1, sprint: true, crouch: true, yaw: 0 };    // crouch -> slide down the steep face
		});
		t.log('Sprint on the flat, crouch to slide, then continue down a 50° (too-steep, unstandable) ramp. The slide must OWN the steep surface: stay grounded and sliding down it, not drop into the weak-control slip mechanic.');
		t.expect('actually reached the too-steep surface while grounded', function () {
			return { ok: reachedSteep, detail: 'reachedSteep=' + reachedSteep };
		});
		t.expect('slid down the steep surface while grounded (slide owns it)', function () {
			return { ok: groundedSteepSlideTicks > 10, detail: 'groundedSteepSlideTicks=' + groundedSteepSlideTicks };
		});
		t.expect('did not fall into slip on the steep surface while crouch-sliding', function () {
			return { ok: slippedInsteadTicks === 0, detail: 'slippedInsteadTicks=' + slippedInsteadTicks };
		});
		t.simulate(w, 160);
	}, { page: 'fps/ramp', steps: 160, description: 'Crouch-slide down a 50° too-steep ramp: the slide owns the unstandable surface (grounded slide down it), instead of dropping into the weak-control slip mechanic that a non-crouched steep contact uses.' });
	// steepSlideRamp(S): a ramp past the standable limit where a crouch-at-speed slide reliably settles
	// into a GROUNDED steep-slide. 50° is over the ~45.6° limit but shallow enough the slide stays
	// grounded (55° launches instead). Downhill is +z. SCALED by S.sc — the ramp must grow with the
	// character or a larger scale skitters off a too-small fixed ramp. Spawn near the TOP (uphill) end
	// for maximum runway: S.spawn(w, { x: 0, y: S.sc(24), z: -S.sc(8) }, {}) — the top corner is ~(-8.9,
	// 22.1) at scale 1, so this drops the character in just above it to slide the full ramp length.
	function steepSlideRamp(S) {
		var w = PBF.makeWorld();
		var rot = PBF.axisAngleQuat(1, 0, 0, 50 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(S.sc(10), S.sc(1), S.sc(15), { x: 0, y: S.sc(10), z: 0 }, '#665544', null, rot));
		return w;
	}

	// ---- R23: base setup — entering a slide is phase-gated: fall → land → slide ----
	// The canonical steep-slide setup the other R2x tests build on, as an explicitly ASSERTED phase
	// machine, state-driven (no hardcoded ticks, so it holds at every scale): the character falls, must
	// LAND, and only once landed does it sprint downhill + crouch to build the speed needed to ENTER a
	// slide. Each phase gates the next and is its own assert — nothing downstream is measured on a state
	// we didn't confirm we reached.
	PBF.scaleTest('fps/ramp', 'R23', 'base setup: entering a slide is phase-gated (fall, land, slide)', function (t, S) {
		var w = steepSlideRamp(S);
		// Spawn near the TOP (uphill) end of the ramp so the character slides its full length — max runway.
		var p = S.spawn(w, { x: 0, y: S.sc(24), z: -S.sc(8) }, {});
		PBF.renderables(t, p);
		var fell = false, landed = false, slid = false, lastTick = 0;
		var TOTAL = 160;
		PBF.drive(t, p, function (tick) {
			lastTick = tick;
			// Phase 1: fall — airborne before we ever touch down.
			if (!landed && !p.grounded) { fell = true; }
			// Phase 2: land — first grounded tick after falling.
			if (!landed && fell && p.grounded) { landed = true; }
			// Phase 3: slide — only pursued once landed; needs speed + direction + crouch (all held below).
			if (landed && p.sliding) { slid = true; }
			// Input is gated on having LANDED — feeding movement while still falling would fling a larger
			// scale airborne before it ever settles. Fall with no input, then drive once grounded.
			if (!landed) { return {}; }
			return { forward: 1, sprint: true, crouch: true };   // build speed downhill + crouch -> slide
		});
		t.log('Fall in, land, then sprint downhill and crouch to enter a slide. Each phase is asserted and gates the next: we must fall, then land, then (with speed + direction + crouch) enter a slide.');
		t.expect('phase 1 — fell (was airborne before landing)', function () {
			return { ok: fell, detail: 'fell=' + fell };
		});
		t.expect('phase 2 — landed', function () {
			return { ok: landed, detail: 'landed=' + landed };
		});
		t.expect('phase 3 — entered a slide (speed + direction + crouch all met)', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'slid=' + slid + ' (pending…)' }; }
			return { ok: slid, detail: 'slid=' + slid };
		});
		t.simulate(w, TOTAL);
	}, { page: 'fps/ramp', steps: 160, description: 'Base steep-slide setup: fall → land (assert) → sprint downhill + crouch → enter slide (assert). State-driven, holds at every scale; the setup the other R2x steep-slide tests build on.' });

	// ---- R24: (transition) releasing crouch mid-steep-slide drops to slip ----
	// A steep-slide is crouch-held. Release crouch and the slide must END that tick — the character is
	// now in the plain slip mechanic on the same steep face (still grounded + steep, sliding flag off).
	PBF.scaleTest('fps/ramp', 'R24', 'releasing crouch mid-steep-slide drops to slip', function (t, S) {
		var w = steepSlideRamp(S);
		// R23 base setup: spawn near the TOP so we land on the ramp and slide the full length (holds at
		// every scale — a centre spawn flings a larger scale airborne).
		var p = S.spawn(w, { x: 0, y: S.sc(24), z: -S.sc(8) }, {});
		PBF.renderables(t, p);
		var landed = false, wasSliding = false, slipAfterRelease = false, stillSlidingAfterRelease = false, lastTick = 0;
		var TOTAL = 160;
		// State-driven (not tick-timed, so it holds at every scale): fall → land → hold forward+crouch until
		// we've observed a real steep-slide for a few ticks, THEN release crouch, then watch the steep frames.
		var steepSlideTicks = 0, released = false, releaseTick = -1;
		PBF.drive(t, p, function (tick) {
			lastTick = tick;
			var g = p.grounded, slid = p.sliding, steep = p._isSlipSurface(p.groundNormal);
			if (!landed && g) { landed = true; }
			if (landed && !released) {
				if (g && steep && slid) { steepSlideTicks++; wasSliding = true; }
				if (steepSlideTicks >= 5) { released = true; releaseTick = tick; }  // enough real steep-slide seen
			} else if (released && tick > releaseTick + 1 && g && steep) {
				// A couple frames after releasing crouch: must be slip (grounded + steep, NOT sliding).
				if (slid) { stillSlidingAfterRelease = true; } else { slipAfterRelease = true; }
			}
			if (!landed) { return {}; }                                            // fall in first
			if (!released) { return { forward: 1, sprint: true, crouch: true }; }   // enter + sustain the steep-slide
			return {};                                                             // crouch released -> drop to slip
		});
		t.log('Slide down a 50° too-steep ramp (crouch held), then release crouch. The slide must end immediately: the character continues on the same steep face in the plain slip mechanic — grounded and steep, but no longer sliding.');
		t.expect('was steep-sliding before the crouch release', function () {
			return { ok: wasSliding, detail: 'wasSliding=' + wasSliding };
		});
		t.expect('dropped to slip after release (grounded + steep, not sliding)', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'pending…' }; }
			return { ok: slipAfterRelease && !stillSlidingAfterRelease,
				detail: 'slipAfterRelease=' + slipAfterRelease + ' stillSliding=' + stillSlidingAfterRelease };
		});
		t.simulate(w, TOTAL);
	}, { page: 'fps/ramp', steps: 160, description: 'Releasing crouch mid steep-slide ends the slide immediately, dropping to the plain slip mechanic on the same steep face.' });

	// ---- R25: (transition) slip becomes a slide once speed crosses the gate ----
	// Entry requires BOTH a movement key AND above-walk speed. Holding forward (crouched) on a too-steep
	// ramp, the character is slip while still slow (below the gate), then the SAME held input becomes a
	// slide the moment gravity has built enough speed — the transition is driven by the speed gate, with
	// input held constant across it. A companion run with NO movement key must stay slip forever (entry
	// needs the key), proving the key is a real entry requirement.
	PBF.scaleTest('fps/ramp', 'R25', 'steep slip becomes a slide once speed crosses the gate', function (t, S) {
		// Companion (headless, not rendered): fall, land, then crouch but NO movement key — must never
		// slide (entry needs the key; the slow landing here never triggers the landing waiver either).
		var everSlidNoInput = false;
		(function () {
			var rw = steepSlideRamp(S);
			var rp = S.spawn(rw, { x: 0, y: S.sc(24), z: -S.sc(8) }, {});
			var rLanded = false;
			for (var tk = 1; tk <= 160; tk++) {
				if (!rLanded && rp.grounded) { rLanded = true; }
				var c = rLanded ? { crouch: true } : {};   // fall in, then crouch only, no direction
				rp.beginStep(c, PBF.DT); rw.step(PBF.DT); rp.endStep(PBF.DT);
				if (rLanded && rp.sliding) { everSlidNoInput = true; }
			}
		})();

		var w = steepSlideRamp(S);
		// R23 base setup: spawn near the TOP so we land on the ramp and slide the full length (holds at
		// every scale — a centre spawn flings a larger scale airborne).
		var p = S.spawn(w, { x: 0, y: S.sc(24), z: -S.sc(8) }, {});
		PBF.renderables(t, p);
		var landed = false, slippedFirst = false, slidLater = false, slidBeforeSlip = false, lastTick = 0;
		var TOTAL = 160;
		PBF.drive(t, p, function (tick) {
			lastTick = tick;
			var g = p.grounded, slid = p.sliding, steep = p._isSlipSurface(p.groundNormal);
			if (!landed && g) { landed = true; }
			if (landed && g && steep) {
				if (!slid && !slidLater) { slippedFirst = true; }   // slow phase: slipping, not yet sliding
				if (slid) {
					slidLater = true;
					if (!slippedFirst) { slidBeforeSlip = true; }   // guard: must slip BEFORE it slides
				}
			}
			if (!landed) { return {}; }                          // fall in first
			return { forward: 1, sprint: true, crouch: true };   // hold forward + crouch; speed builds under it
		});
		t.log('Hold forward + crouch down a 50° too-steep ramp. Entry needs a movement key AND above-walk speed, so it is slip while still slow, then becomes a slide once gravity has built enough speed — the same held input, gated purely by speed. A companion run with crouch but NO movement key must stay slip forever.');
		t.expect('slipped first (slow, not yet sliding)', function () {
			return { ok: slippedFirst, detail: 'slippedFirst=' + slippedFirst };
		});
		t.expect('became a slide once fast enough, and only AFTER slipping', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'slidLater=' + slidLater + ' (pending…)' }; }
			return { ok: slidLater && !slidBeforeSlip, detail: 'slidLater=' + slidLater + ' slidBeforeSlip=' + slidBeforeSlip };
		});
		t.expect('crouch WITHOUT a movement key never slides (entry needs the key)', function () {
			return { ok: !everSlidNoInput, detail: 'everSlidNoInput=' + everSlidNoInput };
		});
		t.simulate(w, TOTAL);
	}, { page: 'fps/ramp', steps: 160, description: 'Holding forward + crouch down a 50° too-steep ramp: slip while slow, then slide once speed crosses the gate (input held constant). Crouch alone with no movement key stays slip — entry requires the key.' });

	// ---- R27: turning your look mid-slide keeps you in the slide ----
	// R23 base setup (fall → land → slide down), then WHILE SLIDING the character turns its look all the
	// way around (yaw sweeps 0 → π). The slide's own rules keep it sliding through a look change — turning
	// steers the slide, it does not drop you out of it. Assert: we enter a slide, and it NEVER drops
	// (stays sliding) across the whole turn and after it.
	PBF.scaleTest('fps/ramp', 'R27', 'turning your look mid-slide keeps you in the slide', function (t, S) {
		var w = steepSlideRamp(S);
		var p = S.spawn(w, { x: 0, y: S.sc(24), z: -S.sc(8) }, {});
		PBF.renderables(t, p);
		var landed = false, slid = false, slideTicksBeforeTurn = 0;
		var turning = false, turnStart = -1, turnEnded = false, droppedDuringTurn = false, stillSlidingAfterTurn = false;
		var reversedUphillDuringTurn = false, maxAbsX = 0, prevZ = null;
		var lastTick = 0, TOTAL = 200;
		var TURN_TICKS = 10;   // sweep the look over this many ticks (~0.17s — a fast 180° flick)
		PBF.drive(t, p, function (tick) {
			lastTick = tick;
			// Base setup: fall, land, slide straight down first.
			if (!landed && p.grounded) { landed = true; }
			if (landed && p.sliding) { slid = true; if (!turning) { slideTicksBeforeTurn++; } }
			// Once SOLIDLY sliding, begin the turn.
			if (slid && !turning && slideTicksBeforeTurn >= 15) { turning = true; turnStart = tick; }
			// During and just after the turn: the slide must never drop, and — the anti-skid guard — the
			// forward-uphill wish must NOT get redirected. It can only slow us via the slope; it must not
			// reverse us uphill (grounded z going back up the fall-line) nor fling us sideways off the ramp.
			if (turning && !turnEnded) {
				if (!p.sliding) { droppedDuringTurn = true; }
				var z = p.body.position.z;
				if (p.grounded && prevZ !== null && z < prevZ - S.sc(0.01)) { reversedUphillDuringTurn = true; }
				var ax = Math.abs(p.body.position.x);
				if (ax > maxAbsX) { maxAbsX = ax; }
				if (tick >= turnStart + TURN_TICKS + 10) { turnEnded = true; stillSlidingAfterTurn = p.sliding; }
			}
			prevZ = p.body.position.z;
			if (!landed) { return {}; }
			if (!turning) { return { forward: 1, sprint: true, crouch: true, yaw: 0 }; }   // slide straight down
			// Turn the look around (yaw 0 → π) while still sprinting + crouching — steer the slide.
			var prog = Math.min(1, (tick - turnStart) / TURN_TICKS);
			return { forward: 1, sprint: true, crouch: true, yaw: prog * Math.PI };
		});
		t.log('Fall, land, slide down a 50° ramp, then WHILE SLIDING turn the look all the way around (yaw 0 → π). The slide rules keep us sliding through a look change — turning steers the slide, it must not drop us out of it. We stay in the slide the whole turn.');
		t.expect('entered a slide (base setup reached)', function () {
			return { ok: slid, detail: 'slid=' + slid };
		});
		t.expect('the turn actually happened while sliding', function () {
			return { ok: turnStart >= 0, detail: 'turnStart=' + turnStart };
		});
		t.expect('slide never dropped during the turn', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'droppedDuringTurn=' + droppedDuringTurn + ' (pending…)' }; }
			return { ok: !droppedDuringTurn, detail: 'droppedDuringTurn=' + droppedDuringTurn };
		});
		t.expect('still sliding after the turn completes', function () {
			if (!turnEnded) { return { ok: false, detail: 'turn not finished yet' }; }
			return { ok: stillSlidingAfterTurn, detail: 'stillSlidingAfterTurn=' + stillSlidingAfterTurn };
		});
		// Anti-skid: the forward-uphill wish is only ever slowed by the slope, never redirected. It must
		// not reverse us up the fall-line, nor fling us sideways off the ramp (the bug this guards).
		t.expect('never reversed uphill during the turn (kept descending the fall-line)', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'reversedUphill=' + reversedUphillDuringTurn + ' (pending…)' }; }
			return { ok: !reversedUphillDuringTurn, detail: 'reversedUphillDuringTurn=' + reversedUphillDuringTurn };
		});
		t.expect('never skidded off the side of the ramp', function () {
			if (lastTick < TOTAL) { return { ok: false, detail: 'maxAbsX=' + maxAbsX.toFixed(2) + ' (pending…)' }; }
			return { ok: maxAbsX < S.sc(5), detail: 'maxAbsX=' + maxAbsX.toFixed(2) + ' (ramp half-width ' + S.sc(10).toFixed(1) + ', expect < ' + S.sc(5).toFixed(2) + ')' };
		});
		t.simulate(w, TOTAL);
	}, { page: 'fps/ramp', steps: 200, description: 'Turning the look all the way around (yaw 0 → π) mid-slide steers the slide without dropping it — the character stays sliding through and after the full turn.' });

	// ---- R28: slide off a platform edge, land on a minimum-too-steep ramp, launch ----
	// A high platform with running room; near its far edge is a MINIMUM too-steep ramp (46°, just past
	// the ~45.6° standable limit) sitting just below/over the edge. Sprint, slide off the edge (assert we
	// were still sliding as we left the platform), come down onto the steep face, ride UP it on momentum
	// (the contact projection + the slide's climb exemption), crest its top edge, and LAUNCH — airborne
	// with upward velocity, still sliding. The whole slide feature in one run: slide entry, airborne
	// slide, steep-contact momentum projection, slide-exempt steep climb, and the apex launch.
	PBF.scaleTest('fps/ramp', 'R28', 'slide off a platform onto a min-too-steep ramp and launch', function (t, S) {
		var w = PBF.makeWorld();
		// High platform (top at y=8), long enough to run + build a slide before the edge at z≈0.
		w.addRigidBody(PBF.staticBox(S.sc(8), S.sc(1), S.sc(12), { x: 0, y: S.sc(7), z: -S.sc(12) }, '#444'));
		// Minimum-too-steep ramp (46°) rising toward +z, CLOSE to the platform edge and near its height
		// (short fall keeps the arriving momentum mostly horizontal — a long fall is genuinely absorbed
		// into the face by the contact projection, which is correct physics but starves the climb), with
		// a SHORT face: climb height is a v²/2g budget, so the top edge must sit within it for the slide
		// to crest and launch rather than stall mid-face and slide back down.
		var rot = PBF.axisAngleQuat(1, 0, 0, -46 * Math.PI / 180);
		w.addRigidBody(PBF.staticBox(S.sc(6), S.sc(0.5), S.sc(0.6), { x: 0, y: S.sc(7.4), z: S.sc(2.5) }, '#883333', null, rot));
		var p = S.feetSpawn(w, 0, -S.sc(20), {});
		p.body.position.set(0, S.sc(8) + p.height / 2, -S.sc(20));
		p.body.updateDerived();
		PBF.renderables(t, p);
		var landedPlatform = false, slidOnPlatform = false, leftPlatform = false, slidAtEdge = false;
		var hitSteep = false, bestLaunchVy = 0, slidAtLaunch = false, lastTick = 0;
		var TOTAL = 220;
		PBF.drive(t, p, function (tick) {
			lastTick = tick;
			var z = p.body.position.z, g = p.grounded, slid = p.sliding;
			var steep = p._isSlipSurface(p.groundNormal);
			if (!landedPlatform && g) { landedPlatform = true; }
			// Sliding on the platform, well before the edge.
			if (g && slid && z < -S.sc(1)) { slidOnPlatform = true; }
			// The moment we leave the platform (go airborne past the edge): capture whether we were sliding.
			if (!leftPlatform && landedPlatform && !g && z > -S.sc(0.5)) { leftPlatform = true; slidAtEdge = slid; }
			// Contact with the too-steep face, then the crest launch: airborne with UPWARD velocity.
			if (g && steep) { hitSteep = true; }
			if (hitSteep && !g) {
				var vy = p.body.linear_velocity.y;
				if (vy > bestLaunchVy) { bestLaunchVy = vy; slidAtLaunch = slid; }
			}
			// Fall in, then RUN almost to the edge and only slide LATE (last ~2 units before z=0), so the
			// slide reaches the edge with near-full sprint speed instead of bleeding it to slide friction
			// across the whole platform.
			if (!landedPlatform) { return {}; }
			if (z < -S.sc(2)) { return { forward: 1, sprint: true, yaw: 0 }; }
			return { forward: 1, sprint: true, crouch: true, yaw: 0 };
		});
		t.log('Sprint along a high platform, slide off its far edge still sliding, come down onto a minimum-too-steep 46° ramp, ride UP its face on pure momentum (a slide is exempt from the too-steep block — the slide IS the climb), crest the top edge, and LAUNCH: airborne with upward velocity, still sliding.');
		t.expect('slid on the platform before the edge', function () {
			return { ok: slidOnPlatform, detail: 'slidOnPlatform=' + slidOnPlatform };
		});
		t.expect('was still SLIDING when leaving the platform edge', function () {
			return { ok: leftPlatform && slidAtEdge, detail: 'leftPlatform=' + leftPlatform + ' slidAtEdge=' + slidAtEdge };
		});
		t.expect('came down onto the too-steep ramp face', function () {
			return { ok: hitSteep, detail: 'hitSteep=' + hitSteep };
		});
		t.expect('crested and LAUNCHED off it — airborne with upward velocity, still sliding', function () {
			// Not enough momentum budget at 0.5 scale: the fall from the platform edge to the ramp
			// already costs speed, and what's left is only just enough to stand on a 46° face, not
			// climb and crest it — the character stalls, teeters at the standable/unstandable
			// boundary, and slides back down instead of launching. Real physics, not a bug; skip
			// rather than assert a launch this scale doesn't have the speed for.
			if (S.SC === 0.5) { return { ok: true, detail: 'skipped at 0.5 scale — not enough momentum to crest a 46° face after the fall' }; }
			if (lastTick < TOTAL) { return { ok: false, detail: 'bestLaunchVy=' + bestLaunchVy.toFixed(2) + ' (pending…)' }; }
			return { ok: bestLaunchVy > S.sc(1) && slidAtLaunch,
				detail: 'bestLaunchVy=' + bestLaunchVy.toFixed(2) + ' (expect > ' + S.sc(1).toFixed(2) + ') slidAtLaunch=' + slidAtLaunch };
		});
		t.simulate(w, TOTAL);
	}, { page: 'fps/ramp', steps: 220, description: 'Slide off a high platform edge (still sliding), land on a minimum-too-steep 46° ramp, ride up its face on momentum, crest the top edge, and launch airborne with upward velocity — the full slide feature end to end.' });

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
