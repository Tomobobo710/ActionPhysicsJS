/**
 * Tom's Suite — FPSCharacterController PLATFORM tests (PL1-PL3).
 *
 * One controller per test, all 3 scales. Exercises the base-velocity mechanism (endStep acquires a
 * platform's linear_velocity when the ground probe lands on an isPlatform body; beginStep adds it into
 * the swept move; _updateVertical adds its vertical component into a jump ADDITIVELY, so jumping off a
 * rising platform flings you higher).
 */
(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	var Vector3 = ActionPhysics.Vector3;
	var G = 'fps/platform', P = 'fps/platform';

	// ---- PL1: ride an elevator through a FULL up+down cycle, staying planted, no jitter ----
	PBF.scaleTest(G, 'PL1', 'ride a vertical elevator through a full up+down cycle, no jitter', function (t, S) {
		var w = S.flat();
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 5, z: 0 }, 2.5, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, {});
		PBF.renderables(t, p, [mover]);

		var groundedFlips = 0, lastGrounded = null, maxGap = 0, settledTick = 3;
		var sawUp = false, sawDown = false;
		var peakY = -Infinity, minYAfterPeak = Infinity;
		var everUngroundedWhilePlatformMoving = false;

		PBF.drive(t, p, function (tick) {
			var settled = tick > settledTick;
			if (settled) {
				if (lastGrounded !== null && p.grounded !== lastGrounded) groundedFlips++;
				lastGrounded = p.grounded;

				if (!p.grounded) everUngroundedWhilePlatformMoving = true;
				else {
					var platTopY = mover.position.y + (0.3 * S.SC) / 2;
					var feetY = p.body.position.y - p.height / 2;
					var gap = Math.abs(feetY - platTopY);
					if (gap > maxGap) maxGap = gap;
				}
			}

			if (mover.linear_velocity.y < 0) sawDown = true; else sawUp = true;
			if (p.body.position.y > peakY) peakY = p.body.position.y;
			if (sawDown && p.body.position.y < minYAfterPeak) minYAfterPeak = p.body.position.y;

			return {}; // no input — pure ride, isolates the base-velocity mechanism
		}, [mover]);

		t.log('Stand still on an elevator through a FULL ascent + descent (the mover reverses mid-test) — ' +
			'must stay grounded and tracking the platform the whole time, with no grounded-state flicker ' +
			'and no growing gap between feet and platform surface.');
		t.expect('never went airborne while riding (stayed planted)', function () {
			return { ok: !everUngroundedWhilePlatformMoving, detail: 'everUngrounded=' + everUngroundedWhilePlatformMoving };
		});
		t.expect('no grounded-state flicker (0 flips while riding)', function () {
			return { ok: groundedFlips === 0, detail: 'groundedFlips=' + groundedFlips };
		});
		t.expect('feet-to-platform gap stayed tight the whole ride (no jitter)', function () {
			return { ok: maxGap < S.sc(0.05), detail: 'maxGap=' + maxGap.toFixed(4) };
		});
		t.expect('rode both legs — ascent then descent — not just one direction', function () {
			return { ok: sawUp && sawDown, detail: 'sawUp=' + sawUp + ' sawDown=' + sawDown };
		});
		t.expect('actually came back down after the peak (real descent, not stuck at the top)', function () {
			return { ok: minYAfterPeak < peakY - S.sc(1.0), detail: 'peakY=' + peakY.toFixed(2) + ' minYAfterPeak=' + minYAfterPeak.toFixed(2) };
		});
		t.simulate(w, 260);
	}, { page: P, steps: 260, description: 'Riding an elevator through a full up+down cycle stays grounded the whole time with no jitter.' });

	// ---- PL2: genuinely ride a horizontal platform across a gap ----
	PBF.scaleTest(G, 'PL2', 'ride a horizontal moving platform across a gap and dismount on the far side', function (t, S) {
		var w = S.flat();
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0, z: -3 }, { x: 0, y: 0, z: 3 }, 1.2, '#4a7ab0');
		var platY = mover.position.y + (0.3 * S.SC) / 2;
		var platformFrontZ = mover.position.z - S.sc(1);
		var p = S.spawn(w, { x: 0, y: platY + 0.9 * S.SC + 0.001, z: platformFrontZ - S.sc(2) }, {});
		PBF.renderables(t, p, [mover]);

		var onPlatform = function () {
			var cands = p._probeGroundCandidates(p.stepDownDist);
			for (var i = 0; i < cands.length; i++) if (cands[i].object && cands[i].object.isPlatform) return true;
			return false;
		};

		var boardedAt = -1, stoppedWalkingAt = -1, everFellOffMidRide = false, reachedFarSideWhileAboard = false;
		var farSideZ = S.sc(3); // the platform's OTHER endpoint (matches {x:0,y:0,z:3} above)
		PBF.drive(t, p, function (tick) {
			var standing = p.grounded && onPlatform();
			if (standing && boardedAt < 0) boardedAt = tick;
			// Keep walking until actually near the platform's OWN center in Z, not a fixed tick count —
			// walk speed varies with scale, so a fixed tick budget can (and did) carry the player clean
			// across the platform's whole 2-unit width and off the far edge before ever settling aboard.
			var nearMiddle = standing && Math.abs(p.body.position.z - mover.position.z) < S.sc(0.3);
			if (nearMiddle && stoppedWalkingAt < 0) stoppedWalkingAt = tick;
			if (boardedAt >= 0) {
				if (!standing && p.grounded && !reachedFarSideWhileAboard && p.body.position.z < farSideZ - S.sc(0.2)) {
					everFellOffMidRide = true;
				}
				if (standing && p.body.position.z >= farSideZ - S.sc(0.2)) reachedFarSideWhileAboard = true;
			}
			return { forward: stoppedWalkingAt < 0 ? 1 : 0, yaw: 0 };
		}, [mover]);

		t.log('Walk onto a horizontal platform, then stand still — the base-velocity mechanism (not the ' +
			'player\'s own momentum) must carry the rider all the way to the platform\'s far endpoint ' +
			'while still aboard, never sliding off partway through the crossing.');
		t.expect('boarded the platform', function () {
			return { ok: boardedAt > 0, detail: 'boardedAt=' + boardedAt };
		});
		// A genuine walk-on should take roughly as long as covering the spawn-to-platform approach
		// distance at moveSpeed — a few dozen ticks, not hundreds. A test that only checks "eventually
		// boards" within a huge tick budget can hide a real bug behind a technical pass.
		t.expect('boarded within a reasonable time (not a multi-second chase/stall)', function () {
			if (boardedAt < 0) return { ok: false, detail: 'not boarded yet' };
			return { ok: boardedAt <= 60, detail: 'boardedAt=' + boardedAt + ' (must be <= 60 ticks / 1s)' };
		});
		t.expect('settled near the platform\'s middle (didn\'t walk clean over it)', function () {
			return { ok: stoppedWalkingAt > 0, detail: 'stoppedWalkingAt=' + stoppedWalkingAt };
		});
		t.expect('did not fall off mid-ride (no friction-drag-then-drop)', function () {
			return { ok: !everFellOffMidRide, detail: 'everFellOffMidRide=' + everFellOffMidRide };
		});
		t.expect('rode all the way to the far side while still aboard', function () {
			return { ok: reachedFarSideWhileAboard, detail: 'reachedFarSideWhileAboard=' + reachedFarSideWhileAboard + ' finalZ=' + p.body.position.z.toFixed(2) + ' farSideZ=' + farSideZ.toFixed(2) };
		});
		t.simulate(w, 500);
	}, { page: P, steps: 500, description: 'Walking onto a horizontal platform and standing still, the base-velocity mechanism carries the rider all the way to the far side without falling off partway.' });

	// ---- PL3: ride an elevator from the bottom, no jitter, jump near the top, fling higher than normal ----
	PBF.scaleTest(G, 'PL3', 'ride from the bottom, jump near the top, and fling higher than a normal jump', function (t, S) {
		var w = S.flat();
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 8, z: 0 }, 6, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, {});
		PBF.renderables(t, p, [mover]);

		// The theoretical peak rise of the SAME jump (jumpSpeed) from a dead stop — v^2 = 2*g*h —
		// computed analytically rather than run in a second world, so this stays a single,
		// live-drivable, watchable world (one elevator, one ride, one jump, one fling).
		var g = -w.gravity.y;
		var normalJumpRise = (p.jumpSpeed * p.jumpSpeed) / (2 * g);

		var groundedFlips = 0, lastGrounded = null, maxGap = 0, settledTick = 6;
		// Jump near the very END of the ascent (8 units at 6 u/s = ~80 ticks to the top), not mid-ride —
		// that's the moment the fling is most dramatic and most clearly distinguishable from "just
		// riding," since the platform is about to stop climbing right after.
		var jumpTick = 72, wasGroundedBeforeJump = false, jumpedAt = -1;
		var jumpVelYAtJump = null, jumpPeakY = -Infinity, jumpStartY = null;
		var landedAfterFling = false;
		PBF.drive(t, p, function (tick) {
			if (tick === settledTick + 1) lastGrounded = p.grounded;
			if (tick > settledTick) {
				if (p.grounded !== lastGrounded) groundedFlips++;
				lastGrounded = p.grounded;
				if (p.grounded) {
					var platTopY = mover.position.y + (0.3 * S.SC) / 2;
					var feetY = p.body.position.y - p.height / 2;
					var gap = Math.abs(feetY - platTopY);
					if (gap > maxGap) maxGap = gap;
				}
			}

			if (tick === jumpTick) {
				wasGroundedBeforeJump = p.grounded;
				jumpVelYAtJump = mover.linear_velocity.y;
				jumpStartY = p.body.position.y;
			}
			if (tick === jumpTick && wasGroundedBeforeJump) jumpedAt = jumpTick;
			if (jumpedAt > 0 && p.body.position.y > jumpPeakY) jumpPeakY = p.body.position.y;
			if (jumpedAt > 0 && jumpPeakY > jumpStartY && p.grounded && p.body.position.y < jumpPeakY) {
				landedAfterFling = true;
			}

			return { jumpPressed: tick === jumpTick };
		}, [mover]);

		t.log('Ride the elevator up from the bottom (must stay planted with no jitter), then jump near the ' +
			'very END of the ascent — the jump should reach measurably higher than jumpSpeed alone would ' +
			'produce from a dead stop, because the platform\'s rising base velocity is added additively ' +
			'into the jump, not overwritten by it — then fall back and land.');
		t.expect('rode the elevator with no grounded-state flicker before the jump', function () {
			return { ok: groundedFlips === 0, detail: 'groundedFlips=' + groundedFlips };
		});
		t.expect('feet-to-platform gap stayed tight while riding (no jitter)', function () {
			return { ok: maxGap < S.sc(0.05), detail: 'maxGap=' + maxGap.toFixed(4) };
		});
		t.expect('jumped while still riding partway up', function () {
			return { ok: jumpedAt > 0, detail: 'jumpedAt=' + jumpedAt };
		});
		t.expect('platform was genuinely rising at the moment of the jump', function () {
			if (jumpVelYAtJump == null) return { ok: false, detail: 'settling…' };
			return { ok: jumpVelYAtJump > S.sc(0.5), detail: 'jumpVelYAtJump=' + jumpVelYAtJump.toFixed(3) };
		});
		t.expect('the fling rose measurably higher than a normal jump from a dead stop would', function () {
			if (jumpStartY == null) return { ok: false, detail: 'settling…' };
			var actualRise = jumpPeakY - jumpStartY;
			return {
				ok: actualRise > normalJumpRise + S.sc(0.15),
				detail: 'actualRise=' + actualRise.toFixed(3) + ' normalJumpRise=' + normalJumpRise.toFixed(3)
			};
		});
		t.expect('fell back down and landed after the fling', function () {
			return { ok: landedAfterFling, detail: 'landedAfterFling=' + landedAfterFling };
		});
		t.simulate(w, 340);
	}, { page: P, steps: 340, description: 'Riding an elevator from the bottom with no jitter, jumping near the top of the ascent flings the player measurably higher than a normal jump, then they fall back and land.' });

	// ---- PL4: sprint-slide down a lane, cross a platform heading the other way, keep going normally ----
	PBF.scaleTest(G, 'PL4', 'sliding across an oncoming platform behaves normally, not a boost pad', function (t, S) {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(S.sc(15), S.sc(1), S.sc(30), { x: 0, y: -S.sc(1), z: S.sc(10) }, '#333'));
		// Platform starts at the FAR end of the lane and travels toward the player (who starts at the
		// near end), flush with the floor, so a sprint+slide down the lane meets it head-on and
		// crosses over it — matching the real in-game scenario directly. Thickness (0.3, pre-scale)
		// matches game.js's actual shuttle platform — it must stay under the controller's stepHeight
		// (0.4 pre-scale) so the character steps up onto it instead of clipping/phasing at its edge.
		var mover = S.splatform(w, 4, 0.3, { x: 0, y: 0.15, z: 15 }, { x: 0, y: 0.15, z: -15 }, 3, '#4a7ab0');
		var p = S.feetSpawn(w, 0, -S.sc(10), {});
		PBF.renderables(t, p, [mover]);

		// t.expect(label, fn) is the ONLY mechanism that works both headless AND live in the browser
		// viewer — the live render loop (render.js) drives ticks itself and re-evaluates every
		// t.expect() predicate each tick via ctx.evalTick; code placed after t.simulate() in the test
		// body runs exactly once, synchronously, at t=0, before the browser ever steps a single tick
		// (see render.js's captureSetup, which stubs out simulate() and runs the rest of the test
		// function immediately) — a plain t.checkTrue() there always sees the untouched initial state
		// in the live viewer even though it works fine headless. So: accumulate state every tick in
		// PBF.drive's per-tick callback as before, but each PREDICATE below only resolves (ok:true or
		// ok:false) once the run has reached its final tick — before that it stays pending — so it
		// still gets ONE check against the FULLY accumulated result, the same as before, just via the
		// mechanism that also works live.
		var enteredSlide = false, slidingAtBoard = false, boardedAt = -1, leftAt = -1,
			ownAtBoard = -1, prevOwn = -1, grewAfterBoardTick = false, worstGrowth = 0,
			stalledAfterBoard = false, minPzAdvanceAfterBoard = Infinity, prevPz = null;
		var TOTAL_TICKS = 220, curTick = 0;
		PBF.drive(t, p, function (tick) {
			curTick = tick;
			if (p.sliding) { enteredSlide = true; }
			var onPlatform = p._baseVelocity.z !== 0 || p._baseVelocity.x !== 0;
			if (onPlatform && boardedAt < 0) { boardedAt = tick; slidingAtBoard = p.sliding; }
			if (!onPlatform && boardedAt >= 0 && leftAt < 0) { leftAt = tick; }
			var standing = p.grounded && p.sliding && onPlatform;
			if (standing) {
				var own = Math.hypot(p._ownVelocityX, p._ownVelocityZ);
				if (ownAtBoard < 0) { ownAtBoard = own; }
				else if (prevOwn >= 0 && own > prevOwn) {
					grewAfterBoardTick = true;
					if (own - prevOwn > worstGrowth) worstGrowth = own - prevOwn;
				}
				prevOwn = own;
			}
			// "Slid over, not stopped dead": WHILE ABOARD (between boardedAt and leftAt only — not for
			// the rest of the run after dismount, where ordinary end-of-slide friction decay also
			// slows the per-tick advance and would otherwise be misread as a stall), the player must
			// keep making forward progress every tick, not flatline in place (the boost-pad bug's
			// stall symptom). Measured directly: broken crawls ~0.01-0.11 units/tick while aboard;
			// correctly-fixed advances ~0.14-0.17+ units/tick throughout the crossing.
			if (boardedAt >= 0 && leftAt < 0 && prevPz !== null) {
				var advance = p.body.position.z - prevPz;
				if (advance < minPzAdvanceAfterBoard) { minPzAdvanceAfterBoard = advance; }
				if (advance <= S.sc(0.06)) { stalledAfterBoard = true; }
			}
			prevPz = p.body.position.z;
			// Sprint long enough to actually close most of the distance to the platform before
			// crouching — triggering the slide right away (as tick<=20 did) launches it at the start
			// line, so it's fully decayed well before ever reaching the platform. Real play sprints
			// most of the approach, then slides in at the last moment.
			if (tick <= 45) { return { forward: 1, sprint: true, yaw: 0 }; }
			return { forward: 1, sprint: true, crouch: true, yaw: 0 };
		}, [mover]);

		t.log('Sprint then crouch-slide straight down a lane; a platform travels the same lane toward the player and they cross over it. Expect: nothing special happens — own speed keeps decaying from friction the whole time, same as sliding on plain ground, and the player slides all the way OVER the platform rather than stalling dead partway across. Any tick where own speed goes UP instead of down while aboard is the platform\'s motion leaking into the character\'s own momentum — the boost-pad bug.');
		t.expect('entered a slide at some point', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'enteredSlide=' + enteredSlide + ' (pending…)' }; }
			return { ok: enteredSlide, detail: 'enteredSlide=' + enteredSlide };
		});
		t.expect('was still sliding when it reached/boarded the platform', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'boardedAt=' + boardedAt + ' (pending…)' }; }
			return { ok: boardedAt > 0 && slidingAtBoard, detail: 'boardedAt=' + boardedAt + ' slidingAtBoard=' + slidingAtBoard };
		});
		t.expect('slid all the way over the platform, never stalled dead', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'boardedAt=' + boardedAt + ' leftAt=' + leftAt + ' (pending…)' }; }
			return { ok: boardedAt > 0 && leftAt > boardedAt && !stalledAfterBoard,
				detail: 'boardedAt=' + boardedAt + ' leftAt=' + leftAt + ' stalledAfterBoard=' + stalledAfterBoard +
					' minPzAdvanceAfterBoard=' + (isFinite(minPzAdvanceAfterBoard) ? minPzAdvanceAfterBoard.toFixed(4) : 'n/a') };
		});
		t.expect('own speed never grows while aboard, no boost-pad compounding', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'worstGrowth=' + worstGrowth.toFixed(3) + ' (pending…)' }; }
			return { ok: !grewAfterBoardTick, detail: 'worstGrowth=' + worstGrowth.toFixed(3) + ' ownAtBoard=' + ownAtBoard.toFixed(2) };
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 220, description: 'Sprint-sliding down a lane while a platform travels toward the player down the same lane — crossing it must feel like normal ground, not a boost pad.' });

	// ---- PL5: sprint-slide down a lane, catch a platform heading the SAME way — must NOT boost ----
	PBF.scaleTest(G, 'PL5', 'catching an outrunning platform from behind must not launch the player faster', function (t, S) {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(S.sc(15), S.sc(1), S.sc(30), { x: 0, y: -S.sc(1), z: S.sc(10) }, '#333'));
		// Platform starts just ahead of the player and travels AWAY down the same lane the player is
		// sprinting/sliding — the player catches up from behind and boards while it's still moving away.
		// The bug: touching the platform launches the player to a MUCH higher speed than boarding a
		// platform moving at PLATFORM_SPEED should ever produce. Boarding a platform genuinely moving
		// at PLATFORM_SPEED SHOULD add roughly PLATFORM_SPEED to the rider's ground-frame total speed —
		// that's just riding a moving walkway (verified directly: the pre-fix "pass" here was masking a
		// SEPARATE endStep bug — see FPSCharacterController's outgoingBaseVelocityX/Z fix — that
		// silently cancelled part of this legitimate addition instead of correctly applying it; fixing
		// that bug correctly RAISED the measured spike to ~= PLATFORM_SPEED, which is right). What must
		// NOT happen is a launch far beyond that — multiple times PLATFORM_SPEED, or unbounded growth.
		var PLATFORM_SPEED = 3;
		var mover = S.splatform(w, 4, 0.3, { x: 0, y: 0.15, z: 5 }, { x: 0, y: 0.15, z: 25 }, PLATFORM_SPEED, '#4a7ab0');
		var p = S.feetSpawn(w, 0, -S.sc(10), {});
		PBF.renderables(t, p, [mover]);

		var boardedAt = -1, speedBeforeBoard = -1, worstSpikeAfterBoard = 0;
		var TOTAL_TICKS = 220, curTick = 0;
		PBF.drive(t, p, function (tick) {
			curTick = tick;
			var onPlatform = p._baseVelocity.z !== 0 || p._baseVelocity.x !== 0;
			var total = Math.hypot(p.body.linear_velocity.x, p.body.linear_velocity.z);
			if (!onPlatform) { speedBeforeBoard = total; }
			if (onPlatform) {
				if (boardedAt < 0) { boardedAt = tick; }
				var spike = total - speedBeforeBoard;
				if (spike > worstSpikeAfterBoard) { worstSpikeAfterBoard = spike; }
			}
			if (tick <= 45) { return { forward: 1, sprint: true, yaw: 0 }; }
			return { forward: 1, sprint: true, crouch: true, yaw: 0 };
		}, [mover]);

		t.log('Sprint then crouch-slide down a lane, catching a platform from behind that\'s moving away down the same lane. The bug: touching the platform hauls the player to a much higher speed than they already had. Expect: boarding must NOT launch the player faster — speed while aboard stays close to whatever speed they already had going in.');
		t.expect('boarded the platform from behind', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'boardedAt=' + boardedAt + ' (pending…)' }; }
			return { ok: boardedAt > 0, detail: 'boardedAt=' + boardedAt };
		});
		t.expect('touching the platform did not launch the player far beyond the platform\'s own speed', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'worstSpikeAfterBoard=' + worstSpikeAfterBoard.toFixed(2) + ' (pending…)' }; }
			// Threshold is PLATFORM_SPEED (scaled) + a margin — boarding legitimately adds ~PLATFORM_SPEED
			// to the rider's total speed; a real launch bug blows well past that, not by a hair.
			var limit = S.sc(PLATFORM_SPEED) + S.sc(1);
			return { ok: worstSpikeAfterBoard <= limit, detail: 'speedBeforeBoard=' + speedBeforeBoard.toFixed(2) +
				' worstSpikeAfterBoard=' + worstSpikeAfterBoard.toFixed(2) + ' limit=' + limit.toFixed(2) };
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 220, description: 'Sprint-sliding down a lane and catching a platform from behind, moving away down the same lane — boarding it must not launch the player to a higher speed.' });

	// ---- PL6/PL7 shared rig: a long rotating platform (helicopter-blade style), stationary at mount
	// time so boarding isn't itself the thing under test — only isRotatingPlatform's omega x r
	// tangential-velocity contribution (see endStep's acquire in FPSCharacterController) is. Spin a full
	// 360 deg one way, then a full 360 deg the other way, and check the RIDER (not the platform, which
	// trivially returns to its own start orientation after 360) ends up back near its start position —
	// exactly what "carried in a circle and set back down" should look like — while sweeping a real arc
	// during the spin (radius-proportional to distance from the pivot; near-zero at the pivot itself).
	// A rider tracing a circle of radius riderOffsetX must stay over SOLID platform footprint at every
	// angle through the spin, not just at the start — a narrow/long footprint (e.g. a thin arm) sweeps
	// its edge out from under an off-axis rider well before a full revolution completes, dropping them
	// onto the floor mid-spin (verified directly: an 8x2 arm drops a radius-3.5 rider around tick 90).
	// A SQUARE footprint whose half-side covers the rider's radius (the INSCRIBED circle, not just the
	// corner-to-corner diagonal) keeps solid surface under the rider at every angle instead.
	var ROT_RATE = 0.3; // rad/s — fast enough to actually watch, slow enough the ground probe/step-clamp keeps tracking a rotating footprint every tick (verified: 1.2 rad/s intermittently drops the rider)
	var PLATFORM_HALF_SIDE = 5; // base (scale-1) units — must exceed the far-end rider's radius (3.5) with margin
	// Builds the scene and installs the tick-driver, but does NOT call t.simulate() — the runner's
	// contract (see runner.js's expect/simulate) requires every t.expect() to be registered BEFORE
	// simulate() runs (simulate() evaluates expectations live, tick by tick, then calls failUnmet() on
	// whatever's still pending the moment it returns — an expect() registered AFTER simulate() has
	// already run just sits at status:'pending' forever and is silently NOT counted as a failure). So
	// this returns { state, run(t) }: state is the live (mutated in place) result object the caller's
	// t.expect() predicates read from, and run(t) is what actually calls t.simulate() — the caller must
	// register every expect() against `state` first, THEN call run(t).
	function riderRotationRig(S, riderOffsetX, rotRate) {
		var rate = rotRate !== undefined ? rotRate : ROT_RATE;
		var w = S.flat();
		// Square platform centered at the world origin, wide enough that a rider anywhere within
		// PLATFORM_HALF_SIDE of the pivot stays over solid footprint through the ENTIRE spin, not just at
		// the start — riderOffsetX places the character somewhere along the X axis (in BASE units, scaled
		// internally like every other coordinate here) before any spin starts.
		var mover = S.srotplatform(w, PLATFORM_HALF_SIDE * 2, PLATFORM_HALF_SIDE * 2, 0.3, { x: 0, y: 0.15, z: 0 }, 0, '#4a7ab0');
		var platY = mover.position.y + (0.3 * S.SC) / 2;
		var startX = riderOffsetX * S.SC;
		var p = S.spawn(w, { x: startX, y: platY + 0.9 * S.SC + 0.001, z: 0 }, {});

		var mountSettleTicks = 15; // stand still first so the character is genuinely resting before spin starts
		var spinOneRevTicks = Math.round((2 * Math.PI / rate) / (1 / 60));
		var spinBackStart = mountSettleTicks + spinOneRevTicks;
		var TOTAL_TICKS = spinBackStart + spinOneRevTicks + 20; // +20 settle after the return spin

		// `state` is mutated in place tick by tick — the caller's t.expect() predicates close over THIS
		// object (not a snapshot), so they see live values as the sim runs, matching every other live
		// t.expect() pattern in this suite (e.g. PL4/PL5's curTick-guarded pending checks).
		var state = {
			curTick: 0, totalTicks: TOTAL_TICKS,
			boardedBeforeSpin: false, everUngroundedWhileSpinning: false,
			maxRadiusSeen: 0, minRadiusSeen: Infinity, totalAngleSwept: 0, everSweptWrongWay: false,
			startPos: null, endPos: null, expectedRadius: Math.abs(startX)
		};
		var angleAccum = 0, lastAngle = null; // UNSIGNED accumulated distance travelled (see below) — a
		// signed running total would net out to ~0 by construction once the return spin cancels the
		// outbound spin, which is exactly the "frozen the whole time" bug this test exists to catch —
		// so it can't discriminate a real 360-out-360-back ride from never having moved at all.
		// GROUND-TRUTH direction reference: a point rigidly fixed to the platform's own local frame at
		// (1,0,0) — its WORLD position (via the engine's own rotation.transformVectorInto, not a hand-derived
		// sign assumption) defines which way "the platform's actual spin direction" sweeps, independent of
		// any convention assumed elsewhere. The rider's own angular delta must always agree in sign with
		// this material point's angular delta.
		var matRef = new Vector3(1, 0, 0), lastMatAngle = null, wrongStreak = 0;

		return {
			state: state,
			run: function (t) {
				PBF.renderables(t, p, [mover]);
				PBF.drive(t, p, function (tick) {
					state.curTick = tick;
					if (tick === mountSettleTicks) {
						state.boardedBeforeSpin = p.grounded && p._baseVelocity.x === 0 && p._baseVelocity.z === 0;
						state.startPos = { x: p.body.position.x, z: p.body.position.z };
					}
					// Reassigned ONE TICK BEFORE the tick it's meant to take effect on: PBF.drive's onTick
					// runs movers.forEach(m => m.tick(dt)) BEFORE this callback (see _util_fps.js's drive()),
					// so a reassignment made ON tick N only takes effect starting tick N+1's movers pass —
					// verified directly (an earlier version reassigned AT mountSettleTicks+1 and measured
					// the platform's own angular_velocity still reading 0 on that exact tick, costing a full
					// dead tick of spin-up that's invisible as a fraction of PL6's slow 1257-tick revolution
					// but shows up as an ~11deg "never returns to start" residual at PL7's fast 157-tick one).
					if (tick === mountSettleTicks) { mover.tick = function (dt) { mover.angular_velocity.set(0, rate, 0); }; }
					if (tick === spinBackStart - 1) { mover.tick = function (dt) { mover.angular_velocity.set(0, -rate, 0); }; }
					if (tick === spinBackStart + spinOneRevTicks - 1) { mover.tick = function (dt) { mover.angular_velocity.set(0, 0, 0); }; }
					if (tick > mountSettleTicks && tick <= spinBackStart + spinOneRevTicks) {
						if (!p.grounded) { state.everUngroundedWhileSpinning = true; }
						var rx = p.body.position.x - mover.position.x, rz = p.body.position.z - mover.position.z;
						var radius = Math.hypot(rx, rz);
						if (radius > state.maxRadiusSeen) { state.maxRadiusSeen = radius; }
						if (radius < state.minRadiusSeen) { state.minRadiusSeen = radius; }
						// Only accumulate angle once there's an actual radius to have an angle about — right
						// at the pivot (radius ~0) atan2 is numerically meaningless and must not add noise.
						// A rider held FIXED at radius r never leaves that radius either — maxRadiusSeen
						// alone can't distinguish "swept a real arc" from "sat frozen while the platform
						// spun under them" (verified directly: disabling the rotational contribution
						// entirely left maxRadiusSeen unchanged, since the rider's OWN starting radius
						// already equals the expected one). ANGLE actually swept is the real discriminator.
						if (radius > S.sc(0.1)) {
							var angle = Math.atan2(rz, rx);
							if (lastAngle !== null) {
								var d = angle - lastAngle;
								while (d > Math.PI) { d -= 2 * Math.PI; }
								while (d < -Math.PI) { d += 2 * Math.PI; }
								angleAccum += Math.abs(d);
								state.totalAngleSwept = angleAccum;
								// DIRECTION check: compare the rider's own angular delta this tick against a
								// GROUND-TRUTH material point rigidly attached to the platform (transformed by
								// the platform's OWN rotation quaternion via the engine's own transformVectorInto —
								// not a hand-derived "which sign should this be" assumption, which is exactly
								// what got this wrong the first time: the engine's rotation convention for a given
								// angular_velocity.y sign turned out to be the OPPOSITE of the standard
								// right-hand-rule assumption used when first deriving the omega x r formula).
								// A sign flip anywhere in the controller's omega x r math shows up as the
								// rider's delta and the material point's delta disagreeing in sign — caught
								// live: an earlier version swept the rider backwards relative to the platform's
								// actual spin, while every other check here (radius, angle MAGNITUDE,
								// return-to-start) still passed, since none of them look at direction.
								var matWorld = new Vector3();
								mover.rotation.transformVectorInto(matRef, matWorld);
								var matAngle = Math.atan2(matWorld.z, matWorld.x);
								if (lastMatAngle !== null) {
									var md = matAngle - lastMatAngle;
									while (md > Math.PI) { md -= 2 * Math.PI; }
									while (md < -Math.PI) { md += 2 * Math.PI; }
									// A short disagreement right at the moment the platform's spin REVERSES is
									// expected transient lag (the rider's own residual velocity from the old
									// direction hasn't fully washed out yet the instant angular_velocity flips
									// sign) — verified directly: a 1-tick blip at ROT_RATE, a 7-tick blip at 8x
									// ROT_RATE (proportionally longer at higher speed, but still a brief cluster
									// right at the reversal, not sustained). A REAL sign-flip bug disagrees on
									// EVERY tick of the spin, not a handful at one boundary — 10 is well clear of
									// the observed transient at 8x speed while still catching a sustained bug
									// orders of magnitude faster.
									if (Math.abs(md) > 1e-6 && Math.abs(d) > 1e-6 && (d > 0) !== (md > 0)) {
										wrongStreak++;
										if (wrongStreak > 10) { state.everSweptWrongWay = true; }
									} else {
										wrongStreak = 0;
									}
								}
								lastMatAngle = matAngle;
							}
							lastAngle = angle;
						}
					}
					if (tick === TOTAL_TICKS) { state.endPos = { x: p.body.position.x, z: p.body.position.z }; }
					return {}; // no input — pure ride, isolates the rotational base-velocity mechanism
				}, [mover]);
				t.simulate(w, TOTAL_TICKS);
			}
		};
	}

	// ---- PL6: character at the FAR END of a long rotating platform — must be swept in a real arc ----
	PBF.scaleTest(G, 'PL6', 'riding the far end of a rotating platform sweeps a full-radius arc, returns after 360+360', function (t, S) {
		var rig = riderRotationRig(S, 3.5); // near the platform's own end (half-length 4, pre-scale)
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };

		t.log('Stand still at the far end of a long platform (not yet spinning) so mounting is settled, then ' +
			'spin it a full 360 one way and a full 360 back the other way. A rider out near the end must be ' +
			'swept through a real arc whose radius matches their distance from the pivot — not stand frozen ' +
			'in world space while the platform turns under them — and end up back close to their start ' +
			'position once both spins complete.');
		// Every predicate below gates on `pending()` (curTick < totalTicks) before reading final-shape
		// results, matching PL4/PL5's live-expectation pattern — t.expect() predicates run EVERY tick
		// against the LIVE (mutated in place) rig.state, so a naive check would flip true prematurely on
		// an early tick's partial data (e.g. maxRadiusSeen looks "in range" after just 1 tick).
		t.expect('boarded and settled before the spin started', function () {
			if (pending() && st.startPos === null) { return { ok: false, detail: 'settling…' }; }
			return { ok: st.boardedBeforeSpin, detail: 'boardedBeforeSpin=' + st.boardedBeforeSpin };
		});
		t.expect('never went airborne while riding the spin', function () {
			if (pending()) { return { ok: false, detail: 'everUngrounded=' + st.everUngroundedWhileSpinning + ' (pending…)' }; }
			return { ok: !st.everUngroundedWhileSpinning, detail: 'everUngrounded=' + st.everUngroundedWhileSpinning };
		});
		t.expect('held a steady radius near the platform\'s own arm length (not flung off or dragged in)', function () {
			if (pending()) { return { ok: false, detail: 'maxR=' + st.maxRadiusSeen.toFixed(2) + ' (pending…)' }; }
			var ok = st.maxRadiusSeen > st.expectedRadius * 0.8 && st.maxRadiusSeen < st.expectedRadius * 1.2
				&& st.minRadiusSeen > st.expectedRadius * 0.8;
			return { ok: ok, detail: 'minRadiusSeen=' + st.minRadiusSeen.toFixed(3) + ' maxRadiusSeen=' + st.maxRadiusSeen.toFixed(3) + ' expectedRadius=' + st.expectedRadius.toFixed(3) };
		});
		// The real discriminator: a rider frozen in place at the same radius the whole time (the bug this
		// test exists to catch — a no-op rotational contribution) sweeps ~0 total angle even though its
		// radius never changes (verified directly by disabling the mechanism: maxRadiusSeen stayed
		// unchanged at the rider's own starting radius, so radius ALONE can't catch this). A genuinely
		// carried rider sweeps ~2*2*PI (a full revolution out, a full revolution back) minus whatever
		// integration/collision-clip error the mechanism has.
		t.expect('was actually carried around — swept close to a full 360 out and 360 back (not frozen)', function () {
			if (pending()) { return { ok: false, detail: 'totalAngleSwept=' + st.totalAngleSwept.toFixed(2) + ' (pending…)' }; }
			var expected = 2 * (2 * Math.PI); // one rev out + one rev back
			var ok = st.totalAngleSwept > expected * 0.7;
			return { ok: ok, detail: 'totalAngleSwept=' + st.totalAngleSwept.toFixed(3) + ' rad (expected ~' + expected.toFixed(3) + ')' };
		});
		t.expect('back near the start position after a full 360 + full 360 back', function () {
			if (pending()) { return { ok: false, detail: 'settling…' }; }
			var sp = st.startPos, ep = st.endPos;
			var d = Math.hypot(ep.x - sp.x, ep.z - sp.z);
			return { ok: d < S.sc(0.5), detail: 'drift=' + d.toFixed(3) + ' start=(' + sp.x.toFixed(2) + ',' + sp.z.toFixed(2) + ') end=(' + ep.x.toFixed(2) + ',' + ep.z.toFixed(2) + ')' };
		});
		// Caught live: an earlier version of the omega x r math swept the rider the WRONG way around the
		// pivot — opposite the platform's own actual spin direction — while every check above (radius,
		// total angle magnitude, return-to-start) still passed, since none of them look at direction, only
		// magnitude. This is the one that actually catches that class of bug.
		t.expect('was swept the SAME way around the pivot as the platform actually spins (not backwards)', function () {
			if (pending()) { return { ok: false, detail: 'everSweptWrongWay=' + st.everSweptWrongWay + ' (pending…)' }; }
			return { ok: !st.everSweptWrongWay, detail: 'everSweptWrongWay=' + st.everSweptWrongWay };
		});
		rig.run(t);
	}, { page: P, steps: 400, description: 'Riding the far end of a rotating platform through a full 360 one way then 360 back sweeps a real radius-matching arc, in the correct direction, and returns near the start position.' });

	// ---- PL7: same as PL6 (far end of a long rotating platform), but spinning 8x faster — the same
	// checks must still hold at a much higher tangential speed, where integration/collision-clip error
	// is more likely to show up (a bug that only appears at speed, invisible at PL6's gentler rate).
	PBF.scaleTest(G, 'PL7', 'riding the far end of a rotating platform at 8x speed sweeps a full-radius arc, returns after 360+360', function (t, S) {
		var rig = riderRotationRig(S, 3.5, ROT_RATE * 8); // near the platform's own end (half-length 4, pre-scale), 8x PL6's rate
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };

		t.log('Same as PL6, but the platform spins 4x faster. A rider out near the end must still be swept ' +
			'through a real arc whose radius matches their distance from the pivot at this higher tangential ' +
			'speed, not flung off or dragged in, and still end up back close to their start position once both ' +
			'spins complete.');
		t.expect('boarded and settled before the spin started', function () {
			if (pending() && st.startPos === null) { return { ok: false, detail: 'settling…' }; }
			return { ok: st.boardedBeforeSpin, detail: 'boardedBeforeSpin=' + st.boardedBeforeSpin };
		});
		t.expect('never went airborne while riding the spin', function () {
			if (pending()) { return { ok: false, detail: 'everUngrounded=' + st.everUngroundedWhileSpinning + ' (pending…)' }; }
			return { ok: !st.everUngroundedWhileSpinning, detail: 'everUngrounded=' + st.everUngroundedWhileSpinning };
		});
		t.expect('held a steady radius near the platform\'s own arm length (not flung off or dragged in)', function () {
			if (pending()) { return { ok: false, detail: 'maxR=' + st.maxRadiusSeen.toFixed(2) + ' (pending…)' }; }
			var ok = st.maxRadiusSeen > st.expectedRadius * 0.8 && st.maxRadiusSeen < st.expectedRadius * 1.2
				&& st.minRadiusSeen > st.expectedRadius * 0.8;
			return { ok: ok, detail: 'minRadiusSeen=' + st.minRadiusSeen.toFixed(3) + ' maxRadiusSeen=' + st.maxRadiusSeen.toFixed(3) + ' expectedRadius=' + st.expectedRadius.toFixed(3) };
		});
		t.expect('was actually carried around — swept close to a full 360 out and 360 back (not frozen)', function () {
			if (pending()) { return { ok: false, detail: 'totalAngleSwept=' + st.totalAngleSwept.toFixed(2) + ' (pending…)' }; }
			var expected = 2 * (2 * Math.PI); // one rev out + one rev back
			var ok = st.totalAngleSwept > expected * 0.7;
			return { ok: ok, detail: 'totalAngleSwept=' + st.totalAngleSwept.toFixed(3) + ' rad (expected ~' + expected.toFixed(3) + ')' };
		});
		t.expect('back near the start position after a full 360 + full 360 back', function () {
			if (pending()) { return { ok: false, detail: 'settling…' }; }
			var sp = st.startPos, ep = st.endPos;
			var d = Math.hypot(ep.x - sp.x, ep.z - sp.z);
			return { ok: d < S.sc(0.5), detail: 'drift=' + d.toFixed(3) + ' start=(' + sp.x.toFixed(2) + ',' + sp.z.toFixed(2) + ') end=(' + ep.x.toFixed(2) + ',' + ep.z.toFixed(2) + ')' };
		});
		t.expect('was swept the SAME way around the pivot as the platform actually spins (not backwards)', function () {
			if (pending()) { return { ok: false, detail: 'everSweptWrongWay=' + st.everSweptWrongWay + ' (pending…)' }; }
			return { ok: !st.everSweptWrongWay, detail: 'everSweptWrongWay=' + st.everSweptWrongWay };
		});
		rig.run(t);
	}, { page: P, steps: 400, description: 'Riding the far end of a rotating platform at 8x speed through a full 360 one way then 360 back sweeps a real radius-matching arc, in the correct direction, and returns near the start position.' });

	// ---- PL8: character at the MIDDLE (pivot) of a rotating platform — should see near-zero sweep ----
	PBF.scaleTest(G, 'PL8', 'riding the middle (pivot) of a rotating platform stays near-stationary through 360+360', function (t, S) {
		var rig = riderRotationRig(S, 0); // dead-center — the pivot itself
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };

		t.log('Same rig as PL6, but the rider stands at the platform\'s own CENTER (the pivot) instead of its ' +
			'far end. A point exactly at the pivot has zero radius, so omega x r must impart ~zero tangential ' +
			'velocity — unlike the far-end rider, this one should barely move at all while the platform spins ' +
			'a full 360 one way and 360 back.');
		t.expect('boarded and settled before the spin started', function () {
			if (pending() && st.startPos === null) { return { ok: false, detail: 'settling…' }; }
			return { ok: st.boardedBeforeSpin, detail: 'boardedBeforeSpin=' + st.boardedBeforeSpin };
		});
		t.expect('never went airborne while riding the spin', function () {
			if (pending()) { return { ok: false, detail: 'everUngrounded=' + st.everUngroundedWhileSpinning + ' (pending…)' }; }
			return { ok: !st.everUngroundedWhileSpinning, detail: 'everUngrounded=' + st.everUngroundedWhileSpinning };
		});
		t.expect('stayed near the pivot the whole time (near-zero radius swept)', function () {
			if (pending()) { return { ok: false, detail: 'maxR=' + st.maxRadiusSeen.toFixed(2) + ' (pending…)' }; }
			return { ok: st.maxRadiusSeen < S.sc(0.5), detail: 'maxRadiusSeen=' + st.maxRadiusSeen.toFixed(3) };
		});
		t.expect('back near the start position after a full 360 + full 360 back', function () {
			if (pending()) { return { ok: false, detail: 'settling…' }; }
			var sp = st.startPos, ep = st.endPos;
			var d = Math.hypot(ep.x - sp.x, ep.z - sp.z);
			return { ok: d < S.sc(0.5), detail: 'drift=' + d.toFixed(3) + ' start=(' + sp.x.toFixed(2) + ',' + sp.z.toFixed(2) + ') end=(' + ep.x.toFixed(2) + ',' + ep.z.toFixed(2) + ')' };
		});
		rig.run(t);
	}, { page: P, steps: 400, description: 'Riding the pivot of a rotating platform through a full 360 one way then 360 back stays near-stationary throughout.' });

	// ---- PL9-PL12: jump-off-platform base-velocity opt flags (_updateVertical) ----
	// Two independent controller options, opposite defaults:
	//   jumpKeepsVerticalBaseVelocity   (default true / opt-OUT)  — jumping off a RISING platform flings
	//                                                                you higher (PL3 already covers the
	//                                                                default-on case; PLV below covers
	//                                                                explicitly opting OUT).
	//   jumpKeepsHorizontalBaseVelocity (default false / opt-IN)  — jumping off a platform that's moving
	//                                                                or spinning SIDEWAYS does NOT carry
	//                                                                that speed into the jump by default;
	//                                                                a project can opt back in.
	// Horizontal carry is tested against BOTH platform kinds (linear PL9/PL10, rotating PL11/PL12) since
	// the flag is meant to apply uniformly regardless of what's producing the horizontal base velocity.

	// Shared rig: ride a horizontally-moving platform (linear or rotating) up to a known ground speed,
	// then jump, and report the horizontal speed immediately after the jump tick. buildMover returns
	// {mover, riderX} — riderX is where buildMover's own platform actually IS at t=0, in BASE (scale-1)
	// units, so the character spawns standing on real platform surface (not off to the side of a linear
	// mover's start point, and not at a rotating mover's pivot, where radius=0 gives zero tangential
	// speed and the whole test would trivially pass regardless of the option under test).
	function jumpOffHorizontalPlatformRig(S, buildMover, controllerOpts) {
		var w = S.flat();
		var built = buildMover(w, S);
		var mover = built.mover;
		var platY = mover.position.y + (0.3 * S.SC) / 2;
		var startX = built.riderX * S.SC;
		var p = S.spawn(w, { x: startX, y: platY + 0.9 * S.SC + 0.001, z: 0 }, controllerOpts || {});
		var settleTicks = 30, jumpTick = 40, TOTAL_TICKS = jumpTick + 5;
		var state = { curTick: 0, totalTicks: TOTAL_TICKS, boardedBeforeJump: false, speedRightAfterJump: null, jumpedAt: -1 };
		return {
			state: state,
			run: function (t) {
				PBF.renderables(t, p, [mover]);
				PBF.drive(t, p, function (tick) {
					state.curTick = tick;
					if (tick === settleTicks) {
						state.boardedBeforeJump = p.grounded && (p._baseVelocity.x !== 0 || p._baseVelocity.z !== 0);
					}
					if (tick === jumpTick + 1) {
						state.speedRightAfterJump = Math.hypot(p.body.linear_velocity.x, p.body.linear_velocity.z);
					}
					return { jumpPressed: tick === jumpTick };
				}, [mover]);
				t.simulate(w, TOTAL_TICKS);
			}
		};
	}

	function linearSidewaysMover(w, S) {
		// Starts at x=-6 (startFrac defaults to 0, mover begins AT pointA) — spawn the rider there too.
		return { mover: S.splatform(w, 2, 0.3, { x: -6, y: 0.15, z: 0 }, { x: 6, y: 0.15, z: 0 }, 4, '#4a7ab0'), riderX: -6 };
	}
	function rotatingSpinMover(w, S) {
		// Off-center (riderX=3.5, not the pivot at 0) so there's real tangential speed to test.
		return { mover: S.srotplatform(w, 10, 10, 0.3, { x: 0, y: 0.15, z: 0 }, 1.2, '#a04a7a'), riderX: 3.5 };
	}

	// ---- PL9: jump off a LINEAR platform, default (opt-out) — horizontal speed should drop to ~0 ----
	PBF.scaleTest(G, 'PL9', 'jumping off a linear platform by default sheds its horizontal speed', function (t, S) {
		var rig = jumpOffHorizontalPlatformRig(S, linearSidewaysMover, {});
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };
		t.log('Ride a sideways-moving platform, then jump with jumpKeepsHorizontalBaseVelocity left at its ' +
			'default (false). The jump should shed the platform\'s horizontal speed — landing with only the ' +
			'character\'s own (near-zero, no input given) horizontal velocity, not the platform\'s.');
		t.expect('boarded the platform before jumping', function () {
			if (pending() && st.boardedBeforeJump === false && st.curTick < 30) { return { ok: false, detail: 'settling…' }; }
			return { ok: st.boardedBeforeJump, detail: 'boardedBeforeJump=' + st.boardedBeforeJump };
		});
		t.expect('horizontal speed right after the jump is near zero (default sheds platform speed)', function () {
			if (pending()) { return { ok: false, detail: 'settling…' }; }
			var sp = st.speedRightAfterJump;
			return { ok: sp !== null && sp < S.sc(0.3), detail: 'speedRightAfterJump=' + (sp === null ? 'n/a' : sp.toFixed(3)) };
		});
		rig.run(t);
	}, { page: P, steps: 45, description: 'Jumping off a linear platform with the default option sheds the platform\'s horizontal speed instead of carrying it into the jump.' });

	// ---- PL10: jump off a LINEAR platform, opted IN — horizontal speed should be KEPT ----
	PBF.scaleTest(G, 'PL10', 'jumping off a linear platform opted into horizontal carry keeps its speed', function (t, S) {
		var rig = jumpOffHorizontalPlatformRig(S, linearSidewaysMover, { jumpKeepsHorizontalBaseVelocity: true });
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };
		t.log('Same rig as PL9, but jumpKeepsHorizontalBaseVelocity is explicitly opted IN. The jump should ' +
			'KEEP the platform\'s horizontal speed — the classic conveyor-belt-momentum feel.');
		t.expect('boarded the platform before jumping', function () {
			if (pending() && st.boardedBeforeJump === false && st.curTick < 30) { return { ok: false, detail: 'settling…' }; }
			return { ok: st.boardedBeforeJump, detail: 'boardedBeforeJump=' + st.boardedBeforeJump };
		});
		t.expect('horizontal speed right after the jump still matches the platform\'s speed (opted-in carry)', function () {
			if (pending()) { return { ok: false, detail: 'settling…' }; }
			var sp = st.speedRightAfterJump;
			return { ok: sp !== null && sp > S.sc(2), detail: 'speedRightAfterJump=' + (sp === null ? 'n/a' : sp.toFixed(3)) };
		});
		rig.run(t);
	}, { page: P, steps: 45, description: 'Jumping off a linear platform with horizontal carry opted in keeps the platform\'s speed in the jump.' });

	// ---- PL11: jump off a ROTATING platform, default (opt-out) — tangential speed should drop to ~0 ----
	PBF.scaleTest(G, 'PL11', 'jumping off a rotating platform by default sheds its tangential speed', function (t, S) {
		var rig = jumpOffHorizontalPlatformRig(S, rotatingSpinMover, {});
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };
		t.log('Ride a spinning platform off-center (so it has real tangential speed), then jump with ' +
			'jumpKeepsHorizontalBaseVelocity left at its default (false). The jump should shed the platform\'s ' +
			'spin speed, same as the linear case in PL9.');
		t.expect('boarded the platform before jumping', function () {
			if (pending() && st.boardedBeforeJump === false && st.curTick < 30) { return { ok: false, detail: 'settling…' }; }
			return { ok: st.boardedBeforeJump, detail: 'boardedBeforeJump=' + st.boardedBeforeJump };
		});
		t.expect('horizontal speed right after the jump is near zero (default sheds platform speed)', function () {
			if (pending()) { return { ok: false, detail: 'settling…' }; }
			var sp = st.speedRightAfterJump;
			return { ok: sp !== null && sp < S.sc(0.3), detail: 'speedRightAfterJump=' + (sp === null ? 'n/a' : sp.toFixed(3)) };
		});
		rig.run(t);
	}, { page: P, steps: 45, description: 'Jumping off a rotating platform with the default option sheds its tangential speed instead of carrying it into the jump.' });

	// ---- PL12: jump off a ROTATING platform, opted IN — tangential speed should be KEPT ----
	PBF.scaleTest(G, 'PL12', 'jumping off a rotating platform opted into horizontal carry keeps its speed', function (t, S) {
		var rig = jumpOffHorizontalPlatformRig(S, rotatingSpinMover, { jumpKeepsHorizontalBaseVelocity: true });
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };
		t.log('Same rig as PL11, but jumpKeepsHorizontalBaseVelocity is explicitly opted IN. The jump should ' +
			'KEEP the platform\'s tangential spin speed — a deliberate "flung off the carousel" feel.');
		t.expect('boarded the platform before jumping', function () {
			if (pending() && st.boardedBeforeJump === false && st.curTick < 30) { return { ok: false, detail: 'settling…' }; }
			return { ok: st.boardedBeforeJump, detail: 'boardedBeforeJump=' + st.boardedBeforeJump };
		});
		t.expect('horizontal speed right after the jump still matches the spin speed (opted-in carry)', function () {
			if (pending()) { return { ok: false, detail: 'settling…' }; }
			var sp = st.speedRightAfterJump;
			return { ok: sp !== null && sp > S.sc(2), detail: 'speedRightAfterJump=' + (sp === null ? 'n/a' : sp.toFixed(3)) };
		});
		rig.run(t);
	}, { page: P, steps: 45, description: 'Jumping off a rotating platform with horizontal carry opted in keeps its tangential speed in the jump.' });

	// ---- PL13: jump off an elevator ONE TICK BEFORE IT REVERSES, explicitly opted OUT of vertical
	// fling — no extra height ----
	PBF.scaleTest(G, 'PL13', 'jumping right before an elevator reverses, opted OUT of vertical fling, jumps normally', function (t, S) {
		var w = S.flat();
		// Elevator height 8, speed 6 u/s, reaches the top and reverses at tick 79 (verified directly:
		// mover.linear_velocity.y is a flat +6 the ENTIRE ride, then instantly flips to -6 at tick 79 —
		// no gradual slowdown, no easing). Jump at tick 78 — the LAST tick the platform is still
		// genuinely rising (+6) — so the platform is GENUINELY still climbing at jump time (checked
		// directly below via jumpVelYAtJump, not assumed), AND it immediately falls away from under the
		// character (reverses to -6) the very next tick instead of continuing to chase them upward.
		// Jumping any EARLIER mid-ascent (verified directly at tick 60) launches correctly but then gets
		// RE-CAUGHT partway up — the still-climbing platform (6 u/s) closes the gap faster than the
		// character's own jump velocity (jumpSpeed 4.6 u/s, decaying under gravity) can open it, so the
		// platform catches back up and carries the character the rest of the way to its own peak before
		// riding them back down — a real, separate, still-open bug (mid-arc re-catch on a platform
		// climbing faster than jumpSpeed), NOT what this test exists to check. Jumping right at the
		// reversal sidesteps that entirely: there's no time left for the platform to close any gap
		// before it starts moving away instead.
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 8, z: 0 }, 6, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, { jumpKeepsVerticalBaseVelocity: false });

		var g = -(w.gravity ? w.gravity.y : -9.81);
		var normalJumpRise = (p.jumpSpeed * p.jumpSpeed) / (2 * g);

		// TOTAL_TICKS must cover the ride up to the jump PLUS enough time for the un-flung jump's fall
		// back down to actually complete. Gravity is a world-constant, NOT scale-proportional (verified
		// directly: a naive S.SC-scaled budget passed at scale 1/2 but cut off scale 0.5's landing a few
		// ticks early — the fall time doesn't scale linearly with S.SC the way the elevator's own climb
		// does), so this uses one fixed, generously-oversized budget checked directly against all three
		// scales' actual landing ticks instead of trying to derive a formula.
		var jumpTick = 78, TOTAL_TICKS = 260, curTick = 0;
		var jumpedAt = -1, jumpStartY = null, jumpPeakY = -Infinity, wasGroundedBeforeJump = false, landedAfterJump = false;
		var jumpVelYAtJump = null;
		PBF.renderables(t, p, [mover]);
		PBF.drive(t, p, function (tick) {
			curTick = tick;
			if (tick === jumpTick) {
				wasGroundedBeforeJump = p.grounded;
				jumpStartY = p.body.position.y;
				jumpVelYAtJump = mover.linear_velocity.y;
			}
			if (tick === jumpTick && wasGroundedBeforeJump) { jumpedAt = jumpTick; }
			if (jumpedAt > 0 && p.body.position.y > jumpPeakY) { jumpPeakY = p.body.position.y; }
			if (jumpedAt > 0 && jumpPeakY > jumpStartY && p.grounded && p.body.position.y < jumpPeakY) {
				landedAfterJump = true;
			}
			return { jumpPressed: tick === jumpTick };
		}, [mover]);

		t.log('Ride the elevator partway up, then jump WHILE it\'s still genuinely climbing, with ' +
			'jumpKeepsVerticalBaseVelocity explicitly opted OUT. Unlike PL3 (default, flings higher), this ' +
			'jump should rise about the same as a normal jump from a dead stop — NOT get the platform\'s ' +
			'still-rising speed added on top.');
		t.expect('jumped while still riding partway up', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'jumpedAt=' + jumpedAt + ' (pending…)' }; }
			return { ok: jumpedAt > 0, detail: 'jumpedAt=' + jumpedAt };
		});
		// The real discriminator this test exists to check — without this, a jump timed AFTER the
		// platform has already reversed (or stopped) would trivially "pass" the rise check below with
		// nothing actually being opted out of, since there'd be no upward platform speed to fling from
		// in the first place (caught live: an earlier version of this test did exactly that).
		t.expect('the platform was genuinely still rising at the moment of the jump', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'jumpVelYAtJump=' + jumpVelYAtJump + ' (pending…)' }; }
			return { ok: jumpVelYAtJump !== null && jumpVelYAtJump > S.sc(0.5), detail: 'jumpVelYAtJump=' + (jumpVelYAtJump === null ? 'n/a' : jumpVelYAtJump.toFixed(3)) };
		});
		// Gated on landedAfterJump (the arc having actually completed), NOT just "jumpPeakY - jumpStartY
		// < threshold right now" — that trivially reads ~0 on the very tick the jump fires, BEFORE the
		// arc has risen at all, and t.expect() latches green the FIRST tick a predicate returns true (see
		// runner.js's evalTick) — an ungated check here would falsely pass immediately at liftoff, before
		// a real fling has any chance to develop over the following ticks (caught live: this exact bug
		// let a sabotaged ignore-the-option build pass here even though the real rise was 5x normal).
		t.expect('the jump rose about a NORMAL amount, not flung higher by the platform (opted out)', function () {
			if (!landedAfterJump) { return { ok: false, detail: 'jumpedAt=' + jumpedAt + ' landedAfterJump=' + landedAfterJump + ' (pending…)' }; }
			var actualRise = jumpPeakY - jumpStartY;
			return {
				ok: actualRise < normalJumpRise + S.sc(0.3),
				detail: 'actualRise=' + actualRise.toFixed(3) + ' normalJumpRise=' + normalJumpRise.toFixed(3)
			};
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 260, description: 'Riding an elevator to its top and jumping there with vertical fling opted OUT rises about a normal jump height instead of getting flung higher.' });

	// ---- PL14: jump on a DESCENDING elevator — the jump must not get eaten/reduced by the platform's
	// negative vertical base velocity ----
	PBF.scaleTest(G, 'PL14', 'jumping on a descending elevator still launches a full jump', function (t, S) {
		var w = S.flat();
		// Same elevator as PL13, reversing at tick 79 (verified directly: mover.linear_velocity.y is a
		// flat +6 the whole ride up, then instantly -6). Jump well AFTER the reversal (tick 120) so the
		// platform is unambiguously, solidly descending — not a borderline/transitional tick.
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 8, z: 0 }, 6, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		// jumpIgnoresDescendingBaseVelocity left at its default (true) — a jump's launch speed should
		// come entirely from jumpSpeed, unreduced by the platform's own negative vertical velocity. The
		// bug this test exists to catch: additive-but-unclamped fling math (jumpSpeed + baseVelocity.y)
		// SUBTRACTS from the launch when baseVelocity.y is negative — at this platform's 6 u/s descent
		// (well above jumpSpeed ~4.6), an unclamped version would launch at a NEGATIVE velocity, i.e.
		// the "jump" wouldn't leave the ground at all — caught live: jumping on a descending platform
		// visibly ate the jump entirely before this fix.
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, {});

		var jumpTick = 120, TOTAL_TICKS = 260, curTick = 0;
		var jumpedAt = -1, wasGroundedBeforeJump = false, jumpVelYAtJump = null, vyRightAfterJump = null;
		PBF.renderables(t, p, [mover]);
		PBF.drive(t, p, function (tick) {
			curTick = tick;
			if (tick === jumpTick) {
				wasGroundedBeforeJump = p.grounded;
				jumpVelYAtJump = mover.linear_velocity.y;
			}
			if (tick === jumpTick && wasGroundedBeforeJump) { jumpedAt = jumpTick; }
			if (tick === jumpTick + 1) { vyRightAfterJump = p.body.linear_velocity.y; }
			return { jumpPressed: tick === jumpTick };
		}, [mover]);

		t.log('Ride the elevator up and past its reversal so it\'s genuinely descending, then jump. The ' +
			'platform\'s own downward speed must NOT subtract from the launch — jumpIgnoresDescendingBaseVelocity ' +
			'defaults to true, so the jump should launch at essentially the character\'s own full jumpSpeed, ' +
			'not get reduced or eaten entirely by the platform\'s negative vertical velocity.');
		t.expect('jumped while riding a genuinely descending platform', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'jumpedAt=' + jumpedAt + ' (pending…)' }; }
			return { ok: jumpedAt > 0, detail: 'jumpedAt=' + jumpedAt };
		});
		t.expect('the platform was genuinely descending at the moment of the jump', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'jumpVelYAtJump=' + jumpVelYAtJump + ' (pending…)' }; }
			return { ok: jumpVelYAtJump !== null && jumpVelYAtJump < -S.sc(0.5), detail: 'jumpVelYAtJump=' + (jumpVelYAtJump === null ? 'n/a' : jumpVelYAtJump.toFixed(3)) };
		});
		t.expect('the jump launched at essentially full jumpSpeed, not reduced by the descent', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'vyRightAfterJump=' + vyRightAfterJump + ' (pending…)' }; }
			// A tick of gravity has already applied by the time this is sampled, so allow a small margin
			// below jumpSpeed rather than requiring an exact match.
			var minAcceptable = p.jumpSpeed - S.sc(0.5);
			return {
				ok: vyRightAfterJump !== null && vyRightAfterJump > minAcceptable,
				detail: 'vyRightAfterJump=' + (vyRightAfterJump === null ? 'n/a' : vyRightAfterJump.toFixed(3)) + ' jumpSpeed=' + p.jumpSpeed.toFixed(3) + ' minAcceptable=' + minAcceptable.toFixed(3)
			};
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 260, description: 'Jumping on a descending elevator launches a full, un-eaten jump — the platform\'s own downward speed does not subtract from the launch.' });

	// ---- PL15: ride a horizontal platform its FULL path (start -> far end -> reverse -> back to
	// start), standing still the whole time — no shuffle (own-velocity spike) at boarding, at the far
	// end where it reverses, or when it comes to rest back at the start. ----
	PBF.scaleTest(G, 'PL15', 'riding a horizontal platform start-to-end-and-back has no shuffle anywhere', function (t, S) {
		var w = S.flat();
		// A brisk, fast platform (not PL9/PL10's gentle 4 u/s) — shuffle (if present) should be most
		// visible at real speed, not hidden by a slow ride.
		var speed = 8 * S.SC;
		// splatform()'s own built-in auto-ping-pong is NOT used here — this test needs to drive the
		// mover itself (settle -> ride -> hard reverse -> ride back -> stop) on its OWN schedule, and
		// letting the built-in auto-reverse run at the same time as a manual override fights it: the
		// built-in logic already flips direction once it reaches an endpoint, so a manual "reverse at
		// tick N" assumes the platform is still at a known position, but the built-in reverse may have
		// ALREADY happened first — verified directly, this exact conflict overshot the mover's own path
		// endpoint by several units before the manual override ever took effect, at which point the
		// character's feet ended up off the platform's actual footprint entirely (the real cause of the
		// "goes airborne" failure, not a real per-tick physics bug). Passing startFrac doesn't help
		// either since the auto-tick still runs every frame regardless — instead, take full manual
		// control of mover.tick from tick 1, so there is only ONE driver of its motion, ever.
		var startZ = -8 * S.SC, endZ = 8 * S.SC;
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: startZ / S.SC }, { x: 0, y: 0.15, z: endZ / S.SC }, 0, '#4a7ab0');
		mover.tick = function () { mover.linear_velocity.set(0, 0, speed); }; // start moving toward +z immediately
		var platY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: platY + 0.9 * S.SC + 0.001, z: startZ }, {});

		var pathLen = endZ - startZ;
		var oneWayTicks = Math.ceil((pathLen / speed) / (1 / 60)) + 5; // +5 settle margin
		var settleTicks = 15;
		var reverseAtTick = settleTicks + oneWayTicks;
		var stopAtTick = reverseAtTick + oneWayTicks;
		var TOTAL_TICKS = stopAtTick + 20;

		var state = {
			curTick: 0, totalTicks: TOTAL_TICKS,
			boardedBeforeRide: false,
			maxOwnSpeedAtBoard: 0, maxOwnSpeedAtReverse: 0, maxOwnSpeedAtStop: 0, maxOwnSpeedOverall: 0,
			everUngrounded: false,
			// POSITION offset from the platform's own center (x/z, in the mover's local frame) — not
			// just velocity. A shuffle could show up as the character's actual position drifting off
			// center (e.g. small per-tick ground-clamp corrections that don't register as a velocity
			// spike but still accumulate into visible lateral wobble) even if _ownVelocity stays near
			// zero the whole time.
			maxOffsetAtBoard: 0, maxOffsetAtReverse: 0, maxOffsetAtStop: 0, maxOffsetOverall: 0
		};
		PBF.renderables(t, p, [mover]);
		PBF.drive(t, p, function (tick) {
			state.curTick = tick;
			if (tick === settleTicks) {
				state.boardedBeforeRide = p.grounded && (p._baseVelocity.x !== 0 || p._baseVelocity.z !== 0);
			}
			if (tick === reverseAtTick) {
				// Reverse HARD — snap the mover's own velocity, same abrupt flip the rotating-platform
				// reversal used (this is the exact shape of change the original shuffle bug was caused
				// by: this tick's endStep acquires the NEW velocity, but gb was built from the OLD one
				// last tick — see FPSCharacterController's outgoingBaseVelocityX/Z fix).
				mover.tick = function () { mover.linear_velocity.set(0, 0, -speed); };
			}
			if (tick === stopAtTick) {
				mover.tick = function () { mover.linear_velocity.set(0, 0, 0); };
			}
			// tick 1's spawn-drop settle (feet start slightly above the surface, one tick to land) is a
			// legitimate transient every test in this suite has, not a mid-ride shuffle failure — only
			// count ungrounded ticks AFTER boarding is expected to be settled.
			if (tick > settleTicks && !p.grounded) { state.everUngrounded = true; }
			var ownSpeed = Math.hypot(p._ownVelocityX, p._ownVelocityZ);
			if (ownSpeed > state.maxOwnSpeedOverall) { state.maxOwnSpeedOverall = ownSpeed; }
			// Position offset from the platform's own center — the character spawned centered (x=0,
			// z=mover's own z), so any nonzero offset here is real drift, not just a velocity blip.
			var offset = Math.hypot(p.body.position.x - mover.position.x, p.body.position.z - mover.position.z);
			if (offset > state.maxOffsetOverall) { state.maxOffsetOverall = offset; }
			// Sample the shuffle-prone windows individually: a few ticks around boarding, around the
			// reversal, and around the stop — a real shuffle shows up as a spike concentrated right at
			// one of these transitions, not spread evenly through the whole ride.
			if (tick >= settleTicks - 2 && tick <= settleTicks + 10) {
				if (ownSpeed > state.maxOwnSpeedAtBoard) { state.maxOwnSpeedAtBoard = ownSpeed; }
				if (offset > state.maxOffsetAtBoard) { state.maxOffsetAtBoard = offset; }
			}
			if (tick >= reverseAtTick - 2 && tick <= reverseAtTick + 10) {
				if (ownSpeed > state.maxOwnSpeedAtReverse) { state.maxOwnSpeedAtReverse = ownSpeed; }
				if (offset > state.maxOffsetAtReverse) { state.maxOffsetAtReverse = offset; }
			}
			if (tick >= stopAtTick - 2 && tick <= stopAtTick + 10) {
				if (ownSpeed > state.maxOwnSpeedAtStop) { state.maxOwnSpeedAtStop = ownSpeed; }
				if (offset > state.maxOffsetAtStop) { state.maxOffsetAtStop = offset; }
			}
			return {}; // no input — pure ride, isolates the base-velocity mechanism from player input
		}, [mover]);

		var pending = function () { return state.curTick < state.totalTicks; };
		t.log('Stand still and ride a horizontal platform its full path — start, all the way to the far end, ' +
			'reverse hard, ride all the way back to the start, then stop — with no player input the whole ' +
			'time. The character\'s OWN velocity (with the platform\'s speed already subtracted back out) ' +
			'must stay near zero throughout: at boarding, at the reversal, and at the final stop. A shuffle ' +
			'is a spike in that own-velocity right at one of those transitions, not steady riding.');
		t.expect('boarded the platform before the ride started', function () {
			if (pending() && state.curTick < settleTicks) { return { ok: false, detail: 'settling…' }; }
			return { ok: state.boardedBeforeRide, detail: 'boardedBeforeRide=' + state.boardedBeforeRide };
		});
		t.expect('never went airborne during the ride', function () {
			if (pending()) { return { ok: false, detail: 'everUngrounded=' + state.everUngrounded + ' (pending…)' }; }
			return { ok: !state.everUngrounded, detail: 'everUngrounded=' + state.everUngrounded };
		});
		t.expect('no shuffle at boarding (own-velocity spike near zero)', function () {
			if (pending()) { return { ok: false, detail: 'maxOwnSpeedAtBoard=' + state.maxOwnSpeedAtBoard.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOwnSpeedAtBoard < S.sc(0.5), detail: 'maxOwnSpeedAtBoard=' + state.maxOwnSpeedAtBoard.toFixed(3) };
		});
		t.expect('no shuffle at the hard reversal (own-velocity spike near zero)', function () {
			if (pending()) { return { ok: false, detail: 'maxOwnSpeedAtReverse=' + state.maxOwnSpeedAtReverse.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOwnSpeedAtReverse < S.sc(0.5), detail: 'maxOwnSpeedAtReverse=' + state.maxOwnSpeedAtReverse.toFixed(3) };
		});
		t.expect('no shuffle at the final stop (own-velocity spike near zero)', function () {
			if (pending()) { return { ok: false, detail: 'maxOwnSpeedAtStop=' + state.maxOwnSpeedAtStop.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOwnSpeedAtStop < S.sc(0.5), detail: 'maxOwnSpeedAtStop=' + state.maxOwnSpeedAtStop.toFixed(3) };
		});
		t.expect('no shuffle ANYWHERE across the whole ride (own-velocity never spikes)', function () {
			if (pending()) { return { ok: false, detail: 'maxOwnSpeedOverall=' + state.maxOwnSpeedOverall.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOwnSpeedOverall < S.sc(0.5), detail: 'maxOwnSpeedOverall=' + state.maxOwnSpeedOverall.toFixed(3) };
		});
		// POSITION checks — the character spawned dead-center on the platform (x=0, z=mover's own z).
		// A CONSTANT one-tick-of-travel lag riding behind the platform's true position is expected and
		// accepted (verified directly: it measures speed*dt exactly, e.g. 0.133 at speed=8 — this is the
		// acquire-in-endStep/apply-next-tick architecture's inherent one-tick delay, not a bug), and it
		// resolves with a single clean snap the instant the platform stops (also verified directly: one
		// tick, no oscillation, no jitter afterward) — accepted as-is, not something to fix. The
		// threshold here is sized to comfortably cover that expected lag at any of this suite's 3
		// scales while still catching a genuinely UNBOUNDED/growing drift, which would be a real bug.
		var maxExpectedLag = S.sc(1.0); // several times the largest observed one-tick lag (0.266 at scale 2), with margin
		t.expect('offset at boarding stayed within the expected one-tick-lag bound (no unbounded drift)', function () {
			if (pending()) { return { ok: false, detail: 'maxOffsetAtBoard=' + state.maxOffsetAtBoard.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOffsetAtBoard < maxExpectedLag, detail: 'maxOffsetAtBoard=' + state.maxOffsetAtBoard.toFixed(3) };
		});
		t.expect('offset at the hard reversal stayed within the expected one-tick-lag bound (no unbounded drift)', function () {
			if (pending()) { return { ok: false, detail: 'maxOffsetAtReverse=' + state.maxOffsetAtReverse.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOffsetAtReverse < maxExpectedLag, detail: 'maxOffsetAtReverse=' + state.maxOffsetAtReverse.toFixed(3) };
		});
		t.expect('offset at the final stop stayed within the expected one-tick-lag bound (no unbounded drift)', function () {
			if (pending()) { return { ok: false, detail: 'maxOffsetAtStop=' + state.maxOffsetAtStop.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOffsetAtStop < maxExpectedLag, detail: 'maxOffsetAtStop=' + state.maxOffsetAtStop.toFixed(3) };
		});
		t.expect('offset across the ENTIRE ride stayed within the expected one-tick-lag bound (no unbounded drift)', function () {
			if (pending()) { return { ok: false, detail: 'maxOffsetOverall=' + state.maxOffsetOverall.toFixed(3) + ' (pending…)' }; }
			return { ok: state.maxOffsetOverall < maxExpectedLag, detail: 'maxOffsetOverall=' + state.maxOffsetOverall.toFixed(3) };
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 400, description: 'Riding a horizontal platform its full path — start, far end, hard reversal, back to start, stop — with no input has no own-velocity shuffle at any transition.' });

	// ---- PL16: crouching while riding a fast rotating platform, no move input, must NOT launch a slide ----
	PBF.scaleTest(G, 'PL16', 'crouching on a fast rotating platform does not trigger a slide', function (t, S) {
		var w = S.flat();
		// Fast spin (2.4 rad/s, matching the live game's rotor) with the rider far enough out that the
		// platform's own tangential speed (omega*r) alone clears moveSpeed — this is the exact scenario
		// slide-entry's speed test must NOT count: riding fast is not the same as MOVING fast yourself.
		// Spun up 0 -> rate at settleTick via mover.tick reassignment (same pattern riderRotationRig
		// uses) so the character boards a STATIONARY platform first, same as jumping onto a live one.
		var rate = 2.4;
		var mover = S.srotplatform(w, 10, 10, 0.3, { x: 0, y: 0.15, z: 0 }, 0, '#a04a7a');
		var riderX = S.sc(4.5);
		var startY = S.sc(0.15) + S.sc(0.15) + 0.9 * S.SC + 0.001;
		var p = S.spawn(w, { x: riderX, y: startY, z: 0 }, {});
		PBF.renderables(t, p, [mover]);

		var settleTick = 15, crouchTick = 60;
		var exceededMoveSpeed = false, everSlid = false;
		PBF.drive(t, p, function (tick) {
			if (tick === settleTick) { mover.tick = function (dt) { mover.angular_velocity.set(0, rate, 0); }; }
			var rawSpeed = PBF.hsp(p);
			if (tick > crouchTick && rawSpeed > p.moveSpeed) { exceededMoveSpeed = true; }
			if (p.sliding) { everSlid = true; }
			return tick > crouchTick ? { crouch: true } : {};
		}, [mover]);
		t.log('Ride a fast rotating platform far from the pivot (its own tangential speed alone exceeds ' +
			'moveSpeed), then crouch with NO move input. The platform\'s speed must not be mistaken for the ' +
			'character\'s own — no slide should ever start.');
		t.expect('the platform\'s raw speed genuinely exceeded moveSpeed (test is exercising the real case)', function () {
			return { ok: exceededMoveSpeed, detail: 'exceededMoveSpeed=' + exceededMoveSpeed };
		});
		t.expect('never entered a slide from riding alone', function () {
			return { ok: !everSlid, detail: 'everSlid=' + everSlid };
		});
		t.simulate(w, 200);
	}, { page: P, steps: 200, description: 'Crouching while riding a fast rotating platform (no move input) must not trigger a slide — the platform\'s own speed is not the character\'s own effort.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
