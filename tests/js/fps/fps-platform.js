(function (Runner, PBF, ActionPhysics) {

	var Vector3 = ActionPhysics.Vector3;
	var G = 'fps/platform', P = 'fps/platform';

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
			return {};
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
		var farSideZ = S.sc(3);
		PBF.drive(t, p, function (tick) {
			var standing = p.grounded && onPlatform();
			if (standing && boardedAt < 0) boardedAt = tick;

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

	PBF.scaleTest(G, 'PL3', 'ride from the bottom, jump near the top, and fling higher than a normal jump', function (t, S) {
		var w = S.flat();
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 8, z: 0 }, 6, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, {});
		PBF.renderables(t, p, [mover]);

		var g = -w.gravity.y;
		var normalJumpRise = (p.jumpSpeed * p.jumpSpeed) / (2 * g);

		var groundedFlips = 0, lastGrounded = null, maxGap = 0, settledTick = 6;

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

	PBF.scaleTest(G, 'PL4', 'sliding across an oncoming platform behaves normally, not a boost pad', function (t, S) {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(S.sc(15), S.sc(1), S.sc(30), { x: 0, y: -S.sc(1), z: S.sc(10) }, '#333'));

		var mover = S.splatform(w, 4, 0.3, { x: 0, y: 0.15, z: 15 }, { x: 0, y: 0.15, z: -15 }, 3, '#4a7ab0');
		var p = S.feetSpawn(w, 0, -S.sc(10), {});
		PBF.renderables(t, p, [mover]);

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

			if (boardedAt >= 0 && leftAt < 0 && prevPz !== null) {
				var advance = p.body.position.z - prevPz;
				if (advance < minPzAdvanceAfterBoard) { minPzAdvanceAfterBoard = advance; }
				if (advance <= S.sc(0.06)) { stalledAfterBoard = true; }
			}
			prevPz = p.body.position.z;

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

	PBF.scaleTest(G, 'PL5', 'catching an outrunning platform from behind must not launch the player faster', function (t, S) {
		var w = PBF.makeWorld();
		w.addRigidBody(PBF.staticBox(S.sc(15), S.sc(1), S.sc(30), { x: 0, y: -S.sc(1), z: S.sc(10) }, '#333'));

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

			var limit = S.sc(PLATFORM_SPEED) + S.sc(1);
			return { ok: worstSpikeAfterBoard <= limit, detail: 'speedBeforeBoard=' + speedBeforeBoard.toFixed(2) +
				' worstSpikeAfterBoard=' + worstSpikeAfterBoard.toFixed(2) + ' limit=' + limit.toFixed(2) };
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 220, description: 'Sprint-sliding down a lane and catching a platform from behind, moving away down the same lane — boarding it must not launch the player to a higher speed.' });

	var ROT_RATE = 0.3;
	var PLATFORM_HALF_SIDE = 5;

	function riderRotationRig(S, riderOffsetX, rotRate) {
		var rate = rotRate !== undefined ? rotRate : ROT_RATE;
		var w = S.flat();

		var mover = S.srotplatform(w, PLATFORM_HALF_SIDE * 2, PLATFORM_HALF_SIDE * 2, 0.3, { x: 0, y: 0.15, z: 0 }, 0, '#4a7ab0');
		var platY = mover.position.y + (0.3 * S.SC) / 2;
		var startX = riderOffsetX * S.SC;
		var p = S.spawn(w, { x: startX, y: platY + 0.9 * S.SC + 0.001, z: 0 }, {});

		var mountSettleTicks = 15;
		var spinOneRevTicks = Math.round((2 * Math.PI / rate) / (1 / 60));
		var spinBackStart = mountSettleTicks + spinOneRevTicks;
		var TOTAL_TICKS = spinBackStart + spinOneRevTicks + 20;

		var state = {
			curTick: 0, totalTicks: TOTAL_TICKS,
			boardedBeforeSpin: false, everUngroundedWhileSpinning: false,
			maxRadiusSeen: 0, minRadiusSeen: Infinity, totalAngleSwept: 0, everSweptWrongWay: false,
			startPos: null, endPos: null, expectedRadius: Math.abs(startX)
		};
		var angleAccum = 0, lastAngle = null;

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

					if (tick === mountSettleTicks) { mover.tick = function (dt) { mover.angular_velocity.set(0, rate, 0); }; }
					if (tick === spinBackStart - 1) { mover.tick = function (dt) { mover.angular_velocity.set(0, -rate, 0); }; }
					if (tick === spinBackStart + spinOneRevTicks - 1) { mover.tick = function (dt) { mover.angular_velocity.set(0, 0, 0); }; }
					if (tick > mountSettleTicks && tick <= spinBackStart + spinOneRevTicks) {
						if (!p.grounded) { state.everUngroundedWhileSpinning = true; }
						var rx = p.body.position.x - mover.position.x, rz = p.body.position.z - mover.position.z;
						var radius = Math.hypot(rx, rz);
						if (radius > state.maxRadiusSeen) { state.maxRadiusSeen = radius; }
						if (radius < state.minRadiusSeen) { state.minRadiusSeen = radius; }

						if (radius > S.sc(0.1)) {
							var angle = Math.atan2(rz, rx);
							if (lastAngle !== null) {
								var d = angle - lastAngle;
								while (d > Math.PI) { d -= 2 * Math.PI; }
								while (d < -Math.PI) { d += 2 * Math.PI; }
								angleAccum += Math.abs(d);
								state.totalAngleSwept = angleAccum;

								var matWorld = new Vector3();
								mover.rotation.transformVectorInto(matRef, matWorld);
								var matAngle = Math.atan2(matWorld.z, matWorld.x);
								if (lastMatAngle !== null) {
									var md = matAngle - lastMatAngle;
									while (md > Math.PI) { md -= 2 * Math.PI; }
									while (md < -Math.PI) { md += 2 * Math.PI; }

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
					return {};
				}, [mover]);
				t.simulate(w, TOTAL_TICKS);
			}
		};
	}

	PBF.scaleTest(G, 'PL6', 'riding the far end of a rotating platform sweeps a full-radius arc, returns after 360+360', function (t, S) {
		var rig = riderRotationRig(S, 3.5);
		var st = rig.state;
		var pending = function () { return st.curTick < st.totalTicks; };

		t.log('Stand still at the far end of a long platform (not yet spinning) so mounting is settled, then ' +
			'spin it a full 360 one way and a full 360 back the other way. A rider out near the end must be ' +
			'swept through a real arc whose radius matches their distance from the pivot — not stand frozen ' +
			'in world space while the platform turns under them — and end up back close to their start ' +
			'position once both spins complete.');

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
			var expected = 2 * (2 * Math.PI);
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
	}, { page: P, steps: 400, description: 'Riding the far end of a rotating platform through a full 360 one way then 360 back sweeps a real radius-matching arc, in the correct direction, and returns near the start position.' });

	PBF.scaleTest(G, 'PL7', 'riding the far end of a rotating platform at 8x speed sweeps a full-radius arc, returns after 360+360', function (t, S) {
		var rig = riderRotationRig(S, 3.5, ROT_RATE * 8);
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
			var expected = 2 * (2 * Math.PI);
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

	PBF.scaleTest(G, 'PL8', 'riding the middle (pivot) of a rotating platform stays near-stationary through 360+360', function (t, S) {
		var rig = riderRotationRig(S, 0);
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
		return { mover: S.splatform(w, 2, 0.3, { x: -6, y: 0.15, z: 0 }, { x: 6, y: 0.15, z: 0 }, 4, '#4a7ab0'), riderX: -6 };
	}
	function rotatingSpinMover(w, S) {
		return { mover: S.srotplatform(w, 10, 10, 0.3, { x: 0, y: 0.15, z: 0 }, 1.2, '#a04a7a'), riderX: 3.5 };
	}

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

	PBF.scaleTest(G, 'PL13', 'jumping right before an elevator reverses, opted OUT of vertical fling, jumps normally', function (t, S) {
		var w = S.flat();

		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 8, z: 0 }, 6, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: startY + 0.9 * S.SC + 0.001, z: 0 }, { jumpKeepsVerticalBaseVelocity: false });

		var g = -(w.gravity ? w.gravity.y : -9.81);
		var normalJumpRise = (p.jumpSpeed * p.jumpSpeed) / (2 * g);

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

		t.expect('the platform was genuinely still rising at the moment of the jump', function () {
			if (curTick < TOTAL_TICKS) { return { ok: false, detail: 'jumpVelYAtJump=' + jumpVelYAtJump + ' (pending…)' }; }
			return { ok: jumpVelYAtJump !== null && jumpVelYAtJump > S.sc(0.5), detail: 'jumpVelYAtJump=' + (jumpVelYAtJump === null ? 'n/a' : jumpVelYAtJump.toFixed(3)) };
		});

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

	PBF.scaleTest(G, 'PL14', 'jumping on a descending elevator still launches a full jump', function (t, S) {
		var w = S.flat();

		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: 0 }, { x: 0, y: 8, z: 0 }, 6, '#4a7ab0');
		var startY = mover.position.y + (0.3 * S.SC) / 2;

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

			var minAcceptable = p.jumpSpeed - S.sc(0.5);
			return {
				ok: vyRightAfterJump !== null && vyRightAfterJump > minAcceptable,
				detail: 'vyRightAfterJump=' + (vyRightAfterJump === null ? 'n/a' : vyRightAfterJump.toFixed(3)) + ' jumpSpeed=' + p.jumpSpeed.toFixed(3) + ' minAcceptable=' + minAcceptable.toFixed(3)
			};
		});
		t.simulate(w, TOTAL_TICKS);
	}, { page: P, steps: 260, description: 'Jumping on a descending elevator launches a full, un-eaten jump — the platform\'s own downward speed does not subtract from the launch.' });

	PBF.scaleTest(G, 'PL15', 'riding a horizontal platform start-to-end-and-back has no shuffle anywhere', function (t, S) {
		var w = S.flat();

		var speed = 8 * S.SC;

		var startZ = -8 * S.SC, endZ = 8 * S.SC;
		var mover = S.splatform(w, 2, 0.3, { x: 0, y: 0.15, z: startZ / S.SC }, { x: 0, y: 0.15, z: endZ / S.SC }, 0, '#4a7ab0');
		mover.tick = function () { mover.linear_velocity.set(0, 0, speed); };
		var platY = mover.position.y + (0.3 * S.SC) / 2;
		var p = S.spawn(w, { x: 0, y: platY + 0.9 * S.SC + 0.001, z: startZ }, {});

		var pathLen = endZ - startZ;
		var oneWayTicks = Math.ceil((pathLen / speed) / (1 / 60)) + 5;
		var settleTicks = 15;
		var reverseAtTick = settleTicks + oneWayTicks;
		var stopAtTick = reverseAtTick + oneWayTicks;
		var TOTAL_TICKS = stopAtTick + 20;

		var state = {
			curTick: 0, totalTicks: TOTAL_TICKS,
			boardedBeforeRide: false,
			maxOwnSpeedAtBoard: 0, maxOwnSpeedAtReverse: 0, maxOwnSpeedAtStop: 0, maxOwnSpeedOverall: 0,
			everUngrounded: false,

			maxOffsetAtBoard: 0, maxOffsetAtReverse: 0, maxOffsetAtStop: 0, maxOffsetOverall: 0
		};
		PBF.renderables(t, p, [mover]);
		PBF.drive(t, p, function (tick) {
			state.curTick = tick;
			if (tick === settleTicks) {
				state.boardedBeforeRide = p.grounded && (p._baseVelocity.x !== 0 || p._baseVelocity.z !== 0);
			}
			if (tick === reverseAtTick) {

				mover.tick = function () { mover.linear_velocity.set(0, 0, -speed); };
			}
			if (tick === stopAtTick) {
				mover.tick = function () { mover.linear_velocity.set(0, 0, 0); };
			}

			if (tick > settleTicks && !p.grounded) { state.everUngrounded = true; }
			var ownSpeed = Math.hypot(p._ownVelocityX, p._ownVelocityZ);
			if (ownSpeed > state.maxOwnSpeedOverall) { state.maxOwnSpeedOverall = ownSpeed; }

			var offset = Math.hypot(p.body.position.x - mover.position.x, p.body.position.z - mover.position.z);
			if (offset > state.maxOffsetOverall) { state.maxOffsetOverall = offset; }

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
			return {};
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

		var maxExpectedLag = S.sc(1.0);
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

	PBF.scaleTest(G, 'PL16', 'crouching on a fast rotating platform does not trigger a slide', function (t, S) {
		var w = S.flat();

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
	typeof module !== 'undefined' && module.exports ? require('../_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
