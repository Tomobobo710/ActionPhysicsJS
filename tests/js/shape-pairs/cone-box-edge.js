(function (Runner, U) {

	// Cone catches the top edge of a still-falling box, apex-first. Run over three grounds - a
	// BoxShape control, one MeshShape, and four MeshShape tiles - to isolate the mesh contact path.

	var TOTAL = 300;
	var MAX_V = 15;
	var MAX_W = 30;

	// box: half-extents; released from just above rest so it is still moving when the cone arrives.
	var BOX_H = 0.35;
	var BOX_DROP = 0.9;

	// ConeShape local +y is the BASE. Rotate PI - CONE_LEAN so the apex points at the box's +x edge.
	var CONE_R = 0.35, CONE_HH = 0.42;
	var CONE_LEAN = 0.6;
	var CONE_GAP = 0.35;

	// A closed 12-triangle box mesh, outward-wound, centered on the origin.
	function boxMeshVF(hx, hy, hz) {
		var v = [
			[-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
			[-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]
		];
		var f = [
			0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
			0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2,
			1, 2, 6, 1, 6, 5, 0, 4, 7, 0, 7, 3
		];
		return { v: v, f: f };
	}

	// U.ground's box as one MeshShape, same dims and position.
	function groundMesh(t, w) {
		var bm = boxMeshVF(20, 0.5, 20);
		return t.mesh(w, bm.v, bm.f, 0, U.withMat({ pos: [0, -0.5, 0], color: '#243B2A' }));
	}

	// Four separate MeshShape tiles meeting at seams through the origin, each its own static body,
	// so the box lands across a tile boundary.
	function groundMeshTiles(t, w) {
		var half = 10;          // each tile spans 10 in x and z -> 2x2 covers the 20x20 U.ground
		var hy = 0.5;
		var bm = boxMeshVF(half, hy, half);
		var centers = [[-half, -half], [half, -half], [-half, half], [half, half]];
		var bodies = [];
		for (var i = 0; i < centers.length; i++) {
			bodies.push(t.mesh(w, bm.v, bm.f, 0, U.withMat({
				pos: [centers[i][0], -hy, centers[i][1]], color: '#243B2A'
			})));
		}
		return bodies;
	}

	function coneEdgeTest(name, makeGround, descTail) {
		Runner.test('shape-pairs', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			makeGround(t, w);

			var boxRestY = BOX_H + 0.02;
			var box = t.box(w, BOX_H, BOX_H, BOX_H, 1, U.withMat({
				pos: [0, boxRestY + BOX_DROP, 0], color: '#4a90d0'
			}));

			var coneRot = U.axisAngle(null, 0, 0, 1, Math.PI - CONE_LEAN);
			var cone = t.cone(w, CONE_R, CONE_HH, 1, U.withMat({
				pos: [BOX_H + 0.10, boxRestY + BOX_DROP + BOX_H + CONE_GAP + CONE_HH, 0],
				rot: coneRot, color: '#d07a4a'
			}));

			var lastTick = 0;
			var worstV = 0, worstVWho = '', worstVTick = 0;
			var worstW = 0, worstWWho = '', worstWTick = 0;
			var contactTick = -1;
			var leftMap = false;
			// Nothing in the scene twists the box about Y, so any yaw is spurious solver torque.
			// Track peak rate and accumulated total, so a slow drift is caught as well as a spike.
			var worstBoxYawRate = 0, worstBoxYawTick = 0, finalBoxYawDeg = 0;

			// The peak |v|/|w| asserts miss a slow multi-tick pump that still skis the box metres.
			var boxLanded = false, landTick = -1;
			var landX = 0, landZ = 0;
			var maxWanderFromLanding = 0, maxWanderTick = 0;
			var relaunchCount = 0, wasGrounded = false;   // ground crossings after first landing
			var maxHeightAfterLanding = 0, maxHeightTick = 0;
			var GROUND_Y = 2 * BOX_H + 0.5;   // box origin height when resting flat on the tile tops
			// per-phase peak |w|, to show when the energy enters
			var wByPhase = { fall: 0, impact: 0, post: 0 };
			var boxPathLen = 0, prevBoxX = null, prevBoxZ = null;

			t.onTick(function (world, tick) {
				lastTick = tick;
				var pair = [['box', box], ['cone', cone]];
				for (var i = 0; i < pair.length; i++) {
					var b = pair[i][1];
					var sv = U.speed(b), sw = U.spin(b);
					if (sv > worstV) { worstV = sv; worstVWho = pair[i][0]; worstVTick = tick; }
					if (sw > worstW) { worstW = sw; worstWWho = pair[i][0]; worstWTick = tick; }
					var p = b.position;
					if (Math.abs(p.x) > 15 || Math.abs(p.z) > 15 || p.y < -3 || p.y > 25) leftMap = true;
				}
				var wy = Math.abs(box.angular_velocity.y);
				if (wy > worstBoxYawRate) { worstBoxYawRate = wy; worstBoxYawTick = tick; }
				var r = box.rotation;
				var yaw = Math.atan2(2 * (r.w * r.y + r.x * r.z), 1 - 2 * (r.y * r.y + r.x * r.x));
				finalBoxYawDeg = yaw * 180 / Math.PI;

				var bx = box.position.x, by = box.position.y, bz = box.position.z, bw = U.spin(box);
				if (contactTick < 0) {
					var d = Math.hypot(bx - cone.position.x, by - cone.position.y, bz - cone.position.z);
					if (d < BOX_H + CONE_HH + 0.05) contactTick = tick;
				}

				// first landing: box within 0.1 of resting height AND slow-ish vertically
				var grounded = by <= GROUND_Y + 0.10;
				if (!boxLanded && grounded && Math.abs(box.linear_velocity.y) < 3) {
					boxLanded = true; landTick = tick; landX = bx; landZ = bz;
				}
				if (boxLanded) {
					var wander = Math.hypot(bx - landX, bz - landZ);
					if (wander > maxWanderFromLanding) { maxWanderFromLanding = wander; maxWanderTick = tick; }
					var h = by - GROUND_Y;
					if (h > maxHeightAfterLanding) { maxHeightAfterLanding = h; maxHeightTick = tick; }
					// re-launch: transition airborne (h > 0.15) after having been grounded
					if (by <= GROUND_Y + 0.06) wasGrounded = true;
					else if (wasGrounded && h > 0.15) { relaunchCount++; wasGrounded = false; }
					if (prevBoxX !== null) boxPathLen += Math.hypot(bx - prevBoxX, bz - prevBoxZ);
				}
				prevBoxX = bx; prevBoxZ = bz;

				// phase-tagged box |w| peaks
				var phase = (contactTick < 0) ? 'fall' : (tick <= contactTick + 8 ? 'impact' : 'post');
				if (bw > wByPhase[phase]) wByPhase[phase] = bw;
			});

			t.expect('cone/box contact does not create linear energy (|v| <= ' + MAX_V + ')', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  worst |v|=' + worstV.toFixed(2) };
				return {
					ok: worstV <= MAX_V,
					detail: 'worst |v|=' + worstV.toFixed(2) + ' (' + worstVWho + ' @ tick ' + worstVTick +
						', contact at tick ' + contactTick + ', limit ' + MAX_V + ')'
				};
			});

			t.expect('cone/box contact does not create angular energy (|w| <= ' + MAX_W + ')', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  worst |w|=' + worstW.toFixed(2) };
				return {
					ok: worstW <= MAX_W,
					detail: 'worst |w|=' + worstW.toFixed(2) + ' (' + worstWWho + ' @ tick ' + worstWTick +
						', contact at tick ' + contactTick + ', limit ' + MAX_W + ')'
				};
			});

			t.expect('neither body left the map', function () {
				if (lastTick < TOTAL) return false;
				return { ok: !leftMap, detail: leftMap ? 'a body was flung out of bounds' : 'both stayed in bounds' };
			});

			// 5 deg leaves room for impact jitter; a real solver leak drives it far past that.
			t.expect('box does not rotate about Y (no spurious contact torque)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				return {
					ok: Math.abs(finalBoxYawDeg) < 5,
					detail: 'final box yaw=' + finalBoxYawDeg.toFixed(2) + ' deg   peak |wY|=' +
						worstBoxYawRate.toFixed(3) + ' @ tick ' + worstBoxYawTick + '   (limit 5 deg)'
				};
			});

			// The cone nudges the box slightly; nothing shoves it metres. A larger wander is the
			// solver pumping energy in over many ticks, which the per-tick |v| asserts never see.
			t.expect('box settles near where it landed (wander < 0.6 m from landing spot)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + (boxLanded ? '  wander so far=' + maxWanderFromLanding.toFixed(2) : '  (box has not landed)') };
				if (!boxLanded) return { ok: false, detail: 'box never landed within ' + TOTAL + ' ticks' };
				return {
					ok: maxWanderFromLanding < 0.6,
					detail: 'landed @ tick ' + landTick + ' at (' + landX.toFixed(2) + ',' + landZ.toFixed(2) +
						'); max wander ' + maxWanderFromLanding.toFixed(2) + ' m @ tick ' + maxWanderTick +
						'; total floor path ' + boxPathLen.toFixed(2) + ' m   (limit 0.6 m)'
				};
			});

			// Cone contact presses the box down, never launches it; any hop > 0.15 m is spurious.
			t.expect('box does not re-launch off the floor after landing', function () {
				if (lastTick < TOTAL) return false;
				if (!boxLanded) return { ok: false, detail: 'box never landed' };
				return {
					ok: relaunchCount === 0 && maxHeightAfterLanding < 0.15,
					detail: relaunchCount + ' re-launch(es); peak height after landing ' +
						maxHeightAfterLanding.toFixed(2) + ' m @ tick ' + maxHeightTick + '   (limit 0.15 m)'
				};
			});

			// Diagnostic only: always passes, prints the fall -> impact -> post breakdown.
			t.expect('[diagnostic] box |w| by phase (fall / impact / post-impact)', function () {
				if (lastTick < TOTAL) return false;
				return {
					ok: true,
					detail: 'fall=' + wByPhase.fall.toFixed(2) + '  impact(+8t)=' + wByPhase.impact.toFixed(2) +
						'  post=' + wByPhase.post.toFixed(2) + '   (contact @ tick ' + contactTick + ')'
				};
			});

			t.simulate(w, TOTAL);
		}, {
			visual: true, steps: TOTAL, page: 'shape-pairs/cone-box-edge',
			description:
				"A dynamic box is released from a small height so it is still falling when a dynamic cone, " +
				"released just outside its +x top edge and rotated apex-down, catches that edge. The cone/box " +
				"contact is a glancing apex/rim-vs-edge case between two moving bodies. " +
				"Asserts: peak |v|/|w| bounded (catches a one-tick blow-up); box does not yaw (no torque " +
				"acts on it); AND - the real failure mode - after landing the box settles within 0.6 m of " +
				"where it touched down and does not bounce back off the floor. A slow multi-tick energy " +
				"pump keeps per-tick |v| low while still skiing the box metres across the floor, so the " +
				"wander / re-launch asserts are what actually catch the mesh-contact bug; the [diagnostic] " +
				"line reports which phase (fall / impact / post) the box's angular energy enters. " + descTail
		});
	}

	coneEdgeTest('cone apex catches the edge of a falling box (BoxShape ground)', U.ground,
		"Ground is the standard BoxShape - the control: no mesh triangles anywhere in the contact chain.");

	coneEdgeTest('cone apex catches the edge of a falling box (MeshShape ground)', groundMesh,
		"Ground is a MeshShape of the same dimensions, so the box lands on mesh triangles - the bedroom's " +
		"contact path. If this fails where the BoxShape-ground version passes, the mesh contact is the cause.");

	coneEdgeTest('cone apex catches the edge of a falling box (4 MeshShape tiles)', groundMeshTiles,
		"Ground is FOUR separate MeshShape box tiles in a 2x2 grid meeting at seams through the origin, so " +
		"the falling box lands right where four tiles butt together - the bedroom furniture is many mesh " +
		"pieces joined this way, not one surface. Tests whether contacts spanning a tile seam make it worse " +
		"than the single-MeshShape ground.");
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
