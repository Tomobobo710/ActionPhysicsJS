/**
 * Single props settling on a CompoundShape floor of MeshShape tiles - the perf-settle scene's
 * geometry, one prop at a time so a failure names the shape and the placement.
 *
 * Motion is judged by path length rather than speed: a prop buzzing in place has a small |v| and
 * decaying kinetic energy, so speed- and energy-based checks call it settled, while summing how far
 * it actually travels does not.
 */
(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;

	// Same tile size as the perf-settle scene: a 200m map over 55 tiles a side.
	var TILE = 200 / 55;
	// Long enough that a prop which lands on a tile edge and topples off the step has finished doing
	// so before the settle window opens. At 300 ticks a box was still tumbling down a 0.11m ledge and
	// scored as "vibrating" - that was the test being too short, not the engine misbehaving.
	var TOTAL_TICKS = 900;
	// Motion is only judged after everything has had time to land and settle.
	var SETTLE_FROM = 840;

	// Distance travelled over the settle window. A resting body scores ~0 (solver residual is
	// sub-millimetre), a rocking one 0.1m+; 0.05 sits clear of both.
	var MAX_PATH = 0.05;
	// Same idea for orientation - a rocking prop racks up radians while ending where it started.
	var MAX_ROT = 0.30;
	// A prop must not be pushed below the tile it rests on.
	var MAX_SINK = 0.30;
	// Speed/spin under which the prop counts as having come to rest for the first time.
	var REST_SPEED = 0.05;
	var REST_SPIN = 0.10;
	// Speed a resting prop may gain in one tick. Contacts remove energy, so anything past this came
	// from a phantom penetration.
	var MAX_KICK = 0.5;
	// How far it may move after first coming to rest. Settling a few cm as neighbouring contacts
	// resolve is fine; sliding most of a tile is the prop being shoved around.
	var MAX_DRIFT = 0.15;
	// Consecutive ticks under the rest thresholds before a prop counts as settled (1 second).
	var REST_HOLD_TICKS = 60;
	// Deepest overlap a settled prop's contacts may report. Catches a slow monotonic burrow, which
	// the path and drift checks miss entirely: sinking at under a millimetre per tick stays well
	// inside MAX_PATH and reads |v|=0, since the motion is positional drift rather than velocity.
	var MAX_PENETRATION = 0.005;

	// Height of the tile whose CENTRE is (cx, cz), assigned to all four of its corners exactly as the
	// perf scene does. Neighbouring tiles derive different heights from their own centres, so adjacent
	// tiles do not share edge vertices: the floor is a staircase of disconnected plateaus with a
	// vertical gap at every seam and no side walls, and a prop straddling one is partly over open air.
	// Interpolating per-corner instead would make the floor watertight and remove the case under test.
	function tileCentreHeight(cx, cz, sloped) {
		return sloped ? Math.sin(cx * 0.05) * 0.6 + Math.cos(cz * 0.05) * 0.6 : 0;
	}

	// The height of the tile that CONTAINS (x, z) - what a prop at (x, z) is actually resting on.
	function floorUnder(x, z, sloped) {
		var gx = Math.round(x / TILE), gz = Math.round(z / TILE);
		return tileCentreHeight(gx * TILE, gz * TILE, sloped);
	}

	// A 5x5 grid of MeshShape quad tiles as one CompoundShape, centred on the origin - the perf
	// scene's floor in miniature. Each tile is two triangles (so a prop lands across a diagonal seam)
	// and, when sloped, sits at its own flat height with a step down to its neighbours.
	function buildFloor(t, w, sloped) {
		var compound = new AP.CompoundShape();
		var ident = new AP.Quaternion(0, 0, 0, 1);
		var zero = new AP.Vector3(0, 0, 0);
		for (var gz = -2; gz <= 2; gz++) {
			for (var gx = -2; gx <= 2; gx++) {
				var cx = gx * TILE, cz = gz * TILE, th = TILE / 2;
				var h0 = tileCentreHeight(cx, cz, sloped);
				var v = [
					new AP.Vector3(cx - th, h0, cz - th),
					new AP.Vector3(cx + th, h0, cz - th),
					new AP.Vector3(cx + th, h0, cz + th),
					new AP.Vector3(cx - th, h0, cz + th)
				];
				compound.addChildShape(new AP.MeshShape(v, [0, 2, 1, 0, 3, 2]), zero, ident);
			}
		}
		var body = new AP.RigidBody(compound, 0);
		body._color = '#3a4a3a';
		w.addRigidBody(body);
		t.bodies.push(body);
		return body;
	}

	function makeProp(t, w, kind, pos, rot) {
		var opts = { pos: pos, color: kind === 'box' ? '#c98' : (kind === 'cylinder' ? '#8ac' : '#c88') };
		if (rot) opts.rot = rot;
		if (kind === 'box') return t.box(w, 0.25, 0.25, 0.25, 10, opts);
		if (kind === 'cylinder') return t.cylinder(w, 0.25, 0.25, 10, opts);
		if (kind === 'cone') return t.cone(w, 0.25, 0.3, 10, opts);
		return t.sphere(w, 0.25, 10, opts);
	}

	// One prop, one placement, one set of asserts.
	function settleTest(kind, placement, x, z, rot, sloped, ticks, driftLimit, penLimit) {
		var maxDrift = driftLimit || MAX_DRIFT;
		var maxPen = penLimit || MAX_PENETRATION;
		var totalTicks = ticks || TOTAL_TICKS;
		var settleFrom = totalTicks - 60;
		var name = kind + ' settles at ' + placement + (sloped ? ' (sloped floor)' : '');
		Runner.test('compound-floor', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			buildFloor(t, w, sloped);
			var body = makeProp(t, w, kind, [x, floorUnder(x, z, sloped) + 1.0, z], rot);

			var pathLen = 0, rotLen = 0, maxStep = 0;
			var prev = null;
			var worstSink = 0;
			var lastTick = 0;
			var sawNonFinite = false;

			// Whole-run tracking. The settle-window checks only see the last 60 ticks, so a prop
			// kicked at tick 270 that tumbles and happens to be still by 840 would pass them.
			var restTick = -1;          // first tick the prop was properly at rest on the ground
			var restPos = null;         // where it was then
			var driftAfterRest = 0;     // how far it has moved since (a settled prop must not wander)
			var worstKick = 0;          // biggest one-tick speed gain while already resting
			var worstKickTick = -1;
			var prevSpeed = 0;
			var restCandidate = 0, restCandidatePos = null;
			var worstPen = 0, worstPenTick = -1;   // deepest overlap the solver reports once settled

			t.onTick(function (world, tick) {
				lastTick = tick;
				var p = body.position, q = body.rotation;
				if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) { sawNonFinite = true; return; }

				var below = floorUnder(p.x, p.z, sloped) - p.y;
				if (below > worstSink) worstSink = below;

				var v = body.linear_velocity, av = body.angular_velocity;
				var speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				var spin = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);

				// A prop resting on the ground must not gain speed from nothing. Anything that was
				// slow last tick and is moving this tick got it from the contact solver, not gravity.
				if (tick > 30 && prevSpeed < REST_SPEED && speed - prevSpeed > worstKick) {
					worstKick = speed - prevSpeed;
					worstKickTick = tick;
				}
				prevSpeed = speed;

				// Once it has come to rest ON THE GROUND, note where - and never let it wander from
				// there. Requires an actual contact: at tick 1 the prop is stationary in mid-air (it
				// has not been released yet), and latching there would score the whole fall as drift.
				var touching = false;
				var ms = world.narrowphase.manifolds;
				if (ms && ms._manifolds) {
					for (var it = ms._manifolds.values(), e = it.next(); !e.done; e = it.next()) {
						var mm = e.value;
						if ((mm.bodyA === body || mm.bodyB === body) && mm.points.length > 0) {
							touching = true;
							// signedDistance is positive when penetrating.
							if (tick > 120) {
								for (var pi = 0; pi < mm.points.length; pi++) {
									var sd = mm.points[pi].signedDistance;
									if (sd > worstPen) { worstPen = sd; worstPenTick = tick; }
								}
							}
							break;
						}
					}
				}
				if (restTick < 0) {
					// Must hold still for a sustained stretch, not just one tick. A prop part-way
					// through toppling off a ledge passes through |v|=0 at the top of its arc; latching
					// there would score the rest of the (legitimate) topple as wandering.
					if (touching && speed < REST_SPEED && spin < REST_SPIN) {
						restCandidate++;
						if (restCandidate === 1) restCandidatePos = { x: p.x, y: p.y, z: p.z };
						if (restCandidate >= REST_HOLD_TICKS) {
							restTick = tick;
							restPos = restCandidatePos;
						}
					} else {
						restCandidate = 0;
					}
				} else {
					var dd = Math.sqrt((p.x - restPos.x) * (p.x - restPos.x) +
						(p.y - restPos.y) * (p.y - restPos.y) + (p.z - restPos.z) * (p.z - restPos.z));
					if (dd > driftAfterRest) driftAfterRest = dd;
				}

				if (tick < settleFrom) return;
				if (prev) {
					var d = Math.sqrt((p.x - prev.x) * (p.x - prev.x) +
						(p.y - prev.y) * (p.y - prev.y) + (p.z - prev.z) * (p.z - prev.z));
					pathLen += d;
					if (d > maxStep) maxStep = d;
					var dot = Math.abs(prev.qw * q.w + prev.qx * q.x + prev.qy * q.y + prev.qz * q.z);
					if (dot > 1) dot = 1;
					rotLen += 2 * Math.acos(dot);
				}
				prev = { x: p.x, y: p.y, z: p.z, qw: q.w, qx: q.x, qy: q.y, qz: q.z };
			});

			t.expect('comes to rest instead of vibrating in place', function () {
				if (lastTick < totalTicks) return { ok: false, detail: 'still running (' + lastTick + '/' + totalTicks + ')' };
				if (sawNonFinite) return { ok: false, detail: 'NON-FINITE position' };
				var detail = 'path=' + pathLen.toFixed(5) + 'm (limit ' + MAX_PATH + ')  rot=' +
					rotLen.toFixed(4) + 'rad (limit ' + MAX_ROT + ')  worst step=' + maxStep.toFixed(6) + 'm' +
					'  over ticks ' + settleFrom + '-' + totalTicks;
				if (pathLen > MAX_PATH) return { ok: false, detail: 'VIBRATING: ' + detail };
				if (rotLen > MAX_ROT) return { ok: false, detail: 'ROCKING: ' + detail };
				return { ok: true, detail: detail };
			});

			t.expect('does not gain speed from nothing once resting', function () {
				if (lastTick < totalTicks) return { ok: false, detail: 'still running' };
				var detail = 'biggest one-tick speed gain while at rest=' + worstKick.toFixed(3) +
					' m/s (limit ' + MAX_KICK + ')' + (worstKickTick > 0 ? ' @ tick ' + worstKickTick : '');
				if (worstKick > MAX_KICK) return { ok: false, detail: 'KICKED: ' + detail };
				return { ok: true, detail: detail };
			});

			t.expect('stays put after it first comes to rest', function () {
				if (lastTick < totalTicks) return { ok: false, detail: 'still running' };
				if (restTick < 0) return { ok: false, detail: 'NEVER CAME TO REST in ' + totalTicks + ' ticks' };
				var detail = 'first at rest @ tick ' + restTick + ', wandered ' + driftAfterRest.toFixed(3) +
					'm after that (limit ' + maxDrift + ')';
				if (driftAfterRest > maxDrift) return { ok: false, detail: 'WANDERED: ' + detail };
				return { ok: true, detail: detail };
			});

			t.expect('rests on the surface instead of sinking into it', function () {
				if (lastTick < totalTicks) return { ok: false, detail: 'still running' };
				var detail = 'deepest contact overlap after tick 120=' + worstPen.toFixed(5) +
					'm (limit ' + maxPen + ')' + (worstPenTick > 0 ? ' @ tick ' + worstPenTick : '');
				if (worstPen > maxPen) return { ok: false, detail: 'SINKING: ' + detail };
				return { ok: true, detail: detail };
			});

			t.expect('is not pushed through the floor', function () {
				if (lastTick < totalTicks) return { ok: false, detail: 'still running' };
				var detail = 'worst dip below local tile=' + worstSink.toFixed(4) + 'm (limit ' + MAX_SINK + ')';
				if (worstSink > MAX_SINK) return { ok: false, detail: 'SANK: ' + detail };
				return { ok: true, detail: detail };
			});

			t.simulate(w, totalTicks);
		}, {
			visual: true, steps: totalTicks,
			description: 'A single ' + kind + ' dropped onto a 5x5 CompoundShape floor of MeshShape tiles at ' +
				placement + '. Asserts it comes to rest, sits on the surface, and stays put.'
		});
	}

	// Tilted 45 degrees about Z: lands on an edge, then has to topple onto a face and stay there.
	var TILT45 = [0, 0, Math.sin(0.3927), Math.cos(0.3927)];

	var KINDS = ['box', 'cylinder', 'cone', 'sphere'];
	for (var i = 0; i < KINDS.length; i++) {
		var k = KINDS[i];
		settleTest(k, 'a tile centre', 0, 0, null, false);
		// Straddles the quad's own diagonal - two triangles under one prop.
		settleTest(k, 'the quad diagonal', 0.4, 0.0, null, false);
		settleTest(k, 'a tile seam', TILE / 2, 0, null, false);
		settleTest(k, 'a four-tile corner', TILE / 2, TILE / 2, null, false);
		// Sloped: every tile at its own height, so the prop lands on a step rather than a plane.
		settleTest(k, 'a tile seam', TILE / 2, 0, null, true);
	}

	// Cylinders and cones on their side. Upright they rest on a flat circular base and get a clean
	// multi-point patch; tipped over they rest on a curved line, where the support point is far more
	// sensitive to which triangle is under it. 90 degrees about Z lays local +Y onto the floor.
	var ONSIDE = [0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4)];
	var SIDE_KINDS = ['cylinder', 'cone'];
	for (var s = 0; s < SIDE_KINDS.length; s++) {
		var sk = SIDE_KINDS[s];
		settleTest(sk, 'a tile centre, on its side', 0, 0, ONSIDE, false);
		settleTest(sk, 'the quad diagonal, on its side', 0.4, 0.0, ONSIDE, false);
		settleTest(sk, 'a tile seam, on its side', TILE / 2, 0, ONSIDE, false);
		settleTest(sk, 'a four-tile corner, on its side', TILE / 2, TILE / 2, ONSIDE, false);
		settleTest(sk, 'a tile seam, on its side', TILE / 2, 0, ONSIDE, true);
	}

	// Dropped tilted, a box must topple onto a face and then hold still.
	settleTest('box', 'the quad diagonal, dropped tilted 45deg', 0.4, 0.0, TILT45, false);
	// A knife edge: a box tilted 45deg has its lowest feature at the centre of its bottom edge, and
	// TILE/2 puts that edge on the cliff line between two tiles 0.109m apart. It balances on the rim,
	// topples off, and comes to rest tilted against the step - it cannot lie flat on a 0.109m ledge.
	// Hence the wider budgets: the topple needs ~1500 ticks and covers ~0.17m, and the box wedges
	// against the step holding a steady ~7mm of overlap (static compression, not a burrow).
	settleTest('box', 'a tile seam, dropped tilted 45deg', TILE / 2, 0, TILT45, true, 1500, 0.25, 0.015);
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner);
