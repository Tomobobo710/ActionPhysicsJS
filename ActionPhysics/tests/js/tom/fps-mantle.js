/**
 * Tom's Suite — FPSCharacterController MANTLE tests (MT1-MT6).
 *
 * Mantle is input-triggered: cmd.mantle=true fires the probe once in the look direction.
 * All tests at 3 scales via PBF.scaleTest.
 */
(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	var G = 'fps/mantle', P = 'fps/mantle';

	// Helper: add a static ledge box to the world. topY = y of the top face.
	function addLedge(w, hx, hy, hz, pos, color) {
		var b = PBF.staticBox(hx, hy, hz, pos, color || '#5a8a5a');
		w.addRigidBody(b);
		return b;
	}

	// ---- MT1: walk up, tap mantle, arc completes, land on top ----
	PBF.scaleTest(G, 'MT1', 'walk to a ledge, tap mantle, land on top', function (t, S) {
		var w = S.flat();
		// Ledge: top at 1.0×SC (above stepHeight 0.4×SC), positioned at z=hz ahead of the player.
		var topY = S.sc(1.0);
		var hy = topY / 2, hx = S.sc(0.8), hz = S.sc(0.8);
		addLedge(w, hx, hy, hz, { x: 0, y: hy, z: hz });
		var spawnZ = -(hz + S.sc(1.5));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		PBF.renderables(t, p);

		var mantleStarted = false, landedOnTop = false, tappedAt = -1;
		PBF.drive(t, p, function (tick) {
			if (p._moveState === 'mantle') { mantleStarted = true; }
			// "On top" = feet at ledge-top height (tight band, not just "higher than before") AND
			// within the ledge's XZ footprint — a fall back onto the near-side floor must not pass.
			var withinFootprint = Math.abs(p.body.position.x) < hx + S.sc(0.2) &&
				p.body.position.z > -S.sc(0.2) && p.body.position.z < 2 * hz + S.sc(0.2);
			if (mantleStarted && p.grounded && withinFootprint &&
				Math.abs(p.body.position.y - p.height / 2 - topY) < S.sc(0.15)) {
				landedOnTop = true;
			}
			// Walk toward the ledge for 40 ticks, then tap mantle.
			var doMantle = (tick === 40);
			if (doMantle) { tappedAt = tick; }
			return { forward: tick < 40 ? 1 : 0, mantle: doMantle };
		});
		t.log('Walk to a ledge above step height, tap mantle; expect the arc to complete and place the character on top.');
		t.expect('mantle arc started on tap', function () {
			return { ok: mantleStarted, detail: 'mantleStarted=' + mantleStarted + ' tappedAt=' + tappedAt };
		});
		t.expect('landed on the ledge top', function () {
			return { ok: landedOnTop, detail: 'landedOnTop=' + landedOnTop + ' pos=(' + p.body.position.x.toFixed(2) + ',' + p.body.position.y.toFixed(2) + ',' + p.body.position.z.toFixed(2) + ')' };
		});
		t.simulate(w, 200);
	}, { page: P, steps: 200, description: 'Walk to a ledge above step height, tap mantle — arc completes and places character on top.' });

	// ---- MT2: tap mantle while airborne (jump approach) ----
	PBF.scaleTest(G, 'MT2', 'jump toward a ledge, tap mantle while airborne', function (t, S) {
		var w = S.flat();
		// Tall enough that an ordinary jump can't step onto it (would need far more than jump height
		// to clear), but within mantleHeight (2.2×SC default) — only the mantle arc can close the gap.
		var topY = S.sc(2.0);
		var hy = topY / 2, hx = S.sc(0.8), hz = S.sc(0.8);
		// Place ledge a bit further out so the player reaches it while still airborne.
		var ledgeFaceZ = S.sc(2);
		var b = PBF.staticBox(hx, hy, hz, { x: 0, y: hy, z: ledgeFaceZ + hz }, '#5a8a5a');
		w.addRigidBody(b);
		var spawnZ = -(S.sc(1));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		PBF.renderables(t, p);

		var mantleStarted = false, landedOnTop = false, tapped = false;
		PBF.drive(t, p, function (tick) {
			if (p._moveState === 'mantle') { mantleStarted = true; }
			var withinFootprint = Math.abs(p.body.position.x) < hx + S.sc(0.2) &&
				p.body.position.z > ledgeFaceZ - S.sc(0.2) && p.body.position.z < ledgeFaceZ + 2 * hz + S.sc(0.2);
			if (mantleStarted && p.grounded && withinFootprint &&
				Math.abs(p.body.position.y - p.height / 2 - topY) < S.sc(0.15)) {
				landedOnTop = true;
			}
			// Jump immediately, sprint forward. Tap mantle once we're airborne and within reach.
			var nearFace = !p.grounded && p.body.position.z > ledgeFaceZ - p.width / 2 - p.mantleReach;
			var doMantle = nearFace && !tapped && !mantleStarted;
			if (doMantle) { tapped = true; }
			return { forward: 1, sprint: true, jumpPressed: tick === 5, mantle: doMantle };
		});
		t.log('Jump toward a ledge and tap mantle while airborne and near the face; expect the arc to complete.');
		t.expect('mantle arc started while airborne', function () {
			return { ok: mantleStarted, detail: 'mantleStarted=' + mantleStarted + ' tapped=' + tapped };
		});
		t.expect('landed on the ledge top', function () {
			return { ok: landedOnTop, detail: 'landedOnTop=' + landedOnTop + ' pos=(' + p.body.position.x.toFixed(2) + ',' + p.body.position.y.toFixed(2) + ',' + p.body.position.z.toFixed(2) + ')' };
		});
		t.simulate(w, 250);
	}, { page: P, steps: 250, description: 'Jump toward a ledge and tap mantle while airborne — arc completes and deposits character on top.' });

	// ---- MT3: too-tall ledge (above mantleHeight) is NOT mantled ----
	PBF.scaleTest(G, 'MT3', 'ledge above mantleHeight is not mantled even on tap', function (t, S) {
		var w = S.flat();
		// Default mantleHeight is 2.2×SC. Ledge at 3×SC — unreachable.
		var topY = S.sc(3.0);
		var hy = topY / 2, hx = S.sc(0.8), hz = S.sc(0.8);
		addLedge(w, hx, hy, hz, { x: 0, y: hy, z: hz });
		var spawnZ = -(hz + S.sc(1));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		PBF.renderables(t, p);

		var mantleStarted = false;
		PBF.drive(t, p, function (tick) {
			if (p._moveState === 'mantle') { mantleStarted = true; }
			return { forward: tick < 30 ? 1 : 0, mantle: tick === 30 };
		});
		t.log('Tap mantle at a ledge above mantleHeight; expect no mantle (blocked).');
		t.expect('no mantle triggered for too-tall ledge', function () {
			return { ok: !mantleStarted, detail: 'mantleStarted=' + mantleStarted };
		});
		t.simulate(w, 120);
	}, { page: P, steps: 120, description: 'Tapping mantle at a ledge taller than mantleHeight does not trigger the arc.' });

	// ---- MT4: no mantle when not pressing cmd.mantle (idle against ledge) ----
	PBF.scaleTest(G, 'MT4', 'no mantle without cmd.mantle input', function (t, S) {
		var w = S.flat();
		var topY = S.sc(1.0);
		var hy = topY / 2, hx = S.sc(0.8), hz = S.sc(0.8);
		addLedge(w, hx, hy, hz, { x: 0, y: hy, z: hz });
		var spawnZ = -(hz + S.sc(1.5));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		PBF.renderables(t, p);

		var mantleStarted = false;
		PBF.drive(t, p, function () {
			if (p._moveState === 'mantle') { mantleStarted = true; }
			return { forward: 1 }; // never send cmd.mantle
		});
		t.log('Walk into a ledge without ever pressing mantle; the arc must never trigger.');
		t.expect('no mantle without input', function () {
			return { ok: !mantleStarted, detail: 'mantleStarted=' + mantleStarted };
		});
		t.simulate(w, 150);
	}, { page: P, steps: 150, description: 'Walking into a ledge without cmd.mantle never triggers the arc.' });

	// ---- MT5: mantle onto a sloped ledge top ----
	PBF.scaleTest(G, 'MT5', 'mantle up onto a sloped ledge top', function (t, S) {
		var w = S.flat();
		// A box tilted ~20° about X so its top face is a walkable slope at roughly topY height.
		var angle = 20 * Math.PI / 180;
		var rot = PBF.axisAngleQuat(1, 0, 0, angle);
		var hy = S.sc(0.5), hx = S.sc(0.8), hz = S.sc(0.8);
		var topY = hy * 2 * Math.cos(angle);
		var b = PBF.staticBox(hx, hy, hz, { x: 0, y: topY / 2, z: hz }, '#8a7a3a', null, rot);
		w.addRigidBody(b);
		var spawnZ = -(hz + S.sc(1.5));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		PBF.renderables(t, p);

		var mantleStarted = false, landedOnTop = false;
		// The ledge occupies x in [-hx,hx], z in [0, 2*hz] (footprint, ignoring the tilt), top
		// surface near y=topY. "On top" = feet resting near topY AND within the footprint — not
		// just "higher than before", which would also pass a fall back onto the near-side floor.
		PBF.drive(t, p, function (tick) {
			if (p._moveState === 'mantle') { mantleStarted = true; }
			var withinFootprint = p.body.position.x > -hx - S.sc(0.2) && p.body.position.x < hx + S.sc(0.2) &&
				p.body.position.z > -S.sc(0.2) && p.body.position.z < 2 * hz + S.sc(0.2);
			if (mantleStarted && p.grounded && withinFootprint &&
				Math.abs(p.body.position.y - p.height / 2 - topY) < S.sc(0.25)) {
				landedOnTop = true;
			}
			return { forward: tick < 40 ? 1 : 0, mantle: tick === 40 };
		});
		t.log('Tap mantle at a sloped ledge top; the arc should work on non-flat surfaces and land ON the surface, not beside it.');
		t.expect('mantle triggered onto sloped geometry', function () {
			return { ok: mantleStarted, detail: 'mantleStarted=' + mantleStarted };
		});
		t.expect('landed ON the ledge top (not fallen back beside it)', function () {
			return { ok: landedOnTop, detail: 'landedOnTop=' + landedOnTop + ' pos=(' + p.body.position.x.toFixed(2) + ',' + p.body.position.y.toFixed(2) + ',' + p.body.position.z.toFixed(2) + ')' };
		});
		t.simulate(w, 250);
	}, { page: P, steps: 250, description: 'Mantling onto a sloped ledge top works — arc is not restricted to flat surfaces.' });

	// ---- MT6: no mantle when a ceiling blocks standing on the ledge top ----
	PBF.scaleTest(G, 'MT6', 'no mantle when ceiling blocks standing on ledge top', function (t, S) {
		var w = S.flat();
		// A low, clearly-mantleable box (same height as MT1's).
		var topY = S.sc(1.0);
		var hy = topY / 2, hx = S.sc(0.8), hz = S.sc(0.8);
		addLedge(w, hx, hy, hz, { x: 0, y: hy, z: hz });
		// Ceiling directly over the box, just above its top — leaves only 0.5×SC of headroom on top,
		// too low to stand (standHeight ~1.8×SC).
		var ceilThick = S.sc(0.2);
		var ceilBottomY = topY + S.sc(0.5);
		var ceilY = ceilBottomY + ceilThick / 2;
		var ceilBody = PBF.staticBox(hx, ceilThick / 2, hz, { x: 0, y: ceilY, z: hz }, '#aa4444');
		w.addRigidBody(ceilBody);
		var spawnZ = -(hz + S.sc(1.5));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		PBF.renderables(t, p);

		var mantleStarted = false, climbedOntoCeiling = false;
		var probedTopY = null;
		PBF.drive(t, p, function (tick) {
			if (p._moveState === 'mantle') { mantleStarted = true; }
			// The failure mode we're actually guarding against: ending up resting ON TOP of the
			// ceiling slab (climbed over/through it) rather than never leaving the floor.
			if (p.grounded && p.body.position.y > ceilY + S.sc(0.1)) { climbedOntoCeiling = true; }
			// Right before the tap, ask the ledge probe directly what it sees — this is the ONLY
			// way to tell "refused because of the headroom check" apart from "refused because
			// detection failed for some other reason" (e.g. found nothing, or wrongly found the
			// ceiling's own top face instead of the box's top). Without this, "no mantle" alone is
			// not evidence the headroom check specifically did its job.
			if (tick === 39) {
				var fwd = p.getForwardHorizontal(p.yaw);
				var probed = p._probeLedgeAhead(fwd.x, fwd.z);
				probedTopY = probed ? probed.topPoint.y : null;
			}
			return { forward: tick < 40 ? 1 : 0, mantle: tick === 40 };
		});
		t.log('Tap mantle at a ledge with insufficient headroom above; the probe must find the BOX top (not the ceiling), and the arc must still refuse to trigger.');
		t.expect('ledge probe found the box top, not the ceiling', function () {
			if (probedTopY === null) return { ok: false, detail: 'probe found nothing at all — test setup or detection is broken, not exercising the headroom check' };
			var foundBoxTop = Math.abs(probedTopY - topY) < S.sc(0.1);
			var foundCeiling = Math.abs(probedTopY - ceilY) < S.sc(0.1) || probedTopY > topY + S.sc(0.3);
			return { ok: foundBoxTop && !foundCeiling, detail: 'probedTopY=' + probedTopY.toFixed(3) + ' expected boxTop=' + topY.toFixed(3) + ' ceilY=' + ceilY.toFixed(3) };
		});
		t.expect('no mantle when headroom insufficient', function () {
			return { ok: !mantleStarted, detail: 'mantleStarted=' + mantleStarted };
		});
		t.expect('did not end up climbed onto the ceiling', function () {
			return { ok: !climbedOntoCeiling, detail: 'climbedOntoCeiling=' + climbedOntoCeiling + ' y=' + p.body.position.y.toFixed(3) };
		});
		t.simulate(w, 150);
	}, { page: P, steps: 150, description: 'Tapping mantle at a ledge with a ceiling too low to stand on does not trigger the arc.' });

	// ---- MT7: mantle onto a genuinely dynamic, PUSHABLE prop ----
	PBF.scaleTest(G, 'MT7', 'mantle onto a pushable prop', function (t, S) {
		var w = S.flat();
		var side = S.sc(1.2);
		var spawnZ = -(side / 2 + S.sc(1.5));
		var p = S.feetSpawn(w, 0, spawnZ, {});
		// A REAL dynamic body (finite mass via PBF.object, not staticBox), sized as a fraction of
		// the CHARACTER's own push-mass gate (p._pushMassLimit, itself scale^3-derived — see
		// _applyScale) rather than an independently-scaled guess, so this stays proportionally
		// consistent at every scale: light enough to genuinely drift when walked into, but heavy
		// enough (still well below the wall threshold) to stay close by and be there to land on
		// once the mantle arc completes a moment later.
		var pushableMass = p._pushMassLimit * 0.3;
		var box = PBF.object(w, side, pushableMass, { x: 0, y: side / 2, z: side / 2 }, '#7a5a3a', { friction: 0.4, restitution: 0.1 });
		PBF.renderables(t, p);

		var mantleStarted = false, landedOnTop = false, boxPushed = false;
		var boxStartX = box.position.x, boxStartZ = box.position.z;
		PBF.drive(t, p, function (tick) {
			if (p._moveState === 'mantle') { mantleStarted = true; }
			// Confirm the "pushable" claim actually holds — the box should visibly move from the
			// approach before mantle starts.
			var boxDrift = Math.hypot(box.position.x - boxStartX, box.position.z - boxStartZ);
			if (boxDrift > S.sc(0.01)) { boxPushed = true; }
			var withinFootprint = Math.abs(p.body.position.x - box.position.x) < side / 2 + S.sc(0.3) &&
				Math.abs(p.body.position.z - box.position.z) < side / 2 + S.sc(0.3);
			if (mantleStarted && p.grounded && withinFootprint &&
				Math.abs(p.body.position.y - p.height / 2 - box.position.y - side / 2) < S.sc(0.2)) {
				landedOnTop = true;
			}
			return { forward: tick < 40 ? 1 : 0, mantle: tick === 40 };
		});
		t.log('Tap mantle at a light, PUSHABLE dynamic box (finite mass, actually gets shoved); mantle should still work while the box moves.');
		t.expect('the box actually got pushed (genuinely dynamic, not effectively static)', function () {
			return { ok: boxPushed, detail: 'boxPushed=' + boxPushed };
		});
		t.expect('mantle triggered onto the box', function () {
			return { ok: mantleStarted, detail: 'mantleStarted=' + mantleStarted };
		});
		t.expect('landed on top of the box', function () {
			return { ok: landedOnTop, detail: 'landedOnTop=' + landedOnTop + ' pos=(' + p.body.position.x.toFixed(2) + ',' + p.body.position.y.toFixed(2) + ',' + p.body.position.z.toFixed(2) + ')' };
		});
		t.simulate(w, 250);
	}, { page: P, steps: 250, description: 'Tapping mantle at a light, genuinely pushable dynamic box mounts the top — mantle works on movable props too.' });

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
