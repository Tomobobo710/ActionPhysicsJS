// Tom's Suite — BALANCE (must not invent motion).
// A body placed exactly at an unstable equilibrium — a cone balanced on its tip, and a cone resting on
// a single rim edge at 45° — with zero velocity and zero spin. This is a real (if precarious) rest, like
// a pencil balanced on its point. The physics contract: with NO disturbance the solver must not inject
// any lateral or tilting impulse. If the silhouette starts drifting/leaning with nothing touching it,
// that is invented motion (the bug). Oracle: over the whole run the body stays put — its origin barely
// moves horizontally and it barely spins.
(function (Runner, U) {
	Runner.suite('tom');

	var TOTAL = 180;

	// Predicate: the body must STAY PUT at its spawn pose — it must not drift horizontally, sink/rise
	// vertically, or spin, more than the given caps at ANY tick of the run. Latches to fail the moment any
	// is exceeded; otherwise passes once the run has played out. The vertical (fall) check matters: a body
	// spawned in the wrong place would plummet straight down with no drift and no spin, and a drift-only
	// oracle would call that a pass.
	function holdsStill(b, maxDrift, maxSpin) {
		var x0 = b.position.x, y0 = b.position.y, z0 = b.position.z, broke = false, ticks = 0;
		return function () {
			ticks++;
			var drift = Math.sqrt((b.position.x - x0) * (b.position.x - x0) + (b.position.z - z0) * (b.position.z - z0));
			var fall = Math.abs(b.position.y - y0);
			var sw = U.spin(b);
			if (drift > maxDrift || fall > maxDrift || sw > maxSpin) broke = true;
			if (broke) return { ok: false, detail: 'INVENTED MOTION: drift=' + drift.toFixed(3) + ' fall=' + fall.toFixed(3) + ' |w|=' + sw.toFixed(3) };
			return { ok: ticks >= TOTAL - 1, detail: 'drift=' + drift.toFixed(4) + ' fall=' + fall.toFixed(4) + ' |w|=' + sw.toFixed(4) };
		};
	}

	Runner.test('balance', 'cone balanced on its tip holds (no invented motion)', function (t) {
		t.log('Place a cone in perfect tip equilibrium, untouched. It must not start toppling on its own.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		// Tip down (180° about X), COM at y = half_height so the tip rests exactly on the floor. Placed at
		// rest (not dropped) so the ONLY motion that can appear is motion the engine invents.
		var cone = t.cone(w, 0.4, 0.5, 1, { pos: [0, 0.5, 0], rot: U.axisAngle(t, 1, 0, 0, Math.PI), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#42C6FF' });
		t.expect('stays balanced on its tip — no self-driven drift or spin', holdsStill(cone, 0.15, 0.6));
		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'balance',
		description:
			"A cone is placed dead-straight on its tip (COM directly above the point), zero velocity, zero " +
			"spin — an unstable but real equilibrium. PASS: untouched, it holds; the solver invents no lateral " +
			"or tilting motion. FAIL: the silhouette leans/drifts on its own. Being placed at rest (not dropped) " +
			"means any motion that appears was manufactured by the engine."
	});

	Runner.test('balance', 'cone on a 45° rim edge holds (knife-edge balance)', function (t) {
		t.log('Place a cone tilted 45° so one rim point rests on the floor, untouched. It must hold.');
		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);
		var r = 0.4, hh = 0.4;
		// Build tilted at 45° about Z, then lift so the lowest rim point sits ~exactly on the floor.
		var cone = t.cone(w, r, hh, 1, { pos: [0, 5, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 4), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#FFD166' });
		cone.updateDerived();
		// Sample the base-rim ring in WORLD space at the current spawn (y=5), find its lowest point, then
		// drop the whole body by that height so the lowest rim point lands ~exactly on the floor (y=0).
		// (newY = currentY - lowestRimWorldY; the old code negated lowest and spawned the cone underground.)
		var lowest = Infinity, tmp = t.vec(0, 0, 0), out = t.vec(0, 0, 0), xf = cone.getTransform();
		for (var k = 0; k < 256; k++) { var a = (k / 256) * Math.PI * 2; tmp.set(Math.cos(a) * r, -hh, Math.sin(a) * r); xf.transformPointInto(tmp, out); if (out.y < lowest) lowest = out.y; }
		cone.position.set(0, cone.position.y - lowest + 0.0005, 0);
		cone.angular_velocity.set(0, 0, 0); cone.linear_velocity.set(0, 0, 0);
		cone.updateDerived();
		t.expect('stays on the knife-edge — no self-driven drift or spin', holdsStill(cone, 0.15, 0.8));
		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'balance',
		description:
			"A cone is placed at exactly 45°, resting on a single point of its base rim (an unstable knife-edge " +
			"balance), dead still. PASS: untouched, it holds — the engine invents no motion. FAIL: it drifts or " +
			"rolls off on its own. A real drop always carries velocity, so it never lands frozen like this in " +
			"practice; this isolates the 'motion from nothing' question."
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
