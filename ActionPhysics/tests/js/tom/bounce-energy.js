// Tom's Suite — BOUNCE ENERGY.
// A physical settle only LOSES energy, so each bounce apex must be no higher than the previous one.
// We drop several bodies and track every apex (the height at which upward velocity crosses back through
// zero). If any apex comes out higher than its predecessor, energy was injected — a resting-velocity
// clamp or bad restitution snapping mid-settle. The check flips red the instant a growing apex appears.
//
// Apex tracking is stateful, held in the predicate's closure and advanced every tick — same headless
// and in-browser.
(function (Runner, U) {
	Runner.suite('tom');

	var TOTAL = 300;

	// Returns a predicate that PASSES if the body's bounce apexes never grow through the whole run.
	// It only resolves (green) at the end of the settle window; until then it reports live state and,
	// if it ever sees a growing apex, it latches to a failing state that can never go green.
	function noEnergyGain(b) {
		var rising = false, peak = b.position.y, lastApex = null, worstGrowth = 0, grew = false, ticks = 0;
		return function () {
			ticks++;
			var y = b.position.y, vy = b.linear_velocity.y;
			if (vy > 0.01) { rising = true; if (y > peak) peak = y; }
			else if (rising && vy <= 0.01) {
				rising = false;
				if (lastApex != null) { var g = peak - lastApex; if (g > 1e-3) { grew = true; if (g > worstGrowth) worstGrowth = g; } }
				lastApex = peak; peak = y;
			}
			if (grew) return { ok: false, detail: 'APEX GREW by ' + worstGrowth.toFixed(4) + ' (energy injected)' };
			// pass only once the run is essentially over and no growth was ever seen
			var settled = U.speed(b) < 0.05 && ticks > TOTAL - 40;
			return { ok: settled && !grew, detail: 'apexN=' + (lastApex == null ? 0 : 1) + ' worstGrowth=' + worstGrowth.toFixed(4) + (settled ? ' settled' : '') };
		};
	}

	function bounce(name, page, build, desc) {
		Runner.test('bounce energy', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);
			var b = build(t, w);
			t.expect('every bounce apex only decays (no energy added)', noEnergyGain(b));
			t.simulate(w, TOTAL);
		}, { visual: true, steps: TOTAL, page: page, description: desc });
	}

	bounce('box (flat)', 'box',
		function (t, w) { return t.box(w, 0.45, 0.45, 0.45, 10, { pos: [0, 3, 0], friction: 3, restitution: 0, color: '#5577ff' }); },
		"A box dropped flat. PASS: each bounce is no higher than the last as it settles — energy only " +
		"leaves the system. A growing apex means the solver injected energy.");

	bounce('box (corner)', 'box',
		function (t, w) { return t.box(w, 0.45, 0.45, 0.45, 10, { pos: [0, 2, 0], rot: U.axisAngle(t, 0.3, 0.2, 0.15, Math.PI / 5), friction: 3, restitution: 0, color: '#ff8c42' }); },
		"A box slammed corner-first. PASS: apexes only decay. The off-center impact must not pump the " +
		"box higher on a later bounce.");

	bounce('sphere', 'sphere',
		function (t, w) { return t.sphere(w, 0.4, 1, { pos: [0, 3, 0], friction: 3, restitution: 0, color: '#0ff' }); },
		"A sphere dropped from y=3. PASS: bounce heights only decrease as it settles.");

	bounce('cone (nose-down)', 'cone',
		function (t, w) { return t.cone(w, 0.4, 0.5, 1, { pos: [0, 1.2, 0], rot: U.axisAngle(t, 1, 0, 0, Math.PI), friction: 3, restitution: 0, color: '#ffd166' }); },
		"A cone dropped nose-down, landing on its tip. PASS: it tips onto its slant with each apex no " +
		"higher than the last — no energy added at the point contact.");

	bounce('cone (base-down)', 'cone',
		function (t, w) { return t.cone(w, 0.4, 0.5, 1, { pos: [0, 1.2, 0], friction: 3, restitution: 0, color: '#a3d900' }); },
		"A cone dropped base-down (stable pose). PASS: apexes only decay as it settles on its base.");

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
