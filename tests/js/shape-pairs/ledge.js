(function (Runner, U) {

	// A prop parked on a ledge, overhanging it by a fraction of its own width. Nothing pushes it.
	// While its centre of mass is over the support it must sit exactly still; past the edge it must
	// tip off. Run on a BoxShape ledge and a MeshShape ledge, since the two take different contact
	// paths, and at several overhangs so a shape that holds at 40% but not 20% shows up as the
	// contradiction it is.

	var TOTAL = 400;
	var LEDGE_TOP = 1.0;
	var LEDGE = { hx: 2, hy: 0.5, hz: 2 };
	var S = 0.3;                 // prop half-extent / radius
	var SUPPORTED = [0, 0.2, 0.4];
	var OVERHUNG = [0.6, 0.8];
	var REST_V = 0.10, REST_W = 0.30;
	var MAX_DRIFT = 0.05;

	function boxMeshVF(hx, hy, hz) {
		return {
			v: [[-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
				[-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]],
			f: [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
				3, 7, 6, 3, 6, 2, 1, 2, 6, 1, 6, 5, 0, 4, 7, 0, 7, 3]
		};
	}

	function makeProp(t, w, kind, opts) {
		if (kind === 'box') return t.box(w, S, S, S, 1, opts);
		if (kind === 'cylinder') return t.cylinder(w, S, S, 1, opts);
		if (kind === 'cone') return t.cone(w, S, S, 1, opts);
		return t.sphere(w, S, 1, opts);
	}

	// `frac` of the prop's width hangs past the ledge edge. 0 = flush with the edge, 1 = fully off.
	function ledgeCase(kind, groundKind, frac, supported) {
		var pct = Math.round(frac * 100);
		var name = kind + ' on a ' + groundKind + ' ledge, ' + pct + '% overhang';
		Runner.test('shape-pairs', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);

			var ledgePos = [0, LEDGE_TOP - LEDGE.hy, 0];
			if (groundKind === 'mesh') {
				var vf = boxMeshVF(LEDGE.hx, LEDGE.hy, LEDGE.hz);
				t.mesh(w, vf.v, vf.f, 0, U.withMat({ pos: ledgePos, color: '#243B2A' }));
			} else {
				t.box(w, LEDGE.hx, LEDGE.hy, LEDGE.hz, 0, U.withMat({ pos: ledgePos, color: '#243B2A' }));
			}

			var propX = LEDGE.hx - S + 2 * S * frac;
			var prop = makeProp(t, w, kind, U.withMat({
				pos: [propX, LEDGE_TOP + S + 0.001, 0], color: '#c98a3a'
			}));

			var x0 = prop.position.x, z0 = prop.position.z;
			var lastTick = 0, maxDrift = 0, peakV = 0, peakW = 0, peakVTick = 0, peakWTick = 0;
			var minY = Infinity;

			t.onTick(function (world, tick) {
				lastTick = tick;
				var p = prop.position, sv = U.speed(prop), sw = U.spin(prop);
				if (sv > peakV) { peakV = sv; peakVTick = tick; }
				if (sw > peakW) { peakW = sw; peakWTick = tick; }
				if (p.y < minY) minY = p.y;
				var d = Math.sqrt((p.x - x0) * (p.x - x0) + (p.z - z0) * (p.z - z0));
				if (d > maxDrift) maxDrift = d;
			});

			if (supported) {
				// The whole claim: a supported prop is not pushed by anything, so it must not move.
				t.expect('stays put on the ledge (drift < ' + MAX_DRIFT + ' m)', function () {
					if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
					return {
						ok: maxDrift < MAX_DRIFT,
						detail: 'max drift=' + maxDrift.toFixed(4) + ' m  final x=' + prop.position.x.toFixed(4) +
							' (dropped at ' + x0.toFixed(4) + ')'
					};
				});
				t.expect('never picks up speed from nothing', function () {
					if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
					return {
						ok: peakV < REST_V && peakW < REST_W,
						detail: 'peak |v|=' + peakV.toFixed(3) + ' @t' + peakVTick +
							'  peak |w|=' + peakW.toFixed(3) + ' @t' + peakWTick +
							'  (limits ' + REST_V + ' / ' + REST_W + ')'
					};
				});
				t.expect('does not fall off the ledge', function () {
					if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
					return {
						ok: prop.position.y > LEDGE_TOP - 0.5,
						detail: 'final y=' + prop.position.y.toFixed(3) + ' (ledge top ' + LEDGE_TOP + ')'
					};
				});
			} else {
				// Past the edge the prop must actually leave, and land on the floor rather than
				// being flung.
				t.expect('tips off the ledge and lands on the floor', function () {
					if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
					var y = prop.position.y;
					return {
						ok: y < LEDGE_TOP - 0.3 && y > -1,
						detail: 'final y=' + y.toFixed(3) + '  peak |v|=' + peakV.toFixed(2)
					};
				});
				t.expect('is not flung doing it (|v| <= 12)', function () {
					if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
					return { ok: peakV <= 12, detail: 'peak |v|=' + peakV.toFixed(2) + ' @t' + peakVTick };
				});
			}

			t.simulate(w, TOTAL);
		}, {
			visual: true, steps: TOTAL, page: 'shape-pairs/ledge',
			description: 'A ' + kind + ' parked on a ' + groundKind + ' ledge with ' + pct +
				'% of its width hanging past the edge. ' +
				(supported ? 'Its centre of mass is still over the support, so it must sit dead still.'
					: 'Its centre of mass is past the edge, so it must tip off and land, not be flung.')
		});
	}

	['box', 'sphere', 'cylinder', 'cone'].forEach(function (kind) {
		['box', 'mesh'].forEach(function (groundKind) {
			SUPPORTED.forEach(function (f) { ledgeCase(kind, groundKind, f, true); });
			OVERHUNG.forEach(function (f) { ledgeCase(kind, groundKind, f, false); });
		});
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
