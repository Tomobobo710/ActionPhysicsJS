(function (Runner, U) {

	// Same drop matrix as pairs.js, but the surface the top shape lands on is a static MeshShape
	// box (12 triangles) instead of a dynamic BoxShape. The bedroom furniture is exactly this:
	// static CompoundShape-of-MeshShape, so a prop touching it goes through convex-vs-triangle
	// narrowphase, never the closed-form primitive paths. pairs.js (BoxShape surface) resolves
	// cone contacts at ~17 rad/s peak; if the same drop onto a mesh box of identical size blows
	// up, the mesh contact path is the culprit.
	//
	// Only the TOP body is dropped (6 shapes x 2 orientations = 12 rows). The mesh box is static
	// (mass 0), matching the furniture - no dynamic-mesh inertia questions.

	// Default per-case tick budget is 300. A tipped cone landing on the mesh box does a slow
	// post-impact roll-and-slide that only fully stops around tick ~550 (same behaviour as
	// hull-on-hull tipped in pairs.js) - give it room via the override.
	var TOTAL_OVERRIDE = { 'cone on mesh-box (tipped)': 640 };
	var DROP_GAP = 1.0;
	var MAX_V = 15;
	var MAX_W = 30;

	var BOX_H = 0.5;   // mesh-box half-extent; comfortably larger than any dropped shape's footprint

	// 12-triangle box mesh, outward-wound, centered on origin.
	function boxMeshVF(hx, hy, hz) {
		var v = [
			[-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
			[-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]
		];
		var f = [
			0, 2, 1, 0, 3, 2,   // -z
			4, 5, 6, 4, 6, 7,   // +z
			0, 1, 5, 0, 5, 4,   // -y
			3, 7, 6, 3, 6, 2,   // +y
			1, 2, 6, 1, 6, 5,   // +x
			0, 4, 7, 0, 7, 3    // -x
		];
		return { v: v, f: f };
	}

	var SHAPES = {
		box: { make: function (t, w, m, o) { return t.box(w, 0.35, 0.35, 0.35, m, o); }, halfUp: 0.35, halfTip: 0.35 },
		sphere: { make: function (t, w, m, o) { return t.sphere(w, 0.35, m, o); }, halfUp: 0.35, halfTip: 0.35 },
		cylinder: { make: function (t, w, m, o) { return t.cylinder(w, 0.32, 0.4, m, o); }, halfUp: 0.4, halfTip: 0.32 },
		cone: { make: function (t, w, m, o) { return t.cone(w, 0.35, 0.42, m, o); }, halfUp: 0.42, halfTip: 0.35 },
		capsule: { make: function (t, w, m, o) { return t.capsule(w, 0.28, 1.0, m, o); }, halfUp: 0.5, halfTip: 0.28 },
		hull: {
			make: function (t, w, m, o) {
				var pts = [
					[-0.35, -0.3, -0.32], [0.35, -0.3, -0.32], [0.35, -0.3, 0.32], [-0.35, -0.3, 0.32],
					[-0.28, 0.34, -0.24], [0.28, 0.34, -0.24], [0.28, 0.34, 0.24], [-0.28, 0.34, 0.24]
				];
				return t.convex(w, pts, m, o);
			}, halfUp: 0.34, halfTip: 0.35
		}
	};
	var KEYS = ['box', 'sphere', 'cylinder', 'cone', 'capsule', 'hull'];

	var TIP_X = U.axisAngle(null, 1, 0, 0, Math.PI / 2);
	var CASES = [
		{ name: 'upright', rot: null, key: 'halfUp' },
		{ name: 'tipped', rot: TIP_X, key: 'halfTip' }
	];

	function makeTest(topKey, cs) {
		var topDef = SHAPES[topKey];
		var testName = topKey + ' on mesh-box (' + cs.name + ')';
		var TOTAL = TOTAL_OVERRIDE[testName] || 300;

		Runner.test('shape-pairs', testName, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);

			// static mesh box resting ON the ground (ground top at y=0): origin at y = BOX_H,
			// so its bottom face sits at y=0 and its top surface is at y = 2*BOX_H.
			var bm = boxMeshVF(BOX_H, BOX_H, BOX_H);
			var mesh = t.mesh(w, bm.v, bm.f, 0, U.withMat({ pos: [0, BOX_H, 0], color: '#556' }));
			var meshTopY = 2 * BOX_H;

			var rot = cs.rot ? { rot: cs.rot } : {};
			var topHalf = topDef[cs.key];
			var topY = meshTopY + DROP_GAP + topHalf + 0.02;
			var top = topDef.make(t, w, 2, U.withMat(Object.assign({ pos: [0, topY, 0], color: '#d07a4a' }, rot)));

			var lastTick = 0;
			var worstV = 0, worstVTick = 0;
			var worstW = 0, worstWTick = 0;
			var leftMap = false;
			var REST_V = 0.1, REST_W = 0.1, HOLD = 30;
			var restRun = 0, restRunMax = 0;

			t.onTick(function (world, tick) {
				lastTick = tick;
				var sv = U.speed(top), sw = U.spin(top);
				if (sv > worstV) { worstV = sv; worstVTick = tick; }
				if (sw > worstW) { worstW = sw; worstWTick = tick; }
				if (sv <= REST_V && sw <= REST_W) restRun++; else restRun = 0;
				if (restRun > restRunMax) restRunMax = restRun;
				var p = top.position;
				if (Math.abs(p.x) > 15 || Math.abs(p.z) > 15 || p.y < -3 || p.y > 25) leftMap = true;
			});

			t.expect('mesh contact does not create linear energy (|v| <= ' + MAX_V + ')', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  worst |v|=' + worstV.toFixed(2) };
				return { ok: worstV <= MAX_V, detail: 'worst |v|=' + worstV.toFixed(2) + ' @ tick ' + worstVTick + ' (limit ' + MAX_V + ')' };
			});

			t.expect('mesh contact does not create angular energy (|w| <= ' + MAX_W + ')', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  worst |w|=' + worstW.toFixed(2) };
				return { ok: worstW <= MAX_W, detail: 'worst |w|=' + worstW.toFixed(2) + ' @ tick ' + worstWTick + ' (limit ' + MAX_W + ')' };
			});

			t.expect('body did not leave the map', function () {
				if (lastTick < TOTAL) return false;
				return { ok: !leftMap, detail: leftMap ? 'flung out of bounds' : 'stayed in bounds' };
			});

			t.expect('body settles and stays settled (rest held ' + HOLD + '+ ticks)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  best rest run=' + restRunMax };
				return {
					ok: restRun >= HOLD,
					detail: 'rest run at end=' + restRun + '/' + HOLD + ' (best=' + restRunMax + ')  |v|=' +
						U.speed(top).toFixed(3) + ' |w|=' + U.spin(top).toFixed(3)
				};
			});

			t.simulate(w, TOTAL);
		}, {
			visual: true, steps: TOTAL, page: 'shape-pairs/mesh-' + cs.name,
			description:
				'Dynamic ' + topKey + ' dropped ~' + DROP_GAP + 'm onto a STATIC MeshShape box (12 triangles) ' +
				'of half-extent ' + BOX_H + ', ' + cs.name + '. Same drop as the pairs.js matrix but the surface ' +
				'is mesh triangles, not a BoxShape - so the contact goes through convex-vs-triangle narrowphase, ' +
				'the exact path the bedroom furniture uses. Watches |v| (limit ' + MAX_V + ') and |w| (limit ' +
				MAX_W + '). A spike here that pairs.js does not show pins the bug on the mesh contact path.'
		});
	}

	for (var i = 0; i < KEYS.length; i++)
		for (var c = 0; c < CASES.length; c++)
			makeTest(KEYS[i], CASES[c]);
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
