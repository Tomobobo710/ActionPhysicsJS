(function (Runner, U) {

	// Dynamic-on-dynamic drop matrix. For every unordered pair of primitive shapes (box, sphere,
	// cylinder, cone, capsule, convex hull), in two orientation cases, a BOTTOM dynamic body is
	// settled on the static ground and a TOP dynamic body of the other shape is dropped ~1m onto
	// it under gravity. The pair may topple - what it may NOT do is gain energy: |v| and |w| are
	// watched every tick and a spike past a sane bound fails the case. This surfaces bad
	// narrowphase/solver pairings (e.g. a cone contact pumping angular velocity) directly.

	var SETTLE_TICKS = 45;    // let the bottom body come to rest on the ground first
	// Default per-case tick budget is 260. A few cases do a slow post-impact slide that isn't
	// done by then and get a longer budget here, keyed by "<top> on <bot> (<case>)".
	var TOTAL_OVERRIDE = {
		'hull on hull (tipped)': 520
	};
	var DROP_GAP = 1.0;       // how far above the bottom body's top the top body starts

	// hard "this is a blow-up" bounds - normal settling never comes near these
	var MAX_V = 15;
	var MAX_W = 30;

	// Each shape: how to make it, its resting half-height (origin -> lowest point) at identity,
	// and at 90deg about X. Convex hull is an axis-aligned box-ish cloud so its extents are known.
	var SHAPES = {
		box: {
			make: function (t, w, mass, opts) { return t.box(w, 0.35, 0.35, 0.35, mass, opts); },
			halfUp: 0.35, halfTip: 0.35
		},
		sphere: {
			make: function (t, w, mass, opts) { return t.sphere(w, 0.35, mass, opts); },
			halfUp: 0.35, halfTip: 0.35
		},
		cylinder: {
			make: function (t, w, mass, opts) { return t.cylinder(w, 0.32, 0.4, mass, opts); },
			halfUp: 0.4, halfTip: 0.32   // on its side, radius is the half-height
		},
		cone: {
			make: function (t, w, mass, opts) { return t.cone(w, 0.35, 0.42, mass, opts); },
			halfUp: 0.42, halfTip: 0.35
		},
		capsule: {
			make: function (t, w, mass, opts) { return t.capsule(w, 0.28, 1.0, mass, opts); },
			halfUp: 0.5, halfTip: 0.28
		},
		hull: {
			make: function (t, w, mass, opts) {
				var pts = [
					[-0.35, -0.3, -0.32], [0.35, -0.3, -0.32], [0.35, -0.3, 0.32], [-0.35, -0.3, 0.32],
					[-0.28, 0.34, -0.24], [0.28, 0.34, -0.24], [0.28, 0.34, 0.24], [-0.28, 0.34, 0.24]
				];
				return t.convex(w, pts, mass, opts);
			},
			halfUp: 0.34, halfTip: 0.35
		}
	};

	var SHAPE_KEYS = ['box', 'sphere', 'cylinder', 'cone', 'capsule', 'hull'];

	// 90deg about X as a normalized quaternion [x,y,z,w].
	var TIP_X = U.axisAngle(null, 1, 0, 0, Math.PI / 2);
	var CASES = [
		{ name: 'upright', rot: null, key: 'halfUp' },
		{ name: 'tipped', rot: TIP_X, key: 'halfTip' }
	];

	function makePairTest(botKey, topKey, cs) {
		var botDef = SHAPES[botKey], topDef = SHAPES[topKey];
		var testName = topKey + ' on ' + botKey + ' (' + cs.name + ')';
		var TOTAL = TOTAL_OVERRIDE[testName] || 260;

		Runner.test('shape-pairs', testName, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);

			var rot = cs.rot ? { rot: cs.rot } : {};
			var botHalf = botDef[cs.key];
			var topHalf = topDef[cs.key];

			// bottom body: origin at its resting half-height above the ground top (y=0)
			var bot = botDef.make(t, w, 3, U.withMat(Object.assign({
				pos: [0, botHalf + 0.02, 0], color: '#4a90d0'
			}, rot)));

			// top body: DROP_GAP above the bottom body's top surface
			var topY = botHalf + 0.04 + botHalf + DROP_GAP + topHalf;
			var top = topDef.make(t, w, 2, U.withMat(Object.assign({
				pos: [0, topY, 0], color: '#d07a4a'
			}, rot)));

			var lastTick = 0;
			var worstV = 0, worstVWho = '', worstVTick = 0;
			var worstW = 0, worstWWho = '', worstWTick = 0;
			var leftMap = false;
			// consecutive-tick rest hold: both bodies must stay under threshold for HOLD ticks
			// in a row, so a slow zero-crossing in an ongoing oscillation does NOT count as settled.
			var REST_V = 0.1, REST_W = 0.1, HOLD = 30;
			var restRun = 0, restRunMax = 0;

			t.onTick(function (world, tick) {
				lastTick = tick;
				var pair = [['bot', bot], ['top', top]];
				var allSlow = true;
				for (var i = 0; i < pair.length; i++) {
					var b = pair[i][1];
					var sv = U.speed(b), sw = U.spin(b);
					if (sv > worstV) { worstV = sv; worstVWho = pair[i][0]; worstVTick = tick; }
					if (sw > worstW) { worstW = sw; worstWWho = pair[i][0]; worstWTick = tick; }
					if (sv > REST_V || sw > REST_W) allSlow = false;
					var p = b.position;
					if (Math.abs(p.x) > 15 || Math.abs(p.z) > 15 || p.y < -3 || p.y > 25) leftMap = true;
				}
				if (allSlow) restRun++; else restRun = 0;
				if (restRun > restRunMax) restRunMax = restRun;
			});

			t.expect('neither body gains linear energy (|v| stays bounded)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  worst |v|=' + worstV.toFixed(2) };
				return {
					ok: worstV <= MAX_V,
					detail: 'worst |v|=' + worstV.toFixed(2) + ' (' + worstVWho + ' @ tick ' + worstVTick + ', limit ' + MAX_V + ')'
				};
			});

			t.expect('neither body gains angular energy (|w| stays bounded)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  worst |w|=' + worstW.toFixed(2) };
				return {
					ok: worstW <= MAX_W,
					detail: 'worst |w|=' + worstW.toFixed(2) + ' (' + worstWWho + ' @ tick ' + worstWTick + ', limit ' + MAX_W + ')'
				};
			});

			t.expect('neither body left the map', function () {
				if (lastTick < TOTAL) return false;
				return { ok: !leftMap, detail: leftMap ? 'a body left the map' : 'both stayed in bounds' };
			});

			t.expect('both bodies settle and stay settled (rest held ' + HOLD + '+ ticks)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL + '  best rest run=' + restRunMax };
				return {
					ok: restRun >= HOLD,
					detail: 'rest run at end=' + restRun + '/' + HOLD + ' (best=' + restRunMax + ')   ' +
						'bot |v|=' + U.speed(bot).toFixed(3) + ' |w|=' + U.spin(bot).toFixed(3) +
						'   top |v|=' + U.speed(top).toFixed(3) + ' |w|=' + U.spin(top).toFixed(3)
				};
			});

			t.simulate(w, TOTAL);
		}, {
			visual: true, steps: TOTAL, page: 'shape-pairs/' + cs.name,
			description:
				'Dynamic ' + topKey + ' dropped ~' + DROP_GAP + 'm onto a dynamic ' + botKey +
				' resting on the ground, both ' + cs.name + '. Both bodies are movable (mass > 0). ' +
				'Watches |v| (limit ' + MAX_V + ') and |w| (limit ' + MAX_W + ') every tick: the stack may ' +
				'topple, but a spike past those bounds is the narrowphase/solver creating energy from a ' +
				'contact. PASS: no spike, nothing leaves the map, both settle.'
		});
	}

	for (var bi = 0; bi < SHAPE_KEYS.length; bi++) {
		for (var ti = bi; ti < SHAPE_KEYS.length; ti++) {
			for (var ci = 0; ci < CASES.length; ci++) {
				makePairTest(SHAPE_KEYS[bi], SHAPE_KEYS[ti], CASES[ci]);
			}
		}
	}
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
