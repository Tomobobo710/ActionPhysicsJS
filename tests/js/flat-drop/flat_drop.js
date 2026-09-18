(function (Runner, U) {

	var DROP_Y = 2.5;
	var TICKS = 300;
	var MAX_DRIFT = 0.03;
	var MAX_LATERAL_V = 0.01;

	var TIP_X = U.axisAngle(null, 1, 0, 0, Math.PI / 2);

	var SPOTS = [
		{ name: 'at the origin', x: 0, z: 0 },
		{ name: 'offset in x', x: -1.6, z: 0 },
		{ name: 'offset in z', x: 0, z: -1.6 },
		{ name: 'offset in x and z', x: -1.6, z: -1.6 }
	];

	function speed(b) { var v = b.linear_velocity; return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

	function dropTest(shapeLabel, page, build, restY, tipped) {
		SPOTS.forEach(function (spot) {
			var label = shapeLabel + ' — dropped ' + spot.name;
			Runner.test('contacts/flat-drop', label, function (t) {
				var w = t.makeWorld({ gravity: -9.8 });
				U.ground(t, w);

				var opts = { pos: [spot.x, DROP_Y, spot.z], color: '#8aa0c0' };
				if (tipped) opts.rot = TIP_X;
				var body = build(t, w, U.withMat(opts));

				var lastTick = 0;
				var worstX = 0, worstZ = 0, worstTick = 0;
				var peakLateralSpeed = 0, peakLateralSpeedTick = 0;
				var minPoints = Infinity, pointsAtRest = 0;

				t.onTick(function (world, tick) {
					lastTick = tick;

					var dx = Math.abs(body.position.x - spot.x);
					var dz = Math.abs(body.position.z - spot.z);
					if (dx > worstX) { worstX = dx; worstTick = tick; }
					if (dz > worstZ) { worstZ = dz; worstTick = tick; }

					var lv = body.linear_velocity;
					var latV = Math.sqrt(lv.x * lv.x + lv.z * lv.z);
					if (latV > peakLateralSpeed) { peakLateralSpeed = latV; peakLateralSpeedTick = tick; }

					for (var m = world.narrowphase.manifolds.first; m; m = m.next_manifold) {
						if (m.bodyA !== body && m.bodyB !== body) continue;
						if (!m.points.length) continue;
						if (m.points.length < minPoints) minPoints = m.points.length;
						pointsAtRest = m.points.length;
					}
				});

				t.expect('lands at rest height (y = ' + restY + ')', function () {
					if (lastTick < TICKS) return false;
					var y = body.position.y;
					return { ok: Math.abs(y - restY) < 0.02, detail: 'y=' + y.toFixed(5) + ' (expected ' + restY + ')' };
				});

				t.expect('comes to rest', function () {
					if (lastTick < TICKS) return false;
					return { ok: speed(body) < 0.01, detail: '|v|=' + speed(body).toFixed(5) };
				});

				t.expect('never travels sideways (drift < ' + MAX_DRIFT + ' m on either axis)', function () {
					if (lastTick < TICKS) return false;
					var worst = Math.max(worstX, worstZ);
					return {
						ok: worst < MAX_DRIFT,
						detail: 'driftX=' + worstX.toFixed(4) + ' driftZ=' + worstZ.toFixed(4) + ' m @ tick ' + worstTick +
							'  final=(' + body.position.x.toFixed(4) + ',' + body.position.z.toFixed(4) +
							') dropped at (' + spot.x + ',' + spot.z + ')'
					};
				});

				t.expect('never gains sideways speed from a level floor', function () {
					if (lastTick < TICKS) return false;
					return {
						ok: peakLateralSpeed < MAX_LATERAL_V,
						detail: 'peak lateral |v|=' + peakLateralSpeed.toFixed(4) + ' m/s @ tick ' + peakLateralSpeedTick
					};
				});

				t.expect('manifold width while touching', function () {
					if (lastTick < TICKS) return false;
					return {
						ok: true,
						detail: 'narrowest=' + (minPoints === Infinity ? 'never touched' : minPoints) +
							' point(s), at rest=' + pointsAtRest
					};
				});

				t.simulate(w, TICKS);
			}, {
				visual: true, steps: TICKS, page: page + '-' + (spot.x === 0 && spot.z === 0 ? 'origin' : (spot.z === 0 ? 'x' : (spot.x === 0 ? 'z' : 'xz'))),
				description:
					shapeLabel + ' dropped from y=' + DROP_Y + ' at (' + spot.x + ',' + spot.z + ') onto a plain box ' +
					'floor, no rotation applied in flight and nothing pushing it sideways. It must land under its ' +
					'drop point. PASS: drift under ' + MAX_DRIFT + ' m on both axes. Running the same drop at the ' +
					'origin and offset separates a shape that drifts wherever it lands from one that only drifts ' +
					'away from the origin.'
			});
		});
	}

	dropTest('sphere', 'flat-drop-sphere',
		function (t, w, o) { return t.sphere(w, 0.14, 1, o); }, 0.14, false);

	dropTest('capsule (upright)', 'flat-drop-capsule',
		function (t, w, o) { return t.capsule(w, 0.13, 0.5, 1, o); }, 0.25, false);

	dropTest('capsule (on its side)', 'flat-drop-capsule-side',
		function (t, w, o) { return t.capsule(w, 0.13, 0.5, 1, o); }, 0.13, true);

	dropTest('box', 'flat-drop-box',
		function (t, w, o) { return t.box(w, 0.16, 0.16, 0.16, 1, o); }, 0.16, false);

	dropTest('cylinder (on its cap)', 'flat-drop-cylinder',
		function (t, w, o) { return t.cylinder(w, 0.13, 0.18, 1, o); }, 0.18, false);

	dropTest('cylinder (on its side)', 'flat-drop-cylinder-side',
		function (t, w, o) { return t.cylinder(w, 0.13, 0.18, 1, o); }, 0.13, true);

	dropTest('cone (base down)', 'flat-drop-cone',
		function (t, w, o) { return t.cone(w, 0.15, 0.22, 1, o); }, 0.22, false);

	dropTest('mesh box (12 triangles)', 'flat-drop-mesh',
		function (t, w, o) {
			var h = 0.16;
			var v = [
				[-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h],
				[-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h]
			];
			var f = [
				0, 2, 1, 0, 3, 2,
				4, 5, 6, 4, 6, 7,
				0, 1, 5, 0, 5, 4,
				3, 7, 6, 3, 6, 2,
				1, 2, 6, 1, 6, 5,
				0, 4, 7, 0, 7, 3
			];
			return t.mesh(w, v, f, 1, o);
		}, 0.16, false);

	dropTest('convex hull (cube-shaped)', 'flat-drop-hull',
		function (t, w, o) {
			var pts = [
				[-0.16, -0.16, -0.16], [0.16, -0.16, -0.16], [0.16, -0.16, 0.16], [-0.16, -0.16, 0.16],
				[-0.16, 0.16, -0.16], [0.16, 0.16, -0.16], [0.16, 0.16, 0.16], [-0.16, 0.16, 0.16]
			];
			return t.convex(w, pts, 1, o);
		}, 0.16, false);

}(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
));
