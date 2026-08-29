// The 385-box pyramid - the real stability stress target.
//
// This is a HARD scene and legitimately FAILS several assertions as of the last full run (drift,
// spin, no settling by tick 1200). Real signal, not a test bug - do not soften this file or raise
// solver settings to pass it; fix the engine, or record why it doesn't yet.
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;

	var SIZE = 10;
	var TICKS = 1200;
	var BOX_H = 2.0;      // full height of a box (half-extent 1)
	var LAYER_GAP = 2.2;  // vertical spacing between layers at spawn

	// Columns sit 2.6 apart and each layer is inset 1.2, so a box spans [x-1, x+1] and overlaps the
	// two boxes below it by 0.80 on one side and 0.60 on the other, per axis. Every box is therefore
	// supported at spawn - nothing bridges a gap. Drifting 0.60 toward the weaker side removes that
	// support entirely, so 0.60 is the point where the stack stops being held up by what it started on.
	var SUPPORT_LOSS = 0.6;
	var MAX_ROTATION = 8;

	// Local-space corners, so checks measure each box's real world extent, not just its center.
	var CORNERS = (function () {
		var c = [];
		for (var x = -1; x <= 1; x += 2)
			for (var y = -1; y <= 1; y += 2)
				for (var z = -1; z <= 1; z += 2) c.push([x, y, z]);
		return c;
	})();

	Runner.test('collision/pyramid', 'a 385-box pyramid on a ground plane stays a pyramid (1200 ticks, engine defaults)', function (t) {
		var w = t.makeWorld({ gravity: -9.8 });
		// Wide thin static box as ground - the pile never travels far enough to need an infinite plane.
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });

		var boxes = [];
		for (var i = 0; i < SIZE; i++) {
			for (var j = 0; j < SIZE - i; j++) {
				for (var k = 0; k < SIZE - i; k++) {
					var x = 2 * j * 1.3 - SIZE + i * 1.2,
						y = i * LAYER_GAP + 1,
						z = 2 * k * 1.3 - SIZE + i * 1.2;
					// Engine-default materials throughout - a bare does-it-hold scene, not a sweep.
					var b = t.box(w, 1, 1, 1, 1, { pos: [x, y, z], color: '#B08968' });
					boxes.push({ body: b, layer: i, x0: x, y0: y, z0: z });
				}
			}
		}

		var tmpV = t.vec(0, 0, 0);
		var ticks = 0;

		var anyNonFinite = false;
		var worstLatDriftEver = 0;

		function totalRotationDegrees(body) {
			var wq = Math.abs(body.rotation.w);
			if (wq > 1) wq = 1;
			return 2 * Math.acos(wq) * 180 / Math.PI;
		}

		function tiltDegrees(body) {
			tmpV.set(0, 1, 0);
			body.rotation.transformVectorInPlace(tmpV);
			var d = Math.max(-1, Math.min(1, tmpV.y));
			return Math.acos(d) * 180 / Math.PI;
		}

		function lowestCornerOf(body) {
			var lo = Infinity;
			for (var c = 0; c < 8; c++) {
				tmpV.set(CORNERS[c][0], CORNERS[c][1], CORNERS[c][2]);
				body.rotation.transformVectorInPlace(tmpV);
				tmpV.addInPlace(body.position);
				if (tmpV.y < lo) lo = tmpV.y;
			}
			return lo;
		}

		var layerSum = [], layerCount = [], layerMean = [];
		function computeLayerMeans() {
			var L;
			for (L = 0; L < SIZE; L++) { layerSum[L] = 0; layerCount[L] = 0; }
			for (var n = 0; n < boxes.length; n++) {
				var y = boxes[n].body.position.y;
				if (!isFinite(y)) continue;
				layerSum[boxes[n].layer] += y; layerCount[boxes[n].layer]++;
			}
			for (L = 0; L < SIZE; L++) layerMean[L] = layerCount[L] ? layerSum[L] / layerCount[L] : null;
		}

		t.onTick(function (world, tick) {
			ticks = tick;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], p = o.body.position;
				if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) { anyNonFinite = true; continue; }
				var dx = p.x - o.x0, dz = p.z - o.z0;
				var lat = Math.sqrt(dx * dx + dz * dz);
				if (lat > worstLatDriftEver) worstLatDriftEver = lat;
			}
		});

		var latAtHalf = null;
		t.onTick(function (world, tick) {
			if (tick === Math.floor(TICKS / 2)) latAtHalf = worstLatDriftEver;
		});

		var tiltAtHalf = null;
		function worstTiltNow() {
			var worst = 0;
			for (var n = 0; n < boxes.length; n++) {
				if (!isFinite(boxes[n].body.position.y)) continue;
				var tl = tiltDegrees(boxes[n].body);
				if (tl > worst) worst = tl;
			}
			return worst;
		}
		t.onTick(function (world, tick) {
			if (tick === Math.floor(TICKS / 2)) tiltAtHalf = worstTiltNow();
		});

		t.expect('every box stays finite (no numerical blowup)', function () {
			if (ticks < TICKS) return false;
			return { ok: !anyNonFinite, detail: anyNonFinite ? 'at least one box went NaN/Infinity' : 'all ' + boxes.length + ' boxes finite at every tick' };
		});

		t.expect('no box rests inside the floor (penetration < 0.012)', function () {
			if (ticks < TICKS) return false;
			var pen = 0;
			for (var n = 0; n < boxes.length; n++) {
				var body = boxes[n].body;
				if (!isFinite(body.position.y)) continue;
				var d = -lowestCornerOf(body);
				if (d > pen) pen = d;
			}
			return { ok: pen < 0.012, detail: 'deepest resting floor penetration=' + pen.toFixed(4) };
		});

		t.expect('layers rest one box height apart (spacing within 0.012 of ' + BOX_H + ')', function () {
			if (ticks < TICKS) return false;
			computeLayerMeans();
			var tightest = Infinity, tightestAt = -1, widest = -Infinity, widestAt = -1, L;
			for (L = 1; L < SIZE; L++) {
				if (layerMean[L] == null || layerMean[L - 1] == null) continue;
				var gap = layerMean[L] - layerMean[L - 1];
				if (gap < tightest) { tightest = gap; tightestAt = L; }
				if (gap > widest) { widest = gap; widestAt = L; }
			}
			if (tightestAt < 0) return false;
			return {
				ok: tightest >= BOX_H - 0.012 && widest <= BOX_H + 0.012,
				detail: 'tightest=' + tightest.toFixed(4) + ' (layer ' + tightestAt + '), widest=' + widest.toFixed(4) + ' (layer ' + widestAt + '), box height ' + BOX_H
			};
		});

		t.expect('no box is left extruded above its spawn height (rise < 0.01)', function () {
			if (ticks < TICKS) return false;
			var worst = 0;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], y = o.body.position.y;
				if (!isFinite(y)) continue;
				var r = y - o.y0;
				if (r > worst) worst = r;
			}
			return { ok: worst < 0.01, detail: 'worst resting rise=' + worst.toFixed(4) };
		});

		t.expect('lateral creep stops once settled (second half adds < 0.05)', function () {
			if (ticks < TICKS) return false;
			if (latAtHalf == null) return false;
			var added = worstLatDriftEver - latAtHalf;
			return {
				ok: added < 0.05,
				detail: 'drift ' + latAtHalf.toFixed(3) + ' by half-time, ' + worstLatDriftEver.toFixed(3) + ' at end (+' + added.toFixed(4) + ')'
			};
		});

		t.expect('no box slides off a supporting neighbour (drift < ' + SUPPORT_LOSS + ')', function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = null, lost = 0;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], p = o.body.position;
				if (!isFinite(p.x)) continue;
				var dx = p.x - o.x0, dz = p.z - o.z0;
				var l = Math.sqrt(dx * dx + dz * dz);
				if (l >= SUPPORT_LOSS) lost++;
				if (l > worst) { worst = l; which = o; }
			}
			return {
				ok: worst < SUPPORT_LOSS,
				detail: 'worst drift=' + worst.toFixed(3) + (which ? ' (layer ' + which.layer + ')' : '') +
					', ' + lost + ' boxes past the ' + SUPPORT_LOSS + ' support-loss limit'
			};
		});

		t.expect('the pile as a whole does not migrate (median drift < ' + (SUPPORT_LOSS / 10) + ')', function () {
			if (ticks < TICKS) return false;
			var drifts = [];
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], p = o.body.position;
				if (!isFinite(p.x)) continue;
				var dx = p.x - o.x0, dz = p.z - o.z0;
				drifts.push(Math.sqrt(dx * dx + dz * dz));
			}
			if (!drifts.length) return false;
			drifts.sort(function (a, b) { return a - b; });
			var median = drifts[Math.floor(drifts.length / 2)];
			var p90 = drifts[Math.floor(drifts.length * 0.9)];
			return {
				ok: median < SUPPORT_LOSS / 10,
				detail: 'median drift=' + median.toFixed(3) + ', p90=' + p90.toFixed(3) + ', support-loss limit ' + SUPPORT_LOSS
			};
		});

		t.expect('no box has spun in place (every box under ' + MAX_ROTATION + ' degrees of total rotation)', function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = null, over = 0;
			for (var n = 0; n < boxes.length; n++) {
				var body = boxes[n].body;
				if (!isFinite(body.position.x)) continue;
				var rot = totalRotationDegrees(body);
				if (rot > MAX_ROTATION) over++;
				if (rot > worst) { worst = rot; which = boxes[n]; }
			}
			return {
				ok: over === 0,
				detail: 'worst=' + worst.toFixed(2) + ' degrees' + (which ? ' (layer ' + which.layer + ')' : '') +
					', ' + over + ' boxes past ' + MAX_ROTATION + ' degrees'
			};
		});

		t.expect('no box is left tipped (every box under 0.75 degrees)', function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = -1, over = 0;
			for (var n = 0; n < boxes.length; n++) {
				if (!isFinite(boxes[n].body.position.y)) continue;
				var tl = tiltDegrees(boxes[n].body);
				if (tl > 0.75) over++;
				if (tl > worst) { worst = tl; which = boxes[n].layer; }
			}
			return {
				ok: worst < 0.75,
				detail: 'worst tilt=' + worst.toFixed(2) + ' degrees' + (which >= 0 ? ' (layer ' + which + ')' : '') + ', ' + over + ' boxes past 0.75 degrees'
			};
		});

		t.expect('tilt stops growing once settled (second half adds < 0.25 degrees)', function () {
			if (ticks < TICKS) return false;
			if (tiltAtHalf == null) return false;
			var now = worstTiltNow();
			var added = now - tiltAtHalf;
			return {
				ok: added < 0.25,
				detail: 'worst tilt ' + tiltAtHalf.toFixed(2) + ' deg at half-time, ' + now.toFixed(2) + ' at end (' + (added >= 0 ? '+' : '') + added.toFixed(2) + ')'
			};
		});

		t.expect('every layer rests level (no box more than 0.015 off its layer mean)', function () {
			if (ticks < TICKS) return false;
			computeLayerMeans();
			var worst = 0, which = null;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], y = o.body.position.y, mean = layerMean[o.layer];
				if (!isFinite(y) || mean == null) continue;
				var d = Math.abs(y - mean);
				if (d > worst) { worst = d; which = o; }
			}
			return {
				ok: worst < 0.015,
				detail: 'worst deviation from layer mean=' + worst.toFixed(4) + (which ? ' (layer ' + which.layer + ')' : '')
			};
		});

		t.expect('the pyramid still holds its shape at tick ' + TICKS, function () {
			if (ticks < TICKS) return false;
			var worst = 0, which = null;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], y = o.body.position.y;
				if (!isFinite(y)) continue;
				var allowed = 0.25 * o.layer + 0.25;
				var off = Math.abs(y - o.y0) - allowed;
				if (off > worst) { worst = off; which = o; }
			}
			return {
				ok: worst <= 0,
				detail: worst <= 0
					? 'every box within its settling allowance'
					: 'worst box exceeds allowance by ' + worst.toFixed(3) + (which ? ' (layer ' + which.layer + ', spawn y=' + which.y0.toFixed(1) + ' now y=' + which.body.position.y.toFixed(2) + ')' : '')
			};
		});

		t.expect('no upper-layer box ever falls to ground level (layer >= 2 stays above y = 2.5)', function () {
			if (ticks < TICKS) return false;
			var fallen = 0, worst = null;
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n];
				if (o.layer < 2) continue;
				var y = o.body.position.y;
				if (!isFinite(y)) continue;
				if (y < 2.5) {
					fallen++;
					if (!worst || y < worst.body.position.y) worst = o;
				}
			}
			return {
				ok: fallen === 0,
				detail: fallen === 0
					? 'no upper-layer box reached the floor'
					: fallen + ' upper-layer boxes fell to ground level' + (worst ? ' (worst: layer ' + worst.layer + ' spawned at y=' + worst.y0.toFixed(1) + ', now y=' + worst.body.position.y.toFixed(2) + ')' : '')
			};
		});

		t.expect('the whole pile has come to rest (every box slower than 0.001 units/s)', function () {
			if (ticks < TICKS) return false;
			var fastest = 0;
			for (var n = 0; n < boxes.length; n++) {
				var v = boxes[n].body.linear_velocity;
				if (!isFinite(v.x)) continue;
				var sp = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				if (sp > fastest) fastest = sp;
			}
			return { ok: fastest < 0.001, detail: 'fastest box at tick ' + TICKS + ' = ' + fastest.toFixed(3) + ' units/s' };
		});

		t.expect('nothing is still rotating at the end (angular speed < 0.0025 rad/s)', function () {
			if (ticks < TICKS) return false;
			var fastest = 0, which = null;
			for (var n = 0; n < boxes.length; n++) {
				var a = boxes[n].body.angular_velocity;
				if (!isFinite(a.x)) continue;
				var sp = Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
				if (sp > fastest) { fastest = sp; which = boxes[n]; }
			}
			return {
				ok: fastest < 0.0025,
				detail: 'fastest rotation=' + fastest.toFixed(5) + ' rad/s' + (which ? ' (layer ' + which.layer + ')' : '')
			};
		});

		t.simulate(w, TICKS);
	}, {
		visual: true, steps: TICKS, page: 'pyramid',
		description:
			"The 385-box pyramid at true engine defaults. A pyramid on solid ground is a stable equilibrium: " +
			"layers must hold their heights, boxes must neither sink into each other nor get extruded " +
			"upward, nothing may creep or tip, no upper-layer box may reach the floor, and the whole pile " +
			"must be at rest by tick 1200."
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
