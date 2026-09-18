(function (Runner, U) {

	// Contacts remove energy. A body dropped onto static ground can never end a run with more
	// mechanical energy than it settled with, and must come to rest ON the surface rather than
	// sunk into it. This is the plan's "no energy injection" rule measured directly, on the
	// smallest scene that shows it: one prop, one static tile.
	//
	// The energy is the TOTAL, rotation included, so a prop that trades spin for height on landing is
	// measured as the dissipative event it is rather than as a gain. Measured this way a tumbling drop
	// is clean, and what the cases pin down is that it STAYS clean: no injection at the landing, no
	// slow burrow into the tile afterwards, and no residual drift once it has settled.

	// The lowest point of a body in world space: its shape's support point along -Y, rotated by the
	// body's orientation and moved to its position. Independent of the solver's contact bookkeeping,
	// so a stale or missing manifold cannot make it report a penetration that is not there.
	var _down = null, _low = null;
	function lowestPointY(t, body) {
		var AP = t.AP;
		if (!_down) { _down = new AP.Vector3(0, -1, 0); _low = new AP.Vector3(); }
		body.shape.supportInto(_low, _down);
		body.rotation.transformVectorInPlace(_low);
		return _low.y + body.position.y;
	}

	var TOTAL = 600;
	var SETTLE_FROM = 120;      // energy is only judged once the prop has landed
	var MAX_GAIN = 0.05;        // joules a settled prop may regain (solver residual only)
	var MAX_SINK = 0.02;        // how far into the surface it may end up

	var TILE = 200 / 55;        // same tile size as the perf scene
	var TILE_Y = 0.427;

	function tileMesh(t, w, hx, hz, y) {
		var G = t.AP;
		var v = [[-hx, y, -hz], [hx, y, -hz], [hx, y, hz], [-hx, y, hz]]
			.map(function (p) { return new G.Vector3(p[0], p[1], p[2]); });
		var b = new G.RigidBody(new G.MeshShape(v, [0, 2, 1, 0, 3, 2]), 0);
		var mat = U.withMat({});
		b.friction = mat.friction; b.restitution = mat.restitution;
		b._color = '#2f4636';
		w.addRigidBody(b); t.bodies.push(b);
		return b;
	}

	// `spin` is the angular velocity the prop is released with; null drops it flat.
	function energyCase(name, groundKind, spin) {
		Runner.test('solver/energy', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });

			if (groundKind === 'mesh') tileMesh(t, w, TILE / 2, TILE / 2, TILE_Y);
			else t.box(w, TILE / 2, 0.5, TILE / 2, 0, U.withMat({ pos: [0, TILE_Y - 0.5, 0], color: '#2f4636' }));

			var HX = 0.594, HY = 0.602, HZ = 0.535, MASS = 11.01;
			var prop = t.box(w, HX, HY, HZ, MASS, U.withMat({
				pos: [0, TILE_Y + 3, 0], color: '#c98a3a'
			}));
			if (spin) prop.angular_velocity.set(spin[0], spin[1], spin[2]);

			// Mechanical energy referenced to the tile surface, INCLUDING rotational kinetic energy.
			// Rotation cannot be dropped from an energy budget. A tumbling prop's contacts trade spin for
			// height as it rocks off a corner onto its lowest face, and with the spin term missing that
			// legitimate trade - the total falling the whole way, spin spent on height it does not get
			// back - reads as a prop that regained energy. The rule under test is that contacts only ever
			// REMOVE energy, and total mechanical energy is the quantity that states it. Leaving rotation
			// out is not what keeps a spin-up visible either: one spun up from nothing raises the total
			// as well, and 'comes to rest' asserts |w| < 0.05 outright.
			function energy() {
				var v = prop.linear_velocity, w = prop.angular_velocity;
				var I = prop.inertiaTensor, q = prop.rotation;
				// Rot KE = 1/2 w.(R I R^T w). Computed as 1/2 (R^T w).(I_local (R^T w)) - identical, but
				// it needs one rotation instead of two, and the body-frame inertia is diagonal anyway.
				var qx = q.x, qy = q.y, qz = q.z, qw = q.w;
				var R = [
					1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
					2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
					2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)];
				var wx = [w.x, w.y, w.z], u = [0, 0, 0], iu = [0, 0, 0];
				for (var j = 0; j < 3; j++) {
					for (var k = 0; k < 3; k++) u[j] += R[k * 3 + j] * wx[k];
				}
				var M = [I.e00, I.e01, I.e02, I.e10, I.e11, I.e12, I.e20, I.e21, I.e22];
				for (var a = 0; a < 3; a++) {
					for (var b = 0; b < 3; b++) iu[a] += M[a * 3 + b] * u[b];
				}
				var rotKE = 0.5 * (u[0] * iu[0] + u[1] * iu[1] + u[2] * iu[2]);
				return 0.5 * MASS * (v.x * v.x + v.y * v.y + v.z * v.z) + rotKE +
					MASS * 9.8 * (prop.position.y - TILE_Y);
			}

			var lastTick = 0, minE = Infinity, worstGain = 0, gainTick = 0, peakW = 0;
			t.onTick(function (world, tick) {
				lastTick = tick;
				var sw = U.spin(prop);
				if (sw > peakW) peakW = sw;
				if (tick < SETTLE_FROM) return;
				var e = energy();
				if (e < minE) minE = e;
				var gain = e - minE;
				if (gain > worstGain) { worstGain = gain; gainTick = tick; }
			});

			t.expect('gains no energy once settled (< ' + MAX_GAIN + ' J)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				return {
					ok: worstGain < MAX_GAIN,
					detail: 'regained ' + worstGain.toFixed(4) + ' J @t' + gainTick +
						'  (peak |w| during run ' + peakW.toFixed(2) + ')'
				};
			});

			t.expect('rests ON the surface, not sunk into it', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				// Measured as POSE vs SURFACE, not "centre of mass at an assumed rest height". Dropped
				// tumbling the box settles onto whichever face is lowest-energy - here the HZ face, which
				// leaves its centre 0.067 m below where the HY face would have put it, and which is the
				// height it was dropped on. That is a legal rest, not a sink, and judging the centre
				// against a fixed TILE_Y + HY reads the difference between those two faces as penetration.
				// What must never happen is any PART of the box being below the tile, so take the box's
				// lowest point - its support point along -Y, in world space - and compare that. Nothing
				// here assumes which face it lands on, so it holds for a tumbling drop and a flat one
				// alike.
				var lowest = lowestPointY(t, prop);
				var sink = TILE_Y - lowest;
				return {
					ok: sink < MAX_SINK,
					detail: 'lowest point y=' + lowest.toFixed(5) + ' (surface ' + TILE_Y.toFixed(4) +
						')  sunk ' + sink.toFixed(5) + ' m (limit ' + MAX_SINK + ')'
				};
			});

			t.expect('comes to rest', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				return {
					ok: U.speed(prop) < 0.05 && U.spin(prop) < 0.05,
					detail: '|v|=' + U.speed(prop).toFixed(4) + ' |w|=' + U.spin(prop).toFixed(4)
				};
			});

			t.simulate(w, TOTAL);
		}, {
			visual: true, steps: TOTAL, page: 'solver/energy',
			description: 'A box dropped ' + (spin ? 'tumbling' : 'flat') + ' onto a single static ' +
				groundKind + ' tile. Contacts only remove energy, so once it has landed it must never ' +
				'regain any, and it must settle on the surface rather than sinking through it.'
		});
	}

	var SPIN = [2.1, 0.7, 1.4];
	energyCase('box dropped flat on a box tile gains no energy', 'box', null);
	energyCase('box dropped flat on a mesh tile gains no energy', 'mesh', null);
	energyCase('box dropped tumbling on a box tile gains no energy', 'box', SPIN);
	energyCase('box dropped tumbling on a mesh tile gains no energy', 'mesh', SPIN);

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
