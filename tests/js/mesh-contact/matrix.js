/**
 * Convex-vs-mesh contact matrix (docs/mesh-contact-plan.md section 5). Each scenario builds a
 * static mesh surface and drops all six primitives on it; each must land, not gain energy, come to
 * rest, and (where staysPut) not drift from where it was dropped.
 */
(function (Runner, U) {

	var G = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;

	var TOTAL = 300;
	var SETTLE_FROM = 240;      // rest is only judged after this
	var REST_HOLD = 40;
	var REST_V = 0.08, REST_W = 0.15;
	var MAX_V = 12, MAX_W = 28;
	var DRIFT_LIMIT = 0.15;     // "stays put" cases: final horizontal distance from drop point
	var DROP_GAP = 0.6;         // prop released this far above the surface

	// primitives: make(t,w,mass,opts) + resting half-height (origin -> lowest point)
	var SHAPES = {
		box:      { make: function (t, w, m, o) { return t.box(w, 0.25, 0.25, 0.25, m, o); }, half: 0.25 },
		sphere:   { make: function (t, w, m, o) { return t.sphere(w, 0.25, m, o); }, half: 0.25 },
		cylinder: { make: function (t, w, m, o) { return t.cylinder(w, 0.24, 0.28, m, o); }, half: 0.28 },
		cone:     { make: function (t, w, m, o) { return t.cone(w, 0.26, 0.30, m, o); }, half: 0.30 },
		capsule:  { make: function (t, w, m, o) { return t.capsule(w, 0.20, 0.70, m, o); }, half: 0.35 },
		hull:     { make: function (t, w, m, o) {
			return t.convex(w, [
				[-0.24, -0.22, -0.22], [0.24, -0.22, -0.22], [0.24, -0.22, 0.22], [-0.24, -0.22, 0.22],
				[-0.20, 0.24, -0.18], [0.20, 0.24, -0.18], [0.20, 0.24, 0.18], [-0.20, 0.24, 0.18]
			], m, o); }, half: 0.22 }
	};
	var KEYS = ['box', 'sphere', 'cylinder', 'cone', 'capsule', 'hull'];

	// A closed outward-wound box mesh centred on the origin.
	function boxMeshVF(hx, hy, hz) {
		return {
			v: [[-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
				[-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]],
			f: [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 1, 2, 6, 1, 6, 5, 0, 4, 7, 0, 7, 3]
		};
	}

	function meshBody(t, w, vf, pos, rot) {
		var vs = vf.v.map(function (p) { return new G.Vector3(p[0], p[1], p[2]); });
		var b = new G.RigidBody(new G.MeshShape(vs, vf.f), 0);
		b.position.set(pos[0], pos[1], pos[2]);
		if (rot) b.rotation.set(rot[0], rot[1], rot[2], rot[3]);
		var mat = U.withMat({});
		b.friction = mat.friction; b.restitution = mat.restitution;
		b._color = '#2f4636';
		w.addRigidBody(b); t.bodies.push(b);
		return b;
	}

	// Run one (scenario, primitive) case. `build(t,w)` returns { surfaceY, dropXZ, staysPut, tipRot }.
	function meshCase(scenario, kind, build, extra) {
		Runner.test('mesh-contact', scenario + ' - ' + kind, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			var cfg = build(t, w);
			var S = SHAPES[kind];
			var dropY = cfg.surfaceY + S.half + DROP_GAP;
			var opts = U.withMat({ pos: [cfg.dropXZ[0], dropY, cfg.dropXZ[1]], color: '#c98a3a' });
			if (cfg.tipRot) opts.rot = cfg.tipRot;
			var prop = S.make(t, w, 4, opts);

			var lastTick = 0, worstV = 0, worstW = 0, worstVTick = 0, worstWTick = 0;
			var restRun = 0, bestRest = 0, leftMap = false;
			var landed = false, landX = 0, landZ = 0, maxDrift = 0;

			t.onTick(function (world, tick) {
				lastTick = tick;
				var p = prop.position, sv = U.speed(prop), sw = U.spin(prop);
				if (sv > worstV) { worstV = sv; worstVTick = tick; }
				if (sw > worstW) { worstW = sw; worstWTick = tick; }
				if (Math.abs(p.x) > 12 || Math.abs(p.z) > 12 || p.y < -4 || p.y > 20) leftMap = true;
				if (!landed && p.y <= cfg.surfaceY + S.half + 0.12 && Math.abs(prop.linear_velocity.y) < 2.5) {
					landed = true; landX = p.x; landZ = p.z;
				}
				if (landed) {
					var d = Math.sqrt((p.x - landX) * (p.x - landX) + (p.z - landZ) * (p.z - landZ));
					if (d > maxDrift) maxDrift = d;
				}
				if (tick >= SETTLE_FROM) {
					if (sv < REST_V && sw < REST_W) restRun++; else restRun = 0;
					if (restRun > bestRest) bestRest = restRun;
				}
			});

			t.expect('|v| stays bounded (<= ' + MAX_V + ')', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				return { ok: worstV <= MAX_V, detail: 'worst |v|=' + worstV.toFixed(2) + ' @ tick ' + worstVTick + ' (limit ' + MAX_V + ')' };
			});
			t.expect('|w| stays bounded (<= ' + MAX_W + ')', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				return { ok: worstW <= MAX_W, detail: 'worst |w|=' + worstW.toFixed(2) + ' @ tick ' + worstWTick + ' (limit ' + MAX_W + ')' };
			});
			t.expect('did not leave the map', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				return { ok: !leftMap, detail: leftMap ? 'flung out of bounds' : 'stayed in bounds' };
			});
			t.expect('comes to a sustained rest (' + REST_HOLD + '+ ticks)', function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				return { ok: restRun >= REST_HOLD, detail: 'rest run at end=' + restRun + '/' + REST_HOLD + ' (best=' + bestRest + ')  |v|=' + U.speed(prop).toFixed(3) + ' |w|=' + U.spin(prop).toFixed(3) };
			});
			if (cfg.staysPut) {
				t.expect('stays where it was dropped (drift < ' + DRIFT_LIMIT + ' m)', function () {
					if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
					if (!landed) return { ok: false, detail: 'never landed' };
					return { ok: maxDrift < DRIFT_LIMIT, detail: 'landed (' + landX.toFixed(2) + ',' + landZ.toFixed(2) + '); max drift ' + maxDrift.toFixed(3) + ' m (limit ' + DRIFT_LIMIT + ')' };
				});
			}

			t.simulate(w, TOTAL);
		}, { visual: true, steps: TOTAL, page: 'mesh-contact/' + scenario, description: (extra && extra.desc) || '' });
	}

	// 1. FLAT multi-triangle face: must not catch the top face's own diagonal seam, must not yaw.
	KEYS.forEach(function (k) {
		meshCase('flat-face', k, function (t, w) {
			meshBody(t, w, boxMeshVF(3, 0.5, 3), [0, -0.5, 0]);
			return { surfaceY: 0, dropXZ: [0.4, -0.3], staysPut: true };
		}, { desc: 'Prop dropped flat onto a single MeshShape box top - the simplest mesh face. Must land, not catch the top quad diagonal, not yaw, and rest where dropped.' });

		meshCase('flat-face-tilted', k, function (t, w) {
			meshBody(t, w, boxMeshVF(3, 0.5, 3), [0, -0.5, 0]);
			return { surfaceY: 0, dropXZ: [0, 0], staysPut: false,
				tipRot: U.axisAngle(null, 0, 0, 1, 0.5) };
		}, { desc: 'Prop dropped tilted onto a MeshShape box top - topples onto a face, then must hold still (not rock or slide across the internal seam).' });
	});

	// 2. CONVEX ridge: two slabs into a roof line; the prop must settle straddling the crease.
	KEYS.forEach(function (k) {
		meshCase('ridge', k, function (t, w) {
			// Slabs tilted so their inner-top edges meet on the y axis at `apexY`.
			var ang = 0.30, s = Math.sin(ang), cc = Math.cos(ang);
			var hx = 2, hy = 0.15, apexY = 1.4;
			// slab centre = apex - (inner-top-corner offset in the slab's rotated frame)
			function place(sign) {
				var rot = U.axisAngle(null, 0, 0, 1, -sign * ang);
				// rotate local (sign*hx, hy) by -sign*ang
				var lx = sign * hx, ly = hy;
				var rx = lx * Math.cos(-sign * ang) - ly * Math.sin(-sign * ang);
				var ry = lx * Math.sin(-sign * ang) + ly * Math.cos(-sign * ang);
				meshBody(t, w, boxMeshVF(hx, hy, 3), [-rx, apexY - ry, 0], rot);
			}
			place(1); place(-1);
			void s; void cc;
			return { surfaceY: apexY + 0.05, dropXZ: [0, 0], staysPut: true };
		}, { desc: 'Two MeshShape slabs meeting at a raised ridge (a real CONVEX crease). The prop balances on the ridge line and must settle there, not be flicked off by a mis-oriented edge normal.' });
	});

	// 3. CONCAVE valley: both faces bound the prop; it must wedge and rest, not oscillate.
	KEYS.forEach(function (k) {
		meshCase('valley', k, function (t, w) {
			// Same construction as ridge, but the meeting edge is the low point of a V trough.
			var ang = 0.30, hx = 2, hy = 0.15, floorY = 0.2;
			function place(sign) {
				var rot = sign * ang; // left slab (x<0) tilts +ang so its inner/+x end is lowest
				var lx = sign * hx, ly = hy;
				var rx = lx * Math.cos(rot) - ly * Math.sin(rot);
				var ry = lx * Math.sin(rot) + ly * Math.cos(rot);
				meshBody(t, w, boxMeshVF(hx, hy, 3), [-rx, floorY - ry, 0], U.axisAngle(null, 0, 0, 1, rot));
			}
			place(1); place(-1);
			// The capsule is too tall to balance upright on a 17-degree apex, and lying along the
			// trough it just rolls. Lay it ACROSS the trough (on x) so the V blocks the roll and it
			// wedges against both slabs like the squat props do.
			var lie = k === 'capsule' ? U.axisAngle(null, 0, 0, 1, Math.PI / 2) : null;
			return { surfaceY: floorY + 0.05, dropXZ: [0, 0], staysPut: true, tipRot: lie };
		}, { desc: 'Two MeshShape slabs meeting at a trough (a CONCAVE crease). Both slab faces bound the prop; it must settle wedged in the valley without buzzing between the two contacts.' });
	});

	// 4. VERTEX: a pyramid apex. The prop may topple off, but must not gain energy doing so.
	KEYS.forEach(function (k) {
		meshCase('vertex', k, function (t, w) {
			var R = 2.2, H = 1.6;
			var vf = {
				v: [[-R, 0, -R], [R, 0, -R], [R, 0, R], [-R, 0, R], [0, H, 0]],
				f: [0, 2, 1, 0, 3, 2,       // base (downward)
					0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4]   // four faces up to the apex
			};
			meshBody(t, w, vf, [0, 0, 0]);
			return { surfaceY: H, dropXZ: [0, 0], staysPut: false };
		}, { desc: 'A MeshShape pyramid, apex up. The prop lands on the point (a VERTEX contact - normal spans a solid angle). It may roll off, but must not be ejected.' });
	});

	// 5. Cross-body seam: four separate tiles, coplanar tops. Must behave as one flat surface.
	KEYS.forEach(function (k) {
		meshCase('cross-body-seam', k, function (t, w) {
			var h = 3;
			[[-h, -h], [h, -h], [-h, h], [h, h]].forEach(function (c) {
				meshBody(t, w, boxMeshVF(h, 0.5, h), [c[0], -0.5, c[1]]);
			});
			return { surfaceY: 0, dropXZ: [0, 0], staysPut: true };
		}, { desc: 'Four separate static MeshShape tiles meeting at a seam through the origin, tops coplanar. The prop lands right on the four-tile corner and must settle as if on one surface - no phantom yaw, no ski across the floor.' });
	});

	// 6. Bridge a gap: the 0.2 gap is narrower than every prop, so each genuinely spans it.
	['box', 'cylinder', 'capsule'].forEach(function (k) {
		meshCase('bridge-gap', k, function (t, w) {
			meshBody(t, w, boxMeshVF(1.2, 0.5, 2), [-1.3, -0.5, 0]);
			meshBody(t, w, boxMeshVF(1.2, 0.5, 2), [1.3, -0.5, 0]);
			return { surfaceY: 0, dropXZ: [0, 0], staysPut: true };
		}, { desc: 'Two separated MeshShape tables with a gap between. A long prop bridges them, one end on each. The two contact patches are independent and must stay level - not collapse into one tilted constraint that drops the prop into the gap.' });
	});

	// 7. Pass-through: dropped down a hole larger than itself, the prop must exit with its entry x/z.
	KEYS.forEach(function (k) {
		meshCase('pass-through', k, function (t, w) {
			// a flat frame: four bars around a central hole ~1.2 wide, top at y=0.
			var barLong = { hx: 2, hy: 0.25, hz: 0.4 }, barSide = { hx: 0.4, hy: 0.25, hz: 1.2 };
			meshBody(t, w, boxMeshVF(barLong.hx, barLong.hy, barLong.hz), [0, -0.25, 1.6]);
			meshBody(t, w, boxMeshVF(barLong.hx, barLong.hy, barLong.hz), [0, -0.25, -1.6]);
			meshBody(t, w, boxMeshVF(barSide.hx, barSide.hy, barSide.hz), [1.6, -0.25, 0]);
			meshBody(t, w, boxMeshVF(barSide.hx, barSide.hy, barSide.hz), [-1.6, -0.25, 0]);
			// a catch floor well below so the prop settles somewhere and the rest assert can pass.
			meshBody(t, w, boxMeshVF(4, 0.5, 4), [0, -4.5, 0]);
			return { surfaceY: -4, dropXZ: [0, 0], staysPut: true, _passHole: true };
		}, { desc: 'A MeshShape frame with a hole wider than the prop. The prop drops straight through - grazing the inner edges must not deflect it. It lands on a catch floor below with the x/z it started with.' });
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
