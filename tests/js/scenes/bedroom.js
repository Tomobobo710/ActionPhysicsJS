(function (Runner, U) {

	// A static compound coffee table (annulus top with a hole, 4 legs) and a static compound office
	// chair, both concave MeshShape children in one CompoundShape. Mixed props drop onto both.

	var TOTAL = 360;
	var DROP_Y = 2.5;

	// mesh builders: each returns { verts:[[x,y,z]...], faces:[flat idx triples] }, outward-wound

	function boxMesh(hx, hy, hz) {
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
		return { verts: v, faces: f };
	}

	// Annular slab with a hole through the y axis: the coffee table top.
	function annulusMesh(rOuter, rInner, ty, seg) {
		var v = [], f = [];
		var hy = ty / 2;
		// per segment i: [outerBottom, outerTop, innerBottom, innerTop]
		for (var i = 0; i < seg; i++) {
			var a = (i / seg) * Math.PI * 2;
			var co = Math.cos(a), si = Math.sin(a);
			v.push([co * rOuter, -hy, si * rOuter]); // 4i+0
			v.push([co * rOuter, hy, si * rOuter]);  // 4i+1
			v.push([co * rInner, -hy, si * rInner]); // 4i+2
			v.push([co * rInner, hy, si * rInner]);  // 4i+3
		}
		function quad(p0, p1, p2, p3) { f.push(p0, p1, p2, p0, p2, p3); }
		for (i = 0; i < seg; i++) {
			var b0 = i * 4;
			var b1 = ((i + 1) % seg) * 4;
			var oB0 = b0 + 0, oT0 = b0 + 1, iB0 = b0 + 2, iT0 = b0 + 3;
			var oB1 = b1 + 0, oT1 = b1 + 1, iB1 = b1 + 2, iT1 = b1 + 3;
			// top ring (faces +y): outerTop -> innerTop
			quad(oT0, oT1, iT1, iT0);
			// bottom ring (faces -y)
			quad(oB0, iB0, iB1, oB1);
			// outer wall (faces outward)
			quad(oB0, oB1, oT1, oT0);
			// inner wall (faces inward, toward the hole axis)
			quad(iB0, iT0, iT1, iB1);
		}
		return { verts: v, faces: f };
	}

	// Concave 5-point star prism: the office-chair base.
	function starPrismMesh(rOuter, rInner, ty, points) {
		var v = [], f = [];
		var hy = ty / 2;
		var n = points * 2;
		// ring of n outline verts, alternating outer/inner radius, bottom then top
		for (var i = 0; i < n; i++) {
			var a = (i / n) * Math.PI * 2 + Math.PI / 2;
			var r = (i % 2 === 0) ? rOuter : rInner;
			v.push([Math.cos(a) * r, -hy, Math.sin(a) * r]); // 2i+0 bottom
			v.push([Math.cos(a) * r, hy, Math.sin(a) * r]);  // 2i+1 top
		}
		var cBot = v.push([0, -hy, 0]) - 1;
		var cTop = v.push([0, hy, 0]) - 1;
		function quad(p0, p1, p2, p3) { f.push(p0, p1, p2, p0, p2, p3); }
		for (i = 0; i < n; i++) {
			var j = (i + 1) % n;
			var b0 = i * 2, b1 = j * 2;
			// side wall
			quad(b0, b1, b1 + 1, b0 + 1);
			// top fan (faces +y)
			f.push(cTop, b0 + 1, b1 + 1);
			// bottom fan (faces -y)
			f.push(cBot, b1, b0);
		}
		return { verts: v, faces: f };
	}

	// Static CompoundShape from { mesh, pos } children at identity rotation, placed at `origin`.
	function staticFurniture(t, w, origin, parts, color) {
		var G = t.AP;
		var shape = new G.CompoundShape();
		var ident = new G.Quaternion(0, 0, 0, 1);
		for (var i = 0; i < parts.length; i++) {
			var p = parts[i];
			var verts = p.mesh.verts.map(function (q) { return new G.Vector3(q[0], q[1], q[2]); });
			var child = new G.MeshShape(verts, p.mesh.faces);
			shape.addChildShape(child, new G.Vector3(p.pos[0], p.pos[1], p.pos[2]), ident);
		}
		var body = new G.RigidBody(shape, 0); // static
		body.position.set(origin[0], origin[1], origin[2]);
		var mat = U.withMat({});
		body.friction = mat.friction;
		body.restitution = mat.restitution;
		body._color = color || '#8a7';
		w.addRigidBody(body);
		t.bodies.push(body);
		return body;
	}

	function coffeeTable(t, w, origin) {
		var topTy = 0.12, topOuter = 1.1, topInner = 0.45;
		var legHalf = 0.08, legLen = 0.7;
		var legY = -(legLen / 2) - topTy / 2;
		var legInset = topOuter - 0.25;
		var d = legInset / Math.SQRT2;
		var parts = [
			{ mesh: annulusMesh(topOuter, topInner, topTy, 28), pos: [0, 0, 0] },
			{ mesh: boxMesh(legHalf, legLen / 2, legHalf), pos: [d, legY, d] },
			{ mesh: boxMesh(legHalf, legLen / 2, legHalf), pos: [-d, legY, d] },
			{ mesh: boxMesh(legHalf, legLen / 2, legHalf), pos: [d, legY, -d] },
			{ mesh: boxMesh(legHalf, legLen / 2, legHalf), pos: [-d, legY, -d] }
		];
		return {
			body: staticFurniture(t, w, origin, parts, '#6b4f3a'),
			topY: origin[1] + topTy / 2,
			topOuter: topOuter, topInner: topInner
		};
	}

	function officeChair(t, w, origin) {
		var baseTy = 0.1, baseOuter = 0.85, baseInner = 0.18;
		var poleR = 0.06, poleLen = 0.75;
		var seatHx = 0.5, seatHy = 0.06, seatHz = 0.5;
		var backHx = 0.5, backHy = 0.45, backHz = 0.06;

		var baseY = baseTy / 2;                      // base rests on the floor
		var poleY = baseY + poleLen / 2;
		var seatY = baseY + poleLen + seatHy;
		var backY = seatY + backHy;
		var backZ = -(seatHz - backHz);              // seat back at the rear edge

		var parts = [
			{ mesh: starPrismMesh(baseOuter, baseInner, baseTy, 5), pos: [0, baseY, 0] },
			{ mesh: boxMesh(poleR, poleLen / 2, poleR), pos: [0, poleY, 0] },
			{ mesh: boxMesh(seatHx, seatHy, seatHz), pos: [0, seatY, 0] },
			{ mesh: boxMesh(backHx, backHy, backHz), pos: [0, backY, backZ] }
		];
		return {
			body: staticFurniture(t, w, origin, parts, '#3a4a5a'),
			seatY: origin[1] + seatY + seatHy
		};
	}

	Runner.test('bedroom', 'props settle on a compound coffee table (with a hole) and an office chair', function (t) {
		t.log('A static compound coffee table (annulus top with a hole + 4 legs) and a static compound');
		t.log('office chair (5-point star base -> pole -> seat -> seat back), each built from concave');
		t.log('MeshShape children. Mixed props are dropped onto both; they must settle, not tunnel or launch.');

		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);

		var table = coffeeTable(t, w, [-1.6, 0.76, 0]);
		var chair = officeChair(t, w, [1.8, 0.0, 0]);

		var props = [];
		function drop(body, name) { body._name = name; props.push(body); return body; }

		// Over the table: the cylinder is aimed at the hole and should fall through.
		drop(t.box(w, 0.16, 0.16, 0.16, 1, U.withMat({ pos: [-1.6 + 0.75, DROP_Y, 0], color: '#d9a441' })), 'box-on-ring');
		drop(t.cylinder(w, 0.13, 0.18, 1, U.withMat({ pos: [-1.6, DROP_Y, 0], color: '#8aa0c0' })), 'cyl-through-hole');
		drop(t.sphere(w, 0.14, 1, U.withMat({ pos: [-1.6 - 0.7, DROP_Y, 0.1], color: '#c0563a' })), 'ball-on-ring');

		drop(t.box(w, 0.18, 0.18, 0.18, 1, U.withMat({ pos: [1.8, DROP_Y, 0.22], color: '#7bbf6a' })), 'box-on-seat');
		drop(t.cone(w, 0.15, 0.22, 1, U.withMat({ pos: [2.02, DROP_Y, -0.22], color: '#c0873a' })), 'cone-on-seat');
		drop(t.sphere(w, 0.12, 1, U.withMat({ pos: [1.45, DROP_Y, 0.05], color: '#b04ac0' })), 'ball-on-seat');

		var boxOnTable = props[0], cylThruHole = props[1], ballOnTable = props[2];
		var boxOnChair = props[3], coneOnChair = props[4], ballOnChair = props[5];

		var tableTopSurfaceY = table.topY + 0.06;   // annulus half-thickness above table.topY
		var chairSeatSurfaceY = chair.seatY;        // already the seat's top face

		var lastTick = 0;
		var worstBelowFloor = -Infinity, worstBelowBody = '';
		var worstLateral = 0;
		var peakV = 0, peakVWho = '', peakW = 0, peakWWho = '';
		var coneMaxV = 0, coneMaxVTick = 0, coneMaxW = 0, coneMaxWTick = 0;
		var boxChairMaxV = 0, boxChairMaxVTick = 0, boxChairMaxW = 0, boxChairMaxWTick = 0;
		t.onTick(function (world, tick) {
			lastTick = tick;
			for (var i = 0; i < props.length; i++) {
				var p = props[i].position;
				var below = -0.5 - p.y; // ground top is y=0; -0.5 is well under any resting prop
				if (below > worstBelowFloor) { worstBelowFloor = below; worstBelowBody = props[i]._name; }
				var lat = Math.max(Math.abs(p.x), Math.abs(p.z));
				if (lat > worstLateral) worstLateral = lat;
				var sv = U.speed(props[i]), sw = U.spin(props[i]);
				if (sv > peakV) { peakV = sv; peakVWho = props[i]._name; }
				if (sw > peakW) { peakW = sw; peakWWho = props[i]._name; }
			}
			var csv = U.speed(coneOnChair), csw = U.spin(coneOnChair);
			if (csv > coneMaxV) { coneMaxV = csv; coneMaxVTick = tick; }
			if (csw > coneMaxW) { coneMaxW = csw; coneMaxWTick = tick; }
			var bsv = U.speed(boxOnChair), bsw = U.spin(boxOnChair);
			if (bsv > boxChairMaxV) { boxChairMaxV = bsv; boxChairMaxVTick = tick; }
			if (bsw > boxChairMaxW) { boxChairMaxW = bsw; boxChairMaxWTick = tick; }
		});

		// At rest, and origin within `tol` of (surface + half-height): touching, not floating or sunk.
		function restsOn(body, surfaceY, halfH, tol) {
			return function () {
				if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
				var y = body.position.y, sv = U.speed(body), sw = U.spin(body);
				var target = surfaceY + halfH;
				var atRest = sv < 0.15 && sw < 0.15;
				var onIt = Math.abs(y - target) < tol;
				return {
					ok: atRest && onIt,
					detail: 'y=' + y.toFixed(3) + ' target≈' + target.toFixed(3) + ' (±' + tol + ')  |v|=' +
						sv.toFixed(3) + ' |w|=' + sw.toFixed(3)
				};
			};
		}

		t.expect('no prop tunnelled through the floor', function () {
			if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
			return { ok: worstBelowFloor < 1.5, detail: 'worst dip=' + worstBelowFloor.toFixed(2) + ' (' + worstBelowBody + ')' };
		});

		t.expect('no prop slid off the map', function () {
			if (lastTick < TOTAL) return false;
			return { ok: worstLateral < 12, detail: 'max lateral=' + worstLateral.toFixed(2) };
		});

		t.expect('every prop comes to rest by the end', function () {
			if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
			var moving = [], maxSp = 0, who = '';
			for (var i = 0; i < props.length; i++) {
				var sp = U.speed(props[i]), sw = U.spin(props[i]);
				if (sp > 0.15 || sw > 0.15) moving.push(props[i]._name);
				if (sp > maxSp) { maxSp = sp; who = props[i]._name; }
			}
			return { ok: moving.length === 0, detail: moving.length ? 'still moving: ' + moving.join(', ') : 'all at rest (max |v|=' + maxSp.toFixed(3) + ' ' + who + ')' };
		});


		t.expect('cube on the table rests on the table',
			restsOn(boxOnTable, tableTopSurfaceY, 0.16, 0.12));

		t.expect('sphere on the table rests on the table',
			restsOn(ballOnTable, tableTopSurfaceY, 0.14, 0.12));

		t.expect('cylinder dropped over the hole rests on the floor', function () {
			if (lastTick < TOTAL) return false;
			var y = cylThruHole.position.y, sv = U.speed(cylThruHole), sw = U.spin(cylThruHole);
			// On an end -> y≈0.18, on its side -> y≈0.13; accept either, below the table top.
			var onFloor = y > 0.05 && y < 0.30;
			return {
				ok: sv < 0.15 && sw < 0.15 && onFloor && y < table.topY,
				detail: 'cyl y=' + y.toFixed(3) + ' (floor rest 0.13-0.18, table top y=' + table.topY.toFixed(3) +
					')  |v|=' + sv.toFixed(3) + ' |w|=' + sw.toFixed(3)
			};
		});

		// Nothing in the cylinder's path applies a lateral force, so it should land where it dropped.
		t.expect('cylinder falls straight through the hole (final x/z near drop x/z)', function () {
			if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
			var dx = cylThruHole.position.x - (-1.6);
			var dz = cylThruHole.position.z - 0;
			var drift = Math.sqrt(dx * dx + dz * dz);
			return {
				ok: drift < 0.03,
				detail: 'drift=' + drift.toFixed(3) + ' m  (dx=' + dx.toFixed(3) + ' dz=' + dz.toFixed(3) +
					', limit 0.03)'
			};
		});

		t.expect('cube on the chair does not explode (|v|<12, |w|<25 all run)', function () {
			if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
			return {
				ok: boxChairMaxV < 12 && boxChairMaxW < 25,
				detail: 'cube peak |v|=' + boxChairMaxV.toFixed(2) + ' (@t' + boxChairMaxVTick + ')  peak |w|=' +
					boxChairMaxW.toFixed(2) + ' (@t' + boxChairMaxWTick + ')  final y=' + boxOnChair.position.y.toFixed(3)
			};
		});

		t.expect('cone on the chair does not explode (|v|<8, |w|<15 all run)', function () {
			if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
			return {
				ok: coneMaxV < 8 && coneMaxW < 15,
				detail: 'cone peak |v|=' + coneMaxV.toFixed(2) + ' (@t' + coneMaxVTick + ')  peak |w|=' +
					coneMaxW.toFixed(2) + ' (@t' + coneMaxWTick + ')  final y=' + coneOnChair.position.y.toFixed(3)
			};
		});

		t.expect('sphere on the chair rests on the chair seat',
			restsOn(ballOnChair, chairSeatSurfaceY, 0.12, 0.14));

		t.expect('nothing explodes (no prop exceeds |v|=12 or |w|=25 at any tick)', function () {
			if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
			return {
				ok: peakV < 12 && peakW < 25,
				detail: 'peak |v|=' + peakV.toFixed(2) + ' (' + peakVWho + ')  peak |w|=' + peakW.toFixed(2) + ' (' + peakWWho + ')'
			};
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'bedroom',
		description:
			"A bedroom scene: a static CompoundShape coffee table whose top is a concave annulus (a hole " +
			"straight through the middle) on four box legs, and a static CompoundShape office chair — a " +
			"5-point star-prism base, a pole, a seat, and a seat back. Both furniture pieces are compounds " +
			"of concave MeshShape children. Six mixed props (boxes, spheres, a cylinder, a cone) are dropped " +
			"onto them. PASS: nothing tunnels through the floor or slides off the map, every prop settles, " +
			"and the cylinder aimed at the table's hole drops through it to a resting spot below the table top."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
