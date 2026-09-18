(function (Runner, U) {

	// The bedroom office chair on its own, with only the two props that misbehave there. Each prop
	// runs alone or together, so a failure names its own cause: alone both settle, together the cone
	// lands on the box, is deflected across the seat, and spikes going off the seat's corner.

	var TOTAL = 240;
	var DROP_Y = 2.5;

	function boxMesh(hx, hy, hz) {
		return {
			verts: [
				[-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
				[-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]
			],
			faces: [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
				3, 7, 6, 3, 6, 2, 1, 2, 6, 1, 6, 5, 0, 4, 7, 0, 7, 3]
		};
	}

	// 5-point star prism (the chair base), same builder bedroom.js uses.
	function starPrismMesh(rOuter, rInner, ty, points) {
		var v = [], f = [], hy = ty / 2, n = points * 2, i;
		for (i = 0; i < n; i++) {
			var a = (i / n) * Math.PI * 2 + Math.PI / 2;
			var r = (i % 2 === 0) ? rOuter : rInner;
			v.push([Math.cos(a) * r, -hy, Math.sin(a) * r]);
			v.push([Math.cos(a) * r, hy, Math.sin(a) * r]);
		}
		var cBot = v.push([0, -hy, 0]) - 1;
		var cTop = v.push([0, hy, 0]) - 1;
		function quad(p0, p1, p2, p3) { f.push(p0, p1, p2, p0, p2, p3); }
		for (i = 0; i < n; i++) {
			var j = (i + 1) % n, b0 = i * 2, b1 = j * 2;
			quad(b0, b1, b1 + 1, b0 + 1);
			f.push(cTop, b0 + 1, b1 + 1);
			f.push(cBot, b1, b0);
		}
		return { verts: v, faces: f };
	}

	// Static CompoundShape chair at `origin`. Returns the seat's top-face Y and its x/z extents.
	function officeChair(t, w, origin) {
		var G = t.AP;
		var baseTy = 0.1, baseOuter = 0.85, baseInner = 0.18;
		var poleR = 0.06, poleLen = 0.75;
		var seatHx = 0.5, seatHy = 0.06, seatHz = 0.5;
		var backHx = 0.5, backHy = 0.45, backHz = 0.06;

		var baseY = baseTy / 2;
		var poleY = baseY + poleLen / 2;
		var seatY = baseY + poleLen + seatHy;
		var backY = seatY + backHy;
		var backZ = -(seatHz - backHz);

		var parts = [
			{ mesh: starPrismMesh(baseOuter, baseInner, baseTy, 5), pos: [0, baseY, 0] },
			{ mesh: boxMesh(poleR, poleLen / 2, poleR), pos: [0, poleY, 0] },
			{ mesh: boxMesh(seatHx, seatHy, seatHz), pos: [0, seatY, 0] },
			{ mesh: boxMesh(backHx, backHy, backHz), pos: [0, backY, backZ] }
		];

		var shape = new G.CompoundShape();
		var ident = new G.Quaternion(0, 0, 0, 1);
		for (var i = 0; i < parts.length; i++) {
			var p = parts[i];
			var verts = p.mesh.verts.map(function (q) { return new G.Vector3(q[0], q[1], q[2]); });
			shape.addChildShape(new G.MeshShape(verts, p.mesh.faces),
				new G.Vector3(p.pos[0], p.pos[1], p.pos[2]), ident);
		}
		var body = new G.RigidBody(shape, 0);
		body.position.set(origin[0], origin[1], origin[2]);
		var mat = U.withMat({});
		body.friction = mat.friction;
		body.restitution = mat.restitution;
		body._color = '#3a4a5a';
		w.addRigidBody(body);
		t.bodies.push(body);
		return {
			body: body,
			seatTopY: origin[1] + seatY + seatHy,
			seatMinX: origin[0] - seatHx, seatMaxX: origin[0] + seatHx,
			seatMinZ: origin[2] - seatHz, seatMaxZ: origin[2] + seatHz
		};
	}

	// One case. `which` selects the props: 'box', 'cone', or 'both'.
	function chairCase(name, which, opts) {
		Runner.test('chair', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);
			var chair = officeChair(t, w, [1.8, 0.0, 0]);

			var box = null, cone = null;
			if (which === 'box' || which === 'both') {
				box = t.box(w, 0.18, 0.18, 0.18, 1, U.withMat({ pos: [1.8, DROP_Y, 0.22], color: '#7bbf6a' }));
			}
			if (which === 'cone' || which === 'both') {
				cone = t.cone(w, 0.15, 0.22, 1, U.withMat({ pos: [2.02, DROP_Y, -0.22], color: '#c0873a' }));
			}

			var lastTick = 0;
			var peak = {};   // name -> { v, vTick, w, wTick }
			function track(b, nm) {
				if (!b) return;
				peak[nm] = { v: 0, vTick: 0, w: 0, wTick: 0 };
			}
			track(box, 'box'); track(cone, 'cone');

			t.onTick(function (world, tick) {
				lastTick = tick;
				[[box, 'box'], [cone, 'cone']].forEach(function (pair) {
					var b = pair[0], nm = pair[1];
					if (!b) return;
					var sv = U.speed(b), sw = U.spin(b), pk = peak[nm];
					if (sv > pk.v) { pk.v = sv; pk.vTick = tick; }
					if (sw > pk.w) { pk.w = sw; pk.wTick = tick; }
				});
			});

			// The cone may topple, so height is checked against the shape's lowest point via the
			// support function - a tilted cone's AABB corner hangs below the cone and reads as sunk.
			var lowestPointY = function (b) {
				var inv = new t.AP.Quaternion(b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w).invert();
				var dL = new t.AP.Vector3(0, -1, 0);
				inv.transformVectorInPlace(dL);
				var sL = new t.AP.Vector3();
				b.shape.supportInto(sL, dL);
				b.rotation.transformVectorInPlace(sL);
				return sL.y + b.position.y;
			};
			if (opts.restsOnSeat) {
				opts.restsOnSeat.forEach(function (nm) {
					var b = nm === 'box' ? box : cone;
					t.expect(nm + ' settles at rest on the seat', function () {
						if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
						var p = b.position;
						var onSeat = p.x > chair.seatMinX && p.x < chair.seatMaxX &&
							p.z > chair.seatMinZ && p.z < chair.seatMaxZ;
						var lowY = lowestPointY(b);
						var sitsOnTop = Math.abs(lowY - chair.seatTopY) < 0.03;
						var still = U.speed(b) < 0.05 && U.spin(b) < 0.1;
						return {
							ok: onSeat && sitsOnTop && still,
							detail: 'pos(' + p.x.toFixed(3) + ',' + p.y.toFixed(3) + ',' + p.z.toFixed(3) + ')' +
								' lowestY=' + lowY.toFixed(3) + ' seatTop=' + chair.seatTopY.toFixed(3) +
								' onSeat=' + onSeat + ' |v|=' + U.speed(b).toFixed(3) + ' |w|=' + U.spin(b).toFixed(3)
						};
					});
				});
			}

			Object.keys(peak).forEach(function (nm) {
				var lim = opts.spinLimit;
				t.expect(nm + ' never spins past |w|=' + lim, function () {
					if (lastTick < TOTAL) return { ok: false, detail: 'tick ' + lastTick + '/' + TOTAL };
					var pk = peak[nm];
					return {
						ok: pk.w <= lim,
						detail: 'peak |w|=' + pk.w.toFixed(2) + ' @t' + pk.wTick +
							'  peak |v|=' + pk.v.toFixed(2) + ' @t' + pk.vTick + ' (limit ' + lim + ')'
					};
				});
			});

			t.simulate(w, TOTAL);
		}, { visual: true, steps: TOTAL, page: 'chair', description: opts.desc });
	}

	chairCase('box alone lands on the chair seat and settles', 'box', {
		restsOnSeat: ['box'], spinLimit: 2,
		desc: 'A box dropped on the middle of the seat, chair only. It should land flat and stop dead - no spin at all.'
	});

	chairCase('cone alone lands on the chair seat and settles', 'cone', {
		restsOnSeat: ['cone'], spinLimit: 15,
		desc: 'A cone dropped near the back-right of the seat, chair only. It grazes the top front corner of the seat back on the way down, then must settle on the seat.'
	});

	chairCase('box and cone together do not gain energy', 'both', {
		spinLimit: 15,
		desc: 'Both props, as bedroom.js drops them. The cone lands on the box and is knocked sideways across the seat - that part is fair. It must not pick up runaway spin going over the seat corner.'
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
