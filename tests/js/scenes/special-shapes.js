(function (Runner, U) {

	var TOTAL = 300;

	function Builder() { this.ref = { x: 0, y: 0, z: 0 }; }
	Builder.prototype.setReferencePoint = function (p) { this.ref = { x: p.x, y: p.y, z: p.z }; };
	Builder.prototype.createTriangle = function (idx, pos, a, b, c, forceFlip, doubleSided) {
		var ax = pos[a * 3], ay = pos[a * 3 + 1], az = pos[a * 3 + 2];
		var bx = pos[b * 3], by = pos[b * 3 + 1], bz = pos[b * 3 + 2];
		var cx = pos[c * 3], cy = pos[c * 3 + 1], cz = pos[c * 3 + 2];
		var ux = bx - ax, uy = by - ay, uz = bz - az;
		var vx = cx - ax, vy = cy - ay, vz = cz - az;
		var nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
		var gx = (ax + bx + cx) / 3, gy = (ay + by + cy) / 3, gz = (az + bz + cz) / 3;
		var dot = nx * (gx - this.ref.x) + ny * (gy - this.ref.y) + nz * (gz - this.ref.z);
		var eps = 1e-6; if (Math.abs(dot) < eps) dot = dot >= 0 ? eps : -eps;
		var v1, v2, v3;
		if (dot >= 0) { v1 = a; v2 = b; v3 = c; } else { v1 = a; v2 = c; v3 = b; }
		if (forceFlip) { var tmp = v2; v2 = v3; v3 = tmp; }
		idx.push(v1, v2, v3);
		if (doubleSided) idx.push(v1, v3, v2);
	};
	Builder.prototype.createQuad = function (idx, pos, a, b, c, d, forceFlip, doubleSided) {
		this.createTriangle(idx, pos, a, b, c, forceFlip, doubleSided);
		this.createTriangle(idx, pos, a, c, d, forceFlip, doubleSided);
	};

	function finish(v, idx) {
		var verts = [];
		for (var i = 0; i < v.length; i += 3) verts.push([v[i], v[i + 1], v[i + 2]]);
		return { verts: verts, faces: idx };
	}

	function trophy() {
		var b = new Builder(); var v = [], idx = [];
		var push = function (px, py, pz) { v.push(px, py, pz); return v.length / 3 - 1; };
		var box = function (cx, cy, cz, sx, sy, sz) {
			b.setReferencePoint({ x: cx, y: cy, z: cz });
			var hx = sx / 2, hy = sy / 2, hz = sz / 2, P = function (px, py, pz) { return push(cx + px, cy + py, cz + pz); };
			var a = P(-hx, -hy, -hz), bb = P(hx, -hy, -hz), cc = P(hx, hy, -hz), d = P(-hx, hy, -hz);
			var e = P(-hx, -hy, hz), f = P(hx, -hy, hz), g = P(hx, hy, hz), h = P(-hx, hy, hz);
			b.createQuad(idx, v, a, bb, cc, d); b.createQuad(idx, v, e, f, g, h);
			b.createQuad(idx, v, a, d, h, e); b.createQuad(idx, v, bb, f, g, cc);
			b.createQuad(idx, v, d, cc, g, h); b.createQuad(idx, v, a, e, f, bb);
		};
		box(0, 0.1, 0, 0.6, 0.2, 0.6);
		box(0, 0.3, 0, 0.4, 0.2, 0.4);
		box(0, 0.6, 0, 0.14, 0.4, 0.14);
		var starY = 1.1, Ro = 0.4, Ri = 0.16, pts = [];
		for (var i = 0; i < 10; i++) { var a = (i / 10) * Math.PI * 2 + Math.PI / 2, rr = i % 2 === 0 ? Ro : Ri; pts.push(push(Math.cos(a) * rr, starY + Math.sin(a) * rr, 0)); }
		var ctr = push(0, starY, 0);
		for (var j = 0; j < 10; j++) { var k = (j + 1) % 10; b.createTriangle(idx, v, ctr, pts[j], pts[k], false, true); }
		return finish(v, idx);
	}

	function rocket() {
		var b = new Builder(); var v = [], idx = [];
		var SIDES = 8, R = 0.34, BODY_H = 1.6, NOSE_H = 0.9;
		var push = function (px, py, pz) { v.push(px, py, pz); return v.length / 3 - 1; };
		var ring = function (yy, r) { var ids = []; for (var i = 0; i < SIDES; i++) { var a = (i / SIDES) * Math.PI * 2; ids.push(push(Math.cos(a) * r, yy, Math.sin(a) * r)); } return ids; };
		b.setReferencePoint({ x: 0, y: BODY_H / 2, z: 0 });
		var bot = ring(0, R), top = ring(BODY_H, R);
		for (var i = 0; i < SIDES; i++) { var j = (i + 1) % SIDES; b.createQuad(idx, v, bot[i], bot[j], top[j], top[i]); }

		var botCtr = push(0, 0, 0);
		for (var c = 0; c < SIDES; c++) { var cj = (c + 1) % SIDES; b.createTriangle(idx, v, botCtr, bot[c], bot[cj]); }
		b.setReferencePoint({ x: 0, y: BODY_H + NOSE_H / 2, z: 0 });
		var noseRing = ring(BODY_H, R);
		var apex = push(0, BODY_H + NOSE_H, 0);
		for (var n = 0; n < SIDES; n++) { var nj = (n + 1) % SIDES; b.createTriangle(idx, v, noseRing[n], noseRing[nj], apex); }
		for (var f = 0; f < 4; f++) {
			var fa = (f / 4) * Math.PI * 2, cx = Math.cos(fa), sz = Math.sin(fa);
			b.setReferencePoint({ x: cx * (R + 0.3), y: 0.3, z: sz * (R + 0.3) });
			var p1 = push(cx * R, 0.8, sz * R), p2 = push(cx * R, 0, sz * R), p3 = push(cx * (R + 0.6), 0, sz * (R + 0.6));
			b.createTriangle(idx, v, p1, p2, p3, false, true);
		}
		return finish(v, idx);
	}

	function specialShape(name, buildMesh, dropY, color, desc) {
		Runner.test('special shapes', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);
			var m = buildMesh();

			var body = U.meshBody(t, w, m.verts, m.faces, 2, { pos: [0, dropY, 0], color: color });

			t.expect('settles on the floor without exploding', U.settles(body, { maxY: dropY + 3 }));

			t.simulate(w, TOTAL);
		}, { visual: true, steps: TOTAL, page: 'special shapes', description: desc });
	}

	specialShape('trophy (stacked, offset COM)', trophy, 3, '#f2c744',
		"A trophy mesh — stacked base + tiers + a double-sided star — with its center of mass well above the " +
		"local origin. Fed raw into MeshShape, its inertia integral goes negative and the body launches. " +
		"Built simulable (recentered to COM + point-cloud inertia), PASS: it settles on the floor instead.");

	specialShape('rocket (octagon body + nose + fins)', rocket, 3, '#d9dde6',
		"A rocket mesh — octagonal body, nose cone, four double-sided fins — a thin, tall, offset-COM body, the " +
		"exact marginal case that drives MeshShape's inertia negative and makes it explode. Built " +
		"simulable (recentered to COM + point-cloud inertia), PASS: inertia stays positive and it settles.");
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
