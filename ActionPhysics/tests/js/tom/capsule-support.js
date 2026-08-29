// Tom's Suite — CAPSULE SUPPORT POINTS.
// findSupportPoint(direction, out) for CapsuleShape — the query GJK/EPA relies on. Chandler's
// support-points suite covers sphere/box/cylinder/cone but NOT the capsule, which is exactly where two
// bugs lived and crashed GJK:
//   • Equatorial rows ([1,0,0], [-1,0,0], [0,0,±1]): the support must sit at y=0 (barrel equator). A
//     two-way y-sign wrongly snapped these to the top cap ring (y = +cylinder_half_height), making every
//     horizontal support coplanar → flat simplex.
//   • Zero-direction row ([0,0,0]): must return a finite point, not NaN from a 0/0 normalization.
// Base/translated/rotated configs, same data-driven form and EPSILON as Chandler's support-points.
//
// CapsuleShape(1.5, 5): radius 1.5, cylinder_half_height = (5 - 2*1.5)/2 = 1.0. Cap tips at y = ±2.5.
// Deliberately large (a tall barrel, not a near-sphere) so the support points are easy to see in the viewer.
(function (Runner) {
	Runner.suite('tom');
	var EPS = 0.01;

	function cap(G) { return new G.CapsuleShape(1.5, 5); }

	// [group, name, dir[3], expected[3], pos[3]|null, rot[4]|null]
	var ROWS = [
		// Base — the equator MUST be y=0 (guards the three-way-sign fix), caps at y=±2.5.
		['Base','equator +x',[1,0,0],[1.5,0,0],null,null],
		['Base','equator -x',[-1,0,0],[-1.5,0,0],null,null],
		['Base','equator +z',[0,0,1],[0,0,1.5],null,null],
		['Base','equator -z',[0,0,-1],[0,0,-1.5],null,null],
		['Base','top cap',[0,1,0],[0,2.5,0],null,null],
		['Base','bottom cap',[0,-1,0],[0,-2.5,0],null,null],
		['Base','diagonal +x+y',[1,1,0],[1.0607, 2.0607, 0],null,null],
		['Base','diagonal +x-y-z',[1,-1,-1],[0.866,-1.866,-0.866],null,null],
		// Zero direction — no "most extreme" point exists; must be finite (guards the 0/0 NaN fix).
		['Base','zero direction',[0,0,0],[0,0,0],null,null],
		// Translated — support tracks world position.
		['Translated','equator +x',[1,0,0],[7.5,9,0],[6,9,0],null],
		['Translated','top cap',[0,1,0],[6,11.5,0],[6,9,0],null],
		// Rotated 90deg about x: barrel axis (local y) maps to world +z in this engine's rotation
		// convention. A world +y query inverse-rotates to local (0,~0,-1) - horizontal in local space
		// (the equator, not either cap center - see CapsuleShape.supportInto's own comment), which
		// rotates back to world y=segmentHalfLength, z=0.
		['Rotated','+y after 90x',[0,1,0],[0,1.5,0],[0,0,0],[1,0,0,1]],
		// Translated + rotated.
		['Translated & Rotated','+y after 90x',[0,1,0],[6,10.5,0],[6,9,0],[1,0,0,1]]
	];

	var DESC = "Support points for CapsuleShape — the farthest point in a direction, the query GJK relies " +
		"on. Chandler's support-points suite skips the capsule; these fill it in. The equator rows assert " +
		"the horizontal support sits at y=0 (a two-way y-sign wrongly snapped it to the top cap, making " +
		"every horizontal support coplanar and crashing EPA); the zero-direction row asserts a finite point " +
		"rather than a 0/0 NaN. Checked within 0.01 across base, translated, and rotated configs.";

	ROWS.forEach(function (r) {
		var group = r[0], name = r[1], dir = r[2], exp = r[3], pos = r[4], rot = r[5];
		Runner.test('capsule support', group + ': ' + name, function (t) {
			var opts = { mass: 1 };
			if (pos) opts.pos = pos;   // null -> origin
			if (rot) opts.rot = rot;   // null -> identity
			var b = t.loneBody(cap(t.AP), opts);
			var out = t.vec(0, 0, 0);
			b.findSupportPoint(t.vec(dir[0], dir[1], dir[2]), out);
			t.support = { dir: dir, point: [out.x, out.y, out.z] };
			t.check(out.distanceTo(t.vec(exp[0], exp[1], exp[2])), 0, EPS,
				'support point in direction (' + dir.join(',') + ') is (' + exp.map(function (n) { return +n.toFixed(3); }).join(',') + ')');
		}, { visual: true, steps: 0, page: 'capsule support', description: DESC });
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner);
