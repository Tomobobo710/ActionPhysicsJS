(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;

	var BOX = [[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1],[-1,1,-1],[1,1,-1],[1,1,1],[-1,1,1]];
	var PYR = [[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1],[0,2,0]];
	var EPS = 1e-5;
	var DESC = "A ConvexShape is built from a cloud of vertices (any convex hull). From just the points " +
		"it has to compute the right volume, the right center of mass (needed for correct physics), and " +
		"the right support points (needed for collision). Checked on a unit cube (volume 8, centered) and " +
		"a pyramid (volume 4, center of mass pulled down toward the wide base at y=-0.25).";

	function makeConvex(t, verts, color) {
		var vs = verts.map(function (v) { return new V(v[0], v[1], v[2]); });
		return t.loneBody(new AP.ConvexShape(vs), { mass: 0, color: color });
	}

	Runner.test('collision/convexshape', 'box: volume = 8', function (t) {
		var b = makeConvex(t, BOX, '#45B7D1');
		t.check(b.shape.volume(), 8, EPS, 'unit cube volume = 8');
	}, { visual: true, steps: 0, page: 'convexshape', description: DESC });

	Runner.test('collision/convexshape', 'box: center of mass at origin', function (t) {
		var b = makeConvex(t, BOX, '#45B7D1');
		t.check(b.shape.computeMassData().centerOfMass.distanceTo(new V(0, 0, 0)), 0, EPS, 'center of mass at (0,0,0)');
	}, { visual: true, steps: 0, page: 'convexshape', description: DESC });

	Runner.test('collision/convexshape', 'pyramid: volume = 4', function (t) {
		var b = makeConvex(t, PYR, '#F4D35E');
		t.check(b.shape.volume(), 4, EPS, 'pyramid volume = 4');
	}, { visual: true, steps: 0, page: 'convexshape', description: DESC });

	Runner.test('collision/convexshape', 'pyramid: center of mass at (0,-0.25,0)', function (t) {
		var b = makeConvex(t, PYR, '#F4D35E');
		t.check(b.shape.computeMassData().centerOfMass.distanceTo(new V(0, -0.25, 0)), 0, EPS, 'center of mass pulled toward the base');
	}, { visual: true, steps: 0, page: 'convexshape', description: DESC });

	function supportTest(name, verts, color, dir, expected, label) {
		Runner.test('collision/convexshape', name, function (t) {
			var b = makeConvex(t, verts, color);
			var p = new V();
			b.findSupportPoint(new V(dir[0], dir[1], dir[2]), p);
			t.support = { dir: dir, point: [p.x, p.y, p.z] };
			t.check(p.distanceTo(new V(expected[0], expected[1], expected[2])), 0, EPS, label);
		}, { visual: true, steps: 0, page: 'convexshape', description: DESC });
	}

	supportTest('box: support point (corner)', BOX, '#45B7D1', [-1, -1, -1], [-1, -1, -1], 'support toward a corner');

	supportTest('box: support point (face)', BOX, '#45B7D1', [1, 0, 0], [1, -1, -1], 'support toward a face (first tied max in point order)');
	supportTest('box: support point (edge)', BOX, '#45B7D1', [0, 1, 1], [1, 1, 1], 'support toward an edge');
	supportTest('pyramid: support point (corner)', PYR, '#F4D35E', [-1, -1, -1], [-1, -1, -1], 'support toward a base corner');
	supportTest('pyramid: support point (apex)', PYR, '#F4D35E', [0, 1, 0], [0, 2, 0], 'support straight up = the apex');
	supportTest('pyramid: support point (edge)', PYR, '#F4D35E', [0, -1, 1], [1, -1, 1], 'support toward a base edge');

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
