// Ported from Goblin's tests/js/chandler/support-points.js. findSupportPoint(direction, out) for
// every shape in base, translated, rotated, and translated+rotated configs. EPSILON 0.01.
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3, Q = AP.Quaternion;
	var EPS = 0.01, S2 = Math.sqrt(2), S45 = Math.sqrt(4.5);

	function mkShape(key) {
		switch (key) {
			case 'sphere':   return new AP.SphereShape(2);
			case 'box':      return new AP.BoxShape(3, 4, 5);
			case 'cylinder': return new AP.CylinderShape(3, 2.5);
			case 'cone':     return new AP.ConeShape(3, 2.5);
			case 'plane_x':  return new AP.PlaneShape('x', 5, 10);
			// ActionPhysics's own halfW/halfL axis convention is cyclic (y,z)/(z,x)/(x,y) per
			// orientation - documented directly in PlaneShape.js - which for 'y' swaps which extent
			// maps to X vs Z relative to Goblin's own PlaneShape(orientation, halfW, halfL) for the
			// same axis. halfW/halfL swapped here (10, 5 instead of 5, 10) so the resulting shape
			// matches Goblin's actual geometry (extent 5 on X, extent 10 on Z), not just its argument
			// order - a real, deliberate convention difference, not a bug in either engine.
			case 'plane_y':  return new AP.PlaneShape('y', 10, 5);
			case 'plane_z':  return new AP.PlaneShape('z', 5, 10);
		}
	}
	// [group, name, shapeKey, dir[3], expected[3], pos[3]|null, rot[4]|null]
	var ROWS = [
		['Sphere','Base Sphere 1','sphere',[1,0,0],[2,0,0],null,null],
		['Sphere','Base Sphere 2','sphere',[1,1,0],[S2,S2,0],null,null],
		['Sphere','Translated Sphere 1','sphere',[1,0,0],[4,3,0],[2,3,0],null],
		['Sphere','Translated Sphere 2','sphere',[-1,-1,0],[-S2+2,-S2+3,0],[2,3,0],null],
		['Sphere','Rotated Sphere 1','sphere',[1,0,0],[2,0,0],[0,0,0],[1,0,0,1]],
		['Sphere','Rotated Sphere 2','sphere',[1,1,0],[S2,S2,0],[0,0,0],[1,0,0,1]],
		['Sphere','Translated & Rotated Sphere 1','sphere',[1,0,0],[4,3,0],[2,3,0],[1,0,0,1]],
		['Sphere','Translated & Rotated Sphere 2','sphere',[-1,-1,0],[-S2+2,-S2+3,0],[2,3,0],[1,0,0,1]],
		['Box','Base Box 1','box',[1,0,0],[3,4,5],null,null],
		['Box','Base Box 2','box',[0,-1,-1],[3,-4,-5],null,null],
		['Box','Translated Box 1','box',[1,0,0],[4,7,10],[1,3,5],null],
		['Box','Translated Box 2','box',[-1,-0.2,0],[-5,-4,85],[-2,0,80],null],
		['Box','Rotated Box 1','box',[1,0,1],[5,4,3],[0,0,0],[0,1,0,1]],
		['Box','Rotated Box 2','box',[-1,1,-1],[-1.423,4,-5.655],[0,0,0],[0,0.415,0,1]],
		['Box','Translated & Rotated Box 1','box',[1,1,-1],[7,7,-3],[2,3,0],[0,1,0,1]],
		['Box','Translated & Rotated Box 2','box',[-1,-1,1],[-3.66,-1,-1.41],[2,3,0],[0,0.415,0,1]],
		['Cylinder','Base Cylinder 1','cylinder',[1,0,0],[3,2.5,0],null,null],
		['Cylinder','Base Cylinder 2','cylinder',[0,-1,0],[0,-2.5,0],null,null],
		['Cylinder','Base Cylinder 3','cylinder',[1,1,-1],[S45,2.5,-S45],null,null],
		['Cylinder','Translated Cylinder 1','cylinder',[1,0,0],[5,5.5,0],[2,3,0],null],
		['Cylinder','Translated Cylinder 2','cylinder',[1,1,-1],[2+S45,5.5,-S45],[2,3,0],null],
		['Cylinder','Rotated Cylinder 1','cylinder',[1,0,0],[3,0,2.5],[0,0,0],[1,0,0,1]],
		['Cylinder','Rotated Cylinder 2','cylinder',[1,1,-1],[S45,S45,-2.5],[0,0,0],[1,0,0,1]],
		['Cylinder','Translated & Rotated Cylinder 1','cylinder',[1,0,0],[5,3,2.5],[2,3,0],[1,0,0,1]],
		['Cylinder','Translated & Rotated Cylinder 2','cylinder',[1,1,-1],[4.121,5.121,-2.5],[2,3,0],[1,0,0,1]],
		['Cone','Base Cone 1','cone',[0,1,0],[0,2.5,0],null,null],
		['Cone','Base Cone 2','cone',[0,-1,0],[0,-2.5,0],null,null],
		['Cone','Base Cone 3','cone',[1,-1,-1],[S45,-2.5,-S45],null,null],
		['Cone','Translated Cone 1','cone',[0,1,0],[2,5.5,0],[2,3,0],null],
		['Cone','Translated Cone 2','cone',[1,-1,-1],[2+S45,0.5,-S45],[2,3,0],null],
		['Cone','Rotated Cone 1','cone',[0,1,0],[0,3,-2.5],[0,0,0],[1,0,0,1]],
		['Cone','Rotated Cone 2','cone',[1,1,-1],[S45,S45,-2.5],[0,0,0],[1,0,0,1]],
		['Cone','Translated & Rotated Cone 1','cone',[1,0,0],[5,3,-2.5],[2,3,0],[1,0,0,1]],
		['Cone','Translated & Rotated Cone 2','cone',[1,1,-1],[4.121,5.121,-2.5],[2,3,0],[1,0,0,1]],
		['PlaneX','Base PlaneX 1','plane_x',[1,0,0],[0,5,10],null,null],
		['PlaneX','Base PlaneX 2','plane_x',[0,-1,0],[0,-5,10],null,null],
		['PlaneX','Rotated PlaneX','plane_x',[1,0,0],[0,-10,5],[0,0,0],[1,0,0,1]],
		['PlaneX','Translated & Rotated PlaneX','plane_x',[1,1,-1],[12,8,0],[2,3,0],[0,1,0,1]],
		['PlaneY','Base PlaneY 1','plane_y',[1,0,0],[5,0,10],null,null],
		['PlaneY','Base PlaneY 2','plane_y',[0,-1,0],[5,0,10],null,null],
		['PlaneY','Rotated PlaneY','plane_y',[1,0,0],[5,-10,0],[0,0,0],[1,0,0,1]],
		['PlaneY','Translated & Rotated PlaneY','plane_y',[1,1,-1],[12,3,-5],[2,3,0],[0,1,0,1]],
		['PlaneZ','Base PlaneZ 1','plane_z',[1,0,0],[5,10,0],null,null],
		['PlaneZ','Base PlaneZ 2','plane_z',[0,-1,0],[5,-10,0],null,null],
		['PlaneZ','Rotated PlaneZ','plane_z',[1,0,0],[5,0,10],[0,0,0],[1,0,0,1]],
		['PlaneZ','Translated & Rotated PlaneZ','plane_z',[1,1,-1],[2,13,-5],[2,3,0],[0,1,0,1]]
	];

	var DESC = "A support point is the farthest point of a shape in a given direction. For each " +
		"shape, in a chosen direction, the returned point must be exactly the correct extreme " +
		"vertex/surface point, staying correct when the shape is moved and rotated.";

	ROWS.forEach(function (r) {
		var group = r[0], name = r[1], key = r[2], dir = r[3], exp = r[4], pos = r[5], rot = r[6];
		Runner.test('support/' + group.toLowerCase(), name, function (t) {
			var opts = { mass: 1 };
			if (pos) opts.pos = pos;
			if (rot) opts.rot = rot;
			var body = t.loneBody(mkShape(key), opts);
			var out = new V();
			body.findSupportPoint(new V(dir[0], dir[1], dir[2]), out);
			t.support = { dir: dir, point: [out.x, out.y, out.z] };
			t.check(out.distanceTo(new V(exp[0], exp[1], exp[2])), 0, EPS,
				'support point in direction (' + dir.join(',') + ') is (' + exp.map(function (n) { return +n.toFixed(2); }).join(',') + ')');
		}, { visual: true, steps: 0, page: 'support-points', description: DESC });
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
