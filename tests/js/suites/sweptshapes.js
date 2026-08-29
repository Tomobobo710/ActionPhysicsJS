(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var EPS = 1e-9;
	var DESC = "A swept shape is a base shape dragged along a line segment - the volume it sweeps " +
		"out, used for swept collision. These check its bounding box and its support points.";

	function sweptBox() { return new AP.LineSweptShape(new AP.BoxShape(1, 1, 1), new V(0, -2, 3), new V(5, 0, 0)); }
	function sweptSphere() { return new AP.LineSweptShape(new AP.SphereShape(2), new V(-2, 0, 3), new V(-2, 1, -1)); }

	Runner.test('collision/sweptshapes', 'swept box: bounding box is correct', function (t) {
		var s = sweptBox();
		var aabb = new AP.AABB();
		s.localAABBInto(aabb);
		t.check(aabb.min.x, -1, EPS, 'aabb.min.x'); t.check(aabb.min.y, -3, EPS, 'aabb.min.y'); t.check(aabb.min.z, -1, EPS, 'aabb.min.z');
		t.check(aabb.max.x, 6, EPS, 'aabb.max.x'); t.check(aabb.max.y, 1, EPS, 'aabb.max.y'); t.check(aabb.max.z, 4, EPS, 'aabb.max.z');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

	Runner.test('collision/sweptshapes', 'swept box: support points are correct', function (t) {
		var s = sweptBox(); var p = new V();
		s.supportInto(p, new V(0, -1, 0)); t.check(p.distanceTo(new V(1, -3, 4)), 0, EPS, 'support down');
		s.supportInto(p, new V(0, 1, 0)); t.check(p.distanceTo(new V(6, 1, 1)), 0, EPS, 'support up');
		s.supportInto(p, new V(-1, 0, 1)); t.check(p.distanceTo(new V(-1, -1, 4)), 0, EPS, 'support diagonal');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

	Runner.test('collision/sweptshapes', 'swept sphere: bounding box is correct', function (t) {
		var s = sweptSphere();
		var aabb = new AP.AABB();
		s.localAABBInto(aabb);
		t.check(aabb.min.x, -4, EPS, 'aabb.min.x'); t.check(aabb.min.y, -2, EPS, 'aabb.min.y'); t.check(aabb.min.z, -3, EPS, 'aabb.min.z');
		t.check(aabb.max.x, 0, EPS, 'aabb.max.x'); t.check(aabb.max.y, 3, EPS, 'aabb.max.y'); t.check(aabb.max.z, 5, EPS, 'aabb.max.z');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

	Runner.test('collision/sweptshapes', 'swept sphere: support points are correct', function (t) {
		var s = sweptSphere(); var p = new V();
		s.supportInto(p, new V(0, -1, 0)); t.check(p.distanceTo(new V(-2, -2, 3)), 0, EPS, 'support down');
		s.supportInto(p, new V(0, 1, 0)); t.check(p.distanceTo(new V(-2, 3, -1)), 0, EPS, 'support up');
		s.supportInto(p, new V(-1, 0, 1)); t.check(p.distanceTo(new V(-3.414213562373095, 0, 4.414213562373095)), 0, EPS, 'support diagonal');
	}, { visual: true, steps: 0, page: 'sweptshapes', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
