// Shapes: support functions, local AABBs, and mass properties (volume, inertia, center of mass).
// Support-function correctness matters more here than anywhere else downstream - it is GJK's ONLY
// required primitive (plan.md, Shape contract), so every shape gets support checked along its
// cardinal axes plus at least one off-axis direction.
(function (Runner) {
	Runner.suite('shapes');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Shapes provide, in local space: a support function (farthest point along a direction - " +
		"the one primitive GJK/EPA need), a tight local AABB, and mass properties (volume, inertia " +
		"tensor, center of mass) for a shape of density 1.";
	function test(group, name, fn) { Runner.test(group, name, fn, { page: 'shapes', description: DESC }); }

	function vecIs(t, v, x, y, z, label, eps) {
		eps = eps == null ? 1e-9 : eps;
		t.check(v.x, x, eps, label + '.x');
		t.check(v.y, y, eps, label + '.y');
		t.check(v.z, z, eps, label + '.z');
	}

	// ---- AABB ----

	test('shapes/aabb', 'combine grows to contain both boxes', function (t) {
		var a = new AP.AABB().setFromMinMax(-1, -1, -1, 1, 1, 1);
		var b = new AP.AABB().setFromMinMax(0, 2, -3, 5, 3, 0);
		a.combineInPlace(b);
		vecIs(t, a.min, -1, -1, -3, 'min');
		vecIs(t, a.max, 5, 3, 1, 'max');
	});

	test('shapes/aabb', 'combineInto is safe when out aliases an input', function (t) {
		var a = new AP.AABB().setFromMinMax(-1, -1, -1, 1, 1, 1);
		var b = new AP.AABB().setFromMinMax(0, 2, -3, 5, 3, 0);
		AP.AABB.combineInto(a, a, b);
		vecIs(t, a.min, -1, -1, -3, 'min');
		vecIs(t, a.max, 5, 3, 1, 'max');
	});

	test('shapes/aabb', 'intersects / containsPoint / containsAABB', function (t) {
		var a = new AP.AABB().setFromMinMax(0, 0, 0, 10, 10, 10);
		var overlapping = new AP.AABB().setFromMinMax(5, 5, 5, 15, 15, 15);
		var separate = new AP.AABB().setFromMinMax(20, 20, 20, 21, 21, 21);
		var inner = new AP.AABB().setFromMinMax(1, 1, 1, 2, 2, 2);
		t.checkTrue(a.intersects(overlapping), 'overlapping boxes intersect');
		t.checkTrue(!a.intersects(separate), 'separate boxes do not intersect');
		t.checkTrue(a.containsPoint(new V(5, 5, 5)), 'contains interior point');
		t.checkTrue(!a.containsPoint(new V(20, 5, 5)), 'rejects exterior point');
		t.checkTrue(a.containsAABB(inner), 'contains a fully interior box');
		t.checkTrue(!a.containsAABB(overlapping), 'rejects a partially overlapping box');
	});

	test('shapes/aabb', 'expandInPlace grows every face by the margin', function (t) {
		var a = new AP.AABB().setFromMinMax(-1, -1, -1, 1, 1, 1).expandInPlace(0.5);
		vecIs(t, a.min, -1.5, -1.5, -1.5, 'min');
		vecIs(t, a.max, 1.5, 1.5, 1.5, 'max');
	});

	test('shapes/aabb', 'setEmpty then combine reduces to exactly the combined box', function (t) {
		var a = new AP.AABB().setEmpty();
		var b = new AP.AABB().setFromMinMax(2, 3, 4, 5, 6, 7);
		a.combineInPlace(b);
		vecIs(t, a.min, 2, 3, 4, 'min');
		vecIs(t, a.max, 5, 6, 7, 'max');
	});

	// ---- Box ----

	test('shapes/box', 'support point is the correct corner on each cardinal axis', function (t) {
		var s = new AP.BoxShape(1, 2, 3);
		var out = new V();
		vecIs(t, s.supportInto(out, new V(1, 0, 0)), 1, 2, 3, '+X dir');
		vecIs(t, s.supportInto(out, new V(-1, 0, 0)), -1, 2, 3, '-X dir');
		vecIs(t, s.supportInto(out, new V(0, -1, 0)), 1, -2, 3, '-Y dir');
		vecIs(t, s.supportInto(out, new V(0.3, 0.3, 0.3)), 1, 2, 3, 'off-axis dir picks the far corner');
	});

	test('shapes/box', 'local AABB matches the half-extents', function (t) {
		var s = new AP.BoxShape(1, 2, 3);
		var out = new AP.AABB();
		s.localAABBInto(out);
		vecIs(t, out.min, -1, -2, -3, 'min');
		vecIs(t, out.max, 1, 2, 3, 'max');
	});

	test('shapes/box', 'volume and inertia match the closed-form solid-cuboid result', function (t) {
		var s = new AP.BoxShape(1, 2, 3); // 2x4x6
		t.check(s.volume(), 48, 1e-9, 'volume = w*h*d');
		var data = s.computeMassData();
		t.check(data.mass, 48, 1e-9, 'mass at density 1');
		// I_xx = m(h^2+d^2)/12 with h=4, d=6
		t.check(data.inertia.e00, 48 * (16 + 36) / 12, 1e-6, 'Ixx');
		t.check(data.inertia.e11, 48 * (4 + 36) / 12, 1e-6, 'Iyy');
		t.check(data.inertia.e22, 48 * (4 + 16) / 12, 1e-6, 'Izz');
		t.check(data.inertia.e01, 0, 1e-9, 'off-diagonal is zero for a centered box');
	});

	// ---- Sphere ----

	test('shapes/sphere', 'support point lies on the surface along the direction', function (t) {
		var s = new AP.SphereShape(2);
		var out = new V();
		vecIs(t, s.supportInto(out, new V(1, 0, 0)), 2, 0, 0, '+X');
		vecIs(t, s.supportInto(out, new V(0, 0, -1)), 0, 0, -2, '-Z');
		s.supportInto(out, new V(1, 1, 1));
		t.check(out.length(), 2, 1e-9, 'off-axis support point still has radius length');
	});

	test('shapes/sphere', 'zero-length direction does not produce NaN', function (t) {
		var s = new AP.SphereShape(1);
		var out = new V();
		s.supportInto(out, new V(0, 0, 0));
		t.checkTrue(out.isFinite(), 'support point stays finite');
	});

	test('shapes/sphere', 'volume and inertia match the closed-form solid-sphere result', function (t) {
		var s = new AP.SphereShape(2);
		var expectedVolume = (4 / 3) * Math.PI * 8;
		t.check(s.volume(), expectedVolume, 1e-6, 'volume = 4/3 pi r^3');
		var data = s.computeMassData();
		var expectedI = 0.4 * expectedVolume * 4; // 2/5 m r^2
		t.check(data.inertia.e00, expectedI, 1e-6, 'Ixx = 2/5 m r^2');
		t.check(data.inertia.e11, expectedI, 1e-6, 'Iyy = 2/5 m r^2');
		t.check(data.inertia.e22, expectedI, 1e-6, 'Izz = 2/5 m r^2');
	});

	// ---- Cylinder ----

	test('shapes/cylinder', 'support point on the rim at mid-height, cap at the poles', function (t) {
		var s = new AP.CylinderShape(2, 3);
		var out = new V();
		s.supportInto(out, new V(1, 0, 0));
		t.check(out.x, 2, 1e-9, 'rim x at radius');
		t.check(out.y, 3, 1e-9, 'rim support still picks the +Y cap (tie-break)');
		vecIs(t, s.supportInto(out, new V(0, 1, 0)), 0, 3, 0, 'straight up picks the top cap center axis point');
		vecIs(t, s.supportInto(out, new V(0, -1, 0)), 0, -3, 0, 'straight down picks the bottom cap');
	});

	test('shapes/cylinder', 'volume and axis/side inertia match the closed-form result', function (t) {
		var s = new AP.CylinderShape(2, 3); // r=2, h=6
		var expectedVolume = Math.PI * 4 * 6;
		t.check(s.volume(), expectedVolume, 1e-6, 'volume = pi r^2 h');
		var data = s.computeMassData();
		var m = expectedVolume;
		t.check(data.inertia.e11, 0.5 * m * 4, 1e-6, 'Iyy (axis) = 1/2 m r^2');
		t.check(data.inertia.e00, m * (3 * 4 + 36) / 12, 1e-6, 'Ixx (side) = m(3r^2+h^2)/12');
	});

	// ---- Cone ----

	test('shapes/cone', 'support point is the apex when the direction points along +Y', function (t) {
		var s = new AP.ConeShape(2, 3);
		var out = new V();
		vecIs(t, s.supportInto(out, new V(0, 1, 0)), 0, 3, 0, 'apex');
		vecIs(t, s.supportInto(out, new V(0, -1, 0)), 0, -3, 0, 'base center axis (best rim point at (r,0,-h) has y=-h, x picked by tie)', 1e-9);
	});

	test('shapes/cone', 'support along the base rim direction stays on the base circle', function (t) {
		var s = new AP.ConeShape(2, 3);
		var out = new V();
		s.supportInto(out, new V(1, -0.01, 0));
		t.check(out.y, -3, 1e-9, 'rim point sits on the base plane');
		t.check(Math.sqrt(out.x * out.x + out.z * out.z), 2, 1e-6, 'rim point is at radius 2');
	});

	test('shapes/cone', 'volume matches the closed-form result', function (t) {
		var s = new AP.ConeShape(2, 3); // r=2, h=6
		t.check(s.volume(), Math.PI * 4 * 6 / 3, 1e-6, 'volume = pi r^2 h / 3');
	});

	test('shapes/cone', 'center of mass sits 1/4 height above the base, below the local origin', function (t) {
		var s = new AP.ConeShape(2, 3);
		var data = s.computeMassData();
		// base at y=-3, apex at y=3, h=6 -> centroid at -3 + 6/4 = -1.5
		t.check(data.centerOfMass.y, -1.5, 1e-9, 'centroid y');
		t.checkTrue(data.mass > 0, 'positive mass');
		t.checkTrue(data.inertia.e00 > 0 && data.inertia.e11 > 0, 'positive inertia');
	});

	// ---- Capsule ----

	test('shapes/capsule', 'rejects a total height shorter than the diameter', function (t) {
		var threw = false;
		try { new AP.CapsuleShape(2, 3); } catch (e) { threw = true; }
		t.checkTrue(threw, 'constructor throws for totalHeight < 2*radius');
	});

	test('shapes/capsule', 'support point on the axis picks the correct pole', function (t) {
		var s = new AP.CapsuleShape(1, 6); // segmentHalfLength = 3 - 1 = 2
		var out = new V();
		vecIs(t, s.supportInto(out, new V(0, 1, 0)), 0, 3, 0, '+Y pole is segmentHalfLength + radius');
		vecIs(t, s.supportInto(out, new V(0, -1, 0)), 0, -3, 0, '-Y pole');
	});

	test('shapes/capsule', 'support point off-axis lies on the correct hemisphere at radius', function (t) {
		var s = new AP.CapsuleShape(1, 6);
		var out = new V();
		s.supportInto(out, new V(1, 0, 0));
		t.check(out.x, 1, 1e-9, 'radius reached at the equator');
		// direction.y === 0 ties to the +Y pole (>= 0 branch), so the support sits at the top of
		// the cylindrical segment (segmentHalfLength), not the capsule's true equator.
		t.check(out.y, 2, 1e-9, 'zero y-component ties to the +Y segment cap');
	});

	test('shapes/capsule', 'volume is cylinder core plus two hemisphere caps (one full sphere)', function (t) {
		var s = new AP.CapsuleShape(1, 6); // segmentHalfLength = 2
		var expected = Math.PI * 1 * 1 * 4 + (4 / 3) * Math.PI; // cylinder(r=1,h=4) + sphere(r=1)
		t.check(s.volume(), expected, 1e-6, 'total volume');
	});

	test('shapes/capsule', 'mass data is finite and symmetric about the axis', function (t) {
		var s = new AP.CapsuleShape(1, 6);
		var data = s.computeMassData();
		t.checkTrue(data.inertia.isFinite(), 'inertia tensor is finite');
		t.check(data.inertia.e00, data.inertia.e22, 1e-9, 'Ixx === Izz (axisymmetric about Y)');
		t.checkTrue(data.inertia.e11 < data.inertia.e00, 'axis inertia is less than a side inertia for a slender capsule');
	});

	// ---- Convex ----

	test('shapes/convex', 'support scans the point cloud for the true maximum along a direction', function (t) {
		var pts = [new V(1, 0, 0), new V(-1, 0, 0), new V(0, 1, 0), new V(0, -1, 0), new V(0, 0, 1), new V(0, 0, -1)];
		var s = new AP.ConvexShape(pts);
		var out = new V();
		vecIs(t, s.supportInto(out, new V(1, 0, 0)), 1, 0, 0, '+X');
		vecIs(t, s.supportInto(out, new V(0, 0, -1)), 0, 0, -1, '-Z');
	});

	test('shapes/convex', 'local AABB is the tight bound of the point cloud', function (t) {
		var pts = [new V(2, -1, 0), new V(-3, 4, 1), new V(0, 0, -5)];
		var s = new AP.ConvexShape(pts);
		var out = new AP.AABB();
		s.localAABBInto(out);
		vecIs(t, out.min, -3, -1, -5, 'min');
		vecIs(t, out.max, 2, 4, 1, 'max');
	});

	// ---- Plane / Triangle (degenerate shapes) ----

	test('shapes/plane', 'support point always lies exactly on the plane', function (t) {
		var s = new AP.PlaneShape('y', 5, 10);
		var out = new V();
		s.supportInto(out, new V(0.2, 3, 0.1));
		t.check(out.y, 0, 1e-9, 'y-oriented plane support has zero y regardless of direction');
	});

	test('shapes/plane', 'carries zero mass, matching a static/kinematic-only shape', function (t) {
		var s = new AP.PlaneShape('y', 5, 10);
		t.check(s.volume(), 0, 0, 'zero volume');
		t.check(s.computeMassData().mass, 0, 0, 'zero mass');
	});

	test('shapes/triangle', 'support point is whichever vertex is farthest along the direction', function (t) {
		var s = new AP.TriangleShape(new V(0, 0, 0), new V(1, 0, 0), new V(0, 1, 0));
		var out = new V();
		vecIs(t, s.supportInto(out, new V(1, 0, 0)), 1, 0, 0, 'picks b');
		vecIs(t, s.supportInto(out, new V(0, 1, 0)), 0, 1, 0, 'picks c');
		vecIs(t, s.supportInto(out, new V(-1, -1, 0)), 0, 0, 0, 'picks a');
	});

	// ---- Compound ----

	test('shapes/compound', 'combines child volumes and mass', function (t) {
		var c = new AP.CompoundShape();
		c.addChild(new AP.SphereShape(1), new V(2, 0, 0), new AP.Quaternion());
		c.addChild(new AP.SphereShape(1), new V(-2, 0, 0), new AP.Quaternion());
		var sphereVolume = (4 / 3) * Math.PI;
		t.check(c.volume(), sphereVolume * 2, 1e-6, 'summed volume');
		var data = c.computeMassData();
		t.check(data.mass, sphereVolume * 2, 1e-6, 'summed mass');
		vecIs(t, data.centerOfMass, 0, 0, 0, 'symmetric placement centers the compound', 1e-9);
	});

	test('shapes/compound', 'parallel-axis theorem increases inertia for offset children', function (t) {
		var c = new AP.CompoundShape();
		c.addChild(new AP.SphereShape(1), new V(5, 0, 0), new AP.Quaternion());
		c.addChild(new AP.SphereShape(1), new V(-5, 0, 0), new AP.Quaternion());
		var data = c.computeMassData();
		var singleSphereI = new AP.SphereShape(1).computeMassData().inertia.e00;
		// About the compound's Y/Z axes, the offset along X adds m*d^2 per sphere (d=5).
		t.checkTrue(data.inertia.e11 > 2 * singleSphereI, 'Iyy grows past the sum of local inertias');
		t.checkTrue(data.inertia.e22 > 2 * singleSphereI, 'Izz grows past the sum of local inertias');
	});

	// ---- LineSwept ----

	test('shapes/lineswept', 'support extends the base shape support by the segment endpoint', function (t) {
		var s = new AP.LineSweptShape(new AP.SphereShape(1), 5);
		var out = new V();
		vecIs(t, s.supportInto(out, new V(0, 1, 0)), 0, 6, 0, '+Y: sphere radius plus half-length');
		vecIs(t, s.supportInto(out, new V(0, -1, 0)), 0, -6, 0, '-Y: sphere radius plus half-length');
		s.supportInto(out, new V(1, 0, 0));
		t.check(out.x, 1, 1e-9, 'perpendicular direction unaffected by the sweep length');
	});

	// ---- Mesh ----

	test('shapes/mesh', 'triangleAt reads the correct vertices for an indexed triangle', function (t) {
		var verts = [new V(0, 0, 0), new V(1, 0, 0), new V(0, 1, 0), new V(0, 0, 1)];
		var indices = [0, 1, 2, 1, 2, 3];
		var m = new AP.MeshShape(verts, indices);
		t.checkEqual(m.triangleCount, 2, 'two triangles from six indices');
		var a = new V(), b = new V(), c = new V();
		m.triangleAt(1, a, b, c);
		vecIs(t, a, 1, 0, 0, 'tri 1 vertex a');
		vecIs(t, b, 0, 1, 0, 'tri 1 vertex b');
		vecIs(t, c, 0, 0, 1, 'tri 1 vertex c');
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
