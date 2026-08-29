// RigidBody: identity/transform/mass fields, updateDerived() (world AABB + world inverse inertia),
// and the static/kinematic/dynamic body-type split. Motion/Forces/Material fields are plain data
// at this stage - nothing reads them yet (the solver does) - so they're exercised here only as
// far as "constructed with the right defaults", not behaviourally.
(function (Runner) {
	Runner.suite('shapes'); // bodies sit alongside shapes until the collision suite exists
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "RigidBody: shape + world transform + mass/motion/material state. updateDerived() " +
		"refreshes the world AABB and world-space inverse inertia tensor from position/rotation - " +
		"narrowphase and the solver assume it has already run for the current tick.";
	function test(group, name, fn, opts) {
		opts = opts || {};
		Runner.test(group, name, fn, { page: 'bodies', description: DESC, visual: !!opts.visual, steps: 0 });
	}

	test('shapes/rigidbody', 'a positive-mass body is dynamic, zero mass is static', function (t) {
		var dyn = new AP.RigidBody(new AP.SphereShape(1), 2);
		var stat = new AP.RigidBody(new AP.SphereShape(1), 0);
		t.checkEqual(dyn.bodyType, AP.RigidBody.DYNAMIC, 'positive mass -> DYNAMIC');
		t.checkTrue(!dyn.is_static, 'dynamic body reports is_static false');
		t.checkEqual(stat.bodyType, AP.RigidBody.STATIC, 'zero mass -> STATIC');
		t.checkTrue(stat.is_static, 'static body reports is_static true');
	});

	test('shapes/rigidbody', 'mass and inverse mass are consistent', function (t) {
		var b = new AP.RigidBody(new AP.BoxShape(1, 1, 1), 4);
		t.check(b.mass, 4, 0, 'mass');
		t.check(b._massInverted, 0.25, 1e-12, 'inverse mass');
		var s = new AP.RigidBody(new AP.BoxShape(1, 1, 1), 0);
		t.check(s._massInverted, 0, 0, 'a zero-mass body has zero inverse mass, not Infinity');
	});

	test('shapes/rigidbody', 'inertia tensor scales from the shape density-1 result by mass/volume', function (t) {
		var b = new AP.RigidBody(new AP.SphereShape(1), 2);
		var expected = 0.4 * 2 * 1 * 1; // 2/5 m r^2, m=2, r=1
		t.check(b.inertiaTensor.e00, expected, 1e-9, 'Ixx scaled to the requested mass');
		t.check(b.inertiaTensor.e11, expected, 1e-9, 'Iyy scaled to the requested mass');
		t.checkTrue(b.inverseInertiaTensor.isFinite(), 'inverse inertia is finite');
	});

	test('shapes/rigidbody', 'updateDerived computes a world AABB from position and shape', function (t) {
		var b = new AP.RigidBody(new AP.BoxShape(1, 2, 3), 1);
		b.position.set(5, -2, 0);
		b.updateDerived();
		var box = b.getAABB();
		t.check(box.min.x, 4, 1e-9, 'min.x = position.x - halfWidth');
		t.check(box.max.x, 6, 1e-9, 'max.x = position.x + halfWidth');
		t.check(box.min.y, -4, 1e-9, 'min.y = position.y - halfHeight');
		t.check(box.max.y, 0, 1e-9, 'max.y = position.y + halfHeight');
	});

	test('shapes/rigidbody', 'updateDerived AABB accounts for rotation, not just position', function (t) {
		var b = new AP.RigidBody(new AP.BoxShape(1, 1, 1), 1);
		b.rotation.setAxisAngle(new V(0, 0, 1), Math.PI / 4); // 45 degrees about Z
		b.updateDerived();
		var box = b.getAABB();
		var expectedHalfExtent = Math.sqrt(2); // a unit half-box's corner distance
		t.check(box.max.x, expectedHalfExtent, 1e-9, 'rotated box AABB widens past the unrotated half-extent');
	});

	test('shapes/rigidbody', 'setGravity overrides the world default per-body', function (t) {
		var b = new AP.RigidBody(new AP.SphereShape(1), 1);
		t.checkEqual(b.gravity, null, 'defaults to null (use World.gravity)');
		b.setGravity(0, 0, 0);
		t.checkTrue(b.gravity !== null, 'set to an explicit vector');
		t.check(b.gravity.y, 0, 0, 'zero-g override');
	});

	test('shapes/rigidbody', 'addListener/emit delivers the payload', function (t) {
		var b = new AP.RigidBody(new AP.SphereShape(1), 1);
		var got = null;
		b.addListener('collide', function (arg) { got = arg; });
		b.emit('collide', { other: 'x' });
		t.checkTrue(got && got.other === 'x', 'listener fired with the emitted payload');
	});

	test('shapes/rigidbody', 'each body gets a unique, increasing id', function (t) {
		var a = new AP.RigidBody(new AP.SphereShape(1), 1);
		var b = new AP.RigidBody(new AP.SphereShape(1), 1);
		t.checkTrue(b.id > a.id, 'ids increase across construction order');
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
