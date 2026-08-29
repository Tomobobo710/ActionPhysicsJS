(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Sweep-and-prune over body AABBs: sorts on the widest-spread axis, walks the sorted " +
		"list, confirms the other two axes directly. No false negatives; over-reporting is fine " +
		"(midphase/narrowphase are the ones that actually decide). Two statics never pair with " +
		"each other, and collision mask/group are checked here so a filtered pair never reaches " +
		"midphase at all.";
	function test(group, name, fn, opts) {
		opts = opts || {};

		Runner.test(group, name, fn, { page: 'broadphase', description: DESC, visual: opts.visual !== false, steps: 0 });
	}

	function mkBody(t, shape, mass, pos, color) {
		return t.loneBody(shape, { mass: mass, pos: pos, color: color });
	}

	function pairIds(pairs) {
		return pairs.map(function (p) { return p[0].id + '-' + p[1].id; });
	}

	test('collision/broadphase', 'two overlapping spheres produce one candidate pair', function (t) {
		var bp = new AP.SAPBroadphase();
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 1, [1.5, 0, 0]);
		bp.add(a); bp.add(b);
		var pairs = bp.computePairs();
		t.checkEqual(pairs.length, 1, 'exactly one candidate pair');
		t.checkTrue(pairs[0][0] === a && pairs[0][1] === b, 'pair keeps the lower-id body first');
	});

	test('collision/broadphase', 'two far-apart spheres produce no pair', function (t) {
		var bp = new AP.SAPBroadphase();
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 1, [10, 0, 0]);
		bp.add(a); bp.add(b);
		t.checkEqual(bp.computePairs().length, 0, 'no false candidate for separated AABBs');
	});

	test('collision/broadphase', 'AABBs overlapping only off the sweep axis are still caught', function (t) {

		var bp = new AP.SAPBroadphase();
		var wide1 = mkBody(t, new AP.SphereShape(0.1), 1, [-50, 0, 0]);
		var wide2 = mkBody(t, new AP.SphereShape(0.1), 1, [50, 0, 0]);
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 1, [0, 1.5, 0]);
		bp.add(wide1); bp.add(wide2); bp.add(a); bp.add(b);
		var ids = pairIds(bp.computePairs());
		t.checkTrue(ids.indexOf(a.id + '-' + b.id) !== -1, 'off-axis overlap still reported despite a wide sweep-axis spread');
	});

	test('collision/broadphase', 'two static bodies never pair with each other', function (t) {
		var bp = new AP.SAPBroadphase();
		var a = mkBody(t, new AP.SphereShape(1), 0, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 0, [0.5, 0, 0]);
		bp.add(a); bp.add(b);
		t.checkEqual(bp.computePairs().length, 0, 'static-static pairs are filtered - nothing can move them together');
	});

	test('collision/broadphase', 'a dynamic body still pairs against an overlapping static one', function (t) {
		var bp = new AP.SAPBroadphase();
		var ground = mkBody(t, new AP.BoxShape(10, 0.5, 10), 0, [0, -0.5, 0]);
		var falling = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		bp.add(ground); bp.add(falling);
		t.checkEqual(bp.computePairs().length, 1, 'dynamic-static pair survives the static-static filter');
	});

	test('collision/broadphase', 'collision mask/group filters a pair before it reaches midphase', function (t) {
		var bp = new AP.SAPBroadphase();
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 1, [0.5, 0, 0]);
		a.collision_groups = 0x1; a.collision_mask = 0x2;
		b.collision_groups = 0x2; b.collision_mask = 0x1;
		bp.add(a); bp.add(b);
		t.checkTrue(bp.computePairs().length === 1, 'compatible mask/group still pairs');

		var bp2 = new AP.SAPBroadphase();
		var c = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var d = mkBody(t, new AP.SphereShape(1), 1, [0.5, 0, 0]);
		c.collision_groups = 0x1; c.collision_mask = 0x1;
		d.collision_groups = 0x2; d.collision_mask = 0x2;
		bp2.add(c); bp2.add(d);
		t.checkEqual(bp2.computePairs().length, 0, 'incompatible mask/group is filtered out');
	});

	test('collision/broadphase', 'remove takes a body out of future pair generation', function (t) {
		var bp = new AP.SAPBroadphase();
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 1, [0.5, 0, 0]);
		bp.add(a); bp.add(b);
		t.checkEqual(bp.computePairs().length, 1, 'paired before removal');
		bp.remove(a);
		t.checkEqual(bp.computePairs().length, 0, 'no pairs once one body is removed');
	});

	test('collision/broadphase', 'candidate pairs among a small cluster of bodies', function (t) {
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0], '#4af');
		var b = mkBody(t, new AP.SphereShape(1), 1, [1.5, 0, 0], '#4af');
		var c = mkBody(t, new AP.SphereShape(1), 1, [0, 3, 0], '#4af');
		var d = mkBody(t, new AP.SphereShape(1), 1, [6, 0, 0], '#666');
		var bp = new AP.SAPBroadphase();
		[a, b, c, d].forEach(function (x) { bp.add(x); });
		var pairs = bp.computePairs();
		t.checkEqual(pairs.length, 1, 'only the two touching spheres (a,b) pair up');
		t.checkTrue(pairIds(pairs)[0] === a.id + '-' + b.id, 'a-b is the reported pair');
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
