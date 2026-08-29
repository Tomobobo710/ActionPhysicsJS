// Contacts: ContactDetails (normalizes GJK/EPA output into one signed-distance convention),
// ContactManifold (owns point lifetime across ticks - the ONE thing plan.md's Contact management
// bug reference is about), ContactManifoldList (the set of active manifolds, keyed by body pair).
(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3, Q = AP.Quaternion;
	var DESC = "ContactDetails normalizes GJK's separated result and EPA's overlapping result into " +
		"one signed distance (negative = separated, positive = overlapping, per plan.md). " +
		"ContactManifold owns point lifetime entirely - detection only ever adds or refreshes; only " +
		"the manifold itself removes a point, and only once per TICK, never mid-substep (this is " +
		"the exact bug plan.md's Contact management bug reference describes in the predecessor).";
	function test(group, name, fn, opts) {
		opts = opts || {};
		// Every test here builds real shapes via mkBody (below), which draws them - so visual
		// defaults to true unless a test explicitly opts out (none currently do; mkFlatContact-only
		// tests still construct the two bodies a manifold is keyed to, so they draw too).
		Runner.test(group, name, fn, { page: 'contacts', description: DESC, visual: opts.visual !== false, steps: 0 });
	}

	// Builds a real body AND draws it (via t.loneBody), so every contacts test - which all construct
	// shapes to test against - is watchable, not just asserted.
	function mkBody(t, shape, mass, pos) {
		return t.loneBody(shape, { mass: mass, pos: pos, color: pos && pos[0] < 0 ? '#4af' : '#f84' });
	}

	function contactFor(bodyA, bodyB) {
		var placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
		var placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };
		var sup = new AP.MinkowskiSupport(placedA, placedB);
		var gjkResult = new AP.GJK().run(sup);
		var cd = new AP.ContactDetails();
		if (gjkResult.overlapping) {
			cd.setFromEPA(new AP.EPA().run(sup, gjkResult.simplex));
		} else {
			cd.setFromGJKSeparated(gjkResult);
		}
		return cd;
	}

	function mkFlatContact(px, py, pz, depth) {
		var cd = new AP.ContactDetails();
		cd.pointOnA.set(px, py, pz); cd.pointOnB.set(px, py, pz); cd.point.set(px, py, pz);
		cd.normal.set(1, 0, 0); cd.signedDistance = depth;
		return cd;
	}

	// ---- ContactDetails: sign convention ----

	test('collision/contacts', 'a separated pair produces a negative signed distance', function (t) {
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 1, [5, 0, 0]);
		var cd = contactFor(a, b);
		t.check(cd.signedDistance, -3, 1e-6, 'negative = separated, per plan.md');
	});

	test('collision/contacts', 'an overlapping pair produces a positive signed distance', function (t) {
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [1.5, 0, 0]);
		var cd = contactFor(a, b);
		t.check(cd.signedDistance, 0.5, 1e-6, 'positive = overlapping, per plan.md');
	});

	test('collision/contacts', 'point is the midpoint of the two witness points', function (t) {
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [1.5, 0, 0]);
		var cd = contactFor(a, b);
		t.check(cd.point.x, (cd.pointOnA.x + cd.pointOnB.x) / 2, 1e-9, 'point.x is the midpoint');
	});

	// ---- ContactManifold: warm-start persistence ----

	test('collision/contacts', 'a re-confirmed contact keeps its manifold point and preserves accumulated lambda', function (t) {
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [1.5, 0, 0]);
		var manifold = new AP.ContactManifold(a, b);
		manifold.update([contactFor(a, b)]);
		t.checkEqual(manifold.pointCount, 1, 'one point after the first tick');
		manifold.points[0].normalLambda = 7.5; // simulate the solver having accumulated something

		manifold.update([contactFor(a, b)]); // same bodies, same positions: same contact next tick
		t.checkEqual(manifold.pointCount, 1, 'still one point, not a fresh one');
		t.check(manifold.points[0].normalLambda, 7.5, 1e-9, 'accumulated lambda survived the refresh - the warm start');
	});

	test('collision/contacts', 'a genuinely new point starts with zero lambda, not inherited from an old point', function (t) {
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0, 0, 0]);
		var manifold = new AP.ContactManifold(a, mkBody(t, new AP.BoxShape(1, 1, 1), 1, [1.5, 0, 0]));
		var far = mkFlatContact(100, 100, 100, 0.1); // far outside MATCH_DISTANCE of anything existing
		manifold.update([far]);
		t.checkEqual(manifold.pointCount, 1, 'the far point was added');
		t.checkEqual(manifold.points[0].normalLambda, 0, 'a brand new point has no warm-start history');
	});

	// ---- ContactManifold: removal only on update(), only for un-reconfirmed points ----

	test('collision/contacts', 'a contact that stops being detected is removed on the next update()', function (t) {
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [1.5, 0, 0]);
		var manifold = new AP.ContactManifold(a, b);
		manifold.update([contactFor(a, b)]);
		t.checkEqual(manifold.pointCount, 1, 'contact present');
		manifold.update([]); // nothing detected this tick (bodies separated, narrowphase reports no candidates)
		t.checkEqual(manifold.pointCount, 0, 'the un-reconfirmed point is pruned');
	});

	test('collision/contacts', 'calling update() with the SAME contacts repeatedly never drops points mid-sequence', function (t) {
		// This is the actual bug-reference scenario in miniature: a point that is genuinely still
		// in contact must survive being "refreshed" over and over, the way a solver's own
		// substeps would call update() if it were (wrongly) run per-substep instead of per-tick.
		// ContactManifold.update() is documented as tick-only; this proves calling it repeatedly
		// with an unchanged, still-valid contact is itself safe and idempotent - the ACTUAL fix is
		// architectural (the solver never calls update() at all), but this pins down that the data
		// structure doesn't compound the mistake if it were.
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [1.5, 0, 0]);
		var manifold = new AP.ContactManifold(a, b);
		for (var i = 0; i < 20; i++) {
			manifold.update([contactFor(a, b)]);
			t.checkTrue(manifold.pointCount === 1, 'tick ' + i + ': point survives a repeated, still-valid refresh');
		}
	});

	// ---- ContactManifold: 4-point cap and reduction ----

	test('collision/contacts', 'a 5th point triggers reduction back to 4, always keeping the deepest', function (t) {
		var a = mkBody(t, new AP.BoxShape(5, 5, 5), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(5, 5, 5), 1, [9, 0, 0]);
		var manifold = new AP.ContactManifold(a, b);
		var contacts = [
			mkFlatContact(4.9, 4.9, 4.9, 0.5), // deepest
			mkFlatContact(4.9, 4.9, -4.9, 0.1),
			mkFlatContact(4.9, -4.9, 4.9, 0.1),
			mkFlatContact(4.9, -4.9, -4.9, 0.1),
			mkFlatContact(4.85, 4.85, 4.85, 0.05) // near-duplicate of the deepest corner
		];
		contacts.forEach(function (c) { manifold._addPoint(c); });
		t.checkEqual(manifold.pointCount, 4, 'capped at MAX_POINTS');
		var hasDeepest = manifold.points.some(function (p) { return p.signedDistance === 0.5; });
		t.checkTrue(hasDeepest, 'the deepest point is always retained');
	});

	test('collision/contacts', 'reduction prefers the largest-area combination over a near-duplicate point', function (t) {
		var a = mkBody(t, new AP.BoxShape(5, 5, 5), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(5, 5, 5), 1, [9, 0, 0]);
		var manifold = new AP.ContactManifold(a, b);
		var contacts = [
			mkFlatContact(4.9, 4.9, 4.9, 0.5),
			mkFlatContact(4.9, 4.9, -4.9, 0.1),
			mkFlatContact(4.9, -4.9, 4.9, 0.1),
			mkFlatContact(4.9, -4.9, -4.9, 0.1),
			mkFlatContact(4.85, 4.85, 4.85, 0.05) // near-duplicate of the FIRST corner, not deepest
		];
		contacts.forEach(function (c) { manifold._addPoint(c); });
		var nearDup = manifold.points.some(function (p) { return Math.abs(p.point.x - 4.85) < 1e-6; });
		t.checkTrue(!nearDup, 'the near-duplicate point lost to the well-spread corners, not kept redundantly');
	});

	test('collision/contacts', 'never exceeds MAX_POINTS even with many points added directly', function (t) {
		var a = mkBody(t, new AP.BoxShape(5, 5, 5), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(5, 5, 5), 1, [9, 0, 0]);
		var manifold = new AP.ContactManifold(a, b);
		for (var i = 0; i < 12; i++) {
			manifold._addPoint(mkFlatContact(4.9 - i * 0.01, (i % 3) - 1, (i % 2) * 2 - 1, i * 0.05));
		}
		t.checkTrue(manifold.pointCount <= AP.ContactManifold.MAX_POINTS, 'stayed at or under the cap after 12 adds');
	});

	// ---- ContactManifoldList ----

	test('collision/contacts', 'getOrCreate uses a canonical body order regardless of call order', function (t) {
		var a = mkBody(t, new AP.SphereShape(1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.SphereShape(1), 1, [1, 0, 0]);
		var list = new AP.ContactManifoldList();
		var m1 = list.getOrCreate(a, b);
		var m2 = list.getOrCreate(b, a); // reversed argument order
		t.checkTrue(m1 === m2, 'the same manifold is returned regardless of argument order');
	});

	test('collision/contacts', 'refresh() prunes a manifold that ends the tick with zero points', function (t) {
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [1.5, 0, 0]);
		var list = new AP.ContactManifoldList();
		list.getOrCreate(a, b);
		var byPair = new Map();
		byPair.set(a.id < b.id ? a.id + ':' + b.id : b.id + ':' + a.id, [contactFor(a, b)]);
		list.refresh(byPair);
		t.checkEqual(list.size, 1, 'manifold survives while contacts are present');

		list.refresh(new Map()); // nothing detected this tick for any pair
		t.checkEqual(list.size, 0, 'the empty manifold is pruned');
	});

	// ---- contact point + normal overlay ----

	test('collision/contacts', 'contact point and normal for two overlapping boxes', function (t) {
		var a = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [-0.75, 0, 0]);
		var b = mkBody(t, new AP.BoxShape(1, 1, 1), 1, [0.75, 0, 0]);
		var cd = contactFor(a, b);
		t.support = { dir: [cd.normal.x, cd.normal.y, cd.normal.z], point: [cd.point.x, cd.point.y, cd.point.z] };
		t.check(cd.signedDistance, 0.5, 1e-6, 'depth = 2 - 1.5');
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
