(function (Runner) {
	Runner.suite('collision'); // queries sit with the rest of the geometry-heavy suites
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "World.rayIntersect(start, end) / shapeIntersect(shape, start, end): ray and shape " +
		"casts against every body via a single GJK query per candidate. Reports the FIRST body hit " +
		"along the segment, or null.";
	function test(group, name, fn) {
		Runner.test(group, name, fn, { page: 'queries', description: DESC, visual: true, steps: 1 });
	}

	function mkWorld() {
		return new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), new AP.Solver());
	}

	// ---- rayIntersect: a straight hit reports the exact face point and outward normal ----

	test('queries/ray', 'a ray straight through a box hits the near face at the exact point, with the correct outward normal', function (t) {
		var world = mkWorld();
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var hit = null;
		t.expect('the ray reports a hit on the box (not null)', function () {
			hit = world.rayIntersect(new V(-5, 0, 0), new V(5, 0, 0));
			return { ok: hit !== null && hit.body === box, detail: hit ? 'hit body' : 'no hit' };
		});
		t.expect('the hit point sits exactly on the near face (x = -0.5, within 1e-9)', function () {
			return { ok: hit && Math.abs(hit.point.x - (-0.5)) < 1e-9 && Math.abs(hit.point.y) < 1e-9 && Math.abs(hit.point.z) < 1e-9,
				detail: hit ? 'point=(' + hit.point.x.toFixed(6) + ',' + hit.point.y.toFixed(6) + ',' + hit.point.z.toFixed(6) + ')' : 'no hit' };
		});
		t.expect('the hit normal points straight back along -X, the true outward face normal (not a diagonal GJK degenerate fallback)', function () {
			return { ok: hit && Math.hypot(hit.normal.x - (-1), hit.normal.y, hit.normal.z) < 1e-9,
				detail: hit ? 'normal=(' + hit.normal.x.toFixed(6) + ',' + hit.normal.y.toFixed(6) + ',' + hit.normal.z.toFixed(6) + ')' : 'no hit' };
		});
		t.expect('the reported fraction along the segment matches the geometric distance (4.5 of 10 units)', function () {
			return { ok: hit && Math.abs(hit.fraction - 0.45) < 1e-9, detail: hit ? 'fraction=' + hit.fraction.toFixed(6) : 'no hit' };
		});
		t.simulate(world, 1);
	});

	// ---- rayIntersect: a genuine miss, not a false hit from a loose broadphase reject ----

	test('queries/ray', 'a ray that passes well clear of every body reports no hit at all', function (t) {
		var world = mkWorld();
		t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var hit;
		t.expect('the ray reports null (a box sitting 5 units away from the ray line is a genuine miss)', function () {
			hit = world.rayIntersect(new V(-5, 5, 0), new V(5, 5, 0));
			return { ok: hit === null, detail: hit ? 'unexpectedly hit at fraction ' + hit.fraction.toFixed(4) : 'null, correct' };
		});
		t.simulate(world, 1);
	});

	// ---- the NEAREST of several candidates, not just any hit ----

	test('queries/ray', 'a ray through two boxes in a row reports the NEARER one, not just any hit', function (t) {
		var world = mkWorld();
		var near = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var far = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [3, 0, 0], color: '#f84' });
		var hit;
		t.expect('the reported body is the nearer box, not the farther one', function () {
			hit = world.rayIntersect(new V(-5, 0, 0), new V(10, 0, 0));
			return { ok: hit && hit.body === near, detail: hit ? (hit.body === near ? 'near box' : 'WRONG: far box') : 'no hit' };
		});
		t.expect('the hit point is on the near box face (x = -0.5), not the far one', function () {
			return { ok: hit && Math.abs(hit.point.x - (-0.5)) < 1e-9, detail: hit ? 'x=' + hit.point.x.toFixed(6) : 'no hit' };
		});
		t.simulate(world, 1);
	});

	// ---- starting already inside a body: fraction 0, not a crash or a false miss ----

	test('queries/ray', 'a ray that starts already inside a box reports an immediate hit at fraction 0', function (t) {
		var world = mkWorld();
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var hit;
		t.expect('the query reports a hit at zero travel, not null and not a crash', function () {
			hit = world.rayIntersect(new V(0, 0, 0), new V(5, 0, 0));
			return { ok: hit && hit.body === box && hit.fraction === 0, detail: hit ? 'fraction=' + hit.fraction : 'no hit' };
		});
		t.simulate(world, 1);
	});

	// ---- a static body is an ordinary query target ----

	test('queries/ray', 'a ray straight down hits a large static ground plane at the exact surface height', function (t) {
		var world = mkWorld();
		t.box(world, 50, 0.5, 50, 0, { pos: [0, -10, 0], color: '#888' }); // static ground
		var hit;
		t.expect('the ray hits the ground top surface at y = -9.5, exactly', function () {
			hit = world.rayIntersect(new V(0, 5, 0), new V(0, -20, 0));
			return { ok: hit && Math.abs(hit.point.y - (-9.5)) < 1e-9, detail: hit ? 'y=' + hit.point.y.toFixed(6) : 'no hit' };
		});
		t.expect('the hit normal points straight up (the ground top face), matching a real "what is under me" query', function () {
			return { ok: hit && Math.hypot(hit.normal.x, hit.normal.y - 1, hit.normal.z) < 1e-9,
				detail: hit ? 'normal=(' + hit.normal.x.toFixed(6) + ',' + hit.normal.y.toFixed(6) + ',' + hit.normal.z.toFixed(6) + ')' : 'no hit' };
		});
		t.simulate(world, 1);
	});

	// ---- swept shape hits offset by its own radius - the concept a zero-radius ray cannot exercise ----

	test('queries/shape', 'a swept sphere hits a box earlier than a zero-radius ray would, by exactly its own radius', function (t) {
		var world = mkWorld();
		t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0, 0], color: '#4af' });
		var sphere = new AP.SphereShape(0.25);
		var sweepHit = null, rayHit = null;
		t.expect('the sphere sweep hits the box (not null)', function () {
			sweepHit = world.shapeIntersect(sphere, new V(-5, 0, 0), new V(5, 0, 0));
			return { ok: sweepHit !== null, detail: sweepHit ? 'hit' : 'no hit' };
		});
		t.expect('the sweep stops exactly one sphere radius short of the ray-equivalent contact point (0.25 further back, within 1e-6)', function () {
			rayHit = world.rayIntersect(new V(-5, 0, 0), new V(5, 0, 0));
			var expectedX = rayHit.point.x - sphere.radius; // sphere center stops 0.25 before the surface hit
			return { ok: sweepHit && Math.abs(sweepHit.point.x - expectedX) < 1e-6,
				detail: sweepHit ? 'sweep x=' + sweepHit.point.x.toFixed(6) + ' expected=' + expectedX.toFixed(6) : 'no hit' };
		});
		t.simulate(world, 1);
	});

	// ---- orientation is respected: same swept shape at two rotations reaches the target at two distances ----

	test('queries/shape', 'a swept box reaches a target box at a different distance depending on its own held rotation', function (t) {
		var world = mkWorld();
		t.box(world, 0.5, 0.5, 0.5, 1, { pos: [3, 0, 0], color: '#4af' }); // target: an axis-aligned box
		var sweptShape = new AP.BoxShape(0.5, 0.1, 0.1); // long (half-extent 0.5) along local X, thin (0.1) along local Y/Z

		var longAxisLeading = new AP.Quaternion(0, 0, 0, 1); // long half-extent leads: bulk reaches out farther
		var thinAxisLeading = AP.Quaternion.fromAxisAngle(new V(0, 0, 1), Math.PI / 2); // thin extent leads: center can approach closer before contact

		var hitLong = null, hitThin = null;
		t.expect('with its long extent leading (pointing at the target), the shape touches earlier - its own bulk reaches out further', function () {
			hitLong = world.shapeIntersect(sweptShape, new V(-5, 0, 0), new V(5, 0, 0), longAxisLeading);
			return { ok: hitLong !== null, detail: hitLong ? 'fraction=' + hitLong.fraction.toFixed(4) : 'no hit' };
		});
		t.expect('rotated 90 degrees so its THIN extent leads instead, the shape travels farther (closer to the target) before its own leading face touches', function () {
			hitThin = world.shapeIntersect(sweptShape, new V(-5, 0, 0), new V(5, 0, 0), thinAxisLeading);
			return { ok: hitThin !== null, detail: hitThin ? 'fraction=' + hitThin.fraction.toFixed(4) : 'no hit' };
		});
		t.expect('the thin-leading orientation genuinely reaches further than the long-leading one - rotation is actually applied, not ignored', function () {
			return { ok: hitLong && hitThin && hitThin.fraction > hitLong.fraction,
				detail: 'long-leading=' + (hitLong ? hitLong.fraction.toFixed(4) : '?') + ' thin-leading=' + (hitThin ? hitThin.fraction.toFixed(4) : '?') };
		});
		t.simulate(world, 1);
	});

	// ---- a too-short sweep is a genuine miss; segment length is respected ----

	test('queries/shape', 'a sweep too short to reach a distant box reports no hit, even though the shape is heading straight at it', function (t) {
		var world = mkWorld();
		t.box(world, 0.5, 0.5, 0.5, 1, { pos: [10, 0, 0], color: '#4af' });
		var sphere = new AP.SphereShape(0.25);
		var hit;
		t.expect('a sweep from -5 to 2 (heading toward the box at x=10, but stopping short) reports null', function () {
			hit = world.shapeIntersect(sphere, new V(-5, 0, 0), new V(2, 0, 0));
			return { ok: hit === null, detail: hit ? 'unexpectedly hit at fraction ' + hit.fraction.toFixed(4) : 'null, correct' };
		});
		t.simulate(world, 1);
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
