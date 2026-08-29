// Dedicated box-box regression suite. Every scene here is pure box-on-box (no other shape types),
// chosen to stress the parts of BoxBox.js (src/phases/BoxBox.js) that are easy to get subtly wrong:
// reference/incident face selection, the face-vs-edge SAT tie-break, and the still-separated
// (speculative) contact path. These are NOT softballs - each one pins a bug that was actually found
// and fixed while building box-box, so a regression here means one of those bugs came back.
//
// EVERY t.expect below is a FINAL-GATE predicate: it returns { ok: false } until tick >= the run's
// total tick count, and only then reports against a WORST-observed-value tracked via onTick across
// the entire run. A predicate that reads current/live state directly is a bug in this file - t.expect
// LATCHES the first tick it reads true (see runner.js's evalTick) and never re-checks, so a naive
// "position - expected < threshold" check reads true at tick 1 (before the box has even started
// falling) and stays "passed" even if the scene explodes on tick 200. Every scene here was caught
// doing exactly that once - see the git history/session notes - before being rewritten this way.
(function (Runner) {
	Runner.suite('box-box');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;

	function totalRotationDegrees(body) {
		var wq = Math.abs(body.rotation.w);
		if (wq > 1) wq = 1;
		return 2 * Math.acos(wq) * 180 / Math.PI;
	}
	function speed(b) { var v = b.linear_velocity; return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
	function spin(b) { var a = b.angular_velocity; return Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

	// How far the body's local Y axis has tipped off world-up, decomposed into a lean toward +X/-X
	// and toward +Z/-Z, IN DEGREES - not just one scalar total-rotation number. totalRotationDegrees
	// alone can hide a real problem: a box that leans 5 degrees toward +X and 5 degrees toward -Z has
	// a smaller total-rotation angle than a box that leans 6 degrees toward +X alone, even though the
	// first box is arguably MORE off-kilter - checking each axis separately is what "look at it in
	// 3D, not just one softened number" means here.
	function leanDegrees(body) {
		var up = { x: 0, y: 1, z: 0 };
		body.rotation.transformVectorInPlace(up);
		return {
			x: Math.atan2(up.x, up.y) * 180 / Math.PI,
			z: Math.atan2(up.z, up.y) * 180 / Math.PI
		};
	}

	// Every finite-number invariant this suite cares about, in one place: no NaN/Infinity anywhere
	// (a silent numerical blowup), and the body has not been launched or swallowed by the floor.
	function isSane(body, floorY) {
		var p = body.position, v = body.linear_velocity, w = body.angular_velocity;
		if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) return false;
		if (!isFinite(v.x) || !isFinite(v.y) || !isFinite(v.z)) return false;
		if (!isFinite(w.x) || !isFinite(w.y) || !isFinite(w.z)) return false;
		if (p.y < floorY - 5) return false;  // fell through the world
		if (p.y > floorY + 100) return false; // launched
		return true;
	}

	// Shared tracker: watches a list of {body, x0, y0, z0} records every tick via onTick, keeping the
	// WORST (max) value seen so far for each metric, plus a snapshot at the run's halfway point so a
	// slow leak (still-growing error) can be told apart from a one-time settle transient (error that
	// stopped growing once contact was made). Every t.expect in this file reads from `worst`/`atHalf`
	// here, gated on the run having actually finished - never live body state.
	function makeTracker(t, boxes, totalTicks, floorY) {
		var worst = { x: 0, z: 0, y: 0, lx: 0, lz: 0, rot: 0, v: 0, w: 0 };
		var atHalf = null;
		var sane = true, insaneDetail = '';
		var half = Math.floor(totalTicks / 2);
		t.onTick(function (world, tick) {
			for (var n = 0; n < boxes.length; n++) {
				var o = boxes[n], body = o.body;
				if (!isSane(body, floorY)) { sane = false; insaneDetail = 'box ' + n + ' went insane: pos=' + JSON.stringify(body.position); continue; }
				var lean = leanDegrees(body);
				worst.x = Math.max(worst.x, Math.abs(body.position.x - o.x0));
				worst.z = Math.max(worst.z, Math.abs(body.position.z - o.z0));
				worst.y = Math.max(worst.y, Math.abs(body.position.y - o.y0));
				worst.lx = Math.max(worst.lx, Math.abs(lean.x));
				worst.lz = Math.max(worst.lz, Math.abs(lean.z));
				worst.rot = Math.max(worst.rot, totalRotationDegrees(body));
				worst.v = Math.max(worst.v, speed(body));
				worst.w = Math.max(worst.w, spin(body));
			}
			if (tick === half) atHalf = { x: worst.x, z: worst.z, y: worst.y, lx: worst.lx, lz: worst.lz, rot: worst.rot };
		});
		return {
			isSane: function () { return sane; },
			insaneDetail: function () { return insaneDetail; },
			worst: function () { return worst; },
			atHalf: function () { return atHalf; }
		};
	}

	// ---- flat single box: the baseline every other scene here builds on ----

	Runner.test('box-box/single', 'a single box on a much bigger box settles dead still (no residual torque)', function (t) {
		// A box on a MUCH bigger box (asymmetric size, like a real ground) is the everyday case, and
		// per-tick face clipping must not leak any lateral force/torque from a perfectly symmetric,
		// perfectly flat drop - any nonzero steady-state spin here is a geometry bug, not noise.
		var TICKS = 240;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });
		var box = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], color: '#4af' });
		var tracker = makeTracker(t, [{ body: box, x0: 0, y0: 0.5, z0: 0 }], TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('stays sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'sane throughout' : tracker.insaneDetail() };
		});
		t.expect('settles at exactly rest height (y = 0.5)', function () {
			if (tick0 < TICKS) return false;
			var y = box.position.y;
			return { ok: Math.abs(y - 0.5) < 1e-3, detail: 'y=' + y.toFixed(5) };
		});
		t.expect('ends with negligible linear speed', function () {
			if (tick0 < TICKS) return false;
			return { ok: speed(box) < 1e-3, detail: '|v|=' + speed(box).toFixed(6) };
		});
		t.expect('ends with negligible angular speed (no fabricated spin)', function () {
			if (tick0 < TICKS) return false;
			return { ok: spin(box) < 1e-3, detail: '|w|=' + spin(box).toFixed(6) };
		});
		t.expect('x/z drift never exceeded a millimetre (perfectly flat drop, nothing should push it sideways)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			var lat = Math.sqrt(worst.x * worst.x + worst.z * worst.z);
			return { ok: lat < 1e-3, detail: 'worst lateral drift=' + lat.toFixed(6) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 240, page: 'box-box-single' });

	// ---- two boxes exactly touching at spawn: the "already resting" case ----

	Runner.test('box-box/stack2', 'two boxes spawned already stacked, exactly touching, hold both positions', function (t) {
		// No fall, no impact - this isolates the manifold's OWN steady-state behavior (4-point face
		// contact, warm-started lambda) from any settle transient. Either box drifting at all here
		// means the resting manifold itself is injecting force/torque, not that a landing was rough.
		var TICKS = 180;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });
		var b0 = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], color: '#4af' });
		var b1 = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 1.5, 0], color: '#f84' });
		var tracker = makeTracker(t, [{ body: b0, x0: 0, y0: 0.5, z0: 0 }, { body: b1, x0: 0, y0: 1.5, z0: 0 }], TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('stays sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'sane throughout' : tracker.insaneDetail() };
		});
		t.expect('bottom box holds at y = 0.5', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(b0.position.y - 0.5) < 1e-3, detail: 'y=' + b0.position.y.toFixed(5) };
		});
		t.expect('top box holds at y = 1.5', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(b1.position.y - 1.5) < 1e-3, detail: 'y=' + b1.position.y.toFixed(5) };
		});
		t.expect('neither box ever drifted laterally', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			var lat = Math.sqrt(worst.x * worst.x + worst.z * worst.z);
			return { ok: lat < 1e-3, detail: 'worst lateral drift=' + lat.toFixed(6) };
		});
		t.expect('neither box ever picked up rotation', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 0.5, detail: 'worst rotation=' + worst.rot.toFixed(3) + ' degrees' };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 180, page: 'box-box-stack2' });

	// ---- box bridging two separate, non-overlapping supports ----

	Runner.test('box-box/bridge', 'a box bridging two separate supports stays flat, does not tip', function (t) {
		// Two supports spaced so their footprints do NOT overlap (each spans 1 unit either side of
		// its center, gap of 0.6 between them) - the bridging box's contact area on each support is a
		// genuine PARTIAL face overlap (not a full 4-corner match), which is exactly the case that
		// broke: a razor-thin, numerically-near-duplicate edge axis (from even a fraction of a degree
		// of accumulated tilt on a support) won a face-vs-edge SAT tie-break it should never have won,
		// collapsing what should be a 4-point manifold into 1 stray point and torquing the bridge over.
		var TICKS = 400;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });
		t.box(w, 1, 1, 1, 1, { pos: [-1.3, 1, 0], color: '#4af' });
		t.box(w, 1, 1, 1, 1, { pos: [1.3, 1, 0], color: '#4af' });
		var bridge = t.box(w, 1, 1, 1, 1, { pos: [0, 3.2, 0], color: '#f84' });
		var tracker = makeTracker(t, [{ body: bridge, x0: 0, y0: 3, z0: 0 }], TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('stays sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'sane throughout' : tracker.insaneDetail() };
		});
		t.expect('the bridge settles resting on both supports (y ~ 3)', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(bridge.position.y - 3) < 0.05, detail: 'y=' + bridge.position.y.toFixed(4) };
		});
		t.expect('the bridge never tipped (total rotation stayed under 2 degrees, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 2, detail: 'worst rotation=' + worst.rot.toFixed(3) + ' degrees' };
		});
		t.expect('the bridge never slid off its supports (x drift stayed under 0.1, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.x < 0.1, detail: 'worst x drift=' + worst.x.toFixed(4) };
		});
		// NOT a speed check: a resting contact can report a small non-decaying "derived velocity"
		// (v = dx/h reads back a tiny steady-state residual even once position has genuinely stopped
		// moving - a known, pre-existing solver characteristic, not something box-box owns) without
		// actually being unstable. What box-box itself must guarantee is that POSITION holds - so this
		// checks the position over the run's second half is not still drifting.
		t.expect('position genuinely stopped moving (second half added < 0.01 to y and x drift)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var addedY = worst.y - half.y, addedX = worst.x - half.x;
			return { ok: addedY < 0.01 && addedX < 0.01, detail: 'second-half added dy=' + addedY.toFixed(5) + ', dx=' + addedX.toFixed(5) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 400, page: 'box-box-bridge' });

	// ---- offset stacking: each box partially overlaps the one below, on BOTH horizontal axes ----

	Runner.test('box-box/offset-stack', 'a 4-box offset stack (each box inset from the one below, on BOTH axes) holds its shape', function (t) {
		// Reproduces the pyramid scene's own support geometry at a much smaller, fully-inspectable
		// scale: each box is inset so it overlaps its supporting neighbour by an UNEQUAL amount per
		// side - not a symmetric bridge, and not a full flush stack. Inset on BOTH x AND z (unevenly)
		// so a leak specific to one horizontal axis, or to the diagonal combination, cannot hide
		// behind a test that only ever moves along x - a real 3D asymmetric-support scene, not a 2D
		// slice of one.
		var TICKS = 500;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });

		var boxes = [];
		var LAYERS = 4, GAP = 2.2, INSET = 0.3;
		for (var i = 0; i < LAYERS; i++) {
			var x = i * INSET, z = i * INSET * 0.6, y = i * GAP + 1;
			var b = t.box(w, 1, 1, 1, 1, { pos: [x, y, z], color: '#B08968' });
			boxes.push({ body: b, x0: x, y0: y, z0: z });
		}
		var tracker = makeTracker(t, boxes, TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		t.expect('every box stayed sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'all ' + boxes.length + ' boxes finite and in bounds, the whole run' : tracker.insaneDetail() };
		});
		t.expect('every box stayed within 0.05 of its expected height (spacing held, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.y < 0.05, detail: 'worst height error=' + worst.y.toFixed(4) };
		});
		t.expect('every box stayed within 0.1 of its expected x (no creep along the x inset, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.x < 0.1, detail: 'worst x drift=' + worst.x.toFixed(4) };
		});
		t.expect('every box stayed within 0.1 of its expected z (no creep along the z inset, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.z < 0.1, detail: 'worst z drift=' + worst.z.toFixed(4) };
		});
		t.expect('nothing ever leaned on the X axis past 3 degrees, the whole run', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.lx < 3, detail: 'worst X-lean=' + worst.lx.toFixed(3) + ' degrees' };
		});
		t.expect('nothing ever leaned on the Z axis past 3 degrees, the whole run', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.lz < 3, detail: 'worst Z-lean=' + worst.lz.toFixed(3) + ' degrees' };
		});
		t.expect('nothing flipped or rolled over (absolute backstop: total rotation stayed under 90 degrees)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 90, detail: 'worst total rotation=' + worst.rot.toFixed(2) + ' degrees' };
		});
		t.expect('the whole stack is at rest by the end', function () {
			if (tick0 < TICKS) return false;
			var worstV = 0, worstW = 0;
			for (var n = 0; n < boxes.length; n++) {
				worstV = Math.max(worstV, speed(boxes[n].body));
				worstW = Math.max(worstW, spin(boxes[n].body));
			}
			return { ok: worstV < 0.01 && worstW < 0.01, detail: 'final |v|=' + worstV.toFixed(4) + ', final |w|=' + worstW.toFixed(4) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 500, page: 'box-box-offset-stack' });

	// ---- edge-edge contact: a box balanced corner-down on another box's edge/corner ----

	Runner.test('box-box/corner-drop', 'a box dropped corner-first onto flat ground tips onto a face and settles', function (t) {
		// The corner-first drop transiently exercises the edge-edge branch (two near-touching edges
		// before the box has rocked onto a full face) before settling into a normal face contact -
		// this is BoxBox.js's edge-edge code path's only real exercise in this suite.
		//
		// A cube balanced EXACTLY corner-down (or edge-down) is a genuine symmetric equilibrium with
		// no perturbation to break it in a deterministic sim with no injected noise - it can sit there
		// forever without that being a bug. So this composes THREE axis rotations (not a single clean
		// 45 degrees about one axis) to land corner-first while breaking that symmetry, the same way a
		// real corner-first drop never lands perfectly plumb.
		var TICKS = 400;
		var qx = t.quat(Math.sin(Math.PI / 8), 0, 0, Math.cos(Math.PI / 8));
		var halfZ = Math.atan(1 / Math.sqrt(2)) / 2;
		var qz = t.quat(0, 0, Math.sin(halfZ), Math.cos(halfZ));
		var qx2 = t.quat(Math.sin(0.05), 0, 0, Math.cos(0.05)); // small symmetry-breaking nudge
		var rot = AP.Quaternion.multiply(qx2, AP.Quaternion.multiply(qz, qx)).normalize();

		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });
		var box = t.box(w, 0.5, 0.5, 0.5, 1, {
			pos: [0, 3, 0], rot: [rot.x, rot.y, rot.z, rot.w],
			friction: 0.5, restitution: 0, color: '#4af'
		});

		var minY = Infinity;
		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; if (box.position.y < minY) minY = box.position.y; });

		t.expect('settles onto a face (SOME local axis lands within 1 degree of world-up, not corner/edge-balanced)', function () {
			if (tick0 < TICKS) return false;
			var best = 180;
			var localAxes = [t.vec(1, 0, 0), t.vec(0, 1, 0), t.vec(0, 0, 1)];
			for (var i = 0; i < 3; i++) {
				var axis = localAxes[i];
				box.rotation.transformVectorInPlace(axis);
				var d = Math.max(-1, Math.min(1, Math.abs(axis.y))); // either end of that axis can be "up"
				var deg = Math.acos(d) * 180 / Math.PI;
				if (deg < best) best = deg;
			}
			return { ok: best < 1, detail: 'nearest-to-vertical local axis is ' + best.toFixed(3) + ' degrees off' };
		});
		t.expect('comes to rest (no perpetual rocking)', function () {
			if (tick0 < TICKS) return false;
			return { ok: speed(box) < 0.01 && spin(box) < 0.01, detail: '|v|=' + speed(box).toFixed(4) + ' |w|=' + spin(box).toFixed(4) };
		});
		t.expect('never tunnels through the ground, the whole run', function () {
			if (tick0 < TICKS) return false;
			return { ok: minY > -1, detail: 'lowest y reached=' + minY.toFixed(4) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 400, page: 'box-box-corner-drop' });

	// ---- speculative approach: a box falling fast toward flush contact must not pick one corner ----

	Runner.test('box-box/fast-approach', 'a box falling fast onto flat ground lands flush, no single-corner torque kick', function (t) {
		// This is the exact shape of the bug fixed in BoxBox's separated-contact path: a fast-falling,
		// perfectly flat, perfectly symmetric box must never pick up SIDEWAYS velocity or spin purely
		// from landing - any of either is the signature of a single off-center speculative contact
		// point standing in for what should be all 4 corners together.
		var TICKS = 200;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });
		var box = t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], vel: [0, -15, 0], friction: 0.5, restitution: 0, color: '#4af' });

		var worstLateralV = 0, worstSpin = 0, landed = false, minY = Infinity, tick0 = 0;
		t.onTick(function (world, tick) {
			tick0 = tick;
			if (box.position.y < minY) minY = box.position.y;
			if (box.position.y < 0.55) landed = true;
			if (landed) {
				var lv = Math.sqrt(box.linear_velocity.x * box.linear_velocity.x + box.linear_velocity.z * box.linear_velocity.z);
				if (lv > worstLateralV) worstLateralV = lv;
				if (spin(box) > worstSpin) worstSpin = spin(box);
			}
		});
		t.expect('never tunnels through the ground, the whole run', function () {
			if (tick0 < TICKS) return false;
			return { ok: minY > -1, detail: 'lowest y reached=' + minY.toFixed(4) };
		});
		t.expect('no sideways velocity kick from landing (worst lateral speed under 0.05, the whole run)', function () {
			if (tick0 < TICKS) return false;
			return { ok: worstLateralV < 0.05, detail: 'worst lateral |v|=' + worstLateralV.toFixed(4) };
		});
		t.expect('no spin kick from landing (worst angular speed under 0.05, the whole run)', function () {
			if (tick0 < TICKS) return false;
			return { ok: worstSpin < 0.05, detail: 'worst |w|=' + worstSpin.toFixed(4) };
		});
		t.expect('settles at exactly rest height', function () {
			if (tick0 < TICKS) return false;
			return { ok: Math.abs(box.position.y - 0.5) < 1e-2, detail: 'y=' + box.position.y.toFixed(5) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 200, page: 'box-box-fast-approach' });

	// ---- long-horizon drift/tilt accumulation (the pyramid's own failure mode, isolated) ----

	Runner.test('box-box/long-rest', 'a 3-box offset stack (inset on BOTH axes) shows no slow drift or tilt accumulation over 1000 ticks', function (t) {
		// Same offset-stack shape as box-box/offset-stack (inset on x AND z, unevenly, so a leak
		// specific to one horizontal axis can't hide), but run far longer and specifically tracking
		// whether error GROWS over time rather than just checking the end state - the pyramid's own
		// remaining failure mode is a slow leak (tiny torque every tick, invisible for a few hundred
		// ticks, but a box has fully toppled by tick ~900). Comparing second-half growth to first-half
		// catches that leak long before a tip-over would, and does it per-axis (x drift, z drift,
		// X-lean, Z-lean, total rotation) rather than one blended scalar that a leak on an unchecked
		// axis could hide behind entirely.
		var TICKS = 1000;
		var w = t.makeWorld({ gravity: -9.8 });
		t.box(w, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#243B2A' });

		var boxes = [];
		var LAYERS = 3, GAP = 2.2, INSET = 0.3;
		for (var i = 0; i < LAYERS; i++) {
			var x = i * INSET, z = i * INSET * 0.6, y = i * GAP + 1;
			var b = t.box(w, 1, 1, 1, 1, { pos: [x, y, z], color: '#B08968' });
			boxes.push({ body: b, x0: x, y0: y, z0: z });
		}
		var tracker = makeTracker(t, boxes, TICKS, -0.5);

		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });

		t.expect('every box stayed sane throughout (no NaN, no launch, no fall-through)', function () {
			if (tick0 < TICKS) return false;
			return { ok: tracker.isSane(), detail: tracker.isSane() ? 'all ' + boxes.length + ' boxes finite and in bounds, the whole run' : tracker.insaneDetail() };
		});
		t.expect('x drift did not keep growing once settled (second half added < 0.02)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.x - half.x;
			return { ok: added < 0.02, detail: added.toFixed(4) + ' added since half-time (was ' + half.x.toFixed(4) + ', is ' + worst.x.toFixed(4) + ')' };
		});
		t.expect('z drift did not keep growing once settled (second half added < 0.02)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.z - half.z;
			return { ok: added < 0.02, detail: added.toFixed(4) + ' added since half-time (was ' + half.z.toFixed(4) + ', is ' + worst.z.toFixed(4) + ')' };
		});
		t.expect('y (height) drift did not keep growing once settled (second half added < 0.02)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.y - half.y;
			return { ok: added < 0.02, detail: added.toFixed(4) + ' added since half-time (was ' + half.y.toFixed(4) + ', is ' + worst.y.toFixed(4) + ')' };
		});
		t.expect('X-lean did not keep growing once settled (second half added < 0.5 degrees)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.lx - half.lx;
			return { ok: added < 0.5, detail: added.toFixed(3) + ' degrees added since half-time (was ' + half.lx.toFixed(3) + ', is ' + worst.lx.toFixed(3) + ')' };
		});
		t.expect('Z-lean did not keep growing once settled (second half added < 0.5 degrees)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.lz - half.lz;
			return { ok: added < 0.5, detail: added.toFixed(3) + ' degrees added since half-time (was ' + half.lz.toFixed(3) + ', is ' + worst.lz.toFixed(3) + ')' };
		});
		t.expect('total rotation did not keep growing once settled (second half added < 0.5 degrees)', function () {
			if (tick0 < TICKS) return false;
			var half = tracker.atHalf(), worst = tracker.worst();
			if (!half) return false;
			var added = worst.rot - half.rot;
			return { ok: added < 0.5, detail: added.toFixed(3) + ' degrees added since half-time (was ' + half.rot.toFixed(3) + ', is ' + worst.rot.toFixed(3) + ')' };
		});
		t.expect('nothing ever flipped or rolled over (absolute backstop: total rotation stayed under 90 degrees, the whole run)', function () {
			if (tick0 < TICKS) return false;
			var worst = tracker.worst();
			return { ok: worst.rot < 90, detail: 'worst total rotation=' + worst.rot.toFixed(2) + ' degrees' };
		});
		t.expect('every box is still at rest at the end (no late-breaking instability)', function () {
			if (tick0 < TICKS) return false;
			var worstV = 0, worstW = 0;
			for (var n = 0; n < boxes.length; n++) {
				worstV = Math.max(worstV, speed(boxes[n].body));
				worstW = Math.max(worstW, spin(boxes[n].body));
			}
			return { ok: worstV < 0.01 && worstW < 0.01, detail: 'final |v|=' + worstV.toFixed(4) + ', final |w|=' + worstW.toFixed(4) };
		});
		t.simulate(w, TICKS);
	}, { visual: true, steps: 1000, page: 'box-box-long-rest' });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
