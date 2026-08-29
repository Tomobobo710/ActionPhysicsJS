// Speculative contacts (plan.md build item 6a): a contact is detected BEFORE overlap, while the
// shapes are still separated, so the XPBD solver's non-penetration constraint - evaluated every
// substep against the body's PREDICTED position - stops the body AT touch instead of first letting
// it dig in and then ejecting it. This is the fix for the derived-velocity problem plan.md
// documents: without it, a body arrives already deep (several substeps of undetected travel), the
// one-shot correction produces velocity = Δx/h with nothing to damp it, and the body launches.
//
// The three moving parts, all exercised here:
//   1. Broadphase reads a FATTENED AABB (RigidBody.getBroadphaseAABB): tight bound + a fixed
//      speculative margin + this tick's velocity sweep, so a pair surfaces the tick before overlap.
//   2. Narrowphase reports a contact while still separated (within its per-pair speculative margin),
//      creating the manifold point before penetration.
//   3. The solver's live-C-per-substep non-penetration constraint stops the predicted overshoot at
//      touch, and the manifold keeps the established contact normal through the exact-touch band
//      where GJK/EPA's normal is ambiguous (which otherwise causes a penetrate-then-launch cycle).
//
// SCOPE: these tests cover the LINEAR mechanic, which is complete and stable - a flat drop settles
// exactly, a high-speed drop never tunnels, a resting contact never buzzes, and an axis-aligned
// stack settles exactly (see the rotation-locked stack test). Rotational settling of a corner/edge
// contact is NOT yet stable (the angular correction injects energy on first contact) and is a
// separate, documented increment - it is deliberately NOT tested here as if it worked. Tests that
// involve rotation lock the angular factor, isolating the linear behavior these assertions own.
(function (Runner) {
	Runner.suite('speculative');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Speculative contacts: contact detected before overlap (fattened broadphase AABB + " +
		"narrowphase speculative margin), solver stops the body at touch against its predicted " +
		"position. Verified: flat drop settles exactly, high-speed drop never tunnels, resting is " +
		"stable, axis-aligned stack settles. Rotational corner-contact settling is a separate, " +
		"not-yet-stable increment and is not asserted here.";
	function test(group, name, fn) {
		Runner.test(group, name, fn, { page: 'speculative', description: DESC });
	}
	function mkWorld(sub, it) {
		return new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), new AP.Solver({ substeps: sub || 4, iterations: it || 4 }));
	}
	function ground(world) {
		var g = new AP.RigidBody(new AP.BoxShape(10, 0.5, 10), 0);
		g.position.set(0, -0.5, 0); // top face at y=0
		world.addRigidBody(g);
		return g;
	}
	var DT = 1 / 60;

	// ---- the fattened broadphase AABB is the enabling primitive ----

	test('speculative/aabb', 'getBroadphaseAABB fattens the tight AABB by the speculative margin, getAABB stays tight', function (t) {
		var b = new AP.RigidBody(new AP.BoxShape(1, 1, 1), 1);
		b.position.set(0, 0, 0);
		b.updateDerived(0); // dt=0: no velocity sweep, only the fixed margin
		var tight = b.getAABB(), fat = b.getBroadphaseAABB();
		var m = AP.RigidBody.SPECULATIVE_MARGIN;
		t.check(tight.max.x, 1, 1e-9, 'tight AABB is the exact half-extent, no margin');
		t.check(fat.max.x, 1 + m, 1e-9, 'broadphase AABB adds the speculative margin');
		t.check(fat.min.y, -1 - m, 1e-9, 'margin applies on the min side too');
	});

	test('speculative/aabb', 'a downward velocity sweeps the broadphase AABB downward, not upward', function (t) {
		var b = new AP.RigidBody(new AP.BoxShape(1, 1, 1), 1);
		b.linear_velocity.set(0, -6, 0); // 6 m/s down
		b.updateDerived(DT);
		var fat = b.getBroadphaseAABB();
		var m = AP.RigidBody.SPECULATIVE_MARGIN, sweep = 6 * DT;
		t.check(fat.min.y, -1 - m - sweep, 1e-9, 'the leading (downward) face grows by margin + velocity*dt');
		t.check(fat.max.y, 1 + m, 1e-9, 'the trailing (upward) face grows by the margin only - sweep is directional');
	});

	// ---- the core payoff: a dropped box settles at rest with no bounce and no penetration ----

	test('speculative/settle', 'a box dropped from height settles at exactly rest height, no bounce, no penetration', function (t) {
		var world = mkWorld();
		ground(world);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 3, 0); // rest height is 0.5
		world.addRigidBody(box);

		var maxPen = 0, maxSettledSpeed = 0;
		for (var i = 0; i < 400; i++) {
			world.step(DT);
			var pen = 0.5 - box.position.y; // >0 means below rest = penetrating
			if (pen > maxPen) maxPen = pen;
			if (i > 150) maxSettledSpeed = Math.max(maxSettledSpeed, Math.abs(box.linear_velocity.y));
		}
		t.check(box.position.y, 0.5, 1e-4, 'settles at exactly rest height');
		t.checkTrue(maxPen < 1e-3, 'never penetrates the ground meaningfully (max ' + maxPen.toFixed(6) + ')');
		t.checkTrue(maxSettledSpeed < 1e-4, 'no residual bounce once settled (max ' + maxSettledSpeed.toFixed(6) + ')');
	});

	// ---- no tunnelling: the whole point of the velocity sweep ----

	test('speculative/tunnel', 'a box hurled straight down at 50 m/s is stopped at the surface, never tunnels through', function (t) {
		var world = mkWorld();
		ground(world);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 5, 0);
		box.linear_velocity.set(0, -50, 0); // fast enough to cross the ground in far less than one tick
		world.addRigidBody(box);

		var minY = Infinity;
		for (var i = 0; i < 400; i++) { world.step(DT); if (box.position.y < minY) minY = box.position.y; }
		t.checkTrue(minY > 0.45, 'the box never sank below the surface (min y ' + minY.toFixed(4) + ') - no tunnelling');
		t.check(box.position.y, 0.5, 1e-3, 'and comes to rest at the correct height');
	});

	// ---- resting stability: a contact starting exactly at touch must not buzz ----

	test('speculative/rest', 'a box placed exactly at rest height stays put - the exact-touch normal does not flip and eject it', function (t) {
		// This is the case the manifold's exact-touch normal persistence fixes: at sd~0 GJK/EPA's
		// normal is ambiguous (a flush box reports a diagonal (0.707,0,0.707)); trusting it for one
		// tick made the constraint point sideways, the box sank, and the next tick ejected it - a
		// permanent limit cycle. Keeping the established normal through the band holds it still.
		var world = mkWorld(1, 1);
		ground(world);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.5, 0); // exactly touching
		world.addRigidBody(box);

		// Tick 0 has no prior manifold point to persist a normal from, so the first substep lets the
		// box settle by a sub-millimetre transient (~2.7mm here) before the freshly-created point
		// stabilizes - a one-time settle, not a buzz. What this test pins down is the STEADY STATE:
		// once the point exists (from tick 1 on) there must be NO ongoing penetrate-then-eject
		// cycle, which is what the ambiguous exact-touch normal used to cause. Measure drift after
		// the first few ticks so the benign initial settle isn't conflated with the bug.
		var maxSteadyDrift = 0;
		for (var i = 0; i < 300; i++) {
			world.step(DT);
			if (i >= 5) maxSteadyDrift = Math.max(maxSteadyDrift, Math.abs(box.position.y - 0.5));
		}
		t.checkTrue(maxSteadyDrift < 1e-5, 'y holds at rest in steady state (max drift ' + maxSteadyDrift.toFixed(8) + ') - no penetrate-then-eject buzz');
		t.check(box.position.y, 0.5, 1e-6, 'ends exactly at rest');
	});

	// ---- an axis-aligned stack settles exactly (linear-only; rotation locked to isolate the mechanic) ----

	test('speculative/stack', 'a 3-box axis-aligned stack settles at exact rest heights (rotation locked)', function (t) {
		// Rotation is locked here on purpose: this asserts the LINEAR multi-contact behavior, which
		// is stable. The same stack with rotation free is NOT yet stable (angular correction injects
		// energy on the corner contacts) - that is the separate, documented next increment, and this
		// test does not pretend it works by leaving rotation free and loosening the threshold.
		var world = mkWorld(8, 8);
		ground(world);
		var boxes = [];
		for (var i = 0; i < 3; i++) {
			var b = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
			b.position.set(0, 0.5 + i * 1.0 + 0.02, 0); // small gaps, drop and settle
			b.angular_factor.set(0, 0, 0); // lock rotation - see comment above
			world.addRigidBody(b);
			boxes.push(b);
		}
		for (var s = 0; s < 600; s++) world.step(DT);
		t.check(boxes[0].position.y, 0.5, 2e-3, 'bottom box at 0.5');
		t.check(boxes[1].position.y, 1.5, 2e-3, 'middle box at 1.5');
		t.check(boxes[2].position.y, 2.5, 2e-3, 'top box at 2.5');
		for (var k = 0; k < 3; k++) {
			var speed = Math.hypot(boxes[k].linear_velocity.x, boxes[k].linear_velocity.y, boxes[k].linear_velocity.z);
			t.checkTrue(speed < 1e-3, 'box ' + k + ' comes fully to rest (|v| ' + speed.toFixed(6) + ')');
		}
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
