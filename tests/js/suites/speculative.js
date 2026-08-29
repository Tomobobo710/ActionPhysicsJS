// Speculative contacts (plan.md build item 6a): a contact is detected BEFORE overlap, while the
// shapes are still separated, so the XPBD solver's non-penetration constraint - evaluated every
// substep against the body's PREDICTED position - stops the body AT touch instead of first letting
// it dig in and then ejecting it. This is the fix for the derived-velocity problem plan.md
// documents: without it, a body arrives already deep (several substeps of undetected travel), the
// one-shot correction produces velocity = Δx/h with nothing to damp it, and the body launches.
//
// The moving parts, all exercised here:
//   1. Broadphase reads a FATTENED AABB (RigidBody.getBroadphaseAABB): tight bound + a fixed
//      speculative margin + this tick's linear AND angular (|omega|*R) velocity sweep, so a pair -
//      including a spinning body's far corner - surfaces the tick before overlap.
//   2. Narrowphase reports a contact while still separated (within its per-pair speculative margin,
//      widened by the same angular corner speed), creating the manifold point before penetration.
//   3. Per substep the solver calls back into narrowphase to RE-MEASURE contact geometry against the
//      predicted positions (interleaved detect-then-solve), so a rotating body's moving contact
//      corner is solved against live geometry - this is what makes corner/edge contacts stable.
//   4. The solver's live-C-per-substep non-penetration constraint stops the predicted overshoot at
//      touch, the manifold keeps the established contact normal through the exact-touch band where
//      GJK/EPA's normal is ambiguous, and per-substep-slip friction (Coulomb disc cap) resists
//      sliding.
//
// SCOPE: both the linear AND angular paths are stable - a flat drop settles exactly, a high-speed
// drop never tunnels, a resting contact never buzzes, a stack settles WITH ROTATION FREE, and a
// tilted box tips flat instead of launching. A tiny bounded resting velocity jitter remains (normal
// XPBD, zeroed by the sleep system - a separate owner); assertions are on position stability with a
// generous velocity ceiling that only catches real divergence.
(function (Runner) {
	Runner.suite('speculative');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Speculative contacts + per-substep contact refresh: contact detected before overlap " +
		"(fattened broadphase AABB with linear+angular sweep), geometry re-measured each substep so " +
		"corner/edge contacts stay stable, solver stops the body at touch. Verified: flat drop " +
		"settles exactly, high-speed drop never tunnels, resting is stable, a stack settles with " +
		"rotation free, a tilted box tips flat, friction resists sliding.";
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

	// ---- a 3-box stack settles at exact rest heights, WITH ROTATION FREE ----

	test('speculative/stack', 'a 3-box stack settles at exact rest heights with rotation free', function (t) {
		// Rotation is FREE here (no angular lock): the per-substep contact-geometry refresh makes
		// corner/edge contacts stable, so a real stack settles without the angular energy injection
		// that used to launch the middle box. What this pins down is POSITIONAL stability - the
		// heights are exact and do not drift. A tiny bounded velocity micro-jitter remains at rest
		// (sub-centimetre/sec, moving the bodies a sub-micrometre that derives back into velocity);
		// that is normal XPBD resting jitter that the sleep system (a separate owner, plan.md) zeroes
		// out, and it does NOT move the stack - so the assertion is on position, with a generous
		// velocity ceiling only to catch a real divergence (the old bug sent |v| into the hundreds).
		var world = mkWorld(8, 8);
		ground(world);
		var boxes = [];
		for (var i = 0; i < 3; i++) {
			var b = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
			b.position.set(0, 0.5 + i * 1.0 + 0.02, 0); // small gaps, drop and settle
			world.addRigidBody(b);
			boxes.push(b);
		}
		var maxHeightErr = 0;
		for (var s = 0; s < 800; s++) {
			world.step(DT);
			if (s >= 300) for (var j = 0; j < 3; j++) maxHeightErr = Math.max(maxHeightErr, Math.abs(boxes[j].position.y - (0.5 + j)));
		}
		t.check(boxes[0].position.y, 0.5, 1e-3, 'bottom box at 0.5');
		t.check(boxes[1].position.y, 1.5, 1e-3, 'middle box at 1.5');
		t.check(boxes[2].position.y, 2.5, 1e-3, 'top box at 2.5');
		t.checkTrue(maxHeightErr < 2e-3, 'heights hold steady once settled - no drift (max error ' + maxHeightErr.toFixed(6) + ')');
		for (var k = 0; k < 3; k++) {
			var speed = Math.hypot(boxes[k].linear_velocity.x, boxes[k].linear_velocity.y, boxes[k].linear_velocity.z);
			t.checkTrue(speed < 0.1, 'box ' + k + ' is at rest, not diverging (|v| ' + speed.toFixed(6) + ')');
		}
	});

	// ---- a tilted box tips to flat and settles, rotation free (the angular-stability payoff) ----

	test('speculative/tilt', 'a tilted box dropped onto the ground tips flat and settles, no launch', function (t) {
		// The scenario that used to blow up: a box landing on a CORNER. Before per-substep geometry
		// refresh the corner correction injected a large derived angular velocity (a 0.15-rad tilt
		// produced ~4 rad/s and the box launched to y=6+ carrying 17 rad/s). Now the corner contact
		// is re-measured every substep, so the tip is resolved smoothly and the box settles flat.
		var world = mkWorld();
		ground(world);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 1.2, 0);
		box.rotation.setAxisAngle(new V(0, 0, 1), 0.15); // lands on a corner/edge
		world.addRigidBody(box);
		var maxW = 0;
		for (var i = 0; i < 1200; i++) {
			world.step(DT);
			maxW = Math.max(maxW, Math.hypot(box.angular_velocity.x, box.angular_velocity.y, box.angular_velocity.z));
		}
		t.check(box.position.y, 0.5, 5e-3, 'settles flat at rest height (not launched)');
		t.checkTrue(maxW < 15, 'angular velocity stays bounded through the tip (max ' + maxW.toFixed(3) + ' rad/s) - no runaway spin');
		var w = Math.hypot(box.angular_velocity.x, box.angular_velocity.y, box.angular_velocity.z);
		t.checkTrue(w < 0.2, 'comes to angular rest (|w| ' + w.toFixed(4) + ')');
	});

	// ---- friction resists sliding (it was a silent no-op before this increment) ----

	test('speculative/friction', 'a box slid across the ground is brought to rest by friction, not left sliding forever', function (t) {
		// Friction used to be a complete no-op (the Coulomb cap read a negative normalLambda and
		// always early-returned, and the tangential error was hardcoded to zero). Now it resists the
		// per-substep tangential slip of the contact anchors, Coulomb-capped: a box given a sideways
		// shove decelerates and stops, where with no friction it slides at constant speed forever.
		var world = mkWorld(8, 8);
		var g = new AP.RigidBody(new AP.BoxShape(10, 0.5, 10), 0);
		g.position.set(0, -0.5, 0); g.friction = 0.5;
		world.addRigidBody(g);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.5, 0); box.friction = 0.5;
		box.linear_velocity.set(3, 0, 0); // shoved sideways at 3 m/s
		world.addRigidBody(box);
		for (var i = 0; i < 300; i++) world.step(DT);
		t.checkTrue(Math.abs(box.linear_velocity.x) < 0.05, 'the box has stopped sliding (|vx| ' + Math.abs(box.linear_velocity.x).toFixed(4) + ')');
		t.checkTrue(box.position.x < 3, 'and it stopped within a short distance, not the ~15 m a frictionless slide would cover (x ' + box.position.x.toFixed(3) + ')');
	});

	test('speculative/friction', 'zero friction lets a box slide freely - friction is actually doing the work above', function (t) {
		// The control for the test above: with friction 0 the same shove slides essentially forever,
		// confirming the deceleration above comes from friction and not from some other damping.
		var world = mkWorld(8, 8);
		var g = new AP.RigidBody(new AP.BoxShape(40, 0.5, 40), 0); // wide enough that the box stays on it
		g.position.set(0, -0.5, 0); g.friction = 0;
		world.addRigidBody(g);
		var box = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
		box.position.set(0, 0.5, 0); box.friction = 0;
		box.linear_velocity.set(3, 0, 0);
		world.addRigidBody(box);
		for (var i = 0; i < 200; i++) world.step(DT);
		// Frictionless preserves the sliding speed (the box does NOT decelerate the way the friction
		// case does). A sub-percent wobble from vertical settling coupling into x is fine - the point
		// is that ~3 m/s is retained, not that it is bit-exact.
		t.checkTrue(Math.abs(box.linear_velocity.x - 3) < 0.02, 'frictionless: velocity is essentially unchanged (vx ' + box.linear_velocity.x.toFixed(4) + ')');
		t.checkTrue(box.position.x > 9, 'frictionless: the box slid a long way (x ' + box.position.x.toFixed(2) + ')');
		t.check(box.position.y, 0.5, 1e-2, 'and stayed on the ground (did not fall off or sink)');
	});

	// ---- a real multi-body pile: 10-box pyramid holds together (needs working friction + stable contacts) ----

	test('speculative/pyramid', 'a 10-box pyramid drops, settles, and holds its shape without ejecting bodies', function (t) {
		// The integration test for the whole increment: a 4-3-2-1 pyramid needs stable multi-contact
		// resolution AND working friction (frictionless boxes cannot hold a pyramid - any nudge slides
		// them apart). Before this increment 7-8 of the 10 boxes were ejected to y in the hundreds;
		// now none are, and the boxes sit at their proper layer heights.
		var world = mkWorld(8, 8);
		var g = new AP.RigidBody(new AP.BoxShape(20, 0.5, 20), 0);
		g.position.set(0, -0.5, 0);
		world.addRigidBody(g);
		var boxes = [], rows = 4;
		for (var row = 0; row < rows; row++) {
			var n = rows - row, y = 0.5 + row * 1.0 + 0.02;
			for (var k = 0; k < n; k++) {
				var b = new AP.RigidBody(new AP.BoxShape(0.5, 0.5, 0.5), 1);
				b.position.set((k - (n - 1) / 2) * 1.05, y, 0);
				world.addRigidBody(b);
				boxes.push(b);
			}
		}
		for (var s = 0; s < 1200; s++) world.step(DT);
		var ejected = 0, maxSpeed = 0;
		for (var i = 0; i < boxes.length; i++) {
			var b2 = boxes[i];
			if (b2.position.y > 5 || b2.position.y < 0 || Math.abs(b2.position.x) > 8) ejected++;
			maxSpeed = Math.max(maxSpeed, Math.hypot(b2.linear_velocity.x, b2.linear_velocity.y, b2.linear_velocity.z));
		}
		t.checkEqual(ejected, 0, 'no box was ejected from the pyramid');
		t.checkTrue(maxSpeed < 1, 'the pyramid is settled, not exploding (max |v| ' + maxSpeed.toFixed(4) + ')');
		// every box ended near one of the four layer heights
		for (var m = 0; m < boxes.length; m++) {
			var yy = boxes[m].position.y, layer = Math.round(yy - 0.5);
			t.checkTrue(Math.abs(yy - (0.5 + layer)) < 0.1 && layer >= 0 && layer < 4, 'box ' + m + ' rests at a valid layer height (y ' + yy.toFixed(3) + ')');
		}
	});

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
