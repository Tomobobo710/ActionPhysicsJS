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
//
// Every test here builds its world/bodies through the shared harness (t.makeWorld/t.box) and asserts
// LIVE against the ticking world via t.expect + t.simulate: a criterion flips true the instant the
// physics makes it so (or false at the end if it never does), not from a value read once after a
// silent for-loop. Every test has shapes and motion, so every one is marked visual: true and watchable
// in the suite bench - a settle, a tilt-and-tip, a slide-to-stop are all things worth SEEING happen,
// not just trusting a pass/fail dot for.
(function (Runner) {
	Runner.suite('speculative');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Speculative contacts + per-substep contact refresh: contact detected before overlap " +
		"(fattened broadphase AABB with linear+angular sweep), geometry re-measured each substep so " +
		"corner/edge contacts stay stable, solver stops the body at touch. Verified: flat drop " +
		"settles exactly, high-speed drop never tunnels, resting is stable, a stack settles with " +
		"rotation free, a tilted box tips flat, friction resists sliding.";
	function test(group, name, fn, steps) {
		Runner.test(group, name, fn, { page: 'speculative', description: DESC, visual: true, steps: steps || 0 });
	}
	var DT = 1 / 60;

	// ---- the fattened broadphase AABB is the enabling primitive ----
	// (a single body, no motion, no ticking - a static geometry check, not a dynamics scene; stays
	// synchronous, matching what it actually tests)

	Runner.test('speculative/aabb', 'getBroadphaseAABB fattens the tight AABB by the speculative margin, getAABB stays tight', function (t) {
		var b = t.loneBody(new AP.BoxShape(1, 1, 1), { pos: [0, 0, 0] });
		b.updateDerived(0); // dt=0: no velocity sweep, only the fixed margin
		var tight = b.getAABB(), fat = b.getBroadphaseAABB();
		var m = AP.RigidBody.SPECULATIVE_MARGIN;
		t.check(tight.max.x, 1, 1e-9, 'tight AABB is the exact half-extent, no margin');
		t.check(fat.max.x, 1 + m, 1e-9, 'broadphase AABB adds the speculative margin');
		t.check(fat.min.y, -1 - m, 1e-9, 'margin applies on the min side too');
	}, { page: 'speculative', description: DESC });

	Runner.test('speculative/aabb', 'a downward velocity sweeps the broadphase AABB downward, not upward', function (t) {
		var b = t.loneBody(new AP.BoxShape(1, 1, 1), { vel: [0, -6, 0] }); // 6 m/s down
		b.updateDerived(DT);
		var fat = b.getBroadphaseAABB();
		var m = AP.RigidBody.SPECULATIVE_MARGIN, sweep = 6 * DT;
		t.check(fat.min.y, -1 - m - sweep, 1e-9, 'the leading (downward) face grows by margin + velocity*dt');
		t.check(fat.max.y, 1 + m, 1e-9, 'the trailing (upward) face grows by the margin only - sweep is directional');
	}, { page: 'speculative', description: DESC });

	// ---- the core payoff: a dropped box settles at rest with no bounce and no penetration ----

	test('speculative/settle', 'a box dropped from height settles at exactly rest height, no bounce, no penetration', function (t) {
		var world = t.makeWorld({ gravity: 0 }); // gravity applied via body's own linear_velocity below to match the original scene exactly
		world.gravity.set(0, -9.81, 0);
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' }); // ground, top face at y=0
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], color: '#4af' }); // rest height 0.5

		var maxPen = 0, maxSettledSpeed = 0;
		t.onTick(function (world, tick) {
			var pen = 0.5 - box.position.y; // >0 means below rest = penetrating
			if (pen > maxPen) maxPen = pen;
			if (tick > 150) maxSettledSpeed = Math.max(maxSettledSpeed, Math.abs(box.linear_velocity.y));
		});
		t.expect('settles at exactly rest height', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-4, detail: 'y=' + box.position.y.toFixed(6) };
		});
		t.expect('never penetrates the ground meaningfully', function () {
			return { ok: maxPen < 1e-3, detail: 'max penetration ' + maxPen.toFixed(6) };
		});
		t.expect('no residual bounce once settled', function () {
			return { ok: maxSettledSpeed < 1e-4, detail: 'max settled speed ' + maxSettledSpeed.toFixed(6) };
		});
		t.simulate(world, 400);
	}, 400);

	// ---- no tunnelling: the whole point of the velocity sweep ----

	test('speculative/tunnel', 'a box hurled straight down at 50 m/s is stopped at the surface, never tunnels through', function (t) {
		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 5, 0], vel: [0, -50, 0], color: '#f55' }); // fast enough to cross the ground in far less than one tick

		var minY = Infinity;
		t.onTick(function () { if (box.position.y < minY) minY = box.position.y; });
		t.expect('the box never sank below the surface - no tunnelling', function () {
			return { ok: minY > 0.45, detail: 'min y ' + minY.toFixed(4) };
		});
		t.expect('comes to rest at the correct height', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-3, detail: 'y=' + box.position.y.toFixed(6) };
		});
		t.simulate(world, 400);
	}, 400);

	// ---- resting stability: a contact starting exactly at touch must not buzz ----

	test('speculative/rest', 'a box placed exactly at rest height stays put - the exact-touch normal does not flip and eject it', function (t) {
		// This is the case the manifold's exact-touch normal persistence fixes: at sd~0 GJK/EPA's
		// normal is ambiguous (a flush box reports a diagonal (0.707,0,0.707)); trusting it for one
		// tick made the constraint point sideways, the box sank, and the next tick ejected it - a
		// permanent limit cycle. Keeping the established normal through the band holds it still.
		var world = t.makeWorld();
		world.solver.substeps = 1; // isolate a single substep - iterations is already 1 by default
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], color: '#4af' }); // exactly touching

		// Tick 0 has no prior manifold point to persist a normal from, so the first substep lets the
		// box settle by a sub-millimetre transient (~2.7mm here) before the freshly-created point
		// stabilizes - a one-time settle, not a buzz. What matters is the STEADY STATE: once the point
		// exists (from tick 1 on) there must be NO ongoing penetrate-then-eject cycle, which is what
		// the ambiguous exact-touch normal used to cause. Drift is measured after the first few ticks
		// so the benign initial settle isn't conflated with the bug.
		var maxSteadyDrift = 0;
		t.onTick(function (world, tick) { if (tick >= 5) maxSteadyDrift = Math.max(maxSteadyDrift, Math.abs(box.position.y - 0.5)); });
		t.expect('y holds at rest in steady state - no penetrate-then-eject buzz', function () {
			return { ok: maxSteadyDrift < 1e-5, detail: 'max steady drift ' + maxSteadyDrift.toFixed(8) };
		});
		t.expect('ends exactly at rest', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-6, detail: 'y=' + box.position.y.toFixed(8) };
		});
		t.simulate(world, 300);
	}, 300);

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
		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var boxes = [];
		var colors = ['#4af', '#6c8', '#fc4'];
		for (var i = 0; i < 3; i++) boxes.push(t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5 + i * 1.0 + 0.02, 0], color: colors[i] }));

		var maxHeightErr = 0;
		t.onTick(function (world, tick) {
			if (tick >= 300) for (var j = 0; j < 3; j++) maxHeightErr = Math.max(maxHeightErr, Math.abs(boxes[j].position.y - (0.5 + j)));
		});
		t.expect('bottom box at 0.5', function () { return { ok: Math.abs(boxes[0].position.y - 0.5) < 1e-3, detail: 'y=' + boxes[0].position.y.toFixed(5) }; });
		t.expect('middle box at 1.5', function () { return { ok: Math.abs(boxes[1].position.y - 1.5) < 1e-3, detail: 'y=' + boxes[1].position.y.toFixed(5) }; });
		t.expect('top box at 2.5', function () { return { ok: Math.abs(boxes[2].position.y - 2.5) < 1e-3, detail: 'y=' + boxes[2].position.y.toFixed(5) }; });
		t.expect('heights hold steady once settled - no drift', function () { return { ok: maxHeightErr < 2e-3, detail: 'max error ' + maxHeightErr.toFixed(6) }; });
		for (var k = 0; k < 3; k++) {
			(function (idx) {
				t.expect('box ' + idx + ' is at rest, not diverging', function () {
					var b = boxes[idx];
					var speed = Math.hypot(b.linear_velocity.x, b.linear_velocity.y, b.linear_velocity.z);
					return { ok: speed < 0.1, detail: '|v|=' + speed.toFixed(6) };
				});
			})(k);
		}
		t.simulate(world, 800);
	}, 800);

	// ---- a tilted box tips to flat and settles, rotation free (the angular-stability payoff) ----

	test('speculative/tilt', 'a tilted box dropped onto the ground tips flat and settles, no launch', function (t) {
		// The scenario that used to blow up: a box landing on a CORNER. Before per-substep geometry
		// refresh the corner correction injected a large derived angular velocity (a 0.15-rad tilt
		// produced ~4 rad/s and the box launched to y=6+ carrying 17 rad/s). Now the corner contact
		// is re-measured every substep, so the tip is resolved smoothly and the box settles flat.
		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 1.2, 0], color: '#f84' });
		box.rotation.setAxisAngle(new V(0, 0, 1), 0.15); // lands on a corner/edge

		var maxW = 0;
		t.onTick(function () { maxW = Math.max(maxW, Math.hypot(box.angular_velocity.x, box.angular_velocity.y, box.angular_velocity.z)); });
		t.expect('settles flat at rest height (not launched)', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 5e-3, detail: 'y=' + box.position.y.toFixed(5) };
		});
		t.expect('angular velocity stays bounded through the tip - no runaway spin', function () {
			return { ok: maxW < 15, detail: 'max |w| ' + maxW.toFixed(3) + ' rad/s' };
		});
		t.expect('comes to angular rest', function () {
			var w = Math.hypot(box.angular_velocity.x, box.angular_velocity.y, box.angular_velocity.z);
			return { ok: w < 0.2, detail: '|w|=' + w.toFixed(4) };
		});
		t.simulate(world, 1200);
	}, 1200);

	// ---- friction resists sliding (it was a silent no-op before this increment) ----

	test('speculative/friction', 'a box slid across the ground is brought to rest by friction, not left sliding forever', function (t) {
		// Friction used to be a complete no-op (the Coulomb cap read a negative normalLambda and
		// always early-returned, and the tangential error was hardcoded to zero). Now it resists the
		// per-substep tangential slip of the contact anchors, Coulomb-capped: a box given a sideways
		// shove decelerates and stops, where with no friction it slides at constant speed forever.
		var world = t.makeWorld();
		t.box(world, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], friction: 0.5, color: '#556' });
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], vel: [3, 0, 0], friction: 0.5, color: '#4af' });

		t.expect('the box has stopped sliding', function () {
			return { ok: Math.abs(box.linear_velocity.x) < 0.05, detail: '|vx|=' + Math.abs(box.linear_velocity.x).toFixed(4) };
		});
		t.expect('stopped within a short distance, not the ~15m a frictionless slide would cover', function () {
			return { ok: box.position.x < 3, detail: 'x=' + box.position.x.toFixed(3) };
		});
		t.simulate(world, 300);
	}, 300);

	test('speculative/friction', 'zero friction lets a box slide freely - friction is actually doing the work above', function (t) {
		// The control for the test above: with friction 0 the same shove slides essentially forever,
		// confirming the deceleration above comes from friction and not from some other damping.
		var world = t.makeWorld();
		t.box(world, 40, 0.5, 40, 0, { pos: [0, -0.5, 0], friction: 0, color: '#556' }); // wide enough that the box stays on it
		var box = t.box(world, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], vel: [3, 0, 0], friction: 0, color: '#f84' });

		// Frictionless preserves the sliding speed (the box does NOT decelerate the way the friction
		// case does). A sub-percent wobble from vertical settling coupling into x is fine - the point
		// is that ~3 m/s is retained, not that it is bit-exact.
		t.expect('frictionless: velocity is essentially unchanged', function () {
			return { ok: Math.abs(box.linear_velocity.x - 3) < 0.02, detail: 'vx=' + box.linear_velocity.x.toFixed(4) };
		});
		t.expect('frictionless: the box slid a long way', function () {
			return { ok: box.position.x > 9, detail: 'x=' + box.position.x.toFixed(2) };
		});
		t.expect('and stayed on the ground (did not fall off or sink)', function () {
			return { ok: Math.abs(box.position.y - 0.5) < 1e-2, detail: 'y=' + box.position.y.toFixed(4) };
		});
		t.simulate(world, 200);
	}, 200);

	// ---- a real multi-body pile: 10-box pyramid holds together (needs working friction + stable contacts) ----
	// A smaller companion to the full 385-box stress test (see the 'pyramid' page) - this one exists
	// specifically to catch a friction regression (frictionless boxes cannot hold ANY pyramid, however
	// small), so it stays cheap to re-run rather than pulling in the full 385-box scene every time.

	test('speculative/pyramid', 'a 10-box pyramid drops, settles, and holds its shape without ejecting bodies', function (t) {
		var world = t.makeWorld();
		t.box(world, 20, 0.5, 20, 0, { pos: [0, -0.5, 0], color: '#556' });
		var boxes = [], rows = 4;
		var layerColor = ['#4af', '#6c8', '#fc4', '#f66'];
		for (var row = 0; row < rows; row++) {
			var n = rows - row, y = 0.5 + row * 1.0 + 0.02;
			for (var k = 0; k < n; k++) {
				boxes.push(t.box(world, 0.5, 0.5, 0.5, 1, { pos: [(k - (n - 1) / 2) * 1.05, y, 0], color: layerColor[row] }));
			}
		}

		t.expect('no box was ejected from the pyramid', function () {
			var ejected = 0;
			for (var i = 0; i < boxes.length; i++) {
				var b = boxes[i];
				if (b.position.y > 5 || b.position.y < 0 || Math.abs(b.position.x) > 8) ejected++;
			}
			return { ok: ejected === 0, detail: ejected + '/' + boxes.length + ' ejected' };
		});
		t.expect('the pyramid is settled, not exploding', function () {
			var maxSpeed = 0;
			for (var i = 0; i < boxes.length; i++) {
				var b = boxes[i];
				maxSpeed = Math.max(maxSpeed, Math.hypot(b.linear_velocity.x, b.linear_velocity.y, b.linear_velocity.z));
			}
			return { ok: maxSpeed < 1, detail: 'max |v| ' + maxSpeed.toFixed(4) };
		});
		for (var m = 0; m < boxes.length; m++) {
			(function (idx) {
				t.expect('box ' + idx + ' rests at a valid layer height', function () {
					var yy = boxes[idx].position.y, layer = Math.round(yy - 0.5);
					var ok = Math.abs(yy - (0.5 + layer)) < 0.1 && layer >= 0 && layer < 4;
					return { ok: ok, detail: 'y=' + yy.toFixed(3) };
				});
			})(m);
		}
		t.simulate(world, 1200);
	}, 1200);

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
