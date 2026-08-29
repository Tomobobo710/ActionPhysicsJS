/**
 * Tom's Suite — CHARACTER-vs-CHARACTER collision tests (CC1-CC3).
 *
 * Two independent FPSCharacterControllers in one world. A player is a WALL to another player: full
 * body-block via the other player's ghost (findBlock forces keep=0 for a body tagged isCharacterGhost),
 * never a soft mass-yield push like an ordinary object. These tests drive TWO controllers per tick —
 * _util_fps.js's PBF.drive() only brackets one, so this file rolls its own two-controller bracket
 * (same beginStep -> world.step -> endStep ordering PBF.drive uses internally).
 */
(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	// Drive TWO controllers per tick with the same bracket ordering PBF.drive() uses for one:
	// onTick fires beginStep for both (before world.step, which the runner owns), and endStep for
	// both is patched into evalTick so it runs immediately after world.step, before any expectation reads state.
	function driveTwo(t, a, b, cmdForA, cmdForB) {
		var lastEndTick = 0;
		function ensureEnded(tick) {
			if (tick > lastEndTick) { a.endStep(PBF.DT); b.endStep(PBF.DT); lastEndTick = tick; }
		}
		t.onTick(function (world, tick) {
			var cmdA = (cmdForA ? cmdForA(tick) : null) || {};
			var cmdB = (cmdForB ? cmdForB(tick) : null) || {};
			a.beginStep(cmdA, PBF.DT);
			b.beginStep(cmdB, PBF.DT);
		});
		var realEvalTick = t.evalTick;
		t.evalTick = function (world, tick) {
			ensureEnded(tick);
			return realEvalTick(world, tick);
		};
	}

	// "Never violates" / final-measurement gate. t.expect flips a criterion PASS the FIRST tick its
	// predicate returns true, and never re-checks it after - correct for a predicate that starts
	// false and should eventually become and STAY true (settling), wrong for a predicate like
	// "hasn't passed through yet" that is trivially true before the real test action even starts
	// (tick 1, nobody has moved) and would latch a pass immediately regardless of what happens on
	// every later tick. This was a REAL, confirmed-live bug in this exact file: A sprinting straight
	// through B for 600 ticks (B never moves, A ends up 100+ units past B) was reported as a pass,
	// because "A never passes B" read true at tick 1 and t.expect never looked again. Gate every
	// state-must-hold-the-whole-run assertion in this file on the run's LAST tick instead, matching
	// fps-knockback.js's own finalGate (same bug class, already fixed there).
	function finalGate(t, total) {
		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		return function () { return tick0 >= total; };
	}

	// ---- CC1: sprinting into a stationary player is a hard body-block (no pass-through) ----
	PBF.scaleTest('fps/character-collision', 'CC1', 'sprint into player hard-blocks (no pass-through)', function (t, S) {
		var w = S.flat();
		var a = S.feetSpawn(w, 0, -2, {});
		var b = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, a, [b.body, b._ghost]);

		var passedThrough = false, bMoved = 0, bz0 = b.body.position.z;
		driveTwo(t, a, b,
			function (tick) { return tick > 30 ? { forward: 1, sprint: true } : {}; },
			function () { return {}; }
		);
		t.onTick(function () {
			if (a.body.position.z > b.body.position.z + S.sc(0.1)) passedThrough = true;
			bMoved = Math.abs(b.body.position.z - bz0);
		});

		var done = finalGate(t, 600);
		t.log('Sprint A straight at stationary B for 9+ seconds (600 ticks) — a WALL never gets walked through, no matter how long you push.');
		t.expect('A never passes B (no walk-through)', function () {
			if (!done()) return false;
			return { ok: !passedThrough, detail: 'passedThrough=' + passedThrough + ' A.z=' + a.body.position.z.toFixed(3) + ' B.z=' + b.body.position.z.toFixed(3) };
		});
		t.expect('B never moves (full block, not a soft push)', function () {
			if (!done()) return false;
			return { ok: bMoved < 0.01, detail: 'bMoved=' + bMoved.toFixed(4) };
		});
		t.simulate(w, 600);
	}, { page: 'fps/character-collision', steps: 600, description: 'Sprint into a stationary player for 9+ seconds; must stay blocked the whole time — no eventual slip-through, no displacement of the other player.' });

	// ---- CC2: a light bump (short tap) also fails to displace or pass through ----
	PBF.scaleTest('fps/character-collision', 'CC2', 'short bump does not pass through or move player', function (t, S) {
		var w = S.flat();
		var a = S.feetSpawn(w, 0, -2, {});
		var b = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, a, [b.body, b._ghost]);

		var bz0 = b.body.position.z, bMoved = 0, passedThrough = false;
		driveTwo(t, a, b,
			function (tick) { return (tick > 30 && tick < 45) ? { forward: 1 } : {}; },
			function () { return {}; }
		);
		t.onTick(function () {
			if (a.body.position.z > b.body.position.z + S.sc(0.1)) passedThrough = true;
			bMoved = Math.abs(b.body.position.z - bz0);
		});

		var done = finalGate(t, 90);
		t.log('A single short forward tap into B (unlike push.js P1, which expects a light OBJECT to visibly roll) — a player must not budge at all.');
		t.expect('B does not move from a bump (full block)', function () {
			if (!done()) return false;
			return { ok: bMoved < 0.01, detail: 'bMoved=' + bMoved.toFixed(4) };
		});
		t.expect('A does not pass through B', function () {
			if (!done()) return false;
			return { ok: !passedThrough, detail: 'passedThrough=' + passedThrough };
		});
		t.simulate(w, 90);
	}, { page: 'fps/character-collision', steps: 90, description: 'A short tap into another player must not move them or let you pass through — unlike a pushable object.' });

	// ---- CC3: both players sprinting at each other both hard-stop, neither passes the midpoint into the other's start side ----
	PBF.scaleTest('fps/character-collision', 'CC3', 'head-on sprint both block, no swap-through', function (t, S) {
		var w = S.flat();
		var a = S.feetSpawn(w, 0, -3, {});
		var b = S.feetSpawn(w, 0, 3, {});
		PBF.renderables(t, a, [b.body, b._ghost]);

		var swapped = false;
		driveTwo(t, a, b,
			function (tick) { return tick > 30 ? { forward: 1, sprint: true } : {}; },
			function (tick) { return tick > 30 ? { forward: -1, sprint: true } : {}; }
		);
		t.onTick(function () {
			// If they ever fully swap sides, that's a walk-through in either direction.
			if (a.body.position.z > 3 || b.body.position.z < -3) swapped = true;
		});

		var done = finalGate(t, 300);
		t.log('A and B sprint head-on at each other from opposite sides; both must hard-stop at contact — neither ends up on the other'
			+ '\'s starting side.');
		t.expect('neither player swaps to the other\'s starting side', function () {
			if (!done()) return false;
			return { ok: !swapped, detail: 'swapped=' + swapped + ' A.z=' + a.body.position.z.toFixed(3) + ' B.z=' + b.body.position.z.toFixed(3) };
		});
		t.simulate(w, 300);
	}, { page: 'fps/character-collision', steps: 300, description: 'Two players sprint head-on into each other; both must hard-stop at contact with neither passing the other.' });

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
