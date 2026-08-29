(function (Runner, PBF, ActionPhysics) {

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

	function finalGate(t, total) {
		var tick0 = 0;
		t.onTick(function (world, tick) { tick0 = tick; });
		return function () { return tick0 >= total; };
	}

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
	typeof module !== 'undefined' && module.exports ? require('../_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
