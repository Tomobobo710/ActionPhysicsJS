(function (Runner, PBF, ActionPhysics) {

	function step(p, world, cmd) {
		p.beginStep(cmd || {}, PBF.DT);
		world.step(PBF.DT);
		p.endStep(PBF.DT);
	}

	PBF.scaleTest('fps/netcode', 'N1', 'movement deterministic', function (t, S) {
		var run = function () {
			var w = S.flat(); var p = S.feetSpawn(w, 0, 0, {});
			for (var tk = 0; tk < 100; tk++) step(p, w, { forward: 1, right: tk % 3 - 1, sprint: tk % 2 === 0, yaw: tk * 0.03 });
			return [p.body.position.x, p.body.position.y, p.body.position.z, p.body.linear_velocity.x, p.body.linear_velocity.z];
		};

		var w = S.flat(); var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			var tk = tick - 1;
			return { forward: 1, right: tk % 3 - 1, sprint: tk % 2 === 0, yaw: tk * 0.03 };
		});

		t.log('Run the same 100-tick command sequence twice; the two end states must match exactly.');
		t.expect('two runs produce identical end state', function () {
			var a = run(), b = run();
			var identical = a.every(function (v, i) { return v === b[i]; });
			return { ok: identical, detail: identical ? 'identical' : 'DIVERGED' };
		});
		t.simulate(w, 100);
	}, { page: 'fps/netcode', steps: 100, description: 'Determinism guard: the same command stream replayed twice in the same run yields identical player state.' });

	PBF.scaleTest('fps/netcode', 'N2', 'distant object doesnt alter player path', function (t, S) {
		var runP = function (withObject) {
			var w = S.flat();
			if (withObject) PBF.object(w, 5, 5, { x: 20, y: 0.5, z: 20 });
			var p = S.feetSpawn(w, 0, 0, {});
			for (var tk = 0; tk < 60; tk++) step(p, w, { forward: 1, sprint: true });
			return [p.body.position.x, p.body.position.z];
		};

		var w = S.flat(); PBF.object(w, 5, 5, { x: 20, y: 0.5, z: 20 });
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		PBF.drive(t, p, function () { return { forward: 1, sprint: true }; });

		t.log('Sprint forward with a heavy object parked ~28 units away; the player path must be identical to no-object.');
		t.expect('player end x identical with vs without object', function () {
			var a = runP(false), b = runP(true);
			return { ok: a[0] === b[0], detail: 'noObject=' + JSON.stringify(a) + ' withObject=' + JSON.stringify(b) };
		});
		t.expect('player end z identical with vs without object', function () {
			var a = runP(false), b = runP(true);
			return { ok: a[1] === b[1], detail: 'noObject=' + JSON.stringify(a) + ' withObject=' + JSON.stringify(b) };
		});
		t.simulate(w, 60);
	}, { page: 'fps/netcode', steps: 60, description: 'An object far from the player must not perturb the player path (no phantom coupling).' });

	PBF.scaleTest('fps/netcode', 'N3', 'resim deterministic with ghost', function (t, S) {
		var resimRun = function () {
			var w = S.flat(); PBF.object(w, 5, 5, { x: 0, y: 0.5, z: 3 });
			var p = S.feetSpawn(w, 0, 0, {});
			p.beginResim();
			for (var tk = 0; tk < 80; tk++) step(p, w, { forward: 1, sprint: true, yaw: tk * 0.02 });
			p.endResim();
			return [p.body.position.x, p.body.position.y, p.body.position.z, p.body.linear_velocity.x, p.body.linear_velocity.z];
		};

		var w = S.flat(); PBF.object(w, 5, 5, { x: 0, y: 0.5, z: 3 });
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		p.beginResim();
		PBF.drive(t, p, function (tick) {
			var tk = tick - 1;
			return { forward: 1, sprint: true, yaw: tk * 0.02 };
		});

		t.log('Replay an 80-tick command inside beginResim/endResim twice; reconciled player state must match exactly.');
		t.expect('two resim runs produce identical player state', function () {
			var a = resimRun(), b = resimRun();
			var identical = a.every(function (v, i) { return v === b[i]; });
			return { ok: identical, detail: identical ? 'identical' : 'DIVERGED ' + JSON.stringify(a) + ' vs ' + JSON.stringify(b) };
		});
		t.simulate(w, 80);
		p.endResim();
	}, { page: 'fps/netcode', steps: 80, description: 'Rollback determinism: replaying commands inside a resim gives an identical player path, ghost present.' });

	PBF.scaleTest('fps/netcode', 'N4', 'ghost driven during resim (pushes objects)', function (t, S) {
		var w = S.flat(); S.scaledObject(w, 1, 3, 1.3);
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);

		var gz0 = null, gzMoved = 0;
		PBF.drive(t, p, function (tick) {
			if (tick === 20) { gz0 = p._ghost.position.z; p.beginResim(); }
			if (tick > 20) {
				gzMoved = Math.abs(p._ghost.position.z - gz0);
				return { forward: 1 };
			}
			return {};
		});

		t.log('Settle 20 ticks, then walk into an object inside a resim; the ghost must move with the player (>0.5).');
		t.expect('ghost moves during resim', function () {
			if (gz0 == null) return { ok: false, detail: 'settling…' };
			return { ok: gzMoved > S.sc(0.5), detail: 'ghostMovedDuringResim=' + gzMoved.toFixed(3) };
		});
		t.simulate(w, 50);
		p.endResim();
	}, { page: 'fps/netcode', steps: 50, description: 'The ghost (the object-pushing body) is driven during rollback, so pushes reproduce on reconcile.' });

	PBF.scaleTest('fps/netcode', 'N5', 'pushed object reconciles (no rubber-band)', function (t, S) {
		var cmds = []; for (var i = 0; i < 30; i++) cmds.push({ forward: 1 });
		var z0 = S.sc(1.3);

		var liveRun = function () {
			var w = S.flat(); var pr = S.scaledObject(w, 1, 3, 1.3); var p = S.feetSpawn(w, 0, 0, {});
			for (var tk = 0; tk < 20; tk++) step(p, w, {});
			for (var c = 0; c < cmds.length; c++) step(p, w, cmds[c]);
			return pr.position.z;
		};
		var resimRun = function () {
			var w = S.flat(); var pr = S.scaledObject(w, 1, 3, 1.3); var p = S.feetSpawn(w, 0, 0, {});
			for (var tk = 0; tk < 20; tk++) step(p, w, {});
			p.beginResim();
			for (var c = 0; c < cmds.length; c++) step(p, w, cmds[c]);
			p.endResim();
			return pr.position.z;
		};

		var w = S.flat(); S.scaledObject(w, 1, 3, 1.3); var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) { return tick > 20 ? { forward: 1 } : {}; });

		var liveZ = liveRun(), resimZ = resimRun();
		t.log('Push an object two ways — live prediction and inside a resim — final object z must match (divergence < sc(0.01)).');
		t.expect('the live run actually pushed the object (> sc(1))', function () {
			return { ok: (liveZ - z0) > S.sc(1), detail: 'livePush=' + (liveZ - z0).toFixed(3) };
		});
		t.expect('live push == resim push (divergence < sc(0.01))', function () {
			var div = Math.abs(liveZ - resimZ);
			return { ok: div < S.sc(0.01), detail: 'livePush=' + (liveZ - z0).toFixed(3) + ' resimPush=' + (resimZ - z0).toFixed(3) + ' divergence=' + div.toFixed(4) };
		});
		t.simulate(w, 50);
	}, { page: 'fps/netcode', steps: 50, description: 'Object-prediction regression guard: a pushed object reconciles to the same place live and in resim.' });

	PBF.scaleTest('fps/netcode', 'N6', 'pushed object does not oscillate under live reconcile', function (t, S) {
		var RUNAHEAD = 5, SETTLE = 80, TICKS = 140, cmd = { forward: 1 };
		var z0 = S.sc(1.3);

		var objectState = function (b) {
			return { px: b.position.x, py: b.position.y, pz: b.position.z,
				vx: b.linear_velocity.x, vy: b.linear_velocity.y, vz: b.linear_velocity.z,
				ax: b.angular_velocity.x, ay: b.angular_velocity.y, az: b.angular_velocity.z,
				rx: b.rotation.x, ry: b.rotation.y, rz: b.rotation.z, rw: b.rotation.w };
		};
		var setObject = function (b, s) {
			b.position.set(s.px, s.py, s.pz);
			b.linear_velocity.set(s.vx, s.vy, s.vz);
			b.angular_velocity.set(s.ax, s.ay, s.az);
			b.rotation.x = s.rx; b.rotation.y = s.ry; b.rotation.z = s.rz; b.rotation.w = s.rw;
			if (b.updateDerived) b.updateDerived();
		};

		var experiment = function () {
			var wA = S.flat(), prA = S.scaledObject(wA, 1, 3, 1.3), pA = S.spawn(wA, { x: 0, y: 1, z: 0 }, {});
			var wcW = S.flat(), prc = S.scaledObject(wcW, 1, 3, 1.3), pc = S.spawn(wcW, { x: 0, y: 1, z: 0 }, {});
			for (var tk = 0; tk < 20; tk++) { step(pA, wA, {}); step(pc, wcW, {}); }
			var hist = [], cz = [];
			for (var tick = 0; tick < TICKS; tick++) {
				step(pA, wA, cmd); hist.push({ player: pA.getState(), object: objectState(prA) });
				step(pc, wcW, cmd);
				var ai = hist.length - 1 - RUNAHEAD;
				if (ai >= 0) {
					var a = hist[ai];
					pc.setState(a.player); setObject(prc, a.object);
					pc.beginResim();
					for (var k = 0; k < RUNAHEAD; k++) step(pc, wcW, cmd);
					pc.endResim();
				}
				cz.push(prc.position.z);
			}
			var rev = 0, maxJump = 0;
			for (var i = Math.max(2, SETTLE); i < cz.length; i++) {
				var d1 = cz[i] - cz[i - 1], d2 = cz[i - 1] - cz[i - 2];
				if (d1 * d2 < 0) rev++;
				if (Math.abs(d1) > maxJump) maxJump = Math.abs(d1);
			}
			var pushedFwd = prc.position.z - z0;
			return { rev: rev, maxJump: maxJump, pushed: pushedFwd > S.sc(3), pushedFwd: pushedFwd };
		};

		var w = S.flat(); S.scaledObject(w, 1, 3, 1.3); var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) { return tick > 20 ? cmd : {}; });
		var R = experiment();
		t.log('A client pushes an object while reconciling every tick against clean authority. The pushed object must not shudder (≤4 reversals) after the 80-tick settle.');
		t.expect('actually pushed the object', function () {
			return { ok: R.pushed, detail: 'pushedFwd=' + R.pushedFwd.toFixed(2) };
		});
		t.expect('no shudder (<=4 reversals)', function () {
			return { ok: R.rev <= 4, detail: 'reversals=' + R.rev + '/' + (TICKS - SETTLE) };
		});
		t.expect('no frame jump spikes (< sc(0.3))', function () {
			return { ok: R.maxJump < S.sc(0.3), detail: 'maxFrameJump=' + R.maxJump.toFixed(3) + ' limit=' + S.sc(0.3).toFixed(2) };
		});
		t.simulate(w, TICKS);
	}, { page: 'fps/netcode', steps: 140, description: 'Push-oscillation guard: a client reconciles every tick against clean authority while pushing an object; the object must not shudder.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
