(function (Runner, PBF, ActionPhysics) {
	Runner.suite('tom');

	function box(w, side, mass, pos, color, mat) {
		return PBF.object(w, side, mass, pos, color || '#f00', mat);
	}

	PBF.scaleTest('fps/push', 'P1', 'push moves an object at all', function (t, S) {
		var w = S.flat();
		var pr = S.scaledObject(w, 1, 0.5, 0.3 + 0.5 + 0.02);
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);

		var z0 = null, moved = 0;
		PBF.drive(t, p, function (tick) {
			if (tick === 30) z0 = pr.position.z;
			if (z0 != null) moved = pr.position.z - z0;
			return (tick === 31 || tick === 32) ? { forward: 1 } : {};
		});

		t.log('Settle, then one 2-frame forward tap into a light box; it should roll forward (+z) at all.');
		t.expect('object moved forward (>0)', function () {
			if (z0 == null) return { ok: false, detail: 'settling…' };
			return { ok: moved > 0, detail: 'moved=' + moved.toFixed(4) };
		});
		t.simulate(w, 92);
	}, { page: 'fps/push', steps: 92, description: 'A single short forward tap into a light box; the box should visibly roll forward.' });

	PBF.scaleTest('fps/push', 'P2', 'push mass-monotonic (heavy never easier)', function (t, S) {
		var side = S.sc(1), boxZ = S.sc(0.3 + 0.5 + 0.02);
		var masses = [0.5, 1, 2, 5, 10, 20, 50];
		var BLOCK = 280;
		var w = S.flat();
		var peaks = {}, curPr = null;

		function buildBlock(world, idx, m) {
			world.addRigidBody(PBF.staticBox(60, 0.5, 60, { x: 0, y: -0.5, z: 0 }));
			curPr = box(world, side, m * S.SC * S.SC * S.SC, { x: 0, y: side / 2, z: boxZ });
			var ctrl = S.feetSpawn(world, 0, 0, {});
			return ctrl;
		}

		var seq = PBF.sequentialBlocks(t, w, masses, BLOCK, buildBlock,
			function (local, idx, m) {
				if (curPr) {
					var vz = curPr.linear_velocity.z;
					if (!(m in peaks) || vz > peaks[m]) peaks[m] = vz;
				}
			},
			function (local) { return (local === 31) ? { forward: 1 } : {}; });

		t.log('Each mass (0.5→50) gets a single-frame tap, IN TURN — watch the box launch, then the next, heavier box. Heavier must launch NO faster than lighter, full stop — no light-mass exemption.');
		t.expect('heavier never launches faster (strictly monotonic, ALL masses)', function () {
			if (seq.tickOf() < masses.length * BLOCK) return { ok: false, detail: 'pushing mass ' + (masses[Math.min(masses.length - 1, Math.floor(seq.tickOf() / BLOCK))]) + '…' };
			var res = {}; masses.forEach(function (m) { res[m] = +(peaks[m] || 0).toFixed(3); });
			var mono = true, viol = '', prev = Infinity;
			masses.forEach(function (m) {
				if (res[m] > prev + S.sc(0.01)) { mono = false; if (!viol) viol = ' (mass ' + m + ' peak ' + res[m] + ' > lighter ' + prev + ')'; }
				prev = res[m];
			});
			return { ok: mono, detail: JSON.stringify(res) + viol };
		});
		t.simulate(w, masses.length * BLOCK);
	}, { page: 'fps/push', steps: 1960, description: 'Seven objects of rising mass, tapped ONE AT A TIME in sequence (lightest first). Watch each launch, then the next heavier one launch NO faster — strictly monotonic across every mass, no exemption for the light end.' });

	PBF.scaleTest('fps/push', 'P3', 'very heavy object resists', function (t, S) {
		var w = S.flat();
		var pr = S.scaledObject(w, 1, 500, 3);
		var side = S.sc(1), boxTop = pr.position.y + side / 2;
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var z0 = pr.position.z, moved = 0, heldTop = 0, tick0 = 0;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			moved = pr.position.z - z0;
			var feet = p.body.position.y - p.height / 2;
			if (tick > 30 && p.grounded && feet > boxTop - S.sc(0.15) && p.body.position.z > pr.position.z - S.sc(0.5)) heldTop++;
			return { forward: 1, sprint: true };
		});

		t.log('Sprint into a very heavy (mass 500·SC³) box for 120 ticks; it should barely move (<sc(0.5)) AND stop the char — not be climbed onto.');
		t.expect('heavy object barely moves (<sc(0.5))', function () {
			if (tick0 < 120) return { ok: false, detail: 'pushing…' };
			return { ok: Math.abs(moved) < S.sc(0.5), detail: 'moved=' + moved.toFixed(3) };
		});
		t.expect('char is not climbed onto the box (no sustained mount)', function () {
			if (tick0 < 120) return { ok: false, detail: 'pushing… heldTop=' + heldTop };
			var mounted = heldTop >= 10;
			return { ok: !mounted,
				detail: 'feetEnd=' + (p.body.position.y - p.height / 2).toFixed(2) + ' boxTop=' + boxTop.toFixed(2) + ' heldTopTicks=' + heldTop };
		});
		t.simulate(w, 120);
	}, { page: 'fps/push', steps: 120, description: 'Sprint into a mass-500 box; it should hardly budge AND stop the character. Fails if the char climbs/steps over the box.' });

	PBF.scaleTest('fps/push', 'P5', 'lean-to-shove sustains', function (t, S) {
		var w = S.flat();
		var pr = S.scaledObject(w, 1, 3, 3);
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var zMid = null, moved2 = 0;
		PBF.drive(t, p, function (tick) {
			if (tick === 40) zMid = pr.position.z;
			if (zMid != null) moved2 = pr.position.z - zMid;
			return { forward: 1 };
		});

		t.log('Lean-shove a box (walk into it); the push must SUSTAIN — in the second 40 ticks the box keeps advancing (>sc(0.5)), it doesn\'t stall out.');
		t.expect('lean-shove sustains — box keeps advancing (>sc(0.5))', function () {
			if (zMid == null) return { ok: false, detail: 'first half…' };
			return { ok: moved2 > S.sc(0.5), detail: 'movedSecondHalf=' + moved2.toFixed(2) + ' expect>' + S.sc(0.5).toFixed(2) };
		});
		t.simulate(w, 80);
	}, { page: 'fps/push', steps: 80, description: 'Lean-shove a box by walking into it; the push sustains — the box keeps moving through the second half.' });

	PBF.scaleTest('fps/push', 'P6', 'back off, ghost heels no runaway', function (t, S) {
		var w = S.flat();
		var pr = S.scaledObject(w, 1, 3, 3);
		var p = S.feetSpawn(w, 0, 0, {});
		PBF.renderables(t, p);
		var gap = 99, spd = 99, y0 = p.body.position.y, peakRise = 0, steppedOver = false, tick0 = 0;
		var half = pr.position.y;
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			gap = Math.hypot(p.body.position.x - p._ghost.position.x, p.body.position.z - p._ghost.position.z);
			spd = PBF.hsp(p);
			peakRise = Math.max(peakRise, p.body.position.y - y0);
			if (p.body.position.z > pr.position.z + half) steppedOver = true;
			return tick <= 40 ? { forward: 1, sprint: true } : {};
		});

		t.log('Sprint-shove 40 ticks then release; the ghost heels to the player (gap<0.5) with no runaway — and the char must have SHOVED the box, not climbed over it.');
		t.expect('ghost heels back to the player (gap < 0.5)', function () {
			if (tick0 < 100) return { ok: false, detail: 'shoving…' };
			return { ok: gap < 0.5, detail: 'gap=' + gap.toFixed(3) };
		});
		t.expect('no runaway (speed settles < 0.5)', function () {
			if (tick0 < 100) return { ok: false, detail: 'shoving…' };
			return { ok: spd < 0.5, detail: 'playerSpeed=' + spd.toFixed(3) };
		});
		t.expect('did not step over the box (no climb)', function () {
			if (tick0 < 100) return { ok: false, detail: 'shoving… steppedOver=' + steppedOver };
			return { ok: !steppedOver, detail: 'steppedOver=' + steppedOver };
		});
		t.expect('no spurious rise from the box interaction (< sc(0.3))', function () {
			if (tick0 < 100) return { ok: false, detail: 'shoving… peakRise=' + peakRise.toFixed(2) };
			return { ok: peakRise < S.sc(0.3), detail: 'peakRise=' + peakRise.toFixed(2) };
		});
		t.simulate(w, 100);
	}, { page: 'fps/push', steps: 100, description: 'After a shove, releasing input: the ghost snaps back to the player and speed settles to zero — and the char must have shoved the box, not climbed over it.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
