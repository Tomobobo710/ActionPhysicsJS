(function (Runner, PBF, ActionPhysics) {

	var RigidBody = ActionPhysics.RigidBody, BoxShape = ActionPhysics.BoxShape, SphereShape = ActionPhysics.SphereShape;

	function finalGate(t, total) {
		var tick0 = 0;
		t.onTick(function (w, tick) { tick0 = tick; });
		return function () { return tick0 >= total; };
	}

	function box(w, side, mass, pos, color, mat) {
		mat = mat || { friction: 0.3, restitution: 0.2 };
		var b = new RigidBody(new BoxShape(side / 2, side / 2, side / 2), mass);
		b.position.set(pos.x, pos.y, pos.z);
		b.friction = mat.friction !== undefined ? mat.friction : 0.3;
		b.restitution = mat.restitution !== undefined ? mat.restitution : 0.2;
		b._color = color || '#0f0';
		w.addRigidBody(b);
		return b;
	}

	PBF.scaleTest('fps/knockback', 'K1', 'frontal knockback', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, peak = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 30) {
				var cs = S.sc(0.6);
				var headY = p.body.position.y + p.height / 2 - cs / 2;
				c = box(w, cs, 3 * S.SC * S.SC * S.SC, { x: 0, y: headY, z: S.sc(-1.5) }, '#0f0');
				c.linear_velocity.set(0, 0, S.sc(15));
			}
			if (tick > 30) peak = Math.max(peak, PBF.hsp(p));
			return {};
		});
		t.log('A scaled crate flies into the front of a standing player; the hit should knock them (peak hsp > ' + S.sc(0.5).toFixed(2) + ').');
		t.expect('peak knockback speed > sc(0.5)', function () {
			return { ok: c != null && peak > S.sc(0.5), detail: 'peak=' + peak.toFixed(2) + ' expect>' + S.sc(0.5).toFixed(2) };
		});
		t.simulate(w, 90);
	}, { page: 'fps/knockback', steps: 90, description: 'A scaled crate hits the front of a standing player; watch the knock.' });

	PBF.scaleTest('fps/knockback', 'K2', 'hit-from-behind boosts forward', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, maxFwd = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 30) {
				var z0 = p.body.position.z;
				var cs = S.sc(0.6);
				var headY = p.body.position.y + p.height / 2 - cs / 2;
				c = box(w, cs, 4 * S.SC * S.SC * S.SC, { x: 0, y: headY, z: z0 - S.sc(4) }, '#0f0');
				c.linear_velocity.set(0, 0, S.sc(20));
			}
			if (tick > 30) maxFwd = Math.max(maxFwd, p.body.linear_velocity.z);
			return {};
		});
		t.log('A crate from BEHIND (-z) should boost the stationary player forward (+z vz > ' + S.sc(0.5).toFixed(2) + ').');
		t.expect('forward velocity boost > sc(0.5)', function () {
			return { ok: c != null && maxFwd > S.sc(0.5), detail: 'maxForwardVz=' + maxFwd.toFixed(2) + ' expect>' + S.sc(0.5).toFixed(2) };
		});
		t.simulate(w, 90);
	}, { page: 'fps/knockback', steps: 90, description: 'Direction-independent knockback: a crate from behind boosts the player forward.' });

	PBF.scaleTest('fps/knockback', 'K3a', 'dropped object settles', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, maxBallSpeed = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 30) c = S.dropOnHead(w, p, S.sc(0.3), S.sc(0.6), 2 * S.SC * S.SC * S.SC, { friction: 0.4, restitution: 0.05 });
			if (c && tick > 90) maxBallSpeed = Math.max(maxBallSpeed,
				Math.hypot(c.linear_velocity.x, c.linear_velocity.y, c.linear_velocity.z));
			return {};
		});
		t.log('A box dropped on the head should land and come to REST (ball speed after settle < ' + S.sc(0.5).toFixed(2) + ').');
		var done = finalGate(t, 150);
		t.expect('ball settles (speed < sc(0.5))', function () {
			return { ok: done() && c != null && maxBallSpeed < S.sc(0.5), detail: 'ballSpeedAfterSettle=' + maxBallSpeed.toFixed(2) };
		});
		t.simulate(w, 150);
	}, { page: 'fps/knockback', steps: 150, description: 'A dropped box lands on the head and comes to rest.' });

	PBF.scaleTest('fps/knockback', 'K3b', 'straight drop does not fling player', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, maxPlayerH = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 30) {
				var cs = S.sc(0.6);
				var dropY = p.body.position.y + p.height / 2 + S.sc(3.4);
				c = box(w, cs, 2 * S.SC * S.SC * S.SC, { x: 0, y: dropY, z: 0 }, '#0f0', { friction: 0.4, restitution: 0.05 });
				c.linear_velocity.set(0, -S.sc(10), 0);
			}
			if (tick > 30) maxPlayerH = Math.max(maxPlayerH, PBF.hsp(p));
			return {};
		});
		t.log('A straight vertical drop has no closing horizontal speed; player horiz speed stays ~0 (< ' + S.sc(0.3).toFixed(2) + ').');
		var done = finalGate(t, 150);
		t.expect('player not flung (horiz < sc(0.3))', function () {
			return { ok: done() && c != null && maxPlayerH < S.sc(0.3), detail: 'maxPlayerHoriz=' + maxPlayerH.toFixed(2) + ' expect<' + S.sc(0.3).toFixed(2) };
		});
		t.simulate(w, 150);
	}, { page: 'fps/knockback', steps: 150, description: 'A box dropped straight down onto the head must not fling the player.' });

	PBF.scaleTest('fps/knockback', 'K4', 'no feedback runaway idle', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var mx = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function () { mx = Math.max(mx, PBF.hsp(p)); return {}; });
		t.log('Idle for 300 ticks; the player must never spontaneously accelerate (max hsp < 0.01).');
		var done = finalGate(t, 300);
		t.expect('no idle runaway (max < 0.01)', function () {
			return { ok: done() && mx < 0.01, detail: 'maxIdleSpeed=' + mx.toFixed(4) };
		});
		t.simulate(w, 300);
	}, { page: 'fps/knockback', steps: 300, description: 'A stationary player must not drift from feedback runaway.' });

	PBF.scaleTest('fps/knockback', 'K5', 'knockback cap holds', function (t, S) {
		var CAP = 16;
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, { receiveMaxSpeed: CAP });
		var c = null, peak = 0;
		var cs = S.sc(1), vel = S.sc(14);
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 30) {
				var headY = p.body.position.y + p.height / 2 - cs / 2;
				c = box(w, cs, 50 * S.SC * S.SC * S.SC, { x: 0, y: headY, z: p.body.position.z - S.sc(3) }, '#0f0');
				c.linear_velocity.set(0, 0, vel);
			}
			if (c && tick > 30) peak = Math.max(peak, PBF.hsp(p));
			return {};
		});
		var cap = CAP * S.SC;
		t.log('A heavy crate hits hard at head level; peak knockback must REACH the cap (' + cap.toFixed(1) + ') and NOT EXCEED it.');
		var done = finalGate(t, 80);
		t.expect('peak knockback reaches the cap (hit is genuinely strong enough)', function () {
			if (!done() || c == null) return { ok: false, detail: 'settling…' };
			return { ok: peak >= 0.8 * cap, detail: 'peak=' + peak.toFixed(2) + ' cap=' + cap.toFixed(1) };
		});
		t.expect('peak knockback does not exceed the cap', function () {
			if (!done() || c == null) return { ok: false, detail: 'settling…' };
			return { ok: peak <= cap * 1.05, detail: 'peak=' + peak.toFixed(2) + ' cap=' + cap.toFixed(1) };
		});
		t.simulate(w, 80);
	}, { page: 'fps/knockback', steps: 80, description: 'A heavy crate hits hard at head level; peak knockback must reach the receiveMaxSpeed cap and not exceed it.' });

	PBF.scaleTest('fps/knockback', 'K6', 'object rests on head (no sink)', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, penetration = 99;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 20) c = S.dropOnHead(w, p, S.sc(0.3), S.sc(0.8), 2 * S.SC * S.SC * S.SC);
			if (c) {
				var headTop = p.body.position.y + p.height / 2;
				var boxBottom = c.position.y - S.sc(0.4);
				penetration = headTop - boxBottom;
			}
			return {};
		});
		t.log('A box dropped on the head must sit ON TOP, not sink in (penetration < ' + S.sc(0.2).toFixed(2) + ').');
		var done = finalGate(t, 140);
		t.expect('no sink (penetration < sc(0.2))', function () {
			return { ok: done() && c != null && penetration < S.sc(0.2), detail: 'penetration=' + penetration.toFixed(3) };
		});
		t.simulate(w, 140);
	}, { page: 'fps/knockback', steps: 140, description: 'A box resting on the head stays on top rather than clipping into it.' });

	PBF.scaleTest('fps/knockback', 'K7', 'light ball does not knock', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, peak = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 30) {
				var r = S.sc(0.15);
				var headY = p.body.position.y + p.height / 2 - r;
				c = new RigidBody(new SphereShape(r), 0.2 * S.SC * S.SC * S.SC);
				c.position.set(0, headY, S.sc(-1.2));
				c.friction = 0.4; c.restitution = 0.1;
				c._color = '#0f0';
				w.addRigidBody(c);
				c.linear_velocity.set(0, 0, S.sc(10));
			}
			if (tick > 30) peak = Math.max(peak, PBF.hsp(p));
			return {};
		});
		t.log('A tiny light ball rolling gently in has negligible momentum; player must not be knocked (peak < 0.5).');
		var done = finalGate(t, 90);
		t.expect('no knock (peak < 0.5)', function () {
			return { ok: done() && c != null && peak < 0.5, detail: 'playerPeak=' + peak.toFixed(3) };
		});
		t.simulate(w, 90);
	}, { page: 'fps/knockback', steps: 90, description: 'A small light ball must not knock a stationary player.' });

	PBF.scaleTest('fps/knockback', 'K8', 'pushing stationary object no self-knock', function (t, S) {
		var w = S.flat();
		S.scaledObject(w, 1, 5, 3, '#0f0');
		var p = S.feetSpawn(w, 0, 0, {});
		var minVz = 99, tick0 = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			tick0 = tick;
			if (tick > 5) minVz = Math.min(minVz, p.body.linear_velocity.z);
			return { forward: 1 };
		});
		t.log('WALKING into a stationary crate should push it — the player must not get a backward knock (min vz > -0.5).');
		var done = finalGate(t, 60);
		t.expect('no self-knock (min forward vz > -0.5)', function () {
			return { ok: done() && minVz > -S.sc(0.5), detail: 'minForwardVz=' + (minVz === 99 ? '-' : minVz.toFixed(2)) };
		});
		t.simulate(w, 60);
	}, { page: 'fps/knockback', steps: 60, description: 'Pushing a stationary object must not knock the player backward.' });

	PBF.scaleTest('fps/knockback', 'K9', 'dropped box settles (no spin/slide)', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, maxAng = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 20) c = S.dropOnHead(w, p, S.sc(0.3), S.sc(0.6), 1 * S.SC * S.SC * S.SC, { friction: 0.5, restitution: 0 });
			if (c && tick > 170) {
				var av = c.angular_velocity;
				maxAng = Math.max(maxAng, Math.hypot(av.x, av.y, av.z));
			}
			return {};
		});
		t.log('A box dropped flat on the head must settle — late angular velocity near zero (< 0.3).');
		var done = finalGate(t, 220);
		t.expect('settles (late angVel < 0.3)', function () {
			return { ok: done() && c != null && maxAng < 0.3, detail: 'lateAngVel=' + maxAng.toFixed(3) };
		});
		t.simulate(w, 220);
	}, { page: 'fps/knockback', steps: 220, description: 'A dropped box settles on the head without spinning or sliding.' });

	PBF.scaleTest('fps/knockback', 'K10', 'resting object settles (no oscillation)', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var boxSide = S.sc(0.5);
		var boxZ = p.depth / 2 + boxSide / 2 + S.sc(0.05);
		var c = box(w, boxSide, 1 * S.SC * S.SC * S.SC, { x: 0, y: boxSide / 2, z: boxZ }, '#0f0', { friction: 0.4, restitution: 0 });
		var maxLate = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick > 150) maxLate = Math.max(maxLate,
				Math.hypot(c.linear_velocity.x, c.linear_velocity.y, c.linear_velocity.z));
			return {};
		});
		t.log('A light object resting against the front of a stationary player should settle (late speed < 0.2).');
		var done = finalGate(t, 200);
		t.expect('settles (late speed < 0.2)', function () {
			return { ok: done() && maxLate < 0.2, detail: 'lateSpeed=' + maxLate.toFixed(3) };
		});
		t.simulate(w, 200);
	}, { page: 'fps/knockback', steps: 200, description: 'A light object resting against the player settles without oscillating.' });

	PBF.scaleTest('fps/knockback', 'K10', 'sustained push no oscillation', function (t, S) {
		var w = S.flat();
		S.scaledObject(w, 1, 3, 2);
		var p = S.feetSpawn(w, 0, 0, {});
		var vz = [];
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick > 80) vz.push(p.body.linear_velocity.z);
			return { forward: 1 };
		});
		var limit = 0.25 * S.SC * S.SC;
		t.log('A steady push must not oscillate; forward-velocity variance during the sample window must stay small (< ' + limit.toFixed(3) + ').');
		var done = finalGate(t, 140);
		t.expect('steady push (variance < 0.25*SC^2)', function () {
			if (!done() || vz.length < 2) return { ok: false, detail: 'settling…' };
			var m = 0; for (var i = 0; i < vz.length; i++) m += vz[i]; m /= vz.length;
			var varr = 0; for (var j = 0; j < vz.length; j++) varr += (vz[j] - m) * (vz[j] - m); varr /= vz.length;
			return { ok: varr < limit, detail: 'forwardVelVariance=' + varr.toFixed(4) + ' limit=' + limit.toFixed(3) };
		});
		t.simulate(w, 140);
	}, { page: 'fps/knockback', steps: 140, description: 'A sustained push of a moving box stays flat (no push/knockback limit cycle).' });

	PBF.scaleTest('fps/knockback', 'K11', 'head object not launched by moving', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, restY = null, maxUpVy = 0, maxRise = 0;
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 20) c = S.dropOnHead(w, p, S.sc(0.3), S.sc(0.8), 2 * S.SC * S.SC * S.SC);
			if (tick === 140) restY = c.position.y;
			if (tick > 140 && tick <= 152) {
				maxUpVy = Math.max(maxUpVy, c.linear_velocity.y);
				maxRise = Math.max(maxRise, c.position.y - restY);
			}
			return tick > 140 ? { forward: 1, sprint: true } : {};
		});
		t.log('A object resting on the head must not be launched upward when the player sprints off (kickVy < ' + S.sc(0.5).toFixed(2) + ', rise < ' + S.sc(0.15).toFixed(2) + ').');
		var done = finalGate(t, 180);
		t.expect('no upward kick velocity from the sprint-start snap (< sc(0.5))', function () {
			if (!done() || restY == null) return { ok: false, detail: 'settling…' };
			return { ok: maxUpVy < S.sc(0.5), detail: 'kickVy=' + maxUpVy.toFixed(2) + ' limit=' + S.sc(0.5).toFixed(2) };
		});
		t.expect('no meaningful rise from the sprint-start snap (< sc(0.15))', function () {
			if (!done() || restY == null) return { ok: false, detail: 'settling…' };
			return { ok: maxRise < S.sc(0.15), detail: 'rise=' + maxRise.toFixed(2) + ' limit=' + S.sc(0.15).toFixed(2) };
		});
		t.simulate(w, 180);
	}, { page: 'fps/knockback', steps: 180, description: 'A object resting on the head must not be flung up when the player starts sprinting.' });

	PBF.scaleTest('fps/knockback', 'K13', 'walking out drops head object to floor', function (t, S) {
		var w = S.flat();
		var p = S.feetSpawn(w, 0, 0, {});
		var c = null, onFloor = false, farFromPlayer = false, dist = 0, boxY = 0;
		var floorY = S.sc(0.4);
		PBF.renderables(t, p);
		PBF.drive(t, p, function (tick) {
			if (tick === 20) c = S.dropOnHead(w, p, S.sc(0.3), S.sc(0.8), 2 * S.SC * S.SC * S.SC);
			if (c) {
				boxY = c.position.y;
				dist = Math.hypot(c.position.x - p.body.position.x, c.position.z - p.body.position.z);
				onFloor = Math.abs(boxY - floorY) < S.sc(0.15);
				farFromPlayer = dist > S.sc(1.0);
			}
			return tick > 70 ? { forward: 1 } : {};
		});
		t.log('After walking out from under it, the object should fall to the FLOOR and be left behind (onFloor & dist > ' + S.sc(1.0).toFixed(2) + ').');
		var done = finalGate(t, 250);
		t.expect('object drops to the floor', function () {
			if (!done() || c == null) return { ok: false, detail: 'settling…' };
			return { ok: onFloor, detail: 'boxY=' + boxY.toFixed(2) + ' floorY=' + floorY.toFixed(2) };
		});
		t.expect('object is left behind (far from player)', function () {
			if (!done() || c == null) return { ok: false, detail: 'settling…' };
			return { ok: farFromPlayer, detail: 'distFromPlayer=' + dist.toFixed(2) };
		});
		t.simulate(w, 250);
	}, { page: 'fps/knockback', steps: 250, description: 'Walking out from under a head object drops it straight to the floor, not hovering along.' });
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
