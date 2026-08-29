/**
 * ActionPhysics browser visualization + live console. Renderer-AGNOSTIC: all drawing goes through an adapter
 * (window.APBench.adapter — the Three adapter by default; a host may supply its own). This file owns
 * the runner glue, the honesty contract, the timeline console, the HUD, and the fixed-timestep stepping.
 *
 * Exposes window.APRender:
 *   run(test, opts)  — run one test live: stream its console, draw its bodies via the adapter, resolve verdict.
 *   clear()          — clear the viewport + console.
 *   setAdapter(a)    — choose the rendering backend (defaults to the Three adapter).
 *
 * HONESTY CONTRACT: the picture, the console, and the verdict all come from the SAME run of the SAME test.
 *
 * STEP-DRIVER SEAM: a test's world is advanced one tick by ctx.stepWorld(world) if the ctx provides it,
 * else the default world.step(1/60). A host's controller tests set ctx.stepWorld to run their
 * beginStep/fixed_update/endStep lifecycle — so the same live loop drives either engine.
 */
(function () {
	var Runner = window.APRunner;

	// The active rendering backend. Defaults to Three; setAdapter() swaps it (per suite / mode toggle).
	var adapter = window.APThreeAdapter || null;
	function setAdapter(a) { adapter = a; }

	var anim = { world: null, meshes: [], steps: 0, tick: 0, stepping: false };
	var consoleEl = function () { return document.getElementById('console'); };
	var hudEl = function () { return document.getElementById('hud'); };
	var viewport3d = function () { return document.getElementById('viewport3d'); };

	var _inited = false;
	function ensureInit() {
		if (_inited) return;
		adapter.init(viewport3d());
		window.addEventListener('resize', function () { if (adapter.resize) adapter.resize(); });
		_inited = true;
		loop();
	}

	// One tick of the sim. Default is world.step(1/60); a ctx may override via ctx.stepWorld to
	// drive a host's own lifecycle (e.g. beginStep/fixed_update/endStep).
	function stepWorldOnce(ctx, world) {
		if (ctx && typeof ctx.stepWorld === 'function') ctx.stepWorld(world);
		else world.step(1 / 60);
	}

	function midpoint(a, b) { return { x: (a[0] + b[0]) / 2, y: (a[1] + b[1]) / 2, z: (a[2] + b[2]) / 2 }; }

	// ---- console (criteria-first checklist) ----
	function clearConsole() { var c = consoleEl(); if (c) c.innerHTML = ''; }

	function paintTimeline(timeline) {
		var c = consoleEl(); if (!c) return [];
		c.innerHTML = '';
		var nodes = [];
		timeline.forEach(function (e) {
			var d = document.createElement('div');
			if (e.type === 'criterion') {
				d.className = 'cl crit pending';
				d.textContent = '○ ' + e.label;
			} else {
				d.className = 'cl info hidden';
				d.textContent = '  ' + e.msg;
			}
			c.appendChild(d);
			nodes.push({ el: d, entry: e });
		});
		return nodes;
	}
	function revealTimeline(nodes, fast, onDone) {
		var c = consoleEl();
		var i = 0;
		(function step() {
			if (i >= nodes.length) { if (onDone) onDone(); return; }
			var n = nodes[i++];
			if (n.entry.type === 'criterion') {
				var ok = n.entry.status === 'pass';
				n.el.className = 'cl crit ' + (ok ? 'pass' : 'fail');
				n.el.textContent = (ok ? '✓ ' : '✗ ') + n.entry.label + (n.entry.detail ? '   — ' + n.entry.detail : '');
			} else {
				n.el.className = 'cl info';
			}
			if (c) c.scrollTop = c.scrollHeight;
			setTimeout(step, fast ? 0 : (n.entry.type === 'criterion' ? 120 : 45));
		})();
	}

	// ---- the public run ----
	function run(test, opts) {
		opts = opts || {};
		ensureInit();
		document.getElementById('empty').style.display = 'none';
		anim.live = false; anim.world = null; anim.meshes = [];
		adapter.clear(); clearConsole();

		var hud = hudEl();
		hud.style.display = 'block';
		function setHud(state) {
			hud.innerHTML = '<div class="vt">' + test.page + ' — ' + test.name + '</div>'
				+ (state === 'run' ? '<div class="vv" style="color:#8b949e">running…</div>'
					: '<div class="vv ' + (state ? 'pass' : 'fail') + '">' + (state ? 'PASS' : 'FAIL: ' + lastResult.error) + '</div>')
				+ (test.steps > 0 ? '<div class="vv" id="tickline"></div>' : '');
		}
		var lastResult;
		setHud('run');

		var cap = captureSetup(test);
		lastResult = cap.result;
		var nodes = paintTimeline(cap.result.timeline);
		var entryToNode = {}; nodes.forEach(function (n) { entryToNode[nodeKey(n.entry)] = n; });
		nodes.forEach(function (n) { if (n.entry.type === 'log') n.el.className = 'cl info'; });

		if (cap.scheduled && cap.world) {
			// The per-frame reconcile (in loop()) owns adding/removing/syncing drawables — don't pre-add here,
			// just clear stale drawable back-pointers so a body reused across runs re-registers cleanly.
			(cap.bodies || []).forEach(function (b) { if (b) b._drawable = null; });
			adapter.frameCamera(cap.bodies);
			anim.ctx = cap.ctx; anim.world = cap.world;
			anim.bodies = cap.bodies;
			anim._drawn = [];               // fresh drawable-tracking for this run (adapter.clear() already wiped the view)
			anim.totalTicks = cap.simTicks;
			anim.tick = 0; anim.live = true; anim.fast = !!opts.fast; anim._finished = false;
			anim._accum = 0; anim._last = null;
			anim.reflect = function () {
				cap.result.timeline.forEach(function (e) { if (e.type === 'criterion') updateNode(entryToNode[nodeKey(e)], e); });
			};
			anim.onFinish = function () {
				anim.ctx.failUnmet();
				anim.reflect();
				var ok = true, err = '';
				cap.result.timeline.forEach(function (e) { if (e.type === 'criterion' && e.status === 'fail') { ok = false; err = (err ? err + '; ' : '') + e.label; } });
				lastResult = { suite: test.suite, group: test.group, name: test.name, ok: ok, error: err, ctx: cap.ctx };
				setHud(ok);
				if (opts.onComplete) opts.onComplete(lastResult);
			};
			return lastResult;
		}

		// NON-LIVE (geometry / math): result already computed; replay checklist, draw static state.
		if (cap.ctx.ray && adapter.drawRay) adapter.drawRay(cap.ctx.ray);
		if (cap.ctx.support && adapter.drawSupport) adapter.drawSupport(cap.ctx.support);
		if (cap.ctx.swept && adapter.drawSwept) { adapter.drawSwept(cap.ctx.swept); adapter.frameCamera([{ position: midpoint(cap.ctx.swept.start, cap.ctx.swept.end) }]); }
		if (cap.bodies && cap.bodies.length) { cap.bodies.forEach(function (b) { adapter.addBody(b); }); adapter.frameCamera(cap.bodies); }
		revealTimeline(nodes, opts.fast, function () {
			setHud(cap.result.ok);
			if (opts.onComplete) opts.onComplete(cap.result);
		});
		return cap.result;
	}

	// Run the test's fn but intercept simulate() so the world is left at t=0 with expectations declared
	// (all pending) and nothing evaluated yet — the renderer will step the world and evaluate live.
	function captureSetup(test) {
		var captured = { world: null, simTicks: 0, isLive: false };
		var clone = {
			suite: test.suite, group: test.group, name: test.name, page: test.page, visual: test.visual, steps: test.steps, description: test.description,
			fn: function (ctx) {
				var realSim = ctx.simulate;
				ctx.simulate = function (world, totalTicks) { captured.world = world; captured.simTicks = totalTicks; captured.isLive = true; };
				try { test.fn(ctx); } finally { ctx.simulate = realSim; }
			}
		};
		var result = Runner.runOne(clone);
		return {
			result: result, ctx: result.ctx, world: captured.world, bodies: result.ctx.bodies,
			scheduled: captured.isLive && result.ctx.expectations && result.ctx.expectations.length > 0,
			simTicks: captured.simTicks
		};
	}
	function nodeKey(entry) { return entry.type + ':' + (entry.label || entry.msg); }

	function updateNode(n, entry) {
		if (!n) return;
		if (entry.status === 'pending') {
			n.el.className = 'cl crit pending';
			n.el.textContent = '○ ' + entry.label + (entry.detail ? '   — ' + entry.detail : '');
		} else {
			var ok = entry.status === 'pass';
			n.el.className = 'cl crit ' + (ok ? 'pass' : 'fail');
			n.el.textContent = (ok ? '✓ ' : '✗ ') + entry.label + (entry.detail ? '   — ' + entry.detail : '');
		}
	}

	var DT_MS = 1000 / 60;

	function loop(now) {
		requestAnimationFrame(loop);
		if (anim.live && anim.world && anim.ctx) {
			if (anim._last == null) anim._last = now;
			var speed = (anim.fast ? 6 : 1) * (window.PB_SLOWMO || 1);
			anim._accum += Math.min(250, (now - anim._last)) * speed;
			anim._last = now;
			var canEarlyOut = !(anim.ctx.tickHooks && anim.ctx.tickHooks.length);
			var done = false, guard = 0;
			while (anim._accum >= DT_MS && anim.tick < anim.totalTicks && !(done && canEarlyOut) && guard++ < 600) {
				anim._accum -= DT_MS;
				anim.ctx.runTickHooks(anim.world, anim.tick + 1);
				stepWorldOnce(anim.ctx, anim.world); anim.tick++;
				done = anim.ctx.evalTick(anim.world, anim.tick);
			}
			anim.reflect();
			// Reconcile the DRAWN set with what's actually in play THIS frame. Bodies are created AND destroyed
			// mid-sim: the controller REBUILDS its collider on crouch/scale (swaps controller.object, removing
			// the old one from the world), and tests drop crates in. So each frame we compute the live set —
			// everything in the world + the controller's current collider + explicit extras — then ADD drawables
			// for newcomers, SYNC all, and REMOVE drawables for anything that left (or the stale tall collider
			// lingers on screen after a crouch). Diffing against world membership is what keeps the picture true.
			var live = [];
			var w = anim.world;
			if (w && w.objects && typeof w.objects.forEach === 'function') w.objects.forEach(function (o) { if (live.indexOf(o) === -1) live.push(o); });
			(anim.ctx.bodies || []).forEach(function (o) { if (o && live.indexOf(o) === -1) live.push(o); });
			// The controller REBUILDS its collider (controller.object) on crouch/scale as a fresh body that
			// defaults to isVisible:false — so force the CURRENT collider visible every frame, or it vanishes
			// (or a stale one lingers) after a crouch. This is the thing that keeps the capsule short when crouched.
			var ctrl = anim.ctx._pbController;
			if (ctrl && ctrl.object) {
				var co = ctrl.object;
				if (typeof co.setVisibility === 'function') co.setVisibility(true); else co.isVisible = true;
				if (live.indexOf(co) === -1) live.push(co);
			}
			// remove drawables whose body is no longer live
			anim._drawn = anim._drawn || [];
			for (var di = anim._drawn.length - 1; di >= 0; di--) {
				var old = anim._drawn[di];
				if (live.indexOf(old) === -1) {
					if (old._drawable != null && adapter.removeBody) adapter.removeBody(old._drawable);
					old._drawable = null;
					anim._drawn.splice(di, 1);
				}
			}
			// add newcomers + sync everything
			live.forEach(function (b) {
				if (!b) return;
				if (b._drawable == null) { b._drawable = adapter.addBody(b); anim._drawn.push(b); }
				if (b._drawable != null) adapter.syncBody(b._drawable, b);
			});
			var tl = document.getElementById('tickline'); if (tl) tl.textContent = 'tick ' + anim.tick + ' / ' + anim.totalTicks;
			if ((anim.tick >= anim.totalTicks || (done && canEarlyOut)) && !anim._finished) { anim._finished = true; anim.onFinish(); }
		}
		if (adapter) adapter.render();
	}

	function clear() {
		if (adapter) adapter.clear();
		clearConsole();
		var hud = hudEl(); if (hud) hud.style.display = 'none';
		document.getElementById('empty').style.display = '';
		anim.live = false;
	}

	window.APRender = { run: run, clear: clear, resize: function () { if (adapter && adapter.resize) adapter.resize(); }, setAdapter: setAdapter };
})();
