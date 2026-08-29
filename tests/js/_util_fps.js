// Tom's Suite — FPSCharacterController helpers. Build a World + FPSCharacterController directly;
// the visual bench draws every world body by shape/_color, exactly like everything else in this suite.
//
// A controller test does:
//   1. build a World + FPSCharacterController + any object bodies,
//   2. register those + the controller for the viewer via PBF.renderables(t, controller, extras),
//   3. drive the controller each tick via PBF.drive(t, controller, cmdFor) (installs a tick hook),
//   4. call t.simulate(world, ticks) and assert with t.expect(...) live — same as every other test here.
(function (root, factory) {
	var AP = (typeof module !== 'undefined' && module.exports)
		? require('../../build/actionphysics.js')   // js/_util_fps.js -> ../../build/actionphysics.js
		: root.ActionPhysics;
	var Runner = (typeof module !== 'undefined' && module.exports)
		? require('./runner.js')
		: root.APRunner;
	var mod = factory(AP, Runner);
	if (typeof module !== 'undefined' && module.exports) module.exports = mod;
	else root.PBF = mod;
})(typeof window !== 'undefined' ? window : this, function (AP, Runner) {

	var DT = 1 / 60;

	// Default controller options: bare physics core, no cosmetic identity needed for a headless/visual
	// physics test. FPSCharacterController has no weapon/combat/grab/model/view/input components at
	// all — there's nothing to disable here.
	var CONTROLLER_COLOR = '#cc4444';

	// Shared material defaults applied to every body built by this file unless the caller passes its
	// own — friction/restitution/damping meaningfully change contact behavior (a bare-default floor
	// vs. this one changes how a pushed box actually settles), so every helper here goes through one
	// place rather than letting RigidBody's own bare defaults (friction:0.5, restitution:0, zero
	// damping) apply inconsistently per test.
	var MATERIAL_DEFAULTS = { friction: 3.0, restitution: 0.33, linearDamping: 0.1, angularDamping: 0.9 };
	function applyMat(b, mat) {
		var m = mat || {};
		b.friction = m.friction !== undefined ? m.friction : MATERIAL_DEFAULTS.friction;
		b.restitution = m.restitution !== undefined ? m.restitution : MATERIAL_DEFAULTS.restitution;
		b.linear_damping = m.linearDamping !== undefined ? m.linearDamping : MATERIAL_DEFAULTS.linearDamping;
		b.angular_damping = m.angularDamping !== undefined ? m.angularDamping : MATERIAL_DEFAULTS.angularDamping;
	}

	function makeWorld() {
		var w = new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), new AP.Solver());
		w.gravity = new AP.Vector3(0, -9.81, 0);
		return w;
	}

	// A static floor slab, top face at y=0 — the standard scene every controller test shares. Mass 0 =
	// static (this engine's convention, not Infinity).
	function flatWorld() {
		var w = makeWorld();
		var floor = new AP.RigidBody(new AP.BoxShape(60, 0.5, 60), 0);
		floor.position.set(0, -0.5, 0);
		applyMat(floor, null);
		floor._color = '#333333';
		w.addRigidBody(floor);
		w._floor = floor;
		return w;
	}

	// A visible dynamic object (box). side/mass in world units; pos is a Vector3-shaped {x,y,z}.
	// mat overrides friction/restitution; objects default to {friction:0.4, restitution:0.1} —
	// distinct from the engine MATERIAL_DEFAULTS above, which is the fallback for static geometry.
	function object(w, side, mass, pos, color, mat) {
		var b = new AP.RigidBody(new AP.BoxShape(side / 2, side / 2, side / 2), mass);
		b.position.set(pos.x, pos.y, pos.z);
		applyMat(b, Object.assign({ friction: 0.4, restitution: 0.1 }, mat || {}));
		b._color = color || '#cc4444';
		w.addRigidBody(b);
		return b;
	}

	// Static (mass=0) box geometry — walls, platforms, ledges, ceilings, ramps. half-extents
	// hx/hy/hz; pos {x,y,z}; rot an optional Quaternion (see axisAngleQuat below).
	function staticBox(hx, hy, hz, pos, color, mat, rot) {
		var b = new AP.RigidBody(new AP.BoxShape(hx, hy, hz), 0);
		b.position.set(pos.x, pos.y, pos.z);
		if (rot) b.rotation.set(rot.x, rot.y, rot.z, rot.w);
		b.updateDerived();
		applyMat(b, mat);
		b._color = color || '#888888';
		return b;
	}

	// A static ladder volume — a solid box tagged isLadder so FPSCharacterController's mount probe
	// recognizes it. half-extents hx/hy/hz; pos {x,y,z}.
	function ladder(w, hx, hy, hz, pos, color) {
		var b = new AP.RigidBody(new AP.BoxShape(hx, hy, hz), 0);
		b.position.set(pos.x, pos.y, pos.z);
		b.updateDerived();
		applyMat(b, { friction: 0, restitution: 0 });
		b._color = color || '#8a5a2b';
		b.isLadder = true;
		w.addRigidBody(b);
		return b;
	}

	// Spawn a controller. extra merges over the base options (e.g. {scale:0.5, climbSteepSlopes:true}).
	function spawn(w, pos, extra) {
		var opts = Object.assign({ position: pos, color: CONTROLLER_COLOR }, extra || {});
		var p = new AP.FPSCharacterController(w, opts);
		return p;
	}

	// A scripted moving platform: a plain dynamic box, tagged isPlatform (same convention as
	// ladder()'s isLadder), ping-ponging between two points. Driven by VELOCITY only, never a direct
	// position write — a dynamic (finite-mass) body always applies its own linear_velocity to its own
	// position every world.step() regardless of collision mask (the mask only excludes CONTACT
	// resolution, not integration), so writing position directly on top of that fights the solver's
	// own integration of the same tick's velocity and visibly jitters the body. Setting ONLY velocity
	// means the solver's own integration is the sole thing moving it.
	//
	// pointA/pointB are {x,y,z}; speed in units/sec. Returns the body; call .tick(dt) once per frame
	// BEFORE world.step() (see drive()'s movers param below).
	// options.startFrac (0..1, default 0) places the mover along the A->B path at t=0 — 0 = at A heading
	// toward B, 1 = at B heading toward A. Lets a caller choose which direction the mover is heading at
	// spawn, independent of where along the path it starts (matters when a rider approaches from
	// somewhere the mover's default A->B direction would move AWAY from rather than toward).
	function platform(w, hx, hy, hz, pointA, pointB, speed, color, options) {
		var opts = options || {};
		var b = new AP.RigidBody(new AP.BoxShape(hx, hy, hz), 10);
		applyMat(b, { friction: 0, restitution: 0 });
		b._color = color || '#4a7ab0';
		b.isPlatform = true;
		b.setGravity(0, 0, 0);
		b.angular_factor.set(0, 0, 0);
		b.collision_mask = 0; // out of the solver's contact resolution — see comment above (0 = matches
		                      // no group, this engine's own "excluded entirely" convention; see the FPS
		                      // controller's own Body.js for why this differs from a "1" mask elsewhere)
		w.addRigidBody(b);

		var seg = { x: pointB.x - pointA.x, y: pointB.y - pointA.y, z: pointB.z - pointA.z };
		var segLen = Math.sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z);
		var dir = segLen > 0 ? { x: seg.x / segLen, y: seg.y / segLen, z: seg.z / segLen } : { x: 0, y: 0, z: 0 };
		var startFrac = Math.max(0, Math.min(1, opts.startFrac !== undefined ? opts.startFrac : 0));
		var dist = segLen * startFrac;
		var sign = startFrac >= 1 ? -1 : 1;
		b.position.set(pointA.x + seg.x * startFrac, pointA.y + seg.y * startFrac, pointA.z + seg.z * startFrac);
		b.updateDerived();

		b.tick = function (dt) {
			if (segLen <= 0 || dt <= 0) return;
			dist += sign * speed * dt;
			if (dist >= segLen) { dist = segLen; sign = -1; }
			else if (dist <= 0) { dist = 0; sign = 1; }
			b.linear_velocity.set(dir.x * sign * speed, dir.y * sign * speed, dir.z * sign * speed);
		};
		return b;
	}

	// A stationary-center rotating platform (helicopter-blade style): a static-footprint dynamic box
	// spinning in place about its own Y axis, tagged isPlatform + isRotatingPlatform (the latter is what
	// FPSCharacterController's endStep reads to add omega x r tangential velocity on top of linear_velocity
	// — see endStep's acquire). Unlike platform() above, angular_factor.y stays 1 (spin must actually
	// integrate); gravity off and collision_mask=0 for the same reasons platform() uses them (no falling,
	// out of the solver's contact resolution so nothing fights the scripted spin). radPerSec > 0 = CCW
	// looking down +Y (right-hand rule); negative = CW. Returns the body; call .tick(dt) each frame BEFORE
	// world.step(), same bracket as platform()'s movers.
	function rotatingPlatform(w, hx, hy, hz, center, radPerSec, color) {
		var b = new AP.RigidBody(new AP.BoxShape(hx, hy, hz), 10);
		applyMat(b, { friction: 0, restitution: 0 });
		b._color = color || '#4a7ab0';
		b.isPlatform = true;
		b.isRotatingPlatform = true;
		b.setGravity(0, 0, 0);
		b.angular_factor.set(0, 1, 0);
		b.collision_mask = 0;
		b.position.set(center.x, center.y, center.z);
		b.updateDerived();
		w.addRigidBody(b);

		b.tick = function (dt) {
			b.angular_velocity.set(0, radPerSec, 0);
		};
		return b;
	}

	// Register the renderable bodies for the viewer. The only correct source of "everything visible" is
	// the WORLD's actual contents at setup time: the floor, every wall/ramp/object a test built, and the
	// controller's own body + ghost. Pulling straight from `world.bodies` means a test never has to
	// remember to list its own scene explicitly, and can never miss a body it built.
	//
	// A controller that REBUILDS its own body mid-run (crouch/setScale) swaps `controller.body`/
	// `controller._ghost` for fresh RigidBody instances added into the SAME world — those are still
	// caught by this scan if renderables() is called again, or if the viewer re-scans; a one-shot list
	// captured at t=0 will miss a later rebuild's fresh body, same limitation the source carries.
	function renderables(t, controller, extras) {
		var bodies = controller.world.bodies.slice();
		if (extras) extras.forEach(function (b) { if (bodies.indexOf(b) === -1) bodies.push(b); });
		t.bodies = bodies;
		return bodies;
	}

	// Install a tick hook that drives beginStep/world.step/endStep each tick. `cmdFor(tick)` returns
	// the input command for that tick (e.g. {forward:1, sprint:true}); default is no input. The
	// controller brackets a single physics step exactly like a real game loop would.
	//
	// ActionPhysics's own runner exposes `ctx.stepWorld` as a first-class per-tick step override (see
	// runner.js's simulate()), used the same way by both the headless loop and the live render loop — so
	// beginStep/world.step/endStep can be driven from ONE place here, unlike a runner with no such hook
	// (which would need beginStep in onTick and endStep patched into evalTick to run in between).
	// `movers` (optional): array of platform()-built bodies, ticked each frame BEFORE beginStep — same
	// bracket ordering a real game loop would use (mover.tick(dt) before world.step()).
	function drive(t, controller, cmdFor, movers) {
		var tick = 0;
	// Tint the controller's body by its live movement sub-mode so state is readable in the viewer:
	//   green = sliding, orange = slip (too-steep, no footing), base red = walk/normal.
	// Set on controller.body every tick because a crouch/scale rebuild swaps the body instance.
	function tintByState() {
		var b = controller.body;
		if (!b) { return; }
		if (controller.sliding) { b._color = '#3fb950'; }
		else if (controller._isSlipSurface(controller.groundNormal)) { b._color = '#e08a2b'; }
		else { b._color = CONTROLLER_COLOR; }
	}
		t.stepWorld = function (world) {
			tick++;
			if (movers) movers.forEach(function (m) { m.tick(DT); });
			var cmd = (cmdFor ? cmdFor(tick) : null) || {};
			controller.beginStep(cmd, DT);
			world.step(DT);
			controller.endStep(DT);
			tintByState();
		};
	}

	// ---- Sequential multi-block scenes -----------------------------------------------------------------
	// Some tests run N variants of a scene IN TURN inside ONE reused world — wipe + rebuild at the start
	// of each block, watch it play out, move to the next (e.g. sweeping a ramp test across several
	// angles, or a push test across several object masses, one after another in the same viewer run).
	// Built on the same t.stepWorld hook drive() uses, generalized to a CURRENT controller that changes
	// across block boundaries.
	//
	// blocks: array of block descriptors. blockTicks: ticks per block (shared by all blocks).
	// build(world, blockIndex, blockDescriptor) must wipe the world's existing rigid bodies and
	// construct the new block's scene, returning the controller to drive for this block.
	// onBlockTick(local, blockIndex, blockDescriptor, controller, world) runs once per tick, AFTER this
	// tick's beginStep/world.step/endStep have completed, with `local` the 1-based tick index WITHIN the
	// current block — this is where a test records its measurements.
	function sequentialBlocks(t, world, blocks, blockTicks, build, onBlockTick, cmdFor) {
		var cur = null, pbTick = 0;
		function rebuild(idx) {
			world.bodies.slice().forEach(function (b) { world.removeRigidBody(b); });
			cur = build(world, idx, blocks[idx]);
		}
		rebuild(0);
		t.stepWorld = function (w) {
			pbTick++;
			var idx = Math.floor((pbTick - 1) / blockTicks);
			var local = ((pbTick - 1) % blockTicks) + 1;
			if (idx < blocks.length && local === 1 && pbTick > 1) rebuild(idx);
			var cmd = (cmdFor && idx < blocks.length) ? (cmdFor(local, idx, blocks[idx]) || {}) : {};
			cur.beginStep(cmd, DT);
			w.step(DT);
			cur.endStep(DT);
			if (idx < blocks.length) onBlockTick(local, idx, blocks[idx], cur, w);
		};
		return { tickOf: function () { return pbTick; }, controllerOf: function () { return cur; } };
	}

	// ---- SCALE-PARAMETRIC support ---------------------------------------------------------------------
	// Every controller test runs at three character scales. The WORLD is NOT scaled (ramps stay the
	// same angle, walls stay put); only the CHARACTER scales, and the test's own instruments
	// (proportional objects, head-relative drops) + world-unit thresholds scale with it.
	var SCALES = [0.5, 1.0, 2.0];

	// Per-scale helper bundle handed to a scaleTest build fn.
	function scaleHelpers(SC) {
		return {
			SC: SC,
			// world-unit threshold that scales with the character (a speed, a distance the body covers)
			sc: function (x) { return x * SC; },
			flat: flatWorld,
			// scale-PROPORTIONAL object: side and mass scale so its size+mass RATIO to the character is
			// constant (a fixed 1x1x1 object is a step at 2x / an immovable wall at 0.5x). base values
			// are scale-1.
			scaledObject: function (w, baseSide, baseMass, z, color, mat) {
				var s = baseSide * SC, m = baseMass * SC * SC * SC;
				return object(w, s, m, { x: 0, y: s / 2, z: z * SC }, color, mat);
			},
			// A square pillar ladder (1.2x6x1.2 pre-scale, climbable on every face) centered at the
			// world origin, base on the floor.
			pillarLadder: function (w, color) {
				var wd = 0.6 * SC, h = 6 * SC;
				return ladder(w, wd, h / 2, wd, { x: 0, y: h / 2, z: 0 }, color);
			},
			// scale-proportional platform mover: a square-footprint box (side baseSide, thickness
			// baseThick, both scale-1, FULL size not half-extent) traveling between two SCALED points
			// at a scaled speed. pointA/pointB are given in BASE (scale-1) world units.
			splatform: function (w, baseSide, baseThick, pointA, pointB, baseSpeed, color, options) {
				var side = baseSide * SC, thick = baseThick * SC;
				var a = { x: pointA.x * SC, y: pointA.y * SC, z: pointA.z * SC };
				var b = { x: pointB.x * SC, y: pointB.y * SC, z: pointB.z * SC };
				return platform(w, side / 2, thick / 2, side / 2, a, b, baseSpeed * SC, color, options);
			},
			// scale-proportional rotating platform: a long footprint (baseLen along X, baseWidth along Z,
			// baseThick tall, all scale-1, FULL size) spinning about a SCALED center. radPerSec is a
			// world-unit angular rate (NOT distance-scaled — an angle isn't a length).
			srotplatform: function (w, baseLen, baseWidth, baseThick, center, radPerSec, color) {
				var len = baseLen * SC, wid = baseWidth * SC, thick = baseThick * SC;
				var c = { x: center.x * SC, y: center.y * SC, z: center.z * SC };
				return rotatingPlatform(w, len / 2, thick / 2, wid / 2, c, radPerSec, color);
			},
			// drop a box onto the character's ACTUAL head from `above` units over it (head-relative, so
			// impact speed is the same at every scale). side/mass default to 0.8/2.
			dropOnHead: function (w, p, above, side, mass, mat) {
				var s = side !== undefined ? side : 0.8;
				var headTop = p.body.position.y + p.height / 2;
				var b = new AP.RigidBody(new AP.BoxShape(s / 2, s / 2, s / 2), mass !== undefined ? mass : 2);
				b.position.set(0, headTop + above, 0);
				applyMat(b, Object.assign({ friction: 0.4, restitution: 0 }, mat || {}));
				b._color = '#0f0';
				w.addRigidBody(b);
				return b;
			},
			// spawn with this scale baked in; extra merges (e.g. climbSteepSlopes:true)
			spawn: function (w, pos, extra) { return spawn(w, pos, Object.assign({ scale: SC }, extra || {})); },
			// spawn feet-planted at the floor (removes the fall transient a fixed y=1 spawn causes off-scale)
			feetSpawn: function (w, x, z, extra) { return spawn(w, { x: x, y: 0.9 * SC + 0.001, z: z }, Object.assign({ scale: SC }, extra || {})); }
		};
	}

	// Register ONE logical test at all three scales as separate watchable rows: "<name> @0.5" etc.
	// build(t, S) receives the test ctx and the per-scale helper bundle S (S.SC, S.sc, S.scaledObject, S.spawn, ...).
	// ActionPhysics's Runner.test is (group, name, fn, meta) — no separate `id` argument like the
	// source's runner, so id and name are folded into one string here.
	function scaleTest(group, id, name, build, meta) {
		meta = meta || {};
		SCALES.forEach(function (SC) {
			var S = scaleHelpers(SC);
			Runner.test(group, id + ' ' + name + ' @' + SC, function (t) {
				build(t, S);
			}, {
				visual: true,
				steps: meta.steps || 0,
				page: meta.page || group,
				description: (meta.description || '') + '  [scale ' + SC + ']'
			});
		});
	}

	function hsp(p) { var v = p.body.linear_velocity; return Math.hypot(v.x, v.z); }

	// Construct a quaternion directly from the standard axis-angle formula. axis need not be
	// pre-normalized for a UNIT axis like (1,0,0) — every ramp test here rotates about a single
	// cardinal axis.
	function axisAngleQuat(x, y, z, rad) {
		var s = Math.sin(rad / 2);
		return new AP.Quaternion(x * s, y * s, z * s, Math.cos(rad / 2));
	}

	return {
		DT: DT, SCALES: SCALES,
		flatWorld: flatWorld, makeWorld: makeWorld, object: object, staticBox: staticBox, ladder: ladder, platform: platform, rotatingPlatform: rotatingPlatform, spawn: spawn,
		renderables: renderables, drive: drive, sequentialBlocks: sequentialBlocks,
		scaleHelpers: scaleHelpers, scaleTest: scaleTest,
		axisAngleQuat: axisAngleQuat,
		hsp: hsp
	};
});
