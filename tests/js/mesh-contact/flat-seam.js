/**
 * A coplanar seam is not a feature.
 *
 * The flat square these scenes land on is built two ways: ONE static box face, or the TWO coplanar
 * triangles that tile exactly that same square (split on the z=x diagonal, which passes through the
 * origin). Identical surface, identical material, identical drop - so the two must agree. Any
 * difference is the contact set reading its own triangulation rather than the plane it describes.
 *
 * The probe is a WIDE, THIN slab dropped FLAT from a height. Two reasons:
 *
 *  - Flat means the answer is known before the run. Every contact normal is vertical, the facing
 *    surfaces are parallel, and the slip at contact is uniform translation - which friction opposes
 *    uniformly. There is no lever arm anywhere, so the slab can acquire NO rotation: not a small
 *    amount, none. The reference box face scores exactly 0.0000 rad/s, and so does every other
 *    configuration measured here, so this is a rule the engine already meets everywhere else.
 *  - Thin means that if a torque IS invented it cannot hide. The slab's inertia about a horizontal
 *    axis is small, so a spurious couple shows up as visible rotation instead of a 0.01 rad/s
 *    rounding artifact. A 0.5 x 0.5 x 0.5 cube dropped the same way masks it: it needs several
 *    times the torque to turn at all, and reads 0.0000 on both grounds.
 *
 * The third case is the control - the same flat drop inside ONE triangle, off the seam. It is clean
 * on every build, which is what identifies the seam, and not the mesh, as the trigger.
 */
(function (Runner, U) {

	var G = typeof module !== 'undefined' && module.exports
		? require('../../../build/actionphysics.js')
		: window.ActionPhysics;

	var TOTAL = 300;            // the slab has landed and settled long before this
	var TILE = 200 / 55;        // same tile as the perf scene and solver/energy
	var TILE_Y = 0.427;
	var DROP_GAP = 2.0;         // released this far above the surface

	// The probe: 2.4 x 0.24 x 1.8, dropped flat. Wide enough to straddle the diagonal with room to
	// spare, thin enough that a couple cannot pass unnoticed.
	var HX = 1.2, HY = 0.12, HZ = 0.9, MASS = 6;
	// ... and the control, small enough to land entirely inside one triangle (z <= x throughout).
	var CHX = 0.5, CHY = 0.12, CHZ = 0.4, CDX = 0.9, CDZ = -0.1;

	var MAX_W = 0.05;           // rad/s - a flat drop on a flat surface acquires NO rotation
	var MAX_TILT = 0.5;         // deg   - and does not rock off its face

	function material(b) {
		b.friction = U.MAT.friction;
		b.restitution = U.MAT.restitution;
		b.linear_damping = U.MAT.linear_damping;
		b.angular_damping = U.MAT.angular_damping;
		b.angular_friction = U.MAT.angular_friction;
		return b;
	}

	// The flat square, as one box face ('box') or as two coplanar triangles tiling it ('mesh').
	function ground(t, w, kind, keep) {
		var b;
		if (kind === 'box') {
			b = new G.RigidBody(new G.BoxShape(TILE / 2, 0.5, TILE / 2), 0);
			b.position.set(0, TILE_Y - 0.5, 0);
		} else {
			var h = TILE / 2;
			var v = [[-h, TILE_Y, -h], [h, TILE_Y, -h], [h, TILE_Y, h], [-h, TILE_Y, h]]
				.map(function (p) { return new G.Vector3(p[0], p[1], p[2]); });
			b = new G.RigidBody(new G.MeshShape(v, [0, 2, 1, 0, 3, 2]), 0);
		}
		material(b); b._color = '#2f4636';
		w.addRigidBody(b);
		if (keep) t.bodies.push(b);
		return b;
	}

	// One scenario: build the world, drop the slab flat (zero initial spin - that is the whole point),
	// and carry the running peak spin / peak tilt so a momentary bobble cannot be averaged away.
	function scenario(t, kind, dx, dz, hx, hy, hz, keep) {
		// The kept world is the one the browser viewer animates; the rest exist only to be measured.
		var w = keep ? t.makeWorld({ gravity: -9.8 })
			: new G.World(new G.SAPBroadphase(), new G.NarrowPhase(), new G.Solver());
		if (!keep) w.gravity = new G.Vector3(0, -9.8, 0);
		ground(t, w, kind, keep);

		var prop = new G.RigidBody(new G.BoxShape(hx, hy, hz), MASS);
		prop.position.set(dx, TILE_Y + hy + DROP_GAP, dz);
		material(prop); prop._color = '#c98a3a';
		w.addRigidBody(prop);
		if (keep) t.bodies.push(prop);
		return { world: w, prop: prop, peakW: 0, peakTilt: 0 };
	}

	var _dir = null;
	function sample(s) {
		if (!_dir) _dir = new G.Vector3();
		var av = s.prop.angular_velocity;
		var sw = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
		if (sw > s.peakW) s.peakW = sw;
		// Tilt of the slab's own up axis away from world up. Drops flat, stays flat => 0 degrees.
		_dir.set(0, 1, 0);
		s.prop.rotation.transformVectorInPlace(_dir);
		var y = _dir.y > 1 ? 1 : (_dir.y < -1 ? -1 : _dir.y);
		var tilt = Math.acos(y) * 180 / Math.PI;
		if (tilt > s.peakTilt) s.peakTilt = tilt;
	}

	Runner.test('mesh-contact/flat-seam', 'flat slab dropped on a coplanar mesh seam acquires no spin', function (t) {

		// Four worlds, stepped in lockstep through ctx.stepWorld so every one of them is sampled on
		// every tick - the animated one included. Stepping them by hand rather than calling simulate()
		// per world is what lets one test compare two representations of the same surface directly
		// instead of asserting against a number copied from somewhere else.
		var seam = scenario(t, 'mesh', 0, 0, HX, HY, HZ, true);     // animated (and the case under test)
		var face = scenario(t, 'box', 0, 0, HX, HY, HZ, false);     // the same square, one face
		var control = scenario(t, 'mesh', CDX, CDZ, CHX, CHY, CHZ, false);  // off the seam, one triangle
		var controlFace = scenario(t, 'box', CDX, CDZ, CHX, CHY, CHZ, false);
		var all = [seam, face, control, controlFace];
		var steps = 0;

		t.stepWorld = function () {
			for (var i = 0; i < all.length; i++) { all[i].world.step(1 / 60); sample(all[i]); }
			steps++;
		};

		function waiting() { return { ok: false, detail: 'tick ' + steps + '/' + TOTAL }; }

		t.expect('a flat drop on the seam acquires no spin (peak |w| < ' + MAX_W + ' rad/s)', function () {
			if (steps < TOTAL) return waiting();
			return {
				ok: seam.peakW < MAX_W,
				detail: 'peak |w| seam=' + seam.peakW.toFixed(4) + ' rad/s vs one box face=' + face.peakW.toFixed(4)
			};
		});

		t.expect('a flat drop on the seam does not rock off its face (tilt < ' + MAX_TILT + ' deg)', function () {
			if (steps < TOTAL) return waiting();
			return {
				ok: seam.peakTilt < MAX_TILT,
				detail: 'peak tilt seam=' + seam.peakTilt.toFixed(2) + ' deg vs one box face=' + face.peakTilt.toFixed(2)
			};
		});

		t.expect('control: the same drop inside one triangle acquires no spin (peak |w| < ' + MAX_W + ')', function () {
			if (steps < TOTAL) return waiting();
			return {
				ok: control.peakW < MAX_W && control.peakTilt < MAX_TILT,
				detail: 'peak |w|=' + control.peakW.toFixed(4) + ' tilt=' + control.peakTilt.toFixed(2) +
					' deg off the seam, vs ' + controlFace.peakW.toFixed(4) + ' on the box face'
			};
		});

		t.simulate(seam.world, TOTAL);

	}, {
		visual: true, steps: TOTAL, page: 'mesh-contact/flat-seam',
		description: 'A 2.4 x 0.24 x 1.8 slab is dropped FLAT (zero initial spin) from 2 m onto a ' +
			'flat square built two ways: as one static box face, and as the two coplanar triangles that ' +
			'tile that same square. A flat drop on a flat surface has no lever arm anywhere - every ' +
			'contact normal is vertical and the slip is uniform - so the slab must acquire no rotation ' +
			'at all, and must not rock off its face. The box face scores exactly 0.0000 rad/s and 0.00 ' +
			'degrees, and so does the same drop made off the seam inside a single triangle; the seam ' +
			'straddling drop is the case under test.'
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
