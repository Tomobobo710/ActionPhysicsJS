(function (Runner, ActionPhysics, U) {
	Runner.suite('tom');

	var Vector3 = ActionPhysics.Vector3, RigidBody = ActionPhysics.RigidBody,
		BoxShape = ActionPhysics.BoxShape, SphereShape = ActionPhysics.SphereShape,
		CylinderShape = ActionPhysics.CylinderShape, MeshShape = ActionPhysics.MeshShape;

	// An inverted (reversed-winding) box mesh: all face normals point into the box.
	function invertedBoxMesh(hx, hy, hz) {
		var xs = [-1, 1], ys = [-1, 1], zs = [-1, 1], v = [];
		for (var yi = 0; yi < 2; yi++) for (var zi = 0; zi < 2; zi++) for (var xi = 0; xi < 2; xi++)
			v.push(new Vector3(xs[xi] * hx, ys[yi] * hy, zs[zi] * hz));
		var I = function (xi, yi, zi) { return yi * 4 + zi * 2 + xi; };
		var faces = [];
		function quad(p, rev) {
			var f = rev ? [p[0], p[2], p[1], p[0], p[3], p[2]] : [p[0], p[1], p[2], p[0], p[2], p[3]];
			for (var k = 0; k < 6; k++) faces.push(f[k]);
		}
		// -1 winding: roof +Y/-Y/-Z/-Z reversed, +X/-X use the outward order — all point INWARD.
		quad([I(0, 1, 0), I(0, 1, 1), I(1, 1, 1), I(1, 1, 0)], true);  // +Y
		quad([I(0, 0, 0), I(1, 0, 0), I(1, 0, 1), I(0, 0, 1)], true);  // -Y
		quad([I(1, 0, 0), I(1, 0, 1), I(1, 1, 1), I(1, 1, 0)], false); // +X
		quad([I(0, 0, 0), I(0, 1, 0), I(0, 1, 1), I(0, 0, 1)], false); // -X
		quad([I(0, 0, 1), I(1, 0, 1), I(1, 1, 1), I(0, 1, 1)], true);  // +Z
		quad([I(0, 0, 0), I(0, 1, 0), I(1, 1, 0), I(1, 0, 0)], true);  // -Z
		return new MeshShape(v, faces);
	}

	// The NON-inverted twin: the same box with the mirror winding on every face, so its normals point
	// OUT. Control case to confirm whether a behavior is winding-dependent or winding-independent.
	function boxMesh(hx, hy, hz) {
		var xs = [-1, 1], ys = [-1, 1], zs = [-1, 1], v = [];
		for (var yi = 0; yi < 2; yi++) for (var zi = 0; zi < 2; zi++) for (var xi = 0; xi < 2; xi++)
			v.push(new Vector3(xs[xi] * hx, ys[yi] * hy, zs[zi] * hz));
		var I = function (xi, yi, zi) { return yi * 4 + zi * 2 + xi; };
		var faces = [];
		function quad(p, rev) {
			var f = rev ? [p[0], p[2], p[1], p[0], p[3], p[2]] : [p[0], p[1], p[2], p[0], p[2], p[3]];
			for (var k = 0; k < 6; k++) faces.push(f[k]);
		}
		quad([I(0, 1, 0), I(0, 1, 1), I(1, 1, 1), I(1, 1, 0)], false); // +Y
		quad([I(0, 0, 0), I(1, 0, 0), I(1, 0, 1), I(0, 0, 1)], false); // -Y
		quad([I(1, 0, 0), I(1, 0, 1), I(1, 1, 1), I(1, 1, 0)], true);  // +X
		quad([I(0, 0, 0), I(0, 1, 0), I(0, 1, 1), I(0, 0, 1)], true);  // -X
		quad([I(0, 0, 1), I(1, 0, 1), I(1, 1, 1), I(0, 1, 1)], false); // +Z
		quad([I(0, 0, 0), I(0, 1, 0), I(1, 1, 0), I(1, 0, 0)], false); // -Z
		return new MeshShape(v, faces);
	}

	// A closed uv-sphere made of triangles, wound so normals point OUT. This is a "round" mesh with no
	// coplanar faces — the cleanest possible mesh-mesh comparison against the box (which has 6 large
	// coplanar faces). nth segments around, nphi rings. Single pole vertices, wrapped seam => a true
	// closed 2-manifold (every edge shared by exactly 2 triangles).
	function sphereData(r, nth, nphi) {
		var verts = [], ids = [];
		verts.push(new Vector3(0, r, 0));
		ids.push([0]);
		for (var p = 1; p <= nphi - 1; p++) {
			var phi = Math.PI * p / nphi, y = r * Math.cos(phi), rr = r * Math.sin(phi), row = [];
			for (var t = 0; t < nth; t++) {
				var th = 2 * Math.PI * t / nth;
				row.push(verts.push(new Vector3(rr * Math.cos(th), y, rr * Math.sin(th))) - 1);
			}
			ids.push(row);
		}
		verts.push(new Vector3(0, -r, 0));
		ids.push([verts.length - 1]);
		var f = [];
		function tri(a, b, c) { f.push(a, b, c); }
		function T(x) { return x % nth; }
		for (var t = 0; t < nth; t++) tri(ids[1][T(t + 1)], ids[1][T(t)], ids[0][0]);                 // top cap
		for (var p = 1; p <= nphi - 2; p++)
			for (var t = 0; t < nth; t++) {   // rings
				var a = ids[p][t], b = ids[p + 1][t], c = ids[p + 1][T(t + 1)], d = ids[p][T(t + 1)];
				tri(a, c, b); tri(a, d, c);
			}
		for (var t = 0; t < nth; t++) tri(ids[nphi][0], ids[nphi - 1][T(t)], ids[nphi - 1][T(t + 1)]);  // bottom cap
		return { v: verts, f: f };
	}

	// Outward-wound sphere mesh.
	function sphereMesh(r, nth, nphi) {
		var d = sphereData(r, nth, nphi);
		return new MeshShape(d.v, d.f);
	}

	// Inward-wound twin (reversed winding on every triangle) => normals point in. Control for the
	// outward sphere, mirroring how invertedBoxMesh relates to boxMesh.
	function invertedSphereMesh(r, nth, nphi) {
		var d = sphereData(r, nth, nphi), f = [];
		for (var i = 0; i < d.f.length; i += 3) f.push(d.f[i], d.f[i + 2], d.f[i + 1]);
		return new MeshShape(d.v, f);
	}

	Runner.test('mesh collision', 'inverted box smashed into inverted box', function (t) {
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		floor.friction = 0.3;
		w.addRigidBody(floor);

		// Resting inverted box (static), sitting on the floor.
		var rest = new RigidBody(invertedBoxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		// Thrown inverted box (dynamic), pitched at the resting one along +x.
		var thrown = new RigidBody(invertedBoxMesh(half, half, half), 1);
		thrown.position.set(-2.5, 1, 0);
		thrown.linear_velocity.set(5, 0, 0);
		thrown.friction = 0.3;
		thrown.updateDerived();
		w.addRigidBody(thrown);

		// How far the two boxes overlap along x. gap = (restLeft) - (thrownRight).
		// Positive/zero = not touching; large negative = tunneled through.
		// Track the thrown box's orientation too: a box that spins wildly (rotates off identity) on
		// impact has received conflicting contact impulses — it must instead stop cleanly, still upright.
		var minGap = Infinity, touched = false, ticks = 0, minUprightW = Infinity, maxRise = 0;
		t.onTick(function (world, tick) {   // an onTick hook forces the full tick budget in both runs
			ticks = tick;
			var gap = (rest.position.x - half) - (thrown.position.x + half);
			if (gap < minGap) minGap = gap;
			if (gap <= 0.02) touched = true;
			// rotation.w is 1 when upright (no spin); record how far it ever left upright.
			var w = Math.abs(thrown.rotation.w);
			if (w < minUprightW) minUprightW = w;
			// how high the box's center ever rose above its resting height (1.0 on the floor). A clean
			// side-on stop should stay near 1.0; flying up = the contact is pushing it up, not away.
			var rise = thrown.position.y - 1;
			if (rise > maxRise) maxRise = rise;
		});

		t.log('Smash an inverted box into a resting inverted box on a floor. It must collide and stop cleanly, still upright, without spinning out or tunneling through.');

		t.expect('thrown box must collide, stop upright, and end BESIDE (not inside) the resting box (final minGap >= -0.05, final w >= 0.95, end non-overlap)', function (world) {
			if (ticks < 200) return false;   // still running: keep pending
			var finalGap = (rest.position.x - half) - (thrown.position.x + half);   // end state: 0 = touching, negative = still embedded
			return {
				ok: touched && minGap >= -0.05 && minUprightW >= 0.95 && maxRise <= 0.2 && finalGap >= -0.05,
				detail: 'minGap=' + (minGap === Infinity ? 'n/a' : minGap.toFixed(3)) + ' touched=' + touched +
					' minUprightW=' + minUprightW.toFixed(3) + ' maxRise=' + maxRise.toFixed(3) +
					' finalGap=' + finalGap.toFixed(3) +
					' thrown.x=' + thrown.position.x.toFixed(3) + ' thrown.y=' + thrown.position.y.toFixed(3)
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'An inverted (reversed-winding) box mesh thrown into a resting identical box. It must collide as a solid and stop cleanly, still upright, ending beside the resting box and NOT overlapping it — spinning out / tunneling / getting stuck inside is a failure.' });

	Runner.test('mesh collision', 'inverted box stacked on inverted box', function (t) {
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		floor.friction = 0.3;
		w.addRigidBody(floor);

		// Resting inverted box (static), sitting on the floor. Its top face is at y = 2.
		var rest = new RigidBody(invertedBoxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		// Dropped inverted box (dynamic), released above the resting one. Should land on its top face
		// (bottom at y = 2, center at y = 3) and stay stacked, NOT sink through to the floor.
		var dropped = new RigidBody(invertedBoxMesh(half, half, half), 1);
		dropped.position.set(0, 4, 0);
		dropped.updateDerived();
		w.addRigidBody(dropped);

		// Vertical gap = (dropped bottom) - (rest top) = (dropped.y - half) - (rest.y + half).
		// Stacked => gap ~ 0 (dropped.y ~ 3); sank through => dropped.y ~ 1 (on the floor).
		var minGap = Infinity, touched = false, ticks = 0;
		// Tail window checks velocity/position spread, not just final proximity, to catch persistent jitter.
		var TAIL = 40;
		var tailYs = [], maxTailSpeed = 0, maxTailAngSpeed = 0;
		t.onTick(function (world, tick) {   // forces the full tick budget in both runs
			ticks = tick;
			var gap = (dropped.position.y - half) - (rest.position.y + half);
			if (gap < minGap) minGap = gap;
			if (gap <= 0.02) touched = true;

			if (tick > 200 - TAIL) {
				tailYs.push(dropped.position.y);
				var v = dropped.linear_velocity, av = dropped.angular_velocity;
				var speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				var angSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
				if (speed > maxTailSpeed) maxTailSpeed = speed;
				if (angSpeed > maxTailAngSpeed) maxTailAngSpeed = angSpeed;
			}
		});

		t.log('Drop an inverted box onto an inverted box on a floor. It must land and stack on top — it must not sink through to the floor, and it must actually STOP (no persistent jitter/oscillation in the tail).');

		t.expect('dropped box must stack on the resting one and settle (final dropped.y ~ 3, minGap >= -0.5, no tail jitter)', function (world) {
			if (ticks < 200) return false;   // still running: keep pending
			var ySpread = tailYs.length ? (Math.max.apply(null, tailYs) - Math.min.apply(null, tailYs)) : Infinity;
			return {
				ok: touched && minGap >= -0.5 && Math.abs(dropped.position.y - 3) < 0.4 &&
					ySpread < 0.001 && maxTailSpeed < 0.01 && maxTailAngSpeed < 0.01,
				detail: 'dropped.y=' + dropped.position.y.toFixed(3) + ' minGap=' + (minGap === Infinity ? 'n/a' : minGap.toFixed(3)) + ' touched=' + touched +
					' tailYSpread=' + ySpread.toFixed(4) + ' maxTailSpeed=' + maxTailSpeed.toFixed(4) + ' maxTailAngSpeed=' + maxTailAngSpeed.toFixed(4)
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'An inverted (reversed-winding) box dropped onto a resting inverted box. It must land and stack; sinking through to the floor is a failure.' });

	Runner.test('mesh collision', 'normal box stacked on normal box', function (t) {
		// CONTROL: identical drop, but BOTH boxes are non-inverted (outward normals). Determines whether
		// the parallel-face stacking failure is winding-specific or winding-independent.
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		floor.friction = 0.3;
		w.addRigidBody(floor);

		var rest = new RigidBody(boxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		var dropped = new RigidBody(boxMesh(half, half, half), 1);
		dropped.position.set(0, 4, 0);
		dropped.updateDerived();
		w.addRigidBody(dropped);

		var minGap = Infinity, touched = false, ticks = 0;
		var TAIL = 40;
		var tailYs = [], maxTailSpeed = 0, maxTailAngSpeed = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var gap = (dropped.position.y - half) - (rest.position.y + half);
			if (gap < minGap) minGap = gap;
			if (gap <= 0.02) touched = true;

			if (tick > 200 - TAIL) {
				tailYs.push(dropped.position.y);
				var v = dropped.linear_velocity, av = dropped.angular_velocity;
				var speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				var angSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
				if (speed > maxTailSpeed) maxTailSpeed = speed;
				if (angSpeed > maxTailAngSpeed) maxTailAngSpeed = angSpeed;
			}
		});

		t.log('Control: drop a NORMAL box onto a NORMAL box. If it also sinks through, the stacking failure is winding-independent (a parallel-face mesh-mesh bug), not caused by inverted winding. Must also actually STOP, not just numerically land near the target.');

		t.expect('normal box must stack on the resting one and settle (final dropped.y ~ 3, minGap >= -0.5, no tail jitter)', function (world) {
			if (ticks < 200) return false;
			var ySpread = tailYs.length ? (Math.max.apply(null, tailYs) - Math.min.apply(null, tailYs)) : Infinity;
			return {
				ok: touched && minGap >= -0.5 && Math.abs(dropped.position.y - 3) < 0.4 &&
					ySpread < 0.001 && maxTailSpeed < 0.01 && maxTailAngSpeed < 0.01,
				detail: 'dropped.y=' + dropped.position.y.toFixed(3) + ' minGap=' + (minGap === Infinity ? 'n/a' : minGap.toFixed(3)) + ' touched=' + touched +
					' tailYSpread=' + ySpread.toFixed(4) + ' maxTailSpeed=' + maxTailSpeed.toFixed(4) + ' maxTailAngSpeed=' + maxTailAngSpeed.toFixed(4)
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'Control: the same drop but with non-inverted (outward) box meshes. Records whether this pairing behaves differently from the inverted one.' });

	Runner.test('mesh collision', 'normal box smashed into normal box', function (t) {
		// CONTROL for the inverted-slam test. Identical slam with NON-inverted (outward) boxes. If a
		// normal box slams and stops cleanly & upright while an inverted one spins out, the wig-out is
		// winding-dependent — a contact-normal bug in TriangleTriangle that ignores reversed winding.
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		floor.friction = 0.3;
		w.addRigidBody(floor);

		var rest = new RigidBody(boxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		var thrown = new RigidBody(boxMesh(half, half, half), 1);
		thrown.position.set(-2.5, 1, 0);
		thrown.linear_velocity.set(5, 0, 0);
		thrown.friction = 0.3;
		thrown.updateDerived();
		w.addRigidBody(thrown);

		var minGap = Infinity, touched = false, ticks = 0, minUprightW = Infinity, maxRise = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var gap = (rest.position.x - half) - (thrown.position.x + half);
			if (gap < minGap) minGap = gap;
			if (gap <= 0.02) touched = true;
			var w = Math.abs(thrown.rotation.w);
			if (w < minUprightW) minUprightW = w;
			var rise = thrown.position.y - 1;
			if (rise > maxRise) maxRise = rise;
		});

		t.log('Control: smash a NORMAL box into a NORMAL box. Expected to stop cleanly and upright. Compare against the inverted slam — if this stays upright but the inverted one spins out, the failure is winding-dependent.');

		t.expect('normal box must collide, stop upright, and end BESIDE (not inside) the resting box (final minGap >= -0.05, final w >= 0.95, end non-overlap)', function (world) {
			if (ticks < 200) return false;
			var finalGap = (rest.position.x - half) - (thrown.position.x + half);   // end state: 0 = touching, negative = still embedded
			return {
				ok: touched && minGap >= -0.05 && minUprightW >= 0.95 && maxRise <= 0.2 && finalGap >= -0.05,
				detail: 'minGap=' + (minGap === Infinity ? 'n/a' : minGap.toFixed(3)) + ' touched=' + touched +
					' minUprightW=' + minUprightW.toFixed(3) + ' maxRise=' + maxRise.toFixed(3) +
					' finalGap=' + finalGap.toFixed(3) +
					' thrown.x=' + thrown.position.x.toFixed(3) + ' thrown.y=' + thrown.position.y.toFixed(3)
			};
		});
		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'Control: the inverted-slam test but with non-inverted (outward) box meshes. Records whether the thrown box stays upright here versus the inverted scenario.' });

	// Helper for the cross-winding slam tests below: same scene, pick winding of rest (static) and thrown.
	function mixedSlam(t, restMeshFn, thrownMeshFn, label) {
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		floor.friction = 0.3;
		w.addRigidBody(floor);

		var rest = new RigidBody(restMeshFn(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		var thrown = new RigidBody(thrownMeshFn(half, half, half), 1);
		thrown.position.set(-2.5, 1, 0);
		thrown.linear_velocity.set(5, 0, 0);
		thrown.friction = 0.3;
		thrown.updateDerived();
		w.addRigidBody(thrown);

		var minGap = Infinity, touched = false, ticks = 0, minUprightW = Infinity, maxRise = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var gap = (rest.position.x - half) - (thrown.position.x + half);
			if (gap < minGap) minGap = gap;
			if (gap <= 0.02) touched = true;
			var w = Math.abs(thrown.rotation.w);
			if (w < minUprightW) minUprightW = w;
			var rise = thrown.position.y - 1;
			if (rise > maxRise) maxRise = rise;
		});

		t.log(label);

		t.expect('thrown box must collide, stop upright, and end BESIDE (not inside) the resting box (final minGap >= -0.05, final w >= 0.95, end non-overlap)', function (world) {
			if (ticks < 200) return false;
			var finalGap = (rest.position.x - half) - (thrown.position.x + half);   // end state: 0 = touching, negative = still embedded
			return {
				ok: touched && minGap >= -0.05 && minUprightW >= 0.95 && maxRise <= 0.2 && finalGap >= -0.05,
				detail: 'minGap=' + (minGap === Infinity ? 'n/a' : minGap.toFixed(3)) + ' touched=' + touched +
					' minUprightW=' + minUprightW.toFixed(3) + ' maxRise=' + maxRise.toFixed(3) +
					' finalGap=' + finalGap.toFixed(3) +
					' thrown.x=' + thrown.position.x.toFixed(3) + ' thrown.y=' + thrown.position.y.toFixed(3)
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}

	Runner.test('mesh collision', 'inverted rest box smashed by normal box', function (t) {
		mixedSlam(t, invertedBoxMesh, boxMesh,
			'MIXED WINDING (rest=INVERTED, thrown=NORMAL): records how this pairing behaves.');
		}, { visual: true, page: 'mesh', steps: 200, description: 'Cross-winding slam: an inverted (reversed-winding) resting box hit by a normal box. Just records the behavior; no mechanism is assumed.' });

	Runner.test('mesh collision', 'normal rest box smashed by inverted box', function (t) {
		mixedSlam(t, boxMesh, invertedBoxMesh,
			'MIXED WINDING (rest=NORMAL, thrown=INVERTED): records how this pairing behaves.');
		}, { visual: true, page: 'mesh', steps: 200, description: 'Cross-winding slam: a normal resting box hit by an inverted (reversed-winding) box. Just records the behavior; no mechanism is assumed.' });

	Runner.test('mesh collision', 'sphere dropped onto inverted mesh box', function (t) {
		// Convex (sphere) body vs a MESH body. This pair routes through meshConvex (GJK/EPA), the path
		// used for e.g. the character and the compound-mesh test, NOT the broken mesh-vs-mesh
		// TriangleTriangle path. Records whether a convex body stacks on a mesh box.
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1, r = 0.5;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		w.addRigidBody(floor);

		// Inverted mesh box as the platform (top face at y=2).
		var rest = new RigidBody(invertedBoxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		// Sphere dropped from above. Resting on the box top: center at y = 2 + r = 2.5. Spawns at y=5.
		var sphere = new RigidBody(new SphereShape(r), 1);
		sphere.position.set(0, 5, 0);
		sphere.updateDerived();
		w.addRigidBody(sphere);

		var ticks = 0, maxRise = 0;
		// Tail window checks velocity/position spread, not just final proximity, to catch persistent jitter.
		var TAIL = 40;
		var tailYs = [], maxTailSpeed = 0, maxTailAngSpeed = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			// Rise relative to SPAWN (y=5): the sphere should only ever fall from here. Going above it
			// means the contact launched it upward. Its legit resting point is y=2.5 (below spawn).
			var rise = sphere.position.y - 5;
			if (rise > maxRise) maxRise = rise;

			if (tick > 200 - TAIL) {
				tailYs.push(sphere.position.y);
				var v = sphere.linear_velocity, av = sphere.angular_velocity;
				var speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				var angSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
				if (speed > maxTailSpeed) maxTailSpeed = speed;
				if (angSpeed > maxTailAngSpeed) maxTailAngSpeed = angSpeed;
			}
		});

		t.log('Drop a sphere onto an inverted mesh box. A convex body against a mesh uses meshConvex, so it may behave like the (working) character/compound cases rather than the broken mesh-mesh path.');

		t.expect('sphere lands, rests at y ~ 2.5, and actually STOPS (no persistent jitter/oscillation/spin in the tail)', function (world) {
			if (ticks < 200) return false;
			var ySpread = tailYs.length ? (Math.max.apply(null, tailYs) - Math.min.apply(null, tailYs)) : Infinity;
			return {
				ok: Math.abs(sphere.position.y - 2.5) < 0.4 && maxRise <= 0.2 &&
					ySpread < 0.001 && maxTailSpeed < 0.01 && maxTailAngSpeed < 0.01,
				detail: 'sphere.y=' + sphere.position.y.toFixed(3) + ' (rest ~ 2.5) maxRise=' + maxRise.toFixed(3) +
					' tailYSpread=' + ySpread.toFixed(4) + ' maxTailSpeed=' + maxTailSpeed.toFixed(4) +
					' maxTailAngSpeed=' + maxTailAngSpeed.toFixed(4)
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'A convex sphere dropped onto an inverted mesh box. Records whether a convex-vs-mesh collision (meshConvex path) stacks correctly and actually comes to rest (not just numerically near the target), versus the broken mesh-mesh path.' });

	// Generic convex-on-inverted-mesh-box drop. buildShape(G) makes the shape; restY is the body's
	// expected resting center height on the box top (y=2); spawn at y=5. Records rest + whether the
	// contact ever launched it above spawn.
	function convexMeshDrop(t, label, buildShape, restY, extra) {
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		w.addRigidBody(floor);

		var rest = new RigidBody(invertedBoxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		var body = new RigidBody(buildShape(t.AP), 1);
		body.position.set(0, 5, 0);
		body.updateDerived();
		w.addRigidBody(body);

		var ticks = 0, maxRise = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var rise = body.position.y - 5;   // relative to spawn; body should only fall from here
			if (rise > maxRise) maxRise = rise;
		});

		t.log('Drop ' + label + ' onto an inverted mesh box. Convex-vs-mesh uses meshConvex (GJK/EPA), not the mesh-vs-mesh TriangleTriangle path.');

		t.expect(label + ' must land and rest on the inverted mesh box (final y ~ ' + restY.toFixed(1) + ', never launched above spawn)', function (world) {
			if (ticks < 200) return false;
			return {
				ok: Math.abs(body.position.y - restY) < 0.4 && maxRise <= 0.2,
				detail: label + '.y=' + body.position.y.toFixed(3) + ' (rest ~ ' + restY.toFixed(1) + ') maxRise=' + maxRise.toFixed(3) + (extra ? ' ' + extra(body) : '')
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}

	Runner.test('mesh collision', 'cone dropped onto inverted mesh box', function (t) {
		// Cone(radius, half_height): rest centered on the box top. A cone lands point-up or point-down
		// depending on inertia; restY is approximate. Record actual behavior; don't assume which way.
		convexMeshDrop(t, 'cone',
			function (G) { return new G.ConeShape(0.5, 0.5); },
			2 + 0.5, function (b) { return 'spinW=' + b.rotation.w.toFixed(2); });
	}, { visual: true, page: 'mesh', steps: 200, description: 'A convex cone dropped onto an inverted mesh box. Records rest height and whether it was launched, via the meshConvex path.' });

	Runner.test('mesh collision', 'capsule dropped onto inverted mesh box', function (t) {
		// Capsule(radius, total_height): lies flat on the box top, center at y = 2 + radius.
		convexMeshDrop(t, 'capsule',
			function (G) { return new G.CapsuleShape(0.3, 1.0); },
			2 + 0.3, function (b) { return 'spinW=' + b.rotation.w.toFixed(2); });
	}, { visual: true, page: 'mesh', steps: 200, description: 'A convex capsule dropped onto an inverted mesh box. Records rest height and whether it was launched, via the meshConvex path.' });

	Runner.test('mesh collision', 'cylinder rolled sideways onto inverted mesh box', function (t) {
		// A cylinder set on its SIDE (rotated 90° about z) so its CURVED surface is what contacts the box
		// top. This exercises the meshConvex path against a curved (non-faceted, no flat face) surface.
		// Cylinder(radius, half_height) runs along y; rotate to lie along x, resting on the rounded band
		// at center y = 2 + radius.
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1, r = 0.5;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		w.addRigidBody(floor);

		var rest = new RigidBody(invertedBoxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		var cyl = new RigidBody(new CylinderShape(r, 0.5), 1);
		cyl.position.set(0, 5, 0);
		cyl.rotation.set(0, 0, 1, 0);   // 90° about z: cylinder lies sideways, curved band facing down
		cyl.updateDerived();
		w.addRigidBody(cyl);

		var ticks = 0, maxRise = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var rise = cyl.position.y - 5;   // relative to spawn; should only fall from here
			if (rise > maxRise) maxRise = rise;
		});

		t.log('Roll a cylinder onto its side and drop it on an inverted mesh box so the CURVED band hits the box. Convex-vs-mesh (meshConvex).');

		t.expect('cylinder must land on its curved side and rest on the inverted mesh box (final y ~ 2.5, never launched above spawn)', function (world) {
			if (ticks < 200) return false;
			return {
				ok: Math.abs(cyl.position.y - (2 + r)) < 0.4 && maxRise <= 0.2,
				detail: 'cyl.y=' + cyl.position.y.toFixed(3) + ' (rest ~ ' + (2 + r).toFixed(1) + ') maxRise=' + maxRise.toFixed(3) + ' spinW=' + cyl.rotation.w.toFixed(2)
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'A cylinder laid on its side so the curved surface contacts an inverted mesh box. Records rest height and whether it was launched, via the meshConvex path.' });

	Runner.test('mesh collision', 'mesh sphere dropped onto inverted mesh box', function (t) {
		// A MESH sphere vs a MESH box — meshes on BOTH sides, so it routes through the (suspect)
		// mesh-vs-mesh TriangleTriangle path, unlike the convex SphereShape test below. If an outward
		// sphere stacks on a mesh box while the box-vs-box cases fail, the bug is face-related (boxes).
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1, r = 0.5;
		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0); floor.updateDerived(); w.addRigidBody(floor);

		var rest = new RigidBody(invertedBoxMesh(half, half, half), 0);   // top face at y=2
		rest.position.set(0, 1, 0); rest.updateDerived(); w.addRigidBody(rest);

		var ms = new RigidBody(sphereMesh(r, 16, 10), 1);   // outward mesh sphere
		ms.position.set(0, 5, 0); ms.updateDerived(); w.addRigidBody(ms);

		var ticks = 0, maxRise = 0;
		// Tail window checks velocity/position spread, not just final proximity, to catch persistent jitter.
		var TAIL = 40;
		var tailYs = [], maxTailSpeed = 0, maxTailAngSpeed = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var rise = ms.position.y - 5;   // relative to spawn; should only fall
			if (rise > maxRise) maxRise = rise;

			if (tick > 200 - TAIL) {
				tailYs.push(ms.position.y);
				var v = ms.linear_velocity, av = ms.angular_velocity;
				var speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				var angSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
				if (speed > maxTailSpeed) maxTailSpeed = speed;
				if (angSpeed > maxTailAngSpeed) maxTailAngSpeed = angSpeed;
			}
		});

		t.log('Drop a MESH sphere (outward, 288 tris) onto an inverted mesh box. Both bodies are meshes => the mesh-mesh TriangleTriangle path. It should land and rest at y = 2 + r = 2.5.');

		t.expect('mesh sphere lands, rests at y ~ 2.5, and actually STOPS (no persistent jitter/oscillation/spin in the tail)', function (world) {
			if (ticks < 200) return false;
			var ySpread = tailYs.length ? (Math.max.apply(null, tailYs) - Math.min.apply(null, tailYs)) : Infinity;
			return {
				ok: Math.abs(ms.position.y - 2.5) < 0.4 && maxRise <= 0.2 &&
					ySpread < 0.001 && maxTailSpeed < 0.01 && maxTailAngSpeed < 0.01,
				detail: 'ms.y=' + ms.position.y.toFixed(3) + ' (rest ~ 2.5) maxRise=' + maxRise.toFixed(3) +
					' tailYSpread=' + ySpread.toFixed(4) + ' maxTailSpeed=' + maxTailSpeed.toFixed(4) +
					' maxTailAngSpeed=' + maxTailAngSpeed.toFixed(4)
			};
		});
		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'A mesh-authored sphere dropped onto an inverted mesh box (meshes on both sides, mesh-mesh path). Records whether a round outward mesh stacks - and actually comes to rest - where box meshes fail.' });

	Runner.test('mesh collision', 'outward mesh sphere stacked on outward mesh sphere', function (t) {
		// Two round outward meshes vs each other — the least box-like mesh-mesh case there is.
		// Outward sphere on outward sphere. If the round shape stacks cleanly (unlike boxes), mesh-mesh
		// is innately fine and the box bug is about its flat coplanar faces / winding.
		var w = t.makeWorld({ gravity: -9.8 });
		var r = 0.5;
		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0); floor.updateDerived(); w.addRigidBody(floor);

		var bottom = new RigidBody(sphereMesh(r, 16, 10), 0);   // rests center y = r = 0.5
		bottom.position.set(0, r, 0); bottom.updateDerived(); w.addRigidBody(bottom);

		var top = new RigidBody(sphereMesh(r, 16, 10), 1);             // should rest center y = 3r = 1.5
		top.position.set(0, 2.5, 0); top.updateDerived(); w.addRigidBody(top);   // spawn 1 diameter above rest

		var ticks = 0, maxRise = 0;
		var TAIL = 40;
		var tailYs = [], maxTailSpeed = 0, maxTailAngSpeed = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var rise = top.position.y - 2.5;   // relative to spawn; should only fall
			if (rise > maxRise) maxRise = rise;

			if (tick > 200 - TAIL) {
				tailYs.push(top.position.y);
				var v = top.linear_velocity, av = top.angular_velocity;
				var speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				var angSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
				if (speed > maxTailSpeed) maxTailSpeed = speed;
				if (angSpeed > maxTailAngSpeed) maxTailAngSpeed = angSpeed;
			}
		});

		t.log('Drop an outward mesh sphere onto a resting outward mesh sphere. Round-mesh round-mesh. It should settle at top center y = 3r = 1.5.');

		t.expect('top mesh sphere rests on the bottom sphere (not penetrating, not on the ground, no jitter or spin)', function (world) {
			if (ticks < 200) return false;
			var gap = (top.position.y - r) - (bottom.position.y + r);   // contact gap between the two sphere surfaces
			var ySpread = tailYs.length ? (Math.max.apply(null, tailYs) - Math.min.apply(null, tailYs)) : Infinity;
			return {
				ok: gap >= -0.05 &&          // not penetrating the bottom sphere
					top.position.y > 1.0 &&      // not on the ground (fell through to floor at y~0.5)
					maxRise <= 0.2 &&            // not launched upward
					ySpread < 0.001 &&           // no vertical jitter
					maxTailSpeed < 0.01 &&       // not moving
					maxTailAngSpeed < 0.01,      // not spinning
				detail: 'top.y=' + top.position.y.toFixed(3) + ' (rest ~ 1.5) gap=' + gap.toFixed(3) +
					' maxRise=' + maxRise.toFixed(3) + ' tailYSpread=' + ySpread.toFixed(4) +
					' maxTailSpeed=' + maxTailSpeed.toFixed(4) + ' maxTailAngSpeed=' + maxTailAngSpeed.toFixed(4)
			};
		});
		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'Two outward round sphere meshes. If they stack cleanly while boxes fail, the box bug is flat-face/winding specific; if they also sink, mesh-mesh is broadly broken.' });

	Runner.test('mesh collision', 'inverted mesh sphere stacked on inverted mesh sphere', function (t) {
		// Mirror of the box inverted-stack failure: two INWARD-wound sphere meshes. Rounds out whether
		// the inverted mesh-mesh failure is a box-face phenomenon or applies to any inverted mesh.
		var w = t.makeWorld({ gravity: -9.8 });
		var r = 0.5;
		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0); floor.updateDerived(); w.addRigidBody(floor);

		var bottom = new RigidBody(invertedSphereMesh(r, 16, 10), 0);
		bottom.position.set(0, r, 0); bottom.updateDerived(); w.addRigidBody(bottom);

		var top = new RigidBody(invertedSphereMesh(r, 16, 10), 1);
		top.position.set(0, 2.5, 0); top.updateDerived(); w.addRigidBody(top);   // spawn 1 diameter above rest

		var ticks = 0, maxRise = 0;
		var TAIL = 40;
		var tailYs = [], maxTailSpeed = 0, maxTailAngSpeed = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			var rise = top.position.y - 2.5;
			if (rise > maxRise) maxRise = rise;

			if (tick > 200 - TAIL) {
				tailYs.push(top.position.y);
				var v = top.linear_velocity, av = top.angular_velocity;
				var speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
				var angSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
				if (speed > maxTailSpeed) maxTailSpeed = speed;
				if (angSpeed > maxTailAngSpeed) maxTailAngSpeed = angSpeed;
			}
		});

		t.log('Drop an INVERTED (inward-wound) mesh sphere onto a resting INVERTED mesh sphere. Control for the outward pair.');

		t.expect('top inverted mesh sphere rests on the bottom sphere (not penetrating, not on the ground, no jitter or spin)', function (world) {
			if (ticks < 200) return false;
			var gap = (top.position.y - r) - (bottom.position.y + r);
			var ySpread = tailYs.length ? (Math.max.apply(null, tailYs) - Math.min.apply(null, tailYs)) : Infinity;
			return {
				ok: gap >= -0.05 &&          // not penetrating the bottom sphere
					top.position.y > 1.0 &&      // not on the ground (fell through to floor at y~0.5)
					maxRise <= 0.2 &&            // not launched upward
					ySpread < 0.001 &&           // no vertical jitter
					maxTailSpeed < 0.01 &&       // not moving
					maxTailAngSpeed < 0.01,      // not spinning
				detail: 'top.y=' + top.position.y.toFixed(3) + ' (rest ~ 1.5) gap=' + gap.toFixed(3) +
					' maxRise=' + maxRise.toFixed(3) + ' tailYSpread=' + ySpread.toFixed(4) +
					' maxTailSpeed=' + maxTailSpeed.toFixed(4) + ' maxTailAngSpeed=' + maxTailAngSpeed.toFixed(4)
			};
		});
		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'Two INWARD-wound sphere meshes. Isolates whether the inverted mesh-mesh failure is specific to box faces or affects any inverted mesh (round included).' });

	Runner.test('mesh collision', 'PROBE: contact normals during inverted smash', function (t) {
		// Same scene as the inverted smash, but instead of asserting outcomes we read the WORLD's live
		// contact manifolds each tick and log the normals the mesh-mesh path actually produced. This is
		// data-gathering only: we want to SEE the conflicting/erratic contact set, not decide anything.
		var w = t.makeWorld({ gravity: -9.8 });
		var half = 1;

		var floor = new RigidBody(new BoxShape(20, 0.5, 20), 0);
		floor.position.set(0, -0.5, 0);
		floor.updateDerived();
		floor.friction = 0.3;
		w.addRigidBody(floor);

		var rest = new RigidBody(invertedBoxMesh(half, half, half), 0);
		rest.position.set(0, 1, 0);
		rest.updateDerived();
		w.addRigidBody(rest);

		var thrown = new RigidBody(invertedBoxMesh(half, half, half), 1);
		thrown.position.set(-2.5, 1, 0);
		thrown.linear_velocity.set(5, 0, 0);
		thrown.friction = 0.3;
		thrown.updateDerived();
		w.addRigidBody(thrown);

		// Quantize a world-space normal's dominant axis into a compact tag.
		function axis(n) {
			var ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z);
			if (ax >= ay && ax >= az) return (n.x > 0 ? '+x' : '-x');
			if (ay >= ax && ay >= az) return (n.y > 0 ? '+y' : '-y');
			return (n.z > 0 ? '+z' : '-z');
		}

		// Per-first-contact-tick snapshot of normal diversity.
		var firstSplit = null, totalTicks = 0, wholeRun = { count: 0, tags: {} };
		var allManifolds = { count: 0, points: 0 }, minGap = Infinity;
		// Motion telemetry: the thrown box now SEPARATES, but the user reports it does so via violent
		// jitter / teleports rather than a clean stop. Track the worst single-tick position jump (which
		// catches a deep-penetration correction snapping the box elsewhere), max speed reached, and how
		// often its horizontal velocity flips sign (back-and-forth oscillation). Position is read in the
		// pre-step hook, so delta = (this hook) - (last hook) covers one full step (integration + any
		// contact correction).
		var prevX = null, maxJump = 0, maxJumpTick = 0, maxSpeed = 0, maxSpeedTick = 0,
			velocityFlips = 0, lastVxSign = 0;
		var maxPairDepth = 0, maxPairDepthTick = 0, depthDiag = null;   // deepest box-pair penetration seen (driver of the position snap)
		var impactWindow = [];   // (tick, x, vx, gap) samples so we can SEE the motion around contact
		t.onTick(function (world, tick) {
			totalTicks = tick;
			var vx = thrown.linear_velocity.x;
			var speed = Math.abs(vx);
			if (speed > maxSpeed) { maxSpeed = speed; maxSpeedTick = tick; }

			if (lastVxSign !== 0 && ((vx > 0 && lastVxSign < 0) || (vx < 0 && lastVxSign > 0))) {
				velocityFlips++;
			}
			if (vx !== 0) lastVxSign = (vx > 0 ? 1 : -1);

			var x = thrown.position.x;
			var gap = (rest.position.x - half) - (thrown.position.x + half);
			if (gap < minGap) minGap = gap;
			if (prevX !== null) {
				var jump = Math.abs(x - prevX);
				if (jump > maxJump) { maxJump = jump; maxJumpTick = tick; }
			}
			prevX = x;

			// Sample the trajectory densely while the boxes are near each other (|gap| < 3), sparse elsewhere.
			if (Math.abs(gap) < 3 || jumpWindow()) impactWindow.push([tick, +x.toFixed(2), +vx.toFixed(1), +gap.toFixed(2)]);

			var n = 0, tags = {};
			var anyManifolds = 0, anyPoints = 0;
			var manifoldIter = world.narrowphase.manifolds.values();
			for (var mr = manifoldIter.next(); !mr.done; mr = manifoldIter.next()) {
				var m = mr.value;
				anyManifolds++;
				for (var i = 0; i < m.points.length; i++) {
					var p = m.points[i];
					anyPoints += (p ? 1 : 0);
					var isPair = (m.bodyA === rest && m.bodyB === thrown) || (m.bodyA === thrown && m.bodyB === rest);
					if (!isPair || !p) continue;
					n++;
					// signedDistance: positive = overlapping/penetrating (this engine's own convention -
					// negative when separated, see ContactDetails.js). "penetration_depth" here is that.
					tags[axis(p.normal)] = 1;
					if (p.signedDistance > maxPairDepth) {
						maxPairDepth = p.signedDistance; maxPairDepthTick = tick;
						depthDiag = [];
						for (var d = 0; d < m.points.length; d++) {
							var q = m.points[d];
							if (q) depthDiag.push(q.point.x.toFixed(2) + ',' + q.point.y.toFixed(2) + 'd' + q.signedDistance.toFixed(2) + axis(q.normal));
						}
					}
				}
			}
			var distinct = Object.keys(tags).length;
			wholeRun.count += n;
			for (var k in tags) wholeRun.tags[k] = (wholeRun.tags[k] || 0) + 1;
			allManifolds.count += anyManifolds;
			allManifolds.points += anyPoints;
			if (n > 0 && distinct >= 2 && firstSplit === null) {
				firstSplit = { tick: tick, count: n, tags: tags };
			}
		});
		function jumpWindow() { return impactWindow.length < 80; }

		t.log('Smash two inverted boxes together. The mesh-mesh path emits one unilateral contact per triangle pair. If multiple DIFFERENT normals (esp. opposing ones) coexist in a single tick, the box gets contradictory impulses -> spins/wiggles.');

		t.expect('PROBE records contact-normal diversity (no pass/fail; just data)', function (world) {
			if (totalTicks < 200) return false;   // keep pending until the run finishes so we report the full accumulation
			return {
				ok: true,   // always okay: this test only gathers information
				detail: 'pairPoints=' + wholeRun.count +
					' distinctNormalsSeen=' + Object.keys(wholeRun.tags).join(',') +
					' worldManifolds=' + allManifolds.count + ' worldPoints=' + allManifolds.points +
					' minGap=' + (minGap === Infinity ? 'n/a' : minGap.toFixed(3)) +
					' finalGap=' + ((rest.position.x - half) - (thrown.position.x + half)).toFixed(3) +
					' maxJump=' + maxJump.toFixed(2) + '@' + maxJumpTick +
					' maxSpeed=' + maxSpeed.toFixed(1) + '@' + maxSpeedTick +
					' maxDepth=' + maxPairDepth.toFixed(2) + '@' + maxPairDepthTick +
					(depthDiag ? ' [Pts:' + depthDiag.join(' ') + ']' : '') +
					' vxFlips=' + velocityFlips +
					(firstSplit ? ' SPLIT@' + firstSplit.tick + ' count=' + firstSplit.count + ' dirs=' + Object.keys(firstSplit.tags).join(',') : ' noSplit') +
					' traj[' + impactWindow.map(function (s) { return s[0] + ':' + s[1] + '/' + s[2]; }).join(' ') + ']'
			};
		});

		U.syncBodies(t, w);
		t.simulate(w, 200);
	}, { visual: true, page: 'mesh', steps: 200, description: 'DATA probe: reads live contact manifolds during an inverted box smash and reports how many distinct/opposing normals coexist in a tick. Always passes; purely informational.' });
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil);