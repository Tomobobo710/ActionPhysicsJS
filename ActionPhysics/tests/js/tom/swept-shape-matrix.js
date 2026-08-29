// Tom's Suite — SWEPT SHAPE MATRIX.
// World.shapeIntersect sweeps a base shape along a segment and returns what it hits. Two things have to
// hold for every combination and were each broken at some point:
//   • Every convex BASE shape (box, sphere, capsule, cone, cylinder, convex) must produce a usable swept
//     probe. CapsuleShape.findSupportPoint had two bugs that crashed GJK/EPA: a two-way y-sign that made
//     equatorial supports coplanar (flat simplex), and a 0/0 on a zero-length direction (NaN simplex).
//   • Every TARGET type must be detected: primitive (getContact returns a contact directly), MeshShape and
//     CompoundShape (contacts route through the addContact callback; shapeIntersect used to miss them).
// This sweeps each base straight through each target and asserts at least one hit against that target.
//
// Pure query (no simulation); deterministic, headless == browser.
(function (Runner, U) {
	Runner.suite('tom');

	// A unit-cube triangle mesh (8 verts, 12 tris) centered at the origin.
	function cubeMesh(t, w, H) {
		var v = [
			[-H,-H,-H],[ H,-H,-H],[ H, H,-H],[-H, H,-H],
			[-H,-H, H],[ H,-H, H],[ H, H, H],[-H, H, H]
		];
		var f = [ 0,1,2, 0,2,3,  4,6,5, 4,7,6,  0,4,5, 0,5,1,  3,2,6, 3,6,7,  0,3,7, 0,7,4,  1,5,6, 1,6,2 ];
		return t.mesh(w, v, f, 0, U.withMat({ pos: [0,0,0], color: '#889' }));
	}

	// A two-box compound (crossed bars), added like runner's add() but for a CompoundShape.
	function crossCompound(t, w) {
		var G = t.AP, z = new G.Vector3(0,0,0), q = new G.Quaternion(0,0,0,1);
		var shape = new G.CompoundShape();
		shape.addChild(new G.BoxShape(1, 0.3, 0.3), z, q);
		shape.addChild(new G.BoxShape(0.3, 0.3, 1), z, q);
		var b = new G.RigidBody(shape, 0);
		w.addRigidBody(b); b._color = '#a86'; t.bodies.push(b);
		return b;
	}

	// Base-shape factories — one per convex shape that can be a swept probe.
	var BASES = {
		box:      function (G) { return new G.BoxShape(0.3, 0.3, 0.3); },
		sphere:   function (G) { return new G.SphereShape(0.3); },
		capsule:  function (G) { return new G.CapsuleShape(0.3, 0.8); },
		cone:     function (G) { return new G.ConeShape(0.3, 0.4); },
		cylinder: function (G) { return new G.CylinderShape(0.3, 0.4); },
		convex:   function (G) { return new G.ConvexShape([
			new G.Vector3(0.3,0,0), new G.Vector3(-0.3,0,0), new G.Vector3(0,0.3,0),
			new G.Vector3(0,-0.3,0), new G.Vector3(0,0,0.3), new G.Vector3(0,0,-0.3) ]); }
	};

	// Target-shape factories — one per collision path (primitive / mesh / compound).
	var TARGETS = {
		primitive: function (t, w) { return t.box(w, 1, 1, 1, 0, U.withMat({ pos: [0,0,0], color: '#556' })); },
		mesh:      function (t, w) { return cubeMesh(t, w, 1); },
		compound:  function (t, w) { return crossCompound(t, w); }
	};

	Object.keys(TARGETS).forEach(function (targetName) {
		Runner.test('swept shape matrix', 'every base shape hits a ' + targetName + ' target', function (t) {
			t.log('Sweep each base shape through a ' + targetName + ' target; each must register a hit.');

			var w = t.makeWorld({ gravity: 0 });
			var target = TARGETS[targetName](t, w);

			Object.keys(BASES).forEach(function (baseName) {
				var base = BASES[baseName](t.AP);
				var start = t.vec(-3, 0, 0), end = t.vec(3, 0, 0);
				t.expect(baseName + ' swept through the ' + targetName + ' returns a hit', function () {
					// World.shapeIntersect reports the single nearest hit (or null), not an array — the
					// only body in this scene is `target`, so a single-hit query is sufficient here.
					var hit = w.shapeIntersect(base, start, end);
					var mine = (hit && hit.body === target) ? hit : null;
					var ok = false, detail = baseName + ' hits=' + (mine ? 1 : 0);
					if (mine) {
						var n = mine.normal, nlen = Math.sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
						ok = nlen > 0.5 && isFinite(nlen);
						detail += ' |n|=' + nlen.toFixed(2);
					}
					return { ok: ok, detail: detail };
				});
			});

			t.simulate(w, 2);
		}, {
			visual: true, steps: 0, page: 'swept shape matrix',
			description:
				"Sweeps every convex base shape (box, sphere, capsule, cone, cylinder, convex) straight " +
				"through a " + targetName + " target via World.shapeIntersect and asserts each one returns a " +
				"hit with a sane normal. Guards both the capsule support-point fixes (equatorial coplanarity " +
				"and zero-direction NaN, which crashed GJK/EPA) and the mesh/compound query path (contacts " +
				"routed through the addContact callback that shapeIntersect used to drop)."
		});
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
