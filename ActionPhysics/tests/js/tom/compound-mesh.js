// Tom's Suite — COMPOUND vs MESH.
// A compound body (two crossed boxes) dropped onto a MESH body — this pairing has previously exposed
// real bugs where a compound child's collision proxy failed to resolve back to its real body before a
// contact event fired against it. Oracle: it does NOT throw, and the cross settles ON the mesh (rests,
// no tunnel).
//
// Fully scripted / deterministic (headless == browser).
(function (Runner, U) {
	Runner.suite('tom');

	var ARM_LEN = 1.5, ARM_THICK = 0.2;   // each cross arm: long in one axis, thin in the others
	var DROP_Y = 3;                        // spawn height above the mesh top
	var TOTAL = 240;

	// A flat square mesh platform, top face at y=0 (two triangles). Static (mass 0).
	// Verts as [[x,y,z],...], faces as flat index triples — the shape t.mesh expects.
	function meshPlatform(t, w, half) {
		var v = [[-half, 0, -half], [half, 0, -half], [half, 0, half], [-half, 0, half]];
		var f = [0, 1, 2, 0, 2, 3];
		return U.meshBody(t, w, v, f, 0, { pos: [0, 0, 0], color: '#556' });
	}

	// A cubic MESH BOX platform: a closed, outward-wound box, top face at y=+half. This turns the ground
	// into a real closed mesh body, so a compound body dropped onto it now collides mesh-body vs mesh-body
	// through TriangleTriangle (not the flat-panel meshConvex path). Oracle is the same: the cross must
	// settle resting on the box's top face — which may well expose the mesh-mesh breakage here too.
	function meshBoxPlatform(t, w, half) {
		var xs = [-half, half], ys = [-half, half], zs = [-half, half], v = [];
		for (var yi = 0; yi < 2; yi++) for (var zi = 0; zi < 2; zi++) for (var xi = 0; xi < 2; xi++)
			v.push([xs[xi], ys[yi], zs[zi]]);
		var I = function (xi, yi, zi) { return yi * 4 + zi * 2 + xi; };
		var faces = [];
		function quadf(p0, p1, p2, p3) { // two triangles, outward winding
			faces.push(p0, p1, p2, p0, p2, p3);
		}
		quadf(I(0, 1, 0), I(0, 1, 1), I(1, 1, 1), I(1, 1, 0)); // +Y
		quadf(I(0, 0, 0), I(1, 0, 0), I(1, 0, 1), I(0, 0, 1)); // -Y
		quadf(I(1, 0, 0), I(1, 0, 1), I(1, 1, 1), I(1, 1, 0)); // +X
		quadf(I(0, 0, 0), I(0, 1, 0), I(0, 1, 1), I(0, 0, 1)); // -X
		quadf(I(0, 0, 1), I(1, 0, 1), I(1, 1, 1), I(0, 1, 1)); // +Z
		quadf(I(0, 0, 0), I(0, 1, 0), I(1, 1, 0), I(1, 0, 0)); // -Z
		return U.meshBody(t, w, v, faces, 0, { pos: [0, -half, 0], color: '#565' });
	}

	// Build a two-box cross as one compound RigidBody and add it to the world, mirroring runner's add():
	// apply material + pos, addRigidBody, register for the viewer. (No t.compound helper exists.)
	function crossCompound(t, w, mass, opts) {
		var G = t.AP;
		var shape = new G.CompoundShape();
		var zero = new G.Vector3(0, 0, 0), ident = new G.Quaternion(0, 0, 0, 1);
		shape.addChild(new G.BoxShape(ARM_LEN, ARM_THICK, ARM_THICK), zero, ident);   // arm along x
		shape.addChild(new G.BoxShape(ARM_THICK, ARM_THICK, ARM_LEN), zero, ident);   // arm along z
		opts = U.withMat(opts || {});
		var b = new G.RigidBody(shape, mass);
		if (opts.pos) b.position.set(opts.pos[0], opts.pos[1], opts.pos[2]);
		if (opts.friction != null) b.friction = opts.friction;
		if (opts.restitution != null) b.restitution = opts.restitution;
		if (opts.linear_damping != null) b.linear_damping = opts.linear_damping;
		if (opts.angular_damping != null) b.angular_damping = opts.angular_damping;
		w.addRigidBody(b);
		b._color = opts.color || '#f33';
		t.bodies.push(b);
		return b;
	}

	Runner.test('compound-mesh', 'compound cross settles on a mesh (no emit crash)', function (t) {
		t.log('Drop a two-box compound cross onto a mesh platform — must not throw, must rest on the mesh.');

		var w = t.makeWorld({ gravity: -9.8 });
		var platform = meshBoxPlatform(t, w, 6);
		var cross = crossCompound(t, w, 1, { pos: [0, DROP_Y, 0], color: '#f33' });

		// Metric A: it settles on top of the mesh. The arms are ARM_THICK half-height, so a cross resting
		// flat on the y=0 mesh top sits with its center at y≈ARM_THICK. settles() also guards against tunnel
		// (minY) and explosion (maxY): if the contact ever resolved wrong, the cross would fall through the
		// mesh or launch, and this fails.
		t.expect('cross comes to rest on the mesh (no tunnel/explode)',
			U.settles(cross, { hold: 20, minY: -1, maxY: 20 }));

		// Metric B: it actually rests AT the mesh surface, not below it — center near the arm's half-thickness
		// above y=0. (A pure "settles" could also pass if it tunneled and rested on nothing; this pins it to
		// the surface.) Expectations latch on first ok, so we only report ok once we're in the tail AND at the
		// surface — never before, or it would latch pass at tick 1.
		var lastTick = 0;
		t.onTick(function (world, tick) { lastTick = tick; });
		t.expect('cross rests at the mesh surface (y near arm half-thickness)', function (world) {
			var y = cross.position.y, inTail = lastTick > TOTAL - 30;
			return { ok: inTail && Math.abs(y - ARM_THICK) < 0.25, detail: 'y=' + y.toFixed(3) + ' target≈' + ARM_THICK + (inTail ? '' : ' (measuring…)') };
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'compound-mesh',
		description:
			"A two-box compound 'cross' is dropped onto a MESH platform — a pairing that has previously " +
			"exposed a compound child's collision proxy failing to resolve back to its real body in a " +
			"mesh contact. PASS: no throw, and the cross settles resting on the mesh surface (center near " +
			"the arm half-thickness above the mesh top) without tunneling through or exploding."
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
