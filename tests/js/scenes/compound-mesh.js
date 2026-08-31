(function (Runner, U) {

	var ARM_LEN = 1.5, ARM_THICK = 0.2;
	var DROP_Y = 3;
	var TOTAL = 240;

	function meshPlatform(t, w, half) {
		var v = [[-half, 0, -half], [half, 0, -half], [half, 0, half], [-half, 0, half]];
		var f = [0, 1, 2, 0, 2, 3];
		return U.meshBody(t, w, v, f, 0, { pos: [0, 0, 0], color: '#556' });
	}

	function meshBoxPlatform(t, w, half) {
		var xs = [-half, half], ys = [-half, half], zs = [-half, half], v = [];
		for (var yi = 0; yi < 2; yi++) for (var zi = 0; zi < 2; zi++) for (var xi = 0; xi < 2; xi++)
			v.push([xs[xi], ys[yi], zs[zi]]);
		var I = function (xi, yi, zi) { return yi * 4 + zi * 2 + xi; };
		var faces = [];
		function quadf(p0, p1, p2, p3) {
			faces.push(p0, p1, p2, p0, p2, p3);
		}
		quadf(I(0, 1, 0), I(0, 1, 1), I(1, 1, 1), I(1, 1, 0));
		quadf(I(0, 0, 0), I(1, 0, 0), I(1, 0, 1), I(0, 0, 1));
		quadf(I(1, 0, 0), I(1, 0, 1), I(1, 1, 1), I(1, 1, 0));
		quadf(I(0, 0, 0), I(0, 1, 0), I(0, 1, 1), I(0, 0, 1));
		quadf(I(0, 0, 1), I(1, 0, 1), I(1, 1, 1), I(0, 1, 1));
		quadf(I(0, 0, 0), I(0, 1, 0), I(1, 1, 0), I(1, 0, 0));
		return U.meshBody(t, w, v, faces, 0, { pos: [0, -half, 0], color: '#565' });
	}

	function crossCompound(t, w, mass, opts) {
		var G = t.AP;
		var shape = new G.CompoundShape();
		var zero = new G.Vector3(0, 0, 0), ident = new G.Quaternion(0, 0, 0, 1);
		shape.addChildShape(new G.BoxShape(ARM_LEN, ARM_THICK, ARM_THICK), zero, ident);
		shape.addChildShape(new G.BoxShape(ARM_THICK, ARM_THICK, ARM_LEN), zero, ident);
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

		t.expect('cross comes to rest on the mesh (no tunnel/explode)',
			U.settles(cross, { hold: 20, minY: -1, maxY: 20 }));

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
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
