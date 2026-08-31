(function (Runner, U) {

	var LEG_HALF = 0.15;
	var LEG_SPAN = 1.2;
	var DROP_Y = 2.5;
	var SHOVE_AT = 150;
	var TOTAL = 320;

	function fineMeshFloor(t, w, half, res) {
		var v = [], f = [];
		var step = (half * 2) / res;
		for (var z = 0; z <= res; z++) {
			for (var x = 0; x <= res; x++) {
				v.push([-half + x * step, 0, -half + z * step]);
			}
		}
		function idx(x, z) { return z * (res + 1) + x; }
		for (z = 0; z < res; z++) {
			for (x = 0; x < res; x++) {
				var a = idx(x, z), b = idx(x + 1, z), c = idx(x + 1, z + 1), d = idx(x, z + 1);
				f.push(a, c, b, a, d, c);
			}
		}
		return U.meshBody(t, w, v, f, 0, { pos: [0, 0, 0], color: '#556' });
	}

	function tableCompound(t, w, mass, opts) {
		var G = t.AP;
		var shape = new G.CompoundShape();
		var ident = new G.Quaternion(0, 0, 0, 1);
		var offsets = [
			[ LEG_SPAN, 0,  LEG_SPAN], [-LEG_SPAN, 0,  LEG_SPAN],
			[ LEG_SPAN, 0, -LEG_SPAN], [-LEG_SPAN, 0, -LEG_SPAN]
		];
		offsets.forEach(function (o) {
			shape.addChildShape(new G.BoxShape(LEG_HALF, LEG_HALF, LEG_HALF), new G.Vector3(o[0], o[1], o[2]), ident);
		});
		opts = U.withMat(opts || {});
		var b = new G.RigidBody(shape, mass);
		if (opts.pos) b.position.set(opts.pos[0], opts.pos[1], opts.pos[2]);
		if (opts.friction != null) b.friction = opts.friction;
		if (opts.restitution != null) b.restitution = opts.restitution;
		if (opts.linear_damping != null) b.linear_damping = opts.linear_damping;
		if (opts.angular_damping != null) b.angular_damping = opts.angular_damping;
		w.addRigidBody(b);
		b._color = opts.color || '#3af';
		t.bodies.push(b);
		return b;
	}

	Runner.test('compound-mesh', 'four-legged compound rests flat on a finely-subdivided mesh (per-child leaf cache)', function (t) {
		t.log('Drop a 4-legged compound table onto a fine mesh grid — each leg caches its own triangles independently.');

		var w = t.makeWorld({ gravity: -9.8 });
		fineMeshFloor(t, w, 6, 40);
		var table = tableCompound(t, w, 2, { pos: [0, DROP_Y, 0], color: '#3af' });

		var shoved = false, lastTick = 0;
		t.onTick(function (world, tick) {
			lastTick = tick;
			if (tick === SHOVE_AT) {
				table.linear_velocity.x += 1.5;
				shoved = true;
			}
		});

		t.expect('table settles resting flat on the mesh, twice (before and after the shove)', function (world) {
			if (lastTick < SHOVE_AT + 10) return false;
			var y = table.position.y, sv = U.speed(table), sw = U.spin(table);
			var flat = Math.abs(table.rotation.x) < 0.05 && Math.abs(table.rotation.z) < 0.05;
			var atRest = sv < 0.05 && sw < 0.05;
			var onSurface = Math.abs(y - LEG_HALF) < 0.2;
			return {
				ok: shoved && atRest && flat && onSurface && lastTick > SHOVE_AT + 40,
				detail: 'y=' + y.toFixed(3) + ' target=' + LEG_HALF + ' flat=' + flat + ' rest=' + atRest + ' shoved=' + shoved
			};
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'compound-mesh',
		description:
			"A compound body with 4 separate leg children is dropped onto a finely-subdivided mesh floor, " +
			"so each leg's footprint lands on different triangles. Exercises the per-compound-child mesh " +
			"leaf cache (each child caches its own triangle set, keyed off its own stable shape object, " +
			"not the shared body id every child would otherwise alias). Settles once (cache-hit tail), " +
			"gets shoved sideways (cache invalidation + rebuild), and must settle again flat and on the " +
			"surface — a wrong cache key would show up as sinking, tilting, or one leg's contacts leaking " +
			"into another's."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
