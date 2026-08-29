(function (Runner, PBF, AP) {
	Runner.suite('tom');

	var RADIUS = 0.4;
	var JIGGLE_AT = 40;
	var CLEAR_TICKS = 30;
	var PIVOT_AT = JIGGLE_AT + CLEAR_TICKS;
	var TOTAL = 340;

	function sphere(w, radius, mass, pos, color, mat) {
		var b = new AP.RigidBody(new AP.SphereShape(radius), mass);
		b.position.set(pos.x, pos.y, pos.z);
		b.friction = mat.friction; b.restitution = mat.restitution;
		b._color = color;
		w.addRigidBody(b);
		return b;
	}

	Runner.test('fps/entangle', 'jiggled-off sphere does not track the character\'s later perpendicular walk', function (t) {
		t.log('Settle a sphere on a standing character\'s head, jiggle it loose on X, clear the area, then pivot the character onto Z — the falling/settling sphere should never track that pivot.');

		var w = PBF.flatWorld();
		var p = PBF.spawn(w, { x: 0, y: 0.9 + 0.001, z: 0 }, {});
		var headTop = p.body.position.y + p.height / 2;
		var ball = sphere(w, RADIUS, 2, { x: 0, y: headTop + 0.2 + RADIUS, z: 0 }, '#0ff', { friction: 0.4, restitution: 0.1 });
		var startX = ball.position.x, startZ = ball.position.z;
		PBF.renderables(t, p, [ball]);

		var phase = 'idle', pt = 0;
		PBF.drive(t, p, function (tick) {
			if (tick === JIGGLE_AT && phase === 'idle') { phase = 'right'; pt = 0; }
			if (phase === 'right') { if (++pt >= 2) { phase = 'wait'; pt = 0; } return { right: 1 }; }
			else if (phase === 'wait') { if (++pt >= 1) { phase = 'left'; pt = 0; } return {}; }
			else if (phase === 'left') { if (++pt >= 2) { phase = 'clear'; } return { right: -1 }; }
			else if (phase === 'clear') { if (tick >= PIVOT_AT) { phase = 'pivot'; } return { right: 1 }; }
			else if (phase === 'pivot') { return { forward: 1 }; }
			return {};
		});

		var lastTick = 0;
		t.onTick(function (world, tick) { lastTick = tick; });

		var landedTick = -1;
		t.expect('sphere reseats within 60 ticks of the jiggle', function () {
			var top = p.body.position.y + p.height / 2;
			var bottom = ball.position.y - RADIUS;
			if (landedTick < 0 && lastTick >= JIGGLE_AT && Math.abs(bottom - top) < 0.06 && Math.abs(ball.linear_velocity.y) < 0.3) landedTick = lastTick;
			var ok = landedTick > 0 && (landedTick - JIGGLE_AT) <= 60;
			return { ok: ok, detail: 'gap=' + (bottom - top).toFixed(3) + (landedTick > 0 ? ' reseat@+' + (landedTick - JIGGLE_AT) : '') };
		});

		var groundedTick = -1;
		t.expect('sphere hits the ground within 120 ticks of the jiggle', function () {
			var floorTop = w._floor.position.y + 0.5;
			var bottom = ball.position.y - RADIUS;
			if (groundedTick < 0 && lastTick >= JIGGLE_AT && Math.abs(bottom - floorTop) < 0.06 && Math.abs(ball.linear_velocity.y) < 0.3) groundedTick = lastTick;
			var ok = groundedTick > 0 && (groundedTick - JIGGLE_AT) <= 120;
			return { ok: ok, detail: 'gap=' + (bottom - floorTop).toFixed(3) + (groundedTick > 0 ? ' grounded@+' + (groundedTick - JIGGLE_AT) : '') };
		});

		t.expect('sphere rests near where it started (horizontal dist < 2.0)', function () {
			var settled = lastTick >= JIGGLE_AT + 250
				&& Math.abs(ball.linear_velocity.x) < 0.1 && Math.abs(ball.linear_velocity.y) < 0.1 && Math.abs(ball.linear_velocity.z) < 0.1;
			if (!settled) return { ok: false, detail: 'settling…' };
			var dx = ball.position.x - startX, dz = ball.position.z - startZ;
			var dist = Math.hypot(dx, dz);
			return { ok: dist < 2.0, detail: 'dist=' + dist.toFixed(3) };
		});

		var pivotZ = null;
		t.expect('sphere Z does not track the character\'s perpendicular walk (|Δz| < 1.0)', function () {
			if (pivotZ === null && lastTick >= PIVOT_AT) pivotZ = ball.position.z;
			if (pivotZ === null) return { ok: false, detail: 'pre-pivot…' };
			var late = lastTick >= JIGGLE_AT + 250;
			var dz = Math.abs(ball.position.z - pivotZ);
			return { ok: late && dz < 1.0, detail: 'Δz=' + dz.toFixed(3) + (late ? '' : ' (measuring…)') };
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'fps/entangle',
		description:
			"A sphere rests on a standing character's head. A tap-wait-tap input jiggle on X knocks it loose, " +
			"the character walks away on that same X axis to clear the area, then PIVOTS onto the " +
			"perpendicular Z axis for the rest of the run — while the sphere is likely still mid-fall and no " +
			"longer touching the character. PASS: the sphere reseats and grounds on schedule, ends up resting " +
			"near where it started, and its Z position barely moves from its value at the pivot moment " +
			"(|Δz| < 1.0) despite the character now moving entirely along Z. The regression is the sphere's Z " +
			"tracking the character's later Z-only walk with no contact left to explain it — phantom coupling " +
			"between the two bodies."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
