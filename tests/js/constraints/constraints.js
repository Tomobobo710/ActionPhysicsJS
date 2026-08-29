(function (Runner) {
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "PointConstraint pins a local anchor on bodyA to a local anchor on bodyB (or a fixed " +
		"world point when bodyB is null): 3 translational DOF removed, all rotation free. Solved as " +
		"one coupled 3x3 XPBD position constraint, not three independent scalar passes.";
	function test(group, name, fn, steps) {
		Runner.test(group, name, fn, { page: 'constraints', description: DESC, visual: true, steps: steps || 0 });
	}
	var DT = 1 / 60;

	function mkWorld() {
		return new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), new AP.Solver());
	}

	function anchorWorld(body, localAnchor, out) {
		out.copy(localAnchor);
		body.rotation.transformVectorInPlace(out);
		out.addInPlace(body.position);
		return out;
	}

	function axisWorld(body, localAxis, out) {
		out.copy(localAxis);
		body.rotation.transformVectorInPlace(out);
		return out;
	}

	test('constraints/point', 'a pendulum pinned to a fixed world point never lets its anchor drift from the pin', function (t) {
		var world = mkWorld();
		var box = t.box(world, 0.2, 0.2, 0.2, 1, { pos: [0, 1, 0], vel: [2, 0, 0], color: '#4af' });
		var localAnchor = new V(0, 1, 0), pinPoint = new V(0, 2, 0);
		var pin = new AP.PointConstraint(box, null, localAnchor, pinPoint);
		world.addConstraint(pin);

		var tmp = new V();
		var maxAnchorErr = 0, maxSpeed = 0;
		t.onTick(function () {
			var a = anchorWorld(box, localAnchor, tmp);
			var err = Math.hypot(a.x - pinPoint.x, a.y - pinPoint.y, a.z - pinPoint.z);
			if (err > maxAnchorErr) maxAnchorErr = err;
			var sp = Math.hypot(box.linear_velocity.x, box.linear_velocity.y, box.linear_velocity.z);
			if (sp > maxSpeed) maxSpeed = sp;
		});
		t.expect('anchor stays pinned to the fixed point throughout the whole swing (err < 1e-6)', function () {
			return { ok: maxAnchorErr < 1e-6, detail: 'max anchor error=' + maxAnchorErr.toExponential(3) };
		});
		t.expect('speed stays bounded near the initial push - no energy injected by the joint', function () {
			return { ok: maxSpeed < 2.5, detail: 'max speed=' + maxSpeed.toFixed(4) + ' (started at 2.0)' };
		});
		t.simulate(world, 300);
	}, 300);

	test('constraints/point', 'two dynamic bodies pinned together stay pinned even when they start flying apart', function (t) {
		var world = mkWorld();
		world.gravity.set(0, 0, 0);
		var a = t.box(world, 0.2, 0.2, 0.2, 1, { pos: [-0.5, 0, 0], vel: [-3, 0, 0], color: '#4af' });
		var b = t.box(world, 0.2, 0.2, 0.2, 1, { pos: [0.5, 0, 0], vel: [3, 0, 0], color: '#f84' });
		var localA = new V(0.5, 0, 0), localB = new V(-0.5, 0, 0);
		var joint = new AP.PointConstraint(a, b, localA, localB);
		world.addConstraint(joint);

		var tmpA = new V(), tmpB = new V();
		var maxAnchorGap = 0;
		t.onTick(function () {
			var wa = anchorWorld(a, localA, tmpA), wb = anchorWorld(b, localB, tmpB);
			var gap = Math.hypot(wa.x - wb.x, wa.y - wb.y, wa.z - wb.z);
			if (gap > maxAnchorGap) maxAnchorGap = gap;
		});
		t.expect('the two anchors are held at the SAME world point despite opposite initial velocities (gap < 1e-5)', function () {
			return { ok: maxAnchorGap < 1e-5, detail: 'max anchor gap=' + maxAnchorGap.toExponential(3) };
		});
		t.expect('equal mass: the pair settles on a shared point roughly midway between their starts', function () {

			var dist = Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y, a.position.z - b.position.z);
			return { ok: dist < 1.5, detail: 'body separation=' + dist.toFixed(3) };
		});
		t.simulate(world, 120);
	}, 120);

	test('constraints/point', 'a dynamic body pinned to a STATIC body cannot drag the static one', function (t) {
		var world = mkWorld();
		var anchor = t.box(world, 0.3, 0.3, 0.3, 0, { pos: [0, 3, 0], color: '#888' });
		var box = t.box(world, 0.2, 0.2, 0.2, 1, { pos: [0, 2, 0], vel: [2, 0, 0], color: '#4af' });
		var joint = new AP.PointConstraint(box, anchor, new V(0, 1, 0), new V(0, 0, 0));

		world.addConstraint(joint);
		var anchorPos0 = { x: anchor.position.x, y: anchor.position.y, z: anchor.position.z };

		t.expect('the static body never moves, even though the joint pulls on it every substep', function () {
			var moved = Math.hypot(anchor.position.x - anchorPos0.x, anchor.position.y - anchorPos0.y, anchor.position.z - anchorPos0.z);
			return { ok: moved < 1e-9, detail: 'static body displacement=' + moved.toExponential(3) };
		});
		t.expect('the dynamic body swings on the pin (its own anchor point stays fixed to the static body)', function () {
			var tmp = new V();
			var a = anchorWorld(box, new V(0, 1, 0), tmp);
			var err = Math.hypot(a.x - anchor.position.x, a.y - anchor.position.y, a.z - anchor.position.z);
			return { ok: err < 1e-6, detail: 'anchor error=' + err.toExponential(3) };
		});
		t.simulate(world, 200);
	}, 200);

	test('constraints/hinge', 'a door hinged to a fixed world axis swings without its pivot or axis drifting', function (t) {
		var world = mkWorld();
		world.gravity.set(0, -9.81, 0);
		var door = t.box(world, 0.5, 0.5, 0.05, 1, { pos: [0, 0, 0], avel: [0, 2, 0], color: '#4af' });
		var localAxis = new V(0, 1, 0), localPivot = new V(-0.5, 0, 0);
		var hinge = new AP.HingeConstraint(door, localAxis, localPivot, null, null);
		world.addConstraint(hinge);

		var tmp = new V();
		var maxPivotErr = 0, maxAxisTilt = 0;
		t.onTick(function () {
			var p = anchorWorld(door, localPivot, tmp);
			var pivErr = Math.hypot(p.x - (-0.5), p.y - 0, p.z - 0);
			if (pivErr > maxPivotErr) maxPivotErr = pivErr;
			var a = axisWorld(door, localAxis, tmp);
			var tilt = Math.acos(Math.max(-1, Math.min(1, a.y))) * 180 / Math.PI;
			if (tilt > maxAxisTilt) maxAxisTilt = tilt;
		});
		t.expect('the pivot stays fixed at the bolted point (err < 1e-3)', function () {
			return { ok: maxPivotErr < 1e-3, detail: 'max pivot error=' + maxPivotErr.toExponential(3) };
		});
		t.expect('the hinge axis stays locked to world +Y - the door does not tilt or twist off its axis (< 1 degree)', function () {
			return { ok: maxAxisTilt < 1, detail: 'max axis tilt=' + maxAxisTilt.toFixed(4) + ' degrees' };
		});
		t.expect('the door actually swings - its far edge sweeps away from its start (this is not a no-op joint)', function () {
			var farEdge = anchorWorld(door, new V(0.5, 0, 0), tmp);
			var swept = Math.hypot(farEdge.x - 0.5, farEdge.z - 0);
			return { ok: swept > 0.3, detail: 'far-edge displacement=' + swept.toFixed(3) };
		});
		t.simulate(world, 400);
	}, 400);

	test('constraints/hinge', 'two dynamic bodies hinged together keep a shared pivot and co-axial hinge axes', function (t) {
		var world = mkWorld();
		world.gravity.set(0, -9.81, 0);
		var linkA = t.box(world, 0.5, 0.15, 0.15, 1, { pos: [-0.5, 3, 0], color: '#4af' });
		var linkB = t.box(world, 0.5, 0.15, 0.15, 1, { pos: [0.5, 3, 0], color: '#f84' });

		var axis = new V(0, 0, 1);
		var pivotOnA = new V(0.5, 0, 0), pivotOnB = new V(-0.5, 0, 0);
		var hinge = new AP.HingeConstraint(linkA, axis, pivotOnA, linkB, pivotOnB);
		world.addConstraint(hinge);

		var tmpA = new V(), tmpB = new V();
		var maxPivotGap = 0;
		t.onTick(function () {
			var wa = anchorWorld(linkA, pivotOnA, tmpA), wb = anchorWorld(linkB, pivotOnB, tmpB);
			var gap = Math.hypot(wa.x - wb.x, wa.y - wb.y, wa.z - wb.z);
			if (gap > maxPivotGap) maxPivotGap = gap;
		});
		t.expect('the shared pivot never separates, even as both links fall and swing under gravity', function () {
			return { ok: maxPivotGap < 1e-2, detail: 'max pivot gap=' + maxPivotGap.toExponential(3) };
		});
		t.expect('the chain hangs under gravity - neither link launches or flies apart', function () {
			var ok = linkA.position.y < 3.5 && linkA.position.y > -2 && linkB.position.y < 3.5 && linkB.position.y > -2;
			return { ok: ok, detail: 'linkA.y=' + linkA.position.y.toFixed(3) + ' linkB.y=' + linkB.position.y.toFixed(3) + ' (started at y=3)' };
		});
		t.simulate(world, 300);
	}, 300);

	test('constraints/weld', 'a welded body cannot move OR rotate, even when shoved and spun simultaneously', function (t) {
		var world = mkWorld();
		var box = t.box(world, 0.3, 0.3, 0.3, 1, { pos: [0, 2, 0], vel: [3, 0, 0], avel: [0, 0, 5], color: '#4af' });
		var weld = new AP.WeldConstraint(box, null, new V(0, 0, 0), new V(0, 0, 0));
		world.addConstraint(weld);

		var startPos = { x: box.position.x, y: box.position.y, z: box.position.z };
		var startRot = { x: box.rotation.x, y: box.rotation.y, z: box.rotation.z, w: box.rotation.w };
		var maxPosErr = 0, maxRotErr = 0;
		t.onTick(function () {
			var posErr = Math.hypot(box.position.x - startPos.x, box.position.y - startPos.y, box.position.z - startPos.z);
			if (posErr > maxPosErr) maxPosErr = posErr;
			var dot = Math.max(-1, Math.min(1, Math.abs(box.rotation.x * startRot.x + box.rotation.y * startRot.y + box.rotation.z * startRot.z + box.rotation.w * startRot.w)));
			var angErr = 2 * Math.acos(dot) * 180 / Math.PI;
			if (angErr > maxRotErr) maxRotErr = angErr;
		});
		t.expect('position never moves from the weld point (err < 1e-9)', function () {
			return { ok: maxPosErr < 1e-9, detail: 'max position error=' + maxPosErr.toExponential(3) };
		});
		t.expect('rotation never leaves the welded orientation, despite an initial 5 rad/s spin (err < 0.01 degrees)', function () {
			return { ok: maxRotErr < 0.01, detail: 'max rotation error=' + maxRotErr.toExponential(3) + ' degrees' };
		});
		t.simulate(world, 300);
	}, 300);

	test('constraints/weld', 'two dynamic bodies welded together rotate as ONE rigid unit despite opposite initial spins', function (t) {
		var world = mkWorld();
		var a = t.box(world, 0.3, 0.3, 0.3, 1, { pos: [-0.3, 3, 0], avel: [0, 0, 3], color: '#4af' });
		var b = t.box(world, 0.3, 0.3, 0.3, 1, { pos: [0.3, 3, 0], avel: [0, 0, -3], color: '#f84' });
		var weld = new AP.WeldConstraint(a, b, new V(0.3, 0, 0), new V(-0.3, 0, 0));
		world.addConstraint(weld);

		var tmpA = new V(), tmpB = new V();
		var maxPivotGap = 0;
		t.onTick(function () {
			var pa = anchorWorld(a, new V(0.3, 0, 0), tmpA);
			var pb = anchorWorld(b, new V(-0.3, 0, 0), tmpB);
			var gap = Math.hypot(pa.x - pb.x, pa.y - pb.y, pa.z - pb.z);
			if (gap > maxPivotGap) maxPivotGap = gap;
		});
		t.expect('the shared pivot never separates - opposite spins do not tear the weld apart (gap < 1e-5)', function () {
			return { ok: maxPivotGap < 1e-5, detail: 'max pivot gap=' + maxPivotGap.toExponential(3) };
		});
		t.expect('the pair settles to a shared rotation rate (opposite spins canceled by the rigid fuse), not still fighting each other', function () {
			var relSpin = Math.hypot(a.angular_velocity.x - b.angular_velocity.x, a.angular_velocity.y - b.angular_velocity.y, a.angular_velocity.z - b.angular_velocity.z);
			return { ok: relSpin < 0.5, detail: 'relative spin=' + relSpin.toFixed(4) + ' rad/s (they started 6 rad/s apart)' };
		});
		t.simulate(world, 200);
	}, 200);

	test('constraints/slider', 'a piston slides freely along its axis but stays locked on every other DOF', function (t) {
		var world = mkWorld();
		var box = t.box(world, 0.3, 0.3, 0.3, 1, { pos: [0, 2, 0], vel: [3, 1, 1], avel: [0, 0, 4], color: '#4af' });
		var slider = new AP.SliderConstraint(box, new V(1, 0, 0), new V(0, 0, 0), null, new V(0, 2, 0));
		world.addConstraint(slider);

		var maxYErr = 0, maxZErr = 0, maxRotErr = 0;
		t.onTick(function () {
			var yErr = Math.abs(box.position.y - 2), zErr = Math.abs(box.position.z - 0);
			if (yErr > maxYErr) maxYErr = yErr;
			if (zErr > maxZErr) maxZErr = zErr;
			var dot = Math.max(-1, Math.min(1, Math.abs(box.rotation.w)));
			var angErr = 2 * Math.acos(dot) * 180 / Math.PI;
			if (angErr > maxRotErr) maxRotErr = angErr;
		});
		t.expect('perpendicular position (Y) never drifts from the slide axis (err < 1e-6)', function () {
			return { ok: maxYErr < 1e-6, detail: 'max Y error=' + maxYErr.toExponential(3) };
		});
		t.expect('perpendicular position (Z) never drifts from the slide axis (err < 1e-6)', function () {
			return { ok: maxZErr < 1e-6, detail: 'max Z error=' + maxZErr.toExponential(3) };
		});
		t.expect('rotation stays fully locked despite an initial 4 rad/s spin (err < 0.01 degrees)', function () {
			return { ok: maxRotErr < 0.01, detail: 'max rotation error=' + maxRotErr.toExponential(3) + ' degrees' };
		});
		t.expect('the piston actually travels along its free axis - this is not a no-op weld (moved > 5 units in 200 ticks)', function () {
			return { ok: box.position.x > 5, detail: 'final x=' + box.position.x.toFixed(3) };
		});
		t.simulate(world, 200);
	}, 200);

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
