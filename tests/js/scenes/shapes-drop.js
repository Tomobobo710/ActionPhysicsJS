(function (Runner, U) {

	var SETTLE = 300;

	function drop(name, page, build, desc) {
		Runner.test('shape drops', name, function (t) {
			var w = t.makeWorld({ gravity: -9.8 });
			U.ground(t, w);
			var b = build(t, w);
			t.expect('comes to rest on the floor within ' + SETTLE + ' ticks', U.settles(b));
			t.simulate(w, SETTLE);
		}, { visual: true, steps: SETTLE, page: page, description: desc });
	}

	drop('box — flat drop', 'box',
		function (t, w) { return t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#4d89e5' }); },
		"A unit box dropped flat from y=3. PASS: it lands squarely and settles motionless. A box that " +
		"keeps sliding, jittering, or gaining height signals a bad contact/friction solve.");

	drop('box — 45° corner drop', 'box',
		function (t, w) { return t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 3, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 4), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#e5894d' }); },
		"A unit box dropped tilted 45° so it lands corner/edge-first. PASS: it tips onto a face and " +
		"comes to rest. It must not balance forever on the edge, bounce upward, or spin out.");

	drop('sphere — drop', 'sphere',
		function (t, w) { return t.sphere(w, 0.4, 1, { pos: [0, 3, 0], friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#F4D35E' }); },
		"A sphere dropped from y=3. PASS: it lands and settles without rolling away or picking up spin " +
		"from a symmetric contact.");

	drop('cylinder — upright', 'cylinder',
		function (t, w) { return t.cylinder(w, 0.4, 1, 1, { pos: [0, 3, 0], friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#FFAA00' }); },
		"A cylinder (half-height 1) dropped standing upright. PASS: it lands on its flat end and settles. " +
		"Toppling from a dead-straight drop, or never stopping, is a fail.");

	drop('cylinder — on its side', 'cylinder',
		function (t, w) { return t.cylinder(w, 0.4, 1, 1, { pos: [0, 3, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 2), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#FFAA00' }); },
		"A cylinder laid on its side (90° about Z), landing on its round barrel. PASS: it settles without " +
		"rolling forever — the case that historically wobbled/crept instead of stopping.");

	drop('capsule — upright', 'capsule',
		function (t, w) { return t.capsule(w, 0.4, 2, 1, { pos: [0, 3, 0], friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#45B7D1' }); },
		"A capsule (total height 2) dropped upright. PASS: it lands on a hemispherical cap and settles.");

	drop('capsule — on its side', 'capsule',
		function (t, w) { return t.capsule(w, 0.4, 2, 1, { pos: [0, 3, 0], rot: U.axisAngle(t, 0, 0, 1, Math.PI / 2), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#45B7D1' }); },
		"A capsule laid on its side, landing on its round barrel. PASS: it settles rather than rolling " +
		"or wobbling indefinitely.");

	drop('cone — base down', 'cone',
		function (t, w) { return t.cone(w, 0.4, 0.5, 1, { pos: [0, 3, 0], friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#a3d900' }); },
		"A cone dropped base-down (its stable rest pose). PASS: it lands flat on its base and settles.");

	drop('cone — nose down', 'cone',
		function (t, w) { return t.cone(w, 0.4, 0.5, 1, { pos: [0, 1.2, 0], rot: U.axisAngle(t, 1, 0, 0, Math.PI), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#FF8C42' }); },
		"A cone dropped nose-down from just above the floor. It lands on its tip, tips onto its slant, " +
		"and PASS: comes to rest. Never settling here is the classic cone-rolls-forever regression.");

	drop('convex — straight', 'convex',
		function (t, w) { return t.convex(w, [[0, 0.6, 0], [0, -0.6, 0], [0.6, 0, 0], [-0.6, 0, 0], [0, 0, 0.6], [0, 0, -0.6]], 1, { pos: [0, 3, 0], friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#c792ea' }); },
		"A convex hull (octahedron) dropped point-down. PASS: it tips onto a face and settles.");

	drop('convex — tilted', 'convex',
		function (t, w) { return t.convex(w, [[0, 0.6, 0], [0, -0.6, 0], [0.6, 0, 0], [-0.6, 0, 0], [0, 0, 0.6], [0, 0, -0.6]], 1, { pos: [0, 3, 0], rot: U.axisAngle(t, 1, 0.3, 0, Math.PI / 5), friction: U.MAT.friction, restitution: U.MAT.restitution, linear_damping: U.MAT.linear_damping, angular_damping: U.MAT.angular_damping, color: '#c792ea' }); },
		"The same octahedron dropped at a tilt so it lands on an edge. PASS: it rolls onto a face and " +
		"comes to rest.");
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
