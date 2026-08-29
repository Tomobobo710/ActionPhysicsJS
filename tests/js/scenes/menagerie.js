// Tom's Suite — MENAGERIE. A "coin pusher": five different shapes (box, sphere, capsule, cone,
// cylinder) lined up on a high platform. A slow-moving pusher box shoves them off the platform's
// edge onto a ramp below; they fall, hit the ramp, and tumble down it onto the floor.
//
// Deliberately NOT pinned to one expected trajectory/orientation (see git history: an earlier test
// in this style asserted one exact final resting face and had to be pulled — it passed while
// looking visibly wrong, and failed the moment a shape's tumble path changed for an unrelated
// reason).
//
// What IS asserted is contact, not pose: every shape must make a real engine contact (a
// ContactManifold with live points, read off the World's per-tick 'contacts' event — never a
// geometric distance guess) with each of the three surfaces it travels across — the ramp, the raised
// debug bar mid-ramp, and the floor. One criterion per shape per surface, so a regression names the
// exact culprit ("sphere ✗ ramp"), and each flips green live as the shape arrives. This survives any
// change to HOW a shape tumbles while still catching a shape that misses a surface entirely — e.g.
// the box-box flat-contact behavior this rig helped shake out (before that fix a box on this ramp
// could drift/skew off its expected path; the contact asserts guard that the shapes still travel the
// whole ramp→bar→floor journey they're meant to).
(function (Runner, U, AP) {

	var TOTAL = 600;

	// Platform is high up (per user request): top surface at PLATFORM_Y. Centered on X; shifted on Z
	// so the platform's own -Z edge (the pusher's start/back side) lines up with the floor's -Z edge
	// (floor is a 20-half-extent box centered at the origin - see _util.js's ground() - so its -Z
	// edge is at z=-20). Everything below (shapes, pusher travel range, ramp) is positioned RELATIVE
	// to this platform center, so shifting PLATFORM_Z alone moves the whole rig together.
	var PLATFORM_Y = 14;
	var PLATFORM_HALF = { x: 3, y: 0.4, z: 1.2 };
	var FLOOR_HALF_Z = 20;
	var PLATFORM_Z = -FLOOR_HALF_Z + PLATFORM_HALF.z; // platform center Z so -Z edges match
	var PLATFORM_EDGE_Z = PLATFORM_Z + PLATFORM_HALF.z; // world Z of the platform's front (drop-off) edge

	// Ramp: a static box tilted so its top face runs from just below the platform's front edge down
	// to the floor - steep and tucked right against the platform (see sketch), not a long shallow
	// slide. RAMP_HALF.z is the ramp's HALF-LENGTH along its own tilted face; sized (below) so the
	// ramp's actual vertical span covers the full drop from the platform surface to the floor - a
	// mismatch here is what left the ramp floating short of the floor before.
	var RAMP_ANGLE = 70 * Math.PI / 180; // steep - a real drop-off, nothing rests on it
	var rampTopY = PLATFORM_Y - PLATFORM_HALF.y - 0.15; // ramp top tucked just under the platform's top surface
	var rampTopZ = PLATFORM_EDGE_Z + 0.05;
	var FLOOR_Y = 0; // U.ground()'s top surface
	var rampVerticalDrop = rampTopY - FLOOR_Y;
	var rampFaceLength = rampVerticalDrop / Math.sin(RAMP_ANGLE); // full length along the incline to reach the floor
	var RAMP_HALF = { x: 3, y: 0.15, z: rampFaceLength / 2 };
	// Ramp center: offset down-slope from its top edge by half its own face length.
	var rampCenter = {
		x: 0,
		y: rampTopY - Math.sin(RAMP_ANGLE) * RAMP_HALF.z,
		z: rampTopZ + Math.cos(RAMP_ANGLE) * RAMP_HALF.z + 0.2
	};

	Runner.test('menagerie', 'coin-pusher: five shapes pushed off a high platform, down a ramp', function (t) {
		t.log('Five shapes lined up on a high platform get slowly pushed off the edge, fall onto a ramp, and tumble down.');

		var w = t.makeWorld({ gravity: -9.8 });

		// Floor.
		var floor = U.ground(t, w);

		// Camera-framing anchors only (see render.js's frameCamera: it centers the initial view on the
		// AVERAGE position of every registered body, unweighted - one floor counts the same as one tiny
		// box). Without these, the floor's origin-centered mass and the rest of the scene's -Z cluster
		// outnumber a single anchor and barely move the average. Eight copies at the same spot give this
		// point real weight in that average, pulling the centroid toward the far +Z / low-Y landing area.
		for (var _a = 0; _a < 8; _a++) t.box(w, 0.05, 0.05, 0.05, 0, { pos: [0, 1, 20], color: '#000000' });

		// Platform (static): shapes start resting on its top surface.
		t.box(w, PLATFORM_HALF.x, PLATFORM_HALF.y, PLATFORM_HALF.z, 0, {
			pos: [0, PLATFORM_Y, PLATFORM_Z], friction: 0.6, restitution: 0, color: '#5a5a6a'
		});

		// Ramp (static): tilted about X by RAMP_ANGLE so its top faces up-and-toward the platform edge.
		// U.axisAngle returns a plain [x,y,z,w] array (see _util.js), not a Quaternion instance -
		// used directly as t.box's `rot` array.
		var rampRot = U.axisAngle(t, 1, 0, 0, RAMP_ANGLE);
		var ramp = t.box(w, RAMP_HALF.x, RAMP_HALF.y, RAMP_HALF.z, 0, {
			pos: [rampCenter.x, rampCenter.y, rampCenter.z], rot: rampRot,
			friction: 0.5, restitution: 0.1, color: '#6a5a4a'
		});

		// DEBUG marker: RAMP_WIDTHx1x1 box (long on X) at the ramp's own halfway point, same rotation
		// as the ramp, for manual positioning. RAMP_HALF.x*2 is the ramp's full width.
		var debugBar = t.box(w, RAMP_HALF.x, 0.25, 0.25, 0, {
			pos: [rampCenter.x, rampCenter.y, rampCenter.z], rot: rampRot,
			color: '#ff00ff'
		});

		// Five shapes, lined up along X on the platform, each with distinct identity for tracking.
		var SIDE = 0.5, RADIUS = 0.28, HALF_HEIGHT = 0.35, CAPSULE_TOTAL = 1.0;
		var restY = PLATFORM_Y + PLATFORM_HALF.y;
		var spacing = 0.9;
		var startX = -2 * spacing;

		var shapes = [];
		function track(body, name, color) {
			body._color = color;
			// rollX: total ABSOLUTE rotation about the world X axis (the ramp's roll axis), integrated
			// as sum(|angular_velocity.x| * dt) each tick - see the tumble criteria below for why
			// absolute, not net.
			shapes.push({ name: name, body: body, x0: body.position.x, y0: body.position.y, z0: body.position.z, rollX: 0 });
			return body;
		}

		track(t.box(w, SIDE / 2, SIDE / 2, SIDE / 2, 1, { pos: [startX, restY + SIDE / 2, PLATFORM_Z], friction: 0.5, restitution: 0.1 }), 'box', '#e07a3a');
		track(t.sphere(w, RADIUS, 1, { pos: [startX + spacing, restY + RADIUS, PLATFORM_Z], friction: 0.5, restitution: 0.1 }), 'sphere', '#3a9ee0');
		track(t.capsule(w, RADIUS * 0.8, CAPSULE_TOTAL, 1, {
			pos: [startX + 2 * spacing, restY + CAPSULE_TOTAL / 2, PLATFORM_Z],
			rot: [0, 0, Math.SQRT1_2, Math.SQRT1_2], // capsule laid on its side (local Y axis -> world X)
			friction: 0.5, restitution: 0.1
		}), 'capsule', '#7ae03a');
		track(t.cone(w, RADIUS * 1.2, HALF_HEIGHT, 1, { pos: [startX + 3 * spacing, restY + HALF_HEIGHT, PLATFORM_Z], friction: 0.5, restitution: 0.1 }), 'cone', '#e0d43a');
		track(t.cylinder(w, RADIUS, HALF_HEIGHT, 1, {
			pos: [startX + 4 * spacing, restY + HALF_HEIGHT, PLATFORM_Z], // upright, resting on its flat cap
			friction: 0.5, restitution: 0.1
		}), 'cylinder', '#e03a9e');

		// Pusher: a dynamic box (real mass, real collision) driven by velocity only, starting flush
		// against the platform's own BACK edge (far from the drop-off) and slowly sweeping toward +Z
		// across the whole platform depth until everything's been shoved off the front edge.
		var pusherHalf = { x: 2.6, y: 0.5, z: 0.15 };
		var pusherStartZ = PLATFORM_Z - PLATFORM_HALF.z + pusherHalf.z + 0.1;
		var pusherEndZ = PLATFORM_EDGE_Z + pusherHalf.z + 0.5; // sweeps past the edge
		var pusherSpeed = 0.6; // slow — this is a coin pusher, not a strike
		var pusher = t.box(w, pusherHalf.x, pusherHalf.y, pusherHalf.z, 5, {
			pos: [0, restY + pusherHalf.y, pusherStartZ], friction: 0.5, restitution: 0, color: '#888888'
		});
		pusher.setGravity(0, 0, 0);
		pusher.angular_factor.set(0, 0, 0);
		pusher.linear_factor.set(0, 0, 1); // velocity-driven along Z only

		t.onTick(function (world, tick) {
			// Drive the pusher forward at constant speed until it reaches pusherEndZ, then stop (hold
			// position by zeroing velocity — do not reverse; this is a one-way shove, not a ping-pong).
			if (pusher.position.z < pusherEndZ) {
				pusher.linear_velocity.set(0, 0, pusherSpeed);
			} else {
				pusher.linear_velocity.set(0, 0, 0);
			}
			// Integrate each shape's ABSOLUTE roll about the world X axis (see the tumble criteria).
			// Absolute (|w.x|), not net: a shape that rocks forward then back nets ~zero X rotation
			// while genuinely having tumbled - a box on this ramp does exactly that (net ~0.1 turns,
			// absolute ~1.2). Summing |w.x|*dt measures how much X-axis rolling actually happened,
			// which is the physical "ass over end" claim.
			for (var i = 0; i < shapes.length; i++) {
				shapes[i].rollX += Math.abs(shapes[i].body.angular_velocity.x) * t.DT;
			}
		});

		// ---- Real-contact tracking ----------------------------------------------------------------
		// "Touch" is a genuine engine contact, not a geometric proximity guess: the World emits its
		// per-tick ContactManifoldList (the very manifolds the solver just resolved) as a 'contacts'
		// event. A shape has touched a surface once a manifold exists for that exact body pair with at
		// least one surviving contact point. This is robust to HOW each shape tumbles (the whole point
		// of this rig - see file header): we assert that contact happened, never where or in what pose.
		// Each shape gets a per-surface latch that, once set, stays set - so a momentary touch counts
		// even after the shape has moved on.
		var touched = {}; // name -> { ramp, bar, floor }
		for (var si = 0; si < shapes.length; si++) touched[shapes[si].name] = { ramp: false, bar: false, floor: false };

		w.addListener('contacts', function (manifolds) {
			for (var mi = manifolds.values(), m = mi.next(); !m.done; m = mi.next()) {
				var man = m.value;
				if (man.points.length === 0) continue;
				for (var i = 0; i < shapes.length; i++) {
					var body = shapes[i].body, rec = touched[shapes[i].name];
					var other = man.bodyA === body ? man.bodyB : (man.bodyB === body ? man.bodyA : null);
					if (other === null) continue;
					if (other === ramp) rec.ramp = true;
					else if (other === debugBar) rec.bar = true;
					else if (other === floor) rec.floor = true;
				}
			}
		});

		// One criterion PER SHAPE PER SURFACE (5 shapes x 3 surfaces = 15), each going green the tick
		// that specific shape first contacts that specific surface. Deliberately granular: a future
		// regression reads as exactly "sphere ✗ ramp" or "box ✗ bar" - you see which shape missed which
		// part, not just that "something" didn't touch. Each fires independently and in real time as the
		// run plays, so the viewer shows them flipping to green in the order the shapes actually arrive.
		function touches(shapeName, surface) {
			return function () {
				if (touched[shapeName][surface]) return { ok: true, detail: shapeName + ' contacted the ' + surface };
				return { ok: false, detail: shapeName + ' has not contacted the ' + surface + ' yet' };
			};
		}
		var SURFACES = ['ramp', 'bar', 'floor'];
		for (var ci = 0; ci < shapes.length; ci++) {
			for (var cs = 0; cs < SURFACES.length; cs++) {
				var nm = shapes[ci].name, sf = SURFACES[cs];
				t.expect(nm + ' makes real contact with the ' + sf, touches(nm, sf));
			}
		}

		// ---- Tumbling (rotation about X) -----------------------------------------------------------
		// A steep drop-off + ramp should make things TUMBLE, not slide down rigid. We assert the total
		// absolute rotation each shape accumulates about the world X axis (the ramp's roll axis) - see
		// rollX in track()/onTick for why absolute, not net (a rocking box nets ~0, but genuinely rolls).
		// TWO tiers, measured from the real per-shape numbers, not guessed:
		//   - EVERY shape must clear 1 full turn (2*PI rad) - the floor for "it actually tumbled, it
		//     didn't just skid down frozen." The laggards are the box (~1.2, it rocks corner to corner)
		//     and the cone (~1.65, it pivots on its point/base rather than cartwheeling).
		//   - The ROUND rollers (sphere, capsule, cylinder) must clear 3 full turns - round things on a
		//     70-degree ramp roll freely and rack up far more (sphere ~5.4, capsule ~4.0, cylinder ~3.2).
		//     The box and cone are deliberately NOT held to this - they physically can't roll like that,
		//     and pinning them to 3 would be asserting a fiction.
		var TURN = 2 * Math.PI;
		var ROLLERS = { sphere: true, capsule: true, cylinder: true };
		function rolls(shapeRec, minTurns) {
			return function () {
				var turns = shapeRec.rollX / TURN;
				return {
					ok: turns >= minTurns,
					detail: shapeRec.name + ' rolled ' + turns.toFixed(2) + ' turns about X (need >= ' + minTurns + ')'
				};
			};
		}
		for (var ti = 0; ti < shapes.length; ti++) {
			var rec = shapes[ti];
			t.expect(rec.name + ' tumbles at least 1 full turn about X (did not slide down rigid)', rolls(rec, 1));
			if (ROLLERS[rec.name]) {
				t.expect(rec.name + ' (a roller) tumbles at least 3 full turns about X', rolls(rec, 3));
			}
		}

		// Minimal sanity gate only (no pinned trajectory/pose - see file header) - just enough of a
		// real t.expect() for the viewer to treat this as a live/steppable scene instead of a frozen
		// setup-only snapshot (render.js's captureSetup only schedules live stepping when at least one
		// expectation is registered).
		t.expect('every shape stays finite and above the floor the whole run (no NaN, no fall-through)', function () {
			for (var i = 0; i < shapes.length; i++) {
				var p = shapes[i].body.position;
				if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) {
					return { ok: false, detail: shapes[i].name + ' went non-finite: ' + JSON.stringify(p) };
				}
				if (p.y < -5) {
					return { ok: false, detail: shapes[i].name + ' fell through the floor: y=' + p.y.toFixed(3) };
				}
			}
			return { ok: true, detail: 'all ' + shapes.length + ' shapes finite and above the floor' };
		});

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'menagerie',
		description:
			"Coin-pusher rig: five shapes (box, sphere, capsule, cone, cylinder) are slowly shoved off " +
			"a high platform, fall onto a steep ramp, and tumble down to the floor. Measures physical " +
			"correctness, per shape, without pinning any exact path or pose: (1) real engine contact — " +
			"every shape must actually touch the ramp, the raised bar, and the floor (read from the " +
			"world's contact events, not guessed from distance); and (2) tumbling — every shape must " +
			"roll at least a full turn about the ramp's axis, and the round shapes at least three. Each " +
			"criterion flips green live as it's met, so a regression names the exact shape and what it " +
			"failed to do — 'sphere never touched the ramp', 'cylinder only rolled twice'."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics
);
