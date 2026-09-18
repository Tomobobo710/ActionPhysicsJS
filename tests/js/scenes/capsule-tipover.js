// Tom's Suite — CAPSULE TIPOVER. A capsule is placed upright-ish (tilted 15 degrees about X) with its
// lowest point exactly 0.1m above the floor, at the center of the world, and dropped. It must fall
// onto its side, settle, and then go to sleep.
//
// Why this is a real test and not a restatement of the drop: a capsule standing on its bottom
// hemisphere is a NEUTRAL equilibrium — the contact point sits directly under the center of mass, so
// gravity produces precisely zero torque and an upright capsule will happily stand there forever. It
// only topples because of the 15-degree start: the contact shifts to SEG_HALF*sin(15) = 5.2cm off
// the center line, and that offset is the entire lever arm. So the test is measuring the engine
// turning a small static imbalance into a full tip-over, then coming to complete rest from it.
//
// The four criteria are deliberately overlapping rather than four views of one number. Each of the
// first three can be satisfied at a DIFFERENT tick — on-side at t1, settled at t2, asleep at t3 — and
// a body can pass all three in sequence while never once satisfying them together (e.g. it rolls to
// its side, gets nudged, and only stops moving after drifting back up). The fourth criterion is the
// conjunction held for HOLD consecutive ticks, which is the claim that actually matters: it is at
// rest, asleep, ON ITS SIDE, all at the same instant. That is what distinguishes a real rest from a
// body caught mid-settle by the tick budget.
(function (Runner, U) {

	var TOTAL = 200;

	// Capsule geometry. SEG_HALF is derived exactly the way CapsuleShape derives it
	// (totalHeight/2 - radius) rather than hardcoded, so this rig cannot drift from the shape's own
	// idea of where its barrel ends and its caps begin.
	var RADIUS = 0.3;
	var TOTAL_HEIGHT = 1.0;
	var SEG_HALF = TOTAL_HEIGHT / 2 - RADIUS;    // 0.2

	var TILT = 15 * Math.PI / 180;
	var CLEARANCE = 0.1;                          // lowest point starts this far above the floor

	// Thresholds. Each is written as the tolerance it actually is, so a near-miss is legible.
	//   SIDE_TOL — |body-up . world-up| below this counts as lying on its side. A settled capsule on
	//              its barrel should read ~0; this is generous enough for a residual rock.
	//   V_TOL/W_TOL — the house settle thresholds (matching _util.js's settles()).
	//   HOLD — consecutive ticks a condition must survive before it counts, so no single frame can
	//          pass it (same convention as _util.js's settles() and sleep.js's sleepsAndParks()).
	var SIDE_TOL = 0.05;
	var V_TOL = 0.05;
	var W_TOL = 0.05;
	var HOLD = 20;

	// A launch/explode guard: a body that flies off can never legitimately settle or sleep.
	var MAX_Y = 20;

	Runner.test('capsule/tipover', 'a tilted capsule dropped 0.1m falls onto its side, settles, and sleeps', function (t) {
		t.log('A capsule is tilted 15 degrees and dropped from 0.1m of clearance. It must fall onto its side, settle, and go to sleep within ' + TOTAL + ' ticks.');

		var w = t.makeWorld({ gravity: -9.8 });
		U.ground(t, w);   // the usual static floor box, top face at y = 0

		// Drop height, derived rather than tuned: the distance from a tilted capsule's center down to
		// its lowest point is SEG_HALF*cos(tilt) + radius (the support distance in -Y), so this places
		// that lowest point exactly CLEARANCE above the floor while the body sits at x=z=0.
		var centerY = SEG_HALF * Math.cos(TILT) + RADIUS + CLEARANCE;

		var tiltQ = U.axisAngle(t, 1, 0, 0, TILT);   // plain [x,y,z,w], as t.box/t.capsule `rot` expects
		var cap = t.capsule(w, RADIUS, TOTAL_HEIGHT, 1, U.withMat({
			pos: [0, centerY, 0], rot: tiltQ, color: '#45B7D1'
		}));

		// Ask the body itself where its own up axis points in world space — the engine's answer, not a
		// re-derivation of the quaternion here. Reused buffers so this allocates nothing per tick.
		var LOCAL_UP = t.vec(0, 1, 0), WORLD_UP = t.vec(0, 0, 0);
		function upY() {
			cap.rotation.transformVectorInto(LOCAL_UP, WORLD_UP);
			return WORLD_UP.y;
		}
		function line() {
			return 'up.y=' + upY().toFixed(3) + ' y=' + cap.position.y.toFixed(3) +
				' |v|=' + U.speed(cap).toFixed(3) + ' |w|=' + U.spin(cap).toFixed(3) +
				' awake=' + cap.isAwake;
		}

		// ---- 1. it fell onto its side -----------------------------------------------------------------
		// Starts at up.y = cos(15) = 0.966 and can only reach ~0 by actually toppling, so this cannot
		// pass on the setup pose. Live up.y is reported so the viewer watches 0.97 tick down to 0.
		t.expect('capsule has fallen onto its side (|up.y| < ' + SIDE_TOL + ')', (function () {
			var run = 0;
			return function () {
				var a = Math.abs(upY());
				if (a < SIDE_TOL) run++;
				return { ok: run >= HOLD, detail: line() + ' held=' + run + '/' + HOLD };
			};
		})());

		// ---- 2. it is settled within the budget -------------------------------------------------------
		t.expect('capsule is settled by tick ' + TOTAL + ' (|v| and |w| under ' + V_TOL + ' for ' + HOLD + ' ticks)', (function () {
			var run = 0;
			return function () {
				if (U.speed(cap) < V_TOL && U.spin(cap) < W_TOL) run++; else run = 0;
				return { ok: run >= HOLD, detail: line() + ' settled=' + run + '/' + HOLD };
			};
		})());

		// ---- 3. it is asleep within the budget --------------------------------------------------------
		// Asleep implies the island manager zeroed the velocity, so both are asserted: this catches a
		// "sleep without park" bug (isAwake cleared but velocity left nonzero) the same way sleep.js does.
		t.expect('capsule is asleep by tick ' + TOTAL + ' (isAwake=false, velocity zeroed)', (function () {
			var run = 0;
			return function () {
				var parked = !cap.isAwake && U.speed(cap) === 0 && U.spin(cap) === 0;
				if (parked) run++; else run = 0;
				return { ok: run >= HOLD, detail: line() + ' parked=' + run + '/' + HOLD };
			};
		})());

		// ---- 4. asleep AND settled AND on its side, at the same time, sustained ------------------------
		// The conjunction the other three do not imply. Held, so a body merely passing through the
		// correct pose on its way somewhere else cannot satisfy it.
		t.expect('capsule is asleep AND settled AND on its side simultaneously, held ' + HOLD + ' ticks', (function () {
			var run = 0, blown = false;
			return function () {
				if (cap.position.y > MAX_Y) blown = true;
				var onSide = Math.abs(upY()) < SIDE_TOL;
				var settled = U.speed(cap) < V_TOL && U.spin(cap) < W_TOL;
				var asleep = !cap.isAwake && U.speed(cap) === 0 && U.spin(cap) === 0;
				if (blown) return { ok: false, detail: line() + ' LAUNCHED' };
				if (onSide && settled && asleep) run++; else run = 0;
				return {
					ok: run >= HOLD,
					detail: line() + ' onSide=' + onSide + ' settled=' + settled + ' asleep=' + asleep + ' held=' + run + '/' + HOLD
				};
			};
		})());

		t.simulate(w, TOTAL);
	}, {
		visual: true, steps: TOTAL, page: 'capsule',
		description:
			"A capsule (r=0.3, h=1.0) is tilted 15 degrees about X with its lowest point exactly 0.1m " +
			"above the floor, at the center of the world, and dropped. A capsule on its bottom hemisphere " +
			"is a neutral equilibrium, so the 15-degree start is the only thing that makes it topple: the " +
			"contact point sits 5.2cm off the center line and that offset is the whole lever arm. PASS: " +
			"(1) it falls onto its side, |up.y| < " + SIDE_TOL + "; (2) it is settled, |v| and |w| under " +
			V_TOL + " for " + HOLD + " ticks; (3) it is asleep — isAwake false with velocity genuinely " +
			"zeroed; and (4) all three hold AT THE SAME TIME for " + HOLD + " ticks, which the individual " +
			"criteria do not imply. A capsule that stays standing, one that never comes to rest, and one " +
			"still moving at tick " + TOTAL + " all fail, and the failure names which."
	});

})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util.js') : window.TomUtil
);
