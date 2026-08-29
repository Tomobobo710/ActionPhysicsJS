// Crouch-at-speed slide: the per-tick velocity evolver for an active slide (slope acceleration,
// friction, steering). Entry/exit decisions themselves live in endStep's "MOVEMENT STATE DECISION"
// block (Movement/Step.js) — this file only advances an already-active slide by one tick.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Slide velocity EVOLVER — advances one tick of the slide's surface-tracking model (slope accel,
 * friction, steering). Pure: only called from beginStep's MOVE_SLIDE branch, which is only reached
 * when endStep has ALREADY decided this tick is a slide (see the "MOVEMENT STATE DECISION" block
 * in endStep) and has already written the correct starting tangential velocity into gb — including
 * the one-time entry boost (slideBoost), applied there rather than here so it lands on the exact
 * tick the state transition itself is observable, not one tick later. This function does not
 * decide whether to slide — it has no entry gate, no exit gate, no stored flag. It reads gb (this
 * tick's starting velocity, already tangent to groundNormal), advances it one tick, and returns
 * the result.
 *
 * @method _updateSlide
 * @private
 * @param {Object} cmd
 * @param {Number} wishX - desired horizontal velocity x from input (unsteered).
 * @param {Number} wishZ - desired horizontal velocity z from input (unsteered).
 * @param {Number} dt
 * @return {Object} result
 * @return {Number} result.vx - this tick's slide velocity, x.
 * @return {Number} result.vy - this tick's slide velocity, y (surface-follow component).
 * @return {Number} result.vz - this tick's slide velocity, z.
 */
proto._updateSlide = function(cmd, wishX, wishZ, dt) {
    // _ownVelocityX/Z, NOT gb.x/z — gb carries the platform's base velocity baked in (see the
    // constructor's _baseVelocity comment). Evolving the raw gb value would re-seed the slide's own
    // momentum with the platform's speed already added, which then compounds every tick instead of
    // properly decaying (the platform reads as if its own speed were the character's own build-up —
    // a "boost pad" while sliding on a moving platform).
    var vx = this._ownVelocityX;
    var vz = this._ownVelocityZ;
    var sp = Math.sqrt(vx * vx + vz * vz);

    var n = this.groundNormal;
    var slopeMag = Math.sqrt(n.x * n.x + n.z * n.z);
    var gy = this._gravityVec.y;
    var onSlope = slopeMag >= this.slideSlopeMin;
    // Downhill fall-line unit vector, used both by the slope-accel step below and by the reversal
    // brake's uphill test further down. Only meaningful when onSlope; 0 otherwise (unused there).
    var dx = onSlope ? n.x / slopeMag : 0;
    var dz = onSlope ? n.z / slopeMag : 0;

    if (onSlope) {
        // Gravity accelerates the fall-line (downhill) component; the cross-slope (sideways)
        // part bleeds lightly. Returned as full 3D so the grounded branch doesn't re-project it.
        var along = vx * dx + vz * dz;
        var crossX = vx - along * dx;
        var crossZ = vz - along * dz;
        // Along-slope gravitational accel is g*sin(theta) — slopeMag alone (sin of the tilt from
        // horizontal). An extra n.y (cos theta) factor here would be wrong: sin(theta)*cos(theta)
        // PEAKS at 45° and falls back off toward vertical, so a 55°+ face would decelerate barely
        // harder than a 20° one, and a near-vertical wall almost not at all — backwards from real
        // physics, where steeper always means more deceleration, up to g at 90°.
        along += -gy * slopeMag * this.slideSlopeAccel * dt;
        var cs = Math.sqrt(crossX * crossX + crossZ * crossZ);
        var cn = Math.max(0, cs - this.slideSlopeFriction * dt);
        var cf = cs > FPSC.EPS_DIR ? cn / cs : 0;
        crossX *= cf;
        crossZ *= cf;
        vx = along * dx + crossX;
        vz = along * dz + crossZ;
        sp = Math.sqrt(vx * vx + vz * vz);
    } else {
        var next = Math.max(0, sp - this.slideFriction * dt);
        var f = sp > FPSC.EPS_DIR ? next / sp : 0;
        vx *= f;
        vz *= f;
        sp = next;
    }

    // Rotate the slide heading toward input without adding speed (renormalize to sp).
    var wl = Math.sqrt(wishX * wishX + wishZ * wishZ);
    if (this.slideControl > 0 && wl > FPSC.EPS_DIR && sp > FPSC.EPS_DIR) {
        var wnx = wishX / wl, wnz = wishZ / wl;
        // Wish opposing current motion (e.g. holding backward mid-slide) is a deliberate reversal,
        // not a carve — the ordinary partial blend below would slowly rotate the heading through an
        // arc instead of braking straight back. Detect that case (wish nearly opposite current
        // velocity) and brake toward zero along the CURRENT heading instead of blending toward
        // wish; once speed has bled down, the same blend below is what picks the (now-reversed)
        // heading back up, so the reversal itself still ends up sliding in the wish direction — it
        // just brakes-then-goes instead of curving through it. Applies on flat ground too: without
        // this, flat sliding's own friction decay would bleed speed down to the slideEndSpeed exit
        // threshold WHILE the un-braked blend was arcing the heading toward wish, so a backward
        // hold curved through a U-turn on its way out instead of braking straight.
        var brakeRate = onSlope ? this.slideSlopeFriction * this.slideReversalBrakeMult
            : this.slideFriction * this.slideReversalBrakeMult;
        var vnx = vx / sp, vnz = vz / sp;
        var facing = wnx * vnx + wnz * vnz; // 1 = same direction, -1 = dead opposite
        // ANGLE-BLIND: on ANY slope, gravity always wins the fall-line — you can't carve a slide uphill
        // against it, only brake. A wish with any uphill component (against the downhill fall-line
        // dx/dz) must BRAKE toward a stop, not carve; otherwise the carve below redirects the blocked
        // uphill momentum into a cross-slope skid off the side. On flat there's no fall-line to fight,
        // so only a near-opposite wish counts as a reversal there (unchanged).
        var uphillOnSlope = onSlope && (wnx * dx + wnz * dz) < 0;
        if (uphillOnSlope || facing < FPSC.SLIDE_REVERSAL_DOT) {
            var braked = Math.max(0, sp - brakeRate * dt);
            var bf = sp > FPSC.EPS_DIR ? braked / sp : 0;
            vx *= bf;
            vz *= bf;
        } else {
            var tx = vx + (wnx * sp - vx) * this.slideControl;
            var tz = vz + (wnz * sp - vz) * this.slideControl;
            var tl = Math.sqrt(tx * tx + tz * tz) || 1;
            vx = (tx / tl) * sp;
            vz = (tz / tl) * sp;
        }
    }

    var vy = 0;
    if (onSlope) {
        var inv2 = 1 / slopeMag;
        var alongOut = vx * (n.x * inv2) + vz * (n.z * inv2);
        vy = -alongOut * slopeMag / Math.max(n.y, 0.1);
        // The velocity returned here is already tangent to the surface — including on a TOO-STEEP slope.
        // The too-steep-can't-move-up rules don't re-clip it: an active slide is exempt everywhere they
        // apply (see _collideAndSlide's climbSteepSlopes opt) — the slide IS the climb.
    }
    // Flat ground (!onSlope): groundNormal.y is ~1, so gb.y should stay ~0 — the caller (beginStep's
    // ground clamp path, same as WALK) doesn't need a nonzero vy to track a surface that's already
    // level. vy=0 here is that "no vertical correction needed" case, not a special flat-only shape.
    return { vx: vx, vy: vy, vz: vz };
};
