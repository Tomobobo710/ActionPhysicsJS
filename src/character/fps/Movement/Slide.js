// Crouch-at-speed slide: the per-tick velocity evolver for an active slide (slope acceleration,
// friction, steering). Entry/exit decisions live in endStep's movement-state decision (Step.js).
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Slide velocity EVOLVER — advances one tick of the slide's surface-tracking model. Pure: only called
 * from beginStep's MOVE_SLIDE branch, which is only reached when endStep has ALREADY decided this tick
 * is a slide and written the starting tangential velocity (including the entry boost) into gb. It reads
 * gb, advances it one tick, and returns the result.
 *
 * @method _updateSlide
 * @private
 * @param {Object} cmd
 * @param {Number} wishX - desired horizontal velocity x from input (unsteered).
 * @param {Number} wishZ - desired horizontal velocity z from input (unsteered).
 * @param {Number} dt
 * @return {Object} result - { vx, vy, vz }
 */
proto._updateSlide = function(cmd, wishX, wishZ, dt) {
    // _ownVelocityX/Z, NOT gb.x/z — gb carries the platform's base velocity baked in, and evolving it
    // would re-seed the slide's momentum with the platform's speed every tick (a boost pad).
    var vx = this._ownVelocityX;
    var vz = this._ownVelocityZ;
    var sp = Math.sqrt(vx * vx + vz * vz);

    var n = this.groundNormal;
    var slopeMag = Math.sqrt(n.x * n.x + n.z * n.z);
    var gy = this._gravityVec.y;
    var onSlope = slopeMag >= this.slideSlopeMin;
    // Downhill fall-line unit vector, used by the slope-accel step and the reversal brake's uphill test.
    var dx = onSlope ? n.x / slopeMag : 0;
    var dz = onSlope ? n.z / slopeMag : 0;

    if (onSlope) {
        // Gravity accelerates the fall-line component; the cross-slope part bleeds lightly. Returned as
        // full 3D so the grounded branch doesn't re-project it.
        var along = vx * dx + vz * dz;
        var crossX = vx - along * dx;
        var crossZ = vz - along * dz;
        // Along-slope gravitational accel is g*sin(theta) — slopeMag alone. An extra cos(theta) factor
        // would peak at 45° and fall off toward vertical, which is backwards from real physics.
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
        // A wish opposing current motion is a deliberate reversal, not a carve: brake along the current
        // heading instead of blending toward wish (which would arc through a U-turn). The same blend then
        // picks the reversed heading back up once speed has bled. Applies on flat ground too.
        var brakeRate = onSlope ? this.slideSlopeFriction * this.slideReversalBrakeMult
            : this.slideFriction * this.slideReversalBrakeMult;
        var vnx = vx / sp, vnz = vz / sp;
        var facing = wnx * vnx + wnz * vnz; // 1 = same direction, -1 = dead opposite
        // On ANY slope, gravity wins the fall-line — any uphill wish must brake, not carve (otherwise the
        // carve redirects blocked uphill momentum into a cross-slope skid). On flat, only a near-opposite
        // wish is a reversal.
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
        // The returned velocity is already tangent to the surface, including on a too-steep slope: an
        // active slide is exempt from the too-steep-can't-move-up rules (see _collideAndSlide).
    }
    // Flat ground: groundNormal.y is ~1, so vy=0 is the "no vertical correction needed" case.
    return { vx: vx, vy: vy, vz: vz };
};
