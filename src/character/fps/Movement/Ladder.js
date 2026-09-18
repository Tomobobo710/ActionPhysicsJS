// Ladder climbing: a fourth movement state alongside grounded/slip/airborne/slide, resolved once per
// beginStep before the main movement dispatch runs (see Step.js).
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Ladder state transitions + climb velocity. The ladder body is never excluded from collision —
 * _collideAndSlide still runs afterward, holding the character against the face tick over tick.
 *
 * Mount requires movement intent toward the ladder (probes along the current input direction), so
 * jumping away and holding the opposite key back can't remount it a frame later.
 *
 * Forward/back and strafe contributions are summed WITHOUT normalizing the combined wish vector
 * (unlike ground movement, which clamps the wish to unit length). Holding inputs diagonally therefore
 * climbs strictly faster than either axis alone, and can exceed the clamped x/z gait speed — this is
 * intentional. Look pitch steers climb direction (the forward axis is the full pitched look direction,
 * so looking down and holding forward descends).
 *
 * Jump dismounts with a purely horizontal shove away from the face.
 *
 * @method _updateLadder
 * @private
 * @param {Object} cmd
 * @param {Number} moveYaw
 * @param {Number} movePitch
 * @param {Number} dt
 * @return {Boolean} true if this tick's velocity is fully owned by the ladder branch
 */
proto._updateLadder = function(cmd, moveYaw, movePitch, dt) {
    var gb = this.body.linear_velocity;

    var hit;
    if (this._onLadder) {
        var probeDir = new Vector3(-this._ladderNormal.x, 0, -this._ladderNormal.z);
        hit = this._findLadderAhead(probeDir);
    } else {
        var fwdH = this.getForwardHorizontal(moveYaw);
        var rgtH = this.getRightHorizontal(moveYaw);
        var cmdF0 = cmd.forward || 0;
        var cmdR0 = cmd.right || 0;
        var wishdir = new Vector3(
            fwdH.x * cmdF0 + rgtH.x * cmdR0, 0, fwdH.z * cmdF0 + rgtH.z * cmdR0
        );
        hit = this._findLadderAhead(wishdir);
    }

    if (cmd.jumpPressed && this._onLadder) {
        var n0 = this._ladderNormal;
        gb.x = n0.x * this.ladderDismountPushSpeed;
        gb.z = n0.z * this.ladderDismountPushSpeed;
        gb.y = 0;
        this._onLadder = false;
        // The dismount shove sweeps out of the ladder volume on its own; the query returns a real
        // geometric normal for an overlapping start, so no depenetration nudge is needed.
        // Next tick's dispatch must see AIRBORNE, not the pre-mount ground state.
        this._moveState = FPSC.MOVE_AIRBORNE;
        this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
        return true;
    }

    if (!hit) {
        this._onLadder = false;
        this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
        return false;
    }

    var hasMoveInput = (cmd.forward || 0) !== 0 || (cmd.right || 0) !== 0;
    if (!this._onLadder && !hasMoveInput) { return false; }

    this._onLadder = true;
    this.grounded = false;
    // Mounting owns movement now. beginStep's dispatch only reads this._moveState when NOT on a ladder,
    // so nothing else would clear a stale grounded state.
    this._moveState = FPSC.MOVE_LADDER;
    this._ladderNormal.set(hit.normal.x, 0, hit.normal.z);
    var nl = Math.sqrt(this._ladderNormal.x * this._ladderNormal.x + this._ladderNormal.z * this._ladderNormal.z);
    if (nl > FPSC.EPS_LEN) { this._ladderNormal.x /= nl; this._ladderNormal.z /= nl; }

    this.body.setGravity(0, 0, 0);
    if (!hasMoveInput) { gb.x = 0; gb.y = 0; gb.z = 0; return true; }

    var cp = Scalar.cos(movePitch);
    var fwd = new Vector3(Scalar.sin(moveYaw) * cp, Scalar.sin(movePitch), Scalar.cos(moveYaw) * cp);
    var rgt = this.getRightHorizontal(moveYaw);
    var cmdF = cmd.forward || 0;
    var cmdR = cmd.right || 0;

    // Additive, not normalized: diagonal input climbs faster (intentional — see the header).
    var velX = fwd.x * cmdF * this.ladderClimbSpeed + rgt.x * cmdR * this.ladderStrafeSpeed;
    var velY = fwd.y * cmdF * this.ladderClimbSpeed;
    var velZ = fwd.z * cmdF * this.ladderClimbSpeed + rgt.z * cmdR * this.ladderStrafeSpeed;

    var n = this._ladderNormal;
    var out = velX * n.x + velZ * n.z;
    gb.x = velX - out * n.x;
    gb.z = velZ - out * n.z;
    gb.y = velY - out;

    // Descent is blocked against solid ground here (endStep's ground clamp is skipped while mounted).
    // The ladder body itself is excluded from candidates: while mounted, the character's collider sits
    // embedded in the ladder volume, so a downward probe can report the ladder's own top-facing surface
    // as ground just below and clamp onto it (climbing the ladder like stairs).
    if (gb.y < 0) {
        var half = this.height / 2;
        var reach2 = -gb.y * dt + this._skin;
        var descentCandidates = this._probeGroundCandidates(reach2);
        var ground = null;
        for (var gi = 0; gi < descentCandidates.length; gi++) {
            if (!descentCandidates[gi].object || !descentCandidates[gi].object.isLadder) { ground = descentCandidates[gi]; break; }
        }
        if (ground) {
            var feetGap = this.body.position.y - half - ground.point.y;
            if (feetGap <= reach2) {
                var clampedY = ground.point.y + half;
                if (!this._resimulating) { this._viewDisplacementY += clampedY - this.body.position.y; }
                this.body.position.set(this.body.position.x, clampedY, this.body.position.z);
                this.body.updateDerived();
                gb.y = 0;
                this.grounded = true;
                // Still LADDER while _onLadder stays true; this only matters for the tick after dismounting.
                this._moveState = FPSC.MOVE_WALK;
                this.groundNormal.set(ground.normal.x, ground.normal.y, ground.normal.z);
            }
        }
    }
    return true;
};
