// Vertical motion: jump + gravity/landing hook. Gravity/landing itself is left to the solver; only
// jump/jetpack thrust writes vertical velocity directly (see _updateVertical). Also the overridable
// gait-speed hook (_getMoveSpeed), kept here since jump/speed are the two "kit" hooks a subclass
// typically overrides together.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

// ---- Overridable kit hooks --------------------------------------------

/**
 * Gait selection: sprint > walk > run, scaled by crouch. Override to change gait rules without
 * touching ground/step/wall logic.
 * @method _getMoveSpeed
 * @protected
 * @param {Object} cmd
 * @return {Number} target horizontal speed for this tick.
 */
proto._getMoveSpeed = function(cmd) {
    // Gait priority: sprint > walk > run. Crouch scales the chosen gait.
    var gait = cmd.sprint ? this.sprintSpeed : cmd.walk ? this.walkSpeed : this.moveSpeed;
    return cmd.crouch ? gait * this.crouchSpeedMult : gait;
};

/**
 * Vertical hook. Base = grounded jump only (gravity/landing handled by the solver). A jump adds
 * platform base velocity's Y component additively, not an overwrite — jumping off a rising
 * platform flings the character higher than jumpSpeed alone would.
 * @method _updateVertical
 * @protected
 * @param {Object} cmd
 * @param {Number} dt
 */
proto._updateVertical = function(cmd, dt) {
    var canJump = this.grounded || this._coyoteTimer > 0;
    var wantJump = cmd.jumpPressed || this._jumpBufferTimer > 0;
    if (canJump && wantJump) {
        // VERTICAL: additive, not a bare overwrite — jumping off a platform that's currently rising
        // carries its vertical base velocity into the jump (a "fling"), on top of whatever base
        // velocity the character already had that tick. Gated by _jumpKeepsVerticalBaseVelocity
        // (default true — the established, expected platforming feel; PL3 depends on it).
        var vBase = this._jumpKeepsVerticalBaseVelocity ? this._baseVelocity.y : 0;
        // The player's jump is a WISH to leave the surface — that wish should only ever be helped by
        // the platform's current motion, never fought. A platform still RISING adds free height (the
        // fling above, working as intended); a platform DESCENDING must not subtract from the jump —
        // ignore negative vBase at the moment of jumping (default on; a project that wants a
        // descending platform to actively suppress a jump can opt out). This is deliberately scoped to
        // the JUMP MOMENT only, not standing/riding in general — normal ground-follow on a descending
        // platform (not jumping) is unaffected and still correctly rides it down; only the instant the
        // player presses jump does their intent take priority over the platform's own motion.
        if (this._jumpIgnoresDescendingBaseVelocity && vBase < 0) { vBase = 0; }
        this.body.linear_velocity.y = this.jumpSpeed + vBase;
        // HORIZONTAL: gated by _jumpKeepsHorizontalBaseVelocity (default FALSE — opposite default from
        // vertical). Applies to ANY platform's horizontal base velocity, linear or rotating alike —
        // left alone, a jump off a fast-moving/spinning platform launches the rider sideways at
        // whatever speed the platform was imparting, since nothing decays it once airborne. Needs BOTH
        // zeroed when opted out, not just one:
        //   - this.body.linear_velocity.x/z (= gb, a live alias set up earlier in beginStep): the
        //     AIRBORNE movement-state dispatch that runs right after this call reads gb.x/z DIRECTLY as
        //     its base velocity (`var cur = gb`) when there's no move input — zeroing only
        //     _baseVelocity below does nothing for that path, gb itself must be clean.
        //   - this._baseVelocity.x/z: also read a few lines later in the SAME beginStep call (the
        //     dispatch's own bvx/bvz, added into the swept move regardless of movement state) — leaving
        //     it non-zero re-adds the platform's speed right back even after gb is cleared above.
        if (!this._jumpKeepsHorizontalBaseVelocity) {
            this.body.linear_velocity.x = this._ownVelocityX;
            this.body.linear_velocity.z = this._ownVelocityZ;
            this._baseVelocity.x = 0;
            this._baseVelocity.z = 0;
        }
        this.grounded = false;
        // beginStep's movement-state dispatch runs right after this call, on the SAME tick — must
        // see AIRBORNE now, not whatever grounded sub-state was true a moment ago.
        this._moveState = FPSC.MOVE_AIRBORNE;
        this._groundSuppress = FPSC.GROUND_SUPPRESS_JUMP;
        // See endStep's `suppressed` — a FIXED tick count alone isn't enough here: it doesn't know
        // how far the character actually needs to climb to clear the surface they jumped off. A
        // still-rising surface underfoot (a platform still climbing, or a ramp whose OWN surface
        // keeps rising ahead of a character sprinting up it) can have gb.y still healthily positive
        // well past GROUND_SUPPRESS_JUMP's fixed window, and would otherwise get back in ground-clamp
        // snap range the instant the countdown lapses, re-catching the jump before it ever really
        // left. This flag extends suppression for as long as gb.y stays genuinely positive (checked
        // in endStep), on top of (not instead of) the fixed countdown — so a jump still can't
        // suppress forever if something keeps gb.y positive indefinitely (a runaway edge case), but a
        // normal jump's natural gravity decay is what ends it, not an arbitrary tick count picked for
        // a flat floor.
        this._jumpRising = true;
        this._coyoteTimer = 0;
        this._jumpBufferTimer = 0;
    } else if (cmd.jumpPressed) {
        this._jumpBufferTimer = this.jumpBuffer;
    }
};
