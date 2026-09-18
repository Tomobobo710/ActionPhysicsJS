// Vertical motion: jump + gravity/landing hook. Gravity/landing itself is left to the solver; only
// jump/jetpack thrust writes vertical velocity directly. Also the overridable gait-speed hook.
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
 * Vertical hook. Base = grounded jump only (gravity/landing handled by the solver). A jump adds the
 * platform's vertical base velocity additively, not as an overwrite.
 * @method _updateVertical
 * @protected
 * @param {Object} cmd
 * @param {Number} dt
 */
proto._updateVertical = function(cmd, dt) {
    var canJump = this.grounded || this._coyoteTimer > 0;
    var wantJump = cmd.jumpPressed || this._jumpBufferTimer > 0;
    if (canJump && wantJump) {
        // Additive (not an overwrite): jumping off a rising platform carries its vertical base velocity
        // into the jump (see _jumpKeepsVerticalBaseVelocity).
        var vBase = this._jumpKeepsVerticalBaseVelocity ? this._baseVelocity.y : 0;
        // A descending platform must not subtract from the jump — ignore negative vBase at jump time.
        // Scoped to the jump moment only; normal riding on a descending platform is unaffected.
        if (this._jumpIgnoresDescendingBaseVelocity && vBase < 0) { vBase = 0; }
        this.body.linear_velocity.y = this.jumpSpeed + vBase;
        // Horizontal carry defaults OFF. When opted out, BOTH gb.x/z and _baseVelocity.x/z must be
        // zeroed to their own-velocity values: the AIRBORNE dispatch reads gb directly, and the base
        // velocity is added into the swept move a few lines later.
        if (!this._jumpKeepsHorizontalBaseVelocity) {
            this.body.linear_velocity.x = this._ownVelocityX;
            this.body.linear_velocity.z = this._ownVelocityZ;
            this._baseVelocity.x = 0;
            this._baseVelocity.z = 0;
        }
        this.grounded = false;
        // The movement-state dispatch right after this call must see AIRBORNE now.
        this._moveState = FPSC.MOVE_AIRBORNE;
        this._groundSuppress = FPSC.GROUND_SUPPRESS_JUMP;
        // Extends ground-suppression past the fixed countdown for as long as gb.y stays positive (a
        // still-rising surface underfoot). Cleared in endStep once ordinary gravity decay ends it.
        this._jumpRising = true;
        this._coyoteTimer = 0;
        this._jumpBufferTimer = 0;
    } else if (cmd.jumpPressed) {
        this._jumpBufferTimer = this.jumpBuffer;
    }
};
