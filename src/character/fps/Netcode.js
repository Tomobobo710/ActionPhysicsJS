// Entity interface: getState/setState complete the duck-typed contract
// {beginStep, endStep, getState, setState} an external framework drives, and beginResim/endResim
// bracket a rollback-and-resim of already-run commands.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Reconciliation hooks (opt-in), called around a rollback-and-resim. During resim the controller
 * re-derives already-perceived state, so its step/crouch snaps must not feed a render smoother.
 * @method beginResim
 */
proto.beginResim = function() { this._resimulating = true; };
/**
 * @method endResim
 */
proto.endResim = function() { this._resimulating = false; };

/**
 * Snapshot this controller's authoritative state for the network: position, velocity, facing,
 * grounded, collider size, moveState and the various timers/normals resim must re-adopt exactly.
 * The ghost is deliberately NOT serialized (setState re-derives it locally).
 * @method getState
 * @return {Object} state
 */
proto.getState = function() {
    var p = this.body.position;
    var v = this.body.linear_velocity;
    return {
        x: p.x, y: p.y, z: p.z,
        vx: v.x, vy: v.y, vz: v.z,
        yaw: this.yaw, pitch: this.pitch,
        grounded: this.grounded,
        w: this.width, h: this.height,
        moveState: this._moveState,
        sliding: this._moveState === FPSC.MOVE_SLIDE,
        gs: this._groundSuppress,
        ct: this._coyoteTimer,
        jb: this._jumpBufferTimer,
        gnx: this.groundNormal.x, gny: this.groundNormal.y, gnz: this.groundNormal.z,
        climb: this.climbSteepSlopes,
        onLadder: this._onLadder,
        lnx: this._ladderNormal.x, lnz: this._ladderNormal.z,
        mantleActive: this._mantleActive,
        mantleTimer: this._mantleTimer,
        mantleSX: this._mantleStartX, mantleSY: this._mantleStartY, mantleSZ: this._mantleStartZ,
        mantleTopY: this._mantleTopBodyY,
        mantleLX: this._mantleLandX, mantleLZ: this._mantleLandZ,
        userData: this.userData
    };
};

/**
 * Apply an authoritative state (from a snapshot). Sets position, velocity and grounded; does not
 * touch yaw/pitch. Used for reconciliation before replaying already-run commands.
 * @method setState
 * @param {Object} s - a snapshot as produced by getState.
 */
proto.setState = function(s) {
    // Rebuild the collider at the authoritative center/height first, so the geometry matches before
    // replay (a height mismatch would re-plant crouch from the wrong baseline every snapshot).
    if (s.h !== undefined && Math.abs(s.h - this.height) > FPSC.EPS_SPEED_MARGIN) {
        this.crouching = s.h < this.standHeight - FPSC.EPS_SPEED_MARGIN;
        this.height = s.h;
        this.eyeHeight = this.crouching ? this.standEye * this.crouchRatio : this.standEye;
        this._buildBody(new Vector3(s.x, s.y, s.z));
    }
    this.body.position.set(s.x, s.y, s.z);
    this.body.updateDerived();
    var v = this.body.linear_velocity;
    v.x = s.vx;
    v.y = s.vy;
    v.z = s.vz;
    this.velocityY = s.vy;
    // _ownVelocityX/Z aren't snapshot fields — re-derive them from gb so they don't go stale.
    this._ownVelocityX = v.x - this._baseVelocity.x;
    this._ownVelocityZ = v.z - this._baseVelocity.z;
    if (s.grounded !== undefined) { this.grounded = s.grounded; }
    // Adopt the authoritative movement state directly so resim starts where live prediction was.
    if (s.moveState !== undefined) { this._moveState = s.moveState; }
    if (s.gs !== undefined) { this._groundSuppress = s.gs; }
    if (s.ct !== undefined) { this._coyoteTimer = s.ct; }
    if (s.jb !== undefined) { this._jumpBufferTimer = s.jb; }
    if (s.gnx !== undefined) { this.groundNormal.set(s.gnx, s.gny, s.gnz); }
    // The authoritative steep-slope allowance: a command only sets INTENT, an authority grants/refuses.
    if (s.climb !== undefined) { this.climbSteepSlopes = s.climb; }
    if (s.onLadder !== undefined) { this._onLadder = s.onLadder; }
    if (s.lnx !== undefined) { this._ladderNormal.set(s.lnx, 0, s.lnz); }
    if (s.mantleActive !== undefined) { this._mantleActive = s.mantleActive; }
    if (s.mantleTimer !== undefined) { this._mantleTimer = s.mantleTimer; }
    if (s.mantleSX !== undefined) {
        this._mantleStartX = s.mantleSX; this._mantleStartY = s.mantleSY; this._mantleStartZ = s.mantleSZ;
    }
    if (s.mantleTopY !== undefined) { this._mantleTopBodyY = s.mantleTopY; }
    if (s.mantleLX !== undefined) { this._mantleLandX = s.mantleLX; this._mantleLandZ = s.mantleLZ; }
    // Restore gravity if mantling — _updateMantle zeroes it on entry but setState re-adopts mid-flight.
    if (this._mantleActive) { this.body.setGravity(0, 0, 0); }
    else { this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z); }
    // Re-baseline the ghost LOCALLY (not from the snapshot) so every resim starts from the same state.
    // Opt-out (hardsnapGhostOnReconcile=false): leave the ghost drifted.
    if (this._ghost && this._hardsnapGhostOnReconcile) {
        var bp = this.body.position, pv = this.body.linear_velocity;
        this._ghost.position.set(bp.x, bp.y + (this._ghostGroundInset || 0) / 2, bp.z);
        this._ghost.linear_velocity.set(pv.x, pv.y, pv.z);
        this._ghostCommandedVel = { x: pv.x, y: pv.y, z: pv.z };
    }
    this._prevCrouch = this.crouching;
    if (s.userData !== undefined) { this.userData = s.userData; }
};
