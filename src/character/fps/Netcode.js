// Entity interface (authoritative snapshots / reconciliation). beginStep/endStep (Movement/Step.js)
// are the sim; getState/setState complete the duck-typed entity contract
// {beginStep, endStep, getState, setState} an external framework can drive, and
// beginResim/endResim bracket a rollback-and-resim of already-run commands.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Reconciliation hooks (opt-in, called by the caller around a ROLLBACK-AND-RESIM of already-run
 * commands — distinct from a game "replay"). During resim the controller re-derives already-
 * perceived state, so its step/crouch snaps must NOT feed a render smoother (that double-counts
 * every step until the resim catches back up). Live ticks are unaffected.
 * @method beginResim
 */
proto.beginResim = function() { this._resimulating = true; };
/**
 * @method endResim
 */
proto.endResim = function() { this._resimulating = false; };

/**
 * Snapshot this controller's authoritative state for the network.
 * @method getState
 * @return {Object} state
 * @return {Number} state.x - body position x.
 * @return {Number} state.y - body position y.
 * @return {Number} state.z - body position z.
 * @return {Number} state.vx - body linear velocity x.
 * @return {Number} state.vy - body linear velocity y.
 * @return {Number} state.vz - body linear velocity z.
 * @return {Number} state.yaw - commanded facing yaw.
 * @return {Number} state.pitch - commanded facing pitch.
 * @return {Boolean} state.grounded
 * @return {Number} state.w - collider width.
 * @return {Number} state.h - collider height (reflects crouch).
 * @return {String} state.moveState - one of FPSC.MOVE_LADDER/MOVE_AIRBORNE/MOVE_WALK/MOVE_SLIP/MOVE_SLIDE;
 *   see the "Movement state machine" comment above endStep — serialized so resim re-adopts the exact
 *   state live prediction was in, not a re-derived guess.
 * @return {Boolean} state.sliding - plain boolean convenience view of moveState === MOVE_SLIDE, for
 *   snapshot consumers that only care about this one bit (e.g. a body model tilting while sliding).
 * @return {Number} state.gs - ground-suppress tick counter (see endStep's `suppressed`).
 * @return {Number} state.ct - coyote-time timer remaining.
 * @return {Number} state.jb - jump-buffer timer remaining.
 * @return {Number} state.gnx - ground normal x.
 * @return {Number} state.gny - ground normal y.
 * @return {Number} state.gnz - ground normal z.
 * @return {Boolean} state.climb - steep-slope walk allowance (can be granted/refused by an authority
 *   outside this controller; serialized so prediction + resim read the authoritative value).
 * @return {Boolean} state.onLadder - ladder mount state.
 * @return {Number} state.lnx - ladder face normal x (points OUT of the ladder face).
 * @return {Number} state.lnz - ladder face normal z.
 * @return {*} state.userData - opaque consumer payload, passed through unexamined.
 *
 * NB: the ghost (the body that pushes objects) is deliberately NOT serialized. It's a local
 * follow-the-character construct; setState re-derives it locally by snapping it to the
 * authoritative character. Serializing it added bandwidth for identical results.
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
 * @param {Number} s.x
 * @param {Number} s.y
 * @param {Number} s.z
 * @param {Number} s.vx
 * @param {Number} s.vy
 * @param {Number} s.vz
 * @param {Boolean} [s.grounded]
 * @param {Number} [s.h] - collider height; a mismatch vs the current height rebuilds the collider
 *   (and re-derives crouching) before position is adopted.
 * @param {String} [s.moveState]
 * @param {Number} [s.gs]
 * @param {Number} [s.ct]
 * @param {Number} [s.jb]
 * @param {Number} [s.gnx]
 * @param {Number} [s.gny]
 * @param {Number} [s.gnz]
 * @param {Boolean} [s.climb]
 * @param {Boolean} [s.onLadder]
 * @param {Number} [s.lnx]
 * @param {Number} [s.lnz]
 * @param {*} [s.userData]
 */
proto.setState = function(s) {
    // Rebuild the collider at the authoritative center/height before adopting position, so the
    // geometry matches the snapshot's before replay (a height mismatch would re-plant crouch from
    // the wrong baseline every snapshot).
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
    // _ownVelocityX/Z aren't snapshot fields — re-derive them from gb so they don't go stale (see
    // constructor comment).
    this._ownVelocityX = v.x - this._baseVelocity.x;
    this._ownVelocityZ = v.z - this._baseVelocity.z;
    if (s.grounded !== undefined) { this.grounded = s.grounded; }
    // Adopt the authoritative movement state directly — resim then starts from exactly the state
    // live prediction was in (WALK/SLIP/SLIDE/AIRBORNE/LADDER), not a locally re-derived guess.
    if (s.moveState !== undefined) { this._moveState = s.moveState; }
    if (s.gs !== undefined) { this._groundSuppress = s.gs; }
    if (s.ct !== undefined) { this._coyoteTimer = s.ct; }
    if (s.jb !== undefined) { this._jumpBufferTimer = s.jb; }
    if (s.gnx !== undefined) { this.groundNormal.set(s.gnx, s.gny, s.gnz); }
    // Adopt the authoritative steep-slope allowance. This is the ONLY place the live flag is written
    // from outside — a command only sets INTENT, an authority grants/refuses it, and the truth comes
    // back here. Read live each tick by the mover/grounding, so no rebuild is needed.
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
    // Restore gravity if mantling — _updateMantle zeroes it on entry but setState re-adopts the
    // arc mid-flight without re-running the entry code.
    if (this._mantleActive) { this.body.setGravity(0, 0, 0); }
    else { this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z); }
    // Re-baseline the ghost LOCALLY (not from the snapshot — the ghost isn't serialized). Snap it onto
    // the just-adopted authoritative character, moving at the character's velocity, so every resim starts
    // from the same consistent ghost state and re-pushes objects identically each time.
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
