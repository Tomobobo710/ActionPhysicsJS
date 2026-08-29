// View/aim/render-interpolation surface, plus the small read-only state accessors a caller polls
// every frame (sliding, moveState, bodyId, raycastIgnore). None of this touches simulation state
// except look()/setLook()/aim(), which are the caller's own facing/aim writes.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * True while a slide is active this tick. Reads the single authoritative _moveState field that
 * endStep sets — see the "Movement state machine" comment above endStep (Movement/Step.js).
 * @property sliding
 * @type {Boolean}
 * @readOnly
 */
Object.defineProperty(proto, 'sliding', { get: function() { return this._moveState === FPSC.MOVE_SLIDE; } });

/**
 * This tick's movement state: one of FPSC.MOVE_LADDER / MOVE_AIRBORNE / MOVE_WALK / MOVE_SLIP /
 * MOVE_SLIDE. Set exactly once per tick, by endStep, from a fresh ground probe — beginStep (which
 * runs BEFORE endStep, on the state endStep decided last tick) only ever READS this, never
 * re-derives it. See the "Movement state machine" comment above endStep for the full design.
 * @property moveState
 * @type {String}
 * @readOnly
 */
Object.defineProperty(proto, 'moveState', { get: function() { return this._moveState; } });

/**
 * This controller's physics-body name (the value raycasts exclude to avoid self-hits).
 * @property bodyId
 * @type {String}
 * @readOnly
 */
Object.defineProperty(proto, 'bodyId', { get: function() { return this._bodyName; } });

/**
 * The body-name list this controller's own probes ignore — pass to a game's own raycasts
 * (weapons, line-of-sight) so a shooter's cast doesn't hit itself.
 * @property raycastIgnore
 * @type {String[]}
 * @readOnly
 */
Object.defineProperty(proto, 'raycastIgnore', { get: function() { return this._ignoreSelf; } });

// ---- Look --------------------------------------------------------------

/**
 * @method look
 * @param {Number} deltaYaw
 * @param {Number} deltaPitch
 */
proto.look = function(deltaYaw, deltaPitch) {
    this.yaw += deltaYaw;
    this.pitch += deltaPitch;
    if (this.pitch > this.maxPitch) { this.pitch = this.maxPitch; }
    if (this.pitch < -this.maxPitch) { this.pitch = -this.maxPitch; }
};

/**
 * @method setLook
 * @param {Number} yaw
 * @param {Number} pitch
 */
proto.setLook = function(yaw, pitch) {
    this.yaw = yaw;
    this.pitch = pitch;
};

/**
 * Full 3D look direction (includes pitch).
 * @method getLookDirection
 * @return {Vector3}
 */
proto.getLookDirection = function() {
    var cp = Scalar.cos(this.pitch);
    return new Vector3(Scalar.sin(this.yaw) * cp, Scalar.sin(this.pitch), Scalar.cos(this.yaw) * cp);
};

/**
 * Set the LIVE, caller-owned aim — call once per render frame from your mouse-look. Render-only:
 * this NEVER enters the simulation (it doesn't touch yaw/pitch, the command, or movement), it just
 * keeps a viewmodel/camera glued to the present view instead of the 60Hz sim yaw — fixing the
 * between-tick "dangle" in every mode.
 *
 * @method aim
 * @param {Number} yaw
 * @param {Number} pitch
 */
proto.aim = function(yaw, pitch) {
    this._liveYaw = yaw;
    this._livePitch = pitch;
    this._liveAimSet = true;
};

/**
 * The live aim's full 3D direction (render-only; from aim()). Falls back to the sim look.
 * @method getLiveAimDirection
 * @return {Vector3}
 */
proto.getLiveAimDirection = function() {
    if (!this._liveAimSet) { return this.getLookDirection(); }
    var cp = Scalar.cos(this._livePitch);
    return new Vector3(Scalar.sin(this._liveYaw) * cp, Scalar.sin(this._livePitch), Scalar.cos(this._liveYaw) * cp);
};

/**
 * Horizontal forward for a given yaw (defaults to current facing).
 * @method getForwardHorizontal
 * @param {Number} [yaw]
 * @return {Vector3}
 */
proto.getForwardHorizontal = function(yaw) {
    if (yaw === undefined) { yaw = this.yaw; }
    return new Vector3(Scalar.sin(yaw), 0, Scalar.cos(yaw));
};

/**
 * Horizontal right for a given yaw (defaults to current facing). Negated to match a
 * left-handed view convention so DirRight strafes to the character's visual right.
 * @method getRightHorizontal
 * @param {Number} [yaw]
 * @return {Vector3}
 */
proto.getRightHorizontal = function(yaw) {
    if (yaw === undefined) { yaw = this.yaw; }
    return new Vector3(-Scalar.cos(yaw), 0, Scalar.sin(yaw));
};

/**
 * World-space eye position (camera goes here).
 * @method getEyePosition
 * @return {Vector3}
 */
proto.getEyePosition = function() {
    var p = this.body.position;
    return new Vector3(p.x, p.y + this.eyeHeight, p.z);
};

/**
 * Return the artificial vertical eye displacement accumulated since the last call (step/landing
 * snaps + crouch/scale swaps) and reset it. A camera folds this into a decaying offset so it
 * eases over those discontinuities. Call once per render frame. Render-only — does not affect sim.
 * @method consumeViewDisplacementY
 * @return {Number}
 */
proto.consumeViewDisplacementY = function() {
    var d = this._viewDisplacementY;
    this._viewDisplacementY = 0;
    return d;
};

/**
 * Peek at the pending vertical eye displacement WITHOUT consuming it. A render-side smoother
 * consumes (consumeViewDisplacementY); a caller that only wants to DETECT a discontinuity this
 * frame (e.g. to snap interpolation instead of sliding the eye) reads this and leaves the value
 * for the smoother. Read-only — never mutates sim or render state.
 * @method peekViewDisplacementY
 * @return {Number}
 */
proto.peekViewDisplacementY = function() {
    return this._viewDisplacementY;
};

/**
 * Stash this fixed tick's eye for sub-tick render interpolation. Call ONCE per REAL fixed step,
 * right after the step settles. A teleport-sized jump (respawn / kill-plane / hard resync) or an
 * artificial step/crouch eye snap snaps the interpolation — prev := curr — so the eye doesn't
 * smear across the discontinuity.
 *
 * @method captureRenderState
 */
proto.captureRenderState = function() {
    var e = this.getEyePosition();
    var snap = !this._currEye;
    if (this._currEye) {
        var dx = e.x - this._currEye.x, dy = e.y - this._currEye.y, dz = e.z - this._currEye.z;
        if (dx * dx + dy * dy + dz * dz > this._renderSnapDist2) { snap = true; } // teleport-sized
    }
    if (Math.abs(this.peekViewDisplacementY()) > FPSC.VIEW_DISP_SNAP) { snap = true; }
    this._prevEye = snap ? e : this._currEye;
    this._currEye = e;
};

/**
 * The render-only eye position: the last two captured fixed-tick eyes lerped by the sub-tick factor
 * `alpha` (0..1, the fraction into the current fixed step the renderer hands the draw call). Falls
 * back to the live physics eye until two ticks have been captured.
 *
 * @method renderEye
 * @param {Number} alpha
 * @return {Vector3}
 */
proto.renderEye = function(alpha) {
    if (!this._prevEye || !this._currEye) { return this.getEyePosition(); }
    var a = alpha < 0 ? 0 : alpha > 1 ? 1 : alpha;
    var ex = this._prevEye.x + (this._currEye.x - this._prevEye.x) * a;
    var ey = this._prevEye.y + (this._currEye.y - this._prevEye.y) * a;
    var ez = this._prevEye.z + (this._currEye.z - this._prevEye.z) * a;
    return new Vector3(ex, ey, ez);
};
