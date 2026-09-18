/**
 * Engine-agnostic first-person character controller built on a physics body, using an angular-locked
 * BOX collider (never tips). Grounding, slopes, walls and resting are handled by hand-written
 * raycast/sweep probes each tick, not the solver:
 *   - HORIZONTAL velocity is set from input each step, then projected along the ground plane and off
 *     walls (move-and-slide), and
 *   - raycast assists handle STEP-UP and STEP-DOWN, which a box collider can't do via the solver.
 * Vertical motion (gravity, landing) is left to the solver; only jump/jetpack thrust writes the
 * vertical velocity directly. Ladder climbing (_updateLadder) and moving platforms (_baseVelocity)
 * are the two extra movement states.
 *
 * DESIGN SEAMS: the controller never reads input. Gameplay samples a pure-data command and brackets a
 * single world step:
 *       const cmd = mySampleInput(input);   // input mapping is policy, outside this class
 *       controller.beginStep(cmd, dt);      // pre-physics: velocity + assists
 *       world.step(dt);                     // ONE world step (all bodies)
 *       controller.endStep(dt);             // post-physics: grounded + step-down
 *
 * EXTENSIBILITY: this base IS the default kit. A game adds a kit by subclassing and overriding
 * `_updateVertical` and/or `_getMoveSpeed` without touching ground/step/wall logic.
 *
 * Units: METERS (gravity -9.81 by default); a ~1.8m human ≈ 1.8 units tall. `scale` resizes the
 * whole character.
 *
 * @class FPSCharacterController
 * @constructor
 * @param {World} world - The physics world this controller's body/ghost live in.
 * @param {Object} [options] - Per-instance overrides of FPS_CONTROLLER_DEFAULTS
 *   (FPSControllerConstants.js), which documents every tunable.
 */
var FPSCharacterController = function(world, options) {
    this.world = world;
    var o = options || {};

    var D = FPS_CONTROLLER_DEFAULTS;
    var dim = D.dimensions, mv = D.movement, jmp = D.jump, slp = D.slopes, sld = D.slide,
        gh = D.ghost, kb = D.knockback, net = D.netcode, vw = D.view, rnd = D.render, msc = D.misc,
        lad = D.ladder, man = D.mantle;

    // Base (pre-scale) values.
    this._baseWidth = o.width !== undefined ? o.width : dim.width;
    this._baseDepth = o.depth !== undefined ? o.depth : dim.depth;
    this._baseHeight = o.height !== undefined ? o.height : dim.height;
    this._baseMass = o.mass !== undefined ? o.mass : dim.mass;
    this._baseEyeHeight = o.eyeHeight !== undefined ? o.eyeHeight : this._baseHeight * dim.eyeHeightRatio;

    this._baseWalkSpeed = o.walkSpeed !== undefined ? o.walkSpeed : mv.walkSpeed;
    this._baseMoveSpeed = o.moveSpeed !== undefined ? o.moveSpeed : mv.moveSpeed;
    this._baseSprintSpeed = o.sprintSpeed !== undefined ? o.sprintSpeed : mv.sprintSpeed;
    this.crouchSpeedMult = o.crouchSpeedMult !== undefined ? o.crouchSpeedMult : mv.crouchSpeedMult;
    this._baseSprintDecay = o.sprintDecay !== undefined ? o.sprintDecay : mv.sprintDecay;
    this._baseGroundStopDecel = o.groundStopDecel !== undefined ? o.groundStopDecel : mv.groundStopDecel;
    this._baseJumpSpeed = o.jumpSpeed !== undefined ? o.jumpSpeed : jmp.jumpSpeed;
    this._baseStepHeight = o.stepHeight !== undefined ? o.stepHeight : jmp.stepHeight;
    this._baseStepDownDist = o.stepDownDist !== undefined ? o.stepDownDist : jmp.stepDownDist;
    // Per-instance contact/sweep tolerance override.
    this._baseSkin = o.skin !== undefined ? o.skin : FPSCharacterController.FPSC.SKIN;

    // Jump-off-a-platform base-velocity behavior. Vertical fling defaults ON; horizontal carry OFF.
    this._jumpKeepsVerticalBaseVelocity = o.jumpKeepsVerticalBaseVelocity !== undefined ? o.jumpKeepsVerticalBaseVelocity !== false : true;
    this._jumpKeepsHorizontalBaseVelocity = o.jumpKeepsHorizontalBaseVelocity === true;
    // A descending platform must not subtract from a jump's launch (see _updateVertical).
    this._jumpIgnoresDescendingBaseVelocity = o.jumpIgnoresDescendingBaseVelocity !== undefined ? o.jumpIgnoresDescendingBaseVelocity !== false : true;

    // Object interaction (push and be pushed) runs through the ghost body (see Ghost.js).
    this._receivePush = o.receivePush !== undefined ? o.receivePush !== false : kb.receivePush;
    // Speed-like, so stored as a base and scaled in _applyScale like sprintSpeed.
    this._baseReceiveMaxSpeed = o.receiveMaxSpeed !== undefined ? o.receiveMaxSpeed : kb.maxSpeed;
    this._receiveKnockbackFraction = o.receiveKnockbackFraction !== undefined ? o.receiveKnockbackFraction : kb.knockbackFraction;
    this._receiveSelfPush = o.receiveSelfPush !== undefined ? o.receiveSelfPush === true : kb.selfPush;
    this._ghostMaterial = o.ghostMaterial || gh.material;
    this._driveGhostDuringResim = o.driveGhostDuringResim !== undefined ? o.driveGhostDuringResim !== false : net.driveGhostDuringResim;
    this._hardsnapGhostOnReconcile = o.hardsnapGhostOnReconcile !== undefined ? o.hardsnapGhostOnReconcile !== false : net.hardsnapGhostOnReconcile;
    this._pushMassLimitOverride = o.pushMassLimit;
    this._pushMassBaseMult = gh.pushMassBaseMult;

    this.airControl = o.airControl !== undefined ? o.airControl : mv.airControl;
    this.friction = o.friction !== undefined ? o.friction : mv.friction;

    this.coyoteTime = o.coyoteTime !== undefined ? o.coyoteTime : jmp.coyoteTime;
    this.jumpBuffer = o.jumpBuffer !== undefined ? o.jumpBuffer : jmp.jumpBuffer;
    this._coyoteTimer = 0;
    this._jumpBufferTimer = 0;

    // Max standable slope in degrees; stored as the cosine the per-tick ground check compares against.
    // 90+ disables the limit.
    this.maxSlopeAngle = o.maxSlopeAngle !== undefined ? o.maxSlopeAngle : slp.maxSlopeAngle;
    this._minStandableNormalY = Scalar.cos(Math.min(90, this.maxSlopeAngle) * Math.PI / 180);
    this.climbSteepSlopes = o.climbSteepSlopes !== undefined ? o.climbSteepSlopes === true : slp.climbSteepSlopes;

    // Slide (crouch-at-speed). slide* values only take effect once sliding.
    this.slideEnabled = o.slideEnabled !== undefined ? o.slideEnabled !== false : sld.enabled;
    this.slideRequiresMoveInput = o.slideRequiresMoveInput !== undefined ? !!o.slideRequiresMoveInput : sld.requiresMoveInput;
    this.slideAllowLandingWithoutInput = o.slideAllowLandingWithoutInput !== undefined ? !!o.slideAllowLandingWithoutInput : sld.allowLandingWithoutInput;
    this._baseSlideMinSpeed = o.slideMinSpeed !== undefined ? o.slideMinSpeed : sld.minSpeed;
    this._baseSlideEndSpeed = o.slideEndSpeed !== undefined ? o.slideEndSpeed : sld.endSpeed;
    this._baseSlideFriction = o.slideFriction !== undefined ? o.slideFriction : sld.friction;
    this.slideBoost = o.slideBoost !== undefined ? o.slideBoost : sld.boost;
    this.slideControl = o.slideControl !== undefined ? o.slideControl : sld.control;
    this.slideSlopeAccel = o.slideSlopeAccel !== undefined ? o.slideSlopeAccel : sld.slopeAccel;
    this.slideSlopeMin = o.slideSlopeMin !== undefined ? o.slideSlopeMin : sld.slopeMin;
    this._baseSlideSlopeFriction = o.slideSlopeFriction !== undefined ? o.slideSlopeFriction : sld.slopeFriction;
    // Reversal brake rate as a multiplier on slideSlopeFriction (see _updateSlide).
    this.slideReversalBrakeMult = o.slideReversalBrakeMult !== undefined ? o.slideReversalBrakeMult : sld.reversalBrakeMult;
    // Authoritative movement state, decided once per endStep. Starts AIRBORNE; the first endStep
    // corrects it.
    this._moveState = FPSCharacterController.FPSC.MOVE_AIRBORNE;
    this._slipJustEntered = false; // gates the SLIP branch's one-time velocity projection; set by endStep
    this._wantCrouch = false; // this tick's crouch intent, stashed by beginStep for endStep
    this._hasMoveInput = false; // this tick's movement input, stashed by beginStep for endStep
    this._prevCrouch = false;

    // Ladders (see _updateLadder). base* values scale with the character like every other speed.
    this._baseLadderClimbSpeed = o.ladderClimbSpeed !== undefined ? o.ladderClimbSpeed : lad.climbSpeed;
    this._baseLadderStrafeSpeed = o.ladderStrafeSpeed !== undefined ? o.ladderStrafeSpeed : lad.strafeSpeed;
    this._baseLadderMountReach = o.ladderMountReach !== undefined ? o.ladderMountReach : lad.mountReach;
    this._baseLadderDismountPushSpeed = o.ladderDismountPushSpeed !== undefined ? o.ladderDismountPushSpeed : lad.dismountPushSpeed;
    this._onLadder = false;
    this._ladderNormal = new Vector3(0, 0, 1); // points OUT of the ladder face, toward the character

    // Mantle (ledge grab + pull-up arc, see _updateMantle).
    this._baseMantleHeight = o.mantleHeight !== undefined ? o.mantleHeight : man.height;
    this._baseMantleReach = o.mantleReach !== undefined ? o.mantleReach : man.reach;
    this._baseMantleSpeed = o.mantleSpeed !== undefined ? o.mantleSpeed : man.speed;
    this.mantleDuration = o.mantleDuration !== undefined ? o.mantleDuration : man.duration;
    this.mantleLiftFrac = o.mantleLiftFrac !== undefined ? o.mantleLiftFrac : man.liftFrac;
    this._mantleActive = false;
    this._mantleTimer = 0;
    // Arc anchors captured at commit time so the arc interpolates position directly (see _updateMantle).
    this._mantleStartX = 0;
    this._mantleStartY = 0;
    this._mantleStartZ = 0;
    this._mantleTopBodyY = 0;
    this._mantleLandX = 0;
    this._mantleLandZ = 0;

    // Moving-platform base velocity, acquired each endStep and applied in the next beginStep (see
    // endStep's acquire block and beginStep's apply). _ownVelocityX/Z is the character's own horizontal
    // velocity, separate from this so decay never bleeds the platform's contribution.
    this._baseVelocity = new Vector3(0, 0, 0);
    this._ownVelocityX = 0;
    this._ownVelocityZ = 0;

    var g = world.gravity || { y: -9.81 };
    this._gravityVec = new Vector3(0, g.y, 0);
    this._groundSuppress = 0;
    this._jumpRising = false; // see _updateVertical's jump branch + endStep's `suppressed`
    this._prevTopCandidateY = null; // last tick's highest ground candidate (slide-launch gate)
    this._slideLaunched = false; // latched the tick a slide apex launch fires; see endStep

    this._color = o.color || msc.color;
    this._visible = o.visible !== undefined ? o.visible === true : msc.visible;
    this._bodyName = o.bodyName || msc.bodyName;

    this.yaw = o.yaw !== undefined ? o.yaw : vw.yaw;
    this.pitch = o.pitch !== undefined ? o.pitch : vw.pitch;
    this.maxPitch = o.maxPitch !== undefined ? o.maxPitch : vw.maxPitch;

    // Render-only aim set per frame via aim(); falls back to yaw/pitch until then.
    this._liveYaw = this.yaw;
    this._livePitch = this.pitch;
    this._liveAimSet = false;

    // Render interpolation: captureRenderState stashes the last two fixed-tick eyes; renderEye(alpha)
    // lerps them. Snap when the per-tick eye jump exceeds _renderSnapDist2 (teleport/respawn).
    this._prevEye = null;
    this._currEye = null;
    // Base (scale-1) snap distance; the squared scale-adjusted value is derived in _applyScale.
    this._baseRenderSnapDist = o.renderSnapDist !== undefined ? o.renderSnapDist : rnd.snapDist;
    this._renderSnapDist2 = this._baseRenderSnapDist * this._baseRenderSnapDist;
    this._renderProxy = null;

    this.grounded = false;
    this.groundNormal = new Vector3(0, 1, 0);
    this.velocityY = 0;
    // Render-only vertical eye displacement from the ground-clamp/crouch/scale snaps.
    this._viewDisplacementY = 0;
    // True while resimulating already-run commands (beginResim/endResim); suppresses view displacement.
    this._resimulating = false;

    // Crouch is an instant collider-height swap; crouchRatio is the crouched fraction of standing height.
    this.crouchRatio = o.crouchRatio !== undefined ? o.crouchRatio : dim.crouchRatio;
    this.crouching = false;

    // Opaque consumer payload; rides the command->state->snapshot path, never read here.
    this.userData = null;

    this.scale = 1;
    var spawnOpt = o.position;
    var spawn = spawnOpt ? new Vector3(spawnOpt.x, spawnOpt.y, spawnOpt.z)
        : new Vector3(msc.spawn.x, msc.spawn.y, msc.spawn.z);
    this._applyScale(o.scale !== undefined ? o.scale : msc.scale);
    this._buildBody(spawn);
};

var proto = FPSCharacterController.prototype;

// Resolve scaled dimensions/speeds from the base values.
proto._applyScale = function(scale) {
    this.scale = scale;
    this.width = this._baseWidth * scale;
    this.depth = this._baseDepth * scale;
    // Standing dimensions, then the active height/eye reflect the crouch state.
    this.standHeight = this._baseHeight * scale;
    this.standEye = this._baseEyeHeight * scale;
    this.height = this.crouching ? this.standHeight * this.crouchRatio : this.standHeight;
    this.eyeHeight = this.crouching ? this.standEye * this.crouchRatio : this.standEye;
    this.mass = this._baseMass * scale * scale * scale; // volume scaling
    this.walkSpeed = this._baseWalkSpeed * scale;
    this.moveSpeed = this._baseMoveSpeed * scale;
    this.sprintSpeed = this._baseSprintSpeed * scale;
    this.sprintDecay = this._baseSprintDecay * scale; // excess-speed bleed rate (Infinity = instant)
    this.groundStopDecel = this._baseGroundStopDecel * scale; // idle ground stop rate (Infinity = instant)
    this.slideMinSpeed = this._baseSlideMinSpeed * scale;
    this.slideEndSpeed = this._baseSlideEndSpeed * scale;
    this.slideFriction = this._baseSlideFriction * scale;
    this.slideSlopeFriction = this._baseSlideSlopeFriction * scale;
    this.jumpSpeed = this._baseJumpSpeed * Math.sqrt(scale); // jump height scales ~linearly
    this.stepHeight = this._baseStepHeight * scale;
    this.stepDownDist = this._baseStepDownDist * scale;
    this.ladderClimbSpeed = this._baseLadderClimbSpeed * scale;
    this.ladderStrafeSpeed = this._baseLadderStrafeSpeed * scale;
    this.ladderMountReach = this._baseLadderMountReach * scale;
    this.ladderDismountPushSpeed = this._baseLadderDismountPushSpeed * scale;
    this.mantleHeight = this._baseMantleHeight * scale;
    this.mantleReach = this._baseMantleReach * scale;
    this.mantleSpeed = this._baseMantleSpeed * scale;
    this._skin = this._baseSkin * scale; // contact tolerance
    this._groundTol = FPSCharacterController.FPSC.GROUND_TOL * scale; // feet distance to count as grounded
    // Terminal fall speed, kept under the ground-probe reach so big drops can't tunnel.
    this._maxFall = 22 * scale;
    // Snap threshold scales with the body so normal high-speed motion doesn't trip the teleport-snap.
    var rs = (this._baseRenderSnapDist || 0.8) * scale;
    this._renderSnapDist2 = rs * rs;
    // Push-mass eligibility limit scales mass-like (volume, scale^3).
    this._pushMassLimit = this._pushMassLimitOverride !== undefined ?
        this._pushMassLimitOverride : this._baseMass * scale * scale * scale * this._pushMassBaseMult;
    this._receiveMaxSpeed = this._baseReceiveMaxSpeed * scale;
};

/**
 * Resize the whole character at runtime (rebuilds the collider, feet planted).
 * @method setScale
 * @param {Number} scale
 */
proto.setScale = function(scale) {
    var p = this.body.position;
    var eyeBefore = p.y + this.eyeHeight;
    var feetY = p.y - this.height / 2;
    this._applyScale(scale);
    this._buildBody(new Vector3(p.x, feetY + this.height / 2, p.z));
    if (!this._resimulating) { this._viewDisplacementY += this.body.position.y + this.eyeHeight - eyeBefore; } // eye jump from the resize
};

// Instantly enter/leave crouch by rebuilding the collider at the new height. Grounded: feet planted,
// top comes down. Airborne: top planted, feet rise (crouch-jump clearance aid).
proto._setCrouch = function(want) {
    if (want === this.crouching) { return; }
    var p = this.body.position;
    var eyeBefore = p.y + this.eyeHeight;
    var feetY = p.y - this.height / 2;
    var headY = p.y + this.height / 2;
    this.crouching = want;
    this._applyScale(this.scale); // recompute height/eye for the new crouch state
    var newCenterY = this.grounded ? (feetY + this.height / 2) : (headY - this.height / 2);
    this._buildBody(new Vector3(p.x, newCenterY, p.z));
    if (!this._resimulating) { this._viewDisplacementY += this.body.position.y + this.eyeHeight - eyeBefore; } // eye jump from the crouch swap
};

/**
 * Add a velocity impulse and force the character airborne (explosions / knockback / rocket-jumping).
 * @method applyKnockback
 */
proto.applyKnockback = function(vx, vy, vz) {
    var gb = this.body.linear_velocity;
    gb.x += vx;
    gb.y += vy;
    gb.z += vz;
    this.grounded = false;
    this._moveState = FPSCharacterController.FPSC.MOVE_AIRBORNE;
    this._groundSuppress = 10;
    this.velocityY = gb.y;
};

// ---- Lifecycle ---------------------------------------------------------

/**
 * @method setPosition
 * @param {Vector3} pos
 */
proto.setPosition = function(pos) {
    this.body.position.set(pos.x, pos.y, pos.z);
    this.body.updateDerived();
    this.body.linear_velocity.set(0, 0, 0);
    this.grounded = false;
    this._moveState = FPSCharacterController.FPSC.MOVE_AIRBORNE;
    if (this._ghost) {
        var inset = this._ghostGroundInset || 0;
        this._ghost.position.set(pos.x, pos.y + inset / 2, pos.z);
        this._ghost.linear_velocity.set(0, 0, 0);
    }
};

/**
 * @method destroy
 */
proto.destroy = function() {
    this.world.removeRigidBody(this.body);
    this._destroyGhost();
};

ActionPhysics.FPSCharacterController = FPSCharacterController;
