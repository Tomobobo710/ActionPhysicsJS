/**
 * Engine-agnostic, reusable first-person character controller built on the physics engine
 * (NOT `CharacterController` — that's a separate, spring-based capsule controller; this
 * one is a kinematic box mover with its own ground/wall/slope/ghost handling). Uses a BOX
 * collider that is angular-locked so it can never tip. Grounding, slopes, walls and resting are
 * handled by hand-written raycast/sweep probes each tick, not by the physics solver — the
 * controller does NOT hard-teleport the body to the ground every frame (that fights the solver
 * and jitters). It only:
 *   - sets HORIZONTAL velocity from input each step (snappy, no momentum fighting),
 *   - projects that velocity along the ground plane (no sliding on slopes) and off walls
 *     (smooth move-and-slide, so we never ram the solver), and
 *   - applies targeted raycast assists for STEP-UP and STEP-DOWN, which the solver can't
 *     do with a box collider.
 * Vertical motion (gravity, landing) is left to the solver; only jump / jetpack thrust write
 * the vertical velocity directly.
 *
 * Also handles two further movement states parallel to ground/air: climbing a body tagged
 * isLadder (see _updateLadder), and riding a body tagged isPlatform via base-velocity inheritance
 * (see _baseVelocity in the constructor, and beginStep/endStep/_updateVertical) — jumping off a
 * rising platform adds its velocity into the jump.
 *
 * DESIGN SEAMS:
 *   The controller never reads input directly. Gameplay samples an input command (pure data, so
 *   any caller can run remote characters' commands through the exact same path) and feeds it in,
 *   bracketing a single physics world step:
 *       const cmd = mySampleInput(input);       // input mapping is policy, lives outside this class
 *       controller.beginStep(cmd, dt);           // pre-physics: velocity + assists
 *       world.step(dt);                          // ONE world step (all bodies)
 *       controller.endStep(dt);                  // post-physics: grounded + step-down
 *
 * EXTENSIBILITY:
 *   This base IS the default "kit" (instantiate it directly). A game adds an alternate kit by
 *   subclassing and overriding `_updateVertical` (jump/gravity) and/or `_getMoveSpeed` without
 *   touching ground/step/wall logic.
 *
 * Units: METERS (gravity -9.81 by default); defaults are in meters (a ~1.8m human ≈ 1.8 units
 * tall). Use `scale` to resize the whole character.
 *
 * @class FPSCharacterController
 * @constructor
 * @param {World} world - The physics world this controller's body/ghost live in.
 * @param {Object} [options] - See FPS_CONTROLLER_DEFAULTS (FPSControllerConstants.js) for every
 *   tunable default and its meaning; each `options.X` below overrides that default per-instance.
 * @param {Vector3} [options.position] - Spawn position (body center). Default (0,20,0).
 * @param {Number} [options.scale=1] - Uniform size multiplier for the whole character.
 * @param {Number} [options.width] - Collider width (x) before scale.
 * @param {Number} [options.depth] - Collider depth (z) before scale.
 * @param {Number} [options.height] - Collider height (y) before scale.
 * @param {Number} [options.mass] - Body mass before scale.
 * @param {Number} [options.eyeHeight] - Eye offset above body CENTER before scale.
 * @param {Number} [options.walkSpeed] - Held-walk gait speed before scale (slower than run).
 * @param {Number} [options.moveSpeed] - RUN speed (the default no-modifier gait) before scale.
 * @param {Number} [options.sprintSpeed] - Sprint move speed before scale.
 * @param {Number} [options.crouchSpeedMult] - Multiplier on the active gait while crouched.
 * @param {Number} [options.sprintDecay] - Rate (units/sec) the sprint boost fades after release.
 * @param {Number} [options.groundStopDecel] - Deceleration (units/sec) on releasing all move keys.
 * @param {Number} [options.airControl] - 0..1 horizontal steering authority per step while airborne.
 * @param {Number} [options.jumpSpeed] - Jump velocity before scale.
 * @param {Number} [options.friction] - Body friction (0 keeps wall-slides clean; kinematic
 *   grounding holds slopes without relying on solver friction).
 * @param {Number} [options.stepHeight] - Max step-UP height before scale.
 * @param {Number} [options.stepDownDist] - Max step-DOWN snap before scale.
 * @param {Number} [options.coyoteTime] - Seconds after leaving a ledge you can still jump (0=off).
 * @param {Number} [options.jumpBuffer] - Seconds before landing a jump press is remembered (0=off).
 * @param {Boolean} [options.slideEnabled=true] - Enable crouch-at-speed sliding.
 * @param {Boolean} [options.slideRequiresMoveInput=true] - Require a movement key held to START a slide (exit never requires it).
 * @param {Boolean} [options.slideAllowLandingWithoutInput=true] - Waive the movement-key requirement on the landing frame, so an impact-slide can start from crouch + speed alone.
 * @param {Number} [options.slideMinSpeed] - Min along-ground speed (pre-scale) to start a slide.
 * @param {Number} [options.slideEndSpeed] - Flat slide ends below this speed (pre-scale).
 * @param {Number} [options.slideFriction] - Speed bled per second on flat ground (pre-scale).
 * @param {Number} [options.slideBoost] - Launch speed multiplier at slide entry.
 * @param {Number} [options.slideControl] - 0..1 carve authority while sliding (speed-preserving).
 * @param {Number} [options.slideSlopeAccel] - Gravity-along-slope multiplier while sliding.
 * @param {Number} [options.slideSlopeMin] - Min slope (sin of angle) that sustains a slide via gravity.
 * @param {Number} [options.slideSlopeFriction] - Cross-slope bleed per second on a sustaining slope.
 * @param {Number} [options.slideReversalBrakeMult] - Multiplier on slideSlopeFriction for how hard a
 *   deliberate on-slope reversal (wish opposing current slide direction) brakes before the carve
 *   steering picks the new heading back up.
 * @param {Boolean} [options.receivePush=true] - Enable object-to-character knockback via the ghost body.
 * @param {Number} [options.receiveMaxSpeed] - Cap on how fast a single object hit can knock the character.
 * @param {Number} [options.receiveKnockbackFraction] - Fraction of the ghost's contact velocity transferred.
 * @param {Number} [options.maxSlopeAngle] - Max standable slope in degrees (90+ disables the limit).
 * @param {Boolean} [options.visible=false] - Whether a consumer should treat the collider as drawable
 *   (this controller does no rendering itself — see `object.isVisible`).
 * @param {String} [options.color] - Cosmetic color tag, opaque to this class.
 * @param {Number} [options.skin] - Contact/sweep tolerance override (see FPSC.SKIN).
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
    // Contact/sweep tolerance. A per-instance override (not just FPSC.SKIN) lets a project tune this
    // for a specific character without touching the shared engine default.
    this._baseSkin = o.skin !== undefined ? o.skin : FPSCharacterController.FPSC.SKIN;

    // Jump-off-a-platform base-velocity behavior — see _updateVertical. Two independent axes, opposite
    // defaults: VERTICAL fling (jumping off a rising elevator flings you higher) defaults ON — it's the
    // established, expected platforming feel and existing tests (PL3) depend on it. HORIZONTAL carry
    // (jumping off a moving/rotating platform keeps its sideways speed) defaults OFF — carrying a fast
    // platform's horizontal speed into a jump (especially a spinning platform's tangential speed) reads
    // as an unwanted "fling" rather than a clean jump; a project that wants the classic
    // conveyor-belt-momentum feel can opt back in per-instance.
    this._jumpKeepsVerticalBaseVelocity = o.jumpKeepsVerticalBaseVelocity !== undefined ? o.jumpKeepsVerticalBaseVelocity !== false : true;
    this._jumpKeepsHorizontalBaseVelocity = o.jumpKeepsHorizontalBaseVelocity === true;
    // A jump is the player's WISH to leave the surface — that wish should only ever be HELPED by the
    // platform's current vertical motion, never fought. Default true (opt-out): a platform descending
    // at jump time contributes nothing negative to the launch, only a rising one still flings higher
    // (via jumpKeepsVerticalBaseVelocity above). Scoped to the jump moment only — normal ground-follow
    // on a descending platform when NOT jumping is unaffected, still correctly rides it down.
    this._jumpIgnoresDescendingBaseVelocity = o.jumpIgnoresDescendingBaseVelocity !== undefined ? o.jumpIgnoresDescendingBaseVelocity !== false : true;

    // Object interaction (push and be pushed) runs through the ghost body (see _buildGhost / _readGhostKnockback).
    this._receivePush = o.receivePush !== undefined ? o.receivePush !== false : kb.receivePush;
    // Speed-like (a velocity cap), so it must scale with character size the same way sprintSpeed
    // does — stored as a BASE here and scaled in _applyScale, not a fixed literal, so a 2x
    // character's (faster, harder-hitting) knockback is judged against a 2x cap, not the 1x default.
    this._baseReceiveMaxSpeed = o.receiveMaxSpeed !== undefined ? o.receiveMaxSpeed : kb.maxSpeed;
    this._receiveKnockbackFraction = o.receiveKnockbackFraction !== undefined ? o.receiveKnockbackFraction : kb.knockbackFraction;
    this._receiveSelfPush = o.receiveSelfPush !== undefined ? o.receiveSelfPush === true : kb.selfPush;
    // Ghost body's physics material — read once here so _buildGhost (called on every rebuild:
    // crouch, setScale, respawn) doesn't need its own access to FPS_CONTROLLER_DEFAULTS.
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

    // Max standable slope, in degrees. Stored as the cosine (_minStandableNormalY) since that's
    // what the per-tick ground-normal check compares against. 90 (or more) disables the limit.
    this.maxSlopeAngle = o.maxSlopeAngle !== undefined ? o.maxSlopeAngle : slp.maxSlopeAngle;
    this._minStandableNormalY = Scalar.cos(Math.min(90, this.maxSlopeAngle) * Math.PI / 180);
    this.climbSteepSlopes = o.climbSteepSlopes !== undefined ? o.climbSteepSlopes === true : slp.climbSteepSlopes;

    // Slide (crouch-at-speed). slide* tuning values only take effect once sliding.
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
    // Reversal brake rate, as a multiplier on slideSlopeFriction — how hard a deliberate reversal
    // (wish opposing current slide direction, see FPSC.SLIDE_REVERSAL_DOT) bleeds speed before the
    // ordinary carve blend picks the new heading back up.
    this.slideReversalBrakeMult = o.slideReversalBrakeMult !== undefined ? o.slideReversalBrakeMult : sld.reversalBrakeMult;
    // Authoritative movement state — see the "Movement state machine" comment above endStep. Starts
    // AIRBORNE; the first tick's endStep probe corrects it (e.g. to WALK if spawned on the ground).
    this._moveState = FPSCharacterController.FPSC.MOVE_AIRBORNE;
    this._slipJustEntered = false; // gates the SLIP branch's one-time velocity projection; set by endStep
    this._wantCrouch = false; // this tick's crouch intent, stashed by beginStep for endStep to read
    this._hasMoveInput = false; // this tick's movement input, stashed by beginStep for endStep to read
    this._prevCrouch = false;

    // Ladders (see _updateLadder). base* values scale with the character like every other speed.
    this._baseLadderClimbSpeed = o.ladderClimbSpeed !== undefined ? o.ladderClimbSpeed : lad.climbSpeed;
    this._baseLadderStrafeSpeed = o.ladderStrafeSpeed !== undefined ? o.ladderStrafeSpeed : lad.strafeSpeed;
    this._baseLadderMountReach = o.ladderMountReach !== undefined ? o.ladderMountReach : lad.mountReach;
    this._baseLadderDismountPushSpeed = o.ladderDismountPushSpeed !== undefined ? o.ladderDismountPushSpeed : lad.dismountPushSpeed;
    this._onLadder = false;
    this._ladderNormal = new Vector3(0, 0, 1); // points OUT of the ladder face, toward the character

    // Mantle (ledge grab + pull-up arc, see _updateMantle / Movement/Mantle.js).
    this._baseMantleHeight = o.mantleHeight !== undefined ? o.mantleHeight : man.height;
    this._baseMantleReach = o.mantleReach !== undefined ? o.mantleReach : man.reach;
    this._baseMantleSpeed = o.mantleSpeed !== undefined ? o.mantleSpeed : man.speed;
    this.mantleDuration = o.mantleDuration !== undefined ? o.mantleDuration : man.duration;
    this.mantleLiftFrac = o.mantleLiftFrac !== undefined ? o.mantleLiftFrac : man.liftFrac;
    this._mantleActive = false;
    this._mantleTimer = 0;
    // Arc anchors: body-center start (X/Y/Z), body-center Y once feet clear the ledge top, and the
    // XZ landing point past the ledge edge — all captured once at commit time (see _updateMantle's
    // detection block) so the arc interpolates position directly instead of driving velocity
    // through _collideAndSlide, which would treat the ledge face as a blocking wall.
    this._mantleStartX = 0;
    this._mantleStartY = 0;
    this._mantleStartZ = 0;
    this._mantleTopBodyY = 0;
    this._mantleLandX = 0;
    this._mantleLandZ = 0;

    // Moving platforms (see endStep's acquire + beginStep's apply). A body tagged isPlatform=true,
    // when it's what the ground probe is currently resting on, has its linear_velocity read into
    // this vector once per endStep. beginStep adds it into the horizontal move so collide-and-slide
    // carries the rider through real swept collision; it stays baked into gb.x/z afterward (position
    // integrates from gb on a LATER, separate world step, so subtracting it back out first would
    // discard the ride). _ownVelocityX/Z tracks the character's OWN horizontal velocity separately, so
    // endStep's groundStopDecel (and the sprint-decay branch) decay the character's momentum without
    // also decaying the platform's contribution. The vertical component is folded into a jump's
    // velocity ASSIGNMENT additively (not overwritten) in _updateVertical.
    this._baseVelocity = new Vector3(0, 0, 0);
    this._ownVelocityX = 0;
    this._ownVelocityZ = 0;

    var g = world.gravity || { y: -9.81 };
    this._gravityVec = new Vector3(0, g.y, 0);
    this._groundSuppress = 0;
    this._jumpRising = false; // see _updateVertical's jump branch + endStep's `suppressed`
    this._prevTopCandidateY = null; // last tick's highest ground candidate — see the slide-launch gate in endStep
    this._slideLaunched = false; // latched true the tick a slide apex launch fires; see endStep

    this._color = o.color || msc.color;
    this._visible = o.visible !== undefined ? o.visible === true : msc.visible;
    this._bodyName = o.bodyName || msc.bodyName;

    this.yaw = o.yaw !== undefined ? o.yaw : vw.yaw;
    this.pitch = o.pitch !== undefined ? o.pitch : vw.pitch;
    this.maxPitch = o.maxPitch !== undefined ? o.maxPitch : vw.maxPitch;

    // Live, render-only aim set per frame via aim(). Separate from yaw/pitch (the commanded,
    // networked, fixed-tick facing) so the view can update every frame without touching the
    // simulation. Falls back to yaw/pitch until aim() is called. See getLiveAimDirection().
    this._liveYaw = this.yaw;
    this._livePitch = this.pitch;
    this._liveAimSet = false;

    // Render interpolation: the body steps at the fixed tick but the screen draws at display
    // refresh. captureRenderState() stashes the last two fixed-tick eyes; renderEye(alpha) lerps
    // them for the draw. _renderSnapDist2 is the squared per-tick eye jump above which the
    // interpolation snaps instead of sliding (teleport/respawn).
    this._prevEye = null;
    this._currEye = null;
    // Base (scale-1) interp snap distance. The SQUARED, scale-adjusted value used at the compare site
    // is (re)derived in _applyScale — a scaled character legitimately moves the eye N× farther per tick,
    // so a fixed 1× threshold would read normal motion as a teleport and snap every tick (killing the
    // sub-tick smoothing → jitter at high scale).
    this._baseRenderSnapDist = o.renderSnapDist !== undefined ? o.renderSnapDist : rnd.snapDist;
    this._renderSnapDist2 = this._baseRenderSnapDist * this._baseRenderSnapDist;
    this._renderProxy = null;

    this.grounded = false;
    this.groundNormal = new Vector3(0, 1, 0);
    this.velocityY = 0;
    // Vertical eye displacement this controller applied via the ground-clamp/crouch/scale snaps
    // (not from velocity integration). Render-only; a camera consumes it to smooth those snaps.
    this._viewDisplacementY = 0;
    // True while the caller is resimulating already-run commands (see beginResim/endResim).
    // View-displacement is suppressed during resim so re-derived state doesn't double-count.
    this._resimulating = false;

    // Crouch is an instant collider-height swap. crouchRatio is the fraction of standing
    // height when crouched.
    this.crouchRatio = o.crouchRatio !== undefined ? o.crouchRatio : dim.crouchRatio;
    this.crouching = false;

    // Opaque consumer payload; the controller never reads inside it. Rides the same
    // command->state->snapshot path as crouch/scale.
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
    this.groundStopDecel = this._baseGroundStopDecel * scale; // idle ground stop rate (Infinity = instant hard-stop)
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
    this._groundTol = FPSCharacterController.FPSC.GROUND_TOL * scale; // how close feet must be to count as grounded
    // Terminal fall speed. Also keeps per-step fall distance < ground-probe reach so
    // the raycast ground clamp can't be tunneled through on big drops.
    this._maxFall = 22 * scale;
    // Render interp snap threshold scales with the body: a 4x character sprints ~4x faster, so its eye
    // legitimately jumps ~4x farther per tick. Without this, that normal motion trips the teleport-snap
    // and the sub-tick smoother snaps every tick instead of easing — the high-scale render jitter.
    var rs = (this._baseRenderSnapDist || 0.8) * scale;
    this._renderSnapDist2 = rs * rs;
    // Push-mass eligibility limit scales with the character, mass-like (volume, scale^3).
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

// Instantly enter/leave crouch by rebuilding the collider at the new height. Grounded: feet
// planted, top comes down. Airborne: top planted, feet rise up (crouch-jump clearance aid).
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
