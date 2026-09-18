// Movement state machine core: beginStep (pre-physics velocity + assists) and endStep (post-physics
// grounding + state decision) bracket a single physics world step.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * PRE-physics: set this tick's horizontal velocity (slope/wall projected) + assists.
 *
 * The movement basis comes from the COMMAND's yaw (cmd.yaw), not any persistent "live aim", so
 * replaying commands during reconciliation can't drag the view backward. Commanded yaw/pitch are
 * recorded as this entity's facing only when the command carries them.
 *
 * Platform base velocity is added immediately before collide-and-slide, so a rider is carried through
 * real swept motion rather than a position teleport.
 *
 * @method beginStep
 * @param {Object} command - pure-data input command struct; any field may be absent
 * @param {Number} dt
 */
proto.beginStep = function(command, dt) {
    var cmd = command || {};

    if (this._jumpBufferTimer > 0) { this._jumpBufferTimer = Math.max(0, this._jumpBufferTimer - dt); }

    if (cmd.scale !== undefined && Math.abs(cmd.scale - this.scale) > FPSC.EPS_LEN) { this.setScale(cmd.scale); }
    // Steep-slope walk intent from the command; an authority can overrule it later via setState.
    if (cmd.climb !== undefined) { this.climbSteepSlopes = !!cmd.climb; }
    var wantCrouch = !!cmd.crouch || (this.crouching && !this._canStand());
    if (wantCrouch !== this.crouching) { this._setCrouch(wantCrouch); }

    if (cmd.userData !== undefined) { this.userData = cmd.userData; }

    var gb = this.body.linear_velocity;

    this._prevY = this.body.position.y;

    var moveYaw = cmd.yaw !== undefined ? cmd.yaw : this.yaw;
    var movePitch = cmd.pitch !== undefined ? cmd.pitch : this.pitch;
    if (cmd.yaw !== undefined) { this.yaw = cmd.yaw; }
    if (cmd.pitch !== undefined) { this.pitch = cmd.pitch; }

    var fwd = this.getForwardHorizontal(moveYaw);
    var rgt = this.getRightHorizontal(moveYaw);
    var cmdF = cmd.forward || 0;
    var cmdR = cmd.right || 0;
    var dirX = fwd.x * cmdF + rgt.x * cmdR;
    var dirZ = fwd.z * cmdF + rgt.z * cmdR;
    var dirLen = Math.sqrt(dirX * dirX + dirZ * dirZ);
    var hasInput = dirLen > FPSC.EPS_DIR;
    this._cmdIdle = !hasInput;
    var speed = this._getMoveSpeed(cmd);
    var wishX = 0;
    var wishZ = 0;
    if (hasInput) {
        // Clamp to unit length, not normalize: a digital diagonal still caps at gait speed, but a
        // partial analog stick keeps its magnitude for a proportional walk.
        var norm = dirLen > 1 ? 1 / dirLen : 1;
        wishX = dirX * norm * speed;
        wishZ = dirZ * norm * speed;
    }

    // Stashed for endStep (after world.step) to use when deciding this tick's movement state.
    this._wantCrouch = wantCrouch;
    this._hasMoveInput = hasInput;

    var onMantleThisTick = this._updateMantle(cmd, moveYaw, dt);

    // _updateLadder mounts/dismounts and, while mounted, owns velocity fully — checked first since it
    // can override every other state this tick.
    var onLadderThisTick = !onMantleThisTick && this._updateLadder(cmd, moveYaw, movePitch, dt);

    var vx, vz;
    if (onLadderThisTick || onMantleThisTick) {
        // LADDER / MANTLE: the hook already wrote gb.x/y/z; velocity is fully its.
        vx = gb.x;
        vz = gb.z;
    } else {
        // A jump flips grounded→airborne HERE, before the dispatch below reads this._moveState.
        this._updateVertical(cmd, dt);

        // ================================================================================
        // MOVEMENT STATE DISPATCH — reads this._moveState, set authoritatively by LAST tick's
        // endStep (or by _updateVertical just above on a jump). Each branch is a self-contained
        // velocity model for one state.
        // ================================================================================
        if (this._moveState === FPSC.MOVE_SLIDE && this.grounded) {
            // SLIDE, GROUNDED: _updateSlide is a pure per-tick evolver (endStep already decided this
            // tick IS a slide); it advances the slide's velocity one tick from gb.
            var slideResult = this._updateSlide(cmd, wishX, wishZ, dt);
            vx = slideResult.vx;
            vz = slideResult.vz;
            gb.y = slideResult.vy;
        } else if (this._moveState === FPSC.MOVE_SLIDE && !this.grounded) {
            // SLIDE, AIRBORNE: carried ballistically until it lands or slows below slideEndSpeed.
            this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
            if (gb.y < -this._maxFall) { gb.y = -this._maxFall; }
            vx = gb.x;
            vz = gb.z;
        } else if (this._moveState === FPSC.MOVE_SLIP) {
            // SLIP: too-steep surface, gravity-fed, weak air-control.
            this.body.setGravity(0, 0, 0);
            var n = this.groundNormal;
            var slopeMag = Math.sqrt(n.x * n.x + n.z * n.z);
            var dxu = slopeMag > FPSC.EPS_LEN ? n.x / slopeMag : 0;
            var dzu = slopeMag > FPSC.EPS_LEN ? n.z / slopeMag : 0;
            var g = -this._gravityVec.y;
            // Project the incoming 3D velocity onto the plane ONLY on the tick contact is new
            // (_slipJustEntered). Every later slip tick has gb.y already zeroed, so gb.x/gb.z are the
            // correctly-accumulated tangential speed — re-projecting would fight that accumulation.
            var gbx = gb.x, gbz = gb.z;
            if (this._slipJustEntered) {
                var dot0 = gb.x * n.x + gb.y * n.y + gb.z * n.z;
                gbx = gb.x - dot0 * n.x;
                gbz = gb.z - dot0 * n.z;
                this._slipJustEntered = false;
            }
            vx = gbx + dxu * g * slopeMag * dt;
            vz = gbz + dzu * g * slopeMag * dt;
            if (hasInput) {
                var twx = wishX, twz = wishZ;
                var up = -(twx * dxu + twz * dzu);
                if (up > 0) { twx += dxu * up; twz += dzu * up; }
                vx += (twx - vx) * this.airControl;
                vz += (twz - vz) * this.airControl;
                var along2 = vx * dxu + vz * dzu;
                if (along2 < 0) { vx -= dxu * along2; vz -= dzu * along2; }
            }
            var alongOut = vx * dxu + vz * dzu;
            gb.y = -alongOut * slopeMag / Math.max(n.y, 0.1);
        } else if (this._moveState === FPSC.MOVE_WALK) {
            // WALK: input-driven ground movement, projected tangent to groundNormal. Gravity off;
            // endStep clamps the feet to the surface (deterministic, no solver jitter on slopes).
            this.body.setGravity(0, 0, 0);
            var n2 = this.groundNormal;
            var mx, mz;
            if (hasInput) {
                // While slowing but still moving, bleed excess speed at sprintDecay. Read _ownVelocityX/Z
                // (not gb) so the platform's baked-in base velocity isn't re-seeded here.
                var cvx = this._ownVelocityX;
                var cvz = this._ownVelocityZ;
                var curSp = Math.sqrt(cvx * cvx + cvz * cvz);
                var wishSp = Math.sqrt(wishX * wishX + wishZ * wishZ);
                if (curSp > wishSp + FPSC.EPS_LEN) {
                    var target = Math.max(wishSp, curSp - this.sprintDecay * dt);
                    var kf = curSp > FPSC.EPS_DIR ? target / curSp : 0;
                    mx = cvx * kf;
                    mz = cvz * kf;
                } else {
                    mx = wishX;
                    mz = wishZ;
                }
            } else {
                // Carry current ground velocity; endStep's groundStopDecel is the sole stop authority.
                mx = this._ownVelocityX;
                mz = this._ownVelocityZ;
            }
            var dot = mx * n2.x + mz * n2.z;
            vx = mx - dot * n2.x;
            vz = mz - dot * n2.z;
            gb.y = -dot * n2.y;
        } else {
            // AIRBORNE: gravity + air control own velocity.
            this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
            var cur = gb;
            if (cur.y < -this._maxFall) { gb.y = -this._maxFall; }
            var curSp2 = Math.sqrt(cur.x * cur.x + cur.z * cur.z);
            var wishSp2 = Math.sqrt(wishX * wishX + wishZ * wishZ);
            if (hasInput) {
                if (wishSp2 >= curSp2) {
                    vx = cur.x + (wishX - cur.x) * this.airControl;
                    vz = cur.z + (wishZ - cur.z) * this.airControl;
                } else {
                    // Steer heading toward wish at the same magnitude, without bleeding speed.
                    var wl = wishSp2 || 1;
                    var tx = (wishX / wl) * curSp2;
                    var tz = (wishZ / wl) * curSp2;
                    vx = cur.x + (tx - cur.x) * this.airControl;
                    vz = cur.z + (tz - cur.z) * this.airControl;
                }
            } else {
                vx = cur.x;
                vz = cur.z;
            }
        }
    }

    if (!onLadderThisTick && !onMantleThisTick) {
        var cs = this._ceilingSlide(vx, gb.y, vz, dt);
        vx = cs.vx;
        vz = cs.vz;
        gb.y = cs.vy;
    }

    // Headroom gate: stop advancing into an overhang too low to fit under. Runs before
    // collide-and-slide so walls act on the already-gated velocity.
    var gated = this._headroomGate(vx, vz, dt);

    // Platform base velocity added immediately before the swept move, so a rider is carried through the
    // SAME collide-and-slide every other velocity goes through. Stays in gb.x/z afterward.
    var bvx = (onLadderThisTick || onMantleThisTick) ? 0 : this._baseVelocity.x;
    var bvz = (onLadderThisTick || onMantleThisTick) ? 0 : this._baseVelocity.z;

    // Step-up/step-down are emergent: collide-and-slide ignores anything shorter than stepHeight, and
    // the ground clamp in endStep raises/lowers us onto it.
    var slid = this._collideAndSlide(gated.x + bvx, gated.z + bvz, dt);
    gb.x = slid.x;
    gb.z = slid.z;
    this._ownVelocityX = slid.x - bvx;
    this._ownVelocityZ = slid.z - bvz;

    this._prevCrouch = !!cmd.crouch;
};

/**
 * POST-physics: decide grounded and clamp the feet to the ground surface. Also acquires this tick's
 * platform base velocity from whatever isPlatform-tagged body the ground probe lands on.
 * @method endStep
 * @param {Number} dt
 */
proto.endStep = function(dt) {
    var gb = this.body.linear_velocity;

    // While mounted on a ladder or mid-mantle arc, skip the ground clamp (it would re-snap us down).
    if (this._onLadder || this._mantleActive) {
        this.velocityY = gb.y;
        if (!this._resimulating || this._driveGhostDuringResim) { this._syncGhost(dt); }
        return;
    }

    if (this._groundSuppress > 0) { this._groundSuppress--; }
    // Only suppress grounding while rising (just jumped/thrust). _jumpRising extends this past the fixed
    // countdown while the character is STILL genuinely ascending; it clears once gb.y decays, so it can't
    // suppress indefinitely.
    if (this._jumpRising && gb.y <= 1) { this._jumpRising = false; }
    var suppressed = this._groundSuppress > 0 && gb.y > 1;

    var half = this.height / 2;
    var maxStick = this.grounded ? this.stepDownDist + this._skin : this._groundTol;

    // Walk candidates highest-first and take the first that ISN'T too tall to step onto, so a lower valid
    // candidate still grounds us when a taller obstacle is in reach.
    var candidates = this._probeGroundCandidates(this.stepDownDist);
    // Slide launch off a ramp apex, only while SLIDING and rising. Two ways the true edge shows up:
    //   1. the highest surface RECEDES (the ramp face runs out ahead), or
    //   2. a MISMATCHED face (e.g. a ramp end-cap) becomes the highest candidate and masks signal 1.
    var topCandidate = candidates.length > 0 ? candidates[0] : null;
    var topCandidateY = topCandidate ? topCandidate.point.y : null;
    var wasSliding = this._moveState === FPSC.MOVE_SLIDE;
    if (this.grounded && wasSliding && gb.y > FPSC.EPS_LEN && topCandidate !== null) {
        var receded = this._prevTopCandidateY !== null && topCandidateY < this._prevTopCandidateY - FPSC.EPS_LEN;
        var normalDot = topCandidate.normal.x * this.groundNormal.x +
            topCandidate.normal.y * this.groundNormal.y +
            topCandidate.normal.z * this.groundNormal.z;
        var mismatched = normalDot < this._minStandableNormalY;
        if (receded || mismatched) { candidates = []; this._slideLaunched = true; }
    }
    // A slide apex launch is latched: once it fires, force every candidate away while the arc is still
    // rising, so a shallow launch can't be ground-clamped back down. Clears once gb.y stops climbing.
    if (this._slideLaunched) {
        if (gb.y > FPSC.EPS_LEN) { candidates = []; }
        else { this._slideLaunched = false; }
    }
    this._prevTopCandidateY = topCandidateY;
    var probe = null, tooHighToStep = false;
    for (var ci = 0; ci < candidates.length; ci++) {
        var c = candidates[ci];
        var rise = (c.point.y + half) - this.body.position.y;
        // feet already inside this surface -> push out onto it, not a step-up to refuse
        var penetrating = (this.body.position.y - half) < c.point.y - this._skin;
        var tooHigh = this.grounded && !penetrating && rise > this.stepHeight + this._skin;
        if (!tooHigh) { probe = c; tooHighToStep = false; break; }
        if (!probe) { probe = c; tooHighToStep = true; } // keep the highest as a fallback reference
    }

    // feetGap > 0 = feet above ground; < 0 = penetrating (always clamp back out).
    var feetGap = probe ? this.body.position.y - half - probe.point.y : Infinity;

    if (!suppressed && probe && feetGap <= maxStick && !tooHighToStep) {
        var p = this.body.position;
        var clampedY = probe.point.y + half;
        if (!this._resimulating) { this._viewDisplacementY += clampedY - p.y; }
        this.body.position.set(p.x, clampedY, p.z);
        this.body.updateDerived();

        // Save the OUTGOING base velocity before overwriting it below: gb was built by LAST tick's
        // beginStep using THIS old value. Splitting gb against the new value would manufacture a
        // one-tick phantom "own velocity" spike when the platform's velocity changes abruptly.
        var outgoingBaseVelocityX = this._baseVelocity.x, outgoingBaseVelocityZ = this._baseVelocity.z;
        var standingOn = probe.object;
        if (standingOn && standingOn.isPlatform) {
            var pv = standingOn.linear_velocity;
            var bvx = pv.x, bvy = pv.y, bvz = pv.z;
            // Rotating platform: carry the character along the platform's own exact arc this tick, Y-axis
            // spin only. Use the CHORD velocity (offset exactly rotated by omega*dt, minus current)/dt,
            // not the instantaneous tangent — applying a tangent straight for a tick spirals outward.
            if (standingOn.isRotatingPlatform && standingOn.angular_velocity) {
                var omegaY = standingOn.angular_velocity.y;
                if (omegaY && dt > 0) {
                    var center = standingOn.position;
                    var rx = this.body.position.x - center.x;
                    var rz = this.body.position.z - center.z;
                    var theta = omegaY * dt;
                    var cosT = Scalar.cos(theta), sinT = Scalar.sin(theta);
                    // Matches the engine's own rotation convention: for omegaY > 0, the rotated offset is
                    // (rx*cos+rz*sin, rz*cos-rx*sin).
                    var rxRot = rx * cosT + rz * sinT;
                    var rzRot = rz * cosT - rx * sinT;
                    bvx += (rxRot - rx) / dt;
                    bvz += (rzRot - rz) / dt;
                }
            }
            this._baseVelocity.set(bvx, bvy, bvz);
        } else {
            this._baseVelocity.set(0, 0, 0);
        }

        // ================================================================================
        // MOVEMENT STATE DECISION — the ONE place per tick this is decided, from the ONE real
        // ground probe this tick has.
        // ================================================================================
        var pn = probe.normal;
        var probeSlope = Math.sqrt(pn.x * pn.x + pn.z * pn.z);

        // Project the incoming 3D velocity onto the surface plane (v -= (v·n)n), always — a no-op when
        // gb is already tangent to this same surface.
        var vdotn = gb.x * pn.x + gb.y * pn.y + gb.z * pn.z;
        var tangentX = gb.x - vdotn * pn.x;
        var tangentZ = gb.z - vdotn * pn.z;
        var horizTangentSpeed = Math.sqrt(tangentX * tangentX + tangentZ * tangentZ);

        // TRUE along-ground speed for the slide entry/sustain SPEED test only (platform velocity excluded,
        // so riding a fast platform can't launch an unwanted slide).
        var vdotnOwn = (gb.x - outgoingBaseVelocityX) * pn.x + gb.y * pn.y + (gb.z - outgoingBaseVelocityZ) * pn.z;
        var tangentOwnX = (gb.x - outgoingBaseVelocityX) - vdotnOwn * pn.x;
        var tangentOwnZ = (gb.z - outgoingBaseVelocityZ) - vdotnOwn * pn.z;
        var horizTangentOwnSpeed = Math.sqrt(tangentOwnX * tangentOwnX + tangentOwnZ * tangentOwnZ);

        var slopeMag0 = probeSlope;
        var ny0 = Math.max(pn.y, 0.1);
        var groundSp;
        if (slopeMag0 > FPSC.EPS_LEN) {
            var dxu0 = pn.x / slopeMag0, dzu0 = pn.z / slopeMag0;
            var alongH = tangentOwnX * dxu0 + tangentOwnZ * dzu0;
            var crossSq = Math.max(0, horizTangentOwnSpeed * horizTangentOwnSpeed - alongH * alongH);
            var surfFall = alongH / ny0;
            groundSp = Math.sqrt(surfFall * surfFall + crossSq);
        } else {
            groundSp = horizTangentOwnSpeed;
        }
        var tangentSpeed = groundSp;

        var isSlipSurface = this._isSlipSurface(pn);
        // Slide ENTRY/SUSTAIN uses the SAME rule on every tick (not just first contact): crouch held, and
        // on a slope ride until crouch releases; on flat, need speed above slideEndSpeed (sustain) or
        // moveSpeed (entry).
        var slopeSlideEligible = probeSlope >= this.slideSlopeMin;
        var hasMoveInputThisTick = this._hasMoveInput;
        var slideInputOk = !this.slideRequiresMoveInput || hasMoveInputThisTick ||
            (this.slideAllowLandingWithoutInput && !this.grounded);
        var slideSustainOk = slopeSlideEligible || tangentSpeed >= this.slideEndSpeed;
        var slideEntryOk = slideInputOk && tangentSpeed > this.moveSpeed + FPSC.EPS_SPEED_MARGIN;
        var wantsSlide = !!this._wantCrouch && (wasSliding ? slideSustainOk : slideEntryOk);

        if (wantsSlide) {
            this._moveState = FPSC.MOVE_SLIDE;
            var enteringSlide = !wasSliding;
            // slideBoost applied HERE, on the exact entry tick, to the velocity endStep commits — not in
            // _updateSlide (which runs next tick), or the boost would show one tick late.
            var boostedX = tangentX, boostedZ = tangentZ;
            if (enteringSlide && this.slideBoost !== 1) {
                boostedX *= this.slideBoost;
                boostedZ *= this.slideBoost;
            }
            gb.x = boostedX;
            gb.z = boostedZ;
            // gb.y is left for _updateSlide's onSlope solve to derive.
            gb.y = 0;
        } else if (isSlipSurface) {
            // Entry edge: this tick starts a NEW slip iff last tick wasn't already one. beginStep's SLIP
            // branch re-projects gb onto groundNormal only on that one entry tick.
            var enteringSlip = this._moveState !== FPSC.MOVE_SLIP;
            this._slipJustEntered = enteringSlip;
            this._moveState = FPSC.MOVE_SLIP;
            // Keep the RAW incoming velocity on the entry tick (beginStep's SLIP branch projects from
            // it); from the second slip tick on, gb.y is zeroed here.
            if (!enteringSlip) { gb.y = 0; }
        } else {
            this._moveState = FPSC.MOVE_WALK;
            gb.x = tangentX;
            gb.z = tangentZ;
            gb.y = 0;
        }
        // Split against the OUTGOING (pre-acquire) base velocity.
        this._ownVelocityX = gb.x - outgoingBaseVelocityX;
        this._ownVelocityZ = gb.z - outgoingBaseVelocityZ;

        // Idle ground-stop: WALK only. Bleeds the character's OWN component so it doesn't fight the ride,
        // then adds base velocity back.
        if (this._cmdIdle && this._moveState === FPSC.MOVE_WALK) {
            var cvx = this._ownVelocityX || 0;
            var cvz = this._ownVelocityZ || 0;
            var sp = Math.sqrt(cvx * cvx + cvz * cvz);
            var target = Math.max(0, sp - this.groundStopDecel * dt);
            var kf = sp > FPSC.EPS_SPD ? target / sp : 0;
            this._ownVelocityX = cvx * kf;
            this._ownVelocityZ = cvz * kf;
            gb.x = this._ownVelocityX + this._baseVelocity.x;
            gb.z = this._ownVelocityZ + this._baseVelocity.z;
        }

        this.grounded = true;
        this.groundNormal.set(probe.normal.x, probe.normal.y, probe.normal.z);
    } else if (tooHighToStep) {
        // Refusing to climb something too tall must NOT count as leaving the ground: no gap, no fall.
        gb.y = 0;
        // Movement state is UNCHANGED here on purpose — the character is still resting as before.
    } else {
        this.grounded = false;
        // A slide that leaves the ground stays MOVE_SLIDE through the airborne arc while horizontal speed
        // is still above slideEndSpeed and crouch is still held. Landing re-enters the decision above.
        var wasSlideBeforeLoss = this._moveState === FPSC.MOVE_SLIDE;
        var stillFastEnough = Math.sqrt(gb.x * gb.x + gb.z * gb.z) >= this.slideEndSpeed;
        if (wasSlideBeforeLoss && this._wantCrouch && stillFastEnough) {
            this._moveState = FPSC.MOVE_SLIDE;
        } else {
            this._moveState = FPSC.MOVE_AIRBORNE;
        }
        // Genuinely airborne — no ground entity to inherit velocity from. A jump already captured
        // baseVelocity.y the tick it fired; clearing here stops future ticks reading a stale platform velocity.
        this._baseVelocity.set(0, 0, 0);
    }

    // Coyote window: refill while grounded, bleed down once airborne. No-op when coyoteTime=0.
    if (this.grounded) { this._coyoteTimer = this.coyoteTime; }
    else if (this._coyoteTimer > 0) { this._coyoteTimer = Math.max(0, this._coyoteTimer - dt); }

    this.velocityY = gb.y;

    // Drive the ghost every tick, INCLUDING during resim, so object pushes are reproduced on rollback.
    // The knockback READBACK is gated separately inside _syncGhost. Opt-out: freeze the ghost during resim.
    if (!this._resimulating || this._driveGhostDuringResim) { this._syncGhost(dt); }
};
