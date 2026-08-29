// Movement state machine core: beginStep (pre-physics velocity + assists) and endStep (post-physics
// grounding + state decision) bracket a single physics world step. See the class doc on
// FPSCharacterController.js for the beginStep/world.step/endStep contract, and the "MOVEMENT STATE
// DECISION" comment inside endStep below for the full state-machine design.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * PRE-physics: set this tick's horizontal velocity (slope/wall projected) + assists.
 *
 * Aim/sim separation: the movement basis comes from the COMMAND's yaw (`cmd.yaw`) — a per-tick
 * input — not from any persistent "live aim". The live aim belongs to the caller (a camera reads
 * it, never the sim), so replaying commands during reconciliation can't drag the view backward.
 * We record the commanded yaw/pitch as this entity's facing (for getState/avatars) only when the
 * command carries them; a caller that never sets yaw keeps driving facing via look() instead.
 *
 * Also applies platform base velocity into the horizontal move (see the constructor's
 * _baseVelocity comment) immediately before collide-and-slide, so a rider is carried through real
 * swept motion rather than a position teleport.
 *
 * @method beginStep
 * @param {Object} command - pure-data input command struct; any field may be absent
 * @param {Number} dt
 */
proto.beginStep = function(command, dt) {
    var cmd = command || {};

    if (this._jumpBufferTimer > 0) { this._jumpBufferTimer = Math.max(0, this._jumpBufferTimer - dt); }

    if (cmd.scale !== undefined && Math.abs(cmd.scale - this.scale) > FPSC.EPS_LEN) { this.setScale(cmd.scale); }
    // Steep-slope walk intent from the command. Applying it immediately lets local prediction climb
    // right away; if an authority later overrules it, setState corrects the flag from the snapshot.
    // Read live per-tick, so a plain assignment is enough.
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
        wishX = (dirX / dirLen) * speed;
        wishZ = (dirZ / dirLen) * speed;
    }

    // Stashed for endStep (this same tick, after world.step) to use when it decides this tick's
    // movement state from the fresh ground probe — see the "MOVEMENT STATE DECISION" block there.
    this._wantCrouch = wantCrouch;
    this._hasMoveInput = hasInput;

    var onMantleThisTick = this._updateMantle(cmd, moveYaw, dt);

    // _updateLadder mounts/dismounts and, while mounted, owns velocity fully — checked first since
    // it can override every other state this tick (a ladder grab works even mid-air or mid-slide).
    var onLadderThisTick = !onMantleThisTick && this._updateLadder(cmd, moveYaw, movePitch, dt);

    var vx, vz;
    if (onLadderThisTick || onMantleThisTick) {
        // LADDER / MANTLE: the hook already wrote gb.x/y/z; velocity is fully its.
        vx = gb.x;
        vz = gb.z;
    } else {
        // A jump flips grounded→airborne HERE, before the dispatch below reads this._moveState —
        // _updateVertical updates this._moveState directly on a jump so the same-tick dispatch
        // correctly takes the AIRBORNE branch instead of the stale GROUNDED one.
        this._updateVertical(cmd, dt);

        // ================================================================================
        // MOVEMENT STATE DISPATCH — reads this._moveState, set authoritatively by LAST tick's
        // endStep (or by _updateVertical just above, on a jump this tick). Never re-derives the
        // state from other flags; each branch below is a fully self-contained velocity model for
        // that one state, duplicated rather than shared, so there is exactly one thing to read
        // (this._moveState) to know which branch is live and exactly one place per state that
        // decides its velocity. See the "Movement state machine" comment above endStep.
        // ================================================================================
        if (this._moveState === FPSC.MOVE_SLIDE && this.grounded) {
            // SLIDE, GROUNDED: crouch-at-speed, owns velocity via _updateSlide's surface-tracking
            // model. _updateSlide is a pure per-tick evolver here — it does NOT decide entry/exit
            // anymore (endStep already decided this tick IS a slide); it only advances the
            // slide's velocity one tick (slope accel, friction, steering) from gb, which endStep
            // already set to the correct tangential speed for this tick.
            var slideResult = this._updateSlide(cmd, wishX, wishZ, dt);
            vx = slideResult.vx;
            vz = slideResult.vz;
            gb.y = slideResult.vy;
        } else if (this._moveState === FPSC.MOVE_SLIDE && !this.grounded) {
            // SLIDE, AIRBORNE: a slide that left the ground (ramp lip, drop-off) — see endStep's
            // "genuinely airborne" branch for the condition that keeps this state through the
            // launch. Carried ballistically (gravity, no air-control degradation, no slope model —
            // there's no surface under the character to track) until it lands or slows below
            // slideEndSpeed, at which point endStep drops it to AIRBORNE.
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
            // (endStep left gb.y raw, non-zero, from the fall/toss, on that one tick — see the
            // "MOVEMENT STATE DECISION" comment in endStep). On every later slip tick, endStep
            // zeroes gb.y (the kinematic model owns vertical here, not the solver), so gb.x/gb.z
            // are ALREADY the correctly-accumulated tangential speed from the previous tick's
            // formula below — re-projecting again would read that zeroed gb.y as "no vertical
            // motion yet" and subtract a spurious correction, fighting the accumulation into a
            // false plateau instead of letting speed build tick over tick.
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
            // WALK: ordinary input-driven ground movement, projected tangent to groundNormal.
            // KINEMATIC GROUND: gravity off; endStep clamps the feet to the surface. Fully
            // deterministic, doesn't rely on the solver to hold us on a slope (which jittered).
            this.body.setGravity(0, 0, 0);
            var n2 = this.groundNormal;
            var mx, mz;
            if (hasInput) {
                // When slowing while still moving, bleed excess speed at sprintDecay instead of
                // snapping to the lower target speed. _ownVelocityX/Z, not gb.x/z — gb may
                // already carry a platform's base velocity baked in (see the constructor
                // comment); reading it here would re-seed "current speed" with the platform's
                // own speed already added, which then gets base velocity added AGAIN below
                // every tick instead of decaying.
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
                // Carry current ground velocity; endStep's groundStopDecel is the sole stop
                // authority. _ownVelocityX/Z, NOT gb.x/z — same reasoning as above.
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

    // Headroom gate: stop us advancing into an overhang too low to fit under (a ramp
    // underside closing onto the floor). A near-horizontal overhang has almost no
    // horizontal surface normal, so collide-and-slide can't see it — we gate on
    // ceiling CLEARANCE instead. Runs before collide-and-slide so walls act on the
    // already-gated velocity.
    var gated = this._headroomGate(vx, vz, dt);

    // Platform base velocity: added in immediately before the swept move so a rider is carried
    // through the SAME collide-and-slide every other velocity goes through (real swept motion, not
    // a position teleport). Stays in gb.x/z afterward — see the constructor's comment for why.
    var bvx = (onLadderThisTick || onMantleThisTick) ? 0 : this._baseVelocity.x;
    var bvz = (onLadderThisTick || onMantleThisTick) ? 0 : this._baseVelocity.z;

    // Step-up/step-down are emergent: collide-and-slide ignores anything shorter than
    // stepHeight, and the ground clamp in endStep raises/lowers us onto it. _collideAndSlide reads
    // this._moveState itself (see its own comment) to exempt an active slide from the too-steep
    // wall rule.
    var slid = this._collideAndSlide(gated.x + bvx, gated.z + bvz, dt);
    gb.x = slid.x;
    gb.z = slid.z;
    this._ownVelocityX = slid.x - bvx;
    this._ownVelocityZ = slid.z - bvz;

    this._prevCrouch = !!cmd.crouch;
};

/**
 * POST-physics: decide grounded and clamp the feet to the ground surface. Also acquires this
 * tick's platform base velocity (see the constructor's _baseVelocity comment) from whatever
 * isPlatform-tagged body the ground probe lands on, read fresh every tick.
 * @method endStep
 * @param {Number} dt
 */
proto.endStep = function(dt) {
    var gb = this.body.linear_velocity;

    // While mounted on a ladder or mid-mantle arc, skip the ground clamp — it would otherwise
    // re-snap the character onto the floor every tick while being carried upward.
    if (this._onLadder || this._mantleActive) {
        this.velocityY = gb.y;
        if (!this._resimulating || this._driveGhostDuringResim) { this._syncGhost(dt); }
        return;
    }

    if (this._groundSuppress > 0) { this._groundSuppress--; }
    // Only suppress grounding while rising (just jumped/thrust); while falling the ground
    // catch must stay live or the body tunnels through the floor. _jumpRising extends this past the
    // fixed countdown for as long as the character is STILL genuinely ascending — see its own
    // comment at the jump site for why a flat tick count alone isn't enough (a still-rising surface
    // underfoot, platform or ramp, can re-enter snap range before the countdown's fixed window would
    // ever expect it to). Cleared the moment gb.y decays past the threshold, so this can't suppress
    // indefinitely — ordinary gravity decay is what ends it.
    if (this._jumpRising && gb.y <= 1) { this._jumpRising = false; }
    var suppressed = this._groundSuppress > 0 && gb.y > 1;

    var half = this.height / 2;
    var maxStick = this.grounded ? this.stepDownDist + this._skin : this._groundTol;

    // Walk candidates highest-first and take the first that ISN'T too tall to step onto (relative
    // to current feet, only while already grounded — see tooHighToStep below). Falling through to
    // a lower, valid candidate keeps grounding honest when a taller obstacle (e.g. a box shoved
    // against the footprint) is also in reach.
    var candidates = this._probeGroundCandidates(this.stepDownDist);
    // Slide launch off a ramp apex — only while SLIDING and rising (walking off the same edge just
    // follows the ground down). ANGLE-BLIND: a slide treats every slope identically regardless of
    // steepness, so this gate never asks "is this too steep" — only "is this still the surface I'm
    // riding." Two ways the true edge shows up in the probe, both handled here:
    //   1. The highest surface RECEDES: the ramp face we were climbing runs out ahead, so the highest
    //      remaining ramp hit drops vs last tick. Clamping to it would hug us down a one-tick dip.
    //   2. A MISMATCHED face (e.g. the ramp's own end-cap) becomes the highest candidate: taller than
    //      the ramp face but not the surface we're riding (normal meaningfully off groundNormal). It can
    //      mask signal #1 by sitting on top, so we test it independently — riding a ramp, the candidate
    //      still ON that same face keeps matching every tick and never trips this; only a genuinely
    //      different face (the real edge) does.
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
    // A slide apex launch is latched, not a one-tick decision: the tick it fires, grounded flips false
    // immediately, so the gate above (which requires it true) can never re-arm to catch a second graze
    // later in the same arc. Without this latch, a low/shallow launch that skims just above the ramp's
    // tail gets ground-clamped straight back down the very next tick the probe happens to reach it — a
    // one-tick "dip" mid-arc. Sliding off an apex must NEVER re-hug the geometry, full stop, so once
    // latched we force every candidate away regardless of what the probe finds, for as long as the arc
    // is still rising. The latch clears once gb.y stops climbing (the arc has peaked and started to
    // fall) — from that point a real landing is legitimate and ground detection must resume normally.
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

        // Save the OUTGOING base velocity before overwriting it below — gb (about to be split into
        // own-vs-base components further down) was built by LAST tick's beginStep using THIS old
        // value, not the new one we're about to acquire. Splitting gb against the NEW value instead
        // manufactures a one-tick phantom "own velocity" spike whenever the platform's velocity
        // changes abruptly between ticks (a reversing elevator/shuttle, or a rotating platform
        // changing direction each tick): gb still reflects the old speed, so subtracting the new
        // speed leaves a large bogus residual that then has to visibly bleed off via the idle
        // ground-stop decay below. Using the OLD value here keeps the split correct for the
        // tick gb was actually built on; the NEW value (acquired below) still lands in
        // this._baseVelocity for beginStep to pick up fresh next tick, same as always.
        var outgoingBaseVelocityX = this._baseVelocity.x, outgoingBaseVelocityZ = this._baseVelocity.z;
        var standingOn = probe.object;
        if (standingOn && standingOn.isPlatform) {
            var pv = standingOn.linear_velocity;
            var bvx = pv.x, bvy = pv.y, bvz = pv.z;
            // Rotating platform: carry the character along the platform's own EXACT arc this tick,
            // Y-axis spin only (the only axis a standable platform can usefully spin on). Recomputed
            // fresh every tick from the CURRENT offset (not cached), so as the character walks
            // toward/away from the pivot the imparted speed tracks the true radius, and so it decays
            // to zero at the pivot itself.
            //
            // NOT a naive omega x r tangential velocity: that's only the arc's INSTANTANEOUS tangent,
            // and applying it as a straight line for a full tick always overshoots the true curve —
            // every tick's move ends up very slightly outside the circle, and next tick's tangent is
            // computed from that already-drifted position, so the error compounds tick over tick into
            // an outward spiral (visible at high spin rates as being "flung off the platform"). Fix:
            // compute the CHORD velocity instead — the constant velocity that carries the rider from
            // its current offset to the offset EXACTLY rotated by theta=omegaY*dt, i.e.
            // (rotated - current) / dt. This reproduces the platform's real circular motion exactly
            // regardless of angular speed, instead of approximating it.
            if (standingOn.isRotatingPlatform && standingOn.angular_velocity) {
                var omegaY = standingOn.angular_velocity.y;
                if (omegaY && dt > 0) {
                    var center = standingOn.position;
                    var rx = this.body.position.x - center.x;
                    var rz = this.body.position.z - center.z;
                    var theta = omegaY * dt;
                    var cosT = Scalar.cos(theta), sinT = Scalar.sin(theta);
                    // Matches the engine's own rotation convention (verified against RigidBody's quaternion
                    // integration directly, not assumed): for omegaY > 0, the rotated offset is
                    // (rx*cos+rz*sin, rz*cos-rx*sin) — the same sense that produced the correct
                    // (omegaY*rz, -omegaY*rx) instantaneous tangent this replaces.
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
        // ground probe this tick has. beginStep (next tick) only ever reads this._moveState; it
        // never re-derives sliding/slipping/walking from other flags.
        // ================================================================================
        var pn = probe.normal;
        var probeSlope = Math.sqrt(pn.x * pn.x + pn.z * pn.z);

        // Project the incoming 3D velocity onto the surface plane ONCE, here, on every grounding
        // tick — not just the first-contact tick. (v -= (v·n)n): removes the into-surface
        // component, keeps the along-surface (tangential) component. On a tick where the body was
        // already resting on this same surface last tick too, this is a no-op (gb is already
        // tangent), so it's safe to always run — no separate "first contact only" special case.
        var vdotn = gb.x * pn.x + gb.y * pn.y + gb.z * pn.z;
        var tangentX = gb.x - vdotn * pn.x;
        var tangentZ = gb.z - vdotn * pn.z;
        var horizTangentSpeed = Math.sqrt(tangentX * tangentX + tangentZ * tangentZ);

        // TRUE along-the-ground speed, for the slide entry/sustain SPEED test only (tangentX/Z above,
        // which DOES include platform velocity, is what actually gets written to gb). Platform
        // velocity is excluded from this speed reading — otherwise a fast rotating platform's own
        // tangential speed alone can exceed moveSpeed with zero player effort, launching an unwanted
        // slide on crouch while just riding. Reconstructs true 3D along-surface speed the same way a
        // slope converts fall speed to horizontal (divide the along-slope component by ny).
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
        // Slide ENTRY/SUSTAIN uses the SAME rule regardless of whether this is the first contact
        // tick or the 500th tick of an already-active slide: crouch held, and (on a slope, ride
        // until crouch releases; on flat, need speed above slideEndSpeed to keep going / above
        // moveSpeed to start). This mirrors _updateSlide's old entry/sustain split, but evaluated
        // ONCE, with this tick's own fresh probe normal and true tangential speed — not the
        // previous tick's groundNormal, not a landing-only special case.
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
            // slideBoost applied HERE, on the exact entry tick, directly to the velocity endStep is
            // about to commit — not inside _updateSlide (which only runs the FOLLOWING tick, in
            // beginStep). Applying it there would show the boost one tick later than the state
            // transition itself, which is observably wrong (a caller reading "just started
            // sliding" this tick would see un-boosted speed).
            var boostedX = tangentX, boostedZ = tangentZ;
            if (enteringSlide && this.slideBoost !== 1) {
                boostedX *= this.slideBoost;
                boostedZ *= this.slideBoost;
            }
            gb.x = boostedX;
            gb.z = boostedZ;
            // gb.y is left for _updateSlide's onSlope solve to derive from the tangential speed
            // above — writing a raw projected vertical here overshoots the surface-follow value
            // and skips the character off the ramp for a tick (a bounce).
            gb.y = 0;
        } else if (isSlipSurface) {
            // Entry edge: this tick starts a NEW slip iff last tick wasn't already one. beginStep's
            // SLIP branch only re-projects gb onto
            // groundNormal on that one entry tick (see its own comment) — every later tick, gb.y
            // is already 0 (set below) and gb.x/gb.z already hold the correctly-accumulated
            // tangential speed from beginStep's own per-tick formula, so re-projecting again would
            // corrupt that accumulation into a false plateau.
            var enteringSlip = this._moveState !== FPSC.MOVE_SLIP;
            this._slipJustEntered = enteringSlip;
            this._moveState = FPSC.MOVE_SLIP;
            // Keep the RAW incoming gb.x/gb.z/gb.y (NOT the tangential projection) on the entry
            // tick — beginStep's SLIP branch does its own plane projection from this.groundNormal
            // next tick, gated to _slipJustEntered, and needs gb.y to still be the real incoming
            // fall speed to project. From the SECOND slip tick on, gb.y is zeroed here as usual —
            // beginStep's per-tick formula derives its own vy from there on, and leaving a stale
            // gb.y would double-count it.
            if (!enteringSlip) { gb.y = 0; }
        } else {
            this._moveState = FPSC.MOVE_WALK;
            gb.x = tangentX;
            gb.z = tangentZ;
            gb.y = 0;
        }
        // Split against the OUTGOING (pre-acquire) base velocity, not the freshly-acquired one — see
        // the comment above outgoingBaseVelocityX/Z's declaration for why.
        this._ownVelocityX = gb.x - outgoingBaseVelocityX;
        this._ownVelocityZ = gb.z - outgoingBaseVelocityZ;

        // Idle ground-stop: WALK only. Bleeds horizontal speed toward zero at groundStopDecel.
        // Reads/writes _ownVelocityX/Z (the character's OWN component), NOT gb.x/z directly — gb
        // may already carry a platform's base velocity baked in, and decaying THAT would fight
        // the ride. The decayed own-component is added back onto base velocity so gb ends up
        // carrying: decayed own motion + full undecayed platform motion.
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
        // Refusing to climb something too tall (e.g. a box shoved into the footprint) must NOT be
        // treated as leaving the ground: the feet haven't moved, there's no gap, no fall — the
        // character is exactly where it was a moment ago, still resting on whatever it was resting
        // on. Staying grounded on rejection keeps the height-limit check
        // (this.grounded && rise > stepHeight) honest on the next tick too.
        gb.y = 0;
        // Movement state is UNCHANGED here on purpose: the character is exactly where it was,
        // still resting on whatever it was resting on, so whatever state that was is still true.
    } else {
        this.grounded = false;
        // A slide that leaves the ground (ramp lip, drop-off) stays MOVE_SLIDE through the airborne
        // arc — carried mostly ballistically rather than air-controlled — as long as horizontal
        // speed is still above slideEndSpeed (the same floor flat sliding itself uses to decide
        // "still going") and crouch is still held. beginStep's SLIDE branch has its own airborne vs.
        // grounded sub-cases for exactly this reason. Landing re-enters the ordinary MOVEMENT STATE
        // DECISION above on the fresh probe normal, so it naturally continues sliding (onto a ramp)
        // or drops to WALK/SLIP there — no separate landing special-case needed here.
        var wasSlideBeforeLoss = this._moveState === FPSC.MOVE_SLIDE;
        var stillFastEnough = Math.sqrt(gb.x * gb.x + gb.z * gb.z) >= this.slideEndSpeed;
        if (wasSlideBeforeLoss && this._wantCrouch && stillFastEnough) {
            this._moveState = FPSC.MOVE_SLIDE;
        } else {
            this._moveState = FPSC.MOVE_AIRBORNE;
        }
        // Genuinely airborne — no ground entity to inherit velocity from. A jump already captured
        // baseVelocity.y additively the tick it fired (_updateVertical); clearing here only stops
        // FUTURE ticks from reading a stale platform velocity while falling free.
        this._baseVelocity.set(0, 0, 0);
    }

    // Coyote window: refill while grounded, bleed down once airborne. No-op when coyoteTime=0.
    if (this.grounded) { this._coyoteTimer = this.coyoteTime; }
    else if (this._coyoteTimer > 0) { this._coyoteTimer = Math.max(0, this._coyoteTimer - dt); }

    this.velocityY = gb.y;

    // Drive the ghost every tick, INCLUDING during resim: the ghost is how the character pushes objects,
    // and object pushes must be reproduced when already-run commands get rolled back and resimulated
    // (otherwise a pushed object is predicted live but snaps back every snapshot — rubber-banding). The
    // ghost drive is deterministic given the character's state. What must NOT run during resim is the
    // knockback READBACK from the ghost into the character (see _syncGhost / _readGhostKnockback): feeding
    // a solver body's contact velocity back into the character mid-rollback is what injects non-determinism
    // into the reconciled character path. That readback is gated inside _syncGhost.
    // Opt-out (driveGhostDuringResim=false): freeze the ghost during resim (older behavior).
    if (!this._resimulating || this._driveGhostDuringResim) { this._syncGhost(dt); }
};
