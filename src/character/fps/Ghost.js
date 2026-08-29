// Ghost body lifecycle: a solver-participating dynamic body that trails the kinematic character for
// object-contact purposes. The character's own body is excluded from solver contacts (collision_mask
// 1); the ghost is its stand-in for object contact. Control is one-way: character position -> ghost
// target. The ghost's position never writes back to the character; only contact-derived knockback
// flows back (_syncGhost / _readGhostKnockback), as a velocity nudge. See _syncGhost.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * GHOST: a solver-participating dynamic body that trails the kinematic character. The character's own
 * body is excluded from solver contacts (collision_mask 1); the ghost is its stand-in for object
 * contact. Control is one-way: character position -> ghost target. The ghost's position never writes
 * back to the character; only contact-derived knockback flows back (_syncGhost), as a velocity nudge.
 *
 * @method _buildGhost
 * @private
 * @param {Vector3} position - the character body's current position.
 * @param {Object} [carriedVel] - {x,y,z} velocity to seed the ghost with (carried over a rebuild).
 */
proto._buildGhost = function(position, carriedVel) {
    // Ghost bottom is inset above the character's feet so it doesn't overlap a surface the character is
    // standing on (that's _probeGround's job). Top is unchanged, so head-height contact is unaffected.
    var groundInset = this.height * FPSC.GHOST_GROUND_INSET;
    var ghostHeight = this.height - groundInset;
    var ghostPos = new Vector3(position.x, position.y + groundInset / 2, position.z);
    var ghostShape = new BoxShape(this.width / 2, ghostHeight / 2, this.depth / 2);
    this._ghost = new RigidBody(ghostShape, this.mass);
    FPSCharacterController._applyMaterial(this._ghost, {
        friction: this._ghostMaterial.friction,
        restitution: this._ghostMaterial.restitution,
        linearDamping: this._ghostMaterial.linearDamping,
        angularDamping: this._ghostMaterial.angularDamping,
        gravity: new Vector3(0, 0, 0)
    });
    this._ghost.position.copy(ghostPos);
    this._ghost.updateDerived();
    this._ghostObject = { body: this._ghost, isVisible: false };
    this._ghostCommandedVel = null;
    this._ghost.name = this._bodyName + "_ghost";
    this._ghost.angular_factor.set(0, 0, 0);
    this._ghost.isKinematicCharacter = true;
    // Distinguishes this body from a real character body for OTHER controllers' sweeps: their own
    // kinematic body is never a wall (it has no mass to yield against), but this ghost IS a real
    // solver-participating mass and should block/get pushed like any other object.
    this._ghost.isCharacterGhost = true;
    this._ghostGroundInset = groundInset;
    if (carriedVel) { this._ghost.linear_velocity.set(carriedVel.x, carriedVel.y, carriedVel.z); }
    this.world.addRigidBody(this._ghost);
};

/**
 * Remove and clear the ghost body, if any (called before every rebuild + on destroy()).
 * @method _destroyGhost
 * @private
 */
proto._destroyGhost = function() {
    if (this._ghostObject) {
        this.world.removeRigidBody(this._ghost);
        this._ghostObject = null;
        this._ghost = null;
    }
};

/**
 * Drive the ghost toward the character each tick and read back contact-driven knockback. Called once
 * per endStep, after the character's position is settled.
 *
 * @method _syncGhost
 * @private
 * @param {Number} dt
 */
proto._syncGhost = function(dt) {
    if (!this._ghost) { return; }
    var p = this.body.position;
    var cv = this.body.linear_velocity;
    var gp = this._ghost.position;
    // Target where the character WILL BE at the end of this tick (p + v*dt), not where it is right
    // now — closing "the gap as of the start of the tick" is already stale by the time it's applied,
    // since the character moves by v*dt over that same tick. Without this the ghost permanently lags
    // by ~one tick's worth of the character's own motion, growing with speed, even at constant
    // velocity (no acceleration needed to produce it).
    var targetX = p.x + cv.x * dt;
    var targetY = p.y + cv.y * dt + (this._ghostGroundInset || 0) / 2;
    var targetZ = p.z + cv.z * dt;
    var dx = targetX - gp.x, dy = targetY - gp.y, dz = targetZ - gp.z;
    var gap = Math.sqrt(dx * dx + dy * dy + dz * dz);
    var gv = this._ghost.linear_velocity;

    // A gap this large is a rebuild/respawn/teleport: beam the ghost to the character instead of chasing.
    var teleportDist = Math.max(this.width, this.height) * 2;
    if (gap > teleportDist) {
        this._ghost.position.set(p.x, p.y + (this._ghostGroundInset || 0) / 2, p.z);
        gv.set(0, 0, 0);
        this._ghostCommandedVel = { x: 0, y: 0, z: 0 };
        return;
    }

    // Knockback signal = (ghost's actual velocity) - (velocity the drive commanded last tick). This
    // runs during resim too: an authority that never resims applies knockback in its own live step, so
    // skipping it here while resimulating would reconcile the character's velocity to a value that
    // permanently disagrees with authority by the knockback amount. It only needs to be deterministic
    // run-to-run (it is — the read is a pure function of the current contact state).
    this._readGhostKnockback();

    // Drive the ghost directly at the velocity that closes the (predicted) gap this tick. No cap:
    // any cap below the gap-closing speed just reintroduces a residual gap on fast motion — the
    // predicted-target math above already keeps this bounded and small under normal conditions.
    gv.x = dx / dt; gv.y = dy / dt; gv.z = dz / dt;

    // Clip the ghost's horizontal velocity through the same swept collide-and-slide the character uses.
    var clip = this._sweptCollideAndSlide({
        position: new Vector3(gp.x, gp.y, gp.z),
        width: this.width, depth: this.depth, height: this.height - (this._ghostGroundInset || 0),
        skin: this._skin, mass: this.mass, stepHeight: 0,
        selfBody: this._ghost, otherSelfBody: this.body,
        climbSteepSlopes: false,
        vx: gv.x, vz: gv.z, dt: dt,
    });
    gv.x = clip.x; gv.z = clip.z;
    if (clip.depenX !== 0 || clip.depenZ !== 0) {
        this._ghost.position.set(gp.x + clip.depenX, gp.y, gp.z + clip.depenZ);
    }

    this._ghostCommandedVel = { x: gv.x, y: gv.y, z: gv.z }; // baseline for next tick's (actual - commanded) knockback read
};

/**
 * Knockback speed = mass ratio (objectMass/(objectMass+playerMass)) x the object's closing speed
 * onto the character, gated to only apply when the object is moving into the character above a small
 * momentum floor. Horizontal only; never moves position, only velocity.
 *
 * @method _readGhostKnockback
 * @private
 */
proto._readGhostKnockback = function() {
    if (!this._receivePush) { return; }
    var world = this.world;
    if (!world || !world.narrowphase) { return; }
    var ghostBody = this._ghost;
    var pb = this.body.linear_velocity;
    var mP = this.mass;

    var manifolds = world.narrowphase.manifolds.values();
    for (var manifold = manifolds.next(); !manifold.done; manifold = manifolds.next()) {
        var m = manifold.value;
        var other =
            m.bodyA === ghostBody ? m.bodyB :
            m.bodyB === ghostBody ? m.bodyA : null;
        // A player is a wall, not a pushable object — no knockback from another player's ghost.
        if (other && other.bodyType === RigidBody.DYNAMIC && other._mass > 0 && !other.isKinematicCharacter) {
            var mB = other._mass;
            var ov = other.linear_velocity;
            var nx = this._ghost.position.x - other.position.x;
            var nz = this._ghost.position.z - other.position.z;
            var nlen = Math.sqrt(nx * nx + nz * nz);
            if (nlen > FPSC.EPS_LEN) { nx /= nlen; nz /= nlen; } else { nx = 0; nz = 0; }
            // n points box->character. The knockback should trigger on how fast the BOX is coming at you
            // (ov.n), NOT the relative closing speed (ov-pb).n. Using the relative speed folds in YOUR
            // OWN approach velocity (-pb.n > 0 when you walk into the box), so pushing a box knocked you
            // backward every tick — you push, it shoves you back, you re-approach: a limit cycle that
            // renders as the box micro-oscillating toward/away from you at close range. Gating on the
            // box's own inbound speed means a box only knocks you when IT carries momentum at you
            // (someone else shoved it, an explosion) — your own push no longer bounces back. Opt-out via
            // receiveSelfPush to restore the old relative-speed behavior.
            var closing = this._receiveSelfPush ?
                (ov.x - pb.x) * nx + (ov.z - pb.z) * nz :   // legacy: relative closing (self-push included)
                ov.x * nx + ov.z * nz;                      // box's own inbound speed only
            if (closing > FPSC.KB_CLOSING_MIN) {
                var massRatio = mB / (mB + mP);
                var kbv = massRatio * closing;
                if (kbv > this._receiveMaxSpeed) { kbv = this._receiveMaxSpeed; }
                kbv *= this._receiveKnockbackFraction;
                // Cap the RESULTING along-n speed, not just this tick's increment: clamping only kb
                // bounds each tick's contribution but not the running total, so sustained contact (a
                // heavy object pressed against the character for many ticks) adds another kb-worth of speed
                // every tick and blows straight past receiveMaxSpeed. Clamp what the character's velocity
                // ALONG n would become after this tick's push to receiveMaxSpeed instead — a fresh hit
                // (little/no existing along-n speed) still gets up to the full kb, but once already at
                // the cap from prior contact, further ticks add nothing more.
                var alongN = pb.x * nx + pb.z * nz;
                var room = this._receiveMaxSpeed - alongN;
                if (room > 0) { kbv = Math.min(kbv, room); } else { kbv = 0; }
                if (kbv > FPSC.KB_MIN) {
                    pb.x += nx * kbv;
                    pb.z += nz * kbv;
                    this.grounded = false;
                    // This runs mid-tick, inside beginStep's ghost sync — the movement-state dispatch
                    // for THIS tick already ran (it's earlier in beginStep), so this can't retroactively
                    // change what velocity model owned this tick's motion. It CAN and must fix what the
                    // NEXT tick sees: without this, next tick's dispatch would read the stale grounded
                    // sub-state (WALK) and immediately re-clamp the character back onto the ground via
                    // WALK's kinematic model, killing the knockback before it ever got airborne.
                    this._moveState = FPSC.MOVE_AIRBORNE;
                    if (this._groundSuppress < FPSC.GROUND_SUPPRESS_KB) { this._groundSuppress = FPSC.GROUND_SUPPRESS_KB; }
                }
            }
            break;
        }
    }
};
