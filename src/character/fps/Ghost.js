// Ghost body lifecycle: a solver-participating dynamic body that trails the kinematic character to
// give it object contact (the character's own body is excluded from the solver). Control is one-way:
// character position -> ghost target. Only contact-derived knockback flows back, as a velocity nudge.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Create the ghost body that trails the character for object contact.
 * @method _buildGhost
 * @private
 * @param {Vector3} position - the character body's current position.
 * @param {Object} [carriedVel] - {x,y,z} velocity to seed the ghost with (carried over a rebuild).
 */
proto._buildGhost = function(position, carriedVel) {
    // Inset the ghost's bottom above the character's feet so it doesn't overlap standing ground.
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
    // Distinguishes this ghost from a raw character body for other controllers' sweeps: a ghost is a
    // real solver mass and should block/get pushed like any other object.
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
    // Target the character's predicted end-of-tick position (p + v*dt), not its current one, so the
    // ghost doesn't permanently lag by ~one tick of the character's own motion.
    var targetX = p.x + cv.x * dt;
    var targetY = p.y + cv.y * dt + (this._ghostGroundInset || 0) / 2;
    var targetZ = p.z + cv.z * dt;
    var dx = targetX - gp.x, dy = targetY - gp.y, dz = targetZ - gp.z;
    var gap = Math.sqrt(dx * dx + dy * dy + dz * dz);
    var gv = this._ghost.linear_velocity;

    // A gap this large is a rebuild/respawn/teleport: beam the ghost instead of chasing.
    var teleportDist = Math.max(this.width, this.height) * 2;
    if (gap > teleportDist) {
        this._ghost.position.set(p.x, p.y + (this._ghostGroundInset || 0) / 2, p.z);
        gv.set(0, 0, 0);
        this._ghostCommandedVel = { x: 0, y: 0, z: 0 };
        return;
    }

    // Knockback signal = (ghost's actual velocity) - (last tick's commanded velocity). Runs during
    // resim too, so reconciliations stay consistent with an authority that applies knockback live.
    this._readGhostKnockback();

    // Drive at the velocity that closes the predicted gap this tick (no cap needed).
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

    this._ghostCommandedVel = { x: gv.x, y: gv.y, z: gv.z }; // baseline for next tick's knockback read
};

/**
 * Knockback speed = the object's closing speed onto the character, gated to only apply above a small
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
    // var mP = this.mass; // only fed the disabled mass-ratio scale below

    var manifolds = world.narrowphase.manifolds.values();
    for (var manifold = manifolds.next(); !manifold.done; manifold = manifolds.next()) {
        var m = manifold.value;
        var other =
            m.bodyA === ghostBody ? m.bodyB :
            m.bodyB === ghostBody ? m.bodyA : null;
        // A player is a wall, not a pushable object — no knockback from another player's ghost.
        if (other && other.bodyType === RigidBody.DYNAMIC && other._mass > 0 && !other.isKinematicCharacter) {
            // var mB = other._mass; // only fed the disabled mass-ratio scale below
            var ov = other.linear_velocity;
            var nx = this._ghost.position.x - other.position.x;
            var nz = this._ghost.position.z - other.position.z;
            var nlen = Math.sqrt(nx * nx + nz * nz);
            if (nlen > FPSC.EPS_LEN) { nx /= nlen; nz /= nlen; } else { nx = 0; nz = 0; }
            // n points box->character. Gate on the BOX's own inbound speed (ov.n), not relative closing
            // speed, so walking into a box doesn't push you back. Opt out via receiveSelfPush.
            var closing = this._receiveSelfPush ?
                (ov.x - pb.x) * nx + (ov.z - pb.z) * nz :   // legacy: relative closing (self-push included)
                ov.x * nx + ov.z * nz;                      // box's own inbound speed only
            if (closing > FPSC.KB_CLOSING_MIN) {
                // `closing` is already post-collision (mass exchange baked in), so it is NOT scaled by
                // the mass ratio again (that double-counted the penalty).
                // var massRatio = mB / (mB + mP);
                // var kbv = massRatio * closing;
                var kbv = closing;
                if (kbv > this._receiveMaxSpeed) { kbv = this._receiveMaxSpeed; }
                kbv *= this._receiveKnockbackFraction;
                // Cap the RESULTING along-n speed, not this tick's increment, so sustained contact
                // can't add another full kb every tick past receiveMaxSpeed.
                var alongN = pb.x * nx + pb.z * nz;
                var room = this._receiveMaxSpeed - alongN;
                if (room > 0) { kbv = Math.min(kbv, room); } else { kbv = 0; }
                if (kbv > FPSC.KB_MIN) {
                    pb.x += nx * kbv;
                    pb.z += nz * kbv;
                    this.grounded = false;
                    // Fix what NEXT tick sees: without this, the next dispatch would read the stale WALK
                    // sub-state and re-clamp the character before knockback got airborne.
                    this._moveState = FPSC.MOVE_AIRBORNE;
                    if (this._groundSuppress < FPSC.GROUND_SUPPRESS_KB) { this._groundSuppress = FPSC.GROUND_SUPPRESS_KB; }
                }
            }
            break;
        }
    }
};
