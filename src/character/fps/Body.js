// Character body lifecycle: creates/recreates the kinematic box collider the character controller
// drives (see _buildBody), and the shared material-application helper used by both the character
// body and its ghost (see Ghost.js).
var proto = FPSCharacterController.prototype;

/**
 * Apply this controller's material/behavior defaults + explicit overrides to a freshly created body.
 * Shared by the character's own body (Body.js) and its ghost (Ghost.js).
 *
 * @method _applyMaterial
 * @private
 * @static
 * @param {RigidBody} body
 * @param {Object} [opts]
 * @param {Number} [opts.friction=3.0]
 * @param {Number} [opts.restitution=0.33]
 * @param {Number} [opts.linearDamping=0.1]
 * @param {Number} [opts.angularDamping=0.9]
 * @param {Vector3} [opts.gravity] - if set, overrides the body's gravity via setGravity.
 */
FPSCharacterController._applyMaterial = function(body, opts) {
    opts = opts || {};
    body.friction = opts.friction !== undefined ? opts.friction : 3.0;
    body.restitution = opts.restitution !== undefined ? opts.restitution : 0.33;
    body.linear_damping = opts.linearDamping !== undefined ? opts.linearDamping : 0.1;
    body.angular_damping = opts.angularDamping !== undefined ? opts.angularDamping : 0.9;
    if (opts.gravity) { body.setGravity(opts.gravity.x, opts.gravity.y, opts.gravity.z); }
};

/**
 * (Re)create the box body at a position, preserving look + velocity where possible.
 * @method _buildBody
 * @private
 * @param {Vector3} position
 */
proto._buildBody = function(position) {
    var carriedVel = null;
    if (this.body) {
        var v = this.body.linear_velocity;
        carriedVel = { x: v.x, y: v.y, z: v.z };
        this.world.removeRigidBody(this.body);
    }
    this._destroyGhost();

    var shape = new BoxShape(this.width / 2, this.height / 2, this.depth / 2);
    this.body = new RigidBody(shape, this.mass);
    FPSCharacterController._applyMaterial(this.body, {});
    this.body.position.copy(position);
    this.body.updateDerived();
    // `object` is the lightweight cosmetic handle a consumer (renderer) can use to decide whether/how
    // to draw the collider. This controller never renders anything itself.
    this.object = { body: this.body, isVisible: this._visible };

    // Never tip; resting/slopes/walls handled by the solver (gravity + friction).
    this.body.angular_factor.set(0, 0, 0);
    this.body.friction = this.friction;
    this.body.restitution = 0;
    this.body.linear_damping = 0;
    this.body.angular_damping = 0;

    // Tag our physics body so raycasts can ignore ourselves. Also ignore the ghost's name so the
    // kinematic body's own probing raycasts never treat its own trailing ghost as a wall.
    this.body.name = this._bodyName;
    this._ignoreSelf = [this._bodyName, this._bodyName + "_ghost"];
    // Mark this as a kinematic character body so OTHER characters' receive-push pass skips it
    // (character-vs-character is already handled by collide-and-slide treating each other as walls;
    // the body-push coupling is only meant for free dynamic objects).
    this.body.isKinematicCharacter = true;

    // Exclude the character from ALL solver contacts: a mask of 0 matches no group, regardless of
    // this body's own collision_groups (RigidBody's default groups/mask is "collide with everything" -
    // the inverse of the "collide with nothing until opted in" convention this exclusion pattern was
    // originally written against, so the exclusion here is an explicit zero mask, not a specific
    // unused group bit). The body still integrates and is still raycast-queryable. Collision is done
    // entirely via raycasts (ground clamp + collide-and-slide), so the solver can never fight the
    // control loop.
    this.body.collision_mask = 0;

    if (carriedVel) { this.body.linear_velocity.set(carriedVel.x, carriedVel.y, carriedVel.z); }

    this.world.addRigidBody(this.body);
    this._buildGhost(position, carriedVel);
};
