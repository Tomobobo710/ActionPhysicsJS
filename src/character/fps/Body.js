// Body lifecycle: (re)builds the kinematic box collider the controller drives, and the shared
// material-application helper used by both the character body and its ghost (Ghost.js).
var proto = FPSCharacterController.prototype;

/**
 * Apply material/behavior defaults + overrides to a body. Shared by the body and its ghost.
 * @static
 * @param {RigidBody} body
 * @param {Object} [opts] - friction/restitution/linearDamping/angularDamping/gravity overrides.
 */
FPSCharacterController._applyMaterial = function(body, opts) {
    opts = opts || {};
    body.friction = opts.friction !== undefined ? opts.friction : 3.0;
    body.restitution = opts.restitution !== undefined ? opts.restitution : 0.33;
    body.linear_damping = opts.linearDamping !== undefined ? opts.linearDamping : 0.1;
    body.angular_damping = opts.angularDamping !== undefined ? opts.angularDamping : 0.9;
    if (opts.gravity) { body.setGravity(opts.gravity.x, opts.gravity.y, opts.gravity.z); }
};

/** (Re)create the box body at a position, preserving velocity where possible. */
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
    // `object` is the cosmetic handle a consumer (renderer) can use; this controller never renders.
    this.object = { body: this.body, isVisible: this._visible };
    // Color tag consumed by the test harness's renderer, not by the engine itself.
    this.body._color = this._color;

    // Never tip; resting/slopes/walls handled by the solver (gravity + friction).
    this.body.angular_factor.set(0, 0, 0);
    this.body.friction = this.friction;
    this.body.restitution = 0;
    this.body.linear_damping = 0;
    this.body.angular_damping = 0;

    // Tag our physics body so raycasts ignore ourselves, plus the ghost's name so our own probing
    // rays never treat the trailing ghost as a wall.
    this.body.name = this._bodyName;
    this._ignoreSelf = [this._bodyName, this._bodyName + "_ghost"];
    // Mark as a kinematic character body so other characters' receive-push pass skips it.
    this.body.isKinematicCharacter = true;

    // Exclude the character from all solver contacts (zero mask): collision is done entirely via
    // raycasts, so the solver can't fight the control loop. Still integrates and is raycast-queryable.
    this.body.collision_mask = 0;

    if (carriedVel) { this.body.linear_velocity.set(carriedVel.x, carriedVel.y, carriedVel.z); }

    this.world.addRigidBody(this.body);
    this._buildGhost(position, carriedVel);
};
