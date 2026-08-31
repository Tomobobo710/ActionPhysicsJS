const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;

let _nextBodyId = 1;

// Shape + world transform + (for dynamic bodies) mass/motion state. See Forces.js, DerivedState.js,
// Accessors.js.
class RigidBody {
    // new RigidBody(shape, mass) - mass > 0 makes a DYNAMIC body, mass <= 0 a STATIC one.
    // new RigidBody(shape, mass, { kinematic: true }) makes a KINEMATIC body: infinite effective
    // mass (contacts never move it, forces/impulses are ignored), but the integrator advances its
    // position from linear_velocity and its rotation from angular_velocity every tick, and a direct
    // position/rotation write is honoured. Drive it by setting its velocity (or writing its
    // transform) each tick - moving platforms, elevators, doors.
    constructor(shape, mass, options) {
        const kinematic = !!(options && options.kinematic);

        // ---- Identity ----
        this.id = _nextBodyId++;
        this.shape = shape;
        this.debugName = null;
        this.world = null; // set by World.addRigidBody
        this.bodyType = kinematic ? BODY_KINEMATIC : (mass > 0 ? BODY_DYNAMIC : BODY_STATIC);

        // ---- Transform ----
        this.position = new Vector3(0, 0, 0);
        this.rotation = new Quaternion(0, 0, 0, 1);
        this._aabb = new AABB();            // tight geometric bound (getAABB)
        this._broadphaseAABB = new AABB();  // fattened for speculative contacts (getBroadphaseAABB)
        this._aabbDirty = true;

        // ---- Mass ----
        // A KINEMATIC or mass<=0 body has infinite effective mass: zero inverse mass, zero inertia.
        // A KINEMATIC body still moves - the integrator drives its transform from its velocity - it
        // just does not RESPOND to contacts or forces.
        this._mass = kinematic ? 0 : (mass || 0);
        this._mass_inverted = this._mass > 0 ? 1 / this._mass : 0;
        this.inertiaTensor = new Matrix3();       // local-space, set by setMassFromShape()
        this.inverseInertiaTensor = new Matrix3(); // local-space inverse
        this._worldInverseInertiaTensor = new Matrix3(); // R * I^-1_local * R^T, refreshed by updateDerived()
        if (shape && this._mass > 0) this.setMassFromShape(shape, this._mass);

        // ---- Motion ----
        this.linear_velocity = new Vector3(0, 0, 0);
        this.angular_velocity = new Vector3(0, 0, 0);
        this.linear_factor = new Vector3(1, 1, 1);   // per-axis velocity mask, e.g. lock an axis with 0
        this.angular_factor = new Vector3(1, 1, 1);

        // ---- Forces ----
        this.accumulated_force = new Vector3(0, 0, 0);
        this.accumulated_torque = new Vector3(0, 0, 0);
        this.gravity = null; // null = use World.gravity; setGravity() overrides per-body

        // ---- Material ---- (matches ActionEngineJS's MATERIAL_DEFAULTS)
        this.friction = 3.0;
        this.restitution = 0.33;
        this.linear_damping = 0.1;
        this.angular_damping = 0.9; // nonzero, or a cleanly rolling shape never stops on friction alone
        // Caps relative angular velocity in the contact's tangent plane, like friction caps slip.
        this.angular_friction = 0.05;

        // ---- Filtering ----
        this.collision_mask = 0xFFFFFFFF;
        this.collision_groups = 1;

        // ---- Events ----
        this._listeners = {};

        // Sleep state, owned entirely by the sleep manager.
        this.isAwake = true;
        this.sleepTimer = 0;
        // Set by the solver when a moving body pushes on this one; consumed by the rest-pin logic in
        // Solver._reconcileRestVelocity to release a pinned body the tick it is disturbed.
        this._restDisturbed = false;
    }

    get is_static() { return this.bodyType === RigidBody.STATIC; }
    get mass() { return this._mass; }

    // Assigning mass re-derives the inertia tensor from the current shape. mass <= 0 (including
    // Infinity, treated as "no dynamics") makes the body STATIC; a positive mass makes it DYNAMIC.
    // A KINEMATIC body's type is not changed by a mass write (it is code-driven regardless).
    set mass(value) {
        const m = (value === Infinity || !(value > 0)) ? 0 : value;
        this.setMassFromShape(this.shape, m);
        if (this.bodyType !== RigidBody.KINEMATIC) {
            this.bodyType = m > 0 ? RigidBody.DYNAMIC : RigidBody.STATIC;
        }
    }

    // This tick's linear acceleration from applied force: F * m^-1. Zero for a body with infinite
    // effective mass (static/kinematic). Read it after applying forces and before step() for "how
    // hard is this being pushed"; it is not a stored integration value (XPBD has no 'a' term), it
    // is recomputed into a per-body vector on each read.
    get acceleration() {
        const a = this._acceleration || (this._acceleration = new Vector3());
        const mi = this._mass_inverted;
        a.x = this.accumulated_force.x * mi;
        a.y = this.accumulated_force.y * mi;
        a.z = this.accumulated_force.z * mi;
        return a;
    }

    // The tight world AABB (same object getAABB() returns). Assumes updateDerived() has run this
    // tick - a stale read is a caller bug, not patched over here.
    get aabb() { return this._aabb; }

    // Sets mass and re-derives the local inertia tensor from the shape (Shape.getInertiaTensor does
    // the density-1 -> mass scaling). mass <= 0 gives the zero tensor (infinite effective mass).
    setMassFromShape(shape, mass) {
        this._mass = mass;
        this._mass_inverted = mass > 0 ? 1 / mass : 0;
        if (mass <= 0) {
            this.inertiaTensor.zero();
            this.inverseInertiaTensor.zero();
            return;
        }
        this.inertiaTensor.copy(shape.getInertiaTensor(mass));
        this.inverseInertiaTensor.invertInto(this.inertiaTensor);
    }

    setGravity(x, y, z) {
        this.gravity = new Vector3(x, y, z);
        return this;
    }

    // Park this body: the solver skips it until something wakes it. A sleeping body holds still by
    // definition, so its velocity is zeroed here. No-op for non-dynamic bodies (they are never awake
    // in the sleep sense) and for an already-sleeping body.
    sleep() {
        if (this.bodyType !== BODY_DYNAMIC || !this.isAwake) return this;
        this.isAwake = false;
        this.sleepTimer = 0;
        this.linear_velocity.set(0, 0, 0);
        this.angular_velocity.set(0, 0, 0);
        return this;
    }

    // Wake this body and restart its sleep countdown. Called by the sleep manager when an island is
    // disturbed, and by the force/impulse API so a push on a sleeping body takes effect.
    wakeUp() {
        if (this.bodyType !== BODY_DYNAMIC) return this;
        this.isAwake = true;
        this.sleepTimer = 0;
        return this;
    }
}

// Scratch for DerivedState.js's allocation-free recompute.
RigidBody._scratchLocalAABB = new AABB();
RigidBody._scratchMat3 = new Matrix3();
RigidBody._scratchMat3b = new Matrix3();
RigidBody._scratchVec = new Vector3();
RigidBody._scratchInvRot = new Quaternion();
RigidBody._scratchSupportDir = new Vector3();
RigidBody._scratchForcePoint = new Vector3(); // Forces.js applyForceAtLocalPoint

RigidBody.STATIC = BODY_STATIC;
RigidBody.KINEMATIC = BODY_KINEMATIC;
RigidBody.DYNAMIC = BODY_DYNAMIC;

RigidBody.SPECULATIVE_MARGIN = 0.02; // meters; matches NarrowPhase.SPECULATIVE_BASE

ActionPhysics.RigidBody = RigidBody;
