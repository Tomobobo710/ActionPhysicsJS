// Body type, as a first-class concept: checked in exactly one place per stage, never as scattered
// mass===Infinity comparisons.
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;

let _nextBodyId = 1;

/**
 * A rigid body: shape + world transform + (for dynamic bodies) mass/motion state. Broadphase and
 * midphase only need shape + transform + AABB; mass/motion/material fields exist so every concern
 * has exactly one owning home.
 *
 * See Forces.js (impulse/force application), DerivedState.js (AABB/inertia recompute), and
 * Accessors.js (support point, ray cast, transform, listeners).
 */
class RigidBody {
    constructor(shape, mass) {
        // ---- Identity ----
        this.id = _nextBodyId++;
        this.shape = shape;
        this.debugName = null;
        this.world = null; // set by World.addRigidBody
        this.bodyType = mass > 0 ? BODY_DYNAMIC : BODY_STATIC;

        // ---- Transform ----
        this.position = new Vector3(0, 0, 0);
        this.rotation = new Quaternion(0, 0, 0, 1);
        this._aabb = new AABB();            // tight geometric bound (getAABB)
        this._broadphaseAABB = new AABB();  // fattened for speculative contacts (getBroadphaseAABB)
        this._aabbDirty = true;

        // ---- Mass ----
        // mass===0 is a static/kinematic body: infinite effective mass, zero inverse.
        this._mass = mass || 0;
        this._massInverted = this._mass > 0 ? 1 / this._mass : 0;
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

        // ---- Material ----
        // Matches ActionEngineJS's own ActionRigidBody3D.MATERIAL_DEFAULTS - a body built directly
        // against this engine should behave the same as one built through that wrapper.
        // angular_damping is not 0: Coulomb friction opposes tangential SLIP, and a cleanly rolling
        // shape has ~zero slip at the contact by construction, so without angular damping a rolling
        // body never stops on friction alone (see rolling_friction below and VelocitySolve.js).
        this.friction = 3.0;
        this.restitution = 0.33;
        this.linear_damping = 0.1;
        this.angular_damping = 0.9;
        // Rolling resistance coefficient (metres): caps a round shape's angular velocity about the
        // contact tangent plane, the same way Coulomb friction caps tangential slip.
        this.rolling_friction = 0.05;

        // ---- Filtering ----
        this.collision_mask = 0xFFFFFFFF;
        this.collision_groups = 1;

        // ---- Events ----
        this._listeners = {};

        // Sleep state, owned entirely by the sleep manager.
        this.isAwake = true;
        this.sleepTimer = 0;
    }

    get is_static() { return this.bodyType === RigidBody.STATIC; }
    get mass() { return this._mass; }

    // Scales the shape's density-1 inertia by (mass / shape.volume()), per Shape's contract -
    // computeMassData() always returns density-1 values.
    setMassFromShape(shape, mass) {
        this._mass = mass;
        this._massInverted = mass > 0 ? 1 / mass : 0;
        if (mass <= 0) {
            this.inertiaTensor.zero();
            this.inverseInertiaTensor.zero();
            return;
        }
        const data = shape.computeMassData();
        const volume = shape.volume();
        const scale = volume > 0 ? mass / volume : 0;
        this.inertiaTensor.copy(data.inertia);
        this.inertiaTensor.e00 *= scale; this.inertiaTensor.e01 *= scale; this.inertiaTensor.e02 *= scale;
        this.inertiaTensor.e10 *= scale; this.inertiaTensor.e11 *= scale; this.inertiaTensor.e12 *= scale;
        this.inertiaTensor.e20 *= scale; this.inertiaTensor.e21 *= scale; this.inertiaTensor.e22 *= scale;
        this.inverseInertiaTensor.invertInto(this.inertiaTensor);
    }

    setGravity(x, y, z) {
        this.gravity = new Vector3(x, y, z);
        return this;
    }
}

// Scratch objects for the allocation-free AABB/inertia recompute in DerivedState.js.
RigidBody._scratchLocalAABB = new AABB();
RigidBody._scratchMat3 = new Matrix3();
RigidBody._scratchMat3b = new Matrix3();
RigidBody._scratchVec = new Vector3();
RigidBody._scratchInvRot = new Quaternion();
RigidBody._scratchSupportDir = new Vector3();

RigidBody.STATIC = BODY_STATIC;
RigidBody.KINEMATIC = BODY_KINEMATIC;
RigidBody.DYNAMIC = BODY_DYNAMIC;

// Fixed broadphase-AABB fattening for speculative contacts (metres). Matches the narrowphase
// speculative base so both stages agree on how early a contact is worth seeing.
RigidBody.SPECULATIVE_MARGIN = 0.02;

ActionPhysics.RigidBody = RigidBody;
