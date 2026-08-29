// Body type, as a first-class concept (plan.md: "What mature engines have that Goblin doesn't" #2).
// Checked in exactly one place per stage, never as scattered mass===Infinity comparisons.
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;

let _nextBodyId = 1;

/**
 * A rigid body: shape + world transform + (for dynamic bodies) mass/motion state. Broadphase and
 * midphase only need shape + transform + AABB; the mass/motion/material fields exist now rather
 * than being bolted on at the solver stage, so this class is not rebuilt twice (plan.md's "one
 * owner per concern" applies to the body's own field layout too - Mass is owned here, not
 * scattered across whichever stage happens to need it first).
 *
 * Field groups match the API surface table in plan.md: Transform, Motion, Forces, Material, Mass,
 * Filtering, Identity.
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
        this._aabb = new AABB();
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
        this.friction = 0.5;
        this.restitution = 0;
        this.linear_damping = 0;
        this.angular_damping = 0;

        // ---- Filtering ----
        this.collision_mask = 0xFFFFFFFF;
        this.collision_groups = 1;

        // ---- Events ----
        this._listeners = {};

        // Sleep (owned entirely by the sleep manager once it exists - plan.md, Sleep). Present
        // here only as the state a body carries; no stage but the sleep manager writes to it.
        this.isAwake = true;
        this.sleepTimer = 0;
    }

    get is_static() { return this.bodyType === RigidBody.STATIC; }
    get mass() { return this._mass; }

    // Scales the shape's density-1 inertia by (mass / shape.volume()), per Shape's contract
    // (src/shapes/Shape.js) — computeMassData() always returns density-1 values.
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

    // Refresh everything derived from position/rotation: the world AABB and the world-space
    // inverse inertia tensor. Called once per body per tick by whichever stage owns "current" -
    // narrowphase and the solver assume it has already run (Rule 1: stage contracts are absolute).
    updateDerived() {
        this._recomputeAABB();
        this._recomputeWorldInverseInertia();
        return this;
    }

    _recomputeAABB() {
        const local = RigidBody._scratchLocalAABB;
        this.shape.localAABBInto(local);
        // Conservative rotated bound via the 8-corner sweep, same technique CompoundShape uses for
        // its own children - correct for any rotation, not just axis-aligned ones.
        const rotMat = RigidBody._scratchMat3;
        rotMat.fromQuaternion(this.rotation);
        const corner = RigidBody._scratchVec;
        this._aabb.setEmpty();
        for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
            corner.x = cx ? local.max.x : local.min.x;
            corner.y = cy ? local.max.y : local.min.y;
            corner.z = cz ? local.max.z : local.min.z;
            rotMat.transformVector3(corner);
            corner.addInPlace(this.position);
            if (corner.x < this._aabb.min.x) this._aabb.min.x = corner.x;
            if (corner.y < this._aabb.min.y) this._aabb.min.y = corner.y;
            if (corner.z < this._aabb.min.z) this._aabb.min.z = corner.z;
            if (corner.x > this._aabb.max.x) this._aabb.max.x = corner.x;
            if (corner.y > this._aabb.max.y) this._aabb.max.y = corner.y;
            if (corner.z > this._aabb.max.z) this._aabb.max.z = corner.z;
        }
        this._aabbDirty = false;
    }

    _recomputeWorldInverseInertia() {
        if (this._massInverted === 0) { this._worldInverseInertiaTensor.zero(); return; }
        const rotMat = RigidBody._scratchMat3;
        rotMat.fromQuaternion(this.rotation);
        const rotT = RigidBody._scratchMat3b;
        rotT.transposeInto(rotMat);
        this._worldInverseInertiaTensor.multiplyFrom(rotMat, this.inverseInertiaTensor);
        this._worldInverseInertiaTensor.multiply(rotT);
    }

    // Broadphase's required primitive: the body's CURRENT world AABB. Assumes updateDerived() has
    // already run this tick (Rule 1) - getAABB() never recomputes on its own, so a stale call is a
    // caller bug surfaced as a stale box, not silently patched over here.
    getAABB() {
        return this._aabb;
    }

    addListener(event, fn) {
        (this._listeners[event] || (this._listeners[event] = [])).push(fn);
        return this;
    }

    emit(event, arg) {
        const list = this._listeners[event];
        if (!list) return;
        for (let i = 0; i < list.length; i++) list[i](arg);
    }
}

// Scratch objects for the allocation-free AABB/inertia recompute above. Per-class, not shared
// across unrelated algorithms (plan.md: "scratch memory: per-stage arenas, never global") - these
// three are private to RigidBody's own derived-state recompute and touched nowhere else.
RigidBody._scratchLocalAABB = new AABB();
RigidBody._scratchMat3 = new Matrix3();
RigidBody._scratchMat3b = new Matrix3();
RigidBody._scratchVec = new Vector3();

RigidBody.STATIC = BODY_STATIC;
RigidBody.KINEMATIC = BODY_KINEMATIC;
RigidBody.DYNAMIC = BODY_DYNAMIC;

ActionPhysics.RigidBody = RigidBody;
