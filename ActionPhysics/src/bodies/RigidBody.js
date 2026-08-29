// Body type, as a first-class concept: checked in exactly one place per stage, never as scattered
// mass===Infinity comparisons.
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;

let _nextBodyId = 1;

/**
 * A rigid body: shape + world transform + (for dynamic bodies) mass/motion state. Broadphase and
 * midphase only need shape + transform + AABB; the mass/motion/material fields exist so every
 * concern has exactly one owning home (Mass is owned here, not scattered across whichever stage
 * happens to need it first).
 *
 * Field groups: Identity, Transform, Mass, Motion, Forces, Material, Filtering, Events.
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
        // These four match ActionEngineJS's own ActionRigidBody3D.MATERIAL_DEFAULTS exactly (the
        // shipping game's real material, not a bare/theoretical one) - the consumer already settled
        // on these as what a physics body should default to, and a body constructed directly against
        // this engine (bypassing the ActionEngineJS wrapper - the test suite, or any other direct
        // caller) should behave the same way as one built through it, not silently softer/bouncier/
        // less-damped. angular_damping is not 0: ordinary Coulomb friction cannot stop a ROLLING
        // contact on its own (it opposes tangential SLIP at the contact point, and a cleanly rolling
        // round shape has ~zero slip there by construction - see rolling_friction below, and
        // Solver._solveRollingResistance), so without angular damping a cylinder/capsule/ball that
        // picks up any spin on landing keeps rolling indefinitely no matter how high friction is.
        this.friction = 3.0;
        this.restitution = 0.33;
        this.linear_damping = 0.1;
        this.angular_damping = 0.9;
        // Rolling resistance coefficient (metres): opposes a round shape's spin AT a contact by
        // capping the angular velocity component about the contact's tangent plane, the same way
        // Coulomb friction caps tangential slip - see Solver._solveContactVelocity's rolling pass.
        // Was 0 (matching Goblin's own RigidBody.rolling_friction default and ActionEngineJS's
        // MATERIAL_DEFAULTS, which has no rollingFriction field at all) - relying on angular_damping
        // alone left round shapes (cylinder/cone/convex) with a small residual spin that never fully
        // decayed to rest. Set to 0.02 per explicit instruction. Known tradeoff, not yet resolved:
        // this value only partially converges the round-shape residual-spin tests (cone/cylinder
        // still fall short of their 0.05 rad/s rest threshold) and destabilizes two previously-solid
        // box-on-box stacking tests (box stacking's own corner/edge contacts use this same mechanism).
        this.rolling_friction = 0.02;

        // ---- Filtering ----
        this.collision_mask = 0xFFFFFFFF;
        this.collision_groups = 1;

        // ---- Events ----
        this._listeners = {};

        // Sleep state, owned entirely by the sleep manager. Present here only as the state a body
        // carries; no other stage writes to it.
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

    // ---- Forces ----
    //
    // An IMPULSE is an instantaneous velocity change (a bat hitting a ball) - applied directly to
    // linear_velocity/angular_velocity right now. Every impulse call in this file reduces to:
    // dv = j * massInverted, dw = I^-1 * (r x j).
    //
    // A FORCE/TORQUE is continuous (thrust, a constant push) - it accumulates into
    // accumulated_force/accumulated_torque, is integrated by the solver once per SUBSTEP
    // (Solver._substep, alongside gravity), and is cleared once per TICK (World.step): forces are
    // re-applied every tick by whoever wants them to keep acting.
    applyImpulse(impulse) {
        if (this._massInverted <= 0) return this;
        this.linear_velocity.x += impulse.x * this._massInverted * this.linear_factor.x;
        this.linear_velocity.y += impulse.y * this._massInverted * this.linear_factor.y;
        this.linear_velocity.z += impulse.z * this._massInverted * this.linear_factor.z;
        return this;
    }

    // Impulse applied at a world-space point (not the body's center) - produces both a linear
    // velocity change AND an angular one (dw = I^-1 * (r x impulse)), same generalized-impulse shape
    // the solver's own _applyVelocityImpulse uses for contacts, exposed here for a caller (a game's
    // hit-react, an explosion) that wants an off-center push.
    applyImpulseAtPoint(impulse, worldPoint) {
        if (this._massInverted <= 0) return this;
        this.applyImpulse(impulse);
        const rx = worldPoint.x - this.position.x, ry = worldPoint.y - this.position.y, rz = worldPoint.z - this.position.z;
        const tqx = ry * impulse.z - rz * impulse.y, tqy = rz * impulse.x - rx * impulse.z, tqz = rx * impulse.y - ry * impulse.x;
        const I = this._worldInverseInertiaTensor;
        this.angular_velocity.x += (I.e00 * tqx + I.e01 * tqy + I.e02 * tqz) * this.angular_factor.x;
        this.angular_velocity.y += (I.e10 * tqx + I.e11 * tqy + I.e12 * tqz) * this.angular_factor.y;
        this.angular_velocity.z += (I.e20 * tqx + I.e21 * tqy + I.e22 * tqz) * this.angular_factor.z;
        return this;
    }

    applyTorqueImpulse(torqueImpulse) {
        if (this._massInverted <= 0) return this;
        const I = this._worldInverseInertiaTensor;
        const tx = torqueImpulse.x, ty = torqueImpulse.y, tz = torqueImpulse.z;
        this.angular_velocity.x += (I.e00 * tx + I.e01 * ty + I.e02 * tz) * this.angular_factor.x;
        this.angular_velocity.y += (I.e10 * tx + I.e11 * ty + I.e12 * tz) * this.angular_factor.y;
        this.angular_velocity.z += (I.e20 * tx + I.e21 * ty + I.e22 * tz) * this.angular_factor.z;
        return this;
    }

    // Accumulates a CONTINUOUS force, integrated by the solver every substep until cleared. Adds,
    // does not overwrite - multiple applyForce calls in the same tick (gravity plus thrust plus wind)
    // all contribute, matching accumulated_force's own name.
    applyForce(force) {
        this.accumulated_force.x += force.x;
        this.accumulated_force.y += force.y;
        this.accumulated_force.z += force.z;
        return this;
    }

    applyTorque(torque) {
        this.accumulated_torque.x += torque.x;
        this.accumulated_torque.y += torque.y;
        this.accumulated_torque.z += torque.z;
        return this;
    }

    // A force applied at a world-space point contributes both the force itself AND the torque it
    // produces about the center (r x force) - the continuous-force analogue of applyImpulseAtPoint.
    applyForceAtPoint(force, worldPoint) {
        this.applyForce(force);
        const rx = worldPoint.x - this.position.x, ry = worldPoint.y - this.position.y, rz = worldPoint.z - this.position.z;
        this.accumulated_torque.x += ry * force.z - rz * force.y;
        this.accumulated_torque.y += rz * force.x - rx * force.z;
        this.accumulated_torque.z += rx * force.y - ry * force.x;
        return this;
    }

    // Zeroes accumulated_force/torque. Called by World.step once per TICK (not per substep - a
    // continuous force stays in effect for every substep within the tick it was applied, then a
    // caller who wants it to keep acting must call applyForce again next tick).
    clearForces() {
        this.accumulated_force.set(0, 0, 0);
        this.accumulated_torque.set(0, 0, 0);
        return this;
    }

    // Refresh everything derived from position/rotation: the world AABB (tight and broadphase
    // variants) and the world-space inverse inertia tensor. Called once per body per tick by
    // whichever stage owns "current" - narrowphase and the solver assume it has already run (Rule
    // 1: stage contracts are absolute).
    //
    // `dt` (optional) is this tick's timestep, used only to size the broadphase AABB's velocity
    // sweep (see _recomputeBroadphaseAABB). The tight AABB (getAABB) never depends on dt.
    updateDerived(dt) {
        this._recomputeAABB();
        this._recomputeBroadphaseAABB(dt || 0);
        this._recomputeWorldInverseInertia();
        return this;
    }

    // The TIGHT world AABB: the exact rotated bound of the shape at the current transform, no
    // margin. This is the body's geometric truth - what a raycast/query wants, and what getAABB()
    // returns. Broadphase uses the fattened variant below instead (getBroadphaseAABB).
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

    // The BROADPHASE world AABB: the tight AABB fattened for speculative contacts by a fixed margin
    // plus this tick's velocity sweep on each axis, so a fast approach is caught a full tick BEFORE
    // the shapes actually overlap. That lookahead is what makes speculative contacts possible at
    // all: narrowphase can only create a pre-overlap (still-separated) contact point for a pair
    // broadphase actually reports, and a raw tight AABB doesn't overlap until the shapes already do
    // (by which time the body has fallen straight through the speculative window). Fattening only
    // ever ADDS candidate pairs, never removes one (Rule: broadphase no-false-negatives) -
    // narrowphase then culls precisely with its own per-pair margin. Kept SEPARATE from the tight
    // AABB so the body's geometric bound stays truthful for queries/rendering.
    //
    // Sweep is directional (grow the box only on the side the body is moving toward on each axis),
    // keeping the fattened box tight rather than symmetric - a body moving down grows its box
    // downward, not upward. SPECULATIVE_MARGIN is the absolute floor for the resting/slow case
    // where velocity*dt alone is ~0.
    _recomputeBroadphaseAABB(dt) {
        const m = RigidBody.SPECULATIVE_MARGIN;
        const sx = this.linear_velocity.x * dt, sy = this.linear_velocity.y * dt, sz = this.linear_velocity.z * dt;
        // Angular sweep: a point at the body's bounding radius R moves at up to |omega|*R, so a
        // spinning body's far corner sweeps that far this tick even when the CENTER (which the
        // linear sweep above tracks) barely moves. Missing this was a real bug: a box that tips
        // onto one corner spins up to a few rad/s, its OPPOSITE corner then approaches the ground
        // at |omega|*R (a couple of m/s) with the center's linear velocity pointing elsewhere, so
        // the linear sweep never grew the box toward that corner - the corner slammed in undetected
        // and the one-shot correction of the resulting deep overlap injected more spin, diverging.
        // Angular motion has no clean per-axis direction, so this term is applied ISOTROPICALLY
        // (all six faces) - the conservative honest choice, never an under-estimate. R is taken from
        // the tight AABB's own half-extent (its farthest corner from center).
        const ex = (this._aabb.max.x - this._aabb.min.x) * 0.5;
        const ey = (this._aabb.max.y - this._aabb.min.y) * 0.5;
        const ez = (this._aabb.max.z - this._aabb.min.z) * 0.5;
        const R = Math.sqrt(ex * ex + ey * ey + ez * ez);
        const wMag = Math.sqrt(this.angular_velocity.x * this.angular_velocity.x +
            this.angular_velocity.y * this.angular_velocity.y + this.angular_velocity.z * this.angular_velocity.z);
        const a = wMag * R * dt;
        this._broadphaseAABB.min.x = this._aabb.min.x - m - a - (sx < 0 ? -sx : 0);
        this._broadphaseAABB.max.x = this._aabb.max.x + m + a + (sx > 0 ? sx : 0);
        this._broadphaseAABB.min.y = this._aabb.min.y - m - a - (sy < 0 ? -sy : 0);
        this._broadphaseAABB.max.y = this._aabb.max.y + m + a + (sy > 0 ? sy : 0);
        this._broadphaseAABB.min.z = this._aabb.min.z - m - a - (sz < 0 ? -sz : 0);
        this._broadphaseAABB.max.z = this._aabb.max.z + m + a + (sz > 0 ? sz : 0);
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

    // ActionMath's Transform (position + rotation + scale, with transformPoint/transformVector
    // convenience methods) synced from this body's own position/rotation. A physics body has no
    // scale (XPBD solves position and orientation directly - see the class header - scale is a
    // rendering-only concept with no physical meaning for a rigid body), so Transform's own scale
    // field is simply left at its default (1,1,1) here, never read or written by anything in this
    // engine. This does NOT replace position/rotation as this body's own state (every solver/
    // narrowphase/query call site reads those fields directly, not through a Transform indirection -
    // changing that would touch hundreds of call sites for no behavioral benefit); it exists only
    // for CONSUMER code (tests, queries) that wants Transform's own API without
    // duplicating its rotate-then-translate math. Lazily allocated once, then reused and re-synced
    // on every call - allocation-free after the first call, matching this file's own discipline for
    // every other derived accessor (getAABB, getBroadphaseAABB).
    getTransform() {
        if (!this._transform) this._transform = new Transform();
        this._transform.syncFromPhysicsBody(this);
        return this._transform;
    }

    // World-space support point: the farthest point on this body's shape along world-space
    // `direction`, written into `out`. Same composition MinkowskiSupport.supportOfInto already uses
    // internally for GJK/EPA (inverse-rotate direction into local space, call the shape's own
    // supportInto, rotate the result back to world space, translate by position) - exposed here as a
    // standalone body method since a caller outside narrowphase (a query, a consumer) has no reason
    // to construct a MinkowskiSupport (which pairs two bodies) just to ask one body's own question.
    findSupportPoint(direction, out) {
        const scratchDir = RigidBody._scratchSupportDir;
        RigidBody._scratchInvRot.copy(this.rotation).invert();
        RigidBody._scratchInvRot.transformVectorInto(direction, scratchDir);
        this.shape.supportInto(out, scratchDir);
        this.rotation.transformVectorInPlace(out);
        out.addInPlace(this.position);
        return out;
    }

    // rayIntersect(start, end) -> { point, normal, distance, fraction } | null. Casts against THIS
    // body alone (see Queries.rayIntersectBody) - for a caller that already holds a body reference
    // and wants a hit test against just that one shape, without World.rayIntersect's whole-scene
    // candidate search.
    rayIntersect(start, end) {
        return Queries.rayIntersectBody(start, end, this);
    }

    // The fattened broadphase-query AABB (tight bound + speculative margin + velocity sweep).
    // Broadphase and midphase read THIS, not getAABB(), so a pair surfaces the tick before overlap
    // (see _recomputeBroadphaseAABB). Same staleness assumption as getAABB(): updateDerived() owns
    // recomputing it once per tick.
    getBroadphaseAABB() {
        return this._broadphaseAABB;
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

    // Runs this body's speculativeContact listeners and returns false if any of them vetoes the
    // point (returns false). `other` is the body on the far side of the contact.
    _speculativeVeto(contact, other) {
        const list = this._listeners.speculativeContact;
        if (!list) return true;
        for (let i = 0; i < list.length; i++) {
            if (list[i]({ contact: contact, other: other }) === false) return false;
        }
        return true;
    }
}

// Scratch objects for the allocation-free AABB/inertia recompute above. Private to RigidBody's
// own derived-state recompute, never shared across unrelated algorithms.
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
// speculative base so the two stages agree on "how early is a contact worth seeing": broadphase
// surfaces the pair at least this far before overlap, and narrowphase then creates the actual
// pre-overlap point within its own (equal-or-larger, velocity-widened) margin. Kept as an
// absolute floor here; the velocity sweep in _recomputeAABB handles fast approaches on top of it.
RigidBody.SPECULATIVE_MARGIN = 0.02;

ActionPhysics.RigidBody = RigidBody;
