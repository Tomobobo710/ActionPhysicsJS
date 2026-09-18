const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;

let _nextBodyId = 1;

class RigidBody {

    constructor(shape, mass, options) {
        const kinematic = !!(options && options.kinematic);

        this.id = _nextBodyId++;
        this.shape = shape;
        this.debugName = null;
        this.world = null;
        this.bodyType = kinematic ? BODY_KINEMATIC : (mass > 0 ? BODY_DYNAMIC : BODY_STATIC);

        this.position = new Vector3(0, 0, 0);
        this.rotation = new Quaternion(0, 0, 0, 1);
        this._aabb = new AABB();
        this._broadphaseAABB = new AABB();
        this._aabbDirty = true;

        this._mass = kinematic ? 0 : (mass || 0);
        this._mass_inverted = this._mass > 0 ? 1 / this._mass : 0;
        this.inertiaTensor = new Matrix3();
        this.inverseInertiaTensor = new Matrix3();
        this._worldInverseInertiaTensor = new Matrix3();
        if (shape && this._mass > 0) this.setMassFromShape(shape, this._mass);

        this.linear_velocity = new Vector3(0, 0, 0);
        this.angular_velocity = new Vector3(0, 0, 0);
        this.linear_factor = new Vector3(1, 1, 1);
        this.angular_factor = new Vector3(1, 1, 1);

        this.accumulated_force = new Vector3(0, 0, 0);
        this.accumulated_torque = new Vector3(0, 0, 0);
        this.gravity = null;

        this.friction = 3.0;
        this.restitution = 0.33;
        this.linear_damping = 0.1;
        this.angular_damping = 0.9;

        this.angular_friction = 0.05;

        this.collision_mask = 0xFFFFFFFF;
        this.collision_groups = 1;

        this._listeners = {};

        this.isAwake = true;
        this.sleepTimer = 0;
    }

    get is_static() { return this.bodyType === RigidBody.STATIC; }
    get mass() { return this._mass; }

    set mass(value) {
        const m = (value === Infinity || !(value > 0)) ? 0 : value;
        this.setMassFromShape(this.shape, m);
        if (this.bodyType !== RigidBody.KINEMATIC) {
            this.bodyType = m > 0 ? RigidBody.DYNAMIC : RigidBody.STATIC;
        }
    }

    get acceleration() {
        const a = this._acceleration || (this._acceleration = new Vector3());
        const mi = this._mass_inverted;
        a.x = this.accumulated_force.x * mi;
        a.y = this.accumulated_force.y * mi;
        a.z = this.accumulated_force.z * mi;
        return a;
    }

    get aabb() { return this._aabb; }

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

        if (!this.isAwake) this.wakeUpFromWorldChange();
        return this;
    }

    sleep() {
        if (this.bodyType !== BODY_DYNAMIC || !this.isAwake) return this;
        this.isAwake = false;
        this.sleepTimer = 0;
        this.linear_velocity.set(0, 0, 0);
        this.angular_velocity.set(0, 0, 0);
        return this;
    }    wakeUp() {
        if (this.bodyType !== BODY_DYNAMIC) return this;
        if (!this.isAwake) this._restRingStale = true;
        this.isAwake = true;
        this.sleepTimer = 0;
        return this;
    }

    wakeUpFromWorldChange() {
        if (this.bodyType !== BODY_DYNAMIC) return this;
        this.wakeUp();
        this._restRingStale = true;
        return this;
    }

}

RigidBody._scratchLocalAABB = new AABB();
RigidBody._scratchMat3 = new Matrix3();
RigidBody._scratchMat3b = new Matrix3();
RigidBody._scratchVec = new Vector3();
RigidBody._scratchInvRot = new Quaternion();
RigidBody._scratchSupportDir = new Vector3();
RigidBody._scratchForcePoint = new Vector3();

RigidBody.STATIC = BODY_STATIC;
RigidBody.KINEMATIC = BODY_KINEMATIC;
RigidBody.DYNAMIC = BODY_DYNAMIC;

RigidBody.SPECULATIVE_MARGIN = 0.02;

ActionPhysics.RigidBody = RigidBody;
