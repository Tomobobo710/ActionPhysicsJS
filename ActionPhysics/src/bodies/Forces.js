// Impulse (instantaneous velocity change) and force/torque (continuous, integrated per-substep,
// cleared once per tick) application.
var proto = RigidBody.prototype;

proto.applyImpulse = function (impulse) {
    if (this._massInverted <= 0) return this;
    this.linear_velocity.x += impulse.x * this._massInverted * this.linear_factor.x;
    this.linear_velocity.y += impulse.y * this._massInverted * this.linear_factor.y;
    this.linear_velocity.z += impulse.z * this._massInverted * this.linear_factor.z;
    return this;
};

// Impulse at a world-space point: linear change plus the angular change it produces about the
// center (dw = I^-1 * (r x impulse)).
proto.applyImpulseAtPoint = function (impulse, worldPoint) {
    if (this._massInverted <= 0) return this;
    this.applyImpulse(impulse);
    const rx = worldPoint.x - this.position.x, ry = worldPoint.y - this.position.y, rz = worldPoint.z - this.position.z;
    const tqx = ry * impulse.z - rz * impulse.y, tqy = rz * impulse.x - rx * impulse.z, tqz = rx * impulse.y - ry * impulse.x;
    const I = this._worldInverseInertiaTensor;
    this.angular_velocity.x += (I.e00 * tqx + I.e01 * tqy + I.e02 * tqz) * this.angular_factor.x;
    this.angular_velocity.y += (I.e10 * tqx + I.e11 * tqy + I.e12 * tqz) * this.angular_factor.y;
    this.angular_velocity.z += (I.e20 * tqx + I.e21 * tqy + I.e22 * tqz) * this.angular_factor.z;
    return this;
};

proto.applyTorqueImpulse = function (torqueImpulse) {
    if (this._massInverted <= 0) return this;
    const I = this._worldInverseInertiaTensor;
    const tx = torqueImpulse.x, ty = torqueImpulse.y, tz = torqueImpulse.z;
    this.angular_velocity.x += (I.e00 * tx + I.e01 * ty + I.e02 * tz) * this.angular_factor.x;
    this.angular_velocity.y += (I.e10 * tx + I.e11 * ty + I.e12 * tz) * this.angular_factor.y;
    this.angular_velocity.z += (I.e20 * tx + I.e21 * ty + I.e22 * tz) * this.angular_factor.z;
    return this;
};

// Continuous force, integrated by the solver every substep until cleared. Adds, not overwrites -
// multiple calls in the same tick (gravity plus thrust plus wind) all contribute.
proto.applyForce = function (force) {
    this.accumulated_force.x += force.x;
    this.accumulated_force.y += force.y;
    this.accumulated_force.z += force.z;
    return this;
};

proto.applyTorque = function (torque) {
    this.accumulated_torque.x += torque.x;
    this.accumulated_torque.y += torque.y;
    this.accumulated_torque.z += torque.z;
    return this;
};

// A force at a world-space point contributes the force itself plus the torque it produces about
// the center (r x force) - the continuous-force analogue of applyImpulseAtPoint.
proto.applyForceAtPoint = function (force, worldPoint) {
    this.applyForce(force);
    const rx = worldPoint.x - this.position.x, ry = worldPoint.y - this.position.y, rz = worldPoint.z - this.position.z;
    this.accumulated_torque.x += ry * force.z - rz * force.y;
    this.accumulated_torque.y += rz * force.x - rx * force.z;
    this.accumulated_torque.z += rx * force.y - ry * force.x;
    return this;
};

// Zeroes accumulated force/torque. Called by World.step once per TICK (not per substep) - a
// caller who wants a force to keep acting must call applyForce again next tick.
proto.clearForces = function () {
    this.accumulated_force.set(0, 0, 0);
    this.accumulated_torque.set(0, 0, 0);
    return this;
};
