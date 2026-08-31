// Impulse (instantaneous velocity change) and force/torque (continuous, integrated per-substep,
// cleared once per tick) application.
var proto = RigidBody.prototype;

proto.applyImpulse = function (impulse) {
    if (this._mass_inverted <= 0) return this;
    if (!this.isAwake) this.wakeUp();
    this.linear_velocity.x += impulse.x * this._mass_inverted * this.linear_factor.x;
    this.linear_velocity.y += impulse.y * this._mass_inverted * this.linear_factor.y;
    this.linear_velocity.z += impulse.z * this._mass_inverted * this.linear_factor.z;
    return this;
};

// Add a velocity delta directly - mass-independent (dv, not an impulse J = m*dv). Use this for a
// "shove" whose strength should not depend on how heavy the target is (a game gravity-gun, a
// scripted knockback). A static/kinematic body (no finite mass) is unaffected. linear_factor still
// masks locked axes.
proto.addLinearVelocity = function (dv) {
    if (this._mass_inverted <= 0) return this;
    if (!this.isAwake) this.wakeUp();
    this.linear_velocity.x += dv.x * this.linear_factor.x;
    this.linear_velocity.y += dv.y * this.linear_factor.y;
    this.linear_velocity.z += dv.z * this.linear_factor.z;
    return this;
};

// Impulse at a world-space point: linear change plus the angular change it produces about the
// center (dw = I^-1 * (r x impulse)).
proto.applyImpulseAtPoint = function (impulse, worldPoint) {
    if (this._mass_inverted <= 0) return this;
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
    if (this._mass_inverted <= 0) return this;
    if (!this.isAwake) this.wakeUp();
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
    if (!this.isAwake && (force.x !== 0 || force.y !== 0 || force.z !== 0)) this.wakeUp();
    this.accumulated_force.x += force.x;
    this.accumulated_force.y += force.y;
    this.accumulated_force.z += force.z;
    return this;
};

proto.applyTorque = function (torque) {
    if (!this.isAwake && (torque.x !== 0 || torque.y !== 0 || torque.z !== 0)) this.wakeUp();
    this.accumulated_torque.x += torque.x;
    this.accumulated_torque.y += torque.y;
    this.accumulated_torque.z += torque.z;
    return this;
};

// A force at a world-space point contributes the force itself plus the torque it produces about
// the center (r x force) - the continuous-force analogue of applyImpulseAtPoint.
proto.applyForceAtWorldPoint = function (force, worldPoint) {
    this.applyForce(force);
    const rx = worldPoint.x - this.position.x, ry = worldPoint.y - this.position.y, rz = worldPoint.z - this.position.z;
    this.accumulated_torque.x += ry * force.z - rz * force.y;
    this.accumulated_torque.y += rz * force.x - rx * force.z;
    this.accumulated_torque.z += rx * force.y - ry * force.x;
    return this;
};

// Same as applyForceAtWorldPoint but the point is given in this body's local frame - transformed to
// world via the current transform, then delegated.
proto.applyForceAtLocalPoint = function (force, localPoint) {
    const world = RigidBody._scratchForcePoint;
    this.getTransform().transformPointInto(localPoint, world);
    return this.applyForceAtWorldPoint(force, world);
};

// Velocity of a point on this body: v_linear + omega x r, where r is `offset` - a vector from the
// center of mass, in world axes (the "local" in the name is historical; the offset is not rotated
// into the body frame). `out` receives the result. Zero angular/linear -> just the linear velocity.
proto.getVelocityInLocalPoint = function (offset, out) {
    const w = this.angular_velocity, v = this.linear_velocity;
    out.x = v.x + (w.y * offset.z - w.z * offset.y);
    out.y = v.y + (w.z * offset.x - w.x * offset.z);
    out.z = v.z + (w.x * offset.y - w.y * offset.x);
    return out;
};

// Zeroes accumulated force/torque. Called by World.step once per TICK (not per substep) - a
// caller who wants a force to keep acting must call applyForce again next tick.
proto.clearForces = function () {
    this.accumulated_force.set(0, 0, 0);
    this.accumulated_torque.set(0, 0, 0);
    return this;
};
