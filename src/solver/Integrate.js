// Integrate velocity and predict position each substep, plus the rotation helpers.
var proto = Solver.prototype;

proto._integrate = function (bodies, gravity, h) {
    for (let i = 0; i < bodies.length; i++) {
        const b = bodies[i];

        // A KINEMATIC body is code-driven: no gravity, no forces, no damping, and its velocity is
        // authoritative (never derived back from position). Just carry its transform along its
        // current velocity so contacts this substep see it where it will be, exactly as a dynamic
        // body's predicted position is used. A driver that writes position directly instead of
        // setting velocity leaves linear/angular velocity at zero and this is a no-op.
        if (b.bodyType === RigidBody.KINEMATIC) {
            const lv = b.linear_velocity;
            if (lv.x !== 0 || lv.y !== 0 || lv.z !== 0) b.position.addScaledInPlace(lv, h);
            const av = b.angular_velocity;
            if (av.x !== 0 || av.y !== 0 || av.z !== 0) Solver._integrateRotation(b.rotation, av, h);
            continue;
        }

        if (b.bodyType !== RigidBody.DYNAMIC || !b.isAwake) continue;

        // These snapshots only need to survive within the substep (derived-velocity + restitution
        // read them later this substep, never across substeps), so reuse the per-body slot rather
        // than allocating a fresh Vector3/Quaternion every body every substep - ~6000 allocs/tick
        // otherwise, forever, even at rest.
        let prevPos = this._prevPos.get(b.id);
        if (!prevPos) { prevPos = new Vector3(); this._prevPos.set(b.id, prevPos); }
        prevPos.copy(b.position);
        let prevRot = this._prevRot.get(b.id);
        if (!prevRot) { prevRot = new Quaternion(); this._prevRot.set(b.id, prevRot); }
        prevRot.copy(b.rotation);
        let bias = this._biasDelta.get(b.id);
        if (!bias) { bias = new Vector3(); this._biasDelta.set(b.id, bias); }
        bias.set(0, 0, 0);
        // Pre-gravity snapshot; restitution's pre-solve velocity reads this.
        let preGrav = this._preGravityVel.get(b.id);
        if (!preGrav) { preGrav = new Vector3(); this._preGravityVel.set(b.id, preGrav); }
        preGrav.copy(b.linear_velocity);

        const g = b.gravity || gravity;
        b.linear_velocity.x += g.x * h * b.linear_factor.x;
        b.linear_velocity.y += g.y * h * b.linear_factor.y;
        b.linear_velocity.z += g.z * h * b.linear_factor.z;

        const af = b.accumulated_force;
        if (af.x !== 0 || af.y !== 0 || af.z !== 0) {
            b.linear_velocity.x += af.x * b._mass_inverted * h * b.linear_factor.x;
            b.linear_velocity.y += af.y * b._mass_inverted * h * b.linear_factor.y;
            b.linear_velocity.z += af.z * b._mass_inverted * h * b.linear_factor.z;
        }
        const at = b.accumulated_torque;
        if (at.x !== 0 || at.y !== 0 || at.z !== 0) {
            const I = b._worldInverseInertiaTensor;
            b.angular_velocity.x += (I.e00 * at.x + I.e01 * at.y + I.e02 * at.z) * h * b.angular_factor.x;
            b.angular_velocity.y += (I.e10 * at.x + I.e11 * at.y + I.e12 * at.z) * h * b.angular_factor.y;
            b.angular_velocity.z += (I.e20 * at.x + I.e21 * at.y + I.e22 * at.z) * h * b.angular_factor.z;
        }

        if (b.linear_damping > 0) b.linear_velocity.scaleInPlace(Math.max(0, 1 - b.linear_damping * h));
        if (b.angular_damping > 0) b.angular_velocity.scaleInPlace(Math.max(0, 1 - b.angular_damping * h));

        b.position.addScaledInPlace(b.linear_velocity, h);
        Solver._integrateRotation(b.rotation, b.angular_velocity, h);
        b._recomputeWorldInverseInertia(); // rotation changed

    }
};

proto._deriveVelocities = function (bodies, h) {
    for (let i = 0; i < bodies.length; i++) {
        const b = bodies[i];
        if (b.bodyType !== RigidBody.DYNAMIC || !b.isAwake) continue;
        const prevPos = this._prevPos.get(b.id);
        const prevRot = this._prevRot.get(b.id);
        const bias = this._biasDelta.get(b.id);
        // Bias-only motion (PositionSolve.js) is excluded so it derives no velocity.
        b.linear_velocity.x = (b.position.x - prevPos.x - bias.x) / h;
        b.linear_velocity.y = (b.position.y - prevPos.y - bias.y) / h;
        b.linear_velocity.z = (b.position.z - prevPos.z - bias.z) / h;
        Solver._deriveAngularVelocity(b.angular_velocity, prevRot, b.rotation, h);
    }
};

// Exact exponential-map quaternion integration: dq = (cos(theta/2), sin(theta/2)*axis).
Solver._integrateRotation = function (rotation, angularVelocity, h) {
    const wx = angularVelocity.x, wy = angularVelocity.y, wz = angularVelocity.z;
    const wLenSq = wx * wx + wy * wy + wz * wz;
    if (wLenSq < 1e-24) return;
    const wLen = Math.sqrt(wLenSq);
    const halfAngle = wLen * h * 0.5;
    const s = Scalar.sin(halfAngle) / wLen;
    const dqx = wx * s, dqy = wy * s, dqz = wz * s, dqw = Scalar.cos(halfAngle);

    const qx = rotation.x, qy = rotation.y, qz = rotation.z, qw = rotation.w;
    const rx = dqw * qx + dqx * qw + dqy * qz - dqz * qy;
    const ry = dqw * qy - dqx * qz + dqy * qw + dqz * qx;
    const rz = dqw * qz + dqx * qy - dqy * qx + dqz * qw;
    const rw = dqw * qw - dqx * qx - dqy * qy - dqz * qz;
    rotation.x = rx; rotation.y = ry; rotation.z = rz; rotation.w = rw;
    rotation.normalize();
};

// Angular velocity from the rotation delta between prevRot and rotation: dq = rotation * conj(prevRot).
Solver._deriveAngularVelocity = function (out, prevRot, rotation, h) {
    let dqx = rotation.w * (-prevRot.x) + rotation.x * prevRot.w + rotation.y * (-prevRot.z) - rotation.z * (-prevRot.y);
    let dqy = rotation.w * (-prevRot.y) - rotation.x * (-prevRot.z) + rotation.y * prevRot.w + rotation.z * (-prevRot.x);
    let dqz = rotation.w * (-prevRot.z) + rotation.x * (-prevRot.y) - rotation.y * (-prevRot.x) + rotation.z * prevRot.w;
    let dqw = rotation.w * prevRot.w - rotation.x * (-prevRot.x) - rotation.y * (-prevRot.y) - rotation.z * (-prevRot.z);
    if (dqw < 0) { dqx = -dqx; dqy = -dqy; dqz = -dqz; dqw = -dqw; } // shorter path
    const sinHalf = Math.sqrt(dqx * dqx + dqy * dqy + dqz * dqz);
    if (sinHalf < 1e-12) { out.x = 0; out.y = 0; out.z = 0; return; }
    const halfAngle = Scalar.atan2(sinHalf, dqw);
    const scale = (2 * halfAngle / h) / sinHalf;
    out.x = dqx * scale; out.y = dqy * scale; out.z = dqz * scale;
};
