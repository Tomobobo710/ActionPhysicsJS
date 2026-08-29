// Hinge: pivot (3 DOF, via composed PointConstraint) + axis lock (2 DOF), optional swing limit/motor.
class HingeConstraint extends Constraint {
    constructor(bodyA, hingeAxisA, pivotA, bodyB, pivotB) {
        super(bodyA, bodyB);
        this.localAxisA = new Vector3().copy(hingeAxisA).normalizeInPlace();
        this.localPivotA = new Vector3().copy(pivotA);
        this.localPivotB = new Vector3().copy(pivotB || new Vector3());

        this.localAxisB = new Vector3();
        if (bodyB) {
            const worldAxis = HingeConstraint._scratchV1.copy(this.localAxisA);
            bodyA.rotation.transformVectorInPlace(worldAxis);
            const invRotB = HingeConstraint._scratchQ.copy(bodyB.rotation).invert();
            this.localAxisB.copy(worldAxis);
            invRotB.transformVectorInPlace(this.localAxisB);
        }
        this._pivot = new PointConstraint(bodyA, bodyB, this.localPivotA, bodyB ? this.localPivotB : this._worldPivotBPlaceholder());

        // Swing-angle reference vector, perpendicular to the axis, in each body's local space.
        this._refA = HingeConstraint._perpendicularTo(this.localAxisA);
        if (bodyB) {
            const worldRef = HingeConstraint._scratchV1.copy(this._refA);
            bodyA.rotation.transformVectorInPlace(worldRef);
            const invRotB = HingeConstraint._scratchQ.copy(bodyB.rotation).invert();
            this._refB = new Vector3().copy(worldRef);
            invRotB.transformVectorInPlace(this._refB);
        } else {
            this._refB = null;
            this._fixedWorldRef = HingeConstraint._scratchV1.copy(this._refA);
            bodyA.rotation.transformVectorInPlace(this._fixedWorldRef);
            this._fixedWorldRef = new Vector3().copy(this._fixedWorldRef);
        }

        this.limit = { min: null, max: null, set: function (min, max) { this.min = min; this.max = max; return this; } };
        this.motor = { targetVelocity: 0, maxTorque: 0, set: function (targetVelocity, maxTorque) { this.targetVelocity = targetVelocity; this.maxTorque = maxTorque; return this; } };
    }

    // Gram-Schmidt: any vector not parallel to axis, made perpendicular + unit length.
    static _perpendicularTo(axis) {
        const seed = Math.abs(axis.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
        const d = seed.x * axis.x + seed.y * axis.y + seed.z * axis.z;
        const perp = new Vector3(seed.x - d * axis.x, seed.y - d * axis.y, seed.z - d * axis.z);
        return perp.normalizeInPlace();
    }

    // Null bodyB: PointConstraint wants a world point, so use bodyA's own world pivot at construction.
    _worldPivotBPlaceholder() {
        const world = HingeConstraint._scratchV2.copy(this.localPivotA);
        this.bodyA.rotation.transformVectorInPlace(world);
        world.addInPlace(this.bodyA.position);
        return new Vector3().copy(world);
    }

    solve(h) {
        if (!this.enabled) return;
        this._pivot.solve(h);
        this._solveAxisAlignment();
        if (this.limit.min != null || this.limit.max != null) this._solveLimit();
        if (this.motor.maxTorque > 0) this._solveMotor(h);
    }

    // Signed swing angle about the axis, refB -> refA, both projected into the plane perpendicular to axis.
    _swingAngle() {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axis = HingeConstraint._scratchAxis.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axis);

        const refA = HingeConstraint._scratchV1.copy(this._refA);
        bodyA.rotation.transformVectorInPlace(refA);

        const refB = HingeConstraint._scratchV2;
        if (bodyB) { refB.copy(this._refB); bodyB.rotation.transformVectorInPlace(refB); }
        else refB.copy(this._fixedWorldRef);

        HingeConstraint._projectOntoPlane(refA, axis);
        HingeConstraint._projectOntoPlane(refB, axis);
        const dot = refA.x * refB.x + refA.y * refB.y + refA.z * refB.z;
        const cx = refB.y * refA.z - refB.z * refA.y, cy = refB.z * refA.x - refB.x * refA.z, cz = refB.x * refA.y - refB.y * refA.x;
        const crossDotAxis = cx * axis.x + cy * axis.y + cz * axis.z;
        return Scalar.atan2(crossDotAxis, dot);
    }

    static _projectOntoPlane(v, axis) {
        const d = v.x * axis.x + v.y * axis.y + v.z * axis.z;
        v.x -= d * axis.x; v.y -= d * axis.y; v.z -= d * axis.z;
        const len = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len > 1e-12) { v.x /= len; v.y /= len; v.z /= len; }
    }

    _solveLimit() {
        const angle = this._swingAngle();
        const min = this.limit.min != null ? this.limit.min : -Infinity;
        const max = this.limit.max != null ? this.limit.max : Infinity;
        let violation = 0;
        if (angle < min) violation = angle - min;
        else if (angle > max) violation = angle - max;
        else return;

        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axis = HingeConstraint._scratchAxis.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axis);

        let wSum = 0;
        const hasB = !!(bodyB && bodyB._massInverted > 0);
        if (bodyA._massInverted > 0) wSum += HingeConstraint._angularEffectiveMass(bodyA, axis.x, axis.y, axis.z);
        if (hasB) wSum += HingeConstraint._angularEffectiveMass(bodyB, axis.x, axis.y, axis.z);
        if (wSum < 1e-12) return;

        const scale = -violation / wSum;
        const tx = axis.x * scale, ty = axis.y * scale, tz = axis.z * scale;
        if (bodyA._massInverted > 0) HingeConstraint._applyAngularDelta(bodyA, tx, ty, tz);
        if (hasB) HingeConstraint._applyAngularDelta(bodyB, -tx, -ty, -tz);
    }

    // Position-space motor: writes a bounded angle step (not velocity directly, since the solver
    // derives velocity from position delta after all constraints run).
    _solveMotor(h) {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axis = HingeConstraint._scratchAxis.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axis);

        let wSum = 0;
        const hasB = !!(bodyB && bodyB._massInverted > 0);
        if (bodyA._massInverted > 0) wSum += HingeConstraint._angularEffectiveMass(bodyA, axis.x, axis.y, axis.z);
        if (hasB) wSum += HingeConstraint._angularEffectiveMass(bodyB, axis.x, axis.y, axis.z);
        if (wSum < 1e-12) return;

        const wA = bodyA.angular_velocity, wB = hasB ? bodyB.angular_velocity : HingeConstraint._zero;
        const relOmega = (wA.x - wB.x) * axis.x + (wA.y - wB.y) * axis.y + (wA.z - wB.z) * axis.z;
        const velError = this.motor.targetVelocity - relOmega;
        if (velError === 0) return;

        const maxDeltaOmega = this.motor.maxTorque * wSum;
        const deltaOmega = velError > 0 ? Math.min(velError, maxDeltaOmega) : Math.max(velError, -maxDeltaOmega);
        let step = deltaOmega * h;
        if (step === 0) return;

        if (this.limit.min != null || this.limit.max != null) {
            const angle = this._swingAngle();
            const min = this.limit.min != null ? this.limit.min : -Infinity;
            const max = this.limit.max != null ? this.limit.max : Infinity;
            if (step > 0 && angle + step > max) step = Math.max(0, max - angle);
            else if (step < 0 && angle + step < min) step = Math.min(0, min - angle);
            if (step === 0) return;
        }

        const scale = step / wSum;
        const tx = axis.x * scale, ty = axis.y * scale, tz = axis.z * scale;
        if (bodyA._massInverted > 0) HingeConstraint._applyAngularDelta(bodyA, tx, ty, tz);
        if (hasB) HingeConstraint._applyAngularDelta(bodyB, -tx, -ty, -tz);
    }

    _solveAxisAlignment() {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axisA = HingeConstraint._scratchV1.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axisA);

        const axisB = HingeConstraint._scratchV2;
        if (bodyB) {
            axisB.copy(this.localAxisB);
            bodyB.rotation.transformVectorInPlace(axisB);
        } else {
            if (!this._fixedWorldAxis) {
                this._fixedWorldAxis = new Vector3().copy(this.localAxisA);
                this.bodyA.rotation.transformVectorInPlace(this._fixedWorldAxis);
            }
            axisB.copy(this._fixedWorldAxis);
        }

        // axisA x axisB: zero when parallel, magnitude ~sin(angle), direction = correction rotation.
        const ex = axisA.y * axisB.z - axisA.z * axisB.y;
        const ey = axisA.z * axisB.x - axisA.x * axisB.z;
        const ez = axisA.x * axisB.y - axisA.y * axisB.x;
        const errLenSq = ex * ex + ey * ey + ez * ez;
        if (errLenSq < 1e-20) return;

        const errLen = Math.sqrt(errLenSq);
        const dx = ex / errLen, dy = ey / errLen, dz = ez / errLen;
        let wSum = 0;
        const hasB = !!(bodyB && bodyB._massInverted > 0);
        if (bodyA._massInverted > 0) wSum += HingeConstraint._angularEffectiveMass(bodyA, dx, dy, dz);
        if (hasB) wSum += HingeConstraint._angularEffectiveMass(bodyB, dx, dy, dz);
        if (wSum < 1e-12) return;

        const scale = -1 / wSum;
        const tx = ex * scale, ty = ey * scale, tz = ez * scale;

        if (bodyA._massInverted > 0) HingeConstraint._applyAngularDelta(bodyA, -tx, -ty, -tz);
        if (hasB) HingeConstraint._applyAngularDelta(bodyB, tx, ty, tz);
    }

    static _angularEffectiveMass(body, dx, dy, dz) {
        const I = body._worldInverseInertiaTensor;
        const ix = I.e00 * dx + I.e01 * dy + I.e02 * dz;
        const iy = I.e10 * dx + I.e11 * dy + I.e12 * dz;
        const iz = I.e20 * dx + I.e21 * dy + I.e22 * dz;
        return dx * ix + dy * iy + dz * iz;
    }

    static _applyAngularDelta(body, tx, ty, tz) {
        const I = body._worldInverseInertiaTensor;
        const wx = I.e00 * tx + I.e01 * ty + I.e02 * tz;
        const wy = I.e10 * tx + I.e11 * ty + I.e12 * tz;
        const wz = I.e20 * tx + I.e21 * ty + I.e22 * tz;
        HingeConstraint._scratchAngular.set(wx * body.angular_factor.x, wy * body.angular_factor.y, wz * body.angular_factor.z);
        Solver._integrateRotation(body.rotation, HingeConstraint._scratchAngular, 1);
    }
}

HingeConstraint._scratchV1 = new Vector3();
HingeConstraint._scratchV2 = new Vector3();
HingeConstraint._scratchAxis = new Vector3();
HingeConstraint._scratchQ = new Quaternion();
HingeConstraint._scratchAngular = new Vector3();
HingeConstraint._zero = new Vector3();

ActionPhysics.HingeConstraint = HingeConstraint;
