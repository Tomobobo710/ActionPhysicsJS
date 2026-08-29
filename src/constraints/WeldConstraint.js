// Rigidly fuses two bodies at a shared point: pivot (composed PointConstraint) + full 3-DOF
// rotation lock at whatever relative orientation existed at construction.
class WeldConstraint extends Constraint {
    constructor(bodyA, bodyB, pivotA, pivotB) {
        super(bodyA, bodyB);
        this.localPivotA = new Vector3().copy(pivotA);
        this.localPivotB = new Vector3().copy(pivotB || new Vector3());

        // Relative rotation to hold: qRel = qB^-1 * qA (or bodyA's own rotation for a world weld).
        this.targetRel = new Quaternion();
        if (bodyB) {
            const invB = WeldConstraint._scratchQ.copy(bodyB.rotation).invert();
            this.targetRel.multiplyQuaternions(invB, bodyA.rotation);
        } else {
            this.targetRel.copy(bodyA.rotation);
        }

        this._pivot = bodyB
            ? new PointConstraint(bodyA, bodyB, this.localPivotA, this.localPivotB)
            : new PointConstraint(bodyA, null, this.localPivotA, WeldConstraint._worldPoint(bodyA, this.localPivotA));
    }

    static _worldPoint(body, localPoint) {
        const w = new Vector3().copy(localPoint);
        body.rotation.transformVectorInPlace(w);
        w.addInPlace(body.position);
        return w;
    }

    solve(h) {
        if (!this.enabled) return;
        this._pivot.solve(h);
        this._solveRotationLock();
    }

    _solveRotationLock() {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const currentRel = WeldConstraint._scratchQ2;
        if (bodyB) {
            const invB = WeldConstraint._scratchQ.copy(bodyB.rotation).invert();
            currentRel.multiplyQuaternions(invB, bodyA.rotation);
        } else {
            currentRel.copy(bodyA.rotation);
        }
        // error = currentRel * targetRel^-1; imaginary part is a direct small-angle correction.
        const invCurrent = WeldConstraint._scratchQ3.copy(currentRel).invert();
        const errQ = WeldConstraint._scratchQ4.multiplyQuaternions(this.targetRel, invCurrent);
        if (errQ.w < 0) { errQ.x = -errQ.x; errQ.y = -errQ.y; errQ.z = -errQ.z; errQ.w = -errQ.w; }
        const ex = errQ.x, ey = errQ.y, ez = errQ.z;
        const errLenSq = ex * ex + ey * ey + ez * ez;
        if (errLenSq < 1e-20) return;

        // error is in bodyB's local frame (currentRel = qB^-1 * qA); rotate to world before applying.
        const worldErr = WeldConstraint._scratchV;
        worldErr.set(ex, ey, ez);
        if (bodyB) bodyB.rotation.transformVectorInPlace(worldErr);
        const wex = worldErr.x, wey = worldErr.y, wez = worldErr.z;
        const wLen = Math.sqrt(wex * wex + wey * wey + wez * wez);
        if (wLen < 1e-12) return;
        const dx = wex / wLen, dy = wey / wLen, dz = wez / wLen;

        let wSum = 0;
        const hasB = !!(bodyB && bodyB._massInverted > 0);
        if (bodyA._massInverted > 0) wSum += WeldConstraint._angularEffectiveMass(bodyA, dx, dy, dz);
        if (hasB) wSum += WeldConstraint._angularEffectiveMass(bodyB, dx, dy, dz);
        if (wSum < 1e-12) return;

        const scale = -1 / wSum;
        const tx = wex * scale, ty = wey * scale, tz = wez * scale;
        if (bodyA._massInverted > 0) WeldConstraint._applyAngularDelta(bodyA, -tx, -ty, -tz);
        if (hasB) WeldConstraint._applyAngularDelta(bodyB, tx, ty, tz);
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
        WeldConstraint._scratchAngular.set(wx * body.angular_factor.x, wy * body.angular_factor.y, wz * body.angular_factor.z);
        Solver._integrateRotation(body.rotation, WeldConstraint._scratchAngular, 1);
    }
}

WeldConstraint._scratchQ = new Quaternion();
WeldConstraint._scratchQ2 = new Quaternion();
WeldConstraint._scratchQ3 = new Quaternion();
WeldConstraint._scratchQ4 = new Quaternion();
WeldConstraint._scratchV = new Vector3();
WeldConstraint._scratchAngular = new Vector3();

ActionPhysics.WeldConstraint = WeldConstraint;
