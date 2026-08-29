/**
 * WeldConstraint: rigidly fuses two bodies (or one body and the world) together at a shared point -
 * all 6 relative DOF removed (3 translational, same PointConstraint pivot; 3 rotational, held at
 * whatever relative orientation the two bodies had when the weld was created). Behaves like the two
 * bodies were merged into one rigid body, without actually merging their shapes/mass.
 *
 * REBUILT, NOT PORTED. The pivot reuses PointConstraint's own 3x3 coupled solve directly (one owner
 * for "solve a point constraint" - Rule 2), same composition HingeConstraint uses. The rotational
 * lock is the general XPBD relative-rotation constraint (Muller et al. 2020 sec 3.4): the RELATIVE
 * rotation between the two bodies (qB * qA^-1, or for a world weld, qA's own rotation) is compared
 * against the relative rotation captured AT CONSTRUCTION TIME - the error quaternion's imaginary
 * part (x,y,z) IS a small-angle axis*sin(angle/2) correction directly (a standard identity: for a
 * near-identity quaternion, (x,y,z) approximates half the rotation vector), used the same way
 * HingeConstraint's axisA x axisB error is - a Lagrange-multiplier-shaped deltaTheta = -error/wSum,
 * distributed by each body's own inverse inertia. Locking all 3 rotational DOF this way (rather than
 * Hinge's 2) is the only difference in the angular half; the pivot half is identical.
 */
class WeldConstraint extends Constraint {
    constructor(bodyA, bodyB, pivotA, pivotB) {
        super(bodyA, bodyB);
        this.localPivotA = new Vector3().copy(pivotA);
        this.localPivotB = new Vector3().copy(pivotB || new Vector3());

        // The relative rotation to HOLD, captured now: for two bodies, qRel = qB^-1 * qA (so that
        // qA * qRel^-1 gives back qB whenever satisfied). For a world weld (bodyB null), qRel is
        // simply bodyA's own rotation at construction - the fixed orientation to hold it at.
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

    // Drives the CURRENT relative rotation back to targetRel via a small-angle correction, same
    // Lagrange-multiplier shape as every other constraint here.
    _solveRotationLock() {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        // currentRel = qB^-1 * qA (or just qA for a world weld) - same convention as construction.
        const currentRel = WeldConstraint._scratchQ2;
        if (bodyB) {
            const invB = WeldConstraint._scratchQ.copy(bodyB.rotation).invert();
            currentRel.multiplyQuaternions(invB, bodyA.rotation);
        } else {
            currentRel.copy(bodyA.rotation);
        }
        // error = currentRel * targetRel^-1 - the rotation that would bring currentRel back to
        // targetRel. Its imaginary part is a direct small-angle correction (same identity Hinge's
        // axis-alignment error uses, generalized from a 2D cross product to the full quaternion).
        const invCurrent = WeldConstraint._scratchQ3.copy(currentRel).invert();
        const errQ = WeldConstraint._scratchQ4.multiplyQuaternions(this.targetRel, invCurrent);
        if (errQ.w < 0) { errQ.x = -errQ.x; errQ.y = -errQ.y; errQ.z = -errQ.z; errQ.w = -errQ.w; } // shorter path
        const ex = errQ.x, ey = errQ.y, ez = errQ.z;
        const errLenSq = ex * ex + ey * ey + ez * ez;
        if (errLenSq < 1e-20) return;

        // The error is expressed in BODY A's local orientation frame relative to B (since currentRel
        // was built as qB^-1 * qA) - rotate it into WORLD space before applying, since the angular
        // correction machinery (and inertia tensors) operate in world space. For a world weld
        // currentRel IS bodyA's world rotation already, so the error is already world-frame; for a
        // two-body weld it must be rotated by bodyB's world rotation first.
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
