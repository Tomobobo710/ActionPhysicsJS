/**
 * HingeConstraint: two bodies (or one body and the world) rotate about a shared axis and pivot
 * point, like a door hinge - 3 translational DOF removed at the pivot (same as PointConstraint) plus
 * 2 of the 3 rotational DOF removed (only rotation about the shared hinge axis is free). No swing
 * limit or motor yet (plan.md's Constraint base/limit/motor layer, item 8, is a later increment - see
 * plan.md's Not-started list; a hinge with no limit/motor is still a complete, correct hinge, just an
 * unbounded/unpowered one).
 *
 * REBUILT AS TWO STACKED XPBD POSITION CONSTRAINTS (plan.md, "This is a rebuild, not a port" -
 * Constraint.js's own header explains why Goblin's ConstraintRow/Jacobian shape is not carried over).
 * The pivot uses exactly PointConstraint's own 3x3 coupled solve (composition, not duplication - see
 * _solvePivot below, which shares PointConstraint's math via the same effective-mass helper). The
 * angular lock uses the standard XPBD relative-rotation trick (Muller et al. 2020 sec 3.4): each
 * body carries the hinge axis in its OWN local space; transformed to world space each substep, the
 * two world axes should be parallel. `axisA x axisB` is a vector whose magnitude is sin(angle between
 * them) and whose direction is the rotation that would bring them together - used directly as a
 * small-angle angular position correction (exact for small deviations, same "delta IS the small-angle
 * correction" idea Solver._applyAngularCorrection already uses for contacts). Distributed to each
 * body by its own inverse inertia, exactly like the contact solver's angular correction.
 */
class HingeConstraint extends Constraint {
    // bodyA/hingeAxisA/pivotA are required; bodyB/pivotB are optional (null bodyB = hinge to a fixed
    // world pivot/axis, matching PointConstraint's own bodyB=null convention). hingeAxisA is in
    // bodyA's LOCAL space; when bodyB is given, bodyB's own local hinge axis is derived once at
    // construction from bodyA's current world axis (so both bodies start perfectly aligned) - the
    // caller does not need to hand-compute bodyB's local axis separately.
    constructor(bodyA, hingeAxisA, pivotA, bodyB, pivotB) {
        super(bodyA, bodyB);
        this.localAxisA = new Vector3().copy(hingeAxisA).normalizeInPlace();
        this.localPivotA = new Vector3().copy(pivotA);
        this.localPivotB = new Vector3().copy(pivotB || new Vector3());
        // bodyB's local hinge axis: derived from bodyA's world axis at construction time so the two
        // bodies start co-axial (matching Goblin's own initial_quaternion capture, but derived
        // directly here rather than stored as an initial relative rotation - this engine needs only
        // the axis, not the free rotation about it, since there is no swing limit yet to measure
        // against an initial reference angle).
        this.localAxisB = new Vector3();
        if (bodyB) {
            const worldAxis = HingeConstraint._scratchV1.copy(this.localAxisA);
            bodyA.rotation.transformVectorInPlace(worldAxis);
            const invRotB = HingeConstraint._scratchQ.copy(bodyB.rotation).invert();
            this.localAxisB.copy(worldAxis);
            invRotB.transformVectorInPlace(this.localAxisB);
        }
        // Reuse PointConstraint's own pivot solve by composing an internal instance rather than
        // duplicating its 3x3 coupled math - one owner for "solve a point constraint" (Rule 2).
        this._pivot = new PointConstraint(bodyA, bodyB, this.localPivotA, bodyB ? this.localPivotB : this._worldPivotBPlaceholder());
    }

    // When bodyB is null, PointConstraint expects its "localAnchorB" to already be a WORLD point
    // (see PointConstraint's own header) - HingeConstraint's constructor-time pivotB argument is
    // meaningless in that case, so the fixed pivot is instead bodyA's OWN world pivot at
    // construction time, matching what a hinge-to-world naturally means (the world half of the
    // pivot is wherever the door frame actually is, i.e. where bodyA's pivot starts).
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
    }

    // Aligns bodyA's and bodyB's world-space hinge axes via a small-angle angular correction along
    // axisA x axisB, distributed by each body's own inverse inertia (no linear component - this is
    // a pure rotation constraint, unlike the pivot). When bodyB is null the axis is locked to its
    // OWN world direction at construction time (a fixed hinge axis in space, e.g. a door hinge bolted
    // to a static frame).
    _solveAxisAlignment() {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axisA = HingeConstraint._scratchV1.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axisA);

        const axisB = HingeConstraint._scratchV2;
        if (bodyB) {
            axisB.copy(this.localAxisB);
            bodyB.rotation.transformVectorInPlace(axisB);
        } else {
            // Fixed world axis: whatever bodyA's world axis was AT CONSTRUCTION time. Recompute once
            // at construction and cache, rather than every substep, since a null bodyB never moves.
            if (!this._fixedWorldAxis) {
                this._fixedWorldAxis = new Vector3().copy(this.localAxisA);
                this.bodyA.rotation.transformVectorInPlace(this._fixedWorldAxis);
            }
            axisB.copy(this._fixedWorldAxis);
        }

        // error = axisA x axisB: zero when parallel, magnitude ~ sin(angle) between them, direction
        // is the small-angle rotation that would bring axisA onto axisB.
        const ex = axisA.y * axisB.z - axisA.z * axisB.y;
        const ey = axisA.z * axisB.x - axisA.x * axisB.z;
        const ez = axisA.x * axisB.y - axisA.y * axisB.x;
        const errLenSq = ex * ex + ey * ey + ez * ez;
        if (errLenSq < 1e-20) return; // already aligned to numerical precision

        // Angular effective mass along the correction direction: wA + wB, each body's inverse
        // inertia projected onto the (unit) correction axis - same idea as the contact solver's
        // linear effective mass, but for a pure-rotation constraint (no r x n lever-arm term, since
        // this correction has no offset - it acts on the whole body's orientation directly).
        const errLen = Math.sqrt(errLenSq);
        const dx = ex / errLen, dy = ey / errLen, dz = ez / errLen;
        let wSum = 0;
        const hasB = !!(bodyB && bodyB._massInverted > 0);
        if (bodyA._massInverted > 0) wSum += HingeConstraint._angularEffectiveMass(bodyA, dx, dy, dz);
        if (hasB) wSum += HingeConstraint._angularEffectiveMass(bodyB, dx, dy, dz);
        if (wSum < 1e-12) return;

        // deltaTheta = -error / wSum (Lagrange-multiplier form, same shape as the contact solver's
        // deltaLambda = -C/wSum): the rotation that resolves the misalignment in one solve.
        const scale = -1 / wSum;
        const tx = ex * scale, ty = ey * scale, tz = ez * scale;

        if (bodyA._massInverted > 0) HingeConstraint._applyAngularDelta(bodyA, -tx, -ty, -tz);
        if (hasB) HingeConstraint._applyAngularDelta(bodyB, tx, ty, tz);
    }

    // wA = axis . (IA^-1 * axis) - the scalar inverse "moment" along a unit rotation axis.
    static _angularEffectiveMass(body, dx, dy, dz) {
        const I = body._worldInverseInertiaTensor;
        const ix = I.e00 * dx + I.e01 * dy + I.e02 * dz;
        const iy = I.e10 * dx + I.e11 * dy + I.e12 * dz;
        const iz = I.e20 * dx + I.e21 * dy + I.e22 * dz;
        return dx * ix + dy * iy + dz * iz;
    }

    // Applies a pure-rotation position correction: dRotation = IA^-1 * torque, then the standard
    // small-angle quaternion integration (same primitive Solver._applyAngularCorrection uses for
    // contacts, reused here directly rather than reimplemented - one owner for "integrate a small
    // angular correction into a quaternion").
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
HingeConstraint._scratchQ = new Quaternion();
HingeConstraint._scratchAngular = new Vector3();

ActionPhysics.HingeConstraint = HingeConstraint;
