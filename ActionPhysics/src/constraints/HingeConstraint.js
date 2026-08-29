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

        // Swing-angle reference frame: a vector perpendicular to the hinge axis, fixed in bodyA's
        // LOCAL space, whose current angle (about the axis, relative to the SAME perpendicular
        // carried by bodyB, or by a fixed world reference when bodyB is null) is the hinge's swing
        // angle. Built once here via Gram-Schmidt against localAxisA so it is guaranteed
        // perpendicular regardless of which direction the caller's axis points.
        this._refA = HingeConstraint._perpendicularTo(this.localAxisA);
        if (bodyB) {
            // bodyB's own copy of the SAME world reference vector, expressed in bodyB's local
            // space at construction time - both bodies start at zero swing angle by construction.
            const worldRef = HingeConstraint._scratchV1.copy(this._refA);
            bodyA.rotation.transformVectorInPlace(worldRef);
            const invRotB = HingeConstraint._scratchQ.copy(bodyB.rotation).invert();
            this._refB = new Vector3().copy(worldRef);
            invRotB.transformVectorInPlace(this._refB);
        } else {
            // Fixed world reference: bodyA's own world reference vector AT CONSTRUCTION time,
            // cached once (mirrors _fixedWorldAxis's own null-bodyB convention below).
            this._refB = null;
            this._fixedWorldRef = HingeConstraint._scratchV1.copy(this._refA);
            bodyA.rotation.transformVectorInPlace(this._fixedWorldRef);
            this._fixedWorldRef = new Vector3().copy(this._fixedWorldRef);
        }

        // limit: { min, max } angle in radians about the hinge axis (right-hand rule), or null for
        // unbounded. set(min, max) below is the caller's entry point (matches Goblin's own
        // constraint.limit.set(min, max) call shape, since that is simply "an angle range", not an
        // implementation detail worth diverging from).
        this.limit = { min: null, max: null, set: function (min, max) { this.min = min; this.max = max; return this; } };
        // motor: drives the hinge toward `targetVelocity` (rad/s about the axis) up to `maxTorque`
        // (world torque units) of effort per substep. maxTorque = 0 means no motor (the default).
        this.motor = { targetVelocity: 0, maxTorque: 0, set: function (targetVelocity, maxTorque) { this.targetVelocity = targetVelocity; this.maxTorque = maxTorque; return this; } };
    }

    // Any vector not parallel to `axis`, made exactly perpendicular via Gram-Schmidt, then
    // normalized - the reference direction a swing angle is measured from.
    static _perpendicularTo(axis) {
        const seed = Math.abs(axis.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
        const d = seed.x * axis.x + seed.y * axis.y + seed.z * axis.z;
        const perp = new Vector3(seed.x - d * axis.x, seed.y - d * axis.y, seed.z - d * axis.z);
        return perp.normalizeInPlace();
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
        if (this.limit.min != null || this.limit.max != null) this._solveLimit();
        if (this.motor.maxTorque > 0) this._solveMotor(h);
    }

    // Current swing angle about the hinge axis: the signed angle (right-hand rule about axisA,
    // range (-PI, PI]) from bodyA's world reference vector to bodyB's (or the fixed world
    // reference when bodyB is null), both projected into the plane perpendicular to the axis so a
    // small amount of axis misalignment (mid-solve, before _solveAxisAlignment has fully
    // converged) does not corrupt the angle measurement.
    _swingAngle() {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axis = HingeConstraint._scratchAxis.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axis);

        const refA = HingeConstraint._scratchV1.copy(this._refA);
        bodyA.rotation.transformVectorInPlace(refA);

        const refB = HingeConstraint._scratchV2;
        if (bodyB) { refB.copy(this._refB); bodyB.rotation.transformVectorInPlace(refB); }
        else refB.copy(this._fixedWorldRef);

        // Project both references into the plane perpendicular to axis, then measure the signed
        // angle FROM refB (the fixed/starting reference) TO refA (bodyA's current one) via
        // atan2(cross . axis, dot) - the standard signed-angle-about-an-axis formula, exact for any
        // angle (not a small-angle approximation like the axis-alignment correction above). This
        // order matters: atan2(refA x refB . axis, ...) gives the angle FROM the current reference
        // back TO the fixed one, i.e. the NEGATIVE of how far the body has actually swung - verified
        // directly against the body's own rotation quaternion (2*atan2(q.z, q.w) for a pure Z-axis
        // rotation) after gravity torqued a hinged plank, which caught this the wrong way round.
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

    // Clamps the swing angle to [limit.min, limit.max] via the same small-angle angular-correction
    // primitive _solveAxisAlignment uses: past a bound, rotate bodyB back toward bodyA by the
    // violation amount, distributed by each body's inverse inertia along the hinge axis.
    _solveLimit() {
        const angle = this._swingAngle();
        const min = this.limit.min != null ? this.limit.min : -Infinity;
        const max = this.limit.max != null ? this.limit.max : Infinity;
        let violation = 0;
        if (angle < min) violation = angle - min; // negative: rotate bodyB's angle UP toward min
        else if (angle > max) violation = angle - max; // positive: rotate bodyB's angle DOWN toward max
        else return;

        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axis = HingeConstraint._scratchAxis.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axis);

        let wSum = 0;
        const hasB = !!(bodyB && bodyB._massInverted > 0);
        if (bodyA._massInverted > 0) wSum += HingeConstraint._angularEffectiveMass(bodyA, axis.x, axis.y, axis.z);
        if (hasB) wSum += HingeConstraint._angularEffectiveMass(bodyB, axis.x, axis.y, axis.z);
        if (wSum < 1e-12) return;

        // deltaTheta = -violation / wSum, same Lagrange-multiplier shape as _solveAxisAlignment.
        // _swingAngle measures the angle FROM the fixed/bodyB reference TO bodyA's current one, so
        // increasing bodyA's own rotation about +axis directly increases the swing angle - verified
        // directly (_applyAngularDelta(bodyA, +axis*s) measured a positive swing-angle change), the
        // opposite sign from _solveAxisAlignment's bodyA call (that correction targets an axis-
        // alignment error, a different quantity with its own independently-verified sign).
        const scale = -violation / wSum;
        const tx = axis.x * scale, ty = axis.y * scale, tz = axis.z * scale;
        if (bodyA._massInverted > 0) HingeConstraint._applyAngularDelta(bodyA, tx, ty, tz);
        if (hasB) HingeConstraint._applyAngularDelta(bodyB, -tx, -ty, -tz);
    }

    // Drives the RELATIVE angular velocity about the hinge axis toward motor.targetVelocity - a
    // POSITION correction (matching how every other constraint here acts: the solver derives
    // velocity from the position delta once per substep AFTER all constraints run, Solver._substep
    // step 4, so a constraint that only ever wrote angular_velocity directly would have that write
    // silently discarded the instant step 4 ran).
    //
    // A torque-limited motor accelerates the relative angular velocity toward motor.targetVelocity
    // at angular acceleration alpha = maxTorque*wSum (wSum is the inverse angular moment along the
    // axis - the standard torque/inertia relation), never overshooting the target in one substep
    // (a real motor stops applying torque the instant it reaches the speed it was driving toward,
    // it does not fling the body past it). deltaOmega below is that bounded velocity CHANGE for
    // this substep; the position step it produces is deltaOmega*h, applied as a small-angle
    // rotation via _applyAngularDelta (a POSITION correction, matching every other constraint here
    // - the solver derives velocity from the position delta once per substep AFTER all constraints
    // run, Solver._substep step 4, so a constraint that only ever wrote angular_velocity directly
    // would have that write silently discarded the instant step 4 ran).
    _solveMotor(h) {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const axis = HingeConstraint._scratchAxis.copy(this.localAxisA);
        bodyA.rotation.transformVectorInPlace(axis);

        let wSum = 0;
        const hasB = !!(bodyB && bodyB._massInverted > 0);
        if (bodyA._massInverted > 0) wSum += HingeConstraint._angularEffectiveMass(bodyA, axis.x, axis.y, axis.z);
        if (hasB) wSum += HingeConstraint._angularEffectiveMass(bodyB, axis.x, axis.y, axis.z);
        if (wSum < 1e-12) return;

        // Rate of change of _swingAngle: bodyA's own rotation about +axis directly increases the
        // swing angle (same convention _solveLimit's own sign fix verified), and bodyB's (or a
        // fixed reference's) rotation the opposite way - so relOmega = wA - wB, not wB - wA.
        const wA = bodyA.angular_velocity, wB = hasB ? bodyB.angular_velocity : HingeConstraint._zero;
        const relOmega = (wA.x - wB.x) * axis.x + (wA.y - wB.y) * axis.y + (wA.z - wB.z) * axis.z;
        const velError = this.motor.targetVelocity - relOmega;
        if (velError === 0) return;

        // maxTorque bounds the angular velocity CHANGE this substep directly (deltaOmega =
        // maxTorque*wSum, the standard impulse/inverse-inertia relation) - not integrated by h
        // again on top of that. h already enters once, through step = deltaOmega*h below; an
        // extra *h here made the motor's real holding strength ~240x weaker than intended (verified
        // directly: a plank needing ~39 N*m to hold level against its own weight never held with
        // maxTorque=1 under the double-h version, but does under this one).
        const maxDeltaOmega = this.motor.maxTorque * wSum;
        const deltaOmega = velError > 0 ? Math.min(velError, maxDeltaOmega) : Math.max(velError, -maxDeltaOmega);
        let step = deltaOmega * h;
        if (step === 0) return;

        // Respect the limit while driving: clamp the step so it cannot push the swing angle past
        // an active bound (a motor holding against its own limit should stall there, not fight it
        // every substep only to be shoved back by _solveLimit next).
        if (this.limit.min != null || this.limit.max != null) {
            const angle = this._swingAngle();
            const min = this.limit.min != null ? this.limit.min : -Infinity;
            const max = this.limit.max != null ? this.limit.max : Infinity;
            if (step > 0 && angle + step > max) step = Math.max(0, max - angle);
            else if (step < 0 && angle + step < min) step = Math.min(0, min - angle);
            if (step === 0) return;
        }

        // Positive step increases the swing angle - same sign convention as _solveLimit, verified
        // the same way (_applyAngularDelta(bodyA, +axis*s) measures a positive swing-angle change).
        const scale = step / wSum;
        const tx = axis.x * scale, ty = axis.y * scale, tz = axis.z * scale;
        if (bodyA._massInverted > 0) HingeConstraint._applyAngularDelta(bodyA, tx, ty, tz);
        if (hasB) HingeConstraint._applyAngularDelta(bodyB, -tx, -ty, -tz);
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
HingeConstraint._scratchAxis = new Vector3();
HingeConstraint._scratchQ = new Quaternion();
HingeConstraint._scratchAngular = new Vector3();
HingeConstraint._zero = new Vector3();

ActionPhysics.HingeConstraint = HingeConstraint;
