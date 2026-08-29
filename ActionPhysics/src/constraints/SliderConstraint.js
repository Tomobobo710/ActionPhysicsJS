/**
 * SliderConstraint: two bodies (or one body and the world) may slide relative to each other ONLY
 * along a shared axis, like a piston - all 3 rotational DOF locked (identical to WeldConstraint's
 * own rotation lock, reused directly below - one owner, Rule 2) plus 2 of the 3 translational DOF
 * locked (position perpendicular to the slide axis; motion ALONG the axis stays free).
 *
 * The rotation lock is a fully-corrected XPBD position constraint - WeldConstraint's own angular
 * half, reused by composition.
 *
 * XPBD position constraint for the linear half: C = (worldPointB - worldPointA) projected onto the
 * plane PERPENDICULAR to the slide axis (the along-axis component is discarded before solving, so
 * it is never corrected - that is what leaves sliding free). Solved as a coupled 2x2 system in the
 * plane's own basis (two tangent directions spanning perpendicular-to-axis), the same generalized-
 * inverse-mass idea as PointConstraint's 3x3 solve, just restricted to 2 DOF instead of 3.
 */
class SliderConstraint extends Constraint {
    // localAxisA is in bodyA's local space (the slide direction). anchorA/anchorB are local points
    // on each body that should stay coincident ALONG the perpendicular plane (their separation along
    // the axis itself is the piston's free travel). bodyB may be null (slides relative to the world).
    constructor(bodyA, localAxisA, anchorA, bodyB, anchorB) {
        super(bodyA, bodyB);
        this.localAxis = new Vector3().copy(localAxisA).normalizeInPlace();
        this.localAnchorA = new Vector3().copy(anchorA);
        this.localAnchorB = new Vector3().copy(anchorB || new Vector3());

        // Rotation lock: reuse WeldConstraint's own angular half directly by constructing one and
        // only ever calling ITS rotation solve, never its pivot solve (the slider has its own,
        // axis-restricted, positional constraint below - Weld's own unrestricted pivot would defeat
        // sliding entirely if used here).
        this._weld = new WeldConstraint(bodyA, bodyB, new Vector3(), new Vector3()); // pivots unused - only _solveRotationLock runs

        this._worldA = new Vector3();
        this._worldB = new Vector3();
        this._rA = new Vector3();
        this._rB = new Vector3();
        this._t1 = new Vector3();
        this._t2 = new Vector3();
    }

    _anchorAWorld(out) {
        out.copy(this.localAnchorA);
        this.bodyA.rotation.transformVectorInPlace(out);
        out.addInPlace(this.bodyA.position);
        return out;
    }
    _anchorBWorld(out) {
        if (!this.bodyB) { out.copy(this.localAnchorB); return out; }
        out.copy(this.localAnchorB);
        this.bodyB.rotation.transformVectorInPlace(out);
        out.addInPlace(this.bodyB.position);
        return out;
    }

    solve(h) {
        if (!this.enabled) return;
        this._weld._solveRotationLock();
        this._solvePerpendicularPosition();
    }

    _solvePerpendicularPosition() {
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const hasB = !!(bodyB && bodyB._massInverted > 0);

        this._anchorAWorld(this._worldA);
        this._anchorBWorld(this._worldB);

        // World-space slide axis (bodyA's frame owns the axis direction, per constructor doc).
        const axis = SliderConstraint._scratchAxis;
        axis.copy(this.localAxis);
        bodyA.rotation.transformVectorInPlace(axis);

        // Full separation, then strip the along-axis component - what's left is exactly the
        // perpendicular error this constraint corrects.
        const sepX = this._worldB.x - this._worldA.x, sepY = this._worldB.y - this._worldA.y, sepZ = this._worldB.z - this._worldA.z;
        const along = sepX * axis.x + sepY * axis.y + sepZ * axis.z;
        const cx = sepX - along * axis.x, cy = sepY - along * axis.y, cz = sepZ - along * axis.z;
        const errLenSq = cx * cx + cy * cy + cz * cz;
        if (errLenSq < 1e-20) return;

        Vector3.subInto(this._rA, this._worldA, bodyA.position);
        if (hasB) Vector3.subInto(this._rB, this._worldB, bodyB.position);
        else this._rB.set(0, 0, 0);

        // Two tangent directions spanning the plane perpendicular to axis - reuse the same
        // find-orthogonal + cross idiom the contact solver's friction basis already uses.
        const t1 = this._t1, t2 = this._t2;
        t1.findOrthogonal(axis);
        Vector3.crossInto(t2, axis, t1);

        const c1 = cx * t1.x + cy * t1.y + cz * t1.z;
        const c2 = cx * t2.x + cy * t2.y + cz * t2.z;

        this._solveAxis(bodyA, hasB ? bodyB : null, this._rA, this._rB, t1, c1);
        this._solveAxis(bodyA, hasB ? bodyB : null, this._rA, this._rB, t2, c2);
    }

    // One scalar axis of the perpendicular correction - same shape as the contact solver's
    // _solvePoint / friction axis: effective mass along `dir`, deltaLambda = -C/wSum, apply.
    _solveAxis(bodyA, bodyB, rA, rB, dir, C) {
        const dx = dir.x, dy = dir.y, dz = dir.z;
        let wSum = bodyA._massInverted + (bodyB ? bodyB._massInverted : 0);
        const rax = rA.y * dz - rA.z * dy, ray = rA.z * dx - rA.x * dz, raz = rA.x * dy - rA.y * dx;
        if (bodyA._massInverted > 0) {
            const IA = bodyA._worldInverseInertiaTensor;
            const ix = IA.e00 * rax + IA.e01 * ray + IA.e02 * raz;
            const iy = IA.e10 * rax + IA.e11 * ray + IA.e12 * raz;
            const iz = IA.e20 * rax + IA.e21 * ray + IA.e22 * raz;
            wSum += rax * ix + ray * iy + raz * iz;
        }
        const rbx = rB.y * dz - rB.z * dy, rby = rB.z * dx - rB.x * dz, rbz = rB.x * dy - rB.y * dx;
        if (bodyB && bodyB._massInverted > 0) {
            const IB = bodyB._worldInverseInertiaTensor;
            const ix = IB.e00 * rbx + IB.e01 * rby + IB.e02 * rbz;
            const iy = IB.e10 * rbx + IB.e11 * rby + IB.e12 * rbz;
            const iz = IB.e20 * rbx + IB.e21 * rby + IB.e22 * rbz;
            wSum += rbx * ix + rby * iy + rbz * iz;
        }
        if (wSum < 1e-12) return;

        const deltaLambda = -C / wSum; // rigid, no accumulated warm-start (matches Weld/Point/Hinge - no compliance yet)
        const px = dx * deltaLambda, py = dy * deltaLambda, pz = dz * deltaLambda;

        if (bodyA._massInverted > 0) {
            bodyA.position.x -= px * bodyA._massInverted * bodyA.linear_factor.x;
            bodyA.position.y -= py * bodyA._massInverted * bodyA.linear_factor.y;
            bodyA.position.z -= pz * bodyA._massInverted * bodyA.linear_factor.z;
            SliderConstraint._applyAngular(bodyA, rA, -px, -py, -pz);
        }
        if (bodyB && bodyB._massInverted > 0) {
            bodyB.position.x += px * bodyB._massInverted * bodyB.linear_factor.x;
            bodyB.position.y += py * bodyB._massInverted * bodyB.linear_factor.y;
            bodyB.position.z += pz * bodyB._massInverted * bodyB.linear_factor.z;
            SliderConstraint._applyAngular(bodyB, rB, px, py, pz);
        }
    }

    static _applyAngular(body, r, px, py, pz) {
        const torqueX = r.y * pz - r.z * py, torqueY = r.z * px - r.x * pz, torqueZ = r.x * py - r.y * px;
        const I = body._worldInverseInertiaTensor;
        const wx = I.e00 * torqueX + I.e01 * torqueY + I.e02 * torqueZ;
        const wy = I.e10 * torqueX + I.e11 * torqueY + I.e12 * torqueZ;
        const wz = I.e20 * torqueX + I.e21 * torqueY + I.e22 * torqueZ;
        SliderConstraint._scratchAngular.set(wx * body.angular_factor.x, wy * body.angular_factor.y, wz * body.angular_factor.z);
        Solver._integrateRotation(body.rotation, SliderConstraint._scratchAngular, 1);
    }
}

SliderConstraint._scratchAxis = new Vector3();
SliderConstraint._scratchAngular = new Vector3();

ActionPhysics.SliderConstraint = SliderConstraint;
