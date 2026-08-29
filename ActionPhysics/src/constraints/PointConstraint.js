/**
 * PointConstraint: pins a local anchor on bodyA to a local anchor on bodyB - a ball/socket joint.
 * 3 translational degrees of freedom removed, all rotation free (both bodies can spin independently
 * about the shared point). bodyB may be null: the joint pins bodyA's anchor to a fixed WORLD point
 * (localAnchorB, in that case, is read as a world-space point directly, not a local offset).
 *
 * XPBD position constraint (Muller et al. 2020 sec 3.4, the general rigid-body point constraint):
 * C = worldAnchorB - worldAnchorA, a VECTOR. Unlike the contact solver's scalar constraints (one
 * normal, two tangents, each solved as an independent 1D correction), a point constraint's 3 DOF are
 * coupled through each body's inertia tensor - solving x/y/z as three independent scalar passes only
 * approximately converges and takes many iterations to look rigid. This solves the full 3x3 coupled
 * system directly: K = (1/mA + 1/mB)*I3 - [rA×]*IA^-1*[rA×] - [rB×]*IB^-1*[rB×] (the generalized
 * inverse-mass matrix for a point constraint), then delta = K^-1 * (-C) is the exact correction that
 * satisfies the constraint in one solve, distributed to each body's position/rotation by its own
 * inverse mass and inertia - same physical idea as the contact solver's scalar effective mass, just
 * promoted to 3x3 because a point constraint removes 3 DOF at once, not 1.
 */
class PointConstraint extends Constraint {
    constructor(bodyA, bodyB, localAnchorA, localAnchorB) {
        super(bodyA, bodyB);
        this.localAnchorA = new Vector3().copy(localAnchorA);
        // If bodyB is null, localAnchorB is instead the fixed WORLD point to pin bodyA's anchor to.
        this.localAnchorB = new Vector3().copy(localAnchorB);

        this._worldA = new Vector3();
        this._worldB = new Vector3();
        this._rA = new Vector3();
        this._rB = new Vector3();
        this._C = new Vector3();
        this._delta = new Vector3();
        this._K = new Matrix3();
        this._Kinv = new Matrix3();
    }

    // Current world position of bodyA's anchor.
    _anchorAWorld(out) {
        out.copy(this.localAnchorA);
        this.bodyA.rotation.transformVectorInPlace(out);
        out.addInPlace(this.bodyA.position);
        return out;
    }

    // Current world position of bodyB's anchor - or the fixed world point if bodyB is null.
    _anchorBWorld(out) {
        if (!this.bodyB) { out.copy(this.localAnchorB); return out; }
        out.copy(this.localAnchorB);
        this.bodyB.rotation.transformVectorInPlace(out);
        out.addInPlace(this.bodyB.position);
        return out;
    }

    // Called once per substep by the solver, same cadence as the contact position solve. `h` is
    // accepted (unused - this joint is rigid, no compliance) to match the shape a compliant joint
    // would need (Muller et al.'s alpha/h^2 term), same as the contact solver's own C=0 rigid case.
    solve(h) {
        if (!this.enabled) return;
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const hasB = !!(bodyB && bodyB._massInverted > 0);

        this._anchorAWorld(this._worldA);
        this._anchorBWorld(this._worldB);
        Vector3.subInto(this._C, this._worldB, this._worldA); // C = worldB - worldA (B->A convention, matching the contact solver's own ordering)
        if (this._C.lengthSquared() < 1e-20) return; // already satisfied to numerical precision

        Vector3.subInto(this._rA, this._worldA, bodyA.position);
        if (hasB) Vector3.subInto(this._rB, this._worldB, bodyB.position);
        else this._rB.set(0, 0, 0);

        this._buildEffectiveMassMatrix(this._K, bodyA, hasB ? bodyB : null, this._rA, this._rB);
        if (!this._Kinv.invertInto(this._K)) return; // singular (both bodies immovable) - nothing to solve

        // delta = Kinv * (-C): the position correction that satisfies C=0 in one solve.
        const cx = -this._C.x, cy = -this._C.y, cz = -this._C.z;
        const K = this._Kinv;
        this._delta.set(
            K.e00 * cx + K.e01 * cy + K.e02 * cz,
            K.e10 * cx + K.e11 * cy + K.e12 * cz,
            K.e20 * cx + K.e21 * cy + K.e22 * cz
        );

        this._applyCorrection(bodyA, this._rA, this._delta, -1);
        if (hasB) this._applyCorrection(bodyB, this._rB, this._delta, 1);
    }

    // Builds the 3x3 generalized inverse-mass matrix K for this point constraint into `out`:
    // K = (1/mA + 1/mB)*I3 - [rA×]*IA^-1*[rA×] - [rB×]*IB^-1*[rB×]. The skew-symmetric cross-product
    // matrix terms couple the three translational DOF through each body's rotational inertia - this
    // is what makes the point constraint's 3 axes solve together instead of independently.
    _buildEffectiveMassMatrix(out, bodyA, bodyB, rA, rB) {
        const mSum = bodyA._massInverted + (bodyB ? bodyB._massInverted : 0);
        out.e00 = mSum; out.e01 = 0; out.e02 = 0;
        out.e10 = 0; out.e11 = mSum; out.e12 = 0;
        out.e20 = 0; out.e21 = 0; out.e22 = mSum;
        if (bodyA._massInverted > 0) PointConstraint._subtractSkewInertiaSkew(out, rA, bodyA._worldInverseInertiaTensor);
        if (bodyB && bodyB._massInverted > 0) PointConstraint._subtractSkewInertiaSkew(out, rB, bodyB._worldInverseInertiaTensor);
    }

    // out -= [r×]^T * I * [r×], where [r×] is the skew-symmetric cross-product matrix of r
    // ( [r×]v = r x v ). This is the standard rigid-body coupling term between a linear positional
    // correction and the rotation it induces at an offset r under inverse inertia I.
    static _subtractSkewInertiaSkew(out, r, I) {
        // [r×] = |  0  -rz  ry |
        //        |  rz  0  -rx |
        //        | -ry  rx  0  |
        const rx = r.x, ry = r.y, rz = r.z;
        // M = I * [r×] (3x3), columns computed directly from I's rows and [r×]'s columns.
        const m00 = I.e01 * rz - I.e02 * ry, m01 = -I.e00 * rz + I.e02 * rx, m02 = I.e00 * ry - I.e01 * rx;
        const m10 = I.e11 * rz - I.e12 * ry, m11 = -I.e10 * rz + I.e12 * rx, m12 = I.e10 * ry - I.e11 * rx;
        const m20 = I.e21 * rz - I.e22 * ry, m21 = -I.e20 * rz + I.e22 * rx, m22 = I.e20 * ry - I.e21 * rx;
        // out -= [r×]^T * M  ( [r×]^T = -[r×] )
        out.e00 -= (-rz * m10 + ry * m20); out.e01 -= (-rz * m11 + ry * m21); out.e02 -= (-rz * m12 + ry * m22);
        out.e10 -= (rz * m00 - rx * m20); out.e11 -= (rz * m01 - rx * m21); out.e12 -= (rz * m02 - rx * m22);
        out.e20 -= (-ry * m00 + rx * m10); out.e21 -= (-ry * m01 + rx * m11); out.e22 -= (-ry * m02 + rx * m12);
    }

    // Applies a linear + angular position correction from a 3-DOF impulse `delta` (already solved),
    // scaled by `sign` (-1 for bodyA, +1 for bodyB, matching C = worldB - worldA's own gradient -
    // same B-to-A convention the contact solver's _applyPositionalCorrection uses).
    _applyCorrection(body, r, delta, sign) {
        if (body._massInverted <= 0) return;
        body.position.x += sign * delta.x * body._massInverted * body.linear_factor.x;
        body.position.y += sign * delta.y * body._massInverted * body.linear_factor.y;
        body.position.z += sign * delta.z * body._massInverted * body.linear_factor.z;

        const px = sign * delta.x, py = sign * delta.y, pz = sign * delta.z;
        const torqueX = r.y * pz - r.z * py, torqueY = r.z * px - r.x * pz, torqueZ = r.x * py - r.y * px;
        const I = body._worldInverseInertiaTensor;
        const wx = I.e00 * torqueX + I.e01 * torqueY + I.e02 * torqueZ;
        const wy = I.e10 * torqueX + I.e11 * torqueY + I.e12 * torqueZ;
        const wz = I.e20 * torqueX + I.e21 * torqueY + I.e22 * torqueZ;
        PointConstraint._scratchAngular.set(wx * body.angular_factor.x, wy * body.angular_factor.y, wz * body.angular_factor.z);
        Solver._integrateRotation(body.rotation, PointConstraint._scratchAngular, 1);
    }
}

PointConstraint._scratchAngular = new Vector3();

ActionPhysics.PointConstraint = PointConstraint;
