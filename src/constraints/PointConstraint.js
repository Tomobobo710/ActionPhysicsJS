// Ball/socket joint: pins bodyA's local anchor to bodyB's (or a fixed world point if bodyB null).
// Full 3x3 coupled XPBD solve (C = worldB - worldA), not 3 independent scalar passes.
class PointConstraint extends Constraint {
    constructor(bodyA, bodyB, localAnchorA, localAnchorB) {
        super(bodyA, bodyB);
        this.localAnchorA = new Vector3().copy(localAnchorA);
        this.localAnchorB = new Vector3().copy(localAnchorB); // world point if bodyB is null

        this._worldA = new Vector3();
        this._worldB = new Vector3();
        this._rA = new Vector3();
        this._rB = new Vector3();
        this._C = new Vector3();
        this._delta = new Vector3();
        this._K = new Matrix3();
        this._Kinv = new Matrix3();
        this.breaking_threshold = null; // null = never breaks
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
        const bodyA = this.bodyA, bodyB = this.bodyB;
        const hasB = !!(bodyB && bodyB._massInverted > 0);

        this._anchorAWorld(this._worldA);
        this._anchorBWorld(this._worldB);
        Vector3.subInto(this._C, this._worldB, this._worldA);
        if (this._C.lengthSquared() < 1e-20) return;

        Vector3.subInto(this._rA, this._worldA, bodyA.position);
        if (hasB) Vector3.subInto(this._rB, this._worldB, bodyB.position);
        else this._rB.set(0, 0, 0);

        this._buildEffectiveMassMatrix(this._K, bodyA, hasB ? bodyB : null, this._rA, this._rB);
        if (!this._Kinv.invertInto(this._K)) return;

        const cx = -this._C.x, cy = -this._C.y, cz = -this._C.z;
        const K = this._Kinv;
        this._delta.set(
            K.e00 * cx + K.e01 * cy + K.e02 * cz,
            K.e10 * cx + K.e11 * cy + K.e12 * cz,
            K.e20 * cx + K.e21 * cy + K.e22 * cz
        );

        // delta/h^2 is the force-equivalent breaking_threshold checks - raw C alone stays near zero
        // regardless of load.
        if (this.breaking_threshold != null && this._delta.length() / (h * h) > this.breaking_threshold) {
            this.enabled = false;
            return;
        }

        this._applyCorrection(bodyA, this._rA, this._delta, -1);
        if (hasB) this._applyCorrection(bodyB, this._rB, this._delta, 1);
    }

    // K = (1/mA + 1/mB)*I3 - [rA×]*IA^-1*[rA×] - [rB×]*IB^-1*[rB×].
    _buildEffectiveMassMatrix(out, bodyA, bodyB, rA, rB) {
        const mSum = bodyA._massInverted + (bodyB ? bodyB._massInverted : 0);
        out.e00 = mSum; out.e01 = 0; out.e02 = 0;
        out.e10 = 0; out.e11 = mSum; out.e12 = 0;
        out.e20 = 0; out.e21 = 0; out.e22 = mSum;
        if (bodyA._massInverted > 0) PointConstraint._subtractSkewInertiaSkew(out, rA, bodyA._worldInverseInertiaTensor);
        if (bodyB && bodyB._massInverted > 0) PointConstraint._subtractSkewInertiaSkew(out, rB, bodyB._worldInverseInertiaTensor);
    }

    // out -= [r×]^T * I * [r×]
    static _subtractSkewInertiaSkew(out, r, I) {
        const rx = r.x, ry = r.y, rz = r.z;
        const m00 = I.e01 * rz - I.e02 * ry, m01 = -I.e00 * rz + I.e02 * rx, m02 = I.e00 * ry - I.e01 * rx;
        const m10 = I.e11 * rz - I.e12 * ry, m11 = -I.e10 * rz + I.e12 * rx, m12 = I.e10 * ry - I.e11 * rx;
        const m20 = I.e21 * rz - I.e22 * ry, m21 = -I.e20 * rz + I.e22 * rx, m22 = I.e20 * ry - I.e21 * rx;
        out.e00 -= (-rz * m10 + ry * m20); out.e01 -= (-rz * m11 + ry * m21); out.e02 -= (-rz * m12 + ry * m22);
        out.e10 -= (rz * m00 - rx * m20); out.e11 -= (rz * m01 - rx * m21); out.e12 -= (rz * m02 - rx * m22);
        out.e20 -= (-ry * m00 + rx * m10); out.e21 -= (-ry * m01 + rx * m11); out.e22 -= (-ry * m02 + rx * m12);
    }

    // sign: -1 for bodyA, +1 for bodyB (matches C = worldB - worldA).
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
