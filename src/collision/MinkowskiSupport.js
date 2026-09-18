class MinkowskiSupport {
    constructor(placedA, placedB) {
        this.a = placedA;
        this.b = placedB;
        this._localDir = new Vector3();
        this._invRotA = new Quaternion();
        this._invRotB = new Quaternion();
        this._invRotA.copy(placedA.rotation).invert();
        this._invRotB.copy(placedB.rotation).invert();

        this._scratchA = new Vector3();
        this._scratchB = new Vector3();
        this._scratchNeg = new Vector3();
    }

    refresh() {
        this._invRotA.copy(this.a.rotation).invert();
        this._invRotB.copy(this.b.rotation).invert();
        return this;
    }

    setSides(placedA, placedB) {
        this.a = placedA;
        this.b = placedB;
        return this.refresh();
    }

    static supportOfInto(out, placed, invRot, dir, scratchDir) {
        invRot.transformVectorInto(dir, scratchDir);
        placed.shape.supportInto(out, scratchDir);
        placed.rotation.transformVectorInPlace(out);
        out.addInPlace(placed.position);
        return out;
    }

    supportInto(out, dir, outA, outB) {
        const sa = outA || this._scratchA;
        const sb = outB || this._scratchB;
        MinkowskiSupport.supportOfInto(sa, this.a, this._invRotA, dir, this._localDir);
        const negDir = this._scratchNeg.set(-dir.x, -dir.y, -dir.z);
        MinkowskiSupport.supportOfInto(sb, this.b, this._invRotB, negDir, this._localDir);
        Vector3.subInto(out, sa, sb);
        return out;
    }
}

ActionPhysics.MinkowskiSupport = MinkowskiSupport;
