class Shape {
    constructor(type) {
        this.type = type;
    }

    supportInto(out, direction) {
        throw new Error('Shape.supportInto not implemented for ' + this.type);
    }

    localAABBInto(out) {
        throw new Error('Shape.localAABBInto not implemented for ' + this.type);
    }

    computeMassData() {
        throw new Error('Shape.computeMassData not implemented for ' + this.type);
    }

    volume() {
        throw new Error('Shape.volume not implemented for ' + this.type);
    }

    getInertiaTensor(mass) {
        const out = new Matrix3();
        const vol = this.volume();
        if (!(mass > 0) || !(vol > 0)) { out.zero(); return out; }
        const s = mass / vol;
        const i = this.computeMassData().inertia;
        out.copy(i);
        out.e00 *= s; out.e01 *= s; out.e02 *= s;
        out.e10 *= s; out.e11 *= s; out.e12 *= s;
        out.e20 *= s; out.e21 *= s; out.e22 *= s;
        return out;
    }
}

ActionPhysics.Shape = Shape;
