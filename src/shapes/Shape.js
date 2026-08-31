// Shape contract, all in local space: supportInto (the only GJK/EPA primitive), localAABBInto,
// computeMassData (density 1), volume (for density scaling). No allocation on supportInto/localAABBInto.
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

    // The local-space inertia tensor for this shape at total mass `mass`. computeMassData() returns
    // density-1 values, so this rescales by mass/volume - the same scaling RigidBody.setMassFromShape
    // applies. Returns a fresh Matrix3. mass <= 0 (or zero volume) gives the zero tensor.
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
