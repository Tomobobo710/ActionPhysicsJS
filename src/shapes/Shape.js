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
}

ActionPhysics.Shape = Shape;
