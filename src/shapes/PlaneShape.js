class PlaneShape extends Shape {
    constructor(orientation, halfW, halfL) {
        super('plane');
        this.orientation = orientation;
        this.halfW = halfW;
        this.halfL = halfL;
    }

    supportInto(out, direction) {
        if (this.orientation === 'x') {
            out.x = 0;
            out.y = direction.y >= 0 ? this.halfW : -this.halfW;
            out.z = direction.z >= 0 ? this.halfL : -this.halfL;
        } else if (this.orientation === 'y') {
            out.x = direction.x >= 0 ? this.halfL : -this.halfL;
            out.y = 0;
            out.z = direction.z >= 0 ? this.halfW : -this.halfW;
        } else {
            out.x = direction.x >= 0 ? this.halfW : -this.halfW;
            out.y = direction.y >= 0 ? this.halfL : -this.halfL;
            out.z = 0;
        }
        return out;
    }

    localAABBInto(out) {
        if (this.orientation === 'x') out.setFromMinMax(0, -this.halfW, -this.halfL, 0, this.halfW, this.halfL);
        else if (this.orientation === 'y') out.setFromMinMax(-this.halfL, 0, -this.halfW, this.halfL, 0, this.halfW);
        else out.setFromMinMax(-this.halfW, -this.halfL, 0, this.halfW, this.halfL, 0);
        return out;
    }

    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.PlaneShape = PlaneShape;
