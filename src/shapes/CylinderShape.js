// Axis is local Y. halfHeight is a half-extent, matching every other shape's convention
// (CapsuleShape's total-height constructor is the one deliberate exception).
class CylinderShape extends Shape {
    constructor(radius, halfHeight) {
        super('cylinder');
        this.radius = radius;
        this.halfHeight = halfHeight;
    }

    supportInto(out, direction) {
        const sigma = Math.sqrt(direction.x * direction.x + direction.z * direction.z);
        if (sigma > 0) {
            const s = this.radius / sigma;
            out.x = direction.x * s;
            out.z = direction.z * s;
        } else {
            out.x = 0;
            out.z = 0;
        }
        out.y = direction.y >= 0 ? this.halfHeight : -this.halfHeight;
        return out;
    }

    localAABBInto(out) {
        out.min.set(-this.radius, -this.halfHeight, -this.radius);
        out.max.set(this.radius, this.halfHeight, this.radius);
        return out;
    }

    volume() {
        return Scalar.PI * this.radius * this.radius * (2 * this.halfHeight);
    }

    computeMassData() {
        const r = this.radius, h = 2 * this.halfHeight;
        const mass = this.volume();
        const iAxis = 0.5 * mass * r * r;                                   // about Y
        const iSide = mass * (3 * r * r + h * h) / 12;                       // about X and Z
        const inertia = new Matrix3().setDiagonal(new Vector3(iSide, iAxis, iSide));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.CylinderShape = CylinderShape;
