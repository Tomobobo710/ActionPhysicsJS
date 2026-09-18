class ConeShape extends Shape {
    constructor(radius, halfHeight) {
        super('cone');
        this.radius = radius;
        this.halfHeight = halfHeight;
    }

    supportInto(out, direction) {
        const h = this.halfHeight;
        const sigma = Math.sqrt(direction.x * direction.x + direction.z * direction.z);

        const rimProjection = sigma * this.radius - direction.y * h;
        const apexProjection = direction.y * h;
        if (apexProjection >= rimProjection) {
            out.x = 0; out.y = h; out.z = 0;
            return out;
        }
        if (sigma > 0) {
            const s = this.radius / sigma;
            out.x = direction.x * s;
            out.z = direction.z * s;
        } else {
            out.x = 0; out.z = 0;
        }
        out.y = -h;
        return out;
    }

    localAABBInto(out) {
        out.min.set(-this.radius, -this.halfHeight, -this.radius);
        out.max.set(this.radius, this.halfHeight, this.radius);
        return out;
    }

    volume() {
        return Scalar.PI * this.radius * this.radius * (2 * this.halfHeight) / 3;
    }

    computeMassData() {
        const r = this.radius, h = 2 * this.halfHeight;
        const mass = this.volume();
        const iAxis = 0.3 * mass * r * r;
        const iSideApex = mass * (3 * r * r + 2 * h * h) / 20;

        const centroidOffset = h / 4;
        const iSideCentroid = iSideApex - mass * centroidOffset * centroidOffset;
        const originOffset = this.halfHeight - h / 4;
        const iSideOrigin = iSideCentroid + mass * originOffset * originOffset;
        const inertia = new Matrix3().setDiagonal(new Vector3(iSideOrigin, iAxis, iSideOrigin));
        return {
            mass: mass,
            inertia: inertia,
            centerOfMass: new Vector3(0, -this.halfHeight + h / 4, 0)
        };
    }
}

ActionPhysics.ConeShape = ConeShape;
