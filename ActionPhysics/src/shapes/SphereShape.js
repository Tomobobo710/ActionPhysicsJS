class SphereShape extends Shape {
    constructor(radius) {
        super('sphere');
        this.radius = radius;
    }

    supportInto(out, direction) {
        // Direction need not be unit length; normalize here so the support point sits exactly
        // on the surface regardless of the caller's vector magnitude.
        const lsq = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (lsq === 0) { out.x = this.radius; out.y = 0; out.z = 0; return out; }
        const s = this.radius / Math.sqrt(lsq);
        out.x = direction.x * s; out.y = direction.y * s; out.z = direction.z * s;
        return out;
    }

    localAABBInto(out) {
        out.min.set(-this.radius, -this.radius, -this.radius);
        out.max.set(this.radius, this.radius, this.radius);
        return out;
    }

    volume() {
        return (4 / 3) * Scalar.PI * this.radius * this.radius * this.radius;
    }

    computeMassData() {
        const mass = this.volume();
        const i = 0.4 * mass * this.radius * this.radius; // solid sphere, density 1: I = 2/5 m r^2
        const inertia = new Matrix3().setDiagonal(new Vector3(i, i, i));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.SphereShape = SphereShape;
