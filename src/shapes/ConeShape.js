// Axis is local Y, apex at +halfHeight, base circle at -halfHeight. halfHeight is a half-extent.
class ConeShape extends Shape {
    constructor(radius, halfHeight) {
        super('cone');
        this.radius = radius;
        this.halfHeight = halfHeight;
    }

    // Exact support of a cone is either the apex or a base-rim point, chosen by whichever the
    // direction favors — no iteration needed, unlike a general convex hull.
    supportInto(out, direction) {
        const h = this.halfHeight;
        const sigma = Math.sqrt(direction.x * direction.x + direction.z * direction.z);
        // Base-rim candidate's projection onto `direction`, compared against the apex's.
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

    // Solid cone, density 1, apex up. Standard formulas are about the base; centerOfMass shifts
    // the origin from local (0,0,0) — the geometric mid-height used for the AABB and support
    // function — to that centroid, at h/4 above the base (i.e. -halfHeight + h/4).
    computeMassData() {
        const r = this.radius, h = 2 * this.halfHeight;
        const mass = this.volume();
        const iAxis = 0.3 * mass * r * r;                                    // about Y, apex frame
        const iSideApex = mass * (3 * r * r + 2 * h * h) / 20;               // about X/Z through apex
        // Parallel-axis shift from the apex-based formula to the centroid (h/4 below apex along axis).
        const centroidOffset = h / 4;
        const iSideCentroid = iSideApex - mass * centroidOffset * centroidOffset;
        const inertia = new Matrix3().setDiagonal(new Vector3(iSideCentroid, iAxis, iSideCentroid));
        return {
            mass: mass,
            inertia: inertia,
            centerOfMass: new Vector3(0, -this.halfHeight + centroidOffset, 0)
        };
    }
}

ActionPhysics.ConeShape = ConeShape;
