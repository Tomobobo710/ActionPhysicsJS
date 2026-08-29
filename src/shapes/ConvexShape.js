// Arbitrary convex hull from a point cloud. Points are LOCAL-space Vector3, taken as already
// forming (or being reducible to) a convex hull — support/mass computation below do not verify
// convexity, matching every other shape here trusting its constructor input.
class ConvexShape extends Shape {
    constructor(points) {
        super('convex');
        this.points = points;
    }

    // Brute-force max-dot scan. O(n) per query; fine for the hull sizes physics shapes use
    // (tens of points), and simplicity here keeps GJK's one required primitive easy to trust.
    supportInto(out, direction) {
        const pts = this.points;
        let bestDot = -Infinity, bestIndex = 0;
        for (let i = 0; i < pts.length; i++) {
            const d = pts[i].x * direction.x + pts[i].y * direction.y + pts[i].z * direction.z;
            if (d > bestDot) { bestDot = d; bestIndex = i; }
        }
        out.x = pts[bestIndex].x; out.y = pts[bestIndex].y; out.z = pts[bestIndex].z;
        return out;
    }

    localAABBInto(out) {
        out.setEmpty();
        const pts = this.points;
        for (let i = 0; i < pts.length; i++) {
            const p = pts[i];
            if (p.x < out.min.x) out.min.x = p.x;
            if (p.y < out.min.y) out.min.y = p.y;
            if (p.z < out.min.z) out.min.z = p.z;
            if (p.x > out.max.x) out.max.x = p.x;
            if (p.y > out.max.y) out.max.y = p.y;
            if (p.z > out.max.z) out.max.z = p.z;
        }
        return out;
    }

    // No closed-form volume/inertia for an arbitrary hull without its face list (which this
    // shape does not carry — see plan.md's component list: Convex is a GJK/EPA support shape,
    // not a tessellated mesh). Approximated as the equivalent-volume sphere from the AABB's
    // bounding radius; a caller needing exact mass properties for a hull supplies its own via
    // a MeshShape (has faces) instead.
    volume() {
        const aabb = new AABB();
        this.localAABBInto(aabb);
        const c = new Vector3();
        aabb.centerInto(c);
        let r = 0;
        for (let i = 0; i < this.points.length; i++) {
            const d = this.points[i].distanceTo(c);
            if (d > r) r = d;
        }
        this._boundingRadius = r;
        return (4 / 3) * Scalar.PI * r * r * r;
    }

    computeMassData() {
        const mass = this.volume();
        const r = this._boundingRadius;
        const i = 0.4 * mass * r * r;
        const inertia = new Matrix3().setDiagonal(new Vector3(i, i, i));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.ConvexShape = ConvexShape;
