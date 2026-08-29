// A shape swept along a line segment from `start` to `end` (LOCAL-space points): the Minkowski sum
// of `shape` with that segment. Used for continuous-collision / swept queries
// without a dedicated CCD solver: the query just asks
// "does this swept volume touch anything", which is a support function away once the base shape
// has one.
class LineSweptShape extends Shape {
    constructor(shape, start, end) {
        super('lineswept');
        this.shape = shape;
        this.start = start;
        this.end = end;
        this.aabb = new AABB();
        this._recomputeAABB();
    }

    _recomputeAABB() {
        this.shape.localAABBInto(this.aabb);
        const sx = this.start.x, sy = this.start.y, sz = this.start.z;
        const ex = this.end.x, ey = this.end.y, ez = this.end.z;
        this.aabb.min.x += Math.min(sx, ex); this.aabb.max.x += Math.max(sx, ex);
        this.aabb.min.y += Math.min(sy, ey); this.aabb.max.y += Math.max(sy, ey);
        this.aabb.min.z += Math.min(sz, ez); this.aabb.max.z += Math.max(sz, ez);
        return this.aabb;
    }

    // Minkowski sum with a segment: support(d) = shape.support(d) + endpoint(d), where the
    // endpoint chosen is whichever end of the segment is farther along d (has the larger dot
    // product with d).
    supportInto(out, direction) {
        this.shape.supportInto(out, direction);
        const ds = this.start.x * direction.x + this.start.y * direction.y + this.start.z * direction.z;
        const de = this.end.x * direction.x + this.end.y * direction.y + this.end.z * direction.z;
        if (de >= ds) { out.x += this.end.x; out.y += this.end.y; out.z += this.end.z; }
        else { out.x += this.start.x; out.y += this.start.y; out.z += this.start.z; }
        return out;
    }

    // Point-in-shape support, matching RigidBody.findSupportPoint's own direct-shape convention.
    findSupportPoint(direction, out) {
        return this.supportInto(out, direction);
    }

    localAABBInto(out) {
        out.min.copy(this.aabb.min);
        out.max.copy(this.aabb.max);
        return out;
    }

    // A sweep is a query tool, not a body shape — it carries no mass properties of its own.
    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.LineSweptShape = LineSweptShape;
