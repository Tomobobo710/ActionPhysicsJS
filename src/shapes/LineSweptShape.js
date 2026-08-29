// A shape swept along a line segment: the Minkowski sum of `shape` with the segment from
// -halfLength to +halfLength along local Y. Used for continuous-collision / swept queries
// (plan.md, component 11: Queries — ray casting, shape sweeps) without a dedicated CCD solver:
// the query just asks "does this swept volume touch anything", which is a support function away
// once the base shape has one.
class LineSweptShape extends Shape {
    constructor(shape, halfLength) {
        super('lineswept');
        this.shape = shape;
        this.halfLength = halfLength;
    }

    // Minkowski sum with a segment: support(d) = shape.support(d) + endpoint(d), where the
    // endpoint chosen is whichever end of the segment is farther along d.
    supportInto(out, direction) {
        this.shape.supportInto(out, direction);
        out.y += direction.y >= 0 ? this.halfLength : -this.halfLength;
        return out;
    }

    localAABBInto(out) {
        this.shape.localAABBInto(out);
        out.min.y -= this.halfLength;
        out.max.y += this.halfLength;
        return out;
    }

    // A sweep is a query tool, not a body shape — it carries no mass properties of its own.
    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.LineSweptShape = LineSweptShape;
