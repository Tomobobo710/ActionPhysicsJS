/**
 * Shape contract. Every shape provides, in LOCAL space (unrotated, centered on its own origin):
 *
 *   supportInto(out, direction)   farthest point on the shape along `direction` (need not be
 *                                 normalized). This is the ONLY primitive GJK/EPA require —
 *                                 everything else here exists for AABBs, mass properties and
 *                                 the visual bench, not collision.
 *   localAABBInto(out)            tight local-space AABB.
 *   computeMassData()             { mass, inertia: Matrix3, centerOfMass: Vector3 } for a shape
 *                                 of density 1; RigidBody scales inertia by (mass / this.volume)
 *                                 when the caller supplies its own mass.
 *   volume()                      for the density scaling above.
 *
 * Shapes never allocate on the hot path: supportInto/localAABBInto write into caller-owned
 * `out` arguments. computeMassData() runs once per body and may allocate.
 */
class Shape {
    // A shape reports the CATEGORY of margin its narrowphase pair needs. Plane and Triangle are
    // degenerate (infinite extent / zero thickness) and get special-cased at dispatch rather than
    // patched inside GJK/EPA — see plan.md, Shapes section.
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
