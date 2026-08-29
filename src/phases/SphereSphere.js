// Closed-form sphere-sphere.
const SphereSphere = {};

SphereSphere.applies = function (placedA, placedB) {
    return placedA.shape instanceof SphereShape && placedB.shape instanceof SphereShape;
};

// Below this center-to-center distance the separating direction is undefined; use a fixed axis.
SphereSphere.DEGENERATE_EPSILON = 1e-9;

SphereSphere.test = function (placedA, placedB, out) {
    const ax = placedA.position.x, ay = placedA.position.y, az = placedA.position.z;
    const bx = placedB.position.x, by = placedB.position.y, bz = placedB.position.z;
    const dx = bx - ax, dy = by - ay, dz = bz - az;
    const distSq = dx * dx + dy * dy + dz * dz;
    const dist = Math.sqrt(distSq);
    const ra = placedA.shape.radius, rb = placedB.shape.radius;

    let nx, ny, nz;
    if (dist > SphereSphere.DEGENERATE_EPSILON) {
        // normal points B -> A, matching GJK/EPA's own convention.
        nx = -dx / dist; ny = -dy / dist; nz = -dz / dist;
    } else {
        nx = 0; ny = 1; nz = 0;
    }

    out.pointOnA.set(ax - nx * ra, ay - ny * ra, az - nz * ra);
    out.pointOnB.set(bx + nx * rb, by + ny * rb, bz + nz * rb);
    out.normal.set(nx, ny, nz);
    out.signedDistance = (ra + rb) - dist; // positive = overlapping, matching the pipeline convention
    Vector3.addInto(out.point, out.pointOnA, out.pointOnB).scaleInPlace(0.5);
    return out;
};
