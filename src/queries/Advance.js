// Shared conservative-advancement sweep core, plus the AABB reject tests both rayIntersect and
// shapeIntersect use before ever constructing a GJK support for a candidate body.

// Casts `placedMover` (already at `start`) toward `start + dir*fullLen` via conservative
// advancement using GJK.run() as the distance oracle: GJK's separated-case distance from the
// mover's current position to the body is the TRUE closest distance in any direction, so the mover
// can safely advance by exactly that much along the ray without ever overshooting into the body.
// Repeating (re-running GJK from the new position) narrows in on the true first-hit point -
// standard sphere-tracing conservative advancement, using an already-proven GJK as the primitive
// instead of a hand-rolled ray/shape algorithm (a hand-rolled version was tried twice and failed a
// 500-config ground-truth cross-check both times on corner/grazing cases).
//
// Convergence for a corner-on or near-tangent approach is geometric (never reaching exactly zero),
// so the iteration cap/epsilon (160, 1e-4) are set generously.
Queries._advance = function (support, placedMover, start, dirX, dirY, dirZ, fullLen) {
    const ux = dirX / fullLen, uy = dirY / fullLen, uz = dirZ / fullLen;
    let traveled = 0;
    // Last normal from a non-degenerate GJK call. GJK's own exact-touch case has no unique normal
    // at distance ~0 and falls back to an arbitrary-but-valid one - using the LAST good approach
    // normal instead avoids ever trusting that degenerate fallback (a ray hitting a flat box face
    // was previously reporting a spurious 45-degree diagonal normal from exactly this).
    let lastGoodNx = -ux, lastGoodNy = -uy, lastGoodNz = -uz;

    for (let iter = 0; iter < 160; iter++) {
        const result = Queries._gjk.run(support);
        if (result.overlapping) {
            // Already inside/touching: EPA expands the same simplex into a real surface normal.
            // This (not a reversed-travel-direction fallback) is what correctly handles a sweep that
            // starts embedded - the reversed-direction fallback can't tell "approaching a surface
            // ahead" from "already past it and moving away".
            const epaResult = Queries._epa.run(support, result.simplex);
            return Queries._finishHit(start, dirX, dirY, dirZ, traveled, fullLen, epaResult.normal.x, epaResult.normal.y, epaResult.normal.z);
        }
        if (result.distance < 1e-4) {
            return Queries._finishHit(start, dirX, dirY, dirZ, traveled, fullLen, lastGoodNx, lastGoodNy, lastGoodNz);
        }
        lastGoodNx = result.normal.x; lastGoodNy = result.normal.y; lastGoodNz = result.normal.z;
        if (traveled + result.distance > fullLen) return null; // cannot reach within the segment
        traveled += result.distance;
        placedMover.position.set(start.x + ux * traveled, start.y + uy * traveled, start.z + uz * traveled);
        support.refresh();
    }
    return null; // did not converge within the cap - treat as a miss, never a false hit
};

Queries._finishHit = function (start, dirX, dirY, dirZ, traveled, fullLen, nx, ny, nz) {
    const fraction = traveled / fullLen;
    return {
        point: new Vector3(start.x + dirX * fraction, start.y + dirY * fraction, start.z + dirZ * fraction),
        normal: new Vector3(nx, ny, nz),
        distance: traveled,
        fraction: fraction
    };
};

// Cheap ray-vs-AABB reject (slab method) before ever constructing a GJK support for a body.
Queries._rayIntersectsAABB = function (start, end, aabb) {
    let tmin = 0, tmax = 1;
    const dirs = [end.x - start.x, end.y - start.y, end.z - start.z];
    const starts = [start.x, start.y, start.z];
    const mins = [aabb.min.x, aabb.min.y, aabb.min.z];
    const maxs = [aabb.max.x, aabb.max.y, aabb.max.z];
    for (let axis = 0; axis < 3; axis++) {
        const d = dirs[axis], s = starts[axis];
        if (Math.abs(d) < 1e-12) {
            if (s < mins[axis] || s > maxs[axis]) return false; // parallel and outside the slab
            continue;
        }
        let t1 = (mins[axis] - s) / d, t2 = (maxs[axis] - s) / d;
        if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return false;
    }
    return true;
};

// Cheap reject for a shape sweep: expand the body's AABB by the swept shape's bounding radius
// (conservative, no-false-negatives, same discipline as broadphase's own margin) and ray-test that.
Queries._sweptAABBMayHit = function (start, end, radius, aabb) {
    const expanded = Queries._scratchExpandedAABB;
    expanded.copy(aabb).expandInPlace(radius);
    return Queries._rayIntersectsAABB(start, end, expanded);
};
