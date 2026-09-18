Queries._advance = function (support, placedMover, start, dirX, dirY, dirZ, fullLen) {
    const ux = dirX / fullLen, uy = dirY / fullLen, uz = dirZ / fullLen;
    let traveled = 0;

    let lastGoodNx = -ux, lastGoodNy = -uy, lastGoodNz = -uz;

    for (let iter = 0; iter < 160; iter++) {
        const result = Queries._gjk.run(support);
        if (result.overlapping) {

            const epaResult = Queries._epa.run(support, result.simplex);
            return Queries._finishHit(start, dirX, dirY, dirZ, traveled, fullLen, epaResult.normal.x, epaResult.normal.y, epaResult.normal.z);
        }
        if (result.distance < 1e-4) {
            return Queries._finishHit(start, dirX, dirY, dirZ, traveled, fullLen, lastGoodNx, lastGoodNy, lastGoodNz);
        }
        lastGoodNx = result.normal.x; lastGoodNy = result.normal.y; lastGoodNz = result.normal.z;
        if (traveled + result.distance > fullLen) return null;
        traveled += result.distance;
        placedMover.position.set(start.x + ux * traveled, start.y + uy * traveled, start.z + uz * traveled);
        support.refresh();
    }
    return null;
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

Queries._rayIntersectsAABB = function (start, end, aabb) {
    let tmin = 0, tmax = 1;
    const dirs = [end.x - start.x, end.y - start.y, end.z - start.z];
    const starts = [start.x, start.y, start.z];
    const mins = [aabb.min.x, aabb.min.y, aabb.min.z];
    const maxs = [aabb.max.x, aabb.max.y, aabb.max.z];
    for (let axis = 0; axis < 3; axis++) {
        const d = dirs[axis], s = starts[axis];
        if (Math.abs(d) < 1e-12) {
            if (s < mins[axis] || s > maxs[axis]) return false;
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

Queries._sweptAABBMayHit = function (start, end, radius, aabb) {
    const expanded = Queries._scratchExpandedAABB;
    expanded.copy(aabb).expandInPlace(radius);
    return Queries._rayIntersectsAABB(start, end, expanded);
};
