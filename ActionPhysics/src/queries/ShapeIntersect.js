// shapeIntersect (swept-shape cast) and its compound/mesh dispatch, plus the stationary-overlap
// test used for a zero-length sweep.

// shapeIntersect(bodies, shape, start, end, rotation, ignore) -> same result shape as rayIntersect.
// Sweeps `shape` (fixed orientation) from start to end. `ignore`: see rayIntersect.
Queries.shapeIntersect = function (bodies, shape, start, end, rotation, ignore) {
    const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
    const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    // A zero-length sweep is a stationary overlap test - unlike a zero-length ray (degenerate,
    // reports a miss), a real shape held still genuinely can overlap something.
    if (fullLen < 1e-12) return Queries._overlapTest(bodies, shape, start, rotation, ignore);
    const localAABB = Queries._scratchLocalAABB;
    shape.localAABBInto(localAABB);
    const radius = Math.sqrt(
        Math.max(localAABB.min.x * localAABB.min.x, localAABB.max.x * localAABB.max.x) +
        Math.max(localAABB.min.y * localAABB.min.y, localAABB.max.y * localAABB.max.y) +
        Math.max(localAABB.min.z * localAABB.min.z, localAABB.max.z * localAABB.max.z)
    );

    let best = null, bestFraction = Infinity;
    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        if (Queries._isIgnored(body, ignore)) continue;
        const aabb = body.getAABB();
        if (!Queries._sweptAABBMayHit(start, end, radius, aabb)) continue;

        const hit = Queries._sweepShapeVsBody(shape, rotation, start, dirX, dirY, dirZ, fullLen, body);
        if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; best.body = body; }
    }
    return best;
};

Queries._sweepShapeVsBody = function (shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
    if (Queries._isCompound(body.shape)) {
        return Queries._sweepShapeVsCompound(shape, rotation, start, dirX, dirY, dirZ, fullLen, body);
    }
    if (Queries._isMesh(body.shape)) {
        return Queries._sweepShapeVsMesh(shape, rotation, start, dirX, dirY, dirZ, fullLen, body);
    }
    const placedShape = Queries._scratchPlacedA;
    placedShape.shape = shape;
    placedShape.position = Queries._scratchPos.set(start.x, start.y, start.z);
    placedShape.rotation = rotation || Queries._identityQuat;

    const placedBody = Queries._scratchPlacedB;
    placedBody.shape = body.shape;
    placedBody.position = body.position;
    placedBody.rotation = body.rotation;

    const support = Queries._scratchSupport;
    support.a = placedShape; support.b = placedBody;
    support._invRotA.copy(placedShape.rotation).invert();
    support._invRotB.copy(body.rotation).invert();

    return Queries._advance(support, placedShape, start, dirX, dirY, dirZ, fullLen);
};

Queries._sweepShapeVsCompound = function (shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
    const children = body.shape.children;
    let best = null, bestFraction = Infinity;
    for (let i = 0; i < children.length; i++) {
        const child = children[i];
        const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, child);

        const placedShape = Queries._scratchPlacedA;
        placedShape.shape = shape;
        placedShape.position = Queries._scratchPos.set(start.x, start.y, start.z);
        placedShape.rotation = rotation || Queries._identityQuat;

        const support = Queries._scratchSupport;
        support.a = placedShape; support.b = placedChild;
        support._invRotA.copy(placedShape.rotation).invert();
        support._invRotB.copy(placedChild.rotation).invert();

        const hit = Queries._advance(support, placedShape, start, dirX, dirY, dirZ, fullLen);
        if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
    }
    return best;
};

Queries._sweepShapeVsMesh = function (shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
    const meshShape = body.shape;
    const a = new Vector3(), b = new Vector3(), c = new Vector3();
    let best = null, bestFraction = Infinity;
    for (let i = 0; i < meshShape.triangleCount; i++) {
        meshShape.triangleAt(i, a, b, c);
        const placedTri = Queries._placedTriangleInto(Queries._scratchCompoundChild, body, Queries._scratchTriangleShape, a, b, c);

        const placedShape = Queries._scratchPlacedA;
        placedShape.shape = shape;
        placedShape.position = Queries._scratchPos.set(start.x, start.y, start.z);
        placedShape.rotation = rotation || Queries._identityQuat;

        const support = Queries._scratchSupport;
        support.a = placedShape; support.b = placedTri;
        support._invRotA.copy(placedShape.rotation).invert();
        support._invRotB.copy(placedTri.rotation).invert();

        const hit = Queries._advance(support, placedShape, start, dirX, dirY, dirZ, fullLen);
        if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
    }
    return best;
};

// Stationary overlap test: does `shape`, held fixed at `start`, touch anything? One GJK query per
// candidate, same AABB-reject structure as the swept queries, but EPA runs directly on an
// overlapping result (no travel direction to fall back on).
Queries._overlapTest = function (bodies, shape, start, rotation, ignore) {
    const localAABB = Queries._scratchLocalAABB;
    shape.localAABBInto(localAABB);
    const radius = Math.sqrt(
        Math.max(localAABB.min.x * localAABB.min.x, localAABB.max.x * localAABB.max.x) +
        Math.max(localAABB.min.y * localAABB.min.y, localAABB.max.y * localAABB.max.y) +
        Math.max(localAABB.min.z * localAABB.min.z, localAABB.max.z * localAABB.max.z)
    );
    const rot = rotation || Queries._identityQuat;

    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        if (Queries._isIgnored(body, ignore)) continue;
        if (Queries._isCompound(body.shape)) {
            const hit = Queries._overlapTestCompound(shape, start, rot, body);
            if (hit) return hit;
            continue;
        }
        if (Queries._isMesh(body.shape)) {
            const hit = Queries._overlapTestMesh(shape, start, rot, body);
            if (hit) return hit;
            continue;
        }
        const aabb = body.getAABB();
        const expanded = Queries._scratchExpandedAABB;
        expanded.copy(aabb).expandInPlace(radius);
        if (start.x < expanded.min.x || start.x > expanded.max.x ||
            start.y < expanded.min.y || start.y > expanded.max.y ||
            start.z < expanded.min.z || start.z > expanded.max.z) continue;

        const hit = Queries._overlapTestOne(shape, start, rot, body);
        if (hit) return hit;
    }
    return null;
};

Queries._overlapTestOne = function (shape, start, rotation, body) {
    const placedShape = Queries._scratchPlacedA;
    placedShape.shape = shape;
    placedShape.position = start;
    placedShape.rotation = rotation;

    const placedBody = Queries._scratchPlacedB;
    placedBody.shape = body.shape;
    placedBody.position = body.position;
    placedBody.rotation = body.rotation;

    const support = Queries._scratchSupport;
    support.a = placedShape; support.b = placedBody;
    support._invRotA.copy(rotation).invert();
    support._invRotB.copy(body.rotation).invert();

    const gjkResult = Queries._gjk.run(support);
    if (!gjkResult.overlapping) return null;
    const epaResult = Queries._epa.run(support, gjkResult.simplex);
    return { point: epaResult.pointA, normal: epaResult.normal, distance: 0, fraction: 0, body: body };
};

Queries._overlapTestCompound = function (shape, start, rotation, body) {
    const children = body.shape.children;
    for (let i = 0; i < children.length; i++) {
        const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, children[i]);
        const placedShape = Queries._scratchPlacedA;
        placedShape.shape = shape;
        placedShape.position = start;
        placedShape.rotation = rotation;

        const support = Queries._scratchSupport;
        support.a = placedShape; support.b = placedChild;
        support._invRotA.copy(rotation).invert();
        support._invRotB.copy(placedChild.rotation).invert();

        const gjkResult = Queries._gjk.run(support);
        if (!gjkResult.overlapping) continue;
        const epaResult = Queries._epa.run(support, gjkResult.simplex);
        return { point: epaResult.pointA, normal: epaResult.normal, distance: 0, fraction: 0, body: body };
    }
    return null;
};

Queries._overlapTestMesh = function (shape, start, rotation, body) {
    const meshShape = body.shape;
    const a = new Vector3(), b = new Vector3(), c = new Vector3();
    for (let i = 0; i < meshShape.triangleCount; i++) {
        meshShape.triangleAt(i, a, b, c);
        const placedTri = Queries._placedTriangleInto(Queries._scratchCompoundChild, body, Queries._scratchTriangleShape, a, b, c);
        const placedShape = Queries._scratchPlacedA;
        placedShape.shape = shape;
        placedShape.position = start;
        placedShape.rotation = rotation;

        const support = Queries._scratchSupport;
        support.a = placedShape; support.b = placedTri;
        support._invRotA.copy(rotation).invert();
        support._invRotB.copy(placedTri.rotation).invert();

        const gjkResult = Queries._gjk.run(support);
        if (!gjkResult.overlapping) continue;
        const epaResult = Queries._epa.run(support, gjkResult.simplex);
        return { point: epaResult.pointA, normal: epaResult.normal, distance: 0, fraction: 0, body: body };
    }
    return null;
};
