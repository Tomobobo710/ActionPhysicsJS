// rayIntersect and its compound/mesh point-sweep dispatch.

// rayIntersect(bodies, start, end, ignore) -> { body, point, normal, distance, fraction } | null.
// The first body the segment hits, or null. `ignore`: a single RigidBody or array, excluded before
// the AABB reject (a caller casting from its own surface would otherwise hit itself at distance 0).
Queries.rayIntersect = function (bodies, start, end, ignore) {
    const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
    const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    if (fullLen < 1e-12) return null; // zero-length ray hits nothing

    let best = null, bestFraction = Infinity;
    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        if (Queries._isIgnored(body, ignore)) continue;
        const aabb = body.getAABB();
        if (!Queries._rayIntersectsAABB(start, end, aabb)) continue;

        const hit = Queries._sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body);
        if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; best.body = body; }
    }
    return best;
};

// Same result shape, against exactly one known body - no candidate filtering/AABB reject. What
// RigidBody.rayIntersect delegates to.
Queries.rayIntersectBody = function (start, end, body) {
    const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
    const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    if (fullLen < 1e-12) return null;
    return Queries._sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body);
};

// Casts a zero-radius point against one body via a single GJK query. CompoundShape isn't itself
// convex (supportInto throws by design), so a compound body dispatches per child.
Queries._sweepPointVsBody = function (start, dirX, dirY, dirZ, fullLen, body) {
    if (Queries._isCompound(body.shape)) {
        return Queries._sweepPointVsCompound(start, dirX, dirY, dirZ, fullLen, body);
    }
    if (Queries._isMesh(body.shape)) {
        return Queries._sweepPointVsMesh(start, dirX, dirY, dirZ, fullLen, body);
    }
    const pointShape = Queries._scratchPointShape;
    const placedPoint = Queries._scratchPlacedA;
    placedPoint.shape = pointShape;
    placedPoint.position = Queries._scratchPos.set(start.x, start.y, start.z);
    placedPoint.rotation = Queries._identityQuat;

    const placedBody = Queries._scratchPlacedB;
    placedBody.shape = body.shape;
    placedBody.position = body.position;
    placedBody.rotation = body.rotation;

    const support = Queries._scratchSupport;
    support.a = placedPoint; support.b = placedBody;
    support._invRotA.copy(Queries._identityQuat);
    support._invRotB.copy(body.rotation).invert();

    return Queries._advance(support, placedPoint, start, dirX, dirY, dirZ, fullLen);
};

Queries._sweepPointVsCompound = function (start, dirX, dirY, dirZ, fullLen, body) {
    const children = body.shape.children;
    let best = null, bestFraction = Infinity;
    for (let i = 0; i < children.length; i++) {
        const child = children[i];
        const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, child);

        const pointShape = Queries._scratchPointShape;
        const placedPoint = Queries._scratchPlacedA;
        placedPoint.shape = pointShape;
        placedPoint.position = Queries._scratchPos.set(start.x, start.y, start.z);
        placedPoint.rotation = Queries._identityQuat;

        const support = Queries._scratchSupport;
        support.a = placedPoint; support.b = placedChild;
        support._invRotA.copy(Queries._identityQuat);
        support._invRotB.copy(placedChild.rotation).invert();

        const hit = Queries._advance(support, placedPoint, start, dirX, dirY, dirZ, fullLen);
        if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
    }
    return best;
};

Queries._sweepPointVsMesh = function (start, dirX, dirY, dirZ, fullLen, body) {
    const shape = body.shape;
    const a = new Vector3(), b = new Vector3(), c = new Vector3();
    let best = null, bestFraction = Infinity;
    for (let i = 0; i < shape.triangleCount; i++) {
        shape.triangleAt(i, a, b, c);
        const placedTri = Queries._placedTriangleInto(Queries._scratchCompoundChild, body, Queries._scratchTriangleShape, a, b, c);

        const pointShape = Queries._scratchPointShape;
        const placedPoint = Queries._scratchPlacedA;
        placedPoint.shape = pointShape;
        placedPoint.position = Queries._scratchPos.set(start.x, start.y, start.z);
        placedPoint.rotation = Queries._identityQuat;

        const support = Queries._scratchSupport;
        support.a = placedPoint; support.b = placedTri;
        support._invRotA.copy(Queries._identityQuat);
        support._invRotB.copy(placedTri.rotation).invert();

        const hit = Queries._advance(support, placedPoint, start, dirX, dirY, dirZ, fullLen);
        if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
    }
    return best;
};
