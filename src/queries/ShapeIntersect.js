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
    placedBody.position = Queries._scratchPlacedBPos.copy(body.position);
    placedBody.rotation = body.rotation;

    const support = Queries._scratchSupport;
    support.a = placedShape; support.b = placedBody;
    support._invRotA.copy(placedShape.rotation).invert();
    support._invRotB.copy(body.rotation).invert();

    return Queries._advance(support, placedShape, start, dirX, dirY, dirZ, fullLen);
};

// Local-space AABB of the swept region: the segment [start, start+dir] fattened by `pad` (the
// moving shape's bounding radius), inverse-transformed into `body`'s frame. Conservative.
Queries._localSweptAABBInto = function (out, body, start, dirX, dirY, dirZ, pad) {
    const invRot = Queries._scratchInvRot.copy(body.rotation).invert();
    out.setEmpty();
    for (let k = 0; k < 2; k++) {
        const wx = start.x + (k ? dirX : 0), wy = start.y + (k ? dirY : 0), wz = start.z + (k ? dirZ : 0);
        Queries._scratchCorner.set(wx - body.position.x, wy - body.position.y, wz - body.position.z);
        invRot.transformVectorInPlace(Queries._scratchCorner);
        const cx = Queries._scratchCorner.x, cy = Queries._scratchCorner.y, cz = Queries._scratchCorner.z;
        if (cx < out.min.x) out.min.x = cx; if (cx > out.max.x) out.max.x = cx;
        if (cy < out.min.y) out.min.y = cy; if (cy > out.max.y) out.max.y = cy;
        if (cz < out.min.z) out.min.z = cz; if (cz > out.max.z) out.max.z = cz;
    }
    out.min.x -= pad; out.min.y -= pad; out.min.z -= pad;
    out.max.x += pad; out.max.y += pad; out.max.z += pad;
    return out;
};

Queries._sweepShapeVsCompound = function (shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
    const compound = body.shape;
    let best = null, bestFraction = Infinity;

    let indices = null;
    if (compound.children.length > Midphase.SMALL_MESH_TRIS) {
        const bvh = ActionPhysics.ensureShapeBVH(compound);
        Queries._localSweptAABBInto(Queries._scratchLocalAABB, body, start, dirX, dirY, dirZ, Queries._sweptShapeRadius(shape));
        indices = Queries._scratchLeafList; indices.length = 0;
        bvh.query(Queries._scratchLocalAABB, function (i) { indices.push(i); });
    }
    const count = indices ? indices.length : compound.children.length;
    const childIndices = indices ? indices.slice() : null; // recursion reuses _scratchLeafList

    for (let k = 0; k < count; k++) {
        const child = compound.children[childIndices ? childIndices[k] : k];

        // Mesh / nested-compound child: not a convex primitive. Recurse at its world placement.
        if (Queries._isMesh(child.shape) || Queries._isCompound(child.shape)) {
            const sub = Queries._childAsBody(body, child);
            const hit = Queries._sweepShapeVsBody(shape, rotation, start, dirX, dirY, dirZ, fullLen, sub);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
            continue;
        }

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
    const a = Queries._scratchTriA, b = Queries._scratchTriB, c = Queries._scratchTriC;
    let best = null, bestFraction = Infinity;

    let indices = null;
    if (meshShape.triangleCount > Midphase.SMALL_MESH_TRIS) {
        const bvh = ActionPhysics.ensureShapeBVH(meshShape);
        Queries._localSweptAABBInto(Queries._scratchLocalAABB, body, start, dirX, dirY, dirZ, Queries._sweptShapeRadius(shape));
        indices = Queries._scratchLeafList; indices.length = 0;
        bvh.query(Queries._scratchLocalAABB, function (i) { indices.push(i); });
    }
    const count = indices ? indices.length : meshShape.triangleCount;

    for (let k = 0; k < count; k++) {
        meshShape.triangleAt(indices ? indices[k] : k, a, b, c);
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

// Bounding-sphere radius of `shape` about its local origin - the pad for a swept-shape AABB.
Queries._sweptShapeRadius = function (shape) {
    const lb = Queries._scratchExpandedAABB;
    shape.localAABBInto(lb);
    return Math.sqrt(
        Math.max(lb.min.x * lb.min.x, lb.max.x * lb.max.x) +
        Math.max(lb.min.y * lb.min.y, lb.max.y * lb.max.y) +
        Math.max(lb.min.z * lb.min.z, lb.max.z * lb.max.z)
    );
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
    placedBody.position = Queries._scratchPlacedBPos.copy(body.position);
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

// Local-space AABB of `shape` held at world `start` (its bounding sphere -> an axis box),
// inverse-transformed into `body`'s frame, for BVH pruning an overlap test.
Queries._localOverlapAABBInto = function (out, body, start, shape) {
    const r = Queries._sweptShapeRadius(shape);
    Queries._scratchCorner.set(start.x - body.position.x, start.y - body.position.y, start.z - body.position.z);
    Queries._scratchInvRot.copy(body.rotation).invert().transformVectorInPlace(Queries._scratchCorner);
    out.min.x = Queries._scratchCorner.x - r; out.max.x = Queries._scratchCorner.x + r;
    out.min.y = Queries._scratchCorner.y - r; out.max.y = Queries._scratchCorner.y + r;
    out.min.z = Queries._scratchCorner.z - r; out.max.z = Queries._scratchCorner.z + r;
    return out;
};

Queries._overlapTestCompound = function (shape, start, rotation, body) {
    const compound = body.shape;
    let indices = null;
    if (compound.children.length > Midphase.SMALL_MESH_TRIS) {
        const bvh = ActionPhysics.ensureShapeBVH(compound);
        Queries._localOverlapAABBInto(Queries._scratchLocalAABB, body, start, shape);
        indices = Queries._scratchLeafList; indices.length = 0;
        bvh.query(Queries._scratchLocalAABB, function (i) { indices.push(i); });
    }
    const children = compound.children;
    const count = indices ? indices.length : children.length;
    const childIndices = indices ? indices.slice() : null; // recursion reuses _scratchLeafList
    for (let k = 0; k < count; k++) {
        const child = children[childIndices ? childIndices[k] : k];

        // Mesh / nested-compound child: recurse at its world placement (GJK can't take a mesh).
        if (Queries._isMesh(child.shape) || Queries._isCompound(child.shape)) {
            const sub = Queries._childAsBody(body, child);
            const hit = Queries._isMesh(child.shape)
                ? Queries._overlapTestMesh(shape, start, rotation, sub)
                : Queries._overlapTestCompound(shape, start, rotation, sub);
            if (hit) return { point: hit.point, normal: hit.normal, distance: 0, fraction: 0, body: body };
            continue;
        }

        const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, child);
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
    const a = Queries._scratchTriA, b = Queries._scratchTriB, c = Queries._scratchTriC;
    let indices = null;
    if (meshShape.triangleCount > Midphase.SMALL_MESH_TRIS) {
        const bvh = ActionPhysics.ensureShapeBVH(meshShape);
        Queries._localOverlapAABBInto(Queries._scratchLocalAABB, body, start, shape);
        indices = Queries._scratchLeafList; indices.length = 0;
        bvh.query(Queries._scratchLocalAABB, function (i) { indices.push(i); });
    }
    const count = indices ? indices.length : meshShape.triangleCount;
    for (let k = 0; k < count; k++) {
        meshShape.triangleAt(indices ? indices[k] : k, a, b, c);
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
