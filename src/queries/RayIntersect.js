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

// rayIntersectAll(bodies, start, end, ignore) -> array of { body, point, normal, distance, fraction },
// EVERY body the segment crosses, sorted nearest-first (empty array = no hit). Same per-body test as
// rayIntersect; use this when the caller filters hits itself (e.g. skip-my-own-body-then-take-the-next).
Queries.rayIntersectAll = function (bodies, start, end, ignore) {
    const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
    const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    const out = [];
    if (fullLen < 1e-12) return out;

    for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        if (Queries._isIgnored(body, ignore)) continue;
        if (!Queries._rayIntersectsAABB(start, end, body.getAABB())) continue;

        const hit = Queries._sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body);
        if (hit) { hit.body = body; out.push(hit); }
    }
    out.sort(function (a, b) { return a.fraction - b.fraction; });
    return out;
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
    placedBody.position = Queries._scratchPlacedBPos.copy(body.position);
    placedBody.rotation = body.rotation;

    const support = Queries._scratchSupport;
    support.a = placedPoint; support.b = placedBody;
    support._invRotA.copy(Queries._identityQuat);
    support._invRotB.copy(body.rotation).invert();

    return Queries._advance(support, placedPoint, start, dirX, dirY, dirZ, fullLen);
};

// The AABB of the ray segment [start, start + dir*fullLen], transformed into `body`'s local space
// (8-corner inverse transform, conservative), written into `out`. Used to prune a mesh/compound
// BVH so a cast doesn't sweep every triangle/child.
Queries._localRayAABBInto = function (out, body, start, dirX, dirY, dirZ, fullLen) {
    const ex = start.x, ey = start.y, ez = start.z;
    const fx = start.x + dirX, fy = start.y + dirY, fz = start.z + dirZ; // dir is already scaled to fullLen by the callers
    const invRot = Queries._scratchInvRot.copy(body.rotation).invert();
    out.setEmpty();
    for (let k = 0; k < 2; k++) {
        const wx = k ? fx : ex, wy = k ? fy : ey, wz = k ? fz : ez;
        Queries._scratchCorner.set(wx - body.position.x, wy - body.position.y, wz - body.position.z);
        invRot.transformVectorInPlace(Queries._scratchCorner);
        const cx = Queries._scratchCorner.x, cy = Queries._scratchCorner.y, cz = Queries._scratchCorner.z;
        if (cx < out.min.x) out.min.x = cx; if (cx > out.max.x) out.max.x = cx;
        if (cy < out.min.y) out.min.y = cy; if (cy > out.max.y) out.max.y = cy;
        if (cz < out.min.z) out.min.z = cz; if (cz > out.max.z) out.max.z = cz;
    }
    return out;
};

Queries._sweepPointVsCompound = function (start, dirX, dirY, dirZ, fullLen, body) {
    const shape = body.shape;
    let best = null, bestFraction = Infinity;

    // BVH-prune: only test children whose local AABB the ray's local AABB overlaps.
    let indices = null;
    if (shape.children.length > Midphase.SMALL_MESH_TRIS) {
        const bvh = ActionPhysics.ensureShapeBVH(shape);
        Queries._localRayAABBInto(Queries._scratchLocalAABB, body, start, dirX, dirY, dirZ, fullLen);
        indices = Queries._scratchLeafList; indices.length = 0;
        bvh.query(Queries._scratchLocalAABB, function (i) { indices.push(i); });
    }
    const count = indices ? indices.length : shape.children.length;

    // Snapshot the child index list before the loop: a mesh/compound child recurses, and the
    // recursion reuses Queries._scratchLeafList.
    const childIndices = indices ? indices.slice() : null;
    for (let k = 0; k < count; k++) {
        const child = shape.children[childIndices ? childIndices[k] : k];

        // A child that is itself a mesh or a nested compound is NOT a convex primitive - GJK would
        // hit MeshShape.supportInto and throw. Recurse into it at its own world placement, exactly
        // as the midphase expands such a child.
        if (Queries._isMesh(child.shape) || Queries._isCompound(child.shape)) {
            const sub = Queries._childAsBody(body, child);
            const hit = Queries._sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, sub);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
            continue;
        }

        const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, child);

        const placedPoint = Queries._scratchPlacedA;
        placedPoint.shape = Queries._scratchPointShape;
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

// A one-off body object placing `child` (a CompoundShapeChild) at its world transform under
// `parentBody`, so a mesh/compound child can be run through the same per-body query path as a
// top-level body. Allocates - only hit when a query ray actually crosses a compound-of-meshes,
// which is rare (a static model collider, once per probe).
Queries._childAsBody = function (parentBody, child) {
    const pos = new Vector3();
    parentBody.rotation.transformVectorInto(child.localPosition, pos);
    pos.addInPlace(parentBody.position);
    const rot = new Quaternion();
    rot.multiplyQuaternions(parentBody.rotation, child.localRotation);
    return {
        shape: child.shape,
        position: pos,
        rotation: rot,
        getAABB: function () { return parentBody.getAABB(); } // conservative; only used for the mesh BVH-prune's own placement math, which reads position/rotation
    };
};

Queries._sweepPointVsMesh = function (start, dirX, dirY, dirZ, fullLen, body) {
    const shape = body.shape;
    const a = Queries._scratchTriA, b = Queries._scratchTriB, c = Queries._scratchTriC;
    let best = null, bestFraction = Infinity;

    // BVH-prune: only sweep triangles whose local AABB the ray's local AABB overlaps. A tiny mesh
    // scans all - a couple of triangles is cheaper than building/walking a tree.
    let indices = null;
    if (shape.triangleCount > Midphase.SMALL_MESH_TRIS) {
        const bvh = ActionPhysics.ensureShapeBVH(shape);
        Queries._localRayAABBInto(Queries._scratchLocalAABB, body, start, dirX, dirY, dirZ, fullLen);
        indices = Queries._scratchLeafList; indices.length = 0;
        bvh.query(Queries._scratchLocalAABB, function (i) { indices.push(i); });
    }
    const count = indices ? indices.length : shape.triangleCount;

    for (let k = 0; k < count; k++) {
        shape.triangleAt(indices ? indices[k] : k, a, b, c);
        const placedTri = Queries._placedTriangleInto(Queries._scratchCompoundChild, body, Queries._scratchTriangleShape, a, b, c);

        const placedPoint = Queries._scratchPlacedA;
        placedPoint.shape = Queries._scratchPointShape;
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
