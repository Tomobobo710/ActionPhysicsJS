// Expanding a broadphase body pair into primitive-vs-primitive candidates.
var proto = Midphase.prototype;

// Expands one side into primitive candidates. `otherAABB`: the other body's fattened AABB, for
// culling. `otherBodyId`: leaf-cache key. `isNestedChild`: `body` is a compound child's placement
// being recursed into (no speculative margin of its own, so the own-margin fattening is skipped).
// `out`: caller-owned array, reset by the caller; results (including nested recursion) append to it.
proto._expandSide = function (body, otherAABB, otherBodyId, isNestedChild, out, depth) {
    depth = depth || 0;
    const shape = body.shape;
    if (!(shape instanceof CompoundShape) && !(shape instanceof MeshShape)) {
        const slot = this._nextPrimSlot();
        slot.shape = shape;
        slot.position = body.position;
        slot.rotation = body.rotation;
        slot.bodyCenter = null;
        out.push(slot);
        return out;
    }

    // A tiny mesh (one tile of a big CompoundShape ground) doesn't earn a BVH walk: per-triangle
    // culling saves at most a test or two, while the local query AABB plus the tree walk costs more.
    // Still cull the mesh as a whole - a distant one must yield no candidates - but do it with a
    // direct world-space AABB overlap against the shape's own bounds.
    if (shape instanceof MeshShape && shape.triangleCount <= Midphase.SMALL_MESH_TRIS) {
        const bounds = Midphase._scratchSmallAABB;
        shape.localAABBInto(bounds);
        if (!Midphase._worldOverlaps(bounds, body, otherAABB)) return out;
        this._emitTriangles(shape, body, out, null);
        return out;
    }

    // Bring the other body's world AABB into this body's local space by inverse-transforming its 8
    // corners - conservative (may over-include), never under-includes.
    const invRot = Midphase._scratchQuat.copy(body.rotation).invert();
    const localQuery = Midphase._scratchAABB.setEmpty();
    const corner = Midphase._scratchVec;
    for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
        corner.x = cx ? otherAABB.max.x : otherAABB.min.x;
        corner.y = cy ? otherAABB.max.y : otherAABB.min.y;
        corner.z = cz ? otherAABB.max.z : otherAABB.min.z;
        corner.subInPlace(body.position);
        invRot.transformVectorInPlace(corner);
        if (corner.x < localQuery.min.x) localQuery.min.x = corner.x;
        if (corner.y < localQuery.min.y) localQuery.min.y = corner.y;
        if (corner.z < localQuery.min.z) localQuery.min.z = corner.z;
        if (corner.x > localQuery.max.x) localQuery.max.x = corner.x;
        if (corner.y > localQuery.max.y) localQuery.max.y = corner.y;
        if (corner.z > localQuery.max.z) localQuery.max.z = corner.z;
    }

    // otherAABB has the other body's margin but not this one's - fatten by this body's own
    // broadphase-vs-tight delta (largest per-axis, so a rotated body isn't under-estimated).
    if (!isNestedChild) {
        const tightAABB = body.getAABB(), bpAABB = body.getBroadphaseAABB();
        const marginX = Math.max(bpAABB.max.x - tightAABB.max.x, tightAABB.min.x - bpAABB.min.x);
        const marginY = Math.max(bpAABB.max.y - tightAABB.max.y, tightAABB.min.y - bpAABB.min.y);
        const marginZ = Math.max(bpAABB.max.z - tightAABB.max.z, tightAABB.min.z - bpAABB.min.z);
        const ownMargin = Math.max(marginX, marginY, marginZ, 0);
        localQuery.min.x -= ownMargin; localQuery.min.y -= ownMargin; localQuery.min.z -= ownMargin;
        localQuery.max.x += ownMargin; localQuery.max.y += ownMargin; localQuery.max.z += ownMargin;
    }

    const hits = this._queryLeaves(shape, otherBodyId, localQuery);
    if (shape instanceof CompoundShape) {
        // Per-depth scratch: this frame keeps reading `body` across iterations, so the callee
        // can't be handed the object we're reading from.
        const nestedBody = this._nestedBodyAt(depth);
        for (let k = 0; k < hits.length; k++) {
            const child = shape.children[hits[k]];
            const slot = this._nextChildSlot();
            body.rotation.transformVectorInto(child.localPosition, slot.position);
            slot.position.addInPlace(body.position);
            slot.rotation.multiplyQuaternions(body.rotation, child.localRotation);
            if (child.shape instanceof MeshShape || child.shape instanceof CompoundShape) {
                // A compound child that is itself a mesh/compound (e.g. a CompoundShape ground
                // made of many small MeshShape tiles) is not itself a primitive - recurse into it
                // at its own world placement, same as expanding a top-level body's shape.
                nestedBody.shape = child.shape;
                nestedBody.position = slot.position;
                nestedBody.rotation = slot.rotation;
                this._expandSide(nestedBody, otherAABB, otherBodyId, true, out, depth + 1);
            } else {
                const prim = this._nextPrimSlot();
                prim.shape = child.shape;
                prim.position = slot.position;
                prim.rotation = slot.rotation;
                prim.bodyCenter = null;
                out.push(prim);
            }
        }
    } else {
        this._emitTriangles(shape, body, out, hits);
    }
    return out;
};

// Appends `shape`'s triangles, baked to world space, into `out`. `hits`: leaf indices to emit, or
// null for all of them.
proto._emitTriangles = function (shape, body, out, hits) {
    const a = Midphase._scratchTriA, b = Midphase._scratchTriB, c = Midphase._scratchTriC;
    const n = hits ? hits.length : shape.triangleCount;
    for (let k = 0; k < n; k++) {
        shape.triangleAt(hits ? hits[k] : k, a, b, c);
        // Baked to world space at identity rotation, so the narrowphase has nothing to undo.
        const slot = this._nextTriSlot();
        body.rotation.transformVectorInto(a, slot.a); slot.a.addInPlace(body.position);
        body.rotation.transformVectorInto(b, slot.b); slot.b.addInPlace(body.position);
        body.rotation.transformVectorInto(c, slot.c); slot.c.addInPlace(body.position);
        slot.shape.a = slot.a; slot.shape.b = slot.b; slot.shape.c = slot.c;
        // position stays at origin (verts are already world-space); bodyCenter is a hint TriTri
        // uses to orient the contact normal.
        slot.bodyCenter.copy(body.position);
        // The slot is already the shape the narrowphase reads - hand it over directly.
        out.push(slot);
    }
};

// Pooled world-space triangle slot, grown as needed. The pool index resets once per expandPair()
// (both sides draw from it).
proto._nextTriSlot = function () {
    if (this._triSlotIndex >= this._triSlots.length) {
        this._triSlots.push({
            a: new Vector3(), b: new Vector3(), c: new Vector3(),
            position: new Vector3(0, 0, 0), rotation: new Quaternion(),
            bodyCenter: new Vector3(), shape: null
        });
        const slot = this._triSlots[this._triSlots.length - 1];
        slot.shape = new TriangleShape(slot.a, slot.b, slot.c);
    }
    return this._triSlots[this._triSlotIndex++];
};

// One pooled { position, rotation } slot for a compound child's world placement, same pooling
// pattern as _nextTriSlot (the child's own shape is reused directly - CompoundShape.children never
// changes at runtime, so child.shape itself needs no pooling).
proto._nextChildSlot = function () {
    if (this._childSlotIndex >= this._childSlots.length) {
        this._childSlots.push({ position: new Vector3(), rotation: new Quaternion() });
    }
    return this._childSlots[this._childSlotIndex++];
};

// Pooled placement for a non-triangle primitive. Holds references to vectors owned elsewhere.
proto._nextPrimSlot = function () {
    if (this._primSlotIndex >= this._primSlots.length) {
        this._primSlots.push({ shape: null, position: null, rotation: null, bodyCenter: null });
    }
    return this._primSlots[this._primSlotIndex++];
};

// Scratch placement for recursing into a nested compound child, one per depth.
proto._nestedBodyAt = function (depth) {
    while (this._nestedBodies.length <= depth) {
        this._nestedBodies.push({ shape: null, position: null, rotation: null });
    }
    return this._nestedBodies[depth];
};

// Expands a broadphase pair into the two sides' primitive lists; the candidate set is their
// cross-product, which callers walk directly instead of materialising it.
//
// Returns a reused { a, b } - arrays and placements are pooled and valid only until the next call.
proto.expandPairSides = function (bodyA, bodyB) {
    // Both sides draw from the same pools, so they reset once here, not per side.
    this._triSlotIndex = 0;
    this._childSlotIndex = 0;
    this._primSlotIndex = 0;
    const sides = this._sides;
    sides.a.length = 0;
    sides.b.length = 0;
    // The fattened broadphase AABB is used for child/triangle culling too, so a compound child or
    // mesh triangle a body is about to reach surfaces the same tick early as the body pair itself.
    this._expandSide(bodyA, bodyB.getBroadphaseAABB(), bodyB.id, false, sides.a, 0);
    this._expandSide(bodyB, bodyA.getBroadphaseAABB(), bodyA.id, false, sides.b, 0);
    return sides;
};

// Materialised cross-product form of expandPairSides, for external callers and tests.
proto.expandPair = function (bodyA, bodyB) {
    const sides = this.expandPairSides(bodyA, bodyB);
    const out = [];
    for (let i = 0; i < sides.a.length; i++) {
        for (let j = 0; j < sides.b.length; j++) {
            out.push({ a: sides.a[i], b: sides.b[j] });
        }
    }
    return out;
};

// Does `local` (a shape-local AABB placed at `body`) overlap the world-space `otherAABB`? The
// local box is rotated into world space by its 8 corners - conservative, never under-includes.
Midphase._worldOverlaps = function (local, body, otherAABB) {
    const rot = body.rotation, pos = body.position;
    const corner = Midphase._scratchVec;
    let minx = Infinity, miny = Infinity, minz = Infinity;
    let maxx = -Infinity, maxy = -Infinity, maxz = -Infinity;
    for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
        corner.set(cx ? local.max.x : local.min.x, cy ? local.max.y : local.min.y, cz ? local.max.z : local.min.z);
        rot.transformVectorInPlace(corner);
        corner.addInPlace(pos);
        if (corner.x < minx) minx = corner.x;
        if (corner.y < miny) miny = corner.y;
        if (corner.z < minz) minz = corner.z;
        if (corner.x > maxx) maxx = corner.x;
        if (corner.y > maxy) maxy = corner.y;
        if (corner.z > maxz) maxz = corner.z;
    }
    return maxx >= otherAABB.min.x && minx <= otherAABB.max.x &&
        maxy >= otherAABB.min.y && miny <= otherAABB.max.y &&
        maxz >= otherAABB.min.z && minz <= otherAABB.max.z;
};

Midphase._scratchSmallAABB = new AABB();
Midphase._scratchQuat = new Quaternion();
Midphase._scratchAABB = new AABB();
Midphase._scratchVec = new Vector3();
// Local-space triangle vertices, read fresh from shape.triangleAt() each hit, then transformed
// into that hit's own pooled world-space slot - see _nextTriSlot.
Midphase._scratchTriA = new Vector3();
Midphase._scratchTriB = new Vector3();
Midphase._scratchTriC = new Vector3();
