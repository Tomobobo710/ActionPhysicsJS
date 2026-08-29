// Expanding a broadphase body pair into primitive-vs-primitive candidates.
var proto = Midphase.prototype;

// Expands one side into primitive candidates: [{ shape, position, rotation }]. `otherAABB` is the
// other body's world (fattened) AABB, used to cull children/triangles that can't matter.
// `otherBodyId` keys the leaf cache. `isNestedChild`: true when `body` is actually a compound
// child's world placement (a plain {shape,position,rotation}, not a real RigidBody) being expanded
// recursively because it is itself a mesh/compound - such a child has no speculative margin of its
// own (that's a whole-body concept), so the own-margin fattening below is skipped for it.
proto._expandSide = function (body, otherAABB, otherBodyId, isNestedChild) {
    const shape = body.shape;
    if (!(shape instanceof CompoundShape) && !(shape instanceof MeshShape)) {
        return [{ shape: shape, position: body.position, rotation: body.rotation }];
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

    // otherAABB carries the OTHER body's speculative margin, but THIS body's own margin never
    // entered the query above - fatten symmetrically by this body's own broadphase-vs-tight AABB
    // delta (taking the largest per-axis delta so a rotated body's local-frame margin isn't
    // under-estimated), or a compound/mesh body approaching under its own margin can get zero
    // candidates for a tick or two while already overlapping, then correct in one deep jolt.
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
    const out = [];
    if (shape instanceof CompoundShape) {
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
                const nested = this._expandSide({ shape: child.shape, position: slot.position, rotation: slot.rotation }, otherAABB, otherBodyId, true);
                for (let n = 0; n < nested.length; n++) out.push(nested[n]);
            } else {
                out.push({ shape: child.shape, position: slot.position, rotation: slot.rotation });
            }
        }
    } else {
        const a = Midphase._scratchTriA, b = Midphase._scratchTriB, c = Midphase._scratchTriC;
        for (let k = 0; k < hits.length; k++) {
            shape.triangleAt(hits[k], a, b, c);
            // Baked into world space directly, at identity rotation - simpler than a per-triangle
            // local frame narrowphase would have to undo.
            const slot = this._nextTriSlot();
            body.rotation.transformVectorInto(a, slot.a); slot.a.addInPlace(body.position);
            body.rotation.transformVectorInto(b, slot.b); slot.b.addInPlace(body.position);
            body.rotation.transformVectorInto(c, slot.c); slot.c.addInPlace(body.position);
            slot.shape.a = slot.a; slot.shape.b = slot.b; slot.shape.c = slot.c;
            // slot.position stays at origin (the triangle's verts are already world-space and the
            // narrowphase support adds slot.position). bodyCenter is a separate hint - the owning
            // body's world center - which TriTri.js uses to orient a mesh face-contact normal
            // reliably even when the two faces are exactly flush.
            slot.bodyCenter.copy(body.position);
            out.push({ shape: slot.shape, position: slot.position, rotation: slot.rotation, bodyCenter: slot.bodyCenter });
        }
    }
    return out;
};

// One pooled { shape: TriangleShape, a/b/c: Vector3 (the shape's own world-space vertices),
// position: Vector3(0,0,0), rotation: identity Quaternion } slot, grown as needed, never shrunk -
// same pattern as NarrowPhase._contactPool. Indexed by _slotIndex, reset once per expandPair() call
// (not per _expandSide) since both sides of one pair draw from the same tick's pool.
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

// Expands a broadphase [bodyA, bodyB] pair into primitive x primitive candidates:
// [{ a: {shape,position,rotation}, b: {shape,position,rotation} }, ...]
proto.expandPair = function (bodyA, bodyB) {
    // Both sides' compound/mesh expansion below draws from the same pooled slots, so the pool is
    // reset once here, not per side - each hit this tick still gets its own slot, consumed
    // synchronously by the caller before the next pair (see PairTest.js) or the next tick.
    this._triSlotIndex = 0;
    this._childSlotIndex = 0;
    // The fattened broadphase AABB is used for child/triangle culling too, so a compound child or
    // mesh triangle a body is about to reach surfaces the same tick early as the body pair itself.
    const sidesA = this._expandSide(bodyA, bodyB.getBroadphaseAABB(), bodyB.id);
    const sidesB = this._expandSide(bodyB, bodyA.getBroadphaseAABB(), bodyA.id);
    const out = [];
    for (let i = 0; i < sidesA.length; i++) {
        for (let j = 0; j < sidesB.length; j++) {
            out.push({ a: sidesA[i], b: sidesB[j] });
        }
    }
    return out;
};

Midphase._scratchQuat = new Quaternion();
Midphase._scratchAABB = new AABB();
Midphase._scratchVec = new Vector3();
// Local-space triangle vertices, read fresh from shape.triangleAt() each hit, then transformed
// into that hit's own pooled world-space slot - see _nextTriSlot.
Midphase._scratchTriA = new Vector3();
Midphase._scratchTriB = new Vector3();
Midphase._scratchTriC = new Vector3();
