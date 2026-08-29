// Expanding a broadphase body pair into primitive-vs-primitive candidates.
var proto = Midphase.prototype;

// Expands one side into primitive candidates. `otherAABB`: the other body's fattened AABB, for
// culling. `otherBodyId`: leaf-cache key. `isNestedChild`: `body` is a compound child's placement
// being recursed into (no speculative margin of its own, so the own-margin fattening is skipped).
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
            // Baked to world space at identity rotation, so the narrowphase has nothing to undo.
            const slot = this._nextTriSlot();
            body.rotation.transformVectorInto(a, slot.a); slot.a.addInPlace(body.position);
            body.rotation.transformVectorInto(b, slot.b); slot.b.addInPlace(body.position);
            body.rotation.transformVectorInto(c, slot.c); slot.c.addInPlace(body.position);
            slot.shape.a = slot.a; slot.shape.b = slot.b; slot.shape.c = slot.c;
            // position stays at origin (verts are already world-space); bodyCenter is a hint TriTri
            // uses to orient the contact normal.
            slot.bodyCenter.copy(body.position);
            out.push({ shape: slot.shape, position: slot.position, rotation: slot.rotation, bodyCenter: slot.bodyCenter });
        }
    }
    return out;
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
