var proto = Midphase.prototype;

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

    if (shape instanceof MeshShape && shape.triangleCount <= Midphase.SMALL_MESH_TRIS) {
        const bounds = Midphase._scratchSmallAABB;
        shape.localAABBInto(bounds);
        if (!Midphase._worldOverlaps(bounds, body, otherAABB)) return out;
        this._emitTriangles(shape, body, out, null);
        return out;
    }

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

        const nestedBody = this._nestedBodyAt(depth);
        for (let k = 0; k < hits.length; k++) {
            const child = shape.children[hits[k]];
            const slot = this._nextChildSlot();
            body.rotation.transformVectorInto(child.localPosition, slot.position);
            slot.position.addInPlace(body.position);
            slot.rotation.multiplyQuaternions(body.rotation, child.localRotation);
            if (child.shape instanceof MeshShape || child.shape instanceof CompoundShape) {

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

proto._emitTriangles = function (shape, body, out, hits) {
    const a = Midphase._scratchTriA, b = Midphase._scratchTriB, c = Midphase._scratchTriC;
    const n = hits ? hits.length : shape.triangleCount;
    for (let k = 0; k < n; k++) {
        shape.triangleAt(hits ? hits[k] : k, a, b, c);

        const slot = this._nextTriSlot();
        body.rotation.transformVectorInto(a, slot.a); slot.a.addInPlace(body.position);
        body.rotation.transformVectorInto(b, slot.b); slot.b.addInPlace(body.position);
        body.rotation.transformVectorInto(c, slot.c); slot.c.addInPlace(body.position);
        slot.shape.a = slot.a; slot.shape.b = slot.b; slot.shape.c = slot.c;

        slot.bodyCenter.copy(body.position);

        out.push(slot);
    }
};

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

proto._nextChildSlot = function () {
    if (this._childSlotIndex >= this._childSlots.length) {
        this._childSlots.push({ position: new Vector3(), rotation: new Quaternion() });
    }
    return this._childSlots[this._childSlotIndex++];
};

proto._nextPrimSlot = function () {
    if (this._primSlotIndex >= this._primSlots.length) {
        this._primSlots.push({ shape: null, position: null, rotation: null, bodyCenter: null });
    }
    return this._primSlots[this._primSlotIndex++];
};

proto._nestedBodyAt = function (depth) {
    while (this._nestedBodies.length <= depth) {
        this._nestedBodies.push({ shape: null, position: null, rotation: null });
    }
    return this._nestedBodies[depth];
};

proto.expandPairSides = function (bodyA, bodyB) {

    this._triSlotIndex = 0;
    this._childSlotIndex = 0;
    this._primSlotIndex = 0;
    const sides = this._sides;
    sides.a.length = 0;
    sides.b.length = 0;

    this._expandSide(bodyA, bodyB.getBroadphaseAABB(), bodyB.id, false, sides.a, 0);
    this._expandSide(bodyB, bodyA.getBroadphaseAABB(), bodyA.id, false, sides.b, 0);
    return sides;
};

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

Midphase._scratchTriA = new Vector3();
Midphase._scratchTriB = new Vector3();
Midphase._scratchTriC = new Vector3();
