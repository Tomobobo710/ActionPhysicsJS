// Expanding a broadphase body pair into primitive-vs-primitive candidates.
var proto = Midphase.prototype;

// Expands one side into primitive candidates: [{ shape, position, rotation }]. `otherAABB` is the
// other body's world (fattened) AABB, used to cull children/triangles that can't matter.
// `otherBodyId` keys the leaf cache.
proto._expandSide = function (body, otherAABB, otherBodyId) {
    const shape = body.shape;
    if (!(shape instanceof CompoundShape) && !(shape instanceof MeshShape)) {
        return [{ shape: shape, position: body.position, rotation: body.rotation, child: null }];
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
    const tightAABB = body.getAABB(), bpAABB = body.getBroadphaseAABB();
    const marginX = Math.max(bpAABB.max.x - tightAABB.max.x, tightAABB.min.x - bpAABB.min.x);
    const marginY = Math.max(bpAABB.max.y - tightAABB.max.y, tightAABB.min.y - bpAABB.min.y);
    const marginZ = Math.max(bpAABB.max.z - tightAABB.max.z, tightAABB.min.z - bpAABB.min.z);
    const ownMargin = Math.max(marginX, marginY, marginZ, 0);
    localQuery.min.x -= ownMargin; localQuery.min.y -= ownMargin; localQuery.min.z -= ownMargin;
    localQuery.max.x += ownMargin; localQuery.max.y += ownMargin; localQuery.max.z += ownMargin;

    const hits = this._queryLeaves(shape, otherBodyId, localQuery);
    const out = [];
    if (shape instanceof CompoundShape) {
        for (let k = 0; k < hits.length; k++) {
            const child = shape.children[hits[k]];
            const worldPos = new Vector3();
            body.rotation.transformVectorInto(child.localPosition, worldPos);
            worldPos.addInPlace(body.position);
            const worldRot = new Quaternion().multiplyQuaternions(body.rotation, child.localRotation);
            out.push({ shape: child.shape, position: worldPos, rotation: worldRot, child: child });
        }
    } else {
        const a = new Vector3(), b = new Vector3(), c = new Vector3();
        for (let k = 0; k < hits.length; k++) {
            shape.triangleAt(hits[k], a, b, c);
            // Baked into world space directly, at identity rotation - simpler than a per-triangle
            // local frame narrowphase would have to undo.
            const wa = new Vector3(), wb = new Vector3(), wc = new Vector3();
            body.rotation.transformVectorInto(a, wa); wa.addInPlace(body.position);
            body.rotation.transformVectorInto(b, wb); wb.addInPlace(body.position);
            body.rotation.transformVectorInto(c, wc); wc.addInPlace(body.position);
            out.push({ shape: new TriangleShape(wa, wb, wc), position: new Vector3(0, 0, 0), rotation: new Quaternion(), child: null });
        }
    }
    return out;
};

// Expands a broadphase [bodyA, bodyB] pair into primitive x primitive candidates:
// [{ a: {shape,position,rotation}, b: {shape,position,rotation} }, ...]
proto.expandPair = function (bodyA, bodyB) {
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
