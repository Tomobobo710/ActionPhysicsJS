/**
 * Midphase: which children of a compound / triangles of a mesh actually need narrowphase?
 *
 * Produces: candidate PRIMITIVE-shape pairs from a broadphase body pair, each candidate carrying
 * the primitive shape plus its WORLD transform (position, rotation) - narrowphase (once it exists)
 * takes these directly, with no compound/mesh awareness of its own. May assume the broadphase pair
 * is real (Rule 1). Must never compute contact data - only "these two primitives might touch".
 *
 * A body's shape is one of three kinds here:
 *   - primitive (Box, Sphere, Convex, ...): the body itself IS the one candidate, no BVH needed.
 *   - CompoundShape: candidates are its children whose world AABB overlaps the other side.
 *   - MeshShape: candidates are its triangles (wrapped as TriangleShape) whose world AABB overlaps.
 *
 * Each CompoundShape/MeshShape gets its own BVH over its children/triangles, built ONCE and cached
 * on the shape instance itself (`shape._midphaseBVH`) - static geometry, built once per plan.md's
 * Spatial section. A shape never rebuilds this even if queried by many different bodies (a mesh
 * asset shared across several static bodies builds its BVH exactly once).
 */
class Midphase {
    constructor() {
        // Static-geometry leaf cache: for a given (shape, queryAABB) the set of leaf indices that
        // overlapped, INCLUDING the empty set. Not caching an empty result was the bug that made
        // every resting body re-walk the BVH and re-run GJK every frame forever (plan.md, Bug
        // reference / Caching) - 80% of all cache checks in that measurement. Keyed by the OTHER
        // body's id, since the query box moves every tick with that body; invalidated once per
        // tick for a moving other-body, kept for a sleeping one (the sleep manager owns eviction
        // once it exists - this cache only ever stores what it's given, never guesses staleness).
        this._leafCache = new Map(); // otherBodyId -> { shape, minx,miny,minz,maxx,maxy,maxz, hits:[leafIndex...] }
    }

    // Clears every cached leaf-walk result. Call this when a static/kinematic compound or mesh
    // body's geometry or transform changes - the cache has no way to know that on its own (it is
    // keyed by the OTHER body, not the static one), matching the "sleeping bodies still get woken
    // explicitly" discipline in plan.md's Sleep section rather than guessing at staleness here.
    invalidate() {
        this._leafCache.clear();
    }

    // Ensures shape._midphaseBVH exists, building it on first use. Compound: one leaf per child.
    // Mesh: one leaf per triangle.
    _ensureBVH(shape) {
        if (shape._midphaseBVH) return shape._midphaseBVH;
        const bvh = new BVH();
        if (shape instanceof CompoundShape) {
            const scratch = new AABB();
            const rotMat = new Matrix3();
            const corner = new Vector3();
            bvh.build(shape.children.length, function (out, i) {
                const child = shape.children[i];
                child.shape.localAABBInto(scratch);
                rotMat.fromQuaternion(child.localRotation);
                out.setEmpty();
                for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
                    corner.x = cx ? scratch.max.x : scratch.min.x;
                    corner.y = cy ? scratch.max.y : scratch.min.y;
                    corner.z = cz ? scratch.max.z : scratch.min.z;
                    rotMat.transformVector3(corner);
                    corner.addInPlace(child.localPosition);
                    if (corner.x < out.min.x) out.min.x = corner.x;
                    if (corner.y < out.min.y) out.min.y = corner.y;
                    if (corner.z < out.min.z) out.min.z = corner.z;
                    if (corner.x > out.max.x) out.max.x = corner.x;
                    if (corner.y > out.max.y) out.max.y = corner.y;
                    if (corner.z > out.max.z) out.max.z = corner.z;
                }
            });
        } else if (shape instanceof MeshShape) {
            const a = new Vector3(), b = new Vector3(), c = new Vector3();
            bvh.build(shape.triangleCount, function (out, i) {
                shape.triangleAt(i, a, b, c);
                out.setEmpty();
                out.min.x = Math.min(a.x, b.x, c.x); out.max.x = Math.max(a.x, b.x, c.x);
                out.min.y = Math.min(a.y, b.y, c.y); out.max.y = Math.max(a.y, b.y, c.y);
                out.min.z = Math.min(a.z, b.z, c.z); out.max.z = Math.max(a.z, b.z, c.z);
            });
        }
        shape._midphaseBVH = bvh;
        return bvh;
    }

    // Leaf indices of `shape` (a Compound or Mesh) whose LOCAL-space AABB overlaps `localQueryAABB`
    // (already expressed in the shape's own local space by the caller). Cached per (shape, other
    // body) — an identical query next tick for a body that hasn't moved returns the same box and
    // hits the cache; a different box invalidates and re-walks.
    _queryLeaves(shape, otherBodyId, localQueryAABB) {
        const cached = this._leafCache.get(otherBodyId);
        if (cached && cached.shape === shape &&
            cached.minx === localQueryAABB.min.x && cached.miny === localQueryAABB.min.y && cached.minz === localQueryAABB.min.z &&
            cached.maxx === localQueryAABB.max.x && cached.maxy === localQueryAABB.max.y && cached.maxz === localQueryAABB.max.z) {
            return cached.hits; // may be [] - an empty result is a valid, cached answer
        }
        const bvh = this._ensureBVH(shape);
        const hits = [];
        bvh.query(localQueryAABB, function (i) { hits.push(i); });
        this._leafCache.set(otherBodyId, {
            shape: shape,
            minx: localQueryAABB.min.x, miny: localQueryAABB.min.y, minz: localQueryAABB.min.z,
            maxx: localQueryAABB.max.x, maxy: localQueryAABB.max.y, maxz: localQueryAABB.max.z,
            hits: hits
        });
        return hits;
    }

    // Expands one side of a broadphase pair into primitive candidates: [{ shape, position,
    // rotation }]. `otherAABB` is the other body's WORLD AABB, used to cull compound children /
    // mesh triangles that can't possibly matter. `otherBodyId` keys the leaf cache (see above).
    _expandSide(body, otherAABB, otherBodyId) {
        const shape = body.shape;
        if (!(shape instanceof CompoundShape) && !(shape instanceof MeshShape)) {
            return [{ shape: shape, position: body.position, rotation: body.rotation }];
        }

        // Bring the other body's world AABB into this body's LOCAL space by inverse-transforming
        // its 8 corners - conservative (may over-include), never under-includes, matching
        // broadphase's own no-false-negatives contract one level down.
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

        const hits = this._queryLeaves(shape, otherBodyId, localQuery);
        const out = [];
        if (shape instanceof CompoundShape) {
            for (let k = 0; k < hits.length; k++) {
                const child = shape.children[hits[k]];
                // World transform = body transform composed with the child's local offset.
                const worldPos = new Vector3();
                body.rotation.transformVectorInto(child.localPosition, worldPos);
                worldPos.addInPlace(body.position);
                const worldRot = new Quaternion().multiplyQuaternions(body.rotation, child.localRotation);
                out.push({ shape: child.shape, position: worldPos, rotation: worldRot });
            }
        } else {
            const a = new Vector3(), b = new Vector3(), c = new Vector3();
            for (let k = 0; k < hits.length; k++) {
                shape.triangleAt(hits[k], a, b, c);
                // Triangle vertices are baked into world space directly, at body identity rotation -
                // simpler than carrying a per-triangle local frame narrowphase would have to undo.
                const wa = new Vector3(), wb = new Vector3(), wc = new Vector3();
                body.rotation.transformVectorInto(a, wa); wa.addInPlace(body.position);
                body.rotation.transformVectorInto(b, wb); wb.addInPlace(body.position);
                body.rotation.transformVectorInto(c, wc); wc.addInPlace(body.position);
                out.push({ shape: new TriangleShape(wa, wb, wc), position: new Vector3(0, 0, 0), rotation: new Quaternion() });
            }
        }
        return out;
    }

    // Expands a broadphase [bodyA, bodyB] pair into primitive x primitive candidates:
    // [{ a: {shape,position,rotation}, b: {shape,position,rotation} }, ...]
    // Never computes contact data (Rule 1) - purely a cross-product of the two expansions.
    expandPair(bodyA, bodyB) {
        const sidesA = this._expandSide(bodyA, bodyB.getAABB(), bodyB.id);
        const sidesB = this._expandSide(bodyB, bodyA.getAABB(), bodyA.id);
        const out = [];
        for (let i = 0; i < sidesA.length; i++) {
            for (let j = 0; j < sidesB.length; j++) {
                out.push({ a: sidesA[i], b: sidesB[j] });
            }
        }
        return out;
    }
}

Midphase._scratchQuat = new Quaternion();
Midphase._scratchAABB = new AABB();
Midphase._scratchVec = new Vector3();

ActionPhysics.Midphase = Midphase;
