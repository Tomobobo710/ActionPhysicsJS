// Per-shape BVH construction (cached on the shape instance, built once) and cached leaf queries.
var proto = Midphase.prototype;

// Ensures shape._midphaseBVH exists, building on first use. Compound: one leaf per child. Mesh:
// one leaf per triangle.
proto._ensureBVH = function (shape) {
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
};

// Leaf indices of `shape` whose local-space AABB overlaps `localQueryAABB` (already in the
// shape's own local space). Cached per (shape, other body) - an identical query next tick for a
// body that hasn't moved hits the cache; a different box invalidates and re-walks.
proto._queryLeaves = function (shape, otherBodyId, localQueryAABB) {
    const cached = this._leafCache.get(otherBodyId);
    if (cached && cached.shape === shape &&
        cached.minx === localQueryAABB.min.x && cached.miny === localQueryAABB.min.y && cached.minz === localQueryAABB.min.z &&
        cached.maxx === localQueryAABB.max.x && cached.maxy === localQueryAABB.max.y && cached.maxz === localQueryAABB.max.z) {
        return cached.hits; // may be [] - a valid, cached answer
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
};
