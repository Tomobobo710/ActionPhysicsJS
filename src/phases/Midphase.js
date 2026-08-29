/**
 * Midphase: which children of a compound / triangles of a mesh actually need narrowphase?
 *
 * Produces candidate PRIMITIVE-shape pairs from a broadphase body pair, each carrying the
 * primitive shape plus its world transform - narrowphase takes these directly, with no
 * compound/mesh awareness of its own. Never computes contact data, only "these two might touch".
 *
 * A body's shape is one of three kinds: primitive (the body IS the one candidate, no BVH needed),
 * CompoundShape (candidates are children whose world AABB overlaps the other side), or MeshShape
 * (candidates are triangles, same rule). See BVHCache.js (per-shape BVH build + leaf query/cache)
 * and ExpandPair.js (turning a broadphase pair into primitive candidates).
 */
class Midphase {
    constructor() {
        // Static-geometry leaf cache: for a (shape, queryAABB) pair, the leaf indices that
        // overlapped, INCLUDING the empty set (not caching empty made every resting body re-walk
        // the BVH forever). Keyed by the OTHER body's id, since the query box moves with it.
        this._leafCache = new Map(); // otherBodyId -> { shape, minx,miny,minz,maxx,maxy,maxz, hits:[leafIndex...] }

        // Per-expandPair()-call pools for compound-child / mesh-triangle world placement, grown as
        // needed, never shrunk - see ExpandPair.js's _nextTriSlot / _nextChildSlot.
        this._triSlots = [];
        this._triSlotIndex = 0;
        this._childSlots = [];
        this._childSlotIndex = 0;
    }

    // Clears every cached leaf-walk result. Call when a static/kinematic compound or mesh body's
    // geometry/transform changes - the cache (keyed by the OTHER body) has no way to know on its own.
    invalidate() {
        this._leafCache.clear();
    }
}

ActionPhysics.Midphase = Midphase;
