// Expands a broadphase body pair into candidate primitive-shape pairs (compound children / mesh
// triangles whose world AABB overlaps the other side). See BVHCache.js and ExpandPair.js.
class Midphase {
    constructor() {
        // otherBodyId -> { shape, min/max bounds, hits:[leafIndex] }. Empty results are cached too
        // (otherwise a resting body re-walks the BVH every tick).
        this._leafCache = new Map();

        // Per-expandPair() world-placement pools, grown as needed. See ExpandPair.js.
        this._triSlots = [];
        this._triSlotIndex = 0;
        this._childSlots = [];
        this._childSlotIndex = 0;
    }

    // Call when a static/kinematic compound/mesh body's geometry or transform changes.
    invalidate() {
        this._leafCache.clear();
    }
}

ActionPhysics.Midphase = Midphase;
