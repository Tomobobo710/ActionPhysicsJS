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
        this._primSlots = [];
        this._primSlotIndex = 0;
        // Scratch placements for nested compound recursion, indexed by depth.
        this._nestedBodies = [];
        // Reused return value of expandPairSides(); arrays are truncated, never replaced.
        this._sides = { a: [], b: [] };
    }

    // Call when a static/kinematic compound/mesh body's geometry or transform changes.
    invalidate() {
        this._leafCache.clear();
    }
}

// At or below this triangle count, a mesh is expanded wholesale instead of BVH-queried - see
// _expandSide. Tiled CompoundShape ground is the motivating case (2 triangles per tile).
Midphase.SMALL_MESH_TRIS = 4;

ActionPhysics.Midphase = Midphase;
