/**
 * ContactManifold: persistent contact state for one pair of primitive shapes, across ticks.
 *
 * Owns point lifetime entirely. Narrowphase (via update(), called once per TICK, never per
 * substep) only ever adds or refreshes points from that tick's GJK/EPA result; only the manifold
 * itself removes a point, and only from update() - never mid-substep, which previously retired
 * points still mid-correction (not actually separated), emptying manifolds and dropping bodies.
 *
 * PERSISTENCE / WARM-START: up to MAX_POINTS points. Each update() matches this tick's result
 * against existing points by proximity in bodyA-local space (a contact feature's position relative
 * to A's own frame stays close between ticks even as A moves). A match refreshes geometry on the
 * EXISTING point object, preserving its accumulated lambda for the solver's warm start.
 *
 * See Update.js (the per-tick match/add/remove) and Reduction.js (4-point cap reduction).
 */
class ContactManifold {
    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.points = []; // ContactDetails[], length 0..MAX_POINTS
        // Local-space (bodyA-relative, at match time) anchor per point, parallel to `points` -
        // used only for next-tick matching, recomputed every update().
        this._localAnchors = [];
    }

    get pointCount() { return this.points.length; }
}

ContactManifold.MAX_POINTS = 4;
// Base match distance (floor for a resting/slow contact) - see Update.js's _matchDistance, which
// widens this by the contact point's own tangential travel per tick, the same shape
// SpeculativeMargin.js already uses for the broadphase/narrowphase gap.
ContactManifold.MATCH_DISTANCE = 0.05;
// Signed-distance half-width of the exact-touch band where GJK/EPA's normal is ambiguous and a
// warm-matched point keeps its established normal instead (see Update.js).
ContactManifold.EXACT_TOUCH_BAND = 0.001;

ContactManifold._scratchNormal = new Vector3();
ContactManifold._scratchRA = new Vector3();
ContactManifold._scratchRB = new Vector3();

ActionPhysics.ContactManifold = ContactManifold;
