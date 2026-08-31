// Persistent contact state for one primitive-shape pair. update() (once per tick, never per
// substep) matches this tick's GJK/EPA result against existing points by bodyA-local proximity,
// refreshing matched points in place to preserve their warm-start lambda. Only update() removes a
// point. See Update.js and Reduction.js.
class ContactManifold {
    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.points = []; // ContactDetails[], 0..MAX_POINTS
        this._localAnchors = []; // bodyA-local anchor per point, for next-tick matching
        this.next_manifold = null; // linked-list view, maintained by ContactManifoldList._relink()
    }

    get pointCount() { return this.points.length; }
}

ContactManifold.MAX_POINTS = 4;
// Base match radius; Update.js._matchDistance widens it by the point's tangential travel per tick.
ContactManifold.MATCH_DISTANCE = 0.05;
// Signed-distance band where GJK/EPA's normal is ambiguous; a matched point keeps its old normal.
ContactManifold.EXACT_TOUCH_BAND = 0.001;
// Same-tick mesh-face points closer than this (meters) with a matching normal are merged (Reduction.js).
ContactManifold.COINCIDENCE_DIST = 0.05;

ContactManifold._scratchNormal = new Vector3();
ContactManifold._scratchRA = new Vector3();
ContactManifold._scratchRB = new Vector3();
ContactManifold._scratchInvRot = new Quaternion();
ContactManifold._scratchLocal = new Vector3();

ActionPhysics.ContactManifold = ContactManifold;
