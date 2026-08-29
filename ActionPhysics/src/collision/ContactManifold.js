// Persistent contact state for one body pair, across ticks. Owns point lifetime; update() (once
// per tick) matches/warm-starts/adds/removes, MAX_POINTS caps per (childA, childB) group so one
// compound child can't evict another's points. See Update.js, Reduction.js.
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
// Base floor; Update.js's _matchDistance widens by tangential travel per tick.
ContactManifold.MATCH_DISTANCE = 0.05;
// Half-width of the exact-touch band where GJK/EPA's normal is ambiguous (see Update.js).
ContactManifold.EXACT_TOUCH_BAND = 0.001;

ContactManifold._scratchNormal = new Vector3();
ContactManifold._scratchRA = new Vector3();
ContactManifold._scratchRB = new Vector3();

// Stable group key for (childA, childB); null child = whole body. Lazily-assigned id per child.
ContactManifold._nextChildId = 1;
ContactManifold._groupKey = function (childA, childB) {
    if (childA && childA._manifoldGroupId === undefined) childA._manifoldGroupId = ContactManifold._nextChildId++;
    if (childB && childB._manifoldGroupId === undefined) childB._manifoldGroupId = ContactManifold._nextChildId++;
    return (childA ? childA._manifoldGroupId : 0) + ':' + (childB ? childB._manifoldGroupId : 0);
};

ActionPhysics.ContactManifold = ContactManifold;
