// Adding a point, and the 4-point manifold cap reduction: always keep the deepest point, and among
// the rest keep whichever 3 form the largest-area quadrilateral with it - maximizing spread gives
// better torque resistance (a corner-only manifold rocks; 4 spread corners don't).
var proto = ContactManifold.prototype;

proto._addPoint = function (contact) {
    const point = contact.clone();
    point.normalLambda = 0; point.tangentLambda1 = 0; point.tangentLambda2 = 0; // fresh: no warm-start data
    point.setLocalAnchors(this.bodyA, this.bodyB);
    const local = ContactManifold._toLocal(this.bodyA, point.pointOnA);

    if (this.points.length < ContactManifold.MAX_POINTS) {
        this.points.push(point);
        this._localAnchors.push(local);
        return;
    }
    this._reduceToFour(point, local);
};

proto._reduceToFour = function (candidatePoint, candidateLocal) {
    // Deepest = largest signedDistance (most overlapping), the point the solver most needs.
    let deepestIdx = -1, deepestVal = candidatePoint.signedDistance;
    for (let i = 0; i < this.points.length; i++) {
        if (this.points[i].signedDistance > deepestVal) { deepestVal = this.points[i].signedDistance; deepestIdx = i; }
    }
    const deepestIsCandidate = deepestIdx === -1;
    const deepestPoint = deepestIsCandidate ? candidatePoint : this.points[deepestIdx];

    // Remaining candidates for the 3 non-deepest slots: exactly 4 of them (4 existing + candidate,
    // minus the deepest), so there are exactly 4 possible triples - enumerate directly.
    const pool = [];
    for (let i = 0; i < this.points.length; i++) if (i !== deepestIdx) pool.push({ point: this.points[i], local: this._localAnchors[i] });
    if (!deepestIsCandidate) pool.push({ point: candidatePoint, local: candidateLocal });

    let bestOmit = 0, bestArea = -1;
    for (let omit = 0; omit < pool.length; omit++) {
        const tri = [];
        for (let i = 0; i < pool.length; i++) if (i !== omit) tri.push(pool[i]);
        const area = ContactManifold._quadArea(deepestPoint.point, tri[0].point.point, tri[1].point.point, tri[2].point.point);
        if (area > bestArea) { bestArea = area; bestOmit = omit; }
    }

    const kept = [];
    for (let i = 0; i < pool.length; i++) if (i !== bestOmit) kept.push(pool[i]);

    this.points = deepestIsCandidate ? [candidatePoint] : [deepestPoint];
    this._localAnchors = deepestIsCandidate ? [candidateLocal] : [this._localAnchors[deepestIdx]];
    for (let i = 0; i < kept.length; i++) { this.points.push(kept[i].point); this._localAnchors.push(kept[i].local); }
};

// Rough quad area via the two diagonal-split triangles - a fine proxy for "how spread out", not a
// proper convex-hull-ordered area.
ContactManifold._quadArea = function (a, b, c, d) {
    return ContactManifold._triArea(a, b, c) + ContactManifold._triArea(a, c, d);
};

ContactManifold._triArea = function (a, b, c) {
    const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
    const cx = aby * acz - abz * acy, cy = abz * acx - abx * acz, cz = abx * acy - aby * acx;
    return 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
};
