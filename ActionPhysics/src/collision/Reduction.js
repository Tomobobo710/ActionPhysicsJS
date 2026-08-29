// Point cap reduction, scoped per (childA, childB) group: keep the deepest point, and among the
// rest keep whichever 3 form the largest-area quadrilateral with it - better torque resistance
// than a corner-only manifold.
var proto = ContactManifold.prototype;

proto._addPoint = function (contact) {
    const point = contact.clone();
    point.normalLambda = 0; point.tangentLambda1 = 0; point.tangentLambda2 = 0; // fresh: no warm-start data
    point.setLocalAnchors(this.bodyA, this.bodyB);
    const local = ContactManifold._toLocal(this.bodyA, point.pointOnA);
    const groupKey = ContactManifold._groupKey(contact.childA, contact.childB);

    const groupIndices = [];
    for (let i = 0; i < this.points.length; i++) {
        if (ContactManifold._groupKey(this.points[i].childA, this.points[i].childB) === groupKey) groupIndices.push(i);
    }

    if (groupIndices.length < ContactManifold.MAX_POINTS) {
        this.points.push(point);
        this._localAnchors.push(local);
        return;
    }
    this._reduceGroupToFour(groupIndices, point, local);
};

// groupIndices: indices into this.points/_localAnchors sharing the new point's group.
proto._reduceGroupToFour = function (groupIndices, candidatePoint, candidateLocal) {
    let deepestAt = -1, deepestVal = candidatePoint.signedDistance;
    for (let k = 0; k < groupIndices.length; k++) {
        const i = groupIndices[k];
        if (this.points[i].signedDistance > deepestVal) { deepestVal = this.points[i].signedDistance; deepestAt = i; }
    }
    const deepestIsCandidate = deepestAt === -1;
    const deepestPoint = deepestIsCandidate ? candidatePoint : this.points[deepestAt];

    const pool = [];
    for (let k = 0; k < groupIndices.length; k++) {
        const i = groupIndices[k];
        if (i !== deepestAt) pool.push({ point: this.points[i], local: this._localAnchors[i] });
    }
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

    const finalPoints = [deepestPoint].concat(kept.map(k => k.point));
    const finalLocals = [deepestIsCandidate ? candidateLocal : this._localAnchors[deepestAt]].concat(kept.map(k => k.local));
    for (let s = 0; s < groupIndices.length; s++) {
        this.points[groupIndices[s]] = finalPoints[s];
        this._localAnchors[groupIndices[s]] = finalLocals[s];
    }
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
