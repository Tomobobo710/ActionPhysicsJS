var proto = NarrowPhase.prototype;

NarrowPhase.REFRESH_REST_LIN_SQ = 0.30 * 0.30;
NarrowPhase.REFRESH_REST_ANG_SQ = 0.60 * 0.60;

proto.refreshManifoldGeometry = function (manifolds) {

    this._poolIndex = 0;
    this._ctHintNormal = null;
    for (const manifold of manifolds.values()) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;

        if (manifold.points.length > 0 && this._allMeshFace(manifold)) {

            if (this._pairAtRest(bodyA, bodyB)) continue;

            if (this._allMeshTriTagged(manifold)) {

                if (this._refreshMeshFaceManifoldFast(manifold, bodyA, bodyB)) continue;
                if (this._midphase) {
                    this._ctHintNormal = manifold.points[0].fromMeshFace ? manifold.points[0].normal : null;
                    this._refreshMeshFaceManifold(manifold, bodyA, bodyB);
                    this._ctHintNormal = null;
                }
                continue;
            }

            if (this._midphase) {
                this._ctHintNormal = manifold.points[0].fromMeshFace ? manifold.points[0].normal : null;
                this._refreshMeshFaceManifold(manifold, bodyA, bodyB);
                this._ctHintNormal = null;
                continue;
            }
            continue;
        }

        if (this._isCompoundOrMesh(bodyA.shape) || this._isCompoundOrMesh(bodyB.shape)) continue;

        const placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
        const placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };
        const freshList = this._testPrimitivePair(placedA, placedB);
        if (freshList.length === 0) continue;

        const claimed = this._refreshClaimed || (this._refreshClaimed = []);
        claimed.length = 0;
        for (let f = 0; f < freshList.length; f++) claimed.push(false);

        for (let i = 0; i < manifold.points.length; i++) {
            const p = manifold.points[i];
            let best = null, bestIdx = -1, bestDistSq = Infinity;
            for (let f = 0; f < freshList.length; f++) {
                if (claimed[f]) continue;
                const fresh = freshList[f];
                const dx = p.point.x - fresh.point.x, dy = p.point.y - fresh.point.y, dz = p.point.z - fresh.point.z;
                const d = dx * dx + dy * dy + dz * dz;
                if (d < bestDistSq) { bestDistSq = d; best = fresh; bestIdx = f; }
            }
            if (!best) continue;
            claimed[bestIdx] = true;
            p.point.copy(best.point);
            p.pointOnA.copy(best.pointOnA);
            p.pointOnB.copy(best.pointOnB);
            p.signedDistance = best.signedDistance;

            if (Math.abs(best.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) p.normal.copy(best.normal);
            p.setLocalAnchors(bodyA, bodyB);
        }
    }
};

proto._pairAtRest = function (bodyA, bodyB) {
    const spd = this._tickStartSpeedSq;
    if (!spd) return false;
    const a = spd.get(bodyA.id), b = spd.get(bodyB.id);

    if (a && (a.lin > NarrowPhase.REFRESH_REST_LIN_SQ || a.ang > NarrowPhase.REFRESH_REST_ANG_SQ)) return false;
    if (b && (b.lin > NarrowPhase.REFRESH_REST_LIN_SQ || b.ang > NarrowPhase.REFRESH_REST_ANG_SQ)) return false;
    return true;
};

proto._allMeshFace = function (manifold) {
    const pts = manifold.points;
    for (let i = 0; i < pts.length; i++) if (!pts[i].fromMeshFace) return false;
    return true;
};

proto._allMeshTriTagged = function (manifold) {
    const pts = manifold.points;
    for (let i = 0; i < pts.length; i++) if (!pts[i].meshTriValid) return false;
    return true;
};

proto._refreshMeshFaceManifoldFast = function (manifold, bodyA, bodyB) {
    const pts = manifold.points;
    const acc = this._meshRefreshAcc || (this._meshRefreshAcc = []);
    acc.length = 0;

    const cvx = pts[0].meshTriIsSideA ? bodyB : bodyA;

    const isBox = cvx.shape instanceof BoxShape;
    if (isBox || !ConvexTri._isCurvedConvex(cvx.shape)) return false;
    const cvxSide = { shape: cvx.shape, position: cvx.position, rotation: cvx.rotation };
    const triSide = this._meshRefreshTri || (this._meshRefreshTri = {
        shape: new TriangleShape(new Vector3(), new Vector3(), new Vector3()),
        position: new Vector3(0, 0, 0), rotation: new Quaternion(), bodyCenter: new Vector3()
    });

    for (let i = 0; i < pts.length; i++) {
        const src = pts[i];

        let seen = false;
        for (let j = 0; j < i; j++) {
            const o = pts[j];
            if (_coincidentTri(src, o)) { seen = true; break; }
        }
        if (seen) continue;

        triSide.shape.a.copy(src.meshTriA);
        triSide.shape.b.copy(src.meshTriB);
        triSide.shape.c.copy(src.meshTriC);
        triSide.bodyCenter.copy(src.meshTriBodyCenter);

        const self = this;
        const nc = function () { return self._nextPooledContact(); };
        const fresh = this._meshRefreshFresh || (this._meshRefreshFresh = []);
        fresh.length = 0;
        const a = src.meshTriIsSideA ? triSide : cvxSide;
        const b = src.meshTriIsSideA ? cvxSide : triSide;
        const r = ConvexTri.test(a, b, fresh, nc, src.normal);

        if (!r || !ConvexTri.lastDeepestInTriangle) return false;
        for (let c = 0; c < r.length; c++) if (r[c].fromMeshFace) acc.push(r[c]);
    }

    if (acc.length === 0) return false;

    for (let i = 0; i < pts.length; i++) {
        const p = pts[i];
        let best = null, bestDistSq = Infinity;
        for (let f = 0; f < acc.length; f++) {
            const dx = p.point.x - acc[f].point.x, dy = p.point.y - acc[f].point.y, dz = p.point.z - acc[f].point.z;
            const d = dx * dx + dy * dy + dz * dz;
            if (d < bestDistSq) { bestDistSq = d; best = acc[f]; }
        }
        if (!best) continue;
        p.point.copy(best.point);
        p.pointOnA.copy(best.pointOnA);
        p.pointOnB.copy(best.pointOnB);
        p.signedDistance = best.signedDistance;
        if (Math.abs(best.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) p.normal.copy(best.normal);
        p.setLocalAnchors(bodyA, bodyB);
    }
    return true;
};

proto._refreshMeshFaceManifold = function (manifold, bodyA, bodyB) {
    const sides = this._midphase.expandPairSides(bodyA, bodyB);
    const sidesA = sides.a, sidesB = sides.b;

    const acc = this._meshRefreshAcc || (this._meshRefreshAcc = []);
    acc.length = 0;

    const self = this;
    const nc = function () { return self._nextPooledContact(); };
    for (let i = 0; i < sidesA.length; i++) {
        for (let j = 0; j < sidesB.length; j++) {
            const pa = sidesA[i], pb = sidesB[j];
            const pc = this._pairResultScratch;
            pc.length = 0;
            if (BoxTriFace.applies(pa, pb)) BoxTriFace.test(pa, pb, pc, nc, this._ctHintNormal);
            else if (CapTriFace.applies(pa, pb)) CapTriFace.test(pa, pb, pc, nc, this._ctHintNormal);
            else if (ConvexTri.applies(pa, pb)) ConvexTri.test(pa, pb, pc, nc, this._ctHintNormal);
            else if (TriTri.applies(pa, pb)) TriTri.test(pa, pb, pc, nc);
            else continue;
            for (let c = 0; c < pc.length; c++) if (pc[c].fromMeshFace) {
                const s = this._nextPooledContact();
                s.copy(pc[c]);
                acc.push(s);
            }
        }
    }
    if (acc.length === 0) return;

    for (let i = 0; i < manifold.points.length; i++) {
        const p = manifold.points[i];
        let best = null, bestDistSq = Infinity;
        for (let f = 0; f < acc.length; f++) {
            const dx = p.point.x - acc[f].point.x, dy = p.point.y - acc[f].point.y, dz = p.point.z - acc[f].point.z;
            const d = dx * dx + dy * dy + dz * dz;
            if (d < bestDistSq) { bestDistSq = d; best = acc[f]; }
        }
        if (!best) continue;
        p.point.copy(best.point);
        p.pointOnA.copy(best.pointOnA);
        p.pointOnB.copy(best.pointOnB);
        p.signedDistance = best.signedDistance;
        if (Math.abs(best.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) p.normal.copy(best.normal);
        p.setLocalAnchors(bodyA, bodyB);
    }
};

var _COINCIDENT_TRI_SQ = 1e-8;
function _coincidentTri(p, q) {
    if (!p.meshTriValid || !q.meshTriValid) return false;
    const dx = p.meshTriA.x - q.meshTriA.x, dy = p.meshTriA.y - q.meshTriA.y, dz = p.meshTriA.z - q.meshTriA.z;
    return dx * dx + dy * dy + dz * dz < _COINCIDENT_TRI_SQ;
}
