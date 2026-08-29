// Per-substep contact geometry refresh: re-measures existing manifold points against current
// predicted transforms. Geometry only - never adds, removes, or re-matches points.
var proto = NarrowPhase.prototype;

proto.refreshManifoldGeometry = function (manifolds) {
    for (const manifold of manifolds.values()) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;

        // Mesh face manifold: re-clip it each substep, like the primitive path below, so its
        // geometry tracks the body as it settles within the tick instead of staying frozen.
        if (this._midphase && manifold.points.length > 0 && this._allMeshFace(manifold)) {
            this._refreshMeshFaceManifold(manifold, bodyA, bodyB);
            continue;
        }

        // Other compound/mesh contacts don't track per-point triangle identity - keep tick-start geometry.
        if (this._isCompoundOrMesh(bodyA.shape) || this._isCompoundOrMesh(bodyB.shape)) continue;

        const placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
        const placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };
        const freshList = this._testPrimitivePair(placedA, placedB);
        if (freshList.length === 0) continue;

        // Move each existing point onto its nearest fresh point (BoxBox reports up to 4).
        for (let i = 0; i < manifold.points.length; i++) {
            const p = manifold.points[i];
            let best = null, bestDistSq = Infinity;
            for (let f = 0; f < freshList.length; f++) {
                const fresh = freshList[f];
                const dx = p.point.x - fresh.point.x, dy = p.point.y - fresh.point.y, dz = p.point.z - fresh.point.z;
                const d = dx * dx + dy * dy + dz * dz;
                if (d < bestDistSq) { bestDistSq = d; best = fresh; }
            }
            if (!best) continue;
            p.point.copy(best.point);
            p.pointOnA.copy(best.pointOnA);
            p.pointOnB.copy(best.pointOnB);
            p.signedDistance = best.signedDistance;
            // Keep the established normal inside the exact-touch band (GJK/EPA is ambiguous there).
            if (Math.abs(best.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) p.normal.copy(best.normal);
            p.setLocalAnchors(bodyA, bodyB); // lambda untouched, warm start survives
        }
    }
};

proto._allMeshFace = function (manifold) {
    const pts = manifold.points;
    for (let i = 0; i < pts.length; i++) if (!pts[i].fromMeshFace) return false;
    return true;
};

// Re-clip the pair and move each existing point onto its nearest fresh clip point. Geometry only;
// point count and warm-start lambdas are untouched.
proto._refreshMeshFaceManifold = function (manifold, bodyA, bodyB) {
    const primitivePairs = this._midphase.expandPair(bodyA, bodyB);
    // _testPrimitivePair reuses its scratch array per call, so copy the results into a private list.
    const acc = this._meshRefreshAcc || (this._meshRefreshAcc = []);
    acc.length = 0;
    for (let i = 0; i < primitivePairs.length; i++) {
        const pc = this._testPrimitivePair(primitivePairs[i].a, primitivePairs[i].b);
        for (let c = 0; c < pc.length; c++) if (pc[c].fromMeshFace) {
            const s = this._nextPooledContact();
            s.copy(pc[c]);
            acc.push(s);
        }
    }
    if (acc.length === 0) return; // lost contact this substep - keep last good geometry

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
