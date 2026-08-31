// Per-substep contact geometry refresh: re-measures existing manifold points against current
// predicted transforms. Geometry only - never adds, removes, or re-matches points.
var proto = NarrowPhase.prototype;

// Speed-squared below which mesh-face contact geometry stays at tick-start values for the substeps.
// Looser than the solver's rest thresholds: a nearly-settled curved prop jitters a few cm/s across a
// tile seam, and re-clipping it each substep only to bail to a full re-expansion is pure cost.
// NarrowPhase.step() still refreshes the tick-start clip once per tick.
NarrowPhase.REFRESH_REST_LIN_SQ = 0.30 * 0.30;
NarrowPhase.REFRESH_REST_ANG_SQ = 0.60 * 0.60;

proto.refreshManifoldGeometry = function (manifolds) {
    // Contacts pooled here are transient - copied into manifold points or the accumulator within
    // this call - and step()'s were already consumed by manifolds.refresh(). Rewinding each substep
    // keeps _poolIndex from climbing all tick and growing the pool.
    this._poolIndex = 0;
    this._ctHintNormal = null; // stale from step()'s pair loop; each branch below sets it as needed
    for (const manifold of manifolds.values()) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;

        if (manifold.points.length > 0 && this._allMeshFace(manifold)) {
            // At rest -> geometry isn't moving this tick; skip the re-clip entirely.
            if (this._pairAtRest(bodyA, bodyB)) continue;
            // Fast path: every point carries its source triangle (ConvexTri), so re-clip only
            // those triangles against the current transform - no midphase re-expansion, no
            // GJK/EPA fallback on non-contact triangles.
            if (this._allMeshTriTagged(manifold)) {
                // Fast re-clip of just the stored triangles. Returns false when a stored triangle
                // stopped producing a contact (the body drifted toward an adjacent tile) - then
                // fall back to the full re-expansion so the new supporting triangle is found.
                if (this._refreshMeshFaceManifoldFast(manifold, bodyA, bodyB)) continue;
                if (this._midphase) {
                    this._ctHintNormal = manifold.points[0].fromMeshFace ? manifold.points[0].normal : null;
                    this._refreshMeshFaceManifold(manifold, bodyA, bodyB);
                    this._ctHintNormal = null;
                }
                continue;
            }
            // Slow path: mixed / TriTri mesh-face manifold - re-expand the pair. Hand any ConvexTri
            // sub-pair the established normal too.
            if (this._midphase) {
                this._ctHintNormal = manifold.points[0].fromMeshFace ? manifold.points[0].normal : null;
                this._refreshMeshFaceManifold(manifold, bodyA, bodyB);
                this._ctHintNormal = null;
                continue;
            }
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

proto._pairAtRest = function (bodyA, bodyB) {
    const spd = this._tickStartSpeedSq;
    if (!spd) return false;
    const a = spd.get(bodyA.id), b = spd.get(bodyB.id);
    // A body with no entry is static/kinematic (never moves) - treat as at rest.
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

// Re-clip only the distinct source triangles the manifold's points came from. The mesh side is
// static ground, so its stored world verts are still valid; only the convex has moved. One
// closed-form test per triangle - no BVH query, no re-expansion, no GJK/EPA fallback.
proto._refreshMeshFaceManifoldFast = function (manifold, bodyA, bodyB) {
    const pts = manifold.points;
    const acc = this._meshRefreshAcc || (this._meshRefreshAcc = []);
    acc.length = 0;

    const cvx = pts[0].meshTriIsSideA ? bodyB : bodyA;
    // This calls the closed-form test directly on the body's own shape, so the shape must be one
    // those tests handle - a CompoundShape passed to a support-map test throws.
    //
    // Boxes are excluded: re-clipping only the stored triangles works for a curved convex, which
    // rests on essentially one triangle, but a box face spans several tiles and the stored set can
    // miss one that carries it, dropping that support and sinking the box through. Boxes take the
    // slow route, which re-expands and finds every triangle under the face.
    const isBox = cvx.shape instanceof BoxShape;
    if (isBox || !ConvexTri._isCurvedConvex(cvx.shape)) return false;
    const cvxSide = { shape: cvx.shape, position: cvx.position, rotation: cvx.rotation };
    const triSide = this._meshRefreshTri || (this._meshRefreshTri = {
        shape: new TriangleShape(new Vector3(), new Vector3(), new Vector3()),
        position: new Vector3(0, 0, 0), rotation: new Quaternion(), bodyCenter: new Vector3()
    });

    for (let i = 0; i < pts.length; i++) {
        const src = pts[i];
        // Points usually share one triangle - skip a re-test for a triangle already done this call.
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

        // Established normal as the orientation hint - the convex-centre heuristic is unsafe for a
        // nearly-settled body. Pooled contacts stay valid for the rest of the tick, so they go
        // straight into `acc`; the next ConvexTri.test reuses `fresh` but not what it already holds.
        const self = this;
        const nc = function () { return self._nextPooledContact(); };
        const fresh = this._meshRefreshFresh || (this._meshRefreshFresh = []);
        fresh.length = 0;
        const a = src.meshTriIsSideA ? triSide : cvxSide;
        const b = src.meshTriIsSideA ? cvxSide : triSide;
        const r = ConvexTri.test(a, b, fresh, nc, src.normal);
        // The stored triangle only stays trustworthy while the body's contact feature is still over
        // it. Once it moves off (settling onto a neighbour tile) the re-clip returns nothing, or for
        // ConvexTri only shallow edge probes - enough to look non-empty but missing the real depth,
        // which starves then over-corrects the solver. Bail to the full re-expansion.
        if (!r || !ConvexTri.lastDeepestInTriangle) return false;
        for (let c = 0; c < r.length; c++) if (r[c].fromMeshFace) acc.push(r[c]);
    }

    if (acc.length === 0) return false; // lost contact this substep - re-expand to confirm

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

// Re-clip the pair and move each existing point onto its nearest fresh clip point. Geometry only;
// point count and warm-start lambdas are untouched.
proto._refreshMeshFaceManifold = function (manifold, bodyA, bodyB) {
    const sides = this._midphase.expandPairSides(bodyA, bodyB);
    const sidesA = sides.a, sidesB = sides.b;
    // _testPrimitivePair reuses its scratch array per call, so copy the results into a private list.
    const acc = this._meshRefreshAcc || (this._meshRefreshAcc = []);
    acc.length = 0;
    // Only fromMeshFace contacts are kept below, and GJK/EPA never produces those - so run just the
    // closed-form face tests here. Going through the full _testPrimitivePair dispatch would fire
    // GJK on every non-contact triangle of the expansion purely to discard the result.
    const self = this;
    const nc = function () { return self._nextPooledContact(); };
    for (let i = 0; i < sidesA.length; i++) {
        for (let j = 0; j < sidesB.length; j++) {
            const pa = sidesA[i], pb = sidesB[j];
            const pc = this._pairResultScratch;
            pc.length = 0;
            if (BoxTriFace.applies(pa, pb)) BoxTriFace.test(pa, pb, pc, nc, this._ctHintNormal);
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

// Two mesh-face contacts sharing the same source triangle (vertex A coincident is enough - the
// tiles are distinct and non-overlapping, so a shared A vertex means the same tile triangle).
var _COINCIDENT_TRI_SQ = 1e-8;
function _coincidentTri(p, q) {
    if (!p.meshTriValid || !q.meshTriValid) return false;
    const dx = p.meshTriA.x - q.meshTriA.x, dy = p.meshTriA.y - q.meshTriA.y, dz = p.meshTriA.z - q.meshTriA.z;
    return dx * dx + dy * dy + dz * dz < _COINCIDENT_TRI_SQ;
}
