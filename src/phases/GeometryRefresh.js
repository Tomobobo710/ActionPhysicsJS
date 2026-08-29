// Per-substep contact geometry refresh: re-measures existing manifold points against the bodies'
// current (predicted) transforms, called by Solver.step's `refresh`. Updates geometry only - never
// adds/removes/re-matches points, that stays the manifold's once-per-tick job.
var proto = NarrowPhase.prototype;

proto.refreshManifoldGeometry = function (manifolds) {
    for (const manifold of manifolds.values()) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
        // Compound/mesh contacts came from an expanded child/triangle whose per-point identity
        // isn't tracked here, so those manifolds keep tick-start geometry.
        if (this._isCompoundOrMesh(bodyA.shape) || this._isCompoundOrMesh(bodyB.shape)) continue;

        const placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
        const placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };
        const freshList = this._testPrimitivePair(placedA, placedB);
        if (freshList.length === 0) continue;

        // Match each existing point to its own nearest fresh point (world space) - a multi-point
        // pair type (BoxBox) reports up to 4 fresh points per tick, one per manifold point, not a
        // single shared measurement like the other closed-form/GJK-EPA pairs.
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
            // Keep the established normal through the exact-touch band (ContactManifold.EXACT_TOUCH_BAND)
            // - the ambiguous diagonal GJK/EPA returns near signed-distance 0 would otherwise clobber a
            // good resting normal every substep.
            if (Math.abs(best.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) p.normal.copy(best.normal);
            // Re-anchor to current geometry so C is measured from where the feature is now. Lambda
            // untouched (warm start survives).
            p.setLocalAnchors(bodyA, bodyB);
        }
    }
};
