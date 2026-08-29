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
        const fresh = this._testPrimitivePair(placedA, placedB);

        // Update the nearest existing point (world space), keeping its warm-start lambda.
        let best = null, bestDistSq = Infinity;
        for (let i = 0; i < manifold.points.length; i++) {
            const p = manifold.points[i];
            const dx = p.point.x - fresh.point.x, dy = p.point.y - fresh.point.y, dz = p.point.z - fresh.point.z;
            const d = dx * dx + dy * dy + dz * dz;
            if (d < bestDistSq) { bestDistSq = d; best = p; }
        }
        if (!best) continue;
        best.point.copy(fresh.point);
        best.pointOnA.copy(fresh.pointOnA);
        best.pointOnB.copy(fresh.pointOnB);
        best.signedDistance = fresh.signedDistance;
        // Keep the established normal through the exact-touch band (ContactManifold.EXACT_TOUCH_BAND)
        // - the ambiguous diagonal GJK/EPA returns near signed-distance 0 would otherwise clobber a
        // good resting normal every substep.
        if (Math.abs(fresh.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) best.normal.copy(fresh.normal);
        // Re-anchor to current geometry so C is measured from where the feature is now. Lambda
        // untouched (warm start survives).
        best.setLocalAnchors(bodyA, bodyB);
    }
};
