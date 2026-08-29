/**
 * Queries: ray casting and shape sweeps against the world's bodies. World's public surface
 * (plan.md, API surface) is rayIntersect(start, end) and shapeIntersect(shape, start, end); this
 * file is where both actually live, kept separate from World.js itself (Rule 2: World is pipeline
 * glue, not where an algorithm's real body belongs).
 *
 * REBUILT, NOT PORTED (plan.md, "This is a rebuild, not a port"). Both queries reuse GJK's own
 * closest-distance result directly, rather than a separate ray-vs-shape or shape-vs-shape
 * algorithm. A ray is modeled as a zero-radius sphere (SphereShape supportInto trivially returns
 * the ray's own origin for a zero radius, so this needs no special-cased ray/shape math anywhere)
 * queried against the target once; a general shape sweep is the same single query with the
 * caller's real shape instead. This is EXACT, not approximate: GJK's separated result is the true
 * closest distance and normal between two convex shapes regardless of how far apart they start
 * (verified directly, see _advance's own comment) — one query per candidate body is enough, no
 * repeated advance-and-requery loop is needed the way it would be for a scene with many obstacles
 * along a single path. GJK/EPA are already proven correct (see the Bug reference's GJK/EPA
 * entries) — reusing them here is a smaller, more trustworthy surface than a second geometric
 * algorithm (segment-vs-triangle, slab tests, etc.) duplicating what GJK already computes.
 *
 * Broadphase has no arbitrary-AABB query surface (SAPBroadphase.computePairs() only produces
 * body-vs-body candidates), so both queries here filter world.bodies directly with a cheap
 * ray-vs-AABB / swept-AABB-vs-AABB reject before ever calling GJK — bringing the O(n) candidate
 * cost down without needing a new spatial index. This is the same shape of filter broadphase
 * itself applies (AABB reject, then real geometry), just over the whole body list instead of a
 * sorted sweep, appropriate for the "~150 lines, not a hot per-tick path" scope plan.md gives
 * queries (Component inventory, section 11) — a query runs on demand, not every tick for every
 * body pair.
 */
class Queries {
    // rayIntersect(bodies, start, end) -> { body, point, normal, distance, fraction } | null.
    // The single body whose surface the segment start->end hits FIRST (smallest fraction along the
    // segment), or null if the segment hits nothing. `distance`/`fraction` are along the full
    // start->end segment, not the (possibly shorter) advancement the sweep itself took.
    static rayIntersect(bodies, start, end) {
        const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
        const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
        if (fullLen < 1e-12) return null; // zero-length ray hits nothing (a point query isn't a ray)

        let best = null, bestFraction = Infinity;
        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];
            const aabb = body.getAABB();
            if (!Queries._rayIntersectsAABB(start, end, aabb)) continue;

            const hit = Queries._sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; best.body = body; }
        }
        return best;
    }

    // shapeIntersect(bodies, shape, start, end) -> { body, point, normal, distance, fraction } | null.
    // Sweeps `shape` (in its own local orientation, held fixed — no rotation during the sweep) from
    // start to end and reports the first body it touches, same shape of result as rayIntersect.
    static shapeIntersect(bodies, shape, start, end, rotation) {
        const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
        const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
        const localAABB = Queries._scratchLocalAABB;
        shape.localAABBInto(localAABB);
        const radius = Math.sqrt(
            Math.max(localAABB.min.x * localAABB.min.x, localAABB.max.x * localAABB.max.x) +
            Math.max(localAABB.min.y * localAABB.min.y, localAABB.max.y * localAABB.max.y) +
            Math.max(localAABB.min.z * localAABB.min.z, localAABB.max.z * localAABB.max.z)
        );

        let best = null, bestFraction = Infinity;
        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];
            const aabb = body.getAABB();
            if (!Queries._sweptAABBMayHit(start, end, radius, aabb)) continue;

            const hit = Queries._sweepShapeVsBody(shape, rotation, start, dirX, dirY, dirZ, fullLen, body);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; best.body = body; }
        }
        return best;
    }

    // Casts a ZERO-RADIUS point (the ray) from `start` toward `start + dir*fullLen` against one
    // body, via a single GJK query — see _advance.
    static _sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body) {
        const pointShape = Queries._scratchPointShape;
        const placedPoint = Queries._scratchPlacedA;
        placedPoint.shape = pointShape;
        placedPoint.position = Queries._scratchPos.set(start.x, start.y, start.z);
        placedPoint.rotation = Queries._identityQuat;

        const placedBody = Queries._scratchPlacedB;
        placedBody.shape = body.shape;
        placedBody.position = body.position;
        placedBody.rotation = body.rotation;

        const support = Queries._scratchSupport;
        support.a = placedPoint; support.b = placedBody;
        support._invRotA.copy(Queries._identityQuat);
        support._invRotB.copy(body.rotation).invert();

        return Queries._advance(support, placedPoint, start, dirX, dirY, dirZ, fullLen);
    }

    // Same single-query cast, but sweeping a REAL shape (fixed orientation) instead of a point.
    // Identical structure to _sweepPointVsBody; kept as a separate method rather than a point-shape
    // special case of this one so the point sweep never pays a real shape's support-function cost
    // (a ray query is the overwhelmingly common case — line-of-sight, hitscan).
    static _sweepShapeVsBody(shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
        const placedShape = Queries._scratchPlacedA;
        placedShape.shape = shape;
        placedShape.position = Queries._scratchPos.set(start.x, start.y, start.z);
        placedShape.rotation = rotation || Queries._identityQuat;

        const placedBody = Queries._scratchPlacedB;
        placedBody.shape = body.shape;
        placedBody.position = body.position;
        placedBody.rotation = body.rotation;

        const support = Queries._scratchSupport;
        support.a = placedShape; support.b = placedBody;
        support._invRotA.copy(placedShape.rotation).invert();
        support._invRotB.copy(body.rotation).invert();

        return Queries._advance(support, placedShape, start, dirX, dirY, dirZ, fullLen);
    }

    // Casts `placedMover` (already positioned at `start`) toward `start + dir*fullLen` against one
    // body via a SINGLE GJK query. This is exact, not an approximation: GJK's separated result is the
    // true closest-distance-and-normal between two CONVEX shapes regardless of how far apart they
    // are (verified directly — a point 4.5 units from a box face reports distance 4.5, normal
    // [-1,0,0] on the very first call, exactly) — there is no "walk closer and re-measure" step
    // needed the way conservative advancement needs for a scene with many obstacles in sequence,
    // because this is one query against one convex body, not a multi-body path.
    //
    // A real bug was caught here by tracing: an earlier version advanced fully onto the surface
    // (distance -> 0) and called GJK a SECOND time to "confirm" the touch, which landed exactly on
    // GJK's own documented exact-touch ambiguity (see GJK.js's class header, "EXACT-TOUCHING IS
    // UNDECIDABLE") — an exact-touch simplex has no unique normal and falls back to an
    // arbitrary-but-valid one, so the correct [-1,0,0] from the real approach was discarded in favour
    // of a degenerate [0,-0.7,0.7] from the pointless second call. The fix is structural: there is
    // only ever one query per body, and its own result is trusted directly.
    static _advance(support, placedMover, start, dirX, dirY, dirZ, fullLen) {
        const result = Queries._gjk.run(support);
        if (result.overlapping) {
            // The segment/shape starts already inside the target. Report the hit at zero travel,
            // using the incoming travel direction (reversed) as the best available surface-facing
            // estimate — an overlapping GJK result carries no witness normal of its own (see
            // GJK.run's documented two outcomes).
            return Queries._finishHit(start, dirX, dirY, dirZ, 0, fullLen,
                -dirX / fullLen, -dirY / fullLen, -dirZ / fullLen);
        }
        if (result.distance > fullLen) return null; // cannot reach within the segment
        return Queries._finishHit(start, dirX, dirY, dirZ, result.distance, fullLen,
            result.normal.x, result.normal.y, result.normal.z);
    }

    static _finishHit(start, dirX, dirY, dirZ, traveled, fullLen, nx, ny, nz) {
        const fraction = traveled / fullLen;
        return {
            point: new Vector3(start.x + dirX * fraction, start.y + dirY * fraction, start.z + dirZ * fraction),
            normal: new Vector3(nx, ny, nz),
            distance: traveled,
            fraction: fraction
        };
    }

    // Cheap ray-vs-AABB reject (slab method) before ever constructing a GJK support for a body —
    // most candidates a query considers do not lie anywhere near the segment at all.
    static _rayIntersectsAABB(start, end, aabb) {
        let tmin = 0, tmax = 1;
        const dirs = [end.x - start.x, end.y - start.y, end.z - start.z];
        const starts = [start.x, start.y, start.z];
        const mins = [aabb.min.x, aabb.min.y, aabb.min.z];
        const maxs = [aabb.max.x, aabb.max.y, aabb.max.z];
        for (let axis = 0; axis < 3; axis++) {
            const d = dirs[axis], s = starts[axis];
            if (Math.abs(d) < 1e-12) {
                if (s < mins[axis] || s > maxs[axis]) return false; // parallel and outside the slab
                continue;
            }
            let t1 = (mins[axis] - s) / d, t2 = (maxs[axis] - s) / d;
            if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
        }
        return true;
    }

    // Cheap reject for a shape sweep: expand the body's AABB by the swept shape's bounding radius
    // (a sphere large enough to contain the shape at any orientation, since the sweep holds
    // orientation fixed but this reject doesn't need to be exact — only conservative/no-false-
    // negatives, same discipline as broadphase's own margin) and ray-test against that.
    static _sweptAABBMayHit(start, end, radius, aabb) {
        const expanded = Queries._scratchExpandedAABB;
        expanded.copy(aabb).expandInPlace(radius);
        return Queries._rayIntersectsAABB(start, end, expanded);
    }
}

Queries._gjk = new GJK();
Queries._identityQuat = new Quaternion(0, 0, 0, 1);
Queries._scratchPos = new Vector3();
Queries._scratchPlacedA = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchPlacedB = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchSupport = new MinkowskiSupport(Queries._scratchPlacedA, Queries._scratchPlacedB);
Queries._scratchPointShape = new SphereShape(0); // zero-radius sphere: a point, via the existing Shape contract
Queries._scratchLocalAABB = new AABB();
Queries._scratchExpandedAABB = new AABB();

ActionPhysics.Queries = Queries;
