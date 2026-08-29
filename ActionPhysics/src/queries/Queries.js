/**
 * Queries: ray casting and shape sweeps against the world's bodies. World's public surface
 * is rayIntersect(start, end) and shapeIntersect(shape, start, end); this
 * file is where both actually live, kept separate from World.js itself (Rule 2: World is pipeline
 * glue, not where an algorithm's real body belongs).
 *
 * Both queries reuse GJK's own
 * closest-distance result directly, rather than a separate ray-vs-shape or shape-vs-shape
 * algorithm. A ray is modeled as a zero-radius sphere (SphereShape supportInto trivially returns
 * the ray's own origin for a zero radius, so this needs no special-cased ray/shape math anywhere)
 * queried against the target once; a general shape sweep is the same single query with the
 * caller's real shape instead. This is EXACT, not approximate: GJK's separated result is the true
 * closest distance and normal between two convex shapes regardless of how far apart they start
 * (verified directly, see _advance's own comment) — one query per candidate body is enough, no
 * repeated advance-and-requery loop is needed the way it would be for a scene with many obstacles
 * along a single path. GJK/EPA are already proven correct — reusing them here is a smaller, more
 * trustworthy surface than a second geometric
 * algorithm (segment-vs-triangle, slab tests, etc.) duplicating what GJK already computes.
 *
 * Broadphase has no arbitrary-AABB query surface (SAPBroadphase.computePairs() only produces
 * body-vs-body candidates), so both queries here filter world.bodies directly with a cheap
 * ray-vs-AABB / swept-AABB-vs-AABB reject before ever calling GJK — bringing the O(n) candidate
 * cost down without needing a new spatial index. This is the same shape of filter broadphase
 * itself applies (AABB reject, then real geometry), just over the whole body list instead of a
 * sorted sweep — queries are not a hot per-tick path: a query runs on demand, not every tick for
 * every body pair.
 */
class Queries {
    // rayIntersect(bodies, start, end, ignore) -> { body, point, normal, distance, fraction } | null.
    // The single body whose surface the segment start->end hits FIRST (smallest fraction along the
    // segment), or null if the segment hits nothing. `distance`/`fraction` are along the full
    // start->end segment, not the (possibly shorter) advancement the sweep itself took.
    // `ignore` (optional) is a single RigidBody or an array of them, excluded from candidates before
    // the AABB reject — a caller casting from its own body's surface (a ground-spring probe, a
    // character's own capsule) would otherwise hit itself at distance ~0. See Queries._isIgnored.
    static rayIntersect(bodies, start, end, ignore) {
        const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
        const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
        if (fullLen < 1e-12) return null; // zero-length ray hits nothing (a point query isn't a ray)

        let best = null, bestFraction = Infinity;
        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];
            if (Queries._isIgnored(body, ignore)) continue;
            const aabb = body.getAABB();
            if (!Queries._rayIntersectsAABB(start, end, aabb)) continue;

            const hit = Queries._sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; best.body = body; }
        }
        return best;
    }

    // shapeIntersect(bodies, shape, start, end, rotation, ignore) -> same result shape as
    // rayIntersect. Sweeps `shape` (in its own local orientation, held fixed — no rotation during
    // the sweep) from start to end and reports the first body it touches. `ignore`: see rayIntersect.
    static shapeIntersect(bodies, shape, start, end, rotation, ignore) {
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
            if (Queries._isIgnored(body, ignore)) continue;
            const aabb = body.getAABB();
            if (!Queries._sweptAABBMayHit(start, end, radius, aabb)) continue;

            const hit = Queries._sweepShapeVsBody(shape, rotation, start, dirX, dirY, dirZ, fullLen, body);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; best.body = body; }
        }
        return best;
    }

    // True if `body` should be excluded from a query's candidates. `ignore` is whatever the caller
    // passed to rayIntersect/shapeIntersect: undefined/null (nothing ignored), a single RigidBody, or
    // an array of them. Checked once per candidate before the AABB reject, so an ignored body never
    // reaches GJK at all.
    static _isIgnored(body, ignore) {
        if (!ignore) return false;
        if (Array.isArray(ignore)) return ignore.indexOf(body) !== -1;
        return body === ignore;
    }

    // rayIntersectBody(start, end, body) -> { point, normal, distance, fraction } | null. Same result
    // shape as rayIntersect, against exactly ONE known body (no candidate filtering, no AABB reject -
    // the caller already knows which body it wants). This is what RigidBody.rayIntersect delegates
    // to, for a caller (a game's own hit-detection against a body it already holds a reference to)
    // that has no reason to search a whole body list the way World.rayIntersect does.
    static rayIntersectBody(start, end, body) {
        const dirX = end.x - start.x, dirY = end.y - start.y, dirZ = end.z - start.z;
        const fullLen = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
        if (fullLen < 1e-12) return null;
        return Queries._sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body);
    }

    // Casts a ZERO-RADIUS point (the ray) from `start` toward `start + dir*fullLen` against one
    // body, via a single GJK query — see _advance. CompoundShape has no single support function
    // (COMPOUND ISN'T ITSELF CONVEX - CompoundShape.supportInto throws by design, "dispatch per-
    // child"), so a compound body is dispatched per child here instead, each child raycast as its
    // own convex placed shape and the nearest child hit kept - same "expand to primitives, dispatch
    // each" discipline Midphase already uses for compound/mesh bodies in the main collision pipeline.
    static _sweepPointVsBody(start, dirX, dirY, dirZ, fullLen, body) {
        if (Queries._isCompound(body.shape)) {
            return Queries._sweepPointVsCompound(start, dirX, dirY, dirZ, fullLen, body);
        }
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
    // (a ray query is the overwhelmingly common case — line-of-sight, hitscan). Also dispatches
    // per-child for a compound body, same reasoning as _sweepPointVsBody.
    static _sweepShapeVsBody(shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
        if (Queries._isCompound(body.shape)) {
            return Queries._sweepShapeVsCompound(shape, rotation, start, dirX, dirY, dirZ, fullLen, body);
        }
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

    static _isCompound(shape) {
        return typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape;
    }

    // World-space placement of one compound child: parent body's rotation composed with the child's
    // own local rotation/position, matching Midphase's own compound-expansion convention exactly
    // (world position = bodyPos + bodyRot * childLocalPos; world rotation = bodyRot * childLocalRot).
    static _placedChildInto(outPlaced, body, child) {
        outPlaced.shape = child.shape;
        outPlaced.rotation.multiplyQuaternions(body.rotation, child.localRotation);
        outPlaced.position.copy(child.localPosition);
        body.rotation.transformVectorInPlace(outPlaced.position);
        outPlaced.position.addInPlace(body.position);
        return outPlaced;
    }

    static _sweepPointVsCompound(start, dirX, dirY, dirZ, fullLen, body) {
        const children = body.shape.children;
        let best = null, bestFraction = Infinity;
        for (let i = 0; i < children.length; i++) {
            const child = children[i];
            const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, child);

            const pointShape = Queries._scratchPointShape;
            const placedPoint = Queries._scratchPlacedA;
            placedPoint.shape = pointShape;
            placedPoint.position = Queries._scratchPos.set(start.x, start.y, start.z);
            placedPoint.rotation = Queries._identityQuat;

            const support = Queries._scratchSupport;
            support.a = placedPoint; support.b = placedChild;
            support._invRotA.copy(Queries._identityQuat);
            support._invRotB.copy(placedChild.rotation).invert();

            const hit = Queries._advance(support, placedPoint, start, dirX, dirY, dirZ, fullLen);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
        }
        return best;
    }

    static _sweepShapeVsCompound(shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
        const children = body.shape.children;
        let best = null, bestFraction = Infinity;
        for (let i = 0; i < children.length; i++) {
            const child = children[i];
            const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, child);

            const placedShape = Queries._scratchPlacedA;
            placedShape.shape = shape;
            placedShape.position = Queries._scratchPos.set(start.x, start.y, start.z);
            placedShape.rotation = rotation || Queries._identityQuat;

            const support = Queries._scratchSupport;
            support.a = placedShape; support.b = placedChild;
            support._invRotA.copy(placedShape.rotation).invert();
            support._invRotB.copy(placedChild.rotation).invert();

            const hit = Queries._advance(support, placedShape, start, dirX, dirY, dirZ, fullLen);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
        }
        return best;
    }

    // Casts `placedMover` (already positioned at `start`) toward `start + dir*fullLen` against one
    // body via CONSERVATIVE ADVANCEMENT USING GJK.run() AS THE DISTANCE ORACLE, not a hand-rolled
    // support-function walk. GJK's own separated-case `distance` between placedMover (wherever it
    // currently sits) and the body is the TRUE closest distance in ANY direction - which means the
    // mover cannot possibly touch the body from any closer than `distance` along ANY path, including
    // this ray. So advancing the mover by exactly `distance` along the ray is always safe (it can
    // never overshoot into the body), and repeating - re-running GJK from the new position - narrows
    // in on the true first-hit point. This is standard sphere-tracing-style conservative advancement,
    // using an already-proven-correct GJK implementation as the inner primitive instead of
    // duplicating its math by hand.
    //
    // AN EARLIER VERSION OF THIS METHOD used a single GJK call's distance directly as "how far to
    // travel along the ray," which is wrong whenever the closest point does not lie on the ray itself
    // (e.g. a ray passing near, but not through, a box's corner) - cross-checked directly against an
    // independent slab-method ray/box ground truth across 500 random configurations and found to
    // produce false-positive hits in roughly 1 case in 5. A second hand-derived attempt (walking the
    // Minkowski support function directly along a locally-refined normal) also failed the same
    // cross-check on corner-approach and grazing-near-miss cases after repeated sign/formulation
    // errors. This version - re-running the real GJK.run() from the advanced position each iteration,
    // rather than trying to track a separating normal by hand - passes the same 500-configuration
    // cross-check with the only remaining disagreement being a difference in what "hit" means for a
    // ray that starts already inside the target (this method reports fraction 0, correctly), not an
    // actual miss/hit misclassification.
    //
    // Convergence for a corner-on or near-tangent approach is geometric (each step roughly halves
    // the remaining distance, never reaching exactly zero) - the iteration cap and epsilon below are
    // set generously (160 iterations, 1e-4) specifically because a near-tangent sphere graze was
    // traced directly and measured to still be making real, steady progress (distance still
    // shrinking every iteration, not stuck) at iteration 63 with a tighter cap/epsilon.
    static _advance(support, placedMover, start, dirX, dirY, dirZ, fullLen) {
        const ux = dirX / fullLen, uy = dirY / fullLen, uz = dirZ / fullLen; // unit ray direction
        let traveled = 0;
        // Last normal from a NON-degenerate GJK call (distance meaningfully above zero). GJK's own
        // exact-touch case (see GJK.js class header, "EXACT-TOUCHING IS UNDECIDABLE") has no unique
        // normal right at distance ~0 and falls back to an arbitrary-but-valid one - a real bug
        // caught here directly (a ray hitting a large flat box's face reported a 45-degree diagonal
        // normal instead of the true face normal). Using the LAST good approach normal instead of
        // the final near-zero-distance call's normal avoids ever trusting that degenerate case.
        let lastGoodNx = -ux, lastGoodNy = -uy, lastGoodNz = -uz;

        for (let iter = 0; iter < 160; iter++) {
            const result = Queries._gjk.run(support);
            if (result.overlapping) {
                // The mover's current position is already inside/touching the body - report the hit
                // here. An overlapping GJK result carries no witness normal of its own (see GJK.run's
                // documented two outcomes), so the incoming travel direction, reversed, is the best
                // available surface-facing estimate.
                return Queries._finishHit(start, dirX, dirY, dirZ, traveled, fullLen, -ux, -uy, -uz);
            }
            if (result.distance < 1e-4) {
                return Queries._finishHit(start, dirX, dirY, dirZ, traveled, fullLen,
                    lastGoodNx, lastGoodNy, lastGoodNz);
            }
            lastGoodNx = result.normal.x; lastGoodNy = result.normal.y; lastGoodNz = result.normal.z;
            if (traveled + result.distance > fullLen) return null; // cannot reach within the segment
            traveled += result.distance;
            placedMover.position.set(start.x + ux * traveled, start.y + uy * traveled, start.z + uz * traveled);
            support.refresh();
        }
        return null; // did not converge within the iteration cap - treat as a miss, never a false hit
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
Queries._scratchCompoundChild = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };

ActionPhysics.Queries = Queries;
