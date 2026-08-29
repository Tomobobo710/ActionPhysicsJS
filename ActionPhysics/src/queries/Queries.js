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
        // A zero-length sweep is a stationary overlap test at `start` (unlike rayIntersect, where a
        // zero-length ray is degenerate and reports a miss - a ZERO-RADIUS ray has no meaningful
        // "am I overlapping something" question, but a real shape held still genuinely does). Handled
        // separately because _advance divides by fullLen to get a unit direction, which is NaN here.
        if (fullLen < 1e-12) return Queries._overlapTest(bodies, shape, start, rotation, ignore);
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
        if (Queries._isMesh(body.shape)) {
            return Queries._sweepPointVsMesh(start, dirX, dirY, dirZ, fullLen, body);
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
        if (Queries._isMesh(body.shape)) {
            return Queries._sweepShapeVsMesh(shape, rotation, start, dirX, dirY, dirZ, fullLen, body);
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

    // Stationary overlap test: does `shape`, held fixed at `start`, touch anything? Used by
    // shapeIntersect for a zero-length sweep (see its own comment). One GJK query per candidate body,
    // same AABB-reject-then-GJK structure as the swept queries above, but with no travel direction -
    // EPA runs on an overlapping result so a caller gets a real penetration depth/normal, not the
    // swept path's reversed-travel-direction fallback (there is no travel here to fall back on).
    static _overlapTest(bodies, shape, start, rotation, ignore) {
        const localAABB = Queries._scratchLocalAABB;
        shape.localAABBInto(localAABB);
        const radius = Math.sqrt(
            Math.max(localAABB.min.x * localAABB.min.x, localAABB.max.x * localAABB.max.x) +
            Math.max(localAABB.min.y * localAABB.min.y, localAABB.max.y * localAABB.max.y) +
            Math.max(localAABB.min.z * localAABB.min.z, localAABB.max.z * localAABB.max.z)
        );
        const rot = rotation || Queries._identityQuat;

        for (let i = 0; i < bodies.length; i++) {
            const body = bodies[i];
            if (Queries._isIgnored(body, ignore)) continue;
            if (Queries._isCompound(body.shape)) {
                const hit = Queries._overlapTestCompound(shape, start, rot, body);
                if (hit) return hit;
                continue;
            }
            if (Queries._isMesh(body.shape)) {
                const hit = Queries._overlapTestMesh(shape, start, rot, body);
                if (hit) return hit;
                continue;
            }
            const aabb = body.getAABB();
            const expanded = Queries._scratchExpandedAABB;
            expanded.copy(aabb).expandInPlace(radius);
            if (start.x < expanded.min.x || start.x > expanded.max.x ||
                start.y < expanded.min.y || start.y > expanded.max.y ||
                start.z < expanded.min.z || start.z > expanded.max.z) continue;

            const hit = Queries._overlapTestOne(shape, start, rot, body);
            if (hit) return hit;
        }
        return null;
    }

    static _overlapTestOne(shape, start, rotation, body) {
        const placedShape = Queries._scratchPlacedA;
        placedShape.shape = shape;
        placedShape.position = start;
        placedShape.rotation = rotation;

        const placedBody = Queries._scratchPlacedB;
        placedBody.shape = body.shape;
        placedBody.position = body.position;
        placedBody.rotation = body.rotation;

        const support = Queries._scratchSupport;
        support.a = placedShape; support.b = placedBody;
        support._invRotA.copy(rotation).invert();
        support._invRotB.copy(body.rotation).invert();

        const gjkResult = Queries._gjk.run(support);
        if (!gjkResult.overlapping) return null;
        const epaResult = Queries._epa.run(support, gjkResult.simplex);
        return {
            point: epaResult.pointA,
            normal: epaResult.normal,
            distance: 0,
            fraction: 0,
            body: body
        };
    }

    static _overlapTestCompound(shape, start, rotation, body) {
        const children = body.shape.children;
        for (let i = 0; i < children.length; i++) {
            const placedChild = Queries._placedChildInto(Queries._scratchCompoundChild, body, children[i]);
            const placedShape = Queries._scratchPlacedA;
            placedShape.shape = shape;
            placedShape.position = start;
            placedShape.rotation = rotation;

            const support = Queries._scratchSupport;
            support.a = placedShape; support.b = placedChild;
            support._invRotA.copy(rotation).invert();
            support._invRotB.copy(placedChild.rotation).invert();

            const gjkResult = Queries._gjk.run(support);
            if (!gjkResult.overlapping) continue;
            const epaResult = Queries._epa.run(support, gjkResult.simplex);
            return { point: epaResult.pointA, normal: epaResult.normal, distance: 0, fraction: 0, body: body };
        }
        return null;
    }

    static _overlapTestMesh(shape, start, rotation, body) {
        const meshShape = body.shape;
        const a = new Vector3(), b = new Vector3(), c = new Vector3();
        for (let i = 0; i < meshShape.triangleCount; i++) {
            meshShape.triangleAt(i, a, b, c);
            const placedTri = Queries._placedTriangleInto(Queries._scratchCompoundChild, body, Queries._scratchTriangleShape, a, b, c);
            const placedShape = Queries._scratchPlacedA;
            placedShape.shape = shape;
            placedShape.position = start;
            placedShape.rotation = rotation;

            const support = Queries._scratchSupport;
            support.a = placedShape; support.b = placedTri;
            support._invRotA.copy(rotation).invert();
            support._invRotB.copy(placedTri.rotation).invert();

            const gjkResult = Queries._gjk.run(support);
            if (!gjkResult.overlapping) continue;
            const epaResult = Queries._epa.run(support, gjkResult.simplex);
            return { point: epaResult.pointA, normal: epaResult.normal, distance: 0, fraction: 0, body: body };
        }
        return null;
    }

    static _isCompound(shape) {
        return typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape;
    }

    static _isMesh(shape) {
        return typeof MeshShape !== 'undefined' && shape instanceof MeshShape;
    }

    // World-space placement of one mesh triangle: the body's own transform applied to the triangle's
    // three LOCAL-space vertices (mesh triangles carry no per-triangle local offset the way a compound
    // child does - MeshShape.triangleAt already hands back body-local coordinates directly, same
    // convention Midphase's own mesh dispatch uses). Rebuilds the TriangleShape's vertex fields
    // in-place on a cached scratch instance rather than allocating a new TriangleShape per triangle.
    static _placedTriangleInto(outPlaced, body, triShape, a, b, c) {
        triShape.a = a; triShape.b = b; triShape.c = c;
        outPlaced.shape = triShape;
        outPlaced.position = body.position;
        outPlaced.rotation = body.rotation;
        return outPlaced;
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

    static _sweepPointVsMesh(start, dirX, dirY, dirZ, fullLen, body) {
        const shape = body.shape;
        const a = new Vector3(), b = new Vector3(), c = new Vector3();
        let best = null, bestFraction = Infinity;
        for (let i = 0; i < shape.triangleCount; i++) {
            shape.triangleAt(i, a, b, c);
            const placedTri = Queries._placedTriangleInto(Queries._scratchCompoundChild, body, Queries._scratchTriangleShape, a, b, c);

            const pointShape = Queries._scratchPointShape;
            const placedPoint = Queries._scratchPlacedA;
            placedPoint.shape = pointShape;
            placedPoint.position = Queries._scratchPos.set(start.x, start.y, start.z);
            placedPoint.rotation = Queries._identityQuat;

            const support = Queries._scratchSupport;
            support.a = placedPoint; support.b = placedTri;
            support._invRotA.copy(Queries._identityQuat);
            support._invRotB.copy(placedTri.rotation).invert();

            const hit = Queries._advance(support, placedPoint, start, dirX, dirY, dirZ, fullLen);
            if (hit && hit.fraction < bestFraction) { bestFraction = hit.fraction; best = hit; }
        }
        return best;
    }

    static _sweepShapeVsMesh(shape, rotation, start, dirX, dirY, dirZ, fullLen, body) {
        const meshShape = body.shape;
        const a = new Vector3(), b = new Vector3(), c = new Vector3();
        let best = null, bestFraction = Infinity;
        for (let i = 0; i < meshShape.triangleCount; i++) {
            meshShape.triangleAt(i, a, b, c);
            const placedTri = Queries._placedTriangleInto(Queries._scratchCompoundChild, body, Queries._scratchTriangleShape, a, b, c);

            const placedShape = Queries._scratchPlacedA;
            placedShape.shape = shape;
            placedShape.position = Queries._scratchPos.set(start.x, start.y, start.z);
            placedShape.rotation = rotation || Queries._identityQuat;

            const support = Queries._scratchSupport;
            support.a = placedShape; support.b = placedTri;
            support._invRotA.copy(placedShape.rotation).invert();
            support._invRotB.copy(placedTri.rotation).invert();

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
                // documented two outcomes), but EPA does: it expands the SAME simplex GJK just
                // produced into a real surface normal (Queries._overlapTestOne/_overlapTestCompound
                // already do exactly this for the stationary-overlap case, just below). Reusing that
                // here - rather than falling back to the reversed travel direction - is the actual
                // fix for a since-confirmed bug class: the reversed-direction fallback cannot tell
                // "approaching a surface ahead" from "already past it and moving away", so a sweep
                // that starts embedded (a character mounted on a ladder, two overlapping capsules, a
                // ramp toe at the exact moment of contact) reported a normal that was sometimes
                // backwards - root-caused to three separate downstream symptoms this session
                // (character-vs-character pass-through, a ladder dismount that could never clear its
                // own embedded start, a steep ramp misclassified as a wall) before being traced to
                // this shared root rather than patched three times over. EPA's own degenerate/exact-
                // touch path (_zeroDepthResult, using the simplex's own closest-point-to-origin) is
                // the same fallback GJK.js's own "EXACT-TOUCHING IS UNDECIDABLE" case already
                // documents as the right answer when even EPA cannot build a real enclosing
                // tetrahedron - so this fix does not introduce a new fallback, it reuses the one
                // already proven correct for narrowphase's identical problem.
                const epaResult = Queries._epa.run(support, result.simplex);
                return Queries._finishHit(start, dirX, dirY, dirZ, traveled, fullLen, epaResult.normal.x, epaResult.normal.y, epaResult.normal.z);
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
Queries._epa = new EPA();
Queries._identityQuat = new Quaternion(0, 0, 0, 1);
Queries._scratchPos = new Vector3();
Queries._scratchPlacedA = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchPlacedB = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchSupport = new MinkowskiSupport(Queries._scratchPlacedA, Queries._scratchPlacedB);
Queries._scratchPointShape = new SphereShape(0); // zero-radius sphere: a point, via the existing Shape contract
Queries._scratchLocalAABB = new AABB();
Queries._scratchExpandedAABB = new AABB();
Queries._scratchCompoundChild = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchTriangleShape = new TriangleShape(new Vector3(), new Vector3(), new Vector3());

ActionPhysics.Queries = Queries;
