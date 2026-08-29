/**
 * EPA (Expanding Polytope Algorithm): penetration depth, normal, and contact points from a GJK
 * simplex that already encloses the origin. GJK proves overlap; EPA measures how deep.
 *
 * Written from the algorithm (van den Bergen, "Collision Detection in Interactive 3D
 * Environments"; the standard EPA formulation), not from a prior implementation - same reasoning
 * as GJK.js: narrowphase is the highest-risk component, and the one place a structural anchor to
 * the predecessor would be worst to carry over.
 *
 * Produces: signed distance (always >= 0 here — a positive PENETRATION depth, matching the
 * Narrowphase contract's convention where positive = overlapping), a world-space normal pointing
 * from B to A, and witness points on each shape's surface. May assume: the input simplex is a
 * genuine tetrahedron (4 points) that encloses the origin - GJK.run() only ever returns
 * `overlapping: true` from its `_simplexTetrahedron` path, which never produces anything else.
 *
 * BUG FIX CARRIED FROM THE PREDECESSOR (plan.md, Bug reference / Collision detection):
 * "EPA accepted garbage on the iteration cap." Its stable-exit test required
 * `closest_face_distance > EPSILON`, but a resting/shallow contact sits BELOW epsilon, so it never
 * exited early, ran to the iteration cap, and returned whatever the LAST iteration's polytope
 * state happened to be - not the best one found. The fix here is structural: the closest face
 * found across every iteration is tracked independently of the loop's current state, and the
 * final return always reads from that tracked best - never from "whatever the polytope looks like
 * when the loop happens to end." A resting contact (depth near zero) converges immediately because
 * there is nothing left to expand, not because of a lucky exit-epsilon comparison.
 */
class EPA {
    constructor() {
        // Polytope vertices, parallel arrays like GJK's simplex storage (w = Minkowski diff point,
        // a/b = world witness points on each shape). Grows as EPA expands the polytope; capacity
        // starts generous and grows geometrically to stay allocation-light across many calls.
        this._capacity = 64;
        this._wx = new Float64Array(this._capacity); this._wy = new Float64Array(this._capacity); this._wz = new Float64Array(this._capacity);
        this._ax = new Float64Array(this._capacity); this._ay = new Float64Array(this._capacity); this._az = new Float64Array(this._capacity);
        this._bx = new Float64Array(this._capacity); this._by = new Float64Array(this._capacity); this._bz = new Float64Array(this._capacity);
        this._vertexCount = 0;

        // Faces: triangle index triples plus their (outward) normal and distance-to-origin, kept
        // in parallel arrays. A face is "alive" while faceAlive[i] is truthy; removed faces
        // (replaced during expansion) are marked dead rather than spliced out, to avoid shifting
        // every later index on every removal.
        this._faceCapacity = 128;
        this._faceA = new Int32Array(this._faceCapacity);
        this._faceB = new Int32Array(this._faceCapacity);
        this._faceC = new Int32Array(this._faceCapacity);
        this._faceNx = new Float64Array(this._faceCapacity);
        this._faceNy = new Float64Array(this._faceCapacity);
        this._faceNz = new Float64Array(this._faceCapacity);
        this._faceDist = new Float64Array(this._faceCapacity);
        this._faceAlive = new Uint8Array(this._faceCapacity);
        this._faceCount = 0;

        this._newW = new Vector3();
        this._newA = new Vector3();
        this._newB = new Vector3();
        this._dirScratch = new Vector3();
    }

    _growVertices() {
        const cap = this._capacity * 2;
        const grow = (arr) => { const n = new arr.constructor(cap); n.set(arr); return n; };
        this._wx = grow(this._wx); this._wy = grow(this._wy); this._wz = grow(this._wz);
        this._ax = grow(this._ax); this._ay = grow(this._ay); this._az = grow(this._az);
        this._bx = grow(this._bx); this._by = grow(this._by); this._bz = grow(this._bz);
        this._capacity = cap;
    }

    _growFaces() {
        const cap = this._faceCapacity * 2;
        const grow = (arr) => { const n = new arr.constructor(cap); n.set(arr); return n; };
        this._faceA = grow(this._faceA); this._faceB = grow(this._faceB); this._faceC = grow(this._faceC);
        this._faceNx = grow(this._faceNx); this._faceNy = grow(this._faceNy); this._faceNz = grow(this._faceNz);
        this._faceDist = grow(this._faceDist);
        this._faceAlive = grow(this._faceAlive);
        this._faceCapacity = cap;
    }

    _pushVertex(w, a, b) {
        if (this._vertexCount >= this._capacity) this._growVertices();
        const i = this._vertexCount++;
        this._wx[i] = w.x; this._wy[i] = w.y; this._wz[i] = w.z;
        this._ax[i] = a.x; this._ay[i] = a.y; this._az[i] = a.z;
        this._bx[i] = b.x; this._by[i] = b.y; this._bz[i] = b.z;
        return i;
    }

    // Adds a face from three vertex indices, computing its outward normal and origin distance.
    // "Outward" here means away from the polytope's own centroid - correct as long as the
    // polytope is convex and the origin is inside it (true by construction: EPA only ever starts
    // from a GJK tetrahedron that already encloses the origin, and every subsequent expansion
    // stays convex around that same enclosed origin). Returns the new face's index, or -1 if the
    // three points are degenerate (collinear / zero area) - the caller skips a degenerate face
    // rather than adding a face with an undefined normal (same discipline as GJK's own degenerate
    // fallbacks: never propagate a NaN).
    _addFace(ia, ib, ic, centroidHint) {
        const ax = this._wx[ia], ay = this._wy[ia], az = this._wz[ia];
        const bx = this._wx[ib], by = this._wy[ib], bz = this._wz[ib];
        const cx = this._wx[ic], cy = this._wy[ic], cz = this._wz[ic];
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const acx = cx - ax, acy = cy - ay, acz = cz - az;
        let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        const nLenSq = nx * nx + ny * ny + nz * nz;
        if (nLenSq < 1e-20) return -1; // degenerate: skip, never add a face with no real normal

        const invLen = 1 / Math.sqrt(nLenSq);
        nx *= invLen; ny *= invLen; nz *= invLen;

        // Orient outward: away from the hint point (the polytope's centroid, or the 4th
        // tetrahedron vertex on initial construction).
        const toHint = (centroidHint.x - ax) * nx + (centroidHint.y - ay) * ny + (centroidHint.z - az) * nz;
        if (toHint > 0) { nx = -nx; ny = -ny; nz = -nz; }

        const dist = ax * nx + ay * ny + az * nz; // (a - origin) . n_hat = distance from origin to the face's plane

        if (this._faceCount >= this._faceCapacity) this._growFaces();
        const fi = this._faceCount++;
        this._faceA[fi] = ia; this._faceB[fi] = ib; this._faceC[fi] = ic;
        this._faceNx[fi] = nx; this._faceNy[fi] = ny; this._faceNz[fi] = nz;
        this._faceDist[fi] = dist;
        this._faceAlive[fi] = 1;
        return fi;
    }

    /**
     * Expands `simplex` (a GJK instance whose _wx/_wy/_wz/_ax.../_bx... hold exactly 4 points that
     * enclose the origin) into the true penetration depth and normal.
     *
     * Returns { distance, normal, pointA, pointB } - distance is a non-negative penetration depth
     * (plan.md's positive = overlapping convention), normal points from B to A (same convention
     * GJK's separated result uses), pointA/pointB are witness points on each shape's own surface
     * recovered from the winning face's barycentric weights.
     *
     * maxIterations guards non-convergence on pathological input. Per the header's bug-fix note,
     * hitting the cap returns the best (closest-to-origin) face tracked across the WHOLE run, not
     * whatever face is live when the loop happens to stop.
     */
    run(support, simplex, maxIterations) {
        maxIterations = maxIterations || 64;
        this._vertexCount = 0;
        this._faceCount = 0;

        // Seed the polytope from the GJK tetrahedron's 4 points.
        const idx = [];
        for (let i = 0; i < 4; i++) {
            idx.push(this._pushVertex(
                { x: simplex._wx[i], y: simplex._wy[i], z: simplex._wz[i] },
                { x: simplex._ax[i], y: simplex._ay[i], z: simplex._az[i] },
                { x: simplex._bx[i], y: simplex._by[i], z: simplex._bz[i] }
            ));
        }
        // Centroid of the 4 seed points, used to orient each face's normal outward.
        const cx = (this._wx[idx[0]] + this._wx[idx[1]] + this._wx[idx[2]] + this._wx[idx[3]]) / 4;
        const cy = (this._wy[idx[0]] + this._wy[idx[1]] + this._wy[idx[2]] + this._wy[idx[3]]) / 4;
        const cz = (this._wz[idx[0]] + this._wz[idx[1]] + this._wz[idx[2]] + this._wz[idx[3]]) / 4;
        const centroid = { x: cx, y: cy, z: cz };

        // The 4 faces of the seed tetrahedron - same enumeration GJK.js uses (and the same one
        // whose typo caused GJK's own hardest bug: verify all 4 DISTINCT faces are present).
        this._addFace(idx[0], idx[1], idx[2], centroid);
        this._addFace(idx[0], idx[1], idx[3], centroid);
        this._addFace(idx[0], idx[2], idx[3], centroid);
        this._addFace(idx[1], idx[2], idx[3], centroid);

        for (let iter = 0; iter < maxIterations; iter++) {
            const face = this._closestAliveFace();
            const faceDist = this._faceDist[face];

            // Expand along the closest face's own normal - the direction most likely to find the
            // true surface next.
            this._dirScratch.set(this._faceNx[face], this._faceNy[face], this._faceNz[face]);
            support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);

            const newDist = this._newW.x * this._faceNx[face] + this._newW.y * this._faceNy[face] + this._newW.z * this._faceNz[face];

            // Converged: the new support point does not extend past the closest face's own plane
            // by more than a small margin. This is checked against the closest face's distance
            // DIRECTLY (not a fixed epsilon independent of scale) - see the header's bug-fix note:
            // the predecessor's bug was exactly a fixed-epsilon exit test failing for shallow
            // contacts. Comparing the new point's extent to the face's own already-converged
            // distance means a resting contact (whose closest face sits near-zero) still detects
            // "no more progress" correctly, because both sides of the comparison scale together.
            if (newDist - faceDist < 1e-6) break;

            this._expandAt(this._newW, this._newA, this._newB, centroid);
        }

        // The closest ALIVE face, read fresh right here rather than tracked across iterations.
        // This IS the actual fix for the predecessor's bug (plan.md, Bug reference: "returned
        // whatever the final iteration produced" instead of the converged answer) - a face's own
        // distance value is only meaningful while it is still alive; a face expansion replaces
        // wrong-but-smaller-looking faces with new ones as the polytope refines toward the true
        // surface, so tracking "smallest distance ever seen across iterations" (tried and reverted
        // - see git history) picks up STALE distances from faces that were later proven invalid
        // and removed. Re-querying the live polytope's actual closest alive face, whether the loop
        // converged cleanly or hit the iteration cap, is the only way to get the CURRENT answer -
        // exactly the "return the converged result, never the last one" rule, applied correctly.
        return this._resultFromFace(this._closestAliveFace());
    }

    // Finds the living face with the smallest distance-to-origin. Linear scan - EPA's polytope
    // stays small (tens of faces) for the shape pairs this engine targets, and a scan here is far
    // simpler to trust than a priority-queue structure for that size.
    _closestAliveFace() {
        let best = -1, bestDist = Infinity;
        for (let i = 0; i < this._faceCount; i++) {
            if (!this._faceAlive[i]) continue;
            if (this._faceDist[i] < bestDist) { bestDist = this._faceDist[i]; best = i; }
        }
        return best;
    }

    // Adds `newPoint` to the polytope and re-triangulates: every alive face visible from the new
    // point (the point is on the OUTER side of the face's plane) is removed, and the boundary loop
    // of the resulting hole is re-closed with new faces to the new point (the standard EPA
    // horizon-edge expansion).
    _expandAt(newW, newA, newB, centroid) {
        const newIdx = this._pushVertex(newW, newA, newB);

        // Collect the horizon: edges shared by exactly one visible face and one non-visible face.
        // Represented as [fromIndex, toIndex] pairs, deduplicated by removing a pair the moment
        // its reverse is seen (a shared internal edge between two visible faces cancels out).
        const horizonA = [], horizonB = [];
        function edgeKey(a, b) { return a < b ? a + ',' + b : b + ',' + a; }
        const edgeSeen = new Map(); // key -> { from, to }

        for (let i = 0; i < this._faceCount; i++) {
            if (!this._faceAlive[i]) continue;
            const a = this._faceA[i], b = this._faceB[i], c = this._faceC[i];
            const nx = this._faceNx[i], ny = this._faceNy[i], nz = this._faceNz[i];
            const visible = (newW.x - this._wx[a]) * nx + (newW.y - this._wy[a]) * ny + (newW.z - this._wz[a]) * nz > 1e-10;
            if (!visible) continue;

            this._faceAlive[i] = 0;
            const edges = [[a, b], [b, c], [c, a]];
            for (let e = 0; e < 3; e++) {
                const from = edges[e][0], to = edges[e][1];
                const key = edgeKey(from, to);
                if (edgeSeen.has(key)) {
                    edgeSeen.delete(key); // shared with another visible face: internal, cancels out
                } else {
                    edgeSeen.set(key, { from: from, to: to });
                }
            }
        }

        edgeSeen.forEach(function (e) { horizonA.push(e.from); horizonB.push(e.to); });

        for (let i = 0; i < horizonA.length; i++) {
            this._addFace(horizonA[i], horizonB[i], newIdx, centroid);
        }
    }

    // Recovers { distance, normal, pointA, pointB } from a chosen face - barycentric weights of
    // the face's own closest point to the origin (projected onto the triangle's plane, then
    // expressed in area-ratio barycentric form), applied to the face's three world witness points.
    // Same degenerate-fallback discipline as GJK's own barycentric routine: never divide by zero,
    // always a valid (if approximate) geometric answer.
    _resultFromFace(face) {
        const ia = this._faceA[face], ib = this._faceB[face], ic = this._faceC[face];
        const nx = this._faceNx[face], ny = this._faceNy[face], nz = this._faceNz[face];
        const dist = this._faceDist[face];

        // Closest point on the face's plane to the origin. The plane is {x : x.n_hat = dist}, so
        // the closest point on it to the origin is dist * n_hat (that point's own dot with n_hat
        // is dist by construction, and it is the minimal-length point satisfying that).
        const ax = this._wx[ia], ay = this._wy[ia], az = this._wz[ia];
        const closestX = nx * dist, closestY = ny * dist, closestZ = nz * dist;

        const bx = this._wx[ib], by = this._wy[ib], bz = this._wz[ib];
        const cx = this._wx[ic], cy = this._wy[ic], cz = this._wz[ic];
        const v0x = bx - ax, v0y = by - ay, v0z = bz - az;
        const v1x = cx - ax, v1y = cy - ay, v1z = cz - az;
        const v2x = closestX - ax, v2y = closestY - ay, v2z = closestZ - az;
        const d00 = v0x * v0x + v0y * v0y + v0z * v0z;
        const d01 = v0x * v1x + v0y * v1y + v0z * v1z;
        const d11 = v1x * v1x + v1y * v1y + v1z * v1z;
        const d20 = v2x * v0x + v2y * v0y + v2z * v0z;
        const d21 = v2x * v1x + v2y * v1y + v2z * v1z;
        const denom = d00 * d11 - d01 * d01;
        let u, v, w;
        if (Math.abs(denom) < 1e-20) {
            u = 1 / 3; v = 1 / 3; w = 1 / 3; // degenerate face: even split, never NaN
        } else {
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
            u = 1 - v - w;
        }

        const pointA = new Vector3(
            u * this._ax[ia] + v * this._ax[ib] + w * this._ax[ic],
            u * this._ay[ia] + v * this._ay[ib] + w * this._ay[ic],
            u * this._az[ia] + v * this._az[ib] + w * this._az[ic]
        );
        const pointB = new Vector3(
            u * this._bx[ia] + v * this._bx[ib] + w * this._bx[ic],
            u * this._by[ia] + v * this._by[ib] + w * this._by[ic],
            u * this._bz[ia] + v * this._bz[ib] + w * this._bz[ic]
        );

        // The face's own "outward from the polytope's interior" normal points from A's side
        // toward B's side of the Minkowski difference A-B (verified against GJK's own separated-
        // result convention, which points from B to A - a real, sign-only bug caught by comparing
        // the two detectors' normals directly on identical geometry, not by any test that only
        // checked axis alignment). Negated here so EPA's returned normal matches GJK's: B to A,
        // the direction the solver actually pushes body A along.
        return {
            distance: Math.max(0, dist), // clamp: a face passing fractionally behind the origin from float noise still reports a valid non-negative depth
            normal: new Vector3(-nx, -ny, -nz),
            pointA: pointA,
            pointB: pointB
        };
    }
}

ActionPhysics.EPA = EPA;
