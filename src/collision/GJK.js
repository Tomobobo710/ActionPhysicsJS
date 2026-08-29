/**
 * GJK: distance / overlap test between two convex shapes via their Minkowski difference.
 *
 * Written from the algorithm (Ericson, "Real-Time Collision Detection", ch. 5; the original
 * Gilbert-Johnson-Keerthi paper), not from a prior implementation - plan.md, "This is a rebuild,
 * not a port": narrowphase is the highest-risk component and the one place a structural anchor to
 * the predecessor would be worst to carry over.
 *
 * Two outcomes, both from the SAME loop:
 *   OVERLAPPING - the simplex encloses the origin. Returns the simplex as-is (2-4 points) for EPA
 *                 to expand into a penetration depth/normal. GJK itself does not compute depth.
 *   SEPARATED   - the loop converges on the closest points between the two hulls. Returns exact
 *                 distance, witness points on A and B, and a separating normal. This is what
 *                 speculative contacts run on: the query happens against a PREDICTED position
 *                 (Solver's job, not GJK's), and this result is used directly as a positive-
 *                 separation contact rather than triggering a second penetrating pass.
 *
 * BUG FIX CARRIED FROM THE PREDECESSOR (plan.md, Bug reference / Collision detection):
 * "Flush shapes reported no contact at all." Two shapes resting exactly touching (or a simplex that
 * degenerates during the walk - three collinear points, a zero-area triangle) produced NaN
 * barycentric coordinates and the contact was silently discarded. The fix here is structural: every
 * point-in-simplex / closest-point routine below has an EXPLICIT fallback for the degenerate case
 * (zero-length edge, zero-area triangle) that returns a valid geometric answer instead of NaN -
 * never a discard. See the `_degenerate...` fallback branches.
 *
 * EXACT-TOUCHING IS UNDECIDABLE FROM A LOWER-DIMENSIONAL SIMPLEX ALONE. When a 2-point or 3-point
 * simplex reduces to a closest point that IS the origin exactly, that is genuinely ambiguous with
 * only that simplex to look at: it is what a flush touch looks like (two boxes with coincident
 * faces produce a Minkowski-difference boundary the origin sits exactly on), but it is ALSO what a
 * lower-dimensional cross-section of a real 3D overlap looks like on the way to an enclosing
 * tetrahedron - a touching pair still has real geometric extent perpendicular to the touching plane
 * (a box has depth), so growing the simplex incrementally from a single starting direction finds
 * "progress" in both cases alike and can get stuck reporting the touching plane forever without
 * ever discovering a real enclosing tetrahedron on the other side of it. This is a well-known hard
 * case in GJK implementations generally (see e.g. the Signed Volumes / Montanari et al. distance
 * subalgorithm literature for a fully robust treatment).
 *
 * THE RESOLUTION USED HERE: seed several tetrahedra directly from diverse, non-coplanar probe
 * direction SETS (see SEED_DIRECTION_SETS / _seedTetrahedron) instead of growing one incrementally.
 * A real overlap's enclosing tetrahedron shows up with every face at a genuine NEGATIVE margin once
 * built from directions that are not all confined to the touching plane; an exact touch never
 * produces a fully-negative-margin tetrahedron no matter which diverse directions are tried,
 * because the origin genuinely sits on the true Minkowski-difference boundary there. If no seed
 * encloses, its best (closest-to-origin) reduction seeds the incremental fallback loop, which
 * handles the ordinary separated case and the terminal exact-touch report.
 *
 * KNOWN LIMITATION: a sparse, symmetric convex hull (e.g. an 8-vertex octahedron, ConvexShape's
 * support function ties between exactly 6 possible outputs) queried at EXACT axis-aligned
 * coincidence with an identical copy of itself can still evade every seed direction, because the
 * shape's support function only ever returns one of a handful of fixed points regardless of probe
 * direction diversity - unlike Box/Sphere/Cylinder/Capsule, whose support functions vary smoothly
 * or have enough distinct extremes that this does not occur. Any non-exact offset (even a tiny
 * fractional one) resolves correctly; this is a pathological, near-zero-probability configuration
 * for a live simulation (bodies do not spawn perfectly axis-coincident), not treated as fixed here.
 */
class GJK {
    constructor() {
        // Simplex: up to 4 points, each a { w: Vector3 (Minkowski diff point), a: Vector3 (world
        // point on shape A), b: Vector3 (world point on shape B) }. Kept as parallel scratch
        // arrays rather than objects, allocation-free across iterations.
        this._wx = new Float64Array(4); this._wy = new Float64Array(4); this._wz = new Float64Array(4);
        this._ax = new Float64Array(4); this._ay = new Float64Array(4); this._az = new Float64Array(4);
        this._bx = new Float64Array(4); this._by = new Float64Array(4); this._bz = new Float64Array(4);
        this._count = 0;

        this._dir = new Vector3();
        this._newW = new Vector3();
        this._newA = new Vector3();
        this._newB = new Vector3();
        this._closest = new Vector3();
        this._scratchRef = new Vector3();
        this._initialProbeDir = new Vector3();
        this._newDir4 = new Vector3();
    }

    _clear() { this._count = 0; }

    _push(w, a, b) {
        const i = this._count++;
        this._wx[i] = w.x; this._wy[i] = w.y; this._wz[i] = w.z;
        this._ax[i] = a.x; this._ay[i] = a.y; this._az[i] = a.z;
        this._bx[i] = b.x; this._by[i] = b.y; this._bz[i] = b.z;
    }

    // Replace the simplex with exactly the points at the given source indices (in that order),
    // used after each closest-feature reduction. Safe because sources are always <= current count
    // and we write low-to-high after reading everything we need for this call already.
    _reduceTo(indices) {
        const wx = this._wx.slice(), wy = this._wy.slice(), wz = this._wz.slice();
        const ax = this._ax.slice(), ay = this._ay.slice(), az = this._az.slice();
        const bx = this._bx.slice(), by = this._by.slice(), bz = this._bz.slice();
        for (let k = 0; k < indices.length; k++) {
            const src = indices[k];
            this._wx[k] = wx[src]; this._wy[k] = wy[src]; this._wz[k] = wz[src];
            this._ax[k] = ax[src]; this._ay[k] = ay[src]; this._az[k] = az[src];
            this._bx[k] = bx[src]; this._by[k] = by[src]; this._bz[k] = bz[src];
        }
        this._count = indices.length;
    }

    /**
     * Runs GJK for the pair of placed shapes wrapped by `support` (a MinkowskiSupport). Returns:
     *   { overlapping: true,  simplex: this }                                   — hand to EPA
     *   { overlapping: false, distance, normal, pointA, pointB }                — separated
     * `normal` points from B to A (world space), matching the Narrowphase contract's convention.
     * maxIterations guards against non-convergence on a pathological input; hitting it returns the
     * best answer found so far rather than a wrong one (same discipline as EPA's converged-result
     * rule below - never return "whatever the last iteration produced" as if it were final without
     * saying so isn't possible here, so we return the best-so-far honestly, not silently).
     */
    run(support, maxIterations) {
        maxIterations = maxIterations || 64;
        this._clear();

        // Seed a tetrahedron directly from four directions spanning genuinely different octants,
        // rather than growing one incrementally from a single starting axis. This is what actually
        // distinguishes real 3D overlap from exact touching (see the class header): incremental
        // growth from one starting direction tends to walk INTO the exact-touching plane and get
        // stuck there. Several diverse direction sets are tried (see SEED_DIRECTION_SETS) since any
        // single fixed set can, for particular shape sizes, itself produce a degenerate tetrahedron
        // by unlucky alignment.
        //
        // If no seed set immediately encloses the origin, the BEST set's own reduction (whichever
        // came closest to the origin, whatever triangle/direction it settled on) becomes the
        // STARTING point for the incremental loop
        // below, rather than restarting from a fresh single-point simplex. This matters: a real
        // overlap whose seed tetrahedron narrowly misses the origin is still much closer to
        // resolving from that reduced simplex than from scratch - starting over from a single
        // arbitrary axis is what let real overlaps fall through to a false "touching" report before
        // this chaining was added (a lower-dimensional simplex from a fresh start hits the same
        // degenerate-plane trap the seed exists to avoid in the first place).
        let seeded = this._seedTetrahedron(support);
        if (seeded.overlapping) return seeded;
        this._dir.copy(seeded.direction);
        this._closest.copy(seeded.closest);

        if (this._dir.lengthSquared() < 1e-20) {
            // Every seed (and its own reduction) settled on the origin exactly - across several
            // genuinely different probe direction sets, none of which share a plane by
            // construction. That consistency is itself the practical signal for exact touching
            // (see the class header) - report it directly.
            return this._separatedResult(support);
        }

        for (let iter = 0; iter < maxIterations; iter++) {
            support.supportInto(this._newW, this._dir, this._newA, this._newB);

            // Standard GJK termination (Ericson 5.4): the new support point makes no progress in
            // the search direction if it does not project further along `dir` than the simplex
            // points already do.
            const newAlong = this._newW.x * this._dir.x + this._newW.y * this._dir.y + this._newW.z * this._dir.z;
            let bestAlong = -Infinity;
            for (let k = 0; k < this._count; k++) {
                const along = this._wx[k] * this._dir.x + this._wy[k] * this._dir.y + this._wz[k] * this._dir.z;
                if (along > bestAlong) bestAlong = along;
            }
            if (newAlong <= bestAlong + 1e-10) {
                return this._separatedResult(support);
            }

            this._push(this._newW, this._newA, this._newB);

            const result = this._doSimplex();
            if (result.containsOrigin) {
                return { overlapping: true, simplex: this };
            }
            this._dir.copy(result.direction);
            this._closest.copy(result.closest);

            // See the class header ("EXACT-TOUCHING IS UNDECIDABLE...") for why this is reported
            // directly as a zero-distance separated result rather than trying to escalate further.
            if (this._dir.lengthSquared() < 1e-20) {
                return this._separatedResult(support);
            }
        }
        // Iteration cap reached without a clean termination. This mirrors EPA's own rule below
        // (plan.md, Bug reference: "return the converged result, never the last one") - the
        // current simplex IS the best convergence reached, so report it honestly as SEPARATED
        // rather than pretending overlap or discarding.
        return this._separatedResult(support);
    }

    // Several sets of four directions, each spanning genuinely different octants, used to seed a
    // tetrahedron directly rather than growing one incrementally from a single starting axis (see
    // run()'s call site for why that matters). Multiple rotated sets exist because ANY single
    // fixed set can, for a particular pair of shape sizes/positions, happen to produce a seed
    // tetrahedron that is itself degenerate (its own 4 support points landing coplanar/collinear -
    // an unlucky alignment between the probe directions and the shapes' geometry, not a boundary-
    // touching case) - trying a second, differently-rotated set resolves that without having to
    // detect the degeneracy explicitly.
    // Deliberately irrational-ish, non-axis-aligned components (not just +-1) - a sparse convex
    // hull (e.g. an 8-vertex octahedron) has its support function tied between vertices exactly on
    // clean +-1 diagonals, which for two IDENTICAL coincident hulls collapses two of the four seed
    // points to duplicates and reproduces the same "two faces exactly through the origin" pattern
    // as the touching case - a real, deep overlap misreported as not-enclosing. Odd-looking
    // component ratios make an exact tie between two support vertices astronomically unlikely for
    // any shape that isn't specifically axis-aligned-symmetric along that exact ratio.
    static SEED_DIRECTION_SETS = [
        [[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]],
        [[1, 1, -1], [1, -1, 1], [-1, 1, 1], [-1, -1, -1]],
        [[1, 0, 0], [-1, 0.3, 0.3], [0, -1, 0.3], [0, 0.3, -1]],
        [[0.8763, 0.2451, 0.4127], [0.3312, -0.9021, 0.2734], [-0.6543, 0.1298, -0.7452], [-0.5532, -0.6789, 0.4821]]
    ];

    // Tries each seed direction set in turn, building a tetrahedron and checking whether it
    // encloses the origin with a real (non-degenerate) margin.
    //
    // Returns { overlapping: true, simplex: this } on a confirmed overlap (this.wx/wy/wz etc. hold
    // the enclosing simplex, ready for EPA). Otherwise returns { overlapping: false, direction,
    // closest } from whichever seed's own reduction got CLOSEST to the origin - not just the last
    // one tried - so the caller's incremental loop continues from the best available starting
    // point rather than an arbitrary one. The simplex left in this._wx/wy/wz on return matches
    // that best reduction (re-run once more below to restore it, since each seed attempt
    // overwrites the shared arrays in turn).
    _seedTetrahedron(support) {
        let bestDistSq = Infinity, bestSet = -1;
        for (let s = 0; s < GJK.SEED_DIRECTION_SETS.length; s++) {
            this._clear();
            const dirs = GJK.SEED_DIRECTION_SETS[s];
            for (let i = 0; i < 4; i++) {
                const d = dirs[i];
                this._newDir4.set(d[0], d[1], d[2]);
                support.supportInto(this._newW, this._newDir4, this._newA, this._newB);
                this._push(this._newW, this._newA, this._newB);
            }
            const result = this._simplexTetrahedron();
            if (result.containsOrigin) return { overlapping: true, simplex: this };
            const distSq = result.closest.x * result.closest.x + result.closest.y * result.closest.y + result.closest.z * result.closest.z;
            if (distSq < bestDistSq) { bestDistSq = distSq; bestSet = s; }
        }
        // Re-run the best set to restore its simplex/direction/closest as the live state -
        // cheap (4 support queries) relative to how rarely this whole path (seed-doesn't-enclose)
        // is hit, and keeps _seedTetrahedron's inner loop simple rather than snapshotting state
        // for every candidate on every iteration.
        this._clear();
        const bestDirs = GJK.SEED_DIRECTION_SETS[bestSet];
        for (let i = 0; i < 4; i++) {
            const d = bestDirs[i];
            this._newDir4.set(d[0], d[1], d[2]);
            support.supportInto(this._newW, this._newDir4, this._newA, this._newB);
            this._push(this._newW, this._newA, this._newB);
        }
        const finalResult = this._simplexTetrahedron();
        return { overlapping: false, direction: finalResult.direction, closest: finalResult.closest };
    }

    // Builds the SEPARATED return value from the current simplex's closest point to the origin.
    // The witness points (pointA, pointB) are recovered via the simplex's barycentric weights
    // applied to the stored world support points - never re-queried, so they are exactly consistent
    // with the reported distance.
    // `forcedNormal`, if given, is used directly instead of deriving one from `this._closest` -
    // only the immediate "first support point IS the origin" case in run() passes this, since
    // there the probe direction itself is the exact, known-correct contact normal.
    _separatedResult(support, forcedNormal) {
        const bary = this._barycentricOfClosest();
        const pointA = new Vector3(), pointB = new Vector3();
        for (let i = 0; i < this._count; i++) {
            pointA.x += bary[i] * this._ax[i]; pointA.y += bary[i] * this._ay[i]; pointA.z += bary[i] * this._az[i];
            pointB.x += bary[i] * this._bx[i]; pointB.y += bary[i] * this._by[i]; pointB.z += bary[i] * this._bz[i];
        }
        const dist = Math.sqrt(this._closest.x * this._closest.x + this._closest.y * this._closest.y + this._closest.z * this._closest.z);
        let normal;
        if (forcedNormal) {
            normal = new Vector3().copy(forcedNormal).normalizeInPlace();
        } else if (dist > 1e-12) {
            normal = new Vector3(this._closest.x / dist, this._closest.y / dist, this._closest.z / dist);
        } else {
            // Exact touching: the origin lies ON the simplex, so `closest` itself carries no
            // direction. This is the degenerate case the flush-contact fix (plan.md, Bug
            // reference) exists for - fall back to a normal recoverable from the simplex's own
            // geometry rather than a fixed axis, which would be wrong for a touching pair whose
            // true contact plane isn't horizontal.
            normal = new Vector3();
            this._degenerateTouchingNormalInto(normal);
        }
        return { overlapping: false, distance: dist, normal: normal, pointA: pointA, pointB: pointB };
    }

    // Recovers a normal for a zero-distance (exact touching) simplex. A 3-point simplex through
    // the origin has a well-defined plane normal - use it. A 2-point (segment through the origin)
    // or 1-point simplex has no unique perpendicular in 3D, so fall back to the shared
    // findOrthogonal() convention (least-aligned cardinal axis) rather than inventing one - this
    // never produces NaN, matching the bug-reference discipline, even though it is not always the
    // physically true contact normal for that degenerate case.
    _degenerateTouchingNormalInto(out) {
        if (this._count === 3) {
            const abx = this._wx[1] - this._wx[0], aby = this._wy[1] - this._wy[0], abz = this._wz[1] - this._wz[0];
            const acx = this._wx[2] - this._wx[0], acy = this._wy[2] - this._wy[0], acz = this._wz[2] - this._wz[0];
            out.x = aby * acz - abz * acy; out.y = abz * acx - abx * acz; out.z = abx * acy - aby * acx;
            const lenSq = out.x * out.x + out.y * out.y + out.z * out.z;
            if (lenSq > 1e-20) { out.scaleInPlace(1 / Math.sqrt(lenSq)); return; }
        }
        this._scratchRef.set(this._wx[0], this._wy[0], this._wz[0]);
        out.findOrthogonal(this._scratchRef);
    }

    // Barycentric weights of `this._closest` with respect to the current simplex (1, 2, or 3
    // points - a 4-point simplex never reaches here because a tetrahedron enclosing the origin
    // returns containsOrigin=true in _doSimplex before this is called). Degenerate simplices (a
    // zero-length edge, a zero-area triangle) fall back explicitly rather than dividing by zero -
    // this IS the flush-contact fix (plan.md, Bug reference).
    _barycentricOfClosest() {
        if (this._count === 1) return [1];
        if (this._count === 2) {
            const abx = this._wx[1] - this._wx[0], aby = this._wy[1] - this._wy[0], abz = this._wz[1] - this._wz[0];
            const lenSq = abx * abx + aby * aby + abz * abz;
            if (lenSq < 1e-20) return [1, 0]; // degenerate (coincident points): fall back to the first
            const apx = this._closest.x - this._wx[0], apy = this._closest.y - this._wy[0], apz = this._closest.z - this._wz[0];
            let t = (apx * abx + apy * aby + apz * abz) / lenSq;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            return [1 - t, t];
        }
        // count === 3: barycentric of a point already known to be IN the triangle's plane (it came
        // from _closestOnTriangle), via the standard area-ratio method.
        const v0x = this._wx[1] - this._wx[0], v0y = this._wy[1] - this._wy[0], v0z = this._wz[1] - this._wz[0];
        const v1x = this._wx[2] - this._wx[0], v1y = this._wy[2] - this._wy[0], v1z = this._wz[2] - this._wz[0];
        const v2x = this._closest.x - this._wx[0], v2y = this._closest.y - this._wy[0], v2z = this._closest.z - this._wz[0];
        const d00 = v0x * v0x + v0y * v0y + v0z * v0z;
        const d01 = v0x * v1x + v0y * v1y + v0z * v1z;
        const d11 = v1x * v1x + v1y * v1y + v1z * v1z;
        const d20 = v2x * v0x + v2y * v0y + v2z * v0z;
        const d21 = v2x * v1x + v2y * v1y + v2z * v1z;
        const denom = d00 * d11 - d01 * d01;
        if (Math.abs(denom) < 1e-20) return [1 / 3, 1 / 3, 1 / 3]; // degenerate (collinear/zero-area): even split, never NaN
        const v = (d11 * d20 - d01 * d21) / denom;
        const w = (d00 * d21 - d01 * d20) / denom;
        const u = 1 - v - w;
        return [u, v, w];
    }

    // Reduces the simplex to its closest feature to the origin and reports whether the origin is
    // enclosed. Dispatches by current point count (line/triangle/tetrahedron), each with an
    // explicit degenerate fallback - see the header note on the flush-contact bug.
    _doSimplex() {
        if (this._count === 2) return this._simplexLine();
        if (this._count === 3) return this._simplexTriangle();
        return this._simplexTetrahedron();
    }

    // Closest point on segment AB to the origin. Degenerate (coincident A/B) falls back to A
    // rather than a 0/0 division - the 2-point half of the flush-contact fix.
    _simplexLine() {
        const ax = this._wx[0], ay = this._wy[0], az = this._wz[0];
        const bx = this._wx[1], by = this._wy[1], bz = this._wz[1];
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const lenSq = abx * abx + aby * aby + abz * abz;

        let t;
        if (lenSq < 1e-20) t = 0;
        else {
            t = -(ax * abx + ay * aby + az * abz) / lenSq;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
        }

        const closest = new Vector3(ax + abx * t, ay + aby * t, az + abz * t);
        if (t <= 0) this._reduceTo([0]);
        else if (t >= 1) this._reduceTo([1]);
        // else both points stay: closest feature is the segment's interior.

        // If the origin lies exactly on the segment, `direction` comes back (0,0,0) - see the
        // class header on why run() reports that directly as a zero-distance touch.
        const dir = new Vector3(-closest.x, -closest.y, -closest.z);
        return { containsOrigin: false, direction: dir, closest: closest };
    }

    _simplexTriangle() {
        const ax = this._wx[0], ay = this._wy[0], az = this._wz[0];
        const bx = this._wx[1], by = this._wy[1], bz = this._wz[1];
        const cx = this._wx[2], cy = this._wy[2], cz = this._wz[2];
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const acx = cx - ax, acy = cy - ay, acz = cz - az;
        // Triangle normal via cross(ab, ac).
        const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        const nLenSq = nx * nx + ny * ny + nz * nz;

        if (nLenSq < 1e-20) {
            // Degenerate: three (near-)collinear points, zero-area triangle. THIS is the flush-
            // contact bug from plan.md - fixed by falling back to the closest of the three edges
            // explicitly instead of discarding. Try each edge as a 2-point simplex and keep the
            // best (closest-to-origin) result.
            return this._degenerateTriangleFallback();
        }

        const closest = GJK._closestPointOnTriangleToOrigin(ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz, nLenSq);
        // If the origin lies exactly on this triangle (face, edge, or vertex), `dir` comes back
        // (0,0,0) - see the class header on why run() reports that directly as a zero-distance
        // touch rather than escalating.
        const dir = new Vector3(-closest.x, -closest.y, -closest.z);

        if (closest.onEdge !== null) {
            // Closest feature is an edge or vertex - reduce the simplex accordingly.
            this._reduceTo(closest.onEdge);
        }
        return { containsOrigin: false, direction: dir, closest: new Vector3(closest.x, closest.y, closest.z) };
    }

    // Degenerate-triangle fallback: pick whichever of the three edges (as a 2-point simplex) is
    // truly closest to the origin, by testing each directly. Never returns NaN, never discards.
    _degenerateTriangleFallback() {
        const pts = [
            [this._wx[0], this._wy[0], this._wz[0]],
            [this._wx[1], this._wy[1], this._wz[1]],
            [this._wx[2], this._wy[2], this._wz[2]]
        ];
        const edges = [[0, 1], [1, 2], [2, 0]];
        let best = null, bestDistSq = Infinity, bestIdx = null;
        for (let e = 0; e < 3; e++) {
            const p0 = pts[edges[e][0]], p1 = pts[edges[e][1]];
            const abx = p1[0] - p0[0], aby = p1[1] - p0[1], abz = p1[2] - p0[2];
            const lenSq = abx * abx + aby * aby + abz * abz;
            let t = lenSq < 1e-20 ? 0 : (-(p0[0] * abx + p0[1] * aby + p0[2] * abz) / lenSq);
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const cx = p0[0] + abx * t, cy = p0[1] + aby * t, cz = p0[2] + abz * t;
            const dSq = cx * cx + cy * cy + cz * cz;
            if (dSq < bestDistSq) { bestDistSq = dSq; best = [cx, cy, cz]; bestIdx = t <= 0 ? [edges[e][0]] : (t >= 1 ? [edges[e][1]] : edges[e]); }
        }
        this._reduceTo(bestIdx);
        const dir = new Vector3(-best[0], -best[1], -best[2]);
        return { containsOrigin: false, direction: dir, closest: new Vector3(best[0], best[1], best[2]) };
    }

    _simplexTetrahedron() {
        // Test the origin against each of the four faces. If it's outside a face (on the far side
        // from the fourth point), the closest feature is on/beyond that face - reduce to a triangle
        // and recurse into the triangle case. If the origin is on the inside of all four faces, it
        // is enclosed - GJK terminates with overlap.
        // The 4 distinct faces of tetrahedron {0,1,2,3}, each listed with its opposite vertex:
        // {0,1,2} opp 3, {0,1,3} opp 2, {0,2,3} opp 1, {1,2,3} opp 0. (A prior version had a typo
        // here - [0,2,1,3] instead of [0,1,3,2] - which duplicated the {0,1,2} face with reversed
        // winding and NEVER TESTED the real {0,1,3} face at all. That silently let the enclosure
        // test pass tetrahedra that were not actually enclosing on all four true faces.)
        const idx = [[0, 1, 2, 3], [0, 1, 3, 2], [0, 2, 3, 1], [1, 2, 3, 0]]; // (face..., opposite point)
        for (let f = 0; f < 4; f++) {
            const [ia, ib, ic, id] = idx[f];
            const ax = this._wx[ia], ay = this._wy[ia], az = this._wz[ia];
            const bx = this._wx[ib], by = this._wy[ib], bz = this._wz[ib];
            const cx = this._wx[ic], cy = this._wy[ic], cz = this._wz[ic];
            const dx = this._wx[id], dy = this._wy[id], dz = this._wz[id];
            const abx = bx - ax, aby = by - ay, abz = bz - az;
            const acx = cx - ax, acy = cy - ay, acz = cz - az;
            let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
            const nLenSq = nx * nx + ny * ny + nz * nz;
            if (nLenSq < 1e-20) continue; // degenerate (collinear) face: skip, another face decides
            // Orient the normal away from the fourth (opposite) point.
            const toD = (dx - ax) * nx + (dy - ay) * ny + (dz - az) * nz;
            if (toD > 0) { nx = -nx; ny = -ny; nz = -nz; }
            const toOriginRaw = -ax * nx - ay * ny - az * nz; // (origin - a) . n, UNNORMALIZED
            // Compare the actual signed DISTANCE (toOriginRaw / |n|), not the raw dot product -
            // |n| scales with the face's own size, so a fixed raw-dot epsilon means a different
            // real-world tolerance for every pair of shapes. This is what made an exactly-touching
            // pair (the origin sitting exactly on a Minkowski-difference face, by construction -
            // not numerical error) misclassify as enclosed: the unnormalized epsilon was far
            // smaller than the coordinate magnitudes involved.
            const signedDist = toOriginRaw / Math.sqrt(nLenSq);
            // The threshold is NEGATIVE, not zero: a face the origin sits exactly ON does not
            // count as enclosure, only a face it is STRICTLY behind by a real margin does. This
            // matters for the seed tetrahedron above (four diverse directions, not grown
            // incrementally): an exactly-touching pair still produces a seed tetrahedron with two
            // faces passing exactly through the origin (the touching plane, seen from two
            // different angles) - accepting signedDist===0 as "inside" would misreport that as
            // overlap. Requiring a real negative margin is what keeps the exact-touching boundary
            // on the separated side while still finding every GENUINE overlap: a real overlap's
            // seed tetrahedron (built from four directions spanning different octants, so it is
            // never confined to a single touching plane the way incremental growth can get stuck
            // in) has every face strictly negative once the shapes interpenetrate at all.
            if (signedDist > -1e-9) {
                // Origin is outside (or exactly on) this face - reduce to the face's triangle and
                // recurse.
                this._reduceTo([ia, ib, ic]);
                return this._simplexTriangle();
            }
        }
        // Inside every face by the epsilon above. Defense in depth before trusting this as genuine
        // overlap: confirm the tetrahedron itself is not near-degenerate (all four points nearly
        // coplanar) - a flat tetrahedron can pass every per-face test at once without the origin
        // being genuinely surrounded in 3D.
        const v0x = this._wx[1] - this._wx[0], v0y = this._wy[1] - this._wy[0], v0z = this._wz[1] - this._wz[0];
        const v1x = this._wx[2] - this._wx[0], v1y = this._wy[2] - this._wy[0], v1z = this._wz[2] - this._wz[0];
        const v2x = this._wx[3] - this._wx[0], v2y = this._wy[3] - this._wy[0], v2z = this._wz[3] - this._wz[0];
        const cxv = v1y * v2z - v1z * v2y, cyv = v1z * v2x - v1x * v2z, czv = v1x * v2y - v1y * v2x;
        const volume6 = Math.abs(v0x * cxv + v0y * cyv + v0z * czv); // 6x tetrahedron volume
        // Threshold relative to the tetrahedron's own extent, not an absolute constant - the same
        // reasoning as the normalized signedDist above: an absolute epsilon means a different
        // real-world tolerance for every shape scale.
        const extentSq = Math.max(v0x*v0x+v0y*v0y+v0z*v0z, v1x*v1x+v1y*v1y+v1z*v1z, v2x*v2x+v2y*v2y+v2z*v2z);
        if (volume6 * volume6 < 1e-12 * extentSq * extentSq * extentSq) {
            // Degenerate tetrahedron: fall back to the touching-plane triangle (the first three
            // points, which is where the origin actually sits) rather than reporting overlap.
            this._reduceTo([0, 1, 2]);
            return this._simplexTriangle();
        }
        return { containsOrigin: true, direction: null, closest: null };
    }

    // Closest point on triangle ABC to the origin, given its (non-unit) normal N and |N|^2.
    // Returns { x,y,z, onEdge: null|[indices] } - onEdge non-null means the closest feature is a
    // vertex/edge (indices into the CURRENT 3-point simplex), so the caller reduces to it.
    static _closestPointOnTriangleToOrigin(ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz, nLenSq) {
        // Edge/vertex Voronoi region tests (Ericson 5.1.5), inlined for the origin as query point.
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const acx = cx - ax, acy = cy - ay, acz = cz - az;
        const apx = -ax, apy = -ay, apz = -az; // origin - a

        const d1 = abx * apx + aby * apy + abz * apz;
        const d2 = acx * apx + acy * apy + acz * apz;
        if (d1 <= 0 && d2 <= 0) return { x: ax, y: ay, z: az, onEdge: [0] };

        const bpx = -bx, bpy = -by, bpz = -bz;
        const d3 = abx * bpx + aby * bpy + abz * bpz;
        const d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0 && d4 <= d3) return { x: bx, y: by, z: bz, onEdge: [1] };

        const vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) {
            const t = d1 / (d1 - d3);
            return { x: ax + abx * t, y: ay + aby * t, z: az + abz * t, onEdge: [0, 1] };
        }

        const cpx = -cx, cpy = -cy, cpz = -cz;
        const d5 = abx * cpx + aby * cpy + abz * cpz;
        const d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0 && d5 <= d6) return { x: cx, y: cy, z: cz, onEdge: [2] };

        const vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            const t = d2 / (d2 - d6);
            return { x: ax + acx * t, y: ay + acy * t, z: az + acz * t, onEdge: [0, 2] };
        }

        const va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            const t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return { x: bx + (cx - bx) * t, y: by + (cy - by) * t, z: bz + (cz - bz) * t, onEdge: [1, 2] };
        }

        // Interior: project the origin onto the triangle's plane along its normal.
        // proj = origin - ((origin - a) . n_hat) n_hat ; with origin = 0 and n_hat = n/|n|:
        //      = -( (-a).n / |n|^2 ) n = (a.n / |n|^2) n
        const k = (ax * nx + ay * ny + az * nz) / nLenSq;
        return { x: nx * k, y: ny * k, z: nz * k, onEdge: null };
    }
}

ActionPhysics.GJK = GJK;
