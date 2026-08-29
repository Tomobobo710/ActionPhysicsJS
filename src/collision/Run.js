// The main GJK loop, plus building the SEPARATED result from a converged simplex.
var proto = GJK.prototype;

/**
 * Runs GJK for the pair of placed shapes wrapped by `support` (a MinkowskiSupport). Returns:
 *   { overlapping: true,  simplex: this }                            -> hand to EPA
 *   { overlapping: false, distance, normal, pointA, pointB }         -> separated
 * `normal` points from B to A (world space). maxIterations guards non-convergence; hitting it
 * returns the best answer found so far, reported honestly as separated.
 */
proto.run = function (support, maxIterations) {
    maxIterations = maxIterations || 64;
    this._clear();

    // Seed a tetrahedron from diverse directions (see Seeding.js). If none encloses, its best
    // reduction seeds the incremental loop below.
    let seeded = this._seedTetrahedron(support);
    if (seeded.overlapping) return seeded;
    this._dir.copy(seeded.direction);
    this._closest.copy(seeded.closest);

    if (this._dir.lengthSquared() < 1e-20) {
        // Origin lies on the Minkowski-difference boundary - either an exact touch or a shallow
        // penetration the seed tetrahedra couldn't enclose. Disambiguate via strict interiority.
        return this._originStrictlyInside(support) ? { overlapping: true, simplex: this } : this._separatedResult(support);
    }

    for (let iter = 0; iter < maxIterations; iter++) {
        support.supportInto(this._newW, this._dir, this._newA, this._newB);

        // Standard GJK termination (Ericson 5.4): no progress if the new support doesn't project
        // further along `dir` than the simplex already does.
        const newAlong = this._newW.x * this._dir.x + this._newW.y * this._dir.y + this._newW.z * this._dir.z;
        let bestAlong = -Infinity;
        for (let k = 0; k < this._count; k++) {
            const along = this._wx[k] * this._dir.x + this._wy[k] * this._dir.y + this._wz[k] * this._dir.z;
            if (along > bestAlong) bestAlong = along;
        }
        if (newAlong <= bestAlong + 1e-10) {
            // Stall. Near-zero closest distance means the origin is on/inside the boundary - defer
            // to the strict-interior check; otherwise separated.
            const closestDistSq = this._closest.x * this._closest.x + this._closest.y * this._closest.y + this._closest.z * this._closest.z;
            if (closestDistSq < GJK.OVERLAP_DISTANCE_EPSILON * GJK.OVERLAP_DISTANCE_EPSILON) {
                return this._originStrictlyInside(support) ? { overlapping: true, simplex: this } : this._separatedResult(support);
            }
            return this._separatedResult(support);
        }

        this._push(this._newW, this._newA, this._newB);

        const result = this._doSimplex();
        if (result.containsOrigin) return { overlapping: true, simplex: this };
        this._dir.copy(result.direction);
        this._closest.copy(result.closest);

        if (this._dir.lengthSquared() < 1e-20) {
            return this._originStrictlyInside(support) ? { overlapping: true, simplex: this } : this._separatedResult(support);
        }
    }
    return this._separatedResult(support); // iteration cap - report honestly as separated
};

// SEPARATED result from the simplex's closest point to the origin. Witness points are recovered
// from barycentric weights on the stored support points, so they stay consistent with `distance`.
proto._separatedResult = function (support, forcedNormal) {
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
        // Exact touching: `closest` carries no direction. Recover a normal from the simplex's own
        // geometry instead of a fixed axis.
        normal = new Vector3();
        this._degenerateTouchingNormalInto(normal);
    }
    return { overlapping: false, distance: dist, normal: normal, pointA: pointA, pointB: pointB };
};

// Recovers a normal for a zero-distance (exact touching) simplex. A 3-point simplex through the
// origin has a well-defined plane normal; a 2- or 1-point simplex falls back to findOrthogonal()
// (never NaN, even though not always the true contact normal for that degenerate case).
proto._degenerateTouchingNormalInto = function (out) {
    if (this._count === 3) {
        const abx = this._wx[1] - this._wx[0], aby = this._wy[1] - this._wy[0], abz = this._wz[1] - this._wz[0];
        const acx = this._wx[2] - this._wx[0], acy = this._wy[2] - this._wy[0], acz = this._wz[2] - this._wz[0];
        out.x = aby * acz - abz * acy; out.y = abz * acx - abx * acz; out.z = abx * acy - aby * acx;
        const lenSq = out.x * out.x + out.y * out.y + out.z * out.z;
        if (lenSq > 1e-20) { out.scaleInPlace(1 / Math.sqrt(lenSq)); return; }
    }
    this._scratchRef.set(this._wx[0], this._wy[0], this._wz[0]);
    out.findOrthogonal(this._scratchRef);
};

// Barycentric weights of `this._closest` w.r.t. the current simplex (1-3 points). Degenerate
// simplices fall back explicitly rather than dividing by zero.
proto._barycentricOfClosest = function () {
    if (this._count === 1) return [1];
    if (this._count === 2) {
        const abx = this._wx[1] - this._wx[0], aby = this._wy[1] - this._wy[0], abz = this._wz[1] - this._wz[0];
        const lenSq = abx * abx + aby * aby + abz * abz;
        if (lenSq < 1e-20) return [1, 0];
        const apx = this._closest.x - this._wx[0], apy = this._closest.y - this._wy[0], apz = this._closest.z - this._wz[0];
        let t = (apx * abx + apy * aby + apz * abz) / lenSq;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        return [1 - t, t];
    }
    // count === 3: barycentric of a point already known to be in the triangle's plane.
    const v0x = this._wx[1] - this._wx[0], v0y = this._wy[1] - this._wy[0], v0z = this._wz[1] - this._wz[0];
    const v1x = this._wx[2] - this._wx[0], v1y = this._wy[2] - this._wy[0], v1z = this._wz[2] - this._wz[0];
    const v2x = this._closest.x - this._wx[0], v2y = this._closest.y - this._wy[0], v2z = this._closest.z - this._wz[0];
    const d00 = v0x * v0x + v0y * v0y + v0z * v0z;
    const d01 = v0x * v1x + v0y * v1y + v0z * v1z;
    const d11 = v1x * v1x + v1y * v1y + v1z * v1z;
    const d20 = v2x * v0x + v2y * v0y + v2z * v0z;
    const d21 = v2x * v1x + v2y * v1y + v2z * v1z;
    const denom = d00 * d11 - d01 * d01;
    if (Math.abs(denom) < 1e-20) return [1 / 3, 1 / 3, 1 / 3];
    const v = (d11 * d20 - d01 * d21) / denom;
    const w = (d00 * d21 - d01 * d20) / denom;
    const u = 1 - v - w;
    return [u, v, w];
};
