// The main EPA expansion loop: grow the polytope toward the true Minkowski-difference surface,
// then extract distance/normal/witness points from the winning face.
var proto = EPA.prototype;

/**
 * Expands `simplex` (a GJK instance whose 4 points enclose the origin) into penetration depth and
 * normal. maxIterations guards non-convergence; hitting the cap still returns the live polytope's
 * closest alive face, never a stale one.
 */
proto.run = function (support, simplex, maxIterations) {
    maxIterations = maxIterations || 64;
    this._vertexCount = 0;
    this._faceCount = 0;

    if (!this._buildInitialTetrahedron(support, simplex)) {
        // No non-degenerate enclosing tetrahedron obtainable (exact touch or numerically flat) -
        // report zero depth, which the solver treats as non-penetrating.
        return this._zeroDepthResult(simplex);
    }
    const idx = [0, 1, 2, 3];
    const cx = (this._wx[idx[0]] + this._wx[idx[1]] + this._wx[idx[2]] + this._wx[idx[3]]) / 4;
    const cy = (this._wy[idx[0]] + this._wy[idx[1]] + this._wy[idx[2]] + this._wy[idx[3]]) / 4;
    const cz = (this._wz[idx[0]] + this._wz[idx[1]] + this._wz[idx[2]] + this._wz[idx[3]]) / 4;
    const centroid = { x: cx, y: cy, z: cz };

    // The 4 faces of the seed tetrahedron.
    this._addFace(idx[0], idx[1], idx[2], centroid);
    this._addFace(idx[0], idx[1], idx[3], centroid);
    this._addFace(idx[0], idx[2], idx[3], centroid);
    this._addFace(idx[1], idx[2], idx[3], centroid);

    for (let iter = 0; iter < maxIterations; iter++) {
        const face = this._closestAliveFace();
        const faceDist = this._faceDist[face];

        this._dirScratch.set(this._faceNx[face], this._faceNy[face], this._faceNz[face]);
        support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);

        const newDist = this._newW.x * this._faceNx[face] + this._newW.y * this._faceNy[face] + this._newW.z * this._faceNz[face];

        // Converged: the new support doesn't extend past the closest face's own plane by more than
        // a small margin, compared relative to the face's own distance (not a fixed epsilon) so
        // shallow contacts converge correctly too.
        if (newDist - faceDist < 1e-6) break;

        this._expandAt(this._newW, this._newA, this._newB, centroid);
    }

    // Re-query the live polytope's actual closest alive face - the only way to get the current
    // answer, whether the loop converged or hit the cap.
    return this._resultFromFace(this._closestAliveFace());
};

// Linear scan for the living face with the smallest distance-to-origin - the polytope stays small
// (tens of faces) for this engine's shape pairs.
proto._closestAliveFace = function () {
    let best = -1, bestDist = Infinity;
    for (let i = 0; i < this._faceCount; i++) {
        if (!this._faceAlive[i]) continue;
        if (this._faceDist[i] < bestDist) { bestDist = this._faceDist[i]; best = i; }
    }
    return best;
};

// Adds `newPoint` and re-triangulates: every alive face visible from it is removed, and the
// resulting hole's horizon is re-closed with new faces to the new point.
proto._expandAt = function (newW, newA, newB, centroid) {
    const newIdx = this._pushVertex(newW, newA, newB);

    // Horizon: edges shared by exactly one visible face and one non-visible face. A shared
    // internal edge between two visible faces is seen twice and cancels out.
    const horizonA = [], horizonB = [];
    function edgeKey(a, b) { return a < b ? a + ',' + b : b + ',' + a; }
    const edgeSeen = new Map();

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
            if (edgeSeen.has(key)) edgeSeen.delete(key);
            else edgeSeen.set(key, { from: from, to: to });
        }
    }

    edgeSeen.forEach(function (e) { horizonA.push(e.from); horizonB.push(e.to); });

    for (let i = 0; i < horizonA.length; i++) {
        this._addFace(horizonA[i], horizonB[i], newIdx, centroid);
    }
};

// Recovers { distance, normal, pointA, pointB } from a face - barycentric weights of the face's
// own closest point to the origin, applied to its three world witness points.
proto._resultFromFace = function (face) {
    const ia = this._faceA[face], ib = this._faceB[face], ic = this._faceC[face];
    const nx = this._faceNx[face], ny = this._faceNy[face], nz = this._faceNz[face];
    const dist = this._faceDist[face];

    // The plane is {x : x.n_hat = dist}, so dist*n_hat is the closest point on it to the origin.
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
        u = 1 / 3; v = 1 / 3; w = 1 / 3;
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

    // The face's own outward normal points A-side to B-side of the Minkowski difference A-B;
    // negated here to match GJK's convention (B to A).
    return {
        distance: Math.max(0, dist),
        normal: new Vector3(-nx, -ny, -nz),
        pointA: pointA,
        pointB: pointB
    };
};
