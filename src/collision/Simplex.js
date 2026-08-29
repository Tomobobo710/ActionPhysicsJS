// Simplex reduction: closest point to the origin for a 2/3/4-point simplex, with degenerate
// fallbacks so a flush contact never produces NaN.
var proto = GJK.prototype;

// Dispatches by point count; a 4-point simplex either encloses the origin or reduces to a triangle.
proto._doSimplex = function () {
    if (this._count === 2) return this._simplexLine();
    if (this._count === 3) return this._simplexTriangle();
    return this._simplexTetrahedron();
};

// Closest point on segment AB to the origin. Degenerate (coincident A/B) falls back to A.
proto._simplexLine = function () {
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

    // Origin exactly on the segment -> direction is (0,0,0), reported as a zero-distance touch by run().
    const dir = new Vector3(-closest.x, -closest.y, -closest.z);
    return { containsOrigin: false, direction: dir, closest: closest };
};

proto._simplexTriangle = function () {
    const ax = this._wx[0], ay = this._wy[0], az = this._wz[0];
    const bx = this._wx[1], by = this._wy[1], bz = this._wz[1];
    const cx = this._wx[2], cy = this._wy[2], cz = this._wz[2];
    const abx = bx - ax, aby = by - ay, abz = bz - az;
    const acx = cx - ax, acy = cy - ay, acz = cz - az;
    const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
    const nLenSq = nx * nx + ny * ny + nz * nz;

    if (nLenSq < 1e-20) return this._degenerateTriangleFallback(); // three (near-)collinear points

    const closest = GJK._closestPointOnTriangleToOrigin(ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz, nLenSq);
    const dir = new Vector3(-closest.x, -closest.y, -closest.z);

    if (closest.onEdge !== null) this._reduceTo(closest.onEdge);
    return { containsOrigin: false, direction: dir, closest: new Vector3(closest.x, closest.y, closest.z) };
};

// Zero-area triangle: pick whichever of its three edges (as a 2-point simplex) is truly closest
// to the origin, tested directly.
proto._degenerateTriangleFallback = function () {
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
};

proto._simplexTetrahedron = function () {
    // The 4 distinct faces of tetrahedron {0,1,2,3}, each with its opposite vertex.
    const idx = [[0, 1, 2, 3], [0, 1, 3, 2], [0, 2, 3, 1], [1, 2, 3, 0]];
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
        if (nLenSq < 1e-20) continue; // degenerate face: another face decides

        // Orient the normal away from the opposite point.
        const toD = (dx - ax) * nx + (dy - ay) * ny + (dz - az) * nz;
        if (toD > 0) { nx = -nx; ny = -ny; nz = -nz; }
        const toOriginRaw = -ax * nx - ay * ny - az * nz;
        // Signed DISTANCE (not raw dot) - |n| scales with the face's own size, so a raw-dot epsilon
        // gives a different real-world tolerance per shape pair.
        const signedDist = toOriginRaw / Math.sqrt(nLenSq);
        // Threshold is negative, not zero: a face the origin sits exactly ON (an exact touch) must
        // not count as enclosure, only strictly-behind does.
        if (signedDist > -1e-9) {
            this._reduceTo([ia, ib, ic]);
            return this._simplexTriangle();
        }
    }
    // Inside every face. Confirm the tetrahedron isn't itself near-degenerate (near-coplanar
    // points can pass every per-face test without genuinely surrounding the origin in 3D).
    const v0x = this._wx[1] - this._wx[0], v0y = this._wy[1] - this._wy[0], v0z = this._wz[1] - this._wz[0];
    const v1x = this._wx[2] - this._wx[0], v1y = this._wy[2] - this._wy[0], v1z = this._wz[2] - this._wz[0];
    const v2x = this._wx[3] - this._wx[0], v2y = this._wy[3] - this._wy[0], v2z = this._wz[3] - this._wz[0];
    const cxv = v1y * v2z - v1z * v2y, cyv = v1z * v2x - v1x * v2z, czv = v1x * v2y - v1y * v2x;
    const volume6 = Math.abs(v0x * cxv + v0y * cyv + v0z * czv);
    const extentSq = Math.max(v0x*v0x+v0y*v0y+v0z*v0z, v1x*v1x+v1y*v1y+v1z*v1z, v2x*v2x+v2y*v2y+v2z*v2z);
    if (volume6 * volume6 < 1e-12 * extentSq * extentSq * extentSq) {
        this._reduceTo([0, 1, 2]);
        return this._simplexTriangle();
    }
    return { containsOrigin: true, direction: null, closest: null };
};

// Closest point on triangle ABC to the origin, given its (non-unit) normal N and |N|^2. Returns
// { x,y,z, onEdge: null|[indices] } - onEdge non-null means the closest feature is a vertex/edge.
GJK._closestPointOnTriangleToOrigin = function (ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz, nLenSq) {
    const abx = bx - ax, aby = by - ay, abz = bz - az;
    const acx = cx - ax, acy = cy - ay, acz = cz - az;
    const apx = -ax, apy = -ay, apz = -az;

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
    const k = (ax * nx + ay * ny + az * nz) / nLenSq;
    return { x: nx * k, y: ny * k, z: nz * k, onEdge: null };
};
