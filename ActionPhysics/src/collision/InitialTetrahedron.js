// Completes a full, non-degenerate tetrahedron from GJK's simplex (which may hand over as few as
// 1 point - its enclosure/stall paths can confirm overlap from a lower-dimensional simplex when a
// small shape sits shallowly inside a much larger one). EPA needs a real 4-point tetrahedron to
// start, so this grows one dimension at a time by adding Minkowski support points.
var proto = EPA.prototype;

proto._buildInitialTetrahedron = function (support, simplex) {
    this._vertexCount = 0;
    const n = simplex._count !== undefined ? simplex._count : 4;
    for (let i = 0; i < n; i++) {
        this._pushVertex(
            { x: simplex._wx[i], y: simplex._wy[i], z: simplex._wz[i] },
            { x: simplex._ax[i], y: simplex._ay[i], z: simplex._az[i] },
            { x: simplex._bx[i], y: simplex._by[i], z: simplex._bz[i] }
        );
    }
    const AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [0, 1, 1], [1, 0, 1]];

    // 1 point -> 2: add a support along any axis that gives a distinct point.
    if (this._vertexCount === 1) {
        for (let a = 0; a < AXES.length && this._vertexCount < 2; a++) {
            for (let s = -1; s <= 1 && this._vertexCount < 2; s += 2) {
                this._dirScratch.set(AXES[a][0] * s, AXES[a][1] * s, AXES[a][2] * s);
                support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);
                if (this._distinctFrom(0)) this._pushVertex(this._newW, this._newA, this._newB);
            }
        }
        if (this._vertexCount < 2) return false;
    }
    // 2 points -> 3: add a support perpendicular to the segment.
    if (this._vertexCount === 2) {
        const ex = this._wx[1] - this._wx[0], ey = this._wy[1] - this._wy[0], ez = this._wz[1] - this._wz[0];
        for (let a = 0; a < AXES.length && this._vertexCount < 3; a++) {
            let dx = AXES[a][0], dy = AXES[a][1], dz = AXES[a][2];
            const dot = (dx * ex + dy * ey + dz * ez) / (ex * ex + ey * ey + ez * ez + 1e-30);
            dx -= dot * ex; dy -= dot * ey; dz -= dot * ez;
            if (dx * dx + dy * dy + dz * dz < 1e-12) continue;
            for (let s = -1; s <= 1 && this._vertexCount < 3; s += 2) {
                this._dirScratch.set(dx * s, dy * s, dz * s);
                support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);
                if (this._notCollinear(0, 1)) this._pushVertex(this._newW, this._newA, this._newB);
            }
        }
        if (this._vertexCount < 3) return false;
    }
    // 3 points -> 4: add support along the triangle normal (both sides).
    if (this._vertexCount === 3) {
        const ax = this._wx[0], ay = this._wy[0], az = this._wz[0];
        const abx = this._wx[1] - ax, aby = this._wy[1] - ay, abz = this._wz[1] - az;
        const acx = this._wx[2] - ax, acy = this._wy[2] - ay, acz = this._wz[2] - az;
        let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        const nl = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (nl < 1e-12) return false; // triangle itself degenerate
        nx /= nl; ny /= nl; nz /= nl;
        for (let s = -1; s <= 1 && this._vertexCount < 4; s += 2) {
            this._dirScratch.set(nx * s, ny * s, nz * s);
            support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);
            if (this._offPlane(0, 1, 2)) this._pushVertex(this._newW, this._newA, this._newB);
        }
        if (this._vertexCount < 4) return false;
    }
    return this._vertexCount >= 4;
};

proto._distinctFrom = function (i) {
    const dx = this._newW.x - this._wx[i], dy = this._newW.y - this._wy[i], dz = this._newW.z - this._wz[i];
    return dx * dx + dy * dy + dz * dz > 1e-10;
};
proto._notCollinear = function (i, j) {
    const ex = this._wx[j] - this._wx[i], ey = this._wy[j] - this._wy[i], ez = this._wz[j] - this._wz[i];
    const fx = this._newW.x - this._wx[i], fy = this._newW.y - this._wy[i], fz = this._newW.z - this._wz[i];
    const cx = ey * fz - ez * fy, cy = ez * fx - ex * fz, cz = ex * fy - ey * fx;
    return cx * cx + cy * cy + cz * cz > 1e-12;
};
proto._offPlane = function (i, j, k) {
    const ax = this._wx[i], ay = this._wy[i], az = this._wz[i];
    const abx = this._wx[j] - ax, aby = this._wy[j] - ay, abz = this._wz[j] - az;
    const acx = this._wx[k] - ax, acy = this._wy[k] - ay, acz = this._wz[k] - az;
    const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
    const dx = this._newW.x - ax, dy = this._newW.y - ay, dz = this._newW.z - az;
    const vol = dx * nx + dy * ny + dz * nz;
    return vol * vol > 1e-14;
};

// Fallback for a genuinely flat (zero-volume) contact: zero depth, using the simplex's first
// witness points and search normal. The solver's C<=0 guard treats zero depth as non-penetrating.
proto._zeroDepthResult = function (simplex) {
    const pointA = new Vector3(simplex._ax[0], simplex._ay[0], simplex._az[0]);
    const pointB = new Vector3(simplex._bx[0], simplex._by[0], simplex._bz[0]);
    let nx = 0, ny = 1, nz = 0;
    if (simplex._closest && simplex._closest.lengthSquared && simplex._closest.lengthSquared() > 1e-20) {
        const l = Math.sqrt(simplex._closest.lengthSquared());
        nx = simplex._closest.x / l; ny = simplex._closest.y / l; nz = simplex._closest.z / l;
    }
    return { distance: 0, normal: new Vector3(nx, ny, nz), pointA: pointA, pointB: pointB };
};
