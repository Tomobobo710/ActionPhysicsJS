// EPA: penetration depth, normal (B->A), and witness points from a GJK simplex that already
// encloses the origin (van den Bergen). See InitialTetrahedron.js and Expand.js.
class EPA {
    constructor() {
        // Polytope vertices, parallel arrays like GJK's. Capacity grows geometrically.
        this._capacity = 64;
        this._wx = new Float64Array(this._capacity); this._wy = new Float64Array(this._capacity); this._wz = new Float64Array(this._capacity);
        this._ax = new Float64Array(this._capacity); this._ay = new Float64Array(this._capacity); this._az = new Float64Array(this._capacity);
        this._bx = new Float64Array(this._capacity); this._by = new Float64Array(this._capacity); this._bz = new Float64Array(this._capacity);
        this._vertexCount = 0;

        // Faces: index triples + outward normal + distance-to-origin. Removed faces are marked dead
        // (faceAlive), not spliced, to avoid reindexing.
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

    // Adds a face from three vertex indices, oriented outward from `centroidHint`. Returns the new
    // face's index, or -1 if the three points are degenerate (collinear/zero area) - skipped
    // rather than added with an undefined normal.
    _addFace(ia, ib, ic, centroidHint) {
        const ax = this._wx[ia], ay = this._wy[ia], az = this._wz[ia];
        const bx = this._wx[ib], by = this._wy[ib], bz = this._wz[ib];
        const cx = this._wx[ic], cy = this._wy[ic], cz = this._wz[ic];
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const acx = cx - ax, acy = cy - ay, acz = cz - az;
        let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        const nLenSq = nx * nx + ny * ny + nz * nz;
        if (nLenSq < 1e-20) return -1;

        const invLen = 1 / Math.sqrt(nLenSq);
        nx *= invLen; ny *= invLen; nz *= invLen;

        const toHint = (centroidHint.x - ax) * nx + (centroidHint.y - ay) * ny + (centroidHint.z - az) * nz;
        if (toHint > 0) { nx = -nx; ny = -ny; nz = -nz; }

        const dist = ax * nx + ay * ny + az * nz;

        if (this._faceCount >= this._faceCapacity) this._growFaces();
        const fi = this._faceCount++;
        this._faceA[fi] = ia; this._faceB[fi] = ib; this._faceC[fi] = ic;
        this._faceNx[fi] = nx; this._faceNy[fi] = ny; this._faceNz[fi] = nz;
        this._faceDist[fi] = dist;
        this._faceAlive[fi] = 1;
        return fi;
    }
}

ActionPhysics.EPA = EPA;
