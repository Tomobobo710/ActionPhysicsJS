// GJK distance/overlap test via the Minkowski difference (Ericson ch. 5). Two outcomes: OVERLAPPING
// (handed to EPA) or SEPARATED (distance, witness points, normal). Exact-touch cases are handled by
// degenerate fallbacks in the simplex routines rather than producing NaN. See Seeding.js,
// Simplex.js, Run.js.
class GJK {
    constructor() {
        // Simplex: up to 4 points, each (w = Minkowski diff point, a/b = world points on A/B).
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
        this._probeDir = new Vector3();
        this._probeW = new Vector3();
    }

    _clear() { this._count = 0; }

    _push(w, a, b) {
        const i = this._count++;
        this._wx[i] = w.x; this._wy[i] = w.y; this._wz[i] = w.z;
        this._ax[i] = a.x; this._ay[i] = a.y; this._az[i] = a.z;
        this._bx[i] = b.x; this._by[i] = b.y; this._bz[i] = b.z;
    }

    // Keep only the points at `indices`, after a closest-feature reduction.
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
}

// Closest-distance below which a stalled walk is treated as overlapping rather than separated.
GJK.OVERLAP_DISTANCE_EPSILON = 1e-5;

ActionPhysics.GJK = GJK;
