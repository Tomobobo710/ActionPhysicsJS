// Tetrahedron seeding + strict-interior probe. Seeding from several diverse direction sets (rather
// than growing one incrementally from a single axis) is what tells a real 3D overlap apart from an
// exact touch: incremental growth tends to walk into the touching plane and stall there, while a
// real overlap's seed tetrahedron shows every face at a genuine negative margin once built from
// directions spanning different octants.
var proto = GJK.prototype;

// Multiple direction sets exist because any single fixed set can, for a particular shape-size
// pair, itself produce a degenerate seed tetrahedron. Odd/irrational-looking components (not just
// +-1) avoid a sparse hull's (e.g. an octahedron) support function tying between exactly-aligned
// vertices, which for two coincident identical hulls can collapse seed points to duplicates.
GJK.SEED_DIRECTION_SETS = [
    [[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]],
    [[1, 1, -1], [1, -1, 1], [-1, 1, 1], [-1, -1, -1]],
    [[1, 0, 0], [-1, 0.3, 0.3], [0, -1, 0.3], [0, 0.3, -1]],
    [[0.8763, 0.2451, 0.4127], [0.3312, -0.9021, 0.2734], [-0.6543, 0.1298, -0.7452], [-0.5532, -0.6789, 0.4821]]
];

// The 6 signed axes span every face normal of an axis-aligned contact, for the strict-interior
// probe below; the search direction itself is probed separately for oblique contacts.
GJK.INTERIOR_PROBE_DIRS = [
    [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]
];

// Tries each seed set, building a tetrahedron and checking whether it encloses the origin with a
// real margin. Returns { overlapping: true, simplex: this } on confirmed overlap, or
// { overlapping: false, direction, closest } from whichever seed's own reduction got closest to
// the origin, so the caller's incremental loop continues from the best available start.
proto._seedTetrahedron = function (support) {
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
    // Re-run the best set to restore its simplex/direction/closest as the live state.
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
};

// True iff the origin is strictly inside the Minkowski difference (real penetration), false if it
// merely lies on the boundary (exact touch). The support function's farthest extent along every
// direction must be strictly positive for strict interiority; at an exact touch there's a
// separating direction (the contact normal) where it's ~0.
proto._originStrictlyInside = function (support) {
    const margin = GJK.OVERLAP_DISTANCE_EPSILON;
    // The collapsed search direction is numerically along the contact normal - test it first, both signs.
    if (this._closest.lengthSquared() > 1e-20) {
        const l = Math.sqrt(this._closest.lengthSquared());
        this._probeDir.set(this._closest.x / l, this._closest.y / l, this._closest.z / l);
        if (!this._supportExceeds(support, this._probeDir, margin)) return false;
        this._probeDir.set(-this._probeDir.x, -this._probeDir.y, -this._probeDir.z);
        if (!this._supportExceeds(support, this._probeDir, margin)) return false;
    }
    for (let a = 0; a < GJK.INTERIOR_PROBE_DIRS.length; a++) {
        const d = GJK.INTERIOR_PROBE_DIRS[a];
        this._probeDir.set(d[0], d[1], d[2]);
        if (!this._supportExceeds(support, this._probeDir, margin)) return false;
    }
    return true;
};

// support(dir).dir > margin? (margin scaled by |dir| so the comparison is a true distance along dir.)
proto._supportExceeds = function (support, dir, margin) {
    support.supportInto(this._probeW, dir);
    const along = this._probeW.x * dir.x + this._probeW.y * dir.y + this._probeW.z * dir.z;
    const len = Math.sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    return along > margin * len;
};
