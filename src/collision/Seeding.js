// Tetrahedron seeding + strict-interior probe. Seeding from several diverse direction sets, rather
// than growing one incrementally, tells a real 3D overlap apart from an exact touch (incremental
// growth stalls in the touching plane).
var proto = GJK.prototype;

// Multiple sets because any single one can produce a degenerate seed for some shape-size pair. The
// irrational-looking components avoid support ties on sparse hulls (e.g. two coincident octahedra).
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

// Tries each seed set. Returns { overlapping: true } on a confirmed enclosing tetrahedron, else
// { overlapping: false, direction, closest } from whichever seed's reduction got closest to the origin.
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
        // A collapsed seed tetra (degenerate faces all skipped) can falsely report containsOrigin;
        // confirm with a support probe, else fall through to the incremental loop.
        if (result.containsOrigin) {
            if (this._originStrictlyInside(support)) return { overlapping: true, simplex: this };
            continue;
        }
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

// True iff the origin is strictly inside the Minkowski difference (real penetration) vs merely on
// its boundary (exact touch), tested by checking the support extent exceeds a margin in every
// probe direction.
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
