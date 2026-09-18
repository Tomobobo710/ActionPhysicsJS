var proto = GJK.prototype;

GJK.SEED_DIRECTION_SETS = [
    [[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]],
    [[1, 1, -1], [1, -1, 1], [-1, 1, 1], [-1, -1, -1]],
    [[1, 0, 0], [-1, 0.3, 0.3], [0, -1, 0.3], [0, 0.3, -1]],
    [[0.8763, 0.2451, 0.4127], [0.3312, -0.9021, 0.2734], [-0.6543, 0.1298, -0.7452], [-0.5532, -0.6789, 0.4821]]
];

GJK.INTERIOR_PROBE_DIRS = [
    [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]
];

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

        if (result.containsOrigin) {
            if (this._originStrictlyInside(support)) return { overlapping: true, simplex: this };
            continue;
        }
        const distSq = result.closest.x * result.closest.x + result.closest.y * result.closest.y + result.closest.z * result.closest.z;
        if (distSq < bestDistSq) { bestDistSq = distSq; bestSet = s; }
    }

    if (bestSet === -1) {
        this._clear();
        const touchDirs = GJK.SEED_DIRECTION_SETS[0];
        for (let i = 0; i < 4; i++) {
            const d = touchDirs[i];
            this._newDir4.set(d[0], d[1], d[2]);
            support.supportInto(this._newW, this._newDir4, this._newA, this._newB);
            this._push(this._newW, this._newA, this._newB);
        }
        return { overlapping: false, direction: new Vector3(0, 0, 0), closest: new Vector3(0, 0, 0) };
    }

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

proto._originStrictlyInside = function (support) {
    const margin = GJK.OVERLAP_DISTANCE_EPSILON;

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

proto._supportExceeds = function (support, dir, margin) {
    support.supportInto(this._probeW, dir);
    const along = this._probeW.x * dir.x + this._probeW.y * dir.y + this._probeW.z * dir.z;
    const len = Math.sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    return along > margin * len;
};
