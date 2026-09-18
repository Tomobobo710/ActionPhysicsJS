class ConvexShape extends Shape {

    constructor(points) {
        super('convex');
        this.points = points;
        this._hullFaces = null;
        this._massData = null;
        this._polyFaces = null;
    }

    get polyFaces() {
        if (this._polyFaces) return this._polyFaces;
        this._polyFaces = ConvexShape._mergeCoplanar(this._hull(), this.points);
        return this._polyFaces;
    }

    get faces() {
        if (this._facesView) return this._facesView;
        const hull = this._hull();
        const pts = this.points;
        this._facesView = hull.map(function (tri) {
            return {
                a: { point: pts[tri[0]] },
                b: { point: pts[tri[1]] },
                c: { point: pts[tri[2]] }
            };
        });
        return this._facesView;
    }

    supportInto(out, direction) {
        const pts = this.points;
        let bestDot = -Infinity, bestIndex = 0;
        for (let i = 0; i < pts.length; i++) {
            const d = pts[i].x * direction.x + pts[i].y * direction.y + pts[i].z * direction.z;
            if (d > bestDot) { bestDot = d; bestIndex = i; }
        }
        out.x = pts[bestIndex].x; out.y = pts[bestIndex].y; out.z = pts[bestIndex].z;
        return out;
    }

    localAABBInto(out) {
        out.setEmpty();
        const pts = this.points;
        for (let i = 0; i < pts.length; i++) {
            const p = pts[i];
            if (p.x < out.min.x) out.min.x = p.x;
            if (p.y < out.min.y) out.min.y = p.y;
            if (p.z < out.min.z) out.min.z = p.z;
            if (p.x > out.max.x) out.max.x = p.x;
            if (p.y > out.max.y) out.max.y = p.y;
            if (p.z > out.max.z) out.max.z = p.z;
        }
        return out;
    }

    volume() {
        return this._computeMassData().mass;
    }

    computeMassData() {
        const m = this._computeMassData();
        return {
            mass: m.mass,
            inertia: new Matrix3().copy(m.inertia),
            centerOfMass: new Vector3(m.centerOfMass.x, m.centerOfMass.y, m.centerOfMass.z)
        };
    }

    _computeMassData() {
        if (this._massData) return this._massData;
        const faces = this._hull();

        let volume = 0;
        const comAccum = new Vector3(0, 0, 0);
        let Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Ixz = 0, Iyz = 0;

        const pts = this.points;
        for (let f = 0; f < faces.length; f++) {
            const a = pts[faces[f][0]], b = pts[faces[f][1]], c = pts[faces[f][2]];

            const cx = b.y * c.z - b.z * c.y, cy = b.z * c.x - b.x * c.z, cz = b.x * c.y - b.y * c.x;
            const tetVol = (a.x * cx + a.y * cy + a.z * cz) / 6;
            volume += tetVol;

            comAccum.x += tetVol * (a.x + b.x + c.x) / 4;
            comAccum.y += tetVol * (a.y + b.y + c.y) / 4;
            comAccum.z += tetVol * (a.z + b.z + c.z) / 4;

            const sx2 = a.x * a.x + b.x * b.x + c.x * c.x, sy2 = a.y * a.y + b.y * b.y + c.y * c.y, sz2 = a.z * a.z + b.z * b.z + c.z * c.z;
            const sx = a.x + b.x + c.x, sy = a.y + b.y + c.y, sz = a.z + b.z + c.z;
            const sxy = a.x * a.y + b.x * b.y + c.x * c.y;
            const sxz = a.x * a.z + b.x * b.z + c.x * c.z;
            const syz = a.y * a.z + b.y * b.z + c.y * c.z;
            const k = tetVol / 20;
            const ix2 = k * (sx2 + sx * sx), iy2 = k * (sy2 + sy * sy), iz2 = k * (sz2 + sz * sz);
            Ixx += iy2 + iz2;
            Iyy += ix2 + iz2;
            Izz += ix2 + iy2;
            Ixy += k * (sxy + sx * sy);
            Ixz += k * (sxz + sx * sz);
            Iyz += k * (syz + sy * sz);
        }

        volume = Math.abs(volume);
        const com = volume > 0 ? new Vector3(comAccum.x / volume, comAccum.y / volume, comAccum.z / volume) : new Vector3(0, 0, 0);

        const cx = com.x, cy = com.y, cz = com.z;
        const IxxC = Math.abs(Ixx - volume * (cy * cy + cz * cz));
        const IyyC = Math.abs(Iyy - volume * (cx * cx + cz * cz));
        const IzzC = Math.abs(Izz - volume * (cx * cx + cy * cy));
        const IxyC = Ixy - volume * cx * cy;
        const IxzC = Ixz - volume * cx * cz;
        const IyzC = Iyz - volume * cy * cz;

        const inertia = new Matrix3();
        inertia.e00 = IxxC; inertia.e01 = -IxyC; inertia.e02 = -IxzC;
        inertia.e10 = -IxyC; inertia.e11 = IyyC; inertia.e12 = -IyzC;
        inertia.e20 = -IxzC; inertia.e21 = -IyzC; inertia.e22 = IzzC;

        this._massData = { mass: volume, inertia: inertia, centerOfMass: com };
        return this._massData;
    }

    _hull() {
        if (this._hullFaces) return this._hullFaces;
        const pts = this.points;
        if (pts.length < 4) { this._hullFaces = []; return this._hullFaces; }

        const seed = ConvexShape._seedTetrahedron(pts);
        let faces = seed.faces;
        for (let i = 0; i < pts.length; i++) {
            if (seed.used.has(i)) continue;
            ConvexShape._assignToOutsideSet(faces, pts, i);
        }

        while (true) {
            let face = null;
            for (let f = 0; f < faces.length; f++) if (faces[f].outside.length > 0) { face = faces[f]; break; }
            if (!face) break;

            let farIdx = -1, farDist = -Infinity;
            for (let k = 0; k < face.outside.length; k++) {
                const idx = face.outside[k];
                const d = ConvexShape._planeDistance(pts, face, idx);
                if (d > farDist) { farDist = d; farIdx = idx; }
            }

            const visible = [];
            for (let f = 0; f < faces.length; f++) {
                if (ConvexShape._planeDistance(pts, faces[f], farIdx) > 1e-9) visible.push(f);
            }

            const visibleSet = new Set(visible);
            const edgeCount = new Map();
            for (let vi = 0; vi < visible.length; vi++) {
                const fc = faces[visible[vi]];
                ConvexShape._forEachEdge(fc, function (a, b) {
                    const key = a < b ? a + ':' + b : b + ':' + a;
                    let e = edgeCount.get(key);
                    if (!e) { e = { count: 0, a: a, b: b }; edgeCount.set(key, e); }
                    e.count++;
                });
            }
            const horizon = [];
            edgeCount.forEach(function (e) { if (e.count === 1) horizon.push([e.a, e.b]); });

            let orphanPool = [];
            for (let vi = 0; vi < visible.length; vi++) orphanPool = orphanPool.concat(faces[visible[vi]].outside);

            const sortedVisible = visible.slice().sort(function (x, y) { return y - x; });
            for (let vi = 0; vi < sortedVisible.length; vi++) faces.splice(sortedVisible[vi], 1);

            const newFaces = [];
            for (let h = 0; h < horizon.length; h++) {
                const nf = ConvexShape._makeFace(pts, horizon[h][0], horizon[h][1], farIdx, seed.centroid);
                newFaces.push(nf);
            }

            for (let o = 0; o < orphanPool.length; o++) {
                if (orphanPool[o] === farIdx) continue;
                ConvexShape._assignToOutsideSet(newFaces, pts, orphanPool[o]);
            }

            faces = faces.concat(newFaces);
        }

        this._hullFaces = faces.map(function (f) { return [f.a, f.b, f.c]; });
        return this._hullFaces;
    }

    static _seedTetrahedron(pts) {
        let minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;
        for (let i = 1; i < pts.length; i++) {
            if (pts[i].x < pts[minX].x) minX = i; if (pts[i].x > pts[maxX].x) maxX = i;
            if (pts[i].y < pts[minY].y) minY = i; if (pts[i].y > pts[maxY].y) maxY = i;
            if (pts[i].z < pts[minZ].z) minZ = i; if (pts[i].z > pts[maxZ].z) maxZ = i;
        }
        const candidates = [minX, maxX, minY, maxY, minZ, maxZ];

        let ia = candidates[0], ib = candidates[1], bestD = -Infinity;
        for (let i = 0; i < candidates.length; i++) {
            for (let j = i + 1; j < candidates.length; j++) {
                const d = pts[candidates[i]].distanceSquared(pts[candidates[j]]);
                if (d > bestD) { bestD = d; ia = candidates[i]; ib = candidates[j]; }
            }
        }

        let ic = -1, bestLineD = -Infinity;
        for (let i = 0; i < pts.length; i++) {
            if (i === ia || i === ib) continue;
            const d = ConvexShape._pointLineDistanceSquared(pts[i], pts[ia], pts[ib]);
            if (d > bestLineD) { bestLineD = d; ic = i; }
        }

        const normal = ConvexShape._faceNormal(pts[ia], pts[ib], pts[ic]);
        let id = -1, bestPlaneD = -Infinity;
        for (let i = 0; i < pts.length; i++) {
            if (i === ia || i === ib || i === ic) continue;
            const dx = pts[i].x - pts[ia].x, dy = pts[i].y - pts[ia].y, dz = pts[i].z - pts[ia].z;
            const d = Math.abs(dx * normal.x + dy * normal.y + dz * normal.z);
            if (d > bestPlaneD) { bestPlaneD = d; id = i; }
        }

        const centroid = new Vector3(
            (pts[ia].x + pts[ib].x + pts[ic].x + pts[id].x) / 4,
            (pts[ia].y + pts[ib].y + pts[ic].y + pts[id].y) / 4,
            (pts[ia].z + pts[ib].z + pts[ic].z + pts[id].z) / 4
        );

        const faces = [
            ConvexShape._makeFace(pts, ia, ib, ic, centroid),
            ConvexShape._makeFace(pts, ia, ib, id, centroid),
            ConvexShape._makeFace(pts, ia, ic, id, centroid),
            ConvexShape._makeFace(pts, ib, ic, id, centroid)
        ];

        const used = new Set([ia, ib, ic, id]);
        return { faces: faces, used: used, centroid: centroid };
    }

    static _makeFace(pts, i0, i1, i2, insidePoint) {
        const normal = ConvexShape._faceNormal(pts[i0], pts[i1], pts[i2]);
        const toInside = insidePoint.x * normal.x + insidePoint.y * normal.y + insidePoint.z * normal.z
            - (pts[i0].x * normal.x + pts[i0].y * normal.y + pts[i0].z * normal.z);
        if (toInside > 0) return { a: i0, b: i2, c: i1, outside: [] };
        return { a: i0, b: i1, c: i2, outside: [] };
    }

    static _faceNormal(a, b, c) {
        const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
        const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
        return new Vector3(aby * acz - abz * acy, abz * acx - abx * acz, abx * acy - aby * acx);
    }

    static _planeDistance(pts, face, idx) {
        const a = pts[face.a], b = pts[face.b], c = pts[face.c], p = pts[idx];
        const n = ConvexShape._faceNormal(a, b, c);
        const len = Math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        if (len < 1e-12) return -Infinity;
        return ((p.x - a.x) * n.x + (p.y - a.y) * n.y + (p.z - a.z) * n.z) / len;
    }

    static _assignToOutsideSet(faces, pts, idx) {
        let bestFace = null, bestDist = 1e-9;
        for (let f = 0; f < faces.length; f++) {
            const d = ConvexShape._planeDistance(pts, faces[f], idx);
            if (d > bestDist) { bestDist = d; bestFace = faces[f]; }
        }
        if (bestFace) bestFace.outside.push(idx);
    }

    static _forEachEdge(face, fn) {
        fn(face.a, face.b);
        fn(face.b, face.c);
        fn(face.c, face.a);
    }

    static _pointLineDistanceSquared(p, a, b) {
        const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
        const apx = p.x - a.x, apy = p.y - a.y, apz = p.z - a.z;
        const abLenSq = abx * abx + aby * aby + abz * abz;
        if (abLenSq < 1e-12) return apx * apx + apy * apy + apz * apz;
        const t = (apx * abx + apy * aby + apz * abz) / abLenSq;
        const cx = a.x + t * abx, cy = a.y + t * aby, cz = a.z + t * abz;
        const dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
        return dx * dx + dy * dy + dz * dz;
    }
}

ConvexShape.COPLANAR_DOT = 0.9999;

ConvexShape._mergeCoplanar = function (tris, pts) {
    const faces = [];
    if (!tris || tris.length === 0) return faces;

    const normals = [];
    for (let i = 0; i < tris.length; i++) {
        const t = tris[i];
        const a = pts[t[0]], b = pts[t[1]], c = pts[t[2]];
        const ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        const vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const len = Math.sqrt(nx * nx + ny * ny + nz * nz);
        normals.push(len < 1e-12 ? null : new Vector3(nx / len, ny / len, nz / len));
    }

    const claimed = new Array(tris.length).fill(false);
    for (let i = 0; i < tris.length; i++) {
        if (claimed[i] || !normals[i]) continue;
        const n = normals[i];
        const group = [i];
        claimed[i] = true;
        for (let j = i + 1; j < tris.length; j++) {
            if (claimed[j] || !normals[j]) continue;
            if (n.dot(normals[j]) >= ConvexShape.COPLANAR_DOT) { claimed[j] = true; group.push(j); }
        }

        const edges = new Map();
        for (let g = 0; g < group.length; g++) {
            const t = tris[group[g]];
            for (let e = 0; e < 3; e++) {
                const from = t[e], to = t[(e + 1) % 3];
                edges.set(from + ':' + to, { from: from, to: to });
            }
        }
        const boundary = new Map();
        for (const [key, edge] of edges) {
            if (edges.has(edge.to + ':' + edge.from)) continue;
            if (boundary.has(edge.from)) { boundary.clear(); break; }
            boundary.set(edge.from, edge.to);
        }
        if (boundary.size < 3) continue;

        const start = boundary.keys().next().value;
        const indices = [];
        let cur = start;
        let guard = boundary.size + 1;
        while (guard-- > 0) {
            indices.push(cur);
            const next = boundary.get(cur);
            if (next === undefined) { indices.length = 0; break; }
            cur = next;
            if (cur === start) break;
        }
        if (indices.length !== boundary.size) continue;

        faces.push({ normal: n, indices: indices });
    }
    return faces;
};

ActionPhysics.ConvexShape = ConvexShape;
