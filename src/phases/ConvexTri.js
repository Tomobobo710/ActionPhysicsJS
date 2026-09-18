const ConvexTri = {};

ConvexTri._isCurvedConvex = function (shape) {
    return (shape instanceof SphereShape) || (shape instanceof CylinderShape) ||
        (shape instanceof ConeShape) || (shape instanceof CapsuleShape);
};

ConvexTri.applies = function (placedA, placedB) {
    const aTri = placedA.shape instanceof TriangleShape;
    const bTri = placedB.shape instanceof TriangleShape;
    if (aTri === bTri) return false;
    return aTri ? ConvexTri._isCurvedConvex(placedB.shape) : ConvexTri._isCurvedConvex(placedA.shape);
};

ConvexTri.SEPARATION_LIMIT = 0.5;
ConvexTri.PENETRATION_LIMIT = 1.0;

// A support probe this far behind the triangle plane (as a share of the convex's smallest half-extent)
// is only a face contact if the deep point actually lies OVER the triangle. A shape resting beside a
// perpendicular side face pokes its girth past that face's plane while its deepest point projects
// onto the face's EDGE - reading the whole girth as penetration and flinging the body sideways. A
// prop genuinely wedged on a tile step (the perf heightfield) still has its deep point inside the
// triangle, so it keeps its face contact.
ConvexTri.MAX_PENETRATION_FRACTION = 1.0;
ConvexTri.DEEP_PENETRATION_FRACTION = 0.25;
ConvexTri.EDGE_INSIDE_MARGIN = 0.01;

ConvexTri._minHalfExtent = function (shape) {
    const aabb = ConvexTri._meAABB || (ConvexTri._meAABB = new AABB());
    shape.localAABBInto(aabb);
    return Math.min((aabb.max.x - aabb.min.x), (aabb.max.y - aabb.min.y), (aabb.max.z - aabb.min.z)) * 0.5;
};
ConvexTri.MIN_CULL_LIMIT = 0.05;

// See BoxTriFace.HINT_ALIGN_LIMIT: the hint normal is inherited from whichever mesh face was already
// in contact, so on a tile corner it can be PERPENDICULAR to the triangle being tested. Obeying a
// tangential hint ends up with refN (and therefore the support probe direction and the emitted
// normal) pointing the wrong way.
ConvexTri.HINT_ALIGN_LIMIT = 0.9;
ConvexTri.EDGE_SLACK = 0.06;
ConvexTri.AREA_EPSILON = 1e-12;
ConvexTri.DEDUPE_DIST_SQ = 1e-6;

ConvexTri.PROBE_COUNT = 4;
ConvexTri.PROBE_TILT = 0.5;

ConvexTri._PROBE_U = [ConvexTri.PROBE_TILT, 0, -ConvexTri.PROBE_TILT, 0];
ConvexTri._PROBE_V = [0, ConvexTri.PROBE_TILT, 0, -ConvexTri.PROBE_TILT];

ConvexTri._normalInto = function (out, a, b, c) {
    const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
    let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
    const lenSq = nx * nx + ny * ny + nz * nz;
    if (lenSq < ConvexTri.AREA_EPSILON) return false;
    const inv = 1 / Math.sqrt(lenSq);
    out.set(nx * inv, ny * inv, nz * inv);
    return true;
};

ConvexTri._pointInTri = function (hx, hy, hz, t0, t1, t2, tn, slack) {
    const e0x = t1.x - t0.x, e0y = t1.y - t0.y, e0z = t1.z - t0.z;
    const e1x = t2.x - t1.x, e1y = t2.y - t1.y, e1z = t2.z - t1.z;
    const e2x = t0.x - t2.x, e2y = t0.y - t2.y, e2z = t0.z - t2.z;
    const c0x = hx - t0.x, c0y = hy - t0.y, c0z = hz - t0.z;
    const c1x = hx - t1.x, c1y = hy - t1.y, c1z = hz - t1.z;
    const c2x = hx - t2.x, c2y = hy - t2.y, c2z = hz - t2.z;

    const s0 = tn.x * (e0y * c0z - e0z * c0y) + tn.y * (e0z * c0x - e0x * c0z) + tn.z * (e0x * c0y - e0y * c0x);
    const s1 = tn.x * (e1y * c1z - e1z * c1y) + tn.y * (e1z * c1x - e1x * c1z) + tn.z * (e1x * c1y - e1y * c1x);
    const s2 = tn.x * (e2y * c2z - e2z * c2y) + tn.y * (e2z * c2x - e2x * c2z) + tn.z * (e2x * c2y - e2y * c2x);
    const l0 = Math.sqrt(e0x * e0x + e0y * e0y + e0z * e0z) || 1;
    const l1 = Math.sqrt(e1x * e1x + e1y * e1y + e1z * e1z) || 1;
    const l2 = Math.sqrt(e2x * e2x + e2y * e2y + e2z * e2z) || 1;

    const d0 = s0 / l0, d1 = s1 / l1, d2 = s2 / l2;

    const outPos = Math.max(-d0, -d1, -d2);
    const outNeg = Math.max(d0, d1, d2);
    ConvexTri._lastOutside = Math.min(outPos, outNeg);
    return ConvexTri._lastOutside <= slack;
};

ConvexTri.test = function (placedA, placedB, out, nextContact, hintNormalBToA, margin) {
    const aIsTri = placedA.shape instanceof TriangleShape;
    const triPlaced = aIsTri ? placedA : placedB;
    const cvxPlaced = aIsTri ? placedB : placedA;
    const tri = triPlaced.shape;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    const refN = ConvexTri._refN;
    if (!ConvexTri._normalInto(refN, t0, t1, t2)) return null;
    let oriented = false;
    if (hintNormalBToA) {

        const dot = refN.x * hintNormalBToA.x + refN.y * hintNormalBToA.y + refN.z * hintNormalBToA.z;
        if (Math.abs(dot) >= ConvexTri.HINT_ALIGN_LIMIT) {
            if ((aIsTri && dot > 0) || (!aIsTri && dot < 0)) refN.scaleInPlace(-1);
            oriented = true;
        }
    }
    if (!oriented) {
        const cvxPos = cvxPlaced.position;
        if ((cvxPos.x - t0.x) * refN.x + (cvxPos.y - t0.y) * refN.y + (cvxPos.z - t0.z) * refN.z < 0) {
            refN.scaleInPlace(-1);
        }
    }

    ConvexTri.lastDeepestInTriangle = false;
    ConvexTri.lastDeepestOutsideDist = Infinity;

    ConvexTri.lastVerdict = 'maybe';

    const probeDir = ConvexTri._probeDir;
    const dp = ConvexTri._dp;
    const invRot = ConvexTri._invRot.copy(cvxPlaced.rotation).invert();
    const scratchDir = ConvexTri._scratchDir;

    const cullLimit = (margin === undefined || margin === null)
        ? ConvexTri.SEPARATION_LIMIT
        : Math.min(ConvexTri.SEPARATION_LIMIT, Math.max(margin, ConvexTri.MIN_CULL_LIMIT));

    const penetrationLimit = ConvexTri.PENETRATION_LIMIT;

    probeDir.set(-refN.x, -refN.y, -refN.z);
    MinkowskiSupport.supportOfInto(dp, cvxPlaced, invRot, probeDir, scratchDir);
    const gap = (dp.x - t0.x) * refN.x + (dp.y - t0.y) * refN.y + (dp.z - t0.z) * refN.z;
    if (gap > ConvexTri.SEPARATION_LIMIT) {

        ConvexTri.lastVerdict = 'separated';
        return null;
    }
    if (gap < -penetrationLimit) return null;
    // Deep penetration is only a face contact when the deep point is genuinely over the triangle.
    // If it projects onto (or outside) an edge, the true contact is edge/corner - hand it to GJK/EPA.
    if (gap < -ConvexTri.DEEP_PENETRATION_FRACTION * ConvexTri._minHalfExtent(cvxPlaced.shape)) {
        ConvexTri._pointInTri(dp.x - gap * refN.x, dp.y - gap * refN.y, dp.z - gap * refN.z, t0, t1, t2, refN, 0);
        if (ConvexTri._lastOutside > -ConvexTri.EDGE_INSIDE_MARGIN) return null;
    }

    const cand = ConvexTri._cand;
    let nCand = 0;
    cand[nCand++].copy(dp);

    const tanU = ConvexTri._tanU, tanV = ConvexTri._tanV;
    const ax = Math.abs(refN.x), ay = Math.abs(refN.y), az = Math.abs(refN.z);
    if (ax <= ay && ax <= az) tanU.set(0, -refN.z, refN.y);
    else if (ay <= az) tanU.set(-refN.z, 0, refN.x);
    else tanU.set(-refN.y, refN.x, 0);
    const tl = Math.sqrt(tanU.x * tanU.x + tanU.y * tanU.y + tanU.z * tanU.z) || 1;
    tanU.scaleInPlace(1 / tl);
    tanV.set(refN.y * tanU.z - refN.z * tanU.y,
        refN.z * tanU.x - refN.x * tanU.z,
        refN.x * tanU.y - refN.y * tanU.x);

    const probeCount = (cvxPlaced.shape instanceof SphereShape) ? 0 : ConvexTri.PROBE_COUNT;
    for (let e = 0; e < probeCount; e++) {
        const cu = ConvexTri._PROBE_U[e], cv = ConvexTri._PROBE_V[e];
        probeDir.set(-refN.x + tanU.x * cu + tanV.x * cv,
            -refN.y + tanU.y * cu + tanV.y * cv,
            -refN.z + tanU.z * cu + tanV.z * cv);
        const pl = Math.sqrt(probeDir.x * probeDir.x + probeDir.y * probeDir.y + probeDir.z * probeDir.z) || 1;
        probeDir.scaleInPlace(1 / pl);
        MinkowskiSupport.supportOfInto(cand[nCand], cvxPlaced, invRot, probeDir, scratchDir);
        nCand++;
    }

    const cvxIsA = !aIsTri;
    const normX = cvxIsA ? refN.x : -refN.x;
    const normY = cvxIsA ? refN.y : -refN.y;
    const normZ = cvxIsA ? refN.z : -refN.z;

    let emitted = 0;
    for (let i = 0; i < nCand; i++) {
        const p = cand[i];
        const g = (p.x - t0.x) * refN.x + (p.y - t0.y) * refN.y + (p.z - t0.z) * refN.z;

        if (i === 0 && (g > ConvexTri.SEPARATION_LIMIT || g < -penetrationLimit)) {
            ConvexTri.lastDeepestOutsideDist = Infinity;
        }
        if (g > ConvexTri.SEPARATION_LIMIT || g < -penetrationLimit) continue;

        const projX = p.x - g * refN.x, projY = p.y - g * refN.y, projZ = p.z - g * refN.z;
        const inTri = ConvexTri._pointInTri(projX, projY, projZ, t0, t1, t2, refN, ConvexTri.EDGE_SLACK);
        if (i === 0) {
            ConvexTri.lastDeepestOutsideDist = ConvexTri._lastOutside;
            if (inTri) ConvexTri.lastDeepestInTriangle = true;
        }
        if (!inTri) continue;

        let dup = false;
        for (let j = 0; j < emitted; j++) {
            const q = out[out.length - 1 - j];
            const ddx = q.pointOnA.x - p.x, ddy = q.pointOnA.y - p.y, ddz = q.pointOnA.z - p.z;
            if (ddx * ddx + ddy * ddy + ddz * ddz < ConvexTri.DEDUPE_DIST_SQ) { dup = true; break; }
        }
        if (dup) continue;

        const contact = nextContact();

        if (aIsTri) {
            contact.pointOnB.set(p.x, p.y, p.z);
            contact.pointOnA.set(projX, projY, projZ);
        } else {
            contact.pointOnA.set(p.x, p.y, p.z);
            contact.pointOnB.set(projX, projY, projZ);
        }
        contact.normal.set(normX, normY, normZ);
        contact.signedDistance = -g;
        contact.fromMeshFace = true;
        contact.fromCurvedTri = true;
        contact.setMeshTriangle(t0, t1, t2, triPlaced.bodyCenter, aIsTri);
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
        emitted++;
    }

    if (emitted === 0) {

        const outside = ConvexTri.lastDeepestOutsideDist;
        const along = gap > 0 ? gap : 0;
        // A sphere is sampled at ONE point - its support point along -refN - so the distance from that
        // point out to the triangle is not a distance from the SPHERE to the triangle. A sphere perched
        // on a mesh VERTEX has that support point projecting up-slope past the apex, a radius outside
        // the triangle it is touching, so measuring the point alone declares every triangle at a vertex
        // 'separated' - and a 'separated' verdict skips the GJK/EPA fallback too, so the pair reports no
        // contact at all while the sphere is touching. The next tick then finds it a whole tick's fall
        // into the mesh and pops it out (a sphere dropped on an apex left at 13.8 m/s from a 3.5 m/s
        // arrival). Subtracting the sphere's own radius back off is a real lower bound on the true
        // shape-to-triangle distance, so a sphere near a vertex is measured honestly and handed to
        // GJK/EPA, while a triangle genuinely far away is still culled.
        const reach = (cvxPlaced.shape instanceof SphereShape) ? cvxPlaced.shape.radius : 0;
        if (isFinite(outside) && Math.sqrt(along * along + outside * outside) - reach > cullLimit) {
            ConvexTri.lastVerdict = 'separated';
        }
        return null;
    }
    return out;
};

ConvexTri._refN = new Vector3();
ConvexTri._probeDir = new Vector3();
ConvexTri._scratchDir = new Vector3();
ConvexTri._invRot = new Quaternion();
ConvexTri._dp = new Vector3();
ConvexTri._edgeMid = new Vector3();
ConvexTri._tanU = new Vector3();
ConvexTri._tanV = new Vector3();

ConvexTri._cand = [];
for (var _ci = 0; _ci <= ConvexTri.PROBE_COUNT; _ci++) ConvexTri._cand.push(new Vector3());
ConvexTri.lastDeepestInTriangle = false;
ConvexTri.lastDeepestOutsideDist = Infinity;
ConvexTri._lastOutside = 0;
ConvexTri.lastVerdict = 'maybe';

ConvexTri.REFRESH_DRIFT_TOLERANCE = 0.35;

// Closed-form contact for a cylinder/cone CAP resting on a mesh triangle. ConvexTri's probe cloud is
// fine for a curved line (a cylinder on its side) but is the wrong shape for a flat cap: its points
// are probe samples of the rim, not the real cap-overlap polygon, so the solver cannot tell a
// supported cap from an overhanging one. When the cap normal genuinely faces the triangle, clip the
// cap polygon to the triangle instead - the same true patch BoxTriFace gives a box - and the
// solver's support test (which keys off a real patch extent) can tip an overhang.
const CapTriFace = {};

CapTriFace.applies = function (placedA, placedB) {
    const aTri = placedA.shape instanceof TriangleShape;
    const bTri = placedB.shape instanceof TriangleShape;
    if (aTri === bTri) return false;
    const other = aTri ? placedB.shape : placedA.shape;
    return (other instanceof CylinderShape) || (other instanceof ConeShape) || (other instanceof ConvexShape);
};

CapTriFace.ALIGN_LIMIT = 0.90;
CapTriFace.SEPARATION_LIMIT = 0.5;
CapTriFace.PENETRATION_LIMIT = 1.0;
CapTriFace.MAX_PENETRATION_FRACTION = 0.5;
CapTriFace.EDGE_SLACK = 0.02;
CapTriFace.DEDUPE_DIST_SQ = 1e-8;
CapTriFace.MIN_OVERLAP_COVERAGE = 1e-4;
CapTriFace.SIDES = 12;

CapTriFace.test = function (placedA, placedB, out, nextContact, hintNormalBToA, margin) {
    const aIsTri = placedA.shape instanceof TriangleShape;
    const triPlaced = aIsTri ? placedA : placedB;
    const capPlaced = aIsTri ? placedB : placedA;
    const shape = capPlaced.shape;
    const tri = triPlaced.shape;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    const refN = CapTriFace._refN;
    if (!ConvexTri._normalInto(refN, t0, t1, t2)) return null;
    let oriented = false;
    if (hintNormalBToA) {
        const d = refN.x * hintNormalBToA.x + refN.y * hintNormalBToA.y + refN.z * hintNormalBToA.z;
        if (Math.abs(d) >= BoxTriFace.HINT_ALIGN_LIMIT) {
            if ((aIsTri && d > 0) || (!aIsTri && d < 0)) refN.scaleInPlace(-1);
            oriented = true;
        }
    }
    if (!oriented) {
        const c = capPlaced.position;
        if ((c.x - t0.x) * refN.x + (c.y - t0.y) * refN.y + (c.z - t0.z) * refN.z < 0) refN.scaleInPlace(-1);
    }

    let poly, polyN, capArea;
    if (shape instanceof CylinderShape || shape instanceof ConeShape) {
        const axis = CapTriFace._axis.set(0, 1, 0);
        capPlaced.rotation.transformVectorInPlace(axis);
        const d = axis.x * refN.x + axis.y * refN.y + axis.z * refN.z;

        // Cap normal must genuinely oppose refN (point at the triangle). A cone's only cap is its
        // base (normal -axis); a cylinder tests both caps.
        let sign;
        if (shape instanceof ConeShape) {
            sign = -1;
            if (d < CapTriFace.ALIGN_LIMIT) return null;
        } else {
            if (-d >= CapTriFace.ALIGN_LIMIT) sign = 1;
            else if (d >= CapTriFace.ALIGN_LIMIT) sign = -1;
            else return null;
        }

        const u = CapTriFace._u.set(1, 0, 0);
        const v = CapTriFace._v.set(0, 0, 1);
        capPlaced.rotation.transformVectorInPlace(u);
        capPlaced.rotation.transformVectorInPlace(v);
        const pos = capPlaced.position;
        const cx = pos.x + axis.x * shape.halfHeight * sign;
        const cy = pos.y + axis.y * shape.halfHeight * sign;
        const cz = pos.z + axis.z * shape.halfHeight * sign;

        poly = CapTriFace._poly;
        polyN = CapTriFace.SIDES;
        for (let i = 0; i < polyN; i++) {
            const ang = 2 * Scalar.PI * i / polyN;
            const cs = Scalar.cos(ang) * shape.radius, sn = Scalar.sin(ang) * shape.radius;
            poly[i].set(
                cx + u.x * cs + v.x * sn,
                cy + u.y * cs + v.y * sn,
                cz + u.z * cs + v.z * sn
            );
        }
        capArea = Scalar.PI * shape.radius * shape.radius;
    } else {
        // A flat-faced hull: pick its face that most opposes refN (i.e. faces the triangle).
        const faces = PolyClip.facesOf(capPlaced);
        if (!faces) return null;
        let best = null, bestDot = -Infinity;
        for (let i = 0; i < faces.length; i++) {
            const f = faces[i];
            const dd = -(f.normal.x * refN.x + f.normal.y * refN.y + f.normal.z * refN.z);
            if (dd > bestDot) { bestDot = dd; best = f; }
        }
        if (!best || bestDot < CapTriFace.ALIGN_LIMIT) return null;
        poly = best.verts;
        polyN = poly.length;
        if (polyN < 3 || polyN > CapTriFace._poly.length) return null;
        capArea = BoxTriFace._polyArea(poly, polyN, refN);
    }

    const triVerts = CapTriFace._triVerts;
    triVerts[0] = t0; triVerts[1] = t1; triVerts[2] = t2;

    if (!BoxTriFace._clipToTriangle(poly, polyN, CapTriFace._bufA, CapTriFace._bufB, triVerts, refN, 0)) return null;
    const trueArea = BoxTriFace._polyArea(BoxTriFace._clipOut, BoxTriFace._clipOutN, refN);
    if (trueArea <= capArea * CapTriFace.MIN_OVERLAP_COVERAGE) return null;

    if (!BoxTriFace._clipToTriangle(poly, polyN, CapTriFace._bufA, CapTriFace._bufB, triVerts, refN, CapTriFace.EDGE_SLACK)) return null;
    const src = BoxTriFace._clipOut, srcN = BoxTriFace._clipOutN;

    const shapeScale = (shape instanceof CylinderShape || shape instanceof ConeShape)
        ? Math.min(shape.radius, shape.halfHeight) : ConvexTri._minHalfExtent(shape);
    const penetrationLimit = Math.min(CapTriFace.PENETRATION_LIMIT,
        CapTriFace.MAX_PENETRATION_FRACTION * shapeScale);
    const cvxIsA = !aIsTri;
    const normX = cvxIsA ? refN.x : -refN.x;
    const normY = cvxIsA ? refN.y : -refN.y;
    const normZ = cvxIsA ? refN.z : -refN.z;

    let emitted = 0;
    for (let i = 0; i < srcN; i++) {
        const p = src[i];
        const g = (p.x - t0.x) * refN.x + (p.y - t0.y) * refN.y + (p.z - t0.z) * refN.z;
        if (g > CapTriFace.SEPARATION_LIMIT || g < -penetrationLimit) continue;
        const projX = p.x - g * refN.x, projY = p.y - g * refN.y, projZ = p.z - g * refN.z;

        let dup = false;
        for (let j = 0; j < emitted; j++) {
            const q = out[out.length - 1 - j];
            const qa = cvxIsA ? q.pointOnA : q.pointOnB;
            const dx = qa.x - p.x, dy = qa.y - p.y, dz = qa.z - p.z;
            if (dx * dx + dy * dy + dz * dz < CapTriFace.DEDUPE_DIST_SQ) { dup = true; break; }
        }
        if (dup) continue;

        const contact = nextContact();
        if (aIsTri) {
            contact.pointOnB.set(p.x, p.y, p.z);
            contact.pointOnA.set(projX, projY, projZ);
        } else {
            contact.pointOnA.set(p.x, p.y, p.z);
            contact.pointOnB.set(projX, projY, projZ);
        }
        contact.normal.set(normX, normY, normZ);
        contact.signedDistance = -g;
        contact.fromMeshFace = true;
        contact.fromFacePatch = true;
        contact.setMeshTriangle(t0, t1, t2, triPlaced.bodyCenter, aIsTri);
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
        emitted++;
    }

    if (emitted === 0) return null;
    return out;
};

CapTriFace._refN = new Vector3();
CapTriFace._axis = new Vector3();
CapTriFace._u = new Vector3();
CapTriFace._v = new Vector3();
CapTriFace._triVerts = [null, null, null];
CapTriFace._poly = [];
CapTriFace._bufA = [];
CapTriFace._bufB = [];
for (var _ctfI = 0; _ctfI < 24; _ctfI++) {
    CapTriFace._poly.push(new Vector3());
    CapTriFace._bufA.push(new Vector3());
    CapTriFace._bufB.push(new Vector3());
}

ActionPhysics.CapTriFace = CapTriFace;
ActionPhysics.ConvexTri = ConvexTri;
