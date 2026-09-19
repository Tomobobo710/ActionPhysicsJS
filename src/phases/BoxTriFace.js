const BoxTriFace = {};

BoxTriFace.applies = function (placedA, placedB) {
    const aTri = placedA.shape instanceof TriangleShape;
    const bTri = placedB.shape instanceof TriangleShape;
    if (aTri === bTri) return false;

    const other = aTri ? placedB.shape : placedA.shape;
    return other instanceof BoxShape;
};

BoxTriFace.PARALLEL_COS_LIMIT = 0.90;

BoxTriFace.SEPARATION_LIMIT = 0.5;

BoxTriFace.PENETRATION_LIMIT = 1.0;
BoxTriFace.MAX_PENETRATION_FRACTION = 0.25;

// A hint normal may only orient this triangle when it is genuinely this face's normal. The hint is
// carried over from whichever mesh face was already in contact, so on a tile corner it can belong to a
// PERPENDICULAR face; obeying a tangential one left refN pointing the wrong way, which measures the
// box's whole thickness as penetration instead of its real overlap and shoves it off the tile.
BoxTriFace.HINT_ALIGN_LIMIT = 0.9;

BoxTriFace.EDGE_SLACK = 0.02;
BoxTriFace.DEDUPE_DIST_SQ = 1e-8;

BoxTriFace.MAX_FACE_AREA_RATIO = 4;

// Share of the box face that must genuinely overlap the triangle (slack-free) for this to count as a
// face patch rather than an edge-touching sliver. See _clipToTriangle's second, slack-free pass.
BoxTriFace.MIN_OVERLAP_COVERAGE = 1e-4;

BoxTriFace.lastVerdict = 'maybe';

BoxTriFace.lastFaceCoverage = 0;
BoxTriFace.lastBestDot = 0;

BoxTriFace.test = function (placedA, placedB, out, nextContact, hintNormalBToA, margin) {
    BoxTriFace.lastVerdict = 'maybe';
    BoxTriFace.lastFaceCoverage = 0;
BoxTriFace.lastBestDot = 0;
    const aIsTri = placedA.shape instanceof TriangleShape;
    const triPlaced = aIsTri ? placedA : placedB;
    const boxPlaced = aIsTri ? placedB : placedA;
    const tri = triPlaced.shape;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    const refN = BoxTriFace._refN;
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
        const c = boxPlaced.position;
        if ((c.x - t0.x) * refN.x + (c.y - t0.y) * refN.y + (c.z - t0.z) * refN.z < 0) refN.scaleInPlace(-1);
    }

    const rot = boxPlaced.rotation;
    const axis = BoxTriFace._axis;
    let bestAxis = -1, bestDot = 0, bestSign = 1;
    for (let a = 0; a < 3; a++) {
        axis.set(a === 0 ? 1 : 0, a === 1 ? 1 : 0, a === 2 ? 1 : 0);
        rot.transformVectorInPlace(axis);
        const d = axis.x * refN.x + axis.y * refN.y + axis.z * refN.z;
        const ad = d < 0 ? -d : d;
        if (ad > bestDot) { bestDot = ad; bestAxis = a; bestSign = d < 0 ? 1 : -1; }
    }
    BoxTriFace.lastBestDot = bestDot;
    if (bestDot < BoxTriFace.PARALLEL_COS_LIMIT) return null;

    const shape = boxPlaced.shape;
    const hx = BoxTriFace._hx;
    hx[0] = shape.halfWidth; hx[1] = shape.halfHeight; hx[2] = shape.halfDepth;

    const u = BoxTriFace._u, v = BoxTriFace._v, nAxis = BoxTriFace._nAxis;
    const iu = (bestAxis + 1) % 3, iv = (bestAxis + 2) % 3;
    nAxis.set(bestAxis === 0 ? 1 : 0, bestAxis === 1 ? 1 : 0, bestAxis === 2 ? 1 : 0);
    u.set(iu === 0 ? 1 : 0, iu === 1 ? 1 : 0, iu === 2 ? 1 : 0);
    v.set(iv === 0 ? 1 : 0, iv === 1 ? 1 : 0, iv === 2 ? 1 : 0);
    rot.transformVectorInPlace(nAxis);
    rot.transformVectorInPlace(u);
    rot.transformVectorInPlace(v);

    const penetrationLimit = BoxTriFace.PENETRATION_LIMIT;

    const cx = boxPlaced.position.x + nAxis.x * hx[bestAxis] * bestSign;
    const cy = boxPlaced.position.y + nAxis.y * hx[bestAxis] * bestSign;
    const cz = boxPlaced.position.z + nAxis.z * hx[bestAxis] * bestSign;
    const eu = hx[iu], ev = hx[iv];

    const face = BoxTriFace._face4;
    let n = 0;
    for (let su = -1; su <= 1; su += 2) {
        for (let sv = -1; sv <= 1; sv += 2) {

            const s2 = su < 0 ? sv : -sv;
            const p = face[n++];
            p.set(cx + u.x * eu * su + v.x * ev * s2,
                cy + u.y * eu * su + v.y * ev * s2,
                cz + u.z * eu * su + v.z * ev * s2);
        }
    }

    const triVerts = BoxTriFace._triVerts;
    triVerts[0] = t0; triVerts[1] = t1; triVerts[2] = t2;

    // Which of the two features here is smaller decides which way round to clip. Normally the box's face
    // is, and it is clipped against the triangle's edges below; the reverse - a SMALL mesh triangle on a
    // LARGE box face, which is every small mesh prop resting on a big floor - has no face contact to be had
    // from the triangle's side, so it falls through to GJK/EPA, where a triangle coplanar with the face it
    // rests on is the worst case: EPA returns a near-degenerate direction.
    if (!BoxTriFace._boxFaceIsSmaller(hx, bestAxis, t0, t1, t2)) {
        return BoxTriFace._smallTriangleOnFace(aIsTri, triPlaced, refN, t0, t1, t2,
            cx, cy, cz, u, v, eu, ev, out, nextContact);
    }

    // A face whose region merely TOUCHES the triangle along a shared edge survives the slack clip as a
    // zero-width sliver, and a sliver still reports the full distance between the two planes as
    // penetration - on a ledge that turns an overhanging prop's side face into a lateral shove. The
    // overlap must be real: re-clip without the slack and require actual area before treating this as a
    // face patch.
    if (!BoxTriFace._clipToTriangle(face, 4, BoxTriFace._poly2, BoxTriFace._clip2, triVerts, refN, 0)) return null;
    const trueArea = BoxTriFace._polyArea(BoxTriFace._clipOut, BoxTriFace._clipOutN, refN);
    if (trueArea <= (4 * eu * ev) * BoxTriFace.MIN_OVERLAP_COVERAGE) return null;

    if (!BoxTriFace._clipToTriangle(face, 4, BoxTriFace._poly, BoxTriFace._clip, triVerts, refN, BoxTriFace.EDGE_SLACK)) return null;
    let src = BoxTriFace._clipOut, srcN = BoxTriFace._clipOutN;

    BoxTriFace.lastFaceCoverage = BoxTriFace._polyArea(src, srcN, refN) / (4 * eu * ev);

    const cvxIsA = !aIsTri;
    const normX = cvxIsA ? refN.x : -refN.x;
    const normY = cvxIsA ? refN.y : -refN.y;
    const normZ = cvxIsA ? refN.z : -refN.z;

    let emitted = 0;
    for (let i = 0; i < srcN; i++) {
        const p = src[i];
        const g = (p.x - t0.x) * refN.x + (p.y - t0.y) * refN.y + (p.z - t0.z) * refN.z;
        if (g > BoxTriFace.SEPARATION_LIMIT || g < -penetrationLimit) continue;
        const projX = p.x - g * refN.x, projY = p.y - g * refN.y, projZ = p.z - g * refN.z;

        let dup = false;
        for (let j = 0; j < emitted; j++) {
            const q = out[out.length - 1 - j];
            const qa = cvxIsA ? q.pointOnA : q.pointOnB;
            const dx = qa.x - p.x, dy = qa.y - p.y, dz = qa.z - p.z;
            if (dx * dx + dy * dy + dz * dz < BoxTriFace.DEDUPE_DIST_SQ) { dup = true; break; }
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

// The mirror of the face patch above: the triangle is the smaller feature, so the BOX's face is the
// reference and the triangle is the incident polygon. Clipping it to the face's rectangle and measuring
// against the face's plane gives the contact the face's own normal exactly - no GJK/EPA witness direction
// to tip it - and the same per-point depths the box's face patch would give. Depth is positive when the
// triangle's plane has passed the face's plane the way the box's interior lies, the opposite sense from
// _clipToTriangle, so the sign is carried explicitly rather than negated.
BoxTriFace._smallTriangleOnFace = function (aIsTri, triPlaced, refN, t0, t1, t2,
    cx, cy, cz, u, v, eu, ev, out, nextContact) {

    const poly = BoxTriFace._poly, clip = BoxTriFace._clip;
    poly[0].copy(t0); poly[1].copy(t1); poly[2].copy(t2);
    let src = poly, dst = clip, count = 3;

    for (let pass = 0; pass < 4; pass++) {
        const axis = (pass < 2) ? u : v;
        const limit = (pass < 2) ? eu : ev;
        const sign = (pass % 2 === 0) ? 1 : -1;
        let outCount = 0;
        for (let i = 0; i < count; i++) {
            const cur = src[i], next = src[(i + 1) % count];
            const dCur = ((cur.x - cx) * axis.x + (cur.y - cy) * axis.y + (cur.z - cz) * axis.z) * sign - limit;
            const dNext = ((next.x - cx) * axis.x + (next.y - cy) * axis.y + (next.z - cz) * axis.z) * sign - limit;
            if (dCur <= 0) dst[outCount++].copy(cur);
            if ((dCur <= 0) !== (dNext <= 0)) {
                const tt = dCur / (dCur - dNext);
                dst[outCount++].set(
                    cur.x + (next.x - cur.x) * tt,
                    cur.y + (next.y - cur.y) * tt,
                    cur.z + (next.z - cur.z) * tt
                );
            }
        }
        if (outCount === 0) return null;
        count = outCount;
        const swap = src; src = dst; dst = swap;
    }

    const cvxIsA = !aIsTri;
    const normX = cvxIsA ? refN.x : -refN.x;
    const normY = cvxIsA ? refN.y : -refN.y;
    const normZ = cvxIsA ? refN.z : -refN.z;

    let emitted = 0;
    for (let i = 0; i < count; i++) {
        const p = src[i];
        const g = (p.x - cx) * refN.x + (p.y - cy) * refN.y + (p.z - cz) * refN.z;
        if (g > BoxTriFace.SEPARATION_LIMIT || g < -BoxTriFace.PENETRATION_LIMIT) continue;
        const projX = p.x - g * refN.x, projY = p.y - g * refN.y, projZ = p.z - g * refN.z;

        let dup = false;
        for (let j = 0; j < emitted; j++) {
            const q = out[out.length - 1 - j];
            const qa = aIsTri ? q.pointOnA : q.pointOnB;
            const dx = qa.x - p.x, dy = qa.y - p.y, dz = qa.z - p.z;
            if (dx * dx + dy * dy + dz * dz < BoxTriFace.DEDUPE_DIST_SQ) { dup = true; break; }
        }
        if (dup) continue;

        const contact = nextContact();
        if (aIsTri) {
            contact.pointOnA.set(p.x, p.y, p.z);
            contact.pointOnB.set(projX, projY, projZ);
        } else {
            contact.pointOnA.set(projX, projY, projZ);
            contact.pointOnB.set(p.x, p.y, p.z);
        }
        contact.normal.set(normX, normY, normZ);
        contact.signedDistance = g;
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

// Clips the convex polygon `src` (srcN vertices) against the triangle's three edge planes, writing
// into `bufA`/`bufB` and ping-ponging between them. `slack` widens the clip region so a face lying
// exactly on the mesh's seam keeps its contact. Returns false when the polygon is clipped away
// entirely; otherwise the result lives in _clipOut (with _clipOutN vertices).
BoxTriFace._clipToTriangle = function (p0, n0, bufA, bufB, triVerts, refN, slack) {
    let src = p0, srcN = n0;
    let dst = (p0 === bufA) ? bufB : bufA;
    const eA = BoxTriFace._eA, eN = BoxTriFace._eN;
    for (let e = 0; e < 3; e++) {
        const va = triVerts[e], vb = triVerts[(e + 1) % 3];
        eA.set(vb.x - va.x, vb.y - va.y, vb.z - va.z);

        eN.set(eA.y * refN.z - eA.z * refN.y, eA.z * refN.x - eA.x * refN.z, eA.x * refN.y - eA.y * refN.x);
        const vc = triVerts[(e + 2) % 3];
        if ((vc.x - va.x) * eN.x + (vc.y - va.y) * eN.y + (vc.z - va.z) * eN.z < 0) eN.scaleInPlace(-1);
        const inv = 1 / (Math.sqrt(eN.x * eN.x + eN.y * eN.y + eN.z * eN.z) || 1);
        eN.scaleInPlace(inv);

        let dstN = 0;
        for (let i = 0; i < srcN; i++) {
            const cur = src[i], nxt = src[(i + 1) % srcN];
            const dCur = (cur.x - va.x) * eN.x + (cur.y - va.y) * eN.y + (cur.z - va.z) * eN.z + slack;
            const dNxt = (nxt.x - va.x) * eN.x + (nxt.y - va.y) * eN.y + (nxt.z - va.z) * eN.z + slack;
            if (dCur >= 0) dst[dstN++].copy(cur);
            if ((dCur >= 0) !== (dNxt >= 0)) {
                const tt = dCur / (dCur - dNxt);
                dst[dstN++].set(cur.x + (nxt.x - cur.x) * tt, cur.y + (nxt.y - cur.y) * tt, cur.z + (nxt.z - cur.z) * tt);
            }
        }
        if (dstN === 0) return false;
        src = dst; srcN = dstN;
        dst = (src === bufA) ? bufB : bufA;
    }
    BoxTriFace._clipOut = src;
    BoxTriFace._clipOutN = srcN;
    return true;
};

BoxTriFace._polyArea = function (p, n, nrm) {
    if (n < 3) return 0;
    let sx = 0, sy = 0, sz = 0;
    for (let i = 1; i < n - 1; i++) {
        const ax = p[i].x - p[0].x, ay = p[i].y - p[0].y, az = p[i].z - p[0].z;
        const bx = p[i + 1].x - p[0].x, by = p[i + 1].y - p[0].y, bz = p[i + 1].z - p[0].z;
        sx += ay * bz - az * by; sy += az * bx - ax * bz; sz += ax * by - ay * bx;
    }
    return 0.5 * Math.abs(sx * nrm.x + sy * nrm.y + sz * nrm.z);
};


BoxTriFace._boxFaceIsSmaller = function (hx, axis, t0, t1, t2) {
    const faceArea = 4 * hx[(axis + 1) % 3] * hx[(axis + 2) % 3];
    const ax = t1.x - t0.x, ay = t1.y - t0.y, az = t1.z - t0.z;
    const bx = t2.x - t0.x, by = t2.y - t0.y, bz = t2.z - t0.z;
    const cx = ay * bz - az * by, cy = az * bx - ax * bz, cz = ax * by - ay * bx;
    const triArea = 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
    return faceArea <= triArea * BoxTriFace.MAX_FACE_AREA_RATIO;
};

BoxTriFace._refN = new Vector3();
BoxTriFace._axis = new Vector3();
BoxTriFace._hx = [0, 0, 0];
BoxTriFace._u = new Vector3();
BoxTriFace._v = new Vector3();
BoxTriFace._nAxis = new Vector3();
BoxTriFace._eA = new Vector3();
BoxTriFace._eN = new Vector3();

BoxTriFace._triVerts = [null, null, null];
BoxTriFace._clipOut = null;
BoxTriFace._clipOutN = 0;

BoxTriFace._face4 = [];
BoxTriFace._poly = [];
BoxTriFace._clip = [];
BoxTriFace._poly2 = [];
BoxTriFace._clip2 = [];
for (let i = 0; i < 8; i++) {
    BoxTriFace._face4.push(new Vector3());
    BoxTriFace._poly.push(new Vector3());
    BoxTriFace._clip.push(new Vector3());
    BoxTriFace._poly2.push(new Vector3());
    BoxTriFace._clip2.push(new Vector3());
}

ActionPhysics.BoxTriFace = BoxTriFace;
