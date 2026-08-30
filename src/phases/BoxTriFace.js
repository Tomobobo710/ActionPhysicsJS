// Closed-form contact for a BoxShape face lying flat on a mesh triangle.
//
// GJK/EPA reports one witness point per (box, triangle) pair. A mesh quad is two triangles and a
// resting box usually overlaps both, so each side contributes one wandering point, the manifold
// flickers between one and two points, and every dropped point loses its warm-start lambda - the
// box rocks and gains energy instead of settling.
//
// When the box face is near-parallel to the triangle plane, the contact is instead the box face
// polygon clipped to the triangle: a stable 3-6 point patch. Anything else (box on edge, on a
// corner, or tilted past PARALLEL_COS_LIMIT) returns null and falls through to GJK/EPA, which is
// correct for those - they are single-feature contacts.
const BoxTriFace = {};

BoxTriFace.applies = function (placedA, placedB) {
    const aTri = placedA.shape instanceof TriangleShape;
    const bTri = placedB.shape instanceof TriangleShape;
    if (aTri === bTri) return false; // need exactly one triangle
    // Strictly a BoxShape on the other side. A CompoundShape is not itself convex - its children are
    // dispatched individually by the midphase - so it must never reach the support-map code here.
    const other = aTri ? placedB.shape : placedA.shape;
    return other instanceof BoxShape;
};

// cos of the angle between the box face normal and the triangle normal, above which they count as
// parallel. 0.90 ~ 26 degrees: past that the box is on an edge, not a face. Wide on purpose - a
// tighter limit lets a one-tick landing wobble drop the patch, and the single GJK point that
// replaces it torques the box further over, so the dot never recovers.
BoxTriFace.PARALLEL_COS_LIMIT = 0.90;
// Same speculative band as ConvexTri, so the two paths report contacts over the same range.
BoxTriFace.SEPARATION_LIMIT = 0.5;
BoxTriFace.PENETRATION_LIMIT = 1.0;
// A clipped vertex this far outside the triangle plane's band is dropped.
BoxTriFace.EDGE_SLACK = 0.02;
BoxTriFace.DEDUPE_DIST_SQ = 1e-8;
// Box face area must be within this multiple of the triangle's area for the face patch to apply.
BoxTriFace.MAX_FACE_AREA_RATIO = 4;

BoxTriFace.lastVerdict = 'maybe';
// Fraction of the box face still over this triangle after clipping (1 = entirely inside). Read by
// GeometryRefresh's fast re-clip to tell whether the stored triangle still carries the box.
BoxTriFace.lastFaceCoverage = 0;
BoxTriFace.lastBestDot = 0;

// Returns `out` when it produced a face patch (vetoing GJK/EPA), else null to fall through.
// hintNormalBToA: established manifold normal, used to orient the reference face (same contract as
// ConvexTri.test - the box-centre heuristic flips once a settling box's centre nears the plane).
BoxTriFace.test = function (placedA, placedB, out, nextContact, hintNormalBToA) {
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
    if (hintNormalBToA) {
        const d = refN.x * hintNormalBToA.x + refN.y * hintNormalBToA.y + refN.z * hintNormalBToA.z;
        if ((aIsTri && d > 0) || (!aIsTri && d < 0)) refN.scaleInPlace(-1);
    } else {
        const c = boxPlaced.position;
        if ((c.x - t0.x) * refN.x + (c.y - t0.y) * refN.y + (c.z - t0.z) * refN.z < 0) refN.scaleInPlace(-1);
    }

    // Pick the box face whose outward normal is most opposed to refN - the face pointing at the
    // triangle. Bail unless it is near parallel; a tilted box is a single-feature contact.
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
    BoxTriFace.lastBestDot = bestDot; // diagnostic: how parallel the chosen face was
    if (bestDot < BoxTriFace.PARALLEL_COS_LIMIT) return null; // not lying flat - let GJK/EPA judge

    // Build the face's four world-space corners: centre + sign*halfExtent along the face axis, then
    // +/- the other two half-extents.
    const shape = boxPlaced.shape;
    const hx = BoxTriFace._hx;
    hx[0] = shape.halfWidth; hx[1] = shape.halfHeight; hx[2] = shape.halfDepth;

    // The patch is the BOX face clipped to the triangle, so it is only the right contact when the
    // box face is the smaller feature. A large static box (a ground slab) under a small mesh
    // triangle is the reverse case: the triangle lies entirely within the face, clipping yields the
    // triangle back, and the resulting patch fights the mesh's own contacts. Fall through to GJK/EPA
    // whenever the box face is not comfortably smaller than the triangle.
    if (!BoxTriFace._boxFaceIsSmaller(hx, bestAxis, t0, t1, t2)) return null;

    const u = BoxTriFace._u, v = BoxTriFace._v, nAxis = BoxTriFace._nAxis;
    const iu = (bestAxis + 1) % 3, iv = (bestAxis + 2) % 3;
    nAxis.set(bestAxis === 0 ? 1 : 0, bestAxis === 1 ? 1 : 0, bestAxis === 2 ? 1 : 0);
    u.set(iu === 0 ? 1 : 0, iu === 1 ? 1 : 0, iu === 2 ? 1 : 0);
    v.set(iv === 0 ? 1 : 0, iv === 1 ? 1 : 0, iv === 2 ? 1 : 0);
    rot.transformVectorInPlace(nAxis);
    rot.transformVectorInPlace(u);
    rot.transformVectorInPlace(v);

    const cx = boxPlaced.position.x + nAxis.x * hx[bestAxis] * bestSign;
    const cy = boxPlaced.position.y + nAxis.y * hx[bestAxis] * bestSign;
    const cz = boxPlaced.position.z + nAxis.z * hx[bestAxis] * bestSign;
    const eu = hx[iu], ev = hx[iv];

    const poly = BoxTriFace._poly;
    let n = 0;
    for (let su = -1; su <= 1; su += 2) {
        for (let sv = -1; sv <= 1; sv += 2) {
            // wind the quad consistently: (-,-), (+,-), (+,+), (-,+)
            const s2 = su < 0 ? sv : -sv;
            const p = poly[n++];
            p.set(cx + u.x * eu * su + v.x * ev * s2,
                cy + u.y * eu * su + v.y * ev * s2,
                cz + u.z * eu * su + v.z * ev * s2);
        }
    }

    // Clip the face quad against the triangle's three edge half-planes, in the triangle's plane.
    let src = poly, srcN = 4, dst = BoxTriFace._clip, dstN = 0;
    const eA = BoxTriFace._eA, eN = BoxTriFace._eN;
    const triVerts = [t0, t1, t2];
    for (let e = 0; e < 3; e++) {
        const va = triVerts[e], vb = triVerts[(e + 1) % 3];
        eA.set(vb.x - va.x, vb.y - va.y, vb.z - va.z);
        // Inward half-plane normal: edge x faceNormal, oriented toward the opposite vertex.
        eN.set(eA.y * refN.z - eA.z * refN.y, eA.z * refN.x - eA.x * refN.z, eA.x * refN.y - eA.y * refN.x);
        const vc = triVerts[(e + 2) % 3];
        if ((vc.x - va.x) * eN.x + (vc.y - va.y) * eN.y + (vc.z - va.z) * eN.z < 0) eN.scaleInPlace(-1);
        const inv = 1 / (Math.sqrt(eN.x * eN.x + eN.y * eN.y + eN.z * eN.z) || 1);
        eN.scaleInPlace(inv);

        dstN = 0;
        for (let i = 0; i < srcN; i++) {
            const cur = src[i], nxt = src[(i + 1) % srcN];
            const dCur = (cur.x - va.x) * eN.x + (cur.y - va.y) * eN.y + (cur.z - va.z) * eN.z + BoxTriFace.EDGE_SLACK;
            const dNxt = (nxt.x - va.x) * eN.x + (nxt.y - va.y) * eN.y + (nxt.z - va.z) * eN.z + BoxTriFace.EDGE_SLACK;
            if (dCur >= 0) dst[dstN++].copy(cur);
            if ((dCur >= 0) !== (dNxt >= 0)) {
                const tt = dCur / (dCur - dNxt);
                dst[dstN++].set(cur.x + (nxt.x - cur.x) * tt, cur.y + (nxt.y - cur.y) * tt, cur.z + (nxt.z - cur.z) * tt);
            }
        }
        if (dstN === 0) return null; // face does not overlap this triangle at all
        const tmp = src; src = dst; srcN = dstN; dst = tmp;
    }
    // Coverage = clipped polygon area / full face area, both measured in the triangle's plane.
    BoxTriFace.lastFaceCoverage = BoxTriFace._polyArea(src, srcN, refN) / (4 * eu * ev);

    // Emit the surviving polygon vertices that are within the speculative band.
    const cvxIsA = !aIsTri;
    const normX = cvxIsA ? refN.x : -refN.x;
    const normY = cvxIsA ? refN.y : -refN.y;
    const normZ = cvxIsA ? refN.z : -refN.z;

    let emitted = 0;
    for (let i = 0; i < srcN; i++) {
        const p = src[i];
        const g = (p.x - t0.x) * refN.x + (p.y - t0.y) * refN.y + (p.z - t0.z) * refN.z;
        if (g > BoxTriFace.SEPARATION_LIMIT || g < -BoxTriFace.PENETRATION_LIMIT) continue;
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
        contact.setMeshTriangle(t0, t1, t2, triPlaced.bodyCenter, aIsTri);
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
        emitted++;
    }

    if (emitted === 0) return null;
    return out;
};

// Area of a planar polygon with the given face normal (fan sum of cross products).
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

// Is the box face (the two half-extents perpendicular to `axis`) smaller than the triangle? Compares
// the face's area against the triangle's; a ground slab under a small mesh triangle fails this.
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
// Clip buffers: a quad clipped by three half-planes can reach 7 vertices.
BoxTriFace._poly = [];
BoxTriFace._clip = [];
for (let i = 0; i < 8; i++) { BoxTriFace._poly.push(new Vector3()); BoxTriFace._clip.push(new Vector3()); }

ActionPhysics.BoxTriFace = BoxTriFace;
