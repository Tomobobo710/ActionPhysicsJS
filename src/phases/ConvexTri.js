// Closed-form curved-convex-vs-triangle face contact. A round convex (sphere / cylinder / cone /
// capsule) resting on a MeshShape triangle otherwise goes through GJK/EPA, which yields a single
// witness point whose normal degenerates near a triangle edge or the seam between two tiles: the
// point wanders, kicks the body sideways, and once the convex's reference point crosses to the
// back of the (zero-thickness, one-sided) triangle no contact is generated at all and the body
// free-falls forever. This path instead treats the triangle as a one-sided face: the contact
// normal is the triangle normal oriented toward the side the convex's CENTRE is on, and a contact
// is kept while the convex's deepest point is anywhere from SEPARATION_LIMIT in front of that
// face to PENETRATION_LIMIT behind it - so a convex that dipped below the plane is pushed back
// out along the face normal rather than lost.
//
// Flat-faced convexes (BoxShape, ConvexShape) are deliberately NOT handled here: GJK/EPA already
// gives them a stable face contact against a triangle, and routing them through this
// deepest-point path regresses a rotated box landing across a seam.
const ConvexTri = {};

ConvexTri._isCurvedConvex = function (shape) {
    return (shape instanceof SphereShape) || (shape instanceof CylinderShape) ||
        (shape instanceof ConeShape) || (shape instanceof CapsuleShape);
};

ConvexTri.applies = function (placedA, placedB) {
    const aTri = placedA.shape instanceof TriangleShape;
    const bTri = placedB.shape instanceof TriangleShape;
    if (aTri === bTri) return false; // need exactly one triangle
    return aTri ? ConvexTri._isCurvedConvex(placedB.shape) : ConvexTri._isCurvedConvex(placedA.shape);
};

ConvexTri.SEPARATION_LIMIT = 0.5;   // speculative: report a face contact up to this gap in front of the plane
ConvexTri.PENETRATION_LIMIT = 1.0;  // keep resolving as a face contact up to this depth behind it
ConvexTri.EDGE_SLACK = 0.06;        // how far outside the triangle a support point may project and still count
ConvexTri.AREA_EPSILON = 1e-12;
ConvexTri.DEDUPE_DIST_SQ = 1e-6;

// Unit winding normal of a world-space triangle into `out`. Returns false if degenerate.
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

// Point-in-triangle (point assumed on the plane), with an outward slack of EDGE_SLACK metres.
ConvexTri._pointInTri = function (hx, hy, hz, t0, t1, t2, tn, slack) {
    const e0x = t1.x - t0.x, e0y = t1.y - t0.y, e0z = t1.z - t0.z;
    const e1x = t2.x - t1.x, e1y = t2.y - t1.y, e1z = t2.z - t1.z;
    const e2x = t0.x - t2.x, e2y = t0.y - t2.y, e2z = t0.z - t2.z;
    const c0x = hx - t0.x, c0y = hy - t0.y, c0z = hz - t0.z;
    const c1x = hx - t1.x, c1y = hy - t1.y, c1z = hz - t1.z;
    const c2x = hx - t2.x, c2y = hy - t2.y, c2z = hz - t2.z;
    // signed area (x2) of each sub-triangle about tn; divide by edge length for a metric distance.
    const s0 = tn.x * (e0y * c0z - e0z * c0y) + tn.y * (e0z * c0x - e0x * c0z) + tn.z * (e0x * c0y - e0y * c0x);
    const s1 = tn.x * (e1y * c1z - e1z * c1y) + tn.y * (e1z * c1x - e1x * c1z) + tn.z * (e1x * c1y - e1y * c1x);
    const s2 = tn.x * (e2y * c2z - e2z * c2y) + tn.y * (e2z * c2x - e2x * c2z) + tn.z * (e2x * c2y - e2y * c2x);
    const l0 = Math.sqrt(e0x * e0x + e0y * e0y + e0z * e0z) || 1;
    const l1 = Math.sqrt(e1x * e1x + e1y * e1y + e1z * e1z) || 1;
    const l2 = Math.sqrt(e2x * e2x + e2y * e2y + e2z * e2z) || 1;
    // `tn` here is refN, which may have been flipped away from the triangle's winding normal, so
    // the three sub-areas share a sign but which one is unknown - accept either, like TriTri.
    const d0 = s0 / l0, d1 = s1 / l1, d2 = s2 / l2;
    // How far outside the nearest edge (metres); <= 0 means inside. Take the better of the two
    // winding-sign interpretations.
    const outPos = Math.max(-d0, -d1, -d2);   // if the triangle is CCW about tn
    const outNeg = Math.max(d0, d1, d2);      // if CW
    ConvexTri._lastOutside = Math.min(outPos, outNeg);
    return ConvexTri._lastOutside <= slack;
};

// out: array to push ContactDetails into (pooled via nextContact()). Returns out on success (may
// be empty, which still vetoes the GJK/EPA fallback), or null to fall through to GJK/EPA.
//
// hintNormalBToA (optional): the established contact normal (B -> A) from an existing manifold
// point. When given, the reference face is oriented to match it instead of the convex-centre
// heuristic. GeometryRefresh passes this every substep: a settling convex's centroid can creep to
// within a hair of the (heightfield) triangle plane, at which point the centre heuristic flips the
// normal and the contact shoves the body through. The established normal doesn't drift, so the
// re-clip stays stable.
ConvexTri.test = function (placedA, placedB, out, nextContact, hintNormalBToA) {
    const aIsTri = placedA.shape instanceof TriangleShape;
    const triPlaced = aIsTri ? placedA : placedB;
    const cvxPlaced = aIsTri ? placedB : placedA;
    const tri = triPlaced.shape;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    // refN = the triangle's normal, oriented toward the convex's resting/free side. When an
    // established contact normal is supplied, match it (contact.normal is B -> A, and the emitted
    // normal below is -refN when the triangle is A / +refN when the triangle is B - so refN points
    // opposite the B->A normal on the A-triangle branch). Otherwise fall back to the convex-centre
    // heuristic: a convex approaching from clearly outside has its centre a full radius off the
    // plane, so the sign is unambiguous at first contact (which is the only place this branch runs).
    const refN = ConvexTri._refN;
    if (!ConvexTri._normalInto(refN, t0, t1, t2)) return null;
    if (hintNormalBToA) {
        // want refN aligned with: aIsTri ? -hintNormal : +hintNormal
        const dot = refN.x * hintNormalBToA.x + refN.y * hintNormalBToA.y + refN.z * hintNormalBToA.z;
        if ((aIsTri && dot > 0) || (!aIsTri && dot < 0)) refN.scaleInPlace(-1);
    } else {
        const cvxPos = cvxPlaced.position;
        if ((cvxPos.x - t0.x) * refN.x + (cvxPos.y - t0.y) * refN.y + (cvxPos.z - t0.z) * refN.z < 0) {
            refN.scaleInPlace(-1);
        }
    }

    // Diagnostics read by GeometryRefresh's fast per-triangle re-clip to decide whether the stored
    // triangle is still trustworthy. lastDeepestInTriangle: the true deepest point projects inside
    // the triangle (+ EDGE_SLACK). lastDeepestOutsideDist: how far outside it projects, in metres
    // (0 when inside). A settled body jittering at a tile edge keeps this tiny; a body that has
    // drifted onto a neighbour tile pushes it past a fraction of the tile size, at which point the
    // caller must re-expand to pick up the new supporting triangle.
    ConvexTri.lastDeepestInTriangle = false;
    ConvexTri.lastDeepestOutsideDist = Infinity;
    // 'contact' | 'separated' | 'maybe'. 'separated' means the convex provably has no contact with
    // this triangle (its nearest point toward the plane is clear of the speculative band), so the
    // caller can skip the GJK/EPA fallback entirely - that fallback would just re-derive the same
    // separation at ~100x the cost, and for a prop resting on one tile of a big compound ground
    // that is the overwhelmingly common case. 'maybe' means no face contact but an edge/vertex hit
    // is still possible - fall through to GJK/EPA.
    ConvexTri.lastVerdict = 'maybe';

    // Deepest convex point toward the triangle = support along -refN.
    const probeDir = ConvexTri._probeDir;
    const dp = ConvexTri._dp;
    const invRot = ConvexTri._invRot.copy(cvxPlaced.rotation).invert();
    const scratchDir = ConvexTri._scratchDir;

    probeDir.set(-refN.x, -refN.y, -refN.z);
    MinkowskiSupport.supportOfInto(dp, cvxPlaced, invRot, probeDir, scratchDir);
    const gap = (dp.x - t0.x) * refN.x + (dp.y - t0.y) * refN.y + (dp.z - t0.z) * refN.z;
    if (gap > ConvexTri.SEPARATION_LIMIT) {
        // Every point of the convex is at least `gap` in front of the triangle's plane, so it
        // cannot be touching the triangle (which lies in that plane). Definitively separated.
        ConvexTri.lastVerdict = 'separated';
        return null;
    }
    if (gap < -ConvexTri.PENETRATION_LIMIT) return null; // deep behind - let GJK/EPA judge

    // Gather up to four candidate points: the true deepest point, plus a probe tilted toward each
    // triangle edge midpoint. The tilted probes give a flat-faced or side-lying convex a
    // multi-point manifold (no rocking on a single point); a sphere/curved rest collapses them
    // back to one after dedupe.
    const cand = ConvexTri._cand;
    let nCand = 0;
    cand[nCand++].copy(dp);

    const centroidX = (t0.x + t1.x + t2.x) / 3, centroidY = (t0.y + t1.y + t2.y) / 3, centroidZ = (t0.z + t1.z + t2.z) / 3;
    const edgeMid = ConvexTri._edgeMid;
    for (let e = 0; e < 3; e++) {
        const va = e === 0 ? t0 : (e === 1 ? t1 : t2);
        const vb = e === 0 ? t1 : (e === 1 ? t2 : t0);
        edgeMid.set((va.x + vb.x) * 0.5 - centroidX, (va.y + vb.y) * 0.5 - centroidY, (va.z + vb.z) * 0.5 - centroidZ);
        const lat = edgeMid.x * refN.x + edgeMid.y * refN.y + edgeMid.z * refN.z; // strip any normal component
        edgeMid.set(edgeMid.x - lat * refN.x, edgeMid.y - lat * refN.y, edgeMid.z - lat * refN.z);
        const el = Math.sqrt(edgeMid.x * edgeMid.x + edgeMid.y * edgeMid.y + edgeMid.z * edgeMid.z);
        if (el < 1e-9) continue;
        const s = 0.5 / el;
        probeDir.set(-refN.x + edgeMid.x * s, -refN.y + edgeMid.y * s, -refN.z + edgeMid.z * s);
        const pl = Math.sqrt(probeDir.x * probeDir.x + probeDir.y * probeDir.y + probeDir.z * probeDir.z) || 1;
        probeDir.scaleInPlace(1 / pl);
        MinkowskiSupport.supportOfInto(cand[nCand], cvxPlaced, invRot, probeDir, scratchDir);
        nCand++;
    }

    // refN points from the triangle toward the convex. Pipeline convention: contact.normal points
    // from B to A. So if the convex is B, normal = -refN; if the convex is A, normal = +refN.
    const cvxIsA = !aIsTri;
    const normX = cvxIsA ? refN.x : -refN.x;
    const normY = cvxIsA ? refN.y : -refN.y;
    const normZ = cvxIsA ? refN.z : -refN.z;

    let emitted = 0;
    for (let i = 0; i < nCand; i++) {
        const p = cand[i];
        const g = (p.x - t0.x) * refN.x + (p.y - t0.y) * refN.y + (p.z - t0.z) * refN.z;
        // Record how the true deepest point relates to this triangle, in/out-of-band included.
        if (i === 0 && (g > ConvexTri.SEPARATION_LIMIT || g < -ConvexTri.PENETRATION_LIMIT)) {
            ConvexTri.lastDeepestOutsideDist = Infinity;
        }
        if (g > ConvexTri.SEPARATION_LIMIT || g < -ConvexTri.PENETRATION_LIMIT) continue;
        // project onto the triangle plane
        const projX = p.x - g * refN.x, projY = p.y - g * refN.y, projZ = p.z - g * refN.z;
        const inTri = ConvexTri._pointInTri(projX, projY, projZ, t0, t1, t2, refN, ConvexTri.EDGE_SLACK);
        if (i === 0) {
            ConvexTri.lastDeepestOutsideDist = ConvexTri._lastOutside;
            if (inTri) ConvexTri.lastDeepestInTriangle = true;
        }
        if (!inTri) continue;

        // dedupe against already-emitted points
        let dup = false;
        for (let j = 0; j < emitted; j++) {
            const q = out[out.length - 1 - j];
            const ddx = q.pointOnA.x - p.x, ddy = q.pointOnA.y - p.y, ddz = q.pointOnA.z - p.z;
            if (ddx * ddx + ddy * ddy + ddz * ddz < ConvexTri.DEDUPE_DIST_SQ) { dup = true; break; }
        }
        if (dup) continue;

        const contact = nextContact();
        // pointOnA / pointOnB per the pair's actual A/B roles.
        if (aIsTri) {
            contact.pointOnB.set(p.x, p.y, p.z);                 // convex (B)
            contact.pointOnA.set(projX, projY, projZ);           // triangle (A)
        } else {
            contact.pointOnA.set(p.x, p.y, p.z);                 // convex (A)
            contact.pointOnB.set(projX, projY, projZ);           // triangle (B)
        }
        contact.normal.set(normX, normY, normZ);                 // B -> A
        contact.signedDistance = -g;                             // positive = penetrating
        contact.fromMeshFace = true;
        contact.setMeshTriangle(t0, t1, t2, triPlaced.bodyCenter, aIsTri); // for per-substep re-clip
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
        emitted++;
    }

    if (emitted === 0) {
        // No face contact. Decide whether the convex is nonetheless provably clear of the
        // triangle so the caller can skip GJK/EPA. The nearest triangle feature to the convex's
        // deepest point is an edge (the point projects outside), so a lower bound on the
        // convex-to-triangle distance is hypot(along-plane gap, how-far-outside). When that bound
        // clears the speculative band, it is 'separated'; otherwise an edge/vertex contact is
        // still possible and we leave the verdict at 'maybe' for the GJK/EPA fallback. This is the
        // common case for a prop resting on one tile of a big compound ground: its AABB overlaps
        // the neighbouring coplanar tiles (gap ~ 0) but it sits laterally outside them.
        const outside = ConvexTri.lastDeepestOutsideDist;
        const along = gap > 0 ? gap : 0;
        if (isFinite(outside) && Math.sqrt(along * along + outside * outside) > ConvexTri.SEPARATION_LIMIT) {
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
ConvexTri._cand = [new Vector3(), new Vector3(), new Vector3(), new Vector3()];
ConvexTri.lastDeepestInTriangle = false;
ConvexTri.lastDeepestOutsideDist = Infinity;
ConvexTri._lastOutside = 0;
ConvexTri.lastVerdict = 'maybe';
// Fast re-clip may trust a stored triangle while the convex's deepest point sits at most this far
// (metres) outside it - covers settled-body jitter and a shallow overhang, but not a real slide
// onto the neighbouring tile.
ConvexTri.REFRESH_DRIFT_TOLERANCE = 0.35;

ActionPhysics.ConvexTri = ConvexTri;
