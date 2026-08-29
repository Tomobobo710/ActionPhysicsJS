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
    return (d0 >= -slack && d1 >= -slack && d2 >= -slack) ||
        (d0 <= slack && d1 <= slack && d2 <= slack);
};

// out: array to push ContactDetails into (pooled via nextContact()). Returns out on success (may
// be empty, which still vetoes the GJK/EPA fallback), or null to fall through to GJK/EPA.
ConvexTri.test = function (placedA, placedB, out, nextContact) {
    const aIsTri = placedA.shape instanceof TriangleShape;
    const triPlaced = aIsTri ? placedA : placedB;
    const cvxPlaced = aIsTri ? placedB : placedA;
    const tri = triPlaced.shape;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    // refN = the triangle's normal, oriented toward the side the convex's CENTRE is on (its
    // resting/free side). Using the centre - not the deepest point - is what gives one-sided
    // recovery: a convex whose surface has dipped just behind the face still has its centre on
    // the free side (centre-to-face distance is ~the shape radius, larger than any penetration
    // we keep resolving), so refN stays put and the contact pushes it back out along +refN
    // rather than flipping and shoving it through. t0 is a triangle vertex - always local and
    // correct - so this works for both outward- and inward-wound mesh geometry without needing a
    // body-centre hint (which is degenerate for a compound ground's per-tile children).
    const refN = ConvexTri._refN;
    if (!ConvexTri._normalInto(refN, t0, t1, t2)) return null;
    const cvxPos = cvxPlaced.position;
    if ((cvxPos.x - t0.x) * refN.x + (cvxPos.y - t0.y) * refN.y + (cvxPos.z - t0.z) * refN.z < 0) {
        refN.scaleInPlace(-1);
    }

    // Deepest convex point toward the triangle = support along -refN.
    const probeDir = ConvexTri._probeDir;
    const dp = ConvexTri._dp;
    const invRot = ConvexTri._invRot.copy(cvxPlaced.rotation).invert();
    const scratchDir = ConvexTri._scratchDir;

    probeDir.set(-refN.x, -refN.y, -refN.z);
    MinkowskiSupport.supportOfInto(dp, cvxPlaced, invRot, probeDir, scratchDir);
    const gap = (dp.x - t0.x) * refN.x + (dp.y - t0.y) * refN.y + (dp.z - t0.z) * refN.z;
    if (gap > ConvexTri.SEPARATION_LIMIT || gap < -ConvexTri.PENETRATION_LIMIT) return null;

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
        if (g > ConvexTri.SEPARATION_LIMIT || g < -ConvexTri.PENETRATION_LIMIT) continue;
        // project onto the triangle plane
        const projX = p.x - g * refN.x, projY = p.y - g * refN.y, projZ = p.z - g * refN.z;
        if (!ConvexTri._pointInTri(projX, projY, projZ, t0, t1, t2, refN, ConvexTri.EDGE_SLACK)) continue;

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
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
        emitted++;
    }

    if (emitted === 0) return null;
    return out;
};

ConvexTri._refN = new Vector3();
ConvexTri._probeDir = new Vector3();
ConvexTri._scratchDir = new Vector3();
ConvexTri._invRot = new Quaternion();
ConvexTri._dp = new Vector3();
ConvexTri._edgeMid = new Vector3();
ConvexTri._cand = [new Vector3(), new Vector3(), new Vector3(), new Vector3()];

ActionPhysics.ConvexTri = ConvexTri;
