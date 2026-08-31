// Closed-form curved-convex-vs-triangle face contact, replacing GJK/EPA's single witness point,
// whose normal degenerates near a triangle edge or a tile seam. The triangle is treated as a
// one-sided face: the normal is the triangle normal oriented toward the convex's centre, and a
// contact is kept from SEPARATION_LIMIT in front of the face to PENETRATION_LIMIT behind it.
//
// Flat-faced convexes (BoxShape, ConvexShape) are not handled here - GJK/EPA already gives them a
// stable face contact, and this deepest-point path regresses a rotated box landing across a seam.
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
ConvexTri.MIN_CULL_LIMIT = 0.05;    // floor on the margin-derived 'separated' cull distance
ConvexTri.EDGE_SLACK = 0.06;        // how far outside the triangle a support point may project and still count
ConvexTri.AREA_EPSILON = 1e-12;
ConvexTri.DEDUPE_DIST_SQ = 1e-6;
// Support probes tilted off the contact normal, spaced evenly around it. Four gives a flat-based
// convex a square patch; the tilt is how far off-normal each probe leans.
ConvexTri.PROBE_COUNT = 4;
ConvexTri.PROBE_TILT = 0.5;
// cos/sin of the probe angles scaled by PROBE_TILT, written out because build.js rejects Math.sin/cos.
ConvexTri._PROBE_U = [ConvexTri.PROBE_TILT, 0, -ConvexTri.PROBE_TILT, 0];
ConvexTri._PROBE_V = [0, ConvexTri.PROBE_TILT, 0, -ConvexTri.PROBE_TILT];

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
// hintNormalBToA (optional): established contact normal (B -> A) from an existing manifold point,
// used to orient the reference face instead of the convex-centre heuristic. GeometryRefresh passes
// it every substep - a settling convex's centroid can creep to within a hair of the triangle plane,
// where the centre heuristic flips the normal and shoves the body through.
// margin (optional): the pair's speculative margin. Only tightens the 'separated' verdict.
ConvexTri.test = function (placedA, placedB, out, nextContact, hintNormalBToA, margin) {
    const aIsTri = placedA.shape instanceof TriangleShape;
    const triPlaced = aIsTri ? placedA : placedB;
    const cvxPlaced = aIsTri ? placedB : placedA;
    const tri = triPlaced.shape;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    // refN = the triangle normal oriented toward the convex. Match an established contact normal
    // when one is supplied; otherwise use the convex's centre, which is unambiguous at first
    // contact - the only place that branch runs.
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

    // Read by GeometryRefresh's fast re-clip to decide whether a stored triangle is still the one
    // carrying the body. lastDeepestOutsideDist is how far outside the triangle the deepest point
    // projects, in metres (0 when inside).
    ConvexTri.lastDeepestInTriangle = false;
    ConvexTri.lastDeepestOutsideDist = Infinity;
    // 'separated' = provably no contact, caller may skip the GJK/EPA fallback. 'maybe' = no face
    // contact but an edge/vertex hit is still possible.
    ConvexTri.lastVerdict = 'maybe';

    // Deepest convex point toward the triangle = support along -refN.
    const probeDir = ConvexTri._probeDir;
    const dp = ConvexTri._dp;
    const invRot = ConvexTri._invRot.copy(cvxPlaced.rotation).invert();
    const scratchDir = ConvexTri._scratchDir;

    // PairTest.step discards contacts past the pair's margin, so past that the GJK/EPA fallback is
    // wasted work. Marginless callers keep the flat SEPARATION_LIMIT.
    const cullLimit = (margin === undefined || margin === null)
        ? ConvexTri.SEPARATION_LIMIT
        : Math.min(ConvexTri.SEPARATION_LIMIT, Math.max(margin, ConvexTri.MIN_CULL_LIMIT));

    probeDir.set(-refN.x, -refN.y, -refN.z);
    MinkowskiSupport.supportOfInto(dp, cvxPlaced, invRot, probeDir, scratchDir);
    const gap = (dp.x - t0.x) * refN.x + (dp.y - t0.y) * refN.y + (dp.z - t0.z) * refN.z;
    if (gap > ConvexTri.SEPARATION_LIMIT) {
        // SEPARATION_LIMIT, not cullLimit: this branch also gates whether the pair gets a
        // speculative face contact, so narrowing it changes trajectories.
        ConvexTri.lastVerdict = 'separated';
        return null;
    }
    if (gap < -ConvexTri.PENETRATION_LIMIT) return null; // deep behind - let GJK/EPA judge

    // The deepest point, plus tilted probes giving a flat-based or side-lying convex a multi-point
    // patch. Probe directions come from the convex's own frame, not the triangle's, so a prop
    // straddling a seam gets the same set from every triangle under it.
    //
    // A sphere takes the deepest point alone: it touches a plane at one point, and a probe tilted
    // PROBE_TILT off the normal lands r*(1-cos(TILT)) off the plane - far past DEDUPE_DIST_SQ, so
    // the probes would survive as a phantom flat base.
    const cand = ConvexTri._cand;
    let nCand = 0;
    cand[nCand++].copy(dp);

    // Tangent basis for refN, chosen deterministically from refN alone so it does not rotate with
    // the body or the triangle.
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
        // The deepest point projects outside the triangle, so its nearest feature is an edge and
        // hypot(along-plane gap, how-far-outside) lower-bounds the distance. Clearing the
        // speculative band by that bound proves separation and lets the caller skip GJK/EPA.
        const outside = ConvexTri.lastDeepestOutsideDist;
        const along = gap > 0 ? gap : 0;
        if (isFinite(outside) && Math.sqrt(along * along + outside * outside) > cullLimit) {
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
// One slot for the deepest point plus one per probe.
ConvexTri._cand = [];
for (var _ci = 0; _ci <= ConvexTri.PROBE_COUNT; _ci++) ConvexTri._cand.push(new Vector3());
ConvexTri.lastDeepestInTriangle = false;
ConvexTri.lastDeepestOutsideDist = Infinity;
ConvexTri._lastOutside = 0;
ConvexTri.lastVerdict = 'maybe';
// Metres the deepest point may sit outside a stored triangle and still let the fast re-clip trust
// it: covers jitter and a shallow overhang, not a slide onto the neighbouring tile.
ConvexTri.REFRESH_DRIFT_TOLERANCE = 0.35;

ActionPhysics.ConvexTri = ConvexTri;
