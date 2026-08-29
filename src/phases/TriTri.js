// Closed-form triangle-triangle FACE contact: when two triangles are near-coplanar and near-
// touching, clip the incident triangle against the reference triangle's 3 edge half-planes
// (Sutherland-Hodgman in the reference plane) and emit one contact per surviving vertex - a real
// multi-point manifold spread across the shared face area.
//
// WHY THIS EXISTS: a MeshShape's midphase dispatches a flat box-on-box contact as up to four
// triangle-vs-triangle pairs (two triangles per box face). Routed through GJK/EPA, each pair yields
// exactly ONE witness point, and EPA consistently picks the shared diagonal edge/vertex the two
// face triangles have in common - so all four points collapse onto one edge of the square face
// instead of spanning it. The solver then pushes up on a single edge and the box pitches away from
// a drop that imparted zero torque. Clipping produces the full contact polygon instead.
//
// Only the near-parallel (anti-parallel normals), near-touching FACE case produces contacts here.
// Most other pairs return null and fall through to GJK/EPA in PairTest.js, like BoxBox.test on a
// separated pair. The one exception: a near-perpendicular pair sharing a coincident edge without
// interpenetrating (a mesh box's top face triangle and its own side wall) returns an EMPTY result,
// which vetoes the GJK/EPA fallback - that shared edge is not a contact. Mirrors
// BoxBox._buildFaceContact for the clipping half.
const TriTri = {};

TriTri.applies = function (placedA, placedB) {
    return placedA.shape instanceof TriangleShape && placedB.shape instanceof TriangleShape;
};

// dot(nA, nB) below this (i.e. the two outward normals are anti-parallel to within ~2.5 degrees)
// => a genuine opposing-face pair: one triangle's front faces the other's. A POSITIVE dot near +1
// is two co-oriented coincident faces (e.g. the two boxes' shared +z side walls when perfectly
// aligned) - not a contact, must not be clipped. Anything in between is an edge/point feature for
// GJK/EPA.
TriTri.ANTIPARALLEL_DOT = -0.999;
// How far in FRONT of the reference plane (still separated) the incident triangle is trusted to
// report a speculative face contact - kept generous so all clipped points surface together on the
// approach tick (same reasoning as BoxBox.SEPARATED_AXIS_LIMIT); PairTest.js's speculative margin
// does the real filtering.
TriTri.SEPARATION_LIMIT = 0.5;
// How far BEHIND the reference plane (penetrating) an opposing-face pair is still resolved as a
// face contact. Generous: the clip manifold is exactly what pushes an over-penetrated stack back
// apart, so bailing to GJK/EPA's single point here would reintroduce the torque bug under load.
// A box's vertical SIDE faces are co-ORIENTED (normals parallel, not anti-parallel) when two boxes
// align, so they never reach this branch - they are vetoed earlier by the ndot test.
TriTri.PENETRATION_LIMIT = 1.0;
// A clipped overlap polygon whose area is below this (m^2) is a glancing edge sliver - typically
// the near-zero overlap of two box-face triangles split along OPPOSITE diagonals, which otherwise
// emits a row of points strung along the face diagonal and biases the manifold. The real overlap
// from the same-diagonal triangle pair is a full half-face and clears this easily; a genuine small
// face contact below this threshold is left to GJK/EPA.
TriTri.MIN_AREA = 0.02;
// |dot(nA,nB)| below this counts as "near perpendicular" for the shared-edge veto.
TriTri.PERPENDICULAR_DOT = 0.25;
// Two vertices of one triangle within this distance (meters) of the other triangle's plane, AND
// projecting inside its edges, count as a coincident shared edge.
TriTri.EDGE_COINCIDENCE = 1e-3;
// Degenerate triangle (near-zero area) normal guard.
TriTri.AREA_EPSILON = 1e-12;

// Unit outward normal of a world-space triangle into `out`. Returns false if degenerate.
TriTri._normalInto = function (out, a, b, c) {
    const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
    let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
    const lenSq = nx * nx + ny * ny + nz * nz;
    if (lenSq < TriTri.AREA_EPSILON) return false;
    const inv = 1 / Math.sqrt(lenSq);
    out.set(nx * inv, ny * inv, nz * inv);
    return true;
};

// Does segment p->q pierce triangle (t0,t1,t2) with unit normal tn, strictly between the endpoints?
TriTri._segmentHitsTri = function (px, py, pz, qx, qy, qz, t0, t1, t2, tn) {
    const dx = qx - px, dy = qy - py, dz = qz - pz;
    const denom = dx * tn.x + dy * tn.y + dz * tn.z;
    if (denom > -1e-12 && denom < 1e-12) return false; // parallel to the triangle plane
    const t = ((t0.x - px) * tn.x + (t0.y - py) * tn.y + (t0.z - pz) * tn.z) / denom;
    if (t <= 1e-9 || t >= 1 - 1e-9) return false; // hit is at/beyond an endpoint, not a true crossing
    const hx = px + dx * t, hy = py + dy * t, hz = pz + dz * t;
    // Barycentric inside-test against the three edges.
    const e0x = t1.x - t0.x, e0y = t1.y - t0.y, e0z = t1.z - t0.z;
    const e1x = t2.x - t1.x, e1y = t2.y - t1.y, e1z = t2.z - t1.z;
    const e2x = t0.x - t2.x, e2y = t0.y - t2.y, e2z = t0.z - t2.z;
    const c0x = (hx - t0.x), c0y = (hy - t0.y), c0z = (hz - t0.z);
    const c1x = (hx - t1.x), c1y = (hy - t1.y), c1z = (hz - t1.z);
    const c2x = (hx - t2.x), c2y = (hy - t2.y), c2z = (hz - t2.z);
    const d0 = tn.x * (e0y * c0z - e0z * c0y) + tn.y * (e0z * c0x - e0x * c0z) + tn.z * (e0x * c0y - e0y * c0x);
    const d1 = tn.x * (e1y * c1z - e1z * c1y) + tn.y * (e1z * c1x - e1x * c1z) + tn.z * (e1x * c1y - e1y * c1x);
    const d2 = tn.x * (e2y * c2z - e2z * c2y) + tn.y * (e2z * c2x - e2x * c2z) + tn.z * (e2x * c2y - e2y * c2x);
    return (d0 >= 0 && d1 >= 0 && d2 >= 0) || (d0 <= 0 && d1 <= 0 && d2 <= 0);
};

// Point-in-triangle test (point already assumed on the triangle's plane; tn is the unit normal).
TriTri._pointInTri = function (hx, hy, hz, t0, t1, t2, tn) {
    const e0x = t1.x - t0.x, e0y = t1.y - t0.y, e0z = t1.z - t0.z;
    const e1x = t2.x - t1.x, e1y = t2.y - t1.y, e1z = t2.z - t1.z;
    const e2x = t0.x - t2.x, e2y = t0.y - t2.y, e2z = t0.z - t2.z;
    const c0x = hx - t0.x, c0y = hy - t0.y, c0z = hz - t0.z;
    const c1x = hx - t1.x, c1y = hy - t1.y, c1z = hz - t1.z;
    const c2x = hx - t2.x, c2y = hy - t2.y, c2z = hz - t2.z;
    const d0 = tn.x * (e0y * c0z - e0z * c0y) + tn.y * (e0z * c0x - e0x * c0z) + tn.z * (e0x * c0y - e0y * c0x);
    const d1 = tn.x * (e1y * c1z - e1z * c1y) + tn.y * (e1z * c1x - e1x * c1z) + tn.z * (e1x * c1y - e1y * c1x);
    const d2 = tn.x * (e2y * c2z - e2z * c2y) + tn.y * (e2z * c2x - e2x * c2z) + tn.z * (e2x * c2y - e2y * c2x);
    return (d0 >= 0 && d1 >= 0 && d2 >= 0) || (d0 <= 0 && d1 <= 0 && d2 <= 0);
};

// True if the two triangles lie flat against each other along a shared edge: at least two vertices
// of one triangle sit on the other's plane (within EDGE_COINCIDENCE) and project inside it.
TriTri._sharesCoincidentEdge = function (a0, a1, a2, nA, b0, b1, b2, nB) {
    return TriTri._countOnPlane(b0, b1, b2, a0, a1, a2, nA) >= 2 ||
        TriTri._countOnPlane(a0, a1, a2, b0, b1, b2, nB) >= 2;
};
TriTri._countOnPlane = function (p0, p1, p2, t0, t1, t2, tn) {
    const pts = [p0, p1, p2];
    let n = 0;
    for (let i = 0; i < 3; i++) {
        const p = pts[i];
        const d = (p.x - t0.x) * tn.x + (p.y - t0.y) * tn.y + (p.z - t0.z) * tn.z;
        if (d < -TriTri.EDGE_COINCIDENCE || d > TriTri.EDGE_COINCIDENCE) continue;
        if (TriTri._pointInTri(p.x - d * tn.x, p.y - d * tn.y, p.z - d * tn.z, t0, t1, t2, tn)) n++;
    }
    return n;
};

// True if the two triangles genuinely interpenetrate (some edge of one pierces the interior of the
// other). Coincident-edge / shared-boundary touching is NOT an intersection.
TriTri._trianglesIntersect = function (a0, a1, a2, nA, b0, b1, b2, nB) {
    return TriTri._segmentHitsTri(a0.x, a0.y, a0.z, a1.x, a1.y, a1.z, b0, b1, b2, nB) ||
        TriTri._segmentHitsTri(a1.x, a1.y, a1.z, a2.x, a2.y, a2.z, b0, b1, b2, nB) ||
        TriTri._segmentHitsTri(a2.x, a2.y, a2.z, a0.x, a0.y, a0.z, b0, b1, b2, nB) ||
        TriTri._segmentHitsTri(b0.x, b0.y, b0.z, b1.x, b1.y, b1.z, a0, a1, a2, nA) ||
        TriTri._segmentHitsTri(b1.x, b1.y, b1.z, b2.x, b2.y, b2.z, a0, a1, a2, nA) ||
        TriTri._segmentHitsTri(b2.x, b2.y, b2.z, b0.x, b0.y, b0.z, a0, a1, a2, nA);
};

// out: array to push pooled ContactDetails into. Returns out on success (possibly empty, which
// vetoes the GJK/EPA fallback), or null to fall through to GJK/EPA.
TriTri.test = function (placedA, placedB, out, nextContact) {
    const sA = placedA.shape, sB = placedB.shape;
    const a0 = sA.a, a1 = sA.b, a2 = sA.c;
    const b0 = sB.a, b1 = sB.b, b2 = sB.c;

    const nA = TriTri._nA, nB = TriTri._nB;
    if (!TriTri._normalInto(nA, a0, a1, a2)) return null;
    if (!TriTri._normalInto(nB, b0, b1, b2)) return null;

    const ndot = nA.x * nB.x + nA.y * nB.y + nA.z * nB.z;

    if (ndot > TriTri.ANTIPARALLEL_DOT) {
        // Not an opposing-face pair. The one case TriTri must actively suppress here is a NEARLY
        // PERPENDICULAR pair that only shares a COINCIDENT EDGE and does not interpenetrate - e.g. a
        // box's own top-face triangle and the vertical side-wall triangle it shares an edge with,
        // when another box rests on that top face. GJK/EPA reports that shared edge as a "touch"
        // with an arbitrary (often horizontal) normal, and mixed into the body-pair manifold it
        // injects torque from nothing. Veto it (return the empty result, stopping the GJK/EPA
        // fallback). Every other non-parallel pair - oblique facets of a curved mesh near a contact,
        // genuine edge-first collisions - is left to GJK/EPA (return null).
        if (Math.abs(ndot) < TriTri.PERPENDICULAR_DOT &&
            TriTri._sharesCoincidentEdge(a0, a1, a2, nA, b0, b1, b2, nB) &&
            !TriTri._trianglesIntersect(a0, a1, a2, nA, b0, b1, b2, nB)) {
            return out;
        }
        return null;
    }

    // refN = triangle A's face normal, oriented to point FROM A's face TOWARD B (the A->B
    // direction; the pipeline's B->A normal is its negation, applied at emit). A MeshShape's
    // winding is not a reliable "outward" cue - inverted-winding meshes (used deliberately in the
    // mesh tests) have their triangle normals pointing INTO the body. Resolve the sign from the two
    // OWNING BODY centers when the midphase provided them (bodyCenter): they sit ~a body-width apart
    // and never flicker, unlike the two exactly-flush face triangles' own out-of-plane offset,
    // whose sign is float noise and would flip the contact normal tick to tick on a settled stack.
    // Fall back to B's farthest vertex when the hint is absent (standalone TriangleShape bodies).
    const refN = TriTri._refN;
    refN.copy(nA);
    const cenA = placedA.bodyCenter, cenB = placedB.bodyCenter;
    if (cenA && cenB) {
        if ((cenB.x - cenA.x) * nA.x + (cenB.y - cenA.y) * nA.y + (cenB.z - cenA.z) * nA.z < 0) refN.scaleInPlace(-1);
    } else {
        let farAbs = -1, farSigned = 0;
        for (let k = 0; k < 3; k++) {
            const v = k === 0 ? b0 : (k === 1 ? b1 : b2);
            const d = (v.x - a0.x) * nA.x + (v.y - a0.y) * nA.y + (v.z - a0.z) * nA.z;
            if (Math.abs(d) > farAbs) { farAbs = Math.abs(d); farSigned = d; }
        }
        if (farSigned < 0) refN.scaleInPlace(-1);
    }

    // Signed gap of B's centroid from A's plane along -refN (the penetrating side). Reject a pair
    // that is clearly separated, or so deep it is no longer a face contact.
    const cBx = (b0.x + b1.x + b2.x) / 3, cBy = (b0.y + b1.y + b2.y) / 3, cBz = (b0.z + b1.z + b2.z) / 3;
    const centroidGap = -((cBx - a0.x) * refN.x + (cBy - a0.y) * refN.y + (cBz - a0.z) * refN.z);
    if (centroidGap < -TriTri.SEPARATION_LIMIT || centroidGap > TriTri.PENETRATION_LIMIT) return null;

    // Clip triangle B (incident) against triangle A's three edge half-planes, evaluated in A's
    // plane. Each edge (a_i -> a_{i+1}) gives an inward half-plane with normal = refN x edge,
    // oriented so A's opposite vertex is inside.
    let poly = TriTri._polyIn, clipped = TriTri._polyOut;
    poly[0].set(b0.x, b0.y, b0.z);
    poly[1].set(b1.x, b1.y, b1.z);
    poly[2].set(b2.x, b2.y, b2.z);
    let polyCount = 3;

    const aVerts = TriTri._aVerts;
    aVerts[0] = a0; aVerts[1] = a1; aVerts[2] = a2;

    for (let e = 0; e < 3; e++) {
        const p = aVerts[e], q = aVerts[(e + 1) % 3], r = aVerts[(e + 2) % 3];
        const ex = q.x - p.x, ey = q.y - p.y, ez = q.z - p.z;
        // Inward normal of this edge's half-plane, lying in A's plane.
        let hx = refN.y * ez - refN.z * ey;
        let hy = refN.z * ex - refN.x * ez;
        let hz = refN.x * ey - refN.y * ex;
        // Orient so the third vertex r is on the positive side.
        if ((r.x - p.x) * hx + (r.y - p.y) * hy + (r.z - p.z) * hz < 0) { hx = -hx; hy = -hy; hz = -hz; }

        let outCount = 0;
        for (let c = 0; c < polyCount; c++) {
            const cur = poly[c], next = poly[(c + 1) % polyCount];
            const curD = (cur.x - p.x) * hx + (cur.y - p.y) * hy + (cur.z - p.z) * hz;
            const nextD = (next.x - p.x) * hx + (next.y - p.y) * hy + (next.z - p.z) * hz;
            const curIn = curD >= 0, nextIn = nextD >= 0;
            if (curIn) clipped[outCount++].copy(cur);
            if (curIn !== nextIn) {
                const t = curD / (curD - nextD);
                clipped[outCount++].set(
                    cur.x + (next.x - cur.x) * t,
                    cur.y + (next.y - cur.y) * t,
                    cur.z + (next.z - cur.z) * t
                );
            }
        }
        polyCount = outCount;
        const swap = poly; poly = clipped; clipped = swap;
        if (polyCount < 3) return null; // no in-plane overlap area - GJK/EPA can still report a gap
    }

    // Reject a sliver overlap (glancing edge touch between two parallel triangles): fan-triangulate
    // the clipped polygon and sum the area.
    let area2 = 0;
    for (let c = 1; c < polyCount - 1; c++) {
        const p0 = poly[0], p1 = poly[c], p2 = poly[c + 1];
        const ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z;
        const vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.z - p0.z;
        const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
        area2 += Math.sqrt(cx * cx + cy * cy + cz * cz);
    }
    if (area2 * 0.5 < TriTri.MIN_AREA) return null;

    // Emit one contact per surviving polygon vertex, keeping only points near A's plane (a real or
    // imminent face contact). refN points A->B, so `below` = distance on the -refN side of A's
    // plane is the penetration; negative `below` = still separated.
    let emitted = 0;
    for (let c = 0; c < polyCount; c++) {
        const pt = poly[c];
        const below = -((pt.x - a0.x) * refN.x + (pt.y - a0.y) * refN.y + (pt.z - a0.z) * refN.z);
        if (below < -TriTri.SEPARATION_LIMIT || below > TriTri.PENETRATION_LIMIT) continue;

        const contact = nextContact();
        // Incident witness = the clipped vertex (on/near B's front). Reference witness = that point
        // pushed back onto A's plane along -refN (refN points A->B, so A's surface is `below` behind
        // the incident point in the -refN direction... i.e. + refN * below lands back on the plane).
        contact.pointOnB.set(pt.x, pt.y, pt.z);
        contact.pointOnA.set(pt.x + refN.x * below, pt.y + refN.y * below, pt.z + refN.z * below);
        // Pipeline convention: normal points B -> A. refN points A -> B, so negate.
        contact.normal.set(-refN.x, -refN.y, -refN.z);
        contact.signedDistance = below;
        contact.fromMeshFace = true; // enables the manifold's shared-corner coincidence merge
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
        emitted++;
    }

    if (emitted === 0) return null;
    return out;
};

TriTri._nA = new Vector3();
TriTri._nB = new Vector3();
TriTri._refN = new Vector3();
TriTri._aVerts = [null, null, null];
// Sutherland-Hodgman ping-pong buffers. Clipping a triangle by 3 half-planes yields at most 6
// vertices; 8 slots for headroom.
TriTri._polyIn = [new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3()];
TriTri._polyOut = [new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3()];

ActionPhysics.TriTri = TriTri;
