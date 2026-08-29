// Closed-form triangle-triangle face contact: clip the incident triangle against the reference
// triangle's edge half-planes (Sutherland-Hodgman) and emit one contact per surviving vertex.
// Routing a flat mesh face contact through GJK/EPA instead gives one witness point per triangle
// pair, all landing on the shared diagonal - a single-edge manifold that torques a flat drop.
const TriTri = {};

TriTri.applies = function (placedA, placedB) {
    return placedA.shape instanceof TriangleShape && placedB.shape instanceof TriangleShape;
};

// Opposing-face pair: outward normals anti-parallel to within ~2.5 deg.
TriTri.ANTIPARALLEL_DOT = -0.999;
TriTri.SEPARATION_LIMIT = 0.5;  // report a speculative face contact up to this gap in front of A's plane
TriTri.PENETRATION_LIMIT = 1.0; // keep resolving as a face contact up to this depth behind it
TriTri.MIN_AREA = 0.02;         // below this the clipped overlap is a sliver, not a face
TriTri.PERPENDICULAR_DOT = 0.25;
TriTri.EDGE_COINCIDENCE = 1e-3;
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

// Does segment p->q pierce triangle (t0,t1,t2) strictly between its endpoints?
TriTri._segmentHitsTri = function (px, py, pz, qx, qy, qz, t0, t1, t2, tn) {
    const dx = qx - px, dy = qy - py, dz = qz - pz;
    const denom = dx * tn.x + dy * tn.y + dz * tn.z;
    if (denom > -1e-12 && denom < 1e-12) return false;
    const t = ((t0.x - px) * tn.x + (t0.y - py) * tn.y + (t0.z - pz) * tn.z) / denom;
    if (t <= 1e-9 || t >= 1 - 1e-9) return false;
    return TriTri._pointInTri(px + dx * t, py + dy * t, pz + dz * t, t0, t1, t2, tn);
};

// Point-in-triangle, point assumed on the triangle's plane.
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

// Some edge of one triangle pierces the interior of the other. Shared-boundary touching does not count.
TriTri._trianglesIntersect = function (a0, a1, a2, nA, b0, b1, b2, nB) {
    return TriTri._segmentHitsTri(a0.x, a0.y, a0.z, a1.x, a1.y, a1.z, b0, b1, b2, nB) ||
        TriTri._segmentHitsTri(a1.x, a1.y, a1.z, a2.x, a2.y, a2.z, b0, b1, b2, nB) ||
        TriTri._segmentHitsTri(a2.x, a2.y, a2.z, a0.x, a0.y, a0.z, b0, b1, b2, nB) ||
        TriTri._segmentHitsTri(b0.x, b0.y, b0.z, b1.x, b1.y, b1.z, a0, a1, a2, nA) ||
        TriTri._segmentHitsTri(b1.x, b1.y, b1.z, b2.x, b2.y, b2.z, a0, a1, a2, nA) ||
        TriTri._segmentHitsTri(b2.x, b2.y, b2.z, b0.x, b0.y, b0.z, a0, a1, a2, nA);
};

// Returns `out` on success (may be empty, which vetoes the GJK/EPA fallback in PairTest), or null
// to fall through to GJK/EPA.
TriTri.test = function (placedA, placedB, out, nextContact) {
    const sA = placedA.shape, sB = placedB.shape;
    const a0 = sA.a, a1 = sA.b, a2 = sA.c;
    const b0 = sB.a, b1 = sB.b, b2 = sB.c;

    const nA = TriTri._nA, nB = TriTri._nB;
    if (!TriTri._normalInto(nA, a0, a1, a2)) return null;
    if (!TriTri._normalInto(nB, b0, b1, b2)) return null;

    const ndot = nA.x * nB.x + nA.y * nB.y + nA.z * nB.z;

    if (ndot > TriTri.ANTIPARALLEL_DOT) {
        // Near-perpendicular pair meeting edge-on without interpenetrating (a box's side wall and
        // the top face another box rests on). GJK/EPA would report that shared edge as a contact
        // with an arbitrary horizontal normal; veto it. Real edge-first collisions interpenetrate
        // and reach GJK/EPA via the null return.
        if (Math.abs(ndot) < TriTri.PERPENDICULAR_DOT &&
            !TriTri._trianglesIntersect(a0, a1, a2, nA, b0, b1, b2, nB)) {
            return out;
        }
        return null;
    }

    // refN = A's face normal oriented A->B. Winding is unreliable (inverted-winding meshes point
    // their normals inward), so take the sign from the owning body centers when the midphase
    // provided them; the two flush face triangles' own offset is float noise and flips tick to tick.
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

    const cBx = (b0.x + b1.x + b2.x) / 3, cBy = (b0.y + b1.y + b2.y) / 3, cBz = (b0.z + b1.z + b2.z) / 3;
    const centroidGap = -((cBx - a0.x) * refN.x + (cBy - a0.y) * refN.y + (cBz - a0.z) * refN.z);
    if (centroidGap < -TriTri.SEPARATION_LIMIT || centroidGap > TriTri.PENETRATION_LIMIT) return null;

    // Clip B against A's three edge half-planes (each with inward normal refN x edge).
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
        let hx = refN.y * ez - refN.z * ey;
        let hy = refN.z * ex - refN.x * ez;
        let hz = refN.x * ey - refN.y * ex;
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
        if (polyCount < 3) return null;
    }

    let area2 = 0;
    for (let c = 1; c < polyCount - 1; c++) {
        const p0 = poly[0], p1 = poly[c], p2 = poly[c + 1];
        const ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z;
        const vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.z - p0.z;
        const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
        area2 += Math.sqrt(cx * cx + cy * cy + cz * cz);
    }
    if (area2 * 0.5 < TriTri.MIN_AREA) return null;

    let emitted = 0;
    for (let c = 0; c < polyCount; c++) {
        const pt = poly[c];
        const below = -((pt.x - a0.x) * refN.x + (pt.y - a0.y) * refN.y + (pt.z - a0.z) * refN.z);
        if (below < -TriTri.SEPARATION_LIMIT || below > TriTri.PENETRATION_LIMIT) continue;

        const contact = nextContact();
        contact.pointOnB.set(pt.x, pt.y, pt.z);
        contact.pointOnA.set(pt.x + refN.x * below, pt.y + refN.y * below, pt.z + refN.z * below);
        contact.normal.set(-refN.x, -refN.y, -refN.z); // pipeline convention: B -> A
        contact.signedDistance = below;
        contact.fromMeshFace = true;
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
TriTri._polyIn = [new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3()];
TriTri._polyOut = [new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3()];

ActionPhysics.TriTri = TriTri;
