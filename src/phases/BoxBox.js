// Closed-form box-box: 15-axis SAT (3+3 face normals, 9 edge-cross axes) picks the minimum-
// penetration separating axis, then either clips the incident face against the reference face's
// side planes (face contact, up to 4 points) or takes the closest points between the two
// contributing edges (edge-edge contact, 1 point). Own epsilon, no shared GJK/EPA state.
const BoxBox = {};

BoxBox.applies = function (placedA, placedB) {
    return placedA.shape instanceof BoxShape && placedB.shape instanceof BoxShape;
};

// An edge-edge axis only beats the best face axis if it wins by more than this fraction of the
// face overlap. Guards face-to-face resting boxes, where an edge axis can numerically tie a face
// axis and flicker the manifold between 4 points and 1 tick to tick.
BoxBox.RELATIVE_TOLERANCE = 0.25;
// Absolute floor under the tie-break: near exact touch, RELATIVE_TOLERANCE * faceOverlap vanishes
// and float noise alone would pick the edge branch.
BoxBox.ABSOLUTE_TOLERANCE = 1e-6;
// Edge-cross axes below this squared length are near-parallel edges (degenerate axis).
BoxBox.PARALLEL_EPSILON = 1e-9;
// How far a still-separated pair is trusted to report a speculative contact via SAT before falling
// through to GJK/EPA. Generous; PairTest.step does the real speculative-margin filtering.
BoxBox.SEPARATED_AXIS_LIMIT = 1.0;

// out: array to push ContactDetails into (pooled via nextContact()). Returns out.
BoxBox.test = function (placedA, placedB, out, nextContact) {
    const a = placedA.shape, b = placedB.shape;
    const posA = placedA.position, posB = placedB.position;
    const rotA = placedA.rotation, rotB = placedB.rotation;

    const ax = BoxBox._axesA, bx = BoxBox._axesB;
    ax[0].set(1, 0, 0); rotA.transformVectorInPlace(ax[0]);
    ax[1].set(0, 1, 0); rotA.transformVectorInPlace(ax[1]);
    ax[2].set(0, 0, 1); rotA.transformVectorInPlace(ax[2]);
    bx[0].set(1, 0, 0); rotB.transformVectorInPlace(bx[0]);
    bx[1].set(0, 1, 0); rotB.transformVectorInPlace(bx[1]);
    bx[2].set(0, 0, 1); rotB.transformVectorInPlace(bx[2]);

    const halfA = BoxBox._halfA, halfB = BoxBox._halfB;
    halfA[0] = a.halfWidth; halfA[1] = a.halfHeight; halfA[2] = a.halfDepth;
    halfB[0] = b.halfWidth; halfB[1] = b.halfHeight; halfB[2] = b.halfDepth;

    const d = BoxBox._d;
    Vector3.subInto(d, posB, posA);

    // R[i][j] = ax[i] . bx[j]; absR adds a small epsilon (standard SAT robustness fix so a
    // near-parallel pair of face axes doesn't zero out a projected extent).
    const R = BoxBox._R, absR = BoxBox._absR;
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            R[i][j] = ax[i].dot(bx[j]);
            absR[i][j] = Math.abs(R[i][j]) + 1e-9;
        }
    }

    const dA = [d.dot(ax[0]), d.dot(ax[1]), d.dot(ax[2])];
    const dB = [d.dot(bx[0]), d.dot(bx[1]), d.dot(bx[2])];

    // Least-overlap axis across all 15 candidates, whether or not any is separating: a negative
    // overlap is the gap, which PairTest.step compares against the speculative margin. No early
    // bail on the first separating axis - that would drop every speculative box-box contact.
    let minOverlap = Infinity;
    let bestAxisType = -1; // 0 = face of A, 1 = face of B
    let bestI = -1, bestSign = 1;

    // Face axes of A (3).
    for (let i = 0; i < 3; i++) {
        const ra = halfA[i];
        const rb = halfB[0] * absR[i][0] + halfB[1] * absR[i][1] + halfB[2] * absR[i][2];
        const overlap = ra + rb - Math.abs(dA[i]);
        if (overlap < minOverlap) { minOverlap = overlap; bestAxisType = 0; bestI = i; bestSign = dA[i] >= 0 ? 1 : -1; }
    }
    // Face axes of B (3). bestSign is flipped vs A's: d = posB - posA, so d.dot(bx[j]) >= 0 means
    // A sits on B's -bx[j] side, making that the reference face.
    for (let j = 0; j < 3; j++) {
        const rb = halfB[j];
        const ra = halfA[0] * absR[0][j] + halfA[1] * absR[1][j] + halfA[2] * absR[2][j];
        const overlap = ra + rb - Math.abs(dB[j]);
        if (overlap < minOverlap) { minOverlap = overlap; bestAxisType = 1; bestI = j; bestSign = dB[j] >= 0 ? -1 : 1; }
    }

    const faceOverlap = minOverlap;

    // Edge-edge axes: ax[i] x bx[j], for all 9 combinations.
    let bestEdgeOverlap = Infinity, bestEdgeI = -1, bestEdgeJ = -1;
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            const axis = BoxBox._edgeAxis;
            Vector3.crossInto(axis, ax[i], bx[j]);
            const axisLenSq = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
            if (axisLenSq < BoxBox.PARALLEL_EPSILON) continue;

            axis.scaleInPlace(1 / Math.sqrt(axisLenSq));

            const dist = Math.abs(d.dot(axis));
            let ra = 0, rb = 0;
            for (let k = 0; k < 3; k++) {
                ra += halfA[k] * Math.abs(ax[k].dot(axis));
                rb += halfB[k] * Math.abs(bx[k].dot(axis));
            }
            const overlap = ra + rb - dist;
            if (overlap < bestEdgeOverlap) { bestEdgeOverlap = overlap; bestEdgeI = i; bestEdgeJ = j; }
        }
    }

    // Prefer the face axis unless an edge axis is a clearly tighter fit, and only take the edge
    // branch when the boxes actually overlap. While separated, SAT's "largest gap wins" rule picks
    // an edge-cross axis over the true face axis under even modest relative tilt, collapsing a
    // speculative face contact into a degenerate edge point that warm-starts wrong. Scale the margin
    // by |faceOverlap| so it behaves the same overlapping or separated. A real corner/edge-first
    // collision (box-box/corner-drop) still clears this and takes the edge branch.
    const tieBreakMargin = Math.max(BoxBox.RELATIVE_TOLERANCE * Math.abs(faceOverlap), BoxBox.ABSOLUTE_TOLERANCE);
    if (faceOverlap >= 0 && bestEdgeI >= 0 && bestEdgeOverlap < faceOverlap - tieBreakMargin) {
        BoxBox._buildEdgeContact(placedA, placedB, ax, bx, halfA, halfB, posA, posB,
            bestEdgeI, bestEdgeJ, bestEdgeOverlap, out, nextContact);
        return out;
    }

    // Too far apart for face clipping to mean anything - let GJK/EPA handle it.
    if (faceOverlap < -BoxBox.SEPARATED_AXIS_LIMIT) return null;

    BoxBox._buildFaceContact(placedA, placedB, ax, bx, halfA, halfB, posA, posB,
        bestAxisType, bestI, bestSign, out, nextContact);
    return out;
};

// Edge-edge contact: closest points between the two axis-aligned segments (A's edge i, B's edge
// j), each a line through its box center along that local axis, clamped to the box's own
// half-extent on the other two axes (the edge nearest the other box, not just any parallel edge).
BoxBox._buildEdgeContact = function (placedA, placedB, ax, bx, halfA, halfB, posA, posB, i, j, overlap, out, nextContact) {
    // Edge i of A: pick the two non-i axes' signs from which side of A the segment nearest B sits on.
    const d = BoxBox._d; // still B - A from test()
    const otherA1 = (i + 1) % 3, otherA2 = (i + 2) % 3;
    const signA1 = d.dot(ax[otherA1]) >= 0 ? 1 : -1;
    const signA2 = d.dot(ax[otherA2]) >= 0 ? 1 : -1;

    const pA = BoxBox._pA, uA = BoxBox._uA;
    pA.copy(posA);
    pA.x += ax[otherA1].x * halfA[otherA1] * signA1 + ax[otherA2].x * halfA[otherA2] * signA2;
    pA.y += ax[otherA1].y * halfA[otherA1] * signA1 + ax[otherA2].y * halfA[otherA2] * signA2;
    pA.z += ax[otherA1].z * halfA[otherA1] * signA1 + ax[otherA2].z * halfA[otherA2] * signA2;
    uA.copy(ax[i]);

    const otherB1 = (j + 1) % 3, otherB2 = (j + 2) % 3;
    const signB1 = -d.dot(bx[otherB1]) >= 0 ? 1 : -1;
    const signB2 = -d.dot(bx[otherB2]) >= 0 ? 1 : -1;

    const pB = BoxBox._pB, uB = BoxBox._uB;
    pB.copy(posB);
    pB.x += bx[otherB1].x * halfB[otherB1] * signB1 + bx[otherB2].x * halfB[otherB2] * signB2;
    pB.y += bx[otherB1].y * halfB[otherB1] * signB1 + bx[otherB2].y * halfB[otherB2] * signB2;
    pB.z += bx[otherB1].z * halfB[otherB1] * signB1 + bx[otherB2].z * halfB[otherB2] * signB2;
    uB.copy(bx[j]);

    // Closest points between the two infinite lines pA + s*uA and pB + t*uB, clamped to each
    // edge's own half-extent along i / j respectively (standard segment-segment closest point).
    const r = BoxBox._segR;
    Vector3.subInto(r, pA, pB);
    const uu = uA.dot(uA), uv = uA.dot(uB), vv = uB.dot(uB);
    const ur = uA.dot(r), vr = uB.dot(r);
    const denom = uu * vv - uv * uv;

    let s = Math.abs(denom) > 1e-9 ? (uv * vr - vv * ur) / denom : 0;
    let t = (uv * s + vr) / vv;
    s = Math.max(-halfA[i], Math.min(halfA[i], s));
    t = Math.max(-halfB[j], Math.min(halfB[j], t));

    const closestA = BoxBox._closestA, closestB = BoxBox._closestB;
    closestA.set(pA.x + uA.x * s, pA.y + uA.y * s, pA.z + uA.z * s);
    closestB.set(pB.x + uB.x * t, pB.y + uB.y * t, pB.z + uB.z * t);

    // Normal: the edge-cross axis, oriented A -> B then flipped to the pipeline's B -> A convention.
    const normal = BoxBox._contactNormal;
    Vector3.crossInto(normal, uA, uB);
    const lenSq = normal.x * normal.x + normal.y * normal.y + normal.z * normal.z;
    if (lenSq > BoxBox.PARALLEL_EPSILON) normal.scaleInPlace(1 / Math.sqrt(lenSq));
    else normal.set(0, 1, 0);
    if (normal.dot(d) < 0) normal.scaleInPlace(-1); // point from A toward B first...
    normal.scaleInPlace(-1); // ...then flip to B -> A, matching SphereSphere/SphereBox's convention.

    const contact = nextContact();
    contact.pointOnA.copy(closestA);
    contact.pointOnB.copy(closestB);
    contact.normal.copy(normal);
    contact.signedDistance = overlap;
    Vector3.addInto(contact.point, closestA, closestB).scaleInPlace(0.5);
    out.push(contact);
};

// Face contact: clip the incident face (the face of the non-reference box most anti-parallel to
// the reference normal) against the reference face's 4 side planes (Sutherland-Hodgman), then keep
// clipped points at or behind the reference face itself (the actual overlap region).
BoxBox._buildFaceContact = function (placedA, placedB, ax, bx, halfA, halfB, posA, posB, axisType, i, sign, out, nextContact) {
    const refIsA = axisType === 0;
    const refAxes = refIsA ? ax : bx, incAxes = refIsA ? bx : ax;
    const refHalf = refIsA ? halfA : halfB, incHalf = refIsA ? halfB : halfA;
    const refPos = refIsA ? posA : posB, incPos = refIsA ? posB : posA;

    const refNormal = BoxBox._refNormal;
    refNormal.copy(refAxes[i]);
    refNormal.scaleInPlace(sign);

    // Incident face: the other box's face most anti-parallel to refNormal (min dot over +/- each axis).
    let incFaceIndex = 0, incFaceSign = 1, best = Infinity;
    for (let k = 0; k < 3; k++) {
        const dp = incAxes[k].dot(refNormal);
        if (dp < best) { best = dp; incFaceIndex = k; incFaceSign = 1; }
        if (-dp < best) { best = -dp; incFaceIndex = k; incFaceSign = -1; }
    }
    const incNormal = BoxBox._incNormal;
    incNormal.copy(incAxes[incFaceIndex]);
    incNormal.scaleInPlace(incFaceSign);

    // The other two axes of each box, used to build the 4 corners of each face.
    const refU = (i + 1) % 3, refV = (i + 2) % 3;
    const incU = (incFaceIndex + 1) % 3, incV = (incFaceIndex + 2) % 3;

    const refCenter = BoxBox._refCenter;
    refCenter.set(
        refPos.x + refNormal.x * refHalf[i],
        refPos.y + refNormal.y * refHalf[i],
        refPos.z + refNormal.z * refHalf[i]
    );
    const incCenter = BoxBox._incCenter;
    incCenter.set(
        incPos.x + incNormal.x * incHalf[incFaceIndex],
        incPos.y + incNormal.y * incHalf[incFaceIndex],
        incPos.z + incNormal.z * incHalf[incFaceIndex]
    );

    // Incident face's 4 corners in world space.
    let poly = BoxBox._polyA;
    const hu = incHalf[incU], hv = incHalf[incV];
    const uAxis = incAxes[incU], vAxis = incAxes[incV];
    for (let c = 0; c < 4; c++) {
        const su = (c === 0 || c === 3) ? -1 : 1;
        const sv = (c < 2) ? -1 : 1;
        poly[c].set(
            incCenter.x + uAxis.x * hu * su + vAxis.x * hv * sv,
            incCenter.y + uAxis.y * hu * su + vAxis.y * hv * sv,
            incCenter.z + uAxis.z * hu * su + vAxis.z * hv * sv
        );
    }
    let polyCount = 4;
    let clipped = BoxBox._polyB;

    // Clip against the reference face's 4 side planes (Sutherland-Hodgman), each plane running
    // through the reference face center, normal = +/- refU or +/- refV axis.
    const sidePlanes = BoxBox._sidePlanes;
    sidePlanes[0].axis = refAxes[refU]; sidePlanes[0].sign = 1; sidePlanes[0].limit = refHalf[refU];
    sidePlanes[1].axis = refAxes[refU]; sidePlanes[1].sign = -1; sidePlanes[1].limit = refHalf[refU];
    sidePlanes[2].axis = refAxes[refV]; sidePlanes[2].sign = 1; sidePlanes[2].limit = refHalf[refV];
    sidePlanes[3].axis = refAxes[refV]; sidePlanes[3].sign = -1; sidePlanes[3].limit = refHalf[refV];

    for (let p = 0; p < 4; p++) {
        const plane = sidePlanes[p];
        const planeAxis = plane.axis, planeSign = plane.sign, limit = plane.limit;
        let outCount = 0;
        for (let c = 0; c < polyCount; c++) {
            const cur = poly[c], next = poly[(c + 1) % polyCount];
            const curDist = (Vector3.subInto(BoxBox._tmp, cur, refCenter).dot(planeAxis)) * planeSign - limit;
            const nextDist = (Vector3.subInto(BoxBox._tmp, next, refCenter).dot(planeAxis)) * planeSign - limit;
            const curInside = curDist <= 0, nextInside = nextDist <= 0;
            if (curInside) clipped[outCount++].copy(cur);
            if (curInside !== nextInside) {
                const t = curDist / (curDist - nextDist);
                clipped[outCount++].set(
                    cur.x + (next.x - cur.x) * t,
                    cur.y + (next.y - cur.y) * t,
                    cur.z + (next.z - cur.z) * t
                );
            }
        }
        polyCount = outCount;
        const swap = poly; poly = clipped; clipped = swap;
        if (polyCount === 0) return; // fully clipped away - shouldn't happen given overlap > 0, but safe
    }

    // Keep points behind the reference face (penetrating) and those just in front of it (still
    // separated - a speculative contact). Reporting all 4 corners together before touch, rather
    // than one at a time as they sink in, is what keeps a flat flush approach torque-free.
    const normalAtoB = BoxBox._normalAtoB;
    normalAtoB.copy(refNormal);
    if (!refIsA) normalAtoB.scaleInPlace(-1); // refNormal is B's outward normal when B is reference; flip to A->B

    for (let c = 0; c < polyCount; c++) {
        const pt = poly[c];
        const rel = Vector3.subInto(BoxBox._tmp, pt, refCenter);
        const depth = -rel.dot(refNormal); // positive = behind the reference face (penetrating)
        if (depth < -BoxBox.SEPARATED_AXIS_LIMIT) continue;

        const contact = nextContact();
        // Project the incident-face point onto the reference face along refNormal for the
        // reference-side witness point; the incident point itself is the incident-side witness.
        const onRef = BoxBox._tmp2;
        onRef.set(pt.x + refNormal.x * depth, pt.y + refNormal.y * depth, pt.z + refNormal.z * depth);

        if (refIsA) { contact.pointOnA.copy(onRef); contact.pointOnB.copy(pt); }
        else { contact.pointOnA.copy(pt); contact.pointOnB.copy(onRef); }

        // Pipeline convention: normal points B -> A.
        contact.normal.set(-normalAtoB.x, -normalAtoB.y, -normalAtoB.z);
        contact.signedDistance = depth;
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
    }
};

// --- scratch state -----------------------------------------------------------------------
BoxBox._axesA = [new Vector3(), new Vector3(), new Vector3()];
BoxBox._axesB = [new Vector3(), new Vector3(), new Vector3()];
BoxBox._halfA = [0, 0, 0];
BoxBox._halfB = [0, 0, 0];
BoxBox._d = new Vector3();
BoxBox._R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
BoxBox._absR = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
BoxBox._edgeAxis = new Vector3();

BoxBox._pA = new Vector3();
BoxBox._uA = new Vector3();
BoxBox._pB = new Vector3();
BoxBox._uB = new Vector3();
BoxBox._segR = new Vector3();
BoxBox._closestA = new Vector3();
BoxBox._closestB = new Vector3();
BoxBox._contactNormal = new Vector3();

BoxBox._refNormal = new Vector3();
BoxBox._incNormal = new Vector3();
BoxBox._refCenter = new Vector3();
BoxBox._incCenter = new Vector3();
BoxBox._normalAtoB = new Vector3();
BoxBox._tmp = new Vector3();
BoxBox._tmp2 = new Vector3();
BoxBox._polyA = [new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3()];
BoxBox._polyB = [new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3(), new Vector3()];
BoxBox._sidePlanes = [{ axis: null, sign: 1, limit: 0 }, { axis: null, sign: 1, limit: 0 }, { axis: null, sign: 1, limit: 0 }, { axis: null, sign: 1, limit: 0 }];

ActionPhysics.BoxBox = BoxBox;
