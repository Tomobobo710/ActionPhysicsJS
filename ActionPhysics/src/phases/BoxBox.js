// Closed-form box-box: 15-axis SAT (3+3 face normals, 9 edge-cross axes) picks the minimum-
// penetration separating axis, then either clips the incident face against the reference face's
// side planes (face contact, up to 4 points) or takes the closest points between the two
// contributing edges (edge-edge contact, 1 point). Own epsilon, no shared GJK/EPA state.
const BoxBox = {};

BoxBox.applies = function (placedA, placedB) {
    return placedA.shape instanceof BoxShape && placedB.shape instanceof BoxShape;
};

// SAT tie-break: an edge-edge axis only wins over the best face axis if it beats it by more than
// this fraction of the face overlap - guards the classic SAT jitter case of two boxes resting
// face-to-face, where an edge axis can numerically tie a face axis and flip the contact type
// (4-point face manifold vs 1-point edge manifold) tick to tick.
BoxBox.RELATIVE_TOLERANCE = 0.005;
// Absolute floor under the tie-break margin above - at near-zero overlap (exact touch, sd ~ 0)
// RELATIVE_TOLERANCE * faceOverlap itself vanishes to ~0, so plain float noise between the face
// and edge overlap sums (they differ only by each absR epsilon's rounding) would otherwise win the
// edge branch essentially at random - producing a spurious 1-point edge contact instead of the
// correct 4-point face manifold for a flat box resting on a much larger box (e.g. the ground).
BoxBox.ABSOLUTE_TOLERANCE = 1e-6;
// Edge-cross axes below this squared length are near-parallel edges (degenerate axis, direction
// undefined) - skipped rather than normalizing a near-zero vector.
BoxBox.PARALLEL_EPSILON = 1e-9;
// How far apart (along the chosen SAT axis) a still-separated pair is trusted to report a
// speculative contact via face clipping / edge closest-points, rather than falling through to
// GJK/EPA. Deliberately generous (not tied to any one pair's speculative margin, which depends on
// relative velocity and isn't known here) - a face/edge SAT axis stays geometrically meaningful
// well past any margin PairTest.step would actually keep, so the real filtering happens there; this
// just bounds it so a wildly separated pair doesn't run the clip machinery for nothing.
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

    // Track the axis of LEAST overlap across all 15 candidates, whether or not any individual
    // axis is actually separating (overlap < 0) - a negative-overlap axis here just means the
    // boxes are apart along it, and its magnitude is the gap, which the caller (PairTest.step)
    // compares against the speculative margin exactly like SphereSphere/SphereBox's separated
    // case. Bailing out early on the first negative axis (the classic SAT boolean-overlap test)
    // would silently drop every speculative box-box contact - no more overlap tests, no early return.
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
    // Face axes of B (3). bestSign here is the OPPOSITE test to A's: d = posB - posA, so d.dot(bx[j])
    // >= 0 means A sits on B's -bx[j] side - B's reference face (the one facing A) is the -bx[j]
    // face, sign -1. (For A's own axes above, d points the other way, so the un-flipped sign is
    // already correct there.)
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

    // A negative trueMin just means the boxes are apart along that axis, by that many units - the
    // SAT distance bound is still valid, and face clipping still produces the right witness
    // points/gap for a genuinely nearby separated pair (a flat box approaching the ground a hair
    // above it is the common case: all 4 corners are still equally close, and reporting only ONE of
    // them, as a single-point GJK/EPA fallback would, hands the solver an off-center speculative
    // contact - torque from nothing the instant that point is later confirmed as touching). Only
    // bail to the generic GJK/EPA path when SAT's own numbers stop being geometrically meaningful -
    // see the footprint-disjoint guard below.
    let trueMin = Math.min(faceOverlap, bestEdgeI >= 0 ? bestEdgeOverlap : Infinity);

    // Prefer the face axis unless an edge axis is a clearly tighter fit - biases toward face
    // contacts (4-point manifolds) on ties, which is what keeps resting boxes from flickering
    // into a 1-point edge manifold every few ticks. Uses the ABSOLUTE floor alone (not scaled by a
    // possibly-negative faceOverlap) once separated - a relative fraction of a negative number is
    // itself negative and would wrongly widen, not narrow, the edge axis's window to win the tie.
    const tieBreakMargin = faceOverlap >= 0
        ? Math.max(BoxBox.RELATIVE_TOLERANCE * faceOverlap, BoxBox.ABSOLUTE_TOLERANCE)
        : BoxBox.ABSOLUTE_TOLERANCE;
    if (bestEdgeI >= 0 && bestEdgeOverlap < faceOverlap - tieBreakMargin) {
        // Edge-edge's own closest-point construction assumes the two edges are genuinely the
        // closest FEATURE, which stops holding once the boxes are far enough apart that a face (not
        // an edge) is actually nearest - conservatively only trust it within the same margin as a
        // real speculative contact would be kept by the caller anyway.
        if (bestEdgeOverlap < -BoxBox.SEPARATED_AXIS_LIMIT) return null;
        BoxBox._buildEdgeContact(placedA, placedB, ax, bx, halfA, halfB, posA, posB,
            bestEdgeI, bestEdgeJ, bestEdgeOverlap, out, nextContact);
        return out;
    }

    // Face clipping's geometry (project the incident face onto the reference face's extent) only
    // matches "closest point" once the boxes are far enough apart along OTHER axes that they no
    // longer face each other at all - bail to GJK/EPA past that point rather than returning a
    // clipped-away-to-nothing or geometrically-meaningless result.
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

    // Incident face: the face of the other box whose own outward normal is most anti-parallel to
    // refNormal (the face "facing into" the reference box) - i.e. minimizing (candidate normal) .
    // refNormal over the 6 candidate face normals (+/- each local axis).
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

    // Keep points at or behind the reference face (actual penetration, depth > 0) AND points just
    // in front of it (depth < 0: still separated, a genuine speculative contact - see
    // SEPARATED_AXIS_LIMIT above for why trusting this here, not just at depth ~ 0, is the fix for
    // a flat box approaching flush: without it, only whichever single corner GJK/EPA's separated
    // case happens to pick becomes the pre-touch contact, and the OTHER 3 corners get seen as
    // touching for the first time only on the substep they've already sunk in - reported one
    // corner at a time instead of all 4 together, which reads as a torque impulse from nothing).
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
