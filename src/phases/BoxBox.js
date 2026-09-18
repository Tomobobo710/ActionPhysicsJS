const BoxBox = {};

BoxBox.applies = function (placedA, placedB) {
    return placedA.shape instanceof BoxShape && placedB.shape instanceof BoxShape;
};

BoxBox.RELATIVE_TOLERANCE = 0.25;

BoxBox.ABSOLUTE_TOLERANCE = 1e-6;

BoxBox.PARALLEL_EPSILON = 1e-9;

BoxBox.SEPARATED_AXIS_LIMIT = 1.0;

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

    const R = BoxBox._R, absR = BoxBox._absR;
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            R[i][j] = ax[i].dot(bx[j]);
            absR[i][j] = Math.abs(R[i][j]) + 1e-9;
        }
    }

    const dA = [d.dot(ax[0]), d.dot(ax[1]), d.dot(ax[2])];
    const dB = [d.dot(bx[0]), d.dot(bx[1]), d.dot(bx[2])];

    let minOverlap = Infinity;
    let bestAxisType = -1;
    let bestI = -1, bestSign = 1;

    for (let i = 0; i < 3; i++) {
        const ra = halfA[i];
        const rb = halfB[0] * absR[i][0] + halfB[1] * absR[i][1] + halfB[2] * absR[i][2];
        const overlap = ra + rb - Math.abs(dA[i]);
        if (overlap < minOverlap) { minOverlap = overlap; bestAxisType = 0; bestI = i; bestSign = dA[i] >= 0 ? 1 : -1; }
    }

    for (let j = 0; j < 3; j++) {
        const rb = halfB[j];
        const ra = halfA[0] * absR[0][j] + halfA[1] * absR[1][j] + halfA[2] * absR[2][j];
        const overlap = ra + rb - Math.abs(dB[j]);
        if (overlap < minOverlap) { minOverlap = overlap; bestAxisType = 1; bestI = j; bestSign = dB[j] >= 0 ? -1 : 1; }
    }

    const faceOverlap = minOverlap;

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

    const tieBreakMargin = Math.max(BoxBox.RELATIVE_TOLERANCE * Math.abs(faceOverlap), BoxBox.ABSOLUTE_TOLERANCE);
    if (faceOverlap >= 0 && bestEdgeI >= 0 && bestEdgeOverlap < faceOverlap - tieBreakMargin) {
        BoxBox._buildEdgeContact(placedA, placedB, ax, bx, halfA, halfB, posA, posB,
            bestEdgeI, bestEdgeJ, bestEdgeOverlap, out, nextContact);
        return out;
    }

    if (faceOverlap < -BoxBox.SEPARATED_AXIS_LIMIT) return null;

    BoxBox._buildFaceContact(placedA, placedB, ax, bx, halfA, halfB, posA, posB,
        bestAxisType, bestI, bestSign, out, nextContact);
    return out;
};

BoxBox._buildEdgeContact = function (placedA, placedB, ax, bx, halfA, halfB, posA, posB, i, j, overlap, out, nextContact) {

    const d = BoxBox._d;
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

    const normal = BoxBox._contactNormal;
    Vector3.crossInto(normal, uA, uB);
    const lenSq = normal.x * normal.x + normal.y * normal.y + normal.z * normal.z;
    if (lenSq > BoxBox.PARALLEL_EPSILON) normal.scaleInPlace(1 / Math.sqrt(lenSq));
    else normal.set(0, 1, 0);
    if (normal.dot(d) < 0) normal.scaleInPlace(-1);
    normal.scaleInPlace(-1);

    const contact = nextContact();
    contact.pointOnA.copy(closestA);
    contact.pointOnB.copy(closestB);
    contact.normal.copy(normal);
    contact.signedDistance = overlap;
    contact.fromBoxBox = true;
    Vector3.addInto(contact.point, closestA, closestB).scaleInPlace(0.5);
    out.push(contact);
};

BoxBox._buildFaceContact = function (placedA, placedB, ax, bx, halfA, halfB, posA, posB, axisType, i, sign, out, nextContact) {
    const refIsA = axisType === 0;
    const refAxes = refIsA ? ax : bx, incAxes = refIsA ? bx : ax;
    const refHalf = refIsA ? halfA : halfB, incHalf = refIsA ? halfB : halfA;
    const refPos = refIsA ? posA : posB, incPos = refIsA ? posB : posA;

    const refNormal = BoxBox._refNormal;
    refNormal.copy(refAxes[i]);
    refNormal.scaleInPlace(sign);

    let incFaceIndex = 0, incFaceSign = 1, best = Infinity;
    for (let k = 0; k < 3; k++) {
        const dp = incAxes[k].dot(refNormal);
        if (dp < best) { best = dp; incFaceIndex = k; incFaceSign = 1; }
        if (-dp < best) { best = -dp; incFaceIndex = k; incFaceSign = -1; }
    }
    const incNormal = BoxBox._incNormal;
    incNormal.copy(incAxes[incFaceIndex]);
    incNormal.scaleInPlace(incFaceSign);

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
        if (polyCount === 0) return;
    }

    const normalAtoB = BoxBox._normalAtoB;
    normalAtoB.copy(refNormal);
    if (!refIsA) normalAtoB.scaleInPlace(-1);

    for (let c = 0; c < polyCount; c++) {
        const pt = poly[c];
        const rel = Vector3.subInto(BoxBox._tmp, pt, refCenter);
        const depth = -rel.dot(refNormal);
        if (depth < -BoxBox.SEPARATED_AXIS_LIMIT) continue;

        const contact = nextContact();

        const onRef = BoxBox._tmp2;
        onRef.set(pt.x + refNormal.x * depth, pt.y + refNormal.y * depth, pt.z + refNormal.z * depth);

        if (refIsA) { contact.pointOnA.copy(onRef); contact.pointOnB.copy(pt); }
        else { contact.pointOnA.copy(pt); contact.pointOnB.copy(onRef); }

        contact.normal.set(-normalAtoB.x, -normalAtoB.y, -normalAtoB.z);
        contact.signedDistance = depth;
        contact.fromBoxBox = true;
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        out.push(contact);
    }
};

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
