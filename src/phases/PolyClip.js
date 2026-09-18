var PolyClip = {};

PolyClip.FACE_ALIGN_DOT = 0.98;

PolyClip.FACE_OPPOSED_DOT = 0.90;

PolyClip.SEPARATION_LIMIT = 1.0;

PolyClip.MAX_POINTS = 4;

PolyClip.SCRATCH = 24;

PolyClip.facesOf = function (placed) {
    const shape = placed.shape;
    if (shape instanceof BoxShape) return PolyClip._boxFaces(placed);
    if (shape instanceof ConvexShape) return PolyClip._hullFaces(placed);
    // A curved shape's flat cap as a real polygon. A cylinder resting on its flat end, or a cone on
    // its base, gets a genuine clipped patch instead of GJK/EPA's single wandering witness point -
    // which is what lets the solver's support test tell a supported cap from an overhanging one.
    if (shape instanceof ConeShape) return PolyClip._capFaces(placed, shape.radius, shape.halfHeight, false);
    if (shape instanceof CylinderShape) return PolyClip._capFaces(placed, shape.radius, shape.halfHeight, true);
    return null;
};

// A cylinder's two flat caps (and a cone's single base cap) as regular polygons in the local XZ
// plane, so a curved shape resting on its flat end gets a REAL clipped patch instead of GJK/EPA's
// single wandering witness point. That patch is what lets the solver's support test tell a
// supported cylinder/cone (centre of mass over the patch) from an overhanging one it must tip.
// The curved side is not represented - a cylinder on its side finds no aligned face and keeps the
// GJK/EPA path (which is correct there: the contact is a line, not a face).
PolyClip.CAP_SIDES = 12;
PolyClip._capFaces = function (placed, radius, halfHeight, bothCaps) {
    const rot = placed.rotation, pos = placed.position;
    const axis = PolyClip._capAxis.set(0, 1, 0);
    const u = PolyClip._capU.set(1, 0, 0);
    const v = PolyClip._capV.set(0, 0, 1);
    rot.transformVectorInPlace(axis);
    rot.transformVectorInPlace(u);
    rot.transformVectorInPlace(v);

    const n = PolyClip.CAP_SIDES;
    const faces = [];
    const signs = bothCaps ? [-1, 1] : [-1];
    for (let s = 0; s < signs.length; s++) {
        const sign = signs[s];
        const cx = pos.x + axis.x * halfHeight * sign;
        const cy = pos.y + axis.y * halfHeight * sign;
        const cz = pos.z + axis.z * halfHeight * sign;
        const normal = new Vector3(axis.x * sign, axis.y * sign, axis.z * sign);
        const verts = [];
        for (let i = 0; i < n; i++) {
            const ang = 2 * Scalar.PI * i / n;
            const cs = Scalar.cos(ang) * radius, sn = Scalar.sin(ang) * radius;
            verts.push(new Vector3(
                cx + u.x * cs + v.x * sn,
                cy + u.y * cs + v.y * sn,
                cz + u.z * cs + v.z * sn
            ));
        }
        faces.push({ normal: normal, verts: verts, isCap: true });
    }
    return faces;
};

PolyClip._boxFaces = function (placed) {
    const bs = placed.shape;
    const h = [bs.halfWidth, bs.halfHeight, bs.halfDepth];
    const rot = placed.rotation, pos = placed.position;
    const axes = [new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1)];
    for (let i = 0; i < 3; i++) rot.transformVectorInPlace(axes[i]);

    const faces = [];
    for (let i = 0; i < 3; i++) {
        const u = (i + 1) % 3, v = (i + 2) % 3;
        for (let s = -1; s <= 1; s += 2) {
            const n = new Vector3(axes[i].x * s, axes[i].y * s, axes[i].z * s);
            const cx = pos.x + n.x * h[i], cy = pos.y + n.y * h[i], cz = pos.z + n.z * h[i];
            const verts = [];

            for (let c = 0; c < 4; c++) {
                const du = ((c === 0 || c === 3) ? -1 : 1) * s;
                const dv = (c < 2) ? -1 : 1;
                verts.push(new Vector3(
                    cx + axes[u].x * h[u] * du + axes[v].x * h[v] * dv,
                    cy + axes[u].y * h[u] * du + axes[v].y * h[v] * dv,
                    cz + axes[u].z * h[u] * du + axes[v].z * h[v] * dv
                ));
            }
            faces.push({ normal: n, verts: verts });
        }
    }
    return faces;
};

PolyClip._hullFaces = function (placed) {
    const poly = placed.shape.polyFaces;
    if (!poly || poly.length === 0) return null;
    const rot = placed.rotation, pos = placed.position, pts = placed.shape.points;
    const faces = [];
    for (let f = 0; f < poly.length; f++) {
        const src = poly[f];
        const n = new Vector3(src.normal.x, src.normal.y, src.normal.z);
        rot.transformVectorInPlace(n);
        const verts = [];
        for (let i = 0; i < src.indices.length; i++) {
            const lp = pts[src.indices[i]];
            const wp = new Vector3(lp.x, lp.y, lp.z);
            rot.transformVectorInPlace(wp);
            wp.x += pos.x; wp.y += pos.y; wp.z += pos.z;
            verts.push(wp);
        }
        faces.push({ normal: n, verts: verts });
    }
    return faces;
};

PolyClip._mostParallel = function (faces, dir) {
    let best = null, bestDot = -Infinity;
    for (let i = 0; i < faces.length; i++) {
        const d = faces[i].normal.dot(dir);
        if (d > bestDot) { bestDot = d; best = faces[i]; }
    }
    return { face: best, dot: bestDot };
};

PolyClip.buildFaceContact = function (placedA, placedB, normalBtoA, out, nextContact, separationLimit) {
    const facesA = PolyClip.facesOf(placedA);
    const facesB = PolyClip.facesOf(placedB);
    if (!facesA || !facesB) return 0;

    const dirA = PolyClip._dirA;
    dirA.set(-normalBtoA.x, -normalBtoA.y, -normalBtoA.z);
    const pickA = PolyClip._mostParallel(facesA, dirA);
    const pickB = PolyClip._mostParallel(facesB, normalBtoA);
    if (!pickA.face || !pickB.face) return 0;

    const refIsA = pickA.dot >= pickB.dot;
    const bestDot = refIsA ? pickA.dot : pickB.dot;
    if (bestDot < PolyClip.FACE_ALIGN_DOT) return 0;

    const refFace = refIsA ? pickA.face : pickB.face;
    const refNormal = refFace.normal;

    const incFaces = refIsA ? facesB : facesA;

    const incDir = PolyClip._incDir;
    incDir.set(-refNormal.x, -refNormal.y, -refNormal.z);
    const pickInc = PolyClip._mostParallel(incFaces, incDir);
    if (!pickInc.face || pickInc.dot < PolyClip.FACE_OPPOSED_DOT) return 0;
    const incFace = pickInc.face;

    // A cap-derived patch is a real support patch in every sense: a flat cap against a flat face is
    // a genuine face-on-face contact, so it gets the same centroid velocity solve a box face patch
    // does (see VelocitySolve._boxFacePatchVelocity) and the same centre-of-mass support test.
    const fromFacePatch = true;

    let poly = PolyClip._polyA, clipped = PolyClip._polyB;
    let count = incFace.verts.length;
    if (count < 3 || count > PolyClip.SCRATCH) return 0;
    for (let i = 0; i < count; i++) poly[i].copy(incFace.verts[i]);

    const rv = refFace.verts, rn = rv.length;
    for (let e = 0; e < rn; e++) {
        const a = rv[e], b = rv[(e + 1) % rn];
        const ex = b.x - a.x, ey = b.y - a.y, ez = b.z - a.z;
        const px = refNormal.y * ez - refNormal.z * ey;
        const py = refNormal.z * ex - refNormal.x * ez;
        const pz = refNormal.x * ey - refNormal.y * ex;
        const plen = Math.sqrt(px * px + py * py + pz * pz);
        if (plen < 1e-12) continue;
        const nx = px / plen, ny = py / plen, nz = pz / plen;

        let outCount = 0;
        for (let c = 0; c < count; c++) {
            const cur = poly[c], next = poly[(c + 1) % count];
            const dCur = (cur.x - a.x) * nx + (cur.y - a.y) * ny + (cur.z - a.z) * nz;
            const dNext = (next.x - a.x) * nx + (next.y - a.y) * ny + (next.z - a.z) * nz;
            const curIn = dCur >= 0, nextIn = dNext >= 0;
            if (curIn) {
                if (outCount >= PolyClip.SCRATCH) return 0;
                clipped[outCount++].copy(cur);
            }
            if (curIn !== nextIn) {
                const t = dCur / (dCur - dNext);
                if (outCount >= PolyClip.SCRATCH) return 0;
                clipped[outCount++].set(
                    cur.x + (next.x - cur.x) * t,
                    cur.y + (next.y - cur.y) * t,
                    cur.z + (next.z - cur.z) * t
                );
            }
        }
        count = outCount;
        const swap = poly; poly = clipped; clipped = swap;
        if (count === 0) return 0;
    }
    if (count < 2) return 0;

    const refOrigin = rv[0];
    const kept = PolyClip._kept;
    let keptCount = 0;
    for (let c = 0; c < count; c++) {
        const pt = poly[c];
        const depth = -((pt.x - refOrigin.x) * refNormal.x +
                        (pt.y - refOrigin.y) * refNormal.y +
                        (pt.z - refOrigin.z) * refNormal.z);
        if (depth < -PolyClip.SEPARATION_LIMIT) continue;
        kept[keptCount].point.copy(pt);
        kept[keptCount].depth = depth;
        keptCount++;
        if (keptCount >= PolyClip.SCRATCH) break;
    }
    if (keptCount < 2) return 0;

    if (keptCount > PolyClip.MAX_POINTS) {

        const slice = [];
        for (let i = 0; i < keptCount; i++) slice.push({ x: kept[i].point.x, y: kept[i].point.y, z: kept[i].point.z, depth: kept[i].depth });
        slice.sort(function (p, q) { return q.depth - p.depth; });
        for (let i = 0; i < PolyClip.MAX_POINTS; i++) {
            kept[i].point.set(slice[i].x, slice[i].y, slice[i].z);
            kept[i].depth = slice[i].depth;
        }
        keptCount = PolyClip.MAX_POINTS;
    }

    for (let i = 0; i < keptCount; i++) {
        const pt = kept[i].point, depth = kept[i].depth;
        const contact = nextContact();

        const onRef = PolyClip._onRef;
        onRef.set(pt.x + refNormal.x * depth, pt.y + refNormal.y * depth, pt.z + refNormal.z * depth);

        if (refIsA) { contact.pointOnA.copy(onRef); contact.pointOnB.copy(pt); }
        else { contact.pointOnA.copy(pt); contact.pointOnB.copy(onRef); }

        if (refIsA) contact.normal.set(-refNormal.x, -refNormal.y, -refNormal.z);
        else contact.normal.copy(refNormal);
        contact.signedDistance = depth;
        Vector3.addInto(contact.point, contact.pointOnA, contact.pointOnB).scaleInPlace(0.5);
        contact.fromMeshFace = false;
        contact.fromFacePatch = fromFacePatch;
        out.push(contact);
    }
    return keptCount;
};

PolyClip._dirA = new Vector3();
PolyClip._incDir = new Vector3();
PolyClip._onRef = new Vector3();
PolyClip._capAxis = new Vector3();
PolyClip._capU = new Vector3();
PolyClip._capV = new Vector3();
PolyClip._polyA = []; PolyClip._polyB = []; PolyClip._kept = [];
for (var _pcI = 0; _pcI < PolyClip.SCRATCH; _pcI++) {
    PolyClip._polyA.push(new Vector3());
    PolyClip._polyB.push(new Vector3());
    PolyClip._kept.push({ point: new Vector3(), depth: 0 });
}

ActionPhysics.PolyClip = PolyClip;
