// Cheap conservative "is this convex provably not touching this triangle" test, for the GJK/EPA
// fallback in PairTest to skip. One support-map query plus a point-in-triangle check, vs GJK
// running a full iteration loop only to report the same separation.
//
// Dominant case for a big CompoundShape ground: a prop rests on one tile, its broadphase AABB
// overlaps the neighbouring coplanar tiles, and GJK would run on each just to say "separated".
// ConvexTri already does this for curved convexes as a side effect of building its face contact;
// this covers the flat-faced ones (BoxShape, ConvexShape) it deliberately skips.
const TriPlaneCull = {};

// Same generous bound ConvexTri uses: beyond this a convex can't be in contact with the triangle.
// Comfortably exceeds any per-pair speculative margin at these speeds, so a pair rejected here is
// one PairTest.step() would have discarded against the real margin anyway.
TriPlaneCull.FACE_LIMIT = 0.5;

// placedA/placedB: exactly one is a TriangleShape (world-space verts, identity transform), the
// other any convex. Returns true only when separation is certain (safe to skip GJK/EPA).
TriPlaneCull.separated = function (placedA, placedB) {
    const aTri = placedA.shape instanceof TriangleShape;
    const tri = (aTri ? placedA : placedB).shape;
    const cvx = aTri ? placedB : placedA;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    const n = TriPlaneCull._n;
    if (!ConvexTri._normalInto(n, t0, t1, t2)) return false; // degenerate triangle - let GJK handle

    // Orient n toward the convex, then find the convex's deepest point back toward the triangle.
    const cvxPos = cvx.position;
    if ((cvxPos.x - t0.x) * n.x + (cvxPos.y - t0.y) * n.y + (cvxPos.z - t0.z) * n.z < 0) n.scaleInPlace(-1);

    const probe = TriPlaneCull._probe.set(-n.x, -n.y, -n.z);
    const dp = TriPlaneCull._dp;
    const invRot = TriPlaneCull._invRot.copy(cvx.rotation).invert();
    MinkowskiSupport.supportOfInto(dp, cvx, invRot, probe, TriPlaneCull._scratchDir);

    // Signed distance of the deepest point from the triangle's plane (negative = pokes behind).
    const along = (dp.x - t0.x) * n.x + (dp.y - t0.y) * n.y + (dp.z - t0.z) * n.z;
    if (along < -TriPlaneCull.FACE_LIMIT) return false; // pokes well behind - GJK/EPA judges depth
    if (along > TriPlaneCull.FACE_LIMIT) return true;   // whole convex clears the plane - separated

    // Deepest point is within the band of the plane. Project it and measure how far outside the
    // triangle that projection lands; the true gap is then hypot(along, outside).
    const px = dp.x - along * n.x, py = dp.y - along * n.y, pz = dp.z - along * n.z;
    ConvexTri._pointInTri(px, py, pz, t0, t1, t2, n, 0); // sets ConvexTri._lastOutside
    const outside = ConvexTri._lastOutside;
    if (outside <= 0) return false; // deepest point is over the triangle - contact plausible
    const a = along > 0 ? along : 0;
    return Math.sqrt(a * a + outside * outside) > TriPlaneCull.FACE_LIMIT;
};

TriPlaneCull._n = new Vector3();
TriPlaneCull._probe = new Vector3();
TriPlaneCull._dp = new Vector3();
TriPlaneCull._invRot = new Quaternion();
TriPlaneCull._scratchDir = new Vector3();

ActionPhysics.TriPlaneCull = TriPlaneCull;
