const TriPlaneCull = {};

TriPlaneCull.FACE_LIMIT = 0.5;

TriPlaneCull.separated = function (placedA, placedB) {
    const aTri = placedA.shape instanceof TriangleShape;
    const tri = (aTri ? placedA : placedB).shape;
    const cvx = aTri ? placedB : placedA;
    const t0 = tri.a, t1 = tri.b, t2 = tri.c;

    const n = TriPlaneCull._n;
    if (!ConvexTri._normalInto(n, t0, t1, t2)) return false;

    const cvxPos = cvx.position;
    if ((cvxPos.x - t0.x) * n.x + (cvxPos.y - t0.y) * n.y + (cvxPos.z - t0.z) * n.z < 0) n.scaleInPlace(-1);

    const probe = TriPlaneCull._probe.set(-n.x, -n.y, -n.z);
    const dp = TriPlaneCull._dp;
    const invRot = TriPlaneCull._invRot.copy(cvx.rotation).invert();
    MinkowskiSupport.supportOfInto(dp, cvx, invRot, probe, TriPlaneCull._scratchDir);

    const along = (dp.x - t0.x) * n.x + (dp.y - t0.y) * n.y + (dp.z - t0.z) * n.z;
    if (along < -TriPlaneCull.FACE_LIMIT) return false;
    if (along > TriPlaneCull.FACE_LIMIT) return true;

    const px = dp.x - along * n.x, py = dp.y - along * n.y, pz = dp.z - along * n.z;
    ConvexTri._pointInTri(px, py, pz, t0, t1, t2, n, 0);
    const outside = ConvexTri._lastOutside;
    if (outside <= 0) return false;
    const a = along > 0 ? along : 0;
    return Math.sqrt(a * a + outside * outside) > TriPlaneCull.FACE_LIMIT;
};

TriPlaneCull._n = new Vector3();
TriPlaneCull._probe = new Vector3();
TriPlaneCull._dp = new Vector3();
TriPlaneCull._invRot = new Quaternion();
TriPlaneCull._scratchDir = new Vector3();

ActionPhysics.TriPlaneCull = TriPlaneCull;
