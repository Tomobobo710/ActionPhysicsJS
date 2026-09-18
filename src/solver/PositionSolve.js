// Position-level XPBD solve for one contact point.
var proto = Solver.prototype;

proto._solvePoint = function (point, bodyA, bodyB, h, capPenetration) {
    point.currentAnchorAInto(this._rA, bodyA);
    point.currentAnchorBInto(this._rB, bodyB);
    const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;
    // C = (anchorB - anchorA).normal, recomputed live (point.signedDistance is a stale tick-start
    // snapshot). Normal points B->A; C > 0 = penetrating.
    const C = (this._rB.x - this._rA.x) * nx + (this._rB.y - this._rA.y) * ny + (this._rB.z - this._rA.z) * nz;
    if (C <= 0) return; // speculative contact not yet touching
    if (capPenetration && C > Solver.MAX_PENETRATION_PER_SUBSTEP * 8 && point.fromMeshFace) return;

    // Captured on the substep this point engages, from pre-gravity velocity, for restitution.
    point._preSolveNormalVel = this._contactRelativeNormalVelocityPreGravity(point, bodyA, bodyB);

    Vector3.subInto(this._rA, this._rA, bodyA.position);
    Vector3.subInto(this._rB, this._rB, bodyB.position);

    const wSum = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
    if (wSum < 1e-12) return;

    const cappedC = capPenetration ? Math.min(C, Solver.MAX_PENETRATION_PER_SUBSTEP) : C;

    const oldLambda = point.normalLambda;
    let newLambda = oldLambda - cappedC / wSum;
    if (newLambda > 0) newLambda = 0; // a contact pushes apart, never pulls together
    const deltaLambda = newLambda - oldLambda;
    point.normalLambda = newLambda;

    // Only the share explainable by the body's own closing velocity becomes derived velocity; the
    // rest is a pure position edit (biasDelta), subtracted back out in the velocity-derivation
    // step. Lets a loaded resting body correct fully while a raw spawn overlap resolves gently.
    //
    // The allowance is per contact point per SUBSTEP, and it is spent, not refreshed: a deep overlap
    // is deliberately corrected a little at a time (MAX_PENETRATION_PER_SUBSTEP), so this same point
    // is solved several times in a substep, and crediting each of those passes separately turned a
    // handful of 5 mm position edits into a 19 m/s launch. What the body's own motion explains is how
    // far it closed this substep; correcting the same overlap again inside that substep is not more
    // of its own motion, and must not be credited again.
    const liveRelVel = this._contactRelativeNormalVelocity(point, bodyA, bodyB);
    const alreadyCredited = point._credited || 0;
    let allowance = Math.max(liveRelVel, 0) * h * Solver.EXPLAINABLE_MARGIN - alreadyCredited;
    if (allowance < 0) allowance = 0;
    let velocityC = cappedC;
    if (velocityC > allowance) velocityC = allowance;
    point._credited = alreadyCredited + velocityC;
    const velocityDelta = -velocityC / wSum;
    const biasDelta = deltaLambda - velocityDelta;
    const priorSkipAngular = this._skipPositionAngular;
    // A vertical contact is load-bearing, so its angular response is decided by the support test
    // inside _suppressQuietVerticalLanding: a body whose centre of mass is over the support (or that
    // a real patch holds up) is solved torque-free, while one that has passed the edge keeps its
    // torque - and must, or a prop balanced on the corner of a mesh edge hangs there frozen instead
    // of tipping off. A ConvexTri probe sample on a PERPENDICULAR side face reads the shape's own
    // girth as penetration; correcting that angularly flings the body, so a non-vertical curved
    // contact stays torque-free.
    if ((point.fromCurvedTri && Math.abs(ny) < 0.98) ||
        this._suppressQuietVerticalLanding(bodyA, bodyB, point, nx, ny, nz)) this._skipPositionAngular = true;

    this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, velocityDelta, false);
    this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, biasDelta, true);
    this._skipPositionAngular = priorSkipAngular;
};

// isFlatFaced lives in Solver.js - both this file and VelocitySolve.js use it.

proto._suppressQuietVerticalLanding = function (bodyA, bodyB, point, nx, ny, nz) {
    if (Math.abs(ny) < 0.98 || Math.abs(nx) > 0.12 || Math.abs(nz) > 0.12) return false;
    let body = bodyA.bodyType === RigidBody.DYNAMIC ? bodyA :
        (bodyB.bodyType === RigidBody.DYNAMIC ? bodyB : null);
    if (!body) return false;
    // Only a patch the body is actually sitting ON may be solved torque-free. Once the centre of mass
    // has passed the patch's horizontal extent the contact is one-sided - its normal response cannot
    // hold the body up without rotating it - so a body parked on the edge of a ledge keeps its
    // angular response and tips, instead of standing in a false equilibrium.
    if (this._checkSupport) {
        const tol = this._supportCentreTol;
        if (body.position.x < this._supMinX - tol || body.position.x > this._supMaxX + tol) return false;
        if (body.position.z < this._supMinZ - tol || body.position.z > this._supMaxZ + tol) return false;
    }
    const av = body.angular_velocity, lv = body.linear_velocity;
    if (lv.x * lv.x + lv.z * lv.z > 0.05 * 0.05) return false;
    // A shape with FLAT faces (a box, a hull) tips from face to face, and the rotation that carries
    // it there comes from these very corrections - so it may only be solved torque-free once it has
    // genuinely stopped turning. A CURVED shape has nothing to tip onto: its contact against a
    // vertical surface is a point or a line, and the rotation an off-centre push generates about it
    // is not something the shape's own motion accounts for. Requiring "already still" there is
    // circular - that spurious rock IS the rotation, so the gate it would need to open never opens,
    // and a settled capsule rocks against the mesh at a fifth of a radian per second forever while
    // its reported velocity reads zero.
    if (isFlatFaced(body.shape)) {
        if (av.x * av.x + av.y * av.y + av.z * av.z > 0.01 * 0.01) return false;
        const q = body.rotation;
        const upY = 1 - 2 * (q.x * q.x + q.z * q.z);
        // Only suppress landing torque once the box is genuinely face-up; the old 0.98
        // threshold covered an 11.5-degree cone and could freeze a tilted box mid-tip.
        if (Math.abs(upY) < 0.99985) return false;
    }
    return true;
};

proto._effectiveMass = function (bodyA, bodyB, rA, rB, dx, dy, dz) {
    let w = bodyA._mass_inverted + bodyB._mass_inverted;

    const rax = rA.y * dz - rA.z * dy, ray = rA.z * dx - rA.x * dz, raz = rA.x * dy - rA.y * dx;
    const rbx = rB.y * dz - rB.z * dy, rby = rB.z * dx - rB.x * dz, rbz = rB.x * dy - rB.y * dx;

    if (bodyA._mass_inverted > 0) {
        const IA = bodyA._worldInverseInertiaTensor;
        const ix = IA.e00 * rax + IA.e01 * ray + IA.e02 * raz;
        const iy = IA.e10 * rax + IA.e11 * ray + IA.e12 * raz;
        const iz = IA.e20 * rax + IA.e21 * ray + IA.e22 * raz;
        w += rax * ix + ray * iy + raz * iz;
    }
    if (bodyB._mass_inverted > 0) {
        const IB = bodyB._worldInverseInertiaTensor;
        const ix = IB.e00 * rbx + IB.e01 * rby + IB.e02 * rbz;
        const iy = IB.e10 * rbx + IB.e11 * rby + IB.e12 * rbz;
        const iz = IB.e20 * rbx + IB.e21 * rby + IB.e22 * rbz;
        w += rbx * ix + rby * iy + rbz * iz;
    }
    return w;
};

// Applies dLambda*n (plus angular correction) to both bodies: A along -n, B along +n. When `bias`,
// the movement is also recorded into this._biasDelta so the velocity-derivation step subtracts it.
proto._applyPositionalCorrection = function (bodyA, bodyB, rA, rB, nx, ny, nz, dLambda, bias) {
    const px = nx * dLambda, py = ny * dLambda, pz = nz * dLambda;

    if (bodyA._mass_inverted > 0) {
        const dx = -px * bodyA._mass_inverted * bodyA.linear_factor.x;
        const dy = -py * bodyA._mass_inverted * bodyA.linear_factor.y;
        const dz = -pz * bodyA._mass_inverted * bodyA.linear_factor.z;
        bodyA.position.x += dx; bodyA.position.y += dy; bodyA.position.z += dz;
        if (bias) {
            const b = this._biasDelta.get(bodyA.id);
            if (b) { b.x += dx; b.y += dy; b.z += dz; }
        }
        this._applyAngularCorrection(bodyA, rA, -px, -py, -pz, bias);
    }
    if (bodyB._mass_inverted > 0) {
        const dx = px * bodyB._mass_inverted * bodyB.linear_factor.x;
        const dy = py * bodyB._mass_inverted * bodyB.linear_factor.y;
        const dz = pz * bodyB._mass_inverted * bodyB.linear_factor.z;
        bodyB.position.x += dx; bodyB.position.y += dy; bodyB.position.z += dz;
        if (bias) {
            const b = this._biasDelta.get(bodyB.id);
            if (b) { b.x += dx; b.y += dy; b.z += dz; }
        }
        this._applyAngularCorrection(bodyB, rB, px, py, pz, bias);
    }
};

// Small-angle PBD angular update from a linear positional impulse p at offset r: I^-1*(r x p)*0.5.
// When `bias`, the rotation is also recorded as a bias rotation: like the linear bias displacement, a
// penetration-pop's rotation is a pure position edit, not something the body's own motion did, and
// _deriveVelocities subtracts it back out. Without this, resolving a deep overlap spun the body up in
// one substep (a 0.19 rad correction becomes ~45 rad/s at h=1/240) - the angular half of the same
// energy-injection bug the linear bias already guards against.
proto._applyAngularCorrection = function (body, r, px, py, pz, bias) {
    if (this._skipPositionAngular) return;
    const torqueX = r.y * pz - r.z * py, torqueY = r.z * px - r.x * pz, torqueZ = r.x * py - r.y * px;
    const I = body._worldInverseInertiaTensor;
    const wx = I.e00 * torqueX + I.e01 * torqueY + I.e02 * torqueZ;
    const wy = I.e10 * torqueX + I.e11 * torqueY + I.e12 * torqueZ;
    const wz = I.e20 * torqueX + I.e21 * torqueY + I.e22 * torqueZ;
    const ax = wx * body.angular_factor.x, ay = wy * body.angular_factor.y, az = wz * body.angular_factor.z;
    if (bias) {
        const b = this._biasAng.get(body.id);
        if (b) { b.x += ax; b.y += ay; b.z += az; }
    }
    this._angularCorrA.set(ax, ay, az);
    Solver._integrateRotation(body.rotation, this._angularCorrA, 1); // h=1: this IS the delta, not a rate
};
