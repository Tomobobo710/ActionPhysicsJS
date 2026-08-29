// Phase 2: position-level XPBD constraint solve for a single contact point, plus the effective-mass
// and positional-correction helpers it uses.
var proto = Solver.prototype;

proto._solvePoint = function (point, bodyA, bodyB, h, deferRotation, capPenetration) {
    point.currentAnchorAInto(this._rA, bodyA);
    point.currentAnchorBInto(this._rB, bodyB);
    const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;
    // C = (anchorB - anchorA).normal; normal points B->A (GJK/EPA convention). C > 0 = penetrating.
    // Recomputed live every call from current anchor positions, never from point.signedDistance
    // (a tick-start snapshot, stale by the second substep).
    const C = (this._rB.x - this._rA.x) * nx + (this._rB.y - this._rA.y) * ny + (this._rB.z - this._rA.z) * nz;
    // Speculative contact: created while still separated; only correct once predicted motion has
    // actually reached/passed touch (C > 0). Never pulls a separated pair together across a gap.
    if (C <= 0) return;

    // Pre-solve normal velocity for restitution, captured on the substep that actually engages this
    // point (C > 0), using each body's PRE-gravity velocity (this substep's own gravity add hasn't
    // happened from the impact's point of view yet).
    point._preSolveNormalVel = this._contactRelativeNormalVelocityPreGravity(point, bodyA, bodyB);

    Vector3.subInto(this._rA, this._rA, bodyA.position);
    Vector3.subInto(this._rB, this._rB, bodyB.position);

    const wSum = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
    if (wSum < 1e-12) return; // both bodies immovable along this normal

    // capPenetration is only passed true for a multi-point manifold (see Solver.MAX_PENETRATION_PER_SUBSTEP) -
    // a single-point manifold always resolves C fully in one substep.
    const cappedC = capPenetration ? Math.min(C, Solver.MAX_PENETRATION_PER_SUBSTEP) : C;

    // Rigid (compliance-free) contact: deltaLambda = -C/wSum. normalLambda accumulates <= 0
    // (a contact can push apart, never pull together).
    const oldLambda = point.normalLambda;
    let newLambda = oldLambda - cappedC / wSum;
    if (newLambda > 0) newLambda = 0;
    const deltaLambda = newLambda - oldLambda;
    point.normalLambda = newLambda;

    // Split the correction: only the part explainable by the body's own real closing velocity
    // becomes derived velocity (velocityDelta); the rest is a pure position edit (biasDelta),
    // excluded from step 4's velocity derivation. This is what lets a resting body under load
    // correct fully while a raw spawn overlap resolves as gradual position bias instead of
    // fabricated kinetic energy.
    const liveRelVel = this._contactRelativeNormalVelocity(point, bodyA, bodyB);
    const explainableBySubstep = Math.max(liveRelVel, 0) * h * Solver.EXPLAINABLE_MARGIN;
    let velocityC = cappedC;
    if (velocityC > explainableBySubstep) velocityC = explainableBySubstep;
    const velocityDelta = -velocityC / wSum;
    const biasDelta = deltaLambda - velocityDelta;
    this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, velocityDelta, false, deferRotation);
    this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, biasDelta, true, deferRotation);
};

// Generalized inverse mass along direction (dx,dy,dz): linear + angular contribution from both bodies.
proto._effectiveMass = function (bodyA, bodyB, rA, rB, dx, dy, dz) {
    let w = bodyA._massInverted + bodyB._massInverted;

    const rax = rA.y * dz - rA.z * dy, ray = rA.z * dx - rA.x * dz, raz = rA.x * dy - rA.y * dx;
    const rbx = rB.y * dz - rB.z * dy, rby = rB.z * dx - rB.x * dz, rbz = rB.x * dy - rB.y * dx;

    if (bodyA._massInverted > 0) {
        const IA = bodyA._worldInverseInertiaTensor;
        const ix = IA.e00 * rax + IA.e01 * ray + IA.e02 * raz;
        const iy = IA.e10 * rax + IA.e11 * ray + IA.e12 * raz;
        const iz = IA.e20 * rax + IA.e21 * ray + IA.e22 * raz;
        w += rax * ix + ray * iy + raz * iz;
    }
    if (bodyB._massInverted > 0) {
        const IB = bodyB._worldInverseInertiaTensor;
        const ix = IB.e00 * rbx + IB.e01 * rby + IB.e02 * rbz;
        const iy = IB.e10 * rbx + IB.e11 * rby + IB.e12 * rbz;
        const iz = IB.e20 * rbx + IB.e21 * rby + IB.e22 * rbz;
        w += rbx * ix + rby * iy + rbz * iz;
    }
    return w;
};

// Applies dLambda*n (and the matching angular correction) to both bodies. C = (anchorB-anchorA).n,
// so A moves along -n*dLambda, B along +n*dLambda.
// `bias`: true for the non-explainable share of a split correction - the body still moves (so the
// next substep measures a smaller overlap), but the movement is also recorded into this._biasDelta
// for step 4 to subtract back out.
// `deferRotation` (optional): accumulates the angular delta instead of composing it into
// body.rotation immediately - see _applyAngularCorrection / _flushDeferredRotation.
proto._applyPositionalCorrection = function (bodyA, bodyB, rA, rB, nx, ny, nz, dLambda, bias, deferRotation) {
    const px = nx * dLambda, py = ny * dLambda, pz = nz * dLambda;

    if (bodyA._massInverted > 0) {
        const dx = -px * bodyA._massInverted * bodyA.linear_factor.x;
        const dy = -py * bodyA._massInverted * bodyA.linear_factor.y;
        const dz = -pz * bodyA._massInverted * bodyA.linear_factor.z;
        bodyA.position.x += dx; bodyA.position.y += dy; bodyA.position.z += dz;
        if (bias) {
            const b = this._biasDelta.get(bodyA.id);
            if (b) { b.x += dx; b.y += dy; b.z += dz; }
        }
        this._applyAngularCorrection(bodyA, rA, -px, -py, -pz, deferRotation);
    }
    if (bodyB._massInverted > 0) {
        const dx = px * bodyB._massInverted * bodyB.linear_factor.x;
        const dy = py * bodyB._massInverted * bodyB.linear_factor.y;
        const dz = pz * bodyB._massInverted * bodyB.linear_factor.z;
        bodyB.position.x += dx; bodyB.position.y += dy; bodyB.position.z += dz;
        if (bias) {
            const b = this._biasDelta.get(bodyB.id);
            if (b) { b.x += dx; b.y += dy; b.z += dz; }
        }
        this._applyAngularCorrection(bodyB, rB, px, py, pz, deferRotation);
    }
};

// Small-angle PBD angular update from a linear positional impulse p at offset r: I^-1*(r x p)*0.5.
// With `deferRotation` supplied, adds the small-angle delta into its per-body accumulator instead
// of composing it into body.rotation immediately - see _applyPositionalCorrection / _flushDeferredRotation.
proto._applyAngularCorrection = function (body, r, px, py, pz, deferRotation) {
    const torqueX = r.y * pz - r.z * py, torqueY = r.z * px - r.x * pz, torqueZ = r.x * py - r.y * px;
    const I = body._worldInverseInertiaTensor;
    const wx = I.e00 * torqueX + I.e01 * torqueY + I.e02 * torqueZ;
    const wy = I.e10 * torqueX + I.e11 * torqueY + I.e12 * torqueZ;
    const wz = I.e20 * torqueX + I.e21 * torqueY + I.e22 * torqueZ;
    const ax = wx * body.angular_factor.x, ay = wy * body.angular_factor.y, az = wz * body.angular_factor.z;
    if (deferRotation) {
        const acc = deferRotation.get(body.id);
        if (acc) { acc.x += ax; acc.y += ay; acc.z += az; return; }
    }
    this._angularCorrA.set(ax, ay, az);
    Solver._integrateRotation(body.rotation, this._angularCorrA, 1); // h=1: this IS the delta, not a rate
};

// Applies one body's accumulated deferred small-angle rotation (summed across every point in the
// manifold's pass) as a single quaternion update, then clears the accumulator.
proto._flushDeferredRotation = function (body, deferRotation) {
    const acc = deferRotation.get(body.id);
    if (!acc || (acc.x === 0 && acc.y === 0 && acc.z === 0)) return;
    this._angularCorrA.set(acc.x, acc.y, acc.z);
    Solver._integrateRotation(body.rotation, this._angularCorrA, 1);
    acc.x = 0; acc.y = 0; acc.z = 0;
};
