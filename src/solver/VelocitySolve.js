// Phase 3: velocity-pass contact solve (restitution + Coulomb friction + rolling resistance),
// applied after positions are solved, plus the velocity-space helpers they share.
var proto = Solver.prototype;

// A box-on-box flat face contact reports up to 4 coplanar points sharing one normal. Solving
// restitution and friction point-by-point (Gauss-Seidel) over that patch is what fabricates lateral
// drift on a perfectly symmetric drop (box-box/single): each point's off-center normal impulse spins
// the body a hair, the next point reads the spun state, and the four impulses no longer cancel - the
// residual spin then couples through friction into a net sideways velocity. For that ONE case - both
// shapes actual BoxShapes, every engaged point sharing a single normal (a genuine face patch) - the
// physically correct model is a single contact patch, not four independent points: restitution and
// friction are resolved once at the patch centroid, which is exactly symmetric and leaves no residual
// spin or drift. Everything else (edge/corner box contacts, meshes, spheres, compounds, non-coplanar
// manifolds) keeps the per-point solve, so the exact per-point restitution/friction contracts other
// tests pin down are untouched.
proto.COPLANAR_NORMAL_DOT = 0.9999;

proto._boxFacePatchVelocity = function (manifold, bodyA, bodyB, gravity, h) {
    const pts = manifold.points, n = pts.length;
    if (n < 2) return false;
    if (!(bodyA.shape instanceof BoxShape) || !(bodyB.shape instanceof BoxShape)) return false;

    // Aggregate engaged points; require a single shared normal (a real face patch, not an edge/corner).
    let cAx = 0, cAy = 0, cAz = 0, cBx = 0, cBy = 0, cBz = 0;
    let nx = 0, ny = 0, nz = 0, cnt = 0, maxPre = 0, totLam = 0;
    for (let i = 0; i < n; i++) {
        const p = pts[i];
        if (p.normalLambda >= 0) continue;
        p.currentAnchorAInto(this._rA, bodyA);
        p.currentAnchorBInto(this._rB, bodyB);
        cAx += this._rA.x; cAy += this._rA.y; cAz += this._rA.z;
        cBx += this._rB.x; cBy += this._rB.y; cBz += this._rB.z;
        nx += p.normal.x; ny += p.normal.y; nz += p.normal.z;
        if (p._preSolveNormalVel > maxPre) maxPre = p._preSolveNormalVel;
        totLam += Math.abs(p.normalLambda);
        cnt++;
    }
    if (cnt < 2) return false;
    const nl = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (nl < 1e-9) return false;
    nx /= nl; ny /= nl; nz /= nl;
    for (let i = 0; i < n; i++) {
        const p = pts[i];
        if (p.normalLambda >= 0) continue;
        if (p.normal.x * nx + p.normal.y * ny + p.normal.z * nz < this.COPLANAR_NORMAL_DOT) return false; // not coplanar
    }

    const inv = 1 / cnt;
    cAx *= inv; cAy *= inv; cAz *= inv; cBx *= inv; cBy *= inv; cBz *= inv;
    this._rA.set(cAx - bodyA.position.x, cAy - bodyA.position.y, cAz - bodyA.position.z);
    this._rB.set(cBx - bodyB.position.x, cBy - bodyB.position.y, cBz - bodyB.position.z);

    // --- Restitution at the centroid ---
    const restitution = Math.max(bodyA.restitution, bodyB.restitution);
    if (restitution > 0) {
        const g = bodyA.gravity || bodyB.gravity || gravity;
        const gravityMag = Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
        const restitutionThreshold = gravityMag * h * Solver.RESTITUTION_SLOP_FACTOR;
        if (maxPre > restitutionThreshold) {
            const va = this._pointVelocity(bodyA, this._rA, this._tmpDispB);
            const vax = va.x, vay = va.y, vaz = va.z;
            const vb = this._pointVelocity(bodyB, this._rB, this._tmpDispB);
            const relN = (vb.x - vax) * nx + (vb.y - vay) * ny + (vb.z - vaz) * nz;
            const targetN = -restitution * maxPre;
            if (targetN < relN) {
                const wN = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
                if (wN >= 1e-12) this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, (targetN - relN) / wN);
            }
        }
    }

    // --- Friction at the centroid (Coulomb cap = friction * total engaged normal impulse) ---
    const friction = Math.sqrt(bodyA.friction * bodyB.friction);
    if (friction > 0) {
        const maxImpulse = friction * totLam / h;
        if (maxImpulse > 0) {
            const va = this._pointVelocity(bodyA, this._rA, this._tmpDispB);
            const vax = va.x, vay = va.y, vaz = va.z;
            const vb = this._pointVelocity(bodyB, this._rB, this._tmpDispB);
            const rvx = vb.x - vax, rvy = vb.y - vay, rvz = vb.z - vaz;
            const vn = rvx * nx + rvy * ny + rvz * nz;
            const vtx = rvx - vn * nx, vty = rvy - vn * ny, vtz = rvz - vn * nz;
            const vtMag = Math.sqrt(vtx * vtx + vty * vty + vtz * vtz);
            if (vtMag >= 1e-12) {
                const tx = vtx / vtMag, ty = vty / vtMag, tz = vtz / vtMag;
                const wT = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, tx, ty, tz);
                if (wT >= 1e-12) {
                    let jt = vtMag / wT;
                    if (jt > maxImpulse) jt = maxImpulse;
                    this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, -tx, -ty, -tz, jt);
                }
            }
        }
    }
    return true;
};

proto._solveContactVelocity = function (point, bodyA, bodyB, gravity, h) {
    if (point.normalLambda >= 0) return; // never engaged this substep - nothing to correct

    point.currentAnchorAInto(this._rA, bodyA);
    point.currentAnchorBInto(this._rB, bodyB);
    Vector3.subInto(this._rA, this._rA, bodyA.position);
    Vector3.subInto(this._rB, this._rB, bodyB.position);
    const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;

    // --- Restitution (normal) ---
    const restitution = Math.max(bodyA.restitution, bodyB.restitution);
    const relN = this._contactRelativeNormalVelocity(point, bodyA, bodyB);
    const g = bodyA.gravity || bodyB.gravity || gravity;
    const gravityMag = Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
    const restitutionThreshold = gravityMag * h * Solver.RESTITUTION_SLOP_FACTOR;
    if (restitution > 0 && point._preSolveNormalVel > restitutionThreshold) {
        const targetN = -restitution * point._preSolveNormalVel;
        if (targetN < relN) { // only add separation, never damp an already-separating contact
            const wN = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
            if (wN >= 1e-12) this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, (targetN - relN) / wN);
        }
    }

    // --- Friction (tangent) ---
    const friction = Math.sqrt(bodyA.friction * bodyB.friction);
    if (friction <= 0) return;
    const maxImpulse = friction * Math.abs(point.normalLambda) / h;
    if (maxImpulse <= 0) return;

    this._contactRelativeVelocity(point, bodyA, bodyB, this._tmpDispA);
    const vn = this._tmpDispA.x * nx + this._tmpDispA.y * ny + this._tmpDispA.z * nz;
    let vtx = this._tmpDispA.x - vn * nx, vty = this._tmpDispA.y - vn * ny, vtz = this._tmpDispA.z - vn * nz;
    const vtMag = Math.sqrt(vtx * vtx + vty * vty + vtz * vtz);
    if (vtMag < 1e-12) return;

    const tx = vtx / vtMag, ty = vty / vtMag, tz = vtz / vtMag;
    const wT = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, tx, ty, tz);
    if (wT < 1e-12) return;
    let jt = vtMag / wT; // impulse to fully stop tangential motion, clamped to Coulomb cap
    if (jt > maxImpulse) jt = maxImpulse;
    this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, -tx, -ty, -tz, jt);
};

// Angular friction: damps relative angular velocity ABOUT the contact's tangent plane only (spin
// about the normal is untouched). Shape-agnostic - on a round shape this looks like rolling
// resistance, but it fires the same way for a box pivoting at a contact corner. Applied once per
// manifold via the most-engaged point, not once per point - splitting it per-point let each point's
// correction change the angular velocity the next point read, oscillating instead of converging
// (traced on a shoved cylinder: stuck at a nonzero fixed-point angular velocity forever instead of
// decaying to rest).
proto._solveAngularFriction = function (point, bodyA, bodyB, h) {
    const angularFriction = Math.sqrt(Math.max(bodyA.angular_friction, 0) * Math.max(bodyB.angular_friction, 0));
    if (angularFriction <= 0) return;

    const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;
    const rw = bodyA.angular_velocity, ww = bodyB.angular_velocity;
    let relWx = ww.x - rw.x, relWy = ww.y - rw.y, relWz = ww.z - rw.z;
    const relWn = relWx * nx + relWy * ny + relWz * nz;
    relWx -= relWn * nx; relWy -= relWn * ny; relWz -= relWn * nz;
    const relWMag = Math.sqrt(relWx * relWx + relWy * relWy + relWz * relWz);
    if (relWMag < 1e-9) return;

    const ax = relWx / relWMag, ay = relWy / relWMag, az = relWz / relWMag;
    let wSum = 0;
    if (bodyA._massInverted > 0) {
        const IA = bodyA._worldInverseInertiaTensor;
        wSum += ax * (IA.e00 * ax + IA.e01 * ay + IA.e02 * az) + ay * (IA.e10 * ax + IA.e11 * ay + IA.e12 * az) + az * (IA.e20 * ax + IA.e21 * ay + IA.e22 * az);
    }
    if (bodyB._massInverted > 0) {
        const IB = bodyB._worldInverseInertiaTensor;
        wSum += ax * (IB.e00 * ax + IB.e01 * ay + IB.e02 * az) + ay * (IB.e10 * ax + IB.e11 * ay + IB.e12 * az) + az * (IB.e20 * ax + IB.e21 * ay + IB.e22 * az);
    }
    if (wSum < 1e-12) return;

    const maxAngImpulse = angularFriction * Math.abs(point.normalLambda) / h;
    if (maxAngImpulse <= 0) return;
    let j = relWMag / wSum;
    if (j > maxAngImpulse) j = maxAngImpulse;

    if (bodyA._massInverted > 0) {
        const IA = bodyA._worldInverseInertiaTensor;
        const tqx = ax * j, tqy = ay * j, tqz = az * j;
        bodyA.angular_velocity.x += (IA.e00 * tqx + IA.e01 * tqy + IA.e02 * tqz) * bodyA.angular_factor.x;
        bodyA.angular_velocity.y += (IA.e10 * tqx + IA.e11 * tqy + IA.e12 * tqz) * bodyA.angular_factor.y;
        bodyA.angular_velocity.z += (IA.e20 * tqx + IA.e21 * tqy + IA.e22 * tqz) * bodyA.angular_factor.z;
    }
    if (bodyB._massInverted > 0) {
        const IB = bodyB._worldInverseInertiaTensor;
        const tqx = -ax * j, tqy = -ay * j, tqz = -az * j;
        bodyB.angular_velocity.x += (IB.e00 * tqx + IB.e01 * tqy + IB.e02 * tqz) * bodyB.angular_factor.x;
        bodyB.angular_velocity.y += (IB.e10 * tqx + IB.e11 * tqy + IB.e12 * tqz) * bodyB.angular_factor.y;
        bodyB.angular_velocity.z += (IB.e20 * tqx + IB.e21 * tqy + IB.e22 * tqz) * bodyB.angular_factor.z;
    }
};

// Contact-relative velocity (B's point velocity minus A's) into `out`.
proto._contactRelativeVelocity = function (point, bodyA, bodyB, out) {
    point.currentAnchorAInto(this._tmpPrev, bodyA);
    this._tmpPrev.subInPlace(bodyA.position);
    const va = this._pointVelocity(bodyA, this._tmpPrev, this._tmpDispB);
    const vax = va.x, vay = va.y, vaz = va.z;
    point.currentAnchorBInto(this._tmpPrev, bodyB);
    this._tmpPrev.subInPlace(bodyB.position);
    const vb = this._pointVelocity(bodyB, this._tmpPrev, this._tmpDispB);
    out.set(vb.x - vax, vb.y - vay, vb.z - vaz);
    return out;
};

// Velocity of the material point at center-relative offset r on `body`: v + omega x r.
proto._pointVelocity = function (body, r, out) {
    const w = body.angular_velocity, v = body.linear_velocity;
    out.set(
        v.x + (w.y * r.z - w.z * r.y),
        v.y + (w.z * r.x - w.x * r.z),
        v.z + (w.x * r.y - w.y * r.x)
    );
    return out;
};

proto._contactRelativeNormalVelocity = function (point, bodyA, bodyB) {
    this._contactRelativeVelocity(point, bodyA, bodyB, this._tmpDispA);
    return this._tmpDispA.x * point.normal.x + this._tmpDispA.y * point.normal.y + this._tmpDispA.z * point.normal.z;
};

// Same as _contactRelativeNormalVelocity but using each body's linear velocity from before this
// substep's gravity add - used only for restitution's pre-solve capture (PositionSolve.js).
proto._contactRelativeNormalVelocityPreGravity = function (point, bodyA, bodyB) {
    point.currentAnchorAInto(this._tmpPrev, bodyA);
    this._tmpPrev.subInPlace(bodyA.position);
    const preA = this._preGravityVel.get(bodyA.id) || bodyA.linear_velocity;
    const wa = bodyA.angular_velocity, ra = this._tmpPrev;
    const vax = preA.x + (wa.y * ra.z - wa.z * ra.y);
    const vay = preA.y + (wa.z * ra.x - wa.x * ra.z);
    const vaz = preA.z + (wa.x * ra.y - wa.y * ra.x);

    point.currentAnchorBInto(this._tmpPrev, bodyB);
    this._tmpPrev.subInPlace(bodyB.position);
    const preB = this._preGravityVel.get(bodyB.id) || bodyB.linear_velocity;
    const wb = bodyB.angular_velocity, rb = this._tmpPrev;
    const vbx = preB.x + (wb.y * rb.z - wb.z * rb.y);
    const vby = preB.y + (wb.z * rb.x - wb.x * rb.z);
    const vbz = preB.z + (wb.x * rb.y - wb.y * rb.x);

    const dx = vbx - vax, dy = vby - vay, dz = vbz - vaz;
    return dx * point.normal.x + dy * point.normal.y + dz * point.normal.z;
};

// Applies velocity-space impulse j*(dx,dy,dz) at contact offsets rA/rB (A: -j, B: +j).
proto._applyVelocityImpulse = function (bodyA, bodyB, rA, rB, dx, dy, dz, j) {
    const px = dx * j, py = dy * j, pz = dz * j;
    if (bodyA._massInverted > 0) {
        bodyA.linear_velocity.x -= px * bodyA._massInverted * bodyA.linear_factor.x;
        bodyA.linear_velocity.y -= py * bodyA._massInverted * bodyA.linear_factor.y;
        bodyA.linear_velocity.z -= pz * bodyA._massInverted * bodyA.linear_factor.z;
        this._applyAngularVelocityImpulse(bodyA, rA, -px, -py, -pz);
    }
    if (bodyB._massInverted > 0) {
        bodyB.linear_velocity.x += px * bodyB._massInverted * bodyB.linear_factor.x;
        bodyB.linear_velocity.y += py * bodyB._massInverted * bodyB.linear_factor.y;
        bodyB.linear_velocity.z += pz * bodyB._massInverted * bodyB.linear_factor.z;
        this._applyAngularVelocityImpulse(bodyB, rB, px, py, pz);
    }
};

proto._applyAngularVelocityImpulse = function (body, r, px, py, pz) {
    const tqx = r.y * pz - r.z * py, tqy = r.z * px - r.x * pz, tqz = r.x * py - r.y * px;
    const I = body._worldInverseInertiaTensor;
    body.angular_velocity.x += (I.e00 * tqx + I.e01 * tqy + I.e02 * tqz) * body.angular_factor.x;
    body.angular_velocity.y += (I.e10 * tqx + I.e11 * tqy + I.e12 * tqz) * body.angular_factor.y;
    body.angular_velocity.z += (I.e20 * tqx + I.e21 * tqy + I.e22 * tqz) * body.angular_factor.z;
};

// Two unit vectors spanning the plane perpendicular to `normal`.
Solver._tangentBasis = function (normal, outT1, outT2) {
    outT1.findOrthogonal(normal);
    Vector3.crossInto(outT2, normal, outT1);
};
