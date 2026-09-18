// Phase 3: velocity-pass contact solve (restitution + Coulomb friction + rolling resistance),
// applied after positions are solved, plus the velocity-space helpers they share.
var proto = Solver.prototype;

// Solving restitution + friction point-by-point over a flat face patch fabricates lateral drift on
// a symmetric drop: each point's off-center impulse spins the body a hair, the next reads the spun
// state, and the impulses no longer cancel. For a genuine face patch (a BoxBox face manifold, or a
// mesh face manifold whose points all come from TriTri) resolve it once at the centroid instead.
// Everything else keeps the per-point solve.
proto.COPLANAR_NORMAL_DOT = 0.9999;

proto._boxFacePatchVelocity = function (manifold, bodyA, bodyB, gravity, h) {
    const facePatch = manifold.points.length > 1 && manifold.points[0].fromFacePatch;

    const pts = manifold.points, n = pts.length;
    if (n < 2) return false;
    const bothBoxes = (bodyA.shape instanceof BoxShape) && (bodyB.shape instanceof BoxShape);
    if (facePatch) {
        for (let i = 0; i < n; i++) if (!pts[i].fromFacePatch) return false;
    }
    let allMeshFace = !bothBoxes;
    if (allMeshFace) for (let i = 0; i < n; i++) if (!pts[i].fromMeshFace && !pts[i].fromFacePatch) { allMeshFace = false; break; }
    if (!bothBoxes && !allMeshFace) return false;

    // A mesh face patch is one face by construction, so use all its points for the centroid - not
    // just the ones the position sweep left engaged this substep. A BoxBox patch uses engaged-only.
    const useAll = allMeshFace;
    // How the anchor is chosen (below) depends on whether the patch is a polygon of a flat-faced
    // body. A curved body's points sample the shape's own surface instead, and a shape with mixed
    // children (a compound with a cylinder in it) counts as curved.
    const dyn = bodyA.bodyType === RigidBody.DYNAMIC ? bodyA : bodyB;
    let nx = 0, ny = 0, nz = 0, cnt = 0, engaged = 0, maxPre = 0, totLam = 0;
    let curved = !isFlatFaced(dyn.shape);
    for (let i = 0; i < n; i++) {
        const p = pts[i];
        const isEngaged = p.normalLambda < 0;
        if (isEngaged) {
            engaged++;
            if (p._preSolveNormalVel > maxPre) maxPre = p._preSolveNormalVel;
            totLam += Math.abs(p.normalLambda);
        }
        if (p.fromCurvedTri) curved = true; // a probe cloud, not a polygon patch - see below
        if (!useAll && !isEngaged) continue;
        nx += p.normal.x; ny += p.normal.y; nz += p.normal.z;
        cnt++;
    }
    if (engaged < 1 || cnt < 2) return false;
    const nl = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (nl < 1e-9) return false;
    nx /= nl; ny /= nl; nz /= nl;

    // The anchor is the centre of the contact REGION, and these points are only a SAMPLE of it - four
    // at most, chosen by a spread-maximizing reduction, plus whatever vertices a mesh triangle's clip
    // adds. Their arithmetic mean is therefore not the region's centre: on a slab resting flat across
    // a tile seam this manifold holds three of the face's four corners plus the point where the seam
    // crosses one edge, and that mean lands half a metre off the body - so restitution was applied a
    // half-metre to one side and spun a slab that had been dropped dead flat (measured 0 -> 3.9 rad/s
    // in a single substep, against exactly 0 on the same square drawn as one box face). The centre of
    // the sample's EXTENT along the patch plane does not depend on WHICH interior points the reduction
    // kept, so it stays on the region; for any patch whose points are a fair sample the two agree.
    //
    // A CURVED body is the exception, and keeps the mean: its points sample the SHAPE's surface
    // across the contact band rather than outlining a polygon, so the extreme samples sit out on the
    // shape's shoulders and the midpoint of those extremes is not the centre of the band. Measured:
    // taking a lying cylinder's seam contact from the mean to the extent midpoint stops it rolling
    // (the coin-pusher roller manages 0.63 turns where it needs 3) and drops it through the floor.
    let minAx = Infinity, maxAx = -Infinity, minAy = Infinity, maxAy = -Infinity, minAz = Infinity, maxAz = -Infinity;
    let minBx = Infinity, maxBx = -Infinity, minBy = Infinity, maxBy = -Infinity, minBz = Infinity, maxBz = -Infinity;
    let sumAx = 0, sumAy = 0, sumAz = 0, sumBx = 0, sumBy = 0, sumBz = 0;
    for (let i = 0; i < n; i++) {
        const p = pts[i];
        if (!useAll && p.normalLambda >= 0) continue;
        if (p.normal.x * nx + p.normal.y * ny + p.normal.z * nz < this.COPLANAR_NORMAL_DOT) return false; // not coplanar
        p.currentAnchorAInto(this._rA, bodyA);
        sumAx += this._rA.x; sumAy += this._rA.y; sumAz += this._rA.z;
        if (this._rA.x < minAx) minAx = this._rA.x;
        if (this._rA.x > maxAx) maxAx = this._rA.x;
        if (this._rA.y < minAy) minAy = this._rA.y;
        if (this._rA.y > maxAy) maxAy = this._rA.y;
        if (this._rA.z < minAz) minAz = this._rA.z;
        if (this._rA.z > maxAz) maxAz = this._rA.z;
        p.currentAnchorBInto(this._rB, bodyB);
        sumBx += this._rB.x; sumBy += this._rB.y; sumBz += this._rB.z;
        if (this._rB.x < minBx) minBx = this._rB.x;
        if (this._rB.x > maxBx) maxBx = this._rB.x;
        if (this._rB.y < minBy) minBy = this._rB.y;
        if (this._rB.y > maxBy) maxBy = this._rB.y;
        if (this._rB.z < minBz) minBz = this._rB.z;
        if (this._rB.z > maxBz) maxBz = this._rB.z;
    }

    const inv = 1 / cnt;
    let ax, ay, az, bx, by, bz;
    if (curved) {
        ax = sumAx * inv; ay = sumAy * inv; az = sumAz * inv;
        bx = sumBx * inv; by = sumBy * inv; bz = sumBz * inv;
    } else {
        // Midpoint of that extent, shifted back onto the patch plane (the plane through the sampled
        // points, p.n = mean) along the normal, so the anchor carries no lever arm in the normal
        // direction either - friction is tangential and would see one.
        ax = (minAx + maxAx) * 0.5; ay = (minAy + maxAy) * 0.5; az = (minAz + maxAz) * 0.5;
        bx = (minBx + maxBx) * 0.5; by = (minBy + maxBy) * 0.5; bz = (minBz + maxBz) * 0.5;
        const shiftA = (ax * nx + ay * ny + az * nz) - (sumAx * nx + sumAy * ny + sumAz * nz) * inv;
        const shiftB = (bx * nx + by * ny + bz * nz) - (sumBx * nx + sumBy * ny + sumBz * nz) * inv;
        ax -= shiftA * nx; ay -= shiftA * ny; az -= shiftA * nz;
        bx -= shiftB * nx; by -= shiftB * ny; bz -= shiftB * nz;
    }
    this._rA.set(ax - bodyA.position.x, ay - bodyA.position.y, az - bodyA.position.z);
    this._rB.set(bx - bodyB.position.x, by - bodyB.position.y, bz - bodyB.position.z);

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
                    let jt = vtMag / wT; // impulse to fully stop tangential motion, clamped to Coulomb cap
                    if (jt > maxImpulse) jt = maxImpulse;
                    this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, -tx, -ty, -tz, jt);
                }
            }
        }
    }
    return true;
};

// One velocity solve for a whole coplanar contact set (see Solver._coplanarPatchGroups): restitution
// and friction applied ONCE at the set's shared centroid, so a body resting on four butting mesh
// tiles sees exactly the impulse it would see on one mesh of the same shape. The dynamic body is
// treated as A and every grouped surface as the static B, so the only impulse applied is the one to
// the body. Returns false when the set is not solvable as one (points on two planes, nothing
// engaged), in which case the caller falls back to the per-point solves.
proto._solvePatchGroup = function (group, gravity, h) {
    const dyn = group.dyn, other = group.other;
    const anchor = this._tmpDispA;
    let engaged = 0, maxPre = 0, totLam = 0, cnt = 0;
    let nx = group.nx, ny = group.ny, nz = group.nz;
    let cx = 0, cy = 0, cz = 0;
    for (let g = 0; g < group.manifolds.length; g++) {
        const m = group.manifolds[g];
        const dynIsA = m.bodyA === dyn;
        for (let i = 0; i < m.points.length; i++) {
            const p = m.points[i];
            const isEngaged = p.normalLambda < 0;
            if (isEngaged) {
                engaged++;
                if (p._preSolveNormalVel > maxPre) maxPre = p._preSolveNormalVel;
                totLam += Math.abs(p.normalLambda);
            } else if (!group.useAll) continue;
            if (dynIsA) p.currentAnchorAInto(anchor, m.bodyA);
            else p.currentAnchorBInto(anchor, m.bodyB);
            cx += anchor.x; cy += anchor.y; cz += anchor.z;
            cnt++;
        }
    }
    if (engaged < 1 || cnt < 2) return false;

    const inv = 1 / cnt;
    cx *= inv; cy *= inv; cz *= inv;
    this._rA.set(cx - dyn.position.x, cy - dyn.position.y, cz - dyn.position.z);

    // --- Restitution at the shared centroid (the grouped surfaces are static: no velocity) ---
    const restitution = Math.max(dyn.restitution, other.restitution);
    if (restitution > 0) {
        const g = dyn.gravity || other.gravity || gravity;
        const gravityMag = Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
        const restitutionThreshold = gravityMag * h * Solver.RESTITUTION_SLOP_FACTOR;
        if (maxPre > restitutionThreshold) {
            const va = this._pointVelocity(dyn, this._rA, this._tmpDispB);
            const relN = -(va.x * nx + va.y * ny + va.z * nz);
            const targetN = -restitution * maxPre;
            if (targetN < relN) {
                this._rB.set(0, 0, 0);
                const wN = this._effectiveMass(dyn, other, this._rA, this._rB, nx, ny, nz);
                if (wN >= 1e-12) this._applyVelocityImpulse(dyn, other, this._rA, this._rB, nx, ny, nz, (targetN - relN) / wN);
            }
        }
    }

    // --- Friction at the shared centroid (Coulomb cap = friction * total engaged normal impulse) ---
    const friction = group.friction;
    if (friction > 0) {
        const maxImpulse = friction * totLam / h;
        if (maxImpulse > 0) {
            const va = this._pointVelocity(dyn, this._rA, this._tmpDispB);
            const rvx = -va.x, rvy = -va.y, rvz = -va.z;
            const vn = rvx * nx + rvy * ny + rvz * nz;
            const vtx = rvx - vn * nx, vty = rvy - vn * ny, vtz = rvz - vn * nz;
            const vtMag = Math.sqrt(vtx * vtx + vty * vty + vtz * vtz);
            if (vtMag >= 1e-12) {
                const tx = vtx / vtMag, ty = vty / vtMag, tz = vtz / vtMag;
                this._rB.set(0, 0, 0);
                const wT = this._effectiveMass(dyn, other, this._rA, this._rB, tx, ty, tz);
                if (wT >= 1e-12) {
                    let jt = vtMag / wT;
                    if (jt > maxImpulse) jt = maxImpulse;
                    this._applyVelocityImpulse(dyn, other, this._rA, this._rB, -tx, -ty, -tz, jt);
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
    const stableVertical = this._suppressQuietVerticalLanding(bodyA, bodyB, point, nx, ny, nz);

    // --- Restitution (normal) ---
    const restitution = Math.max(bodyA.restitution, bodyB.restitution);
    const relN = this._contactRelativeNormalVelocity(point, bodyA, bodyB);
    const g = bodyA.gravity || bodyB.gravity || gravity;
    const gravityMag = Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
    const restitutionThreshold = gravityMag * h * Solver.RESTITUTION_SLOP_FACTOR;
    if (restitution > 0 && point._preSolveNormalVel > restitutionThreshold) {
        const targetN = -restitution * point._preSolveNormalVel;
        if (targetN < relN) {
            const wN = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
            if (wN >= 1e-12) this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, (targetN - relN) / wN, stableVertical);
        }
    }

    // --- Friction (tangent) ---
    const friction = Math.sqrt(bodyA.friction * bodyB.friction);
    if (friction <= 0) return;
    const maxImpulse = friction * Math.abs(point.normalLambda) / h;
    if (maxImpulse <= 0) return;

    this._contactRelativeVelocity(point, bodyA, bodyB, this._tmpDispA);
    const vn = this._tmpDispA.x * nx + this._tmpDispA.y * ny + this._tmpDispA.z * nz;
    const vtx = this._tmpDispA.x - vn * nx, vty = this._tmpDispA.y - vn * ny, vtz = this._tmpDispA.z - vn * nz;
    const vtMag = Math.sqrt(vtx * vtx + vty * vty + vtz * vtz);
    if (vtMag < 1e-12) return;

    const tx = vtx / vtMag, ty = vty / vtMag, tz = vtz / vtMag;
    const wT = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, tx, ty, tz);
    if (wT < 1e-12) return;
    let jt = vtMag / wT;
    if (jt > maxImpulse) jt = maxImpulse;
    this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, -tx, -ty, -tz, jt, stableVertical);
};

// Damps relative angular velocity in the contact's tangent plane (spin about the normal is left
// alone). Applied once per manifold at the most-engaged point; per-point splitting oscillates.
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
    if (bodyA._mass_inverted > 0) {
        const IA = bodyA._worldInverseInertiaTensor;
        wSum += ax * (IA.e00 * ax + IA.e01 * ay + IA.e02 * az) + ay * (IA.e10 * ax + IA.e11 * ay + IA.e12 * az) + az * (IA.e20 * ax + IA.e21 * ay + IA.e22 * az);
    }
    if (bodyB._mass_inverted > 0) {
        const IB = bodyB._worldInverseInertiaTensor;
        wSum += ax * (IB.e00 * ax + IB.e01 * ay + IB.e02 * az) + ay * (IB.e10 * ax + IB.e11 * ay + IB.e12 * az) + az * (IB.e20 * ax + IB.e21 * ay + IB.e22 * az);
    }
    if (wSum < 1e-12) return;

    const maxAngImpulse = angularFriction * Math.abs(point.normalLambda) / h;
    if (maxAngImpulse <= 0) return;
    let j = relWMag / wSum;
    if (j > maxAngImpulse) j = maxAngImpulse;

    if (bodyA._mass_inverted > 0) {
        const IA = bodyA._worldInverseInertiaTensor;
        const tqx = ax * j, tqy = ay * j, tqz = az * j;
        bodyA.angular_velocity.x += (IA.e00 * tqx + IA.e01 * tqy + IA.e02 * tqz) * bodyA.angular_factor.x;
        bodyA.angular_velocity.y += (IA.e10 * tqx + IA.e11 * tqy + IA.e12 * tqz) * bodyA.angular_factor.y;
        bodyA.angular_velocity.z += (IA.e20 * tqx + IA.e21 * tqy + IA.e22 * tqz) * bodyA.angular_factor.z;
    }
    if (bodyB._mass_inverted > 0) {
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
proto._applyVelocityImpulse = function (bodyA, bodyB, rA, rB, dx, dy, dz, j, suppressAngular) {
    const px = dx * j, py = dy * j, pz = dz * j;
    if (bodyA._mass_inverted > 0) {
        bodyA.linear_velocity.x -= px * bodyA._mass_inverted * bodyA.linear_factor.x;
        bodyA.linear_velocity.y -= py * bodyA._mass_inverted * bodyA.linear_factor.y;
        bodyA.linear_velocity.z -= pz * bodyA._mass_inverted * bodyA.linear_factor.z;
        if (!suppressAngular) this._applyAngularVelocityImpulse(bodyA, rA, -px, -py, -pz);
    }
    if (bodyB._mass_inverted > 0) {
        bodyB.linear_velocity.x += px * bodyB._mass_inverted * bodyB.linear_factor.x;
        bodyB.linear_velocity.y += py * bodyB._mass_inverted * bodyB.linear_factor.y;
        bodyB.linear_velocity.z += pz * bodyB._mass_inverted * bodyB.linear_factor.z;
        if (!suppressAngular) this._applyAngularVelocityImpulse(bodyB, rB, px, py, pz);
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
