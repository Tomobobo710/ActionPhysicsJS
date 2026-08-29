/**
 * XPBD solver. One solver, per plan.md ("Solver: XPBD, one solver only") - no PGS fallback exists
 * or is planned. Written from the algorithm (Muller et al., "Detailed Rigid Body Simulation with
 * Extended Position Based Dynamics", 2020; Macklin et al.'s earlier XPBD paper for the compliance
 * formulation) - same reasoning as GJK/EPA: this is stage 6, one of the two stages plan.md names
 * as where the engine is won or lost, and a structural anchor to the predecessor's PGS-shaped
 * implementation would be exactly the wrong thing to carry over.
 *
 * THE CENTRAL DESIGN RULE (plan.md, "Solver: XPBD, one solver only" and the pipeline table):
 * velocity is DERIVED from position (v = (x - x_prev) / h) and used RAW - no clamp, no slop, no
 * per-body governor, anywhere in this file. If a body's derived velocity is ever wrong, that is a
 * NARROWPHASE bug (bad contact depth/normal), and the fix belongs there, not here. This is the
 * lesson from Goblin's derived-velocity problem (plan.md's own extended writeup): three clamp
 * variants were tried and each traded one failure for another, because the clamp was never the
 * real fix - a box arriving already 0.089 deep (five substeps of undetected travel) was the bug,
 * and clamping the resulting spin only hid it. Detection quality is what actually prevents that
 * here, not this file.
 *
 * NO POINT-COUNT DIVISOR (plan.md, Bug reference / Solver): each contact point's own accumulated
 * lambda already does the job a divisor was invented to do (stop N-point overcorrection) -
 * removing that compensating term in the predecessor dropped iterations from 15 to 1 and frame
 * cost from 8.43ms to 6.87ms. No division by point count appears anywhere below.
 *
 * SUBSTEPPING: gravity/forces integrate once per substep; each substep runs its own XPBD position
 * solve (a fixed small iteration count per substep, not many iterations of a single big step) -
 * this is the "many small steps" XPBD formulation, not "few steps with many solver iterations."
 */
class Solver {
    constructor(opts) {
        opts = opts || {};
        this.substeps = opts.substeps || 4;
        this.iterations = opts.iterations || 1; // position-solve passes PER SUBSTEP
        // Scratch, owned entirely by the solver (plan.md: per-stage arena, never a shared global).
        this._rA = new Vector3(); this._rB = new Vector3();
        this._deltaPos = new Vector3();
        this._impulse = new Vector3();
        this._tangent1 = new Vector3(); this._tangent2 = new Vector3();
        this._angularCorrA = new Vector3(); this._angularCorrB = new Vector3();
        this._tmpDispA = new Vector3(); this._tmpDispB = new Vector3(); this._tmpPrev = new Vector3(); // friction slip scratch
        this._prevPos = new Map(); // bodyId -> Vector3, this substep's PRE-integration position
        this._prevRot = new Map(); // bodyId -> Quaternion, this substep's PRE-integration rotation
    }

    /**
     * Advances every dynamic body in `bodies` by `dt`, resolving the contacts in `manifolds`
     * (a ContactManifoldList). Gravity is `gravity` (a Vector3) unless a body overrides it via
     * RigidBody.gravity. May assume every contact's depth/normal is accurate (Rule 1) - never
     * re-checks or discounts a contact's own geometry.
     *
     * `refresh(manifolds)`, if given, re-measures each existing contact point's geometry (normal,
     * anchors, depth) against the predicted positions once per substep, before the constraint
     * solve - the interleaved detect-then-solve that makes rotating/corner contacts stable. It must
     * only update existing points' geometry, never add/remove/re-match them (that is the manifold's
     * once-per-tick job). Omitted -> tick-start geometry is reused every substep.
     */
    step(bodies, manifolds, gravity, dt, refresh) {
        const h = dt / this.substeps;
        for (let s = 0; s < this.substeps; s++) {
            this._substep(bodies, manifolds, gravity, h, refresh);
        }
    }

    _substep(bodies, manifolds, gravity, h, refresh) {
        // 1. Integrate velocities (gravity/forces) and predict positions.
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            this._prevPos.set(b.id, new Vector3().copy(b.position));
            this._prevRot.set(b.id, new Quaternion().copy(b.rotation));

            const g = b.gravity || gravity;
            b.linear_velocity.x += g.x * h * b.linear_factor.x;
            b.linear_velocity.y += g.y * h * b.linear_factor.y;
            b.linear_velocity.z += g.z * h * b.linear_factor.z;

            // Damping applied to velocity before the position predict, same substep - standard
            // XPBD ordering (Muller et al. section 3.1).
            if (b.linear_damping > 0) b.linear_velocity.scaleInPlace(Math.max(0, 1 - b.linear_damping * h));
            if (b.angular_damping > 0) b.angular_velocity.scaleInPlace(Math.max(0, 1 - b.angular_damping * h));

            b.position.addScaledInPlace(b.linear_velocity, h);
            Solver._integrateRotation(b.rotation, b.angular_velocity, h);
            // The world inverse inertia tensor depends on the rotation, which just changed - refresh
            // it so _effectiveMass and _applyAngularCorrection this substep use the CURRENT
            // orientation, not the tick-start one. Cheap (one 3x3 similarity transform) and keeps
            // the angular math consistent with the per-substep geometry refresh below; a fast-
            // rotating body would otherwise solve against an orientation several substeps stale.
            b._recomputeWorldInverseInertia();
        }

        // 1b. Re-measure contact geometry against the just-predicted positions. This is the
        // interleaved detect-then-solve that keeps rotating/corner contacts stable: without it the
        // contact's normal and anchors are frozen at tick-start, so a body that rotates fast enough
        // for its contact CORNER to move between substeps gets solved against stale geometry, its
        // far corner slams in undetected, and the one-shot correction of the resulting deep overlap
        // injects a large derived angular velocity (which only grows as the substep shrinks - the
        // rotational form of the derived-velocity problem). `refresh` re-measures geometry ONLY; it
        // never adds/removes/re-matches points (that stays the manifold's once-per-tick job). See
        // step()'s doc and World.step.
        if (refresh) refresh(manifolds);

        // 2. Reset this substep's accumulated lambda to zero (XPBD's lambda is PER-SUBSTEP, reset
        // every substep, not carried across substeps within a tick - only carried across TICKS via
        // the manifold's warm start, which primes the solver's FIRST guess but each substep's own
        // constraint solve still starts its own lambda accumulation at zero for THIS substep's
        // Lagrange multiplier). See Muller et al. section 3.3.
        for (const manifold of manifolds.values()) {
            for (let i = 0; i < manifold.points.length; i++) {
                manifold.points[i].normalLambda = 0;
                manifold.points[i].tangentLambda1 = 0;
                manifold.points[i].tangentLambda2 = 0;
            }
        }

        // 3. Position-level constraint solve: contacts.
        for (let iter = 0; iter < this.iterations; iter++) {
            for (const manifold of manifolds.values()) {
                this._solveManifold(manifold, h);
            }
        }

        // 4. Derive velocity from the position change - RAW, no clamp (see class header).
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            const prevPos = this._prevPos.get(b.id);
            const prevRot = this._prevRot.get(b.id);
            b.linear_velocity.x = (b.position.x - prevPos.x) / h;
            b.linear_velocity.y = (b.position.y - prevPos.y) / h;
            b.linear_velocity.z = (b.position.z - prevPos.z) / h;
            Solver._deriveAngularVelocity(b.angular_velocity, prevRot, b.rotation, h);
        }
    }

    // this = this * (0 + w) * h * 0.5, then normalize - standard PBD quaternion integration
    // (Muller et al.). Written directly rather than via a general quaternion-derivative helper
    // since it's the one place this exact operation is needed.
    static _integrateRotation(rotation, angularVelocity, h) {
        const wx = angularVelocity.x, wy = angularVelocity.y, wz = angularVelocity.z;
        const qx = rotation.x, qy = rotation.y, qz = rotation.z, qw = rotation.w;
        const half = h * 0.5;
        rotation.x = qx + half * (wx * qw + wy * qz - wz * qy);
        rotation.y = qy + half * (wy * qw + wz * qx - wx * qz);
        rotation.z = qz + half * (wz * qw + wx * qy - wy * qx);
        rotation.w = qw + half * (-wx * qx - wy * qy - wz * qz);
        rotation.normalize();
    }

    // Derived angular velocity from the rotation delta between prevRot and rotation, into `out`.
    // omega = 2 * (q_new * conj(q_prev)).xyz / h, sign-corrected so the shorter rotation path is
    // always taken (a quaternion and its negation represent the same rotation, but the derivative
    // formula needs a consistent sign to avoid spurious 2x-angle spins).
    static _deriveAngularVelocity(out, prevRot, rotation, h) {
        let dqx = rotation.w * (-prevRot.x) + rotation.x * prevRot.w + rotation.y * (-prevRot.z) - rotation.z * (-prevRot.y);
        let dqy = rotation.w * (-prevRot.y) - rotation.x * (-prevRot.z) + rotation.y * prevRot.w + rotation.z * (-prevRot.x);
        let dqz = rotation.w * (-prevRot.z) + rotation.x * (-prevRot.y) - rotation.y * (-prevRot.x) + rotation.z * prevRot.w;
        let dqw = rotation.w * prevRot.w - rotation.x * (-prevRot.x) - rotation.y * (-prevRot.y) - rotation.z * (-prevRot.z);
        if (dqw < 0) { dqx = -dqx; dqy = -dqy; dqz = -dqz; } // shorter path
        out.x = 2 * dqx / h; out.y = 2 * dqy / h; out.z = 2 * dqz / h;
    }

    _solveManifold(manifold, h) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
        for (let i = 0; i < manifold.points.length; i++) {
            this._solvePoint(manifold.points[i], bodyA, bodyB, h);
        }
    }

    // One XPBD position-constraint solve for a single contact point.
    //
    // C is recomputed HERE, every call, from the bodies' CURRENT positions via each point's fixed
    // local anchor offsets (ContactDetails.currentAnchorAInto/B) - never read directly from
    // point.signedDistance, which is a snapshot from this TICK's single narrowphase pass, already
    // stale by the second substep. Reusing that stale value was a real bug caught while building
    // this file: a resting box's true overlap grows a little each substep as gravity keeps pulling
    // it down, but a stale, constant C applied every substep with no correction ever making it
    // smaller (because "smaller" was never re-measured) still gets a FRESH deltaLambda each call -
    // the same one, over and over - overcorrecting the box upward well past resting, tick after
    // tick, with nothing to signal "this is done" because the signal (C actually shrinking as the
    // real overlap resolves) never arrived. Recomputing C live is what makes convergence within a
    // tick's substeps possible at all: each correction actually reduces the NEXT measured C.
    //
    // Compliance is 0 here (rigid, infinitely stiff contact) - plan.md's own open question
    // ("whether compliance is ever non-zero in practice") is left open; 0 is the correct default
    // for a contact that should not be springy.
    _solvePoint(point, bodyA, bodyB, h) {
        point.currentAnchorAInto(this._rA, bodyA);
        point.currentAnchorBInto(this._rB, bodyB);
        const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;
        // normal points from B to A (matching GJK/EPA's own convention - verified directly
        // against GJK's separated-result normal after finding and fixing a real sign bug in EPA's
        // own output, see EPA.js). C = (anchorB - anchorA) . normal is positive exactly when B's
        // anchor has moved PAST A's anchor in the normal's own direction - i.e. penetrating.
        const C = (this._rB.x - this._rA.x) * nx + (this._rB.y - this._rA.y) * ny + (this._rB.z - this._rA.z) * nz;
        // SPECULATIVE CONTACT (plan.md, "Continuous collision / speculative contacts"): this guard,
        // combined with the point being detected BEFORE overlap (narrowphase's speculative margin),
        // IS the speculative mechanism - no separate code path. The point is created while still
        // separated (negative signedDistance), so it already exists in the manifold. Then every
        // substep the position predict (Solver._substep step 1) moves the body forward FIRST, and C
        // is re-measured HERE against that predicted position. If the predicted motion overshot the
        // touching plane, C > 0 and the constraint pulls the body back to exactly touch (C = 0) -
        // never deeper, because C is the live overshoot, not a whole tick's accumulated penetration.
        // If the predicted motion did NOT reach touch (C <= 0, still a real gap), there is nothing
        // to correct this substep: a non-penetration contact only ever PUSHES APART, it never pulls
        // a separated pair together across a gap. That is the entire fix for the derived-velocity
        // problem: Δx per substep is now just the small overshoot beyond touch, so derived velocity
        // v = Δx/h stays small, with no clamp anywhere (the central design rule holds).
        if (C <= 0) return;

        // rA/rB for the effective-mass and angular-correction math are offsets from each body's
        // CENTER (not the anchor's world position itself) - overwrite _rA/_rB in place now that C
        // has been read from their anchor positions above.
        Vector3.subInto(this._rA, this._rA, bodyA.position);
        Vector3.subInto(this._rB, this._rB, bodyB.position);

        // Effective inverse mass along the normal: w = 1/mA + 1/mB + (rA x n)·I_A^-1·(rA x n) +
        // (rB x n)·I_B^-1·(rB x n) - the standard generalized inverse mass for a linear+angular
        // constraint (Muller et al. section 3.2, eq. 8).
        const wSum = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
        if (wSum < 1e-12) return; // both bodies effectively immovable along this normal - nothing to solve

        // XPBD Lagrange multiplier update, rigid (compliance-free) contact: deltaLambda =
        // -C / wSum (alpha/h^2 term drops out entirely when compliance is 0 - Muller et al. eq 4).
        // Sign note for this file's convention: C > 0 means penetrating (guarded above), so
        // deltaLambda < 0, and _applyPositionalCorrection's verified pairing turns a NEGATIVE
        // deltaLambda into a push-apart correction. A pushing contact therefore accumulates a
        // NEGATIVE normalLambda here - the opposite sign from the textbook's non-negative
        // convention, purely because the normal points B->A rather than A->B. The physical
        // constraint "a contact can push apart but never pull together" is therefore normalLambda
        // <= 0: clamp the accumulated value at 0 from above and feed back only the change actually
        // applied, so an over-correction on a later iteration/substep can relax the push but can
        // never invert it into an attractive pull across the contact.
        const oldLambda = point.normalLambda;
        let newLambda = oldLambda - C / wSum;
        if (newLambda > 0) newLambda = 0; // contact cannot pull; clamp to no-push
        const deltaLambda = newLambda - oldLambda;
        point.normalLambda = newLambda;

        this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, deltaLambda);

        // Friction: tangential position correction, capped by Coulomb's law using the JUST-UPDATED
        // normalLambda (matches Muller et al.'s ordering: friction uses this iteration's normal
        // impulse, not last iteration's). Two tangent directions spanning the contact plane.
        this._solveFriction(point, bodyA, bodyB, h);
    }

    // Generalized inverse mass along direction (dx,dy,dz) for the pair, combining linear and
    // angular contributions from both bodies. Shared by the normal and each friction direction.
    _effectiveMass(bodyA, bodyB, rA, rB, dx, dy, dz) {
        // linear_factor is a per-axis WORLD-space velocity mask, but this constraint's own
        // direction is a single world vector - exact per-axis locking against an arbitrary contact
        // normal isn't captured by a single isotropic scale, so the isotropic inverse mass is used
        // directly here. Locked axes are a character-controller feature (movement clamped to a
        // plane), not something a general rigid-body contact normal needs to interact with
        // precisely - revisit if that combination turns out to matter.
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
    }

    // Applies the position correction dLambda * n (and the matching angular correction) to both
    // bodies, scaled by their own inverse mass/inertia - standard XPBD position update (Muller et
    // al. eq 6-7). Sign convention: correction pushes A along +n and B along -n, matching normal's
    // B-to-A direction.
    // C = (anchorB - anchorA) . n (see _solvePoint), so the constraint GRADIENT is dC/d(bodyA
    // position) = -n and dC/d(bodyB position) = +n. XPBD's update Δx = Δλ * w * ∇C (Muller et al.
    // eq 6) therefore moves A along -n*Δλ and B along +n*Δλ - the OPPOSITE pairing from what a
    // naive "A gets +n, B gets -n" guess would produce. Verified against a concrete case: a box
    // resting ON TOP of static ground, overlapping by a small amount, with normal pointing from B
    // (the box) to A (the ground) - i.e. DOWNWARD. deltaLambda comes out negative for a positive
    // overlap C, and the box (body B) must move UP to resolve it: +n*deltaLambda with n pointing
    // down and deltaLambda negative gives a positive (upward) y-component - correct only with
    // THIS pairing, not the reversed one that was here before (which pushed the box down, through
    // the ground, for the same inputs - the actual fall-through bug this comment is fixing).
    _applyPositionalCorrection(bodyA, bodyB, rA, rB, nx, ny, nz, dLambda) {
        const px = nx * dLambda, py = ny * dLambda, pz = nz * dLambda;

        if (bodyA._massInverted > 0) {
            bodyA.position.x -= px * bodyA._massInverted * bodyA.linear_factor.x;
            bodyA.position.y -= py * bodyA._massInverted * bodyA.linear_factor.y;
            bodyA.position.z -= pz * bodyA._massInverted * bodyA.linear_factor.z;
            this._applyAngularCorrection(bodyA, rA, -px, -py, -pz);
        }
        if (bodyB._massInverted > 0) {
            bodyB.position.x += px * bodyB._massInverted * bodyB.linear_factor.x;
            bodyB.position.y += py * bodyB._massInverted * bodyB.linear_factor.y;
            bodyB.position.z += pz * bodyB._massInverted * bodyB.linear_factor.z;
            this._applyAngularCorrection(bodyB, rB, px, py, pz);
        }
    }

    // Rotates `body` by the small-angle correction (I^-1 * (r x p)) * 0.5, the standard PBD
    // angular position update from a linear positional impulse applied at offset r (Muller et al.
    // eq 7-8, via the same quaternion-derivative integration _integrateRotation uses).
    _applyAngularCorrection(body, r, px, py, pz) {
        const torqueX = r.y * pz - r.z * py, torqueY = r.z * px - r.x * pz, torqueZ = r.x * py - r.y * px;
        const I = body._worldInverseInertiaTensor;
        const wx = I.e00 * torqueX + I.e01 * torqueY + I.e02 * torqueZ;
        const wy = I.e10 * torqueX + I.e11 * torqueY + I.e12 * torqueZ;
        const wz = I.e20 * torqueX + I.e21 * torqueY + I.e22 * torqueZ;
        this._angularCorrA.set(wx * body.angular_factor.x, wy * body.angular_factor.y, wz * body.angular_factor.z);
        Solver._integrateRotation(body.rotation, this._angularCorrA, 1); // h=1: this IS the delta, not a rate
    }

    // Friction via a persistent anchor, scaled to match the normal constraint's per-substep units.
    //
    // The friction anchor is a material point coincident on both bodies when the contact last stuck.
    // Its tangential separation NOW is the total slip to resist, but that raw separation grows every
    // substep while the normal constraint's error C is a small per-substep penetration - solving the
    // raw separation directly produces a lambda in different units from |normalLambda|, so the
    // Coulomb cap (mu*|normalLambda|) is meaningless against it and friction either never bites or
    // always saturates (the bug that made a box slide down a 15-degree slope it should stick on).
    //
    // The fix: this XPBD contact lambda scales with h^2 (deltaLambda = -C/wSum where the normal C is
    // itself the h^2-order overlap). To keep friction commensurate, the tangential error is likewise
    // taken as a per-substep quantity: the slip that occurred THIS substep (anchor separation minus
    // what it was at substep start), not the accumulated total. That per-substep slip is the same
    // order as the normal penetration, so mu*|normalLambda| caps it correctly - static stick when the
    // slip stays within the cone, dynamic slide (anchor dragged) when it exceeds it.
    _solveFriction(point, bodyA, bodyB, h) {
        const friction = Math.sqrt(bodyA.friction * bodyB.friction); // combined friction, geometric mean (standard convention)
        if (friction <= 0) return;
        const maxFriction = friction * Math.abs(point.normalLambda); // Coulomb cap magnitude (normalLambda <= 0 here)
        if (maxFriction <= 0) return;

        Solver._tangentBasis(point.normal, this._tangent1, this._tangent2);
        this._currentMaxFriction = maxFriction;
        this._solveFrictionAxis(point, bodyA, bodyB, this._tangent1, true);
        this._solveFrictionAxis(point, bodyA, bodyB, this._tangent2, false);
    }

    // One tangent axis: resist THIS substep's tangential slip of the contact anchors, Coulomb-capped
    // as a disc against the other axis (isotropic, so diagonal friction cannot exceed mu*N by
    // sqrt(2)). The slip is measured from prevPos/prevRot (substep start) to now, so it is a
    // per-substep quantity commensurate with the normal constraint - see _solveFriction's header.
    _solveFrictionAxis(point, bodyA, bodyB, tangent, isFirstAxis) {
        const slip = this._contactSlipAlong(point, bodyA, bodyB, tangent);
        point.currentAnchorAInto(this._rA, bodyA);
        point.currentAnchorBInto(this._rB, bodyB);
        Vector3.subInto(this._rA, this._rA, bodyA.position);
        Vector3.subInto(this._rB, this._rB, bodyB.position);
        const tx = tangent.x, ty = tangent.y, tz = tangent.z;
        const wSum = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, tx, ty, tz);
        if (wSum < 1e-12) return;

        const lambdaBefore = isFirstAxis ? point.tangentLambda1 : point.tangentLambda2;
        let newLambda = lambdaBefore - slip / wSum;
        // Coulomb disc cap on the COMBINED two-axis magnitude.
        const other = isFirstAxis ? point.tangentLambda2 : point.tangentLambda1;
        const maxFriction = this._currentMaxFriction;
        const mag = Math.sqrt(newLambda * newLambda + other * other);
        if (mag > maxFriction && mag > 0) newLambda *= maxFriction / mag;
        const deltaLambda = newLambda - lambdaBefore;
        if (isFirstAxis) point.tangentLambda1 = newLambda; else point.tangentLambda2 = newLambda;

        this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, tx, ty, tz, deltaLambda);
    }

    // Tangential slip along `tangent` this substep: (dispB - dispA).tangent, where dispX is how far
    // body X's contact anchor moved from substep start (prevPos/prevRot) to now. (B - A) ordering
    // matches the normal constraint so the shared correction OPPOSES slip rather than reinforcing it
    // (the opposite ordering turns friction into an accelerator). A static body has no prev entry and
    // contributes zero displacement - correct, it did not move.
    _contactSlipAlong(point, bodyA, bodyB, tangent) {
        const dispA = this._anchorDisplacement(point.localAnchorA, bodyA, this._tmpDispA);
        const dispB = this._anchorDisplacement(point.localAnchorB, bodyB, this._tmpDispB);
        const dx = dispB.x - dispA.x, dy = dispB.y - dispA.y, dz = dispB.z - dispA.z;
        return dx * tangent.x + dy * tangent.y + dz * tangent.z;
    }

    // World displacement of the point at `localAnchor` on `body` from substep start to now.
    _anchorDisplacement(localAnchor, body, out) {
        const prevPos = this._prevPos.get(body.id);
        if (!prevPos) { out.set(0, 0, 0); return out; } // static/kinematic: did not move this substep
        const prevRot = this._prevRot.get(body.id);
        out.copy(localAnchor);
        body.rotation.transformVectorInPlace(out);
        out.addInPlace(body.position);
        this._tmpPrev.copy(localAnchor);
        prevRot.transformVectorInPlace(this._tmpPrev);
        this._tmpPrev.addInPlace(prevPos);
        out.subInPlace(this._tmpPrev);
        return out;
    }

    // Two unit vectors spanning the plane perpendicular to `normal` - the friction directions.
    static _tangentBasis(normal, outT1, outT2) {
        outT1.findOrthogonal(normal);
        Vector3.crossInto(outT2, normal, outT1);
    }
}

ActionPhysics.Solver = Solver;
