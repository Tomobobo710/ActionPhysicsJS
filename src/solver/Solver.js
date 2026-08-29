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

        // 1c. Capture the pre-solve contact-relative NORMAL velocity, for restitution. It has to be
        // read here - after gravity integration and geometry refresh, but BEFORE the normal position
        // solve removes it - because restitution restores a fraction of the velocity the body was
        // approaching at, which the solve is about to zero.
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            for (let i = 0; i < manifold.points.length; i++) {
                const p = manifold.points[i];
                p._preSolveNormalVel = this._contactRelativeNormalVelocity(p, bodyA, bodyB);
            }
        }

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

        // 3. Position-level constraint solve: contacts (normal / non-penetration only).
        for (let iter = 0; iter < this.iterations; iter++) {
            for (const manifold of manifolds.values()) {
                this._solveManifold(manifold, h);
            }
        }

        // 4. Derive velocity from the position change - RAW, no clamp (see class header). This
        // captures the normal solve's effect (a stopped body has ~zero normal velocity here).
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

        // 5. Friction + restitution, applied in the VELOCITY pass (Muller et al. 2020 sec 3.6). This
        // is NOT the forbidden derived-velocity clamp (that hid a detection bug by governing the
        // whole body's velocity); it is the physical contact velocity constraint - friction removes
        // tangential relative velocity up to the Coulomb limit, restitution restores a fraction of
        // the pre-solve approach velocity. Both act only at contacts and only on the contact-relative
        // velocity, which is exactly where they belong.
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            for (let i = 0; i < manifold.points.length; i++) {
                this._solveContactVelocity(manifold.points[i], bodyA, bodyB, h);
            }
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

    // Velocity-pass contact solve: friction and restitution, applied to the contact-relative
    // velocity AFTER the position solve has set velocities (Muller et al. 2020 sec 3.6). Working in
    // velocity space here (not position) is what makes friction stable and gives true static stick:
    // a resting body's tangential contact velocity is driven to exactly zero, capped by the Coulomb
    // limit, so it does not creep - the position-anchor approach kept saturating its cap and letting
    // the body slide down a shallow slope. This is a physical contact constraint on the relative
    // velocity, NOT the derived-velocity clamp plan.md forbids (that governed a whole body's velocity
    // to hide a detection bug; this only removes the tangential rub and restores a chosen restitution
    // at the contact, which is exactly what friction and bounce ARE).
    _solveContactVelocity(point, bodyA, bodyB, h) {
        // Only act on a contact that is actually touching/overlapping right now (normalLambda < 0
        // means the normal solve pushed this substep). A purely speculative point that never engaged
        // has normalLambda == 0 and no contact velocity to correct.
        if (point.normalLambda >= 0) return;

        point.currentAnchorAInto(this._rA, bodyA);
        point.currentAnchorBInto(this._rB, bodyB);
        Vector3.subInto(this._rA, this._rA, bodyA.position);
        Vector3.subInto(this._rB, this._rB, bodyB.position);
        const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;

        // --- Restitution (normal) ---
        // Sign convention (normal points B->A): a body APPROACHING the contact has POSITIVE normal
        // relative velocity (relVel . normal > 0 = closing), and after a bounce it should SEPARATE at
        // -e * the approach speed (negative relN). Skip slow approaches (a resting body's one-substep
        // gravity nudge) via a threshold, so restitution does not turn rest into perpetual jitter.
        const restitution = Math.max(bodyA.restitution, bodyB.restitution); // combined: max, standard convention
        const relN = this._contactRelativeNormalVelocity(point, bodyA, bodyB);
        if (restitution > 0 && point._preSolveNormalVel > Solver.RESTITUTION_THRESHOLD) {
            const targetN = -restitution * point._preSolveNormalVel; // desired separating velocity (< 0)
            // Only ADD separation (make relN more negative); never damp an already-separating contact.
            if (targetN < relN) {
                const wN = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
                if (wN >= 1e-12) this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, (targetN - relN) / wN);
            }
        }

        // --- Friction (tangent) ---
        const friction = Math.sqrt(bodyA.friction * bodyB.friction);
        if (friction <= 0) return;
        // Coulomb cap on the friction impulse magnitude: mu times the normal impulse the position
        // solve actually applied this substep. normalLambda is a position Lagrange multiplier;
        // dividing by h converts it to a velocity-space impulse commensurate with the tangential
        // impulses computed below. This is the correct, unit-consistent cap that the position-space
        // attempt never got right.
        const maxImpulse = friction * Math.abs(point.normalLambda) / h;
        if (maxImpulse <= 0) return;

        // Current tangential relative velocity, and the impulse that would zero it.
        this._contactRelativeVelocity(point, bodyA, bodyB, this._tmpDispA); // full relative velocity -> _tmpDispA
        const vn = this._tmpDispA.x * nx + this._tmpDispA.y * ny + this._tmpDispA.z * nz;
        let vtx = this._tmpDispA.x - vn * nx, vty = this._tmpDispA.y - vn * ny, vtz = this._tmpDispA.z - vn * nz;
        const vtMag = Math.sqrt(vtx * vtx + vty * vty + vtz * vtz);
        if (vtMag < 1e-12) return; // no tangential motion to resist

        const tx = vtx / vtMag, ty = vty / vtMag, tz = vtz / vtMag; // tangent = slip direction
        const wT = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, tx, ty, tz);
        if (wT < 1e-12) return;
        // Impulse to fully stop the tangential velocity, clamped to the Coulomb cap (static stick
        // when the full stop is within budget, dynamic slide when clamped).
        let jt = vtMag / wT;
        if (jt > maxImpulse) jt = maxImpulse;
        // Apply along -tangent (oppose the slip).
        this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, -tx, -ty, -tz, jt);
    }

    // Contact-relative velocity (velocity of B's contact point minus A's), into `out`. rA/rB are the
    // center-relative contact offsets (already in this._rA/_rB when called from the velocity pass,
    // but recomputed here from the anchors so this is usable standalone).
    _contactRelativeVelocity(point, bodyA, bodyB, out) {
        point.currentAnchorAInto(this._tmpPrev, bodyA);
        this._tmpPrev.subInPlace(bodyA.position); // rA (center-relative)
        const va = this._pointVelocity(bodyA, this._tmpPrev, this._tmpDispB);
        const vax = va.x, vay = va.y, vaz = va.z;
        point.currentAnchorBInto(this._tmpPrev, bodyB);
        this._tmpPrev.subInPlace(bodyB.position); // rB
        const vb = this._pointVelocity(bodyB, this._tmpPrev, this._tmpDispB);
        out.set(vb.x - vax, vb.y - vay, vb.z - vaz);
        return out;
    }

    // Velocity of the material point at center-relative offset r on `body`: v + omega x r.
    _pointVelocity(body, r, out) {
        const w = body.angular_velocity, v = body.linear_velocity;
        out.set(
            v.x + (w.y * r.z - w.z * r.y),
            v.y + (w.z * r.x - w.x * r.z),
            v.z + (w.x * r.y - w.y * r.x)
        );
        return out;
    }

    // Contact-relative velocity along the normal (B->A). Scalar.
    _contactRelativeNormalVelocity(point, bodyA, bodyB) {
        this._contactRelativeVelocity(point, bodyA, bodyB, this._tmpDispA);
        return this._tmpDispA.x * point.normal.x + this._tmpDispA.y * point.normal.y + this._tmpDispA.z * point.normal.z;
    }

    // Apply a velocity-space impulse j*(dir) at contact offsets rA/rB: A gets -j (B->A convention,
    // matching the position correction's own pairing), B gets +j, each scaled by inverse mass, with
    // the matching angular velocity change. rA/rB are center-relative offsets.
    _applyVelocityImpulse(bodyA, bodyB, rA, rB, dx, dy, dz, j) {
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
    }

    // Angular velocity change from a linear impulse p applied at center-relative offset r:
    // dOmega = I^-1 (r x p). (Velocity-space analog of _applyAngularCorrection.)
    _applyAngularVelocityImpulse(body, r, px, py, pz) {
        const tqx = r.y * pz - r.z * py, tqy = r.z * px - r.x * pz, tqz = r.x * py - r.y * px;
        const I = body._worldInverseInertiaTensor;
        body.angular_velocity.x += (I.e00 * tqx + I.e01 * tqy + I.e02 * tqz) * body.angular_factor.x;
        body.angular_velocity.y += (I.e10 * tqx + I.e11 * tqy + I.e12 * tqz) * body.angular_factor.y;
        body.angular_velocity.z += (I.e20 * tqx + I.e21 * tqy + I.e22 * tqz) * body.angular_factor.z;
    }

    // Two unit vectors spanning the plane perpendicular to `normal` - the friction directions.
    static _tangentBasis(normal, outT1, outT2) {
        outT1.findOrthogonal(normal);
        Vector3.crossInto(outT2, normal, outT1);
    }
}

// Approach speeds slower than this (m/s) do not bounce - below it, restitution would turn a resting
// body's one-substep gravity nudge into perpetual micro-jitter. Standard restitution slop.
Solver.RESTITUTION_THRESHOLD = 0.5;

ActionPhysics.Solver = Solver;
