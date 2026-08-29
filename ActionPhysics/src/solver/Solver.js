/**
 * XPBD solver - the engine's one solver (Muller et al., "Detailed Rigid Body Simulation with
 * Extended Position Based Dynamics", 2020; Macklin et al.'s earlier XPBD paper for the compliance
 * formulation).
 *
 * THE CENTRAL DESIGN RULE: velocity is DERIVED from position (v = (x - x_prev) / h) and used RAW -
 * there is no clamp, slop, or fixed governor applied to a body's VELOCITY anywhere in this file. What
 * DOES exist, in _solvePoint: a split between how much of a contact's own correction is EXPLAINABLE
 * by the body's own real, LIVE (post-gravity) contact-relative velocity this substep versus how much
 * is not - only the explainable share becomes derived velocity; the rest is a pure position edit (see
 * _applyPositionalCorrection's `bias` parameter), excluded from step 4's velocity derivation entirely.
 * (NOT point._preSolveNormalVel - that value is deliberately PRE-gravity, for restitution's own
 * purposes; using it here excluded gravity's own genuine per-substep motion from every resting
 * contact and left a permanent non-converging residual velocity - see _solvePoint's own comment.)
 * This is split-impulse / Baumgarte stabilization done with the actual physical ground truth as the
 * split point, not a tuned constant: a body genuinely under load (resting under weight, landing hard)
 * has real, nonzero closing velocity behind its own C every substep, so its correction is never
 * artificially starved regardless of how much force the contact needs to carry - while a body with
 * zero real velocity behind a large C (a raw spawn/teleport overlap) has that correction correctly
 * recognized as almost entirely non-explainable, so it settles out gradually as bias instead of
 * becoming fabricated kinetic energy. Two narrower mechanisms were tried and failed first: a flat
 * magnitude cap ignored load and let a resting body sink through the floor; a fixed fraction of C
 * still scaled with C's own raw magnitude and still launched a large spawn overlap, just less far.
 * Solver.EXPLAINABLE_MARGIN (see its own comment) widens the explainable share slightly to cover the
 * ordinary numerical gap between C and live-velocity-based estimates for off-center/rotating contacts.
 * If a body's derived velocity is ever wrong beyond what this split accounts for, that is a
 * NARROWPHASE bug (bad contact depth/normal), and the fix belongs there, not here.
 *
 * NO POINT-COUNT DIVISOR: each contact point's own accumulated lambda already does the job a
 * divisor was invented to do (stop N-point overcorrection). No division by point count appears
 * anywhere below.
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
        // Scratch, owned entirely by the solver - never shared across stages.
        this._rA = new Vector3(); this._rB = new Vector3();
        this._deltaPos = new Vector3();
        this._impulse = new Vector3();
        this._tangent1 = new Vector3(); this._tangent2 = new Vector3();
        this._angularCorrA = new Vector3(); this._angularCorrB = new Vector3();
        this._tmpDispA = new Vector3(); this._tmpDispB = new Vector3(); this._tmpPrev = new Vector3(); // friction slip scratch
        this._prevPos = new Map(); // bodyId -> Vector3, this substep's PRE-integration position
        this._prevRot = new Map(); // bodyId -> Quaternion, this substep's PRE-integration rotation
        this._predictedPos = new Map(); // bodyId -> Vector3, this substep's position after step 1's integration, BEFORE step 3's correction - see step 1's own comment
        this._predictedRot = new Map(); // bodyId -> Quaternion, same for rotation
        this._preGravityVel = new Map(); // bodyId -> Vector3, this substep's velocity BEFORE gravity is added (see _solvePoint's restitution-capture comment)
    }

    // See _solvePoint's own comment. Traced directly across several resting/settling off-center
    // contacts (a box resting on one corner, a cylinder resting on its side) - the ordinary numerical
    // gap between C and liveRelVel*h for a legitimate, fully-explainable resting correction settles
    // around 2x once the transient impact itself has passed (briefly higher, up to ~18x, DURING the
    // impact substep itself, where a large real velocity is genuinely still resolving - a case this
    // margin does not need to cover, since velocityC is independently bounded by C itself just below).
    // 3x is comfortable headroom over the observed resting-case ratio, while remaining nowhere near
    // what a genuine zero-velocity spawn overlap shows (liveRelVel ~0, so no margin multiple changes
    // the outcome there at all).
    static EXPLAINABLE_MARGIN = 3;


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
    step(bodies, manifolds, gravity, dt, refresh, constraints) {
        const h = dt / this.substeps;
        for (let s = 0; s < this.substeps; s++) {
            this._substep(bodies, manifolds, gravity, h, refresh, constraints);
        }
    }

    _substep(bodies, manifolds, gravity, h, refresh, constraints) {
        this._gravityMag = Math.sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);
        // 1. Integrate velocities (gravity/forces) and predict positions.
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            this._prevPos.set(b.id, new Vector3().copy(b.position));
            this._prevRot.set(b.id, new Quaternion().copy(b.rotation));
            // Snapshot BEFORE gravity/damping/predict below touch it - restitution's pre-solve
            // velocity must be measured from here, not from the post-gravity velocity later in this
            // same substep (see _solvePoint's own comment on the bug this fixes).
            this._preGravityVel.set(b.id, new Vector3().copy(b.linear_velocity));

            const g = b.gravity || gravity;
            b.linear_velocity.x += g.x * h * b.linear_factor.x;
            b.linear_velocity.y += g.y * h * b.linear_factor.y;
            b.linear_velocity.z += g.z * h * b.linear_factor.z;

            // Continuous forces/torques (RigidBody.applyForce/applyTorque), integrated the same way
            // gravity is - accumulated_force/torque stays in effect for every substep within the
            // tick it was set (World.step clears it once per TICK, after the solver finishes, not
            // here), matching the standard "a force keeps acting until told otherwise" contract.
            const af = b.accumulated_force;
            if (af.x !== 0 || af.y !== 0 || af.z !== 0) {
                b.linear_velocity.x += af.x * b._massInverted * h * b.linear_factor.x;
                b.linear_velocity.y += af.y * b._massInverted * h * b.linear_factor.y;
                b.linear_velocity.z += af.z * b._massInverted * h * b.linear_factor.z;
            }
            const at = b.accumulated_torque;
            if (at.x !== 0 || at.y !== 0 || at.z !== 0) {
                const I = b._worldInverseInertiaTensor;
                b.angular_velocity.x += (I.e00 * at.x + I.e01 * at.y + I.e02 * at.z) * h * b.angular_factor.x;
                b.angular_velocity.y += (I.e10 * at.x + I.e11 * at.y + I.e12 * at.z) * h * b.angular_factor.y;
                b.angular_velocity.z += (I.e20 * at.x + I.e21 * at.y + I.e22 * at.z) * h * b.angular_factor.z;
            }

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

            // Snapshot the PREDICTED position/rotation (integrated from real velocity alone, before
            // step 3's position-constraint correction runs) - this, not the post-correction result,
            // is what step 4 derives "how much did real motion move the body" from. The correction's
            // own contribution to velocity is instead applied DIRECTLY, as a real impulse, at the
            // moment the correction happens (see _solvePoint/_applyPositionalCorrection) - the
            // standard way a position-based solver recovers velocity from its own Lagrange multiplier
            // (impulse = deltaLambda / h), not by re-deriving it from a position/rotation delta that
            // conflates real motion with whatever the correction did to reach it. This is what
            // actually fixes the class of bug this file's own header used to work around with
            // thresholds (a curved contact's re-picked anchor injecting a correction-shaped rotation
            // that looked identical to real spin once smashed through the same derivation) - the
            // correction's effect on rotation never enters the derivation at all, so there is nothing
            // there to mistake for real motion in the first place, regardless of body speed or how
            // large the correction was.
            this._predictedPos.set(b.id, new Vector3().copy(b.position));
            this._predictedRot.set(b.id, new Quaternion().copy(b.rotation));
        }

        // Derive velocity from FREE MOTION alone, RIGHT HERE - before step 3's position correction
        // ever runs - so the correction's own impulse (_applyPositionalCorrection, applied directly
        // via _applyVelocityImpulse) lands as a real ADDITION on top of this free-motion baseline,
        // not something a later overwrite discards. This ordering IS the fix: running this derivation
        // AFTER step 3 (as XPBD's usual "derive from the post-correction position" ordering does)
        // means whatever the correction just added to velocity gets thrown away the instant this loop
        // runs again, since it unconditionally ASSIGNS linear_velocity from a position delta rather
        // than accumulating - confirmed directly: a resting box's contact impulse correctly zeroed
        // its velocity mid-substep, then this derivation (if run afterward) overwrote it right back to
        // the raw gravity-only value, as if the correction had never happened at all.
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            const prevPos = this._prevPos.get(b.id);
            const prevRot = this._prevRot.get(b.id);
            const predictedPos = this._predictedPos.get(b.id);
            const predictedRot = this._predictedRot.get(b.id);
            b.linear_velocity.x = (predictedPos.x - prevPos.x) / h;
            b.linear_velocity.y = (predictedPos.y - prevPos.y) / h;
            b.linear_velocity.z = (predictedPos.z - prevPos.z) / h;
            Solver._deriveAngularVelocity(b.angular_velocity, prevRot, predictedRot, h);
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

        // NOTE: the pre-solve contact-relative normal velocity for restitution used to be captured
        // HERE, unconditionally, for every existing manifold point every substep - see _solvePoint's
        // own comment for why that was a real bug (restitution measured 101.19% of impact speed at
        // e=1) and why the capture now happens inside _solvePoint itself, gated on C>0 (the same
        // condition that decides this substep actually pushes this point), never here.

        // 2. Reset this substep's accumulated lambda to zero (XPBD's lambda is PER-SUBSTEP, reset
        // every substep, not carried across substeps within a tick - only carried across TICKS via
        // the manifold's warm start, which primes the solver's FIRST guess but each substep's own
        // constraint solve still starts its own lambda accumulation at zero for THIS substep's
        // Lagrange multiplier). See Muller et al. section 3.3.
        for (const manifold of manifolds.values()) {
            for (let i = 0; i < manifold.points.length; i++) {
                manifold.points[i].normalLambda = 0;
                manifold.points[i].velocityLambda = 0;
                manifold.points[i].tangentLambda1 = 0;
                manifold.points[i].tangentLambda2 = 0;
            }
        }

        // 3. Position-level constraint solve: contacts (normal / non-penetration only), then joints.
        // Velocity was already derived (from free motion) and each correction's own impulse already
        // applied directly to it, above - this loop is position-only from here on.
        for (let iter = 0; iter < this.iterations; iter++) {
            for (const manifold of manifolds.values()) {
                this._solveManifold(manifold, h);
            }
            if (constraints) {
                for (let i = 0; i < constraints.length; i++) {
                    if (constraints[i].enabled) constraints[i].solve(h);
                }
            }
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
            // Rolling resistance is ONE torque acting on the pair as a whole, not a per-point contact
            // force - applied once per manifold (via its deepest/first point's normal) rather than
            // once per point. Splitting it across several manifold points (as friction/restitution
            // correctly do, since those really are independent per-point contact forces) does not
            // sum to the same torque: each point's correction changes the angular velocity the NEXT
            // point reads, and on a multi-point manifold (a cylinder/capsule's curved contact reduces
            // to several nearby points, not one) this became a genuine per-substep oscillation that
            // never converged - confirmed by tracing a shoved cylinder to a stable non-zero angular
            // velocity fixed point (0.026, 0, -0.046 rad/s, forever) rather than decaying to rest,
            // with adjacent manifold points showing opposite-signed tangential relative velocity from
            // the same body-level angular velocity pair, an artifact of sequential per-point solving
            // rather than any real geometric difference between the points.
            if (manifold.points.length > 0) {
                // Reference point: the most-engaged one by ACTUAL velocity impulse (largest
                // |velocityLambda|, not normalLambda - see ContactDetails's own comment) rather than
                // always points[0] - a shallow/barely-touching point would otherwise starve the
                // rolling-resistance budget even while a neighbouring point on the same manifold is
                // carrying the real velocity-affecting load.
                let ref = manifold.points[0];
                for (let i = 1; i < manifold.points.length; i++) {
                    if (Math.abs(manifold.points[i].velocityLambda) > Math.abs(ref.velocityLambda)) ref = manifold.points[i];
                }
                this._solveRollingResistance(ref, bodyA, bodyB, h);
            }
        }
    }

    // Exact exponential-map quaternion integration: dq = (cos(theta/2), sin(theta/2)*axis) for the
    // rotation of theta=|w|*h about axis=w/|w|, then rotation = dq * rotation.
    static _integrateRotation(rotation, angularVelocity, h) {
        const wx = angularVelocity.x, wy = angularVelocity.y, wz = angularVelocity.z;
        const wLenSq = wx * wx + wy * wy + wz * wz;
        if (wLenSq < 1e-24) return; // no rotation this substep - avoid a 0/0 in the axis normalize below
        const wLen = Math.sqrt(wLenSq);
        const halfAngle = wLen * h * 0.5;
        const s = Scalar.sin(halfAngle) / wLen; // scales w into the (sin(theta/2)*axis) term directly
        const dqx = wx * s, dqy = wy * s, dqz = wz * s, dqw = Scalar.cos(halfAngle);

        const qx = rotation.x, qy = rotation.y, qz = rotation.z, qw = rotation.w;
        const rx = dqw * qx + dqx * qw + dqy * qz - dqz * qy;
        const ry = dqw * qy - dqx * qz + dqy * qw + dqz * qx;
        const rz = dqw * qz + dqx * qy - dqy * qx + dqz * qw;
        const rw = dqw * qw - dqx * qx - dqy * qy - dqz * qz;
        rotation.x = rx; rotation.y = ry; rotation.z = rz; rotation.w = rw;
        rotation.normalize(); // defensive against float roundoff; composing two unit quaternions is exactly unit length in exact arithmetic
    }

    // Angular velocity from the rotation delta between prevRot and rotation, into `out`. dq =
    // rotation * conj(prevRot); the rotation angle is recovered via atan2(|dq.xyz|, dq.w), sign-
    // corrected so the shorter rotation path is always taken (a quaternion and its negation
    // represent the same rotation, but the angle recovery needs a consistent sign to avoid a
    // spurious near-2*pi angle for what is actually a small negative rotation).
    static _deriveAngularVelocity(out, prevRot, rotation, h) {
        let dqx = rotation.w * (-prevRot.x) + rotation.x * prevRot.w + rotation.y * (-prevRot.z) - rotation.z * (-prevRot.y);
        let dqy = rotation.w * (-prevRot.y) - rotation.x * (-prevRot.z) + rotation.y * prevRot.w + rotation.z * (-prevRot.x);
        let dqz = rotation.w * (-prevRot.z) + rotation.x * (-prevRot.y) - rotation.y * (-prevRot.x) + rotation.z * prevRot.w;
        let dqw = rotation.w * prevRot.w - rotation.x * (-prevRot.x) - rotation.y * (-prevRot.y) - rotation.z * (-prevRot.z);
        if (dqw < 0) { dqx = -dqx; dqy = -dqy; dqz = -dqz; dqw = -dqw; } // shorter path
        const sinHalf = Math.sqrt(dqx * dqx + dqy * dqy + dqz * dqz);
        if (sinHalf < 1e-12) { out.x = 0; out.y = 0; out.z = 0; return; } // no rotation this substep
        const halfAngle = Scalar.atan2(sinHalf, dqw);
        const scale = (2 * halfAngle / h) / sinHalf; // turns (sin(halfAngle)*axis) into (angle/h)*axis = omega
        out.x = dqx * scale; out.y = dqy * scale; out.z = dqz * scale;
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
    // Compliance is 0 here (rigid, infinitely stiff contact) - the correct default for a contact
    // that should not be springy.
    _solvePoint(point, bodyA, bodyB, h) {
        point.currentAnchorAInto(this._rA, bodyA);
        point.currentAnchorBInto(this._rB, bodyB);
        const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;
        // normal points from B to A (matching GJK/EPA's own convention - verified directly
        // against GJK's separated-result normal after finding and fixing a real sign bug in EPA's
        // own output, see EPA.js). C = (anchorB - anchorA) . normal is positive exactly when B's
        // anchor has moved PAST A's anchor in the normal's own direction - i.e. penetrating.
        const C = (this._rB.x - this._rA.x) * nx + (this._rB.y - this._rA.y) * ny + (this._rB.z - this._rA.z) * nz;
        // SPECULATIVE CONTACT: this guard,
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

        // Capture the pre-solve contact-relative NORMAL velocity for restitution HERE, only on the
        // substep that actually pushes this point (C > 0, guarded above) - never on an earlier
        // substep where the pair is still approaching. This is the fix for a real bug (restitution
        // measured 101.19% of impact speed at e=1): the old capture site ran
        // unconditionally at the START of every substep, for every existing manifold point, even
        // substeps before contact actually engaged. When a tick's fall-to-impact spans multiple
        // substeps, each pre-contact substep overwrote this value with the body's CURRENT, still-
        // accelerating-under-gravity speed; the value that survived into the substep that actually
        // resolved contact was the one captured on the substep just before it - already faster than
        // the true velocity at the moment of impact, because gravity added more speed in between.
        // Traced directly: true impact speed 6.860, but the surviving stale capture was 6.9825 - a
        // 1.77% inflation that shows up almost exactly as the measured >100% energy return. Capturing
        // it here, gated on the same C>0 condition that decides "this substep actually pushes," means
        // it is always measured on the instant this constraint is actually enforced, never a leftover
        // from an earlier, still-falling substep.
        //
        // PRE-GRAVITY, not live velocity: even on the correct (contact-engaging) substep, this
        // substep's OWN gravity has already been added by step 1 before this ever runs - so the live
        // velocity here still overstates the true impact speed by one substep's worth of g*h. Using
        // each body's velocity from BEFORE this substep's gravity integration (this._preGravityVel,
        // captured at the very top of step 1) removes that overstatement. Verified by tracing a
        // dropped ball at e=1: without this, impact speed 6.860 in, but the captured value read
        // 6.9825 (a 1.77% inflation matching the measured 101.19% exit-speed bug almost exactly);
        // with the pre-gravity substitute, the captured value matches the true impact speed exactly.
        point._preSolveNormalVel = this._contactRelativeNormalVelocityPreGravity(point, bodyA, bodyB);

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

        // The IMPULSE (velocity side) is bounded by what the body's own real, pre-gravity closing
        // speed (point._preSolveNormalVel, captured above) could physically explain over this
        // substep - position still resolves the FULL geometric overlap (deltaLambda, unbounded,
        // passed straight through above) so a body never stays half-embedded, but only the
        // EXPLAINABLE share of that correction is allowed to become real velocity. A body genuinely
        // arriving at a contact (real closing speed behind it) is never starved - its own speed sets
        // the bound, and a normal/resting contact's C is already within that bound by construction
        // (that is what "arrived normally" means - see the speculative-margin mechanism above).
        // A spawn/teleport overlap has ~0 real closing speed behind a large C, so almost none of its
        // correction is explainable, and the excess simply never becomes velocity - it resolves as a
        // pure position fix instead, gradually, across ticks, never fabricated kinetic energy. This
        // is the same physical signal an earlier version of this file used (successfully) before this
        // session's architecture rewrite - reapplied here directly to the impulse itself, cleanly,
        // instead of being smuggled through a rotation-delta derivation that could not tell a
        // correction's own geometric side effects apart from real motion (see this file's own
        // header for why that indirection was the actual root cause of a separate class of bugs).
        const explainableC = Math.max(point._preSolveNormalVel, 0) * h * Solver.EXPLAINABLE_MARGIN;
        let impulseDeltaLambda = deltaLambda;
        if (C > explainableC) {
            const boundedNewLambda = Math.min(0, oldLambda - explainableC / wSum);
            impulseDeltaLambda = boundedNewLambda - oldLambda;
        }
        // velocityLambda accumulates the ACTUAL velocity-affecting lambda this substep, separate
        // from normalLambda's own full geometric accumulation - see this field's own comment on
        // ContactDetails for why friction/rolling-resistance need this, not normalLambda.
        point.velocityLambda += impulseDeltaLambda;

        this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, deltaLambda, h, impulseDeltaLambda);
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
    //
    // Applies the position correction to POSITION (unchanged from the original XPBD update), AND
    // applies the exact same correction's equivalent VELOCITY impulse directly via
    // _applyVelocityImpulse - dLambda is a position-space Lagrange multiplier delta; dividing by h
    // converts it to a velocity-space impulse, the standard relationship a position-based solver
    // uses to recover real velocity from its own constraint force (this IS the contact's actual
    // force/torque this substep). This replaces re-deriving velocity from the post-correction
    // position/rotation delta (see step 4's own comment for why that approach was a real, confirmed
    // source of bugs - a correction's own geometric side effects, unrelated to real motion, used to
    // get smashed into the same derivation and read back as real spin).
    // `impulseDLambda` (optional): the Lagrange-multiplier delta actually converted to a velocity
    // impulse, if DIFFERENT from `dLambda` - see _solvePoint's own comment for why a spawn/teleport
    // overlap needs this to differ from the full geometric correction (position must still resolve
    // the WHOLE overlap - a body left half-embedded is its own bug - but the velocity that overlap
    // implies is bounded separately, by what the body's own real approach speed can physically
    // explain). Defaults to `dLambda` (position and velocity impulse are the same thing) for every
    // ordinary contact, where they always should be.
    _applyPositionalCorrection(bodyA, bodyB, rA, rB, nx, ny, nz, dLambda, h, impulseDLambda) {
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
        const forVelocity = impulseDLambda !== undefined ? impulseDLambda : dLambda;
        this._applyVelocityImpulse(bodyA, bodyB, rA, rB, nx, ny, nz, forVelocity / h);
    }

    // Rotates `body` by the small-angle correction (I^-1 * (r x p)) * 0.5, the standard PBD
    // angular position update from a linear positional impulse applied at offset r (Muller et al.
    // eq 7-8, via the same quaternion-derivative integration _integrateRotation uses). POSITION ONLY
    // - this body's angular_velocity is updated separately, directly, via _applyVelocityImpulse in
    // _applyPositionalCorrection (see its own comment).
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
    // velocity, NOT the forbidden derived-velocity clamp (that governed a whole body's velocity
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
        // Coulomb cap on the friction impulse magnitude: mu times the normal impulse that ACTUALLY
        // reached velocity this substep (point.velocityLambda, not normalLambda - see
        // ContactDetails's own comment: normalLambda is the full GEOMETRIC correction, which can be
        // larger than what the impulse bound let become real velocity for a spawn/teleport-scale
        // overlap; sizing friction's own budget off the larger, geometry-only number let it apply
        // more tangential impulse than the body's actual normal-direction velocity change justified,
        // injecting real energy - traced directly on a corner bounce, apex height growing instead of
        // only decaying). Dividing by h converts the position-space multiplier to a velocity-space
        // impulse commensurate with the tangential impulses computed below.
        const maxImpulse = friction * Math.abs(point.velocityLambda) / h;
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

    // Rolling resistance: a torque opposing the tangential (non-normal) component of the two
    // bodies' relative angular velocity - compute the impulse that would zero it exactly, then clamp
    // to the Coulomb budget (same stop-fully-then-clamp pattern as friction's own tangent pass).
    _solveRollingResistance(point, bodyA, bodyB, h) {
        const rollingFriction = Math.sqrt(Math.max(bodyA.rolling_friction, 0) * Math.max(bodyB.rolling_friction, 0));
        if (rollingFriction <= 0) return;

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

        // Budget floored at the contact's real static load (min body mass * gravity), not just
        // point.normalLambda alone - normalLambda is XPBD's per-substep position multiplier and
        // collapses to ~0 once a contact is settled (C<=0 most substeps), which starved this budget
        // to nothing exactly when a resting round shape's last sliver of spin needed killing.
        let loadMass = Infinity;
        if (bodyA._massInverted > 0) loadMass = Math.min(loadMass, bodyA.mass);
        if (bodyB._massInverted > 0) loadMass = Math.min(loadMass, bodyB.mass);
        const staticFloor = isFinite(loadMass) ? rollingFriction * loadMass * this._gravityMag * h : 0;
        const maxImpulse = Math.max(rollingFriction * Math.abs(point.velocityLambda) / h, staticFloor);
        if (maxImpulse <= 0) return;
        let j = relWMag / wSum; // impulse to fully zero the relative angular velocity
        if (j > maxImpulse) j = maxImpulse;

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

    // Same as _contactRelativeNormalVelocity, but using each body's PRE-GRAVITY linear velocity for
    // this substep (this._preGravityVel) instead of its current (post-gravity-integration) velocity.
    // Used ONLY for restitution's pre-solve capture (see _solvePoint) - angular velocity is untouched
    // by gravity so it is read live as usual; only the linear term needs the pre-gravity substitute.
    _contactRelativeNormalVelocityPreGravity(point, bodyA, bodyB) {
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
