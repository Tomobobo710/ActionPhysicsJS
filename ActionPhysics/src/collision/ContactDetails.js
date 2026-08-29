/**
 * ContactDetails: one contact point between a specific pair of primitive shapes, in the sign
 * convention used across the whole narrowphase: signed distance NEGATIVE when
 * separated, POSITIVE when overlapping. GJK's separated result and EPA's overlapping result both
 * report a non-negative magnitude of their own (gap vs. depth) - normalizing the sign here is the
 * one place that distinction gets collapsed into a single number the rest of the pipeline can
 * treat uniformly (a manifold, a solver row, all just read `signedDistance`).
 *
 * pointOnA / pointOnB are the witness points on each shape's own surface (not the same point once
 * penetrating - that gap IS the depth). `point` is their midpoint, the conventional single contact
 * location a solver/manifold keys off; normal points from B to A, matching GJK/EPA's own
 * convention so no stage has to remember a sign flip.
 */
class ContactDetails {
    constructor() {
        this.point = new Vector3();
        this.pointOnA = new Vector3();
        this.pointOnB = new Vector3();
        this.normal = new Vector3();
        this.signedDistance = 0;
        // Set by the manifold once matched against a previous tick's point (warm-start data).
        // ContactDetails itself never reads or writes this; it exists here purely as a place to
        // carry the value across the manifold's point-matching step without a second parallel
        // array. Owned entirely by the solver once it exists (Rule 2: one owner per concern).
        this.normalLambda = 0;
        this.tangentLambda1 = 0;
        this.tangentLambda2 = 0;

        // Persistent VELOCITY-space accumulated impulses for this contact point, warm-started across
        // ticks by the manifold's point-matching (like normalLambda above, but for the velocity solve
        // rather than the position solve). These are the contact's own memory of the impulse it has
        // been sustaining: a box held under load carries a steady, nonzero normalImpulse tick after
        // tick, which is what lets "held under load" be told apart from "actually closing" as STATE
        // rather than re-inferred from an instantaneous relative-velocity heuristic every substep.
        // Distinct from normalLambda/tangentLambda (position-space XPBD multipliers) on purpose: this
        // is the foundation the velocity solve accumulates into. Owned entirely by the solver; carried
        // here only so the manifold can persist it across the point-matching step, same as the lambdas.
        this.normalImpulse = 0;
        this.frictionImpulse1 = 0;
        this.frictionImpulse2 = 0;

        // Body-LOCAL anchor offsets for pointOnA/pointOnB, set once by the manifold when this
        // point is created (ContactManifold._addPoint / update()'s new-point path) - NOT
        // recomputed by copy()/setFromGJKSeparated/setFromEPA, which only carry the world-space
        // geometry a fresh narrowphase result reports. The solver reads these every SUBSTEP to
        // recompute the contact's CURRENT gap from the bodies' current positions (see Solver.js's
        // class header and _solvePoint) - using the world-space pointOnA/pointOnB directly would
        // read a value frozen at the tick's single narrowphase pass, stale by the time later
        // substeps have already moved the bodies. This is the mechanism, not signedDistance, that
        // the solver actually corrects against.
        this.localAnchorA = new Vector3();
        this.localAnchorB = new Vector3();

        // Contact-relative normal velocity captured just before this substep's position solve (which
        // is about to zero it), so the velocity pass can apply restitution: bounce restores a
        // fraction of the speed the body was APPROACHING at, which is gone by the time the solve
        // finishes. Written each substep by the solver; not warm-start state.
        this._preSolveNormalVel = 0;

        // Live penetration depth captured by the position solve each substep, read by the velocity
        // solve as its Baumgarte push-out target so the normal VELOCITY solve carries depenetration.
        this._penetration = 0;
    }

    // Derives localAnchorA/localAnchorB from the CURRENT pointOnA/pointOnB and the given bodies'
    // CURRENT transforms. Called once, at the moment this point is created in a manifold (never
    // on a re-matched/refreshed point, which keeps its ORIGINAL anchors - that persistence across
    // ticks is what lets the solver see a growing gap as a body drifts, rather than the anchor
    // re-snapping to zero gap every tick).
    setLocalAnchors(bodyA, bodyB) {
        const invRotA = ContactDetails._scratchQuat.copy(bodyA.rotation).invert();
        Vector3.subInto(this.localAnchorA, this.pointOnA, bodyA.position);
        invRotA.transformVectorInPlace(this.localAnchorA);

        const invRotB = ContactDetails._scratchQuat.copy(bodyB.rotation).invert();
        Vector3.subInto(this.localAnchorB, this.pointOnB, bodyB.position);
        invRotB.transformVectorInPlace(this.localAnchorB);
        return this;
    }

    // Current world position of localAnchorA/B, written into out. Used by the solver every
    // substep to find each anchor's LIVE position without re-running narrowphase.
    currentAnchorAInto(out, bodyA) {
        out.copy(this.localAnchorA);
        bodyA.rotation.transformVectorInPlace(out);
        out.addInPlace(bodyA.position);
        return out;
    }

    currentAnchorBInto(out, bodyB) {
        out.copy(this.localAnchorB);
        bodyB.rotation.transformVectorInPlace(out);
        out.addInPlace(bodyB.position);
        return out;
    }

    // Fills this from a GJK separated result (`{distance, normal, pointA, pointB}`, distance is a
    // non-negative GAP). signedDistance becomes negative - separated, per the pipeline convention.
    setFromGJKSeparated(gjkResult) {
        this.pointOnA.copy(gjkResult.pointA);
        this.pointOnB.copy(gjkResult.pointB);
        this.normal.copy(gjkResult.normal);
        this.signedDistance = -gjkResult.distance;
        Vector3.addInto(this.point, gjkResult.pointA, gjkResult.pointB).scaleInPlace(0.5);
        return this;
    }

    // Fills this from an EPA result (`{distance, normal, pointA, pointB}`, distance is a
    // non-negative penetration DEPTH). signedDistance becomes positive - overlapping.
    setFromEPA(epaResult) {
        this.pointOnA.copy(epaResult.pointA);
        this.pointOnB.copy(epaResult.pointB);
        this.normal.copy(epaResult.normal);
        this.signedDistance = epaResult.distance;
        Vector3.addInto(this.point, epaResult.pointA, epaResult.pointB).scaleInPlace(0.5);
        return this;
    }

    copy(other) {
        this.point.copy(other.point);
        this.pointOnA.copy(other.pointOnA);
        this.pointOnB.copy(other.pointOnB);
        this.normal.copy(other.normal);
        this.signedDistance = other.signedDistance;
        this.normalLambda = other.normalLambda;
        this.tangentLambda1 = other.tangentLambda1;
        this.tangentLambda2 = other.tangentLambda2;
        return this;
    }

    clone() {
        return new ContactDetails().copy(this);
    }
}

ContactDetails._scratchQuat = new Quaternion();

ActionPhysics.ContactDetails = ContactDetails;
