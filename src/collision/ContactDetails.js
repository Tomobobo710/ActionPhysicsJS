/**
 * ContactDetails: one contact point between a specific pair of primitive shapes, in the sign
 * convention plan.md establishes for the whole narrowphase: signed distance NEGATIVE when
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
        // Set by the manifold once matched against a previous tick's point (warm-start data -
        // see plan.md's component list: "ContactManifold (4-point cap, dedup, warm-start data)").
        // ContactDetails itself never reads or writes this; it exists here purely as a place to
        // carry the value across the manifold's point-matching step without a second parallel
        // array. Owned entirely by the solver once it exists (Rule 2: one owner per concern).
        this.normalLambda = 0;
        this.tangentLambda1 = 0;
        this.tangentLambda2 = 0;
    }

    // Fills this from a GJK separated result (`{distance, normal, pointA, pointB}`, distance is a
    // non-negative GAP). signedDistance becomes negative - separated, per plan.md's convention.
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

ActionPhysics.ContactDetails = ContactDetails;
