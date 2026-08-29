// One contact point between a primitive shape pair. signedDistance: negative = separated,
// positive = overlapping. normal points B to A. pointOnA/pointOnB are witness points on each
// shape's surface; `point` is their midpoint.
class ContactDetails {
    constructor() {
        this.point = new Vector3();
        this.pointOnA = new Vector3();
        this.pointOnB = new Vector3();
        this.normal = new Vector3();
        this.signedDistance = 0;
        // Warm-start data, set by the manifold on match.
        this.normalLambda = 0;
        this.tangentLambda1 = 0;
        this.tangentLambda2 = 0;

        // Body-local anchors, set once at point creation, re-read every substep to recompute the
        // live gap (see Solver's PositionSolve.js) - not recomputed on refresh/copy.
        this.localAnchorA = new Vector3();
        this.localAnchorB = new Vector3();

        // Contact-relative normal velocity just before this substep's position solve, for
        // restitution. Written each substep by the solver.
        this._preSolveNormalVel = 0;

        // True when this point came from TriTri's mesh face-clip (see TriTri.js). Lets the manifold
        // apply a coincidence merge that only makes sense for the many-triangle mesh case.
        this.fromMeshFace = false;
    }

    // Derives local anchors from current pointOnA/pointOnB + body transforms. Called once, at
    // point creation - never on a re-matched point (which keeps its original anchors).
    setLocalAnchors(bodyA, bodyB) {
        const invRotA = ContactDetails._scratchQuat.copy(bodyA.rotation).invert();
        Vector3.subInto(this.localAnchorA, this.pointOnA, bodyA.position);
        invRotA.transformVectorInPlace(this.localAnchorA);

        const invRotB = ContactDetails._scratchQuat.copy(bodyB.rotation).invert();
        Vector3.subInto(this.localAnchorB, this.pointOnB, bodyB.position);
        invRotB.transformVectorInPlace(this.localAnchorB);
        return this;
    }

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

    // GJK separated result (distance = non-negative gap) -> negative signedDistance.
    setFromGJKSeparated(gjkResult) {
        this.fromMeshFace = false;
        this.pointOnA.copy(gjkResult.pointA);
        this.pointOnB.copy(gjkResult.pointB);
        this.normal.copy(gjkResult.normal);
        this.signedDistance = -gjkResult.distance;
        Vector3.addInto(this.point, gjkResult.pointA, gjkResult.pointB).scaleInPlace(0.5);
        return this;
    }

    // EPA result (distance = non-negative depth) -> positive signedDistance.
    setFromEPA(epaResult) {
        this.fromMeshFace = false;
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
        this.fromMeshFace = other.fromMeshFace;
        return this;
    }

    clone() {
        return new ContactDetails().copy(this);
    }
}

ContactDetails._scratchQuat = new Quaternion();

ActionPhysics.ContactDetails = ContactDetails;
