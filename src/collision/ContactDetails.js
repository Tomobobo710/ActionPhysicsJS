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
        this.normalLambda = 0;   // warm-start data, preserved across a match
        this.tangentLambda1 = 0;
        this.tangentLambda2 = 0;

        // Set once at creation, re-read each substep for the live gap (PositionSolve.js).
        this.localAnchorA = new Vector3();
        this.localAnchorB = new Vector3();

        this._preSolveNormalVel = 0; // for restitution, written each substep
        this.fromMeshFace = false;   // set by TriTri/ConvexTri; gates the mesh-face merge and patch solve

        // Source triangle for a mesh-face contact, in world space (the mesh side is static ground,
        // so these verts don't move within a tick). Lets GeometryRefresh re-clip only the triangle
        // that produced this point each substep instead of re-running the whole midphase +
        // narrowphase for the pair. Set by TriTri/ConvexTri alongside fromMeshFace; meshTriValid
        // stays false when unset so the refresh can fall back.
        this.meshTriValid = false;
        this.meshTriA = new Vector3();
        this.meshTriB = new Vector3();
        this.meshTriC = new Vector3();
        this.meshTriBodyCenter = new Vector3();
        this.meshTriIsSideA = false; // was the triangle placedA (true) or placedB (false) in the pair
    }

    // Derives local anchors from the current witness points. Called once at creation, never on a
    // re-matched point.
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
        this.meshTriValid = other.meshTriValid;
        if (other.meshTriValid) {
            this.meshTriA.copy(other.meshTriA);
            this.meshTriB.copy(other.meshTriB);
            this.meshTriC.copy(other.meshTriC);
            this.meshTriBodyCenter.copy(other.meshTriBodyCenter);
            this.meshTriIsSideA = other.meshTriIsSideA;
        }
        return this;
    }

    // Records the world-space source triangle for a mesh-face contact (see the field comments).
    setMeshTriangle(a, b, c, bodyCenter, isSideA) {
        this.meshTriValid = true;
        this.meshTriA.copy(a);
        this.meshTriB.copy(b);
        this.meshTriC.copy(c);
        if (bodyCenter) this.meshTriBodyCenter.copy(bodyCenter); else this.meshTriBodyCenter.set(0, 0, 0);
        this.meshTriIsSideA = isSideA;
        return this;
    }

    clone() {
        return new ContactDetails().copy(this);
    }
}

ContactDetails._scratchQuat = new Quaternion();

ActionPhysics.ContactDetails = ContactDetails;
