class ContactDetails {
    constructor() {
        this.point = new Vector3();
        this.pointOnA = new Vector3();
        this.pointOnB = new Vector3();
        this.normal = new Vector3();
        this.signedDistance = 0;
        this.normalLambda = 0;
        this.tangentLambda1 = 0;
        this.tangentLambda2 = 0;

        this.localAnchorA = new Vector3();
        this.localAnchorB = new Vector3();

        this._preSolveNormalVel = 0;

        this.prevAnchorA = new Vector3();
        this.prevAnchorB = new Vector3();
        this.fricLocalA = new Vector3();
        this.fricLocalB = new Vector3();
        this.prevAnchorValid = false;
        this.fromMeshFace = false;
        this.edgeAxis = null;
        this.fromBoxBox = false;
        this.fromFacePatch = false;
        // Set by ConvexTri: a curved shape's probe cloud is NOT a clipped patch, so its point extent
        // does not describe the body's real footprint (see Solver._solveManifold's support test).
        this.fromCurvedTri = false;

        this.meshTriValid = false;
        this.meshTriA = new Vector3();
        this.meshTriB = new Vector3();
        this.meshTriC = new Vector3();
        this.meshTriBodyCenter = new Vector3();
        this.meshTriIsSideA = false;
    }

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

    setFromGJKSeparated(gjkResult) {
        this.fromMeshFace = false;
        this.pointOnA.copy(gjkResult.pointA);
        this.pointOnB.copy(gjkResult.pointB);
        this.normal.copy(gjkResult.normal);
        this.signedDistance = -gjkResult.distance;
        Vector3.addInto(this.point, gjkResult.pointA, gjkResult.pointB).scaleInPlace(0.5);
        return this;
    }

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
        this.prevAnchorA.copy(other.prevAnchorA);
        this.prevAnchorB.copy(other.prevAnchorB);
        this.fricLocalA.copy(other.fricLocalA);
        this.fricLocalB.copy(other.fricLocalB);
        this.prevAnchorValid = other.prevAnchorValid;
        this.tangentLambda1 = other.tangentLambda1;
        this.tangentLambda2 = other.tangentLambda2;
        this.fromMeshFace = other.fromMeshFace;
        this.fromBoxBox = other.fromBoxBox;
        this.fromFacePatch = other.fromFacePatch;
        this.fromCurvedTri = other.fromCurvedTri;
        if (other.edgeAxis) {
            if (!this.edgeAxis) this.edgeAxis = new Vector3();
            this.edgeAxis.copy(other.edgeAxis);
        } else {
            this.edgeAxis = null;
        }
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
