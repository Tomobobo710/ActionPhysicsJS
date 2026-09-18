class Queries {
    static _isIgnored(body, ignore) {
        if (!ignore) return false;
        if (Array.isArray(ignore)) return ignore.indexOf(body) !== -1;
        return body === ignore;
    }

    static _isCompound(shape) {
        return typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape;
    }

    static _isMesh(shape) {
        return typeof MeshShape !== 'undefined' && shape instanceof MeshShape;
    }

    static _placedTriangleInto(outPlaced, body, triShape, a, b, c) {
        triShape.a = a; triShape.b = b; triShape.c = c;
        outPlaced.shape = triShape;

        outPlaced.position.copy(body.position);
        outPlaced.rotation.copy(body.rotation);
        return outPlaced;
    }

    static _placedChildInto(outPlaced, body, child) {
        outPlaced.shape = child.shape;
        outPlaced.rotation.multiplyQuaternions(body.rotation, child.localRotation);
        outPlaced.position.copy(child.localPosition);
        body.rotation.transformVectorInPlace(outPlaced.position);
        outPlaced.position.addInPlace(body.position);
        return outPlaced;
    }
}

Queries._gjk = new GJK();
Queries._epa = new EPA();
Queries._identityQuat = new Quaternion(0, 0, 0, 1);
Queries._scratchPos = new Vector3();
Queries._scratchPlacedA = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchPlacedB = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchSupport = new MinkowskiSupport(Queries._scratchPlacedA, Queries._scratchPlacedB);
Queries._scratchPointShape = new SphereShape(0);
Queries._scratchLocalAABB = new AABB();
Queries._scratchExpandedAABB = new AABB();
Queries._scratchCompoundChild = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };

Queries._scratchPlacedBPos = new Vector3();
Queries._scratchTriangleShape = new TriangleShape(new Vector3(), new Vector3(), new Vector3());

Queries._scratchInvRot = new Quaternion(0, 0, 0, 1);
Queries._scratchCorner = new Vector3();
Queries._scratchLeafList = [];
Queries._scratchTriA = new Vector3();
Queries._scratchTriB = new Vector3();
Queries._scratchTriC = new Vector3();

ActionPhysics.Queries = Queries;
