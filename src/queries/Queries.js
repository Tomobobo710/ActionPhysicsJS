// Ray casts and shape sweeps against world.bodies, via GJK's closest-distance result (a ray is a
// zero-radius sphere). O(n) over the body list after a cheap AABB reject. See RayIntersect.js,
// ShapeIntersect.js, Advance.js.
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

    // World-space placement of one mesh triangle onto a cached scratch TriangleShape (no per-
    // triangle allocation). MeshShape.triangleAt already hands back body-local vertices.
    static _placedTriangleInto(outPlaced, body, triShape, a, b, c) {
        triShape.a = a; triShape.b = b; triShape.c = c;
        outPlaced.shape = triShape;
        outPlaced.position = body.position;
        outPlaced.rotation = body.rotation;
        return outPlaced;
    }

    // World-space placement of one compound child, matching Midphase's own convention: world
    // position = bodyPos + bodyRot * childLocalPos; world rotation = bodyRot * childLocalRot.
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
Queries._scratchPointShape = new SphereShape(0); // zero-radius sphere: a point, via the existing Shape contract
Queries._scratchLocalAABB = new AABB();
Queries._scratchExpandedAABB = new AABB();
Queries._scratchCompoundChild = { shape: null, position: new Vector3(), rotation: new Quaternion(0, 0, 0, 1) };
Queries._scratchTriangleShape = new TriangleShape(new Vector3(), new Vector3(), new Vector3());

ActionPhysics.Queries = Queries;
