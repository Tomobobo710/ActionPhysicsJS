/**
 * Ordered source manifest.
 *
 * Explicit and ordered — no globs. Dependency order is a design decision, so it is stated
 * here rather than inferred from directory traversal - glob ordering is not a dependency graph.
 *
 * build.js fails if any .js under src/ is missing from this list, so a new file that is
 * never concatenated is caught at build time instead of as a confusing runtime error.
 */
module.exports = [
    'src/intro.js',

    // ActionMath - pasted verbatim from the shared library, never edited locally.
    // Scalar first: the others call it.
    'src/math/Scalar.js',
    'src/math/Vector2.js',
    'src/math/Vector3.js',
    'src/math/Matrix3.js',
    'src/math/Matrix4.js',
    'src/math/Quaternion.js',
    'src/math/Transform.js',

    // Spatial - AABB has no dependency beyond Vector3.
    'src/spatial/AABB.js',

    // Shapes - Shape base first, then concrete shapes. ConvexShape/CompoundShape reference
    // AABB; CompoundShape references Matrix3.
    'src/shapes/Shape.js',
    'src/shapes/BoxShape.js',
    'src/shapes/SphereShape.js',
    'src/shapes/CylinderShape.js',
    'src/shapes/ConeShape.js',
    'src/shapes/CapsuleShape.js',
    'src/shapes/ConvexShape.js',
    'src/shapes/PlaneShape.js',
    'src/shapes/TriangleShape.js',
    'src/shapes/MeshShape.js',
    'src/shapes/CompoundShape.js',
    'src/shapes/LineSweptShape.js',

    // Bodies - RigidBody references AABB, Matrix3, and the shape contract above.
    'src/bodies/RigidBody.js',

    // Spatial (part 2) - BVH is independent of RigidBody but grouped with phases since midphase
    // is its main consumer.
    'src/spatial/BVH.js',

    // Phases - broadphase needs RigidBody (reads .getAABB(), .bodyType, .id, .collision_mask/groups).
    'src/phases/SAPBroadphase.js',

    'src/outro.js'
];
