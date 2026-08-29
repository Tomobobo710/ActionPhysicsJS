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

    // Bodies - RigidBody references AABB, Matrix3, and the shape contract above. RigidBody.js is
    // the class shell; the phases attach onto RigidBody.prototype from their own files.
    'src/bodies/RigidBody.js',
    'src/bodies/Forces.js',
    'src/bodies/DerivedState.js',
    'src/bodies/Accessors.js',

    // Spatial (part 2) - BVH is independent of RigidBody but grouped with phases since midphase
    // is its main consumer.
    'src/spatial/BVH.js',

    // Phases - broadphase needs RigidBody (reads .getAABB(), .bodyType, .id, .collision_mask/groups).
    'src/phases/SAPBroadphase.js',

    // Midphase needs BVH, RigidBody, CompoundShape, MeshShape, TriangleShape - all already listed
    // above. Midphase.js is the class shell; the phases attach onto the prototype from their own files.
    'src/phases/Midphase.js',
    'src/phases/BVHCache.js',
    'src/phases/ExpandPair.js',

    // Narrowphase (collision detection): MinkowskiSupport wraps two placed shapes; GJK consumes it.
    // GJK.js/EPA.js are class shells; their phases attach onto the prototype from their own files.
    'src/collision/MinkowskiSupport.js',
    'src/collision/GJK.js',
    'src/collision/Seeding.js',
    'src/collision/Simplex.js',
    'src/collision/Run.js',
    'src/collision/EPA.js',
    'src/collision/InitialTetrahedron.js',
    'src/collision/Expand.js',

    // Contacts: ContactDetails normalizes GJK/EPA output; ContactManifold owns point lifetime
    // (class shell + Update.js + Reduction.js); ContactManifoldList owns the set of active manifolds.
    'src/collision/ContactDetails.js',
    'src/collision/ContactManifold.js',
    'src/collision/Update.js',
    'src/collision/Reduction.js',
    'src/collision/ContactManifoldList.js',

    // Narrowphase dispatch - ties Midphase + GJK/EPA + ContactManifoldList together. NarrowPhase.js
    // is the class shell; the phases attach onto NarrowPhase.prototype from their own files.
    // Closed-form pair tests (SphereSphere, SphereBox, BoxBox) load before PairTest.js, which
    // dispatches to them ahead of GJK/EPA - own numerics, no shared epsilon/iteration budget with
    // any other pair.
    'src/phases/NarrowPhase.js',
    'src/phases/SphereSphere.js',
    'src/phases/SphereBox.js',
    'src/phases/BoxBox.js',
    'src/phases/TriTri.js',
    'src/phases/ConvexTri.js',
    'src/phases/PairTest.js',
    'src/phases/SpeculativeMargin.js',
    'src/phases/GeometryRefresh.js',

    // Solver - XPBD. Solver.js is the class shell + orchestration; the phases attach onto
    // Solver.prototype from their own files, same pattern as the FPS controller below.
    'src/solver/Solver.js',
    'src/solver/Integrate.js',
    'src/solver/PositionSolve.js',
    'src/solver/VelocitySolve.js',
    // Island-based sleeping. WIP: the module is complete but not yet called from the world step,
    // so tom/sleep.js fails until a call site lands. Listed here so the build doesn't reject it.
    'src/solver/IslandManager.js',

    // Constraints (joints) - position-level XPBD constraints, built on the solver's own
    // Solver._integrateRotation and each body's mass/inertia fields. Loaded after Solver.
    'src/constraints/Constraint.js',
    'src/constraints/PointConstraint.js',
    'src/constraints/HingeConstraint.js',
    'src/constraints/WeldConstraint.js',
    'src/constraints/SliderConstraint.js',

    // Queries - ray/shape casts, conservative advancement over GJK. Depends on GJK, MinkowskiSupport,
    // SphereShape (used as a zero-radius point), AABB. Loaded after those, before World (World's
    // rayIntersect/shapeIntersect delegate here). Queries.js is the class shell + shared scratch;
    // the phases attach onto Queries (a static-only class) from their own files.
    'src/queries/Queries.js',
    'src/queries/Advance.js',
    'src/queries/RayIntersect.js',
    'src/queries/ShapeIntersect.js',

    // World - pipeline glue.
    'src/world/World.js',

    // Character controllers - use World/RigidBody's privileged interface (raw force/velocity
    // access), loaded last since they depend on everything above.
    'src/character/CharacterController.js',

    // FPS character controller - a kinematic box mover with its own ground/wall/slope/ghost
    // handling, excluded from the solver's own contact resolution entirely (see Body.js). Split
    // across many files, each adding methods onto the same FPSCharacterController.prototype -
    // FPSControllerConstants.js (tunable defaults) and FPSCharacterController.js (the constructor)
    // must load first; Constants.js attaches FPSC/raycast statics onto the constructor function so
    // it must load after that but before every file below, which all read FPSC.
    'src/character/fps/FPSControllerConstants.js',
    'src/character/fps/FPSCharacterController.js',
    'src/character/fps/Constants.js',
    'src/character/fps/Body.js',
    'src/character/fps/Ghost.js',
    'src/character/fps/Collision.js',
    'src/character/fps/Probes.js',
    'src/character/fps/View.js',
    'src/character/fps/Netcode.js',
    'src/character/fps/Movement/Airborne.js',
    'src/character/fps/Movement/Vertical.js',
    'src/character/fps/Movement/Step.js',
    'src/character/fps/Movement/Slide.js',
    'src/character/fps/Movement/Ladder.js',
    'src/character/fps/Movement/Mantle.js',

    'src/outro.js'
];
