# ActionPhysics

A single-file, dependency-free JavaScript physics engine. Ships as a standalone library with no external dependencies — just build it and load the bundle.

## What It Does

Rigid-body simulation with:

- **12 shape types**: Box, Sphere, Cylinder, Cone, Capsule, Convex (point cloud), Plane, Triangle, Mesh (vertex+index), Compound, LineSwept, and a Shape base class
- **GJK/EPA collision detection**: Gilbert–Johnson–Keerthi distance algorithm with Expanding Polytope Algorithm for penetration depth and contact normals
- **Contact manifolds**: Multi-point contacts with warm-start, per-tick lifetime management, and 4-point reduction
- **XPBD solver**: Position-based dynamics with speculative contacts (prevents tunneling), per-substep geometry refresh, Coulomb friction (static + dynamic), restitution, and angular/rolling friction
- **Broadphase & spatial indexing**: Sweep-and-prune (SAP) broadphase + Bounding Volume Hierarchy for efficient pair culling
- **Closed-form midphase**: Dedicated SAT-based tests for sphere-sphere, sphere-box, and box-box pairs (15-axis SAT + Sutherland-Hodgman clipping)
- **Constraints**: Point, Hinge, Weld, and Slider constraints with position-level XPBD solving
- **Queries**: Ray intersection and shape overlap via GJK — no separate ray-vs-shape algorithm needed
- **Character controller**: Spring-based capsule controller with slope projection, jumping, and state management
- **FPS character controller**: Full first-person controller with movement, ladder climbing, mantling, step-up, air control, ghost collision, and view handling
- **Island detection & sleeping**: Union-find over coupled bodies; resting islands sleep automatically to save compute
- **Contact events**: Per-tick `contacts` event from the world for observing real contact manifolds

## Project Structure

```
src/
  math/              — ActionMath: Scalar, Vector2/3, Matrix3/4, Quaternion, Transform
  shapes/            — 12 shape primitives with support functions, AABBs, mass properties
  spatial/           — AABB and BVH data structures
  bodies/            — RigidBody with full field layout (transform, mass, motion, forces, materials)
  phases/            — Broadphase (SAP), midphase (BVH dispatch + closed-form tests), narrowphase (GJK/EPA/contacts)
  collision/         — GJK, EPA, contact details/manifolds, simplex management
  solver/            — Integration, position solve, velocity solve, island manager
  constraints/       — Point, Hinge, Weld, Slider constraint implementations
  queries/           — Ray and shape intersection via GJK
  character/         — Spring-based capsule controller + FPS controller with movement submodules
  world/             — Top-level physics world (step(), add/remove bodies, contact events)

tests/
  js/suites/         — Unit and integration test suites
  js/tom/            — Scene-level tests (FPS scenarios, menagerie, knockback, pyramid stacking, etc.)
  lib/               — Three.js for visual test bench

build/               — Bundled output (actionphysics.js)
actionmath-source/   — Shared math library source (pasted 1:1 into engine build)
```

## Build

```bash
node build.js
```

Synchronous, explicit ordered manifest (`sources.js`). Fails on an unlisted source file. Determinism gate blocks any `Math.*` calls outside the allowlist (`sqrt`, `abs`, `min`, `max`, `floor`, `ceil`, `round`, `sign`, `trunc`, plus constants). Output is stamped with a build timestamp.

## Tests

```bash
# Headless (CI)
node tests/run_headless.js

# Visual bench (browser)
open tests/suite.html
```

Tests covering math primitives, shapes, collision detection, solver correctness, constraints, queries, character controller scenarios, and physics-correctness.

## Units & Conventions

- All shape parameters use **half-extents** (matching the AABB/math convention), except Capsule which takes total height (a capsule's height already includes its caps).
- Physics simulation uses **Float64Array** internally for precision (~15 significant digits) — Float32 is too coarse for contact-depth resolution.
- The math library is **injectable**: if a host environment provides `ActionMath`, the engine adopts it (one class per type, `instanceof` works across boundaries). Otherwise it bundles its own copy.

## Determinism

All transcendental functions route through a custom `Scalar` module rather than `Math.*`. The JS spec leaves `sin`, `cos`, `atan2`, `hypot`, and `pow` implementation-defined; arithmetic and `sqrt` are not. This eliminates platform-dependent variance. Accuracy vs `Math.*`: sin/cos ~3.3e-16, atan ~2.2e-16. Symmetry is bit-exact.

## License

MIT — see LICENSE.
