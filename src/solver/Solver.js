// XPBD solver (Muller et al. 2020). Velocity is derived from position (v = (x - x_prev) / h).
// Per substep: integrate -> refresh contact geometry -> reset lambdas -> solve positions ->
// derive velocity -> solve contact velocity.
class Solver {
    constructor(opts) {
        opts = opts || {};
        this.substeps = opts.substeps || 4;
        this.iterations = opts.iterations || 4; // position-solve passes per substep

        this._rA = new Vector3(); this._rB = new Vector3();
        this._deltaPos = new Vector3();
        this._impulse = new Vector3();
        this._tangent1 = new Vector3(); this._tangent2 = new Vector3();
        this._angularCorrA = new Vector3(); this._angularCorrB = new Vector3();
        this._tmpDispA = new Vector3(); this._tmpDispB = new Vector3(); this._tmpPrev = new Vector3();
        this._prevPos = new Map();
        this._prevRot = new Map();
        this._preGravityVel = new Map();
        this._biasDelta = new Map(); // per-body bias-only correction this substep; excluded from derived velocity
        this._biasAng = new Map();
        this._restRing = new Map(); // per-body ring buffer of recent transforms for rest-velocity reconciliation

        // Horizontal extent of the manifold being solved; filled by _supportBounds for a closed-form
        // box-box patch.
        this._supMinX = 0; this._supMaxX = 0; this._supMinZ = 0; this._supMaxZ = 0;
        this._checkSupport = false;
        this._supportCentreTol = SUPPORT_CENTRE_TOL;
        // Per-substep union of every upward patch each body stands on, keyed by body id.
        this._supportBoundsByBody = new Map();
        this._supportGen = 0;

        // per-tick scratch for _coplanarPatchGroups
        this._patchGroupsByBody = new Map();
        this._patchGroups = new Map();

        // scratch for the contact-region support probe (see _widenSupportWithProbe)
        this._probeNormal = new Vector3();
        this._probeT1 = new Vector3();
        this._probeT2 = new Vector3();
        this._probeDir = new Vector3();
        this._probeOut = new Vector3();
        this._probeLocal = new Vector3();
        this._probeInvRot = new Quaternion();
    }

    // Widens a body's support entry with the body's OWN contact region, for contact points that carry no
    // patch geometry of their own - a bare GJK/EPA witness. One witness is a single sample of the region,
    // not the region: read alone it leaves a shape balanced on a rim, or tips one whose witness sits
    // off-centre. Asking the SHAPE settles both, by probing its support in a ring around the contact and
    // keeping the samples that come back at contact depth.
    _widenSupportWithProbe(body, isA, nx, ny, nz, px, py, pz, sd, e) {
        const shape = body.shape;
        // A sphere's witness already IS its contact region. A compound dispatches per child and a mesh
        // is not convex, so neither has a support function to ask.
        if (shape instanceof SphereShape || shape instanceof CompoundShape || shape instanceof MeshShape) return;
        if (typeof shape.supportInto !== 'function') return;

        // The direction from this body INTO the contact. The normal is stored B-relative, pointing B->A.
        const dx = isA ? -nx : nx, dy = isA ? -ny : ny, dz = isA ? -nz : nz;
        this._probeNormal.set(dx, dy, dz);
        // The tangent basis decides WHICH points of the region get sampled, and an arbitrary
        // perpendicular pair misses the barrel case: a cylinder on its side touches along a LINE, and
        // only sampling towards the shape's own axis reaches that line's ends, because the support in
        // an axially tilted direction lands on the cap end at contact depth, rise zero. An upright
        // barrel keeps the usual basis, and its flat cap - whose samples are coplanar - widens on its own.
        Solver._tangentBasis(this._probeNormal, this._probeT1, this._probeT2);
        const t1 = this._probeT1, t2 = this._probeT2, dir = this._probeDir, out = this._probeOut;
        const inv = this._probeInvRot.copy(body.rotation).invert();

        // How far off the contact plane a sample may sit and still count as part of the body's contact
        // region. An absolute CONTACT tolerance, not a share of the body: scaled to the body it admits
        // samples well clear of the plane and fabricates a support region wide enough to swallow a
        // centre of mass that is genuinely off the contact - a false equilibrium.
        const band = Solver.PROBE_DEPTH_BAND;

        for (let i = 0; i < Solver.PROBE_COUNT; i++) {
            const u = Solver._PROBE_U[i], v = Solver._PROBE_V[i];
            dir.set(dx + t1.x * u + t2.x * v, dy + t1.y * u + t2.y * v, dz + t1.z * u + t2.z * v);
            const len = Math.sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) || 1;
            dir.scaleInPlace(1 / len);
            MinkowskiSupport.supportOfInto(out, body, inv, dir, this._probeLocal);
            // Height above the CONTACT PLANE, not above the witness: the witness is reported at the
            // penetration depth, so judging against it rejects a barrel's own support line for exactly
            // the reason, and by the same R*(1-cos) amount, as a tilted cap's shoulder. Adding the
            // contact's signedDistance moves the reference onto the surface the two bodies actually
            // meet at, which separates them at any radius.
            const proj = (out.x - px) * dx + (out.y - py) * dy + (out.z - pz) * dz + sd;
            if (proj < -band) continue;
            if (out.x < e.minX) e.minX = out.x;
            if (out.x > e.maxX) e.maxX = out.x;
            if (out.z < e.minZ) e.minZ = out.z;
            if (out.z > e.maxZ) e.maxZ = out.z;
        }
    }

    // Records the horizontal extent of `manifold`'s contact patch so the support test can tell a patch
    // the body is sitting ON from one it hangs off: a centre of mass inside the extent is supported
    // from below, while a centre that has passed it is an overhang.
    _supportBounds(bodyA, bodyB, manifold, n) {
        // The support under a body is the UNION of every upward patch it stands on this substep, not
        // whichever manifold happens to be solved: a cone dropped on a tile seam has a patch either side
        // of it and its centre of mass outside either alone, so judged manifold by manifold both read as
        // overhangs and topple it off a surface it is resting flat on.
        const union = this._supportBoundsForBody(bodyA, bodyB);
        if (union) {
            this._supMinX = union.minX; this._supMaxX = union.maxX;
            this._supMinZ = union.minZ; this._supMaxZ = union.maxZ;
            this._supportCentreTol = union.curved ? CURVED_SUPPORT_CENTRE_TOL : SUPPORT_CENTRE_TOL;
            return;
        }
        let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
        let curved = false;
        for (let i = 0; i < n; i++) {
            const p = manifold.points[i];
            if (p.fromCurvedTri) curved = true;
            p.currentAnchorAInto(this._tmpDispA, bodyA);
            p.currentAnchorBInto(this._tmpDispB, bodyB);
            const x = (this._tmpDispA.x + this._tmpDispB.x) * 0.5;
            const z = (this._tmpDispA.z + this._tmpDispB.z) * 0.5;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
        }
        if (minX === Infinity) { minX = maxX = minZ = maxZ = 0; }
        this._supMinX = minX; this._supMaxX = maxX;
        this._supMinZ = minZ; this._supMaxZ = maxZ;
        this._supportCentreTol = curved ? CURVED_SUPPORT_CENTRE_TOL : SUPPORT_CENTRE_TOL;
    }

    // Builds the per-body support union for one substep, keyed by body id. Only upward contacts that
    // actually touch count - a speculative contact carries no load yet - but every point of a patch
    // does, including the separated probe samples that describe a curved region's width. `gen` retires
    // last substep's entries without reallocating.
    _collectSupportBounds(manifolds) {
        const gen = ++this._supportGen;
        const byBody = this._supportBoundsByBody;
        for (const m of manifolds.values()) {
            const a = m.bodyA, b = m.bodyB;
            const aFree = a.bodyType === RigidBody.DYNAMIC && a.isAwake;
            const bFree = b.bodyType === RigidBody.DYNAMIC && b.isAwake;
            if (!aFree && !bFree) continue;
            for (let i = 0; i < m.points.length; i++) {
                const p = m.points[i];
                // A separated point of a real patch carries no load and does not widen the support. A
                // ConvexTri probe sample is different: it samples the SHAPE, so its separated samples
                // are exactly what describe how wide a curved contact region is. Dropping them narrows
                // a capsule's support to noise and lets a settled one be judged overhanging.
                if (p.signedDistance < -REST_TOUCH_BAND && !p.fromCurvedTri) continue;
                const ny = p.normal.y;
                if (ny > -0.98 && ny < 0.98) continue;
                // The normal points B->A, so the body it holds up is the one it points away from.
                const body = ny > 0 ? a : b;
                if (body.bodyType !== RigidBody.DYNAMIC || !body.isAwake) continue;
                p.currentAnchorAInto(this._tmpDispA, a);
                p.currentAnchorBInto(this._tmpDispB, b);
                const x = (this._tmpDispA.x + this._tmpDispB.x) * 0.5;
                const z = (this._tmpDispA.z + this._tmpDispB.z) * 0.5;
                let e = byBody.get(body.id);
                if (!e) { byBody.set(body.id, e = { gen: 0, minX: 0, maxX: 0, minZ: 0, maxZ: 0, curved: false }); }
                if (e.gen !== gen) {
                    e.gen = gen;
                    e.minX = x; e.maxX = x; e.minZ = z; e.maxZ = z;
                    e.curved = !!p.fromCurvedTri;
                } else {
                    if (x < e.minX) e.minX = x; else if (x > e.maxX) e.maxX = x;
                    if (z < e.minZ) e.minZ = z; else if (z > e.maxZ) e.maxZ = z;
                    if (p.fromCurvedTri) e.curved = true;
                }
                // A point with no patch geometry of its own describes no region by itself, so ask the
                // shape where its contact region is. Without this the support test only ever sees the
                // one witness position, which is how a cone ends up parked on its base rim.
                if (!p.fromBoxBox && !p.fromFacePatch && !p.fromMeshFace && !p.fromCurvedTri) {
                    const isA = body === a;
                    this._widenSupportWithProbe(body, isA, p.normal.x, p.normal.y, p.normal.z,
                        isA ? this._tmpDispA.x : this._tmpDispB.x,
                        isA ? this._tmpDispA.y : this._tmpDispB.y,
                        isA ? this._tmpDispA.z : this._tmpDispB.z, p.signedDistance, e);
                    if (!isFlatFaced(body.shape)) e.curved = true;
                }
            }
        }
    }

    _supportBoundsForBody(bodyA, bodyB) {
        const gen = this._supportGen;
        const a = this._supportBoundsByBody.get(bodyA.id);
        if (a && a.gen === gen) return a;
        const b = this._supportBoundsByBody.get(bodyB.id);
        if (b && b.gen === gen) return b;
        return null;
    }

    // Widens what counts as "explainable by the body's own velocity" in _solvePoint's
    // position/velocity split (PositionSolve.js).
    static EXPLAINABLE_MARGIN = 3;

    // Advances dynamic bodies by dt, resolving manifolds and constraints. `refresh(manifolds)`, if
    // given, re-measures contact geometry each substep before the solve.
    step(bodies, manifolds, gravity, dt, refresh, constraints) {
        const h = dt / this.substeps;
        for (let s = 0; s < this.substeps; s++) {
            this._substep(bodies, manifolds, gravity, h, refresh, constraints);
        }
        this._reconcileRestVelocity(bodies, dt);
    }

    // Zeroes the velocity of a body whose sustained motion over the last REST_WINDOW ticks is below the
    // rest thresholds. Past REST_PIN_STREAK quiet ticks it is also transform-pinned, reverting each tick's
    // residual drift: the per-point Gauss-Seidel solve leaks tangential drift for non-box shapes, so a
    // "settled" cylinder or cone would otherwise walk across its own support with its reported velocity
    // reading zero.
    _reconcileRestVelocity(bodies, dt) {
        const win = REST_WINDOW;
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC || !b.isAwake) continue;

            // A body woken by a world change still looks quiet to the ring (it holds the pose it slept
            // in), which would zero its fresh gravity and snap it back every tick. Drop it; it rebuilds
            // and can't re-pin until still for a full window.
            if (b._restRingStale) {
                b._restRingStale = false;
                this._restRing.delete(b.id);
            }
            let r = this._restRing.get(b.id);
            if (!r) {
                r = { pos: [], rot: [], head: 0, count: 0, quietStreak: 0,
                      pinPos: new Vector3(), pinRot: new Quaternion(), pinned: false };
                for (let k = 0; k < win; k++) { r.pos.push(new Vector3()); r.rot.push(new Quaternion()); }
                this._restRing.set(b.id, r);
            }

            if (r.count === win) {
                const oldPos = r.pos[r.head], oldRot = r.rot[r.head];
                const span = win * dt;
                const ndx = b.position.x - oldPos.x, ndy = b.position.y - oldPos.y, ndz = b.position.z - oldPos.z;
                const windowedLinSpeed = Math.sqrt(ndx * ndx + ndy * ndy + ndz * ndz) / span;
                const linQuiet = windowedLinSpeed < REST_LINEAR_SPEED;
                Solver._deriveAngularVelocity(this._tmpDispA, oldRot, b.rotation, span);
                const angQuiet = this._tmpDispA.length() < REST_ANGULAR_SPEED;

                if (linQuiet) b.linear_velocity.set(0, 0, 0);
                if (angQuiet) b.angular_velocity.set(0, 0, 0);

                const disturbed = b._restDisturbed;
                b._restDisturbed = false;
                if (disturbed) { r.quietStreak = 0; r.pinned = false; }

                if (linQuiet && angQuiet && !disturbed) {
                    r.quietStreak++;
                    if (r.quietStreak >= REST_PIN_STREAK) {
                        if (!r.pinned) {
                            r.pinPos.copy(oldPos);
                            r.pinRot.copy(oldRot);
                            r.pinned = true;
                        }
                        b.position.copy(r.pinPos);
                        b.rotation.copy(r.pinRot);
                    }
                } else {
                    r.quietStreak = 0;
                    r.pinned = false;
                }
            } else {
                r.count++;
            }
            r.pos[r.head].copy(b.position);
            r.rot[r.head].copy(b.rotation);
            r.head = (r.head + 1) % win;
        }
    }

    _substep(bodies, manifolds, gravity, h, refresh, constraints) {
        this._integrate(bodies, gravity, h);

        if (refresh) refresh(manifolds);

        this._markRestDisturbances(manifolds);
        this._resetLambdas(manifolds);
        this._solvePositions(manifolds, constraints, h);
        this._deriveVelocities(bodies, h);
        this._solveContactVelocities(manifolds, gravity, h);
    }

    _markRestDisturbances(manifolds) {
        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            let touching = false;
            for (let i = 0; i < manifold.points.length; i++) {
                if (manifold.points[i].signedDistance >= -REST_TOUCH_BAND) { touching = true; break; }
            }
            if (!touching) continue;
            if (a.bodyType === RigidBody.DYNAMIC && Solver._bodyIsMoving(b)) a._restDisturbed = true;
            if (b.bodyType === RigidBody.DYNAMIC && Solver._bodyIsMoving(a)) b._restDisturbed = true;
        }
    }

    static _bodyIsMoving(body) {
        const driven = body.bodyType === RigidBody.KINEMATIC || body.isKinematicCharacter === true;
        if (!driven) return false;
        const lv = body.linear_velocity, av = body.angular_velocity;
        return lv.x * lv.x + lv.y * lv.y + lv.z * lv.z > REST_LINEAR_SPEED * REST_LINEAR_SPEED ||
            av.x * av.x + av.y * av.y + av.z * av.z > REST_ANGULAR_SPEED * REST_ANGULAR_SPEED;
    }

    _resetLambdas(manifolds) {
        for (const manifold of manifolds.values()) {
            for (let i = 0; i < manifold.points.length; i++) {
                manifold.points[i].normalLambda = 0;
                manifold.points[i].tangentLambda1 = 0;
                manifold.points[i].tangentLambda2 = 0;
                manifold.points[i]._credited = 0;
            }
        }
    }

    // Collects the manifolds that describe ONE physical contact set: a dynamic body touching several
    // coplanar non-dynamic surfaces - four mesh tiles butting together, a box lying across two ground
    // pieces. The per-manifold centroid solve is right for a single face patch, but applies one
    // restitution+friction impulse PER SURFACE when there are several: each is offset from the centre
    // of mass, the torques stop cancelling, and the body is handed a lateral kick and spin it was never
    // given. Only runs of two or more are grouped.
    _coplanarPatchGroups(manifolds) {
        const groups = this._patchGroups;
        const byBody = this._patchGroupsByBody;
        groups.clear();
        byBody.clear();
        for (const m of manifolds.values()) {
            const a = m.bodyA, b = m.bodyB;
            const aDyn = a.bodyType === RigidBody.DYNAMIC;
            const bDyn = b.bodyType === RigidBody.DYNAMIC;
            if (aDyn === bDyn) continue;                       // one mobile body, one fixed surface
            const dyn = aDyn ? a : b, other = aDyn ? b : a;
            if (other.bodyType !== RigidBody.STATIC) continue;
            const pts = m.points, n = pts.length;
            if (n < 2) continue;
            const bothBoxes = (dyn.shape instanceof BoxShape) && (other.shape instanceof BoxShape);
            // Contact normals are stored A-relative; express this manifold's in the group's frame,
            // where the dynamic body is A, so manifolds listed the other way round still match.
            const flip = !aDyn;
            const fs = flip ? -1 : 1;
            let meshFace = !bothBoxes, engaged = 0, nx = 0, ny = 0, nz = 0;
            for (let i = 0; i < n; i++) {
                const p = pts[i];
                if (p.normalLambda < 0) engaged++;
                if (!p.fromMeshFace && !p.fromFacePatch) meshFace = false;
                nx += fs * p.normal.x; ny += fs * p.normal.y; nz += fs * p.normal.z;
            }
            if (engaged < 1 || (!bothBoxes && !meshFace)) continue;
            const nl = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
            let entries = byBody.get(dyn.id);
            if (!entries) { entries = []; byBody.set(dyn.id, entries); }
            entries.push({ manifold: m, dyn: dyn, other: other, useAll: meshFace,
                nx: nx / nl, ny: ny / nl, nz: nz / nl,
                friction: Math.sqrt(Math.max(dyn.friction, 0) * Math.max(other.friction, 0)) });
        }
        for (const entries of byBody.values()) {
            let start = 0;
            while (start < entries.length) {
                const head = entries[start];
                let end = start + 1;
                while (end < entries.length) {
                    const q = entries[end];
                    if (q.useAll !== head.useAll) break;
                    if (q.nx * head.nx + q.ny * head.ny + q.nz * head.nz < this.COPLANAR_NORMAL_DOT) break;
                    end++;
                }
                if (end - start > 1) {
                    const group = { manifolds: [], dyn: head.dyn, other: head.other, useAll: head.useAll,
                        nx: head.nx, ny: head.ny, nz: head.nz, friction: 0 };
                    for (let i = start; i < end; i++) {
                        group.manifolds.push(entries[i].manifold);
                        group.friction += entries[i].friction;
                    }
                    group.friction /= (end - start);
                    group.first = group.manifolds[0];
                    for (let i = 0; i < group.manifolds.length; i++) this._patchGroups.set(group.manifolds[i], group);
                }
                start = end;
            }
        }
        return groups;
    }

    // A body's manifolds are grouped per body, then split into coplanar runs. Done in two passes so
    // the run grouping sees one body's manifolds together, in manifold order.
    _solvePositions(manifolds, constraints, h) {
        this._collectSupportBounds(manifolds);
        for (let iter = 0; iter < this.iterations; iter++) {
            for (const manifold of manifolds.values()) {
                this._solveManifold(manifold, h);
            }
            if (constraints) {
                for (let i = 0; i < constraints.length; i++) {
                    if (constraints[i].enabled) constraints[i].solve(h);
                }
            }
        }
    }

    static _manifoldIsInert(bodyA, bodyB) {
        const aFree = bodyA.bodyType === RigidBody.DYNAMIC && bodyA.isAwake;
        const bFree = bodyB.bodyType === RigidBody.DYNAMIC && bodyB.isAwake;
        return !aFree && !bFree;
    }

    _solveManifold(manifold, h) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
        if (Solver._manifoldIsInert(bodyA, bodyB)) return;
        // Reset per manifold, never carried over: a stale extent would be applied to the next
        // manifold's points, which are somewhere else entirely.
        this._checkSupport = false;
        const n = manifold.points.length;
        // A body teetering on a ledge has its patch clipped to a single point, and that point IS the whole
        // support region: the centre of mass can sit past it, and then the contact cannot hold the body up
        // without rotating it, so the extent test must run BEFORE the single-point early return. A lone
        // contact point is a pivot, and suppressing its angular response is what leaves a body balanced
        // upright on its rim, perfectly still, forever.
        this._checkSupport = true;
        this._supportBounds(bodyA, bodyB, manifold, n);
        if (n <= 1) {
            if (n === 1) this._solvePoint(manifold.points[0], bodyA, bodyB, h);
            return;
        }
        // The extent test below only means anything for the closed-form box-box patch, generated
        // complete in one go and clipped to the supported part of the box's face. Mesh and
        // single-witness sets grow and re-clip through a landing, so testing them against it costs more
        // than it buys.
        let boxBoxPatch = true;
        for (let i = 0; i < n; i++) {
            if (!manifold.points[i].fromBoxBox) { boxBoxPatch = false; break; }
        }
        let suppressPatchTorque = false;
        if (n > 1) {
            suppressPatchTorque = true;
            const n0 = manifold.points[0].normal;
            for (let i = 0; i < n; i++) {
                const p = manifold.points[i];
                if ((!p.fromBoxBox && !p.fromFacePatch) || p.normal.x * n0.x + p.normal.y * n0.y + p.normal.z * n0.z < 0.9999) {
                    suppressPatchTorque = false;
                    break;
                }
            }
        }
        // A coplanar patch should be torque-free only after a body is genuinely quiet and
        // upright. During a corner/edge tip, suppressing angular correction prevents the body
        // from rotating onto its face.
        if (suppressPatchTorque) {
            const p0 = manifold.points[0];
            suppressPatchTorque = this._suppressQuietVerticalLanding(bodyA, bodyB, p0, p0.normal.x, p0.normal.y, p0.normal.z);
        }
        if (suppressPatchTorque) this._skipPositionAngular = true;
        for (let i = 0; i < n; i++) {
            // Closed-form box patches provide a complete face manifold; capping their penetration
            // correction can leave a tumbling box sunk below a flat support while its angular contact
            // corrections keep injecting work.
            this._solvePoint(manifold.points[i], bodyA, bodyB, h, !manifold.points[i].fromBoxBox && !manifold.points[i].fromMeshFace);
        }
        if (suppressPatchTorque) this._skipPositionAngular = false;
    }


    _solveContactVelocities(manifolds, gravity, h) {
        const groups = this._coplanarPatchGroups(manifolds);
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            if (Solver._manifoldIsInert(bodyA, bodyB)) continue;
            const group = groups.get(manifold);
            if (group) {
                // Solved once, at the first member's turn in the manifold order, so the rest of the
                // loop (and _solveAngularFriction below) still runs in the usual sequence.
                if (group.first === manifold && !this._solvePatchGroup(group, gravity, h)) {
                    for (let g = 0; g < group.manifolds.length; g++) {
                        const m = group.manifolds[g];
                        for (let i = 0; i < m.points.length; i++) {
                            this._solveContactVelocity(m.points[i], m.bodyA, m.bodyB, gravity, h);
                        }
                    }
                }
            } else if (!this._boxFacePatchVelocity(manifold, bodyA, bodyB, gravity, h)) {
                for (let i = 0; i < manifold.points.length; i++) {
                    this._solveContactVelocity(manifold.points[i], bodyA, bodyB, gravity, h);
                }
            }
            if (manifold.points.length > 0) {
                let ref = manifold.points[0];
                for (let i = 1; i < manifold.points.length; i++) {
                    if (Math.abs(manifold.points[i].normalLambda) > Math.abs(ref.normalLambda)) ref = manifold.points[i];
                }
                this._solveAngularFriction(ref, bodyA, bodyB, h);
            }
        }
    }
}

Solver.RESTITUTION_SLOP_FACTOR = 8;
Solver.MAX_PENETRATION_PER_SUBSTEP = 0.005;

// Half-width of the support extent a body's centre may sit outside of and still count as resting ON
// the patch rather than overhanging it. The dead zone must also cover a curved shape's probe cloud,
// whose samples read a few millimetres of apparent overhang that is sampling noise, not a real
// overhang, and reactivating its torque there makes it buzz instead of rest.
var SUPPORT_CENTRE_TOL = 0.005;
var CURVED_SUPPORT_CENTRE_TOL = 0.02;

// Ring of directions the contact-region probe samples a body's support in - the same ring, tilt and
// count ConvexTri uses on its curved contacts.
Solver.PROBE_COUNT = 4;
// How far off the contact normal the ring samples, and this must be SMALL. At 0.5 rad a probe lands on a
// curved surface's SHOULDER - R*(1-cos 0.5) = 0.106*R off the contact plane - which is not part of the
// contact, so a barrel comes back as "climbed" as a tilted cap and no depth band separates them. Near the
// normal it lands on the contact FEATURE instead, whose rise along the shape's own axis is zero, while a
// tilted cap's samples stay within R*sin(tilt) of its one true contact point.
Solver.PROBE_TILT = 0.07;
// How far off the CONTACT PLANE a probe sample may sit and still count as part of the body's contact
// region. Measured against the plane rather than the witness, and an absolute contact tolerance:
// admitting a sample further off than this would fabricate a region the body is not touching.
Solver.PROBE_DEPTH_BAND = 0.01;
Solver._PROBE_U = [Solver.PROBE_TILT, -Solver.PROBE_TILT, 0, 0];
Solver._PROBE_V = [0, 0, Solver.PROBE_TILT, -Solver.PROBE_TILT];

var REST_WINDOW = 8;
var REST_LINEAR_SPEED = 0.02;
var REST_ANGULAR_SPEED = 0.05;
var REST_PIN_STREAK = 12;
var REST_TOUCH_BAND = 0.005;

// Does this shape have flat faces a body can tip from face to face onto? A box or a hull does; a
// sphere, capsule, cylinder or cone does not - nothing to tip onto, and the rotation an off-centre
// push generates about its point or line contact is real. Compounds count as flat only when every
// child does.
function isFlatFaced(shape) {
    if (shape instanceof BoxShape || shape instanceof ConvexShape) return true;
    if (shape instanceof CompoundShape) {
        const children = shape.children || shape._children;
        if (!children || children.length === 0) return true;
        for (let i = 0; i < children.length; i++) {
            const s = children[i].shape || children[i];
            if (!isFlatFaced(s)) return false;
        }
        return true;
    }
    return false;
}

ActionPhysics.Solver = Solver;
