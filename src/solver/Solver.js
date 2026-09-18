// XPBD solver (Muller et al. 2020). Velocity is derived from position (v = (x - x_prev) / h).
// Per substep: integrate -> refresh contact geometry -> reset lambdas -> solve positions ->
// derive velocity -> solve contact velocity. See Integrate/PositionSolve/VelocitySolve.
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

        // Horizontal extent of the manifold being solved, filled by _supportBounds when the manifold is
        // a closed-form box-box patch (see _solveManifold).
        this._supMinX = 0; this._supMaxX = 0; this._supMinZ = 0; this._supMaxZ = 0;
        this._checkSupport = false;
        this._supportCentreTol = SUPPORT_CENTRE_TOL;
        // Per-substep union of every upward patch each body stands on, keyed by body id. See
        // _collectSupportBounds.
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

    // Widens a body's support entry with the body's OWN contact region, for contact points that
    // carry no patch geometry of their own - a bare GJK/EPA witness.
    //
    // The support under a body is where its contact REGION is, and one witness is a single sample of
    // that region, not the region. Treating it as the region is wrong in both directions, and both
    // are measurable. A cone lying on its side rests on one point of its base rim with its centre of
    // mass half a metre out to the side, and reading that one point as its whole support is what left
    // the cone balanced there - tip 0.400 m in the air, the base radius exactly, |v| and |w| both
    // 0.000 - for a hundred ticks until the test's scripted shove knocked it over. Conversely a cone
    // standing on its base has its witness somewhere on the base disc, so reading the witness alone as
    // the support says the centre of mass hangs past its edge and tips the cone over (rest height
    // 0.07085 where the base radius puts it at 0.22).
    //
    // Asking the SHAPE settles both: probe its support in a ring of directions around the contact and
    // keep the samples that come back at the contact depth. On a flat face they spread across it; on a
    // rounded contact they have already climbed clear of the contact plane and are dropped, collapsing
    // the region to the contact itself. Same ring ConvexTri uses to describe a curved contact against a
    // mesh (see ConvexTri.PROBE_TILT).
    _widenSupportWithProbe(body, isA, nx, ny, nz, px, py, pz, sd, e) {
        const shape = body.shape;
        // A sphere's contact region is a point whatever it rests on, so its witness already IS the
        // region. A compound has no support function of its own (it dispatches per child) and a mesh
        // is not convex, so neither can be asked.
        if (shape instanceof SphereShape || shape instanceof CompoundShape || shape instanceof MeshShape) return;
        if (typeof shape.supportInto !== 'function') return;

        // The direction from this body INTO the contact. The normal is stored B-relative, pointing B->A.
        const dx = isA ? -nx : nx, dy = isA ? -ny : ny, dz = isA ? -nz : nz;
        this._probeNormal.set(dx, dy, dz);
        // The ring's tangent basis decides WHICH points of the contact region get sampled, and an
        // arbitrary perpendicular pair misses the one direction that matters for a barrel: a capsule
        // or cylinder resting on its side touches along a LINE, and the samples that lie on that line
        // are the ones out at the barrel's ends. Probing towards the shape's own axis returns exactly
        // those - the support in a direction tilted towards the axis lands on the cap end AT CONTACT
        // DEPTH, rise zero, because a barrel is straight - so the line's full length is recovered while
        // the curved shoulder samples, which climb clear by a share of the radius, stay rejected. With
        // an arbitrary basis those axial samples are never taken, the region collapses to the single
        // witness, and a barrel whose witness is not under its centre of mass reads as overhanging
        // (measured: the coin-pusher's cylinder rolls 1.76 turns where it needs 3, and leaves the ramp
        // early). An upright barrel has no perpendicular axis component, so it keeps the usual basis,
        // and its flat cap - whose samples are coplanar - widens on its own.
        Solver._tangentBasis(this._probeNormal, this._probeT1, this._probeT2);
        const t1 = this._probeT1, t2 = this._probeT2, dir = this._probeDir, out = this._probeOut;
        const inv = this._probeInvRot.copy(body.rotation).invert();

        // How far off the contact plane a sample may sit and still count as part of the body's
        // contact region. This has to be a CONTACT tolerance, not a share of the body: a sample this
        // far off the plane is not touching the surface, whatever the body's size. Scaling it to the
        // body (the old PROBE_DEPTH_BAND_FRACTION * smallest-extent) let a large-radius curved shape
        // admit samples centimetres clear of the plane - a 300 mm-radius capsule's shoulder samples
        // rise 37 mm at this probe tilt - and so fabricate a contact region wide enough to swallow a
        // centre of mass that is genuinely off the contact. That is a false equilibrium: measured, a
        // capsule dropped at 15 degrees onto a flat floor sat at its starting tilt and slept instead
        // of toppling, because its one real contact point had been replaced by a 270 mm-wide
        // "patch" it was never touching. A flat face still widens correctly at any band - its
        // samples are coplanar to floating point - and a barrel's LINE is recovered by probing along
        // the shape's own axis, so an absolute contact tolerance now separates the two real contact
        // shapes from curvature noise instead of admitting all three.
        const band = Solver.PROBE_DEPTH_BAND;

        for (let i = 0; i < Solver.PROBE_COUNT; i++) {
            const u = Solver._PROBE_U[i], v = Solver._PROBE_V[i];
            dir.set(dx + t1.x * u + t2.x * v, dy + t1.y * u + t2.y * v, dz + t1.z * u + t2.z * v);
            const len = Math.sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) || 1;
            dir.scaleInPlace(1 / len);
            MinkowskiSupport.supportOfInto(out, body, inv, dir, this._probeLocal);
            // Measure the sample's height above the CONTACT PLANE, not above the witness. The witness
            // is reported at the penetration depth, so on a settled barrel it sits up to ~20 mm inside
            // the floor while the barrel's own support line - which IS at contact depth - lies that
            // far above it. Judged against the witness, a barrel's samples are rejected for exactly
            // the same reason, and by the same R*(1-cos) amount, as a tilted cap's shoulder samples,
            // so NO band can admit the one and reject the other: measured, a barrelled cylinder of
            // R=0.28 needs the band above 0.030 while a 15-degree capsule of R=0.30 needs it below
            // 0.032 - a 2 mm window that exists only because the radii happen to differ, and the
            // engine has to be right for both. Adding the contact's own signedDistance moves the
            // reference onto the surface the two bodies actually meet at, and then the barrel's
            // samples read ~3 mm off it while the cap's read ~32 mm: a single absolute contact
            // tolerance separates them at ANY radius, which is what makes the tipover possible to fix
            // without re-breaking the barrels.
            const proj = (out.x - px) * dx + (out.y - py) * dy + (out.z - pz) * dz + sd;
            if (proj < -band) continue;
            if (out.x < e.minX) e.minX = out.x;
            if (out.x > e.maxX) e.maxX = out.x;
            if (out.z < e.minZ) e.minZ = out.z;
            if (out.z > e.maxZ) e.maxZ = out.z;
        }
    }

    // Records the horizontal extent of `manifold`'s contact patch into _supMinX/_supMaxX/_supMinZ/
    // _supMaxZ, so _suppressQuietVerticalLanding can tell a patch the body is sitting ON from one it
    // hangs off: a centre of mass inside this extent is supported from below, while a centre that has
    // passed it is an overhang whose normal response cannot hold the body up without rotating it.
    _supportBounds(bodyA, bodyB, manifold, n) {
        // The support under a body is the UNION of every upward patch it stands on this substep, not
        // the patch of whichever manifold happens to be solved. A cone dropped on the seam of a tiled
        // floor has a patch on each side of the seam and its centre of mass outside either one alone;
        // judged manifold by manifold each patch reads as an overhang, both hand out torque, and the
        // body is toppled off a surface it is in fact resting flat on. Unioned, the centre of mass
        // sits inside the support and the contact is solved torque-free - which is what the same
        // floor drawn as one mesh would do. Only multi-manifold bodies differ from the old behaviour:
        // for a body standing on one patch the union is that patch.
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

    // Builds the per-body support union for one substep, keyed by body id. Only upward contacts
    // count (a patch the body is standing on), and only ones that are actually touching - a
    // speculative contact has no load to carry yet. Every point of a patch counts, including the
    // separated probe samples a curved cloud spreads over the shape it stands on - those are what
    // describe the width of its contact region, and dropping them narrows the support to noise.
    // `gen` retires last substep's entries without
    // reallocating: the map keeps one small record per body for the life of the solver.
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
                // A point of a real patch (box-box, BoxTriFace/CapTriFace, PolyClip) that is still
                // separated carries no load and does not widen the support. A ConvexTri probe sample
                // is different: it samples the SHAPE, so around a curved contact its depth swings by
                // the shape's own radius, and its separated samples are precisely what describe how
                // wide the contact region is. Dropping them narrows a capsule's support to noise and
                // lets a settled one be judged overhanging; keeping them is what the width means.
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

    // Zeroes the velocity of a body whose sustained motion over the last REST_WINDOW ticks is below
    // the rest thresholds, from a per-body ring buffer of recent transforms. Once a body has stayed
    // that quiet for REST_PIN_STREAK consecutive ticks it is also transform-pinned: each tick's
    // residual drift is reverted to the previous sampled pose. The per-point Gauss-Seidel contact
    // solve leaks a little tangential drift every substep for non-box shapes (box patches are already
    // centroid-solved, see VelocitySolve.js), so a "settled" cylinder/cone/sphere slowly walks across
    // its support with its reported velocity reading zero. The streak gate keeps this off any body
    // that is only briefly quiet - a rider settling onto a carrier, a shape between bounces - so only
    // a genuinely parked body gets pinned, and a sleeping body then matches a never-slept one exactly.
    // See NOTES.md.
    _reconcileRestVelocity(bodies, dt) {
        const win = REST_WINDOW;
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC || !b.isAwake) continue;

            // A body woken by a world change still looks quiet to the ring (it holds the pose it
            // slept in), so the ring would zero its fresh gravity and snap it back every tick.
            // Drop it; it rebuilds from scratch and can't re-pin until still for a full window.
            // Routine wakes (impulse, contact, island restless) don't set the flag.
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
    // non-dynamic surfaces whose patches are coplanar - four mesh tiles butting together, a body
    // straddling a tile seam, a box lying across two ground pieces. The per-manifold centroid solve
    // is right for a single face patch (see VelocitySolve._boxFacePatchVelocity) but applies one
    // restitution+friction impulse PER SURFACE when there are several: each impulse is offset from
    // the centre of mass, the first one's torque changes the state the next one measures, the torques
    // stop cancelling, and the body is handed a lateral kick and spin it was never given. Solving the
    // coplanar set as one contact set at one centroid makes the tiled ground behave exactly like the
    // single mesh it represents. Only runs of two or more are grouped; everything else is untouched.
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
        // A body teetering on a ledge or a table edge has its patch clipped down to a single point
        // (or one line of two), and that one point is the whole support region: the centre of mass
        // can sit past it, and then the contact cannot hold the body up without rotating it. The
        // extent test must therefore run for a real patch BEFORE the single-point early return -
        // otherwise a quiet body parked past the edge is solved torque-free and hangs there frozen
        // (see _suppressQuietVerticalLanding).
        // The extent test runs for EVERY contact, including a bare GJK/EPA witness. A lone contact
        // point is a pivot: the centre of mass either sits over it - in which case the contact can
        // carry the weight without turning the body - or it does not, and then the contact MUST
        // rotate the body, so suppressing its angular response is what leaves a cone balanced
        // upright on its base rim with its tip a whole radius in the air, perfectly still, forever.
        // The witness being one sample of a rounded contact region rather than its centre is what the
        // tolerance in _supportBounds absorbs - a resting sphere's witness is under its centre of
        // mass, so it is inside the tolerance, while this cone's is half a metre outside.
        this._checkSupport = true;
        this._supportBounds(bodyA, bodyB, manifold, n);
        if (n <= 1) {
            if (n === 1) this._solvePoint(manifold.points[0], bodyA, bodyB, h);
            return;
        }
        // The support-extent test below is only meaningful for the closed-form box-box patch, which is
        // generated complete in one go: it is clipped to the supported part of the box's face, but every
        // point of it belongs to a patch that exists right now, so the extent describes this contact.
        // Mesh and single-witness contacts have no such guarantee (their set grows and re-clips through
        // a landing), and testing them against it costs far more than it buys.
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
            // Closed-form box patches already provide a complete face manifold; do not
            // artificially cap their penetration correction, which can leave a tumbling box
            // sunk below a flat support while its angular contact corrections keep injecting work.
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
// the patch rather than overhanging it. The dead zone also has to cover a curved shape's probe cloud
// (see _supportBounds), which samples the contact region rather than outlining it: a resting capsule
// or cylinder reads a few millimetres of apparent overhang that is sampling noise, not a real
// overhang, and reactivating its torque there makes it buzz instead of rest.
var SUPPORT_CENTRE_TOL = 0.005;
var CURVED_SUPPORT_CENTRE_TOL = 0.02;

// Ring of directions the contact-region probe samples a body's support in (see
// _widenSupportWithProbe) - the same ring, tilt and count ConvexTri uses on its curved contacts.
// PROBE_DEPTH_BAND is how far off the contact plane a sample may sit and still count as part of the
// body's region: on a flat face the samples are coplanar to floating-point, while on a rounded
// contact they have climbed clear of it by a share of the shape's own radius. It is an absolute
// contact tolerance deliberately - see _widenSupportWithProbe for what scaling it to the body broke.
Solver.PROBE_COUNT = 4;
// How far off the contact normal the ring samples, and this must be SMALL. At 0.5 rad a probe lands
// on a curved surface's SHOULDER - R*(1-cos 0.5) = 0.106*R off the contact plane - which is not part
// of the contact at all, so a barrel's samples came back exactly as "climbed" as a tilted cap's and
// no depth band could separate them. Sampling near the normal lands on the contact FEATURE instead:
// the support along a barrel's own axis sits at the barrel's end ON the contact line, rise zero, so
// the line's full length is recovered, while a tilted cap's samples stay within R*sin(tilt) of its one
// true contact point - too narrow to cover a centre of mass that is genuinely off the contact. A flat
// face is unaffected at any tilt: the support in a tilted direction is still a corner of that face.
// Swept against the suite's conflicting fixtures, 0.5 rad fails both barrels and the coin-pusher, 0.2
// and 0.15 fail the tipover on its sleep budget, and 0.05-0.10 passes every one of them.
Solver.PROBE_TILT = 0.07;
// How far off the CONTACT PLANE a probe sample may sit and still count as part of the body's contact
// region (see _widenSupportWithProbe, which measures against the plane rather than the witness, and
// why that distinction is what makes this band work). An absolute contact tolerance: admitting a
// sample further off than this would fabricate a contact region the body is not touching.
Solver.PROBE_DEPTH_BAND = 0.01;
Solver._PROBE_U = [Solver.PROBE_TILT, -Solver.PROBE_TILT, 0, 0];
Solver._PROBE_V = [0, 0, Solver.PROBE_TILT, -Solver.PROBE_TILT];

var REST_WINDOW = 8;
var REST_LINEAR_SPEED = 0.02;
var REST_ANGULAR_SPEED = 0.05;
var REST_PIN_STREAK = 12;
var REST_TOUCH_BAND = 0.005;

// Does this shape have flat faces a body can tip from face to face onto? A box or a hull does; a
// sphere, capsule, cylinder or cone does not (nothing to tip onto - its contact is a point or a
// line, and the rotation an off-centre push generates about it is real). Compounds count as flat
// only when every child does. Used by the position solve (a flat-faced body may only be solved
// torque-free once it has genuinely stopped turning) and by the velocity solve (a flat-faced
// body's patch is a polygon, whose extent gives a trustworthy anchor - see
// VelocitySolve._boxFacePatchVelocity).
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
