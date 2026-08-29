/**
 * NarrowPhase: dispatch layer tying Midphase's primitive-shape pairs through GJK/EPA into
 * ContactDetails, and routing them into the right ContactManifold.
 *
 * Produces: contacts with accurate point, normal, signed distance (plan.md, Narrowphase contract).
 * May assume the pair is worth testing (a broadphase/midphase candidate). Must never cull contacts
 * for staleness, clamp depth, or second-guess its own math - that discipline lives entirely in
 * GJK/EPA/ContactDetails already; this file only wires them together and routes results into
 * manifolds. It owns exactly one thing of its own: one MinkowskiSupport/GJK/EPA instance PER
 * BODY-PAIR SLOT (reused across ticks for that slot, never shared across different pairs live at
 * the same time), and grouping this tick's contacts by canonical pair key before handing them to
 * ContactManifoldList.refresh().
 */
class NarrowPhase {
    // Base speculative margin (metres): a contact is reported once its signed distance is within
    // this of touching, even while still SEPARATED, so a manifold point exists BEFORE overlap
    // occurs. This is the whole mechanism of speculative contacts (plan.md, "Continuous collision /
    // speculative contacts" and the derived-velocity fix): the solver's non-penetration constraint,
    // evaluated every substep against the body's PREDICTED position, needs a point already present
    // to stop the body AT touch instead of first letting it dig in and then digging it back out
    // (the deep-correction -> large derived velocity failure the base margin prevents). Per-pair,
    // the base is widened by how far the pair can actually close in one tick (|v_rel| * dt) so a
    // fast body's contact is still caught a full tick ahead - see step().
    static SPECULATIVE_BASE = 0.02;

    constructor() {
        this.manifolds = new ContactManifoldList();
        this._dt = 1 / 60; // set each tick by step(); the fallback only matters if step() is never called
        // Scratch GJK/EPA instances, reused across every pair tested this tick. Safe because
        // narrowphase runs pairs one at a time (never two GJK.run() calls interleaved) - see
        // plan.md's scratch-memory rule: per-stage arena, not a global shared across unrelated
        // algorithms. These belong to NarrowPhase alone.
        this._gjk = new GJK();
        this._epa = new EPA();
        this._contactPool = []; // reused ContactDetails objects, grown as needed, never shrunk
        this._poolIndex = 0;
    }

    _nextPooledContact() {
        if (this._poolIndex >= this._contactPool.length) this._contactPool.push(new ContactDetails());
        return this._contactPool[this._poolIndex++];
    }

    // Runs narrowphase for one tick: broadphase pairs in, manifolds refreshed out.
    //   broadphasePairs: [[bodyA, bodyB], ...] from SAPBroadphase.computePairs()
    //   midphase: a Midphase instance (expands compound/mesh pairs to primitives)
    //   dt: this tick's timestep, used to size the per-pair speculative margin (how far the pair
    //       can close in one tick). Optional; falls back to the last value / 1/60 if omitted.
    step(broadphasePairs, midphase, dt) {
        if (dt) this._dt = dt;
        this._poolIndex = 0;
        const contactsByPair = new Map(); // canonical "idA:idB" key -> ContactDetails[]

        for (let p = 0; p < broadphasePairs.length; p++) {
            const bodyA = broadphasePairs[p][0], bodyB = broadphasePairs[p][1];
            const primitivePairs = midphase.expandPair(bodyA, bodyB);
            const key = bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
            const margin = this._speculativeMargin(bodyA, bodyB);

            for (let i = 0; i < primitivePairs.length; i++) {
                const contact = this._testPrimitivePair(primitivePairs[i].a, primitivePairs[i].b);
                // signedDistance: positive = overlapping, negative = separated by that gap (plan.md
                // convention). Report the contact while overlapping OR within the speculative
                // margin of touching; drop it only once the gap exceeds the margin - too far this
                // tick for the pair to reach, so no manifold point is warranted. This is the ONE
                // place narrowphase decides a pair is "not worth a manifold entry" (see
                // _testPrimitivePair) and it is a distance-vs-margin test, never a staleness or
                // depth-quality judgement (Rule 1: narrowphase reports geometry, it does not
                // second-guess the solver's use of it).
                if (contact.signedDistance < -margin) continue;
                let list = contactsByPair.get(key);
                if (!list) { list = []; contactsByPair.set(key, list); }
                list.push(contact);
            }

            // Ensure a manifold exists for this pair even if this tick found zero contacts for it
            // yet, so ContactManifoldList.refresh() below has something to prune if the pair was
            // previously touching and just separated. getOrCreate() is idempotent for an existing
            // pair.
            this.manifolds.getOrCreate(bodyA, bodyB);
        }

        this.manifolds.refresh(contactsByPair);
        return this.manifolds;
    }

    // Per-pair speculative margin: the base margin widened by how far the two bodies can close
    // along their relative velocity in one tick (|v_rel| * dt). A slow/resting pair uses ~the base;
    // a fast approach gets a proportionally larger lookahead so the contact is still reported a full
    // tick before overlap - which is exactly what lets the solver stop the body at touch rather
    // than after it has tunnelled partway in. Uses the full relative speed (not the normal
    // component - narrowphase has no single contact normal yet at this point, and the closing speed
    // is an upper bound on approach along any normal, so it never UNDER-estimates the needed
    // lookahead, matching broadphase's own no-false-negatives discipline).
    _speculativeMargin(bodyA, bodyB) {
        const dvx = bodyA.linear_velocity.x - bodyB.linear_velocity.x;
        const dvy = bodyA.linear_velocity.y - bodyB.linear_velocity.y;
        const dvz = bodyA.linear_velocity.z - bodyB.linear_velocity.z;
        const relSpeed = Math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
        // Add each body's angular corner speed (|omega|*R): a contact FEATURE on a spinning body
        // (a corner, an edge) approaches at up to this rate even when the centre barely moves, so a
        // margin sized only from centre-relative linear speed reports the contact too late for a
        // tipping body - the corner is already deep. Same reasoning as the broadphase AABB's angular
        // sweep (RigidBody._recomputeBroadphaseAABB); both stages must look ahead by the same
        // corner motion or broadphase surfaces the pair and narrowphase then drops it as "too far".
        const angSpeed = NarrowPhase._angularCornerSpeed(bodyA) + NarrowPhase._angularCornerSpeed(bodyB);
        return NarrowPhase.SPECULATIVE_BASE + (relSpeed + angSpeed) * this._dt;
    }

    // Upper bound on how fast any point on `body` moves purely from its rotation: |omega| times the
    // body's bounding radius (farthest corner of its tight AABB from centre).
    static _angularCornerSpeed(body) {
        const w = body.angular_velocity;
        const wMag = Math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        if (wMag === 0) return 0;
        const aabb = body.getAABB();
        const ex = (aabb.max.x - aabb.min.x) * 0.5, ey = (aabb.max.y - aabb.min.y) * 0.5, ez = (aabb.max.z - aabb.min.z) * 0.5;
        return wMag * Math.sqrt(ex * ex + ey * ey + ez * ez);
    }

    // Re-measures the GEOMETRY of the contact points already in each manifold against the bodies'
    // CURRENT (predicted, mid-substep) transforms, in place. The solver calls this once per substep
    // (see Solver.step's `refresh`), so a contact whose feature moves as a body rotates is solved
    // against live geometry rather than a frozen tick-start normal/anchor - the fix for the
    // rotational derived-velocity blow-up on corner contacts.
    //
    // This updates geometry ONLY. It never adds, removes, or re-matches points, and never touches
    // the manifold's point SET or its warm-start lambda - that ownership stays with the manifold's
    // once-per-tick update() (plan.md's rule against mid-tick manifold churn). A point whose contact
    // has genuinely separated this substep simply gets a negative signed distance here and the
    // solver's own C<=0 guard makes it inert; it is not culled mid-tick.
    //
    // Only primitive-vs-primitive body pairs are refreshed. A compound/mesh body's contact came
    // from an expanded child/triangle whose identity this method does not track per point, so those
    // manifolds keep their tick-start geometry (no regression - that is exactly today's behaviour
    // for every contact). Re-expanding compounds/meshes per substep is a later optimisation if
    // rotating compound bodies turn out to need it.
    refreshManifoldGeometry(manifolds) {
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            if (this._isCompoundOrMesh(bodyA.shape) || this._isCompoundOrMesh(bodyB.shape)) continue;

            const placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
            const placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };
            const fresh = this._testPrimitivePair(placedA, placedB); // one fresh contact for this pair

            // Update the nearest existing point (in world space) with the fresh geometry, keeping
            // its warm-start lambda and its persistent local anchors' IDENTITY - only the values
            // the solver reads live (normal, and the anchors it recomputes C from) are refreshed.
            let best = null, bestDistSq = Infinity;
            for (let i = 0; i < manifold.points.length; i++) {
                const p = manifold.points[i];
                const dx = p.point.x - fresh.point.x, dy = p.point.y - fresh.point.y, dz = p.point.z - fresh.point.z;
                const d = dx * dx + dy * dy + dz * dz;
                if (d < bestDistSq) { bestDistSq = d; best = p; }
            }
            if (!best) continue;
            best.point.copy(fresh.point);
            best.pointOnA.copy(fresh.pointOnA);
            best.pointOnB.copy(fresh.pointOnB);
            best.signedDistance = fresh.signedDistance;
            // Keep the ESTABLISHED normal through the exact-touch band, exactly as the manifold's
            // once-per-tick update() does (ContactManifold.EXACT_TOUCH_BAND) - the per-substep
            // refresh MUST honour the same rule, or it silently clobbers a good resting normal with
            // the ambiguous diagonal GJK/EPA returns at signed-distance ~0 every substep, which is
            // the penetrate-then-launch bug the once-per-tick guard was added to prevent (it bit
            // spheres hard: a flush sphere kept getting a (-0.71,0,0.71) normal and launched to y=200+).
            if (Math.abs(fresh.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) best.normal.copy(fresh.normal);
            // Re-anchor to the CURRENT geometry so C is measured from where the feature is NOW - the
            // whole point of refreshing. Persisting stale anchors would defeat it. Lambda is left
            // untouched (warm start survives).
            best.setLocalAnchors(bodyA, bodyB);
        }
    }

    _isCompoundOrMesh(shape) {
        return (typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape) ||
            (typeof MeshShape !== 'undefined' && shape instanceof MeshShape);
    }

    // Runs GJK (and EPA if overlapping) for one primitive-shape pair, returning a pooled
    // ContactDetails. Always returns a filled contact carrying its signed distance (positive =
    // overlapping, negative = separated gap); the caller (step) decides whether that distance is
    // within the speculative margin and thus worth a manifold entry. This function itself never
    // culls - it only measures (Rule 1).
    _testPrimitivePair(placedA, placedB) {
        const support = new MinkowskiSupport(placedA, placedB);
        const gjkResult = this._gjk.run(support);
        const contact = this._nextPooledContact();
        if (gjkResult.overlapping) {
            const epaResult = this._epa.run(support, gjkResult.simplex);
            contact.setFromEPA(epaResult);
        } else {
            contact.setFromGJKSeparated(gjkResult);
        }
        return contact;
    }
}

ActionPhysics.NarrowPhase = NarrowPhase;
