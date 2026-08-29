/**
 * NarrowPhase: dispatch layer tying Midphase's primitive-shape pairs through GJK/EPA into
 * ContactDetails, and routing them into the right ContactManifold.
 *
 * Produces: contacts with accurate point, normal, signed distance.
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
    // occurs. This is the whole mechanism of speculative contacts and the derived-velocity fix):
    // the solver's non-penetration constraint,
    // evaluated every substep against the body's PREDICTED position, needs a point already present
    // to stop the body AT touch instead of first letting it dig in and then digging it back out
    // (the deep-correction -> large derived velocity failure the base margin prevents). Per-pair,
    // the base is widened by how far the pair can actually close in one tick (|v_rel| * dt) so a
    // fast body's contact is still caught a full tick ahead - see step().
    static SPECULATIVE_BASE = 0.02;

    // Minimum world-space distance (metres) a manifold point's fresh GJK/EPA result must move from
    // its own current anchor before refreshManifoldGeometry treats it as a real migration and
    // re-anchors - see that method's own comment. Comfortably above what a genuinely slow/settling
    // contact traverses in one substep, comfortably below the near-full-shape-size jump a degenerate
    // line/face contact's ambiguous closest point showed (traced: a cylinder resting flush on its
    // side jumped by ~0.95-1.0, its own half-length, substep to substep).
    static ANCHOR_REFRESH_MIN_MOVE = 0.05;

    // See _synthesizeMirroredPoint's own comment. |contact normal . shape's own axis| above this
    // means the contact is on an END CAP (normal roughly parallel to the axis), which genuinely only
    // ever touches at one real point - mirroring there would fabricate a second point that does not
    // exist. cos(80 deg) ~= 0.17: only rules out contacts within about 10 degrees of a true end-cap
    // normal, comfortably wide of an ordinary barrel-resting contact (normal within a few degrees of
    // perpendicular to the axis).
    static MIRROR_POINT_MAX_AXIAL_NORMAL = 0.17;

    constructor() {
        this.manifolds = new ContactManifoldList();
        this._dt = 1 / 60; // set each tick by step(); the fallback only matters if step() is never called
        // Scratch GJK/EPA instances, reused across every pair tested this tick. Safe because
        // narrowphase runs pairs one at a time (never two GJK.run() calls interleaved) - see
        // Scratch arena owned by this stage, not shared across unrelated
        // algorithms. These belong to NarrowPhase alone.
        this._gjk = new GJK();
        this._epa = new EPA();
        this._contactPool = []; // reused ContactDetails objects, grown as needed, never shrunk
        this._poolIndex = 0;
        this._scratchVec = new Vector3(); // refreshManifoldGeometry's own anchor-drift check
        this._mirrorAxis = new Vector3(); this._mirrorOffset = new Vector3(); // _synthesizeMirroredPoint's own scratch
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
                // signedDistance: positive = overlapping, negative = separated by that gap. Report
                // the contact while overlapping OR within the speculative
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

                // A cylinder/capsule resting on its curved BARREL (not an end-cap) against a flat
                // surface has a true line of contact, not a point - GJK/EPA can only ever report one
                // point along that line, and every correction from that single point carries a
                // nonzero lever arm relative to the body's own axis (the contact point is never
                // exactly under the center of mass along the contact's own length), even for a
                // perfectly symmetric, perfectly resting body. That lever arm's torque does not
                // average out over time - it accumulates, substep after substep, into a real,
                // exponentially growing rotation (traced directly: a resting cylinder's own rotation
                // drifted from ~0 to 0.002 rad over 2000 ticks, roughly doubling every 200 ticks,
                // despite angular_velocity reading exactly zero throughout - a single-point contact
                // model cannot represent this geometry without this bias). The fix is the same one
                // any two-point stabilization uses: synthesize a SECOND contact point, mirrored along
                // the shape's own axis, so the two points' lever arms are symmetric and cancel by
                // construction instead of compounding in one direction.
                const mirrored = this._synthesizeMirroredPoint(contact, primitivePairs[i].a, primitivePairs[i].b);
                if (mirrored) list.push(mirrored);
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
    // once-per-tick update() (the rule against mid-tick manifold churn). A point whose contact
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
            if (manifold.points.length === 0) continue;

            const placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
            const placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };
            const fresh = this._testPrimitivePair(placedA, placedB); // one fresh contact for this pair
            this._refreshOnePoint(manifold, fresh, bodyA, bodyB, null);

            // A synthesized mirrored point (see _synthesizeMirroredPoint's own comment) needs its OWN
            // fresh geometry refreshed too, matched independently against whichever manifold point is
            // nearest IT specifically - reusing the same `fresh` result for both would refresh the
            // mirrored point with the real point's own geometry, collapsing the two-point stabilization
            // this mechanism exists for back into a single effective point.
            const mirrored = this._synthesizeMirroredPoint(fresh, placedA, placedB);
            if (mirrored) this._refreshOnePoint(manifold, mirrored, bodyA, bodyB, fresh);
        }
    }

    // Updates whichever point in `manifold` is nearest `freshContact` (in world space) with its
    // geometry, keeping the point's warm-start lambda and local-anchor IDENTITY - only the values
    // the solver reads live (normal, and the anchors it recomputes C from) are refreshed. `exclude`
    // (optional): a ContactDetails to skip when picking the nearest point, so a real point and its
    // mirrored counterpart (see refreshManifoldGeometry's own comment) never both claim the same
    // manifold point when they happen to land close together.
    _refreshOnePoint(manifold, freshContact, bodyA, bodyB, exclude) {
        let best = null, bestDistSq = Infinity;
        for (let i = 0; i < manifold.points.length; i++) {
            const p = manifold.points[i];
            if (p === exclude) continue;
            const dx = p.point.x - freshContact.point.x, dy = p.point.y - freshContact.point.y, dz = p.point.z - freshContact.point.z;
            const d = dx * dx + dy * dy + dz * dz;
            if (d < bestDistSq) { bestDistSq = d; best = p; }
        }
        if (!best) return;
        // A curved shape resting flush along a LINE (a cylinder's barrel on flat ground, not a
        // single point) has no unique closest point - GJK/EPA legitimately returns a different,
        // equally-valid point along that line from one call to the next even though nothing about
        // the real contact changed. Re-anchoring to every such jump flips the position solve's
        // lever arm (and so its torque direction) every substep, which kept re-injecting angular
        // velocity a settled round shape's own rolling resistance had just removed - a resting
        // cylinder/cone never fully stopped spinning no matter how strong the damping was, because
        // the anchor itself never held still long enough to stay damped. Re-anchor only when the
        // fresh point has genuinely moved from where THIS point's CURRENT anchor sits in world
        // space right now (currentAnchorBInto - the anchor tracks the body's own rotation each
        // substep even while frozen, so real cumulative drift still crosses this threshold and
        // triggers a re-anchor; comparing against a stale cached pointOnB instead let real motion
        // accumulate invisibly and never re-anchor at all, a real, confirmed bug - a cylinder held
        // at a stale anchor for 1000+ ticks eventually drifted enough to launch outright, y=58+).
        best.currentAnchorBInto(this._scratchVec, bodyB);
        const movedSq = (freshContact.pointOnB.x - this._scratchVec.x) * (freshContact.pointOnB.x - this._scratchVec.x) +
            (freshContact.pointOnB.y - this._scratchVec.y) * (freshContact.pointOnB.y - this._scratchVec.y) +
            (freshContact.pointOnB.z - this._scratchVec.z) * (freshContact.pointOnB.z - this._scratchVec.z);
        const genuineMove = movedSq >= NarrowPhase.ANCHOR_REFRESH_MIN_MOVE * NarrowPhase.ANCHOR_REFRESH_MIN_MOVE;
        best.point.copy(freshContact.point);
        best.signedDistance = freshContact.signedDistance;
        // Keep the ESTABLISHED normal through the exact-touch band, exactly as the manifold's
        // once-per-tick update() does (ContactManifold.EXACT_TOUCH_BAND) - the per-substep
        // refresh MUST honour the same rule, or it silently clobbers a good resting normal with
        // the ambiguous diagonal GJK/EPA returns at signed-distance ~0 every substep, which is
        // the penetrate-then-launch bug the once-per-tick guard was added to prevent (it bit
        // spheres hard: a flush sphere kept getting a (-0.71,0,0.71) normal and launched to y=200+).
        if (Math.abs(freshContact.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) best.normal.copy(freshContact.normal);
        if (genuineMove) {
            best.pointOnA.copy(freshContact.pointOnA);
            best.pointOnB.copy(freshContact.pointOnB);
            // Re-anchor to the CURRENT geometry so C is measured from where the feature is NOW -
            // the whole point of refreshing, for a contact that has actually moved. Persisting a
            // stale anchor through a real migration would defeat this mechanism entirely (see its
            // own comment above - a fast-rotating body's far corner slamming in undetected).
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

    // See step()'s own comment for why this exists. Returns a second ContactDetails mirrored along
    // whichever of placedA/placedB is a cylinder or capsule resting on its BARREL (contact normal
    // roughly perpendicular to the shape's own local Y axis - an end-cap contact has the normal
    // roughly PARALLEL to the axis instead, and genuinely only ever touches at one point, so this
    // correctly does nothing there), or null if neither side qualifies. Only ever synthesizes for a
    // primitive-vs-primitive pair already confirmed touching/overlapping by the real contact - never
    // invents a contact from nothing.
    _synthesizeMirroredPoint(contact, placedA, placedB) {
        let axisBody = null, halfLen = 0;
        if (placedA.shape.type === 'cylinder') { axisBody = placedA; halfLen = placedA.shape.halfHeight; }
        else if (placedA.shape.type === 'capsule') { axisBody = placedA; halfLen = placedA.shape.segmentHalfLength; }
        else if (placedB.shape.type === 'cylinder') { axisBody = placedB; halfLen = placedB.shape.halfHeight; }
        else if (placedB.shape.type === 'capsule') { axisBody = placedB; halfLen = placedB.shape.segmentHalfLength; }
        if (!axisBody || halfLen <= 0) return null;

        this._mirrorAxis.set(0, 1, 0);
        axisBody.rotation.transformVectorInPlace(this._mirrorAxis);
        const nx = contact.normal.x, ny = contact.normal.y, nz = contact.normal.z;
        const axialComponent = Math.abs(this._mirrorAxis.x * nx + this._mirrorAxis.y * ny + this._mirrorAxis.z * nz);
        // End-cap contact (normal mostly ALONG the axis) - genuinely one point, do nothing.
        if (axialComponent > NarrowPhase.MIRROR_POINT_MAX_AXIAL_NORMAL) return null;

        // Where along the axis the real contact point already sits, relative to the shape's own
        // center - the mirror offset is placed symmetrically on the OTHER side of center from there,
        // at the same distance, so the two points bracket the center rather than both landing on the
        // same side (which a fixed +halfLen/-halfLen pair would risk for a contact that is not
        // perfectly centered to begin with).
        this._mirrorOffset.copy(contact.point).subInPlace(axisBody.position);
        const alongAxis = this._mirrorOffset.x * this._mirrorAxis.x + this._mirrorOffset.y * this._mirrorAxis.y + this._mirrorOffset.z * this._mirrorAxis.z;
        const mirrorAlong = -alongAxis; // reflect through the center
        const shift = mirrorAlong - alongAxis;

        const mirrored = this._nextPooledContact();
        mirrored.point.set(contact.point.x + this._mirrorAxis.x * shift, contact.point.y + this._mirrorAxis.y * shift, contact.point.z + this._mirrorAxis.z * shift);
        mirrored.pointOnA.set(contact.pointOnA.x + this._mirrorAxis.x * shift, contact.pointOnA.y + this._mirrorAxis.y * shift, contact.pointOnA.z + this._mirrorAxis.z * shift);
        mirrored.pointOnB.set(contact.pointOnB.x + this._mirrorAxis.x * shift, contact.pointOnB.y + this._mirrorAxis.y * shift, contact.pointOnB.z + this._mirrorAxis.z * shift);
        mirrored.normal.copy(contact.normal);
        mirrored.signedDistance = contact.signedDistance; // same depth - the barrel's radius is constant along its length
        return mirrored;
    }
}

ActionPhysics.NarrowPhase = NarrowPhase;
