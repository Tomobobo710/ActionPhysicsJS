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

    // |contact-normal . curved-shape-axis| above this means the contact is on an END CAP (normal
    // roughly PARALLEL to the axis - a cylinder stood upright, a cone tip-down), which genuinely
    // touches at a point/circle the axial-line bracketing does not apply to. cos(80 deg) ~= 0.17: only
    // a contact within ~10 deg of a true end-cap normal is excluded; an ordinary side/barrel contact
    // (normal within a few degrees of PERPENDICULAR to the axis) is well clear and gets bracketed.
    static AXIAL_LINE_MAX_AXIAL_NORMAL = 0.17;

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
        this._axis = new Vector3(); // _addAxialLineContacts scratch
        this._apex = new Vector3(); this._rim = new Vector3(); // _addConeSlantContacts scratch
        this._freshSet = []; // refreshManifoldGeometry's per-pair fresh contact set (primary + multi-point additions)
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
                // A curved shape (cylinder/capsule/cone) resting on its SIDE touches a flat surface
                // along a LINE, not a point - but GJK/EPA reports a single point. One point on a line
                // contact has a lever arm about the shape's own axis, so any per-substep correction at
                // it injects a torque that does not average out (traced: a resting cylinder's rotation
                // drifts, its residual spin never decays). Bracket the primary contact with real
                // additional points along the shape's own axis, so the manifold represents the true
                // line and the points' torques cancel by construction. Real surface geometry, each an
                // independent impulse-carrying contact (NOT snapshot 24's synthesized mirror).
                this._addAxialLineContacts(contact, primitivePairs[i].a, primitivePairs[i].b, list, margin);
                // A cone resting on its slant touches along the apex-to-down-rim edge. Add those two
                // real endpoints as their own contacts, each with its OWN true depth to the contact
                // plane - an endpoint lifted off the surface (e.g. the apex while the cone is still
                // tip-up mid-settle) gets a negative depth and is culled, so this only ever engages the
                // endpoints that are genuinely in contact, and does not fabricate a contact that would
                // fight the cone's settling into flush.
                this._addConeSlantContacts(contact, primitivePairs[i].a, primitivePairs[i].b, list, margin);
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
        // Fresh contacts here are throwaway (their values are copied into existing manifold points and
        // then discarded), so lease pool slots from the current index and restore it after - the tick's
        // own contacts, produced by step() before the solver ran, keep their slots untouched.
        const savedPoolIndex = this._poolIndex;
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            if (this._isCompoundOrMesh(bodyA.shape) || this._isCompoundOrMesh(bodyB.shape)) continue;

            const placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
            const placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };

            // Regenerate the FULL fresh contact set for this pair against the live transforms - the
            // primary GJK/EPA contact PLUS the same multi-point additions step() makes (axial-line
            // brackets for a barrel, apex/mid/rim for a cone slant). Refreshing only the single GJK
            // point starved a curved shape's OTHER points, leaving their geometry frozen at tick-start
            // so they could not do their job (a cone's apex could not track the settle and the tip
            // floated up). Each fresh point is matched to its nearest existing manifold point, so every
            // point gets live geometry, not just the one nearest the primary.
            this._freshSet.length = 0;
            const fresh = this._testPrimitivePair(placedA, placedB);
            this._freshSet.push(fresh);
            this._addAxialLineContacts(fresh, placedA, placedB, this._freshSet, Infinity);
            this._addConeSlantContacts(fresh, placedA, placedB, this._freshSet, Infinity);

            // Match each existing manifold point to its nearest fresh point (world space) and refresh
            // its live geometry in place, keeping warm-start lambda and anchor identity. Each fresh
            // point may serve as the nearest for at most the points closest to it; a fresh point with
            // no existing point near it is simply unused this substep (the manifold's once-per-tick
            // update() owns adding/removing points, not this method).
            for (let pi = 0; pi < manifold.points.length; pi++) {
                const p = manifold.points[pi];
                let best = null, bestDistSq = Infinity;
                for (let fi = 0; fi < this._freshSet.length; fi++) {
                    const fc = this._freshSet[fi];
                    const dx = p.point.x - fc.point.x, dy = p.point.y - fc.point.y, dz = p.point.z - fc.point.z;
                    const d = dx * dx + dy * dy + dz * dz;
                    if (d < bestDistSq) { bestDistSq = d; best = fc; }
                }
                if (!best) continue;
                p.point.copy(best.point);
                p.pointOnA.copy(best.pointOnA);
                p.pointOnB.copy(best.pointOnB);
                p.signedDistance = best.signedDistance;
                // Keep the ESTABLISHED normal through the exact-touch band, exactly as update() does -
                // else the ambiguous diagonal GJK/EPA returns at signed-distance ~0 clobbers a good
                // resting normal every substep (a flush sphere launched to y=200+ from that bug).
                if (Math.abs(best.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) p.normal.copy(best.normal);
                // Re-anchor to CURRENT geometry so C is measured from where the feature is NOW.
                p.setLocalAnchors(bodyA, bodyB);
            }
            this._poolIndex = savedPoolIndex; // release this pair's throwaway fresh contacts
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

    // Given the primary GJK/EPA contact for a pair, if exactly one side is a cylinder/capsule/cone
    // lying on its SIDE (barrel/slant contact, normal roughly perpendicular to that shape's own
    // axis), append REAL additional contact points spread along the shape's axis so the manifold
    // represents the true LINE of contact instead of a single point. Each appended point shares the
    // primary's normal and penetration (a constant-radius barrel touches a flat plane at the same
    // depth all along its length) and is offset in world space along the shape's axis; the offsets
    // are placed to bracket the primary symmetrically about the shape's own center, so the points'
    // lever arms about the axis cancel by construction. The manifold's 4-point cap and matching
    // absorb these exactly like any other points; each carries its own impulse state in the solver.
    _addAxialLineContacts(primary, placedA, placedB, list, margin) {
        // Identify the curved side and its axial half-extent (distance from center to each bracket
        // point along the axis). Cylinder/capsule: the barrel spans the full segment. Cone: the slant
        // runs tip (+halfHeight) to base rim (-halfHeight), so bracket across that same span.
        // Cylinder and capsule only: their contact line is the BARREL, which is parallel to the axis
        // AND centered on the shape's own center of mass, so symmetric bracket points along the axis
        // have equal-and-opposite lever arms that cancel by construction. A CONE is deliberately NOT
        // handled here: it rests on its SLANT (apex to base rim), a line that is neither axis-parallel
        // nor centered on the COM - the apex and rim endpoints sit at different distances from the
        // center, so symmetric-offset points do not cancel (tried: the cone's residual spin was
        // unchanged). The cone needs its two real endpoints (apex, rim) placed asymmetrically, which
        // is its own geometry - a separate step, not this barrel model forced to fit.
        let curved = null, halfSpan = 0;
        const a = placedA.shape, b = placedB.shape;
        if (a.type === 'cylinder') { curved = placedA; halfSpan = a.halfHeight; }
        else if (a.type === 'capsule') { curved = placedA; halfSpan = a.segmentHalfLength; }
        else if (b.type === 'cylinder') { curved = placedB; halfSpan = b.halfHeight; }
        else if (b.type === 'capsule') { curved = placedB; halfSpan = b.segmentHalfLength; }
        if (!curved || halfSpan <= 0) return;

        // World-space axis of the curved shape (its local +Y).
        this._axis.set(0, 1, 0);
        curved.rotation.transformVectorInPlace(this._axis);
        const nx = primary.normal.x, ny = primary.normal.y, nz = primary.normal.z;
        const axialDot = Math.abs(this._axis.x * nx + this._axis.y * ny + this._axis.z * nz);
        if (axialDot > NarrowPhase.AXIAL_LINE_MAX_AXIAL_NORMAL) return; // end-cap contact, not a side line

        // Project the axis into the contact PLANE (remove its normal component) and renormalize - the
        // bracket points must move along the contact line, which lies in the plane, not off it into
        // or out of the surface. For a shape resting flush on its side the axis is already in-plane;
        // this only matters for a slight tilt, and keeps the appended points ON the surface.
        const axisDotN = this._axis.x * nx + this._axis.y * ny + this._axis.z * nz;
        let px = this._axis.x - axisDotN * nx, py = this._axis.y - axisDotN * ny, pz = this._axis.z - axisDotN * nz;
        const plen = Math.sqrt(px * px + py * py + pz * pz);
        if (plen < 1e-6) return; // axis is along the normal (end-cap); nothing to spread
        px /= plen; py /= plen; pz /= plen;

        // Two bracket points, at +/- halfSpan from the primary along the in-plane axis. They share the
        // primary's normal and signedDistance (constant-radius barrel). pointOnA/pointOnB and point
        // are the primary's, shifted by the same axial offset, so their body-local anchors (set by the
        // manifold) land at the true bracketed surface positions.
        for (let s = -1; s <= 1; s += 2) {
            const off = s * halfSpan;
            const extra = this._nextPooledContact();
            extra.normal.set(nx, ny, nz);
            extra.signedDistance = primary.signedDistance;
            extra.point.set(primary.point.x + px * off, primary.point.y + py * off, primary.point.z + pz * off);
            extra.pointOnA.set(primary.pointOnA.x + px * off, primary.pointOnA.y + py * off, primary.pointOnA.z + pz * off);
            extra.pointOnB.set(primary.pointOnB.x + px * off, primary.pointOnB.y + py * off, primary.pointOnB.z + pz * off);
            if (extra.signedDistance >= -margin) list.push(extra);
        }
    }

    // A cone resting on its slant touches the surface along the apex-to-down-rim edge. Those two
    // endpoints - apex on the axis at local (0,+hh,0), and the down-side base-rim point - are the true
    // contact line. Unlike a cylinder's barrel they are asymmetric about the COM, so the axial
    // bracketing does not apply; they are added here at their real positions, each with its own depth.
    _addConeSlantContacts(primary, placedA, placedB, list, margin) {
        let cone = null;
        if (placedA.shape.type === 'cone') cone = placedA;
        else if (placedB.shape.type === 'cone') cone = placedB;
        if (!cone) return;
        const R = cone.shape.radius, hh = cone.shape.halfHeight;

        // Slant contact only. A cone resting flush on its slant has its axis tilted UP from horizontal
        // by exactly its own half-angle (atan(R / 2*hh)), so axis . normal = sin(halfAngle) at rest -
        // NOT ~0 the way a cylinder's horizontal axis is. Gating a cone with the cylinder's
        // near-perpendicular threshold wrongly rejects it (a fat cone's sin(halfAngle) exceeds 0.17).
        // The correct discriminant is: is this the SLANT (axis . normal ~ +sin(halfAngle), apex up) or
        // an END CAP (base-down: axis . normal ~ +1; tip-down: ~ -1)? Accept a band around the slant
        // value and reject the caps.
        this._axis.set(0, 1, 0);
        cone.rotation.transformVectorInPlace(this._axis);
        const nx = primary.normal.x, ny = primary.normal.y, nz = primary.normal.z;
        const axisDotN = this._axis.x * nx + this._axis.y * ny + this._axis.z * nz;
        const halfAngle = Scalar.atan2(R, 2 * hh);
        const slantDot = Scalar.sin(halfAngle); // |axis . normal| when resting flush on the slant
        // Reject unless within a tolerance of the slant orientation (excludes both end caps, where
        // |axisDotN| ~ 1, and a barrel-like near-0 which a cone does not have).
        if (Math.abs(Math.abs(axisDotN) - slantDot) > 0.25) return;

        // Down-side rim direction: -normal brought into the cone's local frame, projected off the axis
        // onto the base plane (local x/z). That is the base-circle direction whose rim point is lowest
        // against the contact normal - the one that actually touches.
        this._rim.set(-nx, -ny, -nz);
        NarrowPhase._invRot.copy(cone.rotation).invert().transformVectorInPlace(this._rim);
        let dx = this._rim.x, dz = this._rim.z;
        const dlen = Math.sqrt(dx * dx + dz * dz);
        if (dlen < 1e-6) return; // -normal along the axis: end-cap, not a slant
        dx /= dlen; dz /= dlen;

        // Apex and down-rim in world space.
        this._apex.set(0, hh, 0);
        cone.rotation.transformVectorInPlace(this._apex); this._apex.addInPlace(cone.position);
        this._rim.set(dx * R, -hh, dz * R);
        cone.rotation.transformVectorInPlace(this._rim); this._rim.addInPlace(cone.position);

        this._addConeEndpoint(this._apex, primary, margin, list);
        this._addConeEndpoint(this._rim, primary, margin, list);
    }

    // Adds one cone endpoint as a contact, with its OWN depth relative to the primary contact plane.
    // The plane passes through primary.point with the primary normal (which points from B to A). A
    // point DEEPER on the penetrating side than the primary has a larger signed distance; a point
    // lifted off the surface has a smaller (eventually negative) one and is culled past the margin -
    // which is exactly what keeps a not-yet-flush cone's airborne apex from being a fabricated contact.
    _addConeEndpoint(worldPoint, primary, margin, list) {
        const nx = primary.normal.x, ny = primary.normal.y, nz = primary.normal.z;
        const ox = worldPoint.x - primary.point.x, oy = worldPoint.y - primary.point.y, oz = worldPoint.z - primary.point.z;
        // normal points B->A (out of the flat body toward the cone); a point further along +normal is
        // LESS penetrating, so depth relative to the primary is +(offset . normal) below... verified
        // against the flush cone: normal is (0,-1,0), a lower endpoint (offset.y<0) must read DEEPER,
        // and (offset . normal) = -offset.y > 0 for that endpoint - so this sign is correct.
        const deeperBy = ox * nx + oy * ny + oz * nz;
        const sd = primary.signedDistance + deeperBy;
        if (sd < -margin) return; // lifted off past the speculative margin - not a contact
        const extra = this._nextPooledContact();
        extra.normal.set(nx, ny, nz);
        extra.signedDistance = sd;
        extra.point.copy(worldPoint);
        extra.pointOnA.copy(worldPoint);
        extra.pointOnB.copy(worldPoint);
        list.push(extra);
    }
}

NarrowPhase._invRot = new Quaternion();

ActionPhysics.NarrowPhase = NarrowPhase;
