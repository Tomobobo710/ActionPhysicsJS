/**
 * ContactManifold: the persistent contact state for one pair of primitive shapes, across ticks.
 *
 * Owns point lifetime ENTIRELY (plan.md, component 5 and Rule 1/2). Narrowphase (via update())
 * only ever ADDS or REFRESHES points from this tick's GJK/EPA result; only the manifold itself
 * REMOVES a point, and only between ticks (never mid-substep - see the bug reference below).
 * Everywhere else assumes a manifold's point set is stable for the duration of a tick.
 *
 * BUG FIX CARRIED FROM THE PREDECESSOR (plan.md, Bug reference / Contact management):
 * "Refreshing manifolds mid-tick emptied them." Re-running a staleness cull once per SUBSTEP
 * retired points that were merely mid-correction - not actually separated, just still being
 * resolved by the solver's own position projection within the same tick. 38 of 1210 manifolds
 * emptied, dropping bodies onto their neighbours. The fix here is structural: update() (called
 * once per TICK by narrowphase, never per substep) is the only place points are added or pruned.
 * The solver, wherever it substeps within a tick, reads and writes lambda/geometry on the SAME
 * point objects without ever adding, removing, or re-matching them mid-tick.
 *
 * PERSISTENCE / WARM-START: a manifold holds up to 4 points (MAX_POINTS). Each update() call
 * matches this tick's narrowphase result against the existing points (by proximity in LOCAL space
 * relative to body A - world position drifts as A moves, but a contact feature's position
 * relative to A's own frame stays close between ticks unless the contact point itself is sliding).
 * A match copies the new geometry (point/normal/signedDistance) onto the EXISTING point object,
 * preserving its accumulated lambda for the solver's warm start; no match adds a new point (via
 * the 4-point reduction below if already full).
 */
class ContactManifold {
    static MAX_POINTS = 4;
    // A matched point's local-space (body-A-relative) position must stay within this distance of
    // where it was last tick to count as "the same contact" rather than a new one. Chosen as a
    // fraction of a typical contact's own scale rather than an absolute constant - see update()'s
    // matching call for how this get scaled by the manifold's own point spread.
    static MATCH_DISTANCE = 0.05;
    // Signed-distance half-width of the exact-touch band where GJK/EPA's normal is treated as
    // ambiguous and a warm-matched point keeps its established normal instead (see update()). Sized
    // a little above the numerical noise of a flush contact, well below any real overlap depth the
    // solver needs to resolve - inside this band the shapes are touching to within a fraction of a
    // millimetre and the normal genuinely cannot be recovered reliably from a single query.
    static EXACT_TOUCH_BAND = 0.001;

    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.points = []; // ContactDetails[], length 0..MAX_POINTS
        // Local-space (relative to bodyA's CURRENT transform at match time) anchor for each point,
        // parallel to `points` - used only for next-tick matching, recomputed every update().
        this._localAnchors = [];
    }

    get pointCount() { return this.points.length; }

    // Called once per TICK (never per substep - see the class header). `newContacts` is this
    // tick's narrowphase result for this body pair: an array of ContactDetails, typically length 1
    // (one primitive pair -> one GJK/EPA contact) but the manifold accepts any count so a caller
    // batching multiple sub-contacts (e.g. a multi-triangle mesh region) works the same way.
    //
    // Points not re-confirmed this tick (no incoming contact matched them, or the match exceeded
    // MATCH_DISTANCE, or signedDistance separated past REMOVE_DISTANCE) are removed HERE - this is
    // the manifold's one removal path, and it only ever runs from this method.
    // Emits contact lifecycle events on both bodies as this update() call changes point state.
    //   speculativeContact: a brand-new point this tick, while still separated (signedDistance < 0).
    //       A listener on either body returning false vetoes the point outright (removed before the
    //       solver ever sees it) - the one place a listener can affect physics, matching the
    //       documented event-prevention concept.
    //   contact: a brand-new point this tick that is already overlapping (signedDistance >= 0), or a
    //       previously-speculative point that becomes overlapping on a later tick's refresh.
    //   endContact: an existing point removed this tick (separated past match/removal, per-point).
    //   endAllContact: fired once per body when this call empties the manifold to zero points after
    //       having had at least one before.
    update(newContacts) {
        const hadPointsBefore = this.points.length > 0;
        const matched = new Array(newContacts.length).fill(false);

        // Match each existing point against the best (closest, in bodyA-local space) unmatched
        // incoming contact. A match refreshes the existing point's geometry in place, keeping its
        // accumulated lambda - this IS the warm start.
        for (let i = this.points.length - 1; i >= 0; i--) {
            const existing = this.points[i];
            const existingLocal = this._localAnchors[i];
            let bestJ = -1, bestDistSq = ContactManifold.MATCH_DISTANCE * ContactManifold.MATCH_DISTANCE;
            for (let j = 0; j < newContacts.length; j++) {
                if (matched[j]) continue;
                const localCandidate = ContactManifold._toLocal(this.bodyA, newContacts[j].pointOnA);
                const dx = localCandidate.x - existingLocal.x, dy = localCandidate.y - existingLocal.y, dz = localCandidate.z - existingLocal.z;
                const distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < bestDistSq) { bestDistSq = distSq; bestJ = j; }
            }
            if (bestJ === -1) {
                // Not re-confirmed this tick: remove. This is the ONLY place a point is removed -
                // never mid-substep, never from a separate staleness sweep (plan.md, Bug
                // reference). A point that genuinely separated simply stops being reported by
                // narrowphase and is pruned here, on the very next tick's update() call.
                this.points.splice(i, 1);
                this._localAnchors.splice(i, 1);
                this._emitBoth('endContact', existing);
                continue;
            }
            matched[bestJ] = true;
            // Save the accumulated lambda BEFORE copy() overwrites it - copy() pulls every field
            // from newContacts[bestJ], whose lambda fields are always zero (a fresh ContactDetails
            // narrowphase just produced this tick, with no solver history of its own). Losing this
            // ordering was an early, self-inflicted version of this bug: `existing` and
            // `this.points[i]` are the SAME object, so reading "the prior value" AFTER copy() just
            // reads back the zero that was already written - this is why the values are captured
            // into locals first.
            const keepNormalLambda = existing.normalLambda;
            const keepTangentLambda1 = existing.tangentLambda1;
            const keepTangentLambda2 = existing.tangentLambda2;
            // Preserve the ESTABLISHED contact normal across an exact-touch refresh. At a signed
            // distance within EXACT_TOUCH_BAND of zero (shapes touching flush, neither clearly
            // separated nor clearly overlapping), GJK/EPA's normal is genuinely ambiguous - the
            // origin sits ON the Minkowski-difference boundary, so the recovered direction can flip
            // to a diagonal face normal (a box resting flush reports (0.707,0,0.707) instead of the
            // true (0,1,0) for one tick). A persistent contact's normal does NOT actually change
            // tick to tick, so trusting a single ambiguous tick's normal over the one this point
            // has carried while it was unambiguously resolving is the wrong call: it makes the
            // constraint briefly point sideways, the body sinks through, and the next (recovered)
            // tick ejects it back out - a permanent penetrate-then-launch limit cycle. This is the
            // manifold owning contact identity across ticks (its documented job), not a solver-side
            // governor. Outside the band (a real gap or a real overlap), the fresh normal is
            // trustworthy and is taken as-is.
            const keepNormal = Math.abs(newContacts[bestJ].signedDistance) < ContactManifold.EXACT_TOUCH_BAND
                ? ContactManifold._scratchNormal.copy(existing.normal)
                : null;
            const wasOverlapping = existing.signedDistance >= 0;
            existing.copy(newContacts[bestJ]); // geometry refreshed
            existing.normalLambda = keepNormalLambda; // warm start restored
            existing.tangentLambda1 = keepTangentLambda1;
            existing.tangentLambda2 = keepTangentLambda2;
            if (keepNormal) existing.normal.copy(keepNormal); // established normal kept through the ambiguous band
            this._localAnchors[i] = ContactManifold._toLocal(this.bodyA, existing.pointOnA);
            if (!wasOverlapping && existing.signedDistance >= 0) this._emitBoth('contact', existing);
        }

        // Any incoming contact not matched to an existing point is genuinely new.
        for (let j = 0; j < newContacts.length; j++) {
            if (matched[j]) continue;
            if (newContacts[j].signedDistance < 0) {
                if (!this._speculativeAllowed(newContacts[j])) continue; // vetoed by a listener
                this._addPoint(newContacts[j]);
                this._emitBoth('speculativeContact', newContacts[j]);
            } else {
                this._addPoint(newContacts[j]);
                this._emitBoth('contact', newContacts[j]);
            }
        }

        if (hadPointsBefore && this.points.length === 0) this._emitBoth('endAllContact', null);
    }

    // A speculativeContact listener on either body may veto the point by returning false. Fired
    // BEFORE the point is added, so a veto means the point never enters the manifold at all.
    _speculativeAllowed(contact) {
        return this.bodyA._speculativeVeto(contact, this.bodyB) !== false &&
            this.bodyB._speculativeVeto(contact, this.bodyA) !== false;
    }

    _emitBoth(event, contact) {
        this.bodyA.emit(event, { contact: contact, other: this.bodyB });
        this.bodyB.emit(event, { contact: contact, other: this.bodyA });
    }

    _addPoint(contact) {
        const point = contact.clone();
        point.normalLambda = 0; point.tangentLambda1 = 0; point.tangentLambda2 = 0; // fresh point: no warm-start data yet
        // Local anchors are set ONCE here, at creation - see ContactDetails.setLocalAnchors and
        // Solver.js's class header for why the solver needs these (recomputing the contact's
        // CURRENT gap every substep) rather than reusing the single signedDistance this tick's
        // narrowphase pass measured before any substep moved the bodies.
        point.setLocalAnchors(this.bodyA, this.bodyB);
        const local = ContactManifold._toLocal(this.bodyA, point.pointOnA);

        if (this.points.length < ContactManifold.MAX_POINTS) {
            this.points.push(point);
            this._localAnchors.push(local);
            return;
        }

        // Already at the cap: reduce. Standard 4-point manifold reduction (Bullet, Box2D use the
        // same idea) - always KEEP the deepest point (it matters most for the solver), and among
        // the remaining candidates (the new point plus the 3 non-deepest existing ones) keep
        // whichever 3 form the LARGEST-AREA quadrilateral with the deepest point. Maximizing area
        // keeps the manifold spread out (good torque resistance - a box resting on a corner-only
        // manifold rocks; a box resting on 4 spread corners doesn't), rather than collapsing onto
        // whichever points happen to be deepest overall.
        this._reduceToFour(point, local);
    }

    _reduceToFour(candidatePoint, candidateLocal) {
        // Find the deepest point among the 4 existing + the candidate (deepest = largest
        // signedDistance, i.e. most overlapping - the point the solver most needs to resolve).
        let deepestIdx = -1, deepestVal = candidatePoint.signedDistance;
        for (let i = 0; i < this.points.length; i++) {
            if (this.points[i].signedDistance > deepestVal) { deepestVal = this.points[i].signedDistance; deepestIdx = i; }
        }
        const deepestIsCandidate = deepestIdx === -1;
        const deepestPoint = deepestIsCandidate ? candidatePoint : this.points[deepestIdx];

        // Candidate set: every point EXCEPT the deepest (which is locked in), evaluated by which
        // combination of 3 maximizes the quadrilateral area with the deepest point as the 4th
        // corner. With exactly 4 existing + 1 candidate - 1 deepest = 4 remaining candidates for 3
        // slots, there are exactly 4 possible triples (each omitting one candidate) - enumerate
        // all 4 directly rather than a general combinatorial search.
        const pool = [];
        for (let i = 0; i < this.points.length; i++) if (i !== deepestIdx) pool.push({ point: this.points[i], local: this._localAnchors[i] });
        if (!deepestIsCandidate) pool.push({ point: candidatePoint, local: candidateLocal });
        // pool now has exactly 4 entries (3 existing non-deepest + the candidate, when the
        // candidate isn't itself deepest) - or 4 existing non-deepest entries (when the candidate
        // IS deepest, so all 4 existing points are "remaining" and the candidate is locked in).

        let bestOmit = 0, bestArea = -1;
        for (let omit = 0; omit < pool.length; omit++) {
            const tri = [];
            for (let i = 0; i < pool.length; i++) if (i !== omit) tri.push(pool[i]);
            const area = ContactManifold._quadArea(deepestPoint.point, tri[0].point.point, tri[1].point.point, tri[2].point.point);
            if (area > bestArea) { bestArea = area; bestOmit = omit; }
        }

        const kept = [];
        for (let i = 0; i < pool.length; i++) if (i !== bestOmit) kept.push(pool[i]);

        this.points = deepestIsCandidate ? [candidatePoint] : [deepestPoint];
        this._localAnchors = deepestIsCandidate ? [candidateLocal] : [this._localAnchors[deepestIdx]];
        for (let i = 0; i < kept.length; i++) { this.points.push(kept[i].point); this._localAnchors.push(kept[i].local); }
    }

    // Rough quadrilateral area for the 4 candidate corners (order doesn't need to be a proper
    // convex hull walk here - the sum of the two diagonal-split triangle areas is a fine proxy for
    // "how spread out is this point set", which is all the reduction heuristic needs).
    static _quadArea(a, b, c, d) {
        return ContactManifold._triArea(a, b, c) + ContactManifold._triArea(a, c, d);
    }

    static _triArea(a, b, c) {
        const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
        const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
        const cx = aby * acz - abz * acy, cy = abz * acx - abx * acz, cz = abx * acy - aby * acx;
        return 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
    }

    // World point -> bodyA-local space, for next-tick matching. Allocation kept minimal (one
    // Vector3 per call) - matching runs once per tick per manifold, not in a hot per-substep loop.
    static _toLocal(bodyA, worldPoint) {
        const rel = Vector3.subInto(new Vector3(), worldPoint, bodyA.position);
        const invRot = new Quaternion().copy(bodyA.rotation).invert();
        invRot.transformVectorInPlace(rel);
        return rel;
    }
}

ContactManifold._scratchNormal = new Vector3();

ActionPhysics.ContactManifold = ContactManifold;
