// Per-tick pair dispatch and GJK/EPA testing for one primitive-shape pair.
var proto = NarrowPhase.prototype;

proto._nextPooledContact = function () {
    if (this._poolIndex >= this._contactPool.length) this._contactPool.push(new ContactDetails());
    const c = this._contactPool[this._poolIndex++];
    c.fromMeshFace = false;
    c.meshTriValid = false;
    return c;
};

// midphase expands compound/mesh pairs to primitives; dt sizes the speculative margin.
proto.step = function (broadphasePairs, midphase, dt) {
    if (dt) this._dt = dt;
    this._midphase = midphase; // used by the per-substep mesh-face refresh
    this._poolIndex = 0;
    const contactsByPair = new Map(); // canonical "idA:idB" key -> ContactDetails[]

    // Tick-start speeds, consulted by the per-substep geometry refresh to skip re-clipping a
    // manifold whose bodies are effectively at rest (their contact geometry is not moving within
    // the tick, so a re-clip would reproduce the tick-start geometry anyway).
    const spd = this._tickStartSpeedSq || (this._tickStartSpeedSq = new Map());
    spd.clear();
    for (let p = 0; p < broadphasePairs.length; p++) {
        for (let s = 0; s < 2; s++) {
            const b = broadphasePairs[p][s];
            if (b.bodyType !== RigidBody.DYNAMIC || spd.has(b.id)) continue;
            const lv = b.linear_velocity, av = b.angular_velocity;
            spd.set(b.id, {
                lin: lv.x * lv.x + lv.y * lv.y + lv.z * lv.z,
                ang: av.x * av.x + av.y * av.y + av.z * av.z
            });
        }
    }

    for (let p = 0; p < broadphasePairs.length; p++) {
        const bodyA = broadphasePairs[p][0], bodyB = broadphasePairs[p][1];
        const sides = midphase.expandPairSides(bodyA, bodyB);
        const sidesA = sides.a, sidesB = sides.b;
        const key = bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
        const margin = this._speculativeMargin(bodyA, bodyB);

        // If this pair already has a mesh-face manifold, hand ConvexTri its established normal so
        // it orients the reference face by that instead of the convex-centre heuristic (which
        // flips once a settling convex's centroid creeps to the triangle plane). First contact
        // has no prior manifold and falls back to the heuristic, which is safe when approaching
        // from clearly outside.
        const existing = this.manifolds._manifolds.get(key);
        this._ctHintNormal = (existing && existing.points.length > 0 && existing.points[0].fromMeshFace)
            ? existing.points[0].normal : null;
        // Contacts past this gap are discarded below, so closed-form tests can use it to skip the
        // GJK/EPA fallback. Cleared after the loop; the refresh path has no pair margin.
        this._curMargin = margin;

        let sawMeshFace = false;
        for (let i = 0; i < sidesA.length; i++) {
            for (let j = 0; j < sidesB.length; j++) {
                const pairContacts = this._testPrimitivePair(sidesA[i], sidesB[j]);
                for (let c = 0; c < pairContacts.length; c++) {
                    const contact = pairContacts[c];
                    if (contact.signedDistance < -margin) continue; // gap beyond the speculative margin
                    if (contact.fromMeshFace) sawMeshFace = true;
                    let list = contactsByPair.get(key);
                    if (!list) { list = []; contactsByPair.set(key, list); }
                    list.push(contact);
                }
            }
        }

        // A TriTri face manifold is authoritative for the pair; drop the GJK/EPA single points from
        // its other triangle combinations, which would only unbalance the point set.
        if (sawMeshFace) {
            const list = contactsByPair.get(key);
            let w = 0;
            for (let r = 0; r < list.length; r++) if (list[r].fromMeshFace) list[w++] = list[r];
            list.length = w;
        }

        // Ensure a manifold exists even with zero contacts, so refresh() can prune a separated pair.
        this.manifolds.getOrCreate(bodyA, bodyB);
    }
    this._curMargin = null;

    this.manifolds.refresh(contactsByPair, this._dt);
    return this.manifolds;
};

// Contacts for one primitive pair, into a reused scratch array (copy out before the next call).
// Uses a closed-form test when one applies, else GJK/EPA. Never culls.
proto._testPrimitivePair = function (placedA, placedB) {
    const results = this._pairResultScratch;
    results.length = 0;

    if (SphereSphere.applies(placedA, placedB)) {
        results.push(SphereSphere.test(placedA, placedB, this._nextPooledContact()));
        return results;
    }
    if (SphereBox.applies(placedA, placedB)) {
        results.push(SphereBox.test(placedA, placedB, this._nextPooledContact()));
        return results;
    }
    if (BoxBox.applies(placedA, placedB)) {
        const self = this;
        // null = separated; fall through to GJK/EPA.
        const boxResult = BoxBox.test(placedA, placedB, results, function () { return self._nextPooledContact(); });
        if (boxResult !== null) return results;
    }

    if (TriTri.applies(placedA, placedB)) {
        const self = this;
        // null = not a face pair; fall through to GJK/EPA (same contract as BoxBox.test above).
        const triResult = TriTri.test(placedA, placedB, results, function () { return self._nextPooledContact(); });
        if (triResult !== null) return results;
    }

    if (BoxTriFace.applies(placedA, placedB)) {
        const self = this;
        // Face patch when the box lies flat on the triangle; null = not a face case, keep going.
        const bf = BoxTriFace.test(placedA, placedB, results, function () { return self._nextPooledContact(); }, this._ctHintNormal);
        if (bf !== null) return results;
    }

    if (ConvexTri.applies(placedA, placedB)) {
        const self = this;
        // _ctHintNormal is set per pair by step() from the existing manifold, null on first contact.
        const ctResult = ConvexTri.test(placedA, placedB, results, function () { return self._nextPooledContact(); }, this._ctHintNormal, this._curMargin);
        if (ctResult !== null) return results;                       // face contact
        if (ConvexTri.lastVerdict === 'separated') return results;   // provably no contact - skip GJK/EPA
        // 'maybe': non-face contact still possible (edge/vertex) - fall through to GJK/EPA.
    } else if ((placedA.shape instanceof TriangleShape) !== (placedB.shape instanceof TriangleShape)) {
        // Any other convex (box, hull) vs a mesh triangle: a cheap conservative separation test
        // before GJK/EPA. A prop's broadphase AABB overlaps every tile it is near, but it only
        // touches one - GJK would run full iterations just to report "separated" on the rest. The
        // 0.5m bound comfortably exceeds any per-pair speculative margin at these speeds, so this
        // only rejects pairs step() would discard anyway.
        if (TriPlaneCull.separated(placedA, placedB)) return results;
    }

    const contact = this._nextPooledContact();
    const support = this._support.setSides(placedA, placedB);
    const gjkResult = this._gjk.run(support);
    if (gjkResult.overlapping) {
        const epaResult = this._epa.run(support, gjkResult.simplex);
        contact.setFromEPA(epaResult);
        // A penetration depth larger than the smaller shape's extent is a degenerate EPA result;
        // treat it as separated by that distance.
        if (contact.signedDistance > NarrowPhase._maxPlausiblePenetration(placedA.shape, placedB.shape)) {
            contact.setFromGJKSeparated({
                distance: contact.signedDistance,
                normal: contact.normal,
                pointA: contact.pointOnA,
                pointB: contact.pointOnB,
            });
        }
    } else {
        contact.setFromGJKSeparated(gjkResult);
    }
    results.push(contact);
    return results;
};

// The smaller shape's bounding-sphere radius; an EPA depth past this is rejected, never accepted.
NarrowPhase._maxPlausiblePenetration = function (shapeA, shapeB) {
    return Math.min(NarrowPhase._boundingRadius(shapeA), NarrowPhase._boundingRadius(shapeB));
};

NarrowPhase._boundingRadius = function (shape) {
    const aabb = NarrowPhase._brAABB || (NarrowPhase._brAABB = new AABB());
    shape.localAABBInto(aabb);
    const ex = Math.max(Math.abs(aabb.min.x), Math.abs(aabb.max.x));
    const ey = Math.max(Math.abs(aabb.min.y), Math.abs(aabb.max.y));
    const ez = Math.max(Math.abs(aabb.min.z), Math.abs(aabb.max.z));
    return Math.sqrt(ex * ex + ey * ey + ez * ez);
};

proto._isCompoundOrMesh = function (shape) {
    return (typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape) ||
        (typeof MeshShape !== 'undefined' && shape instanceof MeshShape);
};
