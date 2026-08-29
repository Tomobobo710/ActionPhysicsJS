// Per-tick pair dispatch and GJK/EPA testing for one primitive-shape pair.
var proto = NarrowPhase.prototype;

proto._nextPooledContact = function () {
    if (this._poolIndex >= this._contactPool.length) this._contactPool.push(new ContactDetails());
    const c = this._contactPool[this._poolIndex++];
    c.fromMeshFace = false;
    return c;
};

// midphase expands compound/mesh pairs to primitives; dt sizes the speculative margin.
proto.step = function (broadphasePairs, midphase, dt) {
    if (dt) this._dt = dt;
    this._midphase = midphase; // used by the per-substep mesh-face refresh
    this._poolIndex = 0;
    const contactsByPair = new Map(); // canonical "idA:idB" key -> ContactDetails[]

    for (let p = 0; p < broadphasePairs.length; p++) {
        const bodyA = broadphasePairs[p][0], bodyB = broadphasePairs[p][1];
        const primitivePairs = midphase.expandPair(bodyA, bodyB);
        const key = bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
        const margin = this._speculativeMargin(bodyA, bodyB);

        let sawMeshFace = false;
        for (let i = 0; i < primitivePairs.length; i++) {
            const pairContacts = this._testPrimitivePair(primitivePairs[i].a, primitivePairs[i].b);
            for (let c = 0; c < pairContacts.length; c++) {
                const contact = pairContacts[c];
                if (contact.signedDistance < -margin) continue; // gap beyond the speculative margin
                if (contact.fromMeshFace) sawMeshFace = true;
                let list = contactsByPair.get(key);
                if (!list) { list = []; contactsByPair.set(key, list); }
                list.push(contact);
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

    if (ConvexTri.applies(placedA, placedB)) {
        const self = this;
        // null = not a face contact (convex off the triangle, or gap outside the band); fall
        // through to GJK/EPA (same contract as BoxBox.test / TriTri.test above).
        const ctResult = ConvexTri.test(placedA, placedB, results, function () { return self._nextPooledContact(); });
        if (ctResult !== null) return results;
    }

    const contact = this._nextPooledContact();
    const support = this._support.setSides(placedA, placedB);
    const gjkResult = this._gjk.run(support);
    if (gjkResult.overlapping) {
        const epaResult = this._epa.run(support, gjkResult.simplex);
        contact.setFromEPA(epaResult);
    } else {
        contact.setFromGJKSeparated(gjkResult);
    }
    results.push(contact);
    return results;
};

proto._isCompoundOrMesh = function (shape) {
    return (typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape) ||
        (typeof MeshShape !== 'undefined' && shape instanceof MeshShape);
};
