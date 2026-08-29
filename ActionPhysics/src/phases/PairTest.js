// Per-tick pair dispatch and GJK/EPA testing for one primitive-shape pair.
var proto = NarrowPhase.prototype;

proto._nextPooledContact = function () {
    if (this._poolIndex >= this._contactPool.length) this._contactPool.push(new ContactDetails());
    return this._contactPool[this._poolIndex++];
};

// broadphasePairs: [[bodyA, bodyB], ...]. midphase expands compound/mesh pairs to primitives.
// dt sizes the per-pair speculative margin; optional, falls back to the last value / 1/60.
proto.step = function (broadphasePairs, midphase, dt) {
    if (dt) this._dt = dt;
    this._poolIndex = 0;
    const contactsByPair = new Map(); // canonical "idA:idB" key -> ContactDetails[]

    for (let p = 0; p < broadphasePairs.length; p++) {
        const bodyA = broadphasePairs[p][0], bodyB = broadphasePairs[p][1];
        const primitivePairs = midphase.expandPair(bodyA, bodyB);
        const key = bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
        const margin = this._speculativeMargin(bodyA, bodyB);

        for (let i = 0; i < primitivePairs.length; i++) {
            const pairContacts = this._testPrimitivePair(primitivePairs[i].a, primitivePairs[i].b);
            for (let c = 0; c < pairContacts.length; c++) {
                const contact = pairContacts[c];
                // signedDistance: positive = overlapping, negative = separated by that gap. Report
                // while overlapping or within the speculative margin; drop once the gap exceeds it.
                if (contact.signedDistance < -margin) continue;
                let list = contactsByPair.get(key);
                if (!list) { list = []; contactsByPair.set(key, list); }
                list.push(contact);
            }
        }

        // Ensure a manifold exists even with zero contacts this tick, so refresh() below can
        // prune a pair that just separated. Idempotent for an existing pair.
        this.manifolds.getOrCreate(bodyA, bodyB);
    }

    this.manifolds.refresh(contactsByPair, this._dt);
    return this.manifolds;
};

// One or more pooled ContactDetails for one primitive pair, as an array (reused scratch array -
// copy out before the next call). Dispatches to a closed-form test when one applies (own
// numerics, no shared epsilon/iteration budget with any other pair type); falls through to
// GJK/EPA otherwise. Never culls - the caller (step) decides what's worth a manifold entry.
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
        // null = boxes are actually separated - SAT's per-axis numbers aren't a true closest-point
        // witness once apart, so fall through to GJK/EPA below, same as any pair with no
        // closed-form test.
        const boxResult = BoxBox.test(placedA, placedB, results, function () { return self._nextPooledContact(); });
        if (boxResult !== null) return results;
    }

    const contact = this._nextPooledContact();
    const support = new MinkowskiSupport(placedA, placedB);
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
