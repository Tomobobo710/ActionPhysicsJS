/**
 * ContactManifoldList: the full set of active ContactManifolds, keyed by body pair.
 *
 * One manifold per (bodyA, bodyB) pair — a pair with multiple candidate primitive contacts
 * (e.g. a compound body touching another shape at two of its children) accumulates all of THAT
 * tick's contacts into the SAME manifold via update(), since the manifold's own 4-point cap and
 * matching already do the right thing with several new points at once.
 *
 * Same ownership discipline as ContactManifold itself: refresh() is called once per TICK by
 * narrowphase, never per substep. A manifold that ends the tick with zero points (nothing matched,
 * nothing new) is removed from the list here — this is the ONE place a manifold itself is retired,
 * mirroring ContactManifold's own "only update() removes a point" rule one level up.
 */
class ContactManifoldList {
    constructor() {
        this._manifolds = new Map(); // "idA:idB" (idA < idB) -> ContactManifold
    }

    static _key(bodyA, bodyB) {
        return bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
    }

    // Returns the existing manifold for (bodyA, bodyB), creating one if this is a new pair. The
    // returned manifold's bodyA/bodyB are stored in a CANONICAL order (lower id first) so a
    // pair's local-space matching anchor (ContactManifold._toLocal uses bodyA) stays consistent
    // regardless of which order a caller happens to pass the two bodies in from tick to tick.
    getOrCreate(bodyA, bodyB) {
        const key = ContactManifoldList._key(bodyA, bodyB);
        let m = this._manifolds.get(key);
        if (!m) {
            const first = bodyA.id < bodyB.id ? bodyA : bodyB;
            const second = bodyA.id < bodyB.id ? bodyB : bodyA;
            m = new ContactManifold(first, second);
            this._manifolds.set(key, m);
        }
        return m;
    }

    // Applies this tick's contacts (grouped by body pair) to their manifolds, then drops any
    // manifold left with zero points. `contactsByPair` is a Map from "idA:idB" key (matching
    // _key's own canonical ordering) to an array of ContactDetails for that pair this tick. A pair
    // with a manifold but no entry in `contactsByPair` this tick (nothing detected at all) is
    // treated the same as an entry with an empty array - both result in every existing point
    // failing to match and the manifold being pruned.
    refresh(contactsByPair) {
        for (const [key, manifold] of this._manifolds) {
            const contacts = contactsByPair.get(key) || [];
            manifold.update(contacts);
            if (manifold.pointCount === 0) this._manifolds.delete(key);
        }
        // New pairs (a key present in contactsByPair but with no manifold yet) are created by the
        // caller via getOrCreate() before calling refresh() - see Narrowphase's own dispatch loop,
        // which must look up/create the manifold to know where to route each contact in the first
        // place. refresh() only ever prunes and updates EXISTING manifolds; getOrCreate() is the
        // sole entry point for new ones, keeping "one owner" for manifold creation too.
    }

    // All manifolds with at least one point, for the solver to iterate.
    values() {
        return this._manifolds.values();
    }

    get size() { return this._manifolds.size; }
}

ActionPhysics.ContactManifoldList = ContactManifoldList;
