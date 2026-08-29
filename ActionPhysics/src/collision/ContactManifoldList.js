// Active ContactManifolds, keyed by canonical body-pair id. One manifold per pair; refresh() runs
// once per tick, prunes any manifold left with zero points.
class ContactManifoldList {
    constructor() {
        this._manifolds = new Map(); // "idA:idB" (idA < idB) -> ContactManifold
    }

    static _key(bodyA, bodyB) {
        return bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
    }

    // Canonical body order (lower id = bodyA) so local-space matching stays consistent regardless
    // of caller argument order.
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

    // contactsByPair: key -> ContactDetails[] for this tick. A pair with no entry is treated as empty.
    // dt: this tick's timestep, used to size the match-distance tolerance (see Update.js).
    refresh(contactsByPair, dt) {
        for (const [key, manifold] of this._manifolds) {
            const contacts = contactsByPair.get(key) || [];
            manifold.update(contacts, dt);
            if (manifold.pointCount === 0) this._manifolds.delete(key);
        }
        // New pairs are created via getOrCreate() by the caller before refresh() runs.
    }

    values() {
        return this._manifolds.values();
    }

    get size() { return this._manifolds.size; }
}

ActionPhysics.ContactManifoldList = ContactManifoldList;
