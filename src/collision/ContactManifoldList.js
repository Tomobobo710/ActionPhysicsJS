// One ContactManifold per body pair, keyed by canonical id. refresh() runs once per tick and
// prunes any manifold left with zero points.
class ContactManifoldList {
    constructor() {
        this._manifolds = new Map(); // "idA:idB" (idA < idB) -> ContactManifold
    }

    static _key(bodyA, bodyB) {
        return bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
    }

    // Lower id becomes bodyA, so local-space matching is stable regardless of argument order.
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

    // contactsByPair: key -> ContactDetails[] for this tick (missing = empty). Callers create new
    // pairs via getOrCreate() before this runs.
    refresh(contactsByPair, dt) {
        for (const [key, manifold] of this._manifolds) {
            const contacts = contactsByPair.get(key) || [];
            manifold.update(contacts, dt);
            if (manifold.pointCount === 0) this._manifolds.delete(key);
        }
    }

    values() {
        return this._manifolds.values();
    }

    get size() { return this._manifolds.size; }
}

ActionPhysics.ContactManifoldList = ContactManifoldList;
