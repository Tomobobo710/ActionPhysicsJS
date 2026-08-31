// One ContactManifold per body pair, keyed by canonical id. refresh() runs once per tick and
// prunes any manifold left with zero points.
class ContactManifoldList {
    constructor() {
        this._manifolds = new Map(); // "idA:idB" (idA < idB) -> ContactManifold
        // Singly-linked-list view over the live (non-empty) manifolds, relinked at the end of every
        // refresh(). Walk it as: for (let m = list.first; m; m = m.next_manifold). The canonical
        // iteration is values(); this exists for consumers that expect the linked-list shape.
        this.first = null;
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
        this._relink();
    }

    // Rebuild the .first / .next_manifold chain over the surviving manifolds, in Map insertion
    // order (same order values() yields), so the linked-list view and values() agree.
    _relink() {
        let prev = null;
        this.first = null;
        for (const manifold of this._manifolds.values()) {
            manifold.next_manifold = null;
            if (prev) prev.next_manifold = manifold;
            else this.first = manifold;
            prev = manifold;
        }
    }

    values() {
        return this._manifolds.values();
    }

    get size() { return this._manifolds.size; }
}

ActionPhysics.ContactManifoldList = ContactManifoldList;
