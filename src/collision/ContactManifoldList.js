class ContactManifoldList {
    constructor() {
        this._manifolds = new Map();

        this.first = null;
    }

    static _key(bodyA, bodyB) {
        return bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
    }

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

    refresh(contactsByPair, dt) {
        for (const [key, manifold] of this._manifolds) {
            const contacts = contactsByPair.get(key) || [];
            manifold.update(contacts, dt);
            if (manifold.pointCount === 0) this._manifolds.delete(key);
        }
        this._relink();
    }

    removeBody(body) {
        for (const [key, manifold] of this._manifolds) {
            if (manifold.bodyA === body || manifold.bodyB === body) this._manifolds.delete(key);
        }
        this._relink();
    }

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
