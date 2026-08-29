// Sweep-and-prune over fattened AABBs (body.getBroadphaseAABB()), sorted along the most-spread axis.
class SAPBroadphase {
    constructor() {
        this._entries = []; // { body, aabb } - aabb is a live reference
        this._axis = 'x';
    }

    add(body) {
        this._entries.push({ body: body, aabb: body.getBroadphaseAABB() });
    }

    remove(body) {
        for (let i = 0; i < this._entries.length; i++) {
            if (this._entries[i].body === body) { this._entries.splice(i, 1); return; }
        }
    }

    _pickAxis() {
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
        for (let i = 0; i < this._entries.length; i++) {
            const c = this._entries[i].aabb;
            if (c.min.x < minX) minX = c.min.x; if (c.max.x > maxX) maxX = c.max.x;
            if (c.min.y < minY) minY = c.min.y; if (c.max.y > maxY) maxY = c.max.y;
            if (c.min.z < minZ) minZ = c.min.z; if (c.max.z > maxZ) maxZ = c.max.z;
        }
        const sx = maxX - minX, sy = maxY - minY, sz = maxZ - minZ;
        this._axis = (sx >= sy && sx >= sz) ? 'x' : (sy >= sz ? 'y' : 'z');
    }

    // [bodyA, bodyB][], A.id < B.id always.
    computePairs() {
        const n = this._entries.length;
        const pairs = [];
        if (n < 2) return pairs;

        this._pickAxis();
        const axis = this._axis;
        this._entries.sort(function (a, b) { return a.aabb.min[axis] - b.aabb.min[axis]; });

        for (let i = 0; i < n; i++) {
            const ei = this._entries[i];
            const maxOnAxis = ei.aabb.max[axis];
            for (let j = i + 1; j < n; j++) {
                const ej = this._entries[j];
                if (ej.aabb.min[axis] > maxOnAxis) break; // mins only increase from here
                if (!ei.aabb.intersects(ej.aabb)) continue;
                const a = ei.body, b = ej.body;
                if (a.bodyType !== RigidBody.DYNAMIC && b.bodyType !== RigidBody.DYNAMIC) continue;
                if ((a.collision_mask & b.collision_groups) === 0) continue;
                if ((b.collision_mask & a.collision_groups) === 0) continue;
                if (a.id < b.id) pairs.push([a, b]); else pairs.push([b, a]);
            }
        }
        return pairs;
    }
}

ActionPhysics.SAPBroadphase = SAPBroadphase;
