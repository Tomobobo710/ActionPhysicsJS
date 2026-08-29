/**
 * Sweep-and-prune broadphase over AABBs, sorted along a single axis.
 *
 * Produces: candidate body pairs, no false negatives. May assume AABBs are
 * current - it never recomputes one, only reads body.getBroadphaseAABB() (the fattened, speculative-
 * margin variant, so a pair surfaces the tick before overlap). Must never test actual shapes;
 * the only thing this file knows about a body is its AABB.
 *
 * ONE axis, not three. The classic SAP maintains sorted lists on all three axes and intersects
 * them; a single axis with a good pick (the one with the most spread, recomputed occasionally)
 * gets most of the culling at a third of the bookkeeping. The remaining axes are checked directly
 * per candidate pair below, which is cheap because the axis sort has already thrown out most of
 * the O(n^2) pairs.
 */
class SAPBroadphase {
    constructor() {
        // Sorted by AABB min on the sweep axis. Re-sorted every update() - insertion-sort cost is
        // fine because frame-to-frame the order barely changes (temporal coherence), and an O(n
        // log n) sort with no coherence assumption is simpler to trust than a persistent structure
        // during this stage's first pass.
        this._entries = [];   // { body, aabb } - aabb is a snapshot reference, not a copy
        this._axis = 'x';
    }

    // Body lifecycle - the World is the only expected caller.
    add(body) {
        this._entries.push({ body: body, aabb: body.getBroadphaseAABB() });
    }

    remove(body) {
        for (let i = 0; i < this._entries.length; i++) {
            if (this._entries[i].body === body) { this._entries.splice(i, 1); return; }
        }
    }

    // Re-picks the sweep axis from the current AABB spread. Cheap (O(n)) and only needs to be
    // roughly right - a wrong axis costs candidate-pair quality, never correctness, because every
    // axis is still checked directly on each candidate pair in _sweep().
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

    // Returns candidate pairs as [bodyA, bodyB][], A.id < B.id always, so downstream pairing (a
    // manifold cache keyed by two ids) has one canonical order without the caller sorting again.
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
                // Sorted by min on the sweep axis: once ej's min passes ei's max, no later entry
                // can overlap ei on this axis either (their mins only increase from here).
                if (ej.aabb.min[axis] > maxOnAxis) break;
                if (!ei.aabb.intersects(ej.aabb)) continue; // confirms the other two axes
                const a = ei.body, b = ej.body;
                // Two statics/kinematics never need a contact between each other - nothing can
                // move them into or out of overlap from this pair alone. Filtered here rather
                // than downstream so midphase/narrowphase never see a pair that can't matter.
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
