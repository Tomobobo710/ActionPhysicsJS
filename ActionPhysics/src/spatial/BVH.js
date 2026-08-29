/**
 * Static BVH over a fixed set of leaves, built once. Flattened array layout - array indexing, not
 * pointer chasing. One implementation, three call sites: compound children,
 * mesh triangles, swept queries.
 *
 * Construction takes a leaf count and two callbacks:
 *   leafAABBInto(out, leafIndex)   writes the leaf's AABB into `out`
 * The tree never touches what a "leaf" IS - a compound child, a mesh triangle, whatever - it only
 * knows AABBs and indices, so this file has no per-caller special cases.
 *
 * Nodes are stored in three parallel typed arrays: min/max as Float64Array (3 components each),
 * and an Int32Array carrying (left, right, leafIndex) - leafIndex is -1 for an internal node.
 * A leaf node has left = right = -1 implicitly (never read); an internal node has leafIndex = -1.
 */
class BVH {
    constructor() {
        this.nodeCount = 0;
        this._capacity = 0;
        this.minX = null; this.minY = null; this.minZ = null;
        this.maxX = null; this.maxY = null; this.maxZ = null;
        this.left = null; this.right = null; this.leafIndex = null;
        this.root = -1;
    }

    _ensureCapacity(n) {
        if (n <= this._capacity) return;
        this._capacity = n;
        this.minX = new Float64Array(n); this.minY = new Float64Array(n); this.minZ = new Float64Array(n);
        this.maxX = new Float64Array(n); this.maxY = new Float64Array(n); this.maxZ = new Float64Array(n);
        this.left = new Int32Array(n).fill(-1);
        this.right = new Int32Array(n).fill(-1);
        this.leafIndex = new Int32Array(n).fill(-1);
    }

    // Builds the tree over `leafCount` leaves. leafAABBInto(out, i) must fill `out` (an AABB) with
    // leaf i's bound. A degenerate call (leafCount === 0) leaves the tree empty (root === -1);
    // querying an empty tree is not an error, it just visits nothing.
    build(leafCount, leafAABBInto) {
        this.nodeCount = 0;
        this.root = -1;
        if (leafCount === 0) return this;

        // Up to 2*leafCount-1 nodes for a full binary tree over leafCount leaves.
        this._ensureCapacity(Math.max(1, 2 * leafCount - 1));

        const scratch = new AABB();
        const indices = new Int32Array(leafCount);
        const centerX = new Float64Array(leafCount), centerY = new Float64Array(leafCount), centerZ = new Float64Array(leafCount);
        const leafMinX = new Float64Array(leafCount), leafMinY = new Float64Array(leafCount), leafMinZ = new Float64Array(leafCount);
        const leafMaxX = new Float64Array(leafCount), leafMaxY = new Float64Array(leafCount), leafMaxZ = new Float64Array(leafCount);
        for (let i = 0; i < leafCount; i++) {
            leafAABBInto(scratch, i);
            indices[i] = i;
            leafMinX[i] = scratch.min.x; leafMinY[i] = scratch.min.y; leafMinZ[i] = scratch.min.z;
            leafMaxX[i] = scratch.max.x; leafMaxY[i] = scratch.max.y; leafMaxZ[i] = scratch.max.z;
            centerX[i] = (scratch.min.x + scratch.max.x) * 0.5;
            centerY[i] = (scratch.min.y + scratch.max.y) * 0.5;
            centerZ[i] = (scratch.min.z + scratch.max.z) * 0.5;
        }

        const self = this;
        function boundsOf(lo, hi) {
            let bx0 = Infinity, by0 = Infinity, bz0 = Infinity, bx1 = -Infinity, by1 = -Infinity, bz1 = -Infinity;
            for (let k = lo; k < hi; k++) {
                const i = indices[k];
                if (leafMinX[i] < bx0) bx0 = leafMinX[i]; if (leafMaxX[i] > bx1) bx1 = leafMaxX[i];
                if (leafMinY[i] < by0) by0 = leafMinY[i]; if (leafMaxY[i] > by1) by1 = leafMaxY[i];
                if (leafMinZ[i] < bz0) bz0 = leafMinZ[i]; if (leafMaxZ[i] > bz1) bz1 = leafMaxZ[i];
            }
            return { minX: bx0, minY: by0, minZ: bz0, maxX: bx1, maxY: by1, maxZ: bz1 };
        }

        // Median split on the widest axis of [lo, hi)'s combined bound. A cheap, deterministic
        // heuristic (no SAH) - measured cost is ~1.38 children visited per
        // query, which came from exactly this kind of structure; revisit only if a stress test
        // shows it isn't good enough.
        function buildRange(lo, hi) {
            const b = boundsOf(lo, hi);
            const nodeIdx = self.nodeCount++;
            self.minX[nodeIdx] = b.minX; self.minY[nodeIdx] = b.minY; self.minZ[nodeIdx] = b.minZ;
            self.maxX[nodeIdx] = b.maxX; self.maxY[nodeIdx] = b.maxY; self.maxZ[nodeIdx] = b.maxZ;

            if (hi - lo === 1) {
                self.leafIndex[nodeIdx] = indices[lo];
                self.left[nodeIdx] = -1; self.right[nodeIdx] = -1;
                return nodeIdx;
            }

            const sx = b.maxX - b.minX, sy = b.maxY - b.minY, sz = b.maxZ - b.minZ;
            let axis = 0, getCenter = centerX;
            if (sy >= sx && sy >= sz) { axis = 1; getCenter = centerY; }
            else if (sz >= sx && sz >= sy) { axis = 2; getCenter = centerZ; }

            // Partition indices[lo,hi) around the median center on the chosen axis, in place.
            const sub = indices.subarray(lo, hi);
            Array.prototype.sort.call(sub, function (ia, ib) { return getCenter[ia] - getCenter[ib]; });
            const mid = lo + ((hi - lo) >> 1);

            self.leafIndex[nodeIdx] = -1;
            self.left[nodeIdx] = buildRange(lo, mid);
            self.right[nodeIdx] = buildRange(mid, hi);
            return nodeIdx;
        }

        this.root = buildRange(0, leafCount);
        return this;
    }

    // Visits every leaf whose node AABB intersects `queryAABB`, calling onLeaf(leafIndex) for
    // each. No allocation - an explicit stack in a plain array, reused across calls via `_stack`.
    query(queryAABB, onLeaf) {
        if (this.root === -1) return;
        const qminx = queryAABB.min.x, qminy = queryAABB.min.y, qminz = queryAABB.min.z;
        const qmaxx = queryAABB.max.x, qmaxy = queryAABB.max.y, qmaxz = queryAABB.max.z;

        const stack = this._stack || (this._stack = []);
        let sp = 0;
        stack[sp++] = this.root;
        while (sp > 0) {
            const node = stack[--sp];
            if (this.minX[node] > qmaxx || this.maxX[node] < qminx ||
                this.minY[node] > qmaxy || this.maxY[node] < qminy ||
                this.minZ[node] > qmaxz || this.maxZ[node] < qminz) continue;

            if (this.leafIndex[node] !== -1) { onLeaf(this.leafIndex[node]); continue; }
            stack[sp++] = this.left[node];
            stack[sp++] = this.right[node];
        }
    }
}

ActionPhysics.BVH = BVH;
