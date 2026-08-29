// Static triangle mesh: a vertex list plus flat index triples. Zero mass by construction — a
// mesh is a static/kinematic-only shape (its BVH is built once and never updated).
// The midphase BVH over these triangles is built lazily by whatever consumes this shape;
// this class only owns geometry.
class MeshShape extends Shape {
    constructor(vertices, indices) {
        super('mesh');
        this.vertices = vertices;   // Vector3[]
        this.indices = indices;     // flat Uint32Array-able index triples
        this.triangleCount = (indices.length / 3) | 0;
    }

    triangleAt(i, outA, outB, outC) {
        const base = i * 3;
        const va = this.vertices[this.indices[base]];
        const vb = this.vertices[this.indices[base + 1]];
        const vc = this.vertices[this.indices[base + 2]];
        outA.copy(va); outB.copy(vb); outC.copy(vc);
    }

    // A mesh has no single well-defined support point (it's a shell, not a solid convex body) —
    // narrowphase dispatches per-triangle via TriangleShape instead of calling this directly.
    supportInto(out, direction) {
        throw new Error('MeshShape.supportInto: dispatch per-triangle, a mesh is not itself convex');
    }

    localAABBInto(out) {
        out.setEmpty();
        const verts = this.vertices;
        for (let i = 0; i < verts.length; i++) {
            const p = verts[i];
            if (p.x < out.min.x) out.min.x = p.x;
            if (p.y < out.min.y) out.min.y = p.y;
            if (p.z < out.min.z) out.min.z = p.z;
            if (p.x > out.max.x) out.max.x = p.x;
            if (p.y > out.max.y) out.max.y = p.y;
            if (p.z > out.max.z) out.max.z = p.z;
        }
        return out;
    }

    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.MeshShape = MeshShape;
