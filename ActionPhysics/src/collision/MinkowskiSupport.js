/**
 * World-space support function over the Minkowski DIFFERENCE of two placed shapes (A - B), the one
 * primitive GJK/EPA need. Each side is a { shape, position, rotation } as produced by Midphase.
 *
 * A shape's own supportInto() works in LOCAL space. To support a placed shape in world space along
 * world direction d: rotate d into local space (inverse rotation), call supportInto, rotate the
 * result back to world space, translate by position. Minkowski difference support along d is then
 * supportA(d) - supportB(-d) - the standard GJK primitive.
 *
 * Allocation-free: every method writes into a caller-owned `out`. A single instance is built per
 * narrowphase pair and reused across every GJK/EPA iteration for that pair.
 */
class MinkowskiSupport {
    constructor(placedA, placedB) {
        this.a = placedA;
        this.b = placedB;
        this._localDir = new Vector3();
        this._invRotA = new Quaternion();
        this._invRotB = new Quaternion();
        this._invRotA.copy(placedA.rotation).invert();
        this._invRotB.copy(placedB.rotation).invert();
        // Per-instance scratch (plan.md: "scratch memory: per-stage arenas, never global") - EPA
        // calls supportInto() while GJK's own loop may still be holding references into a prior
        // call's result, so a SHARED scratch across instances (or across nested calls) would
        // silently corrupt whichever call didn't finish first.
        this._scratchA = new Vector3();
        this._scratchB = new Vector3();
        this._scratchNeg = new Vector3();
    }

    // Re-derive the cached inverse rotations if a placed side's rotation object was mutated after
    // construction (narrowphase reuses one MinkowskiSupport across substeps for a persistent pair).
    refresh() {
        this._invRotA.copy(this.a.rotation).invert();
        this._invRotB.copy(this.b.rotation).invert();
        return this;
    }

    // World-space support of ONE placed side along world direction `dir`.
    static supportOfInto(out, placed, invRot, dir, scratchDir) {
        invRot.transformVectorInto(dir, scratchDir);
        placed.shape.supportInto(out, scratchDir);
        placed.rotation.transformVectorInPlace(out);
        out.addInPlace(placed.position);
        return out;
    }

    // out = supportA(dir) - supportB(-dir), in world space. Also writes the two world support
    // points into outA/outB if given (EPA needs them per Minkowski-difference vertex, to recover
    // the actual contact points once the penetrating simplex face is known).
    supportInto(out, dir, outA, outB) {
        const sa = outA || this._scratchA;
        const sb = outB || this._scratchB;
        MinkowskiSupport.supportOfInto(sa, this.a, this._invRotA, dir, this._localDir);
        const negDir = this._scratchNeg.set(-dir.x, -dir.y, -dir.z);
        MinkowskiSupport.supportOfInto(sb, this.b, this._invRotB, negDir, this._localDir);
        Vector3.subInto(out, sa, sb);
        return out;
    }
}

ActionPhysics.MinkowskiSupport = MinkowskiSupport;
