// Recomputes everything derived from position/rotation: tight AABB, fattened broadphase AABB, and
// world-space inverse inertia. Called once per body per tick by whichever stage owns "current" -
// narrowphase and the solver assume it has already run.
var proto = RigidBody.prototype;

proto.updateDerived = function (dt) {
    this._recomputeAABB();
    this._recomputeBroadphaseAABB(dt || 0);
    this._recomputeWorldInverseInertia();
    return this;
};

// The TIGHT world AABB: the exact rotated bound of the shape at the current transform, no margin -
// the body's geometric truth, what getAABB()/a raycast wants. Broadphase uses the fattened variant.
proto._recomputeAABB = function () {
    const local = RigidBody._scratchLocalAABB;
    this.shape.localAABBInto(local);
    // Conservative rotated bound via the 8-corner sweep (same technique CompoundShape uses),
    // correct for any rotation, not just axis-aligned ones.
    const rotMat = RigidBody._scratchMat3;
    rotMat.fromQuaternion(this.rotation);
    const corner = RigidBody._scratchVec;
    this._aabb.setEmpty();
    for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
        corner.x = cx ? local.max.x : local.min.x;
        corner.y = cy ? local.max.y : local.min.y;
        corner.z = cz ? local.max.z : local.min.z;
        rotMat.transformVector3(corner);
        corner.addInPlace(this.position);
        if (corner.x < this._aabb.min.x) this._aabb.min.x = corner.x;
        if (corner.y < this._aabb.min.y) this._aabb.min.y = corner.y;
        if (corner.z < this._aabb.min.z) this._aabb.min.z = corner.z;
        if (corner.x > this._aabb.max.x) this._aabb.max.x = corner.x;
        if (corner.y > this._aabb.max.y) this._aabb.max.y = corner.y;
        if (corner.z > this._aabb.max.z) this._aabb.max.z = corner.z;
    }
    this._aabbDirty = false;
};

// The BROADPHASE world AABB: tight AABB fattened by a fixed margin plus this tick's velocity sweep
// per axis, so a fast approach is caught a full tick before the shapes actually overlap - the
// lookahead speculative contacts depend on. Fattening only ever adds candidate pairs, never
// removes one; narrowphase culls precisely with its own per-pair margin. Kept separate from the
// tight AABB so the body's geometric bound stays truthful for queries/rendering.
//
// Sweep is directional (grows only on the side the body is moving toward), keeping the box tight
// rather than symmetric. SPECULATIVE_MARGIN is the absolute floor for the resting/slow case.
proto._recomputeBroadphaseAABB = function (dt) {
    const m = RigidBody.SPECULATIVE_MARGIN;
    const sx = this.linear_velocity.x * dt, sy = this.linear_velocity.y * dt, sz = this.linear_velocity.z * dt;
    // Angular sweep: a point at the body's bounding radius R moves at up to |omega|*R even when the
    // center (tracked by the linear sweep) barely moves - a tipping box's far corner otherwise
    // slams in undetected. Applied isotropically (all six faces), the conservative honest choice,
    // since angular motion has no clean per-axis direction.
    const ex = (this._aabb.max.x - this._aabb.min.x) * 0.5;
    const ey = (this._aabb.max.y - this._aabb.min.y) * 0.5;
    const ez = (this._aabb.max.z - this._aabb.min.z) * 0.5;
    const R = Math.sqrt(ex * ex + ey * ey + ez * ez);
    const wMag = Math.sqrt(this.angular_velocity.x * this.angular_velocity.x +
        this.angular_velocity.y * this.angular_velocity.y + this.angular_velocity.z * this.angular_velocity.z);
    const a = wMag * R * dt;
    this._broadphaseAABB.min.x = this._aabb.min.x - m - a - (sx < 0 ? -sx : 0);
    this._broadphaseAABB.max.x = this._aabb.max.x + m + a + (sx > 0 ? sx : 0);
    this._broadphaseAABB.min.y = this._aabb.min.y - m - a - (sy < 0 ? -sy : 0);
    this._broadphaseAABB.max.y = this._aabb.max.y + m + a + (sy > 0 ? sy : 0);
    this._broadphaseAABB.min.z = this._aabb.min.z - m - a - (sz < 0 ? -sz : 0);
    this._broadphaseAABB.max.z = this._aabb.max.z + m + a + (sz > 0 ? sz : 0);
};

proto._recomputeWorldInverseInertia = function () {
    if (this._massInverted === 0) { this._worldInverseInertiaTensor.zero(); return; }
    const rotMat = RigidBody._scratchMat3;
    rotMat.fromQuaternion(this.rotation);
    const rotT = RigidBody._scratchMat3b;
    rotT.transposeInto(rotMat);
    this._worldInverseInertiaTensor.multiplyFrom(rotMat, this.inverseInertiaTensor);
    this._worldInverseInertiaTensor.multiply(rotT);
};

// Assumes updateDerived() has already run this tick - never recomputes on its own, so a stale call
// is a caller bug surfaced as a stale box, not silently patched over here.
proto.getAABB = function () {
    return this._aabb;
};

// Broadphase/midphase read THIS, not getAABB(), so a pair surfaces the tick before overlap.
proto.getBroadphaseAABB = function () {
    return this._broadphaseAABB;
};
