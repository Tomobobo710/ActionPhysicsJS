// How far ahead of actual touch a contact is reported, so the solver's predicted-position
// non-penetration constraint has a point to work with before overlap happens (the fix for the
// deep-correction -> large-derived-velocity failure mode).
var proto = NarrowPhase.prototype;

// Base margin widened by how far the pair can close in one tick along relative velocity, so a
// fast approach is still caught a full tick ahead of overlap.
proto._speculativeMargin = function (bodyA, bodyB) {
    const dvx = bodyA.linear_velocity.x - bodyB.linear_velocity.x;
    const dvy = bodyA.linear_velocity.y - bodyB.linear_velocity.y;
    const dvz = bodyA.linear_velocity.z - bodyB.linear_velocity.z;
    const relSpeed = Math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
    // A rotating body's contact FEATURE (corner/edge) approaches faster than its center moves -
    // add each body's angular corner speed so a tipping body's corner is still caught in time.
    const angSpeed = NarrowPhase._angularCornerSpeed(bodyA) + NarrowPhase._angularCornerSpeed(bodyB);
    return NarrowPhase.SPECULATIVE_BASE + (relSpeed + angSpeed) * this._dt;
};

// Upper bound on how fast any point on `body` moves purely from rotation: |omega| * bounding radius.
NarrowPhase._angularCornerSpeed = function (body) {
    const w = body.angular_velocity;
    const wMag = Math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    if (wMag === 0) return 0;
    const aabb = body.getAABB();
    const ex = (aabb.max.x - aabb.min.x) * 0.5, ey = (aabb.max.y - aabb.min.y) * 0.5, ez = (aabb.max.z - aabb.min.z) * 0.5;
    return wMag * Math.sqrt(ex * ex + ey * ey + ez * ez);
};
