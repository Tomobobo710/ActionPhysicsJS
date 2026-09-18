var proto = NarrowPhase.prototype;

proto._speculativeMargin = function (bodyA, bodyB) {
    const dvx = bodyA.linear_velocity.x - bodyB.linear_velocity.x;
    const dvy = bodyA.linear_velocity.y - bodyB.linear_velocity.y;
    const dvz = bodyA.linear_velocity.z - bodyB.linear_velocity.z;
    const relSpeed = Math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz);

    const angSpeed = NarrowPhase._angularCornerSpeed(bodyA) + NarrowPhase._angularCornerSpeed(bodyB);
    return NarrowPhase.SPECULATIVE_BASE + (relSpeed + angSpeed) * this._dt;
};

NarrowPhase._angularCornerSpeed = function (body) {
    const w = body.angular_velocity;
    const wMag = Math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    if (wMag === 0) return 0;
    const aabb = body.getAABB();
    const ex = (aabb.max.x - aabb.min.x) * 0.5, ey = (aabb.max.y - aabb.min.y) * 0.5, ez = (aabb.max.z - aabb.min.z) * 0.5;
    return wMag * Math.sqrt(ex * ex + ey * ey + ez * ez);
};
