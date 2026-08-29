// Per-tick manifold update: match existing points against this tick's narrowphase result, warm-
// start matched points, add genuinely new ones, remove unconfirmed ones. Fires contact lifecycle
// events (speculativeContact, contact, endContact, endAllContact) on both bodies as state changes.
var proto = ContactManifold.prototype;

proto.update = function (newContacts, dt) {
    const hadPointsBefore = this.points.length > 0;
    const matched = new Array(newContacts.length).fill(false);

    // Match each existing point against the best (closest, in bodyA-local space) unmatched
    // incoming contact.
    for (let i = this.points.length - 1; i >= 0; i--) {
        const existing = this.points[i];
        const existingLocal = this._localAnchors[i];
        const matchDist = this._matchDistance(existing, dt);
        let bestJ = -1, bestDistSq = matchDist * matchDist;
        for (let j = 0; j < newContacts.length; j++) {
            if (matched[j]) continue;
            const localCandidate = ContactManifold._toLocal(this.bodyA, newContacts[j].pointOnA, ContactManifold._scratchLocal);
            const dx = localCandidate.x - existingLocal.x, dy = localCandidate.y - existingLocal.y, dz = localCandidate.z - existingLocal.z;
            const distSq = dx * dx + dy * dy + dz * dz;
            if (distSq < bestDistSq) { bestDistSq = distSq; bestJ = j; }
        }
        if (bestJ === -1) {
            // Not re-confirmed this tick: remove (the only removal path, never mid-substep).
            this.points.splice(i, 1);
            this._localAnchors.splice(i, 1);
            this._emitBoth('endContact', existing);
            continue;
        }
        matched[bestJ] = true;
        // Capture the warm-start lambdas before copy() zeroes them from the fresh incoming contact.
        const keepNormalLambda = existing.normalLambda;
        const keepTangentLambda1 = existing.tangentLambda1;
        const keepTangentLambda2 = existing.tangentLambda2;
        // Inside EXACT_TOUCH_BAND, GJK/EPA's recovered normal is ambiguous - keep the established
        // one, or a persistent contact hits a penetrate-then-launch limit cycle.
        const keepNormal = Math.abs(newContacts[bestJ].signedDistance) < ContactManifold.EXACT_TOUCH_BAND
            ? ContactManifold._scratchNormal.copy(existing.normal)
            : null;
        const wasOverlapping = existing.signedDistance >= 0;
        existing.copy(newContacts[bestJ]);
        existing.normalLambda = keepNormalLambda;
        existing.tangentLambda1 = keepTangentLambda1;
        existing.tangentLambda2 = keepTangentLambda2;
        if (keepNormal) existing.normal.copy(keepNormal);
        ContactManifold._toLocal(this.bodyA, existing.pointOnA, existingLocal);
        if (!wasOverlapping && existing.signedDistance >= 0) this._emitBoth('contact', existing);
    }

    // Any incoming contact not matched to an existing point is genuinely new.
    for (let j = 0; j < newContacts.length; j++) {
        if (matched[j]) continue;
        if (newContacts[j].signedDistance < 0) {
            if (!this._speculativeAllowed(newContacts[j])) continue; // vetoed by a listener
            this._addPoint(newContacts[j]);
            this._emitBoth('speculativeContact', newContacts[j]);
        } else {
            this._addPoint(newContacts[j]);
            this._emitBoth('contact', newContacts[j]);
        }
    }

    if (hadPointsBefore && this.points.length === 0) this._emitBoth('endAllContact', null);
};

// MATCH_DISTANCE widened by the contact point's tangential travel this tick, so a fast-sliding or
// rolling contact's point still matches instead of rebuilding the manifold (and losing warm-start).
proto._matchDistance = function (point, dt) {
    if (!dt) return ContactManifold.MATCH_DISTANCE;
    const bodyA = this.bodyA, bodyB = this.bodyB;
    point.currentAnchorAInto(ContactManifold._scratchRA, bodyA);
    point.currentAnchorBInto(ContactManifold._scratchRB, bodyB);
    const rax = ContactManifold._scratchRA.x - bodyA.position.x, ray = ContactManifold._scratchRA.y - bodyA.position.y, raz = ContactManifold._scratchRA.z - bodyA.position.z;
    const rbx = ContactManifold._scratchRB.x - bodyB.position.x, rby = ContactManifold._scratchRB.y - bodyB.position.y, rbz = ContactManifold._scratchRB.z - bodyB.position.z;
    const wa = bodyA.angular_velocity, va = bodyA.linear_velocity;
    const wb = bodyB.angular_velocity, vb = bodyB.linear_velocity;
    const vax = va.x + (wa.y * raz - wa.z * ray), vay = va.y + (wa.z * rax - wa.x * raz), vaz = va.z + (wa.x * ray - wa.y * rax);
    const vbx = vb.x + (wb.y * rbz - wb.z * rby), vby = vb.y + (wb.z * rbx - wb.x * rbz), vbz = vb.z + (wb.x * rby - wb.y * rbx);
    const relx = vbx - vax, rely = vby - vay, relz = vbz - vaz;
    const n = point.normal;
    const vdotn = relx * n.x + rely * n.y + relz * n.z;
    const tx = relx - vdotn * n.x, ty = rely - vdotn * n.y, tz = relz - vdotn * n.z;
    const tangentialSpeed = Math.sqrt(tx * tx + ty * ty + tz * tz);
    return ContactManifold.MATCH_DISTANCE + tangentialSpeed * dt;
};

// A speculativeContact listener on either body may veto the point before it's added.
proto._speculativeAllowed = function (contact) {
    return this.bodyA._speculativeVeto(contact, this.bodyB) !== false &&
        this.bodyB._speculativeVeto(contact, this.bodyA) !== false;
};

proto._emitBoth = function (event, contact) {
    this.bodyA.emit(event, { contact: contact, other: this.bodyB });
    this.bodyB.emit(event, { contact: contact, other: this.bodyA });
};

// World point -> bodyA-local space, for next-tick matching. Writes into caller-owned `out`.
ContactManifold._toLocal = function (bodyA, worldPoint, out) {
    Vector3.subInto(out, worldPoint, bodyA.position);
    ContactManifold._scratchInvRot.copy(bodyA.rotation).invert();
    ContactManifold._scratchInvRot.transformVectorInPlace(out);
    return out;
};
