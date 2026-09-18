var proto = ContactManifold.prototype;

ContactManifold.CONTACT_BAND = 0.02;
ContactManifold._isTouching = function (signedDistance) {
    return signedDistance >= -ContactManifold.CONTACT_BAND;
};

proto.update = function (newContacts, dt) {
    const hadPointsBefore = this.points.length > 0;
    const matched = new Array(newContacts.length).fill(false);

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

            this.points.splice(i, 1);
            this._localAnchors.splice(i, 1);
            this._emitBoth('endContact', existing);
            continue;
        }
        matched[bestJ] = true;

        const keepNormalLambda = existing.normalLambda;
        const keepTangentLambda1 = existing.tangentLambda1;
        const keepTangentLambda2 = existing.tangentLambda2;

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

        void wasOverlapping;
        if (ContactManifold._isTouching(existing.signedDistance)) this._emitBoth('contact', existing);
    }

    for (let j = 0; j < newContacts.length; j++) {
        if (matched[j]) continue;
        const nc = newContacts[j];
        if (nc.signedDistance < 0 && !ContactManifold._isTouching(nc.signedDistance)) {

            if (!this._speculativeAllowed(nc)) continue;
            this._addPoint(nc);
            this._emitBoth('speculativeContact', nc);
        } else {
            this._addPoint(nc);
            this._emitBoth('contact', nc);
        }
    }

    if (hadPointsBefore && this.points.length === 0) this._emitBoth('endAllContact', null);
};

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

proto._speculativeAllowed = function (contact) {
    return this.bodyA._speculativeVeto(contact, this.bodyB) !== false &&
        this.bodyB._speculativeVeto(contact, this.bodyA) !== false;
};

proto._emitBoth = function (event, contact) {
    this.bodyA.emit(event, { contact: contact, other: this.bodyB });
    this.bodyB.emit(event, { contact: contact, other: this.bodyA });
};

ContactManifold._toLocal = function (bodyA, worldPoint, out) {
    Vector3.subInto(out, worldPoint, bodyA.position);
    ContactManifold._scratchInvRot.copy(bodyA.rotation).invert();
    ContactManifold._scratchInvRot.transformVectorInPlace(out);
    return out;
};
