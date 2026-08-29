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
            const localCandidate = ContactManifold._toLocal(this.bodyA, newContacts[j].pointOnA);
            const dx = localCandidate.x - existingLocal.x, dy = localCandidate.y - existingLocal.y, dz = localCandidate.z - existingLocal.z;
            const distSq = dx * dx + dy * dy + dz * dz;
            if (distSq < bestDistSq) { bestDistSq = distSq; bestJ = j; }
        }
        if (bestJ === -1) {
            // Not re-confirmed this tick: remove. The only removal path - never mid-substep.
            this.points.splice(i, 1);
            this._localAnchors.splice(i, 1);
            this._emitBoth('endContact', existing);
            continue;
        }
        matched[bestJ] = true;
        // Capture lambda BEFORE copy() overwrites it - copy() pulls every field from
        // newContacts[bestJ], whose lambda is always zero (a fresh ContactDetails has no solver
        // history). `existing` and `this.points[i]` are the same object, so reading "prior value"
        // after copy() would just read back the zero just written.
        const keepNormalLambda = existing.normalLambda;
        const keepTangentLambda1 = existing.tangentLambda1;
        const keepTangentLambda2 = existing.tangentLambda2;
        // Preserve the established normal through an exact-touch refresh: within EXACT_TOUCH_BAND
        // of zero signed distance, GJK/EPA's recovered normal is genuinely ambiguous (can flip to a
        // diagonal face normal for one tick), and a persistent contact's true normal doesn't
        // actually change tick to tick - trusting the ambiguous tick over the established one causes
        // a penetrate-then-launch limit cycle. Outside the band the fresh normal is trustworthy.
        const keepNormal = Math.abs(newContacts[bestJ].signedDistance) < ContactManifold.EXACT_TOUCH_BAND
            ? ContactManifold._scratchNormal.copy(existing.normal)
            : null;
        const wasOverlapping = existing.signedDistance >= 0;
        existing.copy(newContacts[bestJ]);
        existing.normalLambda = keepNormalLambda;
        existing.tangentLambda1 = keepTangentLambda1;
        existing.tangentLambda2 = keepTangentLambda2;
        if (keepNormal) existing.normal.copy(keepNormal);
        this._localAnchors[i] = ContactManifold._toLocal(this.bodyA, existing.pointOnA);
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

// Match tolerance for one existing point: the base floor (MATCH_DISTANCE, for a resting/slow
// contact) widened by how far the contact point itself travels across each body's surface this
// tick - the tangential relative velocity at the contact, times dt. Without this, a fast-sliding
// or fast-rolling contact's point genuinely moves several tenths of a metre per tick in bodyA-
// local space, blows past a fixed-radius match, and the manifold is destroyed and rebuilt from
// scratch every tick - warm-start (accumulated lambda) never survives a single tick for exactly
// the contacts that need it most. Same shape as SpeculativeMargin.js's own base+dynamic split.
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

// World point -> bodyA-local space, for next-tick matching.
ContactManifold._toLocal = function (bodyA, worldPoint) {
    const rel = Vector3.subInto(new Vector3(), worldPoint, bodyA.position);
    const invRot = new Quaternion().copy(bodyA.rotation).invert();
    invRot.transformVectorInPlace(rel);
    return rel;
};
