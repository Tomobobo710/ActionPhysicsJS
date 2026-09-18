var proto = NarrowPhase.prototype;

proto._nextPooledContact = function () {
    if (this._poolIndex >= this._contactPool.length) this._contactPool.push(new ContactDetails());
    const c = this._contactPool[this._poolIndex++];
    c.fromMeshFace = false;
    c.meshTriValid = false;
    c.edgeAxis = null;
    c.fromBoxBox = false;
    c.fromFacePatch = false;
    c.fromCurvedTri = false;
    return c;
};

proto.step = function (broadphasePairs, midphase, dt) {
    if (dt) this._dt = dt;
    this._midphase = midphase;
    this._poolIndex = 0;
    const contactsByPair = new Map();

    const spd = this._tickStartSpeedSq || (this._tickStartSpeedSq = new Map());
    spd.clear();
    for (let p = 0; p < broadphasePairs.length; p++) {
        for (let s = 0; s < 2; s++) {
            const b = broadphasePairs[p][s];
            if (b.bodyType !== RigidBody.DYNAMIC || spd.has(b.id)) continue;
            const lv = b.linear_velocity, av = b.angular_velocity;
            spd.set(b.id, {
                lin: lv.x * lv.x + lv.y * lv.y + lv.z * lv.z,
                ang: av.x * av.x + av.y * av.y + av.z * av.z
            });
        }
    }

    for (let p = 0; p < broadphasePairs.length; p++) {
        const bodyA = broadphasePairs[p][0], bodyB = broadphasePairs[p][1];
        const sides = midphase.expandPairSides(bodyA, bodyB);
        const sidesA = sides.a, sidesB = sides.b;
        const key = bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
        const margin = this._speculativeMargin(bodyA, bodyB);

        const existing = this.manifolds._manifolds.get(key);
        this._ctHintNormal = (existing && existing.points.length > 0 && existing.points[0].fromMeshFace)
            ? existing.points[0].normal : null;

        this._curMargin = margin;

        let sawMeshFace = false;
        for (let i = 0; i < sidesA.length; i++) {
            for (let j = 0; j < sidesB.length; j++) {
                const pairContacts = this._testPrimitivePair(sidesA[i], sidesB[j]);
                for (let c = 0; c < pairContacts.length; c++) {
                    const contact = pairContacts[c];
                    if (contact.signedDistance < -margin) continue;
                    if (contact.fromMeshFace) sawMeshFace = true;
                    let list = contactsByPair.get(key);
                    if (!list) { list = []; contactsByPair.set(key, list); }
                    list.push(contact);
                }
            }
        }

        if (sawMeshFace) {
            const list = contactsByPair.get(key);
            let w = 0;
            for (let r = 0; r < list.length; r++) if (list[r].fromMeshFace) list[w++] = list[r];
            list.length = w;
        }

        this.manifolds.getOrCreate(bodyA, bodyB);
    }
    this._curMargin = null;

    this.manifolds.refresh(contactsByPair, this._dt);
    return this.manifolds;
};

proto._testPrimitivePair = function (placedA, placedB) {
    const results = this._pairResultScratch;
    results.length = 0;

    if (SphereSphere.applies(placedA, placedB)) {
        results.push(SphereSphere.test(placedA, placedB, this._nextPooledContact()));
        return results;
    }
    if (SphereBox.applies(placedA, placedB)) {
        results.push(SphereBox.test(placedA, placedB, this._nextPooledContact()));
        return results;
    }
    if (BoxBox.applies(placedA, placedB)) {
        const self = this;

        const boxResult = BoxBox.test(placedA, placedB, results, function () { return self._nextPooledContact(); });
        if (boxResult !== null) return results;
    }

    if (TriTri.applies(placedA, placedB)) {
        const self = this;

        const triResult = TriTri.test(placedA, placedB, results, function () { return self._nextPooledContact(); });
        if (triResult !== null) return results;
    }

    if (BoxTriFace.applies(placedA, placedB)) {
        const self = this;

        const bf = BoxTriFace.test(placedA, placedB, results, function () { return self._nextPooledContact(); }, this._ctHintNormal, this._curMargin);
        if (bf !== null) return results;
    }

    if (CapTriFace.applies(placedA, placedB)) {
        const self = this;

        const capResult = CapTriFace.test(placedA, placedB, results, function () { return self._nextPooledContact(); }, this._ctHintNormal, this._curMargin);
        if (capResult !== null) return results;
    }

    if (ConvexTri.applies(placedA, placedB)) {
        const self = this;

        const ctResult = ConvexTri.test(placedA, placedB, results, function () { return self._nextPooledContact(); }, this._ctHintNormal, this._curMargin);
        if (ctResult !== null) return results;
        if (ConvexTri.lastVerdict === 'separated') return results;

    } else if ((placedA.shape instanceof TriangleShape) !== (placedB.shape instanceof TriangleShape)) {

        if (TriPlaneCull.separated(placedA, placedB)) return results;
    }

    const contact = this._nextPooledContact();
    const support = this._support.setSides(placedA, placedB);
    const gjkResult = this._gjk.run(support);
    if (gjkResult.overlapping) {
        const epaResult = this._epa.run(support, gjkResult.simplex);
        contact.setFromEPA(epaResult);

        const self = this;
        const clipped = PolyClip.buildFaceContact(
            placedA, placedB, contact.normal, results,
            function () { return self._nextPooledContact(); },
            this._curMargin != null ? this._curMargin : NarrowPhase.SPECULATIVE_BASE);
        if (clipped > 0) return results;

        if (contact.signedDistance > NarrowPhase._maxPlausiblePenetration(placedA.shape, placedB.shape)) {
            contact.setFromGJKSeparated({
                distance: contact.signedDistance,
                normal: contact.normal,
                pointA: contact.pointOnA,
                pointB: contact.pointOnB,
            });
        }
    } else {
        contact.setFromGJKSeparated(gjkResult);
        if (contact.signedDistance > -0.003 &&
            (placedA.shape instanceof ConvexShape || placedB.shape instanceof ConvexShape)) {
            const self = this;
            const clipped = PolyClip.buildFaceContact(
                placedA, placedB, contact.normal, results,
                function () { return self._nextPooledContact(); },
                this._curMargin != null ? this._curMargin : NarrowPhase.SPECULATIVE_BASE);
            if (clipped > 0) return results;
        }
    }
    results.push(contact);
    return results;
};

NarrowPhase._maxPlausiblePenetration = function (shapeA, shapeB) {
    return Math.min(NarrowPhase._boundingRadius(shapeA), NarrowPhase._boundingRadius(shapeB));
};

NarrowPhase._boundingRadius = function (shape) {
    const aabb = NarrowPhase._brAABB || (NarrowPhase._brAABB = new AABB());
    shape.localAABBInto(aabb);
    const ex = Math.max(Math.abs(aabb.min.x), Math.abs(aabb.max.x));
    const ey = Math.max(Math.abs(aabb.min.y), Math.abs(aabb.max.y));
    const ez = Math.max(Math.abs(aabb.min.z), Math.abs(aabb.max.z));
    return Math.sqrt(ex * ex + ey * ey + ez * ez);
};

proto._isCompoundOrMesh = function (shape) {
    return (typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape) ||
        (typeof MeshShape !== 'undefined' && shape instanceof MeshShape);
};
