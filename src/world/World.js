class World {
    constructor(broadphase, narrowphase, solver) {
        this.broadphase = broadphase;
        this.narrowphase = narrowphase;
        this.solver = solver;
        this.midphase = new Midphase();
        this.islandManager = new IslandManager();

        this.allowSleeping = true;
        this.gravity = new Vector3(0, -9.81, 0);
        this.bodies = [];
        this.constraints = [];
        this._listeners = {};
    }

    addListener(event, fn) {
        (this._listeners[event] || (this._listeners[event] = [])).push(fn);
        return this;
    }

    emit(event, arg) {
        const list = this._listeners[event];
        if (!list) return;
        for (let i = 0; i < list.length; i++) list[i](arg);
    }

    addConstraint(constraint) {
        this.constraints.push(constraint);

        IslandManager.wakeTouching(constraint.bodyA, null, this.constraints);
        if (constraint.bodyB) IslandManager.wakeTouching(constraint.bodyB, null, this.constraints);
        return this;
    }

    removeConstraint(constraint) {
        const i = this.constraints.indexOf(constraint);
        if (i !== -1) this.constraints.splice(i, 1);

        IslandManager.wakeTouching(constraint.bodyA, null, this.constraints);
        if (constraint.bodyB) IslandManager.wakeTouching(constraint.bodyB, null, this.constraints);
        return this;
    }

    addRigidBody(body) {
        body.world = this;
        body.updateDerived();
        this.bodies.push(body);
        this.broadphase.add(body);
        return this;
    }

    removeRigidBody(body) {

        IslandManager.wakeTouching(body, this.narrowphase.manifolds, this.constraints);
        const i = this.bodies.indexOf(body);
        if (i !== -1) this.bodies.splice(i, 1);
        this.broadphase.remove(body);
        this.narrowphase.manifolds.removeBody(body);
        body.world = null;
        return this;
    }

    setBodyTransform(body, position, rotation) {
        if (position) body.position.copy(position);
        if (rotation) body.rotation.copy(rotation);
        body._aabbDirty = true;
        body.updateDerived();
        IslandManager.wakeTouching(body, this.narrowphase.manifolds, this.constraints);
        return this;
    }

    step(dt) {
        this.emit('stepStart', dt);
        for (let i = 0; i < this.bodies.length; i++) this.bodies[i].updateDerived(dt);

        const pairs = this.broadphase.computePairs();
        const manifolds = this.narrowphase.step(pairs, this.midphase, dt);

        if (this.allowSleeping) this.islandManager.update(this.bodies, manifolds, this.constraints, dt, this.gravity);

        const narrowphase = this.narrowphase;
        this.solver.step(this.bodies, manifolds, this.gravity, dt, function (mans) {
            narrowphase.refreshManifoldGeometry(mans);
        }, this.constraints);

        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];
            if (b.bodyType === RigidBody.DYNAMIC) b.clearForces();
        }

        this.emit('contacts', manifolds);

        this.emit('stepEnd', dt);
    }

    rayIntersect(start, end, ignore) {
        return Queries.rayIntersect(this.bodies, start, end, ignore);
    }

    rayIntersectAll(start, end, ignore) {
        return Queries.rayIntersectAll(this.bodies, start, end, ignore);
    }

    shapeIntersect(shape, start, end, rotation, ignore) {
        return Queries.shapeIntersect(this.bodies, shape, start, end, rotation, ignore);
    }
}

ActionPhysics.World = World;
