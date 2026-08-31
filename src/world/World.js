// Pipeline glue: owns the body list, drives one tick through broadphase -> midphase/narrowphase ->
// solver. Query methods delegate to Queries.js.
class World {
    constructor(broadphase, narrowphase, solver) {
        this.broadphase = broadphase;
        this.narrowphase = narrowphase;
        this.solver = solver;
        this.midphase = new Midphase();
        this.islandManager = new IslandManager();
        // When false, the island manager is skipped and every dynamic body is solved every tick -
        // nothing ever parks. Sleeping is otherwise transparent (a parked body resumes exactly where
        // it stopped), so this is only for debugging or for a scene that wants to rule sleep out.
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
        return this;
    }

    removeConstraint(constraint) {
        const i = this.constraints.indexOf(constraint);
        if (i !== -1) this.constraints.splice(i, 1);
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
        const i = this.bodies.indexOf(body);
        if (i !== -1) this.bodies.splice(i, 1);
        this.broadphase.remove(body);
        body.world = null;
        return this;
    }

    step(dt) {
        this.emit('stepStart', dt);
        for (let i = 0; i < this.bodies.length; i++) this.bodies[i].updateDerived(dt);

        const pairs = this.broadphase.computePairs();
        const manifolds = this.narrowphase.step(pairs, this.midphase, dt);

        // Decide sleep state before the solver runs; it skips !isAwake dynamic bodies.
        if (this.allowSleeping) this.islandManager.update(this.bodies, manifolds, this.constraints, dt);

        const narrowphase = this.narrowphase;
        this.solver.step(this.bodies, manifolds, this.gravity, dt, function (mans) {
            narrowphase.refreshManifoldGeometry(mans); // per-substep geometry re-measure
        }, this.constraints);

        for (let i = 0; i < this.bodies.length; i++) { // forces are per-tick
            const b = this.bodies[i];
            if (b.bodyType === RigidBody.DYNAMIC) b.clearForces();
        }

        // Resolved contacts, for listeners. Emitted post-solve so positions reflect the result.
        this.emit('contacts', manifolds);

        this.emit('stepEnd', dt);
    }

    rayIntersect(start, end, ignore) {
        return Queries.rayIntersect(this.bodies, start, end, ignore);
    }

    // Every body the segment crosses, nearest-first (empty array = miss). For a caller that filters
    // hits itself; rayIntersect() is the single-nearest form.
    rayIntersectAll(start, end, ignore) {
        return Queries.rayIntersectAll(this.bodies, start, end, ignore);
    }

    shapeIntersect(shape, start, end, rotation, ignore) {
        return Queries.shapeIntersect(this.bodies, shape, start, end, rotation, ignore);
    }
}

ActionPhysics.World = World;
