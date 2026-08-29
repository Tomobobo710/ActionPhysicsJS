/**
 * World: the pipeline glue. Owns the body list and drives one tick through every stage in order -
 * broadphase, midphase, narrowphase, solver.
 *
 * Public surface: addRigidBody, removeRigidBody,
 * addConstraint, removeConstraint, step(dt), gravity, rayIntersect, shapeIntersect. The two query
 * methods are thin delegates to Queries (Rule 2: World is pipeline glue, not where an algorithm's
 * real body lives) — they exist here only because that is the documented public surface callers use.
 */
class World {
    constructor(broadphase, narrowphase, solver) {
        this.broadphase = broadphase;
        this.narrowphase = narrowphase;
        this.solver = solver;
        this.midphase = new Midphase();
        this.gravity = new Vector3(0, -9.81, 0);
        this.bodies = []; // all bodies, static and dynamic alike - broadphase filters by type itself
        this.constraints = []; // joints - Point/Hinge/Slider/Weld, all solved by the same solver each substep
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

    // Advances the whole world by `dt`: broadphase -> midphase/narrowphase (fused inside
    // NarrowPhase.step, which owns calling into Midphase) -> solver.
    // Every dynamic body's derived state (world AABB, world inverse inertia) is refreshed BEFORE
    // broadphase runs, so broadphase's own "AABBs are current" assumption (Rule 1) holds for
    // this tick's bodies, including ones the solver moved last tick.
    step(dt) {
        this.emit('stepStart', dt);
        for (let i = 0; i < this.bodies.length; i++) this.bodies[i].updateDerived(dt);

        const pairs = this.broadphase.computePairs();
        const manifolds = this.narrowphase.step(pairs, this.midphase, dt);

        // Interleaved detect-then-solve: the solver re-measures contact geometry against each
        // substep's predicted positions via this callback (see Solver.step and
        // NarrowPhase.refreshManifoldGeometry), which is what keeps rotating/corner contacts stable.
        const narrowphase = this.narrowphase;
        this.solver.step(this.bodies, manifolds, this.gravity, dt, function (mans) {
            narrowphase.refreshManifoldGeometry(mans);
        }, this.constraints);

        // Continuous forces/torques (RigidBody.applyForce/applyTorque) stayed in effect for every
        // substep this tick (Solver._substep integrates accumulated_force/torque alongside gravity)
        // but do NOT persist into the next tick on their own - a caller who wants a force to keep
        // acting must call applyForce again next tick, the standard per-tick force contract. Cleared
        // here, once per tick, after the solver has already used this tick's value.
        for (let i = 0; i < this.bodies.length; i++) {
            const b = this.bodies[i];
            if (b.bodyType === RigidBody.DYNAMIC) b.clearForces();
        }

        // The solver moved bodies; their derived state (AABB, world inertia) is stale until the
        // NEXT tick's pass above runs. Nothing within this tick reads it again after this point,
        // so refreshing here would be wasted work - narrowphase/broadphase for THIS tick already
        // ran against the pre-solve state, which is correct (Rule 1: each stage assumes the state
        // handed to it, not a moving target updated out from under it mid-tick).
        this.emit('stepEnd', dt);
    }

    // rayIntersect(start, end) -> { body, point, normal, distance, fraction } | null. The first
    // body (if any) the segment start->end hits, via conservative advancement over GJK - see
    // Queries.js.
    rayIntersect(start, end) {
        return Queries.rayIntersect(this.bodies, start, end);
    }

    // shapeIntersect(shape, start, end, rotation) -> same result shape as rayIntersect. Sweeps
    // `shape` (held at a fixed `rotation`, identity if omitted) from start to end.
    shapeIntersect(shape, start, end, rotation) {
        return Queries.shapeIntersect(this.bodies, shape, start, end, rotation);
    }
}

ActionPhysics.World = World;
