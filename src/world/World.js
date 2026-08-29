/**
 * World: the pipeline glue. Owns the body list and drives one tick through every stage in order -
 * broadphase, midphase, narrowphase, solver - exactly the pipeline table in plan.md, nothing more.
 *
 * Public surface matches plan.md's API surface table: addRigidBody, removeRigidBody, step(dt),
 * gravity. addConstraint/removeConstraint/rayIntersect/shapeIntersect are later-stage features
 * (joints, queries) and are not implemented yet - present as documented no-ops would be worse than
 * absent, so they are simply not here until built for real.
 */
class World {
    constructor(broadphase, narrowphase, solver) {
        this.broadphase = broadphase;
        this.narrowphase = narrowphase;
        this.solver = solver;
        this.midphase = new Midphase();
        this.gravity = new Vector3(0, -9.81, 0);
        this.bodies = []; // all bodies, static and dynamic alike - broadphase filters by type itself
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
    // NarrowPhase.step, which owns calling into Midphase per plan.md's phases split) -> solver.
    // Every dynamic body's derived state (world AABB, world inverse inertia) is refreshed BEFORE
    // broadphase runs, so broadphase's own "AABBs are current" assumption (Rule 1) holds for
    // this tick's bodies, including ones the solver moved last tick.
    step(dt) {
        for (let i = 0; i < this.bodies.length; i++) this.bodies[i].updateDerived(dt);

        const pairs = this.broadphase.computePairs();
        const manifolds = this.narrowphase.step(pairs, this.midphase, dt);

        // Interleaved detect-then-solve: the solver re-measures contact geometry against each
        // substep's predicted positions via this callback (see Solver.step and
        // NarrowPhase.refreshManifoldGeometry), which is what keeps rotating/corner contacts stable.
        const narrowphase = this.narrowphase;
        this.solver.step(this.bodies, manifolds, this.gravity, dt, function (mans) {
            narrowphase.refreshManifoldGeometry(mans);
        });

        // The solver moved bodies; their derived state (AABB, world inertia) is stale until the
        // NEXT tick's pass above runs. Nothing within this tick reads it again after this point,
        // so refreshing here would be wasted work - narrowphase/broadphase for THIS tick already
        // ran against the pre-solve state, which is correct (Rule 1: each stage assumes the state
        // handed to it, not a moving target updated out from under it mid-tick).
    }
}

ActionPhysics.World = World;
