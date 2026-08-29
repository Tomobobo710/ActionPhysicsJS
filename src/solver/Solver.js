// XPBD solver (Muller et al. 2020). Velocity is derived from position (v = (x - x_prev) / h).
// Per substep: integrate -> refresh contact geometry -> reset lambdas -> solve positions ->
// derive velocity -> solve contact velocity. See Integrate/PositionSolve/VelocitySolve.
class Solver {
    constructor(opts) {
        opts = opts || {};
        this.substeps = opts.substeps || 4;
        this.iterations = opts.iterations || 4; // position-solve passes per substep

        this._rA = new Vector3(); this._rB = new Vector3();
        this._deltaPos = new Vector3();
        this._impulse = new Vector3();
        this._tangent1 = new Vector3(); this._tangent2 = new Vector3();
        this._angularCorrA = new Vector3(); this._angularCorrB = new Vector3();
        this._tmpDispA = new Vector3(); this._tmpDispB = new Vector3(); this._tmpPrev = new Vector3();
        this._prevPos = new Map();
        this._prevRot = new Map();
        this._preGravityVel = new Map();
        this._biasDelta = new Map(); // per-body bias-only correction this substep; excluded from derived velocity
        this._restRing = new Map(); // per-body ring buffer of recent transforms for rest-velocity reconciliation
    }

    // Widens what counts as "explainable by the body's own velocity" in _solvePoint's
    // position/velocity split (PositionSolve.js).
    static EXPLAINABLE_MARGIN = 3;

    // Advances dynamic bodies by dt, resolving manifolds and constraints. `refresh(manifolds)`, if
    // given, re-measures contact geometry each substep before the solve.
    step(bodies, manifolds, gravity, dt, refresh, constraints) {
        const h = dt / this.substeps;
        for (let s = 0; s < this.substeps; s++) {
            this._substep(bodies, manifolds, gravity, h, refresh, constraints);
        }
        this._reconcileRestVelocity(bodies, dt);
    }

    // Zeroes the velocity of a body whose sustained motion over the last REST_WINDOW ticks is below
    // the rest thresholds, from a per-body ring buffer of recent transforms. See NOTES.md.
    _reconcileRestVelocity(bodies, dt) {
        const win = REST_WINDOW;
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            let r = this._restRing.get(b.id);
            if (!r) {
                r = { pos: [], rot: [], head: 0, count: 0 };
                for (let k = 0; k < win; k++) { r.pos.push(new Vector3()); r.rot.push(new Quaternion()); }
                this._restRing.set(b.id, r);
            }

            if (r.count === win) {
                // Oldest sample is the one about to be overwritten at head.
                const oldPos = r.pos[r.head], oldRot = r.rot[r.head];
                const span = win * dt;
                const ndx = b.position.x - oldPos.x, ndy = b.position.y - oldPos.y, ndz = b.position.z - oldPos.z;
                const windowedLinSpeed = Math.sqrt(ndx * ndx + ndy * ndy + ndz * ndz) / span;
                if (windowedLinSpeed < REST_LINEAR_SPEED) b.linear_velocity.set(0, 0, 0);
                Solver._deriveAngularVelocity(this._tmpDispA, oldRot, b.rotation, span);
                if (this._tmpDispA.length() < REST_ANGULAR_SPEED) b.angular_velocity.set(0, 0, 0);
            } else {
                r.count++;
            }
            r.pos[r.head].copy(b.position);
            r.rot[r.head].copy(b.rotation);
            r.head = (r.head + 1) % win;
        }
    }

    _substep(bodies, manifolds, gravity, h, refresh, constraints) {
        this._integrate(bodies, gravity, h);

        if (refresh) refresh(manifolds);

        this._resetLambdas(manifolds);
        this._solvePositions(manifolds, constraints, h);
        this._deriveVelocities(bodies, h);
        this._solveContactVelocities(manifolds, gravity, h);
    }

    _resetLambdas(manifolds) {
        for (const manifold of manifolds.values()) {
            for (let i = 0; i < manifold.points.length; i++) {
                manifold.points[i].normalLambda = 0;
                manifold.points[i].tangentLambda1 = 0;
                manifold.points[i].tangentLambda2 = 0;
            }
        }
    }

    _solvePositions(manifolds, constraints, h) {
        for (let iter = 0; iter < this.iterations; iter++) {
            for (const manifold of manifolds.values()) {
                this._solveManifold(manifold, h);
            }
            if (constraints) {
                for (let i = 0; i < constraints.length; i++) {
                    if (constraints[i].enabled) constraints[i].solve(h);
                }
            }
        }
    }

    _solveManifold(manifold, h) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
        const n = manifold.points.length;
        if (n <= 1) {
            if (n === 1) this._solvePoint(manifold.points[0], bodyA, bodyB, h);
            return;
        }
        for (let i = 0; i < n; i++) {
            this._solvePoint(manifold.points[i], bodyA, bodyB, h, true);
        }
    }

    _solveContactVelocities(manifolds, gravity, h) {
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            // A flat face patch solves once at its centroid; everything else per-point.
            if (!this._boxFacePatchVelocity(manifold, bodyA, bodyB, gravity, h)) {
                for (let i = 0; i < manifold.points.length; i++) {
                    this._solveContactVelocity(manifold.points[i], bodyA, bodyB, gravity, h);
                }
            }
            if (manifold.points.length > 0) {
                // Angular friction acts at the most-engaged point.
                let ref = manifold.points[0];
                for (let i = 1; i < manifold.points.length; i++) {
                    if (Math.abs(manifold.points[i].normalLambda) > Math.abs(ref.normalLambda)) ref = manifold.points[i];
                }
                this._solveAngularFriction(ref, bodyA, bodyB, h);
            }
        }
    }
}

// Approach speeds below (gravityMag*h)*this don't bounce - suppresses a resting body's one-substep
// gravity nudge without a fixed absolute cutoff that would kill real small/slow bounces.
Solver.RESTITUTION_SLOP_FACTOR = 8;

// Largest single-point penetration a multi-point manifold resolves per substep (PositionSolve.js).
// The rest is picked up next substep, so one point's correction doesn't move the body before its
// siblings are read.
Solver.MAX_PENETRATION_PER_SUBSTEP = 0.005;

// Rest-velocity reconciliation thresholds (see Solver._reconcileRestVelocity, NOTES.md).
var REST_WINDOW = 8;              // ticks in the trailing velocity window
var REST_LINEAR_SPEED = 0.02;     // units/s windowed speed below which a settled body is zeroed
var REST_ANGULAR_SPEED = 0.05;    // rad/s windowed speed below which a settled body is zeroed

ActionPhysics.Solver = Solver;
