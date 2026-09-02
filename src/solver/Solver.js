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
    // the rest thresholds, from a per-body ring buffer of recent transforms. Once a body has stayed
    // that quiet for REST_PIN_STREAK consecutive ticks it is also transform-pinned: each tick's
    // residual drift is reverted to the previous sampled pose. The per-point Gauss-Seidel contact
    // solve leaks a little tangential drift every substep for non-box shapes (box patches are already
    // centroid-solved, see VelocitySolve.js), so a "settled" cylinder/cone/sphere slowly walks across
    // its support with its reported velocity reading zero. The streak gate keeps this off any body
    // that is only briefly quiet - a rider settling onto a carrier, a shape between bounces - so only
    // a genuinely parked body gets pinned, and a sleeping body then matches a never-slept one exactly.
    // See NOTES.md.
    _reconcileRestVelocity(bodies, dt) {
        const win = REST_WINDOW;
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC || !b.isAwake) continue;

            // A body woken by a world change still looks quiet to the ring (it holds the pose it
            // slept in), so the ring would zero its fresh gravity and snap it back every tick.
            // Drop it; it rebuilds from scratch and can't re-pin until still for a full window.
            // Routine wakes (impulse, contact, island restless) don't set the flag.
            if (b._restRingStale) {
                b._restRingStale = false;
                this._restRing.delete(b.id);
            }
            let r = this._restRing.get(b.id);
            if (!r) {
                r = { pos: [], rot: [], head: 0, count: 0, quietStreak: 0,
                      pinPos: new Vector3(), pinRot: new Quaternion(), pinned: false };
                for (let k = 0; k < win; k++) { r.pos.push(new Vector3()); r.rot.push(new Quaternion()); }
                this._restRing.set(b.id, r);
            }

            if (r.count === win) {
                // Oldest sample is the one about to be overwritten at head; newest was written last tick.
                const oldPos = r.pos[r.head], oldRot = r.rot[r.head];
                const span = win * dt;
                const ndx = b.position.x - oldPos.x, ndy = b.position.y - oldPos.y, ndz = b.position.z - oldPos.z;
                const windowedLinSpeed = Math.sqrt(ndx * ndx + ndy * ndy + ndz * ndz) / span;
                const linQuiet = windowedLinSpeed < REST_LINEAR_SPEED;
                Solver._deriveAngularVelocity(this._tmpDispA, oldRot, b.rotation, span);
                const angQuiet = this._tmpDispA.length() < REST_ANGULAR_SPEED;

                if (linQuiet) b.linear_velocity.set(0, 0, 0);
                if (angQuiet) b.angular_velocity.set(0, 0, 0);

                const disturbed = b._restDisturbed;
                b._restDisturbed = false;
                if (disturbed) { r.quietStreak = 0; r.pinned = false; }

                if (linQuiet && angQuiet && !disturbed) {
                    r.quietStreak++;
                    if (r.quietStreak >= REST_PIN_STREAK) {
                        if (!r.pinned) {
                            // First pinned tick: capture the pose to hold. Use the oldest ring sample
                            // (REST_WINDOW ticks back) - it predates most of the drift accumulated
                            // during this quiet stretch, so holding it cancels the walk rather than
                            // freezing wherever the walk had reached.
                            r.pinPos.copy(oldPos);
                            r.pinRot.copy(oldRot);
                            r.pinned = true;
                        }
                        b.position.copy(r.pinPos);
                        b.rotation.copy(r.pinRot);
                    }
                } else {
                    r.quietStreak = 0;
                    r.pinned = false;
                }
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

        this._markRestDisturbances(manifolds);
        this._resetLambdas(manifolds);
        this._solvePositions(manifolds, constraints, h);
        this._deriveVelocities(bodies, h);
        this._solveContactVelocities(manifolds, gravity, h);
    }

    // Flags a dynamic body as disturbed when it shares a touching manifold with an externally-driven
    // mover (see _bodyIsMoving). _reconcileRestVelocity reads the flag to break the body's rest pin
    // the same tick it is pushed, rather than waiting out the trailing window while the pin holds the
    // push out (which reads as an "unpushable" resting object).
    _markRestDisturbances(manifolds) {
        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            let touching = false;
            for (let i = 0; i < manifold.points.length; i++) {
                if (manifold.points[i].signedDistance >= -REST_TOUCH_BAND) { touching = true; break; }
            }
            if (!touching) continue;
            if (a.bodyType === RigidBody.DYNAMIC && Solver._bodyIsMoving(b)) a._restDisturbed = true;
            if (b.bodyType === RigidBody.DYNAMIC && Solver._bodyIsMoving(a)) b._restDisturbed = true;
        }
    }

    // Is `body` an externally-driven mover whose contact should break a resting neighbour's pin? A
    // moving kinematic (platform), or a character controller's velocity-driven ghost body (dynamic in
    // type but commanded every tick, flagged isKinematicCharacter), both qualify while actually
    // moving. A plain dynamic body does NOT: a still-settling pile carries residual velocity that must
    // not chatter its neighbours' pins, and a real dynamic-on-dynamic impact wakes and unpins through
    // the ordinary sleep/streak path.
    static _bodyIsMoving(body) {
        const driven = body.bodyType === RigidBody.KINEMATIC || body.isKinematicCharacter === true;
        if (!driven) return false;
        const lv = body.linear_velocity, av = body.angular_velocity;
        return lv.x * lv.x + lv.y * lv.y + lv.z * lv.z > REST_LINEAR_SPEED * REST_LINEAR_SPEED ||
            av.x * av.x + av.y * av.y + av.z * av.z > REST_ANGULAR_SPEED * REST_ANGULAR_SPEED;
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

    // A contact can only do work if at least one of its bodies is free to move: an awake dynamic
    // body. Two static bodies, or a sleeping body against a static one, form an inert manifold - and
    // solving it anyway lets sub-micron narrowphase drift accumulate into the sleeping body's
    // resting-surface penetration, which then discharges as a position pop when it wakes. A sleeping
    // body against an AWAKE dynamic one is not inert here, but the island manager has already woken
    // it this tick (a touching contact with a restless neighbour force-wakes), so that case does not
    // actually reach the solver with one side still asleep.
    static _manifoldIsInert(bodyA, bodyB) {
        const aFree = bodyA.bodyType === RigidBody.DYNAMIC && bodyA.isAwake;
        const bFree = bodyB.bodyType === RigidBody.DYNAMIC && bodyB.isAwake;
        return !aFree && !bFree;
    }

    _solveManifold(manifold, h) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
        if (Solver._manifoldIsInert(bodyA, bodyB)) return;
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
            if (Solver._manifoldIsInert(bodyA, bodyB)) continue;
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
var REST_PIN_STREAK = 12;         // consecutive fully-quiet ticks before a body's transform is pinned
var REST_TOUCH_BAND = 0.005;      // gap (m) within which a manifold point counts as real contact

ActionPhysics.Solver = Solver;
