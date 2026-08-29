/**
 * XPBD solver (Muller et al. 2020 "Detailed Rigid Body Simulation with Extended Position Based
 * Dynamics"; Macklin et al. for the compliance formulation).
 *
 * Velocity is DERIVED from position (v = (x - x_prev) / h), never clamped. Each substep runs:
 * integrate -> refresh contact geometry -> reset lambdas -> solve positions -> derive velocity ->
 * solve contact velocity (friction/restitution/rolling). See Integrate.js, Geometry.js,
 * PositionSolve.js, VelocitySolve.js for each phase.
 */
class Solver {
    constructor(opts) {
        opts = opts || {};
        this.substeps = opts.substeps || 4;
        this.iterations = opts.iterations || 1; // position-solve passes per substep

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
        this._deferredRotation = new Map(); // per-body accumulated small-angle rotation for a multi-point manifold pass
    }

    // Margin widening what counts as "explainable by the body's own velocity" in _solvePoint's
    // position/velocity split (see PositionSolve.js). 3x covers the ordinary numerical gap for
    // off-center/rotating contacts without letting a real zero-velocity spawn overlap through.
    static EXPLAINABLE_MARGIN = 3;

    /**
     * Advances every dynamic body in `bodies` by `dt`, resolving `manifolds` (a ContactManifoldList)
     * and `constraints` (joints). `refresh(manifolds)`, if given, re-measures each substep's contact
     * geometry against predicted positions before the solve (see Geometry.js).
     */
    step(bodies, manifolds, gravity, dt, refresh, constraints) {
        const h = dt / this.substeps;
        for (let s = 0; s < this.substeps; s++) {
            this._substep(bodies, manifolds, gravity, h, refresh, constraints);
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
        // Multiple points: defer each point's angular correction (see _applyAngularCorrection).
        // Quaternion composition doesn't commute, so several small rotations applied one at a time
        // don't cancel out even when their axis-angle vectors sum to zero by symmetry - summing
        // first and applying once removes that.
        const defer = this._deferredRotation;
        for (let i = 0; i < n; i++) {
            this._solvePoint(manifold.points[i], bodyA, bodyB, h, defer, true);
        }
        this._flushDeferredRotation(bodyA, defer);
        this._flushDeferredRotation(bodyB, defer);
    }

    _solveContactVelocities(manifolds, gravity, h) {
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            for (let i = 0; i < manifold.points.length; i++) {
                this._solveContactVelocity(manifold.points[i], bodyA, bodyB, gravity, h);
            }
            if (manifold.points.length > 0) {
                // Reference point for rolling resistance: the most-engaged one (largest |normalLambda|),
                // not always points[0] - see Contacts.js._solveRollingResistance.
                let ref = manifold.points[0];
                for (let i = 1; i < manifold.points.length; i++) {
                    if (Math.abs(manifold.points[i].normalLambda) > Math.abs(ref.normalLambda)) ref = manifold.points[i];
                }
                this._solveRollingResistance(ref, bodyA, bodyB, h);
            }
        }
    }
}

// Restitution slop multiplier: an approach speed below (gravityMag*h)*this factor doesn't bounce -
// keeps a resting body's own one-substep gravity nudge from becoming perpetual micro-jitter,
// scaled to gravity/timestep instead of a fixed absolute speed so it stays correct across body
// scale (a fixed threshold silently killed real small/slow bounces - e.g. a marble dropped 5mm hit
// the floor at 0.31 m/s, a genuine restitution-worthy impact, and got fully suppressed under a
// flat 0.5 m/s cutoff).
Solver.RESTITUTION_SLOP_FACTOR = 8;

// Largest single-point penetration (C) a multi-point manifold's position-solve resolves in one
// substep (see PositionSolve.js's cappedC; never applied to a single-point manifold). The
// remainder is real, live-measured penetration, picked up on the next substep instead of all at
// once - keeps one point's correction from moving the body before its manifold siblings are read.
Solver.MAX_PENETRATION_PER_SUBSTEP = 0.005;

ActionPhysics.Solver = Solver;
