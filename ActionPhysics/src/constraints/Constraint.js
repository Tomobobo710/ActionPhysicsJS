/**
 * Constraint: base class for the user-facing joints (Point, Hinge, Slider, Weld).
 *
 * REBUILT, NOT PORTED (plan.md, "This is a rebuild, not a port"). Goblin's joints sit on top of
 * PGS's velocity-impulse machinery - Constraint/ConstraintRow/ConstraintLimit/ConstraintMotor, each
 * row an iteratively-solved velocity constraint with a Jacobian and a bias term. This engine has one
 * solver, XPBD (plan.md, "Solver: XPBD, one solver only"), which solves POSITION constraints
 * directly: a joint here computes its own position error (a vector or scalar C) and applies a
 * correction via the same generalized-inverse-mass math the contact solver already uses - there is
 * no row/Jacobian/bias abstraction to port, because XPBD does not have one. Carrying Goblin's row
 * shape over would be exactly the "port with different names" plan.md's rebuild rules warn against.
 *
 * A joint's solve() is called by the solver once per substep (same cadence as the contact solve),
 * inside the position-constraint loop, before velocity is derived - so a joint's effect shows up in
 * derived velocity for free, the same way a contact's does. No compliance/softness is implemented
 * yet (matches the contact solver's own current state - plan.md's open question on compliance is
 * still open); every joint here is rigid.
 */
class Constraint {
    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB; // may be null: a joint anchored to the world (bodyA pinned in space)
        this.enabled = true;
        // Warm-start accumulators, reset per substep like a contact's normalLambda - carried across
        // TICKS via nothing (a joint has no manifold to warm-start from between ticks the way a
        // contact does; its own accumulated lambda within a tick's substeps is enough for XPBD's
        // convergence, matching the contact solver's own per-substep reset discipline).
    }
}

ActionPhysics.Constraint = Constraint;
