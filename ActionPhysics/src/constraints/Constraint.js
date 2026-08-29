/**
 * Constraint: base class for the user-facing joints (Point, Hinge, Slider, Weld).
 *
 * A joint computes its own position error (a vector or scalar C) and applies the correction via
 * the same generalized-inverse-mass math the contact solver already uses. A joint's solve() runs
 * once per substep inside the position-constraint loop, before velocity is derived - so a joint's
 * effect shows up in derived velocity for free, the same way a contact's does. No
 * compliance/softness yet; every joint here is rigid.
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
