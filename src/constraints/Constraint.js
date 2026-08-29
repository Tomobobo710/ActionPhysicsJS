// Base class for the joints (Point, Hinge, Slider, Weld). Each computes its own position error C
// and corrects it with the contact solver's generalized-inverse-mass math; solve() runs once per
// substep before velocity is derived. All rigid, no compliance.
class Constraint {
    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB; // null = anchored to the world
        this.enabled = true;
    }
}

ActionPhysics.Constraint = Constraint;
