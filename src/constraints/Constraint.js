class Constraint {
    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.enabled = true;
    }
}

ActionPhysics.Constraint = Constraint;
