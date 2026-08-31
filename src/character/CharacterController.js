/**
 * Spring-based character controller: a capsule body held at a fixed ride height above the ground
 * by a raycast spring, with slope-projected movement and a small falling/grounded/jumping state
 * machine. This is the smaller of ActionPhysics's two character controllers (the other is the
 * FPS controller) — see the design plan's Component 10 for why they are independent, not a
 * base + specialization.
 *
 * Uses the World/RigidBody privileged interface directly (raw force application, direct velocity
 * writes) rather than the public gameplay surface — a character controller is documented as
 * needing that access.
 */
class CharacterController {
    constructor(world, options) {
        this.world = world;
        options = options || {};

        const radius = options.radius || 2;
        const totalHeight = options.height || 6;
        this.shape = new CapsuleShape(radius, totalHeight);
        this.body = new RigidBody(this.shape, options.mass || 1);

        this.body.angular_factor = options.allowYRotation === false
            ? new Vector3(0, 0, 0)
            : new Vector3(0, 1, 0);

        // Movement configuration
        this.moveSpeed = options.moveSpeed || 50;
        this.maxSpeed = options.maxSpeed || 50;
        this.stopFactor = options.stopFactor || 0.9;
        this.stoppingThreshold = options.stoppingThreshold || 0.1;
        this.jumpForce = options.jumpForce || 60;
        this.airAcceleration = options.airAcceleration || 0.3;
        this.groundAcceleration = options.groundAcceleration || 0.3;

        // Input handling
        this._inputDirection = new Vector3();
        this._hasInputThisFrame = false;
        this._jumpRequested = false;

        // Working vectors
        this.contactNormal = new Vector3(0, 1, 0);
        this.tempVector = new Vector3();
        this.moveVector = new Vector3();
        this.projectedMove = new Vector3();

        // Ground spring config. Once the capsule touches the ground the solver's own contact
        // constraint pins it there regardless of spring force, so the spring must stop a fall before
        // the shape reaches the surface - hence a stiff springStrength default. springDamping is set
        // near critical damping for that strength (2*sqrt(strength*mass)), not scaled proportionally
        // with it - proportional scaling is underdamped and bounces for a long time before settling.
        this.rideHeight = options.rideHeight || 4;
        this.rayLength = options.rayLength || totalHeight;
        this.springStrength = options.springStrength || 300;
        this.springDamping = options.springDamping || 30;

        // State management
        this.states = {};
        this.currentState = null;
        this._lastStateChange = { from: null, to: null, time: Date.now() };

        // Debug tracking
        this._lastGroundHit = null;
        this._lastHeightError = null;
        this._lastSpringForce = null;
        this._lastMoveDelta = new Vector3();
        this._lastProjectedMove = new Vector3();
        this._lastAppliedForce = null;

        this._listeners = {};

        this._initializeStates();
        this.changeState('falling');
    }

    _initializeStates() {
        this.states.falling = {
            name: 'falling',
            enter: () => {},
            update: (deltaTime) => {
                this.updateGroundSpring();
                if (this._hasInputThisFrame) this.move(this._inputDirection, deltaTime);
                if (this._lastGroundHit) return 'grounded';
            },
            exit: () => {}
        };

        this.states.grounded = {
            name: 'grounded',
            enter: () => {},
            update: (deltaTime) => {
                this.updateGroundSpring();

                if (this._jumpRequested) {
                    this._jumpRequested = false;
                    return 'jumping';
                }

                if (this._hasInputThisFrame) {
                    this.move(this._inputDirection, deltaTime);
                } else {
                    const vx = this.body.linear_velocity.x, vz = this.body.linear_velocity.z;
                    const currentHorizontalSpeed = Math.sqrt(vx * vx + vz * vz);
                    if (currentHorizontalSpeed > this.stoppingThreshold) {
                        this.body.linear_velocity.x *= this.stopFactor;
                        this.body.linear_velocity.z *= this.stopFactor;
                    } else {
                        this.body.linear_velocity.x = 0;
                        this.body.linear_velocity.z = 0;
                    }
                }

                if (!this._lastGroundHit) return 'falling';
            },
            exit: () => {}
        };

        this.states.jumping = {
            name: 'jumping',
            enter: () => {
                this.body.linear_velocity.y = this.jumpForce;
            },
            update: (deltaTime) => {
                if (this._hasInputThisFrame) this.move(this._inputDirection, deltaTime);
                if (this.body.linear_velocity.y <= 0) return 'falling';
            },
            exit: () => {}
        };
    }

    /** Requests a jump; only takes effect while grounded. */
    wishJump() {
        if (this.currentState && this.currentState.name === 'grounded') {
            this._jumpRequested = true;
        }
    }

    changeState(newStateName) {
        const newState = this.states[newStateName];
        if (!newState) throw new Error('Invalid state: ' + newStateName);

        if (this.currentState) this.currentState.exit();

        this._lastStateChange = {
            from: this.currentState ? this.currentState.name : null,
            to: newState.name,
            time: Date.now()
        };

        this.currentState = newState;
        this.currentState.enter();
    }

    /** Stores input direction for processing during update(). */
    handleInput(direction) {
        if (direction && direction.lengthSquared() > 0) {
            this._inputDirection.copy(direction);
            this._hasInputThisFrame = true;
        } else {
            this._inputDirection.set(0, 0, 0);
            this._hasInputThisFrame = false;
        }
    }

    /** Advances the state machine. Call once per frame after handleInput(). */
    update(deltaTime) {
        if (this.currentState) {
            const nextState = this.currentState.update(deltaTime);
            if (nextState && nextState !== this.currentState.name) {
                this.changeState(nextState);
            }
        }
        this._hasInputThisFrame = false;
    }

    /**
     * Raycasts straight down from the capsule's base and applies a spring force to hold the body
     * at rideHeight above whatever it hits. Call while in FALLING or GROUNDED.
     */
    updateGroundSpring() {
        const halfHeight = this.shape.totalHeight / 2;
        const rayStart = new Vector3(
            this.body.position.x,
            this.body.position.y - halfHeight - 0.00001,
            this.body.position.z
        );
        const rayEnd = new Vector3(rayStart.x, rayStart.y - this.rayLength, rayStart.z);

        const hit = this.world.rayIntersect(rayStart, rayEnd, this.body);

        if (hit) {
            this._lastGroundHit = hit;
            this.contactNormal.copy(hit.normal);

            const heightError = this.rideHeight - hit.distance;
            const verticalVelocity = this.body.linear_velocity.y;
            const springForce = (heightError * this.springStrength) - (verticalVelocity * this.springDamping);

            this._lastHeightError = heightError;
            this._lastSpringForce = springForce;
            this._lastAppliedForce = { x: 0, y: springForce, z: 0 };

            this.body.applyForce(new Vector3(0, springForce, 0));
        } else {
            this._lastGroundHit = null;
            this._lastHeightError = null;
            this._lastSpringForce = null;
            this._lastAppliedForce = null;
            this.contactNormal.set(0, 1, 0);
        }
    }

    /** Projects `direction` onto the current ground (or air) and drives velocity toward it. */
    move(direction, deltaTime) {
        this.moveVector.copy(direction);
        this.moveVector.scaleInPlace(this.moveSpeed);
        this._lastMoveDelta.copy(this.moveVector);

        if (this.currentState.name === 'falling' || this.currentState.name === 'jumping') {
            const currentY = this.body.linear_velocity.y;
            this.body.linear_velocity.x += (this.moveVector.x - this.body.linear_velocity.x) * this.airAcceleration;
            this.body.linear_velocity.z += (this.moveVector.z - this.body.linear_velocity.z) * this.airAcceleration;
            this.body.linear_velocity.y = currentY;
        } else {
            const dot = this.moveVector.dot(this.contactNormal);
            this.projectedMove.copy(this.moveVector);
            this.tempVector.copy(this.contactNormal);
            this.tempVector.scaleInPlace(dot);
            this.projectedMove.subInPlace(this.tempVector);
            this._lastProjectedMove.copy(this.projectedMove);

            this.body.linear_velocity.x += (this.projectedMove.x - this.body.linear_velocity.x) * this.groundAcceleration;
            this.body.linear_velocity.z += (this.projectedMove.z - this.body.linear_velocity.z) * this.groundAcceleration;
        }

        const vx = this.body.linear_velocity.x, vz = this.body.linear_velocity.z;
        const currentSpeed = Math.sqrt(vx * vx + vz * vz);
        if (currentSpeed > this.maxSpeed) {
            const scale = this.maxSpeed / currentSpeed;
            this.body.linear_velocity.x *= scale;
            this.body.linear_velocity.z *= scale;
        }
    }

    /** Everything about current movement state, forces and contacts — for debugging/inspection. */
    getDebugInfo() {
        return {
            physics: {
                position: { x: this.body.position.x, y: this.body.position.y, z: this.body.position.z },
                velocity: { x: this.body.linear_velocity.x, y: this.body.linear_velocity.y, z: this.body.linear_velocity.z }
            },
            movement: {
                input_direction: this._hasInputThisFrame
                    ? { x: this._inputDirection.x, y: this._inputDirection.y, z: this._inputDirection.z }
                    : null,
                raw_move: { x: this._lastMoveDelta.x, y: this._lastMoveDelta.y, z: this._lastMoveDelta.z },
                projected_move: { x: this._lastProjectedMove.x, y: this._lastProjectedMove.y, z: this._lastProjectedMove.z },
                applied_force: this._lastAppliedForce
            },
            spring: {
                hit_distance: this._lastGroundHit ? this._lastGroundHit.distance : null,
                height_error: this._lastHeightError,
                spring_force: this._lastSpringForce,
                target_height: this.rideHeight,
                spring_strength: this.springStrength,
                spring_damping: this.springDamping
            },
            contact: {
                normal: { x: this.contactNormal.x, y: this.contactNormal.y, z: this.contactNormal.z },
                hit: this._lastGroundHit
                    ? { point: this._lastGroundHit.point, normal: this._lastGroundHit.normal, distance: this._lastGroundHit.distance }
                    : null
            },
            state: { current: this.currentState ? this.currentState.name : null, lastTransition: this._lastStateChange }
        };
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
}

ActionPhysics.CharacterController = CharacterController;
