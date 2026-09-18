class IslandManager {

    static LINEAR_SLEEP_THRESHOLD = 0.05;
    static ANGULAR_SLEEP_THRESHOLD = 0.12;

    static TIME_TO_SLEEP = 0.5;

    static WAKE_ON_MOVE_LINEAR = 1e-5;
    static WAKE_ON_MOVE_ANGULAR = 1e-5;

    constructor() {
        this._parent = new Map();
        this._islands = new Map();
        this._sleepGravity = new Vector3(0, 0, 0);
        this._lastPose = new Map();
    }

    _find(id) {
        let root = id;
        while (this._parent.get(root) !== root) root = this._parent.get(root);

        let cur = id;
        while (this._parent.get(cur) !== root) {
            const next = this._parent.get(cur);
            this._parent.set(cur, root);
            cur = next;
        }
        return root;
    }

    _union(a, b) {
        const ra = this._find(a), rb = this._find(b);
        if (ra !== rb) this._parent.set(ra, rb);
    }

    _ensure(id) {
        if (!this._parent.has(id)) this._parent.set(id, id);
    }

    update(bodies, manifolds, constraints, dt, gravity) {
        this._parent.clear();
        this._islands.clear();

        const gravityChanged = gravity && !IslandManager._vecApproxEqual(gravity, this._sleepGravity);
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC || b.isAwake) continue;
            if (gravityChanged || IslandManager._movedSinceSleep(b) || IslandManager._velocitySetWhileAsleep(b)) b.wakeUp();
        }

        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType === RigidBody.DYNAMIC) this._ensure(b.id);
        }

        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            const aDyn = a.bodyType === RigidBody.DYNAMIC, bDyn = b.bodyType === RigidBody.DYNAMIC;
            if (aDyn && bDyn) this._union(a.id, b.id);
        }

        if (constraints) {
            for (let i = 0; i < constraints.length; i++) {
                const c = constraints[i];
                if (!c.enabled || !c.bodyB) continue;
                const aDyn = c.bodyA.bodyType === RigidBody.DYNAMIC, bDyn = c.bodyB.bodyType === RigidBody.DYNAMIC;
                if (aDyn && bDyn) this._union(c.bodyA.id, c.bodyB.id);
            }
        }

        const forcedAwake = new Set();
        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            IslandManager._maybeForceAwakeFromNeighbor(a, b, forcedAwake);
            IslandManager._maybeForceAwakeFromNeighbor(b, a, forcedAwake);
        }

        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            let last = this._lastPose.get(b.id);
            if (!last) {
                last = { pos: new Vector3(), rot: new Quaternion(), valid: false };
                this._lastPose.set(b.id, last);
            }
            if (last.valid && dt > 0) {
                const dx = b.position.x - last.pos.x, dy = b.position.y - last.pos.y, dz = b.position.z - last.pos.z;
                b._tickLinearSpeed = Math.sqrt(dx * dx + dy * dy + dz * dz) / dt;
                b._tickAngularSpeed = IslandManager._angleBetween(last.rot, b.rotation) / dt;
            } else {

                const lv = b.linear_velocity, av = b.angular_velocity;
                b._tickLinearSpeed = Math.sqrt(lv.x * lv.x + lv.y * lv.y + lv.z * lv.z);
                b._tickAngularSpeed = Math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z);
            }
            last.pos.copy(b.position);
            last.rot.copy(b.rotation);
            last.valid = true;
        }

        const bodyById = IslandManager._indexById(bodies);
        for (const [id, ] of this._parent) {
            const root = this._find(id);
            let island = this._islands.get(root);
            if (!island) { island = { members: [], allQuiet: true }; this._islands.set(root, island); }
            const body = bodyById.get(id);
            island.members.push(body);
            const quiet = !forcedAwake.has(id) && IslandManager._isQuiet(body);
            if (!quiet) island.allQuiet = false;
        }

        for (const island of this._islands.values()) {
            if (island.allQuiet) {
                let minTimer = Infinity;
                for (const body of island.members) {
                    body.sleepTimer += dt;
                    if (body.sleepTimer < minTimer) minTimer = body.sleepTimer;
                }
                if (minTimer >= IslandManager.TIME_TO_SLEEP) {
                    for (const body of island.members) {
                        body.sleep();
                        IslandManager._snapshotForSleep(body);
                    }
                    if (gravity) this._sleepGravity.copy(gravity);
                }
            } else {
                for (const body of island.members) {
                    if (!body.isAwake) body.wakeUp();
                    body.sleepTimer = 0;
                }
            }
        }
    }

    static wakeTouching(body, manifolds, constraints) {
        if (body.bodyType === RigidBody.DYNAMIC) body.wakeUpFromWorldChange();
        if (manifolds) {
            for (const manifold of manifolds.values()) {
                if (manifold.bodyA === body) IslandManager._wakeIfDynamic(manifold.bodyB);
                else if (manifold.bodyB === body) IslandManager._wakeIfDynamic(manifold.bodyA);
            }
        }
        if (constraints) {
            for (let i = 0; i < constraints.length; i++) {
                const c = constraints[i];
                if (c.bodyA === body) IslandManager._wakeIfDynamic(c.bodyB);
                else if (c.bodyB === body) IslandManager._wakeIfDynamic(c.bodyA);
            }
        }
    }

    static _wakeIfDynamic(body) {
        if (body && body.bodyType === RigidBody.DYNAMIC) body.wakeUpFromWorldChange();
    }

    static _snapshotForSleep(body) {
        if (!body._sleepPos) { body._sleepPos = new Vector3(); body._sleepRot = new Quaternion(); }
        body._sleepPos.copy(body.position);
        body._sleepRot.copy(body.rotation);
    }

    static _velocitySetWhileAsleep(body) {
        const lv = body.linear_velocity, av = body.angular_velocity;
        return lv.x !== 0 || lv.y !== 0 || lv.z !== 0 || av.x !== 0 || av.y !== 0 || av.z !== 0;
    }

    static _movedSinceSleep(body) {
        const s = body._sleepPos;
        if (!s) return false;
        const dx = body.position.x - s.x, dy = body.position.y - s.y, dz = body.position.z - s.z;
        if (dx * dx + dy * dy + dz * dz > IslandManager.WAKE_ON_MOVE_LINEAR * IslandManager.WAKE_ON_MOVE_LINEAR) return true;
        const q = body.rotation, r = body._sleepRot;
        const dot = q.x * r.x + q.y * r.y + q.z * r.z + q.w * r.w;
        return 1 - Math.abs(dot) > IslandManager.WAKE_ON_MOVE_ANGULAR;
    }

    static _vecApproxEqual(a, b) {
        const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz < 1e-12;
    }

    static _maybeForceAwakeFromNeighbor(body, other, forcedAwake) {
        if (body.bodyType !== RigidBody.DYNAMIC) return;
        if (other.bodyType === RigidBody.KINEMATIC) {
            if (!IslandManager._isQuiet(other)) forcedAwake.add(body.id);
        } else if (other.bodyType === RigidBody.DYNAMIC) {
            if (other.isAwake && !IslandManager._isQuiet(other)) forcedAwake.add(body.id);
        }
    }

    static _isQuiet(body) {
        const lin = body._tickLinearSpeed, ang = body._tickAngularSpeed;
        if (lin === undefined) {
            const lv = body.linear_velocity, av = body.angular_velocity;
            const linSq = lv.x * lv.x + lv.y * lv.y + lv.z * lv.z;
            const angSq = av.x * av.x + av.y * av.y + av.z * av.z;
            return linSq <= IslandManager.LINEAR_SLEEP_THRESHOLD * IslandManager.LINEAR_SLEEP_THRESHOLD &&
                angSq <= IslandManager.ANGULAR_SLEEP_THRESHOLD * IslandManager.ANGULAR_SLEEP_THRESHOLD;
        }
        return lin <= IslandManager.LINEAR_SLEEP_THRESHOLD && ang <= IslandManager.ANGULAR_SLEEP_THRESHOLD;
    }

    static _angleBetween(qa, qb) {
        let dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
        if (dot < 0) dot = -dot;
        if (dot > 1) dot = 1;
        return 2 * Scalar.acos(dot);
    }

    static _indexById(bodies) {
        const map = new Map();
        for (let i = 0; i < bodies.length; i++) map.set(bodies[i].id, bodies[i]);
        return map;
    }
}

ActionPhysics.IslandManager = IslandManager;
