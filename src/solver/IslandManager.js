// Decides which bodies are asleep each tick, as coupled groups (islands), parking or waking whole
// islands together. Per-body sleep is wrong for stacks: the bottom body sleeps while the top is
// still settling and sags into it. Two dynamic bodies are coupled by a contact manifold or an
// enabled constraint; static/kinematic bodies are boundaries, not links (or the floor would chain
// the whole world into one island). Runs after narrowphase, before the solver.
class IslandManager {
    // A body is "quiet" this tick when both speeds are below these. The angular threshold sits well
    // above the ~0.071 rad/s band a side-resting cylinder oscillates in forever, so it doesn't
    // sleep/wake on the boundary.
    static LINEAR_SLEEP_THRESHOLD = 0.05;        // m/s
    static ANGULAR_SLEEP_THRESHOLD = 0.12;       // rad/s

    // Seconds an entire island must stay quiet before it parks.
    static TIME_TO_SLEEP = 0.5;

    constructor() {
        this._parent = new Map();  // union-find, rebuilt each tick: bodyId -> bodyId
        this._islands = new Map(); // island root id -> { members, allQuiet }, rebuilt each tick
    }

    _find(id) {
        let root = id;
        while (this._parent.get(root) !== root) root = this._parent.get(root);
        // Path compression.
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

    // Updates sleep state for every dynamic body. After this runs, isAwake === false means the
    // solver may skip the body this tick.
    update(bodies, manifolds, constraints, dt) {
        this._parent.clear();
        this._islands.clear();

        // 1. Seed the forest with every dynamic body as a singleton.
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType === RigidBody.DYNAMIC) this._ensure(b.id);
        }

        // 2. Union dynamic bodies coupled by a contact.
        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            const aDyn = a.bodyType === RigidBody.DYNAMIC, bDyn = b.bodyType === RigidBody.DYNAMIC;
            if (aDyn && bDyn) this._union(a.id, b.id);
        }

        // 3. Union dynamic bodies coupled by an enabled constraint (bodyB null = world-anchored, no union).
        if (constraints) {
            for (let i = 0; i < constraints.length; i++) {
                const c = constraints[i];
                if (!c.enabled || !c.bodyB) continue;
                const aDyn = c.bodyA.bodyType === RigidBody.DYNAMIC, bDyn = c.bodyB.bodyType === RigidBody.DYNAMIC;
                if (aDyn && bDyn) this._union(c.bodyA.id, c.bodyB.id);
            }
        }

        // 4. Force awake any dynamic body touching a moving kinematic body or an awake dynamic one,
        //    regardless of its own speed. A static contact is not a forcing influence.
        const forcedAwake = new Set();
        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            IslandManager._maybeForceAwakeFromNeighbor(a, b, forcedAwake);
            IslandManager._maybeForceAwakeFromNeighbor(b, a, forcedAwake);
        }

        // 5. Group by island root, recording each body's quietness.
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

        // 6. Park an island once every member has been quiet past TIME_TO_SLEEP; wake the whole
        //    island if any member is restless.
        for (const island of this._islands.values()) {
            if (island.allQuiet) {
                let minTimer = Infinity;
                for (const body of island.members) {
                    body.sleepTimer += dt;
                    if (body.sleepTimer < minTimer) minTimer = body.sleepTimer;
                }
                if (minTimer >= IslandManager.TIME_TO_SLEEP) {
                    for (const body of island.members) body.sleep();
                }
            } else {
                for (const body of island.members) {
                    if (!body.isAwake) body.isAwake = true;
                    body.sleepTimer = 0;
                }
            }
        }
    }

    // Force `body` awake if `other` is a moving kinematic body or an awake dynamic one.
    static _maybeForceAwakeFromNeighbor(body, other, forcedAwake) {
        if (body.bodyType !== RigidBody.DYNAMIC) return;
        if (other.bodyType === RigidBody.KINEMATIC) {
            if (!IslandManager._isQuiet(other)) forcedAwake.add(body.id);
        } else if (other.bodyType === RigidBody.DYNAMIC) {
            if (other.isAwake && !IslandManager._isQuiet(other)) forcedAwake.add(body.id);
        }
    }

    static _isQuiet(body) {
        const lv = body.linear_velocity, av = body.angular_velocity;
        const linSq = lv.x * lv.x + lv.y * lv.y + lv.z * lv.z;
        const angSq = av.x * av.x + av.y * av.y + av.z * av.z;
        return linSq <= IslandManager.LINEAR_SLEEP_THRESHOLD * IslandManager.LINEAR_SLEEP_THRESHOLD &&
            angSq <= IslandManager.ANGULAR_SLEEP_THRESHOLD * IslandManager.ANGULAR_SLEEP_THRESHOLD;
    }

    static _indexById(bodies) {
        const map = new Map();
        for (let i = 0; i < bodies.length; i++) map.set(bodies[i].id, bodies[i]);
        return map;
    }
}

ActionPhysics.IslandManager = IslandManager;
