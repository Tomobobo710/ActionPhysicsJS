/**
 * IslandManager: decides which bodies are ASLEEP each tick, as coupled groups (islands), and parks
 * or wakes whole islands together.
 *
 * WHY ISLANDS, NOT PER-BODY SLEEP: the real target is a game - stacks, piles, ragdolls, debris -
 * where resting bodies rest ON each other. Per-body sleep is not a smaller-but-correct version there,
 * it is WRONG: the bottom crate of a stack reaches rest and sleeps a moment before the crate on top
 * has finished micro-settling; the sleeping bottom body stops being integrated and stops resolving
 * its contact, and the still-awake top body sags into (or through) it. The fix is that a body may
 * sleep only as part of a group where EVERY member is ready to sleep, and the whole group wakes the
 * instant any member is disturbed. Building that grouping IS the island manager.
 *
 * WHAT COUPLES TWO BODIES INTO ONE ISLAND: a contact manifold with at least one point, or an enabled
 * constraint, BETWEEN TWO DYNAMIC BODIES. Static and kinematic bodies deliberately do NOT propagate
 * membership - the floor (static) touches every resting body in a scene, and chaining all of them
 * into one island through the floor would mean no pile could ever sleep unless the entire world did.
 * A static/kinematic body is a boundary, not a link. (A dynamic body resting on a MOVING kinematic
 * platform is kept awake separately - see step() - since the ground under it is itself in motion.)
 *
 * ORDER IN THE PIPELINE: runs in World.step AFTER narrowphase (so it has this tick's manifolds to
 * build connectivity from) and BEFORE the solver (so the solver can skip sleeping islands entirely).
 * The manifolds it reads are the same ContactManifoldList the solver then reads - zero-point
 * manifolds are already pruned by ContactManifoldList.refresh, so every manifold here is a genuine
 * touch.
 */
class IslandManager {
    // Sleep thresholds. A body is "quiet" this tick when BOTH its linear and angular speed are below
    // these. Set from direct measurement of this engine's own resting bodies, not pasted from a
    // reference - a sphere and a box settle to EXACTLY zero here, while a cylinder resting on its side
    // never fully settles: it oscillates forever in a band, angular speed bouncing between ~0.05 and
    // ~0.071 rad/s, linear between ~0.008 and ~0.013 m/s (traced 600 ticks, no decay). The angular
    // threshold MUST sit comfortably above that ~0.071 ceiling, or a settling cylinder would cross the
    // line back and forth and sleep/wake/sleep/wake right at the boundary - a body balanced on the
    // threshold itself, exactly the knife-edge this engine exists to avoid. 0.12 gives ~1.7x headroom
    // over the observed ceiling while staying far below any real motion (a rolling/tumbling body is
    // >1 rad/s, orders of magnitude clear). The linear threshold has enormous margin over the observed
    // ~0.013 and is the conventional value.
    static LINEAR_SLEEP_THRESHOLD = 0.05;        // m/s
    static ANGULAR_SLEEP_THRESHOLD = 0.12;       // rad/s

    // How long (seconds) an entire island must stay quiet before it is parked. Standard ~0.5s: long
    // enough that a body momentarily still for a real reason (the apex of a bounce, a brief pin
    // between two others mid-collapse) does not false-sleep, short enough that a settled pile parks
    // promptly. Tracked per body as sleepTimer (seconds), advanced only while the body is quiet AND
    // every other body in its island is quiet too - a single restless member holds the whole island
    // awake by resetting the island's readiness, see step().
    static TIME_TO_SLEEP = 0.5;                  // seconds

    constructor() {
        // Union-find parent map, rebuilt each tick: bodyId -> bodyId. Kept as a Map (not an array)
        // because body ids are monotonic and sparse over a session as bodies are added/removed.
        this._parent = new Map();
        // Scratch: island id (a representative body id) -> { members: [], quiet: bool }, rebuilt each
        // tick. Not retained across ticks - island membership is recomputed from live connectivity
        // every tick, since a contact appearing or breaking changes the graph.
        this._islands = new Map();
    }

    _find(id) {
        let root = id;
        while (this._parent.get(root) !== root) root = this._parent.get(root);
        // Path compression: point every node on the path straight at the root, so repeated finds
        // this tick are flat. Safe because the forest is rebuilt from scratch next tick.
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

    /**
     * Updates the sleep state of every dynamic body in `bodies`, using this tick's `manifolds`
     * (a ContactManifoldList) and `constraints` (the world's joint list) for connectivity, advancing
     * timers by `dt`. Parks islands that have been quiet long enough; wakes any body that a moving
     * static/kinematic neighbor or an already-awake dynamic neighbor keeps in motion.
     *
     * After this runs, a body with isAwake === false is one the solver may skip entirely this tick.
     */
    update(bodies, manifolds, constraints, dt) {
        this._parent.clear();
        this._islands.clear();

        // 1. Seed the forest with every DYNAMIC body as its own singleton. Static/kinematic bodies
        //    are intentionally never seeded - they cannot sleep and must not link their neighbors.
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType === RigidBody.DYNAMIC) this._ensure(b.id);
        }

        // 2. Union dynamic bodies coupled by a real contact. A manifold touching a static/kinematic
        //    body does NOT union (that body is not in the forest) - but it is remembered as an
        //    external contact, handled in step 4, because a dynamic body touching a MOVING
        //    kinematic/awake body must be forced awake regardless of its own quietness.
        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            const aDyn = a.bodyType === RigidBody.DYNAMIC, bDyn = b.bodyType === RigidBody.DYNAMIC;
            if (aDyn && bDyn) this._union(a.id, b.id);
        }

        // 3. Union dynamic bodies coupled by an enabled constraint. A constraint's bodyB may be null
        //    (anchored to the world) - that is a boundary like a static body, no union.
        if (constraints) {
            for (let i = 0; i < constraints.length; i++) {
                const c = constraints[i];
                if (!c.enabled || !c.bodyB) continue;
                const aDyn = c.bodyA.bodyType === RigidBody.DYNAMIC, bDyn = c.bodyB.bodyType === RigidBody.DYNAMIC;
                if (aDyn && bDyn) this._union(c.bodyA.id, c.bodyB.id);
            }
        }

        // 4. Determine which dynamic bodies are FORCED awake by an external influence this tick,
        //    independent of their own speed: a dynamic body in contact with a still-moving kinematic
        //    body (a rising platform, a swinging door) cannot be allowed to sleep even if it is
        //    momentarily quiet, because the thing it rests on is about to move it. A contact with a
        //    STATIC body is not a forcing influence - static ground is exactly what a body sleeps on.
        const forcedAwake = new Set();
        for (const manifold of manifolds.values()) {
            const a = manifold.bodyA, b = manifold.bodyB;
            IslandManager._maybeForceAwakeFromNeighbor(a, b, forcedAwake);
            IslandManager._maybeForceAwakeFromNeighbor(b, a, forcedAwake);
        }

        // 5. Group dynamic bodies by island root, and record each body's own quietness (below BOTH
        //    thresholds this tick). A forced-awake body counts as not quiet.
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

        // 6. Per island: if every member is quiet, advance the whole island's sleep timer by dt (each
        //    body carries its own sleepTimer, but they move in lockstep because the gate is the
        //    island's shared allQuiet). Once the island has been quiet past TIME_TO_SLEEP, park every
        //    member together. If ANY member is not quiet, wake the whole island - one restless body
        //    keeps its entire pile simulating, which is the correctness guarantee islands exist for.
        for (const island of this._islands.values()) {
            if (island.allQuiet) {
                let minTimer = Infinity;
                for (const body of island.members) {
                    body.sleepTimer += dt;
                    if (body.sleepTimer < minTimer) minTimer = body.sleepTimer;
                }
                // Park only when the SLOWEST-to-qualify member has also crossed the line, so a body
                // that only just went quiet does not drag the rest to sleep prematurely.
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

    // A dynamic `body` in contact with `other` must be forced awake if `other` is a KINEMATIC body
    // that is itself moving, or a DYNAMIC body that is currently awake - either way the neighbor is
    // about to impart motion this body cannot anticipate while asleep. A static neighbor, or a
    // sleeping dynamic one, is not a forcing influence (that is the whole point of a body being able
    // to rest on top of another sleeping body).
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
