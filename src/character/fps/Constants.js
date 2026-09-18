// Internal statics for FPSCharacterController: algorithm constants (FPSC) and the private raycast
// helper. LOAD ORDER: this file must load AFTER FPSCharacterController.js, since it attaches static
// properties onto that constructor function.

// Internal algorithm constants — thresholds/epsilons/factors baked into the collision, grounding,
// slope and ghost math. NOT caller-facing feel knobs (those live in FPS_CONTROLLER_DEFAULTS); kept
// named so nothing is a bare literal. Changing them changes solver behavior.
FPSCharacterController.FPSC = {
    // Contact tolerances (meters, multiplied by the character scale where used).
    SKIN: 0.01,               // sweep/contact skin width
    GROUND_TOL: 0.1,          // how close the feet must be to a surface to count as grounded
    GHOST_GROUND_INSET: 0.25, // fraction of height the ghost's bottom is lifted above the feet

    // "Effectively zero" epsilons — vector/speed magnitudes below which we treat a quantity as null.
    EPS_LEN: 1e-4,            // general length/normal guard
    EPS_DIR: 1e-5,            // direction-normalization guard
    EPS_SPD: 1e-6,            // speed-normalization guard
    EPS_INPUT2: 1e-10,        // squared move-input threshold (has-input test)
    EPS_SPEED_MARGIN: 1e-3,   // speed must exceed a target by this to count as "above" it

    // Ground-normal.y classifiers (a surface normal's up-component; 1 = flat floor, 0 = vertical wall).
    NY_CEILING: -0.4,         // normal.y ABOVE this is not a ceiling (must face downward to be one)
    NY_STEEP_MIN: 0.3,        // steep-but-not-wall floor-like face lower bound
    NY_GROUNDISH: 0.5,        // normal.y at/above this is walkable-ish ground (skip as a wall/step)
    NY_FLOORLIKE: 0.1,        // normal.y above this tilts up (floor-like), below is a vertical wall
    N_DEGENERATE: 0.5,        // reject a contact normal whose length is below this (bad EPA result)
    TOE_BAND_FRAC: 0.6,       // a too-steep floor-like contact only blocks as a slope-toe within this
                              // fraction of body height above the feet; higher is an overhang

    // Below this dot between wish and current slide direction, wish is a deliberate reversal (brake)
    // rather than a carve.
    SLIDE_REVERSAL_DOT: -0.5,

    // MOVEMENT STATE — one flat, mutually exclusive enum, decided ONCE per tick by endStep and read
    // everywhere else (beginStep dispatches on it verbatim; nothing re-derives it).
    //   LADDER / AIRBORNE / WALK / SLIP / SLIDE (see Movement/Step.js).
    MOVE_LADDER: 'ladder',
    MOVE_AIRBORNE: 'airborne',
    MOVE_WALK: 'walk',
    MOVE_SLIP: 'slip',
    MOVE_SLIDE: 'slide',
    MOVE_MANTLE: 'mantle',

    // A grounded mantle tap is only allowed up to this fraction of standHeight (~chest height).
    MANTLE_CHEST_HEIGHT_FRAC: 0.77,

    // Knockback gating (see _readGhostKnockback).
    KB_CLOSING_MIN: 0.5,      // object must close on the character faster than this (units/s) to knock back
    KB_MIN: 0.05,             // ignore a computed knockback smaller than this

    // Ground-suppress frame counts — ticks the ground clamp is held off after an event.
    GROUND_SUPPRESS_KB: 5,    // after taking knockback
    GROUND_SUPPRESS_JUMP: 8,  // after jumping

    // Sweep sub-stepping + wall interaction.
    SUBSTEP_FRAC: 0.5,        // sub-step length as a fraction of the smallest half-extent
    NEAR_CENTER_FRAC: 0.4,    // "hit near my center" band as a fraction of width
    PUSH_INTO_MIN: 0.5,       // dot(vel, toHit) above this = actively moving into a contact

    // Wall clip / step-up / depenetration.
    KEEP_BLOCKED: 0.01,       // keep-fraction below this = a non-yielding wall (fully blocks / triggers step-up)
    NY_NEAR_VERTICAL: 0.2,    // |normal.y| below this = a near-vertical face (steppable candidate)
    // Depenetration back-probe step, as a fraction of the character's own half-width (independent of skin).
    BACKPROBE_WIDTH_FRAC: 0.1,
    // Climbable-slope look-ahead sample points, as multiples of DEPTH past the footprint edge
    // (scale-invariant: a fixed-meter reach would mis-reach at other scales).
    CLIMB_PROBE_DEPTH_MULTS: [0, 0.5, 1.0, 1.67],

    // Render.
    VIEW_DISP_SNAP: 0.01,     // pending view-displacement above which eye interpolation snaps (crouch/step)
};

/**
 * Cast a ray from start to end in the physics world, returning the nearest hit not among
 * `ignoreObjects`, or null. Adapts World.rayIntersect's result shape to the {object, point, normal, t}
 * shape callers expect. `ignoreObjects` (body `.name` values) is resolved to body references and
 * passed to the query's own `ignore` param, so those bodies are excluded BEFORE the nearest-hit
 * search runs.
 *
 * @method _raycast
 * @private
 * @static
 * @param {World} world
 * @param {Vector3} start
 * @param {Vector3} end
 * @param {String[]} [ignoreObjects] - body `.name` values to skip
 * @return {Object|null} { object, point:Vector3, normal:Vector3, t:Number } or null
 */
FPSCharacterController._raycast = function(world, start, end, ignoreObjects) {
    var ignore = null;
    if (ignoreObjects && ignoreObjects.length) {
        ignore = [];
        var bodies = world.bodies;
        for (var i = 0; i < bodies.length; i++) {
            if (bodies[i].name && ignoreObjects.indexOf(bodies[i].name) !== -1) { ignore.push(bodies[i]); }
        }
    }
    var hit = world.rayIntersect(start, end, ignore);
    if (!hit) { return null; }
    return { object: hit.body, point: hit.point, normal: hit.normal, t: hit.distance };
};
