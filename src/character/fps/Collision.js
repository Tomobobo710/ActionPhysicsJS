// Kinematic wall/step collision: the character is excluded from the solver's own contact resolution
// (collision_mask 1), so this file is what actually stops the character at walls, lets it climb steps,
// and depenetrates it out of geometry it sank into. Shared by both the character body and its ghost
// via _sweptCollideAndSlide.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Kinematic collide-and-slide. The character is excluded from the solver, so we stop
 * ourselves at walls and slide along them here. For the current horizontal velocity
 * we cast a fan of rays (across the footprint width, at a few heights) in the move
 * direction; if a vertical wall is within the box's reach this step we remove the
 * into-wall velocity component and re-test, so corners stop on both walls. Floors and
 * ramps (normal.y >= 0.5) are ignored — those are handled by the ground clamp.
 *
 * @method _collideAndSlide
 * @private
 * @param {Number} vx - incoming horizontal velocity, x.
 * @param {Number} vz - incoming horizontal velocity, z.
 * @param {Number} dt
 * @return {Object} result
 * @return {Number} result.x - clipped horizontal velocity, x.
 * @return {Number} result.z - clipped horizontal velocity, z.
 */
proto._collideAndSlide = function(vx, vz, dt) {
    var res = this._sweptCollideAndSlide({
        position: this.body.position,
        width: this.width, depth: this.depth, height: this.height,
        skin: this._skin, mass: this.mass, stepHeight: this.stepHeight,
        selfBody: this.body, otherSelfBody: this._ghost || null,
        // A SLIDE is exempt from the too-steep-can't-move-up block (the slide IS the climb — momentum,
        // not input, is what carries it up). This must hold while AIRBORNE-sliding too: an airborne
        // slide sweeping into a steep face otherwise wall-clips to zero speed mid-air, which kills the
        // slide before it ever lands on the surface. _climbableSlopeAhead inside still tells real
        // slopes from vertical walls, so walls keep stopping a slide. this._moveState is already
        // MOVE_SLIDE on the true first-contact tick too — endStep decides movement state (including
        // slide entry) BEFORE this function runs later in the same beginStep, so there's no
        // "one tick behind" gap here to patch around.
        climbSteepSlopes: this.climbSteepSlopes || this._moveState === FPSC.MOVE_SLIDE,
        vx: vx, vz: vz, dt: dt,
    });
    // Depenetration is a horizontal position correction out of a wall, separate from the velocity move.
    if (res.depenX !== 0 || res.depenZ !== 0) {
        var bp = this.body.position;
        this.body.position.set(bp.x + res.depenX, bp.y, bp.z + res.depenZ);
        this.body.updateDerived();
    }
    return { x: res.x, z: res.z };
};

/**
 * Sweeps an inset box along a horizontal velocity and clips it against blocking contacts,
 * sub-stepped so long sweeps can't return a wrong-axis normal. Shared by the character body
 * and its ghost so both get identical wall/mass-yield behavior from one implementation.
 *
 * @method _sweptCollideAndSlide
 * @private
 * @param {Object} opts
 * @param {Vector3} opts.position - Sweep origin (box center).
 * @param {Number} opts.width - Box width (x), pre-inset.
 * @param {Number} opts.depth - Box depth (z), pre-inset.
 * @param {Number} opts.height - Box height (y), pre-inset.
 * @param {Number} opts.skin - Contact/sweep tolerance subtracted from each half-extent.
 * @param {Number} opts.mass - Sweeping body's mass, used for the push mass-yield ratio.
 * @param {Number} [opts.stepHeight=0] - Step-up height; 0 disables step-up entirely.
 * @param {RigidBody} opts.selfBody - Body to exclude from its own sweep hits.
 * @param {RigidBody} [opts.otherSelfBody] - A second body to exclude (e.g. the character
 *   excludes its ghost, and vice versa).
 * @param {Boolean} [opts.climbSteepSlopes=false] - Exempt too-steep floor-like faces that have a
 *   climbable slope ahead from the wall-block rule.
 * @param {Number} opts.vx - Incoming horizontal velocity, x.
 * @param {Number} opts.vz - Incoming horizontal velocity, z.
 * @param {Number} opts.dt - Tick duration in seconds.
 * @return {Object} result
 * @return {Number} result.x - Clipped horizontal velocity, x.
 * @return {Number} result.z - Clipped horizontal velocity, z.
 * @return {Number} result.depenX - Position correction out of a penetrated wall, x (0 if none).
 * @return {Number} result.depenZ - Position correction out of a penetrated wall, z (0 if none).
 */
proto._sweptCollideAndSlide = function(opts) {
    var vx = opts.vx, vz = opts.vz;
    var position = opts.position, width = opts.width, depth = opts.depth, height = opts.height,
        skin = opts.skin, mass = opts.mass, dt = opts.dt, selfBody = opts.selfBody, otherSelfBody = opts.otherSelfBody;
    var stepHeight = opts.stepHeight || 0;
    var climbSteepSlopes = !!opts.climbSteepSlopes;
    var world = this.world;
    if (!world || typeof world.shapeIntersect !== "function") { return { x: vx, z: vz, depenX: 0, depenZ: 0 }; }

    // Original move heading, before any clipping this tick — used by the climb-slope-ahead probe
    // so a mid-loop velocity clip doesn't collapse the probe direction.
    var moveLen0 = Math.sqrt(vx * vx + vz * vz);
    var mdx0 = moveLen0 > FPSC.EPS_DIR ? vx / moveLen0 : 0;
    var mdz0 = moveLen0 > FPSC.EPS_DIR ? vz / moveLen0 : 0;

    // Swept-box collide-and-slide: sweep an inset box along the move each tick and clip velocity
    // against the real contact plane.
    var p = position;
    var halfW = width / 2 - skin;
    var halfD = depth / 2 - skin;
    // Lift the swept box a small amount off the feet so it doesn't graze the floor slab's top
    // edge (which returns a degenerate near-vertical normal and fakes a wall), while staying low
    // enough to still catch a steep ramp's toe.
    var lift = skin * 2;
    var halfH = Math.max(0.05, height / 2 - lift / 2);
    var yOffset = lift / 2;
    // Cache the swept probe box per caller (different callers may have different dimensions).
    var cacheKey = selfBody === this.body ? "_sweepBox" : "_altSweepBox";
    if (!this[cacheKey] || this[cacheKey + "W"] !== halfW || this[cacheKey + "H"] !== halfH || this[cacheKey + "D"] !== halfD) {
        this[cacheKey] = new BoxShape(halfW, halfH, halfD);
        this[cacheKey + "W"] = halfW; this[cacheKey + "H"] = halfH; this[cacheKey + "D"] = halfD;
    }
    var boxShape = this[cacheKey];
    var minStandableNy = this._minStandableNormalY;

    // Sub-step so each swept chunk stays well under the smallest half-extent (a long sweep can
    // return a wrong-axis normal from EPA).
    var chunkLen = Math.min(halfW, halfD) * FPSC.SUBSTEP_FRAC;
    var full = Math.sqrt(vx * vx + vz * vz) * dt;
    var nSub = Math.max(1, Math.ceil(full / Math.max(chunkLen, FPSC.EPS_LEN)));
    var sdt = dt / nSub;

    // shapeIntersect's contact normal points FROM the HIT SURFACE TOWARD THE SWEEPING MOVER (the
    // reversed travel direction on a miss-then-touch sweep - see Queries._advance's own
    // lastGoodNx/_finishHit(-ux,-uy,-uz)), not from the mover toward the object. Everything below
    // (findBlock's `into` test, the vertical-wall clip direction, the depenetration back-probe) is
    // written assuming the OPPOSITE convention ("points into the wall") — negating here, once, at
    // the single place the raw query result enters this file, keeps that downstream math correct
    // without hunting down every sign use individually. This was a REAL, confirmed bug: with the
    // un-negated normal, `into = vx*n.x + vz*n.z` computed NEGATIVE while genuinely moving into a
    // wall (an approaching mover's velocity and the surface-outward normal point opposite ways by
    // definition), so `into <= 0` rejected every real block outright — two characters walked
    // straight through each other for 100+ units with the block silently never firing, at ANY
    // gap, not just the ghost-shadowing case fixed above. "Heading into this face" is v.n > 0
    // (post-negation); pushing out of penetration moves along -n (also post-negation).
    //
    // World.shapeIntersect reports only the SINGLE nearest body, unlike the source's multi-hit
    // query. A raw kinematic character body is never itself a wall (no mass to yield against — its
    // ghost is the real solver stand-in for it, see Body.js), so it must be excluded from candidates
    // at the QUERY level via `ignore`, not filtered after the fact: another controller's raw body
    // sits at nearly the same place as its own ghost, so it is almost always the geometrically
    // NEAREST hit and would permanently shadow the ghost behind it if only checked post-hoc - this
    // was a real, confirmed bug (two characters walked straight through each other for 100+ units;
    // the sweep found the other's raw kinematic body every time, discarded it as "not a wall," and
    // never found the ghost sitting right behind it, since World.shapeIntersect only ever reports
    // the SINGLE nearest hit). Every other kinematic-character body in the world (this body/ghost
    // are already excluded via selfBody/otherSelfBody) is ignored at the query itself, so the
    // nearest REAL hit - a wall, a pushable object, or another character's GHOST - is found
    // directly.
    //
    // Nearest valid blocking contact for a sweep, or null. { n, pen, keep }.
    var self_ = this;
    var worldBodies = world.bodies;
    var queryIgnore = otherSelfBody ? [selfBody, otherSelfBody] : [selfBody];
    for (var ki = 0; ki < worldBodies.length; ki++) {
        var kb = worldBodies[ki];
        if (kb.isKinematicCharacter && !kb.isCharacterGhost && kb !== selfBody && kb !== otherSelfBody) {
            queryIgnore.push(kb);
        }
    }
    function findBlock(start, end) {
        var localIgnore = queryIgnore.slice();
        for (var tries = 0; tries < 8; tries++) {
            var h = world.shapeIntersect(boxShape, start, end, null, localIgnore);
            if (!h) { return null; }
            var hn = h.normal;
            if (!hn || !isFinite(hn.x) || !isFinite(hn.y) || !isFinite(hn.z)) { return null; }
            var nlen = Math.sqrt(hn.x * hn.x + hn.y * hn.y + hn.z * hn.z);
            if (nlen < FPSC.N_DEGENERATE) { return null; }
            // Negate: see this function's own header comment for why the raw query result is flipped
            // here, once, before any of the "points into the wall" math below reads it.
            var n = { x: -hn.x, y: -hn.y, z: -hn.z };
            if (Math.abs(n.y) >= minStandableNy) { localIgnore.push(h.body); continue; }
            // Vertical wall: normal horizontal, points character->object; heading in is v.n > 0.
            // Too-steep floor-like face (0.1 < n.y < cutoff): normal tilts up-and-back, so heading
            // in is v.(n.x,n.z) < 0 — sign flipped below.
            var floorLike = n.y > FPSC.NY_FLOORLIKE;
            // A floor-like too-steep face is only a legitimate "slope ahead" block near the feet
            // (walking into a ramp's toe). The same face type contacted up near head height is an
            // OVERHANG (ramp underside above a wedged character), not a slope to stop forward
            // progress on — treating it as a wall-slide clip can zero velocity in every direction,
            // including retreat, trapping the character. Overhead clearance is the headroom gate's
            // job; skip it here so a sideways/backward escape isn't blocked by the same contact.
            if (floorLike && h.point && (h.point.y - (p.y - height / 2)) > height * FPSC.TOE_BAND_FRAC) { localIgnore.push(h.body); continue; }
            if (climbSteepSlopes && self_._climbableSlopeAhead(start, mdx0, mdz0)) { localIgnore.push(h.body); continue; }
            var overlapped = h.fraction === 0 && h.distance === 0;
            // vyDet included for the vertical-wall case only (detection): falling/rising past a
            // near-vertical face counts as heading into it. Floor-like faces keep the horizontal-only
            // test — their block is "slope ahead", owned by the clamp/steep path, not this.
            var into = floorLike ? -(vx * n.x + vz * n.z) : (vx * n.x + vz * n.z + vyDet * n.y);
            if (into <= 0 && !overlapped) { return null; }
            var keep = 0;
            var b = h.body;
            // Platforms never yield like a pushable object — they're scripted geometry. Another
            // player's ghost is a full body-block too — a player is a wall, not a pushable box.
            if (b && !b.isPlatform && !b.isCharacterGhost && b.bodyType === RigidBody.DYNAMIC && b._mass > 0 &&
                b._mass <= self_._pushMassLimit) {
                keep = mass / (mass + b._mass);
            }
            return { n: n, keep: keep, overlapped: overlapped };
        }
        return null;
    }

    // Contact test with no directional gate (unlike findBlock). Used by the recovery back-probe,
    // since after velocity is clipped the body is no longer "moving into" the wall.
    function contactAt(x, y, z) {
        var pt = new Vector3(x, y, z);
        var localIgnore = queryIgnore.slice();
        for (var tries = 0; tries < 8; tries++) {
            var h = world.shapeIntersect(boxShape, pt, pt, null, localIgnore);
            if (!h) { return false; }
            var n = h.normal;
            if (!n || !isFinite(n.x) || !isFinite(n.y) || !isFinite(n.z)) { return false; }
            if (Math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z) < FPSC.N_DEGENERATE) { return false; }
            if (Math.abs(n.y) >= minStandableNy) { localIgnore.push(h.body); continue; } // walkable ground/ramp — not a wall
            return true;
        }
        return false;
    }

    var sy = p.y + yOffset;

    // CLIP velocity against walls the move would hit this tick, and DEPENETRATE out of any wall
    // sunk into (push along -n by overlap+skin so it rests just clear — the swept cast is
    // penetration-based, so without this a fast move ends up inside and sticks). Velocity-only for
    // the move; position correction only for depenetration. Sub-stepped for reliable normals.
    var cx = p.x, cz = p.z;
    var depenX = 0, depenZ = 0;
    // Vertical component of the move, for DETECTION ONLY — so the swept box sees a wall it's about to
    // bury into by falling/rising past it (e.g. dropping straight down a near-vertical ramp face).
    // vy never clips velocity or writes position here; the ground clamp still owns y. It only lets
    // findBlock's `into` test fire so the existing horizontal back-probe can push the box out.
    var vyDet = selfBody ? selfBody.linear_velocity.y : 0;
    for (var s = 0; s < nSub; s++) {
        for (var iter = 0; iter < 4; iter++) {
            var speed = Math.sqrt(vx * vx + vz * vz + vyDet * vyDet);
            if (speed < FPSC.EPS_DIR) { break; }
            var start = new Vector3(cx, sy, cz);
            var end = new Vector3(cx + vx * sdt, sy + vyDet * sdt, cz + vz * sdt);
            var blk = findBlock(start, end);
            if (!blk) { break; }
            // Step-up: before walling a near-vertical, non-yielding face, test if it's clear when
            // swept raised by stepHeight — if so it's steppable, let the move through.
            if (blk.keep < FPSC.KEEP_BLOCKED && Math.abs(blk.n.y) < FPSC.NY_NEAR_VERTICAL && stepHeight > 0) {
                var upStart = new Vector3(cx, sy + stepHeight, cz);
                var upEnd = new Vector3(cx + vx * sdt, sy + stepHeight, cz + vz * sdt);
                if (!findBlock(upStart, upEnd)) { break; }
            }
            var n = blk.n, keep = blk.keep;
            // Clip the into-face velocity using the horizontal blocking direction (never inject
            // vertical; the ground clamp owns y).
            var floorLike = n.y > FPSC.NY_FLOORLIKE;
            var bx = floorLike ? -n.x : n.x, bz = floorLike ? -n.z : n.z;
            var blen = Math.sqrt(bx * bx + bz * bz);
            if (blen < FPSC.EPS_SPD) { break; }
            bx /= blen; bz /= blen;
            var dot = vx * bx + vz * bz;
            if (dot > 0) {
                vx -= dot * bx * (1 - keep);
                vz -= dot * bz * (1 - keep);
            }
            // Depenetration is recovery-only: detect BURIED vs GRAZING with a back-probe (sweep one
            // fixed small step along -n; if still in contact there, nudge out by that step), rather
            // than trusting a reported penetration depth directly (this query never reports one -
            // see findBlock's own comment; contactAt below is the real, load-bearing check, not an
            // early-out guard). Vertical walls only; a floor-like too-steep toe is owned by the
            // clamp / steep-slope path.
            if (!floorLike && keep < FPSC.KEEP_BLOCKED && (blk.overlapped || dot > 0)) {
                var step = Math.min(width, depth) * FPSC.BACKPROBE_WIDTH_FRAC;
                if (contactAt(cx - n.x * step, sy, cz - n.z * step)) {
                    depenX -= n.x * step; depenZ -= n.z * step;
                    cx -= n.x * step; cz -= n.z * step;
                }
            }
            if (keep > FPSC.KEEP_BLOCKED) { break; }
        }
        cx += vx * sdt;
        cz += vz * sdt;
    }
    return { x: vx, z: vz, depenX: depenX, depenZ: depenZ };
};
