// Kinematic wall/step collision: the character is excluded from the solver, so this file stops it at
// walls, lets it climb steps, and depenetrates it. Shared by the body and its ghost.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Kinematic collide-and-slide for the character body. Floors and ramps are ignored here (the ground
 * clamp handles those); only vertical walls clip velocity.
 *
 * @method _collideAndSlide
 * @private
 * @param {Number} vx - incoming horizontal velocity, x.
 * @param {Number} vz - incoming horizontal velocity, z.
 * @param {Number} dt
 * @return {Object} result - { x, z } clipped horizontal velocity.
 */
proto._collideAndSlide = function(vx, vz, dt) {
    var res = this._sweptCollideAndSlide({
        position: this.body.position,
        width: this.width, depth: this.depth, height: this.height,
        skin: this._skin, mass: this.mass, stepHeight: this.stepHeight,
        selfBody: this.body, otherSelfBody: this._ghost || null,
        // A slide is exempt from the too-steep-can't-move-up block (momentum, not input, carries it
        // up), including while airborne-sliding.
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
 * Sweeps an inset box along a horizontal velocity and clips it against blocking contacts, sub-stepped
 * so long sweeps can't return a wrong-axis normal. Shared by the character body and its ghost.
 *
 * @method _sweptCollideAndSlide
 * @private
 * @param {Object} opts
 * @param {Vector3} opts.position - Sweep origin (box center).
 * @param {Number} opts.width, opts.depth, opts.height - Box extents, pre-inset.
 * @param {Number} opts.skin - Contact/sweep tolerance subtracted from each half-extent.
 * @param {Number} opts.mass - Sweeping body's mass, for the push mass-yield ratio.
 * @param {Number} [opts.stepHeight=0] - Step-up height; 0 disables step-up.
 * @param {RigidBody} opts.selfBody - Body to exclude from its own sweep hits.
 * @param {RigidBody} [opts.otherSelfBody] - A second body to exclude (e.g. body excludes its ghost).
 * @param {Boolean} [opts.climbSteepSlopes=false] - Exempt climbable too-steep faces from the wall rule.
 * @param {Number} opts.vx, opts.vz - Incoming horizontal velocity.
 * @param {Number} opts.dt - Tick duration in seconds.
 * @return {Object} result - { x, z, depenX, depenZ }: clipped velocity + penetration correction.
 */
proto._sweptCollideAndSlide = function(opts) {
    var vx = opts.vx, vz = opts.vz;
    var position = opts.position, width = opts.width, depth = opts.depth, height = opts.height,
        skin = opts.skin, mass = opts.mass, dt = opts.dt, selfBody = opts.selfBody, otherSelfBody = opts.otherSelfBody;
    var stepHeight = opts.stepHeight || 0;
    var climbSteepSlopes = !!opts.climbSteepSlopes;
    var world = this.world;
    if (!world || typeof world.shapeIntersect !== "function") { return { x: vx, z: vz, depenX: 0, depenZ: 0 }; }

    // Original move heading, before any clipping this tick — used by the climb-slope-ahead probe.
    var moveLen0 = Math.sqrt(vx * vx + vz * vz);
    var mdx0 = moveLen0 > FPSC.EPS_DIR ? vx / moveLen0 : 0;
    var mdz0 = moveLen0 > FPSC.EPS_DIR ? vz / moveLen0 : 0;

    // Swept-box collide-and-slide: sweep an inset box along the move each tick and clip velocity
    // against the real contact plane.
    var p = position;
    var halfW = width / 2 - skin;
    var halfD = depth / 2 - skin;
    // Lift the swept box off the feet so it doesn't graze the floor slab's top edge (which returns a
    // degenerate near-vertical normal that fakes a wall), while still catching a steep ramp's toe.
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

    // Sub-step so each swept chunk stays well under the smallest half-extent (a long sweep can return
    // a wrong-axis normal from EPA).
    var chunkLen = Math.min(halfW, halfD) * FPSC.SUBSTEP_FRAC;
    var full = Math.sqrt(vx * vx + vz * vz) * dt;
    var nSub = Math.max(1, Math.ceil(full / Math.max(chunkLen, FPSC.EPS_LEN)));
    var sdt = dt / nSub;

    // shapeIntersect's contact normal points FROM the surface TOWARD the sweeping mover (the reversed
    // travel direction). Everything below assumes the opposite convention ("points into the wall"), so
    // the raw result is negated once, here, where it enters.
    //
    // This query reports only the SINGLE nearest body, so raw kinematic character bodies (never walls —
    // a ghost is the real stand-in) must be excluded at the QUERY level via `ignore`, not filtered
    // after the fact: another controller's raw body sits nearly on top of its ghost and would shadow it.
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
            // Negate to the "points into the wall" convention (see above).
            var n = { x: -hn.x, y: -hn.y, z: -hn.z };
            if (Math.abs(n.y) >= minStandableNy) { localIgnore.push(h.body); continue; }
            // Vertical wall: normal horizontal, points character->object; heading in is v.n > 0.
            // Too-steep floor-like face (0.1 < n.y < cutoff): heading in is v.(n.x,n.z) < 0.
            var floorLike = n.y > FPSC.NY_FLOORLIKE;
            // A floor-like too-steep face only blocks as a "slope ahead" near the feet; the same face up
            // near head height is an overhang (headroom gate's job), so skip it to avoid trapping.
            if (floorLike && h.point && (h.point.y - (p.y - height / 2)) > height * FPSC.TOE_BAND_FRAC) { localIgnore.push(h.body); continue; }
            if (climbSteepSlopes && self_._climbableSlopeAhead(start, mdx0, mdz0)) { localIgnore.push(h.body); continue; }
            var overlapped = h.fraction === 0 && h.distance === 0;
            // vyDet is for detection only: falling/rising past a near-vertical face counts as heading
            // into it. Floor-like faces keep the horizontal-only test.
            var into = floorLike ? -(vx * n.x + vz * n.z) : (vx * n.x + vz * n.z + vyDet * n.y);
            if (into <= 0 && !overlapped) { return null; }
            var keep = 0;
            var b = h.body;
            // Platforms never yield (scripted geometry); another player's ghost is a full body-block too.
            if (b && !b.isPlatform && !b.isCharacterGhost && b.bodyType === RigidBody.DYNAMIC && b._mass > 0 &&
                b._mass <= self_._pushMassLimit) {
                keep = mass / (mass + b._mass);
            }
            return { n: n, keep: keep, overlapped: overlapped };
        }
        return null;
    }

    // Contact test with no directional gate (unlike findBlock), used by the recovery back-probe.
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

    // Clip velocity against walls the move would hit, and depenetrate out of any wall sunk into (push
    // along -n so it rests just clear). Sub-stepped for reliable normals.
    var cx = p.x, cz = p.z;
    var depenX = 0, depenZ = 0;
    // vyDet is DETECTION ONLY — it lets the sweep see a wall the body is falling/rising past, but never
    // clips velocity or writes position (the ground clamp owns y).
    var vyDet = selfBody ? selfBody.linear_velocity.y : 0;
    for (var s = 0; s < nSub; s++) {
        for (var iter = 0; iter < 4; iter++) {
            var speed = Math.sqrt(vx * vx + vz * vz + vyDet * vyDet);
            if (speed < FPSC.EPS_DIR) { break; }
            var start = new Vector3(cx, sy, cz);
            var end = new Vector3(cx + vx * sdt, sy + vyDet * sdt, cz + vz * sdt);
            var blk = findBlock(start, end);
            if (!blk) { break; }
            // Step-up: before walling a near-vertical, non-yielding face, test if it's clear when swept
            // raised by stepHeight — if so it's steppable, let the move through.
            if (blk.keep < FPSC.KEEP_BLOCKED && Math.abs(blk.n.y) < FPSC.NY_NEAR_VERTICAL && stepHeight > 0) {
                var upStart = new Vector3(cx, sy + stepHeight, cz);
                var upEnd = new Vector3(cx + vx * sdt, sy + stepHeight, cz + vz * sdt);
                if (!findBlock(upStart, upEnd)) { break; }
            }
            var n = blk.n, keep = blk.keep;
            // Clip the into-face velocity using the horizontal blocking direction (never inject vertical).
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
            // Depenetration is recovery-only: back-probe one small step along -n to tell BURIED from
            // GRAZING, then nudge out. Vertical walls only.
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
