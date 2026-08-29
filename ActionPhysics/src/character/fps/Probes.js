// Ground/ceiling/ladder raycast probes: the hand-written spatial queries beginStep/endStep read each
// tick to decide grounding, standable-slope classification, headroom, and ladder mounting. No probe
// here writes any sim state itself — each one just reports what's in the world.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;
var raycast = FPSCharacterController._raycast;

/**
 * Multi-point ground probe (center + four edge midpoints). Returns ALL floor-like hits, highest
 * first — NOT collapsed to a single "best" here, because the caller needs to fall back to a
 * lower (but valid) hit when the highest one is rejected as too tall to step onto (e.g. one edge
 * ray grazing a box pushed against the footprint, while the other four rays are still squarely
 * over real floor). Collapsing to one hit here would throw the floor away before the caller ever
 * gets a chance to prefer it.
 * @method _probeGroundCandidates
 * @private
 * @param {Number} maxSnap - max downward reach (below the feet) to probe, before scale/skin margins.
 * @return {Object[]} floor-like raycast hits, sorted highest point.y first.
 */
proto._probeGroundCandidates = function(maxSnap) {
    var half = this.height / 2;
    var p = this.body.position;
    // Cast from the higher of this tick's start position and the current position, so a fast
    // descent that penetrated the floor this tick doesn't miss it.
    var topY = Math.max(this._prevY !== undefined ? this._prevY : p.y, p.y) + this._skin;
    var bottomY = p.y - (half + maxSnap + this._skin);
    var ix = this.width / 2 - this._skin;
    var iz = this.depth / 2 - this._skin;
    var offsets = [[0, 0], [ix, 0], [-ix, 0], [0, iz], [0, -iz]];

    var candidates = [];
    for (var i = 0; i < offsets.length; i++) {
        var ox = offsets[i][0];
        var oz = offsets[i][1];
        var start = new Vector3(p.x + ox, topY, p.z + oz);
        var end = new Vector3(p.x + ox, bottomY, p.z + oz);
        var hit = raycast(this.world, start, end, this._ignoreSelf);
        if (!hit || hit.normal.y < FPSC.NY_FLOORLIKE) { continue; }
        // Exclude a pushable object as ground only when walking INTO its side (pushing it), not
        // when it's roughly under our own center (standing on it).
        var gBody = hit.object;
        var gm = gBody && gBody._mass;
        var isPushable = gBody && gBody.bodyType === RigidBody.DYNAMIC && gm > 0 && gm <= this._pushMassLimit;
        if (isPushable) {
            var gv = this.body.linear_velocity;
            var toHitX = hit.point.x - p.x, toHitZ = hit.point.z - p.z;
            var towardLen = Math.sqrt(toHitX * toHitX + toHitZ * toHitZ);
            var movingIntoIt = towardLen > FPSC.EPS_LEN &&
                (gv.x * toHitX + gv.z * toHitZ) / towardLen > FPSC.PUSH_INTO_MIN;
            var nearCenter = towardLen < this.width * FPSC.NEAR_CENTER_FRAC;
            if (movingIntoIt && !nearCenter) { continue; }
        }
        candidates.push(hit);
    }
    candidates.sort(function(a, b) { return b.point.y - a.point.y; });
    return candidates;
};

/**
 * Multi-ray UP probe across the footprint. Returns the LOWEST ceiling (down-facing
 * surface) within `reachAboveFeet` of the feet, or null. Mirror of _probeGround; covers
 * sloped overhead geometry (e.g. a ramp underside) that forward rays can't see.
 *
 * @method _probeCeiling
 * @private
 * @param {Number} reachAboveFeet - how far above the feet to scan.
 * @return {Object|null} the lowest down-facing hit within reach, or null.
 */
proto._probeCeiling = function(reachAboveFeet) {
    var p = this.body.position;
    var feetY = p.y - this.height / 2;
    var startY = feetY + this._skin;
    var endY = feetY + reachAboveFeet + this._skin;
    var ix = this.width / 2 - this._skin;
    var iz = this.depth / 2 - this._skin;
    var offsets = [[0, 0], [ix, 0], [-ix, 0], [0, iz], [0, -iz]];
    var best = null;
    for (var i = 0; i < offsets.length; i++) {
        var ox = offsets[i][0];
        var oz = offsets[i][1];
        var hit = this._raycastSkipPlatforms(
            new Vector3(p.x + ox, startY, p.z + oz),
            new Vector3(p.x + ox, endY, p.z + oz)
        );
        if (!hit || hit.normal.y > FPSC.NY_CEILING) { continue; } // not a ceiling (must face downward)
        if (!best || hit.point.y < best.point.y) { best = hit; }
    }
    return best;
};

/**
 * Same contract as the private _raycast helper (excludes this body + its ghost, returns the
 * nearest remaining hit), but ALSO skips a hit body tagged isPlatform. A scripted moving platform is
 * deliberately excluded from the solver's own contact resolution (see _baseVelocity's constructor
 * comment / platform()'s collision_mask) so a rider is carried via scripted base-velocity, never a
 * real physical shove — but a raw raycast doesn't consult collision_mask at all, so without this a
 * fast-rising platform that catches back up to a character mid-jump gets misread as a solid ceiling
 * overhead by _ceilingSlide, capping the jump's vertical velocity and killing it a few ticks after
 * liftoff, even though no real contact manifold ever forms between the two bodies (the phantom
 * ceiling is a pure raycast/collision_mask mismatch, not a real collision). Used ONLY by
 * _probeCeiling — _probeGround intentionally still sees platforms (that's how riding one works at
 * all), and ordinary walls/ramps/props aren't tagged isPlatform so they're unaffected.
 *
 * Self+ghost exclusion goes through World.rayIntersect's own `ignore` parameter (bodies excluded
 * from candidates BEFORE the nearest-hit search runs), not post-hoc name filtering on the single
 * reported hit — a probe cast from the character's own body would otherwise almost always find
 * itself as the "nearest hit" and get discarded, reporting no ceiling even when a real one is
 * there. Platform exclusion still can't do the same (World.rayIntersect reports only the single
 * nearest body, so a platform hit is reported as "no ceiling" rather than passed over to a real
 * ceiling behind it) — narrower, left as a known gap.
 *
 * @method _raycastSkipPlatforms
 * @private
 * @param {Vector3} start
 * @param {Vector3} end
 * @return {Object|null} the nearest non-self, non-platform hit, or null.
 */
proto._raycastSkipPlatforms = function(start, end) {
    var ignore = this._ghost ? [this.body, this._ghost] : this.body;
    var hit = this.world.rayIntersect(start, end, ignore);
    if (!hit) { return null; }
    var b = hit.body;
    if (b && b.isPlatform) { return null; }
    return { object: b, point: hit.point, normal: hit.normal, t: hit.distance };
};

/**
 * Is there room to stand up? (No ceiling within standHeight of the feet.)
 * @method _canStand
 * @private
 * @return {Boolean}
 */
proto._canStand = function() {
    var feetY = this.body.position.y - this.height / 2;
    var ceil = this._probeCeiling(this.standHeight);
    return !ceil || ceil.point.y - feetY >= this.standHeight - this._skin;
};

/**
 * Single ray probe for a ladder ahead, along `dir` (horizontal, need not be unit length). Placed
 * halfway between the feet and stepHeight above them rather than the body center. Returns the raw
 * hit `{object, point, normal, t}` with the normal flipped to point OUT of the face (toward the
 * caller), or null.
 *
 * @method _findLadderAhead
 * @private
 * @param {Vector3} dir
 * @return {Object|null}
 */
proto._findLadderAhead = function(dir) {
    var p = this.body.position;
    var dlen = Math.sqrt(dir.x * dir.x + dir.z * dir.z);
    if (dlen < FPSC.EPS_LEN) { return null; }
    var dx = dir.x / dlen, dz = dir.z / dlen;
    var reach = this.width / 2 + this.ladderMountReach;
    var feetY = p.y - this.height / 2;
    var probeY = feetY + this.stepHeight / 2;
    var hit = raycast(this.world,
        new Vector3(p.x, probeY, p.z),
        new Vector3(p.x + dx * reach, probeY, p.z + dz * reach),
        this._ignoreSelf);
    if (!hit || !hit.object || !hit.object.isLadder) { return null; }
    return hit;
};

/**
 * Is there a too-steep-but-climbable slope surface rising just ahead of the move? Used only when
 * climbSteepSlopes is on. Casts down-rays a short distance ahead and looks for an upward-tilted,
 * too-steep-to-stand hit that is still a real slope (not flat floor, not a vertical wall).
 * @method _climbableSlopeAhead
 * @private
 * @param {Vector3} start
 * @param {Number} dx - unit-ish horizontal direction x
 * @param {Number} dz - unit-ish horizontal direction z
 * @return {Boolean}
 */
proto._climbableSlopeAhead = function(start, dx, dz) {
    if (dx === 0 && dz === 0) { return false; }
    var feetY = this.body.position.y - this.height / 2;
    var base = this.depth / 2 + this._skin; // footprint edge (both scale with the character)
    for (var mi = 0; mi < FPSC.CLIMB_PROBE_DEPTH_MULTS.length; mi++) {
        var m = FPSC.CLIMB_PROBE_DEPTH_MULTS[mi];
        var ahead = base + m * this.depth; // reach past the footprint in units of depth (scale-invariant)
        var ax = start.x + dx * ahead;
        var az = start.z + dz * ahead;
        var hit = raycast(this.world,
            new Vector3(ax, feetY + this.stepHeight + this._skin, az),
            new Vector3(ax, feetY - this.stepHeight, az),
            this._ignoreSelf);
        if (hit && hit.normal.y > FPSC.NY_STEEP_MIN && hit.normal.y < this._minStandableNormalY) { return true; }
    }
    return false;
};

/**
 * Lowest ceiling clearance over the footprint centered at (cx,cz). Infinity if nothing overhead.
 * @method _ceilingClearanceAt
 * @private
 * @param {Number} cx
 * @param {Number} cz
 * @param {Number} feetY
 * @return {Number} clearance in units above feetY, or Infinity.
 */
proto._ceilingClearanceAt = function(cx, cz, feetY) {
    // Start above step-up height so a steppable obstacle (stair/low box) doesn't register as a
    // low ceiling; anything below feet+stepHeight is the ground clamp's job, not the gate's.
    var startY = feetY + this.stepHeight + this._skin;
    var endY = feetY + this.standHeight + this._skin;
    var ix = this.width / 2 - this._skin;
    var iz = this.depth / 2 - this._skin;
    var offsets = [[0, 0], [ix, 0], [-ix, 0], [0, iz], [0, -iz]];
    var lowest = Infinity;
    for (var i = 0; i < offsets.length; i++) {
        var ox = offsets[i][0];
        var oz = offsets[i][1];
        var hit = raycast(this.world,
            new Vector3(cx + ox, startY, cz + oz),
            new Vector3(cx + ox, endY, cz + oz),
            this._ignoreSelf);
        if (!hit || hit.normal.y > FPSC.NY_CEILING) { continue; } // not a ceiling (must face downward)
        // A dynamic/pushable object is never a "ceiling" — it's something the swept mover + push handle,
        // not the headroom gate. Without this, an object being actively shoved forward can wobble a few
        // degrees off-axis from contact torque, and its top face intermittently pokes above the
        // stepHeight cutoff below on some ticks but not others, flickering the character's forward
        // velocity to zero and back as the box jitters. Only STATIC geometry (mass===Infinity) counts
        // as an overhang.
        if (hit.object && hit.object.bodyType === RigidBody.DYNAMIC) { continue; }
        var clr = hit.point.y - feetY;
        // A "ceiling" clearance at or below step height is NOT an overhang — it's a low obstacle at
        // shin/waist level that the swept mover + push handle, not the headroom gate. Without this, a
        // low stepHeight drops the ray start (feetY+stepHeight) INTO a waist-high object ahead, and the
        // ray reports a bogus ~stepHeight-clearance "ceiling", so the gate walls the character in open
        // space in front of a pushable box (worse the lower stepHeight is). Only count genuine overhangs
        // — clearance meaningfully above the step line — as ceilings.
        if (clr <= this.stepHeight + this._skin) { continue; }
        if (clr < lowest) { lowest = clr; }
    }
    return lowest;
};

/**
 * The "too steep to stand on" rule — a floor whose normal tilts below the standable limit gives no
 * footing (MOVE_SLIP). climbSteepSlopes opts out.
 * @method _isSlipSurface
 * @private
 * @param {Object} normal - a surface normal (uses .y)
 * @return {Boolean}
 */
proto._isSlipSurface = function(normal) {
    return !this.climbSteepSlopes && normal.y < this._minStandableNormalY;
};
