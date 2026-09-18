// Ground/ceiling/ladder raycast probes: the spatial queries beginStep/endStep read each tick to
// decide grounding, slope classification, headroom and ladder mounting. None writes sim state.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;
var raycast = FPSCharacterController._raycast;

/**
 * Multi-point ground probe (center + four edge midpoints). Returns ALL floor-like hits, highest
 * first, so the caller can fall back to a lower valid hit when the highest is too tall to step onto.
 * @method _probeGroundCandidates
 * @private
 * @param {Number} maxSnap - max downward reach (below the feet) to probe, before scale/skin margins.
 * @return {Object[]} floor-like raycast hits, sorted highest point.y first.
 */
proto._probeGroundCandidates = function(maxSnap) {
    var half = this.height / 2;
    var p = this.body.position;
    // Cast from the higher of this tick's start position and the current position, so a fast descent
    // that penetrated the floor this tick doesn't miss it.
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
        // Exclude a pushable object as ground only when walking INTO its side, not when standing on it.
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
 * Multi-ray UP probe across the footprint. Returns the LOWEST ceiling (down-facing surface) within
 * `reachAboveFeet` of the feet, or null. Covers sloped overhead geometry forward rays can't see.
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
 * Like _raycast (excludes this body + ghost), but also skips a hit body tagged isPlatform. A raw
 * raycast doesn't consult collision_mask, so without this a platform catching back up to a character
 * mid-jump reads as a solid ceiling over _ceilingSlide. Used only by _probeCeiling.
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
 * Single ray probe for a ladder ahead, along `dir`. Placed halfway between the feet and stepHeight
 * above them rather than at the body center. Returns the raw hit with the normal flipped to point OUT
 * of the face (toward the caller), or null.
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
 * Is there a too-steep-but-climbable slope surface rising just ahead of the move? Only used when
 * climbSteepSlopes is on.
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
    // Start above step-up height so a steppable obstacle (stair/low box) doesn't register as a ceiling.
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
        // Only STATIC geometry counts as an overhang — a shoved dynamic object can wobble its top face
        // above the cutoff intermittently.
        if (hit.object && hit.object.bodyType === RigidBody.DYNAMIC) { continue; }
        var clr = hit.point.y - feetY;
        // A clearance at or below step height is a low obstacle, not an overhang (the swept mover + push
        // handle it), so it must not wall the character in.
        if (clr <= this.stepHeight + this._skin) { continue; }
        if (clr < lowest) { lowest = clr; }
    }
    return lowest;
};

/**
 * The "too steep to stand on" rule — a floor whose normal tilts below the standable limit gives no
 * footing. climbSteepSlopes opts out.
 * @method _isSlipSurface
 * @private
 * @param {Object} normal - a surface normal (uses .y)
 * @return {Boolean}
 */
proto._isSlipSurface = function(normal) {
    return !this.climbSteepSlopes && normal.y < this._minStandableNormalY;
};
