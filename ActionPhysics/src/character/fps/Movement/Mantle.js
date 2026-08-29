// Mantle: two-phase kinematic arc (lift, then vault) onto a ledge too tall to step onto.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;
var raycast = FPSCharacterController._raycast;

/**
 * Probe forward for a near-vertical face, then down from the hit for its top surface.
 * @method _probeLedgeAhead
 * @private
 * @param {Number} dx
 * @param {Number} dz
 * @return {Object|null} { faceNormal, topPoint, topNormal, probeDx, probeDz }
 */
proto._probeLedgeAhead = function(dx, dz) {
    var dlen = Math.sqrt(dx * dx + dz * dz);
    if (dlen < FPSC.EPS_LEN) { return null; }
    dx /= dlen; dz /= dlen;

    var p = this.body.position;
    var feetY = p.y - this.height / 2;
    var headY = p.y + this.height / 2;

    var reach = this.width / 2 + this.mantleReach;
    // Probe at several heights so a ledge is found whether level, jumped above, or low.
    var probeHeights = [feetY + this.mantleHeight, p.y, feetY + this.stepHeight];

    for (var hi = 0; hi < probeHeights.length; hi++) {
        var probeY = probeHeights[hi];
        var fwdHit = raycast(
            this.world,
            new Vector3(p.x, probeY, p.z),
            new Vector3(p.x + dx * reach, probeY, p.z + dz * reach),
            this._ignoreSelf
        );
        if (!fwdHit) { continue; }
        if (Math.abs(fwdHit.normal.y) > FPSC.NY_GROUNDISH) { continue; } // not a near-vertical face

        var topProbeX = fwdHit.point.x + dx * this._skin;
        var topProbeZ = fwdHit.point.z + dz * this._skin;
        var scanTop = feetY + this.mantleHeight + this._skin;
        var scanBot = feetY + this.stepHeight + this._skin;
        if (scanTop <= scanBot) { continue; }

        // World.rayIntersect reports only the single NEAREST body along the down-probe. Walk past any
        // surface that doesn't belong to the grabbed face's own object (e.g. a ceiling or disconnected
        // surface above it) by adding each mismatched hit to the query's own `ignore` list and
        // re-querying, restoring the "skip past it, find the real one" behavior a single-hit query
        // can't give for free. This was a REAL, confirmed bug: a ledge with a low ceiling directly
        // above it (a common "duck under this, mantle that" layout) made the down-probe find the
        // ceiling's OWN surface first, discard it as a mismatch, and give up outright — reporting no
        // ledge top at all instead of the real one just below, indistinguishable from "no ledge here."
        var downIgnore = this._ghost ? [this.body, this._ghost] : [this.body];
        var downHit = null;
        for (var dtries = 0; dtries < 4; dtries++) {
            var dh = this.world.rayIntersect(
                new Vector3(topProbeX, scanTop, topProbeZ),
                new Vector3(topProbeX, scanBot, topProbeZ),
                downIgnore
            );
            if (!dh) { break; }
            if (dh.body === fwdHit.object && dh.normal.y >= FPSC.NY_FLOORLIKE) {
                downHit = { point: dh.point, normal: dh.normal, object: dh.body };
                break;
            }
            downIgnore.push(dh.body);
        }
        if (!downHit) { continue; }

        return {
            faceNormal: fwdHit.normal,
            topPoint: downHit.point,
            topNormal: downHit.normal,
            probeDx: dx,
            probeDz: dz
        };
    }
    return null;
};

/**
 * Mantle state machine, mirrors _updateLadder's contract. Called once per beginStep before the
 * main dispatch; returns true while the arc owns the tick.
 * @method _updateMantle
 * @private
 * @param {Object} cmd
 * @param {Number} moveYaw
 * @param {Number} dt
 * @return {Boolean}
 */
proto._updateMantle = function(cmd, moveYaw, dt) {
    var gb = this.body.linear_velocity;
    var p = this.body.position;

    // Active arc: drives position directly (bypasses _collideAndSlide, which would otherwise
    // treat the grabbed face as a blocking wall).
    if (this._mantleActive) {
        this._mantleTimer += dt;
        var total = this.mantleDuration;
        var liftEnd = total * this.mantleLiftFrac;
        var frac = Math.min(1, this._mantleTimer / total);

        var newX, newY, newZ;
        if (this._mantleTimer <= liftEnd) {
            var liftFrac = Math.min(1, this._mantleTimer / Math.max(liftEnd, FPSC.EPS_LEN));
            newX = this._mantleStartX;
            newZ = this._mantleStartZ;
            newY = this._mantleStartY + (this._mantleTopBodyY - this._mantleStartY) * liftFrac;
        } else {
            var vaultFrac = Math.min(1, (this._mantleTimer - liftEnd) / Math.max(total - liftEnd, FPSC.EPS_LEN));
            newX = this._mantleStartX + (this._mantleLandX - this._mantleStartX) * vaultFrac;
            newZ = this._mantleStartZ + (this._mantleLandZ - this._mantleStartZ) * vaultFrac;
            newY = this._mantleTopBodyY;
        }

        gb.x = 0; gb.y = 0; gb.z = 0;
        this.body.position.set(newX, newY, newZ);
        this.body.updateDerived();

        var done = frac >= 1;
        if (done) {
            this._mantleActive = false;
            this._mantleTimer = 0;
            this._moveState = FPSC.MOVE_AIRBORNE;
            this.body.setGravity(this._gravityVec.x, this._gravityVec.y, this._gravityVec.z);
            gb.x = 0; gb.y = 0; gb.z = 0;
            return false; // release to normal dispatch this tick so endStep can land us
        }

        return true;
    }

    // Detection: only on an explicit tap.
    if (!cmd.mantle) { return false; }

    var fwd = this.getForwardHorizontal(moveYaw);
    var ledge = this._probeLedgeAhead(fwd.x, fwd.z);
    if (!ledge) { return false; }

    var feetY = p.y - this.height / 2;
    var rise = ledge.topPoint.y - feetY;

    if (rise <= this.stepHeight + this._skin) { return false; } // step-up handles it
    if (rise > this.mantleHeight) { return false; } // out of reach
    // Above chest height, a grounded tap is refused — must already be airborne to grab it.
    var chestHeight = this.standHeight * FPSC.MANTLE_CHEST_HEIGHT_FRAC;
    if (rise > chestHeight && this.grounded) { return false; }

    // Landing point: advance from the grab point past the face by the character's own depth,
    // stepping back toward the face if that overshoots a shallow ledge, until solid standable
    // ground is found.
    var dx = ledge.probeDx, dz = ledge.probeDz;
    var topBodyY = ledge.topPoint.y + this.height / 2;
    var desiredAdvance = this.depth;
    var landX = p.x, landZ = p.z, landFound = false;
    var steps = 4;
    for (var s = steps; s >= 1; s--) {
        var advance = desiredAdvance * (s / steps);
        var tryX = ledge.topPoint.x + dx * advance;
        var tryZ = ledge.topPoint.z + dz * advance;
        var landHit = raycast(
            this.world,
            new Vector3(tryX, topBodyY, tryZ),
            new Vector3(tryX, ledge.topPoint.y - this._skin - this.mantleHeight, tryZ),
            this._ignoreSelf
        );
        if (landHit && landHit.normal.y >= FPSC.NY_FLOORLIKE &&
            Math.abs(landHit.point.y - ledge.topPoint.y) < this.stepHeight + this._skin) {
            landX = tryX; landZ = tryZ; landFound = true;
            break;
        }
    }
    if (!landFound) { return false; }

    // The arc drives position directly, so nothing else checks headroom along the way — verify
    // both the grab point and the landing point can stand up.
    var clearanceAtGrab = this._ceilingClearanceAt(p.x, p.z, ledge.topPoint.y);
    if (clearanceAtGrab < this.standHeight - this._skin) { return false; }
    var clearanceAtLand = this._ceilingClearanceAt(landX, landZ, ledge.topPoint.y);
    if (clearanceAtLand < this.standHeight - this._skin) { return false; }

    this._mantleActive = true;
    this._mantleTimer = 0;
    this._mantleStartX = p.x;
    this._mantleStartY = p.y;
    this._mantleStartZ = p.z;
    this._mantleTopBodyY = topBodyY;
    this._mantleLandX = landX;
    this._mantleLandZ = landZ;

    this._moveState = FPSC.MOVE_MANTLE;
    this.grounded = false;
    this._groundSuppress = FPSC.GROUND_SUPPRESS_JUMP;
    this._jumpRising = true;
    this.body.setGravity(0, 0, 0);
    gb.x = 0; gb.y = 0; gb.z = 0;

    return true;
};
