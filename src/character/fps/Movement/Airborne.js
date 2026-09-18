// Airborne assists: deflecting velocity off a ceiling on the way up (_ceilingSlide), and gating
// horizontal advance into an overhang too low to fit under (_headroomGate). Both are filters on the
// velocity the active movement state produced this tick.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Deflect velocity along an overhead surface instead of capping the rise to zero (a hard cap glues us
 * to ceilings). Projects out the into-surface component using the ceiling's normal: v -= (v.n)n.
 *
 * @method _ceilingSlide
 * @private
 * @param {Number} vx
 * @param {Number} vy
 * @param {Number} vz
 * @param {Number} dt
 * @return {Object} result - { vx, vy, vz }
 */
proto._ceilingSlide = function(vx, vy, vz, dt) {
    if (vy <= 0) { return { vx: vx, vy: vy, vz: vz }; } // not rising -> nothing overhead to resolve
    var reach = this.height + vy * dt + this._skin;
    var ceil = this._probeCeiling(reach);
    if (!ceil) { return { vx: vx, vy: vy, vz: vz }; }
    var gap = ceil.point.y - (this.body.position.y + this.height / 2);
    if (gap > vy * dt + this._skin) { return { vx: vx, vy: vy, vz: vz }; } // won't reach it this tick
    var n = ceil.normal; // down-facing (n.y < 0)
    var dot = vx * n.x + vy * n.y + vz * n.z;
    if (dot < 0) {
        // Heading into the surface: remove that component, leaving motion tangent to it.
        vx -= dot * n.x;
        vy -= dot * n.y;
        vz -= dot * n.z;
    }
    return { vx: vx, vy: vy, vz: vz };
};

/**
 * Treat insufficient headroom as a virtual wall: gate on ceiling clearance ahead (a near-horizontal
 * ramp underside provides no usable surface normal) and slide along the horizontal clearance gradient.
 * @method _headroomGate
 * @private
 * @param {Number} vx
 * @param {Number} vz
 * @param {Number} dt
 * @return {Object} result - { x, z }
 */
proto._headroomGate = function(vx, vz, dt) {
    var speed = Math.sqrt(vx * vx + vz * vz);
    if (speed < FPSC.EPS_DIR) { return { x: vx, z: vz }; }

    if (this.climbSteepSlopes && this._climbableSlopeAhead(this.body.position, vx / speed, vz / speed)) {
        return { x: vx, z: vz };
    }

    var p = this.body.position;
    var feetY = p.y - this.height / 2;
    var need = this.height + this._skin;
    var halfDiag = Math.sqrt((this.width / 2) * (this.width / 2) + (this.depth / 2) * (this.depth / 2));

    // Check clearance at the CURRENT position: _ceilingClearanceAt already samples the full footprint
    // including the leading edge, so projecting a "reach" forward would double-count.
    if (this._ceilingClearanceAt(p.x, p.z, feetY) >= need) { return { x: vx, z: vz }; }

    var eps = halfDiag + this._skin;
    var cR = this._ceilingClearanceAt(p.x + eps, p.z, feetY);
    var cL = this._ceilingClearanceAt(p.x - eps, p.z, feetY);
    var cF = this._ceilingClearanceAt(p.x, p.z + eps, feetY);
    var cB = this._ceilingClearanceAt(p.x, p.z - eps, feetY);

    var cap = this.standHeight + this._skin;
    function fin(c) { return isFinite(c) ? c : cap; }
    var gx = fin(cR) - fin(cL);
    var gz = fin(cF) - fin(cB);
    var glen = Math.sqrt(gx * gx + gz * gz);
    if (glen < FPSC.EPS_DIR) {
        // Grounded: stop (forces a crouch). Airborne: let horizontal flow, ceiling slide owns vertical.
        return this.grounded ? { x: 0, z: 0 } : { x: vx, z: vz };
    }
    gx /= glen;
    gz /= glen;

    var into = vx * gx + vz * gz;
    if (into < 0) {
        vx -= into * gx;
        vz -= into * gz;
    }
    return { x: vx, z: vz };
};
