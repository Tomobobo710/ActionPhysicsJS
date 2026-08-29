// Airborne assists: deflecting velocity off a ceiling on the way up (_ceilingSlide), and gating
// horizontal advance into an overhang too low to fit under (_headroomGate). Neither owns a movement
// state by itself — both act as filters on whatever velocity the active state produced this tick.
var proto = FPSCharacterController.prototype;
var FPSC = FPSCharacterController.FPSC;

/**
 * Deflect velocity along an overhead surface we're about to contact, instead of capping the
 * rise to zero — a hard cap leaves no velocity to escape and glues us to ceilings, flat AND
 * sloped. Projects out the into-surface component using the ceiling's own normal: v -= (v.n) n.
 * A flat underside (n straight down) zeroes only the vertical, so horizontal motion survives; a
 * sloped underside redirects the upward motion down-and-along the slope, sliding us out. Only
 * acts when actually rising toward a ceiling within this tick's reach.
 *
 * @method _ceilingSlide
 * @private
 * @param {Number} vx
 * @param {Number} vy
 * @param {Number} vz
 * @param {Number} dt
 * @return {Object} result
 * @return {Number} result.vx
 * @return {Number} result.vy
 * @return {Number} result.vz
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
 * Treat insufficient headroom as a virtual wall: gate on ceiling clearance ahead (rather than
 * surface normal, which a near-horizontal ramp underside can't provide) and slide along the
 * horizontal gradient of increasing clearance.
 * @method _headroomGate
 * @private
 * @param {Number} vx
 * @param {Number} vz
 * @param {Number} dt
 * @return {Object} result
 * @return {Number} result.x - gated horizontal velocity, x.
 * @return {Number} result.z - gated horizontal velocity, z.
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

    // Check clearance centered at the CURRENT position, not a forward-projected point — _ceilingClearanceAt
    // already samples +-(width/2-skin) / +-(depth/2-skin) around its center argument, which is the box's own
    // full footprint including its leading edge. Projecting a "reach" forward on top of that double-counts.
    // The footprint offsets ARE the reach.
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
