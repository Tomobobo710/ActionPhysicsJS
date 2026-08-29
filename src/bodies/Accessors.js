// Support point, transform sync, ray cast, and event listeners.
var proto = RigidBody.prototype;

// A Transform (position + rotation + scale) synced from this body's own position/rotation, for
// consumer code (tests, queries) that wants Transform's own API. Does not replace position/rotation
// as this body's real state - every solver/narrowphase/query call site reads those fields directly.
// Lazily allocated once, then reused and re-synced on every call.
proto.getTransform = function () {
    if (!this._transform) this._transform = new Transform();
    this._transform.syncFromPhysicsBody(this);
    return this._transform;
};

// World-space support point: the farthest point on this body's shape along world-space
// `direction`. Same composition MinkowskiSupport uses internally (inverse-rotate into local space,
// call the shape's own supportInto, rotate back, translate) - exposed standalone for a caller with
// no reason to construct a MinkowskiSupport (which pairs two bodies) for a single-body question.
proto.findSupportPoint = function (direction, out) {
    const scratchDir = RigidBody._scratchSupportDir;
    RigidBody._scratchInvRot.copy(this.rotation).invert();
    RigidBody._scratchInvRot.transformVectorInto(direction, scratchDir);
    this.shape.supportInto(out, scratchDir);
    this.rotation.transformVectorInPlace(out);
    out.addInPlace(this.position);
    return out;
};

// Casts against THIS body alone, for a caller that already holds a body reference and wants a hit
// test against just that shape without World.rayIntersect's whole-scene search.
proto.rayIntersect = function (start, end) {
    return Queries.rayIntersectBody(start, end, this);
};

proto.addListener = function (event, fn) {
    (this._listeners[event] || (this._listeners[event] = [])).push(fn);
    return this;
};

proto.emit = function (event, arg) {
    const list = this._listeners[event];
    if (!list) return;
    for (let i = 0; i < list.length; i++) list[i](arg);
};

// Runs this body's speculativeContact listeners; returns false if any vetoes the point.
proto._speculativeVeto = function (contact, other) {
    const list = this._listeners.speculativeContact;
    if (!list) return true;
    for (let i = 0; i < list.length; i++) {
        if (list[i]({ contact: contact, other: other }) === false) return false;
    }
    return true;
};
