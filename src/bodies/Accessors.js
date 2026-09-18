var proto = RigidBody.prototype;

proto.getTransform = function () {
    if (!this._transform) this._transform = new Transform();
    this._transform.syncFromPhysicsBody(this);
    return this._transform;
};

proto.findSupportPoint = function (direction, out) {
    const scratchDir = RigidBody._scratchSupportDir;
    RigidBody._scratchInvRot.copy(this.rotation).invert();
    RigidBody._scratchInvRot.transformVectorInto(direction, scratchDir);
    this.shape.supportInto(out, scratchDir);
    this.rotation.transformVectorInPlace(out);
    out.addInPlace(this.position);
    return out;
};

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

proto._speculativeVeto = function (contact, other) {
    const list = this._listeners.speculativeContact;
    if (!list) return true;
    for (let i = 0; i < list.length; i++) {
        if (list[i]({ contact: contact, other: other }) === false) return false;
    }
    return true;
};
