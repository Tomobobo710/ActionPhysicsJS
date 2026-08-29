/**
 * ActionPhysics - a deterministic, dependency-free 3D physics engine.
 *
 * Ships as a single concatenated file. Loads from a plain <script> tag with no server, no bundler and
 * no modules; also usable via require() in Node.
 *
 * MATH IS INJECTABLE. ActionPhysics runs on ActionMath, and carries its own copy so it works alone.
 * But if the host already has ActionMath loaded, ActionPhysics uses the host's classes rather than
 * defining a second set. Without that, a page loading both ends up with two distinct Vector3 classes -
 * two static object pools, and `instanceof` silently false across the boundary.
 *
 * Injection is by name, checked before the bundled copy is defined:
 *   - explicitly, via window.ActionMath = { Vector3, Quaternion, ... } set before this file loads
 *   - or implicitly, by finding the classes already declared in scope
 * Falling back to the bundled copy when neither is present.
 */
(function (root, factory) {
    'use strict';

    // Whatever math the host already has, if any. A host that concatenates its own ActionMath into the
    // same page exposes the classes at script scope, which `typeof` reaches but `root.X` does not.
    var injected = (typeof root.ActionMath === 'object' && root.ActionMath) ? root.ActionMath : {};
    function adopt(name, scoped) {
        if (injected[name]) return injected[name];
        return scoped || null;
    }
    var host = {
        Scalar:     adopt('Scalar',     typeof Scalar     !== 'undefined' ? Scalar     : null),
        Vector2:    adopt('Vector2',    typeof Vector2    !== 'undefined' ? Vector2    : null),
        Vector3:    adopt('Vector3',    typeof Vector3    !== 'undefined' ? Vector3    : null),
        Matrix3:    adopt('Matrix3',    typeof Matrix3    !== 'undefined' ? Matrix3    : null),
        Matrix4:    adopt('Matrix4',    typeof Matrix4    !== 'undefined' ? Matrix4    : null),
        Quaternion: adopt('Quaternion', typeof Quaternion !== 'undefined' ? Quaternion : null),
        Transform:  adopt('Transform',  typeof Transform  !== 'undefined' ? Transform  : null)
    };

    var api = factory(host);
    if (typeof module === 'object' && module.exports) {
        module.exports = api;
    } else {
        root.ActionPhysics = api;
    }
}(typeof globalThis !== 'undefined' ? globalThis : this, function (host) {
    'use strict';

    const ActionPhysics = {};

    // True when every math class came from the host rather than the bundled copy. Reported so a
    // consumer can verify at runtime which set is live.
    ActionPhysics.usingHostMath = false;
