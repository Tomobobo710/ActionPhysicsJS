/**
 * ActionPhysics - a deterministic, dependency-free 3D physics engine. Ships as one concatenated
 * file, loadable from a <script> tag or require().
 *
 * Math is injectable: ActionPhysics runs on ActionMath and bundles its own copy, but if the host
 * already has ActionMath (via window.ActionMath, or the classes in scope) it adopts those instead,
 * so a page loading both doesn't end up with two Vector3 classes and `instanceof` false across them.
 */
(function (root, factory) {
    'use strict';

    // A host that concatenates its own ActionMath exposes the classes at script scope (`typeof`
    // reaches them, `root.X` does not).
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

    // True when every math class came from the host rather than the bundled copy.
    ActionPhysics.usingHostMath = false;
