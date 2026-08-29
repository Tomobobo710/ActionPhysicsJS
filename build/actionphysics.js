// ActionPhysics 0.1.0 — built 2026-08-23T23:33:19.630Z
// ==== src/intro.js ====
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


// ==== src/math/Scalar.js ====
var Scalar;
if (host.Scalar) {
    Scalar = host.Scalar;
} else {
    // Part of ActionMath - the shared math library. This file is pasted here VERBATIM from its source of
    // truth and must not be edited locally. Anything this engine needs is added to ActionMath first, then
    // arrives here through the same paste.
    /**
     * Deterministic scalar functions.
     *
     * The JS spec mandates correctly-rounded arithmetic and sqrt with no extended precision, so that much
     * is already bit-identical across engines and platforms. It does NOT specify sin/cos/tan/asin/acos/atan/
     * atan2/pow/exp/log - those are implementation-approximated and differ in low bits between V8,
     * SpiderMonkey and JavaScriptCore.
     *
     * Everything the rest of the maths needs from that list is implemented here in terms of the operations
     * that ARE specified, so results are reproducible across machines. Anything that must be deterministic
     * calls Scalar rather than Math.
     */
    var Scalar = {};

    Scalar.PI = 3.141592653589793;
    Scalar.TWO_PI = 6.283185307179586;
    Scalar.HALF_PI = 1.5707963267948966;
    Scalar.EPSILON = 1e-9;

    /**
     * Reduce an angle to [-PI, PI]. Uses Cody-Waite style splitting of 2*PI into high and low
     * parts so the subtraction stays exact for the magnitudes physics produces; a single
     * multiply-subtract loses precision for large angles.
     */
    Scalar.wrapAngle = function (x) {
        const TWO_PI_HI = 6.28318530717958623200;
        const TWO_PI_LO = 2.44929359829470635445e-16;
        if (x >= -Scalar.PI && x <= Scalar.PI) {
            return x;
        }
        // Round-to-nearest count of full turns, without Math.round on a huge value.
        const turns = x / Scalar.TWO_PI;
        const n = turns >= 0 ? Math.floor(turns + 0.5) : Math.ceil(turns - 0.5);
        return (x - n * TWO_PI_HI) - n * TWO_PI_LO;
    };

    /**
     * sin(x) — minimax polynomial on the reduced argument.
     *
     * Reduce to [-PI, PI], fold into [-PI/2, PI/2] using sin(PI - x) = sin(x), then evaluate a
     * degree-13 odd polynomial in x^2. Odd-only terms keep sin(-x) === -sin(x) exactly, which
     * matters more for a physics engine than the last ULP: an asymmetric sine injects a
     * directional bias into anything that rotates.
     */
    Scalar.sin = function (x) {
        x = Scalar.wrapAngle(x);

        // Odd: fold to [0, PI] and carry the sign. Doing this by sign rather than by
        // polynomial keeps sin(-x) === -sin(x) bit-exact.
        let sign = 1;
        if (x < 0) { x = -x; sign = -1; }

        // sin(PI - x) = sin(x): fold to [0, PI/2].
        if (x > Scalar.HALF_PI) x = Scalar.PI - x;

        // Both kernels below are only accurate to ~PI/4, so use whichever identity keeps the
        // argument small: sin near 0, cos near PI/2.
        if (x > 0.7853981633974483) {
            return sign * Scalar._cosKernel(Scalar.HALF_PI - x);
        }
        return sign * Scalar._sinKernel(x);
    };

    /** sin on [0, PI/4]. Odd-only polynomial in x^2. */
    Scalar._sinKernel = function (x) {
        const x2 = x * x;
        let p = 1.58962301576546568060e-10;
        p = p * x2 - 2.50507477628578072866e-8;
        p = p * x2 + 2.75573136213857245213e-6;
        p = p * x2 - 1.98412698295895385996e-4;
        p = p * x2 + 8.33333333332211858878e-3;
        p = p * x2 - 1.66666666666666307295e-1;
        return x + x * x2 * p;
    };

    /** cos on [0, PI/4]. Even-only polynomial in x^2. */
    Scalar._cosKernel = function (x) {
        const x2 = x * x;
        let p = -1.13585365213876817300e-11;
        p = p * x2 + 2.08757008419747316778e-9;
        p = p * x2 - 2.75573141792967388112e-7;
        p = p * x2 + 2.48015872888517179954e-5;
        p = p * x2 - 1.38888888888730564116e-3;
        p = p * x2 + 4.16666666666665929218e-2;
        return 1 - 0.5 * x2 + x2 * x2 * p;
    };

    /**
     * cos(x) — its own reduction and its own even polynomial in x^2.
     *
     * NOT sin(x + PI/2): that shift destroys exact evenness (the added constant is not
     * symmetric about zero) and pushes arguments near the peak out to x = PI, the worst point
     * for sin's polynomial — measured cos(0) at 6e-12 off instead of exact. Evaluating an
     * even-only polynomial here makes cos(-x) === cos(x) bit-exact.
     */
    Scalar.cos = function (x) {
        x = Scalar.wrapAngle(x);
        if (x < 0) x = -x;              // even: fold to [0, PI]

        // cos(PI - x) = -cos(x): fold to [0, PI/2].
        let sign = 1;
        if (x > Scalar.HALF_PI) {
            x = Scalar.PI - x;
            sign = -1;
        }

        // Same octant split as sin, for the same reason: each kernel is accurate to ~PI/4.
        if (x > 0.7853981633974483) {
            return sign * Scalar._sinKernel(Scalar.HALF_PI - x);
        }
        return sign * Scalar._cosKernel(x);
    };

    /**
     * atan(x) on the full range, by range reduction onto [0, tan(PI/12)] and an odd polynomial.
     * Odd-only again, so atan(-x) === -atan(x) exactly.
     */
    Scalar.atan = function (x) {
        if (x !== x) return NaN;

        const negate = x < 0;
        if (negate) x = -x;

        let offset = 0;
        if (x > 1) {
            // atan(x) = PI/2 - atan(1/x)
            x = 1 / x;
            offset = Scalar.HALF_PI;
        }
        let invert = offset !== 0;

        // Second reduction: atan(x) = PI/6 + atan((x*sqrt(3) - 1) / (x + sqrt(3)))
        const SQRT3 = 1.7320508075688772;
        let extra = 0;
        if (x > 0.2679491924311227) { // tan(PI/12)
            x = (x * SQRT3 - 1) / (x + SQRT3);
            extra = 0.5235987755982988; // PI/6
        }

        // Rational approximation (Cephes atan), accurate to near machine precision on the
        // reduced range |x| <= tan(PI/12).
        const x2 = x * x;
        let num = -8.750608600031904122785e-1;
        num = num * x2 - 1.615753718733365076637e1;
        num = num * x2 - 7.500855792314704667340e1;
        num = num * x2 - 1.228866684490136173410e2;
        num = num * x2 - 6.485021904942025371773e1;

        let den = x2 + 2.485846490142306297962e1;
        den = den * x2 + 1.650270098316988542046e2;
        den = den * x2 + 4.328810604912902668951e2;
        den = den * x2 + 4.853903996359136964868e2;
        den = den * x2 + 1.945506571482613964425e2;

        let r = x + x * x2 * (num / den);

        r += extra;
        if (invert) r = offset - r;
        return negate ? -r : r;
    };

    /** atan2(y, x) — quadrant-correct, built on Scalar.atan. */
    Scalar.atan2 = function (y, x) {
        if (x > 0) return Scalar.atan(y / x);
        if (x < 0) return y >= 0 ? Scalar.atan(y / x) + Scalar.PI : Scalar.atan(y / x) - Scalar.PI;
        // x === 0
        if (y > 0) return Scalar.HALF_PI;
        if (y < 0) return -Scalar.HALF_PI;
        return 0;
    };

    /**
     * asin(x) for x in [-1, 1]. Out-of-range input is clamped rather than returning NaN:
     * callers reach this through dot products of unit vectors, where |x| can exceed 1 by a few
     * ULP purely from rounding, and that is not an error.
     */
    Scalar.asin = function (x) {
        if (x >= 1) return Scalar.HALF_PI;
        if (x <= -1) return -Scalar.HALF_PI;
        // asin(x) = atan(x / sqrt(1 - x^2)), stable away from |x| = 1.
        return Scalar.atan(x / Math.sqrt(1 - x * x));
    };

    /** acos(x) = PI/2 - asin(x), with the same clamping rationale. */
    Scalar.acos = function (x) {
        if (x >= 1) return 0;
        if (x <= -1) return Scalar.PI;
        return Scalar.HALF_PI - Scalar.asin(x);
    };

    /** tan(x) = sin(x)/cos(x). Callers are responsible for avoiding the poles. */
    Scalar.tan = function (x) {
        return Scalar.sin(x) / Scalar.cos(x);
    };

    /**
     * sqrt(x^2 + y^2 + z^2). Math.hypot is implementation-defined (it does extra work to avoid overflow at
     * extreme magnitudes, and engines differ in how); the direct form is exact under the spec's guarantees
     * for * + and sqrt, and the magnitudes here never approach overflow.
     */
    Scalar.hypot3 = function (x, y, z) {
        return Math.sqrt(x * x + y * y + z * z);
    };

    /** Clamp v into [lo, hi]. */
    Scalar.clamp = function (v, lo, hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };

    ActionPhysics.Scalar = Scalar;

}
ActionPhysics.Scalar = Scalar;


// ==== src/math/Vector2.js ====
var Vector2;
if (host.Vector2) {
    Vector2 = host.Vector2;
} else {
    // Part of ActionMath - the shared math library. This file is pasted here VERBATIM from its source of
    // truth and must not be edited locally. Anything this engine needs is added to ActionMath first, then
    // arrives here through the same paste.
    Vector2 = class Vector2 {
        constructor(x = 0, y = 0) {
            this.x = x;
            this.y = y;
        }

        // Static creation methods
        static create(x = 0, y = 0) {
            return new Vector2(x, y);
        }

        static fromAngle(angle) {
            return new Vector2(Scalar.cos(angle), Scalar.sin(angle));
        }

        static fromArray(arr, offset = 0) {
            return new Vector2(arr[offset], arr[offset + 1]);
        }

        static zero() {
            return new Vector2(0, 0);
        }

        static one() {
            return new Vector2(1, 1);
        }

        static up() {
            return new Vector2(0, -1);
        }

        static down() {
            return new Vector2(0, 1);
        }

        static left() {
            return new Vector2(-1, 0);
        }

        static right() {
            return new Vector2(1, 0);
        }

        // Basic operations (modifying this vector)
        set(x, y) {
            this.x = x;
            this.y = y;
            return this;
        }

        copy(v) {
            this.x = v.x;
            this.y = v.y;
            return this;
        }

        add(v) {
            this.x += v.x;
            this.y += v.y;
            return this;
        }

        subtract(v) {
            this.x -= v.x;
            this.y -= v.y;
            return this;
        }

        multiply(v) {
            this.x *= v.x;
            this.y *= v.y;
            return this;
        }

        divide(v) {
            this.x /= v.x;
            this.y /= v.y;
            return this;
        }

        scale(scalar) {
            this.x *= scalar;
            this.y *= scalar;
            return this;
        }

        negate() {
            this.x = -this.x;
            this.y = -this.y;
            return this;
        }

        normalize() {
            const len = this.length();
            if (len > 0) {
                this.x /= len;
                this.y /= len;
            }
            return this;
        }

        rotate(angle) {
            const cos = Scalar.cos(angle);
            const sin = Scalar.sin(angle);
            const x = this.x * cos - this.y * sin;
            const y = this.x * sin + this.y * cos;
            this.x = x;
            this.y = y;
            return this;
        }

        // Vector properties
        length() {
            return Math.sqrt(this.x * this.x + this.y * this.y);
        }

        lengthSquared() {
            return this.x * this.x + this.y * this.y;
        }

        angle() {
            return Scalar.atan2(this.y, this.x);
        }

        distanceTo(v) {
            const dx = this.x - v.x;
            const dy = this.y - v.y;
            return Math.sqrt(dx * dx + dy * dy);
        }

        distanceToSquared(v) {
            const dx = this.x - v.x;
            const dy = this.y - v.y;
            return dx * dx + dy * dy;
        }

        dot(v) {
            return this.x * v.x + this.y * v.y;
        }

        cross(v) {
            return this.x * v.y - this.y * v.x;
        }

        // Utility methods
        clone() {
            return new Vector2(this.x, this.y);
        }

        equals(v) {
            return this.x === v.x && this.y === v.y;
        }

        isZero() {
            return this.x === 0 && this.y === 0;
        }

        toString() {
            return `Vector2(${this.x}, ${this.y})`;
        }

        toArray() {
            return [this.x, this.y];
        }

        // Static operations (returning new vectors)
        static add(out, a, b) {
            out.x = a.x + b.x;
            out.y = a.y + b.y;
            return out;
        }

        static subtract(out, a, b) {
            out.x = a.x - b.x;
            out.y = a.y - b.y;
            return out;
        }

        static multiply(out, a, b) {
            out.x = a.x * b.x;
            out.y = a.y * b.y;
            return out;
        }

        static divide(out, a, b) {
            out.x = a.x / b.x;
            out.y = a.y / b.y;
            return out;
        }

        static scale(out, v, scalar) {
            out.x = v.x * scalar;
            out.y = v.y * scalar;
            return out;
        }

        static lerp(out, a, b, t) {
            out.x = a.x + (b.x - a.x) * t;
            out.y = a.y + (b.y - a.y) * t;
            return out;
        }

        static min(out, a, b) {
            out.x = Math.min(a.x, b.x);
            out.y = Math.min(a.y, b.y);
            return out;
        }

        static max(out, a, b) {
            out.x = Math.max(a.x, b.x);
            out.y = Math.max(a.y, b.y);
            return out;
        }

        static normalize(out, v) {
            const len = v.length();
            if (len > 0) {
                out.x = v.x / len;
                out.y = v.y / len;
            }
            return out;
        }

        static rotate(out, v, angle) {
            const cos = Scalar.cos(angle);
            const sin = Scalar.sin(angle);
            out.x = v.x * cos - v.y * sin;
            out.y = v.x * sin + v.y * cos;
            return out;
        }

        static dot(a, b) {
            return a.x * b.x + a.y * b.y;
        }

        static cross(a, b) {
            return a.x * b.y - a.y * b.x;
        }

        static distance(a, b) {
            const dx = a.x - b.x;
            const dy = a.y - b.y;
            return Math.sqrt(dx * dx + dy * dy);
        }

        static distanceSquared(a, b) {
            const dx = a.x - b.x;
            const dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        static angle(a, b) {
            return Scalar.atan2(b.y - a.y, b.x - a.x);
        }

        // Advanced operations
        static reflect(out, v, normal) {
            const dot = v.x * normal.x + v.y * normal.y;
            out.x = v.x - 2 * dot * normal.x;
            out.y = v.y - 2 * dot * normal.y;
            return out;
        }

        static project(out, a, b) {
            const dot = (a.x * b.x + a.y * b.y) / (b.x * b.x + b.y * b.y);
            out.x = b.x * dot;
            out.y = b.y * dot;
            return out;
        }

        static perpendicular(out, v) {
            out.x = -v.y;
            out.y = v.x;
            return out;
        }
    }

    ActionPhysics.Vector2 = Vector2;

}
ActionPhysics.Vector2 = Vector2;


// ==== src/math/Vector3.js ====
var Vector3;
if (host.Vector3) {
    Vector3 = host.Vector3;
} else {
    // Part of ActionMath - the shared math library. This file is pasted here VERBATIM from its source of
    // truth and must not be edited locally. Anything this engine needs is added to ActionMath first, then
    // arrives here through the same paste.
    Vector3 = class Vector3 {
        // Vector pool for object reuse
        static _pool = [];
        static _poolSize = 0;
        static _maxPoolSize = 1000;

        // Get a vector from the pool or create a new one
        static getFromPool(x = 0, y = 0, z = 0) {
            if (Vector3._poolSize > 0) {
                const vec = Vector3._pool[--Vector3._poolSize];
                vec.set(x, y, z);
                return vec;
            }
            return new Vector3(x, y, z);
        }

        // Return a vector to the pool when done with it
        static returnToPool(vec) {
            if (Vector3._poolSize < Vector3._maxPoolSize) {
                Vector3._pool[Vector3._poolSize++] = vec;
            }
        }
        constructor(x = 0, y = 0, z = 0) {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        set(x, y, z) {
            if (y === undefined && z === undefined && x.x !== undefined) {
                // If passed another vector
                this.x = x.x;
                this.y = x.y;
                this.z = x.z;
            } else {
                // If passed 3 numbers
                this.x = x;
                this.y = y;
                this.z = z;
            }
            return this;
        }

        // Distance between two vectors
        static distance(a, b) {
            return Scalar.hypot3(a.x - b.x, a.y - b.y, a.z - b.z);
        }

        // For distance calculations between points
        distanceTo(other) {
            const dx = this.x - other.x;
            const dy = this.y - other.y;
            const dz = this.z - other.z;
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        // More efficient squared distance, avoids costly sqrt when possible
        distanceSquared(other) {
            const dx = this.x - other.x;
            const dy = this.y - other.y;
            const dz = this.z - other.z;
            return dx * dx + dy * dy + dz * dz;
        }

        // For horizontal distance (ignoring Y) - useful for camera calculations
        horizontalDistanceTo(other) {
            const dx = this.x - other.x;
            const dz = this.z - other.z;
            return Math.sqrt(dx * dx + dz * dz);
        }

        // More efficient squared horizontal distance
        horizontalDistanceSquared(other) {
            const dx = this.x - other.x;
            const dz = this.z - other.z;
            return dx * dx + dz * dz;
        }

        // For applying movement/translation
        translate(direction, amount) {
            return new Vector3(this.x + direction.x * amount, this.y + direction.y * amount, this.z + direction.z * amount);
        }

        // In-place version to avoid creating a new Vector3
        translateInPlace(direction, amount) {
            this.x += direction.x * amount;
            this.y += direction.y * amount;
            this.z += direction.z * amount;
            return this;
        }

        // For rotation around Y axis (useful for camera orbiting)
        rotateY(angle) {
            const cos = Scalar.cos(angle);
            const sin = Scalar.sin(angle);
            return new Vector3(this.x * cos + this.z * sin, this.y, -this.x * sin + this.z * cos);
        }

        // In-place version to avoid creating a new Vector3
        rotateYInPlace(angle) {
            const cos = Scalar.cos(angle);
            const sin = Scalar.sin(angle);
            const x = this.x;
            const z = this.z;
            this.x = x * cos + z * sin;
            this.z = -x * sin + z * cos;
            return this;
        }

        // Gets a normalized vector representing just the horizontal component
        horizontalNormalize() {
            return new Vector3(this.x, 0, this.z).normalize();
        }

        static transformMat4(vec, mat) {
            // Make sure we can access the matrix data whether it's Array or Float32Array
            const getElement = (idx) => (mat[idx] !== undefined ? mat[idx] : mat.at(idx));

            const x = vec.x;
            const y = vec.y;
            const z = vec.z;
            let w = getElement(3) * x + getElement(7) * y + getElement(11) * z + getElement(15);
            if (w === 0) w = 1;

            return new Vector3(
                (getElement(0) * x + getElement(4) * y + getElement(8) * z + getElement(12)) / w,
                (getElement(1) * x + getElement(5) * y + getElement(9) * z + getElement(13)) / w,
                (getElement(2) * x + getElement(6) * y + getElement(10) * z + getElement(14)) / w
            );
        }
        static fromValues(x, y, z) {
            return new Vector3(x, y, z);
        }

        static min(out, a, b) {
            out.x = Math.min(a.x, b.x);
            out.y = Math.min(a.y, b.y);
            out.z = Math.min(a.z, b.z);
            return out;
        }

        static max(out, a, b) {
            out.x = Math.max(a.x, b.x);
            out.y = Math.max(a.y, b.y);
            out.z = Math.max(a.z, b.z);
            return out;
        }
        static create(x = 0, y = 0, z = 0) {
            return new Vector3(x, y, z);
        }

        // Add optimized add operation that creates less garbage
        add(other) {
            return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
        }

        // In-place addition
        addInPlace(other) {
            this.x += other.x;
            this.y += other.y;
            this.z += other.z;
            return this;
        }

        // Add optimized subtract operation
        sub(other) {
            return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
        }
        // Subtract vector b from vector a
        static subtract(a, b) {
            return new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
        }
        // In-place subtraction
        subInPlace(other) {
            this.x -= other.x;
            this.y -= other.y;
            this.z -= other.z;
            return this;
        }

        // Vector normalization
        normalize() {
            const len = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
            if (len === 0) {
                return new Vector3(0, 0, 0);
            }
            return new Vector3(this.x / len, this.y / len, this.z / len);
        }

        // In-place normalization
        normalizeInPlace() {
            const len = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
            if (len !== 0) {
                this.x /= len;
                this.y /= len;
                this.z /= len;
            }
            return this;
        }

        // Add dot product operation
        dot(other) {
            return this.x * other.x + this.y * other.y + this.z * other.z;
        }

        // Add cross product operation
        cross(other) {
            return new Vector3(
                this.y * other.z - this.z * other.y,
                this.z * other.x - this.x * other.z,
                this.x * other.y - this.y * other.x
            );
        }

        // Static cross product that writes to an output vector (no allocation)
        // Both operands are read into locals BEFORE any write, so this is safe when `out` is also `a` or
        // `b`. Writing directly would corrupt components still needed by the next line.
        static crossInto(out, a, b) {
            const ax = a.x, ay = a.y, az = a.z;
            const bx = b.x, by = b.y, bz = b.z;
            out.x = ay * bz - az * by;
            out.y = az * bx - ax * bz;
            out.z = ax * by - ay * bx;
            return out;
        }

        // Array conversion
        toArray() {
            return [this.x, this.y, this.z];
        }

        // Length calculation
        length() {
            return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
        }

        // Squared length (faster, avoids sqrt)
        lengthSquared() {
            return this.x * this.x + this.y * this.y + this.z * this.z;
        }

        mult(n) {
            return new Vector3(this.x * n, this.y * n, this.z * n);
        }

        scale(scalar) {
            return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
        }

        // Scale a vector by a scalar
        static scale(v, scalar) {
            return new Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
        }

        divideScalar(scalar) {
            if (scalar === 0) {
                console.warn("Vector3: Division by zero!");
                return new Vector3(0, 0, 0);
            }

            return new Vector3(this.x / scalar, this.y / scalar, this.z / scalar);
        }

        subtract(other) {
            return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
        }

        equals(other) {
            const epsilon = 0.000001; // Small threshold for floating point comparison
            return (
                Math.abs(this.x - other.x) < epsilon &&
                Math.abs(this.y - other.y) < epsilon &&
                Math.abs(this.z - other.z) < epsilon
            );
        }

        clone() {
            return new Vector3(this.x, this.y, this.z);
        }

        /**
         * Copy the values from another Vector3 into this one
         * @param {Vector3} v - Vector to copy from
         * @returns {Vector3} this vector
         */
        copy(v) {
            this.x = v.x;
            this.y = v.y;
            this.z = v.z;
            return this;
        }

        lerp(target, t) {
            return new Vector3(
                this.x + (target.x - this.x) * t,
                this.y + (target.y - this.y) * t,
                this.z + (target.z - this.z) * t
            );
        }

        // ---- in-place / allocation-free forms ----
        // The physics solver runs these thousands of times per tick and cannot allocate per operation.
        // The allocating forms above are unchanged. `Into` follows the existing crossInto: write into `out`.

        // out = a + b
        static addInto(out, a, b) {
            out.x = a.x + b.x; out.y = a.y + b.y; out.z = a.z + b.z;
            return out;
        }

        // out = a - b
        static subInto(out, a, b) {
            out.x = a.x - b.x; out.y = a.y - b.y; out.z = a.z - b.z;
            return out;
        }

        // out = v * s
        static scaleInto(out, v, s) {
            out.x = v.x * s; out.y = v.y * s; out.z = v.z * s;
            return out;
        }

        // out = normalized v. A zero vector stays zero rather than becoming NaN.
        static normalizeInto(out, v) {
            const lsq = v.x * v.x + v.y * v.y + v.z * v.z;
            if (lsq === 0) { out.x = 0; out.y = 0; out.z = 0; return out; }
            const inv = 1 / Math.sqrt(lsq);
            out.x = v.x * inv; out.y = v.y * inv; out.z = v.z * inv;
            return out;
        }

        // this += v * s
        addScaledInPlace(v, s) {
            this.x += v.x * s; this.y += v.y * s; this.z += v.z * s;
            return this;
        }

        scaleInPlace(s) {
            this.x *= s; this.y *= s; this.z *= s;
            return this;
        }

        // Component-wise product, in place.
        multiplyInPlace(v) {
            this.x *= v.x; this.y *= v.y; this.z *= v.z;
            return this;
        }

        negateInPlace() {
            this.x = -this.x; this.y = -this.y; this.z = -this.z;
            return this;
        }

        // this = this x v. Caches BOTH operands - v may be this, and v x v must give zero.
        crossInPlace(v) {
            const x = this.x, y = this.y, z = this.z;
            const vx = v.x, vy = v.y, vz = v.z;
            this.x = y * vz - z * vy;
            this.y = z * vx - x * vz;
            this.z = x * vy - y * vx;
            return this;
        }

        // A unit vector perpendicular to v. Crosses with the cardinal axis v is LEAST aligned to - a
        // nearly-parallel axis gives a near-zero vector that normalises into noise.
        findOrthogonal(v) {
            const ax = Math.abs(v.x), ay = Math.abs(v.y), az = Math.abs(v.z);
            if (ax <= ay && ax <= az) { this.x = 0; this.y = -v.z; this.z = v.y; }
            else if (ay <= az) { this.x = -v.z; this.y = 0; this.z = v.x; }
            else { this.x = -v.y; this.y = v.x; this.z = 0; }
            return this.normalizeInPlace();
        }

        minInPlace(v) {
            if (v.x < this.x) this.x = v.x;
            if (v.y < this.y) this.y = v.y;
            if (v.z < this.z) this.z = v.z;
            return this;
        }

        maxInPlace(v) {
            if (v.x > this.x) this.x = v.x;
            if (v.y > this.y) this.y = v.y;
            if (v.z > this.z) this.z = v.z;
            return this;
        }

        isFinite() {
            return Number.isFinite(this.x) && Number.isFinite(this.y) && Number.isFinite(this.z);
        }

        isZero() {
            return this.x === 0 && this.y === 0 && this.z === 0;
        }

        // Exact equality - equals() above uses an epsilon, which is wrong for identity checks.
        equalsExact(v) {
            return this.x === v.x && this.y === v.y && this.z === v.z;
        }

        toString() {
            return '(' + this.x + ', ' + this.y + ', ' + this.z + ')';
        }
    }

    ActionPhysics.Vector3 = Vector3;

}
ActionPhysics.Vector3 = Vector3;


// ==== src/math/Matrix3.js ====
var Matrix3;
if (host.Matrix3) {
    Matrix3 = host.Matrix3;
} else {
    // Part of ActionMath - the shared math library. This file is pasted here VERBATIM from its source of
    // truth and must not be edited locally. Anything this engine needs is added to ActionMath first, then
    // arrives here through the same paste.
    /**
     * Matrix3 - 3x3 matrix with instance fields, allocation-free.
     *
     * The rotation/inertia type. Every operation writes into the receiver or an out parameter rather than
     * returning a new object, because callers run these in tight loops.
     *
     * Element naming is eRC: row R, column C.
     *
     *      | e00 e01 e02 |
     *      | e10 e11 e12 |
     *      | e20 e21 e22 |
     */
    Matrix3 = class Matrix3 {

        constructor() {
            this.e00 = 1; this.e01 = 0; this.e02 = 0;
            this.e10 = 0; this.e11 = 1; this.e12 = 0;
            this.e20 = 0; this.e21 = 0; this.e22 = 1;
        }

        identity() {
            this.e00 = 1; this.e01 = 0; this.e02 = 0;
            this.e10 = 0; this.e11 = 1; this.e12 = 0;
            this.e20 = 0; this.e21 = 0; this.e22 = 1;
            return this;
        }

        zero() {
            this.e00 = 0; this.e01 = 0; this.e02 = 0;
            this.e10 = 0; this.e11 = 0; this.e12 = 0;
            this.e20 = 0; this.e21 = 0; this.e22 = 0;
            return this;
        }

        copy(m) {
            this.e00 = m.e00; this.e01 = m.e01; this.e02 = m.e02;
            this.e10 = m.e10; this.e11 = m.e11; this.e12 = m.e12;
            this.e20 = m.e20; this.e21 = m.e21; this.e22 = m.e22;
            return this;
        }

        /** Diagonal matrix from a vector — how a principal-axis inertia tensor is built. */
        setDiagonal(v) {
            this.zero();
            this.e00 = v.x; this.e11 = v.y; this.e22 = v.z;
            return this;
        }

        /**
         * Rotation matrix equivalent to a unit quaternion. Assumes q is normalised; a non-unit
         * quaternion produces a matrix that also scales.
         */
        fromQuaternion(q) {
            const x = q.x, y = q.y, z = q.z, w = q.w;
            const x2 = x + x, y2 = y + y, z2 = z + z;
            const xx = x * x2, xy = x * y2, xz = x * z2;
            const yy = y * y2, yz = y * z2, zz = z * z2;
            const wx = w * x2, wy = w * y2, wz = w * z2;

            this.e00 = 1 - (yy + zz); this.e01 = xy - wz;       this.e02 = xz + wy;
            this.e10 = xy + wz;       this.e11 = 1 - (xx + zz); this.e12 = yz - wx;
            this.e20 = xz - wy;       this.e21 = yz + wx;       this.e22 = 1 - (xx + yy);
            return this;
        }

        /** this = transpose(m). For a rotation matrix this is also its inverse. */
        transposeInto(m) {
            const e01 = m.e01, e02 = m.e02, e12 = m.e12;
            this.e00 = m.e00; this.e01 = m.e10; this.e02 = m.e20;
            this.e10 = e01;   this.e11 = m.e11; this.e12 = m.e21;
            this.e20 = e02;   this.e21 = e12;   this.e22 = m.e22;
            return this;
        }

        transpose() {
            return this.transposeInto(this);
        }

        /** this = this * m */
        multiply(m) {
            return this.multiplyFrom(this, m);
        }

        /** this = a * b. Safe when `this` aliases either argument. */
        multiplyFrom(a, b) {
            const a00 = a.e00, a01 = a.e01, a02 = a.e02;
            const a10 = a.e10, a11 = a.e11, a12 = a.e12;
            const a20 = a.e20, a21 = a.e21, a22 = a.e22;
            const b00 = b.e00, b01 = b.e01, b02 = b.e02;
            const b10 = b.e10, b11 = b.e11, b12 = b.e12;
            const b20 = b.e20, b21 = b.e21, b22 = b.e22;

            this.e00 = a00 * b00 + a01 * b10 + a02 * b20;
            this.e01 = a00 * b01 + a01 * b11 + a02 * b21;
            this.e02 = a00 * b02 + a01 * b12 + a02 * b22;
            this.e10 = a10 * b00 + a11 * b10 + a12 * b20;
            this.e11 = a10 * b01 + a11 * b11 + a12 * b21;
            this.e12 = a10 * b02 + a11 * b12 + a12 * b22;
            this.e20 = a20 * b00 + a21 * b10 + a22 * b20;
            this.e21 = a20 * b01 + a21 * b11 + a22 * b21;
            this.e22 = a20 * b02 + a21 * b12 + a22 * b22;
            return this;
        }

        determinant() {
            return this.e00 * (this.e11 * this.e22 - this.e12 * this.e21)
                 - this.e01 * (this.e10 * this.e22 - this.e12 * this.e20)
                 + this.e02 * (this.e10 * this.e21 - this.e11 * this.e20);
        }

        /**
         * this = inverse(m). Returns false and leaves `this` as identity if m is singular.
         *
         * Singular is a real, reachable case: a body with zero inertia about some axis (an
         * infinitely thin shape, or a degenerate mesh) produces one. Reporting it lets the
         * caller decide, rather than propagating Infinity into every subsequent computation.
         */
        invertInto(m) {
            const c00 = m.e11 * m.e22 - m.e12 * m.e21;
            const c01 = m.e12 * m.e20 - m.e10 * m.e22;
            const c02 = m.e10 * m.e21 - m.e11 * m.e20;
            const det = m.e00 * c00 + m.e01 * c01 + m.e02 * c02;

            if (det === 0) {
                this.identity();
                return false;
            }

            const inv = 1 / det;
            const e00 = m.e00, e01 = m.e01, e02 = m.e02;
            const e10 = m.e10, e11 = m.e11, e12 = m.e12;
            const e20 = m.e20, e21 = m.e21, e22 = m.e22;

            this.e00 = c00 * inv;
            this.e01 = (e02 * e21 - e01 * e22) * inv;
            this.e02 = (e01 * e12 - e02 * e11) * inv;
            this.e10 = c01 * inv;
            this.e11 = (e00 * e22 - e02 * e20) * inv;
            this.e12 = (e02 * e10 - e00 * e12) * inv;
            this.e20 = c02 * inv;
            this.e21 = (e01 * e20 - e00 * e21) * inv;
            this.e22 = (e00 * e11 - e01 * e10) * inv;
            return true;
        }

        invert() {
            return this.invertInto(this);
        }

        /** Rotate v in place by this matrix. */
        transformVector3(v) {
            const x = v.x, y = v.y, z = v.z;
            v.x = this.e00 * x + this.e01 * y + this.e02 * z;
            v.y = this.e10 * x + this.e11 * y + this.e12 * z;
            v.z = this.e20 * x + this.e21 * y + this.e22 * z;
            return v;
        }

        transformVector3Into(v, out) {
            const x = v.x, y = v.y, z = v.z;
            out.x = this.e00 * x + this.e01 * y + this.e02 * z;
            out.y = this.e10 * x + this.e11 * y + this.e12 * z;
            out.z = this.e20 * x + this.e21 * y + this.e22 * z;
            return out;
        }

        isFinite() {
            return Number.isFinite(this.e00) && Number.isFinite(this.e01) && Number.isFinite(this.e02) &&
                   Number.isFinite(this.e10) && Number.isFinite(this.e11) && Number.isFinite(this.e12) &&
                   Number.isFinite(this.e20) && Number.isFinite(this.e21) && Number.isFinite(this.e22);
        }
    }

    ActionPhysics.Matrix3 = Matrix3;

}
ActionPhysics.Matrix3 = Matrix3;


// ==== src/math/Matrix4.js ====
var Matrix4;
if (host.Matrix4) {
    Matrix4 = host.Matrix4;
} else {
    // Part of ActionMath - the shared math library. This file is pasted here VERBATIM from its source of
    // truth and must not be edited locally. Anything this engine needs is added to ActionMath first, then
    // arrives here through the same paste.
    Matrix4 = class Matrix4 {
        // Float32Array: what a GPU wants, and what rendering should use.
        static create() {
            return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
        }

        // Float64Array, for callers that cannot afford 32-bit rounding.
        //
        // float32 carries ~7 significant digits, so a position in the tens resolves to about 1e-5 - coarser
        // than the quantities a physics solver works in (contact depths around 1e-3, and corrections an
        // order of magnitude below that). It also defeats reproducibility, since the rounding compounds
        // through every transform.
        //
        // Every static below is written as plain indexed reads and writes, so all of them operate on either
        // array type without change.
        static createPrecise() {
            return new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
        }

        static identity(out) {
            out[0] = 1;
            out[1] = 0;
            out[2] = 0;
            out[3] = 0;
            out[4] = 0;
            out[5] = 1;
            out[6] = 0;
            out[7] = 0;
            out[8] = 0;
            out[9] = 0;
            out[10] = 1;
            out[11] = 0;
            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            out[15] = 1;
            return out;
        }
        /**
         * Multiply a vector by a matrix
         * @param {Array} out - Output vector (will be modified)
         * @param {Array|Float32Array} matrix - 4x4 matrix
         * @param {Array} vec - Input vector [x, y, z, w]
         * @returns {Array} - The output vector
         */
        static multiplyVector(out, matrix, vec) {
            const x = vec[0];
            const y = vec[1];
            const z = vec[2];
            const w = vec[3] || 1.0;

            out[0] = matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12] * w;
            out[1] = matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13] * w;
            out[2] = matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14] * w;
            out[3] = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15] * w;

            return out;
        }

        static transformVector(vector, viewMatrix, projectionMatrix) {
            // First multiply by view matrix
            const viewResult = [0, 0, 0, 0];
            for (let i = 0; i < 4; i++) {
                viewResult[i] =
                    vector[0] * viewMatrix[i] +
                    vector[1] * viewMatrix[i + 4] +
                    vector[2] * viewMatrix[i + 8] +
                    vector[3] * viewMatrix[i + 12];
            }

            // Then multiply by projection matrix
            const result = [0, 0, 0, 0];
            for (let i = 0; i < 4; i++) {
                result[i] =
                    viewResult[0] * projectionMatrix[i] +
                    viewResult[1] * projectionMatrix[i + 4] +
                    viewResult[2] * projectionMatrix[i + 8] +
                    viewResult[3] * projectionMatrix[i + 12];
            }

            return result;
        }

        // In-place version that writes to output array (no allocation)
        static transformVectorInto(vector, viewMatrix, projectionMatrix, out) {
            // First multiply by view matrix
            const viewResultX =
                vector[0] * viewMatrix[0] +
                vector[1] * viewMatrix[4] +
                vector[2] * viewMatrix[8] +
                vector[3] * viewMatrix[12];
            const viewResultY =
                vector[0] * viewMatrix[1] +
                vector[1] * viewMatrix[5] +
                vector[2] * viewMatrix[9] +
                vector[3] * viewMatrix[13];
            const viewResultZ =
                vector[0] * viewMatrix[2] +
                vector[1] * viewMatrix[6] +
                vector[2] * viewMatrix[10] +
                vector[3] * viewMatrix[14];
            const viewResultW =
                vector[0] * viewMatrix[3] +
                vector[1] * viewMatrix[7] +
                vector[2] * viewMatrix[11] +
                vector[3] * viewMatrix[15];

            // Then multiply by projection matrix
            out[0] =
                viewResultX * projectionMatrix[0] +
                viewResultY * projectionMatrix[4] +
                viewResultZ * projectionMatrix[8] +
                viewResultW * projectionMatrix[12];
            out[1] =
                viewResultX * projectionMatrix[1] +
                viewResultY * projectionMatrix[5] +
                viewResultZ * projectionMatrix[9] +
                viewResultW * projectionMatrix[13];
            out[2] =
                viewResultX * projectionMatrix[2] +
                viewResultY * projectionMatrix[6] +
                viewResultZ * projectionMatrix[10] +
                viewResultW * projectionMatrix[14];
            out[3] =
                viewResultX * projectionMatrix[3] +
                viewResultY * projectionMatrix[7] +
                viewResultZ * projectionMatrix[11] +
                viewResultW * projectionMatrix[15];
        }
        static fromQuat(out, q) {
            const x = q.x,
                y = q.y,
                z = q.z,
                w = q.w;
            const x2 = x + x,
                y2 = y + y,
                z2 = z + z;
            const xx = x * x2,
                xy = x * y2,
                xz = x * z2;
            const yy = y * y2,
                yz = y * z2,
                zz = z * z2;
            const wx = w * x2,
                wy = w * y2,
                wz = w * z2;

            out[0] = 1 - (yy + zz);
            out[1] = xy + wz;
            out[2] = xz - wy;
            out[3] = 0;

            out[4] = xy - wz;
            out[5] = 1 - (xx + zz);
            out[6] = yz + wx;
            out[7] = 0;

            out[8] = xz + wy;
            out[9] = yz - wx;
            out[10] = 1 - (xx + yy);
            out[11] = 0;

            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            out[15] = 1;

            return out;
        }

        static fromLightDirection(out, dir) {
            // Make sure the direction is normalized
            const nx = dir.x;
            const ny = dir.y;
            const nz = dir.z;

            // Find a perpendicular vector for the "right" direction
            // Using world-up (0,1,0) as a reference
            const right = [
                nz, // Cross product of dir with (0,1,0)
                0,
                -nx
            ];

            // Normalize right vector
            const rLength = Math.sqrt(right[0] * right[0] + right[2] * right[2]);
            right[0] /= rLength;
            right[2] /= rLength;

            // Get up vector by crossing right with direction
            const up = [
                -nx * ny, // Cross product of right with dir
                nx * nx + nz * nz,
                -ny * nz
            ];

            // Build the view matrix
            out[0] = right[0];
            out[1] = up[0];
            out[2] = nx;
            out[3] = 0;

            out[4] = right[1];
            out[5] = up[1];
            out[6] = ny;
            out[7] = 0;

            out[8] = right[2];
            out[9] = up[2];
            out[10] = nz;
            out[11] = 0;

            out[12] = 0;
            out[13] = 0;
            out[14] = 0;
            out[15] = 1;

            return out;
        }
        static copy(out, a) {
            out[0] = a[0];
            out[1] = a[1];
            out[2] = a[2];
            out[3] = a[3];
            out[4] = a[4];
            out[5] = a[5];
            out[6] = a[6];
            out[7] = a[7];
            out[8] = a[8];
            out[9] = a[9];
            out[10] = a[10];
            out[11] = a[11];
            out[12] = a[12];
            out[13] = a[13];
            out[14] = a[14];
            out[15] = a[15];
            return out;
        }

        static multiply(out, a, b) {
            const a00 = a[0],
                a01 = a[1],
                a02 = a[2],
                a03 = a[3];
            const a10 = a[4],
                a11 = a[5],
                a12 = a[6],
                a13 = a[7];
            const a20 = a[8],
                a21 = a[9],
                a22 = a[10],
                a23 = a[11];
            const a30 = a[12],
                a31 = a[13],
                a32 = a[14],
                a33 = a[15];

            let b0 = b[0],
                b1 = b[1],
                b2 = b[2],
                b3 = b[3];
            out[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
            out[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
            out[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
            out[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

            b0 = b[4];
            b1 = b[5];
            b2 = b[6];
            b3 = b[7];
            out[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
            out[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
            out[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
            out[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

            b0 = b[8];
            b1 = b[9];
            b2 = b[10];
            b3 = b[11];
            out[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
            out[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
            out[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
            out[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

            b0 = b[12];
            b1 = b[13];
            b2 = b[14];
            b3 = b[15];
            out[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
            out[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
            out[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
            out[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
            return out;
        }
        static fromRotationTranslation(out, q, v) {
            // Similar to his code but using our Quaternion class
            const x = q.x,
                y = q.y,
                z = q.z,
                w = q.w;
            const x2 = x + x;
            const y2 = y + y;
            const z2 = z + z;

            const xx = x * x2;
            const xy = x * y2;
            const xz = x * z2;
            const yy = y * y2;
            const yz = y * z2;
            const zz = z * z2;
            const wx = w * x2;
            const wy = w * y2;
            const wz = w * z2;

            out[0] = 1 - (yy + zz);
            out[1] = xy + wz;
            out[2] = xz - wy;
            out[3] = 0;
            out[4] = xy - wz;
            out[5] = 1 - (xx + zz);
            out[6] = yz + wx;
            out[7] = 0;
            out[8] = xz + wy;
            out[9] = yz - wx;
            out[10] = 1 - (xx + yy);
            out[11] = 0;
            out[12] = v.x;
            out[13] = v.y;
            out[14] = v.z;
            out[15] = 1;
            return out;
        }

        static transformVertex(vertex, modelMatrix) {
            const v = [vertex.x, vertex.y, vertex.z, 1];
            const result = [0, 0, 0, 0];

            for (let i = 0; i < 4; i++) {
                result[i] =
                    v[0] * modelMatrix[i] +
                    v[1] * modelMatrix[i + 4] +
                    v[2] * modelMatrix[i + 8] +
                    v[3] * modelMatrix[i + 12];
            }

            return new Vector3(result[0] / result[3], result[1] / result[3], result[2] / result[3]);
        }

        static transformNormal(normal, modelMatrix) {
            // Calculate inverse transpose of 3x3 portion of model matrix
            const a = modelMatrix[0],
                b = modelMatrix[1],
                c = modelMatrix[2],
                d = modelMatrix[4],
                e = modelMatrix[5],
                f = modelMatrix[6],
                g = modelMatrix[8],
                h = modelMatrix[9],
                i = modelMatrix[10];

            const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
            const invdet = 1.0 / det;

            const invTranspose = [
                (e * i - f * h) * invdet,
                (c * h - b * i) * invdet,
                (b * f - c * e) * invdet,
                (f * g - d * i) * invdet,
                (a * i - c * g) * invdet,
                (c * d - a * f) * invdet,
                (d * h - e * g) * invdet,
                (b * g - a * h) * invdet,
                (a * e - b * d) * invdet
            ];

            const x = normal.x * invTranspose[0] + normal.y * invTranspose[1] + normal.z * invTranspose[2];
            const y = normal.x * invTranspose[3] + normal.y * invTranspose[4] + normal.z * invTranspose[5];
            const z = normal.x * invTranspose[6] + normal.y * invTranspose[7] + normal.z * invTranspose[8];

            return new Vector3(x, y, z).normalize();
        }
        static perspective(out, fovy, aspect, near, far) {
            const f = 1.0 / Scalar.tan(fovy / 2);
            out[0] = f / aspect;
            out[1] = 0;
            out[2] = 0;
            out[3] = 0;
            out[4] = 0;
            out[5] = f;
            out[6] = 0;
            out[7] = 0;
            out[8] = 0;
            out[9] = 0;
            out[10] = (far + near) / (near - far);
            out[11] = -1;
            out[12] = 0;
            out[13] = 0;
            out[14] = (2 * far * near) / (near - far);
            out[15] = 0;
            return out;
        }

        static ortho(out, left, right, bottom, top, near, far) {
            const lr = 1 / (left - right);
            const bt = 1 / (bottom - top);
            const nf = 1 / (near - far);
            out[0] = -2 * lr;
            out[1] = 0;
            out[2] = 0;
            out[3] = 0;
            out[4] = 0;
            out[5] = -2 * bt;
            out[6] = 0;
            out[7] = 0;
            out[8] = 0;
            out[9] = 0;
            out[10] = 2 * nf;
            out[11] = 0;
            out[12] = (left + right) * lr;
            out[13] = (top + bottom) * bt;
            out[14] = (far + near) * nf;
            out[15] = 1;
            return out;
        }

        static translate(out, a, v) {
            const x = v[0],
                y = v[1],
                z = v[2];

            if (a !== out) {
                out[0] = a[0];
                out[1] = a[1];
                out[2] = a[2];
                out[3] = a[3];
                out[4] = a[4];
                out[5] = a[5];
                out[6] = a[6];
                out[7] = a[7];
                out[8] = a[8];
                out[9] = a[9];
                out[10] = a[10];
                out[11] = a[11];
            }

            out[12] = a[0] * x + a[4] * y + a[8] * z + a[12];
            out[13] = a[1] * x + a[5] * y + a[9] * z + a[13];
            out[14] = a[2] * x + a[6] * y + a[10] * z + a[14];
            out[15] = a[3] * x + a[7] * y + a[11] * z + a[15];

            return out;
        }

        static rotate(out, a, rad, axis) {
            let x = axis[0],
                y = axis[1],
                z = axis[2];

            let len = Scalar.hypot3(x, y, z);
            if (len < 0.000001) {
                return null;
            }
            len = 1 / len;
            x *= len;
            y *= len;
            z *= len;

            const s = Scalar.sin(rad);
            const c = Scalar.cos(rad);
            const t = 1 - c;

            const a00 = a[0],
                a01 = a[1],
                a02 = a[2],
                a03 = a[3];
            const a10 = a[4],
                a11 = a[5],
                a12 = a[6],
                a13 = a[7];
            const a20 = a[8],
                a21 = a[9],
                a22 = a[10],
                a23 = a[11];

            const b00 = x * x * t + c;
            const b01 = y * x * t + z * s;
            const b02 = z * x * t - y * s;
            const b10 = x * y * t - z * s;
            const b11 = y * y * t + c;
            const b12 = z * y * t + x * s;
            const b20 = x * z * t + y * s;
            const b21 = y * z * t - x * s;
            const b22 = z * z * t + c;

            out[0] = a00 * b00 + a10 * b01 + a20 * b02;
            out[1] = a01 * b00 + a11 * b01 + a21 * b02;
            out[2] = a02 * b00 + a12 * b01 + a22 * b02;
            out[3] = a03 * b00 + a13 * b01 + a23 * b02;
            out[4] = a00 * b10 + a10 * b11 + a20 * b12;
            out[5] = a01 * b10 + a11 * b11 + a21 * b12;
            out[6] = a02 * b10 + a12 * b11 + a22 * b12;
            out[7] = a03 * b10 + a13 * b11 + a23 * b12;
            out[8] = a00 * b20 + a10 * b21 + a20 * b22;
            out[9] = a01 * b20 + a11 * b21 + a21 * b22;
            out[10] = a02 * b20 + a12 * b21 + a22 * b22;
            out[11] = a03 * b20 + a13 * b21 + a23 * b22;

            if (a !== out) {
                out[12] = a[12];
                out[13] = a[13];
                out[14] = a[14];
                out[15] = a[15];
            }
            return out;
        }

        static scale(out, a, v) {
            const x = v[0],
                y = v[1],
                z = v[2];
            out[0] = a[0] * x;
            out[1] = a[1] * x;
            out[2] = a[2] * x;
            out[3] = a[3] * x;
            out[4] = a[4] * y;
            out[5] = a[5] * y;
            out[6] = a[6] * y;
            out[7] = a[7] * y;
            out[8] = a[8] * z;
            out[9] = a[9] * z;
            out[10] = a[10] * z;
            out[11] = a[11] * z;
            out[12] = a[12];
            out[13] = a[13];
            out[14] = a[14];
            out[15] = a[15];
            return out;
        }

        static invert(out, a) {
            const a00 = a[0],
                a01 = a[1],
                a02 = a[2],
                a03 = a[3];
            const a10 = a[4],
                a11 = a[5],
                a12 = a[6],
                a13 = a[7];
            const a20 = a[8],
                a21 = a[9],
                a22 = a[10],
                a23 = a[11];
            const a30 = a[12],
                a31 = a[13],
                a32 = a[14],
                a33 = a[15];

            const b00 = a00 * a11 - a01 * a10;
            const b01 = a00 * a12 - a02 * a10;
            const b02 = a00 * a13 - a03 * a10;
            const b03 = a01 * a12 - a02 * a11;
            const b04 = a01 * a13 - a03 * a11;
            const b05 = a02 * a13 - a03 * a12;
            const b06 = a20 * a31 - a21 * a30;
            const b07 = a20 * a32 - a22 * a30;
            const b08 = a20 * a33 - a23 * a30;
            const b09 = a21 * a32 - a22 * a31;
            const b10 = a21 * a33 - a23 * a31;
            const b11 = a22 * a33 - a23 * a32;

            let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
            if (!det) {
                return null;
            }
            det = 1.0 / det;

            out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
            out[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
            out[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
            out[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
            out[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
            out[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
            out[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
            out[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
            out[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
            out[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
            out[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
            out[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
            out[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
            out[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
            out[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
            out[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;

            return out;
        }

        static lookAt(out, eye, center, up) {
            let x0, x1, x2, y0, y1, y2, z0, z1, z2, len;
            const eyex = eye[0];
            const eyey = eye[1];
            const eyez = eye[2];
            const upx = up[0];
            const upy = up[1];
            const upz = up[2];
            const centerx = center[0];
            const centery = center[1];
            const centerz = center[2];

            if (
                Math.abs(eyex - centerx) < 0.000001 &&
                Math.abs(eyey - centery) < 0.000001 &&
                Math.abs(eyez - centerz) < 0.000001
            ) {
                return Matrix4.identity(out);
            }

            z0 = eyex - centerx;
            z1 = eyey - centery;
            z2 = eyez - centerz;

            len = 1 / Scalar.hypot3(z0, z1, z2);
            z0 *= len;
            z1 *= len;
            z2 *= len;

            // Cross product of up and z
            x0 = upy * z2 - upz * z1;
            x1 = upz * z0 - upx * z2;
            x2 = upx * z1 - upy * z0;
            len = Scalar.hypot3(x0, x1, x2);

            // Handle the case where up and z are colinear (or nearly so)
            if (len < 0.000001) {
                // Find a perpendicular vector to z
                // Try cross product with (1,0,0) first
                if (Math.abs(z0) < 0.9) {
                    // Cross with X axis
                    x0 = 0;
                    x1 = z2;
                    x2 = -z1;
                } else {
                    // Cross with Z axis if Z is near X
                    x0 = z1;
                    x1 = -z0;
                    x2 = 0;
                }
                len = Scalar.hypot3(x0, x1, x2);
                len = 1 / len;
                x0 *= len;
                x1 *= len;
                x2 *= len;
            } else {
                // Normal case - normalize the computed cross product
                len = 1 / len;
                x0 *= len;
                x1 *= len;
                x2 *= len;
            }

            y0 = z1 * x2 - z2 * x1;
            y1 = z2 * x0 - z0 * x2;
            y2 = z0 * x1 - z1 * x0;

            len = Scalar.hypot3(y0, y1, y2);
            if (!len) {
                y0 = 0;
                y1 = 0;
                y2 = 0;
            } else {
                len = 1 / len;
                y0 *= len;
                y1 *= len;
                y2 *= len;
            }

            out[0] = x0;
            out[1] = y0;
            out[2] = z0;
            out[3] = 0;
            out[4] = x1;
            out[5] = y1;
            out[6] = z1;
            out[7] = 0;
            out[8] = x2;
            out[9] = y2;
            out[10] = z2;
            out[11] = 0;
            out[12] = -(x0 * eyex + x1 * eyey + x2 * eyez);
            out[13] = -(y0 * eyex + y1 * eyey + y2 * eyez);
            out[14] = -(z0 * eyex + z1 * eyey + z2 * eyez);
            out[15] = 1;

            return out;
        }

        static transpose(out, a) {
            if (out === a) {
                const a01 = a[1],
                    a02 = a[2],
                    a03 = a[3],
                    a12 = a[6],
                    a13 = a[7],
                    a23 = a[11];

                out[1] = a[4];
                out[2] = a[8];
                out[3] = a[12];
                out[4] = a01;
                out[6] = a[9];
                out[7] = a[13];
                out[8] = a02;
                out[9] = a12;
                out[11] = a[14];
                out[12] = a03;
                out[13] = a13;
                out[14] = a23;
            } else {
                out[0] = a[0];
                out[1] = a[4];
                out[2] = a[8];
                out[3] = a[12];
                out[4] = a[1];
                out[5] = a[5];
                out[6] = a[9];
                out[7] = a[13];
                out[8] = a[2];
                out[9] = a[6];
                out[10] = a[10];
                out[11] = a[14];
                out[12] = a[3];
                out[13] = a[7];
                out[14] = a[11];
                out[15] = a[15];
            }

            return out;
        }

        static rotateX(out, a, rad) {
            const s = Scalar.sin(rad);
            const c = Scalar.cos(rad);
            const a10 = a[4];
            const a11 = a[5];
            const a12 = a[6];
            const a13 = a[7];
            const a20 = a[8];
            const a21 = a[9];
            const a22 = a[10];
            const a23 = a[11];

            out[4] = a10 * c + a20 * s;
            out[5] = a11 * c + a21 * s;
            out[6] = a12 * c + a22 * s;
            out[7] = a13 * c + a23 * s;
            out[8] = a20 * c - a10 * s;
            out[9] = a21 * c - a11 * s;
            out[10] = a22 * c - a12 * s;
            out[11] = a23 * c - a13 * s;

            // If the source and destination differ, we need to copy the unchanged rows
            if (a !== out) {
                out[0] = a[0];
                out[1] = a[1];
                out[2] = a[2];
                out[3] = a[3];
                out[12] = a[12];
                out[13] = a[13];
                out[14] = a[14];
                out[15] = a[15];
            }

            return out;
        }
        static rotateZ(out, a, rad) {
            const s = Scalar.sin(rad);
            const c = Scalar.cos(rad);
            const a00 = a[0];
            const a01 = a[1];
            const a02 = a[2];
            const a03 = a[3];
            const a10 = a[4];
            const a11 = a[5];
            const a12 = a[6];
            const a13 = a[7];

            out[0] = a00 * c + a10 * s;
            out[1] = a01 * c + a11 * s;
            out[2] = a02 * c + a12 * s;
            out[3] = a03 * c + a13 * s;
            out[4] = a10 * c - a00 * s;
            out[5] = a11 * c - a01 * s;
            out[6] = a12 * c - a02 * s;
            out[7] = a13 * c - a03 * s;

            // If the source and destination differ, we need to copy the unchanged rows
            if (a !== out) {
                out[8] = a[8];
                out[9] = a[9];
                out[10] = a[10];
                out[11] = a[11];
                out[12] = a[12];
                out[13] = a[13];
                out[14] = a[14];
                out[15] = a[15];
            }

            return out;
        }

        static rotateY(out, a, rad) {
            const s = Scalar.sin(rad);
            const c = Scalar.cos(rad);
            const a00 = a[0];
            const a01 = a[1];
            const a02 = a[2];
            const a03 = a[3];
            const a20 = a[8];
            const a21 = a[9];
            const a22 = a[10];
            const a23 = a[11];

            out[0] = a00 * c - a20 * s;
            out[1] = a01 * c - a21 * s;
            out[2] = a02 * c - a22 * s;
            out[3] = a03 * c - a23 * s;
            out[8] = a00 * s + a20 * c;
            out[9] = a01 * s + a21 * c;
            out[10] = a02 * s + a22 * c;
            out[11] = a03 * s + a23 * c;

            // If the source and destination differ, we need to copy the unchanged rows
            if (a !== out) {
                out[4] = a[4];
                out[5] = a[5];
                out[6] = a[6];
                out[7] = a[7];
                out[12] = a[12];
                out[13] = a[13];
                out[14] = a[14];
                out[15] = a[15];
            }

            return out;
        }
    }

    ActionPhysics.Matrix4 = Matrix4;

}
ActionPhysics.Matrix4 = Matrix4;


// ==== src/math/Quaternion.js ====
var Quaternion;
if (host.Quaternion) {
    Quaternion = host.Quaternion;
} else {
    // Part of ActionMath - the shared math library. This file is pasted here VERBATIM from its source of
    // truth and must not be edited locally. Anything this engine needs is added to ActionMath first, then
    // arrives here through the same paste.
    Quaternion = class Quaternion {
        constructor(x = 0, y = 0, z = 0, w = 1) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        // Normalises the axis, for the same reason as setAxisAngle below.
        static fromAxisAngle(axis, angle) {
            return new Quaternion().setAxisAngle(axis, angle);
        }

        static fromEulerY(yAngle) {
            const halfAngle = yAngle * 0.5;
            return new Quaternion(0, Scalar.sin(halfAngle), 0, Scalar.cos(halfAngle));
        }

        static fromEuler(roll, pitch, yaw) {
            // Convert euler angles to quaternion
            // Rotation order: roll (Z-axis) -> pitch (X-axis) -> yaw (Y-axis)
            // This matches the old Arwing.transformVertex() rotation order
            const cr = Scalar.cos(roll * 0.5);
            const sr = Scalar.sin(roll * 0.5);
            const cp = Scalar.cos(pitch * 0.5);
            const sp = Scalar.sin(pitch * 0.5);
            const cy = Scalar.cos(yaw * 0.5);
            const sy = Scalar.sin(yaw * 0.5);

            // ZXY order: Qz * Qx * Qy
            const w = cy * cp * cr + sy * sp * sr;
            const x = cy * sp * cr + sy * cp * sr;
            const y = sy * cp * cr - cy * sp * sr;
            const z = cy * cp * sr - sy * sp * cr;

            return new Quaternion(x, y, z, w);
        }

        // Orientation that points a +Z-forward object (mesh, projectile) along a direction vector.
        // Uses the same Euler convention as fromEuler(0, -pitch, yaw), so a +Z model faces exactly
        // where it's heading. Returns identity for a near-zero vector.
        static fromDirection(vx, vy, vz) {
            const sp = Scalar.hypot3(vx, vy, vz);
            if (sp < 1e-6) return new Quaternion(0, 0, 0, 1);
            const yaw = Scalar.atan2(vx, vz);
            const pitch = Scalar.asin(Math.max(-1, Math.min(1, vy / sp)));
            return Quaternion.fromEuler(0, -pitch, yaw);
        }

        // Hamilton product a∘b (applies b first, then a). Composes two rotations into one.
        static multiply(a, b) {
            return new Quaternion(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        // The axis is normalised here, so a non-unit axis still yields a unit quaternion. Skipping that
        // scales the whole quaternion by |axis|, and a non-unit quaternion silently scales every vector it
        // rotates. A zero axis gives identity rather than NaN.
        setAxisAngle(axis, angle) {
            const lsq = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
            if (lsq === 0) return this.identity();
            const inv = 1 / Math.sqrt(lsq);
            const halfAngle = angle * 0.5;
            const s = Scalar.sin(halfAngle) * inv;
            this.x = axis.x * s;
            this.y = axis.y * s;
            this.z = axis.z * s;
            this.w = Scalar.cos(halfAngle);
            return this;
        }

        setFromEuler(roll, pitch, yaw) {
            // Convert euler angles to quaternion
            // Rotation order: roll (Z-axis) -> pitch (X-axis) -> yaw (Y-axis)
            // This matches the old Arwing.transformVertex() rotation order
            const cr = Scalar.cos(roll * 0.5);
            const sr = Scalar.sin(roll * 0.5);
            const cp = Scalar.cos(pitch * 0.5);
            const sp = Scalar.sin(pitch * 0.5);
            const cy = Scalar.cos(yaw * 0.5);
            const sy = Scalar.sin(yaw * 0.5);

            // ZXY order: Qz * Qx * Qy
            this.w = cy * cp * cr + sy * sp * sr;
            this.x = cy * sp * cr + sy * cp * sr;
            this.y = sy * cp * cr - cy * sp * sr;
            this.z = cy * cp * sr - sy * sp * cr;
            return this;
        }

        slerp(other, t) {
            let cosHalfTheta = this.x * other.x + this.y * other.y + this.z * other.z + this.w * other.w;

            if (Math.abs(cosHalfTheta) >= 1.0) {
                return this;
            }

            const halfTheta = Scalar.acos(cosHalfTheta);
            const sinHalfTheta = Math.sqrt(1.0 - cosHalfTheta * cosHalfTheta);

            if (Math.abs(sinHalfTheta) < 0.001) {
                return new Quaternion(
                    this.x * 0.5 + other.x * 0.5,
                    this.y * 0.5 + other.y * 0.5,
                    this.z * 0.5 + other.z * 0.5,
                    this.w * 0.5 + other.w * 0.5
                );
            }

            const ratioA = Scalar.sin((1 - t) * halfTheta) / sinHalfTheta;
            const ratioB = Scalar.sin(t * halfTheta) / sinHalfTheta;

            return new Quaternion(
                this.x * ratioA + other.x * ratioB,
                this.y * ratioA + other.y * ratioB,
                this.z * ratioA + other.z * ratioB,
                this.w * ratioA + other.w * ratioB
            );
        }

        /**
         * Transform a vector by this quaternion rotation
         * Uses the formula: v' = q * v * q^-1
         * @param {Vector3} vector - The vector to rotate
         * @returns {Vector3} The rotated vector
         */
        transformVector(vector) {
            const x = vector.x,
                y = vector.y,
                z = vector.z;
            const qx = this.x,
                qy = this.y,
                qz = this.z,
                qw = this.w;

            // Calculate q * v
            const ix = qw * x + qy * z - qz * y;
            const iy = qw * y + qz * x - qx * z;
            const iz = qw * z + qx * y - qy * x;
            const iw = -qx * x - qy * y - qz * z;

            // Calculate (q * v) * q^-1
            const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
            const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
            const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

            return new Vector3(rx, ry, rz);
        }

        // ---- in-place / allocation-free forms ----
        // The physics solver runs these thousands of times per tick and cannot allocate per operation.
        // The allocating forms above are unchanged.

        set(x, y, z, w) {
            this.x = x; this.y = y; this.z = z; this.w = w;
            return this;
        }

        copy(q) {
            this.x = q.x; this.y = q.y; this.z = q.z; this.w = q.w;
            return this;
        }

        clone() {
            return new Quaternion(this.x, this.y, this.z, this.w);
        }

        identity() {
            return this.set(0, 0, 0, 1);
        }

        // this = this * q
        multiplyInPlace(q) {
            return this.multiplyQuaternions(this, q);
        }

        // this = a * b. Safe when this aliases either argument.
        multiplyQuaternions(a, b) {
            const ax = a.x, ay = a.y, az = a.z, aw = a.w;
            const bx = b.x, by = b.y, bz = b.z, bw = b.w;
            this.x = aw * bx + ax * bw + ay * bz - az * by;
            this.y = aw * by - ax * bz + ay * bw + az * bx;
            this.z = aw * bz + ax * by - ay * bx + az * bw;
            this.w = aw * bw - ax * bx - ay * by - az * bz;
            return this;
        }

        // Rescale to unit length. A rotation built by repeated multiplication drifts off the unit sphere,
        // and a non-unit quaternion silently SCALES every vector it rotates - the object appears to grow or
        // shrink. slerp() also assumes unit length: its acos(dot) is only the half-angle if both are unit,
        // which is what the >= 1.0 guard there is really working around.
        // A zero quaternion becomes identity rather than NaN.
        normalize() {
            const lsq = this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w;
            if (lsq === 0) return this.identity();
            const inv = 1 / Math.sqrt(lsq);
            this.x *= inv; this.y *= inv; this.z *= inv; this.w *= inv;
            return this;
        }

        // this = inverse of q. Conjugate only - assumes unit length.
        invertQuaternion(q) {
            this.x = -q.x; this.y = -q.y; this.z = -q.z; this.w = q.w;
            return this;
        }

        invert() {
            return this.invertQuaternion(this);
        }

        dot(q) {
            return this.x * q.x + this.y * q.y + this.z * q.z + this.w * q.w;
        }

        lengthSquared() {
            return this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w;
        }

        length() {
            return Math.sqrt(this.lengthSquared());
        }

        // Rotate `vector` IN PLACE. Cross-product form - no temporary quaternions, no allocation.
        transformVectorInPlace(vector) {
            const qx = this.x, qy = this.y, qz = this.z, qw = this.w;
            const vx = vector.x, vy = vector.y, vz = vector.z;

            const tx = 2 * (qy * vz - qz * vy);
            const ty = 2 * (qz * vx - qx * vz);
            const tz = 2 * (qx * vy - qy * vx);

            vector.x = vx + qw * tx + (qy * tz - qz * ty);
            vector.y = vy + qw * ty + (qz * tx - qx * tz);
            vector.z = vz + qw * tz + (qx * ty - qy * tx);
            return vector;
        }

        // out = this applied to vector, leaving vector untouched.
        transformVectorInto(vector, out) {
            out.x = vector.x; out.y = vector.y; out.z = vector.z;
            return this.transformVectorInPlace(out);
        }

        // Unsigned angle to q, in [0, PI]. |dot| handles double cover: q and -q are the same rotation.
        angleBetween(q) {
            let d = Math.abs(this.dot(q));
            if (d > 1) d = 1;
            return 2 * Scalar.acos(d);
        }

        // Signed angle about `axis`, in [-PI, PI]. Joint limits need direction, not just magnitude.
        signedAngleBetween(q, axis) {
            const ix = -this.x, iy = -this.y, iz = -this.z, iw = this.w;
            const dx = q.w * ix + q.x * iw + q.y * iz - q.z * iy;
            const dy = q.w * iy - q.x * iz + q.y * iw + q.z * ix;
            const dz = q.w * iz + q.x * iy - q.y * ix + q.z * iw;
            const dw = q.w * iw - q.x * ix - q.y * iy - q.z * iz;

            const vecLen = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (vecLen === 0) return 0;
            const angle = 2 * Scalar.atan2(vecLen, dw);
            const sign = (dx * axis.x + dy * axis.y + dz * axis.z) >= 0 ? 1 : -1;
            const TWO_PI = 6.283185307179586;
            return sign * (angle > Math.PI ? angle - TWO_PI : angle);
        }

        isFinite() {
            return Number.isFinite(this.x) && Number.isFinite(this.y) &&
                   Number.isFinite(this.z) && Number.isFinite(this.w);
        }

        equals(q) {
            return this.x === q.x && this.y === q.y && this.z === q.z && this.w === q.w;
        }

        toString() {
            return '(' + this.x + ', ' + this.y + ', ' + this.z + ', ' + this.w + ')';
        }
    }

    ActionPhysics.Quaternion = Quaternion;

}
ActionPhysics.Quaternion = Quaternion;


// ==== src/math/Transform.js ====
var Transform;
if (host.Transform) {
    Transform = host.Transform;
} else {
    // Part of ActionMath - the shared math library. This file is pasted here VERBATIM from its source of
    // truth and must not be edited locally. Anything this engine needs is added to ActionMath first, then
    // arrives here through the same paste.

    /**
     * Transform - Encapsulates position, rotation, scale and model matrix calculation
     * Used by all 3D objects to track their world transform
     */
    Transform = class Transform {
        constructor() {
            this.position = new Vector3(0, 0, 0);
            this.rotation = new Quaternion(0, 0, 0, 1);
            this.scale = new Vector3(1, 1, 1);
        }

        /**
         * Sync transform from a physics body
         * @param {Object} body - Physics body with position and rotation
         */
        syncFromPhysicsBody(body) {
            if (!body) return;

            this.position.x = body.position.x;
            this.position.y = body.position.y;
            this.position.z = body.position.z;

            this.rotation = body.rotation;
        }

        /**
         * Copy this transform's values
         * @returns {Transform} New Transform with same values
         */
        clone() {
            const clone = new Transform();
            clone.position = this.position.clone();
            clone.rotation = new Quaternion(this.rotation.x, this.rotation.y, this.rotation.z, this.rotation.w);
            clone.scale = this.scale.clone();
            return clone;
        }

        /**
         * Transform a point from local space to world space
         * Applies rotation, scale, and translation
         * @param {Vector3} point - Point in local space
         * @returns {Vector3} Point in world space
         */
        transformPoint(point) {
            // 1. Scale first (Local Space)
            const sx = point.x * this.scale.x;
            const sy = point.y * this.scale.y;
            const sz = point.z * this.scale.z;

            // 2. Rotate (after scaling)
            const qx = this.rotation.x,
                qy = this.rotation.y,
                qz = this.rotation.z,
                qw = this.rotation.w;

            // q * v (v is the scaled point)
            const ix = qw * sx + qy * sz - qz * sy;
            const iy = qw * sy + qz * sx - qx * sz;
            const iz = qw * sz + qx * sy - qy * sx;
            const iw = -qx * sx - qy * sy - qz * sz;

            // (q * v) * q^-1
            const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
            const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
            const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

            // 3. Translate
            return new Vector3(rx + this.position.x, ry + this.position.y, rz + this.position.z);
        }

        /**
         * Transform a vector from local space to world space
         * Only applies rotation and scale (no translation)
         * @param {Vector3} vector - Vector in local space
         * @returns {Vector3} Vector in world space
         */
        transformVector(vector) {
            // 1. Scale first (Local Space)
            const sx = vector.x * this.scale.x;
            const sy = vector.y * this.scale.y;
            const sz = vector.z * this.scale.z;

            // 2. Rotate (after scaling)
            const qx = this.rotation.x,
                qy = this.rotation.y,
                qz = this.rotation.z,
                qw = this.rotation.w;

            // q * v (v is the scaled vector)
            const ix = qw * sx + qy * sz - qz * sy;
            const iy = qw * sy + qz * sx - qx * sz;
            const iz = qw * sz + qx * sy - qy * sx;
            const iw = -qx * sx - qy * sy - qz * sz;

            // (q * v) * q^-1
            const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
            const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
            const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

            return new Vector3(rx, ry, rz);
        }

        /**
         * Transform a point from local space to world space into a destination vector
         * Applies rotation, scale, and translation
         * Reuses the destination vector to avoid allocation
         * @param {Vector3} point - Point in local space
         * @param {Vector3} dest - Destination vector to store result
         * @returns {Vector3} The destination vector
         */
        transformPointInto(point, dest) {
            // 1. Scale first (Local Space)
            const sx = point.x * this.scale.x;
            const sy = point.y * this.scale.y;
            const sz = point.z * this.scale.z;

            // 2. Rotate (after scaling)
            const qx = this.rotation.x,
                qy = this.rotation.y,
                qz = this.rotation.z,
                qw = this.rotation.w;

            // q * v
            const ix = qw * sx + qy * sz - qz * sy;
            const iy = qw * sy + qz * sx - qx * sz;
            const iz = qw * sz + qx * sy - qy * sx;
            const iw = -qx * sx - qy * sy - qz * sz;

            // (q * v) * q^-1
            const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
            const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
            const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

            // 3. Translate and store in destination
            dest.x = rx + this.position.x;
            dest.y = ry + this.position.y;
            dest.z = rz + this.position.z;
            return dest;
        }

        /**
         * Transform a vector from local space to world space into a destination vector
         * Only applies rotation and scale (no translation)
         * Reuses the destination vector to avoid allocation
         * @param {Vector3} vector - Vector in local space
         * @param {Vector3} dest - Destination vector to store result
         * @returns {Vector3} The destination vector
         */
        transformVectorInto(vector, dest) {
            // 1. Scale first (Local Space)
            const sx = vector.x * this.scale.x;
            const sy = vector.y * this.scale.y;
            const sz = vector.z * this.scale.z;

            // 2. Rotate (after scaling)
            const qx = this.rotation.x,
                qy = this.rotation.y,
                qz = this.rotation.z,
                qw = this.rotation.w;

            // q * v
            const ix = qw * sx + qy * sz - qz * sy;
            const iy = qw * sy + qz * sx - qx * sz;
            const iz = qw * sz + qx * sy - qy * sx;
            const iw = -qx * sx - qy * sy - qz * sz;

            // (q * v) * q^-1
            const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
            const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
            const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

            // Store in destination
            dest.x = rx;
            dest.y = ry;
            dest.z = rz;
            return dest;
        }
    }

    ActionPhysics.Transform = Transform;

}
ActionPhysics.Transform = Transform;


// ==== src/outro.js ====
    // Every math class came from the host, so exactly one set is live in the page.
    ActionPhysics.usingHostMath = !!(host.Vector3 && host.Quaternion && host.Matrix4 && host.Scalar);

    return ActionPhysics;
}));
