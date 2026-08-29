// ActionPhysics 0.1.0 — built 2026-08-24T06:31:13.958Z
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


// ==== src/spatial/AABB.js ====
/**
 * Axis-aligned bounding box. min/max are Vector3 instances (in whichever precision the host
 * math package uses), min <= max on every axis always holds outside of construction.
 *
 * Every method here is allocation-free — this type lives inside broadphase/BVH sweeps that
 * run every tick over every body.
 */
class AABB {
    constructor() {
        this.min = new Vector3(Infinity, Infinity, Infinity);
        this.max = new Vector3(-Infinity, -Infinity, -Infinity);
    }

    setEmpty() {
        this.min.set(Infinity, Infinity, Infinity);
        this.max.set(-Infinity, -Infinity, -Infinity);
        return this;
    }

    setFromMinMax(minX, minY, minZ, maxX, maxY, maxZ) {
        this.min.set(minX, minY, minZ);
        this.max.set(maxX, maxY, maxZ);
        return this;
    }

    copy(other) {
        this.min.copy(other.min);
        this.max.copy(other.max);
        return this;
    }

    // this = the box around center +/- halfExtents, both Vector3.
    setFromCenterHalfExtents(center, halfExtents) {
        this.min.x = center.x - halfExtents.x;
        this.min.y = center.y - halfExtents.y;
        this.min.z = center.z - halfExtents.z;
        this.max.x = center.x + halfExtents.x;
        this.max.y = center.y + halfExtents.y;
        this.max.z = center.z + halfExtents.z;
        return this;
    }

    // Grow this box (in place) to also contain `other`.
    combineInPlace(other) {
        if (other.min.x < this.min.x) this.min.x = other.min.x;
        if (other.min.y < this.min.y) this.min.y = other.min.y;
        if (other.min.z < this.min.z) this.min.z = other.min.z;
        if (other.max.x > this.max.x) this.max.x = other.max.x;
        if (other.max.y > this.max.y) this.max.y = other.max.y;
        if (other.max.z > this.max.z) this.max.z = other.max.z;
        return this;
    }

    // this = union(a, b). Safe when this aliases a or b.
    static combineInto(out, a, b) {
        out.min.x = Math.min(a.min.x, b.min.x);
        out.min.y = Math.min(a.min.y, b.min.y);
        out.min.z = Math.min(a.min.z, b.min.z);
        out.max.x = Math.max(a.max.x, b.max.x);
        out.max.y = Math.max(a.max.y, b.max.y);
        out.max.z = Math.max(a.max.z, b.max.z);
        return out;
    }

    // Grow every face outward by `margin` (in place). Used for a speculative-contact skin, so a
    // fast-moving body's broadphase box still catches a pair before penetration.
    expandInPlace(margin) {
        this.min.x -= margin; this.min.y -= margin; this.min.z -= margin;
        this.max.x += margin; this.max.y += margin; this.max.z += margin;
        return this;
    }

    intersects(other) {
        return this.min.x <= other.max.x && this.max.x >= other.min.x &&
               this.min.y <= other.max.y && this.max.y >= other.min.y &&
               this.min.z <= other.max.z && this.max.z >= other.min.z;
    }

    containsPoint(p) {
        return p.x >= this.min.x && p.x <= this.max.x &&
               p.y >= this.min.y && p.y <= this.max.y &&
               p.z >= this.min.z && p.z <= this.max.z;
    }

    containsAABB(other) {
        return other.min.x >= this.min.x && other.max.x <= this.max.x &&
               other.min.y >= this.min.y && other.max.y <= this.max.y &&
               other.min.z >= this.min.z && other.max.z <= this.max.z;
    }

    // Half of the box's surface area (xy + yz + zx face pairs). A cheap, consistent BVH split
    // heuristic — never called per-tick, only when the static tree is (re)built.
    surfaceArea() {
        const dx = this.max.x - this.min.x;
        const dy = this.max.y - this.min.y;
        const dz = this.max.z - this.min.z;
        return dx * dy + dy * dz + dz * dx;
    }

    centerInto(out) {
        out.x = (this.min.x + this.max.x) * 0.5;
        out.y = (this.min.y + this.max.y) * 0.5;
        out.z = (this.min.z + this.max.z) * 0.5;
        return out;
    }

    isFinite() {
        return this.min.isFinite() && this.max.isFinite();
    }
}

ActionPhysics.AABB = AABB;


// ==== src/shapes/Shape.js ====
/**
 * Shape contract. Every shape provides, in LOCAL space (unrotated, centered on its own origin):
 *
 *   supportInto(out, direction)   farthest point on the shape along `direction` (need not be
 *                                 normalized). This is the ONLY primitive GJK/EPA require —
 *                                 everything else here exists for AABBs, mass properties and
 *                                 the visual bench, not collision.
 *   localAABBInto(out)            tight local-space AABB.
 *   computeMassData()             { mass, inertia: Matrix3, centerOfMass: Vector3 } for a shape
 *                                 of density 1; RigidBody scales inertia by (mass / this.volume)
 *                                 when the caller supplies its own mass.
 *   volume()                      for the density scaling above.
 *
 * Shapes never allocate on the hot path: supportInto/localAABBInto write into caller-owned
 * `out` arguments. computeMassData() runs once per body and may allocate.
 */
class Shape {
    // A shape reports the CATEGORY of margin its narrowphase pair needs. Plane and Triangle are
    // degenerate (infinite extent / zero thickness) and get special-cased at dispatch rather than
    // patched inside GJK/EPA — see plan.md, Shapes section.
    constructor(type) {
        this.type = type;
    }

    supportInto(out, direction) {
        throw new Error('Shape.supportInto not implemented for ' + this.type);
    }

    localAABBInto(out) {
        throw new Error('Shape.localAABBInto not implemented for ' + this.type);
    }

    computeMassData() {
        throw new Error('Shape.computeMassData not implemented for ' + this.type);
    }

    volume() {
        throw new Error('Shape.volume not implemented for ' + this.type);
    }
}

ActionPhysics.Shape = Shape;


// ==== src/shapes/BoxShape.js ====
// Every dimension is a half-extent (plan.md, Units and conventions) — matches AABB, matches
// every other shape's convention. No shape silently uses a different one.
class BoxShape extends Shape {
    constructor(halfWidth, halfHeight, halfDepth) {
        super('box');
        this.halfWidth = halfWidth;
        this.halfHeight = halfHeight;
        this.halfDepth = halfDepth;
    }

    supportInto(out, direction) {
        out.x = direction.x >= 0 ? this.halfWidth : -this.halfWidth;
        out.y = direction.y >= 0 ? this.halfHeight : -this.halfHeight;
        out.z = direction.z >= 0 ? this.halfDepth : -this.halfDepth;
        return out;
    }

    localAABBInto(out) {
        out.min.set(-this.halfWidth, -this.halfHeight, -this.halfDepth);
        out.max.set(this.halfWidth, this.halfHeight, this.halfDepth);
        return out;
    }

    volume() {
        return 8 * this.halfWidth * this.halfHeight * this.halfDepth;
    }

    computeMassData() {
        const w = 2 * this.halfWidth, h = 2 * this.halfHeight, d = 2 * this.halfDepth;
        const mass = this.volume();
        // Solid cuboid, density 1: I_xx = m(h^2+d^2)/12, cyclic.
        const inertia = new Matrix3().setDiagonal(new Vector3(
            mass * (h * h + d * d) / 12,
            mass * (w * w + d * d) / 12,
            mass * (w * w + h * h) / 12
        ));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.BoxShape = BoxShape;


// ==== src/shapes/SphereShape.js ====
class SphereShape extends Shape {
    constructor(radius) {
        super('sphere');
        this.radius = radius;
    }

    supportInto(out, direction) {
        // Direction need not be unit length; normalize here so the support point sits exactly
        // on the surface regardless of the caller's vector magnitude.
        const lsq = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (lsq === 0) { out.x = this.radius; out.y = 0; out.z = 0; return out; }
        const s = this.radius / Math.sqrt(lsq);
        out.x = direction.x * s; out.y = direction.y * s; out.z = direction.z * s;
        return out;
    }

    localAABBInto(out) {
        out.min.set(-this.radius, -this.radius, -this.radius);
        out.max.set(this.radius, this.radius, this.radius);
        return out;
    }

    volume() {
        return (4 / 3) * Scalar.PI * this.radius * this.radius * this.radius;
    }

    computeMassData() {
        const mass = this.volume();
        const i = 0.4 * mass * this.radius * this.radius; // solid sphere, density 1: I = 2/5 m r^2
        const inertia = new Matrix3().setDiagonal(new Vector3(i, i, i));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.SphereShape = SphereShape;


// ==== src/shapes/CylinderShape.js ====
// Axis is local Y. halfHeight is a half-extent (see plan.md, Units and conventions) — this is
// the deliberate departure from the predecessor, whose capsule took total height and silently
// broke callers written in half-extents everywhere else.
class CylinderShape extends Shape {
    constructor(radius, halfHeight) {
        super('cylinder');
        this.radius = radius;
        this.halfHeight = halfHeight;
    }

    supportInto(out, direction) {
        const sigma = Math.sqrt(direction.x * direction.x + direction.z * direction.z);
        if (sigma > 0) {
            const s = this.radius / sigma;
            out.x = direction.x * s;
            out.z = direction.z * s;
        } else {
            out.x = 0;
            out.z = 0;
        }
        out.y = direction.y >= 0 ? this.halfHeight : -this.halfHeight;
        return out;
    }

    localAABBInto(out) {
        out.min.set(-this.radius, -this.halfHeight, -this.radius);
        out.max.set(this.radius, this.halfHeight, this.radius);
        return out;
    }

    volume() {
        return Scalar.PI * this.radius * this.radius * (2 * this.halfHeight);
    }

    computeMassData() {
        const r = this.radius, h = 2 * this.halfHeight;
        const mass = this.volume();
        const iAxis = 0.5 * mass * r * r;                                   // about Y
        const iSide = mass * (3 * r * r + h * h) / 12;                       // about X and Z
        const inertia = new Matrix3().setDiagonal(new Vector3(iSide, iAxis, iSide));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.CylinderShape = CylinderShape;


// ==== src/shapes/ConeShape.js ====
// Axis is local Y, apex at +halfHeight, base circle at -halfHeight. halfHeight is a half-extent.
class ConeShape extends Shape {
    constructor(radius, halfHeight) {
        super('cone');
        this.radius = radius;
        this.halfHeight = halfHeight;
    }

    // Exact support of a cone is either the apex or a base-rim point, chosen by whichever the
    // direction favors — no iteration needed, unlike a general convex hull.
    supportInto(out, direction) {
        const h = this.halfHeight;
        const sigma = Math.sqrt(direction.x * direction.x + direction.z * direction.z);
        // Base-rim candidate's projection onto `direction`, compared against the apex's.
        const rimProjection = sigma * this.radius - direction.y * h;
        const apexProjection = direction.y * h;
        if (apexProjection >= rimProjection) {
            out.x = 0; out.y = h; out.z = 0;
            return out;
        }
        if (sigma > 0) {
            const s = this.radius / sigma;
            out.x = direction.x * s;
            out.z = direction.z * s;
        } else {
            out.x = 0; out.z = 0;
        }
        out.y = -h;
        return out;
    }

    localAABBInto(out) {
        out.min.set(-this.radius, -this.halfHeight, -this.radius);
        out.max.set(this.radius, this.halfHeight, this.radius);
        return out;
    }

    volume() {
        return Scalar.PI * this.radius * this.radius * (2 * this.halfHeight) / 3;
    }

    // Solid cone, density 1, apex up. Standard formulas are about the base; centerOfMass shifts
    // the origin from local (0,0,0) — the geometric mid-height used for the AABB and support
    // function — to that centroid, at h/4 above the base (i.e. -halfHeight + h/4).
    computeMassData() {
        const r = this.radius, h = 2 * this.halfHeight;
        const mass = this.volume();
        const iAxis = 0.3 * mass * r * r;                                    // about Y, apex frame
        const iSideApex = mass * (3 * r * r + 2 * h * h) / 20;               // about X/Z through apex
        // Parallel-axis shift from the apex-based formula to the centroid (h/4 below apex along axis).
        const centroidOffset = h / 4;
        const iSideCentroid = iSideApex - mass * centroidOffset * centroidOffset;
        const inertia = new Matrix3().setDiagonal(new Vector3(iSideCentroid, iAxis, iSideCentroid));
        return {
            mass: mass,
            inertia: inertia,
            centerOfMass: new Vector3(0, -this.halfHeight + centroidOffset, 0)
        };
    }
}

ActionPhysics.ConeShape = ConeShape;


// ==== src/shapes/CapsuleShape.js ====
// Axis is local Y. Constructor takes TOTAL height, unlike every other shape here — noted
// explicitly because it is the one deliberate exception to the half-extent rule (plan.md,
// Units and conventions): a capsule's height already includes its hemispherical caps, so there
// is no natural "half-extent" reading that isn't itself confusing. segmentHalfLength is the
// half-length of the cylindrical core only (between sphere centers), derived once here.
class CapsuleShape extends Shape {
    constructor(radius, totalHeight) {
        super('capsule');
        if (totalHeight < 2 * radius) {
            throw new Error('CapsuleShape: totalHeight must be >= 2 * radius');
        }
        this.radius = radius;
        this.totalHeight = totalHeight;
        this.segmentHalfLength = totalHeight / 2 - radius;
    }

    supportInto(out, direction) {
        const centerY = direction.y >= 0 ? this.segmentHalfLength : -this.segmentHalfLength;
        const lsq = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (lsq === 0) { out.x = this.radius; out.y = centerY; out.z = 0; return out; }
        const s = this.radius / Math.sqrt(lsq);
        out.x = direction.x * s;
        out.y = direction.y * s + centerY;
        out.z = direction.z * s;
        return out;
    }

    localAABBInto(out) {
        const halfExtent = this.segmentHalfLength + this.radius;
        out.min.set(-this.radius, -halfExtent, -this.radius);
        out.max.set(this.radius, halfExtent, this.radius);
        return out;
    }

    volume() {
        const r = this.radius, hs = this.segmentHalfLength;
        const cylinder = Scalar.PI * r * r * (2 * hs);
        const sphere = (4 / 3) * Scalar.PI * r * r * r;
        return cylinder + sphere;
    }

    // Composite of a cylindrical core plus two hemispherical caps, each contributing its own
    // parallel-axis term. Standard closed forms; see e.g. Bullet/Rapier capsule inertia derivations.
    computeMassData() {
        const r = this.radius, hs = this.segmentHalfLength;
        const cylinderVolume = Scalar.PI * r * r * (2 * hs);
        const hemisphereVolume = (2 / 3) * Scalar.PI * r * r * r; // one hemisphere
        const mass = cylinderVolume + 2 * hemisphereVolume;

        const cylinderMass = cylinderVolume;   // density 1
        const hemisphereMass = hemisphereVolume;

        const iAxisCyl = 0.5 * cylinderMass * r * r;
        const iSideCyl = cylinderMass * (3 * r * r + (2 * hs) * (2 * hs)) / 12;

        // Solid hemisphere about its own flat-face centroid axis (through the sphere center, Y):
        const iAxisHemi = 0.4 * hemisphereMass * r * r; // same coefficient as full sphere for the polar axis
        // About an axis through the hemisphere's centroid perpendicular to the pole, then shifted
        // by the parallel-axis theorem out to the capsule's cylinder-cap junction at y = hs.
        const hemiCentroidOffset = (3 / 8) * r; // centroid distance from flat face along the axis
        const iSideHemiAboutOwnCentroid = hemisphereMass * (83 / 320) * r * r;
        const distFromCapsuleCenter = hs + hemiCentroidOffset;
        const iSideHemiShifted = iSideHemiAboutOwnCentroid + hemisphereMass * distFromCapsuleCenter * distFromCapsuleCenter;

        const iAxis = iAxisCyl + 2 * iAxisHemi;
        const iSide = iSideCyl + 2 * iSideHemiShifted;

        const inertia = new Matrix3().setDiagonal(new Vector3(iSide, iAxis, iSide));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.CapsuleShape = CapsuleShape;


// ==== src/shapes/ConvexShape.js ====
// Arbitrary convex hull from a point cloud. Points are LOCAL-space Vector3, taken as already
// forming (or being reducible to) a convex hull — support/mass computation below do not verify
// convexity, matching every other shape here trusting its constructor input.
class ConvexShape extends Shape {
    constructor(points) {
        super('convex');
        this.points = points;
    }

    // Brute-force max-dot scan. O(n) per query; fine for the hull sizes physics shapes use
    // (tens of points), and simplicity here keeps GJK's one required primitive easy to trust.
    supportInto(out, direction) {
        const pts = this.points;
        let bestDot = -Infinity, bestIndex = 0;
        for (let i = 0; i < pts.length; i++) {
            const d = pts[i].x * direction.x + pts[i].y * direction.y + pts[i].z * direction.z;
            if (d > bestDot) { bestDot = d; bestIndex = i; }
        }
        out.x = pts[bestIndex].x; out.y = pts[bestIndex].y; out.z = pts[bestIndex].z;
        return out;
    }

    localAABBInto(out) {
        out.setEmpty();
        const pts = this.points;
        for (let i = 0; i < pts.length; i++) {
            const p = pts[i];
            if (p.x < out.min.x) out.min.x = p.x;
            if (p.y < out.min.y) out.min.y = p.y;
            if (p.z < out.min.z) out.min.z = p.z;
            if (p.x > out.max.x) out.max.x = p.x;
            if (p.y > out.max.y) out.max.y = p.y;
            if (p.z > out.max.z) out.max.z = p.z;
        }
        return out;
    }

    // No closed-form volume/inertia for an arbitrary hull without its face list (which this
    // shape does not carry — see plan.md's component list: Convex is a GJK/EPA support shape,
    // not a tessellated mesh). Approximated as the equivalent-volume sphere from the AABB's
    // bounding radius; a caller needing exact mass properties for a hull supplies its own via
    // a MeshShape (has faces) instead.
    volume() {
        const aabb = new AABB();
        this.localAABBInto(aabb);
        const c = new Vector3();
        aabb.centerInto(c);
        let r = 0;
        for (let i = 0; i < this.points.length; i++) {
            const d = this.points[i].distanceTo(c);
            if (d > r) r = d;
        }
        this._boundingRadius = r;
        return (4 / 3) * Scalar.PI * r * r * r;
    }

    computeMassData() {
        const mass = this.volume();
        const r = this._boundingRadius;
        const i = 0.4 * mass * r * r;
        const inertia = new Matrix3().setDiagonal(new Vector3(i, i, i));
        return { mass: mass, inertia: inertia, centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.ConvexShape = ConvexShape;


// ==== src/shapes/PlaneShape.js ====
// A finite plane: a flat rectangle with zero thickness. Degenerate by construction — special-
// cased explicitly rather than patched into GJK/EPA later (plan.md, Shapes: "Plane and Triangle
// are degenerate and are special-cased explicitly at the shape level, not patched later").
//
// orientation selects which local axis is the normal: 'x', 'y', or 'z'. halfW/halfL extend along
// the other two axes, in the cyclic order (y,z) for 'x', (z,x) for 'y', (x,y) for 'z' — i.e. the
// same convention Vector3.cross uses, so normal x axis1 = axis2 always holds.
class PlaneShape extends Shape {
    constructor(orientation, halfW, halfL) {
        super('plane');
        this.orientation = orientation;
        this.halfW = halfW;
        this.halfL = halfL;
    }

    // Zero thickness means the support point always lies exactly on the plane, regardless of
    // the direction's component along the normal.
    supportInto(out, direction) {
        if (this.orientation === 'x') {
            out.x = 0;
            out.y = direction.y >= 0 ? this.halfW : -this.halfW;
            out.z = direction.z >= 0 ? this.halfL : -this.halfL;
        } else if (this.orientation === 'y') {
            out.x = direction.x >= 0 ? this.halfL : -this.halfL;
            out.y = 0;
            out.z = direction.z >= 0 ? this.halfW : -this.halfW;
        } else {
            out.x = direction.x >= 0 ? this.halfW : -this.halfW;
            out.y = direction.y >= 0 ? this.halfL : -this.halfL;
            out.z = 0;
        }
        return out;
    }

    localAABBInto(out) {
        if (this.orientation === 'x') out.setFromMinMax(0, -this.halfW, -this.halfL, 0, this.halfW, this.halfL);
        else if (this.orientation === 'y') out.setFromMinMax(-this.halfL, 0, -this.halfW, this.halfL, 0, this.halfW);
        else out.setFromMinMax(-this.halfW, -this.halfL, 0, this.halfW, this.halfL, 0);
        return out;
    }

    // A plane is meant for static/kinematic use (infinite-mass equivalent geometry); it carries
    // zero volume and zero-mass data rather than pretending to a solid it is not.
    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.PlaneShape = PlaneShape;


// ==== src/shapes/TriangleShape.js ====
// A single zero-thickness triangle in local space. Degenerate like PlaneShape, for the same
// reason — see PlaneShape's header. Used both standalone and as the per-triangle shape a
// MeshShape's midphase dispatches into (plan.md: "midphase — which triangles of a mesh?").
class TriangleShape extends Shape {
    constructor(a, b, c) {
        super('triangle');
        this.a = a; this.b = b; this.c = c;
    }

    supportInto(out, direction) {
        const a = this.a, b = this.b, c = this.c;
        const da = a.x * direction.x + a.y * direction.y + a.z * direction.z;
        const db = b.x * direction.x + b.y * direction.y + b.z * direction.z;
        const dc = c.x * direction.x + c.y * direction.y + c.z * direction.z;
        const best = (da >= db && da >= dc) ? a : (db >= dc ? b : c);
        out.x = best.x; out.y = best.y; out.z = best.z;
        return out;
    }

    localAABBInto(out) {
        out.setEmpty();
        const pts = [this.a, this.b, this.c];
        for (let i = 0; i < 3; i++) {
            const p = pts[i];
            if (p.x < out.min.x) out.min.x = p.x;
            if (p.y < out.min.y) out.min.y = p.y;
            if (p.z < out.min.z) out.min.z = p.z;
            if (p.x > out.max.x) out.max.x = p.x;
            if (p.y > out.max.y) out.max.y = p.y;
            if (p.z > out.max.z) out.max.z = p.z;
        }
        return out;
    }

    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.TriangleShape = TriangleShape;


// ==== src/shapes/MeshShape.js ====
// Static triangle mesh: a vertex list plus flat index triples. Zero mass by construction — a
// mesh is a static/kinematic-only shape (plan.md: BVH is "built once for static geometry").
// The midphase BVH over these triangles is built lazily by whatever consumes this shape
// (plan.md, Spatial: "one BVH implementation, three call sites"); this class only owns geometry.
class MeshShape extends Shape {
    constructor(vertices, indices) {
        super('mesh');
        this.vertices = vertices;   // Vector3[]
        this.indices = indices;     // flat Uint32Array-able index triples
        this.triangleCount = (indices.length / 3) | 0;
    }

    triangleAt(i, outA, outB, outC) {
        const base = i * 3;
        const va = this.vertices[this.indices[base]];
        const vb = this.vertices[this.indices[base + 1]];
        const vc = this.vertices[this.indices[base + 2]];
        outA.copy(va); outB.copy(vb); outC.copy(vc);
    }

    // A mesh has no single well-defined support point (it's a shell, not a solid convex body) —
    // narrowphase dispatches per-triangle via TriangleShape instead of calling this directly.
    supportInto(out, direction) {
        throw new Error('MeshShape.supportInto: dispatch per-triangle, a mesh is not itself convex');
    }

    localAABBInto(out) {
        out.setEmpty();
        const verts = this.vertices;
        for (let i = 0; i < verts.length; i++) {
            const p = verts[i];
            if (p.x < out.min.x) out.min.x = p.x;
            if (p.y < out.min.y) out.min.y = p.y;
            if (p.z < out.min.z) out.min.z = p.z;
            if (p.x > out.max.x) out.max.x = p.x;
            if (p.y > out.max.y) out.max.y = p.y;
            if (p.z > out.max.z) out.max.z = p.z;
        }
        return out;
    }

    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.MeshShape = MeshShape;


// ==== src/shapes/CompoundShape.js ====
// A CompoundShapeChild is a leaf shape at a fixed local offset/orientation within a compound.
// Plain data — the compound's owning body drives everything else.
class CompoundShapeChild {
    constructor(shape, localPosition, localRotation) {
        this.shape = shape;
        this.localPosition = localPosition;       // Vector3
        this.localRotation = localRotation;        // Quaternion
    }
}

// A rigid union of child shapes, each at its own local offset. Mass properties combine via the
// parallel-axis theorem per child; the midphase BVH over children is built by whatever consumes
// this shape, same division of ownership as MeshShape.
class CompoundShape extends Shape {
    constructor(children) {
        super('compound');
        this.children = children || []; // CompoundShapeChild[]
    }

    addChild(shape, localPosition, localRotation) {
        this.children.push(new CompoundShapeChild(shape, localPosition, localRotation));
        return this;
    }

    // Not itself convex — narrowphase dispatches per-child, same reasoning as MeshShape.
    supportInto(out, direction) {
        throw new Error('CompoundShape.supportInto: dispatch per-child, a compound is not itself convex');
    }

    localAABBInto(out) {
        out.setEmpty();
        const childAABB = new AABB();
        const rotMat = new Matrix3();
        const corner = new Vector3();
        for (let i = 0; i < this.children.length; i++) {
            const child = this.children[i];
            child.shape.localAABBInto(childAABB);
            rotMat.fromQuaternion(child.localRotation);
            // Rotate the child's local AABB conservatively: transform all 8 corners.
            for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
                corner.x = cx ? childAABB.max.x : childAABB.min.x;
                corner.y = cy ? childAABB.max.y : childAABB.min.y;
                corner.z = cz ? childAABB.max.z : childAABB.min.z;
                rotMat.transformVector3(corner);
                corner.addInPlace(child.localPosition);
                if (corner.x < out.min.x) out.min.x = corner.x;
                if (corner.y < out.min.y) out.min.y = corner.y;
                if (corner.z < out.min.z) out.min.z = corner.z;
                if (corner.x > out.max.x) out.max.x = corner.x;
                if (corner.y > out.max.y) out.max.y = corner.y;
                if (corner.z > out.max.z) out.max.z = corner.z;
            }
        }
        return out;
    }

    volume() {
        let v = 0;
        for (let i = 0; i < this.children.length; i++) v += this.children[i].shape.volume();
        return v;
    }

    // Combines child mass data about the compound's own local origin, via the parallel-axis
    // theorem: a child's inertia about the compound origin is its own local inertia (rotated into
    // the compound frame) plus m * (translation contribution from the offset).
    computeMassData() {
        let totalMass = 0;
        const centerOfMass = new Vector3(0, 0, 0);
        const childData = [];
        for (let i = 0; i < this.children.length; i++) {
            const child = this.children[i];
            const data = child.shape.computeMassData();
            childData.push(data);
            totalMass += data.mass;
            centerOfMass.addScaledInPlace(child.localPosition, data.mass);
        }
        if (totalMass > 0) centerOfMass.scaleInPlace(1 / totalMass);

        const inertia = new Matrix3().zero();
        const rotMat = new Matrix3();
        const rotated = new Matrix3();
        const rotatedT = new Matrix3();
        for (let i = 0; i < this.children.length; i++) {
            const child = this.children[i];
            const data = childData[i];
            rotMat.fromQuaternion(child.localRotation);
            // rotated = R * I_local * R^T — child's local inertia expressed in the compound frame.
            rotated.multiplyFrom(rotMat, data.inertia);
            rotatedT.transposeInto(rotMat);
            rotated.multiply(rotatedT);

            // Parallel-axis shift from the child's own centroid to the compound's centerOfMass.
            const dx = child.localPosition.x + data.centerOfMass.x - centerOfMass.x;
            const dy = child.localPosition.y + data.centerOfMass.y - centerOfMass.y;
            const dz = child.localPosition.z + data.centerOfMass.z - centerOfMass.z;
            const d2 = dx * dx + dy * dy + dz * dz;
            const m = data.mass;

            inertia.e00 += rotated.e00 + m * (d2 - dx * dx);
            inertia.e01 += rotated.e01 + m * (-dx * dy);
            inertia.e02 += rotated.e02 + m * (-dx * dz);
            inertia.e10 += rotated.e10 + m * (-dy * dx);
            inertia.e11 += rotated.e11 + m * (d2 - dy * dy);
            inertia.e12 += rotated.e12 + m * (-dy * dz);
            inertia.e20 += rotated.e20 + m * (-dz * dx);
            inertia.e21 += rotated.e21 + m * (-dz * dy);
            inertia.e22 += rotated.e22 + m * (d2 - dz * dz);
        }

        return { mass: totalMass, inertia: inertia, centerOfMass: centerOfMass };
    }
}

ActionPhysics.CompoundShapeChild = CompoundShapeChild;
ActionPhysics.CompoundShape = CompoundShape;


// ==== src/shapes/LineSweptShape.js ====
// A shape swept along a line segment: the Minkowski sum of `shape` with the segment from
// -halfLength to +halfLength along local Y. Used for continuous-collision / swept queries
// (plan.md, component 11: Queries — ray casting, shape sweeps) without a dedicated CCD solver:
// the query just asks "does this swept volume touch anything", which is a support function away
// once the base shape has one.
class LineSweptShape extends Shape {
    constructor(shape, halfLength) {
        super('lineswept');
        this.shape = shape;
        this.halfLength = halfLength;
    }

    // Minkowski sum with a segment: support(d) = shape.support(d) + endpoint(d), where the
    // endpoint chosen is whichever end of the segment is farther along d.
    supportInto(out, direction) {
        this.shape.supportInto(out, direction);
        out.y += direction.y >= 0 ? this.halfLength : -this.halfLength;
        return out;
    }

    localAABBInto(out) {
        this.shape.localAABBInto(out);
        out.min.y -= this.halfLength;
        out.max.y += this.halfLength;
        return out;
    }

    // A sweep is a query tool, not a body shape — it carries no mass properties of its own.
    volume() { return 0; }

    computeMassData() {
        return { mass: 0, inertia: new Matrix3().zero(), centerOfMass: new Vector3(0, 0, 0) };
    }
}

ActionPhysics.LineSweptShape = LineSweptShape;


// ==== src/bodies/RigidBody.js ====
// Body type, as a first-class concept (plan.md: "What mature engines have that Goblin doesn't" #2).
// Checked in exactly one place per stage, never as scattered mass===Infinity comparisons.
const BODY_STATIC = 0;
const BODY_KINEMATIC = 1;
const BODY_DYNAMIC = 2;

let _nextBodyId = 1;

/**
 * A rigid body: shape + world transform + (for dynamic bodies) mass/motion state. Broadphase and
 * midphase only need shape + transform + AABB; the mass/motion/material fields exist now rather
 * than being bolted on at the solver stage, so this class is not rebuilt twice (plan.md's "one
 * owner per concern" applies to the body's own field layout too - Mass is owned here, not
 * scattered across whichever stage happens to need it first).
 *
 * Field groups match the API surface table in plan.md: Transform, Motion, Forces, Material, Mass,
 * Filtering, Identity.
 */
class RigidBody {
    constructor(shape, mass) {
        // ---- Identity ----
        this.id = _nextBodyId++;
        this.shape = shape;
        this.debugName = null;
        this.world = null; // set by World.addRigidBody
        this.bodyType = mass > 0 ? BODY_DYNAMIC : BODY_STATIC;

        // ---- Transform ----
        this.position = new Vector3(0, 0, 0);
        this.rotation = new Quaternion(0, 0, 0, 1);
        this._aabb = new AABB();            // tight geometric bound (getAABB)
        this._broadphaseAABB = new AABB();  // fattened for speculative contacts (getBroadphaseAABB)
        this._aabbDirty = true;

        // ---- Mass ----
        // mass===0 is a static/kinematic body: infinite effective mass, zero inverse.
        this._mass = mass || 0;
        this._massInverted = this._mass > 0 ? 1 / this._mass : 0;
        this.inertiaTensor = new Matrix3();       // local-space, set by setMassFromShape()
        this.inverseInertiaTensor = new Matrix3(); // local-space inverse
        this._worldInverseInertiaTensor = new Matrix3(); // R * I^-1_local * R^T, refreshed by updateDerived()
        if (shape && this._mass > 0) this.setMassFromShape(shape, this._mass);

        // ---- Motion ----
        this.linear_velocity = new Vector3(0, 0, 0);
        this.angular_velocity = new Vector3(0, 0, 0);
        this.linear_factor = new Vector3(1, 1, 1);   // per-axis velocity mask, e.g. lock an axis with 0
        this.angular_factor = new Vector3(1, 1, 1);

        // ---- Forces ----
        this.accumulated_force = new Vector3(0, 0, 0);
        this.accumulated_torque = new Vector3(0, 0, 0);
        this.gravity = null; // null = use World.gravity; setGravity() overrides per-body

        // ---- Material ----
        this.friction = 0.5;
        this.restitution = 0;
        this.linear_damping = 0;
        this.angular_damping = 0;

        // ---- Filtering ----
        this.collision_mask = 0xFFFFFFFF;
        this.collision_groups = 1;

        // ---- Events ----
        this._listeners = {};

        // Sleep (owned entirely by the sleep manager once it exists - plan.md, Sleep). Present
        // here only as the state a body carries; no stage but the sleep manager writes to it.
        this.isAwake = true;
        this.sleepTimer = 0;
    }

    get is_static() { return this.bodyType === RigidBody.STATIC; }
    get mass() { return this._mass; }

    // Scales the shape's density-1 inertia by (mass / shape.volume()), per Shape's contract
    // (src/shapes/Shape.js) — computeMassData() always returns density-1 values.
    setMassFromShape(shape, mass) {
        this._mass = mass;
        this._massInverted = mass > 0 ? 1 / mass : 0;
        if (mass <= 0) {
            this.inertiaTensor.zero();
            this.inverseInertiaTensor.zero();
            return;
        }
        const data = shape.computeMassData();
        const volume = shape.volume();
        const scale = volume > 0 ? mass / volume : 0;
        this.inertiaTensor.copy(data.inertia);
        this.inertiaTensor.e00 *= scale; this.inertiaTensor.e01 *= scale; this.inertiaTensor.e02 *= scale;
        this.inertiaTensor.e10 *= scale; this.inertiaTensor.e11 *= scale; this.inertiaTensor.e12 *= scale;
        this.inertiaTensor.e20 *= scale; this.inertiaTensor.e21 *= scale; this.inertiaTensor.e22 *= scale;
        this.inverseInertiaTensor.invertInto(this.inertiaTensor);
    }

    setGravity(x, y, z) {
        this.gravity = new Vector3(x, y, z);
        return this;
    }

    // Refresh everything derived from position/rotation: the world AABB (tight and broadphase
    // variants) and the world-space inverse inertia tensor. Called once per body per tick by
    // whichever stage owns "current" - narrowphase and the solver assume it has already run (Rule
    // 1: stage contracts are absolute).
    //
    // `dt` (optional) is this tick's timestep, used only to size the broadphase AABB's velocity
    // sweep (see _recomputeBroadphaseAABB). The tight AABB (getAABB) never depends on dt.
    updateDerived(dt) {
        this._recomputeAABB();
        this._recomputeBroadphaseAABB(dt || 0);
        this._recomputeWorldInverseInertia();
        return this;
    }

    // The TIGHT world AABB: the exact rotated bound of the shape at the current transform, no
    // margin. This is the body's geometric truth - what a raycast/query wants, and what getAABB()
    // returns. Broadphase uses the fattened variant below instead (getBroadphaseAABB).
    _recomputeAABB() {
        const local = RigidBody._scratchLocalAABB;
        this.shape.localAABBInto(local);
        // Conservative rotated bound via the 8-corner sweep, same technique CompoundShape uses for
        // its own children - correct for any rotation, not just axis-aligned ones.
        const rotMat = RigidBody._scratchMat3;
        rotMat.fromQuaternion(this.rotation);
        const corner = RigidBody._scratchVec;
        this._aabb.setEmpty();
        for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
            corner.x = cx ? local.max.x : local.min.x;
            corner.y = cy ? local.max.y : local.min.y;
            corner.z = cz ? local.max.z : local.min.z;
            rotMat.transformVector3(corner);
            corner.addInPlace(this.position);
            if (corner.x < this._aabb.min.x) this._aabb.min.x = corner.x;
            if (corner.y < this._aabb.min.y) this._aabb.min.y = corner.y;
            if (corner.z < this._aabb.min.z) this._aabb.min.z = corner.z;
            if (corner.x > this._aabb.max.x) this._aabb.max.x = corner.x;
            if (corner.y > this._aabb.max.y) this._aabb.max.y = corner.y;
            if (corner.z > this._aabb.max.z) this._aabb.max.z = corner.z;
        }
        this._aabbDirty = false;
    }

    // The BROADPHASE world AABB: the tight AABB fattened for speculative contacts by a fixed margin
    // plus this tick's velocity sweep on each axis, so a fast approach is caught a full tick BEFORE
    // the shapes actually overlap. That lookahead is what makes speculative contacts possible at
    // all: narrowphase can only create a pre-overlap (still-separated) contact point for a pair
    // broadphase actually reports, and a raw tight AABB doesn't overlap until the shapes already do
    // (by which time the body has fallen straight through the speculative window). Fattening only
    // ever ADDS candidate pairs, never removes one (Rule: broadphase no-false-negatives) -
    // narrowphase then culls precisely with its own per-pair margin. Kept SEPARATE from the tight
    // AABB so the body's geometric bound stays truthful for queries/rendering.
    //
    // Sweep is directional (grow the box only on the side the body is moving toward on each axis),
    // keeping the fattened box tight rather than symmetric - a body moving down grows its box
    // downward, not upward. SPECULATIVE_MARGIN is the absolute floor for the resting/slow case
    // where velocity*dt alone is ~0.
    _recomputeBroadphaseAABB(dt) {
        const m = RigidBody.SPECULATIVE_MARGIN;
        const sx = this.linear_velocity.x * dt, sy = this.linear_velocity.y * dt, sz = this.linear_velocity.z * dt;
        // Angular sweep: a point at the body's bounding radius R moves at up to |omega|*R, so a
        // spinning body's far corner sweeps that far this tick even when the CENTER (which the
        // linear sweep above tracks) barely moves. Missing this was a real bug: a box that tips
        // onto one corner spins up to a few rad/s, its OPPOSITE corner then approaches the ground
        // at |omega|*R (a couple of m/s) with the center's linear velocity pointing elsewhere, so
        // the linear sweep never grew the box toward that corner - the corner slammed in undetected
        // and the one-shot correction of the resulting deep overlap injected more spin, diverging.
        // Angular motion has no clean per-axis direction, so this term is applied ISOTROPICALLY
        // (all six faces) - the conservative honest choice, never an under-estimate. R is taken from
        // the tight AABB's own half-extent (its farthest corner from center).
        const ex = (this._aabb.max.x - this._aabb.min.x) * 0.5;
        const ey = (this._aabb.max.y - this._aabb.min.y) * 0.5;
        const ez = (this._aabb.max.z - this._aabb.min.z) * 0.5;
        const R = Math.sqrt(ex * ex + ey * ey + ez * ez);
        const wMag = Math.sqrt(this.angular_velocity.x * this.angular_velocity.x +
            this.angular_velocity.y * this.angular_velocity.y + this.angular_velocity.z * this.angular_velocity.z);
        const a = wMag * R * dt;
        this._broadphaseAABB.min.x = this._aabb.min.x - m - a - (sx < 0 ? -sx : 0);
        this._broadphaseAABB.max.x = this._aabb.max.x + m + a + (sx > 0 ? sx : 0);
        this._broadphaseAABB.min.y = this._aabb.min.y - m - a - (sy < 0 ? -sy : 0);
        this._broadphaseAABB.max.y = this._aabb.max.y + m + a + (sy > 0 ? sy : 0);
        this._broadphaseAABB.min.z = this._aabb.min.z - m - a - (sz < 0 ? -sz : 0);
        this._broadphaseAABB.max.z = this._aabb.max.z + m + a + (sz > 0 ? sz : 0);
    }

    _recomputeWorldInverseInertia() {
        if (this._massInverted === 0) { this._worldInverseInertiaTensor.zero(); return; }
        const rotMat = RigidBody._scratchMat3;
        rotMat.fromQuaternion(this.rotation);
        const rotT = RigidBody._scratchMat3b;
        rotT.transposeInto(rotMat);
        this._worldInverseInertiaTensor.multiplyFrom(rotMat, this.inverseInertiaTensor);
        this._worldInverseInertiaTensor.multiply(rotT);
    }

    // Broadphase's required primitive: the body's CURRENT world AABB. Assumes updateDerived() has
    // already run this tick (Rule 1) - getAABB() never recomputes on its own, so a stale call is a
    // caller bug surfaced as a stale box, not silently patched over here.
    getAABB() {
        return this._aabb;
    }

    // The fattened broadphase-query AABB (tight bound + speculative margin + velocity sweep).
    // Broadphase and midphase read THIS, not getAABB(), so a pair surfaces the tick before overlap
    // (see _recomputeBroadphaseAABB). Same staleness assumption as getAABB(): updateDerived() owns
    // recomputing it once per tick.
    getBroadphaseAABB() {
        return this._broadphaseAABB;
    }

    addListener(event, fn) {
        (this._listeners[event] || (this._listeners[event] = [])).push(fn);
        return this;
    }

    emit(event, arg) {
        const list = this._listeners[event];
        if (!list) return;
        for (let i = 0; i < list.length; i++) list[i](arg);
    }
}

// Scratch objects for the allocation-free AABB/inertia recompute above. Per-class, not shared
// across unrelated algorithms (plan.md: "scratch memory: per-stage arenas, never global") - these
// three are private to RigidBody's own derived-state recompute and touched nowhere else.
RigidBody._scratchLocalAABB = new AABB();
RigidBody._scratchMat3 = new Matrix3();
RigidBody._scratchMat3b = new Matrix3();
RigidBody._scratchVec = new Vector3();

RigidBody.STATIC = BODY_STATIC;
RigidBody.KINEMATIC = BODY_KINEMATIC;
RigidBody.DYNAMIC = BODY_DYNAMIC;

// Fixed broadphase-AABB fattening for speculative contacts (metres). Matches the narrowphase
// speculative base so the two stages agree on "how early is a contact worth seeing": broadphase
// surfaces the pair at least this far before overlap, and narrowphase then creates the actual
// pre-overlap point within its own (equal-or-larger, velocity-widened) margin. Kept as an
// absolute floor here; the velocity sweep in _recomputeAABB handles fast approaches on top of it.
RigidBody.SPECULATIVE_MARGIN = 0.02;

ActionPhysics.RigidBody = RigidBody;


// ==== src/spatial/BVH.js ====
/**
 * Static BVH over a fixed set of leaves, built once. Flattened array layout - array indexing, not
 * pointer chasing (plan.md, Spatial). One implementation, three call sites: compound children,
 * mesh triangles, swept queries.
 *
 * Construction takes a leaf count and two callbacks:
 *   leafAABBInto(out, leafIndex)   writes the leaf's AABB into `out`
 * The tree never touches what a "leaf" IS - a compound child, a mesh triangle, whatever - it only
 * knows AABBs and indices, so this file has no per-caller special cases.
 *
 * Nodes are stored in three parallel typed arrays: min/max as Float64Array (3 components each),
 * and an Int32Array carrying (left, right, leafIndex) - leafIndex is -1 for an internal node.
 * A leaf node has left = right = -1 implicitly (never read); an internal node has leafIndex = -1.
 */
class BVH {
    constructor() {
        this.nodeCount = 0;
        this._capacity = 0;
        this.minX = null; this.minY = null; this.minZ = null;
        this.maxX = null; this.maxY = null; this.maxZ = null;
        this.left = null; this.right = null; this.leafIndex = null;
        this.root = -1;
    }

    _ensureCapacity(n) {
        if (n <= this._capacity) return;
        this._capacity = n;
        this.minX = new Float64Array(n); this.minY = new Float64Array(n); this.minZ = new Float64Array(n);
        this.maxX = new Float64Array(n); this.maxY = new Float64Array(n); this.maxZ = new Float64Array(n);
        this.left = new Int32Array(n).fill(-1);
        this.right = new Int32Array(n).fill(-1);
        this.leafIndex = new Int32Array(n).fill(-1);
    }

    // Builds the tree over `leafCount` leaves. leafAABBInto(out, i) must fill `out` (an AABB) with
    // leaf i's bound. A degenerate call (leafCount === 0) leaves the tree empty (root === -1);
    // querying an empty tree is not an error, it just visits nothing.
    build(leafCount, leafAABBInto) {
        this.nodeCount = 0;
        this.root = -1;
        if (leafCount === 0) return this;

        // Up to 2*leafCount-1 nodes for a full binary tree over leafCount leaves.
        this._ensureCapacity(Math.max(1, 2 * leafCount - 1));

        const scratch = new AABB();
        const indices = new Int32Array(leafCount);
        const centerX = new Float64Array(leafCount), centerY = new Float64Array(leafCount), centerZ = new Float64Array(leafCount);
        const leafMinX = new Float64Array(leafCount), leafMinY = new Float64Array(leafCount), leafMinZ = new Float64Array(leafCount);
        const leafMaxX = new Float64Array(leafCount), leafMaxY = new Float64Array(leafCount), leafMaxZ = new Float64Array(leafCount);
        for (let i = 0; i < leafCount; i++) {
            leafAABBInto(scratch, i);
            indices[i] = i;
            leafMinX[i] = scratch.min.x; leafMinY[i] = scratch.min.y; leafMinZ[i] = scratch.min.z;
            leafMaxX[i] = scratch.max.x; leafMaxY[i] = scratch.max.y; leafMaxZ[i] = scratch.max.z;
            centerX[i] = (scratch.min.x + scratch.max.x) * 0.5;
            centerY[i] = (scratch.min.y + scratch.max.y) * 0.5;
            centerZ[i] = (scratch.min.z + scratch.max.z) * 0.5;
        }

        const self = this;
        function boundsOf(lo, hi) {
            let bx0 = Infinity, by0 = Infinity, bz0 = Infinity, bx1 = -Infinity, by1 = -Infinity, bz1 = -Infinity;
            for (let k = lo; k < hi; k++) {
                const i = indices[k];
                if (leafMinX[i] < bx0) bx0 = leafMinX[i]; if (leafMaxX[i] > bx1) bx1 = leafMaxX[i];
                if (leafMinY[i] < by0) by0 = leafMinY[i]; if (leafMaxY[i] > by1) by1 = leafMaxY[i];
                if (leafMinZ[i] < bz0) bz0 = leafMinZ[i]; if (leafMaxZ[i] > bz1) bz1 = leafMaxZ[i];
            }
            return { minX: bx0, minY: by0, minZ: bz0, maxX: bx1, maxY: by1, maxZ: bz1 };
        }

        // Median split on the widest axis of [lo, hi)'s combined bound. A cheap, deterministic
        // heuristic (no SAH) - correct measured cost per plan.md is 1.38 children visited per
        // query, which came from exactly this kind of structure; revisit only if a stress test
        // shows it isn't good enough.
        function buildRange(lo, hi) {
            const b = boundsOf(lo, hi);
            const nodeIdx = self.nodeCount++;
            self.minX[nodeIdx] = b.minX; self.minY[nodeIdx] = b.minY; self.minZ[nodeIdx] = b.minZ;
            self.maxX[nodeIdx] = b.maxX; self.maxY[nodeIdx] = b.maxY; self.maxZ[nodeIdx] = b.maxZ;

            if (hi - lo === 1) {
                self.leafIndex[nodeIdx] = indices[lo];
                self.left[nodeIdx] = -1; self.right[nodeIdx] = -1;
                return nodeIdx;
            }

            const sx = b.maxX - b.minX, sy = b.maxY - b.minY, sz = b.maxZ - b.minZ;
            let axis = 0, getCenter = centerX;
            if (sy >= sx && sy >= sz) { axis = 1; getCenter = centerY; }
            else if (sz >= sx && sz >= sy) { axis = 2; getCenter = centerZ; }

            // Partition indices[lo,hi) around the median center on the chosen axis, in place.
            const sub = indices.subarray(lo, hi);
            Array.prototype.sort.call(sub, function (ia, ib) { return getCenter[ia] - getCenter[ib]; });
            const mid = lo + ((hi - lo) >> 1);

            self.leafIndex[nodeIdx] = -1;
            self.left[nodeIdx] = buildRange(lo, mid);
            self.right[nodeIdx] = buildRange(mid, hi);
            return nodeIdx;
        }

        this.root = buildRange(0, leafCount);
        return this;
    }

    // Visits every leaf whose node AABB intersects `queryAABB`, calling onLeaf(leafIndex) for
    // each. No allocation - an explicit stack in a plain array, reused across calls via `_stack`.
    query(queryAABB, onLeaf) {
        if (this.root === -1) return;
        const qminx = queryAABB.min.x, qminy = queryAABB.min.y, qminz = queryAABB.min.z;
        const qmaxx = queryAABB.max.x, qmaxy = queryAABB.max.y, qmaxz = queryAABB.max.z;

        const stack = this._stack || (this._stack = []);
        let sp = 0;
        stack[sp++] = this.root;
        while (sp > 0) {
            const node = stack[--sp];
            if (this.minX[node] > qmaxx || this.maxX[node] < qminx ||
                this.minY[node] > qmaxy || this.maxY[node] < qminy ||
                this.minZ[node] > qmaxz || this.maxZ[node] < qminz) continue;

            if (this.leafIndex[node] !== -1) { onLeaf(this.leafIndex[node]); continue; }
            stack[sp++] = this.left[node];
            stack[sp++] = this.right[node];
        }
    }
}

ActionPhysics.BVH = BVH;


// ==== src/phases/SAPBroadphase.js ====
/**
 * Sweep-and-prune broadphase over AABBs, sorted along a single axis.
 *
 * Produces: candidate body pairs, no false negatives (plan.md, Broadphase). May assume AABBs are
 * current - it never recomputes one, only reads body.getBroadphaseAABB() (the fattened, speculative-
 * margin variant, so a pair surfaces the tick before overlap). Must never test actual shapes;
 * the only thing this file knows about a body is its AABB.
 *
 * ONE axis, not three. The classic SAP maintains sorted lists on all three axes and intersects
 * them; a single axis with a good pick (the one with the most spread, recomputed occasionally)
 * gets most of the culling at a third of the bookkeeping. The remaining axes are checked directly
 * per candidate pair below, which is cheap because the axis sort has already thrown out most of
 * the O(n^2) pairs.
 */
class SAPBroadphase {
    constructor() {
        // Sorted by AABB min on the sweep axis. Re-sorted every update() - insertion-sort cost is
        // fine because frame-to-frame the order barely changes (temporal coherence), and an O(n
        // log n) sort with no coherence assumption is simpler to trust than a persistent structure
        // during this stage's first pass.
        this._entries = [];   // { body, aabb } - aabb is a snapshot reference, not a copy
        this._axis = 'x';
    }

    // Body lifecycle - the World is the only expected caller.
    add(body) {
        this._entries.push({ body: body, aabb: body.getBroadphaseAABB() });
    }

    remove(body) {
        for (let i = 0; i < this._entries.length; i++) {
            if (this._entries[i].body === body) { this._entries.splice(i, 1); return; }
        }
    }

    // Re-picks the sweep axis from the current AABB spread. Cheap (O(n)) and only needs to be
    // roughly right - a wrong axis costs candidate-pair quality, never correctness, because every
    // axis is still checked directly on each candidate pair in _sweep().
    _pickAxis() {
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
        for (let i = 0; i < this._entries.length; i++) {
            const c = this._entries[i].aabb;
            if (c.min.x < minX) minX = c.min.x; if (c.max.x > maxX) maxX = c.max.x;
            if (c.min.y < minY) minY = c.min.y; if (c.max.y > maxY) maxY = c.max.y;
            if (c.min.z < minZ) minZ = c.min.z; if (c.max.z > maxZ) maxZ = c.max.z;
        }
        const sx = maxX - minX, sy = maxY - minY, sz = maxZ - minZ;
        this._axis = (sx >= sy && sx >= sz) ? 'x' : (sy >= sz ? 'y' : 'z');
    }

    // Returns candidate pairs as [bodyA, bodyB][], A.id < B.id always, so downstream pairing (a
    // manifold cache keyed by two ids) has one canonical order without the caller sorting again.
    computePairs() {
        const n = this._entries.length;
        const pairs = [];
        if (n < 2) return pairs;

        this._pickAxis();
        const axis = this._axis;
        this._entries.sort(function (a, b) { return a.aabb.min[axis] - b.aabb.min[axis]; });

        for (let i = 0; i < n; i++) {
            const ei = this._entries[i];
            const maxOnAxis = ei.aabb.max[axis];
            for (let j = i + 1; j < n; j++) {
                const ej = this._entries[j];
                // Sorted by min on the sweep axis: once ej's min passes ei's max, no later entry
                // can overlap ei on this axis either (their mins only increase from here).
                if (ej.aabb.min[axis] > maxOnAxis) break;
                if (!ei.aabb.intersects(ej.aabb)) continue; // confirms the other two axes
                const a = ei.body, b = ej.body;
                // Two statics/kinematics never need a contact between each other - nothing can
                // move them into or out of overlap from this pair alone. Filtered here rather
                // than downstream so midphase/narrowphase never see a pair that can't matter.
                if (a.bodyType !== RigidBody.DYNAMIC && b.bodyType !== RigidBody.DYNAMIC) continue;
                if ((a.collision_mask & b.collision_groups) === 0) continue;
                if ((b.collision_mask & a.collision_groups) === 0) continue;
                if (a.id < b.id) pairs.push([a, b]); else pairs.push([b, a]);
            }
        }
        return pairs;
    }
}

ActionPhysics.SAPBroadphase = SAPBroadphase;


// ==== src/phases/Midphase.js ====
/**
 * Midphase: which children of a compound / triangles of a mesh actually need narrowphase?
 *
 * Produces: candidate PRIMITIVE-shape pairs from a broadphase body pair, each candidate carrying
 * the primitive shape plus its WORLD transform (position, rotation) - narrowphase (once it exists)
 * takes these directly, with no compound/mesh awareness of its own. May assume the broadphase pair
 * is real (Rule 1). Must never compute contact data - only "these two primitives might touch".
 *
 * A body's shape is one of three kinds here:
 *   - primitive (Box, Sphere, Convex, ...): the body itself IS the one candidate, no BVH needed.
 *   - CompoundShape: candidates are its children whose world AABB overlaps the other side.
 *   - MeshShape: candidates are its triangles (wrapped as TriangleShape) whose world AABB overlaps.
 *
 * Each CompoundShape/MeshShape gets its own BVH over its children/triangles, built ONCE and cached
 * on the shape instance itself (`shape._midphaseBVH`) - static geometry, built once per plan.md's
 * Spatial section. A shape never rebuilds this even if queried by many different bodies (a mesh
 * asset shared across several static bodies builds its BVH exactly once).
 */
class Midphase {
    constructor() {
        // Static-geometry leaf cache: for a given (shape, queryAABB) the set of leaf indices that
        // overlapped, INCLUDING the empty set. Not caching an empty result was the bug that made
        // every resting body re-walk the BVH and re-run GJK every frame forever (plan.md, Bug
        // reference / Caching) - 80% of all cache checks in that measurement. Keyed by the OTHER
        // body's id, since the query box moves every tick with that body; invalidated once per
        // tick for a moving other-body, kept for a sleeping one (the sleep manager owns eviction
        // once it exists - this cache only ever stores what it's given, never guesses staleness).
        this._leafCache = new Map(); // otherBodyId -> { shape, minx,miny,minz,maxx,maxy,maxz, hits:[leafIndex...] }
    }

    // Clears every cached leaf-walk result. Call this when a static/kinematic compound or mesh
    // body's geometry or transform changes - the cache has no way to know that on its own (it is
    // keyed by the OTHER body, not the static one), matching the "sleeping bodies still get woken
    // explicitly" discipline in plan.md's Sleep section rather than guessing at staleness here.
    invalidate() {
        this._leafCache.clear();
    }

    // Ensures shape._midphaseBVH exists, building it on first use. Compound: one leaf per child.
    // Mesh: one leaf per triangle.
    _ensureBVH(shape) {
        if (shape._midphaseBVH) return shape._midphaseBVH;
        const bvh = new BVH();
        if (shape instanceof CompoundShape) {
            const scratch = new AABB();
            const rotMat = new Matrix3();
            const corner = new Vector3();
            bvh.build(shape.children.length, function (out, i) {
                const child = shape.children[i];
                child.shape.localAABBInto(scratch);
                rotMat.fromQuaternion(child.localRotation);
                out.setEmpty();
                for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
                    corner.x = cx ? scratch.max.x : scratch.min.x;
                    corner.y = cy ? scratch.max.y : scratch.min.y;
                    corner.z = cz ? scratch.max.z : scratch.min.z;
                    rotMat.transformVector3(corner);
                    corner.addInPlace(child.localPosition);
                    if (corner.x < out.min.x) out.min.x = corner.x;
                    if (corner.y < out.min.y) out.min.y = corner.y;
                    if (corner.z < out.min.z) out.min.z = corner.z;
                    if (corner.x > out.max.x) out.max.x = corner.x;
                    if (corner.y > out.max.y) out.max.y = corner.y;
                    if (corner.z > out.max.z) out.max.z = corner.z;
                }
            });
        } else if (shape instanceof MeshShape) {
            const a = new Vector3(), b = new Vector3(), c = new Vector3();
            bvh.build(shape.triangleCount, function (out, i) {
                shape.triangleAt(i, a, b, c);
                out.setEmpty();
                out.min.x = Math.min(a.x, b.x, c.x); out.max.x = Math.max(a.x, b.x, c.x);
                out.min.y = Math.min(a.y, b.y, c.y); out.max.y = Math.max(a.y, b.y, c.y);
                out.min.z = Math.min(a.z, b.z, c.z); out.max.z = Math.max(a.z, b.z, c.z);
            });
        }
        shape._midphaseBVH = bvh;
        return bvh;
    }

    // Leaf indices of `shape` (a Compound or Mesh) whose LOCAL-space AABB overlaps `localQueryAABB`
    // (already expressed in the shape's own local space by the caller). Cached per (shape, other
    // body) — an identical query next tick for a body that hasn't moved returns the same box and
    // hits the cache; a different box invalidates and re-walks.
    _queryLeaves(shape, otherBodyId, localQueryAABB) {
        const cached = this._leafCache.get(otherBodyId);
        if (cached && cached.shape === shape &&
            cached.minx === localQueryAABB.min.x && cached.miny === localQueryAABB.min.y && cached.minz === localQueryAABB.min.z &&
            cached.maxx === localQueryAABB.max.x && cached.maxy === localQueryAABB.max.y && cached.maxz === localQueryAABB.max.z) {
            return cached.hits; // may be [] - an empty result is a valid, cached answer
        }
        const bvh = this._ensureBVH(shape);
        const hits = [];
        bvh.query(localQueryAABB, function (i) { hits.push(i); });
        this._leafCache.set(otherBodyId, {
            shape: shape,
            minx: localQueryAABB.min.x, miny: localQueryAABB.min.y, minz: localQueryAABB.min.z,
            maxx: localQueryAABB.max.x, maxy: localQueryAABB.max.y, maxz: localQueryAABB.max.z,
            hits: hits
        });
        return hits;
    }

    // Expands one side of a broadphase pair into primitive candidates: [{ shape, position,
    // rotation }]. `otherAABB` is the other body's WORLD AABB, used to cull compound children /
    // mesh triangles that can't possibly matter. `otherBodyId` keys the leaf cache (see above).
    _expandSide(body, otherAABB, otherBodyId) {
        const shape = body.shape;
        if (!(shape instanceof CompoundShape) && !(shape instanceof MeshShape)) {
            return [{ shape: shape, position: body.position, rotation: body.rotation }];
        }

        // Bring the other body's world AABB into this body's LOCAL space by inverse-transforming
        // its 8 corners - conservative (may over-include), never under-includes, matching
        // broadphase's own no-false-negatives contract one level down.
        const invRot = Midphase._scratchQuat.copy(body.rotation).invert();
        const localQuery = Midphase._scratchAABB.setEmpty();
        const corner = Midphase._scratchVec;
        for (let cx = 0; cx < 2; cx++) for (let cy = 0; cy < 2; cy++) for (let cz = 0; cz < 2; cz++) {
            corner.x = cx ? otherAABB.max.x : otherAABB.min.x;
            corner.y = cy ? otherAABB.max.y : otherAABB.min.y;
            corner.z = cz ? otherAABB.max.z : otherAABB.min.z;
            corner.subInPlace(body.position);
            invRot.transformVectorInPlace(corner);
            if (corner.x < localQuery.min.x) localQuery.min.x = corner.x;
            if (corner.y < localQuery.min.y) localQuery.min.y = corner.y;
            if (corner.z < localQuery.min.z) localQuery.min.z = corner.z;
            if (corner.x > localQuery.max.x) localQuery.max.x = corner.x;
            if (corner.y > localQuery.max.y) localQuery.max.y = corner.y;
            if (corner.z > localQuery.max.z) localQuery.max.z = corner.z;
        }

        const hits = this._queryLeaves(shape, otherBodyId, localQuery);
        const out = [];
        if (shape instanceof CompoundShape) {
            for (let k = 0; k < hits.length; k++) {
                const child = shape.children[hits[k]];
                // World transform = body transform composed with the child's local offset.
                const worldPos = new Vector3();
                body.rotation.transformVectorInto(child.localPosition, worldPos);
                worldPos.addInPlace(body.position);
                const worldRot = new Quaternion().multiplyQuaternions(body.rotation, child.localRotation);
                out.push({ shape: child.shape, position: worldPos, rotation: worldRot });
            }
        } else {
            const a = new Vector3(), b = new Vector3(), c = new Vector3();
            for (let k = 0; k < hits.length; k++) {
                shape.triangleAt(hits[k], a, b, c);
                // Triangle vertices are baked into world space directly, at body identity rotation -
                // simpler than carrying a per-triangle local frame narrowphase would have to undo.
                const wa = new Vector3(), wb = new Vector3(), wc = new Vector3();
                body.rotation.transformVectorInto(a, wa); wa.addInPlace(body.position);
                body.rotation.transformVectorInto(b, wb); wb.addInPlace(body.position);
                body.rotation.transformVectorInto(c, wc); wc.addInPlace(body.position);
                out.push({ shape: new TriangleShape(wa, wb, wc), position: new Vector3(0, 0, 0), rotation: new Quaternion() });
            }
        }
        return out;
    }

    // Expands a broadphase [bodyA, bodyB] pair into primitive x primitive candidates:
    // [{ a: {shape,position,rotation}, b: {shape,position,rotation} }, ...]
    // Never computes contact data (Rule 1) - purely a cross-product of the two expansions.
    expandPair(bodyA, bodyB) {
        // The fattened broadphase AABB (speculative margin included) is used for child/triangle
        // culling too, so a compound child or mesh triangle that a body is about to reach is
        // surfaced the same tick early as the body pair itself - conservative (over-includes, never
        // under-includes), matching broadphase's own no-false-negatives contract one level down.
        const sidesA = this._expandSide(bodyA, bodyB.getBroadphaseAABB(), bodyB.id);
        const sidesB = this._expandSide(bodyB, bodyA.getBroadphaseAABB(), bodyA.id);
        const out = [];
        for (let i = 0; i < sidesA.length; i++) {
            for (let j = 0; j < sidesB.length; j++) {
                out.push({ a: sidesA[i], b: sidesB[j] });
            }
        }
        return out;
    }
}

Midphase._scratchQuat = new Quaternion();
Midphase._scratchAABB = new AABB();
Midphase._scratchVec = new Vector3();

ActionPhysics.Midphase = Midphase;


// ==== src/collision/MinkowskiSupport.js ====
/**
 * World-space support function over the Minkowski DIFFERENCE of two placed shapes (A - B), the one
 * primitive GJK/EPA need. Each side is a { shape, position, rotation } as produced by Midphase.
 *
 * A shape's own supportInto() works in LOCAL space. To support a placed shape in world space along
 * world direction d: rotate d into local space (inverse rotation), call supportInto, rotate the
 * result back to world space, translate by position. Minkowski difference support along d is then
 * supportA(d) - supportB(-d) - the standard GJK primitive.
 *
 * Allocation-free: every method writes into a caller-owned `out`. A single instance is built per
 * narrowphase pair and reused across every GJK/EPA iteration for that pair.
 */
class MinkowskiSupport {
    constructor(placedA, placedB) {
        this.a = placedA;
        this.b = placedB;
        this._localDir = new Vector3();
        this._invRotA = new Quaternion();
        this._invRotB = new Quaternion();
        this._invRotA.copy(placedA.rotation).invert();
        this._invRotB.copy(placedB.rotation).invert();
        // Per-instance scratch (plan.md: "scratch memory: per-stage arenas, never global") - EPA
        // calls supportInto() while GJK's own loop may still be holding references into a prior
        // call's result, so a SHARED scratch across instances (or across nested calls) would
        // silently corrupt whichever call didn't finish first.
        this._scratchA = new Vector3();
        this._scratchB = new Vector3();
        this._scratchNeg = new Vector3();
    }

    // Re-derive the cached inverse rotations if a placed side's rotation object was mutated after
    // construction (narrowphase reuses one MinkowskiSupport across substeps for a persistent pair).
    refresh() {
        this._invRotA.copy(this.a.rotation).invert();
        this._invRotB.copy(this.b.rotation).invert();
        return this;
    }

    // World-space support of ONE placed side along world direction `dir`.
    static supportOfInto(out, placed, invRot, dir, scratchDir) {
        invRot.transformVectorInto(dir, scratchDir);
        placed.shape.supportInto(out, scratchDir);
        placed.rotation.transformVectorInPlace(out);
        out.addInPlace(placed.position);
        return out;
    }

    // out = supportA(dir) - supportB(-dir), in world space. Also writes the two world support
    // points into outA/outB if given (EPA needs them per Minkowski-difference vertex, to recover
    // the actual contact points once the penetrating simplex face is known).
    supportInto(out, dir, outA, outB) {
        const sa = outA || this._scratchA;
        const sb = outB || this._scratchB;
        MinkowskiSupport.supportOfInto(sa, this.a, this._invRotA, dir, this._localDir);
        const negDir = this._scratchNeg.set(-dir.x, -dir.y, -dir.z);
        MinkowskiSupport.supportOfInto(sb, this.b, this._invRotB, negDir, this._localDir);
        Vector3.subInto(out, sa, sb);
        return out;
    }
}

ActionPhysics.MinkowskiSupport = MinkowskiSupport;


// ==== src/collision/GJK.js ====
/**
 * GJK: distance / overlap test between two convex shapes via their Minkowski difference.
 *
 * Written from the algorithm (Ericson, "Real-Time Collision Detection", ch. 5; the original
 * Gilbert-Johnson-Keerthi paper), not from a prior implementation - plan.md, "This is a rebuild,
 * not a port": narrowphase is the highest-risk component and the one place a structural anchor to
 * the predecessor would be worst to carry over.
 *
 * Two outcomes, both from the SAME loop:
 *   OVERLAPPING - the simplex encloses the origin. Returns the simplex as-is (2-4 points) for EPA
 *                 to expand into a penetration depth/normal. GJK itself does not compute depth.
 *   SEPARATED   - the loop converges on the closest points between the two hulls. Returns exact
 *                 distance, witness points on A and B, and a separating normal. This is what
 *                 speculative contacts run on: the query happens against a PREDICTED position
 *                 (Solver's job, not GJK's), and this result is used directly as a positive-
 *                 separation contact rather than triggering a second penetrating pass.
 *
 * BUG FIX CARRIED FROM THE PREDECESSOR (plan.md, Bug reference / Collision detection):
 * "Flush shapes reported no contact at all." Two shapes resting exactly touching (or a simplex that
 * degenerates during the walk - three collinear points, a zero-area triangle) produced NaN
 * barycentric coordinates and the contact was silently discarded. The fix here is structural: every
 * point-in-simplex / closest-point routine below has an EXPLICIT fallback for the degenerate case
 * (zero-length edge, zero-area triangle) that returns a valid geometric answer instead of NaN -
 * never a discard. See the `_degenerate...` fallback branches.
 *
 * EXACT-TOUCHING IS UNDECIDABLE FROM A LOWER-DIMENSIONAL SIMPLEX ALONE. When a 2-point or 3-point
 * simplex reduces to a closest point that IS the origin exactly, that is genuinely ambiguous with
 * only that simplex to look at: it is what a flush touch looks like (two boxes with coincident
 * faces produce a Minkowski-difference boundary the origin sits exactly on), but it is ALSO what a
 * lower-dimensional cross-section of a real 3D overlap looks like on the way to an enclosing
 * tetrahedron - a touching pair still has real geometric extent perpendicular to the touching plane
 * (a box has depth), so growing the simplex incrementally from a single starting direction finds
 * "progress" in both cases alike and can get stuck reporting the touching plane forever without
 * ever discovering a real enclosing tetrahedron on the other side of it. This is a well-known hard
 * case in GJK implementations generally (see e.g. the Signed Volumes / Montanari et al. distance
 * subalgorithm literature for a fully robust treatment).
 *
 * THE RESOLUTION USED HERE: seed several tetrahedra directly from diverse, non-coplanar probe
 * direction SETS (see SEED_DIRECTION_SETS / _seedTetrahedron) instead of growing one incrementally.
 * A real overlap's enclosing tetrahedron shows up with every face at a genuine NEGATIVE margin once
 * built from directions that are not all confined to the touching plane; an exact touch never
 * produces a fully-negative-margin tetrahedron no matter which diverse directions are tried,
 * because the origin genuinely sits on the true Minkowski-difference boundary there. If no seed
 * encloses, its best (closest-to-origin) reduction seeds the incremental fallback loop, which
 * handles the ordinary separated case and the terminal exact-touch report.
 *
 * KNOWN LIMITATION: a sparse, symmetric convex hull (e.g. an 8-vertex octahedron, ConvexShape's
 * support function ties between exactly 6 possible outputs) queried at EXACT axis-aligned
 * coincidence with an identical copy of itself can still evade every seed direction, because the
 * shape's support function only ever returns one of a handful of fixed points regardless of probe
 * direction diversity - unlike Box/Sphere/Cylinder/Capsule, whose support functions vary smoothly
 * or have enough distinct extremes that this does not occur. Any non-exact offset (even a tiny
 * fractional one) resolves correctly; this is a pathological, near-zero-probability configuration
 * for a live simulation (bodies do not spawn perfectly axis-coincident), not treated as fixed here.
 */
class GJK {
    constructor() {
        // Simplex: up to 4 points, each a { w: Vector3 (Minkowski diff point), a: Vector3 (world
        // point on shape A), b: Vector3 (world point on shape B) }. Kept as parallel scratch
        // arrays rather than objects, allocation-free across iterations.
        this._wx = new Float64Array(4); this._wy = new Float64Array(4); this._wz = new Float64Array(4);
        this._ax = new Float64Array(4); this._ay = new Float64Array(4); this._az = new Float64Array(4);
        this._bx = new Float64Array(4); this._by = new Float64Array(4); this._bz = new Float64Array(4);
        this._count = 0;

        this._dir = new Vector3();
        this._newW = new Vector3();
        this._newA = new Vector3();
        this._newB = new Vector3();
        this._closest = new Vector3();
        this._scratchRef = new Vector3();
        this._initialProbeDir = new Vector3();
        this._newDir4 = new Vector3();
        this._probeDir = new Vector3();  // scratch for the strict-interior penetration probe
        this._probeW = new Vector3();
    }

    _clear() { this._count = 0; }

    _push(w, a, b) {
        const i = this._count++;
        this._wx[i] = w.x; this._wy[i] = w.y; this._wz[i] = w.z;
        this._ax[i] = a.x; this._ay[i] = a.y; this._az[i] = a.z;
        this._bx[i] = b.x; this._by[i] = b.y; this._bz[i] = b.z;
    }

    // Replace the simplex with exactly the points at the given source indices (in that order),
    // used after each closest-feature reduction. Safe because sources are always <= current count
    // and we write low-to-high after reading everything we need for this call already.
    _reduceTo(indices) {
        const wx = this._wx.slice(), wy = this._wy.slice(), wz = this._wz.slice();
        const ax = this._ax.slice(), ay = this._ay.slice(), az = this._az.slice();
        const bx = this._bx.slice(), by = this._by.slice(), bz = this._bz.slice();
        for (let k = 0; k < indices.length; k++) {
            const src = indices[k];
            this._wx[k] = wx[src]; this._wy[k] = wy[src]; this._wz[k] = wz[src];
            this._ax[k] = ax[src]; this._ay[k] = ay[src]; this._az[k] = az[src];
            this._bx[k] = bx[src]; this._by[k] = by[src]; this._bz[k] = bz[src];
        }
        this._count = indices.length;
    }

    /**
     * Runs GJK for the pair of placed shapes wrapped by `support` (a MinkowskiSupport). Returns:
     *   { overlapping: true,  simplex: this }                                   — hand to EPA
     *   { overlapping: false, distance, normal, pointA, pointB }                — separated
     * `normal` points from B to A (world space), matching the Narrowphase contract's convention.
     * maxIterations guards against non-convergence on a pathological input; hitting it returns the
     * best answer found so far rather than a wrong one (same discipline as EPA's converged-result
     * rule below - never return "whatever the last iteration produced" as if it were final without
     * saying so isn't possible here, so we return the best-so-far honestly, not silently).
     */
    run(support, maxIterations) {
        maxIterations = maxIterations || 64;
        this._clear();

        // Seed a tetrahedron directly from four directions spanning genuinely different octants,
        // rather than growing one incrementally from a single starting axis. This is what actually
        // distinguishes real 3D overlap from exact touching (see the class header): incremental
        // growth from one starting direction tends to walk INTO the exact-touching plane and get
        // stuck there. Several diverse direction sets are tried (see SEED_DIRECTION_SETS) since any
        // single fixed set can, for particular shape sizes, itself produce a degenerate tetrahedron
        // by unlucky alignment.
        //
        // If no seed set immediately encloses the origin, the BEST set's own reduction (whichever
        // came closest to the origin, whatever triangle/direction it settled on) becomes the
        // STARTING point for the incremental loop
        // below, rather than restarting from a fresh single-point simplex. This matters: a real
        // overlap whose seed tetrahedron narrowly misses the origin is still much closer to
        // resolving from that reduced simplex than from scratch - starting over from a single
        // arbitrary axis is what let real overlaps fall through to a false "touching" report before
        // this chaining was added (a lower-dimensional simplex from a fresh start hits the same
        // degenerate-plane trap the seed exists to avoid in the first place).
        let seeded = this._seedTetrahedron(support);
        if (seeded.overlapping) return seeded;
        this._dir.copy(seeded.direction);
        this._closest.copy(seeded.closest);

        if (this._dir.lengthSquared() < 1e-20) {
            // Every seed (and its own reduction) settled on the origin exactly: the origin lies ON
            // the Minkowski difference boundary. That is EITHER an exact touch (depth 0, SEPARATED by
            // the pipeline's convention) OR a shallow penetration the seed tetrahedra could not
            // enclose for this shape-size ratio (a small sphere on a large box, OVERLAPPING). The
            // simplex alone cannot tell them apart, so probe: the origin is STRICTLY interior (real
            // penetration) iff every probe direction's support extends strictly past it. If so, it is
            // overlapping and EPA measures the depth; otherwise it is an exact touch, reported
            // separated at distance 0 (never NaN - the documented flush-contact discipline).
            return this._originStrictlyInside(support) ? { overlapping: true, simplex: this } : this._separatedResult(support);
        }

        for (let iter = 0; iter < maxIterations; iter++) {
            support.supportInto(this._newW, this._dir, this._newA, this._newB);

            // Standard GJK termination (Ericson 5.4): the new support point makes no progress in
            // the search direction if it does not project further along `dir` than the simplex
            // points already do.
            const newAlong = this._newW.x * this._dir.x + this._newW.y * this._dir.y + this._newW.z * this._dir.z;
            let bestAlong = -Infinity;
            for (let k = 0; k < this._count; k++) {
                const along = this._wx[k] * this._dir.x + this._wy[k] * this._dir.y + this._wz[k] * this._dir.z;
                if (along > bestAlong) bestAlong = along;
            }
            if (newAlong <= bestAlong + 1e-10) {
                // Stall: the support makes no further progress toward the origin. Normally this means
                // separated - but it ALSO fires when the origin is already ON or just inside the
                // Minkowski difference and the search direction has collapsed toward zero length
                // (closest point ~= origin). Those are opposite conclusions. Disambiguate by the
                // closest DISTANCE: a genuinely separated pair has a POSITIVE gap here, so a closest
                // distance of ~0 means the origin is enclosed/touching = OVERLAPPING, and the current
                // simplex is handed to EPA to extract depth. Without this, a small sphere shallowly
                // penetrating a much larger box (whose seed tetrahedron does not enclose the origin
                // for that size ratio, and whose incremental walk then stalls at the face) was
                // reported SEPARATED with distance 0 - so EPA never ran, the penetration went
                // uncorrected, and the one-shot fix a tick later launched the body. The threshold is
                // a length, well below any contact depth the solver resolves but far above the
                // ~1e-8 numerical floor of a true closest-point-at-origin.
                const closestDistSq = this._closest.x * this._closest.x + this._closest.y * this._closest.y + this._closest.z * this._closest.z;
                if (closestDistSq < GJK.OVERLAP_DISTANCE_EPSILON * GJK.OVERLAP_DISTANCE_EPSILON) {
                    // Origin ~= on the boundary: real penetration (strictly inside) or exact touch.
                    return this._originStrictlyInside(support) ? { overlapping: true, simplex: this } : this._separatedResult(support);
                }
                return this._separatedResult(support);
            }

            this._push(this._newW, this._newA, this._newB);

            const result = this._doSimplex();
            if (result.containsOrigin) {
                return { overlapping: true, simplex: this };
            }
            this._dir.copy(result.direction);
            this._closest.copy(result.closest);

            // Search direction collapsed to zero: origin on the current simplex - touch or
            // penetration. Discriminate by strict interiority, same as the post-seed guard above.
            if (this._dir.lengthSquared() < 1e-20) {
                return this._originStrictlyInside(support) ? { overlapping: true, simplex: this } : this._separatedResult(support);
            }
        }
        // Iteration cap reached without a clean termination. This mirrors EPA's own rule below
        // (plan.md, Bug reference: "return the converged result, never the last one") - the
        // current simplex IS the best convergence reached, so report it honestly as SEPARATED
        // rather than pretending overlap or discarding.
        return this._separatedResult(support);
    }

    // Several sets of four directions, each spanning genuinely different octants, used to seed a
    // tetrahedron directly rather than growing one incrementally from a single starting axis (see
    // run()'s call site for why that matters). Multiple rotated sets exist because ANY single
    // fixed set can, for a particular pair of shape sizes/positions, happen to produce a seed
    // tetrahedron that is itself degenerate (its own 4 support points landing coplanar/collinear -
    // an unlucky alignment between the probe directions and the shapes' geometry, not a boundary-
    // touching case) - trying a second, differently-rotated set resolves that without having to
    // detect the degeneracy explicitly.
    // Deliberately irrational-ish, non-axis-aligned components (not just +-1) - a sparse convex
    // hull (e.g. an 8-vertex octahedron) has its support function tied between vertices exactly on
    // clean +-1 diagonals, which for two IDENTICAL coincident hulls collapses two of the four seed
    // points to duplicates and reproduces the same "two faces exactly through the origin" pattern
    // as the touching case - a real, deep overlap misreported as not-enclosing. Odd-looking
    // component ratios make an exact tie between two support vertices astronomically unlikely for
    // any shape that isn't specifically axis-aligned-symmetric along that exact ratio.
    // Closest-distance below which a STALLED incremental walk is treated as overlapping rather than
    // separated (see the stall branch in run()). A length: comfortably below any real contact depth
    // the solver resolves, comfortably above the numerical noise of a true closest-point-at-origin.
    static OVERLAP_DISTANCE_EPSILON = 1e-5;

    // Fixed probe directions for the strict-interior penetration test (_originStrictlyInside). The 6
    // signed axes span every face normal of an axis-aligned contact; the search direction is probed
    // separately (both signs) for oblique contacts. Enough coverage that an exact touch always
    // exposes its zero-margin separating direction, cheap enough to run only at the rare collapse.
    static INTERIOR_PROBE_DIRS = [
        [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]
    ];

    static SEED_DIRECTION_SETS = [
        [[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]],
        [[1, 1, -1], [1, -1, 1], [-1, 1, 1], [-1, -1, -1]],
        [[1, 0, 0], [-1, 0.3, 0.3], [0, -1, 0.3], [0, 0.3, -1]],
        [[0.8763, 0.2451, 0.4127], [0.3312, -0.9021, 0.2734], [-0.6543, 0.1298, -0.7452], [-0.5532, -0.6789, 0.4821]]
    ];

    // Tries each seed direction set in turn, building a tetrahedron and checking whether it
    // encloses the origin with a real (non-degenerate) margin.
    //
    // Returns { overlapping: true, simplex: this } on a confirmed overlap (this.wx/wy/wz etc. hold
    // the enclosing simplex, ready for EPA). Otherwise returns { overlapping: false, direction,
    // closest } from whichever seed's own reduction got CLOSEST to the origin - not just the last
    // one tried - so the caller's incremental loop continues from the best available starting
    // point rather than an arbitrary one. The simplex left in this._wx/wy/wz on return matches
    // that best reduction (re-run once more below to restore it, since each seed attempt
    // overwrites the shared arrays in turn).
    _seedTetrahedron(support) {
        let bestDistSq = Infinity, bestSet = -1;
        for (let s = 0; s < GJK.SEED_DIRECTION_SETS.length; s++) {
            this._clear();
            const dirs = GJK.SEED_DIRECTION_SETS[s];
            for (let i = 0; i < 4; i++) {
                const d = dirs[i];
                this._newDir4.set(d[0], d[1], d[2]);
                support.supportInto(this._newW, this._newDir4, this._newA, this._newB);
                this._push(this._newW, this._newA, this._newB);
            }
            const result = this._simplexTetrahedron();
            if (result.containsOrigin) return { overlapping: true, simplex: this };
            const distSq = result.closest.x * result.closest.x + result.closest.y * result.closest.y + result.closest.z * result.closest.z;
            if (distSq < bestDistSq) { bestDistSq = distSq; bestSet = s; }
        }
        // Re-run the best set to restore its simplex/direction/closest as the live state -
        // cheap (4 support queries) relative to how rarely this whole path (seed-doesn't-enclose)
        // is hit, and keeps _seedTetrahedron's inner loop simple rather than snapshotting state
        // for every candidate on every iteration.
        this._clear();
        const bestDirs = GJK.SEED_DIRECTION_SETS[bestSet];
        for (let i = 0; i < 4; i++) {
            const d = bestDirs[i];
            this._newDir4.set(d[0], d[1], d[2]);
            support.supportInto(this._newW, this._newDir4, this._newA, this._newB);
            this._push(this._newW, this._newA, this._newB);
        }
        const finalResult = this._simplexTetrahedron();
        return { overlapping: false, direction: finalResult.direction, closest: finalResult.closest };
    }

    // True iff the origin is STRICTLY inside the Minkowski difference (real penetration), false if it
    // merely lies on the boundary (exact touch). The support function reaches farthest along any
    // direction d to the point with the largest d.w; the origin is strictly interior iff that
    // farthest extent is strictly positive along EVERY direction (support(d).d > margin for all d).
    // At an exact touch there is a separating direction (the contact normal) where support(d).d is
    // ~0 - the shapes meet exactly there with no overlap to spare. Probing a fixed spread of
    // directions plus, crucially, both signs of the last search direction (which points along the
    // near-degenerate contact normal) reliably catches that zero. The margin is a small absolute
    // extent, well below any contact depth the solver cares about but above numerical touch noise.
    _originStrictlyInside(support) {
        const margin = GJK.OVERLAP_DISTANCE_EPSILON;
        // The collapsed search direction is (numerically) along the contact normal - the most likely
        // separating axis - so test it first, both signs.
        if (this._closest.lengthSquared() > 1e-20) {
            const l = Math.sqrt(this._closest.lengthSquared());
            this._probeDir.set(this._closest.x / l, this._closest.y / l, this._closest.z / l);
            if (!this._supportExceeds(support, this._probeDir, margin)) return false;
            this._probeDir.set(-this._probeDir.x, -this._probeDir.y, -this._probeDir.z);
            if (!this._supportExceeds(support, this._probeDir, margin)) return false;
        }
        for (let a = 0; a < GJK.INTERIOR_PROBE_DIRS.length; a++) {
            const d = GJK.INTERIOR_PROBE_DIRS[a];
            this._probeDir.set(d[0], d[1], d[2]);
            if (!this._supportExceeds(support, this._probeDir, margin)) return false;
        }
        return true;
    }

    // support(dir).dir > margin ? (dir need not be unit; margin is scaled by |dir| so the comparison
    // is a true distance along dir.)
    _supportExceeds(support, dir, margin) {
        support.supportInto(this._probeW, dir);
        const along = this._probeW.x * dir.x + this._probeW.y * dir.y + this._probeW.z * dir.z;
        const len = Math.sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        return along > margin * len;
    }

    // Builds the SEPARATED return value from the current simplex's closest point to the origin.
    // The witness points (pointA, pointB) are recovered via the simplex's barycentric weights
    // applied to the stored world support points - never re-queried, so they are exactly consistent
    // with the reported distance.
    // `forcedNormal`, if given, is used directly instead of deriving one from `this._closest` -
    // only the immediate "first support point IS the origin" case in run() passes this, since
    // there the probe direction itself is the exact, known-correct contact normal.
    _separatedResult(support, forcedNormal) {
        const bary = this._barycentricOfClosest();
        const pointA = new Vector3(), pointB = new Vector3();
        for (let i = 0; i < this._count; i++) {
            pointA.x += bary[i] * this._ax[i]; pointA.y += bary[i] * this._ay[i]; pointA.z += bary[i] * this._az[i];
            pointB.x += bary[i] * this._bx[i]; pointB.y += bary[i] * this._by[i]; pointB.z += bary[i] * this._bz[i];
        }
        const dist = Math.sqrt(this._closest.x * this._closest.x + this._closest.y * this._closest.y + this._closest.z * this._closest.z);
        let normal;
        if (forcedNormal) {
            normal = new Vector3().copy(forcedNormal).normalizeInPlace();
        } else if (dist > 1e-12) {
            normal = new Vector3(this._closest.x / dist, this._closest.y / dist, this._closest.z / dist);
        } else {
            // Exact touching: the origin lies ON the simplex, so `closest` itself carries no
            // direction. This is the degenerate case the flush-contact fix (plan.md, Bug
            // reference) exists for - fall back to a normal recoverable from the simplex's own
            // geometry rather than a fixed axis, which would be wrong for a touching pair whose
            // true contact plane isn't horizontal.
            normal = new Vector3();
            this._degenerateTouchingNormalInto(normal);
        }
        return { overlapping: false, distance: dist, normal: normal, pointA: pointA, pointB: pointB };
    }

    // Recovers a normal for a zero-distance (exact touching) simplex. A 3-point simplex through
    // the origin has a well-defined plane normal - use it. A 2-point (segment through the origin)
    // or 1-point simplex has no unique perpendicular in 3D, so fall back to the shared
    // findOrthogonal() convention (least-aligned cardinal axis) rather than inventing one - this
    // never produces NaN, matching the bug-reference discipline, even though it is not always the
    // physically true contact normal for that degenerate case.
    _degenerateTouchingNormalInto(out) {
        if (this._count === 3) {
            const abx = this._wx[1] - this._wx[0], aby = this._wy[1] - this._wy[0], abz = this._wz[1] - this._wz[0];
            const acx = this._wx[2] - this._wx[0], acy = this._wy[2] - this._wy[0], acz = this._wz[2] - this._wz[0];
            out.x = aby * acz - abz * acy; out.y = abz * acx - abx * acz; out.z = abx * acy - aby * acx;
            const lenSq = out.x * out.x + out.y * out.y + out.z * out.z;
            if (lenSq > 1e-20) { out.scaleInPlace(1 / Math.sqrt(lenSq)); return; }
        }
        this._scratchRef.set(this._wx[0], this._wy[0], this._wz[0]);
        out.findOrthogonal(this._scratchRef);
    }

    // Barycentric weights of `this._closest` with respect to the current simplex (1, 2, or 3
    // points - a 4-point simplex never reaches here because a tetrahedron enclosing the origin
    // returns containsOrigin=true in _doSimplex before this is called). Degenerate simplices (a
    // zero-length edge, a zero-area triangle) fall back explicitly rather than dividing by zero -
    // this IS the flush-contact fix (plan.md, Bug reference).
    _barycentricOfClosest() {
        if (this._count === 1) return [1];
        if (this._count === 2) {
            const abx = this._wx[1] - this._wx[0], aby = this._wy[1] - this._wy[0], abz = this._wz[1] - this._wz[0];
            const lenSq = abx * abx + aby * aby + abz * abz;
            if (lenSq < 1e-20) return [1, 0]; // degenerate (coincident points): fall back to the first
            const apx = this._closest.x - this._wx[0], apy = this._closest.y - this._wy[0], apz = this._closest.z - this._wz[0];
            let t = (apx * abx + apy * aby + apz * abz) / lenSq;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            return [1 - t, t];
        }
        // count === 3: barycentric of a point already known to be IN the triangle's plane (it came
        // from _closestOnTriangle), via the standard area-ratio method.
        const v0x = this._wx[1] - this._wx[0], v0y = this._wy[1] - this._wy[0], v0z = this._wz[1] - this._wz[0];
        const v1x = this._wx[2] - this._wx[0], v1y = this._wy[2] - this._wy[0], v1z = this._wz[2] - this._wz[0];
        const v2x = this._closest.x - this._wx[0], v2y = this._closest.y - this._wy[0], v2z = this._closest.z - this._wz[0];
        const d00 = v0x * v0x + v0y * v0y + v0z * v0z;
        const d01 = v0x * v1x + v0y * v1y + v0z * v1z;
        const d11 = v1x * v1x + v1y * v1y + v1z * v1z;
        const d20 = v2x * v0x + v2y * v0y + v2z * v0z;
        const d21 = v2x * v1x + v2y * v1y + v2z * v1z;
        const denom = d00 * d11 - d01 * d01;
        if (Math.abs(denom) < 1e-20) return [1 / 3, 1 / 3, 1 / 3]; // degenerate (collinear/zero-area): even split, never NaN
        const v = (d11 * d20 - d01 * d21) / denom;
        const w = (d00 * d21 - d01 * d20) / denom;
        const u = 1 - v - w;
        return [u, v, w];
    }

    // Reduces the simplex to its closest feature to the origin and reports whether the origin is
    // enclosed. Dispatches by current point count (line/triangle/tetrahedron), each with an
    // explicit degenerate fallback - see the header note on the flush-contact bug.
    _doSimplex() {
        if (this._count === 2) return this._simplexLine();
        if (this._count === 3) return this._simplexTriangle();
        return this._simplexTetrahedron();
    }

    // Closest point on segment AB to the origin. Degenerate (coincident A/B) falls back to A
    // rather than a 0/0 division - the 2-point half of the flush-contact fix.
    _simplexLine() {
        const ax = this._wx[0], ay = this._wy[0], az = this._wz[0];
        const bx = this._wx[1], by = this._wy[1], bz = this._wz[1];
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const lenSq = abx * abx + aby * aby + abz * abz;

        let t;
        if (lenSq < 1e-20) t = 0;
        else {
            t = -(ax * abx + ay * aby + az * abz) / lenSq;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
        }

        const closest = new Vector3(ax + abx * t, ay + aby * t, az + abz * t);
        if (t <= 0) this._reduceTo([0]);
        else if (t >= 1) this._reduceTo([1]);
        // else both points stay: closest feature is the segment's interior.

        // If the origin lies exactly on the segment, `direction` comes back (0,0,0) - see the
        // class header on why run() reports that directly as a zero-distance touch.
        const dir = new Vector3(-closest.x, -closest.y, -closest.z);
        return { containsOrigin: false, direction: dir, closest: closest };
    }

    _simplexTriangle() {
        const ax = this._wx[0], ay = this._wy[0], az = this._wz[0];
        const bx = this._wx[1], by = this._wy[1], bz = this._wz[1];
        const cx = this._wx[2], cy = this._wy[2], cz = this._wz[2];
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const acx = cx - ax, acy = cy - ay, acz = cz - az;
        // Triangle normal via cross(ab, ac).
        const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        const nLenSq = nx * nx + ny * ny + nz * nz;

        if (nLenSq < 1e-20) {
            // Degenerate: three (near-)collinear points, zero-area triangle. THIS is the flush-
            // contact bug from plan.md - fixed by falling back to the closest of the three edges
            // explicitly instead of discarding. Try each edge as a 2-point simplex and keep the
            // best (closest-to-origin) result.
            return this._degenerateTriangleFallback();
        }

        const closest = GJK._closestPointOnTriangleToOrigin(ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz, nLenSq);
        // If the origin lies exactly on this triangle (face, edge, or vertex), `dir` comes back
        // (0,0,0) - see the class header on why run() reports that directly as a zero-distance
        // touch rather than escalating.
        const dir = new Vector3(-closest.x, -closest.y, -closest.z);

        if (closest.onEdge !== null) {
            // Closest feature is an edge or vertex - reduce the simplex accordingly.
            this._reduceTo(closest.onEdge);
        }
        return { containsOrigin: false, direction: dir, closest: new Vector3(closest.x, closest.y, closest.z) };
    }

    // Degenerate-triangle fallback: pick whichever of the three edges (as a 2-point simplex) is
    // truly closest to the origin, by testing each directly. Never returns NaN, never discards.
    _degenerateTriangleFallback() {
        const pts = [
            [this._wx[0], this._wy[0], this._wz[0]],
            [this._wx[1], this._wy[1], this._wz[1]],
            [this._wx[2], this._wy[2], this._wz[2]]
        ];
        const edges = [[0, 1], [1, 2], [2, 0]];
        let best = null, bestDistSq = Infinity, bestIdx = null;
        for (let e = 0; e < 3; e++) {
            const p0 = pts[edges[e][0]], p1 = pts[edges[e][1]];
            const abx = p1[0] - p0[0], aby = p1[1] - p0[1], abz = p1[2] - p0[2];
            const lenSq = abx * abx + aby * aby + abz * abz;
            let t = lenSq < 1e-20 ? 0 : (-(p0[0] * abx + p0[1] * aby + p0[2] * abz) / lenSq);
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const cx = p0[0] + abx * t, cy = p0[1] + aby * t, cz = p0[2] + abz * t;
            const dSq = cx * cx + cy * cy + cz * cz;
            if (dSq < bestDistSq) { bestDistSq = dSq; best = [cx, cy, cz]; bestIdx = t <= 0 ? [edges[e][0]] : (t >= 1 ? [edges[e][1]] : edges[e]); }
        }
        this._reduceTo(bestIdx);
        const dir = new Vector3(-best[0], -best[1], -best[2]);
        return { containsOrigin: false, direction: dir, closest: new Vector3(best[0], best[1], best[2]) };
    }

    _simplexTetrahedron() {
        // Test the origin against each of the four faces. If it's outside a face (on the far side
        // from the fourth point), the closest feature is on/beyond that face - reduce to a triangle
        // and recurse into the triangle case. If the origin is on the inside of all four faces, it
        // is enclosed - GJK terminates with overlap.
        // The 4 distinct faces of tetrahedron {0,1,2,3}, each listed with its opposite vertex:
        // {0,1,2} opp 3, {0,1,3} opp 2, {0,2,3} opp 1, {1,2,3} opp 0. (A prior version had a typo
        // here - [0,2,1,3] instead of [0,1,3,2] - which duplicated the {0,1,2} face with reversed
        // winding and NEVER TESTED the real {0,1,3} face at all. That silently let the enclosure
        // test pass tetrahedra that were not actually enclosing on all four true faces.)
        const idx = [[0, 1, 2, 3], [0, 1, 3, 2], [0, 2, 3, 1], [1, 2, 3, 0]]; // (face..., opposite point)
        for (let f = 0; f < 4; f++) {
            const [ia, ib, ic, id] = idx[f];
            const ax = this._wx[ia], ay = this._wy[ia], az = this._wz[ia];
            const bx = this._wx[ib], by = this._wy[ib], bz = this._wz[ib];
            const cx = this._wx[ic], cy = this._wy[ic], cz = this._wz[ic];
            const dx = this._wx[id], dy = this._wy[id], dz = this._wz[id];
            const abx = bx - ax, aby = by - ay, abz = bz - az;
            const acx = cx - ax, acy = cy - ay, acz = cz - az;
            let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
            const nLenSq = nx * nx + ny * ny + nz * nz;
            if (nLenSq < 1e-20) continue; // degenerate (collinear) face: skip, another face decides
            // Orient the normal away from the fourth (opposite) point.
            const toD = (dx - ax) * nx + (dy - ay) * ny + (dz - az) * nz;
            if (toD > 0) { nx = -nx; ny = -ny; nz = -nz; }
            const toOriginRaw = -ax * nx - ay * ny - az * nz; // (origin - a) . n, UNNORMALIZED
            // Compare the actual signed DISTANCE (toOriginRaw / |n|), not the raw dot product -
            // |n| scales with the face's own size, so a fixed raw-dot epsilon means a different
            // real-world tolerance for every pair of shapes. This is what made an exactly-touching
            // pair (the origin sitting exactly on a Minkowski-difference face, by construction -
            // not numerical error) misclassify as enclosed: the unnormalized epsilon was far
            // smaller than the coordinate magnitudes involved.
            const signedDist = toOriginRaw / Math.sqrt(nLenSq);
            // The threshold is NEGATIVE, not zero: a face the origin sits exactly ON does not
            // count as enclosure, only a face it is STRICTLY behind by a real margin does. This
            // matters for the seed tetrahedron above (four diverse directions, not grown
            // incrementally): an exactly-touching pair still produces a seed tetrahedron with two
            // faces passing exactly through the origin (the touching plane, seen from two
            // different angles) - accepting signedDist===0 as "inside" would misreport that as
            // overlap. Requiring a real negative margin is what keeps the exact-touching boundary
            // on the separated side while still finding every GENUINE overlap: a real overlap's
            // seed tetrahedron (built from four directions spanning different octants, so it is
            // never confined to a single touching plane the way incremental growth can get stuck
            // in) has every face strictly negative once the shapes interpenetrate at all.
            if (signedDist > -1e-9) {
                // Origin is outside (or exactly on) this face - reduce to the face's triangle and
                // recurse.
                this._reduceTo([ia, ib, ic]);
                return this._simplexTriangle();
            }
        }
        // Inside every face by the epsilon above. Defense in depth before trusting this as genuine
        // overlap: confirm the tetrahedron itself is not near-degenerate (all four points nearly
        // coplanar) - a flat tetrahedron can pass every per-face test at once without the origin
        // being genuinely surrounded in 3D.
        const v0x = this._wx[1] - this._wx[0], v0y = this._wy[1] - this._wy[0], v0z = this._wz[1] - this._wz[0];
        const v1x = this._wx[2] - this._wx[0], v1y = this._wy[2] - this._wy[0], v1z = this._wz[2] - this._wz[0];
        const v2x = this._wx[3] - this._wx[0], v2y = this._wy[3] - this._wy[0], v2z = this._wz[3] - this._wz[0];
        const cxv = v1y * v2z - v1z * v2y, cyv = v1z * v2x - v1x * v2z, czv = v1x * v2y - v1y * v2x;
        const volume6 = Math.abs(v0x * cxv + v0y * cyv + v0z * czv); // 6x tetrahedron volume
        // Threshold relative to the tetrahedron's own extent, not an absolute constant - the same
        // reasoning as the normalized signedDist above: an absolute epsilon means a different
        // real-world tolerance for every shape scale.
        const extentSq = Math.max(v0x*v0x+v0y*v0y+v0z*v0z, v1x*v1x+v1y*v1y+v1z*v1z, v2x*v2x+v2y*v2y+v2z*v2z);
        if (volume6 * volume6 < 1e-12 * extentSq * extentSq * extentSq) {
            // Degenerate tetrahedron: fall back to the touching-plane triangle (the first three
            // points, which is where the origin actually sits) rather than reporting overlap.
            this._reduceTo([0, 1, 2]);
            return this._simplexTriangle();
        }
        return { containsOrigin: true, direction: null, closest: null };
    }

    // Closest point on triangle ABC to the origin, given its (non-unit) normal N and |N|^2.
    // Returns { x,y,z, onEdge: null|[indices] } - onEdge non-null means the closest feature is a
    // vertex/edge (indices into the CURRENT 3-point simplex), so the caller reduces to it.
    static _closestPointOnTriangleToOrigin(ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz, nLenSq) {
        // Edge/vertex Voronoi region tests (Ericson 5.1.5), inlined for the origin as query point.
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const acx = cx - ax, acy = cy - ay, acz = cz - az;
        const apx = -ax, apy = -ay, apz = -az; // origin - a

        const d1 = abx * apx + aby * apy + abz * apz;
        const d2 = acx * apx + acy * apy + acz * apz;
        if (d1 <= 0 && d2 <= 0) return { x: ax, y: ay, z: az, onEdge: [0] };

        const bpx = -bx, bpy = -by, bpz = -bz;
        const d3 = abx * bpx + aby * bpy + abz * bpz;
        const d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0 && d4 <= d3) return { x: bx, y: by, z: bz, onEdge: [1] };

        const vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) {
            const t = d1 / (d1 - d3);
            return { x: ax + abx * t, y: ay + aby * t, z: az + abz * t, onEdge: [0, 1] };
        }

        const cpx = -cx, cpy = -cy, cpz = -cz;
        const d5 = abx * cpx + aby * cpy + abz * cpz;
        const d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0 && d5 <= d6) return { x: cx, y: cy, z: cz, onEdge: [2] };

        const vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            const t = d2 / (d2 - d6);
            return { x: ax + acx * t, y: ay + acy * t, z: az + acz * t, onEdge: [0, 2] };
        }

        const va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            const t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return { x: bx + (cx - bx) * t, y: by + (cy - by) * t, z: bz + (cz - bz) * t, onEdge: [1, 2] };
        }

        // Interior: project the origin onto the triangle's plane along its normal.
        // proj = origin - ((origin - a) . n_hat) n_hat ; with origin = 0 and n_hat = n/|n|:
        //      = -( (-a).n / |n|^2 ) n = (a.n / |n|^2) n
        const k = (ax * nx + ay * ny + az * nz) / nLenSq;
        return { x: nx * k, y: ny * k, z: nz * k, onEdge: null };
    }
}

ActionPhysics.GJK = GJK;


// ==== src/collision/EPA.js ====
/**
 * EPA (Expanding Polytope Algorithm): penetration depth, normal, and contact points from a GJK
 * simplex that already encloses the origin. GJK proves overlap; EPA measures how deep.
 *
 * Written from the algorithm (van den Bergen, "Collision Detection in Interactive 3D
 * Environments"; the standard EPA formulation), not from a prior implementation - same reasoning
 * as GJK.js: narrowphase is the highest-risk component, and the one place a structural anchor to
 * the predecessor would be worst to carry over.
 *
 * Produces: signed distance (always >= 0 here — a positive PENETRATION depth, matching the
 * Narrowphase contract's convention where positive = overlapping), a world-space normal pointing
 * from B to A, and witness points on each shape's surface. May assume: the input simplex is a
 * genuine tetrahedron (4 points) that encloses the origin - GJK.run() only ever returns
 * `overlapping: true` from its `_simplexTetrahedron` path, which never produces anything else.
 *
 * BUG FIX CARRIED FROM THE PREDECESSOR (plan.md, Bug reference / Collision detection):
 * "EPA accepted garbage on the iteration cap." Its stable-exit test required
 * `closest_face_distance > EPSILON`, but a resting/shallow contact sits BELOW epsilon, so it never
 * exited early, ran to the iteration cap, and returned whatever the LAST iteration's polytope
 * state happened to be - not the best one found. The fix here is structural: the closest face
 * found across every iteration is tracked independently of the loop's current state, and the
 * final return always reads from that tracked best - never from "whatever the polytope looks like
 * when the loop happens to end." A resting contact (depth near zero) converges immediately because
 * there is nothing left to expand, not because of a lucky exit-epsilon comparison.
 */
class EPA {
    constructor() {
        // Polytope vertices, parallel arrays like GJK's simplex storage (w = Minkowski diff point,
        // a/b = world witness points on each shape). Grows as EPA expands the polytope; capacity
        // starts generous and grows geometrically to stay allocation-light across many calls.
        this._capacity = 64;
        this._wx = new Float64Array(this._capacity); this._wy = new Float64Array(this._capacity); this._wz = new Float64Array(this._capacity);
        this._ax = new Float64Array(this._capacity); this._ay = new Float64Array(this._capacity); this._az = new Float64Array(this._capacity);
        this._bx = new Float64Array(this._capacity); this._by = new Float64Array(this._capacity); this._bz = new Float64Array(this._capacity);
        this._vertexCount = 0;

        // Faces: triangle index triples plus their (outward) normal and distance-to-origin, kept
        // in parallel arrays. A face is "alive" while faceAlive[i] is truthy; removed faces
        // (replaced during expansion) are marked dead rather than spliced out, to avoid shifting
        // every later index on every removal.
        this._faceCapacity = 128;
        this._faceA = new Int32Array(this._faceCapacity);
        this._faceB = new Int32Array(this._faceCapacity);
        this._faceC = new Int32Array(this._faceCapacity);
        this._faceNx = new Float64Array(this._faceCapacity);
        this._faceNy = new Float64Array(this._faceCapacity);
        this._faceNz = new Float64Array(this._faceCapacity);
        this._faceDist = new Float64Array(this._faceCapacity);
        this._faceAlive = new Uint8Array(this._faceCapacity);
        this._faceCount = 0;

        this._newW = new Vector3();
        this._newA = new Vector3();
        this._newB = new Vector3();
        this._dirScratch = new Vector3();
    }

    _growVertices() {
        const cap = this._capacity * 2;
        const grow = (arr) => { const n = new arr.constructor(cap); n.set(arr); return n; };
        this._wx = grow(this._wx); this._wy = grow(this._wy); this._wz = grow(this._wz);
        this._ax = grow(this._ax); this._ay = grow(this._ay); this._az = grow(this._az);
        this._bx = grow(this._bx); this._by = grow(this._by); this._bz = grow(this._bz);
        this._capacity = cap;
    }

    _growFaces() {
        const cap = this._faceCapacity * 2;
        const grow = (arr) => { const n = new arr.constructor(cap); n.set(arr); return n; };
        this._faceA = grow(this._faceA); this._faceB = grow(this._faceB); this._faceC = grow(this._faceC);
        this._faceNx = grow(this._faceNx); this._faceNy = grow(this._faceNy); this._faceNz = grow(this._faceNz);
        this._faceDist = grow(this._faceDist);
        this._faceAlive = grow(this._faceAlive);
        this._faceCapacity = cap;
    }

    _pushVertex(w, a, b) {
        if (this._vertexCount >= this._capacity) this._growVertices();
        const i = this._vertexCount++;
        this._wx[i] = w.x; this._wy[i] = w.y; this._wz[i] = w.z;
        this._ax[i] = a.x; this._ay[i] = a.y; this._az[i] = a.z;
        this._bx[i] = b.x; this._by[i] = b.y; this._bz[i] = b.z;
        return i;
    }

    // Adds a face from three vertex indices, computing its outward normal and origin distance.
    // "Outward" here means away from the polytope's own centroid - correct as long as the
    // polytope is convex and the origin is inside it (true by construction: EPA only ever starts
    // from a GJK tetrahedron that already encloses the origin, and every subsequent expansion
    // stays convex around that same enclosed origin). Returns the new face's index, or -1 if the
    // three points are degenerate (collinear / zero area) - the caller skips a degenerate face
    // rather than adding a face with an undefined normal (same discipline as GJK's own degenerate
    // fallbacks: never propagate a NaN).
    _addFace(ia, ib, ic, centroidHint) {
        const ax = this._wx[ia], ay = this._wy[ia], az = this._wz[ia];
        const bx = this._wx[ib], by = this._wy[ib], bz = this._wz[ib];
        const cx = this._wx[ic], cy = this._wy[ic], cz = this._wz[ic];
        const abx = bx - ax, aby = by - ay, abz = bz - az;
        const acx = cx - ax, acy = cy - ay, acz = cz - az;
        let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        const nLenSq = nx * nx + ny * ny + nz * nz;
        if (nLenSq < 1e-20) return -1; // degenerate: skip, never add a face with no real normal

        const invLen = 1 / Math.sqrt(nLenSq);
        nx *= invLen; ny *= invLen; nz *= invLen;

        // Orient outward: away from the hint point (the polytope's centroid, or the 4th
        // tetrahedron vertex on initial construction).
        const toHint = (centroidHint.x - ax) * nx + (centroidHint.y - ay) * ny + (centroidHint.z - az) * nz;
        if (toHint > 0) { nx = -nx; ny = -ny; nz = -nz; }

        const dist = ax * nx + ay * ny + az * nz; // (a - origin) . n_hat = distance from origin to the face's plane

        if (this._faceCount >= this._faceCapacity) this._growFaces();
        const fi = this._faceCount++;
        this._faceA[fi] = ia; this._faceB[fi] = ib; this._faceC[fi] = ic;
        this._faceNx[fi] = nx; this._faceNy[fi] = ny; this._faceNz[fi] = nz;
        this._faceDist[fi] = dist;
        this._faceAlive[fi] = 1;
        return fi;
    }

    /**
     * Expands `simplex` (a GJK instance whose _wx/_wy/_wz/_ax.../_bx... hold exactly 4 points that
     * enclose the origin) into the true penetration depth and normal.
     *
     * Returns { distance, normal, pointA, pointB } - distance is a non-negative penetration depth
     * (plan.md's positive = overlapping convention), normal points from B to A (same convention
     * GJK's separated result uses), pointA/pointB are witness points on each shape's own surface
     * recovered from the winning face's barycentric weights.
     *
     * maxIterations guards non-convergence on pathological input. Per the header's bug-fix note,
     * hitting the cap returns the best (closest-to-origin) face tracked across the WHOLE run, not
     * whatever face is live when the loop happens to stop.
     */
    // Completes a full, non-degenerate tetrahedron (4 points, real volume) in this._wx.. from the
    // GJK simplex's `_count` points (1..4), adding Minkowski support points as needed. Returns true
    // on success (4 vertices are in place, indices 0..3), false if the contact is genuinely flat
    // (no volume obtainable - an exact touch). Standard EPA preamble (van den Bergen).
    _buildInitialTetrahedron(support, simplex) {
        this._vertexCount = 0;
        const n = simplex._count !== undefined ? simplex._count : 4;
        for (let i = 0; i < n; i++) {
            this._pushVertex(
                { x: simplex._wx[i], y: simplex._wy[i], z: simplex._wz[i] },
                { x: simplex._ax[i], y: simplex._ay[i], z: simplex._az[i] },
                { x: simplex._bx[i], y: simplex._by[i], z: simplex._bz[i] }
            );
        }
        // A small set of probe directions to grow dimensionality with; each is tried in +/- form.
        const AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [0, 1, 1], [1, 0, 1]];

        // 1 point -> 2: add a support along any axis that gives a distinct point.
        if (this._vertexCount === 1) {
            for (let a = 0; a < AXES.length && this._vertexCount < 2; a++) {
                for (let s = -1; s <= 1 && this._vertexCount < 2; s += 2) {
                    this._dirScratch.set(AXES[a][0] * s, AXES[a][1] * s, AXES[a][2] * s);
                    support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);
                    if (this._distinctFrom(0)) this._pushVertex(this._newW, this._newA, this._newB);
                }
            }
            if (this._vertexCount < 2) return false;
        }
        // 2 points -> 3: add a support perpendicular to the segment, giving a non-collinear point.
        if (this._vertexCount === 2) {
            const ex = this._wx[1] - this._wx[0], ey = this._wy[1] - this._wy[0], ez = this._wz[1] - this._wz[0];
            for (let a = 0; a < AXES.length && this._vertexCount < 3; a++) {
                // direction = axis component perpendicular to the edge
                let dx = AXES[a][0], dy = AXES[a][1], dz = AXES[a][2];
                const dot = (dx * ex + dy * ey + dz * ez) / (ex * ex + ey * ey + ez * ez + 1e-30);
                dx -= dot * ex; dy -= dot * ey; dz -= dot * ez;
                if (dx * dx + dy * dy + dz * dz < 1e-12) continue;
                for (let s = -1; s <= 1 && this._vertexCount < 3; s += 2) {
                    this._dirScratch.set(dx * s, dy * s, dz * s);
                    support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);
                    if (this._notCollinear(0, 1)) this._pushVertex(this._newW, this._newA, this._newB);
                }
            }
            if (this._vertexCount < 3) return false;
        }
        // 3 points -> 4: add support along the triangle normal (both sides) for a point off-plane.
        if (this._vertexCount === 3) {
            const ax = this._wx[0], ay = this._wy[0], az = this._wz[0];
            const abx = this._wx[1] - ax, aby = this._wy[1] - ay, abz = this._wz[1] - az;
            const acx = this._wx[2] - ax, acy = this._wy[2] - ay, acz = this._wz[2] - az;
            let nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
            const nl = Math.sqrt(nx * nx + ny * ny + nz * nz);
            if (nl < 1e-12) return false; // triangle itself degenerate
            nx /= nl; ny /= nl; nz /= nl;
            for (let s = -1; s <= 1 && this._vertexCount < 4; s += 2) {
                this._dirScratch.set(nx * s, ny * s, nz * s);
                support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);
                if (this._offPlane(0, 1, 2)) this._pushVertex(this._newW, this._newA, this._newB);
            }
            if (this._vertexCount < 4) return false;
        }
        return this._vertexCount >= 4;
    }

    // Is this._newW distinct (beyond epsilon) from stored vertex i?
    _distinctFrom(i) {
        const dx = this._newW.x - this._wx[i], dy = this._newW.y - this._wy[i], dz = this._newW.z - this._wz[i];
        return dx * dx + dy * dy + dz * dz > 1e-10;
    }
    // Is this._newW non-collinear with stored vertices i, j?
    _notCollinear(i, j) {
        const ex = this._wx[j] - this._wx[i], ey = this._wy[j] - this._wy[i], ez = this._wz[j] - this._wz[i];
        const fx = this._newW.x - this._wx[i], fy = this._newW.y - this._wy[i], fz = this._newW.z - this._wz[i];
        const cx = ey * fz - ez * fy, cy = ez * fx - ex * fz, cz = ex * fy - ey * fx;
        return cx * cx + cy * cy + cz * cz > 1e-12;
    }
    // Is this._newW off the plane through stored vertices i, j, k (real tetra volume)?
    _offPlane(i, j, k) {
        const ax = this._wx[i], ay = this._wy[i], az = this._wz[i];
        const abx = this._wx[j] - ax, aby = this._wy[j] - ay, abz = this._wz[j] - az;
        const acx = this._wx[k] - ax, acy = this._wy[k] - ay, acz = this._wz[k] - az;
        const nx = aby * acz - abz * acy, ny = abz * acx - abx * acz, nz = abx * acy - aby * acx;
        const dx = this._newW.x - ax, dy = this._newW.y - ay, dz = this._newW.z - az;
        const vol = dx * nx + dy * ny + dz * nz;
        return vol * vol > 1e-14;
    }

    // Fallback for a genuinely flat (zero-volume) contact: report zero depth, using the simplex's
    // first witness points and its search normal. The solver treats zero depth as non-penetrating.
    _zeroDepthResult(simplex) {
        const pointA = new Vector3(simplex._ax[0], simplex._ay[0], simplex._az[0]);
        const pointB = new Vector3(simplex._bx[0], simplex._by[0], simplex._bz[0]);
        let nx = 0, ny = 1, nz = 0;
        if (simplex._closest && simplex._closest.lengthSquared && simplex._closest.lengthSquared() > 1e-20) {
            const l = Math.sqrt(simplex._closest.lengthSquared());
            nx = simplex._closest.x / l; ny = simplex._closest.y / l; nz = simplex._closest.z / l;
        }
        return { distance: 0, normal: new Vector3(nx, ny, nz), pointA: pointA, pointB: pointB };
    }

    run(support, simplex, maxIterations) {
        maxIterations = maxIterations || 64;
        this._vertexCount = 0;
        this._faceCount = 0;

        // GJK may hand over fewer than 4 points: its enclosure/stall paths can confirm overlap from
        // a 1-, 2-, or 3-point simplex (a small shape shallowly inside a much larger one, where the
        // seed tetrahedra never enclosed the origin). EPA needs a full, origin-enclosing tetrahedron
        // to start, so complete one first from whatever GJK provided. Reading _wx[3] unconditionally
        // when only 3 points exist read stale array slots and crashed (undefined support point) -
        // this preamble is the fix.
        if (!this._buildInitialTetrahedron(support, simplex)) {
            // Could not form a non-degenerate enclosing tetrahedron (true exact touch, or numerically
            // flat contact): depth is zero along the best direction available. Report a zero-depth
            // contact using the simplex's first witness points and the search normal - the solver's
            // C<=0 guard treats a zero-depth contact as non-penetrating, which is correct here.
            return this._zeroDepthResult(simplex);
        }
        const idx = [0, 1, 2, 3];
        // Centroid of the 4 seed points, used to orient each face's normal outward.
        const cx = (this._wx[idx[0]] + this._wx[idx[1]] + this._wx[idx[2]] + this._wx[idx[3]]) / 4;
        const cy = (this._wy[idx[0]] + this._wy[idx[1]] + this._wy[idx[2]] + this._wy[idx[3]]) / 4;
        const cz = (this._wz[idx[0]] + this._wz[idx[1]] + this._wz[idx[2]] + this._wz[idx[3]]) / 4;
        const centroid = { x: cx, y: cy, z: cz };

        // The 4 faces of the seed tetrahedron - same enumeration GJK.js uses (and the same one
        // whose typo caused GJK's own hardest bug: verify all 4 DISTINCT faces are present).
        this._addFace(idx[0], idx[1], idx[2], centroid);
        this._addFace(idx[0], idx[1], idx[3], centroid);
        this._addFace(idx[0], idx[2], idx[3], centroid);
        this._addFace(idx[1], idx[2], idx[3], centroid);

        for (let iter = 0; iter < maxIterations; iter++) {
            const face = this._closestAliveFace();
            const faceDist = this._faceDist[face];

            // Expand along the closest face's own normal - the direction most likely to find the
            // true surface next.
            this._dirScratch.set(this._faceNx[face], this._faceNy[face], this._faceNz[face]);
            support.supportInto(this._newW, this._dirScratch, this._newA, this._newB);

            const newDist = this._newW.x * this._faceNx[face] + this._newW.y * this._faceNy[face] + this._newW.z * this._faceNz[face];

            // Converged: the new support point does not extend past the closest face's own plane
            // by more than a small margin. This is checked against the closest face's distance
            // DIRECTLY (not a fixed epsilon independent of scale) - see the header's bug-fix note:
            // the predecessor's bug was exactly a fixed-epsilon exit test failing for shallow
            // contacts. Comparing the new point's extent to the face's own already-converged
            // distance means a resting contact (whose closest face sits near-zero) still detects
            // "no more progress" correctly, because both sides of the comparison scale together.
            if (newDist - faceDist < 1e-6) break;

            this._expandAt(this._newW, this._newA, this._newB, centroid);
        }

        // The closest ALIVE face, read fresh right here rather than tracked across iterations.
        // This IS the actual fix for the predecessor's bug (plan.md, Bug reference: "returned
        // whatever the final iteration produced" instead of the converged answer) - a face's own
        // distance value is only meaningful while it is still alive; a face expansion replaces
        // wrong-but-smaller-looking faces with new ones as the polytope refines toward the true
        // surface, so tracking "smallest distance ever seen across iterations" (tried and reverted
        // - see git history) picks up STALE distances from faces that were later proven invalid
        // and removed. Re-querying the live polytope's actual closest alive face, whether the loop
        // converged cleanly or hit the iteration cap, is the only way to get the CURRENT answer -
        // exactly the "return the converged result, never the last one" rule, applied correctly.
        return this._resultFromFace(this._closestAliveFace());
    }

    // Finds the living face with the smallest distance-to-origin. Linear scan - EPA's polytope
    // stays small (tens of faces) for the shape pairs this engine targets, and a scan here is far
    // simpler to trust than a priority-queue structure for that size.
    _closestAliveFace() {
        let best = -1, bestDist = Infinity;
        for (let i = 0; i < this._faceCount; i++) {
            if (!this._faceAlive[i]) continue;
            if (this._faceDist[i] < bestDist) { bestDist = this._faceDist[i]; best = i; }
        }
        return best;
    }

    // Adds `newPoint` to the polytope and re-triangulates: every alive face visible from the new
    // point (the point is on the OUTER side of the face's plane) is removed, and the boundary loop
    // of the resulting hole is re-closed with new faces to the new point (the standard EPA
    // horizon-edge expansion).
    _expandAt(newW, newA, newB, centroid) {
        const newIdx = this._pushVertex(newW, newA, newB);

        // Collect the horizon: edges shared by exactly one visible face and one non-visible face.
        // Represented as [fromIndex, toIndex] pairs, deduplicated by removing a pair the moment
        // its reverse is seen (a shared internal edge between two visible faces cancels out).
        const horizonA = [], horizonB = [];
        function edgeKey(a, b) { return a < b ? a + ',' + b : b + ',' + a; }
        const edgeSeen = new Map(); // key -> { from, to }

        for (let i = 0; i < this._faceCount; i++) {
            if (!this._faceAlive[i]) continue;
            const a = this._faceA[i], b = this._faceB[i], c = this._faceC[i];
            const nx = this._faceNx[i], ny = this._faceNy[i], nz = this._faceNz[i];
            const visible = (newW.x - this._wx[a]) * nx + (newW.y - this._wy[a]) * ny + (newW.z - this._wz[a]) * nz > 1e-10;
            if (!visible) continue;

            this._faceAlive[i] = 0;
            const edges = [[a, b], [b, c], [c, a]];
            for (let e = 0; e < 3; e++) {
                const from = edges[e][0], to = edges[e][1];
                const key = edgeKey(from, to);
                if (edgeSeen.has(key)) {
                    edgeSeen.delete(key); // shared with another visible face: internal, cancels out
                } else {
                    edgeSeen.set(key, { from: from, to: to });
                }
            }
        }

        edgeSeen.forEach(function (e) { horizonA.push(e.from); horizonB.push(e.to); });

        for (let i = 0; i < horizonA.length; i++) {
            this._addFace(horizonA[i], horizonB[i], newIdx, centroid);
        }
    }

    // Recovers { distance, normal, pointA, pointB } from a chosen face - barycentric weights of
    // the face's own closest point to the origin (projected onto the triangle's plane, then
    // expressed in area-ratio barycentric form), applied to the face's three world witness points.
    // Same degenerate-fallback discipline as GJK's own barycentric routine: never divide by zero,
    // always a valid (if approximate) geometric answer.
    _resultFromFace(face) {
        const ia = this._faceA[face], ib = this._faceB[face], ic = this._faceC[face];
        const nx = this._faceNx[face], ny = this._faceNy[face], nz = this._faceNz[face];
        const dist = this._faceDist[face];

        // Closest point on the face's plane to the origin. The plane is {x : x.n_hat = dist}, so
        // the closest point on it to the origin is dist * n_hat (that point's own dot with n_hat
        // is dist by construction, and it is the minimal-length point satisfying that).
        const ax = this._wx[ia], ay = this._wy[ia], az = this._wz[ia];
        const closestX = nx * dist, closestY = ny * dist, closestZ = nz * dist;

        const bx = this._wx[ib], by = this._wy[ib], bz = this._wz[ib];
        const cx = this._wx[ic], cy = this._wy[ic], cz = this._wz[ic];
        const v0x = bx - ax, v0y = by - ay, v0z = bz - az;
        const v1x = cx - ax, v1y = cy - ay, v1z = cz - az;
        const v2x = closestX - ax, v2y = closestY - ay, v2z = closestZ - az;
        const d00 = v0x * v0x + v0y * v0y + v0z * v0z;
        const d01 = v0x * v1x + v0y * v1y + v0z * v1z;
        const d11 = v1x * v1x + v1y * v1y + v1z * v1z;
        const d20 = v2x * v0x + v2y * v0y + v2z * v0z;
        const d21 = v2x * v1x + v2y * v1y + v2z * v1z;
        const denom = d00 * d11 - d01 * d01;
        let u, v, w;
        if (Math.abs(denom) < 1e-20) {
            u = 1 / 3; v = 1 / 3; w = 1 / 3; // degenerate face: even split, never NaN
        } else {
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
            u = 1 - v - w;
        }

        const pointA = new Vector3(
            u * this._ax[ia] + v * this._ax[ib] + w * this._ax[ic],
            u * this._ay[ia] + v * this._ay[ib] + w * this._ay[ic],
            u * this._az[ia] + v * this._az[ib] + w * this._az[ic]
        );
        const pointB = new Vector3(
            u * this._bx[ia] + v * this._bx[ib] + w * this._bx[ic],
            u * this._by[ia] + v * this._by[ib] + w * this._by[ic],
            u * this._bz[ia] + v * this._bz[ib] + w * this._bz[ic]
        );

        // The face's own "outward from the polytope's interior" normal points from A's side
        // toward B's side of the Minkowski difference A-B (verified against GJK's own separated-
        // result convention, which points from B to A - a real, sign-only bug caught by comparing
        // the two detectors' normals directly on identical geometry, not by any test that only
        // checked axis alignment). Negated here so EPA's returned normal matches GJK's: B to A,
        // the direction the solver actually pushes body A along.
        return {
            distance: Math.max(0, dist), // clamp: a face passing fractionally behind the origin from float noise still reports a valid non-negative depth
            normal: new Vector3(-nx, -ny, -nz),
            pointA: pointA,
            pointB: pointB
        };
    }
}

ActionPhysics.EPA = EPA;


// ==== src/collision/ContactDetails.js ====
/**
 * ContactDetails: one contact point between a specific pair of primitive shapes, in the sign
 * convention plan.md establishes for the whole narrowphase: signed distance NEGATIVE when
 * separated, POSITIVE when overlapping. GJK's separated result and EPA's overlapping result both
 * report a non-negative magnitude of their own (gap vs. depth) - normalizing the sign here is the
 * one place that distinction gets collapsed into a single number the rest of the pipeline can
 * treat uniformly (a manifold, a solver row, all just read `signedDistance`).
 *
 * pointOnA / pointOnB are the witness points on each shape's own surface (not the same point once
 * penetrating - that gap IS the depth). `point` is their midpoint, the conventional single contact
 * location a solver/manifold keys off; normal points from B to A, matching GJK/EPA's own
 * convention so no stage has to remember a sign flip.
 */
class ContactDetails {
    constructor() {
        this.point = new Vector3();
        this.pointOnA = new Vector3();
        this.pointOnB = new Vector3();
        this.normal = new Vector3();
        this.signedDistance = 0;
        // Set by the manifold once matched against a previous tick's point (warm-start data -
        // see plan.md's component list: "ContactManifold (4-point cap, dedup, warm-start data)").
        // ContactDetails itself never reads or writes this; it exists here purely as a place to
        // carry the value across the manifold's point-matching step without a second parallel
        // array. Owned entirely by the solver once it exists (Rule 2: one owner per concern).
        this.normalLambda = 0;
        this.tangentLambda1 = 0;
        this.tangentLambda2 = 0;

        // Body-LOCAL anchor offsets for pointOnA/pointOnB, set once by the manifold when this
        // point is created (ContactManifold._addPoint / update()'s new-point path) - NOT
        // recomputed by copy()/setFromGJKSeparated/setFromEPA, which only carry the world-space
        // geometry a fresh narrowphase result reports. The solver reads these every SUBSTEP to
        // recompute the contact's CURRENT gap from the bodies' current positions (see Solver.js's
        // class header and _solvePoint) - using the world-space pointOnA/pointOnB directly would
        // read a value frozen at the tick's single narrowphase pass, stale by the time later
        // substeps have already moved the bodies. This is the mechanism, not signedDistance, that
        // the solver actually corrects against.
        this.localAnchorA = new Vector3();
        this.localAnchorB = new Vector3();

        // Contact-relative normal velocity captured just before this substep's position solve (which
        // is about to zero it), so the velocity pass can apply restitution: bounce restores a
        // fraction of the speed the body was APPROACHING at, which is gone by the time the solve
        // finishes. Written each substep by the solver; not warm-start state.
        this._preSolveNormalVel = 0;
    }

    // Derives localAnchorA/localAnchorB from the CURRENT pointOnA/pointOnB and the given bodies'
    // CURRENT transforms. Called once, at the moment this point is created in a manifold (never
    // on a re-matched/refreshed point, which keeps its ORIGINAL anchors - that persistence across
    // ticks is what lets the solver see a growing gap as a body drifts, rather than the anchor
    // re-snapping to zero gap every tick).
    setLocalAnchors(bodyA, bodyB) {
        const invRotA = ContactDetails._scratchQuat.copy(bodyA.rotation).invert();
        Vector3.subInto(this.localAnchorA, this.pointOnA, bodyA.position);
        invRotA.transformVectorInPlace(this.localAnchorA);

        const invRotB = ContactDetails._scratchQuat.copy(bodyB.rotation).invert();
        Vector3.subInto(this.localAnchorB, this.pointOnB, bodyB.position);
        invRotB.transformVectorInPlace(this.localAnchorB);
        return this;
    }

    // Current world position of localAnchorA/B, written into out. Used by the solver every
    // substep to find each anchor's LIVE position without re-running narrowphase.
    currentAnchorAInto(out, bodyA) {
        out.copy(this.localAnchorA);
        bodyA.rotation.transformVectorInPlace(out);
        out.addInPlace(bodyA.position);
        return out;
    }

    currentAnchorBInto(out, bodyB) {
        out.copy(this.localAnchorB);
        bodyB.rotation.transformVectorInPlace(out);
        out.addInPlace(bodyB.position);
        return out;
    }

    // Fills this from a GJK separated result (`{distance, normal, pointA, pointB}`, distance is a
    // non-negative GAP). signedDistance becomes negative - separated, per plan.md's convention.
    setFromGJKSeparated(gjkResult) {
        this.pointOnA.copy(gjkResult.pointA);
        this.pointOnB.copy(gjkResult.pointB);
        this.normal.copy(gjkResult.normal);
        this.signedDistance = -gjkResult.distance;
        Vector3.addInto(this.point, gjkResult.pointA, gjkResult.pointB).scaleInPlace(0.5);
        return this;
    }

    // Fills this from an EPA result (`{distance, normal, pointA, pointB}`, distance is a
    // non-negative penetration DEPTH). signedDistance becomes positive - overlapping.
    setFromEPA(epaResult) {
        this.pointOnA.copy(epaResult.pointA);
        this.pointOnB.copy(epaResult.pointB);
        this.normal.copy(epaResult.normal);
        this.signedDistance = epaResult.distance;
        Vector3.addInto(this.point, epaResult.pointA, epaResult.pointB).scaleInPlace(0.5);
        return this;
    }

    copy(other) {
        this.point.copy(other.point);
        this.pointOnA.copy(other.pointOnA);
        this.pointOnB.copy(other.pointOnB);
        this.normal.copy(other.normal);
        this.signedDistance = other.signedDistance;
        this.normalLambda = other.normalLambda;
        this.tangentLambda1 = other.tangentLambda1;
        this.tangentLambda2 = other.tangentLambda2;
        return this;
    }

    clone() {
        return new ContactDetails().copy(this);
    }
}

ContactDetails._scratchQuat = new Quaternion();

ActionPhysics.ContactDetails = ContactDetails;


// ==== src/collision/ContactManifold.js ====
/**
 * ContactManifold: the persistent contact state for one pair of primitive shapes, across ticks.
 *
 * Owns point lifetime ENTIRELY (plan.md, component 5 and Rule 1/2). Narrowphase (via update())
 * only ever ADDS or REFRESHES points from this tick's GJK/EPA result; only the manifold itself
 * REMOVES a point, and only between ticks (never mid-substep - see the bug reference below).
 * Everywhere else assumes a manifold's point set is stable for the duration of a tick.
 *
 * BUG FIX CARRIED FROM THE PREDECESSOR (plan.md, Bug reference / Contact management):
 * "Refreshing manifolds mid-tick emptied them." Re-running a staleness cull once per SUBSTEP
 * retired points that were merely mid-correction - not actually separated, just still being
 * resolved by the solver's own position projection within the same tick. 38 of 1210 manifolds
 * emptied, dropping bodies onto their neighbours. The fix here is structural: update() (called
 * once per TICK by narrowphase, never per substep) is the only place points are added or pruned.
 * The solver, wherever it substeps within a tick, reads and writes lambda/geometry on the SAME
 * point objects without ever adding, removing, or re-matching them mid-tick.
 *
 * PERSISTENCE / WARM-START: a manifold holds up to 4 points (MAX_POINTS). Each update() call
 * matches this tick's narrowphase result against the existing points (by proximity in LOCAL space
 * relative to body A - world position drifts as A moves, but a contact feature's position
 * relative to A's own frame stays close between ticks unless the contact point itself is sliding).
 * A match copies the new geometry (point/normal/signedDistance) onto the EXISTING point object,
 * preserving its accumulated lambda for the solver's warm start; no match adds a new point (via
 * the 4-point reduction below if already full).
 */
class ContactManifold {
    static MAX_POINTS = 4;
    // A matched point's local-space (body-A-relative) position must stay within this distance of
    // where it was last tick to count as "the same contact" rather than a new one. Chosen as a
    // fraction of a typical contact's own scale rather than an absolute constant - see update()'s
    // matching call for how this get scaled by the manifold's own point spread.
    static MATCH_DISTANCE = 0.05;
    // Signed-distance half-width of the exact-touch band where GJK/EPA's normal is treated as
    // ambiguous and a warm-matched point keeps its established normal instead (see update()). Sized
    // a little above the numerical noise of a flush contact, well below any real overlap depth the
    // solver needs to resolve - inside this band the shapes are touching to within a fraction of a
    // millimetre and the normal genuinely cannot be recovered reliably from a single query.
    static EXACT_TOUCH_BAND = 0.001;

    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.points = []; // ContactDetails[], length 0..MAX_POINTS
        // Local-space (relative to bodyA's CURRENT transform at match time) anchor for each point,
        // parallel to `points` - used only for next-tick matching, recomputed every update().
        this._localAnchors = [];
    }

    get pointCount() { return this.points.length; }

    // Called once per TICK (never per substep - see the class header). `newContacts` is this
    // tick's narrowphase result for this body pair: an array of ContactDetails, typically length 1
    // (one primitive pair -> one GJK/EPA contact) but the manifold accepts any count so a caller
    // batching multiple sub-contacts (e.g. a multi-triangle mesh region) works the same way.
    //
    // Points not re-confirmed this tick (no incoming contact matched them, or the match exceeded
    // MATCH_DISTANCE, or signedDistance separated past REMOVE_DISTANCE) are removed HERE - this is
    // the manifold's one removal path, and it only ever runs from this method.
    update(newContacts) {
        const matched = new Array(newContacts.length).fill(false);

        // Match each existing point against the best (closest, in bodyA-local space) unmatched
        // incoming contact. A match refreshes the existing point's geometry in place, keeping its
        // accumulated lambda - this IS the warm start.
        for (let i = this.points.length - 1; i >= 0; i--) {
            const existing = this.points[i];
            const existingLocal = this._localAnchors[i];
            let bestJ = -1, bestDistSq = ContactManifold.MATCH_DISTANCE * ContactManifold.MATCH_DISTANCE;
            for (let j = 0; j < newContacts.length; j++) {
                if (matched[j]) continue;
                const localCandidate = ContactManifold._toLocal(this.bodyA, newContacts[j].pointOnA);
                const dx = localCandidate.x - existingLocal.x, dy = localCandidate.y - existingLocal.y, dz = localCandidate.z - existingLocal.z;
                const distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < bestDistSq) { bestDistSq = distSq; bestJ = j; }
            }
            if (bestJ === -1) {
                // Not re-confirmed this tick: remove. This is the ONLY place a point is removed -
                // never mid-substep, never from a separate staleness sweep (plan.md, Bug
                // reference). A point that genuinely separated simply stops being reported by
                // narrowphase and is pruned here, on the very next tick's update() call.
                this.points.splice(i, 1);
                this._localAnchors.splice(i, 1);
                continue;
            }
            matched[bestJ] = true;
            // Save the accumulated lambda BEFORE copy() overwrites it - copy() pulls every field
            // from newContacts[bestJ], whose lambda fields are always zero (a fresh ContactDetails
            // narrowphase just produced this tick, with no solver history of its own). Losing this
            // ordering was an early, self-inflicted version of this bug: `existing` and
            // `this.points[i]` are the SAME object, so reading "the prior value" AFTER copy() just
            // reads back the zero that was already written - this is why the values are captured
            // into locals first.
            const keepNormalLambda = existing.normalLambda;
            const keepTangentLambda1 = existing.tangentLambda1;
            const keepTangentLambda2 = existing.tangentLambda2;
            // Preserve the ESTABLISHED contact normal across an exact-touch refresh. At a signed
            // distance within EXACT_TOUCH_BAND of zero (shapes touching flush, neither clearly
            // separated nor clearly overlapping), GJK/EPA's normal is genuinely ambiguous - the
            // origin sits ON the Minkowski-difference boundary, so the recovered direction can flip
            // to a diagonal face normal (a box resting flush reports (0.707,0,0.707) instead of the
            // true (0,1,0) for one tick). A persistent contact's normal does NOT actually change
            // tick to tick, so trusting a single ambiguous tick's normal over the one this point
            // has carried while it was unambiguously resolving is the wrong call: it makes the
            // constraint briefly point sideways, the body sinks through, and the next (recovered)
            // tick ejects it back out - a permanent penetrate-then-launch limit cycle. This is the
            // manifold owning contact identity across ticks (its documented job), not a solver-side
            // governor. Outside the band (a real gap or a real overlap), the fresh normal is
            // trustworthy and is taken as-is.
            const keepNormal = Math.abs(newContacts[bestJ].signedDistance) < ContactManifold.EXACT_TOUCH_BAND
                ? ContactManifold._scratchNormal.copy(existing.normal)
                : null;
            existing.copy(newContacts[bestJ]); // geometry refreshed
            existing.normalLambda = keepNormalLambda; // warm start restored
            existing.tangentLambda1 = keepTangentLambda1;
            existing.tangentLambda2 = keepTangentLambda2;
            if (keepNormal) existing.normal.copy(keepNormal); // established normal kept through the ambiguous band
            this._localAnchors[i] = ContactManifold._toLocal(this.bodyA, existing.pointOnA);
        }

        // Any incoming contact not matched to an existing point is genuinely new.
        for (let j = 0; j < newContacts.length; j++) {
            if (matched[j]) continue;
            this._addPoint(newContacts[j]);
        }
    }

    _addPoint(contact) {
        const point = contact.clone();
        point.normalLambda = 0; point.tangentLambda1 = 0; point.tangentLambda2 = 0; // fresh point: no warm-start data yet
        // Local anchors are set ONCE here, at creation - see ContactDetails.setLocalAnchors and
        // Solver.js's class header for why the solver needs these (recomputing the contact's
        // CURRENT gap every substep) rather than reusing the single signedDistance this tick's
        // narrowphase pass measured before any substep moved the bodies.
        point.setLocalAnchors(this.bodyA, this.bodyB);
        const local = ContactManifold._toLocal(this.bodyA, point.pointOnA);

        if (this.points.length < ContactManifold.MAX_POINTS) {
            this.points.push(point);
            this._localAnchors.push(local);
            return;
        }

        // Already at the cap: reduce. Standard 4-point manifold reduction (Bullet, Box2D use the
        // same idea) - always KEEP the deepest point (it matters most for the solver), and among
        // the remaining candidates (the new point plus the 3 non-deepest existing ones) keep
        // whichever 3 form the LARGEST-AREA quadrilateral with the deepest point. Maximizing area
        // keeps the manifold spread out (good torque resistance - a box resting on a corner-only
        // manifold rocks; a box resting on 4 spread corners doesn't), rather than collapsing onto
        // whichever points happen to be deepest overall.
        this._reduceToFour(point, local);
    }

    _reduceToFour(candidatePoint, candidateLocal) {
        // Find the deepest point among the 4 existing + the candidate (deepest = largest
        // signedDistance, i.e. most overlapping - the point the solver most needs to resolve).
        let deepestIdx = -1, deepestVal = candidatePoint.signedDistance;
        for (let i = 0; i < this.points.length; i++) {
            if (this.points[i].signedDistance > deepestVal) { deepestVal = this.points[i].signedDistance; deepestIdx = i; }
        }
        const deepestIsCandidate = deepestIdx === -1;
        const deepestPoint = deepestIsCandidate ? candidatePoint : this.points[deepestIdx];

        // Candidate set: every point EXCEPT the deepest (which is locked in), evaluated by which
        // combination of 3 maximizes the quadrilateral area with the deepest point as the 4th
        // corner. With exactly 4 existing + 1 candidate - 1 deepest = 4 remaining candidates for 3
        // slots, there are exactly 4 possible triples (each omitting one candidate) - enumerate
        // all 4 directly rather than a general combinatorial search.
        const pool = [];
        for (let i = 0; i < this.points.length; i++) if (i !== deepestIdx) pool.push({ point: this.points[i], local: this._localAnchors[i] });
        if (!deepestIsCandidate) pool.push({ point: candidatePoint, local: candidateLocal });
        // pool now has exactly 4 entries (3 existing non-deepest + the candidate, when the
        // candidate isn't itself deepest) - or 4 existing non-deepest entries (when the candidate
        // IS deepest, so all 4 existing points are "remaining" and the candidate is locked in).

        let bestOmit = 0, bestArea = -1;
        for (let omit = 0; omit < pool.length; omit++) {
            const tri = [];
            for (let i = 0; i < pool.length; i++) if (i !== omit) tri.push(pool[i]);
            const area = ContactManifold._quadArea(deepestPoint.point, tri[0].point.point, tri[1].point.point, tri[2].point.point);
            if (area > bestArea) { bestArea = area; bestOmit = omit; }
        }

        const kept = [];
        for (let i = 0; i < pool.length; i++) if (i !== bestOmit) kept.push(pool[i]);

        this.points = deepestIsCandidate ? [candidatePoint] : [deepestPoint];
        this._localAnchors = deepestIsCandidate ? [candidateLocal] : [this._localAnchors[deepestIdx]];
        for (let i = 0; i < kept.length; i++) { this.points.push(kept[i].point); this._localAnchors.push(kept[i].local); }
    }

    // Rough quadrilateral area for the 4 candidate corners (order doesn't need to be a proper
    // convex hull walk here - the sum of the two diagonal-split triangle areas is a fine proxy for
    // "how spread out is this point set", which is all the reduction heuristic needs).
    static _quadArea(a, b, c, d) {
        return ContactManifold._triArea(a, b, c) + ContactManifold._triArea(a, c, d);
    }

    static _triArea(a, b, c) {
        const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
        const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
        const cx = aby * acz - abz * acy, cy = abz * acx - abx * acz, cz = abx * acy - aby * acx;
        return 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
    }

    // World point -> bodyA-local space, for next-tick matching. Allocation kept minimal (one
    // Vector3 per call) - matching runs once per tick per manifold, not in a hot per-substep loop.
    static _toLocal(bodyA, worldPoint) {
        const rel = Vector3.subInto(new Vector3(), worldPoint, bodyA.position);
        const invRot = new Quaternion().copy(bodyA.rotation).invert();
        invRot.transformVectorInPlace(rel);
        return rel;
    }
}

ContactManifold._scratchNormal = new Vector3();

ActionPhysics.ContactManifold = ContactManifold;


// ==== src/collision/ContactManifoldList.js ====
/**
 * ContactManifoldList: the full set of active ContactManifolds, keyed by body pair.
 *
 * One manifold per (bodyA, bodyB) pair — a pair with multiple candidate primitive contacts
 * (e.g. a compound body touching another shape at two of its children) accumulates all of THAT
 * tick's contacts into the SAME manifold via update(), since the manifold's own 4-point cap and
 * matching already do the right thing with several new points at once.
 *
 * Same ownership discipline as ContactManifold itself: refresh() is called once per TICK by
 * narrowphase, never per substep. A manifold that ends the tick with zero points (nothing matched,
 * nothing new) is removed from the list here — this is the ONE place a manifold itself is retired,
 * mirroring ContactManifold's own "only update() removes a point" rule one level up.
 */
class ContactManifoldList {
    constructor() {
        this._manifolds = new Map(); // "idA:idB" (idA < idB) -> ContactManifold
    }

    static _key(bodyA, bodyB) {
        return bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
    }

    // Returns the existing manifold for (bodyA, bodyB), creating one if this is a new pair. The
    // returned manifold's bodyA/bodyB are stored in a CANONICAL order (lower id first) so a
    // pair's local-space matching anchor (ContactManifold._toLocal uses bodyA) stays consistent
    // regardless of which order a caller happens to pass the two bodies in from tick to tick.
    getOrCreate(bodyA, bodyB) {
        const key = ContactManifoldList._key(bodyA, bodyB);
        let m = this._manifolds.get(key);
        if (!m) {
            const first = bodyA.id < bodyB.id ? bodyA : bodyB;
            const second = bodyA.id < bodyB.id ? bodyB : bodyA;
            m = new ContactManifold(first, second);
            this._manifolds.set(key, m);
        }
        return m;
    }

    // Applies this tick's contacts (grouped by body pair) to their manifolds, then drops any
    // manifold left with zero points. `contactsByPair` is a Map from "idA:idB" key (matching
    // _key's own canonical ordering) to an array of ContactDetails for that pair this tick. A pair
    // with a manifold but no entry in `contactsByPair` this tick (nothing detected at all) is
    // treated the same as an entry with an empty array - both result in every existing point
    // failing to match and the manifold being pruned.
    refresh(contactsByPair) {
        for (const [key, manifold] of this._manifolds) {
            const contacts = contactsByPair.get(key) || [];
            manifold.update(contacts);
            if (manifold.pointCount === 0) this._manifolds.delete(key);
        }
        // New pairs (a key present in contactsByPair but with no manifold yet) are created by the
        // caller via getOrCreate() before calling refresh() - see Narrowphase's own dispatch loop,
        // which must look up/create the manifold to know where to route each contact in the first
        // place. refresh() only ever prunes and updates EXISTING manifolds; getOrCreate() is the
        // sole entry point for new ones, keeping "one owner" for manifold creation too.
    }

    // All manifolds with at least one point, for the solver to iterate.
    values() {
        return this._manifolds.values();
    }

    get size() { return this._manifolds.size; }
}

ActionPhysics.ContactManifoldList = ContactManifoldList;


// ==== src/phases/NarrowPhase.js ====
/**
 * NarrowPhase: dispatch layer tying Midphase's primitive-shape pairs through GJK/EPA into
 * ContactDetails, and routing them into the right ContactManifold.
 *
 * Produces: contacts with accurate point, normal, signed distance (plan.md, Narrowphase contract).
 * May assume the pair is worth testing (a broadphase/midphase candidate). Must never cull contacts
 * for staleness, clamp depth, or second-guess its own math - that discipline lives entirely in
 * GJK/EPA/ContactDetails already; this file only wires them together and routes results into
 * manifolds. It owns exactly one thing of its own: one MinkowskiSupport/GJK/EPA instance PER
 * BODY-PAIR SLOT (reused across ticks for that slot, never shared across different pairs live at
 * the same time), and grouping this tick's contacts by canonical pair key before handing them to
 * ContactManifoldList.refresh().
 */
class NarrowPhase {
    // Base speculative margin (metres): a contact is reported once its signed distance is within
    // this of touching, even while still SEPARATED, so a manifold point exists BEFORE overlap
    // occurs. This is the whole mechanism of speculative contacts (plan.md, "Continuous collision /
    // speculative contacts" and the derived-velocity fix): the solver's non-penetration constraint,
    // evaluated every substep against the body's PREDICTED position, needs a point already present
    // to stop the body AT touch instead of first letting it dig in and then digging it back out
    // (the deep-correction -> large derived velocity failure the base margin prevents). Per-pair,
    // the base is widened by how far the pair can actually close in one tick (|v_rel| * dt) so a
    // fast body's contact is still caught a full tick ahead - see step().
    static SPECULATIVE_BASE = 0.02;

    constructor() {
        this.manifolds = new ContactManifoldList();
        this._dt = 1 / 60; // set each tick by step(); the fallback only matters if step() is never called
        // Scratch GJK/EPA instances, reused across every pair tested this tick. Safe because
        // narrowphase runs pairs one at a time (never two GJK.run() calls interleaved) - see
        // plan.md's scratch-memory rule: per-stage arena, not a global shared across unrelated
        // algorithms. These belong to NarrowPhase alone.
        this._gjk = new GJK();
        this._epa = new EPA();
        this._contactPool = []; // reused ContactDetails objects, grown as needed, never shrunk
        this._poolIndex = 0;
    }

    _nextPooledContact() {
        if (this._poolIndex >= this._contactPool.length) this._contactPool.push(new ContactDetails());
        return this._contactPool[this._poolIndex++];
    }

    // Runs narrowphase for one tick: broadphase pairs in, manifolds refreshed out.
    //   broadphasePairs: [[bodyA, bodyB], ...] from SAPBroadphase.computePairs()
    //   midphase: a Midphase instance (expands compound/mesh pairs to primitives)
    //   dt: this tick's timestep, used to size the per-pair speculative margin (how far the pair
    //       can close in one tick). Optional; falls back to the last value / 1/60 if omitted.
    step(broadphasePairs, midphase, dt) {
        if (dt) this._dt = dt;
        this._poolIndex = 0;
        const contactsByPair = new Map(); // canonical "idA:idB" key -> ContactDetails[]

        for (let p = 0; p < broadphasePairs.length; p++) {
            const bodyA = broadphasePairs[p][0], bodyB = broadphasePairs[p][1];
            const primitivePairs = midphase.expandPair(bodyA, bodyB);
            const key = bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
            const margin = this._speculativeMargin(bodyA, bodyB);

            for (let i = 0; i < primitivePairs.length; i++) {
                const contact = this._testPrimitivePair(primitivePairs[i].a, primitivePairs[i].b);
                // signedDistance: positive = overlapping, negative = separated by that gap (plan.md
                // convention). Report the contact while overlapping OR within the speculative
                // margin of touching; drop it only once the gap exceeds the margin - too far this
                // tick for the pair to reach, so no manifold point is warranted. This is the ONE
                // place narrowphase decides a pair is "not worth a manifold entry" (see
                // _testPrimitivePair) and it is a distance-vs-margin test, never a staleness or
                // depth-quality judgement (Rule 1: narrowphase reports geometry, it does not
                // second-guess the solver's use of it).
                if (contact.signedDistance < -margin) continue;
                let list = contactsByPair.get(key);
                if (!list) { list = []; contactsByPair.set(key, list); }
                list.push(contact);
            }

            // Ensure a manifold exists for this pair even if this tick found zero contacts for it
            // yet, so ContactManifoldList.refresh() below has something to prune if the pair was
            // previously touching and just separated. getOrCreate() is idempotent for an existing
            // pair.
            this.manifolds.getOrCreate(bodyA, bodyB);
        }

        this.manifolds.refresh(contactsByPair);
        return this.manifolds;
    }

    // Per-pair speculative margin: the base margin widened by how far the two bodies can close
    // along their relative velocity in one tick (|v_rel| * dt). A slow/resting pair uses ~the base;
    // a fast approach gets a proportionally larger lookahead so the contact is still reported a full
    // tick before overlap - which is exactly what lets the solver stop the body at touch rather
    // than after it has tunnelled partway in. Uses the full relative speed (not the normal
    // component - narrowphase has no single contact normal yet at this point, and the closing speed
    // is an upper bound on approach along any normal, so it never UNDER-estimates the needed
    // lookahead, matching broadphase's own no-false-negatives discipline).
    _speculativeMargin(bodyA, bodyB) {
        const dvx = bodyA.linear_velocity.x - bodyB.linear_velocity.x;
        const dvy = bodyA.linear_velocity.y - bodyB.linear_velocity.y;
        const dvz = bodyA.linear_velocity.z - bodyB.linear_velocity.z;
        const relSpeed = Math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
        // Add each body's angular corner speed (|omega|*R): a contact FEATURE on a spinning body
        // (a corner, an edge) approaches at up to this rate even when the centre barely moves, so a
        // margin sized only from centre-relative linear speed reports the contact too late for a
        // tipping body - the corner is already deep. Same reasoning as the broadphase AABB's angular
        // sweep (RigidBody._recomputeBroadphaseAABB); both stages must look ahead by the same
        // corner motion or broadphase surfaces the pair and narrowphase then drops it as "too far".
        const angSpeed = NarrowPhase._angularCornerSpeed(bodyA) + NarrowPhase._angularCornerSpeed(bodyB);
        return NarrowPhase.SPECULATIVE_BASE + (relSpeed + angSpeed) * this._dt;
    }

    // Upper bound on how fast any point on `body` moves purely from its rotation: |omega| times the
    // body's bounding radius (farthest corner of its tight AABB from centre).
    static _angularCornerSpeed(body) {
        const w = body.angular_velocity;
        const wMag = Math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        if (wMag === 0) return 0;
        const aabb = body.getAABB();
        const ex = (aabb.max.x - aabb.min.x) * 0.5, ey = (aabb.max.y - aabb.min.y) * 0.5, ez = (aabb.max.z - aabb.min.z) * 0.5;
        return wMag * Math.sqrt(ex * ex + ey * ey + ez * ez);
    }

    // Re-measures the GEOMETRY of the contact points already in each manifold against the bodies'
    // CURRENT (predicted, mid-substep) transforms, in place. The solver calls this once per substep
    // (see Solver.step's `refresh`), so a contact whose feature moves as a body rotates is solved
    // against live geometry rather than a frozen tick-start normal/anchor - the fix for the
    // rotational derived-velocity blow-up on corner contacts.
    //
    // This updates geometry ONLY. It never adds, removes, or re-matches points, and never touches
    // the manifold's point SET or its warm-start lambda - that ownership stays with the manifold's
    // once-per-tick update() (plan.md's rule against mid-tick manifold churn). A point whose contact
    // has genuinely separated this substep simply gets a negative signed distance here and the
    // solver's own C<=0 guard makes it inert; it is not culled mid-tick.
    //
    // Only primitive-vs-primitive body pairs are refreshed. A compound/mesh body's contact came
    // from an expanded child/triangle whose identity this method does not track per point, so those
    // manifolds keep their tick-start geometry (no regression - that is exactly today's behaviour
    // for every contact). Re-expanding compounds/meshes per substep is a later optimisation if
    // rotating compound bodies turn out to need it.
    refreshManifoldGeometry(manifolds) {
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            if (this._isCompoundOrMesh(bodyA.shape) || this._isCompoundOrMesh(bodyB.shape)) continue;

            const placedA = { shape: bodyA.shape, position: bodyA.position, rotation: bodyA.rotation };
            const placedB = { shape: bodyB.shape, position: bodyB.position, rotation: bodyB.rotation };
            const fresh = this._testPrimitivePair(placedA, placedB); // one fresh contact for this pair

            // Update the nearest existing point (in world space) with the fresh geometry, keeping
            // its warm-start lambda and its persistent local anchors' IDENTITY - only the values
            // the solver reads live (normal, and the anchors it recomputes C from) are refreshed.
            let best = null, bestDistSq = Infinity;
            for (let i = 0; i < manifold.points.length; i++) {
                const p = manifold.points[i];
                const dx = p.point.x - fresh.point.x, dy = p.point.y - fresh.point.y, dz = p.point.z - fresh.point.z;
                const d = dx * dx + dy * dy + dz * dz;
                if (d < bestDistSq) { bestDistSq = d; best = p; }
            }
            if (!best) continue;
            best.point.copy(fresh.point);
            best.pointOnA.copy(fresh.pointOnA);
            best.pointOnB.copy(fresh.pointOnB);
            best.signedDistance = fresh.signedDistance;
            // Keep the ESTABLISHED normal through the exact-touch band, exactly as the manifold's
            // once-per-tick update() does (ContactManifold.EXACT_TOUCH_BAND) - the per-substep
            // refresh MUST honour the same rule, or it silently clobbers a good resting normal with
            // the ambiguous diagonal GJK/EPA returns at signed-distance ~0 every substep, which is
            // the penetrate-then-launch bug the once-per-tick guard was added to prevent (it bit
            // spheres hard: a flush sphere kept getting a (-0.71,0,0.71) normal and launched to y=200+).
            if (Math.abs(fresh.signedDistance) >= ContactManifold.EXACT_TOUCH_BAND) best.normal.copy(fresh.normal);
            // Re-anchor to the CURRENT geometry so C is measured from where the feature is NOW - the
            // whole point of refreshing. Persisting stale anchors would defeat it. Lambda is left
            // untouched (warm start survives).
            best.setLocalAnchors(bodyA, bodyB);
        }
    }

    _isCompoundOrMesh(shape) {
        return (typeof CompoundShape !== 'undefined' && shape instanceof CompoundShape) ||
            (typeof MeshShape !== 'undefined' && shape instanceof MeshShape);
    }

    // Runs GJK (and EPA if overlapping) for one primitive-shape pair, returning a pooled
    // ContactDetails. Always returns a filled contact carrying its signed distance (positive =
    // overlapping, negative = separated gap); the caller (step) decides whether that distance is
    // within the speculative margin and thus worth a manifold entry. This function itself never
    // culls - it only measures (Rule 1).
    _testPrimitivePair(placedA, placedB) {
        const support = new MinkowskiSupport(placedA, placedB);
        const gjkResult = this._gjk.run(support);
        const contact = this._nextPooledContact();
        if (gjkResult.overlapping) {
            const epaResult = this._epa.run(support, gjkResult.simplex);
            contact.setFromEPA(epaResult);
        } else {
            contact.setFromGJKSeparated(gjkResult);
        }
        return contact;
    }
}

ActionPhysics.NarrowPhase = NarrowPhase;


// ==== src/solver/Solver.js ====
/**
 * XPBD solver. One solver, per plan.md ("Solver: XPBD, one solver only") - no PGS fallback exists
 * or is planned. Written from the algorithm (Muller et al., "Detailed Rigid Body Simulation with
 * Extended Position Based Dynamics", 2020; Macklin et al.'s earlier XPBD paper for the compliance
 * formulation) - same reasoning as GJK/EPA: this is stage 6, one of the two stages plan.md names
 * as where the engine is won or lost, and a structural anchor to the predecessor's PGS-shaped
 * implementation would be exactly the wrong thing to carry over.
 *
 * THE CENTRAL DESIGN RULE (plan.md, "Solver: XPBD, one solver only" and the pipeline table):
 * velocity is DERIVED from position (v = (x - x_prev) / h) and used RAW - no clamp, no slop, no
 * per-body governor, anywhere in this file. If a body's derived velocity is ever wrong, that is a
 * NARROWPHASE bug (bad contact depth/normal), and the fix belongs there, not here. This is the
 * lesson from Goblin's derived-velocity problem (plan.md's own extended writeup): three clamp
 * variants were tried and each traded one failure for another, because the clamp was never the
 * real fix - a box arriving already 0.089 deep (five substeps of undetected travel) was the bug,
 * and clamping the resulting spin only hid it. Detection quality is what actually prevents that
 * here, not this file.
 *
 * NO POINT-COUNT DIVISOR (plan.md, Bug reference / Solver): each contact point's own accumulated
 * lambda already does the job a divisor was invented to do (stop N-point overcorrection) -
 * removing that compensating term in the predecessor dropped iterations from 15 to 1 and frame
 * cost from 8.43ms to 6.87ms. No division by point count appears anywhere below.
 *
 * SUBSTEPPING: gravity/forces integrate once per substep; each substep runs its own XPBD position
 * solve (a fixed small iteration count per substep, not many iterations of a single big step) -
 * this is the "many small steps" XPBD formulation, not "few steps with many solver iterations."
 */
class Solver {
    constructor(opts) {
        opts = opts || {};
        this.substeps = opts.substeps || 4;
        this.iterations = opts.iterations || 1; // position-solve passes PER SUBSTEP
        // Scratch, owned entirely by the solver (plan.md: per-stage arena, never a shared global).
        this._rA = new Vector3(); this._rB = new Vector3();
        this._deltaPos = new Vector3();
        this._impulse = new Vector3();
        this._tangent1 = new Vector3(); this._tangent2 = new Vector3();
        this._angularCorrA = new Vector3(); this._angularCorrB = new Vector3();
        this._tmpDispA = new Vector3(); this._tmpDispB = new Vector3(); this._tmpPrev = new Vector3(); // friction slip scratch
        this._prevPos = new Map(); // bodyId -> Vector3, this substep's PRE-integration position
        this._prevRot = new Map(); // bodyId -> Quaternion, this substep's PRE-integration rotation
    }

    /**
     * Advances every dynamic body in `bodies` by `dt`, resolving the contacts in `manifolds`
     * (a ContactManifoldList). Gravity is `gravity` (a Vector3) unless a body overrides it via
     * RigidBody.gravity. May assume every contact's depth/normal is accurate (Rule 1) - never
     * re-checks or discounts a contact's own geometry.
     *
     * `refresh(manifolds)`, if given, re-measures each existing contact point's geometry (normal,
     * anchors, depth) against the predicted positions once per substep, before the constraint
     * solve - the interleaved detect-then-solve that makes rotating/corner contacts stable. It must
     * only update existing points' geometry, never add/remove/re-match them (that is the manifold's
     * once-per-tick job). Omitted -> tick-start geometry is reused every substep.
     */
    step(bodies, manifolds, gravity, dt, refresh) {
        const h = dt / this.substeps;
        for (let s = 0; s < this.substeps; s++) {
            this._substep(bodies, manifolds, gravity, h, refresh);
        }
    }

    _substep(bodies, manifolds, gravity, h, refresh) {
        // 1. Integrate velocities (gravity/forces) and predict positions.
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            this._prevPos.set(b.id, new Vector3().copy(b.position));
            this._prevRot.set(b.id, new Quaternion().copy(b.rotation));

            const g = b.gravity || gravity;
            b.linear_velocity.x += g.x * h * b.linear_factor.x;
            b.linear_velocity.y += g.y * h * b.linear_factor.y;
            b.linear_velocity.z += g.z * h * b.linear_factor.z;

            // Damping applied to velocity before the position predict, same substep - standard
            // XPBD ordering (Muller et al. section 3.1).
            if (b.linear_damping > 0) b.linear_velocity.scaleInPlace(Math.max(0, 1 - b.linear_damping * h));
            if (b.angular_damping > 0) b.angular_velocity.scaleInPlace(Math.max(0, 1 - b.angular_damping * h));

            b.position.addScaledInPlace(b.linear_velocity, h);
            Solver._integrateRotation(b.rotation, b.angular_velocity, h);
            // The world inverse inertia tensor depends on the rotation, which just changed - refresh
            // it so _effectiveMass and _applyAngularCorrection this substep use the CURRENT
            // orientation, not the tick-start one. Cheap (one 3x3 similarity transform) and keeps
            // the angular math consistent with the per-substep geometry refresh below; a fast-
            // rotating body would otherwise solve against an orientation several substeps stale.
            b._recomputeWorldInverseInertia();
        }

        // 1b. Re-measure contact geometry against the just-predicted positions. This is the
        // interleaved detect-then-solve that keeps rotating/corner contacts stable: without it the
        // contact's normal and anchors are frozen at tick-start, so a body that rotates fast enough
        // for its contact CORNER to move between substeps gets solved against stale geometry, its
        // far corner slams in undetected, and the one-shot correction of the resulting deep overlap
        // injects a large derived angular velocity (which only grows as the substep shrinks - the
        // rotational form of the derived-velocity problem). `refresh` re-measures geometry ONLY; it
        // never adds/removes/re-matches points (that stays the manifold's once-per-tick job). See
        // step()'s doc and World.step.
        if (refresh) refresh(manifolds);

        // 1c. Capture the pre-solve contact-relative NORMAL velocity, for restitution. It has to be
        // read here - after gravity integration and geometry refresh, but BEFORE the normal position
        // solve removes it - because restitution restores a fraction of the velocity the body was
        // approaching at, which the solve is about to zero.
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            for (let i = 0; i < manifold.points.length; i++) {
                const p = manifold.points[i];
                p._preSolveNormalVel = this._contactRelativeNormalVelocity(p, bodyA, bodyB);
            }
        }

        // 2. Reset this substep's accumulated lambda to zero (XPBD's lambda is PER-SUBSTEP, reset
        // every substep, not carried across substeps within a tick - only carried across TICKS via
        // the manifold's warm start, which primes the solver's FIRST guess but each substep's own
        // constraint solve still starts its own lambda accumulation at zero for THIS substep's
        // Lagrange multiplier). See Muller et al. section 3.3.
        for (const manifold of manifolds.values()) {
            for (let i = 0; i < manifold.points.length; i++) {
                manifold.points[i].normalLambda = 0;
                manifold.points[i].tangentLambda1 = 0;
                manifold.points[i].tangentLambda2 = 0;
            }
        }

        // 3. Position-level constraint solve: contacts (normal / non-penetration only).
        for (let iter = 0; iter < this.iterations; iter++) {
            for (const manifold of manifolds.values()) {
                this._solveManifold(manifold, h);
            }
        }

        // 4. Derive velocity from the position change - RAW, no clamp (see class header). This
        // captures the normal solve's effect (a stopped body has ~zero normal velocity here).
        for (let i = 0; i < bodies.length; i++) {
            const b = bodies[i];
            if (b.bodyType !== RigidBody.DYNAMIC) continue;
            const prevPos = this._prevPos.get(b.id);
            const prevRot = this._prevRot.get(b.id);
            b.linear_velocity.x = (b.position.x - prevPos.x) / h;
            b.linear_velocity.y = (b.position.y - prevPos.y) / h;
            b.linear_velocity.z = (b.position.z - prevPos.z) / h;
            Solver._deriveAngularVelocity(b.angular_velocity, prevRot, b.rotation, h);
        }

        // 5. Friction + restitution, applied in the VELOCITY pass (Muller et al. 2020 sec 3.6). This
        // is NOT the forbidden derived-velocity clamp (that hid a detection bug by governing the
        // whole body's velocity); it is the physical contact velocity constraint - friction removes
        // tangential relative velocity up to the Coulomb limit, restitution restores a fraction of
        // the pre-solve approach velocity. Both act only at contacts and only on the contact-relative
        // velocity, which is exactly where they belong.
        for (const manifold of manifolds.values()) {
            const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
            for (let i = 0; i < manifold.points.length; i++) {
                this._solveContactVelocity(manifold.points[i], bodyA, bodyB, h);
            }
        }
    }

    // this = this * (0 + w) * h * 0.5, then normalize - standard PBD quaternion integration
    // (Muller et al.). Written directly rather than via a general quaternion-derivative helper
    // since it's the one place this exact operation is needed.
    static _integrateRotation(rotation, angularVelocity, h) {
        const wx = angularVelocity.x, wy = angularVelocity.y, wz = angularVelocity.z;
        const qx = rotation.x, qy = rotation.y, qz = rotation.z, qw = rotation.w;
        const half = h * 0.5;
        rotation.x = qx + half * (wx * qw + wy * qz - wz * qy);
        rotation.y = qy + half * (wy * qw + wz * qx - wx * qz);
        rotation.z = qz + half * (wz * qw + wx * qy - wy * qx);
        rotation.w = qw + half * (-wx * qx - wy * qy - wz * qz);
        rotation.normalize();
    }

    // Derived angular velocity from the rotation delta between prevRot and rotation, into `out`.
    // omega = 2 * (q_new * conj(q_prev)).xyz / h, sign-corrected so the shorter rotation path is
    // always taken (a quaternion and its negation represent the same rotation, but the derivative
    // formula needs a consistent sign to avoid spurious 2x-angle spins).
    static _deriveAngularVelocity(out, prevRot, rotation, h) {
        let dqx = rotation.w * (-prevRot.x) + rotation.x * prevRot.w + rotation.y * (-prevRot.z) - rotation.z * (-prevRot.y);
        let dqy = rotation.w * (-prevRot.y) - rotation.x * (-prevRot.z) + rotation.y * prevRot.w + rotation.z * (-prevRot.x);
        let dqz = rotation.w * (-prevRot.z) + rotation.x * (-prevRot.y) - rotation.y * (-prevRot.x) + rotation.z * prevRot.w;
        let dqw = rotation.w * prevRot.w - rotation.x * (-prevRot.x) - rotation.y * (-prevRot.y) - rotation.z * (-prevRot.z);
        if (dqw < 0) { dqx = -dqx; dqy = -dqy; dqz = -dqz; } // shorter path
        out.x = 2 * dqx / h; out.y = 2 * dqy / h; out.z = 2 * dqz / h;
    }

    _solveManifold(manifold, h) {
        const bodyA = manifold.bodyA, bodyB = manifold.bodyB;
        for (let i = 0; i < manifold.points.length; i++) {
            this._solvePoint(manifold.points[i], bodyA, bodyB, h);
        }
    }

    // One XPBD position-constraint solve for a single contact point.
    //
    // C is recomputed HERE, every call, from the bodies' CURRENT positions via each point's fixed
    // local anchor offsets (ContactDetails.currentAnchorAInto/B) - never read directly from
    // point.signedDistance, which is a snapshot from this TICK's single narrowphase pass, already
    // stale by the second substep. Reusing that stale value was a real bug caught while building
    // this file: a resting box's true overlap grows a little each substep as gravity keeps pulling
    // it down, but a stale, constant C applied every substep with no correction ever making it
    // smaller (because "smaller" was never re-measured) still gets a FRESH deltaLambda each call -
    // the same one, over and over - overcorrecting the box upward well past resting, tick after
    // tick, with nothing to signal "this is done" because the signal (C actually shrinking as the
    // real overlap resolves) never arrived. Recomputing C live is what makes convergence within a
    // tick's substeps possible at all: each correction actually reduces the NEXT measured C.
    //
    // Compliance is 0 here (rigid, infinitely stiff contact) - plan.md's own open question
    // ("whether compliance is ever non-zero in practice") is left open; 0 is the correct default
    // for a contact that should not be springy.
    _solvePoint(point, bodyA, bodyB, h) {
        point.currentAnchorAInto(this._rA, bodyA);
        point.currentAnchorBInto(this._rB, bodyB);
        const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;
        // normal points from B to A (matching GJK/EPA's own convention - verified directly
        // against GJK's separated-result normal after finding and fixing a real sign bug in EPA's
        // own output, see EPA.js). C = (anchorB - anchorA) . normal is positive exactly when B's
        // anchor has moved PAST A's anchor in the normal's own direction - i.e. penetrating.
        const C = (this._rB.x - this._rA.x) * nx + (this._rB.y - this._rA.y) * ny + (this._rB.z - this._rA.z) * nz;
        // SPECULATIVE CONTACT (plan.md, "Continuous collision / speculative contacts"): this guard,
        // combined with the point being detected BEFORE overlap (narrowphase's speculative margin),
        // IS the speculative mechanism - no separate code path. The point is created while still
        // separated (negative signedDistance), so it already exists in the manifold. Then every
        // substep the position predict (Solver._substep step 1) moves the body forward FIRST, and C
        // is re-measured HERE against that predicted position. If the predicted motion overshot the
        // touching plane, C > 0 and the constraint pulls the body back to exactly touch (C = 0) -
        // never deeper, because C is the live overshoot, not a whole tick's accumulated penetration.
        // If the predicted motion did NOT reach touch (C <= 0, still a real gap), there is nothing
        // to correct this substep: a non-penetration contact only ever PUSHES APART, it never pulls
        // a separated pair together across a gap. That is the entire fix for the derived-velocity
        // problem: Δx per substep is now just the small overshoot beyond touch, so derived velocity
        // v = Δx/h stays small, with no clamp anywhere (the central design rule holds).
        if (C <= 0) return;

        // rA/rB for the effective-mass and angular-correction math are offsets from each body's
        // CENTER (not the anchor's world position itself) - overwrite _rA/_rB in place now that C
        // has been read from their anchor positions above.
        Vector3.subInto(this._rA, this._rA, bodyA.position);
        Vector3.subInto(this._rB, this._rB, bodyB.position);

        // Effective inverse mass along the normal: w = 1/mA + 1/mB + (rA x n)·I_A^-1·(rA x n) +
        // (rB x n)·I_B^-1·(rB x n) - the standard generalized inverse mass for a linear+angular
        // constraint (Muller et al. section 3.2, eq. 8).
        const wSum = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
        if (wSum < 1e-12) return; // both bodies effectively immovable along this normal - nothing to solve

        // XPBD Lagrange multiplier update, rigid (compliance-free) contact: deltaLambda =
        // -C / wSum (alpha/h^2 term drops out entirely when compliance is 0 - Muller et al. eq 4).
        // Sign note for this file's convention: C > 0 means penetrating (guarded above), so
        // deltaLambda < 0, and _applyPositionalCorrection's verified pairing turns a NEGATIVE
        // deltaLambda into a push-apart correction. A pushing contact therefore accumulates a
        // NEGATIVE normalLambda here - the opposite sign from the textbook's non-negative
        // convention, purely because the normal points B->A rather than A->B. The physical
        // constraint "a contact can push apart but never pull together" is therefore normalLambda
        // <= 0: clamp the accumulated value at 0 from above and feed back only the change actually
        // applied, so an over-correction on a later iteration/substep can relax the push but can
        // never invert it into an attractive pull across the contact.
        const oldLambda = point.normalLambda;
        let newLambda = oldLambda - C / wSum;
        if (newLambda > 0) newLambda = 0; // contact cannot pull; clamp to no-push
        const deltaLambda = newLambda - oldLambda;
        point.normalLambda = newLambda;

        this._applyPositionalCorrection(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, deltaLambda);
    }

    // Generalized inverse mass along direction (dx,dy,dz) for the pair, combining linear and
    // angular contributions from both bodies. Shared by the normal and each friction direction.
    _effectiveMass(bodyA, bodyB, rA, rB, dx, dy, dz) {
        // linear_factor is a per-axis WORLD-space velocity mask, but this constraint's own
        // direction is a single world vector - exact per-axis locking against an arbitrary contact
        // normal isn't captured by a single isotropic scale, so the isotropic inverse mass is used
        // directly here. Locked axes are a character-controller feature (movement clamped to a
        // plane), not something a general rigid-body contact normal needs to interact with
        // precisely - revisit if that combination turns out to matter.
        let w = bodyA._massInverted + bodyB._massInverted;

        const rax = rA.y * dz - rA.z * dy, ray = rA.z * dx - rA.x * dz, raz = rA.x * dy - rA.y * dx;
        const rbx = rB.y * dz - rB.z * dy, rby = rB.z * dx - rB.x * dz, rbz = rB.x * dy - rB.y * dx;

        if (bodyA._massInverted > 0) {
            const IA = bodyA._worldInverseInertiaTensor;
            const ix = IA.e00 * rax + IA.e01 * ray + IA.e02 * raz;
            const iy = IA.e10 * rax + IA.e11 * ray + IA.e12 * raz;
            const iz = IA.e20 * rax + IA.e21 * ray + IA.e22 * raz;
            w += rax * ix + ray * iy + raz * iz;
        }
        if (bodyB._massInverted > 0) {
            const IB = bodyB._worldInverseInertiaTensor;
            const ix = IB.e00 * rbx + IB.e01 * rby + IB.e02 * rbz;
            const iy = IB.e10 * rbx + IB.e11 * rby + IB.e12 * rbz;
            const iz = IB.e20 * rbx + IB.e21 * rby + IB.e22 * rbz;
            w += rbx * ix + rby * iy + rbz * iz;
        }
        return w;
    }

    // Applies the position correction dLambda * n (and the matching angular correction) to both
    // bodies, scaled by their own inverse mass/inertia - standard XPBD position update (Muller et
    // al. eq 6-7). Sign convention: correction pushes A along +n and B along -n, matching normal's
    // B-to-A direction.
    // C = (anchorB - anchorA) . n (see _solvePoint), so the constraint GRADIENT is dC/d(bodyA
    // position) = -n and dC/d(bodyB position) = +n. XPBD's update Δx = Δλ * w * ∇C (Muller et al.
    // eq 6) therefore moves A along -n*Δλ and B along +n*Δλ - the OPPOSITE pairing from what a
    // naive "A gets +n, B gets -n" guess would produce. Verified against a concrete case: a box
    // resting ON TOP of static ground, overlapping by a small amount, with normal pointing from B
    // (the box) to A (the ground) - i.e. DOWNWARD. deltaLambda comes out negative for a positive
    // overlap C, and the box (body B) must move UP to resolve it: +n*deltaLambda with n pointing
    // down and deltaLambda negative gives a positive (upward) y-component - correct only with
    // THIS pairing, not the reversed one that was here before (which pushed the box down, through
    // the ground, for the same inputs - the actual fall-through bug this comment is fixing).
    _applyPositionalCorrection(bodyA, bodyB, rA, rB, nx, ny, nz, dLambda) {
        const px = nx * dLambda, py = ny * dLambda, pz = nz * dLambda;

        if (bodyA._massInverted > 0) {
            bodyA.position.x -= px * bodyA._massInverted * bodyA.linear_factor.x;
            bodyA.position.y -= py * bodyA._massInverted * bodyA.linear_factor.y;
            bodyA.position.z -= pz * bodyA._massInverted * bodyA.linear_factor.z;
            this._applyAngularCorrection(bodyA, rA, -px, -py, -pz);
        }
        if (bodyB._massInverted > 0) {
            bodyB.position.x += px * bodyB._massInverted * bodyB.linear_factor.x;
            bodyB.position.y += py * bodyB._massInverted * bodyB.linear_factor.y;
            bodyB.position.z += pz * bodyB._massInverted * bodyB.linear_factor.z;
            this._applyAngularCorrection(bodyB, rB, px, py, pz);
        }
    }

    // Rotates `body` by the small-angle correction (I^-1 * (r x p)) * 0.5, the standard PBD
    // angular position update from a linear positional impulse applied at offset r (Muller et al.
    // eq 7-8, via the same quaternion-derivative integration _integrateRotation uses).
    _applyAngularCorrection(body, r, px, py, pz) {
        const torqueX = r.y * pz - r.z * py, torqueY = r.z * px - r.x * pz, torqueZ = r.x * py - r.y * px;
        const I = body._worldInverseInertiaTensor;
        const wx = I.e00 * torqueX + I.e01 * torqueY + I.e02 * torqueZ;
        const wy = I.e10 * torqueX + I.e11 * torqueY + I.e12 * torqueZ;
        const wz = I.e20 * torqueX + I.e21 * torqueY + I.e22 * torqueZ;
        this._angularCorrA.set(wx * body.angular_factor.x, wy * body.angular_factor.y, wz * body.angular_factor.z);
        Solver._integrateRotation(body.rotation, this._angularCorrA, 1); // h=1: this IS the delta, not a rate
    }

    // Velocity-pass contact solve: friction and restitution, applied to the contact-relative
    // velocity AFTER the position solve has set velocities (Muller et al. 2020 sec 3.6). Working in
    // velocity space here (not position) is what makes friction stable and gives true static stick:
    // a resting body's tangential contact velocity is driven to exactly zero, capped by the Coulomb
    // limit, so it does not creep - the position-anchor approach kept saturating its cap and letting
    // the body slide down a shallow slope. This is a physical contact constraint on the relative
    // velocity, NOT the derived-velocity clamp plan.md forbids (that governed a whole body's velocity
    // to hide a detection bug; this only removes the tangential rub and restores a chosen restitution
    // at the contact, which is exactly what friction and bounce ARE).
    _solveContactVelocity(point, bodyA, bodyB, h) {
        // Only act on a contact that is actually touching/overlapping right now (normalLambda < 0
        // means the normal solve pushed this substep). A purely speculative point that never engaged
        // has normalLambda == 0 and no contact velocity to correct.
        if (point.normalLambda >= 0) return;

        point.currentAnchorAInto(this._rA, bodyA);
        point.currentAnchorBInto(this._rB, bodyB);
        Vector3.subInto(this._rA, this._rA, bodyA.position);
        Vector3.subInto(this._rB, this._rB, bodyB.position);
        const nx = point.normal.x, ny = point.normal.y, nz = point.normal.z;

        // --- Restitution (normal) ---
        // Sign convention (normal points B->A): a body APPROACHING the contact has POSITIVE normal
        // relative velocity (relVel . normal > 0 = closing), and after a bounce it should SEPARATE at
        // -e * the approach speed (negative relN). Skip slow approaches (a resting body's one-substep
        // gravity nudge) via a threshold, so restitution does not turn rest into perpetual jitter.
        const restitution = Math.max(bodyA.restitution, bodyB.restitution); // combined: max, standard convention
        const relN = this._contactRelativeNormalVelocity(point, bodyA, bodyB);
        if (restitution > 0 && point._preSolveNormalVel > Solver.RESTITUTION_THRESHOLD) {
            const targetN = -restitution * point._preSolveNormalVel; // desired separating velocity (< 0)
            // Only ADD separation (make relN more negative); never damp an already-separating contact.
            if (targetN < relN) {
                const wN = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, nx, ny, nz);
                if (wN >= 1e-12) this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, nx, ny, nz, (targetN - relN) / wN);
            }
        }

        // --- Friction (tangent) ---
        const friction = Math.sqrt(bodyA.friction * bodyB.friction);
        if (friction <= 0) return;
        // Coulomb cap on the friction impulse magnitude: mu times the normal impulse the position
        // solve actually applied this substep. normalLambda is a position Lagrange multiplier;
        // dividing by h converts it to a velocity-space impulse commensurate with the tangential
        // impulses computed below. This is the correct, unit-consistent cap that the position-space
        // attempt never got right.
        const maxImpulse = friction * Math.abs(point.normalLambda) / h;
        if (maxImpulse <= 0) return;

        // Current tangential relative velocity, and the impulse that would zero it.
        this._contactRelativeVelocity(point, bodyA, bodyB, this._tmpDispA); // full relative velocity -> _tmpDispA
        const vn = this._tmpDispA.x * nx + this._tmpDispA.y * ny + this._tmpDispA.z * nz;
        let vtx = this._tmpDispA.x - vn * nx, vty = this._tmpDispA.y - vn * ny, vtz = this._tmpDispA.z - vn * nz;
        const vtMag = Math.sqrt(vtx * vtx + vty * vty + vtz * vtz);
        if (vtMag < 1e-12) return; // no tangential motion to resist

        const tx = vtx / vtMag, ty = vty / vtMag, tz = vtz / vtMag; // tangent = slip direction
        const wT = this._effectiveMass(bodyA, bodyB, this._rA, this._rB, tx, ty, tz);
        if (wT < 1e-12) return;
        // Impulse to fully stop the tangential velocity, clamped to the Coulomb cap (static stick
        // when the full stop is within budget, dynamic slide when clamped).
        let jt = vtMag / wT;
        if (jt > maxImpulse) jt = maxImpulse;
        // Apply along -tangent (oppose the slip).
        this._applyVelocityImpulse(bodyA, bodyB, this._rA, this._rB, -tx, -ty, -tz, jt);
    }

    // Contact-relative velocity (velocity of B's contact point minus A's), into `out`. rA/rB are the
    // center-relative contact offsets (already in this._rA/_rB when called from the velocity pass,
    // but recomputed here from the anchors so this is usable standalone).
    _contactRelativeVelocity(point, bodyA, bodyB, out) {
        point.currentAnchorAInto(this._tmpPrev, bodyA);
        this._tmpPrev.subInPlace(bodyA.position); // rA (center-relative)
        const va = this._pointVelocity(bodyA, this._tmpPrev, this._tmpDispB);
        const vax = va.x, vay = va.y, vaz = va.z;
        point.currentAnchorBInto(this._tmpPrev, bodyB);
        this._tmpPrev.subInPlace(bodyB.position); // rB
        const vb = this._pointVelocity(bodyB, this._tmpPrev, this._tmpDispB);
        out.set(vb.x - vax, vb.y - vay, vb.z - vaz);
        return out;
    }

    // Velocity of the material point at center-relative offset r on `body`: v + omega x r.
    _pointVelocity(body, r, out) {
        const w = body.angular_velocity, v = body.linear_velocity;
        out.set(
            v.x + (w.y * r.z - w.z * r.y),
            v.y + (w.z * r.x - w.x * r.z),
            v.z + (w.x * r.y - w.y * r.x)
        );
        return out;
    }

    // Contact-relative velocity along the normal (B->A). Scalar.
    _contactRelativeNormalVelocity(point, bodyA, bodyB) {
        this._contactRelativeVelocity(point, bodyA, bodyB, this._tmpDispA);
        return this._tmpDispA.x * point.normal.x + this._tmpDispA.y * point.normal.y + this._tmpDispA.z * point.normal.z;
    }

    // Apply a velocity-space impulse j*(dir) at contact offsets rA/rB: A gets -j (B->A convention,
    // matching the position correction's own pairing), B gets +j, each scaled by inverse mass, with
    // the matching angular velocity change. rA/rB are center-relative offsets.
    _applyVelocityImpulse(bodyA, bodyB, rA, rB, dx, dy, dz, j) {
        const px = dx * j, py = dy * j, pz = dz * j;
        if (bodyA._massInverted > 0) {
            bodyA.linear_velocity.x -= px * bodyA._massInverted * bodyA.linear_factor.x;
            bodyA.linear_velocity.y -= py * bodyA._massInverted * bodyA.linear_factor.y;
            bodyA.linear_velocity.z -= pz * bodyA._massInverted * bodyA.linear_factor.z;
            this._applyAngularVelocityImpulse(bodyA, rA, -px, -py, -pz);
        }
        if (bodyB._massInverted > 0) {
            bodyB.linear_velocity.x += px * bodyB._massInverted * bodyB.linear_factor.x;
            bodyB.linear_velocity.y += py * bodyB._massInverted * bodyB.linear_factor.y;
            bodyB.linear_velocity.z += pz * bodyB._massInverted * bodyB.linear_factor.z;
            this._applyAngularVelocityImpulse(bodyB, rB, px, py, pz);
        }
    }

    // Angular velocity change from a linear impulse p applied at center-relative offset r:
    // dOmega = I^-1 (r x p). (Velocity-space analog of _applyAngularCorrection.)
    _applyAngularVelocityImpulse(body, r, px, py, pz) {
        const tqx = r.y * pz - r.z * py, tqy = r.z * px - r.x * pz, tqz = r.x * py - r.y * px;
        const I = body._worldInverseInertiaTensor;
        body.angular_velocity.x += (I.e00 * tqx + I.e01 * tqy + I.e02 * tqz) * body.angular_factor.x;
        body.angular_velocity.y += (I.e10 * tqx + I.e11 * tqy + I.e12 * tqz) * body.angular_factor.y;
        body.angular_velocity.z += (I.e20 * tqx + I.e21 * tqy + I.e22 * tqz) * body.angular_factor.z;
    }

    // Two unit vectors spanning the plane perpendicular to `normal` - the friction directions.
    static _tangentBasis(normal, outT1, outT2) {
        outT1.findOrthogonal(normal);
        Vector3.crossInto(outT2, normal, outT1);
    }
}

// Approach speeds slower than this (m/s) do not bounce - below it, restitution would turn a resting
// body's one-substep gravity nudge into perpetual micro-jitter. Standard restitution slop.
Solver.RESTITUTION_THRESHOLD = 0.5;

ActionPhysics.Solver = Solver;


// ==== src/world/World.js ====
/**
 * World: the pipeline glue. Owns the body list and drives one tick through every stage in order -
 * broadphase, midphase, narrowphase, solver - exactly the pipeline table in plan.md, nothing more.
 *
 * Public surface matches plan.md's API surface table: addRigidBody, removeRigidBody, step(dt),
 * gravity. addConstraint/removeConstraint/rayIntersect/shapeIntersect are later-stage features
 * (joints, queries) and are not implemented yet - present as documented no-ops would be worse than
 * absent, so they are simply not here until built for real.
 */
class World {
    constructor(broadphase, narrowphase, solver) {
        this.broadphase = broadphase;
        this.narrowphase = narrowphase;
        this.solver = solver;
        this.midphase = new Midphase();
        this.gravity = new Vector3(0, -9.81, 0);
        this.bodies = []; // all bodies, static and dynamic alike - broadphase filters by type itself
    }

    addRigidBody(body) {
        body.world = this;
        body.updateDerived();
        this.bodies.push(body);
        this.broadphase.add(body);
        return this;
    }

    removeRigidBody(body) {
        const i = this.bodies.indexOf(body);
        if (i !== -1) this.bodies.splice(i, 1);
        this.broadphase.remove(body);
        body.world = null;
        return this;
    }

    // Advances the whole world by `dt`: broadphase -> midphase/narrowphase (fused inside
    // NarrowPhase.step, which owns calling into Midphase per plan.md's phases split) -> solver.
    // Every dynamic body's derived state (world AABB, world inverse inertia) is refreshed BEFORE
    // broadphase runs, so broadphase's own "AABBs are current" assumption (Rule 1) holds for
    // this tick's bodies, including ones the solver moved last tick.
    step(dt) {
        for (let i = 0; i < this.bodies.length; i++) this.bodies[i].updateDerived(dt);

        const pairs = this.broadphase.computePairs();
        const manifolds = this.narrowphase.step(pairs, this.midphase, dt);

        // Interleaved detect-then-solve: the solver re-measures contact geometry against each
        // substep's predicted positions via this callback (see Solver.step and
        // NarrowPhase.refreshManifoldGeometry), which is what keeps rotating/corner contacts stable.
        const narrowphase = this.narrowphase;
        this.solver.step(this.bodies, manifolds, this.gravity, dt, function (mans) {
            narrowphase.refreshManifoldGeometry(mans);
        });

        // The solver moved bodies; their derived state (AABB, world inertia) is stale until the
        // NEXT tick's pass above runs. Nothing within this tick reads it again after this point,
        // so refreshing here would be wasted work - narrowphase/broadphase for THIS tick already
        // ran against the pre-solve state, which is correct (Rule 1: each stage assumes the state
        // handed to it, not a moving target updated out from under it mid-tick).
    }
}

ActionPhysics.World = World;


// ==== src/outro.js ====
    // Every math class came from the host, so exactly one set is live in the page.
    ActionPhysics.usingHostMath = !!(host.Vector3 && host.Quaternion && host.Matrix4 && host.Scalar);

    return ActionPhysics;
}));
