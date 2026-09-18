class Vector3 {

    static _pool = [];
    static _poolSize = 0;
    static _maxPoolSize = 1000;

    static getFromPool(x = 0, y = 0, z = 0) {
        if (Vector3._poolSize > 0) {
            const vec = Vector3._pool[--Vector3._poolSize];
            vec.set(x, y, z);
            return vec;
        }
        return new Vector3(x, y, z);
    }

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

            this.x = x.x;
            this.y = x.y;
            this.z = x.z;
        } else {

            this.x = x;
            this.y = y;
            this.z = z;
        }
        return this;
    }

    static distance(a, b) {
        return Scalar.hypot3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    distanceTo(other) {
        const dx = this.x - other.x;
        const dy = this.y - other.y;
        const dz = this.z - other.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    distanceSquared(other) {
        const dx = this.x - other.x;
        const dy = this.y - other.y;
        const dz = this.z - other.z;
        return dx * dx + dy * dy + dz * dz;
    }

    horizontalDistanceTo(other) {
        const dx = this.x - other.x;
        const dz = this.z - other.z;
        return Math.sqrt(dx * dx + dz * dz);
    }

    horizontalDistanceSquared(other) {
        const dx = this.x - other.x;
        const dz = this.z - other.z;
        return dx * dx + dz * dz;
    }

    translate(direction, amount) {
        return new Vector3(this.x + direction.x * amount, this.y + direction.y * amount, this.z + direction.z * amount);
    }

    translateInPlace(direction, amount) {
        this.x += direction.x * amount;
        this.y += direction.y * amount;
        this.z += direction.z * amount;
        return this;
    }

    rotateY(angle) {
        const cos = Scalar.cos(angle);
        const sin = Scalar.sin(angle);
        return new Vector3(this.x * cos + this.z * sin, this.y, -this.x * sin + this.z * cos);
    }

    rotateYInPlace(angle) {
        const cos = Scalar.cos(angle);
        const sin = Scalar.sin(angle);
        const x = this.x;
        const z = this.z;
        this.x = x * cos + z * sin;
        this.z = -x * sin + z * cos;
        return this;
    }

    horizontalNormalize() {
        return new Vector3(this.x, 0, this.z).normalize();
    }

    static transformMat4(vec, mat) {

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

    add(other) {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    addInPlace(other) {
        this.x += other.x;
        this.y += other.y;
        this.z += other.z;
        return this;
    }

    sub(other) {
        return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    static subtract(a, b) {
        return new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    subInPlace(other) {
        this.x -= other.x;
        this.y -= other.y;
        this.z -= other.z;
        return this;
    }

    normalize() {
        const len = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
        if (len === 0) {
            return new Vector3(0, 0, 0);
        }
        return new Vector3(this.x / len, this.y / len, this.z / len);
    }

    normalizeInPlace() {
        const len = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
        if (len !== 0) {
            this.x /= len;
            this.y /= len;
            this.z /= len;
        }
        return this;
    }

    dot(other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    cross(other) {
        return new Vector3(
            this.y * other.z - this.z * other.y,
            this.z * other.x - this.x * other.z,
            this.x * other.y - this.y * other.x
        );
    }

    static crossInto(out, a, b) {
        const ax = a.x, ay = a.y, az = a.z;
        const bx = b.x, by = b.y, bz = b.z;
        out.x = ay * bz - az * by;
        out.y = az * bx - ax * bz;
        out.z = ax * by - ay * bx;
        return out;
    }

    toArray() {
        return [this.x, this.y, this.z];
    }

    length() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    lengthSquared() {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    }

    mult(n) {
        return new Vector3(this.x * n, this.y * n, this.z * n);
    }

    scale(scalar) {
        return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
    }

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
        const epsilon = 0.000001;
        return (
            Math.abs(this.x - other.x) < epsilon &&
            Math.abs(this.y - other.y) < epsilon &&
            Math.abs(this.z - other.z) < epsilon
        );
    }

    clone() {
        return new Vector3(this.x, this.y, this.z);
    }

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

    static addInto(out, a, b) {
        out.x = a.x + b.x; out.y = a.y + b.y; out.z = a.z + b.z;
        return out;
    }

    static subInto(out, a, b) {
        out.x = a.x - b.x; out.y = a.y - b.y; out.z = a.z - b.z;
        return out;
    }

    static scaleInto(out, v, s) {
        out.x = v.x * s; out.y = v.y * s; out.z = v.z * s;
        return out;
    }

    static normalizeInto(out, v) {
        const lsq = v.x * v.x + v.y * v.y + v.z * v.z;
        if (lsq === 0) { out.x = 0; out.y = 0; out.z = 0; return out; }
        const inv = 1 / Math.sqrt(lsq);
        out.x = v.x * inv; out.y = v.y * inv; out.z = v.z * inv;
        return out;
    }

    addScaledInPlace(v, s) {
        this.x += v.x * s; this.y += v.y * s; this.z += v.z * s;
        return this;
    }

    scaleInPlace(s) {
        this.x *= s; this.y *= s; this.z *= s;
        return this;
    }

    multiplyInPlace(v) {
        this.x *= v.x; this.y *= v.y; this.z *= v.z;
        return this;
    }

    negateInPlace() {
        this.x = -this.x; this.y = -this.y; this.z = -this.z;
        return this;
    }

    crossInPlace(v) {
        const x = this.x, y = this.y, z = this.z;
        const vx = v.x, vy = v.y, vz = v.z;
        this.x = y * vz - z * vy;
        this.y = z * vx - x * vz;
        this.z = x * vy - y * vx;
        return this;
    }

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

    equalsExact(v) {
        return this.x === v.x && this.y === v.y && this.z === v.z;
    }

    toString() {
        return '(' + this.x + ', ' + this.y + ', ' + this.z + ')';
    }
}

ActionPhysics.Vector3 = Vector3;
