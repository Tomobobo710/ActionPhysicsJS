class Transform {
    constructor() {
        this.position = new Vector3(0, 0, 0);
        this.rotation = new Quaternion(0, 0, 0, 1);
        this.scale = new Vector3(1, 1, 1);
    }

    syncFromPhysicsBody(body) {
        if (!body) return;

        this.position.x = body.position.x;
        this.position.y = body.position.y;
        this.position.z = body.position.z;

        this.rotation = body.rotation;
    }

    clone() {
        const clone = new Transform();
        clone.position = this.position.clone();
        clone.rotation = new Quaternion(this.rotation.x, this.rotation.y, this.rotation.z, this.rotation.w);
        clone.scale = this.scale.clone();
        return clone;
    }

    transformPoint(point) {

        const sx = point.x * this.scale.x;
        const sy = point.y * this.scale.y;
        const sz = point.z * this.scale.z;

        const qx = this.rotation.x,
            qy = this.rotation.y,
            qz = this.rotation.z,
            qw = this.rotation.w;

        const ix = qw * sx + qy * sz - qz * sy;
        const iy = qw * sy + qz * sx - qx * sz;
        const iz = qw * sz + qx * sy - qy * sx;
        const iw = -qx * sx - qy * sy - qz * sz;

        const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

        return new Vector3(rx + this.position.x, ry + this.position.y, rz + this.position.z);
    }

    transformVector(vector) {

        const sx = vector.x * this.scale.x;
        const sy = vector.y * this.scale.y;
        const sz = vector.z * this.scale.z;

        const qx = this.rotation.x,
            qy = this.rotation.y,
            qz = this.rotation.z,
            qw = this.rotation.w;

        const ix = qw * sx + qy * sz - qz * sy;
        const iy = qw * sy + qz * sx - qx * sz;
        const iz = qw * sz + qx * sy - qy * sx;
        const iw = -qx * sx - qy * sy - qz * sz;

        const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

        return new Vector3(rx, ry, rz);
    }

    transformPointInto(point, dest) {

        const sx = point.x * this.scale.x;
        const sy = point.y * this.scale.y;
        const sz = point.z * this.scale.z;

        const qx = this.rotation.x,
            qy = this.rotation.y,
            qz = this.rotation.z,
            qw = this.rotation.w;

        const ix = qw * sx + qy * sz - qz * sy;
        const iy = qw * sy + qz * sx - qx * sz;
        const iz = qw * sz + qx * sy - qy * sx;
        const iw = -qx * sx - qy * sy - qz * sz;

        const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

        dest.x = rx + this.position.x;
        dest.y = ry + this.position.y;
        dest.z = rz + this.position.z;
        return dest;
    }

    transformVectorInto(vector, dest) {

        const sx = vector.x * this.scale.x;
        const sy = vector.y * this.scale.y;
        const sz = vector.z * this.scale.z;

        const qx = this.rotation.x,
            qy = this.rotation.y,
            qz = this.rotation.z,
            qw = this.rotation.w;

        const ix = qw * sx + qy * sz - qz * sy;
        const iy = qw * sy + qz * sx - qx * sz;
        const iz = qw * sz + qx * sy - qy * sx;
        const iw = -qx * sx - qy * sy - qz * sz;

        const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;

        dest.x = rx;
        dest.y = ry;
        dest.z = rz;
        return dest;
    }
}

ActionPhysics.Transform = Transform;
