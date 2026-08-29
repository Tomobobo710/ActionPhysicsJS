// Axis is local Y. Constructor takes TOTAL height (includes hemispherical caps), unlike every
// other shape's half-extent convention.
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

    // Sphere-swept-segment support: radius*normalize(dir) offset by the farther cap center. At
    // dir.y ~0 the true farthest point is the barrel equator, not a cap center - handled explicitly.
    supportInto(out, direction) {
        const lsq = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (lsq === 0) { out.x = 0; out.y = 0; out.z = 0; return out; }
        if (Math.abs(direction.y) < 1e-9) {
            const s = this.radius / Math.sqrt(lsq);
            out.x = direction.x * s;
            out.y = 0;
            out.z = direction.z * s;
            return out;
        }
        const centerY = direction.y > 0 ? this.segmentHalfLength : -this.segmentHalfLength;
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

    // Cylinder core + two hemispherical caps, each with its own parallel-axis term.
    computeMassData() {
        const r = this.radius, hs = this.segmentHalfLength;
        const cylinderVolume = Scalar.PI * r * r * (2 * hs);
        const hemisphereVolume = (2 / 3) * Scalar.PI * r * r * r;
        const mass = cylinderVolume + 2 * hemisphereVolume;

        const cylinderMass = cylinderVolume;
        const hemisphereMass = hemisphereVolume;

        const iAxisCyl = 0.5 * cylinderMass * r * r;
        const iSideCyl = cylinderMass * (3 * r * r + (2 * hs) * (2 * hs)) / 12;

        const iAxisHemi = 0.4 * hemisphereMass * r * r;
        const hemiCentroidOffset = (3 / 8) * r;
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
