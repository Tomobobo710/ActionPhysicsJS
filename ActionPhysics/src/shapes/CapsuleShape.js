// Axis is local Y. Constructor takes TOTAL height, unlike every other shape here — noted
// explicitly because it is the one deliberate exception to the half-extent rule: a capsule's
// height already includes its hemispherical caps, so there
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
        // A capsule's support point is radius*normalize(direction) offset by whichever cap CENTER
        // (top or bottom) is farther along direction - the standard "sphere-swept segment" support
        // function. At direction.y exactly 0 that choice is genuinely ambiguous (both cap centers
        // dot to the same value along a purely horizontal direction, and the true farthest point is
        // the barrel equator itself, not either cap center) - picking a cap center there anyway
        // would put every purely-horizontal support on one ring instead of the equator, degenerate
        // for a GJK/EPA simplex that samples several such directions. Zero-Y is handled first and
        // explicitly rather than folded into the >= 0 branch below.
        const lsq = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        // A zero-length direction has no "farthest point" - any answer is arbitrary. Return the
        // capsule's own center: finite, harmless, and never mistaken for a real support (the
        // fallback the historical 0/0 NaN bug needed, not a physically meaningful point).
        if (lsq === 0) { out.x = 0; out.y = 0; out.z = 0; return out; }
        // Exact equality would miss a direction that is horizontal in intent but carries floating-
        // point residue from an upstream rotation (e.g. a world-space horizontal direction inverse-
        // rotated into a tilted capsule's local space lands at y ~ 1e-16, not exactly 0) - an epsilon
        // catches that case too, and is tight enough to never misfire on a direction that is
        // meaningfully off-axis (where the ordinary cap-center branch below is the right answer).
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

    // Composite of a cylindrical core plus two hemispherical caps, each contributing its own
    // parallel-axis term. Standard closed forms.
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
