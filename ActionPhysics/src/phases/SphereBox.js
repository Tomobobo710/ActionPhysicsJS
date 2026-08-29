// Closed-form sphere-box: closest point on an oriented box to the sphere center, clamped per-axis
// in the box's local frame. Own epsilon, no shared GJK/EPA state.
const SphereBox = {};

SphereBox.applies = function (placedA, placedB) {
    return (placedA.shape instanceof SphereShape && placedB.shape instanceof BoxShape) ||
        (placedA.shape instanceof BoxShape && placedB.shape instanceof SphereShape);
};

// Below this distance from the box surface to the sphere center, the surface normal is undefined
// (center exactly on/past every face at once - only reachable at the box's own center) - falls
// back to a fixed axis rather than dividing by ~0.
SphereBox.DEGENERATE_EPSILON = 1e-9;

SphereBox.test = function (placedA, placedB, out) {
    const sphereFirst = placedA.shape instanceof SphereShape;
    const spherePlaced = sphereFirst ? placedA : placedB;
    const boxPlaced = sphereFirst ? placedB : placedA;
    const sphere = spherePlaced.shape, box = boxPlaced.shape;

    // Sphere center in the box's local frame.
    const invRot = SphereBox._scratchQuat.copy(boxPlaced.rotation).invert();
    const local = SphereBox._scratchV1;
    local.copy(spherePlaced.position).subInPlace(boxPlaced.position);
    invRot.transformVectorInPlace(local);

    // Closest point on the box to that center, clamped per axis; also track whether the center is
    // strictly inside (all three axes already within their half-extent - deep penetration).
    const hw = box.halfWidth, hh = box.halfHeight, hd = box.halfDepth;
    const insideX = local.x > -hw && local.x < hw;
    const insideY = local.y > -hh && local.y < hh;
    const insideZ = local.z > -hd && local.z < hd;
    const inside = insideX && insideY && insideZ;

    const closest = SphereBox._scratchV2;
    let localNx = 0, localNy = 0, localNz = 0, penetration = 0;
    if (inside) {
        // Center is inside the box: push out along whichever axis has the LEAST penetration
        // (the standard box-interior-point resolution - the shortest way out).
        const px = hw - Math.abs(local.x), py = hh - Math.abs(local.y), pz = hd - Math.abs(local.z);
        if (px <= py && px <= pz) { localNx = local.x >= 0 ? 1 : -1; penetration = px; closest.set(local.x >= 0 ? hw : -hw, local.y, local.z); }
        else if (py <= pz) { localNy = local.y >= 0 ? 1 : -1; penetration = py; closest.set(local.x, local.y >= 0 ? hh : -hh, local.z); }
        else { localNz = local.z >= 0 ? 1 : -1; penetration = pz; closest.set(local.x, local.y, local.z >= 0 ? hd : -hd); }
    } else {
        closest.set(
            Math.max(-hw, Math.min(hw, local.x)),
            Math.max(-hh, Math.min(hh, local.y)),
            Math.max(-hd, Math.min(hd, local.z))
        );
    }

    const dx = local.x - closest.x, dy = local.y - closest.y, dz = local.z - closest.z;
    const distSq = dx * dx + dy * dy + dz * dz;
    const dist = Math.sqrt(distSq);

    let worldNx, worldNy, worldNz;
    if (inside) {
        // Normal already chosen above (box-local axis of least penetration).
        SphereBox._scratchV1.set(localNx, localNy, localNz);
        boxPlaced.rotation.transformVectorInPlace(SphereBox._scratchV1);
    } else if (dist > SphereBox.DEGENERATE_EPSILON) {
        SphereBox._scratchV1.set(dx / dist, dy / dist, dz / dist);
        boxPlaced.rotation.transformVectorInPlace(SphereBox._scratchV1);
    } else {
        SphereBox._scratchV1.set(0, 1, 0);
        boxPlaced.rotation.transformVectorInPlace(SphereBox._scratchV1);
    }
    worldNx = SphereBox._scratchV1.x; worldNy = SphereBox._scratchV1.y; worldNz = SphereBox._scratchV1.z;
    // Normal points sphere-side -> box-side in this local derivation; flip to A->B then to B->A
    // (the pipeline convention) based on which placed side is actually the sphere.
    if (!sphereFirst) { worldNx = -worldNx; worldNy = -worldNy; worldNz = -worldNz; }

    const worldClosest = SphereBox._scratchV3;
    worldClosest.copy(closest);
    boxPlaced.rotation.transformVectorInPlace(worldClosest);
    worldClosest.addInPlace(boxPlaced.position);

    const signedDistance = inside ? (sphere.radius + penetration) : (sphere.radius - dist);

    const pointOnSphere = SphereBox._scratchV4;
    // Point on the sphere's own surface, along the normal from the box back toward the sphere.
    const towardSphereX = sphereFirst ? worldNx : -worldNx, towardSphereY = sphereFirst ? worldNy : -worldNy, towardSphereZ = sphereFirst ? worldNz : -worldNz;
    pointOnSphere.set(
        spherePlaced.position.x - towardSphereX * sphere.radius,
        spherePlaced.position.y - towardSphereY * sphere.radius,
        spherePlaced.position.z - towardSphereZ * sphere.radius
    );

    if (sphereFirst) {
        out.pointOnA.copy(pointOnSphere);
        out.pointOnB.copy(worldClosest);
    } else {
        out.pointOnA.copy(worldClosest);
        out.pointOnB.copy(pointOnSphere);
    }
    out.normal.set(worldNx, worldNy, worldNz);
    out.signedDistance = signedDistance;
    Vector3.addInto(out.point, out.pointOnA, out.pointOnB).scaleInPlace(0.5);
    return out;
};

SphereBox._scratchQuat = new Quaternion();
SphereBox._scratchV1 = new Vector3();
SphereBox._scratchV2 = new Vector3();
SphereBox._scratchV3 = new Vector3();
SphereBox._scratchV4 = new Vector3();
