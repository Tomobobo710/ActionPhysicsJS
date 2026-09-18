class Midphase {
    constructor() {

        this._leafCache = new Map();

        this._triSlots = [];
        this._triSlotIndex = 0;
        this._childSlots = [];
        this._childSlotIndex = 0;
        this._primSlots = [];
        this._primSlotIndex = 0;

        this._nestedBodies = [];

        this._sides = { a: [], b: [] };
    }

    invalidate() {
        this._leafCache.clear();
    }
}

Midphase.SMALL_MESH_TRIS = 4;

ActionPhysics.Midphase = Midphase;
