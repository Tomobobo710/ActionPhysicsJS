// Dispatches Midphase's primitive-shape pairs through the closed-form tests / GJK/EPA into
// ContactManifoldList. See PairTest.js, SpeculativeMargin.js, GeometryRefresh.js.
class NarrowPhase {
    constructor() {
        this.manifolds = new ContactManifoldList();
        // Same object under the Goblin-style name; walk it as .contact_manifolds.first ->
        // .next_manifold. The `contacts` World event delivers this same list.
        this.contact_manifolds = this.manifolds;
        this._dt = 1 / 60; // set each tick by step()
        this._gjk = new GJK();
        this._epa = new EPA();
        // Rebound per pair via setSides() (PairTest.js).
        this._support = new MinkowskiSupport({ shape: null, position: new Vector3(), rotation: new Quaternion() }, { shape: null, position: new Vector3(), rotation: new Quaternion() });
        this._contactPool = []; // reused ContactDetails, grown as needed
        this._poolIndex = 0;
        this._pairResultScratch = [];
    }
}

NarrowPhase.SPECULATIVE_BASE = 0.02; // meters, see SpeculativeMargin.js

ActionPhysics.NarrowPhase = NarrowPhase;
