/**
 * NarrowPhase: dispatch layer tying Midphase's primitive-shape pairs through GJK/EPA into
 * ContactDetails, routed into ContactManifoldList. See PairTest.js (per-pair GJK/EPA + tick
 * dispatch), SpeculativeMargin.js (how far ahead a contact is reported), GeometryRefresh.js
 * (per-substep contact geometry re-measure).
 */
class NarrowPhase {
    constructor() {
        this.manifolds = new ContactManifoldList();
        this._dt = 1 / 60; // set each tick by step()
        // Scratch GJK/EPA, reused across every pair tested this tick - safe since pairs run
        // one at a time, never interleaved.
        this._gjk = new GJK();
        this._epa = new EPA();
        // Reused across every GJK/EPA-fallback pair this tick, rebound per pair via setSides() -
        // see PairTest.js.
        this._support = new MinkowskiSupport({ shape: null, position: new Vector3(), rotation: new Quaternion() }, { shape: null, position: new Vector3(), rotation: new Quaternion() });
        this._contactPool = []; // reused ContactDetails, grown as needed, never shrunk
        this._poolIndex = 0;
        this._pairResultScratch = []; // reused per-pair contact list, see PairTest.js
    }
}

// Base speculative margin (meters) - see SpeculativeMargin.js.
NarrowPhase.SPECULATIVE_BASE = 0.02;

ActionPhysics.NarrowPhase = NarrowPhase;
