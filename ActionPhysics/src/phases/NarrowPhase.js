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
        this._contactPool = []; // reused ContactDetails, grown as needed, never shrunk
        this._poolIndex = 0;
    }
}

// Base speculative margin (metres) - see SpeculativeMargin.js.
NarrowPhase.SPECULATIVE_BASE = 0.02;

ActionPhysics.NarrowPhase = NarrowPhase;
