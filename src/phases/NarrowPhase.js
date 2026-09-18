class NarrowPhase {
    constructor() {
        this.manifolds = new ContactManifoldList();

        this.contact_manifolds = this.manifolds;
        this._dt = 1 / 60;
        this._gjk = new GJK();
        this._epa = new EPA();

        this._support = new MinkowskiSupport({ shape: null, position: new Vector3(), rotation: new Quaternion() }, { shape: null, position: new Vector3(), rotation: new Quaternion() });
        this._contactPool = [];
        this._poolIndex = 0;
        this._pairResultScratch = [];
    }
}

NarrowPhase.SPECULATIVE_BASE = 0.02;

ActionPhysics.NarrowPhase = NarrowPhase;
