# Engineering notes

## Rest-velocity reconciliation is the on-ramp to a real sleep system

`Solver._reconcileRestVelocity` (in `src/solver/Solver.js`) is NOT sleep, but it is the
detection half of one. If you ever build actual sleeping, reuse it — don't reinvent it.

### What it does today
After every tick's substeps finish, it looks at each body's *net position/rotation change
over the last 8 ticks* (a per-body ring buffer). If that sustained motion is below a rest
threshold (`REST_LINEAR_SPEED` / `REST_ANGULAR_SPEED`), it zeroes the body's reported
velocity. The body is still fully simulated every tick — this only corrects the reported
velocity (and, because the next tick's integrator reads that zero back, bleeds a little
residual energy out of settled bodies).

### Why it exists
XPBD derives velocity from `(x - x_prev)/h` on the *last substep only*. A settled but
load-bearing contact (e.g. the bottom row of a tall stack) never reaches a fixed point —
it bobs in a tiny substep limit cycle. Net position barely moves, but the last substep's
snapshot freezes a large steady velocity (~0.38 units/s seen on the pyramid's bottom row)
that never decays. That phantom is why "the whole pile has come to rest" failed. Measuring
sustained motion over a window instead of one tick's snapshot cancels the bob.

### Why real sleep does NOT just drop in and replace it
A textbook sleep system's trigger is "has `body.linear_velocity` stayed below threshold for
N ticks?" — it reads the *reported* velocity. But that reading is exactly what's wrong here:
a settled body reports a large phantom velocity, so a naive sleep system would conclude
"still moving" and **never sleep the pile** — the same failure, one layer up.

So sleep must be built on the *same* windowed settled-detector this pass already implements.

### The actual refactor, if/when you build sleep
- KEEP: the windowed net-motion detector (the ring buffer + threshold check). ~35 lines.
- SWAP: the two `velocity.set(0,0,0)` calls become "flag body asleep, skip its
  integrate/solve, drop it from broadphase churn."
- ADD (the real work, and the reason this wasn't done now): a wake-up path — contact with a
  moving body, a neighbour waking, an explosion/impulse, a raycast/shapecast/query hitting a
  sleeper. Getting wake-up wrong means bodies frozen mid-air or refusing to react. That's a
  real feature with real edge cases, not a bug fix, which is why the pyramid fix used
  reconciliation (the right-sized tool) instead.

Payoff of going all the way to sleep: the CPU win. Today a settled 385-box pyramid is still
fully simulated every tick; a real sleep system makes it nearly free.

### Tuning the constants (`REST_WINDOW`, `REST_LINEAR_SPEED`, `REST_ANGULAR_SPEED`)
- `REST_WINDOW = 8` ticks (~0.13 s at 60 Hz): long enough to average out the few-tick limit
  cycle, short enough that a body which starts genuinely moving is reflected within a couple
  of frames.
- The speed gates keep the zeroing to the near-rest regime — a body whose *windowed* speed
  exceeds them is really moving and keeps its raw substep-derived velocity untouched. They
  sit ~an order of magnitude under the pyramid test's own "at rest" bars (0.001 units/s,
  0.0025 rad/s vs gates 0.02 / 0.05), so a truly settled pile reads flat while any real slow
  slide or roll still registers.
- Caveat: because the gate is windowed, a body in genuine sustained slow motion *below* the
  gate (a very slow creep down a shallow ramp) gets snapped to zero. Below anything you'd
  notice in gameplay, but it is a real behavioral edit, not just a reporting fix.
- These are currently hardcoded globals. If a scene needs different behavior, they're the
  natural thing to make configurable per-World (same place a real sleep system would expose
  its own thresholds).

---

## Pyramid stability: position-solve iterations

Turning `collision/pyramid` green needed the position-solve `iterations` default raised from
1 to 4. A single Gauss-Seidel pass can't converge a large coupled stack of contact manifolds,
so the (deliberately sheared) pyramid crept. This is a real convergence limitation, not an
artifact. Cost is ~4x the position-solve work; the resting stacks are what need it.


## K1 knockback: the mass ratio was double-counted (fixed)

Raising `iterations` to 4 made the crate-vs-character contact stiffer, which briefly LOOKED
like it broke knockback (K1 went from passing to failing). Chasing that led to the real,
pre-existing bug — the iterations change only nudged an already-wrong number across the line.

`_readGhostKnockback` (src/character/fps/Ghost.js) applied the character's knockback as
`massRatio * closing`, where `closing` is the pushing object's velocity **read after the
solver already resolved its collision with the ghost**. The collision itself already applied
momentum conservation (the crate decelerated because it dumped momentum into the ghost).
Multiplying that post-collision velocity by `massRatio = mB/(mB+mP)` again charged the mass
penalty a SECOND time.

Proof it's a double-count, not physics: a FREE box of the character's mass, hit by the same
crate with the ghost's exact material (friction 0, restitution 0, linDamp 0, angDamp 0.9),
settles at peak ~4.0 — that 4.0 already IS the mass-ratio-correct answer from the real solve.
The character's formula produced ~0.46 for the same hit: `0.23 * 2.0`, i.e. mass-penalized
twice. (The free-box vs character gap was ~9x, entirely in the controller, not the engine or
the material — the material was tested and exonerated.)

Fix: knock back by `closing` directly (drop the `massRatio` multiply). All of K1..K13 pass at
all three scales, including K7 "light ball does not knock" and K8 "no self-knock" — so the
other gates (closing-speed floor, max-speed cap, self-push exclusion) still hold the line; the
mass ratio was purely redundant, not load-bearing. This retired the pyramid-vs-knockback
"opposite directions" tension entirely: iterations:4 and full K1 knockback now coexist, no
velocity-gating needed.

## Box-box flat-drop lateral drift: resolve the face patch at its centroid (fixed)

A perfectly symmetric flat box dropped onto a much larger box (`box-box/single`) picked up
~36 mm of lateral drift the instant it landed — a long-standing open item (see snapshots 52–54).
The cause is a sequential-Gauss-Seidel symmetry break in the VELOCITY pass, not the position
pass. A flat landing produces a 4-point coplanar face manifold with one shared normal. Solving
restitution + friction point-by-point spins the body a hair (each off-centre normal impulse from
the box's default restitution 0.33 applies torque; the next point reads the spun state, so the
four impulses stop cancelling), and friction then converts that residual spin into a net sideways
velocity. Restitution is the dominant contributor: forcing box restitution to 0 drops the drift
from 0.036 to 0.011, and the 0.011 remainder is the same effect seeded by the position solve's own
tiny asymmetry instead.

Fix (`src/solver/VelocitySolve.js` `_boxFacePatchVelocity`, called from
`Solver._solveContactVelocities`): when — and ONLY when — both shapes are actual `BoxShape`s and
every engaged manifold point shares a single normal (a genuine face patch, `COPLANAR_NORMAL_DOT`),
resolve restitution and friction ONCE at the patch centroid instead of per-point. A centroid solve
of a symmetric patch is exactly symmetric, so it leaves zero residual spin or drift. Every other
contact (edge/corner box hits, meshes, spheres, compounds, non-coplanar manifolds) keeps the
per-point solve untouched.

WHY NOT the "Jacobi-style manifold solve" snapshots 52–54 pointed at: it was tried here and every
variant regressed. Full Jacobi on the velocity pass (all points measured off one frozen velocity,
applied together) zeroes the drift but explodes the 385-box pyramid and breaks the mesh-box stacks
— coupled stacks need the sequential information flow. Jacobi on the position pass alone regresses
11 tests and doesn't even fix the drift (the velocity pass reintroduces it). Aggregating restitution
alone at the centroid breaks the exact restitution-ratio tests (`e=0.5/0.8/1` want per-point exit
speed to 1e-6). Aggregating BOTH restitution and friction, but for ALL multi-point manifolds, fixes
the drift but adds tail jitter to two "box stacked on box" tests — which are box-shaped MESH stacks,
not `BoxBox`. Gating the whole thing on real `BoxShape` pairs is what threads the needle: the mesh
stacks keep their stable per-point path, the flat box-drop gets the symmetric centroid path. Full
headless suite 769/769 after this.

The sibling `box-box/offset-stack` failure was a separate, unrelated test-data bug: it spawned unit
(2-tall) boxes with a 2.2 vertical GAP, leaving a 0.2 gap per layer, then measured settling drift
against the SPAWN height (< 0.05) — impossible, the boxes fall 0.2/0.4/0.6 onto each other. Changed
GAP to 2.0 (= box height) so the stack spawns already resting, which is the "inset horizontally"
shape the test describes.
