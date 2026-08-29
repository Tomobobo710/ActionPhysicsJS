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

## Pyramid stability: iterations is a shared global dial

Turning `collision/pyramid` green needed the position-solve `iterations` default raised from
1. A single Gauss-Seidel pass can't converge a large coupled stack of contact manifolds, so
the (deliberately sheared) pyramid crept. This is a real convergence limitation, not an
artifact.

Tradeoff to be aware of: **`iterations` is one global knob that the pyramid and the
`fps/knockback` tests pull in opposite directions.** Stacks want a stiff, well-converged
contact solve; a fresh crate-into-player impact wants a soft single pass that transmits its
full impulse. Measured:

| iterations | pyramid drift | pyramid green | knockback @1 | knockback @2 |
|-----------:|--------------:|:-------------:|:------------:|:------------:|
| 1 (old)    | 1.38          | no            | pass         | pass         |
| 2          | 0.11          | yes           | 0.49 (needs 0.50) | pass    |
| 4          | 0.05          | yes           | fail (0.46)  | fail         |

The clean long-term fix is **velocity-gated iterations**: iterate resting contacts hard,
leave fresh/fast impacts on a single pass, so both pass at once. Not yet done — current
default is a compromise. Check `src/solver/Solver.js` for the value actually in the tree.
