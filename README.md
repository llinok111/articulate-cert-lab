# articulate-cert-lab

**Continuous-time self-collision certificate for articulated rigid-body chains: 6 chains / 12 capsule links / 330 link pairs are decided over the *whole* time interval — not per frame — in 95.0 ms (median of 7) with 30,999 gap evaluations, versus 931.9 ms for a uniform 20,000-frame oracle (9.81× faster) that is only correct up to its step. A uniform 120-frame scan instead finishes in 5.5 ms but misses 5 of the 5 collisions.**

Upstream: none

`src/core.mjs` is a pure ES module (no `three.js`, no DOM, no filesystem, no environment variables). The browser page feeds it the same JSON the benchmark uses and displays its verdict live; noding the module is the same code path.

## How to run

```bash
node tools/gen-sample.mjs     # 1. rebuild data/chains.json (deterministic)
node bench/verify.mjs         # 2. behavioural assertions -> exit 0 on success
node bench/bench.mjs          # 3. writes bench/result.json
python3 -m http.server 4173   # 4. open http://localhost:4173
```

## What it actually proves

Each link is a capsule. For a pair `(i,j)` the certificate works on the gap

```
gap_ij(t) = dist(segA_i(t), segB_i(t), segA_j(t), segB_j(t)) − r_i − r_j
```

The distance between two segments is 1-Lipschitz in each of its four endpoints, so on any interval `[t0,t1]` lying inside one keyframe span the joint speeds give a computable bound `L` with `|gap'| ≤ L`. Evaluating the gap at `t0`, the midpoint and `t1` therefore yields the **sound** lower bound `gap(t) ≥ min(samples) − L·(t1−t0)/2` for every `t` in the interval. If it is positive the whole interval is certified collision-free; otherwise the interval is bisected. A collision is only ever reported together with an explicit `t*` where the *evaluated* gap is `≤ 0`, so a false positive is structurally impossible. No sampling is used for the verdict.

| quantity | value | where |
| --- | --- | --- |
| chains × links × certified pairs | 6 × 12 × 330 (55/chain, adjacent pairs excluded) | `bench/verify.mjs` |
| colliding pairs found | 5 (3 chains clean, 3 chains colliding) | `bench/verify.mjs` |
| unresolved intervals | 0 | `bench/verify.mjs` |
| collision interval widths | 1.25e-4 … 1.78e-3 (timeline t ∈ [0,1]) | `data/chains.json` |
| certificate time (median of 7) | **95.0 ms** (min 91.0, p95 105.0, σ 5.08, relSD 5.30%, cold 177.6 ms) | `bench/result.json` |
| gap evaluations used | 30,999 | `bench/result.json` |
| uniform 20,000-frame oracle | 6,600,000 pair evaluations, 931.9 ms → certificate **9.81× faster** | `bench/result.json` |
| uniform 120-frame scan | 5.47 ms but **misses 5/5** collisions; 1,200 frames still misses 3/5 | `bench/verify.mjs` |
| certificate vs oracle agreement | 0 missed, 0 false (5 = 5) | `bench/verify.mjs` |
| scale invariance | identical verdict set at 1e-6, 1e-3, 1, 1e3, 1e6 | `bench/verify.mjs` |
| minimal motion damping repair | s = 0.9933 / 0.9229 / 0.9112 per colliding chain; damped motion certifies 0 collisions | `data/chains.json` |

The oracle in `baseline/naive.mjs` is a **separate implementation** (quaternion kinematics and its own segment-distance routine) written so that agreement is evidence rather than a shared formula; on the same inputs the two gap functions differ by at most `3.6e-15` (asserted in `bench/verify.mjs`). It shares no constant with the certificate — its only knob is the frame count.

## Repair: minimal motion damping

For a colliding chain the trajectory is re-parameterised as `θ_s(t) = θ_rest + s·(θ(t) − θ_rest)`. `s = 0` is the rest pose. `s` is bisected (44 steps) while the *same* certificate is run, so the returned `s` is the largest value that was verified collision-free; the immediate neighbour above it contains an explicit collision witness. This is damping of the motion amplitude, **not** time remapping — slowing time down would not remove a collision, it would only delay it.

## 本轮创新

First time this account's interval-Lipschitz certification is applied to **multi-link articulated chains**: the Lipschitz constant is derived from the *chain* (per-link frame-origin speed accumulated over the joint chain plus the angular-speed bound times the link radius), which lets 330 link pairs be decided over continuous time instead of per frame. Previous work here (`toi-cert`) certified a single moving triangle pair and `skinint-cert` a single skinned mesh; the combination *articulated chain + link-pair interval certificate + witness-based repair* is new, and the headline numbers (95.0 ms vs 931.9 ms / 5-of-5 collisions missed at 120 frames) are recomputable with the three commands above.

## Known limitations

- **Threshold model.** The certificate is exact for *capsule* links. Links are capsules here; a real asset's triangle soup would first need a conservative capsule decomposition — that step is not implemented.
- **Relative time epsilon, absolute geometry.** Subdivision stops at `relTimeEps = 1e-13` of the timeline, not at a length threshold, so no geometric epsilon has to be disclosed and the verdict is unchanged from scale 1e-6 to 1e6 (asserted). A gap whose true minimum is exactly 0 (a tangency) cannot be certified clean and would be reported in `tangencies` instead; on this corpus `tangencies = 0`.
- **Lipschitz bound quality.** `L` is a first-order bound built from the maximum joint speed in the interval; a fast segment forces deep subdivision. The 3D chains here use a deliberately sharp (1e-3 wide) curl, which is why the certificate spends 30,999 evaluations. Energies that are smooth in `t` would cost less.
- **Adjacent links are excluded** (`minSep = 2`): neighbouring capsules meet at their shared joint by construction. Interpenetration between them is not modelled.
- **Rest-pose assumption.** Damping is only offered if the rest pose itself certifies clean; otherwise it returns `feasible: false`.
- **Live browser re-certification** for one chain takes roughly 15–30 ms of main-thread work in the page (per the on-screen readout); the timeline redraw is per-frame and re-evaluates only the current instant.
- The benchmark is headless (`node`); no browser automation is used, and no screenshot-based evidence is claimed.

## Layout

```
index.html            # importmap (three@0.169.0 pinned) + controls
main.js               # rendering only; consumes src/core.mjs output
src/core.mjs          # certificate, forward kinematics, repair — pure ES module
tools/gen-sample.mjs  # deterministic 6-chain generator -> data/chains.json
baseline/naive.mjs    # independent quaternion oracle + uniform frame scan
bench/verify.mjs      # 17 behavioural assertions
bench/bench.mjs       # cold start + 7 warm reps -> bench/result.json
```

MIT licensed; see `LICENSE`.
