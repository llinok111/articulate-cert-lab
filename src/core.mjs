// src/core.mjs — Pure geometry + continuous-time self-collision certificate
// for articulated rigid-body chains. No rendering, no DOM, no IO, no deps.
//
// Model
//   spec.joints[i] = {axis:[x,y,z], origin:[x,y,z]}  revolute; origin expressed in
//                    the PARENT link frame (world frame for i = 0).
//   spec.links[i]  = {a:[x,y,z], b:[x,y,z], r:number}  capsule in link frame.
//   traj           = {times:[...], angles:[ [j0..jn-1], ... ]}  piecewise-linear,
//                    angles[k] is the full joint vector at times[k].
//
// Certificate (sound, not sampling)
//   gap_ij(t) = dist(segA_i(t), segB_i(t), segA_j(t), segB_j(t)) - r_i - r_j.
//   dist(seg,seg) is 1-Lipschitz in each of its four endpoints, so for any interval
//   [t0,t1] inside one keyframe span:
//        L = sp_i + om_i*rho_i + sp_j + om_j*rho_j
//   bounds |gap'(t)|. Hence gap(t) >= gap(t0) - L*(t-t0). If that lower bound is
//   positive the whole interval is certified collision-free. Otherwise the interval
//   is bisected; the recursion terminates because once x1-x0 < gap_min/L the bound
//   is positive. A collision is only ever reported with an explicit t* where the
//   evaluated gap is <= 0, so false positives are impossible by construction.

export const EPS = 1e-15;

export const v = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
  unit: (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
};

// ---- 3x3 rotations (row-major flat arrays) --------------------------------
export function rotAxisAngle(axis, ang) {
  const [x, y, z] = v.unit(axis);
  const c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
  ];
}
export function mm(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let s = 0; for (let k = 0; k < 3; k++) s += A[i * 3 + k] * B[k * 3 + j];
    C[i * 3 + j] = s;
  }
  return C;
}
export function mv(A, p) {
  return [A[0] * p[0] + A[1] * p[1] + A[2] * p[2],
          A[3] * p[0] + A[4] * p[1] + A[5] * p[2],
          A[6] * p[0] + A[7] * p[1] + A[8] * p[2]];
}

// ---- forward kinematics ----------------------------------------------------
export function sampleAngles(traj, t) {
  const times = traj.times, K = traj.angles.length, J = traj.angles[0].length;
  if (t <= times[0]) return traj.angles[0].slice();
  if (t >= times[K - 1]) return traj.angles[K - 1].slice();
  let k = 1; while (times[k] < t) k++;
  const a = times[k - 1], b = times[k], u = (t - a) / (b - a);
  const A = traj.angles[k - 1], B = traj.angles[k], out = new Array(J);
  for (let j = 0; j < J; j++) out[j] = A[j] + (B[j] - A[j]) * u;
  return out;
}

export function linkFrames(spec, angles) {
  const n = spec.links.length, R = new Array(n), T = new Array(n);
  for (let i = 0; i < n; i++) {
    const rrel = rotAxisAngle(spec.joints[i].axis, angles[i]);
    if (i === 0) { T[0] = spec.joints[0].origin.slice(); R[0] = rrel; }
    else { T[i] = v.add(T[i - 1], mv(R[i - 1], spec.joints[i].origin)); R[i] = mm(R[i - 1], rrel); }
  }
  return { R, T };
}

export function linkEndpoints(spec, F, i) {
  const L = spec.links[i];
  return [v.add(F.T[i], mv(F.R[i], L.a)), v.add(F.T[i], mv(F.R[i], L.b))];
}

// ---- closest points between two segments (Ericson, Real-Time Collision Det.) --
export function segSegClosest(p1, q1, p2, q2) {
  const d1 = v.sub(q1, p1), d2 = v.sub(q2, p2), r = v.sub(p1, p2);
  const a = v.dot(d1, d1), e = v.dot(d2, d2), f = v.dot(d2, r);
  const EPSQ = 1e-24;
  let s, t;
  if (a <= EPSQ && e <= EPSQ) { s = 0; t = 0; }
  else if (a <= EPSQ) { s = 0; t = Math.min(1, Math.max(0, f / e)); }
  else {
    const c = v.dot(d1, r);
    if (e <= EPSQ) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
    else {
      const b = v.dot(d1, d2), den = a * e - b * b;
      s = den !== 0 ? Math.min(1, Math.max(0, (b * f - c * e) / den)) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
      else if (t > 1) { t = 1; s = Math.min(1, Math.max(0, (b - c) / a)); }
    }
  }
  const c1 = v.add(p1, v.mul(d1, s)), c2 = v.add(p2, v.mul(d2, t));
  return { s, t, c1, c2, d: v.len(v.sub(c1, c2)) };
}
export function segSegDist(p1, q1, p2, q2) { return segSegClosest(p1, q1, p2, q2).d; }

// ---- per-interval Lipschitz bound -----------------------------------------
export function angleSpeedBound(traj, t0, t1) {
  const K = traj.times.length, J = traj.angles[0].length;
  const w = new Array(J).fill(0);
  for (let k = 1; k < K; k++) {
    const a = traj.times[k - 1], b = traj.times[k];
    const lo = Math.max(a, t0), hi = Math.min(b, t1);
    if (hi <= lo) continue;
    const dt = b - a;
    for (let j = 0; j < J; j++) {
      const s = Math.abs(traj.angles[k][j] - traj.angles[k - 1][j]) / dt;
      if (s > w[j]) w[j] = s;
    }
  }
  return w;
}

export function linkSpeedBounds(spec, w) {
  const n = spec.links.length, omega = new Array(n).fill(0), spd = new Array(n).fill(0), rho = new Array(n).fill(0);
  let acc = 0;
  for (let i = 0; i < n; i++) {
    acc += w[i]; omega[i] = acc;
    spd[i] = i === 0 ? 0 : spd[i - 1] + omega[i - 1] * v.len(spec.joints[i].origin);
  }
  for (let i = 0; i < n; i++) rho[i] = Math.max(v.len(spec.links[i].a), v.len(spec.links[i].b));
  return { omega, spd, rho };
}

export function pairLipschitz(spec, traj, t0, t1, i, j) {
  const w = angleSpeedBound(traj, t0, t1);
  const b = linkSpeedBounds(spec, w);
  return (b.spd[i] + b.omega[i] * b.rho[i]) + (b.spd[j] + b.omega[j] * b.rho[j]);
}

// ---- gap evaluation --------------------------------------------------------
export function pairGap(spec, traj, t, i, j) {
  const F = linkFrames(spec, sampleAngles(traj, t));
  const [A, B] = linkEndpoints(spec, F, i);
  const [C, D] = linkEndpoints(spec, F, j);
  return segSegDist(A, B, C, D) - spec.links[i].r - spec.links[j].r;
}

export function collisionPairs(spec, minSep = 2) {
  const n = spec.links.length, out = [];
  for (let i = 0; i < n; i++) for (let j = i + minSep; j < n; j++) out.push([i, j]);
  return out;
}

// Rough but scale-covariant bound on the reach of the chain (used to make the
// subdivision epsilon relative, so verdicts are invariant under uniform scaling).
export function reachBound(spec) {
  let reach = 0, maxR = 0;
  for (let i = 0; i < spec.links.length; i++) {
    reach += v.len(spec.joints[i].origin) + v.len(spec.links[i].b);
    maxR = Math.max(maxR, spec.links[i].r);
  }
  return reach + maxR;
}
export function bboxDiag(spec) { return 2 * reachBound(spec); }

export function scaleSpec(spec, s) {
  return {
    joints: spec.joints.map((j) => ({ axis: j.axis.slice(), origin: v.mul(j.origin, s) })),
    links: spec.links.map((l) => ({ a: v.mul(l.a, s), b: v.mul(l.b, s), r: l.r * s })),
  };
}

// ---- interval refinement (bisection on the sign of gap) --------------------
function boundary(gapFn, inside, outside, maxIter = 80) {
  let lo = outside, hi = inside; // gap(lo) > 0, gap(hi) <= 0
  for (let k = 0; k < maxIter && Math.abs(hi - lo) > 1e-13; k++) {
    const mid = 0.5 * (lo + hi);
    if (gapFn(mid) > 0) lo = mid; else hi = mid;
  }
  return hi; // colliding side of the crossing, |gap| ~ 0
}

export function bracketCollision(spec, traj, i, j, tw, opts = {}) {
  const tMin = opts.tMin ?? traj.times[0], tMax = opts.tMax ?? traj.times[traj.times.length - 1];
  const gapFn = (t) => pairGap(spec, traj, t, i, j);
  const span = tMax - tMin;
  let step = Math.max(span * 1e-4, 1e-12);
  let left = tMin, right = tMax;
  // walk left until the gap turns positive
  let b = tw;
  for (let k = 0; k < 200; k++) {
    const cand = Math.max(tMin, b - step);
    if (gapFn(cand) > 0) { left = cand; break; }
    b = cand;
    if (cand <= tMin) { left = tMin; break; }
    step *= 2;
  }
  step = Math.max(span * 1e-4, 1e-12);
  let a = tw;
  for (let k = 0; k < 200; k++) {
    const cand = Math.min(tMax, a + step);
    if (gapFn(cand) > 0) { right = cand; break; }
    a = cand;
    if (cand >= tMax) { right = tMax; break; }
    step *= 2;
  }
  const tL = gapFn(left) > 0 ? boundary(gapFn, tw, left) : tMin;
  const tR = gapFn(right) > 0 ? boundary(gapFn, tw, right) : tMax;
  return { t0: Math.min(tL, tR), t1: Math.max(tL, tR), witness: tw };
}

// ---- the certificate -------------------------------------------------------
export function certifyPair(spec, traj, i, j, opts = {}) {
  const tMin = opts.tMin ?? traj.times[0], tMax = opts.tMax ?? traj.times[traj.times.length - 1];
  const diag = opts.diag ?? bboxDiag(spec);
  const eps = (opts.relTimeEps ?? 1e-13) * (tMax - tMin);
  const maxDepth = opts.maxDepth ?? 60;
  const knots = traj.times.filter((t) => t > tMin && t < tMax);
  const segs = []; let a = tMin;
  for (const k of knots) { segs.push([a, k]); a = k; }
  segs.push([a, tMax]);

  let evals = 0, unresolved = 0, tangencies = 0;
  const witnesses = [];
  const tangencyBrackets = [];
  const gapAt = (t) => { evals++; return pairGap(spec, traj, t, i, j); };

  for (const [s0, s1] of segs) {
    const stack = [[s0, s1, 0]];
    while (stack.length) {
      const [x0, x1, d] = stack.pop();
      const xm = 0.5 * (x0 + x1);
      const g0 = gapAt(x0), gm = gapAt(xm), g1 = gapAt(x1);
      const gmin = Math.min(g0, gm, g1);
      if (gmin <= 0) {                       // explicit witness: fake positives impossible
        if (g0 <= 0) witnesses.push(x0); else if (gm <= 0) witnesses.push(xm); else witnesses.push(x1);
        continue;
      }
      const L = pairLipschitz(spec, traj, x0, x1, i, j);
      // 3-sample Lipschitz bound: any t in [x0,x1] is within (x1-x0)/2 of a sample
      const lower = gmin - L * (x1 - x0) * 0.5;
      if (lower > 0) continue;               // certified clean on this whole node
      if (d >= maxDepth || x1 - x0 <= eps) {
        tangencyBrackets.push({ t0: x0, t1: x1, lo: lower, hi: gmin });
        tangencies++;
        continue;
      }
      stack.push([xm, x1, d + 1], [x0, xm, d + 1]);
    }
  }
  witnesses.sort((p, q) => p - q);
  const uniq = [];
  for (const w of witnesses) if (!uniq.length || w - uniq[uniq.length - 1] > 1e-12) uniq.push(w);
  let intervals = null;
  if (opts.intervals && uniq.length) {
    const raw = uniq.map((w) => bracketCollision(spec, traj, i, j, w, { tMin, tMax }))
      .sort((a, b) => a.t0 - b.t0);
    intervals = [];
    for (const iv of raw) {
      const last = intervals[intervals.length - 1];
      if (last && iv.t0 <= last.t1 + 1e-9) last.t1 = Math.max(last.t1, iv.t1);
      else intervals.push({ t0: iv.t0, t1: iv.t1 });
    }
  }
  return {
    pair: [i, j], evals, unresolved, tangencies, tangencyBrackets,
    certified: unresolved === 0 && tangencies === 0,
    collides: uniq.length > 0, witnesses: uniq, intervals,
  };
}

export function certifyChain(spec, traj, opts = {}) {
  const minSep = opts.minSep ?? 2;
  const pairs = collisionPairs(spec, minSep);
  const diag = opts.diag ?? bboxDiag(spec);
  const per = [];
  for (const [i, j] of pairs) per.push(certifyPair(spec, traj, i, j, { ...opts, diag }));
  const bad = per.filter((p) => p.collides);
  return {
    chains: 1, pairs: per.length, perPair: per,
    collidingPairs: bad.length,
    collisionIntervals: bad.reduce((s, p) => s + (p.intervals ? p.intervals.length : p.witnesses.length), 0),
    witnesses: bad.flatMap((p) => p.witnesses.map((w) => ({ pair: p.pair, t: w }))),
    unresolved: per.reduce((s, p) => s + p.unresolved + p.tangencies, 0),
    tangencies: per.reduce((s, p) => s + p.tangencies, 0),
    evals: per.reduce((s, p) => s + p.evals, 0),
    certifiedClean: per.filter((p) => !p.collides && p.certified).length,
  };
}

// ---- sampled oracles (for cross-checking; the certificate never relies on them)
export function sampleChain(spec, traj, { samples, minSep = 2, tMin, tMax }) {
  const t0 = tMin ?? traj.times[0], t1 = tMax ?? traj.times[traj.times.length - 1];
  const pairs = collisionPairs(spec, minSep);
  const idx = new Array(spec.links.length);
  for (let i = 0; i < spec.links.length; i++) idx[i] = i;
  const minGap = new Array(pairs.length).fill(Infinity);
  const firstHit = new Array(pairs.length).fill(null);
  for (let k = 0; k < samples; k++) {
    const t = samples === 1 ? t0 : t0 + (t1 - t0) * (k / (samples - 1));
    const F = linkFrames(spec, sampleAngles(traj, t));
    const ep = idx.map((i) => linkEndpoints(spec, F, i));
    for (let p = 0; p < pairs.length; p++) {
      const [i, j] = pairs[p];
      const g = segSegDist(ep[i][0], ep[i][1], ep[j][0], ep[j][1]) - spec.links[i].r - spec.links[j].r;
      if (g < minGap[p]) minGap[p] = g;
      if (g <= 0 && firstHit[p] === null) firstHit[p] = t;
    }
  }
  const hits = [];
  for (let p = 0; p < pairs.length; p++) if (firstHit[p] !== null) hits.push({ pair: pairs[p], t: firstHit[p], minGap: minGap[p] });
  return { samples, pairs: pairs.length, hits, hitPairs: hits.length, pairsArr: pairs, minGap };
}

// ---- minimal motion damping repair ----------------------------------------
export function dampTrajectory(traj, s) {
  const rest = traj.angles[0];
  return {
    times: traj.times.slice(),
    angles: traj.angles.map((row) => row.map((a, j) => rest[j] + s * (a - rest[j]))),
  };
}

export function dampSearch(spec, traj, opts = {}) {
  const iters = opts.iters ?? 44;
  const minSep = opts.minSep ?? 2;
  const test = (s) => certifyChain(spec, dampTrajectory(traj, s), { minSep, intervals: false, diag: opts.diag });
  const base0 = test(0);
  if (base0.collidingPairs > 0 || base0.unresolved > 0) {
    return { factor: 0, certifiedClean: false, feasible: false, reason: 'rest-pose-not-clean', iters: 1 };
  }
  const at1 = test(1);
  if (at1.collidingPairs === 0 && at1.unresolved === 0) {
    return { factor: 1, certifiedClean: true, feasible: true, reason: 'original-motion-clean', iters: 1 };
  }
  let lo = 0, hi = 1, n = 0;
  for (let k = 0; k < iters; k++) {
    const mid = 0.5 * (lo + hi);
    const r = test(mid); n++;
    if (r.collidingPairs === 0 && r.unresolved === 0) lo = mid; else hi = mid;
  }
  return { factor: lo, bracket: [lo, hi], certifiedClean: true, feasible: true, iters: n, reason: 'bisected' };
}

// ---- trajectory helpers ----------------------------------------------------
export function keyframeTimes(traj) { return traj.times.slice(); }

export function rectPolyline(traj) {
  // exact sup-norm speed per joint (piecewise linear => slope is exact)
  const K = traj.times.length, J = traj.angles[0].length, out = new Array(J).fill(0);
  for (let k = 1; k < K; k++) {
    const dt = traj.times[k] - traj.times[k - 1];
    for (let j = 0; j < J; j++) out[j] = Math.max(out[j], Math.abs(traj.angles[k][j] - traj.angles[k - 1][j]) / dt);
  }
  return out;
}
