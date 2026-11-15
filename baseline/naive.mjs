// baseline/naive.mjs — independent sampling baseline / oracle.
//
// Written from scratch (quaternion rotations, its own segment-segment routine)
// so that agreement with src/core.mjs is evidence rather than shared code.
// It shares NO magic constant with the certificate: the only knob is the frame
// count N, and it is exactly the thing the certificate is meant to replace.
//
// Usage: node baseline/naive.mjs [samples]

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const unit = (a) => { const l = norm(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

// rotate vector p by unit quaternion q = [w,x,y,z]
function qrot(q, p) {
  const [w, x, y, z] = q;
  const t = cross([x, y, z], p);
  const tt = mul(t, 2);
  return add(add(p, mul(tt, w)), cross([x, y, z], tt));
}
function qmul(a, b) {
  const [aw, ax, ay, az] = a, [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}
function axisQuat(axis, ang) {
  const [x, y, z] = unit(axis), h = ang / 2, s = Math.sin(h);
  return [Math.cos(h), x * s, y * s, z * s];
}

export function framesAt(spec, angles) {
  const n = spec.links.length, q = new Array(n), T = new Array(n);
  for (let i = 0; i < n; i++) {
    const rel = axisQuat(spec.joints[i].axis, angles[i]);
    if (i === 0) { q[0] = rel; T[0] = spec.joints[0].origin.slice(); }
    else { q[i] = qmul(q[i - 1], rel); T[i] = add(T[i - 1], qrot(q[i - 1], spec.joints[i].origin)); }
  }
  return { q, T };
}

// distance between segments AB and CD (parametric clamp, independent derivation)
export function segDist(A, B, C, D) {
  const u = sub(B, A), v = sub(D, C), w = sub(A, C);
  const a = dot(u, u), b = dot(u, v), c = dot(v, v), d = dot(u, w), e = dot(v, w);
  const den = a * c - b * b;
  let s = 0, t = 0;
  if (den > 1e-30) {
    s = (b * e - c * d) / den;
    t = (a * e - b * d) / den;
  }
  s = Math.max(0, Math.min(1, s)); t = Math.max(0, Math.min(1, t));
  const p = add(A, mul(u, s)), q2 = add(C, mul(v, t));
  return norm(sub(p, q2));
}

export function anglesAt(traj, t) {
  const T = traj.times, K = T.length;
  if (t <= T[0]) return traj.angles[0].slice();
  if (t >= T[K - 1]) return traj.angles[K - 1].slice();
  let k = 1; while (T[k] < t) k++;
  const u = (t - T[k - 1]) / (T[k] - T[k - 1]);
  return traj.angles[k - 1].map((a, j) => a + (traj.angles[k][j] - a) * u);
}

export function gapAt(spec, traj, t, i, j) {
  const F = framesAt(spec, anglesAt(traj, t));
  const e = (k, p) => add(F.T[k], qrot(F.q[k], p));
  return segDist(e(i, spec.links[i].a), e(i, spec.links[i].b), e(j, spec.links[j].a), e(j, spec.links[j].b))
    - spec.links[i].r - spec.links[j].r;
}

export function pairList(spec, minSep = 2) {
  const out = [];
  for (let i = 0; i < spec.links.length; i++) for (let j = i + minSep; j < spec.links.length; j++) out.push([i, j]);
  return out;
}

// Uniform frame scan — the naive method. Returns first-hit time per pair.
export function frameScan(spec, traj, samples, minSep = 2) {
  const t0 = traj.times[0], t1 = traj.times[traj.times.length - 1];
  const pairs = pairList(spec, minSep);
  const first = new Array(pairs.length).fill(null);
  const minGap = new Array(pairs.length).fill(Infinity);
  for (let k = 0; k < samples; k++) {
    const t = samples === 1 ? t0 : t0 + (t1 - t0) * (k / (samples - 1));
    const F = framesAt(spec, anglesAt(traj, t));
    const E = spec.links.map((_, i) => [add(F.T[i], qrot(F.q[i], spec.links[i].a)), add(F.T[i], qrot(F.q[i], spec.links[i].b))]);
    for (let p = 0; p < pairs.length; p++) {
      const [i, j] = pairs[p];
      const g = segDist(E[i][0], E[i][1], E[j][0], E[j][1]) - spec.links[i].r - spec.links[j].r;
      if (g < minGap[p]) minGap[p] = g;
      if (g <= 0 && first[p] === null) first[p] = t;
    }
  }
  const hits = [];
  for (let p = 0; p < pairs.length; p++) if (first[p] !== null) hits.push({ pair: pairs[p], t: first[p], minGap: minGap[p] });
  return { samples, pairs: pairs.length, hits, hitPairs: hits.length, minGap };
}
