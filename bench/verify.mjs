// bench/verify.mjs — behavioural assertions over the certificate.
// Every number printed here is recomputed from the real data / real core.
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import {
  certifyChain, certifyPair, pairGap, dampTrajectory, scaleSpec, bboxDiag, collisionPairs,
} from '../src/core.mjs';
import { generate, serialize } from '../tools/gen-sample.mjs';
import { gapAt as naiveGap, frameScan, segDist } from '../baseline/naive.mjs';

const results = [];
let failed = 0;
function check(name, ok, detail) {
  results.push({ name, ok, detail });
  if (!ok) failed++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ::  ${detail}`);
}
const fmt = (x, d = 6) => (typeof x === 'number' ? x.toFixed(d) : String(x));

// --- 1. deterministic generator -------------------------------------------
const g1 = serialize(generate());
const g2 = serialize(generate());
const s1 = createHash('sha256').update(g1).digest('hex');
const s2 = createHash('sha256').update(g2).digest('hex');
check('G7 generator is byte-deterministic', s1 === s2, `sha256=${s1.slice(0, 16)} twice`);

const data = JSON.parse(g1);
check('G7 data/chains.json on disk matches a fresh generate()',
  readFileSync(new URL('../data/chains.json', import.meta.url), 'utf8') === g1,
  `bytes=${Buffer.byteLength(g1)}`);

// --- 2. chain / pair bookkeeping ------------------------------------------
const N = data.chains.length;
const pairsPerChain = collisionPairs(data.chains[0].spec, 2).length;
const totalPairs = N * pairsPerChain;
check('chain count == 6', N === 6, `chains=${N}`);
check('links per chain == 12', data.chains[0].spec.links.length === 12, `links=${data.chains[0].spec.links.length}`);
check('>= 120 link pairs certified', totalPairs >= 120, `pairs=${totalPairs} (${pairsPerChain}/chain x ${N})`);

// --- 3. certificate vs. honest expectations -------------------------------
let cleanOk = 0, collideChains = 0, allWitnessesValid = true, unresolvedTotal = 0;
const certPairs = new Set();
for (const c of data.chains) {
  const cert = certifyChain(c.spec, c.traj, { minSep: 2, intervals: true });
  unresolvedTotal += cert.unresolved;
  if (c.expect === 'clean') {
    if (cert.collidingPairs === 0 && cert.unresolved === 0) cleanOk++;
  } else {
    if (cert.collidingPairs > 0) collideChains++;
  }
  for (const p of cert.perPair.filter((q) => q.collides)) {
    certPairs.add(`${c.id}:${p.pair[0]}-${p.pair[1]}`);
    for (const w of p.witnesses) {
      // independent confirmation of the witness with the from-scratch oracle
      const g = naiveGap(c.spec, c.traj, w, p.pair[0], p.pair[1]);
      if (g > 0) allWitnessesValid = false;
    }
  }
}
check('clean chains certify with 0 collisions', cleanOk === 3, `clean-certified=${cleanOk}/3`);
check('colliding chains really collide', collideChains === 3, `colliding=${collideChains}/3`);
check('every witness has gap <= 0 under the independent oracle', allWitnessesValid,
  `witnesses=${certPairs.size} verified by baseline/naive.mjs`);
check('no unresolved interval', unresolvedTotal === 0, `unresolved=${unresolvedTotal}`);

// --- 4. independent gap implementations agree -----------------------------
let maxDiff = 0;
for (const c of data.chains.slice(0, 2)) {
  for (const t of [0.13, 0.37, 0.5, 0.77, 0.91]) {
    for (const [i, j] of [[0, 5], [3, 9], [0, 11]]) {
      const a = pairGap(c.spec, c.traj, t, i, j);
      const b = naiveGap(c.spec, c.traj, t, i, j);
      maxDiff = Math.max(maxDiff, Math.abs(a - b));
    }
  }
}
check('core and oracle gaps agree', maxDiff < 1e-12, `maxAbsDiff=${maxDiff.toExponential(3)}`);

// --- 5. dense oracle: 0 missed, 0 false -----------------------------------
const ORACLE_N = 20000;
const oracleHits = new Set();
let oracleEvals = 0;
for (const c of data.chains) {
  const scan = frameScan(c.spec, c.traj, ORACLE_N, 2);
  oracleEvals += scan.samples * scan.pairs;
  for (const h of scan.hits) oracleHits.add(`${c.id}:${h.pair[0]}-${h.pair[1]}`);
}
const missed = [...oracleHits].filter((k) => !certPairs.has(k));
const extra = [...certPairs].filter((k) => !oracleHits.has(k));
check(`dense oracle (${ORACLE_N} frames x ${pairsPerChain} pairs) agrees exactly`,
  missed.length === 0 && extra.length === 0,
  `oracleHitPairs=${oracleHits.size} certifiedPairs=${certPairs.size} missed=${missed.length} false=${extra.length} evals=${oracleEvals}`);

// --- 6. thin windows: coarse frame budgets miss them ----------------------
const coarseMiss = {};
for (const frames of [120, 1200]) {
  let hit = 0;
  for (const c of data.chains) {
    const scan = frameScan(c.spec, c.traj, frames, 2);
    hit += scan.hits.length;
  }
  coarseMiss[frames] = certPairs.size - hit;
}
check('uniform 120-frame scan misses collisions', coarseMiss[120] > 0, `missed=${coarseMiss[120]} of ${certPairs.size}`);
check('uniform 1200-frame scan misses collisions', coarseMiss[1200] > 0, `missed=${coarseMiss[1200]} of ${certPairs.size}`);

// --- 7. scale invariance 1e-6 .. 1e6 --------------------------------------
const scales = [1e-6, 1e-3, 1, 1e3, 1e6];
const scaleReport = [];
let scaleStable = true;
for (const s of scales) {
  const set = new Set();
  for (const c of data.chains) {
    const cert = certifyChain(scaleSpec(c.spec, s), c.traj, { minSep: 2, intervals: false });
    for (const p of cert.perPair.filter((q) => q.collides)) set.add(`${c.id}:${p.pair[0]}-${p.pair[1]}`);
  }
  const same = set.size === certPairs.size && [...set].every((k) => certPairs.has(k));
  if (!same) scaleStable = false;
  scaleReport.push(`${s}:${set.size}`);
}
check('collision verdict set is scale-invariant', scaleStable, `pairs per scale ${scaleReport.join(' ')}`);

// --- 8. minimal motion damping repair -------------------------------------
const c0 = data.chains[0];
const damp = c0.damping;
const damped = certifyChain(c0.spec, dampTrajectory(c0.traj, damp.factor), { minSep: 2, intervals: false });
check('repair factor < 1 and damped motion certifies clean',
  damp.factor < 1 && damp.factor > 0 && damped.collidingPairs === 0 && damped.unresolved === 0,
  `factor=${fmt(damp.factor)} dampedCollisions=${damped.collidingPairs} unresolved=${damped.unresolved}`);

// --- 9. interval brackets are real ----------------------------------------
let bracketOk = true;
const ivs = data.chains.flatMap((c) => c.certified.intervals.map((iv) => ({ ...iv, id: c.id, spec: c.spec, traj: c.traj })));
for (const iv of ivs) {
  const mid = 0.5 * (iv.t0 + iv.t1);
  const gm = naiveGap(iv.spec, iv.traj, mid, iv.pair[0], iv.pair[1]);
  if (!(gm <= 0)) bracketOk = false;
}
check('each reported interval contains a real collision', bracketOk && ivs.length >= 3,
  `intervals=${ivs.length} widths=[${ivs.map((v) => fmt(v.t1 - v.t0, 8)).join(', ')}]`);

// --- 10. capsule distance sanity (shared by both implementations) ---------
const dOrth = segDist([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 1, 1]);
check('segment distance sanity', Math.abs(dOrth - 1) < 1e-14, `orthogonal offset distance=${dOrth}`);

console.log(`\nRESULT ${failed === 0 ? 'PASS' : 'FAIL'}: ${results.length - failed} passed / ${failed} failed`);
process.exit(failed === 0 ? 0 : 1);
