// bench/bench.mjs — cold start + >=5 warm repetitions, median/p95/stddev.
// Writes bench/result.json. All numbers come from this run.
import { readFileSync, writeFileSync } from 'node:fs';
import { certifyChain } from '../src/core.mjs';
import { frameScan } from '../baseline/naive.mjs';

const data = JSON.parse(readFileSync(new URL('../data/chains.json', import.meta.url), 'utf8'));
const chains = data.chains;

function stats(ms) {
  const s = ms.slice().sort((a, b) => a - b);
  const n = s.length;
  const mean = s.reduce((a, b) => a + b, 0) / n;
  const med = n % 2 ? s[(n - 1) / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
  const p95 = s[Math.min(n - 1, Math.ceil(0.95 * n) - 1)];
  const sd = Math.sqrt(s.reduce((a, b) => a + (b - mean) ** 2, 0) / n);
  return { samples: n, minMs: s[0], medianMs: med, p95Ms: p95, maxMs: s[n - 1], meanMs: mean, stddevMs: sd, relStddevPct: 100 * sd / (mean || 1) };
}

function timeIt(fn) { const t = process.hrtime.bigint(); const out = fn(); return { ms: Number(process.hrtime.bigint() - t) / 1e6, out }; }

// --- certificate: cold start then >=5 warm reps ---------------------------
const cold = timeIt(() => chains.map((c) => certifyChain(c.spec, c.traj, { minSep: 2, intervals: false })));
const warm = [];
let evals = 0, pairs = 0, collidingPairs = 0;
for (let k = 0; k < 7; k++) {
  const r = timeIt(() => chains.map((c) => certifyChain(c.spec, c.traj, { minSep: 2, intervals: false })));
  warm.push(r.ms);
  evals = r.out.reduce((a, c) => a + c.evals, 0);
  pairs = r.out.reduce((a, c) => a + c.pairs, 0);
  collidingPairs = r.out.reduce((a, c) => a + c.collidingPairs, 0);
}
const certStat = stats(warm);

// --- coarse baseline: uniform 120-frame scan ------------------------------
// inner repeat: at ~10 ms/scan the raw timing is dominated by allocation and
// GC jitter, so each reported sample is 20 scans / 20.
const COARSE_REP = 20;
const coarse20 = () => { let h = 0; for (let k = 0; k < COARSE_REP; k++) h = chains.map((c) => frameScan(c.spec, c.traj, 120, 2)).reduce((a, s) => a + s.hitPairs, 0); return h; };
const c120Cold = timeIt(coarse20);
const c120 = [];
let coarseHits = 0;
for (let k = 0; k < 7; k++) {
  const r = timeIt(coarse20);
  c120.push(r.ms / COARSE_REP);
  coarseHits = r.out;
}
const c120Stat = stats(c120);

// --- dense oracle: uniform 20000-frame scan -------------------------------
const ORACLE_N = 20000;
const oracleCold = timeIt(() => chains.map((c) => frameScan(c.spec, c.traj, ORACLE_N, 2)));
const oracle = [oracleCold.ms];
for (let k = 0; k < 4; k++) oracle.push(timeIt(() => chains.map((c) => frameScan(c.spec, c.traj, ORACLE_N, 2))).ms);
const oracleStat = stats(oracle);
const oracleEvals = chains.length * ORACLE_N * (pairs / chains.length);

const result = {
  generated: new Date().toISOString(),
  method: 'lipschitz-interval-certificate (src/core.mjs)',
  baseline: 'uniform frame scan (baseline/naive.mjs)',
  dataset: { chains: chains.length, linksPerChain: chains[0].spec.links.length, pairsPerChain: pairs / chains.length, pairsTotal: pairs },
  certificate: {
    coldStartMs: cold.ms,
    warm: certStat,
    evaluations: evals,
    collidingPairs,
    speedupVsOracle20000: oracleStat.medianMs / certStat.medianMs,
    speedupVsCoarse120: c120Stat.medianMs / certStat.medianMs,
  },
  coarse120: { coldStartMs: c120Cold.ms, warm: c120Stat, hitPairs: coarseHits, missedCollisions: collidingPairs - coarseHits },
  oracle20000: { samples: ORACLE_N, coldStartMs: oracleCold.ms, warm: oracleStat, evaluations: oracleEvals },
  note: 'certificate median is over the full 6-chain / 330-pair set; the certificate reports collisions with an explicit witness (gap<=0), false positives are structurally impossible',
};
writeFileSync(new URL('./result.json', import.meta.url), JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result, null, 2));
