// tools/gen-sample.mjs — deterministic generator for the 6 articulated chains.
// Usage: node tools/gen-sample.mjs   (writes data/chains.json)
import { writeFileSync, mkdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { certifyChain, dampSearch, bboxDiag } from '../src/core.mjs';

const LINKS = 12;
const AXIS_TILT = (i, tilt) => [tilt * Math.sin(i * 1.1), tilt * Math.cos(i * 1.1), 1];
const WOBBLE = (j, wob) => 1 + wob * Math.sin(j * 2.7);

// keyframe schedules: `sharp` squeezes the curl through a ~5e-4 wide time window
const SHARP_F = [0, 0.05, 0.15, 0.35, 0.55, 0.8, 0.9, 1.0, 0.9, 0.8, 0.45, 0.2, 0.05];
const SHARP_T = [0, 0.1, 0.2, 0.3, 0.45, 0.49, 0.499, 0.5, 0.501, 0.51, 0.6, 0.8, 1.0];
const SMOOTH_F = [0, 0.05, 0.15, 0.3, 1.0, 0.9, 0.6, 0.35, 0.1, 0.02];
const SMOOTH_T = [0, 0.1, 0.25, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

export const CONFIG = [
  { id: 'curl-tight', curl: 0.98, tilt: 0.05, wob: 0.12, sched: 'sharp', expect: 'collide' },
  { id: 'curl-over', curl: 1.06, tilt: 0.09, wob: 0.18, sched: 'sharp', expect: 'collide' },
  { id: 'curl-3d', curl: 1.09, tilt: 0.16, wob: 0.06, sched: 'sharp', expect: 'collide' },
  { id: 'arc-open', curl: 0.34, tilt: 0.12, wob: 0.20, sched: 'smooth', expect: 'clean' },
  { id: 'arc-drape', curl: 0.44, tilt: 0.30, wob: 0.15, sched: 'smooth', expect: 'clean' },
  { id: 'arc-twist', curl: 0.26, tilt: 0.46, wob: 0.25, sched: 'smooth', expect: 'clean' },
];

export function buildChain(cfg) {
  const joints = [], links = [];
  for (let i = 0; i < LINKS; i++) {
    joints.push({ axis: AXIS_TILT(i, cfg.tilt), origin: i === 0 ? [0, 0, 0] : [1, 0, 0] });
    links.push({ a: [0, 0, 0], b: [1, 0, 0], r: 0.18 });
  }
  const spec = { joints, links };
  const delta = (2 * Math.PI / LINKS) * cfg.curl;
  const T = cfg.sched === 'sharp' ? SHARP_T : SMOOTH_T;
  const F = cfg.sched === 'sharp' ? SHARP_F : SMOOTH_F;
  const times = T.slice();
  const angles = F.map((f) => Array.from({ length: LINKS }, (_, j) => delta * WOBBLE(j, cfg.wob) * f));
  return { id: cfg.id, expect: cfg.expect, spec, traj: { times, angles } };
}

export function generate() {
  const chains = CONFIG.map(buildChain);
  const out = { generator: 'tools/gen-sample.mjs', links: LINKS, minSep: 2, chains: [] };
  for (const c of chains) {
    const cert = certifyChain(c.spec, c.traj, { minSep: 2, intervals: true });
    const damp = dampSearch(c.spec, c.traj, { minSep: 2, iters: 40 });
    out.chains.push({
      id: c.id, expect: c.expect, spec: c.spec, traj: c.traj,
      bboxDiag: bboxDiag(c.spec),
      certified: {
        pairs: cert.pairs, collidingPairs: cert.collidingPairs, unresolved: cert.unresolved,
        witnesses: cert.witnesses, intervals: cert.perPair.filter((p) => p.collides).flatMap((p) => (p.intervals || []).map((iv) => ({ pair: p.pair, t0: iv.t0, t1: iv.t1 }))),
      },
      damping: damp,
    });
  }
  return out;
}

export function serialize(obj) { return JSON.stringify(obj, null, 1) + '\n'; }

function main() {
  const data = generate();
  const text = serialize(data);
  mkdirSync(new URL('../data/', import.meta.url), { recursive: true });
  writeFileSync(new URL('../data/chains.json', import.meta.url), text);
  const sha = createHash('sha256').update(text).digest('hex');
  console.log(`wrote data/chains.json  bytes=${Buffer.byteLength(text)}  sha256=${sha}`);
  for (const c of data.chains) {
    console.log(`  ${c.id.padEnd(12)} expect=${c.expect.padEnd(8)} collidingPairs=${c.certified.collidingPairs} unresolved=${c.certified.unresolved} intervals=${c.certified.intervals.length} damping=${c.damping.factor.toFixed(6)}`);
  }
}

if (import.meta.url === `file://${process.argv[1]}`) main();
