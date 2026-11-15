// main.js — render layer only. Every number on screen is produced by the same
// src/core.mjs that bench/verify.mjs runs in node; this file computes nothing.
import * as THREE from 'three';
import { certifyChain, pairGap, sampleAngles, linkFrames, linkEndpoints, collisionPairs, dampTrajectory } from './src/core.mjs';

const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e13);
const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 500);
const cam = { theta: 0.9, phi: 1.1, dist: 26 };
camera.position.set(0, 0, 1);
const grid = new THREE.GridHelper(40, 40, 0x1b2633, 0x141c26);
scene.add(grid);
scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x1a2230, 1.1));
const key = new THREE.DirectionalLight(0xffffff, 1.4); key.position.set(6, 14, 10); scene.add(key);

const SEG = new THREE.CylinderGeometry(1, 1, 1, 14, 1, true);
const CAP = new THREE.SphereGeometry(1, 14, 10);
const UP = new THREE.Vector3(0, 1, 0);

const data = await fetch('./data/chains.json').then((r) => r.json());
const chainSel = document.getElementById('chain');
data.chains.forEach((c, i) => chainSel.add(new Option(`${c.id} (${c.expect})`, String(i))));
const tSlider = document.getElementById('t');
const sSlider = document.getElementById('s');
const verdict = document.getElementById('verdict');
const ivalBody = document.querySelector('#ivals tbody');

let current = 0;
let dampedTraj = null;
let lastCert = null;

// one mesh per link per chain, created once
const groups = [], linkMeshes = [];
for (const c of data.chains) {
  const g = new THREE.Group();
  const meshes = [];
  for (let i = 0; i < c.spec.links.length; i++) {
    const mat = new THREE.MeshStandardMaterial({ color: 0x4c8bf5, roughness: 0.45, metalness: 0.1, transparent: true, opacity: 0.92 });
    const seg = new THREE.Mesh(SEG, mat), capA = new THREE.Mesh(CAP, mat), capB = new THREE.Mesh(CAP, mat);
    g.add(seg, capA, capB);
    meshes.push({ seg, capA, capB, mat, base: new THREE.Color(0x4c8bf5) });
  }
  groups.push(g); linkMeshes.push(meshes); scene.add(g);
  g.visible = false;
}

function activeChain() { return data.chains[current]; }
function activeTraj() { return dampedTraj ?? activeChain().traj; }

function pose() {
  const c = activeChain(), traj = activeTraj();
  const t = Number(tSlider.value);
  const F = linkFrames(c.spec, sampleAngles(traj, t));
  const gaps = collisionPairs(c.spec, 2).map(([i, j]) => ({ i, j, g: pairGap(c.spec, traj, t, i, j) }));
  const hit = new Set();
  for (const q of gaps) if (q.g <= 0) { hit.add(q.i); hit.add(q.j); }
  for (let i = 0; i < c.spec.links.length; i++) {
    const [A, B] = linkEndpoints(c.spec, F, i);
    const a = new THREE.Vector3(...A), b = new THREE.Vector3(...B);
    const dir = new THREE.Vector3().subVectors(b, a);
    const len = dir.length(); dir.normalize();
    const { seg, capA, capB, mat } = linkMeshes[current][i];
    const r = c.spec.links[i].r;
    seg.position.copy(a).addScaledVector(dir, len / 2);
    seg.quaternion.setFromUnitVectors(UP, dir);
    seg.scale.set(r, len, r);
    capA.position.copy(a); capA.scale.setScalar(r);
    capB.position.copy(b); capB.scale.setScalar(r);
    const col = hit.has(i) ? 0xff5c5c : 0x4c8bf5;
    mat.color.setHex(col);
  }
  return { gaps, hit };
}

function recertify() {
  const c = activeChain(), traj = activeTraj();
  const t0 = performance.now();
  const cert = certifyChain(c.spec, traj, { minSep: 2, intervals: true });
  const ms = performance.now() - t0;
  lastCert = { cert, ms };
  const clean = cert.collidingPairs === 0 && cert.unresolved === 0;
  verdict.innerHTML = `pairs certified <b>${cert.pairs}</b><br>` +
    `colliding pairs <span class="${cert.collidingPairs ? 'bad' : 'ok'}">${cert.collidingPairs}</span><br>` +
    `unresolved <span class="${cert.unresolved ? 'warn' : 'ok'}">${cert.unresolved}</span><br>` +
    `gap evaluations <b>${cert.evals}</b><br>` +
    `live re-certify <b>${ms.toFixed(1)} ms</b><br>` +
    `<span class="${clean ? 'ok' : 'bad'}">${clean ? 'CERTIFIED COLLISION-FREE on the whole interval' : 'COLLISION POSSIBLE — see intervals'}</span>`;
  ivalBody.innerHTML = '';
  for (const p of cert.perPair.filter((q) => q.collides)) {
    for (const iv of p.intervals ?? []) {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>[${p.pair[0]},${p.pair[1]}]</td><td>[${iv.t0.toFixed(6)}, ${iv.t1.toFixed(6)}]</td><td>${(iv.t1 - iv.t0).toExponential(2)}</td>`;
      ivalBody.appendChild(tr);
    }
  }
  if (!ivalBody.children.length) ivalBody.innerHTML = '<tr><td colspan="3" class="ok">no collision interval</td></tr>';
  return cert;
}

function sync() {
  document.getElementById('tval').textContent = Number(tSlider.value).toFixed(3);
  document.getElementById('sval').textContent = Number(sSlider.value).toFixed(3);
  for (let i = 0; i < groups.length; i++) groups[i].visible = i === current;
  pose();
}

chainSel.onchange = () => { current = Number(chainSel.value); dampedTraj = null; recertify(); sync(); };
sSlider.oninput = () => {
  const c = activeChain();
  dampedTraj = dampTrajectory(c.traj, Number(sSlider.value));
  sync();
};
sSlider.onchange = () => recertify();
tSlider.oninput = sync;
document.getElementById('recert').onclick = () => recertify();

function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * devicePixelRatio || canvas.height !== h * devicePixelRatio) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
}
canvas.addEventListener('pointerdown', (e) => { canvas.setPointerCapture(e.pointerId); canvas.dataset.drag = e.pointerId; canvas.dataset.x = e.clientX; canvas.dataset.y = e.clientY; });
canvas.addEventListener('pointermove', (e) => {
  if (canvas.dataset.drag === undefined || Number(canvas.dataset.drag) !== e.pointerId) return;
  cam.theta -= (e.clientX - Number(canvas.dataset.x)) * 0.008;
  cam.phi = Math.max(0.08, Math.min(Math.PI - 0.08, cam.phi - (e.clientY - Number(canvas.dataset.y)) * 0.008));
  canvas.dataset.x = e.clientX; canvas.dataset.y = e.clientY;
});
canvas.addEventListener('pointerup', (e) => { canvas.releasePointerCapture(e.pointerId); delete canvas.dataset.drag; });

recertify();
sync();
function frame() {
  resize();
  cam.dist = 26;
  camera.position.set(
    cam.dist * Math.sin(cam.phi) * Math.cos(cam.theta),
    cam.dist * Math.cos(cam.phi),
    cam.dist * Math.sin(cam.phi) * Math.sin(cam.theta),
  );
  camera.lookAt(0, 0, 0);
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
frame();
