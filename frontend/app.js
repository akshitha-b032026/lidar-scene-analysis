const $ = id => document.getElementById(id);
const api = p => fetch('/api' + p).then(r => { if (!r.ok) throw new Error(r.statusText); return r; });

// ---- categories --------------------------------------------------------------
const GROUPS = {
  vehicle: ['Vehicles', 0x5cc8ff], human: ['Pedestrians', 0xff5d8f], cycle: ['Cycles', 0xffc857],
  movable_object: ['Movable objects', 0x7bdc9f], animal: ['Animals', 0xc77dff], static_object: ['Static objects', 0x9aa8bd],
};
const groupOf = c => c.startsWith('human') ? 'human' : /bicycle|motorcycle/.test(c) ? 'cycle'
  : GROUPS[c.split('.')[0]] ? c.split('.')[0] : 'static_object';

const CAMN = { CAM_FRONT_LEFT: 'Front left', CAM_FRONT: 'Front', CAM_FRONT_RIGHT: 'Front right', CAM_BACK_LEFT: 'Back left', CAM_BACK: 'Back', CAM_BACK_RIGHT: 'Back right' };
const state = { mode: 'split', camLayout: 4, camData: {}, flags: {}, cams: [], cam: 'CAM_FRONT', samples: [], boxes: [], selected: null, hidden: new Set(), raw: null, timer: null, loadId: 0 };

const shown = b => !state.hidden.has(groupOf(b.category)) && (!$('onlyFlagged').checked || state.flags[b.id]);

// ---- three.js scene ----------------------------------------------------------
const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b121b);
const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 500);
camera.up.set(0, 0, 1);               // ego frame: x forward, y left, z up
camera.position.set(-18, -14, 14);
const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(8, 0, 0);
controls.update();

const grid = new THREE.GridHelper(120, 24, 0x2a3b52, 0x1a2738);
grid.rotation.x = Math.PI / 2; grid.position.z = -1.9; scene.add(grid);
// ego vehicle marker
const ego = new THREE.Mesh(new THREE.BoxGeometry(4.1, 1.85, 1.5), new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true }));
ego.position.set(0, 0, -1.1); scene.add(ego);

let cloud = null, boxGroup = new THREE.Group(); scene.add(boxGroup);
const pickables = [];
const cloudMat = new THREE.PointsMaterial({ size: 1.6, sizeAttenuation: false, vertexColors: true });

function resize() {
  const { clientWidth: w, clientHeight: h } = view;
  if (!w || !h) return;                 // view is hidden in camera-only mode
  renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view); resize();
(function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();

// ---- point cloud -------------------------------------------------------------
function ramp(t) {            // 0..1 -> blue → teal → yellow → red
  const c = new THREE.Color(); c.setHSL((1 - Math.min(Math.max(t, 0), 1)) * 0.68, 0.85, 0.55); return c;
}
function colorize() {
  if (!state.raw) return;
  const n = state.raw.length / 4, mode = $('color').value, col = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const x = state.raw[4 * i], y = state.raw[4 * i + 1], z = state.raw[4 * i + 2], it = state.raw[4 * i + 3];
    const t = mode === 'height' ? (z + 2) / 6 : mode === 'intensity' ? it / 100 : Math.hypot(x, y) / +$('range').value;
    const c = ramp(t); col[3 * i] = c.r; col[3 * i + 1] = c.g; col[3 * i + 2] = c.b;
  }
  cloud.geometry.setAttribute('color', new THREE.BufferAttribute(col, 3));
}
function setCloud(buf) {
  state.raw = new Float32Array(buf);
  const n = state.raw.length / 4, pos = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) { pos[3 * i] = state.raw[4 * i]; pos[3 * i + 1] = state.raw[4 * i + 1]; pos[3 * i + 2] = state.raw[4 * i + 2]; }
  if (cloud) { scene.remove(cloud); cloud.geometry.dispose(); }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  cloud = new THREE.Points(g, cloudMat); cloud.frustumCulled = false; scene.add(cloud);
  colorize();
  return n;
}

// ---- boxes -------------------------------------------------------------------
const EDGES = [0,1,1,2,2,3,3,0, 4,5,5,6,6,7,7,4, 0,4,1,5,2,6,3,7];
function buildBoxes() {
  boxGroup.clear(); pickables.length = 0;
  state.boxes.forEach(b => {
    if (!shown(b)) return;
    const sel = state.selected === b.id, color = sel ? 0xffffff : GROUPS[groupOf(b.category)][1];
    const pts = [];
    EDGES.forEach(i => pts.push(new THREE.Vector3(...b.corners[i])));
    const mid = new THREE.Vector3(...b.corners[0]).add(new THREE.Vector3(...b.corners[1])).multiplyScalar(0.5); // heading tick
    pts.push(new THREE.Vector3(...b.center), mid);
    const lines = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color }));
    boxGroup.add(lines);
    if (state.flags[b.id]) {            // amber marker above flagged boxes
      const mk = new THREE.Mesh(new THREE.OctahedronGeometry(0.3), new THREE.MeshBasicMaterial({ color: 0xff9f1c }));
      mk.position.set(b.center[0], b.center[1], b.center[2] + b.size[2] / 2 + 0.8); boxGroup.add(mk);
    }
    const [w, l, h] = b.size;
    const m = new THREE.Mesh(new THREE.BoxGeometry(l, w, h),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: sel ? 0.22 : 0.0, depthWrite: false }));
    m.position.set(...b.center); m.rotation.z = b.yaw; m.userData.id = b.id;
    boxGroup.add(m); pickables.push(m);
  });
}

// ---- UI ----------------------------------------------------------------------
function renderList() {
  const vis = state.boxes.filter(shown).sort((a, b) => a.distance - b.distance);
  $('count').textContent = `(${vis.length})`;
  $('list').innerHTML = vis.map(b => `<li data-id="${b.id}" class="${b.id === state.selected ? 'sel' : ''}">
    <span><span class="sw" style="background:#${GROUPS[groupOf(b.category)][1].toString(16).padStart(6, '0')}"></span> ${state.flags[b.id] ? '⚠ ' : ''}${b.category.replace(/\./g, ' › ')}</span>
    <span class="d">${b.distance.toFixed(1)} m</span></li>`).join('') || '<li class="muted">No objects</li>';
}
function renderDetail() {
  const b = state.boxes.find(x => x.id === state.selected);
  $('detail').hidden = !b; if (!b) return;
  const f = (a, d = 2) => a.map(v => v.toFixed(d)).join(', ');
  $('detailBody').innerHTML = [
    ['Category', b.category], ['Position (x, y, z)', f(b.center)], ['Size (w × l × h)', f(b.size) + ' m'],
    ['Distance', b.distance.toFixed(2) + ' m'], ['Heading', b.yaw_deg + '°'],
    ['LiDAR points', b.num_lidar_pts], ['Visibility', b.visibility],
    ['Checks', state.flags[b.id] ? state.flags[b.id].issues.join('; ') : 'No issues found'],
  ].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
}
function select(id) {
  state.selected = state.selected === id ? null : id;
  buildBoxes(); renderList(); renderDetail(); drawOverlay();
}
$('list').onclick = e => { const li = e.target.closest('li[data-id]'); if (li) select(li.dataset.id); };

const ray = new THREE.Raycaster(), mouse = new THREE.Vector2();
let down = null;
renderer.domElement.addEventListener('pointerdown', e => down = [e.clientX, e.clientY]);
renderer.domElement.addEventListener('pointerup', e => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;   // ignore drags
  const r = renderer.domElement.getBoundingClientRect();
  mouse.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObjects(pickables)[0];
  if (hit) select(hit.object.userData.id); else if (state.selected) select(state.selected);
});

function buildFilters() {
  $('filters').innerHTML = Object.entries(GROUPS).map(([k, [name, c]]) =>
    `<label><input type="checkbox" data-g="${k}" checked><span class="sw" style="background:#${c.toString(16).padStart(6, '0')}"></span>${name}</label>`).join('');
  $('filters').onchange = e => {
    e.target.checked ? state.hidden.delete(e.target.dataset.g) : state.hidden.add(e.target.dataset.g);
    buildBoxes(); renderList(); drawOverlay();
  };
}

// ---- camera view -------------------------------------------------------------
const FOUR = ['CAM_FRONT_LEFT', 'CAM_FRONT', 'CAM_FRONT_RIGHT', 'CAM_BACK'];
const TWO = ['CAM_FRONT', 'CAM_BACK'];
const cell = c => document.querySelector(`.cell[data-c="${c}"]`);
const curToken = () => { const s = state.samples[+$('frame').value]; return s && s.token; };
function visibleCams() {
  return state.camLayout === 1 ? [state.cam] : state.camLayout === 2 ? TWO.filter(c => state.cams.includes(c)) : state.camLayout === 4 ? FOUR.filter(c => state.cams.includes(c)) : state.cams;
}
function ensureGrid() {                       // rebuild the cells only when the camera set changes
  const cams = visibleCams(), sig = state.camLayout + cams.join(), g = $('camGrid');
  if (g.dataset.sig === sig) return;
  g.dataset.sig = sig; g.className = 'g' + state.camLayout; state.camData = {};
  g.innerHTML = cams.map(c => `<div class="cell" data-c="${c}"><img alt="${CAMN[c] || c} camera"><canvas></canvas><span class="cap">${CAMN[c] || c}</span></div>`).join('');
  $('cam').classList.toggle('multi', state.camLayout > 1);
  $('camTabs').hidden = state.camLayout !== 1;
  document.querySelectorAll('#layoutSeg button').forEach(b => b.classList.toggle('on', +b.dataset.l === state.camLayout));
}
function drawCell(c) {
  const el = cell(c), d = state.camData[c]; if (!el) return;
  const cv = el.querySelector('canvas'), ctx = cv.getContext('2d');
  ctx.clearRect(0, 0, cv.width, cv.height);
  if (!d) return;
  if ($('camLidar').checked && d.pts) {        // LiDAR points, coloured by depth (near = red, far = blue)
    for (let i = 0; i < d.pts.length; i += 3) {
      ctx.fillStyle = `hsl(${Math.round((1 - Math.min(d.pts[i + 2] / 60, 1)) * 245)},85%,55%)`;
      ctx.fillRect(d.pts[i] - 2, d.pts[i + 1] - 2, 4, 4);
    }
  }
  if (!$('camBoxes').checked) return;
  d.proj.boxes.forEach(b => {
    const g = groupOf(b.category); if (!shown(b)) return;
    const sel = state.selected === b.id;
    ctx.strokeStyle = sel ? '#ffffff' : '#' + GROUPS[g][1].toString(16).padStart(6, '0');
    ctx.lineWidth = sel ? 5 : 2.5;
    ctx.beginPath();
    for (let k = 0; k < EDGES.length; k += 2) {
      const p = b.pts[EDGES[k]], q = b.pts[EDGES[k + 1]];
      if (p && q) { ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); }
    }
    ctx.stroke();
  });
}
function drawOverlay() { if (state.cams.length) visibleCams().forEach(drawCell); }
async function loadCamera(token, id) {
  if (!token || !state.cams.length || !camOn()) return;
  ensureGrid();
  await Promise.all(visibleCams().map(async c => {
    const el = cell(c); if (!el) return;
    const im = el.querySelector('img'); im.onerror = () => { el.querySelector('.cap').textContent = (CAMN[c] || c) + ': image not found in samples/' + c; };
    im.src = `/api/samples/${token}/image/${c}`;
    try {
      const [p, buf] = await Promise.all([
        api(`/samples/${token}/projection/${c}`).then(r => r.json()),
        $('camLidar').checked ? api(`/samples/${token}/camera_points/${c}`).then(r => r.arrayBuffer()) : null]);
      if (id !== state.loadId) return;
      state.camData[c] = { proj: p, pts: buf ? new Float32Array(buf) : null };
      const cv = el.querySelector('canvas'); cv.width = p.width; cv.height = p.height;
    } catch (e) { state.camData[c] = null; }
    drawCell(c);
  }));
}
function buildCamTabs() {
  $('camTabs').innerHTML = state.cams.map(c => `<button data-c="${c}" class="${c === state.cam ? 'on' : ''}">${CAMN[c] || c}</button>`).join('');
}
$('camTabs').onclick = e => {
  const c = e.target.dataset.c; if (!c) return;
  state.cam = c; buildCamTabs(); loadCamera(curToken(), state.loadId);
};
$('layoutSeg').onclick = e => {
  const l = +e.target.dataset.l; if (!l) return;
  state.camLayout = l; loadCamera(curToken(), state.loadId);
};
const camOn = () => state.mode !== 'lidar';
function setMode(m) {
  if (m !== 'lidar' && !state.cams.length) m = 'lidar';
  state.mode = m; $('stage').className = 'mode-' + m;
  document.querySelectorAll('#modes [data-m]').forEach(b => b.classList.toggle('on', b.dataset.m === m));
  if (camOn() && state.samples.length) loadCamera(state.samples[+$('frame').value].token, state.loadId);
}
function togglePanel() { document.body.classList.toggle('nosidebar'); $('hidePanel').textContent = document.body.classList.contains('nosidebar') ? 'Show panel' : 'Hide panel'; }
function toggleFullscreen() { document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen(); }
$('modes').onclick = e => {
  if (e.target.dataset.m) setMode(e.target.dataset.m);
  else if (e.target.id === 'hidePanel') togglePanel();
  else if (e.target.id === 'fs') toggleFullscreen();
};
addEventListener('keydown', e => {
  if (['SELECT', 'TEXTAREA'].includes(e.target.tagName) || e.ctrlKey || e.metaKey || e.altKey) return;
  const m = { 1: 'lidar', 2: 'camera', 3: 'split', 4: 'inset' }[e.key];
  if (m) setMode(m); else if (e.key === 'h') togglePanel(); else if (e.key === 'f') toggleFullscreen();
});
$('camBoxes').onchange = drawOverlay;
$('camLidar').onchange = () => loadCamera(state.samples[+$('frame').value].token, state.loadId);
$('onlyFlagged').onchange = () => { buildBoxes(); renderList(); drawOverlay(); };

// ---- data loading ------------------------------------------------------------
async function loadFrame(i) {
  const s = state.samples[i]; if (!s) return;
  const id = ++state.loadId;
  loadCamera(s.token, id);
  $('frameLabel').textContent = `${i + 1} / ${state.samples.length}`;
  try {
    const [pr, br, cr] = await Promise.all([
      api(`/samples/${s.token}/points?max_range=${$('range').value}`), api(`/samples/${s.token}/boxes`), api(`/samples/${s.token}/checks`)]);
    const [buf, boxes, flags] = await Promise.all([pr.arrayBuffer(), br.json(), cr.json()]);
    if (id !== state.loadId) return;                       // a newer frame was requested
    const n = setCloud(buf);
    state.boxes = boxes;
    state.flags = Object.fromEntries(flags.map(f => [f.id, f]));
    $('flagSummary').textContent = flags.length ? `⚠ ${flags.length} of ${boxes.length} objects flagged in this frame` : 'No annotation issues found in this frame';
    if (!boxes.some(b => b.id === state.selected)) state.selected = null;
    buildBoxes(); renderList(); renderDetail();
    $('stats').textContent = `${n.toLocaleString()} of ${(+pr.headers.get('X-Total-Points')).toLocaleString()} points · ${boxes.length} objects`;
  } catch (e) { $('stats').textContent = 'Could not load this frame: ' + e.message; }
}
async function loadScene(token) {
  state.samples = await (await api(`/scenes/${token}/samples`)).json();
  state.cams = await (await api(`/samples/${state.samples[0].token}/cameras`)).json();
  if (!state.cams.includes(state.cam)) state.cam = state.cams[0];
  $('camSec').hidden = !state.cams.length; $('cam').hidden = !state.cams.length; buildCamTabs();
  document.querySelectorAll('#modes [data-m]').forEach(b => b.disabled = !state.cams.length && b.dataset.m !== 'lidar');
  setMode(state.cams.length ? state.mode : 'lidar');
  $('frame').max = state.samples.length - 1; $('frame').value = 0;
  state.selected = null; loadFrame(0);
}
function togglePlay() {
  if (state.timer) { clearInterval(state.timer); state.timer = null; $('play').textContent = 'Play'; return; }
  $('play').textContent = 'Pause';
  state.timer = setInterval(() => {
    const f = $('frame'); f.value = (+f.value + 1) % state.samples.length; loadFrame(+f.value);
  }, 500);
}

$('play').onclick = togglePlay;
$('frame').oninput = e => loadFrame(+e.target.value);
$('scene').onchange = e => loadScene(e.target.value);
$('color').onchange = colorize;
$('psize').oninput = e => { cloudMat.size = +e.target.value; $('psizeV').textContent = e.target.value; };
let rt; $('range').oninput = e => { $('rangeV').textContent = e.target.value + ' m'; clearTimeout(rt); rt = setTimeout(() => loadFrame(+$('frame').value), 250); };

(async function init() {
  buildFilters();
  try {
    const info = await (await api('/info')).json();
    $('mode').textContent = info.mode === 'demo' ? 'Demo mode · synthetic data (nuScenes not found)' : 'nuScenes Mini · LIDAR_TOP';
    const scenes = await (await api('/scenes')).json();
    $('scene').innerHTML = scenes.map(s => `<option value="${s.token}" title="${s.description}">${s.name}</option>`).join('');
    const upd = () => $('desc').textContent = (scenes.find(s => s.token === $('scene').value) || {}).description || '';
    $('scene').addEventListener('change', upd); upd();
    loadScene(scenes[0].token);
  } catch (e) { $('mode').textContent = 'Backend not reachable. Start it with: uvicorn backend.app:app'; }
})();
