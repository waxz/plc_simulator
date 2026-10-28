// viewer.js - the 3D view of the running simulation (served by view3d.py).
//
// World coordinates are the layout's: x, y in metres on the floor, z up.
// GET /scene gives the layout once (scene3d.static_scene: belt outlines to
// extrude, sensors, panels, machines, the DXF floor plan); /events streams
// the moving parts and live states ~30 times a second
// (scene3d.dynamic_state). Moving parts come with a full pose (x, y, z,
// yaw) and size, so whatever lifts or carries them only changes numbers.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const COLORS = {
  bg: 0x1f2329, floor: 0x262b32, grid: 0x30363e, gridMajor: 0x3a414b, drawing: 0x5d6673,
  rubber: 0x3a3f46, chevron: 0x4b525b, metal: 0x9aa3ad, dark: 0x2b3038,
  sensorOff: 0x2f6b45, sensorOn: 0xff3b30, source: 0x4678c8, sink: 0xc8803c,
  pick: 0x40b268, place: 0x3898d6,
};

const container = document.getElementById("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(COLORS.bg);
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.05, 5000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.12;
controls.maxPolarAngle = Math.PI * 0.495;
controls.zoomToCursor = true;             // big floor plans: zoom where you point

scene.add(new THREE.HemisphereLight(0xdfe8f2, 0x30353c, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 1.8);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.bias = -0.0005;
scene.add(sun, sun.target);

// Everything of one layout hangs off `layout`, so a new layout is one remove.
let layout = null;
let sceneInfo = null;
let sceneVersion = -1;
const live = { belts: new Map(), sensors: new Map(), leds: new Map(), buttons: new Map(), stoppers: new Map(),
               robots: new Map(), elevators: new Map(), bodies: new Map(), drawing: null, labels: [] };
const ui = { drawing: true, labels: false };

// -- small helpers ---------------------------------------------------------------------------

const unitBox = new THREE.BoxGeometry(1, 1, 1).translate(0, 0, 0.5);     // bottom at z = 0
const materials = new Map();
function mat(color, extra = {}) {
  const key = `${color}|${JSON.stringify(extra)}`;
  if (!materials.has(key)) materials.set(key, new THREE.MeshStandardMaterial({ color, roughness: 0.75, ...extra }));
  return materials.get(key);
}
function box(w, l, h, material) {
  const m = new THREE.Mesh(unitBox, material);
  m.scale.set(w, l, h);
  m.castShadow = m.receiveShadow = true;
  return m;
}
function post(x, y, z0, z1, size, material) {
  const m = box(size, size, Math.max(0.001, z1 - z0), material);
  m.position.set(x, y, z0);
  return m;
}
const deg = (d) => (d * Math.PI) / 180;
function levelZ(level) { return level * sceneInfo.levelHeight; }
function beltTopZ(level) { return levelZ(level) + sceneInfo.beltTop; }
function tagged(obj, label) { obj.traverse((o) => { o.userData.label = label; }); return obj; }

function label(text, x, y, z) {
  const canvas = document.createElement("canvas");
  const ctx = canvas.getContext("2d");
  ctx.font = "28px Segoe UI, sans-serif";
  canvas.width = Math.ceil(ctx.measureText(text).width) + 16;
  canvas.height = 40;
  ctx.font = "28px Segoe UI, sans-serif";
  ctx.fillStyle = "rgba(22,25,30,0.8)";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#d6dbe1";
  ctx.fillText(text, 8, 29);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
  sprite.scale.set(canvas.width / 120, canvas.height / 120, 1);
  sprite.position.set(x, y, z);
  sprite.renderOrder = 10;
  sprite.visible = ui.labels;
  live.labels.push(sprite);
  return sprite;
}

// The moving belt surface: chevrons pointing along the belt, scrolled by its speed.
const CHEVRON_PERIOD = 0.4;   // m
function chevronTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d");
  g.fillStyle = "#3a3f46";
  g.fillRect(0, 0, 64, 64);
  g.strokeStyle = "#50575f";
  g.lineWidth = 7;
  g.beginPath();
  g.moveTo(16, 6); g.lineTo(40, 32); g.lineTo(16, 58);
  g.stroke();
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

// -- building the layout -----------------------------------------------------------------------

function buildFloor(bounds) {
  const [x0, y0, x1, y1] = bounds;
  const size = Math.max(x1 - x0, y1 - y0) + 30;
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(size, size), mat(COLORS.floor, { roughness: 0.95 }));
  floor.position.set(cx, cy, 0);
  floor.receiveShadow = true;
  layout.add(floor);
  const grid = new THREE.GridHelper(Math.ceil(size), Math.ceil(size), COLORS.gridMajor, COLORS.grid);
  grid.rotation.x = Math.PI / 2;
  grid.position.set(Math.round(cx), Math.round(cy), 0.002);
  layout.add(grid);
}

function buildDrawing(polylines) {
  if (!polylines.length) return;
  let n = 0;
  for (const pl of polylines) n += pl.length / 2 - 1;
  const pos = new Float32Array(n * 6);
  let k = 0;
  for (const pl of polylines) {
    for (let i = 0; i + 3 < pl.length; i += 2) {
      pos[k++] = pl[i]; pos[k++] = pl[i + 1]; pos[k++] = 0.004;
      pos[k++] = pl[i + 2]; pos[k++] = pl[i + 3]; pos[k++] = 0.004;
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  live.drawing = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: COLORS.drawing }));
  live.drawing.visible = ui.drawing;
  layout.add(live.drawing);
}

function shapeOf(outline, holes) {
  const shape = new THREE.Shape(outline.map(([x, y]) => new THREE.Vector2(x, y)));
  for (const hole of holes || []) shape.holes.push(new THREE.Path(hole.map(([x, y]) => new THREE.Vector2(x, y))));
  return shape;
}

function buildBelt(b) {
  const t = sceneInfo.beltThickness;
  const geo = new THREE.ExtrudeGeometry(shapeOf(b.outline, b.holes), { depth: t, bevelEnabled: false });
  let top = mat(COLORS.rubber, { roughness: 0.9 });
  let texture = null;
  if (b.dir !== null && b.dir !== undefined) {
    texture = chevronTexture();
    texture.repeat.set(1 / CHEVRON_PERIOD, 1 / CHEVRON_PERIOD);
    texture.rotation = deg(b.dir);          // u runs along the belt (cap UVs are world metres)
    top = new THREE.MeshStandardMaterial({ map: texture, roughness: 0.9 });
  }
  const mesh = new THREE.Mesh(geo, [top, mat(new THREE.Color(b.color).getHex(), { metalness: 0.2 })]);
  mesh.castShadow = mesh.receiveShadow = true;
  const group = new THREE.Group();
  group.add(mesh);
  mesh.position.z = -t;                      // the group's z is the belt top
  group.position.z = beltTopZ(b.level);
  if (!b.elevator) {
    // Stands at some outline corners, down to the floor.
    const step = Math.max(1, Math.round(b.outline.length / 10));
    for (let i = 0; i < b.outline.length; i += step) {
      const [x, y] = b.outline[i];
      group.add(post(x, y, -group.position.z, -t, 0.05, mat(COLORS.metal, { metalness: 0.5 })));
    }
  }
  layout.add(tagged(group, `Belt ${b.id}`));
  live.belts.set(b.id, { group, texture, dir: b.dir });
  const [x, y] = b.outline[0];
  layout.add(label(b.id, x, y, group.position.z + 0.3));
}

function buildSensor(s) {
  const g = new THREE.Group();
  g.position.set(s.x, s.y, beltTopZ(s.level));
  g.rotation.z = deg(s.rot);
  const postMat = mat(COLORS.dark);
  const across = s.w >= s.h;                 // posts at the ends of the longer side
  const half = (across ? s.w : s.h) / 2 + 0.04;
  const at = (d) => (across ? [d, 0] : [0, d]);
  for (const d of [-half, half]) {
    const [px, py] = at(d);
    g.add(post(px, py, -0.1, 0.35, 0.05, postMat));
  }
  const beams = [];
  if (s.type === "laser_banner") {
    const n = s.beams;
    for (let i = 0; i < n; i++) {
      const off = (i - (n - 1) / 2) * s.pitch;
      const beam = box(across ? 2 * half : 0.008, across ? 0.008 : 2 * half, 0.008,
                       new THREE.MeshBasicMaterial({ color: COLORS.sensorOff }));
      beam.position.set(across ? 0 : off, across ? off : 0, 0.12);
      beam.castShadow = false;
      g.add(beam);
      beams.push(beam);
    }
  } else {
    const beam = box(across ? 2 * half : 0.012, across ? 0.012 : 2 * half, 0.012,
                     new THREE.MeshBasicMaterial({ color: COLORS.sensorOff }));
    beam.position.z = 0.12;
    beam.castShadow = false;
    g.add(beam);
    beams.push(beam);
  }
  layout.add(tagged(g, `Sensor ${s.label || s.id}`));
  live.sensors.set(s.id, beams);
  layout.add(label(s.label || s.id, s.x, s.y, g.position.z + 0.5));
}

function buildZone(z, color, height, floorZ, labelText) {
  const m = box(z.w, z.h, height, mat(color, { transparent: true, opacity: 0.35 }));
  m.position.set(z.x, z.y, floorZ);
  m.rotation.z = deg(z.rot || 0);
  m.castShadow = false;
  layout.add(tagged(m, labelText));
}

function buildButton(b) {
  const g = new THREE.Group();
  g.position.set(b.x, b.y, 0);
  g.rotation.z = deg(b.rot || 0);
  g.add(post(0, 0, 0, 1.0, 0.12, mat(COLORS.dark)));
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.05, 24).rotateX(Math.PI / 2),
                             new THREE.MeshStandardMaterial({ color: b.color, roughness: 0.4 }));
  cap.position.z = 1.025;
  cap.userData.button = b.id;
  g.add(cap);
  layout.add(tagged(g, `Button ${b.label || b.id} - click to press`));
  cap.userData.button = b.id;
  live.buttons.set(b.id, cap);
  layout.add(label(b.label || b.id, b.x, b.y, 1.3));
}

function buildLed(l) {
  const g = new THREE.Group();
  g.position.set(l.x, l.y, 0);
  g.add(post(0, 0, 0, 1.1, 0.08, mat(COLORS.dark)));
  const r = Math.min(0.12, Math.max(0.05, l.r));
  const lamp = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14),
                              new THREE.MeshStandardMaterial({ color: l.color, emissive: l.color, emissiveIntensity: 0 }));
  lamp.position.z = 1.1 + r;
  g.add(lamp);
  layout.add(tagged(g, `Lamp ${l.label || l.id}`));
  live.leds.set(l.id, lamp);
  layout.add(label(l.label || l.id, l.x, l.y, 1.5));
}

function buildStopper(s) {
  const g = new THREE.Group();
  g.position.set(s.x, s.y, beltTopZ(s.level));
  g.rotation.z = deg(s.rot);
  const bar = box(Math.max(0.03, s.d), s.w, 0.1, mat(new THREE.Color(s.color).getHex(), { metalness: 0.3 }));
  g.add(bar);
  g.add(post(0, -s.w / 2 - 0.04, -0.3, 0.05, 0.05, mat(COLORS.dark)));
  g.add(post(0, s.w / 2 + 0.04, -0.3, 0.05, 0.05, mat(COLORS.dark)));
  layout.add(tagged(g, `Stopper ${s.id}`));
  live.stoppers.set(s.id, { bar, up: 1 });
}

function buildRobot(r) {
  const base = levelZ(r.level);
  const color = new THREE.Color(r.color).getHex();
  const g = new THREE.Group();
  const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.3, 0.3, 28).rotateX(Math.PI / 2),
                                  mat(COLORS.dark));
  pedestal.position.set(r.x, r.y, base + 0.15);
  pedestal.castShadow = true;
  const shoulderZ = base + 1.0;
  const column = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.16, 0.7, 24).rotateX(Math.PI / 2), mat(color));
  column.position.set(r.x, r.y, base + 0.65);
  column.castShadow = true;
  const links = [box(1, 0.14, 0.14, mat(color)), box(1, 0.11, 0.11, mat(color))];
  for (const l of links) l.geometry = new THREE.BoxGeometry(1, 1, 1).translate(0.5, 0, 0);
  const wrist = box(0.08, 0.08, 1, mat(COLORS.metal, { metalness: 0.6 }));
  const fingers = [box(0.03, 0.03, 0.12, mat(COLORS.metal)), box(0.03, 0.03, 0.12, mat(COLORS.metal))];
  g.add(pedestal, column, ...links, wrist, ...fingers);
  for (const [p, c] of [[r.pick, COLORS.pick], [r.place, COLORS.place]]) {
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.16, 0.2, 32), new THREE.MeshBasicMaterial({ color: c }));
    ring.position.set(p[0], p[1], p[2] + 0.005);
    g.add(ring);
  }
  layout.add(tagged(g, `Robot ${r.label || r.id}`));
  const L = Math.max(0.6, r.reach * 0.55);
  live.robots.set(r.id, { r, links, wrist, fingers, shoulder: new THREE.Vector3(r.x, r.y, shoulderZ), L });
  layout.add(label(r.label || r.id, r.x, r.y, shoulderZ + 0.8));
}

function buildElevator(e) {
  const g = new THREE.Group();
  g.position.set(e.x, e.y, 0);
  g.rotation.z = deg(e.rot);
  const top = (e.levels - 1) * sceneInfo.levelHeight + sceneInfo.beltTop + 0.6;
  const frame = mat(new THREE.Color(e.color).getHex(), { metalness: 0.4 });
  const hw = e.w / 2 + 0.06, hh = e.h / 2 + 0.06;
  for (const [x, y] of [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]]) g.add(post(x, y, 0, top, 0.07, frame));
  for (const y of [-hh, hh]) {
    const beam = box(2 * hw + 0.07, 0.07, 0.07, frame);
    beam.position.set(0, y, top - 0.07);
    g.add(beam);
  }
  // Landings: an outline at each floor's belt height.
  for (let L = 0; L < e.levels; L++) {
    const z = levelZ(L) + sceneInfo.beltTop;
    const pts = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh], [-hw, -hh]].map(([x, y]) => new THREE.Vector3(x, y, z));
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: e.color })));
  }
  const platform = box(e.w + 0.04, e.h + 0.04, 0.05, mat(COLORS.dark));
  g.add(platform);
  layout.add(tagged(g, `Elevator ${e.label || e.id}`));
  live.elevators.set(e.id, { platform, beltId: e.beltId });
  layout.add(label(e.label || e.id, e.x, e.y, top + 0.3));
}

function buildLayout(info) {
  if (layout) scene.remove(layout);
  for (const key of ["belts", "sensors", "leds", "buttons", "stoppers", "robots", "elevators", "bodies"]) live[key].clear();
  live.labels = [];
  live.drawing = null;
  sceneInfo = info;
  sceneVersion = info.version;
  layout = new THREE.Group();
  scene.add(layout);
  document.getElementById("title").textContent = info.title || "Conveyor 3D View";
  document.title = `${info.title || "Layout"} - 3D View`;
  buildFloor(info.bounds);
  buildDrawing(info.drawing || []);
  info.belts.forEach(buildBelt);
  info.sensors.forEach(buildSensor);
  info.sources.forEach((s) => buildZone(s, COLORS.source, 0.04, beltTopZ(s.level), `Source ${s.label || s.id}`));
  info.sinks.forEach((s) => buildZone(s, COLORS.sink, 0.6, 0, `Sink ${s.label || s.id}`));
  info.buttons.forEach(buildButton);
  info.leds.forEach(buildLed);
  info.stoppers.forEach(buildStopper);
  info.robots.forEach(buildRobot);
  info.elevators.forEach(buildElevator);
  fit();
}

// -- live state -------------------------------------------------------------------------------

const bodyGroup = new THREE.Group();
scene.add(bodyGroup);
let lastState = null;

function updateBodies(rows) {
  const seen = new Set();
  let parcels = 0, pallets = 0;
  for (const [id, kind, x, y, z, yaw, w, l, h, color] of rows) {
    seen.add(id);
    kind === "pallet" ? pallets++ : parcels++;
    let entry = live.bodies.get(id);
    if (!entry) {
      const material = kind === "pallet" ? mat(new THREE.Color(color).getHex(), { roughness: 0.95 })
                                         : mat(new THREE.Color(color).getHex(), { roughness: 0.85 });
      const mesh = box(1, 1, 1, material);
      mesh.userData.label = `${kind === "pallet" ? "Pallet" : "Parcel"} ${id}`;
      bodyGroup.add(mesh);
      entry = { mesh, target: new THREE.Vector3(x, y, z), yaw: deg(yaw) };
      mesh.position.copy(entry.target);
      mesh.rotation.z = entry.yaw;
      live.bodies.set(id, entry);
    }
    entry.target.set(x, y, z);
    entry.yaw = deg(yaw);
    entry.mesh.scale.set(w, l, h);
  }
  for (const [id, entry] of live.bodies) {
    if (!seen.has(id)) {
      bodyGroup.remove(entry.mesh);
      live.bodies.delete(id);
    }
  }
  document.getElementById("parcels").textContent = parcels;
  document.getElementById("pallets").textContent = pallets;
}

function placeBetween(mesh, a, b) {
  const d = new THREE.Vector3().subVectors(b, a);
  mesh.position.copy(a);
  mesh.scale.x = Math.max(0.001, d.length());
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), d.normalize());
}

function updateRobot(entry, [tx, ty, tz, holding, heldH]) {
  const { shoulder: S, L, links, wrist, fingers } = entry;
  const gripZ = tz + (holding ? heldH : 0.3);          // the gripper touches the part's top
  const W = new THREE.Vector3(tx, ty, gripZ + 0.25);    // the wrist above it
  const dx = tx - S.x, dy = ty - S.y;
  const yaw = Math.atan2(dy, dx);
  const horiz = Math.hypot(dx, dy), vert = W.z - S.z;
  const D = Math.min(Math.max(Math.hypot(horiz, vert), 0.05), 2 * L - 1e-3);
  const a = Math.atan2(vert, horiz) + Math.acos(Math.min(1, D / (2 * L)));   // elbow up
  const E = new THREE.Vector3(S.x + Math.cos(yaw) * Math.cos(a) * L, S.y + Math.sin(yaw) * Math.cos(a) * L,
                              S.z + Math.sin(a) * L);
  placeBetween(links[0], S, E);
  placeBetween(links[1], E, W);
  wrist.scale.set(0.08, 0.08, 0.25);
  wrist.position.set(tx, ty, gripZ);
  const open = holding ? 0.12 : 0.18;
  fingers.forEach((f, i) => {
    f.position.set(tx + Math.cos(yaw + Math.PI / 2) * (i ? open : -open), ty + Math.sin(yaw + Math.PI / 2) * (i ? open : -open),
                   gripZ - 0.08);
  });
}

function applyState(state) {
  if (state.layout !== sceneVersion) {
    loadScene();
    return;
  }
  lastState = state;
  document.getElementById("time").textContent = `${state.t.toFixed(1)} s`;
  updateBodies(state.bodies);
  for (const [id, beams] of live.sensors) {
    const v = state.sensors[id];
    beams.forEach((beam, i) => {
      const on = typeof v === "string" ? v[i] === "1" : !!v;
      beam.material.color.setHex(on ? COLORS.sensorOn : COLORS.sensorOff);
    });
  }
  for (const [id, lamp] of live.leds) lamp.material.emissiveIntensity = state.leds[id] ? 2.2 : 0;
  for (const [id, cap] of live.buttons) {
    const pressed = state.buttons[id];
    cap.position.z = pressed ? 1.005 : 1.025;
    cap.material.emissive = cap.material.color;
    cap.material.emissiveIntensity = pressed ? 0.6 : 0;
  }
  for (const [id, s] of live.stoppers) s.target = state.stoppers[id] ? 1 : 0;
  for (const [id, entry] of live.robots) if (state.robots[id]) updateRobot(entry, state.robots[id]);
  for (const [id, e] of live.elevators) {
    const row = state.elevators[id];
    if (row) e.platform.position.z = row[0] + sceneInfo.beltTop - sceneInfo.beltThickness - 0.05;
  }
  for (const [id, b] of live.belts) {
    const row = state.belts[id];
    if (!row) continue;
    b.speed = row[0];
    if (row[1] !== null) b.group.position.z = row[1];
  }
}

// -- connection ---------------------------------------------------------------------------------

async function loadScene() {
  if (loadScene.busy) return;
  loadScene.busy = true;
  const title = document.getElementById("title");
  if (!sceneInfo) title.textContent = "Loading layout...";
  try {
    const info = await (await fetch("scene", { cache: "no-store" })).json();
    if (info.belts) buildLayout(info);
    document.getElementById("error").style.display = "none";
  } catch (err) {
    showError(`Could not load the layout (is the simulator running?): ${err}`);
  } finally {
    loadScene.busy = false;
  }
}

function connect() {
  const link = document.getElementById("link");
  const events = new EventSource("events");
  events.onopen = () => link.classList.add("on");
  events.onerror = () => link.classList.remove("on");     // EventSource reconnects by itself
  events.onmessage = (e) => applyState(JSON.parse(e.data));
}

function showError(text) {
  const el = document.getElementById("error");
  el.textContent = text;
  el.style.display = "block";
}

// -- camera and interaction ------------------------------------------------------------------

function fit(topDown = false) {
  if (!sceneInfo) return;
  const [x0, y0, x1, y1] = sceneInfo.bounds;
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const size = Math.max(x1 - x0, y1 - y0, 4);
  controls.target.set(cx, cy, sceneInfo.beltTop);
  if (topDown) camera.position.set(cx, cy - 0.001, size * 1.25);
  else camera.position.set(cx - size * 0.45, cy - size * 0.85, size * 0.65);
  camera.near = size / 2000;
  camera.far = size * 50;
  camera.updateProjectionMatrix();
  const s = size * 0.75;
  sun.position.set(cx - s * 0.6, cy - s * 0.4, s * 1.2);
  sun.target.position.set(cx, cy, 0);
  Object.assign(sun.shadow.camera, { left: -s, right: s, top: s, bottom: -s, near: 0.1, far: s * 4 });
  sun.shadow.camera.updateProjectionMatrix();
  controls.update();
}

function toggle(button, key, apply) {
  ui[key] = !ui[key];
  button.classList.toggle("on", ui[key]);
  apply(ui[key]);
}
const drawingBtn = document.getElementById("drawing");
const labelsBtn = document.getElementById("labels");
document.getElementById("fit").onclick = () => fit(false);
document.getElementById("top").onclick = () => fit(true);
drawingBtn.onclick = () => toggle(drawingBtn, "drawing", (on) => { if (live.drawing) live.drawing.visible = on; });
labelsBtn.onclick = () => toggle(labelsBtn, "labels", (on) => live.labels.forEach((l) => { l.visible = on; }));
window.addEventListener("keydown", (e) => {
  if (e.key === "f" || e.key === "F") fit(false);
  else if (e.key === "t" || e.key === "T") fit(true);
  else if (e.key === "d" || e.key === "D") drawingBtn.click();
  else if (e.key === "l" || e.key === "L") labelsBtn.click();
});

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const tip = document.getElementById("tip");
function pick(event) {
  pointer.set((event.clientX / window.innerWidth) * 2 - 1, -(event.clientY / window.innerHeight) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects([layout, bodyGroup].filter(Boolean), true);
  return hits.find((h) => h.object.userData.label && h.object.visible);
}
renderer.domElement.addEventListener("pointermove", (e) => {
  const hit = e.buttons ? null : pick(e);
  if (hit) {
    tip.textContent = hit.object.userData.label;
    tip.style.left = `${e.clientX + 14}px`;
    tip.style.top = `${e.clientY + 10}px`;
    tip.style.display = "block";
  } else tip.style.display = "none";
});
let pressed = null;
function sendButton(id, down) {
  fetch("button", { method: "POST", headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ id, pressed: down }) }).catch(() => {});
}
renderer.domElement.addEventListener("pointerdown", (e) => {
  if (e.button !== 0) return;
  const hit = pick(e);
  const id = hit && hit.object.userData.button;
  if (id) {
    pressed = id;
    controls.enabled = false;
    sendButton(id, true);
  }
});
window.addEventListener("pointerup", () => {
  if (pressed) {
    sendButton(pressed, false);
    pressed = null;
    controls.enabled = true;
  }
});
window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// -- frame loop -------------------------------------------------------------------------------

const clock = new THREE.Clock();
function shortestAngle(from, to) {
  return from + (((to - from + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
}
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.1);
  const k = Math.min(1, dt * 18);            // ease toward the newest state between frames
  for (const entry of live.bodies.values()) {
    entry.mesh.position.lerp(entry.target, k);
    entry.mesh.rotation.z = shortestAngle(entry.mesh.rotation.z, entry.yaw) * k + entry.mesh.rotation.z * (1 - k);
  }
  for (const s of live.stoppers.values()) {
    s.up += ((s.target ?? 1) - s.up) * Math.min(1, dt * 12);
    s.bar.position.z = -0.12 + s.up * 0.12;
  }
  for (const b of live.belts.values()) {
    if (b.texture && b.speed) b.texture.offset.x -= (b.speed * dt) / CHEVRON_PERIOD;
  }
  controls.update();
  renderer.render(scene, camera);
}

loadScene().then(connect);
animate();
