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
import {
  buildChute, buildConsole, buildDiverter, buildDoor, buildEye, buildPallet, buildRobot6, buildRollerArc,
  buildRollerConveyor,
  buildSafety, buildScanner, buildTransfer, buildTurntableRing, setChute, setDevice, setDoor, setRobot6, setSafety,
  setDiverter, setScanner, setTransfer, stepDiverter, stepDoor, stepTransfer, turnRollers,
} from "./models.js";

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
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
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
const live = { belts: new Map(), sensors: new Map(), devices: new Map(), stoppers: new Map(), scanners: new Map(),
               transfers: new Map(), rollers: new Map(),
               diverters: new Map(), eyes: new Map(), chutes: new Map(), doors: new Map(), safety: new Map(),
               robots: new Map(), elevators: new Map(), bodies: new Map(), drawing: null, labels: [] };
const ui = { drawing: true, labels: false };

// -- small helpers ---------------------------------------------------------------------------

const unitBox = new THREE.BoxGeometry(1, 1, 1).translate(0, 0, 0.5);     // bottom at z = 0
const noPick = () => {};          // the floor, grid and floor plan: never under the mouse
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
  floor.raycast = noPick;
  layout.add(floor);
  const grid = new THREE.GridHelper(Math.ceil(size), Math.ceil(size), COLORS.gridMajor, COLORS.grid);
  grid.rotation.x = Math.PI / 2;
  grid.position.set(Math.round(cx), Math.round(cy), 0.002);
  grid.raycast = noPick;
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
  live.drawing.raycast = noPick;
  layout.add(live.drawing);
}

function shapeOf(outline, holes) {
  const shape = new THREE.Shape(outline.map(([x, y]) => new THREE.Vector2(x, y)));
  for (const hole of holes || []) shape.holes.push(new THREE.Path(hole.map(([x, y]) => new THREE.Vector2(x, y))));
  return shape;
}

function buildBelt(b) {
  if (b.roller && !b.table && !b.elevator) {
    // A roller conveyor: rollers and side frames, on stands.
    const unit = (b.roller.kind === "arc" ? buildRollerArc : buildRollerConveyor)(b.roller, b.color);
    unit.group.position.z = beltTopZ(b.level);
    const step = Math.max(1, Math.round(b.outline.length / 10));
    const z = unit.group.position.z;
    const stands = new THREE.Group();
    for (let i = 0; i < b.outline.length; i += step) {
      const [x, y] = b.outline[i];
      stands.add(post(x, y, 0, z - 0.12, 0.05, mat(COLORS.metal, { metalness: 0.5 })));
    }
    layout.add(stands);
    layout.add(tagged(unit.group, `Roller conveyor ${b.id}`));
    live.belts.set(b.id, { group: unit.group, texture: null, dir: null, pivot: null, angle0: 0 });
    live.rollers.set(b.id, unit);
    return;
  }
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
  let pivot = null;
  if (b.table) {
    // On a turntable: turned about the table's centre (built at its angle then).
    pivot = new THREE.Group();
    pivot.position.set(b.table.x, b.table.y, 0);
    group.position.x = -b.table.x;
    group.position.y = -b.table.y;
    pivot.add(group);
  }
  if (!b.elevator && !b.table) {
    // Stands at some outline corners, down to the floor.
    const step = Math.max(1, Math.round(b.outline.length / 10));
    for (let i = 0; i < b.outline.length; i += step) {
      const [x, y] = b.outline[i];
      group.add(post(x, y, -group.position.z, -t, 0.05, mat(COLORS.metal, { metalness: 0.5 })));
    }
  }
  layout.add(tagged(pivot || group, `Belt ${b.id}`));
  live.belts.set(b.id, { group, texture, dir: b.dir, pivot, angle0: b.table ? b.table.angle : 0 });
  const [x, y] = b.outline[0];
  layout.add(label(b.id, x, y, group.position.z + 0.3));
}

const BANNER_HEAD = 0.75;      // m above the belt: the laser banner's head, its lasers shining down

// A laser banner: a head across the belt on two posts, a laser every pitch
// along it shining straight down (local y = along the banner, as the physics).
function buildBanner(s) {
  const g = new THREE.Group();
  g.position.set(s.x, s.y, beltTopZ(s.level));
  g.rotation.z = deg(s.rot);
  const span = (s.beams - 1) * s.pitch;
  const half = Math.max(span, s.h) / 2 + 0.06;
  const postMat = mat(COLORS.dark);
  for (const d of [-half, half]) g.add(post(0, d, -0.3, BANNER_HEAD + 0.06, 0.05, postMat));
  const head = box(0.09, 2 * half + 0.05, 0.06, mat(COLORS.dark, { metalness: 0.4 }));
  head.position.z = BANNER_HEAD;
  g.add(head);
  const beams = [];
  for (let i = 0; i < s.beams; i++) {
    const beam = box(0.006, 0.006, BANNER_HEAD, new THREE.MeshBasicMaterial({ color: COLORS.sensorOff }));
    beam.position.set(0, (i - (s.beams - 1) / 2) * s.pitch, 0);
    beam.castShadow = false;
    g.add(beam);
    beams.push(beam);
  }
  layout.add(tagged(g, `Laser banner ${s.label || s.id} (${s.beams} beams down, ${s.pitch} m apart)`));
  live.sensors.set(s.id, beams);
  layout.add(label(s.label || s.id, s.x, s.y, g.position.z + BANNER_HEAD + 0.3));
}

function buildSensor(s) {
  if (s.type === "laser_banner") return buildBanner(s);
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
  const beam = box(across ? 2 * half : 0.012, across ? 0.012 : 2 * half, 0.012,
                   new THREE.MeshBasicMaterial({ color: COLORS.sensorOff }));
  beam.position.z = 0.12;
  beam.castShadow = false;
  g.add(beam);
  beams.push(beam);
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

// Operator buttons and lamps on control consoles (scene3d.control_panels).
function buildPanel(panel) {
  const built = buildConsole(panel);
  layout.add(built.group);
  for (const [id, unit] of built.devices) live.devices.set(id, unit);
  layout.add(label(panel.title || panel.id, panel.x, panel.y, 1.45));
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
  const robot = buildRobot6(r);
  layout.add(robot.group);
  // Taught pick / place points.
  for (const [p, c] of [[r.pick, COLORS.pick], [r.place, COLORS.place]]) {
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.12, 0.15, 40), new THREE.MeshBasicMaterial({ color: c }));
    ring.position.set(p[0], p[1], p[2] + 0.004);
    layout.add(tagged(ring, `${c === COLORS.pick ? "Pick" : "Place"} point of ${r.label || r.id}`));
  }
  live.robots.set(r.id, robot);
  layout.add(label(r.label || r.id, r.x, r.y, r.z + r.geometry.d1 + r.geometry.a2 + 0.4));
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
  for (const key of ["belts", "sensors", "devices", "stoppers", "robots", "elevators", "scanners", "diverters", "eyes",
                     "chutes", "doors", "safety", "transfers", "rollers"]) live[key].clear();
  for (const entry of live.bodies.values()) bodyGroup.remove(entry.mesh);
  live.bodies.clear();
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
  (info.panels || []).forEach(buildPanel);
  info.stoppers.forEach(buildStopper);
  info.robots.forEach(buildRobot);
  info.elevators.forEach(buildElevator);
  // Logistics equipment.
  for (const sc of info.scanners || []) { const u = buildScanner(sc); layout.add(u.group); live.scanners.set(sc.id, u); }
  for (const d of info.diverters || []) { const u = buildDiverter(d); layout.add(u.group); live.diverters.set(d.id, u); }
  for (const t of info.turntables || []) layout.add(buildTurntableRing(t));
  for (const t of info.transfers || []) { const u = buildTransfer(t); layout.add(u.group); live.transfers.set(t.id, u); }
  for (const e of info.eyes || []) {
    const u = buildEye(e);
    layout.add(u.group);
    if (!live.eyes.has(e.acc)) live.eyes.set(e.acc, []);
    live.eyes.get(e.acc)[e.zone] = u;
  }
  for (const c of info.chutes || []) { const u = buildChute(c); layout.add(u.group); live.chutes.set(c.id, u); }
  for (const d of info.doors || []) { const u = buildDoor(d); layout.add(u.group); live.doors.set(d.id, u); }
  for (const sf of info.safety || []) {
    const u = buildSafety(sf);
    layout.add(u.group);
    live.safety.set(sf.id, u);
    layout.add(label(sf.label || sf.id, sf.x, sf.y, 1.85));
  }
  fit();
}

// -- live state -------------------------------------------------------------------------------

const bodyGroup = new THREE.Group();
scene.add(bodyGroup);
let lastState = null;

function updateBodies(rows) {
  const seen = new Set();
  let parcels = 0, pallets = 0;
  for (const [id, kind, x, y, z, yaw, w, l, h, color, barcode, destination] of rows) {
    seen.add(id);
    kind === "pallet" ? pallets++ : parcels++;
    let entry = live.bodies.get(id);
    if (!entry) {
      let mesh;
      if (kind === "pallet") {
        // EUR pallet; its deck boards along the longer side.
        mesh = new THREE.Group();
        const pallet = buildPallet(color);
        if (l > w) pallet.rotation.z = Math.PI / 2;
        mesh.add(pallet);
        tagged(mesh, `Pallet ${id}`);
      } else {
        mesh = box(1, 1, 1, mat(new THREE.Color(color).getHex(), { roughness: 0.85 }));
        mesh.userData.label = barcode ? `Parcel ${barcode}${destination ? ` -> ${destination}` : ""}` : `Parcel ${id}`;
      }
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
  for (const [id, unit] of live.devices) setDevice(unit, unit.kind === "led" ? state.leds[id] : state.buttons[id]);
  for (const [id, s] of live.stoppers) s.target = state.stoppers[id] ? 1 : 0;
  for (const [id, robot] of live.robots) if (state.robots[id]) setRobot6(robot, state.robots[id]);
  for (const [id, e] of live.elevators) {
    const row = state.elevators[id];
    if (row) e.platform.position.z = row[0] + sceneInfo.beltTop - sceneInfo.beltThickness - 0.05;
  }
  for (const [id, u] of live.scanners) if (state.scanners && state.scanners[id]) setScanner(u, state.scanners[id]);
  for (const [id, u] of live.transfers) if (state.transfers && state.transfers[id]) setTransfer(u, state.transfers[id]);
  for (const [id, u] of live.diverters) {
    const s = state.diverters && state.diverters[id];
    if (s) setDiverter(u, s);
  }
  for (const [id, eyes] of live.eyes) {
    const zones = (state.accumulators || {})[id] || 0;
    eyes.forEach((u, i) => { if (u) u.beam.material.color.setHex(zones >> i & 1 ? COLORS.sensorOn : COLORS.sensorOff); });
  }
  for (const [id, u] of live.chutes) if (state.chutes && state.chutes[id]) setChute(u, state.chutes[id]);
  for (const [id, u] of live.doors) if (state.doors && state.doors[id]) setDoor(u, state.doors[id]);
  for (const [id, u] of live.safety) if (state.safety && state.safety[id]) setSafety(u, state.safety[id]);
  for (const [id, b] of live.belts) {
    const row = state.belts[id];
    if (!row) continue;
    b.speed = row[0];
    if (row[1] !== null) b.group.position.z = row[1];
    if (b.pivot && row[2] !== null && row[2] !== undefined) b.pivot.rotation.z = deg(row[2] - b.angle0);
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

// The live stream holds a connection open for as long as it runs, and a
// browser allows only six to one server: with six 3D tabs open, a seventh
// would never load. So only a tab on screen streams; a hidden one lets go
// and picks up again when it is shown.
let events = null;
function connect() {
  if (events || document.hidden) return;
  const link = document.getElementById("link");
  events = new EventSource("events");
  events.onopen = () => link.classList.add("on");
  events.onerror = () => link.classList.remove("on");     // EventSource reconnects by itself
  events.onmessage = (e) => applyState(JSON.parse(e.data));
}

function disconnect() {
  if (!events) return;
  events.close();
  events = null;
  document.getElementById("link").classList.remove("on");
}

document.addEventListener("visibilitychange", () => (document.hidden ? disconnect() : connect()));
window.addEventListener("pagehide", disconnect);

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
  renderer.domElement.style.cursor = hit && (hit.object.userData.button || hit.object.userData.door) ? "pointer" : "";
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
  const door = hit && hit.object.userData.door;
  if (door) {
    fetch("door", { method: "POST", headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ id: door }) }).catch(() => {});
    return;
  }
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
  for (const u of live.diverters.values()) stepDiverter(u, dt);
  for (const u of live.transfers.values()) stepTransfer(u, dt);
  for (const [id, u] of live.rollers) {
    const speed = live.belts.get(id)?.speed;
    if (speed) turnRollers(u, speed * dt);
  }
  for (const u of live.doors.values()) stepDoor(u, dt);
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

// For debugging and automated checks in the browser console.
window.view3d = { THREE, scene, camera, controls, renderer, live, pick: (x, y) => pick({ clientX: x, clientY: y }) };
loadScene().then(connect);
animate();
