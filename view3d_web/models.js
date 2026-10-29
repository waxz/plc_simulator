// models.js - detailed 3D models for the viewer: operator control consoles,
// EUR pallets and 6-axis industrial robots (FANUC style). Plain three.js
// meshes; viewer.js places them and drives their live parts.
import * as THREE from "three";

const deg = (d) => (d * Math.PI) / 180;
const materials = new Map();
export function mat(color, extra = {}) {
  const key = `${color}|${JSON.stringify(extra)}`;
  if (!materials.has(key)) materials.set(key, new THREE.MeshStandardMaterial({ color, roughness: 0.6, ...extra }));
  return materials.get(key);
}
function mesh(geometry, material, shadow = true) {
  const m = new THREE.Mesh(geometry, material);
  m.castShadow = m.receiveShadow = shadow;
  return m;
}
// A box from its centre and size.
function cuboid(w, l, h, material, x = 0, y = 0, z = 0) {
  const m = mesh(new THREE.BoxGeometry(w, l, h), material);
  m.position.set(x, y, z);
  return m;
}
// A cylinder along local z (three.js builds them along y).
function cylZ(r, h, material, segments = 32, rTop = r) {
  return mesh(new THREE.CylinderGeometry(rTop, r, h, segments).rotateX(Math.PI / 2), material);
}
// A cylinder along local y.
function cylY(r, h, material, segments = 32) {
  return mesh(new THREE.CylinderGeometry(r, r, h, segments), material);
}
// A cylinder along local x.
function cylX(r, h, material, segments = 32) {
  return mesh(new THREE.CylinderGeometry(r, r, h, segments).rotateZ(Math.PI / 2), material);
}

function textPlate(text, w, h, { fg = "#ffffff", bg = "#16191e", bold = false } = {}) {
  const canvas = document.createElement("canvas");
  canvas.width = 256;
  canvas.height = Math.max(32, Math.round((256 * h) / w));
  const g = canvas.getContext("2d");
  g.fillStyle = bg;
  g.fillRect(0, 0, canvas.width, canvas.height);
  g.fillStyle = fg;
  let size = Math.round(canvas.height * 0.62);
  g.font = `${bold ? "600 " : ""}${size}px Segoe UI, Arial, sans-serif`;
  while (g.measureText(text).width > canvas.width * 0.92 && size > 8) {
    size -= 1;
    g.font = `${bold ? "600 " : ""}${size}px Segoe UI, Arial, sans-serif`;
  }
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText(text, canvas.width / 2, canvas.height / 2 + 1);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  const plate = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.5 }));
  return plate;
}

// -- control console ----------------------------------------------------------------------------
//
// A floor-standing operator console (RAL 7035 light grey) with a sloped desk:
// pilot lamps on the top row, pushbuttons below, selectors / emergency stop
// at the bottom, each with a legend plate. Local +y faces the operator.

const CONSOLE = { body: 0xc5c9cc, desk: 0x2c3036, bezel: 0x1a1d21, chrome: 0xb8bec5, estopRing: 0xf2c200 };
const PITCH = { col: 0.105, row: 0.135 };

export function buildConsole(panel) {
  const g = new THREE.Group();
  g.position.set(panel.x, panel.y, 0);
  g.rotation.z = deg(panel.yaw - 90);          // local +y -> the facing direction
  const cols = Math.max(2, panel.cols), rows = Math.max(1, panel.rows);
  const W = cols * PITCH.col + 0.14, D = rows * PITCH.row + 0.12;
  const body = mat(CONSOLE.body, { roughness: 0.55 });
  // Pedestal with a plinth and a cable door line.
  g.add(cuboid(W - 0.06, 0.26, 0.06, mat(0x3a3f45), 0, -0.02, 0.03));
  g.add(cuboid(W - 0.08, 0.24, 0.84, body, 0, -0.02, 0.48));
  g.add(cuboid(W - 0.12, 0.004, 0.62, mat(0xb5b9bc), 0, 0.101, 0.46));
  // Desk housing: a wedge below the sloped face.
  const tilt = deg(18);
  const desk = new THREE.Group();
  desk.position.set(0, 0, 0.98);
  desk.rotation.x = -tilt;                       // the front (operator side) lower
  g.add(desk);
  desk.add(cuboid(W, D, 0.07, body, 0, 0, -0.035));
  desk.add(cuboid(W - 0.03, D - 0.03, 0.004, mat(CONSOLE.desk, { roughness: 0.4 }), 0, 0, 0.002));
  g.add(cuboid(W, 0.05, 0.14, body, 0, -D / 2 * Math.cos(tilt) + 0.025, 0.98 + (D / 2) * Math.sin(tilt) - 0.07));
  // Title plate along the back edge.
  const title = textPlate(panel.title || panel.id, Math.min(W - 0.04, 0.32), 0.035, { bold: true, bg: "#d9dcdf", fg: "#1c2024" });
  title.position.set(0, -D / 2 + 0.03, 0.005);
  title.rotation.z = Math.PI;                    // read from the operator's side (+y)
  desk.add(title);
  const devices = new Map();
  for (const d of panel.devices) {
    const x = ((d.cols - 1) / 2 - d.col) * PITCH.col;     // the first on the operator's left
    const y = -D / 2 + 0.085 + d.row * PITCH.row;
    const unit = buildDevice(d);
    unit.group.position.set(x, y, 0.004);
    desk.add(unit.group);
    const legend = textPlate(d.label, 0.088, 0.02);
    legend.position.set(x, y + 0.048, 0.005);
    legend.rotation.z = Math.PI;
    desk.add(legend);
    devices.set(d.id, unit);
  }
  return { group: g, devices };
}

function buildDevice(d) {
  const group = new THREE.Group();
  const bezel = mat(CONSOLE.bezel, { roughness: 0.35, metalness: 0.3 });
  const color = new THREE.Color(d.color);
  const unit = { group, kind: d.kind, style: d.style, id: d.id };
  if (d.style === "lamp") {
    group.add(cylZ(0.028, 0.012, bezel).translateZ(0.006));
    const lens = mesh(new THREE.SphereGeometry(0.02, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2).rotateX(Math.PI / 2),
                      new THREE.MeshStandardMaterial({ color: color.clone().multiplyScalar(0.45), emissive: color,
                                                       emissiveIntensity: 0, roughness: 0.25, transparent: true, opacity: 0.95 }));
    lens.position.z = 0.012;
    group.add(lens);
    unit.lens = lens;
  } else if (d.style === "estop") {
    group.add(cylZ(0.048, 0.004, mat(CONSOLE.estopRing, { roughness: 0.5 })).translateZ(0.002));
    group.add(cylZ(0.016, 0.03, bezel).translateZ(0.019));
    const head = mesh(new THREE.SphereGeometry(0.034, 28, 14, 0, Math.PI * 2, 0, Math.PI / 2).rotateX(Math.PI / 2),
                      mat(0xd61f1f, { roughness: 0.35 }));
    head.position.z = 0.032;
    group.add(head);
    unit.cap = head;
    unit.rest = 0.032;
    unit.travel = 0.012;
  } else if (d.style === "selector") {
    group.add(cylZ(0.028, 0.012, bezel).translateZ(0.006));
    const knob = cuboid(0.012, 0.045, 0.018, mat(0x202328), 0, 0, 0.021);
    group.add(knob);
    unit.knob = knob;
  } else {
    group.add(cylZ(0.028, 0.012, mat(CONSOLE.chrome, { metalness: 0.7, roughness: 0.3 })).translateZ(0.006));
    const cap = cylZ(0.021, 0.014, new THREE.MeshStandardMaterial({ color, roughness: 0.35, emissive: color, emissiveIntensity: 0 }));
    cap.position.z = 0.017;
    group.add(cap);
    unit.cap = cap;
    unit.rest = 0.017;
    unit.travel = 0.006;
  }
  // The whole operator device (cap, ring, knob) presses it; lamps only show.
  group.traverse((o) => {
    o.userData.label = `${d.label} (${d.kind === "led" ? "lamp" : d.style})${d.kind === "led" ? "" : " - click to press"}`;
    if (d.kind !== "led") o.userData.button = d.id;
  });
  return unit;
}

export function setDevice(unit, on) {
  if (unit.lens) unit.lens.material.emissiveIntensity = on ? 2.4 : 0;
  if (unit.cap) {
    unit.cap.position.z = unit.rest - (on ? unit.travel : 0);
    if (unit.style === "button") unit.cap.material.emissiveIntensity = on ? 0.35 : 0;
  }
  if (unit.knob) unit.knob.rotation.z = on ? deg(45) : deg(-45);
}

// -- EUR pallet (EN 13698-1, 1200 x 800 x 144 mm) --------------------------------------------------
//
// Built at unit size (x: -0.5..0.5 along the deck boards, y: -0.5..0.5,
// z: 0..1); the caller scales it to the pallet's length, width and height.

const MM = { L: 1200, W: 800, H: 144 };
function pBox(group, material, x0, x1, y0, y1, z0, z1) {
  const m = mesh(new THREE.BoxGeometry((x1 - x0) / MM.L, (y1 - y0) / MM.W, (z1 - z0) / MM.H), material);
  m.position.set(((x0 + x1) / 2) / MM.L - 0.5, ((y0 + y1) / 2) / MM.W - 0.5, ((z0 + z1) / 2) / MM.H);
  group.add(m);
}

export function buildPallet(color) {
  const base = new THREE.Color(color || "#c8a26a");
  const wood = (k) => mat(base.clone().lerp(new THREE.Color("#d9b98a"), 0.55).multiplyScalar(k).getHex(), { roughness: 0.9 });
  const block = mat(base.clone().lerp(new THREE.Color("#a7814f"), 0.5).getHex(), { roughness: 0.95 });
  const g = new THREE.Group();
  const lanes = [[0, 100], [350, 450], [700, 800]];             // the three block / bottom-board lanes
  const posts = [[0, 145], [527.5, 672.5], [1055, 1200]];       // blocks along the length
  // Bottom boards (3, full length), blocks (9), cross boards (3), top deck (5).
  lanes.forEach(([y0, y1], i) => pBox(g, wood(0.92 + 0.03 * i), 0, 1200, y0, y1, 0, 22));
  for (const [x0, x1] of posts) for (const [y0, y1] of lanes) pBox(g, block, x0, x1, y0, y1, 22, 100);
  posts.forEach(([x0, x1], i) => pBox(g, wood(0.95 + 0.02 * i), x0, x1, 0, 800, 100, 122));
  // Top deck: 145 / 100 / 145 / 100 / 145 mm boards, 41.25 mm gaps.
  const boards = [[0, 145], [186.25, 286.25], [327.5, 472.5], [513.75, 613.75], [655, 800]];
  boards.forEach(([y0, y1], i) => pBox(g, wood(1.0 - 0.025 * (i % 2)), 0, 1200, y0, y1, 122, 144));
  return g;
}

// -- 6-axis robot (FANUC M-10iA / M-20iA style) ------------------------------------------------------
//
// The same chain as robot6.forward(): J1 about z at the pedestal top, the
// shoulder (a1, 0, d1), J2 about y with the upper arm along z, J3 about y
// by (J3 - J2) with the forearm along x (offset a3 up), J4 about x, J5 about
// y at the wrist (d4 along), J6 about x at the flange (d6 along), the
// gripper along x to the TCP.

const FANUC = { yellow: 0xf2c200, motor: 0x2a2e33, steel: 0xaeb5bc, cabinet: 0x3d434a };

export function buildRobot6(r) {
  const G = r.geometry, R = G.reach;
  const yellow = mat(FANUC.yellow, { roughness: 0.45, metalness: 0.1 });
  const motor = mat(FANUC.motor, { roughness: 0.5, metalness: 0.4 });
  const steel = mat(FANUC.steel, { roughness: 0.35, metalness: 0.8 });
  const root = new THREE.Group();
  root.position.set(r.x, r.y, r.z);
  // Pedestal and base.
  root.add(cylZ(0.13 * R, G.mount, mat(0x4a5058, { roughness: 0.7 })).translateZ(G.mount / 2));
  root.add(cylZ(0.15 * R, 0.02, mat(0x4a5058)).translateZ(0.01));
  const j1 = new THREE.Group();
  j1.position.z = G.mount;
  root.add(j1);
  const turretH = G.d1 - G.mount;
  j1.add(cylZ(0.12 * R, turretH * 0.45, yellow, 40).translateZ(turretH * 0.225));
  j1.add(cuboid(0.2 * R, 0.17 * R, turretH * 0.7, yellow, G.a1 * 0.6, 0, turretH * 0.6));
  j1.add(cylZ(0.04 * R, 0.08 * R, motor).translateX(-0.06 * R).translateZ(turretH * 0.45));   // J1 motor
  // J2: shoulder and upper arm.
  const j2 = new THREE.Group();
  j2.position.set(G.a1, 0, turretH);
  j1.add(j2);
  j2.add(cylY(0.075 * R, 0.2 * R, yellow, 40));
  const j2Motor = cylY(0.045 * R, 0.08 * R, motor);
  j2Motor.position.y = -0.14 * R;
  j2.add(j2Motor);
  j2.add(cuboid(0.1 * R, 0.09 * R, G.a2, yellow, 0, 0.02 * R, G.a2 / 2));
  // J3: elbow and forearm housing.
  const j3 = new THREE.Group();
  j3.position.z = G.a2;
  j2.add(j3);
  j3.add(cylY(0.065 * R, 0.17 * R, yellow, 40));
  j3.add(cuboid(0.16 * R, 0.13 * R, 0.1 * R, yellow, -0.02 * R, 0, G.a3 * 0.6));
  const j3Motor = cylX(0.035 * R, 0.1 * R, motor);
  j3Motor.position.set(-0.12 * R, 0, G.a3);
  j3.add(j3Motor);
  // J4: forearm roll; the forearm tube to the wrist.
  const j4 = new THREE.Group();
  j4.position.set(0, 0, G.a3);
  j3.add(j4);
  const tube = cylX(0.038 * R, G.d4 * 0.82, yellow, 32);
  tube.position.x = G.d4 * 0.45;
  j4.add(tube);
  // J5: wrist bend.
  const j5 = new THREE.Group();
  j5.position.x = G.d4;
  j4.add(j5);
  j5.add(cylY(0.032 * R, 0.085 * R, yellow, 32));
  // J6: flange and the gripper.
  const j6 = new THREE.Group();
  j6.position.x = G.d6;
  j5.add(j6);
  j6.add(cylX(0.022 * R, 0.02, steel).translateX(-0.01));
  const T = G.tool;
  j6.add(cuboid(T * 0.45, 0.12, 0.07, mat(0x30353b, { metalness: 0.5, roughness: 0.4 }), T * 0.25, 0, 0));
  const fingers = [cuboid(T * 0.55, 0.018, 0.05, steel, T * 0.72, 0, 0), cuboid(T * 0.55, 0.018, 0.05, steel, T * 0.72, 0, 0)];
  fingers.forEach((f) => j6.add(f));
  // Status beacon on the pedestal: green running, red on a fault.
  const beacon = new THREE.Group();
  beacon.position.set(-0.13 * R, 0, G.mount);
  beacon.add(cylZ(0.012, 0.06, motor).translateZ(0.03));
  const lampOk = cylZ(0.022, 0.035, new THREE.MeshStandardMaterial({ color: 0x1f8f45, emissive: 0x2bd36a, emissiveIntensity: 1.2 }));
  lampOk.position.z = 0.078;
  const lampFault = cylZ(0.022, 0.035, new THREE.MeshStandardMaterial({ color: 0x8f1f1f, emissive: 0xff3b30, emissiveIntensity: 0 }));
  lampFault.position.z = 0.114;
  beacon.add(lampOk, lampFault);
  root.add(beacon);
  const group = new THREE.Group();
  group.add(root);
  group.traverse((o) => { o.userData.label = `Robot ${r.label || r.id} (6-axis)`; });
  return { group, joints: [j1, j2, j3, j4, j5, j6], fingers, lampOk, lampFault, tool: T };
}

export function setRobot6(robot, [a1, a2, a3, a4, a5, a6, holding, fault]) {
  const [j1, j2, j3, j4, j5, j6] = robot.joints;
  j1.rotation.z = deg(a1);
  j2.rotation.y = deg(a2);
  j3.rotation.y = deg(a3 - a2);
  j4.rotation.x = deg(a4);
  j5.rotation.y = deg(a5);
  j6.rotation.x = deg(a6);
  const open = holding ? 0.035 : 0.075;       // jaws along the flange's y
  robot.fingers[0].position.y = -open;
  robot.fingers[1].position.y = open;
  robot.lampFault.material.emissiveIntensity = fault ? 2.2 : 0;
  robot.lampOk.material.emissiveIntensity = fault ? 0 : 1.2;
}

// -- roller conveyors and right-angle transfers --------------------------------------------------

const ROLLER_R = 0.03;

// A roller conveyor (local x along it, the group at the roller tops): side
// frames and one instanced mesh of rollers, turned by turnRollers().
export function buildRollerConveyor(r, color) {
  const g = new THREE.Group();
  g.position.set(r.x, r.y, 0);
  g.rotation.z = deg(r.rot);
  const frame = mat(new THREE.Color(color).getHex(), { metalness: 0.4, roughness: 0.5 });
  for (const y of [-r.width / 2 - 0.03, r.width / 2 + 0.03]) {
    g.add(cuboid(r.length, 0.05, 0.12, frame, 0, y, -0.07));
  }
  const n = Math.max(1, Math.floor(r.length / r.pitch));
  const geo = new THREE.CylinderGeometry(ROLLER_R, ROLLER_R, r.width, 14);       // along local y
  const rollers = new THREE.InstancedMesh(geo, mat(0xb8bec5, { metalness: 0.75, roughness: 0.3 }), n);
  rollers.castShadow = rollers.receiveShadow = true;
  const xs = [];
  for (let i = 0; i < n; i++) xs.push(-r.length / 2 + (i + 0.5) * (r.length / n));
  const unit = { group: g, rollers, xs, angle: 0 };
  g.add(rollers);
  turnRollers(unit, 0);
  return unit;
}

// A curved roller conveyor (the group at the arc's centre and roller tops):
// arc side frames and tapered rollers along the radii.
export function buildRollerArc(r, color) {
  const g = new THREE.Group();
  g.position.set(r.x, r.y, 0);
  const frame = mat(new THREE.Color(color).getHex(), { metalness: 0.4, roughness: 0.5 });
  const a0 = deg(r.start), a1 = deg(r.start + r.sweep);
  for (const rad of [r.radius - r.width / 2 - 0.03, r.radius + r.width / 2 + 0.03]) {
    const shape = new THREE.Shape();
    shape.absarc(0, 0, rad + 0.025, a0, a1, false);
    shape.absarc(0, 0, rad - 0.025, a1, a0, true);
    const rail = mesh(new THREE.ExtrudeGeometry(shape, { depth: 0.12, bevelEnabled: false, curveSegments: 48 }), frame);
    rail.position.z = -0.13;
    g.add(rail);
  }
  const n = Math.max(1, Math.floor(deg(r.sweep) * r.radius / r.pitch));
  const inner = r.radius - r.width / 2, outer = r.radius + r.width / 2;
  // Tapered: thicker at the outside (local +y = outward).
  const geo = new THREE.CylinderGeometry(ROLLER_R * outer / r.radius, ROLLER_R * inner / r.radius, r.width, 14);
  const rollers = new THREE.InstancedMesh(geo, mat(0xb8bec5, { metalness: 0.75, roughness: 0.3 }), n);
  rollers.castShadow = rollers.receiveShadow = true;
  const angles = [];
  for (let i = 0; i < n; i++) angles.push(a0 + (a1 - a0) * (i + 0.5) / n);
  const unit = { group: g, rollers, angles, radius: r.radius, spin: 0, ccw: r.ccw, arc: true };
  g.add(rollers);
  turnRollers(unit, 0);
  return unit;
}

const _m = new THREE.Matrix4(), _q = new THREE.Quaternion(), _p = new THREE.Vector3(), _s = new THREE.Vector3(1, 1, 1);
const _q2 = new THREE.Quaternion(), _zAxis = new THREE.Vector3(0, 0, 1);
const _yAxis = new THREE.Vector3(0, 1, 0);
export function turnRollers(unit, distance) {
  if (unit.arc) {
    unit.spin += (unit.ccw ? 1 : -1) * distance / ROLLER_R;
    unit.angles.forEach((a, i) => {
      _p.set(Math.cos(a) * unit.radius, Math.sin(a) * unit.radius, -ROLLER_R);
      _q.setFromAxisAngle(_zAxis, a - Math.PI / 2);         // the cylinder's axis (y) along the radius
      _q2.setFromAxisAngle(_yAxis, unit.spin);
      unit.rollers.setMatrixAt(i, _m.compose(_p, _q.multiply(_q2), _s));
    });
    unit.rollers.instanceMatrix.needsUpdate = true;
    return;
  }
  unit.angle -= distance / ROLLER_R;                   // rolling the load forward (+x)
  _q.setFromAxisAngle(_yAxis, -unit.angle);
  unit.xs.forEach((x, i) => {
    _p.set(x, 0, -ROLLER_R);
    unit.rollers.setMatrixAt(i, _m.compose(_p, _q, _s));
  });
  unit.rollers.instanceMatrix.needsUpdate = true;
}

// A right-angle transfer: a drive box under the rollers and chain strands
// across the travel that rise above the rollers when it lifts.
export function buildTransfer(t) {
  const g = new THREE.Group();
  g.position.set(t.x, t.y, t.z);
  g.rotation.z = deg(t.rot);
  g.add(cuboid(t.w * 0.85, t.h * 0.85, 0.12, mat(0x30353b), 0, 0, -0.2));
  const strands = new THREE.Group();
  const chain = mat(0x2a2d31, { metalness: 0.6, roughness: 0.45 });
  for (const k of [-0.3, 0, 0.3]) strands.add(cuboid(0.035, t.h * 0.95, 0.03, chain, k * t.w, 0, 0));
  strands.rotation.z = deg(t.cross - 90);        // strands run in the transfer direction
  g.add(strands);
  g.traverse((o) => { o.userData.label = `${t.label || t.id} (90-degree transfer)`; });
  const unit = { group: g, strands, lift: 0, target: 0 };
  setTransfer(unit, [0, false]);
  return unit;
}

export function setTransfer(unit, [lift]) {
  unit.target = lift;
}

export function stepTransfer(unit, dt) {
  unit.lift += (unit.target - unit.lift) * Math.min(1, dt * 15);
  unit.strands.position.z = -0.05 + 0.065 * unit.lift;      // below the rollers .. 15 mm above them
}

// -- logistics equipment (logistics.py) -------------------------------------------------------------

const STEEL = 0x9aa3ad, DARK = 0x2b3038;

// A scanner: an arch over the belt with a read line; a DWS a tunnel with a scale plate.
export function buildScanner(s) {
  const g = new THREE.Group();
  g.position.set(s.x, s.y, s.z);
  g.rotation.z = deg(s.rot);
  const frame = mat(new THREE.Color(s.color).getHex(), { metalness: 0.3, roughness: 0.4 });
  const half = s.h / 2 + 0.06, top = s.type === "dws" ? 0.85 : 0.65;
  for (const y of [-half, half]) g.add(cuboid(0.06, 0.06, top + 0.1, frame, 0, y, top / 2 - 0.05));
  g.add(cuboid(s.type === "dws" ? Math.max(0.3, s.w) : 0.08, 2 * half + 0.06, 0.08, frame, 0, 0, top));
  if (s.type === "dws") {
    // The tunnel's side panels and the weighing plate in the belt.
    const panel = new THREE.MeshStandardMaterial({ color: 0x9fb4c7, transparent: true, opacity: 0.25, roughness: 0.2 });
    for (const y of [-half, half]) g.add(cuboid(Math.max(0.3, s.w), 0.01, top - 0.1, panel, 0, y, top / 2));
    g.add(cuboid(Math.max(0.3, s.w), s.h * 0.9, 0.01, mat(0x707a85, { metalness: 0.6 }), 0, 0, 0.004));
  }
  const head = cuboid(0.12, 0.16, 0.08, mat(DARK), 0, 0, top - 0.08);
  g.add(head);
  // The read line on the belt: lit while a parcel is in the zone.
  const beam = new THREE.Mesh(new THREE.PlaneGeometry(0.012, s.h),
                              new THREE.MeshBasicMaterial({ color: 0xff2a2a, transparent: true, opacity: 0.35 }));
  beam.position.z = 0.006;
  g.add(beam);
  g.traverse((o) => { o.userData.label = `${s.label || s.id} (${s.type})`; });
  return { group: g, beam };
}

export function setScanner(unit, [present, code, noRead]) {
  unit.beam.material.opacity = present ? 0.95 : 0.35;
  unit.beam.material.color.setHex(noRead ? 0xffb020 : 0xff2a2a);
  void code;
}

// A pusher: a frame beside the belt with a paddle that strokes across; a wheel
// diverter: rows of small wheels in the belt that turn and lift when active.
export function buildDiverter(d) {
  const g = new THREE.Group();
  g.position.set(d.x, d.y, d.z);
  g.rotation.z = deg(d.rot);
  const color = new THREE.Color(d.color).getHex();
  const unit = { group: g, type: d.type, stroke: 0, target: 0, wheels: [] };
  if (d.type === "pusher") {
    const side = d.angle < 0 ? 1 : -1;            // the pusher stands on the side it pushes away from
    const y0 = side * (d.h / 2 + 0.12);
    g.add(cuboid(d.w * 0.9, 0.18, 0.22, mat(DARK), 0, y0 + side * 0.06, 0.11));
    const paddle = new THREE.Group();
    paddle.add(cuboid(d.w * 0.7, 0.03, 0.16, mat(color, { roughness: 0.4 }), 0, 0, 0.1));
    paddle.add(cuboid(0.04, Math.abs(y0) + 0.05, 0.04, mat(STEEL, { metalness: 0.7 }), 0, side * (Math.abs(y0) / 2), 0.1));
    paddle.position.y = y0 - side * 0.02;
    g.add(paddle);
    Object.assign(unit, { paddle, y0, travel: -side * (d.h * 0.85) });
  } else {
    const a = deg(d.angle) / 2;
    for (let i = -2; i <= 2; i++) {
      for (let j = -2; j <= 2; j++) {
        const wheel = cylY(0.035, 0.025, mat(color, { roughness: 0.5 }), 16);
        wheel.position.set(i * d.w / 5.5, j * d.h / 5.5, -0.02);
        wheel.rotation.z = 0;
        wheel.userData.turn = a;
        g.add(wheel);
        unit.wheels.push(wheel);
      }
    }
  }
  g.traverse((o) => { o.userData.label = `${d.label || d.id} (to ${d.destination})`; });
  return unit;
}

export function stepDiverter(unit, dt) {
  unit.stroke += (unit.target - unit.stroke) * Math.min(1, dt * 10);
  if (unit.paddle) unit.paddle.position.y = unit.y0 + unit.travel * unit.stroke;
  for (const w of unit.wheels) {
    w.position.z = -0.02 + 0.03 * unit.stroke;          // pop up through the belt
    w.rotation.z = w.userData.turn * unit.stroke;
  }
}

export function buildTurntableRing(t) {
  const g = new THREE.Group();
  g.position.set(t.x, t.y, 0);
  const color = new THREE.Color(t.color).getHex();
  g.add(cylZ(t.r, 0.06, mat(color, { metalness: 0.4, roughness: 0.4 }), 48).translateZ(t.z - 0.18));
  g.add(cylZ(0.12, t.z - 0.18, mat(DARK), 24).translateZ((t.z - 0.18) / 2));
  g.traverse((o) => { o.userData.label = `${t.label || t.id} (turntable)`; });
  return g;
}

// A photo-eye at an accumulation zone's end: two posts and a beam.
export function buildEye(e) {
  const g = new THREE.Group();
  g.position.set(e.x, e.y, e.z);
  g.rotation.z = deg(e.heading);
  const half = e.w / 2 + 0.04;
  for (const y of [-half, half]) g.add(cuboid(0.03, 0.03, 0.14, mat(DARK), -0.12, y, 0.07));
  const beam = new THREE.Mesh(new THREE.BoxGeometry(0.008, 2 * half, 0.008),
                              new THREE.MeshBasicMaterial({ color: 0x2f6b45 }));
  beam.position.set(-0.12, 0, 0.1);
  g.add(beam);
  return { group: g, beam };
}

// A chute: a sloped slide into a bin whose fill rises with the count.
export function buildChute(c) {
  const g = new THREE.Group();
  g.position.set(c.x, c.y, 0);
  const bin = mat(0x5b6470, { metalness: 0.5, roughness: 0.5 });
  const h = 0.55, t = 0.03;
  g.add(cuboid(c.w, c.h, t, bin, 0, 0, t / 2));
  for (const [x, y, w, l] of [[0, -c.h / 2, c.w, t], [0, c.h / 2, c.w, t], [-c.w / 2, 0, t, c.h], [c.w / 2, 0, t, c.h]]) {
    g.add(cuboid(w, l, h, bin, x, y, h / 2));
  }
  const fill = cuboid(c.w - 0.08, c.h - 0.08, 1, mat(0xc8a26a, { roughness: 0.9 }), 0, 0, 0);
  fill.scale.z = 0.001;
  fill.position.z = t;
  fill.geometry = new THREE.BoxGeometry(1, 1, 1).translate(0, 0, 0.5);
  fill.scale.set(c.w - 0.08, c.h - 0.08, 0.001);
  g.add(fill);
  const lamp = cylZ(0.04, 0.06, new THREE.MeshStandardMaterial({ color: 0x7a1f1f, emissive: 0xff3b30, emissiveIntensity: 0 }));
  lamp.position.set(c.w / 2, c.h / 2, h + 0.03);
  g.add(lamp);
  g.traverse((o) => { o.userData.label = `${c.label || c.id} (chute, ${c.capacity})`; });
  return { group: g, fill, lamp, capacity: c.capacity, height: h - 0.05 };
}

export function setChute(unit, [count, full]) {
  unit.fill.scale.z = Math.max(0.001, unit.height * Math.min(1, count / Math.max(1, unit.capacity)));
  unit.lamp.material.emissiveIntensity = full ? 2.2 : 0;
}

// A guard door: posts and a mesh-panel leaf on a hinge (click to open / close).
export function buildDoor(d) {
  const g = new THREE.Group();
  g.position.set(d.x, d.y, 0);
  g.rotation.z = deg(d.rot);
  const frame = mat(new THREE.Color(d.color).getHex(), { roughness: 0.5 });
  const H = 1.9;
  for (const x of [-d.w / 2 - 0.03, d.w / 2 + 0.03]) g.add(cuboid(0.05, 0.05, H, frame, x, 0, H / 2));
  const hinge = new THREE.Group();
  hinge.position.x = -d.w / 2;
  const leaf = new THREE.Group();
  leaf.add(cuboid(d.w, 0.03, 0.04, frame, d.w / 2, 0, 0.12));
  leaf.add(cuboid(d.w, 0.03, 0.04, frame, d.w / 2, 0, H - 0.08));
  leaf.add(cuboid(0.04, 0.03, H - 0.2, frame, 0.02, 0, H / 2));
  leaf.add(cuboid(0.04, 0.03, H - 0.2, frame, d.w - 0.02, 0, H / 2));
  leaf.add(cuboid(d.w - 0.06, 0.005, H - 0.24,
                  new THREE.MeshStandardMaterial({ color: 0x3a3f45, transparent: true, opacity: 0.45, roughness: 0.8 }),
                  d.w / 2, 0, H / 2));
  const lock = cuboid(0.06, 0.08, 0.1, mat(0x30353b), d.w - 0.05, 0.05, 1.05);
  leaf.add(lock);
  hinge.add(leaf);
  g.add(hinge);
  g.traverse((o) => { o.userData.label = `${d.label || d.id} (guard door) - click to open / close`; o.userData.door = d.id; });
  return { group: g, hinge, lock, open: 0, target: 0 };
}

export function setDoor(unit, [closed, locked]) {
  unit.target = closed ? 0 : 1;
  unit.lock.material = mat(locked ? 0x2bd36a : 0x30353b);
}

export function stepDoor(unit, dt) {
  unit.open += (unit.target - unit.open) * Math.min(1, dt * 6);
  unit.hinge.rotation.z = -deg(100) * unit.open;
}

// A safety relay: a yellow box on a post with a status beacon.
export function buildSafety(s) {
  const g = new THREE.Group();
  g.position.set(s.x, s.y, 0);
  g.add(cuboid(0.06, 0.06, 1.2, mat(DARK), 0, 0, 0.6));
  g.add(cuboid(0.3, 0.16, 0.36, mat(0xeab308, { roughness: 0.5 }), 0, 0, 1.3));
  const beacon = cylZ(0.05, 0.12, new THREE.MeshStandardMaterial({ color: 0x1f8f45, emissive: 0x2bd36a, emissiveIntensity: 1.5 }));
  beacon.position.z = 1.54;
  g.add(beacon);
  g.traverse((o) => { o.userData.label = `${s.label || s.id} (safety relay)`; });
  return { group: g, beacon };
}

export function setSafety(unit, [ok, tripped]) {
  const on = ok ? 0x2bd36a : 0xff3b30;
  unit.beacon.material.color.setHex(ok ? 0x1f8f45 : 0x8f1f1f);
  unit.beacon.material.emissive.setHex(on);
  unit.beacon.material.emissiveIntensity = tripped || ok ? 1.8 : 0.9;
}
