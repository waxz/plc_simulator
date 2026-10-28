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
  if (unit.cap) unit.cap.userData.button = d.id;
  if (unit.knob) unit.knob.userData.button = d.id;
  group.traverse((o) => { o.userData.label = `${d.label} (${d.kind === "led" ? "lamp" : d.style})`; });
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
