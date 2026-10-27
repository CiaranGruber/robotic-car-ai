// Live 3D viewer for the AI4R car simulator (tests/policy/simulator). It draws the world in ROS coordinates:
// metres, +x forwards/east, +y left/north, +z up. The car group is base_link, so the car's camera, lidar points
// and the policy's own drawings (all in base_link) are its children.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const $ = (id) => document.getElementById(id);
const V3 = THREE.Vector3;
const deg = (d) => d * Math.PI / 180;

// ---- Renderer, scene and light -------------------------------------------------------------------------------
const container = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const SKY = new THREE.Color(0xcad6e2);
scene.background = SKY;
scene.fog = new THREE.Fog(SKY, 14, 45);
scene.add(new THREE.HemisphereLight(0xffffff, 0x958f84, 1.15));
const sun = new THREE.DirectionalLight(0xfff4e6, 2.3);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 0.5, far: 40 });
sun.shadow.bias = -0.0004;
scene.add(sun, sun.target);

const floor = new THREE.Mesh(new THREE.PlaneGeometry(400, 400),
  new THREE.MeshStandardMaterial({ color: 0xd8d3c9, roughness: 1 }));
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(400, 400, 0xb1aba0, 0xc4bfb5);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.0008;
scene.add(grid);

const standard = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, ...extra });
const shadowy = (mesh) => { mesh.castShadow = true; mesh.receiveShadow = true; return mesh; };

// ---- Cones -----------------------------------------------------------------------------------------------------
const CONE_H = 0.15, CONE_R = 0.05, MAX_CONES = 4000;
const CONE_COLOURS = { BLUE: new THREE.Color(0x1d5fd8), YELLOW: new THREE.Color(0xf4bf0c),
  TOUCHED: new THREE.Color(0xe03030), UNKNOWN: new THREE.Color(0x8a8a8a) };
const coneGeometry = new THREE.ConeGeometry(CONE_R, CONE_H, 28).rotateX(Math.PI / 2).translate(0, 0, CONE_H / 2);
const bandGeometry = new THREE.CylinderGeometry(0.0212, 0.0282, 0.022, 28, 1, true)
  .rotateX(Math.PI / 2).translate(0, 0, 0.08);
const baseGeometry = new THREE.BoxGeometry(0.115, 0.115, 0.008).translate(0, 0, 0.004);
const coneBodies = shadowy(new THREE.InstancedMesh(coneGeometry, standard(0xffffff, { roughness: 0.45 }), MAX_CONES));
const coneBands = shadowy(new THREE.InstancedMesh(bandGeometry, standard(0xf5f5f2, { roughness: 0.4 }), MAX_CONES));
const coneBases = shadowy(new THREE.InstancedMesh(baseGeometry, standard(0xffffff, { roughness: 0.7 }), MAX_CONES));
for (const mesh of [coneBodies, coneBands, coneBases]) { mesh.count = 0; mesh.frustumCulled = false; scene.add(mesh); }
const placeMatrix = new THREE.Matrix4();
const fadedColour = new THREE.Color();

// cones: [{x, y, colour, faded}]; faded cones are remembered but not in the latest detections.
function setCones(cones) {
  const n = Math.min(cones.length, MAX_CONES);
  for (let i = 0; i < n; i++) {
    const c = cones[i];
    placeMatrix.makeTranslation(c.x, c.y, 0);
    for (const mesh of [coneBodies, coneBands, coneBases]) mesh.setMatrixAt(i, placeMatrix);
    const colour = CONE_COLOURS[c.colour] || CONE_COLOURS.UNKNOWN;
    fadedColour.copy(colour).lerp(CONE_COLOURS.UNKNOWN, c.faded ? 0.55 : 0);
    coneBodies.setColorAt(i, fadedColour);
    coneBases.setColorAt(i, fadedColour);
  }
  for (const mesh of [coneBodies, coneBands, coneBases]) {
    mesh.count = n;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
  }
}

// ---- Text sprites ----------------------------------------------------------------------------------------------
const textCache = new Map();
function textTexture(text, background = 'rgba(17,21,27,0.78)') {
  const key = text + '|' + background;
  if (textCache.has(key)) return textCache.get(key);
  const lines = String(text).split('\n');
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  const size = 44, pad = 14;
  ctx.font = `600 ${size}px system-ui, sans-serif`;
  const width = Math.max(...lines.map((l) => ctx.measureText(l).width)) + 2 * pad;
  canvas.width = Math.ceil(width);
  canvas.height = lines.length * size * 1.2 + 2 * pad;
  ctx.fillStyle = background;
  ctx.beginPath(); ctx.roundRect(0, 0, canvas.width, canvas.height, 16); ctx.fill();
  ctx.font = `600 ${size}px system-ui, sans-serif`;
  ctx.fillStyle = '#f2f5f8';
  ctx.textBaseline = 'top';
  lines.forEach((line, i) => ctx.fillText(line, pad, pad + i * size * 1.2));
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  if (textCache.size > 300) { const [first] = textCache.keys(); textCache.get(first).dispose(); textCache.delete(first); }
  textCache.set(key, texture);
  return texture;
}
function textSprite(text, height = 0.06, background) {
  const texture = textTexture(text, background);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true }));
  sprite.scale.set(height * texture.image.width / texture.image.height, height, 1);
  sprite.renderOrder = 10;
  return sprite;
}

// ---- ArUco markers ---------------------------------------------------------------------------------------------
function markerTexture(bits, id) {
  const cells = 8, px = 32;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = cells * px;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#fafafa'; ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#060606';
  for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) {
    if (!bits || bits[r][c] === 0) ctx.fillRect((c + 1) * px, (r + 1) * px, px, px);
  }
  if (!bits) { ctx.fillStyle = '#fafafa'; ctx.font = `700 ${px * 2}px sans-serif`; ctx.fillText(String(id), px * 2.6, px * 4.8); }
  const texture = new THREE.CanvasTexture(canvas);
  texture.magFilter = THREE.NearestFilter;
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 8;
  return texture;
}
// Face of a marker: plane x -> world +y (to the viewer's right), plane y -> up, plane normal -> +x (towards the viewer)
const FACE_BASIS = new THREE.Matrix4().makeBasis(new V3(0, 1, 0), new V3(0, 0, 1), new V3(1, 0, 0));
const SIGHT_LABEL = 'rgba(122,48,190,0.92)';

function buildMarker(m) {
  const group = new THREE.Group();
  const board = m.size_m * 8 / 6;
  const face = new THREE.Mesh(new THREE.PlaneGeometry(board, board),
    new THREE.MeshStandardMaterial({ map: markerTexture(m.bits, m.id), roughness: 0.8 }));
  face.quaternion.setFromRotationMatrix(FACE_BASIS);
  face.position.set(0.0035, 0, 0);
  const backing = shadowy(new THREE.Mesh(new THREE.BoxGeometry(0.006, board * 1.06, board * 1.06), standard(0x5d6168)));
  const holder = new THREE.Group();
  holder.add(face, backing);
  holder.position.z = m.height_m ?? m.z ?? 0.2;
  group.add(holder);
  const bottom = holder.position.z - board / 2;
  if (bottom > 0.01) {
    const post = shadowy(new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.01, bottom, 12).rotateX(Math.PI / 2),
      standard(0x75797f, { metalness: 0.4 })));
    post.position.set(-0.01, 0, bottom / 2);
    group.add(post);
  }
  const label = `ID ${m.id}: ${m.command || 'no command'}`;
  group.userData = { label, sprite: textSprite(label, 0.055), seenSprite: textSprite(label, 0.055, SIGHT_LABEL) };
  group.userData.sprite.position.set(0, 0, holder.position.z + board / 2 + 0.06);
  group.userData.seenSprite.position.copy(group.userData.sprite.position);
  group.userData.seenSprite.visible = false;
  group.add(group.userData.sprite, group.userData.seenSprite);
  group.position.set(m.x, m.y, 0);
  group.rotation.z = deg(m.facing_deg);
  return group;
}
function setMarkerSeen(group, seen) {
  group.userData.sprite.visible = !seen;
  group.userData.seenSprite.visible = seen;
}

// ---- The static world (simulation) -----------------------------------------------------------------------------
const world = new THREE.Group();
scene.add(world);
let staticCones = [], markerGroups = [];

function clearGroup(group) {
  for (const child of [...group.children]) {
    group.remove(child);
    child.traverse((o) => { o.geometry?.dispose?.(); });
  }
}

function roadMesh(outline) {
  const half = Math.floor(outline.length / 2);
  const left = outline.slice(0, half), right = outline.slice(half).reverse();
  const positions = [];
  for (let i = 0; i < half - 1; i++) {
    const [a, b, c, d] = [left[i], left[i + 1], right[i + 1], right[i]];
    for (const p of [a, b, c, a, c, d]) positions.push(p[0], p[1], 0.002);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(geometry, standard(0xb9bcb6, { roughness: 0.95, side: THREE.DoubleSide }));
  mesh.receiveShadow = true;
  return mesh;
}

function buildWorld(track) {
  clearGroup(world);
  for (const road of track.roads) world.add(roadMesh(road));
  staticCones = track.cones.map(([x, y, colour]) => ({ x, y, colour, faded: false }));
  setCones(staticCones);
  markerGroups = track.markers.map((m) => { const g = buildMarker(m); world.add(g); return g; });
  for (const o of track.obstacles) {
    const box = shadowy(new THREE.Mesh(new THREE.BoxGeometry(o.length_m, o.width_m, o.height_m), standard(0xb08a5a, { roughness: 0.9 })));
    box.position.set(o.x, o.y, o.height_m / 2);
    box.rotation.z = deg(o.yaw_deg);
    world.add(box);
  }
  if (track.start && track.cones.length) {
    const [sx, sy, sh] = track.start;
    const startLabel = textSprite(`start  ${track.name}`, 0.07);
    startLabel.position.set(sx - 0.45 * Math.cos(sh), sy - 0.45 * Math.sin(sh), 0.06);
    world.add(startLabel);
  }
}

// ---- The car ---------------------------------------------------------------------------------------------------
const car = new THREE.Group();
scene.add(car);
const wheels = { front: [], spin: [] };
let wheelRadius = 0.0525;

function buildCar(g, camera) {
  clearGroup(car);
  wheels.front = []; wheels.spin = [];
  const length = g.front + g.rear, middle = (g.front - g.rear) / 2;
  const add = (geometry, material, x, y, z) => { const m = shadowy(new THREE.Mesh(geometry, material)); m.position.set(x, y, z); car.add(m); return m; };
  const dark = standard(0x25272c, { roughness: 0.5 });
  add(new THREE.BoxGeometry(length * 0.95, g.width * 0.72, 0.022), dark, middle, 0, 0.045);
  add(new THREE.BoxGeometry(length * 0.8, g.width * 0.84, 0.07), standard(0xc8312a, { roughness: 0.35, metalness: 0.25 }), middle - 0.01, 0, 0.098);
  add(new THREE.BoxGeometry(length * 0.3, g.width * 0.64, 0.035), standard(0x18212d, { roughness: 0.1, metalness: 0.6 }), middle - 0.04, 0, 0.148);
  add(new THREE.BoxGeometry(0.03, g.width * 0.72, 0.03), dark, g.front - 0.015, 0, 0.04);
  add(new THREE.BoxGeometry(0.045, g.width * 0.86, 0.008), dark, -g.rear + 0.04, 0, 0.165);
  wheelRadius = g.wheel_diameter / 2;
  const tyre = standard(0x0d0d0e, { roughness: 0.95 }), hub = standard(0xa7adb3, { roughness: 0.3, metalness: 0.7 });
  for (const [x, front] of [[g.front_axle, true], [g.rear_axle, false]]) {
    for (const side of [1, -1]) {
      const pivot = new THREE.Group();
      pivot.position.set(x, side * g.wheel_track / 2, wheelRadius);
      const spin = new THREE.Group();
      spin.add(shadowy(new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius, wheelRadius, 0.042, 30), tyre)));
      spin.add(new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius * 0.58, wheelRadius * 0.58, 0.044, 18), hub));
      for (const angle of [0, Math.PI / 3, 2 * Math.PI / 3]) {
        const spoke = new THREE.Mesh(new THREE.BoxGeometry(wheelRadius * 1.1, 0.046, 0.01), dark);
        spoke.rotation.y = angle;
        spin.add(spoke);
      }
      pivot.add(spin);
      car.add(pivot);
      wheels.spin.push(spin);
      if (front) wheels.front.push(pivot);
    }
  }
  // Camera mast and head, at the mount position and tilt
  const mastHeight = camera.z - 0.13;
  add(new THREE.CylinderGeometry(0.007, 0.009, mastHeight, 12).rotateX(Math.PI / 2), standard(0x6e737a, { metalness: 0.5 }),
    camera.x, 0, 0.13 + mastHeight / 2);
  const head = add(new THREE.BoxGeometry(0.03, 0.10, 0.028), standard(0x101114), camera.x, 0, camera.z);
  head.rotation.set(0, deg(camera.tilt_deg), deg(camera.yaw_deg));
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.01, 16).rotateZ(Math.PI / 2),
    standard(0x1e3a66, { roughness: 0.05, metalness: 0.9 }));
  lens.position.set(0.018, 0, 0);
  head.add(lens);
  const lidar = add(new THREE.CylinderGeometry(0.035, 0.035, 0.03, 24).rotateX(Math.PI / 2), standard(0x141518), ...g.lidar);
  const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.036, 0.006, 24).rotateX(Math.PI / 2), standard(0x3a8cff, { emissive: 0x1144aa }));
  ring.position.z = 0.008;
  lidar.add(ring);
  // The body is on its own layer, which the car's camera does not draw, so its view shows the surroundings
  car.traverse((object) => object.layers.set(BODY_LAYER));
  car.add(cameraMount, viewArea, lidarPoints, policyGroup);
}
const BODY_LAYER = 1;

// The car's camera: looks along +x tilted down, with the detector's measured horizontal view
const CAMERA_ASPECT = 16 / 9;
const cameraMount = new THREE.Group();
const carCamera = new THREE.PerspectiveCamera(55, CAMERA_ASPECT, 0.03, 60);
cameraMount.add(carCamera);
const viewArea = new THREE.Mesh(new THREE.BufferGeometry(),
  new THREE.MeshBasicMaterial({ color: 0x3d7fff, transparent: true, opacity: 0.13, depthWrite: false }));
viewArea.position.z = 0.003;

function placeCamera(camera) {
  cameraMount.position.set(camera.x, 0, camera.z);
  cameraMount.rotation.set(0, 0, deg(camera.yaw_deg));
  const t = deg(camera.tilt_deg), s = Math.sin(t), c = Math.cos(t);
  carCamera.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(new V3(0, -1, 0), new V3(s, 0, c), new V3(-c, 0, s)));
  carCamera.fov = 2 * Math.atan(Math.tan(deg(camera.hfov_deg) / 2) / CAMERA_ASPECT) * 180 / Math.PI;
  carCamera.updateProjectionMatrix();
  const half = deg(camera.hfov_deg) / 2, yaw = deg(camera.yaw_deg);
  const shape = new THREE.Shape();
  shape.absarc(0, 0, camera.far, yaw - half, yaw + half, false);
  shape.lineTo(camera.near * Math.cos(yaw + half), camera.near * Math.sin(yaw + half));
  shape.absarc(0, 0, camera.near, yaw + half, yaw - half, true);
  viewArea.geometry.dispose();
  viewArea.geometry = new THREE.ShapeGeometry(shape, 32);
}

// ---- Lidar and the policy's drawings (base_link) ---------------------------------------------------------------
const lidarPoints = new THREE.Points(new THREE.BufferGeometry(),
  new THREE.PointsMaterial({ color: 0xff2d55, size: 0.03, sizeAttenuation: true }));
lidarPoints.frustumCulled = false;
function setLidar(points) {
  const flat = new Float32Array(points.length * 3);
  points.forEach((p, i) => { flat[3 * i] = p[0]; flat[3 * i + 1] = p[1]; flat[3 * i + 2] = p[2]; });
  lidarPoints.geometry.setAttribute('position', new THREE.BufferAttribute(flat, 3));
}

const policyGroup = new THREE.Group();
const layerVisible = { car: false };
const lineMaterials = new Set();
let layersKey = '';
const sphereGeometry = new THREE.SphereGeometry(0.5, 12, 8);

function setLayers(layers) {
  const key = JSON.stringify(layers);
  if (key === layersKey) return;
  layersKey = key;
  clearGroup(policyGroup);
  lineMaterials.clear();
  Object.entries(layers || {}).forEach(([name, layer], index) => {
    if (!(name in layerVisible)) { layerVisible[name] = true; addLayerToggle(name); }
    const group = new THREE.Group();
    group.name = name;
    group.visible = layerVisible[name];
    const lift = 0.012 + 0.002 * index;
    for (const line of layer.lines || []) {
      if (line.points.length < 2) continue;
      const points = line.points.map(([x, y]) => [x, y, lift]);
      if (line.closed) points.push(points[0]);
      const geometry = new LineGeometry();
      geometry.setPositions(points.flat());
      const [r, g, b, a] = line.colour;
      const material = new LineMaterial({ color: new THREE.Color(r, g, b), linewidth: 3, transparent: a < 1, opacity: a });
      material.resolution.set(container.clientWidth, container.clientHeight);
      lineMaterials.add(material);
      group.add(new Line2(geometry, material));
    }
    for (const [x, y, d, [r, g, b, a]] of layer.spheres || []) {
      const sphere = new THREE.Mesh(sphereGeometry, new THREE.MeshBasicMaterial({ color: new THREE.Color(r, g, b), transparent: a < 1, opacity: a }));
      sphere.scale.setScalar(Math.max(d, 0.02));
      sphere.position.set(x, y, 0.2 + 0.015 * index);
      group.add(sphere);
    }
    for (const box of layer.boxes || []) {
      const [r, g, b, a] = box.colour;
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(...box.size),
        new THREE.MeshBasicMaterial({ color: new THREE.Color(r, g, b), transparent: true, opacity: Math.min(a, 0.55), depthWrite: false }));
      mesh.position.set(...box.centre);
      if (box.orientation) mesh.quaternion.set(...box.orientation);
      group.add(mesh);
    }
    for (const text of layer.texts || []) {
      const sprite = textSprite(text.text, 0.045);
      sprite.position.set(...text.position);
      group.add(sprite);
    }
    policyGroup.add(group);
  });
}

function addLayerToggle(name) {
  const label = document.createElement('label');
  label.className = 'check';
  const box = document.createElement('input');
  box.type = 'checkbox';
  box.checked = layerVisible[name];
  box.addEventListener('change', () => {
    layerVisible[name] = box.checked;
    const group = policyGroup.getObjectByName(name);
    if (group) group.visible = box.checked;
  });
  label.append(box, ` ${name}`);
  $('layers').append(label);
}

// ---- Live markers (with the car) -------------------------------------------------------------------------------
const liveMarkers = new Map();
function setLiveMarkers(markers) {
  const ids = new Set(markers.map((m) => m.id));
  for (const [id, group] of liveMarkers) if (!ids.has(id)) { world.remove(group); liveMarkers.delete(id); }
  for (const m of markers) {
    let group = liveMarkers.get(m.id);
    if (!group) { group = buildMarker(m); world.add(group); liveMarkers.set(m.id, group); }
    group.position.set(m.x, m.y, 0);
    group.rotation.z = deg(m.facing_deg);
    setMarkerSeen(group, m.age_s < 0.5);
  }
}

// ---- Views -----------------------------------------------------------------------------------------------------
const chaseCamera = new THREE.PerspectiveCamera(55, 1, 0.03, 300);
const topCamera = new THREE.PerspectiveCamera(45, 1, 0.1, 300);
const orbitCamera = new THREE.PerspectiveCamera(50, 1, 0.03, 300);
orbitCamera.position.set(-2.5, -2.5, 2.2);
for (const camera of [chaseCamera, topCamera, orbitCamera]) camera.layers.enable(BODY_LAYER);
const orbit = new OrbitControls(orbitCamera, renderer.domElement);
orbit.enableDamping = true;
orbit.enabled = false;
let view = 'chase';
const chaseLook = new V3();
const lastCarPosition = new V3();

function setView(next) {
  view = next;
  orbit.enabled = view === 'orbit';
  for (const button of document.querySelectorAll('[data-view]')) button.classList.toggle('on', button.dataset.view === view);
  $('inset-label').textContent = view === 'camera' ? 'Chase view' : 'Car camera';
}

function updateViews(dt) {
  const k = 1 - Math.exp(-dt * 6);
  const wanted = car.localToWorld(new V3(-1.55, 0, 0.9));
  chaseCamera.position.lerp(wanted, k);
  chaseLook.lerp(car.localToWorld(new V3(1.1, 0, 0.05)), k);
  chaseCamera.lookAt(chaseLook);
  const forward = new V3(1, 0, 0).applyQuaternion(car.quaternion);
  topCamera.position.copy(car.position).add(new V3(0, 0, 7.5));
  topCamera.up.copy(forward);
  topCamera.lookAt(car.position);
  const moved = car.position.clone().sub(lastCarPosition);
  orbitCamera.position.add(moved);
  orbit.target.add(moved);
  lastCarPosition.copy(car.position);
  if (view === 'orbit') orbit.update();
  sun.position.copy(car.position).add(new V3(-3, -4, 8));
  sun.target.position.copy(car.position);
}

// ---- Smooth motion between frames ------------------------------------------------------------------------------
const target = { x: 0, y: 0, heading: 0, wheelAngle: 0, distance: 0 };
let shown = null;
function animateCar(dt) {
  if (!shown) return;
  const k = 1 - Math.exp(-dt * 18);
  shown.x += (target.x - shown.x) * k;
  shown.y += (target.y - shown.y) * k;
  shown.heading += Math.atan2(Math.sin(target.heading - shown.heading), Math.cos(target.heading - shown.heading)) * k;
  shown.wheelAngle += (target.wheelAngle - shown.wheelAngle) * k;
  shown.distance += (target.distance - shown.distance) * k;
  car.position.set(shown.x, shown.y, 0);
  car.rotation.set(0, 0, shown.heading);
  for (const pivot of wheels.front) pivot.rotation.z = shown.wheelAngle;
  for (const spin of wheels.spin) spin.rotation.y = shown.distance / wheelRadius;
}

// ---- Rendering -------------------------------------------------------------------------------------------------
const clock = new THREE.Clock();
const insetFrame = $('inset-frame');
const renderSize = { w: 0, h: 0 };
function render() {
  requestAnimationFrame(render);
  const dt = Math.min(clock.getDelta(), 0.1);
  animateCar(dt);
  updateViews(dt);
  const w = container.clientWidth, h = container.clientHeight;
  if (w !== renderSize.w || h !== renderSize.h) {
    renderer.setSize(w, h);
    Object.assign(renderSize, { w, h });
    for (const material of lineMaterials) material.resolution.set(w, h);
  }
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, w, h);
  if (view === 'camera') {
    renderer.setClearColor(0x05070a);
    renderer.clear();
    let vw = w, vh = w / CAMERA_ASPECT;
    if (vh > h) { vh = h; vw = h * CAMERA_ASPECT; }
    renderer.setScissorTest(true);
    renderer.setScissor((w - vw) / 2, (h - vh) / 2, vw, vh);
    renderer.setViewport((w - vw) / 2, (h - vh) / 2, vw, vh);
    renderer.setClearColor(SKY);
    renderer.render(scene, carCamera);
  } else {
    const main = view === 'top' ? topCamera : view === 'orbit' ? orbitCamera : chaseCamera;
    main.aspect = w / h;
    main.updateProjectionMatrix();
    renderer.setClearColor(SKY);
    renderer.render(scene, main);
  }
  const inset = $('show-inset').checked;
  insetFrame.style.display = inset ? 'block' : 'none';
  if (inset) {
    const iw = Math.min(w * 0.34, 520), ih = iw / CAMERA_ASPECT, margin = 16, bottom = 70;
    const insetCamera = view === 'camera' ? chaseCamera : carCamera;
    if (insetCamera === chaseCamera) { chaseCamera.aspect = CAMERA_ASPECT; chaseCamera.updateProjectionMatrix(); }
    renderer.setScissorTest(true);
    renderer.setScissor(w - iw - margin, bottom, iw, ih);
    renderer.setViewport(w - iw - margin, bottom, iw, ih);
    renderer.render(scene, insetCamera);
    Object.assign(insetFrame.style, { right: margin + 'px', bottom: bottom + 'px', width: iw + 'px', height: ih + 'px' });
  }
}

// ---- Frames from the server ------------------------------------------------------------------------------------
let mode = null, sceneVersion = -1, sceneData = null, lastFrame = null;
const fmt = (v, digits = 2, unit = '') => (v === null || v === undefined ? '-' : `${Number(v).toFixed(digits)}${unit}`);

async function loadScene() {
  const response = await fetch('/scene');
  sceneData = await response.json();
  mode = sceneData.mode;
  sceneVersion = sceneData.version;
  buildWorld(sceneData.track);
  buildCar(sceneData.car, sceneData.camera);
  placeCamera(sceneData.camera);
  shown = null;
  layersKey = '';
  $('mode').textContent = mode === 'car' ? 'live car' : 'simulation';
  $('mode').classList.toggle('car', mode === 'car');
  $('trackname').textContent = sceneData.track.name;
  for (const element of document.querySelectorAll('.sim-only')) element.classList.toggle('hidden', mode !== 'sim');
  $('restart').textContent = mode === 'car' ? 'Clear memory' : 'Restart';
  $('distance-label').textContent = mode === 'car' ? 'Yaw rate' : 'Driven';
  const trackSelect = $('track-select');
  trackSelect.replaceChildren(...sceneData.tracks.map((t) => new Option(t.name, t.file, false, t.file === sceneData.track.file)));
  const markerBox = $('marker-box');
  markerBox.classList.toggle('hidden', mode !== 'sim' || !sceneData.track.markers.length);
  if (sceneData.track.markers.length) {
    const current = sceneData.track.markers[0].id;
    const names = sceneData.marker_names;
    const ids = [...new Set([...Object.keys(names).map(Number), current])].sort((a, b) => a - b);
    $('marker-select').replaceChildren(...ids.map((id) => new Option(`ID ${id}: ${names[id] || 'no command'}`, id, false, id === current)));
  }
}

function applyFrame(frame) {
  lastFrame = frame;
  if (frame.scene_version !== sceneVersion) { sceneVersion = frame.scene_version; loadScene(); return; }
  if (!sceneData) return;
  const c = frame.car;
  Object.assign(target, { x: c.x, y: c.y, heading: c.heading, wheelAngle: c.wheel_angle || 0, distance: c.distance || 0 });
  if (!shown) {
    shown = { ...target };
    lastCarPosition.set(c.x, c.y, 0);
    chaseCamera.position.set(c.x - 1.5 * Math.cos(c.heading), c.y - 1.5 * Math.sin(c.heading), 0.9);
    orbit.target.set(c.x, c.y, 0);
    orbitCamera.position.set(c.x - 2.5, c.y - 2.5, 2.2);
  }
  if (mode === 'sim') {
    const touched = new Set(frame.touched);
    staticCones.forEach((cone, i) => { cone.colourShown = touched.has(i) ? 'TOUCHED' : cone.colour; });
    setCones(staticCones.map((cone) => ({ ...cone, colour: cone.colourShown })));
    (frame.markers_seen || []).forEach((seen, i) => markerGroups[i] && setMarkerSeen(markerGroups[i], seen));
  } else {
    setCones(frame.cones.map(([x, y, colour, age, now]) => ({ x, y, colour, faded: !now })));
    setLiveMarkers(frame.markers);
  }
  setLidar(frame.lidar || []);
  setLayers(frame.layers);
  updateHud(frame);
}

function updateHud(frame) {
  const c = frame.car;
  $('clock').textContent = `${frame.t.toFixed(2)} s`;
  $('paused-badge').classList.toggle('hidden', !frame.paused);
  $('manual-badge').classList.toggle('hidden', !frame.manual);
  $('pause').textContent = frame.paused ? 'Resume' : 'Pause';
  $('manual').classList.toggle('on', !!frame.manual);
  $('speed').textContent = fmt(c.speed, 2, ' m/s');
  $('drive').textContent = fmt(c.drive, 2);
  $('steer').textContent = fmt(c.steer, 2);
  const bar = (id, v) => { const el = $(id); const x = Math.max(-1, Math.min(1, v || 0)); el.style.left = `${50 + Math.min(0, x) * 50}%`; el.style.width = `${Math.abs(x) * 50}%`; };
  bar('drive-bar', c.drive);
  bar('steer-bar', c.steer);
  const node = $('node');
  const rows = [];
  if (frame.mode === 'sim') {
    $('distance').textContent = fmt(c.distance, 2, ' m');
    node.textContent = frame.node.driving ? 'State 3: driving the policy' : `State ${frame.node.state}: ${frame.node.reason}`;
    node.className = 'state ' + (frame.node.driving ? 'driving' : 'zero');
    const hud = frame.hud;
    rows.push(['Lane', hud.lane || '-'], ['Lateral', fmt(hud.lateral_error_m, 2, ' m')], ['Heading', fmt(hud.heading_error_deg, 1, '°')],
      ['Cones seen / used', `${hud.cones_detected} / ${hud.cones_used}`], ['Lane side', hud.lane_side || '-'],
      ['Marker command', hud.marker_command || '-'], ['Next junction', hud.next_intersection || '-'],
      ['Junction phase', hud.intersection_phase || '-'], ['Obstacle ahead', fmt(hud.obstacle_distance_m, 2, ' m')],
      ['Policy stop', hud.mpc_stop || '-'], ['Cones touched', String(frame.touched.length)]);
    $('perception-title').textContent = 'Policy';
  } else {
    $('distance').textContent = fmt(c.yaw_rate, 2, ' rad/s');
    node.textContent = frame.hud.node || frame.status;
    node.className = 'state ' + (/policy actions/i.test(frame.hud.node || '') ? 'driving' : 'zero');
    rows.push(['Link', frame.status.startsWith('connected') ? 'connected' : 'not connected'],
      ['Cones shown', String(frame.cones.length)], ['Markers', frame.markers.map((m) => m.id).join(', ') || '-']);
    for (const [name, t] of Object.entries(frame.hud.topics)) rows.push([name, t.age_s === null ? 'none' : `${t.rate_hz} Hz`]);
    $('perception-title').textContent = 'Car streams';
  }
  $('perception').replaceChildren(...rows.flatMap(([k, v]) => { const dt = document.createElement('dt'); dt.textContent = k; const dd = document.createElement('dd'); dd.textContent = v; return [dt, dd]; }));
  $('events').textContent = frame.mode === 'sim' ? (frame.events.join('\n') || 'No events yet') : frame.status;
  const banner = $('banner');
  if (frame.ended) {
    banner.style.display = 'block';
    $('banner-text').textContent = `Run ended: ${frame.ended}`;
    $('banner-restart').classList.remove('hidden');
  } else {
    banner.style.display = 'none';
  }
}

function connect() {
  const banner = $('banner');
  const source = new EventSource('/stream');
  source.onmessage = (event) => applyFrame(JSON.parse(event.data));
  source.onerror = () => {
    banner.style.display = 'block';
    $('banner-text').textContent = 'Lost the simulator. Is it still running? Retrying...';
    $('banner-restart').classList.add('hidden');
  };
}

// ---- Controls --------------------------------------------------------------------------------------------------
const control = (body) => fetch('/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
$('pause').addEventListener('click', () => control({ op: lastFrame?.paused ? 'resume' : 'pause' }));
$('restart').addEventListener('click', () => control({ op: 'restart' }));
$('banner-restart').addEventListener('click', () => control({ op: 'restart' }));
$('manual').addEventListener('click', () => control({ op: 'manual', value: !lastFrame?.manual }));
$('speed-select').addEventListener('change', (e) => control({ op: 'speed', value: Number(e.target.value) }));
$('track-select').addEventListener('change', (e) => control({ op: 'track', value: e.target.value }));
$('marker-select').addEventListener('change', (e) => control({ op: 'marker_id', value: Number(e.target.value) }));
for (const button of document.querySelectorAll('[data-view]')) button.addEventListener('click', () => setView(button.dataset.view));
const toggle = (id, apply) => { const box = $(id); box.addEventListener('change', () => apply(box.checked)); apply(box.checked); };
toggle('show-fov', (on) => { viewArea.visible = on; });
toggle('show-lidar', (on) => { lidarPoints.visible = on; });
toggle('show-policy', (on) => { policyGroup.visible = on; });

const held = new Set();
let keysSent = '';
function sendKeys() {
  const forward = (held.has('ArrowUp') || held.has('KeyW') ? 1 : 0) - (held.has('ArrowDown') || held.has('KeyS') ? 1 : 0);
  const turn = (held.has('ArrowLeft') || held.has('KeyA') ? 1 : 0) - (held.has('ArrowRight') || held.has('KeyD') ? 1 : 0);
  const key = `${forward},${turn}`;
  if (key !== keysSent) { keysSent = key; control({ op: 'keys', forward, turn }); }
}
const DRIVE_KEYS = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'KeyW', 'KeyA', 'KeyS', 'KeyD']);
window.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLSelectElement) return;
  if (DRIVE_KEYS.has(e.code) && lastFrame?.manual) { held.add(e.code); sendKeys(); e.preventDefault(); return; }
  if (e.repeat) return;
  const box = (id) => { const b = $(id); b.checked = !b.checked; b.dispatchEvent(new Event('change')); };
  switch (e.code) {
    case 'Space': control({ op: lastFrame?.paused ? 'resume' : 'pause' }); e.preventDefault(); break;
    case 'KeyR': control({ op: 'restart' }); break;
    case 'KeyM': if (mode === 'sim') control({ op: 'manual', value: !lastFrame?.manual }); break;
    case 'Digit1': setView('chase'); break;
    case 'Digit2': setView('camera'); break;
    case 'Digit3': setView('top'); break;
    case 'Digit4': setView('orbit'); break;
    case 'KeyI': box('show-inset'); break;
    case 'KeyV': box('show-fov'); break;
    case 'KeyL': box('show-lidar'); break;
    case 'KeyP': box('show-policy'); break;
  }
});
window.addEventListener('keyup', (e) => { if (held.delete(e.code)) sendKeys(); });
window.addEventListener('blur', () => { held.clear(); sendKeys(); });

setView('chase');
render();
connect();
