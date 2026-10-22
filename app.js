// ==========================================================
// 空間偏好 AR：視野版 v3
//   1. 偵測與測量線條移至後台運算 (不顯示於前端 UI)
//   2. 鏡頭捕捉人物與手持手機偵測 -> 連機玩家(敵人)白色方框標記 + 真實/虛擬建築遮擋剔除
//   3. Canny Edge Detector + ARKit/LiDAR 幾何特徵空間標記物 (A-1, A-2...)，靠近 <2.2m 激活定位
//   4. 雙模式多人連機架構：支援 Node.js WebSocket 伺服器，並內建 WebRTC (PeerJS) 讓 GitHub Pages 靜態託管也能跨設備連機與查看伺服器地圖
//   5. 視線外點雲密度依轉動時中央區域掃描次數決定
//   6. 中央視野區大小依玩家實際移動速度動態縮放 (不動最大、慢放大、快縮小)
// ==========================================================
(() => {
'use strict';

const DEG = Math.PI / 180;
const $ = (id) => document.getElementById(id);
const clamp01 = (v) => (v < 0 ? 0 : v > 1 ? 1 : v);
const lerp = (a, b, t) => a + (b - a) * t;
const rand = (a, b) => a + Math.random() * (b - a);

// ---------- 畫布 ----------
const cv = $('view');
const ctx = cv.getContext('2d');
const scene = document.createElement('canvas'); // 鏡頭影像 + 虛擬建築
const sctx = scene.getContext('2d');
const sm = document.createElement('canvas'); // 低解析度副本，供分析用
const smctx = sm.getContext('2d', { willReadFrequently: true });
const video = $('cam');

let W = 0, H = 0, DPR = 1, U = 1;
const SAMPLE = 5;
let AW = 0, AH = 0;
let gray = new Float32Array(1);
let edgeMag = new Float32Array(1);
let bandEdge = [0], BAND_TOTAL = 1;

// ---------- 設定 ----------
const S = {
  sizeMul: 1, // 中央區大小基準倍率
  fovLong: 65, // 相機長邊視角（度）
  stepLen: 0.7, // 一步的長度（公尺）
  dotThreshold: 14,
  dotDensity: 4,
  dotMin: 0.7,
  dotMax: 1.6,
  dotFalloff: 320,
  markerCount: 18,
  bandCell: [25, 12.5, 6.25],
  bandWidth: [50, 37.5, 25],
  bandPow: 1.4,
  limitOn: true,
  showHud: true,
  showDet: false, // 強制關閉前台偵測線條顯示，移至後台運算
  freeGen: false,
};

const CEN_X = 0.2064;
const CEN_UP = 0.3058;
const CEN_DOWN = 0.4107;

function loadSettings() {
  try {
    const raw = localStorage.getItem('arVision.v3');
    if (raw) Object.assign(S, JSON.parse(raw));
    S.showDet = false; // 確保偵測線條保持在後台
  } catch (e) {}
}
function saveSettings() {
  try {
    localStorage.setItem('arVision.v3', JSON.stringify({
      sizeMul: S.sizeMul, fovLong: S.fovLong, stepLen: S.stepLen,
      dotThreshold: S.dotThreshold, showHud: S.showHud
    }));
  } catch (e) {}
}

// ==========================================================
// 狀態與雙模式多人連機網路 (Node.js WebSocket + GitHub Pages PeerJS 雲端連機)
// ==========================================================
let running = false;
let C = { l: 0, r: 0, t: 0, b: 0 };
const EYE = 1.5;
const pos = { x: 0, z: 0 };
let pending = 0;
let moveSpeed = 0;
let steps = 0;
let holdWalk = false;
let lastT = 0;
let detTick = 0;
let timeSinceStart = 0;

let B = { sr: [1, 0, 0], su: [0, 1, 0], F: [0, 0, -1] };
let FPX = 600;
let hasSensor = false;
let sensor = { a: 0, b: 90, g: 0, seen: false };
let h0 = null;
let yawM = 0, pitchM = 0;
let headX = 0, headZ = -1;
let prevYaw = 0;

const net = {
  mode: 'connecting', // 'ws' | 'webrtc-host' | 'webrtc-client' | 'local'
  room: 'AR-ROOM-1',
  ws: null,
  peer: null,
  conns: [],
  id: 'P_' + Math.random().toString(36).substring(2, 6).toUpperCase(),
  connected: false,
  players: {}, // 其他連線玩家 id -> { id, x, z, yaw, speed }
  markers: {}, // 空間標記物 id (如 A-1) -> { id, x, y, z, nx, nz, active, activatedBy }
  walls: [],
  lastSync: 0,
  localSeq: 0,
};

function nextLocalMarkerId() {
  net.localSeq++;
  const group = String.fromCharCode(65 + Math.floor((net.localSeq - 1) / 20));
  const num = ((net.localSeq - 1) % 20) + 1;
  return `${group}-${num}`;
}

function handleNetMessage(msg, fromConn = null) {
  if (!msg || !msg.type) return;
  if (msg.type === 'init') {
    if (msg.id) net.id = msg.id;
    net.players = Object.assign({}, msg.players || {});
    delete net.players[net.id];
    net.markers = msg.markers || {};
    net.walls = msg.walls || [];
  } else if (msg.type === 'playerJoined' || msg.type === 'playerMove') {
    if (msg.player && msg.player.id !== net.id) {
      net.players[msg.player.id] = msg.player;
    }
    if (net.mode === 'webrtc-host') broadcastPeers(msg, fromConn);
  } else if (msg.type === 'move') {
    // 來自 WebRTC client 的移動訊息
    const pid = msg.id;
    if (pid && pid !== net.id) {
      net.players[pid] = { id: pid, x: msg.x, z: msg.z, yaw: msg.yaw, speed: msg.speed, lastSeen: Date.now() };
      if (net.mode === 'webrtc-host') broadcastPeers({ type: 'playerMove', player: net.players[pid] }, fromConn);
    }
  } else if (msg.type === 'playerLeft') {
    delete net.players[msg.id];
    if (net.mode === 'webrtc-host') broadcastPeers(msg, fromConn);
  } else if (msg.type === 'registerMarker') {
    if (net.mode === 'webrtc-host') {
      const exists = Object.values(net.markers).find((m) => Math.hypot(m.x - msg.x, m.z - msg.z) < 1.2);
      if (!exists) {
        const id = nextLocalMarkerId();
        net.markers[id] = { id, x: msg.x, y: msg.y || 1.2, z: msg.z, nx: msg.nx || 0, nz: msg.nz || 1, active: false };
        broadcastPeers({ type: 'markerSpawned', marker: net.markers[id] });
      }
    }
  } else if (msg.type === 'activateMarker') {
    const m = net.markers[msg.id];
    if (m) {
      m.active = true;
      if (Array.isArray(msg.walls)) {
        msg.walls.forEach((w) => { if (net.walls.length < 300) net.walls.push(w); });
      }
      if (net.mode === 'webrtc-host') {
        broadcastPeers({ type: 'markerActivated', marker: m, playerId: msg.playerId || net.id, walls: net.walls });
      }
    }
  } else if (msg.type === 'markerSpawned' || msg.type === 'markerActivated') {
    if (msg.marker) net.markers[msg.marker.id] = msg.marker;
    if (msg.walls) net.walls = msg.walls;
  }
}

function broadcastPeers(data, excludeConn = null) {
  net.conns.forEach((c) => {
    if (c !== excludeConn && c.open) {
      try { c.send(data); } catch (e) {}
    }
  });
}

function initPeerJSFallback() {
  if (!window.Peer) {
    net.mode = 'local';
    return;
  }
  const cleanRoom = net.room.replace(/[^a-zA-Z0-9_-]/g, '');
  const hostId = `ar-vision-room-${cleanRoom}-host`;

  // 先嘗試成為該房間的 Host (方便 GitHub Pages 免架設後端直接連機)
  const peer = new Peer(hostId);
  net.peer = peer;

  peer.on('open', () => {
    net.mode = 'webrtc-host';
    net.connected = true;
  });

  peer.on('connection', (conn) => {
    net.conns.push(conn);
    conn.on('open', () => {
      const allPlayers = Object.assign({}, net.players);
      allPlayers[net.id] = { id: net.id, x: pos.x, z: pos.z, yaw: gameHeading(), speed: moveSpeed };
      conn.send({
        type: 'init',
        id: 'P_' + Math.random().toString(36).substring(2, 6).toUpperCase(),
        players: allPlayers,
        markers: net.markers,
        walls: net.walls,
      });
    });
    conn.on('data', (data) => handleNetMessage(data, conn));
    conn.on('close', () => {
      net.conns = net.conns.filter((c) => c !== conn);
    });
  });

  // 若 Host ID 已被第一位玩家或 Dashboard 佔用，則自動轉為 Client 連入該 Host
  peer.on('error', (err) => {
    if (err.type === 'unavailable-id') {
      const clientPeer = new Peer();
      net.peer = clientPeer;
      clientPeer.on('open', () => {
        const conn = clientPeer.connect(hostId);
        conn.on('open', () => {
          net.mode = 'webrtc-client';
          net.connected = true;
          net.conns = [conn];
          conn.send({ type: 'playerJoined', player: { id: net.id, x: pos.x, z: pos.z, yaw: gameHeading(), speed: moveSpeed } });
        });
        conn.on('data', (data) => handleNetMessage(data, conn));
        conn.on('close', () => { net.connected = false; });
      });
    } else {
      net.mode = 'local';
    }
  });
}

function initNetwork() {
  const roomInput = $('roomCode');
  if (roomInput && roomInput.value.trim()) {
    net.room = roomInput.value.trim();
  }
  // 若在 GitHub Pages (github.io) 或純靜態環境，直接啟用 WebRTC PeerJS 連機；若在本地/自有伺服器則優先連 WebSocket
  if (location.hostname.endsWith('github.io')) {
    initPeerJSFallback();
    return;
  }
  try {
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${proto}//${location.host}`;
    const ws = new WebSocket(wsUrl);
    net.ws = ws;

    const timeout = setTimeout(() => {
      if (!net.connected) {
        try { ws.close(); } catch (e) {}
        initPeerJSFallback();
      }
    }, 1800);

    ws.onopen = () => {
      clearTimeout(timeout);
      net.mode = 'ws';
      net.connected = true;
    };
    ws.onclose = () => {
      if (net.mode === 'ws') {
        net.connected = false;
        initPeerJSFallback();
      }
    };
    ws.onerror = () => {};
    ws.onmessage = (ev) => {
      try { handleNetMessage(JSON.parse(ev.data)); } catch (e) {}
    };
  } catch (e) {
    initPeerJSFallback();
  }
}

function sendNet(payload) {
  payload.id = payload.id || net.id;
  if (net.mode === 'ws' && net.ws && net.ws.readyState === WebSocket.OPEN) {
    net.ws.send(JSON.stringify(payload));
  } else if (net.mode === 'webrtc-client' && net.conns[0] && net.conns[0].open) {
    net.conns[0].send(payload);
  } else if (net.mode === 'webrtc-host' || net.mode === 'local') {
    handleNetMessage(payload);
  }
}

// ==========================================================
// 視線掃描記憶網格 (決定外圍點雲密度)
// ==========================================================
const SCAN_AZ = 72;
const SCAN_EL = 36;
const scanMap = new Float32Array(SCAN_AZ * SCAN_EL);

function dirToScanIdx(rx, ry, rz) {
  const len = Math.hypot(rx, ry, rz) || 1;
  const az = Math.atan2(rx / len, -rz / len);
  const el = Math.asin(Math.max(-1, Math.min(1, ry / len)));
  const bi = Math.min(SCAN_AZ - 1, Math.max(0, Math.floor(((az + Math.PI) / (2 * Math.PI)) * SCAN_AZ)));
  const bj = Math.min(SCAN_EL - 1, Math.max(0, Math.floor(((el + Math.PI / 2) / Math.PI) * SCAN_EL)));
  return bj * SCAN_AZ + bi;
}

function updateScanMemory(dt) {
  const stepX = Math.max(12, (C.r - C.l) / 8);
  const stepY = Math.max(12, (C.b - C.t) / 8);
  for (let py = C.t; py <= C.b; py += stepY) {
    for (let px = C.l; px <= C.r; px += stepX) {
      const r = rayW(px, py);
      const idx = dirToScanIdx(r[0], r[1], r[2]);
      scanMap[idx] = Math.min(100, scanMap[idx] + dt * 18);
    }
  }
}

function getScanDensityFactor(px, py) {
  const r = rayW(px, py);
  const idx = dirToScanIdx(r[0], r[1], r[2]);
  const count = scanMap[idx];
  return 0.15 + 2.85 * clamp01(count / 25);
}

// ==========================================================
// 尺寸
// ==========================================================
function resize() {
  DPR = Math.min(window.devicePixelRatio || 1, 2);
  W = Math.max(200, window.innerWidth);
  H = Math.max(200, window.innerHeight);
  for (const c of [cv, scene]) {
    c.width = Math.round(W * DPR);
    c.height = Math.round(H * DPR);
  }
  cv.style.width = W + 'px';
  cv.style.height = H + 'px';
  AW = Math.ceil(W / SAMPLE);
  AH = Math.ceil(H / SAMPLE);
  sm.width = AW;
  sm.height = AH;
  gray = new Float32Array(AW * AH);
  edgeMag = new Float32Array(AW * AH);
  U = Math.min(1.1, Math.max(0.55, Math.min(W, H) / 560));
  bandEdge = [0];
  for (let i = 0; i < S.bandWidth.length; i++) bandEdge.push(bandEdge[i] + S.bandWidth[i] * U);
  BAND_TOTAL = bandEdge[bandEdge.length - 1];
  C = centerRect();
  detectResize();
  SAFE_TOP = safeTop();
}
window.addEventListener('resize', resize);
window.addEventListener('orientationchange', () => setTimeout(resize, 200));

// ==========================================================
// 感測器與相機座標
// ==========================================================
function screenAngle() {
  let a = 0;
  if (screen.orientation && typeof screen.orientation.angle === 'number') a = screen.orientation.angle;
  else if (typeof window.orientation === 'number') a = window.orientation;
  return ((a % 360) + 360) % 360;
}

function e2g(v) {
  const h = Math.atan2(v[0], v[1]);
  const th = h - h0;
  const rho = Math.hypot(v[0], v[1]);
  return [rho * Math.sin(th), v[2], -rho * Math.cos(th)];
}

function sensorBasis() {
  const x = sensor.b * DEG, y = sensor.g * DEG, z = sensor.a * DEG;
  const cX = Math.cos(x), cY = Math.cos(y), cZ = Math.cos(z);
  const sX = Math.sin(x), sY = Math.sin(y), sZ = Math.sin(z);
  const m11 = cZ * cY - sZ * sX * sY, m12 = -cX * sZ, m13 = cY * sZ * sX + cZ * sY;
  const m21 = cY * sZ + cZ * sX * sY, m22 = cZ * cX, m23 = sZ * sY - cZ * cY * sX;
  const m31 = -cX * sY, m32 = sX, m33 = cX * cY;
  const Re = [m11, m21, m31];
  const Ue = [m12, m22, m32];
  const Fe = [-m13, -m23, -m33];
  if (h0 === null) {
    const hf = Math.hypot(Fe[0], Fe[1]) > 0.3 ? Fe : Ue;
    h0 = Math.atan2(hf[0], hf[1]);
  }
  let r = e2g(Re), u = e2g(Ue);
  const f = e2g(Fe);
  const ang = screenAngle();
  let sr = r, su = u;
  if (ang === 90) { sr = u.map((q) => -q); su = r; }
  else if (ang === 180) { sr = r.map((q) => -q); su = u.map((q) => -q); }
  else if (ang === 270) { sr = u; su = r.map((q) => -q); }
  return { sr, su, F: f, up: u };
}

function manualBasis() {
  const cy = Math.cos(yawM), sy = Math.sin(yawM), cp = Math.cos(pitchM), sp = Math.sin(pitchM);
  return {
    sr: [cy, 0, sy],
    su: [-sy * sp, cp, cy * sp],
    F: [sy * cp, sp, -cy * cp],
    up: [-sy * sp, cp, cy * sp],
  };
}

function updateCamera() {
  const nb = hasSensor && sensor.seen ? sensorBasis() : manualBasis();
  B = nb;
  let hx = nb.F[0], hz = nb.F[2];
  let hl = Math.hypot(hx, hz);
  if (hl < 0.35) {
    const s = nb.F[1] < 0 ? 1 : -1;
    hx = nb.up[0] * s;
    hz = nb.up[2] * s;
    hl = Math.hypot(hx, hz);
  }
  if (hl > 1e-4) {
    headX = hx / hl;
    headZ = hz / hl;
  }
}

function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

function toCam(vx, vy, vz) {
  const v = [vx, vy, vz];
  return [dot3(v, B.sr), dot3(v, B.su), dot3(v, B.F)];
}
function toScreen(c) {
  return [W / 2 + (FPX * c[0]) / c[2], H / 2 - (FPX * c[1]) / c[2]];
}

// ==========================================================
// 視野：中央區依據實際移動速度動態放大與縮小
//   不動 (speed ~ 0) -> 最大 (1.28x)
//   慢走 (0.05 ~ 0.75 m/s) -> 放大 (1.02 ~ 1.28x)
//   快走 (> 0.75 m/s) -> 縮小 (最低至 0.38x)
// ==========================================================
const fov = { l: 1.25, r: 1.25, u: 1.25, d: 1.25, move: 0, speedScale: 1.25, prox: { l: 0, r: 0, f: 0 } };
const FOV_MIN = 0.28, SIDE_SHRINK = 0.5, FRONT_SHRINK = 0.3;
const COVER_RANGE = 3.0;
const FOV_IN = 0.18, FOV_OUT = 0.08;

function centerRect() {
  const k = S.sizeMul;
  const kx = k * (W < H ? 1.3 : 1);
  return {
    l: Math.max(0, W / 2 - CEN_X * W * kx * fov.l),
    r: Math.min(W, W / 2 + CEN_X * W * kx * fov.r),
    t: Math.max(0, H / 2 - CEN_UP * H * k * fov.u),
    b: Math.min(H, H / 2 + CEN_DOWN * H * k * fov.d),
  };
}

function rectDist(x, y) {
  const dx = Math.max(C.l - x, 0, x - C.r);
  const dy = Math.max(C.t - y, 0, y - C.b);
  return Math.max(dx, dy);
}

function rayHitBox(ox, oz, dx, dz, o, pad) {
  const r = o.rot || 0, cr = Math.cos(r), sr = Math.sin(r);
  const px = ox - o.x, pz = oz - o.z;
  const org = [px * cr + pz * sr, -px * sr + pz * cr];
  const dir = [dx * cr + dz * sr, -dx * sr + dz * cr];
  const lo = [-o.w / 2 - pad, -o.d / 2 - pad];
  const hi = [o.w / 2 + pad, o.d / 2 + pad];
  let t0 = 0, t1 = COVER_RANGE;
  for (let k = 0; k < 2; k++) {
    if (Math.abs(dir[k]) < 1e-9) {
      if (org[k] < lo[k] || org[k] > hi[k]) return Infinity;
    } else {
      let a = (lo[k] - org[k]) / dir[k];
      let b = (hi[k] - org[k]) / dir[k];
      if (a > b) { const t = a; a = b; b = t; }
      t0 = Math.max(t0, a);
      t1 = Math.min(t1, b);
      if (t0 > t1) return Infinity;
    }
  }
  return t0;
}

function coverDist(angRad) {
  const fx = headX, fz = headZ;
  const rx = -headZ, rz = headX;
  const c = Math.cos(angRad), sn = Math.sin(angRad);
  const dx = fx * c + rx * sn, dz = fz * c + rz * sn;
  let best = COVER_RANGE;
  for (const o of obs) {
    if (o.h < 0.8 || o.dying) continue;
    if (Math.hypot(o.x - pos.x, o.z - pos.z) > COVER_RANGE + Math.max(o.w, o.d) / 2 + 0.5) continue;
    const hit = rayHitBox(pos.x, pos.z, dx, dz, o, 0.3);
    if (hit < best) best = hit;
  }
  return best;
}

function coverProx(list) {
  let best = 0;
  for (const it of list) {
    const q = clamp01(1 - coverDist(it[0] * DEG) / COVER_RANGE);
    best = Math.max(best, it[1] * q * q * (3 - 2 * q));
  }
  return best;
}

function updateVision() {
  const tgt = { l: 1, r: 1, u: 1, d: 1 };
  if (running) {
    let speedFactor = 1.28;
    if (moveSpeed < 0.05) {
      speedFactor = 1.28; // 不動：最大
    } else if (moveSpeed <= 0.75) {
      speedFactor = lerp(1.28, 1.02, (moveSpeed - 0.05) / 0.7); // 慢走：放大
    } else {
      speedFactor = lerp(1.02, 0.38, clamp01((moveSpeed - 0.75) / 1.15)); // 快走：縮小
    }
    fov.speedScale = speedFactor;
    fov.move = clamp01(moveSpeed / 1.8);

    const pr = coverProx([[40, 0.85], [90, 1], [140, 0.7]]);
    const pl = coverProx([[-40, 0.85], [-90, 1], [-140, 0.7]]);
    const pf = coverProx([[0, 1], [-20, 0.8], [20, 0.8]]);
    fov.prox.l = pl; fov.prox.r = pr; fov.prox.f = pf;

    tgt.l = speedFactor * (1 - SIDE_SHRINK * pl);
    tgt.r = speedFactor * (1 - SIDE_SHRINK * pr);
    tgt.u = speedFactor * (1 - FRONT_SHRINK * pf);
    tgt.d = tgt.u;
  } else {
    fov.move = 0; fov.speedScale = 1.28;
    fov.prox.l = fov.prox.r = fov.prox.f = 0;
  }
  for (const k of ['l', 'r', 'u', 'd']) {
    const goal = Math.max(FOV_MIN, tgt[k] * susp.fovMul);
    fov[k] = lerp(fov[k], goal, goal < fov[k] ? FOV_IN : FOV_OUT);
  }
  C = centerRect();
}

// ==========================================================
// 視野：外圍點雲（根據轉動時中央區域掃描次數決定密度）
// ==========================================================
function hash(a, b, c) {
  let x = Math.imul(a | 0, 73856093) ^ Math.imul(b | 0, 19349663) ^ Math.imul(c | 0, 83492791);
  x = Math.imul(x ^ (x >>> 13), 1274126177);
  x ^= x >>> 16;
  return (x >>> 0) / 4294967296;
}

function analyze() {
  smctx.drawImage(scene, 0, 0, AW, AH);
  const p = smctx.getImageData(0, 0, AW, AH).data;
  for (let k = 0; k < AW * AH; k++) gray[k] = 0.299 * p[4 * k] + 0.587 * p[4 * k + 1] + 0.114 * p[4 * k + 2];
  for (let j = 1; j < AH - 1; j++) {
    for (let i = 1; i < AW - 1; i++) {
      const k = j * AW + i;
      const gx = -gray[k - AW - 1] + gray[k - AW + 1] - 2 * gray[k - 1] + 2 * gray[k + 1] - gray[k + AW - 1] + gray[k + AW + 1];
      const gy = -gray[k - AW - 1] - 2 * gray[k - AW] - gray[k - AW + 1] + gray[k + AW - 1] + 2 * gray[k + AW] + gray[k + AW + 1];
      const m = Math.sqrt(gx * gx + gy * gy) / 4;
      edgeMag[k] = 0.6 * m + 0.4 * edgeMag[k];
    }
  }
}

function blit(x, y, w, h) {
  if (w <= 0 || h <= 0) return;
  ctx.drawImage(scene, x * DPR, y * DPR, w * DPR, h * DPR, x, y, w, h);
}

function drawDots() {
  const thr = S.dotThreshold;
  const buckets = [[], [], []];
  const ud = Math.min(1.3, Math.max(0.85, U * 1.4));
  const radii = [S.dotMin * ud, ((S.dotMin + S.dotMax) / 2) * ud, S.dotMax * ud];
  for (let j = 0; j < AH; j++) {
    for (let i = 0; i < AW; i++) {
      const k = j * AW + i;
      const x = i * SAMPLE + SAMPLE / 2, y = j * SAMPLE + SAMPLE / 2;
      const d = rectDist(x, y);
      if (d <= 0) continue;
      const edgeN = clamp01((edgeMag[k] - thr * 0.5) / (thr * 3.5));
      const lumN = Math.pow(gray[k] / 255, 2.2) * 0.3;
      const v = Math.max(edgeN, lumN);
      if (v <= 0.02) continue;
      const prox = 0.3 + 0.7 * (1 - Math.min(1, d / (S.dotFalloff * U)));
      const scanMul = getScanDensityFactor(x, y);
      const n = Math.min(9, Math.floor(v * prox * S.dotDensity * scanMul + hash(i, j, 0)));
      if (n <= 0) continue;
      const b = edgeN > 0.66 ? 2 : edgeN > 0.33 ? 1 : 0;
      for (let q = 0; q < n; q++) {
        buckets[b].push(x + (hash(i, j, q + 1) - 0.5) * SAMPLE, y + (hash(i, j, q + 11) - 0.5) * SAMPLE);
      }
    }
  }
  ctx.save();
  ctx.fillStyle = 'rgba(255,255,255,0.92)';
  for (let b = 0; b < 3; b++) {
    const pts = buckets[b], r = radii[b];
    ctx.beginPath();
    for (let q = 0; q < pts.length; q += 2) {
      ctx.moveTo(pts[q] + r, pts[q + 1]);
      ctx.arc(pts[q], pts[q + 1], r, 0, Math.PI * 2);
    }
    ctx.fill();
  }
  ctx.restore();
}

function drawBand() {
  for (let L = 0; L < S.bandCell.length; L++) {
    const s = S.bandCell[L] * U;
    const d0 = bandEdge[L], d1 = bandEdge[L + 1];
    const kx0 = Math.floor((0 - C.l) / s) - 1, kx1 = Math.ceil((W - C.l) / s) + 1;
    const ky0 = Math.floor((0 - C.t) / s) - 1, ky1 = Math.ceil((H - C.t) / s) + 1;
    for (let ky = ky0; ky <= ky1; ky++) {
      for (let kx = kx0; kx <= kx1; kx++) {
        const x0 = C.l + kx * s, y0 = C.t + ky * s;
        const cx = x0 + s / 2, cy = y0 + s / 2;
        if (cx < 0 || cx > W || cy < 0 || cy > H) continue;
        const d = rectDist(cx, cy);
        if (d <= d0 || d > d1) continue;
        const p = Math.pow(1 - d / BAND_TOTAL, S.bandPow);
        if (hash(kx, ky, L + 50) > p) continue;
        blit(x0, y0, s, s);
      }
    }
  }
}

function composeLimited() {
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, W, H);
  analyze();
  drawDots();
  drawBand();
  blit(C.l, C.t, C.r - C.l, C.b - C.t);
}

// ==========================================================
// 遮擋判定系統 (Occlusion Culling)
// ==========================================================
function segIntersect2D(ax, az, bx, bz, cx, cz, dx, dz) {
  const d1x = bx - ax, d1z = bz - az, d2x = dx - cx, d2z = dz - cz;
  const den = d1x * d2z - d1z * d2x;
  if (Math.abs(den) < 1e-9) return false;
  const t = ((cx - ax) * d2z - (cz - az) * d2x) / den;
  const u = ((cx - ax) * d1z - (cz - az) * d1x) / den;
  return t > 0.03 && t < 0.97 && u >= 0 && u <= 1;
}

function isOccluded3D(tx, tz, targetHeight = 1.7) {
  const distToTarget = Math.hypot(tx - pos.x, tz - pos.z);
  if (distToTarget < 0.4) return false;

  for (const w of world.walls) {
    if (w.hits < 2) continue;
    if (segIntersect2D(pos.x, pos.z, tx, tz, w.x1, w.z1, w.x2, w.z2)) {
      return true;
    }
  }

  for (const e of world.ents) {
    if (e.hits < 3 || e.type !== 'column') continue;
    const r = Math.max(0.2, e.w / 2);
    const box = { x: e.x, z: e.z, w: r * 2, d: r * 2, rot: 0 };
    const dx = (tx - pos.x) / distToTarget, dz = (tz - pos.z) / distToTarget;
    const hit = rayHitBox(pos.x, pos.z, dx, dz, box, 0.05);
    if (hit < distToTarget - 0.2) return true;
  }

  const dx = (tx - pos.x) / distToTarget, dz = (tz - pos.z) / distToTarget;
  for (const o of obs) {
    if (o.dying) continue;
    if (o.h < 1.1 && targetHeight > 1.3) continue;
    const hit = rayHitBox(pos.x, pos.z, dx, dz, o, 0.02);
    if (hit < distToTarget - 0.15) return true;
  }

  return false;
}

// ==========================================================
// Canny Edge Detector + ARKit/LiDAR 幾何特徵空間標記物 (A-1, A-2...)
// ==========================================================
const MARKER_ACTIVATE_DIST = 2.2;
let cannyMag = new Float32Array(1);
let cannyDir = new Uint8Array(1);
let cannyNms = new Float32Array(1);
let cannyEdge = new Uint8Array(1);
let lastMarkerSpawn = 0;

function runCannyEdgeDetector() {
  const w = DW, h = DH;
  if (w < 10 || h < 10) return [];
  if (cannyMag.length !== w * h) {
    cannyMag = new Float32Array(w * h);
    cannyDir = new Uint8Array(w * h);
    cannyNms = new Float32Array(w * h);
    cannyEdge = new Uint8Array(w * h);
  }
  cannyNms.fill(0);
  cannyEdge.fill(0);

  let maxM = 0;
  for (let j = 2; j < h - 2; j++) {
    for (let i = 2; i < w - 2; i++) {
      const k = j * w + i;
      const gx = dgx[k], gy = dgy[k];
      const m = dmag[k];
      cannyMag[k] = m;
      if (m > maxM) maxM = m;
      const ang = ((Math.atan2(gy, gx) * 180) / Math.PI + 180) % 180;
      if (ang < 22.5 || ang >= 157.5) cannyDir[k] = 0;
      else if (ang < 67.5) cannyDir[k] = 1;
      else if (ang < 112.5) cannyDir[k] = 2;
      else cannyDir[k] = 3;
    }
  }

  for (let j = 2; j < h - 2; j++) {
    for (let i = 2; i < w - 2; i++) {
      const k = j * w + i;
      const m = cannyMag[k];
      if (m < 8) continue;
      const dir = cannyDir[k];
      let m1 = 0, m2 = 0;
      if (dir === 0) { m1 = cannyMag[k - 1]; m2 = cannyMag[k + 1]; }
      else if (dir === 1) { m1 = cannyMag[k - w + 1]; m2 = cannyMag[k + w - 1]; }
      else if (dir === 2) { m1 = cannyMag[k - w]; m2 = cannyMag[k + w]; }
      else { m1 = cannyMag[k - w - 1]; m2 = cannyMag[k + w + 1]; }
      if (m >= m1 && m >= m2) cannyNms[k] = m;
    }
  }

  const highThr = Math.max(18, maxM * 0.32);
  const lowThr = highThr * 0.45;
  const anchorCandidates = [];

  for (let j = 4; j < h - 4; j++) {
    for (let i = 4; i < w - 4; i++) {
      const k = j * w + i;
      if (cannyNms[k] >= highThr) {
        cannyEdge[k] = 2;
        if (dR[k] > (det.harrisMax || 1) * 0.04) {
          anchorCandidates.push({ i, j, score: cannyNms[k] + dR[k] * 0.01 });
        }
      } else if (cannyNms[k] >= lowThr) {
        cannyEdge[k] = 1;
      }
    }
  }
  anchorCandidates.sort((a, b) => b.score - a.score);
  return anchorCandidates.slice(0, 12);
}

function hitWorldSurface(px, py) {
  const r = rayW(px, py);
  const lenXZ = Math.hypot(r[0], r[2]);
  if (lenXZ < 1e-4) return null;
  const dx = r[0] / lenXZ, dz = r[2] / lenXZ;

  let bestDist = 9.0;
  let hitPoint = null;

  for (const w of world.walls) {
    if (w.hits < 2) continue;
    const d1x = dx * 12, d1z = dz * 12;
    const d2x = w.x2 - w.x1, d2z = w.z2 - w.z1;
    const den = d1x * d2z - d1z * d2x;
    if (Math.abs(den) < 1e-6) continue;
    const t = ((w.x1 - pos.x) * d2z - (w.z1 - pos.z) * d2x) / den;
    const u = ((w.x1 - pos.x) * d1z - (w.z1 - pos.z) * d1x) / den;
    if (t > 0.05 && t < 1.0 && u >= 0 && u <= 1) {
      const dist = t * 12;
      const hitY = EYE + (r[1] / lenXZ) * dist;
      if (dist < bestDist && hitY >= 0.2 && hitY <= 2.5) {
        bestDist = dist;
        const s = sideOf(w);
        hitPoint = {
          x: pos.x + dx * dist,
          y: hitY,
          z: pos.z + dz * dist,
          nx: s.nx,
          nz: s.nz,
          dist,
        };
      }
    }
  }

  for (const e of world.ents) {
    if (e.hits < 2) continue;
    const ed = Math.hypot(e.x - pos.x, e.z - pos.z);
    if (ed < 0.8 || ed > bestDist) continue;
    const ang = Math.abs(wrapPi(Math.atan2(e.x - pos.x, -(e.z - pos.z)) - Math.atan2(dx, -dz)));
    if (ang < 6 * DEG) {
      const hitY = Math.max(0.3, Math.min(2.2, EYE + (r[1] / lenXZ) * ed));
      bestDist = ed;
      const nx = (pos.x - e.x) / ed, nz = (pos.z - e.z) / ed;
      hitPoint = { x: e.x, y: hitY, z: e.z, nx, nz, dist: ed };
    }
  }

  return hitPoint;
}

function updateSpatialMarkers(now) {
  if (now - lastMarkerSpawn > 2200) {
    const cands = runCannyEdgeDetector();
    for (const c of cands) {
      const px = (c.i + 0.5) * DS, py = (c.j + 0.5) * DS;
      const surf = hitWorldSurface(px, py);
      if (surf && surf.dist >= 1.2 && surf.dist <= 6.5) {
        sendNet({
          type: 'registerMarker',
          x: +surf.x.toFixed(2),
          y: +surf.y.toFixed(2),
          z: +surf.z.toFixed(2),
          nx: +surf.nx.toFixed(2),
          nz: +surf.nz.toFixed(2),
          edgeScore: +c.score.toFixed(1),
        });
        lastMarkerSpawn = now;
        break;
      }
    }
  }

  for (const id in net.markers) {
    const m = net.markers[id];
    const dist = Math.hypot(m.x - pos.x, m.z - pos.z);
    if (dist <= MARKER_ACTIVATE_DIST) {
      const camPos = toCam(m.x - pos.x, (m.y || 1.2) - EYE, m.z - pos.z);
      if (camPos[2] > 0.2) {
        const scr = toScreen(camPos);
        if (scr[0] >= 0 && scr[0] <= W && scr[1] >= 0 && scr[1] <= H && !isOccluded3D(m.x, m.z, m.y || 1.2)) {
          if (!m.active) {
            m.active = true;
            sendNet({
              type: 'activateMarker',
              id: m.id,
              playerId: net.id,
              walls: world.walls.filter((w) => w.hits >= 2).map((w) => ({ x1: +w.x1.toFixed(2), z1: +w.z1.toFixed(2), x2: +w.x2.toFixed(2), z2: +w.z2.toFixed(2) })),
            });
            say(`已激活空間標記 [${m.id}] — 座標已回傳伺服器同步`, 2800);
            buzz([30, 40, 30]);
          }
          if (m.lastObsDist) {
            const driftFix = 0.04;
            const dirX = (pos.x - m.x) / (dist || 1), dirZ = (pos.z - m.z) / (dist || 1);
            pos.x = lerp(pos.x, m.x + dirX * dist, driftFix);
            pos.z = lerp(pos.z, m.z + dirZ * dist, driftFix);
          }
          m.lastObsDist = dist;
        }
      }
    }
  }
}

function drawSpatialMarkers(c) {
  const near = 0.15;
  c.save();
  for (const id in net.markers) {
    const m = net.markers[id];
    const dist = Math.hypot(m.x - pos.x, m.z - pos.z);
    if (dist > 3.8) continue;
    if (isOccluded3D(m.x, m.z, m.y || 1.2)) continue;

    const my = m.y || 1.2;
    const centerCam = toCam(m.x - pos.x, my - EYE, m.z - pos.z);
    if (centerCam[2] <= near) continue;

    const nx = m.nx || 0, nz = m.nz || 1;
    const tx = -nz, tz = nx;
    const sz = 0.18;

    const corners = [
      toCam(m.x - tx * sz - pos.x, my + sz - EYE, m.z - tz * sz - pos.z),
      toCam(m.x + tx * sz - pos.x, my + sz - EYE, m.z + tz * sz - pos.z),
      toCam(m.x + tx * sz - pos.x, my - sz - EYE, m.z + tz * sz - pos.z),
      toCam(m.x - tx * sz - pos.x, my - sz - EYE, m.z - tz * sz - pos.z),
    ];
    if (corners.some((pt) => pt[2] <= near)) continue;
    const pts = corners.map(toScreen);

    const isClose = dist <= MARKER_ACTIVATE_DIST;
    c.strokeStyle = m.active ? 'rgba(110,255,225,0.95)' : isClose ? 'rgba(255,255,255,0.9)' : 'rgba(255,255,255,0.45)';
    c.lineWidth = m.active ? 2.2 : 1.5;

    c.beginPath();
    c.moveTo(pts[0][0], pts[0][1]);
    c.lineTo(pts[1][0], pts[1][1]);
    c.lineTo(pts[2][0], pts[2][1]);
    c.lineTo(pts[3][0], pts[3][1]);
    c.closePath();
    c.stroke();

    const centerScr = toScreen(centerCam);
    c.font = '600 12px ui-monospace, Menlo, monospace';
    c.textAlign = 'center';
    c.textBaseline = 'bottom';
    c.fillStyle = m.active ? '#6effe1' : '#ffffff';
    c.fillText(`${m.id} (${dist.toFixed(1)}m)`, centerScr[0], Math.min(pts[0][1], pts[1][1]) - 6);
  }
  c.restore();
}

// ==========================================================
// 鏡頭捕捉人體樣態與手持手機偵測 (連機玩家 = 敵人)
// ==========================================================
let aiModel = null;
let aiBusy = false;
let detectedEnemies = [];

async function loadVisionModel() {
  try {
    if (window.cocoSsd) {
      aiModel = await window.cocoSsd.load({ base: 'lite_mobilenet_v2' });
    }
  } catch (e) {
    console.warn('COCO-SSD 模型載入失敗，將使用連機座標推算敵人:', e);
  }
}

async function runPersonPhoneDetection() {
  if (!running || aiBusy || !aiModel || video.readyState < 2) return;
  aiBusy = true;
  try {
    const preds = await aiModel.detect(scene);
    const persons = preds.filter((p) => p.class === 'person' && p.score > 0.42);
    const phones = preds.filter((p) => p.class === 'cell phone' && p.score > 0.35);

    const visualList = [];
    for (const per of persons) {
      const [bx, by, bw, bh] = per.bbox;
      const sx = bx / DPR, sy = by / DPR, sw = bw / DPR, sh = bh / DPR;

      let holdingPhone = phones.some((ph) => {
        const px = (ph.bbox[0] + ph.bbox[2] / 2) / DPR;
        const py = (ph.bbox[1] + ph.bbox[3] / 2) / DPR;
        return px >= sx - sw * 0.25 && px <= sx + sw * 1.25 && py >= sy - sh * 0.1 && py <= sy + sh * 0.85;
      });

      const footFp = floorPoint(sx + sw / 2, sy + sh);
      const estDist = footFp ? Math.min(15, footFp.d) : Math.max(1.2, (1.7 * FPX) / Math.max(20, sh));
      const r = rayW(sx + sw / 2, sy + sh / 2);
      const rLen = Math.hypot(r[0], r[2]) || 1;
      const wx = footFp ? footFp.x : pos.x + (r[0] / rLen) * estDist;
      const wz = footFp ? footFp.z : pos.z + (r[2] / rLen) * estDist;

      for (const pid in net.players) {
        const np = net.players[pid];
        if (Math.hypot(np.x - wx, np.z - wz) < 2.5) holdingPhone = true;
      }

      if (holdingPhone || Object.keys(net.players).length > 0 || per.score > 0.6) {
        const occluded = isOccluded3D(wx, wz, 1.7);
        visualList.push({
          sx, sy, sw, sh,
          wx, wz, dist: estDist,
          holdingPhone,
          occluded,
          label: holdingPhone ? '連機玩家 [敵人・持手機]' : '偵測人物 [敵人]',
        });
      }
    }
    detectedEnemies = visualList;
  } catch (e) {}
  aiBusy = false;
}

function drawEnemies(c) {
  c.save();
  c.strokeStyle = '#ffffff';
  c.lineWidth = 2.2;
  c.font = '600 12px -apple-system, "PingFang TC", monospace';
  c.fillStyle = '#ffffff';

  const drawnPositions = [];

  for (const en of detectedEnemies) {
    if (en.occluded || isOccluded3D(en.wx, en.wz, 1.7)) continue;
    drawnPositions.push([en.wx, en.wz]);
    c.strokeRect(en.sx, en.sy, en.sw, en.sh);
    c.fillText(`${en.label} ${en.dist.toFixed(1)}m`, en.sx, Math.max(16, en.sy - 6));
  }

  for (const pid in net.players) {
    const p = net.players[pid];
    const dist = Math.hypot(p.x - pos.x, p.z - pos.z);
    if (dist < 0.3 || dist > 22) continue;
    if (drawnPositions.some((dp) => Math.hypot(dp[0] - p.x, dp[1] - p.z) < 1.5)) continue;
    if (isOccluded3D(p.x, p.z, 1.7)) continue;

    const topCam = toCam(p.x - pos.x, 1.75 - EYE, p.z - pos.z);
    const botCam = toCam(p.x - pos.x, 0 - EYE, p.z - pos.z);
    if (topCam[2] <= 0.2 || botCam[2] <= 0.2) continue;

    const topScr = toScreen(topCam);
    const botScr = toScreen(botCam);
    const bh = Math.abs(botScr[1] - topScr[1]);
    const bw = bh * 0.45;
    const bx = topScr[0] - bw / 2;
    const by = Math.min(topScr[1], botScr[1]);

    c.strokeRect(bx, by, bw, bh);
    c.fillText(`敵人 [${p.id}] ${dist.toFixed(1)}m`, bx, Math.max(16, by - 6));
  }
  c.restore();
}

// ==========================================================
// 場景：鏡頭影像 + 虛擬建築（白色線條）
// ==========================================================
function drawVideo() {
  sctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  const vw = video.videoWidth, vh = video.videoHeight;
  if (vw && vh && video.readyState >= 2) {
    const s = Math.max(W / vw, H / vh);
    const dw = vw * s, dh = vh * s;
    sctx.drawImage(video, (W - dw) / 2, (H - dh) / 2, dw, dh);
    FPX = (s * (Math.max(vw, vh) / 2)) / Math.tan((S.fovLong * DEG) / 2);
  } else {
    sctx.fillStyle = '#1b1d22';
    sctx.fillRect(0, 0, W, H);
    FPX = (Math.max(W, H) / 2) / Math.tan((S.fovLong * DEG) / 2);
  }
}

function drawStructures(c) {
  const near = 0.15;
  const now = performance.now();
  c.lineJoin = 'round';
  for (const o of obs) {
    const dx = o.x - pos.x, dz = o.z - pos.z;
    const dist = Math.hypot(dx, dz);
    if (dist > 24) continue;
    const reach = Math.max(o.w, o.d);
    const cc = toCam(dx, -EYE, dz);
    if (cc[2] < -reach && dist > 3) continue;

    let gy = 1, gx = 1;
    if (o.gen || o.fade) {
      const t = clamp01((now - o.born) / (o.gen ? 2600 : 1200));
      const e = (u) => 1 - (1 - u) * (1 - u);
      gx = e(t);
      gy = e((t - 0.2) / 0.8);
      if (o.dying) { const k = 1 - clamp01((now - o.dying) / 900); gx *= k; gy *= k; }
      if (gy <= 0.01 && gx <= 0.01) continue;
    }
    const segs = dist < 9 ? o.segsNear : o.segsFar;
    const alpha = Math.max(0.35, 1 - dist / 24);
    c.strokeStyle = `rgba(255,255,255,${alpha.toFixed(2)})`;
    c.lineWidth = dist < 6 ? 2 : 1.4;
    const cr = Math.cos(o.rot || 0), sr = Math.sin(o.rot || 0);
    const ax = o.gAx === 9 ? 0 : (o.gAx || 0) * o.w / 2;
    const noX = o.gAx === 9;
    const tx = (x, z) => {
      const lx = noX ? x : ax + (x - ax) * gx;
      const lz = noX ? z : z * (o.gAx ? 1 : gx);
      return [lx * cr - lz * sr, lx * sr + lz * cr];
    };
    c.beginPath();
    for (let q = 0; q < segs.length; q++) {
      const sg = segs[q];
      const p1 = tx(sg[0], sg[2]), p2 = tx(sg[3], sg[5]);
      let a = toCam(dx + p1[0], sg[1] * gy - EYE, dz + p1[1]);
      let b = toCam(dx + p2[0], sg[4] * gy - EYE, dz + p2[1]);
      if (a[2] < near && b[2] < near) continue;
      if (a[2] < near) { const t = (near - a[2]) / (b[2] - a[2]); a = [lerp(a[0], b[0], t), lerp(a[1], b[1], t), near]; }
      else if (b[2] < near) { const t = (near - b[2]) / (a[2] - b[2]); b = [lerp(b[0], a[0], t), lerp(b[1], a[1], t), near]; }
      const pa = toScreen(a), pb = toScreen(b);
      c.moveTo(pa[0], pa[1]);
      c.lineTo(pb[0], pb[1]);
    }
    c.stroke();
  }
}

// ==========================================================
// 虛擬建築與移動習慣生成
// ==========================================================
let obs = [];

function buildSegs(w, d, h, kind, withWindows) {
  const segs = [];
  const hx = w / 2, hz = d / 2;
  const L = (x1, y1, z1, x2, y2, z2) => segs.push([x1, y1, z1, x2, y2, z2]);
  const rect = (y, ix) => {
    const ax = hx - (ix || 0), az = hz - (ix || 0);
    L(-ax, y, -az, ax, y, -az); L(ax, y, -az, ax, y, az);
    L(ax, y, az, -ax, y, az); L(-ax, y, az, -ax, y, -az);
  };
  rect(0); rect(h);
  L(-hx, 0, -hz, -hx, h, -hz); L(hx, 0, -hz, hx, h, -hz);
  L(hx, 0, hz, hx, h, hz); L(-hx, 0, hz, -hx, h, hz);

  if (kind === 'pillar') {
    rect(0.25, -0.05); rect(h - 0.3, -0.05);
    return segs;
  }
  if (kind === 'low') {
    rect(h - 0.06, 0.05);
    return segs;
  }

  const plinth = 0.5;
  const ns = Math.max(1, Math.round((h - plinth) / 1.0));
  const storey = (h - plinth) / ns;
  rect(plinth);
  for (let i = 1; i < ns; i++) rect(plinth + i * storey);
  rect(h - 0.12);
  const longX = w >= d;
  const len = longX ? w : d;
  const nb = Math.max(1, Math.round(len / 1.4));
  const cols = [];
  for (let i = 0; i <= nb; i++) cols.push(-len / 2 + (i * len) / nb);
  for (const c0 of cols) {
    if (longX) { L(c0, 0, -hz, c0, h, -hz); L(c0, 0, hz, c0, h, hz); }
    else { L(-hx, 0, c0, -hx, h, c0); L(hx, 0, c0, hx, h, c0); }
  }
  if (withWindows) {
    for (let i = 0; i < nb; i++) {
      const a = cols[i] + 0.18, b = cols[i + 1] - 0.18;
      for (let s = 0; s < ns; s++) {
        const y0 = plinth + s * storey + 0.22, y1 = plinth + (s + 1) * storey - 0.22;
        for (const sd of [-1, 1]) {
          if (longX) {
            const z = sd * hz;
            L(a, y0, z, b, y0, z); L(b, y0, z, b, y1, z); L(b, y1, z, a, y1, z); L(a, y1, z, a, y0, z);
          } else {
            const x = sd * hx;
            L(x, y0, a, x, y0, b); L(x, y0, b, x, y1, b); L(x, y1, b, x, y1, a); L(x, y1, a, x, y0, a);
          }
        }
      }
    }
  }
  return segs;
}

function makeObs(x, z, w, d, h, kind, rot, gAx) {
  return {
    x, z, w, d, h, kind, rot: rot || 0, gAx: gAx || 0,
    gen: false, fade: false, gid: 0, born: 0, dying: 0,
    segsFar: buildSegs(w, d, h, kind, false),
    segsNear: buildSegs(w, d, h, kind, true),
  };
}

const habit = { open: 0, hug: 0, fast: 0, still: 0, turn: 0, hx: 0, hz: 0 };
let genCount = 0, genGid = 0, genHistory = [];
let genLog = { text: '', until: 0 };
let nextGenAt = 0;
const MAX_GEN = 10;
const GEN_INTERVAL = 6000;
const GEN_FIRST = 8000;

function resetHabit() {
  habit.open = habit.hug = habit.fast = habit.still = habit.turn = 0;
  habit.hx = habit.hz = 0;
  genCount = 0; genGid = 0; genHistory = [];
  genLog = { text: '', until: 0 };
}

function updateHabit(dt) {
  const k = Math.min(1, dt / 10);
  const prox = Math.max(fov.prox.l, fov.prox.r);
  const moving = moveSpeed > 0.25;
  habit.open += ((prox < 0.12 ? 1 : 0) - habit.open) * k;
  habit.hug += ((prox > 0.45 ? 1 : 0) - habit.hug) * k;
  habit.still += ((moving ? 0 : 1) - habit.still) * k;
  habit.fast += ((moveSpeed > 1.3 ? 1 : 0) - habit.fast) * k;
  let dyaw = Math.atan2(headX, -headZ) - prevYaw;
  prevYaw = Math.atan2(headX, -headZ);
  if (dyaw > Math.PI) dyaw -= 2 * Math.PI;
  if (dyaw < -Math.PI) dyaw += 2 * Math.PI;
  habit.turn += (Math.max(-1, Math.min(1, dyaw * 12)) - habit.turn) * Math.min(1, dt / 5);
  if (moving) {
    habit.hx += (headX - habit.hx) * Math.min(1, dt * 1.2);
    habit.hz += (headZ - habit.hz) * Math.min(1, dt * 1.2);
  }
}

const GEN_INFO = {
  cover: { name: '掩體群', why: '你多在空曠處' },
  alley: { name: '巷弄', why: '你習慣貼牆走' },
  wall: { name: '長牆', why: '你走得很快' },
  room: { name: '小房間', why: '你常停下來' },
};

function roomSpecs(cx, cz, w, d, h, doors) {
  const T = 0.25, DW2 = 1.0, out = [];
  const hw = w / 2, hd = d / 2;
  for (const pr of [['n', cz - hd], ['s', cz + hd]]) {
    if (doors.includes(pr[0])) {
      const seg = (w - DW2) / 2;
      out.push([cx - DW2 / 2 - seg / 2, pr[1], seg, T, h], [cx + DW2 / 2 + seg / 2, pr[1], seg, T, h]);
    } else out.push([cx, pr[1], w, T, h]);
  }
  const len = d - T;
  for (const pr of [['w', cx - hw], ['e', cx + hw]]) {
    if (doors.includes(pr[0])) {
      const seg = (len - DW2) / 2;
      out.push([pr[1], cz - DW2 / 2 - seg / 2, T, seg, h], [pr[1], cz + DW2 / 2 + seg / 2, T, seg, h]);
    } else out.push([pr[1], cz, T, len, h]);
  }
  return out;
}

function specObj(sp) { return { x: sp[0], z: sp[1], w: sp[2], d: sp[3], rot: sp[6] || 0 }; }

function toLocal(o, px, pz) {
  const r = o.rot || 0, c = Math.cos(r), s = Math.sin(r), dx = px - o.x, dz = pz - o.z;
  return [dx * c + dz * s, -dx * s + dz * c];
}
function distPointBox(px, pz, o) {
  const l = toLocal(o, px, pz);
  return Math.hypot(Math.max(Math.abs(l[0]) - o.w / 2, 0), Math.max(Math.abs(l[1]) - o.d / 2, 0));
}
function pts9(o) {
  const r = o.rot || 0, c = Math.cos(r), s = Math.sin(r), out = [];
  for (const a of [-1, 0, 1]) for (const b of [-1, 0, 1]) {
    const lx = (a * o.w) / 2, lz = (b * o.d) / 2;
    out.push([o.x + lx * c - lz * s, o.z + lx * s + lz * c]);
  }
  return out;
}
function boxesOverlap(a, b, pad) {
  for (const p of pts9(a)) if (distPointBox(p[0], p[1], b) < pad) return true;
  for (const p of pts9(b)) if (distPointBox(p[0], p[1], a) < pad) return true;
  return false;
}
function segCross(ax, az, bx, bz, cx, cz, dx, dz) {
  const d1x = bx - ax, d1z = bz - az, d2x = dx - cx, d2z = dz - cz;
  const den = d1x * d2z - d1z * d2x;
  if (Math.abs(den) < 1e-9) return false;
  const t = ((cx - ax) * d2z - (cz - az) * d2x) / den;
  const u = ((cx - ax) * d1z - (cz - az) * d1x) / den;
  return t > 0.02 && t < 0.98 && u > 0.02 && u < 0.98;
}

function wallSpec(x1, z1, x2, z2, h, T, kind) {
  return [(x1 + x2) / 2, (z1 + z2) / 2, Math.hypot(x2 - x1, z2 - z1), T || 0.25, h || 2.6, kind || 'wall', Math.atan2(z2 - z1, x2 - x1), -1];
}

function specsValid(specs, ign) {
  for (const sp of specs) {
    const o = specObj(sp);
    if (distPointBox(pos.x, pos.z, o) < 1.6) return false;
    for (const q of obs) if (!q.dying && boxesOverlap(o, q, 0.35)) return false;
    const long = o.w >= o.d, hl = (long ? o.w : o.d) / 2;
    const c = Math.cos(o.rot), s = Math.sin(o.rot);
    const ex = long ? c : -s, ez = long ? s : c;
    const ax = o.x - ex * hl, az = o.z - ez * hl, bx = o.x + ex * hl, bz = o.z + ez * hl;
    for (const w of world.walls) {
      if (w.hits < 2 || (ign && ign.has(w.id))) continue;
      if (segCross(ax, az, bx, bz, w.x1, w.z1, w.x2, w.z2)) return false;
    }
    for (const e of world.ents) {
      if (e.hits < 3 || (ign && ign.has(e.id))) continue;
      if (distPointBox(e.x, e.z, o) < 0.2) return false;
    }
  }
  return true;
}

function aScore(x, z, used) {
  const dx = x - pos.x, dz = z - pos.z, d = Math.hypot(dx, dz);
  const ang = Math.abs(wrapPi(Math.atan2(dx, -dz) - Math.atan2(headX, -headZ)));
  return Math.cos(Math.min(ang, Math.PI)) * 0.6 + 1 / (1 + d / 4) + rand(0, 0.3) - 0.5 * (used || 0);
}

function sideOf(w) {
  const len = Math.hypot(w.x2 - w.x1, w.z2 - w.z1) || 1;
  const ux = (w.x2 - w.x1) / len, uz = (w.z2 - w.z1) / len;
  let nx = -uz, nz = ux;
  const mx = (w.x1 + w.x2) / 2, mz = (w.z1 + w.z2) / 2;
  if ((pos.x - mx) * nx + (pos.z - mz) * nz < 0) { nx = -nx; nz = -nz; }
  return { ux, uz, nx, nz, len, mx, mz };
}

function wallsNear(x, z, r) {
  const ids = new Set();
  for (const w of world.walls) {
    if (Math.hypot(w.x1 - x, w.z1 - z) < r || Math.hypot(w.x2 - x, w.z2 - z) < r) ids.add(w.id);
  }
  return ids;
}

function buildRoom(e) {
  const L = 3.0;
  let arms = e.dirs.slice();
  let u = null, v = null;
  if (arms.length >= 2) {
    let best = null;
    for (let i = 0; i < arms.length; i++) for (let j = i + 1; j < arms.length; j++) {
      const d = Math.abs(wrapPi(arms[i] - arms[j]));
      if (d > 50 * DEG && d < 130 * DEG && (!best || Math.abs(d - Math.PI / 2) < Math.abs(best[2] - Math.PI / 2))) best = [arms[i], arms[j], d];
    }
    if (best) { u = best[0]; v = best[1]; }
  }
  if (u === null && arms.length >= 1) {
    u = arms[0];
    const cand = [u + Math.PI / 2, u - Math.PI / 2];
    let pick = null;
    for (const a of cand) {
      const px = e.x + (Math.cos(u) + Math.cos(a)) * L * 0.5, pz = e.z + (Math.sin(u) + Math.sin(a)) * L * 0.5;
      if (isFree(px, pz)) { pick = a; break; }
    }
    if (pick === null) return null;
    v = pick;
  }
  if (u === null) return null;
  const ux = Math.cos(u), uz = Math.sin(u), vx = Math.cos(v), vz = Math.sin(v);
  let freeN = 0;
  for (const f of [0.35, 0.5, 0.65]) if (isFree(e.x + (ux + vx) * L * f, e.z + (uz + vz) * L * f)) freeN++;
  if (!freeN) return null;
  const P1 = [e.x + ux * L, e.z + uz * L], P2 = [P1[0] + vx * L, P1[1] + vz * L];
  const Q1 = [e.x + vx * L, e.z + vz * L];
  const h1 = L / 2 - 0.5;
  const specs = [
    wallSpec(P1[0], P1[1], P2[0], P2[1]),
    wallSpec(Q1[0], Q1[1], Q1[0] + ux * h1, Q1[1] + uz * h1),
    wallSpec(P2[0], P2[1], P2[0] - ux * h1, P2[1] - uz * h1),
  ];
  const ign = wallsNear(e.x, e.z, 0.7); ign.add(e.id);
  return { specs, ign, label: '小房間・以轉角為一角' };
}

function buildAlley(w) {
  const s = sideOf(w);
  const Ln = Math.min(s.len, 5.5);
  const specs = [[s.mx + s.nx * 1.9, s.mz + s.nz * 1.9, Ln, 0.25, 2.6, 'wall', Math.atan2(s.uz, s.ux), -1]];
  const ign = new Set([w.id]);
  return { specs, ign, label: '巷弄・平行於真牆' };
}

function buildExtend(a) {
  const L = 2.6;
  let dx = a.dx, dz = a.dz, label = '長牆・沿牆延伸';
  if (Math.random() < 0.4) {
    const nx = -dz, nz = dx;
    const side = ((pos.x - a.x) * nx + (pos.z - a.z) * nz) >= 0 ? 1 : -1;
    dx = nx * side; dz = nz * side;
    label = '長牆・在端點轉折';
  }
  const specs = [wallSpec(a.x, a.z, a.x + dx * L, a.z + dz * L)];
  return { specs, ign: wallsNear(a.x, a.z, 0.7), label };
}

function buildCoverCol(e) {
  const cw = Math.max(0.3, e.w);
  const specs = [[e.x, e.z, cw, cw, 3.0, 'pillar', 0, 9]];
  const base = Math.atan2(pos.z - e.z, pos.x - e.x);
  for (const off of [0.9, -0.9]) {
    const a = base + off;
    specs.push([e.x + Math.cos(a) * 1.3, e.z + Math.sin(a) * 1.3, 0.9, 0.9, 1.05, 'low', a + Math.PI / 2, 0]);
  }
  return { specs, ign: new Set([e.id]), label: '掩體群・柱子往上長一層' };
}

function buildCoverWall(w) {
  const s = sideOf(w);
  const specs = [];
  for (const t of [0.3, 0.7]) {
    specs.push([w.x1 + (w.x2 - w.x1) * t + s.nx * 0.65, w.z1 + (w.z2 - w.z1) * t + s.nz * 0.65, 1.2, 0.6, 1.05, 'low', Math.atan2(s.uz, s.ux), 0]);
  }
  return { specs, ign: new Set([w.id]), label: '掩體群・貼著真牆' };
}

function candidates(type) {
  const out = [];
  const okd = (x, z) => { const d = Math.hypot(x - pos.x, z - pos.z); return d >= 1.8 && d <= 11; };
  const ents = world.ents.filter((e) => e.hits >= 3 && e.used < 3 && okd(e.x, e.z));
  const walls = world.walls.filter((w) => w.hits >= 2 && w.used < 2 && Math.hypot(w.x2 - w.x1, w.z2 - w.z1) >= 1.2 && okd((w.x1 + w.x2) / 2, (w.z1 + w.z2) / 2));
  if (type === 'room') {
    for (const e of ents) if (e.type === 'corner') out.push({ ref: e, s: aScore(e.x, e.z, e.used), build: () => buildRoom(e) });
  } else if (type === 'alley') {
    for (const w of walls) out.push({ ref: w, s: aScore((w.x1 + w.x2) / 2, (w.z1 + w.z2) / 2, w.used), build: () => buildAlley(w) });
  } else if (type === 'wall') {
    for (const w of world.walls) {
      if (w.hits < 2 || w.used >= 3) continue;
      const len = Math.hypot(w.x2 - w.x1, w.z2 - w.z1);
      if (len < 1.2) continue;
      const ux = (w.x2 - w.x1) / len, uz = (w.z2 - w.z1) / len;
      for (const sg of [1, -1]) {
        const x = sg > 0 ? w.x2 : w.x1, z = sg > 0 ? w.z2 : w.z1;
        if (!okd(x, z)) continue;
        out.push({ ref: w, s: aScore(x, z, w.used), build: () => buildExtend({ x, z, dx: ux * sg, dz: uz * sg }) });
      }
    }
    for (const o of obs) {
      if (!o.gen || o.dying || o.kind !== 'wall' || o.grown) continue;
      const c = Math.cos(o.rot || 0), s = Math.sin(o.rot || 0);
      for (const sg of [1, -1]) {
        const x = o.x + c * sg * o.w / 2, z = o.z + s * sg * o.w / 2;
        if (!okd(x, z)) continue;
        out.push({ ref: o, s: aScore(x, z, 0) - 0.15, build: () => buildExtend({ x, z, dx: c * sg, dz: s * sg }) });
      }
    }
  } else if (type === 'cover') {
    for (const e of ents) if (e.type === 'column') out.push({ ref: e, s: aScore(e.x, e.z, e.used) + 0.2, build: () => buildCoverCol(e) });
    for (const w of walls) out.push({ ref: w, s: aScore((w.x1 + w.x2) / 2, (w.z1 + w.z2) / 2, w.used) - 0.2, build: () => buildCoverWall(w) });
  }
  out.sort((p, q) => q.s - p.s);
  return out;
}

let genLabel = '';
function tryAttach(type) {
  const list = candidates(type);
  for (let k = 0; k < Math.min(8, list.length); k++) {
    const c = list[k], b = c.build();
    if (!b || !specsValid(b.specs, b.ign)) continue;
    genGid++;
    const now = performance.now();
    for (const sp of b.specs) {
      const o = makeObs(sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], sp[6], sp[7]);
      o.gen = true; o.gid = genGid; o.born = now;
      obs.push(o);
    }
    if (c.ref.used !== undefined) c.ref.used++;
    else c.ref.grown = true;
    genLabel = b.label;
    return true;
  }
  return false;
}

function genSpecs(type, cx, cz, alongX) {
  const loc = [];
  if (type === 'room') {
    return roomSpecs(cx, cz, 3.6, 3.6, 2.6, alongX ? 'ew' : 'ns').map((s) => [s[0], s[1], s[2], s[3], s[4], 'wall', 0, 0]);
  }
  if (type === 'cover') {
    loc.push([-1.1, -0.8, 0.9, 0.9, 1.05, 'low'], [0.9, -0.9, 0.9, 0.9, 1.15, 'low'],
      [-0.8, 1.0, 0.9, 0.9, 1.0, 'low'], [1.2, 0.8, 0.9, 0.9, 0.95, 'low'], [0, 0, 0.4, 0.4, 2.6, 'pillar']);
  } else if (type === 'alley') {
    loc.push([0, -0.925, 5, 0.25, 2.6, 'wall'], [0, 0.925, 5, 0.25, 2.6, 'wall']);
  } else if (type === 'wall') {
    loc.push([-1.7, 0, 2.2, 0.25, 2.6, 'wall'], [1.7, 0, 2.2, 0.25, 2.6, 'wall']);
  }
  return loc.map((q) => (alongX
    ? [cx + q[0], cz + q[1], q[2], q[3], q[4], q[5], 0, 0]
    : [cx + q[1], cz + q[0], q[3], q[2], q[4], q[5], 0, 0]));
}

function tryPlaceFree(type) {
  let hx = habit.hx, hz = habit.hz;
  let len = Math.hypot(hx, hz);
  if (len < 0.25) { hx = headX; hz = headZ; len = 1; }
  hx /= len; hz /= len;
  const a = habit.turn * 0.5;
  const dirx = hx * Math.cos(a) - hz * Math.sin(a);
  const dirz = hx * Math.sin(a) + hz * Math.cos(a);
  const alongHeading = Math.abs(hx) > Math.abs(hz);
  for (let k = 0; k < 30; k++) {
    const spread = (k / 30) * Math.PI * 0.9;
    const ta = rand(-spread, spread);
    const far = 3.6 + rand(0, 2.2 + k * 0.12);
    const ux = dirx * Math.cos(ta) - dirz * Math.sin(ta);
    const uz = dirx * Math.sin(ta) + dirz * Math.cos(ta);
    const cx = pos.x + ux * far + rand(-0.4, 0.4);
    const cz = pos.z + uz * far + rand(-0.4, 0.4);
    let alongX;
    if (type === 'alley') alongX = alongHeading;
    else if (type === 'wall') alongX = !alongHeading;
    else if (type === 'room') alongX = alongHeading;
    else alongX = Math.random() < 0.5;
    const specs = genSpecs(type, cx, cz, alongX);
    if (!specsValid(specs, null)) continue;
    genGid++;
    const now = performance.now();
    for (const sp of specs) {
      const o = makeObs(sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], 0, 0);
      o.gen = true; o.gid = genGid; o.born = now;
      obs.push(o);
    }
    genLabel = '自由生成（未附著）';
    return true;
  }
  return false;
}

function tryPlace(type) { return S.freeGen ? tryPlaceFree(type) : tryAttach(type); }

function generateByHabit() {
  const scores = {
    cover: habit.open + 0.12,
    alley: habit.hug * 1.2 + 0.05,
    wall: habit.fast * 1.2 + 0.05,
    room: habit.still + 0.1,
  };
  const recent = genHistory.slice(-3);
  const order = Object.keys(scores)
    .map((k) => ({ k, v: scores[k] - 0.2 * recent.filter((r) => r === k).length + rand(0, 0.12) }))
    .sort((p1, p2) => p2.v - p1.v);
  for (const it of order) {
    if (tryPlace(it.k)) {
      genCount++;
      genHistory.push(it.k);
      const info = GEN_INFO[it.k];
      genLog = { text: `生成 #${genCount}：${info.name}（${info.why}）→ ${genLabel}`, until: performance.now() + 6000 };
      pruneOldGen();
      return true;
    }
  }
  return false;
}

function pruneOldGen() {
  const alive = {};
  for (const o of obs) if (o.gen && !o.dying) (alive[o.gid] = alive[o.gid] || []).push(o);
  const ids = Object.keys(alive).map(Number).sort((p1, p2) => p1 - p2);
  const now = performance.now();
  for (const id of ids) {
    if (ids.length <= MAX_GEN) break;
    const far = alive[id].every((o) => Math.hypot(o.x - pos.x, o.z - pos.z) > 10);
    if (!far) continue;
    for (const o of alive[id]) o.dying = now;
    ids.splice(ids.indexOf(id), 1);
  }
}

function updateGeneration(now) {
  if (obs.some((o) => o.dying && now - o.dying > 900)) obs = obs.filter((o) => !(o.dying && now - o.dying > 900));
  if (now >= nextGenAt) {
    const ok = generateByHabit();
    nextGenAt = now + (ok ? GEN_INTERVAL : 1800);
  }
}

// ==========================================================
// 後台空間偵測管線（全部移至後台運算，不在 UI 繪製偵測線）
// ==========================================================
const DS = 4;
let DW = 0, DH = 0;
const dm = document.createElement('canvas');
const dmctx = dm.getContext('2d', { willReadFrequently: true });
const segCv = document.createElement('canvas');
const segCtx = segCv.getContext('2d');
let dgray = new Float32Array(1), dgx = new Float32Array(1), dgy = new Float32Array(1), dmag = new Float32Array(1);
let dR = new Float32Array(1), dlab = new Uint8Array(1), dqueue = new Int32Array(1), dpx = null;
let segImg = null;

const det = {
  feats: [], vlines: [], ms: 0, passes: 0, skipped: 0,
  floorSeed: null, floorPct: 0, wallPct: 0, ceilPct: 0,
  depth: '後台單眼幾何 + Canny 邊緣定位', lastRun: 0, thr: 12,
};
const world = { ents: [], walls: [], free: new Map(), nid: 1, pass: 0 };
let angSpeed = 0;
let prevF = null;

function detectResize() {
  DW = Math.ceil(W / DS);
  DH = Math.ceil(H / DS);
  dm.width = DW; dm.height = DH;
  segCv.width = DW; segCv.height = DH;
  const n = DW * DH;
  dgray = new Float32Array(n); dgx = new Float32Array(n); dgy = new Float32Array(n);
  dmag = new Float32Array(n); dR = new Float32Array(n);
  dlab = new Uint8Array(n); dqueue = new Int32Array(n);
  segImg = segCtx.createImageData(DW, DH);
}

function rayW(px, py) {
  const cx = (px - W / 2) / FPX, cy = -(py - H / 2) / FPX;
  return [
    cx * B.sr[0] + cy * B.su[0] + B.F[0],
    cx * B.sr[1] + cy * B.su[1] + B.F[1],
    cx * B.sr[2] + cy * B.su[2] + B.F[2],
  ];
}
function rayY(px, py) { return ((px - W / 2) / FPX) * B.sr[1] - ((py - H / 2) / FPX) * B.su[1] + B.F[1]; }
function floorPoint(px, py) {
  const r = rayW(px, py);
  if (r[1] > -0.035) return null;
  const t = EYE / -r[1];
  return { x: pos.x + r[0] * t, z: pos.z + r[2] * t, d: Math.hypot(r[0], r[2]) * t };
}
function vertDir(px, py) {
  const cx = (px - W / 2) / FPX, cy = -(py - H / 2) / FPX;
  const u0 = B.sr[1], u1 = B.su[1], u2 = B.F[1];
  let sx = FPX * (u0 - cx * u2), sy = -FPX * (u1 - cy * u2);
  const l = Math.hypot(sx, sy) || 1;
  return [sx / l, sy / l];
}

function trackAng(dt) {
  if (prevF) {
    const d = Math.max(-1, Math.min(1, dot3(prevF, B.F)));
    angSpeed += (Math.acos(d) / Math.max(dt, 1e-3) - angSpeed) * Math.min(1, dt * 8);
  }
  prevF = [B.F[0], B.F[1], B.F[2]];
}

function detGradients() {
  dpx = dmctx.getImageData(0, 0, DW, DH).data;
  const n = DW * DH;
  for (let k = 0; k < n; k++) dgray[k] = 0.299 * dpx[4 * k] + 0.587 * dpx[4 * k + 1] + 0.114 * dpx[4 * k + 2];
  for (let j = 1; j < DH - 1; j++) {
    for (let i = 1; i < DW - 1; i++) {
      const k = j * DW + i;
      const gx = (-dgray[k - DW - 1] + dgray[k - DW + 1] - 2 * dgray[k - 1] + 2 * dgray[k + 1] - dgray[k + DW - 1] + dgray[k + DW + 1]) / 4;
      const gy = (-dgray[k - DW - 1] - 2 * dgray[k - DW] - dgray[k - DW + 1] + dgray[k + DW - 1] + 2 * dgray[k + DW] + dgray[k + DW + 1]) / 4;
      dgx[k] = gx; dgy[k] = gy; dmag[k] = Math.sqrt(gx * gx + gy * gy);
    }
  }
}

function detHarris() {
  let mx = 0;
  for (let j = 2; j < DH - 2; j++) {
    for (let i = 2; i < DW - 2; i++) {
      let sxx = 0, syy = 0, sxy = 0;
      for (let b = -1; b <= 1; b++) {
        for (let a = -1; a <= 1; a++) {
          const k = (j + b) * DW + i + a;
          const ix = dgx[k], iy = dgy[k];
          sxx += ix * ix; syy += iy * iy; sxy += ix * iy;
        }
      }
      const r = sxx * syy - sxy * sxy - 0.05 * (sxx + syy) * (sxx + syy);
      dR[j * DW + i] = r;
      if (r > mx) mx = r;
    }
  }
  const thr = Math.max(mx * 0.06, 1);
  const out = [];
  for (let j = 3; j < DH - 3; j++) {
    for (let i = 3; i < DW - 3; i++) {
      const k = j * DW + i, r = dR[k];
      if (r < thr) continue;
      let ok = true;
      for (let b = -2; b <= 2 && ok; b++) for (let a = -2; a <= 2; a++) if ((a || b) && dR[k + b * DW + a] > r) { ok = false; break; }
      if (ok) out.push({ i, j, r });
    }
  }
  out.sort((p, q) => q.r - p.r);
  det.feats = out.slice(0, 90);
  det.harrisMax = mx;
}

function detVertical(thr) {
  const NB = 720;
  const votes = new Float32Array(NB);
  const cb = [], ck = [];
  for (let j = 1; j < DH - 1; j++) {
    for (let i = 1; i < DW - 1; i++) {
      const k = j * DW + i, m = dmag[k];
      if (m < thr) continue;
      const px = (i + 0.5) * DS, py = (j + 0.5) * DS;
      const vv = vertDir(px, py);
      const tx = -dgy[k] / m, ty = dgx[k] / m;
      if (Math.abs(tx * vv[0] + ty * vv[1]) < 0.965) continue;
      const r = rayW(px, py);
      const az = Math.atan2(r[0], -r[2]);
      const bin = Math.min(NB - 1, Math.floor(((az + Math.PI) / (2 * Math.PI)) * NB));
      votes[bin]++;
      cb.push(bin); ck.push(k);
    }
  }
  const sv = new Float32Array(NB);
  for (let b = 0; b < NB; b++) sv[b] = votes[(b + NB - 1) % NB] + votes[b] + votes[(b + 1) % NB];
  const minV = Math.max(14, 0.08 * DH);
  const peaks = [];
  for (let b = 0; b < NB; b++) {
    if (sv[b] < minV) continue;
    let top = true;
    for (let q = -3; q <= 3; q++) if (sv[(b + q + NB) % NB] > sv[b] || (q < 0 && sv[(b + q + NB) % NB] === sv[b])) { top = false; break; }
    if (top) peaks.push(b);
  }
  const lines = [];
  for (const pb of peaks) {
    const rowN = new Uint16Array(DH), rowX = new Float32Array(DH);
    for (let q = 0; q < cb.length; q++) {
      const d = Math.abs(((cb[q] - pb + NB + NB / 2) % NB) - NB / 2);
      if (d > 1) continue;
      const k = ck[q], j = (k / DW) | 0, i = k - j * DW;
      rowN[j]++; rowX[j] += i;
    }
    let best = null, start = -1, last = -9;
    const consider = () => { if (start >= 0 && (!best || last - start > best[1] - best[0])) best = [start, last]; };
    for (let j = 0; j < DH; j++) {
      if (!rowN[j]) continue;
      if (start < 0 || j - last > 3) { consider(); start = j; }
      last = j;
    }
    consider();
    if (!best) continue;
    const jt = best[0], jb = best[1];
    if (jb - jt < Math.max(14, 0.08 * DH)) continue;
    const avgX = (j0, j1) => {
      let s = 0, n = 0;
      for (let j = j0; j <= j1; j++) if (rowN[j]) { s += rowX[j]; n += rowN[j]; }
      return n ? (s / n + 0.5) * DS : null;
    };
    const xb = avgX(Math.max(jt, jb - 4), jb), xt = avgX(jt, Math.min(jb, jt + 4));
    if (xb == null || xt == null) continue;
    const yb = (jb + 1) * DS, yt = jt * DS;
    const ib = Math.round(xb / DS), clipped = jb >= DH - 5;
    const az = ((pb + 0.5) / NB) * 2 * Math.PI - Math.PI;
    const fp = clipped ? null : floorPoint(xb, yb);
    lines.push({ xb, yb, xt, yt, ib, jb, jt, az, fp, clipped, kind: 'line' });
  }
  det.vlines = lines;
  return lines;
}

function junctionSide(ib, jb, sgn, thr) {
  const vv = vertDir(ib * DS, jb * DS);
  let C2 = 0, S2 = 0, Wt = 0, n = 0;
  for (let j = Math.max(1, jb - 7); j <= Math.min(DH - 2, jb + 5); j++) {
    for (let s = 3; s <= 18; s++) {
      const i = ib + sgn * s;
      if (i < 1 || i >= DW - 1) break;
      const k = j * DW + i, m = dmag[k];
      if (m < thr) continue;
      const tx = -dgy[k] / m, ty = dgx[k] / m;
      if (Math.abs(tx * vv[0] + ty * vv[1]) > 0.85) continue;
      const phi = Math.atan2(ty, tx);
      C2 += m * Math.cos(2 * phi); S2 += m * Math.sin(2 * phi); Wt += m; n++;
    }
  }
  if (n < 6) return null;
  if (Math.hypot(C2, S2) / Wt < 0.55) return null;
  const phi = 0.5 * Math.atan2(S2, C2);
  if (Math.abs(Math.cos(phi)) < 0.25) return null;
  return { phi, n };
}

function marchLine(i0, j0, dx, dy, thr) {
  const alongX = Math.abs(dx) >= Math.abs(dy);
  const step = alongX ? Math.sign(dx) : Math.sign(dy);
  const slope = alongX ? dy / dx : dx / dy;
  let x = i0, y = j0, miss = 0, gx = x, gy = y;
  for (let s = 0; s < DW * 2; s++) {
    if (alongX) { x += step; y += slope * step; } else { y += step; x += slope * step; }
    const ii = Math.round(x), jj = Math.round(y);
    if (ii < 3 || ii >= DW - 3 || jj < 3 || jj >= DH - 3) break;
    let bm = 0, bo = 0;
    for (let o = -2; o <= 2; o++) {
      const m = alongX ? dmag[(jj + o) * DW + ii] : dmag[jj * DW + ii + o];
      if (m > bm) { bm = m; bo = o; }
    }
    if (bm >= thr * 0.6) {
      miss = 0;
      if (alongX) y += bo * 0.35; else x += bo * 0.35;
      gx = x; gy = y;
    } else if (++miss > 4) break;
  }
  return [gx, gy];
}

function harrisNear(i, j) {
  let m = 0;
  for (let b = -3; b <= 3; b++) for (let a = -3; a <= 3; a++) {
    const ii = i + a, jj = j + b;
    if (ii < 2 || jj < 2 || ii >= DW - 2 || jj >= DH - 2) continue;
    if (dR[jj * DW + ii] > m) m = dR[jj * DW + ii];
  }
  return m > (det.harrisMax || 1) * 0.06;
}

function wrapPi(a) { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; }

function classifyLines(lines, thr) {
  const obsE = [], obsW = [];
  det.sides = [];
  const ranged = lines.filter((l) => l.fp && l.fp.d >= 0.6 && l.fp.d <= 10);
  const used = new Set();
  for (let a = 0; a < ranged.length; a++) {
    for (let b = a + 1; b < ranged.length; b++) {
      if (used.has(a) || used.has(b)) continue;
      const A = ranged[a], Bn = ranged[b];
      const da = Math.abs(wrapPi(A.az - Bn.az));
      const r = (A.fp.d + Bn.fp.d) / 2;
      const wd = r * da;
      if (wd < 0.1 || wd > 0.9 || Math.abs(A.fp.d - Bn.fp.d) > 0.45) continue;
      used.add(a); used.add(b);
      obsE.push({ type: 'column', x: (A.fp.x + Bn.fp.x) / 2, z: (A.fp.z + Bn.fp.z) / 2, w: wd, dirs: [] });
    }
  }
  ranged.forEach((L, idx) => {
    if (used.has(idx)) return;
    const jb = Math.min(DH - 2, L.jb + 1);
    const sides = [-1, 1].map((sg) => {
      const js = junctionSide(L.ib, jb, sg, thr);
      if (!js) return null;
      let dx = Math.cos(js.phi), dy = Math.sin(js.phi);
      if (sg * dx < 0) { dx = -dx; dy = -dy; }
      const e = marchLine(L.ib, jb, dx, dy, thr);
      const fe = floorPoint((e[0] + 0.5) * DS, (e[1] + 0.5) * DS);
      return { sg, phi: js.phi, end: fe };
    });
    let type = 'edge';
    const dirs = [];
    const [sl, sr] = sides;
    if (sl && sr) {
      let dphi = Math.abs(sl.phi - sr.phi) % Math.PI;
      dphi = Math.min(dphi, Math.PI - dphi);
      if (dphi > 14 * DEG || (dphi > 8 * DEG && harrisNear(L.ib, jb))) type = 'corner';
    }
    for (const sd of sides) {
      if (!sd || !sd.end) continue;
      const len = Math.hypot(sd.end.x - L.fp.x, sd.end.z - L.fp.z);
      if (len < 0.8 || len > 12) continue;
      dirs.push(Math.atan2(sd.end.z - L.fp.z, sd.end.x - L.fp.x));
      obsW.push({ x1: L.fp.x, z1: L.fp.z, x2: sd.end.x, z2: sd.end.z });
    }
    obsE.push({ type, x: L.fp.x, z: L.fp.z, w: 0.2, dirs });
  });
  return { obsE, obsW };
}

function detSegment(thr, now) {
  const n = DW * DH;
  dlab.fill(0);
  const smp = [];
  for (let j = Math.floor(DH * 0.88); j < DH - 1; j++) {
    for (let i = Math.floor(DW * 0.3); i < Math.floor(DW * 0.7); i++) {
      const k = j * DW + i;
      if (dmag[k] > thr * 2) continue;
      if (rayY((i + 0.5) * DS, (j + 0.5) * DS) > -0.15) continue;
      smp.push([dpx[4 * k], dpx[4 * k + 1], dpx[4 * k + 2]]);
    }
  }
  if (smp.length > 20) {
    let lo = smp[0], hi = smp[0];
    const lum = (c) => c[0] + c[1] + c[2];
    for (const c of smp) { if (lum(c) < lum(lo)) lo = c; if (lum(c) > lum(hi)) hi = c; }
    let cs = [lo.slice(), hi.slice()];
    for (let it = 0; it < 3; it++) {
      const acc = [[0, 0, 0, 0], [0, 0, 0, 0]];
      for (const c of smp) {
        const d0 = Math.abs(c[0] - cs[0][0]) + Math.abs(c[1] - cs[0][1]) + Math.abs(c[2] - cs[0][2]);
        const d1 = Math.abs(c[0] - cs[1][0]) + Math.abs(c[1] - cs[1][1]) + Math.abs(c[2] - cs[1][2]);
        const q = d0 <= d1 ? 0 : 1;
        acc[q][0] += c[0]; acc[q][1] += c[1]; acc[q][2] += c[2]; acc[q][3]++;
      }
      for (let q = 0; q < 2; q++) if (acc[q][3] > 5) cs[q] = [acc[q][0] / acc[q][3], acc[q][1] / acc[q][3], acc[q][2] / acc[q][3]];
    }
    det.floorSeed = det.floorSeed ? cs.map((c, q) => c.map((v, m) => v + (det.floorSeed[q][m] - v) * 0.6)) : cs;
  }
  const seed = det.floorSeed;
  const floorLike = new Uint8Array(n);
  const ceilLike = new Uint8Array(n);
  for (let j = 1; j < DH - 1; j++) {
    for (let i = 1; i < DW - 1; i++) {
      const k = j * DW + i;
      const ry = rayY((i + 0.5) * DS, (j + 0.5) * DS);
      const flat = dmag[k] < thr * 1.6;
      if (ry < -0.03 && seed) {
        let dc = 9999;
        for (const sc of seed) dc = Math.min(dc, Math.abs(dpx[4 * k] - sc[0]) + Math.abs(dpx[4 * k + 1] - sc[1]) + Math.abs(dpx[4 * k + 2] - sc[2]));
        if (dc < 75 && dmag[k] < thr * 3) floorLike[k] = 1;
      } else if (ry > 0.5 && flat) ceilLike[k] = 1;
    }
  }
  let qh = 0, qt = 0;
  for (let j = Math.floor(DH * 0.88); j < DH - 1; j++) {
    for (let i = 1; i < DW - 1; i++) {
      const k = j * DW + i;
      if (floorLike[k] && !dlab[k]) { dlab[k] = 1; dqueue[qt++] = k; }
    }
  }
  while (qh < qt) {
    const k = dqueue[qh++];
    const i = k % DW;
    for (const nb of [k - 1, k + 1, k - DW, k + DW]) {
      if (nb < 0 || nb >= n) continue;
      if ((nb === k - 1 && i === 0) || (nb === k + 1 && i === DW - 1)) continue;
      if (floorLike[nb] && !dlab[nb]) { dlab[nb] = 1; dqueue[qt++] = nb; }
    }
  }
  const solid = world.walls.filter((w) => w.hits >= 2);
  if (solid.length) {
    for (let j = 1; j < DH - 1; j++) {
      for (let i = 1; i < DW - 1; i++) {
        const k = j * DW + i;
        if (dlab[k] !== 1) continue;
        const fp = floorPoint((i + 0.5) * DS, (j + 0.5) * DS);
        if (!fp) continue;
        for (const w of solid) {
          if (segCrossAny(pos.x, pos.z, fp.x, fp.z, w.x1, w.z1, w.x2, w.z2)) { dlab[k] = 2; break; }
        }
      }
    }
  }
  let cf = 0, cw = 0, cc = 0;
  for (let k = 0; k < n; k++) {
    if (dlab[k] === 0) dlab[k] = ceilLike[k] ? 3 : 2;
    const l = dlab[k];
    if (l === 1) cf++;
    else if (l === 3) cc++;
    else cw++;
  }
  det.floorPct = cf / n; det.wallPct = cw / n; det.ceilPct = cc / n;
  for (let j = 2; j < DH - 1; j += 3) {
    for (let i = 2; i < DW - 1; i += 3) {
      if (dlab[j * DW + i] !== 1) continue;
      const fp = floorPoint((i + 0.5) * DS, (j + 0.5) * DS);
      if (!fp || fp.d > 7) continue;
      world.free.set(Math.floor(fp.x / 0.5) + ',' + Math.floor(fp.z / 0.5), now);
    }
  }
  if (world.free.size > 5000) {
    const cut = now - 40000;
    for (const [key, t] of world.free) if (t < cut) world.free.delete(key);
  }
}

function segCrossAny(ax, az, bx, bz, cx, cz, dx, dz) {
  const d1x = bx - ax, d1z = bz - az, d2x = dx - cx, d2z = dz - cz;
  const den = d1x * d2z - d1z * d2x;
  if (Math.abs(den) < 1e-9) return false;
  const t = ((cx - ax) * d2z - (cz - az) * d2x) / den;
  const u = ((cx - ax) * d1z - (cz - az) * d1x) / den;
  return t > 0 && t < 1 && u >= 0 && u <= 1;
}
function isFree(x, z) {
  return world.free.has(Math.floor(x / 0.5) + ',' + Math.floor(z / 0.5));
}

function entType(e) {
  const v = e.votes;
  if (v.corner >= 2 && v.corner >= v.column) return 'corner';
  if (v.column >= 2 && v.column >= v.edge) return 'column';
  return v.corner > v.edge ? 'corner' : v.column > v.edge ? 'column' : 'edge';
}

function fuseEnt(o, now) {
  let best = null, bd = 0.7;
  for (const e of world.ents) {
    const d = Math.hypot(e.x - o.x, e.z - o.z);
    if (d < bd) { bd = d; best = e; }
  }
  if (!best) {
    best = { id: world.nid++, x: o.x, z: o.z, w: o.w || 0.3, hits: 0, miss: 0, votes: { corner: 0, column: 0, edge: 0 }, dirs: [], used: 0, pass: 0, born: now };
    world.ents.push(best);
  } else {
    best.x += (o.x - best.x) * 0.25; best.z += (o.z - best.z) * 0.25;
    if (o.type === 'column') best.w += (o.w - best.w) * 0.25;
  }
  best.hits = Math.min(best.hits + 1, 30);
  best.miss = 0; best.pass = world.pass; best.last = now;
  best.votes[o.type]++;
  best.type = entType(best);
  for (const a of o.dirs) {
    let hit = false;
    for (let q = 0; q < best.dirs.length; q++) {
      if (Math.abs(wrapPi(best.dirs[q] - a)) < 15 * DEG) { best.dirs[q] += wrapPi(a - best.dirs[q]) * 0.3; hit = true; break; }
    }
    if (!hit && best.dirs.length < 3) best.dirs.push(a);
  }
}

function fuseWall(w, now) {
  const len = Math.hypot(w.x2 - w.x1, w.z2 - w.z1);
  if (len < 0.8) return;
  const ang = Math.atan2(w.z2 - w.z1, w.x2 - w.x1);
  for (const e of world.walls) {
    const ea = Math.atan2(e.z2 - e.z1, e.x2 - e.x1);
    let da = Math.abs(wrapPi(ang - ea));
    da = Math.min(da, Math.PI - da);
    if (da > 12 * DEG) continue;
    const ux = Math.cos(ea), uz = Math.sin(ea), nx = -uz, nz = ux;
    const off1 = (w.x1 - e.x1) * nx + (w.z1 - e.z1) * nz;
    const off2 = (w.x2 - e.x1) * nx + (w.z2 - e.z1) * nz;
    if (Math.abs(off1) > 0.5 || Math.abs(off2) > 0.5) continue;
    const elen = Math.hypot(e.x2 - e.x1, e.z2 - e.z1);
    const t1 = (w.x1 - e.x1) * ux + (w.z1 - e.z1) * uz, t2 = (w.x2 - e.x1) * ux + (w.z2 - e.z1) * uz;
    const lo = Math.min(0, t1, t2), hi = Math.max(elen, t1, t2);
    if (Math.min(elen, Math.max(t1, t2)) - Math.max(0, Math.min(t1, t2)) < -0.8) continue;
    if (hi - lo > 14) continue;
    const off = ((off1 + off2) / 2) * 0.3;
    e.x1 = e.x1 + ux * lo + nx * off; e.z1 = e.z1 + uz * lo + nz * off;
    e.x2 = e.x1 - ux * lo + ux * hi; e.z2 = e.z1 - uz * lo + uz * hi;
    e.hits = Math.min(e.hits + 1, 30); e.miss = 0; e.pass = world.pass; e.last = now;
    return;
  }
  world.walls.push({ id: world.nid++, x1: w.x1, z1: w.z1, x2: w.x2, z2: w.z2, hits: 1, miss: 0, pass: world.pass, last: now, used: 0 });
}

function inView(x, z, tol) {
  const dx = x - pos.x, dz = z - pos.z, d = Math.hypot(dx, dz);
  if (d < 1.2 || d > 8) return false;
  return Math.abs(wrapPi(Math.atan2(dx, -dz) - Math.atan2(headX, -headZ))) < tol;
}

function decayWorld() {
  const tol = 22 * DEG;
  for (const e of world.ents) {
    if (e.pass === world.pass || !inView(e.x, e.z, tol)) continue;
    if (++e.miss > 10) { e.hits -= 1; e.miss = 0; }
  }
  for (const w of world.walls) {
    if (w.pass === world.pass || !inView((w.x1 + w.x2) / 2, (w.z1 + w.z2) / 2, tol)) continue;
    if (++w.miss > 14) { w.hits -= 1; w.miss = 0; }
  }
  world.ents = world.ents.filter((e) => e.hits > 0);
  world.walls = world.walls.filter((w) => w.hits > 0);
}

function injectWorld(ents, walls) {
  const now = performance.now();
  for (const e of ents || []) for (let q = 0; q < 5; q++) fuseEnt({ type: e.type, x: e.x, z: e.z, w: e.w || 0.3, dirs: e.dirs || [] }, now);
  for (const w of walls || []) for (let q = 0; q < 4; q++) fuseWall(w, now);
}

function runDetection(now) {
  if (!running || DW < 8) return;
  if (angSpeed > 1.4) { det.skipped++; return; }
  const t0 = performance.now();
  dmctx.drawImage(scene, 0, 0, DW, DH);
  detGradients();
  const thr = Math.max(9, S.dotThreshold * 0.9);
  det.thr = thr;
  detHarris();
  const lines = detVertical(thr);
  detSegment(thr, now);
  world.pass++;
  const { obsE, obsW } = classifyLines(lines, thr);
  for (const o of obsE) fuseEnt(o, now);
  for (const w of obsW) fuseWall(w, now);
  decayWorld();
  updateSpatialMarkers(now);
  det.passes++;
  det.ms += (performance.now() - t0 - det.ms) * 0.2;
}

function worldCounts() {
  let col = 0, cor = 0, edg = 0;
  for (const e of world.ents) {
    if (e.hits < 3) continue;
    if (e.type === 'column') col++; else if (e.type === 'corner') cor++; else edg++;
  }
  return { col, cor, edg, wall: world.walls.filter((w) => w.hits >= 2).length };
}

function seg3(c, ax, ay, az, bx, by, bz) {
  const near = 0.15;
  let a = toCam(ax - pos.x, ay - EYE, az - pos.z);
  let b = toCam(bx - pos.x, by - EYE, bz - pos.z);
  if (a[2] < near && b[2] < near) return;
  if (a[2] < near) { const t = (near - a[2]) / (b[2] - a[2]); a = [lerp(a[0], b[0], t), lerp(a[1], b[1], t), near]; }
  else if (b[2] < near) { const t = (near - b[2]) / (a[2] - b[2]); b = [lerp(b[0], a[0], t), lerp(b[1], a[1], t), near]; }
  const pa = toScreen(a), pb = toScreen(b);
  c.moveTo(pa[0], pa[1]); c.lineTo(pb[0], pb[1]);
}

// ==========================================================
// 右上角羅盤方位
// ==========================================================
const cmp = { nOff: 0, have: false, absSeen: false, src: '相對（無羅盤）', deg: 0, pitch: 0, roll: 0, gyro: 0, lastMeas: 0 };
let SAFE_TOP = 0;

function safeTop() {
  try {
    const d = document.createElement('div');
    d.style.cssText = 'position:fixed;top:0;left:0;padding-top:env(safe-area-inset-top,0px);visibility:hidden;pointer-events:none';
    document.body.appendChild(d);
    const v = parseFloat(getComputedStyle(d).paddingTop) || 0;
    d.remove();
    return v;
  } catch (e) { return 0; }
}

function gameHeading() { return Math.atan2(headX, -headZ); }

function cmpMeas(bearing, src) {
  const gh = gameHeading();
  const off = wrapPi(bearing - gh);
  if (!cmp.have) { cmp.nOff = off; cmp.have = true; }
  else cmp.nOff = wrapPi(cmp.nOff + wrapPi(off - cmp.nOff) * 0.04);
  cmp.src = src;
  cmp.lastMeas = performance.now();
}

function cmpFromEvent(e) {
  if (e.webkitCompassHeading != null && !isNaN(e.webkitCompassHeading) && B.F && Math.hypot(B.F[0], B.F[2]) > 0.5) {
    cmpMeas(e.webkitCompassHeading * DEG, '磁北（iOS 羅盤＋陀螺儀）');
  }
}

function onOrientAbs(e) {
  if (e.alpha == null || e.beta == null || e.gamma == null) return;
  cmp.absSeen = true;
  const x = e.beta * DEG, y = e.gamma * DEG, z = e.alpha * DEG;
  const cX = Math.cos(x), cY = Math.cos(y), cZ = Math.cos(z), sX = Math.sin(x), sY = Math.sin(y), sZ = Math.sin(z);
  const e0 = -(cY * sZ * sX + cZ * sY);
  const n0 = -(sZ * sY - cZ * cY * sX);
  if (Math.hypot(e0, n0) < 0.5) return;
  cmpMeas(Math.atan2(e0, n0), '磁北（Android 羅盤＋陀螺儀）');
}

function updateCompass() {
  cmp.deg = ((gameHeading() + cmp.nOff) / DEG + 360 * 4) % 360;
  cmp.pitch = Math.asin(Math.max(-1, Math.min(1, B.F[1]))) / DEG;
  cmp.roll = Math.atan2(B.sr[1], B.su[1]) / DEG;
}

const CARD = ['北', '東北', '東', '東南', '南', '西南', '西', '西北'];
function cardinal(deg) { return CARD[Math.round(deg / 45) % 8]; }

function drawCompass() {
  const R = 36, cx = W - R - 12, cy = SAFE_TOP + R + 10;
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  ctx.save();
  ctx.fillStyle = 'rgba(0,0,0,0.6)';
  ctx.strokeStyle = 'rgba(255,255,255,0.55)';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  const rel = (bearingDeg) => (bearingDeg - cmp.deg) * DEG;
  for (let a = 0; a < 360; a += 30) {
    const t = rel(a), s = Math.sin(t), c = -Math.cos(t);
    const major = a % 90 === 0;
    ctx.strokeStyle = 'rgba(255,255,255,' + (major ? 0.9 : 0.4) + ')';
    ctx.beginPath();
    ctx.moveTo(cx + s * (R - (major ? 9 : 5)), cy + c * (R - (major ? 9 : 5)));
    ctx.lineTo(cx + s * (R - 1), cy + c * (R - 1));
    ctx.stroke();
  }
  ctx.font = '600 11px -apple-system, "PingFang TC", sans-serif';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  [['N', 0, '#ff6a5a'], ['E', 90, '#fff'], ['S', 180, '#fff'], ['W', 270, '#fff']].forEach(([L, a, col]) => {
    const t = rel(a);
    ctx.fillStyle = col;
    ctx.fillText(L, cx + Math.sin(t) * (R - 18), cy - Math.cos(t) * (R - 18));
  });
  const put = (x, z, col, r) => {
    const b = Math.atan2(x - pos.x, -(z - pos.z));
    const tt = b + cmp.nOff - cmp.deg * DEG;
    ctx.fillStyle = col;
    ctx.beginPath(); ctx.arc(cx + Math.sin(tt) * (R - 4), cy - Math.cos(tt) * (R - 4), r, 0, Math.PI * 2); ctx.fill();
  };
  for (const id in net.markers) {
    const m = net.markers[id];
    put(m.x, m.z, m.active ? '#6effe1' : 'rgba(255,255,255,0.5)', 2.2);
  }
  for (const pid in net.players) {
    const p = net.players[pid];
    put(p.x, p.z, '#ff5252', 2.8);
  }
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, 11, 0, Math.PI * 2); ctx.clip();
  ctx.translate(cx, cy);
  ctx.rotate((-cmp.roll * DEG));
  const off = Math.max(-10, Math.min(10, cmp.pitch * 0.2));
  ctx.fillStyle = 'rgba(90,160,255,0.55)'; ctx.fillRect(-14, -14 + off, 28, 14);
  ctx.fillStyle = 'rgba(160,110,60,0.65)'; ctx.fillRect(-14, off, 28, 14);
  ctx.strokeStyle = '#fff'; ctx.beginPath(); ctx.moveTo(-14, off); ctx.lineTo(14, off); ctx.stroke();
  ctx.restore();
  ctx.fillStyle = '#fff';
  ctx.beginPath(); ctx.moveTo(cx, cy - R - 1); ctx.lineTo(cx - 5, cy - R - 9); ctx.lineTo(cx + 5, cy - R - 9); ctx.closePath(); ctx.fill();
  ctx.restore();
}

// ==========================================================
// AR 懸疑場景
// ==========================================================
const susp = {
  fovMul: 1, erratic: 0, yawAbs: 0, rev: 0, lastSign: 0, spdJit: 0, prevSpeed: 0, prevYaw: 0,
  next: 0, lastEv: 0, count: 0, last: '', ac: null,
  fig: null, door: null, flicker: null, glitchUntil: 0, flashUntil: 0, flashA: 0,
  sub: null, edge: null,
};

function audioInit() {
  try {
    const AC = window.AudioContext || window.webkitAudioContext;
    if (AC && !susp.ac) susp.ac = new AC();
    if (susp.ac && susp.ac.state === 'suspended') susp.ac.resume();
  } catch (e) { susp.ac = null; }
}
function noiseBuf(ac, sec) {
  const b = ac.createBuffer(1, Math.floor(ac.sampleRate * sec), ac.sampleRate), d = b.getChannelData(0);
  for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
  return b;
}
function sfxSteps(pan, n) {
  const ac = susp.ac;
  if (!ac) return;
  let t = ac.currentTime + 0.05;
  for (let i = 0; i < n; i++) {
    const src = ac.createBufferSource(); src.buffer = noiseBuf(ac, 0.12);
    const f = ac.createBiquadFilter(); f.type = 'lowpass'; f.frequency.value = 380 + i * 40;
    const g = ac.createGain(); const vol = 0.12 + i * 0.07;
    g.gain.setValueAtTime(0, t); g.gain.linearRampToValueAtTime(vol, t + 0.01); g.gain.exponentialRampToValueAtTime(0.001, t + 0.12);
    let node = g;
    if (ac.createStereoPanner) { const p = ac.createStereoPanner(); p.pan.value = pan * (1 - i * 0.08); g.connect(p); node = p; }
    src.connect(f); f.connect(g); node.connect(ac.destination);
    src.start(t);
    t += 0.62 - i * 0.04;
  }
}
function sfxThump() {
  const ac = susp.ac;
  if (!ac) return;
  const o = ac.createOscillator(), g = ac.createGain(), t = ac.currentTime;
  o.type = 'sine'; o.frequency.setValueAtTime(70, t); o.frequency.exponentialRampToValueAtTime(38, t + 0.5);
  g.gain.setValueAtTime(0.4, t); g.gain.exponentialRampToValueAtTime(0.001, t + 0.6);
  o.connect(g); g.connect(ac.destination); o.start(t); o.stop(t + 0.65);
}
function sfxCreak() {
  const ac = susp.ac;
  if (!ac) return;
  const o = ac.createOscillator(), g = ac.createGain(), f = ac.createBiquadFilter(), t = ac.currentTime;
  o.type = 'sawtooth'; o.frequency.setValueAtTime(110, t); o.frequency.linearRampToValueAtTime(170, t + 1.1);
  f.type = 'bandpass'; f.frequency.value = 600; f.Q.value = 6;
  g.gain.setValueAtTime(0, t); g.gain.linearRampToValueAtTime(0.07, t + 0.2); g.gain.linearRampToValueAtTime(0, t + 1.2);
  o.connect(f); f.connect(g); g.connect(ac.destination); o.start(t); o.stop(t + 1.25);
}
function buzz(p) { try { if (navigator.vibrate) navigator.vibrate(p); } catch (e) {} }

function say(text, ms) { susp.sub = { text, born: performance.now(), until: performance.now() + (ms || 4200) }; }
function whisperEdge(text, ms) {
  susp.edge = { text, born: performance.now(), until: performance.now() + (ms || 4800), side: Math.random() < 0.5 ? -1 : 1, y: rand(0.3, 0.7) };
}

function bearingTo(x, z) { return wrapPi(Math.atan2(x - pos.x, -(z - pos.z)) - gameHeading()); }
function halfH() { return Math.max(12 * DEG, Math.atan((W / 2) / FPX)); }

function spotOk(x, z) {
  for (const w of world.walls) if (w.hits >= 2 && segCrossAny(pos.x, pos.z, x, z, w.x1, w.z1, w.x2, w.z2)) return false;
  for (const o of obs) if (!o.dying && distPointBox(x, z, o) < 0.5) return false;
  return true;
}

function placeFigure(now, dist, hops) {
  const best = world.ents
    .filter((e) => e.hits >= 3 && e.type !== 'edge')
    .map((e) => ({ e, b: bearingTo(e.x, e.z), d: Math.hypot(e.x - pos.x, e.z - pos.z) }))
    .filter((q) => Math.abs(q.b) > 0.5 * halfH() && Math.abs(q.b) < 1.0 * halfH() && q.d > 3 && q.d < 10)
    .sort((p, q) => Math.abs(Math.abs(p.b) - 0.75 * halfH()) - Math.abs(Math.abs(q.b) - 0.75 * halfH()))[0];
  if (best && hops === 0) {
    const dx = best.e.x - pos.x, dz = best.e.z - pos.z, d = Math.hypot(dx, dz) || 1;
    const sd = best.b > 0 ? 1 : -1;
    const x = best.e.x + (-dz / d) * 0.5 * sd + (dx / d) * 0.4, z = best.e.z + (dx / d) * 0.5 * sd + (dz / d) * 0.4;
    if (spotOk(x, z)) return { x, z, hide: true };
  }
  for (let k = 0; k < 40; k++) {
    const side = Math.random() < 0.5 ? -1 : 1;
    const a = gameHeading() + side * rand(0.6, 0.95) * halfH();
    const d = Math.max(2.6, dist * rand(0.45, 1.05));
    const x = pos.x + Math.sin(a) * d, z = pos.z - Math.cos(a) * d;
    if (spotOk(x, z)) return { x, z, hide: false };
  }
  return null;
}

function evFigure(now) {
  const spot = placeFigure(now, rand(5, 7.5), 0);
  if (!spot) return false;
  susp.fig = { x: spot.x, z: spot.z, born: now, vanish: 0, hops: 0, fail: 0, seenCenter: 0, respawnAt: 0, seed: Math.random() * 100 };
  whisperEdge('視野邊緣，好像有人', 4200);
  buzz([20]);
  return true;
}
function updateFigure(dt, now) {
  const f = susp.fig;
  if (!f) return;
  if (f.vanish) {
    if (now - f.vanish > 800 && !f.respawnAt) {
      if (f.hops >= 3) { susp.fig = null; say('你一直在看錯的地方', 3600); return; }
      f.respawnAt = now + rand(1400, 2600);
    }
    if (f.respawnAt && now >= f.respawnAt) {
      const d0 = Math.hypot(f.x - pos.x, f.z - pos.z);
      const sp = placeFigure(now, Math.max(2.8, d0 - 1.3), f.hops + 1);
      if (!sp) { f.respawnAt = now + 1000; if (++f.fail > 5) susp.fig = null; return; }
      f.x = sp.x; f.z = sp.z; f.vanish = 0; f.respawnAt = 0; f.hops++; f.born = now; f.seenCenter = 0;
      if (f.hops === 1) whisperEdge('它又出現了，離你更近', 3600);
      sfxSteps(bearingTo(f.x, f.z) > 0 ? 0.7 : -0.7, 2);
    }
    return;
  }
  const b = Math.abs(bearingTo(f.x, f.z)), d = Math.hypot(f.x - pos.x, f.z - pos.z);
  const inside = b < Math.max(6 * DEG, 0.42 * halfH()) && d < 12;
  f.seenCenter = inside ? f.seenCenter + dt : Math.max(0, f.seenCenter - dt * 2);
  if (f.seenCenter > 0.35 || d < 2.2) {
    f.vanish = now;
    susp.glitchUntil = now + 260; susp.flashUntil = now + 120; susp.flashA = 0.5;
    sfxThump();
  }
}

function evDoor(now) {
  const cand = world.walls
    .filter((w) => w.hits >= 2 && Math.hypot(w.x2 - w.x1, w.z2 - w.z1) >= 1.6)
    .map((w) => {
      const mx = (w.x1 + w.x2) / 2, mz = (w.z1 + w.z2) / 2;
      return { w, d: Math.hypot(mx - pos.x, mz - pos.z), b: Math.abs(bearingTo(mx, mz)) };
    })
    .filter((q) => q.d > 2 && q.d < 8 && q.b < 70 * DEG)
    .sort((p, q) => p.b - q.b)[0];
  if (!cand) return false;
  const s = sideOf(cand.w);
  const t = rand(0.35, 0.65);
  const x = cand.w.x1 + (cand.w.x2 - cand.w.x1) * t + s.nx * 0.04;
  const z = cand.w.z1 + (cand.w.z2 - cand.w.z1) * t + s.nz * 0.04;
  susp.door = { x, z, rot: Math.atan2(s.uz, s.ux), nx: s.nx, nz: s.nz, born: now, closeAt: 0 };
  say('牆上多了一扇門——模型裡原本沒有', 4600);
  sfxCreak(); buzz([40, 50, 40]);
  susp.flicker = { start: now, dur: 900 };
  return true;
}
function updateDoor(now) {
  const d = susp.door;
  if (!d) return;
  const dist = Math.hypot(d.x - pos.x, d.z - pos.z);
  if (!d.closeAt && (dist < 1.7 || now - d.born > 16000)) {
    d.closeAt = now;
    if (dist < 1.7) { sfxThump(); susp.flashUntil = now + 140; susp.flashA = 0.6; say('門關上了，牆又平了', 3200); }
  }
  if (d.closeAt && now - d.closeAt > 700) susp.door = null;
}

function evBlackout(now) {
  susp.flicker = { start: now, dur: 1700 };
  say('視野收縮', 2600);
  sfxThump(); buzz([60, 40, 60, 40, 90]);
}
function evSteps() {
  const side = Math.random() < 0.5 ? -1 : 1;
  sfxSteps(side * 0.85, 5);
  buzz([15, 150, 15, 150, 25, 140, 35]);
  say(side < 0 ? '左後方有腳步聲' : '右後方有腳步聲', 3600);
}
function evGlitch(now) {
  susp.glitchUntil = now + 700;
  say('訊號中斷　重新定位中…', 2600);
  sfxThump();
  for (const e of world.ents) { e.x += rand(-0.15, 0.15); e.z += rand(-0.15, 0.15); }
}
const WHISPERS = ['你剛剛走過這裡嗎？', '牆的位置和剛才不一樣', '邊緣比中心更誠實', '不要一直看中間', '有些東西只在看不清楚時存在', '再往前一步'];
function evWhisper() { whisperEdge(WHISPERS[Math.floor(Math.random() * WHISPERS.length)], 5200); }

function triggerSuspense(name) {
  const now = performance.now();
  if (!name) {
    const hasWall = world.walls.some((w) => w.hits >= 2);
    const pool = [['figure', 3], ['door', hasWall ? 2.5 : 0], ['blackout', 1.5], ['steps', 1.2], ['glitch', 1.3], ['whisper', 1.2]]
      .filter((p) => p[1] > 0 && p[0] !== susp.last && !(p[0] === 'figure' && susp.fig));
    let tot = 0;
    for (const p of pool) tot += p[1];
    let r = Math.random() * tot;
    name = pool[0][0];
    for (const p of pool) { if ((r -= p[1]) <= 0) { name = p[0]; break; } }
  }
  let ok = true;
  if (name === 'figure') ok = evFigure(now);
  else if (name === 'door') ok = evDoor(now);
  else if (name === 'blackout') evBlackout(now);
  else if (name === 'steps') evSteps();
  else if (name === 'glitch') evGlitch(now);
  else if (name === 'whisper') evWhisper();
  if (!ok) { evWhisper(); name = 'whisper'; }
  susp.last = name; susp.count++; susp.lastEv = now;
  susp.next = now + rand(10000, 17000);
  return name;
}

function updateSuspense(dt, now) {
  const yaw = gameHeading();
  let dy = wrapPi(yaw - susp.prevYaw);
  susp.prevYaw = yaw;
  const rate = dy / Math.max(dt, 1e-3);
  susp.yawAbs += (Math.abs(rate) - susp.yawAbs) * Math.min(1, dt * 0.8);
  if (Math.abs(rate) > 0.4) {
    const sg = rate > 0 ? 1 : -1;
    if (susp.lastSign && sg !== susp.lastSign) susp.rev += 1;
    susp.lastSign = sg;
  }
  susp.rev *= Math.exp(-dt / 4);
  const jit = Math.abs(moveSpeed - susp.prevSpeed) / Math.max(dt, 1e-3);
  susp.prevSpeed = moveSpeed;
  susp.spdJit += (jit - susp.spdJit) * Math.min(1, dt * 0.6);
  susp.erratic = Math.min(1, 0.5 * clamp01(susp.yawAbs / 0.9) + 0.5 * clamp01(susp.rev / 2.5) + 0.3 * clamp01(susp.spdJit / 2));

  if (running && now >= susp.next) {
    const quiet = now - susp.lastEv > 50000;
    if (susp.erratic > 0.38 || (quiet && moveSpeed > 0.3)) triggerSuspense();
  }
  updateFigure(dt, now);
  updateDoor(now);

  let mul = 1;
  if (susp.flicker) {
    const t = (now - susp.flicker.start) / susp.flicker.dur;
    if (t >= 1) susp.flicker = null;
    else {
      const pulse = Math.sin(t * 38) * Math.sin(t * 17 + 1);
      mul = pulse > 0.15 ? 1 : 0.35;
      if (t > 0.8) mul = lerp(mul, 1, (t - 0.8) / 0.2);
    }
  }
  susp.fovMul = mul;
}

function figSegs(f, now) {
  const dx = f.x - pos.x, dz = f.z - pos.z, d = Math.hypot(dx, dz) || 1;
  const rx = -dz / d, rz = dx / d;
  const sc = 1.12;
  const sway = Math.sin(now / 900 + f.seed) * 0.03;
  const P = (s, y) => [f.x + rx * (s + sway * y) * sc, y * sc, f.z + rz * (s + sway * y) * sc];
  const segs = [];
  const L = (a, b) => segs.push([a[0], a[1], a[2], b[0], b[1], b[2]]);
  const head = [];
  for (let i = 0; i < 8; i++) { const a = (i / 8) * Math.PI * 2; head.push(P(Math.cos(a) * 0.1, 1.62 + Math.sin(a) * 0.12)); }
  for (let i = 0; i < 8; i++) L(head[i], head[(i + 1) % 8]);
  L(P(0, 1.5), P(0, 0.95));
  L(P(-0.2, 1.42), P(0.2, 1.42));
  L(P(-0.2, 1.42), P(-0.26, 0.98)); L(P(-0.26, 0.98), P(-0.22, 0.72));
  L(P(0.2, 1.42), P(0.26, 0.98)); L(P(0.26, 0.98), P(0.22, 0.72));
  L(P(-0.12, 0.95), P(0.12, 0.95));
  L(P(-0.12, 0.95), P(-0.13, 0.48)); L(P(-0.13, 0.48), P(-0.12, 0));
  L(P(0.12, 0.95), P(0.13, 0.48)); L(P(0.13, 0.48), P(0.12, 0));
  return segs;
}

function drawSuspScene(c, now) {
  const strokeSegs = (segs, alpha, lw) => {
    c.strokeStyle = `rgba(255,255,255,${alpha.toFixed(2)})`;
    c.lineWidth = lw;
    c.beginPath();
    for (const sg of segs) seg3(c, sg[0], sg[1], sg[2], sg[3], sg[4], sg[5]);
    c.stroke();
  };
  const f = susp.fig;
  if (f) {
    let a = clamp01((now - f.born) / 500);
    if (f.vanish) a *= 1 - clamp01((now - f.vanish) / 500);
    if (Math.sin(now / 70 + f.seed) > 0.92) a *= 0.3;
    if (a > 0.02) strokeSegs(figSegs(f, now), 0.95 * a, 2.2);
  }
  const d = susp.door;
  if (d) {
    const c0 = Math.cos(d.rot), s0 = Math.sin(d.rot);
    const W2 = 0.475, HH = 2.1;
    const lp = (lx, y, lz) => [d.x + lx * c0 - lz * s0, y, d.z + lx * s0 + lz * c0];
    const open = d.closeAt ? 1 - clamp01((now - d.closeAt) / 700) : clamp01((now - d.born) / 2200);
    const ang = open * 0.75;
    const segs = [];
    const L = (a, b) => segs.push([a[0], a[1], a[2], b[0], b[1], b[2]]);
    const g = clamp01((now - d.born) / 900) * (d.closeAt ? open : 1);
    const sy = HH * g;
    L(lp(-W2, 0, 0), lp(-W2, sy, 0)); L(lp(W2, 0, 0), lp(W2, sy, 0)); L(lp(-W2, sy, 0), lp(W2, sy, 0));
    const zdir = (d.nx * -s0 + d.nz * c0) >= 0 ? 1 : -1;
    const ex = -W2 + Math.cos(ang) * 2 * W2, ez2 = zdir * Math.sin(ang) * 2 * W2;
    L(lp(-W2, 0, 0), lp(ex, 0, ez2)); L(lp(-W2, sy, 0), lp(ex, sy, ez2)); L(lp(ex, 0, ez2), lp(ex, sy, ez2));
    L(lp(-W2, 0, 0), lp(ex, sy, ez2));
    strokeSegs(segs, 0.9 * (Math.sin(now / 90) > 0.9 ? 0.4 : 1), 2);
  }
}

function drawSuspOverlay(now) {
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  if (now < susp.glitchUntil) {
    const n = 4 + Math.floor(Math.random() * 4);
    for (let i = 0; i < n; i++) {
      const sh = rand(0.02, 0.09) * H, sy = rand(0, H - sh), off = rand(-0.09, 0.09) * W;
      ctx.drawImage(cv, 0, sy * DPR, W * DPR, sh * DPR, off, sy, W, sh);
    }
    ctx.fillStyle = 'rgba(255,255,255,0.06)';
    ctx.fillRect(0, rand(0, H), W, 2);
  }
  if (now < susp.flashUntil) {
    ctx.fillStyle = `rgba(0,0,0,${susp.flashA})`;
    ctx.fillRect(0, 0, W, H);
  }
  if (susp.flicker) {
    const t = (now - susp.flicker.start) / susp.flicker.dur;
    if (Math.sin(t * 38) < -0.7) { ctx.fillStyle = 'rgba(0,0,0,0.55)'; ctx.fillRect(0, 0, W, H); }
  }
  if (susp.edge) {
    const e = susp.edge, t = now - e.born, left = e.until - now;
    if (left < 0) susp.edge = null;
    else {
      const a = Math.min(clamp01(t / 700), clamp01(left / 900));
      ctx.font = '13px -apple-system, "PingFang TC", sans-serif';
      ctx.textBaseline = 'middle';
      ctx.fillStyle = `rgba(255,255,255,${(0.75 * a).toFixed(2)})`;
      const jitter = Math.sin(now / 55) * 1.5 * (Math.sin(now / 300) > 0.6 ? 1 : 0);
      ctx.save();
      ctx.translate(e.side < 0 ? 12 : W - 12, e.y * H + jitter);
      ctx.rotate((e.side < 0 ? -1 : 1) * Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillText(e.text.split('').join(' '), 0, 0);
      ctx.restore();
    }
  }
  if (susp.sub) {
    const s = susp.sub, left = s.until - now;
    if (left < 0) susp.sub = null;
    else {
      const a = Math.min(clamp01((now - s.born) / 300), clamp01(left / 600));
      ctx.font = '600 15px -apple-system, "PingFang TC", sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      const tw = ctx.measureText(s.text).width + 24, y = H * 0.74;
      ctx.fillStyle = `rgba(0,0,0,${(0.6 * a).toFixed(2)})`;
      ctx.fillRect(W / 2 - tw / 2, y - 16, tw, 32);
      ctx.fillStyle = `rgba(255,255,255,${a.toFixed(2)})`;
      ctx.fillText(s.text, W / 2, y);
    }
  }
}

function resetSuspense() {
  susp.fig = null; susp.door = null; susp.flicker = null; susp.sub = null; susp.edge = null;
  susp.glitchUntil = 0; susp.flashUntil = 0; susp.fovMul = 1; susp.count = 0; susp.last = '';
  susp.rev = 0; susp.yawAbs = 0; susp.spdJit = 0; susp.erratic = 0;
  susp.next = performance.now() + 14000; susp.lastEv = performance.now();
}

// ==========================================================
// 移動與網路座標回傳
// ==========================================================
function updateMove(dt, now) {
  if (holdWalk) pending = Math.min(pending + 1.3 * dt, 2);
  const mv = Math.min(pending, 2.0 * dt);
  pending -= mv;
  pos.x += headX * mv;
  pos.z += headZ * mv;
  moveSpeed += (mv / Math.max(dt, 1e-3) - moveSpeed) * Math.min(1, dt * 6);
  if (moveSpeed < 0.02) moveSpeed = 0;

  if (now - net.lastSync > 120) {
    net.lastSync = now;
    sendNet({
      type: 'move',
      id: net.id,
      x: +pos.x.toFixed(2),
      z: +pos.z.toFixed(2),
      yaw: +gameHeading().toFixed(2),
      speed: +moveSpeed.toFixed(2),
    });
  }
}

let accBase = 9.8, prevSig = 0, lastStep = 0;
function onMotion(e) {
  const rr = e.rotationRate;
  if (rr && rr.alpha != null) cmp.gyro += (Math.hypot(rr.alpha, rr.beta, rr.gamma) - cmp.gyro) * 0.2;
  const a = e.accelerationIncludingGravity;
  if (!a || a.x == null) return;
  const mag = Math.hypot(a.x, a.y, a.z);
  accBase += (mag - accBase) * 0.08;
  const sig = mag - accBase;
  const now = performance.now();
  if (sig > 1.3 && prevSig <= 1.3 && now - lastStep > 320) {
    lastStep = now;
    steps++;
    pending += S.stepLen;
  }
  prevSig = sig;
}

function onOrient(e) {
  if (e.beta == null || e.gamma == null) return;
  sensor.a = e.alpha == null ? 0 : e.alpha;
  sensor.b = e.beta;
  sensor.g = e.gamma;
  sensor.seen = true;
  hasSensor = true;
  cmpFromEvent(e);
}

// ==========================================================
// HUD 狀態文字（偵測線條已在後台運行，僅顯示簡潔定位與連機狀態）
// ==========================================================
function pct(v) { return Math.round(v * 100) + '%'; }

function drawHud(now) {
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  if (!S.showHud) {
    if (now < genLog.until) hudBox([`> ${genLog.text}`]);
    return;
  }
  const mList = Object.values(net.markers);
  const activeM = mList.filter((m) => m.active).length;
  const playerCount = Object.keys(net.players).length + 1;
  const speedState = moveSpeed < 0.05 ? '不動(最大)' : moveSpeed <= 0.75 ? '慢走(放大)' : '快走(縮小)';
  const modeLabel = net.mode === 'ws' ? '伺服器' : net.mode.startsWith('webrtc') ? `雲端(${net.room})` : '單機';

  const lines = [
    `> 連機 [${modeLabel}] ID:${net.id} 人數:${playerCount} 座標:(${pos.x.toFixed(1)}, ${pos.z.toFixed(1)})`,
    `> 視野 ${speedState} 倍率:${pct(fov.speedScale)} 速度:${moveSpeed.toFixed(2)}m/s`,
    `> 標記物 激活:${activeM}/${mList.length} (<2.2m激活) 可見敵人:${detectedEnemies.filter((e) => !e.occluded).length}`,
    `> 建築 ${new Set(obs.filter((o) => o.gen && !o.dying).map((o) => o.gid)).size} 處 步數:${steps} 後台偵測:${det.ms.toFixed(0)}ms`,
  ];
  if (now < genLog.until) lines.push(`> ${genLog.text}`);
  hudBox(lines);
}

function hudBox(lines) {
  const top = 10 + SAFE_TOP;
  ctx.font = '11px ui-monospace, Menlo, monospace';
  const maxW = Math.max(120, W - 16 - 110);
  const fit = (t) => { while (t.length > 2 && ctx.measureText(t).width > maxW - 16) t = t.slice(0, -2) + '…'; return t; };
  lines = lines.map(fit);
  let w = 0;
  for (const t of lines) w = Math.max(w, ctx.measureText(t).width);
  ctx.fillStyle = 'rgba(0,0,0,0.55)';
  ctx.fillRect(8, top - 4, w + 16, lines.length * 15 + 8);
  ctx.fillStyle = 'rgba(255,255,255,0.92)';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  lines.forEach((t, i) => ctx.fillText(t, 16, top + i * 15));
}

// ==========================================================
// 主迴圈
// ==========================================================
function frame(t) {
  requestAnimationFrame(frame);
  if (!running) return;
  const dt = Math.min(0.1, Math.max(0.001, (t - lastT) / 1000));
  lastT = t;
  timeSinceStart += dt;

  const now = performance.now();
  updateCamera();
  trackAng(dt);
  updateMove(dt, now);
  updateVision();
  updateScanMemory(dt);
  updateHabit(dt);
  updateSuspense(dt, now);
  updateCompass();
  updateGeneration(now);

  drawVideo();
  if (++detTick % 4 === 0) runDetection(now);
  if (detTick % 12 === 0) runPersonPhoneDetection();

  drawStructures(sctx);
  drawSpatialMarkers(sctx);
  drawSuspScene(sctx, now);

  if (S.limitOn) {
    composeLimited();
  } else {
    ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    ctx.drawImage(scene, 0, 0, W, H);
    C = centerRect();
  }

  drawSpatialMarkers(ctx);
  drawEnemies(ctx);
  drawSuspOverlay(now);
  drawCompass();
  drawHud(performance.now());
}

// ==========================================================
// 啟動與操作
// ==========================================================
function calibrate() {
  h0 = null;
  yawM = 0;
  world.ents = []; world.walls = []; world.free.clear();
  obs = obs.filter((o) => !o.gen);
  cmp.have = false;
}

function resetWorld() {
  obs = [];
  pos.x = pos.z = 0;
  pending = 0;
  moveSpeed = 0;
  steps = 0;
  scanMap.fill(0);
  resetHabit();
  world.ents = []; world.walls = []; world.free.clear();
  resetSuspense();
}

function seedWorld() {
  updateCamera();
  nextGenAt = performance.now() + GEN_FIRST;
}

async function start() {
  const err = $('err');
  err.textContent = '';
  try {
    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
      const r = await DeviceOrientationEvent.requestPermission();
      if (r !== 'granted') err.textContent = '沒有取得方向感測權限，將改用拖曳螢幕轉向。';
    }
    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
      try { await DeviceMotionEvent.requestPermission(); } catch (e) {}
    }
  } catch (e) {
    err.textContent = '感測器權限被拒絕，將改用拖曳螢幕轉向。';
  }

  audioInit();
  initNetwork();
  loadVisionModel();
  window.addEventListener('deviceorientation', onOrient);
  window.addEventListener('deviceorientationabsolute', onOrientAbs);
  window.addEventListener('devicemotion', onMotion);

  try {
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) throw new Error('此瀏覽器不支援鏡頭（需要 https 或 localhost）');
    const stream = await navigator.mediaDevices.getUserMedia({
      video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 720 } },
      audio: false,
    });
    video.srcObject = stream;
    await video.play();
  } catch (e) {
    err.textContent = '無法開啟鏡頭：' + (e && e.message ? e.message : e) + '。將以深色背景進入測試模式。';
    await new Promise((res) => setTimeout(res, 1500));
  }

  try { if (navigator.wakeLock) await navigator.wakeLock.request('screen'); } catch (e) {}

  $('intro').style.display = 'none';
  $('bar').hidden = false;
  resize();
  resetWorld();
  calibrate();
  seedWorld();
  running = true;
  lastT = performance.now();
}

function bindUI() {
  $('start').addEventListener('click', start);
  $('bCal').addEventListener('click', calibrate);
  $('bLimit').addEventListener('click', (e) => {
    S.limitOn = !S.limitOn;
    e.currentTarget.classList.toggle('on', S.limitOn);
  });
  const walk = $('bWalk');
  const on = (e) => { e.preventDefault(); holdWalk = true; walk.classList.add('on'); };
  const off = (e) => { e.preventDefault(); holdWalk = false; walk.classList.remove('on'); };
  walk.addEventListener('pointerdown', on);
  walk.addEventListener('pointerup', off);
  walk.addEventListener('pointercancel', off);
  walk.addEventListener('pointerleave', off);
  $('bSet').addEventListener('click', () => $('panel').classList.toggle('open'));

  const bind = (id, out, key, fmt, onChange) => {
    const el = $(id), o = $(out);
    if (!el || !o) return;
    el.value = S[key];
    o.textContent = fmt(S[key]);
    el.addEventListener('input', () => {
      S[key] = parseFloat(el.value);
      o.textContent = fmt(S[key]);
      if (onChange) onChange();
      saveSettings();
    });
  };
  bind('sSize', 'oSize', 'sizeMul', (v) => v.toFixed(2), () => { C = centerRect(); });
  bind('sFov', 'oFov', 'fovLong', (v) => v + '°');
  bind('sStep', 'oStep', 'stepLen', (v) => v.toFixed(2));
  bind('sDot', 'oDot', 'dotThreshold', (v) => String(v));

  const hudBtn = $('bHud');
  if (hudBtn) {
    hudBtn.classList.toggle('on', S.showHud);
    hudBtn.addEventListener('click', () => { S.showHud = !S.showHud; hudBtn.classList.toggle('on', S.showHud); saveSettings(); });
  }
  $('bClear').addEventListener('click', () => { obs = obs.filter((o) => !o.gen); genCount = 0; genHistory = []; });
  const freeBtn = $('bFree');
  if (freeBtn) {
    freeBtn.classList.toggle('on', S.freeGen);
    freeBtn.addEventListener('click', () => { S.freeGen = !S.freeGen; freeBtn.classList.toggle('on', S.freeGen); });
  }
  $('bSus').addEventListener('click', () => { audioInit(); triggerSuspense(); });
  $('bScan').addEventListener('click', () => { world.ents = []; world.walls = []; world.free.clear(); scanMap.fill(0); });
  $('bGen').addEventListener('click', () => { generateByHabit(); });

  let drag = null;
  cv.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY }; });
  window.addEventListener('pointerup', () => { drag = null; });
  window.addEventListener('pointermove', (e) => {
    if (!drag || (hasSensor && sensor.seen)) return;
    yawM += (e.clientX - drag.x) * 0.006;
    pitchM = Math.max(-1.3, Math.min(1.3, pitchM - (e.clientY - drag.y) * 0.006));
    drag = { x: e.clientX, y: e.clientY };
  });
  window.addEventListener('keydown', (e) => { if (e.code === 'KeyW') holdWalk = true; });
  window.addEventListener('keyup', (e) => { if (e.code === 'KeyW') holdWalk = false; });
}

loadSettings();
bindUI();
resize();
requestAnimationFrame(frame);

window.__ar = { S, fov, habit, net, scanMap, get obs() { return obs; }, pos, sensor, tryPlace, generateByHabit, start, calibrate, world, det, cmp, susp, injectWorld, triggerSuspense, runDetection, worldCounts };
})();
