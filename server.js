const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const path = require('path');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

app.use(express.static(path.join(__dirname)));

// 伺服器端全域狀態：玩家、已生成/激活的標記物 (A-1, A-2...)、牆體地圖
const state = {
  players: {},   // id -> { id, x, z, yaw, speed, holdingPhone, lastSeen }
  markers: {},   // id (例如 "A-1") -> { id, x, y, z, nx, nz, active, activatedBy, timestamp }
  walls: [],     // 共享的環境牆體線段
  markerSeq: 0,  // 用來生成 A-1, A-2...
};

function nextMarkerId() {
  state.markerSeq++;
  const group = String.fromCharCode(65 + Math.floor((state.markerSeq - 1) / 20)); // A, B, C...
  const num = ((state.markerSeq - 1) % 20) + 1;
  return `${group}-${num}`;
}

function broadcast(data, excludeWs = null) {
  const msg = JSON.stringify(data);
  wss.clients.forEach((client) => {
    if (client !== excludeWs && client.readyState === WebSocket.OPEN) {
      client.send(msg);
    }
  });
}

wss.on('connection', (ws) => {
  const playerId = 'P_' + Math.random().toString(36).substring(2, 7).toUpperCase();
  state.players[playerId] = {
    id: playerId,
    x: 0,
    z: 0,
    yaw: 0,
    speed: 0,
    holdingPhone: true,
    lastSeen: Date.now(),
  };

  // 初次連線：回傳玩家 ID 與目前伺服器上的所有標記物與玩家
  ws.send(JSON.stringify({
    type: 'init',
    id: playerId,
    players: state.players,
    markers: state.markers,
    walls: state.walls,
  }));

  broadcast({ type: 'playerJoined', player: state.players[playerId] }, ws);

  ws.on('message', (raw) => {
    try {
      const data = JSON.parse(raw);
      const p = state.players[playerId];
      if (!p) return;
      p.lastSeen = Date.now();

      // 1. 玩家移動與朝向更新
      if (data.type === 'move') {
        p.x = data.x;
        p.z = data.z;
        p.yaw = data.yaw;
        p.speed = data.speed || 0;
        broadcast({ type: 'playerMove', player: p }, ws);
      }

      // 2. 客戶端掃描到新物體邊緣，請求註冊候選標記物
      else if (data.type === 'registerMarker') {
        const exists = Object.values(state.markers).find(
          (m) => Math.hypot(m.x - data.x, m.z - data.z) < 1.2
        );
        if (!exists) {
          const id = nextMarkerId();
          state.markers[id] = {
            id,
            x: data.x,
            y: data.y || 1.2,
            z: data.z,
            nx: data.nx || 0,
            nz: data.nz || 1,
            edgeScore: data.edgeScore || 1,
            active: false,
            discoveredBy: playerId,
            activatedBy: null,
            timestamp: Date.now(),
          };
          broadcast({ type: 'markerSpawned', marker: state.markers[id] });
        }
      }

      // 3. 玩家靠近標記物 (< 2.2m) 激活標記物，作為定位基準並擴張伺服器地圖
      else if (data.type === 'activateMarker') {
        const m = state.markers[data.id];
        if (m) {
          m.active = true;
          m.activatedBy = playerId;
          m.timestamp = Date.now();
          if (Array.isArray(data.walls)) {
            data.walls.forEach((w) => {
              if (state.walls.length < 300) state.walls.push(w);
            });
          }
          broadcast({
            type: 'markerActivated',
            marker: m,
            playerId,
            walls: state.walls,
          });
        }
      }
    } catch (e) {
      console.error('WS parse error:', e);
    }
  });

  ws.on('close', () => {
    delete state.players[playerId];
    broadcast({ type: 'playerLeft', id: playerId });
  });
});

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`====================================================`);
  console.log(`AR 多人連機與空間定位伺服器已啟動：http://localhost:${PORT}`);
  console.log(`伺服器即時地圖監控端：http://localhost:${PORT}/dashboard.html`);
  console.log(`====================================================`);
});
