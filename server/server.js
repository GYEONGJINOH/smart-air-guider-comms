const http = require('http');
const express = require('express');
const cors = require('cors');
const { Server } = require('socket.io');

const app = express();
const PORT = process.env.PORT || 3000;
const DEVICE_API_KEY = process.env.DEVICE_API_KEY || ''; // 직접 HTTP POST하는 장치용
const GATEWAY_TOKEN = process.env.GATEWAY_TOKEN || ''; // 게이트웨이 Socket.IO 인증용

const ALLOWED_ORIGINS = [
  'https://smart-air-guider.vercel.app',
  'http://localhost:5173',
];

app.use(cors({ origin: ALLOWED_ORIGINS }));
app.use(express.json({ limit: '10kb' }));

// 최신 센서 상태 (in-memory). 프로세스 재시작 시 초기화됨.
let latest = {
  temperature: null,
  humidity: null,
  power: null,
  x: null,
  y: null,
  z: null,
  yaw: null,
  pitch: null,
  mode: '대기',
  updatedAt: null,
};

const NUMERIC_FIELDS = ['temperature', 'humidity', 'power', 'x', 'y', 'z', 'yaw', 'pitch'];

function applyUpdate(body = {}) {
  const next = { ...latest };
  for (const field of NUMERIC_FIELDS) {
    if (typeof body[field] === 'number' && Number.isFinite(body[field])) {
      next[field] = body[field];
    }
  }
  if (typeof body.mode === 'string') {
    next.mode = body.mode;
  }
  next.updatedAt = new Date().toISOString();
  latest = next;
  return latest;
}

// 프론트엔드가 2초마다 폴링하는 엔드포인트 (기존 그대로 유지).
app.get('/sensor', (req, res) => {
  res.json(latest);
});

// 장치가 MQTT/게이트웨이 없이 직접 HTTP로 보낼 때를 위한 경로 (선택 사항).
app.post('/sensor', (req, res) => {
  if (DEVICE_API_KEY && req.get('x-api-key') !== DEVICE_API_KEY) {
    return res.status(401).json({ error: 'invalid api key' });
  }
  applyUpdate(req.body);
  res.status(204).end();
});

app.get('/health', (req, res) => {
  res.json({ status: 'ok', lastUpdate: latest.updatedAt });
});

const httpServer = http.createServer(app);
const io = new Server(httpServer, { cors: { origin: ALLOWED_ORIGINS } });

// 게이트웨이만 연결을 허용 (공유 토큰 검사).
io.use((socket, next) => {
  if (GATEWAY_TOKEN && socket.handshake.auth?.token !== GATEWAY_TOKEN) {
    return next(new Error('unauthorized'));
  }
  next();
});

io.on('connection', (socket) => {
  console.log('게이트웨이 연결됨:', socket.id);

  socket.on('sensor:update', (payload) => {
    const updated = applyUpdate(payload);
    // 실시간 갱신이 필요한 다른 클라이언트(대시보드 등)에도 전파.
    socket.broadcast.emit('sensor:update', updated);
  });

  socket.on('disconnect', (reason) => {
    console.log('게이트웨이 연결 끊김:', socket.id, reason);
  });
});

httpServer.listen(PORT, () => {
  console.log(`smart-air-guider server listening on port ${PORT}`);
});
