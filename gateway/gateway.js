require('dotenv').config();
const http = require('http');
const express = require('express');
const cors = require('cors');
const mqtt = require('mqtt');

const {
  PORT = 3000,
  MQTT_URL,
  MQTT_USERNAME,
  MQTT_PASSWORD,
  MQTT_TOPIC = 'smart-air-guider/nodes/+/sensor',
  DEVICE_API_KEY = '', // 장치가 MQTT 없이 직접 HTTP POST할 때 검사(선택)
} = process.env;

if (!MQTT_URL) {
  console.error('MQTT_URL은 필수입니다. .env를 확인하세요.');
  process.exit(1);
}

const ALLOWED_ORIGINS = [
  'https://smart-air-guider.vercel.app',
  'http://localhost:5173',
];

// ---------- 상태 (게이트웨이·백엔드가 공유하는 최신 센서값) ----------
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

// ---------- 게이트웨이 기능: MQTT 브로커 구독 ----------
const mqttClient = mqtt.connect(MQTT_URL, {
  username: MQTT_USERNAME,
  password: MQTT_PASSWORD,
  reconnectPeriod: 2000,
});

mqttClient.on('connect', () => {
  console.log('MQTT 브로커 연결됨');
  mqttClient.subscribe(MQTT_TOPIC, (err) => {
    if (err) console.error('구독 실패:', err.message);
    else console.log('구독 중:', MQTT_TOPIC);
  });
});

mqttClient.on('reconnect', () => console.log('MQTT 재연결 시도 중...'));
mqttClient.on('error', (err) => console.error('MQTT 오류:', err.message));

mqttClient.on('message', (topic, messageBuf) => {
  let payload;
  try {
    payload = JSON.parse(messageBuf.toString());
  } catch (err) {
    console.error('잘못된 JSON 페이로드:', topic, err.message);
    return;
  }
  console.log('센서 데이터 수신:', topic, payload);
  // 별도 백엔드로 중계하지 않고 이 프로세스 안에서 바로 상태에 반영한다.
  applyUpdate(payload);
});

// ---------- 백엔드 기능: 프론트엔드가 폴링하는 HTTP API ----------
const app = express();
app.use(cors({ origin: ALLOWED_ORIGINS }));
app.use(express.json({ limit: '10kb' }));

// 프론트엔드가 2초마다 폴링하는 엔드포인트 (기존 그대로 유지).
app.get('/sensor', (req, res) => {
  res.json(latest);
});

// 장치가 MQTT 없이 직접 HTTP로 보낼 때를 위한 경로 (선택 사항).
app.post('/sensor', (req, res) => {
  if (DEVICE_API_KEY && req.get('x-api-key') !== DEVICE_API_KEY) {
    return res.status(401).json({ error: 'invalid api key' });
  }
  applyUpdate(req.body);
  res.status(204).end();
});

app.get('/health', (req, res) => {
  res.json({
    status: 'ok',
    mqttConnected: mqttClient.connected,
    lastUpdate: latest.updatedAt,
  });
});

const httpServer = http.createServer(app);
httpServer.listen(PORT, () => {
  console.log(`gateway+backend listening on port ${PORT}`);
});
