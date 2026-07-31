# Smart Air Guider - 통신 모듈

기존 프론트엔드([smart-air-guider.vercel.app](https://smart-air-guider.vercel.app/))는 2초마다
`GET https://smart-air-guider.onrender.com/sensor`를 폴링해서 온습도/소비전력/사용자 3D 위치/송풍 방향을
표시합니다. 이 저장소는 그 API 뒤에서 실제 센서 데이터를 만들어내는 통신 구간을 구현합니다.

## 아키텍처

```
[센서 노드: ESP32]  --MQTT(TLS) publish-->  [MQTT 브로커]
                                                  |
                                          MQTT subscribe
                                                  v
                                          [게이트웨이: Node.js]
                                                  |
                                        Socket.IO (실시간 push)
                                                  v
                            [백엔드: Express + Socket.IO] --GET /sensor(HTTP polling)--> [기존 프론트엔드]
```

- **firmware/smart_air_guider/** — ESP32 센서 노드(v1). DHT22(온습도), INA219(소비전력), PIR 3개(좌/중/우
  zone으로 사용자 방향 추정), 서보 2개(yaw/pitch)를 제어하고 MQTT로 결과를 발행합니다.
- **firmware/smart_air_guider_tinyml/** — ESP32-S3 TinyML 노드(v2). 열화상(MLX90640) + 24GHz 레이더
  (LD2450)를 온디바이스 CNN으로 융합해 실제 인체를 인식·추종합니다. **인터넷 없이 동작**하며
  네트워크는 관측/백필 전용입니다. 설계 근거는 [docs/tinyml-design.md](docs/tinyml-design.md) 참고.
- **tools/** — TinyML 데이터 수집(`capture_dataset.py`) 및 학습·양자화(`train_person_thermal.py`).
- **broker/** — 로컬 개발용 Mosquitto(docker-compose). 운영에서는 HiveMQ Cloud 등 TLS를 지원하는
  무료 클라우드 브로커 사용을 전제로 합니다(Render 무료 웹서비스는 raw TCP/MQTT를 직접 노출하지 못함).
- **gateway/** — MQTT 브로커를 구독해서 백엔드로 Socket.IO를 통해 실시간 중계하는 Node.js 프로세스.
- **server/** — Express 백엔드. 게이트웨이의 Socket.IO 연결(`sensor:update` 이벤트)로 최신 상태를
  갱신하고, 기존 프론트엔드가 쓰는 `GET /sensor`를 그대로 제공합니다. 필요하면 장치가 MQTT 없이
  `POST /sensor`로 바로 보낼 수도 있습니다(선택 사항).

## 데이터 스키마

```json
{
  "temperature": 24.5,
  "humidity": 45.2,
  "power": 12.3,
  "x": 100, "y": 150, "z": 200,
  "yaw": 30, "pitch": 0,
  "mode": "자동 추적"
}
```

## 로컬 개발 순서

1. **브로커 실행** (로컬 테스트용)
   ```
   cd broker && docker compose up -d
   ```
2. **백엔드 실행**
   ```
   cd server
   npm install
   set GATEWAY_TOKEN=change-me-shared-secret   # PowerShell: $env:GATEWAY_TOKEN="..."
   npm start
   ```
3. **게이트웨이 실행**
   ```
   cd gateway
   npm install
   copy .env.example .env   # MQTT_URL=mqtt://localhost:1883, BACKEND_URL=http://localhost:3000 로 로컬 테스트
   npm start
   ```
4. **센서 노드**: `firmware/smart_air_guider/secrets.h.example`을 `secrets.h`로 복사 후 WiFi/MQTT
   정보 입력, Arduino IDE로 ESP32에 업로드.

### MQTT 없이 빠르게 테스트하고 싶을 때

```
curl -X POST http://localhost:3000/sensor -H "Content-Type: application/json" \
  -d "{\"temperature\":24.5,\"humidity\":45,\"power\":12,\"x\":0,\"y\":150,\"z\":200,\"yaw\":0,\"pitch\":0,\"mode\":\"자동 추적\"}"
curl http://localhost:3000/sensor
```

## TinyML 노드 (v2, 오프라인 인체 추종)

v1의 PIR 3존 방식은 (a) 사람과 열원/반려동물을 구분하지 못하고, (b) 좌표가 하드코딩 상수 3개이며,
(c) `connectWiFi()`가 블로킹이라 인터넷이 끊기면 서보 제어까지 멈춥니다. v2는 이 세 가지를
온디바이스 추론과 오프라인 우선 구조로 해결합니다.

```
MLX90640 (32x24 열화상) ─┐
                         ├─▶ ESP32-S3 온디바이스 CNN ─▶ 칼만 추적 ─▶ 서보 2축 조준
LD2450 (24GHz 레이더)  ──┘        (약 30KB int8)
                                        │
                          로컬 HTTP / ESP-NOW / LoRa  ← 인터넷 불필요
                                        │
                          (연결될 때만) MQTT ─▶ gateway ─▶ server
```

인터넷이 끊긴 동안에도 노드가 동일 스키마의 `GET /sensor`를 로컬에서 서빙하므로, 프론트엔드는
fetch URL만 폴백 순서로 바꾸면 그대로 동작합니다:

```js
const ENDPOINTS = [
  "http://smartair.local/sensor",                  // 로컬 (인터넷 불필요)
  "http://192.168.4.1/sensor",                     // 노드 SoftAP 직결
  "https://smart-air-guider.onrender.com/sensor",  // 클라우드
];
```

빌드 순서:

1. 데이터 수집 — `python tools/capture_dataset.py --port COM5 --out dataset/s01.npz --minutes 20`
2. 학습/양자화 — `python tools/train_person_thermal.py --data dataset/`
3. 생성된 `model_person.h`를 `firmware/smart_air_guider_tinyml/`에 복사
4. Arduino IDE에서 보드 `ESP32S3 Dev Module`, Partition `Huge APP`로 업로드

## 운영 배포

- **백엔드**: Render Web Service (`smart-air-guider.onrender.com`). 환경변수: `DEVICE_API_KEY`(선택),
  `GATEWAY_TOKEN`.
- **MQTT 브로커**: HiveMQ Cloud 등 무료 TLS 브로커.
- **게이트웨이**: 상시 실행 가능한 아무 호스트(Render private service, Raspberry Pi, VPS 등)에서
  `node gateway.js`. `.env`에 브로커/백엔드 접속 정보 설정.
