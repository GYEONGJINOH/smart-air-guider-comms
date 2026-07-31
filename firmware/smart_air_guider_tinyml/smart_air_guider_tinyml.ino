// Smart Air Guider - TinyML 인체 추종 노드 (ESP32-S3)
//
// 설계서: docs/tinyml-design.md
//
// 기존 firmware/smart_air_guider/ 와의 차이:
//   PIR 3존 하드코딩 좌표      ->  열화상 CNN + 레이더 융합 연속 좌표
//   WiFi 무한 블로킹           ->  제어 루프와 네트워크의 완전 분리
//   클라우드 없으면 동작 불가   ->  추론/제어/개인화가 전부 MCU 내부에서 완결
//
// 핵심 구조:
//   loop() {
//     runControlCycle();   // 8Hz. 네트워크 코드를 한 줄도 호출하지 않는다.
//     net.service();       // 남는 시간에 best-effort. 실패해도 즉시 리턴.
//   }
//
// 필요 라이브러리:
//   - Adafruit MLX90640 (+ Adafruit BusIO)
//   - ESP32Servo
//   - DHT sensor library (Adafruit)
//   - esp-tflite-micro   https://github.com/espressif/esp-tflite-micro
//     (또는 Edge Impulse Arduino 라이브러리 export)
//
// 보드: ESP32S3 Dev Module / PSRAM 활성 / Flash 8MB / Partition: Huge APP

#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>

#include "config.h"
#include "secrets.h"  // firmware/smart_air_guider/secrets.h.example 참고
#include "person_model.h"
#include "tracker.h"
#include "aim.h"
#include "offline.h"

PersonModel gModel;
RadarLD2450 gRadar;
Tracker gTracker;
Aimer gAimer;
ComfortNet gComfort;
OfflineNet gNet;
DHT dht(PIN_DHT, DHT22);

// 저하 사다리(설계서 §6.4) 추적용
bool gThermalOk = false;
uint32_t gLastThermalMs = 0;
uint32_t gLastPresentMs = 0;
uint32_t gLastControlMs = 0;
uint32_t gLastPublishMs = 0;
uint32_t gLastEnvMs = 0;
float gTemp = 25, gHum = 50, gPower = 0;
float gTempHistory = 25;  // 온도 변화율 계산용
const char* gMode = "대기";

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Smart Air Guider / TinyML]");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(1000000);  // MLX90640 8Hz를 안정적으로 읽으려면 Fast Mode+ 필요

  pinMode(PIN_PIR, INPUT);
  dht.begin();
  gAimer.begin();

#if USE_RADAR
  gRadar.begin(Serial1);
#endif

  gThermalOk = gModel.begin();
  Serial.printf("thermal+model: %s\n", gThermalOk ? "OK" : "FAIL (레이더 단독으로 동작)");

  // 네트워크는 마지막에, 그리고 실패해도 그냥 진행한다.
  gNet.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("제어 루프 시작 (네트워크 상태와 무관)");
}

// ---------------------------------------------------------------- 제어 주기
void runControlCycle() {
  Detection dets[MAX_PERSONS];
  int n = 0;

#if USE_RADAR
  gRadar.poll();
#endif

  if (gThermalOk && gModel.readFrame()) {
    n = gModel.infer(dets, MAX_PERSONS);
    gLastThermalMs = millis();
  }

  float dt = CONTROL_PERIOD_MS / 1000.0f;
  gTracker.update(dets, n, gRadar, dt);
  const TrackState& st = gTracker.state();

  // ---- 모드 결정
  AimMode aim = AIM_HOLD;
  uint32_t now = millis();

  if (st.present) {
    gLastPresentMs = now;

    // 쾌적도 판단: 사람 표면온도까지 쓰기 때문에 실내 온도만 보는 것보다 정확하다.
    ComfortInput ci{gTemp, gHum, st.surface_temp, st.z / 100.0f,
                    (gTemp - gTempHistory), (now - gLastPresentMs) / 60000.0f};
    ComfortClass c = gComfort.predict(ci);

    if (c == COMFORT_HOT) {
      aim = AIM_FOLLOW;
      gMode = (st.person_count > 1) ? "평균 조준" : "자동 추적";
    } else {
      aim = AIM_AVOID;
      gMode = "간접풍";
    }

    // 저하 사다리 표시 (대시보드에서 어떤 센서로 동작 중인지 보인다)
    if (!gThermalOk || now - gLastThermalMs > THERMAL_TIMEOUT_MS) {
      gMode = "자동 추적(레이더)";
    } else if (!st.radar_fused) {
      gMode = "자동 추적(열화상)";
    }
  } else if (now - gLastPresentMs > IDLE_TO_SLEEP_MS) {
    aim = AIM_NEUTRAL;
    gMode = "절전";
    // 여기서 IR로 에어컨 OFF (USE_IR_AC)
  } else if (now - gLastPresentMs > LOST_TIMEOUT_MS) {
    aim = AIM_HOLD;
    gMode = "대기";
  }

  gAimer.setTargetFrom(st, aim);
  gAimer.service();

  // ---- 리모컨 명령 (ESP-NOW / BLE / IR — 전부 인터넷 불필요)
  switch (gNet.takeRemoteCmd()) {
    case CMD_COOLER:
      gComfort.personalize(COMFORT_HOT);  // 사용자가 덥다고 알려준 것
      break;
    case CMD_WARMER:
      gComfort.personalize(COMFORT_COLD);
      break;
    case CMD_MODE_FOLLOW:
      gMode = "자동 추적";
      break;
    case CMD_MODE_AVOID:
      gMode = "간접풍";
      break;
    default:
      break;
  }

  // ---- 스냅샷 갱신 (로컬 HTTP 서버가 즉시 서빙 가능한 상태로 유지)
  gNet.snap.temperature = gTemp;
  gNet.snap.humidity = gHum;
  gNet.snap.power = gPower;
  gNet.snap.x = st.x;
  gNet.snap.y = st.y;
  gNet.snap.z = st.z;
  gNet.snap.yaw = gAimer.yawDeg();
  gNet.snap.pitch = gAimer.pitchDeg();
  gNet.snap.mode = gMode;
  gNet.snap.radar_fused = st.radar_fused;
  gNet.snap.infer_us = gModel.lastInferUs();
}

// ---------------------------------------------------------------- 환경 센서
void readEnvironment() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) {
    gTempHistory += 0.05f * (gTemp - gTempHistory);  // 5분 스케일 EMA
    gTemp = t;
  }
  if (!isnan(h)) gHum = h;
  // gPower: PZEM-004T(AC) 또는 INA219(DC)에서 읽어 채울 것
}

// ---------------------------------------------------------------- loop
void loop() {
  uint32_t now = millis();

  // ① 제어: 항상 정확히 8Hz로. 네트워크와 무관.
  if (now - gLastControlMs >= CONTROL_PERIOD_MS) {
    gLastControlMs = now;
    runControlCycle();
  }

  if (now - gLastEnvMs >= 2000) {
    gLastEnvMs = now;
    readEnvironment();
  }

  // ② 로깅: 인터넷 유무와 무관하게 항상 로컬 링버퍼에 적재
  if (now - gLastPublishMs >= PUBLISH_INTERVAL_MS) {
    gLastPublishMs = now;
    const TrackState& st = gTracker.state();
    LogRecord r{};
    r.ts = now;
    r.temp_x10 = (int16_t)(gTemp * 10);
    r.hum_x10 = (int16_t)(gHum * 10);
    r.power_x10 = (int16_t)(gPower * 10);
    r.x = (int16_t)st.x;
    r.y = (int16_t)st.y;
    r.z = (int16_t)st.z;
    r.yaw_x10 = (int16_t)(gAimer.yawDeg() * 10);
    r.pitch_x10 = (int16_t)(gAimer.pitchDeg() * 10);
    r.flags = (st.radar_fused ? 1 : 0) | (st.present ? 2 : 0);
    gNet.enqueue(r);

    Serial.printf("[%s] x=%.0f y=%.0f z=%.0f yaw=%.1f pitch=%.1f "
                  "infer=%.1fms net=%s q=%lu\n",
                  gMode, st.x, st.y, st.z, gAimer.yawDeg(), gAimer.pitchDeg(),
                  gModel.lastInferUs() / 1000.0f,
                  gNet.online() ? "on" : "OFF", (unsigned long)gNet.pending());
  }

  // ③ 네트워크: 남는 시간에만. 논블로킹.
  gNet.service();
}
