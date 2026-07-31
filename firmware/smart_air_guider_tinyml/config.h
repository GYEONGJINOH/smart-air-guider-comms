// Smart Air Guider - TinyML 노드 설정
//
// 기구 상수는 반드시 실측 후 보정할 것. 특히 SENSOR_OFFSET_*, K_DROP_DEG_PER_M,
// SERVO_*_CENTER_US 는 설치 환경마다 다르다.
#pragma once

// ===================== 기능 플래그 =====================
#define USE_THERMAL 1   // MLX90640 (I2C)
#define USE_RADAR 1     // HLK-LD2450 (UART)
#define USE_IR_AC 0     // IR로 에어컨 본체 제어 (IRremoteESP8266 필요)
#define USE_ESPNOW 1    // 인터넷 없이 원거리 리모컨
#define USE_CLOUD 1     // 인터넷이 될 때만 best-effort 업로드

// ===================== 핀 =====================
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

#define PIN_RADAR_RX 17  // LD2450 TX -> ESP32 RX
#define PIN_RADAR_TX 18  // LD2450 RX <- ESP32 TX
#define RADAR_BAUD 256000

#define PIN_DHT 4
#define PIN_SERVO_YAW 5
#define PIN_SERVO_PITCH 6
#define PIN_PIR 7  // 웨이크업 트리거 (절전 모드 탈출)
#define PIN_IR_LED 15

// ===================== 열화상 (MLX90640) =====================
#define TH_W 32
#define TH_H 24
#define TH_PIXELS (TH_W * TH_H)
#define MLX_ADDR 0x33
#define MLX_REFRESH_HZ 8

// 렌즈 화각. 55x35(BAA) 또는 110x75(BAB). 모듈 사양에 맞출 것.
#define TH_HFOV_DEG 55.0f
#define TH_VFOV_DEG 35.0f

// ===================== 모델 =====================
#define MODEL_IN_W 32
#define MODEL_IN_H 24
#define MODEL_IN_C 2   // [정규화 프레임, 1초 시간차분]
#define MODEL_OUT_W 8  // 히트맵 격자
#define MODEL_OUT_H 6
#define TENSOR_ARENA_BYTES (60 * 1024)

#define DETECT_THRESHOLD 0.55f  // 히트맵 sigmoid 임계값
#define MAX_PERSONS 3

// ===================== 기구 형상 (cm) =====================
// 센서 원점 기준으로 측정한 "토출구 중심"의 상대 위치
#define SENSOR_OFFSET_X 0.0f
#define SENSOR_OFFSET_Y -12.0f  // 센서가 토출구보다 12cm 위에 달린 경우
#define SENSOR_OFFSET_Z 0.0f
#define MOUNT_HEIGHT_CM 220.0f  // 바닥 기준 센서 설치 높이
#define VENT_Y_CM 0.0f          // 토출구 y (센서 좌표계)

// 냉기 제트 낙하 보정: 먼 거리일수록 위로 조준한다. 실측 캘리브레이션 필요.
#define K_DROP_DEG_PER_M 1.5f

// ===================== 서보 =====================
#define SERVO_YAW_MIN_DEG -60.0f
#define SERVO_YAW_MAX_DEG 60.0f
#define SERVO_PITCH_MIN_DEG -20.0f
#define SERVO_PITCH_MAX_DEG 30.0f

#define SERVO_YAW_CENTER_US 1500
#define SERVO_PITCH_CENTER_US 1500
#define SERVO_US_PER_DEG 10.0f  // 대부분의 표준 서보: 180도 = 1000..2000us

#define SERVO_SLEW_DEG_PER_CYCLE 3.0f  // 8Hz 기준 24 deg/s
#define SERVO_DEADBAND_DEG 2.0f
#define SERVO_IDLE_DETACH_MS 500  // 정착 후 PWM 해제(지터/소음/전류 제거)

// ===================== 제어 루프 =====================
#define CONTROL_HZ 8
#define CONTROL_PERIOD_MS (1000 / CONTROL_HZ)
#define LOST_TIMEOUT_MS 2000     // 미검출 -> "대기"
#define IDLE_TO_SLEEP_MS 300000  // 5분 무인 -> 절전
#define TARGET_DWELL_MS 1500     // 다중 인원 왕복 방지
#define SWEEP_DWELL_MS 20000     // 교대 조준 유지 시간

// ===================== 융합 =====================
#define ASSOC_GATE_DEG 8.0f  // 열화상 방위각 <-> 레이더 타깃 연관 게이트
#define RADAR_TIMEOUT_MS 1000
#define THERMAL_TIMEOUT_MS 1000

// ===================== 네트워크 (제어 경로 아님) =====================
#define PUBLISH_INTERVAL_MS 2000
#define WIFI_RETRY_INTERVAL_MS 30000  // 논블로킹 재시도 주기
#define WIFI_ATTEMPT_TIMEOUT_MS 8000
#define MDNS_HOSTNAME "smartair"
#define SOFTAP_SSID "SmartAirGuider"
#define SOFTAP_PASS "airguider"

// 로컬 링버퍼(LittleFS)
#define LOG_PATH "/queue.bin"
#define LOG_CAPACITY 4096  // 레코드 수
