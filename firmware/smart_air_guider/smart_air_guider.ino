// Smart Air Guider - 센서 노드 (ESP32)
//
// 역할: 온습도(DHT22) / 소비전력(INA219) / 사용자 위치(PIR 3존) 를 읽어
//       서보(yaw/pitch)로 송풍 방향을 제어하고, 결과를 MQTT 브로커로 발행한다.
//       (브로커 이후 단계는 gateway/ 가 구독해서 백엔드로 중계한다)
//
// 데이터 흐름:
//   [이 센서 노드] --MQTT publish(TLS)--> [MQTT 브로커] <--MQTT subscribe-- [게이트웨이] --Socket.IO--> [백엔드]
//
// 필요 라이브러리 (Arduino Library Manager):
//   - DHT sensor library (Adafruit) + Adafruit Unified Sensor
//   - Adafruit INA219
//   - ESP32Servo
//   - PubSubClient
//   - ArduinoJson
//
// 사용 전: secrets.h.example 을 secrets.h 로 복사하고 WiFi/MQTT 정보를 채울 것.

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_INA219.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include "secrets.h"

// ---------- 핀 설정 ----------
#define DHTPIN 4
#define DHTTYPE DHT22

#define SERVO_YAW_PIN 25
#define SERVO_PITCH_PIN 33

// PIR 3개를 좌/중/우에 배치해 대략적인 사용자 방향(zone)을 판별한다.
// HC-SR501은 존재유무만 감지하므로 x/y/z는 zone별 사전 정의 좌표를 사용한다.
// 실측값에 맞게 아래 zones[] 좌표/yaw를 조정할 것.
#define PIR_LEFT_PIN 14
#define PIR_CENTER_PIN 27
#define PIR_RIGHT_PIN 26

const unsigned long SEND_INTERVAL_MS = 2000; // 발행 주기
const float SERVO_STEP_DEG = 3.0; // 한 번의 loop당 서보가 움직이는 최대 각도(부드러운 추적)

struct Zone {
  const char* name;
  int pin;
  float x, y, z; // cm, 실측 후 보정 필요
  float yaw, pitch; // deg
};

Zone zones[] = {
  { "left",   PIR_LEFT_PIN,   -100, 150, 200, -30, 0 },
  { "center", PIR_CENTER_PIN,    0, 150, 200,   0, 0 },
  { "right",  PIR_RIGHT_PIN,   100, 150, 200,  30, 0 },
};
const int ZONE_COUNT = sizeof(zones) / sizeof(zones[0]);

DHT dht(DHTPIN, DHTTYPE);
Adafruit_INA219 ina219;
Servo yawServo;
Servo pitchServo;

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);
String publishTopic = String(MQTT_TOPIC_PREFIX) + NODE_ID + "/sensor";

// 마지막으로 감지된 사용자 위치/각도 (감지가 끊겨도 마지막 값을 유지)
float currentX = 0, currentY = 150, currentZ = 200;
float currentYaw = 0, currentPitch = 0;
float targetYaw = 0, targetPitch = 0;
const char* currentMode = "대기";

unsigned long lastSendMs = 0;

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi 연결됨. IP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  while (!mqttClient.connected()) {
    Serial.print("MQTT 브로커 연결 중...");
    // 개발 편의를 위해 인증서 검증을 생략한다.
    // 운영 환경에서는 setInsecure() 대신 브로커의 루트 CA를 setCACert()로 지정할 것.
    secureClient.setInsecure();

    String clientId = String("smart-air-guider-") + NODE_ID;
    if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("연결됨");
    } else {
      Serial.printf("실패, rc=%d. 2초 후 재시도\n", mqttClient.state());
      delay(2000);
    }
  }
}

// 활성화된 PIR zone의 인덱스를 반환, 없으면 -1
int readActiveZone() {
  for (int i = 0; i < ZONE_COUNT; i++) {
    if (digitalRead(zones[i].pin) == HIGH) {
      return i;
    }
  }
  return -1;
}

float stepTowards(float current, float target, float maxStep) {
  float diff = target - current;
  if (fabs(diff) <= maxStep) return target;
  return current + (diff > 0 ? maxStep : -maxStep);
}

void publishSensorData(float temperature, float humidity, float power) {
  StaticJsonDocument<256> doc;
  doc["nodeId"] = NODE_ID;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["power"] = power;
  doc["x"] = currentX;
  doc["y"] = currentY;
  doc["z"] = currentZ;
  doc["yaw"] = currentYaw;
  doc["pitch"] = currentPitch;
  doc["mode"] = currentMode;

  char payload[256];
  size_t len = serializeJson(doc, payload);

  bool ok = mqttClient.publish(publishTopic.c_str(), (uint8_t*)payload, len, false);
  Serial.printf("MQTT publish [%s] -> %s\n", publishTopic.c_str(), ok ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIR_LEFT_PIN, INPUT);
  pinMode(PIR_CENTER_PIN, INPUT);
  pinMode(PIR_RIGHT_PIN, INPUT);

  dht.begin();

  Wire.begin();
  if (!ina219.begin()) {
    Serial.println("INA219를 찾을 수 없습니다. 배선을 확인하세요.");
  }

  yawServo.setPeriodHertz(50);
  pitchServo.setPeriodHertz(50);
  yawServo.attach(SERVO_YAW_PIN, 500, 2400);
  pitchServo.attach(SERVO_PITCH_PIN, 500, 2400);
  yawServo.write(90 + currentYaw);
  pitchServo.write(90 + currentPitch);

  connectWiFi();
  connectMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  int zoneIdx = readActiveZone();

  if (zoneIdx >= 0) {
    Zone& z = zones[zoneIdx];
    currentX = z.x;
    currentY = z.y;
    currentZ = z.z;
    targetYaw = z.yaw;
    targetPitch = z.pitch;
    currentMode = "자동 추적";
  } else {
    currentMode = "대기";
    // 감지 없음: 마지막 위치를 유지하고 서보도 마지막 각도에 정지시킨다.
  }

  currentYaw = stepTowards(currentYaw, targetYaw, SERVO_STEP_DEG);
  currentPitch = stepTowards(currentPitch, targetPitch, SERVO_STEP_DEG);
  yawServo.write(constrain(90 + currentYaw, 0, 180));
  pitchServo.write(constrain(90 + currentPitch, 0, 180));

  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    if (isnan(temperature)) temperature = 0;
    if (isnan(humidity)) humidity = 0;

    float power = ina219.getPower_mW() / 1000.0; // W

    publishSensorData(temperature, humidity, power);
  }

  delay(50);
}
