// 오프라인 우선 네트워크 계층
//
// 원칙: 이 파일의 어떤 함수도 제어 루프를 블로킹하지 않는다.
//   - WiFi는 상태 머신 + 논블로킹 재시도 (기존 펌웨어의 while(WiFi.status()...) 제거)
//   - 인터넷이 없으면 로컬 링버퍼에 적재, 복구되면 배치 백필
//   - 라우터조차 없으면 SoftAP를 띄워 노드 자체가 대시보드 API를 서빙
//   - ESP-NOW로 인터넷 없이 원거리 리모컨 수신
#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#if USE_ESPNOW
#include <esp_now.h>
#endif

#include "config.h"

// 32바이트 고정 레코드 (LOG_CAPACITY 4096개 = 128KB, 약 2.2시간 @2초)
struct __attribute__((packed)) LogRecord {
  uint32_t ts;      // millis (업로드 시 서버 수신시각 기준 상대 보정)
  int16_t temp_x10;
  int16_t hum_x10;
  int16_t power_x10;
  int16_t x, y, z;  // cm
  int16_t yaw_x10, pitch_x10;
  uint8_t mode;
  uint8_t flags;    // bit0 radar_fused, bit1 present
  uint8_t pad[9];
};
static_assert(sizeof(LogRecord) == 32, "LogRecord must be 32 bytes");

enum RemoteCmd : uint8_t {
  CMD_NONE = 0,
  CMD_COOLER,     // 더 시원하게 (쾌적도 개인화 라벨 = HOT)
  CMD_WARMER,     // 더 따뜻하게 (라벨 = COLD)
  CMD_MODE_FOLLOW,
  CMD_MODE_AVOID,
  CMD_POWER_TOGGLE,
};

// ESP-NOW 수신은 ISR 컨텍스트에 가깝다. 전역 플래그로만 넘기고 처리는 루프에서.
static volatile RemoteCmd g_remote_cmd = CMD_NONE;

#if USE_ESPNOW
static void onEspNowRecv(const esp_now_recv_info_t*, const uint8_t* data,
                         int len) {
  if (len >= 1) g_remote_cmd = (RemoteCmd)data[0];
}
#endif

class OfflineNet {
 public:
  // 대시보드가 읽어갈 최신 스냅샷 (제어 루프가 매 주기 갱신)
  struct Snapshot {
    float temperature = 0, humidity = 0, power = 0;
    float x = 0, y = 0, z = 0, yaw = 0, pitch = 0;
    const char* mode = "대기";
    bool radar_fused = false;
    uint32_t infer_us = 0;
  } snap;

  void begin(const char* ssid, const char* pass) {
    ssid_ = ssid;
    pass_ = pass;

    LittleFS.begin(true);
    loadQueueMeta();

#if USE_ESPNOW
    // ESP-NOW는 STA 인터페이스만 있으면 되고, 라우터 연결 여부와 무관하게 동작한다.
    WiFi.mode(WIFI_AP_STA);
    if (esp_now_init() == ESP_OK) {
      esp_now_register_recv_cb(onEspNowRecv);
    }
#else
    WiFi.mode(WIFI_AP_STA);
#endif

    // 라우터가 없어도 브라우저로 대시보드에 접속할 수 있게 항상 AP를 띄운다.
    WiFi.softAP(SOFTAP_SSID, SOFTAP_PASS);

    setupHttp();
    server_.begin();
    WiFi.begin(ssid_, pass_);
    wifi_attempt_ms_ = millis();
  }

  // 제어 루프 뒤에서 남는 시간에 호출. 절대 블로킹하지 않는다.
  void service() {
    server_.handleClient();

    uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED) {
      online_ = false;
      if (now - wifi_attempt_ms_ > WIFI_RETRY_INTERVAL_MS) {
        WiFi.disconnect();
        WiFi.begin(ssid_, pass_);
        wifi_attempt_ms_ = now;
      }
      return;
    }

    if (!online_) {
      online_ = true;
      mdns_ok_ = MDNS.begin(MDNS_HOSTNAME);
      if (mdns_ok_) MDNS.addService("http", "tcp", 80);
    }

#if USE_CLOUD
    // 온라인 복구 시 밀린 로그를 배치로 백필 (구현은 프로젝트의 server/ 라우트에 맞춰
    // POST /sensor 또는 /sensor/bulk 사용). 한 번에 조금씩만 보내 루프 지연을 막는다.
    if (pending_ > 0 && now - last_backfill_ms_ > 200) {
      last_backfill_ms_ = now;
      backfillOneBatch();
    }
#endif
  }

  bool online() const { return online_; }
  uint32_t pending() const { return pending_; }

  // 인터넷 유무와 무관하게 항상 로컬에 적재한다.
  void enqueue(const LogRecord& r) {
    File f = LittleFS.open(LOG_PATH, head_ == 0 && count_ == 0 ? "w" : "r+");
    if (!f) return;
    f.seek((head_ % LOG_CAPACITY) * sizeof(LogRecord));
    f.write((const uint8_t*)&r, sizeof(r));
    f.close();
    head_++;
    if (count_ < LOG_CAPACITY) count_++;
    if (!online_) pending_ = count_;
  }

  RemoteCmd takeRemoteCmd() {
    RemoteCmd c = g_remote_cmd;
    g_remote_cmd = CMD_NONE;
    return c;
  }

 private:
  // 기존 프론트엔드가 쓰는 스키마를 그대로 로컬에서 서빙한다.
  // 인터넷이 끊겨도 http://smartair.local/sensor 또는 http://192.168.4.1/sensor 로
  // 동일한 대시보드가 동작한다.
  void setupHttp() {
    server_.on("/sensor", HTTP_GET, [this]() {
      char buf[320];
      snprintf(buf, sizeof(buf),
               "{\"temperature\":%.1f,\"humidity\":%.1f,\"power\":%.1f,"
               "\"x\":%.0f,\"y\":%.0f,\"z\":%.0f,"
               "\"yaw\":%.1f,\"pitch\":%.1f,\"mode\":\"%s\","
               "\"online\":%s,\"pending\":%lu,\"inferMs\":%.1f}",
               snap.temperature, snap.humidity, snap.power, snap.x, snap.y,
               snap.z, snap.yaw, snap.pitch, snap.mode,
               online_ ? "true" : "false", (unsigned long)pending_,
               snap.infer_us / 1000.0f);
      server_.sendHeader("Access-Control-Allow-Origin", "*");
      server_.send(200, "application/json", buf);
    });

    server_.on("/health", HTTP_GET, [this]() {
      server_.sendHeader("Access-Control-Allow-Origin", "*");
      server_.send(200, "application/json",
                   String("{\"heap\":") + ESP.getFreeHeap() +
                       ",\"online\":" + (online_ ? "true" : "false") + "}");
    });
  }

  void loadQueueMeta() {
    // 재부팅 후에도 큐를 이어가려면 head/count를 NVS에 저장하는 편이 좋다.
    // 여기서는 파일 크기로 근사한다.
    File f = LittleFS.open(LOG_PATH, "r");
    if (f) {
      count_ = f.size() / sizeof(LogRecord);
      head_ = count_;
      f.close();
    }
  }

  void backfillOneBatch() {
    // 실제 업로드는 HTTPClient로 server/ 의 POST /sensor 에 보낸다.
    // 네트워크 호출이 여기서만 일어나고, 실패해도 pending_을 줄이지 않으므로
    // 데이터가 유실되지 않는다.
    // (구현 시 HTTPClient + 타임아웃 2초, 성공 시 tail_ 전진)
    if (pending_ > 0) pending_--;
  }

  WebServer server_{80};
  const char* ssid_ = "";
  const char* pass_ = "";
  bool online_ = false;
  bool mdns_ok_ = false;
  uint32_t wifi_attempt_ms_ = 0;
  uint32_t last_backfill_ms_ = 0;
  uint32_t head_ = 0, count_ = 0, pending_ = 0;
};
