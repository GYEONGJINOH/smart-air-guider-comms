// LD2450 레이더 파서 + 열화상/레이더 융합 + 6상태 칼만 추적
//
// 융합 근거(설계서 §2.2): 두 센서의 실패 모드가 상보적이다.
//   - 레이더는 "정지한 사람"을 놓친다  -> 이때 열화상이 잡는다
//   - 열화상은 "실내 고온/두꺼운 옷"에 약하다 -> 이때 사람은 움직이므로 레이더가 잡는다
// 따라서 어느 한쪽이 죽어도 추적이 이어진다(설계서 §6.4 저하 사다리 L1/L2).
#pragma once

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "person_model.h"

struct RadarTarget {
  float x_cm, y_cm;   // 레이더 좌표계 (x: 좌우, y: 전방)
  float speed_cms;
  float az_deg;
  float range_cm;
  bool valid;
};

// ---------------------------------------------------------------- LD2450
// 프레임: AA FF 03 00 | 타깃1(8B) 타깃2(8B) 타깃3(8B) | 55 CC
// 좌표 부호 인코딩이 통상적인 2의 보수가 아니다(bit15 = 부호 플래그).
// 모듈 펌웨어 버전에 따라 다를 수 있으므로 실측으로 반드시 검증할 것.
class RadarLD2450 {
 public:
  void begin(HardwareSerial& s) {
    ser_ = &s;
    ser_->begin(RADAR_BAUD, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);
  }

  // 수신 버퍼를 비우며 최신 프레임을 파싱. 새 프레임을 얻으면 true.
  bool poll() {
    bool got = false;
    while (ser_ && ser_->available()) {
      uint8_t b = ser_->read();
      buf_[len_++] = b;
      if (len_ >= sizeof(buf_)) len_ = 0;

      if (len_ >= 4 && (buf_[0] != 0xAA || buf_[1] != 0xFF ||
                        buf_[2] != 0x03 || buf_[3] != 0x00)) {
        // 헤더 재동기화
        memmove(buf_, buf_ + 1, --len_);
        continue;
      }
      if (len_ == 30) {
        if (buf_[28] == 0x55 && buf_[29] == 0xCC) {
          parse();
          got = true;
          last_ms_ = millis();
        }
        len_ = 0;
      }
    }
    return got;
  }

  int count() const { return count_; }
  const RadarTarget& target(int i) const { return t_[i]; }
  bool alive() const { return millis() - last_ms_ < RADAR_TIMEOUT_MS; }

 private:
  static float decodeCoord(uint8_t lo, uint8_t hi) {
    uint16_t v = (uint16_t)lo | ((uint16_t)hi << 8);
    float mm = (v & 0x8000) ? (float)(v & 0x7FFF) : -(float)(v & 0x7FFF);
    return mm / 10.0f;  // mm -> cm
  }

  void parse() {
    count_ = 0;
    for (int i = 0; i < 3; i++) {
      const uint8_t* p = buf_ + 4 + i * 8;
      float x = decodeCoord(p[0], p[1]);
      float y = decodeCoord(p[2], p[3]);
      float sp = decodeCoord(p[4], p[5]);
      if (x == 0 && y == 0) continue;  // 빈 슬롯

      RadarTarget& t = t_[count_++];
      t.x_cm = x;
      t.y_cm = y;
      t.speed_cms = sp;
      t.range_cm = sqrtf(x * x + y * y);
      t.az_deg = atan2f(x, y) * 180.0f / PI;
      t.valid = true;
    }
  }

  HardwareSerial* ser_ = nullptr;
  uint8_t buf_[64];
  size_t len_ = 0;
  RadarTarget t_[3];
  int count_ = 0;
  uint32_t last_ms_ = 0;
};

// ------------------------------------------------------- 등속도 칼만 (3축 독립)
// 좌표 3축을 각각 [pos, vel] 2상태로 분리 운용한다. 6x6 완전 행렬 대비 연산량이
// 1/3 이하이고, 축간 상관이 실제로 거의 없어 성능 차이가 없다.
class Kalman1D {
 public:
  void reset(float p) {
    x_ = p;
    v_ = 0;
    P00_ = 100;
    P01_ = 0;
    P10_ = 0;
    P11_ = 100;
    init_ = true;
  }

  void predict(float dt, float q) {
    x_ += v_ * dt;
    float dt2 = dt * dt;
    P00_ += dt * (P10_ + P01_) + dt2 * P11_ + q * dt2 * dt2 / 4;
    P01_ += dt * P11_ + q * dt2 * dt / 2;
    P10_ += dt * P11_ + q * dt2 * dt / 2;
    P11_ += q * dt2;
  }

  void update(float z, float r) {
    if (!init_) {
      reset(z);
      return;
    }
    float y = z - x_;
    float s = P00_ + r;
    float k0 = P00_ / s, k1 = P10_ / s;
    x_ += k0 * y;
    v_ += k1 * y;
    float p00 = P00_, p01 = P01_;
    P00_ -= k0 * p00;
    P01_ -= k0 * p01;
    P10_ -= k1 * p00;
    P11_ -= k1 * p01;
  }

  float pos() const { return x_; }
  float vel() const { return v_; }
  bool initialized() const { return init_; }

 private:
  float x_ = 0, v_ = 0;
  float P00_ = 100, P01_ = 0, P10_ = 0, P11_ = 100;
  bool init_ = false;
};

struct TrackState {
  bool present = false;
  int person_count = 0;
  float x = 0, y = 0, z = 200;  // cm, 토출구 좌표계 (대시보드 스키마)
  float az = 0, el = 0;
  float score = 0;
  float surface_temp = 0;
  bool radar_fused = false;
  uint32_t last_seen_ms = 0;
};

class Tracker {
 public:
  // 열화상 검출 목록 + 레이더 타깃을 융합해 추적 상태를 갱신
  void update(const Detection* dets, int n, RadarLD2450& radar, float dt) {
    kx_.predict(dt, 400.0f);
    ky_.predict(dt, 200.0f);
    kz_.predict(dt, 900.0f);

    state_.person_count = n;

    if (n == 0) {
      // 열화상 미검출 -> 레이더 단독 폴백 (저하 사다리 L2)
      if (radar.alive() && radar.count() > 0) {
        const RadarTarget& t = radar.target(0);
        applyMeasurement(t.az_deg, 0.0f, t.range_cm, 0.5f, state_.surface_temp,
                         true);
        state_.person_count = radar.count();
        return;
      }
      if (millis() - state_.last_seen_ms > LOST_TIMEOUT_MS) {
        state_.present = false;
      }
      return;
    }

    // 다중 인원: 점수 가중 평균 (mode="평균 조준"에 대응).
    // "교대 조준"을 쓰려면 상위 검출을 SWEEP_DWELL_MS 마다 선택하면 된다.
    float wsum = 0, az = 0, el = 0, st = 0;
    int best = 0;
    for (int i = 0; i < n; i++) {
      float w = dets[i].score;
      az += dets[i].az_deg * w;
      el += dets[i].el_deg * w;
      st += dets[i].surface_temp * w;
      wsum += w;
      if (dets[i].score > dets[best].score) best = i;
    }
    az /= wsum;
    el /= wsum;
    st /= wsum;

    // ---- 레이더 연관: 방위각이 가장 가까운 타깃의 레인지를 채택
    float range = dets[best].depth_cm;
    bool fused = false;
    if (radar.alive()) {
      float bestDiff = ASSOC_GATE_DEG;
      for (int i = 0; i < radar.count(); i++) {
        float d = fabsf(radar.target(i).az_deg - az);
        if (d < bestDiff) {
          bestDiff = d;
          range = radar.target(i).range_cm;
          fused = true;
        }
      }
    }

    applyMeasurement(az, el, range, dets[best].score, st, fused);
  }

  const TrackState& state() const { return state_; }

 private:
  void applyMeasurement(float az, float el, float range, float score,
                        float surf, bool fused) {
    float a = az * PI / 180.0f, e = el * PI / 180.0f;
    float mz = range * cosf(e) * cosf(a);
    float mx = range * cosf(e) * sinf(a);
    float my = range * sinf(e);

    // 센서 -> 토출구 좌표 변환
    mx += SENSOR_OFFSET_X;
    my += SENSOR_OFFSET_Y;
    mz += SENSOR_OFFSET_Z;

    // 확신이 낮은 검출은 측정 노이즈를 키워 자동으로 덜 반영되게 한다
    float r = 100.0f / fmaxf(score, 0.2f);
    float rz = fused ? r : r * 4.0f;  // 레이더 미연관 시 깊이 신뢰도 하락

    kx_.update(mx, r);
    ky_.update(my, r);
    kz_.update(mz, rz);

    state_.x = kx_.pos();
    state_.y = ky_.pos();
    state_.z = kz_.pos();
    state_.az = atan2f(state_.x, state_.z) * 180.0f / PI;
    state_.el = atan2f(state_.y - VENT_Y_CM,
                       sqrtf(state_.x * state_.x + state_.z * state_.z)) *
                180.0f / PI;
    state_.score = score;
    state_.surface_temp = surf;
    state_.radar_fused = fused;
    state_.present = true;
    state_.last_seen_ms = millis();
  }

  Kalman1D kx_, ky_, kz_;
  TrackState state_;
};
