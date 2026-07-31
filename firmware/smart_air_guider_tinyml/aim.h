// 조준 기구학 + 슬루 제한 서보 구동
//
// 추적 좌표(x,y,z)를 서보 각도(yaw,pitch)로 바꾸고, 급격한 기동/헌팅/소음을
// 억제하는 구동 정책을 적용한다. 대시보드의 yaw/pitch 필드가 여기서 나온다.
#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

#include "config.h"
#include "tracker.h"

enum AimMode {
  AIM_FOLLOW,   // 직접 조준
  AIM_AVOID,    // 간접풍 (사람 회피)
  AIM_HOLD,     // 마지막 각도 유지
  AIM_NEUTRAL,  // 중립 복귀
};

class Aimer {
 public:
  void begin() {
    yaw_.setPeriodHertz(50);
    pitch_.setPeriodHertz(50);
    attach();
    writeServos();
  }

  // 목표 각도 계산. 반환값은 명령 각도(슬루 적용 전).
  void setTargetFrom(const TrackState& s, AimMode mode) {
    float ty = cur_yaw_, tp = cur_pitch_;

    switch (mode) {
      case AIM_FOLLOW:
      case AIM_AVOID: {
        ty = atan2f(s.x, fmaxf(s.z, 30.0f)) * 180.0f / PI;
        float horiz = sqrtf(s.x * s.x + s.z * s.z);
        tp = atan2f(s.y - VENT_Y_CM, fmaxf(horiz, 30.0f)) * 180.0f / PI;

        // 냉기 제트 낙하 보정: 멀수록 위로 조준 (K_DROP은 실측 캘리브레이션)
        tp += K_DROP_DEG_PER_M * (s.z / 100.0f);

        if (mode == AIM_AVOID) ty += (ty >= 0 ? -25.0f : 25.0f);
        break;
      }
      case AIM_NEUTRAL:
        ty = 0;
        tp = 0;
        break;
      case AIM_HOLD:
      default:
        break;
    }

    ty = constrain(ty, SERVO_YAW_MIN_DEG, SERVO_YAW_MAX_DEG);
    tp = constrain(tp, SERVO_PITCH_MIN_DEG, SERVO_PITCH_MAX_DEG);

    // 드웰: 목표가 유의미하게 바뀐 뒤 일정 시간은 새 목표를 받지 않는다.
    // 두 사람 사이를 서보가 왕복하며 갈리는 것을 막는다.
    uint32_t now = millis();
    if (fabsf(ty - tgt_yaw_) > SERVO_DEADBAND_DEG ||
        fabsf(tp - tgt_pitch_) > SERVO_DEADBAND_DEG) {
      if (now - last_target_change_ms_ < TARGET_DWELL_MS) return;
      last_target_change_ms_ = now;
    }
    tgt_yaw_ = ty;
    tgt_pitch_ = tp;
  }

  // 제어 주기마다 호출. 슬루 제한 후 PWM 출력.
  void service() {
    float py = cur_yaw_, pp = cur_pitch_;
    cur_yaw_ = slew(cur_yaw_, tgt_yaw_, SERVO_SLEW_DEG_PER_CYCLE);
    cur_pitch_ = slew(cur_pitch_, tgt_pitch_, SERVO_SLEW_DEG_PER_CYCLE);

    bool moving = (cur_yaw_ != py) || (cur_pitch_ != pp);
    uint32_t now = millis();

    if (moving) {
      if (!attached_) attach();
      last_move_ms_ = now;
      writeServos();
    } else if (attached_ && now - last_move_ms_ > SERVO_IDLE_DETACH_MS) {
      // 정착 후 PWM 해제: 서보 지터·소음·대기전류 제거
      yaw_.detach();
      pitch_.detach();
      attached_ = false;
    }
  }

  float yawDeg() const { return cur_yaw_; }
  float pitchDeg() const { return cur_pitch_; }

 private:
  void attach() {
    yaw_.attach(PIN_SERVO_YAW, 500, 2400);
    pitch_.attach(PIN_SERVO_PITCH, 500, 2400);
    attached_ = true;
  }

  void writeServos() {
    yaw_.writeMicroseconds(
        (int)(SERVO_YAW_CENTER_US + cur_yaw_ * SERVO_US_PER_DEG));
    pitch_.writeMicroseconds(
        (int)(SERVO_PITCH_CENTER_US + cur_pitch_ * SERVO_US_PER_DEG));
  }

  static float slew(float cur, float tgt, float maxStep) {
    float d = tgt - cur;
    if (fabsf(d) <= SERVO_DEADBAND_DEG) return cur;  // 데드밴드
    if (fabsf(d) <= maxStep) return tgt;
    return cur + (d > 0 ? maxStep : -maxStep);
  }

  Servo yaw_, pitch_;
  float cur_yaw_ = 0, cur_pitch_ = 0;
  float tgt_yaw_ = 0, tgt_pitch_ = 0;
  bool attached_ = false;
  uint32_t last_move_ms_ = 0;
  uint32_t last_target_change_ms_ = 0;
};

// ---------------------------------------------------------------- 쾌적도 MLP
// 6 -> 16 -> 16 -> 3. 파라미터 약 400개(<1KB)라 TFLM 없이 직접 구현하는 편이
// 오히려 가볍다. 가중치는 tools/train_comfort.py 가 생성한다.
// 마지막 레이어만 온디바이스 SGD로 갱신해 사용자 개인화를 수행한다.
enum ComfortClass { COMFORT_HOT = 0, COMFORT_OK = 1, COMFORT_COLD = 2 };

struct ComfortInput {
  float air_temp, humidity, skin_temp, dist_m, temp_slope, dwell_min;
};

class ComfortNet {
 public:
  ComfortClass predict(const ComfortInput& in, float* prob_out = nullptr) {
    float x[6] = {(in.air_temp - 25.0f) / 5.0f,
                  (in.humidity - 50.0f) / 20.0f,
                  (in.skin_temp - 32.0f) / 3.0f,
                  (in.dist_m - 2.0f) / 2.0f,
                  in.temp_slope,
                  (in.dwell_min - 15.0f) / 15.0f};

    float h1[16], h2[16], o[3];
    dense(x, 6, w1_, b1_, h1, 16, true);
    dense(h1, 16, w2_, b2_, h2, 16, true);
    dense(h2, 16, w3_, b3_, o, 3, false);
    softmax(o, 3);
    memcpy(last_h2_, h2, sizeof(h2));

    if (prob_out) memcpy(prob_out, o, sizeof(o));
    int best = 0;
    for (int i = 1; i < 3; i++)
      if (o[i] > o[best]) best = i;
    memcpy(last_prob_, o, sizeof(o));
    return (ComfortClass)best;
  }

  // 사용자가 리모컨으로 "더 시원하게/따뜻하게"를 누른 순간의 정답 라벨로
  // 마지막 레이어만 1스텝 갱신한다. 클라우드 재학습 없이 개인화된다.
  void personalize(ComfortClass label, float lr = 0.05f) {
    for (int k = 0; k < 3; k++) {
      float grad = last_prob_[k] - (k == label ? 1.0f : 0.0f);
      for (int j = 0; j < 16; j++) w3_[j * 3 + k] -= lr * grad * last_h2_[j];
      b3_[k] -= lr * grad;
    }
  }

 private:
  static void dense(const float* in, int ni, const float* w, const float* b,
                    float* out, int no, bool relu) {
    for (int k = 0; k < no; k++) {
      float s = b[k];
      for (int j = 0; j < ni; j++) s += in[j] * w[j * no + k];
      out[k] = relu ? (s > 0 ? (s < 6 ? s : 6) : 0) : s;  // ReLU6
    }
  }
  static void softmax(float* v, int n) {
    float m = v[0];
    for (int i = 1; i < n; i++)
      if (v[i] > m) m = v[i];
    float s = 0;
    for (int i = 0; i < n; i++) {
      v[i] = expf(v[i] - m);
      s += v[i];
    }
    for (int i = 0; i < n; i++) v[i] /= s;
  }

  // tools/train_comfort.py 가 생성한 값으로 교체할 것 (아래는 플레이스홀더)
  float w1_[6 * 16] = {0}, b1_[16] = {0};
  float w2_[16 * 16] = {0}, b2_[16] = {0};
  float w3_[16 * 3] = {0}, b3_[3] = {0};
  float last_h2_[16] = {0};
  float last_prob_[3] = {0.33f, 0.34f, 0.33f};
};
