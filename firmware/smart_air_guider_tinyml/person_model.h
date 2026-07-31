// 열화상 전처리 + TinyML 추론 + 히트맵 디코딩
//
// 파이프라인:
//   MLX90640 raw(℃) -> 배경 차분/대비 정규화 -> int8 [24,32,2] -> CNN
//   -> heatmap(6x8) / offset(6x8x2) / logdepth(6x8) -> 방위각·고도각·거리
//
// 전처리에서 "프레임 중앙값 기준 정규화"를 쓰는 것이 핵심이다. 냉방 중 실내 온도가
// 22℃에서 30℃까지 변해도 모델 입력 분포가 흔들리지 않는다.
//
// 라이브러리: esp-tflite-micro (ESP32-S3에서 ESP-NN int8 가속)
//   https://github.com/espressif/esp-tflite-micro
//   ※ Arduino 공식 Arduino_TensorFlowLite 는 유지보수 중단 상태이므로 쓰지 않는다.
#pragma once

#include <Arduino.h>
#include <math.h>

#include "config.h"

#if USE_THERMAL
#include <Adafruit_MLX90640.h>
#endif

#include "model_person.h"  // xxd -i 로 생성된 int8 모델 배열
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

struct Detection {
  float az_deg;    // 방위각 (+오른쪽)
  float el_deg;    // 고도각 (+위)
  float depth_cm;  // CNN 추정 거리(레이더 미연관 시 폴백)
  float score;
  float surface_temp;  // 인물 패치 평균 온도 (쾌적도 MLP 입력)
};

class PersonModel {
 public:
  bool begin() {
#if USE_THERMAL
    if (!mlx_.begin(MLX_ADDR, &Wire)) return false;
    mlx_.setMode(MLX90640_CHESS);
    mlx_.setResolution(MLX90640_ADC_18BIT);
    mlx_.setRefreshRate(MLX90640_8_HZ);
#endif

    model_ = tflite::GetModel(g_model_person);
    if (model_->version() != TFLITE_SCHEMA_VERSION) return false;

    // 사용하는 연산자만 등록해야 바이너리가 작아진다.
    resolver_.AddConv2D();
    resolver_.AddDepthwiseConv2D();
    resolver_.AddAdd();
    resolver_.AddLogistic();
    resolver_.AddQuantize();
    resolver_.AddDequantize();
    resolver_.AddReshape();

    static tflite::MicroInterpreter interp(model_, resolver_, arena_,
                                           TENSOR_ARENA_BYTES);
    interpreter_ = &interp;
    if (interpreter_->AllocateTensors() != kTfLiteOk) return false;

    input_ = interpreter_->input(0);
    // 학습 스크립트의 출력 순서와 일치해야 한다: [heatmap, offset, logdepth]
    out_heat_ = interpreter_->output(0);
    out_off_ = interpreter_->output(1);
    out_depth_ = interpreter_->output(2);

    for (int i = 0; i < TH_PIXELS; i++) {
      bg_[i] = NAN;
      prev_[i] = NAN;
    }
    return true;
  }

  // 열화상 프레임을 읽어 전처리까지 수행. 실패 시 false.
  bool readFrame() {
#if USE_THERMAL
    if (mlx_.getFrame(raw_) != 0) return false;
#else
    return false;
#endif

    // ---- 배경 모델: 픽셀별 느린 EMA. 정지 열원(노트북/햇빛 벽면)을 흡수한다.
    // 사람이 오래 머물러도 지워지지 않도록 시정수를 아주 길게(약 30초) 잡는다.
    const float kBgAlpha = 1.0f / (MLX_REFRESH_HZ * 30.0f);
    for (int i = 0; i < TH_PIXELS; i++) {
      if (isnan(bg_[i])) bg_[i] = raw_[i];
      bg_[i] += kBgAlpha * (raw_[i] - bg_[i]);
    }

    // ---- 프레임 통계(중앙값 근사 + 표준편차)
    float sum = 0, sum2 = 0;
    for (int i = 0; i < TH_PIXELS; i++) {
      sum += raw_[i];
      sum2 += raw_[i] * raw_[i];
    }
    float mean = sum / TH_PIXELS;
    float var = sum2 / TH_PIXELS - mean * mean;
    float sd = sqrtf(var > 0 ? var : 0.01f);
    float scale = fmaxf(3.0f * sd, 1.5f);  // 최소 1.5K로 하한 → 균일 장면 폭주 방지
    ref_temp_ = mean;

    // ---- int8 양자화 입력 채널 2개 구성
    int8_t* in = input_->data.int8;
    const float iscale = input_->params.scale;
    const int izp = input_->params.zero_point;

    for (int i = 0; i < TH_PIXELS; i++) {
      float n = (raw_[i] - mean) / scale;              // ch0: 대비 정규화
      float d = isnan(prev_[i]) ? 0.0f                 // ch1: 1초 시간차분
                                : (raw_[i] - prev_[i]) / scale;
      n = constrain(n, -1.0f, 3.0f);
      d = constrain(d, -2.0f, 2.0f);

      in[i * MODEL_IN_C + 0] = quantize(n, iscale, izp);
      in[i * MODEL_IN_C + 1] = quantize(d, iscale, izp);
    }

    // 1초(=MLX_REFRESH_HZ 프레임) 전 프레임을 차분 기준으로 유지
    if (++frame_count_ % MLX_REFRESH_HZ == 0) {
      memcpy(prev_, raw_, sizeof(raw_));
    }
    return true;
  }

  // 추론 + 히트맵 디코딩. 검출 개수를 반환.
  int infer(Detection* out, int max_out) {
    uint32_t t0 = micros();
    if (interpreter_->Invoke() != kTfLiteOk) return 0;
    last_infer_us_ = micros() - t0;

    const int8_t* heat = out_heat_->data.int8;
    const float hs = out_heat_->params.scale;
    const int hz = out_heat_->params.zero_point;

    // 로컬 맥시마 추출 (3x3 max pooling과 동치 → NMS 불필요)
    int found = 0;
    for (int cv = 0; cv < MODEL_OUT_H && found < max_out; cv++) {
      for (int cu = 0; cu < MODEL_OUT_W && found < max_out; cu++) {
        int idx = cv * MODEL_OUT_W + cu;
        float s = (heat[idx] - hz) * hs;
        if (s < DETECT_THRESHOLD) continue;
        if (!isLocalMax(heat, cu, cv)) continue;

        float ox = dq(out_off_, idx * 2 + 0);
        float oy = dq(out_off_, idx * 2 + 1);

        // 격자 좌표 -> 원본 픽셀 좌표
        float u = (cu + 0.5f + ox) * ((float)TH_W / MODEL_OUT_W);
        float v = (cv + 0.5f + oy) * ((float)TH_H / MODEL_OUT_H);
        u = constrain(u, 0.0f, (float)TH_W - 1);
        v = constrain(v, 0.0f, (float)TH_H - 1);

        Detection& d = out[found++];
        d.az_deg = (u / (TH_W - 1) - 0.5f) * TH_HFOV_DEG;
        d.el_deg = -(v / (TH_H - 1) - 0.5f) * TH_VFOV_DEG;
        d.depth_cm = expf(dq(out_depth_, idx)) * 100.0f;  // log(m) -> cm
        d.score = s;
        d.surface_temp = patchMean((int)u, (int)v, 2);
      }
    }
    return found;
  }

  float ambientRef() const { return ref_temp_; }
  uint32_t lastInferUs() const { return last_infer_us_; }
  const float* raw() const { return raw_; }

 private:
  static int8_t quantize(float v, float scale, int zp) {
    int q = (int)lroundf(v / scale) + zp;
    return (int8_t)constrain(q, -128, 127);
  }
  static float dq(TfLiteTensor* t, int i) {
    return (t->data.int8[i] - t->params.zero_point) * t->params.scale;
  }

  bool isLocalMax(const int8_t* h, int cu, int cv) const {
    int8_t c = h[cv * MODEL_OUT_W + cu];
    for (int dv = -1; dv <= 1; dv++) {
      for (int du = -1; du <= 1; du++) {
        if (!du && !dv) continue;
        int u = cu + du, v = cv + dv;
        if (u < 0 || v < 0 || u >= MODEL_OUT_W || v >= MODEL_OUT_H) continue;
        if (h[v * MODEL_OUT_W + u] > c) return false;
      }
    }
    return true;
  }

  // 검출 중심 주변 패치의 평균 온도 = 인물 표면온도(쾌적도 MLP 입력)
  float patchMean(int u, int v, int r) const {
    float s = 0;
    int n = 0;
    for (int dv = -r; dv <= r; dv++) {
      for (int du = -r; du <= r; du++) {
        int x = u + du, y = v + dv;
        if (x < 0 || y < 0 || x >= TH_W || y >= TH_H) continue;
        s += raw_[y * TH_W + x];
        n++;
      }
    }
    return n ? s / n : ref_temp_;
  }

#if USE_THERMAL
  Adafruit_MLX90640 mlx_;
#endif
  float raw_[TH_PIXELS];
  float bg_[TH_PIXELS];
  float prev_[TH_PIXELS];
  float ref_temp_ = 25.0f;
  uint32_t frame_count_ = 0;
  uint32_t last_infer_us_ = 0;

  const tflite::Model* model_ = nullptr;
  tflite::MicroMutableOpResolver<8> resolver_;
  tflite::MicroInterpreter* interpreter_ = nullptr;
  TfLiteTensor* input_ = nullptr;
  TfLiteTensor* out_heat_ = nullptr;
  TfLiteTensor* out_off_ = nullptr;
  TfLiteTensor* out_depth_ = nullptr;
  alignas(16) uint8_t arena_[TENSOR_ARENA_BYTES];
};
