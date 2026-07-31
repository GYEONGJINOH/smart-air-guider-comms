// 플레이스홀더 — 학습된 모델로 반드시 교체할 것.
//
//   python tools/train_person_thermal.py --data dataset/
//   -> model_person.h 가 생성되면 이 파일을 덮어쓴다.
//
// 학습 전에도 나머지 펌웨어(레이더 추적, 서보 조준, 오프라인 계층)를 검증할 수 있도록
// USE_THERMAL 0 으로 빌드하면 이 파일 없이도 동작한다(저하 사다리 L2 = 레이더 단독).
#pragma once

#if !defined(MODEL_PERSON_TRAINED)
#warning "model_person.h 가 플레이스홀더 상태입니다. tools/train_person_thermal.py 로 생성한 파일로 교체하세요."
#endif

alignas(16) const unsigned char g_model_person[] = {0x00};
const unsigned int g_model_person_len = 1;
