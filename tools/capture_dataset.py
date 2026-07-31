"""데이터 수집 + 레이더 자동 라벨링

설계서: docs/tinyml-design.md §3.4

TinyML 프로젝트의 최대 병목은 라벨링이다. 이 시스템은 레이더가 라벨러 역할을 하므로
수작업이 거의 없다: 사람이 방을 돌아다니면 레이더가 (x, y)를 뱉고, 그것을 열화상
픽셀 좌표로 투영해 히트맵 라벨을 자동 생성한다.

펌웨어를 CAPTURE_MODE로 빌드하면 노드가 아래 형식을 시리얼로 출력한다:
    F,<768개 온도 x100 정수>
    R,<타깃수>,<x1_cm>,<y1_cm>,<v1>,...
    ---

사용법:
    python capture_dataset.py --port COM5 --out dataset/session01.npz --minutes 20
"""

import argparse
import time

import numpy as np
import serial

H, W = 24, 32
HFOV, VFOV = 55.0, 35.0        # 렌즈 사양에 맞출 것
SENSOR_HEIGHT_CM = 220.0       # 센서 설치 높이
ASSUMED_PERSON_Y_CM = 130.0    # 가슴 높이 근사 (서 있는 성인)


def radar_to_pixel(x_cm, y_cm):
    """레이더 (x, y) -> 열화상 픽셀 (u, v) 투영.

    레이더는 2D(수평면)만 주므로 v는 사람 가슴 높이를 가정해 계산한다.
    이 근사가 학습에 문제가 되지 않는 이유: 조준에 실제로 필요한 정밀도는
    방위각(u)이고, 고도각(v)은 오차가 커도 pitch 보정 범위 안에 들어온다.
    앉기/눕기 구간만 나중에 수동 보정하면 된다.
    """
    az = np.degrees(np.arctan2(x_cm, y_cm))
    dy = ASSUMED_PERSON_Y_CM - SENSOR_HEIGHT_CM
    el = np.degrees(np.arctan2(dy, np.hypot(x_cm, y_cm)))

    u = (az / HFOV + 0.5) * (W - 1)
    v = (-el / VFOV + 0.5) * (H - 1)
    if not (0 <= u < W and 0 <= v < H):
        return None
    return float(u), float(v), float(np.hypot(x_cm, y_cm) / 100.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", required=True)
    ap.add_argument("--minutes", type=float, default=20)
    ap.add_argument("--negative", action="store_true",
                    help="무인 구간 수집. 레이더 타깃을 무시하고 라벨을 비운다.")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=2)
    frames, labels = [], []
    pending_frame = None
    deadline = time.time() + args.minutes * 60

    print("수집 시작. Ctrl+C 로 중단.")
    try:
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", "ignore").strip()
            if not line:
                continue

            if line.startswith("F,"):
                vals = np.fromstring(line[2:], sep=",", dtype=np.float32)
                if vals.size != H * W:
                    continue
                pending_frame = (vals / 100.0).reshape(H, W)

            elif line.startswith("R,") and pending_frame is not None:
                parts = line[2:].split(",")
                cnt = int(parts[0])
                objs = []
                if not args.negative:
                    for i in range(cnt):
                        x = float(parts[1 + i * 3])
                        y = float(parts[2 + i * 3])
                        p = radar_to_pixel(x, y)
                        if p:
                            objs.append(p)
                frames.append(pending_frame)
                labels.append(objs)
                pending_frame = None

                if len(frames) % 100 == 0:
                    pos = sum(1 for l in labels if l)
                    print(f"  {len(frames)} 프레임 (양성 {pos}, "
                          f"네거티브 {len(frames)-pos})")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    np.savez_compressed(
        args.out,
        frames=np.array(frames, dtype=np.float32),
        labels=np.array(labels, dtype=object),
    )
    pos = sum(1 for l in labels if l)
    print(f"\n저장: {args.out}")
    print(f"  총 {len(frames)} 프레임 / 양성 {pos} / 네거티브 {len(frames)-pos}")
    if len(frames) and (len(frames) - pos) / len(frames) < 0.3:
        print("  ⚠ 네거티브 비율이 30% 미만이다. --negative 로 무인 구간을 더 수집할 것.")


if __name__ == "__main__":
    main()
