"""열화상 인체 검출 TinyCenterNet 학습 -> int8 TFLite -> model_person.h

설계서: docs/tinyml-design.md §3

사용법:
    python train_person_thermal.py --data dataset/ --epochs 200
    # 결과: model_person.tflite, model_person.h (firmware 폴더에 복사)

데이터 형식 (tools/capture_dataset.py 가 생성):
    frames : (N, 24, 32) float32, 섭씨 원본
    labels : (N, K, 3)  [u, v, range_m], 사람 없으면 (N, 0, 3)
"""

import argparse
import glob
import os

import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

H, W = 24, 32
OH, OW = 6, 8          # 히트맵 격자 (stride 4)
STRIDE_U, STRIDE_V = W / OW, H / OH


# --------------------------------------------------------------- 전처리
def normalize(frames):
    """펌웨어 person_model.h 의 전처리와 반드시 동일해야 한다.

    프레임 중앙값 기준 정규화가 핵심: 실내 온도가 22℃든 30℃든 입력 분포가
    동일해지므로, 냉방 중 온도 변화로 인한 도메인 시프트가 사라진다.
    """
    f = frames.astype(np.float32)
    med = np.median(f, axis=(1, 2), keepdims=True)
    sd = np.std(f, axis=(1, 2), keepdims=True)
    scale = np.maximum(3.0 * sd, 1.5)
    return np.clip((f - med) / scale, -1.0, 3.0), scale


def build_inputs(frames, fps=8):
    """[정규화 프레임, 1초 시간차분] 2채널 구성."""
    norm, scale = normalize(frames)
    prev = np.roll(frames, fps, axis=0)
    prev[:fps] = frames[:fps]
    diff = np.clip((frames - prev) / scale[..., 0][:, None, None], -2.0, 2.0)
    return np.stack([norm, diff], axis=-1).astype(np.float32)


def make_targets(labels):
    """CenterNet 타깃: 가우시안 히트맵 + 서브픽셀 오프셋 + log 거리."""
    n = len(labels)
    hm = np.zeros((n, OH, OW, 1), np.float32)
    off = np.zeros((n, OH, OW, 2), np.float32)
    dep = np.zeros((n, OH, OW, 1), np.float32)
    mask = np.zeros((n, OH, OW, 1), np.float32)

    sigma = 1.2
    for i, objs in enumerate(labels):
        for u, v, rng in objs:
            cu, cv = u / STRIDE_U, v / STRIDE_V
            iu, iv = int(cu), int(cv)
            if not (0 <= iu < OW and 0 <= iv < OH):
                continue
            yy, xx = np.mgrid[0:OH, 0:OW]
            g = np.exp(-((xx - cu) ** 2 + (yy - cv) ** 2) / (2 * sigma**2))
            hm[i, :, :, 0] = np.maximum(hm[i, :, :, 0], g)
            off[i, iv, iu] = [cu - iu - 0.5, cv - iv - 0.5]
            dep[i, iv, iu, 0] = np.log(max(rng, 0.3))
            mask[i, iv, iu, 0] = 1.0
    return hm, off, dep, mask


# --------------------------------------------------------------- 증강
def augment(x, hm, off, dep, mask):
    """좌우 반전이 가장 효과가 크다. 라벨의 u도 함께 뒤집는다."""
    if np.random.rand() < 0.5:
        x = x[:, ::-1]
        hm = hm[:, ::-1]
        off = off[:, ::-1]
        off[..., 0] *= -1
        dep = dep[:, ::-1]
        mask = mask[:, ::-1]
    # 게인/오프셋: 실내 온도 변동 모사
    x = x * np.random.uniform(0.9, 1.1) + np.random.uniform(-0.15, 0.15)
    # MLX90640 NETD 0.1K 상당 노이즈
    x += np.random.normal(0, 0.05, x.shape).astype(np.float32)
    # 랜덤 데드픽셀
    for _ in range(np.random.randint(0, 3)):
        x[np.random.randint(H), np.random.randint(W), 0] = 0
    return x.astype(np.float32), hm, off, dep, mask


# --------------------------------------------------------------- 모델
def dwsep(x, filters, stride, name):
    x = layers.DepthwiseConv2D(3, strides=stride, padding="same",
                               use_bias=False, name=f"{name}_dw")(x)
    x = layers.BatchNormalization(name=f"{name}_dwbn")(x)
    x = layers.ReLU(6.0, name=f"{name}_dwrelu")(x)
    x = layers.Conv2D(filters, 1, use_bias=False, name=f"{name}_pw")(x)
    x = layers.BatchNormalization(name=f"{name}_pwbn")(x)
    return layers.ReLU(6.0, name=f"{name}_pwrelu")(x)


def build_model():
    inp = keras.Input((H, W, 2), name="thermal")
    x = layers.Conv2D(16, 3, strides=2, padding="same", use_bias=False)(inp)
    x = layers.BatchNormalization()(x)
    x = layers.ReLU(6.0)(x)          # 12x16x16
    x = dwsep(x, 32, 1, "b1")        # 12x16x32
    x = dwsep(x, 64, 2, "b2")        #  6x 8x64
    x = dwsep(x, 64, 1, "b3")        #  6x 8x64

    hm = layers.Conv2D(1, 1, activation="sigmoid", name="heatmap")(x)
    off = layers.Conv2D(2, 1, name="offset")(x)
    dep = layers.Conv2D(1, 1, name="logdepth")(x)
    # 출력 순서가 펌웨어 person_model.h 의 output(0/1/2)와 일치해야 한다.
    return keras.Model(inp, [hm, off, dep])


# --------------------------------------------------------------- 손실
def focal_loss(y_true, y_pred):
    """CenterNet 변형 focal loss. 배경이 압도적으로 많은 문제를 다룬다."""
    y_pred = tf.clip_by_value(y_pred, 1e-4, 1 - 1e-4)
    pos = tf.cast(tf.equal(y_true, 1.0), tf.float32)
    neg = 1.0 - pos
    pos_loss = -pos * tf.pow(1 - y_pred, 2.0) * tf.math.log(y_pred)
    neg_loss = (-neg * tf.pow(1 - y_true, 4.0) * tf.pow(y_pred, 2.0)
                * tf.math.log(1 - y_pred))
    n = tf.maximum(tf.reduce_sum(pos), 1.0)
    return (tf.reduce_sum(pos_loss) + tf.reduce_sum(neg_loss)) / n


def masked_l1(mask):
    def loss(y_true, y_pred):
        m = tf.broadcast_to(mask, tf.shape(y_true))
        return (tf.reduce_sum(tf.abs(y_true - y_pred) * m)
                / tf.maximum(tf.reduce_sum(m), 1.0))
    return loss


# --------------------------------------------------------------- 양자화 내보내기
def export_int8(model, rep_x, out="model_person"):
    conv = tf.lite.TFLiteConverter.from_keras_model(model)
    conv.optimizations = [tf.lite.Optimize.DEFAULT]

    def rep():
        for i in range(min(300, len(rep_x))):
            yield [rep_x[i : i + 1].astype(np.float32)]

    conv.representative_dataset = rep
    conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    conv.inference_input_type = tf.int8
    conv.inference_output_type = tf.int8
    tfl = conv.convert()

    with open(f"{out}.tflite", "wb") as f:
        f.write(tfl)

    # xxd -i 없이 직접 C 헤더 생성
    with open(f"{out}.h", "w") as f:
        f.write("// 자동 생성 파일 - 직접 수정하지 말 것\n")
        f.write("// tools/train_person_thermal.py 가 생성\n#pragma once\n\n")
        f.write(f"// size: {len(tfl)} bytes\n")
        f.write("alignas(16) const unsigned char g_model_person[] = {\n")
        for i in range(0, len(tfl), 12):
            f.write("  " + ", ".join(f"0x{b:02x}" for b in tfl[i : i + 12]) + ",\n")
        f.write("};\n")
        f.write(f"const unsigned int g_model_person_len = {len(tfl)};\n")

    print(f"[export] {out}.tflite = {len(tfl)/1024:.1f} KB")
    print(f"[export] {out}.h -> firmware/smart_air_guider_tinyml/ 에 복사")


# --------------------------------------------------------------- 평가
def evaluate(model, x, hm_t, labels):
    from math import atan2, degrees

    HFOV, VFOV = 55.0, 35.0
    pred = model.predict(x, verbose=0)
    hm = pred[0][..., 0]
    errs = []
    fp = 0
    fn = 0
    for i in range(len(x)):
        peak = np.unravel_index(np.argmax(hm[i]), hm[i].shape)
        score = hm[i][peak]
        has_gt = len(labels[i]) > 0
        if score < 0.55:
            fn += int(has_gt)
            continue
        if not has_gt:
            fp += 1
            continue
        u = (peak[1] + 0.5) * STRIDE_U
        v = (peak[0] + 0.5) * STRIDE_V
        gu, gv = labels[i][0][0], labels[i][0][1]
        d_az = abs((u - gu) / (W - 1) * HFOV)
        d_el = abs((v - gv) / (H - 1) * VFOV)
        errs.append((d_az**2 + d_el**2) ** 0.5)

    n = max(len(x), 1)
    print(f"[eval] 각도 오차 MAE = {np.mean(errs):.2f}deg "
          f"(p95 {np.percentile(errs, 95):.2f}deg)")
    print(f"[eval] 오검출 {fp}/{n} ({100*fp/n:.2f}%), 미검출 {fn}/{n} ({100*fn/n:.2f}%)")


# --------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="dataset")
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--batch", type=int, default=64)
    args = ap.parse_args()

    frames, labels = [], []
    for p in sorted(glob.glob(os.path.join(args.data, "*.npz"))):
        d = np.load(p, allow_pickle=True)
        frames.append(d["frames"])
        labels.extend(list(d["labels"]))
    frames = np.concatenate(frames, axis=0)
    print(f"[data] {len(frames)} 프레임, "
          f"네거티브 {sum(1 for l in labels if len(l)==0)/len(labels)*100:.0f}%")

    x = build_inputs(frames)
    hm, off, dep, mask = make_targets(labels)

    n = len(x)
    idx = np.random.permutation(n)
    split = int(n * 0.85)
    tr, va = idx[:split], idx[split:]

    model = build_model()
    model.summary()
    model.compile(
        optimizer=keras.optimizers.Adam(1e-3),
        loss={"heatmap": focal_loss,
              "offset": masked_l1(mask[tr]),
              "logdepth": masked_l1(mask[tr])},
        loss_weights={"heatmap": 1.0, "offset": 1.0, "logdepth": 0.5},
    )

    model.fit(
        x[tr], {"heatmap": hm[tr], "offset": off[tr], "logdepth": dep[tr]},
        validation_data=(x[va],
                         {"heatmap": hm[va], "offset": off[va], "logdepth": dep[va]}),
        epochs=args.epochs, batch_size=args.batch,
        callbacks=[
            keras.callbacks.ReduceLROnPlateau(patience=10, factor=0.5),
            keras.callbacks.EarlyStopping(patience=25, restore_best_weights=True),
        ],
    )

    evaluate(model, x[va], hm[va], [labels[i] for i in va])
    export_int8(model, x[tr])


if __name__ == "__main__":
    main()
