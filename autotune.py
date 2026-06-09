import json
import os
import sys
import time

import cv2
import numpy as np

import config
from camera import VideoStream
from robot import discover_robot

CALIB_PATH = os.path.join(os.path.dirname(__file__), "calibration.json")
SAMPLES = 60


def binarize(gray, line_is_dark, method, fixed_thr, blur):
    if blur and blur >= 3:
        k = blur | 1
        gray = cv2.GaussianBlur(gray, (k, k), 0)
    if method == "adaptive":
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 31, 10)
    elif method == "fixed":
        _, mask = cv2.threshold(gray, fixed_thr, 255, cv2.THRESH_BINARY)
    else:
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if line_is_dark:
        mask = cv2.bitwise_not(mask)
    return mask


def detect_polarity(gray):
    _, m = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    dark_frac = float((m == 0).mean())
    return dark_frac < 0.5


def near_band(mask):
    h = mask.shape[0]
    y0, y1 = int(config.ROI_NEAR[0] * h), int(config.ROI_NEAR[1] * h)
    return mask[y0:y1, :]


def band_centroid(band, min_area):
    cnts, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    big = [c for c in cnts if cv2.contourArea(c) > min_area]
    if not big:
        return None, 0
    c = max(big, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, len(big)
    return M["m10"] / M["m00"], len(big)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else (config.ROBOT_IP or discover_robot())
    if not ip:
        print("Робота не знайдено. Вкажи IP: python autotune.py 10.1.66.73")
        return
    cam = VideoStream(ip).start()
    print("[autotune] постав робота РІВНО на лінію. ENTER — зберегти, Esc — вийти.")

    offsets, fills, multi, polar, contrasts = [], [], 0, [], []
    while True:
        _, frame = cam.read()
        if frame is None:
            continue
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        line_dark = detect_polarity(gray)
        polar.append(line_dark)

        mask = binarize(gray, line_dark, "otsu", config.FIXED_THRESHOLD, config.BLUR_KSIZE)
        band = near_band(mask)
        y0, y1 = int(config.ROI_NEAR[0] * h), int(config.ROI_NEAR[1] * h)
        gband = gray[y0:y1, :]
        fill = float((band > 0).mean())
        cx, nseg = band_centroid(band, config.MIN_LINE_AREA)

        lp = gband[band > 0]; bp = gband[band == 0]
        if lp.size > 20 and bp.size > 20:
            c = float(bp.mean() - lp.mean())
            contrasts.append(c if line_dark else -c)

        fills.append(fill)
        if nseg >= 2:
            multi += 1
        if cx is not None and fill < 0.5:
            offsets.append(cx - w / 2.0)

        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        y0, y1 = int(config.ROI_NEAR[0] * h), int(config.ROI_NEAR[1] * h)
        cv2.rectangle(vis, (0, y0), (w - 1, y1), (0, 255, 0), 1)
        cv2.line(vis, (w // 2, 0), (w // 2, h), (120, 120, 120), 1)
        if cx is not None:
            cv2.circle(vis, (int(cx), (y0 + y1) // 2), 6, (0, 0, 255), -1)
        med_off = float(np.median(offsets)) if offsets else 0.0
        txt = (f"line={'DARK' if line_dark else 'LIGHT'} fill={fill:.2f} "
               f"seg={nseg} offset~{med_off:+.1f}px  samples={len(offsets)}")
        cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(vis, "ENTER=save  Esc=quit", (8, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("autotune", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("[autotune] вихід без збереження.")
            break
        if key in (13, 10):
            if len(offsets) < 10:
                print("[autotune] замало даних — потримай робота на лінії ще трохи.")
                continue
            save(offsets, fills, polar, multi, len(fills), contrasts)
            break

    cam.stop()
    cv2.destroyAllWindows()


def save(offsets, fills, polar, multi, n, contrasts=None):
    line_dark = sum(polar) > len(polar) / 2
    mean_fill = float(np.mean(fills)) if fills else 0.0
    method = "adaptive" if mean_fill > 0.45 else "otsu"
    center_offset = round(float(np.median(offsets)), 1)

    calib = {
        "LINE_IS_DARK": bool(line_dark),
        "THRESHOLD_METHOD": method,
        "BLUR_KSIZE": int(config.BLUR_KSIZE),
        "CENTER_OFFSET": center_offset,
    }
    if contrasts:
        med_c = float(np.median(contrasts))
        calib["MIN_CONTRAST"] = int(max(18, min(60, med_c * 0.5)))
    with open(CALIB_PATH, "w", encoding="utf-8") as f:
        json.dump(calib, f, ensure_ascii=False, indent=2)

    print("\n=== КАЛІБРУВАННЯ ЗБЕРЕЖЕНО -> calibration.json (підхопиться автоматично) ===")
    for k, v in calib.items():
        print(f"  {k} = {v}")
    if multi > n * 0.3:
        print("  [увага] часто видно >1 сегмента в ближній смузі — можливо ROI_NEAR зачіпає "
              "перетин або фон. Перевір позицію або звузь ROI_NEAR у config.py.")
    if abs(center_offset) > 40:
        print(f"  [увага] великий зсув ({center_offset}px). Перевір, що робот стояв РІВНО на лінії.")
    print("  Готово. Запускай: python main.py")


if __name__ == "__main__":
    main()
