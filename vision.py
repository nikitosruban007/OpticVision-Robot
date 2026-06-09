from dataclasses import dataclass, field

import cv2
import numpy as np

import config


@dataclass
class VisionResult:
    found: bool = False
    error: float = 0.0
    heading_error: float = 0.0
    near_x: float = None
    far_x: float = None
    is_junction: bool = False
    is_startfinish: bool = False
    width: int = 0
    mask: object = field(default=None, repr=False)


def _binarize(gray):
    if config.BLUR_KSIZE and config.BLUR_KSIZE >= 3:
        k = config.BLUR_KSIZE | 1
        gray = cv2.GaussianBlur(gray, (k, k), 0)

    if config.THRESHOLD_METHOD == "otsu":
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    elif config.THRESHOLD_METHOD == "adaptive":
        mask = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 10,
        )
    else:
        _, mask = cv2.threshold(gray, config.FIXED_THRESHOLD, 255, cv2.THRESH_BINARY)

    if config.LINE_IS_DARK:
        mask = cv2.bitwise_not(mask)

    k = getattr(config, "MORPH_KSIZE", 5)
    if k >= 3:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def _band_segments(band):
    contours, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    segments = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < config.MIN_LINE_AREA:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx = m["m10"] / m["m00"]
        segments.append((cx, area))
    segments.sort(key=lambda s: -s[1])
    return segments


def _pick_segment(segments, predicted_x):
    if not segments:
        return None
    if predicted_x is None or len(segments) == 1:
        return segments[0][0]
    return min(segments, key=lambda s: abs(s[0] - predicted_x))[0]


def _is_startfinish(band):
    h, w = band.shape
    col_fill = (band > 0).sum(axis=0) / max(h, 1)
    wide = (col_fill > 0.5).sum() / max(w, 1)
    return wide >= config.STARTLINE_WIDTH_FRAC


def _band_contrast(gray_band, mask_band):
    line_px = gray_band[mask_band > 0]
    bg_px = gray_band[mask_band == 0]
    if line_px.size < 20 or bg_px.size < 20:
        return 0.0
    diff = float(bg_px.mean() - line_px.mean())
    return diff if config.LINE_IS_DARK else -diff


def analyze(frame, predicted_x=None):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = _binarize(gray)

    def slab(roi):
        y0 = int(roi[0] * h)
        y1 = int(roi[1] * h)
        return mask[y0:y1, :], gray[y0:y1, :]

    near_mask, near_gray = slab(config.ROI_NEAR)
    far_mask, far_gray = slab(config.ROI_FAR)

    near_segs = _band_segments(near_mask)
    far_segs = _band_segments(far_mask)

    res = VisionResult(width=w, mask=mask)
    res.is_startfinish = _is_startfinish(near_mask)
    res.is_junction = len(near_segs) >= 2 or len(far_segs) >= 2

    min_contrast = getattr(config, "MIN_CONTRAST", 35)
    near_contrast = _band_contrast(near_gray, near_mask)
    if not near_segs or near_contrast < min_contrast:
        res.found = False
        return res

    near_x = _pick_segment(near_segs, predicted_x)
    res.near_x = near_x

    far_x = None
    if far_segs and _band_contrast(far_gray, far_mask) >= min_contrast:
        far_x = _pick_segment(far_segs, predicted_x if predicted_x is not None else near_x)
    res.far_x = far_x

    if near_x is None:
        res.found = False
        return res

    center = w / 2.0 + getattr(config, "CENTER_OFFSET", 0.0)
    err_near = (near_x - center) / (w / 2.0)
    err_far = (far_x - center) / (w / 2.0) if far_x is not None else err_near
    res.found = True
    res.error = (config.ROI_WEIGHT_NEAR * err_near
                 + config.ROI_WEIGHT_FAR * err_far)
    res.error = float(np.clip(res.error, -1.0, 1.0))

    max_dx_frac = getattr(config, "HEADING_MAX_DX_FRAC", 0.45)
    if far_x is not None and abs(far_x - near_x) <= max_dx_frac * w:
        near_y = (config.ROI_NEAR[0] + config.ROI_NEAR[1]) / 2.0 * h
        far_y = (config.ROI_FAR[0] + config.ROI_FAR[1]) / 2.0 * h
        dy = max(near_y - far_y, 1.0)
        dx = far_x - near_x
        angle = np.arctan2(dx, dy)
        res.heading_error = float(np.clip(angle / (np.pi / 2), -1.0, 1.0))
    else:
        res.heading_error = 0.0
    return res
