ROBOT_IP = "10.1.66.73"
WS_PORT = 80
STREAM_PORT = 81
DISCOVERY_PORT = 8888
DISCOVERY_TIMEOUT = 5.0

SPEED_STRAIGHT = 180
SPEED_CURVE = 150
SPEED_PIVOT = 120
SPEED_MIN, SPEED_MAX = 85, 180
SPEED_SEND_DELTA = 8

CONTROL_HZ = 25
WATCHDOG_S = 0.5

THRESHOLD_METHOD = "otsu"
FIXED_THRESHOLD = 80
LINE_IS_DARK = True
BLUR_KSIZE = 5
MIN_LINE_AREA = 150
MORPH_KSIZE = 5
MIN_CONTRAST = 35
HEADING_MAX_DX_FRAC = 0.45

ROI_NEAR = (0.78, 0.95)
ROI_FAR = (0.50, 0.65)
ROI_WEIGHT_NEAR = 0.8
ROI_WEIGHT_FAR = 0.2

CENTER_OFFSET = 0.0

KP = 0.9
KI = 0.05
KD = 0.20
PID_OUTPUT_CLAMP = 1.0
KI_ACTIVE_BAND = 0.25
DEADBAND = 0.04
STEER_EMA = 0.5

USE_HEADING = True
K_LATERAL = 1.0
K_HEADING = 0.4

STARTLINE_WIDTH_FRAC = 0.6
STARTLINE_IGNORE_S = 2.0

LOST_GRACE_S = 0.4
LOST_RECOVER_S = 9.0

SHOW_DEBUG = True

import json as _json, os as _os
_calib_path = _os.path.join(_os.path.dirname(__file__), "calibration.json")
if _os.path.exists(_calib_path):
    try:
        with open(_calib_path, "r", encoding="utf-8") as _f:
            _c = _json.load(_f)
        for _k, _v in _c.items():
            if _k in ("ROI_NEAR", "ROI_FAR") and isinstance(_v, list):
                _v = tuple(_v)
            globals()[_k] = _v
        print(f"[config] застосовано calibration.json: {sorted(_c)}")
    except Exception as _e:
        print(f"[config] не вдалось прочитати calibration.json: {_e}")
