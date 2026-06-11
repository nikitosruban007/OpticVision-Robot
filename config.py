ROBOT_IP = "10.1.66.73"
WS_PORT = 80
STREAM_PORT = 81
DISCOVERY_PORT = 8888
DISCOVERY_TIMEOUT = 5.0

# --- ШВИДКІСТЬ: швидко, але без зриву лінії ---
SPEED_STRAIGHT = 218
SPEED_CURVE = 149
SPEED_TIGHT_TURN = 120
SPEED_PIVOT = 130
SPEED_MIN, SPEED_MAX = 85, 230
SPEED_SEND_DELTA = 8
SPEED_RAMP_TOP = 0.52
TIGHT_TURN_U = 0.70

CONTROL_HZ = 25
WATCHDOG_S = 0.5

STALE_FRAME_S = 0.22
BLIND_STOP_S = 0.55

THRESHOLD_METHOD = "otsu"
FIXED_THRESHOLD = 80
LINE_IS_DARK = True
BLUR_KSIZE = 5
MIN_LINE_AREA = 150
MORPH_KSIZE = 5
MIN_CONTRAST = 35
MIN_CONTRAST_CAP = 45
HEADING_MAX_DX_FRAC = 0.55

MAX_SEGMENT_WIDTH_FRAC = 0.65
MAX_BAND_FILL = 0.55
FAR_ONLY_GAIN = 1.0

ROI_NEAR = (0.78, 0.95)
ROI_FAR = (0.42, 0.62)
ROI_WEIGHT_NEAR = 0.65
ROI_WEIGHT_FAR = 0.35
OPPOSITE_FAR_LOCK_ERROR = 0.12
OPPOSITE_FAR_SCALE = 0.15

CENTER_OFFSET = 0.0

# --- PID: Збалансований підсил + сильніша фільтрація ---
KP = 0.40                 # Стабільніше на швидкості: менше перекидає left/right
KI = 0.0
KD = 0.08                 # Менше D-піків перед поворотом
DERIV_EMA = 0.15          # Плавніша похідна, менше тряски
PID_OUTPUT_CLAMP = 1.0
KI_ACTIVE_BAND = 0.25
DEADBAND = 0.10           # Менше мікрокорекцій на прямій
CORNER_DEADBAND = 0.07
STEER_EMA = 0.30          # Більше згладжування на вході в поворот

# --- PWM дискретних команд + гістерезис напрямку ---
STEER_PWM_PERIOD = 4      # База для частки поворотів; імпульси розподіляються рівномірно
STEER_PWM_GAIN = 1.45
CORNER_STEER_BOOST = 1.12
CORNER_PWM_GAIN = 1.05
STEER_RATE_LIMIT = 0.20
CORNER_STEER_RATE_LIMIT = 0.16
TURN_MIN_HOLD = 5         # Тримаємо напрям, щоб не трусився left/right
TURN_FLIP_FORCE = 0.82    # Міняємо бік тільки коли похибка справді велика
TURN_FLIP_CONFIRM_TICKS = 3
CORNER_LOCK_ERROR = 0.20
CORNER_LOCK_RELEASE_ERROR = 0.06
CORNER_LOCK_RELEASE_TICKS = 4
CORNER_REVERSE_FORCE = 0.58
CORNER_REVERSE_CONFIRM_TICKS = 5
CORNER_LOCK_MIN_DUTY = 2
FAR_ONLY_MIN_DUTY = 2

USE_HEADING = True
K_LATERAL = 1.0
K_HEADING = 0.45
CORNER_SLOW_ERROR = 0.32
CORNER_SLOW_HEADING = 0.22
OPPOSITE_HEADING_LOCK_ERROR = 0.18
OPPOSITE_HEADING_SCALE = 0.25
OPPOSITE_TURN_LOCK_ERROR = 0.22
RECOVER_SIGN_ERROR = 0.10
FAR_ONLY_FLIP_FORCE = 0.55
FAR_ONLY_FLIP_CONFIRM_TICKS = 3
FAR_ONLY_KEEP_ERROR = 0.24

STARTLINE_WIDTH_FRAC = 0.6
STARTLINE_IGNORE_S = 2.0

LOST_GRACE_S = 0.35
LOST_RECOVER_S = 9.0
LOST_GRACE_TURN = True

SHOW_DEBUG = True
START_KEY = 32            # ПРОБІЛ — старт/стоп

# --- SUMO assist: необов'язкова детекція опонента + легкий push ---
# За замовчуванням вимкнено, щоб не заважати line-tracker режиму.
SUMO_ASSIST_ENABLED = False
SUMO_PUSH_ENABLED = False
SUMO_ROI = (0.20, 0.82)
SUMO_CONFIRM_TICKS = 2
SUMO_CENTER_DEADBAND = 0.18
SUMO_TURN_SPEED = 95
SUMO_PUSH_SPEED = 105

# Якщо знаєш колір опонента, додай HSV-діапазони:
# SUMO_HSV_RANGES = [((0, 80, 50), (12, 255, 255)), ((170, 80, 50), (179, 255, 255))]
# Якщо порожньо, працює універсальний контурний детектор.
SUMO_HSV_RANGES = []
SUMO_MIN_AREA_FRAC = 0.025
SUMO_MAX_AREA_FRAC = 0.45
SUMO_MIN_HEIGHT_FRAC = 0.10
SUMO_MIN_WIDTH_FRAC = 0.08
SUMO_DARK_THRESHOLD = 70
SUMO_BRIGHT_THRESHOLD = 190
SUMO_MORPH_KSIZE = 7

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
        if MIN_CONTRAST > MIN_CONTRAST_CAP:
            print(f"[config] MIN_CONTRAST={MIN_CONTRAST} зависокий -> {MIN_CONTRAST_CAP}")
            MIN_CONTRAST = MIN_CONTRAST_CAP
        print(f"[config] застосовано calibration.json: {sorted(_c)}")
    except Exception as _e:
        print(f"[config] не вдалось прочитати calibration.json: {_e}")
