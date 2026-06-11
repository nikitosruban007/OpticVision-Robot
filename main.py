import sys
import time

import cv2

import config
import sumo
import vision
from camera import VideoStream
from controller import Controller
from robot import RobotClient


def draw_debug(frame, res, command, fps, elapsed, running, opponent=None):
    h, w = frame.shape[:2]
    for roi, color in ((config.ROI_NEAR, (0, 255, 0)), (config.ROI_FAR, (0, 180, 255))):
        y0, y1 = int(roi[0] * h), int(roi[1] * h)  
        cv2.rectangle(frame, (0, y0), (w - 1, y1), color, 1)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (120, 120, 120), 1)
    ny = int((config.ROI_NEAR[0] + config.ROI_NEAR[1]) / 2 * h)
    if res is not None and res.near_x is not None:
        cv2.circle(frame, (int(res.near_x), ny), 6, (0, 0, 255), -1)
    if res is not None and res.near_x is not None and res.far_x is not None:
        fy = int((config.ROI_FAR[0] + config.ROI_FAR[1]) / 2 * h)
        cv2.arrowedLine(frame, (int(res.near_x), ny), (int(res.far_x), fy),
                        (0, 255, 255), 2, tipLength=0.25)
    state = "RUN" if running else "PAUSE"
    err = res.error if res is not None else 0.0
    head = res.heading_error if res is not None else 0.0
    txt = (f"[{state}] cmd={command} lat={err:+.2f} head={head:+.2f} "
           f"fps={fps:4.1f} t={elapsed:5.1f}s")
    if res is not None:
        if res.far_only:
            txt += " [far-only]"
        if res.is_junction:
            txt += " [junction]"
        if res.is_startfinish:
            txt += " [S/F]"
        if not res.found:
            txt += " [LOST]"
    if opponent is not None and opponent.found:
        txt += f" [sumo {opponent.offset:+.2f}]"
    color = (0, 255, 0) if running else (0, 200, 255)
    cv2.putText(frame, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    cv2.putText(frame, "SPACE = старт/стоп    q/Esc = вихiд", (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    if getattr(config, "SUMO_ASSIST_ENABLED", False):
        sumo.draw_debug(frame, opponent)
    cv2.imshow("line-tracker", frame)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else None
    robot = RobotClient(ip).connect()
    cam = VideoStream(robot.ip).start()
    ctrl = Controller(robot)
    sumo_assist = sumo.SumoAssist(robot) if getattr(config, "SUMO_ASSIST_ENABLED", False) else None

    running = False            # стартуємо на ПАУЗІ — поки не натиснеш ПРОБІЛ
    start_t = None
    last_frame_id = -1
    last_new_frame_t = time.time()
    predicted_x = None
    res = None
    opponent = None
    blind_warned = False
    period = 1.0 / config.CONTROL_HZ

    stale_s = getattr(config, "STALE_FRAME_S", 0.35)
    blind_stop_s = getattr(config, "BLIND_STOP_S", 1.2)
    start_key = getattr(config, "START_KEY", 32)

    # завжди є вікно для зчитування клавіш (старт/стоп по кнопці)
    cv2.namedWindow("line-tracker", cv2.WINDOW_NORMAL)

    print("[main] готовий. ПРОБІЛ — старт/стоп руху, q/Esc — вихiд.")
    try:
        while True:
            tick = time.time()
            fid, frame = cam.read()

            fresh = frame is not None and fid != last_frame_id
            if fresh:
                last_frame_id = fid
                last_new_frame_t = tick
                res = vision.analyze(frame, predicted_x)
                if sumo_assist is not None:
                    opponent = sumo_assist.detect(frame)
                if res.found and res.near_x is not None:
                    predicted_x = res.near_x

            stale_age = tick - last_new_frame_t
            command = "stop"
            elapsed = (tick - start_t) if (running and start_t) else 0.0

            # ---------- ПАУЗА: стоїмо, лише тримаємо watchdog ----------
            if not running:
                robot.stop()
                command = "stop"
            # ---------- немає свіжих кадрів: не кермуємо за старим ----------
            elif res is None or stale_age > stale_s:
                if stale_age > blind_stop_s:
                    robot.stop()
                    command = "stop"
                    if not blind_warned:
                        print("[main] вiдеопотiк завмер — стоп до появи кадрiв")
                        blind_warned = True
                elif res is None:
                    robot.stop()
                    command = "stop"
                else:
                    robot.set_speed(config.SPEED_CURVE)
                    robot.move("forward")
                    command = "stale-forward"
            # ---------- нормальне керування ----------
            else:
                blind_warned = False
                sumo_command = None
                if sumo_assist is not None and opponent is not None:
                    sumo_command = sumo_assist.follow(opponent)

                if sumo_command is not None:
                    command = sumo_command
                elif res.found:
                    slow = res.is_junction or res.is_startfinish or res.far_only
                    command = ctrl.follow(res.error, res.heading_error, slow=slow, far_only=res.far_only)
                else:
                    predicted_x = None
                    if ctrl.recover():
                        command = "recover"
                    else:
                        # лінію не вдалось повернути — пауза, чекаємо ПРОБІЛ
                        print("[main] лiнiю не повернуто — пауза. ПРОБІЛ щоб продовжити.")
                        running = False
                        robot.stop()
                        command = "stop"

            # ---------- відмальовка + клавіші ----------
            if frame is not None:
                draw_debug(frame, res, command, cam.fps, elapsed, running, opponent)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("[main] вихiд")
                break
            if key == start_key:                 # ПРОБІЛ — старт/стоп
                running = not running
                if running:
                    ctrl.reset()
                    start_t = time.time()
                    print("[main] СТАРТ")
                else:
                    robot.stop()
                    print("[main] СТОП (пауза)")

            time.sleep(max(0.0, period - (time.time() - tick)))
    finally:
        robot.close()
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
