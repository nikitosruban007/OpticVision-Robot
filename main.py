import sys
import time

import cv2

import config
import vision
from camera import VideoStream
from controller import Controller
from robot import RobotClient


def draw_debug(frame, res, command, fps, elapsed):
    h, w = frame.shape[:2]
    for roi, color in ((config.ROI_NEAR, (0, 255, 0)), (config.ROI_FAR, (0, 180, 255))):
        y0, y1 = int(roi[0] * h), int(roi[1] * h)
        cv2.rectangle(frame, (0, y0), (w - 1, y1), color, 1)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (120, 120, 120), 1)
    ny = int((config.ROI_NEAR[0] + config.ROI_NEAR[1]) / 2 * h)
    if res.near_x is not None:
        cv2.circle(frame, (int(res.near_x), ny), 6, (0, 0, 255), -1)
    if res.near_x is not None and res.far_x is not None:
        fy = int((config.ROI_FAR[0] + config.ROI_FAR[1]) / 2 * h)
        cv2.arrowedLine(frame, (int(res.near_x), ny), (int(res.far_x), fy),
                        (0, 255, 255), 2, tipLength=0.25)
    txt = (f"cmd={command} lat={res.error:+.2f} head={res.heading_error:+.2f} "
           f"fps={fps:4.1f} t={elapsed:5.1f}s")
    if res.is_junction:
        txt += " [junction]"
    if res.is_startfinish:
        txt += " [START/FINISH]"
    cv2.putText(frame, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imshow("line-tracker", frame)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else None
    robot = RobotClient(ip).connect()
    cam = VideoStream(robot.ip).start()
    ctrl = Controller(robot)

    started = False
    start_t = None
    last_frame_id = -1
    predicted_x = None
    period = 1.0 / config.CONTROL_HZ
    boot_t = time.time()

    print("[main] поїхали. q/Esc — стоп.")
    try:
        while True:
            tick = time.time()
            fid, frame = cam.read()
            if frame is None:
                robot.move("forward")
                time.sleep(period)
                continue

            if fid != last_frame_id:
                last_frame_id = fid
                res = vision.analyze(frame, predicted_x)
                predicted_x = res.near_x if res.near_x is not None else predicted_x
            command = "forward"

            elapsed = (time.time() - start_t) if start_t else 0.0

            since_boot = time.time() - boot_t
            if res.is_startfinish and since_boot > config.STARTLINE_IGNORE_S:
                if not started:
                    started = True
                    start_t = time.time()
                    print("[main] СТАРТ — таймер пішов")
                elif elapsed > 3.0:
                    robot.stop()
                    print(f"[main] ФІНІШ. Час: {elapsed:.2f} с")
                    break

            if res.found:
                command = ctrl.follow(res.error, res.heading_error)
            else:
                if not ctrl.recover():
                    print("[main] лінію не вдалось повернути вчасно — стоп")
                    robot.stop()
                    break

            if config.SHOW_DEBUG:
                draw_debug(frame, res, command, cam.fps, elapsed)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    print("[main] ручна зупинка")
                    break

            time.sleep(max(0.0, period - (time.time() - tick)))
    finally:
        robot.close()
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
