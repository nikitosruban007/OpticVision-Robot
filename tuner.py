import sys

import cv2
import numpy as np

import config
from camera import VideoStream
from robot import RobotClient, discover_robot

WIN = "tuner"


def nothing(_):
    pass


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else (config.ROBOT_IP or discover_robot())
    if not ip:
        print("Робота не знайдено."); return
    cam = VideoStream(ip).start()

    cv2.namedWindow(WIN)
    cv2.createTrackbar("thresh", WIN, config.FIXED_THRESHOLD, 255, nothing)
    cv2.createTrackbar("otsu(0/1)", WIN, 1, 1, nothing)
    cv2.createTrackbar("dark_line(0/1)", WIN, int(config.LINE_IS_DARK), 1, nothing)
    cv2.createTrackbar("blur", WIN, config.BLUR_KSIZE, 21, nothing)
    cv2.createTrackbar("roi_top%", WIN, int(config.ROI_NEAR[0] * 100), 100, nothing)
    cv2.createTrackbar("roi_bot%", WIN, int(config.ROI_NEAR[1] * 100), 100, nothing)

    print("Esc — вийти і надрукувати значення.")
    while True:
        _, frame = cam.read()
        if frame is None:
            continue
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        k = cv2.getTrackbarPos("blur", WIN)
        if k >= 3:
            gray = cv2.GaussianBlur(gray, (k | 1, k | 1), 0)

        if cv2.getTrackbarPos("otsu(0/1)", WIN):
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        else:
            t = cv2.getTrackbarPos("thresh", WIN)
            _, mask = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)
        if cv2.getTrackbarPos("dark_line(0/1)", WIN):
            mask = cv2.bitwise_not(mask)

        top = cv2.getTrackbarPos("roi_top%", WIN) / 100.0
        bot = cv2.getTrackbarPos("roi_bot%", WIN) / 100.0
        if bot <= top:
            bot = min(1.0, top + 0.05)
        band = mask[int(top * h):int(bot * h), :]

        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(vis, (0, int(top * h)), (w - 1, int(bot * h)), (0, 255, 0), 1)
        m = cv2.moments(band)
        if m["m00"] > 0:
            cx = int(m["m10"] / m["m00"])
            cv2.circle(vis, (cx, int((top + bot) / 2 * h)), 6, (0, 0, 255), -1)

        cv2.imshow(WIN, vis)
        cv2.imshow("camera", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    print("\n--- перенеси у config.py ---")
    print(f"THRESHOLD_METHOD = {'otsu' if cv2.getTrackbarPos('otsu(0/1)', WIN) else 'fixed'!r}")
    print(f"FIXED_THRESHOLD = {cv2.getTrackbarPos('thresh', WIN)}")
    print(f"LINE_IS_DARK = {bool(cv2.getTrackbarPos('dark_line(0/1)', WIN))}")
    print(f"BLUR_KSIZE = {cv2.getTrackbarPos('blur', WIN)}")
    print(f"ROI_NEAR = ({cv2.getTrackbarPos('roi_top%', WIN)/100:.2f}, "
          f"{cv2.getTrackbarPos('roi_bot%', WIN)/100:.2f})")
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
