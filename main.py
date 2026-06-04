import time
import cv2
import numpy as np
from robot import RobotInterface
from vision import LineTracker

IP = "192.168.4.1"
MAX_SPEED = 255
TURN_SPEED = 135
SEARCH_SPEED = 100
DEADZONE = 40


def run():
    bot = RobotInterface(IP)
    vision = LineTracker()

    bot.start()
    time.sleep(1)

    current_speed = MAX_SPEED
    bot.set_speed(current_speed)
    last_err = 0

    is_running = False
    show_video = True

    blank_screen = np.zeros((150, 400, 3), dtype=np.uint8)
    cv2.putText(blank_screen, "VIDEO DISABLED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(blank_screen, "Press 'V' to enable video", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(blank_screen, "G: GO | S: STOP | Q: QUIT", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    try:
        while True:
            frame = bot.get_frame()
            if frame is None:
                continue

            err, found, debug_frame = vision.process(frame, debug=show_video)
            current_action = "STOP"

            if is_running:
                if found:
                    last_err = err
                    if abs(err) <= DEADZONE:
                        if current_speed != MAX_SPEED:
                            current_speed = MAX_SPEED
                            bot.set_speed(current_speed)
                        bot.move("forward")
                        current_action = "FORWARD"
                    elif err < -DEADZONE:
                        if current_speed != TURN_SPEED:
                            current_speed = TURN_SPEED
                            bot.set_speed(current_speed)
                        bot.move("left")
                        current_action = "LEFT"
                    else:
                        if current_speed != TURN_SPEED:
                            current_speed = TURN_SPEED
                            bot.set_speed(current_speed)
                        bot.move("right")
                        current_action = "RIGHT"
                else:
                    if current_speed != SEARCH_SPEED:
                        current_speed = SEARCH_SPEED
                        bot.set_speed(current_speed)
                    if last_err < 0:
                        bot.move("left")
                        current_action = "SEARCH LEFT"
                    else:
                        bot.move("right")
                        current_action = "SEARCH RIGHT"
            else:
                bot.move("stop")

            if show_video and debug_frame is not None:
                if not is_running:
                    cv2.putText(debug_frame, "PAUSED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.putText(debug_frame, "V: Video | G: Go | S: Stop", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 255), 1)
                else:
                    cv2.putText(debug_frame, f"ACT: {current_action}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)
                    cv2.putText(debug_frame, f"ERR: {err}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(debug_frame, f"SPD: {current_speed}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 1)

                cv2.imshow("KPI Robot", debug_frame)
            else:
                temp_blank = blank_screen.copy()
                status_color = (0, 255, 0) if is_running else (0, 0, 255)
                status_text = "RUNNING" if is_running else "PAUSED"
                cv2.putText(temp_blank, f"STATE: {status_text}", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color,
                            2)

                cv2.imshow("KPI Robot", temp_blank)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                is_running = True
            elif key == ord('s'):
                is_running = False
            elif key == ord('v'):
                show_video = not show_video

    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()