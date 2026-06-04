import time
import cv2
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

    try:
        while True:
            frame = bot.get_frame()
            if frame is None:
                continue

            err, found, debug_frame = vision.process(frame)

            if found:
                last_err = err
                if abs(err) <= DEADZONE:
                    if current_speed != MAX_SPEED:
                        current_speed = MAX_SPEED
                        bot.set_speed(current_speed)
                    bot.move("forward")
                elif err < -DEADZONE:
                    if current_speed != TURN_SPEED:
                        current_speed = TURN_SPEED
                        bot.set_speed(current_speed)
                    bot.move("left")
                else:
                    if current_speed != TURN_SPEED:
                        current_speed = TURN_SPEED
                        bot.set_speed(current_speed)
                    bot.move("right")
            else:
                if current_speed != SEARCH_SPEED:
                    current_speed = SEARCH_SPEED
                    bot.set_speed(current_speed)
                if last_err < 0:
                    bot.move("left")
                else:
                    bot.move("right")

    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()