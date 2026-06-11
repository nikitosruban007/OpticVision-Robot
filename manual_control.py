import json
import os
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

import config
from robot import RobotClient, discover_robot


WINDOW_NAME = "WASD Robot Controller + Camera"
KEY_TIMEOUT_S = 0.28
CAMERA_RETRY_S = 1.0
DISPLAY_HZ = 60
MJPEG_READ_SIZE = 4096
MJPEG_MAX_BUFFER = 2 * 1024 * 1024


def load_robot_ip():
    """Return robot IP from argv/config/calibration without blocking discovery."""
    if len(sys.argv) > 1 and sys.argv[1].strip():
        ip = sys.argv[1].strip()
        print(f"[manual] IP з аргументу: {ip}")
        return ip

    ip = getattr(config, "ROBOT_IP", None)
    calib_path = os.path.join(os.path.dirname(__file__), "calibration.json")
    if os.path.exists(calib_path):
        try:
            with open(calib_path, "r", encoding="utf-8") as f:
                calib = json.load(f)
            ip = calib.get("ROBOT_IP") or calib.get("robot_ip") or ip
            if calib.get("ROBOT_IP") or calib.get("robot_ip"):
                print(f"[manual] IP з calibration.json: {ip}")
        except Exception as exc:
            print(f"[manual] calibration.json не прочитано: {exc}")

    return ip


def connect_robot(ip):
    candidates = []
    if ip:
        candidates.append(ip)

    last_error = None
    for candidate in candidates:
        try:
            print(f"[manual] Підключаємось до робота: {candidate}")
            return RobotClient(ip=candidate).connect()
        except Exception as exc:
            last_error = exc
            print(f"[manual] Не вдалось підключитись до {candidate}: {exc}")

    print("[manual] Шукаємо робота через UDP discovery...")
    discovered = discover_robot()
    if discovered and discovered not in candidates:
        try:
            print(f"[manual] Підключаємось до знайденого робота: {discovered}")
            return RobotClient(ip=discovered).connect()
        except Exception as exc:
            last_error = exc

    if last_error:
        raise last_error
    return RobotClient(ip=None).connect()


class VideoSource:
    """Low-latency MJPEG reader with reconnect and stale-frame dropping."""

    def __init__(self, ip):
        port = getattr(config, "STREAM_PORT", 81)
        self.urls = [
            f"http://{ip}:{port}/stream",
            f"http://{ip}:{port}/video_feed",
        ]
        self.response = None
        self.url = None
        self.frame = None
        self.fps = 0.0
        self.running = False
        self.lock = threading.Lock()
        self._last_frame_t = time.time()
        self._last_warn_t = 0.0

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def _loop(self):
        while self.running:
            opened = False
            for url in self.urls:
                if not self.running:
                    break
                self.url = url
                try:
                    print(f"[video] Відкриваємо MJPEG: {url}")
                    request = urllib.request.Request(
                        url,
                        headers={
                            "Cache-Control": "no-cache",
                            "Pragma": "no-cache",
                        },
                    )
                    self.response = urllib.request.urlopen(request, timeout=3)
                    print(f"[video] Відеопотік відкрито: {url}")
                    opened = True
                    self._read_stream()
                except Exception as exc:
                    if self.running:
                        print(f"[video] Потік недоступний {url}: {exc}")
                finally:
                    self._close_response()

            if self.running and not opened:
                now = time.time()
                if now - self._last_warn_t > 3.0:
                    print("[video] Камеру не знайдено, повторюю підключення...")
                    self._last_warn_t = now
                time.sleep(CAMERA_RETRY_S)

    def _read_stream(self):
        buffer = bytearray()
        while self.running and self.response is not None:
            chunk = self.response.read(MJPEG_READ_SIZE)
            if not chunk:
                raise RuntimeError("порожній chunk з камери")

            buffer.extend(chunk)
            latest_jpeg = None
            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    if len(buffer) > MJPEG_MAX_BUFFER:
                        del buffer[:-2]
                    break

                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start > 0:
                        del buffer[:start]
                    if len(buffer) > MJPEG_MAX_BUFFER:
                        del buffer[:-MJPEG_MAX_BUFFER]
                    break

                latest_jpeg = bytes(buffer[start:end + 2])
                del buffer[:end + 2]

            if latest_jpeg is None:
                continue

            frame = cv2.imdecode(np.frombuffer(latest_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            self._store_frame(frame)

    def _store_frame(self, frame):
        now = time.time()
        dt = now - self._last_frame_t
        if dt > 0:
            instant_fps = 1.0 / dt
            self.fps = instant_fps if self.fps == 0.0 else 0.9 * self.fps + 0.1 * instant_fps
        self._last_frame_t = now
        with self.lock:
            self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def stop(self):
        self.running = False
        self._close_response()
        time.sleep(0.05)

    def _close_response(self):
        response = self.response
        self.response = None
        if response is not None:
            try:
                response.close()
            except Exception:
                pass


class ManualController:
    def __init__(self, robot):
        self.robot = robot
        self.active_cmd = "stop"
        self.last_input_t = 0.0
        self.last_sent_cmd = None
        self.speed_straight = getattr(config, "SPEED_STRAIGHT", 255)
        self.speed_pivot = getattr(config, "SPEED_PIVOT", 190)

    def handle_key(self, key):
        if key in (255, -1):
            return

        key_char = chr(key).lower() if 0 <= key < 256 else ""
        mapping = {
            "w": "forward",
            "s": "backward",
            "a": "left",
            "d": "right",
        }

        if key_char in mapping:
            self.active_cmd = mapping[key_char]
            self.last_input_t = time.time()
        elif key == 32:
            self.active_cmd = "stop"
            self.last_input_t = 0.0
            self.robot.stop()
            self.last_sent_cmd = "stop"

    def update(self):
        if time.time() - self.last_input_t > KEY_TIMEOUT_S:
            command = "stop"
        else:
            command = self.active_cmd

        if command in ("left", "right"):
            self.robot.set_speed(self.speed_pivot)
        elif command in ("forward", "backward"):
            self.robot.set_speed(self.speed_straight)

        if command == "stop":
            self.robot.stop()
        else:
            self.robot.move(command)

        self.last_sent_cmd = command
        return command


def make_placeholder(ip, video):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, "Waiting for camera stream...", (130, 220), font, 0.8, (255, 255, 255), 2)
    cv2.putText(frame, f"Robot: {ip}", (16, 455), font, 0.55, (180, 180, 180), 1)
    if video and video.url:
        cv2.putText(frame, video.url, (16, 475), font, 0.45, (120, 120, 120), 1)
    return frame


def main():
    robot_ip = load_robot_ip()
    robot = None
    video = None

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("[manual] WASD - рух, SPACE - стоп, Q/Esc - вихід.")

    try:
        robot = connect_robot(robot_ip)
        robot_ip = robot.ip
        print(f"[manual] Підключено до робота @ {robot_ip}")

        video = VideoSource(robot_ip).start()
        controller = ManualController(robot)
        control_period = 1.0 / max(1, getattr(config, "CONTROL_HZ", 25))
        display_period = 1.0 / DISPLAY_HZ
        last_control_t = 0.0

        while True:
            tick = time.time()

            frame = video.read()
            if frame is None:
                frame = make_placeholder(robot_ip, video)
            cv2.imshow(WINDOW_NAME, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

            controller.handle_key(key)
            now = time.time()
            if now - last_control_t >= control_period:
                controller.update()
                last_control_t = now

            time.sleep(max(0.0, display_period - (time.time() - tick)))

    except Exception as exc:
        print(f"[manual] Помилка: {exc}")
    finally:
        if robot is not None:
            robot.close()
        if video is not None:
            video.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
