import websocket
import threading
import time
import requests
import cv2


class RobotInterface:
    def __init__(self, ip):
        self.ip = ip
        self.ws_url = f"ws://{ip}/ws"
        self.action_url = f"http://{ip}/action"
        self.stream_url = f"http://{ip}:81/stream"
        self.ws = None
        self.current_command = "stop"
        self.speed = 170
        self.running = False
        self.lock = threading.Lock()
        self.cap = None
        self.latest_frame = None

    def start(self):
        self.ws = websocket.create_connection(self.ws_url, timeout=2)
        self.ws.send("ping")
        self.running = True
        threading.Thread(target=self._heartbeat, daemon=True).start()

        self.cap = cv2.VideoCapture(self.stream_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        threading.Thread(target=self._video_loop, daemon=True).start()

    def _heartbeat(self):
        while self.running:
            with self.lock:
                cmd = self.current_command
            if cmd != "stop":
                try:
                    self.ws.send(cmd)
                except:
                    pass
            time.sleep(0.15)

    def _video_loop(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.01)

    def get_frame(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def set_speed(self, speed):
        self.speed = max(85, min(255, int(speed)))
        try:
            self.ws.send(f"speed:{self.speed}")
        except:
            pass

    def move(self, cmd):
        with self.lock:
            self.current_command = cmd
        if cmd == "stop":
            try:
                self.ws.send("stop")
            except:
                try:
                    requests.get(self.action_url, params={"go": "stop"}, timeout=0.5)
                except:
                    pass

    def stop(self):
        self.running = False
        self.move("stop")
        if self.ws:
            self.ws.close()
        if self.cap:
            self.cap.release()