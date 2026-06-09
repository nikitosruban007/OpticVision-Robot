import threading
import time

import cv2

import config


class VideoStream:
    def __init__(self, ip):
        self.url = f"http://{ip}:{config.STREAM_PORT}/stream"
        self.cap = None
        self.frame = None
        self.frame_id = 0
        self.lock = threading.Lock()
        self.running = False
        self._last_t = time.time()
        self.fps = 0.0

    def start(self):
        self.cap = cv2.VideoCapture(self.url)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if not self.cap.isOpened():
            raise RuntimeError(f"Не вдалось відкрити відеопотік: {self.url}")
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        for _ in range(100):
            if self.frame is not None:
                break
            time.sleep(0.05)
        return self

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            now = time.time()
            dt = now - self._last_t
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
            self._last_t = now
            with self.lock:
                self.frame = frame
                self.frame_id += 1

    def read(self):
        with self.lock:
            if self.frame is None:
                return self.frame_id, None
            return self.frame_id, self.frame.copy()

    def stop(self):
        self.running = False
        time.sleep(0.05)
        if self.cap:
            self.cap.release()
