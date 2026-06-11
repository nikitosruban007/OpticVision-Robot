import threading
import time
import urllib.request

import cv2
import numpy as np

import config

MJPEG_READ_SIZE = 4096
MJPEG_MAX_BUFFER = 2 * 1024 * 1024
CAMERA_RETRY_S = 1.0


class VideoStream:
    """MJPEG-грабер: декодує тільки найновіший JPEG і викидає старі кадри."""

    def __init__(self, ip):
        port = getattr(config, "STREAM_PORT", 81)
        self.urls = [
            f"http://{ip}:{port}/stream",
            f"http://{ip}:{port}/video_feed",
        ]
        self.url = self.urls[0]
        self.response = None
        self.frame = None
        self.frame_id = 0
        self.last_frame_t = 0.0
        self.lock = threading.Lock()
        self.running = False
        self._last_t = time.time()
        self._last_warn_t = 0.0
        self.fps = 0.0

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        for _ in range(100):
            if self.frame is not None:
                break
            time.sleep(0.05)
        return self

    def _loop(self):
        while self.running:
            opened = False
            for url in self.urls:
                if not self.running:
                    break
                self.url = url
                try:
                    request = urllib.request.Request(
                        url,
                        headers={"Cache-Control": "no-cache", "Pragma": "no-cache"},
                    )
                    self.response = urllib.request.urlopen(request, timeout=3)
                    print(f"[camera] відеопотік відкрито: {url}")
                    opened = True
                    self._read_stream()
                except Exception as exc:
                    if self.running:
                        print(f"[camera] потік недоступний {url}: {exc}")
                finally:
                    self._close_response()

            if self.running and not opened:
                now = time.time()
                if now - self._last_warn_t > 3.0:
                    print("[camera] камеру не знайдено, повторюю підключення...")
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

            now = time.time()
            dt = now - self._last_t
            if dt > 0:
                instant_fps = 1.0 / dt
                self.fps = instant_fps if self.fps == 0.0 else 0.9 * self.fps + 0.1 * instant_fps
            self._last_t = now
            with self.lock:
                self.frame = frame
                self.frame_id += 1
                self.last_frame_t = now

    def read(self):
        with self.lock:
            if self.frame is None:
                return self.frame_id, None
            return self.frame_id, self.frame.copy()

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
