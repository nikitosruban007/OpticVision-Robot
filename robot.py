import json
import socket
import threading
import time

from websocket import create_connection

import config


def discover_robot(timeout=config.DISCOVERY_TIMEOUT, port=config.DISCOVERY_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(timeout)
    sock.bind(("", port))
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                break
            text = data.decode("utf-8", "ignore")
            if not text.startswith("KPI_ROBOT_CAR"):
                continue
            fields = dict(
                part.split("=", 1)
                for part in text.split(";")
                if "=" in part
            )
            ip = fields.get("ip")
            if ip:
                print(f"[robot] знайдено {fields.get('name', '?')} @ {ip}")
                return ip
    finally:
        sock.close()
    return None


class RobotClient:
    def __init__(self, ip=None):
        self.ip = ip or config.ROBOT_IP or discover_robot()
        if not self.ip:
            raise RuntimeError("Робота не знайдено. Вкажи ROBOT_IP у config.py.")
        self.ws = None
        self._send_lock = threading.Lock()
        self._last_status = {}
        self._draining = False
        self._current_speed = None
        self._last_cmd = None

    # --- з'єднання ----------------------------------------------------------
    def connect(self):
        url = f"ws://{self.ip}:{config.WS_PORT}/ws"
        self.ws = create_connection(url, timeout=3)
        self._start_drain()
        self.set_speed(config.SPEED_STRAIGHT)
        return self

    def _start_drain(self):
        self._draining = True

        def loop():
            while self._draining:
                try:
                    msg = self.ws.recv()
                except Exception:
                    break
                if isinstance(msg, str) and msg.startswith("{"):
                    try:
                        self._last_status = json.loads(msg)
                    except ValueError:
                        pass

        threading.Thread(target=loop, daemon=True).start()

    # --- надсилання ---------------------------------------------------------
    def _send(self, text):
        with self._send_lock:
            self.ws.send(text)

    def move(self, command):
        self._send(command)
        self._last_cmd = command

    def set_speed(self, value):
        value = max(config.SPEED_MIN, min(config.SPEED_MAX, int(value)))
        if (self._current_speed is None
                or abs(value - self._current_speed) >= config.SPEED_SEND_DELTA):
            self._send(f"speed:{value}")
            self._current_speed = value

    def led(self, on):
        self._send("led:on" if on else "led:off")

    def stop(self):
        self.move("stop")

    @property
    def status(self):
        return dict(self._last_status)

    def close(self):
        self._draining = False
        try:
            if self.ws:
                self.stop()
                time.sleep(0.05)
                self.ws.close()
        except Exception:
            pass

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
