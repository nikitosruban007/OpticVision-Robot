import time

import config


class PID:
    def __init__(self, kp, ki, kd, clamp):
        self.kp, self.ki, self.kd, self.clamp = kp, ki, kd, clamp
        self._i = 0.0
        self._prev = 0.0
        self._t = None

    def reset(self):
        self._i = 0.0
        self._prev = 0.0
        self._t = None

    def step(self, error):
        now = time.time()
        dt = 0.0 if self._t is None else max(now - self._t, 1e-3)
        self._t = now
        active_band = getattr(config, "KI_ACTIVE_BAND", 1.0)
        if self.ki > 0 and abs(error) < active_band:
            self._i += error * dt
            self._i = max(-self.clamp, min(self.clamp, self._i))
        elif abs(error) >= active_band:
            self._i *= 0.9
        d = 0.0 if dt == 0 else (error - self._prev) / dt
        self._prev = error
        u = self.kp * error + self.ki * self._i + self.kd * d
        return max(-self.clamp, min(self.clamp, u))


class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.pid = PID(config.KP, config.KI, config.KD, config.PID_OUTPUT_CLAMP)
        self._accum = 0.0
        self.last_error_sign = 1
        self.lost_since = None
        self._steer_f = 0.0

    def _speed_for(self, u):
        a = abs(u)
        if a < config.DEADBAND:
            return config.SPEED_STRAIGHT
        if a > 0.6:
            return config.SPEED_CURVE
        t = (a - config.DEADBAND) / (0.6 - config.DEADBAND)
        return int(config.SPEED_STRAIGHT
                   + t * (config.SPEED_CURVE - config.SPEED_STRAIGHT))

    def follow(self, error, heading_error=0.0):
        self.lost_since = None

        if getattr(config, "USE_HEADING", False):
            steer = (config.K_LATERAL * error
                     + config.K_HEADING * heading_error)
            steer = max(-1.0, min(1.0, steer))
        else:
            steer = error

        a = getattr(config, "STEER_EMA", 1.0)
        self._steer_f = a * steer + (1 - a) * self._steer_f
        steer = self._steer_f

        u = self.pid.step(steer)
        if abs(steer) > 1e-3:
            self.last_error_sign = 1 if steer > 0 else -1

        self.robot.set_speed(self._speed_for(u))

        if abs(u) < config.DEADBAND:
            self._accum = 0.0
            self.robot.move("forward")
            return "forward"

        self._accum += abs(u)
        turn = "right" if u > 0 else "left"
        if self._accum >= 1.0:
            self._accum -= 1.0
            self.robot.move(turn)
            return turn
        self.robot.move("forward")
        return "forward"

    def recover(self):
        if self.lost_since is None:
            self.lost_since = time.time()
            self.pid.reset()
            self._steer_f = 0.0
        elapsed = time.time() - self.lost_since
        if elapsed < config.LOST_GRACE_S:
            self.robot.move("forward")
            return True
        if elapsed > config.LOST_RECOVER_S:
            self.robot.stop()
            return False
        self.robot.set_speed(config.SPEED_PIVOT)
        self.robot.move("right" if self.last_error_sign > 0 else "left")
        return True
