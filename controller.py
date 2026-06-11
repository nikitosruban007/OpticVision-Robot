import time
import config


class PID:
    def __init__(self, kp, ki, kd, clamp):
        self.kp, self.ki, self.kd, self.clamp = kp, ki, kd, clamp
        self._i = 0.0
        self._prev = 0.0
        self._d_f = 0.0
        self._t = None

    def reset(self):
        self._i = 0.0
        self._prev = 0.0
        self._d_f = 0.0
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

        d_raw = 0.0 if dt == 0 else (error - self._prev) / dt
        df = getattr(config, "DERIV_EMA", 0.3)
        self._d_f = df * d_raw + (1 - df) * self._d_f
        self._prev = error

        u = self.kp * error + self.ki * self._i + self.kd * self._d_f
        return max(-self.clamp, min(self.clamp, u))


class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.pid = PID(config.KP, config.KI, config.KD, config.PID_OUTPUT_CLAMP)
        self.last_error_sign = 1
        self.lost_since = None
        self._steer_f = 0.0
        self._pwm_phase = 0
        self._turn_dir = "forward"
        self._last_turn = None
        self._ticks_since_flip = 99
        self._u_cmd = 0.0
        self._pending_turn = None
        self._pending_turn_ticks = 0
        self._far_only_flip_ticks = 0
        self._corner_lock_turn = None
        self._corner_release_ticks = 0
        self._corner_reverse_turn = None
        self._corner_reverse_ticks = 0
        self._corner_lock_switched = False

    def reset(self):
        self.pid.reset()
        self._steer_f = 0.0
        self._pwm_phase = 0
        self._last_turn = None
        self._ticks_since_flip = 99
        self._u_cmd = 0.0
        self._pending_turn = None
        self._pending_turn_ticks = 0
        self._far_only_flip_ticks = 0
        self._corner_lock_turn = None
        self._corner_release_ticks = 0
        self._corner_reverse_turn = None
        self._corner_reverse_ticks = 0
        self._corner_lock_switched = False
        self.lost_since = None

    def _speed_for(self, u, slow=False, corner=False):
        a = abs(u)
        if corner:
            if a >= getattr(config, "TIGHT_TURN_U", 0.68):
                return getattr(config, "SPEED_TIGHT_TURN", config.SPEED_CURVE)
            return config.SPEED_CURVE
        if slow:
            return config.SPEED_CURVE
        if a < config.DEADBAND:
            return config.SPEED_STRAIGHT
        top = getattr(config, "SPEED_RAMP_TOP", 0.6)
        if a >= top:
            return config.SPEED_CURVE
        t = (a - config.DEADBAND) / max(top - config.DEADBAND, 1e-3)
        return int(config.SPEED_STRAIGHT
                   + t * (config.SPEED_CURVE - config.SPEED_STRAIGHT))

    def follow(self, error, heading_error=0.0, slow=False, far_only=False):
        self.lost_since = None
        corner = (slow
                  or abs(error) >= getattr(config, "CORNER_SLOW_ERROR", 0.35)
                  or abs(heading_error) >= getattr(config, "CORNER_SLOW_HEADING", 0.25))
        far_only_opposite = (far_only
                             and error * self.last_error_sign < 0
                             and abs(error) < getattr(config, "FAR_ONLY_FLIP_FORCE", 0.55))
        if far_only_opposite:
            self._far_only_flip_ticks += 1
            if self._far_only_flip_ticks >= int(getattr(config, "FAR_ONLY_FLIP_CONFIRM_TICKS", 3)):
                far_only_opposite = False
        else:
            self._far_only_flip_ticks = 0
        if (not far_only_opposite
                and abs(error) >= getattr(config, "RECOVER_SIGN_ERROR", 0.10)):
            self.last_error_sign = 1 if error > 0 else -1
        if far_only_opposite:
            error = self.last_error_sign * getattr(config, "FAR_ONLY_KEEP_ERROR", 0.24)
            heading_error = 0.0

        if getattr(config, "USE_HEADING", False):
            lateral = config.K_LATERAL * error
            heading = config.K_HEADING * heading_error
            if (corner
                    and error * heading_error < 0
                    and abs(error) >= getattr(config, "OPPOSITE_HEADING_LOCK_ERROR", 0.18)):
                heading *= getattr(config, "OPPOSITE_HEADING_SCALE", 0.25)
            steer = lateral + heading
            if (corner
                    and abs(error) >= getattr(config, "OPPOSITE_HEADING_LOCK_ERROR", 0.18)
                    and steer * error < 0):
                steer = lateral
            steer = max(-1.0, min(1.0, steer))
        else:
            steer = error

        a = getattr(config, "STEER_EMA", 1.0)
        self._steer_f = a * steer + (1 - a) * self._steer_f
        steer = self._steer_f

        u = self.pid.step(steer)
        if corner and abs(u) > 1e-3:
            boost = getattr(config, "CORNER_STEER_BOOST", 1.0)
            u = max(-1.0, min(1.0, u * boost))

        limit_name = "CORNER_STEER_RATE_LIMIT" if corner else "STEER_RATE_LIMIT"
        limit = getattr(config, limit_name, 1.0)
        du = max(-limit, min(limit, u - self._u_cmd))
        self._u_cmd = max(-1.0, min(1.0, self._u_cmd + du))
        u = self._u_cmd

        self.robot.set_speed(self._speed_for(u, slow=slow, corner=corner))

        # Мертва зона: їдемо рівно, скидаємо фазу ШІМ
        deadband = getattr(config, "CORNER_DEADBAND", config.DEADBAND) if corner else config.DEADBAND
        if abs(u) < deadband:
            self._pwm_phase = 0
            self._pending_turn = None
            self._pending_turn_ticks = 0
            self.robot.move("forward")
            return "forward"

        period = max(1, int(getattr(config, "STEER_PWM_PERIOD", 4)))
        gain = getattr(config, "STEER_PWM_GAIN", 1.0)
        if corner:
            gain *= getattr(config, "CORNER_PWM_GAIN", 1.0)
        duty = int(round(min(abs(u) * gain, 1.0) * period))
        if far_only:
            duty = max(duty, min(period, int(getattr(config, "FAR_ONLY_MIN_DUTY", 2))))
        turn = "right" if u > 0 else "left"
        error_turn = None
        if corner and abs(error) >= getattr(config, "OPPOSITE_TURN_LOCK_ERROR", 0.22):
            error_turn = "right" if error > 0 else "left"

        self._update_corner_lock(corner, error, turn)
        if self._corner_lock_switched:
            self._last_turn = None
            self._ticks_since_flip = 99
            self._pending_turn = None
            self._pending_turn_ticks = 0
            self._corner_lock_switched = False
        if self._corner_lock_turn is not None:
            if error_turn is not None and error_turn != self._corner_lock_turn:
                self._pwm_phase = 0
                self._pending_turn = error_turn
                self._pending_turn_ticks = 0
                self.robot.move("forward")
                return "forward"
            if turn != self._corner_lock_turn:
                turn = self._corner_lock_turn
                u = abs(u) if turn == "right" else -abs(u)
                duty = int(round(min(abs(u) * gain, 1.0) * period))
            duty = max(duty, min(period, int(getattr(config, "CORNER_LOCK_MIN_DUTY", 2))))
        elif error_turn is not None and turn != error_turn:
            self._pwm_phase = 0
            self._pending_turn = error_turn
            self._pending_turn_ticks = 0
            self.robot.move("forward")
            return "forward"

        self._ticks_since_flip += 1
        min_hold = int(getattr(config, "TURN_MIN_HOLD", 3))
        flip_force = getattr(config, "TURN_FLIP_FORCE", 0.5)

        # Зміна left<->right має підтвердитись кілька тіків, інакше це шум кадру.
        if (self._last_turn is not None and turn != self._last_turn
                and abs(u) < flip_force):
            if self._pending_turn == turn:
                self._pending_turn_ticks += 1
            else:
                self._pending_turn = turn
                self._pending_turn_ticks = 1

            confirm_ticks = int(getattr(config, "TURN_FLIP_CONFIRM_TICKS", 2))
            if (self._ticks_since_flip < min_hold
                    or self._pending_turn_ticks < confirm_ticks):
                self._pwm_phase = 0
                self.robot.move("forward")
                return "forward"
        else:
            self._pending_turn = None
            self._pending_turn_ticks = 0

        if (self._last_turn is not None and turn != self._last_turn
                and self._ticks_since_flip < min_hold
                and abs(u) < flip_force):
            self._pwm_phase = 0
            self.robot.move("forward")
            return "forward"

        cmd = turn if self._pwm_phase < duty else "forward"
        self._pwm_phase = (self._pwm_phase + 1) % period

        if cmd in ("left", "right") and cmd != self._last_turn:
            self._last_turn = cmd
            self._ticks_since_flip = 0
            self._pending_turn = None
            self._pending_turn_ticks = 0

        self.robot.move(cmd)
        return cmd

    def _update_corner_lock(self, corner, error, turn):
        lock_error = getattr(config, "CORNER_LOCK_ERROR", 0.20)
        release_error = getattr(config, "CORNER_LOCK_RELEASE_ERROR", 0.08)
        release_ticks = int(getattr(config, "CORNER_LOCK_RELEASE_TICKS", 3))
        reverse_force = getattr(config, "CORNER_REVERSE_FORCE", 0.58)
        reverse_ticks = int(getattr(config, "CORNER_REVERSE_CONFIRM_TICKS", 5))

        if not corner:
            if abs(error) <= release_error:
                self._corner_release_ticks += 1
            else:
                self._corner_release_ticks = 0
            if self._corner_release_ticks >= release_ticks:
                self._corner_lock_turn = None
                self._corner_reverse_turn = None
                self._corner_reverse_ticks = 0
            return

        error_turn = "right" if error > 0 else "left"
        if self._corner_lock_turn is None:
            if abs(error) >= lock_error:
                self._corner_lock_turn = error_turn
                self._corner_release_ticks = 0
                self._corner_lock_switched = True
            return

        if abs(error) <= release_error:
            self._corner_release_ticks += 1
            if self._corner_release_ticks >= release_ticks:
                self._corner_lock_turn = None
                self._corner_reverse_turn = None
                self._corner_reverse_ticks = 0
            return
        self._corner_release_ticks = 0

        if error_turn != self._corner_lock_turn and abs(error) >= reverse_force:
            if self._corner_reverse_turn == error_turn:
                self._corner_reverse_ticks += 1
            else:
                self._corner_reverse_turn = error_turn
                self._corner_reverse_ticks = 1
            if self._corner_reverse_ticks >= reverse_ticks:
                self._corner_lock_turn = error_turn
                self._corner_reverse_turn = None
                self._corner_reverse_ticks = 0
                self._corner_lock_switched = True
        else:
            self._corner_reverse_turn = None
            self._corner_reverse_ticks = 0

    def recover(self):
        if self.lost_since is None:
            self.lost_since = time.time()
            self.pid.reset()
            self._steer_f = 0.0
            self._pwm_phase = 0
            self._u_cmd = 0.0
            self._pending_turn = None
            self._pending_turn_ticks = 0
            self._far_only_flip_ticks = 0
            self._corner_lock_turn = None
            self._corner_release_ticks = 0
            self._corner_reverse_turn = None
            self._corner_reverse_ticks = 0
            self._corner_lock_switched = False
        elapsed = time.time() - self.lost_since
        if elapsed < config.LOST_GRACE_S:
            if getattr(config, "LOST_GRACE_TURN", False):
                self.robot.set_speed(config.SPEED_CURVE)
                self.robot.move("right" if self.last_error_sign > 0 else "left")
            else:
                self.robot.move("forward")
            return True
        if elapsed > config.LOST_RECOVER_S:
            self.robot.stop()
            return False
        self.robot.set_speed(config.SPEED_PIVOT)
        self.robot.move("right" if self.last_error_sign > 0 else "left")
        return True
