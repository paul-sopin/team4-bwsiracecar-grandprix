"""
Team 4 - small AHRS for the Grand Prix

Same idea as our state_estimation attitude_node, just way smaller. That one is a
full ROS 2 package and we can't run ROS on top of racecar_core during the race,
so this does the bare minimum: read the IMU, report heading and turn rate.
"""

import math
import time

CALIB_FRAMES = 120     # ~2 seconds at rest. don't move the car during this
ACCEL_TRUST = 0.02     # accelerometer weight. keep it small or roll/pitch gets jittery
GRAVITY = 9.81


def wrap(a):
    # -pi to pi, so the angle never accumulates past a full turn
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class AHRS:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0         # radians, 0 = whatever way we were facing at startup
        self.yaw_rate = 0.0    # rad/s, sign is the direction
        self.ready = False     # True once gyro bias is measured

        self._bias = 0.0
        self._gyro_sum = [0.0, 0.0, 0.0]
        self._accel_sum = [0.0, 0.0, 0.0]
        self._n = 0
        self._last = None

        # axis layout, figured out during calibration. see _resolve_axes
        self._fwd = 0
        self._side = 1
        self._up = 2

    def _resolve_axes(self):
        """
        Find which index is vertical, which tells us which gyro index is yaw.

        The library hands these back in different orders depending on where the
        code is running. On the real car z is up, so yaw is index 2. In the sim
        y is up, so yaw is index 1. Hardcode either one and the filter reads a
        completely wrong axis on the other platform.

        The car is level and sitting still during calibration, so the vertical
        axis is just whichever accel axis is reading gravity.
        """
        means = [s / self._n for s in self._accel_sum]
        self._up = max(range(3), key=lambda i: abs(means[i]))
        # other two are horizontal. forward is the lower spare index, x on both.
        spare = [i for i in range(3) if i != self._up]
        self._fwd, self._side = spare[0], spare[1]

    def update(self, rc):
        accel = rc.physics.get_linear_acceleration()
        gyro = rc.physics.get_angular_velocity()

        now = time.time()
        if self._last is None:
            self._last = now
            return
        dt = now - self._last
        self._last = now
        # huge dt means something stalled. throw the frame out.
        if dt <= 0 or dt > 0.5:
            return

        # even sitting completely still the gyro reads a small nonzero rate, and
        # integrating that drifts badly. average it during the wait at the red
        # light, then subtract it off everything after.
        if not self.ready:
            for i in range(3):
                self._gyro_sum[i] += gyro[i]
                self._accel_sum[i] += accel[i]
            self._n += 1
            if self._n >= CALIB_FRAMES:
                # axes first, we don't know which gyro index is yaw until then
                self._resolve_axes()
                self._bias = self._gyro_sum[self._up] / self._n
                self.ready = True
            return

        # by resolved index, not by name
        a_fwd, a_side, a_up = accel[self._fwd], accel[self._side], accel[self._up]
        g_fwd, g_side, g_up = gyro[self._fwd], gyro[self._side], gyro[self._up]

        self.yaw_rate = g_up - self._bias
        self.yaw = wrap(self.yaw + self.yaw_rate * dt)

        # roll/pitch. gyro is smooth and drifts, gravity is noisy and doesn't,
        # so blend them. complementary filter, way less work than a Kalman.
        self.roll += g_fwd * dt
        self.pitch += g_side * dt
        # magnitude of the accel vector. no magnetometer here, the Trial 2D node
        # is the one with that.
        accel_norm = math.sqrt(a_fwd * a_fwd + a_side * a_side + a_up * a_up)
        # only trust gravity when we're not braking, cornering hard, or hitting something
        if abs(accel_norm - GRAVITY) < 2.0:
            self.roll = (1 - ACCEL_TRUST) * self.roll + ACCEL_TRUST * math.atan2(a_side, a_up)
            self.pitch = (1 - ACCEL_TRUST) * self.pitch + ACCEL_TRUST * math.atan2(
                -a_fwd, math.sqrt(a_side * a_side + a_up * a_up)
            )

    def heading(self):
        # degrees, easier to read on the display and in the console
        return math.degrees(self.yaw)

    def turn_rate(self):
        return math.degrees(self.yaw_rate)
