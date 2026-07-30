"""
Team 4 - lightweight AHRS for the Grand Prix

This is the same idea as our state_estimation attitude_node, but much smaller.
That node is a full ROS 2 package, and we cannot run ROS on top of racecar_core
during the race, so this reimplements the essentials: read the IMU, and report
heading and how quickly we are rotating.
"""

import math
import time

CALIB_FRAMES = 120     # about 2 seconds at rest. Do not move the car during this
ACCEL_TRUST = 0.02     # how much we trust the accelerometer. Keep it small or roll/pitch gets jittery
GRAVITY = 9.81


def wrap(a):
    # Keeps angles within -pi to pi so the value never accumulates past a full turn
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class AHRS:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0         # radians, 0 = the direction we were facing at startup
        self.yaw_rate = 0.0    # rad/s, sign indicates direction of rotation
        self.ready = False     # True once the gyro bias has been measured

        self._bias = 0.0
        self._gyro_sum = [0.0, 0.0, 0.0]
        self._accel_sum = [0.0, 0.0, 0.0]
        self._n = 0
        self._last = None

        # Axis layout, resolved during calibration. See _resolve_axes below.
        self._fwd = 0
        self._side = 1
        self._up = 2

    def _resolve_axes(self):
        """
        Work out which index is the vertical axis, and therefore which gyro
        index is yaw.

        The library returns these in different orders depending on where the
        code runs. On the physical car the z axis points up, so yaw is index 2.
        In the simulator the y axis points up, so yaw is index 1. Hardcoding
        either one means the filter reads the wrong axis on the other platform,
        which is why this is detected instead of assumed.

        The car is stationary and level during calibration, so the vertical
        axis is simply whichever accelerometer axis is reading gravity.
        """
        means = [s / self._n for s in self._accel_sum]
        self._up = max(range(3), key=lambda i: abs(means[i]))
        # The remaining two axes are horizontal. Forward is the lowest spare
        # index, which is x on both platforms.
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
        # A very large dt means something stalled, so discard that frame
        if dt <= 0 or dt > 0.5:
            return

        # The important correction: even at a complete standstill the gyro reads
        # a small nonzero rate, and integrating that produces significant drift.
        # We use the wait at the red light to average it, then subtract it off.
        if not self.ready:
            for i in range(3):
                self._gyro_sum[i] += gyro[i]
                self._accel_sum[i] += accel[i]
            self._n += 1
            if self._n >= CALIB_FRAMES:
                # Which axis is vertical has to be settled before we know which
                # gyro axis holds yaw, so resolve the layout first.
                self._resolve_axes()
                self._bias = self._gyro_sum[self._up] / self._n
                self.ready = True
            return

        # Pull the axes out by resolved index rather than by name
        a_fwd, a_side, a_up = accel[self._fwd], accel[self._side], accel[self._up]
        g_fwd, g_side, g_up = gyro[self._fwd], gyro[self._side], gyro[self._up]

        self.yaw_rate = g_up - self._bias
        self.yaw = wrap(self.yaw + self.yaw_rate * dt)

        # Roll and pitch: the gyro is smooth but drifts, while gravity does not
        # drift but is noisy. Blending the two (a complementary filter) gives a
        # usable estimate for far less work than a Kalman filter.
        self.roll += g_fwd * dt
        self.pitch += g_side * dt
        # Note: this is the MAGNITUDE of the acceleration vector. There is no
        # magnetometer in this filter (the Trial 2D node has one; it is separate).
        accel_norm = math.sqrt(a_fwd * a_fwd + a_side * a_side + a_up * a_up)
        # Only trust gravity when the car is not braking, cornering hard, or colliding
        if abs(accel_norm - GRAVITY) < 2.0:
            self.roll = (1 - ACCEL_TRUST) * self.roll + ACCEL_TRUST * math.atan2(a_side, a_up)
            self.pitch = (1 - ACCEL_TRUST) * self.pitch + ACCEL_TRUST * math.atan2(
                -a_fwd, math.sqrt(a_side * a_side + a_up * a_up)
            )

    def heading(self):
        # Degrees are easier to read on the display and in the console
        return math.degrees(self.yaw)

    def turn_rate(self):
        return math.degrees(self.yaw_rate)
