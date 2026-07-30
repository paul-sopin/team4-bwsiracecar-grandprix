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
        self._sum = 0.0
        self._n = 0
        self._last = None

    def update(self, rc):
        ax, ay, az = rc.physics.get_linear_acceleration()
        gx, gy, gz = rc.physics.get_angular_velocity()

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
            self._sum += gz
            self._n += 1
            if self._n >= CALIB_FRAMES:
                self._bias = self._sum / self._n
                self.ready = True
            return

        self.yaw_rate = gz - self._bias
        self.yaw = wrap(self.yaw + self.yaw_rate * dt)

        # Roll and pitch: the gyro is smooth but drifts, while gravity does not
        # drift but is noisy. Blending the two (a complementary filter) gives a
        # usable estimate for far less work than a Kalman filter.
        self.roll += gx * dt
        self.pitch += gy * dt
        # Note: this is the MAGNITUDE of the acceleration vector. There is no
        # magnetometer in this filter (the Trial 2D node has one; it is separate).
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        # Only trust gravity when the car is not braking, cornering hard, or colliding
        if abs(accel_norm - GRAVITY) < 2.0:
            self.roll = (1 - ACCEL_TRUST) * self.roll + ACCEL_TRUST * math.atan2(ay, az)
            self.pitch = (1 - ACCEL_TRUST) * self.pitch + ACCEL_TRUST * math.atan2(
                -ax, math.sqrt(ay * ay + az * az)
            )

    def heading(self):
        # Degrees are easier to read on the display and in the console
        return math.degrees(self.yaw)

    def turn_rate(self):
        return math.degrees(self.yaw_rate)
