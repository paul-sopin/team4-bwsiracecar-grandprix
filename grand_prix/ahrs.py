"""
Team 4 - baby AHRS for the grand prix

basically the same idea as our state_estimation attitude_node but WAY smaller
cuz that one is a whole ros2 package and we cant run ros on top of racecar_core
during the race lol. this just eats the imu and spits out heading + how fast
we're spinning.
"""

import math
import time

CALIB_FRAMES = 120     # ~2 sec of just sitting there. dont move the car during this
ACCEL_TRUST = 0.02     # how much we believe the accelerometer. keep it small or it gets jittery
GRAVITY = 9.81


def wrap(a):
    # keeps angles between -pi and pi so it doesnt count to like 900 degrees
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class AHRS:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0         # radians, 0 = whatever way we were pointed at the start
        self.yaw_rate = 0.0    # rad/s, + is one way - is the other
        self.ready = False     # goes True once the gyro bias is figured out

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
        # if dt is huge something froze, just skip that frame
        if dt <= 0 or dt > 0.5:
            return

        # THE ANNOYING PART: even sitting perfectly still the gyro reads like
        # 0.01 rad/s and if u integrate that you drift like crazy. so we sit
        # there at the red light, average it, and subtract it off forever
        if not self.ready:
            self._sum += gz
            self._n += 1
            if self._n >= CALIB_FRAMES:
                self._bias = self._sum / self._n
                self.ready = True
            return

        self.yaw_rate = gz - self._bias
        self.yaw = wrap(self.yaw + self.yaw_rate * dt)

        # roll/pitch: gyro is smooth but drifts, gravity doesnt drift but is noisy
        # so just mix them (complementary filter, way easier than a kalman)
        self.roll += gx * dt
        self.pitch += gy * dt
        # heads up: this is the MAGNITUDE of the accel vector, we do NOT have a
        # magnetometer here. (the trial 2D node does, thats a whole different thing)
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        # only trust gravity when we're not slamming into stuff
        if abs(accel_norm - GRAVITY) < 2.0:
            self.roll = (1 - ACCEL_TRUST) * self.roll + ACCEL_TRUST * math.atan2(ay, az)
            self.pitch = (1 - ACCEL_TRUST) * self.pitch + ACCEL_TRUST * math.atan2(
                -ax, math.sqrt(ay * ay + az * az)
            )

    def heading(self):
        # degrees is just easier to read on the display
        return math.degrees(self.yaw)

    def turn_rate(self):
        return math.degrees(self.yaw_rate)
