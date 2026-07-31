"""
Team 4 - small AHRS for the Grand Prix

Same idea as our state_estimation attitude_node, just way smaller. That one is a
full ROS 2 package and we can't run ROS on top of racecar_core during the race,
so this does the bare minimum: read the IMU, report heading and turn rate.

Yaw is the one axis with nothing to check it. Roll and pitch get pulled straight
by gravity every frame; nothing in the IMU knows which way is north, so yaw is
pure integration and slides. attitude_node solves that with a magnetometer in a
yaw EKF, and the same fix scales down to a complementary filter here -- see
MAG_TRUST below and mag.py, which does the ROS plumbing to actually get /mag.
Without a compass this behaves exactly as it did before: gyro-only, drifting.
"""

import math
import time

CALIB_FRAMES = 120     # ~2 seconds at rest. don't move the car during this
ACCEL_TRUST = 0.02     # accelerometer weight. keep it small or roll/pitch gets jittery
GRAVITY = 9.81

# --- magnetometer, for the yaw complementary filter -------------------------
# Gravity fixes roll and pitch because it tells us which way is down. Nothing
# tells us which way is north, so yaw is pure integration and drifts. The
# compass is the only sensor on the car that can bound it.
#
# This is the same trick as roll/pitch, one axis over: gyro is smooth and
# drifts, compass is noisy and doesn't, so lean on the gyro and nudge toward
# the compass. Everything below exists because a compass on an RC car is a much
# worse sensor than gravity is, and most of it is about deciding when NOT to
# believe it.
MAG_TRUST = 0.02       # same weight as ACCEL_TRUST, and for the same reason

# Hard-iron offset. The car's own steel and magnets shift the field by more than
# Earth's field is strong, so raw compass readings are useless until this is
# subtracted. Leave as None to learn it while driving; fill it in from a
# previous run's printout (in tesla) to skip the learning and lock on sooner.
MAG_OFFSET = None                # e.g. (12.4e-6, -3.1e-6, 40.2e-6)
MAG_SCALE = (1.0, 1.0, 1.0)      # soft-iron, learned alongside the offset

# Which way the compass heading turns compared to the gyro. 0 works it out by
# watching the two while the car drives; +1/-1 pin it. Getting this backwards
# makes the correction drive yaw away from truth instead of toward it, so it is
# measured rather than assumed.
MAG_SENSE = 0

MAG_MIN_FIELD = 15e-6            # Earth's field is ~25-65 uT. outside this band
MAG_MAX_FIELD = 90e-6            # we are reading a motor, not the planet
MAG_INNOV_GATE = math.radians(35)   # ignore corrections larger than this
MAG_READY_SPREAD = 6.0e-6        # min/max spread needed before the offset is trusted
MAG_READY_TURN = math.radians(260)  # and how far the car must have turned
MAG_SENSE_TURN = math.radians(20)   # much less turning needed to just get the sign
MAG_SENSE_STEP = math.radians(0.5)  # ignore frames the car barely turned in


def wrap(a):
    # -pi to pi, so the angle never accumulates past a full turn
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class AHRS:
    def __init__(self, mag=None):
        """mag: anything with a .read() returning (fwd, left, up) in tesla, or
        None. See mag.py. Without one this is exactly the filter it always was."""
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

        # --- compass state ---
        self.mag = mag
        self.mag_locked = False    # True once yaw is actually being corrected
        self.mag_field = None      # last corrected reading, for the printout
        self.mag_rejected = 0      # samples thrown out since the last status()
        self.mag_used = 0

        self._mag_offset = list(MAG_OFFSET) if MAG_OFFSET else [0.0, 0.0, 0.0]
        self._mag_scale = list(MAG_SCALE)
        self._mag_learning = MAG_OFFSET is None
        self._mag_min = [math.inf] * 3
        self._mag_max = [-math.inf] * 3
        self._mag_radius = [0.0, 0.0, 0.0]

        self._mag_sense = MAG_SENSE      # 0 until worked out
        self._sense_score = 0.0          # correlation between compass and gyro
        self._mag_ref = None             # compass heading of "yaw = 0"
        self._prev_heading = None
        self._prev_yaw = None
        self._turned = 0.0               # total yaw travel since calibration

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

    # -- compass ----------------------------------------------------------
    def _learn_hard_iron(self, raw):
        """Track each axis's extremes while the car turns.

        A compass swung through a full circle traces a sphere centred on zero.
        Ours traces one pushed off centre by the car's own steel and motor
        magnets, by more than Earth's field is strong. So the centre of the
        min/max box IS that offset, and the radii are the soft-iron scale.
        This is why the raw reading is useless until the car has turned.
        """
        for i in range(3):
            self._mag_min[i] = min(self._mag_min[i], raw[i])
            self._mag_max[i] = max(self._mag_max[i], raw[i])
            if self._mag_max[i] > self._mag_min[i]:
                self._mag_offset[i] = 0.5 * (self._mag_max[i] + self._mag_min[i])
                self._mag_radius[i] = 0.5 * (self._mag_max[i] - self._mag_min[i])

        good = [r for r in self._mag_radius if r > 1e-9]
        if len(good) >= 2:
            average = sum(good) / len(good)
            self._mag_scale = [average / r if r > 1e-9 else 1.0
                               for r in self._mag_radius]

    def _mag_heading(self, raw):
        """Tilt compensated compass heading, or None if the field looks wrong."""
        field = tuple((raw[i] - self._mag_offset[i]) * self._mag_scale[i]
                      for i in range(3))
        strength = math.sqrt(sum(c * c for c in field))
        # Earth's field is 25-65 uT everywhere on the planet. Outside this band
        # we are reading the drive motor, a steel door frame, or a bad offset
        if not MAG_MIN_FIELD < strength < MAG_MAX_FIELD:
            self.mag_rejected += 1
            return None
        self.mag_field = field

        m_fwd, m_left, m_up = field
        cos_r, sin_r = math.cos(self.roll), math.sin(self.roll)
        cos_p, sin_p = math.cos(self.pitch), math.sin(self.pitch)
        # rotate the field flat before taking its angle. without this, leaning
        # the car into a corner reads as the car having turned
        flat_fwd = m_fwd * cos_p + m_left * sin_r * sin_p + m_up * cos_r * sin_p
        flat_left = m_left * cos_r - m_up * sin_r
        return math.atan2(-flat_left, flat_fwd)

    def _track_sense(self, heading):
        """Does the compass turn the same way the gyro says we are turning?

        Whether the compass reads left handed depends on how the chip is
        mounted, and ahrs can't reuse the accel axes to find out -- different
        chip. Backwards would make every correction shove yaw away from truth,
        so it gets measured: multiply the two changes together and watch the
        sign. Turns that agree give a positive product, opposed turns a
        negative one, and noise averages to nothing.
        """
        if self._prev_heading is not None:
            d_heading = wrap(heading - self._prev_heading)
            d_yaw = wrap(self.yaw - self._prev_yaw)
            if abs(d_yaw) > MAG_SENSE_STEP:   # too small to tell signal from noise
                self._sense_score += d_heading * d_yaw
        self._prev_heading, self._prev_yaw = heading, self.yaw

    def _try_lock(self, heading):
        """Start correcting yaw, once the compass has earned it."""
        if self._mag_learning:
            need_turn = MAG_READY_TURN      # a full circle, to trace the sphere
        elif not MAG_SENSE:
            need_turn = MAG_SENSE_TURN      # enough to see which way it moves
        else:
            need_turn = 0.0                 # offset and sense both given, lock now

        if self._turned < need_turn:
            return
        if self._mag_learning and min(self._mag_radius[0],
                                      self._mag_radius[1]) < MAG_READY_SPREAD:
            return

        if MAG_SENSE:
            sense = MAG_SENSE
        elif self._sense_score:
            sense = 1 if self._sense_score > 0 else -1
        else:
            return                          # no evidence either way yet

        self._mag_sense = sense
        # Pin the compass to where we already think we are, not to true north.
        # yaw keeps meaning "radians from however the car was pointed at
        # startup", so the display, the logs and anything reading heading()
        # don't quietly change meaning the moment this locks. All we want from
        # the compass is that yaw stops sliding, not a new definition of zero.
        self._mag_ref = wrap(sense * heading - self.yaw)
        self.mag_locked = True
        # Stop learning the offset here. The reference above was measured
        # against THIS offset, so letting it keep moving afterwards slides the
        # compass heading out from under it -- which is drift, reintroduced
        # through the calibration we added to remove drift.
        self._mag_learning = False
        print("AHRS: compass locked in. sense={} offset uT=({:+.1f},{:+.1f},{:+.1f})"
              .format(sense, *[o * 1e6 for o in self._mag_offset]))

    def _mag_update(self):
        """One compass sample folded into yaw. Same complementary idea as
        roll/pitch: trust the gyro, nudge toward the compass."""
        if self.mag is None:
            return
        raw = self.mag.read()
        if raw is None:
            return

        if self._mag_learning:
            self._learn_hard_iron(raw)

        heading = self._mag_heading(raw)
        if heading is None:
            return

        self._track_sense(heading)

        if not self.mag_locked:
            self._try_lock(heading)
            return

        measured = wrap(self._mag_sense * heading - self._mag_ref)
        innovation = wrap(measured - self.yaw)
        # A correction this large is not drift -- drift is slow. It is a steel
        # doorway, a motor spike, or a bad lock. Coast on the gyro instead
        if abs(innovation) > MAG_INNOV_GATE:
            self.mag_rejected += 1
            return
        self.yaw = wrap(self.yaw + MAG_TRUST * innovation)
        self.mag_used += 1

    def mag_status(self):
        """One line for update_slow(). Resets the used/rejected counters."""
        if self.mag is None:
            return "off (no reader passed to AHRS)"
        if not getattr(self.mag, "available", True):
            return "NO DATA, yaw is gyro-only: " + self.mag.status()

        used, rejected = self.mag_used, self.mag_rejected
        self.mag_used = self.mag_rejected = 0

        if not self.mag_locked:
            return ("calibrating, TURN THE CAR: radii x={:.1f} y={:.1f}uT "
                    "(need {:.0f}), turned {:.0f}/{:.0f}deg, sense={}"
                    .format(self._mag_radius[0] * 1e6, self._mag_radius[1] * 1e6,
                            MAG_READY_SPREAD * 1e6, math.degrees(self._turned),
                            math.degrees(MAG_READY_TURN if self._mag_learning
                                         else MAG_SENSE_TURN),
                            "?" if not self._sense_score
                            else ("+1" if self._sense_score > 0 else "-1")))

        strength = (math.sqrt(sum(c * c for c in self.mag_field)) * 1e6
                    if self.mag_field else 0.0)
        note = " INTERFERENCE" if rejected > used else ""
        return ("LOCKED sense={} |B|={:.0f}uT used/rejected={}/{}{}"
                .format(self._mag_sense, strength, used, rejected, note))

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
        # magnitude of the accel vector, to tell gravity apart from manoeuvring
        accel_norm = math.sqrt(a_fwd * a_fwd + a_side * a_side + a_up * a_up)
        # only trust gravity when we're not braking, cornering hard, or hitting something
        if abs(accel_norm - GRAVITY) < 2.0:
            self.roll = (1 - ACCEL_TRUST) * self.roll + ACCEL_TRUST * math.atan2(a_side, a_up)
            self.pitch = (1 - ACCEL_TRUST) * self.pitch + ACCEL_TRUST * math.atan2(
                -a_fwd, math.sqrt(a_side * a_side + a_up * a_up)
            )

        # gravity has no opinion about yaw, so that one integrates unchecked and
        # slides. the compass is the only thing that can pull it back. after
        # roll/pitch on purpose, the tilt compensation reads them
        self._turned += abs(self.yaw_rate) * dt
        self._mag_update()

    def heading(self):
        # degrees, easier to read on the display and in the console
        return math.degrees(self.yaw)

    def turn_rate(self):
        return math.degrees(self.yaw_rate)
