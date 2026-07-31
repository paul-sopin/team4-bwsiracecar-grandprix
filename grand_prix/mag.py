"""
Team 4 - magnetometer reader for the Grand Prix AHRS

racecar_core gives us accel and gyro through rc.physics and nothing else. The
compass is a different chip on a different topic: /imu/realsense is the
RealSense IMU (accel + gyro only, the D435i has no magnetometer), and /mag is
an LSM9DS1 publishing sensor_msgs/MagneticField. Trial 2D's attitude_node
subscribes to both. We are not running ROS during the race, so this module does
the one piece of ROS plumbing we cannot avoid.

It runs its own node on its own executor and drains it without blocking, rather
than adding a node to whatever executor racecar_core is spinning. Two reasons:
we do not depend on racecar_core's internals, and a subscription that racecar's
loop forgets to spin would silently hand us a stale field forever, which reads
as "the compass works and the car is pointing that way" instead of as an error.

Everything here fails soft. No rclpy, no topic, nobody publishing: `available`
goes False, `read()` returns None, and the AHRS carries on gyro-only, which is
exactly what it did before this file existed.
"""

MAG_TOPIC = "/mag"

# Where the compass chip's axes point relative to the car. These are the
# chip_forward / chip_up parameters from Trial 2D's attitude_node, and they are
# NOT the same as the accel/gyro axes -- different chip, different mounting,
# which is why ahrs.py's runtime axis detection cannot be reused for this.
MAG_FORWARD = "z"
MAG_UP = "-y"

AXIS_VECTORS = {
    "x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0),
    "z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0),
}

MAX_DRAIN = 8      # callbacks pulled per read(), so a backlog cannot build up


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


class Magnetometer:
    """Latest /mag sample, in car body axes (forward, left, up), in tesla.

    Usage:
        mag = Magnetometer()
        ...
        field = mag.read()      # (fwd, left, up) or None
    """

    def __init__(self, topic=MAG_TOPIC, forward=MAG_FORWARD, up=MAG_UP):
        self.topic = topic
        self.available = False
        self.reason = "not connected yet"
        self.samples = 0

        self._e_fwd = AXIS_VECTORS[forward]
        self._e_up = AXIS_VECTORS[up]
        self._e_left = _cross(self._e_up, self._e_fwd)

        self._tried = False
        self._executor = None
        self._node = None
        self._latest = None

    def _connect(self):
        """One attempt, on the first read(). Never raises."""
        self._tried = True
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import MagneticField
        except ImportError as error:
            self.reason = "no rclpy/sensor_msgs: {}".format(error)
            return

        try:
            # racecar_core normally does this first. if it has not, and we are
            # being used from a bench script, bring it up ourselves
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("grand_prix_mag_reader")
            # sensor QoS is best-effort: a dropped compass sample costs us
            # nothing, and reliable QoS would not match the publisher anyway
            self._node.create_subscription(MagneticField, self.topic,
                                           self._callback, qos_profile_sensor_data)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as error:   # noqa: BLE001 - never take the car down
            self.reason = "subscribe failed: {}".format(error)
            self._executor = self._node = None
            return

        self.reason = "subscribed to {}, waiting for data".format(self.topic)

    def _callback(self, msg):
        field = (msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)
        self._latest = (_dot(field, self._e_fwd),
                        _dot(field, self._e_left),
                        _dot(field, self._e_up))
        self.samples += 1
        if not self.available:
            self.available = True
            self.reason = "receiving {}".format(self.topic)

    def read(self):
        """Newest field vector in body axes, or None. Does not block."""
        if not self._tried:
            self._connect()
        if self._executor is None:
            return None
        try:
            # timeout 0 means "handle what is already waiting and return".
            # looping drains a backlog so we always act on the newest sample
            for _ in range(MAX_DRAIN):
                self._executor.spin_once(timeout_sec=0.0)
        except Exception as error:   # noqa: BLE001
            self.reason = "spin failed: {}".format(error)
            self._executor = None
            self.available = False
            return None
        return self._latest

    def status(self):
        return self.reason

    def shutdown(self):
        if self._node is not None:
            try:
                self._executor.remove_node(self._node)
                self._node.destroy_node()
            except Exception:   # noqa: BLE001
                pass
            self._executor = self._node = None
