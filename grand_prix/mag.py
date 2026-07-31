"""
Team 4 - compass reader for the AHRS

rc.physics gives us accel and gyro and that is all it gives us. The compass is a
different chip. /imu/realsense is the RealSense IMU, which has no magnetometer
in it, and the compass is an LSM9DS1 publishing on /mag. attitude_node in Trial
2D reads both of them. We are not running ROS during a race, so this file is the
one bit of ROS we could not get around.

It makes its own node and its own executor instead of hanging a subscription off
whatever racecar_core is spinning. We did not want to depend on racecar_core
internals, and if anything stops spinning our subscription we would keep getting
the last reading forever. That does not look broken. It looks like the compass
is working and the car is holding a steady heading, which is worse.

Everything in here fails quietly. No rclpy, no topic, nothing publishing, and
available stays False, read() gives back None, and the AHRS goes back to running
on the gyro alone the way it did before we wrote any of this.
"""

MAG_TOPIC = "/mag"

# Which way the compass chip is facing in the car. Same chip_forward and chip_up
# values attitude_node uses in Trial 2D. These are not the accel and gyro axes.
# It is a different chip mounted a different way, so the axis detection in
# ahrs.py tells us nothing about this one.
MAG_FORWARD = "z"
MAG_UP = "-y"

AXIS_VECTORS = {
    "x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0),
    "z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0),
}

MAX_DRAIN = 8      # how many callbacks we pull per read, so nothing piles up


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


class Magnetometer:
    """Last compass reading, in car axes (forward, left, up), in tesla.

    Usage:
        mag = Magnetometer()
        ...
        field = mag.read()      # (fwd, left, up), or None
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
        """Runs once, on the first read. Does not throw."""
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
            # racecar_core has usually done this already. if it has not we are
            # probably being run from a bench script, so start it ourselves
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("grand_prix_mag_reader")
            # sensor QoS is best effort. losing a compass reading costs us
            # nothing, and a reliable subscriber would not match the publisher,
            # so it would connect to nothing and look just like a dead topic
            self._node.create_subscription(MagneticField, self.topic,
                                           self._callback, qos_profile_sensor_data)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as error:   # noqa: BLE001, losing the compass beats crashing
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
        """Newest reading in car axes, or None. Never blocks."""
        if not self._tried:
            self._connect()
        if self._executor is None:
            return None
        try:
            # timeout 0 means handle whatever is already waiting and come
            # straight back. the loop is so we end up on the newest reading
            # instead of working through old ones one frame at a time
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
        # nothing calls this during a race. the process dies and takes the node
        # with it. it is here for bench scripts that reconnect without restarting
        if self._node is not None:
            try:
                self._executor.remove_node(self._node)
                self._node.destroy_node()
            except Exception:   # noqa: BLE001
                pass
            self._executor = self._node = None
