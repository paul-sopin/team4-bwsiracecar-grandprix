"""
telemetry.py
Telemetry and debugging for the gap follower.

Thin wrapper over rc.telemetry, which racecar_neo already gives us for recording
and graphing. The one thing it can't do is live viewing, it records now and
graphs later, so the on screen HUD is still worth having.

Usage, inside grand_prix_AHRS.py:
    from telemetry import TelemetryLogger
    logger = TelemetryLogger(rc)   # once, outside the main loop
    def update():
        ...
        logger.log(
            target_angle=target_angle,
            left_dist=left_dist,
            right_dist=right_dist,
            angle=angle,
            speed=speed,
            heading=imu.heading(),
            turn_rate=imu.turn_rate(),
            gap_bias=GAP_BIAS[gap_mode],
        )
        logger.draw_hud(rc, target_angle, angle, speed, heading=imu.heading())

Nothing to call at the end, visualize() fires on exit and makes the graph.
"""

# ORDER MATTERS. rc.telemetry.record() is positional, not keyed. This tuple is
# the one place the order is defined and everything downstream follows it, so
# don't rearrange it without checking grand_prix_AHRS.py too.
#
# These are the V2 gap follower fields, which is what races.
FIELD_ORDER = (
    "target_angle",
    "left_dist",
    "right_dist",
    "angle",
    "speed",
    "heading",
    "turn_rate",
    "gap_bias",   # which gap mode the tag reader has us in, as a number so it
                  # graphs. -1 leftmost, 0 largest, 1 rightmost
)


class TelemetryLogger:
    def __init__(self, rc):
        # needs rc because the recording lives on rc.telemetry, not a file we open
        self._rc = rc
        # declare_variables only does anything the first time it's ever called.
        # change FIELD_ORDER and you have to restart the script.
        self._rc.telemetry.declare_variables(*FIELD_ORDER)
        self._frame = 0  # just for us, rc.telemetry does its own timestamps

    # keywords here because it reads better at the call site, converted to the
    # positional order record() wants. miss a field and you get a KeyError,
    # which beats silently logging the wrong columns and finding out from the graph.
    def log(self, **fields):
        try:
            values = tuple(fields[name] for name in FIELD_ORDER)
        except KeyError as missing:
            raise KeyError(
                f"missing telemetry field {missing}, expected all of: {FIELD_ORDER}"
            ) from missing

        self._rc.telemetry.record(*values)
        self._frame += 1
        # no flush, no file handle. rc.telemetry persists this itself.

    def draw_hud(self, rc, target_angle, angle, speed, heading=None):
        """
        Debug overlay: text readout plus a bar showing where the controller
        thinks the open gap is, relative to straight ahead. Watchable live
        instead of waiting on the post run graph.

        Pass heading to get the AHRS heading in the readout.

        DON'T CALL THIS IN THE RACE LOOP on the dot matrix. That display is 8x24
        and scrolls anything longer at ~2 chars/sec. This readout is much longer
        than that, so every frame leaves it permanently mid scroll and you can't
        read any of it. It's for slow tuning runs, or a display with room for a
        real line of text. The race script uses update_slow() once a second.
        """
        hud_text = f"tgt:{target_angle:5.1f} ang:{angle:+.2f} spd:{speed:.2f}"
        if heading is not None:
            hud_text += f" hdg:{heading:+.0f}"

        # bar goes -90 (left) through 0 (center) to +90 (right)
        bar_width = 21  # odd on purpose so there's a clean center tick
        center = bar_width // 2
        pos = int(center + (target_angle / 90.0) * center)
        pos = max(0, min(bar_width - 1, pos))  # clamp in case the angle is out of range

        bar = ["-"] * bar_width
        bar[center] = "|"   # straight ahead
        bar[pos] = "X"      # where we're aiming
        hud_bar = "".join(bar)

        rc.display.show_text(hud_text + "\n" + hud_bar)

    # force a graph now instead of waiting for exit. for long test runs where you
    # want to check the data partway through.
    def save_graph(self):
        self._rc.telemetry.visualize()
