"""Control library for the DH-Robotics DH5 six-axis hand.

Layers, bottom to top:

    dh5.registers     addresses and limits, pure data
    dh5.conversions   raw registers <-> percent / float, pure functions
    dh5.modbus        DH5ModbusAPI - serial Modbus RTU transport
    dh5.hand          DH5Hand - axes, percent, simultaneous moves, sensors
    dh5.sensors       stale-data and contact detection for fingertip readings
    dh5.gestures      poses as data plus one runner
    dh5.cli           the interactive DH5> prompt

Typical use::

    from dh5 import DH5Hand, configure_logging

    configure_logging()
    with DH5Hand(port="COM3") as hand:
        if hand.initialize():
            hand.move({2: 100, 5: 100})
"""

import logging

from . import conversions, gestures, registers, sensors
from .gestures import GESTURES, Gesture, Pose, perform
from .hand import DH5Hand, HandState, MoveResult, format_faults
from .modbus import DH5ModbusAPI

__version__ = "1.0.0"

__all__ = [
    "DH5Hand",
    "DH5ModbusAPI",
    "GESTURES",
    "Gesture",
    "HandState",
    "MoveResult",
    "Pose",
    "configure_logging",
    "conversions",
    "format_faults",
    "gestures",
    "perform",
    "registers",
    "sensors",
]


def configure_logging(level=logging.INFO, stream=None) -> None:
    """Send this library's log records to the console in the `[dh5] message`
    style the old `log()` helper used.

    Applications that configure logging themselves (ROS2 nodes, test
    runners) should simply not call this - the library only ever writes to
    `logging.getLogger("dh5.*")` and adds no handler of its own.
    """
    logger = logging.getLogger(__name__)
    # The library installs a NullHandler by default, which must not count.
    if not any(not isinstance(h, logging.NullHandler) for h in logger.handlers):
        handler = logging.StreamHandler(stream)
        handler.setFormatter(logging.Formatter("[dh5] %(message)s"))
        logger.addHandler(handler)
    logger.setLevel(level)
    logger.propagate = False


# Keep the library silent by default if the application configures nothing,
# rather than printing "No handlers could be found" style warnings.
logging.getLogger(__name__).addHandler(logging.NullHandler())
