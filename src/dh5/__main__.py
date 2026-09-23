"""DH5 six-axis hand - startup script and manual command prompt.

Run this to bring the hand up and get an interactive prompt:

    python -m dh5                            # defaults below  (or: dh5)
    python -m dh5 --port COM5
    python -m dh5 --no-demo        # straight to the prompt

What it does:

1. Open the serial connection.
2. Clear any standing fault.
3. Initialize all six axes and wait until every one reports ready.
4. Set a starting speed and force on all axes (one Modbus frame each).
5. Optionally run a short demo move.
6. Drop into the `DH5>` prompt, where the hand is only driven by the
   commands you type. Type `help` there for the command list.
7. Close the connection on the way out, whatever happened.
"""

import argparse
import logging
import sys

from dh5 import DH5Hand, cli, configure_logging, hand, registers

logger = logging.getLogger("dh5.startup")

# --------------------------------------------------------------------------
# Defaults - override any of these on the command line.
# --------------------------------------------------------------------------
PORT = "COM3"
MODBUS_ID = 1
BAUD_RATE = 115200
STOP_BITS = 1
PARITY = "N"

INIT_MODE = registers.INIT_MODE_OPEN  # 1 = close, 2 = open, 3 = find total stroke
INIT_TIMEOUT = 20.0                   # seconds to wait for all axes
INIT_POLL_INTERVAL = 0.5              # seconds between status checks

START_SPEED = 5  # percent, applied to every axis at startup
START_FORCE = 100  # percent, applied to every axis at startup


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default=PORT, help=f"serial port (default {PORT})")
    parser.add_argument("--baud-rate", type=int, default=BAUD_RATE)
    parser.add_argument("--modbus-id", type=int, default=MODBUS_ID)
    parser.add_argument("--init-mode", type=int, default=INIT_MODE, choices=registers.INIT_MODES,
                        help="1 = close, 2 = open, 3 = find total stroke")
    parser.add_argument("--speed", type=int, default=START_SPEED,
                        help=f"startup speed for all axes (default {START_SPEED})")
    parser.add_argument("--force", type=int, default=START_FORCE, help="startup force for all axes")
    parser.add_argument("--no-demo", action="store_true", help="skip the startup demo move")
    parser.add_argument("--no-init", action="store_true", help="skip initialization (already homed)")
    parser.add_argument("--verbose", action="store_true", help="log every Modbus exchange")
    return parser.parse_args(argv)


def demo(hand: DH5Hand) -> None:
    """Move every axis to 60%, wait for it, then on to 100%.

    Both moves wait for completion. Without the wait, the second command
    overwrites the setpoint before the hand has travelled and the 60%
    position is never actually reached - which is what the original script
    did.
    """
    logger.info("Demo: all axes to 60%, then to 100%.")
    # hand.move({axis: 50 for axis in hand.axes}, wait=True)
    hand.move({axis: 60 for axis in hand.axes}, wait=True)
    hand.move({axis: 99 for axis in hand.axes}, wait=True)


def main(argv=None) -> int:
    args = parse_args(argv)
    configure_logging(logging.DEBUG if args.verbose else logging.INFO)

    hand = DH5Hand(
        port=args.port,
        modbus_id=args.modbus_id,
        baud_rate=args.baud_rate,
        stop_bits=STOP_BITS,
        parity=PARITY,
    )

    if not hand.connect():
        logger.error("Could not open %s - is the hand plugged in and the port free?", args.port)
        return 1

    try:
        if not args.no_init:
            if not hand.initialize(args.init_mode, timeout=INIT_TIMEOUT,
                                   poll_interval=INIT_POLL_INTERVAL):
                logger.error("Aborting: initialization did not complete.")
                return 1

        # One frame each, so every axis takes the new value at the same instant.
        hand.set_speeds({axis: args.speed for axis in hand.axes})
        hand.set_forces({axis: args.force for axis in hand.axes})

        if not args.no_demo:
            demo(hand)

        cli.run(hand)
        return 0

    except KeyboardInterrupt:
        logger.warning("Interrupted.")
        return 130
    finally:
        logger.info("Closing connection...")
        hand.close()


if __name__ == "__main__":
    sys.exit(main())
