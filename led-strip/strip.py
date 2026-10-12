"""Control the LED strip on the Atech board from the command line.

Examples:
    python strip.py state
    python strip.py effect rainbow
    python strip.py color warm
    python strip.py brightness 150
    python strip.py test colors      # watch the strip, compare with what's printed
    python strip.py test count
    python strip.py count 48
    python strip.py order BRG
    python strip.py flash red:400
    python strip.py fill 0:9:blue

Use --port COM6 to pick the port (auto-detected otherwise).
"""

import argparse
import sys
import time

from atech import Board

PREFIX = "strip"


def next_events(board, key, timeout):
    """Yield values of events with this key until the timeout runs out."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        event = board.transport.recv(timeout=0.1)
        if event is not None and event.key == key:
            yield event.value


def wait_for(board, key, timeout=3.0):
    """Return the value of the first event with this key, or None."""
    return next(next_events(board, key, timeout), None)


def main() -> int:
    parser = argparse.ArgumentParser(description="Control the Atech LED strip.")
    parser.add_argument("command", help="state, effect, color, brightness, speed, flash, pixel, fill, "
                                        "count, order, khz, test, off")
    parser.add_argument("value", nargs="?", default="", help="value for the command")
    parser.add_argument("--port", default=None, help="serial port, e.g. COM6")
    args = parser.parse_args()

    cmd = "dump" if args.command == "state" else args.command

    with Board.connect(port=args.port) as board:
        time.sleep(0.3)
        board.send(f"{PREFIX}_{cmd}", args.value)

        if cmd == "test" and args.value in ("", "colors"):
            print("Sending pure colours, 1.5 s each. Note what the strip SHOWS for each:")
            for sent in next_events(board, f"{PREFIX}_test", timeout=9.5):
                print(f"  sending {sent.upper():5}  ->  strip shows: ?")
            print("If the strip showed e.g. GREEN while RED was sent, the order is wrong —\n"
                  "run `python strip.py order <XYZ>` where X/Y/Z are the colours it showed\n"
                  "for red/green/blue sends, e.g. green,red,blue -> GRB.")
            return 0

        # Opening the port can reboot the board, swallowing the first command.
        # If nothing comes back, repeat the command, then fall back to asking for state.
        state = wait_for(board, f"{PREFIX}_state", timeout=1.5)
        for retry_cmd, retry_value in ((cmd, args.value), ("dump", "")):
            if state is not None:
                break
            board.send(f"{PREFIX}_{retry_cmd}", retry_value)
            state = wait_for(board, f"{PREFIX}_state", timeout=2.0)
        if state is None:
            print("No reply from the board. Is the led-strip firmware flashed, and the port free?")
            return 1
        print(", ".join(f"{k}={v}" for k, v in state.items()))
        if cmd == "test" and args.value == "count":
            print("First pixel is RED, every 10th is WHITE, the last configured one is GREEN.\n"
                  "If lit pixels stop before green -> count too high; if the strip continues\n"
                  "past green -> count too low. Fix with `python strip.py count N`.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
