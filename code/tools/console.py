"""Serial console for the balancer: keeps the port open (every open resets the Mega), prints the log,
and sends what you type.

    uv run python -m tools.console [port]

Typing:
    cx 21        a setting (any word + value): sent as the firmware command ":cx 21"
    get          every setting      save  keep them through a reset      mode circle  switch pattern
    n            a single character is sent as that key (n = next pattern, s = stop, ? = ball position)
    home         re-home (Enter on its own does nothing, so a stray Enter can't re-home)

Everything is also appended to tools/console.log, and lines appended to tools/console.cmd are sent
as if typed (so another program can drive it while it runs).
"""
import glob
import pathlib
import sys
import threading
import time

import serial

HERE = pathlib.Path(__file__).parent
LOG, CMD = HERE / "console.log", HERE / "console.cmd"
HELLO = b"~balancer-hello~"


def encode(line):
    """What to send for one typed line."""
    line = line.strip()
    if not line:
        return b""
    if line == "home":
        return b"\n"
    if len(line) == 1:
        return line.encode()
    return b":" + line.lstrip(":").encode() + b"\n"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else (sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/ttyACM*")
                                                        + glob.glob("/dev/ttyUSB*")) or sys.exit("No Arduino found: plug in USB, or pass the port"))[0]
    s = serial.Serial(port, 115200, timeout=0.05)
    lock = threading.Lock()
    log = open(LOG, "a", buffering=1)
    t0 = time.time()

    def send(data, why):
        if data:
            with lock:
                s.write(data)
            log.write(f"{time.time() - t0:8.2f} >>> {why}\n")

    def reader():
        hello = False
        while True:
            line = s.readline()
            if line:
                text = line.decode(errors="replace").rstrip()
                if not text.startswith("V "):            # the viz stream goes to the log only
                    print(text, flush=True)
                log.write(f"{time.time() - t0:8.2f} {text}\n")
            if not hello and time.time() - t0 > 2.5:      # after the bootloader: keys on
                send(HELLO, "hello")
                hello = True

    def cmd_file():
        CMD.write_text("")
        done = 0
        while True:
            lines = CMD.read_text().splitlines()
            for line in lines[done:]:
                send(encode(line), line)
                time.sleep(0.03)            # the Mega's input buffer is 64 bytes: a burst of lines overflows it
            done = len(lines)
            time.sleep(0.1)

    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=cmd_file, daemon=True).start()
    print(f"{port}: the Mega resets on open, homes and raises. Type 'get' for the settings, Ctrl-C to quit.")
    try:
        for line in sys.stdin:
            send(encode(line), line.strip())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
