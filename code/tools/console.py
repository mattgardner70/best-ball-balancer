"""Live serial console for the balancer.

    uv run python code/tools/console.py [PORT]

Shows everything the Mega prints (and logs it to console.log), unlocks the keys with the hello
phrase, and sends whatever you type: `n` + Return for the next pattern, `P` to raise P, `k1`, and so
on. An empty line does nothing; type `home` to send the re-home key. Ctrl-C to quit.
Opening the port resets the Mega, so it homes again.
"""

import glob
import sys
import threading
import time

import serial

HELLO = b"~balancer-hello~"


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        sys.exit("No Arduino found: plug in USB, or pass the port (e.g. COM3).")
    return ports[0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    s = serial.Serial(port, 115200, timeout=0.1)
    print(f"Connected to {port}. Type a key and press Return (Ctrl-C quits).")
    time.sleep(2.5)                           # the Mega resets when the port opens
    s.write(HELLO)
    log = open("console.log", "a")

    def reader():
        while True:
            text = s.read(4096).decode(errors="replace")
            if text:
                sys.stdout.write(text); sys.stdout.flush()
                log.write(text); log.flush()
                if "Balancer." in text:      # it restarted on its own: unlock the keys again
                    time.sleep(0.3); s.write(HELLO)

    threading.Thread(target=reader, daemon=True).start()
    try:
        for line in sys.stdin:
            cmd = line.strip()
            if cmd == "home":
                s.write(b"\n")
            elif cmd:
                s.write(cmd.encode())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
