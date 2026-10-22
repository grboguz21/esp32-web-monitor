"""Steady-state speed sweep for motor 1: duty steps from high to low in + and - direction.

Talks to the firmware (0.12.0 or later) over USB the same way the dashboard does: RUN / STOP /
PING commands, ENC reports. Writes a CSV next to this script.
"""
import csv
import statistics
import sys
import threading
import time
from pathlib import Path

import serial

PORT = "/dev/ttyUSB0"
BAUD = 460800
MOTOR = 1
CPR = 186.2                 # measured counts per output-shaft revolution
SUPPLY_V = 12.0
DUTIES = [100, 80, 60, 50, 40, 30, 25, 20, 17, 15, 13, 12, 11, 10]
SETTLE_S = 2.0
SAMPLE_S = 1.5
STOPPED_RPM = 5.0

lock = threading.Lock()
latest = {"cps": None, "samples": [], "collecting": False, "hello": False}
running = True


def reader(ser):
    buf = b""
    while running:
        buf += ser.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("ascii", "ignore").strip()
            if text.startswith("HELLO,"):
                latest["hello"] = True
            elif text.startswith("ENC,"):
                parts = text.split(",")
                if len(parts) == 5:
                    cps = float(parts[2 if MOTOR == 1 else 4])
                    with lock:
                        latest["cps"] = cps
                        if latest["collecting"]:
                            latest["samples"].append(cps)


def pinger(ser):
    while running:
        ser.write(b"PING\n")
        time.sleep(0.2)


def send(ser, text):
    ser.write((text + "\n").encode())


def measure(ser, duty):
    send(ser, f"RUN,{MOTOR},{duty},0")
    time.sleep(SETTLE_S)
    with lock:
        latest["samples"] = []
        latest["collecting"] = True
    time.sleep(SAMPLE_S)
    with lock:
        latest["collecting"] = False
        samples = list(latest["samples"])
    rpm = [s * 60 / CPR for s in samples]
    mean = statistics.fmean(rpm) if rpm else float("nan")
    spread = statistics.pstdev(rpm) if len(rpm) > 1 else 0.0
    return mean, spread, len(rpm)


def main():
    global running
    out = Path(__file__).with_name("kv_sweep.csv")   # written next to this script
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()
    # Opening the port may reset the board; wait for it to identify itself
    deadline = time.time() + 6
    send(ser, "HELLO?")
    while not latest["hello"] and time.time() < deadline:
        time.sleep(0.2)
        send(ser, "HELLO?")
    if not latest["hello"]:
        print("No HELLO from the board, aborting")
        return 1
    send(ser, f"ENCINV,{MOTOR},0")
    threading.Thread(target=pinger, args=(ser,), daemon=True).start()

    rows = []
    try:
        for direction in (1, -1):
            for duty in DUTIES:
                signed = duty * direction
                mean, spread, n = measure(ser, signed)
                volts = signed / 100 * SUPPLY_V
                rows.append((signed, volts, mean, spread, n))
                print(f"duty {signed:+4d} %  {volts:+6.2f} V  ->  {mean:+8.1f} RPM  (±{spread:.1f}, n={n})",
                      flush=True)
                if abs(mean) < STOPPED_RPM:
                    break   # below the start threshold it would not restart
            send(ser, "STOP")
            time.sleep(1.5)
    finally:
        send(ser, "STOP")
        time.sleep(0.3)
        running = False
        ser.close()

    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["duty_pct", "volts", "rpm_mean", "rpm_std", "samples"])
        w.writerows(rows)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
