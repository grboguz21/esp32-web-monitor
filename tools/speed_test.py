"""Speed-control check for motor 1 (firmware 0.15.0 or later): target steps with feedforward only,
then with feedforward + PI, logged at 200 Hz. Prints rise time, overshoot, settling and
steady-state error per step; raw ticks go to speed_test_raw.csv.
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
SEGMENT_S = 2.0
RUNS = [
    ("FF", 1, [300, 600, 300]),
    ("FF+PI", 2, [300, 600, 300, 150, -400]),
]

lock = threading.Lock()
state = {"hello": False, "log": []}
running = True


def reader(ser):
    buf = b""
    while running:
        try:
            buf += ser.read(1024)
        except (serial.SerialException, TypeError):
            return
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("ascii", "ignore").strip()
            if text.startswith("HELLO,"):
                state["hello"] = True
            elif text.startswith("L,"):
                p = text.split(",")
                if len(p) == 8:   # L,t,duty,c1,c2,target,ref,speed
                    with lock:
                        state["log"].append((int(p[1]) / 1e6, float(p[2]), float(p[5]), float(p[7]), float(p[6])))


def pinger(ser):
    while running:
        try:
            ser.write(b"PING\n")
        except serial.SerialException:
            return
        time.sleep(0.2)


def send(ser, text):
    ser.write((text + "\n").encode())


def step_metrics(seg, prev_target, target):
    """seg: list of (t, duty, target, speed) from the step on."""
    t0 = seg[0][0]
    t = [r[0] - t0 for r in seg]
    w = [r[3] for r in seg]
    step = target - prev_target
    tail = [wi for ti, wi in zip(t, w) if ti > t[-1] - 0.5]
    ss_err = target - statistics.fmean(tail)
    ripple = statistics.pstdev(tail)
    rise = None
    if abs(step) > 1:
        lo, hi = prev_target + 0.1 * step, prev_target + 0.9 * step
        t10 = next((ti for ti, wi in zip(t, w) if (wi - lo) * step >= 0), None)
        t90 = next((ti for ti, wi in zip(t, w) if (wi - hi) * step >= 0), None)
        if t10 is not None and t90 is not None:
            rise = t90 - t10
    peak = max(w) if step > 0 else min(w)   # raw peak; settling below uses a smoothed trace
    overshoot = max(0.0, (peak - target) / step * 100) if abs(step) > 1 else 0.0
    band = max(0.05 * abs(step), 5)
    smooth = [statistics.fmean(w[max(0, i - 5):i + 5]) for i in range(len(w))]   # 50 ms centred
    settle = None
    for i in range(len(smooth) - 1, -1, -1):
        if abs(smooth[i] - target) > band:
            settle = t[i + 1] if i + 1 < len(t) else None
            break
    else:
        settle = 0.0
    return rise, overshoot, settle, ss_err, ripple


def main():
    global running
    here = Path(__file__).parent
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()
    deadline = time.time() + 6
    while not state["hello"] and time.time() < deadline:
        send(ser, "HELLO?")
        time.sleep(0.3)
    if not state["hello"]:
        print("No HELLO from the board, aborting")
        return 1
    send(ser, f"ENCINV,{MOTOR},0")
    send(ser, f"CPR,{MOTOR},186.2")
    send(ser, "GAIN,0.0089,0.071")
    send(ser, "REF,100,0.00125")
    threading.Thread(target=pinger, args=(ser,), daemon=True).start()

    raw = []
    try:
        for name, mode, targets in RUNS:
            with lock:
                state["log"] = []
            send(ser, "LOG,1")
            marks = []
            for target in targets:
                marks.append((time.time(), target))
                send(ser, f"SPEED,{MOTOR},{target},{mode}")
                time.sleep(SEGMENT_S)
            send(ser, "STOP")
            time.sleep(0.3)
            send(ser, "LOG,0")
            time.sleep(0.3)
            with lock:
                log = list(state["log"])
            raw += [(name, *r) for r in log]

            print(f"\n{name}:")
            # Walk the log in order: a step starts when the target changes, and the run
            # ends at the first undriven tick after STOP
            segments, current = [], None
            for r in log:
                if r[1] == 0 and r[2] != 0 and current and len(current) > 10:
                    break
                if current is None or abs(r[2] - current[0][2]) >= 0.05:
                    current = [r]
                    segments.append(current)
                else:
                    current.append(r)
            prev = 0.0
            for seg in segments:
                target = seg[0][2]
                if target == 0:
                    prev = 0.0
                    continue
                rise, ovs, settle, err, ripple = step_metrics(seg, prev, target)
                fmt = lambda v, unit: "  n/a" if v is None else f"{v * 1000:5.0f} {unit}"
                print(f"  {prev:+5.0f} -> {target:+5.0f} RPM:  rise {fmt(rise, 'ms')}   overshoot {ovs:4.1f} %   "
                      f"settle(5%) {fmt(settle, 'ms')}   steady error {err:+6.1f} RPM   ripple ±{ripple:.1f}")
                prev = target
            time.sleep(1.5)
    finally:
        send(ser, "STOP")
        send(ser, "LOG,0")
        time.sleep(0.3)
        running = False
        time.sleep(0.2)
        ser.close()

    with (here / "speed_test_raw.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run", "t_s", "duty", "target_rpm", "speed_rpm", "ref_rpm"])
        w.writerows(raw)
    print("\nsaved speed_test_raw.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
