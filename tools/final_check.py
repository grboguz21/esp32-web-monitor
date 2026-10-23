"""Final check for motor 1 with the default (aggressive) gains, firmware 0.18.0:
target steps, low speeds from rest, a 0 -> 600 RPM ramp and distance moves.
Raw 200 Hz ticks go to final_check_raw.csv next to this script.
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
CPR = 186.2
KP, KI = 0.0417, 0.333
TAU_REF_MS, KA = 100, 0.00125
STEPS = [300, 600, 300, 150, -400, 0]
STEP_S = 2.0
LOW_SPEEDS = [50, 80, 120, -50, -80, -120]
LOW_S = 4.0
RAMP_TO, RAMP_S = 600, 2.0
MOVES = [(5.0, 300, 1500), (-5.0, 300, 1500), (1.0, 300, 1500), (0.25, 200, 1500), (5.0, 600, 3000)]

lock = threading.Lock()
state = {"hello": False, "log": [], "move": None}
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
            p = text.split(",")
            if p[0] == "HELLO":
                state["hello"] = True
            elif p[0] == "L" and len(p) == 8:   # L,t,duty,c1,c2,target,ref,speed
                try:
                    row = (int(p[1]) / 1e6, float(p[2]), int(p[3]), float(p[5]), float(p[6]), float(p[7]))
                except ValueError:
                    continue
                with lock:
                    state["log"].append(row)
            elif p[0] == "MOVE" and len(p) == 6:
                try:
                    state["move"] = (int(p[1]), int(p[2]), int(p[3]), float(p[4]), int(p[5]))
                except ValueError:
                    pass


def pinger(ser):
    while running:
        try:
            ser.write(b"PING\n")
        except serial.SerialException:
            return
        time.sleep(0.2)


def send(ser, text):
    ser.write((text + "\n").encode())


def record(ser, action):
    """Log while action(ser) runs; returns rows (t, duty, count, target, ref, speed)."""
    with lock:
        state["log"] = []
    send(ser, "LOG,1")
    time.sleep(0.1)
    action(ser)
    send(ser, "LOG,0")
    time.sleep(0.3)
    with lock:
        return list(state["log"])


def smooth(values, n=10):
    out, acc = [], 0.0
    for i, v in enumerate(values):
        acc += v
        if i >= n:
            acc -= values[i - n]
        out.append(acc / min(i + 1, n))
    return out


def step_report(log, targets):
    # sequential segmentation on the commanded target
    i, prev = 0, 0.0
    for target in targets:
        while i < len(log) and abs(log[i][3] - target) > 0.05:
            i += 1
        start = i
        while i < len(log) and abs(log[i][3] - target) <= 0.05:
            i += 1
        seg = log[start:i]
        if len(seg) < 50:
            print(f"  {prev:+5.0f} -> {target:+5.0f}: no data")
            prev = target
            continue
        t0 = seg[0][0]
        t = [r[0] - t0 for r in seg]
        w = [r[5] for r in seg]
        ws = smooth(w)
        step = target - prev
        peak = max(w) if step > 0 else min(w)
        ovs = max(0.0, (peak - target) / step * 100) if abs(step) > 1 else 0.0
        band = max(0.05 * abs(step), 10)
        settle = 0.0
        for k in range(len(ws) - 1, -1, -1):
            if abs(ws[k] - target) > band:
                settle = t[k + 1] if k + 1 < len(t) else float("nan")
                break
        tail = [wi for ti, wi in zip(t, w) if ti > t[-1] - 0.5]
        err = target - statistics.fmean(tail)
        print(f"  {prev:+5.0f} -> {target:+5.0f} RPM:  overshoot {ovs:4.1f} %   settle {settle * 1000:5.0f} ms   "
              f"steady error {err:+5.1f} RPM   ripple ±{statistics.pstdev(tail):.1f}")
        prev = target


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
    send(ser, f"CPR,{MOTOR},{CPR}")
    send(ser, f"GAIN,{KP},{KI}")
    send(ser, f"REF,{TAU_REF_MS},{KA}")
    threading.Thread(target=pinger, args=(ser,), daemon=True).start()
    raw = []

    try:
        # 1. steps
        def steps(s):
            for target in STEPS:
                send(s, f"SPEED,{MOTOR},{target},2")
                time.sleep(STEP_S)
            send(s, "STOP")
        log = record(ser, steps)
        raw += [("steps", *r) for r in log]
        print("\nSteps (FF + PI):")
        step_report(log, STEPS)
        time.sleep(1.0)

        # 2. low speeds, each from rest
        print("\nLow speed (from rest, last 3 s):")
        for target in LOW_SPEEDS:
            def low(s, target=target):
                send(s, f"SPEED,{MOTOR},{target},2")
                time.sleep(LOW_S)
                send(s, "STOP")
            log = record(ser, low)
            raw += [(f"low{target}", *r) for r in log]
            t0 = log[0][0] if log else 0
            tail = [r[5] for r in log if r[0] - t0 > LOW_S - 3.0 and abs(r[3] - target) < 0.05]
            ws = smooth(tail, 20)   # 100 ms average
            if tail:
                stalls = sum(1 for v in ws if v * target < 0.3 * target * target)
                print(f"  {target:+4d} RPM:  mean {statistics.fmean(tail):+6.1f}   ±{statistics.pstdev(tail):4.1f}   "
                      f"100 ms avg range {min(ws):+5.0f}..{max(ws):+5.0f}   below 30 % of target {stalls / len(ws) * 100:4.1f} % of the time")
            time.sleep(0.8)

        # 3. ramp
        def ramp(s):
            t_start = time.time()
            while True:
                el = time.time() - t_start
                target = min(RAMP_TO, RAMP_TO * el / RAMP_S)
                send(s, f"SPEED,{MOTOR},{target:.0f},2")
                if el >= RAMP_S:
                    break
                time.sleep(0.02)
            time.sleep(1.0)
            send(s, "STOP")
        log = record(ser, ramp)
        raw += [("ramp", *r) for r in log]
        mid = [r for r in log if 150 < r[3] < RAMP_TO - 50]
        if mid:
            lag_cmd = [r[3] - r[5] for r in mid]
            lag_ref = [r[4] - r[5] for r in mid]
            print(f"\nRamp 0 -> {RAMP_TO} RPM in {RAMP_S:.0f} s:  speed behind command {statistics.fmean(lag_cmd):+.1f} RPM "
                  f"(max {max(lag_cmd):+.0f}),  behind reference {statistics.fmean(lag_ref):+.1f} ±{statistics.pstdev(lag_ref):.1f} RPM")
            print(f"  (the reference model alone lags by rate x tau_ref = {RAMP_TO / RAMP_S * TAU_REF_MS / 1000:.0f} RPM)")
        time.sleep(1.0)

        # 4. distance moves
        print("\nMoves (final error after braking):")
        for turns, vmax, accel in MOVES:
            counts = round(turns * CPR)
            def move(s):
                state["move"] = None
                send(s, f"MOVE,{MOTOR},{counts},{vmax},{accel}")
                t_end = time.time() + 15
                time.sleep(0.2)
                while time.time() < t_end:
                    m = state["move"]
                    if m and m[0] in (3, 4) and m[1] == counts:
                        break
                    time.sleep(0.02)
                time.sleep(0.5)   # does it creep after the brake lets go?
            t0 = time.time()
            log = record(ser, move)
            raw += [(f"move{turns}", *r) for r in log]
            m = state["move"]
            if not m or m[1] != counts:
                print(f"  {turns:+5.2f} turns: no MOVE report")
                continue
            phase, target, pos, _, final_err = m
            # position relative to the start of the move, from the log's counts
            c0 = log[0][2] if log else 0
            peak = max((r[2] - c0) * (1 if counts > 0 else -1) for r in log) if log else 0
            over = peak - abs(counts)
            moving = [r for r in log if abs(r[3]) > 0.5]
            dur = moving[-1][0] - moving[0][0] if moving else float("nan")
            err_now = target - pos
            print(f"  {turns:+5.2f} turns ({counts:+5d} counts) at {vmax} RPM, {accel} RPM/s:  "
                  f"{'done' if phase == 3 else 'phase ' + str(phase)} in {dur:.2f} s   "
                  f"error at stop {final_err:+d} counts ({final_err / CPR * 360:+.0f}°)   "
                  f"0.5 s later {err_now:+d}   overshoot {max(0, over)} counts")
            send(ser, "STOP")
            time.sleep(0.8)
    finally:
        send(ser, "STOP")
        send(ser, "LOG,0")
        time.sleep(0.3)
        running = False
        time.sleep(0.2)
        ser.close()

    with (here / "final_check_raw.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["test", "t_s", "duty", "count", "target_rpm", "ref_rpm", "speed_rpm"])
        w.writerows(raw)
    print("\nsaved final_check_raw.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
