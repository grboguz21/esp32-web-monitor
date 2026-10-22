"""Step tests for motor 1 with the firmware's 200 Hz log (firmware 0.13.0 or later).

For each step the motor settles at duty A, logging starts, the duty jumps to B and the
response is recorded. A first-order-plus-dead-time model is fitted to each response:
    w(t) = w0 + (w1 - w0) * (1 - exp(-(t - theta) / tau))   for t > theta
Raw ticks go to step_test_raw.csv, results to step_test_results.csv.
"""
import csv
import math
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
SUPPLY_V = 12.0
STEPS = [(30, 60), (60, 30), (20, 40)]
REPEATS = 2
SETTLE_S = 2.5
PRE_S = 0.3
POST_S = 1.5
DIFF_HALF = 2   # speed by central difference over +-2 ticks (20 ms)

lock = threading.Lock()
state = {"hello": False, "log": []}
running = True


def reader(ser):
    buf = b""
    while running:
        try:
            buf += ser.read(512)
        except (serial.SerialException, TypeError):
            return
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("ascii", "ignore").strip()
            if text.startswith("HELLO,"):
                state["hello"] = True
            elif text.startswith("L,"):
                p = text.split(",")
                if len(p) >= 5:   # L,t,duty,count1,count2[,target,ref,speed]
                    with lock:
                        state["log"].append((int(p[1]), float(p[2]), int(p[3]), int(p[4])))


def pinger(ser):
    while running:
        try:
            ser.write(b"PING\n")
        except serial.SerialException:
            return
        time.sleep(0.2)


def send(ser, text):
    ser.write((text + "\n").encode())


def record_step(ser, a, b):
    send(ser, f"RUN,{MOTOR},{a},0")
    time.sleep(SETTLE_S)
    with lock:
        state["log"] = []
    send(ser, "LOG,1")
    time.sleep(PRE_S)
    send(ser, f"RUN,{MOTOR},{b},0")
    time.sleep(POST_S)
    send(ser, "LOG,0")
    time.sleep(0.3)   # let the queue drain
    with lock:
        return list(state["log"])


def speeds(log):
    """(t_s, rpm, duty) per tick, speed by central difference."""
    out = []
    idx = 1 + (MOTOR - 1)
    col = 2 + (MOTOR - 1)
    for i in range(DIFF_HALF, len(log) - DIFF_HALF):
        t0, _, *c0 = log[i - DIFF_HALF]
        t1, _, *c1 = log[i + DIFF_HALF]
        dt = (t1 - t0) / 1e6
        if dt <= 0:
            continue
        cps = (c1[MOTOR - 1] - c0[MOTOR - 1]) / dt
        out.append((log[i][0] / 1e6, cps * 60 / CPR, log[i][1]))
    return out


def fit_fopdt(t, w, w0, w1):
    """Grid search for theta and tau (seconds) minimising the squared error."""
    best = (float("inf"), 0, 0)
    for theta_ms in range(0, 81):
        theta = theta_ms / 1000
        for tau_ms in range(5, 401):
            tau = tau_ms / 1000
            err = 0.0
            for ti, wi in zip(t, w):
                model = w0 if ti <= theta else w0 + (w1 - w0) * (1 - math.exp(-(ti - theta) / tau))
                err += (wi - model) ** 2
            if err < best[0]:
                best = (err, theta, tau)
    rms = math.sqrt(best[0] / len(t))
    return best[1], best[2], rms


def analyse(log, a, b):
    sp = speeds(log)
    step_i = next(i for i, (_, _, d) in enumerate(sp) if abs(d - b) < 0.05)
    t_step = sp[step_i][0]
    pre = [r for _, r, _ in sp[:step_i]][:-DIFF_HALF]
    w0 = statistics.fmean(pre)
    tail = [r for t, r, _ in sp if t > sp[-1][0] - 0.5]
    w1 = statistics.fmean(tail)
    t = [ti - t_step for ti, _, _ in sp[step_i:]]
    w = [r for _, r, _ in sp[step_i:]]
    theta, tau, rms = fit_fopdt(t, w, w0, w1)
    dv = (b - a) / 100 * SUPPLY_V
    k_rpm_per_v = (w1 - w0) / dv
    return {"w0": w0, "w1": w1, "theta": theta, "tau": tau, "rms": rms, "k": k_rpm_per_v}


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
    threading.Thread(target=pinger, args=(ser,), daemon=True).start()

    results, raw, intervals = [], [], []
    try:
        for direction in (1, -1):
            for a, b in STEPS:
                for rep in range(REPEATS):
                    sa, sb = a * direction, b * direction
                    log = record_step(ser, sa, sb)
                    intervals += [(log[i][0] - log[i - 1][0]) for i in range(1, len(log))]
                    for row in log:
                        raw.append((sa, sb, rep, *row))
                    r = analyse(log, sa, sb)
                    results.append((sa, sb, rep, r))
                    print(f"{sa:+4d}% -> {sb:+4d}%  #{rep + 1}:  {r['w0']:+7.1f} -> {r['w1']:+7.1f} RPM   "
                          f"theta {r['theta'] * 1000:4.0f} ms   tau {r['tau'] * 1000:4.0f} ms   "
                          f"K {r['k']:6.1f} RPM/V   fit rms {r['rms']:.1f} RPM   ticks {len(log)}",
                          flush=True)
            send(ser, "STOP")
            time.sleep(1.5)
    finally:
        send(ser, "STOP")
        time.sleep(0.3)
        running = False
        time.sleep(0.2)
        ser.close()

    with (here / "step_test_raw.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["from_pct", "to_pct", "repeat", "t_us", "duty", "count1", "count2"])
        w.writerows(raw)
    with (here / "step_test_results.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["from_pct", "to_pct", "repeat", "w0_rpm", "w1_rpm", "theta_ms", "tau_ms", "k_rpm_per_v", "rms_rpm"])
        for sa, sb, rep, r in results:
            w.writerow([sa, sb, rep, f"{r['w0']:.1f}", f"{r['w1']:.1f}", f"{r['theta'] * 1000:.0f}",
                        f"{r['tau'] * 1000:.0f}", f"{r['k']:.1f}", f"{r['rms']:.1f}"])

    if intervals:
        late = sum(1 for d in intervals if d > 7500)
        print(f"tick interval: mean {statistics.fmean(intervals):.0f} us, min {min(intervals)} us, "
              f"max {max(intervals)} us, gaps > 7.5 ms: {late} of {len(intervals)}")
    print("saved step_test_raw.csv and step_test_results.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
