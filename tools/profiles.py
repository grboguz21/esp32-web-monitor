"""Back up or restore the motor profiles stored on the board.

    python tools/profiles.py backup [file]    read both profiles, write JSON (default docs/data/profiles.json)
    python tools/profiles.py restore [file]   send them back and save them to the board's flash

Disconnect the dashboard first; it holds the port.
"""
import json
import sys
import time
from pathlib import Path

import serial

PORT = "/dev/ttyUSB0"
BAUD = 460800
MOTORS = (1, 2)
DEFAULT_FILE = Path(__file__).resolve().parent.parent / "docs/data/profiles.json"


def open_board():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)   # opening the port can reset the board
    ser.reset_input_buffer()
    return ser


def talk(ser, commands, wait=1.0):
    for c in commands:
        ser.write((c + "\n").encode())
        time.sleep(0.05)
    end = time.time() + wait
    buf = b""
    while time.time() < end:
        buf += ser.read(4096)
    return buf.decode("ascii", "ignore").splitlines()


def parse_ff(text):
    return [[float(r), float(d)] for r, d in (p.split(":") for p in text.split(";"))]


def backup(path):
    ser = open_board()
    lines = talk(ser, ["HELLO?"] + [f"PROFILE?,{m}" for m in MOTORS], wait=1.5)
    ser.close()
    version = next((l.split(",")[2] for l in lines if l.startswith("HELLO,")), "?")
    profiles = {}
    for line in lines:
        p = line.strip().split(",")
        if p[0] == "PROFILE" and len(p) == 11:
            m = p[1]
            profiles.setdefault(m, {}).update({
                "cpr": float(p[2]), "invert": p[3] == "1", "kick_plus": float(p[4]), "kick_minus": float(p[5]),
                "kp": float(p[6]), "ki": float(p[7]), "tau_ref_ms": float(p[8]), "ka": float(p[9]),
                "max_rpm": float(p[10]),
            })
        elif p[0] == "FF" and len(p) == 4:
            profiles.setdefault(p[1], {})["ff_plus" if p[2] == "+" else "ff_minus"] = parse_ff(p[3])
    missing = [m for m in map(str, MOTORS) if "cpr" not in profiles.get(m, {})]
    if missing:
        sys.exit(f"no profile from motor(s) {', '.join(missing)}; is the dashboard still connected?")
    data = {"firmware": version, "saved": time.strftime("%Y-%m-%d %H:%M"), "profiles": profiles}
    path.write_text(json.dumps(data, indent=2) + "\n")
    print(f"saved {path}")
    for m, p in profiles.items():
        print(f"  motor {m}: CPR {p['cpr']}, invert {p['invert']}, start {p['kick_plus']}/{p['kick_minus']} %, "
              f"max {p['max_rpm']:.0f} RPM, Kp {p['kp']}, Ki {p['ki']}, kA {p['ka']}, "
              f"curve {len(p['ff_plus'])}+{len(p['ff_minus'])} points")


def restore(path):
    data = json.loads(path.read_text())
    ser = open_board()
    commands = []
    for m, p in data["profiles"].items():
        ff = lambda rows: ";".join(f"{r:.1f}:{d:.1f}" for r, d in rows)
        commands += [
            f"CPR,{m},{p['cpr']}", f"ENCINV,{m},{1 if p['invert'] else 0}",
            f"KICK,{m},{p['kick_plus']},{p['kick_minus']}", f"MAXRPM,{m},{p['max_rpm']}",
            f"FF,{m},+,{ff(p['ff_plus'])}", f"FF,{m},-,{ff(p['ff_minus'])}",
            f"GAIN,{p['kp']},{p['ki']},{m}", f"REF,{p['tau_ref_ms']},{p['ka']},{m}", f"SAVE,{m}",
        ]
    lines = talk(ser, commands, wait=2.0)
    ser.close()
    saved = [l for l in lines if l.startswith("SAVED,")]
    print("\n".join(saved) if saved else "no SAVED reply; check the board")


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("backup", "restore"):
        sys.exit(__doc__)
    target = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_FILE
    backup(target) if sys.argv[1] == "backup" else restore(target)
