"""Build and upload the firmware from the dashboard.

    python3 tools/build_server.py

Runs `pio run -e <env> -t upload` when the dashboard asks and streams the progress back
(compile, then upload) as one JSON object per line. Listens on 127.0.0.1 only and answers
only the dashboard opened from disk (Origin "null") or from localhost, so other web pages
cannot start an upload.

    POST /build   {"target": "ota" | "usb"}  -> progress lines, the last one has "done"
    GET  /status                             -> {"busy": bool}
"""
import json
import re
import shutil
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HOST = "127.0.0.1"
PORT = 8765
ROOT = Path(__file__).resolve().parent.parent
ENVS = {"ota": "ota", "usb": "nodemcu-32s"}
ALLOWED_ORIGINS = {"null", "http://localhost", "http://127.0.0.1"}
# Files compiled by the last full build, per env: the compile bar's 100 %
COUNTS_FILE = ROOT / ".pio" / "build_server_counts.json"
DEFAULT_COUNT = 120

COMPILING = re.compile(r"^Compiling \S+")
LINKING = re.compile(r"^(Linking \S+|Building \.pio/)")
UPLOAD_START = re.compile(r"^(Configuring upload protocol|Uploading \.pio|Looking for upload port|Auto-detected)")
ESPOTA = re.compile(r"Uploading:\s*\[[= ]*\]\s*(\d+)%")
# esptool 5: "Writing at 0x0001c0c3 <bar>  2.2% 16.00kB/719.06kB"; older: "(2 %)"
ESPTOOL = re.compile(r"Writing at 0x[0-9a-f]+\S*\s.*?\(?(\d+)(?:\.\d+)?\s*%")
# esptool writes the bootloader and partition table first; only the app counts
FIRMWARE_WRITE = re.compile(r"^Writing '.*firmware\.bin'")

busy = threading.Lock()


def pio_path():
    return shutil.which("pio") or str(Path.home() / ".platformio/penv/bin/pio")


def load_counts():
    try:
        return json.loads(COUNTS_FILE.read_text())
    except (OSError, ValueError):
        return {}


def save_count(env, n):
    counts = load_counts()
    counts[env] = n
    try:
        COUNTS_FILE.parent.mkdir(exist_ok=True)
        COUNTS_FILE.write_text(json.dumps(counts))
    except OSError:
        pass


def lines_of(stream):
    """Output lines, split on \\r too: espota redraws its progress bar in place."""
    buf = b""
    while True:
        chunk = stream.read1(4096)
        if not chunk:
            break
        buf += chunk
        parts = re.split(rb"[\r\n]", buf)
        buf = parts.pop()
        for p in parts:
            if p.strip():
                yield p.decode(errors="replace").rstrip()
    if buf.strip():
        yield buf.decode(errors="replace").rstrip()


def run_build(target, emit):
    env = ENVS[target]
    expected = load_counts().get(env, DEFAULT_COUNT)
    compiled = 0
    compile_pct = 0
    upload_pct = 0
    phase = "compile"
    app_seen = False
    emit({"phase": phase, "compile": 0, "upload": 0, "line": f"pio run -e {env} -t upload"})
    proc = subprocess.Popen(
        [pio_path(), "run", "-e", env, "-t", "upload"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    tail = []
    for line in lines_of(proc.stdout):
        tail = (tail + [line])[-30:]
        if phase == "compile":
            if COMPILING.match(line):
                compiled += 1
                compile_pct = max(compile_pct, min(90, round(compiled / max(expected, compiled) * 90)))
            elif LINKING.match(line):
                compile_pct = max(compile_pct, 95)
            elif UPLOAD_START.match(line):
                phase = "upload"
                compile_pct = 100
        if FIRMWARE_WRITE.match(line):
            app_seen = True
        m = ESPOTA.search(line) or (ESPTOOL.search(line) if app_seen else None)
        if m:
            phase = "upload"
            compile_pct = 100
            upload_pct = int(m.group(1))
        emit({"phase": phase, "compile": compile_pct, "upload": upload_pct, "line": line})
    code = proc.wait()
    ok = code == 0
    if ok and compiled > expected * 0.8:   # a full build: remember its size
        save_count(env, compiled)
    emit({"done": True, "ok": ok, "compile": 100 if ok else compile_pct,
          "upload": 100 if ok else upload_pct, "tail": [] if ok else tail[-12:]})


class Handler(BaseHTTPRequestHandler):
    def origin_ok(self):
        return self.headers.get("Origin") in ALLOWED_ORIGINS

    def cors(self):
        self.send_header("Access-Control-Allow-Origin", self.headers.get("Origin"))
        self.send_header("Access-Control-Allow-Methods", "GET, POST")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Build")

    def do_OPTIONS(self):
        if not self.origin_ok():
            self.send_error(403)
            return
        self.send_response(204)
        self.cors()
        self.end_headers()

    def do_GET(self):
        if self.path != "/status" or not self.origin_ok():
            self.send_error(404)
            return
        body = json.dumps({"busy": busy.locked()}).encode()
        self.send_response(200)
        self.cors()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        # The custom header forces a CORS preflight, which only allowed origins pass
        if self.path != "/build" or not self.origin_ok() or self.headers.get("X-Build") != "1":
            self.send_error(403)
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            target = json.loads(self.rfile.read(length) or b"{}").get("target")
        except ValueError:
            target = None
        if target not in ENVS:
            self.send_error(400)
            return
        if not busy.acquire(blocking=False):
            self.send_error(409, "a build is already running")
            return
        try:
            self.send_response(200)
            self.cors()
            self.send_header("Content-Type", "application/x-ndjson")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()

            def emit(obj):
                try:
                    self.wfile.write((json.dumps(obj) + "\n").encode())
                    self.wfile.flush()
                except OSError:
                    pass   # page closed; let the build finish anyway

            run_build(target, emit)
        finally:
            busy.release()

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    print(f"Build helper on http://{HOST}:{PORT}  (project: {ROOT})  Ctrl+C to stop")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
