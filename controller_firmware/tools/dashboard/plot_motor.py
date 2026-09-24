#!/usr/bin/env python3
"""
plot_motor.py — FOC controller live telemetry dashboard.

Data path:

    firmware serial link --USB CDC-->  THIS --WS--> browser     (real board, --serial COMx)
    foc_sim             --TCP:5599--> THIS --WS--> browser     (simulator, default)
    browser command box --WS--> THIS --serial/TCP--> firmware command parser (app/cmd.c)

The firmware emits BINARY framed telemetry (see app/telem.h):

    0xAA 0x55 | TYPE(u8) | LEN(u16 LE) | PAYLOAD[LEN] | CRC16(u16 LE)

CRC16 = CRC-16/CCITT-FALSE over TYPE+LEN+PAYLOAD (fw_telem_frame.py). The
struct formats below mirror telem.h and must stay in lock-step with it.

Usage:
    python3 tools/dashboard/plot_motor.py --sim          # start foc_sim and attach
    python3 tools/dashboard/plot_motor.py                # attach to a running sim on 127.0.0.1:5599
    python3 tools/dashboard/plot_motor.py --serial COM5  # real board over USB
    then open http://localhost:8988

Type commands in the page footer: calibrate, motor <rpm>, motor torque,
iq <A>, motor off, clear, status, help. In the simulator, lines starting with
"sim" change the simulated world (sim load <Nm>, sim vbus <V>, sim lock on ...).
"""
import argparse
import json
import os
import queue
import socket
import struct
import subprocess
import sys
import threading
import time

import tornado.ioloop
import tornado.web
import tornado.websocket

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fw_telem_frame import crc16 as _crc16  # noqa: E402

# ── telemetry channels (decoded field -> display) ────────────────────────────
FIELDS = [
    "iq", "id", "iq_ref", "id_ref",                 # MOTOR
    "state", "mode", "enabled", "fault",             # MOTOR
    "speed_rpm", "speed_request_rpm", "theta_e",     # SPEED
    "vbus", "vd", "vq", "duty_a", "duty_b", "duty_c",  # VOLT
    "ia", "ib", "ic",                                # PHASE
    "fet_temp_c", "amb_temp_c",                      # TEMP
]

# Chart definitions: (title, [field, ...], [colour, ...], [scale key, ...]).
CHARTS = [
    ("motor current (A)",   ["iq", "id", "iq_ref"],                 ["#e05252", "#1f77b4", "#e05252"]),
    ("motor duty (%)",      ["duty_a", "duty_b", "duty_c"],         ["#1f77b4", "#ff7f0e", "#2ca02c"],
                            ["duty", "duty", "duty"]),
    ("motor speed (rpm)",   ["speed_rpm", "speed_request_rpm"],     ["#17becf", "#e05252"]),
    ("motor state",         ["enabled", "fault"],                   ["#2ca02c", "#d62728"]),
    ("voltages (V)",        ["vbus", "vd", "vq"],                   ["#ff7f0e", "#2ca02c", "#9467bd"]),
    ("temperature (degC)",  ["fet_temp_c", "amb_temp_c"],           ["#f7768e", "#7aa2f7"]),
]

STATES = ["BOOT", "IDLE", "CAL", "RUN", "FAULT"]
FAULTS = ["overcurrent", "overvoltage", "undervoltage", "overtemp", "gate driver",
          "current sum", "calibration", "gate driver init", "overspeed", "not calibrated",
          "adc offset", "encoder", "nan", "stall"]

_state = {f: 0.0 for f in FIELDS}
_state_lock = threading.Lock()
_clients = set()
_cmd_q = queue.Queue()  # browser -> firmware command lines
_log_q = queue.Queue()  # LOG frame text -> browser log panel
_last = {"faults": 0, "state": None}


def _set(field_vals):
    """Merge decoded fields into the latest-value table."""
    with _state_lock:
        _state.update(field_vals)


def _snapshot():
    """Copy of the latest-value table for broadcasting."""
    with _state_lock:
        return dict(_state)


def fault_names(mask):
    """Names of the fault bits set in `mask` (bit order matches app/foc.h)."""
    return [n for i, n in enumerate(FAULTS) if mask & (1 << i)]


# ── binary telemetry protocol (mirrors app/telem.h) ─────────────────────────
_SYNC0 = 0xAA
_SYNC1 = 0x55

_MSG_MOTOR = 0x01
_MSG_SPEED = 0x02
_MSG_VOLT = 0x03
_MSG_PHASE = 0x04
_MSG_TEMP = 0x05
_MSG_LOG = 0x10

_MOTOR_FMT = "<BBHffff"      # state, mode, faults, iq, id, iq_ref, id_ref
_SPEED_FMT = "<fff"          # speed_rpm, speed_ref_rpm, theta_e
_VOLT_FMT = "<ffffff"        # vbus, vd, vq, duty_a, duty_b, duty_c
_PHASE_FMT = "<fff"          # ia, ib, ic
_TEMP_FMT = "<hh"            # fet degC*100, amb degC*100

_MAX_PAYLOAD = 128  # TELEM_MAX_PAYLOAD, telem.h


def _extract_frames(buf):
    """Pull complete, CRC-valid frames off the front of `buf`.

    Returns (frames, remaining_buf) where frames is a list of (type, payload).
    Resyncs on the 0xAA 0x55 pair whenever the current position isn't a valid
    frame start (garbage byte, torn frame from a dropped connection, or a bad
    CRC) — the link is a live byte stream, so this has to tolerate picking up
    mid-frame on connect.
    """
    frames = []
    sync = bytes((_SYNC0, _SYNC1))
    while True:
        idx = buf.find(sync)
        if idx == -1:
            buf = buf[-1:] if buf[-1:] == bytes((_SYNC0,)) else b""
            break
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 5:
            break
        mtype = buf[2]
        (length,) = struct.unpack_from("<H", buf, 3)
        if length > _MAX_PAYLOAD:
            buf = buf[2:]
            continue
        frame_len = 5 + length + 2
        if len(buf) < frame_len:
            break
        payload = buf[5:5 + length]
        (crc_rx,) = struct.unpack_from("<H", buf, 5 + length)
        if _crc16(buf[2:5 + length]) == crc_rx:
            frames.append((mtype, payload))
            buf = buf[frame_len:]
        else:
            buf = buf[2:]
    return frames, buf


def _dispatch(mtype, payload):
    """Decode one frame's payload into a {field: value} dict (or None).

    LOG frames are free text (e.g. `status`'s reply), returned under "__log__".
    """
    try:
        if mtype == _MSG_MOTOR and len(payload) == struct.calcsize(_MOTOR_FMT):
            st, mode, flt, iq, id_, iq_ref, id_ref = struct.unpack(_MOTOR_FMT, payload)
            return {"state": st, "mode": mode, "enabled": 1 if st in (2, 3) else 0,
                    "fault": 1 if flt else 0, "__faults__": flt, "iq": iq, "id": id_,
                    "iq_ref": iq_ref, "id_ref": id_ref}
        if mtype == _MSG_SPEED and len(payload) == struct.calcsize(_SPEED_FMT):
            rpm, ref, th = struct.unpack(_SPEED_FMT, payload)
            return {"speed_rpm": rpm, "speed_request_rpm": ref, "theta_e": th}
        if mtype == _MSG_VOLT and len(payload) == struct.calcsize(_VOLT_FMT):
            vbus, vd, vq, da, db, dc = struct.unpack(_VOLT_FMT, payload)
            return {"vbus": vbus, "vd": vd, "vq": vq, "duty_a": da, "duty_b": db, "duty_c": dc}
        if mtype == _MSG_PHASE and len(payload) == struct.calcsize(_PHASE_FMT):
            ia, ib, ic = struct.unpack(_PHASE_FMT, payload)
            return {"ia": ia, "ib": ib, "ic": ic}
        if mtype == _MSG_TEMP and len(payload) == struct.calcsize(_TEMP_FMT):
            fet, amb = struct.unpack(_TEMP_FMT, payload)
            return {"fet_temp_c": fet / 100.0, "amb_temp_c": amb / 100.0}
        if mtype == _MSG_LOG:
            return {"__log__": payload.decode("ascii", "replace").rstrip("\r\n")}
    except struct.error:
        return None
    return None


def _handle_fields(fields):
    """Route one decoded frame: log text and state/fault changes to the log
    panel, numeric fields to the chart state."""
    log_text = fields.pop("__log__", None)
    if log_text is not None:
        _log_q.put(log_text)
    flt = fields.pop("__faults__", None)
    if flt is not None and flt != _last["faults"]:
        _log_q.put("faults: " + (", ".join(fault_names(flt)) or "cleared"))
        _last["faults"] = flt
    st = fields.get("state")
    if st is not None and st != _last["state"]:
        _log_q.put("state: " + (STATES[st] if st < len(STATES) else str(st)))
        _last["state"] = st
    if fields:
        _set(fields)


def _pump(read, write, stop):
    """Shared RX/TX loop for both transports. read() returns bytes, b"" on EOF, None on timeout."""
    write(b"telem on\n")
    buf = b""
    while not stop.is_set():
        try:
            while True:
                write(_cmd_q.get_nowait().encode() + b"\n")
        except queue.Empty:
            pass
        data = read()
        if data is None:
            continue
        if data == b"":
            return
        buf += data
        frames, buf = _extract_frames(buf)
        for mtype, payload in frames:
            fields = _dispatch(mtype, payload)
            if fields:
                _handle_fields(fields)


def reader_thread(host, port, stop):
    """Connect to foc_sim over TCP, parse frames, update _state, forward commands."""
    while not stop.is_set():
        try:
            sock = socket.create_connection((host, port), timeout=3)
        except OSError as exc:
            print(f"[reader] waiting for simulator {host}:{port} ({exc})")
            time.sleep(1.0)
            continue
        print(f"[reader] connected to {host}:{port}")
        sock.settimeout(0.1)

        def read():
            try:
                return sock.recv(4096)
            except socket.timeout:
                return None

        try:
            _pump(read, sock.sendall, stop)
        except OSError:
            pass
        sock.close()
        print("[reader] connection closed; retrying")
        time.sleep(0.5)


def serial_thread(port, stop):
    """Same as reader_thread but over the board's USB CDC port."""
    import serial  # pyserial
    while not stop.is_set():
        try:
            ser = serial.Serial(port, 921600, timeout=0.1)
        except Exception as exc:
            print(f"[serial] waiting for {port} ({exc})")
            time.sleep(1.0)
            continue
        print(f"[serial] connected to {port}")
        try:
            _pump(lambda: ser.read(4096) or None, ser.write, stop)
        except Exception as exc:
            print(f"[serial] {exc}")
        ser.close()
        time.sleep(0.5)


class IndexHandler(tornado.web.RequestHandler):
    """Serves the dashboard page."""

    def get(self):
        self.set_header("Content-Type", "text/html")
        self.write(self.application.settings["page"])


class WSHandler(tornado.websocket.WebSocketHandler):
    """Browser connection: samples go out, typed commands come in."""

    def check_origin(self, origin):
        return True

    def open(self):
        _clients.add(self)

    def on_close(self):
        _clients.discard(self)

    def on_message(self, message):
        _cmd_q.put(message)


def broadcast():
    """Push one merged sample (t + all fields) to every connected browser."""
    logs = []
    try:
        while True:
            logs.append(_log_q.get_nowait())
    except queue.Empty:
        pass
    if _clients:
        row = _snapshot()
        row["t"] = time.time()
        if logs:
            row["log"] = logs
        msg = json.dumps(row)
        for c in list(_clients):
            try:
                c.write_message(msg)
            except Exception:
                _clients.discard(c)


PAGE_TMPL = """<!doctype html><html><head><meta charset="utf-8">
<title>foc telemetry</title>
<link rel="stylesheet" href="https://unpkg.com/uplot@1.6.31/dist/uPlot.min.css">
<script src="https://unpkg.com/uplot@1.6.31/dist/uPlot.iife.min.js"></script>
<style>
  html,body{margin:0;height:100%;overflow:hidden;background:#111;color:#ddd;font:13px system-ui,sans-serif}
  body{display:flex;flex-direction:column}
  #hdr{flex:0 0 auto;padding:5px 10px;background:#1b1b1b;display:flex;gap:14px;align-items:center}
  #status{margin-left:auto}
  #charts{flex:1 1 auto;min-height:0;display:grid;grid-template-columns:1fr 1fr;grid-template-rows:repeat(3,1fr);gap:5px;padding:5px}
  .chart{background:#181818;border:1px solid #2a2a2a;border-radius:5px;padding:3px;display:flex;flex-direction:column;min-height:0;overflow:hidden}
  .ttl{flex:0 0 auto;font-size:12px;color:#ccc;padding:1px 4px}
  .body{flex:1 1 auto;min-height:0}
  .u-legend{color:#ccc;font-size:10px}
  #foot{flex:0 0 auto;display:flex;gap:6px;padding:5px 10px;background:#1b1b1b}
  #cmd{flex:1;background:#0d0d0d;color:#eee;border:1px solid #333;padding:6px;font-family:monospace}
  #log{flex:0 0 44px;overflow:auto;padding:2px 10px;font-family:monospace;font-size:11px;color:#7a7;white-space:pre-wrap}
  button{background:#2a4;color:#fff;border:0;padding:6px 14px;border-radius:4px;cursor:pointer}
</style></head><body>
<div id="hdr"><b>foc telemetry</b><span id="rate">0 sps</span>
  <span id="status">&#x1f534; disconnected</span></div>
<div id="charts"></div>
<div id="log"></div>
<div id="foot">
  <input id="cmd" placeholder="command (e.g. calibrate | motor 3000 | motor torque | iq 2 | motor off | sim load 0.05 | status | help)" autofocus>
  <button onclick="sendCmd()">Send</button></div>
<script>
const CHARTS = __CHARTS__;
const FIELDS = __FIELDS__;
const WINDOW = __WINDOW__;
const data = { t: [] };
FIELDS.forEach(f => data[f] = []);
const charts = [];
const chartsEl = document.getElementById("charts");

const AXIS_COLOR = "#aaa";
const GRID_COLOR = "#333";
const LEGEND_H = 18;

function mkChart(def) {
  const wrap = document.createElement("div"); wrap.className = "chart";
  const ttl = document.createElement("div"); ttl.className = "ttl"; ttl.textContent = def.l;
  const body = document.createElement("div"); body.className = "body";
  wrap.appendChild(ttl); wrap.appendChild(body); chartsEl.appendChild(wrap);

  const scaleKeys = def.s || def.y.map(() => "y");
  const uniqScales = [...new Set(scaleKeys)];

  const series = [{}];
  def.y.forEach((k, i) => series.push({
    label: k, stroke: def.c[i], width: 1.5, spanGaps: true, scale: scaleKeys[i],
    dash: (k === "speed_request_rpm" || k.endsWith("_ref")) ? [4, 4] : undefined,
  }));

  const scales = { x: { time: true } };
  uniqScales.forEach(s => {
    scales[s] = (s === "duty") ? { auto: false, min: 0, max: 100 } : {};
  });

  const axes = [{ scale: "x", stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR } }];
  uniqScales.forEach((s, i) => axes.push({
    scale: s, stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, side: i === 0 ? 3 : 1,
  }));

  const opts = {
    width: Math.max(100, body.clientWidth),
    height: Math.max(40, body.clientHeight - LEGEND_H),
    series, scales, axes,
    cursor: { sync: { key: "foc" } },
    legend: { live: true } };
  const u = new uPlot(opts, [[], ...def.y.map(() => [])], body);
  u.root.querySelector(".u-legend")?.classList.add("u-inline");
  charts.push({ u, def, body });
}
CHARTS.forEach(mkChart);
window.addEventListener("resize", () =>
  charts.forEach(c => c.u.setSize({
    width: Math.max(100, c.body.clientWidth),
    height: Math.max(40, c.body.clientHeight - LEGEND_H),
  })));

let nsamp = 0;
setInterval(() => { document.getElementById("rate").textContent = nsamp + " sps"; nsamp = 0; }, 1000);

function redraw() {
  const t0 = data.t.length ? data.t[data.t.length - 1] - WINDOW : 0;
  while (data.t.length && data.t[0] < t0) {
    data.t.shift(); FIELDS.forEach(f => data[f].shift());
  }
  charts.forEach(c => {
    c.u.setData([data.t, ...c.def.y.map(k => data[k])]);
    if ((c.def.s || []).includes("duty")) {
      c.u.setScale("duty", { min: 0, max: 100 });
    }
  });
}

// Wire values are SI units from app/telem.h; duties arrive as 0..1.
const CONVERT = {
  duty_a: (v) => v * 100,
  duty_b: (v) => v * 100,
  duty_c: (v) => v * 100,
};

const ws = new WebSocket("ws://" + location.host + "/ws");
ws.onopen = () => document.getElementById("status").innerHTML = "&#x1f7e2; connected";
ws.onclose = () => document.getElementById("status").innerHTML = "&#x1f534; disconnected";
ws.onmessage = (ev) => {
  const r = JSON.parse(ev.data);
  data.t.push(r.t);
  FIELDS.forEach(f => {
    let v = r[f] ?? null;
    if (v !== null && CONVERT[f]) v = CONVERT[f](v);
    data[f].push(v);
  });
  nsamp++;
  if (r.log) { r.log.forEach(log); }
};
setInterval(redraw, 50);

function log(s) { const l = document.getElementById("log"); l.textContent += s + "\\n"; l.scrollTop = l.scrollHeight; }
function sendCmd() {
  const c = document.getElementById("cmd"); const v = c.value.trim();
  if (v) { ws.send(v); log("> " + v); c.value = ""; }
}
document.getElementById("cmd").addEventListener("keydown", e => { if (e.key === "Enter") sendCmd(); });
</script></body></html>"""


def render_page(window):
    """Fill the chart definitions into the page template."""
    charts_json = [
        {"l": chart[0], "y": chart[1], "c": chart[2],
         "s": chart[3] if len(chart) > 3 else None}
        for chart in CHARTS
    ]
    return (PAGE_TMPL
            .replace("__CHARTS__", json.dumps(charts_json))
            .replace("__FIELDS__", json.dumps(FIELDS))
            .replace("__WINDOW__", str(window)))


def find_sim():
    """Locate a built foc_sim executable in controller_firmware/build-sim."""
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         os.pardir, os.pardir))
    for name in ("foc_sim.exe", "foc_sim"):
        for sub in ("build-sim", os.path.join("build-sim", "Release"), os.path.join("build-sim", "Debug")):
            p = os.path.join(root, "controller_firmware", sub, name)
            if os.path.isfile(p):
                return p
            p = os.path.join(root, sub, name)
            if os.path.isfile(p):
                return p
    return None


def main():
    """Parse arguments, start the link thread (and simulator) and serve the page."""
    ap = argparse.ArgumentParser(description="FOC controller telemetry dashboard")
    ap.add_argument("--sim", nargs="?", const="auto", default=None,
                    help="start the simulator (optionally give the foc_sim path) and attach to it")
    ap.add_argument("--serial", help="attach to the board's USB CDC port instead (e.g. COM5, /dev/ttyACM0)")
    ap.add_argument("--bridge-host", default="127.0.0.1")
    ap.add_argument("--bridge-port", type=int, default=5599, help="foc_sim TCP port")
    ap.add_argument("--http", type=int, default=8988, help="dashboard HTTP port")
    ap.add_argument("--window", type=float, default=4.0, help="x-axis span (s)")
    args = ap.parse_args()

    sim_proc = None
    if args.sim:
        path = find_sim() if args.sim == "auto" else args.sim
        if not path:
            sys.exit("foc_sim not found: build it with `cmake --preset sim && cmake --build build-sim`")
        sim_proc = subprocess.Popen([path, "--port", str(args.bridge_port)])
        print(f"started {path}")

    stop = threading.Event()
    if args.serial:
        target, targs = serial_thread, (args.serial, stop)
    else:
        target, targs = reader_thread, (args.bridge_host, args.bridge_port, stop)
    threading.Thread(target=target, args=targs, daemon=True).start()

    app = tornado.web.Application(
        [(r"/", IndexHandler), (r"/ws", WSHandler)],
        page=render_page(args.window))
    app.listen(args.http)
    print(f"dashboard: open http://localhost:{args.http}")
    tornado.ioloop.PeriodicCallback(broadcast, 50).start()  # 20 Hz
    try:
        tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        stop.set()
    finally:
        if sim_proc:
            sim_proc.terminate()


if __name__ == "__main__":
    main()
