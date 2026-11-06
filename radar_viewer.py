#!/usr/bin/env python3
"""
Live point-cloud viewer for the AWR1843BOOST, shown in a web browser.

Configures the radar (using radar_test.py), then serves a page with a top-down
view and a rotatable 3D view of every radar frame.

Usage:
  python3 radar_viewer.py                    # uses awr1843_profile.cfg next to this file
  python3 radar_viewer.py --cfg other.cfg --port 8765

Then open http://localhost:8765 in a browser on your laptop. VS Code Remote-SSH
forwards the port automatically (see its PORTS tab). From a plain ssh session,
reconnect with:  ssh -L 8765:localhost:8765 <user>@<jetson-ip>

See README.md in this folder for setup and troubleshooting.
"""
import argparse
import json
import math
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

from radar_test import find_ports, read_frames, send_cfg, send_command

HERE = Path(__file__).resolve().parent


class LatestFrame:
    """Hands the newest radar frame from the reader thread to every open browser tab."""

    def __init__(self):
        self._cond = threading.Condition()
        self._seq = 0
        self._msg = None

    def publish(self, msg):
        with self._cond:
            self._seq += 1
            self._msg = msg
            self._cond.notify_all()

    def wait_newer(self, seq, timeout):
        """Return (seq, msg) once a message newer than `seq` exists; on timeout `seq` is unchanged."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq != seq, timeout)
            return self._seq, self._msg


def read_radar(data, latest):
    """Reader thread: parse frames from the data port and publish them as JSON."""
    try:
        for frame_num, pts in read_frames(data):
            points = [[round(c, 3) for c in p] for p in pts if all(map(math.isfinite, p))]
            latest.publish(json.dumps({'frame': frame_num, 'points': points}))
    except Exception as e:  # e.g. the USB cable was unplugged
        latest.publish(json.dumps({'error': f'Lost the radar: {e}'}))


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            body = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/stream':
            # Server-sent events: one "data:" message per radar frame.
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            seq = 0
            try:
                while True:
                    new_seq, msg = self.server.latest.wait_newer(seq, timeout=1.0)
                    if new_seq != seq:
                        seq = new_seq
                        self.wfile.write(f'data: {msg}\n\n'.encode())
                    else:
                        self.wfile.write(b': keepalive\n\n')  # also notices closed tabs
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass  # keep the terminal quiet


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', default=str(HERE / 'awr1843_profile.cfg'), help='radar config file')
    ap.add_argument('--port', type=int, default=8765, help='web page port (default 8765)')
    ap.add_argument('--host', default='127.0.0.1',
                    help='address to serve on; 0.0.0.0 lets other computers on the network open the page directly')
    args = ap.parse_args()

    # `kill` (SIGTERM) and closing the terminal (SIGHUP) should still stop the radar cleanly.
    # Under nohup, SIGHUP is deliberately ignored so the viewer outlives the terminal; keep that.
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    if signal.getsignal(signal.SIGHUP) != signal.SIG_IGN:
        signal.signal(signal.SIGHUP, interrupt)

    server = ThreadingHTTPServer((args.host, args.port), Handler)  # fails fast if the port is taken
    server.latest = LatestFrame()
    cli_port, data_port = find_ports()
    # exclusive=True refuses to start if another copy of the viewer already has the radar open.
    cli = serial.Serial(cli_port, 115200, timeout=0.1, exclusive=True)
    data = serial.Serial(data_port, 921600, timeout=0.1, exclusive=True)
    try:
        print(f"Configuring radar from {args.cfg}")
        if not send_cfg(cli, args.cfg):
            print("\nSome config lines failed (see [ERR] above); starting the viewer anyway.")
        threading.Thread(target=read_radar, args=(data, server.latest), daemon=True).start()
        print(f"\nViewer running at http://localhost:{args.port}   (Ctrl+C to stop)", flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        send_command(cli, 'sensorStop')
        cli.close()
        data.close()
        print("Radar stopped.")


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AWR1843 live point cloud</title>
<style>
  :root { color-scheme: dark; --bg: #0d1117; --line: #21262d; --muted: #8b949e; --text: #c9d1d9; }
  * { box-sizing: border-box; }
  html, body { height: 100%; margin: 0; }
  body { display: flex; flex-direction: column; background: var(--bg); color: var(--text);
         font: 13px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
  header { display: flex; flex-wrap: wrap; align-items: center; gap: 8px 18px;
           padding: 8px 14px; border-bottom: 1px solid var(--line); }
  h1 { margin: 0; font-size: 15px; font-weight: 600; }
  #status { padding: 2px 10px; border-radius: 999px; background: #30363d; font-variant-numeric: tabular-nums; }
  #status.live { background: #10301c; color: #56d364; }
  #status.wait { background: #3a2b00; color: #e3b341; }
  #status.down { background: #451414; color: #ff7b72; }
  .controls { display: flex; flex-wrap: wrap; align-items: center; gap: 14px; margin-left: auto; }
  .controls label { display: flex; align-items: center; gap: 6px; color: var(--muted); }
  select, button { background: #21262d; color: var(--text); border: 1px solid #30363d;
                   border-radius: 6px; padding: 3px 8px; font: inherit; }
  button { cursor: pointer; }
  input[type=range] { width: 90px; }
  main { flex: 1; min-height: 0; display: grid; grid-template-columns: 1fr 1fr; gap: 1px; background: var(--line); }
  @media (max-width: 800px) { main { grid-template-columns: 1fr; grid-template-rows: 1fr 1fr; } }
  .panel { position: relative; background: var(--bg); min-height: 0; overflow: hidden; }
  .panel h2 { position: absolute; top: 8px; left: 12px; margin: 0; font-size: 12px; font-weight: 500;
              color: var(--muted); pointer-events: none; z-index: 1; }
  canvas { position: absolute; inset: 0; width: 100%; height: 100%; touch-action: none; }
  #view3d { cursor: grab; }
  #view3d:active { cursor: grabbing; }
  footer { display: flex; flex-wrap: wrap; align-items: center; gap: 10px; padding: 6px 14px;
           border-top: 1px solid var(--line); color: var(--muted); }
  #legendBar { width: 200px; height: 10px; border-radius: 3px; }
  .axes { margin-left: auto; }
</style>
</head>
<body>
<header>
  <h1>AWR1843 live point cloud</h1>
  <span id="status">Connecting…</span>
  <div class="controls">
    <label>Color by
      <select id="colorBy">
        <option value="vel">radial velocity</option>
        <option value="z">height</option>
      </select>
    </label>
    <label>Trail <input id="trail" type="range" min="1" max="30" value="5"> <span id="trailVal">5</span> frames</label>
    <label><input id="movingOnly" type="checkbox"> Moving points only</label>
    <button id="reset">Reset view</button>
  </div>
</header>
<main>
  <div class="panel"><h2>Top view · scroll to zoom</h2><canvas id="viewTop"></canvas></div>
  <div class="panel"><h2>3D view · drag to rotate, scroll to zoom</h2><canvas id="view3d"></canvas></div>
</main>
<footer>
  <span id="legendLo"></span><div id="legendBar"></div><span id="legendHi"></span>
  <span class="axes">Radar frame, meters: x right · y forward · z up (z = 0 at radar height)</span>
</footer>
<script>
(() => {
'use strict';
const $ = id => document.getElementById(id);
const topCanvas = $('viewTop'), canvas3d = $('view3d');

const FOV = 60 * Math.PI / 180;   // AWR1843BOOST azimuth field of view, ± from straight ahead
const V_FULL = 1.0;               // |radial velocity| in m/s drawn at full color
const Z_LO = -1.5, Z_HI = 1.5;    // height range in m for the height colors
const MOVING = 0.01;              // |v| above this counts as moving (static points report exactly 0)
// range: meters shown in front of the radar. yaw/pitch/zoom: 3D camera behind and above the radar.
const DEFAULTS = { range: 2, yaw: -0.25, pitch: 0.9, zoom: 2.2 };
const view = { ...DEFAULTS };

const frames = [];   // recent frames, oldest first; each is an array of [x, y, z, v]
let latest = null;   // newest message: {frame, points}
let lastAt = 0;      // performance.now() when the newest frame arrived
let fps = 0;
let connected = false;
let radarError = '';
let dirty = true;

// ---- colors ----------------------------------------------------------------
const STATIC = [175, 184, 193], TOWARD = [56, 139, 253], AWAY = [248, 81, 73];
const VIRIDIS = [[68, 1, 84], [59, 82, 139], [33, 145, 140], [94, 201, 98], [253, 231, 37]];
const mix = (a, b, t) => a.map((v, i) => Math.round(v + (b[i] - v) * t));
const css = (c, a = 1) => `rgba(${c[0]},${c[1]},${c[2]},${a})`;

function pointColor(p) {
  if ($('colorBy').value === 'z') {
    const t = Math.min(1, Math.max(0, (p[2] - Z_LO) / (Z_HI - Z_LO))) * (VIRIDIS.length - 1);
    const i = Math.min(VIRIDIS.length - 2, Math.floor(t));
    return mix(VIRIDIS[i], VIRIDIS[i + 1], t - i);
  }
  const t = Math.max(-1, Math.min(1, p[3] / V_FULL));
  return t < 0 ? mix(STATIC, TOWARD, -t) : mix(STATIC, AWAY, t);
}

function updateLegend() {
  const vel = $('colorBy').value === 'vel';
  const stops = vel ? [TOWARD, STATIC, AWAY] : VIRIDIS;
  $('legendBar').style.background = `linear-gradient(to right, ${stops.map(c => css(c)).join(', ')})`;
  $('legendLo').textContent = vel ? `toward radar −${V_FULL} m/s` : `${Z_LO} m`;
  $('legendHi').textContent = vel ? `+${V_FULL} m/s away (gray = not moving)` : `+${Z_HI} m height`;
}

// ---- shared helpers --------------------------------------------------------
function prepare(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const W = Math.max(1, Math.round(w * dpr)), H = Math.max(1, Math.round(h * dpr));
  if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return [ctx, w, h];
}

// Scale spacing in meters as [labeled step, fine step]. The finest unit is 0.05 m, close to the
// radar's range resolution with awr1843_profile.cfg (about 4.4 cm). Coarser steps take over as you
// zoom out, so the lines never get closer than a few pixels.
const GRID_STEPS = [[0.1, 0.05], [0.25, 0.05], [0.5, 0.1], [1, 0.2], [2, 0.5], [5, 1], [10, 2]];
function gridSteps(pxPerMeter, range) {
  const [major, minor] = GRID_STEPS.find(([m]) => m * pxPerMeter >= 28 && range / m <= 60)
                         || GRID_STEPS[GRID_STEPS.length - 1];
  return { major, minor: minor * pxPerMeter >= 5 && range / minor <= 150 ? minor : null };
}
const fmt = m => `${+m.toFixed(2)} m`;

// Points to draw as [point, age], oldest frames first; age 0 is the newest frame.
function visiblePoints() {
  const out = [], movingOnly = $('movingOnly').checked;
  frames.forEach((pts, i) => {
    const age = frames.length - 1 - i;
    for (const p of pts) if (!movingOnly || Math.abs(p[3]) > MOVING) out.push([p, age]);
  });
  return out;
}
const alphaFor = age => age === 0 ? 1 : 0.08 + 0.6 * (1 - age / frames.length);

// ---- top view --------------------------------------------------------------
function drawTop() {
  const [ctx, w, h] = prepare(topCanvas);
  const R = view.range, pad = 26;
  const s = Math.max(1e-3, Math.min((w / 2 - pad) / R, (h - 2 * pad) / R));   // pixels per meter
  const ox = w / 2, oy = h - pad;
  const { major, minor } = gridSteps(s, R);

  ctx.fillStyle = 'rgba(56,139,253,0.07)';               // field of view
  ctx.beginPath(); ctx.moveTo(ox, oy);
  ctx.arc(ox, oy, R * s, -Math.PI / 2 - FOV, -Math.PI / 2 + FOV);
  ctx.closePath(); ctx.fill();

  const rings = (step, color) => {                        // range rings out to R
    ctx.strokeStyle = color; ctx.beginPath();
    for (let k = 1; k * step <= R + 1e-9; k++) {
      ctx.moveTo(ox - k * step * s, oy); ctx.arc(ox, oy, k * step * s, Math.PI, 2 * Math.PI);
    }
    ctx.stroke();
  };
  ctx.lineWidth = 1;   // translucent gray stays visible on both the background and the blue wedge
  if (minor) rings(minor, 'rgba(139,148,158,0.13)');
  rings(major, 'rgba(139,148,158,0.32)');

  ctx.strokeStyle = '#3d444d';                            // straight-ahead and side-to-side axes
  ctx.beginPath();
  ctx.moveTo(ox - R * s, oy); ctx.lineTo(ox + R * s, oy);
  ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - R * s);
  ctx.stroke();

  ctx.fillStyle = '#6e7681'; ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';    // distance labels up the middle
  for (let k = 1; k * major <= R + 1e-9; k++) ctx.fillText(fmt(k * major), ox + 4, oy - k * major * s - 2);
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';     // left/right labels along the bottom
  const every = major * s >= 44 ? 1 : 2;
  for (let k = -Math.floor(R / major + 1e-9); k * major <= R + 1e-9; k++) {
    if (k !== 0 && k % every === 0) ctx.fillText(`${+(k * major).toFixed(2)}`, ox + k * major * s, oy + 6);
  }

  ctx.fillStyle = '#e6edf3';                              // the radar
  ctx.beginPath(); ctx.moveTo(ox, oy - 8); ctx.lineTo(ox - 6, oy + 4); ctx.lineTo(ox + 6, oy + 4);
  ctx.closePath(); ctx.fill();

  let outside = 0;                                        // newest points that don't fit at this zoom
  for (const [p, age] of visiblePoints()) {
    const x = ox + p[0] * s, y = oy - p[1] * s;
    if (x < 0 || x > w || y < 0 || y > h) { if (age === 0) outside++; continue; }
    ctx.fillStyle = css(pointColor(p), alphaFor(age));
    ctx.beginPath(); ctx.arc(x, y, age ? 2.5 : 3.5, 0, 2 * Math.PI); ctx.fill();
  }

  ctx.textAlign = 'right'; ctx.textBaseline = 'top'; ctx.fillStyle = '#8b949e';
  ctx.fillText(minor ? `rings every ${fmt(minor)}, labeled every ${fmt(major)}` : `rings every ${fmt(major)}`, w - 12, 9);
  if (outside) {
    ctx.fillStyle = '#e3b341';
    ctx.fillText(`${outside} point${outside > 1 ? 's' : ''} outside this view (scroll to zoom out)`, w - 12, 25);
  }
}

// ---- 3D view ---------------------------------------------------------------
function makeCamera(w, h) {
  const R = view.range, d = view.zoom * R, target = [0, 0.55 * R, 0];
  const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
  const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw);
  const eye = [target[0] + d * cp * sy, target[1] - d * cp * cy, target[2] + d * sp];
  const fwd = [-cp * sy, cp * cy, -sp], right = [cy, sy, 0], up = [-sy * sp, cy * sp, cp];
  const focal = 1.2 * Math.min(w, h);   // narrow-ish lens: less stretching of the wide field of view
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const project = p => {
    const v = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
    const depth = dot(v, fwd);
    if (depth < 0.05) return null;                       // behind the camera
    return [w / 2 + focal * dot(v, right) / depth, h / 2 - focal * dot(v, up) / depth, depth];
  };
  return { project, focal };
}

function draw3d() {
  const [ctx, w, h] = prepare(canvas3d);
  const { project, focal } = makeCamera(w, h);
  const R = view.range, camDist = view.zoom * R;
  const { major, minor } = gridSteps(focal / camDist, R);     // scale at the distance the camera looks at
  const G = Math.ceil(R / major - 1e-9) * major;               // grid size: a whole number of labeled steps
  const segment = (a, b) => {
    const pa = project(a), pb = project(b);
    if (pa && pb) { ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); }
  };

  // grid on the radar's horizontal plane (z = 0), in short pieces so it survives close zoom
  const grid = (step, color) => {
    const n = Math.round(G / step);
    ctx.strokeStyle = color; ctx.beginPath();
    for (let i = -n; i <= n; i++) for (let j = 0; j < n; j++) segment([i * step, j * step, 0], [i * step, (j + 1) * step, 0]);
    for (let j = 0; j <= n; j++) for (let i = -n; i < n; i++) segment([i * step, j * step, 0], [(i + 1) * step, j * step, 0]);
    ctx.stroke();
  };
  ctx.lineWidth = 1;
  if (minor) grid(minor, 'rgba(139,148,158,0.11)');
  grid(major, 'rgba(139,148,158,0.3)');

  ctx.strokeStyle = 'rgba(56,139,253,0.45)'; ctx.beginPath();   // field of view edges
  for (const side of [-1, 1]) {
    const ex = side * Math.sin(FOV) * major, ey = Math.cos(FOV) * major;
    for (let k = 0; k < Math.round(G / major); k++) segment([ex * k, ey * k, 0], [ex * (k + 1), ey * (k + 1), 0]);
  }
  ctx.stroke();

  const o = project([0, 0, 0]);                                  // radar axes, one labeled step long
  ctx.lineWidth = 2; ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
  for (const [dir, color, name] of [[[1, 0, 0], '#f85149', 'x'], [[0, 1, 0], '#56d364', 'y'], [[0, 0, 1], '#388bfd', 'z']]) {
    const e = project(dir.map(c => c * major));
    if (!o || !e) continue;
    ctx.strokeStyle = ctx.fillStyle = color;
    ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(e[0], e[1]); ctx.stroke();
    ctx.fillText(name, e[0] + 3, e[1] - 2);
  }
  ctx.lineWidth = 1;

  const pts = [];
  for (const [p, age] of visiblePoints()) {
    const q = project(p);
    if (q) pts.push([q, p, age]);
  }
  pts.sort((a, b) => b[0][2] - a[0][2]);                         // far points first
  for (const [q, p, age] of pts) {
    const c = pointColor(p);
    if (age === 0) {                                             // stem to z = 0 shows height
      const g = project([p[0], p[1], 0]);
      if (g) { ctx.strokeStyle = css(c, 0.3); ctx.beginPath(); ctx.moveTo(g[0], g[1]); ctx.lineTo(q[0], q[1]); ctx.stroke(); }
    }
    ctx.fillStyle = css(c, alphaFor(age));
    // same size as the top view at the camera's focus distance; nearer points look bigger
    const r = (age ? 2.5 : 3.5) * camDist / q[2];
    ctx.beginPath(); ctx.arc(q[0], q[1], Math.max(1.2, Math.min(6, r)), 0, 2 * Math.PI); ctx.fill();
  }

  ctx.textAlign = 'right'; ctx.textBaseline = 'top'; ctx.fillStyle = '#8b949e';
  ctx.fillText(`grid squares ${fmt(minor || major)}`, w - 12, 9);
}

// ---- status ----------------------------------------------------------------
function statusText() {
  const age = (performance.now() - lastAt) / 1000;
  if (!connected) return ['down', 'Not connected to the Jetson. Is radar_viewer.py running?'];
  if (radarError) return ['down', radarError];
  if (!latest) return ['wait', 'Waiting for radar frames…'];
  if (age > 2) return ['wait', `No frames for ${Math.round(age)} s`];
  return ['live', `Live · frame ${latest.frame} · ${latest.points.length} points · ${fps.toFixed(1)} fps`];
}
setInterval(() => {
  const [cls, text] = statusText();
  $('status').className = cls;
  $('status').textContent = text;
}, 250);

// ---- data from the Jetson --------------------------------------------------
function trimFrames() {
  const keep = +$('trail').value;
  if (frames.length > keep) frames.splice(0, frames.length - keep);
}

const source = new EventSource('stream');   // the browser reconnects by itself if this drops
source.onopen = () => { connected = true; radarError = ''; };
source.onerror = () => { connected = false; };
source.onmessage = e => {
  const msg = JSON.parse(e.data);
  if (msg.error) { radarError = msg.error; return; }
  const now = performance.now();
  if (lastAt) {
    const inst = 1000 / Math.max(1, now - lastAt);
    fps = fps ? 0.8 * fps + 0.2 * inst : inst;
  }
  lastAt = now;
  latest = msg;
  frames.push(msg.points);
  trimFrames();
  dirty = true;
};

// ---- mouse and controls ----------------------------------------------------
const zoomFactor = e => Math.exp(e.deltaY * 0.001);
topCanvas.addEventListener('wheel', e => {
  e.preventDefault();
  view.range = Math.min(60, Math.max(0.3, view.range * zoomFactor(e)));
  dirty = true;
}, { passive: false });
canvas3d.addEventListener('wheel', e => {
  e.preventDefault();
  view.zoom = Math.min(6, Math.max(0.3, view.zoom * zoomFactor(e)));
  dirty = true;
}, { passive: false });

let drag = null;
canvas3d.addEventListener('pointerdown', e => { drag = [e.clientX, e.clientY]; canvas3d.setPointerCapture(e.pointerId); });
canvas3d.addEventListener('pointermove', e => {
  if (!drag) return;
  view.yaw -= (e.clientX - drag[0]) * 0.008;
  view.pitch = Math.min(1.55, Math.max(-0.3, view.pitch + (e.clientY - drag[1]) * 0.008));
  drag = [e.clientX, e.clientY];
  dirty = true;
});
for (const type of ['pointerup', 'pointercancel']) canvas3d.addEventListener(type, () => { drag = null; });

$('reset').onclick = () => { Object.assign(view, DEFAULTS); dirty = true; };
$('trail').oninput = () => { $('trailVal').textContent = $('trail').value; trimFrames(); dirty = true; };
$('colorBy').onchange = () => { updateLegend(); dirty = true; };
$('movingOnly').onchange = () => { dirty = true; };
window.addEventListener('resize', () => { dirty = true; });

function render() {
  if (dirty) { dirty = false; drawTop(); draw3d(); }
  requestAnimationFrame(render);
}
updateLegend();
render();
})();
</script>
</body>
</html>
"""

if __name__ == '__main__':
    main()
