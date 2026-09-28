#!/usr/bin/env python3
# =============================================================================
#  AEB CONTROL DASHBOARD  —  Team PRIORITY ZERO
#  Autonomous Emergency Braking · Raspberry Pi 5 · QNX Neutrino RTOS
#  QNX eHACK 2026 · Problem Statement #2
#
#  ZERO DEPENDENCIES — standard library only.
#      Ubuntu/Debian:  sudo apt install python3-tk
#
#  RUN
#      python3 aeb_dashboard.py                  # demo — no hardware needed
#      python3 aeb_dashboard.py --csv aeb_log.csv
#      python3 aeb_dashboard.py --udp 5005
#
#  KEYS   SPACE pause/run · R restart run · F inject brake fault · Q quit
# =============================================================================

import argparse, csv, math, os, queue, socket, threading, time
from collections import deque

try:
    import tkinter as tk
except ImportError:
    raise SystemExit("tkinter missing  ->  sudo apt install python3-tk")

# ----------------------------------------------------------------- palette
BG, PANEL, PANEL_HI, STROKE = "#080C14", "#111A2B", "#1A2740", "#25334D"
TEXT, MUTED, DIM = "#EAF0FA", "#8FA0BC", "#54637E"
C_SAFE, C_WARN, C_STRONG, C_EMERG = "#28D9A0", "#FFC94A", "#FF8A3D", "#FF4757"
C_ACCENT, C_CYAN = "#3F9BFF", "#31E1F7"

STATE_COLOR = {"SAFE": C_SAFE, "WARNING": C_WARN,
               "STRONG": C_STRONG, "EMERGENCY": C_EMERG}
STATE_GLOW = {"SAFE": "#0C3A2C", "WARNING": "#3A3113",
              "STRONG": "#3A2413", "EMERGENCY": "#3D1620"}
STATE_CAPTION = {"SAFE": "road clear  ·  cruising",
                 "WARNING": "obstacle ahead  ·  partial braking",
                 "STRONG": "closing fast  ·  strong braking",
                 "EMERGENCY": "collision imminent  ·  full braking"}
LEVELS = ["NONE", "PARTIAL", "STRONG", "FULL"]
LEVEL_COLOR = {"NONE": DIM, "PARTIAL": C_WARN, "STRONG": C_STRONG, "FULL": C_EMERG}
DUTY = {"NONE": 60, "PARTIAL": 40, "STRONG": 70, "FULL": 100}
DEADLINE_MS = 50.0

# ------------------------------------------------- controller braking model
T_DELAY_S, SAFETY_FACTOR, TTC_CRITICAL = 0.080, 1.3, 0.6
DECEL_PARTIAL, DECEL_STRONG, DECEL_FULL = 2.0, 4.5, 7.5


def stopping_distance(v, a):
    return float("inf") if a <= 0 else v * T_DELAY_S + (v * v) / (2.0 * a)


def thresholds(v, sf=SAFETY_FACTOR):
    return (sf * stopping_distance(v, DECEL_PARTIAL),
            sf * stopping_distance(v, DECEL_STRONG),
            sf * stopping_distance(v, DECEL_FULL))


def decide(distance, v, sf=SAFETY_FACTOR, ttc_thresh=TTC_CRITICAL,
           dp=DECEL_PARTIAL, ds=DECEL_STRONG, df=DECEL_FULL):
    ttc = distance / v if v > 0.01 else float("inf")
    d_p = sf * stopping_distance(v, dp)
    d_s = sf * stopping_distance(v, ds)
    d_f = sf * stopping_distance(v, df)
    margin = distance - d_f
    if ttc < ttc_thresh: return "FULL", "EMERGENCY", ttc, margin, df
    if margin < 0.0:     return "FULL", "EMERGENCY", ttc, margin, df
    if distance > d_p:   return "NONE", "SAFE", ttc, margin, 0.0
    if distance > d_s:   return "PARTIAL", "WARNING", ttc, margin, dp
    if distance > d_f:   return "STRONG", "STRONG", ttc, margin, ds
    return "FULL", "EMERGENCY", ttc, margin, df


class Telemetry:
    __slots__ = ("t speed distance ttc margin state level cmd_decel meas_decel "
                 "latency_ms brake_fault impact interlock").split()

    def __init__(self):
        self.t = self.speed = self.margin = self.cmd_decel = 0.0
        self.meas_decel = self.latency_ms = 0.0
        self.distance, self.ttc = 50.0, 999.0
        self.state, self.level = "SAFE", "NONE"
        self.brake_fault = self.impact = self.interlock = False


# ============================================================ data sources
class DemoSource:
    """Built-in vehicle model running the real decision ladder."""

    def __init__(self):
        self.q, self._stop = queue.Queue(), threading.Event()
        self.paused = False
        self.inject_fault = False
        self.t0 = time.time()
        self._reset()

    def _reset(self):
        self.speed, self.distance, self.settle = 13.0, 95.0, 0.0

    def reset(self):
        self._reset()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _run(self):
        dt = 0.04
        while not self._stop.is_set():
            if self.paused:
                time.sleep(dt)
                continue
            lv, st, ttc, mg, dc = decide(self.distance, self.speed)
            eff = dc * (0.10 if self.inject_fault else 1.0)
            self.speed = max(0.0, self.speed - eff * dt)
            self.distance = max(0.0, self.distance - self.speed * dt)

            tm = Telemetry()
            tm.t = time.time() - self.t0
            tm.speed, tm.distance = self.speed, self.distance
            tm.ttc, tm.margin, tm.state, tm.level = ttc, mg, st, lv
            tm.cmd_decel = dc
            tm.meas_decel = max(0.0, eff * 0.95 + math.sin(tm.t * 9) * 0.08)
            tm.latency_ms = (2.05 + 0.6 * abs(math.sin(tm.t * 3.3))
                             + 0.25 * abs(math.sin(tm.t * 11)))
            tm.brake_fault = self.inject_fault and dc > 0.1 and self.speed > 1.0
            tm.impact = self.distance <= 0.05 and self.speed > 0.5
            self.q.put(tm)

            if self.speed <= 0.02 or self.distance <= 0.02:
                self.settle += dt
                if self.settle > 1.8:
                    self._reset()
            time.sleep(dt)


class CsvSource:
    """Tails aeb_log.csv written by logger_task on the target."""

    def __init__(self, path):
        self.path, self.q = path, queue.Queue()
        self._stop = threading.Event()
        self.paused = False
        self.inject_fault = False

    def reset(self): pass
    def start(self): threading.Thread(target=self._run, daemon=True).start()
    def stop(self): self._stop.set()

    def _run(self):
        last, pd, pt = 0, None, None
        while not self._stop.is_set():
            try:
                if not os.path.exists(self.path):
                    time.sleep(0.4)
                    continue
                if os.path.getsize(self.path) < last:
                    last = 0
                with open(self.path) as f:
                    f.seek(last)
                    for row in csv.reader(f):
                        if not row or row[0].startswith("t_ns"):
                            continue
                        try:
                            t_ns = float(row[0]); state, level = row[1], row[2]
                            dist, ttc = float(row[3]), float(row[4])
                            mg, lat = float(row[5]), float(row[6])
                        except (ValueError, IndexError):
                            continue
                        tm = Telemetry()
                        tm.t, tm.distance, tm.ttc = t_ns / 1e9, dist, ttc
                        tm.margin, tm.state, tm.level = mg, state, level
                        tm.latency_ms = lat / 1e6
                        if pd is not None and pt is not None and tm.t > pt:
                            tm.speed = max(0.0, (pd - dist) / (tm.t - pt))
                        pd, pt = dist, tm.t
                        tm.cmd_decel = {"NONE": 0.0, "PARTIAL": DECEL_PARTIAL,
                                        "STRONG": DECEL_STRONG,
                                        "FULL": DECEL_FULL}.get(level, 0.0)
                        tm.meas_decel = tm.cmd_decel
                        self.q.put(tm)
                    last = f.tell()
            except OSError:
                pass
            time.sleep(0.12)


class UdpSource:
    """Listens for 'key=value;...' telemetry datagrams from the target."""

    def __init__(self, port):
        self.port, self.q = port, queue.Queue()
        self._stop = threading.Event()
        self.t0 = time.time()
        self.paused = False
        self.inject_fault = False

    def reset(self): pass
    def start(self): threading.Thread(target=self._run, daemon=True).start()
    def stop(self): self._stop.set()

    def _run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(0.5)
        s.bind(("0.0.0.0", self.port))
        while not self._stop.is_set():
            try:
                data, _ = s.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            tm = self.parse(data.decode("utf-8", "ignore"))
            if tm:
                tm.t = time.time() - self.t0
                self.q.put(tm)
        s.close()

    @staticmethod
    def parse(text):
        d = {}
        for part in text.strip().split(";"):
            if "=" in part:
                k, v = part.split("=", 1)
                d[k.strip()] = v.strip()
        if not d:
            return None
        tm = Telemetry()

        def num(k, dv=0.0):
            try:
                return float(d.get(k, dv))
            except ValueError:
                return dv

        tm.speed, tm.distance = num("speed"), num("dist")
        tm.ttc, tm.margin = num("ttc", 999.0), num("margin")
        tm.state = d.get("state", "SAFE").upper()
        tm.level = d.get("level", "NONE").upper()
        tm.cmd_decel, tm.meas_decel = num("cmd"), num("meas")
        tm.latency_ms = num("lat")
        tm.brake_fault = d.get("fault", "0") not in ("0", "", "no")
        tm.impact = d.get("impact", "0") not in ("0", "", "no")
        tm.interlock = d.get("lock", "0") not in ("0", "", "no")
        return tm


# ============================================================== ui helpers
def rpts(x1, y1, x2, y2, r):
    return [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r, x2, y2 - r, x2, y2,
            x2 - r, y2, x1 + r, y2, x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]


def mix(c1, c2, t):
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    t = max(0.0, min(1.0, t))
    return "#%02x%02x%02x" % tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


# ================================================================ dashboard
class Dash:
    W, H = 1400, 860

    def __init__(self, root, src, label):
        self.root, self.src, self.label = root, src, label
        self.tm = Telemetry()
        self.lat = deque(maxlen=200)
        self.events = deque(maxlen=7)
        self.last_state = None
        self.frame = self.pkts = 0
        self.last_rx = 0.0
        self.sweep = 0.0

        root.title("AEB Control Dashboard — Team PRIORITY ZERO")
        root.configure(bg=BG)
        root.geometry(f"{self.W}x{self.H}")
        root.minsize(1240, 800)
        self.cv = tk.Canvas(root, width=self.W, height=self.H, bg=BG,
                            highlightthickness=0)
        self.cv.pack(fill="both", expand=True)

        root.bind("<space>", lambda e: self._pause())
        root.bind("r", lambda e: self.src.reset())
        root.bind("f", lambda e: self._fault())
        root.bind("q", lambda e: root.destroy())

        self._static()
        self._tick()

    def _pause(self):
        self.src.paused = not getattr(self.src, "paused", False)

    def _fault(self):
        self.src.inject_fault = not getattr(self.src, "inject_fault", False)

    def _card(self, x, y, w, h, title=None, fill=PANEL):
        self.cv.create_polygon(rpts(x, y, x + w, y + h, 13), smooth=True,
                               fill=fill, outline=STROKE, width=1)
        if title:
            self.cv.create_text(x + 15, y + 17, text=title.upper(), anchor="w",
                                fill=MUTED, font=("Segoe UI", 8, "bold"))

    # ---------------------------------------------------------------- build
    def _static(self):
        cv = self.cv
        cv.create_rectangle(0, 0, self.W, 86, fill="#0B111C", outline="")
        cv.create_text(26, 30, text="AEB CONTROL DASHBOARD", anchor="w",
                       fill=TEXT, font=("Segoe UI", 19, "bold"))
        cv.create_text(28, 56, anchor="w", fill=MUTED, font=("Segoe UI", 9),
                       text="Autonomous Emergency Braking   ·   Raspberry Pi 5   ·   "
                            "QNX Neutrino RTOS   ·   8 SCHED_FIFO threads / 4 cores")
        cv.create_text(self.W - 26, 24, text="TEAM PRIORITY ZERO", anchor="e",
                       fill=C_ACCENT, font=("Segoe UI", 10, "bold"))
        cv.create_text(self.W - 26, 43, anchor="e", fill=DIM,
                       font=("Segoe UI", 8),
                       text="QNX eHACK 2026 · Problem Statement #2")
        self.id_link = cv.create_text(self.W - 26, 62, anchor="e", text="",
                                      fill=MUTED, font=("Segoe UI", 8))
        cv.create_line(0, 86, self.W, 86, fill=STROKE)

        # ---------- hero: road view ----------
        self.rx, self.ry, self.rw, self.rh = 26, 100, 902, 300
        self._card(self.rx, self.ry, self.rw, self.rh,
                   "live road view  ·  braking zones scale with speed")
        self.road_y = self.ry + 172
        self.road_x0 = self.rx + 30
        self.road_x1 = self.rx + self.rw - 30
        cv.create_rectangle(self.road_x0, self.road_y - 44,
                            self.road_x1, self.road_y + 44,
                            fill="#0C1320", outline="")
        for yy in (self.road_y - 44, self.road_y + 44):
            cv.create_line(self.road_x0, yy, self.road_x1, yy, fill=STROKE)
        self.zone_ids = []
        self.dyn_ids = []
        self.id_zonelbl = [cv.create_text(0, 0, text="", anchor="w", fill=DIM,
                                          font=("Segoe UI", 8, "bold"),
                                          state="hidden") for _ in range(3)]
        self.id_gap = cv.create_text(0, 0, text="", fill=TEXT,
                                     font=("Segoe UI", 10, "bold"), state="hidden")
        self.id_scale = cv.create_text(self.road_x0, self.ry + 258, anchor="w",
                                       text="", fill=DIM, font=("Segoe UI", 8))

        # ---------- state banner ----------
        self.bx, self.by, self.bw, self.bh = 946, 100, 428, 128
        self.id_glow = cv.create_polygon(
            rpts(self.bx - 3, self.by - 3, self.bx + self.bw + 3,
                 self.by + self.bh + 3, 15), smooth=True,
            fill=STATE_GLOW["SAFE"], outline="")
        self.id_banner = cv.create_polygon(
            rpts(self.bx, self.by, self.bx + self.bw, self.by + self.bh, 13),
            smooth=True, fill=PANEL, outline=C_SAFE, width=2)
        self.id_state = cv.create_text(self.bx + self.bw / 2, self.by + 50,
                                       text="SAFE", fill=C_SAFE,
                                       font=("Segoe UI", 33, "bold"))
        self.id_cap = cv.create_text(self.bx + self.bw / 2, self.by + 85, text="",
                                     fill=MUTED, font=("Segoe UI", 9))
        self.id_lock = cv.create_text(self.bx + self.bw / 2, self.by + 107, text="",
                                      fill=DIM, font=("Segoe UI", 8, "bold"))

        # ---------- radar scope ----------
        self._card(946, 244, 206, 218, "radar  ·  RD-03D")
        self.sc_cx, self.sc_cy, self.sc_r = 1049, 364, 72
        for k in (1, 2, 3):
            rr = self.sc_r * k / 3
            cv.create_oval(self.sc_cx - rr, self.sc_cy - rr,
                           self.sc_cx + rr, self.sc_cy + rr, outline=PANEL_HI)
        cv.create_line(self.sc_cx - self.sc_r, self.sc_cy,
                       self.sc_cx + self.sc_r, self.sc_cy, fill=PANEL_HI)
        cv.create_line(self.sc_cx, self.sc_cy - self.sc_r,
                       self.sc_cx, self.sc_cy + self.sc_r, fill=PANEL_HI)
        self.id_sweep = cv.create_line(self.sc_cx, self.sc_cy,
                                       self.sc_cx, self.sc_cy - self.sc_r,
                                       fill=C_CYAN, width=2)
        self.id_blipring = cv.create_oval(0, 0, 0, 0, outline=C_EMERG,
                                          state="hidden")
        self.id_blip = cv.create_oval(0, 0, 0, 0, fill=C_EMERG, outline="",
                                      state="hidden")
        cv.create_text(1049, 448, text="60 m range  ·  Doppler closing speed",
                       fill=DIM, font=("Segoe UI", 7))

        # ---------- speedometer ----------
        self._card(1168, 244, 206, 218, "vehicle speed")
        self.g_cx, self.g_cy, self.g_r = 1271, 366, 70
        cv.create_arc(self.g_cx - self.g_r, self.g_cy - self.g_r,
                      self.g_cx + self.g_r, self.g_cy + self.g_r,
                      start=210, extent=-240, style="arc", outline=PANEL_HI,
                      width=12)
        for i in range(9):
            a = math.radians(210 - i * 30)
            r1, r2 = self.g_r - 17, self.g_r - 9
            cv.create_line(self.g_cx + r1 * math.cos(a),
                           self.g_cy - r1 * math.sin(a),
                           self.g_cx + r2 * math.cos(a),
                           self.g_cy - r2 * math.sin(a), fill=DIM)
        self.id_arc = cv.create_arc(self.g_cx - self.g_r, self.g_cy - self.g_r,
                                    self.g_cx + self.g_r, self.g_cy + self.g_r,
                                    start=210, extent=-1, style="arc",
                                    outline=C_ACCENT, width=12)
        self.id_needle = cv.create_line(self.g_cx, self.g_cy,
                                        self.g_cx, self.g_cy - self.g_r + 20,
                                        fill=TEXT, width=2)
        cv.create_oval(self.g_cx - 5, self.g_cy - 5, self.g_cx + 5, self.g_cy + 5,
                       fill=TEXT, outline="")
        self.id_speed = cv.create_text(self.g_cx, self.g_cy + 36, text="0.0",
                                       fill=TEXT, font=("Segoe UI", 21, "bold"))
        cv.create_text(self.g_cx, self.g_cy + 57, text="m/s", fill=MUTED,
                       font=("Segoe UI", 8))

        # ---------- metric strip ----------
        self.metrics = {}
        for i, (key, lbl, unit) in enumerate(
                [("dist", "DISTANCE", "m"), ("ttc", "TIME TO COLLISION", "s"),
                 ("margin", "SAFETY MARGIN", "m"), ("decel", "DECELERATION", "m/s²")]):
            x = 26 + i * 226
            self._card(x, 416, 214, 92)
            cv.create_text(x + 15, 436, text=lbl, anchor="w", fill=DIM,
                           font=("Segoe UI", 8, "bold"))
            v = cv.create_text(x + 15, 470, text="--", anchor="w", fill=TEXT,
                               font=("Segoe UI", 25, "bold"))
            cv.create_text(x + 199, 474, text=unit, anchor="e", fill=MUTED,
                           font=("Segoe UI", 9))
            cv.create_polygon(rpts(x + 15, 488, x + 199, 495, 3), smooth=True,
                              fill=PANEL_HI, outline="")
            b = cv.create_polygon(rpts(x + 15, 488, x + 18, 495, 3), smooth=True,
                                  fill=C_SAFE, outline="")
            self.metrics[key] = (v, b, x + 15, x + 199)

        # ---------- brake ladder ----------
        self._card(26, 524, 452, 116, "brake actuation")
        self.lad = []
        for i, lv in enumerate(LEVELS):
            x = 44 + i * 106
            rid = cv.create_polygon(rpts(x, 556, x + 94, 602, 8), smooth=True,
                                    fill=PANEL_HI, outline="")
            tid = cv.create_text(x + 47, 579, text=lv, fill=DIM,
                                 font=("Segoe UI", 9, "bold"))
            self.lad.append((rid, tid))
        self.id_duty = cv.create_text(44, 622, anchor="w", text="", fill=MUTED,
                                      font=("Segoe UI", 8))

        # ---------- IMU ----------
        self._card(490, 524, 438, 116, "IMU  ·  brake verification")
        cv.create_text(508, 552, text="COMMANDED", anchor="w", fill=DIM,
                       font=("Segoe UI", 8, "bold"))
        self.id_cmd = cv.create_text(770, 552, text="--", anchor="e", fill=TEXT,
                                     font=("Segoe UI", 9, "bold"))
        cv.create_polygon(rpts(508, 560, 770, 568, 4), smooth=True,
                          fill=PANEL_HI, outline="")
        self.id_cmdbar = cv.create_polygon(rpts(508, 560, 511, 568, 4),
                                           smooth=True, fill=C_ACCENT, outline="")
        cv.create_text(508, 586, text="MEASURED", anchor="w", fill=DIM,
                       font=("Segoe UI", 8, "bold"))
        self.id_meas = cv.create_text(770, 586, text="--", anchor="e", fill=TEXT,
                                      font=("Segoe UI", 9, "bold"))
        cv.create_polygon(rpts(508, 594, 770, 602, 4), smooth=True,
                          fill=PANEL_HI, outline="")
        self.id_measbar = cv.create_polygon(rpts(508, 594, 511, 602, 4),
                                            smooth=True, fill=C_SAFE, outline="")
        cv.create_text(508, 620, anchor="w", fill=DIM, font=("Segoe UI", 7),
                       text="MPU-6050 confirms the brakes physically bite")
        self.id_fpill = cv.create_polygon(rpts(790, 552, 912, 578, 7),
                                          smooth=True, fill=PANEL_HI, outline="")
        self.id_ftxt = cv.create_text(851, 565, text="BRAKE OK", fill=C_SAFE,
                                      font=("Segoe UI", 8, "bold"))
        self.id_ipill = cv.create_polygon(rpts(790, 584, 912, 610, 7),
                                          smooth=True, fill=PANEL_HI, outline="")
        self.id_itxt = cv.create_text(851, 597, text="NO IMPACT", fill=MUTED,
                                      font=("Segoe UI", 8, "bold"))

        # ---------- deadline ----------
        self._card(946, 476, 428, 164, "deadline performance  ·  sense → brake")
        self.id_lat = cv.create_text(966, 522, anchor="w", text="--", fill=C_SAFE,
                                     font=("Segoe UI", 30, "bold"))
        cv.create_text(1056, 528, anchor="w", text="ms", fill=MUTED,
                       font=("Segoe UI", 9))
        self.sp_x, self.sp_y, self.sp_w, self.sp_h = 1096, 500, 258, 44
        cv.create_rectangle(self.sp_x, self.sp_y, self.sp_x + self.sp_w,
                            self.sp_y + self.sp_h, outline=STROKE)
        cv.create_text(self.sp_x + self.sp_w, self.sp_y - 5, anchor="e",
                       text="latency history", fill=DIM, font=("Segoe UI", 7))
        self.id_spark = None
        cv.create_polygon(rpts(966, 560, 1354, 570, 5), smooth=True,
                          fill=PANEL_HI, outline="")
        self.id_latbar = cv.create_polygon(rpts(966, 560, 969, 570, 5),
                                           smooth=True, fill=C_SAFE, outline="")
        cv.create_text(966, 582, anchor="w", text="0", fill=DIM,
                       font=("Segoe UI", 7))
        cv.create_text(1354, 582, anchor="e", fill=DIM, font=("Segoe UI", 7),
                       text=f"{DEADLINE_MS:.0f} ms deadline")
        self.id_stats = cv.create_text(966, 610, anchor="w", text="", fill=MUTED,
                                       font=("Consolas", 8))
        self.id_miss = cv.create_text(1354, 610, anchor="e", text="", fill=C_SAFE,
                                      font=("Segoe UI", 9, "bold"))

        # ---------- timeline ----------
        self._card(26, 656, 1348, 176, "collision timeline")
        cv.create_text(46, 686, anchor="w", fill=DIM,
                       font=("Consolas", 8, "bold"),
                       text=f"{'TIME':<10}{'STATE':<12}{'LEVEL':<10}{'DIST':<10}"
                            f"{'TTC':<10}{'MARGIN':<11}{'LATENCY'}")
        self.rows = [cv.create_text(46, 708 + i * 17, anchor="w", text="",
                                    fill=MUTED, font=("Consolas", 9))
                     for i in range(7)]
        cv.create_text(self.W - 26, 843, anchor="e", fill=DIM,
                       font=("Segoe UI", 8),
                       text="SPACE pause   ·   R restart   ·   "
                            "F inject brake fault   ·   Q quit")

    # ---------------------------------------------------------------- loop
    def _tick(self):
        got = None
        try:
            while True:
                got = self.src.q.get_nowait()
                self.pkts += 1
        except queue.Empty:
            pass
        if got is not None:
            self.last_rx = time.time()
            self.tm = got
            self.lat.append(got.latency_ms)
            if got.state != self.last_state:
                self.events.appendleft(got)
                self.last_state = got.state
        self.frame += 1
        self.sweep = (self.sweep + 7.5) % 360
        self._render()
        self.root.after(40, self._tick)

    def _render(self):
        cv, tm = self.cv, self.tm
        col = STATE_COLOR.get(tm.state, MUTED)
        age = time.time() - self.last_rx if self.last_rx else 999
        live = age < 1.5
        paused = getattr(self.src, "paused", False)
        fault = getattr(self.src, "inject_fault", False)

        cv.itemconfig(self.id_link,
                      text=f"{self.label}   "
                           f"{'|| PAUSED' if paused else ('● LIVE' if live else '○ no data')}"
                           f"{'   ⚠ FAULT INJECTED' if fault else ''}   "
                           f"{self.pkts} samples")

        # banner (pulses in EMERGENCY)
        if tm.state == "EMERGENCY":
            k = 0.5 + 0.5 * math.sin(self.frame * 0.35)
            cv.itemconfig(self.id_glow, fill=mix(BG, C_EMERG, 0.14 + 0.20 * k))
            cv.itemconfig(self.id_banner,
                          outline=mix(C_EMERG, "#FFFFFF", 0.25 * k),
                          width=2 + 1.5 * k)
        else:
            cv.itemconfig(self.id_glow, fill=STATE_GLOW.get(tm.state, PANEL))
            cv.itemconfig(self.id_banner, outline=col, width=2)
        cv.itemconfig(self.id_state, text=tm.state, fill=col)
        cv.itemconfig(self.id_cap, text=STATE_CAPTION.get(tm.state, ""))
        cv.itemconfig(self.id_lock,
                      text="◉ INTERLOCK ARMED" if tm.interlock else "○ INTERLOCK OPEN",
                      fill=C_SAFE if tm.interlock else DIM)

        self._road(tm, col)
        self._radar(tm)

        # speedometer
        f = max(0.0, min(1.0, tm.speed / 20.0))
        cv.itemconfig(self.id_arc, extent=-max(0.6, 240 * f), outline=col)
        a = math.radians(210 - 240 * f)
        rr = self.g_r - 21
        cv.coords(self.id_needle, self.g_cx, self.g_cy,
                  self.g_cx + rr * math.cos(a), self.g_cy - rr * math.sin(a))
        cv.itemconfig(self.id_needle, fill=col)
        cv.itemconfig(self.id_speed, text=f"{tm.speed:.1f}")

        # metrics
        self._metric("dist", f"{tm.distance:.1f}", tm.distance / 60.0, col)
        self._metric("ttc", "∞" if tm.ttc > 99 else f"{tm.ttc:.2f}",
                     0 if tm.ttc > 99 else 1 - min(1, tm.ttc / 5),
                     C_EMERG if tm.ttc < TTC_CRITICAL else col)
        self._metric("margin", f"{tm.margin:+.1f}", min(1, abs(tm.margin) / 40),
                     C_SAFE if tm.margin >= 0 else C_EMERG)
        self._metric("decel", f"{tm.cmd_decel:.1f}", tm.cmd_decel / DECEL_FULL, col)

        # brake ladder
        act = LEVELS.index(tm.level) if tm.level in LEVELS else 0
        for i, (rid, tid) in enumerate(self.lad):
            on = (i <= act and tm.level != "NONE") or (i == 0 and tm.level == "NONE")
            c = LEVEL_COLOR[LEVELS[i]]
            if on and i == act and tm.level == "FULL":
                k = 0.5 + 0.5 * math.sin(self.frame * 0.4)
                c = mix(C_EMERG, "#FFFFFF", 0.30 * k)
            cv.itemconfig(rid, fill=c if on else PANEL_HI)
            cv.itemconfig(tid, fill="#08111C" if on else DIM)
        cv.itemconfig(self.id_duty,
                      text=f"PWM duty {DUTY.get(tm.level, 0)}%   ·   "
                           f"both L298N drivers   ·   4 wheels short-braked")

        # IMU
        cv.itemconfig(self.id_cmd, text=f"{tm.cmd_decel:.2f}")
        cv.itemconfig(self.id_meas, text=f"{tm.meas_decel:.2f}")
        cf = max(0.0, min(1.0, tm.cmd_decel / DECEL_FULL))
        mf = max(0.0, min(1.0, tm.meas_decel / DECEL_FULL))
        cv.coords(self.id_cmdbar, *rpts(508, 560, 508 + max(4, 262 * cf), 568, 4))
        cv.coords(self.id_measbar, *rpts(508, 594, 508 + max(4, 262 * mf), 602, 4))
        cv.itemconfig(self.id_measbar, fill=C_EMERG if tm.brake_fault else C_SAFE)
        cv.itemconfig(self.id_fpill, fill="#3D1620" if tm.brake_fault else PANEL_HI)
        cv.itemconfig(self.id_ftxt,
                      text="⚠ BRAKE FAULT" if tm.brake_fault else "✔ BRAKE OK",
                      fill=C_EMERG if tm.brake_fault else C_SAFE)
        cv.itemconfig(self.id_ipill, fill="#3D1620" if tm.impact else PANEL_HI)
        cv.itemconfig(self.id_itxt, text="⚠ IMPACT" if tm.impact else "NO IMPACT",
                      fill=C_EMERG if tm.impact else MUTED)

        # deadline
        if self.lat:
            mn, mx = min(self.lat), max(self.lat)
            av = sum(self.lat) / len(self.lat)
            miss = sum(1 for v in self.lat if v > DEADLINE_MS)
            lf = max(0.0, min(1.0, tm.latency_ms / DEADLINE_MS))
            lc = C_SAFE if lf < 0.6 else (C_WARN if lf < 0.9 else C_EMERG)
            cv.itemconfig(self.id_lat, text=f"{tm.latency_ms:.2f}", fill=lc)
            cv.coords(self.id_latbar,
                      *rpts(966, 560, 966 + max(4, 388 * lf), 570, 5))
            cv.itemconfig(self.id_latbar, fill=lc)
            cv.itemconfig(self.id_stats,
                          text=f"min {mn:5.2f}   avg {av:5.2f}   "
                               f"max {mx:5.2f}   n={len(self.lat)}")
            cv.itemconfig(self.id_miss, text=f"{miss} misses",
                          fill=C_SAFE if miss == 0 else C_EMERG)
            self._spark()

        # timeline
        for i, rid in enumerate(self.rows):
            if i < len(self.events):
                e = self.events[i]
                ttc_s = "inf" if e.ttc > 99 else f"{e.ttc:.2f}"
                cv.itemconfig(rid, fill=STATE_COLOR.get(e.state, MUTED),
                              text=f"{e.t:<10.2f}{e.state:<12}{e.level:<10}"
                                   f"{e.distance:<10.2f}{ttc_s:<10}"
                                   f"{e.margin:<+11.2f}{e.latency_ms:.3f} ms")
            else:
                cv.itemconfig(rid, text="")

    def _metric(self, key, val, frac, color):
        vid, bid, x0, x1 = self.metrics[key]
        self.cv.itemconfig(vid, text=val)
        w = (x1 - x0) * max(0.0, min(1.0, frac))
        self.cv.coords(bid, *rpts(x0, 488, x0 + max(3, w), 495, 3))
        self.cv.itemconfig(bid, fill=color)

    # ------------------------------------------------------------ road view
    def _road(self, tm, col):
        cv = self.cv
        for i in self.zone_ids + self.dyn_ids:
            cv.delete(i)
        self.zone_ids, self.dyn_ids = [], []

        span = max(30.0, min(110.0, tm.distance * 1.25))
        usable = self.road_x1 - self.road_x0 - 76
        ppm = usable / span

        def X(m):
            return self.road_x0 + 46 + m * ppm

        d_p, d_s, d_f = thresholds(tm.speed)
        y0, y1 = self.road_y - 44, self.road_y + 44
        bands = [(0.0, min(d_f, span), "#3D1620", "FULL", C_EMERG),
                 (min(d_f, span), min(d_s, span), "#3A2413", "STRONG", C_STRONG),
                 (min(d_s, span), min(d_p, span), "#3A3113", "WARNING", C_WARN)]
        for a, b, fill, _l, _c in bands:
            if b - a > 0.4:
                self.zone_ids.append(cv.create_rectangle(
                    X(a), y0 + 1, X(b), y1 - 1, fill=fill, outline=""))
        for i, (a, b, _f, lbl, lc) in enumerate(bands):
            lid = self.id_zonelbl[i]
            if b - a > 6 and X(a) < self.road_x1 - 40:
                cv.coords(lid, X(a) + 5, y0 - 13)
                cv.itemconfig(lid, text=lbl, state="normal", fill=lc)
            else:
                cv.itemconfig(lid, state="hidden")

        # lane dashes scroll at vehicle speed
        off = (self.frame * max(0.6, tm.speed * 0.9)) % 44
        x = self.road_x0 + 8 - off
        while x < self.road_x1:
            if x > self.road_x0:
                self.dyn_ids.append(cv.create_line(
                    x, self.road_y, min(x + 22, self.road_x1), self.road_y,
                    fill="#22314A", width=3))
            x += 44

        vx, vy = self.road_x0 + 46, self.road_y
        ox = X(min(tm.distance, span))

        # radar cone
        self.dyn_ids.append(cv.create_polygon(
            vx + 30, vy, min(ox, self.road_x1 - 4), vy - 30,
            min(ox, self.road_x1 - 4), vy + 30,
            fill=mix(BG, C_CYAN, 0.10), outline=""))

        # ego vehicle
        d = self.dyn_ids
        d.append(cv.create_polygon(rpts(vx - 34, vy - 17, vx + 30, vy + 17, 6),
                                   smooth=True, fill="#1B2C4A",
                                   outline=col, width=2))
        d.append(cv.create_polygon(rpts(vx - 16, vy - 12, vx + 8, vy + 12, 4),
                                   smooth=True, fill="#233A61", outline=""))
        for wy in (vy - 22, vy + 16):
            for wx in (vx - 26, vx + 12):
                d.append(cv.create_rectangle(wx, wy, wx + 13, wy + 6,
                                             fill="#0C1320", outline=""))
        d.append(cv.create_text(vx - 2, vy + 34, text="EGO", fill=DIM,
                                font=("Segoe UI", 7, "bold")))
        if tm.level != "NONE":
            k = 0.55 + 0.45 * math.sin(self.frame * 0.5)
            d.append(cv.create_rectangle(vx - 37, vy - 12, vx - 32, vy + 12,
                                         fill=mix("#4A0F16", C_EMERG, k),
                                         outline=""))

        # obstacle
        if tm.distance <= span:
            d.append(cv.create_polygon(rpts(ox - 24, vy - 20, ox + 24, vy + 20, 6),
                                       smooth=True, fill="#2A1620",
                                       outline=C_EMERG, width=2))
            d.append(cv.create_polygon(rpts(ox - 14, vy - 12, ox + 14, vy + 12, 4),
                                       smooth=True, fill="#43222E", outline=""))
            d.append(cv.create_text(ox, vy + 36, text="OBSTACLE", fill=C_EMERG,
                                    font=("Segoe UI", 7, "bold")))
            gx = (vx + 30 + ox) / 2
            d.append(cv.create_line(vx + 32, vy - 58, ox - 26, vy - 58,
                                    fill=MUTED, arrow="both"))
            cv.coords(self.id_gap, gx, vy - 70)
            cv.itemconfig(self.id_gap, text=f"{tm.distance:.1f} m",
                          state="normal", fill=col)
        else:
            cv.itemconfig(self.id_gap, state="hidden")

        cv.itemconfig(self.id_scale,
                      text=f"view span {span:.0f} m    ·    thresholds at this speed:"
                           f"   partial {d_p:.1f} m     strong {d_s:.1f} m"
                           f"     full {d_f:.1f} m")

    def _radar(self, tm):
        cv = self.cv
        a = math.radians(self.sweep)
        cv.coords(self.id_sweep, self.sc_cx, self.sc_cy,
                  self.sc_cx + self.sc_r * math.sin(a),
                  self.sc_cy - self.sc_r * math.cos(a))
        if tm.distance <= 60:
            rr = self.sc_r * (tm.distance / 60.0)
            bx, by = self.sc_cx, self.sc_cy - rr
            c = STATE_COLOR.get(tm.state, C_EMERG)
            cv.coords(self.id_blip, bx - 5, by - 5, bx + 5, by + 5)
            cv.itemconfig(self.id_blip, state="normal", fill=c)
            ph = self.frame % 20
            pr = 6 + ph
            cv.coords(self.id_blipring, bx - pr, by - pr, bx + pr, by + pr)
            cv.itemconfig(self.id_blipring, state="normal",
                          outline=mix(BG, c, 1 - ph / 20))
        else:
            cv.itemconfig(self.id_blip, state="hidden")
            cv.itemconfig(self.id_blipring, state="hidden")

    def _spark(self):
        cv = self.cv
        if self.id_spark:
            cv.delete(self.id_spark)
            self.id_spark = None
        if len(self.lat) < 2:
            return
        hi = max(max(self.lat), DEADLINE_MS * 0.25)
        step = self.sp_w / max(1, len(self.lat) - 1)
        pts = []
        for i, v in enumerate(self.lat):
            pts += [self.sp_x + i * step,
                    self.sp_y + self.sp_h - (v / hi) * (self.sp_h - 6) - 3]
        self.id_spark = cv.create_line(*pts, fill=C_CYAN, width=1)


def main():
    ap = argparse.ArgumentParser(description="AEB Dashboard — Team PRIORITY ZERO")
    ap.add_argument("--csv", metavar="PATH", help="tail aeb_log.csv from the target")
    ap.add_argument("--udp", metavar="PORT", type=int, help="UDP telemetry port")
    a = ap.parse_args()
    if a.csv:
        src, lbl = CsvSource(a.csv), f"CSV · {os.path.basename(a.csv)}"
    elif a.udp:
        src, lbl = UdpSource(a.udp), f"UDP · port {a.udp}"
    else:
        src, lbl = DemoSource(), "DEMO · simulated vehicle"
    src.start()
    root = tk.Tk()
    Dash(root, src, lbl)
    try:
        root.mainloop()
    finally:
        src.stop()


if __name__ == "__main__":
    main()

# =============================================================================
#  OPTIONAL — C-side UDP telemetry sender, add inside logger_task() in aeb.c.
#  The logger runs at priority 10 (lowest), so telemetry can never delay a
#  braking decision.
#
#    #include <arpa/inet.h>
#    static int tfd = -1; static struct sockaddr_in taddr;
#    tfd = socket(AF_INET, SOCK_DGRAM, 0);
#    memset(&taddr, 0, sizeof taddr);
#    taddr.sin_family = AF_INET; taddr.sin_port = htons(5005);
#    taddr.sin_addr.s_addr = inet_addr("192.168.1.50");   /* laptop IP */
#
#    char p[256];
#    int n = snprintf(p, sizeof p,
#      "speed=%.2f;dist=%.2f;ttc=%.2f;margin=%.2f;state=%s;level=%s;"
#      "cmd=%.2f;meas=%.2f;lat=%.3f;fault=%d;impact=%d;lock=%d",
#      g_ctl.speed_mps, e.distance_m, e.ttc_s, e.margin_m, ST[e.state],
#      LV[e.level], g_ctl.commanded_decel, g_ctl.measured_decel,
#      e.latency_ns/1e6, g_ctl.brake_fault, g_ctl.impact, g_ctl.interlock_ok);
#    sendto(tfd, p, n, 0, (struct sockaddr *)&taddr, sizeof taddr);
# =============================================================================
