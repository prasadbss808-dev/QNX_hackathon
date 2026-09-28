# Autonomous Emergency Braking Controller

**Team PRIORITY ZERO** · QNX eHACK 2026 · **Problem Statement #2 — Automotive**
Vasavi College of Engineering (Autonomous), Hyderabad

| Roll No. | Name |
|---|---|
| 1602-23-735-073 | B BALU |
| 1602-23-735-114 | B S S PRASAD |

---

## 1. What this is

A hard real-time Autonomous Emergency Braking controller running on **QNX
Neutrino RTOS 8.0** on a **Raspberry Pi 5**. A mmWave radar measures the range
and closing speed of the obstacle ahead; a collision-prediction model decides
how hard to brake; two motor drivers actuate four wheels; an IMU verifies the
braking physically took effect; and a watchdog forces emergency braking if any
critical task stops responding.

The braking maths would run on any computer. What QNX provides is the
guarantee that it runs **in time, every cycle** — and the controller *measures*
that guarantee rather than assuming it.

```
radar ──▶ collision prediction ──▶ brake controller ──▶ motors
  │               │                       │
  └───────────────┴──── heartbeats ───────┴──▶ watchdog ──▶ emergency pulse
                                                   ▲
                                          IMU brake verification
```

---

## 2. Package contents

```
PRIORITY_ZERO_AEB/
├── README.md                    this file
├── REQUIREMENTS.md              requirement-by-requirement compliance map
├── HARDWARE.md                  every wire, component by component
├── Makefile                     builds both binaries
├── src/
│   └── aeb.c                    the controller  (single file, 8 threads)
├── tools/
│   └── hwtest.c                 hardware bring-up tester (separate program)
├── dashboard/
│   └── aeb_dashboard.py         optional live dashboard (stdlib only)
└── docs/
    ├── chassis_layout.png       component layout on the 4-wheel chassis
    ├── chassis_layout.svg       same, vector
    └── dashboard_preview.png    dashboard screenshot
```

`src/aeb.c` is the submission. `tools/hwtest.c` is a **separate diagnostic
program** used during bring-up — it is not part of the controller and shares
no code with it, only the same pin map.

---

## 3. Build and run

### Build (host with QNX SDP 8.0)

```sh
source ~/qnx800/qnxsdp-env.sh
make                              # builds bin/aeb and bin/hwtest
make deploy TARGET=<board-ip>     # optional: scp both to the board
```

Or directly:

```sh
qcc -Vgcc_ntoaarch64le -O2 -Wall -std=gnu11 -D_QNX_SOURCE src/aeb.c -o aeb -lm
qcc -Vgcc_ntoaarch64le -O2 -Wall -std=gnu11 -D_QNX_SOURCE tools/hwtest.c -o hwtest -lm
```

### Run on the target

```sh
./hwtest      # verify the hardware first, component by component
./aeb         # then the controller
```

### CLI (mandatory interface)

| Command | Action |
|---|---|
| `v <mps>` | set vehicle speed |
| `o <m>` | set obstacle distance (bench mode) |
| `sf <x>` | set safety factor |
| `ttc <s>` | set critical time-to-collision threshold |
| `a <p s f>` | set the three deceleration levels (m/s²) |
| `e` | run a worked braking example |
| `s` | status — speed, state, level, parameters, IMU |
| `t` | collision timeline |
| `l` | latency statistics vs the 50 ms deadline |
| `i` | IMU reading |
| `h` | help |
| `q` | quit, leaving motors safe |

**Bench demo without a sensor attached:** `v 10` then `o 8`, then watch `s`,
`t` and `l`. The full decision path, state machine, watchdog and latency
measurement all run; only the radar input is substituted.

---

## 4. Architecture

### Threads (one process, SCHED_FIFO)

| Task | Tier (per problem statement) | Priority | Core |
|---|---|---|---|
| `watchdog_task` | safety supervisor | 62 | 3 |
| `predict_task` | **Highest** | 58 | 1 |
| `brake_task` | **Highest** | 58 | 0 |
| `pwm_task` | brake-actuation helper | 57 | 0 |
| `radar_task` | **High** (Sensor Task) | 50 | 2 |
| `imu_task` | **High** (sensor tier) | 50 | 2 |
| `vehicle_task` | **Medium** (Vehicle State) | 30 | 2 |
| `logger_task` | Low | 10 | 2 |

Prediction and braking share the same *Highest* tier, as the problem statement
specifies, and run on separate cores so they execute in parallel. The watchdog
sits strictly above both so it can always preempt the brake task it supervises.

### Inter-task communication

Synchronous **QNX message passing** on the data path:

```
radar_task --MsgSend--> predict_task --MsgSend--> brake_task
```

`MsgSend` blocks the sender until the receiver replies, so exactly one sensor
reading is ever in flight — the pipeline is self-regulating and needs no
shared-memory locks.

The watchdog uses an **asynchronous pulse** instead. A blocking message to a
hung brake task would hang the watchdog too; a pulse never waits, so the
supervisor cannot be taken down by the failure it exists to catch.

### Braking model

```
D = v·t_delay + v²/(2a)                      stopping distance
t_delay = 80 ms   (20 sensor + 10 software + 20 driver + 30 mechanical)
safety factor = 1.3
```

Graduated ladder, with the SF-inflated stopping distance at each level:

| Gap | Level | Deceleration | State |
|---|---|---|---|
| > d_partial | NONE | 0 | SAFE |
| d_strong … d_partial | PARTIAL | 2.0 m/s² | WARNING |
| d_full … d_strong | STRONG | 4.5 m/s² | STRONG |
| ≤ d_full | FULL | 7.5 m/s² | EMERGENCY |

Two overrides force FULL braking: time-to-collision below 0.6 s, and a
negative safety margin (the vehicle cannot stop even at maximum braking).
The system always fails toward *more* braking, never toward silence.

### Deadline

Every sensor reading carries a `CLOCK_MONOTONIC` timestamp. When the brake
task actuates, it computes the actual sense-to-brake latency and stores it in
a 128-sample history. The `l` command reports min / max / average and the
number of deadline misses against the 50 ms budget.

A system can have perfect priorities and still miss deadlines. Measurement is
what proves it does not.

---

## 5. Hardware

| Component | Role |
|---|---|
| Raspberry Pi 5 | ECU — runs QNX and the controller |
| RD-03D 24 GHz mmWave radar | range + Doppler closing speed, UART @ 256000 |
| MPU-6050 IMU | measured deceleration, brake-fault and impact detection, I²C |
| 2 × L298N motor driver | four-wheel actuation |
| 4 × DC gear motor | drive and dynamic braking |
| 4×AA pack (6 V) | motor power only |
| USB power bank (5 V/3 A) | Raspberry Pi power — separate domain |
| SPST rocker switch | hardware motor-power cutoff |
| 22 kΩ resistor | pull-down on the interlock sense input |

Full pin-by-pin wiring is in **HARDWARE.md**; the physical layout is in
`docs/chassis_layout.png`.

A note on the radar: the RD-03D is a 24 GHz module, not an automotive part.
It is used here as a functional stand-in for a production 77 GHz radar because
it supplies the same measurement primitives — range and Doppler closing
velocity — that a real AEB controller consumes. Moving to an automotive sensor
would change the driver layer only, not the safety architecture.

---

## 6. Optional dashboard

`dashboard/aeb_dashboard.py` is a live telemetry dashboard. It uses **only the
Python standard library** (tkinter) — no pip installs.

```sh
sudo apt install python3-tk          # once, on Debian/Ubuntu
python3 dashboard/aeb_dashboard.py               # demo mode, no hardware
python3 dashboard/aeb_dashboard.py --csv aeb_log.csv
python3 dashboard/aeb_dashboard.py --udp 5005
```

It shows a live road view with the braking zones scaled to current speed, a
radar scope, speed gauge, brake ladder, IMU commanded-vs-measured comparison,
the deadline panel and the collision timeline. `SPACE` pauses, `R` restarts,
`F` injects a brake fault to demonstrate the IMU safety check.

The dashboard's decision function is a direct port of `decide()` in `aeb.c`
and produces identical results, so demo mode is faithful to the controller.

---

## 7. Known limitations

Stated plainly, because they are engineering facts rather than oversights.

- **Pi 5 GPIO.** The Pi 5 routes GPIO through the RP1 south-bridge rather than
  the classic BCM memory map. The GPIO block in `aeb.c` is written against the
  classic layout (correct on Pi 4-class BSPs) with every Pi-5 change point
  marked `***RP1***`. It must be bound to the target BSP's GPIO window.
- **Device names.** `/dev/ser*` and `/dev/i2c*` names vary by QNX image. The
  UART open tries several candidates and reports which succeeded; `hwtest`
  test 0 lists what the image actually publishes.
- **Radar firmware.** RD-03D frame formats vary between firmware revisions.
  The parser implements the documented multi-target frame; `hwtest` tests 6
  and 7 verify it against the physical unit.
- **Camera.** Not part of this submission. An earlier iteration included a
  camera as a sensor-fusion confirmation guard; it was removed so that radar
  is unambiguously the single primary sensor.

---

## 8. Verification performed

| Check | Result |
|---|---|
| `aeb.c` compiles with `-Wall -Wextra` | clean, no warnings |
| `hwtest.c` compiles with `-Wall -Wextra` | clean, no warnings |
| Braking ladder across 6 distances | correct at every band |
| Clear road (50 m @ 10 m/s) | NONE — no false braking |
| Stationary vehicle, 2 m obstacle | NONE — no false braking |
| Close obstacle (5 m @ 10 m/s) | FULL + EMERGENCY |
| RD-03D frame decode (X=1000, Y=2000, v=150) | 2.236 m, 1.50 m/s — exact |
| IMU conversion (−1.8 g) | 17.65 m/s² |
| Brake-fault threshold | fires below 50 % of commanded, quiet above |
