# Requirements Compliance

**Team PRIORITY ZERO** · QNX eHACK 2026 · Problem Statement #2

Every requirement from the problem-statement sheet, mapped to the code that
implements it. Line numbers refer to `src/aeb.c`.

---

## Objective

> *Detect imminent collision and trigger braking within a deterministic
> response deadline.*

Collision detection is in `decide()`; the deadline is **measured on every
cycle**, not assumed. Each sensor reading carries a `CLOCK_MONOTONIC`
timestamp; `brake_task` computes the sense-to-actuation latency when it
actuates and records it. The `l` CLI command reports min / max / average and
the count of misses against the 50 ms budget.

---

## Platform

> *QNX on Raspberry Pi 4/5*

QNX Neutrino RTOS 8.0 on Raspberry Pi 5 (aarch64). Builds with `qcc`.

---

## Tasks and scheduling

| Required task | Tier | Implementation | Priority |
|---|---|---|---|
| Sensor Task | High | `radar_task` | 50 |
| Collision Prediction | **Highest** | `predict_task` | 58 |
| Brake Controller | **Highest** | `brake_task` | 58 |
| Vehicle State | Medium | `vehicle_task` | 30 |
| Logger | — | `logger_task` | 10 |

Prediction and braking are at the **same** priority, as the sheet specifies,
and are pinned to **different cores** so they run in parallel rather than
contending.

Three additional threads support the above:

| Thread | Priority | Why it exists |
|---|---|---|
| `watchdog_task` | 62 | required by *RTOS Concepts: Watchdog* and *Timers: Safety Watchdog*; placed above the Highest tier so it can preempt the brake task it supervises |
| `imu_task` | 50 | optional hardware named in the sheet; closes the loop on the actuator |
| `pwm_task` | 57 | motor enable signal generation — an implementation detail of the brake controller, isolated so its 1 kHz timing cannot disturb the decision logic |

Core affinity (`ThreadCtl` runmask):

| Core | Threads |
|---|---|
| 0 | brake, pwm — safety actuation |
| 1 | predict — collision prediction |
| 2 | radar, imu, vehicle, logger — sensing and housekeeping |
| 3 | watchdog — supervision |

---

## RTOS concepts

### Deadline Scheduling

`BRAKING_DEADLINE_US` = 50 000 µs. Latency is sampled every actuation into a
128-entry ring, with min / max / mean and a miss counter exposed through `l`.
The deadline is therefore a *measured* property of the running system.

### Priority Preemption

All threads run `SCHED_FIFO` via `pthread_setschedparam`. A higher-priority
thread preempts a lower one immediately — no time slicing. The concrete chain:
the logger can be interrupted by the radar, which is interrupted by
prediction, which is interrupted by braking, which the watchdog can interrupt.

### State Machines

Four states — `ST_SAFE`, `ST_WARNING`, `ST_STRONG`, `ST_EMERGENCY` — mapped to
four braking levels. The machine always escalates toward more braking and
never toward silence; two independent overrides (critical TTC, negative safety
margin) force the EMERGENCY state directly.

### Watchdog

Every critical task emits a heartbeat pulse each cycle. The watchdog checks
liveness every 10 ms; if any critical task is silent for more than 120 ms it
is presumed dead and an emergency pulse is sent straight to the brake task,
bypassing the normal pipeline.

---

## Inter-task communication

> *Message Passing*

Native QNX synchronous message passing on the data path:

```
radar_task  --MsgSend-->  predict_task  --MsgSend-->  brake_task
                          MsgReceive/MsgReply
```

`MsgSend` blocks the sender until the receiver replies, so exactly **one**
reading is in flight at any moment. The pipeline is self-regulating: the radar
cannot queue stale readings ahead of the controller, and no shared-memory
locking is required anywhere on the data path.

The watchdog path deliberately uses an **asynchronous pulse**
(`MsgSendPulse`). If the watchdog used a blocking message and the brake task
were the hung task, the watchdog would block too — the supervisor destroyed by
the failure it exists to catch. A pulse never waits.

---

## Timers

| Required timer | Implementation | Period |
|---|---|---|
| Sensor Timer | `timer_create` + pulse in `radar_task` | 20 ms (50 Hz) |
| Braking Deadline | timestamp difference measured in `brake_task` | 50 ms budget |
| Safety Watchdog | `timer_create` + pulse in `watchdog_task` | 10 ms tick, 120 ms timeout |

All timers use `CLOCK_MONOTONIC`, which never steps backward. Threads waiting
on a timer pulse consume no CPU.

---

## Logging and visualisation

| Required output | Implementation |
|---|---|
| Collision Timeline | `timeline_print()` — `t` command; 16-event history with state, level, distance, TTC, margin and latency |
| Response Latency | `lat_stats()` — `l` command; min / max / mean / misses over 128 samples |
| Vehicle State | `cli_status()` — `s` command; speed, obstacle, state, level, deceleration, parameters, IMU |

Critical tasks never call `printf`. They push fixed-size records into a ring
buffer; the lowest-priority logger thread drains it to the console and to
`aeb_log.csv`. Instrumentation therefore cannot delay a braking decision.

---

## Interface

> *CLI Mandatory + Optional Python/Qt/Web Dashboard*

**CLI** — 12 commands, listed in README §3. Every model parameter (speed,
obstacle, safety factor, TTC threshold, all three deceleration levels) is
tunable live, so behaviour can be changed on demand without recompiling.

**Optional dashboard** — `dashboard/aeb_dashboard.py`, Python standard library
only. Live road view with speed-scaled braking zones, radar scope, brake
ladder, IMU commanded-vs-measured comparison, deadline panel and timeline.

---

## Optional hardware

> *Ultrasonic Sensor, IMU, Motor Driver, GPIO/PWM*

| Listed | Used |
|---|---|
| Ultrasonic sensor | **RD-03D 24 GHz mmWave radar** instead |
| IMU | MPU-6050 over I²C — deceleration feedback, brake-fault detection, impact detection |
| Motor Driver | 2 × L298N driving four wheels |
| GPIO/PWM | 12 GPIO for direction and enable, 1 kHz software PWM |

Radar was chosen over ultrasonic because AEB is fundamentally a *velocity*
problem. The stopping-distance equation contains `v²`, so velocity error is
squared in the result. Radar reports closing speed directly through the
Doppler shift; an ultrasonic sensor would require differentiating noisy range
samples, and the necessary filtering would add lag to the very reaction budget
the controller is trying to protect. This is also why production AEB is
radar-based and ultrasonic is reserved for low-speed parking assistance.

---

## Project description

> *Simulate vehicle movement and dynamically calculate collision risk.
> Emergency braking must preempt non-critical workloads and execute within a
> specified deadline.*

`vehicle_task` closes the simulation loop at 100 Hz, integrating speed under
the commanded deceleration and updating the gap — so the system is a true
feedback loop rather than a one-shot calculation: sense → decide → act →
vehicle responds → the changed gap becomes the next reading.

Collision risk is recomputed from scratch every 20 ms cycle, so the model
self-corrects continuously.

Preemption of non-critical work is structural: the logger runs at priority 10
and the vehicle model at 30, while braking runs at 58 on a dedicated core. A
busy logger cannot delay a brake command, and the measured latency figures
demonstrate this rather than merely asserting it.
