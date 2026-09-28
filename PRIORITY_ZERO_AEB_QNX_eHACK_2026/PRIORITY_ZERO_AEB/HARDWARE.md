# Hardware Wiring

**Team PRIORITY ZERO** · AEB Controller · Raspberry Pi 5 + QNX

Every connection, component by component. Pin numbers are given as **BCM GPIO**
and **physical header pin**, because the code uses BCM numbers while you count
physical pins on the board.

Physical layout: `docs/chassis_layout.png`.

---

## 1. Raspberry Pi 5 — every pin used

| BCM | Phys | Connects to |
|---|---|---|
| — | **1** (3.3 V) | MPU-6050 VCC |
| GPIO2 (SDA1) | **3** | MPU-6050 SDA |
| GPIO3 (SCL1) | **5** | MPU-6050 SCL |
| — | **2** or **4** (5 V) | RD-03D radar VCC |
| — | **6** (GND) | star ground |
| GPIO14 (TXD) | **8** | radar RX |
| GPIO15 (RXD) | **10** | radar TX |
| GPIO17 | **11** | L298N #2 IN1 |
| GPIO18 | **12** | L298N #2 ENA |
| GPIO27 | **13** | L298N #2 IN2 |
| GPIO22 | **15** | L298N #2 IN3 |
| GPIO23 | **16** | L298N #2 IN4 |
| GPIO24 | **18** | 22 kΩ resistor → GND pin 6 |
| GPIO5 | **29** | L298N #1 IN1 |
| GPIO6 | **31** | L298N #1 IN2 |
| GPIO12 | **32** | L298N #1 ENA |
| GPIO13 | **33** | L298N #1 ENB |
| GPIO19 | **35** | L298N #2 ENB |
| GPIO16 | **36** | L298N #1 IN3 |
| GPIO26 | **37** | L298N #1 IN4 |

**Power inlet:** USB-C ← power bank, 5 V / 3 A minimum.
There are 17 signal connections plus 3 power/ground pins. No pin conflicts.

---

## 2. L298N #1 — FRONT axle

**Screw terminals**

| Terminal | Connects to |
|---|---|
| 12V (Vs) | switched +6 V from the master switch |
| GND | star ground |
| 5V | **leave unconnected** |

**Signal pins**

| Pin | From | Phys |
|---|---|---|
| ENA | GPIO12 | 32 |
| IN1 | GPIO5 | 29 |
| IN2 | GPIO6 | 31 |
| IN3 | GPIO16 | 36 |
| IN4 | GPIO26 | 37 |
| ENB | GPIO13 | 33 |

**Motor outputs**

| Output pair | Motor |
|---|---|
| OUT1 / OUT2 | M1 front-left |
| OUT3 / OUT4 | M2 front-right |

**Jumpers:** remove the ENA cap, remove the ENB cap (software PWM drives those
pins). Leave the onboard 5 V-regulator jumper fitted.

---

## 3. L298N #2 — REAR axle

**Screw terminals**

| Terminal | Connects to |
|---|---|
| 12V (Vs) | switched +6 V from the master switch |
| GND | star ground |
| 5V | **leave unconnected** |

**Signal pins**

| Pin | From | Phys |
|---|---|---|
| ENA | GPIO18 | 12 |
| IN1 | GPIO17 | 11 |
| IN2 | GPIO27 | 13 |
| IN3 | GPIO22 | 15 |
| IN4 | GPIO23 | 16 |
| ENB | GPIO19 | 35 |

**Motor outputs**

| Output pair | Motor |
|---|---|
| OUT1 / OUT2 | M3 rear-left |
| OUT3 / OUT4 | M4 rear-right |

**Jumpers:** remove ENA and ENB caps; keep the 5 V-regulator jumper fitted.

---

## 4. Gear motors

| Motor | Position | Driver / outputs | Code wheel index |
|---|---|---|---|
| M1 | front-left | #1 OUT1 + OUT2 | 0 |
| M2 | front-right | #1 OUT3 + OUT4 | 1 |
| M3 | rear-left | #2 OUT1 + OUT2 | 2 |
| M4 | rear-right | #2 OUT3 + OUT4 | 3 |

Two wires each, no polarity at wiring time. If a wheel spins the wrong way
during `hwtest` test 3, swap that motor's two OUT wires — no code change.
Left and right motors face opposite directions, so one whole side normally
needs swapping; that is expected, not a fault.

**Direction truth table** (per channel, as driven by `apply_level()`):

| Action | INx | INy | EN duty |
|---|---|---|---|
| Forward (SAFE) | HIGH | LOW | 60 % |
| Partial brake | LOW | LOW | 40 % |
| Strong brake | LOW | LOW | 70 % |
| Full brake | LOW | LOW | 100 % |
| Reverse (unused) | LOW | HIGH | — |

Both inputs low is a short brake, which is why braking is active rather than a
coast.

---

## 5. RD-03D mmWave radar

| Radar pin | Connects to | Phys |
|---|---|---|
| VCC | Pi 5 V | 2 or 4 |
| GND | star ground | — |
| RX | Pi GPIO14 / TXD | 8 |
| TX | Pi GPIO15 / RXD | 10 |

TX and RX **cross over** — the radar's receive line goes to the Pi's transmit.
UART at 256000 baud, multi-target mode. The code tries `/dev/ser1` through
`/dev/ser4` and reports which opened.

---

## 6. MPU-6050 IMU

| IMU pin | Connects to | Phys |
|---|---|---|
| VCC | Pi **3.3 V** | **1** |
| GND | star ground | — |
| SDA | Pi GPIO2 / SDA1 | 3 |
| SCL | Pi GPIO3 / SCL1 | 5 |
| AD0 | GND | — |
| INT, XDA, XCL | not connected | — |

**Power this from 3.3 V, not 5 V.** The breakout's I²C pull-up resistors
reference VCC; at 5 V they would pull SDA and SCL toward 5 V and over-drive the
Pi's 3.3 V-only GPIO.

AD0 tied low sets the I²C address to **0x68**.

**Orientation:** mount the board flat with the **+X axis pointing forward**.
This matches `decel = -ax` in `imu_task()`. If braking shows a positive
deceleration on the vehicle, flip the sign on that one line.

---

## 7. Master switch (SPST rocker, 2 terminals)

| Terminal | Connects to |
|---|---|
| 1 | 4×AA pack **+6 V** |
| 2 | splits to **both L298N 12V (Vs)** terminals |

Either terminal takes either wire. Position `I` = on. The switch never touches
the Pi — it is a pure hardware cutoff for motor power, independent of software.

**On the interlock:** the controller reads GPIO24 to report whether the system
is armed. That sense line is designed for a DPDT switch with a second pole. With
a 2-terminal SPST there is no second pole, so GPIO24 is held low through the
pull-down below and the CLI honestly reports `interlock=open` at all times. The
hardware cutoff is unaffected — software cannot re-energise the motors when the
switch is open.

---

## 8. 4×AA battery pack (6 V)

| Wire | Connects to |
|---|---|
| Red (+) | master switch terminal 1 |
| Black (−) | star ground |

Motor power only. Never connected to the Raspberry Pi.

Note: the L298N drops roughly 1.4–2 V, so the motors see about 4–4.6 V, and
this falls as the cells drain. A 2S Li-ion pack (7.4 V) gives more headroom if
the motors become sluggish.

---

## 9. Power bank

| | |
|---|---|
| USB-C output | Raspberry Pi 5 USB-C input |

5 V, 3 A minimum. This is the Pi's only power source. Keeping it on a separate
domain from the motors prevents motor switching noise from browning out the Pi.

---

## 10. Pull-down resistor (22 kΩ)

| Leg | Connects to |
|---|---|
| one | GPIO24 — phys pin **18** |
| other | GND — phys pin **6** |

No polarity. Without it the unused interlock input floats and reads randomly;
with it the pin reads a stable low. Any value from 4.7 kΩ to 100 kΩ works.

---

## 11. Star ground — one junction, seven wires

Every ground meets at a **single physical point** (terminal block or solder
junction). Do not daisy-chain.

1. Battery pack **−**
2. L298N #1 **GND**
3. L298N #2 **GND**
4. Raspberry Pi **GND** (pin 6)
5. RD-03D **GND**
6. MPU-6050 **GND**
7. 22 kΩ resistor return

This is the single most common cause of erratic behaviour if done wrong.

---

## Pre-power checklist

- [ ] Four jumper caps removed — ENA and ENB on **both** L298N boards
- [ ] Both onboard 5 V-regulator jumpers still fitted
- [ ] Nothing connected to either L298N **5V** terminal
- [ ] IMU on **3.3 V** (pin 1); radar on **5 V** (pin 2/4)
- [ ] Radar TX/RX **crossed**
- [ ] 22 kΩ from GPIO24 to GND fitted
- [ ] All seven grounds meet at the star point
- [ ] Master switch **OFF** before first boot
- [ ] Wheels **off the ground** for all motor tests

Then run `./hwtest` and work through tests 0 to 9 in order before running
`./aeb`.
