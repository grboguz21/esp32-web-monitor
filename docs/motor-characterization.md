# Motor 1 characterization

Measured on 22 Oct 2026. These are the numbers the speed controller is built on; if the
hardware changes, re-run the tools below and compare against this page.

## Setup

| | |
|---|---|
| Motor | GA25-370, 12 V, 1350 RPM, Hall encoder (black GND, blue VCC, yellow A, green B) |
| Driver | L298N, one per motor, channel A only |
| Supply | 12 V adapter |
| PWM | 1 kHz, 10-bit, on IN1 (+) / IN2 (-) with ENA held HIGH (slow decay) |
| Pins | ENA D25, IN1 D26, IN2 D27, encoder A D34, B D35 (encoder powered from 3V3) |
| Firmware | 0.13.0 to 0.18.1 |

**Volts are commanded volts: duty x 12 V.** The L298N drops roughly 2 V on top of that, so
the motor sees less. That loss is folded into kS and the curve below; keep using the same
units everywhere and it cancels out.

## Encoder

**CPR = 186.2 counts per output-shaft revolution** (4x quadrature).

Measured by turning the output shaft 10 turns by hand, twice: 1863 and 1860 counts. That
matches an 11 PPR encoder (44 counts per motor turn) behind a ~1:4.23 gearbox, so CPR is not
an integer; measure over several turns.

## Start and stop thresholds

Duty ramped slowly (30 s ramps) on the dashboard's threshold card.

| | + | - |
|---|---|---|
| Start (from rest) | 19.7 % (2.36 V) | 20.8 % (2.50 V) |
| Stop (ramping down) | 11.6 % (1.39 V) | 9.6 % (1.15 V) |

Static friction is about twice the kinetic friction (start/stop ratio 1.7 for +, 2.2 for -).
**kS+ = 1.39 V, kS- = 1.15 V** (the stop thresholds, since the controller mostly works on a
turning motor). Below about 50 RPM the motor sticks and lurches; that range is not reliably
controllable with this hardware.

## Steady-state speed vs duty

Data: [`data/motor1_steady_state.csv`](data/motor1_steady_state.csv), from
[`tools/kv_sweep.py`](../tools/kv_sweep.py). 2 s settle, then 1.5 s average per point; spread
about ±3 RPM.

| Duty | Volts | + RPM | - RPM |
|---|---|---|---|
| 100 % | 12.00 | 1115 | -1127 |
| 80 % | 9.60 | 893 | -906 |
| 60 % | 7.20 | 675 | -690 |
| 50 % | 6.00 | 559 | -574 |
| 40 % | 4.80 | 441 | -454 |
| 30 % | 3.60 | 321 | -331 |
| 25 % | 3.00 | 257 | -268 |
| 20 % | 2.40 | 191 | -199 |
| 17 % | 2.04 | 146 | -154 |
| 15 % | 1.80 | 110 | -118 |
| 13 % | 1.56 | 72 | -78 |
| 12 % | 1.44 | 50 | -57 |
| 11 % | 1.32 | 0 (stopped) | -25 |
| 10 % | 1.20 | | 0 (stopped) |

The curve is not one straight line: the slope is about 117 RPM/V between 12 and 40 % and
about 94 RPM/V between 30 and 100 %. A single `V = kS + kV*w` fit misses by 0.2 to 0.45 V,
so **the feedforward interpolates this table** (per direction) instead. For reference:

| Fit range | kV (V per rad/s) | kS (V) |
|---|---|---|
| 30-100 %, + | 0.1013 | 0.12 |
| 30-100 %, - | 0.1013 | -0.02 |
| 12-40 %, + | 0.0823 | 0.87 |
| 12-40 %, - | 0.0810 | 0.82 |

The - direction runs about 10 RPM faster at the same duty and stops at a lower duty.

## Step response

Data: [`data/motor1_step_results.csv`](data/motor1_step_results.csv) and the 200 Hz raw log
[`data/motor1_step_raw.csv`](data/motor1_step_raw.csv), from
[`tools/step_test.py`](../tools/step_test.py). Steps start from a turning motor so static
friction stays out; a first-order-plus-dead-time model is fitted to each.

| Step | tau | K |
|---|---|---|
| +30 -> +60 % | 114-119 ms | 97 RPM/V |
| +60 -> +30 % | 129-131 ms | 97.5 RPM/V |
| +20 -> +40 % | 133-136 ms | 103 RPM/V |
| -30 -> -60 % | 121-129 ms | 100-101 RPM/V |
| -60 -> -30 % | 133 ms | 100 RPM/V |
| -20 -> -40 % | 137-139 ms | 105.5 RPM/V |

- **tau ~ 125 ms**, **K ~ 100 RPM/V** (agrees with the steady-state slope)
- **theta ~ 0-2 ms** in the motor itself. The controller's 20 ms backward-difference speed
  estimate and the 5 ms tick add about 15 ms, which is what the PI design uses.
- **kA = tau / K ~ 0.00125 V per RPM/s** (0.012 V per rad/s²)

Control loop timing over 4313 ticks: period 5000 µs, min 4998, max 5002, no missed ticks.

## Speed controller

All in the 200 Hz tick on core 1 (firmware 0.15.0):

```
reference:    w_ref += (w_target - w_ref) / tau_ref * dt
feedforward:  V_ff = table(w_ref) + kA * dw_ref/dt
PI:           V = V_ff + Kp * (w_ref - w) + Ki * integral(w_ref - w)
anti-windup:  don't integrate while |V| > 12 V and the error pushes further out
```

The integral resets when the motor, mode or direction changes; the reference starts from the
measured speed.

**Battery compensation (firmware 0.20.0).** The controller works in volts and divides by the
supply voltage to get the duty. With an INA219 on the supply line it uses that reading
(low-passed, 0.2 s); without one it assumes 12 V, which is what everything here was measured
at. The gains do not change. What the battery does change is the top speed: the L298N drops
about 2 V, so a 3S pack at 10.5 V leaves roughly 8.5 V for the motor (about 850 RPM at full
duty). Keep targets under about 700 RPM so the controller has room on a low pack.

PI gains from SIMC with K = 100 RPM/V, tau = 0.125 s, theta = 0.015 s:
`Kp = tau / (K (tau_c + theta))`, `Ti = min(tau, 4 (tau_c + theta))`, `Ki = Kp / Ti`

| Setting | tau_c | Kp (V/RPM) | Ki (V/(RPM·s)) |
|---|---|---|---|
| Calm | 125 ms | 0.0089 | 0.071 |
| Medium | 50 ms | 0.0192 | 0.154 |
| **Aggressive (default)** | 15 ms | **0.0417** | **0.333** |

Defaults: **tau_ref = 100 ms, kA = 0.00125, Kp = 0.0417, Ki = 0.333** (firmware 0.17.0).

The aggressive set was picked in a hand-brake test: with the reference model carrying target
changes, higher PI gains no longer cause overshoot on a step, and they cut both the dip while
braking and the jump when the brake is released. With the calm gains, about 2.5 V built up in
the integral while braked and threw the motor to ~550 RPM (target 300) on release. The cost is
that the ±8 RPM speed ripple shows up as about ±0.3 V on the output.

**Motor profiles (firmware 0.21.0).** Everything above that belongs to one motor (CPR,
direction, start thresholds, gains, top speed and both feedforward curves) is a per-motor
profile kept in the ESP32's flash. The built-in defaults are motor 1's numbers. The dashboard
reads the profiles on connect and saves changes back; `PROFILE?`, `KICK`, `MAXRPM`, `FF`,
`SAVE` and `DEFAULTS` do the same over serial (see the protocol comment in `src/main.cpp`).

## Validation

Data: [`data/motor1_speed_control_raw.csv`](data/motor1_speed_control_raw.csv), from
[`tools/speed_test.py`](../tools/speed_test.py). Settling is to 5 % on a 50 ms average. These
runs used the calm gains (firmware 0.15.0), before the breakaway handling of 0.16.0.

Without the reference model (PI on target minus speed), feedforward and PI both pushed on a
step: 12-36 % overshoot and 1-2 s settling. With it:

| Step (FF + PI) | Overshoot | Settling | Steady-state error |
|---|---|---|---|
| 0 -> 300 RPM | 2.0 % | 330 ms | +0.7 RPM |
| 300 -> 600 RPM | 1.9 % | 295 ms | -0.6 RPM |
| 600 -> 300 RPM | 1.2 % | 275 ms | +0.1 RPM |
| 300 -> 150 RPM | 5.5 % | 485 ms | -0.3 RPM |
| 150 -> -400 RPM | 1.0 % | 340 ms | +1.0 RPM |

Rise time is about 190 ms, as expected for tau_ref = 100 ms; a smaller tau_ref is faster.
Feedforward alone lands 7-12 RPM short of the target.

Disturbance (shaft braked by hand at 300 RPM): with FF + PI the speed dipped to the low 200s
and came back to 300 while still braked, with the duty rising to about 70 %. With feedforward
only the motor stopped. Releasing the brake with the calm gains overshot to ~550 RPM; the
aggressive gains brought that down clearly (judged on the dashboard chart).

Breakaway (firmware 0.16.0): while the motor is not turning in the target's direction (below
30 RPM), the reference holds at the measured speed, the integral stays empty and a kick rises
from the start threshold (+40 %/s, max 60 %) until it moves, so it does not lurch once free.

Speed ripple is about ±8 RPM: one encoder count in the 20 ms window is about 16 RPM.

## Final check and distance moves

Data: [`data/motor1_final_check_raw.csv`](data/motor1_final_check_raw.csv), from
[`tools/final_check.py`](../tools/final_check.py), default gains, firmware 0.18.1.

| Step (FF + PI) | Overshoot | Settling | Steady-state error |
|---|---|---|---|
| 0 -> 300 RPM (from rest) | 7.4 % | 350 ms | 0.0 RPM |
| 300 -> 600 RPM | 4.1 % | 365 ms | -0.1 RPM |
| 600 -> 300 RPM | 3.3 % | 300 ms | +0.2 RPM |
| 300 -> 150 RPM | 14.1 % (21 RPM) | 245 ms | 0.0 RPM |
| 150 -> -400 RPM | 3.4 % | 325 ms | +0.4 RPM |

The aggressive gains overshoot a bit more than the calm ones; on the small 300 -> 150 step
the 21 RPM peak is about one count of speed ripple.

Low speed, started from rest: ±50, ±80 and ±120 RPM all hold their mean to 0.1 RPM with
±8-9 RPM ripple (100 ms averages stay within about 20 RPM of the target). Firmware 0.18.0
could not hold -50 RPM (it sat at -29): at low speed the estimate jumps in 16 RPM steps,
one reading of 16 RPM counted as stuck and reset the reference. Since 0.18.1 the stuck
check has hysteresis: free above 30 RPM (or half the target), stuck again only after 40 ms
without turning.

Ramp 0 -> 600 RPM in 2 s: the speed follows the reference to 0.0 ±8 RPM; the reference
itself runs 30 RPM behind the command (ramp rate × tau_ref), which is by design.

**Distance moves** (`MOVE` command, dashboard "Distance test" card): trapezoidal speed
profile (accelerate, cruise, slow down so it can stop in the distance left, 40 RPM final
approach) through the speed loop with the profile as the reference, then 300 ms of
short-circuit braking (both inputs low, ENA high).

| Move | Max speed, accel | Time | Error at stop |
|---|---|---|---|
| +5 turns | 300 RPM, 1500 RPM/s | 1.26 s | 0 counts |
| -5 turns | 300 RPM, 1500 RPM/s | 1.16 s | 0 counts |
| +1 turn | 300 RPM, 1500 RPM/s | 0.37 s | -3 counts (6°) |
| +0.25 turn | 200 RPM, 1500 RPM/s | 0.15 s | -1 count (2°) |
| +5 turns | 600 RPM, 3000 RPM/s | 0.69 s | -1 count (2°) |

Within 3 counts (6° of the output shaft) every time, and nothing creeps after the brake.
All of this is with the wheel in the air.

## Re-running

The tools talk to the firmware over USB (460800 baud) the same way the dashboard does, so
disconnect the dashboard first. They need pyserial (PlatformIO's Python has it):

```bash
~/.platformio/penv/bin/python tools/kv_sweep.py     # steady-state curve, ~1.5 min
~/.platformio/penv/bin/python tools/step_test.py    # step responses, ~1.5 min
~/.platformio/penv/bin/python tools/speed_test.py   # controller check, ~25 s
~/.platformio/penv/bin/python tools/final_check.py  # steps, low speed, ramp, moves, ~1.5 min
```

Each writes its CSV next to the script. The motor spins in both directions up to full speed:
keep it free to turn.
