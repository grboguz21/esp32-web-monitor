# ESP32 Motor Controller

> **Work in progress.** This branch builds on my [ESP32 Web Monitor](https://github.com/grboguz21/esp32-web-monitor/tree/main) draft.

Two 12 V encoder gear motors, driven and tuned from the browser over USB. No app to install: the dashboard is a single web page that talks to the ESP32 through Web Serial.

![PID tab](docs/pid.png)

## Tabs

**Dashboard.** Is the board alive? Connection status, RAM, loop time and stack use, plus the serial log.

**Power.** Voltage, current and power for each motor from an INA219, with live charts and the lowest voltage / highest current since the last reset.

![Power tab](docs/power.png)

**Encoder.** Live count and speed for each motor. Find the counts per revolution by turning the shaft by hand, and flip the direction if it counts backwards.

**Motor.** Give each motor a duty and watch PWM and RPM side by side. Handy for checking wiring and direction.

![Motor tab](docs/motor.png)

**PID.** Speed control with feedforward + PI. Change the target or the gains while it runs and see target, reference and measured speed on one chart. The **Distance test** below it turns the wheel a set number of turns or meters and brakes on target.

![Distance test](docs/distance.png)

**Board pinout.** The DevKit's pins with what each one can do. Click a pin for details.

![Board pinout](docs/board-pinout.png)

The Power and Motor shots use the built-in demo data. The PID and Distance shots are a recorded run of motor 1.

## Hardware

- DOIT ESP32 DevKit V1
- 2× GA25-370 12 V 1350 RPM gear motors with Hall encoders
- 2× L298N motor drivers, one per motor
- 2× INA219 current sensors (optional; bridge A0 on the second one for address 0x41)
- 12 V 2 A power supply

## Wiring

| | Motor 1 | Motor 2 |
|---|---|---|
| L298N ENA | D25 | D18 |
| L298N IN1 | D26 | D19 |
| L298N IN2 | D27 | D23 |
| Encoder A (yellow) | D34 | D16 |
| Encoder B (green) | D35 | D17 |
| Encoder VCC (blue) / GND (black) | 3V3 / GND | 3V3 / GND |

Pull the ENA jumper off the L298N. Both INA219s share 3V3, GND, SDA → D21 and SCL → D22. Tie the 12 V ground to an ESP32 GND pin.

The dashboard draws all of this too: **Show motor wiring** under Motor Control, **Show wiring** on the Power tab.

## Trying it out

1. Open this folder in VS Code with the PlatformIO extension and check out the `motor-controller` branch.
2. Plug the ESP32 in and click **Upload** (the → arrow in the bottom bar).
3. Open `dashboard/index.html` in Chrome or Edge.
4. Click **Add port…**, pick your board, then **Connect**.

No hardware yet? Hit **Demo data** on the Power tab. Before uploading new code, click **Disconnect** so the port is free.

Measurements and controller tuning for motor 1 are in [docs/motor-characterization.md](docs/motor-characterization.md).

## Next up

- characterize motor 2 the same way
- tests on the ground with the robot's weight
- drive both motors together (straight line, turns)

## License

MIT
