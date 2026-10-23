#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ESP32Encoder.h>
#include <esp_system.h>

constexpr uint32_t SERIAL_BAUD = 460800;   // room for the 200 Hz log next to the telemetry
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 500;
constexpr uint32_t HEALTH_INTERVAL_MS = 1000;

// Dashboard communication runs on core 0 so loop() keeps core 1 to itself
constexpr BaseType_t TELEMETRY_CORE = 0;
constexpr uint32_t TELEMETRY_STACK_BYTES = 6144;
constexpr UBaseType_t TELEMETRY_PRIORITY = 1;
constexpr uint32_t TELEMETRY_PERIOD_MS = 5;

constexpr const char* DEVICE_NAME = "ESP32";
// One INA219 per motor on the default I2C pins; both optional, detected at runtime.
// The motor 2 sensor has its A0 pads bridged, which moves it to address 0x41.
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t POWER_SENSOR_ADDRESSES[] = {0x40, 0x41};
constexpr size_t POWER_CHANNELS = sizeof(POWER_SENSOR_ADDRESSES);
constexpr uint32_t POWER_SAMPLE_MS = 10;      // 100 Hz, averaged into each report
constexpr uint32_t POWER_REPORT_MS = 100;
constexpr uint32_t POWER_ABSENT_REPORT_MS = 1000;
constexpr uint32_t SENSOR_PROBE_MS = 2000;

// One L298N per motor. PWM goes to IN1 (+ direction) or IN2 (- direction) with ENA held
// HIGH, so in the off phase the low-side transistors carry the motor current (slow decay)
// instead of the diodes. ENA LOW lets the motor coast; that is the stopped state.
struct MotorPins {
  int ena;
  int in1;
  int in2;
  int encA;
  int encB;
};
constexpr MotorPins MOTOR_PINS[] = {
  {25, 26, 27, 34, 35},   // motor 1 (GPIO34/35 have no internal pull-ups)
  {18, 19, 23, 16, 17},   // motor 2
};
constexpr size_t MOTOR_COUNT = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);
// The onboard LED can stand in for a motor to check the test without any wiring
constexpr int LED_PIN = 2;
constexpr unsigned LED_TARGET = MOTOR_COUNT + 1;   // RUN target number after the motors
constexpr int ALL_MOTORS = MOTOR_COUNT + 1;         // MotorRun.motor for DUTY: every motor at once
constexpr uint32_t PWM_FREQ_HZ = 1000;      // same as Arduino-ESP32's analogWrite() default
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1;
constexpr uint32_t CONTROL_PERIOD_US = 5000;  // fixed 200 Hz control tick on core 1
constexpr uint32_t MOTOR_WATCHDOG_MS = 1000;  // stop if the dashboard goes quiet this long
constexpr uint32_t MOTOR_REPORT_MS = 40;      // 25 Hz
constexpr uint32_t SPEED_WINDOW_MS = 50;      // encoder speed = count change over this window
constexpr size_t LOG_QUEUE_LENGTH = 256;      // 1.28 s of ticks buffered between the cores

// Speed control (200 Hz on core 1): feedforward from the measured steady-state curve plus
// a PI on the speed error, with clamping anti-windup. Volts are duty x SUPPLY_V.
constexpr float SUPPLY_V = 12.0f;
constexpr float DEFAULT_CPR = 186.2f;          // measured on motor 1 (counts per output rev)
constexpr size_t SPEED_TAPS = 4;               // controller speed: backward difference over 20 ms
constexpr float DEFAULT_KP = 0.0417f;          // V per RPM (SIMC, tau_c = theta = 15 ms)
constexpr float DEFAULT_KI = 0.333f;           // V per RPM*s; chosen in a hand-brake test
// Reference model: the target is followed by a reference speed with this time constant;
// feedforward drives the motor along the reference (with kA for the acceleration) and the
// PI only corrects the gap between reference and measured speed
constexpr float DEFAULT_TAU_REF_S = 0.100f;
constexpr float DEFAULT_KA = 0.00125f;         // V per RPM/s (tau / K from the step test)
// Breakaway: while the motor is stuck, the reference waits, the PI is off and a kick
// starting at the measured start threshold rises until the motor moves. Without this the
// integral builds up during the stall and the motor lurches past the target once free.
constexpr float STUCK_RPM = 30.0f;             // breaking free: moving at least this fast
constexpr float STILL_RPM = 8.0f;              // half a count per speed window: not turning
constexpr uint32_t STILL_MS = 40;              // not turning this long while running: stuck again
constexpr float KICK_START_PLUS = 19.7f;       // % start thresholds measured on motor 1
constexpr float KICK_START_MINUS = 20.8f;
constexpr float KICK_RATE = 40.0f;             // % per second added while still stuck
constexpr float KICK_MAX = 60.0f;              // %

struct FfPoint {
  float rpm;
  float duty;   // % that holds this speed in steady state
};
// Motor 1 steady-state sweep (12 V, 1 kHz PWM); the 0 RPM point is the stop threshold
constexpr FfPoint FF_PLUS[] = {
  {0, 11.6}, {50.3, 12}, {71.6, 13}, {109.9, 15}, {146.2, 17}, {191.1, 20}, {256.6, 25},
  {320.9, 30}, {440.7, 40}, {558.8, 50}, {675.3, 60}, {892.8, 80}, {1115.1, 100},
};
constexpr FfPoint FF_MINUS[] = {
  {0, 10.0}, {25.3, 11}, {56.9, 12}, {78.4, 13}, {118.1, 15}, {153.5, 17}, {199.4, 20},
  {268.0, 25}, {331.2, 30}, {453.6, 40}, {574.4, 50}, {689.9, 60}, {905.8, 80}, {1127.0, 100},
};

enum class RunMode : uint8_t { OpenLoop = 0, Feedforward = 1, FeedforwardPi = 2, Move = 3 };

// Move: drive a set number of encoder counts with a trapezoidal speed profile (accelerate,
// cruise, decelerate so it can stop in the distance left), run by the speed loop, then brake
constexpr float MOVE_MIN_RPM = 40.0f;          // final approach speed, above the stuck limit
constexpr int32_t MOVE_TOLERANCE_COUNTS = 2;   // close enough: start braking
constexpr uint32_t MOVE_BRAKE_MS = 300;        // both inputs low (short-circuit brake), then coast
enum class MovePhase : uint8_t { Idle = 0, Moving = 1, Braking = 2, Done = 3, Aborted = 4 };

constexpr const char* FIRMWARE_VERSION = "0.19.0";

// Protocol (one message per line):
//   ESP32 -> PC : HB,<uptime_ms>               heartbeat
//                 HELLO,<device>,<version>     identification
//                 INFO,<chip>,<revision>,<cores>,<cpu_mhz>,<flash_bytes>,<heap_bytes>,<mac>,<reset_reason>
//                 HEALTH,<free_heap>,<min_free_heap>,<max_alloc_heap>,<loop_avg_us>,<loop_max_us>,<loop_hz>,<stack_free_bytes>
//                 (loop_* and stack_free_bytes describe the loop() task on core 1)
//                 POWER,<motor>,<voltage_V>,<current_mA>,<power_mW>,<min_voltage_V>,<max_current_mA>
//                 (motor is 1 or 2; averages and extremes over the last report window)
//                 POWER,<motor>,NA             no INA219 found for that motor
//                 RUNOUT,<state>,<motor>,<duty>,<remaining_ms>
//                 (state 0 idle, 1 running, 2 stopped by the watchdog, 3 duration finished;
//                  duty in % with the sign as direction; remaining_ms 0 when there is no duration)
//                 ENC,<count1>,<cps1>,<count2>,<cps2>
//                 (encoder counts and counts per second; the sign follows the + direction
//                  once ENCINV has been set to match)
//                 (motor 4 = both, from DUTY)
//                 DUTYOUT,<duty1>,<duty2>      duty driven on each motor right now (any mode)
//                 SPD,<mode>,<target_rpm>,<ref_rpm>,<speed_rpm>,<ff_duty>,<pi_duty>
//                 MOVE,<phase>,<target_counts>,<position_counts>,<profile_rpm>,<final_error_counts>
//                 (phase 0 idle, 1 moving, 2 braking, 3 done, 4 aborted; position from the start)
//                 (speed control of the running motor; mode 0 open loop, 1 FF, 2 FF + PI)
//                 L,<t_us>,<duty>,<count1>,<count2>,<target_rpm>,<ref_rpm>,<speed_rpm>
//                 (one line per 200 Hz control tick while logging is on; t_us is micros()
//                  at the tick, duty the one applied in that tick, speed the controller's)
//   PC -> ESP32 : HELLO?                       request HELLO and INFO
//                 PING                         dashboard is alive (keeps the motor allowed to run)
//                 RUN,<target>,<duty>,<duration_ms>
//                                              drive one motor: duty -100..100 %, + = IN1, - = IN2;
//                                              target 3 is the onboard LED (brightness = |duty|);
//                                              duration_ms 0 runs until STOP
//                 DUTY,<duty1>,<duty2>,<duration_ms>
//                                              drive both motors open loop at once (0 = coast)
//                 STOP                         stop the motor
//                 ENCZERO,<motor>              zero that motor's encoder count
//                 ENCINV,<motor>,<0|1>         flip that encoder's sign
//                 LOG,<0|1>                    stream the 200 Hz control ticks
//                 SPEED,<motor>,<rpm>,<mode>   closed/open speed control: mode 1 FF, 2 FF + PI
//                 GAIN,<kp>,<ki>               PI gains in V/RPM and V/(RPM*s)
//                 CPR,<motor>,<counts>         counts per output revolution for RPM
//                 REF,<tau_ms>,<ka>            reference time constant and kA (V per RPM/s)
//                 MOVE,<motor>,<counts>,<max_rpm>,<accel_rpm_per_s>
//                                              turn a set distance (signed counts) and brake

struct LoopStats {
  uint32_t lastStartUs = 0;
  uint64_t sumUs = 0;
  uint32_t count = 0;
  uint32_t maxUs = 0;
};

// Written by loop() on core 1, read and reset by the telemetry task on core 0
LoopStats loopStats;
portMUX_TYPE loopStatsMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t mainLoopTask = nullptr;

enum class RunState : uint8_t { Idle = 0, Running = 1, Watchdog = 2, Done = 3 };

struct MotorRun {
  int motor = 0;              // index into MOTOR_PINS, or MOTOR_COUNT for the LED
  RunMode mode = RunMode::OpenLoop;
  float duty = 0;             // -100..100 %, open loop
  float duties[MOTOR_COUNT] = {};   // DUTY: per motor, open loop
  float targetRpm = 0;        // speed modes
  int32_t moveCounts = 0;     // move mode: signed distance
  float moveMaxRpm = 0;
  float moveAccel = 0;        // RPM per second
  uint32_t moveSeq = 0;       // bumped by every MOVE so loop() notices a new one
  uint32_t durationMs = 0;    // 0 = until STOP
  uint32_t startMs = 0;
};

// Shared between the telemetry task (core 0: commands, reports) and loop() (core 1: output)
portMUX_TYPE runMux = portMUX_INITIALIZER_UNLOCKED;
MotorRun motorRun;
RunState runState = RunState::Idle;
uint32_t lastPingMs = 0;
float outputDuty = 0;   // duty currently driven, signed
float motorDuty[MOTOR_COUNT] = {};   // duty on each motor right now
float gainKp = DEFAULT_KP;
float gainKi = DEFAULT_KI;
float refTauS = DEFAULT_TAU_REF_S;
float gainKa = DEFAULT_KA;
float motorCpr[MOTOR_COUNT] = {DEFAULT_CPR, DEFAULT_CPR};

struct ControlStatus {
  float refRpm = 0;
  float speedRpm = 0;   // controller's speed estimate
  float ffDuty = 0;
  float piDuty = 0;
};
ControlStatus controlStatus;   // for the running motor

struct MoveStatus {
  MovePhase phase = MovePhase::Idle;
  int32_t targetCounts = 0;
  int32_t positionCounts = 0;
  float profileRpm = 0;
  int32_t finalErrorCounts = 0;
};
MoveStatus moveStatus;
uint32_t moveSeqCounter = 0;

struct EncoderStatus {
  int64_t count = 0;
  float countsPerSec = 0;
};

// Encoder readings (written by loop() on core 1) and settings (written by the telemetry task)
EncoderStatus encoderStatus[MOTOR_COUNT];
bool encoderInvert[MOTOR_COUNT] = {};
bool encoderZero[MOTOR_COUNT] = {};   // set by ENCZERO, consumed by loop()
ESP32Encoder encoders[MOTOR_COUNT];

// One entry per control tick, produced on core 1 and printed by the telemetry task
struct LogEntry {
  uint32_t tUs;
  float duty;
  int32_t count[MOTOR_COUNT];
  float targetRpm;
  float refRpm;
  float speedRpm;
};
QueueHandle_t logQueue = nullptr;
volatile bool logEnabled = false;

struct PowerWindow {
  float sumVoltage = 0;
  float sumCurrentMa = 0;
  float sumPowerMw = 0;
  float minVoltage = 0;
  float maxCurrentMa = 0;
  uint32_t count = 0;
};

struct PowerChannel {
  uint8_t address;
  Adafruit_INA219 sensor;
  bool ready = false;
  uint32_t lastProbe = 0;
  uint32_t lastReport = 0;
  PowerWindow window;

  explicit PowerChannel(uint8_t addr) : address(addr), sensor(addr) {}
};

// Only touched by the telemetry task on core 0
PowerChannel powerChannels[POWER_CHANNELS] = {
  PowerChannel(POWER_SENSOR_ADDRESSES[0]),
  PowerChannel(POWER_SENSOR_ADDRESSES[1]),
};

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

void printHello() {
  Serial.printf("HELLO,%s,%s\n", DEVICE_NAME, FIRMWARE_VERSION);
}

void printInfo() {
  uint64_t mac = ESP.getEfuseMac();
  Serial.printf("INFO,%s,%u,%u,%lu,%lu,%lu,%02X:%02X:%02X:%02X:%02X:%02X,%s\n",
                ESP.getChipModel(),
                (unsigned)ESP.getChipRevision(),
                (unsigned)ESP.getChipCores(),
                (unsigned long)ESP.getCpuFreqMHz(),
                (unsigned long)ESP.getFlashChipSize(),
                (unsigned long)ESP.getHeapSize(),
                (uint8_t)(mac), (uint8_t)(mac >> 8), (uint8_t)(mac >> 16),
                (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40),
                resetReasonName(esp_reset_reason()));
}

void handleCommand(const String& cmd) {
  if (cmd == "HELLO?") {
    printHello();
    printInfo();
  } else if (cmd == "PING") {
    portENTER_CRITICAL(&runMux);
    lastPingMs = millis();
    portEXIT_CRITICAL(&runMux);
  } else if (cmd == "STOP") {
    portENTER_CRITICAL(&runMux);
    runState = RunState::Idle;
    portEXIT_CRITICAL(&runMux);
  } else if (cmd.startsWith("LOG,")) {
    logEnabled = cmd.endsWith("1");
  } else if (cmd.startsWith("ENCZERO,")) {
    unsigned motor;
    if (sscanf(cmd.c_str(), "ENCZERO,%u", &motor) == 1 && motor >= 1 && motor <= MOTOR_COUNT) {
      portENTER_CRITICAL(&runMux);
      encoderZero[motor - 1] = true;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("ENCINV,")) {
    unsigned motor, invert;
    if (sscanf(cmd.c_str(), "ENCINV,%u,%u", &motor, &invert) == 2 && motor >= 1 && motor <= MOTOR_COUNT) {
      portENTER_CRITICAL(&runMux);
      encoderInvert[motor - 1] = invert != 0;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("MOVE,")) {
    unsigned motor;
    long counts;
    float maxRpm, accel;
    if (sscanf(cmd.c_str(), "MOVE,%u,%ld,%f,%f", &motor, &counts, &maxRpm, &accel) == 4 &&
        motor >= 1 && motor <= MOTOR_COUNT && counts != 0 && maxRpm >= MOVE_MIN_RPM &&
        maxRpm <= 1200 && accel >= 100 && accel <= 20000) {
      portENTER_CRITICAL(&runMux);
      motorRun.motor = motor - 1;
      motorRun.mode = RunMode::Move;
      motorRun.targetRpm = 0;
      motorRun.duty = 0;
      motorRun.moveCounts = counts;
      motorRun.moveMaxRpm = maxRpm;
      motorRun.moveAccel = accel;
      motorRun.moveSeq = ++moveSeqCounter;
      motorRun.durationMs = 0;
      motorRun.startMs = millis();
      lastPingMs = millis();
      runState = RunState::Running;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("SPEED,")) {
    unsigned motor, mode;
    float rpm;
    if (sscanf(cmd.c_str(), "SPEED,%u,%f,%u", &motor, &rpm, &mode) == 3 &&
        motor >= 1 && motor <= MOTOR_COUNT && mode >= 1 && mode <= 2) {
      portENTER_CRITICAL(&runMux);
      motorRun.motor = motor - 1;
      motorRun.mode = (RunMode)mode;
      motorRun.targetRpm = rpm;
      motorRun.duty = 0;
      motorRun.durationMs = 0;
      motorRun.startMs = millis();
      lastPingMs = millis();
      runState = RunState::Running;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("GAIN,")) {
    float kp, ki;
    if (sscanf(cmd.c_str(), "GAIN,%f,%f", &kp, &ki) == 2 && kp >= 0 && ki >= 0) {
      portENTER_CRITICAL(&runMux);
      gainKp = kp;
      gainKi = ki;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("REF,")) {
    float tauMs, ka;
    if (sscanf(cmd.c_str(), "REF,%f,%f", &tauMs, &ka) == 2 && tauMs >= 5 && ka >= 0) {
      portENTER_CRITICAL(&runMux);
      refTauS = tauMs / 1000.0f;
      gainKa = ka;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("CPR,")) {
    unsigned motor;
    float cpr;
    if (sscanf(cmd.c_str(), "CPR,%u,%f", &motor, &cpr) == 2 && motor >= 1 && motor <= MOTOR_COUNT && cpr > 0) {
      portENTER_CRITICAL(&runMux);
      motorCpr[motor - 1] = cpr;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("DUTY,")) {
    float d1, d2;
    unsigned long duration;
    if (sscanf(cmd.c_str(), "DUTY,%f,%f,%lu", &d1, &d2, &duration) == 3) {
      portENTER_CRITICAL(&runMux);
      motorRun.motor = ALL_MOTORS;
      motorRun.mode = RunMode::OpenLoop;
      motorRun.targetRpm = 0;
      motorRun.duties[0] = constrain(d1, -100.0f, 100.0f);
      motorRun.duties[1] = constrain(d2, -100.0f, 100.0f);
      motorRun.duty = motorRun.duties[0];
      motorRun.durationMs = duration;
      motorRun.startMs = millis();
      lastPingMs = millis();
      runState = RunState::Running;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("RUN,")) {
    unsigned motor;
    float duty;
    unsigned long duration;
    if (sscanf(cmd.c_str(), "RUN,%u,%f,%lu", &motor, &duty, &duration) == 3 &&
        motor >= 1 && motor <= LED_TARGET) {
      portENTER_CRITICAL(&runMux);
      motorRun.motor = motor - 1;
      motorRun.mode = RunMode::OpenLoop;
      motorRun.targetRpm = 0;
      motorRun.duty = constrain(duty, -100.0f, 100.0f);
      motorRun.durationMs = duration;
      motorRun.startMs = millis();
      lastPingMs = millis();
      runState = RunState::Running;
      portEXIT_CRITICAL(&runMux);
    }
  }
}

void readCommands() {
  static String buffer;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();
      handleCommand(buffer);
      buffer = "";
    } else if (buffer.length() < 64) {
      buffer += c;
    }
  }
}

// Measures the time between consecutive loop() starts
void measureLoop() {
  uint32_t now = micros();
  portENTER_CRITICAL(&loopStatsMux);
  if (loopStats.lastStartUs != 0) {
    uint32_t dt = now - loopStats.lastStartUs;
    loopStats.sumUs += dt;
    loopStats.count++;
    if (dt > loopStats.maxUs) loopStats.maxUs = dt;
  }
  loopStats.lastStartUs = now;
  portEXIT_CRITICAL(&loopStatsMux);
}

void sendHeartbeat() {
  static uint32_t lastBeat = 0;
  uint32_t now = millis();
  if (now - lastBeat >= HEARTBEAT_INTERVAL_MS) {
    lastBeat = now;
    Serial.printf("HB,%lu\n", now);
  }
}

void sendHealth() {
  static uint32_t lastReport = 0;
  uint32_t now = millis();
  uint32_t elapsed = now - lastReport;
  if (elapsed < HEALTH_INTERVAL_MS) return;
  lastReport = now;

  // Take a snapshot under the lock; never print while holding it
  portENTER_CRITICAL(&loopStatsMux);
  LoopStats snapshot = loopStats;
  loopStats.sumUs = 0;
  loopStats.count = 0;
  loopStats.maxUs = 0;
  portEXIT_CRITICAL(&loopStatsMux);

  float avgUs = snapshot.count ? (float)snapshot.sumUs / snapshot.count : 0;
  uint32_t loopHz = (uint64_t)snapshot.count * 1000 / elapsed;

  Serial.printf("HEALTH,%lu,%lu,%lu,%.2f,%lu,%lu,%lu\n",
                (unsigned long)ESP.getFreeHeap(),
                (unsigned long)ESP.getMinFreeHeap(),
                (unsigned long)ESP.getMaxAllocHeap(),
                avgUs,
                (unsigned long)snapshot.maxUs,
                (unsigned long)loopHz,
                (unsigned long)uxTaskGetStackHighWaterMark(mainLoopTask));
}

bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// Looks for each sensor every SENSOR_PROBE_MS until found, then samples it
void samplePower() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();
  bool sampleDue = now - lastSample >= POWER_SAMPLE_MS;
  if (sampleDue) lastSample = now;

  for (PowerChannel& ch : powerChannels) {
    if (!ch.ready) {
      if (ch.lastProbe != 0 && now - ch.lastProbe < SENSOR_PROBE_MS) continue;
      ch.lastProbe = now;
      // begin() also rewrites the calibration, which the chip loses when unplugged
      if (i2cDevicePresent(ch.address) && ch.sensor.begin()) {
        ch.ready = true;
        ch.window = PowerWindow();
      }
      continue;
    }
    if (!sampleDue) continue;

    float voltage = ch.sensor.getBusVoltage_V();   // measured at Vin-, i.e. what the motor sees
    float currentMa = ch.sensor.getCurrent_mA();
    PowerWindow& w = ch.window;
    if (w.count == 0) {
      w.minVoltage = voltage;
      w.maxCurrentMa = currentMa;
    }
    w.sumVoltage += voltage;
    w.sumCurrentMa += currentMa;
    w.sumPowerMw += voltage * currentMa;
    w.minVoltage = min(w.minVoltage, voltage);
    w.maxCurrentMa = max(w.maxCurrentMa, currentMa);
    w.count++;
  }
}

void sendPower() {
  uint32_t now = millis();
  for (size_t i = 0; i < POWER_CHANNELS; i++) {
    PowerChannel& ch = powerChannels[i];
    unsigned motor = i + 1;

    if (ch.ready && now - ch.lastReport >= POWER_REPORT_MS) {
      ch.lastReport = now;
      if (!i2cDevicePresent(ch.address)) {   // unplugged since the last report
        ch.ready = false;
        continue;
      }
      PowerWindow& w = ch.window;
      if (w.count == 0) continue;
      float n = w.count;
      Serial.printf("POWER,%u,%.3f,%.1f,%.1f,%.3f,%.1f\n", motor,
                    w.sumVoltage / n, w.sumCurrentMa / n, w.sumPowerMw / n,
                    w.minVoltage, w.maxCurrentMa);
      w = PowerWindow();
    } else if (!ch.ready && now - ch.lastReport >= POWER_ABSENT_REPORT_MS) {
      ch.lastReport = now;
      Serial.printf("POWER,%u,NA\n", motor);
    }
  }
}

void sendMotorRun() {
  static uint32_t lastReport = 0;
  uint32_t now = millis();
  if (now - lastReport < MOTOR_REPORT_MS) return;
  lastReport = now;

  portENTER_CRITICAL(&runMux);
  RunState state = runState;
  MotorRun run = motorRun;
  float duty = outputDuty;
  float duties[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) duties[i] = motorDuty[i];
  portEXIT_CRITICAL(&runMux);

  uint32_t remaining = 0;
  if (state == RunState::Running && run.durationMs > 0) {
    uint32_t elapsed = now - run.startMs;
    remaining = elapsed < run.durationMs ? run.durationMs - elapsed : 0;
  }
  Serial.printf("RUNOUT,%u,%d,%.1f,%lu\n", (unsigned)state, run.motor + 1, duty,
                (unsigned long)remaining);
  Serial.printf("DUTYOUT,%.1f,%.1f\n", duties[0], duties[1]);

  portENTER_CRITICAL(&runMux);
  EncoderStatus enc[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) enc[i] = encoderStatus[i];
  portEXIT_CRITICAL(&runMux);
  Serial.printf("ENC,%lld,%.0f,%lld,%.0f\n", enc[0].count, enc[0].countsPerSec,
                enc[1].count, enc[1].countsPerSec);

  portENTER_CRITICAL(&runMux);
  ControlStatus ctl = controlStatus;
  portEXIT_CRITICAL(&runMux);
  Serial.printf("SPD,%u,%.1f,%.1f,%.1f,%.1f,%.1f\n", (unsigned)run.mode, run.targetRpm, ctl.refRpm,
                ctl.speedRpm, ctl.ffDuty, ctl.piDuty);

  portENTER_CRITICAL(&runMux);
  MoveStatus mv = moveStatus;
  portEXIT_CRITICAL(&runMux);
  Serial.printf("MOVE,%u,%ld,%ld,%.1f,%ld\n", (unsigned)mv.phase, (long)mv.targetCounts,
                (long)mv.positionCounts, mv.profileRpm, (long)mv.finalErrorCounts);
}

void sendLog() {
  LogEntry e;
  while (xQueueReceive(logQueue, &e, 0) == pdTRUE) {
    Serial.printf("L,%lu,%.1f,%ld,%ld,%.1f,%.1f,%.1f\n", (unsigned long)e.tUs, e.duty,
                  (long)e.count[0], (long)e.count[1], e.targetRpm, e.refRpm, e.speedRpm);
  }
}

// Runs on core 0 and owns the serial port and I2C bus, so this I/O never slows loop() down
void telemetryTask(void*) {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  printHello();
  printInfo();
  for (;;) {
    readCommands();
    sendHeartbeat();
    sendHealth();
    samplePower();
    sendPower();
    sendMotorRun();
    sendLog();
    vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

// ---------- Motor output (core 1) ----------

void setupMotors() {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;   // ignored on GPIO34/35
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    const MotorPins& p = MOTOR_PINS[i];
    pinMode(p.ena, OUTPUT);
    digitalWrite(p.ena, LOW);   // coast
    ledcAttach(p.in1, PWM_FREQ_HZ, PWM_BITS);
    ledcAttach(p.in2, PWM_FREQ_HZ, PWM_BITS);
    ledcWrite(p.in1, 0);
    ledcWrite(p.in2, 0);
    // Counted by the PCNT hardware, so no pulses are lost to interrupt timing
    encoders[i].attachFullQuad(p.encA, p.encB);
    encoders[i].setFilter(250);   // ignore glitches shorter than ~3 us
    encoders[i].clearCount();
  }
  ledcAttach(LED_PIN, PWM_FREQ_HZ, PWM_BITS);
  ledcWrite(LED_PIN, 0);
}

// duty in % (-100..100): + drives IN1, - drives IN2; enabled = ENA HIGH
void applyDuty(size_t motor, float duty, bool enabled) {
  const MotorPins& p = MOTOR_PINS[motor];
  uint32_t pwm = (uint32_t)(fabsf(duty) / 100.0f * PWM_MAX + 0.5f);
  ledcWrite(p.in1, duty > 0 ? pwm : 0);
  ledcWrite(p.in2, duty < 0 ? pwm : 0);
  digitalWrite(p.ena, enabled ? HIGH : LOW);
}

// Duty (%) that holds `rpm` in steady state, by linear interpolation in the measured
// curve for that direction; 0 RPM gives 0 (stopped), beyond the last point 100 %
template <size_t N>
float interpolateFf(const FfPoint (&table)[N], float rpm) {
  if (rpm >= table[N - 1].rpm) return table[N - 1].duty;
  for (size_t i = 1; i < N; i++) {
    if (rpm <= table[i].rpm) {
      float f = (rpm - table[i - 1].rpm) / (table[i].rpm - table[i - 1].rpm);
      return table[i - 1].duty + f * (table[i].duty - table[i - 1].duty);
    }
  }
  return table[N - 1].duty;
}

float feedforwardDuty(float rpm) {
  if (rpm > 0) return interpolateFf(FF_PLUS, rpm);
  if (rpm < 0) return -interpolateFf(FF_MINUS, -rpm);
  return 0;
}

struct PiState {
  float integralV = 0;
  float refRpm = 0;
  float kickDuty = 0;   // breakaway kick while stuck, 0 once moving
  bool stuck = false;
  uint32_t stillMs = 0;  // how long it has not turned in the target's direction
  float lastTargetRpm = 0;
  RunMode lastMode = RunMode::OpenLoop;
  int lastMotor = -1;
};

struct Gains {
  float kp;
  float ki;
  float ka;
  float tauRefS;
};

// Speed modes: returns the duty (%) and fills the reference and FF / PI split for reporting
// directRef: the move profile supplies the reference and its rate, skipping the filter
float speedControl(PiState& pi, const MotorRun& run, float speedRpm, const Gains& g,
                   float& ffDuty, float& piDuty, bool directRef = false, float directRate = 0) {
  constexpr float dt = CONTROL_PERIOD_US / 1e6f;
  bool restart = run.motor != pi.lastMotor || run.mode != pi.lastMode || pi.lastTargetRpm == 0;
  if (restart) {
    pi.refRpm = speedRpm;   // follow on from where the motor is
    pi.integralV = 0;
  } else if ((run.targetRpm > 0) != (pi.lastTargetRpm > 0)) {
    pi.integralV = 0;       // direction change: friction flips sign, start the integral clean
  }
  pi.lastMotor = run.motor;
  pi.lastMode = run.mode;
  pi.lastTargetRpm = run.targetRpm;

  ffDuty = 0;
  piDuty = 0;
  if (run.targetRpm == 0) {
    pi.refRpm = 0;
    pi.kickDuty = 0;
    return 0;
  }

  // Stuck: hold the reference at the motor, keep the integral empty and push with a rising
  // kick until it breaks free. With hysteresis, because at low speed the estimate jumps in
  // 16 RPM steps: free once faster than STUCK_RPM (half the target if that is slower),
  // stuck again only after STILL_MS without turning the target's way.
  float freeLimit = fminf(STUCK_RPM, fabsf(run.targetRpm) / 2);
  float along = run.targetRpm > 0 ? speedRpm : -speedRpm;
  pi.stillMs = along < STILL_RPM ? pi.stillMs + CONTROL_PERIOD_US / 1000 : 0;
  if (restart) pi.stuck = along < freeLimit;
  else if (pi.stuck && along >= freeLimit) pi.stuck = false;
  else if (!pi.stuck && pi.stillMs >= STILL_MS) pi.stuck = true;
  if (pi.stuck) {
    float start = run.targetRpm > 0 ? KICK_START_PLUS : KICK_START_MINUS;
    pi.kickDuty = pi.kickDuty == 0 ? start : fminf(pi.kickDuty + KICK_RATE * dt, KICK_MAX);
    pi.refRpm = speedRpm;
    pi.integralV = 0;
    ffDuty = run.targetRpm > 0 ? pi.kickDuty : -pi.kickDuty;
    return ffDuty;
  }
  pi.kickDuty = 0;

  // Reference: first-order approach to the target, or the move profile itself
  float refRate;
  if (directRef) {
    refRate = directRate;
    pi.refRpm = run.targetRpm;
  } else {
    refRate = (run.targetRpm - pi.refRpm) / g.tauRefS;   // RPM/s
    pi.refRpm += refRate * dt;
  }

  float ffV = feedforwardDuty(pi.refRpm) / 100.0f * SUPPLY_V + g.ka * refRate;
  ffV = constrain(ffV, -SUPPLY_V, SUPPLY_V);
  ffDuty = ffV / SUPPLY_V * 100.0f;
  if (run.mode != RunMode::FeedforwardPi) return ffDuty;

  float error = pi.refRpm - speedRpm;
  float pV = g.kp * error;
  float candidate = pi.integralV + g.ki * error * dt;
  float uCandidate = ffV + pV + candidate;
  // Clamping: integrate unless the output is saturated and the error pushes it further
  bool saturated = fabsf(uCandidate) > SUPPLY_V;
  if (!saturated || (uCandidate > 0) != (error > 0)) pi.integralV = candidate;
  float u = constrain(ffV + pV + pi.integralV, -SUPPLY_V, SUPPLY_V);
  piDuty = (u - ffV) / SUPPLY_V * 100.0f;
  return u / SUPPLY_V * 100.0f;
}

struct MoveRuntime {
  uint32_t seq = 0;
  MovePhase phase = MovePhase::Idle;
  int64_t startCount = 0;
  int32_t targetCounts = 0;
  int32_t positionCounts = 0;
  float profileRpm = 0;          // unsigned profile speed
  int32_t finalErrorCounts = 0;
  uint32_t brakeUntilMs = 0;
};

// One tick of a move: profile speed -> speed loop, then brake once within tolerance.
// Returns the duty; outputs the signed profile speed for the log.
float runMove(MoveRuntime& mv, const MotorRun& run, PiState& pi, float speedRpm, const Gains& g,
              uint32_t now, float& ffDuty, float& piDuty, float& profileSigned) {
  constexpr float dt = CONTROL_PERIOD_US / 1e6f;
  portENTER_CRITICAL(&runMux);
  int64_t count = encoderStatus[run.motor].count;
  float cpr = motorCpr[run.motor];
  portEXIT_CRITICAL(&runMux);

  if (run.moveSeq != mv.seq) {   // a new MOVE: measure from here
    mv = MoveRuntime();
    mv.seq = run.moveSeq;
    mv.startCount = count;
    mv.targetCounts = run.moveCounts;
    mv.phase = MovePhase::Moving;
    pi = PiState();
  }
  mv.positionCounts = (int32_t)(count - mv.startCount);
  int32_t remaining = mv.targetCounts - mv.positionCounts;
  float dir = mv.targetCounts > 0 ? 1.0f : -1.0f;
  ffDuty = 0;
  piDuty = 0;
  profileSigned = 0;

  if (mv.phase == MovePhase::Moving) {
    bool arrived = abs(remaining) <= MOVE_TOLERANCE_COUNTS || remaining * dir < 0;
    if (arrived) {
      mv.phase = MovePhase::Braking;
      mv.brakeUntilMs = now + MOVE_BRAKE_MS;
      mv.profileRpm = 0;
    } else {
      // Fastest speed that can still stop in the distance left: v^2 = 2 a d
      float remainingRev = fabsf((float)remaining) / cpr;
      float stopRpm = sqrtf(2.0f * run.moveAccel * remainingRev * 60.0f);
      float wanted = fmaxf(MOVE_MIN_RPM, fminf(run.moveMaxRpm, stopRpm));
      float previous = mv.profileRpm;
      mv.profileRpm = wanted > previous ? fminf(wanted, previous + run.moveAccel * dt) : wanted;
      profileSigned = dir * mv.profileRpm;
      MotorRun speedRun = run;
      speedRun.mode = RunMode::FeedforwardPi;
      speedRun.targetRpm = profileSigned;
      float rate = dir * (mv.profileRpm - previous) / dt;
      return speedControl(pi, speedRun, speedRpm, g, ffDuty, piDuty, true, rate);
    }
  }
  if (mv.phase == MovePhase::Braking) {
    mv.finalErrorCounts = mv.targetCounts - mv.positionCounts;   // keeps updating while it stops
    if ((int32_t)(now - mv.brakeUntilMs) >= 0) {
      mv.phase = MovePhase::Done;
      portENTER_CRITICAL(&runMux);
      if (runState == RunState::Running && motorRun.moveSeq == mv.seq) runState = RunState::Done;
      portEXIT_CRITICAL(&runMux);
    }
  }
  return 0;   // braking: duty 0 with ENA high shorts the motor through the low side
}

void readEncoders(uint32_t now, float (&speedRpm)[MOTOR_COUNT]);

void runMotors() {
  // Fixed-period schedule: the next tick is due one period after the previous one was,
  // not after it actually ran, so the rate does not drift
  static uint32_t nextTickUs = 0;
  static PiState pi;
  static MoveRuntime mv;
  uint32_t tickUs = micros();
  if (nextTickUs != 0 && (int32_t)(tickUs - nextTickUs) < 0) return;
  if (nextTickUs == 0 || (int32_t)(tickUs - nextTickUs) > (int32_t)CONTROL_PERIOD_US) {
    nextTickUs = tickUs;   // first tick, or fell more than a period behind: resync
  }
  nextTickUs += CONTROL_PERIOD_US;
  uint32_t now = millis();

  portENTER_CRITICAL(&runMux);
  if (runState == RunState::Running && now - lastPingMs > MOTOR_WATCHDOG_MS) {
    runState = RunState::Watchdog;
  }
  if (runState == RunState::Running && motorRun.durationMs > 0 &&
      now - motorRun.startMs >= motorRun.durationMs) {
    runState = RunState::Done;
  }
  MotorRun run = motorRun;
  bool running = runState == RunState::Running;
  Gains gains = {gainKp, gainKi, gainKa, refTauS};
  portEXIT_CRITICAL(&runMux);

  // Measure first, then decide this tick's duty
  float speedRpm[MOTOR_COUNT];
  readEncoders(now, speedRpm);

  float duty = 0, ffDuty = 0, piDuty = 0;
  bool motorActive = running && run.motor < (int)MOTOR_COUNT;
  float moveTargetRpm = 0;
  if (motorActive && run.mode == RunMode::Move) {
    duty = runMove(mv, run, pi, speedRpm[run.motor], gains, now, ffDuty, piDuty, moveTargetRpm);
  } else if (motorActive) {
    duty = run.mode == RunMode::OpenLoop
        ? run.duty
        : speedControl(pi, run, speedRpm[run.motor], gains, ffDuty, piDuty);
  } else {
    pi = PiState();
    if (mv.phase == MovePhase::Moving || mv.phase == MovePhase::Braking) mv.phase = MovePhase::Aborted;
  }
  portENTER_CRITICAL(&runMux);
  moveStatus = {mv.phase, mv.targetCounts, mv.positionCounts, mv.profileRpm, mv.finalErrorCounts};
  portEXIT_CRITICAL(&runMux);
  bool allActive = running && run.motor == ALL_MOTORS;
  float applied[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    bool active = motorActive && i == (size_t)run.motor;
    applied[i] = allActive ? run.duties[i] : active ? duty : 0;
    applyDuty(i, applied[i], active || (allActive && applied[i] != 0));
  }
  if (allActive) duty = run.duties[0];
  bool ledActive = running && run.motor == (int)MOTOR_COUNT;
  float ledDuty = ledActive ? fabsf(run.duty) : 0;   // the LED has no direction
  ledcWrite(LED_PIN, (uint32_t)(ledDuty / 100.0f * PWM_MAX + 0.5f));

  float shownSpeed = run.motor < (int)MOTOR_COUNT ? speedRpm[run.motor] : allActive ? speedRpm[0] : 0;
  portENTER_CRITICAL(&runMux);
  outputDuty = !running ? 0 : ledActive ? ledDuty : duty;
  for (size_t i = 0; i < MOTOR_COUNT; i++) motorDuty[i] = applied[i];
  controlStatus = {run.mode == RunMode::OpenLoop ? 0 : pi.refRpm, shownSpeed, ffDuty, piDuty};
  portEXIT_CRITICAL(&runMux);

  if (logEnabled) {
    LogEntry e;
    e.tUs = tickUs;
    portENTER_CRITICAL(&runMux);
    e.duty = outputDuty;
    for (size_t i = 0; i < MOTOR_COUNT; i++) e.count[i] = (int32_t)encoderStatus[i].count;
    portEXIT_CRITICAL(&runMux);
    e.targetRpm = run.mode == RunMode::OpenLoop ? 0 : run.mode == RunMode::Move ? moveTargetRpm : run.targetRpm;
    e.refRpm = run.mode == RunMode::OpenLoop ? 0 : pi.refRpm;
    e.speedRpm = shownSpeed;
    xQueueSend(logQueue, &e, 0);   // never block the control loop; drop if the queue is full
  }
}

// Counts every tick. Two speeds: the controller's (backward difference over SPEED_TAPS
// ticks, in RPM) and the reported one (count change over SPEED_WINDOW_MS)
void readEncoders(uint32_t now, float (&speedRpm)[MOTOR_COUNT]) {
  static uint32_t lastSpeedMs = 0;
  static int64_t lastCount[MOTOR_COUNT] = {};
  static float countsPerSec[MOTOR_COUNT] = {};
  static int64_t history[MOTOR_COUNT][SPEED_TAPS + 1] = {};
  static size_t head = 0;

  bool zero[MOTOR_COUNT];
  bool invert[MOTOR_COUNT];
  float cpr[MOTOR_COUNT];
  portENTER_CRITICAL(&runMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    zero[i] = encoderZero[i];
    encoderZero[i] = false;
    invert[i] = encoderInvert[i];
    cpr[i] = motorCpr[i];
  }
  portEXIT_CRITICAL(&runMux);

  bool speedDue = now - lastSpeedMs >= SPEED_WINDOW_MS;
  float dt = (now - lastSpeedMs) / 1000.0f;
  if (speedDue) lastSpeedMs = now;
  head = (head + 1) % (SPEED_TAPS + 1);
  size_t oldest = (head + 1) % (SPEED_TAPS + 1);
  constexpr float windowS = SPEED_TAPS * (CONTROL_PERIOD_US / 1e6f);

  EncoderStatus status[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    if (zero[i]) {
      encoders[i].clearCount();
      lastCount[i] = 0;
      for (int64_t& h : history[i]) h = 0;
    }
    int64_t raw = encoders[i].getCount();
    int64_t signedCount = invert[i] ? -raw : raw;
    history[i][head] = signedCount;
    speedRpm[i] = (signedCount - history[i][oldest]) / windowS * 60.0f / cpr[i];
    if (speedDue && dt > 0) {
      countsPerSec[i] = (raw - lastCount[i]) / dt;
      lastCount[i] = raw;
    }
    status[i].count = signedCount;
    status[i].countsPerSec = invert[i] ? -countsPerSec[i] : countsPerSec[i];
  }

  portENTER_CRITICAL(&runMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) encoderStatus[i] = status[i];
  portEXIT_CRITICAL(&runMux);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  logQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogEntry));
  setupMotors();
  // setup() runs inside the loop task, so this is the task loop() runs in
  mainLoopTask = xTaskGetCurrentTaskHandle();
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", TELEMETRY_STACK_BYTES, nullptr,
                          TELEMETRY_PRIORITY, nullptr, TELEMETRY_CORE);
}

// Core 1: time-critical work only
void loop() {
  measureLoop();
  runMotors();
}
