#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ESP32Encoder.h>
#include <esp_system.h>

constexpr uint32_t SERIAL_BAUD = 115200;
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
constexpr uint32_t PWM_FREQ_HZ = 1000;      // same as Arduino-ESP32's analogWrite() default
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1;
constexpr uint32_t MOTOR_UPDATE_MS = 5;
constexpr uint32_t MOTOR_WATCHDOG_MS = 1000;  // stop if the dashboard goes quiet this long
constexpr uint32_t MOTOR_REPORT_MS = 40;      // 25 Hz
constexpr uint32_t SPEED_WINDOW_MS = 50;      // encoder speed = count change over this window

constexpr const char* FIRMWARE_VERSION = "0.12.0";

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
//   PC -> ESP32 : HELLO?                       request HELLO and INFO
//                 PING                         dashboard is alive (keeps the motor allowed to run)
//                 RUN,<target>,<duty>,<duration_ms>
//                                              drive one motor: duty -100..100 %, + = IN1, - = IN2;
//                                              target 3 is the onboard LED (brightness = |duty|);
//                                              duration_ms 0 runs until STOP
//                 STOP                         stop the motor
//                 ENCZERO,<motor>              zero that motor's encoder count
//                 ENCINV,<motor>,<0|1>         flip that encoder's sign

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
  float duty = 0;             // -100..100 %
  uint32_t durationMs = 0;    // 0 = until STOP
  uint32_t startMs = 0;
};

// Shared between the telemetry task (core 0: commands, reports) and loop() (core 1: output)
portMUX_TYPE runMux = portMUX_INITIALIZER_UNLOCKED;
MotorRun motorRun;
RunState runState = RunState::Idle;
uint32_t lastPingMs = 0;
float outputDuty = 0;   // duty currently driven, signed

struct EncoderStatus {
  int64_t count = 0;
  float countsPerSec = 0;
};

// Encoder readings (written by loop() on core 1) and settings (written by the telemetry task)
EncoderStatus encoderStatus[MOTOR_COUNT];
bool encoderInvert[MOTOR_COUNT] = {};
bool encoderZero[MOTOR_COUNT] = {};   // set by ENCZERO, consumed by loop()
ESP32Encoder encoders[MOTOR_COUNT];

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
  } else if (cmd.startsWith("RUN,")) {
    unsigned motor;
    float duty;
    unsigned long duration;
    if (sscanf(cmd.c_str(), "RUN,%u,%f,%lu", &motor, &duty, &duration) == 3 &&
        motor >= 1 && motor <= LED_TARGET) {
      portENTER_CRITICAL(&runMux);
      motorRun.motor = motor - 1;
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
  portEXIT_CRITICAL(&runMux);

  uint32_t remaining = 0;
  if (state == RunState::Running && run.durationMs > 0) {
    uint32_t elapsed = now - run.startMs;
    remaining = elapsed < run.durationMs ? run.durationMs - elapsed : 0;
  }
  Serial.printf("RUNOUT,%u,%d,%.1f,%lu\n", (unsigned)state, run.motor + 1, duty,
                (unsigned long)remaining);

  portENTER_CRITICAL(&runMux);
  EncoderStatus enc[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) enc[i] = encoderStatus[i];
  portEXIT_CRITICAL(&runMux);
  Serial.printf("ENC,%lld,%.0f,%lld,%.0f\n", enc[0].count, enc[0].countsPerSec,
                enc[1].count, enc[1].countsPerSec);
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

void readEncoders(uint32_t now);

void runMotors() {
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  if (now - lastUpdate < MOTOR_UPDATE_MS) return;
  lastUpdate = now;

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
  portEXIT_CRITICAL(&runMux);

  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    bool active = running && i == (size_t)run.motor;
    applyDuty(i, active ? run.duty : 0, active);
  }
  bool ledActive = running && run.motor == (int)MOTOR_COUNT;
  float ledDuty = ledActive ? fabsf(run.duty) : 0;   // the LED has no direction
  ledcWrite(LED_PIN, (uint32_t)(ledDuty / 100.0f * PWM_MAX + 0.5f));

  portENTER_CRITICAL(&runMux);
  outputDuty = !running ? 0 : ledActive ? ledDuty : run.duty;
  portEXIT_CRITICAL(&runMux);

  readEncoders(now);
}

// Counts every update, speed from the count change over SPEED_WINDOW_MS
void readEncoders(uint32_t now) {
  static uint32_t lastSpeedMs = 0;
  static int64_t lastCount[MOTOR_COUNT] = {};
  static float countsPerSec[MOTOR_COUNT] = {};

  bool zero[MOTOR_COUNT];
  bool invert[MOTOR_COUNT];
  portENTER_CRITICAL(&runMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    zero[i] = encoderZero[i];
    encoderZero[i] = false;
    invert[i] = encoderInvert[i];
  }
  portEXIT_CRITICAL(&runMux);

  bool speedDue = now - lastSpeedMs >= SPEED_WINDOW_MS;
  float dt = (now - lastSpeedMs) / 1000.0f;
  if (speedDue) lastSpeedMs = now;

  EncoderStatus status[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    if (zero[i]) {
      encoders[i].clearCount();
      lastCount[i] = 0;
    }
    int64_t raw = encoders[i].getCount();
    if (speedDue && dt > 0) {
      countsPerSec[i] = (raw - lastCount[i]) / dt;
      lastCount[i] = raw;
    }
    status[i].count = invert[i] ? -raw : raw;
    status[i].countsPerSec = invert[i] ? -countsPerSec[i] : countsPerSec[i];
  }

  portENTER_CRITICAL(&runMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) encoderStatus[i] = status[i];
  portEXIT_CRITICAL(&runMux);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
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
