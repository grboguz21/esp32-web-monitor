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

// One L298N per motor. PWM goes to IN1/IN2 with ENA held HIGH, so in the off phase the
// low-side transistors carry the motor current (slow decay) instead of the diodes.
// ENA LOW lets the motor coast; that is the stopped state.
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
constexpr uint32_t PWM_FREQ_HZ = 5000;
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1;
constexpr uint32_t MOTOR_UPDATE_MS = 5;
constexpr uint32_t SPEED_WINDOW_MS = 50;
constexpr uint32_t MOTOR_WATCHDOG_MS = 1000;  // stop if the dashboard goes quiet this long
constexpr uint32_t MOTOR_REPORT_MS = 40;      // 25 Hz

constexpr const char* FIRMWARE_VERSION = "0.6.0";

// Protocol (one message per line):
//   ESP32 -> PC : HB,<uptime_ms>               heartbeat
//                 HELLO,<device>,<version>     identification
//                 INFO,<chip>,<revision>,<cores>,<cpu_mhz>,<flash_bytes>,<heap_bytes>,<mac>,<reset_reason>
//                 HEALTH,<free_heap>,<min_free_heap>,<max_alloc_heap>,<loop_avg_us>,<loop_max_us>,<loop_hz>,<stack_free_bytes>
//                 (loop_* and stack_free_bytes describe the loop() task on core 1)
//                 POWER,<motor>,<voltage_V>,<current_mA>,<power_mW>,<min_voltage_V>,<max_current_mA>
//                 (motor is 1 or 2; averages and extremes over the last report window)
//                 POWER,<motor>,NA             no INA219 found for that motor
//                 MOTORS,<state>,<duty1>,<count1>,<cps1>,<duty2>,<count2>,<cps2>
//                 (state 0 idle, 1 running, 2 stopped by the watchdog; duty in %,
//                  count is encoder counts since the test started, cps = counts/s)
//   PC -> ESP32 : HELLO?                       request HELLO and INFO
//                 PING                         dashboard is alive (keeps motors allowed)
//                 SQW,<mask>,<high>,<low>,<period_ms>
//                                              square wave test: mask 1 = motor 1, 2 = motor 2,
//                                              3 = both; duty in % (-100..100, sign = direction)
//                 STOP                         stop all motors

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

enum class TestState : uint8_t { Idle = 0, Running = 1, Watchdog = 2 };

struct SquareWave {
  uint8_t mask = 0;
  float dutyHigh = 0;
  float dutyLow = 0;
  uint32_t periodMs = 2000;
  uint32_t startMs = 0;
};

struct MotorStatus {
  float duty = 0;
  int64_t count = 0;
  float countsPerSec = 0;
};

// Shared between the telemetry task (core 0: commands, reports) and loop() (core 1: motors)
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;
SquareWave squareWave;
TestState testState = TestState::Idle;
bool restartTest = false;        // set by SQW, consumed by loop() to zero the encoders
uint32_t lastPingMs = 0;
MotorStatus motorStatus[MOTOR_COUNT];

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
    portENTER_CRITICAL(&motorMux);
    lastPingMs = millis();
    portEXIT_CRITICAL(&motorMux);
  } else if (cmd == "STOP") {
    portENTER_CRITICAL(&motorMux);
    testState = TestState::Idle;
    portEXIT_CRITICAL(&motorMux);
  } else if (cmd.startsWith("SQW,")) {
    unsigned mask;
    float high, low;
    unsigned long period;
    if (sscanf(cmd.c_str(), "SQW,%u,%f,%f,%lu", &mask, &high, &low, &period) == 4 &&
        mask >= 1 && mask <= 3 && period >= 100) {
      portENTER_CRITICAL(&motorMux);
      squareWave.mask = mask;
      squareWave.dutyHigh = constrain(high, -100.0f, 100.0f);
      squareWave.dutyLow = constrain(low, -100.0f, 100.0f);
      squareWave.periodMs = period;
      squareWave.startMs = millis();
      lastPingMs = millis();
      testState = TestState::Running;
      restartTest = true;
      portEXIT_CRITICAL(&motorMux);
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

void sendMotors() {
  static uint32_t lastReport = 0;
  uint32_t now = millis();
  if (now - lastReport < MOTOR_REPORT_MS) return;
  lastReport = now;

  portENTER_CRITICAL(&motorMux);
  TestState state = testState;
  MotorStatus m[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) m[i] = motorStatus[i];
  portEXIT_CRITICAL(&motorMux);

  Serial.printf("MOTORS,%u,%.1f,%lld,%.0f,%.1f,%lld,%.0f\n", (unsigned)state,
                m[0].duty, m[0].count, m[0].countsPerSec,
                m[1].duty, m[1].count, m[1].countsPerSec);
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
    sendMotors();
    vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

// ---------- Motors (core 1) ----------

void setupMotors() {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;   // ignored on GPIO34/35
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    const MotorPins& p = MOTOR_PINS[i];
    pinMode(p.ena, OUTPUT);
    digitalWrite(p.ena, LOW);   // coast until a test starts
    ledcAttach(p.in1, PWM_FREQ_HZ, PWM_BITS);
    ledcAttach(p.in2, PWM_FREQ_HZ, PWM_BITS);
    ledcWrite(p.in1, 0);
    ledcWrite(p.in2, 0);
    encoders[i].attachFullQuad(p.encA, p.encB);
    encoders[i].setFilter(250);   // ignore glitches shorter than ~3 us
    encoders[i].clearCount();
  }
}

// duty in % (-100..100); the sign picks the direction
void applyDuty(size_t motor, float duty, bool enabled) {
  const MotorPins& p = MOTOR_PINS[motor];
  uint32_t pwm = (uint32_t)(fabsf(duty) / 100.0f * PWM_MAX + 0.5f);
  ledcWrite(p.in1, duty > 0 ? pwm : 0);
  ledcWrite(p.in2, duty < 0 ? pwm : 0);
  digitalWrite(p.ena, enabled ? HIGH : LOW);
}

void runMotors() {
  static uint32_t lastUpdate = 0;
  static uint32_t lastSpeedMs = 0;
  static int64_t lastCount[MOTOR_COUNT] = {};
  static float countsPerSec[MOTOR_COUNT] = {};
  uint32_t now = millis();
  if (now - lastUpdate < MOTOR_UPDATE_MS) return;
  lastUpdate = now;

  portENTER_CRITICAL(&motorMux);
  if (testState == TestState::Running && now - lastPingMs > MOTOR_WATCHDOG_MS) {
    testState = TestState::Watchdog;
  }
  SquareWave sq = squareWave;
  bool running = testState == TestState::Running;
  bool restart = restartTest;
  restartTest = false;
  portEXIT_CRITICAL(&motorMux);

  if (restart) {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
      encoders[i].clearCount();
      lastCount[i] = 0;
      countsPerSec[i] = 0;
    }
    lastSpeedMs = now;
  }

  bool speedDue = now - lastSpeedMs >= SPEED_WINDOW_MS;
  float dt = (now - lastSpeedMs) / 1000.0f;
  if (speedDue) lastSpeedMs = now;

  MotorStatus status[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    float duty = 0;
    bool active = running && (sq.mask & (1 << i));
    if (active) {
      bool highHalf = (now - sq.startMs) % sq.periodMs < sq.periodMs / 2;
      duty = highHalf ? sq.dutyHigh : sq.dutyLow;
    }
    applyDuty(i, duty, active);

    int64_t count = encoders[i].getCount();
    if (speedDue && dt > 0) {
      countsPerSec[i] = (count - lastCount[i]) / dt;
      lastCount[i] = count;
    }
    status[i] = {duty, count, countsPerSec[i]};
  }

  portENTER_CRITICAL(&motorMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) motorStatus[i] = status[i];
  portEXIT_CRITICAL(&motorMux);
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
