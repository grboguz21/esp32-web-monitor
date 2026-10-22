#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
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
};
constexpr MotorPins MOTOR_PINS[] = {
  {25, 26, 27},   // motor 1
  {18, 19, 23},   // motor 2
};
constexpr size_t MOTOR_COUNT = sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]);

// PWM test: a square wave of duty on any of these output-safe pins
// (not UART0, not the I2C bus, not input-only or flash pins)
constexpr int PWM_TEST_PINS[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27, 32, 33};
constexpr uint32_t PWM_FREQ_HZ = 5000;
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1;
constexpr uint32_t PWM_UPDATE_MS = 5;
constexpr uint32_t PWM_WATCHDOG_MS = 1000;   // stop if the dashboard goes quiet this long
constexpr uint32_t PWM_REPORT_MS = 40;       // 25 Hz

constexpr const char* FIRMWARE_VERSION = "0.7.0";

// Protocol (one message per line):
//   ESP32 -> PC : HB,<uptime_ms>               heartbeat
//                 HELLO,<device>,<version>     identification
//                 INFO,<chip>,<revision>,<cores>,<cpu_mhz>,<flash_bytes>,<heap_bytes>,<mac>,<reset_reason>
//                 HEALTH,<free_heap>,<min_free_heap>,<max_alloc_heap>,<loop_avg_us>,<loop_max_us>,<loop_hz>,<stack_free_bytes>
//                 (loop_* and stack_free_bytes describe the loop() task on core 1)
//                 POWER,<motor>,<voltage_V>,<current_mA>,<power_mW>,<min_voltage_V>,<max_current_mA>
//                 (motor is 1 or 2; averages and extremes over the last report window)
//                 POWER,<motor>,NA             no INA219 found for that motor
//                 PWMOUT,<state>,<pin>,<duty>
//                 (state 0 idle, 1 running, 2 stopped by the watchdog; duty in % on that pin)
//   PC -> ESP32 : HELLO?                       request HELLO and INFO
//                 PING                         dashboard is alive (keeps the PWM test allowed)
//                 PWM,<pin>,<high>,<low>,<period_ms>
//                                              square wave of duty (0..100 %) on a GPIO
//                 STOP                         stop the PWM test

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

struct PwmTest {
  int pin = -1;
  float dutyHigh = 0;
  float dutyLow = 0;
  uint32_t periodMs = 2000;
  uint32_t startMs = 0;
};

// Shared between the telemetry task (core 0: commands, reports) and loop() (core 1: output)
portMUX_TYPE testMux = portMUX_INITIALIZER_UNLOCKED;
PwmTest pwmTest;
TestState testState = TestState::Idle;
uint32_t lastPingMs = 0;
float outputDuty = 0;   // duty currently on the test pin

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

bool isPwmTestPin(int pin) {
  for (int p : PWM_TEST_PINS) if (p == pin) return true;
  return false;
}

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
    portENTER_CRITICAL(&testMux);
    lastPingMs = millis();
    portEXIT_CRITICAL(&testMux);
  } else if (cmd == "STOP") {
    portENTER_CRITICAL(&testMux);
    testState = TestState::Idle;
    portEXIT_CRITICAL(&testMux);
  } else if (cmd.startsWith("PWM,")) {
    int pin;
    float high, low;
    unsigned long period;
    if (sscanf(cmd.c_str(), "PWM,%d,%f,%f,%lu", &pin, &high, &low, &period) == 4 &&
        isPwmTestPin(pin) && period >= 100) {
      portENTER_CRITICAL(&testMux);
      pwmTest.pin = pin;
      pwmTest.dutyHigh = constrain(high, 0.0f, 100.0f);
      pwmTest.dutyLow = constrain(low, 0.0f, 100.0f);
      pwmTest.periodMs = period;
      pwmTest.startMs = millis();
      lastPingMs = millis();
      testState = TestState::Running;
      portEXIT_CRITICAL(&testMux);
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

void sendPwmTest() {
  static uint32_t lastReport = 0;
  uint32_t now = millis();
  if (now - lastReport < PWM_REPORT_MS) return;
  lastReport = now;

  portENTER_CRITICAL(&testMux);
  TestState state = testState;
  int pin = pwmTest.pin;
  float duty = outputDuty;
  portEXIT_CRITICAL(&testMux);

  Serial.printf("PWMOUT,%u,%d,%.1f\n", (unsigned)state, pin, duty);
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
    sendPwmTest();
    vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

// ---------- PWM test output (core 1) ----------

void setupMotorPins() {
  for (const MotorPins& p : MOTOR_PINS) {
    pinMode(p.ena, OUTPUT);
    digitalWrite(p.ena, LOW);   // coast
    pinMode(p.in1, OUTPUT);
    digitalWrite(p.in1, LOW);
    pinMode(p.in2, OUTPUT);
    digitalWrite(p.in2, LOW);
  }
}

// The motor whose IN1/IN2 is this pin, or -1
int motorForPin(int pin) {
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    if (MOTOR_PINS[i].in1 == pin || MOTOR_PINS[i].in2 == pin) return i;
  }
  return -1;
}

void releasePin(int pin) {
  if (pin < 0) return;
  ledcDetach(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  int motor = motorForPin(pin);
  if (motor >= 0) digitalWrite(MOTOR_PINS[motor].ena, LOW);
}

void runPwmTest() {
  static uint32_t lastUpdate = 0;
  static int activePin = -1;   // pin currently attached to the PWM peripheral
  uint32_t now = millis();
  if (now - lastUpdate < PWM_UPDATE_MS) return;
  lastUpdate = now;

  portENTER_CRITICAL(&testMux);
  if (testState == TestState::Running && now - lastPingMs > PWM_WATCHDOG_MS) {
    testState = TestState::Watchdog;
  }
  PwmTest test = pwmTest;
  bool running = testState == TestState::Running;
  portEXIT_CRITICAL(&testMux);

  int wantPin = running ? test.pin : -1;
  if (wantPin != activePin) {
    releasePin(activePin);
    activePin = -1;
    if (wantPin >= 0 && ledcAttach(wantPin, PWM_FREQ_HZ, PWM_BITS)) {
      activePin = wantPin;
      // Driving a motor input: let that L298N channel conduct
      int motor = motorForPin(wantPin);
      if (motor >= 0) digitalWrite(MOTOR_PINS[motor].ena, HIGH);
    }
  }

  float duty = 0;
  if (activePin >= 0) {
    bool highHalf = (now - test.startMs) % test.periodMs < test.periodMs / 2;
    duty = highHalf ? test.dutyHigh : test.dutyLow;
    ledcWrite(activePin, (uint32_t)(duty / 100.0f * PWM_MAX + 0.5f));
  }

  portENTER_CRITICAL(&testMux);
  outputDuty = duty;
  portEXIT_CRITICAL(&testMux);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  setupMotorPins();
  // setup() runs inside the loop task, so this is the task loop() runs in
  mainLoopTask = xTaskGetCurrentTaskHandle();
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", TELEMETRY_STACK_BYTES, nullptr,
                          TELEMETRY_PRIORITY, nullptr, TELEMETRY_CORE);
}

// Core 1: time-critical work only
void loop() {
  measureLoop();
  runPwmTest();
}
