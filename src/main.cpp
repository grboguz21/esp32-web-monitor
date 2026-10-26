#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ESP32Encoder.h>
#include <esp_system.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>

constexpr uint32_t SERIAL_BAUD = 460800;   // room for the 200 Hz log next to the telemetry
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 500;
constexpr uint32_t HEALTH_INTERVAL_MS = 1000;

// Dashboard communication runs on core 0 so loop() keeps core 1 to itself
constexpr BaseType_t TELEMETRY_CORE = 0;
constexpr uint32_t TELEMETRY_STACK_BYTES = 12288;   // Wi-Fi and the WebSocket server run here too
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
// a PI on the speed error, with clamping anti-windup. The controller works in volts; duty is
// volts / the supply voltage, measured by the first INA219 found (battery compensation) or
// SUPPLY_V when there is none. The feedforward curve was measured at FF_TABLE_V.
constexpr float SUPPLY_V = 12.0f;
constexpr float FF_TABLE_V = 12.0f;
constexpr float SUPPLY_FILTER_S = 0.2f;        // smooths the measured supply a little
constexpr float SUPPLY_MIN_V = 6.0f;           // outside this range the reading is not trusted
constexpr float SUPPLY_MAX_V = 14.0f;
constexpr uint32_t SUPPLY_STALE_MS = 500;      // no fresh reading: fall back to SUPPLY_V
constexpr uint32_t BATTERY_REPORT_MS = 500;
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
// Defaults: motor 1 steady-state sweep (12 V, 1 kHz PWM); the 0 RPM point is the stop threshold
constexpr FfPoint FF_PLUS[] = {
  {0, 11.6}, {50.3, 12}, {71.6, 13}, {109.9, 15}, {146.2, 17}, {191.1, 20}, {256.6, 25},
  {320.9, 30}, {440.7, 40}, {558.8, 50}, {675.3, 60}, {892.8, 80}, {1115.1, 100},
};
constexpr FfPoint FF_MINUS[] = {
  {0, 10.0}, {25.3, 11}, {56.9, 12}, {78.4, 13}, {118.1, 15}, {153.5, 17}, {199.4, 20},
  {268.0, 25}, {331.2, 30}, {453.6, 40}, {574.4, 50}, {689.9, 60}, {905.8, 80}, {1127.0, 100},
};
constexpr float DEFAULT_MAX_RPM = 1100.0f;

// Everything that differs from one motor to the next. Each motor has its own, kept in flash
// (SAVE) and loaded at boot; a different motor only needs a new profile, not new firmware.
constexpr size_t FF_MAX_POINTS = 16;
struct FfTable {
  uint8_t count;
  FfPoint points[FF_MAX_POINTS];   // rising rpm; the last point's duty holds above it
};
constexpr uint32_t PROFILE_VERSION = 1;
struct MotorProfile {
  uint32_t version;
  float cpr;
  uint8_t invert;
  float kickPlus;    // % start thresholds (breakaway kick starts here)
  float kickMinus;
  float kp;          // V per RPM
  float ki;          // V per RPM*s
  float tauRefS;
  float ka;          // V per RPM/s
  float maxRpm;
  FfTable ffPlus;
  FfTable ffMinus;
};

enum class RunMode : uint8_t { OpenLoop = 0, Feedforward = 1, FeedforwardPi = 2, Move = 3, Drive = 4 };

// Drive: both speed loops at once, each with its own profile. With sync on, the wheel that
// has turned further slows down and the other speeds up, so equal targets also mean equal
// distance (the robot keeps straight). The correction is capped at a share of the target.
constexpr float SYNC_MAX_SHARE = 0.2f;

// Move: drive a set number of encoder counts with a trapezoidal speed profile (accelerate,
// cruise, decelerate so it can stop in the distance left), run by the speed loop, then brake
constexpr float MOVE_MIN_RPM = 40.0f;          // final approach speed, above the stuck limit
constexpr int32_t MOVE_TOLERANCE_COUNTS = 2;   // close enough: start braking
constexpr uint32_t MOVE_BRAKE_MS = 300;        // both inputs low (short-circuit brake), then coast
enum class MovePhase : uint8_t { Idle = 0, Moving = 1, Braking = 2, Done = 3, Aborted = 4 };

constexpr const char* FIRMWARE_VERSION = "0.23.0";

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
//                 BATT,<supply_V>,<measured 0|1>  supply the controller uses (0: SUPPLY_V assumed)
//                 PROFILE,<motor>,<cpr>,<invert>,<kick+>,<kick->,<kp>,<ki>,<tau_ms>,<ka>,<max_rpm>
//                 FF,<motor>,<+|->,<rpm>:<duty>;...   that motor's feedforward curve
//                 SAVED,<motor>,<ok>               profile written to flash
//                 WIFI,<0 off|1 joining|2 on>,<ssid>,<ip>,<rssi_dbm>
//                 MOVE,<phase>,<target_counts>,<position_counts>,<profile_rpm>,<final_error_counts>
//                 DRIVE,<active>,<target1>,<target2>,<rpm1>,<rpm2>,<rev1>,<rev2>,<correction_rpm>
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
//                 DRIVE,<rpm1>,<rpm2>[,<sync>]  both speed loops at once (each motor's own sign);
//                                              sync = RPM correction per rev of difference, 0 off
//                 DUTY,<duty1>,<duty2>,<duration_ms>
//                                              drive both motors open loop at once (0 = coast)
//                 STOP                         stop the motor
//                 WIFI?  /  WIFISET,<ssid>,<password>  /  WIFIOFF     home network (kept in flash)
//                 PROFILE?,<motor>             print that motor's profile (PROFILE + two FF lines)
//                 GAIN,<kp>,<ki>[,<motor>]     / REF,<tau_ms>,<ka>[,<motor>]: no motor = both
//                 KICK,<motor>,<plus>,<minus>  start thresholds, %
//                 MAXRPM,<motor>,<rpm>
//                 FF,<motor>,<+|->,<rpm>:<duty>;...   replace a feedforward curve
//                 SAVE,<motor>                 keep the profile in flash (written once idle)
//                 DEFAULTS,<motor>             back to the built-in profile (not saved until SAVE)
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
  float driveRpm[MOTOR_COUNT] = {};   // drive mode: each motor's target, in its own direction
  float syncGain = 0;         // RPM of correction per revolution of difference; 0 = off
  uint32_t driveSeq = 0;      // bumped by every DRIVE that starts a new run
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
MotorProfile profiles[MOTOR_COUNT];   // under runMux; cpr and invert live in motorCpr / encoderInvert
bool profileSavePending[MOTOR_COUNT] = {};
float measuredSupplyV = SUPPLY_V;   // filtered INA219 bus voltage, written on core 0
uint32_t supplyMeasuredMs = 0;      // millis() of the last good reading, 0 = never
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

struct DriveStatus {
  bool active = false;
  float target[MOTOR_COUNT] = {};
  float speed[MOTOR_COUNT] = {};
  float pos[MOTOR_COUNT] = {};   // revolutions since the drive started, motor's own direction
  float correction = 0;          // RPM taken from the leading wheel and given to the other
};
DriveStatus driveStatus;
uint32_t driveSeqCounter = 0;
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

constexpr const char* WIFI_HOSTNAME = "esp32-robot";
constexpr uint16_t WS_PORT = 81;
constexpr size_t WS_OUT_MAX = 16384;   // drop output beyond this if Wi-Fi stalls

WebSocketsServer ws(WS_PORT);
String wsOut;                // lines waiting for the next flush
int wsClients = 0;
String wifiSsid, wifiPass;
wl_status_t wifiLast = WL_IDLE_STATUS;

class DualOut : public Print {
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (wsClients > 0 && wsOut.length() < WS_OUT_MAX) wsOut += (char)c;
    return 1;
  }
  size_t write(const uint8_t* buf, size_t n) override {
    Serial.write(buf, n);
    if (wsClients > 0 && wsOut.length() + n < WS_OUT_MAX) wsOut.concat((const char*)buf, n);
    return n;
  }
};
DualOut out;

void printHello() {
  out.printf("HELLO,%s,%s\n", DEVICE_NAME, FIRMWARE_VERSION);
}

void printInfo() {
  uint64_t mac = ESP.getEfuseMac();
  out.printf("INFO,%s,%u,%u,%lu,%lu,%lu,%02X:%02X:%02X:%02X:%02X:%02X,%s\n",
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

// ---------- Wi-Fi ----------
// Optional: with a network saved (WIFISET), the board joins it as esp32-robot.local and
// offers the same line protocol over a WebSocket on port 81. Everything printed goes to USB
// and to connected WebSocket pages; their lines go to the same command handler. All of it
// runs in the telemetry task on core 0, so the control loop on core 1 is not touched.

void handleCommand(const String& cmd);

void onWsEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    wsClients++;
  } else if (type == WStype_DISCONNECTED) {
    wsClients = max(0, wsClients - 1);
  } else if (type == WStype_TEXT) {
    String text((const char*)payload, length);
    int start = 0;
    while (start < (int)text.length()) {
      int end = text.indexOf('\n', start);
      if (end < 0) end = text.length();
      String line = text.substring(start, end);
      line.trim();
      if (line.length()) handleCommand(line);
      start = end + 1;
    }
  }
}

void printWifi() {
  wl_status_t st = WiFi.status();
  int state = wifiSsid.isEmpty() ? 0 : st == WL_CONNECTED ? 2 : 1;
  out.printf("WIFI,%d,%s,%s,%d\n", state, wifiSsid.c_str(),
             state == 2 ? WiFi.localIP().toString().c_str() : "", state == 2 ? (int)WiFi.RSSI() : 0);
}

void startWifi() {
  Preferences prefs;
  if (prefs.begin("wifi", true)) {
    wifiSsid = prefs.getString("ssid", "");
    wifiPass = prefs.getString("pass", "");
    prefs.end();
  }
  WiFi.disconnect(true);
  if (wifiSsid.isEmpty()) {
    WiFi.mode(WIFI_OFF);
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
}

// WIFISET,<ssid>,<password>   (the password is the rest of the line, commas allowed)
void setWifi(const String& cmd) {
  int c1 = cmd.indexOf(',');
  int c2 = cmd.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return;
  String ssid = cmd.substring(c1 + 1, c2);
  String pass = cmd.substring(c2 + 1);
  Preferences prefs;
  if (prefs.begin("wifi", false)) {
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
  }
  startWifi();
  printWifi();
}

void serviceWifi() {
  static bool wsStarted = false;
  if (!wsStarted && !wifiSsid.isEmpty()) {
    ws.begin();
    ws.onEvent(onWsEvent);
    wsStarted = true;
  }
  if (wsStarted) ws.loop();
  wl_status_t st = WiFi.status();
  if (st != wifiLast) {   // joined or lost the network
    wifiLast = st;
    if (st == WL_CONNECTED) {
      MDNS.end();
      if (MDNS.begin(WIFI_HOSTNAME)) MDNS.addService("ws", "tcp", WS_PORT);
    }
    if (!wifiSsid.isEmpty()) printWifi();
  }
  if (wsOut.length() && wsClients > 0) {   // one frame per pass instead of one per line
    ws.broadcastTXT(wsOut);
    wsOut = "";
  } else if (wsClients == 0) {
    wsOut = "";
  }
}

// ---------- Motor profiles ----------

MotorProfile defaultProfile() {
  MotorProfile p = {};
  p.version = PROFILE_VERSION;
  p.cpr = DEFAULT_CPR;
  p.invert = 0;
  p.kickPlus = KICK_START_PLUS;
  p.kickMinus = KICK_START_MINUS;
  p.kp = DEFAULT_KP;
  p.ki = DEFAULT_KI;
  p.tauRefS = DEFAULT_TAU_REF_S;
  p.ka = DEFAULT_KA;
  p.maxRpm = DEFAULT_MAX_RPM;
  p.ffPlus.count = sizeof(FF_PLUS) / sizeof(FF_PLUS[0]);
  memcpy(p.ffPlus.points, FF_PLUS, sizeof(FF_PLUS));
  p.ffMinus.count = sizeof(FF_MINUS) / sizeof(FF_MINUS[0]);
  memcpy(p.ffMinus.points, FF_MINUS, sizeof(FF_MINUS));
  return p;
}

// Flash namespace "motor1", "motor2"; a blob of the struct, ignored if the version differs
void loadProfiles() {
  Preferences prefs;
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    MotorProfile p = defaultProfile();
    char ns[8];
    snprintf(ns, sizeof(ns), "motor%u", (unsigned)(i + 1));
    if (prefs.begin(ns, true)) {
      MotorProfile stored;
      if (prefs.getBytes("profile", &stored, sizeof(stored)) == sizeof(stored) &&
          stored.version == PROFILE_VERSION) {
        p = stored;
      }
      prefs.end();
    }
    profiles[i] = p;
    motorCpr[i] = p.cpr;
    encoderInvert[i] = p.invert;
  }
}

// Writing flash stalls both cores for a moment, so it waits until no motor is running
void saveProfiles() {
  portENTER_CRITICAL(&runMux);
  bool running = runState == RunState::Running;
  portEXIT_CRITICAL(&runMux);
  if (running) return;
  Preferences prefs;
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    if (!profileSavePending[i]) continue;
    profileSavePending[i] = false;
    portENTER_CRITICAL(&runMux);
    MotorProfile p = profiles[i];
    p.cpr = motorCpr[i];
    p.invert = encoderInvert[i];
    portEXIT_CRITICAL(&runMux);
    char ns[8];
    snprintf(ns, sizeof(ns), "motor%u", (unsigned)(i + 1));
    if (prefs.begin(ns, false)) {
      bool ok = prefs.putBytes("profile", &p, sizeof(p)) == sizeof(p);
      prefs.end();
      out.printf("SAVED,%u,%d\n", (unsigned)(i + 1), ok ? 1 : 0);
    }
  }
}

void printFfTable(size_t motor, char dir, const FfTable& t) {
  String line = "FF," + String(motor + 1) + "," + dir + ",";
  for (size_t k = 0; k < t.count; k++) {
    if (k) line += ";";
    line += String(t.points[k].rpm, 1) + ":" + String(t.points[k].duty, 1);
  }
  out.println(line);
}

void printProfile(size_t motor) {
  portENTER_CRITICAL(&runMux);
  MotorProfile p = profiles[motor];
  float cpr = motorCpr[motor];
  bool invert = encoderInvert[motor];
  portEXIT_CRITICAL(&runMux);
  out.printf("PROFILE,%u,%.2f,%d,%.1f,%.1f,%.5f,%.4f,%.0f,%.5f,%.0f\n", (unsigned)(motor + 1), cpr,
                invert ? 1 : 0, p.kickPlus, p.kickMinus, p.kp, p.ki, p.tauRefS * 1000.0f, p.ka, p.maxRpm);
  printFfTable(motor, '+', p.ffPlus);
  printFfTable(motor, '-', p.ffMinus);
}

// FF,<motor>,<+|->,<rpm>:<duty>;<rpm>:<duty>;...  rising rpm, 2..FF_MAX_POINTS points
void parseFfTable(const String& cmd) {
  int c1 = cmd.indexOf(','), c2 = cmd.indexOf(',', c1 + 1), c3 = cmd.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return;
  unsigned motor = cmd.substring(c1 + 1, c2).toInt();
  String dir = cmd.substring(c2 + 1, c3);
  if (motor < 1 || motor > MOTOR_COUNT || (dir != "+" && dir != "-")) return;
  FfTable t = {};
  String rest = cmd.substring(c3 + 1);
  int start = 0;
  while (start < (int)rest.length() && t.count < FF_MAX_POINTS) {
    int end = rest.indexOf(';', start);
    if (end < 0) end = rest.length();
    String pair = rest.substring(start, end);
    int colon = pair.indexOf(':');
    if (colon < 0) return;
    float rpm = pair.substring(0, colon).toFloat();
    float duty = pair.substring(colon + 1).toFloat();
    if (rpm < 0 || duty < 0 || duty > 100) return;
    if (t.count > 0 && rpm <= t.points[t.count - 1].rpm) return;   // must rise
    t.points[t.count++] = {rpm, duty};
    start = end + 1;
  }
  if (t.count < 2) return;
  portENTER_CRITICAL(&runMux);
  (dir == "+" ? profiles[motor - 1].ffPlus : profiles[motor - 1].ffMinus) = t;
  portEXIT_CRITICAL(&runMux);
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
        maxRpm <= profiles[motor - 1].maxRpm * 1.1f && accel >= 100 && accel <= 20000) {
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
  } else if (cmd.startsWith("DRIVE,")) {
    float r1, r2, sync = 0;
    int n = sscanf(cmd.c_str(), "DRIVE,%f,%f,%f", &r1, &r2, &sync);
    if (n >= 2 && sync >= 0) {
      portENTER_CRITICAL(&runMux);
      bool continuing = runState == RunState::Running && motorRun.mode == RunMode::Drive;
      motorRun.motor = ALL_MOTORS;
      motorRun.mode = RunMode::Drive;
      motorRun.driveRpm[0] = constrain(r1, -profiles[0].maxRpm, profiles[0].maxRpm);
      motorRun.driveRpm[1] = constrain(r2, -profiles[1].maxRpm, profiles[1].maxRpm);
      motorRun.syncGain = sync;
      // a new speed while driving keeps the distance count, so sync carries on across changes
      if (!continuing) motorRun.driveSeq = ++driveSeqCounter;
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
    unsigned motor = 0;
    int n = sscanf(cmd.c_str(), "GAIN,%f,%f,%u", &kp, &ki, &motor);
    if (n >= 2 && kp >= 0 && ki >= 0 && motor <= MOTOR_COUNT) {
      portENTER_CRITICAL(&runMux);
      for (size_t i = 0; i < MOTOR_COUNT; i++) {
        if (n == 3 && i != motor - 1) continue;
        profiles[i].kp = kp;
        profiles[i].ki = ki;
      }
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("REF,")) {
    float tauMs, ka;
    unsigned motor = 0;
    int n = sscanf(cmd.c_str(), "REF,%f,%f,%u", &tauMs, &ka, &motor);
    if (n >= 2 && tauMs >= 5 && ka >= 0 && motor <= MOTOR_COUNT) {
      portENTER_CRITICAL(&runMux);
      for (size_t i = 0; i < MOTOR_COUNT; i++) {
        if (n == 3 && i != motor - 1) continue;
        profiles[i].tauRefS = tauMs / 1000.0f;
        profiles[i].ka = ka;
      }
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd == "WIFI?") {
    printWifi();
  } else if (cmd.startsWith("WIFISET,")) {
    setWifi(cmd);
  } else if (cmd == "WIFIOFF") {
    Preferences prefs;
    if (prefs.begin("wifi", false)) {
      prefs.clear();
      prefs.end();
    }
    startWifi();
    printWifi();
  } else if (cmd.startsWith("PROFILE?,")) {
    unsigned motor;
    if (sscanf(cmd.c_str(), "PROFILE?,%u", &motor) == 1 && motor >= 1 && motor <= MOTOR_COUNT) {
      printProfile(motor - 1);
    }
  } else if (cmd.startsWith("KICK,")) {
    unsigned motor;
    float plus, minus;
    if (sscanf(cmd.c_str(), "KICK,%u,%f,%f", &motor, &plus, &minus) == 3 && motor >= 1 &&
        motor <= MOTOR_COUNT && plus > 0 && plus <= 100 && minus > 0 && minus <= 100) {
      portENTER_CRITICAL(&runMux);
      profiles[motor - 1].kickPlus = plus;
      profiles[motor - 1].kickMinus = minus;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("MAXRPM,")) {
    unsigned motor;
    float rpm;
    if (sscanf(cmd.c_str(), "MAXRPM,%u,%f", &motor, &rpm) == 2 && motor >= 1 && motor <= MOTOR_COUNT &&
        rpm >= 50 && rpm <= 20000) {
      portENTER_CRITICAL(&runMux);
      profiles[motor - 1].maxRpm = rpm;
      portEXIT_CRITICAL(&runMux);
    }
  } else if (cmd.startsWith("FF,")) {
    parseFfTable(cmd);
  } else if (cmd.startsWith("SAVE,")) {
    unsigned motor;
    if (sscanf(cmd.c_str(), "SAVE,%u", &motor) == 1 && motor >= 1 && motor <= MOTOR_COUNT) {
      profileSavePending[motor - 1] = true;   // written once the motors are idle
    }
  } else if (cmd.startsWith("DEFAULTS,")) {
    unsigned motor;
    if (sscanf(cmd.c_str(), "DEFAULTS,%u", &motor) == 1 && motor >= 1 && motor <= MOTOR_COUNT) {
      MotorProfile p = defaultProfile();
      portENTER_CRITICAL(&runMux);
      profiles[motor - 1] = p;
      motorCpr[motor - 1] = p.cpr;
      encoderInvert[motor - 1] = p.invert;
      portEXIT_CRITICAL(&runMux);
      printProfile(motor - 1);
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
    } else if (buffer.length() < 400) {   // FF tables are the longest lines
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
    out.printf("HB,%lu\n", now);
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

  out.printf("HEALTH,%lu,%lu,%lu,%.2f,%lu,%lu,%lu\n",
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

PowerChannel* firstReadyChannel() {
  for (PowerChannel& ch : powerChannels) if (ch.ready) return &ch;
  return nullptr;
}

// Battery compensation: low-pass the supply voltage for the speed loop on core 1
void updateSupply(float voltage, uint32_t now) {
  constexpr float alpha = (POWER_SAMPLE_MS / 1000.0f) / (SUPPLY_FILTER_S + POWER_SAMPLE_MS / 1000.0f);
  portENTER_CRITICAL(&runMux);
  bool fresh = supplyMeasuredMs != 0 && now - supplyMeasuredMs < SUPPLY_STALE_MS;
  measuredSupplyV = fresh ? measuredSupplyV + alpha * (voltage - measuredSupplyV) : voltage;
  supplyMeasuredMs = now;
  portEXIT_CRITICAL(&runMux);
}

void sendBattery() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < BATTERY_REPORT_MS) return;
  last = now;
  portENTER_CRITICAL(&runMux);
  bool fresh = supplyMeasuredMs != 0 && now - supplyMeasuredMs < SUPPLY_STALE_MS;
  float v = fresh ? measuredSupplyV : SUPPLY_V;
  portEXIT_CRITICAL(&runMux);
  out.printf("BATT,%.2f,%d\n", v, fresh ? 1 : 0);
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
    if (&ch == firstReadyChannel() && voltage >= SUPPLY_MIN_V && voltage <= SUPPLY_MAX_V) {
      updateSupply(voltage, now);
    }
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
      out.printf("POWER,%u,%.3f,%.1f,%.1f,%.3f,%.1f\n", motor,
                    w.sumVoltage / n, w.sumCurrentMa / n, w.sumPowerMw / n,
                    w.minVoltage, w.maxCurrentMa);
      w = PowerWindow();
    } else if (!ch.ready && now - ch.lastReport >= POWER_ABSENT_REPORT_MS) {
      ch.lastReport = now;
      out.printf("POWER,%u,NA\n", motor);
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
  out.printf("RUNOUT,%u,%d,%.1f,%lu\n", (unsigned)state, run.motor + 1, duty,
                (unsigned long)remaining);
  out.printf("DUTYOUT,%.1f,%.1f\n", duties[0], duties[1]);

  portENTER_CRITICAL(&runMux);
  EncoderStatus enc[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) enc[i] = encoderStatus[i];
  portEXIT_CRITICAL(&runMux);
  out.printf("ENC,%lld,%.0f,%lld,%.0f\n", enc[0].count, enc[0].countsPerSec,
                enc[1].count, enc[1].countsPerSec);

  portENTER_CRITICAL(&runMux);
  ControlStatus ctl = controlStatus;
  portEXIT_CRITICAL(&runMux);
  out.printf("SPD,%u,%.1f,%.1f,%.1f,%.1f,%.1f\n", (unsigned)run.mode, run.targetRpm, ctl.refRpm,
                ctl.speedRpm, ctl.ffDuty, ctl.piDuty);

  portENTER_CRITICAL(&runMux);
  MoveStatus mv = moveStatus;
  portEXIT_CRITICAL(&runMux);
  out.printf("MOVE,%u,%ld,%ld,%.1f,%ld\n", (unsigned)mv.phase, (long)mv.targetCounts,
                (long)mv.positionCounts, mv.profileRpm, (long)mv.finalErrorCounts);

  portENTER_CRITICAL(&runMux);
  DriveStatus dr = driveStatus;
  portEXIT_CRITICAL(&runMux);
  out.printf("DRIVE,%d,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.1f\n", dr.active ? 1 : 0, dr.target[0],
                dr.target[1], dr.speed[0], dr.speed[1], dr.pos[0], dr.pos[1], dr.correction);
}

void sendLog() {
  LogEntry e;
  while (xQueueReceive(logQueue, &e, 0) == pdTRUE) {
    out.printf("L,%lu,%.1f,%ld,%ld,%.1f,%.1f,%.1f\n", (unsigned long)e.tUs, e.duty,
                  (long)e.count[0], (long)e.count[1], e.targetRpm, e.refRpm, e.speedRpm);
  }
}

// Runs on core 0 and owns the serial port and I2C bus, so this I/O never slows loop() down
void telemetryTask(void*) {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  printHello();
  printInfo();
  startWifi();
  for (;;) {
    serviceWifi();
    readCommands();
    sendHeartbeat();
    sendHealth();
    samplePower();
    sendPower();
    sendBattery();
    saveProfiles();
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

struct Gains {
  float kp;
  float ki;
  float ka;
  float tauRefS;
  float supplyV;   // volts -> duty
  float kickPlus;
  float kickMinus;
  FfTable ffPlus;
  FfTable ffMinus;
};

// Duty (%) that holds `rpm` in steady state, by linear interpolation in the measured
// curve for that direction; 0 RPM gives 0 (stopped), beyond the last point 100 %
float interpolateFf(const FfTable& table, float rpm) {
  size_t n = table.count;
  if (n == 0) return 0;
  const FfPoint* p = table.points;
  if (rpm >= p[n - 1].rpm) return p[n - 1].duty;
  for (size_t i = 1; i < n; i++) {
    if (rpm <= p[i].rpm) {
      float f = (rpm - p[i - 1].rpm) / (p[i].rpm - p[i - 1].rpm);
      return p[i - 1].duty + f * (p[i].duty - p[i - 1].duty);
    }
  }
  return p[n - 1].duty;
}

float feedforwardDuty(const Gains& g, float rpm) {
  if (rpm > 0) return interpolateFf(g.ffPlus, rpm);
  if (rpm < 0) return -interpolateFf(g.ffMinus, -rpm);
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
    float start = run.targetRpm > 0 ? g.kickPlus : g.kickMinus;
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

  float ffV = feedforwardDuty(g, pi.refRpm) / 100.0f * FF_TABLE_V + g.ka * refRate;
  ffV = constrain(ffV, -g.supplyV, g.supplyV);
  ffDuty = ffV / g.supplyV * 100.0f;
  if (run.mode != RunMode::FeedforwardPi) return ffDuty;

  float error = pi.refRpm - speedRpm;
  float pV = g.kp * error;
  float candidate = pi.integralV + g.ki * error * dt;
  float uCandidate = ffV + pV + candidate;
  // Clamping: integrate unless the output is saturated and the error pushes it further
  bool saturated = fabsf(uCandidate) > g.supplyV;
  if (!saturated || (uCandidate > 0) != (error > 0)) pi.integralV = candidate;
  float u = constrain(ffV + pV + pi.integralV, -g.supplyV, g.supplyV);
  piDuty = (u - ffV) / g.supplyV * 100.0f;
  return u / g.supplyV * 100.0f;
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
  static PiState drivePi[MOTOR_COUNT];
  static int64_t driveStart[MOTOR_COUNT] = {};
  static uint32_t driveSeqSeen = 0;
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
  bool supplyFresh = supplyMeasuredMs != 0 && now - supplyMeasuredMs < SUPPLY_STALE_MS;
  const MotorProfile& prof = profiles[run.motor < (int)MOTOR_COUNT ? run.motor : 0];
  float supplyV = supplyFresh ? measuredSupplyV : SUPPLY_V;
  Gains gains = {prof.kp, prof.ki, prof.ka, prof.tauRefS, supplyV,
                 prof.kickPlus, prof.kickMinus, prof.ffPlus, prof.ffMinus};
  bool driving = running && run.mode == RunMode::Drive;
  Gains driveGains[MOTOR_COUNT];
  float cprs[MOTOR_COUNT];
  if (driving) {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
      const MotorProfile& p = profiles[i];
      driveGains[i] = {p.kp, p.ki, p.ka, p.tauRefS, supplyV, p.kickPlus, p.kickMinus, p.ffPlus, p.ffMinus};
      cprs[i] = motorCpr[i];
    }
  }
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

  // Drive: one speed loop per motor, the targets nudged by the distance difference
  float driveDuty[MOTOR_COUNT] = {};
  DriveStatus ds;
  if (driving) {
    int64_t count[MOTOR_COUNT];
    portENTER_CRITICAL(&runMux);
    for (size_t i = 0; i < MOTOR_COUNT; i++) count[i] = encoderStatus[i].count;
    portEXIT_CRITICAL(&runMux);
    if (run.driveSeq != driveSeqSeen) {   // a new drive: distances count from here
      driveSeqSeen = run.driveSeq;
      for (size_t i = 0; i < MOTOR_COUNT; i++) {
        drivePi[i] = PiState();
        driveStart[i] = count[i];
      }
    }
    float t0 = run.driveRpm[0], t1 = run.driveRpm[1];
    float pos0 = (count[0] - driveStart[0]) / cprs[0];
    float pos1 = (count[1] - driveStart[1]) / cprs[1];
    float correction = 0;
    // Sync only makes sense when both wheels should cover the same distance
    if (run.syncGain > 0 && t0 != 0 && t1 != 0 && fabsf(fabsf(t0) - fabsf(t1)) < 0.5f) {
      float ahead = (t0 > 0 ? pos0 : -pos0) - (t1 > 0 ? pos1 : -pos1);   // revs motor 1 leads by
      float cap = SYNC_MAX_SHARE * fabsf(t0);
      correction = constrain(run.syncGain * ahead, -cap, cap);
    }
    float target[MOTOR_COUNT] = {
      t0 - (t0 > 0 ? 1 : -1) * correction / 2,
      t1 + (t1 > 0 ? 1 : -1) * correction / 2,
    };
    if (t0 == 0) target[0] = 0;
    if (t1 == 0) target[1] = 0;
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
      MotorRun r = run;
      r.motor = i;
      r.mode = RunMode::FeedforwardPi;
      r.targetRpm = target[i];
      float ff, pid;
      driveDuty[i] = speedControl(drivePi[i], r, speedRpm[i], driveGains[i], ff, pid);
    }
    ds = {true, {target[0], target[1]}, {speedRpm[0], speedRpm[1]}, {pos0, pos1}, correction};
  } else {
    for (size_t i = 0; i < MOTOR_COUNT; i++) drivePi[i] = PiState();
    ds = driveStatus;   // keep the last numbers on screen, marked inactive
    ds.active = false;
  }
  portENTER_CRITICAL(&runMux);
  driveStatus = ds;
  portEXIT_CRITICAL(&runMux);

  bool allActive = running && run.motor == ALL_MOTORS && run.mode == RunMode::OpenLoop;
  float applied[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    bool active = motorActive && i == (size_t)run.motor;
    applied[i] = driving ? driveDuty[i] : allActive ? run.duties[i] : active ? duty : 0;
    applyDuty(i, applied[i], driving || active || (allActive && applied[i] != 0));
  }
  if (allActive) duty = run.duties[0];
  if (driving) duty = driveDuty[0];
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
  loadProfiles();
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
