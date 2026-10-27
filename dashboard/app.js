// Must match SERIAL_BAUD in src/main.cpp
const BAUD_RATE = 460800;
// ESP32 sends a heartbeat every 500 ms; consider it lost after 3 missed beats
const HEARTBEAT_TIMEOUT_MS = 1500;
const MAX_LOG_LINES = 200;

// USB-serial chips commonly found on ESP32 boards, keyed by "vendorId:productId"
const KNOWN_CHIPS = {
  '1a86:7523': 'CH340',
  '1a86:55d4': 'CH9102',
  '10c4:ea60': 'CP210x',
  '0403:6001': 'FTDI',
};

const els = {
  connectBtn: document.getElementById('connectBtn'),
  wifiHost: document.getElementById('wifiHost'),
  wifiConnectBtn: document.getElementById('wifiConnectBtn'),
  wifiStatus: document.getElementById('wifiStatus'),
  wifiSsid: document.getElementById('wifiSsid'),
  wifiPass: document.getElementById('wifiPass'),
  wifiSaveBtn: document.getElementById('wifiSaveBtn'),
  wifiForgetBtn: document.getElementById('wifiForgetBtn'),
  addPortBtn: document.getElementById('addPortBtn'),
  portSelect: document.getElementById('portSelect'),
  unsupported: document.getElementById('unsupported'),
  statusCard: document.getElementById('statusCard'),
  statusLabel: document.getElementById('statusLabel'),
  deviceName: document.getElementById('deviceName'),
  firmwareVersion: document.getElementById('firmwareVersion'),
  uptime: document.getElementById('uptime'),
  lastBeat: document.getElementById('lastBeat'),
  resetReason: document.getElementById('resetReason'),
  chipModel: document.getElementById('chipModel'),
  chipCpu: document.getElementById('chipCpu'),
  chipFlash: document.getElementById('chipFlash'),
  chipMac: document.getElementById('chipMac'),
  healthCard: document.getElementById('healthCard'),
  hFree: document.getElementById('hFree'),
  hFreeBar: document.getElementById('hFreeBar'),
  hFreeSub: document.getElementById('hFreeSub'),
  hLoopAvg: document.getElementById('hLoopAvg'),
  hLoopMax: document.getElementById('hLoopMax'),
  hStack: document.getElementById('hStack'),
  hStackBar: document.getElementById('hStackBar'),
  hStackSub: document.getElementById('hStackSub'),
  hBatt: document.getElementById('hBatt'),
  hBattBar: document.getElementById('hBattBar'),
  hBattSub: document.getElementById('hBattSub'),
  hLoopBar: document.getElementById('hLoopBar'),
  powerCards: document.querySelectorAll('.power'),
  testCard: document.getElementById('testCard'),
  testBadge: document.getElementById('testBadge'),
  testDuty1: document.getElementById('testDuty1'),
  testDuty2: document.getElementById('testDuty2'),
  testNow1: document.getElementById('testNow1'),
  testNow2: document.getElementById('testNow2'),
  testDuration: document.getElementById('testDuration'),
  testStartBtn: document.getElementById('testStartBtn'),
  spdCard: document.getElementById('spdCard'),
  spdBadge: document.getElementById('spdBadge'),
  spdMotor: document.getElementById('spdMotor'),
  spdMode: document.getElementById('spdMode'),
  spdTarget: document.getElementById('spdTarget'),
  spdKp: document.getElementById('spdKp'),
  spdKi: document.getElementById('spdKi'),
  spdTauRef: document.getElementById('spdTauRef'),
  spdKa: document.getElementById('spdKa'),
  spdStartBtn: document.getElementById('spdStartBtn'),
  spdStopBtn: document.getElementById('spdStopBtn'),
  spdTargetNow: document.getElementById('spdTargetNow'),
  spdSpeedNow: document.getElementById('spdSpeedNow'),
  spdErrorNow: document.getElementById('spdErrorNow'),
  spdDutyNow: document.getElementById('spdDutyNow'),
  spdDutySplit: document.getElementById('spdDutySplit'),
  spdProfile: document.getElementById('spdProfile'),
  spdDefaultsBtn: document.getElementById('spdDefaultsBtn'),
  mvBadge: document.getElementById('mvBadge'),
  mvMotor: document.getElementById('mvMotor'),
  mvUnit: document.getElementById('mvUnit'),
  mvAmount: document.getElementById('mvAmount'),
  mvWheel: document.getElementById('mvWheel'),
  mvMaxRpm: document.getElementById('mvMaxRpm'),
  mvAccel: document.getElementById('mvAccel'),
  mvStartBtn: document.getElementById('mvStartBtn'),
  mvStopBtn: document.getElementById('mvStopBtn'),
  mvTargetNow: document.getElementById('mvTargetNow'),
  mvTargetSub: document.getElementById('mvTargetSub'),
  mvTraveled: document.getElementById('mvTraveled'),
  mvTraveledSub: document.getElementById('mvTraveledSub'),
  mvRemaining: document.getElementById('mvRemaining'),
  mvProfile: document.getElementById('mvProfile'),
  mvFinalError: document.getElementById('mvFinalError'),
  mvFinalSub: document.getElementById('mvFinalSub'),
  mvChartTitle: document.getElementById('mvChartTitle'),
  mvEstimate: document.getElementById('mvEstimate'),
  encCard: document.getElementById('encCard'),
  encBadge: document.getElementById('encBadge'),
  encChartTitle: document.getElementById('encChartTitle'),
  testStopBtn: document.getElementById('testStopBtn'),
  log: document.getElementById('log'),
  clearLogBtn: document.getElementById('clearLogBtn'),
};

const { t } = I18N;

// esp_reset_reason() names sent by the firmware -> severity (labels live in i18n.js)
const RESET_LEVELS = {
  POWERON: 'ok',
  EXT: 'ok',
  SW: 'ok',
  DEEPSLEEP: 'ok',
  PANIC: 'bad',
  INT_WDT: 'bad',
  TASK_WDT: 'bad',
  WDT: 'bad',
  BROWNOUT: 'bad',
  SDIO: 'ok',
};

function resetReason(name) {
  const known = name in RESET_LEVELS;
  return [t(`reset.${known ? name : 'UNKNOWN'}`), known ? RESET_LEVELS[name] : 'bad'];
}

const HEAP_WARN_RATIO = 0.2;     // warn when less than 20 % of the heap is free
const STACK_WARN_BYTES = 1024;
const LOOP_STACK_BYTES = 8192;   // Arduino loop task stack
const LOOP_BUDGET_US = 5000;     // control tick

let knownPorts = [];    // ports this page already has permission for
let port = null;        // the open port
let reader = null;
let readableClosed = null;
let device = null;      // { name, version }
let info = null;        // chip info and reset reason, from INFO
let health = null;      // latest HEALTH values
let battery = null;     // { volts, measured } from BATT
let wifi = null;        // { state 0 off / 1 joining / 2 on, ssid, ip, rssi } from WIFI
const WIFI_HOST_STORAGE_KEY = 'esp32-dashboard-wifi-host';
// Motor profiles live on the board (flash); the page reads them on connect and edits them
const boardProfile = { 1: null, 2: null };   // { cpr, invert, kickPlus, kickMinus, kp, ki, tauMs, ka, maxRpm, ffPlus, ffMinus }
const profileSaved = { 1: false, 2: false };
const saveTimers = {};
// 3S Li-ion: 12.6 V full, warn below 10.2 V, critical below 9.6 V
const BATT_FULL_V = 12.6;
const BATT_EMPTY_V = 9.6;
const BATT_LOW_V = 10.2;
// Power data per motor (1 and 2), one INA219 each
const MOTORS = [1, 2];
const newMotorPower = () => ({
  power: null,          // latest POWER averages: { voltage, currentMa, powerMw }
  sensor: 'unknown',    // 'unknown' (no POWER yet), 'missing' or 'ok'
  minVoltage: null,     // extremes since connecting or the last reset
  maxCurrentMa: null,
});
const motorPower = { 1: newMotorPower(), 2: newMotorPower() };

let demoTimer = null;   // fake POWER data for previewing the Power card without a sensor

const chartLabels = {
  empty: () => t('noData'),
  none: () => t('noSeries'),
  axis: (s) => (s === 0 ? t('now') : t('axisAgo', { n: s })),
  ago: (s) => (s === 0 ? t('now') : t('secAgo', { n: s })),
};
const motorSeries = MOTORS.map((m) => ({
  key: m,
  label: () => t(`motor${m}`),
  short: `M${m}`,
  color: `--series-${m}`,
}));
const currentChart = new LiveChart(document.getElementById('currentChart'), {
  series: motorSeries,
  legend: document.getElementById('currentLegend'),
  format: (ma) => formatCurrent(ma),
  minSpan: 50,
  labels: chartLabels,
});
const voltageChart = new LiveChart(document.getElementById('voltageChart'), {
  series: motorSeries,
  legend: document.getElementById('voltageLegend'),
  format: (v) => `${num(v, 2)} V`,
  tickFormat: (v) => `${num(v, 1)} V`,
  minSpan: 0.5,
  labels: chartLabels,
});

// Motor control: both motors open loop (DUTY), reported back as RUNOUT and DUTYOUT
const TEST_STATES = ['testIdle', 'testRunning', 'testWatchdog', 'testDone'];
// Motors (board labels): each one's L298N inputs, + drives IN1 and - drives IN2
const TEST_TARGETS = {
  1: { label: 'motor1', in1: 'D26', in2: 'D27' },
  2: { label: 'motor2', in1: 'D19', in2: 'D23' },
};
let testState = null;    // null until the firmware reports RUNOUT, then 0 idle / 1 running / 2 watchdog / 3 done
let testOutput = null;   // { motor, duty, remainingMs }; motor 4 = both (DUTY)
const motorDuty = { 1: 0, 2: 0 };   // duty on each motor right now, from DUTYOUT
let simTimer = null;     // simulated run while demo data is on and no board is connected

// Encoders: latest ENC report, counts per output-shaft revolution and sign flip per motor
const ENC_STORAGE_KEY = 'esp32-dashboard-encoder';
let encSettings = { cpr: {}, invert: {} };
try { encSettings = { ...encSettings, ...JSON.parse(localStorage.getItem(ENC_STORAGE_KEY)) }; } catch { /* storage unavailable */ }
const newEnc = () => ({ count: null, cps: 0 });
const encData = { 1: newEnc(), 2: newEnc() };
let encSim = null;   // demo: { timer, count: {1,2}, speed: {1,2} }

// Speed control: the firmware's SPD report for the running motor
let spd = null;   // { mode, target, ref, speed, ff, pi }
const spdChart = new LiveChart(document.getElementById('spdChart'), {
  series: [
    { key: 'speed', label: () => t('spdSpeedNow'), short: () => t('spdSpeedNow'), color: '--series-1' },
    { key: 'ref', label: () => t('spdRefNow'), short: () => t('spdRefNow'), color: '--series-3' },
    { key: 'target', label: () => t('spdTargetNow'), short: () => t('spdTargetNow'), color: '--series-2' },
  ],
  legend: document.getElementById('spdLegend'),
  format: (v) => `${Math.round(v)} RPM`,
  minSpan: 50,
  windowMs: 20000,
  labels: chartLabels,
});
const spdRunning = () => testState === 1 && (spd?.mode === 1 || spd?.mode === 2) && testOutput?.motor === Number(els.spdMotor.value);

// Distance test: the firmware's MOVE report (counts from where the move started)
const MV_STORAGE_KEY = 'esp32-dashboard-move';
const DEFAULT_CPR = 186.2;
const CONTROL_TICK_S = 0.005;   // firmware control period
let mv = null;        // { phase, target, pos, profileRpm, finalError }
let mvMotor = null;   // motor of the last move started from this page
let mvStartedAt = null;   // performance.now() when the last move was sent
let mvTookMs = null;      // start to brake of the last move
const mvChart = new LiveChart(document.getElementById('mvChart'), {
  series: [
    { key: 'pos', label: () => t('mvPosition'), short: () => t('mvPosition'), color: '--series-1' },
    { key: 'target', label: () => t('mvTargetNow'), short: () => t('mvTargetNow'), color: '--series-2' },
  ],
  legend: document.getElementById('mvLegend'),
  format: (v) => (els.mvUnit.value === 'm' ? `${num(v, 2)} m` : num(v, 2)),
  minSpan: 0.5,
  windowMs: 20000,
  labels: chartLabels,
});
const mvSpeedChart = new LiveChart(document.getElementById('mvSpeedChart'), {
  series: [
    { key: 'speed', label: () => t('mvPosition'), short: () => t('mvPosition'), color: '--series-1' },
    { key: 'profile', label: () => t('mvProfileSpeed'), short: () => t('mvProfileSpeed'), color: '--series-2' },
  ],
  legend: document.getElementById('mvSpeedLegend'),
  format: (v) => `${Math.round(v)} RPM`,
  minSpan: 50,
  windowMs: 20000,
  labels: chartLabels,
});

function saveEnc() {
  try { localStorage.setItem(ENC_STORAGE_KEY, JSON.stringify(encSettings)); } catch { /* storage unavailable */ }
}

// RPM when every motor's counts per rev is known, otherwise raw counts per second
const encUseRpm = () => MOTORS.every((m) => encSettings.cpr[m] > 0);
const encSpeedValue = (m, cps) => (encUseRpm() ? (cps * 60) / encSettings.cpr[m] : cps);

const encChart = new LiveChart(document.getElementById('encChart'), {
  series: motorSeries,
  legend: document.getElementById('encLegend'),
  format: (v) => (encUseRpm() ? `${Math.round(v)} RPM` : `${Math.round(v)} ${t('cpsUnit')}`),
  minSpan: 20,
  windowMs: 20000,
  labels: chartLabels,
});

// Text from the board (network names) goes into innerHTML only through this
const escapeHtml = (text) => String(text).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]);

// Numbers in the page's language: 1,25 in Turkish, 1.25 in English
const num = (v, digits) => v.toLocaleString(I18N.lang === 'tr' ? 'tr-TR' : 'en-US',
  { minimumFractionDigits: digits, maximumFractionDigits: digits });
const signedPercent = (d) => `${d > 0 ? '+' : d < 0 ? '−' : ''}${Math.abs(d).toFixed(0)} %`;

const dutyChart = new LiveChart(document.getElementById('dutyChart'), {
  series: motorSeries,
  legend: document.getElementById('dutyLegend'),
  format: signedPercent,
  range: [-100, 100],
  windowMs: 20000,
  labels: chartLabels,
});
const rpmChart = new LiveChart(document.getElementById('rpmChart'), {
  series: motorSeries,
  legend: document.getElementById('rpmLegend'),
  format: (v) => `${Math.round(v)} RPM`,
  minSpan: 50,
  windowMs: 20000,
  labels: chartLabels,
});
// RPM with the measured CPR, or motor 1's until the encoder card has one
const encRpm = (m, cps) => (cps * 60) / (encSettings.cpr[m] > 0 ? encSettings.cpr[m] : DEFAULT_CPR);

let uptimeMs = null;
let lastBeatAt = 0;     // performance.now() of the last heartbeat

// ---------- Port list ----------

function portLabel(p) {
  const { usbVendorId, usbProductId } = p.getInfo();
  if (usbVendorId === undefined) return 'Serial port';
  const hex = (n) => n.toString(16).padStart(4, '0');
  const id = `${hex(usbVendorId)}:${hex(usbProductId)}`;
  const chip = KNOWN_CHIPS[id] ?? (usbVendorId === 0x303a ? 'Espressif USB' : 'USB serial');
  return `${chip} (${id})`;
}

async function refreshPorts(selectPort) {
  const previous = selectPort ?? knownPorts[els.portSelect.value];
  knownPorts = await navigator.serial.getPorts();

  els.portSelect.innerHTML = '';
  if (knownPorts.length === 0) {
    els.portSelect.add(new Option(t('noPorts'), ''));
  } else {
    knownPorts.forEach((p, i) => {
      const suffix = knownPorts.length > 1 ? ` #${i + 1}` : '';
      els.portSelect.add(new Option(portLabel(p) + suffix, String(i)));
    });
    const keep = knownPorts.indexOf(previous);
    els.portSelect.value = String(keep >= 0 ? keep : 0);
  }
  render();
}

async function addPort() {
  try {
    // Browser security: the first access to a port must go through Chrome's picker.
    // After that the port appears in getPorts() and in our own dropdown.
    const p = await navigator.serial.requestPort();
    await refreshPorts(p);
  } catch (err) {
    // NotFoundError = user closed the picker without choosing
    if (err.name !== 'NotFoundError') log(t('log.addFailed', { msg: err.message }));
  }
}

// ---------- Connection ----------

async function connect() {
  const selected = knownPorts[els.portSelect.value];
  if (!selected) return;
  try {
    await selected.open({ baudRate: BAUD_RATE });
  } catch (err) {
    log(t('log.connectFailed', { msg: err.message }));
    return;
  }
  stopDemo();
  if (imuDemoTimer) stopImuDemo();
  port = selected;
  log(t('log.opened', { port: portLabel(port) }));
  render();
  readLoop();
}

async function disconnect() {
  if (port?.ws) {   // Wi-Fi: closing the socket runs the same cleanup
    port.ws.close();
    return;
  }
  // Cancelling the reader ends readLoop, which then closes the port
  if (reader) await reader.cancel().catch(() => {});
}

function handleLines(lines) {
  for (const line of lines) {
    // One bad message must not stop the read loop
    try {
      handleLine(sanitize(line));
    } catch (err) {
      log(t('log.badLine', { msg: err.message }));
    }
  }
}

// Wi-Fi: the board's WebSocket on port 81 carries the same lines as USB
function connectWifi() {
  const host = els.wifiHost.value.trim();
  if (!host || port) return;
  try { localStorage.setItem(WIFI_HOST_STORAGE_KEY, host); } catch { /* storage unavailable */ }
  let ws;
  try {
    ws = new WebSocket(`ws://${host}:81/`);
  } catch {
    log(t('log.wsFailed', { host }));
    return;
  }
  let opened = false;
  let buffer = '';
  ws.onopen = () => {
    opened = true;
    stopDemo();
    if (imuDemoTimer) stopImuDemo();
    port = { ws, host };
    log(t('log.wsOpened', { host }));
    send('HELLO?');
    render();
  };
  ws.onmessage = (e) => {
    buffer += typeof e.data === 'string' ? e.data : '';
    const lines = buffer.split('\n');
    buffer = lines.pop();
    handleLines(lines);
  };
  ws.onerror = () => { if (!opened) log(t('log.wsFailed', { host })); };
  ws.onclose = () => {
    if (!opened) return;
    log(t('log.wsClosed'));
    closePort();
  };
}

async function readLoop() {
  const decoder = new TextDecoderStream();
  readableClosed = port.readable.pipeTo(decoder.writable).catch(() => {});
  reader = decoder.readable.getReader();

  let buffer = '';
  // The port usually opens mid-transmission, so the first line is likely a fragment
  let firstLine = true;
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split('\n');
      buffer = lines.pop();
      if (firstLine && lines.length > 0) {
        lines.shift();
        firstLine = false;
      }
      handleLines(lines);
    }
  } catch (err) {
    log(t('log.readError', { msg: err.message }));
  } finally {
    reader.releaseLock();
  }
  await closePort();
}

async function closePort() {
  const wasWifi = Boolean(port?.ws);
  if (!wasWifi) {
    await readableClosed;
    try {
      await port.close();
    } catch {
      // Port may already be gone (cable unplugged)
    }
  }
  port = null;
  reader = null;
  device = null;
  info = null;
  health = null;
  resetPowerData();
  resetTestData();
  resetEncData();
  spd = null;
  battery = null;
  wifi = null;
  imu = null;
  for (const m of MOTORS) {
    boardProfile[m] = null;
    profileSaved[m] = false;
  }
  spdChart.clear();
  mv = null;
  mvChart.clear();
  mvSpeedChart.clear();
  uptimeMs = null;
  lastBeatAt = 0;
  log(t('log.closed'));
  await refreshPorts();
}

// Writes are queued: only one writer may hold the port at a time, and PING runs on a timer
let sendQueue = Promise.resolve();

function send(text) {
  if (port?.ws) {   // Wi-Fi: the socket keeps order itself
    if (port.ws.readyState === WebSocket.OPEN) port.ws.send(text + '\n');
    return Promise.resolve();
  }
  sendQueue = sendQueue.then(async () => {
    if (!port?.writable) return;
    const writer = port.writable.getWriter();
    try {
      await writer.write(new TextEncoder().encode(text + '\n'));
    } finally {
      writer.releaseLock();
    }
  }).catch(() => {});
  return sendQueue;
}

// ---------- Protocol ----------

// The protocol is plain ASCII; drop anything else (line noise, boot-time garbage).
// Boot garbage can also be printable, so cut everything before a known message type.
function sanitize(line) {
  const ascii = line.replace(/[^\x20-\x7E]/g, '').trim();
  const start = ascii.search(/(?:HB|HELLO|INFO|HEALTH),/);
  return start > 0 ? ascii.slice(start) : ascii;
}

function handleLine(line) {
  if (!line) return;
  const [type, ...fields] = line.split(',');

  if (type === 'HB' && fields.length === 1) {
    lastBeatAt = performance.now();
    uptimeMs = Number(fields[0]);
    // Dashboard may connect after the boot-time HELLO/INFO were sent
    if (!device || !info) send('HELLO?');
    return;
  }

  if (type === 'HELLO' && fields.length === 2) {
    device = { name: fields[0], version: fields[1] };
    log(t('log.identified', device));
    for (const m of MOTORS) send(`PROFILE?,${m}`);   // the board keeps each motor's settings
    send('WIFI?');
    return;
  }

  if (type === 'INFO' && fields.length === 8) {
    const [chip, revision, cores, cpuMhz, flash, heap, mac, reset] = fields;
    info = {
      chip, revision, cores: Number(cores), cpuMhz: Number(cpuMhz),
      flash: Number(flash), heap: Number(heap), mac, resetReason: reset,
    };
    const [label, level] = resetReason(info.resetReason);
    if (level === 'bad') log(t('log.resetWarn', { reason: label }));
    return;
  }

  if (type === 'PROFILE' && fields.length === 10) {
    const [m, cpr, invert, kickPlus, kickMinus, kp, ki, tauMs, ka, maxRpm] = fields.map(Number);
    if (!(m in boardProfile)) return;
    boardProfile[m] = { ...boardProfile[m], cpr, invert: invert === 1, kickPlus, kickMinus, kp, ki, tauMs, ka, maxRpm };
    profileSaved[m] = true;
    // The encoder card and RPM conversions follow the board
    encSettings.cpr[m] = cpr;
    encSettings.invert[m] = invert === 1;
    saveEnc();
    if (Number(els.spdMotor.value) === m) fillGainFields(m);
    return;
  }

  if (type === 'FF' && fields.length === 3) {
    const m = Number(fields[0]);
    if (!boardProfile[m]) return;
    const points = fields[2].split(';').map((p) => p.split(':').map(Number));
    boardProfile[m][fields[1] === '+' ? 'ffPlus' : 'ffMinus'] = points;
    return;
  }

  if (type === 'SAVED' && fields.length === 2) {
    const m = Number(fields[0]);
    profileSaved[m] = fields[1] === '1';
    log(t(profileSaved[m] ? 'log.saved' : 'log.saveFailed', { m }));
    return;
  }

  if (type === 'IMU') {
    if (fields[0] === 'NA') {
      imu = { present: false, mode: Number(fields[1] ?? 0), calStored: fields[2] === '1', calLoaded: false };
      return;
    }
    if (fields.length !== 13) return;
    const [yaw, pitch, roll, ax, ay, az, cs, cg, ca, cm, mode, stored, loaded] = fields.map(Number);
    imu = { present: true, yaw, pitch, roll, ax, ay, az, cal: [cs, cg, ca, cm], mode, calStored: stored === 1, calLoaded: loaded === 1 };
    const v = { Yaw: imuYaw(), Pitch: pitch, Roll: roll, X: ax, Y: ay, Z: az };
    for (const [k, chart] of Object.entries(imuCharts)) chart.push('v', v[k]);
    return;
  }

  if (type === 'IMUCAL' && fields.length === 1) {
    if (imuCalAction) log(t(fields[0] === '1' ? (imuCalAction === 'save' ? 'log.imuCalSaved' : 'log.imuCalCleared') : 'log.imuCalFailed'));
    imuCalAction = null;
    return;
  }

  if (type === 'WIFI' && fields.length === 4) {
    const [state, ssid, ip, rssi] = fields;
    wifi = { state: Number(state), ssid, ip, rssi: Number(rssi) };
    return;
  }

  if (type === 'BATT' && fields.length === 2) {
    battery = { volts: Number(fields[0]), measured: fields[1] === '1' };
    return;
  }

  if (type === 'HEALTH' && fields.length === 7) {
    const [freeHeap, minFreeHeap, maxAlloc, loopAvgUs, loopMaxUs, loopHz, stackFree] =
      fields.map(Number);
    health = { freeHeap, minFreeHeap, maxAlloc, loopAvgUs, loopMaxUs, loopHz, stackFree };
    return;
  }

  if (type === 'POWER' && fields[0] in motorPower) {
    const motor = Number(fields[0]);
    const m = motorPower[motor];
    if (fields[1] === 'NA') {
      if (m.sensor !== 'missing') {
        currentChart.clear(motor);
        voltageChart.clear(motor);
      }
      Object.assign(m, newMotorPower(), { sensor: 'missing' });
      return;
    }
    if (fields.length === 6) {
      const [voltage, currentMa, powerMw, windowMinV, windowMaxMa] = fields.slice(1).map(Number);
      m.power = { voltage, currentMa, powerMw };
      m.sensor = 'ok';
      m.minVoltage = m.minVoltage === null ? windowMinV : Math.min(m.minVoltage, windowMinV);
      m.maxCurrentMa = m.maxCurrentMa === null ? windowMaxMa : Math.max(m.maxCurrentMa, windowMaxMa);
      currentChart.push(motor, currentMa);
      voltageChart.push(motor, voltage);
      return;
    }
  }

  if (type === 'SPD' && fields.length === 6) {
    const [mode, target, ref, speed, ff, pi] = fields.map(Number);
    spd = { mode, target, ref, speed, ff, pi };
    const active = testState === 1 && (mode === 1 || mode === 2);
    spdChart.push('speed', speed);
    spdChart.push('ref', active ? ref : 0);
    spdChart.push('target', active ? target : 0);
    return;
  }

  if (type === 'MOVE' && fields.length === 5) {
    const [phase, target, pos, profileRpm, finalError] = fields.map(Number);
    mv = { phase, target, pos, profileRpm, finalError };
    if (mvStartedAt !== null && (phase === 2 || phase === 3)) {
      mvTookMs = performance.now() - mvStartedAt;
      mvStartedAt = null;
    }
    if (phase !== 0 && mvMotor) {
      mvChart.push('pos', mvFromCounts(pos));
      mvChart.push('target', mvFromCounts(target));
      mvSpeedChart.push('speed', spd ? spd.speed : 0);
      mvSpeedChart.push('profile', Math.sign(target) * profileRpm);
    }
    return;
  }

  if (type === 'ENC' && fields.length === 4) {
    const [c1, s1, c2, s2] = fields.map(Number);
    for (const [m, count, cps] of [[1, c1, s1], [2, c2, s2]]) {
      encData[m] = { count, cps };
      encChart.push(m, encSpeedValue(m, cps));
      rpmChart.push(m, encRpm(m, cps));
      if (cal?.motor === m) {
        cal.encSamples.push(encRpm(m, cps));
        calChart.push('rpm', encRpm(m, cps));
      }
    }
    return;
  }

  if (type === 'RUNOUT' && fields.length === 4) {
    const [state, motor, duty, remainingMs] = fields.map(Number);
    testState = state;
    testOutput = { motor, duty, remainingMs };
    return;
  }

  // 200 Hz control log, only switched on by the calibration's step test
  if (type === 'L') {
    if (cal?.logSink && fields.length === 7) {
      const [tUs, duty, , , , , speed] = fields.map(Number);
      cal.logSink.push({ t: tUs / 1e6, duty, rpm: speed });
    }
    return;
  }

  if (type === 'DRIVE' && fields.length === 8) {
    const [active, t1, t2, s1, s2, p1, p2, corr] = fields.map(Number);
    drive = { active: active === 1, target: [t1, t2], speed: [s1, s2], pos: [p1, p2], corr };
    if (drive.active) {   // only live numbers: the first reports after a press can still be the last run's
      const f = driveFwd();
      driveRpmChart.push(1, s1 * f[0]);
      driveRpmChart.push(2, s2 * f[1]);
      driveDistChart.push('diff', driveDistMm());
    }
    return;
  }

  if (type === 'DUTYOUT' && fields.length === 2) {
    const [d1, d2] = fields.map(Number);
    motorDuty[1] = d1;
    motorDuty[2] = d2;
    dutyChart.push(1, d1);
    dutyChart.push(2, d2);
    return;
  }

  // Anything else (boot messages, debug prints) goes to the log as-is
  log(line);
}

// ---------- UI ----------

function currentState() {
  if (!port) return 'offline';
  // Wi-Fi can pause for a moment on a weak signal, so it gets more slack before 'waiting'
  const timeout = port.ws ? HEARTBEAT_TIMEOUT_MS * 2 : HEARTBEAT_TIMEOUT_MS;
  if (lastBeatAt && performance.now() - lastBeatAt < timeout) return 'online';
  return 'waiting';
}

function formatUptime(ms) {
  const total = Math.floor(ms / 1000);
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return [h, m, s].map((n) => String(n).padStart(2, '0')).join(':');
}

function render() {
  const state = currentState();
  els.statusCard.dataset.state = state;
  els.statusLabel.textContent = t(`state.${state}`);

  els.connectBtn.textContent = port ? t('disconnect') : t('connect');
  els.connectBtn.disabled = !port && knownPorts.length === 0;
  els.wifiConnectBtn.disabled = Boolean(port);
  els.wifiHost.disabled = Boolean(port);
  els.wifiSsid.placeholder = t('wifiSsid');
  els.wifiPass.placeholder = t('wifiPass');
  els.wifiSaveBtn.disabled = !port || !els.wifiSsid.value.trim();
  els.wifiForgetBtn.disabled = !port || !wifi || wifi.state === 0;
  els.wifiStatus.innerHTML = !port || !wifi ? t('wifiUnknown')
    : wifi.state === 0 ? t('wifiOff')
    : wifi.state === 1 ? t('wifiJoining', { ssid: escapeHtml(wifi.ssid) })
    : t('wifiOn', { ssid: `<b>${escapeHtml(wifi.ssid)}</b>`, ip: `<b>${escapeHtml(wifi.ip)}</b>`, rssi: wifi.rssi });
  els.portSelect.disabled = !!port || knownPorts.length === 0;
  els.addPortBtn.disabled = !!port;

  els.deviceName.textContent = device?.name ?? '—';
  els.firmwareVersion.textContent = device?.version ?? '—';
  els.uptime.textContent = uptimeMs !== null ? formatUptime(uptimeMs) : '—';
  els.lastBeat.textContent = lastBeatAt
    ? t('msAgo', { n: Math.round(performance.now() - lastBeatAt) })
    : '—';

  renderInfo();
  renderHealth(state);
  renderPower(state);
  renderTest(state);
}

function formatBytes(n) {
  if (n >= 1024 * 1024) return `${num(n / 1024 / 1024, 1)} MB`;
  if (n >= 1024) return `${num(n / 1024, 1)} KB`;
  return `${n} B`;
}

function renderInfo() {
  if (!info) {
    els.resetReason.textContent = '—';
    delete els.resetReason.dataset.level;
    els.chipModel.textContent = '—';
    els.chipCpu.textContent = '—';
    els.chipFlash.textContent = '—';
    els.chipMac.textContent = '—';
    return;
  }
  const [label, level] = resetReason(info.resetReason);
  els.resetReason.textContent = label;
  els.resetReason.dataset.level = level;
  els.chipModel.textContent = `${info.chip} rev ${info.revision}`;
  els.chipCpu.textContent = t('cores', { cores: info.cores, mhz: info.cpuMhz });
  els.chipFlash.textContent = formatBytes(info.flash);
  els.chipMac.textContent = info.mac;
}

// Usage bar: fills with the ratio; green up to 40 %, to yellow by 70 %, to red when full
function meterColor(ratio) {
  if (ratio <= 0.4) return 'var(--meter-good)';
  if (ratio <= 0.7) {
    const k = Math.round(((ratio - 0.4) / 0.3) * 100);
    return `color-mix(in oklab, var(--meter-mid) ${k}%, var(--meter-good))`;
  }
  const k = Math.round(((ratio - 0.7) / 0.3) * 100);
  return `color-mix(in oklab, var(--meter-bad) ${k}%, var(--meter-mid))`;
}

// Bar for an amount left: full and green, emptying through yellow to red
function setRemaining(bar, ratio) {
  setMeter(bar, ratio);
  if (ratio !== null) bar.style.background = meterColor(1 - ratio);
}

function setMeter(bar, ratio) {
  bar.style.width = ratio !== null ? `${(ratio * 100).toFixed(1)}%` : '0';
  bar.style.background = ratio !== null ? meterColor(ratio) : '';
}

function renderHealth(state) {
  els.healthCard.toggleAttribute('data-stale', state !== 'online');

  if (!health) {
    for (const el of [els.hFree, els.hLoopAvg, els.hStack]) el.textContent = '—';
    for (const el of [els.hFreeSub, els.hLoopMax, els.hStackSub]) el.textContent = '';
    for (const bar of [els.hFreeBar, els.hLoopBar, els.hStackBar]) setMeter(bar, null);
    renderBattery();
    return;
  }

  // Every tile shows what is left; the bar empties and turns yellow, then red, as it runs out
  const total = info?.heap;
  const freeRatio = total ? Math.min(1, Math.max(0, health.freeHeap / total)) : null;
  els.hFree.textContent = formatBytes(health.freeHeap);
  els.hFree.classList.toggle('warn', freeRatio !== null && freeRatio < HEAP_WARN_RATIO);
  setRemaining(els.hFreeBar, freeRatio);
  els.hFreeSub.textContent = total
    ? t('heapOf', { pct: Math.round(freeRatio * 100), total: formatBytes(total) })
    : t('totalUnknown');

  // Loop: time left in the 5 ms control tick after the slowest round
  const loopFreeUs = Math.max(0, LOOP_BUDGET_US - health.loopMaxUs);
  els.hLoopAvg.textContent = `${num(loopFreeUs / 1000, 2)} ms`;
  setRemaining(els.hLoopBar, loopFreeUs / LOOP_BUDGET_US);
  els.hLoopMax.textContent = t('loopBudget', { us: health.loopMaxUs, avg: num(Number(health.loopAvgUs), 1) });

  const stackRatio = Math.min(1, Math.max(0, health.stackFree / LOOP_STACK_BYTES));
  els.hStack.textContent = formatBytes(health.stackFree);
  els.hStack.classList.toggle('warn', health.stackFree < STACK_WARN_BYTES);
  els.hStackSub.textContent = t('heapOf', { pct: Math.round(stackRatio * 100), total: formatBytes(LOOP_STACK_BYTES) });
  setRemaining(els.hStackBar, stackRatio);
  renderBattery();
}

// Battery: the bar shows the charge left, so it goes red as the pack empties
function renderBattery() {
  const b = battery;
  if (!b) {
    els.hBatt.textContent = '—';
    els.hBattSub.textContent = '';
    els.hBatt.classList.remove('warn');
    setMeter(els.hBattBar, null);
    return;
  }
  els.hBatt.textContent = `${num(b.volts, 2)} V`;
  if (!b.measured) {   // nothing measured: a full grey bar, so it does not read as a charge level
    els.hBattSub.textContent = t('battAssumed');
    els.hBatt.classList.remove('warn');
    setMeter(els.hBattBar, 1);
    els.hBattBar.style.background = 'var(--offline)';
    return;
  }
  const charge = Math.min(1, Math.max(0, (b.volts - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V)));
  setRemaining(els.hBattBar, charge);
  els.hBatt.classList.toggle('warn', b.volts < BATT_LOW_V);
  els.hBattSub.textContent = t(b.volts < BATT_EMPTY_V ? 'battCritical' : b.volts < BATT_LOW_V ? 'battLow' : 'battComp');
}

function formatCurrent(ma) {
  if (Math.abs(ma) >= 1000) return `${num(ma / 1000, 2)} A`;
  return `${Math.round(ma)} mA`;
}

function formatPower(mw) {
  if (Math.abs(mw) >= 1000) return `${num(mw / 1000, 2)} W`;
  return `${Math.round(mw)} mW`;
}

// The same Power card appears on the Dashboard and the Power tab, one row per motor
function renderPower(state) {
  const demo = demoTimer !== null;
  const live = state === 'online' || demo;
  const badges = {
    ok: ['sensorOk', 'ok'],
    missing: ['sensorMissing', 'bad'],
    unknown: ['sensorWaiting', ''],
    demo: ['sensorDemo', 'demo'],
  };
  const anyData = MOTORS.some((m) => motorPower[m].power);

  for (const card of els.powerCards) {
    card.toggleAttribute('data-stale', !live);
    const cardField = (name) => card.querySelector(`[data-pfield="${name}"]`);
    cardField('reset').disabled = !anyData;
    cardField('demo').textContent = demo ? t('demoStop') : t('demoStart');
    cardField('demo').disabled = !!port;

    for (const row of card.querySelectorAll('[data-motor]')) {
      const m = motorPower[row.dataset.motor];
      const sensor = demo ? 'demo' : state === 'online' ? m.sensor : 'unknown';
      const field = (name) => row.querySelector(`[data-pfield="${name}"]`);
      const [badgeKey, level] = badges[sensor];
      field('badge').textContent = t(badgeKey);
      if (level) field('badge').dataset.level = level;
      else delete field('badge').dataset.level;
      field('hint').hidden = sensor !== 'missing';

      const values = m.power
        ? {
            voltage: `${num(m.power.voltage, 2)} V`,
            voltageSub: t('lowestV', { v: `${num(m.minVoltage, 2)} V` }),
            current: formatCurrent(m.power.currentMa),
            currentSub: t('peakI', { i: formatCurrent(m.maxCurrentMa) }),
            power: formatPower(m.power.powerMw),
          }
        : { voltage: '—', voltageSub: '', current: '—', currentSub: '', power: '—' };
      for (const [name, text] of Object.entries(values)) field(name).textContent = text;
    }
  }
}

function renderTest(state) {
  const demo = demoTimer !== null;
  const live = state === 'online' || demo;
  const supported = testState !== null;
  let badge;
  if (simTimer !== null) badge = ['testDemo', 'demo'];
  else if ((state === 'online' || simIdleTimer !== null) && supported) {
    badge = [TEST_STATES[testState] ?? 'testIdle', ['', 'run', 'bad', 'done'][testState] ?? ''];
  }
  else if (demo) badge = ['testIdle', ''];
  else badge = ['sensorWaiting', ''];

  els.testCard.toggleAttribute('data-stale', !live);
  els.testBadge.textContent = t(badge[0]);
  if (badge[1]) els.testBadge.dataset.level = badge[1];
  else delete els.testBadge.dataset.level;
  els.testStartBtn.disabled = !((state === 'online' && supported) || (demo && !port));
  els.testStopBtn.disabled = !(simTimer !== null || (port && testState === 1));
  els.testDuration.placeholder = t('durationHint');
  const running = (simTimer !== null || testState === 1) && testOutput;
  for (const m of MOTORS) {
    const enc = encData[m];
    const parts = [];
    if (running) parts.push(signedPercent(motorDuty[m]));
    if (live && enc.count !== null) parts.push(`${Math.round(encRpm(m, enc.cps))} RPM`);
    els[`testNow${m}`].textContent = parts.length ? parts.join(' · ') : '—';
  }
  renderSpd(state);
  renderMove(state);
  renderEnc(state);
}

// Counts <-> the chosen unit, with the motor's CPR and the wheel diameter
function mvCpr() {
  const m = mvMotor ?? Number(els.mvMotor.value);
  return encSettings.cpr[m] > 0 ? encSettings.cpr[m] : DEFAULT_CPR;
}
const mvWheelM = () => Math.max(0.01, (Number(els.mvWheel.value) || 65) / 1000);
const mvFromCounts = (counts) => {
  const turns = counts / mvCpr();
  return els.mvUnit.value === 'm' ? turns * Math.PI * mvWheelM() : turns;
};
const mvFormat = (counts) => (els.mvUnit.value === 'm'
  ? `${num(mvFromCounts(counts), 3)} m`
  : `${num(mvFromCounts(counts), 2)} ${t('mvTurnsShort')}`);
const mvDetail = (counts) => {
  const turns = counts / mvCpr();
  const mm = turns * Math.PI * mvWheelM() * 1000;
  return `${counts} · ${(turns * 360).toFixed(0)}° · ${mm.toFixed(0)} mm`;
};

// Same profile as the firmware: accelerate, cruise, decelerate (trapezoid, or a triangle
// when the distance is too short to reach max speed). Ignores the 40 RPM final approach.
function mvEstimate() {
  const amount = Math.abs(Number(els.mvAmount.value) || 0);
  const turns = els.mvUnit.value === 'm' ? amount / (Math.PI * mvWheelM()) : amount;
  const maxRpm = Math.max(40, Math.min(boardProfile[mvMotor ?? els.mvMotor.value]?.maxRpm ?? 1100, Number(els.mvMaxRpm.value) || 300));
  const accel = Math.max(100, Math.min(20000, Number(els.mvAccel.value) || 1500));
  const v = maxRpm / 60;    // rev/s
  const a = accel / 60;     // rev/s^2
  const rampTurns = (v * v) / (2 * a);
  if (turns === 0) return null;
  if (2 * rampTurns <= turns) {
    return { turns, maxRpm, peakRpm: maxRpm, reachS: v / a, totalS: 2 * v / a + (turns - 2 * rampTurns) / v };
  }
  const peak = Math.sqrt(a * turns);
  return { turns, maxRpm, peakRpm: peak * 60, reachS: peak / a, totalS: 2 * peak / a };
}

function renderMoveEstimate() {
  const e = mvEstimate();
  const parts = [];
  if (e) {
    const reached = e.peakRpm >= e.maxRpm - 0.5;
    const fmt = (key, vars) => t(key, vars).replace(/(\d[\d.,]*)/g, '<b>$1</b>');
    parts.push(reached
      ? fmt('mvEstReach', { rpm: Math.round(e.peakRpm), s: num(e.reachS, 2) })
      : fmt('mvEstPeak', { max: e.maxRpm, rpm: Math.round(e.peakRpm), s: num(e.reachS, 2) }));
    parts.push(fmt('mvEstTotal', { s: num(e.totalS, 2) }));
    parts.push(fmt('mvEstDistance', { turns: num(e.turns, 2) }));
  }
  if (mvTookMs !== null) parts.push(t('mvEstLast', { s: `<b>${num(mvTookMs / 1000, 2)}</b>` }));
  els.mvEstimate.innerHTML = parts.map((p) => `<span>${p}</span>`).join('');
}

function renderMove(state) {
  renderMoveEstimate();
  const supported = state === 'online' && mv !== null;
  const moving = supported && testState === 1 && spd?.mode === 3;
  els.mvBadge.textContent = !supported ? t('sensorWaiting')
    : moving ? (mv.phase === 2 ? t('mvBraking') : t('testRunning'))
    : mv.phase === 3 ? t('mvDone') : mv.phase === 4 ? t('mvAborted') : t('testIdle');
  if (moving) els.mvBadge.dataset.level = 'run';
  else delete els.mvBadge.dataset.level;
  els.mvStartBtn.disabled = !supported || moving;
  els.mvStopBtn.disabled = !moving;
  els.mvChartTitle.textContent = t(els.mvUnit.value === 'm' ? 'mvChartMeters' : 'mvChartTurns');
  const show = supported && mv.phase !== 0;
  els.mvTargetNow.textContent = show ? mvFormat(mv.target) : '—';
  els.mvTargetSub.textContent = show ? mvDetail(mv.target) : '';
  els.mvTraveled.textContent = show ? mvFormat(mv.pos) : '—';
  els.mvTraveledSub.textContent = show ? mvDetail(mv.pos) : '';
  els.mvRemaining.textContent = show ? mvFormat(mv.target - mv.pos) : '—';
  els.mvProfile.textContent = moving && mv.phase === 1 ? `${Math.round(mv.profileRpm)} RPM` : '';
  const finished = show && (mv.phase === 3 || mv.phase === 2);
  els.mvFinalError.textContent = finished ? mvFormat(mv.target - mv.pos) : '—';
  els.mvFinalSub.textContent = finished ? mvDetail(mv.target - mv.pos) : '';
}

function renderSpd(state) {
  renderImu(state);
  renderCal(state);
  renderDrive(state);
  renderProfile();
  const supported = state === 'online' && spd !== null;
  const running = supported && spdRunning();
  els.spdBadge.textContent = !supported ? t('sensorWaiting') : running ? t('testRunning') : t('testIdle');
  if (running) els.spdBadge.dataset.level = 'run';
  else delete els.spdBadge.dataset.level;
  els.spdStartBtn.disabled = !supported;
  els.spdStopBtn.disabled = !running;
  const show = running && spd;
  els.spdTargetNow.textContent = show ? `${Math.round(spd.target)} RPM` : '—';
  els.spdSpeedNow.textContent = supported ? `${Math.round(spd.speed)} RPM` : '—';
  els.spdErrorNow.textContent = show ? `${Math.round(spd.target - spd.speed)} RPM` : '—';
  els.spdDutyNow.textContent = show ? signedPercent(spd.ff + spd.pi) : '—';
  els.spdDutySplit.textContent = show ? `FF ${num(spd.ff, 1)} % · PI ${spd.pi >= 0 ? '+' : ''}${num(spd.pi, 1)} %` : '';
}

function renderEnc(state) {
  const live = (state === 'online' && encData[1].count !== null) || encSim !== null;
  els.encBadge.textContent = encSim ? t('testDemo') : live ? t('encLive') : t('sensorWaiting');
  if (encSim) els.encBadge.dataset.level = 'demo';
  else if (live) els.encBadge.dataset.level = 'ok';
  else delete els.encBadge.dataset.level;
  els.encChartTitle.textContent = t('encChart', { unit: encUseRpm() ? 'RPM' : t('cpsUnit') });

  for (const row of els.encCard.querySelectorAll('[data-emotor]')) {
    const m = Number(row.dataset.emotor);
    const field = (name) => row.querySelector(`[data-efield="${name}"]`);
    const d = encData[m];
    const cpr = encSettings.cpr[m] || 0;
    const has = live && d.count !== null;
    field('count').textContent = has ? d.count.toLocaleString() : '—';
    field('turns').textContent = has && cpr ? t('encTurns', { n: num(d.count / cpr, 2) }) : '';
    field('speed').textContent = !has ? '—'
      : cpr ? `${Math.round((d.cps * 60) / cpr)} RPM` : `${Math.round(d.cps)} ${t('cpsUnit')}`;
    field('invert').checked = Boolean(encSettings.invert[m]);
    field('zero').disabled = !live;
    field('oneTurn').disabled = !live || !d.count;
    if (document.activeElement !== field('cpr')) field('cpr').value = cpr || '';
  }
}

function log(message) {
  const time = new Date().toLocaleTimeString(I18N.lang === 'tr' ? 'tr-TR' : 'en-GB');
  const lines = (els.log.textContent + `[${time}] ${message}\n`).split('\n');
  els.log.textContent = lines.slice(-MAX_LOG_LINES - 1).join('\n');
  els.log.scrollTop = els.log.scrollHeight;
}

// ---------- Init ----------

// Wi-Fi works in any browser; USB needs Web Serial
try { els.wifiHost.value = localStorage.getItem(WIFI_HOST_STORAGE_KEY) || els.wifiHost.value; } catch { /* storage unavailable */ }
els.wifiConnectBtn.addEventListener('click', connectWifi);
els.wifiHost.addEventListener('keydown', (e) => { if (e.key === 'Enter') connectWifi(); });
els.wifiSaveBtn.addEventListener('click', () => {
  const ssid = els.wifiSsid.value.trim();
  if (!port || !ssid || ssid.includes(',')) return;
  send(`WIFISET,${ssid},${els.wifiPass.value}`);
  els.wifiPass.value = '';
});
els.wifiForgetBtn.addEventListener('click', () => { if (port) send('WIFIOFF'); });
els.wifiSsid.addEventListener('input', render);
els.connectBtn.addEventListener('click', () => (port ? disconnect() : connect()));
setInterval(render, 250);

if (!('serial' in navigator)) {
  els.unsupported.hidden = false;
  els.addPortBtn.disabled = true;
  els.portSelect.disabled = true;
} else {
  els.addPortBtn.addEventListener('click', addPort);
  // A previously allowed device was plugged in or unplugged
  navigator.serial.addEventListener('connect', () => refreshPorts());
  navigator.serial.addEventListener('disconnect', (e) => {
    if (e.target === port) log(t('log.unplugged'));
    if (!port) refreshPorts();
  });
  refreshPorts();
}

els.clearLogBtn.addEventListener('click', () => { els.log.textContent = ''; });
// ---------- Demo data ----------

// Fakes the firmware's POWER messages at the real 10 Hz rate. Each motor idles with a
// slow ripple, gets a load bump now and then (motor 2 out of phase) and stalls briefly
// once in a while; the shared supply sags with the total current.
function startDemo() {
  resetPowerData();
  startEncSim();
  let tick = 0;
  const noise = { 1: 0, 2: 0 };
  const profiles = {
    1: { idle: 290, period: 150, phase: 0, stallAt: 380 },
    2: { idle: 250, period: 190, phase: 70, stallAt: 250 },
  };
  demoTimer = setInterval(() => {
    tick++;
    const current = {};
    for (const m of MOTORS) {
      const p = profiles[m];
      noise[m] = noise[m] * 0.85 + (Math.random() - 0.5) * 6;   // smoothed, not jumpy
      const pos = (tick + p.phase) % p.period;
      const load = pos > 60 && pos < 110 ? 420 * Math.sin(((pos - 60) / 50) * Math.PI) : 0;
      const stallPos = tick % 400;
      const stall = stallPos > p.stallAt && stallPos < p.stallAt + 20
        ? 700 * Math.sin(((stallPos - p.stallAt) / 20) * Math.PI) : 0;
      current[m] = p.idle + 18 * Math.sin(tick / 12 + m) + noise[m] + load + stall;
    }
    const supply = 12.02 - (current[1] + current[2]) * 0.0004;
    for (const m of MOTORS) {
      const voltage = supply - current[m] * 0.00015;
      handleLine(`POWER,${m},${voltage.toFixed(3)},${current[m].toFixed(1)},` +
                 `${(voltage * current[m]).toFixed(1)},${(voltage - 0.01).toFixed(3)},${(current[m] + 8).toFixed(1)}`);
    }
    render();
  }, 100);
  render();
}

function stopDemo() {
  if (demoTimer === null) return;
  clearInterval(demoTimer);
  demoTimer = null;
  stopSimTest();
  stopEncSim();
  resetPowerData();
  resetTestData();
  resetEncData();
  render();
}

function resetPowerData() {
  for (const m of MOTORS) Object.assign(motorPower[m], newMotorPower());
  currentChart.clear();
  voltageChart.clear();
}

// ---------- Encoder ----------

// Changes go to the board right away and are saved to its flash a moment later, once the
// typing has stopped (the board itself waits until no motor runs before writing)
function scheduleSave(m) {
  if (!port) return;
  profileSaved[m] = false;
  clearTimeout(saveTimers[m]);
  saveTimers[m] = setTimeout(() => send(`SAVE,${m}`), 1000);
}

// ---------- Speed control ----------

function fillGainFields(m) {
  const p = boardProfile[m];
  if (!p) return;
  els.spdKp.value = p.kp;
  els.spdKi.value = p.ki;
  els.spdTauRef.value = p.tauMs;
  els.spdKa.value = p.ka;
  els.spdTarget.max = p.maxRpm;
  els.spdTarget.min = -p.maxRpm;
}

function sendGains() {
  const m = Number(els.spdMotor.value);
  const kp = Math.max(0, Number(els.spdKp.value) || 0);
  const ki = Math.max(0, Number(els.spdKi.value) || 0);
  const tauRef = Math.max(5, Number(els.spdTauRef.value) || 100);
  const ka = Math.max(0, Number(els.spdKa.value) || 0);
  if (!port) return;
  send(`GAIN,${kp},${ki},${m}`);
  send(`REF,${tauRef},${ka},${m}`);
  if (boardProfile[m]) Object.assign(boardProfile[m], { kp, ki, tauMs: tauRef, ka });
}

function renderProfile() {
  const m = Number(els.spdMotor.value);
  const p = boardProfile[m];
  els.spdDefaultsBtn.disabled = !port || !p;
  if (!p) {
    els.spdProfile.textContent = t('profileWaiting');
    return;
  }
  const b = (v) => `<b>${v}</b>`;
  const text = t('profileSummary', {
    kp: b(num(p.kickPlus, 1)), km: b(num(p.kickMinus, 1)), max: b(Math.round(p.maxRpm)),
    np: b(p.ffPlus?.length ?? 0), nm: b(p.ffMinus?.length ?? 0),
  });
  els.spdProfile.innerHTML = text + (profileSaved[m] ? ` · ${t('profileSaved')}` : '');
}

function startSpeed() {
  stopSimTest();
  sendGains();
  const limit = boardProfile[els.spdMotor.value]?.maxRpm ?? 1200;
  const target = Math.max(-limit, Math.min(limit, Number(els.spdTarget.value) || 0));
  if (port) send(`SPEED,${els.spdMotor.value},${target},${els.spdMode.value}`);
  render();
}

function renderSpdMotors() {
  const selected = els.spdMotor.value || '1';
  els.spdMotor.innerHTML = '';
  for (const [key, target] of Object.entries(TEST_TARGETS)) {
    if (target.pin) continue;
    els.spdMotor.add(new Option(`${t(target.label)} · ${target.in1} / ${target.in2}`, key));
  }
  els.spdMotor.value = selected;
}

els.spdStartBtn.addEventListener('click', startSpeed);
els.spdStopBtn.addEventListener('click', stopTest);
// Target and mode apply straight away while the speed loop runs; gains always
for (const el of [els.spdTarget, els.spdMode]) {
  el.addEventListener('change', () => { if (spdRunning()) startSpeed(); });
}
for (const el of [els.spdKp, els.spdKi, els.spdTauRef, els.spdKa]) {
  el.addEventListener('change', () => {
    sendGains();
    scheduleSave(Number(els.spdMotor.value));
    render();
  });
}
els.spdMotor.addEventListener('change', () => {
  fillGainFields(Number(els.spdMotor.value));
  render();
});
els.spdDefaultsBtn.addEventListener('click', () => {
  const m = Number(els.spdMotor.value);
  if (!port || !confirm(t('profileConfirm', { m }))) return;
  send(`DEFAULTS,${m}`);   // the board answers with the new profile
  scheduleSave(m);
});
renderSpdMotors();

// ---------- Distance test ----------

function saveMove() {
  const data = {
    unit: els.mvUnit.value, amount: els.mvAmount.value, wheel: els.mvWheel.value,
    maxRpm: els.mvMaxRpm.value, accel: els.mvAccel.value,
  };
  try { localStorage.setItem(MV_STORAGE_KEY, JSON.stringify(data)); } catch { /* storage unavailable */ }
}

function loadMove() {
  let data = null;
  try { data = JSON.parse(localStorage.getItem(MV_STORAGE_KEY)); } catch { /* storage unavailable */ }
  if (!data) return;
  if (data.unit) els.mvUnit.value = data.unit;
  for (const [key, el] of [['amount', els.mvAmount], ['wheel', els.mvWheel], ['maxRpm', els.mvMaxRpm], ['accel', els.mvAccel]]) {
    if (data[key] !== undefined && data[key] !== '') el.value = data[key];
  }
}

function startMove() {
  stopSimTest();
  const motor = Number(els.mvMotor.value);
  mvMotor = motor;
  const amount = Number(els.mvAmount.value) || 0;
  const turns = els.mvUnit.value === 'm' ? amount / (Math.PI * mvWheelM()) : amount;
  const counts = Math.round(turns * mvCpr());
  const maxRpm = Math.max(40, Math.min(boardProfile[mvMotor ?? els.mvMotor.value]?.maxRpm ?? 1100, Number(els.mvMaxRpm.value) || 300));
  const accel = Math.max(100, Math.min(20000, Number(els.mvAccel.value) || 1500));
  if (counts === 0) return;
  sendGains();
  mvChart.clear();
  mvSpeedChart.clear();
  mvStartedAt = performance.now();
  mvTookMs = null;
  if (port) send(`MOVE,${motor},${counts},${maxRpm},${accel}`);
  render();
}

function renderMoveMotors() {
  const selected = els.mvMotor.value || '1';
  els.mvMotor.innerHTML = '';
  for (const [key, target] of Object.entries(TEST_TARGETS)) {
    if (target.pin) continue;
    els.mvMotor.add(new Option(`${t(target.label)} · ${target.in1} / ${target.in2}`, key));
  }
  els.mvMotor.value = selected;
}

els.mvStartBtn.addEventListener('click', startMove);
els.mvStopBtn.addEventListener('click', stopTest);
for (const el of [els.mvUnit, els.mvAmount, els.mvWheel, els.mvMaxRpm, els.mvAccel]) {
  el.addEventListener('change', () => {
    if (el === els.mvUnit) {
      els.mvAmount.step = el.value === 'm' ? '0.1' : '0.5';
      mvChart.clear();
    }
    saveMove();
    render();
  });
}
loadMove();
els.mvAmount.step = els.mvUnit.value === 'm' ? '0.1' : '0.5';
renderMoveMotors();

function zeroEncoder(m) {
  if (port) send(`ENCZERO,${m}`);
  else if (encSim) encSim.count[m] = 0;
}

// Demo only: a first-order motor (dead band near the measured start threshold) answering
// the simulated run, reported as the firmware's ENC line
function startEncSim() {
  stopEncSim();
  const CPR = DEFAULT_CPR;   // same as the page uses for RPM, so the demo tops out near 1350 RPM
  const sim = { count: { 1: 0, 2: 0 }, speed: { 1: 0, 2: 0 }, last: performance.now() };
  sim.timer = setInterval(() => {
    const now = performance.now();
    const dt = (now - sim.last) / 1000;
    sim.last = now;
    const parts = [];
    for (const m of MOTORS) {
      const duty = testState === 1 ? motorDuty[m] : 0;
      const drive = Math.max(0, Math.abs(duty) - 11) / 89;
      const target = Math.sign(duty) * drive * (1350 / 60) * CPR * (m === 2 ? 0.95 : 1);
      sim.speed[m] += (target - sim.speed[m]) * Math.min(1, dt / 0.12);
      sim.count[m] += sim.speed[m] * dt;
      const sign = encSettings.invert[m] ? -1 : 1;
      const noise = Math.abs(sim.speed[m]) > 1 ? (Math.random() - 0.5) * 30 : 0;
      parts.push(Math.round(sign * sim.count[m]), Math.round(sign * (sim.speed[m] + noise)));
    }
    handleLine(`ENC,${parts.join(',')}`);
  }, 40);
  encSim = sim;
}

function stopEncSim() {
  if (!encSim) return;
  clearInterval(encSim.timer);
  encSim = null;
}

function resetEncData() {
  for (const m of MOTORS) {
    encData[m] = newEnc();
  }
  encChart.clear();
}

for (const row of els.encCard.querySelectorAll('[data-emotor]')) {
  const m = Number(row.dataset.emotor);
  const field = (name) => row.querySelector(`[data-efield="${name}"]`);
  field('zero').addEventListener('click', () => zeroEncoder(m));
  // Turning several revolutions averages out the fractional counts of a non-integer gear ratio
  field('oneTurn').addEventListener('click', () => {
    const count = Math.abs(encData[m].count ?? 0);
    const turns = Math.max(1, Math.round(Number(field('turnsInput').value) || 1));
    if (!count) return;
    encSettings.cpr[m] = Math.round((count / turns) * 100) / 100;
    saveEnc();
    if (port) send(`CPR,${m},${encSettings.cpr[m]}`);
    scheduleSave(m);
    encChart.clear();
    render();
  });
  field('cpr').addEventListener('change', () => {
    const cpr = Math.round(Number(field('cpr').value) * 100) / 100;
    if (cpr > 0) encSettings.cpr[m] = cpr;
    else delete encSettings.cpr[m];
    saveEnc();
    if (port && cpr > 0) {
      send(`CPR,${m},${cpr}`);
      scheduleSave(m);
    }
    encChart.clear();
    render();
  });
  field('invert').addEventListener('change', () => {
    encSettings.invert[m] = field('invert').checked;
    saveEnc();
    if (port) send(`ENCINV,${m},${encSettings.invert[m] ? 1 : 0}`);
    scheduleSave(m);
    render();
  });
}

// ---------- Motor control ----------

function readTestParams() {
  const duty = (el) => Math.max(-100, Math.min(100, Number(el.value) || 0));
  return {
    duties: [duty(els.testDuty1), duty(els.testDuty2)],
    durationMs: Math.max(0, Math.round((Number(els.testDuration.value) || 0) * 1000)),
  };
}

// The charts are never cleared here, so starts, stops and changes stay on them as history
function startTest() {
  const p = readTestParams();
  if (port) send(`DUTY,${p.duties[0]},${p.duties[1]},${p.durationMs}`);
  else if (demoTimer !== null) startSimTest(p);
  render();
}

function stopTest() {
  if (port) send('STOP');
  else if (simTimer !== null) startSimIdle(0);
  render();
}

// Demo only: reports the same RUNOUT / DUTYOUT lines the firmware would send, including
// the idle ones between runs, so the charts keep flowing like with a real board
let simIdleTimer = null;

function startSimTest(p) {
  stopSimTest();
  const start = performance.now();
  simTimer = setInterval(() => {
    const elapsed = performance.now() - start;
    if (p.durationMs > 0 && elapsed >= p.durationMs) {
      startSimIdle(3);
      render();
      return;
    }
    const remaining = p.durationMs > 0 ? Math.round(p.durationMs - elapsed) : 0;
    handleLine(`RUNOUT,1,4,${p.duties[0].toFixed(1)},${remaining}`);
    handleLine(`DUTYOUT,${p.duties[0].toFixed(1)},${p.duties[1].toFixed(1)}`);
  }, 40);
}

function startSimIdle(state) {
  stopSimTest();
  simIdleTimer = setInterval(() => {
    handleLine(`RUNOUT,${state},4,0.0,0`);
    handleLine('DUTYOUT,0.0,0.0');
  }, 40);
}

function stopSimTest() {
  clearInterval(simTimer);
  clearInterval(simIdleTimer);
  simTimer = null;
  simIdleTimer = null;
}

function resetTestData() {
  testState = null;
  testOutput = null;
  motorDuty[1] = 0;
  motorDuty[2] = 0;
  dutyChart.clear();
  rpmChart.clear();
}

els.testStartBtn.addEventListener('click', () => startTest());
els.testStopBtn.addEventListener('click', stopTest);
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') stopTest(); });
// A new duty takes effect right away while running (Enter or leaving the field)
const dutyRunning = () => simTimer !== null || (port && testState === 1 && testOutput?.motor === 4);
for (const el of [els.testDuty1, els.testDuty2]) {
  el.addEventListener('change', () => { if (dutyRunning()) startTest(); });
}

// ---------- IMU ----------
// BNO055 angles and acceleration (IMU report, 20 Hz). Heading can be zeroed here: the
// offset lives in this page only, the sensor keeps its own absolute heading.

let imu = null;        // { present, yaw, pitch, roll, ax, ay, az, cal: [sys, gyro, accel, mag] }
let imuYawZero = 0;
let imuCalAction = null;   // 'save' | 'clear' waiting for the board's IMUCAL answer
const imuYaw = () => (imu?.present ? (((imu.yaw - imuYawZero) % 360) + 360) % 360 : 0);
const imuChartDefs = {
  Yaw: { range: [0, 360], step: 90, unit: '°', digits: 1 },
  Pitch: { range: [-180, 180], step: 90, unit: '°', digits: 1 },
  Roll: { range: [-180, 180], step: 90, unit: '°', digits: 1 },
  X: { range: [-20, 20], step: 10, unit: 'm/s²', digits: 2 },
  Y: { range: [-20, 20], step: 10, unit: 'm/s²', digits: 2 },
  Z: { range: [-20, 20], step: 10, unit: 'm/s²', digits: 2 },
};
const imuCharts = Object.fromEntries(Object.entries(imuChartDefs).map(([k, d]) => [k, new LiveChart(
  document.getElementById(`imu${k}Chart`), {
    series: [{ key: 'v', label: () => k, short: '', color: '--series-3' }],
    format: (v) => `${num(v, d.digits)} ${d.unit}`,
    tickFormat: (v) => `${Math.round(v)}`,
    range: d.range,
    step: d.step,
    windowMs: 30000,
    labels: chartLabels,
  })]));
const imuEls = {
  badge: document.getElementById('imuBadge'),
  hint: document.getElementById('imuHint'),
  cal: document.getElementById('imuCal'),
  zero: document.getElementById('imuZeroBtn'),
  demo: document.getElementById('imuDemoBtn'),
  mode: document.getElementById('imuMode'),
  calState: document.getElementById('imuCalState'),
  calWrap: document.getElementById('imuCalWrap'),
  calSave: document.getElementById('imuCalSaveBtn'),
  calClear: document.getElementById('imuCalClearBtn'),
  values: Object.fromEntries(Object.keys(imuChartDefs).map((k) => [k, document.getElementById(`imu${k}`)])),
};

function renderImu(state) {
  const demo = imuDemoTimer !== null;
  const online = state === 'online' || demo;
  const live = online && imu?.present;
  imuEls.badge.textContent = t(demo ? 'sensorDemo' : !online || !imu ? 'sensorWaiting' : imu.present ? 'imuLive' : 'imuMissing');
  if (demo) imuEls.badge.dataset.level = 'demo';
  else if (live) imuEls.badge.dataset.level = 'ok';
  else delete imuEls.badge.dataset.level;
  imuEls.demo.textContent = t(demo ? 'demoStop' : 'demoStart');
  imuEls.demo.disabled = Boolean(port);
  imuEls.hint.hidden = !(online && imu && !imu.present);
  imuEls.zero.disabled = !live;
  const v = live ? { Yaw: imuYaw(), Pitch: imu.pitch, Roll: imu.roll, X: imu.ax, Y: imu.ay, Z: imu.az } : null;
  for (const [k, el] of Object.entries(imuEls.values)) {
    el.textContent = v ? `${num(v[k], imuChartDefs[k].digits)} ${imuChartDefs[k].unit}` : '—';
  }
  // The board's mode wins once it reports; until then the page shows its own choice
  if (imu && document.activeElement !== imuEls.mode) imuEls.mode.value = String(imu.mode ?? 0);
  imuEls.mode.disabled = !port && !demo;
  const ndof = imuEls.mode.value === '1';
  const names = ['imuCalSys', 'imuCalGyro', 'imuCalAccel', ...(ndof ? ['imuCalMag'] : [])];   // no compass in IMU mode
  // Saved calibration: what the board says, and whether the mode's sensors allow a save now
  const stored = Boolean(imu?.calStored);
  imuEls.calState.textContent = t(!online || !imu ? 'imuCalNone' : !stored ? 'imuCalNone' : imu.calLoaded ? 'imuCalStoredLoaded' : 'imuCalStoredOnly');
  if (online && stored) imuEls.calWrap.dataset.level = 'ok';
  else delete imuEls.calWrap.dataset.level;
  const needed = ndof ? [0, 1, 2, 3] : [1, 2];   // NDOF: everything; IMU: gyro and accel
  const ready = live && needed.every((i) => imu.cal[i] === 3);
  imuEls.calSave.disabled = !port || !ready;
  imuEls.calClear.disabled = !port || !stored;
  imuEls.cal.innerHTML = names.map((n, i) => {
    const level = live ? imu.cal[i] : 0;
    const dots = [1, 2, 3].map((d) => `<b class="${d <= level ? 'on' : ''}"></b>`).join('');
    return `<span>${t(n)} <i aria-label="${level}/3">${dots}</i></span>`;
  }).join('') + `<span>${t('imuCalHint')}</span>`;
}

// Demo only: a robot driving around a room, sent through the same IMU lines as the board.
// It turns slowly with the odd quick turn, rocks a little and starts uncalibrated.
let imuDemoTimer = null;
let imuDemoMode = 0;
function startImuDemo() {
  const t0 = performance.now();
  let yaw = 90;
  imuDemoTimer = setInterval(() => {
    const t = (performance.now() - t0) / 1000;
    const turnRate = 12 * Math.sin(t / 7) + (Math.sin(t / 2.3) > 0.95 ? 60 : 0);   // °/s
    yaw = (yaw + turnRate * 0.05 + 360) % 360;
    const pitch = 2.5 * Math.sin(t * 0.9) + 0.4 * (Math.random() - 0.5);
    const roll = 1.8 * Math.sin(t * 0.6 + 1) + 0.4 * (Math.random() - 0.5);
    const ax = 0.8 * Math.sin(t * 1.4) + 0.15 * (Math.random() - 0.5);
    const ay = (turnRate / 60) * 1.2 + 0.15 * (Math.random() - 0.5);   // pushed sideways in turns
    const az = 9.81 + 0.12 * (Math.random() - 0.5);
    const cal = [Math.min(3, Math.floor(t / 6)), Math.min(3, Math.floor(t / 2)), Math.min(3, Math.floor(t / 4)), Math.min(3, Math.floor(t / 5))];
    handleLine(`IMU,${yaw.toFixed(2)},${pitch.toFixed(2)},${roll.toFixed(2)},${ax.toFixed(2)},${ay.toFixed(2)},${az.toFixed(2)},${cal.join(',')},${imuDemoMode},0,0`);
  }, 50);
  render();
}
function stopImuDemo() {
  clearInterval(imuDemoTimer);
  imuDemoTimer = null;
  imu = null;
  for (const chart of Object.values(imuCharts)) chart.clear();
  render();
}
imuEls.demo.addEventListener('click', () => (imuDemoTimer ? stopImuDemo() : startImuDemo()));
imuEls.calSave.addEventListener('click', () => {
  if (!port) return;
  imuCalAction = 'save';
  send('IMUCALSAVE');
});
imuEls.calClear.addEventListener('click', () => {
  if (!port) return;
  imuCalAction = 'clear';
  send('IMUCALCLEAR');
});

imuEls.mode.addEventListener('change', () => {
  const mode = Number(imuEls.mode.value);
  if (port) send(`IMUMODE,${mode}`);
  imuDemoMode = mode;
  imuYawZero = 0;
  imuCharts.Yaw.clear();
  render();
});

// Heading dial and artificial horizon, drawn on canvas each frame while the IMU tab is open.
// The shown angles ease towards the latest reading (heading the short way round).
const imuPanel = document.querySelector('[data-panel="imu"]');
const compassCanvas = document.getElementById('imuCompass');
const horizonCanvas = document.getElementById('imuHorizon');
const compassCap = document.getElementById('imuCompassCap');
const horizonCap = document.getElementById('imuHorizonCap');
const shown = { yaw: 0, pitch: 0, roll: 0 };

function prepCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const size = canvas.clientWidth || 220;
  if (canvas.width !== size * dpr) {
    canvas.width = size * dpr;
    canvas.height = size * dpr;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, size, size);
  return { ctx, size, r: size / 2 - 6, css: getComputedStyle(document.documentElement) };
}

function drawCompass(live) {
  const { ctx, size, r, css } = prepCanvas(compassCanvas);
  const tk = (n) => css.getPropertyValue(n).trim();
  const c = size / 2;
  ctx.save();
  ctx.translate(c, c);
  // face
  ctx.beginPath();
  ctx.arc(0, 0, r, 0, Math.PI * 2);
  ctx.fillStyle = tk('--surface');   // stands out from the instrument panel behind it
  ctx.fill();
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = tk('--border');
  ctx.stroke();
  // ticks every 10°, longer every 30°
  for (let d = 0; d < 360; d += 10) {
    const a = (d - 90) * Math.PI / 180;
    const long = d % 30 === 0;
    ctx.beginPath();
    ctx.moveTo(Math.cos(a) * (r - (long ? 12 : 6)), Math.sin(a) * (r - (long ? 12 : 6)));
    ctx.lineTo(Math.cos(a) * (r - 2), Math.sin(a) * (r - 2));
    ctx.strokeStyle = tk(long ? '--muted' : '--border');
    ctx.lineWidth = long ? 1.5 : 1;
    ctx.stroke();
  }
  // labels: N E S W with the compass, degrees without
  const ndof = imuEls.mode.value === '1';
  const labels = ndof ? t('imuCardinal').split(',') : ['0', '90', '180', '270'];
  ctx.font = '600 12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  labels.forEach((text, i) => {
    const a = (i * 90 - 90) * Math.PI / 180;
    ctx.fillStyle = ndof && i === 0 ? tk('--danger') : tk('--text');
    ctx.fillText(text, Math.cos(a) * (r - 24), Math.sin(a) * (r - 24));
  });
  // robot seen from above, nose pointing along the heading
  ctx.rotate(shown.yaw * Math.PI / 180);
  ctx.globalAlpha = live ? 1 : 0.35;
  const w = r * 0.34, h = r * 0.5;
  ctx.beginPath();
  ctx.roundRect(-w / 2, -h / 2, w, h, 6);
  ctx.fillStyle = tk('--surface');
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = tk('--text');
  ctx.stroke();
  ctx.fillStyle = tk('--text');   // wheels
  ctx.fillRect(-w / 2 - 6, -h / 4, 5, h / 2);
  ctx.fillRect(w / 2 + 1, -h / 4, 5, h / 2);
  ctx.beginPath();                // nose arrow
  ctx.moveTo(0, -h / 2 - r * 0.28);
  ctx.lineTo(-9, -h / 2 - 4);
  ctx.lineTo(9, -h / 2 - 4);
  ctx.closePath();
  ctx.fillStyle = tk('--accent');
  ctx.fill();
  ctx.restore();
  compassCap.innerHTML = `<span>${t('imuHeading')}</span><b>${live ? `${num(imuYaw(), 0)}°` : '—'}</b>`;
}

function drawHorizon(live) {
  const { ctx, size, r, css } = prepCanvas(horizonCanvas);
  const tk = (n) => css.getPropertyValue(n).trim();
  const c = size / 2;
  const pxPerDeg = r / 45;   // ±45° of pitch fills the dial
  ctx.save();
  ctx.translate(c, c);
  ctx.beginPath();
  ctx.arc(0, 0, r, 0, Math.PI * 2);
  ctx.clip();
  // sky and ground, turned by roll and shifted by pitch
  ctx.save();
  ctx.rotate(-shown.roll * Math.PI / 180);
  ctx.translate(0, shown.pitch * pxPerDeg);
  ctx.globalAlpha = live ? 1 : 0.35;
  ctx.fillStyle = tk('--sky');
  ctx.fillRect(-size, -size * 2, size * 2, size * 2);
  ctx.fillStyle = tk('--ground');
  ctx.fillRect(-size, 0, size * 2, size * 2);
  ctx.globalAlpha = 1;
  ctx.strokeStyle = 'rgba(255,255,255,0.9)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-size, 0);
  ctx.lineTo(size, 0);
  ctx.stroke();
  // pitch ladder every 10°
  ctx.lineWidth = 1.2;
  ctx.font = '10px system-ui, sans-serif';
  ctx.fillStyle = 'rgba(255,255,255,0.9)';
  ctx.textBaseline = 'middle';
  for (let p = -30; p <= 30; p += 10) {
    if (p === 0) continue;
    const y = -p * pxPerDeg;
    const half = p % 20 === 0 ? r * 0.28 : r * 0.16;
    ctx.beginPath();
    ctx.moveTo(-half, y);
    ctx.lineTo(half, y);
    ctx.stroke();
    if (p % 20 === 0) {
      ctx.textAlign = 'left';
      ctx.fillText(String(Math.abs(p)), half + 4, y);
    }
  }
  ctx.restore();
  // fixed robot marker
  ctx.strokeStyle = tk('--accent');
  ctx.lineWidth = 3;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(-r * 0.42, 0);
  ctx.lineTo(-r * 0.14, 0);
  ctx.lineTo(-r * 0.07, r * 0.07);
  ctx.moveTo(r * 0.42, 0);
  ctx.lineTo(r * 0.14, 0);
  ctx.lineTo(r * 0.07, r * 0.07);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(0, 0, 3, 0, Math.PI * 2);
  ctx.fillStyle = tk('--accent');
  ctx.fill();
  ctx.restore();
  // rim
  ctx.beginPath();
  ctx.arc(c, c, r, 0, Math.PI * 2);
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = tk('--border');
  ctx.stroke();
  horizonCap.innerHTML = `<span>${t('imuAttitude')}</span><b>${live ? `${num(imu.pitch, 1)}° / ${num(imu.roll, 1)}°` : '—'}</b>`;
}

function animateDials() {
  requestAnimationFrame(animateDials);
  if (imuPanel.hidden) return;
  const live = Boolean(imu?.present) && (currentState() === 'online' || imuDemoTimer !== null);
  if (live) {
    const ease = 0.25;
    const dYaw = ((imuYaw() - shown.yaw + 540) % 360) - 180;   // short way round
    shown.yaw += dYaw * ease;
    shown.pitch += (imu.pitch - shown.pitch) * ease;
    shown.roll += (imu.roll - shown.roll) * ease;
  }
  drawCompass(live);
  drawHorizon(live);
}
requestAnimationFrame(animateDials);

imuEls.zero.addEventListener('click', () => {
  if (!imu?.present) return;
  imuYawZero = imu.yaw;
  imuCharts.Yaw.clear();
  render();
});

// ---------- Drive ----------
// Both speed loops at once (DRIVE). Buttons work in the robot's frame: each motor's forward
// sign and which one is the left wheel are set once on the card and kept in this browser.

const DRIVE_STORAGE_KEY = 'esp32-dashboard-drive';
let drive = null;      // latest DRIVE report: { active, target, speed, pos, corr }
let driveCmd = null;   // button held: 'fwd' | 'back' | 'left' | 'right'
let driveSentAt = 0;   // performance.now() of the last DRIVE sent
const driveEls = {
  badge: document.getElementById('driveBadge'),
  speed: document.getElementById('driveSpeed'),
  sync: document.getElementById('driveSyncGain'),
  fwd1: document.getElementById('driveFwd1'),
  fwd2: document.getElementById('driveFwd2'),
  left: document.getElementById('driveLeft'),
  stop: document.getElementById('driveStopBtn'),
  pad: document.querySelectorAll('[data-drive]'),
  m1: document.getElementById('driveM1'),
  m1Sub: document.getElementById('driveM1Sub'),
  m2: document.getElementById('driveM2'),
  m2Sub: document.getElementById('driveM2Sub'),
  diff: document.getElementById('driveDiff'),
  corr: document.getElementById('driveCorr'),
  dist: document.getElementById('driveDist'),
  distSub: document.getElementById('driveDistSub'),
};
const driveRpmChart = new LiveChart(document.getElementById('driveRpmChart'), {
  series: motorSeries,
  legend: document.getElementById('driveRpmLegend'),
  format: (v) => `${Math.round(v)} RPM`,
  minSpan: 50,
  windowMs: 20000,
  labels: chartLabels,
});
const driveDistChart = new LiveChart(document.getElementById('driveDistChart'), {
  series: [{ key: 'diff', label: () => t('driveDistDiff'), short: '', color: '--series-3' }],
  format: (v) => `${num(v, 1)} mm`,
  minSpan: 4,
  windowMs: 20000,
  labels: chartLabels,
});

const driveFwd = () => [Number(driveEls.fwd1.value), Number(driveEls.fwd2.value)];
// The difference the sync works on: how much further motor 1's wheel has gone in the way
// it is being driven. Straight, that is the forward distance difference; turning on the
// spot, the difference in how far each wheel has turned (both should match either way).
let driveSigns = [1, 1];   // each motor's direction in the last drive, kept after it stops
function driveDistMm() {
  if (!drive) return 0;
  for (const i of [0, 1]) if (drive.target[i] !== 0) driveSigns[i] = Math.sign(drive.target[i]);
  const revs = drive.pos[0] * driveSigns[0] - drive.pos[1] * driveSigns[1];
  return revs * Math.PI * mvWheelM() * 1000;
}

function saveDrive() {
  const data = { speed: driveEls.speed.value, sync: driveEls.sync.value, fwd1: driveEls.fwd1.value, fwd2: driveEls.fwd2.value, left: driveEls.left.value };
  try { localStorage.setItem(DRIVE_STORAGE_KEY, JSON.stringify(data)); } catch { /* storage unavailable */ }
}
function loadDrive() {
  let data = null;
  try { data = JSON.parse(localStorage.getItem(DRIVE_STORAGE_KEY)); } catch { /* storage unavailable */ }
  // Default: motors face each other, so forward is + on one and − on the other
  const d = { speed: '150', sync: '60', fwd1: '1', fwd2: '-1', left: '1', ...(data ?? {}) };
  driveEls.speed.value = d.speed;
  driveEls.sync.value = d.sync;
  driveEls.fwd1.value = d.fwd1;
  driveEls.fwd2.value = d.fwd2;
  driveEls.left.value = d.left;
}

// Wheel speeds in the robot's frame, then into each motor's own sign
function sendDrive(cmd) {
  const v = Math.max(0, Number(driveEls.speed.value) || 0);
  const leftIs1 = driveEls.left.value === '1';
  const robot = {   // [left, right] wheel speed, forward = +
    fwd: [v, v], back: [-v, -v], left: [-v, v], right: [v, -v],
  }[cmd];
  const [w1, w2] = leftIs1 ? robot : [robot[1], robot[0]];
  const f = driveFwd();
  const sync = Math.max(0, Number(driveEls.sync.value) || 0);
  if (port) send(`DRIVE,${w1 * f[0]},${w2 * f[1]},${sync}`);
}

function startDrive(cmd) {
  stopSimTest();
  if (driveCmd === null) {
    driveRpmChart.clear();
    driveDistChart.clear();
  }
  driveCmd = cmd;
  driveSentAt = performance.now();
  sendDrive(cmd);
  render();
}

function stopDrive() {
  if (driveCmd !== null && port) send('STOP');
  driveCmd = null;
  render();
}

function renderDrive(state) {
  const online = state === 'online';
  const active = online && drive?.active;
  // Stopped elsewhere or by the watchdog (after giving the board a moment to report)
  if (!active && driveCmd !== null && performance.now() - driveSentAt > 600) driveCmd = null;
  driveEls.badge.textContent = t(active ? 'testRunning' : online ? 'testIdle' : 'sensorWaiting');
  if (active) driveEls.badge.dataset.level = 'run';
  else delete driveEls.badge.dataset.level;
  for (const b of driveEls.pad) {
    b.disabled = !online;
    b.setAttribute('aria-pressed', String(active && driveCmd === b.dataset.drive));
  }
  driveEls.stop.disabled = !active;
  const f = driveFwd();
  for (const [i, el, sub] of [[0, driveEls.m1, driveEls.m1Sub], [1, driveEls.m2, driveEls.m2Sub]]) {
    el.textContent = drive && online ? `${Math.round(drive.speed[i] * f[i])} RPM` : '—';
    sub.textContent = active ? t('driveTarget', { rpm: Math.round(drive.target[i] * f[i]) }) : '';
  }
  const showNums = drive && online;
  driveEls.diff.textContent = showNums ? `${Math.round(drive.speed[0] * f[0] - drive.speed[1] * f[1])} RPM` : '—';
  driveEls.corr.textContent = active
    ? (Number(driveEls.sync.value) > 0 ? t('driveCorrection', { rpm: num(drive.corr, 1) }) : t('driveSyncOff'))
    : '';
  driveEls.dist.textContent = showNums ? `${num(driveDistMm(), 1)} mm` : '—';
  driveEls.distSub.textContent = showNums
    ? `${t('driveTurns', { n: num(drive.pos[0] * f[0], 2) })} · ${t('driveTurns', { n: num(drive.pos[1] * f[1], 2) })}`
    : '';
}

for (const b of driveEls.pad) b.addEventListener('click', () => startDrive(b.dataset.drive));
driveEls.stop.addEventListener('click', stopDrive);
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') stopDrive(); });
// Changing the speed or sync while driving applies right away
for (const el of [driveEls.speed, driveEls.sync]) {
  el.addEventListener('change', () => {
    saveDrive();
    if (driveCmd) sendDrive(driveCmd);
  });
}
for (const el of [driveEls.fwd1, driveEls.fwd2, driveEls.left]) {
  el.addEventListener('change', () => {
    saveDrive();
    if (driveCmd) sendDrive(driveCmd);
    render();
  });
}
loadDrive();

// ---------- Calibration ----------
// Runs the same measurements as tools/kv_sweep.py and tools/step_test.py from the page, on
// one motor in open loop: start thresholds (slow ramps), the steady-state curve (duty steps
// down from 100 %), then step responses for tau, K and kA; SIMC turns those into gains.

let cal = null;        // the running calibration: { motor, abort, encSamples, logSink }
let calResult = null;  // { motor, kickPlus, kickMinus, ffPlus, ffMinus, maxRpm, tau, k, theta, ka, presets }
const CAL_STEPS = ['kick', 'sweep', 'step', 'gains'];
const calChart = new LiveChart(document.getElementById('calChart'), {
  series: [{ key: 'rpm', label: () => t('spdSpeedNow'), short: '', color: '--series-1' }],
  format: (v) => `${Math.round(v)} RPM`,
  minSpan: 50,
  windowMs: 30000,
  labels: chartLabels,
});
const calEls = {
  card: document.getElementById('calCard'),
  badge: document.getElementById('calBadge'),
  motor: document.getElementById('calMotor'),
  start: document.getElementById('calStartBtn'),
  stop: document.getElementById('calStopBtn'),
  steps: document.getElementById('calSteps'),
  bar: document.getElementById('calBar'),
  status: document.getElementById('calStatus'),
  results: document.getElementById('calResults'),
  preset: document.getElementById('calPreset'),
  gains: document.getElementById('calGains'),
  save: document.getElementById('calSaveBtn'),
  kick: document.getElementById('calKickVal'),
  stopAt: document.getElementById('calStopVal'),
  max: document.getElementById('calMaxVal'),
  curve: document.getElementById('calCurveVal'),
  tau: document.getElementById('calTauVal'),
  k: document.getElementById('calKVal'),
  ka: document.getElementById('calKaVal'),
  theta: document.getElementById('calThetaVal'),
};
let calStatusText = '';
let calProgress = null;

class CalAbort extends Error {}

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
async function calWait(ms) {
  const end = performance.now() + ms;
  while (performance.now() < end) {
    if (!cal || cal.abort || !port) throw new CalAbort();
    await sleep(Math.min(50, end - performance.now()));
  }
}
const calRun = (duty) => send(`RUN,${cal.motor},${duty.toFixed(1)},0`);
const calStepState = (step, state, result) => {
  const li = calEls.steps.querySelector(`[data-step="${step}"]`);
  if (state) li.dataset.state = state;
  else delete li.dataset.state;
  if (result !== undefined) li.querySelector('.cal-result').textContent = result;
};
function calSay(text, progress) {
  calStatusText = text;
  if (progress !== undefined) calProgress = progress;
  render();
}
// Supply during the test, so curves and thresholds are stored as if at 12 V (FF_TABLE_V)
const calVolts = () => (battery?.measured ? battery.volts : 12);
const dirLabel = (dir) => (dir > 0 ? '+' : '−');

// Mean RPM over the next `ms`, from the ENC reports (25 Hz)
async function calAverage(ms) {
  cal.encSamples = [];
  await calWait(ms);
  const v = cal.encSamples;
  return v.length ? v.reduce((a, b) => a + b, 0) / v.length : 0;
}

// 1. Ramp the duty slowly until the motor turns; the duty then is the start threshold
async function calKick(dir) {
  const RATE = 4;        // % per second
  const LIMIT = 70;
  let duty = 0;
  let moving = 0;
  cal.encSamples = [];
  while (duty < LIMIT) {
    duty += RATE * 0.1;
    calRun(dir * duty);
    calSay(t('calRampDir', { dir: dirLabel(dir), duty: `${num(duty, 1)} %` }));
    await calWait(100);
    const last = cal.encSamples.slice(-2);
    cal.encSamples = [];
    const speed = last.length ? last[last.length - 1] : 0;
    if (Math.abs(speed) > 15) {
      if (Math.sign(speed) !== dir) {   // counts the wrong way: flip, let it stop, ramp again
        const m = cal.motor;
        encSettings.invert[m] = !encSettings.invert[m];
        saveEnc();
        send(`ENCINV,${m},${encSettings.invert[m] ? 1 : 0}`);
        scheduleSave(m);
        log(t('calFlipped'));
        send('STOP');
        await calWait(1500);
        duty = 0;
        moving = 0;
        cal.encSamples = [];
        continue;
      }
      if (++moving >= 2) break;
    } else {
      moving = 0;
    }
  }
  send('STOP');
  if (duty >= LIMIT) throw new Error(t('calNoStart', { duty: `${LIMIT} %` }));
  await calWait(1500);
  return (duty - RATE * 0.1) * calVolts() / 12;   // the step before it was seen moving
}

// 2. Steady-state speed at duties stepping down from 100 % until the motor stops
async function calSweep(dir, kick, done, total) {
  const duties = [100, 80, 60, 50, 40, 30, 25];
  for (let d = 22; d >= 2; d -= d > 15 ? 3 : 1) duties.push(d);
  const points = [];
  for (let i = 0; i < duties.length; i++) {
    const duty = duties[i];
    if (duty < kick * 0.3) break;
    calRun(dir * duty);
    calSay(t('calSweepAt', { dir: dirLabel(dir), duty: `${duty} %` }), (done + i / duties.length) / total);
    await calWait(i === 0 ? 2500 : 1300);
    const rpm = Math.abs(await calAverage(1000));
    points.push({ duty: duty * calVolts() / 12, rpm });
    if (rpm < Calib.STOPPED_RPM) break;
  }
  send('STOP');
  await calWait(1500);
  const table = Calib.buildFfTable(points);
  if (!table) throw new Error(t('calNoStart', { duty: '100 %' }));
  return table;
}

// 3. Step responses between ~30 % and ~60 % of top speed, logged at 200 Hz
async function calStepTest(table, dir) {
  const max = table[table.length - 1][0];
  const volts = calVolts();
  // back to real duty at this supply, rounded the way RUN sends it so the log matches
  const a = Math.round(Calib.dutyFor(table, 0.3 * max) * 12 / volts * 10) / 10;
  const b = Math.round(Calib.dutyFor(table, 0.6 * max) * 12 / volts * 10) / 10;
  const results = [];
  for (const [from, to] of [[a, b], [b, a]]) {
    calSay(t('calStepAt', { dir: dirLabel(dir), a: `${num(from, 0)} %`, b: `${num(to, 0)} %` }));
    calRun(dir * from);
    await calWait(2000);
    cal.logSink = [];
    send('LOG,1');
    await calWait(300);
    calRun(dir * to);
    await calWait(1500);
    send('LOG,0');
    await calWait(300);
    const r = Calib.analyseStep(cal.logSink, dir * from, dir * to, volts);
    cal.logSink = null;
    if (r) results.push(r);
  }
  send('STOP');
  await calWait(1500);
  return results;
}

async function runCalibration() {
  const m = Number(calEls.motor.value);
  if (!port) return calSay(t('calNeedPort'));
  if (!(encSettings.cpr[m] > 0)) return calSay(t('calNeedCpr'));
  stopSimTest();
  cal = { motor: m, abort: false, encSamples: [], logSink: null };
  calResult = null;
  calChart.clear();
  for (const step of CAL_STEPS) calStepState(step, null, '');
  let step = 'kick';
  try {
    calStepState('kick', 'run');
    const kickPlus = await calKick(1);
    const kickMinus = await calKick(-1);
    calStepState('kick', 'done', `+${num(kickPlus, 1)} / −${num(kickMinus, 1)} %`);

    step = 'sweep';
    calStepState('sweep', 'run');
    const ffPlus = await calSweep(1, kickPlus, 0, 2);
    const ffMinus = await calSweep(-1, kickMinus, 1, 2);
    const maxRpm = Math.floor(Math.min(ffPlus[ffPlus.length - 1][0], ffMinus[ffMinus.length - 1][0]) / 10) * 10;
    calStepState('sweep', 'done', `${maxRpm} RPM`);

    step = 'step';
    calStepState('step', 'run');
    const steps = [...await calStepTest(ffPlus, 1), ...await calStepTest(ffMinus, -1)];
    if (steps.length === 0) throw new Error('no step data');
    const mean = (k) => steps.reduce((acc, r) => acc + r[k], 0) / steps.length;
    const tau = mean('tau');
    const k = mean('k');
    const theta = mean('theta') + CONTROL_TICK_S;   // the fit sees the speed estimate's delay; add one tick
    calStepState('step', 'done', `τ ${num(tau * 1000, 0)} ms · K ${num(k, 0)}`);

    step = 'gains';
    calResult = {
      motor: m, kickPlus, kickMinus, ffPlus, ffMinus, maxRpm, tau, k, theta,
      ka: tau / k, presets: Calib.presets(tau, k, theta),
    };
    calStepState('gains', 'done', 'SIMC');
    calSay(t('calDone'), 1);
  } catch (e) {
    send('STOP');
    send('LOG,0');
    calStepState(step, 'fail');
    calSay(e instanceof CalAbort ? t('calStopped') : t('calFailed', { msg: e.message }), null);
  } finally {
    cal = null;
    render();
  }
}

function saveCalibration() {
  const r = calResult;
  if (!r || !port) return;
  const g = r.presets[calEls.preset.value];
  const ff = (rows) => rows.map(([rpm, duty]) => `${rpm.toFixed(1)}:${duty.toFixed(1)}`).join(';');
  send(`KICK,${r.motor},${r.kickPlus.toFixed(1)},${r.kickMinus.toFixed(1)}`);
  send(`FF,${r.motor},+,${ff(r.ffPlus)}`);
  send(`FF,${r.motor},-,${ff(r.ffMinus)}`);
  send(`MAXRPM,${r.motor},${r.maxRpm}`);
  send(`GAIN,${g.kp.toFixed(5)},${g.ki.toFixed(4)},${r.motor}`);
  send(`REF,100,${r.ka.toFixed(5)},${r.motor}`);
  send(`SAVE,${r.motor}`);
  send(`PROFILE?,${r.motor}`);
  calSay(t('calSaved', { m: r.motor }));
}

function renderCal(state) {
  const running = cal !== null;
  const online = state === 'online';
  calEls.start.disabled = running || !online;
  calEls.stop.disabled = !running;
  calEls.motor.disabled = running;
  calEls.badge.textContent = t(running ? 'testRunning' : online ? 'testIdle' : 'sensorWaiting');
  if (running) calEls.badge.dataset.level = 'run';
  else delete calEls.badge.dataset.level;
  calEls.status.textContent = calStatusText || t(online ? 'calReady' : 'calNeedPort');
  setMeter(calEls.bar, calProgress);
  if (calProgress !== null) calEls.bar.style.background = 'var(--accent)';

  const r = calResult;
  calEls.results.hidden = !r;
  if (!r) return;
  calEls.kick.textContent = `+${num(r.kickPlus, 1)} / −${num(r.kickMinus, 1)} %`;
  calEls.stopAt.textContent = t('calStopAt', { p: num(r.ffPlus[0][1], 1), m: num(r.ffMinus[0][1], 1) });
  calEls.max.textContent = `${r.maxRpm} RPM`;
  calEls.curve.textContent = t('calPoints', { p: r.ffPlus.length, m: r.ffMinus.length });
  calEls.tau.textContent = `${num(r.tau * 1000, 0)} ms`;
  calEls.k.textContent = t('calKUnit', { k: num(r.k, 1) });
  calEls.ka.textContent = num(r.ka, 5);
  calEls.theta.textContent = t('calTheta', { t: num(r.theta * 1000, 0) });
  const g = r.presets[calEls.preset.value];
  calEls.gains.innerHTML = `Kp <b>${num(g.kp, 4)}</b> · Ki <b>${num(g.ki, 3)}</b>`;
  calEls.save.disabled = !online || running;
}

function renderCalMotors() {
  const selected = calEls.motor.value || '1';
  calEls.motor.innerHTML = '';
  for (const [key, target] of Object.entries(TEST_TARGETS)) {
    calEls.motor.add(new Option(`${t(target.label)} · ${target.in1} / ${target.in2}`, key));
  }
  calEls.motor.value = selected;
}

calEls.start.addEventListener('click', () => {
  calStatusText = '';
  calProgress = 0;
  runCalibration();
});
calEls.stop.addEventListener('click', () => {
  if (cal) cal.abort = true;
  send('STOP');
});
document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && cal) cal.abort = true; });
calEls.preset.addEventListener('change', render);
calEls.save.addEventListener('click', saveCalibration);
renderCalMotors();

// Tells the firmware the page is alive; without it the motor stops after 1 s
setInterval(() => { if (port) send('PING'); }, 200);

// INA219 wiring: drawn on demand, motor 2 adds the A0 bridge for address 0x41
function wiringSvg(motor) {
  const pins = [['3V3', 'VCC', 'w-power'], ['GND', 'GND', 'w-gnd'], ['D21', 'SDA', 'w-sda'], ['D22', 'SCL', 'w-scl']];
  const y = (i) => 62 + i * 30;
  const left = pins.map(([esp, ina, cls], i) => `
    <line class="w-wire ${cls}" x1="150" y1="${y(i)}" x2="290" y2="${y(i)}"/>
    <circle class="w-dot" cx="150" cy="${y(i)}" r="3"/><circle class="w-dot" cx="290" cy="${y(i)}" r="3"/>
    <text class="w-pin" x="140" y="${y(i) + 4}" text-anchor="end">${esp}</text>
    <text class="w-pin" x="300" y="${y(i) + 4}">${ina}</text>`).join('');
  const lines = motor === 2
    ? [t('wiringShared'), t('wiringA0'), t('wiringGnd')]
    : [t('wiringAuto'), t('wiringGnd')];
  const notes = lines.map((text, i) => `<text class="w-note" x="20" y="${206 + i * 18}">${text}</text>`).join('');
  const height = 206 + lines.length * 18;
  return `<svg viewBox="0 0 640 ${height}" role="img" aria-label="INA219 → ESP32: VCC 3V3, GND GND, SDA D21, SCL D22; VIN+ 12 V, VIN− L298N">
    <rect class="w-box" x="20" y="30" width="130" height="150" rx="8"/>
    <text class="w-title" x="85" y="22" text-anchor="middle">ESP32</text>
    <rect class="w-box" x="290" y="30" width="150" height="150" rx="8"/>
    <text class="w-title" x="365" y="22" text-anchor="middle">INA219${motor === 2 ? ' #2' : ''}</text>
    ${left}
    <text class="w-pin" x="430" y="86" text-anchor="end">VIN+</text>
    <text class="w-pin" x="430" y="136" text-anchor="end">VIN−</text>
    <line class="w-wire w-power" x1="440" y1="82" x2="520" y2="82"/>
    <line class="w-wire w-power" x1="440" y1="132" x2="520" y2="132"/>
    <circle class="w-dot" cx="440" cy="82" r="3"/><circle class="w-dot" cx="440" cy="132" r="3"/>
    <rect class="w-box" x="520" y="64" width="115" height="36" rx="6"/>
    <text class="w-pin" x="577" y="86" text-anchor="middle">${t('wiringSupply')}</text>
    <rect class="w-box" x="520" y="114" width="115" height="36" rx="6"/>
    <text class="w-pin" x="577" y="136" text-anchor="middle">${t('wiringDriver')}${motor === 2 ? ' #2' : ''}</text>
    ${notes}
  </svg>`;
}

// Motor wiring: ESP32 -> L298N (ENA, IN1, IN2), L298N -> motor, encoder -> ESP32, adapter -> L298N
const MOTOR_WIRING = {
  1: { ena: 'D25', in1: 'D26', in2: 'D27', a: 'D34', b: 'D35' },
  2: { ena: 'D18', in1: 'D19', in2: 'D23', a: 'D16', b: 'D17' },
};

function motorWiringSvg(m) {
  const p = MOTOR_WIRING[m];
  const pin = (x, y) => `<circle class="w-dot" cx="${x}" cy="${y}" r="3"/>`;
  const wire = (cls, x1, y1, x2, y2) => `<line class="w-wire ${cls}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/>`;
  const path = (cls, d) => `<path class="w-wire ${cls}" d="${d}"/>`;
  const text = (x, y, label, anchor = 'start', cls = 'w-pin') => `<text class="${cls}" x="${x}" y="${y + 4}" text-anchor="${anchor}">${label}</text>`;
  // ESP32 -> L298N control lines
  const ctrl = [[p.ena, 'ENA', 70], [p.in1, 'IN1', 100], [p.in2, 'IN2', 130]].map(([esp, l298, y]) =>
    wire('w-ctrl', 150, y, 290, y) + pin(150, y) + pin(290, y) + text(140, y, esp, 'end') + text(300, y, l298)).join('');
  // Encoder -> ESP32, routed under the driver and the adapter
  const enc = [[p.a, 'A', 250, 'w-yellow'], [p.b, 'B', 280, 'w-green'], ['3V3', 'VCC', 310, 'w-blue'], ['GND', 'GND', 340, 'w-black']]
    .map(([esp, e, y, cls]) => wire(cls, 150, y, 570, y) + pin(150, y) + pin(570, y) + text(140, y, esp, 'end') + text(580, y, e)).join('');
  return `<svg viewBox="0 0 720 360" role="img" aria-label="${t('wiringMotor', { m })}: ENA ${p.ena}, IN1 ${p.in1}, IN2 ${p.in2}, A ${p.a}, B ${p.b}">
    <rect class="w-box" x="20" y="40" width="130" height="315" rx="8"/>
    <text class="w-title" x="85" y="32" text-anchor="middle">ESP32</text>
    <rect class="w-box" x="290" y="40" width="160" height="120" rx="8"/>
    <text class="w-title" x="370" y="32" text-anchor="middle">${t('wiringL298', { m })}</text>
    <rect class="w-box" x="570" y="40" width="130" height="315" rx="8"/>
    <text class="w-title" x="635" y="32" text-anchor="middle">${t('wiringMotorBox')}</text>
    ${ctrl}
    ${wire('w-motor', 450, 85, 570, 85)}${pin(450, 85)}${pin(570, 85)}${text(440, 85, 'OUT1', 'end')}${text(580, 85, 'M+')}
    ${wire('w-motor', 450, 115, 570, 115)}${pin(450, 115)}${pin(570, 115)}${text(440, 115, 'OUT2', 'end')}${text(580, 115, 'M−')}
    ${text(635, 225, t('wiringEncoder'), 'middle', 'w-note')}
    ${path('w-power', 'M335 160 V190')}${path('w-gnd', 'M405 160 V190')}
    ${pin(335, 160)}${pin(405, 160)}
    ${text(335, 148, '+12V', 'middle')}${text(405, 148, 'GND', 'middle')}
    <rect class="w-box" x="300" y="190" width="140" height="30" rx="6"/>
    ${text(370, 205, t('wiringAdapter'), 'middle')}
    ${enc}
  </svg>`;
}

// BNO055: the same four wires as the INA219s
function imuWiringSvg() {
  const pins = [['3V3', 'VIN', 'w-power'], ['GND', 'GND', 'w-gnd'], ['D21', 'SDA', 'w-sda'], ['D22', 'SCL', 'w-scl']];
  const y = (i) => 62 + i * 30;
  const rows = pins.map(([esp, b, cls], i) => `
    <line class="w-wire ${cls}" x1="150" y1="${y(i)}" x2="290" y2="${y(i)}"/>
    <circle class="w-dot" cx="150" cy="${y(i)}" r="3"/><circle class="w-dot" cx="290" cy="${y(i)}" r="3"/>
    <text class="w-pin" x="140" y="${y(i) + 4}" text-anchor="end">${esp}</text>
    <text class="w-pin" x="300" y="${y(i) + 4}">${b}</text>`).join('');
  return `<svg viewBox="0 0 460 224" role="img" aria-label="BNO055 → ESP32: VIN 3V3, GND GND, SDA D21, SCL D22">
    <rect class="w-box" x="20" y="30" width="130" height="150" rx="8"/>
    <text class="w-title" x="85" y="22" text-anchor="middle">ESP32</text>
    <rect class="w-box" x="290" y="30" width="150" height="150" rx="8"/>
    <text class="w-title" x="365" y="22" text-anchor="middle">${t('imuWiringBox')}</text>
    ${rows}
    <text class="w-note" x="20" y="210">${t('imuWiringNote')}</text>
  </svg>`;
}

function motorWiringHtml() {
  return MOTORS.map((m) => `<h4>${t('wiringMotor', { m })}</h4>${motorWiringSvg(m)}`).join('')
    + `<p class="w-notes">${t('wiringEnaNote')}<br>${t('wiringGndNote')}<br>${t('wiringEncNote')}</p>`;
}

function renderWiring() {
  document.querySelectorAll('[data-wiring]').forEach((b) => {
    const box = b.parentElement.closest('.power-hint, .wiring-link').querySelector('.wiring');
    const kind = b.dataset.wiring;
    const motors = kind === 'motors';
    b.textContent = t(motors ? (box.hidden ? 'wiringMotorsShow' : 'wiringMotorsHide') : (box.hidden ? 'wiringShow' : 'wiringHide'));
    if (!box.hidden) box.innerHTML = motors ? motorWiringHtml() : kind === 'imu' ? imuWiringSvg() : wiringSvg(Number(kind));
  });
}

document.querySelectorAll('[data-wiring]').forEach((b) => b.addEventListener('click', () => {
  const box = b.parentElement.closest('.power-hint, .wiring-link').querySelector('.wiring');
  box.hidden = !box.hidden;
  renderWiring();
}));
renderWiring();

document.querySelectorAll('[data-pfield="demo"]').forEach((b) => b.addEventListener('click', () => {
  if (demoTimer === null) startDemo();
  else stopDemo();
}));

// Reset: lowest voltage / peak current start again from now, and the charts start empty
document.querySelectorAll('[data-pfield="reset"]').forEach((b) => b.addEventListener('click', () => {
  for (const m of MOTORS) {
    const mp = motorPower[m];
    mp.minVoltage = mp.power?.voltage ?? null;
    mp.maxCurrentMa = mp.power?.currentMa ?? null;
  }
  currentChart.clear();
  voltageChart.clear();
  render();
}));

// ---------- Theme ----------

const THEME_STORAGE_KEY = 'esp32-dashboard-theme';
const systemDark = window.matchMedia('(prefers-color-scheme: dark)');
const themeBtn = document.getElementById('themeBtn');

const ICONS = {
  sun: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/></svg>',
  moon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 12.8A9 9 0 1 1 11.2 3a7 7 0 0 0 9.8 9.8z"/></svg>',
};

// Explicit choice wins; otherwise follow the OS setting
function currentTheme() {
  return document.documentElement.dataset.theme ?? (systemDark.matches ? 'dark' : 'light');
}

// One switch in the top bar for the area fill of every chart
const fillBtn = document.getElementById('fillBtn');
function renderFillButton() {
  fillBtn.setAttribute('aria-pressed', String(LiveChart.fill));
  fillBtn.title = t(LiveChart.fill ? 'chartFillOff' : 'chartFillOn');
  fillBtn.setAttribute('aria-label', fillBtn.title);
}
fillBtn.addEventListener('click', () => {
  LiveChart.setFill(!LiveChart.fill);
  renderFillButton();
});
renderFillButton();

function renderThemeButton() {
  const dark = currentTheme() === 'dark';
  themeBtn.innerHTML = dark ? ICONS.sun : ICONS.moon;
  themeBtn.title = dark ? t('themeToLight') : t('themeToDark');
  themeBtn.setAttribute('aria-label', themeBtn.title);
}

themeBtn.addEventListener('click', () => {
  const next = currentTheme() === 'dark' ? 'light' : 'dark';
  document.documentElement.dataset.theme = next;
  try { localStorage.setItem(THEME_STORAGE_KEY, next); } catch { /* storage unavailable */ }
  renderThemeButton();
});
systemDark.addEventListener('change', renderThemeButton);
renderThemeButton();

// ---------- Tabs ----------

const TAB_STORAGE_KEY = 'esp32-dashboard-tab';

function showTab(name) {
  document.querySelectorAll('.tab').forEach((t) => {
    t.setAttribute('aria-selected', String(t.dataset.tab === name));
  });
  document.querySelectorAll('.tab-panel').forEach((p) => {
    p.hidden = p.dataset.panel !== name;
  });
  try { localStorage.setItem(TAB_STORAGE_KEY, name); } catch { /* storage unavailable */ }
}

document.querySelectorAll('.tab').forEach((t) => {
  t.addEventListener('click', () => showTab(t.dataset.tab));
});

try {
  const saved = localStorage.getItem(TAB_STORAGE_KEY);
  if (saved && document.querySelector(`.tab[data-tab="${saved}"]`)) showTab(saved);
} catch { /* storage unavailable */ }

I18N.onChange(() => {
  renderThemeButton();
  renderFillButton();
  currentChart.renderLegend();
  voltageChart.renderLegend();
  encChart.renderLegend();
  spdChart.renderLegend();
  renderSpdMotors();
  renderWiring();
  mvChart.renderLegend();
  mvSpeedChart.renderLegend();
  driveRpmChart.renderLegend();
  renderCalMotors();
  renderMoveMotors();
  dutyChart.renderLegend();
  rpmChart.renderLegend();
  render();
  if ('serial' in navigator) refreshPorts();
});

render();
