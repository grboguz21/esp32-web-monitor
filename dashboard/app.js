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
  addPortBtn: document.getElementById('addPortBtn'),
  portSelect: document.getElementById('portSelect'),
  unsupported: document.getElementById('unsupported'),
  statusCard: document.getElementById('statusCard'),
  statusLabel: document.getElementById('statusLabel'),
  statusHint: document.getElementById('statusHint'),
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
  hMin: document.getElementById('hMin'),
  hBlock: document.getElementById('hBlock'),
  hBlockSub: document.getElementById('hBlockSub'),
  hLoopAvg: document.getElementById('hLoopAvg'),
  hLoopMax: document.getElementById('hLoopMax'),
  hLoopHz: document.getElementById('hLoopHz'),
  hStack: document.getElementById('hStack'),
  powerCards: document.querySelectorAll('.power'),
  testCard: document.getElementById('testCard'),
  testBadge: document.getElementById('testBadge'),
  testPin: document.getElementById('testPin'),
  testDirBtns: document.querySelectorAll('[data-dir]'),
  testDuty: document.getElementById('testDuty'),
  testDuration: document.getElementById('testDuration'),
  testDutyNow: document.getElementById('testDutyNow'),
  pwmChartTitle: document.getElementById('pwmChartTitle'),
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
  ksMotor: document.getElementById('ksMotor'),
  ksType: document.getElementById('ksType'),
  ksStatus: document.getElementById('ksStatus'),
  rampMax: document.getElementById('rampMax'),
  rampMaxLabel: document.getElementById('rampMaxLabel'),
  rampTime: document.getElementById('rampTime'),
  rampPlusBtn: document.getElementById('rampPlusBtn'),
  rampMinusBtn: document.getElementById('rampMinusBtn'),
  rampStopBtn: document.getElementById('rampStopBtn'),
  ksRecordBtn: document.getElementById('ksRecordBtn'),
  ksCells: {
    startPlus: document.getElementById('ksStartPlus'),
    startMinus: document.getElementById('ksStartMinus'),
    stopPlus: document.getElementById('ksStopPlus'),
    stopMinus: document.getElementById('ksStopMinus'),
  },
  ksSupply: document.getElementById('ksSupply'),
  ksResetBtn: document.getElementById('ksResetBtn'),
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

const HEAP_WARN_RATIO = 0.2;     // warn below 20 % free heap
const STACK_WARN_BYTES = 1024;

let knownPorts = [];    // ports this page already has permission for
let port = null;        // the open port
let reader = null;
let readableClosed = null;
let device = null;      // { name, version }
let info = null;        // chip info and reset reason, from INFO
let health = null;      // latest HEALTH values
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
  format: (v) => `${v.toFixed(2)} V`,
  tickFormat: (v) => `${v.toFixed(1)} V`,
  minSpan: 0.5,
  labels: chartLabels,
});

// Motor direction test: one motor at a signed duty, reported back as RUNOUT
const TEST_STATES = ['testIdle', 'testRunning', 'testWatchdog', 'testDone'];
const TEST_MOTOR_STORAGE_KEY = 'esp32-dashboard-test-motor';
// Test targets (board labels): each motor's L298N inputs, + drives IN1 and - drives IN2;
// target 3 is the onboard LED on D2, which has no direction
const TEST_TARGETS = {
  1: { label: 'motor1', in1: 'D26', in2: 'D27' },
  2: { label: 'motor2', in1: 'D19', in2: 'D23' },
  3: { label: 'testLed', pin: 'D2' },
};
const isLedTarget = (target) => Boolean(TEST_TARGETS[target]?.pin);
let testDirection = 1;   // +1 or -1
let testState = null;    // null until the firmware reports RUNOUT, then 0 idle / 1 running / 2 watchdog / 3 done
let testOutput = null;   // { motor, duty, remainingMs }
let simTimer = null;     // simulated run while demo data is on and no board is connected

// Start and stop thresholds per motor and direction, saved in this browser
const KS_STORAGE_KEY = 'esp32-dashboard-ks';
// { '1': { startPlus, startMinus, stopPlus, stopMinus }, ... } duty in %
let ksResults = {};
try { ksResults = JSON.parse(localStorage.getItem(KS_STORAGE_KEY)) ?? {}; } catch { /* storage unavailable */ }
for (const entry of Object.values(ksResults)) {
  // Earlier versions stored only start thresholds as plus/minus
  if ('plus' in entry) { entry.startPlus ??= entry.plus; delete entry.plus; }
  if ('minus' in entry) { entry.startMinus ??= entry.minus; delete entry.minus; }
}
let ramp = null;        // { timer, motor, dir, type } while a ramp runs

// Encoders: latest ENC report, counts per output-shaft revolution and sign flip per motor
const ENC_STORAGE_KEY = 'esp32-dashboard-encoder';
let encSettings = { cpr: {}, invert: {} };
try { encSettings = { ...encSettings, ...JSON.parse(localStorage.getItem(ENC_STORAGE_KEY)) }; } catch { /* storage unavailable */ }
const newEnc = () => ({ count: null, cps: 0 });
const encData = { 1: newEnc(), 2: newEnc() };
const encDirCheck = { 1: null, 2: null };   // null unknown, true matches +, false reversed
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
  format: (v) => (els.mvUnit.value === 'm' ? `${v.toFixed(2)} m` : `${v.toFixed(2)}`),
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

function saveKs() {
  try { localStorage.setItem(KS_STORAGE_KEY, JSON.stringify(ksResults)); } catch { /* storage unavailable */ }
}

const signedPercent = (d) => `${d > 0 ? '+' : d < 0 ? '−' : ''}${Math.abs(d).toFixed(0)} %`;

const pwmChart = new LiveChart(document.getElementById('pwmChart'), {
  series: [{ key: 'pwm', label: () => t('tDuty'), short: '', color: '--accent' }],
  format: signedPercent,
  range: [-100, 100],
  windowMs: 20000,
  labels: chartLabels,
});

function renderTestPins() {
  let selected = els.testPin.value;
  if (!selected) {
    try { selected = localStorage.getItem(TEST_MOTOR_STORAGE_KEY); } catch { /* storage unavailable */ }
  }
  els.testPin.innerHTML = '';
  for (const [key, target] of Object.entries(TEST_TARGETS)) {
    const pins = target.pin ?? `${target.in1} / ${target.in2}`;
    els.testPin.add(new Option(`${t(target.label)} · ${pins}`, key));
  }
  els.testPin.value = selected in TEST_TARGETS ? selected : '1';

  const ksSelected = els.ksMotor.value || '1';
  els.ksMotor.innerHTML = '';
  for (const [key, target] of Object.entries(TEST_TARGETS)) {
    if (target.pin) continue;   // thresholds are for motors only
    els.ksMotor.add(new Option(`${t(target.label)} · ${target.in1} / ${target.in2}`, key));
  }
  els.ksMotor.value = ksSelected;
}

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
  port = selected;
  log(t('log.opened', { port: portLabel(port) }));
  render();
  readLoop();
}

async function disconnect() {
  // Cancelling the reader ends readLoop, which then closes the port
  if (reader) await reader.cancel().catch(() => {});
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
      for (const line of lines) {
        // One bad message must not stop the read loop
        try {
          handleLine(sanitize(line));
        } catch (err) {
          log(t('log.badLine', { msg: err.message }));
        }
      }
    }
  } catch (err) {
    log(t('log.readError', { msg: err.message }));
  } finally {
    reader.releaseLock();
  }
  await closePort();
}

async function closePort() {
  await readableClosed;
  try {
    await port.close();
  } catch {
    // Port may already be gone (cable unplugged)
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
    sendEncoderSettings();   // the firmware starts with no sign flips after a reset
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
      // While this motor is driven, does + duty give positive counts?
      const duty = testOutput?.motor === m && testState === 1 ? testOutput.duty : 0;
      if (duty !== 0 && Math.abs(cps) > 20) encDirCheck[m] = Math.sign(cps) === Math.sign(duty);
    }
    return;
  }

  if (type === 'RUNOUT' && fields.length === 4) {
    const [state, motor, duty, remainingMs] = fields.map(Number);
    testState = state;
    testOutput = { motor, duty, remainingMs };
    pwmChart.push('pwm', duty);
    return;
  }

  // Anything else (boot messages, debug prints) goes to the log as-is
  log(line);
}

// ---------- UI ----------

function currentState() {
  if (!port) return 'offline';
  if (lastBeatAt && performance.now() - lastBeatAt < HEARTBEAT_TIMEOUT_MS) return 'online';
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
  els.statusHint.textContent = t(`state.${state}.hint`);

  els.connectBtn.textContent = port ? t('disconnect') : t('connect');
  els.connectBtn.disabled = !port && knownPorts.length === 0;
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
  if (n >= 1024 * 1024) return `${(n / 1024 / 1024).toFixed(1)} MB`;
  if (n >= 1024) return `${(n / 1024).toFixed(1)} KB`;
  return `${n} B`;
}

function formatRate(hz) {
  if (hz >= 1e6) return `${(hz / 1e6).toFixed(2)} MHz`;
  if (hz >= 1000) return `${(hz / 1000).toFixed(1)} kHz`;
  return `${hz} Hz`;
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

function renderHealth(state) {
  els.healthCard.toggleAttribute('data-stale', state !== 'online');

  if (!health) {
    for (const el of [els.hFree, els.hFreeSub, els.hMin, els.hBlock, els.hBlockSub,
                      els.hLoopAvg, els.hLoopMax, els.hLoopHz, els.hStack]) {
      el.textContent = '—';
    }
    els.hFreeBar.style.width = '0';
    return;
  }

  const total = info?.heap;
  const freeRatio = total ? health.freeHeap / total : null;
  const heapLow = freeRatio !== null && freeRatio < HEAP_WARN_RATIO;
  els.hFree.textContent = formatBytes(health.freeHeap);
  els.hFree.classList.toggle('warn', heapLow);
  els.hFreeBar.style.width = freeRatio !== null ? `${(freeRatio * 100).toFixed(1)}%` : '0';
  els.hFreeBar.classList.toggle('warn', heapLow);
  els.hFreeSub.textContent = total
    ? t('heapOf', { pct: Math.round(freeRatio * 100), total: formatBytes(total) })
    : t('totalUnknown');

  els.hMin.textContent = formatBytes(health.minFreeHeap);

  // Fragmentation: how much of the free heap is NOT usable as one block
  const frag = health.freeHeap ? 1 - health.maxAlloc / health.freeHeap : 0;
  els.hBlock.textContent = formatBytes(health.maxAlloc);
  els.hBlockSub.textContent = t('fragmentation', { pct: Math.round(frag * 100) });

  els.hLoopAvg.textContent = `${health.loopAvgUs} µs`;
  els.hLoopMax.textContent = t('worst', { us: health.loopMaxUs });
  els.hLoopHz.textContent = formatRate(health.loopHz);

  els.hStack.textContent = formatBytes(health.stackFree);
  els.hStack.classList.toggle('warn', health.stackFree < STACK_WARN_BYTES);
}

function formatCurrent(ma) {
  if (Math.abs(ma) >= 1000) return `${(ma / 1000).toFixed(2)} A`;
  return `${Math.round(ma)} mA`;
}

function formatPower(mw) {
  if (Math.abs(mw) >= 1000) return `${(mw / 1000).toFixed(2)} W`;
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
            voltage: `${m.power.voltage.toFixed(2)} V`,
            voltageSub: t('lowestV', { v: `${m.minVoltage.toFixed(2)} V` }),
            current: formatCurrent(m.power.currentMa),
            currentSub: t('peakI', { i: formatCurrent(m.maxCurrentMa) }),
            power: formatPower(m.power.powerMw),
          }
        : { voltage: '—', voltageSub: '—', current: '—', currentSub: '—', power: '—' };
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
  const ledSelected = isLedTarget(Number(els.testPin.value));
  for (const b of els.testDirBtns) {
    b.setAttribute('aria-pressed', String(!ledSelected && Number(b.dataset.dir) === testDirection));
    b.disabled = ledSelected;
  }
  els.testDuration.placeholder = t('durationHint');

  // Which pin gets the PWM: the running target and direction, or what is selected now
  const running = (simTimer !== null || testState === 1) && testOutput;
  const target = running ? testOutput.motor : Number(els.testPin.value);
  const info = TEST_TARGETS[target] ?? TEST_TARGETS[1];
  const dir = running && testOutput.duty !== 0 ? Math.sign(testOutput.duty) : testDirection;
  els.pwmChartTitle.textContent = info.pin
    ? t('runTargetLed', { led: t(info.label), pin: info.pin })
    : t('runTarget', {
        motor: t(info.label),
        dir: dir > 0 ? '+' : '−',
        pin: dir > 0 ? info.in1 : info.in2,
        input: dir > 0 ? 'IN1' : 'IN2',
      });
  const parts = [];
  if (running) {
    parts.push(signedPercent(testOutput.duty));
    if (testOutput.remainingMs > 0) parts.push(t('remaining', { s: (testOutput.remainingMs / 1000).toFixed(1) }));
  }
  els.testDutyNow.textContent = parts.join(' · ');
  renderSpd(state);
  renderMove(state);
  renderEnc(state);
  renderKs(state);
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
  ? `${mvFromCounts(counts).toFixed(3)} m`
  : `${mvFromCounts(counts).toFixed(2)} ${t('mvTurnsShort')}`);
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
  const maxRpm = Math.max(40, Math.min(1100, Number(els.mvMaxRpm.value) || 300));
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
      ? fmt('mvEstReach', { rpm: Math.round(e.peakRpm), s: e.reachS.toFixed(2) })
      : fmt('mvEstPeak', { max: e.maxRpm, rpm: Math.round(e.peakRpm), s: e.reachS.toFixed(2) }));
    parts.push(fmt('mvEstTotal', { s: e.totalS.toFixed(2) }));
    parts.push(fmt('mvEstDistance', { turns: e.turns.toFixed(2) }));
  }
  if (mvTookMs !== null) parts.push(t('mvEstLast', { s: `<b>${(mvTookMs / 1000).toFixed(2)}</b>` }));
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
  els.mvTargetSub.textContent = show ? mvDetail(mv.target) : '—';
  els.mvTraveled.textContent = show ? mvFormat(mv.pos) : '—';
  els.mvTraveledSub.textContent = show ? mvDetail(mv.pos) : '—';
  els.mvRemaining.textContent = show ? mvFormat(mv.target - mv.pos) : '—';
  els.mvProfile.textContent = moving && mv.phase === 1 ? `${Math.round(mv.profileRpm)} RPM` : '—';
  const finished = show && (mv.phase === 3 || mv.phase === 2);
  els.mvFinalError.textContent = finished ? mvFormat(mv.target - mv.pos) : '—';
  els.mvFinalSub.textContent = finished ? mvDetail(mv.target - mv.pos) : '—';
}

function renderSpd(state) {
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
  els.spdDutySplit.textContent = show ? `FF ${spd.ff.toFixed(1)} % · PI ${spd.pi >= 0 ? '+' : ''}${spd.pi.toFixed(1)} %` : '—';
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
    field('turns').textContent = has && cpr ? t('encTurns', { n: (d.count / cpr).toFixed(2) }) : '—';
    field('speed').textContent = !has ? '—'
      : cpr ? `${Math.round((d.cps * 60) / cpr)} RPM` : `${Math.round(d.cps)} ${t('cpsUnit')}`;
    const check = encDirCheck[m];
    field('dir').textContent = check === null ? t('encDirUnknown') : check ? t('encDirOk') : t('encDirBad');
    field('dir').dataset.state = check === null ? '' : check ? 'ok' : 'bad';
    field('invert').checked = Boolean(encSettings.invert[m]);
    field('zero').disabled = !live;
    field('oneTurn').disabled = !live || !d.count;
    if (document.activeElement !== field('cpr')) field('cpr').value = cpr || '';
  }
}

function renderKs(state) {
  const canRun = (state === 'online' && testState !== null) || (demoTimer !== null && !port);
  const type = ramp ? ramp.type : els.ksType.value;
  els.rampMaxLabel.textContent = t(els.ksType.value === 'start' ? 'rampMax' : 'rampFrom');
  els.rampPlusBtn.disabled = !canRun;
  els.rampMinusBtn.disabled = !canRun;
  els.ksRecordBtn.disabled = !ramp;
  els.rampStopBtn.disabled = !ramp;
  els.ksRecordBtn.textContent = t(type === 'start' ? 'ksRecordStart' : 'ksRecordStop');
  els.ksStatus.textContent = ramp && testOutput
    ? t('ksStatusRun', {
        type: t(ramp.type === 'start' ? 'ksStart' : 'ksStop'),
        dir: ramp.dir > 0 ? '+' : '−',
        duty: `${Math.abs(testOutput.duty).toFixed(1)} %`,
      })
    : '';

  const result = ksResults[els.ksMotor.value] ?? {};
  const supply = Number(els.ksSupply.value) || 12;
  for (const [key, cell] of Object.entries(els.ksCells)) {
    const duty = result[key];
    cell.textContent = duty == null ? '—' : `${duty.toFixed(1)} % · ${(duty / 100 * supply).toFixed(2)} V`;
  }
  els.ksResetBtn.disabled = Object.keys(result).length === 0;
}

function log(message) {
  const time = new Date().toLocaleTimeString(I18N.lang === 'tr' ? 'tr-TR' : 'en-GB');
  const lines = (els.log.textContent + `[${time}] ${message}\n`).split('\n');
  els.log.textContent = lines.slice(-MAX_LOG_LINES - 1).join('\n');
  els.log.scrollTop = els.log.scrollHeight;
}

// ---------- Init ----------

if (!('serial' in navigator)) {
  els.unsupported.hidden = false;
  els.connectBtn.disabled = true;
  els.addPortBtn.disabled = true;
  els.portSelect.disabled = true;
} else {
  els.connectBtn.addEventListener('click', () => (port ? disconnect() : connect()));
  els.addPortBtn.addEventListener('click', addPort);
  // A previously allowed device was plugged in or unplugged
  navigator.serial.addEventListener('connect', () => refreshPorts());
  navigator.serial.addEventListener('disconnect', (e) => {
    if (e.target === port) log(t('log.unplugged'));
    if (!port) refreshPorts();
  });
  refreshPorts();
  setInterval(render, 250);
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

// The firmware starts with defaults after every reset, so the page re-sends its settings
function sendEncoderSettings() {
  for (const m of MOTORS) {
    send(`ENCINV,${m},${encSettings.invert[m] ? 1 : 0}`);
    if (encSettings.cpr[m] > 0) send(`CPR,${m},${encSettings.cpr[m]}`);
  }
  sendGains();
}

// ---------- Speed control ----------

function sendGains() {
  const kp = Math.max(0, Number(els.spdKp.value) || 0);
  const ki = Math.max(0, Number(els.spdKi.value) || 0);
  const tauRef = Math.max(5, Number(els.spdTauRef.value) || 100);
  const ka = Math.max(0, Number(els.spdKa.value) || 0);
  if (port) {
    send(`GAIN,${kp},${ki}`);
    send(`REF,${tauRef},${ka}`);
  }
}

function startSpeed() {
  stopRamp();
  stopSimTest();
  sendGains();
  const target = Math.max(-1200, Math.min(1200, Number(els.spdTarget.value) || 0));
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
for (const el of [els.spdKp, els.spdKi, els.spdTauRef, els.spdKa]) el.addEventListener('change', sendGains);
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
  stopRamp();
  stopSimTest();
  const motor = Number(els.mvMotor.value);
  mvMotor = motor;
  const amount = Number(els.mvAmount.value) || 0;
  const turns = els.mvUnit.value === 'm' ? amount / (Math.PI * mvWheelM()) : amount;
  const counts = Math.round(turns * mvCpr());
  const maxRpm = Math.max(40, Math.min(1100, Number(els.mvMaxRpm.value) || 300));
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
  const CPR = 700;
  const sim = { count: { 1: 0, 2: 0 }, speed: { 1: 0, 2: 0 }, last: performance.now() };
  sim.timer = setInterval(() => {
    const now = performance.now();
    const dt = (now - sim.last) / 1000;
    sim.last = now;
    const parts = [];
    for (const m of MOTORS) {
      const duty = testOutput?.motor === m && testState === 1 ? testOutput.duty : 0;
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
    encDirCheck[m] = null;
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
    encChart.clear();
    render();
  });
  field('cpr').addEventListener('change', () => {
    const cpr = Math.round(Number(field('cpr').value) * 100) / 100;
    if (cpr > 0) encSettings.cpr[m] = cpr;
    else delete encSettings.cpr[m];
    saveEnc();
    encChart.clear();
    render();
  });
  field('invert').addEventListener('change', () => {
    encSettings.invert[m] = field('invert').checked;
    saveEnc();
    encDirCheck[m] = null;   // re-check with the new sign
    if (port) send(`ENCINV,${m},${encSettings.invert[m] ? 1 : 0}`);
    render();
  });
}

// ---------- Motor direction test ----------

function readTestParams() {
  const duty = Math.max(0, Math.min(100, Number(els.testDuty.value) || 0));
  const target = Number(els.testPin.value);
  return {
    motor: target,
    duty: isLedTarget(target) ? duty : duty * testDirection,
    durationMs: Math.max(0, Math.round((Number(els.testDuration.value) || 0) * 1000)),
  };
}

// The chart is never cleared here, so starts, stops and changes stay on it as history
function startTest() {
  stopRamp();
  sendRun(readTestParams());
  render();
}

function sendRun(p) {
  if (port) send(`RUN,${p.motor},${p.duty},${p.durationMs}`);
  else if (demoTimer !== null) startSimTest(p);
}

// Threshold ramps: 'start' climbs from 0 to the limit (press Save when the motor starts
// turning); 'stop' falls from the limit to 0 (press Save when it stops)
function startRamp(dir) {
  stopRamp();
  const motor = Number(els.ksMotor.value);
  const type = els.ksType.value;
  const max = Math.max(1, Math.min(100, Number(els.rampMax.value) || 50));
  const durationMs = Math.max(1, Number(els.rampTime.value) || 30) * 1000;
  const start = performance.now();
  const step = () => {
    const f = Math.min(1, (performance.now() - start) / durationMs);
    const duty = (type === 'start' ? f : 1 - f) * max;
    sendRun({ motor, duty: duty * dir, durationMs: 0 });
    if (f >= 1) stopTest();   // ran out without a save
  };
  ramp = { timer: setInterval(step, 100), motor, dir, type };
  step();
  render();
}

function stopRamp() {
  if (!ramp) return;
  clearInterval(ramp.timer);
  ramp = null;
}

function recordThreshold() {
  if (!ramp || !testOutput) return;
  const entry = (ksResults[ramp.motor] ??= {});
  const key = `${ramp.type}${ramp.dir > 0 ? 'Plus' : 'Minus'}`;
  entry[key] = Math.round(Math.abs(testOutput.duty) * 10) / 10;
  saveKs();
  stopTest();
}

function stopTest() {
  stopRamp();
  if (port) send('STOP');
  else if (simTimer !== null) startSimIdle(testOutput?.motor ?? 1, 0);
  render();
}

// Demo only: reports the same RUNOUT lines the firmware would send, including the
// idle ones (duty 0) between runs, so the chart keeps flowing like with a real board
let simIdleTimer = null;

function startSimTest(p) {
  stopSimTest();
  const start = performance.now();
  simTimer = setInterval(() => {
    const elapsed = performance.now() - start;
    if (p.durationMs > 0 && elapsed >= p.durationMs) {
      startSimIdle(p.motor, 3);
      render();
      return;
    }
    const remaining = p.durationMs > 0 ? Math.round(p.durationMs - elapsed) : 0;
    handleLine(`RUNOUT,1,${p.motor},${p.duty.toFixed(1)},${remaining}`);
  }, 40);
}

function startSimIdle(motor, state) {
  stopSimTest();
  simIdleTimer = setInterval(() => handleLine(`RUNOUT,${state},${motor},0.0,0`), 40);
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
  pwmChart.clear();
}

els.testStartBtn.addEventListener('click', () => startTest());
els.testStopBtn.addEventListener('click', stopTest);
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') stopTest(); });
for (const b of els.testDirBtns) {
  b.addEventListener('click', () => {
    testDirection = Number(b.dataset.dir);
    // Flip a running motor straight away so the direction can be compared
    if (!ramp && (simTimer !== null || (port && testState === 1))) startTest();
    else render();
  });
}

renderTestPins();
// A new duty takes effect right away while the motor runs (Enter or leaving the field)
els.testDuty.addEventListener('change', () => {
  if (!ramp && (simTimer !== null || (port && testState === 1))) startTest();
});
els.rampPlusBtn.addEventListener('click', () => startRamp(1));
els.rampMinusBtn.addEventListener('click', () => startRamp(-1));
els.rampStopBtn.addEventListener('click', stopTest);
els.ksRecordBtn.addEventListener('click', recordThreshold);
els.ksType.addEventListener('change', () => {
  // A stop test should begin above the start threshold, so suggest a higher value
  els.rampMax.value = els.ksType.value === 'start' ? 50 : 40;
  render();
});
els.ksMotor.addEventListener('change', render);
els.ksSupply.addEventListener('input', render);
els.ksResetBtn.addEventListener('click', () => {
  delete ksResults[els.ksMotor.value];
  saveKs();
  render();
});
els.testPin.addEventListener('change', () => {
  try { localStorage.setItem(TEST_MOTOR_STORAGE_KEY, els.testPin.value); } catch { /* storage unavailable */ }
  render();
});

// Tells the firmware the page is alive; without it the motor stops after 1 s
setInterval(() => { if (port) send('PING'); }, 200);

document.querySelectorAll('[data-pfield="demo"]').forEach((b) => b.addEventListener('click', () => {
  if (demoTimer === null) startDemo();
  else stopDemo();
}));

document.querySelectorAll('[data-pfield="reset"]').forEach((b) => b.addEventListener('click', () => {
  for (const m of MOTORS) {
    const mp = motorPower[m];
    mp.minVoltage = mp.power?.voltage ?? null;
    mp.maxCurrentMa = mp.power?.currentMa ?? null;
  }
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
  currentChart.renderLegend();
  voltageChart.renderLegend();
  encChart.renderLegend();
  spdChart.renderLegend();
  renderSpdMotors();
  mvChart.renderLegend();
  mvSpeedChart.renderLegend();
  renderMoveMotors();
  renderTestPins();
  render();
  if ('serial' in navigator) refreshPorts();
});

render();
