// Must match SERIAL_BAUD in src/main.cpp
const BAUD_RATE = 115200;
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
  testMask: document.getElementById('testMask'),
  testHigh: document.getElementById('testHigh'),
  testLow: document.getElementById('testLow'),
  testPeriod: document.getElementById('testPeriod'),
  testCpr: document.getElementById('testCpr'),
  testStartBtn: document.getElementById('testStartBtn'),
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

// Square-wave motor test: latest MOTORS report per motor
const TEST_STATES = ['testIdle', 'testRunning', 'testWatchdog'];
const CPR_STORAGE_KEY = 'esp32-dashboard-cpr';
const newMotorRun = () => ({ duty: null, count: 0, cps: 0 });
const motorRun = { 1: newMotorRun(), 2: newMotorRun() };
let testState = null;   // null until the firmware reports MOTORS, then 0 idle / 1 running / 2 watchdog
let simTimer = null;    // simulated test while demo data is on and no board is connected

function countsPerRev() {
  const cpr = Number(els.testCpr.value);
  return cpr > 0 ? cpr : 0;
}

function formatSpeed(cps) {
  const cpr = countsPerRev();
  return cpr ? `${Math.round((cps * 60) / cpr)} RPM` : `${Math.round(cps)} ${t('cpsUnit')}`;
}

const dutyChart = new LiveChart(document.getElementById('dutyChart'), {
  series: motorSeries,
  legend: document.getElementById('dutyLegend'),
  format: (d) => `${d.toFixed(0)} %`,
  minSpan: 20,
  windowMs: 20000,
  labels: chartLabels,
});
const speedChart = new LiveChart(document.getElementById('speedChart'), {
  series: motorSeries,
  legend: document.getElementById('speedLegend'),
  format: (cps) => formatSpeed(cps),
  minSpan: 100,
  windowMs: 20000,
  labels: chartLabels,
});

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

  if (type === 'MOTORS' && fields.length === 7) {
    const [state, d1, c1, s1, d2, c2, s2] = fields.map(Number);
    testState = state;
    for (const [m, duty, count, cps] of [[1, d1, c1, s1], [2, d2, c2, s2]]) {
      motorRun[m] = { duty, count, cps };
      dutyChart.push(m, duty);
      speedChart.push(m, cps);
    }
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
  else if (state === 'online' && supported) badge = [TEST_STATES[testState] ?? 'testIdle', ['', 'run', 'bad'][testState] ?? ''];
  else if (demo) badge = ['testIdle', ''];
  else badge = ['sensorWaiting', ''];

  els.testCard.toggleAttribute('data-stale', !live);
  els.testBadge.textContent = t(badge[0]);
  if (badge[1]) els.testBadge.dataset.level = badge[1];
  else delete els.testBadge.dataset.level;
  els.testStartBtn.disabled = !((state === 'online' && supported) || (demo && !port));
  els.testStopBtn.disabled = !(simTimer !== null || (port && testState === 1));

  const cpr = countsPerRev();
  for (const row of els.testCard.querySelectorAll('[data-tmotor]')) {
    const m = motorRun[row.dataset.tmotor];
    const field = (name) => row.querySelector(`[data-tfield="${name}"]`);
    const has = m.duty !== null;
    field('duty').textContent = has ? `${m.duty.toFixed(0)} %` : '—';
    field('speed').textContent = has ? formatSpeed(m.cps) : '—';
    field('count').textContent = has ? Math.round(m.count).toLocaleString() : '—';
    field('revs').textContent = has && cpr ? t('revs', { n: (m.count / cpr).toFixed(1) }) : '—';
  }
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
  resetPowerData();
  resetTestData();
  render();
}

function resetPowerData() {
  for (const m of MOTORS) Object.assign(motorPower[m], newMotorPower());
  currentChart.clear();
  voltageChart.clear();
}

// ---------- Motor test ----------

function readTestParams() {
  const clampDuty = (v) => Math.max(-100, Math.min(100, Number(v) || 0));
  return {
    mask: Number(els.testMask.value),
    high: clampDuty(els.testHigh.value),
    low: clampDuty(els.testLow.value),
    period: Math.max(200, Number(els.testPeriod.value) || 2000),
  };
}

function startTest() {
  const p = readTestParams();
  dutyChart.clear();
  speedChart.clear();
  if (port) send(`SQW,${p.mask},${p.high},${p.low},${p.period}`);
  else if (demoTimer !== null) startSimTest(p);
  render();
}

function stopTest() {
  if (port) send('STOP');
  stopSimTest();
  render();
}

// Demo only: a first-order motor model (dead band, lag) answering the square wave,
// reported through the same MOTORS line the firmware sends
function startSimTest(p) {
  stopSimTest();
  const MAX_CPS = 4000;
  const DEAD_BAND = 15;   // % duty before the motor starts to turn
  const TAU_S = 0.15;
  const sim = { 1: { speed: 0, count: 0 }, 2: { speed: 0, count: 0 } };
  const start = performance.now();
  let last = start;
  simTimer = setInterval(() => {
    const now = performance.now();
    const dt = (now - last) / 1000;
    last = now;
    const highHalf = (now - start) % p.period < p.period / 2;
    const parts = [];
    for (const m of MOTORS) {
      const duty = p.mask & (1 << (m - 1)) ? (highHalf ? p.high : p.low) : 0;
      const drive = Math.max(0, Math.abs(duty) - DEAD_BAND) / (100 - DEAD_BAND);
      const target = Math.sign(duty) * drive * MAX_CPS * (m === 2 ? 0.92 : 1);
      sim[m].speed += (target - sim[m].speed) * Math.min(1, dt / TAU_S);
      sim[m].count += sim[m].speed * dt;
      const noise = Math.abs(sim[m].speed) > 1 ? (Math.random() - 0.5) * 40 : 0;
      parts.push(duty.toFixed(1), Math.round(sim[m].count), Math.round(sim[m].speed + noise));
    }
    handleLine(`MOTORS,1,${parts.join(',')}`);
  }, 40);
}

function stopSimTest() {
  if (simTimer === null) return;
  clearInterval(simTimer);
  simTimer = null;
  testState = 0;
  for (const m of MOTORS) motorRun[m] = { ...motorRun[m], duty: 0, cps: 0 };
}

function resetTestData() {
  testState = null;
  for (const m of MOTORS) motorRun[m] = newMotorRun();
  dutyChart.clear();
  speedChart.clear();
}

els.testStartBtn.addEventListener('click', startTest);
els.testStopBtn.addEventListener('click', stopTest);
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') stopTest(); });

try {
  const savedCpr = localStorage.getItem(CPR_STORAGE_KEY);
  if (savedCpr) els.testCpr.value = savedCpr;
} catch { /* storage unavailable */ }
els.testCpr.addEventListener('input', () => {
  try { localStorage.setItem(CPR_STORAGE_KEY, els.testCpr.value); } catch { /* storage unavailable */ }
  render();
});

// Tells the firmware the page is alive; without it the motors stop after 1 s
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
  dutyChart.renderLegend();
  speedChart.renderLegend();
  render();
  if ('serial' in navigator) refreshPorts();
});

render();
