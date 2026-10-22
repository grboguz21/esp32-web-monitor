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
let power = null;       // latest POWER averages: { voltage, currentMa, powerMw }
let powerSensor = 'unknown';  // 'unknown' (no POWER yet), 'missing' or 'ok'
let minVoltage = null;  // extremes since connecting or the last reset
let maxCurrentMa = null;

let demoTimer = null;   // fake POWER data for previewing the Power card without a sensor

const chartLabels = {
  empty: () => t('noData'),
  axis: (s) => (s === 0 ? t('now') : t('axisAgo', { n: s })),
  ago: (s) => (s === 0 ? t('now') : t('secAgo', { n: s })),
};
const currentChart = new LiveChart(document.getElementById('currentChart'), {
  format: (ma) => formatCurrent(ma),
  minSpan: 50,
  labels: chartLabels,
});
const voltageChart = new LiveChart(document.getElementById('voltageChart'), {
  format: (v) => `${v.toFixed(2)} V`,
  tickFormat: (v) => `${v.toFixed(1)} V`,
  minSpan: 0.5,
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
  uptimeMs = null;
  lastBeatAt = 0;
  log(t('log.closed'));
  await refreshPorts();
}

async function send(text) {
  if (!port?.writable) return;
  const writer = port.writable.getWriter();
  try {
    await writer.write(new TextEncoder().encode(text + '\n'));
  } finally {
    writer.releaseLock();
  }
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

  if (type === 'POWER' && fields[0] === 'NA') {
    powerSensor = 'missing';
    power = null;
    return;
  }

  if (type === 'POWER' && fields.length === 5) {
    const [voltage, currentMa, powerMw, windowMinV, windowMaxMa] = fields.map(Number);
    power = { voltage, currentMa, powerMw };
    powerSensor = 'ok';
    currentChart.push(currentMa);
    voltageChart.push(voltage);
    minVoltage = minVoltage === null ? windowMinV : Math.min(minVoltage, windowMinV);
    maxCurrentMa = maxCurrentMa === null ? windowMaxMa : Math.max(maxCurrentMa, windowMaxMa);
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

// The same Power card appears on the Dashboard and the Power tab
function renderPower(state) {
  const demo = demoTimer !== null;
  const live = state === 'online' || demo;
  const sensor = demo ? 'demo' : state === 'online' ? powerSensor : 'unknown';
  const badge = {
    ok: ['sensorOk', 'ok'],
    missing: ['sensorMissing', 'bad'],
    unknown: ['sensorWaiting', ''],
    demo: ['sensorDemo', 'demo'],
  }[sensor];
  const values = power
    ? {
        voltage: `${power.voltage.toFixed(2)} V`,
        voltageSub: t('lowestV', { v: `${minVoltage.toFixed(2)} V` }),
        current: formatCurrent(power.currentMa),
        currentSub: t('peakI', { i: formatCurrent(maxCurrentMa) }),
        power: formatPower(power.powerMw),
      }
    : { voltage: '—', voltageSub: '—', current: '—', currentSub: '—', power: '—' };

  for (const card of els.powerCards) {
    const field = (name) => card.querySelector(`[data-pfield="${name}"]`);
    card.dataset.sensor = sensor;
    card.toggleAttribute('data-stale', !live);
    field('badge').textContent = t(badge[0]);
    if (badge[1]) field('badge').dataset.level = badge[1];
    else delete field('badge').dataset.level;
    field('hint').hidden = sensor !== 'missing';
    field('reset').disabled = !power;
    field('demo').textContent = demo ? t('demoStop') : t('demoStart');
    field('demo').disabled = !!port;
    for (const [name, text] of Object.entries(values)) field(name).textContent = text;
  }

  currentChart.draw();
  voltageChart.draw();
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

// Fakes the firmware's POWER messages: ~300 mA idle, a load bump every ~15 s
// and a short stall now and then, with the supply sagging as current rises
function startDemo() {
  resetPowerData();
  let tick = 0;
  demoTimer = setInterval(() => {
    tick++;
    const phase = tick % 60;
    const load = phase > 30 && phase < 50 ? 450 * Math.sin(((phase - 30) / 20) * Math.PI) : 0;
    const stall = tick % 160 > 152 ? 750 : 0;
    const currentMa = 290 + 20 * Math.sin(tick / 5) + 12 * Math.random() + load + stall;
    const voltage = 12.02 - currentMa * 0.0006 + 0.01 * (Math.random() - 0.5);
    const minV = voltage - 0.02 * Math.random();
    const maxMa = currentMa + 15 * Math.random();
    handleLine(`POWER,${voltage.toFixed(3)},${currentMa.toFixed(1)},${(voltage * currentMa).toFixed(1)},` +
               `${minV.toFixed(3)},${maxMa.toFixed(1)}`);
    render();
  }, 250);
  render();
}

function stopDemo() {
  if (demoTimer === null) return;
  clearInterval(demoTimer);
  demoTimer = null;
  resetPowerData();
  render();
}

function resetPowerData() {
  power = null;
  powerSensor = 'unknown';
  minVoltage = null;
  maxCurrentMa = null;
  currentChart.clear();
  voltageChart.clear();
}

document.querySelectorAll('[data-pfield="demo"]').forEach((b) => b.addEventListener('click', () => {
  if (demoTimer === null) startDemo();
  else stopDemo();
}));

document.querySelectorAll('[data-pfield="reset"]').forEach((b) => b.addEventListener('click', () => {
  minVoltage = power?.voltage ?? null;
  maxCurrentMa = power?.currentMa ?? null;
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
  render();
  if ('serial' in navigator) refreshPorts();
});

render();
