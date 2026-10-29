// Pinout data and rendering for the "Board pinout" tab (DOIT ESP32 DevKit V1 and NodeMCU-32S)
(() => {
  const { t, pick } = I18N;
  const STORAGE_KEY = 'esp32-dashboard-board';

  // Geometry shared by the SVG board and the label columns, so pins line up
  const ROW_H = 34;
  const TOP = 72;
  const BOTTOM = 76;
  const BOARD_W = 340;
  // Drawing units are scaled down on screen; labels use the same factor
  const SCALE = 0.85;

  // Chip colour class per capability; names live in i18n.js as cap.<key>
  const CAPS = {
    pwm: 'cap-pwm',
    adc1: 'cap-adc',
    adc2: 'cap-adc',
    touch: 'cap-touch',
    dac: 'cap-dac',
    uart: 'cap-comm',
    spi: 'cap-comm',
    i2c: 'cap-comm',
    input: 'cap-input',
    strap: 'cap-strap',
    flash: 'cap-strap',
    led: 'cap-led',
  };

  // Bilingual text helper for pin-specific notes
  const L = (en, tr) => ({ en, tr });

  const NOTES = {
    adc2: L('ADC2 cannot be used while Wi-Fi is on. Prefer ADC1 pins (32–39) for analog readings.',
            'Wi-Fi açıkken ADC2 kullanılamaz. Analog okumalar için ADC1 pinlerini (32–39) tercih edin.'),
    noPulls: L('No internal pull-up or pull-down. Add an external resistor if the signal can float.',
               'Dahili pull-up veya pull-down yok. Sinyal boşta kalabiliyorsa harici direnç ekleyin.'),
    encoder: L('Good for encoder inputs or analog sensors. Needs external pull-ups.',
               'Enkoder girişleri veya analog sensörler için iyi bir seçim. Harici pull-up gerekir.'),
    adc1Sense: L('ADC1 keeps working with Wi-Fi on, so this is a good pin for voltage or current sensing.',
                 'ADC1 Wi-Fi açıkken de çalışır, bu yüzden voltaj veya akım ölçümü için iyi bir pin.'),
    dac: L('One of only two true analog outputs (DAC).',
           'Gerçek analog çıkış (DAC) verebilen iki pinden biri.'),
    uart0: L('Wired to the USB-serial chip. The dashboard and firmware uploads use this line.',
             'USB-seri çipine bağlı. Dashboard ve firmware yükleme bu hattı kullanır.'),
    uart0Busy: L('Using it for anything else breaks the dashboard connection.',
                 'Başka bir iş için kullanmak dashboard bağlantısını bozar.'),
    i2cSensor: L('Use it for an INA219/INA226 current sensor later.',
                 'İleride INA219/INA226 akım sensörü için kullanılabilir.'),
    serial2: L('Free hardware serial port (Serial2). Useful for talking to another board.',
               'Boşta olan donanım seri portu (Serial2). Başka bir kartla haberleşmek için kullanışlı.'),
    flash: L('Wired to the onboard SPI flash chip. Using it crashes the board.',
             'Kart üzerindeki SPI flash çipine bağlı. Kullanmak kartı çökertir.'),
  };

  // Everything about a GPIO that does not depend on the board: [safety, chips, notes]
  const GPIO = {
    36: ['input', [['input', 'IN'], ['adc1', 'ADC1_0']], [NOTES.noPulls]],
    39: ['input', [['input', 'IN'], ['adc1', 'ADC1_3']], [NOTES.noPulls]],
    34: ['input', [['input', 'IN'], ['adc1', 'ADC1_6']], [NOTES.encoder]],
    35: ['input', [['input', 'IN'], ['adc1', 'ADC1_7']], [NOTES.encoder]],
    32: ['safe', [['pwm', 'PWM'], ['adc1', 'ADC1_4'], ['touch', 'T9']], [NOTES.adc1Sense]],
    33: ['safe', [['pwm', 'PWM'], ['adc1', 'ADC1_5'], ['touch', 'T8']], [NOTES.adc1Sense]],
    25: ['safe', [['pwm', 'PWM'], ['adc2', 'ADC2_8'], ['dac', 'DAC1']], [NOTES.dac]],
    26: ['safe', [['pwm', 'PWM'], ['adc2', 'ADC2_9'], ['dac', 'DAC2']], [NOTES.dac]],
    27: ['safe', [['pwm', 'PWM'], ['adc2', 'ADC2_7'], ['touch', 'T7']]],
    14: ['caution', [['pwm', 'PWM'], ['adc2', 'ADC2_6'], ['touch', 'T6'], ['spi', 'HSPI CLK']], [
      L('Outputs a short PWM signal at boot. Do not drive a motor directly from it without an enable gate.',
        'Açılışta kısa bir PWM sinyali verir. Bir enable kapısı olmadan doğrudan motor sürmeyin.'),
    ]],
    12: ['caution', [['pwm', 'PWM'], ['adc2', 'ADC2_5'], ['touch', 'T5'], ['spi', 'HSPI MISO'], ['strap', 'BOOT']], [
      L('Selects the flash voltage at boot. Must be LOW at power-up or the board may not boot. Avoid pull-ups here.',
        'Açılışta flash voltajını seçer. Güç verilirken LOW olmalı, yoksa kart açılmayabilir. Buraya pull-up koymayın.'),
    ]],
    13: ['safe', [['pwm', 'PWM'], ['adc2', 'ADC2_4'], ['touch', 'T4'], ['spi', 'HSPI MOSI']]],
    23: ['safe', [['pwm', 'PWM'], ['spi', 'VSPI MOSI']]],
    22: ['safe', [['pwm', 'PWM'], ['i2c', 'SCL']], [
      L('Default I²C clock.', 'Varsayılan I²C saat hattı.'), NOTES.i2cSensor,
    ]],
    1: ['avoid', [['uart', 'TX0']], [NOTES.uart0, NOTES.uart0Busy]],
    3: ['avoid', [['uart', 'RX0']], [NOTES.uart0, NOTES.uart0Busy]],
    21: ['safe', [['pwm', 'PWM'], ['i2c', 'SDA']], [
      L('Default I²C data.', 'Varsayılan I²C veri hattı.'), NOTES.i2cSensor,
    ]],
    19: ['safe', [['pwm', 'PWM'], ['spi', 'VSPI MISO']]],
    18: ['safe', [['pwm', 'PWM'], ['spi', 'VSPI CLK']]],
    5: ['caution', [['pwm', 'PWM'], ['spi', 'VSPI CS'], ['strap', 'BOOT']], [
      L('Strapping pin. Outputs a PWM signal at boot.',
        'Strapping pini. Açılışta PWM sinyali verir.'),
    ]],
    17: ['safe', [['pwm', 'PWM'], ['uart', 'TX2']], [NOTES.serial2]],
    16: ['safe', [['pwm', 'PWM'], ['uart', 'RX2']], [NOTES.serial2]],
    4: ['safe', [['pwm', 'PWM'], ['adc2', 'ADC2_0'], ['touch', 'T0']]],
    0: ['caution', [['pwm', 'PWM'], ['adc2', 'ADC2_1'], ['touch', 'T1'], ['strap', 'BOOT']], [
      L('Must be HIGH at boot for the firmware to run. The BOOT button pulls it LOW to enter upload mode.',
        'Firmware\'in çalışması için açılışta HIGH olmalı. BOOT butonu bunu LOW çekerek yükleme moduna sokar.'),
    ]],
    2: ['caution', [['pwm', 'PWM'], ['adc2', 'ADC2_2'], ['touch', 'T2'], ['led', 'LED'], ['strap', 'BOOT']], [
      L('Drives the onboard blue LED.', 'Kart üzerindeki mavi LED\'i sürer.'),
      L('Must be LOW or floating at boot to enter upload mode.',
        'Yükleme moduna girebilmek için açılışta LOW veya boşta olmalı.'),
    ]],
    15: ['caution', [['pwm', 'PWM'], ['adc2', 'ADC2_3'], ['touch', 'T3'], ['spi', 'HSPI CS'], ['strap', 'BOOT']], [
      L('Strapping pin. Outputs a PWM signal at boot. Pulling it LOW silences the boot log.',
        'Strapping pini. Açılışta PWM sinyali verir. LOW çekilirse açılış logu susar.'),
    ]],
    6: ['avoid', [['flash', 'FLASH CLK']], [NOTES.flash]],
    7: ['avoid', [['flash', 'FLASH D0']], [NOTES.flash]],
    8: ['avoid', [['flash', 'FLASH D1']], [NOTES.flash]],
    9: ['avoid', [['flash', 'FLASH D2']], [NOTES.flash]],
    10: ['avoid', [['flash', 'FLASH D3']], [NOTES.flash]],
    11: ['avoid', [['flash', 'FLASH CMD']], [NOTES.flash]],
  };

  function gpio(label, n) {
    const [safety, chips, notes = []] = GPIO[n];
    const caps = chips.map(([c]) => c);
    const allNotes = caps.includes('adc2') ? [...notes, NOTES.adc2] : notes;
    return { label, gpio: n, safety, chips, notes: allNotes };
  }

  function special(label, safety, title, volts, notes) {
    return { label, safety, chips: [], title, volts, notes };
  }

  const en = () => special('EN', 'ctrl', L('Enable / reset', 'Enable / reset'), L('3.3 V, pulled up', '3.3 V, pull-up\'lı'), [
    L('Pull LOW to reset the ESP32. Same as pressing the EN button.',
      'LOW çekilirse ESP32 resetlenir. EN butonuna basmakla aynı.'),
  ]);
  const gnd = () => special('GND', 'gnd', L('Ground', 'Toprak'), '0 V', []);
  const vin = (label) => special(label, 'power', L('Supply input', 'Besleme girişi'), '5 V', [
    L('Feeds the onboard 3.3 V regulator. Connected to USB 5 V through a diode.',
      'Kart üzerindeki 3.3 V regülatörünü besler. USB 5 V\'a bir diyot üzerinden bağlı.'),
    L('Use 5 V here when running without USB. Never connect motor supply voltage directly.',
      'USB olmadan çalıştırırken buraya 5 V verin. Motor besleme voltajını asla doğrudan bağlamayın.'),
  ]);
  const v33 = () => special('3V3', 'power', L('3.3 V output', '3.3 V çıkışı'), '3.3 V', [
    L('Output of the onboard regulator, shared with the ESP32 itself.',
      'Kart üzerindeki regülatörün çıkışı, ESP32\'nin kendisiyle ortak.'),
    L('Fine for small sensors. Do not power motors or servos from it.',
      'Küçük sensörler için uygun. Motor veya servoları buradan beslemeyin.'),
  ]);

  // Header pins top to bottom, viewed from the component side with USB at the bottom.
  // modRows: how many pin rows the WROOM module covers.
  const BOARDS = {
    nodemcu32s: {
      name: 'ESP32S NodeMCU-32S',
      silk: 'NODEMCU-32S',
      usbChip: 'CP2102 USB-SERIAL',
      modRows: 9.4,
      left: [
        v33(), en(), gpio('VP', 36), gpio('VN', 39), gpio('34', 34), gpio('35', 35), gpio('32', 32),
        gpio('33', 33), gpio('25', 25), gpio('26', 26), gpio('27', 27), gpio('14', 14), gpio('12', 12),
        gnd(), gpio('13', 13), gpio('SD2', 9), gpio('SD3', 10), gpio('CMD', 11), vin('5V'),
      ],
      right: [
        gnd(), gpio('23', 23), gpio('22', 22), gpio('TX', 1), gpio('RX', 3), gpio('21', 21), gnd(),
        gpio('19', 19), gpio('18', 18), gpio('5', 5), gpio('17', 17), gpio('16', 16), gpio('4', 4),
        gpio('0', 0), gpio('2', 2), gpio('15', 15), gpio('SD1', 8), gpio('SD0', 7), gpio('CLK', 6),
      ],
    },
    doit: {
      name: 'DOIT ESP32 DevKit V1',
      silk: 'DEVKIT V1 · DOIT',
      usbChip: 'CH340 USB-SERIAL',
      modRows: 7.4,
      left: [
        en(), gpio('VP', 36), gpio('VN', 39), gpio('D34', 34), gpio('D35', 35), gpio('D32', 32),
        gpio('D33', 33), gpio('D25', 25), gpio('D26', 26), gpio('D27', 27), gpio('D14', 14),
        gpio('D12', 12), gpio('D13', 13), gnd(), vin('VIN'),
      ],
      right: [
        gpio('D23', 23), gpio('D22', 22), gpio('TX0', 1), gpio('RX0', 3), gpio('D21', 21),
        gpio('D19', 19), gpio('D18', 18), gpio('D5', 5), gpio('TX2', 17), gpio('RX2', 16),
        gpio('D4', 4), gpio('D2', 2), gpio('D15', 15), gnd(), v33(),
      ],
    },
  };

  // Labels repeat (GND), so key each pin by its header position
  for (const b of Object.values(BOARDS)) {
    b.left.forEach((p, i) => { p.key = `L${i}`; });
    b.right.forEach((p, i) => { p.key = `R${i}`; });
    b.rows = Math.max(b.left.length, b.right.length);
    b.height = TOP + b.rows * ROW_H + BOTTOM;
  }

  let boardId = 'nodemcu32s';
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved in BOARDS) boardId = saved;
  } catch { /* storage unavailable */ }
  let board = BOARDS[boardId];
  let all = [];
  let byKey = {};

  const FILTERS = ['all', 'motor', 'pwm', 'adc', 'touch', 'dac', 'i2c', 'spi', 'uart', 'input', 'strap'];

  const has = (pin, cap) => pin.chips.some(([c]) => c === cap);

  function matches(pin, filter) {
    switch (filter) {
      case 'all':   return true;
      case 'motor': return pin.safety === 'safe' && has(pin, 'pwm');
      case 'adc':   return has(pin, 'adc1') || has(pin, 'adc2');
      default:      return has(pin, filter);
    }
  }

  let selectedKey = null;
  let activeFilter = 'all';

  const el = (tag, cls, text) => {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text !== undefined) node.textContent = text;
    return node;
  };

  // ---------- Board artwork ----------

  function boardSvg() {
    const parts = [];
    const cx = BOARD_W / 2;
    const BOARD_H = board.height;
    const modBottom = TOP + ROW_H * board.modRows;
    // Silkscreen and chips sit below the module, so they move down with it
    const y = (n) => n + (board.modRows - 7.4) * ROW_H;
    const modW = BOARD_W - 132;
    const modX = cx - modW / 2;

    parts.push(`<rect class="pcb" x="1" y="1" width="${BOARD_W - 2}" height="${BOARD_H - 2}" rx="12"/>`);

    // ESP-WROOM-32 module with PCB antenna and metal shield
    parts.push(`<rect class="mod-pcb" x="${modX}" y="8" width="${modW}" height="${modBottom - 8}" rx="3"/>`);
    const ax1 = modX + 12;
    const ax2 = modX + modW - 12;
    parts.push(`<path class="antenna" d="M${ax1} 16 H${ax2} V24 H${ax1 + 8} V32 H${ax2} V40 H${ax1 + 8} V48 H${ax2}"/>`);
    parts.push(`<rect class="shield" x="${modX + 7}" y="60" width="${modW - 14}" height="${modBottom - 68}" rx="2"/>`);
    const shieldMid = (60 + modBottom - 8) / 2;
    parts.push(`<text class="shield-sub" x="${cx}" y="${shieldMid - 20}" text-anchor="middle">ESPRESSIF</text>`);
    parts.push(`<text class="shield-text" x="${cx}" y="${shieldMid}" text-anchor="middle">ESP-WROOM-32</text>`);
    parts.push(`<text class="shield-sub" x="${cx}" y="${shieldMid + 18}" text-anchor="middle">Wi-Fi · BT · 240 MHz</text>`);

    // Silkscreen and support chips
    parts.push(`<text class="silk-big" x="${cx}" y="${y(358)}" text-anchor="middle">ESP32</text>`);
    parts.push(`<text class="silk-small" x="${cx}" y="${y(374)}" text-anchor="middle">${board.silk}</text>`);
    parts.push(`<rect class="ic" x="${cx - 32}" y="${y(396)}" width="64" height="38" rx="2"/>`);
    parts.push(`<text class="silk-small" x="${cx}" y="${y(450)}" text-anchor="middle">${board.usbChip}</text>`);
    parts.push(`<rect class="ic-tab" x="${cx - 22}" y="${y(466)}" width="44" height="6"/>`);
    parts.push(`<rect class="ic" x="${cx - 30}" y="${y(472)}" width="60" height="26" rx="1"/>`);
    parts.push(`<text class="silk-small" x="${cx}" y="${y(512)}" text-anchor="middle">AMS1117 3.3V</text>`);

    // LEDs
    parts.push(`<rect class="led-pwr" x="${cx - 52}" y="${y(532)}" width="10" height="6" rx="1"/>`);
    parts.push(`<text class="silk-small" x="${cx - 47}" y="${y(550)}" text-anchor="middle">PWR</text>`);
    parts.push(`<rect class="led-blue" x="${cx + 42}" y="${y(532)}" width="10" height="6" rx="1"/>`);
    parts.push(`<text class="silk-small" x="${cx + 47}" y="${y(550)}" text-anchor="middle">IO2</text>`);

    // Buttons and USB connector
    const by = BOARD_H - 62;
    for (const [x, name] of [[cx - 92, 'EN'], [cx + 62, 'BOOT']]) {
      parts.push(`<rect class="btn-base" x="${x}" y="${by}" width="30" height="24" rx="2"/>`);
      parts.push(`<circle class="btn-cap" cx="${x + 15}" cy="${by + 12}" r="7.5"/>`);
      parts.push(`<text class="silk-small" x="${x + 15}" y="${by + 37}" text-anchor="middle">${name}</text>`);
    }
    parts.push(`<rect class="usb" x="${cx - 24}" y="${BOARD_H - 34}" width="48" height="33" rx="3"/>`);
    parts.push(`<rect class="usb-hole" x="${cx - 15}" y="${BOARD_H - 14}" width="30" height="7" rx="2"/>`);

    // Header pins with silkscreen labels
    for (const side of ['left', 'right']) {
      board[side].forEach((pin, i) => {
        const cy = TOP + i * ROW_H + ROW_H / 2;
        const hx = side === 'left' ? 18 : BOARD_W - 18;
        const tx = side === 'left' ? 34 : BOARD_W - 34;
        const anchor = side === 'left' ? 'start' : 'end';
        parts.push(
          `<g class="hole" data-key="${pin.key}">` +
          `<circle class="hole-ring" cx="${hx}" cy="${cy}" r="10"/>` +
          `<circle class="hole-pad" cx="${hx}" cy="${cy}" r="7"/>` +
          `<circle class="hole-drill" cx="${hx}" cy="${cy}" r="3.2"/>` +
          '</g>'
        );
        parts.push(`<text class="silk" x="${tx}" y="${cy + 3.5}" text-anchor="${anchor}">${pin.label}</text>`);
      });
    }

    return `<svg viewBox="0 0 ${BOARD_W} ${BOARD_H}" width="${BOARD_W * SCALE}" height="${BOARD_H * SCALE}" role="img" aria-label="${board.name}, ${board.rows} header pins on each side">${parts.join('')}</svg>`;
  }

  // ---------- Label columns ----------

  function renderSide(side, container) {
    container.style.paddingTop = `${TOP * SCALE}px`;
    for (const pin of board[side]) {
      const row = el('button', `pin-row ${side}`);
      row.type = 'button';
      row.dataset.key = pin.key;
      row.style.height = `${ROW_H * SCALE}px`;
      row.title = pin.gpio !== undefined ? `${pin.label} · GPIO ${pin.gpio}` : pin.label;

      const name = el('span', `pin-name s-${pin.safety}`, pin.label);
      const chips = pin.chips.map(([cap, text]) => el('span', `chip ${CAPS[cap]}`, text));
      const lead = el('span', 'lead');
      row.append(...(side === 'left' ? [...chips, name, lead] : [lead, name, ...chips]));

      row.addEventListener('click', () => select(pin.key));
      row.addEventListener('mouseenter', () => setHover(pin.key, true));
      row.addEventListener('mouseleave', () => setHover(pin.key, false));
      container.append(row);
    }
  }

  function nodesFor(key) {
    return document.querySelectorAll(`[data-key="${key}"]`);
  }

  function setHover(key, on) {
    nodesFor(key).forEach((n) => n.classList.toggle('is-hover', on));
  }

  // ---------- Detail panel ----------

  function renderEmptyDetail() {
    document.getElementById('pinDetail').innerHTML =
      '<div class="detail-empty"><div>' +
      '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"><path d="M9 3h6M9 21h6M3 9v6M21 9v6"/><rect x="7" y="7" width="10" height="10" rx="1.5"/></svg>' +
      `<p>${t('emptyDetail')}</p>` +
      '</div></div>';
  }

  function renderDetail() {
    const pin = byKey[selectedKey];
    const panel = document.getElementById('pinDetail');
    const onLeft = board.left.includes(pin);
    const position = (onLeft ? board.left : board.right).indexOf(pin) + 1;
    const isGpio = pin.gpio !== undefined;
    const inputOnly = has(pin, 'input');

    const rows = isGpio
      ? [
          [t('voltage'), t('gpioVoltage')],
          [t('direction'), inputOnly ? t('dirIn') : t('dirIo')],
          [t('current'), inputOnly ? '—' : t('currentIo')],
          [t('pulls'), inputOnly ? t('pullNone') : t('pullInternal')],
        ]
      : [[t('voltage'), pick(pin.volts)]];

    panel.innerHTML = '';

    const head = el('div', 'detail-head');
    head.append(el('span', `pin-name s-${pin.safety}`, pin.label));
    const titles = el('div');
    titles.append(el('div', 'detail-title', isGpio ? `GPIO ${pin.gpio}` : pick(pin.title)));
    titles.append(el('div', 'detail-sub', t('position', { side: t(onLeft ? 'left' : 'right'), n: position })));
    head.append(titles);

    const verdict = el('div', `verdict s-${pin.safety}`);
    verdict.append(el('strong', '', t(`safety.${pin.safety}`)), el('span', '', t(`safety.${pin.safety}.desc`)));

    const dl = el('dl', 'details');
    for (const [k, v] of rows) {
      const row = el('div');
      row.append(el('dt', '', k), el('dd', '', v));
      dl.append(row);
    }

    panel.append(head, verdict, dl);

    if (pin.chips.length) {
      const list = el('ul', 'fn-list');
      for (const [cap, text] of pin.chips) {
        const li = el('li');
        li.append(el('span', `chip ${CAPS[cap]}`, text), document.createTextNode(t(`cap.${cap}`)));
        list.append(li);
      }
      panel.append(el('h3', '', t('functions')), list);
    }

    if (pin.notes.length) {
      const notes = el('ul', 'notes');
      for (const note of pin.notes) notes.append(el('li', '', pick(note)));
      panel.append(el('h3', '', t('notes')), notes);
    }
  }

  function select(key) {
    document.querySelectorAll('.is-selected').forEach((n) => n.classList.remove('is-selected'));
    selectedKey = key;
    nodesFor(key).forEach((n) => n.classList.add('is-selected'));
    renderDetail();
  }

  // ---------- Filters and legend ----------

  function applyFilter() {
    for (const pin of all) {
      const dim = !matches(pin, activeFilter);
      nodesFor(pin.key).forEach((n) => n.classList.toggle('is-dim', dim));
    }
    document.querySelectorAll('.filter').forEach((b) => {
      b.setAttribute('aria-pressed', String(b.dataset.filter === activeFilter));
    });
  }

  function renderFilters() {
    const box = document.getElementById('pinFilters');
    box.innerHTML = '';
    for (const id of FILTERS) {
      const b = el('button', 'filter', t(`filter.${id}`));
      b.type = 'button';
      b.dataset.filter = id;
      b.addEventListener('click', () => { activeFilter = id; applyFilter(); });
      box.append(b);
    }
  }

  function renderLegend() {
    const box = document.getElementById('pinLegend');
    box.innerHTML = '';
    const items = [
      ['safe', 'var(--online)'], ['caution', 'var(--waiting)'], ['input', 'var(--input)'],
      ['avoid', 'var(--avoid)'], ['power', 'var(--power)'], ['gnd', 'var(--gnd)'],
    ];
    for (const [key, color] of items) {
      const item = el('span');
      const swatch = el('i');
      swatch.style.background = color;
      item.append(swatch, document.createTextNode(t(`safety.${key}`)));
      box.append(item);
    }
  }

  // ---------- Board picker ----------

  function renderBoardTabs() {
    const box = document.getElementById('boardTabs');
    box.innerHTML = '';
    for (const [id, b] of Object.entries(BOARDS)) {
      const tab = el('button', 'board-tab', b.name);
      tab.type = 'button';
      tab.setAttribute('role', 'tab');
      tab.setAttribute('aria-selected', String(id === boardId));
      tab.addEventListener('click', () => showBoard(id));
      box.append(tab);
    }
  }

  function showBoard(id) {
    boardId = id;
    board = BOARDS[id];
    try { localStorage.setItem(STORAGE_KEY, id); } catch { /* storage unavailable */ }
    all = [...board.left, ...board.right];
    byKey = Object.fromEntries(all.map((p) => [p.key, p]));
    selectedKey = null;

    renderBoardTabs();
    renderTitle();
    document.getElementById('boardArt').innerHTML = boardSvg();
    for (const [side, id2] of [['left', 'pinsLeft'], ['right', 'pinsRight']]) {
      const col = document.getElementById(id2);
      col.innerHTML = '';
      renderSide(side, col);
    }
    document.querySelectorAll('.hole').forEach((h) => {
      const key = h.dataset.key;
      h.addEventListener('click', () => select(key));
      h.addEventListener('mouseenter', () => setHover(key, true));
      h.addEventListener('mouseleave', () => setHover(key, false));
    });
    applyFilter();
    renderEmptyDetail();
  }

  function renderTitle() {
    const count = board.left.length + board.right.length;
    document.getElementById('boardTitle').textContent = `${board.name} · ${t('boardPins', { n: count })}`;
  }

  // ---------- Init ----------

  renderFilters();
  renderLegend();
  showBoard(boardId);

  I18N.onChange(() => {
    renderTitle();
    renderFilters();
    renderLegend();
    applyFilter();
    if (selectedKey) renderDetail();
    else renderEmptyDetail();
  });
})();
