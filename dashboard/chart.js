// Small live line chart on a canvas: one series, the last windowMs on the x axis,
// auto-scaled y axis, crosshair + tooltip on hover. Colors come from the CSS tokens,
// so it follows the light/dark theme.
class LiveChart {
  constructor(container, { format, tickFormat = format, minSpan, windowMs = 60000, labels }) {
    this.container = container;
    this.format = format;
    this.tickFormat = tickFormat;
    this.minSpan = minSpan;     // smallest y range, so a flat signal is not blown up into noise
    this.windowMs = windowMs;
    this.labels = labels;       // { empty(), axis(seconds), ago(seconds) }
    this.points = [];
    this.hoverX = null;

    this.canvas = document.createElement('canvas');
    this.tip = document.createElement('div');
    this.tip.className = 'chart-tip';
    this.tip.hidden = true;
    container.append(this.canvas, this.tip);

    this.canvas.addEventListener('mousemove', (e) => { this.hoverX = e.offsetX; this.draw(); });
    this.canvas.addEventListener('mouseleave', () => { this.hoverX = null; this.draw(); });
    new ResizeObserver(() => this.draw()).observe(container);
  }

  push(value) {
    const now = performance.now();
    this.points.push({ t: now, v: value });
    while (this.points.length && this.points[0].t < now - this.windowMs) this.points.shift();
  }

  clear() {
    this.points = [];
    this.draw();
  }

  draw() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (!w || !h) return;   // hidden tab

    const css = getComputedStyle(document.documentElement);
    const token = (name) => css.getPropertyValue(name).trim();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = w * dpr;
    this.canvas.height = h * dpr;
    this.canvas.style.width = `${w}px`;
    this.canvas.style.height = `${h}px`;

    const ctx = this.canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.font = '11px system-ui, sans-serif';

    const pad = { left: 64, right: 14, top: 10, bottom: 24 };
    const pw = w - pad.left - pad.right;
    const ph = h - pad.top - pad.bottom;
    const now = performance.now();
    const x = (t) => pad.left + (1 - (now - t) / this.windowMs) * pw;

    // Time axis: -60 s ... now
    ctx.fillStyle = token('--muted');
    ctx.textBaseline = 'alphabetic';
    const seconds = this.windowMs / 1000;
    for (const s of [seconds, seconds / 2, 0]) {
      const px = pad.left + (1 - s / seconds) * pw;
      ctx.textAlign = s === seconds ? 'left' : s === 0 ? 'right' : 'center';
      ctx.fillText(this.labels.axis(s), px, h - 6);
    }

    if (this.points.length === 0) {
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(this.labels.empty(), pad.left + pw / 2, pad.top + ph / 2);
      this.tip.hidden = true;
      return;
    }

    // Y scale with round tick steps
    const values = this.points.map((p) => p.v);
    let lo = Math.min(...values);
    let hi = Math.max(...values);
    if (hi - lo < this.minSpan) {
      const mid = (hi + lo) / 2;
      lo = mid - this.minSpan / 2;
      hi = mid + this.minSpan / 2;
    }
    const step = niceStep((hi - lo) / 3);
    lo = Math.floor(lo / step) * step;
    hi = Math.ceil(hi / step) * step;
    const y = (v) => pad.top + (1 - (v - lo) / (hi - lo)) * ph;

    ctx.strokeStyle = token('--border');
    ctx.lineWidth = 1;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let v = lo; v <= hi + step / 2; v += step) {
      const py = Math.round(y(v)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(w - pad.right, py);
      ctx.stroke();
      ctx.fillText(this.tickFormat(v), pad.left - 8, py);
    }

    // Series line
    const accent = token('--accent');
    ctx.strokeStyle = accent;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    this.points.forEach((p, i) => (i ? ctx.lineTo(x(p.t), y(p.v)) : ctx.moveTo(x(p.t), y(p.v))));
    ctx.stroke();

    // Latest value, or the hovered one with a crosshair
    let focus = this.points[this.points.length - 1];
    if (this.hoverX !== null) {
      focus = this.points.reduce((best, p) =>
        Math.abs(x(p.t) - this.hoverX) < Math.abs(x(best.t) - this.hoverX) ? p : best);
      ctx.strokeStyle = token('--muted');
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(Math.round(x(focus.t)) + 0.5, pad.top);
      ctx.lineTo(Math.round(x(focus.t)) + 0.5, pad.top + ph);
      ctx.stroke();
    }
    ctx.fillStyle = accent;
    ctx.strokeStyle = token('--surface');
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x(focus.t), y(focus.v), 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    if (this.hoverX === null) {
      this.tip.hidden = true;
      return;
    }
    const ago = Math.round((now - focus.t) / 1000);
    this.tip.textContent = `${this.format(focus.v)} · ${this.labels.ago(ago)}`;
    this.tip.hidden = false;
    const tipX = Math.min(Math.max(x(focus.t) + 10, 0), w - this.tip.offsetWidth);
    this.tip.style.left = `${tipX}px`;
    this.tip.style.top = `${Math.max(y(focus.v) - 34, 0)}px`;
  }
}

function niceStep(raw) {
  const pow = 10 ** Math.floor(Math.log10(raw));
  const n = raw / pow;
  return (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10) * pow;
}
