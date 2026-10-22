// Small live line chart on a canvas: one or more series sharing one y axis, the last
// windowMs on the x axis, redrawn every frame so time scrolls smoothly. Hover shows a
// crosshair and every series' value at that moment. Colors come from CSS tokens, so it
// follows the light/dark theme.
class LiveChart {
  constructor(container, { series, legend, format, tickFormat = format, minSpan, windowMs = 60000, labels }) {
    this.container = container;
    this.series = series;       // [{ key, label, short, color: '--series-1' }]
    this.format = format;
    this.tickFormat = tickFormat;
    this.minSpan = minSpan;     // smallest y range, so a flat signal is not blown up into noise
    this.windowMs = windowMs;
    this.labels = labels;       // { empty(), axis(seconds), ago(seconds) }
    this.data = new Map(series.map((s) => [s.key, []]));
    this.hoverX = null;

    this.canvas = document.createElement('canvas');
    this.tip = document.createElement('div');
    this.tip.className = 'chart-tip';
    this.tip.hidden = true;
    container.append(this.canvas, this.tip);
    this.legend = legend;
    this.renderLegend();

    this.canvas.addEventListener('mousemove', (e) => { this.hoverX = e.offsetX; });
    this.canvas.addEventListener('mouseleave', () => { this.hoverX = null; });

    const frame = () => {
      this.draw();
      requestAnimationFrame(frame);
    };
    requestAnimationFrame(frame);
  }

  renderLegend() {
    if (!this.legend) return;
    this.legend.innerHTML = '';
    for (const s of this.series) {
      const item = document.createElement('span');
      const swatch = document.createElement('i');
      swatch.style.background = `var(${s.color})`;
      item.append(swatch, document.createTextNode(s.label()));
      this.legend.append(item);
    }
  }

  push(key, value) {
    const now = performance.now();
    const points = this.data.get(key);
    points.push({ t: now, v: value });
    while (points.length && points[0].t < now - this.windowMs) points.shift();
  }

  clear(key) {
    for (const [k, points] of this.data) if (!key || k === key) points.length = 0;
  }

  draw() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (!w || !h) return;   // hidden tab

    const css = getComputedStyle(document.documentElement);
    const token = (name) => css.getPropertyValue(name).trim();
    const dpr = window.devicePixelRatio || 1;
    if (this.canvas.width !== w * dpr || this.canvas.height !== h * dpr) {
      this.canvas.width = w * dpr;
      this.canvas.height = h * dpr;
      this.canvas.style.width = `${w}px`;
      this.canvas.style.height = `${h}px`;
    }

    const ctx = this.canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.font = '11px system-ui, sans-serif';

    const pad = { left: 64, right: 34, top: 10, bottom: 24 };
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

    const active = this.series.filter((s) => this.data.get(s.key).length > 0);
    if (active.length === 0) {
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(this.labels.empty(), pad.left + pw / 2, pad.top + ph / 2);
      this.tip.hidden = true;
      return;
    }

    // Shared y scale with round tick steps
    const values = active.flatMap((s) => this.data.get(s.key).map((p) => p.v));
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

    // Lines, clipped to the plot so the oldest point slides out cleanly
    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.left, 0, pw, h);
    ctx.clip();
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    for (const s of active) {
      ctx.strokeStyle = token(s.color);
      ctx.beginPath();
      this.data.get(s.key).forEach((p, i) => (i ? ctx.lineTo(x(p.t), y(p.v)) : ctx.moveTo(x(p.t), y(p.v))));
      ctx.stroke();
    }
    ctx.restore();

    // Focus: the hovered moment (with crosshair) or the latest values
    const nearest = (points, px) => points.reduce((best, p) =>
      Math.abs(x(p.t) - px) < Math.abs(x(best.t) - px) ? p : best);
    const hovering = this.hoverX !== null && this.hoverX >= pad.left;
    const focus = active.map((s) => {
      const points = this.data.get(s.key);
      return { s, p: hovering ? nearest(points, this.hoverX) : points[points.length - 1] };
    });

    if (hovering) {
      ctx.strokeStyle = token('--muted');
      ctx.lineWidth = 1;
      const cx = Math.round(Math.min(Math.max(this.hoverX, pad.left), pad.left + pw)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(cx, pad.top);
      ctx.lineTo(cx, pad.top + ph);
      ctx.stroke();
    }
    for (const { s, p } of focus) {
      ctx.fillStyle = token(s.color);
      ctx.strokeStyle = token('--surface');
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(x(p.t), y(p.v), 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }

    // Direct labels at the line ends, nudged apart when they would overlap
    if (this.series.length > 1) {
      const ends = active.map((s) => {
        const points = this.data.get(s.key);
        const last = points[points.length - 1];
        return { s, y: y(last.v), x: x(last.t) };
      }).sort((a, b) => a.y - b.y);
      for (let i = 1; i < ends.length; i++) ends[i].y = Math.max(ends[i].y, ends[i - 1].y + 13);
      ctx.fillStyle = token('--text');
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      for (const e of ends) ctx.fillText(e.s.short, Math.min(e.x, pad.left + pw) + 8, e.y);
    }

    if (!hovering) {
      this.tip.hidden = true;
      return;
    }
    const ago = Math.round((now - focus[0].p.t) / 1000);
    this.tip.innerHTML = '';
    const time = document.createElement('div');
    time.className = 'chart-tip-time';
    time.textContent = this.labels.ago(ago);
    this.tip.append(time);
    for (const { s, p } of focus) {
      const row = document.createElement('div');
      const swatch = document.createElement('i');
      swatch.style.background = `var(${s.color})`;
      row.append(swatch, document.createTextNode(`${s.label()}  ${this.format(p.v)}`));
      this.tip.append(row);
    }
    this.tip.hidden = false;
    const tipX = this.hoverX + 12 + this.tip.offsetWidth > w ? this.hoverX - 12 - this.tip.offsetWidth : this.hoverX + 12;
    this.tip.style.left = `${Math.max(tipX, 0)}px`;
    this.tip.style.top = `${pad.top}px`;
  }
}

function niceStep(raw) {
  const pow = 10 ** Math.floor(Math.log10(raw));
  const n = raw / pow;
  return (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10) * pow;
}
