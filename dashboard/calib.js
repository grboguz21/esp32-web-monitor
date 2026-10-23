// Calibration math, kept apart from the page so it can be checked against recorded data.
// Same method as tools/kv_sweep.py and tools/step_test.py: a steady-state sweep gives the
// feedforward curve, step responses give a first-order-plus-dead-time model, SIMC gives PI.
const Calib = (() => {
  const FF_MAX_POINTS = 16;        // firmware limit per direction
  const STOPPED_RPM = 5;

  // Sweep points for one direction, highest duty first: [{ duty, rpm }] with rpm >= 0.
  // Returns [[rpm, duty], ...] rising, starting at [0, stop threshold]; the stop threshold
  // sits halfway between the last duty that turned and the first that did not.
  function buildFfTable(points) {
    const moving = points.filter((p) => p.rpm >= STOPPED_RPM);
    const stopped = points.filter((p) => p.rpm < STOPPED_RPM);
    if (moving.length < 2) return null;
    const lowest = moving.reduce((a, b) => (b.duty < a.duty ? b : a));
    const stopDuty = stopped.length
      ? (lowest.duty + Math.max(...stopped.map((p) => p.duty))) / 2
      : lowest.duty;
    const rows = [[0, stopDuty]];
    for (const p of [...moving].sort((a, b) => a.rpm - b.rpm)) {
      const last = rows[rows.length - 1];
      if (p.rpm > last[0] + 1 && p.duty > last[1]) rows.push([p.rpm, p.duty]);   // keep it rising
    }
    // Too many points: drop the most closely spaced ones (never the first or last)
    while (rows.length > FF_MAX_POINTS) {
      let worst = 1;
      for (let i = 2; i < rows.length - 1; i++) {
        if (rows[i + 1][0] - rows[i - 1][0] < rows[worst + 1][0] - rows[worst - 1][0]) worst = i;
      }
      rows.splice(worst, 1);
    }
    return rows;
  }

  // Duty that gives `rpm` on a table, by linear interpolation
  function dutyFor(table, rpm) {
    if (rpm <= table[0][0]) return table[0][1];
    for (let i = 1; i < table.length; i++) {
      if (rpm <= table[i][0]) {
        const f = (rpm - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
        return table[i - 1][1] + f * (table[i][1] - table[i - 1][1]);
      }
    }
    return table[table.length - 1][1];
  }

  // samples: [{ t (s, step at 0), rpm }] after the step; fits
  // w(t) = w0 + (w1 - w0)(1 - exp(-(t - theta) / tau)) by grid search
  function fitFirstOrder(samples, w0, w1) {
    let best = { err: Infinity, theta: 0, tau: 0 };
    for (let thetaMs = 0; thetaMs <= 40; thetaMs++) {
      const theta = thetaMs / 1000;
      for (let tauMs = 5; tauMs <= 600; tauMs++) {
        const tau = tauMs / 1000;
        let err = 0;
        for (const s of samples) {
          const model = s.t <= theta ? w0 : w0 + (w1 - w0) * (1 - Math.exp(-(s.t - theta) / tau));
          err += (s.rpm - model) ** 2;
          if (err >= best.err) break;
        }
        if (err < best.err) best = { err, theta, tau };
      }
    }
    return { theta: best.theta, tau: best.tau, rms: Math.sqrt(best.err / samples.length) };
  }

  // One logged step: [{ t (s), duty, rpm }] at 200 Hz, duty jumping from a to b.
  // volts: supply during the test. Returns { tau, theta, k (RPM/V), w0, w1 } or null.
  function analyseStep(log, a, b, volts) {
    const abs = log.map((r) => ({ t: r.t, duty: Math.abs(r.duty), rpm: Math.abs(r.rpm) }));
    const i = abs.findIndex((r) => Math.abs(r.duty - Math.abs(b)) < 0.05);
    if (i < 20 || abs.length - i < 100) return null;
    const t0 = abs[i].t;
    const before = abs.slice(Math.max(0, i - 50), i).map((r) => r.rpm);
    const after = abs.slice(i).filter((r) => r.t - t0 <= 1.2).map((r) => ({ t: r.t - t0, rpm: r.rpm }));
    const tail = after.filter((r) => r.t > after[after.length - 1].t - 0.4).map((r) => r.rpm);
    const mean = (v) => v.reduce((x, y) => x + y, 0) / v.length;
    const w0 = mean(before);
    const w1 = mean(tail);
    const fit = fitFirstOrder(after, w0, w1);
    const dv = ((Math.abs(b) - Math.abs(a)) / 100) * volts;
    return { ...fit, k: (w1 - w0) / dv, w0, w1 };
  }

  // SIMC PI for a first-order-plus-dead-time model; tau, theta, tauC in s, k in RPM/V.
  // Gains in the firmware's units: kp V/RPM, ki V/(RPM s).
  function simc(tau, k, theta, tauC) {
    const kp = tau / (k * (tauC + theta));
    const ti = Math.min(tau, 4 * (tauC + theta));
    return { kp, ki: kp / ti };
  }

  // The three settings offered after a run: closed loop as slow as the motor, in between,
  // and as fast as the delay allows (theta already includes the speed estimate's delay)
  function presets(tau, k, theta) {
    return {
      calm: simc(tau, k, theta, tau),
      medium: simc(tau, k, theta, Math.max(theta, tau / 2.5)),
      aggressive: simc(tau, k, theta, theta),
    };
  }

  return { FF_MAX_POINTS, STOPPED_RPM, buildFfTable, dutyFor, fitFirstOrder, analyseStep, simc, presets };
})();

if (typeof module !== 'undefined') module.exports = Calib;
