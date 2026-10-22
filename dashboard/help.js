// Info buttons with a "simple + technical" tooltip. Any element with data-help="key"
// gets an (i) button; texts come from i18n.js as help.<key> and help.<key>.tech.
(() => {
  const { t } = I18N;

  const ICON = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 11v5M12 8h.01"/></svg>';
  const GAP = 8;

  const tip = document.createElement('div');
  tip.className = 'tooltip';
  tip.id = 'helpTooltip';
  tip.setAttribute('role', 'tooltip');
  tip.hidden = true;
  document.body.append(tip);

  let anchor = null;    // button the tooltip is attached to
  let pinned = false;   // opened by click: stays until clicked elsewhere

  function fill(key) {
    tip.innerHTML = '';
    const simpleLabel = document.createElement('div');
    simpleLabel.className = 'tip-label';
    simpleLabel.textContent = t('helpSimple');
    const simple = document.createElement('p');
    simple.className = 'tip-simple';
    simple.textContent = t(`help.${key}`);
    const techLabel = document.createElement('div');
    techLabel.className = 'tip-label';
    techLabel.textContent = t('helpTech');
    const tech = document.createElement('p');
    tech.className = 'tip-tech';
    tech.textContent = t(`help.${key}.tech`);
    tip.append(simpleLabel, simple, techLabel, tech);
  }

  function position(btn) {
    const r = btn.getBoundingClientRect();
    const w = tip.offsetWidth;
    const h = tip.offsetHeight;
    let left = r.left + r.width / 2 - w / 2;
    left = Math.max(GAP, Math.min(left, window.innerWidth - w - GAP));
    let top = r.bottom + GAP;
    if (top + h > window.innerHeight - GAP) top = Math.max(GAP, r.top - h - GAP);
    tip.style.left = `${left}px`;
    tip.style.top = `${top}px`;
  }

  function show(btn) {
    anchor = btn;
    fill(btn.dataset.helpKey);
    tip.hidden = false;
    btn.setAttribute('aria-describedby', tip.id);
    position(btn);
  }

  function hide() {
    if (anchor) anchor.removeAttribute('aria-describedby');
    anchor = null;
    pinned = false;
    tip.hidden = true;
  }

  document.querySelectorAll('[data-help]').forEach((target) => {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'info-btn';
    btn.dataset.helpKey = target.dataset.help;
    btn.innerHTML = ICON;
    btn.setAttribute('aria-label', t('helpButton'));
    target.append(btn);

    btn.addEventListener('mouseenter', () => { if (!pinned) show(btn); });
    btn.addEventListener('mouseleave', () => { if (!pinned) hide(); });
    btn.addEventListener('focus', () => { if (!pinned) show(btn); });
    btn.addEventListener('blur', () => { if (!pinned) hide(); });
    btn.addEventListener('click', (e) => {
      // Keep the click from reaching labels or rows behind the button
      e.preventDefault();
      e.stopPropagation();
      if (pinned && anchor === btn) { hide(); return; }
      show(btn);
      pinned = true;
    });
  });

  document.addEventListener('click', (e) => {
    if (pinned && !tip.contains(e.target)) hide();
  });
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape') hide(); });
  window.addEventListener('scroll', () => { if (anchor) position(anchor); }, true);
  window.addEventListener('resize', () => { if (anchor) position(anchor); });

  I18N.onChange(() => {
    document.querySelectorAll('.info-btn').forEach((b) => b.setAttribute('aria-label', t('helpButton')));
    if (anchor) { fill(anchor.dataset.helpKey); position(anchor); }
  });
})();
