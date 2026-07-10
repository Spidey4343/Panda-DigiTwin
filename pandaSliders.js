/**
 * PandaSliderControl — Franka Research 3 (FR3) 7-DOF
 * All joints rotate around Z-axis in their local frame
 * Exact limits from joint_limits.yaml
 */
(function () {
  'use strict';

  const DEG = Math.PI / 180;
  const RAD = 180 / Math.PI;

  const JOINT_CONFIG = [
    { name: 'joint1', label: 'J1', min: -166, max:  166, init:   0, color: '#e63946' },
    { name: 'joint2', label: 'J2', min: -105, max:  105, init: -45, color: '#f4a261' },
    { name: 'joint3', label: 'J3', min: -166, max:  166, init:   0, color: '#2a9d8f' },
    { name: 'joint4', label: 'J4', min: -176, max:   -7, init:-135, color: '#457b9d' },
    { name: 'joint5', label: 'J5', min: -165, max:  165, init:   0, color: '#a8dadc' },
    { name: 'joint6', label: 'J6', min:   25, max:  265, init:  90, color: '#e9c46a' },
    { name: 'joint7', label: 'J7', min: -175, max:  175, init:  45, color: '#c77dff' },
  ];

  const _namedConfigs = {};
  const _instances    = {};

  // ── Safety gate ─────────────────────────────────────────────
  // Builds the full 7-joint radian array (current angles + any overrides),
  // runs it through PandaSafety, and returns the result. Fails open (safe)
  // if the safety module isn't loaded, so a missing script never bricks
  // normal operation.
  function checkTargetSafety(overrides) {
    if (typeof window.PandaSafety === 'undefined') return { safe: true };
    const rad = JOINT_CONFIG.map(cfg => {
      const deg = (overrides && overrides[cfg.name] != null)
        ? overrides[cfg.name]
        : (_instances[cfg.name] ? _instances[cfg.name].getValue() : cfg.init);
      return deg * DEG;
    });
    return window.PandaSafety.checkSafety(rad);
  }

  function warnUnsafe(result) {
    const reason = result.groundViolation ? 'would go below ground' : 'self-collision risk';
    const detail = result.details && result.details[0] ? ' — ' + result.details[0] : '';
    const msg = `Blocked: ${reason}${detail}`;
    if (typeof window.setStatus === 'function') window.setStatus('⛔ ' + msg, 'err');
    if (typeof window.termLog === 'function') window.termLog(msg, 'err');
    console.warn('[FR3 Safety] ' + msg);
  }

  // ── JointSlider class ────────────────────────────────────────
  class JointSlider {
    constructor(cfg, parentId) {
      this.cfg   = cfg;
      this.value = cfg.init;
      this.node  = null;
      this.Rot   = null;
      this._buildDOM(parentId);
      this._initX3D();
    }

    getValue() { return this.value; }

    _buildDOM(parentId) {
      const parent = document.getElementById(parentId);
      if (!parent) return;

      const wrap = document.createElement('div');
      wrap.className = 'joint-slider-wrap';

      wrap.innerHTML = `
        <div class="joint-header">
          <span class="joint-dot" style="background:${this.cfg.color}"></span>
          <span class="joint-label">${this.cfg.label}</span>
          <span class="joint-value" id="val_${this.cfg.name}">${this.cfg.init.toFixed(1)}°</span>
          <span class="joint-limits">${this.cfg.min}° / ${this.cfg.max}°</span>
        </div>
        <div class="joint-track">
          <input type="range"
            id="slider_${this.cfg.name}"
            min="${this.cfg.min}" max="${this.cfg.max}"
            value="${this.cfg.init}" step="0.5"
            style="--accent:${this.cfg.color}"/>
          <div class="joint-bar" id="bar_${this.cfg.name}"
            style="background:${this.cfg.color}"></div>
        </div>`;

      parent.appendChild(wrap);
      this.sliderEl = wrap.querySelector(`#slider_${this.cfg.name}`);
      this.valueEl  = wrap.querySelector(`#val_${this.cfg.name}`);
      this.barEl    = wrap.querySelector(`#bar_${this.cfg.name}`);

      this.sliderEl.addEventListener('input', e => {
        const newVal = parseFloat(e.target.value);
        const check = checkTargetSafety({ [this.cfg.name]: newVal });
        if (!check.safe) {
          warnUnsafe(check);
          e.target.value = this.value; // snap the handle back to the last safe angle
          return;
        }
        this._rotate(newVal);
      });
      this._updateBar();
    }

    _initX3D() {
      // Poll directly for the browser + named node instead of relying on
      // addBrowserCallback's INITIALIZED event (its 3-arg signature was
      // being called with 2 args, so the callback never actually fired
      // and this.node was staying null forever — sliders updated the UI
      // but never touched the live 3D model).
      const attempt = (tries) => {
        if (tries <= 0) {
          console.warn(`[FR3] ${this.cfg.name} failed to connect to 3D scene`);
          return;
        }
        try {
          const browser = (typeof X3D !== 'undefined') &&
            (X3D.getBrowser('#x3dCanvas') || X3D.getBrowser());
          const scene = browser && browser.currentScene;
          const node  = scene && scene.getNamedNode(this.cfg.name);

          if (!node) { setTimeout(() => attempt(tries - 1), 400); return; }

          this.node = node;
          // All joints rotate around Z in local frame
          this.Rot = new X3D.SFRotation(0, 0, 1, this.cfg.init * DEG);
          this.node.rotation = this.Rot;

          // Bidirectional: scene drag → slider
          try {
            this.node.rotation.addFieldCallback('sync_' + this.cfg.name, val => {
              this._syncFromScene(val[3] * RAD);
            });
          } catch(e) { /* field callback not always available */ }

          console.log(`[FR3] ${this.cfg.name} connected`);
        } catch(e) {
          setTimeout(() => attempt(tries - 1), 400);
        }
      };
      setTimeout(() => attempt(30), 400);
    }

    _rotate(deg) {
      this.value = Math.max(this.cfg.min, Math.min(this.cfg.max, deg));
      if (this.valueEl) this.valueEl.textContent = this.value.toFixed(1) + '°';
      this._updateBar();
      if (this.node && this.Rot) {
        this.Rot[3] = this.value * DEG;
        this.node.rotation = this.Rot;
      }
      const idx = JOINT_CONFIG.findIndex(c => c.name === this.cfg.name);
      if (window._pandaIKSliderCallback) window._pandaIKSliderCallback(idx, this.value);
    }

    _syncFromScene(deg) {
      const v = Math.max(this.cfg.min, Math.min(this.cfg.max, deg));
      this.value = v;
      if (this.sliderEl) this.sliderEl.value = v;
      if (this.valueEl)  this.valueEl.textContent = v.toFixed(1) + '°';
      this._updateBar();
    }

    _updateBar() {
      if (!this.barEl) return;
      const pct = ((this.value - this.cfg.min) / (this.cfg.max - this.cfg.min)) * 100;
      this.barEl.style.width = pct.toFixed(1) + '%';
    }

    setAngle(deg, animate) {
      if (animate) {
        this._animateTo(deg);
      } else {
        const clamped = Math.max(this.cfg.min, Math.min(this.cfg.max, deg));
        this._rotate(clamped);
        if (this.sliderEl) this.sliderEl.value = clamped;
      }
    }

    _animateTo(targetDeg) {
      const start = this.value;
      const end   = Math.max(this.cfg.min, Math.min(this.cfg.max, targetDeg));
      const dur   = 700;
      const t0    = performance.now();
      const tick  = now => {
        const t = Math.min(1, (now - t0) / dur);
        const s = t*t*t*(6*t*t - 15*t + 10);
        const v = start + s*(end - start);
        this._rotate(v);
        if (this.sliderEl) this.sliderEl.value = v;
        if (t < 1) requestAnimationFrame(tick);
      };
      requestAnimationFrame(tick);
    }
  }

  // ── Static API ───────────────────────────────────────────────

  function init(parentId) {
    JOINT_CONFIG.forEach(cfg => {
      _instances[cfg.name] = new JointSlider(cfg, parentId);
    });
    if (window.IKController) window.IKController.setSliderInstances(_instances);
  }

  function loadAllAngles(map, animate) {
    // Only gate discrete "go to this target" calls (animate=true — PTP, HOME,
    // stored-position loads). LIN's own trajectory player calls this once per
    // frame with animate=false; its target was already checked up front by
    // IKController.solve(), and re-checking every frame risks the arm
    // stalling mid-motion on a borderline intermediate frame.
    if (animate) {
      const check = checkTargetSafety(map);
      if (!check.safe) { warnUnsafe(check); return false; }
    }
    Object.entries(map).forEach(([name, deg]) => {
      if (_instances[name]) _instances[name].setAngle(deg, animate);
    });
    return true;
  }

  function setSingleJoint(name, deg) {
    const check = checkTargetSafety({ [name]: deg });
    if (!check.safe) { warnUnsafe(check); return false; }
    if (_instances[name]) _instances[name].setAngle(deg, true);
    return true;
  }

  function getAllAngles() {
    const out = {};
    JOINT_CONFIG.forEach(cfg => {
      if (_instances[cfg.name]) out[cfg.name] = _instances[cfg.name].getValue();
    });
    return out;
  }

  function getHomeAngles() {
    const out = {};
    JOINT_CONFIG.forEach(cfg => { out[cfg.name] = cfg.init; });
    return out;
  }

  function getConfigList()         { return Object.keys(_namedConfigs); }
  function saveNamedConfig(name)   { if(!name) return false; _namedConfigs[name]=getAllAngles(); return true; }
  function loadNamedConfig(n,anim) { if(!_namedConfigs[n]) return false; return loadAllAngles(_namedConfigs[n],anim); }
  function deleteNamedConfig(name) { if(!_namedConfigs[name]) return false; delete _namedConfigs[name]; return true; }

  function saveToLocalStorage() {
    try { localStorage.setItem('fr3Angles', JSON.stringify(getAllAngles())); return true; } catch(e){ return false; }
  }
  function loadFromLocalStorage(animate) {
    try { return loadAllAngles(JSON.parse(localStorage.getItem('fr3Angles')||'{}'), animate); } catch(e){ return false; }
  }
  function exportAsJSON()      { return JSON.stringify(getAllAngles(), null, 2); }
  function importFromJSON(str) { try { return loadAllAngles(JSON.parse(str), true); } catch(e){ return false; } }

  window.PandaSliderControl = {
    init, loadAllAngles, setSingleJoint, getAllAngles, getHomeAngles,
    getConfigList, saveNamedConfig, loadNamedConfig, deleteNamedConfig,
    saveToLocalStorage, loadFromLocalStorage, exportAsJSON, importFromJSON,
    JOINT_CONFIG,
  };

})();
