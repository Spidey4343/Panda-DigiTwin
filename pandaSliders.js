/**
 * PandaSliderControl — robot-agnostic joint slider system.
 * Originally built for the Franka Research 3 (FR3), 7-DOF, all joints
 * rotating around Z in their local frame. Extended to also drive the KUKA
 * KR4R600 (6-DOF, mixed Z/Y/X joint axes) so the app's header toggle can
 * switch which robot is live without touching any other module — every
 * function here already iterates the *active* JOINT_CONFIG, so adding a
 * second robot only meant: (1) making JOINT_CONFIG swappable instead of a
 * hardcoded const, (2) giving each joint its own rotation axis instead of
 * assuming Z, and (3) a setRobot() entry point that tears down and rebuilds
 * the slider DOM + X3D bindings for the newly active config.
 */
(function () {
  'use strict';

  const DEG = Math.PI / 180;
  const RAD = 180 / Math.PI;

  // ── Robot configs ───────────────────────────────────────────
  // axis: [x,y,z] rotation axis in the joint's local frame (defaults to
  // Z if omitted, which keeps the original FR3 config unchanged).
  const ROBOTS = {
    fr3: {
      label: 'FR3',
      wrl: 'panda.wrl',
      jointConfig: [
        { name: 'joint1', label: 'J1', min: -166, max:  166, init:   0, color: '#e63946', axis: [0,0,1] },
        { name: 'joint2', label: 'J2', min: -105, max:  105, init: -45, color: '#f4a261', axis: [0,0,1] },
        { name: 'joint3', label: 'J3', min: -166, max:  166, init:   0, color: '#2a9d8f', axis: [0,0,1] },
        { name: 'joint4', label: 'J4', min: -176, max:   -7, init:-135, color: '#457b9d', axis: [0,0,1] },
        { name: 'joint5', label: 'J5', min: -165, max:  165, init:   0, color: '#a8dadc', axis: [0,0,1] },
        { name: 'joint6', label: 'J6', min:   25, max:  265, init:  90, color: '#e9c46a', axis: [0,0,1] },
        { name: 'joint7', label: 'J7', min: -175, max:  175, init:  45, color: '#c77dff', axis: [0,0,1] },
      ],
    },
    kuka: {
      label: 'KR4R600',
      wrl: 'kuka.wrl',
      // Ranges match the "slider shows" values documented in kuka.wrl /
      // the original KR4R600digiTwin.html axisConfig (degrees). Axes match
      // the AxisConverter wiring in kuka.wrl: A1=Z, A2/A3/A5=Y, A4/A6=X.
      jointConfig: [
        { name: 'A1', label: 'A1', min: -165, max:  165, init:   0, color: '#e63946', axis: [0,0,1] },
        { name: 'A2', label: 'A2', min: -190, max:   35, init: -90, color: '#f4a261', axis: [0,1,0] },
        { name: 'A3', label: 'A3', min: -110, max:  145, init:  90, color: '#2a9d8f', axis: [0,1,0] },
        { name: 'A4', label: 'A4', min: -180, max:  180, init:   0, color: '#457b9d', axis: [1,0,0] },
        { name: 'A5', label: 'A5', min: -115, max:  115, init:   0, color: '#a8dadc', axis: [0,1,0] },
        { name: 'A6', label: 'A6', min: -345, max:  345, init:   0, color: '#e9c46a', axis: [1,0,0] },
      ],
    },
  };

  let activeRobot   = 'fr3';
  let JOINT_CONFIG  = ROBOTS.fr3.jointConfig;
  let _parentId     = 'slider-list';

  const _namedConfigs = {};
  const _instances    = {};

  // ── Safety gate ─────────────────────────────────────────────
  // Builds the active robot's full joint radian array (current angles + any
  // overrides), runs it through PandaSafety for whichever robot is active,
  // and returns the result. Fails open (safe) if the safety module isn't
  // loaded, so a missing script never bricks normal operation.
  function checkTargetSafety(overrides) {
    if (typeof window.PandaSafety === 'undefined') return { safe: true };
    const rad = JOINT_CONFIG.map(cfg => {
      const deg = (overrides && overrides[cfg.name] != null)
        ? overrides[cfg.name]
        : (_instances[cfg.name] ? _instances[cfg.name].getValue() : cfg.init);
      return deg * DEG;
    });
    return window.PandaSafety.checkSafety(rad, activeRobot);
  }

  function warnUnsafe(result) {
    const reason = result.groundViolation ? 'would go below ground' : 'self-collision risk';
    const detail = result.details && result.details[0] ? ' — ' + result.details[0] : '';
    const msg = `Blocked: ${reason}${detail}`;
    if (typeof window.setStatus === 'function') window.setStatus('⛔ ' + msg, 'err');
    if (typeof window.termLog === 'function') window.termLog(msg, 'err');
    console.warn(`[${activeRobot.toUpperCase()} Safety] ` + msg);
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
          console.warn(`[${activeRobot.toUpperCase()}] ${this.cfg.name} failed to connect to 3D scene`);
          return;
        }
        try {
          const browser = (typeof X3D !== 'undefined') &&
            (X3D.getBrowser('#x3dCanvas') || X3D.getBrowser());
          const scene = browser && browser.currentScene;
          const node  = scene && scene.getNamedNode(this.cfg.name);

          if (!node) { setTimeout(() => attempt(tries - 1), 400); return; }

          this.node = node;
          const ax = this.cfg.axis || [0, 0, 1]; // default Z, matches original FR3 behavior
          this.Rot = new X3D.SFRotation(ax[0], ax[1], ax[2], this.cfg.init * DEG);
          this.node.rotation = this.Rot;

          // Bidirectional: scene drag → slider
          try {
            this.node.rotation.addFieldCallback('sync_' + this.cfg.name, val => {
              this._syncFromScene(val[3] * RAD);
            });
          } catch(e) { /* field callback not always available */ }

          console.log(`[${activeRobot.toUpperCase()}] ${this.cfg.name} connected`);
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

    destroy() {
      if (this.sliderEl) {
        const wrap = this.sliderEl.closest('.joint-slider-wrap');
        if (wrap && wrap.parentNode) wrap.parentNode.removeChild(wrap);
      }
    }
  }

  // ── Static API ───────────────────────────────────────────────

  function init(parentId) {
    _parentId = parentId;
    JOINT_CONFIG.forEach(cfg => {
      _instances[cfg.name] = new JointSlider(cfg, parentId);
    });
    if (window.IKController) window.IKController.setSliderInstances(_instances);
  }

  // Switches which robot's joints the slider panel drives. Tears down the
  // current slider DOM + instances and rebuilds from the new robot's
  // JOINT_CONFIG. Named configs are cleared since joint names (joint1..7
  // vs A1..A6) aren't shared between robots.
  function setRobot(robotKey) {
    if (!ROBOTS[robotKey]) { console.warn('[PandaSliderControl] unknown robot:', robotKey); return false; }
    activeRobot = robotKey;
    JOINT_CONFIG = ROBOTS[robotKey].jointConfig;

    Object.values(_instances).forEach(inst => inst.destroy());
    Object.keys(_instances).forEach(k => delete _instances[k]);
    Object.keys(_namedConfigs).forEach(k => delete _namedConfigs[k]);

    const parent = document.getElementById(_parentId);
    if (parent) parent.innerHTML = '';

    init(_parentId);
    return true;
  }

  function getActiveRobot() { return activeRobot; }
  function getRobotList()   { return Object.keys(ROBOTS).map(key => ({ key, label: ROBOTS[key].label, wrl: ROBOTS[key].wrl })); }

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
    try { localStorage.setItem('fr3Angles_' + activeRobot, JSON.stringify(getAllAngles())); return true; } catch(e){ return false; }
  }
  function loadFromLocalStorage(animate) {
    try { return loadAllAngles(JSON.parse(localStorage.getItem('fr3Angles_' + activeRobot)||'{}'), animate); } catch(e){ return false; }
  }
  function exportAsJSON()      { return JSON.stringify(getAllAngles(), null, 2); }
  function importFromJSON(str) { try { return loadAllAngles(JSON.parse(str), true); } catch(e){ return false; } }

  window.PandaSliderControl = {
    init, setRobot, getActiveRobot, getRobotList,
    loadAllAngles, setSingleJoint, getAllAngles, getHomeAngles,
    getConfigList, saveNamedConfig, loadNamedConfig, deleteNamedConfig,
    saveToLocalStorage, loadFromLocalStorage, exportAsJSON, importFromJSON,
    get JOINT_CONFIG() { return JOINT_CONFIG; },
    ROBOTS,
  };

})();
