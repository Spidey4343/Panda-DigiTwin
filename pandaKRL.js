/**
 * KRL Interpreter Pipeline — Parser → IR → Dispatcher → Interpreter Loop
 * Covers: PAR-01 (parser), PAR-02 (IR), EXE-01 (interpreter loop),
 *         EXE-02 (dispatcher), CTRL-01/02/03 (IF/WHILE/WAIT),
 *         IO-01 (DECL+assignment), IO-02 ($IN/$OUT), INT-01 (backend API)
 *
 * Supported KRL subset:
 *   HOME | RESET
 *   LIN {X n Y n Z n}
 *   PTP {A1 n A2 n ... A7 n}
 *   WAIT n           (or WAIT FOR n)   — ms
 *   DECL INT x [= expr]
 *   DECL BOOL flag [= expr]
 *   name = expr
 *   $OUT[n] = expr
 *   IF cond THEN ... [ELSE ...] ENDIF
 *   WHILE cond [DO] ... ENDWHILE
 * Expressions support: + - * / == != <> < > <= >= AND OR NOT TRUE FALSE
 * $IN[n], $OUT[n], and any DECL'd variable.
 */
(function () {
  'use strict';

  // ── Expression evaluator ─────────────────────────────────────
  function evalExpr(exprStr, scope) {
    if (exprStr == null || exprStr === '') throw new Error('empty expression');
    let js = String(exprStr).trim();
    js = js.replace(/\$IN\s*\[\s*(\d+)\s*\]/gi, 'IN[$1]');
    js = js.replace(/\$OUT\s*\[\s*(\d+)\s*\]/gi, 'OUT[$1]');
    js = js.replace(/<>/g, '!==');
    js = js.replace(/\bTRUE\b/gi, 'true').replace(/\bFALSE\b/gi, 'false');
    js = js.replace(/\bAND\b/gi, '&&').replace(/\bOR\b/gi, '||').replace(/\bNOT\b/gi, '!');
    // bare '=' used as equality inside expressions (KRL uses '==' rarely) — leave '==' as-is.
    try {
      // eslint-disable-next-line no-new-func
      const fn = new Function('vars', 'IN', 'OUT', `with (vars) { return (${js}); }`);
      return fn(scope.vars, scope.IN, scope.OUT);
    } catch (e) {
      throw new Error(`bad expression "${exprStr}": ${e.message}`);
    }
  }

  // ── Parser: KRL text → IR ────────────────────────────────────
  function parseProgram(text) {
    const lines = String(text)
      .split('\n')
      .map(l => l.trim())
      .filter(l => l.length && !l.startsWith('//') && !l.startsWith(';'));

    let pos = 0;
    const peek = () => lines[pos];
    const atEnd = () => pos >= lines.length;

    function parseBlock(terminators) {
      const nodes = [];
      while (!atEnd()) {
        const upper = peek().toUpperCase();
        if (terminators.some(t => upper === t || upper.startsWith(t + ' '))) break;
        nodes.push(parseStatement());
      }
      return nodes;
    }

    function parseStatement() {
      const line = lines[pos++];
      const upper = line.toUpperCase();
      let m;

      if (upper === 'HOME' || upper === 'RESET') return { type: 'HOME', line };

      if ((m = line.match(/^LIN\s*\{([^}]+)\}/i))) {
        const body = m[1];
        const get = ax => { const mm = body.match(new RegExp(ax + '\\s*([\\-\\d.]+)', 'i')); return mm ? parseFloat(mm[1]) : null; };
        return { type: 'MOVE_LIN', x: get('X'), y: get('Y'), z: get('Z'), line };
      }

      if ((m = line.match(/^PTP\s*\{([^}]+)\}/i))) {
        const body = m[1];
        // Keyed by axis NUMBER (1-7), not a robot-specific joint name — the
        // parser stays robot-agnostic; JSBackend.move_ptp translates these
        // into whichever robot is currently active's real joint names
        // (joint1..joint7 for FR3, A1..A6 for KUKA). A PTP command only
        // ever needs to name as many axes as the active robot has (KUKA
        // commands simply won't include A7, which this loop already
        // tolerates since it only stores an axis if a match is found).
        const angles = {};
        for (let i = 1; i <= 7; i++) {
          const mm = body.match(new RegExp('A' + i + '\\s*([\\-\\d.]+)', 'i'));
          if (mm) angles[i] = parseFloat(mm[1]);
        }
        return { type: 'MOVE_PTP', angles, line };
      }

      if ((m = line.match(/^WAIT\s+(?:FOR\s+)?([\-\d.]+)/i))) {
        return { type: 'WAIT', ms: parseFloat(m[1]), line };
      }

      if ((m = line.match(/^DECL\s+(INT|BOOL)\s+(\w+)\s*(?:=\s*(.+))?$/i))) {
        return { type: 'DECL', varType: m[1].toUpperCase(), name: m[2], init: m[3] ? m[3].trim() : null, line };
      }

      if ((m = line.match(/^\$OUT\s*\[\s*(\d+)\s*\]\s*=\s*(.+)$/i))) {
        return { type: 'IO_OUT', index: parseInt(m[1], 10), expr: m[2].trim(), line };
      }

      if (upper.startsWith('IF ') || upper.startsWith('IF(')) {
        let cond = line.replace(/^IF\s*/i, '').trim();
        cond = cond.replace(/\s+THEN\s*$/i, '').trim();
        const thenBlock = parseBlock(['ELSE', 'ENDIF']);
        let elseBlock = [];
        if (!atEnd() && peek().toUpperCase() === 'ELSE') { pos++; elseBlock = parseBlock(['ENDIF']); }
        if (!atEnd() && peek().toUpperCase() === 'ENDIF') pos++;
        else throw new Error(`IF on line "${line}" missing ENDIF`);
        return { type: 'IF', cond, thenBlock, elseBlock, line };
      }

      if (upper.startsWith('WHILE ') || upper.startsWith('WHILE(')) {
        let cond = line.replace(/^WHILE\s*/i, '').trim();
        cond = cond.replace(/\s+DO\s*$/i, '').trim();
        const body = parseBlock(['ENDWHILE']);
        if (!atEnd() && peek().toUpperCase() === 'ENDWHILE') pos++;
        else throw new Error(`WHILE on line "${line}" missing ENDWHILE`);
        return { type: 'WHILE', cond, body, line };
      }

      if ((m = line.match(/^(\w+)\s*=\s*(.+)$/))) {
        return { type: 'ASSIGN', name: m[1], expr: m[2].trim(), line };
      }

      throw new Error(`could not parse line: "${line}"`);
    }

    return parseBlock([]);
  }

  // ── Dispatcher: IR type → executor ───────────────────────────
  const dispatch = {
    HOME: async (node, ctx) => {
      await ctx.backend.move_ptp(ctx.homeAngles());
      ctx.log('HOME executed', 'ok');
    },
    MOVE_LIN: async (node, ctx) => {
      await ctx.backend.move_lin({ x: node.x, y: node.y, z: node.z });
      ctx.log(`LIN → [${node.x}, ${node.y}, ${node.z}]`, 'ok');
    },
    MOVE_PTP: async (node, ctx) => {
      await ctx.backend.move_ptp(node.angles);
      ctx.log('PTP executed', 'ok');
    },
    WAIT: async (node, ctx) => {
      ctx.log(`WAIT ${node.ms}ms`, '');
      await ctx.backend.wait(node.ms);
    },
    DECL: async (node, ctx) => {
      const val = node.init != null ? evalExpr(node.init, ctx.scope) : (node.varType === 'BOOL' ? false : 0);
      ctx.scope.vars[node.name] = val;
      ctx.scope.types[node.name] = node.varType;
      ctx.log(`DECL ${node.varType} ${node.name} = ${val}`, '');
      ctx.onScopeChange();
    },
    ASSIGN: async (node, ctx) => {
      if (!(node.name in ctx.scope.vars)) throw new Error(`variable "${node.name}" not declared`);
      const val = evalExpr(node.expr, ctx.scope);
      ctx.scope.vars[node.name] = val;
      ctx.log(`${node.name} = ${val}`, '');
      ctx.onScopeChange();
    },
    IO_OUT: async (node, ctx) => {
      const val = !!evalExpr(node.expr, ctx.scope);
      ctx.scope.OUT[node.index] = val;
      ctx.backend.set_output(node.index, val);
      ctx.log(`$OUT[${node.index}] = ${val}`, '');
      ctx.onScopeChange();
    },
    IF: async (node, ctx) => {
      const cond = !!evalExpr(node.cond, ctx.scope);
      ctx.log(`IF ${node.cond} → ${cond}`, '');
      await runBlock(cond ? node.thenBlock : node.elseBlock, ctx);
    },
    WHILE: async (node, ctx) => {
      let guard = 0;
      while (evalExpr(node.cond, ctx.scope)) {
        if (ctx.stopped()) return;
        await runBlock(node.body, ctx);
        if (++guard > 5000) { ctx.log('WHILE aborted: guard limit (possible infinite loop)', 'err'); return; }
      }
    },
  };

  async function runBlock(nodes, ctx) {
    for (const node of nodes) {
      if (ctx.stopped()) return;
      const fn = dispatch[node.type];
      if (!fn) { ctx.log(`no executor for "${node.type}"`, 'err'); continue; }
      try {
        await fn(node, ctx);
      } catch (e) {
        ctx.log(`✗ line "${node.line}": ${e.message}`, 'err');
        throw e;
      }
    }
  }

  // ── Interpreter (public) ─────────────────────────────────────
  const KRLInterpreter = (function () {
    let running = false;
    let stopRequested = false;
    let scope = { vars: {}, types: {}, IN: new Array(9).fill(false), OUT: new Array(9).fill(false) };

    function resetScope() {
      scope = { vars: {}, types: {}, IN: scope.IN.slice(), OUT: new Array(9).fill(false) };
    }

    async function run(ir, opts) {
      const { backend, log, homeAngles, onScopeChange } = opts;
      running = true; stopRequested = false;
      const ctx = {
        scope, backend, log,
        homeAngles: homeAngles || (() => ({})),
        onScopeChange: onScopeChange || (() => {}),
        stopped: () => stopRequested,
      };
      try {
        await runBlock(ir, ctx);
        if (!stopRequested) log('Program finished', 'ok');
        else log('Program stopped', 'warn');
      } catch (e) {
        log('Program aborted: ' + e.message, 'err');
      } finally {
        running = false;
      }
    }

    return {
      run,
      stop: () => { stopRequested = true; },
      isRunning: () => running,
      resetScope,
      getScope: () => scope,
      setInput: (idx, val) => { scope.IN[idx] = !!val; },
    };
  })();

  // ── Backend API Wrapper (INT-01) ─────────────────────────────
  // Unified surface the interpreter drives motion/IO through. Today only a
  // JS/browser backend exists (this one), but every call is async and
  // side-effect-free from the interpreter's point of view, so a future
  // Python/MATLAB backend could implement the same four calls and be
  // swapped in via KRLEngine.setBackend() (INT-02 backend switching).
  const JSBackend = {
    async move_ptp(anglesMap) {
      // anglesMap may be axis-NUMBER-keyed (from a parsed PTP {A1 .. A7 ..}
      // command — see PAR-01) or already real-joint-name-keyed (from
      // ctx.homeAngles(), i.e. PandaSliderControl.getHomeAngles(), which
      // already returns whatever the active robot's real names are).
      // Translate the numeric case into the ACTIVE robot's real joint
      // names here — this is the one place PTP becomes robot-specific,
      // keeping the parser/dispatcher (PAR-01/EXE-02) robot-agnostic.
      const cfg = window.PandaSliderControl.JOINT_CONFIG;
      const named = {};
      Object.entries(anglesMap).forEach(([k, v]) => {
        if (/^\d+$/.test(k)) {
          const idx = parseInt(k, 10) - 1;
          if (cfg[idx]) named[cfg[idx].name] = v;
        } else {
          named[k] = v;
        }
      });
      const ok = window.PandaSliderControl.loadAllAngles(named, true); // 700ms eased animation
      if (!ok) throw new Error('PTP blocked by safety check (self-collision or ground contact)');
      return new Promise(resolve => setTimeout(resolve, 750));
    },
    async move_lin({ x, y, z }) {
      const xEl = document.getElementById('X'), yEl = document.getElementById('Y'), zEl = document.getElementById('Z');
      if (x != null && xEl) xEl.value = x;
      if (y != null && yEl) yEl.value = y;
      if (z != null && zEl) zEl.value = z;
      const result = window.IKController.solve();
      if (result && result.blocked) {
        throw new Error(result.blocked === 'safety'
          ? 'LIN blocked by safety check (self-collision or ground contact)'
          : 'LIN target unreachable');
      }
      return new Promise(resolve => setTimeout(resolve, 900)); // ~51-frame quintic trajectory
    },
    async wait(ms) {
      return new Promise(resolve => setTimeout(resolve, Math.max(0, ms)));
    },
    set_output(index, value) {
      if (typeof window.onKRLOutputChange === 'function') window.onKRLOutputChange(index, value);
    },
    get_input(index) {
      return KRLInterpreter.getScope().IN[index] || false;
    },
  };

  let activeBackend = JSBackend;

  window.KRLEngine = {
    parseProgram,
    evalExpr,
    KRLInterpreter,
    backends: { js: JSBackend },
    getBackend: () => activeBackend,
    setBackend: (nameOrObj) => {
      activeBackend = typeof nameOrObj === 'string' ? window.KRLEngine.backends[nameOrObj] : nameOrObj;
    },
  };
})();
