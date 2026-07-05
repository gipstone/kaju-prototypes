/* KajuBauprobe2 - one-sided build + movable mirror axis.

   Variant 2 of the Bauprobe: the user builds only ONE half by dragging stones
   from the tray, then closes the arch with a movable mirror axis (a vertical
   line, nudged left/right, snapping to sensible closing points). The heavy
   physics (rigid-block statics + collapse) is INHERITED unchanged from
   KajuBauprobe via window.KajuBauprobe._internals - this file adds only the
   one-sided build, the axis logic and its own render/interaction layer.

   The old bauprobe.js is NOT modified. See BAUPROBE2-SPEC.md.                  */
(function () {
  'use strict';

  var IS_BROWSER = typeof window !== 'undefined' && typeof document !== 'undefined';

  // Inherited physics kernels (pure functions, verified free of module state).
  var CORE = IS_BROWSER
    ? (window.KajuBauprobe && window.KajuBauprobe._internals)
    : (typeof require !== 'undefined' ? require('./bauprobe.js')._internals : null);

  /* ================= geometry ================= */

  // A reflected polygon reverses its winding; SAT/collapse needs CCW slices.
  function ensureCCW(poly) {
    var a = 0, i;
    for (i = 0; i < poly.length; i++) {
      var p = poly[i], q = poly[(i + 1) % poly.length];
      a += p[0] * q[1] - q[0] * p[1];
    }
    return a < 0 ? poly.slice().reverse() : poly;
  }

  // Reflect a stone item about the vertical line x = axisX. Geometry is mirrored;
  // scalar/meta fields carry through. Same shape the inherited kernels consume.
  function mirror(it, axisX) {
    function m(p) { return [2 * axisX - p[0], p[1]]; }
    function mFace(f) { return { pin: m(f.pin), pout: m(f.pout) }; }
    var out = {
      type: it.type, inverted: it.inverted, area: it.area, massRel: it.massRel,
      turn: it.turn, chain: it.chain, idx: it.idx, born: it.born,
      poly: it.poly.map(m),
      slices: it.slices.map(function (s) { return ensureCCW(s.map(m)); }),
      centroid: m(it.centroid),
      startFace: mFace(it.startFace),
      endFace: mFace(it.endFace),
      mirrored: true
    };
    if (it.C) out.C = m(it.C);
    return out;
  }

  // How the arch closes. One build has ONE missing angle (180 - 2*sumDeg), so
  // there is 0 or 1 valid closure: missing==0 -> vertical joint (even stone
  // count), missing==a stone's angle -> that stone is the keystone, else the
  // half can't close cleanly (keep building or over-built).
  function closure(built) {
    var STONES = CORE.STONES, sum = 0, i;
    for (i = 0; i < built.length; i++) {
      var st = STONES[built[i].type];
      sum += st.angle * (built[i].inverted ? -1 : 1);
    }
    var missing = 180 - 2 * sum;
    var TOL = 0.01;
    if (Math.abs(missing) < TOL) return { kind: 'joint', keyType: null, missing: 0 };
    if (missing > TOL) {
      for (var k in STONES) {
        if (STONES.hasOwnProperty(k) && !STONES[k].sw && Math.abs(STONES[k].angle - missing) < TOL) {
          return { kind: 'keystone', keyType: k, missing: missing };
        }
      }
    }
    return null;
  }

  /* ================= render + interaction (browser only) ================= */

  var W = 820, H = 700;
  var STONES = CORE ? CORE.STONES : {};
  var TRAY = ['gelb', 'orange', 'gruen', 'wuerfel'];    // no Weichensteine in v1
  var TILE = 84, TILE_GAP = 14, TRAY_Y = H - 100;
  var GROUND_Y = TRAY_Y - 44;                           // screen y of world ground (y=0)

  var canvas, ctx, taskEl, tipEl, verdictEl, mirrorBtn, testBtn, resetBtn;
  var state = null, visible = false;
  var cam = { z: 22, ox: W / 2, tz: 22, tox: W / 2 };
  var trayRects = [];

  function freshState() {
    return { built: [], axisX: 0, mirrored: false, drag: null, phase: 'build',
             model: null, result: null };
  }

  function leftItems() {
    return state.built.length ? CORE.computeHalf(state.built, 0).items : [];
  }
  function shiftItem(it, dx) {
    return { type: it.type, poly: it.poly.map(function (p) { return [p[0] + dx, p[1]]; }) };
  }

  // fit the visible stones into the stage (auto-zoom, centred on their bbox)
  function retarget(items) {
    if (!items.length) { cam.tz = 26; cam.tox = W / 2; return; }
    var minx = 1e9, maxx = -1e9, maxy = 4;
    items.forEach(function (it) {
      it.poly.forEach(function (p) {
        if (p[0] < minx) minx = p[0];
        if (p[0] > maxx) maxx = p[0];
        if (p[1] > maxy) maxy = p[1];
      });
    });
    var w = Math.max(6, maxx - minx), h = Math.max(4, maxy);
    var z = Math.min((W * 0.66) / w, (GROUND_Y - 64) / h);
    cam.tz = Math.max(8, Math.min(42, z));
    cam.tox = W / 2 - ((minx + maxx) / 2) * cam.tz;
  }

  function w2s(p) { return [cam.ox + p[0] * cam.z, GROUND_Y - p[1] * cam.z]; }

  function drawStone(it, ghost) {
    if (!it.poly || !it.poly.length) return;
    ctx.beginPath();
    var s = w2s(it.poly[0]); ctx.moveTo(s[0], s[1]);
    for (var i = 1; i < it.poly.length; i++) { s = w2s(it.poly[i]); ctx.lineTo(s[0], s[1]); }
    ctx.closePath();
    var col = STONES[it.type] ? STONES[it.type].color : '#9a8';
    if (ghost) {
      ctx.globalAlpha = 0.3; ctx.fillStyle = col; ctx.fill(); ctx.globalAlpha = 1;
      ctx.lineWidth = 1.5; ctx.strokeStyle = 'rgba(245,236,226,.5)'; ctx.stroke();
    } else {
      ctx.save(); ctx.translate(4, 4); ctx.fillStyle = 'rgba(0,0,0,.28)'; ctx.fill(); ctx.restore();
      ctx.fillStyle = col; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = '#3a221c'; ctx.stroke();
    }
  }

  function drawGround() {
    ctx.strokeStyle = 'rgba(245,236,226,.38)'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(26, GROUND_Y); ctx.lineTo(W - 26, GROUND_Y); ctx.stroke();
  }

  function drawAxis() {
    var sx = cam.ox + state.axisX * cam.z;
    ctx.save();
    ctx.strokeStyle = state.mirrored ? 'rgba(240,146,0,.85)' : 'rgba(245,236,226,.55)';
    ctx.lineWidth = 1.5; ctx.setLineDash([7, 7]);
    ctx.beginPath(); ctx.moveTo(sx, 34); ctx.lineTo(sx, GROUND_Y); ctx.stroke();
    ctx.restore();
  }

  // static hitboxes — computed once at init so onDown never depends on draw()
  function layoutTray() {
    trayRects = [];
    var total = TRAY.length * TILE + (TRAY.length - 1) * TILE_GAP;
    var x0 = (W - total) / 2;
    for (var i = 0; i < TRAY.length; i++) {
      trayRects.push({ type: TRAY[i], x: x0 + i * (TILE + TILE_GAP), y: TRAY_Y, w: TILE, h: TILE });
    }
  }
  function drawTray() {
    ctx.textAlign = 'center';
    for (var i = 0; i < trayRects.length; i++) {
      var t = trayRects[i];
      ctx.fillStyle = STONES[t.type].color; ctx.fillRect(t.x, t.y, t.w, t.h);
      ctx.lineWidth = 2; ctx.strokeStyle = '#3a221c'; ctx.strokeRect(t.x, t.y, t.w, t.h);
      ctx.fillStyle = '#3a221c'; ctx.font = '700 15px "Ciutadella", system-ui, sans-serif';
      ctx.fillText(STONES[t.type].label, t.x + t.w / 2, t.y + t.h - 11);
    }
  }

  function drawDrag() {
    if (!state.drag) return;
    ctx.globalAlpha = 0.8; ctx.fillStyle = STONES[state.drag.type].color;
    ctx.fillRect(state.drag.x - 28, state.drag.y - 28, 56, 56);
    ctx.lineWidth = 2; ctx.strokeStyle = '#3a221c'; ctx.strokeRect(state.drag.x - 28, state.drag.y - 28, 56, 56);
    ctx.globalAlpha = 1;
  }

  function draw() {
    if (!ctx) return;
    ctx.clearRect(0, 0, W, H);
    var solid = [], ghost = [];
    if (state.phase === 'build') {
      solid = leftItems();
      if (state.mirrored) ghost = solid.map(function (it) { return mirror(it, state.axisX); });
    } else if (state.model) {
      solid = state.model.chain.map(function (it) { return shiftItem(it, state.axisX); });
    }
    retarget(solid.concat(ghost));
    cam.z += (cam.tz - cam.z) * 0.2; cam.ox += (cam.tox - cam.ox) * 0.2;

    drawGround();
    ghost.forEach(function (it) { drawStone(it, true); });
    solid.forEach(function (it) { drawStone(it, false); });
    if (state.phase === 'build') drawAxis();
    drawTray();
    drawDrag();
  }

  /* ---- interaction ---- */

  function canvasXY(e) {
    var r = canvas.getBoundingClientRect();
    return [(e.clientX - r.left) * (W / r.width), (e.clientY - r.top) * (H / r.height)];
  }
  function onDown(e) {
    if (!state || state.phase !== 'build') return;
    var p = canvasXY(e);
    for (var i = 0; i < trayRects.length; i++) {
      var t = trayRects[i];
      if (p[0] >= t.x && p[0] <= t.x + t.w && p[1] >= t.y && p[1] <= t.y + t.h) {
        state.drag = { type: t.type, x: p[0], y: p[1] };
        try { canvas.setPointerCapture(e.pointerId); } catch (_) { /* synthetic events */ }
        return;
      }
    }
  }
  function onMove(e) {
    if (!state || !state.drag) return;
    var p = canvasXY(e); state.drag.x = p[0]; state.drag.y = p[1];
  }
  function onUp(e) {
    if (!state || !state.drag) return;
    var p = canvasXY(e), t = state.drag.type;
    state.drag = null;
    if (p[1] < TRAY_Y - 8) {                     // dropped over the stage
      state.built.push({ type: t, inverted: false });
      state.mirrored = false;
      updateTask();
    }
  }

  function nudgeAxis(dir) {
    if (!state || state.phase !== 'build') return;
    state.axisX += dir * 0.4;
    if (Math.abs(state.axisX) < 0.5) state.axisX = 0;   // magnet to the symmetry vertex
  }

  function runTest() {
    if (!state || state.phase !== 'build' || !state.built.length) return;
    var c = closure(state.built);
    var ev = CORE.evaluateArch(state.built, c ? c.keyType : null, {});
    state.model = ev.model; state.result = ev.result; state.phase = 'result';
    var v;
    if (ev.state === 'stable') v = 'Trägt.';
    else if (ev.state === 'critical') v = 'Wackelt.';
    else if (ev.state === 'unstable') v = 'Rumms.';
    else if (ev.state === 'incomplete') v = 'Offen.';
    else v = '—';
    showVerdict(v);
    if (ev.state === 'incomplete') showTip('Der Bogen ist noch nicht geschlossen – bau weiter oder spiegle zuerst.');
    else if (ev.state === 'unstable') showTip('Zu flach oder unsymmetrisch – der Schub drückt die Füße auseinander.');
    testBtn.disabled = true;
  }

  function reset() {
    state = freshState(); hideVerdict(); hideTip();
    if (testBtn) testBtn.disabled = false;
    updateTask();
  }

  function showVerdict(txt) {
    if (!verdictEl) return;
    verdictEl.textContent = txt;
    verdictEl.style.transition = 'opacity .45s var(--ease-smooth), transform .45s var(--ease-smooth)';
    verdictEl.style.opacity = '1'; verdictEl.style.transform = 'translateY(0) scale(1)';
  }
  function hideVerdict() { if (verdictEl) { verdictEl.style.opacity = '0'; verdictEl.textContent = ''; } }
  function showTip(html) { if (tipEl) { tipEl.innerHTML = html; tipEl.className = 'bauprobe__tip is-on'; } }
  function hideTip() { if (tipEl) tipEl.className = 'bauprobe__tip'; }

  function updateTask() {
    if (!taskEl || !state) return;
    if (!state.built.length) { taskEl.innerHTML = 'Zieh Steine aus dem Vorrat und bau eine Bogenhälfte.'; return; }
    var c = closure(state.built);
    if (c && c.kind === 'joint') taskEl.innerHTML = 'Die Hälfte schließt senkrecht. <b>Spiegeln</b>, dann <b>Bogen testen</b>.';
    else if (c && c.kind === 'keystone') taskEl.innerHTML = 'Es fehlt genau ein <b>' + STONES[c.keyType].label + '</b>-Stein. <b>Spiegeln</b>, dann testen.';
    else taskEl.innerHTML = 'Bau weiter, bis sich die Hälfte sauber schließen lässt.';
  }

  /* ---- loop + init ---- */

  function tick() {
    requestAnimationFrame(tick);
    if (visible && state) draw();
  }

  function init() {
    canvas = document.getElementById('bp2-canvas');
    if (!canvas || !CORE) return;
    ctx = canvas.getContext('2d');
    taskEl = document.getElementById('bp2-task');
    tipEl = document.getElementById('bp2-tip');
    verdictEl = document.getElementById('bp2-verdict');
    mirrorBtn = document.getElementById('bp2-mirror');
    testBtn = document.getElementById('bp2-test');
    resetBtn = document.getElementById('bp2-reset');
    state = freshState();
    layoutTray();
    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointermove', onMove);
    canvas.addEventListener('pointerup', onUp);
    canvas.addEventListener('pointercancel', function () { if (state) state.drag = null; });
    mirrorBtn.addEventListener('click', function () { if (state && state.phase === 'build') state.mirrored = !state.mirrored; });
    document.getElementById('bp2-axis-left').addEventListener('click', function () { nudgeAxis(-1); });
    document.getElementById('bp2-axis-right').addEventListener('click', function () { nudgeAxis(1); });
    testBtn.addEventListener('click', runTest);
    resetBtn.addEventListener('click', reset);
    var io = new IntersectionObserver(function (es) { visible = es[0].isIntersecting; }, { threshold: 0.04 });
    io.observe(canvas);
    updateTask();
    requestAnimationFrame(tick);
  }

  if (IS_BROWSER) {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
  }

  /* ================= self-test harness ================= */

  function selfTest2() {
    var cases = [];
    function test(name, fn) {
      try {
        var r = fn();
        var ok = r && typeof r === 'object' ? !!r.ok : !!r;
        var info = r && typeof r === 'object' ? (r.info || '') : '';
        cases.push({ name: name, ok: ok, info: info });
      } catch (e) {
        cases.push({ name: name, ok: false, info: 'threw: ' + e.message });
      }
    }

    // A synthetic stone carrying exactly the fields mirror() must transform.
    function fixtureStone() {
      return {
        type: 'gelb', inverted: false, area: 12, massRel: 12, turn: 60,
        poly: [[1, 0], [3, 0], [3, 2], [1, 2]],
        slices: [[[1, 0], [3, 0], [3, 2], [1, 2]]],
        centroid: [2, 1],
        startFace: { pin: [1, 0], pout: [1, 2] },
        endFace: { pin: [3, 0], pout: [3, 2] },
        C: [0, 5]
      };
    }

    function signedArea(poly) {
      var a = 0;
      for (var i = 0; i < poly.length; i++) {
        var p = poly[i], q = poly[(i + 1) % poly.length];
        a += p[0] * q[1] - q[0] * p[1];
      }
      return a / 2;
    }

    // ---- mirror() ----
    test('mirror(axis=0) reflects centroid about the axis', function () {
      var m = mirror(fixtureStone(), 0);
      return { ok: m.centroid[0] === -2 && m.centroid[1] === 1,
               info: 'centroid=' + JSON.stringify(m.centroid) };
    });
    test('mirror(axis=3) reflects centroid about x=3', function () {
      var m = mirror(fixtureStone(), 3);
      return { ok: m.centroid[0] === 4 && m.centroid[1] === 1,
               info: 'centroid=' + JSON.stringify(m.centroid) };
    });
    test('mirror reflects every poly vertex', function () {
      var m = mirror(fixtureStone(), 0);
      return { ok: JSON.stringify(m.poly) === JSON.stringify([[-1, 0], [-3, 0], [-3, 2], [-1, 2]]),
               info: 'poly=' + JSON.stringify(m.poly) };
    });
    test('mirror reflects both faces (pin+pout)', function () {
      var m = mirror(fixtureStone(), 0);
      return { ok: m.endFace.pin[0] === -3 && m.endFace.pout[0] === -3 &&
                   m.startFace.pin[0] === -1 && m.startFace.pout[0] === -1,
               info: 'endFace=' + JSON.stringify(m.endFace) };
    });
    test('mirror reflects the circle centre C', function () {
      var m = mirror(fixtureStone(), 3);
      return { ok: m.C[0] === 6 && m.C[1] === 5, info: 'C=' + JSON.stringify(m.C) };
    });
    test('mirror sets mirrored=true, keeps type/turn/area', function () {
      var m = mirror(fixtureStone(), 0);
      return { ok: m.mirrored === true && m.type === 'gelb' && m.turn === 60 && m.area === 12,
               info: 'mirrored=' + m.mirrored + ' type=' + m.type };
    });
    test('mirror keeps slices wound CCW after reflection', function () {
      var src = fixtureStone();
      var m = mirror(src, 0);
      return { ok: signedArea(src.slices[0]) > 0 && signedArea(m.slices[0]) > 0,
               info: 'srcA=' + signedArea(src.slices[0]).toFixed(2) + ' mA=' + signedArea(m.slices[0]).toFixed(2) };
    });

    // ---- closure(): missing = 180 - 2*sumDeg -> 0 or 1 valid closure ----
    test('closure: 1 gelb (60 deg half) -> gelb keystone (60 deg gap)', function () {
      var c = closure([{ type: 'gelb', inverted: false }]);
      return { ok: c && c.kind === 'keystone' && c.keyType === 'gelb', info: JSON.stringify(c) };
    });
    test('closure: gelb+orange (90 deg half) -> vertical joint, no keystone', function () {
      var c = closure([{ type: 'gelb', inverted: false }, { type: 'orange', inverted: false }]);
      return { ok: c && c.kind === 'joint' && c.keyType === null, info: JSON.stringify(c) };
    });
    test('closure: 2 gelb (120 deg) is over-built -> null', function () {
      var c = closure([{ type: 'gelb', inverted: false }, { type: 'gelb', inverted: false }]);
      return { ok: c === null, info: JSON.stringify(c) };
    });
    test('closure: 3 gruen (45 deg half) -> wuerfel keystone (90 deg gap)', function () {
      var c = closure([{ type: 'gruen', inverted: false }, { type: 'gruen', inverted: false }, { type: 'gruen', inverted: false }]);
      return { ok: c && c.kind === 'keystone' && c.keyType === 'wuerfel', info: JSON.stringify(c) };
    });
    test('closure: 1 orange (30 deg, gap 120) -> null (no single stone fits)', function () {
      var c = closure([{ type: 'orange', inverted: false }]);
      return { ok: c === null, info: JSON.stringify(c) };
    });

    var pass = 0;
    for (var i = 0; i < cases.length; i++) if (cases[i].ok) pass++;
    return { pass: pass, fail: cases.length - pass, total: cases.length, cases: cases };
  }

  /* ================= API ================= */

  var API2 = {
    selfTest2: selfTest2,
    // read-only introspection for tests/debugging (mirrors KajuBauprobe.probe)
    probe: function () {
      if (!state) return null;
      return {
        built: state.built.map(function (s) { return s.type; }),
        axisX: state.axisX, mirrored: state.mirrored, phase: state.phase,
        closure: closure(state.built), verdict: verdictEl ? verdictEl.textContent : ''
      };
    }
  };
  if (IS_BROWSER) window.KajuBauprobe2 = API2;
  else if (typeof module !== 'undefined' && module.exports) module.exports = API2;
})();
