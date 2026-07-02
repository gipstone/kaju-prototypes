/* KajuBauprobe v2 - physically honest arch building with the real kaju stone types.
   The user combines real ring-segment stones (gelb 60deg, orange 30deg, gruen 15deg,
   wuerfel 90deg cube). Every step places the stone mirrored on both sides at once;
   joints stay flush (Fuge an Fuge - shared joint points, tangential contact).
   Above the crown the missing angle is shown: 180deg minus the turn of both halves.
   When the gap exactly matches one stone type, that stone floats above the gap as
   a Schlussstein - one tap drops it in and closes the arch.
   Phases: build -> closed. One rAF loop that only ticks while visible. */
(function () {
  'use strict';

  var W = 820, H = 700;
  var groundY = 596;
  var cx = W / 2;
  var SCALE = 15;            // px per cm (Gelb outer radius 4 cm -> 60 px)
  var THICK = 2 * SCALE;     // every stone is 2 cm thick -> 30 px
  var MAX_STONES = 16;       // per half, sanity cap
  var PAPIER = '#f5ece2';

  var STONES = {
    gelb:    { angle: 60, outerR: 4,    innerR: 2,    color: '#f1c953', label: '60°' },
    orange:  { angle: 30, outerR: 8,    innerR: 6,    color: '#f39200', label: '30°' },
    gruen:   { angle: 15, outerR: 16,   innerR: 14,   color: '#6cac53', label: '15°' },
    wuerfel: { angle: 90, outerR: null, innerR: null, color: '#a93015', label: '90°' }
  };
  var PAL_ORDER = ['gelb', 'orange', 'gruen', 'wuerfel'];
  var PAL = { y: 610, w: 120, h: 82, gap: 14 };
  PAL.x0 = (W - (4 * PAL.w + 3 * PAL.gap)) / 2;

  var canvas, ctx, taskEl, tipEl, holdBtn, holdLabel, resetBtn, verdictEl;
  var state, layout, palItems;

  /* ---------- small math helpers ---------- */

  function rad(d) { return d * Math.PI / 180; }
  // canvas coords, y down; dirOf(theta): 180=left, 90=up, 0=right
  function dirOf(d) { return [Math.cos(rad(d)), -Math.sin(rad(d))]; }
  // forward tangent of the left half while theta decreases
  function tanOf(d) { return [Math.sin(rad(d)), Math.cos(rad(d))]; }
  function polarOfU(u) { return Math.atan2(-u[1], u[0]) * 180 / Math.PI; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1]]; }
  function mul(v, s) { return [v[0] * s, v[1] * s]; }
  function mirrorPt(p) { return [2 * cx - p[0], p[1]]; }
  function mirrorPoly(pts) { return pts.map(mirrorPt); }

  function centroid(pts) {
    var x = 0, y = 0;
    pts.forEach(function (p) { x += p[0]; y += p[1]; });
    return [x / pts.length, y / pts.length];
  }

  function polyMinY(pts) {
    var m = 1e9;
    pts.forEach(function (p) { if (p[1] < m) m = p[1]; });
    return m;
  }

  function inPoly(p, pts) {
    var c = false, i, j;
    for (i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      if (((pts[i][1] > p[1]) !== (pts[j][1] > p[1])) &&
          p[0] < (pts[j][0] - pts[i][0]) * (p[1] - pts[i][1]) / (pts[j][1] - pts[i][1]) + pts[i][0]) c = !c;
    }
    return c;
  }

  // ring segment outline: outer arc a0->a1 at Ra, inner arc back at Rb, around C
  function segPoly(C, Ra, Rb, a0, a1) {
    var n = Math.max(4, Math.round(Math.abs(a1 - a0) / 5)), pts = [], i, a;
    for (i = 0; i <= n; i++) {
      a = a0 + (a1 - a0) * i / n;
      pts.push([C[0] + Math.cos(rad(a)) * Ra, C[1] - Math.sin(rad(a)) * Ra]);
    }
    for (i = n; i >= 0; i--) {
      a = a0 + (a1 - a0) * i / n;
      pts.push([C[0] + Math.cos(rad(a)) * Rb, C[1] - Math.sin(rad(a)) * Rb]);
    }
    return pts;
  }

  /* ---------- arch layout (left half, joint walking) ----------
     Cursor = current joint: P (inner joint point) + u (unit vector inner->outer).
     Springer: u = dir(180), P = (cx - innerR of first stone, groundY).
     Normal segment: own centre C = P - Rin*u, sweeps theta0 -> theta0 - angle.
     Inverted segment: centre flips to the outside, C = P + (THICK+Rin)*u,
     sweeps phi0 -> phi0 + angle (arch-outer surface is now the SMALL radius).
     Cube: square set onto the joint face, turns the direction by 90deg
     (exit on the adjacent face) - the Spitzbogen corner piece.               */

  function computeLayout(stones) {
    var items = [], sum = 0, minY = groundY;
    var u = [-1, 0], P;
    var first = stones[0];
    var anchorR = first && STONES[first.type].innerR != null ? STONES[first.type].innerR * SCALE : THICK;
    P = [cx - anchorR, groundY];

    stones.forEach(function (s, i) {
      var t = STONES[s.type], poly, C, th0, th1;
      if (s.type === 'wuerfel') {
        var th = polarOfU(u), T = tanOf(th);
        var A = P.slice(), B = add(A, mul(u, THICK));
        var Cc = add(B, mul(T, THICK)), D = add(A, mul(T, THICK));
        poly = [A, B, Cc, D];
        if (!s.inverted) { sum += t.angle; u = T; P = B; }             // exit face B->Cc
        else             { sum -= t.angle; u = mul(T, -1); P = D; }   // exit face D->A
      } else {
        var Rin = t.innerR * SCALE, Rout = t.outerR * SCALE;
        if (!s.inverted) {
          C = add(P, mul(u, -Rin));
          th0 = polarOfU(u); th1 = th0 - t.angle;
          poly = segPoly(C, Rout, Rin, th0, th1);
          sum += t.angle;
          u = dirOf(th1); P = add(C, mul(dirOf(th1), Rin));
        } else {
          C = add(P, mul(u, THICK + Rin));
          th0 = polarOfU(mul(u, -1)); th1 = th0 + t.angle;
          poly = segPoly(C, Rin, Rout, th0, th1); // radii roles swap: small circle is the arch-outer skin
          sum -= t.angle;
          u = mul(dirOf(th1), -1); P = add(C, mul(dirOf(th1), Rout));
        }
      }
      var my = polyMinY(poly);
      if (my < minY) minY = my;
      items.push({ idx: i, type: s.type, inverted: s.inverted, born: s.born, poly: poly, polyR: mirrorPoly(poly) });
    });

    return { items: items, front: { P: P, u: u }, sum: sum, minY: minY };
  }

  // final geometry of a keystone of the given type, symmetric across the axis
  function keystoneGeom(typeKey) {
    var t = STONES[typeKey], f = layout.front;
    if (typeKey === 'wuerfel') {
      // diamond at the apex: both lower faces meet the 45deg fronts
      var A = [cx, f.P[1]], u135 = dirOf(135), T135 = tanOf(135);
      var B = add(A, mul(u135, THICK)), Cc = add(B, mul(T135, THICK)), D = add(A, mul(T135, THICK));
      return { poly: [A, B, Cc, D] };
    }
    var Rin = t.innerR * SCALE, Rout = t.outerR * SCALE;
    var Cfit = add(f.P, mul(f.u, -Rin));       // fitted to the left front...
    var Ck = [cx, Cfit[1]];                    // ...then clamped onto the symmetry axis
    return { poly: segPoly(Ck, Rout, Rin, 90 + t.angle / 2, 90 - t.angle / 2) };
  }

  /* ---------- state ---------- */

  function freshState() {
    return {
      phase: 'build',      // build | closed
      stones: [],          // left half: {type, inverted, born}
      offer: null,         // stone key currently floating as Schlussstein
      keystone: null,      // {typeKey, placing, done} once tapped
      sel: null,           // selected stone index (pair)
      selSide: -1,         // -1 = left visual tapped, +1 = right
      lastTap: 0,
      pulse: 0
    };
  }

  function missing() { return 180 - 2 * layout.sum; }

  function refresh() {
    layout = computeLayout(state.stones);
    if (state.phase === 'build') {
      var M = missing();
      state.offer = null;
      if (M === 0 && state.stones.length) {
        closeArch(null);
      } else if (M > 0) {
        PAL_ORDER.forEach(function (k) { if (STONES[k].angle === M) state.offer = k; });
      }
    }
    applyTexts();
  }

  /* ---------- texts ---------- */

  function setTask(html) { taskEl.innerHTML = html; }
  function setTip(html) {
    if (!html) { tipEl.classList.remove('is-on'); tipEl.innerHTML = ''; return; }
    tipEl.innerHTML = html;
    tipEl.classList.add('is-on');
  }

  function showVerdict(text) {
    verdictEl.textContent = text;
    if (window.gsap) {
      gsap.killTweensOf(verdictEl);
      gsap.fromTo(verdictEl, { opacity: 0, y: 18, scale: .94 }, { opacity: 1, y: 0, scale: 1, duration: .5, ease: 'back.out(1.7)' });
    } else {
      verdictEl.style.opacity = 1;
    }
  }

  function applyTexts() {
    if (state.phase === 'closed') {
      setTask('Der Bogen tr&auml;gt sich selbst &ndash; ohne Kleber, ohne Schrauben. Nur Druck, Fuge an Fuge.');
      setTip('<b>Werkstatt-Tipp:</b> Der Schlussstein verkeilt alle anderen. Erst mit ihm wird aus losen Steinen ein Tragwerk.');
      return;
    }
    var M = missing();
    if (!state.stones.length) {
      setTask('W&auml;hl unten einen Stein &ndash; er setzt sich links <b>und</b> rechts zugleich.');
      setTip('');
      return;
    }
    if (state.sel != null) {
      setTask('Stein markiert: Vorrat tauscht ihn, &#8635; dreht die Kr&uuml;mmung um, &#10005; nimmt ihn raus.');
      setTip('');
      return;
    }
    if (state.offer) {
      setTask('Der <b>Schlussstein</b> schwebt &uuml;ber der L&uuml;cke &ndash; tipp ihn an und schlie&szlig; den Bogen.');
      setTip('');
      return;
    }
    if (M < 0) {
      setTask('Mehr als 180&deg; verbaut &ndash; nimm einen Stein raus oder dreh einen zur&uuml;ck.');
      setTip('<b>Werkstatt-Tipp:</b> Ein gedrehter Stein (&#8635;) kr&uuml;mmt nach au&szlig;en &ndash; sein Winkel z&auml;hlt r&uuml;ckw&auml;rts.');
      return;
    }
    setTask('Fuge an Fuge &ndash; der Bogen w&auml;chst symmetrisch. Oben steht, was noch fehlt.');
    setTip('');
  }

  /* ---------- tiny tactile audio (quiet, gesture-gated) ---------- */

  var audio = null;
  function tok(freq, dur, gain) {
    try {
      audio = audio || new (window.AudioContext || window.webkitAudioContext)();
      if (audio.state === 'suspended') audio.resume();
      var o = audio.createOscillator(), g = audio.createGain();
      o.type = 'sine';
      o.frequency.value = freq;
      g.gain.setValueAtTime(gain || .1, audio.currentTime);
      g.gain.exponentialRampToValueAtTime(.0001, audio.currentTime + (dur || .09));
      o.connect(g).connect(audio.destination);
      o.start();
      o.stop(audio.currentTime + (dur || .09) + .02);
    } catch (e) { /* silent */ }
  }

  /* ---------- mutations ---------- */

  function addStone(typeKey) {
    if (state.stones.length >= MAX_STONES) return;
    state.stones.push({ type: typeKey, inverted: false, born: performance.now() });
    tok(190, .08, .08);
    setTimeout(function () { tok(150, .1, .07); }, 110);
    refresh();
  }

  function swapStone(idx, typeKey) {
    var s = state.stones[idx];
    if (!s || s.type === typeKey) return;
    s.type = typeKey;
    s.born = performance.now();
    tok(230, .07, .07);
    refresh();
  }

  function invertStone(idx) {
    var s = state.stones[idx];
    if (!s) return;
    s.inverted = !s.inverted;
    tok(260, .07, .07);
    refresh();
  }

  function removeStone(idx) {
    state.stones.splice(idx, 1);
    state.sel = null;
    tok(120, .09, .08);
    refresh();
  }

  function placeKeystone() {
    if (!state.offer) return;
    state.keystone = { typeKey: state.offer, placing: performance.now(), done: false };
    state.offer = null;
    state.sel = null;
    state.phase = 'closed';
    tok(320, .07, .07);
    applyTexts();
  }

  function closeArch(typeKey) {
    state.phase = 'closed';
    state.offer = null;
    state.sel = null;
    state.keystone = typeKey ? { typeKey: typeKey, placing: performance.now(), done: false } : null;
    if (!typeKey) celebrate();
  }

  function celebrate() {
    showVerdict('Trägt.');
    tok(520, .16, .1);
    setTimeout(function () { tok(660, .2, .08); }, 110);
    applyTexts();
  }

  function reset() {
    state = freshState();
    verdictEl.style.opacity = 0;
    if (window.gsap) gsap.killTweensOf(verdictEl);
    setTip('');
    refresh();
  }

  /* ---------- drawing ---------- */

  function drawPoly(pts, fill, alpha, dy, shadow) {
    ctx.save();
    if (alpha != null) ctx.globalAlpha = alpha;
    var oy = dy || 0;
    if (shadow !== false) {
      ctx.beginPath();
      pts.forEach(function (p, i) { i ? ctx.lineTo(p[0] + 5, p[1] + oy + 6) : ctx.moveTo(p[0] + 5, p[1] + oy + 6); });
      ctx.closePath();
      ctx.fillStyle = 'rgba(0,0,0,.34)';
      ctx.fill();
    }
    ctx.beginPath();
    pts.forEach(function (p, i) { i ? ctx.lineTo(p[0], p[1] + oy) : ctx.moveTo(p[0], p[1] + oy); });
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,244,232,.22)';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();
  }

  function strokePoly(pts, color, lw, alpha) {
    ctx.save();
    ctx.globalAlpha = alpha == null ? 1 : alpha;
    ctx.beginPath();
    pts.forEach(function (p, i) { i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
    ctx.closePath();
    ctx.strokeStyle = color;
    ctx.lineWidth = lw;
    ctx.lineJoin = 'round';
    ctx.stroke();
    ctx.restore();
  }

  function buildPalette() {
    palItems = PAL_ORDER.map(function (k, i) {
      var t = STONES[k];
      var x = PAL.x0 + i * (PAL.w + PAL.gap);
      var tcx = x + PAL.w / 2, tcy = PAL.y + 34;
      var poly;
      if (k === 'wuerfel') {
        poly = [[tcx - 15, tcy - 15], [tcx + 15, tcy - 15], [tcx + 15, tcy + 15], [tcx - 15, tcy + 15]];
      } else {
        // true-scale stone, crown up, top edge aligned across the palette
        var C = [tcx, tcy + t.outerR * SCALE - 15];
        poly = segPoly(C, t.outerR * SCALE, t.innerR * SCALE, 90 + t.angle / 2, 90 - t.angle / 2);
      }
      return { key: k, rect: [x, PAL.y, PAL.w, PAL.h], poly: poly, tcx: tcx, tcy: tcy };
    });
  }

  function rr(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  function drawPalette() {
    var hot = state.phase === 'build';
    palItems.forEach(function (it) {
      ctx.save();
      rr(it.rect[0], it.rect[1], it.rect[2], it.rect[3], 14);
      ctx.fillStyle = 'rgba(245,236,226,.06)';
      ctx.fill();
      ctx.strokeStyle = state.sel != null && hot ? 'rgba(241,201,83,.65)' : 'rgba(245,236,226,.16)';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.restore();
      drawPoly(it.poly, STONES[it.key].color, hot ? 1 : .35, 0, false);
      ctx.save();
      ctx.globalAlpha = hot ? .9 : .35;
      ctx.fillStyle = PAPIER;
      ctx.font = '600 15px "Ciutadella", system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(STONES[it.key].label, it.tcx, it.tcy + 38);
      ctx.restore();
    });
  }

  function easeOut(t) { return 1 - Math.pow(1 - Math.min(1, t), 3); }

  // ui button positions for the selected stone: [rotate, delete]
  function selButtons() {
    if (state.sel == null || !layout.items[state.sel]) return null;
    var it = layout.items[state.sel];
    var c = centroid(state.selSide > 0 ? it.polyR : it.poly);
    var by = Math.max(30, c[1] - 52);
    return [
      { x: Math.max(28, Math.min(W - 28, c[0] - 26)), y: by, glyph: '↻', act: 'invert' },
      { x: Math.max(28, Math.min(W - 28, c[0] + 26)), y: by, glyph: '✕', act: 'remove' }
    ];
  }

  function draw(now) {
    ctx.clearRect(0, 0, W, H);

    // warm stage light behind the arch
    var gr = ctx.createRadialGradient(cx, groundY - 200, 60, cx, groundY - 160, 460);
    gr.addColorStop(0, 'rgba(243,146,0,.12)');
    gr.addColorStop(1, 'rgba(243,146,0,0)');
    ctx.fillStyle = gr;
    ctx.fillRect(0, 0, W, H);

    // ground
    ctx.strokeStyle = 'rgba(245,236,226,.75)';
    ctx.lineWidth = 4;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(46, groundY + 2);
    ctx.lineTo(W - 46, groundY + 2);
    ctx.stroke();

    drawPalette();

    var frameMinY = state.stones.length ? layout.minY : groundY - 120;

    // stones, both halves (fly-in from the palette below)
    layout.items.forEach(function (it) {
      var col = STONES[it.type].color;
      var p = easeOut((now - it.born) / 380);
      var dy = (1 - p) * 150, al = .35 + .65 * p;
      drawPoly(it.poly, col, al, dy, p > .5);
      drawPoly(it.polyR, col, al, dy, p > .5);
      if (it.inverted) {
        ctx.save();
        ctx.globalAlpha = .85 * al;
        ctx.fillStyle = PAPIER;
        ctx.font = '600 15px "Ciutadella", system-ui, sans-serif';
        ctx.textAlign = 'center';
        var cL = centroid(it.poly), cR = centroid(it.polyR);
        ctx.fillText('↻', cL[0], cL[1] + dy + 5);
        ctx.fillText('↻', cR[0], cR[1] + dy + 5);
        ctx.restore();
      }
    });

    // placed keystone (drops in), or floating offer (bobs + pulses)
    if (state.keystone) {
      var kg = keystoneGeom(state.keystone.typeKey);
      var kp = easeOut((now - state.keystone.placing) / 320);
      drawPoly(kg.poly, STONES[state.keystone.typeKey].color, .6 + .4 * kp, (1 - kp) * -38, kp >= 1);
      if (kp >= 1 && !state.keystone.done) { state.keystone.done = true; celebrate(); }
      frameMinY = Math.min(frameMinY, polyMinY(kg.poly) - 44);
    } else if (state.offer) {
      var og = keystoneGeom(state.offer);
      var bob = Math.sin(state.pulse * 2.4) * 7;
      var oal = .55 + .18 * Math.sin(state.pulse * 3.2);
      drawPoly(og.poly, STONES[state.offer].color, oal, -38 + bob, false);
      strokePoly(og.poly.map(function (pt) { return [pt[0], pt[1] - 38 + bob]; }), 'rgba(245,236,226,.8)', 2, oal);
      frameMinY = Math.min(frameMinY, polyMinY(og.poly) - 38 + bob - 6);
    }

    // selection highlight + action buttons
    if (state.sel != null && layout.items[state.sel] && state.phase === 'build') {
      var sit = layout.items[state.sel];
      strokePoly(sit.poly, 'rgba(241,201,83,.4)', 7);
      strokePoly(sit.polyR, 'rgba(241,201,83,.4)', 7);
      strokePoly(sit.poly, PAPIER, 3);
      strokePoly(sit.polyR, PAPIER, 3);
      var btns = selButtons();
      if (btns) btns.forEach(function (b) {
        ctx.save();
        ctx.beginPath();
        ctx.arc(b.x, b.y, 17, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(20,16,14,.72)';
        ctx.fill();
        ctx.strokeStyle = 'rgba(245,236,226,.55)';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = PAPIER;
        ctx.font = '600 16px "Ciutadella", system-ui, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(b.glyph, b.x, b.y + 1);
        ctx.restore();
      });
    }

    // missing-angle display above the arch
    var M = missing();
    var txt, col;
    if (state.phase === 'closed') { txt = 'Bogen geschlossen ✓'; col = '#f1c953'; }
    else if (M >= 0) { txt = '∠ ' + M + '° fehlen'; col = 'rgba(245,236,226,.95)'; }
    else { txt = '∠ ' + (-M) + '° zu viel'; col = '#f39200'; }
    ctx.save();
    ctx.fillStyle = col;
    ctx.font = '700 30px "Ciutadella", system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(txt, cx, Math.max(50, frameMinY - 36));
    ctx.restore();
  }

  /* ---------- input ---------- */

  function canvasPos(e) {
    var r = canvas.getBoundingClientRect();
    return [(e.clientX - r.left) * (W / r.width), (e.clientY - r.top) * (H / r.height)];
  }

  function onDown(e) {
    var p = canvasPos(e), i;
    if (state.phase !== 'build') return;

    // 1) floating Schlussstein
    if (state.offer) {
      var oc = centroid(keystoneGeom(state.offer).poly);
      if (Math.hypot(p[0] - oc[0], p[1] - (oc[1] - 38)) < 60) { placeKeystone(); return; }
    }
    // 2) action buttons of the selected stone
    var btns = selButtons();
    if (btns) {
      for (i = 0; i < btns.length; i++) {
        if (Math.hypot(p[0] - btns[i].x, p[1] - btns[i].y) < 22) {
          if (btns[i].act === 'invert') invertStone(state.sel);
          else removeStone(state.sel);
          return;
        }
      }
    }
    // 3) palette: add new pair, or swap the selected stone
    for (i = 0; i < palItems.length; i++) {
      var r = palItems[i].rect;
      if (p[0] >= r[0] && p[0] <= r[0] + r[2] && p[1] >= r[1] && p[1] <= r[1] + r[3]) {
        if (state.sel != null) swapStone(state.sel, palItems[i].key);
        else addStone(palItems[i].key);
        return;
      }
    }
    // 4) stones in the arch (left half + mirrored right half)
    for (i = layout.items.length - 1; i >= 0; i--) {
      var it = layout.items[i];
      var side = inPoly(p, it.poly) ? -1 : (inPoly(p, it.polyR) ? 1 : 0);
      if (side) {
        var nowMs = performance.now();
        if (state.sel === i && nowMs - state.lastTap < 380) { removeStone(i); return; }
        state.sel = i;
        state.selSide = side;
        state.lastTap = nowMs;
        tok(320, .05, .05);
        applyTexts();
        return;
      }
    }
    // 5) empty space: clear selection
    if (state.sel != null) { state.sel = null; applyTexts(); }
  }

  /* ---------- main loop ---------- */

  var visible = false, lastT = 0;

  function tick(now) {
    requestAnimationFrame(tick);
    if (!visible || !state) return;
    var dt = Math.min(0.034, (now - lastT) / 1000 || 0.016);
    lastT = now;
    state.pulse += dt;
    draw(now);
  }

  /* ---------- init ---------- */

  function init() {
    canvas = document.getElementById('bp-canvas');
    if (!canvas) return;
    ctx = canvas.getContext('2d');
    taskEl = document.getElementById('bp-task');
    tipEl = document.getElementById('bp-tip');
    holdBtn = document.getElementById('bp-hold');
    holdLabel = document.getElementById('bp-hold-label');
    resetBtn = document.getElementById('bp-reset');
    verdictEl = document.getElementById('bp-verdict');

    if (holdBtn) { holdBtn.hidden = true; holdBtn.style.display = 'none'; } // .holdbtn CSS sets display:inline-flex, which beats [hidden]
    if (holdLabel) holdLabel.hidden = true;
    resetBtn.hidden = false;
    resetBtn.textContent = 'Von vorn';
    verdictEl.style.opacity = 0;

    buildPalette();
    state = freshState();
    refresh();

    canvas.addEventListener('pointerdown', onDown);
    resetBtn.addEventListener('click', reset);

    var io = new IntersectionObserver(function (entries) {
      visible = entries[0].isIntersecting;
      if (visible) lastT = performance.now();
    }, { threshold: 0.15 });
    io.observe(canvas);

    requestAnimationFrame(tick);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();

  window.KajuBauprobe = { reset: function () { reset(); } };
})();
