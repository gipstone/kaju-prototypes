/* KajuBauprobe - the interactive heart of the experience.
   Phases: build (tap places stones, a drawn hand holds the flank)
           shape (drag adjusts curvature: flatter/steeper)
           hold  (press + release the hold button = letting go)
           verdict (static heuristic from Bauphysik.md: too flat -> thrust
                    pushes the feet apart; too steep -> tips inward)
           load  (marble on crown vs. flanks shows load distribution)
   All motion runs in one rAF loop that only ticks while visible. */
(function () {
  'use strict';

  var W = 820, H = 700;
  var groundY = 596;
  var baseR = 232, ringD = 86;
  var cx = W / 2;
  var COLORS = ['#6cac53', '#f1c953', '#f39200', '#f1c953', '#6cac53'];
  var ROT = '#a93015', PAPIER = '#faf7f4', BRAUN = '#532c29';
  var SEGS = [[180, 144], [144, 108], [108, 72], [72, 36], [36, 0]];
  var ORDER = ['pL', 'pR', 0, 4, 1, 3, 2]; // build order

  var canvas, ctx, taskEl, tipEl, holdBtn, holdLabel, resetBtn, verdictEl;
  var state;

  function deg(a) { return a * Math.PI / 180; }

  function geo(squash) {
    var rx = squash < 1 ? baseR * (1 + (1 - squash) * 0.72) : baseR * (1 - (squash - 1) * 0.28);
    var ry = baseR * squash;
    return { rx: rx, ry: ry, cy: groundY - ringD };
  }

  function ept(a, g, inner) {
    var rx = inner ? g.rx - ringD : g.rx;
    var ry = inner ? Math.max(30, g.ry - ringD) : g.ry;
    return [cx + Math.cos(deg(a)) * rx, g.cy - Math.sin(deg(a)) * ry];
  }

  // polygon (world coords) for a stone id at given squash
  function stonePoly(id, squash) {
    var g = geo(squash);
    var pts = [];
    if (id === 'pL' || id === 'pR') {
      var x0 = id === 'pL' ? cx - g.rx : cx + g.rx - ringD;
      pts = [[x0, groundY - ringD], [x0 + ringD, groundY - ringD], [x0 + ringD, groundY], [x0, groundY]];
    } else {
      var a0 = SEGS[id][0], a1 = SEGS[id][1], n = 7, i;
      for (i = 0; i <= n; i++) pts.push(ept(a0 + (a1 - a0) * i / n, g, false));
      for (i = n; i >= 0; i--) pts.push(ept(a0 + (a1 - a0) * i / n, g, true));
    }
    return pts;
  }

  function centroid(pts) {
    var x = 0, y = 0;
    pts.forEach(function (p) { x += p[0]; y += p[1]; });
    return [x / pts.length, y / pts.length];
  }

  function colorOf(id) { return id === 'pL' || id === 'pR' ? ROT : COLORS[id]; }

  /* ---------- drawing ---------- */

  function drawPoly(pts, fill, dx, dy, rot, pivot, alpha) {
    ctx.save();
    if (alpha != null) ctx.globalAlpha = alpha;
    if (dx || dy || rot) {
      ctx.translate((pivot ? pivot[0] : 0) + (dx || 0), (pivot ? pivot[1] : 0) + (dy || 0));
      ctx.rotate(rot || 0);
      ctx.translate(pivot ? -pivot[0] : 0, pivot ? -pivot[1] : 0);
    }
    // hard offset block shadow (brand depth)
    ctx.beginPath();
    pts.forEach(function (p, i) { i ? ctx.lineTo(p[0] + 5, p[1] + 6) : ctx.moveTo(p[0] + 5, p[1] + 6); });
    ctx.closePath();
    ctx.fillStyle = 'rgba(0,0,0,.34)';
    ctx.fill();
    ctx.beginPath();
    pts.forEach(function (p, i) { i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,244,232,.22)';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();
  }

  function drawHand(x, y, ang, scaleX, alpha) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(ang);
    ctx.scale(scaleX || 1, 1);
    ctx.globalAlpha = alpha == null ? 1 : alpha;
    ctx.beginPath(); // mitten silhouette
    ctx.moveTo(-26, 8);
    ctx.quadraticCurveTo(-34, -14, -12, -20);
    ctx.quadraticCurveTo(16, -26, 30, -10);
    ctx.quadraticCurveTo(40, 2, 30, 14);
    ctx.quadraticCurveTo(12, 26, -10, 20);
    ctx.quadraticCurveTo(-24, 17, -26, 8);
    ctx.closePath();
    ctx.fillStyle = PAPIER;
    ctx.fill();
    ctx.strokeStyle = BRAUN;
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.beginPath(); // thumb
    ctx.moveTo(-16, 4);
    ctx.quadraticCurveTo(-6, 16, 10, 12);
    ctx.strokeStyle = 'rgba(83,44,41,.5)';
    ctx.lineWidth = 2.5;
    ctx.stroke();
    ctx.restore();
  }

  function drawMarble(x, y, rCol) {
    ctx.save();
    ctx.beginPath();
    ctx.arc(x + 3, y + 4, 15, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0,0,0,.32)';
    ctx.fill();
    ctx.beginPath();
    ctx.arc(x, y, 15, 0, Math.PI * 2);
    ctx.fillStyle = rCol || '#0096ae';
    ctx.fill();
    ctx.beginPath();
    ctx.arc(x - 5, y - 5, 4.5, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,255,255,.85)';
    ctx.fill();
    ctx.restore();
  }

  /* ---------- state ---------- */

  function freshState() {
    return {
      phase: 'build',        // build | shape | armed | verdict-anim | stand | fail | done
      placed: 0,
      squash: 1,
      dragged: 0,
      holding: false,
      fly: null,             // {slot,t,from:[x,y],rot0}
      handOut: 0,            // 0 = hands on, 1 = hands gone
      settle: 0,             // settle wobble timer
      warn: 0,               // crown warning pulse
      glow: 0,               // flank glow pulse
      debris: null,          // [{pts(local),pos,rot,vx,vy,vr,col}]
      debrisT: 0,
      marbles: [],           // {x,y,vx,vy,rest:'crown'|'flank'|null,col}
      loadTop: false, loadFlank: false,
      pulse: 0,
    };
  }

  function setTask(html) { taskEl.innerHTML = html; }
  function setTip(html) {
    if (!html) { tipEl.classList.remove('is-on'); tipEl.innerHTML = ''; return; }
    tipEl.innerHTML = html;
    tipEl.classList.add('is-on');
  }

  function showVerdict(text, keep) {
    verdictEl.textContent = text;
    if (window.gsap) {
      gsap.killTweensOf(verdictEl);
      gsap.fromTo(verdictEl, { opacity: 0, y: 18, scale: .94 }, { opacity: 1, y: 0, scale: 1, duration: .5, ease: 'back.out(1.7)' });
      if (!keep) gsap.to(verdictEl, { opacity: 0, duration: .5, delay: 1.7 });
    } else {
      verdictEl.style.opacity = 1;
    }
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

  /* ---------- phase transitions ---------- */

  function place() {
    if (state.fly || state.placed >= ORDER.length) return;
    var slot = state.placed;
    var id = ORDER[slot];
    var from;
    if (id === 2) from = [cx, -80];
    else {
      var g = geo(state.squash);
      var leftish = (id === 'pL' || id === 0 || id === 1);
      from = [leftish ? -90 : W + 90, g.cy - 230];
    }
    state.fly = { slot: slot, t: 0, from: from, rot0: (from[0] < cx ? -1 : 1) * 0.45 };
    tok(210 + slot * 14, .07, .07);
  }

  function finishPlace() {
    state.placed++;
    state.fly = null;
    tok(150, .1, .09);
    if (state.placed >= ORDER.length) {
      state.phase = 'shape';
      setTask('Zieh am Bogen: nach unten wird er flacher, nach oben steiler. Wie weit traust Du Dich?');
    } else {
      var left = ORDER.length - state.placed;
      if (left === 1) setTask('Der Schlussstein fehlt. Tipp noch einmal.');
    }
  }

  function arm() {
    if (state.phase !== 'shape') return;
    state.phase = 'armed';
    holdBtn.disabled = false;
    setTask('Und jetzt der Moment der Wahrheit: <b>Halte den Knopf gedr&uuml;ckt</b> &ndash; und lass los, wenn Du dem Bogen traust.');
  }

  function release() {
    if (!state.holding) return;
    state.holding = false;
    holdBtn.classList.remove('is-holding');
    if (state.phase !== 'armed') return;
    state.phase = 'verdict-anim';
    holdBtn.disabled = true;
    state.handOut = 0.0001; // start hands-leave animation, verdict after
  }

  function verdict() {
    var s = state.squash;
    if (s >= 0.8 && s <= 1.22) {
      state.phase = 'stand';
      state.settle = 0.0001;
      showVerdict('Trägt.');
      tok(520, .16, .1);
      setTimeout(function () { tok(660, .2, .08); }, 110);
      setTask('Er steht &ndash; ganz ohne Kleber. Jetzt leg ihm was drauf: <b>Tipp auf den Scheitel</b>, danach <b>auf die Flanken</b>.');
      setTip('');
      holdBtn.hidden = true;
      resetBtn.hidden = false;
      resetBtn.textContent = 'Anders bauen';
    } else {
      collapse(s < 0.8 ? 'flach' : 'steil');
    }
  }

  function collapse(mode) {
    state.phase = 'fail';
    state.debrisT = 0;
    state.debris = ORDER.slice(0, state.placed).map(function (id, k) {
      var pts = stonePoly(id, state.squash);
      var c = centroid(pts);
      var local = pts.map(function (p) { return [p[0] - c[0], p[1] - c[1]]; });
      var vx, vy, vr;
      var upper = (id === 1 || id === 2 || id === 3);
      if (mode === 'flach') {
        var side = c[0] < cx ? -1 : (id === 2 ? (Math.random() < .5 ? -1 : 1) * .4 : 1);
        vx = side * (id === 'pL' || id === 'pR' ? 150 : 90) * (0.7 + Math.random() * .5);
        vy = upper ? -30 : 10;
        vr = side * (1.2 + Math.random());
      } else {
        var inw = c[0] < cx ? 1 : -1;
        vx = upper ? inw * 110 * (0.6 + Math.random() * .6) : inw * 20;
        vy = upper ? -60 : 0;
        vr = inw * (1.6 + Math.random());
      }
      return { local: local, pos: c, rot: 0, vx: vx, vy: vy, vr: vr, col: colorOf(id), h: Math.max.apply(null, local.map(function (p) { return p[1]; })) };
    });
    showVerdict('Rumms.', true);
    tok(90, .4, .16);
    setTask(mode === 'flach'
      ? 'Zu flach: Der <b>Bogenschub</b> hat die F&uuml;&szlig;e auseinandergedr&uuml;ckt.'
      : 'Zu steil eingew&ouml;lbt: Oben ist der Bogen <b>nach innen</b> gekippt.');
    setTip(mode === 'flach'
      ? '<b>Werkstatt-Tipp:</b> Je flacher der Bogen, desto st&auml;rker schiebt er seitlich nach au&szlig;en. Bau ihn runder &ndash; oder gib ihm griffigen Untergrund.'
      : '<b>Werkstatt-Tipp:</b> Zieh ihn etwas breiter. Ein Bogen tr&auml;gt, wenn die Drucklinie durch die Steine l&auml;uft &ndash; nicht daneben.');
    resetBtn.hidden = false;
    resetBtn.textContent = 'Nochmal bauen';
  }

  function reset() {
    // after the first full build the ritual is known: rebuild instantly
    var quick = state && (state.phase === 'fail' || state.phase === 'stand' || state.phase === 'done');
    state = freshState();
    holdLabel.textContent = 'Halten & loslassen';
    holdBtn.hidden = false;
    resetBtn.hidden = true;
    verdictEl.style.opacity = 0;
    setTip('');
    if (quick) {
      state.placed = ORDER.length;
      state.phase = 'shape';
      holdBtn.disabled = true;
      setTask('Die Steine stehen wieder. Zieh am Bogen: flacher oder steiler &ndash; und dann halten.');
    } else {
      holdBtn.disabled = true;
      setTask('Tipp auf die B&uuml;hne: Setz Stein f&uuml;r Stein.');
    }
  }

  /* ---------- load phase (marble) ---------- */

  function dropMarble(kind) {
    var g = geo(state.squash);
    if (kind === 'crown') {
      state.marbles.push({ x: cx, y: -24, vx: 0, vy: 0, rest: 'crown', col: '#0096ae' });
      tok(320, .07, .06);
    } else {
      state.marbles.push({ x: cx - g.rx * 0.62, y: -24, vx: 0, vy: 0, rest: 'flankL', col: '#a93015' });
      state.marbles.push({ x: cx + g.rx * 0.62, y: -40, vx: 0, vy: 0, rest: 'flankR', col: '#f39200' });
      tok(320, .07, .06);
    }
  }

  function handleLoadTap(x, y) {
    var g = geo(state.squash);
    var crown = [cx, g.cy - g.ry];
    var dc = Math.hypot(x - crown[0], y - crown[1]);
    var fl = [cx - g.rx * 0.66, g.cy - g.ry * 0.45];
    var fr = [cx + g.rx * 0.66, g.cy - g.ry * 0.45];
    var df = Math.min(Math.hypot(x - fl[0], y - fl[1]), Math.hypot(x - fr[0], y - fr[1]));
    if (dc < 110 && dc <= df && !state.loadTop) {
      state.loadTop = true;
      dropMarble('crown');
      setTimeout(function () {
        if (state.phase === 'stand') setTask('Siehst Du das Federn? <b>Volle Last auf dem Scheitel</b> dr&uuml;ckt den Bogen nach innen. Jetzt tipp auf eine Flanke.');
      }, 900);
    } else if (df < 130 && !state.loadFlank) {
      state.loadFlank = true;
      dropMarble('flank');
      if (!state.loadTop) setTimeout(function () {
        if (state.phase === 'stand') setTask('Die Murmeln bleiben liegen: <b>Last nah an den Flanken</b> stabilisiert. Jetzt tipp auf den Scheitel.');
      }, 900);
    }
    if (state.loadTop && state.loadFlank) {
      setTimeout(function () {
        state.phase = 'done';
        setTask('Oben dr&uuml;ckt die Last den Bogen nach innen &ndash; an den <b>Flanken</b> macht sie ihn sogar stabiler. Genau das findet Dein Kind selbst heraus.');
      }, 1600);
    }
  }

  /* ---------- input ---------- */

  function canvasPos(e) {
    var r = canvas.getBoundingClientRect();
    return [(e.clientX - r.left) * (W / r.width), (e.clientY - r.top) * (H / r.height)];
  }

  var drag = null;
  function onDown(e) {
    var p = canvasPos(e);
    if (state.phase === 'build') { place(); return; }
    if (state.phase === 'shape' || state.phase === 'armed') {
      drag = { y0: e.clientY, s0: state.squash };
      try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* synthetic pointer */ }
      return;
    }
    if (state.phase === 'stand') handleLoadTap(p[0], p[1]);
  }
  function onMove(e) {
    if (!drag) return;
    // normalize to on-screen finger travel: full stage height = full range
    var dy = e.clientY - drag.y0;
    var span = canvas.getBoundingClientRect().height || 300;
    state.squash = Math.max(0.55, Math.min(1.5, drag.s0 - (dy / span) * 1.35));
    state.dragged += Math.abs(dy);
    drag.y0 = e.clientY;
    drag.s0 = state.squash;
    if (state.dragged > 22 && state.phase === 'shape') arm();
  }
  function onUp() { drag = null; }

  /* ---------- main loop ---------- */

  var visible = false, lastT = 0;

  function tick(now) {
    requestAnimationFrame(tick);
    if (!visible || !state) return;
    var dt = Math.min(0.034, (now - lastT) / 1000 || 0.016);
    lastT = now;
    state.pulse += dt;

    // integrate fly-in
    if (state.fly) {
      state.fly.t += dt / 0.42;
      if (state.fly.t >= 1) finishPlace();
    }
    // hands leaving -> verdict
    if (state.phase === 'verdict-anim') {
      state.handOut = Math.min(1, state.handOut + dt / 0.38);
      if (state.handOut >= 1) verdict();
    }
    if (state.settle > 0) state.settle = Math.min(1, state.settle + dt / 0.5);
    if (state.warn > 0) state.warn = Math.max(0, state.warn - dt / 0.9);
    if (state.glow > 0) state.glow = Math.max(0, state.glow - dt / 1.1);

    // debris physics
    if (state.debris) {
      state.debrisT += dt;
      if (state.debrisT < 2.2) {
        state.debris.forEach(function (b) {
          b.vy += 1500 * dt;
          b.pos[0] += b.vx * dt;
          b.pos[1] += b.vy * dt;
          b.rot += b.vr * dt;
          var floor = groundY - b.h;
          if (b.pos[1] > floor) {
            b.pos[1] = floor;
            b.vy *= -0.16;
            b.vx *= 0.72;
            b.vr *= 0.5;
          }
        });
      }
    }

    // marbles
    var g = geo(state.squash);
    state.marbles.forEach(function (m) {
      if (m.done) return;
      m.vy += 1500 * dt;
      m.x += m.vx * dt;
      m.y += m.vy * dt;
      if (m.rest === 'crown') {
        var top = g.cy - g.ry - 15;
        if (m.y >= top) {
          m.y = top;
          if (!m.hit) {
            m.hit = true;
            state.warn = 1;
            tok(110, .2, .1);
            m.vx = (Math.random() < .5 ? -1 : 1) * 120; // rolls off
            m.rest = null;
            m.vy = -60;
          }
        }
      } else if (m.rest === 'flankL' || m.rest === 'flankR') {
        // land on the flank surface point
        var a = m.rest === 'flankL' ? 137 : 43;
        var s = ept(a, g, false);
        if (m.y >= s[1] - 13) {
          m.y = s[1] - 13;
          m.x = s[0];
          if (!m.hit) { m.hit = true; state.glow = 1; tok(480, .12, .07); }
          m.vx = 0; m.vy = 0; m.done = true;
        }
      } else {
        if (m.y > groundY - 15) {
          m.y = groundY - 15;
          m.vy *= -0.25;
          m.vx *= 0.9;
          if (Math.abs(m.vy) < 12) m.done = true;
        }
      }
    });

    draw();
  }

  function draw() {
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

    var g = geo(state.squash);
    var settleDy = state.settle > 0 ? Math.sin(Math.min(1, state.settle) * Math.PI) * 3 : 0;
    var warnSq = state.warn > 0 ? 1 - Math.sin(state.warn * Math.PI) * 0.035 : 1;

    if (state.debris) {
      // collapsed bodies
      state.debris.forEach(function (b) {
        ctx.save();
        ctx.translate(b.pos[0], b.pos[1]);
        ctx.rotate(b.rot);
        ctx.beginPath();
        b.local.forEach(function (p, i) { i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
        ctx.closePath();
        ctx.fillStyle = 'rgba(0,0,0,.3)';
        ctx.save(); ctx.translate(4, 5); ctx.fill(); ctx.restore();
        ctx.fillStyle = b.col;
        ctx.fill();
        ctx.strokeStyle = 'rgba(255,244,232,.2)';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.restore();
      });
    } else {
      // ghost target hint while building
      if (state.phase === 'build') {
        ctx.save();
        ctx.setLineDash([7, 8]);
        ctx.strokeStyle = 'rgba(245,236,226,.28)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.ellipse(cx, g.cy, g.rx - ringD / 2, g.ry - ringD / 2, 0, Math.PI, 0, false);
        ctx.stroke();
        ctx.restore();
      }

      // settled stones (apply crown warn squeeze on upper segs)
      for (var k = 0; k < state.placed; k++) {
        var id = ORDER[k];
        var pts = stonePoly(id, state.squash * (id === 1 || id === 2 || id === 3 ? warnSq : 1));
        drawPoly(pts, colorOf(id), 0, settleDy, 0, null, 1);
      }

      // flying stone
      if (state.fly) {
        var f = state.fly;
        var t = 1 - Math.pow(1 - Math.min(1, f.t), 3);
        var id2 = ORDER[f.slot];
        var pts2 = stonePoly(id2, state.squash);
        var c2 = centroid(pts2);
        var dx = (f.from[0] - c2[0]) * (1 - t);
        var dy = (f.from[1] - c2[1]) * (1 - t);
        drawPoly(pts2, colorOf(id2), dx, dy, f.rot0 * (1 - t), c2, 0.5 + 0.5 * t);
        // guiding hand rides along
        drawHand(c2[0] + dx + 34, c2[1] + dy + 26, 0.35, -1, 0.95);
      }

      // glow contour after flank load
      if (state.glow > 0) {
        ctx.save();
        ctx.globalAlpha = Math.sin(Math.min(1, state.glow) * Math.PI) * 0.85;
        ctx.strokeStyle = '#f1c953';
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.ellipse(cx, g.cy, g.rx + 8, g.ry + 8, 0, Math.PI, 0, false);
        ctx.stroke();
        ctx.restore();
      }

      // holding hands on both flanks (until they leave)
      var segsPlaced = state.placed > 2;
      var out = state.handOut;
      var handsVisible = !state.debris && segsPlaced && state.phase !== 'stand' && state.phase !== 'done' && out < 1 && !(state.phase === 'fail');
      if (handsVisible) {
        var squeeze = state.holding ? 9 : 0;
        var oL = ept(148, g, false), oR = ept(32, g, false);
        var ex = out * 240, ey = out * -100, fade = 1 - out;
        ctx.save();
        ctx.scale(1.28, 1.28);
        drawHand((oL[0] - 16 + squeeze - ex) / 1.28, (oL[1] + 10 + ey) / 1.28, -0.55, 1, fade);
        drawHand((oR[0] + 16 - squeeze + ex) / 1.28, (oR[1] + 10 + ey) / 1.28, 0.55, -1, fade);
        ctx.restore();
      }

      // load-phase affordance: pulsing tap targets on crown / flank
      if (state.phase === 'stand') {
        var pulse2 = 0.55 + Math.sin(state.pulse * 3) * 0.35;
        ctx.save();
        ctx.strokeStyle = 'rgba(245,236,226,.95)';
        ctx.fillStyle = 'rgba(245,236,226,.28)';
        ctx.lineWidth = 3;
        var targets = [];
        if (!state.loadTop) targets.push([cx, g.cy - g.ry - 26]);
        if (!state.loadFlank) targets.push(ept(140, g, false).map(function (v, i) { return i === 1 ? v - 18 : v - 14; }));
        targets.forEach(function (tp) {
          ctx.globalAlpha = pulse2;
          ctx.beginPath(); ctx.arc(tp[0], tp[1], 13, 0, Math.PI * 2); ctx.fill();
          ctx.beginPath(); ctx.arc(tp[0], tp[1], 20 + Math.sin(state.pulse * 3) * 4, 0, Math.PI * 2); ctx.stroke();
        });
        ctx.restore();
      }

      // shape-phase affordance: pulsing up/down arrows at the crown
      if (state.phase === 'shape' || state.phase === 'armed') {
        var crown = [cx, g.cy - g.ry];
        var bob = Math.sin(state.pulse * 2.6) * 6;
        ctx.save();
        ctx.strokeStyle = 'rgba(245,236,226,.9)';
        ctx.lineWidth = 5;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        [[-1, -46], [1, 24]].forEach(function (dir) {
          var yy = crown[1] + dir[1] + bob * dir[0] * -1;
          ctx.beginPath();
          ctx.moveTo(cx - 13, yy + 9 * dir[0]);
          ctx.lineTo(cx, yy);
          ctx.lineTo(cx + 13, yy + 9 * dir[0]);
          ctx.stroke();
        });
        ctx.restore();
      }
    }

    // marbles on top
    state.marbles.forEach(function (m) { drawMarble(m.x, m.y, m.col); });
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

    state = freshState();

    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
    window.addEventListener('pointercancel', onUp);

    holdBtn.addEventListener('pointerdown', function (e) {
      e.preventDefault();
      if (state.phase !== 'armed') return;
      state.holding = true;
      holdBtn.classList.add('is-holding');
      holdLabel.textContent = '… und loslassen!';
      try { holdBtn.setPointerCapture(e.pointerId); } catch (err) { /* synthetic pointer */ }
    });
    ['pointerup', 'pointercancel'].forEach(function (ev) {
      holdBtn.addEventListener(ev, function () {
        holdLabel.textContent = 'Halten & loslassen';
        release();
      });
    });
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
