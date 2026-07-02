/* kaju ERLEBNIS - orchestration: story bar, chapter pins, scrubbed
   sequences, block reveals. GSAP + ScrollTrigger (vendored). */
(function () {
  'use strict';

  var reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  if (reduced) document.documentElement.setAttribute('data-reduced', '');

  if (!window.gsap || !window.ScrollTrigger) return; // .blk stays visible via html.no-js
  document.documentElement.classList.remove('no-js');
  gsap.registerPlugin(ScrollTrigger);

  var chapters = Array.prototype.slice.call(document.querySelectorAll('.chapter'));

  /* ================= STORY BAR ================= */
  var segsWrap = document.getElementById('segs');
  var segLabel = document.getElementById('seglabel');
  var fills = [];

  chapters.forEach(function (ch, i) {
    var seg = document.createElement('button');
    seg.className = 'storybar__seg';
    seg.setAttribute('aria-label', 'Kapitel: ' + ch.dataset.title);
    var fill = document.createElement('i');
    fill.className = 'storybar__fill';
    seg.appendChild(fill);
    seg.addEventListener('click', function () {
      var wrap = wrapperOf(ch);
      var top = wrap.getBoundingClientRect().top + window.scrollY + 2;
      window.scrollTo({ top: top, behavior: reduced ? 'auto' : 'smooth' });
    });
    segsWrap.appendChild(seg);
    fills.push(fill);
  });

  function wrapperOf(ch) {
    var p = ch.parentElement;
    return (p && p.classList.contains('pin-spacer')) ? p : ch;
  }

  var activeIdx = -1;
  function updateBar() {
    var vh = window.innerHeight;
    var probe = vh * 0.5;
    for (var i = 0; i < chapters.length; i++) {
      var rect = wrapperOf(chapters[i]).getBoundingClientRect();
      var p = (probe - rect.top) / Math.max(1, rect.height);
      p = Math.max(0, Math.min(1, p));
      fills[i].style.transform = 'scaleX(' + p + ')';
      if (p > 0 && p < 1 && activeIdx !== i) {
        activeIdx = i;
        segLabel.textContent = chapters[i].dataset.title;
        document.documentElement.setAttribute('data-bar', chapters[i].dataset.bar || 'light');
      }
    }
  }
  ScrollTrigger.create({ trigger: document.body, start: 0, end: 'max', onUpdate: updateBar });
  window.addEventListener('resize', updateBar);

  /* ================= BLOCK REVEALS ================= */
  if (!reduced) {
    ScrollTrigger.batch('.chapter:not(.cover) .blk', {
      start: 'top 88%',
      once: true,
      onEnter: function (batch) {
        gsap.to(batch, { opacity: 1, y: 0, duration: 0.75, stagger: 0.09, ease: 'back.out(1.4)', overwrite: true });
      }
    });
    // cover intro on load
    gsap.to('.cover .blk', { opacity: 1, y: 0, duration: 0.8, stagger: 0.14, delay: 0.15, ease: 'back.out(1.3)' });
    gsap.from('#cover-arch', { y: -140, rotation: -9, opacity: 0, duration: 1.05, delay: 0.35, ease: 'back.out(1.35)' });
    gsap.to('#cover-arch', {
      yPercent: -14, ease: 'none',
      scrollTrigger: { trigger: '#k0', start: 'top top', end: 'bottom top', scrub: true }
    });
  } else {
    gsap.set('.blk', { opacity: 1, y: 0 });
  }

  /* ================= CH 1 · STEIN (tactile scrub) ================= */
  if (!reduced) {
    gsap.fromTo('#stein-img', { rotation: -7, scale: 0.95 }, {
      rotation: 6, scale: 1.03, ease: 'none',
      scrollTrigger: { trigger: '#k1', start: 'top bottom', end: 'bottom top', scrub: 0.6 }
    });
  }

  /* ================= CH 2 · WACHSEN (stop-motion scrub) ================= */
  var wCanvas = document.getElementById('wachsen-canvas');
  var wWord = document.getElementById('wachsen-word');
  var wCount = document.getElementById('wachsen-count');
  var WORDS = [
    { at: 0.03, text: 'Setzen.' },
    { at: 0.25, text: 'Halten.' },
    { at: 0.50, text: 'Umgreifen.' },
    { at: 0.70, text: 'Ausrichten.' },
    { at: 0.85, text: 'Loslassen.' }
  ];
  var curWord = -1;

  var seq = KajuSequencer.create({
    canvas: wCanvas,
    pattern: 'assets/frames/bau-{i}.jpg',
    count: 28,
    onFrame: function (idx, count) {
      wCount.textContent = String(idx + 1).padStart(2, '0') + '/' + count;
    }
  });

  function setWord(p) {
    var w = -1;
    for (var i = 0; i < WORDS.length; i++) if (p >= WORDS[i].at) w = i;
    if (w === curWord) return;
    curWord = w;
    if (w < 0) { gsap.to(wWord, { opacity: 0, duration: 0.2 }); return; }
    wWord.textContent = WORDS[w].text;
    gsap.fromTo(wWord, { opacity: 0, y: 12 }, { opacity: 1, y: 0, duration: 0.4, ease: 'power2.out' });
  }

  if (!reduced) {
    ScrollTrigger.create({
      trigger: '#k2',
      start: 'top top',
      end: '+=300%',
      pin: true,
      anticipatePin: 1,
      onUpdate: function (self) {
        seq.progress(self.progress);
        setWord(self.progress);
      }
    });
  } else {
    seq.drawIndex(27);
    wWord.style.opacity = 0;
  }

  /* ================= CH 4 · KRAFT (force flow scrub) ================= */
  var kCanvas = document.getElementById('kraft-canvas');
  var kCtx = kCanvas.getContext('2d');
  var kCap = document.getElementById('kraft-cap');
  var kProgress = 0, kVisible = false, kCapIdx = 0;
  var CAPS = [
    '<b>Jeder Stein dr&uuml;ckt auf den n&auml;chsten.</b> Druck h&auml;lt die Kette zusammen &ndash; Zug gibt es in einem Bogen nicht.',
    '<b>Die Last flie&szlig;t als Drucklinie</b> durch alle Steine &ndash; bis in die F&uuml;&szlig;e.',
    '<b>Unten schiebt der Bogen nach au&szlig;en:</b> der Bogenschub. Ein griffiger Untergrund h&auml;lt dagegen.',
    '<b>Stimmt das Zusammenspiel</b> aus Form, Auflager und Last, tr&auml;gt der Bogen sich selbst. Ganz ohne Kleber.'
  ];

  function kaftCaption(p) {
    var idx = p < 0.3 ? 0 : p < 0.6 ? 1 : p < 0.85 ? 2 : 3;
    if (idx === kCapIdx) return;
    kCapIdx = idx;
    gsap.fromTo(kCap, { opacity: 0, y: 8 }, { opacity: 1, y: 0, duration: 0.45, ease: 'power2.out' });
    kCap.innerHTML = CAPS[idx];
  }

  (function kraftLoop() {
    requestAnimationFrame(kraftLoop);
    if (!kVisible) return;
    KajuKraft.draw(kCtx, kProgress, performance.now() / 1000);
  })();

  var kIO = new IntersectionObserver(function (e) { kVisible = e[0].isIntersecting; }, { threshold: 0.1 });
  kIO.observe(kCanvas);

  if (!reduced) {
    ScrollTrigger.create({
      trigger: '#k4',
      start: 'top top',
      end: '+=240%',
      pin: true,
      anticipatePin: 1,
      onUpdate: function (self) {
        kProgress = self.progress;
        kaftCaption(self.progress);
      }
    });
  } else {
    kProgress = 1;
    kCapIdx = 3;
    kCap.innerHTML = CAPS[3];
    KajuKraft.draw(kCtx, 1, 0);
  }

  /* ================= CH 6 · KÖNNEN (horizontal scrub) ================= */
  var track = document.getElementById('koennen-track');
  var kBar = document.getElementById('koennen-bar');

  if (!reduced) {
    var dist = function () { return Math.max(0, track.scrollWidth - window.innerWidth); };
    gsap.to(track, {
      x: function () { return -dist(); },
      ease: 'none',
      scrollTrigger: {
        trigger: '#k6',
        start: 'top top',
        end: function () { return '+=' + Math.max(600, dist()); },
        pin: true,
        anticipatePin: 1,
        scrub: 0.5,
        invalidateOnRefresh: true,
        onUpdate: function (self) {
          kBar.style.transform = 'scaleX(' + self.progress + ')';
        }
      }
    });
  } else {
    document.getElementById('koennen-viewport').style.overflowX = 'auto';
    kBar.style.transform = 'scaleX(1)';
  }

  /* ================= FINALE ================= */
  var again = document.getElementById('again-btn');
  if (again) again.addEventListener('click', function () {
    window.scrollTo({ top: 0, behavior: reduced ? 'auto' : 'smooth' });
  });

  window.addEventListener('load', function () {
    ScrollTrigger.refresh();
    updateBar();
  });
  updateBar();
})();
