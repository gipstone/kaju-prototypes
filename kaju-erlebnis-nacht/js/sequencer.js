/* KajuSequencer - Apple-style scroll-scrubbed image sequence.
   Mechanics are production-ready: preloads numbered frames, draws the frame
   for a given progress onto a canvas. Swap placeholder frames for the real
   stop-motion series without touching this file (same naming scheme). */
(function () {
  'use strict';

  function create(opts) {
    var canvas = opts.canvas;
    var ctx = canvas.getContext('2d');
    var count = opts.count;
    var urls = [];
    for (var i = 1; i <= count; i++) {
      urls.push(opts.pattern.replace('{i}', String(i).padStart(2, '0')));
    }
    var images = new Array(count);
    var loaded = 0;
    var current = -1;
    var ready = false;

    function drawIndex(idx) {
      idx = Math.max(0, Math.min(count - 1, idx));
      // fall back to the nearest loaded frame so scrubbing never blanks
      var img = images[idx];
      if (!img || !img.complete) {
        for (var off = 1; off < count; off++) {
          var lo = images[idx - off], hi = images[idx + off];
          if (lo && lo.complete) { img = lo; break; }
          if (hi && hi.complete) { img = hi; break; }
        }
      }
      if (!img) return;
      if (idx === current) return;
      current = idx;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      if (opts.onFrame) opts.onFrame(idx, count);
    }

    function progress(p) {
      drawIndex(Math.round(p * (count - 1)));
    }

    // eager-load first frame, then the rest
    urls.forEach(function (u, i) {
      var img = new Image();
      img.onload = function () {
        loaded++;
        if (i === 0 && !ready) { ready = true; drawIndex(0); }
        if (loaded === count && opts.onReady) opts.onReady();
      };
      img.src = u;
      images[i] = img;
    });

    return { progress: progress, drawIndex: drawIndex, count: count };
  }

  window.KajuSequencer = { create: create };
})();
