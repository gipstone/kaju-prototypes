/* KajuKraft - "why does it stand" force-flow rendering on the turquoise
   chapter. Scroll progress drives four phases:
   1: stones + joint pressure pairs  2: thrust line flows through the ring
   3: outward thrust at the feet + ground grip  4: whole picture, calm.
   A gentle rAF clock animates flow particles while the chapter is visible. */
(function () {
  'use strict';

  var W = 820, H = 760;
  var cx = W / 2, groundY = 640;
  var R = 296, r = 188, d = R - r;
  var cy = groundY - d;
  var COLORS = ['#6cac53', '#f1c953', '#f39200', '#f1c953', '#6cac53'];
  var ROT = '#a93015';
  var SEGS = [[180, 144], [144, 108], [108, 72], [72, 36], [36, 0]];

  function deg(a) { return a * Math.PI / 180; }
  function pt(a, rad) { return [cx + Math.cos(deg(a)) * rad, cy - Math.sin(deg(a)) * rad]; }

  function voussoirPath(ctx, a0, a1) {
    ctx.beginPath();
    ctx.arc(cx, cy, R, -deg(a0), -deg(a1), false);
    var p1 = pt(a1, r);
    ctx.lineTo(p1[0], p1[1]);
    ctx.arc(cx, cy, r, -deg(a1), -deg(a0), true);
    ctx.closePath();
  }

  function arrow(ctx, x, y, ang, len, w) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(ang);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(len, 0);
    ctx.moveTo(len - w * 1.4, -w);
    ctx.lineTo(len, 0);
    ctx.lineTo(len - w * 1.4, w);
    ctx.stroke();
    ctx.restore();
  }

  // phase helper: 0..1 inside [a,b]
  function ph(p, a, b) { return Math.max(0, Math.min(1, (p - a) / (b - a))); }

  function draw(ctx, p, t) {
    ctx.clearRect(0, 0, W, H);

    var white = '#ffffff';
    var soft = 'rgba(255,255,255,.55)';

    // ground
    ctx.strokeStyle = 'rgba(255,255,255,.8)';
    ctx.lineWidth = 4;
    ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(60, groundY + 2); ctx.lineTo(W - 60, groundY + 2); ctx.stroke();

    // plinths + stones, solid brand colors with white contour
    var appear = ph(p, 0, 0.14);
    ctx.save();
    ctx.globalAlpha = 0.25 + 0.75 * appear;
    [-1, 1].forEach(function (s) {
      var x = s < 0 ? cx - R : cx + R - d;
      ctx.fillStyle = ROT;
      ctx.fillRect(x, groundY - d, d, d);
      ctx.strokeStyle = 'rgba(255,255,255,.85)';
      ctx.lineWidth = 2.5;
      ctx.strokeRect(x, groundY - d, d, d);
    });
    SEGS.forEach(function (sa, i) {
      voussoirPath(ctx, sa[0], sa[1]);
      ctx.fillStyle = COLORS[i];
      ctx.fill();
      ctx.strokeStyle = 'rgba(255,255,255,.85)';
      ctx.lineWidth = 2.5;
      ctx.stroke();
    });
    ctx.restore();

    // PHASE 1: pressure pairs at the joints (arrows pressing together)
    var p1 = ph(p, 0.05, 0.3) * (1 - ph(p, 0.82, 0.95) * 0.4);
    if (p1 > 0) {
      ctx.save();
      ctx.globalAlpha = p1;
      ctx.strokeStyle = white;
      ctx.lineWidth = 5;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      var pump = 1 + Math.sin(t * 2.4) * 0.10; // gentle breathing
      [144, 108, 72, 36].forEach(function (a) {
        var mid = (R + r) / 2;
        var m = pt(a, mid);
        var ang = -deg(a); // tangent direction along the ring
        var gap = 26 * pump;
        arrow(ctx, m[0] - Math.cos(ang) * gap, m[1] - Math.sin(ang) * gap, ang, 17, 6);
        arrow(ctx, m[0] + Math.cos(ang) * gap, m[1] + Math.sin(ang) * gap, ang + Math.PI, 17, 6);
      });
      ctx.restore();
    }

    // PHASE 2: thrust line flowing from crown to both feet
    var p2 = ph(p, 0.3, 0.62);
    if (p2 > 0) {
      ctx.save();
      var mid = (R + r) / 2;
      ctx.strokeStyle = white;
      ctx.lineWidth = 7;
      ctx.lineCap = 'round';
      ctx.setLineDash([]);
      // two symmetric line halves growing downwards from 90deg
      [[90, 180], [90, 0]].forEach(function (range) {
        var a0 = range[0], a1 = range[0] + (range[1] - range[0]) * p2;
        ctx.beginPath();
        var steps = 40;
        for (var i = 0; i <= steps; i++) {
          var a = a0 + (a1 - a0) * (i / steps);
          var q = pt(a, mid);
          if (i === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]);
        }
        ctx.stroke();
        // continue into the ground when line reaches the foot
        if (p2 > 0.92) {
          var f = pt(range[1], mid);
          ctx.beginPath();
          ctx.moveTo(f[0], f[1]);
          ctx.lineTo(f[0], groundY + 26);
          ctx.stroke();
        }
      });
      // flow particles riding the line
      ctx.fillStyle = white;
      for (var k = 0; k < 7; k++) {
        var tt = ((t * 0.14 + k / 7) % 1) * p2;
        [[90, 180], [90, 0]].forEach(function (range) {
          var a = range[0] + (range[1] - range[0]) * tt;
          var q = pt(a, mid);
          ctx.beginPath();
          ctx.arc(q[0], q[1], 5, 0, Math.PI * 2);
          ctx.fill();
        });
      }
      ctx.restore();
    }

    // PHASE 3: outward thrust at the feet + ground resistance
    var p3 = ph(p, 0.6, 0.85);
    if (p3 > 0) {
      ctx.save();
      ctx.globalAlpha = p3;
      ctx.strokeStyle = '#f1c953';
      ctx.lineWidth = 6;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      var push = Math.sin(t * 2) * 4;
      [-1, 1].forEach(function (s) {
        var fx = s < 0 ? cx - R + d / 2 : cx + R - d / 2;
        arrow(ctx, fx + s * (34 + push), groundY - d / 2, s < 0 ? Math.PI : 0, 34, 8);
        // ground grip arrows answer inwards
        arrow(ctx, fx + s * (86 + push), groundY + 30, s < 0 ? 0 : Math.PI, 26, 7);
      });
      ctx.fillStyle = '#f1c953';
      ctx.font = '600 15px Ciutadella, sans-serif';
      ctx.restore();
    }

    // PHASE 4: calm glow contour
    var p4 = ph(p, 0.85, 1);
    if (p4 > 0) {
      ctx.save();
      ctx.globalAlpha = p4 * 0.9;
      ctx.strokeStyle = white;
      ctx.lineWidth = 3.5;
      ctx.beginPath(); ctx.arc(cx, cy, R + 7, Math.PI, 0, false); ctx.stroke();
      ctx.restore();
    }
  }

  window.KajuKraft = { draw: draw, W: W, H: H };
})();
