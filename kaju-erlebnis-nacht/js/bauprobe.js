/* KajuBauprobe v3 - arch building with true rigid-block statics and collapse.

   The user combines real ring-segment stones (gelb 60deg R2-4, orange 30deg R6-8,
   gruen 15deg R14-16, wuerfel 2x2 corner cube). Building stays symmetric: every
   placed stone appears mirrored on both sides; a fitting keystone closes the arch
   as a SINGLE stone. Once closed, both legs merge into one contact chain.
   "Bogen testen" runs a real limit-state analysis (spec BAUPROBE-PHYSIK):

   - every stone is a rigid 2D body with exact area + centroid (annular sector math)
   - every joint transmits only compression (N>=0) and Coulomb friction (|T|<=mu*N)
   - a two-phase simplex LP checks force+moment equilibrium for EVERY stone at once
     (feasibility = "some admissible thrust line exists", not one prescribed curve)
   - binary search maximises the joint-edge margin alpha, then a deterministic
     objective (min sum N + 0.05 sum |T|) picks one thrust state -> pressure points
   - infeasible with friction but feasible without = sliding failure diagnosis
   - unstable arches collapse via a built-in rigid-body engine (convex 5deg slices,
     SAT + sequential impulses, fixed 1/120s step, deterministic imperfection)

   Deviations from the physics spec, forced by the single-file/ES5 rule of this
   prototype (no npm, no new files, no HTML changes):
   - HiGHS-WASM  -> hand-written dense two-phase simplex (the LP has <200 columns)
   - planck.js   -> built-in impulse solver with the spec's parameters
   - Web Worker  -> synchronous solve, but only on the explicit test button
   The model itself (endpoint forces per joint, alpha margin, hinge/slide flags,
   no tension, no artificial joints) follows the spec exactly.                     */
(function () {
  'use strict';

  var IS_BROWSER = typeof window !== 'undefined' && typeof document !== 'undefined';
  var NOW = function () {
    return (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
  };

  /* ================= constants ================= */

  var W = 820, H = 700;
  var GY = 538;                       // screen y of the ground line (world y = 0)
  var STAGE = { x0: 40, x1: 780, top: 104 };
  var ZMAX = 34, ZMIN = 8;            // auto-zoom bounds (px per cm)
  var PAPIER = '#f5ece2';
  var MAX_STONES = 16;                // per half

  var CLOSURE_TOL_DEG = 0.01;         // spec 4.5
  var CONTACT_TOL_CM = 0.02;          // spec 6.3 (0.2 mm) - fugengenau chain steps
  // Real stones are forgiving: edges are rounded and every joint has ~0.2mm
  // play, so a chain of stones can flex a few millimetres. Sharp polygon
  // corners + rigid chains overestimate clashes, so buildability checks use
  // looser limits than the exact chain construction (Kai: "in echt kann man
  // das problemlos bauen").
  var OVERLAP_TOL_CM = 0.35;          // allowed interpenetration before 'overlap'
  var JOIN_TOL_CM = 0.45;             // branch feet / meeting fronts count as joined
  var LEN_SCALE = 10;                 // spec 7.5: solver works in dm

  // spec 12 - development values, NOT calibrated against real stones yet
  var calibration = {
    calibrated: false,
    stoneStoneMu: 0.50,
    stoneGroundMu: 0.50
  };
  // spec 3.3 - set real measured grams here to override area-proportional masses
  var massCalibration = { gelbGrams: null, orangeGrams: null, gruenGrams: null, wuerfelGrams: null,
                          wgelbGrams: null, worangeGrams: null };

  // Weichensteine (wgelb/worange): one entry face, two exit faces - the union of
  // a segment and its mirror twin laid Fuge auf Fuge (Kai's construction rule).
  var STONES = {
    gelb:    { angle: 60, Ri: 2,  Ra: 4,  color: '#f1c953', label: '60°' },
    orange:  { angle: 30, Ri: 6,  Ra: 8,  color: '#f39200', label: '30°' },
    gruen:   { angle: 15, Ri: 14, Ra: 16, color: '#6cac53', label: '15°' },
    wuerfel: { angle: 90, Ri: null, Ra: null, color: '#a93015', label: '90°' },
    wgelb:   { angle: 60, Ri: 2,  Ra: 4,  color: '#f1c953', label: '±60°', sw: true, base: 'gelb' },
    worange: { angle: 30, Ri: 6,  Ra: 8,  color: '#f39200', label: '±30°', sw: true, base: 'orange' }
  };
  var PAL_ORDER = ['gelb', 'orange', 'gruen', 'wuerfel', 'wgelb', 'worange'];

  var PAL = { y: 554, w: 100, h: 72, gap: 10 };
  PAL.x0 = (W - (6 * PAL.w + 5 * PAL.gap)) / 2;
  var BAR = { y: 640, h: 46 };        // action buttons live BELOW the palette

  /* ================= small math ================= */

  function rad(d) { return d * Math.PI / 180; }
  function deg(r) { return r * 180 / Math.PI; }
  function v2(x, y) { return [x, y]; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1]]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1]]; }
  function mul(a, s) { return [a[0] * s, a[1] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1]; }
  function cross(a, b) { return a[0] * b[1] - a[1] * b[0]; }
  function len(a) { return Math.sqrt(a[0] * a[0] + a[1] * a[1]); }
  function dist(a, b) { return len(sub(a, b)); }
  function norm(a) { var l = len(a); return l > 0 ? [a[0] / l, a[1] / l] : [0, 0]; }
  function perp(a) { return [-a[1], a[0]]; }
  function edir(phi) { return [Math.cos(phi), Math.sin(phi)]; }
  function mirX(p) { return [-p[0], p[1]]; }
  function mirFace(f) { return { pin: mirX(f.pin), pout: mirX(f.pout) }; }
  function clamp(x, a, b) { return x < a ? a : (x > b ? b : x); }

  function polySignedArea(pts) {
    var s = 0, i, j;
    for (i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      s += pts[j][0] * pts[i][1] - pts[i][0] * pts[j][1];
    }
    return s / 2;
  }
  function ensureCCW(pts) {
    return polySignedArea(pts) < 0 ? pts.slice().reverse() : pts;
  }
  function polyCentroidAvg(pts) {
    var x = 0, y = 0;
    for (var i = 0; i < pts.length; i++) { x += pts[i][0]; y += pts[i][1]; }
    return [x / pts.length, y / pts.length];
  }
  // area-weighted centroid, valid for any simple (also concave) polygon
  function polyCentroid(pts) {
    var A = 0, cx = 0, cy = 0, i, j, cr;
    for (i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      cr = pts[j][0] * pts[i][1] - pts[i][0] * pts[j][1];
      A += cr;
      cx += (pts[j][0] + pts[i][0]) * cr;
      cy += (pts[j][1] + pts[i][1]) * cr;
    }
    A /= 2;
    return [cx / (6 * A), cy / (6 * A)];
  }
  function keyStr(chain) { return chain.join('.'); }
  function inPoly(p, pts) {
    var c = false, i, j;
    for (i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      if (((pts[i][1] > p[1]) !== (pts[j][1] > p[1])) &&
          p[0] < (pts[j][0] - pts[i][0]) * (p[1] - pts[i][1]) / (pts[j][1] - pts[i][1]) + pts[i][0]) c = !c;
    }
    return c;
  }
  function polyAABB(pts) {
    var x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9, i;
    for (i = 0; i < pts.length; i++) {
      if (pts[i][0] < x0) x0 = pts[i][0];
      if (pts[i][0] > x1) x1 = pts[i][0];
      if (pts[i][1] < y0) y0 = pts[i][1];
      if (pts[i][1] > y1) y1 = pts[i][1];
    }
    return [x0, y0, x1, y1];
  }

  /* ================= stone geometry (exact, cm, y up) =================
     Annular sector: A = theta/2 (Ra^2-Ri^2)
     centroid radius r_S = 4 sin(theta/2)/(3 theta) * (Ra^3-Ri^3)/(Ra^2-Ri^2)   */

  function stoneArea(type) {
    var t = STONES[type];
    if (type === 'wuerfel') return 4;
    return rad(t.angle) / 2 * (t.Ra * t.Ra - t.Ri * t.Ri);
  }
  function stoneCentroidRadius(type) {
    var t = STONES[type], th = rad(t.angle);
    return 4 * Math.sin(th / 2) / (3 * th) *
      (Math.pow(t.Ra, 3) - Math.pow(t.Ri, 3)) / (t.Ra * t.Ra - t.Ri * t.Ri);
  }
  function stoneMass(type) {
    var g = { gelb: massCalibration.gelbGrams, orange: massCalibration.orangeGrams,
              gruen: massCalibration.gruenGrams, wuerfel: massCalibration.wuerfelGrams }[type];
    return (g != null && g > 0) ? g : stoneArea(type);   // relative mass, spec 3.3
  }

  // faceted outline (5deg steps - matches the convex collision decomposition)
  function sectorPoly(C, Ri, Ra, ph0, ph1) {
    var n = Math.max(1, Math.round(Math.abs(deg(ph1 - ph0)) / 5)), pts = [], i, a;
    for (i = 0; i <= n; i++) {
      a = ph0 + (ph1 - ph0) * i / n;
      pts.push([C[0] + Math.cos(a) * Ra, C[1] + Math.sin(a) * Ra]);
    }
    for (i = n; i >= 0; i--) {
      a = ph0 + (ph1 - ph0) * i / n;
      pts.push([C[0] + Math.cos(a) * Ri, C[1] + Math.sin(a) * Ri]);
    }
    return pts;
  }
  // convex quads (max 5deg each) for collision / overlap tests, spec 11.3
  function sectorSlices(C, Ri, Ra, ph0, ph1) {
    var n = Math.max(1, Math.round(Math.abs(deg(ph1 - ph0)) / 5)), out = [], i, a, b;
    for (i = 0; i < n; i++) {
      a = ph0 + (ph1 - ph0) * i / n;
      b = ph0 + (ph1 - ph0) * (i + 1) / n;
      out.push(ensureCCW([
        [C[0] + Math.cos(a) * Ri, C[1] + Math.sin(a) * Ri],
        [C[0] + Math.cos(a) * Ra, C[1] + Math.sin(a) * Ra],
        [C[0] + Math.cos(b) * Ra, C[1] + Math.sin(b) * Ra],
        [C[0] + Math.cos(b) * Ri, C[1] + Math.sin(b) * Ri]
      ]));
    }
    return out;
  }

  /* ================= chain walker =================
     Left leg in world cm, ground y=0, mirror axis x=0, y up.
     Cursor = current joint face: Pin (arch-inner point) + u (unit inner->outer).
     Springer: Pin=(-anchorR,0), u=(-1,0). A normal stone turns the face CW
     (contributes +angle towards closure), an inverted stone (curvature out)
     turns CCW and contributes -angle. Faces stay exactly 2cm - fugengenau.     */

  function stepStone(cur, type, inverted) {
    var t = STONES[type];
    var Pin = cur.Pin, u = cur.u;
    var item = {
      type: type, inverted: !!inverted,
      area: stoneArea(type), massRel: stoneMass(type)
    };
    var next;
    if (type === 'wuerfel') {
      var T = [u[1], -u[0]];                       // travel direction (u rotated -90)
      var A = Pin.slice(), B = add(Pin, mul(u, 2));
      var C2 = add(B, mul(T, 2)), D = add(A, mul(T, 2));
      item.poly = [A, B, C2, D];
      item.slices = [ensureCCW([A, B, C2, D])];
      item.centroid = polyCentroidAvg(item.poly);
      item.startFace = { pin: A, pout: B };
      if (!inverted) {
        item.endFace = { pin: B, pout: C2 };
        item.turn = 90;
        next = { Pin: B, u: T };
      } else {
        item.endFace = { pin: D, pout: A };
        item.turn = -90;
        next = { Pin: D, u: mul(T, -1) };
      }
    } else {
      var Ri = t.Ri, Ra = t.Ra, th = rad(t.angle);
      var C, ph0, ph1, e1, rs = stoneCentroidRadius(type);
      item.startFace = { pin: Pin.slice(), pout: add(Pin, mul(u, 2)) };
      if (!inverted) {
        C = add(Pin, mul(u, -Ri));                 // own circle centre on the inside
        ph0 = Math.atan2(u[1], u[0]);
        ph1 = ph0 - th;                            // CW sweep
        e1 = edir(ph1);
        item.endFace = { pin: add(C, mul(e1, Ri)), pout: add(C, mul(e1, Ra)) };
        item.turn = t.angle;
        next = { Pin: item.endFace.pin, u: e1 };
      } else {
        C = add(Pin, mul(u, 2 + Ri));              // centre flips to the outside
        ph0 = Math.atan2(-u[1], -u[0]);
        ph1 = ph0 + th;                            // CCW sweep, curvature out
        e1 = edir(ph1);
        item.endFace = { pin: add(C, mul(e1, Ra)), pout: add(C, mul(e1, Ri)) };
        item.turn = -t.angle;
        next = { Pin: item.endFace.pin, u: mul(e1, -1) };
      }
      item.poly = sectorPoly(C, Ri, Ra, ph0, ph1);
      item.slices = sectorSlices(C, Ri, Ra, ph0, ph1);
      item.centroid = add(C, mul(edir((ph0 + ph1) / 2), rs));
      item.C = C;
    }
    return { item: item, cur: next };
  }

  // faceted arc from Pfrom to Pto around C (shortest sweep, <=5deg facets);
  // Pfrom is assumed to be in the list already
  function arcAppend(list, C, r, Pfrom, Pto) {
    var a0 = Math.atan2(Pfrom[1] - C[1], Pfrom[0] - C[0]);
    var a1 = Math.atan2(Pto[1] - C[1], Pto[0] - C[0]);
    var d = a1 - a0;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    var n = Math.max(1, Math.ceil(Math.abs(deg(d)) / 5)), i;
    for (i = 1; i < n; i++) {
      list.push([C[0] + Math.cos(a0 + d * i / n) * r, C[1] + Math.sin(a0 + d * i / n) * r]);
    }
    list.push(Pto.slice());
  }

  function normDeg(d) {
    while (d > 180) d -= 360;
    while (d <= -180) d += 360;
    return d;
  }
  function faceDirDeg(face) {
    return deg(Math.atan2(face.pout[1] - face.pin[1], face.pout[0] - face.pin[0]));
  }
  // rigid motion mapping segment a1->a2 onto b1->b2 (equal lengths)
  function makeRigid(a1, a2, b1, b2) {
    var ang = Math.atan2(b2[1] - b1[1], b2[0] - b1[0]) - Math.atan2(a2[1] - a1[1], a2[0] - a1[0]);
    var cs = Math.cos(ang), sn = Math.sin(ang);
    return function (p) {
      var dx = p[0] - a1[0], dy = p[1] - a1[1];
      return [b1[0] + dx * cs - dy * sn, b1[1] + dx * sn + dy * cs];
    };
  }

  /* Weichenstein: union of a segment and its mirror twin sharing the entry face
     (Kai: "eins gespiegelt, Fuge auf Fuge"). One entry, two exits at +-angle.
     rot = which face sits on the parent joint (the stone REALLY turns to the
     next Fugenpassung): 0 = entry face, 1 = arm N, 2 = arm I.                 */
  function makeSwitchItem(cur, type, rot) {
    rot = rot || 0;
    var t = STONES[type];
    var N = stepStone(cur, t.base, false);
    var I = stepStone(cur, t.base, true);
    var Pin = cur.Pin.slice(), Pout = add(cur.Pin, mul(cur.u, 2));
    var T = [cur.u[1], -cur.u[0]];                 // travel direction
    var faceMid = add(Pin, cur.u);
    var CN = N.item.C, CI = I.item.C, Ra = t.Ra;
    var dvec = sub(CI, CN), d = len(dvec), dirC = mul(dvec, 1 / d);
    var h = Math.sqrt(Math.max(0, Ra * Ra - d * d / 4));
    var midC = mul(add(CN, CI), 0.5);
    var X1 = add(midC, mul(perp(dirC), h));
    var X = dot(sub(X1, faceMid), T) > 0 ? X1 : add(midC, mul(perp(dirC), -h));

    var pts = [Pin.slice()];
    arcAppend(pts, CN, t.Ri, Pin, N.item.endFace.pin);
    pts.push(N.item.endFace.pout.slice());
    arcAppend(pts, CN, Ra, N.item.endFace.pout, X);
    arcAppend(pts, CI, Ra, X, I.item.endFace.pin);
    pts.push(I.item.endFace.pout.slice());
    arcAppend(pts, CI, t.Ri, I.item.endFace.pout, Pout);

    var slices = N.item.slices.concat(I.item.slices);   // overlap is fine for SAT
    var E = { pin: Pin.slice(), pout: Pout.slice() };
    var armN = N.item.endFace, armI = I.item.endFace;
    var centroid = polyCentroid(pts);
    var mainFace, sideFace;

    if (rot) {
      // really turn the stone: re-attach it by one of its arms
      var attach = rot === 1 ? armN : armI;
      var map = makeRigid(attach.pin, attach.pout, Pin, Pout);
      if (dot(sub(map(centroid), faceMid), T) < 0) map = makeRigid(attach.pout, attach.pin, Pin, Pout);
      var mapFace = function (f) { return { pin: map(f.pin), pout: map(f.pout) }; };
      pts = pts.map(map);
      slices = slices.map(function (q) { return ensureCCW(q.map(map)); });
      centroid = map(centroid);
      mainFace = mapFace(rot === 1 ? armI : armN);
      sideFace = mapFace(E);
      // orient the free faces so chains keep growing away from the stone
      [mainFace, sideFace].forEach(function (g) {
        var ug = norm(sub(g.pout, g.pin));
        var gm = mul(add(g.pin, g.pout), 0.5);
        if (dot([ug[1], -ug[0]], sub(gm, centroid)) < 0) {
          var tmp = g.pin; g.pin = g.pout; g.pout = tmp;
        }
      });
    } else {
      mainFace = armN;
      sideFace = armI;
    }

    var area = Math.abs(polySignedArea(pts));
    var grams = { wgelb: massCalibration.wgelbGrams, worange: massCalibration.worangeGrams }[type];
    var uAng = deg(Math.atan2(cur.u[1], cur.u[0]));
    var item = {
      type: type, inverted: false, rot: rot, isSwitch: true,
      poly: pts, slices: slices,
      centroid: centroid, area: area,
      massRel: (grams != null && grams > 0) ? grams : area,
      startFace: { pin: Pin.slice(), pout: Pout.slice() },
      endFace: mainFace,                            // main-path continuation
      exitSideFace: sideFace,
      exitSideCur: { Pin: sideFace.pin.slice(), u: norm(sub(sideFace.pout, sideFace.pin)) },
      turn: -normDeg(faceDirDeg(mainFace) - uAng)
    };
    return { item: item, cur: { Pin: mainFace.pin.slice(), u: norm(sub(mainFace.pout, mainFace.pin)) } };
  }

  function faceOfCur(cur) {
    return { pin: cur.Pin.slice(), pout: add(cur.Pin, mul(cur.u, 2)) };
  }

  // recursive chain walker: places one chain of nodes, records items, the
  // parent joints (links) and every open build front; switch stones recurse
  // into their side chain with an extended chain key
  function walkChainRec(nodes, cur, ctx, chainKey, parentIdx, parentFace) {
    var turn = 0, i, nd, st, item, myIdx;
    var prevIdx = parentIdx, prevFace = parentFace;
    for (i = 0; i < nodes.length; i++) {
      nd = nodes[i];
      st = STONES[nd.type].sw ? makeSwitchItem(cur, nd.type, nd.rot)
                              : stepStone(cur, nd.type, nd.inverted);
      item = st.item;
      item.chain = chainKey;
      item.idx = i;
      item.born = nd.born || 0;
      myIdx = ctx.items.length;
      ctx.items.push(item);
      if (prevIdx >= 0) ctx.links.push({ a: prevIdx, b: myIdx, face: prevFace });
      if (item.isSwitch) {
        var sideKey = chainKey.concat([i]);
        var se = walkChainRec(nd.side || [], item.exitSideCur, ctx, sideKey, myIdx, item.exitSideFace);
        ctx.fronts.push({ chain: sideKey, cur: se.cur, face: faceOfCur(se.cur), endIdx: se.endIdx, main: false });
      }
      prevIdx = myIdx;
      prevFace = item.endFace;
      cur = st.cur;
      turn += item.turn;
    }
    return { cur: cur, turn: turn, endIdx: prevIdx };
  }

  // shift: symmetric slide of the springers along the ground (closing the arch
  // means pushing both legs together on the floor - exactly like real stones)
  function computeHalf(stones, shift) {
    var ctx = { items: [], links: [], fronts: [] };
    var first = stones[0];
    var anchorR = first && STONES[first.type].Ri != null ? STONES[first.type].Ri : 2;
    var cur = { Pin: [-anchorR + (shift || 0), 0], u: [-1, 0] };
    var main = walkChainRec(stones, cur, ctx, [], -1, null);
    ctx.fronts.unshift({ chain: [], cur: main.cur, face: faceOfCur(main.cur), endIdx: main.endIdx, main: true });
    return { items: ctx.items, front: main.cur, sumDeg: main.turn,
             fronts: ctx.fronts, links: ctx.links, mainEndIdx: main.endIdx };
  }

  function mirrorItem(it) {
    return {
      type: it.type, inverted: it.inverted, rot: it.rot, chain: it.chain, idx: it.idx, born: it.born,
      isSwitch: it.isSwitch,
      area: it.area, massRel: it.massRel,
      poly: it.poly.map(mirX),
      slices: it.slices.map(function (q) { return ensureCCW(q.map(mirX)); }),
      centroid: mirX(it.centroid),
      startFace: mirFace(it.startFace),
      endFace: mirFace(it.endFace),
      turn: it.turn, mirrored: true
    };
  }

  /* ================= arch model + validation ================= */

  function faceGap(fa, fb) {
    return Math.max(dist(fa.pin, fb.pin), dist(fa.pout, fb.pout));
  }

  // keystone placed mirror-symmetric on the axis (its bisector = y axis).
  // The left leg front face and the keystone start face are parallel whenever
  // the angle sum closes; the symmetric ground slide makes them coincide.
  function makeKeystone(front, type) {
    var t = STONES[type];
    var phL = Math.atan2(front.u[1], front.u[0]);
    var FLy = front.Pin[1];
    var item = { type: type, inverted: false, isKey: true, turn: t.angle,
                 area: stoneArea(type), massRel: stoneMass(type) };
    if (type === 'wuerfel') {
      var A = [0, FLy];
      var B = add(A, mul(edir(phL), 2));
      var D = [-B[0], B[1]];
      var C2 = [0, B[1] + (D[1] - A[1])];          // = A + e(phL)*2 + e(phL-90)*2 on the axis
      item.poly = [A, B, C2, D];
      item.slices = [ensureCCW([A, B, C2, D])];
      item.centroid = polyCentroidAvg(item.poly);
      item.startFace = { pin: A, pout: B };
      item.endFace = { pin: A.slice(), pout: D };  // both lower faces carry (diamond apex)
    } else {
      var Ck = [0, FLy - t.Ri * Math.sin(phL)];
      var ph1 = phL - rad(t.angle);
      item.poly = sectorPoly(Ck, t.Ri, t.Ra, phL, ph1);
      item.slices = sectorSlices(Ck, t.Ri, t.Ra, phL, ph1);
      item.centroid = add(Ck, mul(edir((phL + ph1) / 2), stoneCentroidRadius(type)));
      item.startFace = { pin: add(Ck, mul(edir(phL), t.Ri)), pout: add(Ck, mul(edir(phL), t.Ra)) };
      item.endFace = mirFace(item.startFace);
    }
    return item;
  }

  // required symmetric slide so the closing joint faces actually meet
  function closureShift(front, keyType) {
    if (!keyType) return -front.Pin[0];
    var t = STONES[keyType];
    if (keyType === 'wuerfel') return -front.Pin[0];
    var phL = Math.atan2(front.u[1], front.u[0]);
    return t.Ri * Math.cos(phL) - front.Pin[0];
  }

  // single on-axis stone capping a mirrored pair of open fronts: entry face
  // horizontal on the symmetry axis, both switch arms reach down onto the pair
  // (Kai: two apex switches can be closed on top with a third switch)
  function fitBridge(frontFace, type) {
    if (!STONES[type]) return null;
    if (STONES[type].sw) {
      var probe = makeSwitchItem({ Pin: [-1, 0], u: [1, 0] }, type, false).item;
      var arms = [probe.endFace, probe.exitSideFace];
      var li = (arms[0].pin[0] + arms[0].pout[0]) < (arms[1].pin[0] + arms[1].pout[0]) ? 0 : 1;
      var armMidY = (arms[li].pin[1] + arms[li].pout[1]) / 2;
      var targetY = (frontFace.pin[1] + frontFace.pout[1]) / 2;
      var h = targetY - armMidY;
      var item = makeSwitchItem({ Pin: [-1, h], u: [1, 0] }, type, false).item;
      var arm = [item.endFace, item.exitSideFace][li];
      var gap = Math.min(faceGap(arm, frontFace),
        Math.max(dist(arm.pin, frontFace.pout), dist(arm.pout, frontFace.pin)));
      item.isBridge = true;
      return { item: item, gap: gap };
    }
    // plain stone as a single on-axis piece capping the mirrored front pair
    // (Kai: when two arms meet, ONE stone should do the job, not a nested pair)
    var curF = { Pin: frontFace.pin.slice(), u: norm(sub(frontFace.pout, frontFace.pin)) };
    var item2 = makeKeystone(curF, type);
    var gap2 = Math.min(faceGap(item2.startFace, frontFace),
      Math.max(dist(item2.startFace.pin, frontFace.pout), dist(item2.startFace.pout, frontFace.pin)));
    item2.isBridge = true;
    return { item: item2, gap: gap2 };
  }

  function attachBridges(m, bridges) {
    if (!bridges) return;
    for (var key in bridges) {
      var spec = bridges[key], found = null;
      for (var i = 0; i < m.fronts.length; i++) {
        if (!m.fronts[i].main && keyStr(m.fronts[i].chain) === key) { found = m.fronts[i]; break; }
      }
      var fit = found ? fitBridge(found.face, spec.type) : null;
      if (fit && fit.gap <= JOIN_TOL_CM) {
        fit.item.born = spec.born || 0;
        fit.item.bridgeKey = key;
        m.bridgeItems.push({ item: fit.item, frontKey: key, endIdx: found.endIdx, frontFace: found.face });
        found.bridged = true;
      } else {
        m.bridgeDropped.push(key);
      }
    }
  }

  // model of the whole arch: left items, keystone, mirrored right items,
  // missing angle, closure + geometric validity, merged chain + contacts
  function archModel(stones, keyType, bridges) {
    var half = computeHalf(stones, 0);
    var m = {
      half: half,
      left: half.items,
      right: half.items.map(mirrorItem),
      key: null, shift: 0,
      missing: 180 - 2 * half.sumDeg - (keyType ? STONES[keyType].angle : 0),
      closed: false, allClosed: false, valid: false, errors: [], joinGap: Infinity,
      fronts: half.fronts, bridgeItems: [], bridgeDropped: []
    };
    if (!stones.length) {
      if (keyType) m.key = makeKeystone(half.front, keyType);
      return m;
    }
    if (Math.abs(m.missing) <= CLOSURE_TOL_DEG) {
      // slide assist for the symmetric bow: push the legs together on the ground
      m.shift = closureShift(half.front, keyType);
      half = computeHalf(stones, m.shift);
      m.half = half;
      m.left = half.items;
      m.right = half.items.map(mirrorItem);
      m.fronts = half.fronts;
    }
    if (keyType) {
      m.key = makeKeystone(half.front, keyType);
      m.joinGap = faceGap(m.key.startFace, faceOfCur(half.front));
    } else {
      var last = faceOfCur(half.front);
      m.joinGap = faceGap(last, mirFace(last));
    }
    attachBridges(m, bridges);
    // closure is structural now: joints exist wherever faces actually meet.
    // Any geometrically valid structure can be tested - open branches are
    // honest cantilevers (Kai: Simulation auch bei offenen Aesten).
    buildChain(m);
    validateChain(m);
    m.valid = m.errors.length === 0;
    m.allClosed = m.closed;
    for (var fi = 0; fi < m.fronts.length; fi++) {
      var ff = m.fronts[fi];
      if (!ff.main && !(ff.bridged || ff.grounded || ff.joined)) m.allClosed = false;
    }
    return m;
  }

  // merged structure: all left items, keystone, all mirrored right items.
  // Contacts come from the walk links plus supports, side-branch feet and
  // joints where two open fronts happen to meet face on face.
  function buildChain(m) {
    var nL = m.left.length, hasKey = !!m.key, i, j;
    var bodies = m.left.slice();
    if (hasKey) bodies.push(m.key);
    for (i = 0; i < m.right.length; i++) bodies.push(m.right[i]);
    m.chain = bodies;
    var R = function (k) { return nL + (hasKey ? 1 : 0) + k; };

    var specs = [];
    specs.push({ a: -1, b: 0, face: m.left[0].startFace, kind: 'stone-ground' });
    specs.push({ a: -1, b: R(0), face: mirFace(m.left[0].startFace), kind: 'stone-ground' });
    var links = m.half.links, l;
    for (i = 0; i < links.length; i++) {
      l = links[i];
      specs.push({ a: l.a, b: l.b, face: l.face, kind: 'stone-stone' });
      specs.push({ a: R(l.a), b: R(l.b), face: mirFace(l.face), kind: 'stone-stone' });
    }
    // apex: keystone joints, or the two main fronts if they actually meet
    var e = m.half.mainEndIdx;
    var mainFrontFace = faceOfCur(m.half.front);
    var mainF = null;
    for (i = 0; i < m.fronts.length; i++) if (m.fronts[i].main) mainF = m.fronts[i];
    if (hasKey) {
      specs.push({ a: e, b: nL, face: m.key.startFace, kind: 'stone-stone' });
      specs.push({ a: nL, b: R(e), face: m.key.endFace, kind: 'stone-stone' });
      m.closed = true;
      if (mainF) mainF.joined = true;
    } else if (faceGap(mainFrontFace, mirFace(mainFrontFace)) <= JOIN_TOL_CM) {
      specs.push({ a: e, b: R(e), face: mainFrontFace, kind: 'stone-stone' });
      m.closed = true;
      if (mainF) mainF.joined = true;
    }
    // every unconsumed front (including an unjoined main front): ground support
    // when the face lies on the floor, otherwise candidate for front joints
    var open = [], f;
    for (i = 0; i < m.fronts.length; i++) {
      f = m.fronts[i];
      if (f.bridged || f.joined) continue;
      var onGround = Math.abs(f.face.pin[1]) <= JOIN_TOL_CM && Math.abs(f.face.pout[1]) <= JOIN_TOL_CM;
      if (onGround) {
        f.grounded = true;
        specs.push({ a: -1, b: f.endIdx, face: f.face, kind: 'stone-ground' });
        specs.push({ a: -1, b: R(f.endIdx), face: mirFace(f.face), kind: 'stone-ground' });
      } else {
        open.push({ face: f.face, idx: f.endIdx, ref: f });
        open.push({ face: mirFace(f.face), idx: R(f.endIdx), ref: f });
      }
    }
    for (i = 0; i < open.length; i++) {
      for (j = i + 1; j < open.length; j++) {
        if (open[i].idx === open[j].idx) continue;
        var g1 = faceGap(open[i].face, open[j].face);
        var g2 = Math.max(dist(open[i].face.pin, open[j].face.pout),
                          dist(open[i].face.pout, open[j].face.pin));
        if (Math.min(g1, g2) <= JOIN_TOL_CM) {
          specs.push({ a: open[i].idx, b: open[j].idx, face: open[i].face, kind: 'stone-stone' });
          open[i].ref.joined = true;
          open[j].ref.joined = true;
          if (open[i].ref.main || open[j].ref.main) m.closed = true;
        }
      }
    }
    // bridge stones: single on-axis stones capping a mirrored front pair
    for (i = 0; i < (m.bridgeItems || []).length; i++) {
      var br = m.bridgeItems[i];
      var bIdx = m.chain.length;
      m.chain.push(br.item);
      specs.push({ a: br.endIdx, b: bIdx, face: br.frontFace, kind: 'stone-stone' });
      specs.push({ a: R(br.endIdx), b: bIdx, face: mirFace(br.frontFace), kind: 'stone-stone' });
    }
    m.contactSpecs = specs;
  }

  function detectContacts(m) {
    var out = [], i, cs, p0, p1, L, t, n, refv, mid;
    for (i = 0; i < m.contactSpecs.length; i++) {
      cs = m.contactSpecs[i];
      p0 = cs.face.pin; p1 = cs.face.pout;
      L = dist(p0, p1);
      t = norm(sub(p1, p0));
      n = perp(t);
      mid = mul(add(p0, p1), 0.5);
      refv = cs.a === -1 ? sub(m.chain[cs.b].centroid, mid)
                         : sub(m.chain[cs.b].centroid, m.chain[cs.a].centroid);
      if (dot(n, refv) < 0) n = mul(n, -1);          // normal points from A to B
      out.push({
        id: 'c' + i, a: cs.a, b: cs.b, p0: p0, p1: p1, t: t, n: n, L: L,
        kind: cs.kind,
        mu: cs.kind === 'stone-ground' ? calibration.stoneGroundMu : calibration.stoneStoneMu
      });
    }
    return out;
  }

  // SAT overlap of two convex CCW polys: max separation over both normal sets
  function satSeparation(Pa, Pb) {
    function maxSep(P, Q) {
      var best = -1e9, i, j, k;
      for (i = 0; i < P.length; i++) {
        j = (i + 1) % P.length;
        var e = sub(P[j], P[i]);
        var nn = norm([e[1], -e[0]]);               // outward for CCW
        var s = 1e9;
        for (k = 0; k < Q.length; k++) {
          var d = dot(nn, sub(Q[k], P[i]));
          if (d < s) s = d;
        }
        if (s > best) best = s;
      }
      return best;
    }
    return Math.max(maxSep(Pa, Pb), maxSep(Pb, Pa));
  }

  function bodiesOverlap(A, B, tol) {
    var ba = polyAABB(A.poly), bb = polyAABB(B.poly);
    if (ba[0] > bb[2] + tol || bb[0] > ba[2] + tol || ba[1] > bb[3] + tol || bb[1] > ba[3] + tol) return false;
    for (var i = 0; i < A.slices.length; i++) {
      for (var j = 0; j < B.slices.length; j++) {
        if (satSeparation(A.slices[i], B.slices[j]) < -tol) return true;
      }
    }
    return false;
  }

  // spec 4.6: geometric validity, NOT a physics verdict. Open joints are no
  // longer an error - unjoined fronts are simply open, the solver judges them.
  function validateChain(m) {
    var errs = [], i, j, b;
    for (i = 0; i < m.chain.length; i++) {
      b = m.chain[i];
      for (j = 0; j < b.poly.length; j++) {
        if (b.poly[j][1] < -OVERLAP_TOL_CM) { errs.push('below-ground'); i = m.chain.length; break; }
      }
    }
    var sf = m.left[0].startFace;
    if (Math.abs(sf.pin[1]) > CONTACT_TOL_CM || Math.abs(sf.pout[1]) > CONTACT_TOL_CM) errs.push('no-support');
    // bodies joined by a contact may touch; everything else must stay clear
    var touching = {};
    for (i = 0; i < m.contactSpecs.length; i++) {
      var cs = m.contactSpecs[i];
      if (cs.a >= 0) touching[Math.min(cs.a, cs.b) + '_' + Math.max(cs.a, cs.b)] = true;
    }
    for (i = 0; i < m.chain.length && errs.indexOf('overlap') < 0; i++) {
      for (j = i + 1; j < m.chain.length; j++) {
        if (touching[i + '_' + j]) continue;
        if (bodiesOverlap(m.chain[i], m.chain[j], OVERLAP_TOL_CM)) { errs.push('overlap'); break; }
      }
    }
    m.errors = errs;
  }

  /* ================= LP: dense two-phase simplex =================
     min c.x  s.t.  A x = b, x >= 0.  Bland's rule, small problems only. */

  function lpSolve(Arows, bvec, cvec) {
    var m = Arows.length, n = Arows.length ? Arows[0].length : 0;
    var total = n + m, i, j, k;
    var T = [], rhs = new Float64Array(m);
    for (i = 0; i < m; i++) {
      var row = new Float64Array(total);
      var neg = bvec[i] < 0;
      for (j = 0; j < n; j++) row[j] = neg ? -Arows[i][j] : Arows[i][j];
      row[n + i] = 1;
      // deterministic epsilon perturbation: breaks the heavy degeneracy of the
      // equilibrium system (b is mostly zero) so Dantzig pivoting stays fast;
      // magnitude 1e-8 is far below any physical or classification threshold
      rhs[i] = (neg ? -bvec[i] : bvec[i]) + 1e-8 * (i + 1);
      T.push(row);
    }
    var basis = new Int32Array(m);
    for (i = 0; i < m; i++) basis[i] = n + i;
    var dRow = new Float64Array(total);
    var EPS = 1e-9, DTOL = 1e-7;

    function pivot(pr, pc) {
      var prow = T[pr], f, ti, pi, pk;
      var inv = 1 / prow[pc];
      for (pk = 0; pk < total; pk++) prow[pk] *= inv;
      prow[pc] = 1;
      rhs[pr] *= inv;
      for (pi = 0; pi < m; pi++) {
        if (pi === pr) continue;
        ti = T[pi]; f = ti[pc];
        if (f === 0) continue;
        for (pk = 0; pk < total; pk++) ti[pk] -= f * prow[pk];
        ti[pc] = 0;
        rhs[pi] -= f * rhs[pr];
        if (rhs[pi] < 0 && rhs[pi] > -1e-11) rhs[pi] = 0;
      }
      f = dRow[pc];
      if (f !== 0) {
        for (pk = 0; pk < total; pk++) dRow[pk] -= f * prow[pk];
        dRow[pc] = 0;
      }
      basis[pr] = pc;
    }

    function initCosts(cost, limit) {
      var ci, cj;
      for (cj = 0; cj < total; cj++) dRow[cj] = cj < limit && cost ? (cost[cj] || 0) : (cost === null && cj >= n ? 1 : 0);
      // subtract c_B * tableau rows
      for (ci = 0; ci < m; ci++) {
        var cb = basis[ci] < limit && cost ? (cost[basis[ci]] || 0) : (cost === null && basis[ci] >= n ? 1 : 0);
        if (cb === 0) continue;
        var ti = T[ci];
        for (cj = 0; cj < total; cj++) dRow[cj] -= cb * ti[cj];
      }
    }

    function iterate(colLimit) {
      // Dantzig rule (fast) with a permanent switch to Bland's rule (anti-cycling)
      // after a fixed pivot budget - deterministic either way.
      var it, enter, leave, best, r, ratio, jc, BLAND_AT = 6000;
      for (it = 0; it < 20000; it++) {
        enter = -1;
        if (it < BLAND_AT) {
          var mostNeg = -DTOL;
          for (jc = 0; jc < colLimit; jc++) {
            if (dRow[jc] < mostNeg) { mostNeg = dRow[jc]; enter = jc; }
          }
        } else {
          for (jc = 0; jc < colLimit; jc++) {
            if (dRow[jc] < -DTOL) { enter = jc; break; }
          }
        }
        if (enter < 0) return 'optimal';
        leave = -1; best = Infinity;
        for (r = 0; r < m; r++) {
          if (T[r][enter] > EPS) {
            ratio = rhs[r] / T[r][enter];
            if (ratio < best - 1e-12 || (ratio < best + 1e-12 && (leave < 0 || basis[r] < basis[leave]))) {
              best = ratio; leave = r;
            }
          }
        }
        if (leave < 0) return 'unbounded';
        pivot(leave, enter);
      }
      return 'maxiter';
    }

    // phase 1: minimise sum of artificials (cost = null marker)
    initCosts(null, 0);
    var st = iterate(total);
    if (st !== 'optimal' && st !== 'unbounded') return { status: 'error', detail: 'phase1-' + st };
    var infeasSum = 0;
    for (i = 0; i < m; i++) if (basis[i] >= n) infeasSum += rhs[i];
    if (infeasSum > 1e-6) return { status: 'infeasible' };
    // drive remaining artificials out of the basis where possible
    for (i = 0; i < m; i++) {
      if (basis[i] >= n) {
        for (j = 0; j < n; j++) {
          if (Math.abs(T[i][j]) > 1e-7) { pivot(i, j); break; }
        }
      }
    }
    var x = new Float64Array(n);
    if (cvec) {
      initCosts(cvec, n);
      st = iterate(n);                               // artificials blocked
      if (st === 'maxiter') return { status: 'error', detail: 'phase2-maxiter' };
    }
    for (i = 0; i < m; i++) if (basis[i] < n) x[basis[i]] = rhs[i];
    var obj = 0;
    if (cvec) for (j = 0; j < n; j++) obj += (cvec[j] || 0) * x[j];
    return { status: 'optimal', x: x, obj: obj };
  }

  /* ================= static equilibrium model =================
     Unknowns per contact j, endpoint k in {0,1}:  N_jk >= 0,  T_jk = Tp - Tm.
     Per stone: sum Fx = 0, sum Fy = 0, sum M(centroid) = 0.
     Friction rows:  Tp + Tm - mu N + slack = 0   (=> |T| <= mu N)
     Alpha rows (spec 8.2): (1-a)N1 - aN0 >= 0  and  (1-a)N0 - aN1 >= 0.        */

  function buildLP(bodies, contacts, opt) {
    var nb = bodies.length, nc = contacts.length;
    var structN = nc * 6;
    var slackFric = opt.friction ? nc * 2 : 0;
    var slackAlpha = opt.alpha > 0 ? nc * 2 : 0;
    var nCols = structN + slackFric + slackAlpha;
    var nRows = nb * 3 + slackFric + slackAlpha;
    var A = [], b = new Float64Array(nRows), i, j, k;
    for (i = 0; i < nRows; i++) A.push(new Float64Array(nCols));

    var totalMass = 0;
    for (i = 0; i < nb; i++) totalMass += bodies[i].massRel;

    for (j = 0; j < nc; j++) {
      var c = contacts[j];
      for (k = 0; k < 2; k++) {
        var p = mul(k ? c.p1 : c.p0, 1 / LEN_SCALE);
        var colN = j * 6 + k * 3, colTp = colN + 1, colTm = colN + 2;
        var sides = [[c.b, 1], [c.a, -1]];
        for (var s = 0; s < 2; s++) {
          var bi = sides[s][0], sg = sides[s][1];
          if (bi < 0) continue;                      // ground has no equations
          var r = sub(p, mul(bodies[bi].centroid, 1 / LEN_SCALE));
          A[bi * 3][colN] += sg * c.n[0];
          A[bi * 3][colTp] += sg * c.t[0];
          A[bi * 3][colTm] -= sg * c.t[0];
          A[bi * 3 + 1][colN] += sg * c.n[1];
          A[bi * 3 + 1][colTp] += sg * c.t[1];
          A[bi * 3 + 1][colTm] -= sg * c.t[1];
          A[bi * 3 + 2][colN] += sg * cross(r, c.n);
          A[bi * 3 + 2][colTp] += sg * cross(r, c.t);
          A[bi * 3 + 2][colTm] -= sg * cross(r, c.t);
        }
      }
    }
    for (i = 0; i < nb; i++) b[i * 3 + 1] = bodies[i].massRel / totalMass;  // Fy = +w_i

    var rowIdx = nb * 3, colIdx = structN;
    if (opt.friction) {
      for (j = 0; j < nc; j++) {
        for (k = 0; k < 2; k++) {
          var cn = j * 6 + k * 3;
          A[rowIdx][cn] = -contacts[j].mu * (opt.muScale || 1);
          A[rowIdx][cn + 1] = 1;
          A[rowIdx][cn + 2] = 1;
          A[rowIdx][colIdx] = 1;
          rowIdx++; colIdx++;
        }
      }
    }
    if (opt.alpha > 0) {
      var a = opt.alpha;
      for (j = 0; j < nc; j++) {
        var n0 = j * 6, n1 = j * 6 + 3;
        A[rowIdx][n1] = 1 - a; A[rowIdx][n0] = -a; A[rowIdx][colIdx] = -1;
        rowIdx++; colIdx++;
        A[rowIdx][n0] = 1 - a; A[rowIdx][n1] = -a; A[rowIdx][colIdx] = -1;
        rowIdx++; colIdx++;
      }
    }
    var cvec = null;
    if (opt.objective) {
      cvec = new Float64Array(nCols);
      for (j = 0; j < nc; j++) {
        for (k = 0; k < 2; k++) {
          cvec[j * 6 + k * 3] = 1;                   // sum N
          cvec[j * 6 + k * 3 + 1] = 0.05;            // + 0.05 * |T| (via Tp+Tm)
          cvec[j * 6 + k * 3 + 2] = 0.05;
        }
      }
    }
    return { A: A, b: b, c: cvec, structN: structN };
  }

  function solveCase(bodies, contacts, opt) {
    var lp = buildLP(bodies, contacts, opt);
    var r = lpSolve(lp.A, lp.b, lp.c);
    r.structN = lp.structN;
    return r;
  }

  // full stability analysis, spec 7-10
  function analyzeStatics(bodies, contacts) {
    var t0 = NOW();
    var res = {
      status: 'unstable', reason: 'NO_COMPRESSION_EQUILIBRIUM',
      alphaMax: null, frictionUtilizationMax: null,
      pressurePoints: [], contacts: [], solveMs: 0, feasible: false, error: null
    };
    var base = solveCase(bodies, contacts, { friction: true, alpha: 0 });
    if (base.status === 'error') { res.error = base.detail; res.status = 'error'; res.reason = 'SOLVER_ERROR'; res.solveMs = NOW() - t0; return res; }
    if (base.status === 'unbounded') { res.error = 'unbounded'; res.status = 'error'; res.reason = 'SOLVER_ERROR'; res.solveMs = NOW() - t0; return res; }
    if (base.status === 'infeasible') {
      var diag = solveCase(bodies, contacts, { friction: false, alpha: 0 });
      res.reason = (diag.status === 'optimal') ? 'FRICTION_LIMIT' : 'NO_COMPRESSION_EQUILIBRIUM';
      res.solveMs = NOW() - t0;
      return res;
    }
    res.feasible = true;

    // spec 8.2: binary search for the largest admissible edge margin alpha
    var lo = 0, hi = 0.499, it, mid;
    for (it = 0; it < 18; it++) {
      mid = (lo + hi) / 2;
      var probe = solveCase(bodies, contacts, { friction: true, alpha: mid });
      if (probe.status === 'optimal') lo = mid; else hi = mid;
    }
    res.alphaMax = lo;

    // spec 8.3: deterministic final state with moderate forces
    var aUse = Math.max(0, lo - 0.001);
    var fin = solveCase(bodies, contacts, { friction: true, alpha: aUse, objective: true });
    if (fin.status !== 'optimal') fin = solveCase(bodies, contacts, { friction: true, alpha: 0, objective: true });
    if (fin.status !== 'optimal') { res.status = 'error'; res.reason = 'SOLVER_ERROR'; res.error = 'final-' + fin.status; res.solveMs = NOW() - t0; return res; }

    var rhoMax = 0, j;
    for (j = 0; j < contacts.length; j++) {
      var c = contacts[j];
      var N0 = fin.x[j * 6], T0 = fin.x[j * 6 + 1] - fin.x[j * 6 + 2];
      var N1 = fin.x[j * 6 + 3], T1 = fin.x[j * 6 + 4] - fin.x[j * 6 + 5];
      var Ns = N0 + N1;
      var sol = { id: c.id, N0: N0, T0: T0, N1: N1, T1: T1, loaded: Ns > 1e-7 };
      if (sol.loaded) {
        sol.sRel = N1 / Ns;
        sol.point = add(c.p0, mul(sub(c.p1, c.p0), sol.sRel));
        sol.hinge = sol.sRel <= 0.015 || sol.sRel >= 0.985;   // spec 9.4
        sol.rho = Math.abs(T0 + T1) / (c.mu * Ns);            // spec 9.5
        if (sol.rho > rhoMax) rhoMax = sol.rho;
        res.pressurePoints.push({ x: sol.point[0], y: sol.point[1], sRel: sol.sRel, hinge: sol.hinge, contact: j });
      } else {
        sol.sRel = null; sol.hinge = false; sol.rho = 0;
      }
      res.contacts.push(sol);
    }
    res.frictionUtilizationMax = rhoMax;

    if (res.alphaMax < 0.01 || rhoMax > 0.95) {               // spec 10
      res.status = 'critical';
      res.reason = res.alphaMax < 0.01 ? 'EDGE_LIMIT' : 'FRICTION_LIMIT';
    } else {
      res.status = 'stable';
      res.reason = 'OK';
    }
    res.solveMs = NOW() - t0;
    return res;
  }

  // spec 15: central evaluation flow. Deviation on Kai's request: any valid
  // structure is testable - closure is not required, open branches are honest.
  function evaluateArch(stones, keyType, bridges) {
    var m = archModel(stones, keyType, bridges);
    if (!stones || !stones.length) return { state: 'incomplete', missing: m.missing, model: m };
    if (!m.valid) return { state: 'invalidGeometry', errors: m.errors, model: m };
    var contacts = detectContacts(m);
    var st = analyzeStatics(m.chain, contacts);
    st.contactGeo = contacts;
    if (st.status === 'error') return { state: 'error', result: st, model: m };
    return { state: st.status, result: st, model: m };
  }

  /* ================= rigid body dynamics (collapse) =================
     Built-in impulse solver: convex fixtures, SAT + face clipping, sequential
     impulses with accumulated clamping + warm starting, fixed 1/120 s step.
     Units: cm, gravity 981 cm/s^2. Deterministic (no RNG, fixed order).      */

  var DYN = {
    dt: 1 / 120,
    gravity: 981,
    restitution: 0.02,
    linearDamping: 0.05,
    angularDamping: 0.08,
    // spec 11.4 says 12 (planck's block solver); this plain sequential-impulse
    // solver needs more sweeps to converge long stone chains - verified: at 12
    // even statically stable arches sag, at >=60 statics and dynamics agree
    velIterations: 100,
    baumgarte: 0.2,
    slop: 0.05,
    restThreshold: 20,
    sleepLin: 0.4,          // cm/s   (= spec 0.02 world units)
    sleepAng: 0.02,         // rad/s
    sleepTime: 0.5,
    imperfectionDeg: 0.03,
    maxSeconds: 12
  };

  function polyInertia(pts, rho) {
    // second moment about the local origin for a CCW polygon, density rho
    var I = 0, i, j;
    for (i = 0; i < pts.length; i++) {
      j = (i + 1) % pts.length;
      var cr = cross(pts[i], pts[j]);
      I += cr * (dot(pts[i], pts[i]) + dot(pts[i], pts[j]) + dot(pts[j], pts[j]));
    }
    return rho * I / 12;
  }

  function makeDynBody(item, idx) {
    var c = item.centroid;
    var fixtures = [], i;
    for (i = 0; i < item.slices.length; i++) {
      fixtures.push(item.slices[i].map(function (p) { return sub(p, c); }));
    }
    // mass and inertia from the true outline (switch stones have overlapping
    // collision slices - summing those would double-count the shared region)
    var outline = ensureCCW(item.poly.map(function (p) { return sub(p, c); }));
    var area = Math.abs(polySignedArea(outline));
    var mass = item.massRel;
    var rho = mass / area;
    var inertia = Math.abs(polyInertia(outline, rho));
    return {
      idx: idx, type: item.type,
      pos: c.slice(), ang: 0,
      vel: [0, 0], angVel: 0,
      invM: 1 / mass, invI: 1 / inertia,
      fixtures: fixtures,
      outline: item.poly.map(function (p) { return sub(p, c); }),
      isStatic: false, still: 0
    };
  }

  function createCollapseWorld(chain) {
    var w = { bodies: [], time: 0, settled: false, cache: {} };
    for (var i = 0; i < chain.length; i++) {
      var b = makeDynBody(chain[i], i);
      // spec 11.6: tiny deterministic imperfection, alternating sign
      b.ang = rad(DYN.imperfectionDeg) * (i % 2 === 0 ? 1 : -1);
      b.start = b.pos.slice();
      w.bodies.push(b);
    }
    w.ground = {
      idx: -1, pos: [0, 0], ang: 0, vel: [0, 0], angVel: 0,
      invM: 0, invI: 0, isStatic: true,
      fixtures: [ensureCCW([[-500, -40], [500, -40], [500, 0], [-500, 0]])],
      outline: []
    };
    return w;
  }

  function bodyWorldVerts(b) {
    var cs = Math.cos(b.ang), sn = Math.sin(b.ang);
    return b.fixtures.map(function (f) {
      return f.map(function (p) {
        return [b.pos[0] + p[0] * cs - p[1] * sn, b.pos[1] + p[0] * sn + p[1] * cs];
      });
    });
  }

  function maxSepIdx(P, Q) {
    var best = -1e9, bi = 0, i, j, k;
    for (i = 0; i < P.length; i++) {
      j = (i + 1) % P.length;
      var e = sub(P[j], P[i]);
      var nn = norm([e[1], -e[0]]);
      var s = 1e9;
      for (k = 0; k < Q.length; k++) {
        var d = dot(nn, sub(Q[k], P[i]));
        if (d < s) s = d;
      }
      if (s > best) { best = s; bi = i; }
    }
    return { s: best, i: bi };
  }

  function clipSegment(pts, d, offset) {
    // keep points with dot(d, p) - offset >= 0, interpolate the crossing
    var out = [], i;
    var d0 = dot(d, pts[0]) - offset, d1 = dot(d, pts[1]) - offset;
    if (d0 >= 0) out.push(pts[0]);
    if (d1 >= 0) out.push(pts[1]);
    if (d0 * d1 < 0) {
      var t = d0 / (d0 - d1);
      out.push(add(pts[0], mul(sub(pts[1], pts[0]), t)));
    }
    return out;
  }

  function collidePolys(VA, VB) {
    var sa = maxSepIdx(VA, VB);
    if (sa.s > 0.01) return null;
    var sb = maxSepIdx(VB, VA);
    if (sb.s > 0.01) return null;
    var ref, inc, refIdx, flip;
    if (sb.s > sa.s + 0.001) { ref = VB; inc = VA; refIdx = sb.i; flip = true; }
    else { ref = VA; inc = VB; refIdx = sa.i; flip = false; }
    var r1 = ref[refIdx], r2 = ref[(refIdx + 1) % ref.length];
    var re = sub(r2, r1);
    var refN = norm([re[1], -re[0]]);
    // incident edge: most anti-parallel normal on the other poly
    var bi = 0, bdot = 1e9, i, j;
    for (i = 0; i < inc.length; i++) {
      j = (i + 1) % inc.length;
      var e = sub(inc[j], inc[i]);
      var nn = norm([e[1], -e[0]]);
      var dd = dot(nn, refN);
      if (dd < bdot) { bdot = dd; bi = i; }
    }
    var seg = [inc[bi], inc[(bi + 1) % inc.length]];
    var dRef = norm(re);
    seg = clipSegment(seg, dRef, dot(dRef, r1));
    if (seg.length < 2) return null;
    seg = clipSegment(seg, mul(dRef, -1), dot(mul(dRef, -1), r2));
    if (!seg.length) return null;
    var pts = [];
    for (i = 0; i < seg.length && pts.length < 2; i++) {
      var sep = dot(refN, sub(seg[i], r1));
      if (sep <= 0.01) pts.push({ p: seg[i], depth: Math.max(0, -sep) });
    }
    if (!pts.length) return null;
    return { n: flip ? mul(refN, -1) : refN, pts: pts };
  }

  function stepWorld(w) {
    if (w.settled) return;
    var dt = DYN.dt, i, j, k, b;
    var dynBodies = w.bodies;

    for (i = 0; i < dynBodies.length; i++) {
      b = dynBodies[i];
      b.vel[1] -= DYN.gravity * dt;
      var dl = 1 / (1 + dt * DYN.linearDamping);
      b.vel[0] *= dl; b.vel[1] *= dl;
      b.angVel *= 1 / (1 + dt * DYN.angularDamping);
      b.vb = [0, 0]; b.wb = 0;      // pseudo velocities (split impulse position bias)
    }
    w.ground.vb = [0, 0]; w.ground.wb = 0;

    // broadphase + narrowphase
    var verts = [], aabbs = [];
    for (i = 0; i < dynBodies.length; i++) {
      verts.push(bodyWorldVerts(dynBodies[i]));
      var bb = [1e9, 1e9, -1e9, -1e9];
      for (j = 0; j < verts[i].length; j++) {
        var vb = polyAABB(verts[i][j]);
        if (vb[0] < bb[0]) bb[0] = vb[0];
        if (vb[1] < bb[1]) bb[1] = vb[1];
        if (vb[2] > bb[2]) bb[2] = vb[2];
        if (vb[3] > bb[3]) bb[3] = vb[3];
      }
      aabbs.push(bb);
    }
    var groundVerts = w.ground.fixtures;
    var cons = [], newCache = {};

    function addManifold(A, B, VA, VB, keyBase, mu) {
      var man = collidePolys(VA, VB);
      if (!man) return;
      for (var q = 0; q < man.pts.length; q++) {
        var p = man.pts[q].p;
        var rA = sub(p, A.pos), rB = sub(p, B.pos);
        var n = man.n, t = perp(n);
        var rnA = cross(rA, n), rnB = cross(rB, n);
        var rtA = cross(rA, t), rtB = cross(rB, t);
        var kn = A.invM + B.invM + A.invI * rnA * rnA + B.invI * rnB * rnB;
        var kt = A.invM + B.invM + A.invI * rtA * rtA + B.invI * rtB * rtB;
        var relv = [
          B.vel[0] - B.angVel * rB[1] - A.vel[0] + A.angVel * rA[1],
          B.vel[1] + B.angVel * rB[0] - A.vel[1] - A.angVel * rA[0]
        ];
        var vn0 = dot(relv, n);
        var key = keyBase + '_' + q;
        var warm = w.cache[key];
        cons.push({
          A: A, B: B, n: n, t: t, rA: rA, rB: rB,
          massN: kn > 0 ? 1 / kn : 0, massT: kt > 0 ? 1 / kt : 0,
          bias: Math.min(300, DYN.baumgarte / dt * Math.max(0, man.pts[q].depth - DYN.slop)),
          rest: DYN.restitution * Math.max(0, -vn0 - DYN.restThreshold),
          mu: mu, key: key,
          jn: warm ? warm.jn : 0, jt: warm ? warm.jt : 0, jb: 0
        });
      }
    }

    for (i = 0; i < dynBodies.length; i++) {
      for (j = i + 1; j < dynBodies.length; j++) {
        var a1 = aabbs[i], a2 = aabbs[j];
        if (a1[0] > a2[2] + 0.1 || a2[0] > a1[2] + 0.1 || a1[1] > a2[3] + 0.1 || a2[1] > a1[3] + 0.1) continue;
        for (k = 0; k < verts[i].length; k++) {
          for (var l = 0; l < verts[j].length; l++) {
            addManifold(dynBodies[i], dynBodies[j], verts[i][k], verts[j][l],
              i + '_' + j + '_' + k + '_' + l, calibration.stoneStoneMu);
          }
        }
      }
      if (aabbs[i][1] < 1.0) {
        for (k = 0; k < verts[i].length; k++) {
          addManifold(w.ground, dynBodies[i], groundVerts[0], verts[i][k],
            'g_' + i + '_' + k, calibration.stoneGroundMu);
        }
      }
    }

    // warm start
    for (i = 0; i < cons.length; i++) {
      var c0 = cons[i];
      if (c0.jn === 0 && c0.jt === 0) continue;
      var P0 = add(mul(c0.n, c0.jn), mul(c0.t, c0.jt));
      c0.A.vel[0] -= P0[0] * c0.A.invM; c0.A.vel[1] -= P0[1] * c0.A.invM;
      c0.A.angVel -= c0.A.invI * cross(c0.rA, P0);
      c0.B.vel[0] += P0[0] * c0.B.invM; c0.B.vel[1] += P0[1] * c0.B.invM;
      c0.B.angVel += c0.B.invI * cross(c0.rB, P0);
    }

    for (var iter = 0; iter < DYN.velIterations; iter++) {
      for (i = 0; i < cons.length; i++) {
        var c = cons[i], A = c.A, B = c.B;
        // real velocity constraint - NO position bias here (split impulse):
        // bias impulses must not inflate the friction cone or add energy
        var rv = [
          B.vel[0] - B.angVel * c.rB[1] - A.vel[0] + A.angVel * c.rA[1],
          B.vel[1] + B.angVel * c.rB[0] - A.vel[1] - A.angVel * c.rA[0]
        ];
        var vn = dot(rv, c.n);
        var dJn = (-vn + c.rest) * c.massN;
        var jn0 = c.jn;
        c.jn = Math.max(0, jn0 + dJn);
        dJn = c.jn - jn0;
        var Pn = mul(c.n, dJn);
        A.vel[0] -= Pn[0] * A.invM; A.vel[1] -= Pn[1] * A.invM;
        A.angVel -= A.invI * cross(c.rA, Pn);
        B.vel[0] += Pn[0] * B.invM; B.vel[1] += Pn[1] * B.invM;
        B.angVel += B.invI * cross(c.rB, Pn);

        rv = [
          B.vel[0] - B.angVel * c.rB[1] - A.vel[0] + A.angVel * c.rA[1],
          B.vel[1] + B.angVel * c.rB[0] - A.vel[1] - A.angVel * c.rA[0]
        ];
        var vt = dot(rv, c.t);
        var dJt = -vt * c.massT;
        var maxT = c.mu * c.jn;
        var jt0 = c.jt;
        c.jt = clamp(jt0 + dJt, -maxT, maxT);
        dJt = c.jt - jt0;
        var Pt = mul(c.t, dJt);
        A.vel[0] -= Pt[0] * A.invM; A.vel[1] -= Pt[1] * A.invM;
        A.angVel -= A.invI * cross(c.rA, Pt);
        B.vel[0] += Pt[0] * B.invM; B.vel[1] += Pt[1] * B.invM;
        B.angVel += B.invI * cross(c.rB, Pt);

        // pseudo-velocity pass: resolves penetration without feeding friction
        var rvb = [
          B.vb[0] - B.wb * c.rB[1] - A.vb[0] + A.wb * c.rA[1],
          B.vb[1] + B.wb * c.rB[0] - A.vb[1] - A.wb * c.rA[0]
        ];
        var vnb = dot(rvb, c.n);
        var dJb = (c.bias - vnb) * c.massN;
        var jb0 = c.jb;
        c.jb = Math.max(0, jb0 + dJb);
        dJb = c.jb - jb0;
        if (dJb !== 0) {
          var Pb = mul(c.n, dJb);
          A.vb[0] -= Pb[0] * A.invM; A.vb[1] -= Pb[1] * A.invM;
          A.wb -= A.invI * cross(c.rA, Pb);
          B.vb[0] += Pb[0] * B.invM; B.vb[1] += Pb[1] * B.invM;
          B.wb += B.invI * cross(c.rB, Pb);
        }
      }
    }
    for (i = 0; i < cons.length; i++) newCache[cons[i].key] = { jn: cons[i].jn, jt: cons[i].jt };
    w.cache = newCache;

    var allStill = true;
    for (i = 0; i < dynBodies.length; i++) {
      b = dynBodies[i];
      b.pos[0] += (b.vel[0] + b.vb[0]) * dt;
      b.pos[1] += (b.vel[1] + b.vb[1]) * dt;
      b.ang += (b.angVel + b.wb) * dt;
      if (Math.abs(b.vel[0]) < DYN.sleepLin && Math.abs(b.vel[1]) < DYN.sleepLin &&
          Math.abs(b.angVel) < DYN.sleepAng) b.still += dt;
      else b.still = 0;
      if (b.still < DYN.sleepTime) allStill = false;
    }
    w.time += dt;
    if ((allStill && w.time > 1.0) || w.time > DYN.maxSeconds) {
      w.settled = true;
      // honest outcome: a statically inadmissible arch can either scatter or
      // slump a few millimetres and wedge itself into a new jammed state
      var maxDrop = 0, height = 1;
      for (i = 0; i < dynBodies.length; i++) {
        b = dynBodies[i];
        if (b.start[1] > height) height = b.start[1];
        if (b.start[1] - b.pos[1] > maxDrop) maxDrop = b.start[1] - b.pos[1];
      }
      w.outcome = maxDrop > Math.max(1.5, 0.2 * height) ? 'collapsed' : 'wedged';
    }
  }

  /* ================= state ================= */

  var canvas, ctx, taskEl, tipEl, holdBtn, holdLabel, resetBtn, verdictEl;
  var state, model, palItems, barButtons = [];
  var cam = { z: 15, ox: W / 2, tz: 15, tox: W / 2 };
  var dbg = false;

  function freshState() {
    return {
      phase: 'build',        // build | solving | result | collapsing | settled
      stones: [],            // left half main chain; switch nodes carry .side = [nodes]
      key: null,             // keystone type once placed (single stone!)
      offer: null,           // stone type floating over the gap
      sel: null,             // {kind:'pair', chain, idx} | {kind:'key'} | {kind:'bridge', frontKey}
      activeFront: [],       // chain key of the front new stones attach to ([] = main)
      bridges: {},           // frontKey -> {type, born}: on-axis singles capping a front pair
      result: null,          // last StabilityResult
      world: null,           // collapse world
      solveToken: 0,
      lastTap: 0, lastTapKey: null,
      pulse: 0
    };
  }

  function countNodes(nodes) {
    var n = 0, i;
    for (i = 0; i < nodes.length; i++) {
      n++;
      if (nodes[i].side) n += countNodes(nodes[i].side);
    }
    return n;
  }
  function resolveChain(chainKey) {
    var arr = state.stones, i, nd;
    for (i = 0; i < chainKey.length; i++) {
      nd = arr && arr[chainKey[i]];
      if (!nd || !nd.side) return null;
      arr = nd.side;
    }
    return arr;
  }
  function selNode() {
    if (!state.sel || state.sel.kind !== 'pair') return null;
    var arr = resolveChain(state.sel.chain);
    return arr ? arr[state.sel.idx] : null;
  }

  function rebuild() {
    model = archModel(state.stones, state.key, state.bridges);
    for (var d = 0; d < model.bridgeDropped.length; d++) delete state.bridges[model.bridgeDropped[d]];
    state.offer = null;
    state.offerModel = null;
    if (!state.key && state.stones.length && model.missing > 0) {
      for (var i = 0; i < PAL_ORDER.length; i++) {
        var k = PAL_ORDER[i];
        if (STONES[k].sw) continue;                  // a switch cannot close the gap
        if (Math.abs(STONES[k].angle - model.missing) <= CLOSURE_TOL_DEG) {
          var probe = archModel(state.stones, k);
          if (probe.closed && probe.joinGap <= CONTACT_TOL_CM) {
            state.offer = k;
            state.offerModel = probe;
            break;
          }
        }
      }
    }
  }

  function refresh() {
    rebuild();
    applyTexts();
  }

  /* ================= audio ================= */

  var audio = null;
  function tok(freq, dur, gain) {
    if (!IS_BROWSER) return;
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

  /* ================= texts ================= */

  function setTask(html) { if (taskEl) taskEl.innerHTML = html; }
  function setTip(html) {
    if (!tipEl) return;
    if (!html) { tipEl.classList.remove('is-on'); tipEl.innerHTML = ''; return; }
    tipEl.innerHTML = html;
    tipEl.classList.add('is-on');
  }
  function showVerdict(text) {
    if (!verdictEl) return;
    verdictEl.textContent = text;
    if (IS_BROWSER && window.gsap) {
      gsap.killTweensOf(verdictEl);
      gsap.fromTo(verdictEl, { opacity: 0, y: 18, scale: .94 }, { opacity: 1, y: 0, scale: 1, duration: .5, ease: 'back.out(1.7)' });
    } else if (verdictEl) {
      verdictEl.style.opacity = 1;
    }
  }
  function hideVerdict() {
    if (!verdictEl) return;
    if (IS_BROWSER && window.gsap) gsap.killTweensOf(verdictEl);
    verdictEl.style.opacity = 0;
  }

  function fmtDeg(x) {
    var r = Math.round(x);
    if (Math.abs(x - r) < 0.005) return String(r);
    return x.toFixed(1);
  }

  function applyTexts() {
    var M = model.missing;
    if (state.phase === 'solving') {
      setTask('Wir pr&uuml;fen, wie die Kr&auml;fte durch den Bogen laufen.');
      setTip('');
      return;
    }
    if (state.phase === 'result' && state.result) {
      if (state.result.status === 'stable') {
        setTask('Der Bogen steht.');
        setTip('<b>Drucklinie:</b> So flie&szlig;t der Druck von Fuge zu Fuge in den Boden. Jeder Punkt liegt sicher in seiner Fuge.');
      } else {
        setTask('Der Bogen steht gerade so.');
        setTip(state.result.reason === 'FRICTION_LIMIT'
          ? '<b>Knapp:</b> An mindestens einer Fuge ist die Reibung fast ausgereizt.'
          : '<b>Knapp:</b> Die Drucklinie ber&uuml;hrt fast eine Fugenkante &ndash; ein Gelenk k&ouml;nnte aufgehen.');
      }
      return;
    }
    if (state.phase === 'collapsing') {
      // spec 13.1: no extra message during motion - the unstable reason stays
      return;
    }
    if (state.phase === 'settled') {
      if (state.world && state.world.outcome === 'wedged') {
        setTask('Der Bogen f&auml;llt nicht &ndash; aber er tr&auml;gt nicht sauber: Er hat sich verkeilt.');
        setTip('Die Druckkr&auml;fte finden keinen sicheren Weg durch die Fugen; der Bogen klemmt sich nur fest. Unten: nochmal fallen lassen oder zur&uuml;ck zum Bauen.');
      } else {
        setTask('Der Bogen ist zusammengefallen.');
        setTip('Schau, wo er aufgegangen ist &ndash; und bau die Stelle anders. Unten: nochmal fallen lassen oder zur&uuml;ck zum Bauen.');
      }
      return;
    }
    // build phase
    if (!state.stones.length) {
      setTask('W&auml;hl unten einen Stein &ndash; er setzt sich links <b>und</b> rechts zugleich.');
      setTip('');
      return;
    }
    if (!model.valid) {
      setTask('Steine &uuml;berschneiden sich oder stecken im Boden &ndash; so l&auml;sst sich das nicht pr&uuml;fen.');
      setTip('Nimm den zuletzt gesetzten Stein raus oder dreh eine Weiche zur&uuml;ck.');
      return;
    }
    if (model.allClosed && !state.sel) {
      setTask('Alles geschlossen. Ob die Konstruktion wirklich tr&auml;gt? Unten: <b>Bogen testen</b>.');
      setTip('<b>Werkstatt-Tipp:</b> Geschlossen hei&szlig;t noch nicht stabil &ndash; erst die Kr&auml;fteprobe zeigt es.');
      return;
    }
    if (model.closed && !state.sel && !state.activeFront.length) {
      setTask('Der Bogen ist geschlossen. Testen geht jederzeit &ndash; auch mit offenen &Auml;sten.');
      setTip('<b>Werkstatt-Tipp:</b> Offene &Auml;ste h&auml;ngen frei &ndash; die Kr&auml;fteprobe zeigt, ob sie halten.');
      return;
    }
    if (state.sel) {
      if (state.sel.kind === 'bridge') {
        setTask('Br&uuml;cke markiert &ndash; sie schlie&szlig;t beide &Auml;ste zugleich. Vorrat (Weiche) tauscht sie, &#10005; nimmt sie raus.');
        setTip('');
        return;
      }
      var sn = selNode();
      if (sn && STONES[sn.type].sw) {
        setTask('Weiche markiert &ndash; &#8635; dreht sie zur n&auml;chsten Fugenpassung; was dann nicht mehr anschlie&szlig;t, f&auml;llt weg.');
      } else {
        setTask('Stein markiert &ndash; unten: Vorrat tauscht ihn, &#8635; dreht die Kr&uuml;mmung, &#10005; nimmt ihn raus.');
      }
      setTip('');
      return;
    }
    if (state.activeFront.length) {
      setTask('Du baust am Seitenast &ndash; neue Steine setzen am leuchtenden Geist an. Anderen Geist antippen wechselt die Stelle.');
      setTip('Erreicht der Ast den Boden (gr&uuml;ner Geist), tr&auml;gt er mit &ndash; sonst h&auml;ngt er frei am Bogen.');
      return;
    }
    if (state.offer) {
      setTask('Der <b>Schlussstein</b> schwebt &uuml;ber der L&uuml;cke &ndash; tipp ihn an. Er wird nur <b>einmal</b> eingebaut.');
      setTip('');
      return;
    }
    if (M < -CLOSURE_TOL_DEG) {
      setTask(fmtDeg(-M) + '&deg; zu viel &ndash; nimm einen Stein raus oder dreh einen zur&uuml;ck.');
      setTip('<b>Werkstatt-Tipp:</b> Ein gedrehter Stein (&#8635;) kr&uuml;mmt nach au&szlig;en &ndash; sein Winkel z&auml;hlt r&uuml;ckw&auml;rts.');
      return;
    }
    setTask('Dem Bogen fehlen noch ' + fmtDeg(M) + '&deg;. Fuge an Fuge w&auml;chst er symmetrisch.');
    setTip('');
  }

  /* ================= mutations ================= */

  function clearResult() {
    state.result = null;
    state.world = null;
    if (state.phase === 'result' || state.phase === 'settled') state.phase = 'build';
    hideVerdict();
  }

  function dropKeyOnEdit() {
    if (state.key) { state.key = null; }
  }

  // add a stone at the active build front (main chain or a switch side chain)
  function addStone(typeKey) {
    if (countNodes(state.stones) >= MAX_STONES) return;
    var arr = resolveChain(state.activeFront);
    if (!arr) { state.activeFront = []; arr = state.stones; }
    clearResult();
    // a stone that caps the active front pair goes in ONCE as an on-axis bridge
    if (state.activeFront.length > 0 && model.fronts) {
      var afKey = keyStr(state.activeFront);
      for (var fi = 0; fi < model.fronts.length; fi++) {
        var fr = model.fronts[fi];
        if (fr.main || fr.bridged || keyStr(fr.chain) !== afKey) continue;
        var fitB = fitBridge(fr.face, typeKey);
        if (fitB && fitB.gap <= JOIN_TOL_CM && !state.bridges[afKey]) {
          state.bridges[afKey] = { type: typeKey, born: NOW() };
          tok(320, .07, .07);
          setTimeout(function () { tok(430, .12, .08); }, 120);
          state.sel = null;
          refresh();
          return;
        }
        break;
      }
    }
    if (state.activeFront.length === 0) dropKeyOnEdit();
    // side branches default to the branch's own curvature sense so the arc
    // continues smoothly; the user can still rotate every stone afterwards
    var inv = false;
    if (state.activeFront.length > 0) {
      inv = arr.length ? !!arr[arr.length - 1].inverted : true;
    }
    var nd = { type: typeKey, inverted: inv, born: NOW() };
    if (STONES[typeKey].sw) { nd.side = []; nd.rot = 0; }
    arr.push(nd);
    // Kai: two stones must never lie inside each other. If the new pair would
    // bury itself in an existing stone (or its own mirror twin), take it back -
    // and set the single on-axis stone instead where that roughly fits.
    var probe2 = archModel(state.stones, state.key, state.bridges);
    var afk = keyStr(state.activeFront), L = -1;
    for (var li2 = 0; li2 < probe2.left.length; li2++) {
      if (keyStr(probe2.left[li2].chain) === afk && probe2.left[li2].idx === arr.length - 1) { L = li2; break; }
    }
    if (L >= 0 && probe2.chain) {
      var twinIdx = probe2.left.length + (probe2.key ? 1 : 0) + L;
      var deep = -1;
      for (var bj2 = 0; bj2 < probe2.chain.length; bj2++) {
        if (bj2 === L) continue;
        if (bodiesOverlap(probe2.chain[L], probe2.chain[bj2], 1.0)) { deep = bj2; break; }
      }
      if (deep >= 0) {
        arr.pop();
        if (deep === twinIdx && state.activeFront.length === 0 && !state.key) {
          var kTry = archModel(state.stones, typeKey, state.bridges);
          if (kTry.key && kTry.joinGap <= JOIN_TOL_CM) {
            state.keyBorn = NOW();
            placeKeystone(typeKey);
            return;
          }
        }
        refresh();
        setTask('Da ist kein Platz mehr &ndash; der Stein l&auml;ge in einem anderen Stein.');
        return;
      }
    }
    tok(190, .08, .08);
    setTimeout(function () { tok(150, .1, .07); }, 110);
    state.sel = null;
    refresh();
  }

  function placeKeystone(typeKey) {
    clearResult();
    state.key = typeKey;
    state.offer = null;
    state.sel = null;
    tok(320, .07, .07);
    setTimeout(function () { tok(430, .12, .08); }, 120);
    refresh();
  }

  function swapSel(typeKey) {
    if (!state.sel) return;
    clearResult();
    if (state.sel.kind === 'bridge') {
      if (!STONES[typeKey].sw) {
        setTask('Nur eine Weiche kann als Br&uuml;cke zwei &Auml;ste zugleich schlie&szlig;en.');
        return;
      }
      var bEntry = state.bridges[state.sel.frontKey];
      if (bEntry && bEntry.type !== typeKey) {
        bEntry.type = typeKey;
        bEntry.born = NOW();
        tok(230, .07, .07);
        refresh();                       // re-fitted; silently dropped if it no longer fits
      }
      return;
    }
    if (state.sel.kind === 'key') {
      if (state.key === typeKey) return;
      if (STONES[typeKey].sw) {
        setTask('Eine Weiche kann kein Schlussstein sein &ndash; ihr zweiter Ausgang h&auml;tte keinen Anschluss.');
        return;
      }
      var probe = archModel(state.stones, typeKey);
      if (probe.closed && probe.joinGap <= CONTACT_TOL_CM) {
        state.key = typeKey;
        tok(230, .07, .07);
        refresh();
      } else {
        setTask('Der passt nicht in die L&uuml;cke &ndash; der Schlussstein braucht genau ' + fmtDeg(model.missing + STONES[state.key].angle) + '&deg;.');
      }
      return;
    }
    var s = selNode();
    if (!s || s.type === typeKey) return;
    if (state.sel.chain.length === 0) dropKeyOnEdit();
    s.type = typeKey;
    if (STONES[typeKey].sw) { if (!s.side) s.side = []; if (s.rot == null) s.rot = 0; }
    else { if (s.side) delete s.side; if (s.rot) delete s.rot; }
    s.born = NOW();
    tok(230, .07, .07);
    refresh();
  }

  function invertSel() {
    if (!state.sel) return;
    if (state.sel.kind === 'key') {
      setTask('Der Schlussstein l&auml;sst sich nicht drehen &ndash; er schlie&szlig;t die L&uuml;cke nach innen.');
      return;
    }
    if (state.sel.kind === 'bridge') {
      setTask('Die Br&uuml;cke ist spiegelsymmetrisch &ndash; Drehen &auml;ndert nichts an ihr.');
      return;
    }
    var s = selNode();
    if (!s) return;
    clearResult();
    if (STONES[s.type].sw) {
      // the stone REALLY turns to its next joint fit. Attached branches stay
      // wherever the rotated stone offers a face at the exact same spot again;
      // whatever is no longer connected gets removed (Kai's rule).
      var arr2 = resolveChain(state.sel.chain);
      if (!arr2) return;
      var idx2 = state.sel.idx;
      var oldItem = null;
      for (var q = 0; q < model.left.length; q++) {
        if (keyStr(model.left[q].chain) === keyStr(state.sel.chain) && model.left[q].idx === idx2) {
          oldItem = model.left[q];
          break;
        }
      }
      var newRot = ((s.rot || 0) + 1) % 3;
      var mainRest = arr2.slice(idx2 + 1);
      var sideChain = s.side || [];
      var keptMain = [], keptSide = [];
      if (oldItem) {
        var cur2 = { Pin: oldItem.startFace.pin, u: norm(sub(oldItem.startFace.pout, oldItem.startFace.pin)) };
        var probe2 = makeSwitchItem(cur2, s.type, newRot).item;
        var oldRoots = [{ face: oldItem.endFace, nodes: mainRest },
                        { face: oldItem.exitSideFace, nodes: sideChain }];
        var newSlots = [{ face: probe2.endFace, key: 'main' },
                        { face: probe2.exitSideFace, key: 'side' }];
        for (var a2 = 0; a2 < oldRoots.length; a2++) {
          if (!oldRoots[a2].nodes.length) continue;
          for (var b2 = 0; b2 < newSlots.length; b2++) {
            if (newSlots[b2].used) continue;
            if (faceGap(oldRoots[a2].face, newSlots[b2].face) <= JOIN_TOL_CM) {
              newSlots[b2].used = true;
              if (newSlots[b2].key === 'main') keptMain = oldRoots[a2].nodes;
              else keptSide = oldRoots[a2].nodes;
              break;
            }
          }
        }
      }
      var removed = (mainRest.length + sideChain.length) - (keptMain.length + keptSide.length);
      s.rot = newRot;
      s.side = keptSide;
      arr2.splice(idx2 + 1);
      for (var c2 = 0; c2 < keptMain.length; c2++) arr2.push(keptMain[c2]);
      if (state.sel.chain.length === 0) dropKeyOnEdit();
      if (!resolveChain(state.activeFront)) state.activeFront = [];
      tok(260, .07, .07);
      refresh();
      if (removed > 0) setTask('Weiche gedreht &ndash; ' + removed + ' nicht mehr verbundene Steine wurden entfernt.');
      return;
    }
    if (state.sel.chain.length === 0) dropKeyOnEdit();
    s.inverted = !s.inverted;
    tok(260, .07, .07);
    refresh();
  }

  function removeSel() {
    if (!state.sel) return;
    clearResult();
    if (state.sel.kind === 'key') {
      state.key = null;
    } else if (state.sel.kind === 'bridge') {
      delete state.bridges[state.sel.frontKey];
    } else {
      // Kai: removing a stone takes the loose rest of its arm with it
      var arr = resolveChain(state.sel.chain);
      if (arr) arr.splice(state.sel.idx);
      if (state.sel.chain.length === 0) dropKeyOnEdit();
      if (!resolveChain(state.activeFront)) state.activeFront = [];
    }
    state.sel = null;
    tok(120, .09, .08);
    refresh();
  }

  function reset() {
    state = freshState();
    hideVerdict();
    setTip('');
    refresh();
  }

  /* ================= test button flow (spec 15) ================= */

  function startTest() {
    if (!state.stones.length || !model.valid) return;
    state.phase = 'solving';
    state.sel = null;
    clearResult();
    applyTexts();
    var token = ++state.solveToken;
    setTimeout(function () {
      if (token !== state.solveToken) return;         // stale request, spec 7.1
      var ev = evaluateArch(state.stones, state.key, state.bridges);
      if (token !== state.solveToken) return;
      if (ev.state === 'incomplete' || ev.state === 'invalidGeometry') {
        state.phase = 'build';
        refresh();
        return;
      }
      state.result = ev.result;
      if (ev.state === 'error') {
        // spec 7.6: a solver error is a technical fault, never a collapse
        state.phase = 'build';
        setTask('Technischer Fehler bei der Kr&auml;ftepr&uuml;fung &ndash; der Bogen bleibt stehen.');
        setTip('');
        return;
      }
      if (ev.state === 'unstable') {
        setTask(ev.result.reason === 'FRICTION_LIMIT'
          ? 'An einer Fuge reicht die Reibung nicht aus.'
          : 'Die Druckkr&auml;fte finden keinen Weg durch alle Fugen.');
        setTip('');
        showVerdict('Kippt.');
        tok(140, .18, .09);
        state.phase = 'collapsing';
        state.world = createCollapseWorld(model.chain);
        return;
      }
      state.phase = 'result';
      applyTexts();
      if (ev.state === 'stable') { showVerdict('Trägt.'); tok(520, .16, .1); setTimeout(function () { tok(660, .2, .08); }, 110); }
      else { showVerdict('Trägt. Knapp.'); tok(420, .14, .09); }
    }, 60);
  }

  function replayCollapse() {
    if (!model.chain) return;
    hideVerdict();
    state.world = createCollapseWorld(model.chain);
    state.phase = 'collapsing';
    tok(140, .12, .08);
  }

  function backToBuild() {
    state.world = null;
    state.phase = 'build';
    clearResult();
    refresh();
  }

  /* ================= camera (auto zoom, Kai req 1) ================= */

  function worldBBox() {
    var bb = [-8, 0, 8, 9], i, j, p;  // minimum view so an empty stage looks sane
    function grow(pts, pad) {
      for (var q = 0; q < pts.length; q++) {
        p = pts[q];
        if (p[0] - pad < bb[0]) bb[0] = p[0] - pad;
        if (p[0] + pad > bb[2]) bb[2] = p[0] + pad;
        if (p[1] - pad < bb[1]) bb[1] = p[1] - pad;
        if (p[1] + pad > bb[3]) bb[3] = p[1] + pad;
      }
    }
    if (state.world) {
      for (i = 0; i < state.world.bodies.length; i++) {
        var b = state.world.bodies[i];
        var cs = Math.cos(b.ang), sn = Math.sin(b.ang);
        var pts = [];
        for (j = 0; j < b.outline.length; j++) {
          var q = b.outline[j];
          pts.push([b.pos[0] + q[0] * cs - q[1] * sn, b.pos[1] + q[0] * sn + q[1] * cs]);
        }
        grow(pts, 0.5);
      }
    } else {
      for (i = 0; i < model.left.length; i++) { grow(model.left[i].poly, 0.5); grow(model.right[i].poly, 0.5); }
      for (i = 0; i < model.bridgeItems.length; i++) grow(model.bridgeItems[i].item.poly, 0.5);
      if (model.key) grow(model.key.poly, 0.5);
      else if (state.offer && state.offerModel) grow(state.offerModel.key.poly, 3);
    }
    return bb;
  }

  function updateCamera(dt) {
    var bb = worldBBox();
    var bw = Math.max(4, bb[2] - bb[0]), bh = Math.max(3, bb[3] - bb[1]);
    var z = Math.min((STAGE.x1 - STAGE.x0) / bw, (GY - STAGE.top) / bh);
    z = clamp(z, ZMIN, ZMAX);
    var midX = (bb[0] + bb[2]) / 2;
    cam.tz = z;
    cam.tox = (STAGE.x0 + STAGE.x1) / 2 - midX * z;
    var k = Math.min(1, dt * 6);
    cam.z += (cam.tz - cam.z) * k;
    cam.ox += (cam.tox - cam.ox) * k;
  }

  function w2sX(x) { return cam.ox + x * cam.z; }
  function w2sY(y) { return GY - y * cam.z; }
  function w2s(p) { return [cam.ox + p[0] * cam.z, GY - p[1] * cam.z]; }
  function s2w(p) { return [(p[0] - cam.ox) / cam.z, (GY - p[1]) / cam.z]; }

  /* ================= drawing ================= */

  function drawPolyScreen(pts, fill, alpha, dy, shadow) {
    ctx.save();
    if (alpha != null) ctx.globalAlpha = alpha;
    var oy = dy || 0;
    if (shadow !== false) {
      ctx.beginPath();
      pts.forEach(function (p, i) { i ? ctx.lineTo(p[0] + 4, p[1] + oy + 5) : ctx.moveTo(p[0] + 4, p[1] + oy + 5); });
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

  function strokePolyScreen(pts, color, lw, alpha) {
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

  function rr(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  function buildPalette() {
    palItems = PAL_ORDER.map(function (k, i) {
      var t = STONES[k];
      var x = PAL.x0 + i * (PAL.w + PAL.gap);
      var tcx = x + PAL.w / 2, tcy = PAL.y + 26;
      var poly;
      if (k === 'wuerfel') {
        poly = [[tcx - 15, tcy - 13], [tcx + 15, tcy - 13], [tcx + 15, tcy + 17], [tcx - 15, tcy + 17]];
      } else if (t.sw) {
        // switch glyph: real union outline, entry face down, fan opening up
        var sw = makeSwitchItem({ Pin: [1, 0], u: [1, 0] }, k, false).item;
        var s = 10;
        poly = sw.poly.map(function (p) { return [tcx + (p[0] - 2) * s, PAL.y + 48 + p[1] * s]; });
      } else {
        var C = [tcx, tcy + t.Ra * 15 - 13];
        var a0 = rad(90 + t.angle / 2), a1 = rad(90 - t.angle / 2);
        poly = [];
        var n = Math.max(4, Math.round(t.angle / 5)), q;
        for (q = 0; q <= n; q++) {
          var a = a0 + (a1 - a0) * q / n;
          poly.push([C[0] + Math.cos(a) * t.Ra * 15, C[1] - Math.sin(a) * t.Ra * 15]);
        }
        for (q = n; q >= 0; q--) {
          var a2 = a0 + (a1 - a0) * q / n;
          poly.push([C[0] + Math.cos(a2) * t.Ri * 15, C[1] - Math.sin(a2) * t.Ri * 15]);
        }
      }
      return { key: k, rect: [x, PAL.y, PAL.w, PAL.h], poly: poly, tcx: tcx, tcy: tcy };
    });
  }

  function drawPalette() {
    var hot = state.phase === 'build' || state.phase === 'result';
    palItems.forEach(function (it) {
      ctx.save();
      rr(it.rect[0], it.rect[1], it.rect[2], it.rect[3], 14);
      ctx.fillStyle = 'rgba(245,236,226,.06)';
      ctx.fill();
      ctx.strokeStyle = state.sel && hot ? 'rgba(241,201,83,.65)' : 'rgba(245,236,226,.16)';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.restore();
      drawPolyScreen(it.poly, STONES[it.key].color, hot ? 1 : .35, 0, false);
      ctx.save();
      ctx.globalAlpha = hot ? .9 : .35;
      ctx.fillStyle = PAPIER;
      ctx.font = '600 14px "Ciutadella", system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(STONES[it.key].label, it.tcx, PAL.y + PAL.h - 10);
      ctx.restore();
    });
  }

  // action bar below the palette (Kai req 2 + 5)
  function computeBarButtons() {
    var btns = [];
    if (state.phase === 'settled') {
      btns.push({ id: 'replay', label: '↻  Nochmal fallen lassen', enabled: true });
      btns.push({ id: 'back', label: '←  Zurück zum Bauen', enabled: true });
    } else if (state.phase === 'collapsing' || state.phase === 'solving') {
      btns.push({ id: 'rotate', label: '↻  Drehen', enabled: false });
      btns.push({ id: 'remove', label: '✕  Entfernen', enabled: false });
      btns.push({ id: 'test', label: state.phase === 'solving' ? 'Prüfe …' : 'Bogen testen', enabled: false });
    } else {
      var hasSel = !!state.sel;
      btns.push({ id: 'rotate', label: '↻  Drehen', enabled: hasSel });
      btns.push({ id: 'remove', label: '✕  Entfernen', enabled: hasSel });
      var testable = state.stones.length > 0 && model.valid;
      btns.push({ id: 'test', label: state.phase === 'result' ? 'Nochmal testen' : 'Bogen testen',
                  enabled: testable, accent: testable && model.allClosed && state.phase !== 'result' });
    }
    var n = btns.length;
    var bw = n === 2 ? 250 : 216, gap = 16;
    var x0 = (W - (n * bw + (n - 1) * gap)) / 2;
    for (var i = 0; i < n; i++) {
      btns[i].rect = [x0 + i * (bw + gap), BAR.y, bw, BAR.h];
    }
    barButtons = btns;
  }

  function drawBar() {
    computeBarButtons();
    barButtons.forEach(function (b) {
      var r = b.rect;
      ctx.save();
      ctx.globalAlpha = b.enabled ? 1 : .35;
      rr(r[0], r[1], r[2], r[3], 12);
      ctx.fillStyle = b.accent ? 'rgba(241,201,83,.16)' : 'rgba(245,236,226,.07)';
      ctx.fill();
      ctx.strokeStyle = b.accent
        ? 'rgba(241,201,83,' + (.55 + .3 * Math.sin(state.pulse * 3)) + ')'
        : 'rgba(245,236,226,.3)';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = b.accent ? '#f1c953' : PAPIER;
      ctx.font = '600 16px "Ciutadella", system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(b.label, r[0] + r[2] / 2, r[1] + r[3] / 2 + 1);
      ctx.restore();
    });
  }

  function easeOut(t) { return 1 - Math.pow(1 - Math.min(1, t), 3); }

  function drawStoneItem(it, now) {
    var col = STONES[it.type].color;
    var p = easeOut((now - (it.born || 0)) / 380);
    var dy = (1 - p) * 120, al = .35 + .65 * p;
    var pts = it.poly.map(w2s);
    drawPolyScreen(pts, col, al, dy, p > .5);
    if (it.inverted || it.rot) {
      var c = w2s(it.centroid);
      ctx.save();
      ctx.globalAlpha = .85 * al;
      ctx.fillStyle = PAPIER;
      ctx.font = '600 ' + Math.max(11, Math.round(cam.z * 0.9)) + 'px "Ciutadella", system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('↻', c[0], c[1] + dy + 5);
      ctx.restore();
    }
    return pts;
  }

  // ghost markers at every open build front (Kai: Geistersteine an den
  // Weichen-Ausgängen; the glowing one is where the next stone will attach)
  var ghostHits = [];
  function drawGhosts() {
    ghostHits = [];
    if (state.phase !== 'build' && state.phase !== 'result') return;
    if (!model.fronts || model.fronts.length < 2) return;   // only once a switch exists
    var activeKey = keyStr(state.activeFront);
    for (var i = 0; i < model.fronts.length; i++) {
      var f = model.fronts[i];
      if (f.main && model.closed) continue;
      if (f.bridged || f.joined) continue;
      var isActive = keyStr(f.chain) === activeKey;
      var grounded = !f.main &&
        Math.abs(f.face.pin[1]) <= CONTACT_TOL_CM && Math.abs(f.face.pout[1]) <= CONTACT_TOL_CM;
      var T = [f.cur.u[1], -f.cur.u[0]];
      ghostMarker(f.face, T, f.chain, isActive, grounded);
      ghostMarker(mirFace(f.face), [-T[0], T[1]], f.chain, isActive, grounded);
    }
  }
  function ghostMarker(face, T, chain, active, grounded) {
    var mid = mul(add(face.pin, face.pout), 0.5);
    var c = w2s(add(mid, mul(T, 1.4)));
    var p1 = w2s(face.pin), p2 = w2s(face.pout);
    var col = grounded ? '#6cac53' : (active ? '#f1c953' : 'rgba(245,236,226,.55)');
    ctx.save();
    ctx.globalAlpha = active ? .95 : .6;
    ctx.strokeStyle = col;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); ctx.stroke();
    ctx.setLineDash([]);
    var r = active ? 13 + Math.sin(state.pulse * 3) * 1.5 : 11;
    ctx.beginPath();
    ctx.arc(c[0], c[1], r, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(20,16,14,.75)';
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = col;
    ctx.font = '600 15px "Ciutadella", system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('+', c[0], c[1] + 1);
    ctx.restore();
    ghostHits.push({ x: c[0], y: c[1], chain: chain });
  }

  function drawPressureLine() {
    var res = state.result;
    if (!res || !res.pressurePoints.length) return;
    var pts = res.pressurePoints.map(function (pp) { return w2s([pp.x, pp.y]); });
    ctx.save();
    ctx.globalAlpha = .9;
    ctx.strokeStyle = 'rgba(241,201,83,.8)';
    ctx.lineWidth = 2.5;
    // discrete support polygon: connect the loaded pressure points of each
    // body - a plain stone gets one segment, a switch a real force fork
    var byBody = {}, bi, bj;
    for (bi = 0; bi < res.contacts.length; bi++) {
      var solB = res.contacts[bi];
      if (!solB.loaded || !res.contactGeo) continue;
      var geo = res.contactGeo[bi];
      if (geo.a >= 0) (byBody[geo.a] = byBody[geo.a] || []).push(solB.point);
      if (geo.b >= 0) (byBody[geo.b] = byBody[geo.b] || []).push(solB.point);
    }
    for (var bk in byBody) {
      var bp = byBody[bk];
      for (bi = 0; bi < bp.length; bi++) {
        for (bj = bi + 1; bj < bp.length; bj++) {
          var q1 = w2s(bp[bi]), q2 = w2s(bp[bj]);
          ctx.beginPath();
          ctx.moveTo(q1[0], q1[1]);
          ctx.lineTo(q2[0], q2[1]);
          ctx.stroke();
        }
      }
    }
    for (var i = 0; i < pts.length; i++) {
      var hinge = res.pressurePoints[i].hinge;
      ctx.beginPath();
      ctx.arc(pts[i][0], pts[i][1], hinge ? 5.5 : 4, 0, Math.PI * 2);
      ctx.fillStyle = hinge ? '#e2543e' : '#141210';
      ctx.fill();
      ctx.strokeStyle = hinge ? '#f5ece2' : 'rgba(241,201,83,.95)';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
    // sliding-critical joints, spec 13.2
    if (res.contactGeo) {
      for (var j = 0; j < res.contacts.length; j++) {
        var sol = res.contacts[j];
        if (!sol.loaded || sol.rho <= 0.95) continue;
        var c = res.contactGeo[j];
        var midp = w2s(mul(add(c.p0, c.p1), 0.5));
        var td = mul(c.t, 1);
        ctx.save();
        ctx.strokeStyle = '#e2543e';
        ctx.lineWidth = 3;
        for (var q = -1; q <= 1; q += 2) {
          ctx.beginPath();
          ctx.moveTo(midp[0] + td[0] * 6 * q - td[1] * 5, midp[1] - td[1] * 6 * q - td[0] * 5);
          ctx.lineTo(midp[0] + td[0] * 6 * q + td[1] * 5, midp[1] - td[1] * 6 * q + td[0] * 5);
          ctx.stroke();
        }
        ctx.restore();
      }
    }
    ctx.restore();
  }

  function drawDebug() {
    var res = state.result;
    if (!res || !res.contactGeo) return;
    ctx.save();
    ctx.font = '11px monospace';
    for (var j = 0; j < res.contacts.length; j++) {
      var sol = res.contacts[j], c = res.contactGeo[j];
      var ends = [c.p0, c.p1];
      for (var k = 0; k < 2; k++) {
        var N = k ? sol.N1 : sol.N0, T = k ? sol.T1 : sol.T0;
        if (N < 1e-7 && Math.abs(T) < 1e-7) continue;
        var f = add(mul(c.n, N), mul(c.t, T));
        var p0 = w2s(ends[k]);
        var p1 = w2s(add(ends[k], mul(f, 8)));
        ctx.strokeStyle = '#4fd2ff';
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(p0[0], p0[1]); ctx.lineTo(p1[0], p1[1]); ctx.stroke();
      }
      var mp = w2s(mul(add(c.p0, c.p1), 0.5));
      ctx.fillStyle = '#4fd2ff';
      ctx.fillText('f' + j + (sol.loaded ? ' ρ' + sol.rho.toFixed(2) : ' –'), mp[0] + 6, mp[1] - 4);
    }
    if (model.chain) {
      for (var i = 0; i < model.chain.length; i++) {
        var cc = w2s(model.chain[i].centroid);
        ctx.fillStyle = '#ff7de9';
        ctx.beginPath(); ctx.arc(cc[0], cc[1], 2.5, 0, Math.PI * 2); ctx.fill();
      }
    }
    ctx.fillStyle = '#4fd2ff';
    ctx.fillText('alphaMax=' + (res.alphaMax != null ? res.alphaMax.toFixed(4) : '–') +
      '  rhoMax=' + (res.frictionUtilizationMax != null ? res.frictionUtilizationMax.toFixed(3) : '–') +
      '  ' + res.reason + '  ' + Math.round(res.solveMs) + 'ms', 14, 20);
    ctx.restore();
  }

  function draw(now) {
    ctx.clearRect(0, 0, W, H);

    // warm stage light
    var gr = ctx.createRadialGradient(W / 2, GY - 180, 60, W / 2, GY - 150, 460);
    gr.addColorStop(0, 'rgba(243,146,0,.12)');
    gr.addColorStop(1, 'rgba(243,146,0,0)');
    ctx.fillStyle = gr;
    ctx.fillRect(0, 0, W, H);

    // ground
    ctx.strokeStyle = 'rgba(245,236,226,.75)';
    ctx.lineWidth = 4;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(30, GY + 2);
    ctx.lineTo(W - 30, GY + 2);
    ctx.stroke();

    drawPalette();
    drawBar();

    if (state.world) {
      // collapse view: bodies from the physics engine
      for (var i = 0; i < state.world.bodies.length; i++) {
        var b = state.world.bodies[i];
        var cs = Math.cos(b.ang), sn = Math.sin(b.ang);
        var pts = [];
        for (var j = 0; j < b.outline.length; j++) {
          var q = b.outline[j];
          pts.push(w2s([b.pos[0] + q[0] * cs - q[1] * sn, b.pos[1] + q[0] * sn + q[1] * cs]));
        }
        drawPolyScreen(pts, STONES[b.type].color, 1, 0, true);
      }
    } else {
      var selPts = null, selPtsR = null, selKey = null;
      var selId = state.sel && state.sel.kind === 'pair' ? keyStr(state.sel.chain) + ':' + state.sel.idx : null;
      for (var s = 0; s < model.left.length; s++) {
        var ptsL = drawStoneItem(model.left[s], now);
        var ptsR = drawStoneItem(model.right[s], now);
        if (selId !== null && keyStr(model.left[s].chain) + ':' + model.left[s].idx === selId) {
          selPts = ptsL; selPtsR = ptsR;
        }
      }
      drawGhosts();
      for (var bi2 = 0; bi2 < model.bridgeItems.length; bi2++) {
        var bit = model.bridgeItems[bi2];
        var bpts = drawStoneItem(bit.item, now);
        if (state.sel && state.sel.kind === 'bridge' && state.sel.frontKey === bit.frontKey) {
          strokePolyScreen(bpts, 'rgba(241,201,83,.4)', 7);
          strokePolyScreen(bpts, PAPIER, 3);
        }
      }
      if (model.key) {
        var kb = model.key;
        kb.born = state.keyBorn || 0;
        var kpts = drawStoneItem(kb, now);
        if (state.sel && state.sel.kind === 'key') selKey = kpts;
      } else if (state.offer && state.offerModel && state.phase === 'build') {
        var og = state.offerModel.key;
        var bob = Math.sin(state.pulse * 2.4) * 7;
        var oal = .55 + .18 * Math.sin(state.pulse * 3.2);
        var opts = og.poly.map(w2s).map(function (p) { return [p[0], p[1] - 34 + bob]; });
        drawPolyScreen(opts, STONES[state.offer].color, oal, 0, false);
        strokePolyScreen(opts, 'rgba(245,236,226,.8)', 2, oal);
      }
      if (selPts) {
        strokePolyScreen(selPts, 'rgba(241,201,83,.4)', 7);
        strokePolyScreen(selPtsR, 'rgba(241,201,83,.4)', 7);
        strokePolyScreen(selPts, PAPIER, 3);
        strokePolyScreen(selPtsR, PAPIER, 3);
      }
      if (selKey) {
        strokePolyScreen(selKey, 'rgba(241,201,83,.4)', 7);
        strokePolyScreen(selKey, PAPIER, 3);
      }
      if (state.phase === 'result') drawPressureLine();
      if (dbg && state.result) drawDebug();
    }

    // headline: missing angle / status, spec 4.5 + 13.1
    var txt = null, col = 'rgba(245,236,226,.95)';
    if (state.phase === 'collapsing' || state.phase === 'settled') {
      txt = state.phase !== 'settled' ? null
        : (state.world && state.world.outcome === 'wedged'
            ? 'Der Bogen hat sich verkeilt.' : 'Der Bogen ist zusammengefallen.');
      col = '#f39200';
    } else if (state.phase === 'solving') {
      txt = 'Kräfte werden geprüft …';
    } else if (state.phase === 'result' && state.result) {
      txt = state.result.status === 'stable' ? 'Die Konstruktion steht. ✓' : 'Die Konstruktion steht – gerade so.';
      col = state.result.status === 'stable' ? '#f1c953' : '#f39200';
    } else if (state.stones.length) {
      if (!model.valid) {
        txt = 'Steine überschneiden sich'; col = '#f39200';
      } else if (model.allClosed) {
        txt = 'Konstruktion geschlossen ✓'; col = '#f1c953';
      } else if (state.activeFront.length) {
        // per-arm readout (Kai): the branch currently being built
        var af = null;
        for (var fi2 = 0; fi2 < model.fronts.length; fi2++) {
          if (!model.fronts[fi2].main && keyStr(model.fronts[fi2].chain) === keyStr(state.activeFront)) {
            af = model.fronts[fi2]; break;
          }
        }
        if (af && (af.grounded || af.joined || af.bridged)) { txt = 'Ast angeschlossen ✓'; col = '#6cac53'; }
        else if (af) {
          var dd = Math.abs(normDeg(faceDirDeg(af.face)));
          dd = Math.min(dd, 180 - dd);
          txt = 'Ast: noch ' + fmtDeg(dd) + '° bis waagerecht';
        }
      } else {
        var M = model.missing;
        if (model.closed) { txt = 'Bogen geschlossen ✓'; col = '#f1c953'; }
        else if (M > CLOSURE_TOL_DEG) txt = 'Noch ' + fmtDeg(M) + '° offen';
        else if (M < -CLOSURE_TOL_DEG) { txt = fmtDeg(-M) + '° zu viel'; col = '#f39200'; }
      }
    }
    if (txt) {
      ctx.save();
      ctx.fillStyle = col;
      ctx.font = '700 28px "Ciutadella", system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(txt, W / 2, 64);
      ctx.restore();
    }
  }

  /* ================= input ================= */

  function canvasPos(e) {
    var r = canvas.getBoundingClientRect();
    return [(e.clientX - r.left) * (W / r.width), (e.clientY - r.top) * (H / r.height)];
  }

  function hitBar(p) {
    for (var i = 0; i < barButtons.length; i++) {
      var r = barButtons[i].rect;
      if (p[0] >= r[0] && p[0] <= r[0] + r[2] && p[1] >= r[1] && p[1] <= r[1] + r[3]) return barButtons[i];
    }
    return null;
  }

  function onDown(e) {
    var p = canvasPos(e), i;

    var bb = hitBar(p);
    if (bb) {
      if (!bb.enabled) return;
      if (bb.id === 'rotate') invertSel();
      else if (bb.id === 'remove') removeSel();
      else if (bb.id === 'test') startTest();
      else if (bb.id === 'replay') replayCollapse();
      else if (bb.id === 'back') backToBuild();
      return;
    }
    if (state.phase === 'solving' || state.phase === 'collapsing' || state.phase === 'settled') return;

    // palette
    for (i = 0; i < palItems.length; i++) {
      var r = palItems[i].rect;
      if (p[0] >= r[0] && p[0] <= r[0] + r[2] && p[1] >= r[1] && p[1] <= r[1] + r[3]) {
        var k = palItems[i].key;
        if (state.sel) { swapSel(k); return; }
        // Kai req 3: a stone that exactly closes the gap goes in ONCE (keystone);
        // only on the main front, and a switch can never be the closing stone
        if (!state.key && state.stones.length && state.activeFront.length === 0 && !STONES[k].sw &&
            Math.abs(STONES[k].angle - model.missing) <= CLOSURE_TOL_DEG) {
          var probe = archModel(state.stones, k);
          if (probe.closed && probe.joinGap <= CONTACT_TOL_CM) {
            state.keyBorn = NOW();
            placeKeystone(k);
            return;
          }
          setTask('Der Winkel passt, aber die Fugen treffen sich nicht &ndash; die Form klafft.');
          return;
        }
        if (state.phase === 'result') { state.phase = 'build'; clearResult(); }
        addStone(k);
        return;
      }
    }

    // ghost fronts: choose where the next stone attaches
    for (i = 0; i < ghostHits.length; i++) {
      if (Math.hypot(p[0] - ghostHits[i].x, p[1] - ghostHits[i].y) < Math.max(26, cam.z * 1.2)) {
        state.activeFront = ghostHits[i].chain.slice();
        state.sel = null;
        tok(360, .05, .05);
        applyTexts();
        return;
      }
    }

    var wp = s2w(p);

    // floating keystone offer
    if (state.offer && state.offerModel && state.phase === 'build') {
      var og = state.offerModel.key;
      var oc = w2s(og.centroid);
      if (Math.hypot(p[0] - oc[0], p[1] - (oc[1] - 34)) < Math.max(44, cam.z * 2.4)) {
        state.keyBorn = NOW();
        placeKeystone(state.offer);
        return;
      }
    }

    // bridge stones
    for (i = 0; i < model.bridgeItems.length; i++) {
      if (inPoly(wp, model.bridgeItems[i].item.poly)) {
        if (state.phase === 'result') { state.phase = 'build'; clearResult(); }
        state.sel = { kind: 'bridge', frontKey: model.bridgeItems[i].frontKey };
        tok(320, .05, .05);
        applyTexts();
        return;
      }
    }

    // keystone body
    if (model.key && inPoly(wp, model.key.poly)) {
      if (state.phase === 'result') { state.phase = 'build'; clearResult(); }
      state.sel = { kind: 'key' };
      tok(320, .05, .05);
      applyTexts();
      return;
    }

    // stones (either side selects the mirrored pair)
    for (i = model.left.length - 1; i >= 0; i--) {
      if (inPoly(wp, model.left[i].poly) || inPoly(wp, model.right[i].poly)) {
        var nowMs = NOW();
        var it = model.left[i];
        var tapKey = keyStr(it.chain) + ':' + it.idx;
        if (state.phase === 'result') { state.phase = 'build'; clearResult(); }
        if (state.sel && state.sel.kind === 'pair' &&
            state.lastTapKey === tapKey && nowMs - state.lastTap < 380) {
          removeSel();
          return;
        }
        state.sel = { kind: 'pair', chain: it.chain.slice(), idx: it.idx };
        state.lastTap = nowMs;
        state.lastTapKey = tapKey;
        tok(320, .05, .05);
        applyTexts();
        return;
      }
    }

    // empty space
    if (state.sel) { state.sel = null; applyTexts(); }
  }

  /* ================= main loop ================= */

  var visible = false, lastT = 0, dynAcc = 0;

  function tick(now) {
    requestAnimationFrame(tick);
    if (!visible || !state) return;
    var dt = Math.min(0.034, (now - lastT) / 1000 || 0.016);
    lastT = now;
    state.pulse += dt;

    if (state.phase === 'collapsing' && state.world) {
      dynAcc = Math.min(dynAcc + dt, 4 * DYN.dt);
      while (dynAcc >= DYN.dt) {
        stepWorld(state.world);
        dynAcc -= DYN.dt;
      }
      if (state.world.settled) {
        state.phase = 'settled';
        showVerdict(state.world.outcome === 'wedged' ? 'Verkeilt.' : 'Eingestürzt.');
        tok(90, .2, .1);
        applyTexts();
      }
    }
    updateCamera(dt);
    draw(now);
  }

  /* ================= self tests (spec 16) ================= */

  function selfTest() {
    var cases = [], t0;
    function chk(name, ok, info) { cases.push({ name: name, ok: !!ok, info: info || '' }); }
    function stonesOf(types) {
      return types.map(function (t) { return { type: t, inverted: false, born: 0 }; });
    }

    // 16.1 geometry
    var expect = {
      gelb: { A: 2 * Math.PI, rs: 28 / (3 * Math.PI) },
      orange: { A: rad(30) / 2 * 28, rs: 4 * Math.sin(rad(15)) / (3 * rad(30)) * (296 / 28) },
      gruen: { A: rad(15) / 2 * 60, rs: 4 * Math.sin(rad(7.5)) / (3 * rad(15)) * (1352 / 60) }
    };
    ['gelb', 'orange', 'gruen'].forEach(function (k) {
      chk('area ' + k, Math.abs(stoneArea(k) - expect[k].A) < 1e-9, stoneArea(k).toFixed(4));
      chk('centroid ' + k, Math.abs(stoneCentroidRadius(k) - expect[k].rs) < 1e-9, stoneCentroidRadius(k).toFixed(4));
    });
    chk('mass ratio orange', Math.abs(stoneMass('orange') / stoneMass('gelb') - 1.16667) < 1e-3);
    chk('mass ratio gruen', Math.abs(stoneMass('gruen') / stoneMass('gelb') - 1.25) < 1e-3);

    var half = computeHalf(stonesOf(['gelb', 'orange', 'gruen']));
    var okFaces = true, okLen = true, i;
    for (i = 0; i < half.items.length; i++) {
      var it = half.items[i];
      if (Math.abs(dist(it.startFace.pin, it.startFace.pout) - 2) > 1e-9) okLen = false;
      if (Math.abs(dist(it.endFace.pin, it.endFace.pout) - 2) > 1e-9) okLen = false;
      if (i > 0 && faceGap(half.items[i - 1].endFace, it.startFace) > 1e-9) okFaces = false;
    }
    chk('faces are exactly 2cm', okLen);
    chk('joints are flush (fugengenau)', okFaces);
    chk('turn sum', Math.abs(half.sumDeg - 105) < 1e-9, half.sumDeg + '°');

    var st2 = stonesOf(['gelb', 'orange', 'gruen']);
    st2[1] = { type: 'wuerfel', inverted: false, born: 0 };
    var half2 = computeHalf(st2);
    var okRe = true;
    for (i = 1; i < half2.items.length; i++) {
      if (faceGap(half2.items[i - 1].endFace, half2.items[i].startFace) > 1e-9) okRe = false;
    }
    chk('rebuild after swap keeps joints flush', okRe);

    var hInv = computeHalf([{ type: 'gelb', inverted: true, born: 0 }]);
    chk('inverted stone subtracts its angle', Math.abs(hInv.sumDeg + 60) < 1e-9, hInv.sumDeg + '°');

    // 16.2 statics - synthetic cases
    function groundContact(p0, p1, bodyIdx, mu) {
      var t = norm(sub(p1, p0));
      return { id: 'g', a: -1, b: bodyIdx, p0: p0, p1: p1, t: t, n: perp(t), L: dist(p0, p1), mu: mu, kind: 'stone-ground' };
    }
    // A: single block on level support
    var bodyA = { centroid: [1, 1], massRel: 4 };
    var rA = analyzeStatics([bodyA], [groundContact([0, 0], [2, 0], 0, 0.5)]);
    chk('A: single block feasible', rA.feasible && rA.status === 'stable', rA.status);
    chk('A: pressure point under centroid', rA.contacts[0].loaded && Math.abs(rA.contacts[0].sRel - 0.5) < 1e-6,
      'sRel=' + (rA.contacts[0].sRel != null ? rA.contacts[0].sRel.toFixed(4) : '–'));
    chk('A: no tangential force', Math.abs(rA.contacts[0].T0 + rA.contacts[0].T1) < 1e-7);

    // B: two stacked blocks
    var b0 = { centroid: [1, 1], massRel: 4 }, b1 = { centroid: [1, 3], massRel: 4 };
    var cB0 = groundContact([0, 0], [2, 0], 0, 0.5);
    var tB = norm([1, 0]);
    var cB1 = { id: 'j', a: 0, b: 1, p0: [0, 2], p1: [2, 2], t: tB, n: [0, 1], L: 2, mu: 0.5, kind: 'stone-stone' };
    var rB = analyzeStatics([b0, b1], [cB0, cB1]);
    chk('B: stacked blocks feasible', rB.feasible, rB.status);
    chk('B: both pressure points inside', rB.contacts.every(function (c) { return c.loaded && c.sRel > 0.01 && c.sRel < 0.99; }));

    // C: centroid outside the support -> no compression solution
    var rC = analyzeStatics([{ centroid: [3, 1], massRel: 1 }], [groundContact([0, 0], [2, 0], 0, 0.5)]);
    chk('C: centroid outside -> unstable', !rC.feasible && rC.status === 'unstable', rC.reason);
    chk('C: reason is compression, not friction', rC.reason === 'NO_COMPRESSION_EQUILIBRIUM', rC.reason);

    // D: block on incline, mu vs tan(beta)
    function incline(beta, mu) {
      var t = [Math.cos(beta), Math.sin(beta)];
      var n = perp(t);
      var mid = [Math.cos(beta), Math.sin(beta)];
      var c = add(mid, n);
      var body = { centroid: c, massRel: 1 };
      var con = { id: 'i', a: -1, b: 0, p0: [0, 0], p1: mul(t, 2), t: t, n: n, L: 2, mu: mu, kind: 'stone-ground' };
      return analyzeStatics([body], [con]);
    }
    var beta = rad(20);
    var rD1 = incline(beta, 0.5), rD2 = incline(beta, 0.3);
    chk('D: incline stable when mu > tan(beta)', rD1.feasible, rD1.status);
    chk('D: incline slides when mu < tan(beta)', !rD2.feasible && rD2.reason === 'FRICTION_LIMIT', rD2.reason);

    // E/F/G: pure arches through the real pipeline
    t0 = NOW();
    var evE = evaluateArch(stonesOf(['gelb']), 'gelb');       // 3 x 60°
    var msE = NOW() - t0;
    chk('E: 3 yellow closes', evE.state !== 'incomplete' && evE.state !== 'invalidGeometry', evE.state);
    chk('E: 3 yellow carries', evE.state === 'stable' || evE.state === 'critical',
      evE.state + ' α=' + (evE.result && evE.result.alphaMax != null ? evE.result.alphaMax.toFixed(3) : '–'));
    var evE2 = evaluateArch(stonesOf(['gelb']), 'gelb');
    chk('E: reproducible', evE2.result && evE.result &&
      evE2.result.alphaMax === evE.result.alphaMax &&
      JSON.stringify(evE2.result.pressurePoints) === JSON.stringify(evE.result.pressurePoints));

    t0 = NOW();
    var evF = evaluateArch(stonesOf(['orange', 'orange', 'orange']), null);  // 6 x 30°
    var msF = NOW() - t0;
    chk('F: 6 orange closes + carries', evF.state === 'stable' || evF.state === 'critical',
      evF.state + ' α=' + (evF.result && evF.result.alphaMax != null ? evF.result.alphaMax.toFixed(3) : '–'));

    t0 = NOW();
    var evG = evaluateArch(stonesOf(['gruen', 'gruen', 'gruen', 'gruen', 'gruen', 'gruen']), null); // 12 x 15°
    var msG = NOW() - t0;
    chk('G: 12 green closes + solver runs', evG.state !== 'incomplete' && evG.state !== 'invalidGeometry' && evG.state !== 'error',
      evG.state + ' α=' + (evG.result && evG.result.alphaMax != null ? evG.result.alphaMax.toFixed(3) : '–') + ' ' + Math.round(msG) + 'ms');

    // H (adapted on Kai's request): open structures are testable - the solver
    // judges loose legs and branches instead of a closure gate blocking them
    var evH = evaluateArch(stonesOf(['gelb', 'gelb']), null);
    chk('H: open arch is testable', evH.state !== 'incomplete' && evH.state !== 'error', evH.state);
    var evH2 = evaluateArch(stonesOf(['gelb']), null);
    chk('H2: single open pair is testable', evH2.state !== 'incomplete' && evH2.state !== 'error',
      evH2.state + ' missing=' + evH2.missing);
    chk('H3: empty stage stays incomplete', evaluateArch([], null).state === 'incomplete');

    // I: low ground friction -> sliding diagnosis
    var muSave = calibration.stoneGroundMu;
    calibration.stoneGroundMu = 0.05;
    var evI = evaluateArch(stonesOf(['gelb']), 'gelb');
    calibration.stoneGroundMu = muSave;
    chk('I: low ground friction -> FRICTION_LIMIT', evI.state === 'unstable' && evI.result.reason === 'FRICTION_LIMIT',
      evI.state + '/' + (evI.result ? evI.result.reason : '–'));

    // keystone single placement (Kai req 3)
    var mK = archModel(stonesOf(['gelb']), 'gelb');
    chk('keystone: single stone, 3 bodies, 4 contacts', mK.closed && mK.chain.length === 3 && mK.contactSpecs.length === 4,
      'bodies=' + (mK.chain ? mK.chain.length : 0) + ' contacts=' + (mK.contactSpecs ? mK.contactSpecs.length : 0));

    // mixed stone types close via the symmetric ground slide (real-floor behaviour)
    var mMix = archModel(stonesOf(['orange', 'orange']), 'gelb');   // 2x30 legs + 60 key
    chk('mixed arch closes fugengenau (slide=' + (mMix.shift ? mMix.shift.toFixed(3) : '0') + 'cm)',
      mMix.closed && mMix.joinGap <= CONTACT_TOL_CM, 'gap=' + mMix.joinGap.toFixed(5));
    var evMix = evaluateArch(stonesOf(['orange', 'orange']), 'gelb');
    chk('mixed arch reaches the solver', evMix.state !== 'incomplete' && evMix.state !== 'invalidGeometry' && evMix.state !== 'error', evMix.state);

    // cube keystone: diamond apex with both lower faces carrying (45 per side + 90)
    var mSpitz = archModel(stonesOf(['orange', 'gruen']), 'wuerfel');
    chk('cube keystone closes as diamond', mSpitz.closed && mSpitz.joinGap <= CONTACT_TOL_CM,
      'gap=' + (mSpitz.joinGap === Infinity ? 'inf' : mSpitz.joinGap.toFixed(5)) + ' missing=' + mSpitz.missing);

    // an unstable but geometrically valid arch must collapse and settle deterministically
    var sUnst = [{ type: 'gelb', inverted: false, born: 0 }, { type: 'gelb', inverted: false, born: 0 },
                 { type: 'orange', inverted: true, born: 0 }];
    var evU = evaluateArch(sUnst, null);
    chk('unstable demo config is detected', evU.state === 'unstable', evU.state + '/' + (evU.result ? evU.result.reason : '-'));
    if (evU.state === 'unstable') {
      var wU = createCollapseWorld(evU.model.chain);
      var steps = 0;
      while (!wU.settled && steps < 120 * DYN.maxSeconds) { stepWorld(wU); steps++; }
      var sane = wU.settled;
      for (i = 0; i < wU.bodies.length; i++) {
        var bU = wU.bodies[i];
        if (!isFinite(bU.pos[0]) || !isFinite(bU.pos[1]) || bU.pos[1] < -6 || bU.pos[1] > 60) sane = false;
      }
      chk('collapse settles with sane positions', sane, 't=' + wU.time.toFixed(2) + 's');
    }

    // open combo with inverted stones: must stay incomplete, never fake-close
    var sInv2 = [{ type: 'gelb', inverted: false, born: 0 }, { type: 'orange', inverted: true, born: 0 },
                 { type: 'orange', inverted: false, born: 0 }, { type: 'gelb', inverted: false, born: 0 }];
    var evX = evaluateArch(sInv2, null);   // per side 60-30+30+60 = 120 -> missing -60
    chk('X: open combo evaluates honestly', evX.state !== 'incomplete' && evX.state !== 'error', evX.state);

    // Weichensteine: mirrored-twin union, one entry, two exits
    var swI = makeSwitchItem({ Pin: [0, 0], u: [1, 0] }, 'wgelb', false).item;
    chk('switch: exit faces are 2cm', Math.abs(dist(swI.endFace.pin, swI.endFace.pout) - 2) < 1e-9 &&
      Math.abs(dist(swI.exitSideFace.pin, swI.exitSideFace.pout) - 2) < 1e-9);
    chk('switch: union area between 1x and 2x segment',
      swI.area > stoneArea('gelb') && swI.area < 2 * stoneArea('gelb'), swI.area.toFixed(3));
    chk('switch: centroid on the symmetry axis', Math.abs(swI.centroid[0] - 1) < 1e-6, swI.centroid[0].toFixed(5));
    // real rotation: the stone re-seats by one of its arms on the parent joint
    var so1 = makeSwitchItem({ Pin: [0, 0], u: [1, 0] }, 'worange', 1).item;
    var so2 = makeSwitchItem({ Pin: [0, 0], u: [1, 0] }, 'worange', 2).item;
    var so0 = makeSwitchItem({ Pin: [0, 0], u: [1, 0] }, 'worange', 0).item;
    chk('switch rot: main exit visibly re-seats',
      faceGap(so0.endFace, so1.endFace) > 1 && faceGap(so1.endFace, so2.endFace) > 1,
      faceGap(so0.endFace, so1.endFace).toFixed(2) + 'cm');
    chk('switch rot: free faces stay 2cm',
      Math.abs(dist(so1.endFace.pin, so1.endFace.pout) - 2) < 1e-9 &&
      Math.abs(dist(so1.exitSideFace.pin, so1.exitSideFace.pout) - 2) < 1e-9 &&
      Math.abs(dist(so2.endFace.pin, so2.endFace.pout) - 2) < 1e-9);
    chk('switch rot: the two arm seats mirror each other', Math.abs(so1.turn + so2.turn) < 1e-6,
      'turns=' + so1.turn.toFixed(1) + '/' + so2.turn.toFixed(1));
    var mSw0 = archModel([{ type: 'worange', rot: 0, born: 0, side: [] }], null);
    var mSw1 = archModel([{ type: 'worange', rot: 1, born: 0, side: [] }], null);
    chk('rotated switch changes the main-path angle', Math.abs(mSw0.missing - mSw1.missing) > 1,
      'missing ' + fmtDeg(mSw0.missing) + ' -> ' + fmtDeg(mSw1.missing));
    // wgelb raster property: its three seats offer the same face spots again
    var yg0 = makeSwitchItem({ Pin: [0, 0], u: [1, 0] }, 'wgelb', 0).item;
    var yg1 = makeSwitchItem({ Pin: [0, 0], u: [1, 0] }, 'wgelb', 1).item;
    chk('wgelb rot: faces land on the same raster (branches survive)',
      faceGap(yg0.endFace, yg1.endFace) < JOIN_TOL_CM && faceGap(yg0.exitSideFace, yg1.exitSideFace) < JOIN_TOL_CM,
      faceGap(yg0.endFace, yg1.endFace).toFixed(3) + 'cm');

    // portal: springer switches, side branches arc back down to the ground
    var sideArc = [];
    for (i = 0; i < 5; i++) sideArc.push({ type: 'orange', inverted: true, born: 0 });
    var portal = [
      { type: 'worange', inverted: false, born: 0, side: sideArc },
      { type: 'orange', inverted: false, born: 0 },
      { type: 'orange', inverted: false, born: 0 }
    ];
    var mPor = archModel(portal, null);
    var grounds = mPor.contactSpecs ? mPor.contactSpecs.filter(function (c) { return c.kind === 'stone-ground'; }).length : 0;
    chk('portal: closes and side feet reach the ground', mPor.closed && mPor.valid && grounds === 4,
      'closed=' + mPor.closed + ' errs=' + (mPor.errors || []).join(',') + ' grounds=' + grounds);
    var evPor = evaluateArch(portal, null);
    chk('portal: solver judges the whole structure',
      evPor.state === 'stable' || evPor.state === 'critical' || evPor.state === 'unstable',
      evPor.state + (evPor.result && evPor.result.alphaMax != null ? ' a=' + evPor.result.alphaMax.toFixed(3) : ''));

    // cantilever stub hanging off a switch: honest, must evaluate without error
    var canti = [
      { type: 'worange', inverted: false, born: 0, side: [{ type: 'orange', inverted: true, born: 0 }] },
      { type: 'gelb', inverted: false, born: 0 }
    ];
    var evCan = evaluateArch(canti, null);
    chk('cantilever branch evaluates', evCan.state !== 'error', evCan.state);

    // Drehen relocates the through-path: the main front of a rotated switch moves
    var rotA = archModel([{ type: 'worange', rot: 0, born: 0, side: [] }], null);
    var rotB = archModel([{ type: 'worange', rot: 1, born: 0, side: [] }], null);
    var fA = null, fB = null;
    rotA.fronts.forEach(function (f) { if (f.main) fA = f; });
    rotB.fronts.forEach(function (f) { if (f.main) fB = f; });
    chk('switch rotation moves the main front', fA && fB && dist(fA.face.pin, fB.face.pin) > 1,
      fA && fB ? dist(fA.face.pin, fB.face.pin).toFixed(2) + 'cm apart' : 'missing front');

    // three switches: apex switch pair closed on top by a single bridge switch
    var triSw = [
      { type: 'orange', inverted: false, born: 0 },
      { type: 'orange', inverted: false, born: 0 },
      { type: 'worange', inverted: false, born: 0, side: [] }
    ];
    var mTri0 = archModel(triSw, null);
    chk('apex switch pair closes the bow', mTri0.closed, 'missing=' + mTri0.missing);
    var triFront = null;
    mTri0.fronts.forEach(function (f) { if (!f.main) triFront = f; });
    var triFit = triFront ? fitBridge(triFront.face, 'worange') : null;
    chk('bridge switch fits on the apex pair', !!triFit && triFit.gap <= JOIN_TOL_CM,
      triFit ? 'gap=' + triFit.gap.toFixed(3) + 'cm' : 'no front');
    var mTri = archModel(triSw, null, { '2': { type: 'worange', born: 0 } });
    chk('three switches joined: bridge attached + valid', mTri.bridgeItems.length === 1 && mTri.valid,
      'bridges=' + mTri.bridgeItems.length + ' errs=' + mTri.errors.join(','));
    var evTri = evaluateArch(triSw, null, { '2': { type: 'worange', born: 0 } });
    chk('three-switch structure reaches the solver',
      evTri.state === 'stable' || evTri.state === 'critical' || evTri.state === 'unstable',
      evTri.state + (evTri.result && evTri.result.alphaMax != null ? ' a=' + evTri.result.alphaMax.toFixed(3) : ''));
    chk('tri-switch: structurally all closed', mTri.allClosed === true, 'allClosed=' + mTri.allClosed);

    // plain stone as on-axis bridge: a synthetic 120deg front pair fits a gelb exactly
    var sFront = { pin: [-1, 4], pout: [-1 + 2 * Math.cos(rad(120)), 4 + 2 * Math.sin(rad(120))] };
    var pb = fitBridge(sFront, 'gelb');
    chk('plain stone fits as on-axis bridge', !!pb && pb.gap < 0.01, pb ? 'gap=' + pb.gap.toFixed(4) : 'null');

    // Kai's cloverleaf: springer switch, arms converging, closed by a rotated
    // switch - must be recognized as closed (front joint) and be testable
    var clover = [
      { type: 'worange', inverted: false, rot: 0, born: 0,
        side: [{ type: 'orange', inverted: false, born: 0 }, { type: 'orange', inverted: false, born: 0 }] },
      { type: 'orange', inverted: true, born: 0 },
      { type: 'orange', inverted: true, born: 0 },
      { type: 'worange', inverted: false, rot: 2, born: 0, side: [] }
    ];
    var mClover = archModel(clover, null);
    var evClover = evaluateArch(clover, null);
    chk('cloverleaf: loop closure via rotated switch is detected',
      mClover.valid && mClover.closed, 'closed=' + mClover.closed + ' errs=' + mClover.errors.join(','));
    chk('cloverleaf: testable despite the open stub',
      evClover.state === 'stable' || evClover.state === 'critical' || evClover.state === 'unstable', evClover.state);

    // half-built portal: main bow closed, side branch mid-air -> still testable
    var evOpenPortal = evaluateArch([
      { type: 'worange', inverted: false, born: 0, side: [{ type: 'orange', inverted: true, born: 0 }] },
      { type: 'orange', inverted: false, born: 0 },
      { type: 'orange', inverted: false, born: 0 }
    ], null);
    chk('open branch does not block the test', evOpenPortal.state !== 'incomplete' && evOpenPortal.state !== 'error',
      evOpenPortal.state);

    // dynamics determinism (spec 16.3)
    var mE = archModel(stonesOf(['gelb']), 'gelb');
    var w1 = createCollapseWorld(mE.chain), w2 = createCollapseWorld(mE.chain);
    for (i = 0; i < 240; i++) { stepWorld(w1); stepWorld(w2); }
    var same = true;
    for (i = 0; i < w1.bodies.length; i++) {
      if (w1.bodies[i].pos[0] !== w2.bodies[i].pos[0] ||
          w1.bodies[i].pos[1] !== w2.bodies[i].pos[1] ||
          w1.bodies[i].ang !== w2.bodies[i].ang) same = false;
    }
    chk('collapse simulation is deterministic', same);

    var pass = cases.filter(function (c) { return c.ok; }).length;
    var out = {
      pass: pass, fail: cases.length - pass, total: cases.length,
      timings: { solve3yellowMs: Math.round(msE), solve6orangeMs: Math.round(msF), solve12greenMs: Math.round(msG) },
      cases: cases
    };
    return out;
  }

  /* ================= init ================= */

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

    if (holdBtn) { holdBtn.hidden = true; holdBtn.style.display = 'none'; }
    if (holdLabel) holdLabel.hidden = true;
    if (resetBtn) {
      resetBtn.hidden = false;
      resetBtn.textContent = 'Von vorn';
      resetBtn.addEventListener('click', reset);
    }
    if (verdictEl) verdictEl.style.opacity = 0;

    buildPalette();
    state = freshState();
    refresh();

    canvas.addEventListener('pointerdown', onDown);

    var io = new IntersectionObserver(function (entries) {
      visible = entries[0].isIntersecting;
      if (visible) lastT = NOW();
    }, { threshold: 0.15 });
    io.observe(canvas);

    requestAnimationFrame(tick);
  }

  if (IS_BROWSER) {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
  }

  var API = {
    reset: function () { if (state) reset(); },
    selfTest: selfTest,
    debug: function (on) { dbg = on !== false; },
    calibration: calibration,
    massCalibration: massCalibration,
    evaluate: function () { return evaluateArch(state ? state.stones : [], state ? state.key : null); },
    // read-only introspection for tests and debugging
    probe: function () {
      if (!state) return null;
      return {
        phase: state.phase, offer: state.offer, key: state.key,
        stones: state.stones.map(function (s) { return { type: s.type, inverted: s.inverted }; }),
        closed: model ? model.closed : false, valid: model ? model.valid : false,
        allClosed: model ? model.allClosed : false,
        missing: model ? model.missing : null,
        camera: { z: cam.z, ox: cam.ox, gy: GY },
        activeFront: state.activeFront,
        fronts: model && model.fronts ? model.fronts.map(function (f) {
          return { chain: f.chain, main: f.main, mid: mul(add(f.face.pin, f.face.pout), 0.5),
                   grounded: !!f.grounded, joined: !!f.joined, bridged: !!f.bridged };
        }) : [],
        ghosts: ghostHits.map(function (g) { return { x: g.x, y: g.y, chain: g.chain }; }),
        centroids: model ? model.left.map(function (it) { return it.centroid; }) : [],
        centroidsR: model ? model.right.map(function (it) { return it.centroid; }) : [],
        keyCentroid: model && model.key ? model.key.centroid : null,
        offerCentroid: state.offerModel ? state.offerModel.key.centroid : null,
        result: state.result ? { status: state.result.status, reason: state.result.reason,
          alphaMax: state.result.alphaMax, rhoMax: state.result.frictionUtilizationMax,
          points: state.result.pressurePoints.length, solveMs: state.result.solveMs } : null,
        settled: state.world ? state.world.settled : null, worldTime: state.world ? state.world.time : null,
        outcome: state.world ? state.world.outcome : null,
        bridges: Object.keys(state.bridges),
        bar: barButtons.map(function (b) { return { id: b.id, enabled: b.enabled, rect: b.rect }; })
      };
    },
    _internals: {
      STONES: STONES, stepStone: stepStone, computeHalf: computeHalf,
      archModel: archModel, detectContacts: detectContacts,
      analyzeStatics: analyzeStatics, evaluateArch: evaluateArch,
      lpSolve: lpSolve, createCollapseWorld: createCollapseWorld, stepWorld: stepWorld,
      bodiesOverlap: bodiesOverlap, fitBridge: fitBridge, makeSwitchItem: makeSwitchItem,
      DYN: DYN
    }
  };
  if (IS_BROWSER) window.KajuBauprobe = API;
  else if (typeof module !== 'undefined' && module.exports) module.exports = API;
})();
