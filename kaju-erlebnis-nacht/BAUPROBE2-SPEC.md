# Bauprobe — Variante 2: Einseitiger Bau + verschiebbare Spiegelachse

**Stand:** 2026-07-05 · **Status:** Design freigegeben (Grobablauf + Achsen-Modell A), Spec im Review

## Ziel

Eine **zweite** Bauprobe-Variante unterhalb der bestehenden. Der Nutzer baut **nur eine Hälfte**
des Bogens — Stein für Stein per **Drag aus dem Vorrat** — und schließt den Bogen dann über eine
**verschiebbare Spiegelachse** und einen **Spiegeln-Knopf**. Didaktik: Symmetrie wird aktiv
*gefunden* statt automatisch erzeugt.

Die bestehende (symmetrische) Bauprobe bleibt funktional **unberührt** und weiter oben im Kapitel.

## Nicht-Ziele (erster Wurf, YAGNI)

- **Keine Weichensteine** (`wgelb`/`worange`) und keine verzweigten Bögen. Nur `gelb`, `orange`,
  `gruen`, `wuerfel`. Weichen werden nachgerüstet, wenn der Kern (Drag + Live-Achse) sitzt.
- Keine Änderung an der bestehenden Physik (Statik/Kollaps) — sie wird **wiederverwendet**.
- Kein Umschreiben der alten Bauprobe auf ein Instanz-Objekt.

## 1. Architektur-Entscheidung

`js/bauprobe.js` (2965 Z.) ist ein **harter Singleton**: eine IIFE mit einmaligen Modul-Variablen
(`canvas, ctx, state, model, cam, ghostHits, visible, dynAcc, audio, palItems, barButtons`),
`init()` bindet hardcoded an die DOM-IDs `bp-canvas`, `bp-task`, `bp-verdict`, `bp-tip`, `bp-reset`
und startet ein eigenes `requestAnimationFrame`-Loop + `IntersectionObserver`. Zwei Instanzen aus
einer Datei sind nicht ohne großen Refactor möglich.

**Gewählt: neue Datei `js/bauprobe2.js` (eigene IIFE), Physik geerbt.**

- **Geerbt (nicht dupliziert):** Steingeometrie (`STONES`, `stepStone`), `mirrorItem`,
  Kontakt-/Statik-Engine (`detectContacts`, `analyzeStatics`), Kollaps-Engine
  (`createCollapseWorld`, `stepWorld`). Diese Kernel sind reine Funktionen der Item-/Chain-Daten
  und kennen keinen DOM-State.
- **Zugriff:** Die Kernel sind **bereits** über `window.KajuBauprobe._internals` exportiert
  (`STONES, stepStone, computeHalf, detectContacts, analyzeStatics, evaluateArch, lpSolve,
  createCollapseWorld, stepWorld, bodiesOverlap, DYN` — Z. 2954-2961). `bauprobe.js` bleibt daher
  **100 % unberührt** — kein Eingriff nötig. **Ausnahme:** `mirrorItem` ist *nicht* exportiert; da
  `bauprobe2` ohnehin im normalisierten Frame (Achse → x=0) spiegelt, bekommt es eine **eigene
  kleine `mirror()`-Helper** (poly / slices+CCW / centroid / faces an x=0) — kein Duplizieren der
  Physik, nur ~15 Z. Geometrie. Statik + Kollaps werden geerbt.
- **Kernel-Reinheit (verifiziert 2026-07-05):** `stepStone`, `mirrorItem`, `detectContacts`,
  `analyzeStatics`, `createCollapseWorld`, `stepWorld` referenzieren **keine** mutable
  Modul-Variable (`state`/`cam`/`world`/…) — reine Funktionen ihrer Argumente. `bauprobe2`
  kollidiert also nicht mit der laufenden alten Instanz.
- **Neu (in `bauprobe2.js`):** eigener State, Drag-Interaktion, Achsen-Logik, Render-Loop,
  Modell-Zusammenbau für die verschiebbare Achse.
- **HTML/CSS:** ein zweites Kapitel-Fragment (neues Canvas + Controls) unter der bestehenden
  Bauprobe. Die alte HTML-Freeze-Regel („nur bauprobe.js") entfällt für diese Arbeit (von Kai
  freigegeben) — die *alte* Section bleibt aber unangetastet.

## 2. Koordinaten & Modell-Normalisierung (Kern der Physik-Anbindung)

Konvention wie im Kernel: Welt-cm, Boden `y=0`, `y` nach oben; die geerbte Engine (`mirrorItem`,
`analyzeStatics`) nimmt die Spiegelachse **fest bei `x=0`** an.

Die neue Variante hat eine **verschiebbare** Achse bei `x = axisX`. Lösung: **Normalisierung.**
Für Spiegelung, Statik und Kollaps wird die gebaute Hälfte in einen Frame verschoben, in dem die
Achse auf `x=0` liegt (`x' = x − axisX`). Dort laufen `mirrorItem` und `analyzeStatics`
**unverändert**. Die Statik ist in x translationsinvariant (Boden bleibt `y=0`), also ist der
verschobene Frame physikalisch identisch. Fürs Rendering wird zurück nach `+axisX` transformiert
(bzw. die Kamera im Achsen-Frame zentriert).

**`chain` für die Engine** = `[...leftItems, ...mirrorItems]` im normalisierten Frame
(genau die Struktur, die `analyzeStatics` und `createCollapseWorld` schon fressen). Ein separater
Schlussstein-Eintrag entsteht nur im Snap-Fall „Schlussstein-Mitte" (siehe §4).

## 3. Bauen (einseitig, Drag)

- **Vorrat:** vier Steine (`gelb 60° · orange 30° · grün 15° · würfel 90°`), im Canvas gezeichnet
  (analog `palItems`, ohne `wgelb`/`worange`).
- **Kette:** intern eine Liste `built = [{type, inverted}]`. Jeder Stein dockt **fugengenau** an
  die `endFace` des vorherigen an — dieselbe Ketten-Mechanik wie `stepStone`/`computeHalf`, nur
  einseitig. Der erste Stein sitzt am linken Kämpfer auf dem Boden (`y=0`); die Kette wächst nach
  oben/innen Richtung Achse.
- **Drag-Geste:** Pointer-down auf einem Vorrat-Stein → ein **Geist** folgt dem Finger →
  eine **Snap-Vorschau** zeigt den Stein an der nächsten gültigen Fuge (dem offenen Kettenende) →
  Pointer-up dockt an. Loslassen fern der gültigen Zone verwirft den Stein (kein Platzieren).
  *Es gibt nur eine offene Baufront (kein Verzweigen), also ist die „richtige Stelle" eindeutig
  das Kettenende.*
- **Overlap-Schutz:** Wie in v3 — ein Stein, der sich in einen bestehenden bohrte, wird
  zurückgerollt (`bodiesOverlap`-Analog auf der linken Kette).
- **Korrigieren:** Ausgewählten Stein (Tap) drehen (`inverted` toggeln) oder entfernen (nimmt den
  Rest ab dem Index mit). Kein Würfel-Sonderfall über das Nötige hinaus.

## 4. Spiegelachse & Spiegeln (Modell A — live gekoppelt)

- **Darstellung:** dünne vertikale Linie über die Bühne (leichter Strich).
- **Verschieben:** zwei Buttons `◀ ▶` bewegen `axisX` in kleinen Schritten. Die Achse ist
  **magnetisch** — sie zieht zu Snap-Kandidaten und rastet dort ein.
- **Snap-Kandidaten** (die x-Positionen, an denen die Spiegelung fugengenau schließt):
  1. **Vertikale Fuge:** die `endFace` des letzten Steins steht senkrecht (`pin.x ≈ pout.x`).
     Achse = dieses `x`. Die gespiegelte `startFace` deckt sich exakt mit der `endFace` →
     Bogen schließt ohne Schlussstein (gerade Steinzahl je Hälfte).
  2. **Schlussstein-Mitte:** die Winkelhalbierende des letzten Steins steht senkrecht. Die Achse
     geht durch seine Mitte; der Stein ist selbst der Schlussstein (rechte Hälfte = sein
     Spiegelbild). Nutzbar, wenn ein einzelner mittiger Stein die Lücke füllt.
- **Spiegeln-Knopf:** schaltet `mirrored` an/aus. Ist `mirrored` aktiv, wird bei jeder
  Achsen-Bewegung die rechte Hälfte **live** neu gespiegelt (Modell A). Der Nutzer schiebt die
  Achse, bis beide Hälften sich treffen.
- **Schluss-Erkennung:** `axisX` liegt (in Toleranz) auf einem Snap-Kandidaten **und** `mirrored`
  ist an → Bogen **geschlossen**, `Bogen testen` wird scharf. Toleranzen wie v3
  (`CONTACT_TOL_CM`, plus die verrundeten `OVERLAP_TOL/JOIN_TOL` für die Baubarkeit).

## 5. Testen (Physik unverändert)

- „Bogen testen" ist **jederzeit** möglich (Prinzip „always-testable" aus v3).
- Eine **ungespiegelte** oder offene Hälfte ist ein **Kragarm** → kippt meist (ehrliches
  Ergebnis über die geerbte Statik/Kollaps-Engine).
- Nach sauberem Schluss läuft `analyzeStatics(chain, contacts)` wie in v3: Urteil
  `stabil/kritisch/instabil`, Drucklinie, und bei `unstable` der Kollaps über
  `createCollapseWorld(chain)` → „Eingestürzt." / „Verkeilt.".

## 6. State (bauprobe2.js)

```
state = {
  built:    [{type, inverted}],   // linke Hälfte, Reihenfolge = Baureihenfolge
  axisX:    number,               // Welt-cm, verschiebbare Spiegelachse
  mirrored: boolean,              // Spiegeln aktiv?
  sel:      index|null,           // markierter Stein
  drag:     {type, pointer} | null,
  phase:    'build'|'solving'|'result'|'collapsing'|'settled',
  result:   StabilityResult|null,
  world:    CollapseWorld|null
}
```

## 7. HTML / CSS

- Neue Section (oder Sub-Block) unter `#k3` mit eigenen IDs: `bp2-canvas`, `bp2-task`,
  `bp2-verdict`, `bp2-tip`, plus die neuen Buttons `bp2-mirror`, `bp2-axis-left`, `bp2-axis-right`,
  `bp2-reset`. Die Controls (Spiegeln, `◀ ▶`, Reset) sind **DOM-Buttons** (mobil-freundlicher, klar
  antippbar); der Bau + die Vorrat-Palette bleiben im Canvas.
- Wiederverwendung der `.bauprobe__*`-Klassen (Canvas 820×700 intrinsisch, `width:100%`).
- Mobile-first: 390px zuerst prüfen.

## 8. Testing / Verifikation

- **Gate:** die bestehenden **66 Selbsttests bleiben grün** (Regression — `bauprobe.js` wird
  nicht angefasst).
- **Neue Selbsttests** (im `chk()`-Stil, eigener Block — in `bauprobe2.js` mit eigenem
  `selfTest2()`), u.a.:
  - Spiegelung an beliebiger `axisX` = Spiegelung an `x=0` nach Normalisierung (Invarianz).
  - Snap „vertikale Fuge": gerade Steinzahl schließt fugengenau (`joinGap ≈ 0`).
  - Snap „Schlussstein-Mitte": ungerade Konfiguration schließt über den mittigen Stein.
  - Ein geschlossener, gespiegelter Bogen liefert dasselbe Statik-Urteil wie derselbe Bogen in
    der v3-Bauprobe (Kernel-Parität).
  - Kragarm (ungespiegelt) ist testbar und instabil.
- **Live:** Variante im Browser mobil (390px) durchspielen: bauen → Achse schieben → spiegeln →
  schließen → testen (stabil-Fall + Einsturz-Fall).

## 9. Risiken / offene Detailpunkte

- **Snap-Präzision der „vertikalen Fuge":** hängt an der exakten `endFace`-Orientierung; Toleranz
  sauber wählen, sonst schnappt die Achse nie oder zu früh. In der Implementierung mit echten
  Koordinaten kalibrieren.
- **Kamera/Zoom:** die gebaute Hälfte + Spiegelbild müssen zusammen ins Bild passen (Auto-Zoom wie
  v3, aber Zentrum = `axisX`).
- **Ghost-Preview-Performance:** Snap-Vorschau nur bei aktivem Drag berechnen.
- **rAF-Loop gaten:** `bauprobe2` startet ein eigenes `requestAnimationFrame`-Loop → per
  `IntersectionObserver` auf Sichtbarkeit gaten (wie die alte Instanz), damit nicht zwei Loops
  dauerhaft laufen.
- **Schließbarkeit = Eigenschaft des Baus, nicht der Achse:** die `endFace` steht nur bei
  bestimmten Winkelsummen senkrecht (60/30/15°). Endet die Hälfte nicht schlussfähig, gibt es
  **keinen** Snap → ehrlich „so schließt der Bogen nicht" (Kragarm-Feedback), niemals eine
  geknickte/überlappende Fuge in `analyzeStatics` füttern.

## 10. Ablage

Code = Source of Truth: `/opt/kaju-prototypes/kaju-erlebnis-nacht/`. Outward-facing (GitHub Pages):
**kein Auto-Push** — commit lokal, Push erst nach Kais Freigabe. Vault-Note
`Obsidian/02 Projekte/shopify/kaju-erlebnis-prototyp.md` nach Umsetzung nachziehen.
