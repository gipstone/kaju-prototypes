# Bauprobe v2 — Spezifikation

## Ziel

Das interaktive Modul wird von einer abstrakten Ellipse mit Slider zu einem physisch korrekten
Bogenbau-Simulator umgebaut. Der Nutzer kombiniert echte Steintypen, sieht die fehlende
Winkellücke und kann die Drucklinie nachvollziehen.

---

## 1. Steingeometrie (Segmentform)

Jeder Stein ist ein kreisringförmiges Segment — kein verzerrtes Polygon.
Alle Maße in echten Zentimetern; Rendermaßstab so wählen, dass ein geschlossener
Rundbogen aus Gelb-Steinen (3 × 60° = 180° je Seite) ca. 60% der Canvas-Breite füllt.
Faustregel: Gelb-Außenradius 4 cm → ~60px → 1 cm ≈ 15px.

| Typ    | Winkel | Außenradius | Innenradius | Dicke | Farbe   |
|--------|--------|-------------|-------------|-------|---------|
| Gelb   | 60°    | 4 cm        | 2 cm        | 2 cm  | #f1c953 |
| Orange | 30°    | 8 cm        | 6 cm        | 2 cm  | #f39200 |
| Grün   | 15°    | 16 cm       | 14 cm       | 2 cm  | #6cac53 |
| Würfel | 90°    | —           | —           | —     | #a93015 |

**Würfel** = Quaderstein 2 cm × 2 cm (canvas: ~30px × 30px) für Spitzbögen:
Schließt einen 90°-Scheitelpunkt wenn beide Hälften mit je 45° Restwinkel aufeinandertreffen.
Er hat keine Kreiskrümmung — einfaches Rechteck mit den gleichen 2 cm Breite.

Canvas-Pfad pro Kreissegment-Stein:
  1. Außenbogen von θ_start nach θ_end (Außenradius)
  2. Gerade zur Innenkante
  3. Innenbogen zurück von θ_end nach θ_start (Innenradius)
  4. Schließen

---

## 2. Bogenbau durch Steinkombination

**Kein Slider. Keine Verzerrung.** Der Bogen entsteht ausschließlich durch Steinkombination.

### Symmetrie

Der Bogen wird **immer spiegelbildlich** aufgebaut: jeder gesetzte Stein erscheint
gleichzeitig als Spiegelbild auf der anderen Seite. Es gibt nur eine "linke Hälfte"
im Datenmodell; rechts = Spiegelbild davon.

### Fuge an Fuge

Jeder neue Stein dockt **exakt** an den letzten gesetzten an — keine Lücke, kein Überlapp.
Die Stoßfuge ist tangential: der neue Stein beginnt genau dort, wo der vorherige endet.

Für Kreissegment-Steine teilen sich benachbarte Steine denselben Mittelpunkt ihrer Kreise NICHT
notwendigerweise — jeder Stein hat seinen eigenen Mittelpunkt. Der Anschluss erfolgt über den
gemeinsamen Fugen-Punkt: Ende-Außen des vorherigen = Start-Außen des nächsten,
Ende-Innen des vorherigen = Start-Innen des nächsten.

Die Richtung am Fugenpunkt bestimmt, wie der nächste Stein liegt (tangential zur Außenkante).

### Stein drehen (invertieren)

Ein markierter Stein kann um 180° gedreht werden → seine Krümmung zeigt nach **außen**
statt nach innen. Das bedeutet:
- Sein Winkel wird von der Summe **abgezogen** (negativer Beitrag)
- Der Bogen wird an dieser Stelle konvex statt konkav
- Einsatz z.B. für S-Kurven oder experimentelle Formen

Visuell: gedrehter Stein wird mit einem Rotation-Icon angezeigt (kleines ↻-Symbol).

---

## 3. Fehlender Winkel

Über dem Bogen wird permanent der noch fehlende Winkel angezeigt:

```
∠ 42° fehlen
```

Berechnung:
- Rundbogen: 180° Ziel (halber Bogen, Kämpfer bis Scheitel)
- Fehlend = 180° − Σ (Winkel_i × Vorzeichen_i)
  - Vorzeichen = +1 (normal) oder −1 (gedreht/invertiert)

Sonderfall Schlussstein (s.u.): Wenn Fehlend ≥ 0° wird auf passende Steintypen geprüft.

---

## 4. Schlussstein-Mechanik

Wenn der fehlende Winkel exakt einem verfügbaren Steintyp entspricht (inkl. Würfel bei 90°):

1. Dieser Stein erscheint **schwebend über der Lücke** im Bogen, leicht transparent/pulsierend
2. Nutzer tippt darauf → Stein wird eingebaut, Bogen geschlossen
3. Anzeige wechselt zu „Bogen geschlossen ✓"

Passt kein einzelner Stein genau: normaler Betrieb (Nutzer wählt aus dem Vorrat).

---

## 5. Nutzerinteraktion

### Vorrat-Palette (unterhalb des Bogens)

Vier Steine in einer Reihe: Gelb (60°) | Orange (30°) | Grün (15°) | Würfel (90°)
Jeder zeigt seinen Winkel als Label.

### Stein setzen

- Kein Stein im Bogen markiert → Klick auf Vorrat setzt neuen Stein an nächster freier Position
- Stein im Bogen markiert → Klick auf Vorrat **tauscht** markierten Stein aus

### Stein markieren / modifizieren

- Tap/Klick auf gesetzten Stein → markiert (heller Rahmen)
- Tap in leeren Bereich → Markierung aufheben
- Markierter Stein: 
  - Klick auf anderen Vorrat-Stein = austauschen
  - Klick auf Drehen-Button (oder ↻ im UI) = invertieren (Krümmung nach außen)
  - Klick auf ✕ = Stein entfernen

---

## 6. Drucklinie (Thrust Line) — Phase 2

> **Für Phase 1 erstmal weglassen.** Erst wenn der Bogenbau-Modus stabil läuft.

### Vereinfachte Annahmen (für spätere Implementierung)

1. Bogenschub H am Scheitel wirkt horizontal durch den Extrados
2. Eigengewicht: ρ = 1400 kg/m³, Tiefe = 10 cm
3. Momentengleichgewicht am halben Bogen → H = Σ(W_i × x_i) / y_Scheitel

### Konstruktion

Startpunkt: Extrados Scheitel, Richtung horizontal.
Schritt: Resultierende = aktuelle Kraft + Steingewicht (vertikal).
Farbe: Grün (innerhalb) / Gelb (Grenzfall) / Rot (austretend).

---

## 7. Phasen

| Phase    | Beschreibung |
|----------|-------------|
| `build`  | Steine setzen, fehlender Winkel sichtbar, Schlussstein erscheint wenn passend |
| `closed` | Bogen komplett; Drucklinie (Phase 2); Lob-Text |
| `load`   | Murmeln (wie bisher, optional) |
| `done`   | Abschluss |

Kein `shape`-Phase, kein `hold`-Button, kein Squash-Slider.

---

## 8. Was bleibt / was fällt weg

**Bleibt:**
- Canvas `#bp-canvas` (W=820, H=700)
- `#bp-task`, `#bp-tip`, `#bp-verdict`, `#bp-reset`
- Bodenlinie, Bühnenbeleuchtung (radial gradient), Farbpalette
- Sound-Toks bei Steinplatzierung

**Fällt weg:**
- `#bp-hold`, `#bp-hold-label` → `hidden` setzen
- Squash/Geo-System (geo(), ept(), SEGS, ringD)
- Shape/Armed/Verdict-Anim-Phasen
- Squash-Drag-Handler

---

## 9. Kämpfer-Koordinaten

- Kämpfer links:  `(cx - archSpan/2, groundY)`
- Kämpfer rechts: `(cx + archSpan/2, groundY)`
- archSpan ≈ 2 × Außenradius des ersten Steins (definiert die Öffnung)
- Steine starten bei 180° (links horizontal) bzw. 0° (rechts horizontal) und bauen Richtung 90° (Scheitel)

---

## 10. Nur `js/bauprobe.js` ändern

Keine neuen Dateien. Keine Änderungen an HTML, CSS oder anderen JS-Modulen.
