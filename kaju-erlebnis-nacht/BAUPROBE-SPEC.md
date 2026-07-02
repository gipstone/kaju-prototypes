# Bauprobe v2 — Spezifikation

## Ziel

Das interaktive Modul wird von einer abstrakten Ellipse mit Slider zu einem physisch korrekten
Bogenbau-Simulator umgebaut. Der Nutzer kombiniert echte Steintypen, sieht die fehlende
Winkellücke und kann die Drucklinie nachvollziehen.

---

## 1. Steingeometrie (Segmentform)

Jeder Stein ist ein kreisringförmiges Segment — kein verzerrtes Polygon.
Alle Maße in echten Zentimetern; zum Rendern: 1 cm = ~12px (anzupassen je Canvas-Größe).

| Typ    | Winkel | Dicke | Außenradius | Innenradius | Farbe   |
|--------|--------|-------|-------------|-------------|---------|
| Gelb   | 60°    | 2 cm  | 4 cm        | 2 cm        | #f1c953 |
| Orange | 30°    | 2 cm  | 8 cm        | 6 cm        | #f39200 |
| Grün   | 15°    | 2 cm  | 16 cm       | 14 cm       | #6cac53 |

Dicke = Außenradius − Innenradius = 2 cm bei allen Typen.

SVG/Canvas-Pfad pro Stein:
- Außenbogen von θ_start bis θ_end (Radius = Außenradius)
- Gerade Kante runter zum Innenradius
- Innenbogen zurück von θ_end bis θ_start (Radius = Innenradius)
- Schließen

---

## 2. Bogenbau durch Steinkombination

**Kein Slider mehr.** Der Bogen entsteht durch das Aneinanderfügen von Steinen.

- Steine stapeln sich symmetrisch von den Kämpfern (Fußpunkten) zur Mitte
- Jeder neue Stein dockt an den letzten gesetzten an (Stoßfuge = tangential)
- Der Bogen ist geschlossen, wenn die Lücke = 0°

### Fehlender Winkel (immer sichtbar)

Über dem Bogen wird der noch fehlende Winkel angezeigt:
```
Fehlender Winkel: 42°
```
Berechnung: 180° − Summe aller gesetzten Steinwinkel (halber Bogen × 2 = Vollbogen 180°
entspricht halbem Kreisumfang von Kämpfer zu Kämpfer).

Sobald der Bogen geschlossen ist: Anzeige „Bogen geschlossen ✓"

---

## 3. Nutzerinteraktion

### Vorrat (unterhalb des Bogens)

Drei Steine werden als Auswahlpalette angezeigt (Gelb | Orange | Grün).
Jeder Vorrat-Stein zeigt Typ-Icon + Winkelangabe.

### Stein im Bogen setzen

- Wenn kein Stein im Bogen markiert ist: Klick auf Vorrat-Stein setzt einen neuen Stein
  an der nächsten freien Position (alternierend links/rechts von Kämpfer zur Mitte)
- Wenn ein Stein im Bogen markiert ist: Klick auf Vorrat-Stein **tauscht** den markierten Stein aus

### Stein markieren

- Klick/Tap auf einen gesetzten Stein im Bogen → Stein wird mit hellem Rahmen umrandet (selected)
- Klick in leeren Bereich → Markierung aufheben
- Markierter Stein kann durch Klick auf Vorrat-Stein ausgetauscht werden

### Drehen (optional, wenn implementierbar)

Markierten Stein um 180° drehen (für spätere Phase — erst Basis implementieren).

---

## 4. Drucklinie (Thrust Line)

### Vereinfachte Annahmen

1. Der Bogenschub H wirkt am **Scheitel** (Krone) horizontal durch den **Extrados** (Außenkante).
2. Die Steine haben gleichmäßiges Eigengewicht: ρ = 1400 kg/m³ (Hartgips), Tiefe = 10 cm.
3. Das Gewicht eines Steins ergibt sich aus seinem Volumen × ρ × g.

### Bogenschub-Berechnung (Momentengleichgewicht am halben Bogen)

Am halben Bogen (eine Seite, Kämpfer bis Scheitel) gilt:

```
ΣM um den Kämpfer = 0
H × y_Scheitel = Σ (W_i × x_i)
→ H = Σ(W_i × x_i) / y_Scheitel
```

Wobei:
- H = horizontaler Bogenschub am Scheitel
- y_Scheitel = Höhe des Scheitels über dem Kämpfer
- W_i = Eigengewicht von Stein i
- x_i = horizontaler Abstand des Schwerpunkts von Stein i vom Kämpfer

### Drucklinie-Konstruktion (infinitesimale Methode)

Startpunkt: Extrados am Scheitel (oberste Außenkante), Richtung = horizontal (Bogenschub H).

Für jedes Bogensegment von Scheitel zu Kämpfer:
```
Resultierende = aktuelle Kraft + Eigengewicht des Segments (vertikal abwärts)
Neue Richtung = Richtung der Resultierenden
Neuer Punkt = alter Punkt + Schritt entlang der Resultierenden
```

Die Drucklinie wird als Kurve gezeichnet. Farb-Feedback:
- Grün: Linie bleibt innerhalb der Steindicke (stabil)
- Gelb: Linie berührt Intrados oder Extrados (Grenzfall)
- Rot: Linie tritt aus dem Steinquerschnitt aus (instabil)

---

## 5. Phasen (angepasst)

| Phase     | Beschreibung |
|-----------|-------------|
| `build`   | Nutzer setzt Steine aus Vorrat; fehlender Winkel wird angezeigt |
| `closed`  | Bogen geschlossen; Drucklinie erscheint; Stabilitäts-Feedback |
| `load`    | Murmeln auf Scheitel/Flanken (wie bisher) |
| `done`    | Abschluss-Text |

Der `shape`-Phase (Slider) entfällt vollständig.
Der `hold`-Button entfällt (kein Loslassen mehr nötig — Bogen steht wenn geschlossen).

---

## 6. Datei-Struktur (keine neuen Dateien nötig)

Nur `js/bauprobe.js` wird ersetzt. Canvas-ID `bp-canvas` bleibt.
HTML-Elemente `bp-task`, `bp-tip`, `bp-verdict`, `bp-reset` bleiben.
`bp-hold` und `bp-hold-label` werden ausgeblendet (hidden).

---

## Offene Fragen / Entscheidungen

- Scale: 1 cm = wie viele Canvas-Pixel? (Vorschlag: so skalieren dass ein vollständiger
  Rundbogen aus Gelb-Steinen ca. 60% der Canvas-Breite füllt → Außenradius 4 cm × 12 = 48px)
- Kämpfer-Winkel: Steine starten bei 180° (horizontal, Fußpunkt links/rechts) und bauen
  zur 90°-Mitte (Scheitel) — also ist 0° = rechts horizontal, 90° = oben, 180° = links.
- Symmetrie: Immer symmetrischer Bogen (gleichzeitig links und rechts gesetzt)?
  → Ja, vereinfacht.
