# Fast Pushback: Source- und Trockentestnachweis

## Endausrichtung bei 2x, 2026-11-26

Das vorherige Endfenster galt nur auf einem geraden Schlusssegment und
bezog sich auf den Flugzeugursprung. Ein frueherer Null-Lenkbefehl allein
verbesserte nicht alle KSEA-Laeufe. Im neuen 2x-Fenster geht die Lenkung
am festen Fahrwerkspunkt kontinuierlich vom Routensoll zum Endkurssoll
ueber. Es ist auf eine halbe Radbasis, hoechstens das halbe Schlusssegment
und im Bogen zusaetzlich auf dessen letzte 15 Grad begrenzt. Keine
Aenderung an Geschwindigkeits-, Brems-, Abschluss- oder Testschwellen.

Alle 45 Routenkombinationen bestanden mit dem regulaeren STARTING-Frame,
Stillstand, Zeitgewinn und den zuvor festgelegten Endlagenkriterien. Bei
30 FPS (2x, Sekunden bis zum Stillstand / Endabstand im ebenen Testmodell):
KSEA N16 39,20 s / 2,555 m; BGTL 43,93 s / 2,535 m; KBNA 55,17 s /
2,051 m. Gegen den vorherigen Laengsregler ohne 2x-Endfenster waren die
1x-Traces (70.508 Frames) und 1,5x-Traces (49.426 Frames) bytegleich.
Das ist ein Trockentest, keine Abnahme in X-Plane.

## Kurvenfahrt mit Laengsregelung, 2026-11-25

Vergleich mit `c130669`: Bei KSEA N16 (30 FPS, 2x) stoppte der alte
Regler nach 5,10 s. In der Kurve forderte er weiter Zug an, obwohl das
Flugzeug die Abschaltschwelle fuer die Laengsgeschwindigkeit erreichte.
Der korrigierte schnelle Regler verwendet denselben Geschwindigkeitsbezug
wie die unveraenderte Bremshuelle. Die MED/POOR-Grenzen bleiben in
Schlepper-m/s definiert; ihre Projektion ist bei 60 Grad in beiden
Richtungen separat getestet.

Planarer Trockentest mit den gespeicherten Routen und AST-3F, Zeit von
Bewegungsbeginn bis zum Stillstand bei 30 FPS (Sekunden):

| Start | 1x | 1,5x | 2x |
| --- | ---: | ---: | ---: |
| KSEA N16 | 76,77 | 52,80 | 40,67 |
| BGTL Military Ramp 1 | 86,77 | 60,67 | 46,57 |
| KBNA C4 | 105,77 | 72,60 | 55,40 |

Alle 45 Kombinationen aus Route, Faktor und 20/30/60/120/variablen FPS
erreichten im Diagnoselauf Routenende und Stillstand ohne Fehlerhalt. Im
direkten Vergleich mit `c130669` waren die Controller-Traces aller 15
1x-Laeufe bytegleich (70.508 Frames).

**Damals gesperrter Zwischenstand.** Die erste Regression auf macOS und Linux bestand,
deckte bei Bogenenden aber nicht den regulaeren STARTING-Frame fuer die
spaetere Endausrichtung ab. Der erweiterte Test prueft auch Endpose und
Zeitgewinn. Alle 15 Fast-Laeufe mit 1,5x bestehen. Bei 2x ueberschreiten
KSEA (20/30/variable FPS) und KBNA (20 FPS) die Grenze fuer den
Positionsfehler (hoechstens 0,5 m mehr als bei 1x). Groesste Differenz:
1,058 m, KSEA variable FPS. Die Schwelle wurde nicht erhoeht.

KSEA, 30 FPS: bei Freigabe des letzten Segments 0,498 m / 0,496 m
Endabstand (1x/2x), beim Stillstand nach Endausrichtung 2,861 m / 3,492 m.
Der Fehler entsteht beim zusaetzlichen Fahren zum Geradestellen, nicht
beim beobachteten Ueberschreiten der Geschwindigkeitsgrenze im ersten Bogen.
Eine isolierte Probe mit normalem Kriechtempo allein genuegte nicht.
Keine Aenderung am Ausrichtungsablauf und keine Simulatorabnahme.

## Lokale Bewegungsskalierung, 2026-11-25

Referenz: `72a329537104c93b432859cc1600136f653aa187`. Anders als beim
historischen Reisecap unten skaliert `Pushback speed` nur die automatische
Bewegung; X-Planes Simulationszeit wird nicht geaendert. 1x ist die Referenz.
Die vorhandene Parkbremsuebergabe und Fast Ground Handling sind unabhaengig.

Beispiel aus dem ebenen Trockentest (300 m gerade rueckwaerts, 20 FPS):

| Auswahl | Fahrzeit bis zum Stillstand im Test | Endposition |
| --- | ---: | ---: |
| 1x (Referenz) | 277,95 s | 301,214 m |
| 1,5x | 185,30 s | 300,031 m |
| 2x | 139,95 s | 300,049 m |

Die Abweichung von 1,214 m bei 1x tritt auch im eingefrorenen Referenzstand
auf. Sie ist kein Sollwert fuer 1,5x oder 2x. Die Messwerte gelten fuer das
Testmodell, nicht als Zusage fuer jede Strecke, jeden Tug oder X-Plane.
Insbesondere ersetzen Regler- und Kurvenstichproben keinen durchgehenden
Kurvenlauf im Simulator. Kein Plugin-Build oder Deploy fuer diesen Nachweis.

Bei 300 m vor- und rueckwaerts ueber 20, 30, 60, 120 und variable FPS:
1,5x benoetigte 66,7-68,2 % der Referenzfahrzeit, 2x 50,4-51,7 %.
Die Kurvenstichproben vergleichen Sollgeschwindigkeit und Lenkung bei links
und rechts laufenden Boegen in beiden Fahrtrichtungen.

1x-Vergleich: 5040 Driving-Faelle sowie bytegleiche Controller-Traces bis
zur ersten im Original undefinierten Entscheidung. Im letzten Kriechabschnitt
kann der Planer zurueckkehren, ohne sein Verzoegerungsflag zu schreiben.
Der automatische Aufrufer initialisiert es jetzt mit `B_FALSE`; explizite
Planerwerte bleiben massgeblich. Fuer den **vollstaendigen** Trace dient
eine zweite Kopie von 72a3295 mit exakt diesem einen initialisierten lokalen
Wert als Referenz. Fuer undefiniertes Altverhalten wird keine Paritaet behauptet.

## Historischer Nachweis: Reisecap

Stand: 2026-11-24. Referenz auf `main`:
`c202b4377869a1fe370ab1ac9760709f911621ce`.

Freigegeben: Implementierung, unabhaengiger Review und Trockentests.
Nicht ausgefuehrt: Plugin-Build, Packaging, Deploy, Commit, Push,
Simulatorstart oder Simulatorsteuerung. Vorhandene Pakete bleiben unveraendert.

## Implementierter Umfang

- Eigenes globales Auswahlfeld `Pushback speed factor` unter Fast Ground
  Handling: Normal (1x), 1.5x, 2x; Default 1, Persistenz erst bei Save.
- Operations-Snapshot in `bp_start()` nach erfolgreicher Startpruefung.
  Wiederholter Start veraendert keinen laufenden Faktor; Reconnect erhaelt ihn.
- Separates, unveraenderliches Limitpaar wird durch den gesamten rekursiven
  Driving-Lookahead gereicht. Legacy-`vehicle_t` und bisherige API bleiben
  erhalten. Normale API und Faktor 1 verwenden NULL-Limits.
- Nur automatische Routen erhalten hoehere Reisecaps. Nach der vorhandenen
  Kosinusumrechnung gilt im erhoehten Pfad zusaetzlich die beladene Tuggrenze.
  Manual und freie Tugfahrt bleiben beim bisherigen Aufruf.
- Bestehende `effective_max_*`-Metadaten beschreiben das ausgewaehlte Profil
  zum Publikationszeitpunkt, ohne Telemetrieschemaaenderung. Slavewerte
  bleiben lokal/nicht autoritativ. Bei einem Profilwechsel wird vor dem
  naechsten Diagnosesample ein neues CSV mit korrekten Metadaten begonnen;
  bereits erfasste Framewerte bleiben erhalten. Runtime-Telemetrie bleibt
  in Distributionen deaktiviert. Keine Aenderung am Bewegungscontroller.
- Vorgelagert korrigierte Save-/Reloadsperre: live busy-Pruefung vor Save,
  korrekte Enable-Semantik, erneute Pruefung im verzoegerten Reloadcallback.
  Ein inzwischen unsicherer Reload wird verworfen, nicht spaeter ausgefuehrt.
  Plannerstatus wird live geprueft, nicht dauerhaft im Fenster gespeichert.

## Trockentests

```sh
python3 tests/run_reload_safety_tests.py
python3 tests/run_pushback_speed_tests.py
env CCC_OVERRIDE_OPTIONS='#+-Wno-error=nontrivial-memcall +-Wno-error=uninitialized-const-pointer +-Wno-error=macro-redefined' sh tests/run_all_tests.sh
git diff --check
env CCC_OVERRIDE_OPTIONS='#+-fsanitize=address,undefined +-fno-sanitize-recover=all' python3 tests/run_pushback_speed_tests.py
```

Die fokussierten Tests bestehen mit `-Wall -Wextra -Werror`; die Drivingtests
aktivieren auch libacfutils-Assertions (`DEBUG`). Testbinaries und extrahierte
Quellen werden nur in temporaeren Verzeichnissen erzeugt, kein Plugin gebaut.

- 5.040 Faelle vergleichen aktuellen echten Drivingcode und eingefrorenen
  Originalcode bitweise: Steuerung, Geschwindigkeit, Decel-Flags,
  `last_mis_hdg`, Segmentanzahl und verbliebener Segmentinhalt. Abgedeckt sind
  Vor-/Rueckwaerts, Gerade/Kurve, beide Kurvenseiten, kurze/mehrteilige Folgen,
  Richtungswechsel, Position/Heading-Abweichungen und Rear-Axle-Konfiguration.
  Auch die additive API mit NULL-Limits wird gegen dieselbe Referenz verglichen.
- Echte rekursive Lookaheadfunktionen: erhoehte Langgeradencaps, Bremsweg,
  End-/Richtungswechsel-Crawl, Winkel-/Querbeschleunigungsgrenzen,
  unveraendertes Legacy-Overcorrection-Cap. AP88- und kleinere synthetische
  Tuggrenzen sowie ungueltige Faktoren und Profile sind enthalten.
- Tatsaechliche `bp_run_push()`-Aufrufkette mit echten Drivingfunktionen:
  alle Faktoren, positive/negative Tugwinkel, finales Tugcap, Pedal-/Parkbrems-
  Unterbrechung, Pause/Hold, Manual-Bypass, Richtungswechselwartezeit,
  Endlenkung, Segmentabschluss und dauerhafter Legacy-Fallback.
- Echte Getter-/Startfunktionen: Default und unlesbare/ungueltige Werte,
  vollstaendiges Stringparsing statt akzeptierter Praefixe wie `2x`,
  abgelehnter Start ohne Snapshot, laufender Start ohne Neusnapshot,
  neuer Start mit aktuellem Wert. Metadatenzuweisungen werden direkt aus dem
  echten Producer kompiliert, fuer Normal, Faktor, Tugcap, Manual und Slave.
- Echter Diagnosepublisher und echter CSV-Writer: Connect-first/Automatik ->
  Manual angefordert/aktiv -> Automatik nach Reconnect; ausgegebene
  Metadaten werden aus den Dateien gelesen. Kein zweiter Wechsel bei
  gleichem Profil, kein Frame-Reset; deaktivierte/geschlossene Diagnose
  erzeugt keinen Refresh. Der Moduswechsel ist im Fixture modelliert,
  kein ausgefuehrter Simulator-Reconnect.
- Reale Richtungswechselkonstante aus dem Source extrahiert und gegen
  die Referenz geprueft; die bestehende Wartezeit von vier Sekunden wird
  unmittelbar unterhalb, genau an und oberhalb der Grenze geprueft.
- Echte Save-/Reloadfunktionen mit SDK-Stubs: busy-Kombinationen,
  modellierter Busy-Zustand nach Stop bis Complete, spaeter busy gewordener Reload,
  Planner geoeffnet/geschlossen und Save-Auswahlwerte. Auswahl ohne Save
  erzeugt keinen Configwrite; Fensterinitialisierung/Rendering werden
  zusaetzlich statisch geprueft, nicht als vollstaendiger GUI-Test ausgegeben.
- Echte `turn_nosewheel()`-Funktion dokumentiert die bestehende asymmetrische
  Rueckwaertsdaempfung beidseits. Der Test repariert oder verdeckt sie nicht.
- Alle geschuetzten bestehenden Kernfunktionen in `bp.c` werden gegen die
  Referenz sourceidentisch geprueft; ebenso Bremshelper, Tugcode, Planner und
  das vollstaendige `vehicle_t`. Ausnahmen sind ausschliesslich die
  dokumentierten Integrationsstellen (Start, Initialisierung, Metadaten,
  automatischer Fahr-Aufruf).

Der fokussierte Geschwindigkeitstest besteht zusaetzlich mit AddressSanitizer
und UndefinedBehaviorSanitizer ohne Befund.

Die vollstaendige bestehende Regression besteht ebenfalls, einschliesslich
passivem Fast-Bremscontroller, GRAB/WINCH-Bugradvertrag und Mutation-Site-Paritaet,
Legacy Recall/Save, Preference-Migration, Interface-/Automationsgates,
Per-Aircraft-Completion, Emergency Tow und deaktivierter Runtime-Telemetrie.
Die Unabhaengigkeit des neuen Faktors von Classic/Fast ist strukturell geprueft;
die bestehende Featurematrix ersetzt keinen realen Flug mit erhoehtem Faktor.

Der unveraenderte Default-Gesamtrunner scheitert mit dem installierten neuen
Apple-Compiler an Warnungen in unveraendertem ImGui/libacfutils-Code
(`nontrivial-memcall`, `uninitialized-const-pointer`, `strlcpy`-Makro).
Nur fuer diesen Testlauf wurden genau diese Warnungen von Error auf Warning
herabgestuft. Keine Vendorquellen, Testassertionen oder dauerhaften
Compilerflags wurden dafuer geaendert. Gesamtlaufprotokoll:
`/tmp/bpb-fast-pushback-tests-20261124.log` (temporaere lokale Evidenz).

## Unabhaengiger Review

Der getrennte Reviewer hat den konkreten Gesamtdiff und die Tests read-only
geprueft. Abschlussurteil vom 2026-11-24: keine verbleibenden materiellen
Findings; freigegeben innerhalb des Source-/Trockentestumfangs.

Vier P2-Findings wurden vor der Freigabe behoben und nachgeprueft:

1. Vollstaendiges Config-Stringparsing statt akzeptierter numerischer Praefixe.
2. Korrekte ausgegebene Diagnosemetadaten auch beim Wechsel Automatik/Manual.
3. Extrahierte Produktionskonstante und exakte Vier-Sekunden-Grenztests.
4. Begrenzte Multiplikation mit anschliessendem Tugcap, inklusive Subnormaltests.

Die Save-/Reloadsperre wurde ebenfalls bestaetigt. Der Reviewer fand keine
unerwarteten Aenderungen an geschuetzter Brems-, Physik-, Lenkungs- oder
Bugradlogik. Er hat die Testimplementierung und das erfolgreiche
Gesamtlaufprotokoll geprueft, nicht selbst Tests ausgefuehrt. Die fokussierten
ASan-/UBSan-Laeufe stammen vom implementierenden Agenten. Reviewfreigabe ist
keine Build-, Deploy- oder Simulatorfreigabe.

## Verbleibende Gates

Kein Nachweis echter X-Plane-Physik, Zielgenauigkeit oder Hardware durch
SDK-Stubs/Driving-Trockentests. Erhoehte Rueckwaertsfahrt mit starken
Lenkkorrekturen ist wegen der bestaetigten Baselineasymmetrie explizit vor
Produktfreigabe im Simulator zu pruefen. Keine Lenkungsreparatur in diesem Diff.

Korrektur bei der anschliessenden autorisierten Buildvorbereitung: Der
unvollstaendige dynamische Planner-Previewblock in `bp_cam.c` ist bereits
im eingefrorenen Referenzstand vollstaendig durch `#if 0` deaktiviert.
Die zuvor behauptete Compileproblematik dieses Blocks war eine falsche
statische Einstufung. Keine Planner-Sourceaenderung ist erforderlich.
Im hier beschriebenen Source-/Trockentestauftrag wurde kein Plugin gebaut;
nachfolgende Build-/Deploynachweise werden gesondert protokolliert.

Build/Deploy, Simulatorabnahme und Commit/Push benoetigen getrenntes Go.
