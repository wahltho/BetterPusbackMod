# Fast Pushback: Source- und Trockentestnachweis

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
