# Fast Pushback: Geschwindigkeitsfaktor fuer automatische Routen

Stand: 2026-11-24. Verbindlicher Entwurf und Sourceimplementierung.

Nach dem Designreview wurden Sourceimplementierung, unabhaengiger Diffreview
und Trockentests ausdruecklich freigegeben. Implementierung und Trockentests
sind vorhanden; Ergebnisse und offene Gates stehen in
`FAST_PUSHBACK_VALIDATION.md`. Keine Plugin-Builds, Deployments, Commits,
Pushes oder Simulatoraktionen in diesem Implementierungsauftrag.

## 1. Entscheidung und Produktvertrag

Eine neue, eigenstaendige Option **Pushback speed factor** beschleunigt die
automatische Bewegung auf einer angenommenen Pushback-/Towroute. Sie ist
unabhaengig von **Fast Ground Handling**, Legacy-/Ground-Operations-Interface,
Route Recall und Auto-Disconnect.

Verbindliche erste Ausbaustufe:

- Auswahl `1.0`, `1.5`, `2.0`; Default `1.0`.
- Der Faktor multipliziert die bisherige Reisegeschwindigkeitsobergrenze,
  nicht die gesamte Simulationszeit oder die fertige Controllerausgabe.
- Vorwaerts- und Rueckwaertssegmente einer automatischen Route sind umfasst.
- Angenommene automatische Emergency-Tow-Routen verwenden denselben Pfad.
- Manuelles Pushen mit/ohne Yoke bleibt unveraendert. Auch Tuganfahrt,
  Aufnahme, Aufziehen, Absenken, Entkoppeln und Abfahrt bleiben unveraendert.
- Kurven-/Winkelgeschwindigkeitsgrenzen, Querbeschleunigung, Beschleunigung,
  Verzoegerung, Lenkraten, Kraftgrenzen und Reibungsgrenzen bleiben erhalten.
- `1.0` durchlaeuft den bestehenden Controllerpfad ohne neue Limits.
- Keine neue Freigabe, kein neuer Dialog, keine automatische Bremsbestaetigung.
- Keine Garantie fuer halbe Dauer bei `2.0`: Kurven, kurze Routen und
  begrenzte Tugs koennen einen kleineren oder keinen Zeitgewinn erlauben.

Eine gleichfoermige Zeitraffung inklusive Kurven mit vergroesserten
Beschleunigungen waere ein anderes Feature und ist nicht dieser Entwurf.
Der Wertebereich oberhalb `1.0` ist eine geplante Produktauswahl, noch keine
im Simulator validierte Stabilitaetszusage.

## 2. Referenz und belegte Eingriffskette

Referenzcheckout: `/Users/wahltho/Documents/Projects/BPB/BetterPusbackMod-main`.
Gepruefter `main`: `c202b4377869a1fe370ab1ac9760709f911621ce`.
Vor Dokumentationsbeginn nur `BPB_EMBEDDED_ZIBOMOD_CONCEPT.md` untracked;
keine getrackten Sourceaenderungen. Vor Implementierung erneut pruefen.

| Owner / Funktion | Heutiger Befund | Konsequenz |
|---|---|---|
| `bp.c:bp_state_init()` | Basis vorwaerts 4 m/s, rueckwaerts 1.11 m/s; Beschleunigung 0.25, Verzoegerung 0.17 m/s^2; Winkel 6/4 deg/s und Querbeschleunigung 0.1 m/s^2 | Diese Basis und alle Nicht-Geschwindigkeitsparameter unveraendert lassen |
| `bp.c:pb_step_tug_load()` | Begrenzt `bp.veh.max_*_spd` mit `max_tow_*_speed` des gewaehlten Tugs | Das so entstandene Legacyprofil bleibt erhalten |
| `bp.c:pb_step_pushing()` | Master ruft `bp_run_push()` auf; Slave folgt dem Flugzeug | Kein neuer Slavewriter und keine zweite Berechnung im Slave |
| `bp.c:bp_run_push()` | Manualabzweig, Bremsunterbrechung, Richtungswechsel, `drive_segs()`, Lenkung, Umrechnung auf Tuggeschwindigkeit | Nur automatische Reisegeschwindigkeitsplanung erweitern |
| `driving.c:next_seg_speed()` | Rekursiver Lookahead fuer gleichgerichtete Segmente; Crawl bei Ende/Richtungswechsel | Dieselben Limits durch die gesamte Lookaheadkette reichen |
| `driving.c:straight_run_speed()` | Kombiniert Reiseobergrenze mit verbleibendem Bremsweg und `max_decel` | Faktor genau hier als alternative Reiseobergrenze zufuehren |
| `driving.c:turn_run_speed()` | Verwendet dieselbe Bremsplanung und begrenzt Winkel-/Querbeschleunigung | Grenzen unveraendert nach der neuen Reiseobergrenze anwenden |
| `driving.c:drive_on_line()` | Pfadkorrektur, verlangsamte Ueberkorrektur und `ang_vel_speed_limit()` | Originales `vehicle_t` behalten, nicht global skalieren |
| `bp.c:push_at_speed()` | Reibungslimit, Krafterzeugung und Physikwrites; verarbeitet Tuggeschwindigkeit | Funktion unveraendert; kein zentraler Faktor an dieser Stelle |
| `bp.c:pb_step_stopping()` und Nachfolgeschritte | Zielausrichtung, Stillstand, bestehende Bremsuebergabe | Kein Faktor, keine neuen Abfragen/Writes oder geaenderten Schwellen |
| `tug.c:tug_drive()` | Eigene Fahrzeugparameter und `drive_segs()` fuer freie Tugbewegung | Bestehenden API-Aufruf ohne neue Limits behalten |
| `bp_cam.c:draw_prediction()` | Zeichnet geometrische Routensegmente | Keine neue zeitabhaengige Route, keine Cacheformatmigration |

Quellanker gelten fuer den genannten Snapshot; Funktionsnamen haben Vorrang
vor spaeter verschobenen Zeilennummern. Dies ist statische Sourceanalyse,
kein ausgefuehrter Controller-/Physiktest.

## 3. Geschwindigkeitsprofil und Einheiten

Fuer jede Fahrtrichtung getrennt:

```text
L = bestehende bp.veh.max_*_spd nach Tugauswahl             [m/s]
T = passende tug.info.max_tow_*_speed                      [m/s]
F = validierter, fuer diese Operation fixierter Faktor
C = min(F * L, T)                                         [m/s]
```

`C` ist nur die neue Reiseobergrenze. Das Originalprofil `bp.veh` wird nicht
veraendert. `straight_run_speed()` setzt `C` anstelle von `cruise_spd` ein,
behaelt aber seine heutige quadratische Bremswegberechnung. Die ausgegebene
Geschwindigkeit kann deshalb deutlich unter `C` liegen. `turn_run_speed()`
und `ang_vel_speed_limit()` begrenzen sie danach wie bisher.

Beispiel rueckwaerts, Tuggrenze mindestens 2.22 m/s:

| Faktor | Reiseobergrenze | Naeherungswert |
|---|---|---|
| 1.0 | 1.11 m/s | 4.0 km/h |
| 1.5 | 1.665 m/s | 6.0 km/h |
| 2.0 | 2.22 m/s | 8.0 km/h |

Ein enger Radius von 10 m ist bereits durch `sqrt(0.1 * 10)` auf
hoechstens 1 m/s begrenzt; der Faktor hebt das nicht auf. Bei 0.17 m/s^2
Verzoegerung vervierfacht Verdoppelung der Geschwindigkeit den idealisierten
Bremsweg bis Null: etwa 3.62 auf 14.50 m. Der vorhandene Lookahead muss
deshalb die erhoehte Obergrenze kennen; hinterher `speed *= F` ist verboten.
Die Zahlen sind algebraische Beispiele, keine gemessenen Simulatorwerte.

Die aktuellen Tugdefaults sind 6/3 m/s vorwaerts/rueckwaerts. Fehlende
`max_tow_*`-Werte fallen im Parser auf diese freien Fahrgeschwindigkeiten
zurueck. AP88 setzt explizit 1.7778 m/s fuer beide belasteten Richtungen.
Mit diesem Tug endet der Rueckwaertsvorteil deshalb bei etwa 1.60x, und
vorwaerts entsteht gegenueber dessen bisherigem Cap kein Zusatznutzen.
Tugkonfigurationen werden fuer das Feature nicht angehoben.

### 3.1 Umrechnung Flugzeug -> Tug

Heute wird die longitudinale Controllerausgabe in `bp_run_push()` umgerechnet:

```text
q = max(cos(nw_defl), 0.1)
v_tug = v_longitudinal / q
```

Nur bei aktivem Faktor groesser 1 wird anschliessend der Betrag von `v_tug`
mit dem richtungsabhaengigen belasteten Tugcap begrenzt. Sonst koennte die
Umrechnung den Cap trotz des Profils wieder ueberschreiten. Vorzeichen
bleibt erhalten, Null bleibt Null. **Dies ist kein nachtraegliches
Multiplizieren**, sondern ein zusaetzlicher Cap vor `push_at_speed()`.
Die Reibungscaps in dieser Funktion wirken weiterhin danach.

Das ist eine Begrenzung der Sollgeschwindigkeit, keine harte Garantie gegen
kurzzeitige Istwertueberschwinger des vorhandenen Kraftreglers. Dessen
Stabilitaet und Zielgenauigkeit muessen nach Test-/Simulator-Go belegt werden.

Bei `F == 1.0` bleibt auch diese Umrechnung ohne neuen Clamp. Ein eventuell
bereits im Legacybetrieb vorhandener Grenzfall wird nicht nebenbei repariert.
Kurven-/Winkelgrenzen beziehen sich weiterhin auf ihre bisherigen
Controllergroessen; keine unbewiesene neue Gleichsetzung mit Tuggeschwindigkeit.

## 4. Konkreter Sourceentwurf

### 4.1 Reiner Faktor-/Limithelper

Neue kleine Datei `src/pushback_speed.h` mit ausschliesslich Standard-C-
Typen/Mathematik, C/C++-kompatibel, ohne libacfutils oder Hostzugriffe.

Festgelegte Aufgaben:

- Normalisieren: nur exakt `1.0`, `1.5`, `2.0` akzeptieren; alles andere,
  einschliesslich NaN/Inf, auf `1.0` zuruecksetzen.
- Die zwei Reiseobergrenzen gemaess Abschnitt 3 bilden.
- Den vorzeichenrichtigen Tugcap nach Umrechnung anwenden.
- Ungueltige/nichtpositive Basis-/Tuglimits duerfen niemals einen erhoehten
  oder unendlichen Sollwert erzeugen. In diesem Fall keinen Speed-Override
  bereitstellen, den Legacypfad verwenden und einmal pro Operation warnen.

Keine eigene Zeitbasis, keine Registry, kein RNG, keine Speicherallokation,
kein globaler Zustand. Dieser Helper ist keine neue libacfutils-Abhaengigkeit.

### 4.2 Additive Driving-API statt veraendertem Fahrzeug

`src/driving.h` erhaelt `drive_speed_limits_t` mit zwei m/s-Feldern fuer
Vorwaerts-/Rueckwaertsreiseobergrenze und eine additive Funktion
`drive_segs_with_speed_limits(...)`. Sie besitzt dieselben bisherigen
Parameter und zusaetzlich einen `const drive_speed_limits_t *`.

`drive_segs(...)` behaelt Signatur und Semantik. Beide delegieren an einen
gemeinsamen internen Rumpf. Der alte Aufruf reicht `NULL` fuer Limits durch.
`NULL` verwendet unveraendert `veh->max_fwd_spd`/`veh->max_rev_spd`.

Der optionale Pointer wird unveraendert durch `next_seg_speed()`,
`straight_run_speed()` und `turn_run_speed()` gereicht. Ausschliesslich die
Auswahl von `cruise_spd` bekommt einen neuen Zweig. Segmentloeschung,
Richtungswechsel, Crawl, Ueberkorrektur und Lenkausgabe bleiben unveraendert.

Insbesondere kein skaliertes `vehicle_t`: Sonst wuerde auch die bisherige
Verlangsamung bei Ueberkorrektur in `drive_on_line()` vergroessert, obwohl
sie nicht zur gewuenschten Reisebeschleunigung gehoert.

### 4.3 Operationseigener Faktor und Live-Aufruf

`bp_state_t` in `bp.h` erhaelt einen skalaren
`automatic_push_speed_factor`. Nach `bp_state_init()` explizit `1.0`, nicht
der Nullwert aus `memset`. Keine Ressource und kein eigener Teardown.

Im erfolgreichen Startpfad von `bp_start()`, nach `bp_can_start()` und vor
Scheduling des Flightloops, wird einmal der validierte Settingswert kopiert.
Bereits laufender Start, fehlgeschlagener Start oder Plannerbewegungen
schreiben den Operationsfaktor nicht um. Ein Reconnect behaelt ihn. Nach
Complete/Abort liefert der bestehende Reset wieder `1.0`; die naechste
Operation uebernimmt den dann gespeicherten Wert neu.

`bp_run_push()` behaelt zuerst seinen Manualabzweig und alle bestehenden
Brems-, Pause- und Richtungswechselbedingungen. Nur am automatischen
`drive_segs()`-Aufruf:

1. Bei Faktor 1 oder nicht bereitstellbarem Profil den bisherigen Aufruf nutzen.
2. Sonst Limits auf dem Stack aus aktuellem Legacyprofil und gewaehltem Tug
   bilden; `drive_segs_with_speed_limits()` verwenden.
3. Bestehende Lenkung und longitudinale Umrechnung ausfuehren.
4. Nur im erhoehten Pfad den Tugcap aus Abschnitt 3.1 anwenden.
5. Bei `hold_requested` unveraendert Null mit `bp.veh.max_decel` anfordern;
   sonst denselben bisherigen `push_at_speed()`-Aufruf mit neuer Sollgeschwindigkeit.

Profilbildung erfolgt nicht kumulativ; niemals bereits skalierte Werte
erneut multiplizieren. Keine Configreads im per-Frame-Drivingcode. Der Slave
ruft diesen Masterpfad nicht auf; keine Synchronisation eines neuen
schreibbaren Datarefs und keine eigenen Physikwrites im Slave.

### 4.4 Bestehende Telemetriemetadaten korrekt publizieren

`telemetry_start()` in `bp.c` liest derzeit `effective_max_fwd_speed_mps`
und `effective_max_rev_speed_mps` aus dem unveraendert bleibenden Legacyprofil.
Das waere nach dem Feature fuer erhoehte automatische Fahrt falsch.

Die bestehende Producerzuweisung muss denselben validierten Profilhelper
wie der Drivingaufruf verwenden: automatische Masteroperation mit gueltigem
Override meldet `C`; Faktor 1, Fallback und Manual melden die bisherigen
Legacywerte. Die Felder bedeuten longitudinale Reiseobergrenzen vor
dynamischen Limits, nicht Tuggeschwindigkeit oder garantierte Istwertcaps.
Keine Schemanummer/-felder aendern. Die Slavewerte bleiben mangels
Faktorsynchronisation lokale, nicht autoritative Werte; das im Diagnosevertrag
explizit festhalten und nicht als Masterprofil auswerten.

Der Snapshot liegt vor `telemetry_start()` vor; Tugauswahl und bestehende
Legacycaps sind an dessen Aufrufstelle bereits bekannt. Die neue Metadaten-
Berechnung ist read-only, darf kein Profil in `bp.veh` zurueckschreiben und
keine Telemetrie in normalen Distributionen einschalten.

## 5. Settings, Persistenz und Texte

Globale Option, analog zur bestehenden unabhaengigen Fast-Option:

| Vertrag | Festlegung |
|---|---|
| Configkey | `pushback_speed_factor`, numerisch |
| Getter in `cfg.h/.cpp` | `double bp_pushback_speed_factor(void)` |
| Fehlender/unlesbarer/ungueltiger Key | Effektiv `1.0`; keine automatische Dateiumschreibung |
| UI-Label | `Pushback speed factor` |
| Auswahl | `Normal (1x)`, `1.5x`, `2x` |
| Widget-ID | `##pushback_speed_factor` |
| Speicherzeitpunkt | Gemeinsam mit staged Fast-/Recall-Werten bei Save preferences |
| Aktivierung | Neue Operation; Save muss bis zum vollstaendigen Operationsende gesperrt sein, siehe Voraussetzung 5.1 |
| Per-Aircraft-Migration | Keine; `per_aircraft_is_global` aendert diesen globalen Key nicht |

SettingsWindow liest den validierten Getter in seinen Auswahlzustand ein.
Schliessen ohne Save aendert diese Option nicht. Save schreibt ausschliesslich
einen der drei gueltigen Faktoren und verwendet den gemaess 5.1 abgesicherten Reload.
Keine Aenderung der Classic-Migration und kein heimliches Einschalten von Fast.

### 5.1 Vorgelagertes Sicherheitspaket: Save/Reload wirklich nur idle

Der unabhaengige Review hat die Annahme eines bereits sicheren Idle-Reloads
widerlegt. `stop_pb_handler()` aktiviert das Preferencesmenue vor Abschluss
der Operation. Ein neues SettingsWindow initialisiert `save_disabled` mit
false. `bp_conf_set_save_enabled(flag)` weist den Enabled-Parameter direkt
`save_disabled` zu, also invertiert zur Benennung und den heutigen Aufrufern.
`bp_do_reload()` deaktiviert den Core bei gesetztem Request ohne erneuten
Idlecheck. Ein Factorsnapshot verhindert diesen Core-Reset nicht.

Das ist ein bestehender Lifecyclefehler, kein Speedalgorithmusfehler.
**Vor der Featureintegration ist ein eigenes, eng begrenztes Sourcepaket
mit ausdruecklichem Source-Go erforderlich:**

- In `cfg.cpp` die vorhandene Enabled-API korrekt auf Disabled abbilden:
  `save_disabled = !flag`. Die heutigen Aufrufer geben bereits `!bp_started`
  weiter und brauchen deshalb keine Aenderung am Controller.
- Den neuen Fensterzustand aus dem aktuellen Busy-Zustand initialisieren.
  Beim Rendern und nochmals unmittelbar im Save-Zweig gilt zusaetzlich
  `!bp_started && !planner_open`. Nicht allein auf den gecachten
  `save_disabled`-Wert oder die Menuefreigabe vertrauen.
- Bei verweigertem Save keine staged Configwrites, Dateipersistenz oder
  Reloadanforderung ausfuehren. Bestehende sonstige Live-Settings werden
  durch dieses Paket nicht zu einem neuen Transaktionssystem umgebaut.
- In `xplane.c:bp_sched_reload()` Operation/Planner vor Requestannahme
  pruefen. Die bestehende void-API kann erhalten bleiben.
- In `bp_do_reload()` unmittelbar vor Coreinvalidierung/-disable erneut
  `!bp_started && !planner_open && !get_pref_widget_status()` pruefen.
  Preferences-Save schliesst das eigene Fenster und setzt dessen Aktivflag
  vor dem naechsten Flightloop zurueck. Nicht den eigenen sichtbaren
  Prefsstatus als pauschale Sperre fuer das Anfordern beim Save verwenden.
- Wird ein angenommener Request bis zur Ausfuehrung unzulaessig, ihn
  verwerfen und einmal melden; kein automatischer spaeterer Reload nach
  Abschluss einer inzwischen gestarteten Operation. Status, Tug und Core
  bleiben dabei unveraendert; der Nutzer kann idle erneut speichern/laden.

Getrennte Allowlist dieses Voraussetzungspakets: `src/cfg.cpp`,
`src/xplane.c`, neuer `tests/reload_safety_test.c`, neuer
`tests/run_reload_safety_tests.py` und Runnerregistrierung in
`tests/run_all_tests.sh`. Keine Aenderung von `bp.c` oder Bremsfunktionen.
Das Speedfeature selbst erweitert seine Allowlist dadurch nicht heimlich.

Pflichtfaelle: Fenster idle/waehrend STOPPING neu oeffnen; laufende Operation
und Late-Plan; Save nach Stop, aber vor Complete; Save nach Complete;
gueltiger Request mit Operation/Planner/neuem Prefsfenster vor dem naechsten
Callback; verworfener Request wird nicht spaeter ausgefuehrt. Alle verweigerten
Pfade muessen ohne Core-Reset, Invalidierung oder zusaetzliche Brakewrites
bleiben. Der idle erlaubte Save-/Reloadpfad funktioniert weiterhin.

Diese Lifecyclekorrektur aendert bewusst eine fehlerhafte UI-/Reloadsemantik
auch bei Faktor 1. Die geforderte Faktor-1-Paritaet des Fahrcontrollers bleibt
davon getrennt. Sie ist kein Anlass, diese Sicherheitskorrektur als versteckte
Brems- oder Geschwindigkeitsaenderung auszugeben.

### 5.2 Neue Texte

Tooltip, verbindliche Aussage:

> Raises the cruising-speed limit of automatic pushback and towing. Tug,
> turn and stopping limits still apply. Does not change manual towing or
> ground handling. 2x does not guarantee half the total time.

Label, neue Auswahltexte und Tooltip in `data/po/strings.pot` aufnehmen.
Bestehende PO-Uebersetzungen nicht erfinden oder als fertig markieren;
fehlende Uebersetzungen verwenden den englischen Fallback.
Den bisherigen Fast-Ground-Handling-Tooltip eindeutig abgrenzen: Diese
Option aendert selbst keine Fahrgeschwindigkeit, die neue Option tut es.
UI-Zeile und Savebutton muessen auf dem kleinsten unterstuetzten Fenster
erreichbar bleiben; noetigenfalls vorhandenes Layout minimal erweitern.

## 6. Unveraenderliche Vertraege und Featuregrenzen

Keine Aenderung an:

- `pbrake_is_set()`, `brakes_set()`, Bremsdatarefs oder SDK-Wrappern.
- `fast_brake_handoff.h`, Resetstellen, Pedalschwelle, VERIFIED/RESTORE,
  `required`, `fast_brakes_relinquished` und allen Cleanupguards.
- STOPPING/STOPPED/LOWERING/UNGRABBING und Parkbremsbestaetigungen.
- `push_at_speed()`, Krafterzeugung, Beschleunigung/Verzoegerung und Reibung.
- `turn_nosewheel()`, Fahrwerks-/Kontaktwrites und Lenkraten.
- Simzeit, `bp.d_t`, Flightloopphase/-frequenz und Pause/Replaybehandlung.
- `artificial_delay()`, `handling_fraction()` und Handlingzeiten.
- Richtungswechselwartezeit, STOP-/Pause-/Abortentscheidungen und Schwellen.
- Segmentgeometrie/-ende, Route Recall/Save, Slot-/Legacycacheformat.
- Kompassposition/-orientierung und geometrischer Plannerdarstellung.
- Bugradstatus ABI 1 mit 17 Elementen, Producergrenzen und Rateberechnung.
- Tugkonfigurationen, Assets, Stimmen, Audiozeitbasis und Zufallssequenz.

Der Controller darf bei Faktor groesser 1 andere Sollgeschwindigkeiten und
infolgedessen andere Bewegungs-/Kraftwerte erzeugen. Das ist der beabsichtigte
Featureeffekt, keine Behauptung unveraenderter Gesamtphysik. Die Algorithmen
und alle Bremsuebergabe-Writer bleiben unveraendert. Durch frueheren
Stillstand duerfen Ablaufereignisse frueher eintreten, niemals durch eine
ersetzte oder erzwungene Parkbremsfreigabe.

Pause und Pilotbremsung koennen wegen hoeherer Anfangsgeschwindigkeit mehr
Weg benoetigen. Das Feature darf deshalb nicht als garantierter Sofortstopp
beworben werden. Akzeptanz prueft realen Bremsweg mit den bisherigen Grenzen.

Bei einer spaeteren Uebernahme in den Zibomod-Port gelten dieselben Vertraege.
Der Port bleibt gemaess `BPB_EMBEDDED_ZIBOMOD_CONCEPT.md` vollstaendig ohne
libacfutils-Abhaengigkeit. Dieses Feature fuehrt weder dort noch im Helper
eine solche Abhaengigkeit neu ein und implementiert keine Portmigration.

### 6.1 Bestehende Rueckwaerts-Lenkdaempfung als ausdrueckliches Gate

`turn_nosewheel()` verwendet nach `ang_vel_speed_limit()` den Vergleich
`speed < bp_ls.tug->pos.spd`. Bei Rueckwaertsfahrt von -2 auf begrenzte
-1 m/s ist die Bedingung falsch; die zusaetzliche Lenkdaempfung ist damit
nicht vorzeichensymmetrisch. Die eigenstaendige Lenkratenbegrenzung bleibt
vorhanden. Das ist ein belegter Baselinebefund, keine neue Folge des Faktors.

Nicht im Feature reparieren und dadurch Defaultparitaet aufgeben.
Stattdessen erhoehte Rueckwaertsfahrt mit grossen Lenkkorrekturen und
Auslenkungen nach beiden Seiten ausdruecklich trocken und im Simulator
pruefen. Unveraenderter Lenkcode allein ist kein Stabilitaetsbeweis bei
hoeherem Tempo. Scheitert dieses Gate, den betroffenen Faktor nicht
freigeben; eine Lenkungskorrektur benoetigt einen eigenen Entwurf/Scope.

## 7. Planner und bestehender Baselinebefund

Die aktive `draw_prediction()` zeichnet `draw_segment()` fuer angenommene
und vorhergesagte geometrische Segmente. `compute_segs()`, Segmenthash und
Routecache bleiben unveraendert; diese Daten beschreiben keine Fahrdauer.
Faktorwechsel erfordern hier keinen neuen geometrischen Cachekey.

**Korrektur bei der Buildvorbereitung:** Der getrackte `bp_cam.c` enthaelt
zusaetzlich einen nicht vom aktiven Zeichnen aufgerufenen Previewblock
`planner_build_controller_path()` / `draw_controller_path()`. Darin werden
unter anderem `planner_prediction_state_t`, `PREDICTION_DT`,
`planner_path_count` und `planner_prediction_signature()` benutzt; ihre
Definitionen wurden im aktuellen Sourcebestand nicht gefunden. Der Block
behauptet ausserdem im Kommentar einen Bezug zu `automatic_steer_target()`,
das im aktuellen Livecontroller nicht vorhanden ist.

Der gesamte Block steht jedoch bereits in der eingefrorenen Referenz unter
`#if 0` (`bp_cam.c:684` bis `:1115`). Die fruehere Einstufung als
kompilationsrelevanter Closurebefund war falsch: Der Praeprozessor entfernt
diesen Block. Source und lokale Workspacekopie dieses Files
haben denselben SHA-256:
`6eb0f0a5bb9ea0f704cf816be90fab8b175c4ba7a6df6f71b504f525527fd963`.

Diesen Block fuer Fast Pushback weder aktivieren noch ausbauen, entfernen
oder durch ein neues Lenkmodell ersetzen. Es ist keine Plannerkorrektur
fuer den Produktbuild erforderlich. Ein erfolgreicher Neubuild wird erst
nach ausgefuehrtem Build belegt.
Falls spaeter ein echter dynamischer Preview aktiviert wird, braucht dessen
Signatur das effektive Speedprofil und die Tuggrenzen. Das ist nicht die
heutige aktive Anzeige und kein Grund fuer vorsorgliche Plannerneuschreibung.

## 8. Implementierungs-Allowlist

Nach gesondertem Source-Go geplanter Diff:

| Datei | Erlaubte Aenderung |
|---|---|
| `src/pushback_speed.h` neu | Reine Validierungs-/Limithelper, Standard-C |
| `src/driving.h` | Optionales Speedlimitpaar und additive API |
| `src/driving.c` | Gemeinsamer API-Rumpf, Pointer durch Lookahead, alternative Reiseobergrenze |
| `src/bp.h` | Ein skalarer Operationsfaktor |
| `src/bp.c` | Init/Snapshot, Auswahl am automatischen Driving-Aufruf, Tugcap vor Kraftregler, bestehende `effective_max_*`-Metadaten mit demselben Profilhelper |
| `src/cfg.h`, `src/cfg.cpp` | Getter, staged Combo, validiertes Speichern und Tooltipabgrenzung |
| `data/po/strings.pot` | Ausschliesslich neue/angepasste UI-Texte |
| `README.md`, `USER_GUIDE.md` | Optionsunabhaengigkeit, Limits und Zeitpunkt der Uebernahme |
| `tests/pushback_speed_test.c` neu | Echte Helper, keine nachprogrammierte Erwartungszustandsmaschine |
| `tests/run_pushback_speed_tests.py` neu | Echte Driving-/Config-/Controllerstellen mit SDK-Stubs/Referenzstand pruefen |
| `tests/run_all_tests.sh` | Neuen Runner aufnehmen |

Keine Aenderung von `bp_cam.c`, `vehicle_physics.c`, `tug.c`, Tugassets,
Buildchain, libacfutils, Zibomod-Source oder bestehenden Bremsreferenzen.
Falls eine Source-Allowlist nicht reicht, vorher konkret begruenden statt
waehrend der Implementierung unbemerkt zu erweitern. Die bestehenden
ungetrackten Dokumente und fremde Aenderungen bleiben erhalten.

## 9. Geplanter Test- und Reviewvertrag

Dieser Abschnitt ist der urspruengliche Testvertrag. Der spaeter autorisierte
Trockentestlauf ist in `FAST_PUSHBACK_VALIDATION.md` belegt. Simulatorfaelle
sind weiterhin offen. Fuer Referenzvergleiche den oben fixierten Sourcezustand
verwenden, nicht waehrenddessen einen beweglichen Upstreamstand.

### 9.1 Reine Helper und Driving

- Alle drei Faktoren, fehlende/unlesbare Config, Null, negative Werte,
  Zwischenwerte, NaN/Inf: gueltige Auswahl oder exakt Default 1.
- Getrennte Vorwaerts-/Rueckwaertscaps, AP88 und synthetischer niedriger
  Tugcap; keine Ueberschreitung durch wiederholte Aufrufe.
- Faktor 1/NULL-Limits gegen fixierten Original-Drivingcode: dieselben
  Sollgeschwindigkeiten, Lenkwerte, decel-Flags, Segmentmutationen und
  `last_mis_hdg`; einschliesslich finalem Crawl und Richtungswechsel.
- Langgerade erreicht bei genug Strecke das zulaessige erhoehte Cap;
  monotone Faktoren erzeugen keine niedrigere reine Reiseobergrenze.
- Kurzgerade, Gerade vor Kurve, Kurve vor Gerade, S-Kurve, enge/weiche
  Links-/Rechtsboegen, Folgekurven und wechselnde Fahrtrichtung.
- Bremsbeginn vor dem neuen Cap ausreichend frueh; Ende/Crawl und
  Richtungswechsel nicht multipliziert. Keine Annahme einer exakten
  Halbierung der Laufzeit fuer diese Faelle.
- Ueberkorrektur weiterhin mit Legacy-`veh.max_rev_spd` begrenzt;
  Winkel-/Querbeschleunigungsgrenzen fuer die heutigen Bezugsgroessen erhalten.
- Umrechnung bei 0, positiven/negativen grossen Tugwinkeln: Vorzeichen,
  Null und finales Tugcap korrekt; Faktor 1 verwendet Originalpfad.
- Erhoehte Rueckwaertsfahrt mit starken Lenkkorrekturen beidseits gegen
  den realen `turn_nosewheel()`-Pfad pruefen; Asymmetrie aus 6.1 nicht
  durch ein idealisiertes Testmodell oder geaenderte Erwartungen verdecken.

Dazu echte `drive_segs()`-/Lookaheadfunktionen kompilieren bzw. mit minimalen
Stubs extrahieren. Nicht nur eine vereinfachte Geschwindigkeitsformel testen.
Integration mit einfachem Bicycle-Modell ist Dry-Evidenz, kein Nachweis fuer
die echte X-Plane-Kraftregelung.

### 9.2 Controller, Features und Lifecycle

- Sourcevergleich aller geschuetzten Bremsfunktionen und bestehenden
  Bremscontrollerfixture, ohne neue Erwartungen an geaenderte Bremslogik.
- Automatik mit Faktor 1/1.5/2, Fast Handling aus/an und beiden Interfaces.
- Parkbremse/Pedale waehrend Bewegung, Pause/Resume, Stop, Abort, Lichtwarnung,
  Richtungswechselwartezeit und normaler Abschluss bleiben erreichbar.
- Realistic-Parkbremse mit gehaltenen Pedalen beim Setzen, spaeterem Release
  und erneutem Antippen nach VERIFIED: bestehender Fast-Readbackvertrag.
- Reconnect behaelt Operationsfaktor; Complete/Abort/Reload und naechster
  Start erzeugen weder stale Profil noch doppelte Skalierung.
- Slave und Manual verwenden unveraenderten Pfad; automatische Emergency
  Tow nur waehrend ihrer angenommenen Route beschleunigt.
- Save/Reload/Neustart und Fenster schliessen ohne Save; kein Einfluss von
  Classicmigration, Fastoption, Recall oder Per-Aircraft-Globaloption.
- Voraussetzung 5.1 vollstaendig pruefen, insbesondere Save nach Stop vor
  Complete und einen erst nach Scheduling busy gewordenen Reloadcallback.
- `effective_max_*`-Metadaten fuer 1/1.5/2, kleinen Tugcap, Fallback und
  Manual entsprechen dem tatsaechlich ausgewaehlten Profil; Slavewerte
  werden nicht als synchronisiertes Masterprofil interpretiert.
- Beide Tugbauarten, unveraenderte Bugradproducer und geometrische Route;
  bekannte Rate-/Statusvalidierung nicht an neue Erwartungen anpassen.

Nach fokussierten Tests die gesamte bestehende `tests/run_all_tests.sh`
ausfuehren. Deren bestehende Coverage ist nicht automatisch ein Test der
neuen Driving-Erweiterung. Distributionstelemetrie bleibt standardmaessig aus.
Ein Startlog darf Faktor und berechnete Reiseobergrenzen einmal nennen;
keine neue per-Frame-Aufzeichnung und keine Telemetrieschemaaenderung noetig.
Die bestehenden Metadatenproducer werden gemaess 4.4 trotzdem korrigiert.

### 9.3 Simulatorabnahme nach separatem Build-/Deploy-Go

Referenzroute und Flugzeug unveraendert vergleichen: lange Gerade,
90-Grad-Kurve links/rechts, S-Kurve, kurze Zielgerade und Richtungswechsel.
Faktoren 1/1.5/2, schwere/leichte Aircraft, GRAB/WINCH, normale/schwache
Reibung und niedrige Framerate pruefen. Konkrete Bewertungsgrenzen:

- Rueckwaertige grosse Lenkkorrekturen und beidseitige Ausgangsauslenkung
  sind wegen 6.1 Pflichtfaelle, nicht bloss ein symmetrischer Bicycle-Test.
- Faktor 1 keine Regression gegen denselben Baselinestand.
- Bei Faktoren groesser 1 kein neuer Segment-/Zielueberschuss ueber 0.5 m
  oder zusaetzlicher Endheadingfehler ueber 1 Grad gegen die Vergleichsfahrt.
- Kein zusaetzlicher maximaler Cross-Track-Fehler ueber 0.5 m gegen die
  Vergleichsfahrt und kein Verlassen der vorhandenen Wingspan-Flaeche.
- Diese Differenzgrenzen sind geplante Akzeptanzgrenzen, keine nachgewiesene
  Baselinegenauigkeit. Eine bereits unsichere Vergleichsfahrt ist kein Pass.
- Pause/Stop bis Stillstand ohne neu auftretende Oszillation, ruckartigen
  Kontaktverlust oder Fortsetzung bei gesetzter Bremse; groesserer Bremsweg
  entsprechend Ausgangsgeschwindigkeit dokumentiert.
- Parkbremse haelt beim Abschluss auch mit Fast Handling und realistic Brake;
  kein zweites Setzen durch einen neu eingefuehrten Nullwrite.
- Zeitgewinn auf der langen, nicht vom Tug begrenzten Gerade nachweisen;
  kein Mindestzeitgewinn bei physikalisch begrenzten Kurven erzwingen.

Scheitert ein Faktor, nicht Kurven-/Bremsgrenzen aufweichen oder Tests
anpassen. Zuerst Eingriff pruefen; falls der bestehende Regler im erlaubten
Bereich nicht stabil ist, Produkt-Go fuer diesen Faktor zurueckhalten und
den Befund vor einer Erweiterung von Kraeften/Lenkung vorlegen.

## 10. Statischer Gegencheck und Freigabegates

Im Entwurf gepruefte Einwaende:

| Einwand | Entscheidung |
|---|---|
| Einfach fertige Geschwindigkeit multiplizieren? | Nein, verletzt den vorhandenen Bremsweg-/Kurvenlookahead |
| Ganzes Fahrzeugprofil oder `d_t` skalieren? | Nein, veraendert Korrekturgates, Zeit-/Physikvertraege und weitere Phasen |
| Beschleunigung/Verzoegerung ebenfalls vergroessern? | Nein, nicht fuer diese erste Ausbaustufe |
| Beladene Tuggrenze nach Kosinusumrechnung umgangen? | Neuer Cap nur im erhoehten automatischen Pfad |
| Freie Tugfahrt durch gemeinsame Drivingfunktion schneller? | Alte API bleibt NULL-Limits; neuer Aufruf nur in Auto-Push |
| Bremsuebergabe als Nebenprodukt anfassen? | Explizit ausgeschlossen und source-/tracebasiert zu pruefen |
| Preview aktivieren, um eine Faktorzeit zu zeigen? | Nein, bestehende Geometrie belassen; separaten Baselinebefund nicht verstecken |
| libacfutils im kuenftigen Port doch wieder erforderlich? | Nein, neue Policy ist Standard-C; keine Portmigration in diesem Scope |
| Save nach Stop kann den Core waehrend Entkopplung reloaden? | Bestehender Fehler; eigenes vorgelagertes Sicherheitspaket 5.1 |
| `effective_max_*` wuerde weiterhin Legacytempo melden? | Producer korrigieren, Schema beibehalten, siehe 4.4 |
| Rueckwaertige Tug-Lenkdaempfung wirklich symmetrisch? | Nein; Baselinebefund 6.1 als ausdrueckliches Freigabegate, keine versteckte Lenkungsreparatur |

Der statische Gegencheck wurde vom ausarbeitenden Agenten vorgenommen.
Der anschliessend ausdruecklich beauftragte unabhaengige Reviewer hat den
Erstentwurf gegen denselben Sourcezustand read-only geprueft. Reviewbasis:
SHA-256 `08077eaf84b485ad1d0073f131270f49098a298b7fa4516a6c0381917ea03cc9`.
Seine Findings: P1 Save-/Reloadsperre, P2 Metadatenproducer, P2 bestehende
Rueckwaerts-Lenkasymmetrie. Die Entwurfskorrekturen stehen in 5.1, 4.4 und
6.1. Die unabhaengige Nachpruefung vom 2026-11-24 bestaetigt alle drei
Findings auf Konzeptebene als geschlossen, ohne neue materielle Widersprueche.
Nachpruefungsbasis vor dieser reinen Statusaktualisierung:
SHA-256 `f4206a79fbf709e71ed47c1c6061d34023e13ebfc3c20d8bd9466690a14c14a5`.
Reviewerurteil: bereit fuer die gesonderte Source-Go-Entscheidung zum
Voraussetzungspaket 5.1 und anschliessend zur Featureimplementierung.
Zum Zeitpunkt des Designreviews gab es noch keine Sourceimplementierung; die
bestehende Lenkasymmetrie ist nicht behoben, sondern als Abnahmegate erfasst.
Vor Produktfreigabe ist zusaetzlich Review des konkreten Gesamtdiffs samt
Testergebnissen erforderlich.

Nachfolgende Gates (Status nach dem spaeteren Implementierungs-Go):

1. Abgeschlossenes Designreview; gesondertes Source-Go fuer das
   Voraussetzungspaket 5.1 und danach die Feature-Allowlist: erfolgt;
   Implementierung auf `main`, Baseline `c202b4377869a1fe370ab1ac9760709f911621ce`.
2. Test-Go und fokussierte, Lifecycle- und bestehende Regressionen: erfolgt;
   Toolchainhinweise und Grenzen siehe `FAST_PUSHBACK_VALIDATION.md`.
3. Unabhaengiger Diffreview: Ergebnis siehe `FAST_PUSHBACK_VALIDATION.md`.
4. Baseline-/Buildverdacht aus Abschnitt 7: geklaert, deaktivierter
   Referenzblock; keine Planner- oder Lenkungsaenderung erforderlich.
5. Build-/Deploy-Go; danach erst Simulatorabnahme und separates Commit-/Push-Go.

Der Featureentwurf legt API, Scope und Semantik fuer eine Implementierung
fest. Keine Aussage dieses Dokuments erklaert die noch offenen Test-,
Review-, Baseline-Build- oder Simulatorgates bereits fuer bestanden.
