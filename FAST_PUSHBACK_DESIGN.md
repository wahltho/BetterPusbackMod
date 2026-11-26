# Fast Pushback: lokale Zeitraffung, erste Stufe bis 2x

Stand 2026-11-25, Sourcebasis main@72a329537104c93b432859cc1600136f653aa187.
Technischer Entwurf, nicht der Stand des installierten Plugins. Das bisherige
Feature begrenzt lediglich das Reisetempo am Tug. FAST_PUSHBACK_VALIDATION.md
und der Buildbericht in ../deploy/ belegen nur diesen alten Stand. Keine
Simulatorabnahme oder Implementierung der folgenden Entscheidungen.

## 1. Produkt und Persistenz

Auf derselben automatischen Route sollen Anfahren, Gerade, Kurven, Abbremsen
und bewegte Endausrichtung bei 2x in etwa der halben Bewegungszeit ablaufen.
Kein Zeitraffer fuer X-Plane, Handling, Ansagen, Wartezeiten oder Piloten.
AP88 und andere Tugs behalten ihr eigenes Ausgangsprofil, aber nicht dessen
unskaliertes Tempo als Obergrenze fuer die beschleunigte Bewegung.

Ein globales Auswahlfeld "Pushback speed" (nur BPB-Bewegung, kein Simspeed-Write): Normal (1x), 1.5x,
2x. Default 1x. 3x ist lediglich eine spaetere Ausbaustufe: kein UI-Eintrag,
kein versteckter Schalter; auch ein manuell eingetragenes 3 ergibt 1x.
Neuer Key pushback_time_factor; alten pushback_speed_factor nicht umdeuten
oder loeschen. Vollstaendiges numerisches Parsing; fehlend, fremder Wert,
Resttext oder nicht endlich -> 1x. Staged UI, Speichern nur idle, bestehende
Save-/Reloadguards erhalten. Snapshot nach erfolgreichem bp_can_start().
Wiederholter Start/Reconnect aendern die Auswahl nicht.

Unabhaengig von Fast Ground Handling, Classic/GO, Recall und Auto-Disconnect.
Automatische Emergency-Tow-Routen sind enthalten. Manual mit/ohne Yoke, Slave-
Physik, freie Tugfahrt und alle Handlingphasen bleiben auf dem Originalpfad.
Keine neuen Flugzeugpositions-/Geschwindigkeitswrites, keine globale Zeit,
keine Aenderung von Tugkonfigurationen, Planner/Kompass, Cache oder Bugrad-
Produzenten (ABI 1, 17 Elemente; beim Tragen weiterhin keine Rotation).
Die aktive Planneranzeige ist geometrisch, der inaktive Previewblock kein
Buildblocker. Fuer den spaeteren Zibomod-Port keine libacfutils-Abhaengigkeit.

## 2. Lokale Fahrregelung

1x verzweigt vor jeder neuen Rechnung auf den heutigen Defaultpfad. F=1.5
oder 2 wirkt nur auf automatische Bewegung des Masters. Keine Mutation der
globalen bp.d_t, bp.cur_t, Fahrzeugkonfiguration oder Messwerte. X-Plane
integriert Kraefte; kein mehrmaliges Durchlaufen desselben Messframes.

drive_segs erhaelt die echte Geometrie aus corr_acf_pos(), deren Geschwindigkeit
geteilt durch F, das originale Profil nach pb_step_tug_load und F*dt als
lokales Reglerintervall. Gerade, Kurve, Crawl, Overcorrection, rekursiver
Lookahead und Vorzeichen bleiben damit im Referenzraum. F multipliziert
den resultierenden longitudinalen Sollwert genau einmal. Der schnelle Regler
und die Bremshuelle verwenden beide Flugzeug-Laengsgeschwindigkeiten.
Nur 1x rechnet den Sollwert auf Tugtempo um. Die physischen Tug-m/s-Grenzen
MED/POOR werden fuer Fast mit max(cos(nw_defl),0.1) in den Laengsrahmen
projiziert. Bei Begrenzung ist Zeitgewinn nachrangig.

Nur bei 2x endet das Lenken entlang der Route mit einem gleitenden Uebergang
zum Endkurs am letzten Segment. Der feste Achspunkt und bei Boegen auch die
verbleibende Bogenlaenge bestimmen das Endfenster. Der Uebergang aendert
nur den Lenkbefehl, nicht Segmentfreigabe, Geschwindigkeitslimit oder
Stillstands- und Parkbremsbedingungen.

Weg, Radius und Lenkwinkel unveraendert; Geschwindigkeiten und Gierraten
physisch mal F, nominale Beschleunigungen mal F^2. Bei 2x also 1.00 statt
0.25 m/s^2 beim Anfahren und 0.68 statt 0.17 m/s^2 bei der Bremsplanung.
Das haelt ideal v^2/(2*a) konstant, garantiert aber keinen realen Bremsweg.

**G1: begrenzter Geschwindigkeitsregler statt F^3-Kraftschritten.** Nur im
Fastzweig von push_at_speed einen physischen PI-Geschwindigkeitsregler
verwenden. v ist die signierte gemessene Flugzeug-Laengsgeschwindigkeit
bp.cur_pos.spd, u der begrenzte Laengssollwert. Je echtem Sample: e=u-v; A=F^2*max_accel des Aufrufers;
a_cmd=clamp(kp*e+I,-A,A), kp=2*F/s, ki=F^2/s^2. I hat die Einheit m/s^2.
Die Integrationsaenderung ki*e*dt nur uebernehmen, wenn sie Beschleunigungs-,
Kraft- oder Rampensaettigung nicht weiter vertieft (oder aus ihr herausfuehrt).
Keine alte 100x-Breakaway-Verstaerkung im Fastregler. Konstanten sind
Startwerte fuer den Harness, keine bereits validierte Abstimmung.

Sollkraft=m*a_cmd; pro realer Sekunde hoechstens C Newton hinzufuegen/abbauen,
C=FORCE_PER_TON*m/1000, und Betrag vor jedem Write <= C. Damit bei 2x/20 FPS
hoechstens 5% C pro Sample statt 40% C mit blindem F^3. Einzustellende
Start-/Haftreibung ist expliziter Versuch: reicht die nominale Kraft nicht
zum Losfahren, keine beliebige Krafterhoehung. Bei angeforderter Fahrt
(auch Crawl und bewegte Ausrichtung) den festen Bezugspunkt und die Simzeit
merken. Nach 0.02 m Fortschritt in Fahrtrichtung oder Segmentwechsel neu
ankern; ohne Fortschritt fuer 2 s -> Fehlerhalt. Bei Hold, Gangwechsel,
Nullziel, Lichtstopp, Piloteninterrupt und Pause disarmen. Damit ist auch
1.5x-Crawl bei 0.15 m/s geschuetzt.

Nur endliche Daten und 0<dt<=0.1 s integrieren; echte Simulatorpause erzeugt
keinen Nachholschritt. Bei Zeitreset, ungueltiger Beobachtung oder groesserer
Luecke: Fastbewegung sperren, keine neuen Kraft-/Quaternionbeitraege aus
diesem Sample, Historie/Integral nullen, Zeit und Pose beim naechsten
gueltigen Sample neu ankern. Danach Fehlerhalt mit Nullsollwert; kein
automatischer Neustart. Der Guard muss vor fastabhaengigen Draw-/Tug- und
Step-Berechnungen greifen, nicht erst in push_at_speed().

## 3. Bremsweg und Fehlerhalt (G2)

Fuer jeden engeren Geschwindigkeitswechsel der verbleibenden gleichgerichteten
Route eine physische Laengsgrenze bilden. Nicht nur Distanz bis zum Ende
betrachten: Geraden, Kurveneintritt, Richtungswechsel, letzte Endausrichtung.
Gerade: Projektion des festen Bezugspunkts wie in drive_segs; Bogen: Restbogen
im richtigen Drehsinn plus folgende Segmentlaengen. Bereich endet am ersten
Richtungswechsel. Winkel-/Querbeschleunigungslimits des nachfolgenden Bogens
bestimmen dessen Eintrittstempo. Berechnung vor Segmentfreigabe nur lesend,
keine zweite Liste konsumieren. Wiederholte Abfragen duerfen keinen
zusaetzlichen Zustandsuebergang bewirken.

An jeder Grenze mit Restweg d und physischem Eintrittstempo w wird die
zulaessige Betraggeschwindigkeit aus einem konservativen Modell begrenzt.
Verwende identische longitudinale Einheiten fuer v,w,d; nicht Tugtempo mit
Rumpfstrecke vermischen. Bei starkem Schlepperwinkel die reale axiale
Kraftkomponente der tatsaechlichen Lenkung beruecksichtigen. Fuer die
aktive Bremsrechnung sei g=max(0,cos(steer)), s das Vorzeichen der Bewegung,
A_brake=0.25*F^2 m/s^2 die ausdrueckliche Gegenkraftautoritaet des
Fastfahrreglers und b=min(0.17*F^2, 0.8*g*min(A_brake,C/m)) in m/s^2.
Die zweite Schranke haelt Kraftreserve frei, beweist keine Haftung.
Bei g<=0 oder b<=0 nicht dividieren, sondern Fehlerhalt. Gegenkraftbetrag
B=m*b/g, Rampenrate R=C pro Sekunde, P=s*last_force. Bis zur Reaktion
nach hoechstens h=0.1 s kann die Zugkraft noch steigen: P_hi=min(C,P+R*h).
Deshalb t_r=h+max(0,P_hi+B)/R; in jedem Sample neu bestimmen.
Waehrend t_r weiterhin moegliche Beschleunigung a_plus einrechnen, nicht
bloss v*t_r. Konservativ a_plus=max(0,P_hi/m,A_brake), ohne Fremdkraefte.

    v_r = v + a_plus*t_r
    D(v,w) = v*t_r + 0.5*a_plus*t_r^2 + max(0, v_r^2-w^2)/(2*b)

Nur fuer Abbremsung (v>w) anwenden. Auf [w, requested_speed] per 24 festen
Bisektionsschritten hoechsten zulaessigen Wert mit D<=d bestimmen, sonst w;
kleinste Grenze aller folgenden Einschraenkungen verwenden. Das ist eine
zusaetzliche Geschwindigkeitsobergrenze, kein Schreibzugriff auf Bremsen.
Letzter Segmentabschluss benoetigt den vorhandenen Crawl zum Ueberfahren
der geometrischen Grenze: Eintritt w=F*CRAWL_SPEED, danach STOPPING bis zum
realen Stillstand; erlaubten Kriech-Nachlauf im Endlagenvergleich mitpruefen.
Kein Zielsollwert Null vor der Linie, der drive_segs dort festhalten koennte.
Am Richtungswechsel ebenfalls Crawl bis zur Segmentfreigabe, danach Nullziel.
Gegenrichtung erst bei realem Stillstand (<0.08 m/s) UND nach vier realen
Simsekunden freigeben. Keine neuen Bremsschreibstellen.

Nur wenn die Huelle bindet UND das Isttempo die modellierte Bremsgrenze
erreicht, G1-Integral nullen und Zielkraft -s*B statt PI-Antrieb vorgeben;
gleiche reale Rampe/Clamp. Aus dem Stand unterhalb der Huelle regulaer
anfahren; bei Erreichen des Eintrittstempos die Gegenkraft nicht bis zum
Vorzeichenwechsel halten. Bei fehlender positiver Bremsreserve,
Ueberschreiten der Huelle mit tatsaechlichem Tempo um >0.15 m/s fuer
zwei gueltige Samples oder ausbleibender Verzogerung trotz voll aufgebauter
Gegenkraft fuer 0.25 s: Fehlerhalt verriegeln. Kein Segmentfortschritt,
keine weitere Ausrichtung, kein neues op_complete; kontrolliertes Nulltempo,
bis der echte Stillstand erreicht ist. Sichtbarer Grund, vorhandener
Stop/Abort bleibt bedienbar, kein selbsttaetiges Resume oder Disconnect.
Fremdschub, Glatteis und unbekannte Aircraftplugins lassen keinen universell
garantierten Bremsweg zu. Die Huelle ist ein Planungsmodell; Fehlerhalt
erkennt dessen Verletzung, macht bereits zurueckgelegten Weg nicht rueckgaengig.

Fehlerlatch hat Vorrang vor Leerlisten-Sprung, Resume und STOPPING-Ausrichtung;
nicht ueber bp_request_pause abbilden (nur in PUSHING zulaessig). Expliziter
Soft Stop im Fehlerhalt merkt Endanforderung, behaelt Nullziel bis zum echten
Stillstand, dann ueber den bestehenden stationaeren stop_from_pause_hold-Pfad
ohne Translation in den normalen Parkbremshandoff. Nie eine Parkbremse
simulieren. Hard Abort hat uneingeschraenkt Vorrang. op_complete ist im
heutigen Protokoll bereits beim Routenende ein Signal an den Slave, NICHT
die Bestaetigung der Parkbremse. Bei spaeterem Fehler in STOPPING ein schon
gesendetes Signal weder zuruecknehmen noch erneut senden; Fehlerhalt ist
lokal keine Freigabe. Gekoppelten Fehlerfall gesondert pruefen: ohne Beleg
fuer synchrones Anhalten keinen beschleunigten gekoppelten Betrieb freigeben.

## 4. Kursowner, Kontakt und Uebergaben

**G3: Bestehende zwei Kursbeitraege nachbilden, nicht einen davon entfernen.**
Bei angekoppeltem Master im Fastfahrkontext (Tug ohne eigene Segmente)
entspricht der erste Beitrag dem leeren tug_run()-Schritt der 1x-Fahrt:
v_tug_alt aus dem privaten zuletzt gueltigen Flightloop-Sample (am Eintritt
mit aktueller gueltiger tug_speed()-Messung initialisieren), nie aus einem
von draw_tugs/pos_only ueberschriebenen tug->pos.spd. Am Ende des zweiten
Kursbeitrags dessen reale Messgeschwindigkeit fuer den naechsten Loop sichern;
nach Zeitfehler Historie invalidieren. v_ref_alt=v_tug_alt/F, in dt_motion gegen Null mit TUG_MAX_ACCEL bewegen;
wenn !steer_override, Winkel wie bisher mit TUG_STEER_RATE*dt_motion
neutralisieren. Erstes Kursinkrement aus v_ref_neu*dt_motion/r. Fuer
anschliessendes tug_anim den Zwischenwert wieder in reale m/s umrechnen
und dt_sim verwenden. Sound/TE nicht mit virtueller Zeit treiben. Zweiter
Beitrag unveraendert in tug_pos_update(): beobachtetes reales tug_speed()
mal dt_sim/r, harte Winkelanschlaege und echte Bugradanbindung. draw-Pfad
pos_only niemals integrieren. Keine weiteren Kursbeitraege. Faktor 1, Slave
und freifahrender Tug behalten tug_run() ungeaendert. Fuer turn_nosewheel
Tugtempo und Aircraft-Gierrate durch F teilen, fuer Winkelintegration und
Stellrate dt_motion benutzen. Nicht nochmals die integrierte Quaternion-
korrektur multiplizieren. Rueckwaertsasymmetrie des signierten Vergleichs
nach ang_vel_speed_limit nicht nebenbei reparieren; beide Seiten testen.

**G4: Ein physischer Kraftwert fuer alle Writer.** Nach Samplevalidierung
und Piloteninterrupt erst G1-Kandidaten begrenzen, dann Bodenkontakt lesen.
Ohne Kontakt ausschliesslich die vorher tatsaechlich ausgegebene Kraft
mit max(0,abs(last_force)-2*C*dt) ohne Vorzeichenwechsel zur Null fuehren;
Integral sperren und Fehlerhalt verriegeln. Vor Ausgabe erneut auf +/-C
begrenzen. Aus derselben Endkraft Fx/Fz, axialer Beitrag, Yaw und kraft-
bedingtes Pitchmoment berechnen; bestehender zusaetzlicher Tuggewichtsanteil
nur mit realer Masse/dt. Genau diese Endkraft in last_force ablegen. Bei
Piloteninterrupt bestehende Neutralisierung hat Vorrang, kein PI-Nachschreiben
und gespeicherte Kraft/Integral nullen. Keine neuen Dataref-Owner.

F bleibt im normalen Fahrt-/Pause-/Licht-/bewegten STOPPING-Pfad erhalten,
auch wenn bp_stop die Segmente geloescht hat. Vier Sekunden Gangwechsel,
Piloteneingriffe und Wartezeiten bleiben reale Simzeit. Soft Stop aus
stationaerem Hold darf keine Fahrt starten. Sind in pb_step_stopping die
Original-Ausrichtungsbedingungen und abs(v_phys)<0.08 m/s erfuellt,
Abschluss einmalig vor den bestehenden Brakewrites verriegeln; folgende
STOPPING-Frames nicht wieder beschleunigen. Direktsprung auf STOPPED, Hard
Abort, Disable und Reconnect invalidieren die Fahrhistorie ebenfalls.
STOPPED bis Complete ausschliesslich Originalpfad. Keine Aenderungen an
pbrake_is_set(), brakes_set(), Bremsdatarefs, Ignore-/Slave-Semantik,
fast_brake_handoff.h, Resetstellen, VERIFIED/RESTORE oder Cleanupguards.

## 5. Implementierungsgrenze und Nachweis

Erwarteter Diff: cfg.cpp/.h (Setting/Getter), pushback_speed.h und kleiner
reiner Bewegungshelper, bp.c/.h (Kontext, Aufrufer, Writer), driving.c/.h
(lesender Lookahead, alte Cap-API ersetzen), tug.c/.h (eng begrenzter erster
gekoppelter Schritt), Diagnosewriter, POT, README/USER_GUIDE und registrierte
Tests. Keine Aenderungen an Buildchain, Assets, Port oder Simulatorzeit.
Diagnose default-off: F, echte/Referenz-Soll-/Istwerte, Kraft vor/nach Limit,
Restweg, b, t_r, aktive Grenze und Fehlergrund. SI-Einheiten kenntlich machen;
Metadaten bei Moduswechsel korrekt rotieren. Keine Produktionslogflut.

1. Faktor 1 bitgleich gegen 72a3295; geschuetzte Writer-/Handoffsequenzen
   und bestehende Drivingreferenz c202b437 erhalten. Neue Configwerte
   einschliesslich abgewiesenem 3x, Close/Save/Reload und Reconnect testen.
2. Fuer 1.5 und 2 echte Controller-/Lenk-/Writerpfade mit rueckgekoppeltem
   Testmodell bei 20/30/60/120 FPS und variablen Frames. Gerade, kurze
   Route, beide Kurvenseiten vor-/rueckwaerts, Richtungswechsel, Kurvenende,
   Off-Path, Stop/Resume, Fehlerhalt, Zeitreset, Kontaktverlust, Reibung,
   Gegenkraft/Blockade. Explizit P<C bei letztem steigendem Antriebsframe:
   fuer h=0.1 s muss die Reserve den Wechsel von min(C,P+R*h) bis -B
   abdecken, auch in Rueckwaertsfahrt. Endliche Outputs, kein Integralaufstau, kein
   Nachholen, korrektes Vorzeichen und Weg-/Winkelgrenzen pruefen.
3. Beide gekoppelten Kursbeitraege einzeln gegen zeittransformiertes 1x
   vergleichen; Draw, Audio, Bugradstatus und Slave ohne Doppelfaktor.
   Classic/GO x Fast Handling an/aus x 1/1.5/2, Ignore an/aus, Manual,
   Reconnect, GRAB/WINCH, AP88. Realistische Parkbremse: Pedale gehalten,
   losgelassen, erneut gedrueckt; keine neue Bremsschreibstelle.
4. Simulatorvergleich mit gleichem Tug/Flugzeug, Masse, Route und trockener
   Piste, einschliesslich Standardflugzeug und privatem Zibomod. Reine
   Bewegungszeit getrennt von Handling und Bestaetigungen messen. Bei langen,
   unbegrenzten Routen gilt T(F)*F/T(1)=1 +/- 0.10; bei 2x also 45-55%
   der Referenzfahrzeit. Auch Kurven und Anfahren separat vergleichen.
   Endlage und Spur innerhalb derselben vor Kandidatenlauf festgehaltenen
   1x-Toleranz. Fehlerlauf/Halt niemals als schnellen Erfolg mitzaehlen.

Die Zahlen sind pruefbare Kandidatenparameter, kein Nachweis fuer Stabilitaet,
Haftung oder erreichte Fahrzeit. Erst Trockentests, dann Simulatorabnahme;
3x erfordert spaeter eine eigene Pruefung, keine Freischaltung per Config.
Build, Packaging, Deploy und Git-Schreiboperationen bleiben separat.

## 6. Reviewstatus

Der vorige Review identifizierte Kraftdiskretisierung, Bremsweg bei Saettigung,
gekoppelte Kursintegration und Kontakt-Recovery als offene Entscheidungen.
Abschnitte 2-4 legen fuer Stufe 1 konkrete Kandidaten fest. Der unabhaengige
Review dieser Fassung identifizierte vier konkrete Praezisierungen: tatsaechliche axiale
Gegenkraft, Blockade bei 1.5x-Crawl, Ausstieg aus Fehlerhalt und
Draw-unabhaengige Tug-Geschwindigkeitshistorie. Diese stehen jetzt in
Abschnitten 2-4. Im Gegenreview blieb eine Korrektur der Reaktionsreserve
offen: Kraftanstieg im letzten Frame vor der Gegenkraft. Obige Formel und
Testfall uebernehmen die konkrete Reviewkorrektur. Damit ist der Kandidat
fuer Implementierung und Trockentests spezifiziert, nicht fuer eine
Auslieferung validiert. Kein Laufzeit- oder Stabilitaetsnachweis.
