# Obnovení a přesný výběr 400Hz SHADOW logů — 11. 10. 2026

Implementováno: plná nominální frekvence vybírá každý skutečný shadow cyklus.
Unit/SITL/build ověřeno. Hardware ověřeno v obou 20s DISARMED oknech: 200 a 400 Hz,
všechny uložené sady platné a bez ztrát v měřicích oknech. Aktivní UTB_ACRO, motorová autorita a FÁZE 2 nejsou součástí změny.

## Zjištěné příčiny a rozsah opravy

Původní packed firmware 34990b72 zapsal po zdravém novém startu při 400 Hz
7 998 úplných **platných** sad v předem určeném 20s okně, tedy 399,90 Hz.
Seq měl dva skoky o 2, přestože queue/backend drops a missing v okně nepřibyly.
C 200 Hz měl 3 999 platných sad, tedy 199,95 Hz. Tyto výsledky nejsou přesný
bezeztrátový 400Hz výsledek. Raw data: `tmp/utb-400-recovery-20261011_173711/PACKED_CD_VALID.bin`,
7 529 246 B, SHA-256 `7c35d8a5107d3908ec0d2df155b6a439f6bb69148b327d7585e0c81b4f9dcae7`.

Původní AP_UTB::update vybíral i plnou frekvenci porovnáním TimeUS s další
časovou hranicí. Při předčasném cyklu kvůli běžnému timer jitteru mohl výběr
odmítnout aktuální vzorek. Regresní vstup 8 000 cyklů s jitterem −5 až +5 us
má v původním algoritmu pouze 7 272 vybraných sad. Nový select_log_sample
vrací true pro každý cyklus, pokud 1 <= rate <= 400, nominal_dt je konečné
kladné číslo a rate * nominal_dt >= 1. Na 400Hz controlleru s LOG_RATE=400
se tedy každému skutečnému cyklu přiřadí jedna diagnostická sada. Při pomalejší
frekvenci zůstává původní časové dělení. Nejsou vytvářeny žádné dodatečné
vzorky po zpoždění a log rate neurčuje frekvenci controlleru.

Samostatný problém postupu: MAVLink download běží synchronně v hlavním vlákně.
AP_Logger_MAVLinkLogTransfer.cpp:107–157 zjišťuje info/boundaries při novém
požadavku; :215–252 může poslat až 250 LOG_DATA paketů na USB. BLOCK backend
AP_Logger_Block.cpp:461, :602 a :770 čte flash pod původními zámky.
Host požadavky po 270 B neodstranily dlouhé periody: opakují také hledání
hranic logu. Pozorované maximum bylo 35 828 us a EKF přestal být zdravý.
Další download po 8 190 B dosáhl maxima 41 972 us. Obojí nastalo **až po**
ukončení obou měřicích oken při SHADOW=0/LOG_DISARMED=0. Přesná vnitřní
větev, která EKF ponechá unhealthy, není tímto pořadím událostí prokázána.
Logger/HAL/EKF nebyly měněny. Nejde o opravu samotného log transferu.

Nový host nástroj Tools/autotest/utb_hardware_capture.py ověří identitu a
konfiguraci, DISARMED heartbeat, skutečný STATE_VALID s potlačeným experimentem,
RC, logger a worker. Logger se nejprve rozběhne při SHADOW=0. Následuje C a D
v jednom zdravém startu, 15s warmup a předem zadané okno pro každý rate.
Nástroj neumí download, erase, reboot, upload, arm ani motor test; po běhu
vypne SHADOW a LOG_DISARMED. Kontroluje úplné UB00–UB36 rámce, čerstvost,
platnost, stack UNKNOWN/zero, lifecycle a změny loss counters. Pravidlo pro
nové maximum > 3 000 us vychází z AP PerfInfo 1,2násobku periody; porovnává
se i s historickým maximem. Toto kumulativní pravidlo neumí zachytit všechna
nová kratší zpoždění; proto je nutná offline PM/jitter kontrola.

## Firmware a zdroje

Commit: `acf04efa99a478a03e1d2d8847f0ddc0b0a4c973`.
Commit mění pouze AP_UTB.cpp, AP_UTB.h a tests/test_utb.cpp.
BENCH_ON APJ git_identity `acf04efa`, board_id 1075, image_size 1 250 104 B.
APJ: 1 127 711 B, SHA-256
`41c42eb92550a23266c6548b81ccc95f0e6e6642af28816220835217198616f5`.
Artefakt:
`tmp/utb-400-recovery-20261011_173711/SkystarsH7HD-bdshot-acf04efa99-UTB_ON-BENCH_ON-PACKED_FULL_RATE/arducopter.apj`.

AP_UTB_BENCH_ENABLED=1 je v archivovaném hwdef.h; ELF obsahuje UTBBench::metrics,
worker_enter, utb_bench_thread i nový select_log_sample. Upload na stejné FC
COM5, USB serial 200024001251303433333336, board_id 1075: CRC ověřeno, bez force.
Po bootu je git_identity potvrzena přes AUTOPILOT_VERSION.

První PARAM_REQUEST_LIST během startu postrádal 8 MAV1 položek. Pozdější
readback všech 1 210 položek a čtení každého chybějícího jména potvrdilo
původní hodnoty. Automaticky se změnily gyro offsets/calibration temperatures
(INS_GYR_CAL=1, AP_InertialSensor.cpp:970, :1827, :1846), COMPASS_DEC
(COMPASS_AUTODEC=1, AP_Compass.cpp:2007), BARO1_GND_PRESS
(volatile/internal, AP_Baro.cpp:101, :398, :1009) a STAT počítadla.
Žádné tyto hodnoty nebyly host nástrojem přepisovány. První neúspěšná kontrola
uploadu je zachována ve fixed-upload-result.json; následné vysvětlené ověření
je fixed-upload-validation.json. Frame, gains, arming, failsafe a logger konfigurace
jsou zachované.

## Software ověření a režie

- SITL UTB ON/BENCH ON build a Skystars UTB ON/BENCH ON build PASS.
- 47 UTB unit testů (42 původních + 5 log-selection regresí), 12 BENCH unit testů PASS.
- 12 Python testů (6 decoder + 6 capture stop conditions), flake8, astyle a diff check PASS.
- FÁZE 0 SITL: runtime OFF, normální/forced/RC arming rejection a armed entry rejection PASS.
- Packed SHADOW SITL: 199,997399 Hz / 399,993331 Hz, 25 789 packed paketů,
  0 partial sets; scénář cíleně zahrnuje logger loss a queue drops. Není to HW timing.
- SITL motor isolation: 0 rozdílů steady RCOU v porovnávaných AP fázích.
- Source audit: evaluate(), RateController, MotorMixer, Reference, State,
  Copter UTB bridge/mode, AP_Motors, AP_Logger, HAL a EKF beze změny proti 34990b72.
  Žádná nová cesta AP_UTB -> motor writer, žádné nové blocking API/heap/atomiky.

Nový selector nemá hodinový ani logger call, alokaci či zámek. ARM disassembly
plné větve používá konečný počet float porovnání/násobení a přímý návrat;
nižší rate zachovává původní výpočet periody. Firmware image proti 34990b72
vzrostl o 112 B. Přesný izolovaný CPU overhead selectoru nebyl změřen;
producer wall time níže měří celou původní cestu s novým výběrem, nikoli pouze tuto funkci.

První build pokusy postrádaly PATH existující venv a ARM toolchain. Po nastavení
PATH podle README prošly. Nebyl instalován žádný balíček. Vypnuté varianty
UTB/BENCH/logging nebyly v této malé opravě znovu sestavovány; předchozí build
matrix je evidence předchozího packed patche, nikoli této revize.

## Reprodukce

```bash
cd /home/jirka/ardupilot_utb
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board sitl --enable-UTB --enable-UTB_BENCH
./waf copter --targets tests/test_utb,tests/test_utb_bench -j4
build/sitl/tests/test_utb
build/sitl/tests/test_utb_bench
.venv/bin/python -m unittest discover -s Tools/autotest/unittest -p 'test_utb_*py'
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH \
  --extra-hwdef "$PWD/tmp/utb-1ha/bench-on.hwdef"
./waf copter -j4
```

Capture běží na Windows s existujícím pymavlink/pyserial. Explicitní port,
identita firmware, odstraněné vrtule a nový výstupní adresář jsou povinné:

```powershell
$taskRepo = '\\wsl$\Ubuntu\home\jirka\ardupilot_utb'
$taskPython = Get-Content -LiteralPath "$taskRepo\tmp\utb-400-recovery-20261011_173711\windows-python-path.txt"
$env:PYTHONPATH = "$taskRepo\.venv\lib\python3.14\site-packages"
& $taskPython "$taskRepo\Tools\autotest\utb_hardware_capture.py" --port COM5 --firmware acf04efa --props-removed --duration 20 --output "$taskRepo\tmp\novy-adresar-capture"
```

Před session: DISARMED/bez vrtulí, zdravý nový start, záloha parametrů/logů,
RC/gains/frame/backend/firmware readback a volné médium. Erase pouze po samostatném
souhlasu a ověřeném archivu. Po C/D nejprve SHADOW=0/LOG_DISARMED=0, potom
stáhnout BIN. Po downloadu před dalším experimentem nový zdravý start a znovu
ověřit guards. Nevynucovat zdraví změnou AHRS/gyro/age/arming kontrol.

## Hardware výsledek opraveného firmware

**PASS pro omezený 20s DISARMED benchmark každé frekvence.** Vrtule odstraněné,
RC aktivní, AP mode ALT_HOLD (2), FRAME_CLASS=1/FRAME_TYPE=18, FSTRATE=0,
LOG_BACKEND_TYPE=4, LOG_FMT=1, SCHED_LOOP_RATE=400. Všechny UTB P/I/D/IMAX/D_HZ
zůstaly 0. Nejde o experiment s nenulovými gains ani o uzavřenou UTB smyčku.

| Metrika | C 200 Hz | D 400 Hz |
|---|---:|---:|
| Předem zvolené TimeUS okno | [317000000,337000000) | [357000000,377000000) |
| Uložené kompletní platné sady | 4 000 / 4 000 | 8 000 / 8 000 |
| Skutečně uložená frekvence | 200,00 Hz | 400,00 Hz |
| UTBR.Val / UTBM.Val / Cap | 1 / 1 / 1 | 1 / 1 / 1 |
| UTBS Calc / Flg / Pol / Obs | 0 / 1023 / 0 / 0 | 0 / 1023 / 0 / 0 |
| PM.NLon (oba reporty) | 0, 0 | 0, 0 |
| PM.MaxT | 2 546 / 2 541 us | 2 572 / 2 547 us |
| PM.Load | 21,2 / 21,2 % | 21,2 / 21,3 % |
| Main period mean (UBMT delta) | 2 499,999126 us | 2 500,000999 us |
| Mean absolutního main jitteru | 2,427680 us | 2,367495 us |
| Producer mean / cumulative max | 38,181580 / 51 us | 39,751092 / 52 us |
| Consumer mean / cumulative max | 11,292674 / 1 732 us | 13,355664 / 1 732 us |
| Main acquire mean / cumulative max | 0,525358 / 74 us | 0,513929 / 74 us |
| Worker hold mean / cumulative max | 4,369136 / 128 us | 4,385351 / 253 us |
| Worker stack minimum / actual region | 1 784 / 3 504 B | 1 784 / 3 504 B |
| Queue HWM / endpoint depth | 4 / 0 | 4 / 0 |
| Delta Drop / Miss / Gone / create failures | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| DSF.Dp / delta | 0 / 0 | 0 / 0 |
| DSF RAM buffer minimum | 49 877 B | 42 184 B |
| Celkový byte rate uloženého logu | 70 463,10 B/s | 118 828,90 B/s |
| Potvrzené main/worker kolize | 0 | 0 |
| UNKNOWN attribution delta | 10 056 | 10 015 |
| UBST[8] task budget overruns delta | 295 | 286 |
| UBST[9] period > nominál delta | 2 437 | 2 441 |

UBMT a UBST delta intervaly jsou [316212489,336244661) a
[356272304,376300042) podle dostupných exporter endpointů. Nejsou to přesně
20s full-window packet intervaly; neporovnávat proto jejich Wr delta 4 006/8 011
s přesným uloženým full-window počtem 4 000/8 000. Maxima/minima a watermark
jsou historické od startu, nikoli izolované maxima oken. Main period maximum
UBST[29]=5 370 us pochází z dřívějšího rozběhu logování. V měřicích oknech se
nezvýšilo. PM reporty shrnují své předcházející 10s intervaly.

D má všech **7 999 sousedních Seq rozdílů přesně 1**. Průměrná perioda UTBC
TimeUS je 2 499,999375 us, min 2 436 us, max 2 565 us; jde o timestamp místa
shadow samplingu, nikoli samostatný mutex či scheduler timer. C zachovává
původní wall-time decimation: Seq rozdíly 1/2/3 jsou 918/2163/918 a průměrná
perioda sad 5 000,003251 us. Tento výběr není rovnoměrné uložení každého druhého
cyklu; nevysvětluje se ztracenými main cykly. Budoucí uniformní 200Hz decimation
by byla samostatná změna.

**Startup a download nejsou bezeztrátovým timing důkazem:** při pre-health probe
s loggerem vypnutým se nahromadilo 17 neuložených vybraných diagnostik a 4 Gone;
Miss=0. Tyto hodnoty se v obou měřicích oknech neměnily. Logger-start PM
TimeUS=304813864 má NLon=1 a MaxT=5 370 us; následný warmup PM již NLon=0.
Warmup byl oddělen předem a neodstraňuje existenci startup události.

BIN: `PACKED_CD_FIXED.bin`, 7 654 746 B, SHA-256
`ed08afb62c8dc575353dda96dfe649655178f9d88aca9ea2bfef4f1307a533be`.
Download začal až po freeze obou variant. Po downloadu maximum main periody
vzrostlo na 32 502 us; EKF_STATUS_REPORT na konci naopak zůstal flags=167.
To potvrzuje dlouhou prodlevu downloadu, nikoli opravu této původní logger
cesty ani nutný vznik EKF unhealthy při každém přenosu. V uloženém BIN bylo
823 RCOU reportů a všechny měly C1–C4=(1000,1000,1000,1000).

Finální readback 1 210 parametrů: DISARMED, UTB_ENABLE=1, UTB_SHADOW=0,
LOG_DISARMED=0, UTB_LOG_RATE=400, UTB_LOG_FMT=1. Proti ověřenému ustálenému
stavu po uploadu se změnil pouze STAT_RUNTIME. Žádný fyzický motor/arming příkaz
nebyl použit.

**Neověřeno:** dlouhodobá udržitelnost a zaplněné médium, neblokující download,
nejhorší mutex/stack zátěž, spolehlivost opakovaných software rebootů, nenulové
UTB gains, dynamika aktivní UTB smyčky a jakýkoli let. Nula potvrzených kolizí
nevylučuje blocking; UNKNOWN zůstává nenulové. Přesná příčina přetrvávajícího
EKF unhealthy po předchozích transferech je otevřená. Současné 20s výsledky
nejsou povolením aktivního UTB_ACRO.


## Manifest změn této opravy

| Soubor | Zásah |
|---|---|
| libraries/AP_UTB/AP_UTB.cpp | Vyčlenění původního výběru logů a plný výběr při nominální frekvenci. |
| libraries/AP_UTB/AP_UTB.h | Deklarace deterministicky testovatelného selectoru, bez nových member dat. |
| libraries/AP_UTB/tests/test_utb.cpp | 5 regresí: jitter, nižší rates/rychlejší loop, přepnutí/reset, bez backfill, neplatný rate/clock boundary. |
| Tools/autotest/utb_hardware_capture.py | Oddělený bezpečný host capture bez stahování mezi C/D. |
| Tools/autotest/unittest/test_utb_hardware_capture.py | 6 regresí úplnosti diagnostiky a stop podmínek. |
| docs/UTB_400_RECOVERY_20261011_CS.md | Tato evidence, firmware, přesný postup a omezení. |
| docs/UTB_PACKED_HW_20261011_CS.md | Odkaz z historické session na novou revizi. |
| docs/UTB_PACKED_LOG_CS.md | Aktuální stav výběru a hardware ověření. |
| docs/UTB_HARDWARE_BENCH_CS.md | Oddělení capture od downloadu a aktuální stav ověření. |
| docs/UTB_ARCHITEKTURA_CS.md | Implementovaný diagnostický výběr a hardware scope. |
| README_BUILD_WSL_CS.md | Ověřené build/capture příkazy, aktuální artefakt a stav. |

Checkpoint před opravou: baseline-tracked.patch, baseline-untracked.tar.gz a
baseline-untracked.json v session adresáři. Všechny nové zdrojové a dokumentační
soubory jsou záměrné. Změny po zdrojovém commitu jsou host nástroj a dokumentace;
žádná další C++ změna není mimo tento commit. Detailní status/manifest/evidence
jsou v témže adresáři.
