# FÁZE 2A: validace aktivního UTB RateController v ArduPilot SITL

**PASS v popsaném malém nesaturovaném SITL experimentu.** Existující UTB
RateController skutečně řídil simulované těleso přes původní AP_Motors,
zatímco původní AP rate PID měl v UTB experimentu nulový počet volání.
Roll/Pitch sledují malé reference dobře; Yaw ještě vyžaduje samostatné ladění.
Výsledek není schválení aktivního řízení na fyzické FC ani produkční tune.

Větev `test/skystars-5inch`, HEAD
`687f825d228b12a64d36651072aed19356178217`, změny v pracovním stromu.
Nebyl vytvořen commit ani push. Nebyla instalována další závislost.
Aktivní architekturu popisuje
[UTB_ACTIVE_SITL_ARCHITECTURE_CS.md](UTB_ACTIVE_SITL_ARCHITECTURE_CS.md).

Veškerá nová evidence je v
`tmp/utb-2a-sitl-20261013_164236/`.
BIN, receive.tlog, konzole, params, JSON, CSV a grafy jsou archivované lokální
artefakty; `tmp/` není součástí zdrojového commitu.
[Manifest zdrojů](../tmp/utb-2a-sitl-20261013_164236/source-manifest.json),
[manifest artefaktů](../tmp/utb-2a-sitl-20261013_164236/artifact-manifest.json),
[závěrečný audit](../tmp/utb-2a-sitl-20261013_164236/final-audit.json).

## Implementováno a testovaný rozsah

Mode 41 UTB_RATE_SITL je dostupný pouze pro skutečný HAL_BOARD_SITL,
AP_UTB_ENABLED=1 a AP_UTB_RATE_SITL_ENABLED=1. Default flag i S_OWNER jsou 0.
Vstup vyžaduje explicitní GCS request, zdravý AHRS/INS, 400 Hz, frame 1/18,
FSTRATE=0 a SHADOW=0. Původní UTB_ACRO=40 zůstává DISARMED/unarmable.
S_OWNER=1 používá existující UTB PID, S_OWNER=2 původní AP PID v identickém
reference/throttle shellu. Nejde o srovnání úplného původního ACRO módu.

Příprava: původní AP GUIDED, standardní virtuální arming s ARMING_CHECK=1,
FS_THR_ENABLE=1, FS_GCS_ENABLE=1 a skutečným GCS heartbeat. Žádné force-arm
pro kladný letový test. Po takeoff 10 m a klidovém gyro/attitude ověření
následuje explicitní entry do 41, dvě sekundy nulové reference a finite
3200-cycle experiment. Po dokončení následuje explicitní LAND cleanup a
standardní disarm. V měřicím okně není AP fallback, altitude/position hold
ani druhý motorový output. Host tilt stop je 35°, controller gyro norm stop
2 rad/s a target norm stop 0,6 rad/s; jde o bounds malého simulačního testu.

## Gains: SITL ONLY

První zadané hodnoty P=0,31, I=0,23, D=0,004, IMAX=0,19, D_HZ=20
byly vyzkoušené jako výchozí software konfigurace. Při zdravém klidném
začátku a nulové referenci způsobily rostoucí R/P oscilace a allocation limit
po 1,052079 s; experiment se zastavil SATURATION, bez přechodu na AP PID.
Vyhovující omezená změna byla **jediné snížení P/I/D na polovinu**.
Následné testy neprováděly automatické hledání extrémních gains.

| Vlastník / osa | P | I | D | IMAX | D cutoff |
|---|---:|---:|---:|---:|---:|
| UTB Roll/Pitch/Yaw | 0,155 | 0,115 | 0,002 | 0,19 | 20 Hz |
| AP Roll/Pitch | 0,135 | 0,135 | 0,0036 | 0,5 | 20 Hz |
| AP Yaw | 0,3 | 0,02 | 0 | 0,5 | 20 Hz |

AP gains jsou skutečné původní SITL defaults, ověřené z PARM. Pro srovnání
se po přípravě u obou vlastníků nastavilo ATC_RAT_*_FLTT=0, FLTE=0 a FF=0.
Shaping v `input_rate_bf_roll_pitch_yaw_2_rads()` zůstává společný. UTB měří
derivaci gyra, AP derivaci chyby; srovnání není tvrzení matematické identity.
Žádné produkční defaulty gains nebyly změněny. Tyto hodnoty jsou **SITL ONLY**,
nejsou tune skutečného 5″ dronu.

## Skutečné odezvy a férovost porovnání

Model `bfxrev`, FRAME_CLASS=1, FRAME_TYPE=18; stejné motor ordering, pohon,
baterie, scheduler a nulový definovaný vítr/noise. Amplitude 0,15 rad/s.
Základní step: 1 s zero, 2 s +reference, 1 s zero, 2 s −reference, 2 s zero.
Kombinace je R=+v, P=−v, Y=+v. Sinus má 0,5 Hz; sign profile střídá znaménko
po jedné sekundě. Poslední phase=2 sample patří do konce sekvence.

Proběhlo **18 úspěšných experimentů**: zero, roll, pitch, yaw, combined,
sine, sign, inertia a delay, každý pro UTB i AP. Každý má **3200/3200**
úplných souvislých sad, 399,993348 Hz, žádný fault/saturation a full spool.
CELKEM 57 600 úspěšných měřicích sad. Chybové pokusy zůstávají odděleně.

Porovnání se zarovnává podle indexu generátoru, raw reference a skutečného
shaped target. Maximum target rozdílu je 4,34e−19 rad/s; raw reference jsou
shodné. Nezávislé TimeUS fáze se liší až 833 µs, méně než jedna nominální
2,5ms iterace. Nejde o bitově shodnou absolutní časovou značku. Počáteční
gyro rozdíl mezi páry má maximum 0,001909 rad/s; oba starty jsou ověřené
v klidovém rozsahu, nikoli prohlášené za identický úplný stav EKF/tělesa.
Srovnání je párové a jednoopakové, není statistický test lepšího regulátoru.

Definice: IAE = integral abs(used target − actual gyro), ITAE navíc násobená
časem od začátku celé osmivteřinové sekvence, s reálným logovaným virtual dt.
Step rise 10–90 % a overshoot používají raw konstantní plateau, tedy zahrnují
společné shaping. Settling vyžaduje setrvání v ±2 % do konce plateau a alespoň
0,25 s zbývajícího pozorování. Steady error je průměr posledních 0,5 s plateau.
Sine nemá step rise/settling; nedosažených 90 % má hodnotu null, nikoli 0.

| Kladný samostatný step | Rise UTB / AP [ms] | Overshoot UTB / AP [%] | Settling UTB / AP [s] | IAE celých 8 s UTB / AP [rad] |
|---|---:|---:|---:|---:|
| Roll | 72,47 / 59,98 | 0,93 / 0,51 | 0,177 / 1,228 | 0,02353 / 0,02004 |
| Pitch | 74,97 / 59,98 | 0,52 / 0,44 | 0,993 / 0,903 | 0,02394 / 0,02021 |
| Yaw | nedosaženo / nedosaženo | 0 / 0 | neustáleno / neustáleno | 0,38817 / 0,26306 |

ITAE Roll: UTB 0,08703 / AP 0,07413 rad·s; Pitch 0,08702 / 0,07796;
Yaw 1,54279 / 0,94743. Max UTB normalized output v samostatných testech
Roll 0,00971, Pitch 0,00974, Yaw 0,02047. Žádná detekovaná saturace.
Negativní R/P rise ~75 ms, overshoot 1,16/1,20 %. Detail všech plateau,
včetně kombinace, sign profile a time metrics, obsahuje
[analysis.json](../tmp/utb-2a-sitl-20261013_164236/analysis-final/analysis.json).

Yaw UTB dosáhl během kladného 2s plateau maximálně 78,8 % požadavku;
mean error na jeho konci je 0,03570 rad/s. V záporném plateau má maximum
71,1 % a mean error −0,04852 rad/s. AP v tomto shellu rovněž nedosáhl 90 %
(kladný peak 70,0 %, mean error 0,04515 rad/s), ale má nižší celkové IAE/ITAE.
UTB tedy prokazuje správné uzavřené rate řízení všech os, nikoli přesné nebo
dokončené ladění Yaw. Ani jeden nulový overshoot při takovém undershootu
není interpretován jako kvalitní tracking.

![Roll skutečné gyro UTB/AP a sdílený target](../tmp/utb-2a-sitl-20261013_164236/analysis-plots/roll-rates.png)

![Yaw skutečné gyro UTB/AP a sdílený target](../tmp/utb-2a-sitl-20261013_164236/analysis-plots/yaw-rates.png)

![UTB skutečné P/I/D a aktivní output](../tmp/utb-2a-sitl-20261013_164236/analysis-plots/roll-utb-pid.png)

![Motorové actuators a počty volání](../tmp/utb-2a-sitl-20261013_164236/analysis-plots/roll-authority.png)

Pro všech devět párů jsou uloženy rates/PID/authority grafy jako PNG i SVG,
se všemi 3200 samples, bez decimace. Jednotlivé nuly v host µs nejsou chybějící
sample: mohou znamenat dobu kratší než rozlišení celých µs.

## Zpoždění a fyzikální varianta

Přidaný delay je přesně jeden sample R/P/Y adapteru (nominal 2,5 ms), shodný
pro oba vlastníky. Není to změna HAL/ESC/sensoru. UTB Roll IAE 0,02745,
kladný overshoot 2,05 %, 2% settling v tomto okně nedoloženo. AP IAE 0,14741,
kladný overshoot 18,06 %, záporný 31,35 %, settling nedoloženo. Žádná detekovaná
saturace/divergence, ale časově kmitavější baseline se neprohlašuje za stabilní
pro všechna zpoždění. Výsledek nelze přenést na skutečný ESC.

Skutečný SIM_Frame loader použil `moment_inertia` +10 % všech os, bez změny
mass/geometry/pohonu. Default Jx=Jy=0,02296875, Jz=0,0459375 kg·m²;
nové Jx=Jy=0,025265625, Jz=0,05053125. Parametry jsou ilustrativní pro tento
SITL model, neidentifikují 5″ Skystars. Oba vlastníci prošli; UTB Roll rise
77,47 ms, kladný overshoot 0,99 %, IAE 0,02301; AP rise 62,48 ms,
overshoot 0,42 %, IAE 0,02000.

## Důkaz autority, motor ordering a izolace

V celém úspěšném UTB měřicím okně APN=0 a UN=MN=Seq; v AP baseline UN=0
a APN=MN=Seq. Sekvence je souvislá a každý cycle má právě jeden motor callback.
Actual requested output odpovídá součtu skutečných UTB P/I/D; při delay=1
součtu předchozího sample. Největší rozdíl je pod 1e−7 normalized units.
AP actuator projekce proti audited BF_X_REV R/P/Y sloupcům mají kladné
znaménko. Nejde o návrat achieved momentu ani nový anti-windup feedback.
M1 rear-right CCW, M2 front-right CW, M3 rear-left CW, M4 front-left CCW;
AP_MotorsMatrix a skutečný SIM_Frame bfxrev ordering byly porovnány přímo.

Disarmed finální integrační test uložil 10 956 UARX / 3652 UAMT packets:
APN=0, UN=3652, arm=0, R/P/Y output=0 a PWM ≤1000. Neúplné startup logy
nejsou používány jako důkaz kompletních armed sad.
DISARMED fault suite potvrdila FRAME/FSTRATE/CONFIG fault, freeze owner/profile/
gains, SHADOW conflict, RC activation rejection, latched stop a recovery teprve
po explicitním exit/entry. UN po faultu neroste, APN zůstává 0.
Neplatné dt, NaN/Inf, stale/repeated IMU, clock wrap, resets a allocation
guard jsou navíc testované izolovanými C++ unit testy.

Původní AP rate PID, motor scheduler ordering, AP_Motors, AC_AttitudeControl,
rate_thread, HAL/DShot, EKF, arming/failsafe a původní SHADOW producer/worker
nebyly upraveny. V Copter rate slotu je jen guarded dispatch; v motor slotu
jen guarded read-only log hook. Custom controller a vznik FSTRATE threadu
jsou blokované pouze během explicitního experimentálního mode 41.
Source audit netvrdí nový hard deadline guarantee ani obecnou AP allocator
ekvivalenci; omezení saturace a throttle helpers uvádí architektonický dokument.

## Buildy a regrese

Přesný source manifest obsahuje **11 upravených + 10 přidaných souborů**.
Předchozích 18 untracked souborů je evidovaných samostatně, beze změny hashů.

| Soubor | Zásah |
|---|---|
| ArduCopter/Attitude.cpp | Guarded výhradní rate dispatch, bezpečný dt pro původní infrastrukturu při faultu |
| ArduCopter/Copter.h | Instance/friend nového módu, lokální FSTRATE/custom-control ochrany |
| ArduCopter/mode.h | Guardovaná enum hodnota 41 a deklarace nového módu |
| ArduCopter/mode.cpp | Guardovaná registrace 41 |
| ArduCopter/mode_utb_rate_sitl.cpp (nový) | Entry/exit, reference, throttle/spool, vlastníci, faults, read-only logging |
| ArduCopter/motors.cpp | Jeden post-output callback, bez nového output volání |
| ArduCopter/defines.h | Config include a guarded log IDs 24/25 |
| ArduCopter/Log.cpp | Guardovaná metadata registrace UARX/UAMT |
| ArduCopter/utb_sitl_log.h (nový) | Fixed packet struktury a pole/unit metadata |
| libraries/AP_UTB/AP_UTB_config.h | Default-OFF flag a skutečný HAL_BOARD_SITL availability guard |
| libraries/AP_UTB/AP_UTB.h | Guardované S_* parametry/getters a read-only získání původních gains |
| libraries/AP_UTB/AP_UTB.cpp | Guardované AP_Param indexy 23–27, bez zásahu do evaluate/PID/mixeru |
| libraries/AP_UTB/AP_UTB_SITLRate.h (nový) | Fixed stav a kontrakt helperu |
| libraries/AP_UTB/AP_UTB_SITLRate.cpp (nový) | Použití původního PID, validace, latch/reset, reference a transport buffer |
| libraries/AP_UTB/tests/test_utb_sitl_rate.cpp (nový) | Devět cílených guard/reset/math unit testů |
| Tools/scripts/build_options.py | Waf option s defaultem 0 a dependency UTB |
| Tools/autotest/test_utb_active_sitl.py (nový) | Izolované skutečné SITL scénáře, normální arming, finite guard a export |
| Tools/autotest/utb_active_sitl_analysis.py (nový) | Striktní CSV evidence, metrics, alignment, motor provenance, grafy |
| Tools/autotest/unittest/test_utb_active_sitl_analysis.py (nový) | Odmítnutí neúplné/falešně zarovnané/nonfinite evidence a definice metrics |
| docs/UTB_ACTIVE_SITL_ARCHITECTURE_CS.md (nový) | Skutečná architektura, authority a omezení |
| docs/UTB_ACTIVE_SITL_VALIDATION_CS.md (nový) | Naměřené výsledky, chyby, buildy a reprodukce |

| Varianta | Výsledek | Flash (text+data) / BSS [B] |
|---|---|---:|
| SITL UTB ON, authority ON, BENCH OFF | PASS | 4 647 683 / 227 680 |
| SITL UTB ON, authority OFF, BENCH ON | PASS | 4 647 405 / 227 296 |
| SITL UTB OFF, authority OFF, BENCH OFF | PASS | 4 610 277 / 224 096 |
| Skystars UTB ON, authority request ON, BENCH ON | PASS, aktivní code absent | 1 251 892 / 128 792 |
| Skystars UTB ON, authority OFF, BENCH OFF | PASS | 1 245 056 / 128 588 |
| Skystars UTB OFF, authority OFF, BENCH OFF | PASS | 1 233 328 / 125 704 |

Finální SITL ELF (5 724 808 B) má SHA-256
`8f9c5b763e24f821567f9c2bae1b31713e7e308466f8fc1f5e5a5a3760ddbe03`.
Je bitově totožný s ELF všech 18 úspěšných experimentů; formátování nezměnilo
binárku. Finální H743 ELF/APJ rovněž odpovídá předchozímu této fáze buildnutému
H743 BENCH_ON/request_ON artefaktu.

- Původní `tests/test_utb`: **47 PASS**.
- Nové `tests/test_utb_sitl_rate`: **9 PASS**.
- `tests/test_utb_math`: **4 PASS** s 75 226 nezávislými replay vectors,
  žádný parity mismatch. První běh bez env měl očekávaný OfflineVectors SKIP;
  následný replay toto pokrytí skutečně doplnil.
- Nové Python measurement unit testy: **4 PASS**.
- FÁZE 0: compiled ON/ENABLE=0, compiled ON/ENABLE=1 a compiled OFF **PASS**;
  GCS normal/forced/RC arming rejection i odmítnutí entry do 40 při arming.
  Pozitivní STABILIZE arming byl běžný, po 45s warmup; legacy runner byl
  spuštěn s explicitním wrapperem, který zakázal pozitivní force-arm.
- SHADOW BF_X_REV: **PASS**, 11 024 validních sad, PID/reference/mixer parity
  max 2,24e−8, 200Hz 199,99944 a 400Hz 399,90698; nulový DISARMED output
  rozdíl mezi SHADOW ON/OFF, geometry epochs/recovery a arming zákazy.
- Authority compile OFF runtime: mode 41 i všech pět S_* parametrů absent,
  původní AP manuální módy dostupné; totéž pro UTB compiled OFF.
- Skystars ELF: ModeUTBRateSITL, AP_UTB_SITLRate, jméno módu, S_* a UARX/UAMT
  absent, i když CXXFLAGS skutečně obsahují požadavek authority=1.
  AP_UTB a UTBBench jsou v odpovídajících ON variantách přítomné.
- flake8, astyle nových C++ souborů a změněných oblastí, `git diff --check`
  prošly. Původní oblasti nejsou plošně formátované.

Kompletní upstream multi-vehicle/autotest suite nebyla spuštěna. Původní
implementace módů je zachovaná source auditem a uvedenými cílenými regresemi;
není zde nové tvrzení o otestování všech původních letových scénářů.

## Časování a overhead

U všech 18 úspěšných oken je virtual period minimum 2499 µs, maximum 3332 µs,
průměr 2500,0416 µs; mean absolute jitter vůči 2500 µs je 2,0391 µs.
PM v celých procesech má obvykle NLon=4–5 za desetisekundový interval,
MaxT=3332 µs, Load=0. PM intervaly obsahují přípravu i cleanup; nejsou čistým
8s experimentálním oknem. Nulový PM.Load není měření host/H743 CPU.

Rate slot Ex používá native `std::chrono::steady_clock`, nezávisle na
virtuálním SITL TimeUS/dt. Napříč UTB testy bylo maximum 125 µs,
AP maximum 479 µs; jednotlivé rozdělení/průměr/P99 jsou v analysis.json.
Je to wall duration celého rate adapteru, může obsahovat host preemption;
nezahrnuje čtyři následné DataFlash writes ani dobu motor outputu.
Nelze z něj odvodit H743 execution time nebo deadline margin.
Aktivní telemetry/logging existuje jen v SITL; při physical compile-out
není jeho runtime overhead přítomen. Původní BENCH/SHADOW overhead se
nepřisuzuje tomuto aktivnímu prototypu.

## Neúspěšné pokusy a jejich omezené opravy

1. První runner předal relativní defaults path do jiného child cwd: startup
   selhal. Output/defaults cesty jsou nyní absolutní; nic neletělo.
2. Disarmed experiment proběhl, ale PARM.Default=NaN blokoval JSON export.
   Neplatné metadata jsou zachované jako explicitní text; řídicí NaN jsou
   stále odmítané. Raw BIN se nezměnil, první B1 byl offline znovu ověřen.
3. Standard arming při 30s bootu odmítl Need Position Estimate. Runner nyní
   čeká i na skutečný EKF position/predicted-position stav. Žádný force retry.
4. První roll pokus chybně změnil AP gains již pro GUIDED přípravu; začátek
   měl velké gyro a UTB saturaci po čtyřech výpočtech. Nenazývá se validním
   měřením. AP příprava ponechává původní gains a kontroluje klidový stav;
   host fault guard je aktivní ihned po entry, úklid má standardní LAND.
5. Nominální UTB gains v klidném zero testu způsobily R/P oscilace a SATURATION
   po 1,052079 s. Doložena je nevhodnost této konfigurace pro skutečný SITL
   sensor/pohon řetězec; jednotlivý fyzikální původ oscilace není identifikován.
   Jediné half-scale P/I/D řešení prošlo následnou úplnou sadou.
6. Inertia model s absolutní cestou se kvůli AP filesystem mapping nenačetl.
   Samostatný opakovaný test používá `../inertia-model.json`; loader i nové J
   jsou doložené v konzoli a zachovaném JSON. Failed startup není letový test.
7. Dvě současně spuštěné legacy regrese kolidovaly na pomocném TCP 5762.
   Nešlo o regresi řízení; fáze 0 byla znovu úspěšně spuštěná sériově.
   Tyto runner procesy se spouštějí po jednom. Žádný soubor se nevymazal.
8. Původní průběžné metrics.json měly step metrics i pro sinus; finální analýza
   je oddělená, step metrics počítá jen pro konstantní plateau a kontroluje
   všech 3200 sad, časové zarovnání, call counts a motor provenance.
   První zero-half report měl hardcoded původní gains; skutečné PARM i nynější
   validační report používají správné half-scale hodnoty. Původní evidence
   zůstává zachovaná, finální autoritou je analysis-final/analysis.json.

## Reprodukce

Příkazy se spouštějí z `/home/jirka/ardupilot_utb`, bez sudo. Stávající
ARM toolchain `/opt/gcc-arm-none-eabi-10-2020-q4-major/bin` musí být v PATH
pro cross-build. Každou variantu ihned zkopírovat do vlastního adresáře,
protože další configure/build mění obsah `build/`.

```sh
.venv/bin/python ./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf --targets bin/arducopter,tests/test_utb_sitl_rate,tests/test_utb,tests/test_utb_math -j6
.venv/bin/python ./waf configure --board sitl --enable-UTB --disable-UTB_RATE_SITL --enable-UTB_BENCH
.venv/bin/python ./waf copter -j6
.venv/bin/python ./waf configure --board sitl --disable-UTB --disable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf copter -j6
.venv/bin/python ./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
.venv/bin/python ./waf copter -j6
.venv/bin/python ./waf configure --board SkystarsH7HD-bdshot --enable-UTB --disable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf copter -j6
.venv/bin/python ./waf configure --board SkystarsH7HD-bdshot --disable-UTB --disable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf copter -j6
```

Aktuální aktivní binárka je
`tmp/utb-2a-sitl-20261013_164236/final-sitl-active/arducopter`.
Pro nový běh zvolit **nový, dosud neexistující output adresář**:

```sh
.venv/bin/python -B Tools/autotest/test_utb_active_sitl.py --binary tmp/utb-2a-sitl-20261013_164236/final-sitl-active/arducopter --output tmp/utb-2a-repeat-basic --suite basic --speedup 3
.venv/bin/python -B Tools/autotest/test_utb_active_sitl.py --binary tmp/utb-2a-sitl-20261013_164236/final-sitl-active/arducopter --output tmp/utb-2a-repeat-guards --suite guards --speedup 3
.venv/bin/python -B Tools/autotest/test_utb_active_sitl.py --binary tmp/utb-2a-sitl-20261013_164236/final-sitl-active/arducopter --output tmp/utb-2a-repeat-responses --suite responses --speedup 3
.venv/bin/python -B Tools/autotest/utb_active_sitl_analysis.py --inputs tmp/utb-2a-sitl-20261013_164236/responses tmp/utb-2a-sitl-20261013_164236/perturbations --output tmp/utb-2a-repeat-analysis
```

`--plots` navíc vyžaduje existující ReportLab a Pillow; aktuální grafy byly
vytvořené stávajícím Windows Python runtime, nic se nedoinstalovalo.
Legacy phase-0 wrapper contract je archivovaný spolu s regresí.
Přesné build argv a compiler flags jsou v commands/build-command JSON a
finálních waf-cache.py. Replay příkaz a vstupní SHA jsou uložené v evidenci.

## H743 APJ artefakty: POUZE CROSS-BUILD, NEUPLOADOVAT V TÉTO FÁZI

| Adresář pod session root | APJ velikost [B] | SHA-256 |
|---|---:|---|
| final-h743-isolation (UTB/BENCH ON, SITL flag requested ON, actual active absent) | 1 129 203 | `8c925227885595c66b0acca5f50729a88643f1bb863d19a33d2d63a31b0142aa` |
| h743-utb-on-bench-off | 1 123 579 | `b371ed8a6fdbaa988fee725fcbfd8b27942262ad3fb14b6ae7a5dd45f3f35be9` |
| h743-utb-off-bench-off | 1 113 283 | `b7a89f1923b20974cabfacb4377953d250a93146384d610e2ac10ff7df95255f` |

Všechny mají board_id=1075. APJ souborová velikost není decompressed image
velikost; manifest obsahuje obě. BENCH OFF/UTB ON image má navíc 4 B paddingu
oproti text+data. Žádný z nich nebyl nahrán do skutečné FC.

## Hardware a další postup

COM5 IDLE/LIST: **NOT EXECUTED – AWAITING PHYSICAL APPROVAL**.
Existující idle.json/list.json byly nalezené. Nebyl otevřen port, vytvořen
approval, poslán LIST/DATA, erase, ARM, motor test, reboot, PARAM_SET ani upload.
Starší fyzická potvrzení nejsou aktuální approval tohoto experimentu.
Poslední známý FC firmware `f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f`
nebyl v této fázi nově přečten. Stav na hardware se tímto reportem nepotvrzuje.

Zůstává: samostatné SITL ladění Yaw; objasnění citlivosti R/P D na plný
SITL řetězec; repeatability/statistika; plně doložený AP allocation
feedback/anti-windup pro saturaci; širší throttle/spool a delay envelope;
attitude/altitude/position nadřazená regulace; úplné upstream regrese.
Aktivní hardware authority není dalším automatickým krokem a není povolená.
Návrh nebo další fáze vyžaduje samostatné odsouhlasení. Všechny původní
1H-D/1H-E untracked soubory i archivní hardware BIN byly zachované.
