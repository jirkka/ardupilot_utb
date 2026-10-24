# FÁZE 3E-3 – Analýza časování Attitude SHADOW

Datum: 2026-10-24. Repo `/home/jirka/ardupilot_utb`, větev `test/skystars-5inch`.

## 1. Co lze a nelze rozhodnout offline

3E-2 fyzicky ověřila 1000 primary + 1000 yaw resetů, max/RMSE 0, motor_changes=0. Její primary Attitude interval 1623–3318 µs a průměrný absolutní jitter 207,142 µs **sám nerozlišuje** nepravidelný začátek hlavní smyčky od proměnlivého času před Attitude. Nebyly uloženy společné main/rate/att timestampy ani časové polohy jednotlivých intervalů. Předchozí Rate měření byla v jiné boot session, na jiném měřicím místě. Příčina HW odchylky zatím není prokázána; v této fázi není povolena komunikace s FC.

Zdrojové pořadí skutečné větve:

1. `AP_Scheduler::loop()` ([zdroj](../libraries/AP_Scheduler/AP_Scheduler.cpp)): `INS.wait_for_sample()`, opětovné získání původního `_rsem`, `_loop_sample_time_us=micros64()`. Toto je použitý **INS-ready/main start**, nikoliv vstup do metody před čekáním na IMU. Následuje `tick()` a `run()`.
2. `Copter::scheduler_tasks` ([zdroj](../ArduCopter/Copter.cpp#L113)): INS update → `run_rate_controller_main` → volitelný custom controller → motor output → historický `utb_shadow_update` → `read_AHRS` → `read_inertia` → `check_ekf_reset` → `update_flight_mode`.
3. `run_rate_controller_main` připravuje dt a kontext; skutečný `AC_AttitudeControl_Multi::rate_controller_run()` převezme gyro a zavolá `rate_controller_run_dt()`. Původní Rate SHADOW timestamp vzniká až uvnitř dt varianty při `_rate_gyro_time_us`, po gain/mix aktualizaci. Nové T_rate je na vstupu do `rate_controller_run`, před gyro čtením. Místa nejsou zaměňována.
4. `read_AHRS` volá `ahrs.update(true)` bez druhého INS update. `AP_AHRS::update` používá původní `_rsem` a `hal.scheduler->boost_end()` ([zdroj](../libraries/AP_AHRS/AP_AHRS.cpp#L454)); tím může skončit dočasné zvýšení priority z INS. Následují DCM/EKF a aktualizace odhadů. Nová diagnostika tyto mutexy ani priority nemění.
5. `read_inertia` aktualizuje position estimates, polohu a větví se podle dostupnosti odhadů ([zdroj](../ArduCopter/inertia.cpp)). `check_ekf_reset` podmíněně volá inertial reset při změně yaw nebo primárního EKF ([zdroj](../ArduCopter/ekf_check.cpp#L248)). To nejsou události každého cyklu.
6. `update_flight_mode` nastavuje reset policy a volá native `flightmode->run()`. DISARMED STABILIZE zpracuje pilotní vstupy a spool policy, provede `reset_yaw_target_and_rate()` a reset Rate I, poté primary Attitude příkaz ([zdroj](../ArduCopter/mode_stabilize.cpp#L9)). Při UAS_RUN=1 je před primary navíc skutečné UTB porovnání yaw resetu s importem historie. Právě tato operace vytváří druhou validní operaci v jednom main cyklu.

FAST_TASK se spouštějí každý scheduler tick, nemají vlastní nižší nominální Hz a nevynechávají se kvůli rozpočtu běžných úloh. `AP_Scheduler::run` prochází sloučené priority; v tomto místě jsou uvedené fast tasky před běžnými rc/GPS/logging úlohami. Běžné úlohy na konci předchozího cyklu a přerušení mohou ovlivnit příští začátek; nejsou však automaticky další 10Hz úlohou vloženou mezi Rate a Attitude.

AHRS wrapper běží každý fast tick, **výpočet uvnitř nemá konstantní práci**. `NavEKF3_core::UpdateFilter` podmiňuje prediction/fusion přes `runUpdates`; IMU downsampling, dostupnost nových senzorových dat, volba fúze a reset události vytvářejí různé větve ([zdroj](../libraries/AP_NavEKF3/AP_NavEKF3_core.cpp#L625)). Četnost vnitřních fúzí nelze ztotožnit s 400Hz voláním wrapperu ani bez konfigurace označit jednou pevnou frekvencí. ChibiOS timer/IO/SPI a USB aktivity mohou také preemptovat main. To jsou doložené možnosti ve zdroji, **nikoliv důkaz příčiny konkrétních 3E-2 extrémů**.

Konkrétní nižší časová škála je v této větvi přímo definovaná: `EKF_TARGET_DT=0.012f`, tedy cílově přibližně 83,3 Hz ([core.h](../libraries/AP_NavEKF3/AP_NavEKF3_core.h#L58)). `readIMUData` spouští `runUpdates` až po akumulaci přibližně 12 ms a povolení frontend prediction, případně při dvojnásobku targetu ([Measurements](../libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp#L483)). Při ideálních 2,5ms IMU krocích to může znamenat prediction přibližně každý pátý main cyklus; není to potvrzená skutečná četnost konkrétního H743 záznamu. Nelze tvrdit, že AHRS má každé volání stejný 400Hz výpočet nebo že zde musí jít o 10Hz úlohu.

Sloučený common task list obsahuje také volitelný FAST gyro-FFT sampling ([AP_Vehicle.cpp](../libraries/AP_Vehicle/AP_Vehicle.cpp#L620)). Všechny uvedené FAST_TASK mají prioritu 0; při shodě `run()` dává přednost vehicle listu, proto tento common FAST sampling přichází až po celé vehicle FAST sekvenci, ne mezi zdejší Rate a Attitude. Běžné tasky mají vlastní požadované Hz a integer tick kvantování (např. nominální rc_loop 250 Hz při 400Hz scheduleru má interval `floor(400/250)=1` tick); příjem nových RC dat je další samostatná událost. Je nutné rozlišovat nominální scheduler frekvenci a skutečnou práci při novém senzorovém/RC vzorku.
## 2. Přesné nové měřicí body a párování

Compile-time `AP_UTB_TIMING_ENABLED`, build option `UTB_TIMING`, default **0**. Samostatné runtime `UTT_RUN=0`, `UTT_COUNT=4000` (1–20000). `UTT_RUN=1` zachytí právě jednu finite session za boot; nespouští UAS ani UNS. Root AP_Param klíč **264**, dosavadní klíče beze změny. FORMAT_VERSION zůstává prvním skalárním záznamem tabulky.

- **T_main:** převzatý uint32 vzorek již existujícího `_loop_sample_time_us`, po návratu z INS wait a získání původního scheduler semaphore; žádné nové čekání. Callback přiřadí vlastní uint32 session cycle ID. Jedno ID odpovídá jednomu skutečnému průchodu scheduler loop; není to AP task index, IMU timestamp ani interní 16bit tick_counter.
- **T_rate:** `micros()` na vstupu do native multirotor `rate_controller_run()`, po diagnostickém enabled/main-thread guardu a před gyro čtením. Původní dt varianta ani PID se nespouštějí podruhé.
- **T_att:** `micros()` na vstupu do jedné ze tří podporovaných primary funkcí (STABILIZE / normal ACRO / RATE_ONLY), po enabled/main-thread guardu, před UAS pre-snapshotem. Staré 3E-2 T_att bylo až v `UTBAttitudeCapture::before`, po kopii vstupů; nová definice je záměrně stejná při UAS OFF/ON a tento posun se přiznává.
- Mode, požadovaný UAS_RUN a supported execution context se přidají v `run_rate_controller_main` před native Rate. UAS_RUN je **požadovaná hodnota z kontextu tohoto cyklu**, nikoliv odvozená značka z pozdějšího exportu. Například poslední one-step cyklus má kontext RUN=1 i když se UAS během `finish()` automaticky přepne na 0.
- Reset yaw se zaznamená jen jako event counter v daném cyklu, není to T_att ani další cyklus. Primary kind, počty rate/att calls a supported/missing_finish flag jsou součástí uložených kontextových řádků.
- `update_flight_mode` po původním mode run a UAS finish uzavře Timing record. Nový main při neuzavřeném předchozím recordu jej označí missing_finish; čas z jiného cyklu se nikdy nedoplní. Duplicitní volání se počítají explicitně, jejich timestamp se nepoužije jako validní párová perioda.

Zvlášť se agreguje:

| Metric ID | Výraz | Reference pro absolute_error/count |
|---|---|---|
| 0 | T_main[n] − T_main[n−1] | 2500 µs |
| 1 | T_rate[n] − T_rate[n−1] | 2500 µs |
| 2 | T_att[n] − T_att[n−1] | 2500 µs |
| 3 | T_att[n] − T_rate[n] | 0; jde o průměrné zpoždění, nikoliv period jitter |
| 4 | delay[n] − delay[n−1] | 0; **signed**, absolute_error dává průměrnou absolutní změnu |

Pro správně spárované sousední záznamy platí identita **ΔT_att = ΔT_rate + Δ(delay)**. Pravidelný main/rate a proměnlivé delay tedy mohou vysvětlit Attitude jitter bez změny main startů. Jestli platí právě na H743, musí ukázat teprve nový schválený HW experiment.

Skupina 0 zahrnuje všechny dostupné intervaly, 1 UAS=0, 2 UAS=1, 3 ostatní/absent UAS. Hraniční intervaly mezi phase/mode jsou v celkových statistikách zachované, ale nezahrnují se do čisté OFF/ON dvojice; phase_changes je ukazuje. Zpoždění metric 3 má jeden vzorek na každý úplný cyklus i u hranic. Missing/duplicate/unsupported/invalid_time a cycle gaps se neskrývají. Timestamp 0 je platný; unsigned rozdíly obsluhují 32bit wrap. Rozdíl nad INT32_MAX je explicitně invalidní, ne záporná perioda.

Každý Metric ukládá count, signed min/max/sum, součet absolutních odchylek, cycle ID a T_main obou extrémů. Průměr=sum/count, MAJ=absolute_error/count se dopočítává mimo fast path; při count=0 nejsou sentinel min/max výsledkem měření. Diagnostické čítače `over_nominal` (>2500 µs) a `over_two_periods` (≥5000 µs) jsou strukturální reference jedné/dvou nominálních period, **nikoli schválené letové nebo stop limity**. U signed delta počítají kladná překročení; záporné extrémy zůstávají v min a absolute_error.

Pevný ring uchovává 8 předchozích recordů. Při novém nejhorším |Att period−2500| uchová až 8 předchozích, aktuální a nejvýše 8 následujících recordů: **max 17 řádků**, bez alokace. U začátku/konce session může být okolí kratší. Shodně velké extrémy ponechají první výskyt. Reset count, mode/UAS a presence fields umožňují kontrolovat kontext, nikoli připsat čas konkrétnímu AHRS/SPI podkroku.

## 3. Izolace a režie

Nová engine/wrapper používá pevné RAM struktury, integer akumulaci, omezené konstantní průchody a main-thread writer. Nevytváří vlákno, mutex, queue, heap ani průběžný logger/MAVLink/text export. Snapshot se publikuje po stopu přes lock-free atomic bool release/acquire; potom se již nepřepisuje. Čtení `@SYS/utb_timing.txt` je pouze po stopu a DISARMED; za běhu vrací pending. Další session potřebuje nový boot, automatické opakování není podporované.

Timing se smí spustit DISARMED; případné armování nebo runtime cancel ukončí pouze diagnostiku. Absence rate threadu, 400Hz loop a native STABILIZE/ACRO určují supported kontext. Nepodporovaný kontext se vykáže; parametry AP se nepřestavují. Runtime OFF přidává pouze guard volání, compile OFF odstraní nové hooks/objekt/kód/parametry/export.

`hook_sum_us`, `hook_calls`, `hook_max_us` měří vlastní RAM/timing práci aktivních callbacků včetně engine agregace a případné kopie worst-context. **Neobsahují čas původních AP regulátorů**, AHRS, motor output ani UAS math. Obsahují účinek případné preempce uvnitř bracketů. Guard a část timer/accounting/publish instrukcí jsou mimo bracket, takže součet není kompletní WCET ani přesný celkový overhead. Runtime-OFF guard cost v těchto čítačích není zachycen. Compile ON/OFF a runtime OFF/ON proto zůstávají oddělené experimentální varianty.

Na SITL `AP_HAL::micros()` vyjadřuje virtuální čas; nulový hook interval **neznamená nulovou CPU práci**. Host wall time celé session zahrnuje simulátor, MAVLink a čekání a také není měřením H743 režie.

## 4. Ověření této fáze

| Kontrola | Skutečně provedený výsledek |
|---|---|
| Nové C++ timing unit testy | **14 PASS**: main vs. stage jitter, zero timestamp, clock/cycle wrap, missing/duplicate, nedokončený cyklus, phase hranice, signed delay delta, mezní čítače, pozice extrémů, 17-record context, invalidní čas, unsupported/mode změna a restart enginu. |
| Stávající C++ regrese | **30 Attitude + 23 Rate SHADOW + 47 UTB + 3 math PASS**; **1 volitelný math OfflineVectors SKIP**. Celkem **117 PASS / 1 SKIP**. |
| SITL timing runtime ON | **4000/4000 spárovaných cyklů**, missing/duplicate/invalid_time/cycle gaps/unsupported 0. Dvě phase změny, 3000 OFF + 1000 one-step cyklů. |
| SITL timing runtime OFF | UTT_RUN=0; timing export zůstává **pending**. Samostatný UAS one-step stále vrací 2000/2000 valid, max/RMSE 0. |
| SITL one-step uvnitř timing ON | 1000 primary + 1000 yaw resetů, valid=2000, max/RMSE 0, motor_changes=0. |
| SITL stávající Attitude regrese | Native ACRO↔STABILIZE, **8000 main / 16000 valid operací**, bez invalid/desync/unsupported a motor changes; max/RMSE 0. |
| SITL stávající Rate regrese | Native ACRO↔STABILIZE, **8000/8000 valid**, max/RMSE 0, motor changes 0. |
| SITL compile OFF | Sestavení i native Attitude regrese: **8000 main / 16000 valid**, max/RMSE 0. Timing symboly v binárce chybějí. |
| Motorová izolace | 2431 chráněných souborů HAL/AP_Motors/arming a ověřené UTB matematiky beze změny. Po odstranění pouze nových Timing guard bloků odpovídá původní kód scheduleru, native Rate/Attitude a Copter integrace předchozímu commitu. DISARMED RCOU množiny runtime OFF/ON shodné: C1–C4 = 1000. |
| Metadata | Parser úspěšný, UTT_RUN a UTT_COUNT mají nové oddělené záznamy. |
| Styl | Flake8 nového runneru, astyle nových C++ souborů, git diff --check úspěšné. |

Acceptance složky: `sitl-timing-on-verified`, `sitl-timing-runtime-off`, `sitl-attitude-regression`, `sitl-rate-regression`, `sitl-compile-off`. Žádný scénář nearmoval ani simulovaný Copter. Nespouštěl se runtime UAS_RUN=2. Jde o disarmed regrese; nové armed/failsafe/letové chování se tímto netvrdí. DISARMED RCOU shoda není důkaz shody všech ozbrojených motorových transientů nebo elektrického výstupu.

Ve virtuálním SITL okně byly periody main/rate/att shodné (2499–3332 µs), delay i změna delay 0. Nejhorší interval byl v cyklu 191 při UAS=0; uložených 17 řádků poskytuje jeho společné timestampy. Není to reprodukce HW příčiny: virtuální čas v průběhu CPU výpočtů nemusí postupovat. `hook_sum_us=0`, `hook_calls=24000` proto znamená **nezměřenou skutečnou CPU režii**, nikoli bezplatnou diagnostiku. Host wall časy acceptance běhů jsou uložené v session.json, bez přepočtu na H743 náklady.

Při vývoji se objevily a byly před acceptance opraveny pouze problémy nové instrumentace: chybějící MAX dependency nahrazená prostým porovnáním, chybějící standardní HAL definice nového unit testu a nevhodné počáteční umístění UTT skupiny před FORMAT_VERSION. `AP_Param::check_var_info` nepovoluje skupinu jako první položku; nové UTT bylo přesunuto za původní první skalár, bez posunu klíčů. Původní neúspěšné build/test logy zůstávají v artefaktech a nejsou zahrnuté do PASS počtů. Nedošlo k úpravě native parametrického validátoru, starých testů ani původní řídicí matematiky.

## 5. Buildy, paměť a skutečný rozsah měření

Všechny čtyři varianty **SITL_ON, SITL_OFF, H743_ON, H743_OFF prošly**. ON/OFF zde znamená nový Timing compile guard; původní Rate i Attitude SHADOW jsou dostupné v obou. Aktivní UTB řízení, 50Hz bench a logger trace jsou při těchto buildech vypnuté. Runtime přepínače začínají na 0.

| H743 sekce | Timing OFF [B] | Timing ON [B] | Rozdíl [B] |
|---|---:|---:|---:|
| .text | 1261500 | 1265280 | +3780 |
| .data | 4172 | 4172 | 0 |
| .bss | 124840 | 126728 | +1888 |
| main stack .pstack | 7168 | 7168 | 0 |
| ISR .mstack | 1536 | 1536 | 0 |

Zarovnání dalších sekcí znamená, že celkový flash přírůstek z build summary je 3776 B. Staticky rezervovaný heap se odpovídajícím způsobem zmenší o 1888 B; nejde o heap alokaci v měřené cestě. Compile OFF neobsahuje AP_UTB_Timing funkce; H743 ON/OFF neobsahují aktivní UTB_Core/UTB_Output/mode42/43 symboly. Matematické moduly pasivního SHADOW zůstávají přítomné záměrně.

`-fstack-usage` pro H743 uvádí jednorázový `Engine::start` **1176 B** (inicializace statistik), pravidelný `Engine::finish` **72 B**, `save_context` 20 B, wrapper loop 24 B, wrapper finish 16 B, export 88 B. Jsou to **statické vlastní framy funkcí**, nikoli změřený transitive stack peak. Jednorázová inicializace nastává v scheduler části před native úlohami. Nesmí se z toho odvozovat bezpečný watermark pro všechny AP call-chain kombinace. Nový runtime stack watermark ani MCU časy v této fázi měřené nejsou; předchozích 4344 B volných z 3E-2 nelze vydávat za nový výsledek.

Kandidát pouze pro nové samostatné schválení:

- Target SkystarsH7HD-bdshot, APJ board_id **1075**.
- `tmp/utb-3e3-20261024_092051/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`
- **1 146 983 B**, SHA-256 **`f75531d90e21ea9ff199197599ba5764cd7127d28ee2ea714ad1a3f945e71672`**.
- OFF APJ: **1 143 607 B**, SHA-256 `6cea81fc0b34066e55b5efc50f68186c152e8263166822c8b9d4676a149cc131`.

Přesné command arrays a návratové kódy jsou v `*-results.json`, sekce/symboly/stack v `isolation-footprint.json`. Reprodukce ON:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
export CXXFLAGS=-fstack-usage
./waf configure --board SkystarsH7HD-bdshot \
  --out=tmp/utb-3e3-20261024_092051/H743_ON \
  --enable-UTB --disable-UTB_RATE_SITL --disable-UTB_NATIVE_BENCH \
  --disable-UTB_LOGGER_TRACE --enable-UTB_NATIVE_SHADOW \
  --enable-UTB_ATTITUDE_SHADOW --enable-UTB_TIMING
./waf copter -j6
```

Pro OFF zvolit samostatný out `H743_OFF` a `--disable-UTB_TIMING`. SITL používá `--board sitl`, samostatný out a stejné feature volby; CXXFLAGS stack-usage byl použit pro H743. Unit targety a skutečné SITL command lines jsou zaznamenané v artefaktech. Známá nezávislá věc AC_CustomControl při compile-time logging-disabled nebyla v této fázi opravována ani použita jako domnělá nová regrese.

## 6. Manifest nové fáze

12 upravených + 5 přidaných souborů; z toho jeden tento dokument. Knihovny Rate/PID/Attitude matematiky, AP_Motors, HAL, arming, failsafe, priority a task table se nepřepisují.

- `ArduCopter/Attitude.cpp`
- `ArduCopter/Copter.h`
- `ArduCopter/Parameters.cpp`
- `ArduCopter/Parameters.h`
- `ArduCopter/mode.cpp`
- `ArduCopter/system.cpp`
- `Tools/scripts/build_options.py`
- `Tools/utb/tools/test_utb_timing_sitl.py`
- `docs/UTB_PHASE_3E3_ATTITUDE_TIMING_CS.md`
- `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`
- `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp`
- `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`
- `libraries/AP_Scheduler/AP_Scheduler.cpp`
- `libraries/AP_UTB/AP_UTB_AttitudeShadow.h`
- `libraries/AP_UTB/AP_UTB_Timing.cpp`
- `libraries/AP_UTB/AP_UTB_Timing.h`
- `libraries/AP_UTB/tests/test_utb_timing.cpp`

`AP_UTB_Timing.*` obsahují engine a finite wrapper; nový unit/runner zajišťují deterministické i live SITL ověření. Copter přidává pouze kontext, init, parametrický objekt a finish. Native controller/scheduler přidávají pouze guardované timestamp/event hooks. AP_Filesystem přidává post-window snapshot a build_options default-OFF volbu. AttitudeShadow.h má jen read-only getter požadovaného runtime stavu. Žádná produkční Python hardware automatizace nepřibyla.

## 7. Návrh jediného následného USB-only DISARMED H743 experimentu

**Návrh, dosud neprovedeno a nenahráno.** Před uploadem znovu ověřit aktuální identitu COM5 / board1075 / serial200024001251303433333336, zdravý start, DISARMED, zálohu parametrů a nový konkrétní souhlas s výše uvedeným SHA. Fyzicky potvrdit USB-only, bez LiPo a vrtulí; žádné motorové příkazy. Bootloader/parametry/logy zachovat. Pokud je potřeba fyzický start, provede jej člověk; žádný další automatický reboot.

1. Native STABILIZE, FSTRATE_ENABLE=0, SCHED_LOOP_RATE=400, nepřítomný rate thread, UAS_RUN=UNS_RUN=UTT_RUN=0. Logger ponechat v původním stavu, zejména jej neotevírat pro tento RAM experiment. Původní failsafe a řízení nepřestavovat kvůli výsledku.
2. Uložit a držet stejné minimální telemetrické intervaly pro baseline/aktivní/po; přečíst threads před oknem, nikoliv tasks.txt. Přibližně 15 s v runtime OFF pro CPU/health baseline; nové intervalové metriky se při UTT_RUN=0 nečekají.
3. Při vypnutých sessions nastavit **UTT_COUNT=4000 a UAS_COUNT=1000**. UTT_COUNT je počet timing recordů, **není to kontinuální UAS 4000krokový test**.
4. Jednou spustit UTT_RUN=1. Přibližně po 3 s jednou spustit **UAS_RUN=1**. Zbylé přibližně 7 s neprovádět žádné další diagnostické požadavky, PARAM_SET, FTP, log listing/download ani změny loggeru; pouze předem definovaná stálá telemetrie/nezbytný skutečný heartbeat. Actual phase hranice se berou z uložených recordů a counters, nikoliv z přesnosti host sleep.
5. Po automatickém zastavení obou sessions ověřit DISARMED, UAS_RUN=UTT_RUN=0 a zdraví. Teprve poté přečíst `@SYS/utb_timing.txt`, `@SYS/utb_attitude_shadow.txt`, threads a dostupné nezatěžující statistiky. Obnovit dočasné parametry a intervaly.
6. Vyžadovat všechny 4000 timing cykly, úplné rate/att calls, explicitně vykázané phase boundaries, supported kontext, bez invalid/desync/mismatch/motor changes. Zvlášť vyhodnotit main/rate/att intervaly a delay delta ve skupinách UAS OFF/one-step a okolí nejhoršího Att intervalu. Jednotlivé timestampy v tomto okolí musí splnit párovací identitu. Stack watermark a hook wall times budou teprve skutečnými H743 výsledky.
7. Při InternalError, restartu, sensor/EKF závadě nebo neočekávané změně motorových hodnot diagnostiku zrušit, zachovat evidence a nepokračovat dalším pokusem. Závažný main-loop interval nebo neúplná data nesmí dostat PASS. Vybrané 2500/5000µs čítače jsou diagnostické značky, nikoliv univerzální kritérium letové bezpečnosti. Část časových anomálií bude pozorovatelná až po uzavření snapshotu; tento návrh neposkytuje nový živý watchdog ani motorový fail-stop.

Jestli budou main/rate pravidelné a Att změny odpovídat delay delta, bude lokalizováno proměnlivé zpoždění **uvnitř** cyklu. Jestli se rozkolísají už main/rate, půjde o jiný či kombinovaný problém. Ani jedna možnost sama nepřipisuje vinu AHRS, loggeru nebo konkrétní úloze; případná další jemnější sonda vyžaduje následné rozhodnutí.

## 8. Git a závěr

První lokální commit: **`95affbde7ee7137c5ed50354e8671b772ad537c7`**, `feat(utb): add validated rate and attitude shadow controllers`. Obsahuje explicitně ověřených **52 souborů**, 6106 vložených / 41 odebraných řádků stavu 3B–3E2, testy, logger trace a odpovídající dokumentaci. První staging manifest, diff a SHA-256 jsou v artefaktech. `kapitoly/historie.txt` zůstalo mimo commit a beze změny.

Druhý lokální commit nese `test(utb): add attitude scheduling timing diagnostics` a právě 17 výše vypsaných souborů této fáze. Jeho výsledné SHA je po vytvoření uložené v `tmp/utb-3e3-20261024_092051/commits.json` a v závěrečném reportu; vlastní hash této revize dokumentu nemůže být součástí jejího Git obsahu. Finální status, kompletní seznamy obou commitů, kontrola indexu a SHA-256 evidence jsou uložené vedle něj. Bez push, reset, clean, stash nebo přepisu historie; tmp, binární firmware a kapitoly nejsou stageované.

**Diagnostický kandidát prošel software/build kontrolami a je připraven k samostatnému schválení krátkého DISARMED uploadu a měření.** Není tím vysvětlena nebo odstraněna původní HW časová anomálie. Nová H743 režie, stack watermark a skutečné společné main/rate/att časování zůstávají **hardware neověřené**. Kontinuální Attitude SHADOW ani let se zatím nedoporučují. V této fázi nebyl otevřen COM5, nic se neflashovalo ani instalovalo.
