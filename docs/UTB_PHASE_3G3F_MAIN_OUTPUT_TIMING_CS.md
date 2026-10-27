# FÁZE 3G-3F – Main execution a native motor-output timing

Stav 2026-10-25: **implementováno, unit/SITL a offline build ověřeno; nový firmware na H743 neověřen**. Žádné připojení k fyzické FC, upload, změna jejích parametrů ani fyzický experiment neproběhly. Nic nebylo doinstalováno.

## 1. Výchozí stav a rozsah

Repo `/home/jirka/ardupilot_utb`, větev `test/skystars-5inch`, HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`. Výchozí tracked strom a index byly čisté. Existující untracked reporty 3G-3/A/B/C/D/E-A/E-B a `kapitoly/historie.txt` zůstaly zachované.

Artefakty této fáze: `tmp/utb-3g3f-20261025_152000/` (dále **P**). `checkpoint.json` obsahuje SHA-256 všech výchozích tracked souborů a chráněných untracked souborů, `untracked-before.tar` jejich archiv. Finální kontrola je v `integrity-final.json`; přesný seznam zdrojů, SHA-256 a Git blob ID v `source-manifest.json`. Git diff samotný nové soubory nepokrývá, proto je doplněn archivem nových souborů. Žádný commit, změna historie nebo push.

Původní matematika UTB_PID/Rate/Attitude, AP regulátory, AP_Motors, HAL/DShot, failsafe a arming se neměnily. Hodnotová izolace a časový vliv jsou dvě různé otázky: pasivní výpočet může posunout okamžik předání původních hodnot.

## 2. Volba architektury před implementací

Zvolena **A – rozšíření existující Timing**. Původní návrhový zápis před změnami C++ je uchován v `P/design-before-implementation.md`.

Existující `AP_UTB_Timing` již vlastní konečnou DISARMED session, parametry UTT_RUN/COUNT, pevné agregace a release/acquire publikaci. Varianta B by zdvojila lifecycle, parametry a publikaci. Nová pomocná hodnotová struktura `AP_UTB_ExecutionTiming` nemá vlastní scheduler, thread, AP parametry ani výstupní rozhraní; je členem Timing.

Nový přepínač `AP_UTB_TIMING_EXEC_ENABLED` má default **0**, waf option `UTB_TIMING_EXEC`, závislosti UTB a UTB_TIMING. Při vypnutí se nová struktura, hooky, virtuální soubor i export odstraní při kompilaci. Runtime zůstává `UTT_RUN=0`. Žádné nové AP_GROUPINFO indexy.

`@SYS/utb_timing.txt` zachovává schéma **UTBTimingV1** a význam původních metrik. Nový soubor `@SYS/utb_execution.txt` má samostatné schéma **UTBExecutionV1**. Při EXEC ON se zveřejnění společné session odloží do konce scheduleru; původní agregace se nadále uzavírají na původním `update_flight_mode` místě. EXEC OFF zachovává původní freeze.

## 3. Skutečný průchod Main a motorový call graph

`AP_Scheduler::loop()` (`libraries/AP_Scheduler/AP_Scheduler.cpp:367`) nejprve uvolní `_rsem`, čeká v INS `wait_for_sample()`, znovu získá `_rsem.take_blocking()` a teprve pak odebere `_loop_sample_time_us=AP_HAL::micros64()`. Jeho spodních 32 bitů je původní Timing Main start. **Čekání na IMU a opětovné získání tohoto semaforu jsou před měřeným začátkem.**

Copter tabulka FAST_TASK v `ArduCopter/Copter.cpp` určuje pořadí:

```text
INS update
→ run_rate_controller_main
  → původní AC_AttitudeControl_Multi::rate_controller_run
  → případný pasivní Chain/Flight Rate consume
→ případný custom controller
→ motors_output_main → motors_output()
→ původní diagnostický shadow hook, pokud povolen
→ read_AHRS / EKF → inertia → check EKF yaw reset
→ update_flight_mode
  → nativní reset / primary Attitude, případný pasivní Attitude
  → Chain/Flight finish → původní Timing finish
→ další fast tasks
→ běžné due tasks podle sloučené vehicle/common tabulky
→ scheduler účetnictví, perf, případný SITL simstate
→ nový scheduler-boundary probe → jeho bounded finalizace → návrat
```

Po návratu scheduleru pokračuje `AP_Vehicle::loop()` (`libraries/AP_Vehicle/AP_Vehicle.cpp:558`): aktualizace G_Dt, jednorázová bezpečnostní inicializace/banner a zpracování InternalError. Následuje práce HAL. **Scheduler boundary není konec veškeré Main/HAL práce.** Další INS wait nastává až při následujícím vstupu scheduleru.

Native PID předá roll/pitch/yaw a FF do původních setters AP_Motors již v `AC_AttitudeControl_Multi::rate_controller_run()`. `Copter::motors_output_main()` (`ArduCopter/motors.cpp:122`) později zavolá existující `motors_output()` pouze mimo rate-thread variantu.

Výstupní cesta:

```text
Copter::motors_output_main → Copter::motors_output(bool) [motors.cpp:62]
→ SRV calc_scaled/cork/output, interlock
→ flightmode->output_to_motors → AP_MotorsMulticopter / AP_MotorsMatrix
→ AP_MotorsMatrix::output_to_motors [AP_MotorsMatrix.cpp:143]
→ AP_Motors::rc_write [AP_Motors_Class.cpp:99]
→ HAL RCOutput::write → SRV push → RCOutput::push [RCOutput.cpp:1372]
→ push_local / trigger_groups [RCOutput.cpp:761,1403]
```

ChibiOS `trigger_groups()` používá stávající `chMtxTryLock(&trigger_mutex)` a signalizuje EVT_PWM_SEND. Samostatné rcout vlákno následně provádí `dshot_send_groups()` (`RCOutput.cpp:1493`), DMA/IRQ dokončení nastává jinde. PID dokončení, mixer dokončení, HAL volání/návrat, elektrické zahájení/dokončení a přijetí ESC nejsou totožné události. Nové sondy HAL/DMA neinstrumentují.

Všechny zde podporované sondy běží v Main. ChibiOS priority jsou doloženy v `AP_HAL_ChibiOS/Scheduler.h` a boost implementaci `Scheduler.cpp`: Main base180, boost182, timer/rcout181, monitor183. Preempce podle aktuálního boostu i IRQ zůstává možná. Měřené spany obsahují skutečná případná čekání a preempce původní cesty; neposkytují rozklad CPU execution versus preemption.

| Oblast | Původní synchronizace/IO a význam |
|---|---|
| Před Main | INS wait a blokující reacquire scheduler semaforu; mimo M1–M5 |
| Rate / UTB consume | Původní nativní výpočet a hodnotový pasivní výpočet; nová sonda nepřidává čekání/IO |
| Motor call | Zahrnuje celou původní output cestu, případné backendy a synchronizaci; žádný její obecný WCET nebyl prokázán |
| AHRS/mode/ostatní tasks | Mohou obsahovat stávající synchronizaci/logger/driver práci; M4 ji může zachytit, neurčí samotného viníka |
| Export | FTP a ExpandingString až mimo okno, frozen a DISARMED; není součást rychlých hooků |

## 4. Měřicí body a metriky

Přesná aktuální čísla řádků, volání a kontext jsou strojově v `P/measurement-points.json`.

| Bod | Skutečné místo |
|---|---|
| Main | `AP_Scheduler.cpp:383`, původní `AP_UTB_Timing::loop(sample_time_us)` |
| Rate | `AC_AttitudeControl_Multi.cpp:547`, původní Timing rate entry před gyro read |
| UTB consume begin/end | `AP_UTB_ChainShadow.cpp:288/292`, kolem existujícího `_engine.rate(_rate_frame, _sysid)` |
| Motor begin/return | `ArduCopter/motors.cpp:126/130`, bezprostředně kolem původního `motors_output()` |
| Primary Attitude | `AC_AttitudeControl.cpp:424`, původní `Timing::attitude(0)` |
| Mode finish | `ArduCopter/mode.cpp:544`, konec mode + Chain/Flight finish; nikoliv celý fast loop |
| Fast boundary | `AP_Scheduler.cpp:239`, první přechod ze seřazených fast úloh na regular priority |
| Scheduler boundary | `AP_Scheduler.cpp:464`, poslední native práce scheduler loop, před vlastní finalizací sondy |

Nové callbacky nejsou scheduler tasks a nepřesouvají původní volání. Rate/Attitude využívají již odebraný timestamp. Optional consume je přítomný jen když opravdu běží odpovídající Rate engine.

| Export metric ID | Název / odpověď na zadání |
|---:|---|
| 0 | M1 `main_to_motor_begin` |
| 1 | M2 `motor_call` |
| 2 | M3 `main_to_motor_return` |
| 3 | M4 `main_to_scheduler_boundary` |
| 4 | M5 `rate_to_motor_begin` |
| 5 | Doplňkový `utb_rate_consume` |
| 6 | `main_to_mode_finish` |
| 7 | `main_to_fast_boundary` |

**M6 zadání je budoucí A/B rozdíl fáze výstupu, není export metric ID5.** Budoucí analýza porovná mean, min/max, pozorovaná maxima a překročení M1–M5. `sum/count` dává u nezáporných spanů mean i mean absolute distance od nuly. Přesná průměrná absolutní odchylka od finálního průměru se z těchto agregací nedá rekonstruovat; neoznačuje se za změřenou. Původní Timing poskytuje absolutní odchylku period od 2500 µs.

Spany jsou vnořené: M3=M1+M2, M4 zahrnuje dřívější části. **Nesčítat překrývající se metriky.** M2 obsahuje i observer efekt begin callbacku a vstup do end callbacku; nejde o čistý odinstrumentovaný čas native funkce. M4 nezahrnuje vlastní close/freeze sondy ani epilog scheduleru. Její finalizace se měří odděleným hook bracketem; neoznačuje se za úplný Main execution duration.

## 5. Čas, identita a validita

Main je low32 z HAL micros64, ostatní body HAL micros ze stejného ChibiOS kernel času. Generated H743 `hwdef.h` má **CH_CFG_ST_RESOLUTION=16**, společný chconf frekvenci **1 MHz**. `AP_HAL_ChibiOS/system.cpp:350` proto používá `hrt_micros32()`, implementované v `hwdef/common/hrt.c:122`; micros64 je v témže souboru:103.

16bit timer je softwarově rozšiřován (`system_time_u32_us`, řádek46). 32bit API a 64bit API sdílí původ kernel clock; běžný 400Hz běh provádí dostatečně častá čtení. **HAL čtení zde používá existující krátké chSysLock/chSysUnlock kritické sekce**, žádný nový čekající mutex. Není korektní tvrdit nulové maskování IRQ. Čtení hodin má rozlišení 1 µs; jeho skutečná cena na H743 zatím změřena není. Extrémní neobsloužený 16bit wrap při dlouhém stall je omezení HAL časového základu; HAL se nemění.

Unsigned32 rozdíly podporují přetečení po přibližně 71,6 min, timestamp0 je platný. Obrácené intervaly nebo rozdíly přes INT32_MAX jsou invalid, nikoliv falešná nula. Celý capture nejvýše 20000 cyklů je hluboko pod půlrozsahem hodin při normálním běhu. Stop není wall-time watchdog při zastavené Main.

Každý event nese přesné **UTT local cycle ID**. Sleduje se presence mask, pořadí, missing, duplicate, invalid, gaps, unsupported. Povinné body jsou Rate, oba motor body, Attitude, mode/fast/scheduler end. Consume smí chybět celý; půl páru je chyba. Neplatný cyklus nepřidá validní nuly do statistik.

Flight source/consume/Loop ID jsou jiné než UTT local cycle. Flight i Timing začínají v různých bodech po samostatných příkazech: součty jejich cyklů se nesmějí mechanicky párovat podle sousedních řádků. Každý export má vlastní session; `session_us` je 64bit uptime začátku, **není globálně unikátní napříč bootem**. Host envelope ukládá firmware, boot counter před/po, identitu procesu v SITL a přesnou session. Host UTC se neodečítá od MCU časů.

## 6. Lifecycle, RAM a export

UTT_RUN default0, podporovaný Main 400Hz context bez rate threadu, STABILIZE/ACRO a původní DISARMED guard. COUNT 1–20000, jedna session za boot. Stop reason: 1 COUNT, 2 cancel/ARM, 4 neplatný start, 6 unsupported/mode change, 7 změna konfigurace/COUNT, 8 chybějící/duplicitní/časově vadný event nebo sekvence. Pokud scheduler-end chybí, fallback při příštím Main startu zabrání pokračování nad limit; úplný stall nelze léčit tímto counterem.

Pevné uint64 sumy a min/max s cycle/time extrémů, dva prahy: >2500 a >=5000 µs. Jde o klasifikaci pozorovaných spanů, nikoliv nové bezpečnostní limity. Žádné automatické shazování native tahu.

Raw kontext je pevný ring předchozích8 + nejvýše17 řádků kolem **největšího validního M3**, až8 před/8 po. Není celá historie ani všeobecná černá skříňka. Prázdný kontext při žádném validním vzorku je legitimní. Bounded kopírování nejvýše17 záznamů může změnit observer cost při novém extrému; zahrnuto do nutnosti HW měření.

Po dokončení Main provede release store do lock-free atomic_bool. Reader provede acquire load; všechny multiword statistiky včetně uint64 zůstanou po publikaci immutable. Není concurrent čtení rozpracovaných hodnot. Pokus o další RUN pouze vynuluje runtime parametr, frozen statistiky nemění. Export je odmítnut při ARM, pending není vydáván za frozen. Reader může alokovat ExpandingString až při FTP exportu mimo měřené okno. Žádné filesystem, textové formátování, DataFlash, MAVLink, alokace nebo nové blocking API v capture hookách.

Publikace je doložena source audit a reálným SITL opakovaným frozen exportem. Samostatný TSAN/concurrent reader stress na cílovém MCU nebyl proveden; jednotkové testy ověřují také uint64 sum nad 2^32, parser konzistenci a odmítnutí smíšené/neúplné session.

## 7. Přesný manifest změn a důvody

Celkem **7 upravených + 7 přidaných souborů** této fáze. Původní untracked dokumentace do tohoto počtu nepatří.

| Soubor | Zásah |
|---|---|
| `ArduCopter/motors.cpp` | Guarded timestampy kolem nezměněného native motors_output |
| `Tools/scripts/build_options.py` | Default-OFF option a závislosti |
| `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp` | Guarded frozen virtuální export |
| `libraries/AP_Scheduler/AP_Scheduler.cpp` | Fast boundary a poslední scheduler boundary |
| `libraries/AP_UTB/AP_UTB_ChainShadow.cpp` | Dva callbacky kolem existujícího pasivního Rate engine |
| `libraries/AP_UTB/AP_UTB_Timing.cpp` | Společný lifecycle, eventy, bounded finalizace, snapshot/export |
| `libraries/AP_UTB/AP_UTB_Timing.h` | Guarded struktura/API a dependency compile guard |
| **nový** `libraries/AP_UTB/AP_UTB_ExecutionTiming.h` | Pevné event/metric/context struktury |
| **nový** `libraries/AP_UTB/AP_UTB_ExecutionTiming.cpp` | Hodnotová validace a agregace |
| **nový** `libraries/AP_UTB/tests/test_utb_execution_timing.cpp` | 19 deterministických C++ testů |
| **nový** `Tools/utb/tools/utb_execution_timing_analysis.py` | Striktní offline parser + CSV/JSON |
| **nový** `Tools/utb/tools/test_utb_execution_timing_sitl.py` | Konečné izolované SITL scénáře |
| **nový** `Tools/utb/tests/test_execution_timing_analysis.py` | 18 parser testů |
| **nový** `docs/UTB_PHASE_3G3F_MAIN_OUTPUT_TIMING_CS.md` | Tento report |

## 8. Testy a skutečné výsledky

| Kontrola | Výsledek |
|---|---|
| C++ UTB suite | **173/173 PASS**, původních154 +19 nových |
| Python UTB suite | **205 PASS, 5 skip, 2 subtests PASS**, původních187 +18 nových; skip není PASS |
| Finální SITL matice | **36 scénářů PASS**: 13 nových +22 původních +1 native compile-OUT |
| flake8, astyle nových C++ souborů, git diff --check | PASS |
| H743 ON/OFF a SITL ON/OFF compile/link | PASS |
| Původní TimingV1 parser / mixed UAS regrese | PASS, formát ani význam původních polí se nezměnil |

Důkazy: `P/unit-cpp-final-results.json`, `P/units-final/`, `P/pytest-verified-results.xml`, `P/test-summary.json`, příslušné session.json a raw frozen soubory. Přesné příkazy jsou v build/test JSON a logách.

Nové C++ testy pokrývají pořadí/ID, missing motor/scheduler, duplicate motor, obrácené časy, nulové a stejné timestampy, timestamp i cycle wrap, neukončený cyklus, gap, unsupported, prázdný a přepsaný bounded kontext, restart engine, min/max/sum a 64bit sum. Parser odmítá chybějící konec, poškozený řádek/stránku, směs session, falešný COUNT stop, nekonzistentní sumy/counts/presence a půl consume páru.

Nových13 SITL případů: baseline, timing, joint, joint-dynamic, cancel, config, mode, arm, repeated, restart, interrupted, unsupported, invalid-count. Cancel/config/ARM/mode jsou očekávané předčasné konce s explicitním reason, nejsou úplné COUNT sessions.

**Joint dynamic DISARMED:** 4000/4000 valid execution sad, health missing/duplicate/invalid/gaps/unsupported0; 3997 consume párů uvnitř Timing okna. Flight vlastní okno: 4000 attempts, 3999 paired Rate +1 warmup, 8000 valid Attitude, shaping6136, 12000 motor checks, motor_changes0, všech84 metrik max/RMSE0. Rozdíl3997 versus3999 odpovídá časově posunutým hranicím samostatně spuštěných sessions; žádná chybějící polovina consume páru se nevydává za validní.

Dynamické RC R/P/Y mění obě znaménka; FF a D_FF jsou skutečně nenulové v existující SITL fixture (R/P FF0.02, Y FF0.01, R D_FF0.001). DISARMED nativní reset integrátoru zůstává, independent_i=0. **Současný armed Timing nebyl testován jako podporovaný: UTT musí při ARM skončit.** Nenulový nezávislý I a dynamický let ověřují samostatné původní armed Flight regrese. S3 je proto splněno tímto rozdělením, nikoliv nepravdivým tvrzením o společném armed capture.

Původní22 SITL regresí zahrnuje Rate/Attitude/Chain/Flight, FF, mode exit, RC failsafe, GCS failsafe, early stop, overflow, export interruption/conflict, unsupported, starý Timing a aktivní UTB SITL ANGLE/ACRO. LAND i RTL nativní řízení pokračují. GCS failsafe fixture používá výslovně **SITL FS_GCS_ENABLE=1**; fyzické parametry se neměnily. Compile-OUT native full-flight regrese rovněž PASS.

Restart test ověřil STAT_BOOTCNT1→2 v restartovaném vlastním SITL a nový pending snapshot. Přerušený FTP export je řízený drop odpovědí; host vrátí timeout, native pokračuje a frozen obsah se nezmění. První verze restart runneru selhala kvůli výchozímu `--wipe` pomocného runneru. Původní FAIL je zachován v `P/exec-restart`; opravena pouze host testovací infrastruktura, následné v2 i finální test PASS. Také první build zastavil chybějící MAX macro; nahrazen lokálním if a následné buildy PASS. Původní chyby nejsou smazané ani vydávané za PASS.

SITL virtuální čas se během CPU výpočtu typicky nepohybuje: M1/M2/M3/M5/consume zde vyšly0, v joint-dynamic M4=833 µs. To je virtuální čas scheduleru/simulace, **nikoliv fyzická režie**. Grafy `P/plots/` a `final-exec-joint-dynamic/flight-analysis/` jsou výslovně označené SITL/offline.

## 9. Buildy, firmware a paměť

Společné waf volby:

```sh
./waf configure --board SkystarsH7HD-bdshot \
  --out=tmp/utb-3g3f-20261025_152000/H743_ON \
  --enable-UTB --disable-UTB_NATIVE_BENCH --disable-UTB_LOGGER_TRACE \
  --enable-UTB_NATIVE_SHADOW --enable-UTB_ATTITUDE_SHADOW \
  --enable-UTB_CHAIN_SHADOW --enable-UTB_FLIGHT_SHADOW \
  --disable-UTB_RATE_SITL --enable-UTB_TIMING --enable-UTB_TIMING_EXEC
./waf copter -j6
```

H743_OFF: jiný out `H743_OFF`, pouze poslední volbu nahradit `--disable-UTB_TIMING_EXEC`. SITL_ON: board sitl, out SITL_ON a `--enable-UTB_RATE_SITL`. SITL_OFF: out SITL_OFF, RATE_SITL ON, `--disable-UTB_TIMING --disable-UTB_TIMING_EXEC`. Configure/build varianty provádět postupně, protože waf uchovává aktivní konfiguraci.

Původní 3G2 kandidát je **historický skutečně existující APJ/ELF**, nebyl v této fázi znovu sestavován. Jeho SHA a sekce byly znovu ověřeny; nové OFF/ON i oba SITL buildy byly skutečně sestaveny. Verze Copter4.7.1, board ID1075. Nové APJ uvádějí source git11a518e5, ale obsahují necommitnutý patch; úplná identita je HEAD + `source-manifest.json` a patch/archiv, nikoliv samotný APJ git string.

| Veličina [B] | Historická 3G2 reference | Nová EXEC OFF | Nová EXEC ON | ON−OFF |
|---|---:|---:|---:|---:|
| ELF .text | 1276148 | 1276148 | 1279576 | +3428 |
| ELF .data | 4172 | 4172 | 4172 | 0 |
| ELF .bss | 172536 | 172536 | 174248 | +1712 |
| Statické globální RAM (.data+.bss) | 176708 | 176708 | 178420 | +1712 |
| Linker .heap | 76728 | 76728 | 75016 | −1712 |
| APJ soubor | 1155815 | 1155815 | 1158367 | +2552 |

`.rodata` nemá samostatnou výstupní sekci, je sloučená do .text; není uváděna jako falešných0 B. Waf aggregate Text/Data/BSS zahrnuje i startup/constructors/stack reservations, proto se liší od literal ELF sekcí. Rezervované main stack7168 a ISR1536 B se nezměnily.

| Varianta | SHA-256 APJ |
|---|---|
| Historická reference | `918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7` |
| EXEC OFF | `edc3092cd4d21b97041bea7fdb55f89f9c764d01a51f253a8786ea47770fa2bb` |
| EXEC ON | `dae4f020db98a509b68c71a5b8702dfc7019ed762f69b7d46c8cbf3ae5e9d497` |

Jednoznačně pojmenovaný ON artefakt: `P/firmware/SkystarsH7HD-bdshot_UTB3G3F_EXEC_ON_dae4f020db98.apj`; OFF obdobně `EXEC_OFF_edc3092cd4d2.apj`. **Žádný nebyl nahrán.**

## 10. Source/symbol audit a observer effect

`source-isolation-audit.json`: po odebrání nových guarded bloků jsou motors.cpp, scheduler.cpp a Sys filesystem zdroje totožné s výchozím HEAD. Původní volání/order beze změny. Nový hodnotový engine nemá motorové API, alokace, IO nebo mutex. PID/mixer/HAL soubory bitově beze změny.

`native-machine-code-comparison.json`: proti historickému ELF jsou v novém OFF přesně shodné bajty motors_output_main, motors_output, native rate_controller_run, AP_MotorsMatrix output, RCOutput push a scheduler loop. Scheduler run se liší v **jediné instrukci line-number argumentu INTERNAL_ERROR:223→226**, doloženo `scheduler-run-machine-differences.json`; běžné řízení je nezměněno. Celý firmware není vydáván za bitově identický. Symbol i image-string audit: nová ExecutionTiming/utb_execution.txt pouze ON; aktivní UTB Core/Output/Acro/Angle na obou H743 absent, aktivní motorová větev zůstává compile-out.

Dodatečný RUN0 path obsahuje4 callbacky/cyklus (motor2, fast, scheduler), které skončí před odběrem hodin, plus bounded test hranice při průchodu tabulkou. Runtime overhead není nulový a H743 cena není změřena. Compile-OUT nemá nové callbacky/struktury.

Aktivní Timing bez consume přidává **9 micros čtení/cyklus**, s consume **13**, plus jedno micros64 při startu session. Rate/Attitude využívají existující čtení. Hooky navíc zahrnují bookkeeping, kopie raw kontextu a close. Export uvádí odděleně původní Timing hook sum/max/calls a nové execution hook sum/max/calls. Tyto brackety **nepokrývají všechen** observer cost: například inlined původní timestamp hooky sdílejí práci a MODE_END není samostatně bracketovaný. Nelze od nich získat úplný odinstrumentovaný overhead prostým součtem.

Offline x86 -O2 microbenchmark čistého RAM engine (bez HAL/AP/hook vstupů): pět dávek po milionu syntetických cyklů, batch mean **60.252 / 62.327 / 62.079 / 63.337 / 68.394 ns/cyklus**. Nejde o per-call maximum, MCU overhead ani WCET. Raw CSV a zdroj v P. SITL host wall časy testů jsou pouze doba experimentů.

Compiler `-fstack-usage` s přesnými H743 flags: event12B, save20B, close48B, begin16B; Timing execution_event16B, scheduler_end24B, dump_execution88B. Jde o lokální frames, **ne úplnou hloubku volání, IRQ/RTOS stack nebo watermark**. Ring/context/current mají26 pevných záznamů, po48B dle cílového layoutu; celkový linker delta1712B je přímo ověřen. ExpandingString heap spotřeba a formátovací stack při FTP nemají v této fázi nový H743 runtime watermark.

Historické E-B hodnoty Main4344/7168, ISR1264/1536 a FTP1216/2992 B se nesmějí přenést na nový firmware. Potenciální observer effect krátkých clock kritických sekcí, stack depth, IRQ interference, runtime heap/fragmentace a inicializace nového H743 kandidáta zůstávají HW neověřené.

## 11. Návrh samostatně schvalované HW kvalifikace

**Zatím pouze plán.** Upload konkrétního SHA vyžaduje nové schválení, stejně tak jednotlivé fyzické běhy. USB-only, bez LiPo/vrtulí, DISARMED, native STABILIZE, identita1075/serial200024001251303433333336, stabilní komunikace a záloha parametrů. Bez mazání logů, ARM, motorových příkazů a automatického retry.

1. Předem ověřit aktuální firmware/zdroje, podporované400Hz/FSTRATE0/rate-thread-off, zdravé senzory/EKF, paměť a RUN0. Neměnit letové parametry pro dosažení PASS. Nespouštět ani nepřepínat logger.
2. Kontrola observer effect: historická reference/nová EXEC OFF a nová ON/RUN0 při stejné telemetrii a obdobném klidném stavu; bez nových instrumentovaných metrik v compile-OUT použít původní Timing/PM/telemetrii, jejich omezení přiznat. Jednorázové sessions vyžadují oddělené schválené fyzické booty.
3. **Profil A:** ON firmware, Flight RUN0, COUNT4000, 15s baseline, jediná UTT_RUN1, přibližně10s capture, následné pozorování; až frozen sekvenční TimingV1 a ExecutionV1 export.
4. **Profil B:** tentýž ON firmware/letové parametry/kabel/USB/telemetrie, nový fyzický boot, oba snapshots pending; Flight COUNT4000/DECIM40, Timing COUNT4000, stejná baseline, jednou UTT_RUN1 a UCS_F_RUN1. Rozdíl startů explicitně evidovat; exportovat až po dokončení všech sessions, sériově. Žádný průběžný FTP/parametrický polling.
5. Baseline/active/post ve všech profilech stejná SYS_STATUS/SYSTEM_TIME/EKF telemetrie, např.2/1/1Hz s ověřením/obnovením původních intervalů. RAM/threads a volnou paměť číst mimo okno, ne `tasks.txt`; PM jen pokud už dostupné bez zásahu do loggeru. Nezískatelné PM.NLon/MaxT označit neověřené.
6. Vyhodnotit M1–M5 min/mean/max, count/sum/extreme IDs, původní Main/Rate/Attitude periody/jitter a Rate→Attitude, consume, CPU, nové i staré hook brackety, heap a Main/ISR/FTP watermark. Flight všech84 metrik, motor_changes0. Počet/presence/identity musí přesně sedět; absent optional consume v A není0µs měření.
7. M6: B−A mean, pozorovaná maxima, min/max a překročení; oddělit změnu času před outputem, trvání outputu, scheduler práce a periody. Dva odlišné booty/IRQ fázování nejsou dokonale párovaný experiment; více opakování pouze po samostatném schválení. Přesný MAD od průměru celého okna dostupný není; nepředstírat jej z omezeného raw ring.
8. Stop při InternalError, restartu, chybě komunikace/health/loggeru, mismatch/DESYNC, motor_changes, neúplných/nevalidních sondách nebo významné časové anomálii. Pro návaznost lze konzervativně použít předchozí vyšetřovací spouštěče Main/Rate>3000µs či perioda>=5000µs; jde o vyšetřovací kritéria z předchozího protokolu, nikoliv garantované bezpečné deadline. Nové spany nad2500µs vyžadují vyhodnocení příčiny, limity nelze založit pouze na libovolném procentu CPU. Při anomálii bez dalšího pokusu zachovat raw, obnovit dočasné nastavení a zastavit.

A/B umí ověřit pozorovaný posun softwarového motor outputu. Neprokáže ESC přijetí, fyzické DShot dokončení ani hard WCET. Pro ně by byla potřebná zvlášť navržená elektrická instrumentace a analýza nejhoršího rozvrhování/IRQ/driver/lock chování ve všech relevantních stavech.

## 12. Rozhodnutí a odpovědi

1. Main začíná po INS wait + reacquire v scheduleru. Měřený konec je poslední native scheduler boundary, ne celý AP_Vehicle/HAL cyklus.
2. Native Rate nastavuje AP_Motors hodnoty v původním rate_controller_run; pozdější motors_output_main provede mixer/output cestu.
3. Přidány consume begin/end, motor begin/return, mode finish, fast boundary, scheduler boundary; Main/Rate/Attitude se sdílí s Timing.
4. Měří se **softwarové volání**, ne elektrický výstup.
5. Main→motor return je přímo M3.
6. Main→scheduler boundary je přímo M4 s popsaným vyloučením finalizace a následné práce.
7. Skutečný H743 overhead zatím neznámý; offline engine batch means a virtuální SITL spany doloženy výše.
8. Flash .text+3428B, statická RAM+1712B, heap−1712B vůči OFF i reference.
9. Motorová hodnotová autorita původního AP zachována; časová izolace se zatím netvrdí.
10. Aktivní H743 UTB motorová větev zůstává compile-out.
11. Původní Flight i Timing regrese PASS.
12. Aktivní UTB ANGLE/ACRO **SITL** regrese PASS.
13. H743 OFF/ON compile/link PASS; historical reference ověřena, znovu nesestavována.
14. **GO pouze k vyžádání samostatného schválení USB-only DISARMED kvalifikace**, za uvedených omezení; není schválen upload ani fyzický test a není to kvalifikace letu.
15. Hard WCET vyžaduje úplné hranice, worst-case IRQ/thread/lock/driver analýzu a důkazy i mimo pozorovaná maxima.
16. I budoucí HW A/B zůstane DISARMED experiment se softwarovými hranicemi, observer efektem a neúplnou znalostí fyzického ESC času.
17. Přesný A/B postup je v části11; bez automatického pokračování.

Konečný Git: stejná větev/HEAD, index čistý, 7 tracked změn a7 nových souborů fáze, původní untracked soubory zachované. Zdrojový manifest, kontrola původních hashů a finální status jsou v P. Zdrojové změny jsou záměrně necommitnuté.
