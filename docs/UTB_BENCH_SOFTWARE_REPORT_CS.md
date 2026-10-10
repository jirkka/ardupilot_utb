# Softwarový report FÁZE 1H-A

Stav: implementována pouze diagnostická instrumentace. Finální výsledky buildů a testů jsou uvedeny níže; hardware Skystars/MicoAir, upload a let nejsou ověřeny/provedeny. Aktivní UTB_ACRO a FÁZE 2 nejsou povoleny.

## Předimplementační kontrola

SkystarsH7HD-bdshot používá W25Q128 SPI dataflash/BLOCK backend. LOG_BACKEND_TYPE=4 je místní BLOCK enum; 1 je filesystem a na této desce není relevantní. Ověřeno z hwdef, generovaného hwdef.h a AP_Logger zdrojů, nikoli z připojeného FC. Instrumentace měří AP_Logger_Block::write_sem. Chip sem a SPI device semaphore se neměří. Nejmenší korekce návrhu byla přesměrování filesystem měření na BLOCK a oprava parametru na 4; filesystem hook zůstal pouze pro SITL.

Samostatný AP_UTB_BENCH_ENABLED default 0 se šíří waf feature i --extra-hwdef do AP_Logger/AP_Scheduler. Preprocessor výpisy jsou `tmp/utb-1ha-preflight-AP_Logger.cpp.macros` a `tmp/utb-1ha-preflight-AP_Scheduler.cpp.macros`; konfigurace v `tmp/utb-1ha-preflight-configure.log`. Compile-time UTB OFF/BENCH ON je samostatná ověřovaná varianta, nikoli runtime profil A.

## Použité API a omezení

`AP_HAL::micros()/micros64()`, původní `Semaphore::take_blocking()/give()`, `in_main_thread()`, ChibiOS `chThdGetSelfX()`, lokální `stack_free(tp->wabase)` a skutečný stack region podle HAL Util. Thread callback je ve vehicle platform adaptéru; AP_Logger společná část neobsahuje ChibiOS include. Export používá původní logger mimo měřený lock a bounded nonblocking channel-0 telemetry. Stats/config mají 32bit lock-free atomic publikaci s jedním pokusem čtenáře, bez spin/retry/heapu.

Acquire latency je wall time t1−t0, hold t2−t1; časy zahrnují instrumentaci a preempci. Scheduler perioda je samostatná post-INS perioda. Interval/generace dává konzervativní potvrzené překryvy stejného mutexu; ostatní main acquire jsou UNKNOWN. Nula potvrzených kolizí neprokazuje nulové čekání.

ChibiOS Semaphores.cpp používá chMtxLock/chMtxUnlock; `modules/ChibiOS/os/rt/src/chmtx.c` implementuje full priority inheritance. Worker 58 nebo exporter 57 může při čekání main 180 získat jeho skutečnou aktuální prioritu, včetně původního scheduler boostu. PI omezuje inverzi přes preempci jinými vlákny, neodstraňuje lock hold ani případné další čekání. Maximální hard bound z této instrumentace nelze dokázat. Výsledky zachytí pozorované maximum, nikoli matematický worst case.

Existující AP_Logger frontend sdílené fmt/rate-limit struktury a neměřené chip/SPI zámky zůstávají riziky. Patch nemění logger policy ani priority původních vláken. Exporter přidává logger/telemetry zátěž; jeho wall time se měří. Samotná diagnostika proto vyžaduje A0/A srovnání na FC.

## Audit izolace a hranice ověření

Audit proti checkpointu kontroluje chráněné motor/HAL/EKF/arming/controller/mixer/reference/test soubory; přesný počet a změny jsou v `tmp/utb-1ha/isolation-audit.json`. Nové fast-path operace jsou pouze časování, fixed storage a atomiky. Původní mutex policy, worker stack request/priorita, čtyřpoložková queue, failure handling a motorová izolace jsou zachovány. Jediné nové logování UBEP probíhá v existujícím consumeru; ostatní bench logování ve vlastním exporteru. Žádný nový UTB motor setter není přidán.

SITL ověřuje funkci a regrese, nikoli H743 wall times/CPU/PI. AP_HAL čas v SITL je virtuální; nulový execution/acquire výsledek není důkaz nulového overheadu. Host helper microbenchmark je pouze Linux/x86 měření. Skutečná rezerva stacku, export overhead a 200/400 Hz trvalé logování zůstávají hardware neověřené. MicoAir743v2 build je kompatibilitní kontrola; jeho SDMMC filesystem mutex není instrumentován na HW.

## První hardware session

Přesný checklist, profily A0/A/B/C/D, časová okna, požadované readbacky, export a stop podmínky jsou v [UTB_HARDWARE_BENCH_CS.md](UTB_HARDWARE_BENCH_CS.md), oddíly 8–10. První session musí zůstat DISARMED bez vrtulí a vyžaduje samostatné schválení. Každý profil začíná rebootem; A0 a BENCH ON mají odlišné hashe, A–D jeden firmware a odlišné parametry. Žádný upload nebyl proveden.

Dosud neověřeno na H743: reálný backend detection, frame/wiring, CPU/logger load, všechny mutex latence/PI důsledky, nepřímé main blocking, worst-case hold, hlavní loop jitter/deadline rezerva, stack watermark/reserve, worker/exporter dlouhodobá stabilita, frekvence kompletních uložených sad a overhead diagnostiky. DISARMED měření samo nepotvrzuje letovou bezpečnost.

Nic nebylo doinstalováno; použity existující nástroje a toolchain. Pracovní strom nebyl commitnut ani pushnut.

## Přesný manifest změn této instrumentace

Rozdíl proti ověřenému checkpointu, včetně nových untracked souborů; původní FÁZE 0/1 dirty změny jsou níže v Git status samostatně.

| Soubor | Stav | Zásah |
|---|---|---|
| [ArduCopter/Copter.h](../ArduCopter/Copter.h) | modified | Guardované deklarace benchmark init/config/exporter metod. |
| [ArduCopter/Log.cpp](../ArduCopter/Log.cpp) | modified | Registrace čtyř guardovaných bench logů. |
| [ArduCopter/defines.h](../ArduCopter/defines.h) | modified | Připojení nových vehicle log IDs bez změny původních. |
| [ArduCopter/system.cpp](../ArduCopter/system.cpp) | modified | Inicializace diagnostiky při bootu. |
| [ArduCopter/utb.cpp](../ArduCopter/utb.cpp) | modified | Producer/consumer wall time, worker lifecycle/heartbeat/stack a UBEP; původní worker policy zachována. |
| [ArduCopter/utb_bench.cpp](../ArduCopter/utb_bench.cpp) | added | Samostatný omezený diagnostický exporter a konzistentní config snapshot. |
| [ArduCopter/utb_bench_log.h](../ArduCopter/utb_bench_log.h) | added | UBMT, UBHI, UBST a UBEP formáty/metadatové popisy. |
| [ArduCopter/utb_bench_platform.cpp](../ArduCopter/utb_bench_platform.cpp) | added | Skutečná thread identity a ChibiOS watermark přes existující API. |
| [README_BUILD_WSL_CS.md](../README_BUILD_WSL_CS.md) | modified | Reprodukovatelné build příkazy, archivace profilů a instalace. |
| [Tools/autotest/test_utb_bench.py](../Tools/autotest/test_utb_bench.py) | added | SITL A/B/C-D/failure/outage regrese a čtyři offline parser testy. |
| [Tools/autotest/utb_bench_analysis.py](../Tools/autotest/utb_bench_analysis.py) | added | Počet kompletních trvale uložených sad a frekvence z BIN/time-window. |
| [Tools/scripts/build_options.py](../Tools/scripts/build_options.py) | modified | Nezávislá UTB_BENCH feature, default 0. |
| [docs/UTB_ARCHITEKTURA_CS.md](../docs/UTB_ARCHITEKTURA_CS.md) | modified | Oddělený stav implementace, software ověření a neověřeného hardware. |
| [docs/UTB_BENCH_SOFTWARE_REPORT_CS.md](../docs/UTB_BENCH_SOFTWARE_REPORT_CS.md) | added | Tento výsledkový report, manifest, firmware identifikace a Git stav. |
| [docs/UTB_HARDWARE_BENCH_CS.md](../docs/UTB_HARDWARE_BENCH_CS.md) | modified | Skutečný backend, implementované měření, build/profil/bench postup a limity. |
| [libraries/AP_Logger/AP_Logger_Block.cpp](../libraries/AP_Logger/AP_Logger_Block.cpp) | modified | Timing kolem původního write_sem acquire/release. |
| [libraries/AP_Logger/AP_Logger_Block.h](../libraries/AP_Logger/AP_Logger_Block.h) | modified | Guardovaný per-mutex diagnostický stav BLOCK backendu. |
| [libraries/AP_Logger/AP_Logger_File.cpp](../libraries/AP_Logger/AP_Logger_File.cpp) | modified | Původní filesystem semaphore probe pouze v SITL. |
| [libraries/AP_Logger/AP_Logger_File.h](../libraries/AP_Logger/AP_Logger_File.h) | modified | Per-mutex stav pouze pro SITL filesystem backend. |
| [libraries/AP_Logger/AP_Logger_UTBBench.cpp](../libraries/AP_Logger/AP_Logger_UTBBench.cpp) | added | Timing, atribuce, publikace a lifecycle counters bez nového main locku. |
| [libraries/AP_Logger/AP_Logger_UTBBench.h](../libraries/AP_Logger/AP_Logger_UTBBench.h) | added | Fixed atomic Stats/Frame/Interval a RAII API. |
| [libraries/AP_Logger/AP_Logger_config.h](../libraries/AP_Logger/AP_Logger_config.h) | modified | Default AP_UTB_BENCH_ENABLED=0. |
| [libraries/AP_Logger/tests/test_utb_bench.cpp](../libraries/AP_Logger/tests/test_utb_bench.cpp) | added | 12 benchmark unit testů včetně souběžných snapshotů/wrap/lock policy. |
| [libraries/AP_Logger/tests/wscript](../libraries/AP_Logger/tests/wscript) | added | Registrace benchmark unit cíle. |
| [libraries/AP_Scheduler/AP_Scheduler.cpp](../libraries/AP_Scheduler/AP_Scheduler.cpp) | modified | Post-INS period/jitter a existing-branch overrun counter. |
| [libraries/AP_UTB/AP_UTB.cpp](../libraries/AP_UTB/AP_UTB.cpp) | modified | Param index 19; index 20 pouze SITL; původní indexy beze změny. |
| [libraries/AP_UTB/AP_UTB.h](../libraries/AP_UTB/AP_UTB.h) | modified | Guardované B_WAIT_US/B_FAIL členy a read-only param getters. |

## Skutečně provedené buildy a testy

Baseline před C++ změnami: všechny kroky PASS, původních 31 unit testů, FÁZE 0 ON/OFF a plná shadow regrese.

| Krok | Výsledek | Log |
|---|---|---|
| sitl-on-bench-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-on-bench-config.log) |
| sitl-on-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-on-bench.log) |
| unit | PASS (exit 0) | [log](../tmp/utb-1ha/post/unit.log) |
| bench-unit | PASS (exit 0) | [log](../tmp/utb-1ha/post/bench-unit.log) |
| bench-sitl | PASS (exit 0) | [log](../tmp/utb-1ha/post/bench-sitl.log) |
| skeleton-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/skeleton-bench.log) |
| shadow-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/shadow-bench.log) |
| h7-on-bench-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-on-bench-config.log) |
| h7-on-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-on-bench.log) |
| h7-on-plain-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-on-plain-config.log) |
| h7-on-plain | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-on-plain.log) |
| h7-off-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-off-config.log) |
| h7-off | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-off.log) |
| sitl-on-plain-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-on-plain-config.log) |
| sitl-on-plain | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-on-plain.log) |
| unit-plain | PASS (exit 0) | [log](../tmp/utb-1ha/post/unit-plain.log) |
| skeleton-plain | PASS (exit 0) | [log](../tmp/utb-1ha/post/skeleton-plain.log) |
| shadow-plain | PASS (exit 0) | [log](../tmp/utb-1ha/post/shadow-plain.log) |
| sitl-off-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-off-config.log) |
| sitl-off | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-off.log) |
| skeleton-off | PASS (exit 0) | [log](../tmp/utb-1ha/post/skeleton-off.log) |
| sitl-off-bench-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-off-bench-config.log) |
| sitl-off-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-off-bench.log) |
| bench-unit-no-utb | PASS (exit 0) | [log](../tmp/utb-1ha/post/bench-unit-no-utb.log) |
| h7-no-logging-bench-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-no-logging-bench-config.log) |
| h7-no-logging-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-no-logging-bench.log) |
| micoair-on-bench-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/micoair-on-bench-config.log) |
| micoair-on-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/micoair-on-bench.log) |
| sitl-on-bench-final-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-on-bench-final-config.log) |
| sitl-on-bench-final | PASS (exit 0) | [log](../tmp/utb-1ha/post/sitl-on-bench-final.log) |
| unit-final | PASS (exit 0) | [log](../tmp/utb-1ha/post/unit-final.log) |
| bench-unit-final | PASS (exit 0) | [log](../tmp/utb-1ha/post/bench-unit-final.log) |
| bench-sitl-final | PASS (exit 0) | [log](../tmp/utb-1ha/post/bench-sitl-final.log) |
| skeleton-bench-final | PASS (exit 0) | [log](../tmp/utb-1ha/post/skeleton-bench-final.log) |
| shadow-bench-final | PASS (exit 0) | [log](../tmp/utb-1ha/post/shadow-bench-final.log) |
| h7-on-bench-final-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-on-bench-final-config.log) |
| h7-on-bench-final | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-on-bench-final.log) |
| h7-off-bench-config | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-off-bench-config.log) |
| h7-off-bench | PASS (exit 0) | [log](../tmp/utb-1ha/post/h7-off-bench.log) |

Přesná argv v [results.json](../tmp/utb-1ha/post/results.json). Počáteční chyby kompilace (forward declarations, unsigned GTest comparison, unit hal symbol) a enum metadata parseru byly opraveny pouze v instrumentaci. Historické failure logy zůstávají v tmp/utb-1ha/post; finální úspěchy jsou výše.

Finální UNIT: **31/31 původních + 12/12 nových PASS**. BENCH unit také PASS při AP_UTB compile-time OFF. Benchmark SITL ověřuje A/B/C-D, simulated create failure, logger outage/recovery, disarmed stav a uložené logy. Offline parser: čtyři PASS (epoch separation, full window, missing UBEP, duplicate record). FÁZE 0 a plná FÁZE 1 shadow regrese ON/OFF jsou v tabulce.

Host-only helper měření: Stats.record 19.45 ns, Stats.read 11.50 ns/operaci při 100000 iteracích. Linux/x86, včetně host scheduling; **není H743 overhead**. Skutečný H743 overhead se musí měřit A0 versus A.

Finální shadow BIN: 25049 kompletních sedmizáznamových sad včetně UBEP, 0 neúplných skupin. Ustálená SITL okna původního shadow testu: 200 Hz → 199.997383 Hz, 400 Hz → 399.326397 Hz. Přesný časový interval/count v shadow-bench-final/results.json. Celkové smíšené epochy se nepoužívají k odhadu jednotlivých rate profilů; full-window hardware throughput vyžaduje explicitní časové meze.

Motor isolation SITL: STABILIZE při stejném RC, 20 vzorků v DISARMED a 20 v armed ground idle, 0 rozdílů steady RCOU mezi UTB OFF/ON. Nebyl proveden vzlet; bitwise shoda transientů/letu se netvrdí. UTB_ACRO arming guard prošel FÁZÍ 0.

## Firmware varianty — velikosti a SHA-256

A–D používají stejný APJ, liší se readback parametry. A0 má jinou binárku. Artifact adresář je součást identifikace, samotné jméno arducopter.apj nestačí. ELF/HEX/BIN/APJ hashe jsou oddělené. HEX obsahuje bootloader; upload žádné varianty nebyl proveden.

| Archiv/účel | Board | UTB | BENCH | Přesné configure flags |
|---|---|---|---|---|
| h7-on-plain: A0 | SkystarsH7HD-bdshot | 1 | 0 | `--board SkystarsH7HD-bdshot --enable-UTB --disable-UTB_BENCH` |
| h7-on-bench-final: A/B/C/D | SkystarsH7HD-bdshot | 1 | 1 | `--board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH --extra-hwdef /home/jirka/ardupilot_utb/tmp/utb-1ha/bench-on.hwdef` |
| h7-off: UTB/BENCH compile-out | SkystarsH7HD-bdshot | 0 | 0 | `--board SkystarsH7HD-bdshot --disable-UTB --disable-UTB_BENCH` |
| h7-off-bench: UTB OFF/BENCH ON guard | SkystarsH7HD-bdshot | 0 | 1 | `--board SkystarsH7HD-bdshot --disable-UTB --enable-UTB_BENCH --extra-hwdef /home/jirka/ardupilot_utb/tmp/utb-1ha/bench-on.hwdef` |
| h7-no-logging-bench: logging-disabled guard, NE pro bench | SkystarsH7HD-bdshot | 1 | 1 | `--board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH --extra-hwdef /home/jirka/ardupilot_utb/tmp/utb-1ha/no-logging.hwdef` |
| micoair-on-bench: MicoAir compatibility, NE Skystars FW | MicoAir743v2 | 1 | 1 | `--board MicoAir743v2 --enable-UTB --enable-UTB_BENCH` |
| sitl-on-plain: SITL UTB ON/BENCH OFF | sitl | 1 | 0 | `--board sitl --enable-UTB --disable-UTB_BENCH` |
| sitl-off: SITL OFF/OFF | sitl | 0 | 0 | `--board sitl --disable-UTB --disable-UTB_BENCH` |
| sitl-off-bench: SITL OFF/ON | sitl | 0 | 1 | `--board sitl --disable-UTB --enable-UTB_BENCH` |
| sitl-on-bench-final: SITL ON/ON | sitl | 1 | 1 | `--board sitl --enable-UTB --enable-UTB_BENCH` |

| Soubor | Velikost B | SHA-256 |
|---|---:|---|
| [tmp/utb-1ha/post/h7-on-plain/arducopter.bin](../tmp/utb-1ha/post/h7-on-plain/arducopter.bin) | 1244148 | `0ab9e5508bb9c5e40074545930654982a52399a98b51c0b533ee0f9a365bf3a3` |
| [tmp/utb-1ha/post/h7-on-plain/arducopter_with_bl.hex](../tmp/utb-1ha/post/h7-on-plain/arducopter_with_bl.hex) | 3782212 | `84e90928f57a8c57678c2b794f9835eb2bdf73ad681d00041acf623ded10ac51` |
| [tmp/utb-1ha/post/h7-on-plain/arducopter.apj](../tmp/utb-1ha/post/h7-on-plain/arducopter.apj) | 1122751 | `490497367f880d68a835e042e7608637da3f3f045679caadd8d6e64f5c0d9382` |
| [tmp/utb-1ha/post/h7-on-plain/arducopter](../tmp/utb-1ha/post/h7-on-plain/arducopter) | 2340480 | `f17f69fa501682b4a14503753b7a6fc65b2c736c8448bcbcd906bc77790e8cdc` |
| [tmp/utb-1ha/post/h7-on-bench-final/arducopter.bin](../tmp/utb-1ha/post/h7-on-bench-final/arducopter.bin) | 1249208 | `be374bc71d2ec4823a93f1ec410147024325ce74675045cba7dc51bfccfaa310` |
| [tmp/utb-1ha/post/h7-on-bench-final/arducopter_with_bl.hex](../tmp/utb-1ha/post/h7-on-bench-final/arducopter_with_bl.hex) | 3796140 | `01691d88efd232b62ae1f1130a01cb1dbd9fab62f3ef7bf4269083fa3b02dd86` |
| [tmp/utb-1ha/post/h7-on-bench-final/arducopter.apj](../tmp/utb-1ha/post/h7-on-bench-final/arducopter.apj) | 1126995 | `2b835086d9c11b885be38cf2700ab560be44511c3b8d879501b12e178363f027` |
| [tmp/utb-1ha/post/h7-on-bench-final/arducopter](../tmp/utb-1ha/post/h7-on-bench-final/arducopter) | 2345772 | `2356b11dde1fe2515901fa43edf599d924ef8389072b9bfedf6e3787578107ad` |
| [tmp/utb-1ha/post/h7-off/arducopter.bin](../tmp/utb-1ha/post/h7-off/arducopter.bin) | 1233328 | `812f18cab6ce240bbe8ceb542308591af7dd37136dd15b6b188bd06986758933` |
| [tmp/utb-1ha/post/h7-off/arducopter_with_bl.hex](../tmp/utb-1ha/post/h7-off/arducopter_with_bl.hex) | 3752448 | `f59ec2f2538f662677c94b407a92334232467ad0d8974e4ee1b1e3706bbc7d24` |
| [tmp/utb-1ha/post/h7-off/arducopter.apj](../tmp/utb-1ha/post/h7-off/arducopter.apj) | 1113287 | `26ec7766bfbe96af4a154320fd13d4c910bbd3526619668e6380fe87779366de` |
| [tmp/utb-1ha/post/h7-off/arducopter](../tmp/utb-1ha/post/h7-off/arducopter) | 2335188 | `359b238e6b9724895b676f070e22b40f4a80416a6f04da7ed81e08b5cd9a28aa` |
| [tmp/utb-1ha/post/h7-off-bench/arducopter.bin](../tmp/utb-1ha/post/h7-off-bench/arducopter.bin) | 1237492 | `9ef5af2b62c9a2bb42e7121e6a82d4db051f4b3314f15e9d441e1343509a5ffb` |
| [tmp/utb-1ha/post/h7-off-bench/arducopter_with_bl.hex](../tmp/utb-1ha/post/h7-off-bench/arducopter_with_bl.hex) | 3763908 | `471e2b46f75ae5565b228806dcab357f18b936f28fc8389ca8d46956398b363c` |
| [tmp/utb-1ha/post/h7-off-bench/arducopter.apj](../tmp/utb-1ha/post/h7-off-bench/arducopter.apj) | 1116751 | `c3b35ff0828199715a7131827055c0c48492f720fa6f165916410c751f6fa4de` |
| [tmp/utb-1ha/post/h7-off-bench/arducopter](../tmp/utb-1ha/post/h7-off-bench/arducopter) | 2339616 | `058054273c3e9472c2a1335db54919d1a1172313efb07eb86c5b946073483dd5` |
| [tmp/utb-1ha/post/h7-no-logging-bench/arducopter.bin](../tmp/utb-1ha/post/h7-no-logging-bench/arducopter.bin) | 1165836 | `8f819a7875db2202122586f29c09c1ec6f4c21e6e3bdeb53e14fb1e909c92c7b` |
| [tmp/utb-1ha/post/h7-no-logging-bench/arducopter_with_bl.hex](../tmp/utb-1ha/post/h7-no-logging-bench/arducopter_with_bl.hex) | 3566832 | `71a05c26621d65613170aa2ebf18954c545465bead0912ed7029126ed2d2cab5` |
| [tmp/utb-1ha/post/h7-no-logging-bench/arducopter.apj](../tmp/utb-1ha/post/h7-no-logging-bench/arducopter.apj) | 1052115 | `b542dd9f228972eee7774f2ea66fb9ca097fd0d6666bb6b0627c80803c2eb2eb` |
| [tmp/utb-1ha/post/h7-no-logging-bench/arducopter](../tmp/utb-1ha/post/h7-no-logging-bench/arducopter) | 2234120 | `1a505d960289b1f47fbf42d6f49328c17108998048cfbd1dbb563ca96753e935` |
| [tmp/utb-1ha/post/micoair-on-bench/arducopter.bin](../tmp/utb-1ha/post/micoair-on-bench/arducopter.bin) | 1448744 | `0e46aa791dbb428118ea1f4c351465d5223452e774809dfb7938c815e44a687c` |
| [tmp/utb-1ha/post/micoair-on-bench/arducopter_with_bl.hex](../tmp/utb-1ha/post/micoair-on-bench/arducopter_with_bl.hex) | 4344912 | `b9c48f5354b8edfdefd4fbc56a9c587c4d9763ff9565e13a98a3013d9ef0020f` |
| [tmp/utb-1ha/post/micoair-on-bench/arducopter.apj](../tmp/utb-1ha/post/micoair-on-bench/arducopter.apj) | 1296476 | `1b7abba6e4c4f606490c691dd60dc5b9dc1d9f598880c24cc912533768a55752` |
| [tmp/utb-1ha/post/micoair-on-bench/arducopter](../tmp/utb-1ha/post/micoair-on-bench/arducopter) | 2707004 | `d43a57fa61ffbbcee52a01d963608990322c88025768a1d6e91cfc51cee432fb` |
| [tmp/utb-1ha/post/sitl-on-plain/arducopter](../tmp/utb-1ha/post/sitl-on-plain/arducopter) | 5714696 | `10adcd31414199ec3caba4bf6ca336f16d780d1aae1f7fd2d3258b018a98eb8f` |
| [tmp/utb-1ha/post/sitl-off/arducopter](../tmp/utb-1ha/post/sitl-off/arducopter) | 5686112 | `9f7ff80d4fcf566c76976faf6b0bf38a6413b86f54c8670bf48e3d31dfc35123` |
| [tmp/utb-1ha/post/sitl-off-bench/arducopter](../tmp/utb-1ha/post/sitl-off-bench/arducopter) | 5693664 | `4f1f727471a859a098db4e5e2487ea47c86e7f679b8a7c3e3e11a7826abec81f` |
| [tmp/utb-1ha/post/sitl-on-bench-final/arducopter](../tmp/utb-1ha/post/sitl-on-bench-final/arducopter) | 5722616 | `3234c9c90aaf4d8ccf69a2858d1b2f3aeeb6456e84736a19a1c85d0c63cf56d4` |

Plný [firmware manifest JSON](../tmp/utb-1ha/firmware-manifest.json) obsahuje i APJ board ID/image size a argv. Profilové parametry jsou v [profiles](../tmp/utb-1ha/profiles/); nejde o doklad parametrů dosud nepřipojené FC.

### Linker/build footprint

Waf BSS nezahrnuje rezervované heap regiony, které obyčejné arm-none-eabi-size počítá do BSS. Níže jsou skutečné Waf summary; runtime heap/stack rezerva se z této tabulky neurčuje. Exporter navíc při bootu žádá stack 3072 B plus původní HAL working-area overhead; backend heap allocation roste o per-mutex stav. Samotné Waf data/BSS nejsou celá RAM cena.

| Varianta | Text B | Data B | Waf BSS B | Flash B | Free flash B |
|---|---:|---:|---:|---:|---:|
| h7-on-plain | 1239876 | 4268 | 128580 | 1244144 | 459788 |
| h7-on-bench-final | 1244096 | 5112 | 128704 | 1249208 | 454728 |
| h7-off | 1229064 | 4264 | 125704 | 1233328 | 470608 |
| h7-off-bench | 1232380 | 5108 | 125828 | 1237488 | 466444 |
| h7-no-logging-bench | 1160724 | 5108 | 127956 | 1165832 | 538100 |
| micoair-on-bench | 1443544 | 5196 | 133348 | 1448740 | 255192 |

BENCH ON minus A0: text +4220 B, data +844 B, Waf BSS +124 B, flash +5064 B. Nejde o CPU čas ani runtime stack overhead. BENCH OFF Waf footprint odpovídá předimplementační baseline.

## Source/style/metadata audit

PASS: chráněné soubory proti checkpointu, motor/arming izolace, preserved acquire/release policy, bounded snapshots a žádná nová fast-path alokace/blocking operace. Původní main logger acquire stále může blokovat; instrumentace jej měří a neruší. Shared AP_Logger frontend a neměřené mutexy zůstávají RISK, nikoli PASS celé architektury.

Chráněných souborů: 2556, změněných: 0. RATE/PID/mixer/reference/arming/HAL/DShot/EKF a původní 31-test soubor jsou beze změny proti checkpointu.

PASS: BENCH OFF ELF neobsahuje benchmark symboly; BENCH ON ano. ARM/native nemají out-of-line atomic runtime load/store/fetch/exchange/compare helpers. Detaily: [symbol audit](../tmp/utb-1ha/final-symbol-audit.json).

PASS: flake8, astyle nových souborů a změněných hunků, git diff --check, param metadata a logger metadata. Formáty/labels/units/multipliers a packed délky čtyř bench logů ověřeny. IDs UTB ON=19/20/21/22, UTB OFF=13/14/15/16; původní IDs se neposouvají. B_WAIT_US index 19, B_FAIL index 20 pouze SITL, oba BENCH-only.

Důkazy v tmp/utb-1ha: isolation-audit.json, source-freeze.json, style-check.json, logger-schema-audit.json, flake8.log, param-metadata.log, logger-metadata.log, diff-check.log. Lokální tmp je ignorované úložiště, není automaticky verzovaná CI evidence.

## Git/checkpoint stav

HEAD `761a38d7042f5a6d91c7dc065043e9cb84b81486`, větev `ardupilot-4.7.1-utb`. Žádný commit/push/upload. Původní FÁZE 0/1 změny zůstávají dirty; jejich přítomnost v status neznamená změnu touto instrumentací. Nové untracked soubory jsou záměrný source/docs/test obsah. Ignorované tmp/build obsahují reprodukční checkpoint, logy a firmware.

```text
 M ArduCopter/Attitude.cpp
 M ArduCopter/Copter.cpp
 M ArduCopter/Copter.h
 M ArduCopter/Log.cpp
 M ArduCopter/defines.h
 M ArduCopter/mode_utb_acro.cpp
 M ArduCopter/system.cpp
 M README_ARDUPILOT_UTB_SETUP_BUILD_CS.md
 M README_BUILD_WSL_CS.md
 M Tools/autotest/test_utb_skeleton.py
 M Tools/scripts/build_options.py
 M docs/UTB_ARCHITEKTURA_CS.md
 M libraries/AP_Logger/AP_Logger_Block.cpp
 M libraries/AP_Logger/AP_Logger_Block.h
 M libraries/AP_Logger/AP_Logger_File.cpp
 M libraries/AP_Logger/AP_Logger_File.h
 M libraries/AP_Logger/AP_Logger_config.h
 M libraries/AP_Scheduler/AP_Scheduler.cpp
 M libraries/AP_UTB/AP_UTB.cpp
 M libraries/AP_UTB/AP_UTB.h
 M libraries/AP_UTB/AP_UTB_MotorMixer.cpp
 M libraries/AP_UTB/AP_UTB_MotorMixer.h
 M libraries/AP_UTB/AP_UTB_RateController.cpp
 M libraries/AP_UTB/AP_UTB_RateController.h
 M libraries/AP_UTB/AP_UTB_State.cpp
 M libraries/AP_UTB/AP_UTB_State.h
 M libraries/AP_UTB/tests/test_utb.cpp
?? ArduCopter/utb.cpp
?? ArduCopter/utb_bench.cpp
?? ArduCopter/utb_bench_log.h
?? ArduCopter/utb_bench_platform.cpp
?? ArduCopter/utb_log.h
?? Tools/autotest/test_utb_bench.py
?? Tools/autotest/test_utb_shadow.py
?? Tools/autotest/utb_bench_analysis.py
?? docs/UTB_BENCH_SOFTWARE_REPORT_CS.md
?? docs/UTB_CONTROL_THEORY_CS.md
?? docs/UTB_HARDWARE_BENCH_CS.md
?? libraries/AP_Logger/AP_Logger_UTBBench.cpp
?? libraries/AP_Logger/AP_Logger_UTBBench.h
?? libraries/AP_Logger/tests/test_utb_bench.cpp
?? libraries/AP_Logger/tests/wscript
?? libraries/AP_UTB/AP_UTB_Reference.cpp
?? libraries/AP_UTB/AP_UTB_Reference.h
```

Checkpoint: 307731958 B, SHA-256 `92bcd530009062832ab8595cbfe43b80c7da795d953668ec6ef8f1a52efae51c`, 7460 tracked/untracked souborů. Manifest, tracked.patch a HEAD/status jsou vedle archivu. Submoduly nebyly měněny.

Toolchain: GNU ARM GCC 10.2.1; native g++ 15.2.0, Python 3.14.4, astyle 3.6.12, flake8 7.4.1. Nic se neinstalovalo.

## Závěr

Software instrumentace a požadované lokální regrese PASS. Připraveno k posouzení první řízené DISARMED session bez vrtulí podle bench checklistu; hardware bezpečnost, worst-case latence a udržitelnost logování jsou stále NEOVĚŘENO. Nejde o schválení letu nebo aktivního UTB controlleru. Práce končí diagnostikou; další krok až po schválení hardware session.
