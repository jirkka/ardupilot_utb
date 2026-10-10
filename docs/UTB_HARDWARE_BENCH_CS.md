# FÁZE 1H-A: hardware benchmark SHADOW UTB

DESIGN: schválený BLOCK návrh. IMPLEMENTED: instrumentace pod AP_UTB_BENCH_ENABLED. Výsledky UNIT/SITL/BUILD VERIFIED jsou v závěrečném softwarovém reportu. **SKYSTARS HARDWARE VERIFIED: NEOVĚŘENO. MICOAIR HARDWARE VERIFIED: NEOVĚŘENO. FLIGHT VERIFIED: NEOVĚŘENO.**

Žádný fyzický benchmark, upload ani let nebyl proveden. Skystars H7 Dual Gyro je testovací deska; finální MicoAir H743 V2 45A AIO AM32 potřebuje samostatné ověření.

## 1. Skutečný backend

SkystarsH7HD-bdshot dědí SkystarsH7HD/hwdef.dat: STM32H743, SPI3 dataflash W25Q128, HAL_LOGGING_DATAFLASH_ENABLED=1. Generovaná konfigurace má HAL_OS_FATFS_IO=0, HAL_OS_LITTLEFS_IO=0 a HAL_USE_FATFS=FALSE. Zapisovatelný filesystem backend zde není. HAL_BOARD_LOG_DIRECTORY ani ROMFS nejsou důkaz filesystem logging.

AP_Logger::Backend_Type: FILESYSTEM=1, MAVLINK=2, BLOCK=4. **Skystars bench používá pouze LOG_BACKEND_TYPE=4.** Hodnota1 skončí jako nepodporovaný backend. Přítomnost a inicializace fyzické flash jsou hardware neověřené.

Měřený zámek: AP_Logger_Block::_WritePrioritisedBlock(), **write_sem** kolem RAM ring-buffer zápisu. Původní take_blocking/give, lock order a watchdog semaphore_line atribuce jsou zachovány. Další AP_Logger_Block::sem, JEDEC SPI device semaphore a fyzické programování/erase flash **nejsou pokryty**. Měření nedokazuje celkovou race bezpečnost AP_Logger.

SITL ekvivalent používá stejný helper na svém skutečném AP_Logger_File::semaphore, výhradně při HAL_BOARD_SITL. Není to H7 hardware měření. V SITL jsou AP_HAL časy virtuální; execution/acquire/period hodnoty nejsou host wall-time ani CPU benchmark a mohou být i nulové. Reálný monotónní hardware čas se používá na ChibiOS. MicoAir filesystem mutex hook zatím nemá.

## 2. Compile-time a varianty

AP_UTB_BENCH_ENABLED default0. BENCH=0 odstraňuje timing, storage, callbacky, exporter, logy a výhradně benchmark parametry. AP_Logger/AP_Scheduler nezávisí na AP_UTB; společná diagnostika patří do AP_Logger_UTBBench.*. BENCH=1 je dostupná při UTB_ENABLE=0 i compile-time UTB OFF. Logging OFF nemá logger export, ostatní metriky mohou jít do telemetry.

| Varianta | BENCH | UTB_ENABLE | UTB_SHADOW | UTB_LOG_RATE |
| --- | --- | --- | --- | --- |
| A0 | 0 | 0 | 0 | bez významu |
| A | 1 | 0 | 0 | bez významu |
| B | 1 | 1 | 0 | bez významu |
| C | 1 | 1 | 1 | 200 |
| D | 1 | 1 | 1 | 400 |

A0/A mají AP_UTB compile-time ON. Samostatný UTB OFF je compile/regression kontrola. A–D sdílejí instrumentovaný firmware a liší se parametrovým profilem. ENABLE je boot latch: restart a readback. Kontrola varianty vyžaduje SHA, APJ board ID, BENCH flags a parametry; nestačí název souboru.

A−A0: cena instrumentace včetně exporteru. B−A: idle worker/status. C−B: shadow +200Hz. D−C: dodatečné logování. A0 správně nemá UB metriky.

## 3. Metoda měření

AP_HAL::micros()/micros64() rozšiřují hardwarový monotónní čas. Obě hwdef používají CH_CFG_ST_RESOLUTION=16; přímý counter se nečte. Uint32 rozdíly fungují přes wrap; signed pořadí intervalů vyžaduje délky pod 2^31µs. Krátké bench běhy nepředstavují důkaz neomezeného provozu/counter wrap bezpečnosti.

- t0 ihned před původním blocking acquire, t1 ihned po návratu: acquire=t1−t0. Obsahuje cenu API, čekání a preempci; nenulová hodnota sama nedokazuje contention.
- t2 ihned před původním give: hold=t2−t1. Obsahuje instrumentaci a preempci, nikoli cenu samotného give/přesný kernel ownership interval.
- Worker pro atribuci publikuje konzervativní konec intervalu těsně před finálním t2. Publikace musí být před unlock; jinak by ji probuzený main nemusel vidět. Koncový úsek může skončit UNKNOWN.
- Potvrzená kolize vyžaduje změnu generace intervalu během main acquire a prokazatelné překrytí stejného mutexu. Stará generace se po wrap nepovažuje za novou kolizi. Všechna nepotvrzená main acquire jsou UNKNOWN, včetně uncontended případů. **Nula potvrzených kolizí není důkaz nulového blocking rizika.**
## 4. Snapshoty, metriky a atribuce

Identita MAIN se určuje existujícím `in_main_thread()`, UTB_WORKER podle skutečné identity registrované při vstupu do workeru. Ostatní vlákna jsou OTHER; nedostupná identita je UNKNOWN. ChibiOS používá `chThdGetSelfX()`, SITL `pthread_self()`. Priorita není identita vlákna. Lokální ChibiOS používá full priority inheritance: nominální main 180 může zvýšit prioritu vlastníka 58 (worker) nebo 57 (exporter). Dědí se skutečná aktuální priorita čekajícího main, včetně případného původního scheduler boostu, nikoli konstanta z benchmarku. PI neruší dobu držení zámku.

Publikace používá pouze lock-free 32bit atomiky a pointery; jejich podporu kontroluje static_assert. Čítače mají jednoho zapisovatele; OTHER/UNKNOWN statistiky daného backendu se publikují uvnitř jeho původního mutexu. Součet se publikuje jako dvě 32bit slova. Čtenář zkusí snapshot jednou a při změně generace jej zahodí. Neobsahuje spin/retry ani nový mutex. Konfigurace varianty, epochy, rate a threshold tvoří samostatný konzistentní snapshot; neplatné čtení má variantu 255.

UBMT obsahuje počet, kumulativní součet, minimum, maximum a poslední wall-time v mikrosekundách. UBHI obsahuje 12 histogramových košů: první 0–1 us, další mocniny dvou, poslední >=2048 us. Metriky:

| Met | Veličina |
|---|---|
| 0 | Main logger mutex acquire latency |
| 1 | UTB worker acquire latency |
| 2 | UTB worker mutex hold duration |
| 3/4 | OTHER/UNKNOWN acquire latency primárního měřeného backendu |
| 5 | Post-INS main loop period |
| 6 | Absolutní odchylka periody od scheduler nominální periody |
| 7/8 | UTB producer/consumer execution wall time |
| 9 | Worker pass wall time |
| 10 | Diagnostický export wall time |

Scheduler overruns se počítají v existující overrun větvi. Samostatný čítač period > nominální perioda není scheduler overrun: může zahrnovat běžný timer jitter. Perioda, acquire latency a execution wall time se nesmějí zaměňovat. Existující PM statistiky zůstávají zachovány.

UBST a NAMED_VALUE_INT `UB00` až `UB36` sdílejí klíče:

| Klíč | Význam |
|---|---|
| 0–3 | Varianta, epocha, požadovaný rate, B_WAIT_US |
| 4–7 | Main acquire nad threshold, potvrzené kolize, jejich max latency, UNKNOWN |
| 8–9 | Scheduler overruns, perioda nad nominálem |
| 10–13 | Worker lifecycle, počet passů, poslední begin/done v us |
| 14–16 | Požadovaný stack, skutečná dostupná velikost, minimum volných bytů |
| 17–18 | Export pass s vynechaným výstupem, exporter lifecycle |
| 19–26 | Req, Enq, Wr, Drop, Miss, HWM, depth, Gone původní UTB queue |
| 27–32 | Max acquire, hold, period, producer, consumer, export |
| 33 | Primární měřený backend: 4 BLOCK, 1 SITL filesystem, 0 žádný |
| 34–35 | Kumulativní pokusy a selhání vytvoření workeru |
| 36 | Samostatná dostupnost loggeru |

Lifecycle: 0 nepožadován, 1 vytváření, 2 selhání, 3 běží. Neznámý stack watermark má UINT32_MAX; telemetry transportuje stejné bity v int32. Přetečení 32bit časů/čítačů je nutné zohlednit při offline analýze.

## 5. Worker stack a lifecycle

Worker zachovává původní stack request 3072 B, prioritu IO=58, drain limit čtyř sad a delay 1 ms. Watermark se odečítá uvnitř jeho vlastního vlákna nejvýše jednou za sekundu. ChibiOS používá stejný postup jako lokální HAL Util: oblast od wabase do adresy thread_t a `stack_free()` z pattern fill. Skutečný region se loguje odděleně od requested hodnoty. SITL neposkytuje ChibiOS watermark.

Watermark je historické minimum pozorované volné oblasti; není důkaz bezpečně dostatečné rezervy ani detektor všech stack corruption. Konečnou rezervu musí posoudit hardware měření s nejhorší očekávanou logger cestou. Původní thread_create failure handling zůstává zachováno; diagnostický exporter je nezávislý. `UTB_B_FAIL` existuje pouze v SITL BENCH sestavení a přeskočí vytvoření workeru. Na FC se nedostatek paměti nesimuluje vyčerpáním heapu.

## 6. Export a jeho overhead

BENCH vytváří samostatný diagnostický exporter s prioritou IO−1=57 a stack request 3072 B. Po každém passu čeká 1000 ms. Selhání vytvoření exporteru se uloží do jeho atomic lifecycle, ale neexistující vlákno se samo nevyexportuje; chybějící UB výstup je nepozorovatelný experiment, nikoli nulová zátěž/latence. Výstup je omezen na 11 UBMT, 11 UBHI a 37 UBST záznamů plus 37 NAMED_VALUE_INT na pass. Každá shadow sada přidává UBEP s epochou. Souhrnné BIN payloady včetně tříbajtových record headerů mají 1932 B/pass (bez FMT/FMTU a původních logů); UBEP přidává 19 B/sadu, tedy 3800 B/s při 200 Hz nebo 7600 B/s při 400 Hz. MAVLink má nejvýše 37 paketů/pass; skutečná linková cena závisí na verzi/signingu. Tyto byte counts nejsou naměřený CPU overhead. Všechny exporter logger zápisy probíhají mimo měřený mutex; exporter může na původním loggeru čekat a sám přidává OTHER contender.

MAVLink export je pouze na channel 0, s původním channel mutexem přes take_nonblocking a kontrolou TX prostoru. Nevolá blocking convenience send. Původní UART/backend synchronizace může ovlivnit exporter nebo sdílené prostředky; tato implementace ji nemění. Počet kroků je omezený, wall time celého exportu nemá dokázaný hard bound.

Každý mutex probe přidává čtení monotónního času, pevný počet atomických operací a aktualizaci statistiky. Main loop přidává měření periody, producer/consumer dva časové odečty a bounded publikaci. Žádná nová heap alokace ani blocking operace se nepřidává do main fast path. Vytvoření diagnostického vlákna probíhá při inicializaci. BENCH také zvětšuje při bootu alokovaný backend o per-mutex stav; samotné Waf BSS tedy nepopisuje celou RAM cenu. Instrumentace sama prodlužuje pozorovaný hold a přidává cache/časovač overhead.

Host microbenchmark helperu je uveden v software reportu; nepřevádí se na H743. Skutečný overhead na FC určí A0 versus A za stejné konfigurace, napájení, logger/telemetry provozu a délky měření. A versus B/C/D pak oddělí zapnutí UTB a shadow logging. CPU load, PM a scheduler ukazatele musí být porovnány společně, nikoli jediným maximem.

## 7. Skutečně uložené kompletní sady

Wr je úspěšné předání sady backendu, nikoli potvrzení trvalého média. Nástroj `Tools/autotest/utb_bench_analysis.py` analyzuje stažený BIN podle (TimeUS, Seq). Kompletní sada vyžaduje právě tři UTBR osy 0/1/2 a po jednom UTBM, UTBA, UTBT a UBEP. Duplicitní a neúplné sady se vylučují; epochy se nepropojují. UBEP je rozhodující pro přiřazení skutečné sady k epoše.

```sh
.venv/bin/python Tools/autotest/utb_bench_analysis.py session.BIN \
  --start-us 10000000 --end-us 70000000 --output session-analysis.json
```

Časové meze musí odpovídat skutečnému měřenému intervalu v daném logu. Full-window throughput používá počet kompletních sad / délku celého okna, včetně mezer. Bez explicitních mezí se hlásí pouze interior frequency a varování; to nezachytí chybějící úvod/konec. Kompletně přepsané flash záznamy nelze rekonstruovat. Po testu neprodleně stáhnout BIN a ověřit jeho úplnost.

## 8. Reprodukovatelné build varianty

Z kořene repozitáře ve WSL, se stávajícím ARM toolchainem v PATH, bez sudo:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
# A0: UTB je zkompilováno, runtime ENABLE=0, BENCH vypnuto
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --disable-UTB_BENCH
./waf copter -j4
# Ihned archivovat build/SkystarsH7HD-bdshot/bin/arducopter* jako A0.

# A/B/C/D: jedna instrumentovaná binárka, odlišné runtime profily
mkdir -p tmp/utb-1ha
printf 'define AP_UTB_BENCH_ENABLED 1\n' > tmp/utb-1ha/bench-on.hwdef
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH \
  --extra-hwdef "$PWD/tmp/utb-1ha/bench-on.hwdef"
./waf copter -j4
# Ihned archivovat jako BENCH_ON, s přesnými parametry A/B/C/D a SHA-256.

# Compile-out kontrola UTB i BENCH
./waf configure --board SkystarsH7HD-bdshot --disable-UTB --disable-UTB_BENCH
./waf copter -j4

# Logging-disabled guard: není firmware pro měření loggeru
printf 'define HAL_LOGGING_ENABLED 0\n' > tmp/utb-1ha/no-logging.hwdef
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH \
  --extra-hwdef "$PWD/tmp/utb-1ha/no-logging.hwdef"
./waf copter -j4

# SITL ON BENCH ON; obdobně přepnout oba enable na disable pro OFF
./waf configure --board sitl --enable-UTB --enable-UTB_BENCH
./waf copter -j4 --targets tests/test_utb,tests/test_utb_bench
build/sitl/tests/test_utb
build/sitl/tests/test_utb_bench
.venv/bin/python Tools/autotest/test_utb_bench.py --binary build/sitl/bin/arducopter --output tmp/bench-sitl
.venv/bin/python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 1 --log-dir tmp/skeleton
.venv/bin/python Tools/autotest/test_utb_shadow.py --binary build/sitl/bin/arducopter --output tmp/shadow

# MicoAir kompatibilní target, samostatný artefakt
./waf configure --board MicoAir743v2 --enable-UTB --enable-UTB_BENCH
./waf copter -j4
```

UTB OFF/BENCH ON SITL se sestaví `--disable-UTB --enable-UTB_BENCH`; pro FÁZI 0 OFF test použít `--compiled 0`. Každá konfigurace přepisuje artefakty stejné desky. K nahrání vždy vybrat archivovaný APJ se správným board targetem a ověřeným SHA-256, nikdy neurčitý poslední soubor z build adresáře. SHA, velikosti, přesné flagy a výsledky jsou v software reportu a manifestu.

## 9. První hardware session — až po samostatném schválení

1. Bez vrtulí, DISARMED, žádné automatické armování; deska bezpečně upevněna. Zajistit vhodné napájení a chlazení. První session provádí obsluha, která může okamžitě vypnout napájení.
2. Zapsat označení PCB/revizi, MCU, IMU orientace a skutečný board ID. Zálohovat parametry a původní firmware. Ověřit hash vybraného APJ a target SkystarsH7HD-bdshot; MicoAir je samostatný test.
3. Ověřit FRAME_CLASS=1 a FRAME_TYPE=12 přímým readbackem FC. BF_X_REV=18 není podporovaný BF_X. Zkontrolovat stávající arming zákazy UTB_ACRO; mód nepoužívat k řízení.
4. Ověřit LOG_BACKEND_TYPE=4 a nalezení W25Q128/BLOCK loggeru. Nastavit LOG_DISARMED=1 a konzistentní původní LOG_BITMASK; zajistit volné místo a stažení logu po každém běhu. A0 nemá UB metriky. BENCH ON musí vykázat UB33=4.
5. Zachovat stejný loop rate, FSTRATE konfiguraci, telemetry stream rates, napájení a teplotní podmínky. Zaznamenat readback všech relevantních parametrů. BENCH nesmí obcházet FSTRATE guard.
6. Pro B_WAIT_US nejprve ponechat 0 (exceed counter vypnutý), získat distribuci a zvolit pozorovací threshold se zdůvodněním proti periodě/main budgetu. Threshold není bezpečnostní záruka.
7. A0 a A měřit nejméně 60 s po ustálení; B stejně. C (200 Hz) nejprve krátký kontrolní běh, potom 60 s pouze při stabilním průběhu. D (400 Hz) zpočátku 10 s, s kontrolou záznamů a zátěže. Tyto délky jsou experimentální okna, nikoli bezpečnostní limity. Pro opakovatelnost nejméně tři běhy, při oteplení střídat pořadí a zaznamenat podmínky.
8. Každý profil začít novým bootem s uloženými parametry; worker vzniká jen při boot ENABLE=1. Změna ENABLE z 0 na 1 bez rebootu jej nevytvoří. Statistiky jsou kumulativní od bootu, nikoli resetované změnou epochy; při C/D přechodu používat delta čítačů a logové časové okno. Profily: A0/A ENABLE=0; B ENABLE=1 SHADOW=0; C ENABLE=1 SHADOW=1 LOG_RATE=200; D totéž LOG_RATE=400. B/C/D vyžadují heartbeat workeru, vytvoření bez selhání a známý ChibiOS watermark. SHADOW aktivovat až po ověření všech guardů.
9. Zachytávat PM, původní scheduler statistiky, UB telemetry a kompletní BIN. Zaznamenat časové hranice epochy a přechody konfigurace. Stáhnout log ještě před přepsáním kruhového flash backendu.
10. Porovnat A0/A overhead, A/B zapnutí UTB a B/C/D logging. Hodnotit histogram/max acquire i hold, UNKNOWN, periodu/jitter, overruns, producer/consumer/export, queue/drops a uloženou frekvenci. Maxima hodnotit se stejnou délkou pozorování; nula potvrzených kolizí neuzavírá riziko.
11. Po každém běhu SHADOW=0, zachovat DISARMED a uložit parametry, BIN, telemetry, analýzu a poznámky. Výsledek přezkoumat před případným delším D testem. Aktivní UTB řízení se nepovoluje.

## 10. Podmínky zastavení a interpretace

Okamžitě zastavit při nečekaném armování/motorovém výstupu, resetu/watchdogu, poruše napájení/chlazení, frame/FSTRATE guard neshodě nebo logger/backend nesouladu. Zastavit při worker failure, ztrátě jeho pravidelného progresu, stack warning/corruption, rostoucím backlogu/drops nebo nedostupnosti potřebných dat: experiment není kontrolovaně pozorovatelný.

Main logger acquire >= nominální main perioda je důvod přerušit experiment a prošetřit: samotné získání zámku spotřebovalo celý nominální rozpočet cyklu. Opakované scheduler overruns nebo podstatné zhoršení PM proti opakované baseline vyžadují zastavení a analýzu. Izolovaná raw perioda těsně nad nominálem může být timer jitter, není automaticky overrun. Absolutní přípustný jitter/CPU/stack limit dosud není na H743 ověřen; nepřidává se libovolné procento jako bezpečnostní certifikace.

Pokud 200/400 Hz není udržitelné, dokumentovat nejvyšší opakovaně ověřenou frekvenci kompletních uložených sad a příčinu omezení. Výsledek bez vrtulí DISARMED nepotvrzuje letovou bezpečnost, worst-case senzory/telemetrii, uzavřenou UTB regulaci ani budoucí aktivní allocator.

## 11. MicoAir743v2 AIO 45A — oddělená kompatibilita

[Výrobce přesného AIO 45A](https://micoair.com/flightcontroller_micoair743v2_aio_45a/) uvádí ArduPilot target MicoAir743v2; místní pinout odpovídá IMU, PWM a SDMMC konfiguraci. Nejde pouze o odhad ze jména MCU. Skutečná dodaná revize, zapojení a orientace zůstávají hardware neověřené.

| Vlastnost | SkystarsH7HD-bdshot | MicoAir743v2 |
|---|---|---|
| MCU/build clock | STM32H743, explicitně 480 MHz | STM32H743, místní build 400 MHz |
| Flash / reserved / param storage | 2 MiB / 128 KiB / 32 KiB | Stejné; RAM map dle místního STM32H743xx |
| Logger | W25Q128 SPI3 BLOCK, typ 4 | microSD SDMMC1 filesystem, typ 1 |
| Instrumentovaný mutex na HW | AP_Logger_Block write_sem | Žádný; filesystem probe pouze SITL |
| IMU | BMI270 SPI1 + SPI4, ROLL180_YAW90 / ROLL180 | BMI088 SPI2 ROLL180_YAW270 + BMI270 SPI3 ROLL180 |
| Barometr | Dle lokálního hwdef | SPL06 I2C2 |
| PWM 1–4 | PB0/PB1 TIM3, PD12/PD13 TIM4 | PE14/PE13/PE11/PE9 TIM1 |
| Další PWM | PA0–3 TIM2, PA8 TIM1 | PB1/PB0 TIM3, PD12/13 TIM4, PE5/6 TIM15; PD14 LED |
| Bidirectional DShot | Existující bdshot target | BIDIR v hwdef TIM1/TIM3/TIM4; ověřit ESC a skutečné kanály |

MicoAir SDMMC: PC12 clock, PD2 command, PC8–11 data; generated FATFS IO je zapnuté. Logger typ 1 zde má smysl, ale současná HW mutex instrumentace BLOCK neplatí. UB33=0 na MicoAir znamená neměřený backend, nikoli nulovou latenci. MicoAir build ověřuje kompatibilitu diagnostiky a její compile guard, nepředstavuje měření filesystem mutexu.

Kapacita MCU uváděná výrobcem není build clock; zde clock neměnit na 480 MHz bez samostatného ověření. Místní hwdef current scale 40.2 a OSD_TYPE=1 se liší od doporučení výrobce pro AIO (14.14, MSP OSD_TYPE=5). Po převzetí desky je nutná konfigurace/kalibrace podle skutečné revize, nikoli automatické převzetí parametrů Skystars. Přesný bootloader/board ID a motor mapping ověřit před použitím. HAL, DShot ani kalibrační defaulty tento patch nemění.

## 12. Checkpoint, software evidence a hranice scope

Před C++ změnami byl ověřen reprodukovatelný checkpoint `tmp/utb-1ha-checkpoint/workspace.tar.gz`, 7460 tracked/untracked souborů, 307731958 B, SHA-256 `92bcd530009062832ab8595cbfe43b80c7da795d953668ec6ef8f1a52efae51c`. Manifest a tracked.patch jsou vedle archivu. Ignorované build/runtime výstupy nejsou zdrojovým checkpointem; submoduly se neměnily.

Výsledky před/po instrumentaci, finální souborový manifest a firmware hashe jsou v [software reportu](UTB_BENCH_SOFTWARE_REPORT_CS.md). Navrženo a implementováno jsou zde odděleny od skutečných výsledků testů v reportu. Hardware latence, PI dopad, CPU load, watermark, deadline rezerva a udržitelnost 200/400 Hz na H743 zůstávají neověřené.

Patch nemění UTB PID/filtr/anti-windup/reference/mixer, arming, AP_Motors, motor setters, HAL/DShot, EKF ani FSTRATE pravidla. Existující AP controller se nevolá podruhé. Frontend AP_Logger synchronizace a neměřené flash/SPI zámky zůstávají samostatná rizika; tento benchmark není důkazem celkové thread safety loggeru. Další architektura ani FÁZE 2 není součástí této implementace.
