# FÁZE 1H-D: offline audit startupu, downloadu a EKF

Stav 12. 10. 2026. Větev `test/skystars-5inch`, auditovaný HEAD
`687f825d228b12a64d36651072aed19356178217`. Firmware v archivovaných short/full
BIN je `f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f`, APJ SHA-256
`a45f3c9f08e21e8b713cfe59e3fe404fe4e866164c6c21f1a00edd0876622878`.
Mezi těmito commity se produkční C++ nezměnilo; liší se host nástroje/test a dokumentace.

**Implementováno a offline ověřeno:** nový analyzátor archivů a čistý host transfer
prototyp s falešným protějškem. **Navrženo:** změny postupu a případná diagnostika.
**Nový hardware test ani oprava produkčního firmwaru neprovedeny.** COM porty nebyly otevřeny.

Závěr: existuje konkrétní synchronní cesta downloadu přes hlavní vlákno.
Periody při downloadu skutečně dosáhly 41,932/43,499 ms. Nezdravé EKF bylo při
full downloadu hlášeno již během přenosu. Přesný interní důvod `healthy()==false`
a podíl jednotlivých čekání nejsou z archivů určitelné. Zvětšování UTB fronty,
změna priority workeru ani úprava EKF z těchto dat nevyplývají.

## Evidence a reprodukce

Původní archiv: `/home/jirka/ardupilot_utb/tmp/utb-synthetic-h743-20261012_150527/`.
Nové výsledky: `/home/jirka/ardupilot_utb/tmp/utb-1hd-offline-20261012_162639/`.
`input-manifest.json` zaznamenává velikosti a SHA-256 všech 92 původních souborů.
`download-audit/evidence.json` obsahuje parsing všech 15 tlogů, tří BIN,
host markerů a úplný manifest vstupů. Původní soubory jsou zachované.

| BIN | Bajty | SHA-256 |
| --- | ---: | --- |
| before-upload-log-1.bin | 7654746 | ed08afb62c8dc575353dda96dfe649655178f9d88aca9ea2bfef4f1307a533be |
| SYNTHETIC_SHORT.bin | 2569496 | d45167bd5832efb08cf2b0350fd1a9cd29388a48828aa4ffa0b2bd0bc63f040e |
| SYNTHETIC_FULL_400.bin | 4349246 | 0677617bfac02af3f3b44d0038bb614ed147e1eeb3812d0b96b80d6e565786b6 |

První BIN je historický firmware `acf04efa`, nikoli nenulový PID experiment.
Short/full MSG identifikují `f9cc3bcc`, ChibiOS `d103983f`, správnou desku a RC Protocol None.
Předchozí reporty byly použity jako referenční výsledky, nikoli jako náhrada parsingu.

```sh
cd /home/jirka/ardupilot_utb
OUT="tmp/utb-1hd-replay-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
.venv/bin/python -B Tools/autotest/utb_download_audit.py \
  --archive tmp/utb-synthetic-h743-20261012_150527 --output "$OUT/download-audit"
UTB_AUDIT_EVIDENCE="$OUT/download-audit/evidence.json" .venv/bin/python -B \
  -m unittest discover -s Tools/autotest/unittest -p test_utb_offline.py -v
```

Nástroj přijímá pouze existující regulérní `.tlog`/`.bin` soubory a otevírá jejich
absolutní cesty přes file reader pymavlink. Neobsahuje připojení k zařízení.
TransferPlan není hotový downloader; pouze deterministicky plánuje a skládá bajty
v paměti, testovaný výhradně proti falešnému protějšku.

## Časová základna a přesnost

- **HOST UTC:** JSON `start/end/events.wall` a přijímací `_timestamp` z tlogu.
  Níže všechny časy jsou UTC, místní čas byl UTC+2.
- **FC boot:** MAVLink `time_boot_ms`; **DataFlash TimeUS:** monotónní FC čas
  příslušného producenta. BENCH čítače jsou kumulativní od bootu.
- `host_timestamp - time_boot_ms/1000` je přijímací odhad boot originu. Zahrnuje
  UART/USB buffering, plánování zpráv a host latenci. Rozsahy a první/poslední
  kotvy každého tlogu jsou uložené v evidence.json. Nevytvářejí přesnou synchronizaci.
- Tyto tlogy neobsahují odeslané `LOG_REQUEST_*` pakety. Jejich přesné okamžiky
  **NEPROKÁZANÉ**. JSON intervaly, LOG_ENTRY/DATA a zdroj host skriptů prokazují
  typ operace, ne čas každého requestu nebo přesný okamžik uvolnění mutexu.
- BIN při downloadu už nezaznamenává vývoj EKF. Velká perioda v UB29 je maximum
  z minulosti: čas jejího exportu není čas konkrétního zablokování.

## Rekonstruovaná časová osa — LOG VERIFIED

| Událost a zdroj | Host UTC, 12. 10. 2026 | FC údaj / meze závěru |
| --- | --- | --- |
| Původní preflight, preflight.tlog | 13:06:36–13:06:53; EKF 13:06:43.916 | FC 1783570–1800590 ms; flags 167, bootcount 71 |
| Archivace původního logu, before-upload-log-download.tlog | DATA 13:09:08.574–13:10:07.352 | 7654746 B; max UB29 32268 µs; EKF zprávy chybí, zdraví při této archivaci neurčeno |
| Bootloader reboot, upload-result.json | 13:13:42.288 | Přechod bootcount 71→72; samostatný upload, nikoli důkaz samovolného restartu |
| Barometr / AHRS / EKF inicializace, upload-after.tlog | baro 13:14:18.456; DCM 13:14:19.100; EKF3 IMU0/1 13:14:21.100; tilt 13:14:22.491 | První kotva 13:14:20.129, FC 5804 ms; přesný elektrický boot není zaznamenán |
| Post-upload read-only validace | EKF 13:16:06.018 | flags 167; bootcount 72 |
| Short state probe, short/capture.json | SHADOW=1 13:21:40.103, LOG_DISARMED=1 13:21:41.120 | Přibližně 1,017 s SHADOW při LOG OFF; UTBS flags7/calc0/obs2/policy1 |
| Short startup, short/capture.tlog | do 13:21:42.252 | Drop 0→404, Missing 0→96; UB29 2580→6546 µs; BIN DSF.Dp124 |
| Short PREROLL / RUNNING, BIN UTBX | host start request 13:21:56.269 | TimeUS 462306871 / 463306871; 2011 platných RUNNING sad, cykly0–2010; přerušeno hostem |
| Short freeze, short/capture.json | B_SYNTH0 13:22:02.291, SHADOW0 13:22:02.393, LOG0 13:22:02.497 | Vrácení gains a GCS failsafe; přesný flash flush neprokázán |
| Short download, short-download.tlog/json | DATA 13:23:59.635–13:24:02.960, chyba 13:24:03.222 | 425880 B; UB29 6546→41932 µs poprvé exportováno 13:24:00.294 (FC586374ms) |
| Obnovení transportu, after-transport.tlog | 13:26:25.809–13:26:34.330 | FC731511–740520ms, bootcount72, EKF167 v13:26:26.923: link chyba neznamenala restart ani okamžitý trvalý EKF výpadek |
| Dotažení stejného short logu, short-download-recovery.tlog/json | DATA 13:29:33.018–13:29:55.670 | 2143616 B, součet2569496; bootcount72 před/po; zdraví během transferu nezaznamenáno |
| Zastavený další capture, full/capture.tlog/json | EKF1024 v13:32:56.720 | FC1123109ms; events=[]: žádný PARAM_SET v tomto pokusu |
| Potvrzený fyzický coldboot a preflight | EKF167 v13:35:45.728 | bootcount73; preflight FC61871–79889ms; doba inicializace EKF po tomto bootu v tlogu chybí |
| Full state probe, full-after-coldboot/capture.json | SHADOW1 13:37:36.396, LOG1 13:37:37.410 | Přibližně1,014s SHADOW/LOGOFF; actual state gate flags7/calc0/obs2/policy1 |
| Full startup, capture.tlog a BIN | čítače ustálené do13:37:38.587 | Drop402, Missing130, DSF.Dp270; maxloop6011µs už před RUNNING |
| Logger warmup / zahájení syntetiky | B_SYNTH1 13:37:52.597 | LOGON→request přibližně15,187s; připravenost ověřená následným bezeztrátovým oknem |
| Full PREROLL / RUNNING / COMPLETE, BIN UTBX | přesná host synchronizace NEPROKÁZANÁ | TimeUS197037671 /198037670 /218037671; COMPLETE Cycle8000; 8000sad, Seq6884–14883 krok1 |
| Full freeze, capture.json | B0 13:38:13.625, SHADOW0 13:38:13.777, LOG0 13:38:13.879 | COMPLETE přirozeně před freeze; závěr capture13:38:15.878 |
| Full download předběžný stav | 13:42:23.948 | FC468293ms, UB29 stále6011µs, UB28(worker hold)630µs |
| LOG_ENTRY/metadata a první velká perioda | UB29=24312 v13:42:27.691 | FC472297ms; před prvním přijatým DATA; nelze rozlišit listing od open/boundary čtení |
| Full DATA transfer | 13:42:27.743–13:43:13.739 | 4349246 B; paced8190B/20ms; bootcount73 před/po |
| Perioda během transferu | UB29=43059 v13:42:29.687; konečné43499 v13:43:01.712 | Poslední export FC506329ms; neznáme právě nejdelší handler ani mutex |
| EKF problém již za transferu | 13:42:49.277 STATUSTEXT | „EKF attitude is bad“, „AHRS: EKF3 core 0 unhealthy“; EKF_STATUS_REPORT v tomto tlogu chybí |
| Následné úplné readback | EKF1024 v13:47:03.663 | FC740564–755582ms, UB29 stále43499µs, bootcount73 |
| Potvrzený další fyzický coldboot, final-coldboot-readback.tlog | EKF167 v13:55:59.919 | bootcount74; FC46870–61887ms; maxloop2561µs; parametry obnovené, DISARMED |

Coldboot potvrzení jsou lidské JSON markery. Úplné vypnutí napájení nebylo
elektricky měřeno. Jeho účinek podporuje změna bootcount a menší boot timestamp.
Na zdravý stav mezi short recovery a zastaveným full capture není souvislé
měření; přesný přechod 167→1024 v tomto intervalu je **NEPROKÁZANÝ**.

## Synchronní download cesta — SOURCE VERIFIED

Číslování následujících cest odpovídá uvedenému HEAD a firmware C++ `f9cc3bcc`.
Kořen cest je `/home/jirka/ardupilot_utb/`.

Klikatelné rozhodující source cesty: [MAVLink synchronní send:215](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp:215>), [Block data a mutex:427](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Block.cpp:427>), [Flash WaitReady:92](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp:92>), [Backend worker startup guard:471](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Backend.cpp:471>), [SPI dokončení/timeout:204](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/SPIDevice.cpp:204>), [ChibiOS priority inheritance:158](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/modules/ChibiOS/os/rt/src/chmtx.c:158>), [EKF healthy:9](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp:9>), [EKF initialized status:775](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp:775>), [UTB evaluate:272](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_UTB/AP_UTB.cpp:272>), [Worker logger dostupnost:156](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/ArduCopter/utb.cpp:156>).


| Krok | Zdroj a řádek | Skutečná operace |
| --- | --- | --- |
| GCS receive/send v main | ArduCopter/Copter.cpp:213, :214; libraries/GCS_MAVLink/GCS_Common.cpp:4398, :1568 | 400Hz scheduled GCS handler; receive logger request a update_send→handle_log_send |
| Povolení downloadu | libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp:34 | WritesEnabled a DISARMED guard, žádný bypass armování |
| Listing | stejný soubor:68 | `_log_send_sem`, get_num_logs, následně get_log_info pro položku |
| Otevření / změna logu | stejný soubor:107 | `_log_send_sem`, get_num_logs→get_log_info→get_log_boundaries, synchronně před data smyčkou |
| Datová dávka | stejný soubor:215, :296 | USB high bandwidth a usb_connected: až250×90B =22500B/call; omezeno TXspace/heartbeat/zbývajícími daty |
| Block raw čtení | libraries/AP_Logger/AP_Logger_Block.cpp:461, :427 | Block `sem`, StartRead, ReadBlock, až přes hranici payload stránky |
| Počet / hranice logů | stejný soubor:488, :602, :669, :715, :770 | Opakované binární hledání stránek; find_last_page/start/end pod stejným rekurzivním sem |
| Převod list ID | libraries/AP_Logger/AP_Logger_Backend.cpp:634, :651 | oldest log cache; netvrdit, že každý90B paket vždy provede celý rescan |
| Skutečné čtení flash | libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp:224 | Cache jedné stránky; jinak WaitReady→dev_sem→command+256B SPI read |

Standardní backend je **BLOCK/Flash_JEDEC**, LOG_BACKEND_TYPE=4, LOG_REPLAY=0.
Deska definuje DataFlash a SPI3 W25Q128 v
`libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD/hwdef.dat:29, :58, :77`.
IMU mají jiné sběrnice SPI1 a SPI4 (`:45, :64, :75, :78`); přímé sdílení SPI3
semaphore s gyro zde není. Sdílené CPU, IRQ nebo DMA nelze z tohoto faktu vyloučit.
Konfigurace bdshot obsahuje SPI3 DMA_NOSHARE, což není důkaz nulové IRQ/CPU interference.

**Důležitá vlastnost chunkování:** po dokončení `packet.count` volá
`handle_log_send_data():339` end_log_transfer→IDLE. Následující chunk potom
opět splňuje open podmínku v `handle_log_request_data():126`. I stejný ID může
vyvolat get_num_logs/info/boundaries znovu. Menší chunky omezí datový burst,
ale mohou zvýšit počet metadata hledání. U předchozího8190B requestu je
horní datová dávka91 paketů, u prototype1440B16 paketů; metadata část zůstává
synchronní. Nelze je vydávat za prokázanou opravu.
Nové requesty na stejné lince během aktivního transferu se na `:111` ignorují;
agresivní gap requests tedy nejsou fronta spolehlivě zpracovaných příkazů.

## Zámky, čekání a priority

| Prostředek | Vlastník/cesta | Co umí současný benchmark |
| --- | --- | --- |
| `_log_send_sem` | Listing/open/data/END, rekurzivně držen přes transfer handler | Neměří tuto latenci ani držení |
| Block `sem` | Flash staging/read cache, metadata, io_timer a write_log_page | Neměří tuto latenci ani držení |
| `dev_sem` = SPI3 bus semaphore | ReadStatusReg, page read/write/erase příkazy | Neměří tuto latenci ani držení |
| `write_sem` | RAM admission `_WritePrioritisedBlock` | Instrumentováno MutexProbe: main acquire / worker hold; není to předchozí trojice |
| HAL DMA lock / dokončení SPI | SPI driver interně | Neměřeno UBMT logger probe |

Block `io_timer():836` má při flush i běžném zápisu nejvýše jednu stránku na
pass, ale `FinishWrite():89` může na hranici bloku spustit erase; při recovery
`io_timer():864` drží Block sem přes více erase operací. Nejde o horní limit
doby v µs. RAW/read metadata jej mohou získat ze main i čekat na log_io.
`write_sem` kolem RAM bufferu není důkaz, že flash backend nikdy blokuje main.

JEDEC `WaitReady():92–107` opakuje Busy(), `delay_microseconds(100)` a po
více než5000ms nastaví flash_died. Busy() zahrnuje WIP i SRP0 (`:189`), nikoli
jen samotný zápis. Každý ReadStatusReg (`:180`) bere a uvolňuje dev_sem.
WaitReady nevlastní dev_sem nepřetržitě, ale volající může stále vlastnit Block
sem a/nebo `_log_send_sem`. Pět sekund je timeout této smyčky, **není garantovaný
horní limit celé operace**: akvizice mutexu nemá zde timeout, SPI čekání a
scheduling přidávají čas. Chip program/erase busy doběhnutí není změřené.

`libraries/AP_HAL_ChibiOS/SPIDevice.cpp:289, :336` vrací bus semaphore;
`do_transfer():204–216` čeká na IRQ/DMA přes suspend s timeoutem
`20000 + 32*len` µs, tj. např.28192µs pro256B. Transfer chyby nemusí dát
`spi_fail` během expected delay. Runtime SPI bitrate a skutečné časy nejsou
v archivech změřené; nelze dělit velikost logu nominálními32MHz a považovat
výsledek za dobu držení mutexu.

ChibiOS Semaphore používá chMtxLock/chMtxUnlock
(`libraries/AP_HAL_ChibiOS/Semaphores.cpp:39, :46`), rekurze je povolená
(`hwdef/common/chconf.h:315`). Priority inheritance je v
`modules/ChibiOS/os/rt/src/chmtx.c:132–216`; vlastník se dočasně zvýší podle
čekatele, včetně řetězu mutexů. Main180, UTB worker58, log_io59
(`Scheduler.h:26, :34`; `AP_Logger.cpp:1501`; `ArduCopter/utb.cpp:127`).
PI omezuje klasickou inversi způsobenou mezilehlým CPU vláknem, neodstraňuje
čekání na flash, SPI IRQ ani dlouhou kritickou sekci. Main může dělat mnoho
operací ve vysoké prioritě, jiné nízkoprioritní práce mohou být zpožděné.
Nalezená blokující cesta není důkaz deadlocku; v datech není prokázaný deadlock.

## Scheduler, INS/AHRS a význam EKF flagu

SOURCE VERIFIED: fast úlohy v `ArduCopter/Copter.cpp:116–139` mají pořadí
INS→AP rate→AP motors→UTB shadow→read_AHRS→flight mode. `read_AHRS():886`
volá `ahrs.update(true)`. Při synchronním GCS/flash handleru tyto main úlohy
neběží. INS backend/timer/DMA může dále sbírat data; toto není totéž jako
pravidelný INS frontend update a EKF prediction/fusion.

`AP_InertialSensor.cpp:2013–2062` po větším zpoždění znovu nastaví příští
sample, `:2129` měří skutečné dt. EKF3 v
`AP_NavEKF3_Measurements.cpp:402–522` používá delta angle/velocity, FIFO,
downsampling a omezenou aktualizaci dtEkfAvg. Zpoždění může poškodit předpoklady
sample/fusion časování, ale kód neobsahuje jednoduché pravidlo
„43ms download automaticky znamená UNINITIALIZED“.

`EXPECT_DELAY_MS(5000)` v send handleru chrání očekávaný pomalý download:
`libraries/AP_HAL/Scheduler.cpp:30–38`, ChibiOS `Scheduler.cpp:745–777, :366`.
Timer může pat watchdog, main boost se ruší. **Nevolá znovu rate controller,
INS nebo AHRS a neudržuje400Hz regulaci.** V této větvi ExpectDelay sám
nevolá PerfInfo.ignore_this_loop; tvrzení, že tento wrapper automaticky maskuje
PM.NLon, by bylo nesprávné. PM obecně umí ignorovat označený loop
(`AP_Scheduler/PerfInfo.cpp:113–130`), ale download PM zde nemáme v BIN.
PM.NLon0 ze stabilního RUNNING nic nedokazuje o download session.

SOURCE VERIFIED, přesný význam: `AP_AHRS.cpp:2956` vybírá **configured**
EKF, nikoli pouze active fallback backend. EKF3→core send_status_report
(`AP_NavEKF3.cpp:1911`) při `!filterStatus.flags.initalized` přidá1024
(`AP_NavEKF3_Outputs.cpp:591–627`). updateFilterStatus vytváří nový status=0
a nastaví initialized podle `healthy()` (`AP_NavEKF3_Control.cpp:775–822`).
Nejde o trvale latched „jednou se inicializoval“ bit.

`NavEKF3_core::healthy()` (`AP_NavEKF3_Outputs.cpp:9–32`) selže při:

- Filter fault: NaN quaternion/velocity, špatně podmíněné fúze, statesInitialised=false
  (`:571–580`).
- Současně velkých vel/pos/height innovation test ratios.
- Méně než1s od ekfStartTime.
- OnGround/AID_NONE a horizontální innovace²>1 nebo filtrovaná height innovace>|1|.

Kód má také DISARMED reset po5s nulového filterStatus
(`AP_NavEKF3_core.cpp:704–723`). V archivech není zachycena zpráva forced reset
ani aktuální fault bitmap/height innovation z downloadu. Žádný z uvedených
důvodů proto nelze označit za konkrétní příčinu. Předchozí zdravá EKF a shodný
bootcount vylučují interpretaci „po uploadu se nikdy neinicializovalo“;
nevylučují vnitřní reset některého jádra.

LOG_REPLAY=0 znamená, že logger allow_start_ekf tento experiment nebrzdí
(`AP_Logger.cpp:1078–1090`); vypnutý LOG_DISARMED není sám o sobě zákazem EKF.

## Windows chyba a logové hranice

LOG VERIFIED: short download skončil ClearCommError PermissionError13,
native22 „Zařízení nezná tento příkaz“. Recovered prefix425880B a následný
transfer2143616B dávají celý nezměněný log a jeho ověřený SHA. Uptime
pokračoval, bootcount72 zůstal a následná read-only EKF byla167. Chyba se stala
na host serial API; není důkaz resetu, SPI failure, EKF resetu nebo deadlocku.
Host/USB buffering, Windows ovladač, napájení či link stav zůstávají UNVERIFIED.

LOG_DISARMED=0 a SHADOW=0 jsou freeze produkce, nikoli garantovaný fsync.
Block log_io dopisuje celé250B payload stránky. Async stop flush je samostatná
cesta (`AP_Logger_Block.cpp:537, :890`; `AP_Logger_Backend.cpp:70`) a
LOG_FILE_DSRMROT=0 v této konfiguraci nedává automatickou rotaci při freeze.
LOG_ENTRY size se musí před resume znovu ověřit; size/time_utc/id změna ruší
identitu transferu. V předchozím experimentu je COMPLETE a všech8000 sad
skutečně v BIN; obecná tail durability budoucí session z toho nevyplývá.

## Startup: výpočet versus pozorovatelnost

SOURCE VERIFIED: producent vybere log sample i při logger unavailable a
publikuje do fronty kapacity4 (`AP_UTB.cpp:430–458, :518–534, :203–225`).
Worker nepopuje data, pokud
`logger.should_log && logging_present && logging_started` není true
(`ArduCopter/utb.cpp:140–174`). LOG_DISARMED=0 na DISARMED FC tedy vytváří
probe se čtyřmi čekajícími snapshoty a další selected samples mají Drop.
Snapshot status se přenáší nezávisle best-effort, worker přitom může být RUNNING.
Worker lifecycle RUNNING neznamená backend připravený na všechny zápisy.

Před první dostupností loggeru má evaluate policy EXPERIMENT_SUPPRESSED a
reset controlleru (`AP_UTB.cpp:324–328`). State health je vyhodnocen zvlášť;
flags7 při probe prokazují zdravý skutečný state, ne vykonaný PID. Od zahájení
experimentu výpadek logování sám nemění PID matematiku; Drop/Missing jsou
pozorovatelnost a anti-windup feedback pochází z mixeru, nikoli z loggeru.
BIN navíc před syntetikou ukazuje Calc4=REFERENCE_INVALID při odpojeném RC;
po jejím zapnutí Calc1=PRIMING a potom Calc0/flags1023. To jsou správné
reference/priming guards, nikoli matematická chyba způsobená Drop/Missing.
Time-budget, state/reference a synthetic eligible guards mohou samostatně
resetovat/abortovat; nesmí být ztotožněny s queue loss.

Po LOGON může worker vidět logging_started, ale backend odmítne worker zápis,
dokud main nedokončí startup message writer:
`AP_Logger_Backend.cpp:471–483`. False se počítá jako Missing přes
`ArduCopter/utb.cpp:178–241` a `AP_UTB.cpp:550–557`. Missing je počet selhaných
sad/pokusů, DSF.Dp jsou dropy backend zpráv; různá granularity a odmítnutí před
RAM admission znamenají, že se nemusí rovnat. Packed UTBC sdružuje sadu,
UTBS/UTBX/benchmark mají další samostatné zápisy.

LOG VERIFIED: čítače už před prvním RUNNING snapshotem dosáhly:

| Boot/session | Drop | Missing | DSF.Dp | Main period max | RUNNING nové ztráty |
| --- | ---: | ---: | ---: | ---: | --- |
| Short | 404 | 96 | 124 | 6546µs | Žádné podle uložených čítačů; 2011sad Seq krok1 |
| Full | 402 | 130 | 270 | 6011µs | Žádné podle uložených čítačů; 8000sad Seq krok1 |

Fronta4 při1,014s probe na400Hz vysvětluje řádově400 dropů. Přesný per-cycle
rozklad před dostupností prvního BIN záznamu není zachycen. Počty a časy
podporují source cestu, nelze každý startup Missing připsat jednomu konkrétnímu
backend return. Dynamické zahájení logu/FMT/param writer a sdílené backend
práce jsou kandidáti startup6011µs; jejich podíl není změřený.

Matematická shoda7999 full kroků z předchozího reportu se nemění. Nová původní
regrese30946 vektorů/29 scénářů a replay75226 simulačních kroků znovu prošly.
Tyto offline kontroly neopravují ani nevysvětlují EKF za downloadu.

## Síla závěrů

| Označení | Závěr |
| --- | --- |
| SOURCE VERIFIED | Main provádí synchronní logger metadata/data/SPI cestu, může čekat na mutex/ready/IRQ a obsloužit velkou dávku |
| LOG VERIFIED | Main max při short41932µs/full43499µs vzrostl během transfer session; u full byl unhealthy text uvnitř DATA intervalu |
| STRONG HYPOTHESIS | Synchronní download práce je hlavní vysvětlení dlouhých period; její časové výpadky mohou spouštět EKF health problém |
| UNVERIFIED | Rozdělení41–43ms mezi metadata/read/mutex/SPI/IRQ; konkrétní EKF fault/innovation/reset; USB/driver root cause; dlouhodobé worst-case |

Nulové potvrzené MAIN/UTB worker kolize a nepřibývající UB28 při transferu
nevylučují BLOCK/SPI čekání: měřený MutexProbe je jiný zámek. Ze známých dat
nevyplývá nutnost měnit UTB RateController, MotorMixer nebo motorovou izolaci.

## Nejvýše tři návrhy, podle přínosu a rizika

1. **Host postup a oddělené transfer session — první volba.** Capture ukončit,
   ověřit uložený COMPLETE/count a identitu logu, odděleně jednou listovat,
   mít právě jeden outstanding request, konečné retry/timeout a heartbeat.
   Po transferu ověřit health/bootcount; bez zdravého návratu nezačínat další
   měření. Uložit odeslané requesty i přijímané zprávy s monotónním host časem.
   `TransferPlan` je offline ověřený základ byte/identity/retry části; doporučený
   chunk1440 je pouze experimentální počáteční volba. Otestovat např.1440/8190
   proti stejnému malému archivovanému logu a měřit UB29/health. Menší chunk
   není prokázaná náprava a může zvýšit metadata overhead. Na FC zatím neaplikováno.
2. **Startup pořadí na hostu.** Validity probe zkrátit nebo provozovat při již
   připraveném loggeru; umožnit ukončení probe, pak LOGON/warmup, ověřit čítače
   a dostupnost platných sad, teprve potom syntetický RUNNING. Pokud je nutný
   LOGOFF probe, označit jeho očekávané Drop odděleně a nezaměňovat je se ztrátou
   RUNNING. Riziko: změna startup sekvence a případný nový log; nejprve mock test.
   Produkční potlačení enqueuing při LOGOFF je jen případný pozdější návrh po
   schválení a ověření epoch/status semantics, nikoli změna provedená zde.
3. **Cílený oddělený download diagnostický build, jen pokud host pokus nestačí.**
   Po reprodukci a schválení měřit dobu LIST/open/data handleru, Block sem wait/hold,
   WaitReady a počet stránek/status polls; přidat read-only EKF faults/core/
   innovation a IMU sample-gap detail. Pevné counters, žádný log zápis pod
   měřeným zámkem, export mimo měřený handler a oddělená BENCH varianta.
   Neměnit AP logger politiku, priority, EKF ani safety. Teprve důkaz rozhodne,
   zda omezit synchronní dávku/cache metadata nebo upravit jinou konkrétní cestu.

## Nejmenší další hardware ověření — pouze návrh

Před samostatným schválením se žádná položka nevykonává. DISARMED, bez vrtulí,
beze změny PID/motor/arming/EKF parametrů; zdravý FC start. Současná konzervativní
flash rezerva1,66MB nestačí automaticky pro další full capture. Použít existující
neměnný malý log, bez mazání a bez nového syntetického capture.

Nejprve60s baseline read-only UB29, EKF_STATUS_REPORT, SYS_STATUS a bootcount,
pak právě jeden LOG_REQUEST_LIST a krátký datový úsek s konečným limitem bajtů.
Zaznamenávat request-send monotónní/UTC a receive čas, requestID/offset/count,
UB29 a zdraví. Po každém úseku nechat main vrátit do běžných period a ověřit
health; neprovádět souběžně velký parameter fetch či opakované listování.

Stop: první EKF attitude unhealthy/1024, neočekávaný restart/ARM stav,
transport error, flash_died/SPI error nebo neobnovený baseline loop režim.
Žádný automatický reboot, opakovaný capture, heap exhaustion ani obejití health.
Číselný povolený stall limit není z této analýzy bezpečnostně kvalifikován;
41–43ms už je prokazatelný výpadek400Hz service a důvod pokus přerušit a analyzovat.

## Ověření a manifest této fáze

34 nových offline unit/regresních testů PASS: transfer8, fyzikální model12,
uzavřená smyčka9, skutečné archivní markery5. Testy zahrnují zdraví při transferu,
startup counters před RUNNING, COMPLETE8000 a byte součet recovery.
Čísla a členění jsou doložené `unit-confirmed.log`; nejde o nové hardware testy.
Flake8 PASS. Původní raw manifest beze změn; git HEAD a produkční C++ zachované.
Konečné ověření/checksums jsou v `verification-final.json` a `output-manifest.json`.

Přidané soubory společné oběma reportům:

- `Tools/autotest/utb_download_audit.py` — parsing a čistý transfer planner.
- `Tools/autotest/utb_closed_loop.py` — samostatný rotační model a simulační/sweep CLI.
- `Tools/autotest/utb_closed_loop_plots.py` — vědecké SVG grafy.
- `Tools/autotest/utb_closed_loop_replay.py` — replay existujícího C++ test runneru.
- `Tools/autotest/unittest/test_utb_offline.py` — offline testy a archivní regrese.
- `docs/UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md` — tento report.
- `docs/UTB_CLOSED_LOOP_MODEL_CS.md` — model, výsledky a fyzikální omezení.

Žádná závislost nebyla instalována. Commit, push, release, upload ani hardware
operace nebyly provedené.
