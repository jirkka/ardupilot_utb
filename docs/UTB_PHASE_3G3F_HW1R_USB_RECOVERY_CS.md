# FÁZE 3G-3F-HW-1R – USB komunikace a post-upload kvalifikace

**PASS pro omezenou read-only kvalifikaci již nahraného EXEC ON firmware. GO pouze k přípravě samostatně schvalovaného HW-2A; žádný RUN nebyl spuštěn.**

Datum25.10.2026, časy v Europe/Prague (CET, UTC+1). Běží skutečná Skystars H743 na COM5, Copter4.7.1/11a518e5, boot115. Žádný nový upload, softwarový reboot, ARM, motorový příkaz, PARAM_SET, změna intervalů, logger start/erase nebo instalace. Původní report HW-1 ani jeho chyby se nepřepisovaly.

Evidence: `tmp/utb-3g3f-hw1r-20261025/`. Strojový souhrn `summary.json`, podrobná analýza `analysis.json`, raw data a úplné zálohy v `qualification/`, všechny hashe v `evidence-sha256.json`.

## Původní problém a nové informace

HW-1 provedla právě jeden schválený upload kandidáta, exit0, standardní CRC Verify100% a uploader reboot. Následná USB identita chyběla před otevřením portu; aplikace tehdy zůstala neověřená. Původní důkazy: `tmp/utb-3g3f-hw1-20261025_offline-01/upload/`, `upload-console.txt`, `post-upload/result.json`, `post-console.txt` a `docs/UTB_PHASE_3G3F_HW1_FIRMWARE_VALIDATION_CS.md`. Celý jejich původní hashovaný manifest byl znovu ověřen, žádná náhradní verze nevznikla.

Obsluha popsala opakovanou zkušenost s mizejícím COM po uploadu/restartu a obnovením fyzickým USB reconnectem. Je to provozní zkušenost, **nikoliv prokázaná příčina posledního výpadku**. Už původní HW-1 nedoložila chybu firmware; skončila INCONCLUSIVE.

## Fyzický reconnect a Windows

- První pasivní PnP snapshot: **2026-10-25T18:42:36.244301+01:00**. Správná FC už byla přítomná jako ArduPilot COM5,1209:5741/serial200024001251303433333336, statusOK.
- Potvrzení obsluhy přijato **2026-10-25T18:44:18.083311+01:00**: „USB jsem nyní odpojil a připojil; všechny podmínky platí“. Potvrzen právě jeden fyzický krok v tomto protokolu, pouze USB, bez LiPo/vrtulí, FC bezpečně položená a sériové aplikace zavřené. Přesný okamžik odpojení/připojení nebyl nezávisle měřen; timestamp je přijetí potvrzení, nikoliv elektrická hrana napájení.
- Po potvrzení jeden PnP snapshot **2026-10-25T18:44:19.230829+01:00**, opět správná COM5/VID/PID/serial, žádné DFU nebo neznámé USB zařízení v relevantním výběru. Ostatní COM6/COM7 patří Bluetooth.
- Pozdější jednorázový metadata-only helper po zavření spojení opět našel stejnou FC na COM5 (`usb-helper-live.json`); neotevřel sériový port.

**Důležitá kauzalita:** protože správná FC byla viditelná již před novým potvrzeným reconnectem, toto není řízený absent→present důkaz, že právě tento krok obnovil USB. Doba ani příčina návratu od konce HW-1 do prvního snapshotu HW-1R nebyly pozorované. Žádný firmware FAIL se samotnému zmizení COM nepřisuzuje. Existující Windows události jsou uchované; chybějící událost není důkaz absence krátkého USB výpadku.

## Identita, firmware a boot

| Veličina | Skutečný údaj |
|---|---|
| Port / USB VID:PID | COM5 /1209:5741 |
| USB serial / UID2 prvních12B |200024001251303433333336 |
| MAVLink source / board |1/1 /1075 |
| AUTOPILOT_VERSION |Copter4.7.1/11a518e5 |
| STAT_BOOTCNT |115 na začátku i v úplné finální záloze |
| SYSTEM_TIME zachycené první/poslední |139486→206127 ms, bez poklesu |
| První/poslední RX |2026-10-25T18:45:50.173063+01:00 /2026-10-25T18:47:06.026063+01:00 |
| Řízení spojení |115200,DTR=False a RTS=False před open, autoreconnectFalse |

Embedded git odpovídá očekávanému source HEAD, ale není kryptografickým identifikátorem celého dirty buildu. Znovu ověřený APJ SHA-256:

`dae4f020db98a509b68c71a5b8702dfc7019ed762f69b7d46c8cbf3ae5e9d497`

Kombinace původního jediného uploadu/CRC, původního14souborového source manifestu, očekávaného AUTOPILOT_VERSION/board/UID a skutečně nového Execution SYS rozhraní významně dokládá běh očekávané aplikace. **Není to SHA-256 readback interní flash.**

Boot112 byl naposledy přečten před HW-1 uploadem. Aktuálně115; mezilehlé hodnoty113/114 nebyly přečtené. Standardní upload reboot a potvrzený fyzický reconnect jsou očekávané přechody, ale úplnou starší historii jednotlivých nárůstů nelze zrekonstruovat. Během nynějšího spojení není pozorován restart, boot115 je stabilní a uptime monotónní. Neodvozuje se automaticky chybový restart jen z rozdílu115−112.

## Read-only protokol, stabilita a health

Celé jediné otevření trvalo **77.609s**. Před otevřením ještě30s pasivní čekání na ustálení inicializace. Po identitě40s konzervativní health pozorování; pak sekvenční paměť, úplná záloha, požadované SYS soubory, další paměť, jeden inventář, finální úplná záloha a10s závěrečné pasivní health okno. Žádná reconnect smyčka nebo další otevření po skončení.
Přijato2730 paketů, raw RX102962B, raw TX2605B. HEARTBEAT77, největší host mezera1.063000s. Parser receive errors0; tento údaj není úplný důkaz nulových ztrát všech zpráv. HB po celou dobu DISARMED STABILIZE. EKF flags[167], attitude valid. SYS_STATUS gyro/accel/baro/AHRS/logger health prošly, InternalError masky/počet a dropped0 ve všech přijatých vzorcích.

Napájení pouze USB je potvrzení obsluhy. Bateriové PreArm varování nízkého napětí a nepřipojené RC nejsou automaticky porucha tohoto USB-only protokolu; nejsou ani ARM/letové GO. Fyzické napájecí kolejnice ani battery-under-load nebyly kvalifikované.

TX audit: {'COMMAND_LONG': 19, 'PARAM_REQUEST_READ': 29, 'FILE_TRANSFER_PROTOCOL': 36, 'PARAM_REQUEST_LIST': 2, 'LOG_REQUEST_LIST': 1}. COMMAND_LONG jen REQUEST_MESSAGE512; read-only FTP jen ResetSessions/OpenFileRO/BurstReadFile/TerminateSession. FTP ResetSessions není reboot FC. Žádný PARAM_SET/SET_MESSAGE_INTERVAL, ARM, motor/servo/PWM/DShot příkaz. Raw přenos je uložen nezávisle na dekódovaném seznamu odchozích zpráv.

## Parametry

První i finální záloha **1220/1220** jmen a úplné indexy0–1219; neindexované65535 notifikace oddělené. Proti první záloze nového bootu se finální parametry nezměnily. Proti původní HW-1 pre-upload záloze existují pouze níže uvedené změny:

| Parametr | Před HW-1 uploadem | HW-1R finální |
|---|---:|---:|
| BARO1_GND_PRESS | 101028.1875 | 101162.757812 |
| COMPASS_DEC | 0.0936297029257 | 0.0936292260885 |
| INS_GYR1_CALTEMP | 27.8630008698 | 28.5110015869 |
| INS_GYR2OFFS_X | -0.00115532032214 | -0.00145147775766 |
| INS_GYR2OFFS_Y | 0.00127842603251 | 0.00125568860676 |
| INS_GYR2OFFS_Z | 0.00250692712143 | 0.00278192595579 |
| INS_GYR2_CALTEMP | 27.1930007935 | 27.7849998474 |
| INS_GYROFFS_X | -0.0102765252814 | -0.0124047361314 |
| INS_GYROFFS_Y | -0.00302289659157 | -0.00264663016424 |
| INS_GYROFFS_Z | 0.00244876532815 | 0.0030563196633 |
| STAT_BOOTCNT | 112 | 115 |
| STAT_RUNTIME | 192845 | 194200 |

Gyro offsety/kalibrační teploty vznikají při původní startup kalibraci (`AP_InertialSensor.cpp:969,1692–1849`), BARO1_GND_PRESS je interní kalibrační tlak (`AP_Baro.cpp:93–101,398`), COMPASS_DEC může dopočítat původní `Compass::try_set_initial_location()` (`AP_Compass.cpp:1989–2007`) při AUTODEC1. Mechanismy jsou doložené source; konkrétní historická volání před připojením nebyla trasována. Změny těchto automatických/startup hodnot se neobnovovaly starou zálohou. STAT_BOOTCNT/RUNTIME jsou provozní čítače.

ATC/PID, motor/servo/FRAME, RC, failsafe, filtry, EKF/AHRS, baterie i ostatní letová konfigurace se zachovaly. COMPASS_USE/USE2/USE3 zůstávají0; absolutní yaw a letová způsobilost se tím neověřují. Žádný automatický PARAM_SET nebo hromadná obnova `.param`.

SCHED_LOOP_RATE400,FSTRATE_ENABLE0,SCHED_OPTIONS0,LOG_DISARMED0,LOG_BACKEND_TYPE4. Všechny UTT_RUN/UCS_F_RUN/UCS_RUN/UAS_RUN/UNS_RUN0. Tato fáze neměří skutečnou periodu400Hz ani latenci motorové cesty.

## Diagnostická rozhraní

| Soubor | Skutečný obsah | Bajtů |
|---|---|---:|
| @SYS/utb_timing.txt |UTBTimingV1,pending |20 |
| @SYS/utb_execution.txt |UTBExecutionV1,pending |23 |
| @SYS/utb_flight.txt |UTBFlightV1,pending |20 |

Všechna tři rozhraní byla načtena sekvenčně a obsahově ověřena, bez RUN. Žádná nečekaná frozen session nebo vyrobené měřicí údaje. Pending nemá ID dokončené session: příslušnost k aktuálnímu bootu je doložena okolní identitou/boot115 a dostupným singletonem/parametry, nikoliv vymyšleným session timestampem. Nový soubor společně s firmware identity potvrzuje dostupnost nové diagnostiky; její vlastní aktivní výpočet je stále HW neověřený. Upozornění MAVFTP expected100000/got20/23/346/1135 je skutečný EOF krátkých virtuálních souborů; přenosy error_code0 a obsah se kontroloval.

## Runtime RAM a stack

Tři snapshoty: po úvodní stabilitě, po parametrických/read-only dotazech a po sekvenčních SYS exportech. Nejde o boot-time minimum ani aktivní Execution session.

| Region | Před SYS FREE/LRG B | Po dotazech | Po SYS exportech |
|---|---:|---:|---:|
| 0x30000000 | 3688/3640 | 3688/3640 | 4712/4192 |
| 0x20000000 | 47944/47944 | 47944/47944 | 37936/37936 |
| 0x24000000 | 524280/524280 | 524280/524280 | 524280/524280 |
| 0x00000400 | 64504/64504 | 64504/64504 | 64504/64504 |
| 0x30040000 | 32760/32760 | 32760/32760 | 32760/32760 |
| 0x38000000 | 54536/53816 | 54536/53816 | 54536/53816 |

V0x20000000 byl třetí snapshot FREE/LRG o10008B nižší; v0x30000000 FREE naopak+1024B a největší blok+552B. Zbylé regiony stejné. Přesný vlastník alokace není změřen; rozhraní ExpandingString/FTP samo alokuje a snapshot má observer effect, ale není doloženo, že vysvětluje celý rozdíl. Nejde o prokázaný leak ani o důkaz návratu po dalším uvolnění. Další připojení kvůli tomu nebylo provedeno. Zůstává37936B souvisle v0x20000000 a další typované regiony, bez alokační chyby; součet není univerzálně použitelný DMA blok.

Starý HW-1 preflight měl7672/3128B v0x30000000 a37936/37936B v0x20000000. Rozdílný boot/uptime/historie exportů neumožňují izolovaně odečíst statickou cenu1712B od těchto runtime snapshotů. Linker EXEC ON .heap75016B, .text+3428B a .bss+1712B jsou dřívější build údaje, ne zdejší FREE.

| Vlákno | Po stabilitě volno/přiděleno B | Po dotazech | Po SYS exportech |
|---|---:|---:|---:|
| ISR | 1296/1536 | 1296/1536 | 1296/1536 |
| ArduCopter | 4344/7168 | 4344/7168 | 4344/7168 |
| idle | 288/496 | 288/496 | 288/496 |
| UART_RX | 864/1200 | 864/1200 | 864/1200 |
| OTG1 | 352/752 | 352/752 | 352/752 |
| monitor | 1096/1456 | 1096/1456 | 1096/1456 |
| timer | 1552/1968 | 1552/1968 | 1552/1968 |
| rcout | 504/944 | 504/944 | 504/944 |
| rcin | 816/1456 | 816/1456 | 816/1456 |
| io | 1544/2480 | 1544/2480 | 1544/2480 |
| storage | 960/1456 | 960/1456 | 960/1456 |
| UART4 | 368/752 | 368/752 | 368/752 |
| log_io | 1432/2016 | 1432/2016 | 1432/2016 |
| I2C0 | 984/1456 | 984/1456 | 984/1456 |
| OSD | 976/1712 | 976/1712 | 976/1712 |
| led | 280/688 | 280/688 | 280/688 |
| UART1 | 336/752 | 336/752 | 336/752 |
| I2C1 | 1000/1456 | 1000/1456 | 1000/1456 |
| SPI1 | 728/1456 | 728/1456 | 728/1456 |
| SPI4 | 864/1456 | 864/1456 | 864/1456 |
| utb_log | 2472/3504 | 2472/3504 | 2472/3504 |
| FTP | 1704/2992 | 1592/2992 | 1568/2992 |

Main4344/7168, ISR1296/1536 stabilní; FTP1704→1592→1568/2992 při rozhraních, utb_log2472/3504 stabilní. Předchozí E-B Main4344/7168, ISR1264/1536, FTP1216/2992 jsou jiný boot a plné frozen exporty: lepší aktuální watermark nedokazuje menší worst-case stack. Pending export má jen20/23B. Aktivní capture/finalizace a plné Execution exporty mohou jít hlouběji a musí být součástí HW-2A. Žádný rate thread, stack overflow nebo alokační selhání nebyly pozorovány; žádná nová libovolná bezpečnostní hranice se nezavádí.

## CPU pouze při RUN0

SYS_STATUS pouze5 vzorků: min18.3 %, mean18.8 %, max19.3 %. Závěrečný jediný vzorek18.3 %. Jde o řídké průměrované scheduler load údaje při read-only protokolu, nikoliv cenu aktivního Execution, period jitter nebo WCET. Telemetrické intervaly se neměnily.

## Původní DataFlash logy

| ID | Velikost B | time_utc |
|---|---:|---:|
| 1 | 2516746 | 0 |
| 2 | 7924746 | 0 |
| 3 | 718496 | 0 |
| 4 | 725746 | 1792564233 |

Celkem4/11885734B; přesná shoda kompletních LOG_ENTRY metadat s předupload HW-1. Pouze jeden listing, bez stahování BIN nebo otevření loggeru. Jde o **metadatový PASS**, nikoliv bitovou shodu všech flash dat. Úplné historické BIN archivy ověřené v HW-1 zůstávají zachované. LOG_DISARMED0, žádný nový log/erase/recovery problém v dostupné evidenci; samotná tato kontrola neověřuje persistentní letové logování nebo nulové budoucí riziko loggeru.

## Pravidlo pro další HW fáze a helper

Při zmizení COM po schváleném uploadu/restartu/USB napájení nejprve pasivní PnP/VID/PID/serial/DFU/události. Samotná absence není firmware FAIL. Potom **jeden konkrétně schválený fyzický reconnect obsluhou**, bez LiPo/vrtulí. Po skutečném potvrzení znovu hledat podle serialu200024001251303433333336 a1209:5741; COM číslo se smí změnit. Teprve při jednoznačné identitě jedno read-only otevření115200, DTR/RTSFalse před open. Chyba/DFU/ambiguous/absence po omezeném sledování znamená stop, žádný automatický druhý reconnect, reboot, upload nebo rollback. Nový fyzický krok a nové spojení vždy musí patřit do schváleného rozsahu.

Nový malý `Tools/utb/tools/utb_usb_identity.py` čte pouze metadata `serial.tools.list_ports`; nemá serial.Serial/MAVLink/flash/napájecí operaci. Vrací absent/ambiguous/present nebo unexpected_mode; jiný VID/PID se stejným serialem nedovolí použít jako aplikační port. Samotný DFU bez očekávaného serialu nelze přiřadit konkrétní FC pouze tímto helperem, proto zůstává potřeba PnP snapshotu.

CLI vyžaduje --confirmed-reconnect; příznak je záznam potvrzení, nikoliv náhrada skutečného souhlasu. Evidence se vytváří výhradně novým souborem (existující chybu nepřepíše). Default je jeden výčet, volitelný --wait-seconds je omezený0–10s a nejvýše11 výčtů; chyby/ambiguous/unexpected_mode se neopakují. Nekonečný polling ani automatické pokračování do MAVLink neexistují. Port z helperu musí následný schválený runner znovu ověřit pomocí MAVLink board/UID.

Příklad **jen pro už schválený a potvrzený fyzický krok**:

```text
python Tools/utb/tools/utb_usb_identity.py --confirmed-reconnect --evidence NOVY_ADRESAR/usb.json --wait-seconds 5
```

Helper15 unit PASS (6 subtests), celý Python UTB soubor220 PASS,5 skip,8 subtests PASS, flake8 a git diff --check PASS. Windows reálný metadata-only test helperu po zavření MAVLink úspěšný, COM5/serial správně, serial_port_openedFalse. Testy používají syntetické metadata pro COM8/duplicitní/absent/wrongmode/chybu a omezené polling; nevydávají simulované chyby za HW měření. C++/SITL/build nebyly v této fázi znovu spouštěny, firmware se neměnil.

## Připravenost a hranice dalšího postupu

**GO k samostatnému schválení HW-2A**, nikoliv jeho spuštění. Aktuální boot115 má Timing/Execution/Flight pending a všechny RUN0; není prokázaně potřeba další fyzický restart. Před novým měřením znovu ověřit identitu/zdraví/boot a pending, zachovat původní AP motorovou autoritu. Navrhovaný jediný běh: UTT_COUNT4000 a po výslovném schválení jednou UTT_RUN1, UCS_F_RUN0. Baseline/případné telemetrické intervaly a jejich obnovení musí být součástí konkrétního budoucího souhlasu. Po freeze sekvenční Timing+Execution a paměť/stack, žádný export v okně.

HW-2B bude odděleně schválené porovnání Native+Flight+Timing při shodné konfiguraci/telemetrii a s novým vhodným bootem. Měřit softwarové M1–M5/Main/Rate/Attitude/consume/observer; nyní nic z toho nebylo aktivováno.

Zůstává neprokázaná příčina USB výpadku a návratu, úplná historie mezilehlých bootů, dlouhodobá USB stabilita, přechodná alokační příčina třetího snapshotu, aktivní Execution stack/latence, logger při zápisu, hard WCET, elektrické DShot dokončení a letová způsobilost. Úspěch RUN0 není PASS aktivního400Hz měření ani souhlas s letem.

## Git a konečný stav

HEAD11a518e5857b24cc554b3cb7e6492998013ddd1a, větev test/skystars-5inch, index čistý. Původních7 modified +7 added z3G-3F, reportHW-1, veškerá původní evidence a kapitoly/historie.txt bitově zachované. Tato fáze přidává pouze3 nové soubory: tento report, metadata helper a jeho test. Žádný commit/push/reset/clean/stash/rebase, build nebo instalace.

FC naposledy telemetrií ověřena zdravá DISARMED STABILIZE, boot115, všechny RUN0, logger0drops/InternalError0. Parametry/intervaly nebyly hostem změněny, nebylo co obnovovat. Port uzavřený, po skončení metadata-only výčet FC stále vidí. Další HW fáze nebyla zahájena.
