# FÁZE 3G-3B – Read-only preflight Skystars H743

Datum 25. 10. 2026. **GO pouze k vyžádání samostatného schválení jednoho uploadu v následující fázi 3G-3C.** Upload ani experiment nejsou tímto rozhodnutím schválené a neproběhly.

## Rozsah a důkazy

Před otevřením portu znovu potvrzeno USB-only napájení, stejný kabel a port notebooku jako 3G-3A, odpojená LiPo, demontované vrtule, DISARMED a zavřené ostatní sériové aplikace. Záznam `physical-confirmation.json`. Jediné otevření COM5, bez reconnectu. Existující runner 3G-3A byl upraven jen pro 40s okno a přesnou validaci parametrů/TX auditu. Neindexované notifikace 65535 se evidují zvlášť; TX audit se připojuje až po automatické detekci MAVLink 2, aby se neztratil při výměně parseru.

Nové důkazy: `/home/jirka/ardupilot_utb/tmp/utb-3g3b-preflight-20261025_094421/`. `receive.raw`, `messages.jsonl`, `result.json`, `analysis.json`, `parameters.json`, `parameters.param`, `memory.txt`, `threads.txt`, PnP snapshoty a SHA-256 manifest. Časy hostitele nejsou měřením periody hlavní regulační smyčky.

Nic nebylo instalováno. Bez sestavení, změn C++, PARAM_SET, změn intervalů, rebootu, ARM, motorových příkazů, DataFlash aktivace, BIN downloadu, mazání nebo zápisu firmware.

## Offline integrita

HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`, větev `test/skystars-5inch`. Všech 15 souborů původního zdrojového manifestu 3G-2 odpovídá SHA-256 před i po kontrole. Všech 34 souborů hashovaného důkazního manifestu 3G-3A ověřeno. Předchozí dva necommitnuté reporty a `kapitoly/historie.txt` zachovány bitově; jejich hashe jsou v `offline-audit.json`.

Přesný kandidát:

`/home/jirka/ardupilot_utb/tmp/utb-3g2-20261025_071149/firmware/SkystarsH7HD-bdshot_UTB3G2_PASSIVE_RAM_918dedaaeae9.apj`

- Velikost **1 155 815 B**, board ID **1075**, obraz po dekompresi **1 281 164 B**.
- SHA-256 **`918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7`**.
- APJ git_identity `d78e34b3`: očekávaný build před implementačním commitem; identita doložena současně manifestem a SHA.
- Existující H743_FINAL symbol audit obsahuje Flight, Chain a Timing. Symboly aktivního UTB_Core/Output/Acro/Angle chybí; Flight init nastavuje RUN na 0.
- Jde o audit připraveného artefaktu. Kandidát dosud na této desce neběží.

## Identita a časová osa živé relace

COM5, VID:PID `1209:5741`, USB serial **200024001251303433333336**. Windows instance `USB\VID_1209&PID_5741\200024001251303433333336`; stav OK a ProblemCode 0 před/po kontrole. Topologie `Port_#0001.Hub_#0003`, `PCIROOT(0)#PCI(0803)#PCI(0004)#USBROOT(0)#USB(1)` odpovídá 3G-3A. Driver ani napájení Windows se neměnily.

MAVLink source **1/1**, ArduCopter **4.7.1 / 90b24237**, board_version>>16 **1075**, UID2 prvních 12 B souhlasí se serialem. Běží původní 3F firmware; nikoliv kandidátní Flight firmware. MCU revision nebyla znovu čtena bootloaderem.

První RX `2026-10-25T08:47:08.886591+00:00`, poslední RX `2026-10-25T08:48:07.742546+00:00` (UTC; místní čas CET je UTC+1). Celé otevření **60,438 s**, stabilní health okno **40,109 s**, následná záloha **6,797 s**. Paměť a vlákna načteny sekvenčně (0,046 a 0,063 s), potom log inventory a závěrečný health. COM uzavřen, bez dalšího otevření.

| Kontrola | Výsledek |
|---|---:|
| Přijaté zprávy / raw RX | 1394 / 51 345 B |
| HEARTBEAT / nejdelší host mezera | 60 / 1,063 s |
| Parser chyby / pozorované sekvenční mezery | 0 / 0 |
| SerialException / zjištěný restart | 0 / 0 |
| Uptime první / poslední SYSTEM_TIME | 2236116 / 2292271 ms |
| STAT_BOOTCNT | 110, shodný s 3G-3A |
| SYS_STATUS CPU | 18,2–18,6 % (6 vzorků) |
| EKF flags | 167 ve všech 6 odpovědích |

Všech 60 heartbeatů DISARMED STABILIZE (base_mode 81, custom_mode 0); poslední v čase 60,047 s. Gyro/accel/AHRS a logger indikátory zdravé v dostupných SYS_STATUS, EKF attitude valid. Všechny errors_count1/2/3/4 a errors_comm jsou 0. Zdroj `libraries/GCS_MAVLink/GCS_Common.cpp:6021`: count1/2 = InternalError maska, count4 = počet, count3 = dropped log zprávy. Nulové dropped při vypnutém logování neověřují propustnost loggeru.

Zaznamenané PreArm hlášky RC not found a nízké napětí Battery 1 odpovídají USB-only stavu bez RC/LiPo. Nejde o schválení armování. Žádná zaznamenaná chyba gyro/EKF/InternalError. PM.NLon, přesný jitter 400Hz smyčky ani nový SHADOW nebyly v této fázi měřeny.

TX audit: 29 COMMAND_LONG, výhradně REQUEST_MESSAGE (512); jedna PARAM_REQUEST_LIST; osm read-only FTP zpráv (ResetSessions/OpenFileRO/BurstReadFile/TerminateSession) pouze pro memory/threads; jedna LOG_REQUEST_LIST. Žádný PARAM_SET, SET_MESSAGE_INTERVAL, ARM, motorový nebo bootloader příkaz. ResetSessions se týká jen FTP relace, nikoliv restartu FC.

## Parametry a podporovaný profil

**1215/1215 unikátních názvů a indexů 0–1214**, žádný chybějící, duplicitní či konfliktní index; dva samostatné STAT_RUNTIME s indexem 65535. Celkem 1217 PARAM_VALUE. Záloha dokončena v čase 48,110 s relace, heartbeat pokračoval.

Proti 3G-3A jediný rozdíl STAT_RUNTIME 160325 → 161998 s. PID/filtry/SMAX/notch, RC mapování, FS, motorové, servo, bateriové, GPS, kompas a EKF parametry jsou shodné. Kompletní srovnání je v analysis.json a vybrané relevantní hodnoty v selected_parameters. Proti 3F-3 zůstává stejných 12 rozdílů jako v 3G-3A:

| Parametr | 3F-3 | Nyní |
|---|---:|---:|
| BARO1_GND_PRESS | 99344.734375 | 101112.726562 |
| COMPASS_DEC | 0 | 0.0936296507716 |
| INS_GYR1_CALTEMP | 28.9270000458 | 22.4420013428 |
| INS_GYR2OFFS_X | 0.000300836836686 | -3.51374510501e-05 |
| INS_GYR2OFFS_Y | 0.00218398449942 | -0.00210712337866 |
| INS_GYR2OFFS_Z | 0.00288892770186 | 0.00107814604416 |
| INS_GYR2_CALTEMP | 28.2090015411 | 21.8590011597 |
| INS_GYROFFS_X | -0.012337025255 | -0.00448013748974 |
| INS_GYROFFS_Y | -0.00276684807613 | -0.00152507657185 |
| INS_GYROFFS_Z | 0.0034382333979 | 0.0021507977508 |
| STAT_BOOTCNT | 106 | 110 |
| STAT_RUNTIME | 155663 | 161998 |

Gyro offsety/kalibrační teploty a baro tlak odpovídají dříve zaznamenaným inicializačním změnám, statistiky běhu/bootu postupují. Nevznikla nová změna letové konfigurace. Nejde o nové ověření kvality kalibrace za letu.

- `SCHED_LOOP_RATE=400`.
- `FSTRATE_ENABLE=0`.
- `UCS_RUN=0`.
- `UAS_RUN=0`.
- `UNS_RUN=0`.
- `SCHED_OPTIONS=0`.
- `LOG_DISARMED=0`.
- `FRAME_CLASS=1`.
- `FRAME_TYPE=18`.
- `FS_THR_ENABLE=1`.
- `FS_GCS_ENABLE=0`.
- `COMPASS_USE=0`.
- `COMPASS_USE2=0`.
- `COMPASS_USE3=0`.
- `COMPASS_ENABLE=1`.
- `COMPASS_AUTODEC=1`.
- `AHRS_EKF_TYPE=3`.
- `EK3_SRC1_YAW=1`.

SMAX a PID notch konfigurace zůstávají v původním podporovaném profilu; bez rate threadu v threads.txt. `g2.att_enable` je registrovaný FSTRATE_ENABLE (`ArduCopter/Parameters.cpp:1123`), nikoliv chybějící samostatný ATT_ENABLE. Parametry UCS_F_* na starém firmwaru chybějí očekávaně.

## Deklinace a omezení yaw

COMPASS_DEC **0,09362965077161789 rad (~5,3654°)** a AUTODEC=1 jsou beze změny vůči 3G-3A. COMPASS_ENABLE=1, existuje magnetometr DEV_ID 1453065 a RAW_IMU obsahuje magnetická data. **COMPASS_USE/USE2/USE3=0**: kompas se nepoužívá k letovému yaw. EK3_SRC1_YAW=1 samo o sobě nedokládá skutečně aktivní magnetické fúzování.

GPS při jednorázovém čtení fix 3, 15 satelitů, poloha 49,1050208 / 17,4005691°. `libraries/AP_Compass/AP_Compass.cpp:1989` v try_set_initial_location ověřuje AUTODEC, enabled a AHRS polohu a nastavuje deklinaci podle AP_Declination. To vysvětluje možnost změny bez PARAM_SET a bez aktivního COMPASS_USE. **Historické konkrétní volání ani přesná příčina změny 0→DEC nejsou zachyceny**, proto se nepovažují za prokázané.

ATTITUDE je konečný a klidný (yaw přibližně −0,0010 rad); AHRS2 ukazuje sekundární orientaci (yaw přibližně 2,3066 rad). AHRS.error_yaw=1 není EKF innovation test: `AP_AHRS.cpp:924` vrací DCM metriku a `AP_AHRS_DCM.h:213` ji inicializuje na 1. `GCS_Common.cpp:617` posílá v AHRS2 sekundární řešení. Tyto údaje nelze vydávat za shodu ani za absolutní yaw přesnost. Dostupné EKF flags 167 a SYS_STATUS neukazují poruchu aktivní orientace; rozdíl sekundárního yaw není nově změřená kalibrace. **Absolutní heading/yaw a letová způsobilost nejsou ověřeny.** Pro pasivní USB-only preflight nebyl zjištěn nový blocker; před případným letem je nutné samostatné ověření zdroje yaw a kalibrace.

## Paměť a stack – skutečně běžící starý firmware

| Region | FREE B | Největší blok B | TYPE |
|---|---:|---:|---:|
| 0x30000000 | 8 | 8 | 8 |
| 0x20000000 | 86856 | 82000 | 2 |
| 0x24000000 | 524280 | 524280 | 4 |
| 0x00000400 | 64504 | 64504 | 2 |
| 0x30040000 | 32760 | 32760 | 8 |
| 0x38000000 | 54536 | 53816 | 1 |

Součet FREE **762 944 B**, oproti 3G-3A −7920 B; největší jednotlivý blok **524 280 B** v regionu 0x24000000. V 0x20000000 FREE 86 856 vs 94 776 B a LRG 82 000 vs 92 008 B. Zbylé regiony beze změny. Rozdíl není důkazem úniku: dva snapshoty mají jiný průběh FTP/parametrických operací a přesná alokační příčina nebyla měřena. Nelze ani tvrdit, že jej způsobila konkrétní alokace. Dostupná paměť a dokončení kontrol neukazují selhání, historické minimum a trend úniku nejsou dostupné.

MEMINFO.freemem32 **769 760 B** vs 777 680 B. FREE není jediný souvislý blok ani zaměnitelná RAM všech typů; součet nelze použít jako důkaz dostupnosti konkrétní DMA alokace. Rozhraní Util::mem_info (`libraries/AP_HAL_ChibiOS/Util.cpp:478`) používá chHeapStatus jednotlivých regionů; available_memory a MEMINFO mají odlišnou sémantiku. Žádné historické minimum heapu se nevydává za měření.

| Stack watermark volno / přiděleno | 3G-3A | Nyní |
|---|---:|---:|
| Main | 4344/7168 B | 4344/7168 B |
| ISR | 1288/1536 B | 1240/1536 B |
| FTP | 1704/2992 B | 1592/2992 B |
| utb_log | viz původní snapshot | 2448/3504 B |

Úplná vlákna jsou v threads.txt. Změna ISR −48 B a FTP −112 B není prokázaný overflow; watermark je dosavadní nejhlubší využití, nikoliv bezpečnostní mez. Žádný rate thread ani aktivní Flight worker se neobjevil. Bez záruky WCET/nejhoršího stacku kandidáta.

## Logger

| ID | time_utc | Velikost B |
|---|---:|---:|
| 1 | 0 | 2516746 |
| 2 | 0 | 7924746 |
| 3 | 0 | 718496 |
| 4 | 1792564233 | 725746 |

Čtyři logy, součet **11 885 734 B**, přesná shoda ID/časů/velikostí s 3G-3A a koncem 3F-3. Pouze inventory, nikoliv bitový readback obsahu nebo měření volných sektorů. Logger je v SYS_STATUS zdravý, dropped=0; LOG_DISARMED=0. Nový log nebyl otevřen, žádný starý stažen nebo smazán. Budoucí RAM-only test DataFlash kapacitu nepotřebuje.

Známé prodlevy prvního otevření loggeru 5503/6480 µs zůstávají nevysvětlené. Tato fáze je netriggerovala a neopravila; průběžné zápisy, mazání sektorů a obnovení loggeru nejsou nově kvalifikované.

## Rizika kandidáta a podklady 3G-3C

Existující build artefakty H743_OFF/ON/FINAL: Flight ON přidává **39 016 B .bss**, samotný recorder **38 912 B (128×304)**. Timing přidává dalších **1880 B .bss**, výsledné .bss **172 536 B**, linker heap **76 728 B**. Jde o statická čísla kandidáta, nikoliv zde naměřenou runtime rezervu. Žádný výpočet typu FREE starého FW mínus recorder se nepoužívá. Statické lokální frame 320/832 B nejsou transitive maximum stacku. Z dostupných buildů a symbol auditu není znám konkrétní startup/memory blocker; skutečný boot, heap, stack a časování kandidáta na této FC jsou stále **neověřené**.

Případné samostatné schválení **3G-3C** má určit tento přesný APJ/SHA/velikost/board/serial a rozsah:

1. Jeden standardní upload výše uvedeného APJ na jednoznačně identifikovanou FC včetně standardního restartu uploaderem. Bez bootloader rewrite, DFU recovery, mazání logů či přepisu konfigurace; při chybě žádný automatický druhý pokus.
2. USB-only, bez LiPo/props/ARM; předem znovu potvrdit fyzické podmínky. Po uploadu ověřit zdravou inicializaci, verzi, CRC uploaderu, úplné parametry, inventář logů, paměť/stack, RUN=0 a pending snapshot. CRC není kryptografický readback MCU.
3. Pouze diagnostické UCS_F_COUNT=4000 a UCS_F_DECIM=40. UTT_RUN, UCS_RUN, UAS_RUN, UNS_RUN zůstanou 0; LOG_DISARMED=0 a SCHED_OPTIONS=0. Žádné změny řízení/filtrů kvůli shodě.
4. Dočasná telemetrie SYS_STATUS 2 Hz, SYSTEM_TIME 1 Hz, EKF_STATUS_REPORT 1 Hz se zálohou a obnovením intervalů. Po 3s ustálení 15s baseline; **jediný UCS_F_RUN=1**, přibližně 10 s pouze příjem, žádné FTP/polling/listing během okna.
5. Po automatickém stopu RUN=0, health/DISARMED a 8–10 s stejných zpráv. Poté sekvenční export metadata + osmi RAM stránek původním analyzátorem; memory/stack a finální parametry/inventory. Obnovit diagnostické hodnoty a intervaly.
6. Stop při identitní neshodě, COM chybě, restartu, ARM, poruše senzorů/loggeru, InternalError, numerické neshodě/desync/motor_changes nebo významné časové anomálii. Bez automatického opakování, rebootu či nové instalace. Případný další fyzický start pouze samostatným krokem obsluhy.

Tento plán **není proveden**. Žádný další SHADOW ani skutečný let není schválen. Přesná Main/Attitude entry perioda bez další sondy zůstane neověřená; UTT se v tomto protokolu nepřidává.

## Konečný stav a Git

Poslední přijatý heartbeat DISARMED STABILIZE, dostupný EKF/SYS_STATUS bez nové chyby. COM5 uzavřen. Parametry nezapisovány, původní log inventory zachován. Nebyl proveden upload, SHADOW, build ani commit/push.

HEAD **11a518e5857b24cc554b3cb7e6492998013ddd1a**, větev test/skystars-5inch. Tracked pracovní strom a index beze změny. Původní necommitnuté dva reporty a kapitola zachovány, přibývá pouze tento necommitnutý report; důkazy jsou v ignorovaném tmp. Závěrečný přesný status a diff --check jsou v git-final.txt.

**GO pouze k předložení samostatného schválení uploadu a jednoho USB-only experimentu.** Původní USB závada není kauzálně vysvětlená; opětovná úspěšná relace dokládá aktuální stabilitu, nikoliv její trvalé odstranění. Absolutní yaw, runtime paměť/časování kandidáta, aktivní řízení a let zůstávají neověřené.
