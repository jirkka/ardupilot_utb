# FÁZE 3G-3A – Stabilita USB/COM Skystars H743

Datum25.10.2026, časové údaje níže Europe/Prague (CET, UTC+1); raw zprávy obsahují také explicitní UTC a host monotónní čas.

## Závěr

**GO pouze k samostatně obnovenému read-only preflightu3G-3. Nejde o souhlas s uploadem nebo Flight SHADOW experimentem.** Při tomto zapojení se předchozí ClearCommError nereprodukoval. Původní USB připojení zůstává podezřelé, ale kořenová příčina není prokázána.

Jedno pozorování trvalo **120.093s**, celé první otevřené spojení166.359s. Správná FC byla trvale v pozorovaných heartbeatech DISARMED STABILIZE; žádná SerialException, CRC/parser chyba, zjištěný restart nebo pozorovaná změna COM. Záloha je po offline kontrole **1215/1215 indexovaných parametrů, indexy0–1214 úplné**. Následné samostatně schválené read-only otevření dokončilo memory/threads/MEMINFO/inventář a závěrečný health, pak se port uzavřel.

Původní runner přitom obsahoval chybu závěrečné kontroly indexů; jeho `success=false` nebyl zaměněn za USB chybu ani přepsán. Podrobnosti a výslovné povolení dalšího otevření jsou uvedené níže.

## Fyzické podmínky a rozsah

Obsluha vrátila stejný USB kabel do dříve stabilního portu notebooku. Navazuje se na potvrzené USB-only, odpojenou LiPo, demontované vrtule a zákaz ARM/motorových příkazů z3G-3; během měření se port ani kabel neměnil. Změnu fyzického zapojení provedla obsluha, nikoliv software. Nikdo z měřicích skriptů neposlal ARM, motorový příkaz, reboot, PARAM_SET, SET_MESSAGE_INTERVAL, upload nebo SHADOW RUN. Žádná změna driverů či Windows napájení, instalace, nový build nebo Git commit/push.

Existující Windows Python3.10 a již dostupný pyserial/pymavlink z projektového prostředí; nic nedoinstalováno. Původní40s/30s preflight neuměl požadované120s sledování, sekvence a průběžnou enumeraci, proto vznikl malý izolovaný runner jen v ignorovaném tmp. Nový komunikační protokol nebyl implementován.

## Windows identita, driver a topologie

- COM5; VID:PID1209:5741, serial200024001251303433333336.
- Instance `USB\VID_1209&PID_5741\200024001251303433333336`.
- FriendlyName ArduPilot (COM5), BusReportedDeviceDesc SkystarsH7HD-bdshot, stavOK, ProblemCode0.
- Port_#0001.Hub_#0003; `PCIROOT(0)#PCI(0803)#PCI(0004)#USBROOT(0)#USB(1)`.
- Balíček driveru ArduPilot Project2.0.0.10, INF oem118.inf, datum26.12.2019. CIM u balíčku uvádí IsSigned=false/Signer=null; tento údaj sám nevysvětluje výpadek.
- Službausbser, skutečný systémový soubor `C:\Windows\System32\drivers\usbser.sys` verze10.0.26100.9278. Nezaměňovat INF verzi s verzí systémové binárky.

Zachycený rodičovský řetězec:

- `USB\VID_1209&PID_5741\200024001251303433333336` — ArduPilot (COM5) , stavOK.
- `USB\ROOT_HUB30\5&38439293&0&0` — Kořenový rozbočovač USB (USB 3.0), stavOK.
- `PCI\VEN_1022&DEV_15C1&SUBSYS_15C11022&REV_00\4&DEDE588&0&0443` — AMD USB 3.10 eXtensible Host Controller - 1.20 (Microsoft), stavOK.
- `PCI\VEN_1022&DEV_14EB&SUBSYS_14EB1022&REV_00\3&2411E6FE&0&43` — Kořenový port sběrnice PCI Express, stavOK.
- `ACPI\PNP0A08\1` — Zařízení root complex sběrnice PCI Express, stavOK.
- `ACPI_HAL\PNP0C08\0` — Systém vyhovující standardu ACPI (Microsoft), stavOK.

Před otevřením nebyl v dostupném přehledu nalezen Mission Planner, Python/MAVLink monitor nebo běžný sériový terminál. Samotné názvy procesů nejsou úplná enumerace handle všech aplikací. Naše úspěšné otevření používá Windows CreateFile s share_mode0, tedy výhradní přístup (`serialwin32.py:55–62`). Nebyl doložen konflikt a žádný cizí proces nebyl ukončen.

DTR=False a RTS=False byly nastavené na ještě neotevřeném objektu pyserial; otevřelo se rovnou115200baud. Vynechal se úvodní1200baud workaround, který standardní mavserial používá kvůli některým Linux kernelům. Nebylo záměrné pulzování linek. Tím není tvrzeno fyzické měření elektrických linek USB/FC.

## Jeden120s test a raw evidence

První přijatá zpráva: **2026-10-25T09:17:52.936491+01:00**. Firmware z AUTOPILOT_VERSION **ArduCopter4.7.1 /90b24237**, board_version>>16=1075, UID2 prvních12B se shodným USB serialem. MCU Rev.V je historická identifikace; zde se nevstupovalo do bootloaderu a revision registr se nečetl.

| Veličina | Skutečný výsledek |
|---|---:|
| Samostatné stabilní pozorovací okno | 120.093s |
| Celé první spojení včetně parametrů | 166.359s |
| Přijaté zprávy, celé první spojení | 1548 |
| Raw RX | 53834B |
| HEARTBEAT | 166 |
| HB host mezera min/průměr/P95/max | 0.937000/0.998861/1.032000/1.063000s |
| Parser/CRC errors | 0 |
| Zjištěné anomálie MAVLink sequence | 0 |
| mavutil odhad ztrát | 0 |
| USB serial/COM enumerace | 82 vzorků, stáleCOM5 |
| SYSTEM_TIME v prvním spojení | 478980→597053ms, bez poklesu |
| SerialException / serial read API chyba | 0 |

Heartbeat je native stream. Identita se vyžádala jednou; SYSTEM_TIME nejvýše po5s, SYS_STATUS/EKF nejvýše po10s, jen pokud nebyla čerstvá zpráva. Žádné konfigurování streamů. Heartbeat timeout5s je politika host monitoru, nikoliv nový limit řízení FC; při chybě není autoreconnect. Windows port/serial se kontrolovaly přibližně každé2s; mezi snapshoty nelze vyloučit libovolně krátkou skrytou PnP událost. Nepřerušené čtení, pravidelný heartbeat a nulové parser/sequence chyby nepodporují pozorovaný výpadek komunikace.

`messages.jsonl` obsahuje host monotonic, elapsed, UTC, fázi, system/component ID, MAVLink sequence a celé dekódované zprávy. `receive.raw` jsou skutečné RX bajty. `rx-per-second.csv` má všech120 sekundových binů pozorování, `heartbeat-gaps.csv` všechny mezery celého prvního spojení. Příchodový jitter na hostu není H743 Main/Rate/Attitude perioda.

![Skutečný příjem a heartbeat mezery](../tmp/utb-3g3a-usb-20261025_091414/usb-stability.png)

MAVLink automatické přepnutí na verzi2 vyměnilo parser objekt; pomocný wrapper nad jeho metodou send proto neposkytuje úplný TX audit (`result.json.outbound` je prázdný). Read-only rozsah je doložen přímo odesílacími místy izolovaných skriptů a použitými read-only FTP operacemi, nikoliv tvrzením o úplném packet capture TX. Raw RX a hook příchozích zpráv tím dotčené nebyly. Tento limit je zachován v evidenci.

## Parametry a vlastní chyba runneru

Odpovědělo **1215 unikátních jmen a všech1215 indexů0–1214**. Žádný chybějící či konfliktní index. Indexovaná série v host čase **121.281–128.062s** od startu; rozpětí6.781s. Celá série dorazila nejpozději6.844s po přechodu do parametrické fáze; přesný TX timestamp PARAM_REQUEST_LIST nezachytil wrapper, proto jde o horní mez podle začátku fáze.

Navíc přišlo5 neindexovaných aktualizací STAT_RUNTIME s param_index65535. `GCS_Param.cpp:359` pro průběžné notifikace nastavuje param_index=-1, což se v uint16 přenáší jako65535. Nejde o chybějící1216.parametr. Původní kontrola požadovala rovnost celé množiny přijatých indexů s range(1215), takže po45s nesprávně vyvolala `AssertionError('Incomplete parameter backup')` a port uzavřela. Nešlo o SerialException a test se automaticky neopakoval.

Offline rekonstrukce z uložených odpovědí ověřila každý platný index, jedinečnost jmen a nepřítomnost konfliktů;5 notifikací se eviduje zvlášť, neskrývá se. Původní výsledek a traceback zůstávají v result.json. Úplná záloha je `parameters-recovered.json` a `.param`, kontrola v `parameter-recovery.json`.

### Porovnání proti konci3F-3

PID/ATC filtry, RC, failsafe, motorové a servo parametry, SCHED/FSTRATE, UCS/UAS/UNS, LOG_DISARMED a SCHED_OPTIONS jsou beze změny. SCHED_LOOP_RATE400, FSTRATE_ENABLE0, UCS_RUN/UAS_RUN/UNS_RUN0, UCS_COUNT4000, LOG_DISARMED0 a SCHED_OPTIONS0. Nový Flight firmware není nahraný a jeho parametry se nepředpokládají.

Rozdíly celého seznamu:

| Parametr | Konec3F-3 | Aktuální indexovaná odpověď |
|---|---:|---:|
| BARO1_GND_PRESS | 99344.734375 | 101112.7265625 |
| COMPASS_DEC | 0.0 | 0.09362965077161789 |
| INS_GYR1_CALTEMP | 28.927000045776367 | 22.442001342773438 |
| INS_GYR2OFFS_X | 0.00030083683668635786 | -3.513745105010457e-05 |
| INS_GYR2OFFS_Y | 0.002183984499424696 | -0.002107123378664255 |
| INS_GYR2OFFS_Z | 0.0028889277018606663 | 0.001078146044164896 |
| INS_GYR2_CALTEMP | 28.209001541137695 | 21.85900115966797 |
| INS_GYROFFS_X | -0.012337025254964828 | -0.00448013748973608 |
| INS_GYROFFS_Y | -0.00276684807613492 | -0.0015250765718519688 |
| INS_GYROFFS_Z | 0.003438233397901058 | 0.002150797750800848 |
| STAT_BOOTCNT | 106.0 | 110.0 |
| STAT_RUNTIME | 155663.0 | 160325.0 |


Runtime/boot statistiky, gyro startup offsets/teploty a barometrický ground pressure se mezi boot session mění; staré hodnoty se nevracely. COMPASS_DEC se změnil na0.0936296508rad a COMPASS_AUTODEC=1. `Compass::set_initial_location()` v AP_Compass.cpp:1990–2010 při automatické deklinaci počítá hodnotu z polohy. Tento mechanismus změnu vysvětluje jako možnost, ale konkrétní inicializační volání nebylo zaznamenané; před případným uploadem má původní preflight tuto senzorovou odchylku výslovně posoudit. Není změnou PID/motorového řízení provedenou runnerem.

## Samostatně schválené dokončení read-only kontrol

Po zjištěné vlastní validační chybě byl výslovně vyžádán souhlas s **jedním dalším otevřením pouze pro memory/threads/MEMINFO/log inventory/health**, bez opakování120s testu nebo celé zálohy. Odpověď „ano, udělej co potřebuješ“ je uložená s přesným omezeným rozsahem v finish-approval.json. Zákaz automatického znovupřipojení tak nebyl obcházen.

Dodatečné spojení trvalo **17.438s**, první zpráva09:23:26.148CET. Board/USB/verze a DISARMED byly znovu ověřené. STAT_BOOTCNT před/po **110/110**, také úplná záloha má110. Počet106 na konci3F-3 je jiný historický boot, nikoliv restart během nynějšího testu. Uptime během jednotlivých pozorovaných spojení neklesl; nepozorovaný interval mezi nimi není vydáván za souvislý capture.

Sekvenční FTP: memory.txt346B/0,062s, threads.txt1135B/0,047s; oba přenosy error_code0. Hlášení expected100000/got346 či1135 jsou kratší skutečný EOF virtuálního souboru, nikoliv prokázaná ztráta dat. Nečetl se tasks.txt a žádný RUN se neaktivoval.

### Aktuální paměť

| Region | LEN | FREE [B] | Největší blok [B] | TYPE |
|---|---:|---:|---:|---:|
| 0x30000000 | 256 KiB | 8 | 8 | 8 |
| 0x20000000 | 128 KiB | 94776 | 92008 | 2 |
| 0x24000000 | 512 KiB | 524280 | 524280 | 4 |
| 0x00000400 | 63 KiB | 64504 | 64504 | 2 |
| 0x30040000 | 32 KiB | 32760 | 32760 | 8 |
| 0x38000000 | 64 KiB | 54536 | 53816 | 1 |


Součet FREE snapshotů **770864B**, MEMINFO.freemem32 **777680B** v jiném okamžiku;16bit freemem je saturované65535. `mem_available()` kromě heapů započítává také chCoreGetStatusX a případnou DMA rezervu (`malloc.c:422`), zatímco memory.txt vypisuje jednotlivé regiony. Rozdílná čísla se proto nepovažují automaticky za leak nebo nekonzistenci. Součet není největší souvislý blok; jednotlivé typy RAM nejsou zaměnitelné pro každou alokaci. FREE8B v prvním regionu samo není důkaz celkové nedostatečné paměti.

**Historické runtime minimum heapu není poskytované těmito rozhraními.** Nezaměňovat aktuální volné bloky, linker heap, statické frames a watermark. Jde o starý firmware90b24237 bez nového39KB Flight recorderu, nikoliv měření paměti kandidáta3G-2.

### Skutečné stack watermarky

| Vlákno | Priorita | Volný watermark [B] | Přiděleno [B] |
|---|---:|---:|---:|
| ISR | 255 | 1288 | 1536 |
| ArduCopter | 182 | 4344 | 7168 |
| idle | 1 | 288 | 496 |
| UART_RX | 60 | 864 | 1200 |
| OTG1 | 60 | 352 | 752 |
| monitor | 183 | 1096 | 1456 |
| timer | 181 | 1544 | 1968 |
| rcout | 181 | 504 | 944 |
| rcin | 177 | 840 | 1456 |
| io | 58 | 1544 | 2480 |
| storage | 59 | 960 | 1456 |
| UART4 | 60 | 328 | 752 |
| log_io | 59 | 1528 | 2016 |
| I2C0 | 176 | 984 | 1456 |
| OSD | 59 | 976 | 1712 |
| led | 60 | 280 | 688 |
| UART1 | 60 | 328 | 752 |
| I2C1 | 176 | 1000 | 1456 |
| SPI1 | 181 | 856 | 1456 |
| SPI4 | 181 | 808 | 1456 |
| utb_log | 58 | 2456 | 3504 |
| FTP | 58 | 1704 | 2992 |


Main **4344/7168B**, ISR **1288/1536B** volných. Oproti jinému bootu3F-3 je main stejný, ISR o8B větší pozorovaná rezerva; není z toho odvozena změna algoritmu nebo garantovaný letový limit. Watermark ukazuje dosavadní nejvyšší využití, ne okamžitou spotřebu konkrétní funkce. Nový firmware musí být změřen zvlášť.

### Původní logy a konečný health

Inventář obsahuje4 logy, velikosti2516746/7924746/718496/725746B, celkem **11885734B**. ID/časy/velikosti se shodují s uloženým koncem3F-3. Žádný BIN download, vynucené otevření nového logu, LOG_DISARMED1, mazání nebo formátování. Metadata nejsou bitový readback flash a neudávají přesnou aktuální použitelnou souvislou volnou kapacitu.

Po FTP/listingu zdravé gyro/accel/AHRS/logger a EKF attitude, errors_count1/2/3/4=0, DISARMED STABILIZE, uptime825713ms. errors_count3 je logger dropped counter, nikoliv InternalError maska. Závěrečný port je uzavřený a Windows stále uvádí COM5/stejnýserial/stavOK/ProblemCode0.

## Windows události a diagnostická kategorie

V dostupném System logu byly prověřené záznamy v50min okně včetně předchozího neúspěšného připojení a současného testu; závěrečný dotaz obsahoval12 událostí, žádnou vybranou PnP/USB/DriverFramework událost. Kernel-PnP/Configuration je zapnutý, bez záznamů v tomto okně. USB-USBHUB3/Operational není na tomto systému dostupný, DriverFrameworks-UserMode/Operational je vypnutý. Žádný kanál se nezapínal, oprávnění ani konfigurace se neměnily.

Nebyl doložen USB disconnect/reset, nový device arrival, COM reassignment, změna napájení nebo restart během nynějšího měření. Nedostupnost podrobnějších kanálů neprokazuje, že původní výpadek neměl USB příčinu. Pro předchozí ClearCommError není dost evidence k definitivnímu rozlišení kategoriíA/B/C/D; **kořenová příčina NEURČENO**. Aktuálně žádná z poruchových kategorií nebyla reprodukovaná.

## Přímé porovnání

| Charakteristika | Předchozí připojení3G-3 | Nynější původní port notebooku |
|---|---|---|
| Počáteční Windows detekce | Chyběla, později obnovena | Správná od začátku této fáze |
| USB/MAVLink identita | Potvrzená | Potvrzená znovu |
| Komunikace | Chyba po13,922s | 120,093s okno bez chyby, celá první relace166,359s |
| Serial API | ClearCommError/WinError22 | Bez SerialException |
| Parametry | Nezískané | 1215/1215 indexů doložených offline |
| USB události | Příčina neurčena | Ve dostupných kanálech žádné relevantní |
| Restart | Nepotvrzen | V pozorovaných datech nezjištěn |
| Systémové soubory | Nečtené | Sekvenční memory/threads/MEMINFO dokončené |

Není to řízený A/B důkaz vadného portu. Kromě fyzického zapojení se změnila příprava linek/úvodního baudrate a snížila četnost read-only požadavků (původně přibližně3/s, zde fallback nejvýše0,4/s). Výsledek neizoluje jejich jednotlivý vliv. Z jednoho úspěšného testu nelze prohlásit port, kabel, driver nebo firmware za definitivní příčinu či obecně bezchybný.

## Doporučení a konečný Git stav

**GO k samostatnému obnovení read-only preflightu3G-3**: identita správná, ≥120s bez sériové chyby, pravidelný heartbeat, bez zjištěného restartu/USB výpadku, úplné parametry a dokončené bezpečné read-only kontroly. Vlastní runner false-negative je objasněný a oddělený od zdraví USB; původní raw výsledek zůstává zachovaný.

Původní preflight má znovu ověřit aktuální konfiguraci/health, paměť a souhlas s přesně určeným firmwarem. **Žádný upload, další SHADOW nebo HW experiment není tímto spuštěn či schválen.** Nový39KB recorder a reálné časování kandidáta3G-2 zůstávají neověřené. Parametry/streamy nebyly nastavovány, proto nebylo co obnovovat; příchozí automatické STAT_RUNTIME/kalibrační aktualizace nejsou PARAM_SET runneru.

HEAD stále `11a518e5857b24cc554b3cb7e6492998013ddd1a`, větev test/skystars-5inch, index i sledované zdroje čisté. Zachován původní necommitnutý report3G-3 a kapitoly/historie.txt. Nový tento report zůstává necommitnutý, artefakty jen v `tmp/utb-3g3a-usb-20261025_091414/`. Žádný reset, clean, stash, rebase, commit ani push.

Hlavní evidence: windows-pnp-before/final.json, windows-driver.json, processes-before.json, windows-events-during/after/final.json, attempt.json, messages.jsonl, receive.raw, result.json, parameter-recovery.json, parameters-recovered.json/.param, finish-approval.json, finish-attempt.json, finish-messages.jsonl, finish-result.json, memory.txt, threads.txt a analysis.json. Původní chybný runner ani jeho traceback se nepřepisují. Přesné hashování artefaktů je v evidence-sha256.json.
