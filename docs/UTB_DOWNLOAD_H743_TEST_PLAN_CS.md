# FÁZE 1H-E: plán diagnostiky DataFlash/EKF na H743

Stav k 13. 10. 2026: **SOFTWARE VERIFIED** pro offline nástroj a jeho guardy.
Nový experiment na FC: **UNVERIFIED, neproveden**.
Větev `test/skystars-5inch`, HEAD `687f825d228b12a64d36651072aed19356178217`.
Poslední známý firmware FC `f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f`.

## 1. Výchozí důkazy a hranice závěru

[Audit FÁZE 1H-D](UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md) obsahuje úplnou source mapu,
časovou osu a původní důkazy. Tato fáze přidává experimentální nástroj a protokol.
Nepředkládá novou root cause analýzu jako hotový výsledek.

**HARDWARE VERIFIED v předchozí fázi:** maximum benchmarkové periody při
downloadu 41 932 a 43 499 µs; v plném transferu hlášení nezdravého EKF již během
LOG_DATA; následný flag 1024; po fyzickém studeném startu flag 167.
Tyto hodnoty nejsou bezpečnostní limity. Čítač UB29 je maximum periody hlavní
smyčky, nikoli doba jedné flash operace.

**SOURCE VERIFIED:** listing, metadata i raw čtení Block/JEDEC backendu mají
synchronní cesty v obsluze MAVLink. Relevantní source:

- `libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp:68` listing,
  `:107` DATA a metadata, `:215` send budget, `:296` raw čtení, `:339` konec.
- `libraries/AP_Logger/AP_Logger_Block.cpp:427` read semaphore a stránky,
  `:461` get_log_data, `:488` počet logů, `:602`, `:669`, `:715`, `:770` metadata.
- `libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp:92` WaitReady,
  `:180` status se SPI semaphore, `:224` PageToBuffer.
- `libraries/AP_HAL_ChibiOS/SPIDevice.cpp:204` čekání na SPI/DMA,
  `:336` bus semaphore. Flash SPI3 je od gyro SPI1/SPI4 oddělený;
  to nevylučuje blokování main threadu při synchronním čtení.
- `libraries/GCS_MAVLink/GCS_Common.cpp:1568` log send,
  `:4398` message handling; `ArduCopter/Copter.cpp:116` fast pořadí.
- `ArduCopter/utb_bench.cpp:67` exporter po 1 s; `:113` timing maxima,
  `:129` UBST a NAMED_VALUE_INT. Export není záznamem každé jednotlivé události.

Naměřená perioda dokládá zpomalení smyčky. Nedokládá, zda následnou změnu EKF
způsobilo právě čtení stránky, metadata scan, semaphore nebo transport.
Flag 1024 také sám neznamená, že se EKF nikdy neinicializoval:
`AP_AHRS.cpp:2956`, `AP_NavEKF3_Outputs.cpp:591` a
`AP_NavEKF3_Control.cpp:775` vážou reportované initialized/healthy stavy na
více interních podmínek. Jejich rozlišení není běžným MAVLink reportem dostupné.

## 2. Hypotézy a rozlišovací měření

Všechny kauzální hypotézy níže mají stav **HYPOTHESIS**.

| Hypotéza / mechanismus | Současný důkaz | Chybějící data | Nejmenší rozlišovací test | Očekávaná posloupnost | Falešná interpretace |
|---|---|---|---|---|---|
| Metadata scan / hledání hranic Block logu synchronně v main | Source cesty get_num_logs/get_log_info; maximum rostlo před prvním přijatým LOG_DATA | Délky jednotlivých metadatových volání na FC | Idle → samostatný LIST; potom jeden DATA na archivovaných krátkém a dlouhém logu | LIST/DATA request → metadata → delší perioda → teprve odpověď | Pozdní přijetí UB29 může referovat starší událost; délka logu není jediná změněná proměnná |
| Raw stránka / WaitReady / SPI transfer | Synchronní PageToBuffer, WaitReady a DMA wait v source | Začátek/konec konkrétní FC read operace a cache hit | Jeden omezený DATA, následně oddělené paced varianty stejného logu | Request → čtení → perioda → DATA | Host round trip obsahuje fronty a USB; není změřenou dobou flash read |
| Mutex contention main versus logger | Block a SPI semaphores; dosavadní write admission probe měří jiný zámek | Owner/wait/hold na Block semaphore a SPI device semaphore při downloadu | Nejprve LIST/DATA při LOG_DISARMED=0, bez UTB shadow | Request → čekání/zpracování → perioda; kolize by musela časově předcházet | Nula UB05 kolizí ani malé UB27/28 nevylučují blokování na neměřeném zámku |
| IMU/EKF deadline gap jako následek dlouhé main periody | Perioda desítky ms a unhealthy v jednom transferu | Skutečné INS/EKF update intervaly, core health reason a přesný začátek změny | Časově svázat UB29, EKF report/variances a STATUSTEXT se stejným DATA; případně později samostatně schválená úzká FC diagnostika | Delší perioda → výpadek/update age → health změna | EKF report bez timestampu dorazí pozdě; korelace není kauzalita |
| MAVLink obsluha / CPU / TX fronty | DATA běží přes GCS obsluhu; host pacing mění hustotu požadavků | Čas handleru, RX/TX queue occupancy, PM během LOG_OFF | Idle se stejnými read-only polling požadavky, LIST a stejné množství DATA s pacingem 20/100 ms | Hustší requests → host/FC fronty → případná perioda/health změna | Menší chunky mohou zvýšit metadata overhead; SYS_STATUS.load není PM.Load |
| USB/serial transport nebo souběh mechanismů | Krátký download skončil ClearCommError; boot counter se nezměnil | TX timestamps, raw wire a přesná chyba driveru; kontinuita FC během odpojení | Bounded tool s oddělenými TX/RX a OSError záznamy, bez reconnect | Transport chyba může předcházet, následovat či být nezávislá na EKF změně | COM chyba není důkaz rebootu; při výpadku linky nelze tvrdit zdravý ani nezdravý FC |

Pokud host měření odliší LIST od DATA, lokalizuje problém na širší source větev.
K přesné funkci/zámku může být stále nutný samostatný diagnostický firmware.
Ten zde není implementován ani schválen. Měl by přidat jen čas vstup/výstup a
výsledek vybraných read/metadatových operací a interní health reason, s bounded
bufferem a exportem mimo měřený zámek; teprve po reprodukci a rozboru overheadu.

## 3. Co lze skutečně pozorovat

| Veličina | Zdroj / časová báze | Význam a omezení |
|---|---|---|
| Host send attempt / send completed | `time.monotonic()` a samostatně `time.time()` | Začátek a návrat host zápisu, nikoli přijetí požadavku FC |
| RX a raw MAVLink | receive.tlog, receive-wire.bin, timeline.jsonl | Host přijetí zahrnuje buffering; originální wire zůstává zachován |
| Skutečný TX wire | transmit.tlog | Zprávy po úspěšném host write; short write/-1 je chyba, nikoli úspěšné odeslání |
| FC boot čas | time_boot_ms, např. UBxx | Čas exportu zprávy; 32bit wrap není sám reboot. Různé zprávy mohou obsahovat různě staré samples |
| FC TimeUS/time_usec | Pouze pokud ho konkrétní zpráva nese | Nástroj uchová hodnotu, nedoplňuje smyšlený timestamp do EKF_STATUS_REPORT |
| UB08/UB09/UB29 | NAMED_VALUE_INT | Task budget overruns / period>nominal čítač / kumulativní maximum periody; žádný z nich není konkrétní flash read duration |
| UB27/UB28 | NAMED_VALUE_INT | Maximum měřeného main acquire / worker hold pro logger write admission probe; nikoli všechny read/SPI mutexy |
| UB22/23/24/25, lifecycle a ostatní UBxx | NAMED_VALUE_INT | Existující diagnostika; uchovány všechny doručené klíče, chybějící export znamená UNKNOWN |
| EKF flags, variances | EKF_STATUS_REPORT | Reportovaný stav, ne root cause uvnitř konkrétního EKF core |
| HEARTBEAT, STAT_BOOTCNT, AUTOPILOT_VERSION | MAVLink / PARAM_REQUEST_READ | DISARMED, mód, identita, kontinuita startu; nikoli důkaz absence každého interního resetu |
| SYS_STATUS.load | MAVLink | Load v desetinách procent; uvádět pod tímto názvem, nepřeznačovat na DataFlash PM.Load |
| PM.NLon / PM.MaxT / histogram každé periody | Při tomto LOG_OFF experimentu běžnou telemetrií nedostupné | `pm_nlon=null`; existující BIN PM lze použít historicky, ne pro novou session |
| Délka konkrétní flash operace / read mutex latency | Aktuálním fw přímo neposkytovaná | `flash_operation_us=null`; nesmí se odvozovat přesně z host round trip |

EKF_STATUS_REPORT nemá vlastní sample timestamp; čerstvý host receive není
důkaz přesného času vzniku EKF sample. REQUEST_MESSAGE a zachované pořadí
omezují nejasnost, ale neodstraňují buffering.

Ubxx se exportují přibližně po 1 s, ale transport může zprávy zahodit.
Maximum vzniklé uvnitř této sekundy nemá vlastní event timestamp. Časové
zarovnání proto uvádět jako interval s nejistotou fronty/linky. Offset
host_receive − FC_boot není přesný boot wall time. Ve starých tlog chybí TX;
replay je výslovně označen `archive_elapsed_s`, nikoli skutečný monotónní čas.

## 4. Host nástroj a ochrany

`Tools/autotest/utb_download_experiment.py` používá původní `TransferPlan`
z `utb_download_audit.py`; nepřidává download protokol ani logger backend.

Před LIST/DATA ověří:

- FC source system/component 1/1, board ID 1075, vendor 4617, product 22337,
  celých 18 B UID2 a `flight_sw_version=67568127`.
- `flight_custom_version=f9cc3bcc` a stabilní identitu v dalších reportech.
  AUTOPILOT_VERSION poskytuje jen 8 znaků source prefixu; celý SHA-256 nahraného
  APJ nelze touto cestou kryptograficky přečíst. Ověření plného artefaktu zůstává
  archivním/administrativním důkazem, nikoli výsledkem MAVLink readbacku.
- HEARTBEAT multirotor/ArduPilot, DISARMED, stabilní mód odlišný od UTB_ACRO,
  ne critical/emergency/poweroff; čerstvé heartbeat a EKF reporty.
- EKF attitude flag a nepřítomnost flagu 1024; boot counter je přečtený a stálý.
- `UTB_SHADOW=0`, `UTB_B_SYNTH=0`, `LOG_DISARMED=0`, `FRAME_CLASS=1`,
  `FRAME_TYPE=18`, `FSTRATE_ENABLE=0`, `LOG_BACKEND_TYPE=4` a dostupnost UB29.
  Jde o read-only podmínky. Neshoda experiment zastaví; nástroj hodnoty nenastavuje.

Live transport se vytvoří teprve po validaci samostatného approval JSON:
approved/props_removed/exclusive_port=true, port COM5, časově omezené schválení
a SHA-256 přesného plánu. Digest je
`sha256(json.dumps(plan, sort_keys=True).encode())`.
Tento soubor je kontrola workflow, nikoli kryptografický podpis oprávnění.
Dodané `approval-NOT-GRANTED.json` je false a expirované; port neotevře.

Host journal overhead před send a transport send call se ukládají odděleně.
Transport call zahrnuje MAVLink encoding i TX tlog flush; není čistým FC časem.
Serial write timeout je 0,5 s, každý send znovu kontroluje celkový deadline.
Host filesystem flush není hard real-time operace; deadline může být při
blokujícím host I/O překročen. Jde o bounded experiment policy, nikoli hard
deadline guarantee. Žádný nový zápis nebo blocking API nevzniká ve FC fast path.

Povolené výstupy jsou jen GCS HEARTBEAT, REQUEST_MESSAGE (148/193/1),
PARAM_REQUEST_READ, LOG_REQUEST_LIST/DATA/END. REQUEST_MESSAGE používá standardní
MAV_CMD_REQUEST_MESSAGE, bez změny stream rate. Neexistuje PARAM_SET, ARM,
RC_OVERRIDE, motor test, reboot, erase ani auto reconnect. GCS heartbeat a tři
read requests + boot counter běží po 1 s stejně v idle i DATA scénářích.
Nejde tedy o nulovou host zátěž; její TX je dohledatelný v timeline a tlog.

Jen jeden pending DATA range. Limity implementace: chunk 90–8190 B po násobcích
90, nejvýše 8 chunků, 0–2 timeout retries jednoho range, nejvýše 180 s.
Dodané fyzické plány používají **retries=0**, takže chyba neopakuje experiment.
Chunk 8190 B omezuje přenos na 91 MAVLink LOG_DATA payloadů a zachovává dříve
použitou velikost pro srovnání. Nejde o prokázanou bezpečnou velikost.

Timeout, OSError, KeyboardInterrupt, ztráta fresh telemetry, health/identity
změna, malformed wire, neúplný či konfliktní payload zastaví další DATA/LIST.
Nástroj pošle LOG_REQUEST_END, pokud linka ještě umožňuje zápis, a uloží partial
BIN spolu s byte-validity bitmapou. Nuly v neobdržené části nejsou platná data.
Přijatý prefix musí odpovídat předem hashově ověřenému archivnímu BIN.
Pokus o opakované `Engine.run()` je odmítnut. Případná recovery observation
po stopu nic znovu nestahuje, neobchází health guard pro další experiment.

Zastavení hostu **nedokáže přerušit již běžící synchronní funkci na FC**.
LOG_REQUEST_END bude zpracován až poté, co FC obslouží zprávu. Guardy omezují
další expozici a uchovávají důkazy; negarantují deadline běžící flash operace.

## 5. Minimální experimentální matice

Všechno pouze bez vrtulí, DISARMED, v klidu, USB bez motorového napájení.
Před každou session musí být FC zdravý a inicializovaný; pouhá přítomnost USB
nestačí. Každý řádek má samostatný plan a samostatný běh nástroje.

| Plan | Baseline / recovery | Maximum LIST / DATA requests | Maximum unikátních dat | Celkový host limit | Účel / interpretace |
|---|---|---|---|---|---|
| idle | 60 / 20 s | 0 / 0 | 0 B | 100 s včetně preflight | Reference se stejným telemetry pollingem; případná health změna není podmíněna downloadem |
| list | 60 / 20 s | 1 / 0 | 0 B | 100 s | Oddělit metadata větev; listing může blokovat i bez LOG_DATA |
| log2-one | 60 / 20 s | 1 / 1 | 8190 B | 100 s | První omezený raw request na již archivovaném krátkém logu |
| log3-one | 60 / 20 s | 1 / 1 | 8190 B | 100 s | Srovnání s dlouhým logem, pokud stále existuje stejný ID/size/time |
| log2-paced20 / paced100 | 60 / 20 s | 1 / 4 | 32 760 B | 100 s | Stejný prefix a objem, 20/100 ms pauza až po úplném chunku |
| log3-paced20 / paced100 | 60 / 20 s | 1 / 4 | 32 760 B | 100 s | Totéž u delšího logu; ne automatické pokračování po chybném prvním requestu |

Preflight max 10 s, request timeout 2 s, retry 0. Limity jsou **návrhové bounds
pro malý experiment**, ne naměřené bezpečnostní limity. 60 s baseline dává
opakované 1Hz exporty; 20 s recovery je omezené pozorování, ne důkaz definitivní
obnovy EKF. Timeout 2 s hodnotí host dostupnost odpovědi, nikoli hard real-time FC.
32 760 B je pod 1 % dlouhého archivního logu, ne požadavek na volné médium.

Dokud není schválen a vyhodnocen idle a LIST, není schválen DATA. První DATA
má jeden request. Paced a long-log srovnání jsou další samostatná rozhodnutí.
Malé chunky se zde nedoporučují jako obecná oprava: opakují metadata/hranice
a mohou celkový overhead zhoršit. Žádný celý dlouhý log se nyní nestahuje.

Archivní identifikátory v plánech nejsou tvrzením o aktuálním obsahu FC:

| BIN | ID / size / UTC | SHA-256 |
|---|---|---|
| SYNTHETIC_SHORT.bin | 2 / 2 569 496 / 1791811304 | d45167bd5832efb08cf2b0350fd1a9cd29388a48828aa4ffa0b2bd0bc63f040e |
| SYNTHETIC_FULL_400.bin | 3 / 4 349 246 / 1791812260 | 0677617bfac02af3f3b44d0038bb614ed147e1eeb3812d0b96b80d6e565786b6 |

Pokud listing neodpovídá, DATA se neprovede. Nemá-li zdravá aktuální session
stejnou konfiguraci a logy, protokol se upraví a znovu předloží; žádné automatické
nahrání fw, změna parametrů, vytvoření nového dlouhého logu nebo mazání.

## 6. Akceptace, stop a nový start

Akceptace jednotlivého běhu znamená kompletní očekávaný prefix, všechny guardy
platné, žádnou transport chybu, jednoznačnou uloženou identitu a timeline.
Takové PASS není schválení letové bezpečnosti ani vyloučení blocking rizika.

Okamžitě zastavit další requests při:

- ARMED, mód UTB_ACRO, jiné letové chování, změna módu či identity/boot counteru;
- EKF bez attitude / s 1024, unhealthy/failsafe/watchdog/flash/SPI STATUSTEXT;
- výpadku fresh HEARTBEAT/EKF >3 s; při 1Hz pollingu jde o ztrátu několika
  očekávaných reportů a pozorovatelnosti, nikoli o tolerovanou dobu flight outage;
- jakékoli COM chybě, malformed wire, změně LOG_ENTRY či odlišném archivním prefixu;
- novém kumulativním UB29 maximu po začátku DATA proti konci baseline;
  guard je konzervativní i vůči běžnému jitteru, nepřiřazuje příčinu;
- vypršení request/byte/time bounds nebo ručním Ctrl+C.

UB29, které už obsahuje staré vyšší maximum, může nový menší spike skrýt.
Nula nových rekordů proto neznamená nulové blocking riziko. Telemetrie také
může novou hodnotu exportovat až po dokončení bounded transferu.
Tyto limity se nesmí nahrazovat libovolným „bezpečným“ prahem desítek ms.

Po unhealthy/timeout/transport chybě se neprovádí automatický další běh,
reconnect ani soft reboot. Omezit pozorování na recovery a uložit důkazy.
Další pokus vyžaduje samostatné rozhodnutí a nový potvrzený zdravý fyzický start.
Pro srovnání kumulativních maxim je nový zdravý start vhodný i mezi úspěšnými
variantami; nezahajovat ho automaticky. Finální coldboot z minulého auditu je
historický důkaz, ne potvrzení dnešního stavu FC.

## 7. Offline reprodukce a budoucí spuštění

Výsledky této fáze: `tmp/utb-1he-offline-20261013_151020/`.
Finální replays mají explicitní clock provenance v `replay-with-clock-provenance/`.
Reprodukční driver vytvoří nový adresář a původní archive hashově zkontroluje:

```sh
cd /home/jirka/ardupilot_utb
.venv/bin/python -B Tools/autotest/utb_1he_offline.py \
  --archive-root tmp/utb-synthetic-h743-20261012_150527 \
  --output tmp/utb-1he-preparation-NEW
.venv/bin/python -B -m unittest discover -s Tools/autotest/unittest \
  -p test_utb_download_experiment.py -v
```

Pro fyzickou session bude předložen konkrétní nezměněný plan, archivní SHA,
port COM5, potvrzení vrtulí/DISARMED/volného portu, délka a stop/cleanup postup.
Teprve samostatné schválení vytvoří platné approval. Nyní neexistuje.
Budoucí command, **nyní nespouštět**:

```powershell
python -B Tools/autotest/utb_download_experiment.py --live `
  --plan <approved-plan.json> --approval <separate-approval.json> `
  --archive <already-verified-BIN> --output <new-session-directory>
```

Na Windows vybrat již existující Python s pymavlink/pyserial. Nic zde nebylo
instalováno. WSL COM5 není automaticky Windows COM5; live command musí běžet
na potvrzeném Windows hostu. Žádná port enumeration ani port open zde neproběhly.

## 8. DataFlash kapacita a případné uvolnění

Historických přibližně 1,66 MB nelze použít jako aktuální free-space measurement.
LOG_DISARMED=0 a SHADOW/SYNTH=0 jsou preflight podmínky, nikoli nové zápisy.
Existující logy se jen read-only porovnávají s archivem; žádný chip erase ani
nové dlouhé logování není součástí experimentu.

Pokud bude někdy nutné médium uvolnit: nejprve read-only kompletní inventura,
bezpečně schválené stažení všech dosud nearchivovaných logů, ověření délky a
SHA-256 každého BIN, druhá kopie, porovnání inventury. Potom samostatně schválit
přesný erase rozsah a obnovit read-only health/inventuru. Tato etapa takové
mazání neprovádí ani neuděluje oprávnění k jeho automatickému provedení.
