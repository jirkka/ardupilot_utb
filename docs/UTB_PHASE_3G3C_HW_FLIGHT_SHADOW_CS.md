# FÁZE 3G-3C – První HW validace pasivního UTB Flight SHADOW

Datum měření 25. 10. 2026 (Europe/Prague, CET/UTC+1). Závěrečné offline vyhodnocení navazuje na uložený jediný experiment; po pauze nebyl znovu otevřen COM ani opakován test.

## Závěr a rozsah

**PASS pro jeden krátký USB-only DISARMED Flight SHADOW na H743 v podporovaném klidovém profilu.** Jediný schválený upload prošel standardní CRC verifikací; nový boot zdravý. Jediný Flight běh dokončil **4000 main pokusů, 1 warmup a 3999 validních paired Rate kroků**, **8000 validních Attitude operací**. Všech **84 exportovaných max. absolutních chyb i RMSE je 0**, mismatch/invalid/DESYNC/unsupported/missing/duplicate a motor_changes jsou 0. Uloženo **100 raw řádků**, osm stránek kompletně exportováno a ověřeno původním analyzátorem.

CPU **18,8267 → 28,7278 → 18,7412 %**. Rate capture perioda průměr **2500 µs**, min/max **2453/2550 µs**, průměrný absolutní jitter **1,69685 µs**. Součet nepřekrývajících se měřených přidaných bracketů **190,11525 µs/main**; nejde o úplnou režii ani WCET. Main stack **4312/7168 B volných** před i po, bez pozorované chyby alokace či ztráty recorderu/exportu.

**Tento výsledek nepovoluje let.** Neověřuje aktivní UTB řízení, ARMED HW historii, letovou dynamiku, nezávislou nenulovou I akumulaci, trvalé logování, nejhorší stack nebo Main deadline. Další měření není automaticky spuštěno.

## Fyzické podmínky a dvě schválení

Před prvním otevřením portu znovu potvrzeno USB-only napájení v osvědčeném portu notebooku, stejný kabel, odpojená LiPo, všechny vrtule demontované, DISARMED a zavřené ostatní sériové aplikace. Žádný ARM, motor test, PWM/DShot příkaz ani fyzický let. Stav zachycuje physical-confirmation.json.

**Brána A:** výslovně schválen právě jeden upload přesného APJ včetně standardního vstupu do bootloaderu a restartu uploaderem, poté pouze kontrola; approval-a.json. Po úspěšné samostatné post-upload kontrole byla **brána B** zvlášť výslovně schválena: jediný UCS_F_RUN=1/COUNT=4000/DECIM=40 s dočasnou telemetrií2/1/1Hz a obnovením; approval-b.json. Souhlas s uploadem nebyl použit jako náhrada brány B.

Bez nových instalací, sestavení, úprav produkčních C++, změn PID, bootloader update, DFU recovery, mazání logů, automatického druhého pokusu, commitu nebo push. Použity existující pyserial/pymavlink a původní ArduPilot uploader; runner vychází z úspěšných předchozích protokolů. Nové skripty pouze v ignorovaném tmp.

## Firmware, identita a skutečný upload

COM5, VID:PID **1209:5741**, USB serial **200024001251303433333336**, MAVLink **1/1**, board ID **1075**. Před uploadem Copter **4.7.1/90b24237**, boot110; po uploadu **4.7.1/d78e34b3**, boot111. Bootloader potvrdil protocol5, board_rev0, serial a **STM32H743/753 Rev.V**, aplikace kapacita1703936B.

Přesně použitý firmware:

`/home/jirka/ardupilot_utb/tmp/utb-3g2-20261025_071149/firmware/SkystarsH7HD-bdshot_UTB3G2_PASSIVE_RAM_918dedaaeae9.apj`

- Velikost **1 155 815 B**; SHA-256 **`918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7`**.
- APJ board1075, git_identity d78e34b3 je očekávaná předimplementační build identita. Integrita je posuzována současně SHA a původním 15souborovým zdrojovým manifestem.
- Flight/Chain/Timing ON; aktivní UTB Core/Output/Acro/Angle na H743 compile-out. RUN default0 potvrzen skutečnými parametry po bootu.
- Jediný standardní upload: identify → erase/program aplikace → Verify100% → uploader reboot. Standardní CRC prošlo; nejde o kryptografický SHA readback MCU. Bootloader, konfigurace a logy nebyly záměrně přepisovány; následné parametry a log inventory jejich dostupnost potvrdily.

Úplný [výstup uploaderu](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/upload-console.txt), [výsledek uploadu](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/upload/result.json). Bootloader identifikační polling během standardního přechodu nebyl druhým erase/program pokusem. Další upload se neprovedl.

## Preflight a nový boot

Před uploadem read-only health **40,016s**, kompletní záloha **1215/1215**, bez parser/COM chyby a pozorovaného restartu. Jediná změna proti3G-3B byl STAT_RUNTIME. Původní čtyři logy metadaty shodné.

Po standardním rebootu vyčkáno alespoň30s na inicializaci. Samostatné health okno **40,047s**, celá relace **59,890s**, záloha **1220/1220 indexů** bez duplicit. Přibyly pouze UCS_F_RUN/COUNT/DECIM a UTT_RUN/COUNT. Počty byly odvozeny ze skutečného param_count; neindexované65535 notifikace nebyly považovány za další indexy.

DISARMED native STABILIZE, SCHED_LOOP_RATE400, FSTRATE_ENABLE0, bez rate threadu, SMAX/NTF/NEF0 na všech osách. Všechny RUN0, LOG_DISARMED0, SCHED_OPTIONS0. Původní PID/ATC, filtry, RC/failsafe, motor/servo a bateriové parametry nezměněné. Startup gyro offsety/teploty, baro tlak, STAT_BOOTCNT/RUNTIME a malá změna automatické COMPASS_DEC jsou přesně uvedené v [post-upload porovnání](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/gate-b-preflight.json); nic nebylo automaticky opravováno.

Gyro/accel/AHRS a logger health, EKF attitude valid; InternalError masky/počet a logger dropped0. PreArm RC not found a nízké napětí baterie odpovídají nepřipojenému RC/LiPo. COMPASS_USE/USE2/USE3 zůstává0, AUTODEC1: absolutní yaw/heading není kvalifikován. Žádné obcházení armingu. Pending metadata byla **UTBFlightV1,pending**; COUNT24000/DECIM200, RUN0. Skutečná post-boot paměť je uvedena níže.

## Jediný měřicí protokol a audit odchozích zpráv

Měření začalo 25.10.2026 kolem10:07 CET; raw telemetry má přesné UTC a host monotónní čas. Jediný trigger **2026-10-25T09:07:43.616516Z** (10:07:43,616516 CET). Schválení bylo před měřením ověřeno souborem approval-b.json a novým exclusive experiment-attempt.json; opakování RUN je zablokované.

Před měřením načtena identita, RUN/konfigurace, pending a memory/threads. COUNT24000→4000, DECIM200→40. Původní intervaly pro SYS_STATUS/SYSTEM_TIME/EKF byly **−1/−1/−1 µs**, dočasně500000/1000000/1000000µs. Po3s ustálení **15,032s baseline**. Pak právě jeden PARAM_SET UCS_F_RUN1. Během stage measurement byl jediným odchozím paketem tento trigger; žádný FTP, polling, PARAM_REQUEST_LIST, log listing nebo tasks.txt. Následovalo přibližně11s pouze příjem a potom ověření automatického RUN0,9s stejné telemetrie, frozen memory/threads a export.

Skutečné časové značky firmware: start **246332150 µs**, stop **256330219 µs**, rozpětí **9,998069s**. Jde o čas mezi startem a koncem konečné sekvence, nikoliv důkaz délky4000 přesných intervalů. Poslední příznak stop reason **1 – dosažen počet**.

Celá měřicí relace včetně exportu/cleanup **58,859s**, parser receive errors0, pozorované sekvenční anomálie **0**; heartbeatů **58**, max host mezera **1.063s**. Bez poklesu SYSTEM_TIME, ARM, změny módu, InternalError, senzorové chyby nebo logger drops v přijaté telemetrii. Žádné fyzické měření elektrických výstupů ESC.

## Numerika, pairing a vlastní historie

| Veličina | Skutečný výsledek |
|---|---:|
| Main attempts / warmup / paired | 4000 / 1 / 3999 |
| Primary Attitude / yaw reset operace | 4000 / 4000 |
| Valid Attitude / Rate | 8000 / 3999 |
| Missing / duplicate / invalid / unsupported | 0 / 0 / 0 / 0 |
| Attitude a Rate DESYNC / mismatch | vše0 |
| Upstream/downstream mismatch | 0 / 0 |
| Počáteční plné Attitude / Rate synchronizace | 1 / 1 |
| Externí I_set replay / PID filter reset | 11994 / 0 |
| Mode/config changes | 0 |
| Motor checks / motor changes | 12000 / 0 |
| Epochy / ARM změny / mode exits | 1 / 0 / 0 |

Všech3999 paired kroků prošlo N→N+1 runtime guardem. Poslední plný metadata pár **source100570 → consume100571**, Rate sequence1→3999; tento konečný pár není domýšlen z decimovaných řádků. Vlastní historie nebyla každý krok importována z AP; skutečné4000 yaw resetů a11994 I_set událostí byly replayovány. **independent_i=0, shaping=0**, FF/D_FF peak0: tento klidový DISARMED profil neprokazuje nenulovou I akumulaci, dynamický shaping nebo nenulové feedforward chování na HW. Dynamické předchozí SITL výsledky se nevydávají za nové HW měření.

| Skupina | Metrik | Porovnání na metriku | Max. abs. chyba | RMSE |
|---|---:|---:|---:|---:|
| Attitude | 26 | 8000 validních operací | 0 | 0 |
| Upstream | 4 | 3999 paired kroků | 0 | 0 |
| Rate/PID | 54 (18×3) | 3999 na osu | 0 | 0 |

Úplný seznam všech84 metrik s názvy, osou, počtem, max/RMSE: [metrics-full.csv](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/metrics-full.csv). Hex float64 metriky dekóduje původní analyzátor; všechny exportované hodnoty jsou0. Attitude mismatch používá implementovanou toleranci, proto není tvrzena obecná bitová totožnost všech stavů. Upstream/Rate mají vlastní strict compare. Raw float32 řádky jsou konečné a porovnávané target/PID projekce odpovídají; nevydávají se za plnou historii všech4000 kroků.

## Recorder a kompletní export

4000 candidates =100 selected +3900 decimated; kapacita128, rows100, overwritten0. Nejsou vyřazované chybné sady: všechny4000 pokusy jsou v plnorate agregovaných čítačích. DECIM40 pouze omezuje průběhy na přibližně10Hz.

Sekvenčně staženo metadata a všech8 stránek, pouze jeden otevřený FTP soubor současně. Stránky0–5 mají po16 řádcích, stránka6 má4, stránka7 platná a prázdná (jen hlavička). Page/session start_us, pořadí ID1–100, unikátní Loop, source/consume guard, finite float32/64, agregované čítače a všech84 metrik prošly původním analyzátorem. Metadata+stránky **81060B**, součet host časů přenosů **1.220s**; host export čas není čas jediné FC funkce. MAVFTP upozornění expected100000/got actual je normální konec virtuálního souboru, export byl následně obsahově ověřen.

CLI (provedeno offline nad skutečnými uloženými daty):

```text
Tools/utb/tools/utb_flight_shadow_analysis.py --metadata tmp/utb-3g3c-hw-20261025_100008/measurement/utb_flight.txt --pages tmp/utb-3g3c-hw-20261025_100008/measurement --output tmp/utb-3g3c-hw-20261025_100008/measurement/analysis --plot
```

[Původní metadata](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/utb_flight.txt), [původní analyzátor JSON](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/summary.json), [dekódované skutečné raw řádky](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/rows.csv). Původní stránky utb_flight0.csv až7.csv jsou zachované v measurement/. Poslední uložený decimovaný řádek nemusí být4000. krok; konečný plný pár je doložen zvlášť metadaty.

## CPU a skutečné časování H743

| Okno | SYS_STATUS n | Průměr % | Min–max % |
|---|---:|---:|---:|
| baseline | 30 | 18.826667 | 18.5–19.1 |
| active_interior_estimated | 18 | 28.727778 | 28.5–29.0 |
| after_before_export | 17 | 18.741176 | 18.4–19.0 |

Aktivní statistika používá vnitřní host okno trigger+0,5 až+9,5s, aby se omezilo přechodové zprůměrování scheduler load. Hranice jsou odhad přiřazení telemetrie; raw čas firmware a host nejsou zaměňovány. CPU nárůst asi9,90 procentního bodu není přesná cena jedné funkce.

Rate capture:3998 měřených intervalů, součet9995000µs, průměr2500µs (400Hz), min2453/max2550µs, průměrný absolutní jitter1,696848µs. Toto není Main perioda ani úplný hard deadline audit. Jednotlivé400Hz intervaly nejsou exportované, proto se nevymýšlí medián/P95/P99 nebo průběh na400Hz.

| Bracket | Počet | Průměr/call µs | Průměr/main µs | Exportované max µs | Vztah |
|---|---:|---:|---:|---:|---|
| Attitude pre-capture | 8000 | 4.013500 | 8.027000 | neexportováno | standalone |
| Attitude post-capture excluding consume | 8000 | 3.154125 | 6.308250 | neexportováno | standalone |
| Attitude consume | 8000 | 56.594750 | 113.189500 | 214 | standalone |
| Rate process | 3999 | 62.153788 | 62.138250 | 74 | standalone |
| Flight recorder | 4000 | 0.452250 | 0.452250 | 49 | standalone |
| Attitude math | 8000 | 18.066750 | 36.133500 | neexportováno | nested in Attitude consume |
| Attitude sync/validation | 8000 | 18.504500 | 37.009000 | neexportováno | nested in Attitude consume |
| Attitude compare | 8000 | 9.691500 | 19.383000 | neexportováno | nested in Attitude consume |
| PID math | 3999 | 7.920730 | 7.918750 | neexportováno | nested in Rate process |
| PID consume | 3999 | 60.633408 | 60.618250 | neexportováno | nested in Rate process |
| Native Rate + capture | 3999 | 16.423606 | 16.419500 | neexportováno | includes native AP; not incremental UTB overhead |

Nepřekrývající se pre + post_without_chain_att + att_consume + rate_process **189,663µs/main**; samostatný recorder **0,45225µs/main**, jeho naměřené maximum49µs, dohromady **190,11525µs/main**. Vnořené sync/math/compare/consume se znovu nepřičítají. Native Rate+capture zahrnuje původní AP a není přidaná režie UTB. Zbytek hook/guard/finish režie není úplně pokrytý; součet není WCET a nesčítají se jednotlivá maxima jako současný nejhorší cyklus.

Proti skutečnému3F-3: měřené brackety191,7865µs/main, CPU28,1722%, jitter1,73487µs. Nyní190,11525µs/main (včetně recorderu),28,7278%,1,69685µs. Jde o odlišné boot/firmware/podmínky, **nikoliv izolovaný A/B důkaz ceny Flight nebo zlepšení výkonu**.

UTT_RUN=0 po celou dobu. Main/Attitude entry periody, kompletní deadline/WCET, PM.NLon/MaxT a per-task UBST8 nejsou tímto experimentem měřeny. Neukazuje se PASS pro tyto veličiny a další experiment kvůli nim nebyl spuštěn.

## Runtime paměť a stack

Skutečné FREE/LRG v B; žádné odečítání recorderu od celkového heapu starého firmwaru:

| Region | Nový boot FREE/LRG | Před RUN | Po výpočtu, před exportem | Po exportu |
|---|---:|---:|---:|---:|
| 0x30000000 | 3688/3640 | 3688/3640 | 3688/3640 | 5576/4192 |
| 0x20000000 | 47944/47944 | 47944/47944 | 47944/47944 | 47944/47944 |
| 0x24000000 | 524280/524280 | 524280/524280 | 524280/524280 | 524280/524280 |
| 0x00000400 | 64504/64504 | 64504/64504 | 64504/64504 | 64504/64504 |
| 0x30040000 | 32760/32760 | 32760/32760 | 32760/32760 | 32760/32760 |
| 0x38000000 | 54536/53816 | 54536/53816 | 54536/53816 | 54536/53816 |

Součet FREE727712B před a po výpočtu,729600B po exportu; největší jednotlivý blok524280B v0x24000000. V0x20000000 zůstalo47944B souvisle, v0x30000000 největší blok3640→4192B. Záznamy v jednotlivých regionech nejsou jeden souvislý/dostupný DMA heap. Export nezpůsobil v těchto snapshotech pokles FREE/LRG; přesná příčina růstu není měřena. Není tvrzeno historické minimum ani důkaz absence úniku v dlouhém provozu.

| Vlákno, watermark volno/přiděleno B | Nový boot | Před RUN | Po výpočtu | Po exportu |
|---|---:|---:|---:|---:|
| ArduCopter | 4312/7168 | 4312/7168 | 4312/7168 | 4312/7168 |
| ISR | 1288/1536 | 1288/1536 | 1288/1536 | 1288/1536 |
| FTP | 1592/2992 | 1592/2992 | 1584/2992 | 1168/2992 |
| utb_log | 2512/3504 | 2480/3504 | 2480/3504 | 2480/3504 |

Main4312/7168B bez dalšího poklesu; proti starému FW4344B rozdíl−32B. FTP1592→1584→1168B po exportu, bez pozorovaného overflow; nejhlubší pozorované využitíFTP1824/2992B. Tuto rezervu nelze označit za obecně dostatečnou pro všechny exportní/letové stavy. ISR1288/1536B a utb_log2480/3504B v okně stabilní. Stack watermark není okamžité využití, bezpečnostní mez ani transitive call-chain maximum.

Statický recorder38912B, Flight .bss přírůstek39016B a Timing další1880B jsou z dřívějšího buildu; linker heap76728B není zdejší runtime FREE. MEMINFO po bootu737840B se s regiony nesčítá; historické heap minimum není dostupné.

## Motorová izolace, logy a konečný stav

Původní ArduPilot zůstal jediným vlastníkem motorů. Zachovaný zdrojový/binary audit kandidáta: aktivní UTB Core/Output/Acro/Angle na H743 chybí, recorder používá hodnotové kopie a nemá motorový sink ani zápis do živé AP PID historie. Všech15 source hashů zůstává shodných s ověřeným3G-2 kandidátem. Runtime **12000 motor checks /0 změn**. TX audit nemá ARM/motorové příkazy a finální motor/servo/RC/failsafe konfigurace se nezměnila. RAM audit není elektrické měření ESC.

| Původní log ID | UTC metadata | Velikost B |
|---|---:|---:|
| 1 | 0 | 2516746 |
| 2 | 0 | 7924746 |
| 3 | 0 | 718496 |
| 4 | 1792564233 | 725746 |

Celkem4 logy /11885734B; přesná shoda metadat před uploadem, po bootu, po experimentu i při konečné kontrole. Bez mazání, nového DataFlash logu nebo celého BIN downloadu. Inventář není bitové ověření flash. Logger drops a InternalError0 v dostupných zprávách.

Cleanup ověřil UCS_F_RUN0, COUNT24000, DECIM200; všechny ostatní RUN0, LOG_DISARMED0, SCHED_OPTIONS0. Telemetrické intervaly−1/−1/−1 obnoveny a přečteny zpět. Finální nezávislá read-only záloha **1220/1220**; proti post-upload pouze STAT_RUNTIME162987→163233. Boot111, firmwared78e34b3, poslední přijatý DISARMED STABILIZE, gyro/accel/AHRS/logger health a EKF attitude valid. Port uzavřen. Konečná relace měla8,094s úvodní health pozorování (zděděný název booleanu stable_40s v runneru zde neznamená40s), celkem28,187s. Bez dalšího experimentu a restartu.

## Grafy a data

- [CPU před / během / po](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/cpu.png) a [CSV](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/cpu.csv).
- [3F-3 Chain proti3G-3C Flight](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/chain-flight-comparison.png).
- [Stack watermark](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/stack.png) a [CSV](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/stack-watermark.csv).
- [FREE/LRG po jednotlivých regionech](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/memory-regions.png) a [CSV](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/memory-regions.csv).
- [Native/UTB body-rate targets](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/body-targets.png), [PID output](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/pid-outputs.png), [I](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/integrator.png), [FF/D_FF](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/feedforward.png).
- [84 numerických max/RMSE](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/numerical-errors.png), [Skutečné decimované sekvence a timestampy](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/measurement/analysis/timing-records.png).
- [Naměřené časové brackety](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/timing-brackets.csv), [souhrnný JSON](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/summary.json), [audit integrity](/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008/analysis/integrity.json).

Grafy obsahují pouze skutečně získané hodnoty. Raw průběhy jsou decimované; přímky spojují uložené body, nerekonstruují chybějící400Hz vzorky. Nulové I/FF/D_FF průběhy jsou skutečným omezením tohoto profilu.

## Odpovědi a zbývající podmínky před případným letem

1. Upload: **ANO**, právě jeden schválený APJ, standardní CRC PASS.
2. Boot: **zdravý v pozorovaných kontrolách**, identita a boot111 doložené.
3. Konfigurace: **zachovaná**, přesné startup/statistické změny oddělené.
4. Paměť: skutečné regiony/stack výše, bez pozorované chyby, bez záruky nejhorší rezervy.
5. Flight spuštěn: **ANO, jediný RUN**.
6. Attempts4000: **ANO**.
7. Rate3999 správně paired: **ANO**,1 warmup.
8. Metriky84: **všechny exportované max/RMSE0**, bez mismatch.
9. Vlastní historie: jedna počáteční synchronizace každého enginu a replay skutečných resetů; nenulová I historie na tomto HW **neověřena**.
10. Režie: CPU a naměřené brackety výše; úplná režie/Main deadline/WCET **neověřeny**.
11. Raw100 řádků: **ANO**, bez overwrite.
12. Osm stránek: **ANO**, včetně platné prázdné poslední stránky.
13. Motorová izolace: **zachovaná v auditovaných místech**, bez aktivního UTB řízení; ne elektrické měření.
14. DataFlash logy: **shodná metadata**, ne bitový readback.
15. Let: **NESCHVÁLEN**. Nejprve samostatný návrh dlouhého/ARMED pasivního ověření s pouze native AP autoritou, pravidla ztráty USB/loggeru a omezené RAM kapacity, kvalifikace časování/stacku pod dynamickou zátěží, skutečný RC/failsafe/senzor/yaw a motorový preflight. Původní USB příčina a první otevření BLOCK loggeru5,5–6,5ms nejsou uzavřené; RAM-only test nekvalifikuje persistentní letové logování. Fyzický let vyžaduje další samostatné schválení, nikoliv automatické pokračování po tomto PASS.

## Git a zachování dat

HEAD **11a518e5857b24cc554b3cb7e6492998013ddd1a**, větev test/skystars-5inch. Původní3G-3/3G-3A/3G-3B reporty a kapitoly/historie.txt bitově nezměněny. Žádné tracked/index změny, build, instalace, commit nebo push. Přidán pouze tento necommitnutý report a nové ignorované artefakty `/home/jirka/ardupilot_utb/tmp/utb-3g3c-hw-20261025_100008`. Přesný konečný status a kontrola hashů jsou v git-final.txt a evidence-sha256.json. Předchozí HW výsledky zůstaly zachované. Práce touto dokumentací končí.
