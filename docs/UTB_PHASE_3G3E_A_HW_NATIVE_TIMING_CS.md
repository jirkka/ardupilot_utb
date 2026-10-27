# FÁZE 3G-3E-A – Referenční HW měření native Main/Rate/Attitude

## Výsledek a rozsah

**PASS pro jediný USB-only DISARMED native + Timing experiment. Podmíněné GO k přípravě samostatně schvalované E-B; žádné letové GO.**

Na skutečné Skystars H743 bylo 2026-10-25 provedeno právě jedno měření 4000 cyklů, v původním bootu **111**. Main 2469–2531 µs, průměr 2499,999750 µs; žádná Main/Rate/Attitude perioda ≥5000 µs. Žádné missing, duplicate, invalid, unsupported, cycle gap ani změna fáze. Diagnostika se automaticky uzavřela reason=1. Tento výsledek není ověření celého Main execution duration, WCET, letu ani aktivní UTB regulace.

Veškerá nová evidence: `/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation`. Produkční kód, firmware a dřívější výsledky nebyly změněny. Nebyl proveden build, upload, reboot, instalace, ARM, motorový příkaz, logger start, mazání logů ani commit/push.

## Schválení a fyzické podmínky

Před připojením bylo přijato výslovné potvrzení „Schvaluji jedno měření a potvrzuji všechny fyzické podmínky“: pouze USB, LiPo odpojená, všechny vrtule demontované, DISARMED, žádné spouštění motorů, zavřený Mission Planner i ostatní sériové aplikace, osvědčený kabel a port jako při 3G-3C. Schválen byl jeden native Timing běh včetně read-only preflightu a dočasných diagnostických parametrů. Uloženo v `approval.json`.

Fyzické podmínky jsou potvrzením obsluhy; telemetrie nezávisle dokládá DISARMED STABILIZE. Nebyl spouštěn Flight, Chain, Attitude ani Rate SHADOW. Historická knihovna hlásila `UTB flags=1 calc=0 obs=0 policy=0`; nejde o aktivní výpočtový SHADOW.

## Offline integrita a Git

- Větev `test/skystars-5inch`, HEAD **11a518e5857b24cc554b3cb7e6492998013ddd1a**.
- Všech 15 položek původního source manifestu odpovídá SHA-256 i Git blob ID.
- APJ `tmp/utb-3g2-20261025_071149/firmware/SkystarsH7HD-bdshot_UTB3G2_PASSIVE_RAM_918dedaaeae9.apj`, 1 155 815 B, board ID 1075.
- SHA-256 **918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7**.
- Skutečný stávající parser `Tools/utb/tools/test_utb_timing_sitl.py::parse` byl offline ověřen na původním HW exportu 3E-4; jeho stará smíšená OFF/ON kontrola neodpovídá novému čistě native profilu a nebyla na nový běh aplikována.
- Nová izolovaná host kontrola ověřuje native counts/skupiny, surový kontext a TX/cleanup. Přijala skutečný export a odmítla šest záměrně poškozených **offline kopií**; nejde o další HW experiment.

Checkpoint před novými artefakty: `checkpoint.json`; výsledné hash kontroly: `integrity-final.json`, `evidence-sha256.json`, `git-final.txt`. Pět původních reportů 3G-3/A/B/C/D i `kapitoly/historie.txt` zůstaly bitově shodné. Tracked pracovní strom i index čisté. Nový necommitnutý soubor je pouze tento report; nové pomocné skripty a data jsou v ignorovaném tmp. Bez commitu a změny historie.

## Připojení a identita

Ověřeno skutečným USB výčtem i MAVLink:

- COM5, VID:PID **1209:5741**, serial **200024001251303433333336**.
- Board ID **1075**, MAVLink source **1/1**.
- ArduCopter **4.7.1 / d78e34b3**, ChibiOS d103983f, SkystarsH7HD-bdshot.
- 115200 baud, **DTR=False, RTS=False před otevřením**, bez 1200baud kroku a bez autoreconnectu.
- Boot counter **111**; uptime během pozorování monotónní, žádný restart.
- Přijato **2991** MAVLink paketů, parser receive errors **0**, raw záznam **114150 B**. To samo není důkaz nulové ztráty všech možných telemetrických paketů; úplnost Timing se posuzuje z jeho čítačů.

Na portu byl jediný vlastník; po dokončení byl zavřen. Záznam všech přijatých zpráv: `measurement/telemetry.jsonl`, `receive.raw`; všech odeslaných požadavků: `measurement/result.json::outbound`. Audit byl nasazen po první HEARTBEAT a přepnutí MAVLink 2 parseru.

## Konfigurace, Timing session a boot

Úplná záloha obsahuje **1220/1220** parametrů s úplnými indexy; neindexované notifikace 65535 nebyly zaměňovány za položky zálohy. Soubory `before-parameters.json` a `final-parameters.json`.

Ověřeno SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0, SCHED_OPTIONS=0, LOG_DISARMED=0, UTB_SHADOW=0. Všechny UCS_F_RUN/UCS_RUN/UAS_RUN/UNS_RUN/UTT_RUN před měřením 0. SMAX/NTF/NEF na RLL/PIT/YAW 0; thread snapshot neobsahuje rate thread. Původní PID, RC, motory, filtry ani failsafe se neměnily. Proti 3G-3C se změnil pouze běžící STAT_RUNTIME, 163233→181151 s; nikoliv letová konfigurace.

Před RUN byl přes skutečné rozhraní `@SYS/utb_timing.txt` načten **UTBTimingV1,pending** (20 B). Společně s očekávaným firmwarem, přítomnými UTT parametry, RUN0 a běžící smyčkou jde o důkaz dostupné session; samotné pending by při chybějícím singletonu nestačilo.

`AP_UTB_Timing::init()` nastavuje RUN0; první RUN1 zahájí finite session. `freeze()` publikuje snapshot a nastaví RUN0; opakování není ve stejném bootu dostupné. Flight používá oddělený stav a Timing start/freeze nevolá. **Nový boot nebyl potřebný a nebyl proveden.** Po tomto experimentu je Timing bootu111 spotřebovaný. Stav Flight z předchozí 3G-3C zůstal uzavřený.

## Protokol a časové značky

Přesné UTC i host monotónní timestampy jsou v TX/telemetrii. Časy níže jsou UTC; v Praze tehdy UTC+1.

- První přijatý HEARTBEAT: **14:06:54.134473 UTC**.
- Pending export dokončen: **14:07:04.505509 UTC**.
- Baseline host elapsed **29.562–44.562 s**, 15 s.
- Jediný UTT_RUN=1: **14:07:37.439663 UTC**, elapsed **45.062 s**.
- UTT_COUNT=4000, původní hodnota také 4000; nebyl změněn počet kroků původní konfigurace.
- Po triggeru až do elapsed **56.125 s** pouze příjem: v celé fázi measurement byl **právě jeden odchozí paket, samotný UTT_RUN=1**. Žádný FTP, PARAM poll, log listing, tasks.txt ani další trigger.
- Následné pozorování přibližně **56.125–65.125 s**.
- Memory/threads po výpočtu: elapsed přibližně **65.328/65.359 s**.
- Frozen export: **14:07:59.008252 UTC**, elapsed **66.625 s**, 2720 B, přenos přibližně 63 ms. Jen náklad přenosu mimo měřené okno, ne režie hlavní smyčky.
- Memory/threads po exportu: **66.656/66.703 s**.
- Celé spojení přibližně **86.187 s**, poté port zavřen.

Součet 3999 Main intervalů je **9.997499 s**, tedy skutečný měřený first-to-last odstup. Není to přesný čas od PARAM_SET do freeze ani nezávislý wall-time watchdog. Timestamp prvního/posledního cyklu není samostatně exportován; nevymýšlí se. Podle triggeru a tohoto součtu vychází orientační konec na elapsed55.059499 s; CPU active okno používá půlsekundové okrajové rezervy.

Firmware timestampy jsou uint32 monotónní mikrosekundy a během dlouhého bootu wrapují. SYSTEM_TIME má rozšířený uptime; číselný timestamp maxima v tabulce proto nelze přímo zaměnit za UTC nebo celý uptime. Host-minus-boot offset měl během baseline/active/after rozpětí přibližně **63 ms**; zahrnuje sériový přenos, fronty a kvantování. Nejde o změřený absolutní synchronizační error vůči skutečnému okamžiku výpočtu.

## Úplnost frozen exportu

Hlavička **UTBTimingV1,frozen,reason=1**. Počet cyklů **4000**. Missing rate/att, duplicate rate/att, missing_finish, invalid_time, cycle_gaps, phase_changes, unsupported: všechny **0**.

ALL a UAS OFF skupiny jsou shodné: 3999 Main/Rate/Attitude/delay-change intervalů, 4000 delay hodnot. Skupiny UAS ON a OTHER mají nulové počty. Všech 20 agregovaných položek je přítomných bez duplicit. Raw kontext **17 záznamů / 16 sousedních dvojic**, pouze native STABILIZE, UAS0, podporovaný profil. Chybějící/duplicitní kroky nebyly filtrovány ani skryty.

Export neobsahuje zvláštní overflow čítač. Kapacita raw kontextu je pevně 17, rozměr i kontinuita byly ověřeny; nejde o úplnou historii ani logger drops. Zdroj uchovává kontext kolem největší absolutní **Attitude** odchylky, nikoliv nutně kolem maximální Main periody.

## Main, Rate, Attitude a delay

| Metrika | N | Min µs | Průměr µs | Max µs | Prům. abs. odchylka µs | Rozsah µs | Součet µs |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Main start-to-start | 3999 | 2469 | 2499.999750 | 2531 | 2.924981 | 62 | 9997499 |
| Rate entry period | 3999 | 2470 | 2499.993748 | 2531 | 3.819205 | 61 | 9997475 |
| Attitude primary entry period | 3999 | 1629 | 2500.015754 | 3332 | 205.765191 | 1703 | 9997563 |
| Rate to Attitude delay | 4000 | 214 | 352.439750 | 1097 | 352.439750 | 883 | 1409759 |
| Delay change | 3999 | -866 | 0.022006 | 834 | 205.035259 | 1700 | 88 |

Odchylka period je proti 2500 µs, u delay a jeho změny proti 0. Průměrná absolutní hodnota samotného delay **352.439750 µs není periodický jitter**. Průměr signed delay-change je 0.022006 µs; jeho průměrná absolutní hodnota 205.035259 µs.

| Perioda | >2500 µs | >3000 µs | ≥5000 µs | Max: cyklus | Max: Main timestamp µs (uint32) |
| --- | --- | --- | --- | --- | --- |
| Main start-to-start | 1391 | 0 | 0 | 871 | 1075255846 |
| Rate entry period | 1723 | 0 | 0 | 871 | 1075255846 |
| Attitude primary entry period | 1988 | nedostupný počet; nejméně 1 | 0 | 1823 | 1077635816 |

Main největší odchylka **±31 µs**; maximum cyklus871, minimum cyklus872. Rate největší absolutní odchylka31 µs. Main i Rate mají prokazatelně **0 intervalů >3000 µs** z maxima; nový firmware čítač pro tuto mez nebyl potřeba. Attitude přesný počet >3000 není dostupný; z maxima3332 plyne alespoň jeden, z počtu >2500 horní mez1988. Není dovoleno z těchto agregátů dopočítat P95/P99 ani histogram.

Rate→Attitude delay minimum214 µs: cyklus1969, Main timestamp1078000815; maximum1097 µs: cyklus2063, Main timestamp1078235815. Delay-change minimum−866 µs v2064, maximum+834 µs v1823. V CSV jsou i min/max timestampy všech metrik.

Agregované max_us/min_us označují **Main začátek daného cyklu**, nikoliv přesný čas Rate/Attitude vstupu. Main timestamp se bere po INS.wait_for_sample a po opětovném získání scheduler semaphore (`AP_Scheduler.cpp:359–369`). Rate vstup je `AC_AttitudeControl_Multi.cpp:543–550`, primary Attitude vstup `AC_AttitudeControl.cpp:421–424`. Timing finish je v `ArduCopter/mode.cpp:543–544`, před dokončením ostatních scheduler úloh. **Main execution duration, dokončení motorového výstupu a hard WCET tímto nejsou změřené.**

## Vyhodnocení širší Attitude periody

Attitude 1629–3332 µs je významně širší než Main/Rate, ale není doložením ztraceného Main cyklu. Záznamy ukazují změny umístění Attitude uvnitř cyklu:

- Cyklus463: Main period2507, Rate2507, Attitude3326 µs; delay narostl z270 na1089 µs (+819).
- Cyklus464: Main2493, Rate2493, Attitude1629 µs; delay klesl z1089 na225 µs (−864).
- Pro všech16 uložených dvojic přesně platí `AttitudePeriod = RatePeriod + DelayChange`; součet této identity souhlasí také za všech3999 agregovaných intervalů.

Největší absolutní Attitude odchylka je **−871 µs** v464; pozitivní maximum3332 µs je v1823, mimo uložený kontext. Nesmí se mu přisoudit konkrétní sousední Rate perioda z jiného cyklu.

Zdrojové pořadí v `ArduCopter/Copter.cpp:113–139`: INS → Rate → AP motor output → historický SHADOW hook (runtime vypnutý) → read_AHRS → read_inertia → check_ekf_reset → update_flight_mode/Attitude. Konkrétní náklad těchto operací ani preempce se zde neměří. **Časová změna delay vysvětluje rozdíl period; neurčuje viníka.** Nelze doložit konkrétní EKF úlohu, USB událost nebo mutex. DataFlash nebyl zapnut a anomálie se mu nepřipisuje.

Žádný Main/Rate interval nedosáhl PM přibližné3000µs hranice a žádná měřená perioda5000 µs. PM.NLon ani PM.MaxT nebyly nově získány: **neověřeno**, nikoliv0. UBST8 nebyl pro tento běh načten. Jeho historická task-budget sémantika se nemění na hard deadline miss. Bez Main-end a output-end měření nelze tvrdit úplnou absenci všech hard deadline misses.

## CPU a observer effect

| Okno: host elapsed s | Počet SYS_STATUS | Min % | Průměr % | Max % |
| --- | --- | --- | --- | --- |
| baseline: 29.562000–44.562000 | 30 | 18.3 | 18.830000 | 19.2 |
| timing_interior_estimate: 45.562000–54.559499 | 17 | 18.8 | 19.194118 | 19.5 |
| after: 56.125000–65.125000 | 18 | 18.4 | 18.855556 | 19.3 |

CPU active proti baseline **+0.364118 procentního bodu**, po běhu návrat přibližně18.8556 %. Telemetrie má nastavené a readbackem potvrzené intervaly SYS_STATUS500000, SYSTEM_TIME1000000, EKF_STATUS_REPORT1000000 µs. Jde o scheduler load average ze SYS_STATUS, nikoliv per-cycle CPU profil. Active okno je host odhad s0.5s rezervami; systémové časové párování a všechny přijaté vzorky jsou v CSV.

Timing vnitřní brackety: **24870 µs / 24000 volání**, průměr **1.036250 µs/hook**, **6.217500 µs/main**, největší bracket **67 µs**. Součet průměrně0.2487 % nominální2500µs periody. To není úplná izolovaná cena instrumentace: část guardů/časování/účtování je mimo bracket; preempce je naopak uvnitř wall-time bracketu. Nesčítat maxima jednotlivých nesoučasných intervalů a neoznačovat67µs za WCET.

Měříme **native firmware s Timing**, bez dokonale neinstrumentované OFF reference. Rozdíl CPU není přesným cycle cost. Historická 3G-3C CPU18.8267→28.7278→18.7412 % zahrnovala Flight a jiný okamžik stejného bootu. Rozdíl proti nynější Timing active19.1941 % není izolovaná cena Flight, natož statisticky párované A/B měření.

## Paměť, stack a export

| Vlákno | Před: free/allocated B | Po výpočtu | Po exportu |
| --- | --- | --- | --- |
| ArduCopter | 4312/7168 | 4312/7168 | 4312/7168 |
| ISR | 1248/1536 | 1248/1536 | 1248/1536 |
| FTP | 1168/2992 | 1168/2992 | 1168/2992 |
| utb_log | 2448/3504 | 2448/3504 | 2448/3504 |
| timer | 1544/1968 | 1544/1968 | 1544/1968 |
| io | 1592/2480 | 1592/2480 | 1592/2480 |
| log_io | 1504/2016 | 1504/2016 | 1504/2016 |

Všechny zaznamenané22 thread/ISR položky zůstaly mezi třemi snapshoty shodné. Žádný nový propad watermarku během testu/exportu. Main4312/7168 a FTP1168/2992 odpovídají poslednímu3G-3C stavu. ISR1248/1536 je již **před tímto měřením** o40B níže než dřívější1288; ve stejném dlouhém bootu je watermark kumulativní a nelze rozdíl připsat této session. Není stanovena nová libovolná bezpečná hranice. Jde o pozorovanou rezervu, nikoliv záruku pod letovou zátěží.

| Region | Před FREE/LRG B | Po výpočtu FREE/LRG B | Po exportu FREE/LRG B |
| --- | --- | --- | --- |
| 0x30000000 | 6640/4192 | 7672/4192 | 6640/4192 |
| 0x20000000 | 47944/47944 | 37936/37936 | 47944/47944 |
| 0x24000000 | 524280/524280 | 524280/524280 | 524280/524280 |
| 0x00000400 | 64504/64504 | 64504/64504 | 64504/64504 |
| 0x30040000 | 32760/32760 | 32760/32760 | 32760/32760 |
| 0x38000000 | 54536/53816 | 54536/53816 | 54536/53816 |

Po výpočtu byl jeden snapshot regionu0x20000000 o10008 B nižší a region0x30000000 o1032 B vyšší; **po exportu se FREE i LRG všech šesti regionů přesně vrátily na výchozí hodnoty**. Trvalý úbytek heapu se tedy v těchto snímcích neprokázal. Konkrétní vlastník přechodné alokace nebyl zachycen a není určen. Virtuální SYS export sám vytváří `ExpandingString` (`AP_Filesystem_Sys.cpp:115`), proto snapshot není bez observer efektu; toto není důkaz, že právě tato alokace vysvětluje celý rozdíl.

MEMINFO.freemem32 bylo před737824, po výpočtu737816, po exportu737816 B; 16bit freemem65535 je saturované a není použitelný přesný údaj. MEMINFO a regionové součty se netváří jako totožný okamžik či totožná definice. Při součtu různých RAM regionů nevzniká jeden souvislý heap. Snapshoty nedokazují historické minimum nebo nepřítomnost všech přechodných alokací. Nebyla pozorována alokační chyba ani nový závažný stack/heap problém.

## Health, motorová autorita, parametry a logy

Po celý záznam HEARTBEAT DISARMED STABILIZE, SYSTEM_TIME bez poklesu. SYS_STATUS gyro/accel/AHRS/logger health vyhověly; všechny přijaté InternalError masky/počty a logger drops0. EKF attitude valid; finální flags167. Žádná parser chyba ani nečekaný restart. USB-only PreArm RC not found a Battery below minimum jsou očekávaná omezení; neprokazují připravenost k armování, yaw/position ani fyzickému letu.

AP zůstal vlastníkem motorů. Původní H743 symbol/source audit zachován: aktivní UTB Core/Output/Acro/Angle vyřazené, pasivní moduly pouze s inertním výstupem. V tomto běhu všechny jiné RUN0 a žádný ARM, servo, PWM/DShot či motor-test paket; jediné PARAM_SET se týkaly UTT_COUNT a UTT_RUN. **Timing nemá čítač motor_changes; nevydává se vymyšlená hodnota0 ani elektrická verifikace výstupů.** Tato fáze nevykonávala UTB řídicí matematiku.

Původní čtyři logy zůstaly stejné podle celých LOG_ENTRY metadat před i po:

| ID | Velikost B | time_utc |
| --- | --- | --- |
| 1 | 2516746 | 0 |
| 2 | 7924746 | 0 |
| 3 | 718496 | 0 |
| 4 | 725746 | 1792564233 |

Součet **11885734 B**. Žádné nové logování, start BLOCK loggeru, DataFlash download, erase nebo formátování. Inventář neznamená bitovou verifikaci obsahu flash.

Obnoveno a readbackem ověřeno UTT_RUN0, UTT_COUNT4000 a intervaly1/2/193 na původní **−1**. Ostatní RUN0, LOG_DISARMED0, SCHED_OPTIONS0. Finální kompletní záloha1220/1220; jediná změna proti preflightu STAT_RUNTIME181151→181181 s, běžný provozní čítač. Není žádná neobnovená dočasná hodnota. Finální FC zdravá DISARMED STABILIZE, COM5 uzavřená.

## Grafy a strojově zpracovatelná evidence

- [Agregované periody](/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation/analysis/periods.png).
- [Dostupné počty překročení](/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation/analysis/thresholds.png).
- [CPU skutečně přijaté vzorky](/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation/analysis/cpu.png).
- [Rate→Attitude min/průměr/max](/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation/analysis/delay-summary.png).
- [Omezený raw kontext a delay identity](/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation/analysis/delay-context.png).
- [Stack watermark](/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation/analysis/stack.png).
- [FREE/LRG regiony](/home/jirka/ardupilot_utb/tmp/utb-3g3e-a-20261025_approval-preparation/analysis/memory.png).

CSV: timing-statistics, raw-context, raw-pairs, cpu, system-time-alignment. JSON: timing, summary, offline-validation-checks. Původní frozen text ani surová telemetrie nejsou přepsané analýzou. Grafy neobsahují domyšlené 4000řádkové průběhy. Přenosové knihovny vypisovaly očekávaný konec krátkých SYS souborů místo původního100000B požadavku; všechny FTP operace skončily error_code0 a obsah byl validován, nejde o ztracené řádky.

## Rozhodnutí a přesná příprava E-B

1. **Původní boot111**, žádný fyzický/softwarový restart.
2. Jediná session prošla4000 cykly, reason1.
3. Main průměr2499.999750 µs, rozsah2469–2531; největší odchylka31 µs.
4. Main >2500:1391/3999; Rate1723/3999; Attitude1988/3999. Samotné překročení nominálu není hard deadline miss.
5. Period ≥5000:0 pro Main/Rate/Attitude. Main/Rate >3000:0; Attitude přesný počet chybí.
6. Rate2470–2531, Attitude1629–3332 µs; širší Attitude perioda má doloženou souvislost s intra-cycle delay, nikoliv určeného viníka.
7. Delay214–1097 µs, průměr352.439750.
8. CPU18.8300→19.1941→18.8556 %, se skutečnými počty30/17/18 vzorků a výše popsanými okny.
9. Stack beze změny mezi třemi snímky; regionové FREE/LRG po exportu stejné jako před; přechodné alokace nejsou kauzálně určené.
10. Konfigurace i metadata logů zachované, spojení uzavřené.
11. V tomto omezeném měření nebyl prokázán blocker další samostatně schválené **DISARMED E-B**. Širší Attitude entry jitter musí být součástí reference, ne automaticky připsaný budoucímu Flight.
12. **E-B nebyla provedena.** Boot111 má nyní spotřebovaný Timing i Flight snapshot; před E-B bude potřeba samostatně schválený fyzický nový start obsluhou, nikoliv automatický reboot.

Pro obhajitelné E-B zachovat přesně stejný APJ, konfiguraci400Hz/FSTRATE0/rate-thread-off, STABILIZE DISARMED, logger0 a stejné USB/kabel/telemetrii2/1/1Hz. UTT_COUNT4000; Flight COUNT/DECIM dle samostatně schváleného plánu (návrh4000/40). Ostatní RUN0. Znovu celý preflight, pending obou sessions, paměť/stack před, po výpočtu a po exportu, baseline15s a následné okno8–10s.

Dva triggery nejsou atomické; uložit přesné TX časy a skutečné počty/časové agregáty a přiznat nepřekrývající se okraje. Timing UAS OFF skupina nerozlišuje Flight aktivitu. Nový boot a jiné okamžité prostředí nelze pokládat za statisticky totožný pár; z jediného A/B nelze stanovit distribuci extrémů, WCET nebo přesnou kauzální cenu Flight. Všímat si zejména Main/Rate stability, změny delay a Attitude fáze, úplnosti, stacku a transient heapu.

Stále otevřené: celá Main execution/output latence, hard WCET/preempce, skutečný Timing OFF observer rozdíl, přesný počet Attitude>3000, kompletní raw distribuce, PM bez loggeru, příčina jednotlivých intra-cycle delay změn, vlastník transient heap alokace, BLOCK logger při reálném zápisu, RC/LiPo/motory a dynamika letu. **Výsledek není povolením letu ani aktivního UTB řízení.**
