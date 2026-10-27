# FÁZE 3G-3E-B – HW native ArduPilot + UTB Flight SHADOW + Timing

## Rozhodnutí

**Numerika PASS. Timing PASS pro jediný omezený USB-only DISARMED experiment. Motorový audit PASS. Oba exporty a obnova konfigurace PASS. Celkově PASS v tomto rozsahu; fyzický let není schválen.**

Skutečná Skystars STM32H743, nový boot **112**, právě jeden UTT_RUN1 a jeden UCS_F_RUN1. Timing4000 cyklů, Flight4000 attempts/3999 paired Rate/8000 Attitude operací; všech84 exportovaných max_abs i RMSE0, bez mismatch/DESYNC. Motor checks12000, motor changes0. Žádný Main/Rate interval>3000 µs ani perioda≥5000 µs. Main maximum2543 µs, Rate2548 µs. Attitude širší entry jitter a větší delay jsou rozebrány níže, nikoliv vydávány za Main deadline miss.

**Omezení host timestampů:** přesný sub-tick rozdíl triggerů není změřen. Oba host monotónní timestampy jsou stejné kvůli15.625ms rozlišení Windows Python3.10 hodin. UTC texty se liší1.001ms, ale jejich clock API také deklaruje15.625ms rozlišení a je nastavitelné; nejde o prokázaný přesný1.001ms odstup ani atomický start. Překrytí enginů je nezávisle doloženo firmware timestampy a konzervativními mezemi, nejméně9.969961s. Tento nedostatek se nezastírá dalším pokusem.

Evidence a grafy: `/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139`. Produkční zdroje, regulátory ani firmware nebyly měněny. Žádný upload, build, instalace, softwarový reboot, ARM, motorový příkaz, logger start/erase ani commit/push. Po skončení obě session frozen, všechny RUN0 a COM5 zavřená. Žádné pokračování další fází.

## Schválení, fyzický boot a preflight

Brána A: potvrzeno jedno fyzické USB odpojení/připojení obsluhou, pouze USB, LiPo odpojená, všechny vrtule demontované, FC bezpečně v klidu, osvědčený kabel/port, žádné motorové operace a zavřené ostatní sériové aplikace. Uloženo `approval-a.json`. Nový boot112 a read-only preflight prošly; žádný dodatečný restart nebyl proveden.

Brána B: po úplném preflightu přijato „Schvaluji jeden společný experiment; fyzické podmínky stále platí“, uloženo `approval-b.json`. Jediný společný experiment zahrnoval právě dvě finite sessions. Nejde o dvě nezávislá opakování. Pouhé fyzické potvrzení brány A nebylo zaměněno za souhlas s RUN.

COM5 / VID:PID1209:5741 / USB serial **200024001251303433333336**; MAVLink1/1, board**1075**, Copter**4.7.1/d78e34b3**, boot**112**. Ověřeno při preflightu i před experimentem. Připojení115200, DTR=False/RTS=False před open, bez1200baud workaroundu, jediný vlastník a autoreconnectFalse. První read-only spojení skončilo úspěšně; další bylo výslovně schválené měřicí spojení, nikoliv retry po chybě.

Stávající APJ: `tmp/utb-3g2-20261025_071149/firmware/SkystarsH7HD-bdshot_UTB3G2_PASSIVE_RAM_918dedaaeae9.apj`, **1155815 B**, SHA-256 **918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7**. Lokální integrita a source/symbol audit souhlasí. Běžící FW ověřen AUTOPILOT_VERSION a dostupnými rozhraními; obsah celé flash nebyl znovu bitově čten.

V obou spojeních DISARMED STABILIZE, gyro/accel/AHRS/logger zdravé a EKF attitude valid; InternalError a logger drops0. Požadovaná konfigurace SCHED_LOOP_RATE400, FSTRATE_ENABLE0, SCHED_OPTIONS0, LOG_DISARMED0, SMAX/NTF/NEF všech os0, bez rate threadu. Před triggerem ostatní UCS_RUN/UAS_RUN/UNS_RUN0 a oba cílové RUN0. Historický UTB_SHADOW0. Žádná automatická úprava PID/RC/failsafe/motorů nebo filtrů.

Úplná preflight záloha1220/1220 parametrů; index65535 správně jako notifikace. Proti E-A pouze startup gyro offsety/teploty, BARO1_GND_PRESS, nepatrný COMPASS_DEC při AUTODEC1 a STAT_BOOTCNT/RUNTIME. Přesné hodnoty `gate-b-preflight.json`. Letová konfigurace původní. Před experimentem znovu úplná záloha1220/1220; proti preflightu se změnil pouze STAT_RUNTIME.

Timing i Flight byly skutečně **pending** v preflightu a znovu před měřením: `@SYS/utb_timing.txt` → UTBTimingV1,pending, `@SYS/utb_flight.txt` → UTBFlightV1,pending. Správný firmware, parametry a RUN0 eliminují nejednoznačnost samotného pending řetězce. Flight sdílí finite lifecycle s Chain; obě session se po publikaci v témže bootu neobnovují.

## Protokol, triggery a překrytí

Původní UTT_COUNT4000/UCS_F_COUNT24000/UCS_F_DECIM200 uloženy. Nastaveno/readback4000/4000/40. Skutečné původní intervaly SYS_STATUS/SYSTEM_TIME/EKF byly−1/−1/−1; dočasně readback500000/1000000/1000000 µs. Přibližně3s ustálení a15s baseline, beze změn loggeru.

| Trigger | Host monotonic s | UTC před send |
| --- | --- | --- |
| UTT_RUN=1 | 620815.218 | 2026-10-25T14:53:53.703264+00:00 |
| UCS_F_RUN=1 | 620815.218 | 2026-10-25T14:53:53.704265+00:00 |

Zprávy byly připravené předem a odeslané postupně z jediného procesu, **bez čekání na první ACK před druhým send**. Rozlišení host hodin bohužel nezachytilo přesný odstup; samotný UTC rozdíl1.001ms není přesně známá latence po vodiči nebo začátku firmware. `analysis/host-clock-info.json` toto omezení dokládá. Žádný druhý trigger pro zarovnání neproběhl. Pro budoucí samostatně schválené běhy má host TX audit použít `perf_counter_ns`; tato session se neopakuje.

Celá fáze measurement obsahuje **právě dva odchozí PARAM_SET**, v pořadí UTT_RUN1/UCS_F_RUN1. Potom jen příjem. Žádný FTP, listing, polling, tasks.txt nebo další PARAM_SET. Po klidovém okně ověřeno automatické UTT_RUN0 a UCS_F_RUN0, pak9s telemetrie; teprve potom memory/threads a frozen exporty.

Flight přesný firmware start**1123275471 µs**, stop**1133273526 µs**, interval**9.998055 s**. Timing nemá samostatně exportované přesné first/stop timestampy; jeho3999 Main intervalů má součet**9.997501 s**. Konzervativní výpočet z reálných Main timestamp anchorů, čísel cyklů, min2458/max2543 a celého součtu dává první Main v**1123261826–1123289366 µs**, poslední v**1133259327–1133286867 µs**. Nejde o rekonstruované přesné hranice.

Pro každý anchor(j,t) je prefix součet mezi(j−1)×min a(j−1)×max; analogicky suffix z celého součtu. Průnik těchto mezí dokládá společný first-to-last Timing span s Flight nejméně**9.969961 s**; horní hranicí může být celý Flight interval9.998055s. Počty cyklů v přesném průniku ani dokonale společný start se netvrdí. Stop Timing je reason1; čas jeho posledního finish není samostatně exportován.

## Numerika, synchronizace a recorder

- Main attempts**4000**, warmup**1**, paired Rate**3999**, PID seen/valid**3999/3999**, sekvence1–3999.
- Attitude seen/valid**8000/8000**: primary STABILIZE4000 a reset4000. Attitude sync**1**, Rate sync**1**; žádné další plné importy historie, config changes0, mode changes0.
- PID externí I_set replay události**11994**, filter resets**0**; independent_i**0**. DISARMED test tudíž neověřuje nenulovou nezávislou I akumulaci za letu. I_set replay není každokrokový plný import historie.
- Všech84 metrik max_abs=RMSE=**0**:26 Attitude s8000 vzorky/metriku,4 target a54 PID s3999 vzorky/metriku. Úplné hodnoty a počty v `analysis/metrics-with-counts.csv`. Attitude používá toleranci; nulový export není univerzální důkaz bitové totožnosti.
- Missing/duplicate/invalid/unsupported, upstream/downstream mismatch, Attitude/PID mismatch/DESYNC/invalid/unsupported: vše**0**. LIMITED3999 odpovídá skutečné DISARMED/spool zpětné vazbě a nebylo skryto.
- Poslední source loop**451324**, consume**451325**. U všech99 uložených paired řádků Source=Loop−1 a source timestamp předchází consume. První recorder řádek je warmup bez platného Rate a není kreslen jako nula platného PID.
- Recorder304B/record, capacity128, candidates4000, selected**100**, decimated3900, overwritten**0**, epoch1, arm_changes/mode_exits0. Osm stránek:0–5 mají16 řádků,6 má4,7 má0; poslední je platný textový soubor s hlavičkami, nikoliv chybějící export.
- `att_shaping=0`: klidový DISARMED profil není zkouškou dynamického pilotního shapingu.

## Timing a srovnání E-A/E-B

Hlavička UTBTimingV1,frozen,reason=1. Cycles4000; všechny missing/duplicate/missing_finish/invalid_time/cycle_gaps/phase_changes/unsupported0. ALL a UAS OFF skupiny shodné, další skupiny prázdné. UAS OFF zde neznamená vypnutý Flight: UAS_RUN zůstal0 po celé společné session.

| Metrika | N | Min | Mean | Max | Sum µs | Mean abs error µs | Rozsah µs |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Main start-to-start | 3999 | 2458 | 2500.000250 | 2543 | 9997501 | 0.959990 | 85 |
| Rate entry period | 3999 | 2453 | 2499.988997 | 2548 | 9997456 | 1.745436 | 95 |
| Attitude primary entry period | 3999 | 1465 | 2500.006252 | 3530 | 9997525 | 217.527632 | 2065 |
| Rate to Attitude delay | 4000 | 350 | 503.982250 | 1405 | 2015929 | 503.982250 | 1055 |
| Delay change | 3999 | -1034 | 0.017254 | 1031 | 69 | 217.318580 | 2065 |

U period je referencí2500 µs, u delay/change0. Absolutní hodnota delay není periodický jitter. Prázdné skupiny nejsou platné nulové vzorky.

| Metrika | Min cyklus / Main timestamp µs | Max cyklus / Main timestamp µs |
| --- | --- | --- |
| Main start-to-start | 3676 / 1132462935 | 3675 / 1132460477 |
| Rate entry period | 3676 / 1132462935 | 3675 / 1132460477 |
| Attitude primary entry period | 2654 / 1129907934 | 1093 / 1126005434 |
| Rate to Attitude delay | 384 / 1124232934 | 853 / 1125405434 |
| Delay change | 854 / 1125407934 | 1093 / 1126005434 |

Timestamp extrema je Main začátek příslušného cyklu, ne nutně samotný Rate/Attitude entry čas. Main čas se bere po INS wait a scheduler lock reacquire. Finish v update_flight_mode není konec všech Main úloh. **Main execution duration, dokončení motorového výstupu a hard WCET nejsou měřené.**

| Metrika (µs) | E-A | E-B | Rozdíl µs | Relativní rozdíl % |
| --- | --- | --- | --- | --- |
| Main start-to-start min | 2469 | 2458 | -11 | -0.445525 |
| Main start-to-start mean | 2499.999750 | 2500.000250 | 0.000500 | 0.000020 |
| Main start-to-start max | 2531 | 2543 | 12 | 0.474121 |
| Main start-to-start mean abs deviation | 2.924981 | 0.959990 | -1.964991 | -67.179619 |
| Rate entry period min | 2470 | 2453 | -17 | -0.688259 |
| Rate entry period mean | 2499.993748 | 2499.988997 | -0.004751 | -0.000190 |
| Rate entry period max | 2531 | 2548 | 17 | 0.671671 |
| Rate entry period mean abs deviation | 3.819205 | 1.745436 | -2.073768 | -54.298435 |
| Attitude primary entry period min | 1629 | 1465 | -164 | -10.067526 |
| Attitude primary entry period mean | 2500.015754 | 2500.006252 | -0.009502 | -0.000380 |
| Attitude primary entry period max | 3332 | 3530 | 198 | 5.942377 |
| Attitude primary entry period mean abs deviation | 205.765191 | 217.527632 | 11.762441 | 5.716438 |
| Rate to Attitude delay min | 214 | 350 | 136 | 63.551402 |
| Rate to Attitude delay mean | 352.439750 | 503.982250 | 151.542500 | 42.998129 |
| Rate to Attitude delay max | 1097 | 1405 | 308 | 28.076572 |
| Rate to Attitude delay mean abs deviation | 352.439750 | 503.982250 | 151.542500 | 42.998129 |
| Delay change min | -866 | -1034 | -168 | 19.399538 |
| Delay change mean | 0.022006 | 0.017254 | -0.004751 | -21.590909 |
| Delay change max | 834 | 1031 | 197 | 23.621103 |
| Delay change mean abs deviation | 205.035259 | 217.318580 | 12.283321 | 5.990833 |

| Perioda | E-A >2500 | E-B >2500 | E-B >3000 | E-B ≥5000 |
| --- | --- | --- | --- | --- |
| Main start-to-start | 1391 | 780 | 0 | 0 |
| Rate entry period | 1723 | 1437 | 0 | 0 |
| Attitude primary entry period | 1988 | 2082 | počet nedostupný | 0 |

Main/Rate>3000 prokazatelně0 z maxim; přesný počet Attitude>3000 chybí (alespoň1, nejvýše2082). P95/P99 a histogram nejsou dostupné. >2500 samo není deadline miss; PM hranice zhruba1.2×perioda se nevztahuje automaticky na intra-cycle Attitude entry. PM.NLon/MaxT a UBST8 pro tento nový běh **nezískány**, nikoliv0.

## Vyhodnocení delay a anomálií

Main maximum proti E-A+12µs a Rate+17µs; průměry stále přibližně2500. Mean absolute jitter Main0.960 proti2.925 µs, Rate1.745 proti3.819 µs. Z jediného páru různých bootů nelze tyto nižší průměry označit za kauzální zlepšení Flight.

Attitude maximum3530 proti3332 µs, minimum1465 proti1629; větší rozptyl existoval již v native E-A. Delay mean narostl o**151.542500 µs** (přibližně43.0 %), maximum o**308 µs** (přibližně28.1 %). To je pozorovaný posun fáze, ne důkaz změny elektrické motorové latence.

Flight Rate consume běží v native rate cestě před motor output; navíc může před primary Attitude proběhnout resetová Attitude operace s UTB consume. To odpovídá směru zvětšení delay. Nelze ale celý rozdíl151.54µs připsat jedinému konkrétnímu bracketu, EKF úloze, loggeru nebo mutexu.

Raw Timing uchovává17 skutečných záznamů kolem největší absolutní Attitude odchylky, ne celou4000řádkovou řadu. Cyklus2653: Main2501, Rate2502, Attitude3489 µs, delay1395 a změna+987. Cyklus2654: Main2500, Rate2499, Attitude1465 µs, delay361 a změna−1034. Pro všech16 uložených párů i celou agregovanou sumu platí `AttitudePeriod = RatePeriod + DelayChange`. Největší pozitivní Attitude3530 je v1093, mimo tento raw kontext. Konkrétní sousední Rate perioda se mu nevymýšlí.

Nebyla pozorována nová Main/Rate>3000 nebo ≥5000 událost, chybějící cyklus nebo health problém. Větší intra-cycle delay je v kontextu dodané diagnostické práce; tento experiment neprokázal závažný timing blocker. Neprokazuje však WCET ani letovou rezervu.

## CPU, brackety a observer effect

| Host okno elapsed s | N SYS_STATUS | Min % | Mean % | Max % |
| --- | --- | --- | --- | --- |
| baseline: 29.828000–44.828000 | 30 | 18.4 | 18.813333 | 19.4 |
| timing_interior_estimate: 45.828000–54.825501 | 19 | 28.6 | 28.910526 | 29.3 |
| after: 56.422000–65.422000 | 18 | 18.4 | 18.783333 | 19.2 |

CPU baseline**18.813333**, společné aktivní**28.910526**, po**18.783333 %**; nárůst proti baseline10.097193 procentního bodu. E-A active19.194118 %: rozdíl**+9.716409 pp**. 3G-3C active28.7278 %: rozdíl přibližně+0.1827 pp. Telemetrie je průměrovaná, oddělené booty/okamžiky nejsou izolované kauzální měření ceny UTB.

Společné active okno má0.5s okrajové rezervy podle obou TX a firmware trvání; nejistota není odstraněna textovým UTC rozlišením. SYSTEM_TIME párování a všechny skutečně přijaté SYS_STATUS jsou uložené. Host-minus-boot offset za celé spojení kolísal přibližně125ms včetně přenosů a parametrické zálohy; nejde o známou absolutní chybu synchronizace.

| Úsek | Sum µs | Mean µs/main | Vztah |
| --- | --- | --- | --- |
| Attitude pre-capture | 32200 | 8.050000 | samostatný měřený bracket |
| Post-capture bez Att consume | 24831 | 6.207750 | samostatný; včetně příslušného auditu |
| Attitude consume | 451616 | 112.904000 | samostatný; 8000 operací |
| Rate process | 248006 | 62.001500 | samostatný; 3999 operací |
| Flight recorder | 1793 | 0.448250 | samostatný; 4000 kandidátů |
| Attitude math | 142295 | 35.573750 | uvnitř Attitude consume |
| Attitude sync/validation | 148724 | 37.181000 | uvnitř Attitude consume |
| Attitude compare | 77864 | 19.466000 | uvnitř Attitude consume |
| PID math | 31255 | 7.813750 | uvnitř Rate process |
| PID consume | 242046 | 60.511500 | uvnitř Rate process |
| Native Rate + capture | 65613 | 16.403250 | obsahuje originální AP, není přidaná režie UTB |

Pět samostatných Flight bracketů dává měřenou část**189.611500 µs/main**, proti3G-3C190.115250 rozdíl−0.503750 µs/main. Max Rate process73µs, max Attitude consume178µs/operaci, max recorder30µs. Vnitřní Attitude math/sync/compare a PID math/consume se znovu nepřičítají. Samostatná cena motorového auditu není exportovaná, je součástí příslušných bracketů. Native capture není čistá přidaná režie.

Timing hooks**29225 µs / 24000 volání**, průměr**7.306250 µs/main**,1.217708µs/hook, maximum67µs. E-A6.2175µs/main; rozdíl+1.08875. Timing a Flight se vykazují odděleně; jejich čísla nejsou součtem celé Main smyčky. Guardy, kopie mimo brackety a preempce nejsou izolované. Vyšší wall bracket může obsahovat preempci; maximum není WCET.

## Paměť a stack

Brána A měla Main4344/7168, ISR1296/1536, FTP1720/2992; první regionFREE2624/LRG2576. Nové snapshoty níže patří ke skutečnému měřicímu spojení, nikoliv k záměně těchto okamžiků.

| Vlákno | Před free/allocated B | Po výpočtu | Po exportu |
| --- | --- | --- | --- |
| ArduCopter | 4344/7168 | 4344/7168 | 4344/7168 |
| ISR | 1288/1536 | 1264/1536 | 1264/1536 |
| FTP | 1608/2992 | 1568/2992 | 1216/2992 |
| utb_log | 2456/3504 | 2456/3504 | 2456/3504 |
| timer | 1544/1968 | 1544/1968 | 1544/1968 |
| io | 1544/2480 | 1544/2480 | 1544/2480 |
| log_io | 1392/2016 | 1392/2016 | 1392/2016 |

Main během experimentu i exportu beze změny. ISR klesl24B, FTP40B po výpočtu a následně352B po úplném exportu; nové pozorované minimum FTP1216/2992 je stále o48B vyšší než E-A1168. Watermark je kumulativní minimum volného stacku, ne okamžitá spotřeba nebo garantovaná letová rezerva. Z těchto malých změn se nevytváří nová arbitrární bezpečnostní hranice. Žádné stack selhání nebylo pozorováno.

| Region | Před FREE/LRG B | Po výpočtu | Po exportu |
| --- | --- | --- | --- |
| 0x30000000 | 3680/3128 | 3680/3128 | 5576/3128 |
| 0x20000000 | 47944/47944 | 47944/47944 | 47944/47944 |
| 0x24000000 | 524280/524280 | 524280/524280 | 524280/524280 |
| 0x00000400 | 64504/64504 | 64504/64504 | 64504/64504 |
| 0x30040000 | 32760/32760 | 32760/32760 | 32760/32760 |
| 0x38000000 | 54536/53816 | 54536/53816 | 54536/53816 |

FREE/LRG všech regionů před a po výpočtu shodné. Po exportu první regionFREE vzrostlo3680→5576 B, LRG3128 beze změny; jiné regiony shodné. Trvalý pokles nebo zhoršení největšího bloku se v těchto snímcích neprokázal. Konkrétní vlastník přesunů mezi read-only preflightem a snapshoty není měřen; SYS exporty mají vlastní alokace a nesynchronní observer efekt.

MEMINFO.freemem32: před737832, po výpočtu737832, po exportu736752 B. Menší1080B rozdíl v jiných okamžicích/definici není důkaz leak ani nelze vydávat za historické minimum. Regionové součty nejsou jeden souvislý heap; 16bit freemem65535 je saturované. Nebyla pozorována alokační chyba.

## Sekvenční export, health a obnovení

Po obou RUN0 a následném pozorování: memory/threads → Timing2738B → Flight metadata10976B → všech osm stránek → memory/threads. Každý FTP soubor byl jediný otevřený v daném kroku, error_code0, obsah validován. Page7 má563B hlaviček a0 datových řádků. Krátké EOF warningy knihovny vůči100000B požadavku neznamenají chybějící řádky. Původní soubory ani důkazy nebyly nahrazeny opravnou kopií.

Měřicí spojení**88.438 s**,4078 přijatých paketů, parser receive errors0, raw225023B. Všechna HEARTBEAT DISARMED STABILIZE, uptime monotónní, bez rebootu. SYS_STATUS všechny InternalError masky/počty a logger drops0, finální EKF flags167. USB-only PreArm RC/battery omezení nejsou zaměněná za letovou připravenost.

Úplná finální záloha1220/1220: proti pre-run jediná změna STAT_RUNTIME183905→183966 s. Obnoveno a readbackem potvrzeno UTT_COUNT4000, UCS_F_COUNT24000, UCS_F_DECIM200; UTT_RUN/UCS_F_RUN/UCS_RUN/UAS_RUN/UNS_RUN0. Intervaly SYS_STATUS/SYSTEM_TIME/EKF zpět−1/−1/−1, LOG_DISARMED0, SCHED_OPTIONS0. Žádný neobnovený dočasný parametr. FC zdravá DISARMED STABILIZE, COM5 zavřená.

Původní LOG_ENTRY před/po shodné: ID1 2516746 B/time0; ID2 7924746/time0; ID3 718496/time0; ID4 725746/time1792564233. Celkem11885734 B. Žádný nový log, DataFlash zápis, erase, formátování nebo forced start_new_log. Inventář je metadata kontrola, nikoliv bitová verifikace flash. BLOCK logger tímto není kvalifikován pro let.

## Motorová izolace a Git

Původní H743 active Core/Output/Acro/Angle compile-out zachován podle source/symbol manifestu. Žádný live AP_Motors není předáván pasivnímu UTB a nebyl přidán motor setter nebo zápis do AP PID historie. Skutečný audit**12000 kontrol / 0 změn**; žádná odchozí zpráva ARM/motor/servo/PWM/DShot. Motor/servo/RC/failsafe parametry shodné. Auditované hodnoty nejsou měřením elektrických ESC výstupů.

HEAD **11a518e5857b24cc554b3cb7e6492998013ddd1a**, větev `test/skystars-5inch`. Tracked pracovní strom i index čisté. Source manifest15 SHA/blob shodný;144 dřívějších archivovaných souborů3G-3C/E-A a původní reporty/historie bitově zachované. Přibyl jen tento necommitnutý report a ignorované nové tmp artefakty. Konečný stav a hash manifest: `git-final.txt`, `integrity-final.json`, `evidence-sha256.json`. Bez commit/reset/clean/stash/rebase/push.

## Grafy a data

- [E-A/E-B Main, Rate, Attitude](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/ea-eb-periods.png), [delay](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/ea-eb-delay.png), [překročení](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/ea-eb-thresholds.png).
- [CPU E-A/E-B](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/ea-eb-cpu.png), [stack](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/stack.png), [FREE/LRG](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/memory.png).
- [Všech84 metrik](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/all84-errors.png), [native/UTB body-rate targets](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/flight/body-targets.png), [PID output](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/flight/pid-outputs.png).
- [FF/DFF](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/flight/feedforward.png), [I](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/flight/integrator.png), [Timing raw kontext](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/timing-context.png).
- [Flight skutečné sekvence](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/flight-sequences.png), [Flight timestampy](/home/jirka/ardupilot_utb/tmp/utb-3g3e-b-20261025_143139/analysis/flight/timing-records.png).

CSV obsahují timing-statistics, ea-eb-comparison, metrics-with-counts, raw-context/raw-pairs, cpu, system-time-alignment a Flight100 řádků. Decimované10Hz body nejsou rekonstruovaná400Hz časová řada. Všechny chyby/počty jsou primárně ve frozen exportech a JSON; grafy nejsou náhradou validity.

## Přímé odpovědi a zbývající kvalifikace

1. Nový boot112 i preflight prošly, obě původně pending.
2. Společná aktivita doložena firmware daty, společný span nejméně9.969961s.
3. Pořadí triggerů doloženo. UTC rozdíl1.001ms, monotonic rozdíl nerozlišen; přesný sub-tick odstup **neověřen**.
4. Timing4000 cyklů/3999 period validní.
5. Flight3999 paired Rate validní, první warmup explicitně1.
6. Všech84 metrik bez vykázané odchylky; tolerantní Attitude není univerzální bitový důkaz.
7. Motorová izolace zachovaná:12000 kontrol/0změn, AP jediná autorita.
8. Main2458–2543µs; max proti E-A+12µs, mean2500.000250.
9. Rate2453–2548µs; max+17µs, mean2499.988997.
10. Attitude1465–3530µs; maximum+198µs, širší jitter existoval již v E-A.
11. Delay mean503.982250µs (+151.5425), max1405 (+308).
12. Main/Rate>3000:0; všech period≥5000:0. Attitude>3000 existuje, přesný počet chybí.
13. CPU aktivní28.910526 % proti E-A19.194118: +9.716409pp; nikoliv izolovaná cena Flight.
14. Main watermark nezhoršen, drobné ISR/FTP změny doložené; žádný pozorovaný alokační/stack fault, historické minimum heapu chybí.
15. Timing i všech8 Flight stránek úplné a validní.
16. Konfigurace/intervaly obnovené, původní4 log metadata zachovaná, port uzavřen.
17. Jediný A/B dokládá funkčnost společné diagnostiky, numeriku a pozorované časování/zdroje v tomto klidovémUSB profilu; ne distribuci extrémů, WCET nebo letovou bezpečnost.
18. Před fyzickým pasivním letem nadále chybí samostatná kvalifikace BLOCK loggeru nebo zvoleného RAM-only uchování/exportu bez ztráty bootu, skutečné RC/failsafe/LiPo/motor/yaw kontroly, timing pod dynamikou a odpovídající provozní preflight. Pokud bude požadována kompletní Main/output latence, současný firmware ji neposkytuje. Další měření vyžaduje nový plán a souhlas.

**Práce končí tímto reportem. Žádný další HW experiment, upload, ARM, motor test nebo let nebyl spuštěn ani schválen.**
