# FÁZE 3G-3F-HW-2A – Referenční Main/Rate/Attitude a Execution Timing

**Výsledek: PASS pro jeden USB-only DISARMED native referenční experiment. GO pouze k samostatnému schválení HW-2B.**

## 1. Rozsah experimentu

Skutečná Skystars STM32H743, 25. 10. 2026; časy níže Europe/Prague, CET UTC+1. Jeden `UTT_RUN=1`, `UTT_COUNT=4000`, native STABILIZE, Flight SHADOW runtime OFF. Nejde o kvalifikaci letu, elektrického dokončení DShot nebo hard WCET. Žádný nový firmware, build, instalace, ARM, motorový příkaz, logger start/erase nebo Git commit/push.

## 2. Offline integrita

APJ `tmp/utb-3g3f-20261025_152000/firmware/SkystarsH7HD-bdshot_UTB3G3F_EXEC_ON_dae4f020db98.apj`, 1 158 367 B, board 1075, SHA-256 `dae4f020db98a509b68c71a5b8702dfc7019ed762f69b7d46c8cbf3ae5e9d497`. Souhlasí původních 14 source SHA/blob manifest položek, 7688 původních souborů a historické evidence E-A/E-B/HW-1R (46/90/51 položek). Parsery byly přehrané na skutečných historických exportech; nic z těchto výsledků se nevydává za nové měření.

## 3. Výchozí Git/source stav

Větev `test/skystars-5inch`, HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`. Původních 7 modified a 19 untracked zachováno bitově; nyní 7 modified a 20 untracked, jediný nový neignorovaný soubor je tento report. Index prázdný. `kapitoly/historie.txt`, rozpracované zdroje i původní výsledky nedotčeny. Podrobný stav `git-before.txt`, `git-final.txt`, `final-integrity.json` v adresáři evidence. Gitlink adresáře nejsou mylně počítané jako nové běžné soubory.

## 4. Schvalovací brány a zachovaná hostitelská chyba

Brána A i obnovená A pro opravený preflight a brána B pro právě tento experiment byly výslovně schválené; odpovědi uložené v `approval-a.json`, `approval-a-renewed.json`, `approval-b.json`. Původní read-only pokus skončil s RX0/TX0, protože host limit na první heartbeat chybně zahrnoval dlouhou přípravu UNC hashů. Nešlo o prokázanou chybu FC. Opravený osmivteřinový limit začíná po otevření portu, průběžný pětivteřinový heartbeat guard zůstal. Původní chyba a skript zachované, nový preflight zvlášť schválený. Pět skutečných receive regresí a 12 nových offline kontrol host whitelistu/validátoru prošlo; žádný z těchto testů se nepřipojoval k hardware. Finální offline audit poprvé zachytil pouze rozdíl v evidenci adresářů Gitlink proti souborovému checkpointu; výběr byl opraven na běžné soubory, nic v submodulech se neměnilo.

## 5. Fyzické podmínky

Potvrzené před A i B: pouze USB, odpojená LiPo, všechny vrtule demontované, FC v klidu, ostatní sériové aplikace zavřené. Trvale DISARMED STABILIZE. Neprováděl se ARM, změna režimu nebo motorový test.

## 6. USB identita a připojení

USB serial `200024001251303433333336`, VID:PID `1209:5741`, aktuálně COM5, MAVLink 1/1. Výběr přes existující `utb_usb_identity`, nikoliv odhad podle čísla portu. 115200 baud, DTR=False/RTS=False před open, jediný vlastník, autoreconnect=False. Žádný fyzický reconnect ani restart. Po A byl port zavřen; po B proběhlo jedno schválené nové otevření a opět zavření. Při výpadku po RUN by se USB automaticky neodpojovalo, protože by se ztratila RAM evidence.

## 7. Firmware a boot

AUTOPILOT_VERSION: board 1075, ArduCopter4.7.1/11a518e5, UID2 odpovídá serialu. Stávající nahraný kandidát dle upload evidence HW-1; APJ hash je ověřený na hostu, kryptografický readback celé MCU image se neprováděl. Boot 115 byl zachován při A, před B i po měření. SYSTEM_TIME monotónní bez restartu.

## 8. Preflight a health

Opravený A trval 60,627 s, 1406 paketů, parser errors0, všechny RUN0, žádný PARAM_SET ani změna intervalů. B spojení trvalo 84.478237 s, 3056 RX paketů /117965 B a159 TX paketů /4918 B, parser errors0. Nezávislé přehrání raw RX/TX potvrdilo 83 DISARMED STABILIZE heartbeatů a jediný PARAM_SET. Gyro/accel/baro/AHRS/logger zdravé, EKF attitude flag platný (finální flags167), InternalError/errors_count1/2/4 a logger errors_count3 po celou pozorovanou telemetrii0. Health se ověřovalo i po exportu/obnově. Chybějící LiPo zůstává omezením, nikoliv důkazem závady.

## 9. Parametrická konfigurace

Úplné zálohy 1220 parametrů, indexy 0–1219; index 65535 je neindexovaná notifikace. `SCHED_LOOP_RATE=400`, `FSTRATE_ENABLE=0`, `SCHED_OPTIONS=0`, `LOG_DISARMED=0`, `LOG_BACKEND_TYPE=4`, SMAX/NTF/NEF všech os0, bez rate threadu. UTT/UCS_F/UCS/UAS/UNS RUN0 před/po, UTB_SHADOW0. ATC/PID/filtry/motor/servo/frame/RC/failsafe/EKF/AHRS/baterie proti HW-1R zachované. Změnil se jen provozní STAT_RUNTIME. COMPASS_USE/USE2/USE3 zůstávají0; absolutní yaw/navigační a letová způsobilost se neověřují.

## 10. Pending a finite lifecycle

TimingV1/ExecutionV1/FlightV1 byly pending při A i B. Timing a Execution sdílí stejný finite UTT lifecycle; Flight vlastní session zůstala pending po celou dobu. Session Execution `session_us=2839637678`, stejná hodnota v koncovém `end`, reason 1=COUNT. UTT local cycles 1–4000 znamenají4000 Main kroků, tedy 3999 následných period; nejde o počítání podle host wall-time. Publikace immutable exportu release/acquire, bez průběžného exportu.

## 11. Paměť/stack před experimentem

V A Main4344/7168, ISR1272/1536, FTP1568/2992 B volného watermarku. První region FREE/LRG2616/2096 B; další47944/47944,524280/524280,64504/64504,32760/32760,54536/53816 B. Bez pozorovaného selhání. V B vlastní snapshot před výpočtem níže; region0 již3680/3160 B. Rozdíl snapshotů zahrnuje přípravu a FTP lifetime, není připsaný výpočtu. MEMINFO je součet regionů, nikoliv jeden souvislý blok.

## 12. Skutečná telemetrie

GET_MESSAGE_INTERVAL před změnou i při obnově: SYS_STATUS/SYSTEM_TIME/EKF -1/-1/-1 µs. Po schválení nastaveno500000/1000000/1000000 µs, ACK i GET readback ověřené. Žádná změna během okna. Všechny COMMAND_LONG v raw TX mají ID510/511/512 (GET/SET_MESSAGE_INTERVAL/REQUEST_MESSAGE), žádný ARM/reboot/motor příkaz. Přesný audit v `measurement/result.json`, `telemetry.jsonl`, `transmit.raw` a `analysis/command-timeline.csv`.

## 13. Baseline

3 s ustálení, poté 15.028888 s pasivní baseline bez jediného TX. CPU30 skutečných vzorků, mean18,516667 %, min18,0/max18,9 %. Baseline nemá změřené M1–M5 ani plné periodové agregace: ty existují pouze během aktivní finite Timing session.

## 14. Jediný UTT_RUN

UTT_COUNT už4000, proto žádný redundantní zápis. Jediný PARAM_SET `UTT_RUN=1`, MAV_PARAM_TYPE_INT8, target1/1. Přesný raw TX uložen; žádný jiný PARAM_SET nebyl odeslán. Poté11,065363 s do prvního pasivního checkpointu a9,010664 s dalšího pozorování, teprve pak PARAM_REQUEST_READ RUN0/boot. Žádný polling, FTP, log listing nebo další příkaz uvnitř tohoto přibližně20s okna. Automatické RUN0 ověřené, žádný druhý pokus.

## 15. Časová osa a hodiny

| Událost | Host elapsed s / další údaj |
|---|---|
| Otevření portu | 0.976881 |
| Baseline begin | 27.365079 |
| Baseline end | 42.393967 |
| Trigger příprava | 42.402264 |
| PARAM_SET před předáním transportu | 42.404762 |
| PARAM_VALUE potvrzen, zápis v host runneru | 42.406338 |
| První pasivní okno konec | 53.467628 |
| Následné pasivní okno konec | 62.478292 |
| Celé spojení včetně kontrol/cleanup | 84.478237 |

Trigger příprava 2026-10-25T19:30:48.516280+01:00; audit těsně před transportem 2026-10-25T19:30:48.519316+01:00, `perf_counter_ns=633832170345600`. To je host čas, nikoliv přesný čas přijetí FC; přípravný stamp není zaměňován za fyzické TX. QueryPerformanceCounter je monotónní/neadjustovatelný, reportované rozlišení100 ns; nanosekundové API nedokazuje nanosekundovou přesnost transportu. Windows `time.monotonic()` zde má15,625ms rozlišení, proto se k auditu nepoužil. UTC wall clock není monotónní.

MCU měření používá HAL micros/micros64 s1µs tickem, ne SITL čas. První Main v rawcycle1 `2839637671`, první→poslední Main podle3999 period 9.997503 s. Host↔boot offset z přijatých SYSTEM_TIME má rozsah -2797.241763 až -2797.136810 s (rozptyl přibližně105 ms); zahrnuje příjem/telemetrii a neprokazuje přesný jednostranný přenosový delay. Všechna periodová čísla níže jsou MCU agregace, nikoliv odhad z hosta.

## 16. TimingV1 výsledky

| Metrika | N | Min µs | Mean µs | Max µs | Sum µs | Mean abs. error µs | >2500 | >3000 | ≥5000 |
|---|---|---|---|---|---|---|---|---|---|
| Main start-to-start | 3999 | 2475 | 2500.000750 | 2523 | 9997503 | 2.829957 | 1336 | 0 | 0 |
| Rate entry period | 3999 | 2467 | 2499.992748 | 2523 | 9997471 | 4.016754 | 1745 | 0 | 0 |
| Attitude primary entry period | 3999 | 1467 | 2499.901225 | 3504 | 9997105 | 203.977244 | 1964 | neověřeno | 0 |
| Rate to Attitude delay | 4000 | 217 | 353.816500 | 1264 | 1415266 | 353.816500 | 0 | 0 | 0 |
| Delay change | 3999 | -1027 | -0.091523 | 1005 | -366 | 203.033758 | 0 | 0 | 0 |

Mean abs. error pro periody znamená `mean(|period−2500|)`, ne odchylku od jejich naměřeného průměru; pro delay/change je referencí0. Přesný počet >3000 pro Attitude není exportovaný a nelze jej určit z min/max/sum; proto neověřeno. Main/Rate i delay/change mají max≤3000, takže jejich počet>3000 je prokazatelně0. Všechny počty≥5000 jsou0. Jitter Main min/max vůči2500: −25/+23 µs, peak-to-peak48 µs; Rate−33/+23, peak-to-peak56 µs. P95/P99/histogram všech 4000 kroků nejsou dostupné.

| Metrika | Min cycle / Main µs | Max cycle / Main µs |
|---|---|---|
| Main start-to-start | 1531 / 2843462671 | 1530 / 2843460196 |
| Rate entry period | 2 / 2839640171 | 2488 / 2845855188 |
| Attitude primary entry period | 3347 / 2848002671 | 3586 / 2848600171 |
| Rate to Attitude delay | 447 / 2840752681 | 1666 / 2843800171 |
| Delay change | 2627 / 2846202671 | 3586 / 2848600171 |

![Timing a Rate→Attitude delay na H743](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/timing-and-delay.png)

## 17. Execution M1–M5

| ID / metrika | N | Min µs | Mean µs | Max µs | Sum µs | >2500 | ≥5000 |
|---|---|---|---|---|---|---|---|
| 0 main_to_motor_begin | 4000 | 28 | 31.489500 | 62 | 125958 | 0 | 0 |
| 1 motor_call | 4000 | 29 | 31.993500 | 41 | 127974 | 0 | 0 |
| 2 main_to_motor_return | 4000 | 57 | 63.483000 | 96 | 253932 | 0 | 0 |
| 3 main_to_scheduler_boundary | 4000 | 315 | 491.404750 | 1383 | 1965619 | 0 | 0 |
| 4 rate_to_motor_begin | 4000 | 13 | 15.368750 | 23 | 61475 | 0 | 0 |
| 5 utb_rate_consume | 0 | absent | absent | absent | 0 | 0 | 0 |
| 6 main_to_mode_finish | 4000 | 266 | 408.748250 | 1311 | 1634993 | 0 | 0 |
| 7 main_to_fast_boundary | 4000 | 280 | 423.963750 | 1324 | 1695855 | 0 | 0 |

Všech sedm povinných metrik má N=4000; všechny >2500/≥5000 počty0. Mean absolutní hodnoty pro nezáporné spany je stejné jako mean, nikoliv MAD od průměru. M3=M1+M2 ve stejných cyklech i v součtech253932=125958+127974; maxima62+41 se nesmí vydávat za maximumM3.

| ID | Min cycle / Main µs | Max cycle / Main µs |
|---|---|---|
| 0 | 17 / 2839677676 | 1 / 2839637671 |
| 1 | 2 / 2839640171 | 476 / 2840825177 |
| 2 | 1082 / 2842340180 | 1 / 2839637671 |
| 3 | 515 / 2840922671 | 3466 / 2848300171 |
| 4 | 46 / 2839750171 | 739 / 2841482671 |
| 6 | 1834 / 2844220176 | 826 / 2841700170 |
| 7 | 667 / 2841302671 | 3106 / 2847400171 |

![Execution agregace](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/execution-aggregates.png)

## 18. Mode/fast/scheduler hranice

Main timestamp vzniká v `AP_Scheduler.cpp:380`, po INS wait a převzetí původního scheduler semaphore. Motor bracket `ArduCopter/motors.cpp:126–130` přímo obklopuje nezměněné `motors_output()`. Mode finish je `AP_UTB_Timing.cpp:300–302` při update_flight_mode; Fast boundary `AP_Scheduler.cpp:238–240` při prvním non-fast tasku; Scheduler end `AP_Scheduler.cpp:464` před diagnostickou finalizací/epilogem.

Mode266/408,748250/1311 µs, Fast280/423,963750/1324 µs, M4=315/491,404750/1383 µs. Jde o vnořené spany z Main startu, nesčítají se. M4 nezahrnuje celý AP_Vehicle/HAL cyklus, vlastní závěrečnou finalizaci sond ani všechna budoucí IRQ; rozdíl2500−1383 není garantovaná rezerva celého řetězce. M2 je délka softwarového volání včetně možné preempce a observer effectu, nikoliv elektrické DShot completion.

## 19. Volitelný UTB consume

`utb_rate_consume count=0`, mean=null/absent. Oba consume eventy chybí společně legitimně, žádný neúplný pár. Flight RUN0 a po exportu stále FlightV1,pending. Neexistuje naměřená0µs UTB matematika ani Flight motor_changes statistika; není doplňovaná.

## 20. Cycle/event validace

Timing: cycles4000; missing_rate/att, duplicate_rate/att, missing_finish, invalid_time, cycle_gaps, phase_changes, unsupported vše0. SkupinaALL=OFF, ON/unsupported skupiny prázdné. Execution missing/duplicate/invalid/gaps/unsupported vše0, requiredmask505. Striktní parser ověřil counts, sum/min/max, pořadí, invarianty, stejné session_us header/end a anchor maximum. Úplnost agregací4000 je z finite čítačů; raw export netvrdí přítomnost4000 jednotlivých řádků. Žádná ztracená diagnostická sada není skrytě vyřazená.

## 21. Raw kontext

Execution anchor=M3 maximum96 µs v cyklu 1, raw cykly 1–9, 9 řádků: žádná předchozí historie před1, osm následujících, kapacita17. Requiredmask505 a bad0 u všech; firmware timestampy a hodnotové rozdíly v `analysis/execution-raw-context.csv` a `execution-context-spans.csv`. Timing anchor worst absolute Attitude error v cyklu 3347,17 řádků a16 adjacent pair rozdílů; identity Att_dt=Rate_dt+delay_change přesné. Ring je záměrně bounded, export nemá samostatný celo-session overwrite/drop counter, nejde o chybu loggeru. Kontexty mají odlišný anchor a nelze je spojovat jako stejné cykly.

Největší M3 nastal při prvním kroku, kde source provádí inicializaci finite sond; časové seskupení není izolované měření ceny této inicializace. Žádné neopodstatněné připsání konkrétního času HAL/IRQ.

![Omezený Execution raw kontext](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/execution-context.png)

![Omezený Timing raw kontext](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/timing-context.png)

## 22. CPU baseline/active/after

| Okno | Host elapsed s | N | Min % | Mean % | Max % |
|---|---|---|---|---|---|
| baseline | 27.365079–42.393967 | 30 | 18.000000 | 18.516667 | 18.900000 |
| timing_interior_estimate | 42.902264–51.899767 | 17 | 18.700000 | 19.111765 | 19.500000 |
| after | 53.467628–62.467628 | 18 | 18.000000 | 18.638889 | 19.000000 |

Active interior je z host triggeru a MCU součtu period, s0,5s okraji; ne přesně synchronizovaná MCU brána. SYS_STATUS load je scheduler load average se zpožděním/kvantizací, nikoliv měření každého400Hz cyklu. Pozorovaný rozdílactive−baseline=0.595098 procentního bodu, nikoliv izolovaná cena nové Execution sondy. Baseline i active mají stejnou telemetrii. Chybějící PM.NLon/MaxT se nedopočítávají: bez DataFlash zápisů a dalších zásahů nejsou pro tato okna ověřené. UBST/per-task účetnictví nebylo v tomto RAM-only profilu exportované a tasks.txt se nečetl.

![CPU časová osa skutečných SYS_STATUS vzorků](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/cpu-timeline.png)

## 23. Hook brackety a observer effect

| Sonda | Sum µs | Calls | Mean/hook µs | Mean/Main µs | Max bracket µs | Mean/Main vůči 2500 µs |
|---|---|---|---|---|---|---|
| Timing | 32059 | 24000 | 1.335792 | 8.014750 | 69 | 0.320590 % |
| Execution | 17419 | 16000 | 1.088688 | 4.354750 | 61 | 0.174190 % |

Jde o skutečně naměřené oddělené brackety, ne celkovou izolovanou dodatečnou režii a ne hard WCET. Nezahrnují všechny guardy, volání HAL hodin a nezařazenou finalizační práci. Měřený wall-time může zahrnovat preempci; část Execution práce je uvnitř původních Timing bracketů (např. start/RATE/ATT), MODE_END má vlastní neměřenou část. Nelze bez odvození sčítat brackety nebo maxima jako cenu firmware. Aktivní sondy navíc čtou HAL hodiny; cenu clock/critical section neodděluje tento běh. Neporovnával se nový compile-OUT a ON v párovaných bootech.

![Naměřené hook brackety](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/hook-brackets.png)

## 24. Stack/heap po výpočtu

| Vlákno | Přiděleno B | Před B | Po výpočtu B | Po exportu B |
|---|---|---|---|---|
| ISR | 1536 | 1272 | 1256 | 1256 |
| ArduCopter | 7168 | 4344 | 4344 | 4344 |
| idle | 496 | 288 | 288 | 288 |
| UART_RX | 1200 | 864 | 864 | 864 |
| OTG1 | 752 | 352 | 352 | 352 |
| monitor | 1456 | 1096 | 1096 | 1096 |
| timer | 1968 | 1544 | 1544 | 1544 |
| rcout | 944 | 504 | 504 | 504 |
| rcin | 1456 | 816 | 816 | 816 |
| io | 2480 | 1544 | 1544 | 1544 |
| storage | 1456 | 944 | 944 | 944 |
| UART4 | 752 | 328 | 328 | 328 |
| log_io | 2016 | 1432 | 1432 | 1432 |
| I2C0 | 1456 | 984 | 984 | 984 |
| OSD | 1712 | 976 | 976 | 976 |
| led | 688 | 280 | 280 | 280 |
| UART1 | 752 | 328 | 328 | 328 |
| I2C1 | 1456 | 1000 | 1000 | 1000 |
| SPI1 | 1456 | 728 | 728 | 728 |
| SPI4 | 1456 | 864 | 864 | 864 |
| utb_log | 3504 | 2448 | 2448 | 2448 |
| FTP | 2992 | 1568 | 1568 | 1568 |

| Region / TYPE | Před FREE/LRG B | Po výpočtu FREE/LRG B | Po exportu FREE/LRG B |
|---|---|---|---|
| 0x30000000 / 8 | 3680/3160 | 3688/3640 | 3688/3640 |
| 0x20000000 / 2 | 47944/47944 | 47944/47944 | 47944/47944 |
| 0x24000000 / 4 | 524280/524280 | 524280/524280 | 524280/524280 |
| 0x00000400 / 2 | 64504/64504 | 64504/64504 | 64504/64504 |
| 0x30040000 / 8 | 32760/32760 | 32760/32760 | 32760/32760 |
| 0x38000000 / 1 | 54536/53816 | 54536/53816 | 54536/53816 |

Main4344 a FTP1568 B free beze změny, ISR1272→1256 B (−16 B), další vypsané watermarky beze změny. Vůči HW-1R ISR1296 je to jiný pozdější stav téhož bootu; nelze cenu jednotlivých úkonů odvodit z watermarku. První region3680/3160→3688/3640 B, ostatní beze změny. Žádné pozorované selhání nebo přetrvávající pokles FREE/LRG. Snapshoty nejsou přechodné minimum heapu uvnitř4000 cyklů; nezaručují budoucí worst-case stack/fragmentaci nebo letovou bezpečnost.

![Stack watermarky](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/stack-watermarks.png)

![Regionová FREE a LRG](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/memory-regions.png)

## 25. Sekvenční export a paměť po exportu

| Soubor | B | Host duration s | Dokončení Europe/Prague |
|---|---|---|---|
| \\wsl$\Ubuntu\home\jirka\ardupilot_utb\tmp\utb-3g3f-hw2a-20261025_reference-01\measurement\utb_timing.txt | 2746 | 0.085337 | 2026-10-25T19:31:10.691145+01:00 |
| \\wsl$\Ubuntu\home\jirka\ardupilot_utb\tmp\utb-3g3f-hw2a-20261025_reference-01\measurement\utb_execution.txt | 1746 | 0.061084 | 2026-10-25T19:31:10.752394+01:00 |
| \\wsl$\Ubuntu\home\jirka\ardupilot_utb\tmp\utb-3g3f-hw2a-20261025_reference-01\measurement\after-flight.txt | 20 | 0.036624 | 2026-10-25T19:31:10.788682+01:00 |

Timing+Execution host transfer time celkem0.146422 s včetně transportu; není to cena exportu v main-loop. FTP zpracováno sekvenčně až více než20 s po triggeru. Původní warning `expected100000, got EOF length` odpovídá skutečné délce malého SYS souboru, reply úspěšný a oba obsahy validované. Flight pending soubor20 B ověřen zvlášť. Snapshot po exportu byl pořízen před offline parse, aby lokální host výpočet zbytečně neprodlužoval dobu do snapshotu; hardware capture už bylo frozen. FREE/LRG po exportu stejné jako po výpočtu a všechny watermarky stabilní. Export neprokazuje nulovou přechodnou alokaci ani logger/write odolnost.

## 26. Motorová autorita

Původní source/symbol audit 3G-3F zachovaný (`source-isolation-audit.json`, `native-machine-code-comparison.json` v původním3G-3F adresáři). Původní 14 source manifest i konkrétníAPJ stále shodné. Aktivní UTB Core/Output/Acro/Angle na H743 compile-out; pasivní sonda pouze časuje původníAP volání. Podrobné původní doložení `docs/UTB_PHASE_3G3F_MAIN_OUTPUT_TIMING_CS.md`, část10. V tomto běhu rawTX neobsahuje ARM/motor/servo/PWM/DShot příkaz, pouze schválený RUN a telemetrii/read-only požadavky; ATC/motor/servo/RC/failsafe parametry beze změny. Originální AP má výhradní hodnotovou motorovou autoritu. Flight neběžel, proto se neuvádí neexistující motor_changes=0. Elektrické výstupy ani ESC se neměřily.

## 27. Logger health a původní logy

LOG_DISARMED0, BLOCK backend4, žádný StartNewLog/erase/download BIN. Logger health a errors_count3=0. Původní čtyři logy:2516746/7924746/718496/725746 B, celkem11885734 B, ID/count/last/time_utc i velikosti před/po shodné. Předběžný inventář pochází zA, poB jeden finální read-only inventář mimo okna. Neprokazuje byte-for-byte nový readback BIN nebo flash výkonnost při zápisu; staré plné archivy se neměnily.

## 28. Cleanup a konečný stav FC

UTT_RUN automaticky0; ostatníRUN0, UTT_COUNT4000 zachované bez zápisu COUNT. Všechny tři intervaly ACK/readback obnovené na−1. Úplná finální záloha1220 parametrů, změna pouze STAT_RUNTIME196802→196864 s; žádná letová konfigurace. Boot 115, DISARMED STABILIZE, EKFflags167, všechny InternalError/drop čítače0; finální SYS_STATUS load 18,7 %. Port uzavřený. Žádný další reconnect/měření; stav po uzavření nebyl dalším otevřením zbytečně ověřován.

## 29. Historické srovnání

| Experiment | Main max µs | Rate max µs | Att max µs | Rate→Att mean µs | CPU active % | Timing hook/Main µs |
|---|---|---|---|---|---|---|
| E-A | 2531 | 2531 | 3332 | 352.439750 | 19.194118 | 6.217500 |
| E-B | 2543 | 2548 | 3530 | 503.982250 | 28.910526 | 7.306250 |
| HW-2A | 2523 | 2523 | 3504 | 353.816500 | 19.111765 | 8.014750 |

Mean Rate→Att HW-2A−E-A=+1,376750 µs (+0,3906 %); Main max2523 proti2531 µs. Attitude max3504 µs proti3332/3530 µs je entry-period jitter při přesunu pomalejší práce, nikoliv sám důkaz ztráty Main/Rate cyklu. Agregátní identita Att_sum−Rate_sum−delay_change_sum=0, bez missing eventů a s3999 periodami. Nelze z omezeného kontextu přiřadit konkrétní scheduler úlohu všem extrémům. E-A/E-B používají starší firmware d78e34b3 a jiné booty; mají podobné vstupy, nikoliv izolovaný ExecutionOFF/ON důkaz. HW-1R RUN0 mean18,8 % má pouze5vzorků a není rovnocenná15sbaseline. M1–M5 pro E-A/E-B nebyly měřené; nevyplňují se falešné nuly.

![Společné skutečně měřené metriky E-A/E-B/HW-2A](../tmp/utb-3g3f-hw2a-20261025_reference-01/analysis/plots/historical-comparison.png)

## 30. Omezení a nejistoty

Jeden10s DISARMED native běh na USB, bez Flight výpočtu, LiPo, rotorové zátěže nebo senzorové dynamiky letu. Neprokazuje stabilitu dlouhého letu, logger/write/erase zatížení, odstranění historických USB výpadků nebo boot logger anomálie. Žádné PM.NLon/MaxT, plná transient stack/heap minima, elektrické dokončení DShot, ESC přijetí, hard deadline miss/WCET či izolovaná celková režie nejsou dodatečně vymyšlené. Z této session nebyl zjištěn Main/Rate interval>3000 nebo≥5000 µs, ale nepozorování v4000 krocích není časová garance.

## 31. Připravenost HW-2B a odpovědi

V rozsahu přijímacích kritérií HW-2A PASS: správná FC/firmware, jeden RUN, obě validní frozen,4000 sad, žádná chybová událost, parametry/log metadata/telemetrie zachované, bez pozorovaného alokačního/stack selhání. V tomto běhu se nezjistil konkrétní timing blocker pro přípravu HW-2B. Doporučení: **připravit jeden samostatně schválený USB-only DISARMED Flight+Timing+Execution běh**, stejná telemetrie/config a porovnání M1–M5 s touto referencí, plus původní Flight numerické a motorové audity. Nic z toho se automaticky nespouští; případný nový boot/pending a fyzické podmínky řešit samostatně. Žádný souhlas s letem.

| # | Otázka | Odpověď |
|---|---|---|
| 1 | Prošel preflight a jaký byl boot? | Ano; opravený read-only preflight i ověření před měřením, boot 115. |
| 2 | Byl využit původní boot? | Ano; bez fyzického/softwarového restartu. |
| 3 | Právě jeden UTT_RUN? | Ano; nezávisle ověřeno z raw TX: jediný PARAM_SET byl UTT_RUN=1. |
| 4 | Dokončeno 4000 Timing cyklů? | Ano; reason=1, 3999 period každého kanálu, 4000 delay hodnot. |
| 5 | Získáno 4000 Execution sad? | Ano; každá povinná metrika má N=4000. |
| 6 | Missing/duplicate/invalid? | Vše 0, rovněž gaps/unsupported a Timing missing_finish/phase_changes 0. |
| 7 | M1 min/mean/max? | 28 / 31.489500 / 62 µs |
| 8 | M2 min/mean/max? | 29 / 31.993500 / 41 µs |
| 9 | M3 min/mean/max? | 57 / 63.483000 / 96 µs |
| 10 | M4 min/mean/max? | 315 / 491.404750 / 1383 µs |
| 11 | M5 min/mean/max? | 13 / 15.368750 / 23 µs |
| 12 | Mode finish a fast boundary? | N=4000; mode 266/408.748250/1311 µs, fast 280/423.963750/1324 µs. |
| 13 | UTB consume vypnutý? | Ano, count=0 a mean absent; Flight pending, nikoliv 0µs výpočet. |
| 14 | Main/Rate/Attitude periody? | Main 2475/2500.000750/2523; Rate 2467/2499.992748/2523; Att 1467/2499.901225/3504 µs. |
| 15 | Rate→Att proti E-A? | 353,816500 proti 352,439750 µs mean; rozdíl +1,376750 µs. Historický kontext, nikoliv izolovaná cena sond. |
| 16 | CPU baseline/active/after? | 18,516667 / 19,111765 / 18,638889 %, N=30/17/18. |
| 17 | Naměřená část režie? | Timing 8,014750 µs/Main, max bracket 69 µs; Execution 4,354750 µs/Main, max bracket 61 µs. Nejde o úplnou dodatečnou režii ani WCET. |
| 18 | Stack/heap zhoršení? | Main/FTP beze změny; ISR free 1272→1256 B; první FREE/LRG 3680/3160→3688/3640 B, po exportu stabilní; selhání nepozorováno. |
| 19 | Oba frozen exporty prošly? | Ano, Timing 2746 B a Execution 1746 B, validace komplet. |
| 20 | Logy a konfigurace zachované? | Ano, všechny 4 log metadata shodné; 1220 parametrů, změnil se pouze STAT_RUNTIME. Intervaly obnovené na -1/-1/-1. |
| 21 | Významný timing blocker pro HW-2B? | V tomto omezeném běhu nezjištěn; Main/Rate ≤2523 µs, M4 max1383 µs, žádný ≥5000µs interval. |
| 22 | Vhodné následné Flight měření? | GO pouze k přípravě a samostatnému schválení HW-2B; nikoliv automatickému spuštění nebo letu. |
| 23 | DShot / hard WCET nejistota? | Elektrické dokončení/přijetí ESC ani hard WCET se neměřily; M2 je návrat softwarového volání. |

## 32. Git, evidence a ověření výstupů

HEAD/větev/index beze změny;7 modified + 20 untracked. Ze stávajících 7688 souborů žádný změněný/odstraněný; jediný nový běžný soubor mimo ignorované tmp je tento report. `git diff --check` doplněný při finální kontrole. Firmware ani produkčníanalyzátory se neupravovaly, nejsou nové C++/SITL/build výsledky. Skutečně nově proběhlo5 receive host regresí a12 measurement-host testů, offline replay parserů a úplná raw RX/TX revalidace. Žádné nové instalace.

Evidence: `tmp/utb-3g3f-hw2a-20261025_reference-01/`, strojový souhrn `analysis/summary.json`, CSVTiming/Execution/CPU/raw/context/command-timeline/system-time-alignment,9 PNG + 9 SVG grafů. SHA-256 všech artefaktů a reportu v `evidence-sha256.json`; jeho vlastní hash v `evidence-sha256.digest`. Plán, schválení, původní chyba i oběA evidence zachované. Grafy zobrazují pouze skutečné agregace a bounded kontext, nikoliv vymyšlenou4000cyklovou časovou řadu.
