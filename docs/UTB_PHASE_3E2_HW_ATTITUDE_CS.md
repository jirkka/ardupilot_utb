# FÁZE 3E-2 – První HW ověření 400Hz Attitude SHADOW

Datum: 2026-10-24. Repo `/home/jirka/ardupilot_utb`, větev `test/skystars-5inch`, HEAD `070c189cdfa8357307ee1aebe8f3677cd849d520` + zachovaný rozpracovaný strom.

**Skutečné H743 měření: numerická shoda a RAM motor audit prošly. Časování vyžaduje další vyhodnocení; kontinuální test zatím nedoporučen.** Proběhl právě jeden one-step experiment. Žádný kontinuální běh, ARM, motorový test ani let.

Artefakty: [`tmp/utb-3e2-hw-20261024_083723/`](../tmp/utb-3e2-hw-20261024_083723/). Surový export: [`measurement/utb_attitude_shadow.txt`](../tmp/utb-3e2-hw-20261024_083723/measurement/utb_attitude_shadow.txt); strojové výsledky: [`analysis/summary.json`](../tmp/utb-3e2-hw-20261024_083723/analysis/summary.json), CSV a PNG/SVG ve stejné složce. Výsledky nejsou převzaté ze SITL.

## 1. Identita, souhlas a firmware

Windows COM5, USB VID:PID `1209:5741`, serial `200024001251303433333336`. Identitu potvrdily Windows USB inventory, MAVLink AUTOPILOT_VERSION/uid2 a před zápisem také bootloader: board ID **1075**, protokol 5, STM32H743/753 revize V. Původní firmware hlásil Copter 4.7.1-dev / `070c189c`; parametr UAS_RUN chyběl.

Po výslovném schválení konkrétního souboru a potvrzení USB-only / bez LiPo / bez vrtulí byl nahrán pouze:

`tmp/utb-3e1-20261023_203945/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`

- Velikost: **1 143 607 B**.
- SHA-256: **`1e62b800a7dc57f9c3ce88e597050bfd4c65752386314339524cfae868bcb7c4`**.
- APJ board_id: **1075**. Před uploadem znovu ověřeno.
- Standardní uploader ověřil zapsanou aplikaci a spustil ji. Bootloader, parametry ani externí log flash se nepřepisovaly. Vstup do bootloaderu a start po uploadu byly součástí schváleného uploadu; žádný další reboot ani opakování experimentu neproběhly.
- SHA označuje ověřený lokální APJ; nejde o nezávislé přečtení SHA aplikace z MCU. Vazbu dokládá identita bootloaderu, úspěšné ověření uploadu a následné nové parametry UAS_.

Před uploadem: 70,844 s read-only pozorování a úplná záloha **1211/1211 parametrů**. Po uploadu: 70,734 s pozorování a kontrola **1213/1213 parametrů**. UAS_RUN=0, UAS_COUNT=4000, UNS_RUN=0. Nebyl odstraněn žádný původní parametr; přibyly pouze dva UAS parametry. Změny gyro offsetů/kalibračních teplot a BARO1_GND_PRESS souvisejí s inicializací; změnily se také provozní statistiky a automatická COMPASS_DEC. Žádné parametry PID/Attitude, failsafe nebo motorů nebyly zapisovány. Přesné rozdíly: `parameters-after-upload-comparison.json`.

## 2. Skutečně provedený protokol

- Native STABILIZE, trvale DISARMED; SYS_STATUS kontrolované gyro/AHRS/logger health bity zdravé, EKF attitude dostupná, EKF flags 167, bez hlášeného InternalError.
- SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0. Před i po okně v `threads.txt` není rate thread. SMAX/NTF/NEF na všech osách 0; konfigurace zůstala původní. ATC_RATE_FF_ENAB=1.
- UNS_RUN=0, UTB_SHADOW=0, LOG_DISARMED=0, SCHED_OPTIONS=0. Historické UTB_ENABLE=1 se neměnilo a nový Attitude experiment tím není řízen.
- Před testem export `UTBAttitudeShadowV1,pending`. Snapshot nebyl použit z předchozí session.
- Telemetrie SYS_STATUS 2 Hz, SYSTEM_TIME a EKF_STATUS_REPORT 1 Hz; předchozí intervaly -1 byly uloženy. Žádný GCS heartbeat ani parametr failsafe se nepřidával; FS_GCS_ENABLE=0 zůstalo zachované.
- 15 s baseline, potom UAS_COUNT=1000. Jediný zápis UAS_RUN=1 v host čase 25,781 s od začátku protokolu.
- Tiché okno trvalo přibližně 4,078 s; vlastní 1000krokový výpočet nominálně 2,5 s. Audit odchozích paketů v tomto okně obsahuje **právě jeden PARAM_SET UAS_RUN=1**. Žádný další PARAM_SET, request, FTP ani logger operace.
- Po okně ověřeno automatické UAS_RUN=0; až následně RAM export a stack diagnostika. `@SYS/tasks.txt` se nepoužil.
- UAS_COUNT obnoven na 4000, UAS_RUN=0; tři telemetrické intervaly obnoveny na -1. Skript skončil a COM5 uzavřel.

Čtyři původní logy měly součet 11 885 734 B; jejich ID, velikosti a časy jsou po uploadu i testu stejné. RAM test nezakládal nový log a nevyžadoval logovací prostor. Seznam logů sám neposkytuje přesnou mapu volných/předmazaných sektorů; neuvádí se z něj měření volné kapacity. Nebyl stažen ani smazán žádný BIN. FTP upozornění na kratší blok než požadovaných 100000 B odpovídala konci malých textových souborů; transfery skončily bez chyby a export obsahuje kompletní hlavičku, čítače i 26 metrik.

## 3. Numerická shoda a úplnost

| Metrika | Skutečný výsledek |
|---|---:|
| Main-loop pokusy | **1000** |
| Primary STABILIZE command_0 seen / valid | **1000 / 1000** |
| RESET_YAW command_4 seen / valid | **1000 / 1000** |
| Všechny operace seen / valid | **2000 / 2000** |
| Synchronizace | **2000** |
| Reset události / porovnané resety | **1000 / 1000** |
| Invalid / DESYNC / UNSUPPORTED / mismatch | **0 / 0 / 0 / 0** |
| Unsupported main-loop / missing / duplicate | **0 / 0 / 0** |
| Změny módu / konfigurace | **0 / 0** |
| Seq / loop | **1–2000 / 1–1000** |
| Motor checks / motor_changes | **2000 / 0** |
| Stop | **reason=1, dokončen počet kroků** |
| Maximum abs / maximum RMSE přes 26 veličin | **0 / 0** |

Počet resetů odpovídá skutečnému `ModeStabilize::run()` ve SHUT_DOWN: `reset_yaw_target_and_rate()` před primary Attitude voláním ([zdroj](../ArduCopter/mode_stabilize.cpp#L31)). Reset není další regulační cyklus. One-step importuje AP prehistorii pro každou operaci, tedy 2000 synchronizací; **nezávislá kontinuální historie tím ověřena není**.

Všech 26 exportovaných maxim i RMSE je 0: čtyři složky target quaternionu, čtyři error quaternionu, Euler target, Euler-rate target, angular velocity/acceleration target, body-rate target, thrust angle/error a yaw FF scalar. Jednotkové quaterniony a konečnost stavů prošly validací; ekvivalentní změny znaménka quaternionu 0. CSV `differences.csv` mapuje každý index na veličinu.

`dynamic=2000` znamená splnění konkrétního kritéria nenulového body-rate a thrust error v enginu; není to důkaz excitace všech os. `shaping=0` znamená, že export nezaznamenal nenulový angular-acceleration target nad interním kritériem. FF je konfiguračně zapnutý, ale tento klidový DISARMED profil **neověřuje dynamickou shaping odezvu na pohyb páček**. Nebyly vynucovány reference, pohyb FC ani přepínání módů. ACRO, relaxace, inertial reset a volná kontinuální historie nejsou tímto měřením pokryté.

![Numerické odchylky](../tmp/utb-3e2-hw-20261024_083723/analysis/differences-h743.png)

## 4. Skutečné H743 časování

Monotónní AP_HAL::micros() na H743; wall time může zahrnovat přerušení/preempci. Průměry jsou ze 2000 skutečných operací; poslední sloupec rozpočítává součet na 1000 main kroků. Maxima různých spanů nejsou nutně ze stejné operace.

| Měřený span | Průměr / operace [µs] | Max / operace [µs] | Součet / main krok [µs] |
|---|---:|---:|---:|
| Pre-capture | 3,786 | 55 | 7,572 |
| Post-capture + audit, consume odečten | 3,079 | 52 | 6,158 |
| Sync / validace / import historie | 11,3915 | 79 | 22,783 |
| UTB math | 16,873 | 141 | 33,746 |
| Comparison | 9,578 | 89 | 19,156 |
| Celý consume | 40,602 | 160 | 81,204 |
| Native AP + capture interval | 18,7035 | 136 | 37,407 |

**Nesčítat překrývající se spany.** Consume zahrnuje sync, math, comparison a okolní práci. Native+capture obsahuje původní AP výpočet a není celý dodatečnou režií UTB. Neprovádí se odečítání nesouvisejících maxim.

Součet nepřekrývajících se měřených úseků **pre-capture + post-capture/audit + consume = 94,934 µs/main krok v průměru**, tedy 3,79736 % nominálních 2500 µs. To je přímo naměřený součet bracketů, nikoli kompletní dodatečné WCET: některé enable/context/prepare/finish/timer/accounting instrukce leží mimo tyto úseky. Per-loop maximum ani rozložení/P95/P99 export neobsahuje. Náklady one-step importu se mohou od budoucího continuous režimu lišit; tento běh je neměří.

![Časové spany](../tmp/utb-3e2-hw-20261024_083723/analysis/timing-h743.png)

## 5. Perioda, jitter a scheduler

| Veličina primary Attitude vstupů | Výsledek |
|---|---:|
| Počet intervalů | 999 |
| Součet | 2 496 723 µs |
| Průměr | **2499,222222 µs** |
| Odpovídající průměrná frekvence | **400,124483 Hz** |
| Minimum / maximum | **1623 / 3318 µs** |
| Průměrný absolutní jitter proti 2500 µs | **207,142142 µs** |
| Největší časné / pozdní odchýlení intervalu | **−877 / +818 µs** |

**Časová odchylka zůstává nevysvětlená.** Zdroj statistiky je `before()` primary Attitude příkazu ([zdroj](../libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp#L277)), tedy uvnitř flight mode po INS, native Rate, motor output, AHRS a DISARMED yaw resetu. Není to timestamp začátku INS nebo celé hlavní smyčky. Změna času práce před tímto místem může změnit tyto intervaly i při pravidelném začátku smyčky.

Předchozí 3D-4/3D-5 měřily jiný bod, native rate vstup: jitter 2,924 / 3,241 µs, maxima 2517 / 2548 µs. Srovnání upozorňuje na riziko, **nedokazuje kauzální regresi hlavní smyčky**. Tento firmware/export neposkytl současnou distribuci INS/main-loop entry pro RUN=0/1, jednotlivé primary intervaly ani časové pozice extrémů. Nelze určit počet pozdních intervalů, skutečných hard deadline misses nebo jejich původ. Ani všech 1000 dokončených pokusů a průměr 400 Hz takový důkaz nenahrazují.

Anomálie byla dostupná až z uzavřeného RAM snapshotu po automatickém dokončení; za běhu se statistiky nečetly. Runner měl následnou kontrolu maxima pod dvě nominální periody, ta není schváleným limitem přijatelného jitteru a neopravňuje označit 3318 µs za PASS. Výsledek časování je proto **RISK/HOLD**. Po zjištění nebyl proveden další běh, reboot ani úprava firmware.

![Perioda](../tmp/utb-3e2-hw-20261024_083723/analysis/period-h743.png)

SYS_STATUS.load (scheduler ukazatel, nikoli detailní profil jednotlivých instrukcí):

| Okno | Vzorky | Průměr [%] | Min–max [%] |
|---|---:|---:|---:|
| Baseline 15 s | 30 | **17,99** | 17,7–18,3 |
| Odhadované aktivní okno ~2,5 s | 5 | **22,12** | 21,5–22,6 |
| Po automatickém stopu před exportem | 16 | **18,05** | 17,8–18,6 |

Rozdíl aktivní okno vs. baseline je **+4,13 procentního bodu**. Hranice aktivního okna jsou odhadnuty z host času RUN příkazu a skutečného součtu period; export neposkytuje první/poslední hardware TimeUS. USB doručení, dispatch a agregace scheduler load znamenají nejistotu, zejména u krajních telemetrických vzorků. Intervaly během baseline/testu/po byly stejné.

PM.NLon, PM.MaxT, per-task budget overruns a přesná perioda/jitter baseline a po testu jsou **neověřené**. LOG_DISARMED zůstal 0, nevznikal průběžný BIN a logger se neotevíral kvůli získání těchto hodnot. Nuly do těchto polí nebyly doplněny. Pole SYS_STATUS errors_count1/2/4 zůstala 0; errors_count3 (logger drop diagnostické pole) také 0. Toto není test propustnosti běžícího DataFlash loggeru.

![Scheduler load](../tmp/utb-3e2-hw-20261024_083723/analysis/cpu-h743.png)

## 6. Stack a motorová izolace

ThreadsV2 exporty byly mimo okno, před baseline a po uzavření session/exportu:

| Thread | Volný watermark před [B] | Po [B] | Přidělený stack [B] |
|---|---:|---:|---:|
| ArduCopter main | **4344** | **4344** | **7168** |
| ISR | 1296 | 1288 | 1536 |
| FTP | 1592 | 1584 | 2992 |
| Původní utb_log worker | 2472 | 2472 | 3504 |

Main má pozorovanou volnou rezervu přibližně 60,6 % přiděleného stacku; test watermark dále nesnížil. Jde o minimum dosud dotčené zásobníkové paměti, nikoli okamžitý SP, úplný worst-case důkaz nebo bezpečnostní limit. FTP běželo mimo okno a nový Attitude modul nevytváří worker. ISR změna může zahrnovat i následný export. Absence overflow/ InternalError a kladný watermark podporují pouze tento krátký profil.

RAM audit provedl **2000 kontrol bez změny motorových hodnot**. Zdrojová izolace a H743 active compile-out z 3E-1 zůstávají beze změny: inertní math adaptér bez živého AP_Motors, žádný zápis UTB do native controlleru; výstupní autoritu má původní AP. Kontrola integrity všech 7644 původních souborů dle checkpointu doplněného manifestem 3E-1 nezjistila změnu.

Tento audit sleduje zachycené RAM motorové/řídicí hodnoty kolem hooků. Nejde o měření elektrického DShot/PWM signálu, ozbrojeného mixéru nebo chování motorů; USB-only DISARMED test takový důkaz neposkytuje.

## 7. Konečný stav a integrita

- FC zůstala DISARMED, native STABILIZE, STANDBY; poslední dostupný EKF flags 167, gyro/AHRS/logger health v kontrolovaném rozsahu zdravé, InternalError čítače 0.
- UAS_RUN=0, UAS_COUNT=4000, UNS_RUN=0, UNS_COUNT=4000; LOG_DISARMED=0, SCHED_OPTIONS=0. Tři telemetrické intervaly vráceny na -1, všechna potvrzení úspěšná.
- Finální read-only kontrola opět obdržela všech **1213 parametrů**. Proti post-upload preflightu se změnil pouze STAT_RUNTIME. STAT_BOOTCNT=101: žádný další restart během testu/exportů. Log inventory zůstalo shodné.
- COM5 uzavřený; už žádná další komunikace/test po závěrečné kontrole není potřebná.
- V této fázi nepřibyl žádný C++/Python produkční zdrojový zásah. Přidán pouze tento dokument; pomocné skripty a data jsou v ignorovaném tmp. Nic se neinstalovalo, nepřekládalo ani necommitovalo. Testy/buildy 3E-1 se neopakovaly a nejsou prezentovány jako nové testy.
- HEAD a větev beze změny, index prázdný; všechny původní rozpracované soubory zachované. `source-integrity.json` ověřil 7644 souborů a také **44 + 38 + 847** historických evidence položek 3D-4 / 3D-5 / 3E-1, bez neshody. Podrobný Git status a SHA-256 artefaktů jsou uložené vedle reportu.

## 8. Rozhodnutí a nejmenší další krok

1. **Numerická shoda: PASS v rozsahu tohoto profilu.** 1000 primary + 1000 skutečných yaw resetů, všechny exportované max/RMSE 0, bez invalid/desync/unsupported/mismatch. Nejde o continuous, dynamický shaping nebo letovou validaci.
2. **Motorová izolace: PASS pro source/RAM audit.** 2000/0, AP motorová autorita zachovaná. Elektrický výstup nebyl měřen.
3. **Režie: změřena, časová stabilita neuzavřená.** Nepřekrývající se diagnostické brackety mají průměr 94,934 µs/main krok; load +4,13 p. b. Neobjasněné primary-entry jitter 207,142 µs a max 3318 µs vyžadují vyhodnocení. Žádný důkaz ztraceného hlavního cyklu ani důkaz jeho nepřítomnosti.
4. **Stack: pozorovaná rezerva vyhověla tomuto měření.** Main 4344/7168 B volných před i po; nejde o obecný safety/WCET certifikát.
5. **UAS_RUN=2 / 4000 kroků zatím nedoporučen.** Nejprve vyjasnit časové měřicí body a rozdíl od Rate baseline. Nejmenší následný návrh je odděleně schválené RAM-only měření společného INS/main-loop nebo rate-entry timestampu a primary Attitude timestampu současně, se stejnou telemetrií v RUN=0/1 a s polohou extrémů v čase. Tím půjde oddělit jitter začátku smyčky od proměnlivého zpoždění před Attitude. Žádná taková instrumentace ani další HW běh v této fázi nebyly provedeny; případný patch/upload vyžaduje další schválení.
