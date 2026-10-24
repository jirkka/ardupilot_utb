# FÁZE 3E-4 – HW časování Main / Rate / Attitude

## 1. Výsledek a rozsah

**Skutečné H743 měření 24. 10. 2026:** jeden schválený USB-only DISARMED experiment, 4000 timing cyklů, uvnitř 1000 one-step Attitude kroků. Hlavní smyčka měla periodu **2449–2550 µs**, Rate **2448–2553 µs**, Attitude **1630–3349 µs**. Velký Attitude jitter je lokalizovaný především na proměnlivou dobu **mezi Rate a Attitude**, nikoliv na odpovídající nepravidelnost začátku hlavní smyčky. Konkrétní funkce uvnitř tohoto úseku dosud není prokázána.

Attitude porovnání: **1000 primary + 1000 skutečných yaw resetů, všech 2000 operací validních**, všech 26 exportovaných veličin max. chyba i RMSE **0**, `motor_changes=0`. Výsledek neověřuje nezávislou kontinuální historii, aktivní řízení, dynamický let ani letovou bezpečnost.

Omezení exportéru: v RAM byly agregovány všechny 4000 cykly, ale ven se exportuje jen **17 raw záznamů** v okolí nejhoršího Attitude intervalu. Nezávislá kontrola identity po jednotlivých párech je proto **16/16 dostupných párů**, nikoliv 3999/3999. Součtová identita sedí ve všech agregovaných skupinách; tím se nenahrazuje nezávislé přepočítání každého páru. Požadavek na úplný raw audit každého párování zůstává částečně neověřený. Z dostupných dat nelze vytvořit histogram ani P95/P99 jednotlivých period.

Artefakty: `tmp/utb-3e4-hw-20261024_101555/`. Surové exporty jsou `measurement/utb_timing.txt` a `measurement/utb_attitude_shadow.txt`, telemetrie `measurement/telemetry.jsonl`, souhrn `analysis/summary.json`, tabulky `analysis/timing-statistics.csv`, `analysis/raw-context.csv`, `analysis/raw-pairs.csv`, `analysis/cpu.csv`. Časy v tabulkách jsou skutečné monotónní MCU timestampy nebo wall-time brackety na H743; host čas slouží jen k přibližnému přiřazení telemetrie.

## 2. Identita, upload a zachování konfigurace

- Větev `test/skystars-5inch`, HEAD `1a76160fc4527dbb8aae6a3454b143fa73c42ebb`.
- COM5, VID:PID `1209:5741`, board ID **1075**, USB serial **200024001251303433333336**. MAVLink systém/komponenta 1/1.
- Nahrán výhradně schválený `tmp/utb-3e3-20261024_092051/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`, **1 146 983 B**, SHA-256 **f75531d90e21ea9ff199197599ba5764cd7127d28ee2ea714ad1a3f945e71672**. Standardní uploader identitu ověřil a aplikaci po zápisu verifikoval.
- Firmware hlásí Copter 4.7.1 / `95affbde`: APJ byl sestaven před druhým commitem 3E3. Shoda jeho 17souborového source manifestu s HEAD je ověřena; nejde o jiný dodatečný build.
- Před uploadem záloha **1213/1213** parametrů, po uploadu a na konci **1215/1215**. Přibyly pouze `UTT_RUN=0`, `UTT_COUNT=4000`. Při startu se změnily runtime/boot statistiky, gyro offsets a kalibrační teploty, ground pressure a nepatrně automatická compass declination. Letové PID, failsafe, motorové ani RC nastavení se nepřepisovalo. Přesné rozdíly: `post-upload-parameter-comparison.json`.
- Po uploadu kompletní inicializace a kontrola health prošly; `STAT_BOOTCNT=102` zůstal po celý experiment i konečnou kontrolu stejný. Nebyl další restart ani druhý pokus.
- Po celou dobu DISARMED, native STABILIZE; fyzicky potvrzené USB-only, bez LiPo a všech vrtulí. Žádný ARM, motorový příkaz nebo aktivní UTB režim.

## 3. Execution context a provedený postup

Read-only ověřeno `SCHED_LOOP_RATE=400`, `FSTRATE_ENABLE=0`, SMAX/NTF/NEF=0 na RLL/PIT/YAW, nepřítomný rate thread, `UNS_RUN=0`, `UTB_SHADOW=0`, `LOG_DISARMED=0`, `SCHED_OPTIONS=0`. Logger health byl dostupný, ale DataFlash se pro experiment neotevíral.

Stejná minimální telemetrie pro baseline/aktivní/po: SYS_STATUS 2 Hz, SYSTEM_TIME 1 Hz, EKF_STATUS_REPORT 1 Hz; původní intervaly všech tří zpráv byly -1 a byly obnovené. Baseline trvala 15,031 s. Jednou se nastavil `UTT_RUN=1`; po **3,079 s host času** jednou `UAS_RUN=1`. Po zbytku okna nebyly odesílané žádné další požadavky. Audit odchozích paketů v měřicí fázi obsahuje pouze tyto dva plánované PARAM_SET. Žádné FTP, param polling, listing/download nebo změna loggeru.

Obě sessions skončily automaticky s reason=1; teprve potom se ověřily RUN=0 a stáhly immutable RAM snapshoty a stack. Counts byly obnovené na původních 4000. Výjimky/warnings klienta MAVFTP typu `expected 100000, got ...` označovaly kratší dokončený virtuální soubor; všechny přenosy vrátily error_code=0 a struktura/obsah exportů prošly kontrolami. Nevznikla chyba loggeru nebo opakovaný experiment.

## 4. Společné časování

`T_main` je původní scheduler sample timestamp po `INS.wait_for_sample`; `T_rate` vstup native Rate controlleru, `T_att` vstup primary Attitude operace. Měří se intervaly těchto bodů, nikoliv délka celého regulačního výpočtu nebo jeho hard deadline.

| Skupina / veličina | Počet | Minimum [µs] | Maximum [µs] | Průměr [µs] | Průměrný absolutní jitter vůči 2500 µs |
|---|---:|---:|---:|---:|---:|
| ALL Main | 3999 | 2449 | 2550 | 2500,003 | 2,929 |
| ALL Rate | 3999 | 2448 | 2553 | 2499,996 | 3,851 |
| ALL Attitude | 3999 | 1630 | 3349 | 2499,981 | 204,098 |
| UAS OFF Main | 2998 | 2454 | 2545 | 2500,007 | 3,130 |
| UAS OFF Rate | 2998 | 2452 | 2546 | 2499,999 | 4,023 |
| UAS OFF Attitude | 2998 | 1630 | 3319 | 2499,841 | 203,490 |
| UAS ON Main | 999 | 2449 | 2550 | 2500,008 | 2,314 |
| UAS ON Rate | 999 | 2448 | 2553 | 2499,999 | 3,336 |
| UAS ON Attitude | 999 | 1634 | 3349 | 2499,672 | 205,600 |

4000 cyklů = 3000 OFF + 1000 ON. Dva přechodové intervaly jsou zahrnuté pouze v ALL: čisté skupiny obsahují 2998 + 999 párů. Žádné OTHER cykly. Main intervalů >2500 µs bylo 1335/3999, čisté OFF 1027/2998 a ON 308/999. Rate 1712/3999, Attitude 2013/3999. **Intervalů ≥5000 µs bylo nula** u všech tří bodů. Meze 2500/5000 µs jsou diagnostické značky z exportéru, nikoliv nové bezpečnostní limity.

Main maximum je o 50 µs nad nominální periodou a hodnoty nad 2500 µs existují. Nelze tvrdit přesnou periodu bez jitteru ani důkaz absence všech deadline misses. V tomto desetisekundovém okně však není zachycená ztráta celé periody, gap sekvence nebo milisekundová nepravidelnost začátku cyklu odpovídající velkému Attitude jitteru. PM.NLon, PM.MaxT a per-task overruns **nebyly měřené**: LOG_DISARMED zůstal 0 a `tasks.txt` se nepoužil. Nula v těchto položkách se nedoplňuje.

| Rate → Attitude delay | Počet | Min [µs] | Max [µs] | Průměr [µs] |
|---|---:|---:|---:|---:|
| ALL | 4000 | 212 | 1114 | 355,271 |
| UAS OFF | 3000 | 212 | 1083 | 345,975 |
| UAS ON | 1000 | 243 | 1114 | 383,159 |

Změna delay mezi cykly ALL **−868 až +840 µs**, průměr absolutní změny **203,349 µs**; OFF 202,905 µs, ON 204,351 µs. UAS ON má v tomto běhu větší průměrný delay o **37,184 µs**, ale velký jitter je přítomný i OFF. Jeden krátký běh neposkytuje statistický test významnosti, randomizaci ani WCET.

## 5. Lokalizace jitteru a identita

Nejhorší uložený interval je cyklus **3984, UAS OFF**, následník delšího intervalu v cyklu 3983:

| Cyklus | T_main [µs] | T_rate [µs] | T_att [µs] | Delay [µs] |
|---|---:|---:|---:|---:|
| 3982 | 336082510 | 336082524 | 336082804 | 280 |
| 3983 | 336085011 | 336085025 | 336086108 | 1083 |
| 3984 | 336087510 | 336087523 | 336087738 | 215 |

V cyklu 3984: Main interval **2499**, Rate **2498**, Attitude **1630**, změna delay **−868 µs**. Přesně platí **1630 = 2498 − 868**. V cyklu 3983 je Rate interval 2501 µs, nárůst delay 803 µs a Attitude interval 3304 µs. Main přitom 2501 µs. Tím je přímo doložen vznik dlouhého/krátkého Attitude intervalu změnou mezilehlého zpoždění při pravidelném Rate.

Identita `ΔT_att = ΔT_rate + Δ(delay)` má nulové reziduum u **16/16** exportovaných sousedních párů. Součtová rezidua ALL (3999), OFF (2998), ON (999) jsou rovněž 0. U ostatních jednotlivých párů jsou dostupné jen agregace, nelze je nezávisle rekonstruovat. Nejdelší Attitude interval **3349 µs je cyklus 2063 při UAS ON**; jeho raw okolí se neuchovalo, protože engine uchovává okolí největší absolutní odchylky od 2500 µs, kterou má kratší interval 1630 µs.

Výsledek odpovídá převážně variantě **A**. Drobný Main/Rate jitter zůstává, ale nevysvětluje ~0,85ms změny Attitude. Varianta B/C nevysvětluje dominantní anomálii. Pro D je doložená dodatečná práce a CPU load, nikoliv vznik velkého jitteru výhradně při ON: OFF má prakticky stejný jitter a obsahuje nejhorší absolutní interval.

Pořadí zdrojů: `ArduCopter/Copter.cpp:115`–139 (INS, Rate, motor output, starý shadow hook, AHRS, inertia, EKF reset, flight mode); Main hook `libraries/AP_Scheduler/AP_Scheduler.cpp:366`, Rate hook `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:508`, finish `ArduCopter/mode.cpp:540`. Tyto odkazy vymezují úsek. **EKF, AHRS, motor output, přerušení ani mutex nejsou tímto měřením jednotlivě označené za viníka.** Původní konkrétní vzorky 1623/3318 µs z 3E2 neměly společné timestampy; nyní je lokalizovaná reprodukce stejného typu jitteru, ne dodatečný důkaz přesné příčiny starých vzorků.

![Worst context](../tmp/utb-3e4-hw-20261024_101555/analysis/worst-context.png)
![OFF a ON](../tmp/utb-3e4-hw-20261024_101555/analysis/off-on-timing.png)

## 6. Numerika, motorová izolace a režie

Všechny 4000 timing cykly supported, missing_rate/att/finish, duplicate_rate/att, invalid_time, cycle_gaps i unsupported **0**, phase_changes **2**, stop reason **1**. Jde o úplnou agregaci; nikoliv export 4000 jednotlivých řádků.

Attitude: attempts=1000, command_0 primary=1000/1000, command_4 yaw reset=1000/1000; seen=valid=motor_checks=2000, synchronizations=2000, resets=reset_compared=1000. Invalid/desync/unsupported/mismatch/missing/duplicate, mode/config changes i motor_changes **0**. One-step každou operaci synchronizuje z AP, tudíž to není ověření nezávislé historie. `dynamic=2000`, ale `shaping=0`: není prokázaná excitace nenulové shaping akcelerace nebo pilotních vstupů. Nulové chyby se vztahují právě na tuto DISARMED konfiguraci.

Motorový audit porovnával hodnoty během 2000 operací; žádná změna diagnostikou. H743 active compile-out a původní motorová autorita AP zůstávají v přesně stejném ověřeném buildu 3E3. V této fázi se zdroje nezměnily; elektrický výstup se neměřil a není tím ověřen aktivní let.

| Měřený bracket | Průměr / operaci [µs] | Průměr / main krok [µs] | Max. jedné operace [µs] |
|---|---:|---:|---:|
| Pre-capture | 3,519 | 7,037 | 32 |
| Post-capture/audit bez consume | 3,088 | 6,176 | 55 |
| Import historie (sync) | 11,727 | 23,454 | 106 |
| UTB math | 16,930 | 33,859 | 147 |
| Comparison | 9,465 | 18,929 | 73 |
| Consume, obsahuje sync/math/comparison | 40,877 | 81,754 | 173 |
| Native AP span včetně capture kontextu | 18,272 | 36,544 | 137 |

Nepřekrývající se přidané Attitude brackety pre + post + consume dávají **94,967 µs/main** (3,799 % nominálních 2500 µs). V 3E2 bylo 94,934 µs/main; rozdíl tohoto měřeného součtu +0,033 µs/main není důkaz identické WCET. Native span není dodatečná režie. Sync/math/comparison se k consume znovu nepřičítají. Maxima různých operací nelze sčítat na nejhorší dobu jednoho cyklu.

Timing hooky: **25 089 µs / 24 000 volání**, průměr **1,045 µs/hook**, **6,272 µs/cyklus** (0,251 % nominální periody), max. bracket **67 µs**. Měří vlastní timing práci včetně případné preempce, ne AP výpočty ani UAS math. Guard, části měření hodin/accountingu/publish jsou mimo bracket. Jde o měřenou část wall-time režie, **nikoliv kompletní overhead nebo WCET**. Součet nesmí být vydávaný za přesný nový požadavek na CPU ani za max. všech hooků v jednom cyklu.

| SYS_STATUS load okno | N | Průměr [%] | Min–max [%] |
|---|---:|---:|---:|
| Baseline, UTT/UAS OFF | 30 | 18,847 | 18,5–19,3 |
| UTT ON / UAS OFF před | 5 | 19,100 | 19,0–19,3 |
| UAS ON vnitřní část | 4 | 23,375 | 23,2–23,6 |
| UTT ON / UAS OFF po | 8 | 19,050 | 18,7–19,3 |
| Obě sessions OFF po okně, před exportem | 16 | 18,669 | 18,3–19,2 |

CPU je původní scheduler load z telemetrie, nikoliv měření pouze UTB. Aktivní fáze jsou přiřazené podle host command času s 0,3s okraji; zpráva obsahuje průměrované zatížení a může mít zpoždění. Krátký ON výsek má pouze 4 vzorky. Rozdíl přibližně +4,53 procentního bodu proti baseline je pozorovaný, nikoliv přesně izolovaný kauzální overhead. Timing-only rozdíl ~0,20–0,25 p.b. odpovídá řádu naměřených hook bracketů. Load při pozdějším FTP není součástí experimentální baseline/ON porovnání.

![CPU](../tmp/utb-3e4-hw-20261024_101555/analysis/cpu.png)

Stack watermark před/po: **ArduCopter 4352/7168 B volných beze změny**, ISR **1272/1536 B beze změny**, FTP **1736→1576/2992 B**, storage **960→952/1456 B**. Export proběhl až po měření, takže FTP watermark zahrnuje exporty. Rezerva main je v pozorovaném běhu značná; watermark není důkaz bezpečné rezervy pro všechny kombinace call-chain, přerušení a letu. Rate thread nepřítomný před i po.

## 7. Konečný stav, integrita a další krok

Konečná read-only kontrola: COM5/board/UID souhlasí, DISARMED STABILIZE, gyro/AHRS/logger health a EKF attitude dostupné, bez pozorovaného InternalError nebo restartu. RUN přepínače **UAS=UTT=UNS=0**, všechny tři COUNT **4000**, LOG_DISARMED=0, SCHED_OPTIONS=0; telemetrické intervaly obnovené na -1. Kompletní konečná záloha je `final-check/parameters.param`; proti post-upload záloze se změnil pouze STAT_RUNTIME. COM5 uzavřený.

Čtyři staré logy mají před i po stejná ID, časové údaje a velikosti: 2 516 746, 7 924 746, 718 496 a 725 746 B, celkem **11 885 734 B**. Žádný nový BIN, mazání nebo přepis logů. Identické listing metadata nedokazují novou bitovou verifikaci celého flash obsahu; úplný download nebyl prováděný.

Žádný nový build, unit/SITL běh, instalace, úprava regulátorů nebo scheduleru. Platí doložené výsledky buildu/regresí 3E3, nikoliv nově provedené testy. Nově se ověřila fyzická H743, struktura snapshotů, agregace, dostupné raw identity, audit telemetrie a obnovení konfigurace. Verifikace manifestů a Git stavu je v `final-integrity.json`; hash manifest nových artefaktů v `artifact-sha256.json`.

Změna mimo ignorované tmp je pouze tento nový dokument. Původní `kapitoly/historie.txt` zůstává beze změny mimo index. HEAD nadále `1a76160fc4527dbb8aae6a3454b143fa73c42ebb`, předchozí commit `95affbde7ee7137c5ed50354e8671b772ad537c7` zachován. Index a tracked worktree čisté, untracked dokument + kapitoly. Bez commit, reset nebo push.

**Rozhodnutí:**

1. Hlavní 400Hz sekvence byla v tomto okně pravidelná s drobným jitterem a všemi cykly; žádná zaznamenaná dvouperiodová prodleva. Přesná hard-deadline garance z toho neplyne.
2. Reprodukovaný velký Attitude jitter pochází převážně z variability úseku Rate→Attitude. Konkrétní operace je nadále neobjasněná.
3. One-step UAS zvyšuje CPU a měřený čas práce, ale nevytvořilo významné zhoršení Main/Rate jitteru tohoto krátkého běhu. Dominantní Attitude jitter existuje už OFF.
4. Timing brackety 6,272 µs/cyklus, Attitude přidané brackety 94,967 µs/main; main stack volných 4352 B. Nejde o úplné WCET ani schválení letové rezervy.
5. **Lze doporučit samostatně schválený následný USB-only DISARMED kontinuální test UAS_RUN=2 / UAS_COUNT=4000**, s novým fyzickým startem kvůli uzavřeným RAM sessions, stejnými health/stop podmínkami a bez tvrzení letové bezpečnosti. Z tohoto běhu nevyplývá nutnost nejprve měnit regulační matematiku nebo scheduler. Test zde neproběhl. Kompletní nezávislý audit všech raw párů by vyžadoval další výslovně schválenou diagnostiku.

Pokud má být určena konkrétní příčina Rate→Attitude variability, nejmenší navazující návrh je samostatná RAM-only sonda se společným cycle ID: konec native Rate, konec motor outputu, před/po `read_AHRS`, po `read_inertia`, po `check_ekf_reset` a vstup primary Attitude. Agregovat min/max/sum jednotlivých **nepřekrývajících se** úseků a uchovat společné okolí největšího delay; vypnutou variantu použít jako kontrolu vlastní režie. Pokud se maximum lokalizuje do AHRS nebo jiného úseku, teprve potom přidat jeho vnitřní body. Zvlášť zachovat resetovou operaci, přerušení/preempci a případná čekání jako dosud nerozlišené příspěvky. Žádný logger/text/export v měřeném okně. Tato další sonda je pouze návrh, nebyla implementována ani nahrána.
