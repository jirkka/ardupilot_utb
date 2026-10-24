# FÁZE 3E-5 – Kontinuální HW ověření UTB_Attitude SHADOW

Datum: 24. 10. 2026. Skutečná Skystars STM32H743, USB-only, bez LiPo a vrtulí, trvale DISARMED.

## 1. Závěr

Jeden schválený experiment `UAS_RUN=2 / UAS_COUNT=4000` dokončil **4000/4000 main-loop pokusů**, **4000 primary + 4000 skutečných yaw resetů**. Všech **8000 operací validních**, všech 26 exportovaných maximálních chyb i RMSE **0**, mismatch/DESYNC/invalid/unsupported **0**, `motor_changes=0`. Stop reason **1 – dokončený počet**.

Celá AP historie byla importována **jednou**, při první operaci. Dalších **7999 operací** navazovalo na vlastní historii UTB. Z toho lze podle skutečné STABILIZE cesty odvodit **4000 primary výpočtů bez importu před primary** a **3999 celých main kroků bez jakéhokoliv úplného importu**. První main krok obsahoval počáteční import před yaw resetem. Nejde o 8000 nezávislých regulačních period; jedna perioda obsahovala primary a reset.

**Rozsah potvrzení:** numerická shoda a kontinuita jsou ověřené pro tento klidový DISARMED profil se skutečnými gyro/AHRS vstupy a pravidelnými yaw resety. Nejsou tím ověřené dlouhodobá yaw historie bez resetů, nenulové dynamické input shaping, let nebo bezpečnost aktivního řízení. `UTT_RUN=0`, takže samostatná nová Main/Rate perioda není změřená.

DISARMED HW ověření Attitude lze pro uvedený profil uzavřít a připravovat společnou validaci Attitude→Rate/PID v SITL. Další HW experiment ani let v této fázi neproběhl.

## 2. Artefakty a firmware

Všechny nové artefakty: `tmp/utb-3e5-hw-20261024_152910/`.

| Soubor | Obsah |
|---|---|
| `source-check.json` | HEAD, Git status, hash zdrojového manifestu, APJ a kapitoly |
| `preflight/preflight.json`, `preflight/preflight-messages.jsonl` | Identita, health a úplných 1215 parametrů před testem |
| `parameters-before.param`, `parameters-after.param` | Úplné textové zálohy před/po |
| `parameter-comparison.json` | Rozdíly konfigurace proti 3E4 |
| `supplemental-preflight.json`, `attitude-before.txt` | Nový boot, pending session, původní log inventory a nepřítomný rate thread |
| `experiment-approval.json` | Rozsah jediného schváleného experimentu a fyzické podmínky |
| `measurement/result.json`, `measurement/telemetry.jsonl` | Přesný postup, všechny odchozí požadavky a přijatá telemetrie |
| `measurement/utb_attitude_shadow.txt`, `measurement/shadow.json` | Kompletní původní a strojově parsovaný frozen snapshot |
| `measurement/threads-before.txt`, `measurement/threads-after.txt` | Stack watermark |
| `analysis/metrics.csv`, `analysis/timing-brackets.csv`, `analysis/cpu.csv`, `analysis/summary.json` | Numerika, historie, časy a CPU |
| `final-check/preflight.json`, `final-parameter-changes.json` | Konečný read-only stav, logger inventory, změny parametrů |
| `final-integrity.json`, `artifact-sha256.json` | Kontrola zachování projektu a SHA-256 výsledků |

Repo `/home/jirka/ardupilot_utb`, větev `test/skystars-5inch`, HEAD **1a76160fc4527dbb8aae6a3454b143fa73c42ebb**. Commit **95affbde7ee7137c5ed50354e8671b772ad537c7** zachován.

Použit stávající firmware z 3E4, žádný nový build/upload. Původní APJ:

`tmp/utb-3e3-20261024_092051/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`

**1 146 983 B**, SHA-256 **f75531d90e21ea9ff199197599ba5764cd7127d28ee2ea714ad1a3f945e71672**. Lokální APJ i 17souborový source manifest odpovídají 3E3. COM5, board **1075**, USB serial **200024001251303433333336**, VID:PID 1209:5741, MAVLink 1/1. Firmware hlásí Copter 4.7.1 / **95affbde**, protože byl sestaven před druhým commitem 3E3. Integrita nahrané aplikace navazuje na verifikovaný upload 3E4 a shodnou živou identitu/verzi; v této fázi se neprováděl nový úplný readback MCU flash. MCU STM32H743 Rev.V je identifikace ze zachovaného bootloader preflightu/uploadu 3E4, nikoliv nový vstup do bootloaderu.

## 3. Nový boot a preflight

Obsluha potvrdila fyzické odpojení/připojení USB a USB-only stav bez LiPo a všech vrtulí. **STAT_BOOTCNT=104**, oproti 102 v konečném stavu 3E4. Mezi fázemi tedy přibyly dvě evidované boot události; automatizace této fáze žádný reboot neodeslala a nevyvozuje z toho jedinou událost od předchozího testu. Během preflightu, testu a závěrečné kontroly zůstal boot 104 a monotónní uptime bez pozorovaného restartu.

Před testem `@SYS/utb_attitude_shadow.txt` obsahoval přesně `UTBAttitudeShadowV1,pending`, Timing soubor rovněž pending. Snapshot se podruhé ověřil při vstupu měřicího runneru; starý frozen výsledek nebyl převzatý.

Read-only potvrzeno:

- DISARMED, native STABILIZE, SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0, žádný rate thread.
- Gyro/accel/AHRS/logger health a EKF attitude dostupné, bez pozorovaného InternalError.
- UAS_RUN=UNS_RUN=UTT_RUN=0; LOG_DISARMED=0, SCHED_OPTIONS=0. SMAX/NTF/NEF=0 na všech osách beze změny.
- Kompletní záloha **1215/1215 parametrů**. Proti 3E4 nejsou nové/odstraněné parametry ani změny PID/failsafe/motorové konfigurace.
- Rozdíly před testem proti starému bootu jsou STAT_RUNTIME, STAT_BOOTCNT, gyro offsets/kalibrační teploty, BARO1_GND_PRESS a COMPASS_DEC (0,09363012→0). Jsou uložené v přesných číslech; tato fáze je nepřepisovala ani neprováděla kalibraci. Původ každé mezifázové změny není samostatně prokázaný. Zdravý AHRS/EKF a podporovaný execution context byly ověřené v aktuálním bootu.
- Čtyři původní BIN logy mají shodná ID/časy/velikosti, celkem **11 885 734 B**. RAM-only experiment nepotřebuje další logovací kapacitu a logování se nezapínalo.

Po preflightu přišlo samostatné potvrzení jediného kontinuálního experimentu. Žádné ochrany nebo konfigurace se neměnily pro dosažení shody.

## 4. Přesný protokol jediného běhu

Před oknem přečtené stack údaje a čerstvá session. Dočasně nastavená stálá telemetrie SYS_STATUS **2 Hz**, SYSTEM_TIME **1 Hz**, EKF_STATUS_REPORT **1 Hz**, původní interval každé zprávy -1. Telemetrie byla stejná před/během/po. Po 3s ustálení baseline **15,016 s**.

Host elapsed od otevření měřicího runneru:

| Čas [s] | Operace |
|---:|---|
| 9,969–24,985 | Baseline UAS=UNS=UTT=0 |
| 24,985 | PARAM_SET UAS_COUNT=4000, původní hodnota byla také 4000 |
| 26,079 | Jediný PARAM_SET UAS_RUN=2 |
| následujících přibližně 11 s | Žádné další odchozí požadavky, pouze příjem přednastavené telemetrie |
| 37,141 | Konec tichého host okna, read-only ověření automatického UAS_RUN=0 a UTT_RUN=0 |
| dalších 8 s | Stejná telemetrie po testu, před exportem |
| po doběhnutí | FTP frozen Attitude snapshotu; validace výsledku |
| 45,422 / 45,469 | Obnovení UAS_RUN=0 / původního UAS_COUNT=4000 |
| do 53,719 | Health, stack po testu, obnovení všech tří stream intervalů na -1 a uzavření COM5 |

Audit odchozí komunikace v měřicí fázi obsahuje **pouze jeden PARAM_SET UAS_RUN=2**, žádné FTP, listing/download, polling, logger změny, další PARAM_SET, ARM nebo motorové příkazy. UNS_RUN/UTT_RUN se ani dočasně nezapnuly. Po experimentu následovala pouze read-only konečná záloha/health/log inventory. Žádné opakování ani další restart.

## 5. Úplnost, sekvence a numerika

| Exportovaná veličina | Výsledek |
|---|---:|
| attempts | **4000** |
| Primary STABILIZE command_0 seen / valid | **4000 / 4000** |
| RESET_YAW command_4 seen / valid | **4000 / 4000** |
| Ostatní command_1,2,3,5,6 seen / valid | **0 / 0** |
| seen / valid / motor_checks | **8000 / 8000 / 8000** |
| Invalid / DESYNC / UNSUPPORTED / mismatch | **0 / 0 / 0 / 0** |
| missing / duplicate primary / unsupported_loops | **0 / 0 / 0** |
| first_seq → last_seq | **1 → 8000** |
| first_loop → last_loop | **1 → 4000** |
| mode_changes / config_changes | **0 / 0** |
| quaternion_sign_equivalent | **0** |
| motor_changes | **0** |
| Stop reason | **1, automatické dokončení počtu** |

Sequence gap se samostatně neexportuje jako nový čítač. Zdroj `AP_UTB_AttitudeShadow.cpp:128`–137 kontroluje každé `seq == předchozí + 1`; desync=0 společně s počtem a hranicemi sekvence dokládá žádnou detekovanou mezeru. Nejde o závěr pouze z počátečního/koncového čísla.

Všech 26 položek má exportovanou max. absolutní chybu **0**, RMSE **0**. `mismatch=0` znamená žádnou validní operaci s chybou přes toleranci `1e-6 × max(1, |AP hodnota|)`; z toho pro každou veličinu plyne **0 překročení tolerance**. Samostatné per-metric čítače překročení firmware nemá, uvedené nuly jsou odvozené z nulového globálního mismatch. Exportované nuly nejsou nový bitový audit interních floatů pod rozlišením textového formátu.

| Metric ID | Porovnávaná skupina | Počet operací na veličinu | Max / RMSE | Překročení tolerance |
|---|---|---:|---:|---:|
| 0–3 | Target quaternion q1–q4 | 8000 | 0 / 0 | 0 |
| 4–7 | Attitude-error quaternion q1–q4 | 8000 | 0 / 0 | 0 |
| 8–10 | Euler target R/P/Y | 8000 | 0 / 0 | 0 |
| 11–13 | Euler-rate target R/P/Y | 8000 | 0 / 0 | 0 |
| 14–16 | Angular velocity target x/y/z | 8000 | 0 / 0 | 0 |
| 17–19 | Angular acceleration target x/y/z | 8000 | 0 / 0 | 0 |
| 20–22 | Body-rate target x/y/z | 8000 | 0 / 0 | 0 |
| 23 | Thrust angle | 8000 | 0 / 0 | 0 |
| 24 | Thrust error | 8000 | 0 / 0 | 0 |
| 25 | Yaw feedforward scalar | 8000 | 0 / 0 | 0 |

Úplný jednotlivý seznam 26 veličin je v `analysis/metrics.csv`. Quaterniony prošly unit-length/finite kontrolou; ekvivalentní opačné znaménko se při porovnávání podporuje, ale v tomto běhu ho nebylo potřeba použít. Export poskytuje agregace, ne všech 8000 individuálních float vektorů; nelze nově rekonstruovat průběh chyby uvnitř každé operace.

## 6. Co bylo skutečně kontinuální a co resetované

### Důkaz ze zdrojového kódu

- `AP_UTB_AttitudeShadow.cpp:246`: `prepare()` spouští `_engine.start(_run.get() == 2)`, takže RUN=2 skutečně volí continuous.
- `AP_UTB_AttitudeShadow.cpp:159`–163: úplný setter `_math.shadow_history(f.before)` se volá jen při `!_continuous || !_synced`. Jinak následuje **read-only porovnání** vlastní historie s AP prehistorií; při neshodě se zaznamená DESYNC a neproběhne maskování importem ve stejném kroku.
- `AP_UTB_AttitudeShadow.cpp:166`–170: `_math.execute(f.kind, f.command, f.reset_rate)` počítá nový stav vlastní instancí, poté se vlastní stav porovnává s native post stavem. Hotový AP post výstup se do UTB nekopíruje.
- `UTB_Attitude.cpp:1121`–1150 vymezuje přesně getter/setter historie. Per-call `shadow_config` a `sample` kopírují konfiguraci a gyro/AHRS vstupy, nikoliv kompletní interní historii. Účinné gain scale a dt jsou skutečné vstupy každého volání.
- `ArduCopter/mode_stabilize.cpp:29`–35: při DISARMED SHUT_DOWN je před primary výpočtem skutečný AP `reset_yaw_target_and_rate()`. Native hook zachytí RESET_YAW a reset flag v `AC_AttitudeControl.cpp:1357`–1370. Default `reset_rate=true` je deklarovaný v `AC_AttitudeControl.h:176`.

Pozorované **synchronizations=1**; mode/config changes=0, žádný unsupported/desync/invalid. Od počáteční epochy tedy není doložená žádná další úplná synchronizace. **Počáteční import 1, import po změně epochy 0, skryté průběžné plné importy nezjištěny**; závěr se opírá o zdroj i čítače, nikoliv pouze o nulovou chybu.

**7999 následných operací bez úplného importu** je přímo `seen − synchronizations`. Rozdělení na 4000 primary bez importu před primary a 3999 celých main kroků bez importu je odvozené ze source order DISARMED STABILIZE: první operací je yaw reset s počáteční synchronizací, teprve potom primary. Snapshot neexportuje první command jako samostatnou položku ani seznam importů po jednotlivých cyklech; rozdělení není vydáváno za další nezávislý hardware čítač.

### Skutečné externí resety

**4000 RESET_YAW replay**, resets=reset_compared=4000. Full target/rate reset, relax a inertial-frame reset **0**; změny módu a konfigurace **0**.

`UTB_Attitude::reset_yaw_target_and_rate()` (`UTB_Attitude.cpp:847`):

1. Z aktuální AHRS yaw a vlastního Euler yaw targetu vypočte yaw shift a levým násobením otočí **vlastní** target quaternion. Není to kopie celého native target quaternionu; Roll/Pitch target se nenastavuje na AHRS Roll/Pitch.
2. Při reset_rate=true nuluje vlastní Euler yaw-rate target a yaw angular-acceleration target.
3. Z vlastního quaternionu a Euler-rate targetu znovu odvodí **celý** body-frame angular velocity feedforward target. Proto tento vektor není v resetových okamžicích čistě volně integrovaný stav.
4. Roll/Pitch Euler-rate/acceleration historie, attitude-error quaternion a další stavové proměnné se neimportují z AP; primary je dále aktualizuje vlastní UTB matematika. Některé z nich jsou nově odvozené výstupy každého primary, nikoliv nezávislé integrátory.

**Neověřeno:** nezávislé dlouhodobé hromadění yaw orientace/rate/akcelerace bez resetu. Roll/Pitch a ostatní proměnné sice zůstaly ve vlastní instanci, ale to při téměř klidové referenci není důkaz správné odezvy všech jejich paměťových stavů za dynamického buzení.

`dynamic=8000`, `shaping=0`. Čítač dynamic testuje `own.body.length_squared()>1e-6` a `own.thrust_error>1e-4`, ne pohyb FC nebo změnu pilotní reference; resetové operace mohou zdědit nenulový body target z předchozího primary. Proto **dynamic=8000 není důkaz 8000 dynamických pilotních impulsů**. FC podle potvrzení ležela v klidu, nebylo aplikováno řízené pohybové/RC buzení. Skutečné IMU/AHRS mohou mít šum a nenulovou attitude odchylku. Per-call command a absolutní stavové hodnoty se neexportují; nelze z těchto agregací doložit jejich amplitudy nebo naprostou nulovost. Shaping s nenulovou angular-acceleration historií tento profil neověřil.

![Historie a Attitude intervaly](../tmp/utb-3e5-hw-20261024_152910/analysis/history-and-period.png)

## 7. H743 časování a skutečná režie

Měření je z monotónních MCU časů, ne ze SITL. Brackety obsahují i případnou preempci; nejsou čistou počítanou cenou instrukcí.

| Bracket | Součet [µs] | Průměr / operaci [µs] | Průměr / main krok [µs] | Max / operaci [µs] |
|---|---:|---:|---:|---:|
| Pre capture | 28614 | 3,57675 | 7,15350 | 58 |
| Post capture / audit bez consume | 24153 | 3,019125 | 6,03825 | 56 |
| Sync/validace historie | 144571 | 18,071375 | 36,14275 | 141 |
| UTB math včetně resetových výpočtů | 136118 | 17,01475 | 34,02950 | 135 |
| Comparison | 75280 | 9,41000 | 18,82000 | 106 |
| Celý engine consume | 377735 | 47,216875 | 94,43375 | 191 |
| Native span včetně capture kontextu | 148659 | 18,582375 | 37,16475 | 138 |

Sync bracket obsahuje validaci, konfigurační kontrolu, sensor snapshot a buď import, nebo kontrolu vlastní návaznosti. **Není to 8000 importů historie.** Čas prvního importu a samotného replay resetu není zvlášť exportovaný; replay math je v math/consume součtech všech operací. Samostatná průměrná/maximální cena resetu proti primary tedy není dostupná.

Přidané nepřekrývající se brackety **pre + post + consume = 430502 µs**, tj. **107,6255 µs/main**, **53,81275 µs/operaci**, přibližně **4,305 % nominálních 2500 µs**. Nezahrnují všechny instrukce guardů a časového accountingu. Native span není dodatečná režie. Sync/math/comparison se ke consume znovu nepřičítají a maxima různých operací se nesčítají do falešného WCET.

| Měřený průměr / main krok | 3E2 one-step | 3E4 one-step | 3E5 continuous |
|---|---:|---:|---:|
| UTB math [µs] | 33,746 | 33,859 | 34,0295 |
| Sync/validace [µs] | 22,783 | 23,454 | 36,14275 |
| Consume [µs] | 81,204 | 81,754 | 94,43375 |
| Pre + post + consume [µs] | 94,934 | 94,967 | **107,6255** |

Proti 3E4 je měřený přidaný součet **+12,6585 µs/main (+13,33 %)**, proti 3E2 +12,6915 µs/main (+13,37 %). Continuous může být pomalejší než import: místo setteru provádí navíc projekci/porovnání kompletní vlastní historie, quaternion/finite kontroly a tolerance. Největší nárůst naměřeného span je sync/validace **+12,68875 µs/main** proti 3E4; math téměř shodná. Jde o srovnání skutečných agregací z různých bootů/oken, nikoliv izolovaný instruction benchmark. 3E4 mělo aktivní Timing navíc, 3E5 UTT=0; jeho celkový systémový load se proto nesmí porovnávat jen odečtem těchto tří bracketů.

![Porovnání bracketů](../tmp/utb-3e5-hw-20261024_152910/analysis/timing-comparison.png)

Attitude primary timestamp: **3999 intervalů, 1631–3338 µs**, průměr **2500,0915 µs**, průměrný absolutní jitter vůči 2500 µs **207,0293 µs**, průměrná frekvence **399,9854 Hz**. Jsou to **Attitude entry intervaly**, ne Main/Rate periody. Velký Attitude jitter zůstává podobného rozsahu jako předchozí měření; v této fázi se neopravoval. Počet intervalů překračujících konkrétní časové meze, histogram, P95/P99 a jednotlivý časový průběh se v tomto exportu neuchovávají.

**Main perioda, Rate perioda, jejich jitter, PM.NLon, PM.MaxT a UBST8 nejsou v tomto běhu měřené.** UTT zůstalo vypnuté, LOG_DISARMED=0, tasks.txt se nečetl. Staré hodnoty 3E4 se nedoplňují jako nová měření. Úplnost attempts/sekvence potvrzuje zpracované pokusy, sama nedokazuje absenci vynechané nominální 2,5ms periody. Stabilní průměrná frekvence není garance hard deadline.

## 8. CPU, stack, zdraví a motorová izolace

| SYS_STATUS CPU load | Počet zpráv | Průměr [%] | Rozsah [%] |
|---|---:|---:|---:|
| Baseline | 30 | **17,920** | 17,6–18,2 |
| Vnitřní aktivní část | 18 | **22,617** | 22,3–22,8 |
| Po testu, před FTP | 16 | **17,900** | 17,6–18,2 |

Nárůst proti baseline **+4,697 procentního bodu**, návrat po testu prakticky k baseline. CPU okna jsou přiřazená podle host command času s vynecháním 0,5s okrajů; SYS_STATUS load je průměrovaný údaj scheduleru, nikoliv výhradní profil UTB. Pozdější špičky při post-window exportu/health požadavcích nejsou součástí aktivního porovnání. Nelze je vydávat za in-window prodlevu.

![CPU](../tmp/utb-3e5-hw-20261024_152910/analysis/cpu.png)

Stack watermark:

- **ArduCopter 4352/7168 B volných před i po**, bez pozorovaného poklesu.
- **ISR 1288→1280/1536 B**.
- FTP **1736→1568/2992 B**; post hodnota zahrnuje až následné exporty.
- Ostatní vlákna v kompletních threads souborech; rate thread nepřítomný.

Rezerva main je pro pozorovaný call-chain značná, ale watermark není důkaz bezpečného WCET/stack maxima za letu. Statistiky časových bracketů nemají individuální rozložení a jejich maxima nepředstavují transitive peak celého cyklu.

Pozorované HEARTBEATy po celou dobu DISARMED STABILIZE, SYS_STATUS InternalError pole 1/2/4 nulová, EKF attitude dostupné, gyro/AHRS/logger health prošly, bez nečekaného restartu. Žádná detekovaná stop událost. V částech přijímaných klientem MAVFTP se telemetry health znovu ověřila po přenosu; to není nový nepřetržitý HW watchdog.

Motorová a native-target izolace:

- **8000 auditů, motor_changes=0**. Native `AC_AttitudeControl.cpp:1717`–1728 při inertním výpočtu porovnává R/P/Y motorové zásahy, R/P/Y FF a native `_ang_vel_body_rads` před/po. Nulový čítač tedy zahrnuje i absenci změny tohoto native control targetu, ne pouze motorových hodnot.
- `AP_UTB_AttitudeShadow.h:16`–60: samostatný hodnotový State, inertní konstruktor Attitude, rate/throttle výstupy no-op. UTB nemá předaný živý AP_Motors ani setter native AP stavu; `f.before/f.after` jsou kopie pro synchronizaci/porovnání.
- Zdrojový manifest i původní H743 firmware zůstaly shodné se 3E3; active UTB compile-out ověřený v původním symbol auditu platí pro stejný APJ. V této fázi se žádná guard větev neobcházela.
- Neprovádělo se ARM, PWM/DShot změna, elektrické měření ani motorový test. RAM audit neprokazuje fyzickou letovou bezpečnost.

## 9. Konečný stav a zachování projektu

Po testu kompletní read-only záloha **1215/1215 parametrů**. Proti preflightu se změnil pouze **STAT_RUNTIME 141535→142246**; všechny ostatní hodnoty stejné, včetně boot104. UAS_RUN=UNS_RUN=UTT_RUN=0; UAS_COUNT=UNS_COUNT=UTT_COUNT=4000, LOG_DISARMED=0, SCHED_OPTIONS=0, původní telemetrické intervaly -1 obnovené. Board/UID/verze, DISARMED a zdraví ověřené, COM5 uzavřený.

Log listing před/po identický: ID1 2516746 B, ID2 7924746 B, ID3 718496 B, ID4 725746 B, celkem 11885734 B; nevznikl nový BIN a nic se nemazalo. Je to kontrola metadat, nikoliv nový bitový download všech starých logů.

Žádné změny C++/firmwaru, loggeru, scheduleru, regulačních parametrů, žádný build, unit/SITL běh nebo instalace. Historické software/build výsledky se neopakují jako nové testy. Nově provedené jsou jeden H743 experiment, offline validace snapshotu/sekvencí/metrik a konečná integrita/parametry.

HEAD i větev beze změny, index a tracked worktree čisté. Necommitnuté jsou původní report 3E4, tento nový report a původní `kapitoly/historie.txt`. Kapitola i staré artefakty zachované podle SHA manifestů. Pomocné skripty a výsledky jsou jen v ignorovaném novém tmp. Bez reset/clean/commit/push.

## 10. Rozhodnutí k dalšímu postupu

1. **Numerická shoda continuous potvrzena v provedeném profilu:** 8000/8000 valid, všech 26 max/RMSE exportováno jako 0, žádné překročení tolerance.
2. **Jedna počáteční synchronizace, 7999 následných operací s vlastní historií**, 4000 replay yaw resetů. Žádná další epochová synchronizace. 3999 celých main kroků bylo bez importu; všechny 4000 primary navazovaly na vlastní post-reset stav.
3. **Yaw cílová orientace a yaw rate/akcelerace byly opakovaně resetované**, angular velocity FF přepočítávaný. Roll/Pitch a další vlastní stavy se nekopírovaly, ale jejich plná dynamická odezva bez klidové degenerace není ověřená. Nenulový input shaping/ACRO/rotace nejsou nové HW PASS.
4. **Měřená přidaná práce 107,6255 µs/main**, o 12,6585 µs/main více než 3E4 one-step, hlavně kontrola návaznosti historie. CPU +4,697 p.b., main stack 4352 B volných. Kompletní overhead/WCET neměřený.
5. **Motorová izolace i pozorované zdraví zachované.** Attitude intervaly odpovídají předchozímu typu jitteru; samostatnou Main/Rate stabilitu ani absenci hard deadline misses tento běh nově nepotvrzuje.
6. **Doporučeno uzavřít omezené DISARMED HW ověření Attitude a připravit společnou SITL validaci řetězce.** V ní párovat Attitude(N) output se skutečně spotřebovaným Rate(N+1) targetem a příslušným gyro/dt/saturation/reset stavem, rozlišit počáteční synchronizaci, externí reset a volný vývoj. Zahrnout nenulové reference, obě znaménka, input shaping/FF, native ACRO/STABILIZE a přechody/resetové události. Současné oddělené snapshoty nejsou end-to-end důkaz. Další hardware nebo aktivní UTB řízení tímto schválené není a žádná navazující fáze se automaticky nespustila.
