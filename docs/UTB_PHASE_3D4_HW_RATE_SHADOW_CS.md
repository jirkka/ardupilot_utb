# FÁZE 3D-4 — první 400Hz hardware měření Rate/PID SHADOW

Datum: 22. 10. 2026. Skutečné měření na STM32H743, nikoli SITL.
Větev `test/skystars-5inch`, HEAD `070c189cdfa8357307ee1aebe8f3677cd849d520`.

## Výsledek a rozhodnutí

| Otázka | Závěr |
| --- | --- |
| Numerická shoda na skutečných 400Hz AP vstupech? | **ANO v provedeném one-step DISARMED profilu:** 1000/1000 validních sad, max. absolutní chyba a RMSE všech 54 veličin jsou 0. |
| Motorová izolace? | **Zachována v měřeném okně:** 1000 kontrol šesti nativních R/P/Y + FF hodnot, motor_changes=0; H743 aktivní UTB cesta zůstává compile-out. |
| Skutečně změřená režie? | UTB math průměr 8,560 µs / max 15 µs; měřený engine consume span 55,943 µs / max 69 µs. Čistá celková dodatečná capture/prepare/audit režie není odděleně změřena. |
| Pokračovat k RUN=2/COUNT=4000? | **Doporučeno k samostatnému schválení jednoho delšího DISARMED testu za stejných podmínek.** Měření neukázalo blokující problém; nejde o schválení letu nebo obecný důkaz bezpečnosti/WCET. |

Jediný schválený test byl proveden jednou. Kontinuální 4000krokový test ani let
neproběhl. Další session vyžaduje nové rozhodnutí a nový fyzický start, protože
stávající snapshot se v této boot session záměrně nepřepisuje.

## Identita, firmware a integrita

- Windows COM5, USB VID:PID 1209:5741.
- USB serial a MAVLink uid2: `200024001251303433333336`.
- Board ID 1075 potvrzen přes MAVLink i bootloader. Bootloader protokol/revize 5,
  STM32H743/753 revision V; aplikační kapacita 1 703 936 B.
- Nahrán výhradně schválený APJ:
  `tmp/utb-3d3-20261021_191141/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`.
- SHA-256: `aebc7cfa73773e552e790a289f7c83f16c8794bed71b56a6101fc4daf18775aa`.
- APJ 1 131 335 B; rozbalený image 1 253 660 B, git identity `070c189c`.
- Upload ověřen standardním bootloader CRC. Mazání/programování se týkalo
  aplikační firmware oblasti, nikoli bootloaderu, konfigurace či externí log flash.
  Uploader měl `no_extf=True`, APJ nemá externí flash image.
- Před uploadem byl získán výslovný souhlas pro tento SHA a jeden uvedený test.
  Žádný další upload ani dodatečný softwarový reboot po standardním uploadu.
- Před i po experimentu souhlasí všech 23 SHA-256 manifestu 3D-3. Závěrečná kontrola
  celkem 7636 předchozích checkpoint/3D-3 souborů nenašla změnu ani odstranění.

Zdroje, PID matematika, logger, scheduler a Attitude SHADOW se v této fázi neměnily.
Nový soubor v repozitáři je pouze tento report; experimentální skripty a data jsou
v ignorovaném adresáři `tmp/utb-3d4-hw-20261022_202457`. Není nový build, commit, reset ani push.

## Preflight, start a konfigurace

Před uploadem proběhlo 51,4 s read-only komunikace, největší pozorovaný interval
heartbeat 1,047 s. FC byla DISARMED; gyro/akcelerometr, AHRS/EKF a logger zdravé.
Záloha obsahuje všech 1212/1212 parametrů (`preflight.json`, `.param`). Původní
firmware byl Copter 4.7.1/070c189c bez UNS_ parametrů.

První kontrola krátce po uploadu zachytila chybějící AHRS health flag a skončila
**před jakýmkoliv spuštěním benchmarku**. Následovalo pouze read-only pozorování
inicializace, nikoli opakování experimentu. V jeho prvním SYSTEM_TIME vzorku při
uptime 52 615 ms již byly health flags a EKF attitude v pořádku; další 12s health
pozorování a 30s záloha prošly. Počáteční chybějící AHRS flag je konzistentní s
inicializací po bootu, nikoli důkaz zamrznutí. První neúspěšný krátký záznam sám
neobsahuje SYSTEM_TIME, proto z něj nelze určit přesné trvání inicializace.
Fyzické odpojení nebo další restart nebyly potřeba.

Po uploadu je 1211 parametrů. Společná PID a letová konfigurace se nezměnila.
Rozdíly záloh jsou nativní gyro bias/teplotní kalibrace po startu, BARO1_GND_PRESS,
COMPASS_DEC a STAT_BOOTCNT/STAT_RUNTIME. Původní kalibrační hodnoty se nevracely:
nejde o ruční změnu tune a nové nativní kalibrace odpovídají aktuálnímu startu.
Tyto aktualizace doloženy v `parameters-after-upload-evaluated.json`, zejména:

- `AP_InertialSensor.cpp:1827–1829`: vlastní startup gyro offset/caltemp.
- `AP_Baro.cpp:398`: aktualizace referenčního tlaku.
- `AP_Compass.cpp:2007`: automatická deklinace.
- `AP_Stats.cpp:82–112`: boot/runtime čítače.

Původní diagnostické UB3_RUN, UTB_B_SYNTH a UTB_B_WAIT_US nový build nevystavuje;
jsou compile-out. Přibyly UNS_RUN a UNS_COUNT. Nebyl vydán příkaz pro mazání nebo
obnovení parametrů. Tabulky ukazují viditelné parametry, nikoli dump celého NVRAM.

Před během ověřeno:

| Položka | Hodnota |
| --- | --- |
| UNS_RUN / UNS_COUNT | 0 / 4000 |
| SCHED_LOOP_RATE / FSTRATE_ENABLE | 400 / 0 |
| RLL/PIT/YAW SMAX | 0 / 0 / 0 |
| RLL/PIT/YAW NTF i NEF | všechny 0 |
| LOG_DISARMED / LOG_BACKEND_TYPE | 0 / 4 |
| UTB_SHADOW / SCHED_OPTIONS | 0 / 0 |

Konfigurace se nepřizpůsobovala kvůli PASS. Žádný rate thread nebyl v závěrečném
seznamu vláken; navíc všech 1000 kroků prošlo execution guardem, který odmítá i
boot session, v níž rate thread dříve běžel. Aktivní UTB motorové režimy na H743
zůstávají vyřazené, jak dokládá linked-symbol audit stejného APJ z 3D-3.

## Přesný průběh jediného měření

1. Znovu ověřena identita a výše uvedené parametry.
2. Před baseline nakonfigurována pouze dočasná MAVLink telemetrie: SYS_STATUS 2 Hz,
   SYSTEM_TIME a EKF_STATUS_REPORT 1 Hz. Původní intervaly byly dotázány a byly -1
   (vypnuto). Nejde o změnu AP parametrů, loggeru ani datové rychlosti SHADOW.
3. 15,015 s baseline s UNS_RUN=0 při stejné telemetrii.
4. Jednou nastavena UNS_COUNT=1000 a po krátkém klidovém intervalu jednou UNS_RUN=1.
5. Následovalo klidové okno do host času 23,453 s. Jediný odchozí paket v celé
   fázi measurement byl počáteční PARAM_SET UNS_RUN=1 v čase 19,375 s; poté žádný
   polling RUN, FTP, log operace, nový diagnostický požadavek ani heartbeat z hostu.
   Běžná již nakonfigurovaná telemetrie byla pouze přijímána.
6. Teprve po klidovém okně ověřeno automatické UNS_RUN=0. Následovalo 8 s
   pozorování po testu, pak read-only export frozen RAM snapshotu, 2127 B.
7. Obnoveno UNS_RUN=0 a UNS_COUNT=4000. Zdravotní kontrola, pak pouze mimo okno
   read-only `@SYS/threads.txt` (1135 B) pro dostupný stack watermark.
8. Zdravý konečný stav, obnoveny původní tři telemetrické intervaly na -1,
   ověřeny konečné parametry, port zavřen.

Zdrojová stopa všech odchozích paketů je v `measurement/result.json` a příjem v
`measurement/telemetry.jsonl`. Nebyly žádné ARM/force ARM, motor test nebo PWM/DShot
příkazy. Logger zůstal vypnutý pro DISARMED po celou dobu. Žádný log listing,
BIN download, mazání flash nebo `@SYS/tasks.txt` nebyly provedeny.

## Numerická shoda a úplnost

| Metrika frozen snapshotu | Výsledek |
| --- | ---: |
| Stop reason / continuous | 1 (COUNT dokončen) / 0 |
| prepared_attempts / seen / valid | 1000 / 1000 / 1000 |
| invalid / desync / unsupported / mismatch | 0 / 0 / 0 / 0 |
| unconsumed_attempts | 0 |
| first_seq / last_seq | 1 / 1000 |
| synchronizations | 1000 |
| config_changes / mode_changes | 0 / 0 |
| native_motor_checks / skipped / changes | 1000 / 0 / 0 |
| Max. absolutní odchylka / max. RMSE | 0 / 0 |

Porovnáno 18 veličin na každé ze tří os v 1000 krocích, tedy 54 000 dvojic.
P/I/D/FF/D_FF, filtry, relevantní stavy a výsledné zásahy jsou numericky shodné
pro tuto skutečnou sekvenci vstupů. Nejde o samostatné syntetické fixture.
FC ležela v klidu, DISARMED, bez LiPo/vrtulí a bez motorového zatížení.

**Je to one-step ověření:** import relevantní AP historie nastává v každém kroku,
což přímo potvrzuje synchronizations=1000. Neověřuje nezávislou kontinuální
historii, integraci v aktivním letu, široké reference, všechny saturace ani
libovolné notch/SMAX konfigurace. Tyto scénáře nelze označit HW PASS.

Motor audit porovnával stejné nativní R/P/Y a FF hodnoty před a po UTB výpočtu;
nevznikla žádná změna. To spolu s hodnotovým rozhraním a compile-out aktivní UTB
cesty podporuje závěr o motorové izolaci. Nejde o měření fyzického tahu nebo
DShot signálů na nenapájených ESC.

## Skutečné časy na H743

Časování používá AP_HAL::micros() → `hrt_micros32()` v ChibiOS
(`AP_HAL_ChibiOS/system.cpp:350–360`, `hwdef/common/hrt.c:122`). Jde o monotónní
čas fyzické FC, ne virtuální SITL čas ani Python host wall time.
Každý span může obsahovat přerušení/preempci a režii samotných časových čtení.

| Rozsah | Součet z 1000 kroků [µs] | Průměr [µs] | Maximum [µs] |
| --- | ---: | ---: | ---: |
| UTB math — tři vlastní PID a rate výpočet | 8560 | 8,560 | 15 |
| Engine process — měřený consume span, včetně math | 55943 | 55,943 | 69 |
| Původní AP PID + capture společný span | 17538 | 17,538 | 25 |

![Měřené časové rozsahy](../tmp/utb-3d4-hw-20261022_202457/analysis/timing-h743.png)

Měřený consume span začíná konstrukcí ShadowTiming v engine process a končí
čtením hodin v jeho destruktoru. Obsahuje import prehistorie one-step, validace,
vlastní math, porovnání a numerické RAM akumulátory. Následná aktualizace
samotných timing čítačů, wrapper a motor audit jsou mimo tento span. Rozdíl průměrů consume − math je 47,383 µs zpracování enginu mimo
vlastní rate math; nejde o cenu celé capture cesty. Consume průměr představuje
2,238 % nominální 2500µs periody, maximum 2,760 %. Jde o poměr měřeného wall span,
nikoli přesné rozdělení CPU cyklů.

**Rozsahy nelze sčítat jako čistou dodatečnou režii.** Math je v consume a
native+capture obsahuje již původní AP PID. Není izolovaný native-only timing
pro odečtení; samostatně se neměří všechno prepare/copy/audit ani jednorázový start
akumulátorů. Celková dodatečná režie proto zůstává neověřená, stejně jako její
přesný WCET. P95/P99 a jednotlivé výpočetní časy export neukládá.

### Perioda a jitter

| Veličina | Naměřeno / odvozeno z naměřených součtů |
| --- | ---: |
| Rate-entry first / last [µs od bootu] | 443766980 / 446264473 |
| Počet intervalů | 999 |
| Součet intervalů | 2 497 493 µs |
| Průměr periody | 2499,992993 µs |
| Minimum / maximum | 2481 / 2517 µs |
| Průměrný absolutní jitter vůči 2500 µs | 2,923924 µs |
| Největší absolutní odchylka periody | 19 µs |
| Frekvence ze součtu intervalů | 400,001121 Hz |

![Perioda a agregovaný rozsah](../tmp/utb-3d4-hw-20261022_202457/analysis/period-h743.png)

Tisíc vzorků má 999 vzájemných intervalů; 2 497 493 µs není chybná délka údajně
1000 intervalů. Perioda je kadence rate-entry v hlavní smyčce, nikoli samotná
výpočetní doba úloh nebo raw IMU backend čas. Poslední native dt=0,0024969999 s,
INS frontend timestamp=446264471 µs, čas čtení gyra=446264476 µs, primární gyro=0.

V tomto krátkém okně není dlouhá perioda typu původních 5,5–6,5 ms ani mezera mezi
pořadovými kroky. Jednotlivé periody a jejich percentily nejsou uložené; graf
zobrazuje pouze skutečné min/průměr/max, nikoli vymyšlené rozložení. Nelze z toho
odvodit univerzální hard-deadline garanci ani stav celé předchozí boot session.

## Zatížení, scheduler a ztráty

`SYS_STATUS.load / 10` je zde zatížení vykázané z
`AP::scheduler().load_average()` (`GCS_Common.cpp:6037–6043`). Nejde o samostatný
DWT profil všech vláken a ISR. Stejné intervaly telemetrie platily pro baseline,
SHADOW i fázi po testu.

| Fáze | Vzorků | Průměr [%] | Min / max [%] |
| --- | ---: | ---: | ---: |
| RUN=0 baseline | 30 | 18,406667 | 18,1 / 18,8 |
| RUN=1 okno | 5 | 21,180000 | 20,5 / 21,4 |
| Po okně, před exportem | 16 | 18,412500 | 18,2 / 18,7 |

Pozorovaný rozdíl průměrů je **+2,773333 procentního bodu**. Jen pět on-window
vzorků a vyhlazování scheduler load omezují přesnost kauzálního rozdělení této
změny. Okno CPU grafu je přibližně zarovnáno přes SYSTEM_TIME a host časy příjmu;
není to synchronní CPU údaj každého z 1000 PID kroků.

![Zatížení scheduleru před/během/po](../tmp/utb-3d4-hw-20261022_202457/analysis/cpu-h743.png)

Všechny zachycené SYS_STATUS mají InternalError bitmask/count 0 a logger dropped
counter 0. Význam errors_count1/2/4 a errors_count3 ověřen přímo v
`GCS_Common.cpp:6021–6061`. Bez nového SHADOW logování nevznikla bounded log queue;
úplnost měření dokládají RAM čítače, nikoli počet nových DataFlash zpráv.

**PM.NLon, PM.MaxT a počet task-budget overruns nebyly tímto během změřeny.**
LOG_DISARMED=0 se nezměnil, nový RAM export tyto původní scheduler čítače
neobsahuje a `@SYS/tasks.txt` se nečetlo. V JSON jsou null, nikoli vymyšlené nuly.
Baseline perioda 400Hz před RUN=1 nebyla samostatně instrumentována. Netvrdíme tedy
přesný rozdíl baseline/ON jitteru nebo absenci všech overruns mimo měřené okno.

## Stack a vlákna

Read-only `@SYS/threads.txt` byl přečten až po frozen exportu a obnově parametrů.
Vrací ThreadsV2; source `AP_HAL_ChibiOS/Util.cpp::thread_info()` používá stack
watermark. Nečte se tasks.txt a nemění se SCHED_OPTIONS. Thread snapshot může
podle buildu resetovat své statistiky, proto je záměrně mimo okno.

| Vlákno | Volný watermark [B] | Skutečný přidělený stack [B] |
| --- | ---: | ---: |
| ArduCopter | 4352 | 7168 |
| ISR | 1280 | 1536 |
| Historický utb_log | 2456 | 3504 |
| FTP | 1592 | 2992 |

Nový 400Hz SHADOW vlastní worker nemá. Vlákno `rate` není v seznamu. Watermark je
kumulativní od vzniku threadu a zahrnuje předchozí inicializaci/komunikaci/export;
nejde o okamžitou rezervu nebo přírůstek pouze SHADOW. Z hlavního stacku bylo
podle watermarku obsazeno nejvýše 2816 B do okamžiku čtení. Tato pozorovaná rezerva není
libovolně zvolený bezpečnostní limit ani důkaz rezervy za všech podmínek.

## Konečný stav a meze

- FC komunikuje, DISARMED, STABILIZE (mode 0), status STANDBY.
- AHRS/sensor/logger health platné, EKF_STATUS_REPORT.flags=167, InternalError=0,
  poslední logger drops=0. Žádný další restart v měření; poslední načtený
  STAT_BOOTCNT z post-upload zálohy je 97.
- UNS_RUN=0, UNS_COUNT=4000, LOG_DISARMED=0, SCHED_OPTIONS=0.
- Dočasné intervaly SYS_STATUS/SYSTEM_TIME/EKF_STATUS_REPORT obnoveny na -1.
- COM5 zavřený. Žádné mazání logů, změna PID, motorový příkaz, ARM nebo LiPo test.
- Zdrojový strom a původní práce zachované; index prázdný, HEAD beze změny.
- Nic se nedoinstalovávalo. Použity existující Windows Python, pymavlink/pyserial
  a matplotlib pro offline grafy; WSL Python pro offline analýzu/hash.

Z naměřených 1000 kroků nevyplývá konkrétní blokující překážka pro jeden následný
10s kontinuální DISARMED test při stejné konfiguraci a vypnutém loggeru.
**Doporučuji jej samostatně schválit**, teprve poté fyzický nový start a nový
baseline. Kontinuální režim může mít jinou režii a v DISARMED bude přebírat skutečné
externí AP resety integrátoru; nelze z tohoto one-step výsledku předjímat jeho PASS.

Neověřeno zůstává: kontinuální vlastní historie na H743, dlouhodobé/teplotní
chování, aktivní integrace v letu, agresivní vstupy a saturace, celý dynamický
rozsah konfigurací, globální WCET a původní logger anomálie. Ta není tímto RAM-only
měřením opravena ani vysvětlena. Při přípravě dalšího testu znovu neaktivovat logger
nebo během okna nepouštět FTP/diagnostické požadavky.

## Artefakty a reprodukovatelnost

Kořen výsledků: `tmp/utb-3d4-hw-20261022_202457`.

- `approval.json`, `upload-candidate.json`, `upload/result.json`: schválení,
  kandidát, identita a úspěšný standardní upload.
- `preflight.json`, `parameters-before-upload.param`: původní identita a záloha.
- `post-upload/preflight.json`: zachycení první ještě nehotové AHRS inicializace.
- `startup-observation/preflight.json`: následný zdravý start a nová záloha.
- `parameters-after-upload-comparison.json` a `...-evaluated.json`: přesné rozdíly
  i jejich source vyhodnocení; první striktní srovnání zastavilo offline kontrolu,
  automatické boot změny byly vyhodnoceny před měřením, nikoli ignorovány.
- `measurement/utb_rate_shadow.txt`, `shadow.json`, `result.json`, `telemetry.jsonl`,
  `threads-after.txt`: skutečná surová data a packet audit.
- `analysis/summary.json`, `cpu.csv`, `timing.csv`, `period.csv`, `differences.csv`,
  `stack.csv`: strojové výsledky, ne odhady nahrazující nedostupné veličiny.
- `analysis/*-h743.png` a `.svg`: grafy výhradně měřených/odvozených veličin.
- `measure-once.py`, `analyze.py`, `plots.py`: provedený protokol a offline výpočty.
  **Measure-once.py není automatické povolení dalšího experimentu; znovu nespouštět.**
- `final-source-integrity.json`, `git-status-final.txt`, `artifact-sha256.json`:
  závěrečná integrita a Git stav.

Numerický parser ověřil 54 řádků, kompletní počty a stop reason; součet period
souhlasí s last_us−first_us. Grafy byly vizuálně zkontrolované. Původní 3D-3 unit,
SITL a build výsledky se nepřeznačují jako nové HW testy; žádné nové sestavení
nebylo pro tuto čistě měřicí fázi potřeba.
