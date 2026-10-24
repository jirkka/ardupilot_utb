# FÁZE 3D-5 — kontinuální 400Hz HW ověření Rate/PID

Datum: 22. 10. 2026. Skutečná Skystars STM32H743, USB, DISARMED, bez LiPo a vrtulí.
Větev `test/skystars-5inch`, HEAD `070c189cdfa8357307ee1aebe8f3677cd849d520`.

## Závěr k pěti otázkám

1. **4000/4000 kontinuálních kroků je numericky shodných s AP** v provedeném
   podporovaném DISARMED profilu: všechny maximální absolutní odchylky a RMSE = 0.
   Úplná historie se synchronizovala jen jednou, poté se vlastní filtry vyvíjely
   kontinuálně. **Samostatná akumulace I bez externích resetů tímto ověřena není.**
2. **1 synchronizace, 11 997 zpracovaných externích I_set příznaků, 0 externích
   resetů filtrů.** I_set události jsou počítané na úrovni osa/krok, nikoli jako
   počet všech C++ volání v původním AP. Podrobné vysvětlení níže.
3. **Smyčka v měřeném okně stabilní, motorová izolace zachovaná:**
   400,000320 Hz, perioda 2451–2548 µs, motor_changes=0 ze 4000 auditů.
   Žádný invalid, DESYNC, UNSUPPORTED, mismatch nebo ztracený připravený krok.
4. **Consume je oproti 3D-4 pomalejší:** průměr 63,243 µs místo 55,943 µs,
   rozdíl +7,300 µs (+13,049 %). Math 8,69875 µs místo 8,560 µs.
   Jde o naměřené spany, nikoli izolovanou cenu celé přidané diagnostické cesty.
5. **Doporučuji uzavřít DISARMED HW část Rate/PID pro tento profil a navázat
   samostatnou implementační fází pasivního UTB_Attitude SHADOW.** Otevřené zůstává
   HW ověření I historie bez resetů, jiné konfigurace, dynamické/letové zatížení
   a úplný WCET. Tato fáze Attitude SHADOW neimplementuje a neschvaluje let.

Proveden přesně jeden schválený RUN=2/COUNT=4000 experiment, bez opakování.

## Firmware, nový start a preflight

Použit již nahraný 3D-3 SHADOW ON firmware, žádný build nebo upload:

- APJ: `tmp/utb-3d3-20261021_191141/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`.
- SHA-256: `aebc7cfa73773e552e790a289f7c83f16c8794bed71b56a6101fc4daf18775aa`.
- Velikost APJ 1 131 335 B; Copter 4.7.1 / `070c189c`.
- Board ID 1075, Windows COM5, USB VID:PID 1209:5741,
  serial `200024001251303433333336`. Stejná identita i v MAVLink uid2.
- Fyzické odpojení a připojení pouze USB potvrzeno obsluhou.
  STAT_BOOTCNT=98, předchozí 3D-4 boot měl 97. Žádný automatický reboot.
- 40s read-only inicializační/health pozorování a 30s kompletní záloha;
  celkem 71,203 s komunikace. Získáno všech 1211 parametrů. Nativní PID
  konfigurace proti záloze 3D-4 beze změny; přesné rozdíly v parameter-comparison.json.
  Automatické kalibrace/čítače po bootu se nevracely na staré hodnoty.
- Zdravé gyro/akcelerometr, AHRS health a EKF attitude; žádná zachycená
  InternalError nebo neočekávaný restart. FC trvale DISARMED.

Read-only ověřeno SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0, RLL/PIT/YAW SMAX=0 a
všechny NTF/NEF=0. UNS_RUN=0 a UNS_COUNT=4000. LOG_DISARMED=0,
UTB_SHADOW=0, SCHED_OPTIONS=0. Nepodporované AP nastavení se neupravovalo.

Před baseline přečten `@SYS/utb_rate_shadow.txt`: přesně
`UTBNativeShadowV1,pending` (26 B), tedy předchozí frozen session není přebírána.
`@SYS/threads.txt` před i po experimentu neobsahuje `rate` thread. Všech 4000
kroků prošlo navíc guardem, který odmítá i boot session, kde rate thread již
někdy běžel. `@SYS/tasks.txt` se nečetlo.

Běžící firmware se znovu nečetl z MCU flash: kryptografický SHA byl ověřen na
uloženém APJ. Identita nahraného firmware navazuje na CRC ověřený upload 3D-4,
stejnou desku, novou boot session, firmware verzi a dostupnou native SHADOW RAM
funkci. Nebyl proveden bootloader entry, upload nebo změna firmware.

## Jediný protokol a izolace okna

1. Po kontrole čerstvé RAM session a vláken následovala health kontrola.
2. Před baseline dotázány původní MAVLink intervaly: SYS_STATUS, SYSTEM_TIME a
   EKF_STATUS_REPORT byly vypnuté (-1). Dočasně nastaveno 2 Hz / 1 Hz / 1 Hz.
   Po 3s ustálení následovala **15,016s baseline UNS_RUN=0** se stejnou telemetrií
   jako během experimentu. Změna intervalů není změna AP parametrů nebo loggeru.
3. UNS_COUNT=4000 nastaven jednou před startem. Jediný RUN=2 trigger v host čase
   25,719 s od začátku protokolu. Následovalo 12s klidové okno bez odchozích
   požadavků; pouze příjem již nakonfigurované telemetrie.
4. **Jediný odchozí paket ve fázi measurement je počáteční PARAM_SET UNS_RUN=2.**
   Žádný další PARAM_SET, polling, FTP, log operace nebo diagnostický příkaz.
5. Po klidovém okně ověřeno automatické UNS_RUN=0 a DISARMED. Po 8s pozorování
   po testu stažen frozen snapshot (2132 B). Nikoli průběžné pollování statistik.
6. Obnoveno UNS_RUN=0 a UNS_COUNT=4000. Mimo okno přečten stack snapshot a znovu
   ověřeno zdraví. Původní intervaly telemetrie obnoveny na -1, COM5 zavřený.

Celý měřicí protokol včetně přípravy/exportu trval 54,156 s. Raw packet audit,
časování příkazů a health záznamy jsou v measurement/result.json a telemetry.jsonl.
MAVFTP varování „expected 100000“ odpovídá placeholder velikosti @SYS stat,
nikoli neúplnému přenosu; všechny přenosy skončily úspěšným EOF.

Firmware sám zastavuje diagnostiku při numerické neshodě, změně auditovaných
motorových hodnot nebo vlastním consume span ≥2500 µs. Externí health se během
okna sledovalo pasivně. Ostatní RAM příznaky lze bez pollování vyhodnotit až po
frozen exportu; v tomto běhu byly všechny neočekávané chyby nulové.

## Úplnost, historie a numerické výsledky

| Veličina | Výsledek |
| --- | ---: |
| Stop reason / continuous | 1 (COUNT dokončen) / 1 |
| prepared_attempts / seen / valid | 4000 / 4000 / 4000 |
| invalid / desync / unsupported / mismatch | 0 / 0 / 0 / 0 |
| unconsumed_attempts | 0 |
| first_seq / last_seq | 1 / 4000 |
| Synchronizace celé historie | 1 |
| Externí I_set replay události | 11997 |
| Externí reset_filter replay události | 0 |
| config_changes / mode_changes | 0 / 0 |
| native_motor_checks / skipped / changes | 4000 / 0 / 0 |
| Max. absolutní chyba / max. RMSE | 0 / 0 |

Všech 18 veličin na každé ze tří os má nulovou chybu ve všech validních krocích:
216 000 skalárních dvojic. Patří sem relevantní filtrované stavy, P/I/D/FF/D_FF,
výstupy a flags. Jde o skutečné živé vstupy původní AP smyčky, nikoli nový
syntetický benchmark. FC ležela v klidu, bez motorového zatížení a bez letového
buzení; nelze tvrdit pokrytí všech dynamických stavů nebo saturací.

### Co přesně znamená 11 997 I_set událostí

`UTB_RateShadow::process()` importuje celou prehistorii na začátku epochy,
proto synchronizations=1 pro všechny tři osy dohromady. V dalších krocích pouze
při skutečném AP pending I_set zavolá vlastní set_integrator a zvýší i_events;
obdobně zvlášť řeší reset_filter. Ostatní filtrové historie se z AP neobnovují.
Zdroj: `libraries/AP_UTB/AP_UTB_NativeShadow.cpp:174–197`.

V tomto běhu platí **11 997 = 3 × (4000 − 1)**. Tedy po počáteční synchronizaci
byl v každém dalším kroku zpracován I_set pro každou osu. Native STABILIZE v
SHUT_DOWN provádí reset_rate_controller_I_terms (`ArduCopter/mode_stabilize.cpp:28–33`),
který resetuje všechny tři nativní rate PID
(`AC_AttitudeControl.cpp:273`, `AC_PID.cpp:384`). To odpovídá trvalému DISARMED stavu.

Čítač měří replay příznaků na úrovni osa/krok. Nerozlišuje reset_I,
set_integrator nebo relax_integrator a nepočítá odděleně více externích volání,
pokud se před jedním PID krokem sloučila do téhož pending flagu. První import
historie není započten jako další tři replay události.

**Nejde o 4000 úplných importů historie:** kontinuální filtry a ostatní
relevantní stavy jsou vlastní. **Integrátor se ale kvůli skutečným AP událostem
v tomto profilu přenastavoval každé kolo.** Test proto potvrzuje kontinuální
filtrační historii a správné zpracování nativních resetů, nikoli nenulovou I
akumulaci bez resetů. Bez oslabení původního AP arming/spool chování tento
DISARMED experiment takovou akumulaci nevyvolával.

## Časy na fyzické H743 a srovnání 3D-4

AP_HAL::micros() v ChibiOS používá hardware hrt_micros32. Spany jsou hardware
wall time včetně případné preempce, nikoli virtuální SITL čas nebo Python čas.

| Rozsah | Součet 3D-5 [µs] | 3D-5 průměr [µs] | 3D-5 max [µs] | 3D-4 průměr / max [µs] |
| --- | ---: | ---: | ---: | ---: |
| UTB math | 34795 | 8,69875 | 16 | 8,560 / 15 |
| Engine consume span, včetně math | 252972 | 63,24300 | 79 | 55,943 / 69 |
| Původní AP + capture společný span | 70843 | 17,71075 | 24 | 17,538 / 25 |

![Časové rozsahy](../tmp/utb-3d5-hw-20261022_210319/analysis/timing-h743.png)

Consume průměr vzrostl o **7,300 µs (+13,049 %)**, maximum o 10 µs.
Math průměr +0,13875 µs, AP+capture +0,17275 µs. Kontinuální cesta provádí také
průběžné kontroly konfigurace/historie a v tomto DISARMED profilu I_set replay.
Naměřená změna je s touto odlišnou cestou konzistentní, ale dva samostatné boot
běhy s rozdílnými senzorovými historiemi nejsou kauzálně izolovaný benchmark.

Měřený consume span začíná konstrukcí ShadowTiming v engine process a končí
čtením hodin v jeho destruktoru. Následná aktualizace timing čítačů, část wrapperu,
prepare a motor audit jsou mimo tento span. Math je uvnitř consume, proto se
nesčítají. AP+capture obsahuje původní AP výpočet a není čistou capture režií.

Consume odpovídá v průměru 2,530 % nominální 2500µs periody, maximum 3,160 %.
Tyto poměry wall span nejsou celková CPU režie všech hooků ani důkaz kompletního
WCET hlavní smyčky. Čistá celková capture/prepare/audit režie, P95/P99 a jednotlivé
časy nejsou tímto firmware exportem dostupné. Zůstávají neověřené.

![Srovnání obou HW běhů](../tmp/utb-3d5-hw-20261022_210319/analysis/comparison-h743.png)

### Perioda a jitter

| Veličina | 3D-5 | 3D-4 |
| --- | ---: | ---: |
| Vzorků / intervalů | 4000 / 3999 | 1000 / 999 |
| Průměr periody [µs] | 2499,997999 | 2499,992993 |
| Min / max periody [µs] | 2451 / 2548 | 2481 / 2517 |
| Průměrný absolutní jitter [µs] | 3,240810 | 2,923924 |
| Frekvence [Hz] | 400,000320 | 400,001121 |

Native first_us=349937632, last_us=359935124. Součet 3999 intervalů
9 997 492 µs souhlasí přesně s last_us−first_us. Tisícům kroků náleží N−1
vzájemných intervalů, nikoli N. Největší absolutní odchylka periody od 2500 µs
je 49 µs. Poslední native dt=0,0024999999 s, IMU frontend timestamp=359935122 µs,
čas čtení gyra=359935127 µs, primární gyro=0.

![Agregovaná perioda](../tmp/utb-3d5-hw-20261022_210319/analysis/period-h743.png)

Průměrný |jitter| se zvýšil o 0,316886 µs. Tento běh byl čtyřikrát delší;
širší min/max nelze automaticky připsat kontinuálnímu algoritmu. Není zde mezera
v sekvenci nebo perioda typu původních 5,5–6,5 ms. Graf zobrazuje skutečné
agregované min/průměr/max, nikoli histogram nebo vymyšlené percentily.
Perioda rate-entry není výpočetní doba všech úloh a baseline per-loop jitter
před RUN=2 nebyl samostatně instrumentován.

## Scheduler load, zdraví a nedostupné PM veličiny

Stejná telemetrie SYS_STATUS 2 Hz během baseline i testu. Load/10 pochází z
AP_Scheduler::load_average, není to samostatný profil CPU všech vláken/ISR.

| Fáze | Vzorků | Průměr [%] | Min / max [%] |
| --- | ---: | ---: | ---: |
| RUN=0 baseline | 30 | 17,983333 | 17,6 / 18,6 |
| RUN=2 okno | 20 | 21,145000 | 19,6 / 21,7 |
| Po okně, před exportem | 16 | 18,006250 | 17,6 / 18,4 |

Pozorovaný přírůstek proti vlastní baseline je **+3,161667 procentního bodu**.
3D-4 měl +2,773333 p. b., rozdíl přírůstků +0,388333 p. b. Absolutní baseline
obou bootů se liší (18,406667 % v 3D-4 versus 17,983333 % zde). Z rozdílu proto
nelze bez dalšího oddělit vliv algoritmu, přerušení a ostatních úloh.
On-window zatížení je odvozené z 20 telemetrických vzorků a vyhlazování scheduleru;
časové zarovnání grafu přes SYSTEM_TIME/host příjem není per-cycle CPU profil.

![Zatížení scheduleru](../tmp/utb-3d5-hw-20261022_210319/analysis/cpu-h743.png)

Ve všech zachycených SYS_STATUS jsou InternalError bitmask/count a logger dropped
counter 0; bez restartu v měřicím okně, senzorové health flags a EKF attitude platné.
4000 úplných RAM kroků dokládá úplnost sady; SHADOW neposílá DataFlash záznamy a
nemá výstupní log queue. Žádná data nebyla zahazována kvůli numerické neshodě.

**PM.NLon, PM.MaxT a task-budget overrun čítač nejsou ověřené.** LOG_DISARMED=0 se
nezměnil a RAM export je neposkytuje. Není bezpečný důvod kvůli jejich získání
zapínat logger, spouštět tasks.txt nebo měnit firmware uprostřed této session.
V JSON jsou null, nikoli nuly. Nelze z této sady tvrdit globální absenci deadline
missů nebo časových anomálií po celý boot.

## Stack a konečný stav FC

ThreadsV2 před i po testu, vždy mimo měřené okno:

| Vlákno | Volný watermark před / po [B] | Skutečný stack [B] |
| --- | ---: | ---: |
| ArduCopter | 4352 / 4352 | 7168 |
| ISR | 1288 / 1288 | 1536 |
| Historický utb_log | 2472 / 2472 | 3504 |
| FTP | 1736 / 1592 | 2992 |

Rate thread před ani po experimentu není přítomen. Nový native SHADOW vlastní
worker nemá. Hlavní watermark odpovídá nejvýše 2816 B obsazeného stacku od vzniku
threadu; není to okamžitá rezerva, přírůstek pouze SHADOW nebo univerzálně
bezpečný limit. Dřívější startup maximum může zastínit menší pozdější spotřebu.
Stejných 4352 B zůstalo volných i v 3D-4. FTP watermark se po okně a exportu
zmenšil o 144 B; tento interval zahrnuje formátování/FTP mimo měření, proto nejde
o údaj o spotřebě stacku samotného 400Hz enginu.

Konečný stav:

- DISARMED, STABILIZE (mode 0), STANDBY; žádný ARM/motorový příkaz.
- AHRS/senzory/logger health v pořádku, EKF flags=167, InternalError=0,
  logger drops=0. Poslední SYSTEM_TIME uptime 377605 ms; načtený boot counter 98.
- UNS_RUN=0, UNS_COUNT=4000, LOG_DISARMED=0, SCHED_OPTIONS=0.
- Tři dočasné intervaly telemetrie obnoveny na původní -1; COM5 zavřený.
- Původní ArduPilot zůstal jediným vlastníkem motorové cesty. Audit porovnal
  4000 sad šesti nativních R/P/Y + FF hodnot, motor_changes=0. Nejde o měření
  fyzického tahu nebo DShot signálů na nenapájených ESC.
- Žádný build/upload, automatický reboot, log listing/download/erase, změna PID,
  loggeru, Attitude SHADOW nebo instalace. Žádný commit, reset či push.
- Původní zdroje, rozpracované soubory a výsledky 3D-4 zachované; novým souborem
  v repozitáři je pouze tento report. Artefakty v ignorovaném tmp adresáři.

## Uzavření a další fáze

DISARMED HW důkaz lze uzavřít v přesném rozsahu: podporovaný nativní 400Hz tok,
one-step shoda 3D-4, kontinuální filtrační historie a správný replay externích
událostí 3D-5, motorová izolace a pozorované krátkodobé časování.

**Neuzavírá se** I paměť bez externích resetů na živé H743, aktivní letová smyčka,
široké reference/saturace, nepodporované notch/SMAX/rate-thread profily, dlouhodobé
teplotní zatížení nebo úplná režie/WCET. Původní logger anomálie z 3D-2 zůstává
neobjasněná; RAM-only experiment ji neopravuje ani nevyvrací.

Tyto hranice nebrání navazující **samostatně zadané implementaci pasivního
UTB_Attitude SHADOW**, při zachování AP motorové autority a nejprve unit/SITL
ověření capture, shaping a resetů attitude historie. V této fázi se další kód
neimplementoval, neprovedl další HW experiment a nepřipravoval aktivní let.

## Artefakty

Kořen: `tmp/utb-3d5-hw-20261022_210319`.

- physical-start.json: potvrzený fyzický restart a rozsah jediného testu.
- preflight.json, preflight-messages.jsonl, parameters-before.param:
  identita, health a kompletní záloha. Textové pole operation zůstalo zděděné
  ze šablony s označením 20s health; skutečně vykonaný skript a časová stopa mají
  40 s pozorování +30 s zálohy. Tento popisný překlep nemění uložené časy/data.
- parameter-comparison.json: srovnání s předchozí konfigurací, bez ATC změn.
- measurement/shadow-before.txt, threads-before.txt: nová pending session a vlákna.
- measurement/utb_rate_shadow.txt, shadow.json, result.json, telemetry.jsonl,
  threads-after.txt: surové HW výsledky, úplnost a packet audit.
- analysis/summary.json, cpu.csv, timing.csv, period.csv, differences.csv, stack.csv:
  strojově čitelné vyhodnocení; nedostupné veličiny explicitně null.
- analysis/comparison-3d4-3d5.json/.csv: přesná srovnání obou HW měření.
- analysis/*-h743.png/.svg: grafy aktuálního běhu a srovnání; vizuálně ověřené.
- measure-once.py, analyze.py, plots.py, compare.py: provedený protokol a offline
  analýza. Measure-once.py se bez nového zadání znovu nespouští.
- final-integrity.json, git-status-final.txt, artifact-sha256.json: závěrečná
  kontrola zachování zdrojů, předchozích výsledků a aktuální Git stav.

Zdrojový HEAD zůstává stejný a index prázdný. Nedošlo k přeznačení SITL výsledků
za HW měření ani k odhadům nahrazujícím chybějící měřené veličiny.
