# FÁZE 3G-2 – Pasivní Flight SHADOW pro H743

Revize 2026-10-25. Výchozí HEAD `d78e34b3ac7bf499236ba2ef452bfd4562b3e65d`, větev `test/skystars-5inch`.

## Výsledek a hranice platnosti

Implementována a softwarově ověřena je **omezená RAM varianta** pasivního native STABILIZE → UTB Attitude → UTB Rate/PID porovnání. Nový persistentní zápis do DataFlash implementován není. Není schválen fyzický let, upload ani USB-only experiment. V této fázi nebylo žádné spojení s COM5, fyzické armování, upload, mazání logů, instalace nebo push.

- **Implementováno:** default-OFF Flight session, pevný recorder, nezávislé historie existujícího Chain enginu, úplné statistiky a frozen export.
- **Unit/SITL ověřeno:** dynamický ARMED native STABILIZE, nenulové reference/gyro/I/shaping/FF, přechody a native failsafe, bounded storage a negativní analýza.
- **Build ověřeno:** SITL/H743 Flight ON/OFF, H743 aktivní UTB compile-out.
- **Hardware neověřeno:** nový recorder, skutečný WCET/CPU, heap/stack watermark a propustnost exportu na H743.
- **Blocker letu:** trvalý záznam, aktuální volná flash, kvalifikace startup/průběžného loggeru a reálné časové/paměťové rezervy. RAM fallback tyto body neřeší.

Artefakty: `tmp/utb-3g2-20261025_071149/`. Původní stav a kapitola jsou v `before.json` a `before-untracked.tar.gz`; kapitola zůstává mimo commit a má nezměněný SHA-256 `3bcacd44527c67c44ea88b2cdfd2eb7edfd02705d88787c27ead2e74c45c293b`.

## Audit původní architektury a loggeru

Existující `AP_UTB_ChainShadow` již poskytuje skutečné párování Attitude(N) → Rate(N+1), 26 Attitude + 4 upstream + 3×18 PID metrik, vlastní kontinuální historie, replay reset/I_set/filter událostí a motorový audit. Nevznikl nový PID, Attitude algoritmus, matematický engine, scheduler task ani logger worker. Změny `UTB_PID`, `UTB_Rate`, `UTB_Attitude`, AP_Motors, HAL/DShot, AP_Logger, AP_Scheduler, arming a failsafe nejsou součástí této fáze; audit hashů je v `protected-source-audit.json`.

Deska SkystarsH7HD-bdshot dědí SPI JEDEC DataFlash z `libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD/hwdef.dat`: `HAL_LOGGING_DATAFLASH_ENABLED=1`, zařízení dataflash na SPI3. Relevantní je BLOCK backend a `AP_Logger_Block`, nikoliv předpoklad SD souborového backendu. Aktuální fyzickou volnou kapacitu nelze bez připojení desky zjistit. Historická čísla nejsou aktuální kapacita.

Prodlevy 5503/6480 µs při prvním otevření loggeru zůstávají neuzavřené. Sonda 3D2 se zachytila před plánovanou aktivací loggeru, proto ji nelze vydávat za důkaz příčiny původní závady. Tato fáze logger neopravuje, neotvírá nový HW log a nemaže existující data. Kontinuální zápis, mazání sektorů a obnovení logování na BLOCK backendu nebyly nově kvalifikovány.

## Datová cesta a motorová izolace

Aktivní větev zůstává RC → native STABILIZE → native Attitude → native Rate/PID → AP_Motors. Stávající capture body pořizují pouze hodnotové kopie skutečně použitých vstupů, konfigurace, předhistorie, posthistorie a výstupů. `ArduCopter/Attitude.cpp` doplňuje pouze ARM/spool/context metadata před původním Chain prepare.

Pasivní větev používá stejné gyro, AHRS, dt, scaling, limit flags a reset události. Vlastní UTB Attitude vytvoří vlastní body target; ten se po správném posunu N → N+1 předá vlastnímu Rate/PID. Native target je oracle, nikoliv skrytý zdroj hotové UTB regulace. Raw RC PWM se v RAM neduplikuje: `Cmd0..2` je skutečný příkaz po původním AP RC mapování; případné raw RCIN zůstává v native BIN logu.

Plná historie se importuje jednou při první synchronizaci. RESET/RESET_YAW/RELAX/INERTIAL a externí I_set/filter reset se přehrávají podle existujícího enginu. ARM/DISARM mění jen označení epochy, nikoliv automatický import nebo reset vlastní historie. Výsledky níže oddělují synchronizace a skutečné resetové operace.

Recorder nemá AP_Motors, živý controller nebo output sink; přijímá pouze const engine a kopíruje hodnoty do RAM. Nemá motorový setter, DataFlash/MAVLink zápis, heap alokaci, mutex, čekání nebo textové formátování ve 400 Hz record cestě. Původní motorové hodnoty se kontrolují kolem zachycených Rate/Attitude operací. `motor_changes=0` je důkaz těchto auditovaných míst a scénářů, nikoliv obecná letová certifikace.

Nový Flight kontext dovoluje native mode0, main thread, 400 Hz, bez fast-rate/rate-thread. Navíc odmítá `ATT_ENABLE!=0` a aktivní AC_CustomControl; `is_safe_to_run()` v `AC_CustomControl.cpp` pouze čte jeho stav. Konfigurace AP se kvůli shodě automaticky nemění. Nadále platí původní podporovaný profil filtrů/SMAX/notch; není rozšířen odstraňováním guardů.

## Přepínače, ukončení a souběh

`AP_UTB_FLIGHT_SHADOW_ENABLED=0` default; waf volba `UTB_FLIGHT_SHADOW` závisí na UTB a existujícím Chain/Native/Attitude SHADOW. Runtime parametry jsou podskupina na dosud volném indexu2 UCS_; indexy původních UCS_RUN=0 a UCS_COUNT=1 zůstávají.

| Parametr | Default | Rozsah / význam |
|---|---:|---|
| UCS_F_RUN | 0 | 0 vypnuto, 1 jedna kontinuální RAM session |
| UCS_F_COUNT | 24000 | 2..48000 main pokusů, nominálně max.120 s |
| UCS_F_DECIM | 200 | 1..400; při400 Hz default2 Hz průběhy |

Při bootu se RUN vynuluje. Legacy UCS_RUN a Flight sdílejí jediný engine a jedinou finite session za boot. Konflikt požadavků ukončí pouze diagnostiku. Odchod ze STABILIZE zastaví session; návrat ji nespouští znovu. Důvody: 1 dosažen COUNT, 2 ruční stop, 3 numerická chyba, 4 neplatný/konfliktní požadavek, 6 unsupported, 7 mode exit.

Všechny podporované numerické kroky se počítají bez ohledu na decimaci. První krok je explicitní Rate warmup, nikoliv ztracený validní krok. Vadný/unsupported koncový vzorek se vynutí mimo decimaci; přesný poslední Attitude input/context je v metadatech. Failing Attitude(N) není zaměňován za Rate(N), který spotřebovává source(N−1).

Publikace používá stávající lock-free release/acquire. Reader nečte běžící recorder. Po freeze se buffer, capture historie i čítače nemění; pozdější RUN se jen vynuluje. ARM/spool aktualizace se v uzavřeném recorderu ignorují. Export je povolen až frozen+DISARMED, není použit reader/writer mutex. Parametry používají původní AP_Param infrastrukturu.

## RAM formát, ztráty a datový tok

`Record` má přesně304 B, 128 pevných řádků =38 912 B. Časování/policy metadata jsou zvlášť. Ring přepisuje nejstarší diagnostický řádek a každé přepsání počítá; originální letové logy se nepřepisují. Identifikátory, source loop, ARM/mode/spool/epoch, reset/limit flags a skutečné dt dovolují zkontrolovat návaznost. Uloženy jsou source AHRS a native/own target quaterniony, command, gyro, obě body-rate reference, P/I/D/FF/D_FF a control/feedforward výstupy R/P/Y. Aktuální Euler orientaci lze odvodit z uloženého AHRS quaternionu; native ANG je navíc dostupné v běžném SITL BIN.

| Frekvence průběhů | Hodnotová data/s | 30 s | 60 s | 120 s |
|---:|---:|---:|---:|---:|
|400 Hz|121600B|3648000B|7296000B|14592000B|
|20 Hz|6080B|182400B|364800B|729600B|
|2 Hz|608B|18240B|36480B|72960B|
|1 Hz|304B|9120B|18240B|36480B|

To jsou logické hodnotové objemy, nikoliv fyzický zápis. Při2 Hz uchová ring60 s/120 řádků, při120 s/240 řádcích přepíše112. Při1 Hz uchová120 s/120 řádků. Při400 Hz uchová jen posledních0,32s. Numerické agregace84 metrik přitom pokrývají celé podporované okno. Z decimace nelze rekonstruovat400 Hz D/FF dynamiku ani špičky mezi uloženými body.

Nový zápis do DataFlash = **0 B/s**. Žádná nová log IDs nebo logger metadata. Legacy AP logování se nemění. Formátování a MAVFTP se provádí teprve po okně a DISARMED:

- `@SYS/utb_flight.txt`: důvod, počty, timing, všech84 metrik a poslední fault context;
- `@SYS/utb_flight0.csv` až `utb_flight7.csv`: osm pevných stránek po16 řádcích.

Stránkování omezuje jednotlivou alokaci ExpandingString, ale nepředstavuje změřenou H743 heap rezervu. Maximální skutečná stránka a exportní objemy jsou uvedeny v artefaktech. Současné otevření více souborů může spotřebovat více heapu; budoucí postup musí exportovat sekvenčně. Za letu se export nepovoluje.

Textový export zachovává float32 hodnoty jako8 hex číslic a chyby/RMSE float64 jako16 hex číslic. HAL printf jinak převádí double na float a není vhodný pro bezeztrátové malé odchylky. Host analyzátor dekóduje přesné bity do běžných CSV/JSON. Nevytváří nové400 Hz vzorky a nevyřazuje vadné řádky. Kontroluje všechny84 identity, úplnost stránek, sekvence, pairing, session/boot identitu, finite hodnoty, ztráty, počty i motor audit. Nulové metriky při invalid/desync nejsou PASS.

## Zachycená a opravená regrese exportu

První unsupported SITL běh vyvolal SIGFPE při MAVFTP exportu prázdné odmítnuté session. Lokální signal/backtrace sonda bez změny firmware zdrojů zachytila `dump_flight → dump → ExpandingString::printf → print_vprintf`. Instrukce `comiss` na adrese offset0xe89ca porovnávala NaN ve větvi `fabsf(value)>9999999` v `libraries/AP_HAL/utility/print_vprintf.cpp:193`. Zdroj NaN byl RMSE bez jediného validního vzorku v původním Chain dumpu, nikoliv regulační matematika.

Minimální oprava zůstává v Chain exportu: při nulovém počtu se přímo vypíše text `nan`; nevolá se float formatter s NaN. HAL, PID a guardy nebyly změněny. Nový unit test kontroluje FE_INVALID/FE_DIVBYZERO a význam nedefinovaného RMSE bez předpokladu konkrétního NaN payloadu. Opakovaný unsupported test na finálním sestavení skončil reason6, attempts0, unsupported1, bez pádu a s pokračujícím native AP. Původní chybné běhy `flight-unsupported/` a `debug-unsupported/` zůstávají jako důkaz; nejsou PASS. Také první vývojové exporty s chybným desetinným formátem zůstávají označené jako vývojové chyby.

## Numerika, dynamické SITL a resetové stavy

Následující tabulka používá finální sestavení `SITL_FINAL`, všechny podporované kroky, nikoliv pouze uchované raw řádky. V každém uvedeném běhu je plná Attitude synchronizace1 a PID synchronizace1, warmup1, všech84 max/RMSE0, mismatch/desync/invalid0 a motor_changes0. Attitude komparátor nadále používá původní toleranci1e−6·max(1,|oracle|), upstream/PID numerickou rovnost; tolerance se nezměnily.

| Scénář | Main | Paired Rate | Valid Attitude | Resety | Vlastní I kroky | Raw / přepsáno | Stop |
|---|---:|---:|---:|---:|---:|---:|---:|
| 60 s Angle let | 24000 | 23999 | 25479 | 1479 | 22517 | 120 / 0 | 1 |
| disarmed | 4000 | 3999 | 8000 | 4000 | 0 | 100 / 0 | 1 |
| early | 2000 | 1999 | 3479 | 1479 | 516 | 10 / 0 | 1 |
| ff | 12000 | 11999 | 13487 | 1487 | 10509 | 120 / 0 | 1 |
| gcs | 5556 | 5555 | 7029 | 1473 | 4079 | 28 / 0 | 7 |
| mode | 5189 | 5188 | 6663 | 1474 | 3711 | 26 / 0 | 7 |
| no-logger | 24000 | 23999 | 25474 | 1474 | 22522 | 120 / 0 | 1 |
| overflow | 8000 | 7999 | 9484 | 1484 | 6512 | 128 / 7872 | 1 |
| pressure | 12000 | 11999 | 13555 | 1555 | 10441 | 60 / 0 | 1 |
| rc | 3836 | 3835 | 5319 | 1483 | 2349 | 20 / 0 | 7 |

Všech84 metrik včetně názvů, max. absolutní chyby, RMSE a počtu vzorků: `tmp/utb-3g2-20261025_071149/metric-catalogue.csv`. Toto je úplný výsledek hlavního60 s běhu:

| Sekce | Osa | ID | Veličina | Max. chyba | RMSE | Počet |
|---|---:|---:|---|---:|---:|---:|
| att | 0 | 0 | target.q1 | 0.0 | 0.0 | 25479 |
| att | 0 | 1 | target.q2 | 0.0 | 0.0 | 25479 |
| att | 0 | 2 | target.q3 | 0.0 | 0.0 | 25479 |
| att | 0 | 3 | target.q4 | 0.0 | 0.0 | 25479 |
| att | 0 | 4 | error.q1 | 0.0 | 0.0 | 25479 |
| att | 0 | 5 | error.q2 | 0.0 | 0.0 | 25479 |
| att | 0 | 6 | error.q3 | 0.0 | 0.0 | 25479 |
| att | 0 | 7 | error.q4 | 0.0 | 0.0 | 25479 |
| att | 0 | 8 | euler.x | 0.0 | 0.0 | 25479 |
| att | 0 | 9 | euler.y | 0.0 | 0.0 | 25479 |
| att | 0 | 10 | euler.z | 0.0 | 0.0 | 25479 |
| att | 0 | 11 | euler_rate.x | 0.0 | 0.0 | 25479 |
| att | 0 | 12 | euler_rate.y | 0.0 | 0.0 | 25479 |
| att | 0 | 13 | euler_rate.z | 0.0 | 0.0 | 25479 |
| att | 0 | 14 | velocity.x | 0.0 | 0.0 | 25479 |
| att | 0 | 15 | velocity.y | 0.0 | 0.0 | 25479 |
| att | 0 | 16 | velocity.z | 0.0 | 0.0 | 25479 |
| att | 0 | 17 | acceleration.x | 0.0 | 0.0 | 25479 |
| att | 0 | 18 | acceleration.y | 0.0 | 0.0 | 25479 |
| att | 0 | 19 | acceleration.z | 0.0 | 0.0 | 25479 |
| att | 0 | 20 | body.x | 0.0 | 0.0 | 25479 |
| att | 0 | 21 | body.y | 0.0 | 0.0 | 25479 |
| att | 0 | 22 | body.z | 0.0 | 0.0 | 25479 |
| att | 0 | 23 | thrust | 0.0 | 0.0 | 25479 |
| att | 0 | 24 | thrust_error | 0.0 | 0.0 | 25479 |
| att | 0 | 25 | feedforward | 0.0 | 0.0 | 25479 |
| target | 0 | 0 | body.x+sysid.x | 0.0 | 0.0 | 23999 |
| target | 0 | 1 | body.y+sysid.y | 0.0 | 0.0 | 23999 |
| target | 0 | 2 | body.z+sysid.z | 0.0 | 0.0 | 23999 |
| target | 0 | 3 | yaw_ff_scale | 0.0 | 0.0 | 23999 |
| pid | 0 | 0 | target | 0.0 | 0.0 | 23999 |
| pid | 0 | 1 | error | 0.0 | 0.0 | 23999 |
| pid | 0 | 2 | derivative | 0.0 | 0.0 | 23999 |
| pid | 0 | 3 | target_derivative | 0.0 | 0.0 | 23999 |
| pid | 0 | 4 | integrator | 0.0 | 0.0 | 23999 |
| pid | 0 | 5 | P | 0.0 | 0.0 | 23999 |
| pid | 0 | 6 | I | 0.0 | 0.0 | 23999 |
| pid | 0 | 7 | D | 0.0 | 0.0 | 23999 |
| pid | 0 | 8 | FF | 0.0 | 0.0 | 23999 |
| pid | 0 | 9 | D_FF | 0.0 | 0.0 | 23999 |
| pid | 0 | 10 | Dmod | 0.0 | 0.0 | 23999 |
| pid | 0 | 11 | control_output | 0.0 | 0.0 | 23999 |
| pid | 0 | 12 | FF_output | 0.0 | 0.0 | 23999 |
| pid | 0 | 13 | limit | 0.0 | 0.0 | 23999 |
| pid | 0 | 14 | PD_limit | 0.0 | 0.0 | 23999 |
| pid | 0 | 15 | reset | 0.0 | 0.0 | 23999 |
| pid | 0 | 16 | I_term_set | 0.0 | 0.0 | 23999 |
| pid | 0 | 17 | actual | 0.0 | 0.0 | 23999 |
| pid | 1 | 0 | target | 0.0 | 0.0 | 23999 |
| pid | 1 | 1 | error | 0.0 | 0.0 | 23999 |
| pid | 1 | 2 | derivative | 0.0 | 0.0 | 23999 |
| pid | 1 | 3 | target_derivative | 0.0 | 0.0 | 23999 |
| pid | 1 | 4 | integrator | 0.0 | 0.0 | 23999 |
| pid | 1 | 5 | P | 0.0 | 0.0 | 23999 |
| pid | 1 | 6 | I | 0.0 | 0.0 | 23999 |
| pid | 1 | 7 | D | 0.0 | 0.0 | 23999 |
| pid | 1 | 8 | FF | 0.0 | 0.0 | 23999 |
| pid | 1 | 9 | D_FF | 0.0 | 0.0 | 23999 |
| pid | 1 | 10 | Dmod | 0.0 | 0.0 | 23999 |
| pid | 1 | 11 | control_output | 0.0 | 0.0 | 23999 |
| pid | 1 | 12 | FF_output | 0.0 | 0.0 | 23999 |
| pid | 1 | 13 | limit | 0.0 | 0.0 | 23999 |
| pid | 1 | 14 | PD_limit | 0.0 | 0.0 | 23999 |
| pid | 1 | 15 | reset | 0.0 | 0.0 | 23999 |
| pid | 1 | 16 | I_term_set | 0.0 | 0.0 | 23999 |
| pid | 1 | 17 | actual | 0.0 | 0.0 | 23999 |
| pid | 2 | 0 | target | 0.0 | 0.0 | 23999 |
| pid | 2 | 1 | error | 0.0 | 0.0 | 23999 |
| pid | 2 | 2 | derivative | 0.0 | 0.0 | 23999 |
| pid | 2 | 3 | target_derivative | 0.0 | 0.0 | 23999 |
| pid | 2 | 4 | integrator | 0.0 | 0.0 | 23999 |
| pid | 2 | 5 | P | 0.0 | 0.0 | 23999 |
| pid | 2 | 6 | I | 0.0 | 0.0 | 23999 |
| pid | 2 | 7 | D | 0.0 | 0.0 | 23999 |
| pid | 2 | 8 | FF | 0.0 | 0.0 | 23999 |
| pid | 2 | 9 | D_FF | 0.0 | 0.0 | 23999 |
| pid | 2 | 10 | Dmod | 0.0 | 0.0 | 23999 |
| pid | 2 | 11 | control_output | 0.0 | 0.0 | 23999 |
| pid | 2 | 12 | FF_output | 0.0 | 0.0 | 23999 |
| pid | 2 | 13 | limit | 0.0 | 0.0 | 23999 |
| pid | 2 | 14 | PD_limit | 0.0 | 0.0 | 23999 |
| pid | 2 | 15 | reset | 0.0 | 0.0 | 23999 |
| pid | 2 | 16 | I_term_set | 0.0 | 0.0 | 23999 |
| pid | 2 | 17 | actual | 0.0 | 0.0 | 23999 |


`att_valid` může přesáhnout počet main kroků: reset a primary STABILIZE jsou samostatně porovnané operace. DISARMED I_set/resety nejsou důkazem ARMED integrace. Nenulovou vlastní ARMED I historii, shaping, FF a D_FF dokládá zejména finální FF běh: 10509 nezávislých I kroků, 5974 shaping operací; maximum uchovaného vlastního |I|=0.00311197108, |FF|=0.0127093559, |D_FF|=0.0103443759, |gyro|=0.681883097 rad/s. Plné synchronizace1+1, replay Attitude resetů1487, externích I_set událostí4455. Obě znaménka, combined příkazy, yaw a skutečná gyro odezva jsou v raw datech a native ANG/RATE/RCIN logu. Ve112 uložených ARMED/THROTTLE_UNLIMITED řádcích hlavního letu jsou Rate reset/I_set flags0 a nenulová vlastní I; plná synchronizace zůstává1+1.

Mode test provádí STABILIZE→ACRO→STABILIZE; RC/GCS testy vyvolávají skutečný native failsafe s jeho MODE reason a standardním přistáním. Diagnostika zastaví reason7, native AP dál řídí a návrat do STABILIZE ji neobnoví. Early test dokončí2000 kroků ještě před zbytkem letu, který normálně pokračuje.

Unsupported (FSTRATE_ENABLE=1) a souběžný legacy/Flight požadavek jsou explicitní odmítnutí6/4, nikoliv nulová numerická chyba. Přerušený proces vytváří pending/neúplnou session, analyzátor ji odmítne. Unit fault injection mění výhradně vlastní diagnostický target nebo oracle projekci; zachytí upstream/downstream chybu a vynucený failure řádek. Chybějící/duplicitní řádek, špatný source, mixed session, invalid statistika a drobná nenulová chyba nejsou skrytě vyřazené.

Test bez loggeru (`LOG_BACKEND_TYPE=0`) dokončil24000 kroků s nulovými numerickými odchylkami, při native letu a standardním LAND/DISARM. Zátěžový pokus s1KB file bufferem/speedup4 dokončil12000 RAM kroků, ale v BIN chybí DSF. `native_logging_pressure_observed=false`: **není tím prokázána velikost drops ani kvalifikace zpožděného fyzického loggeru**. Jde o ověření pokračující RAM numeriky v této fixture, nikoliv PASS robustnosti BLOCK backendu.

## Porovnání A/B/C při stejném scénáři

| Profil | Host process CPU [s] | Host okno [s] | Virtuální okno [s] | RSS [B] | PM.MaxT [virtuální µs] | Max.PM.NLon/interval | DSF drops |
|---|---:|---:|---:|---:|---:|---:|---:|
| native | 3.82 | 30.764456 | 61.8 | 10174464 | 3332 | 5 | 0 |
| chain | 3.79 | 30.821421 | 61.8 | 10145792 | 3332 | 5 | 0 |
| flight | 3.83 | 30.822329 | 61.8 | 10055680 | 3332 | 5 | 0 |

Časy CPU pocházejí z100 Hz `/proc/<vlastní_pid>/stat`, host okno z monotónních hodin; zahrnují celý SITL proces včetně fyziky a native loggeru. Rozdíl Flight−native0,01 CPU s je uvnitř alespoň0,02s kvantizace rozdílu dvou intervalů, navíc existuje OS šum. Z jediného A/B/C nelze vyčíslit izolovaný WCET nebo H743 režii; záporný rozdíl Chain−native není zrychlení algoritmu. RSS není linker ani H743 volná RAM.

PM.Load v těchto SITL datech je0, proto není důkaz skutečného CPU zatížení. PM.MaxT3332 µs a NLon5/interval nastaly také v native bez diagnostiky; nelze je připsat UTB. Zachycená virtuální Rate perioda Flight je2499..3332 µs, průměr2500,007 µs, průměrný absolutní jitter2,004 µs. Nejde o změřené H743 deadline. Matematické/record časové spany ze SITL vracejí0 a nejsou interpretované jako nulová HW cena. Počty blokování původního loggeru nejsou z této fixture měřitelné.

A/B/C raw BIN, RC marks, MODE/ARM/ANG/RATE/RCOU/PID/PM/DSF CSV a výstupy jsou v `comparison-native`, `comparison-chain`, `comparison-flight`. Všechna přistání/disarm proběhla standardní cestou. OFF absence test ověřil nepřítomný UCS_F_RUN a ENOENT nového @SYS souboru.

### Grafy ze skutečných uložených SITL dat

Raw řádky nebyly interpolované na400 Hz. Hlavní let má120 řádků při2 Hz, FF běh120 řádků při4 Hz. Max/RMSE se naopak počítají ze všech podporovaných kroků.

![Body-rate targets a skutečné gyro](../tmp/utb-3g2-20261025_071149/plots-flight-final/body-targets.png)
![Native a vlastní PID výstupy](../tmp/utb-3g2-20261025_071149/plots-flight-final/pid-outputs.png)
![Vlastní I historie](../tmp/utb-3g2-20261025_071149/plots-flight-final/integrator.png)
![FF a D_FF při změnách reference](../tmp/utb-3g2-20261025_071149/plots-ff-final/feedforward.png)
![Všech84 numerických chyb](../tmp/utb-3g2-20261025_071149/plots-flight-final/numerical-errors.png)
![Skutečné decimované dt a identifikátory](../tmp/utb-3g2-20261025_071149/plots-flight-final/timing-records.png)
![Dostupné srovnávací časování](../tmp/utb-3g2-20261025_071149/comparison-timing.png)

Při overflow testu z8000 skutečně vybraných řádků zůstalo128 a7872 přepsání je explicitně vykázaných; numerika přesto obsahuje7999 paired kroků. Při60 s běhu je surový textový export stránek82 872 B, jednotlivá největší stránka11 022 B (skutečná velikost v `comparison-flight/`). Text je větší než hodnotová RAM a nezapisuje se do flash.


## Regrese, buildy a statický audit

- Finální C++:154 PASS +1 SKIP (`UTBMath.OfflineVectors` bez externích vektorů). Z toho8 recorder,20 Chain a14 stávající Timing testů. Logy `unit-final-*.log` a `units-final.json`.
- Python:187 PASS,5 SKIP kvůli nenastavenému externímu `UTB_AUDIT_EVIDENCE`,2 subtests PASS; `python-final.log`.17 nových offline testů používá syntetické protokolové fixture, nikoliv vydávané HW měření.
- Finální Flight matrix12/12; metadata validace `param_parse.py --vehicle ArduCopter --no-emit` PASS; flake8 PASS, astyle nových/změněných částí a `git diff --check` PASS. Původní dvě style odchylky v nezměněné části AP_Filesystem nebyly plošně přeformátované.
- Úplná3G1 aktivní/native matrix16/16 a Rate/Attitude/Chain live regrese3/3 proběhly na `SITL_ON` před opravou pouze výpisu nulového RMSE. Regulační zdroje jsou hashově stejné. Finální release znovu ověřil native/UTB ANGLE/ACRO:4/4 PASS (`final-active-regressions.json`) ; finální Chain v porovnáníB rovněž PASS; přesné binární SHA a příkazy jsou u každého běhu. Výsledky různých binárek se nesměšují.
- Nová kombinace Flight+stávající Timing:4000 DISARMED kroků, missing/duplicate/invalid/unsupported0; `final-timing/timing.json`. Toto je funkční SITL ověření časovací sondy, nikoliv H743 časové měření.

| Build | Flight | Timing | APJ [B] | SHA-256 | Stav |
|---|---|---|---:|---|---|
| H743_OFF | OFF | OFF | 1148735 | `1df5d13362de910560ee6b84fef84ebf1c05a13d5fddc5d27062af990a0417d4` | PASS |
| H743_ON | ON | OFF | 1152955 | `50e7b0bb213e42cec1cc4c2c47cc53af25785d4c38641af7e0e41b20a9ffbc49` | PASS |
| H743_FINAL | ON | ON | 1155815 | `918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7` | PASS |

SITL_ON/OFF a finální SITL_FINAL sestavené úspěšně; příkazy a průběhy jsou v `build-final-results.json`, `SITL_FINAL-results.json` a `postfix-builds.json`. Normální ON/OFF sestavení a finální kandidát jsou rozlišeny výše; původní vývojové/předopravové výsledky nejsou přepsané novou interpretací.


H743 ON proti OFF se stejným Timing OFF: +5252B .text, +39016B .bss, .data beze změny. Linker .heap klesá117624→78608B; nejde o skutečnou volnou RAM po inicializaci všech AP subsystémů. Kandidát s Timing ON přidává proti Flight-only ON dalších2844B .text a1880B .bss; jeho linker .heap je76728B. Další paměťové banky a runtime fragmentace se musí hodnotit zvlášť.

Compiler stack-usage na H743: nový `record`320B, `dump_page`96B, `dump_flight`264B, `finish`32B; stávající `complete_rate`832B, `after_attitude`368B. Jsou to jednotlivé rámce, ne součet celé call-chain nebo změřený watermark. Vyhrazený .pstack7168B a .mstack1536B se nemění. Skutečná rezerva a max. latence alokace/exportu na H743 zůstávají neověřené.

Symbol audit H743 OFF/ON/candidate:0 symbolů `UTB_Core::`, `UTB_Output::`, `UTB_Angle::`, `UTB_Acro::`; Flight symboly pouze v ON. H743 active compile-out se neobchází. Zdrojový audit `protected-source-audit.json` potvrzuje beze změny ověřenou UTB matematiku i původní AP motorovou/arming/failsafe infrastrukturu.

## Kandidátní firmware a reprodukce

Target `SkystarsH7HD-bdshot`, board ID1075, Flight ON, Timing ON, active UTB OFF.

- Soubor: `tmp/utb-3g2-20261025_071149/firmware/SkystarsH7HD-bdshot_UTB3G2_PASSIVE_RAM_918dedaaeae9.apj`
- Velikost: **1155815 B**.
- SHA-256: **`918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7`**.
- Zdrojový build: `H743_FINAL/SkystarsH7HD-bdshot/bin/arducopter.apj`; integrita kopie ověřena.

Jde o pasivní RAM kandidát pro teprve samostatně schvalované USB-only ověření, nikoliv letově kvalifikovaný firmware. Default UCS_F_RUN, UCS_RUN, UNS_RUN, UAS_RUN a UTT_RUN jsou0; aktivní UTB H743 cesta chybí.

Přesné příkazy všech variant jsou v `*-results.json` a `postfix-builds.json`. Kandidát:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
export CXXFLAGS=-fstack-usage
./waf configure --board SkystarsH7HD-bdshot --enable-UTB \
  --disable-UTB_NATIVE_BENCH --disable-UTB_LOGGER_TRACE --disable-UTB_RATE_SITL \
  --enable-UTB_NATIVE_SHADOW --enable-UTB_ATTITUDE_SHADOW --enable-UTB_CHAIN_SHADOW \
  --enable-UTB_FLIGHT_SHADOW --enable-UTB_TIMING \
  --out=tmp/utb-3g2-20261025_071149/H743_FINAL
./waf copter -j6
```

Flight-only ON/OFF varianty mají Timing OFF a liší se enable/disable-UTB_FLIGHT_SHADOW. SITL varianty používají board sitl a enable-UTB_RATE_SITL; tato volba je na H743 vypnutá. Žádný automatický upload.

## Přesný manifest změn

Celkem15 souborů:8 změněných a7 nových. `kapitoly/historie.txt`, firmware a tmp nejsou součástí stagingu.

- `ArduCopter/Attitude.cpp`
- `Tools/scripts/build_options.py`
- `Tools/utb/tests/test_flight_shadow_analysis.py`
- `Tools/utb/tools/test_utb_flight_shadow_sitl.py`
- `Tools/utb/tools/test_utb_skeleton.py`
- `Tools/utb/tools/utb_flight_shadow_analysis.py`
- `docs/UTB_PHASE_3G2_PASSIVE_FLIGHT_SHADOW_CS.md`
- `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`
- `libraries/AP_UTB/AP_UTB_ChainShadow.cpp`
- `libraries/AP_UTB/AP_UTB_ChainShadow.h`
- `libraries/AP_UTB/AP_UTB_FlightShadow.cpp`
- `libraries/AP_UTB/AP_UTB_FlightShadow.h`
- `libraries/AP_UTB/AP_UTB_config.h`
- `libraries/AP_UTB/tests/test_utb_chain_shadow.cpp`
- `libraries/AP_UTB/tests/test_utb_flight_shadow.cpp`


Nové h/cpp recorderu oddělují bounded RAM ukládání od existující matematiky; Chain zajišťuje session policy/publikaci, config a build_options compile guard, Copter jen hodnotový kontext a filesystem jen read-only export. Nové unit/offline/SITL testy a analyzátor zajišťují důkazy. `test_utb_skeleton.py` doplňuje volitelné vypnutí pomocných UART portů pro izolované paralelní nové testy, default původních testů zachovává. Dokumentace popisuje skutečný rozsah; další historické soubory nejsou přesunované nebo přepisované.

## Následný3G-3 – návrh jediného USB-only testu, dosud neprovedeno

1. Nový výslovný souhlas s konkrétním APJ/SHA a experimentem. Potvrdit USB-only, bez LiPo a všech vrtulí, DISARMED a volný COM5. Ověřit board1075 a serial200024001251303433333336, zdraví senzorů/AHRS/EKF a stabilní spojení. Zazálohovat parametry; logy zachovat.
2. Před případným uploadem znovu ověřit integritu souboru a identitu FC. Žádný bootloader/DFU zásah, mazání nebo automatický reboot. Potřebný fyzický restart provede člověk po samostatné výzvě.
3. Ověřit native STABILIZE,400 Hz, fast-rate/rate-thread OFF, podporované notch/SMAX a ATT_ENABLE/custom control. Nepodporované AP nastavení neobcházet kvůli PASS. RUN všech ostatních sond0.
4. Přibližně15 s baseline se stejnou minimální telemetrií. Změřit skutečný CPU, dostupný heap a stack watermark, zdraví a Main/Rate/Attitude periodu. Neotvírat logger jen kvůli benchmarku a nezaměňovat AP virtuální SITL čas za HW čas.
5. Navržené jediné okno: UCS_F_COUNT4000, UCS_F_DECIM40, UCS_F_RUN1, nominálně10 s/100 řádků. Volitelný souběh stávající UTT_COUNT4000/UTT_RUN1 musí být součástí schváleného protokolu a jeho vlastní overhead se oddělí. Během okna žádné FTP/listing/download/PARAM_SET/logger změny ani tasks.txt.
6. Po auto-stop ověřit DISARMED/RUN0/zdraví. Sekvenčně stáhnout metadata a osm stránek, ověřit úplnost,84 metrik,1 warmup,3999 paired, jednotlivé reset operace,1+1 synchronizaci, motor_changes0 a časování. PM.NLon/MaxT uvést pouze pokud je lze získat bez rušení měření; jinak neověřeno.
7. Oddělit překrývající native+capture, UTB math, comparison, consume a recorder spany; nesčítat je jako nezávislé náklady. Změřit reálný watermark po výpočtu i exportu a min. heap. Samotný linker stack size ani warning threshold není důkaz rezervy.
8. Stop při mismatch/desync/invalid, změně motorových hodnot, InternalError, restartu, poruše senzorů/loggeru nebo nové závažné periodové anomálii proti baseline; neprovádět automatický další pokus. Neurčovat nový letový limit libovolným procentem CPU. Obnovit diagnostické parametry/telemetrii a ověřit stav FC.

Ruční natáčení nearmované FC lze navrhnout jen jako zvlášť bezpečně vymezený úsek. Nedokazuje ARMED integraci, zatížení motorů nebo letovou bezpečnost.

## Před případným3G-4 – samostatně schvalovaný pasivní let

Nejdříve musí projít3G-3, aktuální kapacita a nedestruktivní správa existujících logů, qualification persistentního záznamu nebo konkrétně schválený omezený RAM protokol s rizikem ztráty při resetu, logger startup/průběžné blokování, skutečný CPU/WCET/jitter/stack/heap, konkrétní parametry a původní AP failsafe/arming integrace na fyzické konfiguraci. Teprve potom nový pilotní plán a výslovný souhlas. Motory by i nadále řídil výhradně AP STABILIZE. Aktivní UTB režimy na H743 nejsou součástí plánu.

## Závěrečná rozhodnutí

1. Dynamický ARMED native STABILIZE: **SITL ověřeno v popsaných scénářích**, fyzický let neověřen.
2. Motorová autorita: zůstává původnímu AP; hodnotová RAM větev nemá výstupní sink. H743 doloženo zdrojem/buildem/symboly, nový fyzický test neproběhl.
3. Dlouhodobé trvalé logování bez přetížení: **neprokázáno**, RAM je pevně omezená max.120 s session.
4. Data pro porovnání:84 agregací celé podporované session + skutečné decimované průběhy; nedovolují plnou400 Hz rekonstrukci nebo přežití resetu.
5. Nenulové reference/gyro/shaping/I/FF a dynamické scénáře: doloženy skutečnými SITL daty, nikoliv syntetickým HW tvrzením.
6. Vlastní I historie: ARMED pokračuje bez každokrokového importu; reálné DISARMED reset/I_set události jsou zvlášť počítané.
7. RAM ztráty a neúplné exporty: explicitně detekované; fyzické logger chyby/delays nejsou tímto plně kvalifikované.
8. H743 izolace: staticky/buildově zachovaná, runtime nové varianty na desce dosud neověřeno.
9. První USB-only test: vyžaduje nový souhlas, identitu, fyzické podmínky, zdravý start a schválený konkrétní protokol výše.
10. Před letem chybí především reálné časování/paměť, persistentní data a logger kvalifikace; tato fáze let nepovoluje.