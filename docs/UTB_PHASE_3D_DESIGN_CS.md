# FÁZE 3D-1 – logger a návrh pasivního 400Hz porovnání AP/UTB

Datum: **20. 10. 2026**. Větev `test/skystars-5inch`, auditovaný HEAD
`070c189cdfa8357307ee1aebe8f3677cd849d520`. Reference: lokální Copter 4.7.1
s moduly FÁZE 3A a diagnostikou 3B. Navazuje na [výsledky 3C](UTB_PHASE_3C_CS.md)
a [architekturu 3A](UTB_PHASE_3A_CS.md).

**Stav: návrh, nikoliv implementace 400Hz SHADOW.** V této fázi byl přečten kód
a existující výsledky; vznikl pouze tento dokument. Nebyl otevřen COM5, spuštěn
build, změněn firmware, parametr ani regulátor. Dosavadní práce i výsledky
zůstávají zachovány. Nové názvy rozhraní a přepínačů níže jsou návrhy API.

## 1. Rozsah dosavadních důkazů

| Oblast | Stav |
| --- | --- |
| Matematika 3A na H743 | HW ověřeno ve fixture 3B/3C: 3 × 2000 platných sad, max. chyba i RMSE 0; daná konfigurace, SMAX=0 |
| 50Hz diagnostika | Ustálený HW běh bez drops a PM.NLon během tří RUN; nikoliv všechny startup scénáře |
| Cykly 5503/6480 µs | HW naměřeno před RUN, přesný původ neobjasněn |
| Live AP/UTB se shodnou historií | Neimplementováno, neověřeno |
| Nový 400Hz SHADOW | Návrh; unit/SITL/HW neověřeno |
| Aktivní řízení na H743 a let | Neověřeno a tímto dokumentem nepovoleno |

Fixture používá oddělené instance a řízené počáteční stavy. Ani profil se
skutečným gyro/AHRS neporovnává historii živého AP controlleru. Nulové chyby
nelze přenést na všechny filtry, konfigurace nebo aktivní 400Hz smyčku.

## 2. Logger: skutečný backend a možné blokace

### 2.1 Cílový backend

[Hwdef SkystarsH7HD](../libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD/hwdef.dat),
ř. 28,58–77, definuje SPI3 `dataflash` W25Q128; bdshot varianta z něj vychází.
Poslední parametry 3C obsahují **LOG_BACKEND_TYPE=4: BLOCK**, nikoliv FILE/SD=1.
Viz [AP_Logger.cpp](../libraries/AP_Logger/AP_Logger.cpp), ř. 97–102,245–249.
Relevantní jsou AP_Logger_Block a AP_Logger_Flash_JEDEC, nikoliv AP_Logger_File.

### 2.2 Konkrétní cesty

| Zdroj a řádky | Chování |
| --- | --- |
| [AP_Logger_Backend.cpp](../libraries/AP_Logger/AP_Logger_Backend.cpp):330–344,442–468 | První povolený zápis může synchronně volat start_new_log; StartNewLogOK vyžaduje hlavní vlákno. |
| [AP_Logger_Block.cpp](../libraries/AP_Logger/AP_Logger_Block.cpp):545–595,669 a dále | Start drží `sem`, hledá poslední stránku a provádí StartRead. Komentář o absenci zápisu neznamená absenci čtení/čekání. |
| [AP_Logger_Flash_JEDEC.cpp](../libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp):224–248 | PageToBuffer při cache miss volá WaitReady, poté čte SPI pod `dev_sem`. |
| Tentýž soubor:92–107,324–329 | WaitReady opakuje Busy a delay_microseconds(100), timeout je až přes 5000 ms. Není to vhodná horní mez pro 2,5ms smyčku. WriteEnable rovněž volá WaitReady. |
| [AP_Logger_Block.cpp](../libraries/AP_Logger/AP_Logger_Block.cpp):126–174 | Běžný zápis drží `write_sem`, kopíruje do RAM writebuf a při nedostatku prostoru zahazuje zprávu. BENCH zde neměří všechny logger zámky. |
| Tentýž soubor:836–929,89–115 | IO callback odčerpává nejvýše stránku za průchod pod `sem`; FinishWrite programuje flash a na hranici bloku spouští erase. Nejde o garanci maximální latence. |
| [AP_Logger.cpp](../libraries/AP_Logger/AP_Logger.cpp):1014–1069,1142–1240 | WriteStreaming vede přes WriteV a `log_write_fmts_sem`; první formát může alokovat metadata. Název Streaming neznamená neblokující API. |
| [AP_Logger_MAVLinkLogTransfer.cpp](../libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp) | List/download obsahují další synchronní cesty a zámky; neprovádět během měřicího okna. |

`write_sem`, `sem` a `dev_sem` jsou různé zámky. Dosavadní MAIN_ACQUIRE maximum
nelimituje čekání ve start_new_log/flash. Nulové potvrzené kolize main/worker
nevylučují jiný nebo nezachycený překryv. Priority inheritance neodstraní čekání
na flash/SPI ani preempci a nedává WCET.

### 2.3 Co říkají časové záznamy

Důkazy: [audit časové osy](../tmp/utb-3c-20261020_151107/timing-anomaly-source-audit.json),
[3B report](../tmp/utb-3b-measure-20261020_143658/REPORT_CS.md),
[3C report](../tmp/utb-3c-20261020_151107/REPORT_CS.md).

- **5503 µs:** PM.TimeUS=1114993881, NL=4000 při LR=400. Přibližné okno
  1104,994–1114,994 s od startu; první native výpočet až 1123,713489 s.
  První histogram v 1111,877140 s už obsahuje všechny později přítomné
  události svého nejvyššího jitter binu.
- **6480 µs:** PM.TimeUS=914839017, přibližné okno 904,839–914,839 s.
  Velký jitter je už v prvním histogramu v 907,680962 s, před native RUN.
  Další dva warm běhy obdobnou anomálii nemají.
- Počátky PM oken jsou odhady z počtu cyklů a frekvence, nikoliv přesně
  zaznamenané hranice. Histogram neobsahuje timestamp jednotlivé události.
- FC tehdy běžela přibližně 18,6 / 15,2 minuty. Nešlo o bezprostřední MCU boot.
  Přesné označení je první otevření loggeru v pozorované session.
- Obě události předcházejí native matematice i její první registraci U3B formátů.
  Maximum pozdějšího U3B exportu 440 µs je nevysvětluje.
- U 3B jsou přímé erase/list operace dříve než odhadované anomální okno;
  v něm neprobíhal BIN download. Dřívější operace mohly ovlivnit stav flash,
  ale jejich přímá obsluha není časově doloženým viníkem.
- Okna překrývají konec baseline bez logování a první povolení loggeru.
  **Korelace existuje, příčina prokázána není.**

Hypotézy: synchronní hledání/čtení flash při otevření, čekání na flash/zámek,
obsluha nebo uložení parametru při zapnutí logování, jiná scheduler úloha
či preempce. Existující data mezi nimi nerozhodnou; chybí span a event capture.

### 2.4 Scheduler: tři různé veličiny

[AP_Scheduler.cpp](../libraries/AP_Scheduler/AP_Scheduler.cpp):242–295:
UBST8 počítá `time_taken > _task_time_allowed`. Běžná úloha má vlastní
max_time_micros, FAST_TASK nominální periodu. Wall čas zahrnuje preempci
a část dispatch, čítač nemá ID viníka. V 3C existuje přibližně 11–14 událostí/s
i při RUN=0. Nelze je automaticky připsat UTB nebo konkrétní 10Hz úloze.

[PerfInfo.cpp](../libraries/AP_Scheduler/PerfInfo.cpp):119–135,227–230:
PM.NLon počítá periodu nad 120 % nominálu, při 400 Hz nad 3000 µs.
UBST9 v [AP_Logger_UTBBench.cpp](../libraries/AP_Logger/AP_Logger_UTBBench.cpp):434–459
počítá už periodu nad 2500 µs, tedy i malý kladný jitter.

Budget úlohy, mezi-sample jitter a překročení požadované 2,5ms periody nejsou
totéž. Nulové NLon neprokazuje každý deadline ≤2500 µs. Obě anomálie překračují
nominál i PM práh; přesný počet ztracených INS vzorků ze souhrnu určit nelze.

### 2.5 Možnost blokace za letu

Zdrojově ji nelze vyloučit: pokud log ještě není otevřen, první povolený zápis
může přijít až po armování. Otevření předem odstraní tento startovací případ,
nikoliv mutexové čekání, programování/erase, zaplnění nebo zotavení backendu.
Jde o **zdrojově doloženou možnost, nikoliv naměřenou letovou závadu**.
Bez objasnění anomálie a boundary testů se let neschvaluje.

## 3. Minimální cílený DISARMED experiment loggeru

**Zatím nespouštět; vyžaduje samostatný souhlas.** První varianta používá
stávající firmware, UB3_RUN=0, historický SHADOW=0, bez uploadu a erase.

1. Ověřit board 1075, USB serial 200024001251303433333336, DISARMED,
   bez LiPo/vrtulí; uložit parametry, health, BOOTCNT a kapacitu.
   Při nedostatku místa zastavit: BLOCK může přepsat starší logy.
2. Oddělit cold log od LOG_DISARMED=0→1 na již otevřeném logu; samotné
   přepnutí soubor neuzavírá. Cold session připraví fyzický restart obsluhou
   s potvrzením a vyčkáním na zdravé AHRS/EKF. Neprovádět software reboot.
3. Kontrola A: stávající statistiky, bez přidaného per-task sběru. Kontrola B:
   přidat pouze bit0 SCHED_OPTIONS, zachovat ostatní bity a před měřením
   vyčkat na alokaci. Nastavení/uložení tohoto parametru není součástí
   sledovaného logger přechodu. Změny message rates rovněž připravit předem.
4. Alespoň 30 s RUN0/no-log, jednou LOG_DISARMED=1, potom alespoň 30 s
   bez dalších příkazů. Délka pokrývá několik přibližně 10s PM oken,
   není bezpečnostním limitem. Až potom číst per-task max/overruns přes
   MAVFTP `@SYS/tasks.txt`. Srovnat warm přepnutí na už otevřeném logu.
   Nejprve jediný cold/warm pár každé kontroly; další opakování až podle nálezu.
   Tři opakování slouží následné kvalifikaci, nikoliv minimálnímu hledání viníka.
5. Uložit host monotónní časy příkazů/odpovědí a mapování na boot time,
   PM/UBMT/UBHI/UBST, health a per-task statistiku. BIN stahovat až po okně.
   Vždy rozlišit kumulativní maximum od delty sledovaného intervalu.
6. Obnovit parametry a přečíst zpět. Při InternalError, restartu, ztrátě
   EKF/loggeru nebo opakované dlouhé periodě ukončit měření a vyhodnotit důkazy
   před dalším pokusem. Žádný ARM, motorové příkazy nebo mazání logů.

API: [AP_Scheduler.cpp](../libraries/AP_Scheduler/AP_Scheduler.cpp):441–457,491–505;
[PerfInfo.cpp](../libraries/AP_Scheduler/PerfInfo.cpp):45–79;
[AP_Filesystem_Sys.cpp](../libraries/AP_Filesystem/AP_Filesystem_Sys.cpp):102–103.
První čtení tasks.txt může samo zapnout sběr a následnou alokaci. Textový export
má režii, proto až po okně. Per-task maximum může zúžit viníka, nikoliv určit
čekání uvnitř úlohy. Bez logování nejsou PM záznamy uložené; hranice resetů
mohou přechod překrýt. Stávající firmware nemusí přesný cold event zachytit.

Upřesnění z 3D-2: `PerfInfo::reset()` (PerfInfo.cpp:25–35) nuluje také celé
pole TaskInfo. Čtení až po 30s okně proto nezachová viníka původní špičky.
Pro atribuční pokus musí vzniknout snímek před následujícím PM resetem,
s oddělenou režií MAVFTP, nebo musí instrumentace zmrazit událost v RAM.
První pokus 3D-2 znovu ukázal dlouhou periodu a další pokusy byly zastaveny;
viz [výsledek 3D-2](UTB_PHASE_3D2_LOGGER_CS.md).

Pokud první varianta nestačí, až po novém schválení připravit malý diagnostický
build s pevnou RAM stopou: loop start/end, task ID, logger start enter/exit,
find_last_page, WaitReady, čekání/držení jednotlivých semaforů a obsluha parametru.
AP_HAL hardware micros/micros64, modulární uint32 rozdíly, omezené spany,
freeze na prvním dlouhém cyklu. Žádný print, heap, DataFlash nebo nový zámek
při capture ani pod měřeným mutexem. Thread ID a oddělená publikační pole
zabrání race. Export až po okně; OFF/ON sonda změří vlastní režii.
Trigger 3000 µs zachytává stávající PM typ události, není nový letový práh.

## 4. Architektura nového 400Hz SHADOW

### 4.1 Přepínače a podporovaný profil

Navržený samostatný `AP_UTB_NATIVE_SHADOW_ENABLED=0` rozšíří jen dostupnost
matematiky a pasivního capture. Nesmí rozšířit AP_UTB_RATE_SITL_AVAILABLE
na H743. Runtime enable bootuje vypnutý; název/index parametru ověřit před
implementací. UB3_RUN nepoužívat jako přepínač nového letového chování.

Compile OFF odstraní hooky, worker i RAM. Runtime OFF nepočítá, neregistruje
formáty a neexportuje; samostatně změřit cenu zbývajícího enable testu.
První profil: main loop 400 Hz, FSTRATE_ENABLE=0, using_rate_thread=false,
původní AP ACRO/STABILIZE bez custom controlleru přebírajícího výstupy.
Frekvenci ověřit z časů, nestačí SCHED_LOOP_RATE. Unsupported konfigurace
vypne pouze porovnávání s důvodem; AP režim, filtry, gains a arming nezmění.
Historický SHADOW a fixture 3B při kvalifikaci vypnout, jejich kód zachovat.

### 4.2 Pořadí a integrační místa

[ArduCopter/Copter.cpp](../ArduCopter/Copter.cpp):115–139:

```text
INS.update → run_rate_controller_main → případný custom controller
→ motors_output_main → historický utb_shadow_update
→ read_AHRS → read_inertia → check_ekf_reset → update_flight_mode → ...
```

[ArduCopter/Attitude.cpp](../ArduCopter/Attitude.cpp):40–54 nastavuje dt,
spouští native rate controller a pak resetuje dočasné sysid vstupy.
[AC_AttitudeControl_Multi.cpp](../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp):442–476
čte latest gyro, target, gain boost/mix, sysid, scale a limit flags; zapisuje
R/P/Y i FF do AP_Motors. Samotné rate_bf_targets nejsou všechny vstupy.

| Bod | Capture/výpočet |
| --- | --- |
| R0 uvnitř native rate vstupu | Přesné AP gyro, dt, target/sysid před resetem, vstupní historie, config generation; ne druhé čtení gyro. |
| R1 po gain boost/mix u PID výpočtu | Použité pd/i scale, limit flags, FF scalar, actuator sysid a motorové read-only vstupy; po výpočtu AP PID info a výstupy. |
| R2 po motors_output_main | UTB Rate/PID ze zachyceného rámce, porovnání. AP motorový zápis už proběhl. Nepoužít zde novější limit flags pro právě skončený PID. |
| A0/A1 u native attitude input | Argumenty, AHRS, reset/config události a pre-state; potom native target a post-shaping stav. |
| A2 po update_flight_mode | UTB Attitude; srovnat target určený pro následující rate krok. |

R0/R1 jsou krátké read-only hooky pod guardem. Velké snapshoty před motorovým
výstupem mohou změnit latenci: velikost i cenu je nutné změřit. Žádné opakované
AP update_all ani druhý běh native attitude controlleru. Přesné gyro/dt vznikne
uvnitř rate_controller_run_dt. Potřebné privátní hodnoty zpřístupnit explicitním
diagnostickým exportem pod guardem, nepřesouvat původní matematiku.

AHRS se aktualizuje až po AP rate/motor kroku. Pozdější mode update typicky
připravuje target pro další rate cyklus. Rámec nese SeqRate, SeqAttitude,
ResetEpoch, ConfigGeneration, TimeUS, GyroTimeUS, CaptureTimeUS, Mode a dt.
Spojovat podle fáze a sekvence, nikoliv nejbližšího TimeUS; nepřipojit k rate
vzorku attitude z budoucího kroku. Zachovat původní logové timestampy.

První attitude rozsah: [mode_stabilize.cpp](../ArduCopter/mode_stabilize.cpp):28–60
(spool reset yaw/I, input_euler_angle_roll_pitch_euler_rate_yaw_rad, angle boost)
a [mode_acro.cpp](../ArduCopter/mode_acro.cpp):24–67 (reset target/I,
RATE_LOOP_ONLY varianta, případné scale_I_to_angle_P, throttle bez angle boost).
Zachytit skutečný typ volání; oba ACRO inputy nejsou zaměnitelné.

### 4.3 Izolace motorů a senzorových hodnot

Žádná reference na živý AP_Motors nebo měnitelný AP regulátor v diagnostickém
modulu. Jen kopie hodnot. [UTB_Output.cpp](../libraries/AP_UTB/UTB_Output.cpp):28,46,64,87
skutečně volá motor setters, takže označení instance „shadow“ ji neizoluje.

Pro minimum změn použít 3B RAM-only motor sink s vypnutou registrací singletonu
a inertními init/output/test metodami. Nikdy nekonstruovat UTB_Rate s živým
`*copter.motors`. Motorové vstupy z R0/R1 naplnit do diagnostického sinku;
výsledky číst jen z něj. Auditovat constructory, singletony, virtuální output
a HAL/RCOutput cestu. Změna na čistě hodnotové output rozhraní může přijít
samostatně, není podmínkou prvního kroku.

[UTB_State.h](../libraries/AP_UTB/UTB_State.h):24–63 dnes deleguje na živý
AP_AHRS_View. Pro oddělené časové výpočty navrhnout frozen hodnotový adaptér:
použité gyro/quaternion/Euler/trig/accel zachytit v příslušné fázi a UTB čte
jen tento snapshot. Původní live adaptér ponechat pro SITL. Žádný nový EKF,
INS filtr nebo zápis do AHRS.

### 4.4 Historie, reset a saturace

[AC_PID.cpp](../libraries/AC_PID/AC_PID.cpp):196–304,340–389 závisí na historii
target/error filtrů, derivative/target derivative, I, reset flagu, předchozím
P/D a slew limiteru. Notch má další stav. Kopie I a PID info nestačí.
[AC_AttitudeControl.cpp](../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp):979–1030,1252–1296
obsahuje quaternion target/error, rate/accel shaping, yaw a inertial-frame reset.
Přenášet i throttle mix/gain boost, FF scalar a scale reset události v pořadí AP.
Limit flags pocházejí z reálného AP mixeru/spool; sink nevyrábí vlastní saturaci.

Dva testy, jejichž závěry nesmí být směšovány:

1. **Jednokrokový:** před krokem přenést explicitní hodnotový AP stav do UTB.
   Ověřuje matematiku/capture, nikoliv nezávislý vývoj historie.
2. **Kontinuální:** společná reset epocha, pak vlastní UTB historie a doručování
   skutečných reset/config/spool/EKF událostí v původním pořadí. Porovnávat
   pre/post stav. Při mezeře DESYNC a zastavení porovnávání do resynchronizace.
   Neresetovat UTB každý cyklus a nevydávat to za kontinuální validaci.

Snapshot je verzovaná pevná POD struktura s vyjmenovanými poli. Žádný memcpy
celých AC_PID/UTB_PID, AP_Param objektů, polymorfních tříd a filter pointerů.
Filtry potřebují vlastní paměť a úplný hodnotový stav. Alokace/notch konfigurace
jen před enable v DISARMED; ve fast path žádný heap.

Rate-only etapa přebírá skutečný AP body-rate target. Celý řetězec později
používá vlastní UTB Attitude target z odpovídající předchozí A2 sekvence;
nesmí jej každý cyklus přepsat AP targetem a tvrdit, že ověřuje celý řetězec.
Izolované Rate srovnání a propagovaná chyba celého řetězce jsou dva oddělené
testovací režimy se samostatnou inicializací a reportem. V obou případech
zůstává výhradním zdrojem skutečných saturation flags původní AP mixer.

**Otevřené:** [SlewLimiter.cpp](../libraries/Filter/SlewLimiter.cpp):55 a
[UTB_PID.cpp](../libraries/AP_UTB/UTB_PID.cpp):522 čtou vlastní AP_HAL::millis().
AP a UTB mohou překročit ms hranici mezi voláními. Pro SMAX≠0 je nutný sdílený
explicitní čas kroku se zachováním původního default AP chování, nebo profil
UNSUPPORTED. Nelze vypnout SMAX/notch v AP jen kvůli nulové chybě.
První profil smí běžet pouze s reálně podporovanou konfigurací; nepodporované
funkce se hlásí, ne tiše vynechají. Změna parametrů, notchů nebo gyro instance
vytváří novou config/reset epochu.

## 5. RAM porovnání, export a ochrany

### 5.1 Porovnání a fronta

Každý platný rate cyklus porovnat P/I/D/FF/D_FF, Dmod, výsledné R/P/Y a FF,
filtrované targety a relevantní historii. Attitude později quaternion, shaping
rate/accel a body target. Quaternion hodnotit komponentově i geometricky
s ekvivalencí q/−q; geometrická tolerance nesmí skrýt odlišnou historii.

RAM statistiky: valid/invalid/unsupported/desync, max abs chyba, sum squares
a počet pro RMSE, první rozdíl Seq, pevné časové histogramy. Součty uzavírat
do omezených oken s kontrolou overflow/precision. NaN/Inf a chybějící vzorky
nepočítat jako nulové chyby. Žádný text ve fast path.

Oddělit ControllerValid, StateValid, StateSynchronized, LoggingAvailable,
Observable a DiagnosticsOverloaded. Chyba loggeru není chyba PID. UTB chyba
smí vypnout pouze diagnostiku, nikdy nativní controller, failsafe nebo tah.
Budget watchdog zakáže až další krok, neumí přerušit už běžící výpočet;
nenahrazuje předchozí WCET a fault testy.

Main producer: pevný SPSC ring, kopie slotu, release head; consumer acquire,
kopie, release tail. Žádný mutex, spin/CAS smyčka, heap, GCS text ani DataFlash
ve fast produceru. Ověřit lock-free 32bit atomiky na H743. Nesdílet nechráněné
64bit statistiky. Při plné frontě drop celé nové sady, nikdy přepis čteného slotu.

Pracovní rozpočet fronty detailu: **64 × nejvýše 128 B = 8192 B**, při 400 Hz
pouze 160 ms. Není to krytí nejhorší logger latence. State snapshoty patří do
samostatného pevného dvojbufferu; celá historie se nevejde automaticky do 128 B.
Drops/Seq gaps/HWM a export heartbeat jsou součástí evidence.

### 5.2 Kompaktní protokol

Návrh: okno 40 cyklů = 10 Hz; nejvýše osm statických zpráv po 64 B na okno,
tedy **5120 B/s** včetně DataFlash packet headerů, bez page/startup overheadu.
Skupiny: chyby PID/rates/attitude, validita a počty, epochy, časy a fronta.
Detail prvního rozdílu nebo výslovně povolený krátký burst nejvýše 128 B/cyklus
= **51 200 B/s** při 400 Hz. Souhrny mají přednost před detailem.

Před implementací ověřit pole/jednotky, FMT délky a volné log IDs; zde žádná
ID přidělena nejsou. Protokol musí být odlišný od U3B i historických UTBR/UTBM.
128 B může být několik úplných zpráv s Batch/Seq, ne nadlimitní packet.
Sledovat produced/enqueued/dequeued/backend-accepted/offline-complete.
Vložení do writebuf není potvrzení zápisu na flash; dosaženou frekvenci úplných
uložených sad definitivně ověřuje BIN, ne producer counter.

### 5.3 Neuzavřený mutexový kontrakt exportu

**Stávající WriteStreaming není neblokující export.** Worker s nízkou prioritou
může držet logger mutex, na který čeká main. Ani worker try-acquire sám o sobě
nezaručuje, že main nepočká na worker, který už zámek získal.

Proto první implementace **400 Hz pouze RAM**, export až po zastavení.
Průběžný export je další samostatný krok:

1. Statická metadata/FMT, bez runtime mapování formátů a heapu; otevření loggeru
   a dokončení startupu ještě DISARMED, mimo měření.
2. Malá backend-specific try-submit cesta předem sestavených necritical zpráv:
   žádné start_new_log, FMT emit, flash read, WaitReady, čekání ani nested lock
   pod drženým zámkem. Busy/not-ready/no-space → drop. Je to potřebné nové
   rozhraní, nikoliv vlastnost existujícího WriteStreaming.
3. Kritická sekce omezená kopií pevné dávky, ne drain celé fronty. Měřit main
   wait i worker hold všech dotčených zámků, preempci a sektorové hranice;
   nezměnit původní pořadí a politiku loggeru.

I tak zůstává možnost krátkého main čekání na sdílený write_sem.
**Absolutní požadavek nulového nového main blocking tato varianta nesplňuje.**
Pokud je požadavek absolutní, je před exportem nutné samostatně navrhnout
oddělený ingress do původního IO consumeru bez dalšího držitele AP producer
mutexu. To vyžaduje vlastní backendový návrh, nikoliv automatickou změnu AP.
Diagnostické bajty nelze vkládat libovolně mezi stránky, přes něž může pokračovat
native zpráva. Do vyřešení kontraktu zůstává průběžný export otevřený;
RAM-only nepřidává logger writer. Ani ona neléčí původní AP logger blokace.

## 6. Odhad zátěže

**Následující jsou extrapolace, ne nové 400Hz výsledky.**

| Položka | Podklad/odhad |
| --- | --- |
| UTB matematika | 3C pooled mean 34,022 µs ×400 = 13,61 ms/s, 1,36 % wall kapacity; směs profilů, nikoliv WCET |
| Rate / attitude+rate | 3B přibližně 25,9 / 51,4 µs → 1,04 / 2,06 % wall kapacity při 400 Hz; jiné flight konfigurace mohou být náročnější |
| Capture/synchronizace/compare | Neměřeno, není v uvedených procentech; velké snapshoty mohou převážit matematiku |
| Původní 3B formát | 510 B ×400 = 204000 B/s navíc k AP; bez kvalifikace propustnosti nepoužít |
| Nové souhrny/burst | Návrhově 5120 B/s, případně dalších 51200 B/s; bez flash page/startup overheadu |
| Původní export | Mean 101,414 µs ×400 = 4,06 % wall kapacity bez contention; nepřenášet do fast path |
| RAM/stack | Fronta 8 KiB + instance, filtry, snapshoty a statistiky; konečná velikost až sizeof/map/stack měřením |

Native AP už běží, nepřidává se druhá AP fixture. Maxima AP 169, UTB 168,
export 440 µs z 3C nejsou součet ani WCET. Preemptible wall times nejsou
izolovaná CPU utilization. Ověřit proti párovanému baseline se stejným logováním.

Měřit zvlášť pre-AP capture, UTB math, compare, enqueue a celý přídavný úsek;
AP rate-to-motor latency; periodu a rezervu celé smyčky před další INS;
následné AHRS/mode kroky; consumer wall/hold/wait/export. AP_HAL monotónní
hardware hodiny, kontrola ceny prázdné sondy. Virtuální SITL čas není HW timing.

## 7. Postup implementace a ověření

| Krok | Obsah a podmínka postupu |
| --- | --- |
| D0 | Schválený cold/warm experiment; pokud nestačí, samostatně schválit RAM span sondu. Bez oprav podle domněnky. |
| D1 | Default-OFF guard, RAM-only PID/Rate capture, frozen vstupy, inertní sink; bez průběžného exportu a Attitude větve. |
| D2 | Unit snapshotů, obě znaménka, filtry, I shrink při saturaci, FF/DFF, reset, invalid, wrap hodin, unsupported konfigurace. |
| D3 | SITL live AP ACRO/STABILIZE: jednokrokové a potom kontinuální porovnání; throttle/spool, přechody, RC/GCS failsafe, EKF/reset události. |
| D4 | Stress fronty: wrap/full, consumer stall/failure, logger unavailable/full, config/desync/invalid; AP pokračuje nezávisle. Audit race/heap/lock. |
| D5 | H743 build a active compile-out; až po dalším souhlasu s uploadem a session DISARMED RAM-only měření plné režie. |
| D6 | Až po rozhodnutí o mutexovém kontraktu kompaktní export, OFF/ON, flash boundary/stall testy; ne hned 400Hz detail. |
| D7 | Attitude A0/A1/A2 a replay shaping/reset událostí; unit→SITL→DISARMED, nejdříve target, pak celý kontinuální řetězec. |

Před implementací reprodukovatelný checkpoint včetně untracked manifestu/hashů.
Očekávané soubory: malý native-shadow modul a Copter glue v AP_UTB,
AP_UTB_config.h, Copter.h/Attitude.cpp/Copter.cpp, případně mode_acro/stabilize
pro přesný capture; read-only snapshot rozhraní AC_PID/AC_AttitudeControl
a filtrů, frozen UTB_State adaptér, testy/build_options.py/dokumentace.
Exporter až samostatně v AP_Logger. AP_Motors/HAL/DShot matematiku neměnit.
Přesný manifest vznikne z odsouhlaseného rozsahu D1.

Build matice: SITL a SkystarsH7HD-bdshot, nový SHADOW OFF/ON, UTB OFF/ON,
HAL_LOGGING_ENABLED=0, původní BENCH OFF/ON a native fixture guardy.
Zachovat unit/SITL regrese, metadata/unikátní IDs, diff --check, astyle/flake8
podle změn. **V této návrhové fázi tyto buildy/testy spuštěny nebyly.**

## 8. Měřitelné podmínky a stop kritéria

- Pro identické portované operace očekávat max chybu/RMSE 0 v podporovaném
  profilu. Nenulový rozdíl rozdělit na vstupy, historii, čas, konfiguraci
  a aritmetiku. Případnou toleranci odvodit a schválit před hodnocením,
  nikoliv upravit podle dosažené chyby.
- Kvalifikační okno bez nevysvětlených Seq gaps/reset/config mismatch/invalid;
  vyřazené vzorky hlásit s důvodem. DISARMED nevybudí celou letovou saturaci
  ani integraci, tyto scénáře musí prokázat SITL.
- Alespoň tři párovaná OFF/RAM-only/export opakování se stejnou konfigurací;
  min/mean/median/P95/P99/max, nikoliv jen CPU Load. Zvlášť cold/warm/boundary.
- 400Hz požadavek je 2500 µs, PM alarm 3000 µs je jiná veličina.
  Diagnostický budget odvodit z nejmenší změřené rezervy celé původní smyčky,
  ceny sond a doložené preempční rezervy. Dosavadní průměry neobhájí libovolný
  pevný budget/margin. FAST_TASK nelze přerušit jen proto, že překročil budget.
- Souhrnný export bez drops, úplné uzavřené dávky v BIN, omezená HWM,
  žádné zhoršení AP DSF/health. Úmyslný stress drop přesně zaznamenán,
  bez blokace AP. Accepted není persisted.
- Source/call-graph audit izolace a SITL shoda native motorových příkazů
  OFF/ON při stejném vstupu. Žádný živý motor sink/singleton/RCOutput pro UTB;
  porucha diagnostiky nesmí vyvolat thrust cut nebo nahradit AP failsafe.
- Worker create failure, heartbeat, stack watermark/skutečná velikost stacku,
  logger unavailable. Stack warning není dostatečná rezerva. Failure simulovat
  test seam, nikoliv vyčerpáním heapu na FC.
- HW InternalError, restart, závada EKF/loggeru, porušení schváleného rozpočtu
  nebo nový nevysvětlený dlouhý cyklus: zastavit experiment, uložit důkazy.
  Případná budoucí letová diagnostika smí vypnout jen sebe, ne AP řízení.

## 9. Doporučení a otevřené otázky

První práce: **objasnit cold logger špičku stávající diagnostikou, potom D1
RAM-only Rate/PID**. Attitude a průběžný export jsou další oddělené kroky.
Již HW porovnaná regulační matematika se neladí.

Otevřené body:

1. Přesná příčina 5503/6480 µs a potřeba nové RAM span sondy.
2. Úplná historie/reset události live profilu, SMAX/notch a sdílený čas kroku.
3. Kontrakt exportu: absolutně žádné nové main čekání, nebo doloženě omezená
   kritická sekce. Stávající API první variantu nezaručuje.
4. Skutečný capture/state/compute/compare budget, RAM, stack a rezerva 400Hz smyčky.
5. Formát/IDs, unsupported features a resynchronizace; nezaměnit ztrátu
   observability za matematickou chybu controlleru.

Pasivní let s výhradním řízením AP vyžaduje samostatný návrh a schválení
po uzavření těchto bodů, DISARMED kvalifikaci buildu a SITL kontinuálního řetězce.
Tato fáze neschvaluje upload, HW session, armování, let ani aktivní UTB řízení.
