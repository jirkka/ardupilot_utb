# FÁZE 1H-C – Skystars H743: interní reference a nenulový PID

Datum: 12. 10. 2026. Větev `test/skystars-5inch`.

**H743 VERIFIED / PASS pro jednu kompletní DISARMED SHADOW sekvenci:**
8000/8000 platných uložených UTBC, všechny Seq kroky=1, všech deset fází,
matematicky správný PID/mixer a přirozený COMPLETE bez RC vstupu.
Během RUNNING nevzrostly queue/backend ztráty ani DSF.Dp; PM.NLon=0.

**Globální tvrzení „bez negativního dopadu na původní ArduPilot“ není
prokázáno.** Logger startup má ztráty a jednu dlouhou main periodu.
Po downloadu se opakovaně objevil EKF UNINITIALIZED. Tyto události
jsou mimo RUNNING, ale zůstávají skutečnými provozními riziky.
Nejde o schválení letového UTB řízení, UTB_ACRO ani FÁZE 2.

## 1. Hardware, firmware a oprávnění

| Veličina | Skutečně ověřená hodnota |
|---|---|
| FC | Skystars H7 Dual Gyro, STM32H743/753, revision V |
| Board / bootloader | SkystarsH7HD-bdshot, ID 1075, protocol 5 |
| USB | VID 0x1209, PID 0x5741, serial 200024001251303433333336 |
| Port | COM5, ověřená identity a výhradní otevření |
| Source firmware | f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f |
| APJ / FC identity | f9cc3bcc, Copter 4.7.1 |
| Build | UTB ON, BENCH ON, logger ON, default B_SYNTH=0 |
| APJ velikost / image | 1129203 / 1251892 B |
| APJ SHA-256 | a45f3c9f08e21e8b713cfe59e3fe404fe4e866164c6c21f1a00edd0876622878 |

Použit pouze archivovaný APJ:
`tmp/utb-synthetic-reference-20261012_085037/SkystarsH7HD-bdshot-f9cc3bcc0f-UTB_ON-BENCH_ON-SYNTH_OFF/arducopter.apj`.
Uploader ověřil board a image CRC, bez force. Nový firmware byl potvrzen
AUTOPILOT_VERSION i MSG ve skutečném BIN. C++ se během hardware fáze
neměnil a neprováděl se nový build.

Před prvním zápisem bylo potvrzeno: všechny vrtule fyzicky sundané,
DISARMED, receiver odpojený, bezpečný test a volný COM5.
Mission Planner zůstával jako proces, ale port nebyl obsazený; připojení
se otevřelo výhradně. Totožnost nebyla odvozena jen z názvu COM portu.

Původní FS_THR_ENABLE=1 a FS_EKF_ACTION=1 se neměnily. Původní
FS_GCS_ENABLE=0 vyžádalo samostatné schválení dočasné hodnoty 1
s následným obnovením 0. Při měření byl GCS failsafe zapnutý a host
udržoval skutečný 1Hz heartbeat. Žádný failsafe ani health guard se
kvůli testu nevypnul. Přesná potvrzení jsou v safety-confirmation.json,
gcs-failsafe-approval.json a dvou coldboot-confirmation souborech.

## 2. Archivace a časová posloupnost

Nový session adresář, předchozí výsledky nepřepsány:
`tmp/utb-synthetic-h743-20261012_150527/`.

1. Read-only preflight: původní firmware acf04efa, frame 1/18, 400 Hz,
   BLOCK backend 4, SHADOW=0, LOG_DISARMED=0. RC_CHANNELS má chancount=0.
   Kompletní záloha původních 1210 parametrů v parameters-before.json
   a parameters-before.param; GCS heartbeat a read/request operace v tlog.
2. Původní log 1 archivován celý: **7654746 B**, SHA-256
   `ed08afb62c8dc575353dda96dfe649655178f9d88aca9ea2bfef4f1307a533be`.
   Shoduje se s historickým PACKED_CD_FIXED.bin. LOG_ERASE se neodeslal.
3. Kapacita ověřena z log directory, W25Q128/source a page overheadu.
   16MiB médium, 256B page, 250B payload; poslední 64KiB blok rezervuje
   logger format. Log 1 používá přibližně 7838464 B fyzických pages.
   Plán ponechal dvě další 64KiB zaokrouhlovací rezervy a měl přibližně
   8742144 B prostoru pro nový provoz. Odhad z historického 119kB/s
   packed toku není přímý flash allocator counter ani bezpečnostní limit.
   Přesný plán/předpoklady: flash-capacity-plan.json. Nevznikl chip-full stav.
4. Schválená dočasná FS_GCS_ENABLE=1 a standardní reboot do bootloaderu;
   nahrání přesně uvedeného APJ a CRC verify. Uploader „Erase“ je
   programová flash přepisu firmware; nejedná se o vymazání DataFlash logů.
5. První post-boot list neměl osm MAV1 parametrů; host validační krok
   zastavil přípravu a provedl cleanup. Samostatné name reads všech osm
   hodnot potvrdily jako původní nuly; pozdější kompletní readback má 1211
   parametrů. Jediný nový parametr je B_SYNTH=0. Nešlo o ztrátu konfigurace.
   První neúspěšný validation JSON zůstal zachovaný.
6. Krátký běh: skutečný STATE_VALID při logging suppression, gains,
   15s logger warmup, přibližně 1s PREROLL + 5s RUNNING. Poté zdroj,
   SHADOW a logování vypnuty; dočasné parametry obnoveny explicitním čtením.
7. Následný download krátkého logu měl Windows serial ClearCommError a
   „Device COM5 is dead“. Spojení bylo obnovitelné, uptime pokračoval a
   STAT_BOOTCNT zůstal 72. Část LOG_DATA byla zachráněna z tlog, zbytek
   stejného neměnného logu dočten po 8190 B s 20ms host pacing.
   Krátký experiment se neopakoval. Přesná USB/driver příčina je UNVERIFIED.
8. Offline krátký test PASS, ale příprava plného běhu po přenosu zachytila
   EKF_STATUS_REPORT flags=1024. Zastavila se **před prvním PARAM_SET**,
   žádná plná sekvence nebyla tehdy spuštěna. Po výslovně potvrzeném
   úplném odpojení napájení se obnovil zdravý EKF flags=167 a DISARMED.
9. Nový state guard, 15s warmup a jedna plná sekvence profilu 1:
   400 PREROLL + 8000 RUNNING cyklů; COMPLETE. Bez přenosu/listingu logů
   v měřicím okně. Následoval freeze a obnovení parametrů.
10. Celý log 3 stažen s pacingem, velikost a SHA ověřeny. Po tomto přenosu
    se opět objevil EKF UNINITIALIZED, bez neplánovaného rebootu.
    Žádný další benchmark se nespustil. Závěrečný fyzický start je
    samostatně potvrzený a následuje už jen read-only ověření.

Tlogy zachovávají oddělené setup, capture a download fáze. Host nic
nearmoval, neprováděl motor test, neposílal RC override ani flight-mode
změnu, nesmazal logy a neprovedl push/release.

## 3. Skutečná konfigurace měření

Frame 1/18, SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0, AP mode STABILIZE (0),
DISARMED, receiver nepřítomný. UTB_ENABLE=1 od bootu, SHADOW=1 jen při
přípravě/běhu, LOG_FMT=1, LOG_RATE=400, B_SYNTH=1 pouze pro konečnou
sekvenci. LOG_BACKEND_TYPE=4, LOG_BITMASK=180222, LOG_FILE_BUFSIZE=172,
LOG_BLK_RATEMAX=0, LOG_DARM_RATEMAX=0 zůstaly původní.

Gains všech tří os, potvrzené name readbackem i PARM v BIN:
P=0,31, I=0,23, D=0,004, IMAX=0,19, D_HZ=20 Hz.
Přesné názvy: UTB_RAT_RLL_*, UTB_RAT_PIT_*, UTB_RAT_YAW_* se suffixy
P/I/D/IMAX/D_HZ. Výchozí firmware gains se nezměnily; tyto hodnoty jsou
výhradně diagnostické, nikoli letový tune. Profil 2 nebyl na H743 použit.

STATE_VALID nepochází pouze z EKF telemetrie: actual_state_gate v obou
capture.json má flags=7, calc=0, obs=2, policy=1 při LOGGER OFF.
Je to skutečná AP_UTB kontrola AHRS initialised/healthy, primárního gyro,
kvaternionu, post-sampling age a nového INS timestampu.
Ve všech uložených RUNNING sadách je controller i mixer validní a RCms=0.
Nenulový reference zdroj tedy není přítomnost fingovaného receiveru.

## 4. Generátor a 400Hz logger – H743 VERIFIED

| Veličina | Krátký test | Plná sekvence po potvrzeném cold bootu |
|---|---:|---:|
| RUNNING platné uložené sady | 2011 | **8000/8000** |
| Cycle rozsah | 0…2010 | **0…7999** |
| Epoch | 1, samostatný boot | 1, samostatný boot |
| Seq sousední kroky | 2010×1 | **7999×1** |
| Interior uložená frekvence | 399,999204 Hz | **399,999960 Hz** |
| Platný controller / mixer | Všechny sady | Všechny sady |
| Reference fáze | 0,1,2 | **0…9** |
| TIME_BUDGET | Žádný | Žádný |
| Fyzický AP RCOU 1–4 | Všechny 1000 | Všechny 1000 |

Plná sekvence, skutečné TimeUS/Seq/UTBX:

- PREROLL začátek: TimeUS=197037671, Seq=6484, Test=5.
- RUNNING Cycle=0: TimeUS=198037670, Seq=6884, Test=2.
- RUNNING Cycle=7999: TimeUS=218035172, Seq=14883.
- COMPLETE Cycle=8000: TimeUS=218037671, Seq=14884, Test=3.
- PREROLL má 400 nulových cyklů; časově 999999 µs.
- RUNNING interval **[198037670,218037671)** má **20000001 µs**,
  tedy 8000 sad / 20,000001 s = **399,999980 Hz**.

Interior frekvence používá (N−1)/(last−first), nikoli jiné časové okno.
COMPLETE není časová estimace hostu; je explicitně v fyzickém BIN.
Epoch se v RUNNING nezměnil, není ABORTED, duplicita ani Seq mezera.
UTBC ID31/Length242 a UTBX ID23/Length27 odpovídají skutečnému FMT.
V celém plném logu bylo bitově decode/repack prověřeno 14429 UTBC.

WAITING (Test=1) nebyl jako samostatný fyzický snapshot zaznamenán:
při splněné způsobilosti proběhne OFF→WAITING→PREROLL v jednom update.
Tento mezikrok je **SOURCE VERIFIED**; fyzicky uložené stavy jsou
PREROLL→RUNNING→COMPLETE. Samostatné trvání WAITING je zde UNVERIFIED.
Kvůli jeho zviditelnění se validity nezhoršovala ani neměnil firmware.

## 5. PID a mixer proti nezávislému modelu

**H743 VERIFIED:** analytické reference všech uložených vzorků a 7999
navazujících PID/mixer recurrence dvojic prošly existující odvozené
float32 meze; žádná tolerance se nezvětšovala podle hardware výsledku.
Krátký test má dalších 2010 ověřených dvojic. Gains byly zvlášť ověřeny
z PARM, nepředpokládá se jejich správnost jen podle názvu souboru.

| Komponenta, maximum přes osy | Absolutní H743 − model odchylka |
|---|---:|
| Error | 1,49012e−8 rad/s |
| P | 8,32599e−9 |
| I | 7,42740e−9 |
| D | 4,24193e−10 |
| Raw / Output | 3,50096e−8 |
| Diagnostický motor m_i | 5,14219e−8 |
| Achieved R/P/Y | 8,82896e−8 |

Model a tolerance jsou v utb_math_reference.py a ve specifikaci
UTB_PID_MIXER_VALIDATION_CS.md. Recurrence se inicializuje z předchozího
skutečného snapshotu, včetně I a D historie; nejde o model fyzické
uzavřené smyčky nebo o důkaz pro všechny možné float32 vstupy.
Reference sinusů se porovnává s nezávislými rovnicemi; gyro zůstává skutečné.

P mění znaménko s referencí, I se integruje a roll dosáhne přesné float32
IMAX=0,189999997616. Roll I minimum je −0,0862933, pitch maximum 0,136775,
yaw maximum 0,0422346; všechny osy respektují ±IMAX. Menší pitch/yaw I
je očekávaná délka/průběh buzení. Output=Raw v tomto nesaturovaném okně.

D je nenulové ze skutečné gyro derivace: roll rozsah −0,00288982 až
+0,00480923, pitch −0,00232877 až +0,00229576, yaw −0,00201025 až
+0,00239460. Filtr a znaménko odpovídají modelu; skok Desired se
nepovažuje za důkaz D kick. Gyro measurement nebylo synteticky injektováno.

Mixer zůstal v lineární oblasti, bez společného scale či collective
shiftu; integration gate count=0. **Directional anti-windup při momentové
saturaci není v této malé fyzické sekvenci H743 VERIFIED.** Je
SOURCE/UNIT/SITL VERIFIED v oddělených silnějších profilech. Amplituda
se na FC kvůli vynucení saturace nezvyšovala. Vše zůstává SHADOW ONLY.

### Grafy skutečně uložených H743 dat

Všechny 8000 vzorky jsou zachované, bez downsamplingu a syntetických
measurement. Normalizované mixer m_i nejsou PWM ani DShot povely.
Časy jsou relativní k prvnímu RUNNING vzorku; CSV obsahuje Epoch, Seq,
Cycle, dt a všechny kreslené složky.

![Desired, skutečné Measured a Error](../tmp/utb-synthetic-h743-20261012_150527/analysis-full-final/references.svg)

![P, I, D a Output](../tmp/utb-synthetic-h743-20261012_150527/analysis-full-final/pid.svg)

![Detail D ze skutečného gyra](../tmp/utb-synthetic-h743-20261012_150527/analysis-full-final/d-term.svg)

![Diagnostické command/achieved a m1 až m4](../tmp/utb-synthetic-h743-20261012_150527/analysis-full-final/mixer.svg)

## 6. Skutečné časování H743

Použity hardware AP_HAL monotónní časy a existující BENCH metriky,
nikoli virtuální SITL hodiny. Průměry UBMT níže jsou ΔSum/ΔN mezi
endpointy **199009248 a 218034176 µs** (19,024928 s), uvnitř RUNNING.
Min/Max UBMT jsou naopak kumulativní od tohoto bootu. Nelze z nich
vyrobit izolované maximum právě 20s sekvence.

| Metrika | Skutečný průměr | Maximum / význam |
|---|---:|---|
| Main period, UBMT | 2500,000788 µs | Kumulativní 6011 µs ze startupu |
| Main absolutní jitter, UBMT | 2,693561 µs | Kumulativní 3511 µs ze startupu |
| Každé logované scheduler dt | 2499,999952 µs | Min 2456,000 / max 2548,000 µs v RUNNING |
| Absolutní jitter logovaného dt | 2,691531 µs | Všech 8000 dt |
| Shadow TimeUS sousední perioda | 2500,000250 µs | Min 2452 / max 2552 µs |
| Shadow TimeUS absolutní jitter | 5,066883 µs | Jiné místo časového odečtu než scheduler |
| Producer wall time | **38,117871 µs** | **50 µs**, kumulativní |
| Consumer invocation wall time | 17,977465 µs | 1633 µs, kumulativní |
| Worker pass wall time | 39,009078 µs | 1734 µs, kumulativní |
| Main logger acquire | **0,491511 µs** | **59 µs**, kumulativní |
| Worker logger acquire | 0,721643 µs | 106 µs, kumulativní |
| Worker write_sem hold | **4,589570 µs** | **630 µs**, kumulativní |
| Export pass | 692,421053 µs | 1498 µs, kumulativní |
| UTBT.Ex | 29,783250 µs | 27…40 µs v RUNNING |
| PM.Load | **20,0 % / 19,8 %** | Dva reporty v RUNNING |
| PM.NLon / MaxT | **0 / 2548 µs**, **0 / 2534 µs** | Reporty shrnují své předchozí 10s intervaly |

Producer zahrnuje sampling, guards, generátor, PID/mixer i publikaci.
UTBT.Ex je další wall-time rozsah části produceru; **není čistá doba PID**.
Izolovaný controller-only čas není měřený a zůstává UNVERIFIED.
50 µs odpovídá 2 % nominální periody, ale není WCET důkaz; původní
125µs/3-consecutive TIME_BUDGET policy se neměnila a nezalatchovala.

UBST[8] delta **232** = task-budget overruns, přibližně **12,195/s**
v uvedeném counter intervalu. UBST[9] delta **2528** = periody nad
nominálem, přibližně 33,22 % jeho 7610 cyklů. Tyto čítače nejsou počtem
ztracených regulačních cyklů. PM.NLon=0 ani průměr 2500 µs nedokazují
všechny hard deadline a nejhorší možné zátěže.

Worker hold 630 µs byl dosažen už před RUNNING: live baseline i final
jej mají stejný. Podobně 6011µs main maximum je startup, nikoli nová
20s regulační pauza. Nelze přesto tvrdit, že jsou tato maxima bezpečně
omezená nebo že nulová confirmed collision znamená nulové blocking.

## 7. Logger, stack a lifecycle

| Veličina v RUNNING | Výsledek |
|---|---|
| Queue Drop | 402→402, delta 0; stejné ve všech 8000 UTBC |
| Backend Missing | 130→130, delta 0; stejné ve všech 8000 UTBC |
| DSF.Dp | 270 ve všech dochovaných RUNNING samples, delta 0 |
| Gone / create failures | 0 / 0, bez přírůstku |
| Worker / exporter life | RUNNING (3) / RUNNING (3) |
| Worker progress | +15643 passes v counter intervalu |
| Queue HWM / endpoint depth | Kumulativní 4 / 0 |
| Worker stack minimum / actual region | **1760 / 3504 B**, requested 3072 B |
| Confirmed MAIN/worker collisions | 0 |
| UNKNOWN attribution | +9365; nelze považovat za nulové blocking |
| Nejnižší zachycené DSF.FMn | **38707 B** |
| Celkový uložený logical payload | **118391,094 B/s** v COMPLETE ohraničeném okně |
| Z toho UTBC | 96799,995 B/s |

Written je backend admission, nikoli perzistence; rozhodující počet
8000 byl prověřen v celém staženém BIN. Payload rate počítá délky
skutečných FMT zpráv v časovém okně, bez fyzických page headers/SPI
příkazů. DSF buffer minimum je vzorkované, ne kontinuální IO watermark.

Watermark je skutečná volná stack oblast podle existujícího ChibiOS API.
1760 B je o 24 B méně než historických 1784 B; nejde o důkaz rezervy
pro každý budoucí scénář. Nebyla nová stack warning, create failure ani
snížení minima během RUNNING. Mechanismus a warning threshold se neměnily.

**Celá session není bezeztrátová.** Pre-health probe při SHADOW=1 a
LOGGER OFF plní diagnostickou queue; producer přitom stále volí samples,
worker je bez dostupného loggeru nedrainuje. SOURCE VERIFIED vysvětluje
startup overflow kolem 400 sad při zhruba 1s probe. Další logger-start
admission/FMT události nelze z těchto agregátů přiřadit každému odmítnutí.
Ve full bootu vzniklo Drop=402, Miss=130 a DSF.Dp=270 před RUNNING.
První PM má NLon=1 / MaxT=6011 µs. Krátký boot měl Drop=404, Miss=96,
PM startup MaxT=6546 µs. 15s warmup tyto události oddělí, neodstraňuje je.

## 8. Srovnání s historickým zero-gain D

| Metrika | Historické D s RC, zero gains | Nový no-RC profil 1 |
|---|---:|---:|
| Producer mean / cumulative max | 39,751092 / 52 µs | 38,117871 / 50 µs |
| PM.Load | 21,2–21,3 % | 19,8–20,0 % |
| Main jitter mean | 2,367495 µs | 2,693561 µs |
| Main acquire mean / max | 0,513929 / 74 µs | 0,491511 / 59 µs |
| Worker hold mean / max | 4,385351 / 253 µs | 4,589570 / 630 µs |
| Stack free min | 1784 B | 1760 B |
| Stored full sets | 8000 | 8000 |
| Logical payload | 118828,90 B/s | 118391,094 B/s |

Srovnání má stejné frame/400Hz/backend/mask/packed schéma, ale historické
D mělo RC, AP ALT_HOLD (2), jiný source a zero gains. Nový běh je
STABILIZE bez receiveru; obejde pouze diagnostické RC mapping výpočty.
Průměry mají různé endpointy, maxima různý startup. **Nelze tvrdit záporný
izolovaný overhead nenulového PID** podle nižšího load/produceru.
Jitter vzrostl asi o 0,326 µs, mean hold asi o 0,204 µs, startup hold
maximum o 377 µs. Tyto rozdíly se nezaměňují za libovolný safety limit
ani důkaz konkrétní interní mutex příčiny. Přímé A/B stejného no-RC/AP
režimu, worst-case a dlouhodobé ověření zůstávají otevřené.

## 9. Bezpečnost a hranice závěru

- **SOURCE VERIFIED:** C++/HAL/arming/RateController/MotorMixer/Reference/
  State provider se proti testovanému f9cc3bcc neměnily. Nové soubory
  jsou host capture/test a dokumentace. Žádná cesta UTB→AP_Motors/HAL/
  DShot setter, žádné oslabení FSTRATE/frame/AHRS/gyro/age guardů.
- **H743 VERIFIED:** skutečné validní senzorové snapshots, no-RC timestamp,
  všechny PID/mixer komponenty, konečný generator a lossless RUNNING.
- **H743 VERIFIED:** žádný ARMED heartbeat, žádný host arming/motor-test/
  RC-override/mode příkaz. V plném BIN je 364 RCOU zpráv a všechny
  C1–C4=(1000,1000,1000,1000); krátký BIN má 213 stejných zpráv.
- **SOURCE/UNIT/SITL VERIFIED:** původní UTB_ACRO arming zákazy zachovány.
  Na skutečném FC se arm rejection záměrně netestovalo ARM příkazem.
- **UNIT VERIFIED:** 4 nové host guard testy, všech 27 Python regresí a
  flake8 PASS. Původní 67 C++ a SITL testy jsou ověření source f9cc;
  během této host-only fáze se znovu nekompilovaly ani nereinterpretují jako HW.
- **UNVERIFIED:** fyzické ESC signály osciloskopem, uzavřená UTB smyčka,
  saturující fyzický allocator/anti-windup, WCET/nejhorší mutex latence,
  dlouhodobý provoz/zaplněná flash a globální absence negativního AP dopadu.

Download je prokazatelně oddělený od sekvence. Přesto jeho následky
nelze ignorovat: ClearCommError v prvním transferu a EKF flags=1024 po
obou dokončených přenosech. Ani 20ms host pacing EKF incident nevyloučilo.
Časová souvislost je H743 VERIFIED; přesná příčina v AP logger/USB/EKF
je **UNVERIFIED**. Předchozí reporty stejnou třídu problémů již popisují.
Firmware, priority a logger policy se kvůli tomu neupravovaly.

## 10. Finální stav a parameter diff

Dočasně se zapisovalo pouze 15 UTB gains, B_SYNTH/SHADOW/LOG_FMT/LOG_RATE,
LOG_DISARMED a samostatně schválený FS_GCS_ENABLE. Každý zápis měl
PARAM_VALUE potvrzení a následný PARAM_REQUEST_READ. Cleanup při úspěchu
i zastavené přípravě zneplatnil zdroj, vypnul SHADOW/logger a obnovil gains.

Readback po měření má všech 1211 jmen, UTB_ENABLE=1, B_SYNTH=0,
SHADOW=0, LOG_DISARMED=0, LOG_FMT=1, LOG_RATE=400; všech 15 gains je
původních 0, FS_THR_ENABLE=1, FS_GCS_ENABLE zpět původní 0, frame
1/18, FSTRATE=0 a všechny ostatní měněné hodnoty souhlasí se zálohou.
Obnovení GCS=0 je výslovně schválený návrat do původní konfigurace,
ne vypnutí ochrany během experimentu.

Automatické gyro offsets/calibration temperatures po potvrzených
startech, COMPASS_AUTODEC deklinace, BARO1_GND_PRESS a STAT počítadla
se liší. Tyto hodnoty nastavuje původní AP startup/kalibrace; host je
nezapisoval a nevracel zastaralé kalibrace. Přesné before/after hodnoty
jsou v final-readback.json a následném final-coldboot-readback.json.
Nový trvalý parametr je UTB_B_SYNTH=0. Žádný dočasný gain nezůstal aktivní.

**H743 VERIFIED – konečný read-only stav po druhém potvrzeném fyzickém
startu:** firmware f9cc3bcc, board 1075, všech 1211 parametrů znovu
načteno, DISARMED, EKF_STATUS_REPORT flags=167, B_SYNTH=0/SHADOW=0/
LOG_DISARMED=0, všech 15 gains původních 0 a GCS původní 0.
STAT_BOOTCNT=74 odpovídá upload restartu a dvěma potvrzeným fyzickým
startům (původně 71); během sekvencí ani transfer recovery nešlo o
neplánovaný reboot. Závěrečný port byl zavřený a neproběhl další test.
Logy 1–3 zůstaly na FC; součet logical size je 14573488 B.
Odhad zbývajícího prostoru po page overheadu a dvou blokových rezervách
je 1657344 B. Další plný 400Hz test se do tohoto odhadu nevejde;
nebyl spuštěn žádný erase ani další experiment.
PreArm „RC not found“ a „Battery below minimum arming voltage“ jsou
zachované podmínky USB/no-RC bench; žádná arming ochrana se nevypínala.
To není potvrzení připravenosti k letu.

## 11. Artefakty a reprodukce offline

| Soubor v session | Velikost B | SHA-256 |
|---|---:|---|
| before-upload-log-1.bin | 7654746 | ed08afb62c8dc575353dda96dfe649655178f9d88aca9ea2bfef4f1307a533be |
| SYNTHETIC_SHORT.bin | 2569496 | d45167bd5832efb08cf2b0350fd1a9cd29388a48828aa4ffa0b2bd0bc63f040e |
| SYNTHETIC_FULL_400.bin | 4349246 | 0677617bfac02af3f3b44d0038bb614ed147e1eeb3812d0b96b80d6e565786b6 |

Surové tlogy jsou preflight/upload/validation/coldboot, short/capture.tlog,
full/capture.tlog (zastavená příprava, žádné zápisy),
full-after-coldboot/capture.tlog a oddělené download/final-readback tlogy.
Manifest obsahuje velikosti a SHA všech souborů včetně grafů, CSV a
one-off host procedur. Starší raw/failed JSON se nepřepisovaly.

Offline numerické ověření:

```sh
.venv/bin/python -B Tools/autotest/utb_synthetic_log_analysis.py \
  "$PWD/tmp/utb-synthetic-h743-20261012_150527/SYNTHETIC_FULL_400.bin" \
  --output "$PWD/tmp/utb-synthetic-h743-20261012_150527/novy-offline-report.json"
```

full-offline.json/log: model a bitový roundtrip; analysis-full-final/
analysis.json a signals.csv: přesné okno, časování/counters a kreslená
data. analyse_recording.py: reprodukovatelná per-session offline procedura.
make_plots.py: standardní ReportLab LinePlot, existující Windows runtime,
SVG všech vzorků. Balíčky se neinstalovaly.

### Přesný manifest změn v repozitáři

| Soubor | Zásah |
|---|---|
| Tools/autotest/utb_synthetic_hardware_capture.py | Nový explicitní no-RC BENCH-only host kontrakt; identity, whitelist writes/readback, skutečný state guard, finite run, stop/cleanup; žádný transfer/upload/reboot/ARM |
| Tools/autotest/unittest/test_utb_synthetic_hardware_capture.py | Čtyři čistě host testy čerstvosti/health, completion candidate, zakázaného zápisu a RC/EKF stop; žádné spojení |
| docs/UTB_SYNTHETIC_H743_VALIDATION_CS.md | Tento fyzický report, grafy, raw identita a omezení |
| docs/UTB_SYNTHETIC_REFERENCE_BENCH_CS.md | Označení původní softwarové přípravy jako historické a odkaz na fyzické výsledky |

utb_hardware_capture.py a jeho původní RC safety guard se neměnily.
Firmware commit zůstává f9cc3bcc; nové lokální host/report commity
nemění identitu skutečně nahraného APJ. Žádný push/release nevzniká.

## 12. Jednoznačný závěr

**Ano:** skutečný Skystars H743 bez RC provedl celou 400Hz interně
referencovanou sekvenci s nenulovými gains, matematicky správným PID
včetně skutečné D derivace a mixerem, 8000 uloženými platnými sadami,
bez nových ztrát nebo PM dlouhých smyček v RUNNING a při zachované
source motorové izolaci. Jde o jednu konečnou DISARMED SHADOW session.

**UNVERIFIED pro globální „bez negativního dopadu“:** startup a download
nejsou stejně bezproblémové jako RUNNING. EKF po transferu, dlouhé
startup periody, vysoké kumulativní hold a omezený stack/worst-case
důkaz vyžadují oddělený audit před širším použitím. V této fázi se
nic z toho neopravovalo oslabením ochrany nebo novou regulací.
Aktivní UTB_ACRO, motorová autorita a FÁZE 2 zůstávají zakázané.
