# FÁZE 1H-C – Interní reference pro DISARMED SHADOW bez RC

Stav softwarové přípravy k 12. 10. 2026. Větev `test/skystars-5inch`.
Tento dokument zachovává výsledky před fyzickým testem. Následné
H743 ověření nenulového PID a interní reference je samostatně doložené
v [hardware reportu](UTB_SYNTHETIC_H743_VALIDATION_CS.md), včetně
startup/download omezení. Níže uvedené hardware neověřeno platí pro
okamžik dokončení této přípravy.
Firmware a C++/SITL výsledky odpovídají source commitu
`f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f`.
Následující dokumentační commit nemění firmware.

Interní reference je **implementována a softwarově ověřena**. Nový firmware
**nebyl nahrán na FC**. Nenulové gains, interní reference a jejich časování
na skutečném H743 jsou **hardware neověřeno**. Aktivní UTB_ACRO a FÁZE 2
nejsou součástí této práce.

## 1. Závislost na RC a původní bezpečnostní politika

Původní most v `ArduCopter/utb.cpp:71–96` kontroluje kalibraci RC,
`g2.rc_channels.has_valid_input()` a `!failsafe.radio`. Bez receiveru je
`reference_valid=false`; zdravý senzorový stav sám nestačí k výpočtu PID.
`AP_UTB::evaluate()` v `libraries/AP_UTB/AP_UTB.cpp:313–343` odděluje
STATE_INVALID od REFERENCE_INVALID. RateController ani MotorMixer receiver
nevyžadují. `AP_UTB_Reference.*` obsahuje statickou mapu vstupů.

AHRS health není přítomnost RC: `libraries/AP_AHRS/AP_AHRS.cpp:2298`
kontroluje nakonfigurovaný estimator a `:2431` jeho inicializaci.
`AP_AHRS_NAVEKF_SETTLE_TIME_MS` v `AP_AHRS.h:40` je **20000 ms**,
i když starší komentář u initialised uvádí 10 s. Platnost skutečného gyro,
kvaternionu a časů nadále kontroluje `AP_UTB_State.cpp:19–46`.
Žádná hodnota validity, timestamp ani gyro se synteticky nepřepisuje.

`ArduCopter/radio.cpp:115–132` nevstupuje do radio timeout failsafe,
pokud nikdy nebyl přijat RC vstup a FC je DISARMED. Jakmile RC někdy
přijat byl nebo je systém ARMED, platí původní radio failsafe.
`ArduCopter/events.cpp:13` nadále provádí původní failsafe akce.
To umožňuje test od čistého startu bez receiveru při zapnutém failsafe;
nejde o jeho vypnutí. Odpojení dříve připojeného receiveru za běhu může
vyvolat legitimní failsafe a test přeruší.

Nový BENCH guard pouze čte `any_failsafe_triggered()`
(`ArduCopter/Copter.h:440`) a vyžaduje `FS_THR_ENABLE != 0`.
Radio, battery, GCS, EKF, terrain, ADSB nebo dead-reckoning failsafe
znamená nezpůsobilost testu. Původní AP mód dál zpracovává vlastní
RC/failsafe vstupy. Nepřipojené RC tedy ovlivňuje běžný AP režim podle
jeho původní politiky; oddělení UTB reference tuto politiku nemění.

## 2. Implementovaný generátor

`AP_UTB_SyntheticReference.*` je dostupný pouze při
`AP_UTB_ENABLED && AP_UTB_BENCH_ENABLED`. BENCH má nadále default 0.
Nový parametr `UTB_B_SYNTH`, index 22, default **0**, je dostupný jen
v BENCH buildu. Existující indexy ani defaulty gains se nemění.

| UTB_B_SYNTH | Význam | Dostupnost |
|---|---|---|
| 0 | Původní AP-derived RC reference | Normální cesta |
| 1 | Interní konečná sekvence, amplituda 0,3 rad/s | BENCH, DISARMED |
| 2 | Sekvence 8 rad/s pro diagnostickou saturaci | Pouze SITL; na H743 odmítnuta |

Profil 1 není letové nastavení. Diagnostický collective je 0,5 při
platné referenci, jinak 0. Je to vstup matematického mixeru bez vazby
na fyzické motory.

Aktivace vyžaduje boot-enabled UTB, SHADOW, DISARMED, nepřítomnost
watchdog-reset flagu/failsafe, skutečný platný stav, nový INS timestamp,
logger availability, FRAME_CLASS=1 a FRAME_TYPE=12 nebo 18, vypnutý
FSTRATE, dostupnou ACRO mapu, nominal_dt=1/400 s a původní validní
rozsah skutečného dt. Stáří skutečných dat se kontroluje proti samostatnému
post-sampling času; limit při 400 Hz zůstává 10000 µs.

| Test v UTBX | Stav | Chování |
|---|---|---|
| 0 | OFF | Generátor vymazán, návrat do původní RC validace |
| 1 | WAITING | Čekání na skutečnou způsobilost, reference neplatná |
| 5 | PREROLL | 400 nulových cyklů, priming/reset historie před sekvencí |
| 2 | RUNNING | Přesně 8000 cyklů, Cycle=0…7999 |
| 3 | COMPLETE | Nulová neplatná reference; zůstává vypnutý |
| 4 | ABORTED | Nulová neplatná reference; zůstává vypnutý |

ARMED, watchdog, ztráta způsobilosti po startu, změna AP módu nebo
změna aktivního profilu přeruší i PREROLL. Controller/mixer chyba či
původní TIME_BUDGET policy přeruší sekvenci. Běžné počáteční PRIMING
je povoleno v nulovém preroll. Pokud se TIME_BUDGET latch nastaví během
právě dokončovaného cyklu, jeho snapshot je zneplatněn a historie
controlleru resetována už v tomto cyklu.

COMPLETE/ABORTED se po obnově health **automaticky nerozběhne**.
Restart vyžaduje `UTB_B_SYNTH=0` a teprve další nenulový požadavek.
Samotné SHADOW OFF/ON chybu neodblokuje. Změna zdroje resetuje PID
historii a mění Epoch. Vypnutí generátoru nepovažuje neexistující RC
za platnou referenci.

## 3. Přesné signály

Časování je definováno počtem cyklů, nikoli příslibem wall-time deadline.
Nejdříve 400 nulových cyklů PREROLL, nominálně 1 s. V tabulce je čas
měřen od prvního RUNNING cyklu. Celá aktivace je nominálně 21 s.
`n` je RUNNING Cycle, `A=0.3f` (float32 přibližně 0,30000001192).
Profil 2 používá `A=8.0f` pouze v SITL.

| Phase | n, interval včetně počátku | Nominální čas [s) | (roll, pitch, yaw) [rad/s] |
|---|---|---|---|
| 0 | 0…199 | 0…0,5 | (0, 0, 0) |
| 1 | 200…1799 | 0,5…4,5 | (+A, 0, 0) |
| 2 | 1800…3399 | 4,5…8,5 | (−A, 0, 0) |
| 3 | 3400…4199 | 8,5…10,5 | (0, +A, 0) |
| 4 | 4200…4999 | 10,5…12,5 | (0, −A, 0) |
| 5 | 5000…5799 | 12,5…14,5 | (A sin θ, 0, 0) |
| 6 | 5800…6599 | 14,5…16,5 | (0, 0, A sin θ) |
| 7 | 6600…7399 | 16,5…18,5 | (A sin θ, A cos θ, A sin 2θ) |
| 8 | 7400…7799 | 18,5…19,5 | (0, 0, 0) |
| 9 | 7800…7999 | 19,5…20 | (0, 0, 0) |

V každé sinusové části `θ=2π(n−n_start)/800`, kde n_start je počátek
části. Základní frekvence je nominálně 0,5 Hz, kombinovaná yaw 1 Hz.
Skutečná frekvence závisí na měřené periodě smyčky. PID dostává skutečné
dt, nikoli cyklový čas generátoru. Na konci je reference neplatná a PID
se resetuje; nulový konec sekvence nemá neomezené pokračování.

## 4. Dva oddělené způsoby měření

**A – reálné gyro a interní reference:** cesta pro budoucí H743 test
sampluje původní AHRS/INS, skutečné timestampy a skutečné dt. Sekvence
mění pouze diagnostickou Desired a collective. Na nepohybujícím se FC
může být D téměř nulové: D je derivace measurement, nikoli skoku Desired.
SITL varianta A používá gyro poskytované původním simulovaným INS;
není to fyzický H743 senzor.

**B – syntetická reference a syntetické measurement:** samostatný
`test_utb_math` runner dostává 8000 matematických vstupů ze souboru.
Roll measurement je `0.12 sin(2πt+0.7)+ε`, s deterministickým šumem
ε v ±0,002 rad/s, seed 471. Pitch je rampa `0.06t` do cyklu 3999,
potom konstanta 0,6; yaw impuls −0,1 v cyklech 3000…3199.
`t=n/400`, dt má deterministický jitter ±24 µs; cyklus 6000 vyvolá
invalidation/reset. Runner používá skutečné třídy RateController a
MotorMixer a oddělený nezávislý Python model.

Tato measurement nejsou vstupem AHRS, EKF, INS, běžného AP controlleru
ani živého shadow state provideru. Varianta B není dostupná na FC.
Profil 2 v SITL mění referenci, nikoli gyro; není zaměnitelný s variantou B.

Testovací gains všech tří os jsou následující; defaulty v firmware
zůstávají původní. Parametry jsou ověřeny v AP_Param source i PARM logu.

| Parametry | Hodnota |
|---|---|
| UTB_RAT_RLL_P, UTB_RAT_PIT_P, UTB_RAT_YAW_P | 0,31 |
| UTB_RAT_RLL_I, UTB_RAT_PIT_I, UTB_RAT_YAW_I | 0,23 |
| UTB_RAT_RLL_D, UTB_RAT_PIT_D, UTB_RAT_YAW_D | 0,004 |
| UTB_RAT_RLL_IMAX, UTB_RAT_PIT_IMAX, UTB_RAT_YAW_IMAX | 0,19 |
| UTB_RAT_RLL_D_HZ, UTB_RAT_PIT_D_HZ, UTB_RAT_YAW_D_HZ | 20 Hz |

P/I/D, Raw/Output, Desired/Measured/Error, IMAX, integration gate,
command/achieved a mixer saturation jsou nadále v lossless UTBC.
Jednokrokový directional anti-windup je **SHADOW ONLY**; dynamiku
případného skutečného letu by stále řídil ArduPilot. Test není ověřením
UTB integrátoru v uzavřené UTB letové smyčce.

## 5. Logovací identita a overhead

UTBC má stále vehicle ID **31**, velikost **242 B**, původní layout.
Přidána je BENCH-only zpráva **UTBX**, ID **23**, **27 B**,
format `QIIIBBBB`, pole TimeUS/Seq/Epoch/Cycle/Src/Prof/Phase/Test.
Existující vehicle IDs se neposunuly. Skutečný FMT v testovacím BIN
odpovídá těmto hodnotám.

Src=1 znamená interní zdroj požadovaný; jeho platnost určuje Test
společně s controller/state flags. WAITING/ABORTED není platný experiment.
Worker zapisuje UTBX při změně zdroje/profilu/fáze/stavu a přibližně 1Hz
heartbeat. Ze stejného neměnného snapshotu jsou Seq, Epoch, Cycle a
metadata reference. Žádný nový DataFlash zápis není v produceru.

Offline přiřazení v rámci Epoch používá
`Cycle = anchor.Cycle + packet.Seq − anchor.Seq` a ukončí se na explicitní
COMPLETE/ABORTED hranici. Bez UTBX kotvy nelze původ reference určit.
Ztracená koncová metadata nebo chybná konfigurace omezují průkaznost:
samotná úspěšná rekonstrukce nevylučuje logger ztráty ani neověřuje HW původ.
Epochy se nepropojují. PREROLL se nezapočítává do 8000 RUNNING sad.

Packed tok je nominálně 96800 B/s při 400 Hz a 48400 B/s při 200 Hz,
před ostatními AP/BENCH logy. Nové metadata přidávají obvykle desítky B/s;
nejde o garantovaný celkový datový tok. Je třeba měřit celý DSF/flash tok.
UTBX stavové zprávy mohou krátkodobě přidat další zápisy a sdílený
logger mutex risk zůstává.

Generátor provede nanejvýš deset porovnání hranic, pevné výpočty a v
kombinované části dvě sinf a jednu cosf. Nový guard a metadata zvyšují
CPU/RAM/copy overhead; jsou pevné, bez heapu a bez nového čekání.
BENCH OFF generátor, nový parametr i metadata compile-out odstraní.
H743 image narostl proti předchozímu zero-gain source acf04 z 1250104 na
1251892 B, tj. **1788 B**. Toto je flash rozdíl, nikoli odhad CPU nebo
celkové runtime RAM. Velikost BSS nového BENCH ON buildu je 128792 B.

Stávající UB producer wall time zahrnuje sampling, guardy, generátor,
controller, mixer a publikaci. UTBT.Ex také není čistý čas samotného PID.
Consumer wall time zahrnuje běžnou obsluhu diagnostiky. Oddělené maxima
čistého controlleru současná instrumentace nedává. H743 maxima/průměry
produceru, consumeru, mutexu, stacku, jitter a load jsou s novými gains
**UNVERIFIED**. SITL virtuální čas je nenahrazuje.

## 6. Výsledky softwarových testů

Důkazy jsou v
`tmp/utb-synthetic-reference-20261012_085037/validation-committed/`,
`final-timing.json`, `stable-pipeline-sitl.json`, `full-math-regression/`
a příslušných build/test logách ve stejné session.

| Kontrola | Výsledek | Důkaz / omezení |
|---|---|---|
| Původní UTB C++ unit | PASS, 47 | test_utb-committed.log |
| BENCH C++ unit | PASS, 12 | test_utb_bench-committed.log |
| Matematický C++ runner | PASS, 4 | Celý předchozí suite i oddělené measurement |
| Generátor C++ unit | PASS, 4 | Lifecycle, fault latch, body signálů, export 16000 vektorů |
| Python model/decoder/capture/source identity | PASS, 23 | python-final.log; capture testy jsou mocky, žádné FC |
| C++ proti modelu – předchozí suite | PASS | 30946 vzorků, 29 scénářů, 0 failures |
| Generátor proti nezávislým rovnicím | PASS | 16000 vektorů, dvě amplitudy |
| Oddělené measurement proti modelu | PASS | 8000 vzorků, měřicí šum, D filtr, dt jitter, reset |
| No-RC SHADOW a všechny fáze | PASS | SIM_RC_FAIL=1 od bootu; RCms=0; žádné RC override |
| Mode change, EKF invalidation, dokončení/restart | PASS | ABORTED/COMPLETE latch; invalidace neobchází health |
| UTB_ACRO arm rejection | PASS | Mode 40, nucené armování v SITL odmítnuto |
| Motor isolation při interním zdroji | PASS | RCOU 1…4=1000 po celý DISARMED test |
| Běžná RC reference, priming a FÁZE 0/1 regrese | PASS | skeleton-current-source.log a shadow-current-source/results.json |
| H743 nenulový PID a no-RC bench | UNVERIFIED | Nový firmware nebyl nahrán ani spuštěn |

Generátor mění znaménka roll/pitch, budí všechny osy, dosahuje IMAX;
SITL profil 2 vyvolává společné škálování mixeru a integration gate.
No-RC SITL nefalšuje receiver: SIM_RC_FAIL=1 zastaví nativní RC pulsy,
runner posílá pouze skutečný GCS heartbeat. PARM potvrzuje zapnuté
FS_THR_ENABLE=1 a FS_GCS_ENABLE=1. Nevypíná se bezpečnostní kontrola.
Záměrná změna virtuálního AHRS_EKF_TYPE na nedostupný EKF2 prověřuje
zneplatnění, následný návrat na EKF3 neodblokuje přerušený test.

Z 31213 uložených UTBC byly všechny bitově decode/repack ověřeny.
Interním RUNNING oknům patří 12475 platných sad (8475 profil 1 včetně
přerušeného dalšího pokusu, 4000 profil 2). Nezávislá PID/mixer recurrence
byla ověřena na 8485 sousedních platných dvojicích. Přes mezery po 200Hz
decimaci se chybějící integrátorové kroky nepředstírají.

Maximální numerické odchylky vůči modelu: Raw 1,93332e−7, Output
3,76362e−8, jednotlivý motor 1,06126e−7. Tolerance vycházejí z float32
roundoff jednotlivých operací. U sinusových rovnic je maximum 9,67463e−6
pro sadu obou amplitud; bound zahrnuje gamma_6 chybu float32 úhlu,
Lipschitz mez sin/cos a předpoklad jednoho libm/output zaokrouhlení.
Není to naměřená kvalita libm na H743 ani volně zvolená tolerance.

| SITL RUNNING okno | Platné sady | Seq | Uložená frekvence z interior TimeUS |
|---|---|---|---|
| Profil 1, Req=400, Epoch 1 | 8000/8000 | všech 7999 kroků = 1 | 400,01004 Hz |
| Profil 2, Req=200, Epoch 3 | 4000/4000 | 3979×2, 10×1, 10×3 | 199,99668 Hz |
| Profil 1 přerušený změnou módu | 475 | všech 474 kroků = 1 | Není plný experiment |

Frekvence je `(N−1)/(TimeUS_last−TimeUS_first)`. 200Hz výběr používá
původní wall-time decimaci, proto při jitteru nemusí mít všechny kroky 2.
Controller i generátor běží ve všech 400Hz shadow cyklech.
V plném 400Hz RUNNING je logované dt min 2499, max 3332, průměr
2499,937 µs; jsou to časy SITL. Periodické PM zde má NLon 4–5 a MaxT
3332 µs z vlastností simulace. Nejsou to H743 deadline výsledky.

Celá session nemá nulové ztráty: poslední UTBQ má Drop=0, Miss=3,
Gone=5 a HWM=3. Mezi zachycenými UTBQ kotvami uvnitř stabilního
400Hz i 200Hz okna se Drop a Miss nezměnily, DSF.Dp kotvy jsou 0.
Startup/lifecycle události se nesmí přisoudit platnému 20s oknu ani
zaměnit za bezchybnou perzistenci celé session. Požadavek nulových
přírůstků na H743 musí být ověřen vlastním logem a hranicemi okna.

Dodatečná původní RC regrese na stejné f9cc binárce prošla samostatně:
FÁZE 0 ENABLE=0/1, běžné AP módy, normal/forced/RC arming rejection a
odmítnutí vstupu do UTB_ACRO z ARMED. FÁZE 1 ověřila frame/FSTRATE/
config guardy, logger suppression/recovery, změny módu, priming při
změně primary gyro 0→1→0 a SHADOW restart. Mapování RC při B_SYNTH=0
odpovídá kalibraci/reverse/deadzone, 100 vzorků na osu. Porovnání
SHADOW OFF/ON ve STABILIZE dalo shodných 20 DISARMED i 20 ARMED ground
idle výstupů, bez tvrzení bitové ekvivalence všech transientů/letu.
Jde výhradně o původní software RC regrese; jejich simulované RC vstupy
nejsou součástí nového no-RC benchmarku a nezasahují do hardware.

RC regrese zachovala 200,00378 Hz a 399,99333 Hz; queue overflow test
má úmyslně nenulové Drop=890, HWM=4 a samostatné logger lifecycle
události. To je fault/regression scénář, nikoli lossless hardware
akceptace. SYSID read-only target alignment prošel na 1997 párech,
maximum odchylky 2,18279e−11 rad/s. flake8 všech dotčených Python
souborů, astyle všech dotčených C++/header souborů a diff whitespace
kontrola jsou PASS. Nový offline helper byl znovu spuštěn na finálním
no-RC BIN a celý 23-test Python suite prošel i po jeho přidání.

## 7. Build a reprodukce

Všechny příkazy běží z `/home/jirka/ardupilot_utb`, bez sudo, se stávajícím
toolchainem a venv. Nebyly doinstalovány balíčky ani jiné závislosti.

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH
./waf copter -j4
# Ihned archivovat arducopter.apj, .bin a ELF se source SHA a variantou.

./waf configure --board sitl --enable-UTB --enable-UTB_BENCH
./waf copter -j4 --targets tests/test_utb,tests/test_utb_bench,tests/test_utb_math,tests/test_utb_synthetic
build/sitl/tests/test_utb
build/sitl/tests/test_utb_bench
.venv/bin/python -B Tools/autotest/test_utb_synthetic_reference.py \
  --binary build/sitl/bin/arducopter \
  --unit-binary build/sitl/tests/test_utb_synthetic \
  --math-binary build/sitl/tests/test_utb_math \
  --output "$PWD/tmp/utb-synthetic-new-session"
.venv/bin/python -B Tools/autotest/utb_math_validation.py \
  --binary build/sitl/tests/test_utb_math --output "$PWD/tmp/utb-math-new-session"
.venv/bin/python -m unittest discover -s Tools/autotest/unittest -p 'test_utb_*.py' -v
```

Output musí být nový adresář. Testovaný C++ generátor exportuje vectors
a nevynechává svůj export test; samotné spuštění unit binárky bez
UTB_REFERENCE_OUTPUT tento jeden test úmyslně přeskočí.

Build matrix:

| Board | UTB | BENCH | Logger | Stav |
|---|---|---|---|---|
| SkystarsH7HD-bdshot | ON | ON | ON | PASS, source f9cc3bcc0f |
| SITL | ON | ON | ON | PASS, source f9cc3bcc0f, unit/SITL |
| SkystarsH7HD-bdshot | OFF | ON | ON | PASS, compile-out guard |
| SkystarsH7HD-bdshot | ON | OFF | ON | PASS, generátor compile-out |
| SITL | ON | OFF | ON | PASS, generátor compile-out |
| SkystarsH7HD-bdshot | ON | ON | OFF | PASS, source f9cc3bcc0f, compile guard |

Guard varianty byly sestaveny během implementace; releasovaný testovací
APJ níže je explicitně sestavený z f9cc3bcc0f. Pro přepínání používat
`--disable-UTB` a `--disable-UTB_BENCH`. Logging OFF lze ověřit pomocí
`--extra-hwdef` souboru obsahujícího `define HAL_LOGGING_ENABLED 0`.
Tato varianta se **nesmí použít pro HW benchmark loggeru**.

### Jednoznačný artefakt připravený k posouzení

- Source: `f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f`.
- APJ board ID: **1075**, git_identity **f9cc3bcc**.
- UTB ON, BENCH ON, logger ON; default UTB_B_SYNTH=0.
- APJ velikost **1129203 B**, image_size **1251892 B**.
- SHA-256: `a45f3c9f08e21e8b713cfe59e3fe404fe4e866164c6c21f1a00edd0876622878`.
- Umístění:
  `tmp/utb-synthetic-reference-20261012_085037/SkystarsH7HD-bdshot-f9cc3bcc0f-UTB_ON-BENCH_ON-SYNTH_OFF/arducopter.apj`.

ELF má skutečné symboly AP_UTB_SyntheticReference, prepare_synthetic,
utb_log_thread, UTBBench::MutexProbe, task_overrun a stack_sample;
`firmware-symbols.txt` obsahuje kontrolovaný výpis. Default 0 se ověřuje
v AP_Param source; již uložené parametry v FC je nutné přečíst a před
experimentem zkontrolovat. Aktuální build/bin může obsahovat poslední
compile-guard variantu; pro případný upload vybírat jen archivovaný
APJ s výše uvedeným SHA.

Offline analýza staženého souboru, bez jakékoli serial transport cesty:

```sh
.venv/bin/python -B Tools/autotest/utb_synthetic_log_analysis.py \
  /absolutni/cesta/k/SYNTHETIC.BIN --output /absolutni/cesta/k/novemu-reportu.json
```

Nástroj vyžaduje existující běžný BIN, UTBX kotvy a známé testovací gains.
Potvrzení gains z PARM a zálohy je samostatná povinnost. Výsledek
full_400_sequence kontroluje počet, validitu a Seq; necertifikuje PM,
mutexy, stack, ostatní ztráty ani původ dat. Současný
`utb_hardware_capture.py` je zachován a vyžaduje platné RC. Pro tento
no-RC experiment se nepoužívá; RC guard se nesmí obejít fingováním vstupu.
Připravený postup vyžaduje dozorovaný samostatný parameter/log workflow.

## 8. Bezpečnostní audit

**PASS – source a software rozsah.** Proti původnímu acf04 jsou bitově
stejné RateController, MotorMixer, Reference, State provider, AP_Motors,
HAL/ChibiOS/DShot, AHRS/INS, ModeUTBAcro i arming implementation.
Nové změny jsou BENCH reference, jejich guard/reset a metadata.
Žádný motor setter, HAL rc output, SRV_Channels či DShot není v AP_UTB.
Controller ani AP motor output se nevolají podruhé.

Producer nemá nový logger call, čekání ani dynamickou alokaci.
Stávající pevná SPSC queue má kapacitu 4, status používá nonblocking
try-lock a feedback původní atomiky. Worker čte pouze snapshot kopie;
privátní stav generátoru nemá sdíleného zapisovatele. Worker priority
PRIORITY_IO,0 a původní logger lock policy se nemění.

**RISK – hardware dosud neověřeno:** sdílené logger mutexy mohou nepřímo
blokovat main; přidané UTBX zápisy riziko neodstraňují. Neexistuje změřený
H743 bound generátoru, nových gains ani celého consumeru. Nulový počet
potvrzených mutex kolizí není důkaz absence blocking. Stack warning
threshold není dostatečná bezpečná rezerva. Fyzická motorová izolace
nového firmware není ověřena nahráním, pouze source/ELF/SITL.

## 9. První hardware session – připravený postup, dosud neprovedeno

Tento postup je podklad k samostatnému schválení firmware i přesného
souboru parametrů. Neopravňuje automaticky měnit FC.

1. Nové fyzické potvrzení člověkem: všechny vrtule sundané, receiver
   odpojen, vysílač se nepoužívá, stabilní napájení a FC v klidu.
   Ověřit aktuální port a board identity; COM5 se nepředpokládá automaticky.
   Žádné ARM, motor test ani RC override.
2. Archivovat kompletní parametry a všechny původní BIN, velikosti a SHA.
   Oddělit backup/config/firmware/raw/results. Žádné smazání flash bez
   samostatného souhlasu. Ověřit volnou logovací kapacitu pro kontrolní
   i plný běh včetně běžných AP logů.
3. Odsouhlasit konkrétní APJ se SHA uvedeným výše, poté teprve upload.
   Ověřit identity po bootu, DISARMED a B_SYNTH=0/SHADOW=0 před nastavením.
   Uchovat přesný firmware i readback parametrů použitý pro každý běh.
4. Čistý start bez receiveru, skutečný EKF/AHRS settle nejméně 20 s
   od EKF startu a následně STATE_VALID/nové INS timestampy.
   Pokud je radio či jiný failsafe aktivní, zastavit. Nečistit flags,
   nevypínat FS_THR_ENABLE/FS_GCS_ENABLE ani EKF health.
   Je-li původní FS_THR_ENABLE=0, konfiguraci nejdříve samostatně
   schválit; test se takto nespustí.
5. Readback FRAME_CLASS=1, FRAME_TYPE=18 pro potvrzený fyzický BF_X_REV,
   loop 400 Hz, FSTRATE disabled, správné IMU, healthy/initialised AHRS,
   logger backend BLOCK/LOG_BACKEND_TYPE=4. UB backend ID=4.
   UTB_ENABLE=1 musí být účinné od bootu. Původní AP režim zvolit
   stabilní běžný DISARMED mód; UTB_ACRO se pro benchmark neaktivuje.
6. Samostatně schválené nastavení: 15 gains z tabulky, UTB_LOG_FMT=1,
   UTB_LOG_RATE=400, LOG_DISARMED=1, UTB_SHADOW=1, B_SYNTH zatím 0.
   LOG_BITMASK a ostatní parametry převzít z archivované benchmark
   konfigurace; bez nezdůvodněného omezení původní diagnostiky.
   Přesně zálohovat/readback všechny měněné hodnoty. Při GCS failsafe
   zachovat opravdový heartbeat z dozorujícího spojení.
7. Kontrolní krátký běh profilu 1, přibližně 5 s včetně preroll.
   B_SYNTH=1 pouze při platných guardech. Ověřit source/status,
   P/I a znaménka, worker heartbeat/stack, queue, mutex a PM.
   Poté B_SYNTH=0/SHADOW=0. Stahovat až po ukončení měření.
   Pokud health po downloadu zůstane špatné, zastavit a případný nový
   fyzický start provést až v povoleném hardware workflow.
8. Před plnou sekvencí nový reset zdroje přes 0, platný state a prázdný
   backlog; SHADOW=1, B_SYNTH=1. Vyčkat PREROLL a celé RUNNING
   Cycle=0…7999 až COMPLETE. Jde nominálně o 1+20 s; skutečné trvání
   ověřit z TimeUS. Na H743 nepoužívat profil 2.
9. Zmrazit experiment B_SYNTH=0, SHADOW=0 a podle schváleného postupu
   LOG_DISARMED=0, teprve potom download. Potvrdit readback.
   Download nikdy neběží v měřeném okně, může prodloužit main periodu.
10. Archivovat nezměněný BIN a parameter/readback log; dekódovat UTBC,
    UTBX, UTBQ, DSF, PM, UBMT/UBHI/UBST/UBEP. Offline helper prověří
    signály a matematiku. Na závěr obnovit všechny původní parametry
    a potvrdit každou hodnotu readbackem; B_SYNTH nesmí zůstat aktivní.

### Akceptace a stop podmínky

Platné RUNNING okno vyžaduje 8000 uložených platných UTBC a všech 7999
Seq kroků=1, úplné UTBX přiřazení, platný skutečný stav/mixer, správné
gains/model a frekvenci přibližně 400 Hz podle skutečného TimeUS.
Cyklový počet sám není dokladem přesného 20s wall-time trvání.
P/I musí odpovídat signálu, IMAX a saturation/integration gate musí být
vyhodnoceny; D se hodnotí jen při skutečně buzené gyro derivaci.

Po celé stabilní měření se nesmějí zvýšit queue Drop, backend Miss
ani DSF.Dp; startup a změny Epoch se oddělí. Nesmí nastat TIME_BUDGET
latch, worker failure, nový stack warning ani nečekané PM.NLon.
Původní časový UTB budget 5 % nominální periody, při 400 Hz 125 µs a
latch po třech následných překročeních, se nemění. Tento existující
požadavek nenahrazuje celkovou analýzu flight-control deadline.

Vyhodnotit H743 period min/max/průměr a absolutní jitter, UBST[8]
(task budget overrun), UBST[9] (period > nominální perioda), PM.NLon,
producer/consumer wall time, main acquire, worker hold/heartbeat,
stack free/allocated size a queue HWM. UBST[9] ani UBST[8] samostatně
nejsou hard deadline miss. PM.NLon=0 také není formální důkaz všech
hard deadline. Porovnat stejné napájení, AP logger konfiguraci a
stabilní okna proti A/B a starému zero-gain D.

Historický zero-gain H743 D dosáhl 8000 platných sad/400 Hz,
producer mean 39,75 µs/max 52 µs, main acquire max 74 µs,
worker hold max 253 µs, stack free minimum 1784 B z 3504 B.
Jde o jiný source/gains; tyto hodnoty nejsou zárukou nového firmware.
Přijatelné dodatečné load/hold/jitter limity pro nenulový nový test nejsou
předem naměřené; nesmí se prohlásit za bezpečné podle náhodné mezní
hodnoty. Vybočení proti srovnatelnému baseline, nové PM dlouhé periody,
rostoucí backlog či nejednoznačný health jsou důvodem experiment přerušit
anebo výsledek označit UNVERIFIED a provést source/časovou analýzu.

Okamžitě ukončit při ARMED, fyzickém motorovém projevu, resetu/watchdog,
failsafe, neplatném AHRS/gyro/timestampu, TIME_BUDGET, ztrátě loggeru,
workeru, progressu či stack warning. Neodblokovat test změnou safety
prahu. Žádné aktivní UTB řízení ani další fáze.

## 10. Přesný manifest FÁZE 1H-C

Rozsah níže je proti již odděleně uložené matematické FÁZI 1H-B
(adb02b6ddb); checkpoint a předchozí dirty soubory nebyly zahozeny.

| Soubor | Zásah |
|---|---|
| libraries/AP_UTB/AP_UTB_SyntheticReference.h | Nová pevná BENCH-only state machine |
| libraries/AP_UTB/AP_UTB_SyntheticReference.cpp | Konečná sekvence, rovnice, fault/completion latch |
| libraries/AP_UTB/AP_UTB.h | BENCH Inputs metadata, nový parametr a lokální generátor |
| libraries/AP_UTB/AP_UTB.cpp | AP_Param index 22, způsobilost, reset, abort a zneplatnění budget cyklu |
| ArduCopter/utb.cpp | Read-only health/failsafe guard, volba reference, worker metadata |
| ArduCopter/utb_log.h | Oddělený 27B UTBX format; UTBC zachován |
| ArduCopter/defines.h | Volný vehicle log ID 23 pod compile guard |
| libraries/AP_UTB/tests/test_utb_synthetic.cpp | Čtyři unit testy a export všech signálů |
| Tools/autotest/test_utb_synthetic_reference.py | No-RC SITL, rovnice, oddělené measurement/model porovnání |
| Tools/autotest/unittest/test_utb_synthetic_decoder.py | Tři testy attribution/epoch/terminal hranic |
| Tools/autotest/utb_synthetic_log_analysis.py | Offline BIN ověření bez serial rozhraní |
| docs/UTB_SYNTHETIC_REFERENCE_BENCH_CS.md | Implementovaný stav, důkazy a připravený HW postup |

Předchozí untracked matematické nástroje/reporty a archivace H743
výsledků byly rozděleny do d325c44390 a adb02b6ddb. Přehled
`history-reword-map.json`, bundle a refs/utb-checkpoints zachovávají
původní unpublished historii. Starý acf04 má identický source tree jako
nově pojmenovaný 825724b250. Publikovaný feb5b68361 se nepřepisoval,
není proveden push. Historické source/firmware SHA v předchozích
reportech zůstávají identitou skutečně dříve testovaných binárek.

## 11. Závěr a dosud otevřené ověření

UTB SHADOW lze bez RC plnohodnotně **softwarově diagnostikovat**:
skutečný shadow state se zdravým původním estimator/INS, interní
reference, nenulové gains, mixer, reset/fault policy a uložená 400Hz
sekvence prošly SITL; odvozená D dynamika byla ověřena samostatným
matematickým measurement runnerem. Je připraven konkrétní BENCH ON
firmware pro **samostatně schválený DISARMED H743 experiment**.

Na skutečném H743 zbývá: fresh-boot health bez RC s původní failsafe
konfigurací, nonzero P/I/D/mixer numerická shoda, 8000 platných a
neztracených sad v novém firmware, CPU/producer/consumer overhead,
logger mutex riziko, stack/lifecycle, skutečný datový tok a period/jitter
oproti baseline. Fyzickou D dynamiku na klidném FC nelze získat pouze
skokem reference. Žádný výsledek těchto bodů se zatím neoznačuje HW PASS.
