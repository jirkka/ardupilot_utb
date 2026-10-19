# FÁZE 3A – vlastní výpočet podle ArduPilot Copter 4.7.1

Stav: implementace v pracovním stromu nad `b25e828b86f945996489b26fb0357ce05f72afcb`,
větev `test/skystars-5inch`. Výsledky a omezení jsou uvedené níže.
Aktivní řízení je výhradně SITL. Hardware nebyl připojen ani testován.

## Funkční řetězec

| Modul | Vstup | Výpočet a výstup |
|---|---|---|
| `UTB_Core` | původní AHRS view, AP_Motors, dt, změna režimu | jedna společná instance Rate/PID, inicializace a předání historie |
| `UTB_Input` | původní RC kalibrace/reverse/deadzone, AP pilotní parametry | AP expo, kruhové omezení páček, náklon nebo rates v rad/s; manuální throttle |
| `UTB_State` | AP_AHRS_View, původní INS | nejnovější filtrované gyro a quaternion; žádný vlastní odhad stavu |
| `UTB_PID` | target, gyro, dt, původní motor limit flags | vlastní kopie AP P/I/D, FF/D_FF, filtrů, slew limiteru, PD limitu a directional anti-windup |
| `UTB_Rate` | body rates R/P/Y, gyro | stejná sada tří PID pro oba režimy, native gain/throttle mix/angle boost politika |
| `UTB_Attitude` | požadovaný náklon a yaw rate, aktuální quaternion | vlastní kopie AP quaternion attitude regulace a input shapingu → body rates |
| `Copter/UTB_Acro` | RC → UTB_Input | AP rate-only ACRO, bez traineru a bez samonivelace → společný Rate |
| `Copter/UTB_Angle` | RC → UTB_Input | AP STABILIZE: požadované roll/pitch + yaw rate → Attitude → společný Rate |
| `UTB_Output` | výsledné momentové požadavky, FF, throttle | původní AP_Motors setters; vlastní motor output ani HAL neimplementuje |

Kořenové C++ moduly jsou v `libraries/AP_UTB/UTB_*.h/.cpp`, vehicle adaptéry
v `libraries/AP_UTB/Copter/UTB_*.h/.cpp`. Python zůstává v `Tools/utb/`.
`Copter/UTB_Core.cpp` pouze serializuje dosavadní diagnostické pakety.
Neexistuje dědičnost od AC_PID/AC_AttitudeControl ani volání jejich řídicího
výpočtu v nové pilotní cestě. AC_P slouží pouze jako parametrický úložný typ;
vektorová/quaternion geometrie a obecné filter/notch primitivy zůstávají AP.
Vnitřní vazba `UTB_Rate : UTB_Attitude` zachovává původní AP uspořádání
AC_AttitudeControl_Multi/base. Obě části obsahují vlastní matematiku;
ACRO používá pouze zděděný body-rate shaping, ANGLE navíc vlastní attitude
výpočet. Nejde o druhou nezávislou PID instanci.

ACRO: RC → Input → Acro → Rate → PID → Output → AP_Motors.
ANGLE: RC → Input → Angle → Attitude → Rate → PID → Output → AP_Motors.
Původní `Copter::motors_output()` nadále provádí jediný skutečný motor output.
Původní rate slot a custom controller jsou při vlastnictví UTB42/43 přeskočené.
V UARX je nový owner 3, platný vzorek má Flt 0, aktuálně neplatný stav/output255
(není latched stop). UN/MN počítají rate-slot pokusy a post-output callbacky.
APN0 označuje native dispatch vyřazený podle source; není nezávislou dynamickou
sondou všech volání native PID. Nativní FF a další PID detaily jsou v PIDR/P/PY.

## Převzaté algoritmy a původ

Referencí je skutečný source na potvrzeném commitu, nikoliv předchozí UTB PID.
Přesné vstupní SHA-256 jsou v `tmp/utb-3a-20261019_142914/reference-complete-manifest.json`;
kanonické source kopie jsou v témže adresáři pod `ap-reference/`.
Licence GNU GPL v3 nebo pozdější a původní autorství zůstávají zachované.

- `libraries/AC_PID/AC_PID.*`: `update_all`, `update_i`, reset/filter/FF/PD scaling.
- `libraries/Filter/SlewLimiter.*`: lokální UTB implementace původního adaptive P/D slew limiteru.
- `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`: attitude command model,
  body-rate input shaping, quaternion/thrust-heading error, feedforward,
  frame transforms, reset target/rate, landed gain reduction a inertial frame reset.
- `libraries/AP_Math/control.cpp`: lokální kopie uzavřené množiny sqrt/inverse-sqrt
  a position/velocity/acceleration shaping helperů potřebných pro attitude shaping.
  Nejde o novou výškovou ani XY regulační smyčku.
- `AC_AttitudeControl_Multi.*`: vlastní vnitřní Rate/PID sestava, původní
  pořadí motorových setterů, zpětná vazba saturace, throttle mix a angle boost.
  Pomocný native název `update_althold_lean_angle_max` omezuje náklon podle
  throttle headroom; neimplementuje ALT HOLD.
- `ArduCopter/mode_acro.cpp`: relevantní RATE_LOOP_ONLY větev, spool a pilotní mapy.
- `ArduCopter/mode_stabilize.cpp`, `mode.cpp`, `AP_Math/control.cpp`:
  manuální throttle, expo, zpracování požadovaného náklonu a STABILIZE režim.
- `AC_AttitudeControl_Logging.cpp`: ANG/RATE pakety; PIDR/PIDP/PIDY a MAVLink
  attitude/PID telemetry vybírají skutečný aktivní UTB controller.

## Významné odchylky a konfigurace

- Záměrně je vybrané původní **rate-only ACRO**, nikoliv výchozí stabilized ACRO
  ani trainer. Plná orientace není omezená experimentálním náklonovým stopem.
- Nová cesta používá původní AP ACRO_RP/Y rate/expo a STABILIZE pilotní mapy.
  BF Actual Rates a staré `UTB_P_*`/`UTB_A_*`/`UTB_C_*` gain/limit banky jsou
  historické. Novou matematiku neřídí; A_ENABLE/C_ENABLE zůstávají opt-in.
  BF mapa není v 3A připojená jako další přepínatelná větev.
- Vlastní banku native gains/filtrů představuje kořen `UT3_`, key 107 (stávající
  param indexy nezměněné). RP defaults .135/.135/.0036, Y .18/.018/0 odpovídají
  native konstruktoru; nejde o pokračování tuningu2H. SITL copter.parm mění AP yaw
  na .3/.02, proto srovnávací runner nastavuje stejné UT3 yaw hodnoty.
- ANGLE požadovaný náklon omezuje `UT3_ANGLE_MAX`, časování/shaping vlastní banky
  zůstává původní AP. Manuální throttle a angle boost odpovídají STABILIZE.
- Dvě oddělené banky vyžadují integrační předání: native↔UTB reset targetu/filtrů
  a omezený přenos živého I a skutečného throttle mixu; mezi 42↔43 se společný PID a shaping zachovávají.
  Native throttle input cache se naplní před inicializací cílového AP režimu,
  protože ji původní position controller čte během inicializace.
- Inicializace odmítá chybějící opt-in, nezdravý vstupní stav, neplatné native
  gains a FSTRATE. Runtime dt musí být konečné a kladné. Nativní per-axis PID
  odmítá neplatné target/gyro; Output nepředává NaN/Inf do motorové knihovny.
  Chybná numerika hlásí AP InternalError; není zaveden nový automatický LAND,
  latched global stop ani hromadné vypnutí tahu místo původního failsafe.
- `UTB_ENABLE` je původní boot-cached parametr s požadavkem rebootu; změna za
  běhu sama nevypíná boot opt-in. Negativní test musí použít nový start.

## Přehled nastavení nové pilotní cesty

| Nastavení | Parametry | Význam |
|---|---|---|
| Opt-in | `UTB_ENABLE`, `UTB_A_ENABLE`, `UTB_C_ENABLE` | ENABLE je boot-cached; další dva povolují42/43 |
| ACRO vstup | `ACRO_RP_RATE`, `ACRO_RP_EXPO`, `ACRO_RP_RATE_TC`, `ACRO_Y_RATE`, `ACRO_Y_EXPO`, `ACRO_Y_RATE_TC` | původní AP rate/expo/shaping; nikoliv BF Actual |
| ANGLE vstup | `UT3_ANGLE_MAX`, `PILOT_Y_RATE`, `PILOT_Y_EXPO`, `PILOT_Y_RATE_TC`, `UT3_INPUT_TC` | maximální požadovaný náklon, yaw rate a input shaping |
| Vnější P | `UT3_ANG_RLL_P`, `UT3_ANG_PIT_P`, `UT3_ANG_YAW_P` | původní AP attitude law, vlastní nezávislá parametrická banka |
| Společný Rate PID | `UT3_RAT_RLL_*`, `UT3_RAT_PIT_*`, `UT3_RAT_YAW_*` | P/I/D/FF, IMAX, target/error/D filtry, notch a slew dle native AP |

`ATC_*` nadále řídí původní AP režimy; automatické průběžné kopírování do UT3
není zavedené. Manuální throttle používá původní RC/hover mapu. Limity
`UT3_RATE_R/P/Y_MAX` zachovávají native attitude semantics; nejsou dalším
ACRO rate-only capem. Původní náklonový rozsah pre-arm sanity checku UT3 je
10–80°, geometrický helper má původní85° mez. Povolená parametrická doména
není tvrzením o letovém ověření celého rozsahu.

## Původní AP bezpečnost a integrační místa

Původní mandatory/pre-arm kontroly, motor arming/interlock/spool, RC/GCS/EKF
failsafe a mode manager zůstávají.42/43 patří do manual-throttle větve původní
failsafe disarm politiky. Forced arm neobchází vlastní mode readiness checks.
Crash/land/thrust-loss/yaw-imbalance diagnostika dostává skutečné aktivní targety
nebo I složku. ACRO crash check je vypnutý stejně jako native ACRO.
Původní EKF reset události přestavují UTB attitude target stejným AP výpočtem.
Standby událost resetuje také vlastní I a yaw target. Lean arming kontrola
používá limit skutečně vybraného controlleru.
Throttle mix a landed-gain politika běží v původních scheduler místech.

AP_Motors, HAL, DShot, EKF/INS algoritmy, priority a scheduler nejsou přepsané.
Převod RC, datum posledního RC vzorku a failsafe zajišťuje původní RC infrastruktura.
Funkce bez požadavku na polohu zůstávají bez vlastní výškové/polohové obálky.

## Oddělení předchozí rozpracované práce

Před prvním zásahem vznikl ověřený checkpoint všech 31 dirty souborů včetně
untracked: `before.json`, `before.patch`, `before-complete.tar.gz` v adresáři
nové evidence. Žádný historický výsledek ani commit není přepsaný.
Původní experimentální matematika mode41/SHADOW byla obnovena na b25e828b;
relevantní AP integrační místa byla znovu posouzena pro nový controller.
Po deseti úspěšných skutečných SITL scénářích prototypu byl starý pilotní
adaptér a nedokončený safety runner/report archivovaný a vyřazený. Přesné
obsahy jsou ověřené v `retired-pilot-files.tar.gz` a retirement manifestu.
42/43 vybírají výhradně UTB_Acro/UTB_Angle. Původní nedokončené safety výsledky nejsou označené za PASS.
Změna má 58 souborů (33 upravených, 22 nových, 3 vyřazené tracked adaptéry);
přesný seznam, SHA-256 a Git blob ID ukládá `final-change-manifest.json`
v adresáři evidence. Necommitované dva vyřazené soubory jsou rovněž v archivu,
nikoli jako dodatečné Git deletions.

## Ověření

Evidence: `tmp/utb-3a-20261019_142914/`. Testy nové pilotní cesty používají
finální SITL binárku V12, SHA-256
`bdc8e174a5f512096f38d8e28a2c3ba518ac39e42b4cee033679e908dde1ee5d`.

- Implementováno: devět modulů, původní AP matematika a popsaná integrace.
- C++: všech deset UTB testovacích programů prošlo; 138 různých případů včetně
  dvou vector případů spuštěných samostatně s datovým souborem. Pět nových testů
  porovnává PID s AC_PID (30 000 kroků), vstupní mapy, quaternion/shaping/frame
  reset a předání živého I/throttle mixu s původní implementací AP.
- Python: 156 unit testů; offline matematika 30 946 vektorů bez chyby,
  syntetická reference 16 000 vektorů. Flake8 a astyle nových C++ souborů prošly.
- SITL: všech 19 scénářů nové pilotní cesty prošlo. Čerstvý UTB_ENABLE=0 boot,
  DISARMED motorový výstup, zamítnuté běžné/force arm při chybějícím opt-in,
  běžné armování obou režimů, ACRO R+/P−, ANGLE R+/P−, ±45° na obou osách
  se samonivelací, manuální throttle, přímé RC přepínání i s nenulovými páčkami,
  čtyři signed roll/pitch rotace přes 360° a návrat do ANGLE. Rychlost při rotacích
  přibližně 365°/s; integrované rotace R+/R−: 366,48°/−364,66°,
  P+/P−: 365,54°/−365,57°. Špičky při požadavku 45°: roll 45,66° a pitch 45,65°.
- RC/GCS: ztráta skutečně posílaného RC datového proudu nebo GCS heartbeat
  v obou režimech vyvolala původní RTL, mode reason 3/5. Obnovení spojení
  automaticky nevrátilo UTB. EKF primary reset prošel **DISARMED**.
- Historické regrese: FÁZE0 ON/OFF, active-absence a mode41 basic/guards prošly.
  Rozšířená mode41 responses prošla při speedup 2: všech 18 běhů
  (devět profilů UTB/AP včetně změny setrvačnosti a zpoždění), každý 3200 vzorků.
  Packed SHADOW prošel: 200/400 Hz, přepnutí formátu, frame/FSTRATE/param guards,
  potlačení loggeru, system-identification/capture, primary gyro změna, RC mapa
  a motorová izolace ON/OFF v DISARMED i armed ground idle. Ustálené RCOU
  se v tomto omezeném scénáři nelišily; nejde o bitovou ekvivalenci za letu.
  Dosažená SHADOW frekvence 200,004/399,993 Hz, žádné neúplné sady.
  Test zahrnuje i zaplnění fronty; 958 queue drops není měření běžného ustáleného provozu.
- Build: SITL active ON, SHADOW-only a UTB OFF; H743 UTB ON/BENCH ON
  i s active-request a UTB OFF/BENCH OFF prošly. Finální H743 V12 byl znovu
  sestaven po handoff opravách. ELF symbol audit potvrzuje úplné odstranění
  všech devíti aktivních tříd z H743 a variant s active OFF/UTB OFF.
  BENCH zůstává nezávislý; v H743 ON je přítomná původní SHADOW instrumentace.
- Parametry/logger: obě metadata generace prošly. Nová UT3 skupina obsahuje
  61 parametrů, maximální délka názvu 16, indexy starých parametrů se nemění.
- Izolace: knihovny AP_Motors, HAL, INS, AHRS/EKF, scheduler a logger ani
  submoduly nejsou změněné. Jediný skutečný output zůstává Copter motors_output.
- Hardware: **neověřeno**, bez COM5, uploadu či fyzických motorových testů.

Finální H743 compile-out artefakt (není určený pro aktivní UTB let):
`builds/H743-FINAL3A-V12/arducopter.apj`, 1 129 183 B, SHA-256
`d36af89eb87b3799a8587c4ca9cc02d292788b5fe71fd42fa190a3fc046b7d11`.
Přesné příkazy jsou v README_BUILD_WSL_CS.md a v evidenci buildů.

## Omezení a další práce

Ověření platí pro generický stock bfxrev SITL a uvedené scénáře, nikoli pro
fyzickou platformu. Nebylo provedeno ladění ani změna fyzikálního modelu.
Změna primárního EKF byla testovaná DISARMED; porucha EKF za letu, všechny
kombinace battery/fence/sensor failsafe a přechody z mode41 do pilotních režimů
nejsou letově ověřené. Přenos I/mix je testovaný, přechody při libovolně rozdílných
bankách gains nejsou obecně kvalifikované. BF Actual Rates není zapojené do 3A.
SHADOW regrese ověřuje původní SHADOW kontext, nikoli současné porovnání při
letu42/43. ALT HOLD, XY, mise a aktivní hardware řízení nejsou součástí 3A.

Průběžné nálezy integrační revize:
- Přenos I používá `get_i()`: poslední logový `get_pid_info().I` může po resetu
  zůstávat starý. Regrese ověřuje nenulový live I i následný reset před handoffem.
- Native návrat přebírá aktuální throttle mix, aby nepokračoval ve staré
  prioritě alokace z doby před vstupem do UTB.
- Původní test chybně očekával, že změna EK3_PRIMARY v letu vynutí přepnutí.
  AP ji standardně vynucuje DISARMED; timeout testu je zachovaný v evidenci V10.
  Firmware ani EKF policy nebyly měněné kvůli tomuto očekávání. Korigovaný test
  zůstává DISARMED a nenahrazuje ověření poruchy EKF za letu.

- Historická mode41 responses regrese při SITL speedup 5 ztratila diagnostické
  záznamy (DSF.Dp 24 959, pouze 2202 kompletních vzorků). Neprošla a není
  vydávaná za PASS. Opakování stejného firmware při speedup 2 zachovává
  původní požadavek 3200 vzorků; bez změny controlleru či testovacích tolerancí.
