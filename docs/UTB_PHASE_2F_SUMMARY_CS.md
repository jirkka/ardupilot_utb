# FÁZE 2F: pilotní UTB ANGLE v ArduPilot SITL

17. 10. 2026 · větev `test/skystars-5inch` · rodič
`9824542e73bbfb193b21fd4388f74e14ba24439c` · pracovní změny bez nového commitu.

Implementován samostatný skutečný **mode 42 UTB_ANGLE**, který používá AP RC
infrastrukturu a původní vlastní UTB attitude/rate controllery. Není to
externí demonstrátor. Aktivní autorita existuje pouze v HAL_BOARD_SITL.

Výsledek se vztahuje k omezenému RC ANGLE prototypu v malé ověřené obálce.
Není dokončen obecný manuální autopilot ani bezpečná aktivní hardware cesta.
[Architektura a parametry](UTB_ANGLE_MODE_ARCHITECTURE_CS.md) ·
[Validace a omezení](UTB_ANGLE_MODE_SITL_VALIDATION_CS.md).

## Implementováno

- RCMAP, skutečná AP kalibrace/reverse/deadband; kruhová RP reference do 5°.
- Střed RP aktivně vyrovnává náklon; yaw je body rate bez heading hold.
- Ruční throttle přes původní AP mapu/spool, bez angle boost/altitude smyčky.
- GCS/RC volba a nativní arming/failsafe, frozen konfigurace a latched stop.
- Výhradní vlastní attitude/rate dispatch, původní jediný motor output.
- Samostatné parametry A_ENABLE36, A_ANGLE37, A_YMAX38, A_AW39;
  staré indexy ani výchozí RAT/S_* konfigurace se nemění.
- UARC log ID 30,59 B, společné TimeUS/Seq s původními kaskádovými logy.
- Skutečné SITL runnery, unit testy, reprodukovatelné BIN/CSV/grafy.

## Ruční vyzkoušení pouze v lokálním SITL

Z WSL repozitáře, nikdy přes COM5:

```sh
cd /home/jirka/ardupilot_utb
export PATH="$PWD/.venv/bin:$PATH"
.venv/bin/python ./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
.venv/bin/python ./waf copter -j6
.venv/bin/python -B Tools/utb/tools/utb_angle_sitl.py --binary build/sitl/bin/arducopter --output tmp/utb-angle-manual-UNIQUE --suite manual
```

`UNIQUE` nahraď novým názvem; runner nepřepisuje existující experiment.
Vyčká na původní zdravou inicializaci. Vytvoří vlastní lokální model bfxrev,
RC/GCS heartbeat a parametry malé obálky. Příkazy:

```text
mode 42
status
sticks 0.6 0 0
sticks 0 0 0
mode 0
takeoff
sticks 0.6 0 0
sticks 0 0 0
sticks 0 -0.6 0
sticks 0 0 0.5
sticks 0 0 0
throttle 1475
throttle 1472
mode 0
land
quit
```

První `mode 42` a sticks jsou DISARMED kontrola. `takeoff` provede původní
GUIDED takeoff10m standardním armováním a potom přejde do42; není to UTB
výšková regulace. 0,6 znamená3°, ±1 znamená5°, kombinace se omezí kruhem.
Příkazy jsou průběžné: mezi změnami pilot musí hlídat telemetrii a svislou
obálku. PWM 1472 je pouze ověřený pracovní bod konkrétního modelu; pilotní
režim nemá automatické udržování výšky. Malé změny drž krátce.

`status` ukáže skutečné ATTITUDE/GLOBAL_POSITION/RC/HEARTBEAT. Autoritativní
UTB targety jsou v BIN UAAT/UARX/UARC; původní GCS ATT.DesRoll nebo RATE.RDes
nejsou vlastním UTB targetem. Roll target =5°×normalizovaný vstup po kruhovém
clampu; yaw target =0,1rad/s×vstup.

Pro samostatný GCS je druhá telemetrie `tcp:127.0.0.1:5762`. Před převzetím
RC z GCS dej `external`: uvolní overrides a vlastní heartbeat. GCS potom
musí poskytovat svoje validní kanály i heartbeat. Neověřenou kompatibilitu
konkrétního GCS joysticku nelze zaměnit za provedené pilotní testy runneru.
RC5=1000 volí STABILIZE0, RC5=1300 ANGLE42; FLTMODE1/2 a RCMAP jsou v
uchovaném param souboru experimentu. GCS SET_MODE42 je skutečně ověřený.

Ověřené přechody používají vystředěné R/P/Y páčky a ustálený pracovní throttle.
Před přepnutím páčky vystřeď: původní STABILIZE má šestkrát větší úhlovou
mapu než tento UTB limit. Přechod s nenulovým stickem není letově ověřen.
Bezpečný výstup v tomto kontraktu je explicitní `mode 0` nebo `land` (původní LAND9), potom
vyčkat na skutečný disarm a `quit`. Stop latch se nezruší obnovením RC;
potřebuje odchod a validní nový vstup. Při výpadku nezastavuj pouze heartbeat
s běžícími overrides: AP overrides samy aktualizují GCS kontakt.
Ověřená failsafe konfigurace je FS_THR_ENABLE1, FS_GCS_ENABLE1, FS_OPTIONS=0.
FS_OPTIONS=16 způsobilo při GCS ztrátě latched stop a pád modelu.

## Reprodukce automatických kroků

```sh
.venv/bin/python -B Tools/utb/tools/utb_angle_sitl.py --binary build/sitl/bin/arducopter --output tmp/utb-angle-validation-UNIQUE --suite disarmed
.venv/bin/python -B Tools/utb/tools/utb_angle_sitl.py --binary build/sitl/bin/arducopter --output tmp/utb-angle-validation-UNIQUE --suite roll3
```

Stejný output rodič uchovává DISARMED gate. Další samostatné suites:
`pitch3`, `roll5`, `pitch5`, `guards`, `combined`, `transitions`,
`rc-loss`, `gcs-loss`, `remap-roll3`, `ap-roll3`, `ap-pitch3`.
Každý název scénáře je možné v daném adresáři spustit jednou.
Bez kladného DISARMED gate automatické letové suites odmítnou start.

## Build a hardware izolace

```sh
.venv/bin/python ./waf configure --board sitl --enable-UTB --disable-UTB_RATE_SITL --enable-UTB_BENCH
.venv/bin/python ./waf copter -j6
.venv/bin/python ./waf configure --board sitl --disable-UTB --disable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf copter -j6
```

Negativní H743 compile isolation, včetně vynuceného active option:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
.venv/bin/python ./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
.venv/bin/python ./waf copter -j6
```

Logging-disabled H743 varianta přidá `--disable-logging`; UTB OFF varianta
použije všechny tři `--disable-UTB`, `--disable-UTB_RATE_SITL`,
`--disable-UTB_BENCH`. Každou binárku archivovat pod vlastním názvem před
dalším configure. H743 APJ zde slouží pouze jako compile-out důkaz;
**nenahrávat na FC**. Finální artefakty a SHA uvádí validační přehled.

## Známé nedostatky a další etapa

Není ověřen plný manuální RC takeoff/landing, agresivní pilotáž, větší úhly,
kompletní sada AP failsafe kombinací ani aktivní fyzický stroj. Vlastní
fail-stop může znamenat pád; není náhradou původního RTL/LAND.
Nezávislé dynamické globální AP call counters nebyly implementovány;
autorita je doložena source dispatch a skutečnými vlastními sekvencemi.

Před převzetím AltHold z předchozí BP je nutný audit konkrétního source,
souřadnic/units/dt, throttle/motor authority a přechodů; dále návrh oddělené
výškové smyčky, saturace a přenosu integrátorů, vazby na yaw/RP a failsafe.
BP sama nedokládá kompatibilitu. Zde žádný AltHold ani další fáze nezačala.
Před hardware autoritou zbývá identifikace skutečného 5″ modelu, H743
WCET/stack/logger/health validace a samostatně schválená bezpečnostní politika.

Pro grafy byly do izolovaného adresáře archivu `plot-runtime/` doinstalované
Python balíčky včetně matplotlib. Seznam verzí i instalační příkaz jsou
uchované. Systémové/venv ani firmware závislosti se tím neměnily.

## Přesný manifest pracovních změn

26 souborů: 15 upravených, 11 nových. Žádný soubor není odstraněn ani přesunut.

| Stav | Soubor | Účel |
|---|---|---|
| M | `Tools/utb/tools/test_utb_shadow.py` | Cílená oprava komentářů ve statickém auditu; stejné runtime scénáře a ochranné assertions. |
| M | `ArduCopter/Attitude.cpp` | Výhradní dispatch mode 42 do vlastní kaskády; návrat před původním AP rate PID. |
| M | `ArduCopter/Copter.h` | Instance/friend mode 42 a společné FSTRATE/custom-controller vyloučení. |
| M | `ArduCopter/GCS_MAVLink_Copter.cpp` | SITL-only dostupný režim v AVAILABLE_MODES. |
| M | `ArduCopter/Log.cpp` | Registrace nových UARC metadat pod active SITL guardem. |
| M | `ArduCopter/defines.h` | Nové dosud volné log ID 30; staré hodnoty beze změny. |
| M | `ArduCopter/mode.cpp` | Lookup nového režimu42. |
| M | `ArduCopter/mode.h` | SITL-only číslo42 a připojení vlastní deklarace. |
| M | `ArduCopter/motors.cpp` | Jediný mode-specific capture/log callback po původním outputu. |
| M | `docs/UTB_FILE_MAP_CS.md` | Doplňující inventura 2F a oddělení Python/C++. |
| M | `docs/UTB_PROJECT_STATUS_CS.md` | Aktuální vymezený stav2F, zachování historických údajů. |
| M | `docs/UTB_README_CS.md` | Rozcestník nového pilotního režimu. |
| M | `docs/UTB_SOURCE_GUIDE_CS.md` | Index nových zdrojů a jejich současné cesty. |
| M | `libraries/AP_UTB/AP_UTB.cpp` | Čtyři SITL-only parametry na nových indexech 36–39. |
| M | `libraries/AP_UTB/AP_UTB.h` | Guardované uložiště a gettery nových parametrů. |
| A | `Tools/utb/tests/test_utb_angle_sitl.py` | 7 kontrol povelů/evidence a regrese ignorování pouze C++ komentářů. |
| A | `Tools/utb/tools/utb_angle_sitl.py` | Vlastněný simulátor, skutečné RC scénáře, analýza a ruční ovládání. |
| A | `docs/UTB_ANGLE_MODE_ARCHITECTURE_CS.md` | Implementovaný kontrakt, parametry, scheduling a failsafe omezení. |
| A | `docs/UTB_ANGLE_MODE_SITL_VALIDATION_CS.md` | Skutečné výsledky, grafy, build/unit/SITL hranice a neúspěchy. |
| A | `docs/UTB_PHASE_2F_SUMMARY_CS.md` | Přesný manifest, ruční návod a souhrn 2F. |
| A | `libraries/AP_UTB/AP_UTB_AngleRC.cpp` | Skutečná AP normalizace, kalibrace, age guard, kruh a fixed SITL gains. |
| A | `libraries/AP_UTB/AP_UTB_AngleRC.h` | Typy a kontrakt pilotního RC adaptéru, pouze SITL. |
| A | `libraries/AP_UTB/Copter/mode_utb_angle_sitl.cpp` | RC/throttle lifecycle, původní kaskáda, fail-stop a post-output evidence. |
| A | `libraries/AP_UTB/Copter/mode_utb_angle_sitl.h` | Deklarace samostatného pilotního režimu a pevných stavových struktur. |
| A | `libraries/AP_UTB/Copter/utb_angle_log.h` | UARC wire struktura a parser metadata,59 B. |
| A | `libraries/AP_UTB/tests/test_utb_angle_rc.cpp` | 13 testů skutečných AP kanálů a vlastních controller výstupů. |


Závěrečný H743 build nejdříve selhal až při dodatečném HEX balení, protože
subprocess vybral systémový Python bez intelhex. Kompilace a APJ už vznikly.
Opakování se správným `.venv/bin` v PATH prošlo bez instalace závislostí;
intelhex v existujícím venv byl přítomný. Příkazy výše proto exportují PATH
explicitně. Výsledný APJ je bitově shodný s předchozím H743 buildem2F.



## Závěrečný stav ověření

**Omezený RC ANGLE prototyp v SITL: implementováno a prokázáno ve vymezeném
rozsahu.** Jednoosé±3°/±5°, aktivní samonivelace, yaw±0,05rad/s, malé throttle
kroky, center-stick přechody, RC/GCS test s explicitní nativní RTL politikou.
112 C++ a135 Python testů PASS; staré RATE/ATTITUDE/AW a kompletní SHADOW
200/400 Hz regrese PASS. Původní SHADOW testovací parser byl cíleně opraven,
protože chybně započítával Doxygen komentář jako volání; runtime kritéria
se neměnila. Detailní nominální a kombinované metriky jsou ve validačním reportu.

Nejde o PASS pro libovolný manuální rozsah, všechny failsafe volby nebo
aktivní H743. FS_OPTIONS16+GCS ztráta vedla k pádu simulace. Ne všechny
kombinované plateau dosáhly2% settling. Globální nezávislý AP call profiler
nebyl přidán. Celý SITL logging OFF build zůstává blokovaný starou nativní
AC_CustomControl logger závislostí; nové soubory a H743 logging OFF prošly.

Konečný SITL executable:
`../tmp/utb-2f-20261017_093117/builds/sitl-angle-final/arducopter`,
5 772 656 B, SHA-256
`b084edeb3d82c1a8f53dfb6ba8a22a8a45b7d9a707f45d3610420550766cad4e`.

H743 pouze compile-out důkaz:
`../tmp/utb-2f-20261017_093117/builds/h743-completion/arducopter.apj`,
1 129 203 B, SHA-256
`c643a1598ddb4b2526ab3d4d58add280c4ccdea1d4fb46d979dbdac428924e92`.

Předchozí12 regulačních/adaptačních souborů je SHA-shodných. Ověřeno také
3709 souborů historických artifact manifestů2B/2C/checkpointu čitelnosti:
žádná velikost ani SHA se neliší. HEAD zůstává
`9824542e73bbfb193b21fd4388f74e14ba24439c`; nový commit/push nevznikl.
Index zůstává prázdný, změny jsou pouze pracovní. Nová data/source checkpoint
včetně untracked a úplný SHA manifest jsou v samostatném archivu2F.
COM5, fyzické parametry, upload a hardware motorový test neproběhly.
