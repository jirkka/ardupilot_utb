# UTB: vlastní regulační vrstva v ArduPilot Copter

FÁZE2I: [experimentální Actual Rates a rozšířený ANGLE](UTB_PILOT_RATES_CS.md).
Nová samostatná skupina `UTB_BF_*` nabízí RP/Yaw maximum, citlivost středu a expo,
maximální ANGLE náklon a nezávislý RP rate cap. Aktivní pouze v SITL;
`UTB_BF_TYPE=0` je výchozí původní chování. PID gains2H se nemění.
Níže uvedené lineární mapy a malé reference caps platí pro tento legacy profil.

## Základ2H a výchozí legacy profil

FÁZE2G je lokální commit `e9a003cb532c65c0acc54b5bd4f5930d4ce0fdd8`.
FÁZE2H obsahuje společný pilotní Rate PID přes samostatné `UTB_P_*` parametry,
ladění ANGLE P a skutečné opakované SITL ověření. Původní RAT_* konfigurace
mode41 a SHADOW, reference caps, arming a failsafe zůstávají.
[Ladění a omezení2H](UTB_PILOT_PID_TUNING_CS.md) ·
[souhrn a přesný manifest2H](UTB_PHASE_2H_SUMMARY_CS.md).
Aktivní hardware řízení ani ALT HOLD nejsou implementované.

ACRO používá lineární body rate mapu, ANGLE náklonovou mapu a vlastní attitude
P. Oba používají stejnou původní AP manual throttle politiku. Maxima lze
upravit přes AP parametry dostupné v GCS: `UTB_C_RMAX`, `UTB_C_PMAX`,
`UTB_C_YMAX`, `UTB_A_RMAX` a dosavadní `UTB_A_YMAX`, mimo aktivní pilotní režim.
Pevné caps nadále omezují ACRO na0,15/0,15/0,05rad/s; větší rozsah neověřen.
Ověření GCS parametrického API není tvrzení o testu konkrétního GUI klienta.

Podporovaná failsafe konfigurace: FS_GCS_ENABLE1, FS_THR_ENABLE1, FS_OPTIONS0.
Neslučitelná konfigurace se odmítá při vstupu. Nouzový UTB fault-stop s nulovým
tahem není obecná bezpečná letová obnova. Historické výsledky ANGLE jsou
v [2F reportu](UTB_ANGLE_MODE_SITL_VALIDATION_CS.md).

Projekt vytváří přehlednou a samostatně testovatelnou regulační vrstvu pro
výuku a experimenty s multikoptérou. ArduPilot 4.7.1 zajišťuje senzory,
odhad stavu, plánování, bezpečnostní infrastrukturu a výstupní cestu.
Vlastní UTB matematika umožňuje sledovat jednotlivé členy regulátoru a
porovnat výsledek s nezávislým modelem i původním řízením ArduPilotu.

[Společný index zdrojů a komentářový vzor](UTB_SOURCE_GUIDE_CS.md) seskupuje
vlastní UTB části a popisuje navazující komentářovou úpravu po checkpointu 2E.

Začít lze [mapou všech souborů](UTB_FILE_MAP_CS.md),
[přehledem dokončených a otevřených částí](UTB_PROJECT_STATUS_CS.md) a
[návrhem Git checkpointu](UTB_GIT_CONSOLIDATION_CS.md).

## Co je vlastní a co přebíráme

| Část | Implementace a rozsah |
|---|---|
| Rate PID | Vlastní [AP_UTB_RateController](../libraries/AP_UTB/AP_UTB_RateController.cpp): tři osy, integrál, D z měření, filtr, directional integration gate. Sdílená matematika pro SHADOW i aktivní SITL. |
| Attitude | Vlastní [AP_UTB_SITLAttitude](../libraries/AP_UTB/AP_UTB_SITLAttitude.cpp): reduced rotation P pro roll/pitch, omezení normy body rate. Pouze SITL. |
| Diagnostický mixer | Vlastní [AP_UTB_MotorMixer](../libraries/AP_UTB/AP_UTB_MotorMixer.cpp), frame class 1 a typy 12/18. Jeho motorové hodnoty nejsou aktivním výstupem. |
| Aktivní alokace | Původní AP_Motors. [SITLAllocation](../libraries/AP_UTB/AP_UTB_SITLAllocation.cpp) zpětně rekonstruuje normalizovaný přidělený povel pro podporovaný anti-windup kontrakt. |
| Odhad a senzory | AP_AHRS/AP_AHRS_View, INS a původní EKF. UTB nepřidává vlastní EKF. |
| SHADOW reference | AP RC kalibrace/deadzone/reverse a statická ACRO rate/expo mapa. Interní diagnostický generátor je oddělený, DISARMED a za BENCH přepínačem. |
| Integrace a bezpečnost | Původní scheduler, arming/failsafe, HAL/DShot a AP_Motors. Vlastní adaptér aktivní regulace je compile-time dostupný pouze v SITL. |

## Jak funguje kaskáda

V mode 41 `UTB_RATE_SITL`, `UTB_S_OWNER=1`, `UTB_S_ATT=1` vede konečná
roll/pitch reference do vlastního attitude P. Ten vrací požadovanou body rate,
vlastní rate PID vrací normalizované R/P/Y a adaptér je předává původnímu
AP_Motors. Standardní výstup ovlivňuje skutečný ArduPilot SITL plant;
AHRS/INS uzavírají zpětnou vazbu. Podrobný scheduler a diagramy jsou v
[mapě](UTB_FILE_MAP_CS.md) a [attitude architektuře](UTB_ATTITUDE_CONTROLLER_ARCHITECTURE_CS.md).

Throttle je převzatý ruční povel. UTB nemá výškovou smyčku. Změřená výška a
vertikální rychlost slouží v experimentu pouze k zastavení při opuštění
obálky. Yaw reference je body yaw rate; heading hold chybí. `S_ATT=0` zachovává
RATE experiment, `S_ATT=2` je výslovná srovnávací AP attitude větev.

Na H743 běží původní AP regulace a motorová cesta. UTB paralelně vzorkuje
stav, počítá PID a diagnostický mixer a předává snapshoty logger workeru.
Mode 40 `UTB_ACRO` zůstává nearmovatelný. Aktivní modes41/42/43, jejich parametry,
attitude/rate helpery a allocation feedback adaptér jsou z H743 sestavení vyloučeny
i při vynucení build option `UTB_RATE_SITL`.

## Co je skutečně ověřeno

| Oblast | Důkaz a omezení |
|---|---|
| Matematika PID/mixeru | [1H-B](UTB_PID_MIXER_VALIDATION_CS.md), nezávislé rovnice a C++ testy. |
| H743 SHADOW | [1H-C skutečný experiment](UTB_SYNTHETIC_H743_VALIDATION_CS.md): 8000/8000 uložených platných sad při přibližně 400 Hz, nenulový PID, DISARMED bez RC. Fyzické RCOU 1–4 byly 1000. Nejde o UTB uzavřenou regulační smyčku. |
| Aktivní rate SITL | [2A](UTB_ACTIVE_SITL_VALIDATION_CS.md), [2B](UTB_PHASE_2B_SUMMARY_CS.md), [gains a omezení](UTB_RATE_PID_OPTIMIZATION_CS.md). |
| Anti-windup | [2B kontrakt](UTB_RATE_ANTIWINDUP_CS.md): volitelný `S_AW=1` v podporované SITL alokaci. Výchozí `S_AW=0` při saturaci zastavuje; nejde o výchozí zapnutý aktivní anti-windup. |
| Aktivní attitude SITL | [2C](UTB_ATTITUDE_SITL_VALIDATION_CS.md): 37 skutečných sessions, 207200 úplných attitude sad; vedle toho dvě RATE regrese. Vše v omezené obálce. |
| Historický software základ před2F/2G | [Stav k 17. 10.](UTB_PROJECT_STATUS_CS.md): 98 C++ / 128 Python testů, 30 946 matematických vzorků, finální SITL a H743 build/guard kontroly. Historický [souhrn 2C](UTB_PHASE_2C_SUMMARY_CS.md) dokládá širší matici attitude experimentů. |

Výsledky nedokazují letovou způsobilost aktivního UTB na H743. Sdílené logger
mutexy, startup ztráty a download/EKF problém jsou otevřené. Hardwarové
closed-loop ověření ani fyzikální identifikace současného dronu nejsou hotové.

## Parametry, testy a výsledky

Parametry definuje [AP_UTB::var_info](../libraries/AP_UTB/AP_UTB.cpp), registruje
[Parameters.cpp](../ArduCopter/Parameters.cpp). Rate gains jsou
`UTB_RAT_{RLL,PIT,YAW}_{P,I,D,IMAX,D_HZ}`; firmware defaulty jsou nuly.
SITL testy nastavují experimentální gains explicitně. `UTB_S_ARP/S_APP` jsou
attitude P, `S_RMAX` limit rate, `S_ANG` amplituda a `S_PROFILE` konečný profil.
Jejich hodnoty a aktivní konfigurace se během experimentu zmrazují.

C++ testy jsou v [AP_UTB/tests](../libraries/AP_UTB/tests/wscript), benchmark
test v [AP_Logger/tests](../libraries/AP_Logger/tests/test_utb_bench.cpp).
Python unit testy a skutečné SITL runnery jsou jednotlivě popsané v mapě.
Offline model [utb_closed_loop.py](../Tools/utb/tools/utb_closed_loop.py) je jiný
druh důkazu než skutečné řízení procesu ArduPilot SITL.

Experimentální BIN/tlog, CSV, grafy, JSON metriky, buildy a hash manifesty
zůstávají v ignorovaném `tmp/`. Například:

- [2C výsledky a grafy](../tmp/utb-2c-20261016_152820/plots/), [manifest](../tmp/utb-2c-20261016_152820/artifact-manifest.json).
- [2B kompletní archiv](../tmp/utb-2b-20261014_164442/), [souhrnný report](UTB_PHASE_2B_SUMMARY_CS.md).
- [H743 syntetické měření](../tmp/utb-synthetic-h743-20261012_150527/), [report](UTB_SYNTHETIC_H743_VALIDATION_CS.md).

Tyto relativní odkazy jsou funkční v tomto checkoutu. Samotný nový clone
ignorované experimentální archivy neobsahuje; musí se přenést samostatně
včetně manifestů. Malý verzovatelný [demo plant](../Tools/utb/tools/utb_1he_demo_plant.json)
obsahuje demonstrační data, nikoli identifikovaný fyzický dron.

## Sestavení v existujícím WSL prostředí

Z kořene repository, bez instalace dalších závislostí:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
.venv/bin/python ./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf --targets bin/arducopter,tests/test_utb,tests/test_utb_sitl_rate,tests/test_utb_sitl_attitude,tests/test_utb_ap_allocation -j6
```

H743 SHADOW benchmark build (bez uploadu):

```sh
.venv/bin/python ./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH --disable-UTB_RATE_SITL
.venv/bin/python ./waf copter -j6
```

Přepínače jsou nezávislé a mají default OFF. Varianty H743 BENCH OFF a UTB OFF:
`--enable-UTB --disable-UTB_BENCH --disable-UTB_RATE_SITL`, resp.
`--disable-UTB --disable-UTB_BENCH --disable-UTB_RATE_SITL` při `configure`.
Stejný build adresář další sestavení přepíše; pro experiment nejprve archivovat
artefakt, přesné argv a SHA-256. Identita výsledků 2C je v jeho souhrnu.
Potřebné jsou root [waf](../waf), [wscript](../wscript),
[Copter wscript](../ArduCopter/wscript),
[build options](../Tools/scripts/build_options.py), library/test wscript,
submoduly a původní hwdef řetězec. Podrobnosti prostředí:
[WSL postup](../README_BUILD_WSL_CS.md).

## Spuštění existujících testů

SITL sestavit výše uvedeným SITL příkazem a spouštět po jednom. Každý výstupní
adresář musí být nový; předchozí výsledky nemazat. Následující runnery vytvářejí
vlastní lokální simulátor a nepřijímají sériový port fyzické FC.

```sh
build/sitl/tests/test_utb
build/sitl/tests/test_utb_sitl_rate
build/sitl/tests/test_utb_sitl_attitude
build/sitl/tests/test_utb_ap_allocation
UTB_AUDIT_EVIDENCE="$PWD/tmp/utb-1hd-offline-20261012_162639/download-audit/evidence.json" \
  .venv/bin/python -B -m unittest discover -s Tools/utb/tests -p 'test_utb*.py' -v
.venv/bin/python -B Tools/utb/tools/test_utb_active_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-basic --suite basic --speedup 2
.venv/bin/python -B Tools/utb/tools/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-disarmed --suite disarmed
.venv/bin/python -B Tools/utb/tools/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-rate --suite rate
.venv/bin/python -B Tools/utb/tools/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-attitude --suite preliminary
```

Python archive regression potřebuje uvedený archivní fixture; bez něj mohou
být některé testy SKIP. Není dovoleno takový výsledek vydávat za 128 PASS.
`preliminary` nastavuje vlastní P=2; vybrané P=3 se explicitně zkouší pomocí
`--suite repeat --gain 3`. Starší skeleton runner používá vlastní historický
testovací postup; aktuálně doloženou FÁZI 0 regrese a její wrapper eviduje 2C archiv.
Hardwarové capture/download nástroje se pro tyto SITL kontroly nespouštějí.

## Další vývoj

Konsolidační checkpoint byl vytvořen lokálně jako `5a32da404dafd19546439bf6886ab9adb831f4c0`.
[Návrh a audit](UTB_GIT_CONSOLIDATION_CS.md) zachycují jeho původní obsah.
Navazující komentáře a soustředění kódu v AP_UTB jsou zatím necommitnuté.
Výšku a případně později X/Y plánujeme adaptovat z předchozí bakalářské práce.
Její příslušný source je dosud neověřený externí vstup; algoritmy, rozhraní
a licenční/provenienční návaznost musí být posouzené až po dodání zdrojů.
Nejde o automatické zadání nových letových funkcí. Aktivní hardware potřebuje
samostatný návrh, bezpečnostní posouzení a nové ověření.
