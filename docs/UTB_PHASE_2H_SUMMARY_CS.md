# FÁZE2H: společný pilotní tune a regresní checkpoint

Rodič: lokální2G commit `e9a003cb532c65c0acc54b5bd4f5930d4ce0fdd8`, vytvořený
po přesném ověření26 SHA-256/Git blob souborů. Tento dokument uzavírá
následující2H checkpoint. SHA tohoto commitu je v Git historii a závěrečném
reportu. Žádný push/rebase/reset nebo mazání historie, COM5/upload/hardware
test. ALT HOLD není implementovaný.

[Ladění, gains, grafy, regrese a omezení](UTB_PILOT_PID_TUNING_CS.md).
Důkazy:`tmp/utb-2h-20261018_093406`. `source-manifest-final.json` obsahuje SHA-256/Git blob,
přesnou patch a tar včetně nových souborů. `firmware-artifacts-final.json`
rozlišuje prototypové a přesné finální-source buildy a jejich SHA/velikosti.

## Stav

Implementováno:17 oddělených pilotních AP parametrů, frozen entry gains,
bounds/runtime guards a stejný Rate PID tune42/43.
Unit/SITL ověřeno:128 C++ /151 Python;30946 vzorků/29 matematických scénářů;
tři opakování před i po, oba směry/kombinace/přechody, J±10%, RC/GCS a staré
RATE/ANGLE/SHADOW regrese. Build ověřeno:feature matice a nový finální-source
SITL/H743 build; aktivní pilotní řízení/parametry naH743 compile-out.
Hardware neověřeno:nový aktivní tune, skutečná5″ dynamika a aktivní let.
Plánováno:vyšší reference, širší modely, ALT HOLD/XY a hardware autorita.

## Přesný seznam souborů

15 upravených + 4 nových souborů.

| Stav | Soubor |
|---|---|
| upravený | [Tools/utb/README_CS.md](../Tools/utb/README_CS.md) |
| nový | [Tools/utb/tests/test_utb_pilot_tuning.py](../Tools/utb/tests/test_utb_pilot_tuning.py) |
| upravený | [Tools/utb/tools/utb_angle_sitl.py](../Tools/utb/tools/utb_angle_sitl.py) |
| upravený | [Tools/utb/tools/utb_pilot_sitl.py](../Tools/utb/tools/utb_pilot_sitl.py) |
| nový | [Tools/utb/tools/utb_pilot_tuning.py](../Tools/utb/tools/utb_pilot_tuning.py) |
| upravený | [docs/UTB_ARCHITEKTURA_CS.md](../docs/UTB_ARCHITEKTURA_CS.md) |
| upravený | [docs/UTB_FILE_MAP_CS.md](../docs/UTB_FILE_MAP_CS.md) |
| upravený | [docs/UTB_FLIGHT_MODES_CS.md](../docs/UTB_FLIGHT_MODES_CS.md) |
| nový | [docs/UTB_PHASE_2H_SUMMARY_CS.md](../docs/UTB_PHASE_2H_SUMMARY_CS.md) |
| nový | [docs/UTB_PILOT_PID_TUNING_CS.md](../docs/UTB_PILOT_PID_TUNING_CS.md) |
| upravený | [docs/UTB_PROJECT_STATUS_CS.md](../docs/UTB_PROJECT_STATUS_CS.md) |
| upravený | [docs/UTB_README_CS.md](../docs/UTB_README_CS.md) |
| upravený | [docs/UTB_SOURCE_GUIDE_CS.md](../docs/UTB_SOURCE_GUIDE_CS.md) |
| upravený | [libraries/AP_UTB/AP_UTB.cpp](../libraries/AP_UTB/AP_UTB.cpp) |
| upravený | [libraries/AP_UTB/AP_UTB.h](../libraries/AP_UTB/AP_UTB.h) |
| upravený | [libraries/AP_UTB/AP_UTB_AngleRC.h](../libraries/AP_UTB/AP_UTB_AngleRC.h) |
| upravený | [libraries/AP_UTB/Copter/mode_utb_angle_sitl.cpp](../libraries/AP_UTB/Copter/mode_utb_angle_sitl.cpp) |
| upravený | [libraries/AP_UTB/Copter/mode_utb_angle_sitl.h](../libraries/AP_UTB/Copter/mode_utb_angle_sitl.h) |
| upravený | [libraries/AP_UTB/tests/test_utb_acro_rc.cpp](../libraries/AP_UTB/tests/test_utb_acro_rc.cpp) |

Šest C++/header změn zavádí banku, gettery, bounds, entry snapshots a testy.
Původní matematika PID/mixer/attitude, AP_Motors/HAL/EKF, nativní ArduCopter,
arming/failsafe se nemění. Dva stávající Python runnery přebírají volitelný
model/speedup a úplnou časovou parametrickou evidenci. Strict P identita
zůstává, ale používá skutečný Kp místo historické konstanty. Nový tuning
runner a unit testy oddělují experiment a offline analýzu. Dokumenty
rozlišují historický2F/2G a aktuální2H stav.
