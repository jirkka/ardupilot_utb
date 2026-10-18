# Pilotní režimy UTB

| Režim | Číslo | Pilot Roll/Pitch | Výška | Stav |
|---|---:|---|---|---|
| UTB_ACRO historický | 40 | žádná aktivní autorita | žádná | DISARMED skeleton, arming zakázaný |
| UTB_RATE_SITL | 41 | automatický experiment rate/attitude | zděděný experimentální throttle | dosavadní SITL platforma |
| UTB_ANGLE | 42 | náklonová reference, střed vyrovnává | manual throttle | omezená aktivní SITL implementace |
| UTB_ACRO_SITL | 43 | přímé body rates, střed zastaví rotaci | manual throttle | aktivní SITL implementace, rozsah ověření v2G/2H reportech |
| budoucí ALT HOLD | nepřiděleno | attitude target | altitude/vertical-speed controller | pouze plán, neimplementováno |

42 a43 jsou samostatně volitelné přes GCS a původní FLTMODE/RC infrastrukturu.
Přímý přechod nevyžaduje STABILIZE ani konkrétní pořadí vstupu. Parametry,
fault policy, omezení a skutečně provedené scénáře popisují
[architektura](UTB_ACRO_MODE_ARCHITECTURE_CS.md) a
[validace](UTB_ACRO_MODE_SITL_VALIDATION_CS.md).

Budoucí autonomní XY řízení je zamýšlené až jako nadstavba ALT HOLD.
V této fázi není přidán výškový/XY regulátor, nový mixer ani hardware autorita.

## Společný pilotní tune2H

ACRO43 i ANGLE42 používají `UTB_P_*` gains; vnější P platí jen pro ANGLE.
Parametry upravovat mimo oba pilotní režimy; runtime změna latched stop
neruší. RAT_* zůstává pro mode41/SHADOW. H743 aktivní pilotní řízení neobsahuje.
[Porovnání a omezení](UTB_PILOT_PID_TUNING_CS.md). ALT HOLD zůstává plán.
