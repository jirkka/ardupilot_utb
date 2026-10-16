# UTB: současný funkční stav

Stav FÁZE 2E k 16. 10. 2026. Platí pro pracovní strom nad HEAD
`687f825d228b12a64d36651072aed19356178217`, nikoli pro samotný HEAD.
[Rozcestník](UTB_README_CS.md) · [Mapa implementace](UTB_FILE_MAP_CS.md) ·
[Git a ověření kontinuity](UTB_GIT_CONSOLIDATION_CS.md).

SOURCE = zkontrolovaný source; UNIT = unit/model test; SITL = skutečný proces
ArduPilot simulátoru; H743 = skutečný fyzický experiment v uvedeném rozsahu;
UNVERIFIED = nedoložené. Build úspěch sám není H743 validace.

## Hotovo v současném vymezeném rozsahu

| Funkce | Stav | Důkaz / hranice |
|---|---|---|
| Vlastní tříosý Rate PID | SOURCE/UNIT/SITL/H743 SHADOW | [RateController.cpp](../libraries/AP_UTB/AP_UTB_RateController.cpp), [1H-B matematika](UTB_PID_MIXER_VALIDATION_CS.md), [1H-C H743](UTB_SYNTHETIC_H743_VALIDATION_CS.md). H743 pouze výpočet, fyzický plant řídí AP. |
| Vlastní attitude P | SOURCE/UNIT/SITL | [SITLAttitude.cpp](../libraries/AP_UTB/AP_UTB_SITLAttitude.cpp), [2C](UTB_ATTITUDE_SITL_VALIDATION_CS.md): reduced rotation roll/pitch, body yaw rate, bez heading hold. |
| Aktivní rate a attitude SITL | SOURCE/UNIT/SITL | Mode 41, guard `AP_UTB_RATE_SITL_AVAILABLE`, výhradní dispatch a původní AP_Motors. 37 attitude sessions + 2 RATE regrese v 2C. |
| Definovaná politika saturace | SOURCE/UNIT/SITL | **Default `UTB_S_AW=0` je SATURATION stop.** Volitelné `S_AW=1` používá ověřený normalized allocation feedback. Není výchozím aktivním anti-windupem; [2B kontrakt](UTB_RATE_ANTIWINDUP_CS.md). |
| Matematická validace | UNIT/SITL/H743 SHADOW | Nezávislé PID/mixer rovnice, float meze, rotace, allocation projection a recurrence. Hodnoty nejsou fyzikální torque měření. |
| Testy a reprodukce | UNIT/SITL/BUILD | [2C souhrn](UTB_PHASE_2C_SUMMARY_CS.md): 79 C++/128 Python testů, sedm build variant a guards. Nově spuštěné 2E versus převzaté důkazy rozlišuje Git audit. |

„Hotovo“ znamená implementovaný a ověřený experimentální kontrakt. Celkový
aktivní autopilot UTB pro hardware dokončený není. Výchozí firmware gains,
S_OWNER/S_RUN/S_ATT zůstávají nulové; zvolené SITL gains nastavují runnery.

## Částečně ověřeno

| Oblast | Co je doloženo | Co zůstává otevřené |
|---|---|---|
| H743 SHADOW | [1H-C](UTB_SYNTHETIC_H743_VALIDATION_CS.md): 8000/8000 nenulových PID sad za 20 s, přibližně 400 Hz, DISARMED, fyzická RCOU 1000. | Aktivní UTB uzavřená smyčka, let, všechny chybové/saturační situace. |
| CPU a časování | AP_HAL hardware časy, PM a benchmark měření; uvnitř RUNNING PM.NLon=0, dt 2456–2548 µs. | WCET, hard deadlines při všech zátěžích, izolovaný PID-only overhead. UBST[8] jsou task budget overruns, UBST[9] periody nad nominálem. |
| Logger | Packed UTBC umožnil úplné 200/400Hz záznamy ve vybraných oknech; bounded queue, oddělený worker. | Startup drop/missing a dlouhá perioda, nepřímé blokování přes sdílené mutexy, UNKNOWN atribuce, download/EKF příčina. Nulové potvrzené kolize nevylučují blokování. |
| Worker lifecycle/stack | Skutečný H743 watermark: v 1H-C minimum 1760 B z oblasti 3504 B, requested 3072 B; IDLE 2B jiné okno s 2432 B. | Rezerva pro libovolnou budoucí zátěž, worst-case chybové větve. Čísla se vztahují k odlišným experimentům. |
| Saturace/AW | [2B](UTB_RATE_ANTIWINDUP_CS.md): directional gate v podporovaném SITL BF_X_REV AP kontraktu, reverse/unwind a fail-stop. | Obecné frames, scripting/boost/slew/motor loss mimo validovaný kontrakt, aktivní hardware. |
| Robustnost attitude | [2C](UTB_ATTITUDE_SITL_VALIDATION_CS.md): malé reference, inertia ±10 %, noise, jeden další cyklus delay a další konkrétní varianty. | Obecný stability/delay margin. Dva robust plateaus nedosáhly 2% settling; funkční tail tolerance byla splněná. |
| Fyzikální model | Offline simulační/identifikační nástroje a demo data, [1H-E](UTB_PLANT_IDENTIFICATION_CS.md). | Skutečná kompletní identifikace dnešního 5″ dronu. Demo parametry nejsou naměřený model. |

## Zatím chybí

| Funkce | Aktuální situace / plán |
|---|---|
| UTB výška | Žádný aktivní UTB altitude controller. AHRS výška/vz pouze hlídají SITL obálku; throttle je ruční/převzatý. Původní BP je plánovaný externí vstup pro adaptaci. |
| UTB horizontální poloha | Žádná UTB XY smyčka. Případné převzetí BP až po ověření příslušného source. |
| Heading hold | Yaw body rate není yaw/heading regulátor. |
| Autonomní mise | Původní AP módy nejsou vlastní UTB mise. |
| Překážky | Není UTB detekce ani vyhýbání. |
| Studentské rozhraní | Dokumentace a skripty existují, vlastní výukové UI chybí. |
| Aktivní hardware closed loop | Mode 41 a aktivní adapter jsou na H743 compile-out; žádné aktivní UTB hardware ověření. |

Předchozí BP je použita pro kontinuitu projektu a plánování. Příslušný source
výšky/XY není tímto auditem dodán ani ověřen; nelze uvádět její konkrétní
algoritmy, funkce nebo kompatibilitu jako hotové.

## Historické reporty a zdánlivé rozpory

Věcný obsah původních reportů zůstává zachovaný. Audit 2E výslovně eviduje
odstranění jediného nadbytečného prázdného řádku na EOF attitude validation
reportu kvůli úplnému staging diff checku. Rozpor se vykládá podle fáze a platformy:

| Historický dokument / tvrzení | Současná interpretace |
|---|---|
| [UTB_ARCHITEKTURA_CS.md](UTB_ARCHITEKTURA_CS.md), FÁZE 0/1: žádná motorová autorita, FÁZE 2 je plán | Stále platí pro H743 SHADOW a mode 40. Od 2A existuje oddělený aktivní SITL mode 41; jeho současný stav popisuje 2C. |
| [UTB_CONTROL_THEORY_CS.md](UTB_CONTROL_THEORY_CS.md): SHADOW ONLY anti-windup a původní frame 12 kontrakt | Historická matematická specifikace fáze 1. Typ 18 byl explicitně přidán v [BF_X_REV reportu](UTB_BFX_REV_REPORT_CS.md); aktivní allocation feedback je novější SITL-only kontrakt 2B. |
| [UTB_PHASE_2B_SUMMARY_CS.md](UTB_PHASE_2B_SUMMARY_CS.md): attitude není hotový / 2C nezačala | Platilo na konci 2B; [2C souhrn](UTB_PHASE_2C_SUMMARY_CS.md) dokládá pozdější vlastní attitude. |
| [1H-A software report](UTB_BENCH_SOFTWARE_REPORT_CS.md): hardware neověřeno | Stav před prvními fyzickými experimenty; pozdější packed/no-RC reporty dokládají konkrétní H743 okna. |
| [Throughput audit](UTB_LOGGER_THROUGHPUT_AUDIT_CS.md): původní vícezprávový 400Hz tok problematický | Pozdější packed transport a cycle selection mají [samostatný recovery report](UTB_400_RECOVERY_20261011_CS.md). Nelze přenést výsledek na libovolné schéma loggeru. |
| [2C](UTB_ATTITUDE_SITL_VALIDATION_CS.md): 79 testů versus [2B](UTB_PHASE_2B_SUMMARY_CS.md): 69 | Různé sady: 2B zahrnuje 4 independent math testy; 2C hlavní sada je 47+9+9+14. Nejde o ztracené testy. |

FÁZE 2E nepřidává regulátor ani opravu loggeru. Připravuje pouze mapu,
stav, zálohu a konsolidační commit ke schválení.
