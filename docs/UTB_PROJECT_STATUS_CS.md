# UTB: současný funkční stav

Aktuální FÁZE3A nahrazuje pilotní42/43 devíti vlastními moduly s původní
AP matematikou. [Architektura, výsledky a omezení](UTB_PHASE_3A_CS.md).
Nová cesta má společný Rate/PID, native rate-only ACRO a quaternion ANGLE,
parametry UT3_, původní arming/spool/failsafe a H743 active compile-out.
Finálních19 pilotních SITL scénářů prošlo včetně roll/pitch flipů, ±45° ANGLE,
RC/GCS failsafe a přechodů. Hardware aktivní řízení není ověřené ani povolené.

## Historické checkpointy2F–2I

Následující hodnoty, BF profily, omezení a počty testů popisují původní
experimentální implementaci, kterou pilotní3A nahrazuje. Nejsou popisem nové cesty.

Stav k **19. 10. 2026**, větev `test/skystars-5inch`.

FÁZE2I přidává experimentální profil `UTB_BF_TYPE=1`: Actual Rates,
samostatné RP/Yaw maximum a citlivost středu/expo, ANGLE limit do60° a nezávislý
rate cap. Výchozí TYPE=0 zachovává2H. [Parametry, skutečná měření a omezení](UTB_PILOT_RATES_CS.md).
PID gains a generický3kg model zůstávají beze změny. Vyšší nastavitelná hodnota
není automaticky prokázaná dosažená rychlost ani schválení pro hardware.
Aktivní profily jsou pouze SITL; plné roll/pitch rotace přes180° zůstávají neověřené
se zachovaným attitude guardem75°. Nové ověření:133 C++/156 Python a30946 nezávislých matematických vzorků;
20 pilotních legacy sad,5 mode41/basic sad a packed SHADOW prošlo.
RP peak přibližně659°/s u670 reference, Yaw337°/s u500 reference.
ANGLE krátké menší kroky se vracejí; delší45° kosinusově kompenzovaný pokus
končí vertical guardem i při náklonu44,95°. Plný45/60° rozsah není PASS.
Yaw otáčka přes360° v obou směrech ověřena, roll/pitch flip nikoli.
Celá letová obálka2I tedy zůstává nedokvalifikovaná.
Následující souhrn2H je historický základ.

## Ověřený základ2H

FÁZE2G je lokální commit `e9a003cb532c65c0acc54b5bd4f5930d4ce0fdd8`.
FÁZE2H obsahuje společný pilotní Rate PID přes samostatné `UTB_P_*` parametry,
ladění ANGLE P a skutečné opakované SITL ověření. Původní RAT_* konfigurace
mode41 a SHADOW, reference caps, arming a failsafe zůstávají.
[Ladění a omezení2H](UTB_PILOT_PID_TUNING_CS.md) ·
[souhrn a přesný manifest2H](UTB_PHASE_2H_SUMMARY_CS.md).
Aktivní hardware řízení ani ALT HOLD nejsou implementované.

- Implementováno: samostatná pilotní banka, entry snapshots a runtime gain guards.
- Unit/SITL ověřeno:128 C++ /151 Python;30946 vzorků/29 matematických scénářů;
  tři nominální opakování před i po, obě znaménka, kombinace, přechody,
  J±10% perturbace a vyjmenované RATE/ANGLE/ACRO/RC/GCS/SHADOW regrese.
- Build ověřeno:SITL/H743 feature kombinace; aktivní H743 řízení compile-out.
- Hardware neověřeno:nový aktivní tune, fyzická5″ dynamika a aktivní UTB let.
- Plánováno:vyšší reference, širší planty, ALT HOLD/XY a hardware autorita.

Následující reporty2F/2G zachycují historický stav a tehdejší gains/počty,
nikoli aktuální necommitnuté soubory.

## Dokončený základ: pilotní UTB ANGLE, FÁZE 2F

Implementován samostatný **mode42 UTB_ANGLE pouze SITL**, skutečná AP RC
kalibrace/RCMAP/deadband/reverse → vlastní původní attitude P → vlastní
původní rate PID → původní AP_Motors. Ruční throttle používá původní AP mapu,
bez angle boost a bez výškové regulace. Mode40 zůstává nearmovatelný;
automatické profily mode41 jsou zachovány.

Ověřený rozsah: oba směry roll/pitch3° a5°, aktivní návrat do roviny,
kombinované RP, yaw±0,05rad/s, malé ruční throttle změny, GCS/RC výběr,
STABILIZE↔ANGLE přechody a RC/GCS výpadek ve vymezené konfiguraci.
Norma reference nejvýše5°, větší rozsah ani kompletní manuální takeoff/landing
nejsou ověřeny. Unit sada obsahuje 112 C++ testů včetně dvou CSV export testů
a 135 Python testů; není přímo srovnatelná se starším součtem 98 níže.

**Omezení:** FS_OPTIONS16 („continue pilot control“) je neslučitelné s
vlastní fail-stop politikou při GCS výpadku; skutečný test skončil pádem
simulace. Ověřený runner používá explicitně FS_OPTIONS0 a původní AP RTL.
Další kombinace failsafe voleb nejsou automaticky schválené. Fail-stop
s nulovým tahem není bezpečná obecná letová obnova.

H743 active ANGLE zůstává compile-out i s vynucenou active build option.
Žádný COM5, upload, fyzické parametry nebo motorové testy v této fázi.
[Architektura](UTB_ANGLE_MODE_ARCHITECTURE_CS.md),
[SITL validace](UTB_ANGLE_MODE_SITL_VALIDATION_CS.md),
[Souhrn, manifest a ruční návod](UTB_PHASE_2F_SUMMARY_CS.md).

Následující původní oddíly zachycují stav checkpointu organizace před2F;
jejich historické počty, binárky a fyzické výsledky se nepřepisují výsledky2F.

## Hotovo v současném vymezeném rozsahu

| Funkce | Stav | Důkaz / hranice |
|---|---|---|
| Vlastní tříosý Rate PID | SOURCE/UNIT/SITL/H743 SHADOW | [RateController.cpp](../libraries/AP_UTB/AP_UTB_RateController.cpp), [1H-B matematika](UTB_PID_MIXER_VALIDATION_CS.md), [1H-C H743](UTB_SYNTHETIC_H743_VALIDATION_CS.md). H743 pouze výpočet, fyzický plant řídí AP. |
| Vlastní attitude P | SOURCE/UNIT/SITL | [SITLAttitude.cpp](../libraries/AP_UTB/AP_UTB_SITLAttitude.cpp), [2C](UTB_ATTITUDE_SITL_VALIDATION_CS.md): reduced rotation roll/pitch, body yaw rate, bez heading hold. |
| Aktivní rate a attitude SITL | SOURCE/UNIT/SITL | Mode 41, guard `AP_UTB_RATE_SITL_AVAILABLE`, výhradní dispatch a původní AP_Motors. 37 attitude sessions + 2 RATE regrese v 2C. |
| Definovaná politika saturace | SOURCE/UNIT/SITL | **Default `UTB_S_AW=0` je SATURATION stop.** Volitelné `S_AW=1` používá ověřený normalized allocation feedback. Není výchozím aktivním anti-windupem; [2B kontrakt](UTB_RATE_ANTIWINDUP_CS.md). |
| Matematická validace | UNIT/SITL/H743 SHADOW | Nezávislé PID/mixer rovnice, float meze, rotace, allocation projection a recurrence. Hodnoty nejsou fyzikální torque měření. |
| Testy a reprodukce | UNIT/SITL/BUILD | Nejnovější regrese ze 17. 10.: **98 C++ / 128 Python testů**, 30 946 matematických vzorků / 29 scénářů, shodný CSV výstup se starším C++ executablem; [přehled zdrojů](UTB_SOURCE_GUIDE_CS.md). Historický [2C souhrn](UTB_PHASE_2C_SUMMARY_CS.md) dokládá širší experimentální matici. |

„Hotovo“ znamená implementovaný a ověřený experimentální kontrakt. Celkový
aktivní autopilot UTB pro hardware dokončený není. Výchozí firmware gains,
S_OWNER/S_RUN/S_ATT zůstávají nulové; zvolené SITL gains nastavují runnery.

## Ověření checkpointu organizace před2F, 17. 10. 2026

- C++ a Python zdroje byly uspořádané a okomentované, pracovní deklarace
  sjednocené; byly odstraněné dva zbytečné interní ukazatele. Matematické
  výrazy, bezpečnostní kontroly a pořadí controller volání se neměnily.
- C++ je v `libraries/AP_UTB/`, Copter adaptéry v `Copter/`, C++ testy v
  `tests/`. Python je samostatně v `Tools/utb/tools/` a `Tools/utb/tests/`.
  Nezávislá BENCH instrumentace zůstává u AP_Logger kvůli UTB OFF baseline.
- Prošlo 47 core + 9 allocation + 12 benchmark + 14 attitude + 9 rate +
  3 synthetic + 4 matematické C++ testy, celkem **98**. Python: **128**.
- Finální SITL regrese: DISARMED integrace, RATE roll/pitch po 3200 sadách,
  attitude roll s 5600 sadami, SHADOW 200/400 Hz a UTB OFF runtime absence.
  SHADOW dosáhl přibližně 200,0009 / 399,9933 Hz v měřených SITL úsecích.
  Test záměrně vypíná logger, vyvolává plnou frontu a kontroluje nekompletní
  sady; jeho drops nejsou výsledek stabilního hardware benchmarku.
- Sestaveno SITL UTB ON/BENCH ON, SITL UTB ON/BENCH OFF, SITL UTB OFF/BENCH OFF,
  H743 UTB ON/BENCH ON a H743 UTB ON/BENCH ON bez logování. Aktivní UTB SITL
  symboly na H743 chybějí i při vynuceném build option.
- Param/logger metadata, flake8 a diff check prošly. Astyle upravených částí
  prošel; původní jednořádkový getter `sitl_antiwindup()` v AP_UTB.h má
  doloženou starší formátovací odchylku, která se neměnila.
- H743 text je oproti předchozímu layout buildu větší o 12 B, BSS o 4 B.
  Tři porovnané producer/worker/consumer funkce mají stejné compiler stack
  operace. To nedokládá skutečný stack watermark ani WCET na hardware.

Důkazy a SHA-256: [report](../tmp/utb-readable-20261017_084027/REPORT_CS.md),
[ověření](../tmp/utb-readable-20261017_084027/verification-summary.json),
[finální buildy](../tmp/utb-readable-20261017_084027/final-build-results.json).
Tyto ignorované archivy je nutné přenášet zvlášť; nový clone je neobsahuje.

Buildy a testy proběhly před lokálním commitem nad shodnými C++ a Python
zdroji. Archivovaný APJ má původní build identitu z pracovního stromu nad
`5a32da404d`; vytvoření commitu jeho metadata zpětně nemění. Při současném
checkpointu se firmware nepřekládá znovu, nenahrává ani fyzicky netestuje.
Aktuální uspořádání zdrojů je software/build/SITL ověřené; jeho hardware
časování a stack watermark zůstávají neověřené. Předchozí fyzické výsledky
níže se vztahují výhradně k tehdy zaznamenaným firmware a konfiguracím.

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
| Aktivní hardware closed loop | Mode41, nový mode42 a aktivní adapter jsou na H743 compile-out; žádné aktivní UTB hardware ověření. |

Předchozí BP je použita pro kontinuitu projektu a plánování. Příslušný source
výšky/XY není tímto auditem dodán ani ověřen; nelze uvádět její konkrétní
algoritmy, funkce nebo kompatibilitu jako hotové.

## Plánované navazující kroky

Jde o návrh pořadí práce, nikoli o povolení aktivního hardware řízení:

1. Dořešit diagnostiku logger/download/EKF, sdílené mutexy a zopakovat
   vymezená DISARMED měření časování/stacku pro zvolený finální firmware.
2. Doložit fyzikální parametry a dynamiku skutečného 5″ dronu; dále ověřit
   saturaci, anti-windup a robustnost v odpovídajícím simulačním kontraktu.
3. Samostatně navrhnout a schválit přechod k aktivnímu hardware ověřování.
   Současné H743 buildy takový přechod neumožňují.
4. Po stabilizaci nižších smyček navrhnout heading hold, výšku a XY řízení.
   Případný kód z původní BP musí nejdříve projít source auditem a adaptací.
5. Vlastní AUTO, trajektorie, překážky a výukové UI řešit jako další oddělené
   návrhy; v současném AP_UTB nejsou implementované.

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

FÁZE 2E ani navazující úprava organizace nepřidávají nový regulátor nebo
opravu loggeru. Původní konsolidační commit je `5a32da404d`; tento navazující
lokální checkpoint ukládá uspořádání, komentáře, čitelnost a aktuální stav.
