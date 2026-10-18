# FÁZE 2G: pilotní ACRO a společný pilotní adaptér v SITL

Rodič `671ccad97050115e87b9d8451313dd7403fc1e72`, větev `test/skystars-5inch`.
Implementovaný zdrojový rozsah je oddělený od skutečně dokončených experimentů
ve [validačním reportu](UTB_ACRO_MODE_SITL_VALIDATION_CS.md).
Aktivní hardware řízení není součástí tohoto kontraktu.

## Režimy a jediná řídicí cesta

- 40 UTB_ACRO: původní nearmovatelný režim, včetně force-arm zákazu.
- 41 UTB_RATE_SITL: původní automatické rate/attitude experimenty beze změny.
- 42 UTB_ANGLE: pilotní náklon, vlastní attitude → vlastní rate PID.
- 43 UTB_ACRO_SITL: pilotní body rates → stejný původní vlastní rate PID.

Číslo 43 bylo volné. Enum, instance, lookup, AVAILABLE_MODES, výhradní rate
dispatch i post-output capture jsou chráněné `AP_UTB_RATE_SITL_AVAILABLE`.
Ten vyžaduje UTB ON, aktivní SITL build option a skutečný `HAL_BOARD_SITL`.
Samotné `--enable-UTB_RATE_SITL` na H743 nevytvoří aktivní režim ani parametry.

`ModeUTBAcroSITL` je malý samostatný mode43 wrapper nad již existujícím
pilotním adaptérem `ModeUTBAngleSITL`. Dědičnost znovu používá RC validaci,
manual throttle, lifecycle, původní rate helper a capture; nevyjadřuje
závislost na předchozím spuštění ANGLE. Virtuální číslo režimu vybírá explicitní
větev. V ACRO se **nevolá** `_attitude.configure/update`; attitude objekt má
po resetu nula update calls. Quaternion se pouze čte pro diagnostiku a
experimentální tilt guard. Není druhá kopie PID ani reorganizace adresářů.

Fast slot volá právě jeden z mode41/42/43 callbacků a vrací se před původním
AP rate PID. Custom controller a rate thread jsou v těchto modes vyloučené.
Původní motor output zůstává jediný; za ním se čte AP alokace. AP attitude
objekt se používá pro původní manual throttle mapu a reset přechodové historie,
nikoli pro aktivní attitude nebo rate PID výpočet v ACRO.

## RC reference a souřadnice

Použijí se původní `channel_roll/pitch/yaw/throttle` vytvořené AP `init_rc_in()`
podle RCMAP. `RC_Channel::norm_input_dz()` zajišťuje skutečný trim, endpointy,
reverse a deadband. Zachovaná kontrola kalibrace, PWM rozsahu, AP validity,
radio failsafe a původního RC timeoutu odmítá neplatný vstup.
`AP_UTB_AcroRC` znovu používá tuto validaci z `AP_UTB_AngleRC`; jeho dočasná
úhlová reference se neaplikuje a nahradí se lineární body rate mapou:

```text
AP calibrated normalized sticks nR,nP,nY
  → raw = (nR*C_RMAX, nP*C_PMAX, nY*C_YMAX)
  → target = raw (bez expo, traineru, reference filtru a dynamic RATE_TC)
  → původní AP_UTB_SITLRate → původní AP_UTB_RateController
  → původní AP_Motors → SITL bfxrev
```

Kombinované ACRO RP vstupy nemají ANGLE kruhový clamp: každá osa má vlastní
maximum. Celkový target stále prochází původním rate norm guardem 0,6 rad/s.
FRD znaménka: roll+ pravé křídlo dolů; pitch+ příď nahoru; yaw+ doprava.
Vystředěná páčka je nula body rate při libovolném podporovaném náklonu;
nevytváří povel návratu do roviny. To musí doložit skutečný tilt hold po puštění
páčky, nikoli pouze unit výpočet nulového targetu.

## Parametry dostupné přes GCS/Mission Planner

Všechny nové parametry existují pouze v aktivním SITL buildu. Staré AP_Param
indexy 0–39 se nemění. Hard caps jsou soustředěné jako pojmenované `constexpr`
`AP_UTB_AcroRC::{ROLL_MAX,PITCH_MAX,YAW_MAX}`. Změna runtime parametru sama
nemůže zvýšit podporovanou obálku nad tyto meze.

| Parametr | Index | Default | Význam |
|---|---:|---:|---|
| UTB_C_ENABLE | 40 | 0 | Samostatný opt-in mode43, přesně1. |
| UTB_C_RMAX | 41 | 0,15 rad/s | Absolutní roll body rate maximum, hard cap0,15. |
| UTB_C_PMAX | 42 | 0,15 rad/s | Absolutní pitch body rate maximum, hard cap0,15. |
| UTB_C_YMAX | 43 | 0,05 rad/s | Absolutní yaw body rate maximum, hard cap0,05. |
| UTB_C_AW | 44 | 0 | Původní saturation-stop / volitelný allocation feedback1. |
| UTB_A_RMAX | 45 | 0,3 rad/s | ANGLE RP target norm limit, původní helper podporuje0,05–0,3. |

Původní `UTB_A_YMAX`38 dál určuje ANGLE yaw maximum do0,1rad/s.
0,15rad/s≈8,59°/s; 0,05rad/s≈2,86°/s. Nejde o FPV tune pro rychlé rotace.
V GCS vyhledej přesný název parametru, uprav jej **mimo pilotní režim** a pak
znovu vstup do mode. Mode entry přijímá jen konečné kladné hodnoty uvnitř
hard caps a zmrazí konfiguraci. Změna aktivních parametrů za běhu vyvolá
CONFIG latch. Zápis/čtení je původní AP parametrická cesta; Mission Planner
se neupravuje a konkrétní GUI klient nebyl tímto source rozšířením validován.

## Ruční throttle a lifecycle

ACRO i ANGLE volají jedinou stejnou `Mode::get_pilot_desired_throttle()` mapu
s původním `Mode::throttle_hover()` a AP hover parametrem. Nezavádí se druhá
throttle implementace ani entry hover override. `set_throttle_out(...,false,...)`
vylučuje angle boost. Zůstává původní AP spool mechanismus, manual mix0,1,
zero throttle/ground idle a standardní AP arming metody bez bypassu checks.
Žádný výškový nebo horizontální regulátor. GUIDED takeoff ve finite runneru
je jen oddělená testovací příprava; není mode entry podmínkou.

Po nativním LAND může původní vertikální PID při DISARMED ještě vytvářet malý
záporný raw požadavek; AP_Motors jej neaplikuje. Původní pilotní vstupní kontrola
jej nesprávně odmítala. Sdílená entry predicate nyní při DISARMED požaduje
konečnost tohoto neaktivního kontextu; při ARMED zachovává původní0..1 meze.
Raw hodnota se do UTB throttle nepřenáší. Nový vlastní manual požadavek začíná0
a vzniká jen z validovaného RC, původní AP disarmed/spool a arming zůstávají
beze změny. Unit i nativní LAND/SITL reprodukce jsou ve validačním reportu.

Při každém entry se resetují rate i attitude historie, feedback, reference,
počty a první PID vzorek se primuje s I=D=output0. Nepřenáší se starý integrátor
mezi mode instances. Exit resetuje oba kontexty a AP přechodovou historii.
AP nejprve volá nový `init`, následně starý `exit`, poté mění mode pointer;
na hlavním vlákně nevzniká druhý motor output během těchto callbacků.

Přímé přepnutí 42↔43 nevyžaduje centrované páčky ani mezirežim. Sticks dostanou
novou sémantiku v novém mode a throttle zůstává ve stejné mapě. Po resetu může
vzniknout malý konečný P krok; toto není matematicky dokonale spojitý bumpless
transfer. Derivace se primuje a target je omezen původními limity. Podporu
malých nenulových přechodů musí určit naměřené PID/PWM transienty a nezměněné
letové guards, nikoli pouhá úspěšná SET_MODE odpověď. Žádná skrytá AP regulace
nebo automatická výšková smyčka přechod nevyhlazuje. Pro krátké přechodové
experimenty runner potvrzuje skutečný mode/armed heartbeat a následně měří
explicitní dwell; nevkládá původní dodatečné třísekundové observer čekání,
protože při nenulové ACRO páčce by zbytečně zvětšovalo náklon. Skutečné MODE
TimeUS a všechny control samples zůstávají autoritativní offline časovou osou.

## Failsafe a fault jsou různé události

Entry i runtime konfigurace pilotních mode42/43 vyžadují explicitně
`FS_GCS_ENABLE=1`, `FS_THR_ENABLE=1`, `FS_OPTIONS=0`. Tím je odmítnuta například
FS_OPTIONS16 politika pokračování pilotního řízení, která v2F vedla k pádu
modelu. Nejde o řešení všech AP failsafe kombinací.

`set_failsafe_radio()` synchronně nastaví flag a vyvolá původní
`failsafe_radio_on_event()`; GCS check synchronně nastaví flag a zavolá
`failsafe_gcs_on_event()`. Původní `do_failsafe_action()` žádá RTL/LAND před
návratem do hlavního scheduleru. Ve single-thread400Hz konfiguraci mezi
nastavením flagu a tímto handoffem neběží další UTB callback. Native event
handlers nejsou měněné. GCS loss se testuje s odděleným nativním UDP RC,
protože běžící MAVLink overrides aktualizují GCS last-seen i bez heartbeat.

Samostatný neplatný/stale RC, state/INS/gyro/target/allocation fault nebo runtime
změna konfigurace dál vytváří původní latched fault-stop: žádný starý povel,
nulový požadavek/spool shutdown, návrat vyžaduje explicitní exit a zdravý entry.
**Tento nouzový stop může za letu způsobit pád; není obecně bezpečný failsafe.**
Zvláště změna FS_OPTIONS za aktivního letu není podporovaný postup; její
DISARMED odmítnutí/latched stop neprokazuje bezpečné zotavení ve vzduchu.

Zachované guards: frame1/18, nominal400Hz, FSTRATE0, čerstvý AHRS/INS≤10ms,
nezměněný primary gyro/INS timestamp, původní dt interval, gyro norm≤2rad/s,
skutečný RP do35°, výška±2m od entry, |vz|≤0,5m/s, zákaz shadow/synthetic/scripting
konfliktu. Výška a vz slouží pouze jako experimentální stop obálka.

## Logování a důkaz vlastnictví

Žádné nové log ID ani nové zápisy v hardware fast path. Znovu se používají
původní SITL UARX/UAMT/UAFB/UALT/UAAT/UAST/UARC se společným TimeUS/Seq.
UARC.Md rozlišuje42/43. UAAT.Src0 v pilotním mode43 označuje direct rate:
RRef/PRef/ER/EP jsou0 a **nejsou attitude cílem**, Roll/Pitch/Yaw jsou měřená
orientace, RR/PR/YR jsou skutečné body rate targety. UAST.UN má vACRO0;
UAMT.UN a MN počítají skutečné vlastní rate a post-output průchody.

APN/AN0 jsou zdrojově deklarované hodnoty výhradní dispatch větve, nikoli
nezávislý globální profiler AP. Kontroluje se současně zdrojová výhradnost,
Seq, vlastní počty a skutečné motorové povely. Controller/filter/allocator
matematika i původní AP_Motors/HAL/DShot/arming jsou beze změny.

## Omezení statického GCS seznamu FLTMODE

Původní `ArduCopter/Parameters.cpp` uvádí ve statickém `FLTMODE1 @Values`
z UTB pouze historický mode40; ani42 ani43 tam nejsou. Runtime AP_Int8 zápis
`FLTMODE2=42`, `FLTMODE3=43` a skutečné RC přepnutí fungují a byly ověřeny.
Klient používající statický seznam nemusí nabídnout nové položky v grafickém
dropdownu. V SITL lze použít číselný zápis v úplném seznamu parametrů nebo
standardní SET_MODE43/42; dynamický AVAILABLE_MODES katalog má položky obou modes.
Globální hardware metadata nebyla rozšířena o nedostupné aktivní SITL režimy.
Konkrétní Mission Planner GUI a automatické načtení jeho seznamu zůstávají neověřené.
