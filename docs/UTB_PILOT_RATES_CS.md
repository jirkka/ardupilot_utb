# FÁZE 2I – pilotní Actual Rates v SITL

Výchozí `UTB_BF_TYPE=0` zachovává lineární ACRO a původní ANGLE obálku
FÁZE 2H. Nový profil je výslovně experimentální a aktivní pouze v SITL.
PID zisky, D filtr, integrátor, anti-windup matematika a model se neladí.

## Reference a nastavení

`UTB_BF_TYPE=1` vybírá statickou mapu Actual Rates. Pro každou osu R/P/Y:
`UTB_BF_<osa>_C` je citlivost středu v °/s, `_M` maximum v °/s, `_E` expo
v intervalu 0–1. Musí platit `0 <= C <= M <= 670`, `M >= 1`.
Pro normalizovanou páčku x a a=|x| je reference v °/s:

`C*x + (M-C)*a*((1-E)*x + E*x^5)`.

Do regulátoru vstupují rad/s. Při x=0 je reference nula, derivace ve středu
je C, při x=±1 je reference ±M. Samostatné osy nemají kruhové omezení.
Expo 0 stále obsahuje kvadratickou část Actual Rates; nejde obecně o lineární mapu.
Nepovolené hodnoty se odmítají; konfigurace se tiše neořezává.

Příklad profilu: RP C=160, M=670, E=0,3; Yaw C=160, M=500, E=0,3.
To není přesný převod konkrétního osobního BF profilu, jeho ostatní hodnoty
nejsou k dispozici. Soubor `Tools/utb/profiles/actual_670_sitl.parm` nemění gains.
RC kalibrace, deadzone a reverse zůstávají ArduPilot. BF RC smoothing,
feedforward a flight behavior se tím nepřebírají.

`UTB_BF_ANGLE` nastavuje kruhovou RP referenci ANGLE v ° (1–60).
`UTB_BF_ARATE` nastavuje samostatný normový RP body-rate cap v °/s (1–180).
Výchozí experimentální hodnoty jsou 45° a 90°/s. ANGLE yaw zůstává
`UTB_A_YMAX`, nejvýše 0,1 rad/s. ANGLE po vystředění požaduje vodorovnou orientaci;
ACRO po vystředění požaduje nulovou úhlovou rychlost.
Staré `UTB_C_*MAX`, `UTB_A_ANGLE`, `UTB_A_RMAX` zůstávají uložené pro TYPE=0.

Změny provádět mimo pilotní režimy. Celý profil je zmrazený při vstupu;
změna za běhu zachová existující latched CONFIG stop až do explicitního výstupu.
TYPE=1 vyžaduje existující `UTB_C_AW=1` / `UTB_A_AW=1` v příslušném režimu.
Není tím odstraněna motorová saturace ani zaručena dosažitelnost reference.

Nová skupina se registruje jako sousední `ParametersG2::var_info2` index22;
existující UTB indexy1–62 se nepřečíslují. Další vnoření do AP_UTB překračuje
povolenou hloubku AP_Param; prvotní neúspěšný SITL start je uchován v artefaktech.
Samotná úspěšná kompilace není kontrolou platnosti AP parameter table.

## Ochrany a omezení

TYPE=0 a mode41 zachovávají původní limity. Pouze explicitní pilotní TYPE=1
má normový target cap `sqrt(3)*670°/s` a gyro cap `sqrt(3)*800°/s`.
První hodnota pokrývá kombinovaný plný vstup tří os; druhá je experimentální
hlídací mez s rezervou nad referencí, nikoli prokázaný stabilní rozsah.
Pilotní attitude guard se rozšiřuje z35° na75° na RP složku Eulerových úhlů;
není to kruhový limit skutečného náklonu. Rozsah má pokrýt požadovanou referenci
60° a omezený přechod, nikoli převrácený let.

Kontroly AHRS/INS, gyro health, čerstvosti10ms, nového INS timestampu,
400Hz/FSTRATE, frame1/18, svislé rychlosti0,5m/s a výšky±2m zůstávají.
Plné roll/pitch rotace přes180° nejsou povolené ani označené jako ověřené.
Vstup do ANGLE mimo attitude obálku se odmítá; nejde o recovery z inverted ACRO.
Guard stop s nulovým tahem není bezpečným řešením pro skutečný let.
Aktivní režimy a nová banka se na H743 musí zcela compile-out odstranit.

## Primární zdroje a budoucí BF PID

Matematická mapa odpovídá
[Betaflight4.5.0 rc.c, applyActualRates](https://github.com/betaflight/betaflight/blob/4.5.0/src/main/fc/rc.c#L192-L201).
BF interně násobí center a max konfigurační hodnoty deseti a expo dělí100;
zde jsou parametry přímo fyzikální hodnoty a zlomek. Alternativní BF Rates
používá RC Rate a Super Rate s jiným průběhem; druhá mapa se neimplementuje.
Volba AP kalibrace a odděleného opt-in profilu je vlastní engineering decision.

[BF pid.c](https://github.com/betaflight/betaflight/blob/4.5.0/src/main/flight/pid.c#L944-L1068)
pracuje s gyro/rate chybou v °/s; UTB s rad/s. BF má navíc navázané I-term relax,
TPA, anti-gravity a dynamické filtrování. Převzetí vyžaduje samostatný návrh a testy.
[pid_init.c](https://github.com/betaflight/betaflight/blob/4.5.0/src/main/flight/pid_init.c#L275-L278)
škáluje uživatelské P/I/D konstantami z
[pid.h](https://github.com/betaflight/betaflight/blob/4.5.0/src/main/flight/pid.h#L44-L50):
0,032029 / 0,244381 / 0,000529. To nejsou přímo UTB gains.
[Mixer](https://github.com/betaflight/betaflight/blob/4.5.0/src/main/flight/mixer.c#L621-L637)
navíc omezuje PID sum a normalizuje ji `PID_MIXER_SCALING` před alokací.
Případný odvozený převod musí zahrnout180/π, normalizaci a přesný AP actuator
transport/compensation gain. Nestačí převod úhlových jednotek.

Dále je nutné srovnat skutečné dt a pořadí filtrace/derivace, Nyquist při400Hz,
typ a fázi filtrů, I limity, saturaci a alokační zpětnou vazbu. Stejný cutoff
neznamená stejný filtr. Číselné BF PID hodnoty se nepřebírají a nynější gains
zůstávají přesně podle2H. BF zdroje deklarují GPLv3-or-later, stejně jako
vlastní UTB soubory; případný port musí zachovat licence a autorské hlavičky.
Tato fáze přidává matematickou referenci, nikoli port BF PID.

## Měření a stav ověření

Artefakty: `tmp/utb-2i-20261018_204008/`. Baseline je commit
`103d27a85a479ab60203eb799672918875f5f40f`. Testovaný aktivní executable
`builds/sitl-active-v2/arducopter` má SHA-256
`c0dc9dd1d75f96699ecd89d5c8d77f628edbb11b420a6393b4b47179a789e7c3`.
Následné C++ úpravy jsou pouze komentářové/whitespace. Neúspěšné prototypy
jsou zachované samostatně a nejsou tímto testovaným executablem.

### Naměřeno v nezměněném generickém3kg bfxrev modelu

RP stupňování obsahuje obě znaménka obou os při stejném profilu160/670/0,3.
PWM kvantování způsobuje malé odchylky skutečné reference. Uvedeny jsou
maximální absolutní gyro hodnoty pouze během platné regulace:

| Požadavek | Skutečná reference | Roll peak | Pitch peak | Výsledek krátkého pulzu |
|---|---|---|---|---|
|90°/s|90,099°/s|92,13°/s|92,12°/s|bez guard stopu|
|180°/s|179,880°/s|183,85°/s|184,10°/s|bez guard stopu|
|360°/s|360,885°/s|368,45°/s|362,45°/s|bez guard stopu|
|670°/s|670,000°/s|659,29°/s|658,95°/s|attitude guard75°, nikoli trvale dosažených670|

Výsledky jsou krátké transienty, nikoli nová optimalizace nebo plná
kvalifikace stabilního rozsahu. Požadovaný pulz120ms měl při stávající pomalé
ATTITUDE telemetrii skutečně přibližně185–200ms u nižších rychlostí;
670 bylo oříznuto guardem přibližně142ms. Rozhoduje TimeUS v BIN, nikoli
počet sekund požadovaný host skriptem. Starší pokus s obecným SYSTEM_TIME
helperem vedl k delšímu pulzu a vertical stopu; evidence zůstává zachovaná.
Po stopu se UTB state snapshot neobnovuje, proto se zmrazené gyro nepočítá
jako trvající dosažená rychlost ani se z něj nevyhodnocuje settling.

Rate ladder obsahuje6343 kompletních tříosých sad, DSF.Dp=0. Rekonstrukce
alokace zaznamenala limit masky0/7/8 a blokování integrace; nejvyšší absolutní
I=0,0523, D=0,4647, raw=1,6734. To dokládá i saturaci při rychlých vstupech,
nikoli bezpečnou motorovou rezervu. Aktivní alokaci stále provádí AP_Motors.
[Mapa reference](../tmp/utb-2i-20261018_204008/plots/actual-rates.png) ·
[Skutečné RP transienty](../tmp/utb-2i-20261018_204008/plots/rate-ladder.png).

ANGLE ladder:20 scénářů (RP± při10/20/30/45/60°), pevný hover throttle1472.
Při referencích10/20/30° dosaženo přibližně8,7/17,4/26,2° během krátkého
kroku; po vystředění poslední validní náklon nejvýše0,084/0,180/0,249°.
Žádný stop u těchto malých scénářů. Při45° zásah vertical guardu kolem37°,
při60° kolem44°; tyto scénáře proto nejsou PASS pro plný náklon/self-level.
Celkem17822 kompletních sad, DSF.Dp=0.
[Průběhy s pevným plynem](../tmp/utb-2i-20261018_204008/plots/angle-fixed-throttle.png). To není důkaz neschopnosti attitude P
udržet45/60°: konstantní ruční tah při náklonu ztrácí svislou složku.
Skriptované doplňkové ruční throttle experimenty jsou samostatné:

- `angle-throttle-45-0-1`: statická kosinusová kompenzace z původního PWM1472,
  dosaženo44,94°, pak vertical stop6 kolem4,93s od vstupu, bez dokončeného self-level.
- `angle-throttle-native-45-0-1`:51 vzorků `ATTITUDE_TARGET.thrust` z ustáleného
  původního GUIDED hoveru dalo zmrazený základ0,363822 a shodné PWM1472.
  Dosaženo44,95°, poslední validní VZ přibližně−0,492m/s, pak stop6 kolem4,72s.
  Samotné pevné PWM tedy není dostatečné vysvětlení. Přesná příčina zbývající
  svislé odchylky nebyla izolována; jednoduché1/cos(roll)/cos(pitch) nestačilo
  pro tento delší manévr s nezměněným modelem. Další60° kompenzované pokusy
  se po této stop podmínce nespouštěly. Oba běhy jsou **FAIL pro zamýšlené
  úplné ANGLE držení a self-level**, nikoli PASS převodem číselné chyby na úspěch.

Nový helper pouze invertuje původní AP throttle mapu pro konkrétní kalibraci
1000/1500/2000. GCS_MAVLink_Copter.cpp `send_attitude_target()` vysílá
`attitude_control->get_throttle_in()`. Tah se převzal před vstupem, pak je základ
zmrazený; během UTB testu žádná výška ani svislá rychlost neřídí throttle.
To není UTB angle boost, výšková ani polohová smyčka. Vstupy, nepovedené
záznamy a latched stop zůstávají v evidenci. PID/model/guards se kvůli výsledku
neměnily. Vyšší ANGLE rozsah je tedy implementovaný matematicky, **letově
nedokvalifikovaný**, ani45° dlouhé držení zde není schválené.

Yaw500°/s má samostatné skutečné pulzy obou znamének: reference±500°/s,
peak gyro337,18/337,56°/s, skutečná délka1,590/1,600s, žádný fault/stop,
DSF.Dp=0. Rozbalený yaw v platném řídicím okně změnil úhel o+362,74/−366,70°.
Yaw tedy prošel více než jednou otáčkou; není to roll/pitch flip ani inverted
recovery. Požadovaných500°/s nebylo v těchto pulzech dosaženo. Yaw alokace byla
omezena (maska4, blokování integrace4, raw až2,632); žádné PID gains se neladily.
Nelze z krátkého pulzu stanovit univerzální konečnou rychlost daného modelu.
Podrobnosti jsou v `yaw-results.json` a jednotlivých `analysis.json`.

### Implementováno, ověřeno a dosud neověřeno

- Implementováno:12 AP parametrů, Actual Rates, nezávislé ANGLE limity,
  frozen profile a opt-in rozšířené normové guards; legacy TYPE=0 default.
- Unit ověřeno:133 C++ testů v9 executablech a156 Python testů.
  Exportní matematický test byl spuštěn také samostatným independent vector
  runnerem:30946 vzorků,29 scénářů,0 nesouladů (`math-resume/`); běžné
  spuštění bez vstupního souboru tento jeden test záměrně přeskakuje.
  Nové testy pokrývají znaménka/expo/střed/endpoints, neplatné hodnoty,
  rozšířenou ANGLE matematiku i návrat rate adaptéru do legacy limitů po resetu.
- Metadata ověřena:12 unikátních `UTB_BF_*` parametrů, délka jmen≤16,
  parameter/logger parser bez chyb; skutečný runtime AP_Param start i readback.
  Konkrétní Mission Planner GUI nebylo spuštěno; H743 tuto skupinu neobsahuje.
- SITL gate:normal/force arming mode40 odmítnut; DISARMED reference RP670
  a Yaw500 i všechny osy670±, kombinace, live edit→CONFIG latch, obnovení
  parametru samo latch neodstraní. Přímé43↔42 při částečně nenulových
  kombinovaných páčkách a nativní RC/GCS RTL ověřeny. Nejde o přechod z inverted.
- Build ověřeno:aktivníSITL ON/BENCH ON; SITL activeOFF a UTBOFF;
  H743 UTBOFF a UTBON s vynuceným activeON. `nm` potvrzuje odstranění
  aktivního adaptéru a nové třídy z obou H743 variant a vypnutýchSITL variant.
- Source audit:AP_Motors/HAL/arming/failsafe/EKF/scheduler/logger/model beze
  změny; manager a PID/mixer/reference/state beze změny. Všech17 pilotních
  gains odpovídá2H také ve skutečných BIN parametrech. Žádné nové blocking,
  heap alokace ani logger zápisy v hlavní cestě. Profil přidává pevné snapshot
  porovnání a tři statické rate mapy; hardware overhead se neměřil, naH743
  je aktivní profil compile-out.
- Hardware neověřeno:žádný upload, COM5 ani fyzický test. Trvalých RP670°/s,
  roll/pitch flips přes180°, inverted recovery, jiný fyzický plant a hardware
  aktivní řízení nejsou ověřené ani schválené. ALT HOLD není implementované.

### Regrese a rozhodnutí o rozsahu

Původní20 pilotních sad2H prošlo na nové binárce:DISARMED, R/P/Y kroky,
všechny přechody včetně kombinací a throttle, RC/GCS failsafe v obou režimech,
konfigurační a entry/ground limity, remap a nativní AP srovnávací větve.
Dále prošlo5 mode41/basic/guard/RATE/attitude scénářů a celá packed SHADOW
sada (health/suppressed logging, SYSID, primary gyro, reference map,
motor isolation). Evidence:`pilot-regression-v2-results.json`,
`regressions-resume/results.json`, `shadow-resume/results.json`.
Přerušený původní `regressions-final/` není započítaný jako dokončený běh.

Nový profil a legacy regrese jsou ověřené v uvedeném rozsahu. **Celá požadovaná
letová obálka2I dokončená není:**RP670 trvale, Yaw500 dosaženě, plné ANGLE45/60
s návratem i roll/pitch přes180° zůstávají otevřené. Experimentální implementaci
lze odděleně uložit; žádná z těchto neúspěšných kvalifikací se neoznačujePASS.
Další práce vyžaduje vysvětlení svislé dynamiky/throttle/modelu a samostatný
návrh rozšířené manévrové obálky, nikoli prosté vypnutí guardů.

### Reprodukce buildů a experimentů

Z kořene repository, bez sudo; compiler musí být ve stávajícím PATH.
Každý configure přepíše aktuální konfiguraci buildu. Artefakt je nutné
zkopírovat do samostatného adresáře před dalším configure.

```sh
./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
./waf copter -j6
# Aktivní testy pouze s touto variantou, gate SHA musí odpovídat binárce:
.venv/bin/python -B Tools/utb/tools/utb_pilot_rates_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-2I --name disarmed --stage gate
.venv/bin/python -B Tools/utb/tools/utb_pilot_rates_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-2I --name profile --stage profile-gate --maximum 670 --yaw-maximum 500
.venv/bin/python -B Tools/utb/tools/utb_pilot_rates_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-2I --name ladder --stage rate-ladder --maximum 670 --yaw-maximum 500
./waf configure --board sitl --enable-UTB --disable-UTB_RATE_SITL --enable-UTB_BENCH
./waf copter -j6
./waf configure --board sitl --disable-UTB --disable-UTB_RATE_SITL --disable-UTB_BENCH
./waf copter -j6
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
./waf copter -j6
./waf configure --board SkystarsH7HD-bdshot --disable-UTB --disable-UTB_RATE_SITL --disable-UTB_BENCH
./waf copter -j6
```

Výsledky4 compile-out variant jsou v `matrix-final-results.json`, aktivní
SITL v `build-v2-results.json`, jednotkové výsledky v `unit-resume/`, `math-resume/` a
`python-resume-final.log` (původní běhy jsou zachované). Kontrola astyle je čistá pro vlastní změněné C++ soubory;
existující formátovací odchylky native Parameters.cpp/h jsou shodné s baseline,
bez plošného přeformátování. Flake8 a `git diff --check` jsou čisté.
