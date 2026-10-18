# FÁZE 2G: pilotní ACRO a přímé přepínání s ANGLE

## Git a rozsah

Před implementací vznikl schválený lokální FÁZE 2F commit
`671ccad97050115e87b9d8451313dd7403fc1e72`:
`AP_UTB: add pilot ANGLE mode and verified SITL coverage`.
Před commitem byly ověřeny SHA/blob všech 26 souborů připraveného 2F manifestu.
Rodič: `9824542e73bbfb193b21fd4388f74e14ba24439c`.
Větev `test/skystars-5inch` je 1 commit před origin. HEAD zůstává výše uvedený.
FÁZE 2G je **necommitnutá**: 17 upravených + 9 nových souborů, index prázdný.
Žádný push, rebase, mazání historie, hardware spojení nebo upload.

Před změnami uložen tracked Git archive, všechny neignorované untracked soubory,
SHA-256 manifest a Git stav do `tmp/utb-2g-20261017_165925`. Všech 4 393 položek historického
manifestu 2F zůstalo shodných. Mimo tento přesný source manifest se původní
tracked soubory nezměnily; 2 651 chráněných zdrojů včetně AP_Motors/HAL/EKF,
arming/failsafe, původních PID/attitude/mode40/41 zůstalo beze změny.

Implementováno: samostatný ACRO43, ANGLE42, společný původní vlastní Rate PID
a AP manual throttle, přímé RC/GCS přepnutí, frozen AP param limity a FS1/1/0.
Aktivní hardware řízení, vyšší ACRO rychlosti a ALT HOLD nejsou implementované.
Přesný ověřený rozsah, původní neúspěšné pokusy a opravy popisuje
[validační report](UTB_ACRO_MODE_SITL_VALIDATION_CS.md).

## Přesný manifest a účel změn

| Stav | Soubor | Účel |
|---|---|---|
| upravený | [ArduCopter/Attitude.cpp](../ArduCopter/Attitude.cpp) | Výhradní fast rate dispatch mode43. |
| upravený | [ArduCopter/Copter.h](../ArduCopter/Copter.h) | Mode43 instance/friend; rate thread a custom-controller exclusion. |
| upravený | [ArduCopter/GCS_MAVLink_Copter.cpp](../ArduCopter/GCS_MAVLink_Copter.cpp) | Mode43 v AVAILABLE_MODES. |
| upravený | [ArduCopter/mode.cpp](../ArduCopter/mode.cpp) | Lookup43; GCS enable cache mask42/43. |
| upravený | [ArduCopter/mode.h](../ArduCopter/mode.h) | Guarded enum43/header. |
| upravený | [ArduCopter/motors.cpp](../ArduCopter/motors.cpp) | Capture43 za jediným původním motor outputem. |
| nový | [Tools/utb/tests/test_utb_pilot_sitl.py](../Tools/utb/tests/test_utb_pilot_sitl.py) | 7 nových Python testů. |
| upravený | [Tools/utb/tools/utb_angle_sitl.py](../Tools/utb/tools/utb_angle_sitl.py) | Rychlé mode acknowledgement, společný parser a mode-aware analýza. |
| nový | [Tools/utb/tools/utb_pilot_sitl.py](../Tools/utb/tools/utb_pilot_sitl.py) | Skutečné pilotní SITL scénáře a SHA-bound gate. |
| nový | [docs/UTB_ACRO_MODE_ARCHITECTURE_CS.md](../docs/UTB_ACRO_MODE_ARCHITECTURE_CS.md) | Skutečná architektura a safety kontrakt. |
| nový | [docs/UTB_ACRO_MODE_SITL_VALIDATION_CS.md](../docs/UTB_ACRO_MODE_SITL_VALIDATION_CS.md) | Metriky, grafy a omezení důkazů. |
| upravený | [docs/UTB_FILE_MAP_CS.md](../docs/UTB_FILE_MAP_CS.md) | Nové vlastní soubory v existujících složkách. |
| nový | [docs/UTB_FLIGHT_MODES_CS.md](../docs/UTB_FLIGHT_MODES_CS.md) | Přehled modes40–43 a pouze plánovaný ALT HOLD. |
| nový | [docs/UTB_PHASE_2G_SUMMARY_CS.md](../docs/UTB_PHASE_2G_SUMMARY_CS.md) | Tento manifest a reprodukce. |
| upravený | [docs/UTB_PROJECT_STATUS_CS.md](../docs/UTB_PROJECT_STATUS_CS.md) | Ověřený rozsah versus hardware. |
| upravený | [docs/UTB_README_CS.md](../docs/UTB_README_CS.md) | Rozcestník 2G. |
| upravený | [docs/UTB_SOURCE_GUIDE_CS.md](../docs/UTB_SOURCE_GUIDE_CS.md) | Průchod přímou ACRO cestou a parametry. |
| upravený | [libraries/AP_UTB/AP_UTB.cpp](../libraries/AP_UTB/AP_UTB.cpp) | SITL parametry index40–45, zachované původní indexy. |
| upravený | [libraries/AP_UTB/AP_UTB.h](../libraries/AP_UTB/AP_UTB.h) | Guarded členy a read-only gettery. |
| nový | [libraries/AP_UTB/AP_UTB_AcroRC.h](../libraries/AP_UTB/AP_UTB_AcroRC.h) | Lineární RC/body-rate helper, caps, FS a entry context predikáty. |
| nový | [libraries/AP_UTB/Copter/mode_utb_acro_sitl.h](../libraries/AP_UTB/Copter/mode_utb_acro_sitl.h) | Samostatný mode43 nad sdíleným lifecycle. |
| upravený | [libraries/AP_UTB/Copter/mode_utb_angle_sitl.cpp](../libraries/AP_UTB/Copter/mode_utb_angle_sitl.cpp) | Direct ACRO, frozen limity, FS policy, reset a DISARMED LAND re-entry. |
| upravený | [libraries/AP_UTB/Copter/mode_utb_angle_sitl.h](../libraries/AP_UTB/Copter/mode_utb_angle_sitl.h) | Sdílený kontext a virtuální mode number. |
| upravený | [libraries/AP_UTB/Copter/utb_angle_log.h](../libraries/AP_UTB/Copter/utb_angle_log.h) | Upřesnění společného pilotního komentáře. |
| upravený | [libraries/AP_UTB/Copter/utb_sitl_log.h](../libraries/AP_UTB/Copter/utb_sitl_log.h) | UAAT direct-rate Src=0 metadata, stejná ID/formát. |
| nový | [libraries/AP_UTB/tests/test_utb_acro_rc.cpp](../libraries/AP_UTB/tests/test_utb_acro_rc.cpp) | 12 nových C++ testů. |

`source-manifest-final.json` obsahuje velikost, SHA-256 a Git blob každého souboru.
BIN/tlog/CSV, grafy, buildy a reprodukční skripty jsou záměrné ignorované artefakty
v `tmp/utb-2g-20261017_165925`, nikoli v indexu. C++ a Python zůstávají v dosavadních oddělených složkách.

## Buildy, testy a izolace

| Varianta | Výsledek | Velikosti uložených artefaktů |
|---|---|---|
| final-h743-bench-on | PASS | 2346832 B, 1129203 B |
| final-h743-bench-off | PASS | 2340692 B, 1123575 B |
| final-h743-utb-off | PASS | 2335188 B, 1113287 B |
| final-sitl-active-off | PASS | 5727352 B |
| final-sitl-utb-off | PASS | 5686112 B |
| final-sitl-bench-off | PASS | 5760424 B |
| final-sitl-active | PASS | 5777056 B |

Plné příkazy/config/logy: `builds/<varianta>/`. H743 ON varianty záměrně mají
`--enable-UTB_RATE_SITL`; HAL_BOARD guard přesto odstranil aktivní modes41/42/43,
nové C_* a A_RMAX parametry a aktivní vlastní adaptéry. Mode40 arming zákaz
včetně force-arm je zachovaný. Jediný původní AP_Motors output zůstal beze změny.
APN/AN=0 v logu je zdrojová deklarace dispatch větve, nikoli nezávislý profiler.

124 C++ testů PASS včetně CSV math testu, 142 Python PASS.
Matematika: 30 946 vzorků / 29 scénářů / 0 chyb. Parametrická/logger metadata PASS,
39 původních registrovaných položek má stejné indexy. Flake8/diff --check PASS.
Astyle nových/změněných částí PASS; nesouvisející původní one-line metoda v
AP_UTB.h zůstala beze změny. Kompletní standalone math bez CSV jeden test
přeskočí; následný skutečný CSV validační běh provedl i tento test.

Hlavních 20 skutečných pilotních SITL scénářů PASS na ELF5834d2…; jediná pozdější
funkční změna je GCS cache mask. Regulace zdrojově identická, následné doplnění
komentářů tokenově identické. Finální ELF3ff450… má nové DISARMED a kombinované
přechodové ověření. Přesné hashe jsou ve validačním reportu, nejsou zaměňovány.
Následující tabulka uvádí dokončené opakované regrese na finálním ELF:

| Kontrola | Výsledek |
|---|---|
| final-disarmed | PASS |
| final-transitions-combined | PASS |
| legacy-regressions | PASS |
| angle-disarmed | PASS |
| angle-roll5 | PASS |
| angle-pitch5 | PASS |
| aw-guards | PASS |
| saturation | PASS |
| shadow-final | PASS |
| manual-final | PASS |

SITL active OFF a UTB OFF: skutečné runtime zamítnutí modes41/42/43, absence
nových parametrů a dostupnost původních modes0/1/2 PASS. Starší otestované
OFF ELF jsou bitově shodné s finálními OFF buildy (`absence-binary-equivalence.json`).

Manuální parser byl skutečně ověřen DISARMED: mode43/status/42/0 a zápis rate-limit.
Jeho automatické flight helpery ověřují pilotní scénáře, nikoli zvláštní GUI/joystick session.

GCS katalog indexy1..28 a AVAILABLE_MODES_MONITOR PASS včetně enable změn.
Původní stream na hromadný index0 vrátil jen první položku; konkrétní Mission
Planner GUI nebylo ověřeno. Statické FLTMODE @Values obsahují z UTB jen40;
pro42/43 funguje číselný FLTMODE zápis a SET_MODE, neprokazuje to GUI dropdown.
Nové rate parametry používají standardní AP read/write.

Full SITL `--disable-Logging` build **FAIL** na původním
`libraries/AC_CustomControl/AC_CustomControl.cpp:190` (`AP::logger()` při
HAL_LOGGING_ENABLED=0). Tento soubor se neměnil. Vlastní změněné UTB translation
units se stejnými log-off flags samostatně PASS. Malé `--disable-logging` tato
lokální větev ignoruje a není důkazem vypnutí loggeru; oba pokusy jsou uchované.

SITL execution časy jsou host měření, nikoli H743 WCET. Žádné nové DataFlash ID,
heap alokace v UTB fast path ani aktivní hardware callback. Doinstalováno pouze
**lxml6.1.3 do lokálního tmp metadata-runtime**, pro metadata a oficiální MAVLink
klienta; tímto instalačním příkazem se neměnil systém ani projektové .venv.

## Artefakty

Finální SITL ELF `builds/final-sitl-active/arducopter`, 5 777 056 B, SHA-256:
`3ff4501f80f6736c5e5b7b44684431fe31132540654cacc8beee6866d7cb8ebc`.

H743 APJ jsou pouze build/isolation důkaz, **nebyly nahrány**:

| Varianta | APJ B | SHA-256 |
|---|---:|---|
| BENCH ON | 1 129 203 | fbc50d653d2a64c5a092d13764eedcf3491d450862fa5bb2a907433188278ac7 |
| BENCH OFF | 1 123 575 | 910097f1071abeeefb3c50624876899602fd5c59a9cd24b8b5a5602e75d36e34 |
| UTB OFF | 1 113 287 | 2752797e7c067c9ff63c9818264b768ee8e4b709adb8ca63313e3b3b7071e8d7 |

Cesty: `tmp/utb-2g-20261017_165925/builds/final-h743-{bench-on,bench-off,utb-off}/arducopter.apj`.
Všechny varianty a ELF hashe obsahuje `final-build-results.json`.

## Reprodukce a společný manuální runner

Z kořene repo ve WSL, nikdy sudo:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
./waf copter -j6
.venv/bin/python -B Tools/utb/tools/utb_pilot_sitl.py \
  --binary build/sitl/bin/arducopter --output tmp/utb-2g-new-test --suite disarmed
.venv/bin/python -B Tools/utb/tools/utb_pilot_sitl.py \
  --binary build/sitl/bin/arducopter --output tmp/utb-2g-new-test --suite roll
```

Aktivní automatický test vyžaduje DISARMED PASS se shodným SHA. Další suite
sdílejí output root, každá má nový podadresář. Možnosti vypíše `--help`.
Spouštět pouze jeden vlastní SITL současně. Přesný cross-build BENCH ON:

```sh
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
./waf copter -j6
```

BENCH OFF mění pouze na `--disable-UTB_BENCH`. UTB OFF:
`--disable-UTB --disable-UTB_RATE_SITL --disable-UTB_BENCH`.
SITL active OFF ponechá UTB/BENCH ON a změní na `--disable-UTB_RATE_SITL`.
Varianty sdílejí waf output, proto jsou hotové artefakty v pojmenovaných složkách.
Žádný upload příkaz.

```sh
.venv/bin/python -B Tools/utb/tools/utb_pilot_sitl.py \
  --binary build/sitl/bin/arducopter --output tmp/utb-manual-2g-new --suite manual
```

1. Po `Owned SITL ready`: DISARMED `mode 43`, `status`, `mode 42`, `mode 0`.
2. Mimo42/43 uprav rychlosti: `rate-limit UTB_C_RMAX 0.10`. Návrat výchozího
   rozsahu: `rate-limit UTB_C_RMAX 0.15`. Stejné názvy parametrů používá GCS.
   Hard caps R/P0,15 a Y0,05 rad/s; A_RMAX má samostatnou mez0,05..0,3 rad/s.
3. `takeoff`: standardní GUIDED příprava, potom ANGLE42.
4. `mode 43`, krátce `sticks 0.3 0 0`, potom `sticks 0 0 0`: nulová rotace,
   zachovaný náklon. RP pulzy kolem1s, ne trvalý plný stick.
5. `mode 42`: vyrovnání při stejném throttle. `mode 43`: návrat do ACRO.
6. `throttle 1475`, návrat `throttle 1472`: malá ověřená změna v tomto modelu.
   1472 není altitude controller ani univerzální hover PWM.
7. `land`: původní AP LAND a standardní disarm. `quit`: uzavření vlastního procesu.

Runner nepřijímá hardware endpoint. Druhá GCS: `tcp:127.0.0.1:5762`.
Před předáním RC druhému klientu `external`; klient pak musí udržovat skutečný
RC/GCS heartbeat. Jeho GUI/joystick není validován samotnými testy runneru.
Tilt/vz/height obálka je nouzový stop s rizikem pádu modelu, nikoli recovery.

## Odpovědi na závěrečné otázky

1. **ACRO i ANGLE létají v SITL:** ano, v doložené krátké bfxrev obálce.
2. **Páčky:** ACRO lineární body rates; ANGLE náklon→vlastní attitude P→rate;
   yaw rate a manual throttle společné. ACRO bez expo/trainer/shaping.
3. **Bez self-level:** ano, nulová rate držela přibližně±10,3°; ANGLE vyrovnává.
4. **Maxima targetů:** RP±0,15 a Y±0,05 rad/s. Skutečné gyro má překmit,
   pitch až0,186405 rad/s. Vyšší parametry hard guard odmítne.
5. **Nenulové přechody:** ano,22 hran; R+0,3/P−0,3/Y+0,5 i kombinace/throttle.
6. **Plynulost:** bezprostřední PWM krok max3µs, okolí±100ms max31µs/cyklus.
   I/D se primují. Není dokonale spojitý přenos integrátoru.
7. **Throttle:** jedna původní AP mapa, collective na hraně stejný do numerického
   šumu; žádný altitude hold/angle boost.
8. **Odezvy:** RP10–90% přibližně15–17ms, překmit17–24%; yaw482–545ms.
   RP settling není vždy doložen; přesné metriky a srovnání jsou ve validaci.
9. **Failsafe:** FS1/1/0 a původní synchronní RTL pro oba modes; FS16 odmítnut.
   To neřeší všechny možné state/INS/allocator fault/recovery scénáře.
10. **Regrese/izolace:** původní matematika a arming/motorová cesta zachované;
    provedené runtime regrese jsou v tabulce. H743 aktivní compile-out PASS.
    Původní full log-off build má výše uvedený oddělený FAIL.
11. **Před ALT HOLD:** samostatná specifikace vertikální smyčky a throttle handoffu,
    review bezpečného fault recovery, delší kombinované/saturační manévry,
    tuning RP překmitu a další opakování. Hardware autorita vyžaduje samostatné schválení.

ALT HOLD ani autonomní XY nebyly zahájeny. Další commit nebo funkční fáze vyžaduje
samostatný souhlas. [Naměřená validace a grafy](UTB_ACRO_MODE_SITL_VALIDATION_CS.md).
