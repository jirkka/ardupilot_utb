# FÁZE 2C: vlastní attitude kaskáda v ArduPilot SITL

**PASS v popsaném malém SITL rozsahu**, s uvedenými omezeními přesnosti.
Skutečná kaskáda je UTB reduced-rotation attitude P → původní UTB Rate PID
→ původní AP_Motors → skutečný ArduPilot SITL plant → AHRS/INS.
Nejde o wrapper nad AP attitude controllerem ani offline simulátor.

Větev `test/skystars-5inch`, základní HEAD
`687f825d228b12a64d36651072aed19356178217`. Změny zůstaly v pracovním stromu
spolu s předchozími etapami. Nebyl vytvořen commit, push ani release.
COM5 nebyl otevřen; FC parametry ani firmware nebyly zapisované.
Nic nebylo doinstalované. Použité venv, ARM toolchain a plotting runtime
již existovaly. Nové důkazy: `tmp/utb-2c-20261016_152820/`.

[Architektura a rovnice](UTB_ATTITUDE_CONTROLLER_ARCHITECTURE_CS.md),
[měření, grafy a omezení](UTB_ATTITUDE_SITL_VALIDATION_CS.md).

## Implementováno a ověřeno

| Oblast | Stav | Důkaz a rozsah |
|---|---|---|
|Vlastní reduced rotation P|SOURCE/UNIT/SITL PASS|Body→NED quaternion, independent double matrix audit, žádné AP attitude volání při Src1|
|Signed náklony a návrat|SITL PASS|±1°, ±3°, později±5°, combined, sign changes, sinus|
|Nenulový yaw|UNIT/SITL PASS v rozsahu testů|Heading30° a body yaw rate0,05rad/s; nejde o heading hold|
|Tuning/opakovatelnost|SITL PASS|6 finite sweep sessions K1,5/2/3;3× roll +3× pitch pro K3|
|AP comparison|SITL VERIFIED s rozdíly|6 sessions: AP outer se stejným UTB inner vs plný AP řetězec|
|Robustnost|SITL PASS bounded completion|14 scénářů; dvě plateau bez doloženého2% settling jsou popsané v reportu|
|Rate clamp|UNIT/SITL PASS|19 sign-change cycles a1185 low-cap cycles, žádný allocation limit|
|Stávající RATE-only|SITL PASS|2×3200 complete sad; default S_ATT0 zachovává původní cestu|
|Arming/FSTRATE/frame/freeze/latch|UNIT/SITL PASS v provedených testech|Původní guards,7 nových param mutations, re-entry; mode40 normal/force/RC zákazy|
|Výhradní autorita|SOURCE/SITL PASS|Outer UN/AN, inner UN/APN, MN=Seq; nezměněný jediný output hook|
|H743 active path|SOURCE/BUILD PASS: absent|Forced active request, ELF/string absence,3 APJ bitwise equal2B|
|H743 runtime/let/CPU|NOT EXECUTED|Žádný hardware port/upload/aktivní fyzický experiment|
|Altitude/position/heading hold|NOT IMPLEMENTED|Manual throttle, read-only výšková stop obálka|

37 attitude sessions =207200 complete sad, k tomu2 RATE-only lety =6400 sad.
79 C++ a128 Python testů PASS, včetně14 nových C++ skupin a4 Python testů.
Nezměněná velká offline PID vector validace2B nebyla opakovaná.
FÁZE0 ON/OFF, shadow BF_X_REV, basic/guards a runtime absence active-OFF/
UTB-OFF prošly. Legacy FÁZE0 wrapper zachoval force požadavky jen jako
negativní test mode40; pozitivní STABILIZE použil běžné checks,
ARMING_CHECK1 a45s warmup. Aktivní lety nikdy force-arm nepoužily.
Metadata, flake8, changed C++ astyle/changed-hunk style a git diff --check PASS.
Historické vývojové chyby a dvě nedoložené2% metriky jsou ve validačním reportu.

## Regulační výsledek a další krok

SITL-only outer gains: **roll P3 /pitch P3 s⁻¹**, RP norm cap0,3rad/s;
inner gains přesně finální2B. Ze3 sessions na osu: rise+0,724s roll /
0,706s pitch; max překmit všech repeat přechodů0,706/0,777%,2% settling
přibližně1,1–1,4s. Max koncová chyba návratu0,0187° /0,0340°.
Angle IAE0,070796 /0,069757rad*s.

AP při vlastním P4,5/shapingu má asi0,32–0,34s a menší IAE. Vlastní
jednoduchý P je pomalejší. Sinus0,2Hz: amplitude ratio0,926 a phase delay
0,318s. Přidaný2,5ms delay: dokončená sequence, max angle overshoot0,570%;
nejde o obecný latency margin. Yaw-rate a throttle perturbace mají jednotlivá
plateau bez trvalého2% settling, max overshoot2,241/2,043%.
Functional tail acceptance0,25° nebyla po měření měněná. Širší přesnost
nebo robustnost není tímto PASS tvrzená.

Stabilizace nemá zjištěný blocker pro navazující **návrh vlastní výškové
smyčky v SITL**. Zachovat tento reprodukovatelný základ a explicitně
zahrnout yaw/throttle omezení. Tato etapa výšku neimplementuje a nepovoluje
aktivní hardware. Pro skutečný5″ dron nejsou identifikované model/gains,
uzavřená stabilita ani realtime rezerva. Aktivní cesta na H743 není přítomná.

## Přesný manifest FÁZE2C

Delta vůči checkpointu před tímto zadáním, nikoli vůči samotnému HEAD.
Dosavadních50 dirty/untracked source je zachovaných v předimplementačním tar.
Nová etapa mění9 existujících a přidává8 souborů:

| Soubor | Typ | Zásah |
|---|---|---|
|ArduCopter/Copter.h|změněn|Guardovaný include čistého attitude helperu.|
|ArduCopter/defines.h|změněn|SITL IDs28 UAAT a29 UAST, unique ve6 relevantních kombinacích.|
|ArduCopter/mode.h|změněn|Fixed context/result, frozen config, counters a vertical diagnostics v mode41.|
|ArduCopter/mode_utb_rate_sitl.cpp|změněn|Explicitní RATE/UTB-attitude/AP selection, own outer→inner target, guards, post-output log.|
|ArduCopter/utb_sitl_log.h|změněn|Dva fixed pakety, metadata/jednotky, upřesnění Ref/Tar pro attitude.|
|libraries/AP_UTB/AP_UTB.h|změněn|SITL-only param fields/getters.|
|libraries/AP_UTB/AP_UTB.cpp|změněn|Indexy29–35/default S_ATT0 a metadata profilů; evaluate() beze změny.|
|libraries/AP_UTB/AP_UTB_SITLRate.h|změněn|Pouze Fault10 ATTITUDE /11 VERTICAL, stará čísla/matematika zachované.|
|Tools/autotest/test_utb_active_sitl.py|změněn|Čtení UAAT/UAST a absence kontrola7 nových parametrů.|
|libraries/AP_UTB/AP_UTB_SITLAttitude.h|nový|Čistý fixed Config/Result/lifecycle kontrakt, SITL-only.|
|libraries/AP_UTB/AP_UTB_SITLAttitude.cpp|nový|Reduced error, P/limiter, validity a finite reference.|
|libraries/AP_UTB/tests/test_utb_sitl_attitude.cpp|nový|Analytické/double-reference, reset/health/NaN/clock/limit a outer/inner fault testy.|
|Tools/autotest/utb_attitude_sitl.py|nový|Actual SITL DISARMED/flight/sweep/repeats/comparison/robust runner a striktní analýza.|
|Tools/autotest/unittest/test_utb_attitude_sitl.py|nový|Metrics reference a odmítnutí neúplné evidence.|
|docs/UTB_ATTITUDE_CONTROLLER_ARCHITECTURE_CS.md|nový|Rovnice, frames, scheduling, lifecycle, compile isolation.|
|docs/UTB_ATTITUDE_SITL_VALIDATION_CS.md|nový|Actual data, grafy, repeat/AP/robust metrics a omezení.|
|docs/UTB_PHASE_2C_SUMMARY_CS.md|nový|Souhrn, manifest, build/test provenance.|

RateController.cpp/.h, MotorMixer.cpp, Reference.cpp, State.cpp, mode40,
arming/failsafe/EKF/HAL/DShot, AP_Motors a původní motor/rate dispatch se
vůči předchozímu stavu nezměnily. Nové altitude getters čtou cached AHRS state.
Vlastní matematika nemá heap, logger, mutex nebo wait. Diagnostika používá
stávající SITL post-output mechanismus; navíc93B/cyklus. Není to H743 měření.

## Buildy a identita

| Varianta | Stav | Artefakt bytes / SHA-256 |
|---|---|---|
|sitl-active|PASS|5741904 / `60ec137f403f03b59bad96ed42e70c5aab27c34d7011508cbf10eb220b82a414`|
|sitl-authority-off|PASS|5727352 / `f1c62df72104cdf3e4d1846b36f52b42e73b150ea0b5d59c463f08e4ccfef022`|
|sitl-utb-off|PASS|5686112 / `503c80825be1d3cebbe246ad4cc0578aa28279f8e9d6483ba6f44bb98eaed372`|
|h743-bench-on|PASS|1129203 / `8c925227885595c66b0acca5f50729a88643f1bb863d19a33d2d63a31b0142aa`|
|h743-bench-off|PASS|1123579 / `b371ed8a6fdbaa988fee725fcbfd8b27942262ad3fb14b6ae7a5dd45f3f35be9`|
|h743-utb-off|PASS|1113283 / `b7a89f1923b20974cabfacb4377953d250a93146384d610e2ac10ff7df95255f`|
|sitl-no-logging|BUILD PASS|5431624 / `79faf1bbed43c796a5921652fcb8b1cb9b910b24bc7b48858a6d0c22572668a8`|

H743 APJ jsou všechny bitově shodné s2B. Source guard, ELF symbols,
parameter/log string absence a pozitivní BENCH_ON symbol jsou ověřené.
Runtime param absence byla testovaná v SITL OFF variantách; H743 runtime
nebyl čten. Finální active ELF je shodný s programem všech nových měření,
SHA i po finálním source/metadata buildu ověřené v binary-provenance.json.

Z repo root:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
.venv/bin/python ./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf --targets bin/arducopter,tests/test_utb_sitl_attitude,tests/test_utb,tests/test_utb_sitl_rate,tests/test_utb_ap_allocation -j6
.venv/bin/python -B Tools/autotest/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-disarmed --suite disarmed
# Teprve po unit/DISARMED PASS:
.venv/bin/python -B Tools/autotest/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-preliminary --suite preliminary
.venv/bin/python -B Tools/autotest/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-sweep --suite sweep
.venv/bin/python -B Tools/autotest/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-repeat --suite repeat --gain 3
.venv/bin/python -B Tools/autotest/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-compare --suite compare --gain 3
.venv/bin/python -B Tools/autotest/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-robust --suite robust --gain 3
.venv/bin/python -B Tools/autotest/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-rate --suite rate
.venv/bin/python ./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
.venv/bin/python ./waf copter -j6
```

SITL sessions spouštět sériově, výstupní adresáře musí být nové.
Přesné argv ostatních variant v build-matrix/*/commands.json a final-hardware/.
Logging OFF používá stejnou2B extra-hwdef výjimku HAL_LOGGING_ENABLED0 /
AP_CUSTOMCONTROL_ENABLED0 /MODE_SYSTEMID_ENABLED0; nesouvisející upstream
custom-controller/logger call se neopravoval. Logging OFF runtime flight
nebyl opakovaný, testuje se compile guard.

Checkpoint/provenance: before-source-manifest.json, before-working-source.tar.gz,
before.patch, historical-artifacts.json, phase2c-manifest.json,
phase2c-review.patch, all-working-source-final.json, working-tree-final.tar.gz,
git-status-final.txt, HEAD-final.txt, artifact-manifest.json a jeho SHA.
Datové soubory v ignored tmp nejsou součástí source commitu. Untracked source
z předchozích etap i všech8 nových2C souborů jsou záměrné.
