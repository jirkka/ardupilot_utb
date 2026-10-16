# FÁZE 2B: skutečně ověřený stav

Větev `test/skystars-5inch`, HEAD
`687f825d228b12a64d36651072aed19356178217` zůstaly nezměněné.
Nevznikl commit/push, upload, hardware parameter write nebo nové fyzické
motorové oprávnění. Důkazy pouze v `tmp/utb-2b-20261014_164442`.
Nic nebylo doinstalované. Použité toolchain, venv a plotting runtime již existovaly.

## Implementováno a ověřeno

| Oblast | Stav | Konkrétní důkaz |
|---|---|---|
| Analýza yaw transientu | SOURCE/SITL VERIFIED | 8s plateau, I-only/D-only, capture bez allocation residualu |
| Finální samostatné R/P/Y gains | SITL VERIFIED | repeat-final 9/9 nezávislých sessions, 3200/3200 sad |
| AP/UTB authority a reference | SOURCE/SITL VERIFIED | APN/UN/MN, 9 triplets alignment, původní AP baseline |
| AP allocation feedback | SOURCE/UNIT/SITL VERIFIED | Bᵀ(T/G), inverse actuator consistency, 9 new C++ tests |
| Directional integration gate | UNIT/SITL VERIFIED v podporovaném kontraktu | 3x8000 weak-yaw stress, sign reversal, independent deltaI |
| Default SATURATION stop | UNIT/SITL VERIFIED | 1259 latched post-fault samples, UN=2883 frozen, APN=0 |
| Scripting/config/stale guards | SOURCE/UNIT/SITL VERIFIED podle testu | FEEDBACK fault9, aw-guards, guards-final |
| Gyro/dt/INS/primary invalidity | UNIT VERIFIED | původní9 helper tests, default/core47; nenahrazuje HW sensor injection |
| Nominální robustnost | SITL VERIFIED v testovaných variantách | 22 initial +12 targeted final; žádný fault/envelope divergence |
| FÁZE0 / BF_X_REV shadow | SITL VERIFIED | compiled/runtime OFF/ON, isolation, arming a geometry epochs |
| Aktivní motor path na H743 | SOURCE/BUILD VERIFIED: absent | 3 H743 images bitwise equal2A; ELF/parameter/log string absence |
| H743 skutečný experiment | H743 VERIFIED jen read-only IDLE | 40,938s, version/health/timing/stack, žádné LIST/DATA nebo writes |
| H743 nové gains / AW / aktivní stabilita | NOT EXECUTED | společná PID matematika nebyla změněná; nový adapter na H743 není |
| Download/EKF root cause | UNVERIFIED | IDLE PASS není download test; DataFlash free capacity UNKNOWN |
| Velké attitude/reference, obecné frames, motor loss/boost, delší delay | UNVERIFIED/unsupported | strict stop zůstává; nebyla implementována FÁZE2C |

Sdílené RateController.cpp, MotorMixer.cpp, Reference.cpp, State.cpp,
mode_utb_acro.cpp, arming, EKF, HAL/DShot nejsou tímto patchem změněné.
AP_Motors allocator/output rovnice nejsou změněné. Je přidaná výhradně
SITL dostupnost již existujících read-only getterů. Při UTB_ENABLE=0
nelze mode41 aktivovat a zůstává původní AP controller cesta.

## Finální SITL hodnoty a praktický závěr

| Osa | P | I | D | IMAX | D_HZ |
|---|---:|---:|---:|---:|---:|
| Roll | 0,19 | 0,16 | 0,0015 | 0,19 | 15 |
| Pitch | 0,19 | 0,16 | 0,001 | 0,19 | 20 |
| Yaw | 0,30 | 0,60 | 0 | 0,19 | 20 (D inactive) |

Mean IAE final vs old UTB: roll −16,23 %, pitch −27,18 %, yaw −62,50 %.
Final IAE: 0,021854 /0,018493 /0,146432 rad. AP baseline: roll IAE
je nižší o přibližně8,21 % relativně k AP, pitch/yaw UTB IAE nižší
7,67/44,30 %. AP má rychlejší R/P rise. Final positive rise:
69,14/69,14/515,63 ms; yaw 2% settling je kolem1s. Překmity, null/mixed
settling a rozptyl nejsou skryté: viz optimization report/final-aggregate.json.

Yaw původně neměl pevný 78% limit. Starý PID má v tomto modelu pomalé
integrační dorovnání v krátkém 2s plateau; IMAX/headroom nebyly aktivní.
I-only test pomohl, D-only yaw prakticky ne. Delší old plateau dosáhl
98,78 % a až6,886s settling. Zvolený yaw není kalibrace fyzického dronu.

Po předem uloženém Pareto objective 0,75*Jnominal+0,25*Jdelay byla
zvolena více tlumená roll sada. První trial balanced J zlepšil asi 6 %,
finální mean nominal + nový single delay J dal 0,032557 vs0,033292
první nominal sady (asi 2,2 %). Rozdíl není statisticky prokázané globální
optimum; delay cases nejsou tři independent repeats. Final extra 2,5 ms
roll overshoot8,37 %, pitch7,61 %: stále citlivost, ne libovolný delay margin.

Anti-windup je skutečně aktivní v podporované normalized command alokaci,
nikoli jen přerušení. I neroste směrem do saturace, při reverse error
unwind pokračuje; unsupported feedback fail-stop. Aktuální default S_AW=0
stále bezpečně zastavuje. H743 nemá tento active adapter vůbec.

Další bounded návrh nadřazeného roll/pitch **v SITL** je technicky možný
na tomto základu po samostatném zadání/schválení. Kvůli citlivosti na delay
má začít malými referencemi a zachovat guards. Není hotový vlastní attitude
controller, není HW tune/flight clearance a FÁZE2C nebyla zahájena.

## Testy a neúspěšné pokusy

- C++ core47 PASS, původní SITL helper9 PASS (profile7 boundary rozšířené),
  allocator/feedback9 PASS, independent math4 PASS: celkem69 tests.
  Replay30 946 vectors /29 scenarios, bez mismatch mimo forward float bound.
- Python124/124 PASS s UTB_AUDIT_EVIDENCE nastaveným na existující immutable
  1HD archive. Před doplněním fixture119 PASS/5 SKIP; Windows opakování
  také bez chyb. Žádný live COM experiment během unit tests.
- Nominal bounded sweep17/17 actual runs na archivovaném2A executable.
- 97 complete control/capture sessions v novém executable, dohromady339200
  complete sets. Independent max projection error1,010e−7, I recurrence
  error2,982e−8. Podrobnosti v capture-revalidation/summary.json.
- Phase0 ON/OFF, shadow-bf18, expanded guards-final, aw-guards a obě
  runtime absence variants PASS. Arming normal/force/RC rejection v mode40,
  původní manual modes dostupné, frozen fault/reentry a APN isolation.
- Param/logger metadata PASS. Log IDs jsou unique ve všech6 relevantních
  feature variants, common namespace32+, formats≤16 fields/labels≤64bytes.
- flake8, changed C++ astyle a git diff --check ověřené závěrečnými logy.

Pětinásobný speedup s8packets/cycle dal incomplete log; není validní
measurement. Použit speedup2 bez interpolace. Extrémní model slew0,1
(1500× nižší než default150) nedokončil původní AP takeoff před mode41;
není to UTB stability measurement. Varianta slew0,5 prošla, ale zvýšila
R/P overshoot. Rapid slew100 neprokazuje velký motor-lag margin.

Expected stop observer původně ukončil proces před log flush; další cleanup
narazil na správný AP disarm guard nebo již DISARMED heartbeat. Archivované
harness failures jsou rozepsané v antiwindup reportu; C++/arming kvůli nim
nebyl měněný. Finální standardní LAND cleanup prošel bez force.

H743 HEX helper napoprvé použil system Python bez intelhex; existující venv
v PATH problém vyřešila bez instalace. Logger-disabled SITL s default
custom controllerem měl upstream nechráněný AP::logger call. Guard build
používá extra-hwdef s Logging/CUSTOMCONTROL/SYSTEMID=0, kód těchto
komponent nebyl opravovaný. Metadata annotation order UAFB byl opravený
před final metadata PASS; actual binary packet layout se nezměnil.

## Buildy, identita a artefakty

Všechny3 H743 APJ i jejich decoded images jsou přesně stejné jako odpovídající
archivované2A artefakty. Active mode41/S_AW/UAFB/UALT symbols/strings chybí
na H743 i v authority/UTB OFF buildech. BENCH ON symboly byly také pozitivně
ověřené. Žádná změna FPGA/HAL, žádný upload.

| Varianta | Build | APJ/ELF bytes | SHA-256 |
|---|---|---:|---|
| h743-bench-on | PASS | 1129203 | `8c925227885595c66b0acca5f50729a88643f1bb863d19a33d2d63a31b0142aa` |
| h743-bench-off | PASS | 1123579 | `b371ed8a6fdbaa988fee725fcbfd8b27942262ad3fb14b6ae7a5dd45f3f35be9` |
| h743-utb-off | PASS | 1113283 | `b7a89f1923b20974cabfacb4377953d250a93146384d610e2ac10ff7df95255f` |
| sitl-authority-off | PASS | 5727352 | `f1c62df72104cdf3e4d1846b36f52b42e73b150ea0b5d59c463f08e4ccfef022` |
| sitl-utb-off | PASS | 5686112 | `503c80825be1d3cebbe246ad4cc0578aa28279f8e9d6483ba6f44bb98eaed372` |
| sitl-no-logging | PASS | 5423048 | `71496a9dd179468b8b028a30b896b5966849a5a419c55457440b1d385acd489e` |

Finální active SITL executable: `final-active-build/arducopter`, SHA-256
`9e2c47436834b026f8de9edd6586c8e03bfb34e216a8318d61fe91841adba2b2`.
Je bitově shodný s executable použitým v actual2B measurements; nové
unit test changes firmware nemění. Povolení aktivní2B na hardware nelze
získat runtime parametrem, ani forced hardware --enable-UTB_RATE_SITL.

Reprodukce z repo root (existující venv/ARM toolchain):

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
.venv/bin/python ./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf --targets bin/arducopter,tests/test_utb,tests/test_utb_sitl_rate,tests/test_utb_ap_allocation,tests/test_utb_math -j6
.venv/bin/python ./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
.venv/bin/python ./waf copter -j6
```

Přesný argv všech variant v build-matrix/*/commands*.json a final-active-build/
commands.json. Logger-disabled extra-hwdef je zachované v sitl-no-logging/
nolog-extra.hwdef. Runtime commands: `utb_rate_optimization.py --binary
<archived SITL executable> --output <NEW directory> --suite repeat-final`
nebo robust-final, speedup2. Runner je serial-only, vlastní své procesy a
nikdy neotevírá hardware transport. Gain plans/model JSON/params jsou v
každém suite; původní SELECTED a finální FINAL jsou explicitně odlišené.

## Přesný manifest FÁZE2B

Níže delta proti checkpointu **před tímto zadáním**, nikoli všechny změny
z předchozí2A/1HE. Existující tracked clean soubory používají HEAD baseline;
untracked a preexisting changes používají původní tar/sha manifest.

| Soubor | Zásah |
|---|---|
| `ArduCopter/defines.h` | SITL log IDs26/27; ostatní IDs nezměněné. |
| `ArduCopter/mode.h` | Fixed capture/raw/sequence členy a frozen AW politika výhradně v SITL mode41. |
| `ArduCopter/mode_utb_rate_sitl.cpp` | Read-only post-output AP capture, validovaná feedback integrace, long-yaw profile, scripting guard. Stále jeden původní motor output. |
| `ArduCopter/utb_sitl_log.h` | UAFB/UALT fixed packets, metadata a dokumentované časové/normalizační vztahy. |
| `Tools/autotest/test_utb_active_sitl.py` | Capture packet čtení, optional finite duration a expected-fault observer, race-aware standard LAND cleanup, další config/absence guards. |
| `Tools/autotest/unittest/test_utb_rate_optimization.py` | 4 evidence rejection/scoring/projection/attribution tests. |
| `Tools/autotest/utb_active_sitl_analysis.py` | Explicitní expected sample count se zachovaným default3200 pro staré testy. |
| `Tools/autotest/utb_rate_optimization.py` | Bounded actual SITL sweep/repeats/causal/robust/Pareto/stress/guard runner a independent capture/I parity. |
| `docs/UTB_PHASE_2B_SUMMARY_CS.md` | Souhrn skutečného ověření, manifest, reprodukce, artefakty a další meze. |
| `docs/UTB_RATE_ANTIWINDUP_CS.md` | Normalized allocation kontrakt, directional gate, fallback, unit/actual saturation evidence a unsupported domains. |
| `docs/UTB_RATE_H743_BENCH_CS.md` | Jediný skutečný read-only IDLE experiment, identita, timing/stack/log health a explicitní NOT EXECUTED PID test. |
| `docs/UTB_RATE_PID_OPTIMIZATION_CS.md` | Zdrojová/experimentální analýza, nominální vs finální gains, metriky, grafy, robustness a omezení. |
| `libraries/AP_Motors/AP_MotorsMatrix.cpp` | Pouze dostupnost dvou nezměněných read-only getterů na HAL_BOARD_SITL. Allocator/output body beze změny. |
| `libraries/AP_UTB/AP_UTB.cpp` | S_AW index28/default0 pod SITL guardem; S_PROFILE metadata pro profile7. |
| `libraries/AP_UTB/AP_UTB.h` | SITL-only AP_Param field/getter S_AW. |
| `libraries/AP_UTB/AP_UTB_RateController.h` | Pouze zpřesnění komentáře existujícího feedback rozhraní; žádná změna rovnice/API. |
| `libraries/AP_UTB/AP_UTB_SITLAllocation.cpp` | Fixed float rekonstrukce skutečného normalized AP allocated request a strict validation. |
| `libraries/AP_UTB/AP_UTB_SITLAllocation.h` | Pure capture/result kontrakt bez actuator writer/logger závislosti. |
| `libraries/AP_UTB/AP_UTB_SITLRate.cpp` | Volitelný verified feedback, sequence/age checks, FEEDBACK latch, finite long profile; default SAT stop. |
| `libraries/AP_UTB/AP_UTB_SITLRate.h` | SITL-only policy/context/sequence rozhraní, Fault9 a finite duration helper. |
| `libraries/AP_UTB/tests/test_utb_ap_allocation.cpp` | 9 actual AP allocator / PID feedback tests včetně clippingu; žádný HAL output. |
| `libraries/AP_UTB/tests/test_utb_sitl_rate.cpp` | Rozšíření původního finite profile testu o všechny profile7 boundary/body counts. |

Machine-readable phase2b-manifest.json, phase2b-review.patch zahrnují i nové
untracked soubory. Full git status/HEAD a snapshot pracovního stromu jsou
v git-status-final.txt, HEAD-final.txt a working-tree-final.tar.gz.
Všechny předchozí source changes a historické reporty mimo uvedený delta
jsou zachované; původní39-file checkpoint zůstává reprodukovatelný.
Untracked soubory jsou záměrné source/report soubory z 1HD/1HE/2A a této2B;
generated evidence jsou v ignored tmp/build adresářích.

## Fyzická FC na konci povolené session

Poslední skutečné pozorování: DISARMED mode0, healthy EKF flags167,
boot77 stabilní, firmware prefix f9cc3bcc. Parametry nebyly zapisované,
SHADOW=0, B_SYNTH=0, LOG_DISARMED=0, frame1/18, FSTRATE=0, logger backend4.
Port uzavřen. Nenásledovalo další čtení po skončení40s approval scope;
nejde o kontinuální monitorování pozdějšího stavu. DataFlash capacity a
PM.NLon UNKNOWN. Nové hardware PID/active stability měření NOT EXECUTED.
