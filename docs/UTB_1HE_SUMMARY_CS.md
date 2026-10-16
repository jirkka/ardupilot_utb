# FÁZE 1H-E: offline diagnostika a příprava identifikace soustavy

Datum 13. 10. 2026. Větev `test/skystars-5inch`.
HEAD před/po `687f825d228b12a64d36651072aed19356178217`.
Poslední známý firmware FC `f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f` nebyl
v této fázi nahráván ani readbackem nově ověřován.

**Výsledek:** nové pomocné nástroje a měřicí protokoly jsou offline ověřené.
Přesná příčina zhoršení EKF při downloadu a fyzikální parametry současného
5″ dronu zůstávají **UNVERIFIED**. Nic nebylo připojeno k COM5, motory nebyly
spuštěny, DataFlash nebyla mazána, závislosti nebyly instalovány.
Commit, push, nový firmware build a fyzický experiment neproběhly.

## 1. Nově provedeno versus převzato

Nově implementováno:

- Bounded host download engine s identity/health/read-only guardy, TX/RX
  časováním, raw wire capture, timeout/byte/request bounds a bezpečným stopem.
- Jediný pending range přes původní TransferPlan, archivní prefix verification,
  samostatné approval vázané na konkrétní plan, explicitní absence změřeného
  flash operation duration. Výchozí režim je offline; validní live approval
  nebylo vytvořeno ani použito.
- Replay skutečné archivní health telemetrie přes stejné guardy bez TX;
  finální replay odlišuje archive elapsed od skutečného monotónního času.
- Reprodukční driver pro sedm mock scénářů, tři archive replays, osm planů,
  false/expired approval template a syntetické bifilární měření.
- Rozšíření původního rotačního modelu o tensor J, CG/geometrii, F/Q křivky,
  per-motor lag a voltage dependence; nezměněný default baseline.
- SI/provenance import, CAD parallel-axis součet, bifilární reduction s
  fixture/CG/damping correction, covariance a period reduction z CSV.
- 18 sensitivity variant, sedm local FD columns, SVD a strukturální
  identifiability kontrola; sedm kalibračních demo scénářů a devět SVG grafů.
- 55 nových unit testů a tři požadované reporty.

Převzato bez nového hardware tvrzení:

- Source audit synchronního loggeru a předchozí EKF časová osa z
  [UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md](UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md).
- Původní rotační simulátor, PID/reference/replay, plotter a 34 testů;
  výchozí matematika a parametry v
  [UTB_CLOSED_LOOP_MODEL_CS.md](UTB_CLOSED_LOOP_MODEL_CS.md).
- 8000/8000 valid 400Hz SHADOW cyklů a motorová izolace z
  [UTB_SYNTHETIC_H743_VALIDATION_CS.md](UTB_SYNTHETIC_H743_VALIDATION_CS.md).
- Minulé instability sweep časy a předchozí H743 benchmarkové hodnoty.
  Nejsou přeznačeny na fyzikální kalibraci dnešního dronu ani bezpečné limity.

## 2. Testy a důkazy

Nový output root: `tmp/utb-1he-offline-20261013_151020/`.
Před změnami uložen HEAD, git status, SHA/size manifest všech sedmi původních
untracked souborů a celý `untracked-before.tar.gz`; nejde jen o git diff.
`archive-before.json` obsahuje 92 původních H743 session artefaktů.

| Kontrola | Výsledek / rozsah | Důkaz v novém rootu |
|---|---|---|
| Původní unit regrese | **PASS 34/34, bez skip**; archive tests s kopií již ověřených 1H-D evidence | test_utb_offline.py.log, regression-evidence.json |
| Nový host nástroj | **PASS 28/28** | test_utb_download_experiment.py.log |
| Nový plant/import/uncertainty | **PASS 27/27** | test_utb_plant_identification.py.log |
| Původní default simulace | **PASS 24/24 CSV bajtově shodné**, PID input file shodný | baseline-regression/, verification.json |
| Původní C++ PID/mixer runner | **PASS 4/4** při execution archivovaného test binary; žádný nový build | cpp-replay/cpp-test.log |
| C++ parity | **PASS 75 226 samples, 0 mismatches**, původní forward float32 tolerance a přesné flags | cpp-replay/comparison.json |
| Demo import | **PASS** 7 comparable scenarios; všechny physical values illustrative | demo-calibration/calibration.json a CSV |
| Sensitivity | **PASS** 18 variant + 14 lokálních FD běhů; nejde o measured intervals | sensitivity/sensitivity.json, local-sensitivity.csv |
| SVD | **SOFTWARE VERIFIED**, numerický rank 5/7 při relative cutoff 10⁻³ | sensitivity/identifiability-svd.json |
| Bifilár / MC | **PASS** 20 000 synthetic samples, seed 1729, MC/std ratio 1,001805 | final-preparation/measurement-demo/DEMO_inertia_reduction.json |
| Reprodukční driver | **PASS** offline, 7 mock, 3 replay, 8 planů; archives unchanged | final-preparation/offline-results.json |
| SVG | **PASS** 9 vytvořených grafů, XML struktura a disclaimer ověřeny | figures/; nejde o hardware evidence |
| flake8 | **PASS** všech 8 dotčených Python souborů | flake8.log |
| Whitespace / diff | **PASS** git diff --check a kontrola i untracked obsahu | verification.json, changes.patch |
| Archivace / isolation | **PASS** původní 92 H743 + 131 1H-D output artefaktů hashově zachovány; production tree unchanged | verification.json |
| Nový H743/SITL build nebo hardware test | **NEPROVEDENO**, jen host/offline změny | žádný nový APJ/BIN měření nevznikl |

Celkem **89 Python testů PASS**, navíc 4 stávající C++ testy a parity všech
75 226 simulated vectors. První běh původních testů bez evidence env vynechal
pět archive testů; finální běh s `UTB_AUDIT_EVIDENCE` prošel všech 34 bez skip.
Testy raw telemetry byly provedeny jen proti existujícím tlog souborům,
nikoli proti sériovému portu.

Finální offline preparation obsahuje čtyři úspěšné mock scénáře (idle, list,
one, paced) a tři **očekávaně zastavené** (timeout, EKF unhealthy, new loop max).
Tyto očekávané stop výsledky nejsou regresní FAIL.
Full-download replay: 50 241 zpráv, 0 TX, dva konkrétní EKF unhealthy/bad
STATUSTEXT guard observations. Short-download: 5045 zpráv, 0 TX, žádný guard
health stop v tomto konkrétním souboru; chybějící pozdější pozorování není důkaz
zdraví po celé době výpadku. Final-coldboot-readback: 1841 zpráv, 0 TX,
žádný guard health stop. Podrobnosti v [download plánu](UTB_DOWNLOAD_H743_TEST_PLAN_CS.md).

## 3. Nalezené a odstraněné problémy pomocných nástrojů

- Ověření skutečného pymavlink API: generátor nemá předpokládané
  `get_send_buffer()`. Capture používá writer proxy a skutečný encoded packet.
- `mavserial.write()` může zachytit chybu a vrátit −1. Proxy takový výsledek
  odmítne; `tx_sent` nesmí být zaznamenán jako úspěch. Nový regresní test PASS.
- Typo při lokální úpravě CAD součtu zachytil unit test; opraveno před finální
  sadou testů. Žádný chybný výpočet nebyl vydán za fyzické měření.
- Při importu delay by nominálních přesných 2,5 ms dělených kratším float32 DT
  mohlo dát dva cykly. Quantization nyní používá nominální 1/400 s a reportuje
  float32 model interval zvlášť; regresní test přesného a fractional delay PASS.
- Dvojí zadání covariance pair v opačném pořadí by dvojitě započetlo variance;
  nově odmítnuto, včetně ne-PSD covariance. Regresní test PASS.
- Archive receive wall timestamps nejsou původní host monotonic clock;
  finální replay je používá jen jako označený `archive_elapsed_s`.

Žádná nová root cause závada produkčního ArduPilotu nebyla tímto postupem
prokázána nebo opravena. Source cesty a staré časové osy zůstávají důkazem
rizika synchronního downloadu; jeho přesný EKF mechanismus je HYPOTHESIS.

## 4. Přesný manifest změn této fáze

**Změněn vůči checkpointu 1H-D — 1 soubor:**

1. `Tools/autotest/utb_closed_loop.py` — optional tensor/geometrie/křivky/lag/
   voltage; default results zachovány. Soubor byl před začátkem stále untracked.

**Přidáno — 11 souborů:**

1. `Tools/autotest/utb_download_experiment.py` — bounded guarded host engine,
   live approval gate, TX/RX capture a archivní health replay.
2. `Tools/autotest/utb_plant_identification.py` — SI import, bifilár, covariance,
   pendulum CSV reduction, CAD mass properties a kalibrační scénáře.
3. `Tools/autotest/utb_plant_sensitivity.py` — sensitivity, torque bounds,
   structural identifiability a local FD/SVD.
4. `Tools/autotest/utb_plant_plots.py` — využití původního SVG plotteru.
5. `Tools/autotest/utb_1he_offline.py` — reprodukovatelné plans/mock/archive/demo,
   výhradně offline, se zachováním archivních hashů.
6. `Tools/autotest/utb_1he_demo_plant.json` — explicitně illustrative SI dataset.
7. `Tools/autotest/unittest/test_utb_download_experiment.py` — 28 host testů.
8. `Tools/autotest/unittest/test_utb_plant_identification.py` — 27 plant testů.
9. `docs/UTB_DOWNLOAD_H743_TEST_PLAN_CS.md` — hypotézy, observability, bounded
   experimenty a podmínky samostatného fyzického schválení.
10. `docs/UTB_PLANT_IDENTIFICATION_CS.md` — měřicí metody, uncertainty,
    fyzikální model, nové sensitivity výsledky a kalibrační postup.
11. `docs/UTB_1HE_SUMMARY_CS.md` — tento manifest a závěrečné vyhodnocení.

**Šest původních untracked 1H-D souborů beze změny:**
`test_utb_offline.py`, `utb_closed_loop_plots.py`, `utb_closed_loop_replay.py`,
`utb_download_audit.py`, `docs/UTB_CLOSED_LOOP_MODEL_CS.md`,
`docs/UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md`; první čtyři pod Tools/autotest, test pod
Tools/autotest/unittest. Jejich přesné cesty/hash jsou v sources-before.json.
Celkem konečný git status obsahuje **18 záměrných untracked souborů**,
žádnou změnu tracked souboru. Z tohoto důvodu samotný git diff neukazuje nové
nástroje; `changes.patch` a `source-manifest.json` zachycují celou delta fáze.
Dočasné výsledky jsou pouze pod novým session rootem a jsou mimo git manifest.

## 5. Safety/isolation audit

**PASS:** tracked production C++ a všechny submodules jsou beze změny.
AP_UTB controller, mixer, reference, arming/failsafe, AHRS/EKF, AP_Motors,
motor setters, HAL a DShot nejsou editovány. Ve FC fast path není nové I/O,
blocking operace, alokace nebo změna priority. Původní UTB_ACRO arming zákaz a
SHADOW motorová izolace zůstávají source-identické.

Nové host nástroje mají v live transportu jen heartbeat, read requests a
LOG_REQUEST_LIST/DATA/END; jediný COMMAND_LONG je read-only REQUEST_MESSAGE
512 s explicitním whitelistem message IDs 148/193/1. Mutující příkazy nejsou
implementovány a unit guardy je odmítají. Toto je kontrola nových source cest,
ne tvrzení, že synchronní download původního fw je hard real-time bezpečný.
Disk journaling je na hostu a v budoucí session může ovlivnit pacing; jeho
overhead není předem změřen na COM5. H743 instrumentační overhead se touto
etapou nemění, protože firmware ani jeho konfigurace nebyly měněny.

## 6. Deset závěrečných odpovědí

1. **Co chybí k přesné příčině EKF?** Nová TX/RX časová osa při malém reprodukčním
   požadavku, fresh EKF variances/flags/status, UB29/boot continuity. Pro přesnou
   funkci/zámek pravděpodobně FC read/metadatová duration a interní EKF reason;
   dnešní firmware je přímo netelemetruje. Host round trip je nenahrazuje.
2. **Nejmenší rozlišovací experiment?** Stejný idle polling, samostatný LIST,
   jediný DATA do 8190 B na archivním short logu. Další pacing a long-log až po
   vyhodnocení a dalším schválení; žádné velké stahování pro první reprodukci.
3. **Nástroje a ověření?** Guarded host engine/replay, offline reproducer,
   SI/bifilár/CAD import, sensitivity/SVD a plot wrapper; 89 Python testů,
   4 existing C++ tests, 75 226 parity samples, tři skutečné archive replays.
4. **Nejdůležitější fyzikální parametry?** J tensor/CG/geometrie pro absolute
   scale a osy, F/Q gain, motor/ESC/prop lag a celkové delay; voltage/saturation.
   D zvlášť ovlivňuje yaw. Pořadí sensitivity reálného dronu zatím není změřeno.
5. **Jak měřit?** Kalibrovaná váha/reakční podpěry pro m/CG, měřidla/CAD pro r,
   lehký referencovaný bifilár pro J po osách se samostatným J_f, h, d, periodou
   a uncertainty; teprve odděleně schválený laboratorní stand pro F/Q/lag/delay.
6. **Bez motorů?** Configuration inventory, m/CG/r, J a products přes mechanický
   rig, CAD, klidový gyro noise; statické napětí a prop hmotnost. Bez napájení
   pohonu nelze z DISARMED shadow logu zjistit dynamiku motorů.
7. **Co zůstane neznámé bez standu?** Skutečné F/Q versus q,V, max authority,
   deadzone/saturation, lag/delay celého ESC/motor/prop a zátěžový sag/vibrace.
   Výrobce/literatura může dát prior, nikoli kalibraci konkrétního stroje.
8. **Co ukázaly uncertainty scénáře?** Při τ+25 % roll overshoot 3,68→8,40 %;
   jeden sensor cycle 3,68→7,86 %; moment−20 % yaw settle 465→590 ms.
   Společné J,D,F scale a sensor/actuator čistý delay jsou confounded.
   Všechny hodnoty pocházejí z demonstrace, ne ze současného 5″ měření.
9. **Je import připraven na reálná data?** Ano pro podporovaný rigid-body model
   se SI/provenance validation a zachovanou baseline. Nikoli pro automatické
   tvrzení hardware validity; chybí skutečný dataset, trim/poles general modelu,
   ověřený q mapping a nezávislá fyzická data mimo fitting.
10. **Nejlepší další krok práce?** Bez motorů nejdříve doložená konfigurace,
    m/CG/geometrie a reference + bifilární J s uncertainty. Nezávisle předložit
    malý idle/LIST experiment k samostatnému schválení. Žádné aktivní UTB nebo
    FÁZE 2; pohonový stand je samostatná další laboratorní etapa.

## 7. PASS / FAIL / UNVERIFIED / NEXT ACTION

| Status | Konkrétní závěr |
|---|---|
| **PASS / SOFTWARE VERIFIED** | Pomocné nástroje, 89 Python tests, 4 C++ tests/parity, baseline preservation, source/archive integrity, isolation a lint |
| **FAIL** | Žádná otevřená softwarová regresní chyba v testovaném rozsahu; očekávané stop scénáře fungují |
| **HARDWARE VERIFIED — převzato** | Minulé 8000 valid shadow cycles, matematické logy a dobové download/health observations; žádné nové hardware tvrzení |
| **HYPOTHESIS** | Metadata/flash/mutex/timing/transport příčinný řetězec k EKF unhealthy |
| **UNVERIFIED** | Nový COM5 experiment, přesná root cause EKF, skutečné J/CG/F/Q/lag/delay/D, hardware plant calibration a aktivní closed-loop řízení |
| **NEXT ACTION** | Pasivní configuration/CG/J měření; samostatně schválit pouze konkrétní idle a LIST plány, pak vyhodnotit další single DATA |

Nyní se práce zastavuje před fyzickým experimentem. Ke schválení prvního
rozsahu lze předložit `final-preparation/plans/idle.json` a `list.json`:
COM5 read-only, bez vrtulí, DISARMED, frozen params jen čtené, max 100 s na
samostatnou session, žádný DATA request, žádné erase/flash/ARM/motor test,
stop při health/identity/transport změně, uložené tlog/timeline a následná
omezená read-only recovery. Platné approval vyžaduje nové fyzické potvrzení
a přesný plan digest. Tento dokument není schválením spuštění těchto plánů.
