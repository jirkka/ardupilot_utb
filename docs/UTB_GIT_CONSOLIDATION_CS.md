# UTB: audit Gitu a návrh konsolidačního checkpointu

FÁZE 2E, 16. 10. 2026. [Rozcestník](UTB_README_CS.md) ·
[Mapa souborů](UTB_FILE_MAP_CS.md) · [Současný stav](UTB_PROJECT_STATUS_CS.md).

**Připraveno ke schválení jednoho baseline commitu. Commit ani push nebyl
proveden. Skutečný Git index zůstává beze změn.** Tato fáze přidává čtyři dokumenty a odstraňuje jeden nadbytečný prázdný
řádek na konci historického attitude reportu; jeho obsah se nemění.
Žádná regulační implementace, firmware nebo parametr FC se nemění.

## 1. Audit větve a historie

| Položka | Zjištěný stav |
|---|---|
| Větev | `test/skystars-5inch` |
| HEAD | `687f825d228b12a64d36651072aed19356178217` |
| Stav před 2E | 13 tracked modified, 45 untracked souborů; index prázdný, bez konfliktů. |
| Návrh po 2E | 13 modified + 49 untracked = **62 souborů**; všechny původní soubory zachované. |
| origin | `git@github.com:jirkka/ardupilot_utb.git`; živé read-only ls-remote potvrzuje branch `feb5b68361de252af9f5764907ad80d1f92e03d7`. Lokální větev je o 8 commitů napřed. |
| origin/master | Lokální i živý ref `e204ca77a8012342b52af17beb78adaca598113c`. Není baseline tohoto UTB vývoje. |
| upstream | `https://github.com/ArduPilot/ardupilot.git`; žádný lokální upstream tracking ref. Živý master při auditu `8ab6afde107a91df1b5f60ade8f333cb8a61b190`. Bez fetch/rebase; vztah ahead/behind proti nedostupnému lokálnímu objektu se neodhaduje. |
| Další lokální větve | `ardupilot-4.7.1-utb` = `8dc66c8d18`; `master` = `e204ca77a8`. Beze změn. |
| Tagy | Žádné lokální tagy. Remote tagy nebyly vyčítány. |
| Submoduly | 21 inicializovaných položek recursive status včetně vnořených; SHA odpovídají gitlinkům, bez změn. Úplné SHA v checkpointu. |
| Rozdíl tracked proti HEAD před 2E | 13 souborů, +324/−5 řádků. Untracked navíc obsahují podstatnou implementaci i testy. |
| Projektový baseline | `36874030af9f8fd86c089ea6aed6fd5f03dff7f1`, těsně před prvním AP_UTB patchem; 12 následných commitů tvoří souvislou historii do HEAD. |

Od baseline je v pracovním stromu **121 přidaných UTB souborů**, včetně této
čtveřice dokumentů, a **22 upravených původních source/build souborů** plus
dva existující setup dokumenty. To není počet souborů nového commitu:
většina SHADOW/BENCH změn už je v historii. Počítání nepoužívá dnešní
upstream/master, který by přimíchal nesouvisející změny i dřívější redukci forku.

Relevantní zachované commity:

| Commit | Obsah |
|---|---|
| `761a38d704` | Skeleton AP_UTB, FÁZE 0. |
| `8dc66c8d18` | SHADOW a benchmark základ. |
| `6857c74e19` | Referenční Skystars konfigurace. |
| `feb5b68361` | Explicitní BF_X_REV podpora. |
| `40b47c8d30` | Post-sampling validation time. |
| `c97884464f` / `825724b250` | Packed diagnostika / výběr každého 400Hz cyklu. |
| `d325c44390` | Důkazy packed H743 měření. |
| `adb02b6ddb` | PID/mixer matematická validace. |
| `f9cc3bcc0f` / `b9131e8f35` / `687f825d22` | Syntetický DISARMED generátor, software a H743 validace. |

Novější 1H-D/1H-E/2A/2B/2C jsou dosud pracovní strom. Jejich skutečný původ
se zachovává phase manifesty a reporty; nebudou se předstírat dodatečně
vytvořenými historickými commity. Metadata sidecar
`README_ARDUPILOT_UTB_SETUP_BUILD_CS.md:Zone.Identifier` existoval již před
AP_UTB baseline. V 2E se nemaže ani nestaguje; nejde o source nebo build vstup.

## 2. Záloha a integrita před úpravami

Nový nezávislý checkpoint:
[tmp/utb-git-consolidation-20261016_192206](../tmp/utb-git-consolidation-20261016_192206/).

| Soubor checkpointu | Co zachycuje |
|---|---|
| [all-source-before.json](../tmp/utb-git-consolidation-20261016_192206/all-source-before.json) | Všech 7526 tracked cest v superprojectu a 45 untracked souborů, druh/velikost/SHA-256; gitlink adresáře zvlášť jako gitlink-or-directory. Submoduly zachycuje recursive status, nejsou vydávány za běžné source soubory. |
| [tracked-before.patch](../tmp/utb-git-consolidation-20261016_192206/tracked-before.patch) | Binary-safe tracked rozdíl proti HEAD. |
| [untracked-before.tar.gz](../tmp/utb-git-consolidation-20261016_192206/untracked-before.tar.gz) | Všech 45 tehdejších untracked souborů. |
| [all-dirty-source-before.tar.gz](../tmp/utb-git-consolidation-20261016_192206/all-dirty-source-before.tar.gz) | Všech 58 původních dirty/untracked source a dokumentů; jména relativní ke kořeni. |
| status-before / staged-before / HEAD-before / branch-before | Úplný Git stav včetně důkazu prázdného původního indexu. |
| refs-before / remotes-before / submodules-before / history-before / ignored-before | Lokální refs, remotes, submodule SHA, historie a úplný ignored seznam. |
| [historic-full-integrity.json](../tmp/utb-git-consolidation-20261016_192206/historic-full-integrity.json) | Nová kontrola všech 904 položek archivu 2C a všech 2508 položek archivu 2B, bez neshody. |
| [source-continuity.json](../tmp/utb-git-consolidation-20261016_192206/source-continuity.json) | 57/58 souborů finálního 2C manifestu je bitově shodných; u jednoho historického Markdown reportu se liší pouze nadbytečný koncový prázdný řádek. |

2C manifest má SHA-256
`c0f4db6344763834aad946875cc309cb7708a69481cccb26c59ddd73f43dc148`,
904 souborů / **2528886828 B**. 2B manifest má SHA-256
`3f62cd262a928d14736cdcea530a1be9db94f443e2f33206e86826c2a4f6ae03`,
2508 souborů / **5170900988 B**. Velikosti jsou součtem položek; manifest sám
a jeho samostatný checksum se do vlastního seznamu nezahrnují.

Manifest konce 2B a checkpoint před 2C zachycují stejných 50 source souborů.
Změny během 2C identifikuje jeho 17souborový phase manifest; finálních
58 souborů zůstává zachovaných. Historické logy a obsah reportů se v 2E
nepřepisují. Jediná formátovací odchylka je odstranění prázdného řádku
na EOF `docs/UTB_ATTITUDE_SITL_VALIDATION_CS.md` (původně řádek240).
Úplný kandidátní diff odhalil tento problém u untracked souboru; obyčejný
tracked `git diff --check` jej nezahrnoval. Změna je výslovně evidovaná
v source-continuity.json a původní verze zůstává v checkpointu i2C archivu. Tato kontrola prokazuje kontinuitu uložených souborů; nenahrazuje
nový audit každého staršího algoritmického rozhodnutí.

Ignored experimentální výstupy se nebalí znovu celé do source taru:
zůstávají na původních místech s ověřenými manifesty. Checkpoint je lokální
záloha v témže WSL úložišti, nikoli off-device backup. Commit sám nebude
obsahovat velké důkazní archivy; pro přenos počítače je nutné uchovat je zvlášť.

## 3. Rozdělení obsahu a .gitignore

| Kategorie | Co patří do Gitu | Co je lokální archiv |
|---|---|---|
| Původní AP | Původní source a nezbytné malé integrační změny. | Build/cache a osobní prostředí. |
| Sdílené vlastní UTB | PID, State, Reference, diagnostic MotorMixer, manager/config. „Produkční“ zde znamená firmware source, nikoli schválené aktivní H743 řízení. | Generated vectors/výsledky. |
| Copter a SITL integrace | Mode40/41, dispatch, parametry, log structures a build registrace. | SITL executable, EEPROM, BIN/tlog. |
| H743 SHADOW/BENCH | Producer/worker, probes, fixed export, platform adaptér, guardy. | APJ/ELF/HEX a skutečné session výstupy. |
| C++ a Python unit | Sources, test registrace a malé fixtures. | JUnit/cache/log stdout a generované CSV. |
| SITL integrační testy | Lokální owned-process runnery, guard/analysis nástroje. | Každý parametrizovaný session adresář a grafy. |
| Offline model/analýza | Reprodukční skripty, malý explicitní demo plant JSON. | Opakované simulace, PNG/SVG/PDF/CSV výstupy. |
| Konfigurace | Skystars referenční `.param`, malé overlays, compatibility JSON a dokumentace. | Skutečný parameter backup každé session. |
| Dokumentace | Současné rozcestníky i historické reporty s jasným rozsahem. | Raw evidence v `tmp/`, přenášená samostatně. |

`.gitignore` se **nemění**. Existující `/tmp/*`, `build`, `*.apj`, `*.elf`,
`*.bin`, `*.tlog`, `*.pyc`, `.venv` a ostatní AP pravidla pokrývají přítomné
generované výstupy. Nové plošné `*.json`, `*.md`, `*.svg` apod. by mohlo skrýt
potřebné fixtures nebo source dokumentaci. Pomocí `git check-ignore --no-index`
byly prověřeny projektové cpp/h/py/json/md; žádný podstatný soubor není skrytý.
Nové dokumenty se kontrolují také. Ignored soubory zůstávají zachované.

Nejsou navržené velké generated blobs, osobní konfigurace nebo přístupové
údaje. Kandidát neobsahuje privátní klíče ani tokenové soubory. Již verzovaný
historický obsah se v této fázi neodstraňuje.

## 4. Doporučený commit a přesný staging návrh

Doporučení: **jeden konsolidační baseline commit**. Mode41, sdílené hlavičky,
parametry, packet metadata, testy a runnery se měnily přes více fází současně.
Umělé dělení by vyžadovalo rekonstruovat a znovu ověřovat mezistavy, které
nejsou současným ověřeným funkčním základem. Historickou posloupnost již
zachovávají checkpointy a reporty. Samostatný commit dalších funkcí může
následovat až nad tímto jednoznačným baseline.

Navržená message:

```text
AP_UTB: checkpoint verified rate and attitude SITL platform

Preserve the offline diagnostic tools and the tested SITL rate/attitude
cascade, allocation feedback and bounded experiment infrastructure.
Keep H743 restricted to the existing shadow/benchmark implementation.
Add the project entrypoint, source map, status and consolidation audit.
```

Přesný obsah je **13 změněných + 49 nových souborů**, bez mazání:

<!-- STAGING_MANIFEST_BEGIN -->
| Stav | Přesná cesta |
|---|---|
|M|[ArduCopter/Attitude.cpp](../ArduCopter/Attitude.cpp)|
|M|[ArduCopter/Copter.h](../ArduCopter/Copter.h)|
|M|[ArduCopter/Log.cpp](../ArduCopter/Log.cpp)|
|M|[ArduCopter/defines.h](../ArduCopter/defines.h)|
|M|[ArduCopter/mode.cpp](../ArduCopter/mode.cpp)|
|M|[ArduCopter/mode.h](../ArduCopter/mode.h)|
|A|[ArduCopter/mode_utb_rate_sitl.cpp](../libraries/AP_UTB/Copter/mode_utb_rate_sitl.cpp)|
|M|[ArduCopter/motors.cpp](../ArduCopter/motors.cpp)|
|A|[ArduCopter/utb_sitl_log.h](../libraries/AP_UTB/Copter/utb_sitl_log.h)|
|A|[Tools/autotest/test_utb_active_sitl.py](../Tools/utb/tools/test_utb_active_sitl.py)|
|A|[Tools/autotest/unittest/test_utb_active_sitl_analysis.py](../Tools/utb/tests/test_utb_active_sitl_analysis.py)|
|A|[Tools/autotest/unittest/test_utb_attitude_sitl.py](../Tools/utb/tests/test_utb_attitude_sitl.py)|
|A|[Tools/autotest/unittest/test_utb_download_experiment.py](../Tools/utb/tests/test_utb_download_experiment.py)|
|A|[Tools/autotest/unittest/test_utb_offline.py](../Tools/utb/tests/test_utb_offline.py)|
|A|[Tools/autotest/unittest/test_utb_plant_identification.py](../Tools/utb/tests/test_utb_plant_identification.py)|
|A|[Tools/autotest/unittest/test_utb_rate_optimization.py](../Tools/utb/tests/test_utb_rate_optimization.py)|
|A|[Tools/autotest/utb_1he_demo_plant.json](../Tools/utb/tools/utb_1he_demo_plant.json)|
|A|[Tools/autotest/utb_1he_offline.py](../Tools/utb/tools/utb_1he_offline.py)|
|A|[Tools/autotest/utb_active_sitl_analysis.py](../Tools/utb/tools/utb_active_sitl_analysis.py)|
|A|[Tools/autotest/utb_attitude_sitl.py](../Tools/utb/tools/utb_attitude_sitl.py)|
|A|[Tools/autotest/utb_closed_loop.py](../Tools/utb/tools/utb_closed_loop.py)|
|A|[Tools/autotest/utb_closed_loop_plots.py](../Tools/utb/tools/utb_closed_loop_plots.py)|
|A|[Tools/autotest/utb_closed_loop_replay.py](../Tools/utb/tools/utb_closed_loop_replay.py)|
|A|[Tools/autotest/utb_download_audit.py](../Tools/utb/tools/utb_download_audit.py)|
|A|[Tools/autotest/utb_download_experiment.py](../Tools/utb/tools/utb_download_experiment.py)|
|A|[Tools/autotest/utb_plant_identification.py](../Tools/utb/tools/utb_plant_identification.py)|
|A|[Tools/autotest/utb_plant_plots.py](../Tools/utb/tools/utb_plant_plots.py)|
|A|[Tools/autotest/utb_plant_sensitivity.py](../Tools/utb/tools/utb_plant_sensitivity.py)|
|A|[Tools/autotest/utb_rate_optimization.py](../Tools/utb/tools/utb_rate_optimization.py)|
|M|[Tools/scripts/build_options.py](../Tools/scripts/build_options.py)|
|A|[docs/UTB_1HE_SUMMARY_CS.md](../docs/UTB_1HE_SUMMARY_CS.md)|
|A|[docs/UTB_ACTIVE_SITL_ARCHITECTURE_CS.md](../docs/UTB_ACTIVE_SITL_ARCHITECTURE_CS.md)|
|A|[docs/UTB_ACTIVE_SITL_VALIDATION_CS.md](../docs/UTB_ACTIVE_SITL_VALIDATION_CS.md)|
|A|[docs/UTB_ATTITUDE_CONTROLLER_ARCHITECTURE_CS.md](../docs/UTB_ATTITUDE_CONTROLLER_ARCHITECTURE_CS.md)|
|A|[docs/UTB_ATTITUDE_SITL_VALIDATION_CS.md](../docs/UTB_ATTITUDE_SITL_VALIDATION_CS.md)|
|A|[docs/UTB_CLOSED_LOOP_MODEL_CS.md](../docs/UTB_CLOSED_LOOP_MODEL_CS.md)|
|A|[docs/UTB_DOWNLOAD_H743_TEST_PLAN_CS.md](../docs/UTB_DOWNLOAD_H743_TEST_PLAN_CS.md)|
|A|[docs/UTB_FILE_MAP_CS.md](../docs/UTB_FILE_MAP_CS.md)|
|A|[docs/UTB_GIT_CONSOLIDATION_CS.md](../docs/UTB_GIT_CONSOLIDATION_CS.md)|
|A|[docs/UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md](../docs/UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md)|
|A|[docs/UTB_PHASE_2B_SUMMARY_CS.md](../docs/UTB_PHASE_2B_SUMMARY_CS.md)|
|A|[docs/UTB_PHASE_2C_SUMMARY_CS.md](../docs/UTB_PHASE_2C_SUMMARY_CS.md)|
|A|[docs/UTB_PLANT_IDENTIFICATION_CS.md](../docs/UTB_PLANT_IDENTIFICATION_CS.md)|
|A|[docs/UTB_PROJECT_STATUS_CS.md](../docs/UTB_PROJECT_STATUS_CS.md)|
|A|[docs/UTB_RATE_ANTIWINDUP_CS.md](../docs/UTB_RATE_ANTIWINDUP_CS.md)|
|A|[docs/UTB_RATE_H743_BENCH_CS.md](../docs/UTB_RATE_H743_BENCH_CS.md)|
|A|[docs/UTB_RATE_PID_OPTIMIZATION_CS.md](../docs/UTB_RATE_PID_OPTIMIZATION_CS.md)|
|A|[docs/UTB_README_CS.md](../docs/UTB_README_CS.md)|
|M|[libraries/AP_Motors/AP_MotorsMatrix.cpp](../libraries/AP_Motors/AP_MotorsMatrix.cpp)|
|M|[libraries/AP_UTB/AP_UTB.cpp](../libraries/AP_UTB/AP_UTB.cpp)|
|M|[libraries/AP_UTB/AP_UTB.h](../libraries/AP_UTB/AP_UTB.h)|
|M|[libraries/AP_UTB/AP_UTB_RateController.h](../libraries/AP_UTB/AP_UTB_RateController.h)|
|A|[libraries/AP_UTB/AP_UTB_SITLAllocation.cpp](../libraries/AP_UTB/AP_UTB_SITLAllocation.cpp)|
|A|[libraries/AP_UTB/AP_UTB_SITLAllocation.h](../libraries/AP_UTB/AP_UTB_SITLAllocation.h)|
|A|[libraries/AP_UTB/AP_UTB_SITLAttitude.cpp](../libraries/AP_UTB/AP_UTB_SITLAttitude.cpp)|
|A|[libraries/AP_UTB/AP_UTB_SITLAttitude.h](../libraries/AP_UTB/AP_UTB_SITLAttitude.h)|
|A|[libraries/AP_UTB/AP_UTB_SITLRate.cpp](../libraries/AP_UTB/AP_UTB_SITLRate.cpp)|
|A|[libraries/AP_UTB/AP_UTB_SITLRate.h](../libraries/AP_UTB/AP_UTB_SITLRate.h)|
|M|[libraries/AP_UTB/AP_UTB_config.h](../libraries/AP_UTB/AP_UTB_config.h)|
|A|[libraries/AP_UTB/tests/test_utb_ap_allocation.cpp](../libraries/AP_UTB/tests/test_utb_ap_allocation.cpp)|
|A|[libraries/AP_UTB/tests/test_utb_sitl_attitude.cpp](../libraries/AP_UTB/tests/test_utb_sitl_attitude.cpp)|
|A|[libraries/AP_UTB/tests/test_utb_sitl_rate.cpp](../libraries/AP_UTB/tests/test_utb_sitl_rate.cpp)|
<!-- STAGING_MANIFEST_END -->

<!-- BLOB_SUMMARY_BEGIN -->
Finální obsahy 62 kandidátních Git blobů mají dohromady **993913 B**: nových49 souborů **633404 B**, úplné nové obsahy13 změněných souborů **360509 B**. Jde o nekomprimované velikosti obsahu, nikoli o velikost patchu nebo výsledného Git packu. Existující předchozí obsahy změněných souborů zůstávají v historii.
<!-- BLOB_SUMMARY_END -->

Strojově čitelný [staging-manifest.json](../tmp/utb-git-consolidation-20261016_192206/staging-manifest.json)
obsahuje pro každý soubor velikost, SHA-256 a přesný Git blob ID.
[staging-paths.txt](../tmp/utb-git-consolidation-20261016_192206/staging-paths.txt)
a NUL varianta určují jediný schválený kandidátní pathspec.
Kontrola přes samostatný alternativní index nezměnila skutečný `.git/index`.
[candidate.patch](../tmp/utb-git-consolidation-20261016_192206/candidate.patch)
zahrnuje i všechny nové soubory. Vytvoření alternativního indexu může uložit
nové blob/tree objekty do Git object store, nevytváří commit ani nemění refs.
Po případném schválení je nutné znovu ověřit SHA souborů proti manifestu;
mezitím upravený soubor už není přesně tímto kandidátem.

## 5. Ověření v této fázi versus převzaté důkazy

| Kontrola | Skutečně provedeno v 2E | Převzato / omezení |
|---|---|---|
| C++ UTB unit | 47 core +9 rate +14 attitude +9 allocation = **79 PASS** na existujících test binaries; žádný rebuild. | C++ source totožný s ověřenou 2C sadou; report EOF nemění executable. |
| Python unit | **128/128 PASS**, včetně archivního fixture, bez SKIP. | Není live hardware experiment. |
| Základní SITL | Nový `basic`, DISARMED integration PASS. | Stejný hash finálního active2C executable. |
| RATE-only | Nové roll/pitch sessions, dvě úplné sady po3200, PASS. | Nenahrazuje celý gain sweep 2B. |
| ATTITUDE | Nová roll3°/P3/OWNER1/ATT1 session, 5600 sad při399,993 Hz, PASS. | Celých37 sessions a robust/comparison evidence jsou zachované 2C, nikoli nově spuštěné. |
| Active compile guards | Nově source/ELF symbol kontrola OFF variant, pozitivní active větev. | Samotné buildy a runtime absence testy jsou převzaté z hashově ověřeného 2C archivu. |
| H743 izolace | Nová kontrola tří APJ hashů a ELF: aktivní helpers/mode41 nepřítomné; BENCH symbol pouze ON. | Cross-buildy se pro dokumentační změnu neopakovaly. Hardware v2E neotevřený. |
| Logging disabled | Nová kontrola archivního ELF identity/active symbol. | Úspěšný2C build se třemi dokumentovanými extra-hwdef defines; není nově spuštěný runtime test. |
| FÁZE0 / BF_X_REV / guards | Zdrojová kontinuita a integrita původních PASS výsledků. | Sedm2C regresí není znovu prohlašováno za nové runs. |
| Formátování | flake8 všech40 projektových Python source/build souborů PASS; nové Markdown UTF-8/LF, tabulky/fences/odkazy zkontrolované. `git diff --check` včetně kandidátního indexu. | C++ astyle/changed-hunk výsledek převzatý z2C: žádný C++ řádek se v2E nezměnil. |
| Archivní integrita | 904+2508 souborů znovu ověřeno, bez mismatch. | Další historické archivy nebyly celé znovu hashovány; důležité odkazy a source continuity ověřené samostatně. |

Záznamy: [unit-results](../tmp/utb-git-consolidation-20261016_192206/unit-results.json),
[basic](../tmp/utb-git-consolidation-20261016_192206/sitl-basic/basic.json),
[RATE](../tmp/utb-git-consolidation-20261016_192206/sitl-rate/results.json),
[ATTITUDE](../tmp/utb-git-consolidation-20261016_192206/sitl-attitude/results.json),
[ELF/APJ audit](../tmp/utb-git-consolidation-20261016_192206/binary-isolation-rechecked.json).
Použitý active ELF SHA-256:
`60ec137f403f03b59bad96ed42e70c5aab27c34d7011508cbf10eb220b82a414`.

H743 BENCH_ON APJ zůstává1129203 B,
SHA-256 `8c925227885595c66b0acca5f50729a88643f1bb863d19a33d2d63a31b0142aa`.
Není to nový firmware ani upload. Defaultní AP arming/failsafe, mode40 zákaz,
AP_Motors output těla a HAL/DShot zůstávají v2E nedotčené.

## 6. Rozhodnutí a další postup

Dokumentační konsolidace je připravená, současný vývoj i untracked source jsou
bezpečně zachycené lokálním checkpointem. Nebyla zjištěná neočekávaná regrese.
Zůstávají známé meze loggeru/H743 a chybějící aktivní hardware ověření popsané
v [PROJECT_STATUS](UTB_PROJECT_STATUS_CS.md). „Připraveno na další vývoj“
znamená reprodukovatelný software základ s jasnou evidencí, nikoli schválení
aktivního hardwarového letu.

**Další zápis do historie vyžaduje závěrečné schválení tohoto konkrétního
62souborového návrhu. Push vyžaduje samostatné výslovné schválení.**
COM5 nebyl otevřen, firmware/parametry FC nebyly změněné a nic nebylo
doinstalované. Původní i nové lokální experimentální důkazy zůstávají zachované.
