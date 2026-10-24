# FÁZE 3D-3 — nativní Rate/PID SHADOW v RAM

Datum: 21. 10. 2026. Větev `test/skystars-5inch`, výchozí i konečný HEAD
`070c189cdfa8357307ee1aebe8f3677cd849d520`. Změny nejsou commitnuté.

## Stav a rozsah

Implementováno: pasivní porovnání existujících `UTB_PID` a `UTB_Rate` s právě
vykonanou nativní AP rate smyčkou. Jednokroková i kontinuální historie, konečné
měřicí okno, statistiky v RAM a neměnný export po ukončení.

Unit/SITL a sestavení: výsledky níže. **Hardware 3D-3 neověřen**: žádné spojení
s COM5, upload, fyzické armování ani motorový test. Žádný commit ani push.
Attitude SHADOW, logger oprava, nové módy a tuning nejsou součástí této fáze.

Artefakty: `tmp/utb-3d3-20261021_191141/`. Před změnami vznikl úplný
`checkpoint.tar.gz` a `checkpoint-manifest.json`, včetně původních neignorovaných
untracked souborů. Původní 3D-2/logger dokumentace a rozpracované zdroje zůstaly
zachované. Rozdíl této fáze vůči checkpointu je oddělen v `phase-3d3.patch`;
`phase-manifest.json` obsahuje SHA-256 jednotlivých souborů. Patch zahrnuje i
nové soubory, nikoli jen výstup běžného `git diff`.

## Datový tok a integrační místa

1. `Copter::run_rate_controller_main()` připraví pořadové číslo, režim a čas
   rate-entry. Podporovaný kontext je nativní multicopter, 400Hz hlavní smyčka,
   vypnuté FSTRATE a rate thread, který ještě nikdy nebyl spuštěn. Módy 40–43
   nejsou zdrojem nativního srovnání.
2. `AC_AttitudeControl_Multi::rate_controller_run_dt()` zachytí hodnotové kopie
   skutečně použitých vstupů po úpravách gain boost a sysid targetu: gyro,
   target R/P/Y, dt, PD/I scaling, sysid actuator přídavky, yaw FF scaling a
   předchozí limit flags mixeru. Pořadí původních AP PID a motor setters se nemění.
3. `AC_PID::shadow_snapshot() const` před a po každém PID poskytne jeho parametry,
   relevantní historii a PID info. Metoda nepřepisuje žádný AP stav. Zachyceno je
   též číslo primárního gyra, čas čtení gyra a frontend INS `last_update_usec`.
   Poslední údaj není čas jednotlivého raw backend vzorku.
4. Původní AP vykoná `motors_output()`. V již existujícím FAST_TASK
   `utb_shadow_update`, před historickým `UTB_ENABLE` guardem, proběhne nezávislý
   výpočet a porovnání. Úloha ani její priorita nebyly přidány nebo přesunuty.
5. `UTB_Rate::shadow_rate_step()` použije tři vlastní `UTB_PID`, stejné scaling,
   limity, actuator a FF pořadí jako nativní Multi. Vrací jen strukturu šesti
   čísel R/P/Y a FF. Nevytváří ani nedostává objekt `AP_Motors`, motor sink,
   `UTB_Core`, `UTB_Attitude` nebo živý AP controller. Jde o hodnotovou projekci
   již existující 3A rate matematiky, nikoli další PID algoritmus.

Pořadí je INS → nativní AP rate/capture → AP motor output → UTB compare → další
fast tasks, včetně pozdějšího `update_flight_mode`. Reference se nepřepočítává z RC:
UTB obdrží přímo skutečný target použitého AP kroku. AP controller se nespouští
podruhé. AHRS/Attitude není v této fázi porovnáváno.

### Historie a synchronizace

`AC_PID_Shadow` je hodnotový formát bez vlastnictví filtrů a bez ukazatele na AP.
Obsahuje 12 parametrů (P/I/D/FF/IMAX/PDMAX/FLTT/FLTE/FLTD/SMAX/DFF/SRT), slew scale,
integrační stav, filtrovaný target/error, obě derivace, příznaky reset_filter a
I_set a PID info. Nepoužívá se kopie paměti celého PID objektu ani jeho paddingu.

- `UNS_RUN=1`: import relevantní prehistorie v každém kroku. Ověřuje jeden krok,
  ne nezávislou dlouhodobou historii.
- `UNS_RUN=2`: import pouze na začátku epochy; následně zůstávají vlastní filtry a
  integrátor. Přebírají se jen skutečné externí AP události `I_set`/reset_filter,
  každá se počítá. To zahrnuje AP relax/reset integrátoru v DISARMED. Bez těchto
  událostí se integrátor z AP každý cyklus neobnovuje.
- Změna parametrů, primárního gyra, chybějící capture nebo sequence gap způsobí
  přiznaný DESYNC a vynechané porovnání. Následující podporovaný krok zahájí novou
  synchronizační epochu. Skrytá náhrada chybného výsledku nulovou chybou není.
- Chybějící celé consume/audit mezi dvěma prepare ukončí diagnostiku důvodem 7;
  nepřepsaná připravená sada je vykázaná jako `unconsumed_attempts`.

Podpora je nyní omezena na **SMAX přesně 0 a žádný nakonfigurovaný ani alokovaný
notch target/error filtr**. Nenulové SMAX, notch a jiné execution kontexty jsou
UNSUPPORTED, nikoli PASS. Parametry původního AP se kvůli shodě nemění.
Při SMAX=0 neovlivňuje interní slew limiter výstup; jeho wall-clock diagnostická
veličina `slew_rate` a související privátní historie nejsou synchronizovány ani
porovnávány. Dmod a všechny složky ovlivňující podporovaný výstup porovnávány jsou.

V rate threadu se nejdříve ověří main-thread kontext, teprve potom se smí číst
capture příznak. Jakmile byl rate thread jednou spuštěn, zůstává celá boot session
UNSUPPORTED. Nepřistupuje se pak k AP PID ani motorovým getterům; nevzniká
závod při jejich přebírání mezi vlákny.

### Statistiky, zastavení a export

Každý pokus patří právě do valid/invalid/desync/unsupported; mismatch je podmnožina
validních spočítaných sad. Pro 18 veličin na každé ze tří os jsou vedeny maxima
absolutní chyby a součty čtverců; RMSE se dopočítá až při exportu. Patří sem
filtrovaný target/error, derivace, integrační historie, P/I/D/FF/DFF, Dmod,
výsledný zásah a FF do mixeru, limit/PD_limit/reset/I_term_set a actual gyro.
NaN/Inf a neplatné dt jsou vykázané jako invalid a nikdy neoznačují numerickou shodu.

`UNS_COUNT` (1–200000, default 4000) se převezme na začátku. Počet omezuje konečné
měření a rozsah akumulátorů, není to nový letový limit. `UNS_RUN` default 0 a při
bootu se vždy vymaže v RAM. Jedna session za boot; nový RUN již publikovaný stav
nepřepíše. Dokončení COUNT, požadavek STOP, první numerická neshoda, neplatný
požadavek, samotný consume ≥2500 µs, změna motorových čísel nebo chybějící consume
ukončí pouze diagnostiku. Hranice 2500 µs plyne z celé nominální periody 400 Hz;
není důkazem, že kratší consume je bezpečný ve zbývajícím rozpočtu hlavní smyčky.

Fast path nevolá logger, MAVLink, textový export, mutex, heap ani thread_create.
Úložiště a vlastní PID instance jsou pevné. Resetují se pouze numerické statistiky,
nikoli surová paměť PID/filter/atomic objektů. Žádný nový worker ani sdílená fronta.

Po stop se stav jednou publikuje lock-free atomic release/acquire a už se nemění.
`@SYS/utb_rate_shadow.txt` vrací před stop pouze pending; při armed nevydá výsledek.
Čtení má proběhnout výhradně po okně a DISARMED. Stávající infrastruktura @SYS/
MAVFTP používá vlastní formátování/alokace mimo fast path; neznamená to, že veškerá
původní AP komunikace má nulovou režii. Souběžný reader nečte rozpracovaný rámec ani
64bitové živé čítače. Export velkých součtů je přesný celočíselný `%llu`.

## Přesný manifest této fáze

Změněno proti předimplementačnímu checkpointu:

| Soubor | Zásah |
| --- | --- |
| `ArduCopter/Attitude.cpp` | Příprava capture a explicitní execution guard před native rate. |
| `ArduCopter/Copter.h` | Guarded pevný diagnostický objekt. |
| `ArduCopter/Parameters.cpp` | Samostatná skupina UNS_, volný index 109. |
| `ArduCopter/Parameters.h` | Nová enum položka, původní indexy zachovány. |
| `ArduCopter/system.cpp` | Boot init, runtime OFF. |
| `Tools/scripts/build_options.py` | Volba UTB_NATIVE_SHADOW, default OFF, závislost UTB. |
| `Tools/utb/tools/test_utb_skeleton.py` | Volitelný vlastní UDP port simulovaného receiveru, default beze změny. |
| `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp` | Read-only hooks kolem skutečného native PID. |
| `libraries/AC_PID/AC_PID.cpp` | Přidaná const projekce stavu, původní matematika nedotčená. |
| `libraries/AC_PID/AC_PID.h` | Guarded deklarace projekce. |
| `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp` | Read-only neměnný @SYS snapshot. |
| `libraries/AP_UTB/AP_UTB_config.h` | Dostupnost matematiky pro nový SHADOW, nezměněný SITL active guard. |
| `libraries/AP_UTB/Copter/utb.cpp` | Consume po původním motor outputu a read-only motor audit. |
| `libraries/AP_UTB/UTB_PID.cpp` | Projekce a restore vlastní relevantní historie. |
| `libraries/AP_UTB/UTB_PID.h` | Guarded deklarace těchto metod. |
| `libraries/AP_UTB/UTB_Rate.cpp` | Statický hodnotový výpočet tří existujících PID. |
| `libraries/AP_UTB/UTB_Rate.h` | Vstupní/výstupní hodnotové struktury a metoda. |

Přidáno:

| Soubor | Účel |
| --- | --- |
| `libraries/AC_PID/AC_PID_Shadow.h` | Lehký default-OFF guard a hodnotový formát stavu. |
| `libraries/AP_UTB/AP_UTB_NativeShadow.h` | Pevné rámce, statistiky, wrapper. |
| `libraries/AP_UTB/AP_UTB_NativeShadow.cpp` | Engine, klasifikace, synchronizace, immutable export. |
| `libraries/AP_UTB/tests/test_utb_native_shadow.cpp` | Unit oracle proti skutečnému AC_PID a lifecycle testy. |
| `Tools/utb/tools/test_utb_native_shadow_sitl.py` | Vlastněné lokální SITL scénáře, sběr frozen výsledku a nativních logů. |
| `docs/UTB_PHASE_3D3_RATE_SHADOW_CS.md` | Tento stav, důkazy a omezení. |

Celkem 17 změněných a 6 nových souborů této fáze. Původní dirty soubory loggeru,
scheduleru a dokumentace se tím nestávají novými změnami 3D-3. `source-audit.json`
ověřuje jejich zachování proti uloženému checkpointu.

## Ověření

### Unit, build a metadata

- Nových 23/23 unit testů PASS. Jednokrokové a kontinuální historie proti AC_PID,
  filtry, integrátor, saturace/PDMAX, FF/DFF, reset a I_set, změna konfigurace,
  chybějící pre/post capture, sequence gap, NaN/Inf/dt, gyro epoch, SMAX/notch,
  rate thread, záměrná neshoda zachovaná ve statistikách, přetečení sekvence a
  32bitových hodin, chybějící consumer a zákaz přepsání publikované session.
- Stávající `test_utb`: 47/47 PASS. `test_utb_math`: 3 PASS a 1 SKIP
  (`OfflineVectors` bez externí volitelné sady), nikoli 4 PASS.
- SITL ON/OFF a SkystarsH7HD-bdshot ON/OFF sestaveny úspěšně. Všechny používají
  vypnuté aktivní UTB_RATE_SITL, starý native bench a logger trace.
- Param parser úspěšný; UNS_RUN/UNS_COUNT jsou samostatné názvy do 16 znaků.
- flake8 změněných Python zdrojů a build_options PASS; astyle nových C++ souborů
  beze změn; `git diff --check` PASS. Původní rozsáhlé soubory se neformátovaly celé.
- Logging-disabled SITL varianta se v této fázi nekvalifikovala. Známá původní
  chyba `AC_CustomControl.cpp` při logger OFF není opravována ani označena za novou
  regresi 3D-3.

Reprodukce z kořene repozitáře (bez sudo):

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
A="$PWD/tmp/utb-3d3-20261021_191141"
./waf configure --board sitl --enable-UTB --disable-UTB_RATE_SITL \
  --disable-UTB_NATIVE_BENCH --disable-UTB_LOGGER_TRACE \
  --enable-UTB_NATIVE_SHADOW --out="$A/SITL_ON"
./waf copter -j6
./waf --targets tests/test_utb_native_shadow,tests/test_utb,tests/test_utb_math -j6
# OFF: stejné configure s --disable-UTB_NATIVE_SHADOW a --out="$A/SITL_OFF".
CXXFLAGS=-fstack-usage ./waf configure --board SkystarsH7HD-bdshot \
  --enable-UTB --disable-UTB_RATE_SITL --disable-UTB_NATIVE_BENCH \
  --disable-UTB_LOGGER_TRACE --enable-UTB_NATIVE_SHADOW --out="$A/H743_ON"
./waf copter -j6
# H743 OFF: --disable-UTB_NATIVE_SHADOW a --out="$A/H743_OFF".
.venv/bin/python Tools/utb/tools/test_utb_native_shadow_sitl.py \
  --binary "$A/SITL_ON/sitl/bin/arducopter" --output "$A/new-owned-case" \
  --comparison 2 --case flight
```

Waf konfigurace musí běžet sériově kvůli společnému root locku. Scénáře také běží
sériově, protože původní SITL SERIAL1 ponechává port 5762. `qualify.py`,
`run-acceptance.py` a jejich JSON/logy v artefaktech obsahují celé provedené příkazy.

### Finální live SITL výsledky

Finální binárka SITL SHA-256: `447e89e27c0f69374b9aa5f454695dba3b9980432d48097ceadb604b3bbb928e`. Celkem 12 sériově provedených scénářů (11 s RAM session a jeden runtime OFF).

| Scénář (`acceptance-*`) | Pokusy | Valid | DESYNC | UNSUPPORTED | Synchronizace | Max. chyba / RMSE |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| configuration | 8000 | 7999 | 1 | 0 | 2 | 0 / 0 |
| continuous | 8000 | 8000 | 0 | 0 | 1 | 0 / 0 |
| flight | 16000 | 16000 | 0 | 0 | 1 | 0 / 0 |
| flight-ff | 16000 | 16000 | 0 | 0 | 1 | 0 / 0 |
| flight-input | 16000 | 16000 | 0 | 0 | 1 | 0 / 0 |
| flight-one-step | 16000 | 16000 | 0 | 0 | 16000 | 0 / 0 |
| gcs-failsafe | 16000 | 16000 | 0 | 0 | 1 | 0 / 0 |
| one-step | 8000 | 8000 | 0 | 0 | 8000 | 0 / 0 |
| rc-failsafe | 16000 | 16000 | 0 | 0 | 1 | 0 / 0 |
| unsupported-smax | 8000 | 0 | 0 | 8000 | 0 | nevyhodnotitelné |
| unsupported-thread | 8000 | 0 | 0 | 8000 | 0 | nevyhodnotitelné |

Součet: 136000 pokusů, 119999 validních, 1 záměrný DESYNC při změně konfigurace, 16000 explicitně UNSUPPORTED, 0 invalid, 0 mismatch, 0 unconsumed. Každý finite window skončil důvodem 1.

Letový `flight-input` použil nenulové reference nad deadzone a přepnutí STABILIZE → ACRO → STABILIZE, včetně změny znaménka. Max. nativní požadované rates R/P/Y byly 17,381 / 18,591 / 30,418 °/s. Integrátor nebyl nulový (max |I| 0,0119042), průběžné I importy 0, jedna počáteční synchronizace. Jde o mírný pilotní profil, nikoli agresivní akrobacii nebo celý dynamický envelope. Saturace a PD limiting jsou ověřeny v unit testech a DISARMED limitech, nikoli saturovaným letem.

V odděleném `flight-ff` byly skutečně nenulové FF (max 0,000254036) a D_FF (max 0,000297036), oba s nulovou odchylkou AP/UTB. V normálním kontinuálním DISARMED běhu bylo 23997 I_set událostí; v kontinuálním letu, FF letu a `flight-input` bylo I_set=0.

Rate-entry frekvence ze simulovaných monotónních časů: 399.993379–400.010042 Hz; min/max periody 2499/3332 µs, průměrný absolutní jitter přibližně 1,93–2,04 µs. Krátké dlouhé periody jsou přiznané; není to důkaz HW deadline. Všech 16000 native RATE záznamů podporovaných letových oken je v logu přítomno. U UNSUPPORTED FSTRATE není logger rate ani průběh jiné nativní smyčky vydáván za podporované 400Hz porovnání. Přesné tabulky jsou v `acceptance-summary.csv/json`.


Numerická shoda znamená 54 porovnávaných veličin se stejnými skutečně zachycenými
vstupy v daném podporovaném kroku. Neznamená podporu každé AP konfigurace.
DISARMED kontinuální test má mnoho skutečných I_set událostí AP; nezávislý
integrátor bez průběžného importu prokazuje letový kontinuální případ.
FF profil používá pouze v izolovaném SITL experimentu FF=0,05 a D_FF=0,002 na
všech osách; nejde o změnu firmware defaultů ani PID tuning.

RC failsafe se ověřuje nativním MODE reason=3 (RTL/LAND). GCS test ponechává AP
FS_OPTIONS=16: výsledkem je nativní `GCS Failsafe - Continuing Pilot Control`,
ERR subsystem=8/code=1, nikoli vynucený přechod RTL. Samotné zastavení GCS
heartbeat při současném RC_OVERRIDE nestačí, protože override obnovuje GCS
last-seen. GCS test proto ponechá zdravý samostatný simulovaný UDP receiver a
zastaví heartbeat i overrides. Žádný fyzický RC/COM port není použit.

Nativní AP BIN logy slouží jen jako externí důkaz testu. Nový SHADOW nevytváří
DataFlash záznamy. Čtení frozen snapshotu probíhá až po okně a DISARMED. Varování
MAVFTP „expected 100000“ pochází z obecné virtuální velikosti @SYS; skutečný přenos
končí korektně kratším EOF. Analyzátor ověřuje úspěšný přenos a strukturu výsledku.

### Motorová izolace a OFF/ON

Na každém podporovaném kroku byly bezprostředně před a po UTB compare načteny
stejné nativní R/P/Y a tři FF hodnoty AP_Motors; změn je nula. V unsupported
execution kontextu se audit neprovádí a je explicitně počítán jako skipped.
Žádné nativní motorové settery nepřibyly, původní pořadí nativních statementů po
odstranění guarded diagnostických bloků je totožné s checkpointem. Původní
`AC_PID.cpp` je beze změn zachován před přidaným const accessor.

AP_Motors, HAL/DShot, AHRS/INS, arming/failsafe a 3A UTB_Attitude/Core/Output nebyly
upraveny. Compile-out ověřen pomocí linked ELF symbolů: na H743 chybí aktivní
UTB_Core::update_rate, ModeUTBAcroSITL, ModeUTBAngleSITL i UTB_Output::roll; SHADOW
symboly jsou jen v ON. V OFF nejsou ani AC_PID::shadow_snapshot a engine.

Celkem 128000 same-cycle motor auditů, 0 změn; 8000 explicitně vynechaných při rate-thread UNSUPPORTED. U runtime OFF byl proveden tentýž pilotní letový scénář. V nezávislém časově přibližně zarovnaném srovnání 4000 bodů/40 s měly nativní RCOU C1–C4 max. rozdíl 1 µs a RMSE 0,456–0,472 µs. RATE ROut/POut/YOut max. rozdíl 0,002772 / 0,002660 / 0,000978; to nejsou AP/UTB matematické odchylky jedné sady. Metoda a jednotlivé hodnoty: `motor-rerun-comparison.json`.


Dva samostatné uzavřené SITL běhy mají stejné pilotní scénáře, ale ne stejné
naměřené senzorové historie a přesné host časy doručení RC. Jejich celé motorové
trajektorie tedy nelze vydávat za bitově shodné ani rozdíl automaticky připsat
SHADOW. Přímým důkazem invariance při totožném živém AP stavu je výše uvedený
same-cycle audit a read-only source audit. Přesný replay dvou celých běhů se
shodnými senzorovými historiemi není touto sadou doložen.

### Režie, RAM a stack

| Veličina | H743 OFF | H743 ON | Rozdíl |
| --- | ---: | ---: | ---: |
| BIN image B | 1245084 | 1253660 | +8576 |
| .text B | 1240072 | 1248644 | +8572 |
| .data B | 4172 | 4172 | +0 |
| .bss B | 119880 | 122536 | +2656 |

Statická RAM navíc je 2656 B. Velikost `.bss` neobsahuje falešný součet rezervovaného heap/crash prostoru, který obecný `size` u linker scriptu může započítat. Rozdíl BIN a `.text` obsahuje též zarovnání.

| APJ | Velikost B | SHA-256 |
| --- | ---: | --- |
| ON | 1131335 | `aebc7cfa73773e552e790a289f7c83f16c8794bed71b56a6101fc4daf18775aa` |
| OFF | 1123579 | `37a1443d552b7dc270b88012eb71f6bea22c4fe1f76b47dd22a2611e4858989a` |

APJ jsou v `H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj` a odpovídající `H743_OFF` pod artefaktovým adresářem. Nic z toho nebylo nahráno. Host unit engine-only měření: N=6000, průměr 1002,7 ns, medián 881 ns, P95 1222 ns, P99 2355 ns, maximum 178923 ns včetně možného host preemption.


Časy math, consume a native+capture se měří pomocí AP_HAL::micros(). Tyto tři
veličiny nejsou zaměnitelné: capture span obsahuje také původní AP výpočet a
nevyjadřuje čistou dodatečnou režii. Perioda je rozdíl rate-entry časů; dt je
skutečný AP vstup, ne pevně dosazených 2,5 ms.

V použitém SITL jsou hodiny během regulačního kroku virtuálně zastavené, proto
výpočetní časy vycházejí 0 µs. **Nejde o nulovou režii ani o měření na H743.**
Host-only unit microbenchmark měří pouze engine s připravenými hodnotami,
nezahrnuje live capture, motor audit, přípravu ani celou AP smyčku. Přesné hodnoty
jsou v `test_utb_native_shadow-qualification.log`; nejsou odhadem HW WCET.

Statické `.su` rámce nejsou změřenou rezervou stacku ani úplným call-chain WCET.
Sestavení používá -fstack-usage; hlavní rámce: engine process 544 B, native rate
272 B, Copter consume/audit 216 B, snapshot 72 B, rate helper 32 B, prepare/start
16 B. Linkované pstack 7168 B a mstack 1536 B nejsou důkazem dostatečné rezervy.
Na H743 dosud chybí watermark, skutečné CPU, maxima celého capture/compare/audit,
režie RUN=0 a dopad na ostatní úlohy. Publikační atomic bool je compile-time
ověřené lock-free; v okně není nová synchronizace s loggerem.

### Opravené problémy během kvalifikace

- Původní diagnostický motor audit četl motory i při UNSUPPORTED rate-thread
  profilu; nativní worker je mohl mezi čteními legitimně změnit. Audit je nyní
  v tomto kontextu vynechán a počítán jako skipped; PID capture je také blokován.
  Nešlo o změnu motorů způsobenou UTB.
- Přidán stop při chybějícím consumeru místo tichého přepsání rámce.
- Opraven parser vědeckého formátu a následně nahrazen float export velkých
  časových součtů přesnými 64bitovými celými čísly.
- Předběžný GCS runner očekával RTL navzdory nativní FS_OPTIONS=16; ověření nyní
  respektuje původní politiku AP. Jeden souběžný startup test neproběhl kvůli
  obsazenému SITL portu 5762; finální kvalifikace je sériová.

Neúspěšné předběžné artefakty zůstaly zachovány. Za finální sadu se považuje jen
`acceptance-*`, nikoli starší `release-*`, `qualified-*`, `complete-*` či
neúspěšný `final3-*`.

## Omezení a další nejmenší hardware krok

Implementace je kandidát pro **samostatně schválené krátké DISARMED H743 ověření**,
nikoli pro let nebo aktivní 400Hz UTB řízení. Předběžná podpora SMAX=0/notch=0
vychází také ze staršího offline parameter backupu 3D-2; aktuální konfigurace FC
nebyla v této fázi přečtena.

Návrh dalšího testu, dosud neprovedený ani neschválený:

1. Samostatně schválit přesný nový APJ a jeho SHA; ověřit board 1075, správný USB
   serial, DISARMED, bez vrtulí/LiPo a zdravý startup. Dosavadní diagnostický FW
   3D-2 není tento nový SHADOW build.
2. Read-only ověřit FSTRATE=0, main loop 400 Hz, SMAX a notch všech os. Nepodporovaný
   AP profil nepozměňovat kvůli výsledku. UNS_RUN musí být 0. Zachovat výchozí
   parametry a motorovou autoritu AP.
3. S již ustálenou inicializací nejprve RUN=0 baseline. Během RAM okna žádný
   download/list logů, @SYS čtení, změny loggeru ani aktivace LOG_DISARMED.
   Flash kapacita není pro RAM-only session spotřebovávána.
4. První krátké okno COUNT=1000, RUN=1 (nominálně 2,5 s). Po automatickém RUN=0
   DISARMED stáhnout frozen snapshot; požadovat všechny pokusy vysvětlené,
   žádný mismatch/invalid/neočekávaný desync, nulové motor_changes. UNSUPPORTED
   není numerický PASS.
5. Změřit skutečné H743 math/consume/capture, periodu, PM, CPU a stack watermark.
   Celková capture+audit režie vyžaduje odlišení od native práce; samotné zde
   exportované span metriky neuzavírají celý overhead rozpočet.
6. Další boot/session a kontinuální COUNT=4000 až v samostatně schváleném rozsahu.
   V DISARMED očekávat skutečné externí I_set resety; tím se neověří integrátor
   v aktivní letové smyčce. Čtení @SYS/tasks.txt může mít vlastní side effect
   SCHED_OPTIONS bit0, proto jej automaticky nepřidávat do měřeného okna.
7. Při InternalError, změně arming stavu, závadě EKF/loggeru, restartu nebo
   významném zhoršení smyčky experiment zastavit. Žádný ARM, motorový příkaz,
   automatický reboot nebo opakovaný pokus bez vyhodnocení.

Původní logger anomálie 5,5–6,5 ms není touto fází opravena. RAM-only SHADOW se jí
vyhýbá odstraněním svého průběžného exportu; neprokazuje však, že běžný AP logger
nemůže dále blokovat. Hardware časování, komplexní nepodporované konfigurace,
přesný full-trajectory replay a jakýkoliv let zůstávají neověřené.
