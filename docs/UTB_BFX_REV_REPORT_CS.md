# SHADOW BF_X_REV — závěrečný software report (10. 10. 2026)

## Git a rozsah

Větev `test/skystars-5inch`, HEAD `6857c74e19943772f5aff64575bf07c2f5e7ee85`. Změny tohoto rozšíření jsou necommitnuté; žádný stash/reset, push ani merge nebyl proveden. Vývojová větev `ardupilot-4.7.1-utb` zůstává na `8dc66c8d18bfe591387333059c35888af5d08f5a`. Před změnami byl strom čistý; reprodukovatelným základem je původní commit. Hashe finálních změněných souborů kromě tohoto reportu: `tmp/utb-bfx-rev/source-freeze.json`.

Původní1190-parametrický archiv i A0/A/B overlay jsou byte-for-byte nezměněné proti HEAD. SHA archivu `7f39b20c60bc1622a8bb68432408b7f540642e186e762ae6f795db047b71f421`, velikost20966 B. Jediné změny C++ jsou AP_UTB mixer/guard, diagnostická epoch a unit testy. PID/reference/D filtr/anti-windup metoda nejsou změněny.

## Přesný manifest změněných a přidaných souborů

- `Tools/autotest/test_utb_bfx_rev.py`
- `Tools/autotest/test_utb_profiles.py`
- `Tools/autotest/test_utb_shadow.py`
- `configs/skystars_5inch/ANALYZA_KOMPATIBILITY_CS.md`
- `configs/skystars_5inch/OBNOVA_KONFIGURACE_CS.md`
- `configs/skystars_5inch/OVERENI_CS.md`
- `configs/skystars_5inch/README_CS.md`
- `configs/skystars_5inch/bench_C.param`
- `configs/skystars_5inch/bench_D.param`
- `docs/UTB_ARCHITEKTURA_CS.md`
- `docs/UTB_BFX_REV_REPORT_CS.md`
- `docs/UTB_CONTROL_THEORY_CS.md`
- `docs/UTB_HARDWARE_BENCH_CS.md`
- `libraries/AP_UTB/AP_UTB.cpp`
- `libraries/AP_UTB/AP_UTB_MotorMixer.cpp`
- `libraries/AP_UTB/AP_UTB_MotorMixer.h`
- `libraries/AP_UTB/tests/test_utb.cpp`

## Implementace a matematika

- `AP_UTB_MotorMixer.h`: whitelist přesně class1/type12 nebo18, ostatní class/type nevalidní.
- `AP_UTB_MotorMixer.cpp`: jeden allocator; typ18 má yaw_sign=−1, typ12=+1. Stejný sign je použit v forward i inverse yaw. BF_X12 zachovává původní aritmetické pořadí; ostatní části desaturace/limits beze změny.
- `AP_UTB.cpp`: změna class/type zvýší epoch. Existující context reset vynuluje I/D/previous/residual; snapshot se sestavuje znovu a první platný krok je PRIMING s nevalidním controllerem/mixerem. Budget latch a ostatní guards nejsou vymazány. Pipeline counters jsou monotónní lifetime metriky, nikoli resetované per-geometry statistiky; epoch umožní offline oddělení.
- Testy: původní odmítnutí18 bylo záměrně nahrazeno odmítnutím nepodporovaných0/1 a novými testy18. Nové testy používají nezávislé literal reference, nikoli druhé volání stejného mixeru.
- Dokumentace označuje původní FÁZI1 evidence jako historickou; aktuální podporu popisuje teorie oddíl26 a architektura oddílN. Baterie:4S i6S LiPo, beze změny prahů/failsafe.

```text
G12 = 0.5 * [ -1 -1 -1 ]    G18 = 0.5 * [ -1 -1 +1 ]
            [ -1 +1 +1 ]                [ -1 +1 -1 ]
            [ +1 -1 +1 ]                [ +1 -1 -1 ]
            [ +1 +1 -1 ]                [ +1 +1 +1 ]
```

Koeficienty ověřeny proti quad MotorDef, add_motor a normalise_rpy_factors ve skutečném AP_MotorsMatrix této větve. Pořadí M1–M4 je logické pořadí; test order2,1,3,4 a SERVO kanály nejsou zaměňovány. Matematická podpora nepotvrzuje fyzické směry motorů.

Pro obě geometrie GᵀG=I, Gᵀ1=0; dosažený normalizovaný moment je Gᵀm. Y12=(−m1+m2+m3−m4)/2; Y18=(m1−m2−m3+m4)/2. Po společné desaturaci m=T_shift·1+scale·G·c platí achieved=scale·c a collective=sum(m)/4. Yaw sign se proto v achieved vrací do původní soustavy požadavku. Společná R/P/Y priorita, collective shift a clamp[0,1] jsou zachované.

Nezávislý saturující případ R=P=Y=1,T=.5: typ12 dá m=[0,1,1,1],Tach=.75; typ18 m=[0,0,0,1],Tach=.25. Oba scale=.5 a achieved R/P/Y=.5. V testech obou geometrií kladná i záporná yaw saturace zablokovala jen integraci do residual směru a dovolila opačné odvíjení. Přechod12↔18 znovu začíná I=Ki·error·dt, D=0 při konstantním novém měření, bez starého residual. Je to SHADOW ONLY evidence; soustavu stále řídí AP, není to ověření uzavřené UTB smyčky.

## Skutečně spuštěné testy

| Kontrola | Výsledek | Evidence |
|---|---|---|
| Rebuilt UTB unit tests | 35/35 PASS (původních31 +4 nové) | tmp/utb-bfx-rev/unit.log |
| Benchmark unit tests | 12/12 PASS | bench-unit.log |
| FÁZE0 SITL ON | PASS, ordinary/force/RC arm zákazy i armed-entry rejection | skeleton-on.log |
| FÁZE0 SITL OFF | PASS, chybějící UTB parametry/mode, původní AP režimy | skeleton-off.log |
| FÁZE1 BF_X12 | PASS, reference, PID alignment, guards, gyro, logging, queue, isolation | shadow12/results.json |
| BF_X_REV18 | PASS, reference/PID/forward/inverse,200/400Hz, OFF/ON isolation a arming zákazy | bfxrev18-settled/results.json |
| Runtime18→12→18 | PASS, epochs0/1/2 a healthy recovery | tentýž nový SITL report |
| BENCH SITL | PASS A/B/C-D/failure včetně logger recovery a lifecycle | bench-sitl/results.json |
| Profil hash/allowlist | PASS, archiv1190 a všechny A0/A/B/C/D | profiles.log |
| astyle | PASS, všechny4 dotčené C++ soubory dry-run Unchanged | astyle.log |
| flake8 | PASS, všechny3 dotčené Python soubory | flake8.log |
| git diff --check | PASS | diff-check.log |
| Param/log metadata | Registrace AP_Param, IDs/layouts beze změny; skutečné UTB logy parsed | audit.json a SITL logy |

SITL vlastní localhost instance a modely bfx/bfxrev; syntetická RC kalibrace/gains/noise jsou test fixture, nikoli profil pro FC nebo obnovení1190 parametrů. Původní FÁZE0/1 skripty armují nebo létají jen v simulaci pro své kontrolní scénáře. Nový BF_X_REV scénář zůstává DISARMED a pouze ověřuje odmítnuté UTB_ACRO arm requesty. Žádné spojení s fyzickým hardwarem nebylo použito.

BF_X_REV: 11014 valid complete sets, maximální forward odchylka 2.23517417908e-08; interior200Hz=199.999439,400Hz=399.371205. AP output comparison:20 DISARMED ACRO samples/run,0 rozdílů. Nejvyšší rate není důkazem full-window H743 throughput.

BF_X12: 24925 complete sets; primary gyro a exact PID alignment regrese PASS; izolace původního STABILIZE v DISARMED a simulated armed idle PASS. Žádná bitwise flight/transient ekvivalence není tvrzena.

První nový BF_X_REV pokus selhal na startup RC mode selection (STABILIZE místo očekávaného ACRO). Test fixture doplnila ACRO slot a3s settling. Jeden opakovaný běh narazil na paralelní lokální SITL port5762; další běhy byly serializované. Finální bfxrev18-settled běh prošel; PID/mixer algoritmus se kvůli těmto testovým chybám neměnil. Lint chyby nových testů (blank line/line length) byly opraveny. Nebyly oslabené runtime guards ani tolerance matematických assertions.

## Buildy a přesná identifikace artefaktů

| Varianta | Board | UTB/BENCH | Výsledek | Byty APJ / SITL ELF | SHA-256 |
|---|---|---|---|---:|---|
| sitl-on-bench | sitl | 1/1 | PASS | 5722616 | `f46537dc7cb0e3935404dcc7b72b28353bcaf1ff85ca7df3666bced1ff882347` |
| h7-on-bench | SkystarsH7HD-bdshot | 1/1 | PASS | 1126923 | `b9bb1a9df41329a895ba95d93959f8269db65535041c1321d357b93dd12a2591` |
| h7-on-plain | SkystarsH7HD-bdshot | 1/0 | PASS | 1122763 | `82245ccad32f3e8ed0c0b5ce5a130094e944ac57f69420a56d0d7f39028d7104` |
| h7-off | SkystarsH7HD-bdshot | 0/0 | PASS | 1113283 | `06f9f339027307b9b8105e3b2c5a178a8e555544952641f03ff6711af13ca3e7` |
| sitl-off | sitl | 0/0 | PASS | 5686112 | `c4c59d731e8b5bc1904ba3bab3a90c2bf610ede10f77e9bba44b4bab677f6482` |
| h7-no-logging | SkystarsH7HD-bdshot | 1/1 | PASS | 1052099 | `5a9e5458f820f876982adcf7aa0a131eac3b88ffd1d77bd15bad466f1b778409` |
| micoair-on-bench | MicoAir743v2 | 1/1 | PASS | 1296544 | `4719b4f796b7f451fb7022b683b11bc4a8b02400d0f00b960c0fa0e37cceef38` |

Všechny artefakty archivovány v `tmp/utb-bfx-rev/<varianta>/`, jednotlivé manifest.json zahrnují další ELF/bin hashe. Přesné provedené configure/build příkazy a exit codes jsou v [build evidence](../tmp/utb-bfx-rev/results.json). Celá série používá existující .venv a ARM GCC10-2020-q4, ./waf z kořene bez sudo; nic nebylo doinstalováno.

Build h7-no-logging má navíc extra-hwdef HAL_LOGGING_ENABLED=0; není určen k logger benchmarku. h7-on-bench propaguje AP_UTB_BENCH_ENABLED=1 přes --extra-hwdef; BENCH OFF má --disable-UTB_BENCH. MicoAir je kompatibilita společné kompilace, nikoli Skystars profil ani měření jeho filesystem loggeru. SITL ON používá BENCH ON, OFF oba přepínače vypíná; nový samostatný SITL ON/BENCH OFF build nebyl proveden.

Skystars APJ board ID1075. Embedded git_identity je základ6857c74e, protože buildy vznikly v necommitnutém pracovním stromu. **Toto ID samo nerozlišuje starý firmware bez BF_X_REV od nového. Použít přesný SHA-256 a source-freeze manifest.** Nevybírat poslední soubor z build/board/bin; následné konfigurace jej přepisují.

A0 = h7-on-plain + původní bench_A0_A.param. A/B/C/D = jedna h7-on-bench APJ s odpovídajícím samostatným overlay. h7-off/no-logging/MicoAir nejsou náhradou za firmware C/D. Žádný firmware nebyl nahrán.

## Motorová izolace a zbývající rizika

Audit byte equality proti původnímu HEAD:2113 chráněných souborů beze změny, včetně AP_Motors, HAL/ChibiOS/DShot, AC_AttitudeControl, EKF, arming, logger/scheduler a Copter bridge/mode/param/log definitions. AP_UTB library source nemá motor-writer závislosti nebo setters; žádný druhý AP controller se nespouští. Motorovou autoritu má původní AP. Source audit + runtime OFF/ON srovnání jsou uvedená software evidence; hardware netestován.

Při epoch změně worker může dokončit už zahájený old-epoch write. Snapshot si zachová původní epoch; BENCH UBEP/Seq/TimeUS umožní offline oddělení. Nejde o přenos residual do nové geometrie ani o motorový výstup. Queue/feedback/thread/lock architektura není měněna. Sdílené logger mutexy a jejich nepřímé blokování main zůstávají neověřeným H743 rizikem. Zero confirmed collisions není důkaz nulového rizika.

Dosud hardware neověřeno: fyzický BF_X_REV mapping/spin/ESC RPM, kalibrace/senzory, aktuální4S/6S pack a napájení, battery ochrana pro oba packy, main period/jitter/overruns, CPU/producer/consumer čas, mutex latence/hold/UNKNOWN, stack reserve/lifecycle, skutečně persistované complete sets/full-window log rate a kapacita loggeru. Žádný SITL výsledek nepotvrzuje bezpečnost aktivního UTB ani hardware udržitelnost400Hz.

## Návrh první hardware session — zatím nespouštět

1. Samostatně schválit session; před ní žádný upload, arm, motor test nebo let. Odmontovat vrtule, upevnit dron, zajistit napájení/chlazení a možnost okamžitého vypnutí.
2. Zálohovat FC konfiguraci, ověřit target/revizi/board ID a vybraný archivovaný APJ SHA; řídit se [obnovou a fyzickým checklistem](../configs/skystars_5inch/OBNOVA_KONFIGURACE_CS.md). Readback FRAME_CLASS=1/FRAME_TYPE=18, původní SERVO/DShot/failsafe a arming; nepřepisovat frame.
3. Ověřit W25Q128/BLOCK a LOG_BACKEND_TYPE=4,400Hz loop/FSTRATE=0, dostatek místa; zapisovat aktuální pack a všechny UTB gains (overlay je nemění). BENCH ON očekává UB33=4. Matematika při nulových default gains je validní, ale nejde o nenulový PID tune nebo saturující experiment.
4. A0→A→B→C, každá varianta restart/readback,10s warmup a30s měření jako pilotní organizační délky; stejný pack, telemetry, periferie a teplota. Po každé variantě stáhnout a ověřit log. A0 nemá UB metriky.
5. D až po vyhodnocení C bez stop podmínek a nevysvětleného zhoršení:10s warmup a5s měření, potom SHADOW=0 a stažení logu. Délky nejsou číselný bezpečnostní limit ani schválená udržitelnost.
6. Vyhodnotit A−A0, B−A, C−B, D−C přes monotónní HW metriky, PM, jitter/overruns, main wait/worker hold/UNKNOWN, stack/HWM/drops/complete sets. Volit předem zaznamenané full-window TimeUS intervaly; rozdělit epochy. Zastavit při resetu, ztrátě linku, změně arming/mode, pohybu motorů, anomálním napájení/teplotě, TIME_BUDGET, nedostupném loggeru nebo nevysvětleném zhoršení baseline; nepokračovat automaticky doD.

Podrobná metoda a stop podmínky: [hardware bench](UTB_HARDWARE_BENCH_CS.md), oddíly9/10/13. Limity deadline, stack reserve a maximální bezpečný log rate budou určeny z naměřených hodnot a podmínek cílového H743, nikoli odhadem z těchto testů.

Implementace dokončena pouze pro SHADOW. Unit/SITL/build ověřeno výše; hardware neověřeno. Aktivní UTB_ACRO, FÁZE2, merge do vývojové větve, upload a skutečný let nebyly provedeny.
