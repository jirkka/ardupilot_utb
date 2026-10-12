# UTB: kompaktní diagnostika pro 400 Hz

## Stav a motivace

Implementováno: volitelný bezeztrátový formát `UTB_LOG_FMT=1` (Packed v1), nový statický log `UTBC` ID 31. Default `UTB_LOG_FMT=0` zachovává původní UTBR/UTBM/UTBA/UTBT a BENCH UBEP. Parametr má nový index 21; staré indexy a log ID se neposouvají. Přepnutí za běhu mění pouze serializaci snapshotů, nezpůsobuje reset controlleru. Každý snapshot nese vlastní formát.

Dosavadní H743 měření původního formátu: kolem 331 kompletních sad/s při požadovaných 400 Hz, trvalé backend drops a minimum RAM bufferu pod 1024B rezervou. Podrobná evidence je v [auditu propustnosti](UTB_LOGGER_THROUGHPUT_AUDIT_CS.md). Příčinou nelze označit samotnou frekvenci controlleru: ten nadále běží v každém shadow cyklu.

Návrhový odhad: BENCH ON sada klesá z 339 na 242 B (o 28,61 %), 400 sad/s z 135600 na 96800 B/s. Při dosavadním ostatním toku asi 21 kB/s by celkový tok byl kolem 118 kB/s, pod pozorovanými přibližně 131 kB/s. Tento odhad neprokazuje udržitelnost během flash erase ani na zaplněném médiu. BENCH OFF původní sada má 320 B; packed obsahuje epoch i bez BENCH.

**Hardware částečně ověřeno 11. 10. 2026:** firmware `34990b72` nahrán. C: 4000 platných sad / 20 s = 200 Hz. D: 2000 úplných sad / 5 s = 400 Hz pouze transport, všechny neplatné kvůli nezdravému EKF; plánovaný zdravý D test STOP. Podrobnosti, startup ztráty a omezení: [hardware report](UTB_PACKED_HW_20261011_CS.md). Aktivní UTB_ACRO ani FÁZE 2 nejsou povoleny.

## Přenos a zachování hodnot

Packed worker zavolá jednou `WriteBlock_first_succeed()` pro celou sadu. BLOCK backend přijme celý packet do původního RAM bufferu, nebo odmítne celý packet. To odstraňuje částečné přijetí jednotlivých členů UTB sady; úspěšné přijetí do bufferu stále neprokazuje uložení na flash. Politika backendu, rezervy, zámky, priority, flash/SPI operace a ostatní AP logy se nemění.

UTBC hlavička: TimeUS uint64, Seq uint32, Epoch uint32, Md uint8, APUS uint64, APS uint32, APM uint8, Cap uint8, RCms uint32. Následují D0/D1/D2 (tři standardní DataFlash `a`, každé int16[32]), Dr uint32, Ms uint32, Eff float32. S DataFlash 3B hlavičkou má paket 242 B; FMT `QIIBQIBBIaaaIIf`. Log ID 31 je ve vozidlovém rozsahu 0–31; common messages začínají 32.

D0/D1/D2 přenášejí původní bajty, nikoli numericky převedené nebo kvantované hodnoty. Payload 192 B má little-endian layout:

| Offset v payload | Obsah |
|---|---|
| 0, 38, 76 | Osa R/P/Y: Des, Meas, Err, P, I, D, Raw, Out, AP jako 9 float32; Val a Sat uint8 |
| 114 | R/P/Y command, collective, M1–M4, scale, shift: 10 float32 |
| 154 | Mixer Val, Lo, Hi: 3 uint8 |
| 157 | Achieved R/P/Y/collective: 4 float32 |
| 173 | Pos, Neg, Scale, Shift: 4 uint8 |
| 177 | Dt float32 |
| 181 | Ex uint32 |
| 185 | Req uint16 |
| 187 | Max uint32 |
| 191 | Version uint8 = 1 |

Každá původní float32 hodnota, maska, časová značka a počitadlo je zachována. Sat používá stejnou residual/integration-gate podmínku jako původní worker. Chybějící kompletní packet se nedopočítává. Nepodporovaná verze nebo nesprávná délka pole způsobí explicitní chybu offline dekodéru.

Mission Planner přímo vidí metadata a pole D0–D2; jednotlivé PID/mixer sloupce je nutné rozbalit offline. Pro původní přímé grafy lze ponechat LOG_FMT=0. Původní BIN se nikdy nepřepisuje. Logy smí obsahovat oba formáty; klíč je původní (TimeUS, Seq), epoch se zachová. Duplicitní companions jsou v analýze nekompletní sada, nikoli dodatečný vzorek.

```sh
.venv/bin/python Tools/autotest/utb_log_decode.py downloaded.bin --output decoded.jsonl
.venv/bin/python Tools/autotest/utb_bench_analysis.py downloaded.bin \
  --start-us 35000000 --end-us 55000000 --output analysis.json
```

JSONL je logický pohled do UTBR/UTBM/UTBA/UTBT/UBEP, nikoli nový DataFlash BIN. Počty fyzických bajtů se musí měřit z původního UTBC packetu; rozbalené companions nejsou další fyzické zápisy. Interval 35–55 s je příklad; před session zapsat skutečné celé měřicí okno včetně mezer a neuložených sad.

## Režie a izolace

Main pouze přečte AP_Int8 parametr a uloží jeden bajt do již existujícího snapshotu. ARM sizeof Inputs zůstává 128 B, Snapshot 336 B; pevná queue se nezvětšila. Není přidán clock read, atomická operace, heap alokace, čekání ani logger write v produceru. UTB_ENABLE=0 nadále vynechá producer a worker.

ARM prologue: producer má stejný rámec 160 B + uložené registry, evaluate 356 B + registry, consumer 836 B + registry. Encoder používá dalších 192 B lokálních dat + 32 B registrů během svého volání. Encoder a backend call se nenestují do sebe. Tato statická analýza není měřením nejhorší spotřeby worker stacku ani CPU času. Worker stack allocation 3072 B i watermark mechanismus se nemění; na H743 znovu změřit skutečné minimum. Historických 1848/1856 B nelze vydávat za novou rezervu.

BENCH_ON flash image oproti HEAD ba793af1 roste o 752 B (1249240 → 1249992 B). Přesnou časovou režii encoderu a prodloužení jednotlivého 242B logger acquire/hold bez HW neznáme. Méně zápisů nemusí znamenat menší maximum jednotlivého mutex hold. Zůstává nepřímé blocking riziko sdíleného loggeru i UNKNOWN atribuce; nulové confirmed collisions jej nevylučují.

RateController, MotorMixer, Reference, State validity, arming, FSTRATE, AP_Motors, HAL/DShot a EKF nebyly změněny. AP controller se nevolá podruhé. Nový encoder nemá přístup k motorům ani HAL výstupům.

## Navržený první hardware experiment

Firmware identifikovat přesným SHA-256, targetem a zdrojovým manifestem. Historické softwarové APJ níže obsahují základ ba793af1; nový hardware artefakt obsahuje commit 34990b72 a jeho přesný SHA je v hardware reportu. Použít výhradně archivovaný BENCH_ON artefakt, nikoli poslední soubor v přepisovaném build adresáři.

1. Nejdříve schválit nahrání nového artefaktu. Bez vrtulí, DISARMED; záloha parametrů i všech dosavadních flash logů. Ověřit RC a známou skutečnou FRAME_CLASS=1/FRAME_TYPE=18; nepřepisovat geometrii ani gains.
2. Logger Skystars: LOG_BACKEND_TYPE=4, LOG_DISARMED=1; stejné původní LOG_BITMASK, LOG_FILE_BUFSIZE, ostatní log rate a BENCH export. Žádné zvýšení priorit nebo zmenšení AP logování.
3. Po rebootu s UTB_ENABLE=1 nejprve SHADOW=0, počkat na dokončení AHRS initialisation a nový zdravý stav. Nezačínat na zaplněném médiu; mazání pouze po archivaci a samostatném povolení. Během měření nestahovat logy.
4. C: UTB_LOG_FMT=1, UTB_SHADOW=1, UTB_LOG_RATE=200. Krátké předem stanovené okno, SHADOW=0, stažení BIN a nezávislá analýza skutečně uložených platných sad.
5. D až po vyhodnocení C: LOG_FMT=1, SHADOW=1, LOG_RATE=400; omezené krátké okno, následně SHADOW=0. Ověřit skutečný 400Hz scheduler, Calc/Val/Cap, sequence/epoch, 400 platných kompletních UTBC/s přes celé okno, UTBQ/DSF loss delta, RAM buffer minimum, PM.NLon, main jitter/acquire, worker hold/stack/lifecycle. Krátký úspěch neprokazuje dlouhodobou udržitelnost.
6. Stop při armed stavu, TIME_BUDGET, neplatném controller/mixer/stavu, nedostupném loggeru, nárůstu drops/missing v měřicím okně, zaplnění flash, ztrátě worker heartbeat, stack warning nebo nevysvětleném zhoršení PM/main periody či mutex časů proti srovnatelné baseline. Nezavádí se nový univerzální bezpečnostní časový práh. Neúplný tok = neúspěch cíle 400 Hz, i když regulace zůstává zdravá.

Naměřené hodnoty, návrhový odhad toku a dosud neověřená HW rizika musí zůstat oddělené. Parametry po experimentu explicitně přečíst a obnovit do zdokumentovaného výchozího stavu. Neprovádět motor test ani armování.

## Přesný manifest změn

| Soubor | Zásah |
|---|---|
| ArduCopter/defines.h | Nový explicitní UTBC ID31, žádný posun ostatních ID |
| ArduCopter/utb_log.h | Packed 242B packet, metadata a statická kontrola rozměru |
| ArduCopter/utb.cpp | Volitelný jediný worker zápis; původní zápisy zůstávají v legacy větvi |
| libraries/AP_UTB/AP_UTB.h | Parametr AP_Int8 a snapshot byte formátu |
| libraries/AP_UTB/AP_UTB.cpp | AP_Param index21/default0, zachycení formátu v main snapshotu |
| libraries/AP_UTB/AP_UTB_LogPacket.h | Nová explicitní verze a pevný 192B payload |
| libraries/AP_UTB/AP_UTB_LogPacket.cpp | Nový encoder původních hodnot do pevného bufferu |
| libraries/AP_UTB/tests/test_utb.cpp | Tři nové testy lossless hodnot/masek, determinismu a float bitů |
| Tools/autotest/utb_log_decode.py | Nový offline decoder a JSONL export |
| Tools/autotest/unittest/test_utb_log_decode.py | Šest nových wire/completeness/invalid-input testů |
| Tools/autotest/utb_bench_analysis.py | Rozbalení UTBC při analýze; legacy výsledky beze změny |
| Tools/autotest/test_utb_shadow.py | Volitelná packed regrese, runtime přepnutí formátu, dekódování BIN |
| docs/UTB_PACKED_LOG_CS.md | Nový popis formátu, evidence, manifest a bench postup |
| docs/UTB_ARCHITEKTURA_CS.md | Skutečně implementovaný stav a HW omezení |
| docs/UTB_HARDWARE_BENCH_CS.md | Samostatná identifikace packed C/D a HW omezení |
| README_BUILD_WSL_CS.md | Build/test/decode příkazy a stav závislostí |

Celkem 11 změněných tracked + 5 nových souborů. Již existující untracked
`docs/UTB_LOGGER_THROUGHPUT_AUDIT_CS.md` není tímto patchem měněn; zůstává součástí
předimplementačního checkpointu. Ignorované tmp artefakty nejsou zdrojové soubory patche.

## Firmware artefakty

Následující historické softwarové varianty mají board_id=1075 a git_identity=ba793af1. Při těchto buildech šlo o necommitnutý patch nad HEAD `ba793af194004418d7ff3e3460ae8f48df53fab8`, větev
`test/skystars-5inch`. Nebyl vytvořen nový commit ani push.
APJ image byl dekomprimován a porovnán se zachovaným BIN; shoda všech čtyř variant.

Artefakty jsou pod `tmp/utb-packed-log-20261011/`:

| Podadresář | APJ velikost B | SHA-256 APJ |
|---|---:|---|
| SkystarsH7HD-bdshot-UTB_ON-BENCH_ON-PACKED_V1 | 1127695 | `9898254827da113e45e6c05eb3016868af3e67665f4be96344cd119cf59c1fc9` |
| hw-utb-off-bench-on | 1116743 | `8b93861754cef0509b587263a2ca6328d9cb6b2f177086df4c16707357c7505b` |
| hw-utb-on-bench-off | 1123447 | `078405bc1ea5dc0a1024597f508ed2a3b5d1c53c8442b1ce9e592366cf996235` |
| hw-no-logging | 1052115 | `9fe795e8d2c716d48a63820d21d73e5cd5dc2c83cd5e9ddd7183de3d28d4d775` |

Pro navazující hardware experiment je určen výhradně
`SkystarsH7HD-bdshot-UTB_ON-BENCH_ON-PACKED_V1/arducopter.apj`.
Ostatní APJ jsou compile-guard kontrolní artefakty. Po nahrání by bylo nutné
explicitně nastavit LOG_FMT=1; APJ sám parametr nepřepíná. Build adresář po
regresích může obsahovat jinou variantu; z něj se firmware pro session nesmí vybírat.

## Skutečně provedené software ověření

Před změnami: stávající binární unit testy HEAD prošly 39/39 UTB a 12/12 BENCH;
šlo o zachované již sestavené testy, nikoli nový baseline build v této session.
Checkpoint je `tmp/utb-packed-log-20261011/baseline-*` (HEAD archive,
status, tracked diff, samostatný archive untracked souborů). Následné command
logy a výsledky jsou ve stejném adresáři.

Po změnách:

- 42/42 UTB unit PASS, z toho 39 původních + 3 nové packet testy.
- 12/12 BENCH unit PASS; 6/6 nových Python decode unit PASS.
- Kompletní SHADOW SITL PASS pro legacy i packed, včetně 200/400 Hz,
  invalid frame, FSTRATE, neplatných gains, logger suppression, AP módů,
  normálního/forced/RC arming odmítnutí UTB_ACRO, primary gyro, SYSID,
  RC mapping a motorové izolace. Packed navíc přepíná legacy/packed za běhu.
- FÁZE 0 SITL ON/OFF PASS, BF_X_REV regrese PASS, BENCH A/B/C-D/failure PASS
  (zahrnuje bezpečné SITL thread-create failure a logger nedostupnost/recovery).
- 63370 fyzických UTBC z nativních SITL BIN rozbaleno a zpětně sbaleno;
  bajtová shoda všech 192B payloadů, skutečný ID31/Length242/FMT ověřen.
- Legacy analýza skutečných H743 C_.bin/D.bin se shoduje s parserem HEAD:
  14793/40941 kompletních sad přes celé soubory. Jde o offline regresi parseru,
  nikoli nové fyzické měření.
- AP_Param a logger metadata parser PASS; astyle pro všech 8 C++/header
  souborů beze změny, flake8 a git diff --check PASS.
- Source motor/arming isolation PASS, nezměněné bezpečnostní zdroje porovnány
  s HEAD. Copter producer source zůstává bajtově totožný.

| SITL formát | Pozorovaná frekvence při Req200 | Při Req400 |
|---|---:|---:|
| packed | 199.994360 Hz | 398.215382 Hz |
| legacy | 200.003927 Hz | 399.326397 Hz |

Měřeno z intervalů zachovaných kompletních sad při SITL speedup=1 podle
původního 4% tolerančního testu. Není to důkaz přesně 400 bezeztrátových sad/s
na H743. Funkční test také cíleně vyřadí logger a procvičí bounded queue drops;
za celý packed shadow scénář Drop=886, legacy Drop=877. Oba běhy mají
0 partial sets. Tyto celkové counters nejsou baseline hardwarové okno.
SITL Ex=0 a PM hodnoty nejsou hardware CPU měření.

Čtyři cílové Skystars buildy PASS: UTB ON/BENCH ON, UTB OFF/BENCH ON,
UTB ON/BENCH OFF, UTB ON/BENCH ON/LOGGING OFF. Logging OFF encoder
není v ELF, UTB OFF neobsahuje UTBC metadata, BENCH OFF neobsahuje UBST
metadata a ponechá UTBC. BENCH_ON symboly jsou přítomné v určeném firmware.
SITL ON/BENCH ON, OFF/BENCH ON a ON/BENCH OFF build PASS. Packed BENCH OFF
funkční kontrola PASS: 11761 kompletních UTBC, z toho 3761 s platným mixerem,
0 incomplete groups, interior rate 400.001333 Hz; žádný fyzický UBEP/UBST/UBMT/UBHI.
Epoch je přenesena uvnitř UTBC bez závislosti na BENCH. Tento samostatný SITL
běh rovněž neprokazuje H743 throughput.

Navazující [hardware session 11. 10. 2026](UTB_PACKED_HW_20261011_CS.md) částečně ověřila 200Hz validní SHADOW a 5s 400Hz transport, mutex časy, CPU/jitter a stack watermark. Zdravý 400Hz výpočet nebyl ověřen: D STOP kvůli EKF unhealthy; evidovány startup ztráty a dlouhé periody během downloadu po C. Dlouhodobé chování, chip-full dopad a let zůstávají NEOVĚŘENO. AP logger interní locking rizika se tímto patchem neodstraňují. Nic nebylo doinstalováno.

## Plná frekvence a aktuální H743 ověření

Commit acf04efa99 vybírá při požadavku na plnou nominální frekvenci každý
skutečný shadow cyklus; nižší rate zachovává původní wall-time decimation.
Serializace, formát V1, default LOG_FMT=0, controller matematika, guards a motorová
izolace se nemění. Nový BENCH_ON APJ je nahraný a CRC/identita ověřené.
Hardware: 20s DISARMED C 200 Hz = 4 000 platných sad, D 400 Hz = 8 000
platných sad, bez Seq mezery u D, PM.NLon=0 a bez přírůstku pipeline/logger
ztrát v oknech. Firmware/data SHA, software regrese, startup ztráty a download
prodlevy jsou v [aktuálním reportu](UTB_400_RECOVERY_20261011_CS.md).
Testy probíhají před downloadem; před dalším experimentem nový zdravý start.
Dlouhodobé a letové ověření zůstává otevřené.
