# UTB-FLIGHT-LOG-1: pasivní Flight BIN logger

Stav k 2026-10-26: **implementováno, unit/SITL a offline build ověřeny; nový firmware na H743 nenahrán a HW zápis nekvalifikován**.
Větev `test/skystars-5inch`, HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a` zůstal zachován.
Evidence této fáze: `tmp/utb-flight-log1-20261026/`. Žádný commit, push, instalace, připojení COM ani fyzický experiment.

## 1. Cíl

Persistentně uložit hodnoty současných UTB_PID/Rate/Attitude vedle native AP hodnot. Výpočet zůstává 400 Hz, záznam pouze 10 nebo 20 Hz. Jde o pasivní diagnostiku původního AP STABILIZE.

## 2. Původní stav a podklady

Navazuje na UTB_FLIGHT_SHADOW_READINESS_CS, HW2B_FLIGHT_EXEC_TIMING, HW2A_NATIVE_EXEC_TIMING, MAIN_OUTPUT_TIMING, 3G3D_FLIGHT_READINESS, 3A a STORAGE_AUDIT. Původní FlightV1 drží konečný RAM záznam, nikoliv persistentní nové PID záznamy. Historické UTBC/UTBR/UTBM nejsou nahrazením této funkce. HW2B a dřívější ARMED SITL jsou referenční výsledky, nikoliv měření nového BIN writeru.

## 3. Přesný manifest této fáze

- `ArduCopter/Attitude.cpp`
- `ArduCopter/Log.cpp`
- `Tools/scripts/build_options.py`
- `libraries/AP_Logger/LogStructure.h`
- `libraries/AP_UTB/AP_UTB_ChainShadow.cpp`
- `libraries/AP_UTB/AP_UTB_ChainShadow.h`
- `libraries/AP_UTB/AP_UTB_config.h`
- `libraries/AP_UTB/Copter/utb.cpp`
- `libraries/AP_UTB/tests/test_utb_chain_shadow.cpp`
- `libraries/AP_UTB/AP_UTB_FlightBin.h`
- `libraries/AP_UTB/AP_UTB_FlightBin.cpp`
- `libraries/AP_UTB/FlightBinLogStructure.h`
- `libraries/AP_UTB/tests/test_utb_flight_bin.cpp`
- `Tools/utb/tools/utb_flight_bin_analysis.py`
- `Tools/utb/tools/test_utb_flight_bin_sitl.py`
- `Tools/utb/tests/test_flight_bin_analysis.py`
- `docs/UTB_FLIGHT_BIN_LOGGER_IMPLEMENTATION_CS.md`

Devět již existujících souborů se liší proti vstupnímu SHA checkpointu. Osm souborů je přidáno. Úplný manifest je `phase-manifest.json`. Dřívější necommitnuté Execution/Timing, USB dokumentace a ostatní práce zůstaly zachovány. Sdílené již změněné soubory nejsou vydávány za čistý diff proti HEAD; rozhodující je `checkpoint-verification.json`.

- FlightBin.*: pevný transport, ARM lifecycle, decimace, čítače, worker a čtení konečných čítačů.
- FlightBinLogStructure.h: čtyři packed zprávy, statické metadata a jednorázový sedmipaketový writer.
- ChainShadow.*: využití jediného existujícího engine; oddělení BIN session od RAM recorderu.
- Copter/utb.cpp: stávající worker vytvořený při startupu i bez historického UTB_ENABLE, předání výsledku thread_create a bounded drain.
- Attitude.cpp: read-only převzetí původního AP failsafe stavu a kořenový compile guard pro dotčené volitelné bloky.
- Log.cpp / LogStructure.h: registrace metadat a nové ID na konci společného enumu; žádné posunutí původních ID.
- config / build_options: default OFF a závislosti. Testy a parser jsou oddělené od C++ regulátorů.

## 4. Architektura

```text
native AP Rate/Attitude → existující value capture → nezávislý UTB Chain comparator
                                                     ↓
                                        vybraná kompletní sada → SPSC(4)
                                                                   ↓
                                                    existující utb_log → AP_Logger
native AP → původní AP_Motors → původní HAL/ESC
```

Nový writer nemá motorové rozhraní. Přímé logger volání existuje pouze v consumeru, ne v produceru. Regulační matematika se neměnila.

## 5. Tok dat a integrační místa

`ArduCopter/Attitude.cpp::run_rate_controller_main` připraví podporovaný main-thread kontext. Původní Multi rate controller provede jeden nativní výpočet a předá existující pre/post kopie do ChainShadow. `ArduCopter/mode.cpp::update_flight_mode` následně dokončí native Attitude a `utb_chain_shadow.finish()` uzavře cyklus. Odtud vznikne případný hodnotový BIN snapshot. Jediný consumer je `Copter::utb_log_thread` s původní IO prioritou a původní requested stack velikostí 3072 B. Nový thread ani změna priorit nevznikly.

## 6. Význam snapshotů

UFPI bere P/I/D/FF/D_FF z `rate.after[axis].info` a nezávislého `rate_engine().history(axis).info`. UFRS nese opravdu použitý gyro, dt, native target, spotřebovaný UTB target, native/UTB control a scaled feedforward. UFAT nese skutečný zdrojový AHRS quaternion a oba target quaterniony. Žádná aproximace nedostupného native PID info; nevalidní/nespárovaný výpočet se neoznačí validní.

## 7. Synchronizace

Attitude(N) je zdroj pro Rate(N+1). Sada nese `TimeUS`, `SrcUS`, `Lp`, `Src` a `Seq`; musí platit `uint32_t(Src+1)==Lp`. Timestamp Rate je consume, SrcUS patří předchozímu primárnímu Attitude. Writer nepřidává sampling senzorů. UTB historie se importuje při počáteční synchronizaci a dále vyvíjí nezávisle; skutečné native I_set/reset události se přenášejí dosavadním mechanismem. Neimportuje se AP integrátor každým cyklem. FSTRATE/rate-thread, unsupported notch/SMAX a nepodporovaný kontext nejsou potichu upravovány.

## 8. BIN zprávy

Ve zde ověřeném SITL registry mají ID 213–216; identitu při analýze vždy určuje skutečné FMT, nikoliv hard-coded ID. Na H743 je stejný registry sestavený a prošel kontrolou počtu ID.

| Zpráva | Velikost včetně 3B headeru | Pole |
|---|---:|---|
| UFPI | 59 B | TimeUS,Seq,Ax,Md,Flg,AP,AI,AD,AFF,ADF,UP,UI,UD,UFF,UDF |
| UFRS | 67 B | TimeUS,Seq,Ax,Md,Flg,SrcUS,Lp,Src,Dt,Gy,AT,UT,AO,UO,AF,UF |
| UFAT | 72 B | TimeUS,Seq,Flg,SrcUS,Q1..Q4,A1..A4,U1..U4 |
| UFLS | 55 B | TimeUS,Start,V,St,Rsn,Hz,Req,Que,Acc,Drp,Inv,Uns,Rej,HWM,Dep,Sec |

Sada = 3×UFPI + 3×UFRS + UFAT = **450 B**. Formát v1 má nejvýše 16 polí, label string nejvýše 64 znaků a všechny zprávy pod limitem uint8 délky. `static_assert` kontroluje packing. Little endian odpovídá cílovému STM32 a zdejšímu SITL. Nativní FMT/FMTU a původní DataFlash filesystem se nemění. FMT metadata mohou existovat v compile-ON/runtime-OFF logu, samotné UFPI/UFRS/UFAT/UFLS payloady při OFF nevznikají.

`Ax` je 0 Roll, 1 Pitch, 2 Yaw. `AP..ADF` a `UP..UDF` jsou native/UTB komponenty. `AT/UT`, `AO/UO`, `AF/UF` jsou target, control a již škálovaný FF. `Flg`: bit0 ARM pozorovaný na začátku cyklu, bit1 valid, bit2 saturation, bit3 externí I_set, bit4 filter reset, bit5 nenulový vlastní integrátor bez I_set v tomto kroku. UFAT zachovává příslušnou Attitude target orientaci, nikoliv nový aktivní controller.

## 9. Jednotky

TimeUS/SrcUS/Start jsou uint64 mikrosekundy od bootu; metadata s multiplier F převádí na sekundy. Dt je přímo sekunda. Gy/AT/UT jsou rad/s (unit E); PID, output a scaled FF mají normalizovanou původní AP škálu, multiplier 1, nejsou to PWM ani moment v N·m. Quaterniony jsou bezrozměrné. Status Hz používá Hz, Sec sekundy. PID info.FF a scaled output FF jsou záměrně odděleny, včetně yaw feedforward scaling.

## 10. Timestamp a wrap

Producer jednou v aktivním main cyklu získá `AP_HAL::micros64()`. Z původních 32bit source/consume timestampů rekonstruuje nejbližší epochu pomocí signed delta k této referenci; nejde o přetypování uint32 na uint64. Referenční čas může předcházet pozdějšímu rate capture. Odmítá záporný absolutní čas a source novější než consume či source/consume odstup nad 100 ms. Tento poslední guard platí pouze pro diagnostickou sadu, ne pro původní řízení. Jednoznačnost rekonstrukce vyžaduje blízkou referenci (<2^31 µs); konečná session a per-cycle capture to zajišťují. Unit testy ověřily 32bit wrap i pozdější capture a více epoch.

## 11. Finite lifecycle

Compile `AP_UTB_FLIGHT_BIN_ENABLED=0` default; dostupnost dále vyžaduje UTB/Flight/Chain a HAL_LOGGING. Runtime `UCS_B_ENABLE=0`, RATE=10, SECS=60. Povolené frekvence pouze 10/20, délka 1–120 s. Hodnota mimo rozsah diagnostiku odmítne.

Povolení musí být pozorováno v DISARMED, poté přijde skutečná ARM hrana v native STABILIZE. Žádné automatické retry či restart session. Limit je současně `SECS*400` pokusů a monotónní wall time. První Rate krok je legitimní warmup; decimované první sady už vyžadují skutečný pair. Jedna session za boot. Engine je sdílený s dosavadní diagnostikou; souběžný RUN je konflikt, nikoliv druhý engine. Původní FlightV1 recorder se nespouští ani nepřepisuje a jeho formát exportu zůstal ověřen samostatnou regresí. Po spotřebování libovolné sdílené session je pro další jinou session nutný nový boot.

## 12. ARM/DISARM

Samotné povolení při již ARMED session nespustí. DISARM záznam ukončí, druhý ARM během stejného bootu jej neobnoví. thread_create se provádí jen při startu; jeho selhání zabrání spuštění produceru (reason WORKER). Opt-in lze před ARM zakázat bez zápisu nových diagnostických sad. Firmware nemá žádný příkaz ARM/DISARM vyvolaný loggerem.

## 13. Mode/failsafe

Povoleno pouze původní STABILIZE, podporovaný 400Hz main context, bez att_enable/custom control. Přečtení původního `any_failsafe_triggered()` vypne pouze diagnostiku. LAND/RTL či jiný mode ji ukončí; návrat na STABILIZE neprovádí retry. Původní AP failsafe/arming/spool se nemění. Důvody: 0 NONE,1 COUNT,2 DISARM,3 MODE,4 FAULT,5 UNSUPPORTED,6 CONFLICT,7 DISABLED,8 TIME,9 CONFIG,10 FAILSAFE,11 WORKER. State: 0 IDLE,1 RUNNING,2 STOPPED. Konfigurace změněná za session ji ukončí.

## 14. Queue

Čtyři pevné celé sady, producer main, consumer jeden stávající worker. Producer copy precedes release head; consumer acquire head, kopie do vlastního lokálního objektu, potom release tail. Producer acquire tail nemůže přepsat dosud čtený slot. uint32 index distance modulo je omezená na kapacitu, wrap je testován. Plná fronta zahodí celou novou sadu a zvýší Drp. HWM je konzervativně pozorovaná producer occupancy před publikací; při souběžném drain může být vyšší než okamžitá hloubka, nikdy se nevydává za přesný průběh každého okamžiku. Lock-free 32bit atomiky jsou compile-time kontrolovány. Epochu během session neměníme, queue se násilně neresetuje pod consumerem.

## 15. Worker a AP_Logger

Jeden průchod odebere nejvýše čtyři sady. Před prvním logger voláním je slot uvolněný. Každý ze sedmi paketů je nabídnut právě jednou přes původní `WriteBlock_first_succeed`; zpráva není transakcí. Stav nejvýše přibližně 1 Hz plus počátek/konec, bez retry ztracených statusů. Původní worker drží svou prioritu a 1ms delay. Compile-ON připravuje worker už při startupu i při runtime OFF a historickém UTB_ENABLE=0; toto má startup RAM/CPU cenu. Žádná nová alokace při ARM či v produceru.

BLOCK cesta může držet write_sem (`AP_Logger_Block.cpp:127–182`), native AP writer na něj může také čekat. `start_new_log` drží sem, IO timer/FinishWrite pracují s flash a SPI, JEDEC WaitReady čeká v 100µs krocích až na failure timeout 5 s. SPI dev_sem a vlastní logger sem nejsou novým transportem odstraněny. Na Skystars zůstává W25Q128/SPI3, LOG_BACKEND_TYPE=4. FILE SITL tyto vlastnosti fyzického BLOCK backendu neověřuje. Nový producer logger nikdy nevolá; **nepřímý dopad sdíleného loggeru na Main přesto není vyloučen**.

## 16. Ztráty a fault politika

Req = vybrané pokusy, Que = vložené celé kopie, Acc = všech sedm paketů přijal alespoň jeden backend, Drp = plná SPSC, Inv = nevalidní vybraná sada, Uns = odmítnutá unsupported session, Rej = alespoň jeden paket nepřijat nebo backend nedostupný. Acc není potvrzení fyzického uložení. Chybějící UFLS nemá automatický retry; jeho absence znemožní kvalifikovat úplnost. Čítače jsou i v existujícím DISARMED frozen `utb_chain_shadow.txt`; finished=1 znamená ukončený pokus o finální status, nikoliv jeho persistenci.

Parser páruje sedm paketů podle TimeUS+Seq a os, kontroluje source identity, duplicity, konečné účty i skutečný počet uložených kompletních sad. Partial backend write, ztracený paket či chybějící celá sada se nedají vydávat za PASS. Více session v jednom vstupu parser výslovně odmítá místo sloučení. Chyba diagnostiky pouze zastaví diagnostiku; native AP pokračuje. Nepřidává se erase, změna gains ani regulátorový fallback.

## 17. Paměť a stack

ARM compile-time sizeof: Sample 296 B, Queue 1200 B (včetně 1184 B slotů), transport 1272 B. Celkový přírůstek statické RAM v H743 ELF je 1288 B, zahrnuje i integraci do Copter/Chain a padding. Queue nikdy neroste.

Startup working area původního workeru je v této přesné konfiguraci podle ChibiOS makra 3568 B plus MemberProc 8 B, před allocator overhead/alignment; requested stack 3072 B není totéž jako celá heap alokace ani změřená volná rezerva. Při historickém UTB_ENABLE=1 mohl tentýž worker existovat již dříve; při UTB_ENABLE=0 představuje compile-ON dodatečnou startup alokaci. Linker heap níže je hodnota před runtime alokacemi.

Compiler `-fstack-usage`: sample 336 B, make_sample 256 B, enqueue 8 B, worker 504 B, utb_log_thread 368 B, Chain prepare 56 B/finish 32 B. Vnořené sample→make_sample spotřebuje součet příslušných rámců; worker může vnořit logger/driver a ISR. Nejde o end-to-end watermark ani záruku dostatečné rezervy. H743 runtime stack dosud neověřen.

## 18. Datový rozpočet

| Frekvence | Sady B/s | +UFLS 1 Hz B/s | 60 s | 120 s |
|---|---:|---:|---:|---:|
| 10 Hz | 4500 | 4555 | 273300 B | 546600 B |
| 20 Hz | 9000 | 9055 | 543300 B | 1086600 B |

Tabulka je formátový rozpočet, nikoliv změřená propustnost H743. Pro konzervativní počáteční/koncové statusy přičíst až 110 B. Nová FMT/FMTU přidají při prvním zapsání 532 B, další společná metadata a původní AP logy jsou mimo tabulku. BLOCK page header navíc ubírá 6 z 256 B na stránku, rezervované sektory a circular erase je nutné posoudit samostatně. 16 MiB flash nelze pokládat za stejně velký volný payload. Staré inventáře nejsou aktuálním měřením kapacity; nyní nebyla FC čtena. Artifact adresář této fáze přibližně 740 MiB; nevznikla kopie repozitáře a žádná historická data nebyla mazána.

## 19. Unit a statické kontroly

125 relevantních C++ testů PASS: FlightBin 11, Chain 20, NativeShadow 23, AttitudeShadow 30, Timing 14, ExecutionTiming 19, původní Flight recorder 8. Pokrývají i copy hodnot, čas/loop wrap, source identity, filtry, integrátor, decimaci 10/20, whole-set drop, dva souběžné SPSC thready, blokovaný fake backend s uvolněným slotem, partial admission, worker failure, reset a stop podmínky. Navíc původní legacy math sada měla 3 PASS a 1 explicitně SKIP (chybí externí offline vectors); guardem vypnutý test_utb_native spustil 0 testů a není započten jako PASS.

48 Python testů PASS (13 nový parser, 17 původní Flight parser, 18 Execution parser). Včetně skutečného FMT BIN fixture, ztraceného paketu/celé sady, partial write, duplicit, NaN/Inf, version/session a quaternion sign equivalence. Param metadata `param_parse.py --vehicle ArduCopter --no-emit` PASS. Flake8 nových Python souborů a build_options, astyle nových C++ a `git diff --check` ověřeny. Pět dotčených translation units prošlo i s HAL_LOGGING_ENABLED=0 a s kořenovým AP_UTB_ENABLED=0 (včetně explicitních nulových Waf feature definic); to je **compile guard ověření, nikoliv celý logging-disabled firmware build**.

## 20. Skutečné SITL scénáře

| Scénář | Uložené úplné sady | Výsledek |
|---|---:|---|
| Compile ON, runtime OFF | 0 | bez nových payloadů; native scénář dokončen |
| Konečný ARMED dynamic, 10 Hz, 35 s | 350/350 | 28 scalar comparison metrik max_abs=RMSE=0, bez partial/drops |
| 20 Hz, přechod STABILIZE→LAND | 319/319 | nulové chyby, stop MODE |
| Radio failsafe | 170/170 | nulové chyby, stop FAILSAFE, native failsafe dokončen |
| ARM→DISARM→ARM | 31/31 | stop DISARM, žádná druhá session |
| Backend disabled | žádný BIN | native scénář a frozen comparator dokončeny, bez předstírání persistence |
| Buffer FILE 1 KiB, speedup8, 20 Hz | 0 kompletních sad | 700 queued, 0 accepted, 700 rejected, bez queue drops; native dokončen, BIN správně NEKVALIFIKOVÁN |
| Compile OFF | 0 | parametr/metadatové zprávy nepřítomny, native scénář dokončen |
| Původní RAM FlightV1, DISARMED | 4000 pokusů | původní parser/export PASS, logger drops0 |

Konečný opt-in binary má SHA v `sitl-optin-final/status.json`; předchozí scénáře mají své vlastní SHA a logy. Po posledním zpřesnění opt-in byly opakovány FlightBin/Chain unit a dynamic end-to-end scénář; matematika ani stop-condition větve se neměnily. Finální dynamic: 14000 main pokusů,13999 paired,1 warmup,12990 nezávislých I kroků,29006 motor audits,0 changes,1 počáteční Attitude a PID synchronizace. Opravdové externí I/reset události zůstávají v chain.txt. Nenulové FF i D_FF a vlastní I jsou v CSV/grafech.

Uložená frekvence finálního 10Hz běhu 9.999942407 Hz, mean period100000.576 µs, min99960/max100793 µs. Je to perioda decimovaných záznamů, nikoliv změřený Main execution. Původní AP PM v ON i OFF uvádí LR400, přibližně4000 smyček/10s, NLon typicky4–5 a MaxT3332µs. Tyto události jsou i v baseline; netvrdíme PM.NLon=0. PM.Load a interní math/capture časy v tomto SITL vycházejí0 kvůli virtuálnímu času, proto nejsou CPU/H743 měřením.

Procesní CPU celé trajektorie: runtime OFF 9.11 CPU s/144 sim s, předchozí finální dynamic8.85 CPU s/143.2 sim s. Vzorky zahrnují přistání, odlišné přesné časy RC a souběžné host úlohy. **Z rozdílu nelze odvodit izolovanou režii writeru ani zápornou cenu loggeru.** Nový HW worst-case CPU/latency se teprve měří.

![AP vs UTB PID – skutečný SITL BIN](../tmp/utb-flight-log1-20261026/flight-bin-pid.png)
![Gyro a decimace – SITL](../tmp/utb-flight-log1-20261026/flight-bin-timing.png)

První neúspěšný dynamic běh neměl worker při UTB_ENABLE=0; opraveno startup zapojením a znovu ověřeno. První pressure runner chybně očekával zachování final UFLS i při přetížení; test opraven, ztráta zůstává explicitním výsledkem, nikoliv PASS persistence. Původní neúspěšné evidence jsou zachovány.

## 21. Offline buildy H743/SITL a artefakty

Všechny čtyři konfigurace SITL/H743, BIN OFF/ON prošly. Oba H743 APJ mají board1075. H743 active Core/Output/Acro/Angle symboly nejsou přítomné.

| H743 sekce | OFF | ON | Rozdíl |
|---|---:|---:|---:|
| .text | 1279576 | 1283448 | +3872 B |
| .data | 4172 | 4172 | 0 |
| .bss | 174248 | 175536 | +1288 B |
| startup+constructors+exidx | 840 | 840 | 0 |
| Waf flash used včetně data/constructors | 1284588 | 1288460 | +3872 B |
| Linker heap | 75016 | 73728 | −1288 B |

**Konečný ON APJ:** `tmp/utb-flight-log1-20261026/firmware/final/SkystarsH7HD-bdshot_UTB_FLIGHT_BIN_ON_11a518e5_efc95cdf5819.apj`, 1162543 B. SHA-256 `efc95cdf581920d675af3c39baffa78ce227caf815a48097cabce61b9192b7ff`.

**Konečný OFF APJ:** `tmp/utb-flight-log1-20261026/firmware/final/SkystarsH7HD-bdshot_UTB_FLIGHT_BIN_OFF_11a518e5_b2399853ffa6.apj`, 1158367 B. SHA-256 `b2399853ffa60eb0a17187c4e8bb1cc80752558176dd1b7a5ab4571fec7af623`.

Třetí testovatelná varianta = totožný ON APJ plus oddělený `firmware/final/UTB_FLIGHT_BIN_optin_10Hz_60s.parm`; žádný firmware se zapnutým runtime defaultem nebyl vytvořen. Rodičovský firmware adresář obsahuje také starší před-opt-in artefakty, označené jako překonané; pro následující rozhodnutí používat jen `firmware/final/` a plný hash.

Přesné provedené příkazy i exit kódy jsou v `build-matrix-guard.json`; zdroje v `source-manifest.json`, layout/stack/sekce v sousedních JSON/TXT. APJ byl dekódován a ověřen board ID i délka komprimovaného image. Build flags jsou stejné kromě požadovaného BIN přepínače; `UTB_RATE_SITL` je vypnutý.

Příklad reprodukce (WSL; neinstaluje závislosti):

```bash
cd /home/jirka/ardupilot_utb
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board SkystarsH7HD-bdshot \
  --out=tmp/utb-flight-bin-rebuild/H743_ON \
  --enable-UTB --disable-UTB_NATIVE_BENCH --disable-UTB_LOGGER_TRACE \
  --disable-UTB_RATE_SITL --enable-UTB_NATIVE_SHADOW --enable-UTB_ATTITUDE_SHADOW \
  --enable-UTB_CHAIN_SHADOW --enable-UTB_FLIGHT_SHADOW --enable-UTB_TIMING \
  --enable-UTB_TIMING_EXEC --enable-UTB_FLIGHT_BIN
./waf copter -j6
```

OFF mění poslední flag na `--disable-UTB_FLIGHT_BIN`; SITL mění board/out. Nový out v příkladu chrání původní měřicí evidence.

## 22. Native motor isolation

SHA checkpoint potvrzuje nezměněné UTB_PID/Rate/Attitude a původní AP_Motors/AC_AttitudeControl_Multi. Nový transport nepřebírá žádný AP_Motors ani writable native PID handle. Jeho hot-path call graph nemá AP_Logger, text, heap či semaphore; tyto symboly v objektu patří výhradně startup/consumer/frozen exportu, nejsou důkazem hot-path volání. Zachované původní native motor setters jsou jediné aktuátory.

Konečný dynamic má 29006 skutečných before/after motor kontrol s0 změnami. `native-motors.csv` a `raw-motor-comparison.json` obsahují OFF/ON raw RCOU: DISARMED a prvních200ms ARMED idle stejné [1000,1000,1000,1000]. Dynamické host RC události jednotlivých běhů nejsou sample-exact replay, proto se z jejich rozdílných trajektorií netvrdí bitová shoda celého letu. Přesný důkaz nezměněného commandu poskytuje in-cycle audit v témže běhu; nulový časový dopad tím prokázán není.

## 23. Známá omezení

- Nový APJ na fyzické H743 nebyl spuštěn; žádný nový HW timing, watermark, throughput či flash-write údaj není PASS.
- SPSC izoluje producenta od vlastního consumer čekání, nikoliv native AP od sdíleného write_sem/SPI či sector erase.
- Chybějící poslední UFLS, partial writes a zaplněný backend mohou znemožnit uzavřenou offline kvalifikaci; parser to nezakrývá.
- FMT/counters/decimace popisují 10/20Hz data, ne plný 400Hz trace všech výstupů. Comparator přesto kontroluje každý podporovaný krok v RAM.
- ARM-only lifecycle neumožňuje produkční Flight BIN spustit samotným DISARMED PARAM_SET. Není zaveden skrytý force/start bypass.
- Pokud se worker vůbec nevytvoří, nemůže zapsat ani svůj stav WORKER; diagnostiku nelze kvalifikovat podle chybějícího logu. Ve skutečném bootu ověřit i přítomnost utb_log v threads.
- Jeden boot = jedna sdílená diagnostická session; konflikt s jinou session diagnostiku odmítne.
- FILE stress neprokazuje fyzické chování W25Q128. Ani příčina starých5.5–6.5ms událostí není tímto opravena: dřívější TRACE zamrzla už při LOG_REQUEST_LIST, bez zachyceného StartNewLog. Korelace se startem loggeru není důkaz konkrétního místa.
- H743 dynamická heap/stack cena workeru se musí ověřit ve skutečném bootu; linker heap není aktuální volná kapacita.

## 24. Nejmenší další HW kvalifikace (zde neprovedena)

1. Samostatně schválit pouze kandidátní ON APJ s výše uvedeným plným hashem; zkontrolovat USB serial200024001251303433333336/board1075, DISARMED, původní konfiguraci a zdraví.
2. Kompletní param backup, archivace všech dosavadních BIN s hashi, aktuální log inventory a volná kapacita. Při nedostatku místa zastavit, nemazat bez souhlasu.
3. Jeden schválený standardní upload, bez bootloader rewrite či změn gains/failsafe. Ověřit nový boot, native motor autoritu, všechny RUN0, UCS_B_ENABLE0 a konfiguraci400Hz/FSTRATE0/SMAX/notch.
4. USB-only DISARMED baseline: health/PM, memory/threads a stack; metadata a worker ready. Neměřit současně se stahováním/log listingem. Na první logger open vyhradit samostatné pozorované DISARMED okno; nedělat je za letu.
5. Pro úplný sedmipaketový Flight BIN test je nutné **další samostatné rozhodnutí o triggeru**: tento firmware jej spouští až na normální ARM hraně. Pouhé LOG_DISARMED=1 nevyrobí UFPI. Je-li výslovně schválen krátký USB-only ARM bez LiPo/vrtulí a projdou všechny původní prearm kontroly, lze jednou použít SECS5/RATE10 s pevně uloženou FC. Pokud ARM bez baterie neprojde, zastavit; žádný force ARM ani vypínání kontrol. Alternativou pro přísně DISARMED kvalifikaci je teprve samostatně schválený minimální diagnostický trigger využívající tentýž konečný engine; **není součástí tohoto firmware ani provedeného testu**.
6. Při schváleném skutečném zápisovém okně žádný download/MAVFTP/listing; porovnat baseline/běh/post PM.NLon/MaxT, periodu, CPU, mutex timing dostupnou stávající sondou, stack a DSF. Stop při InternalError, health problému, neočekávané motor změně, neshodě, DESYNC, nových neobjasněných dlouhých cyklech nebo nestabilním USB. Číselné limity odvodit ze stejného baseline a dříve schválené timing kvalifikace, nikoliv libovolně rozšířit.
7. Po ukončení a DISARMED obnovit diagnostické parametry, stáhnout jeden BIN, spustit parser a porovnat skutečné kompletní sady s backend/RAM čítači. Stop bez automatického retry. Při neúplném statusu není persistence kvalifikovaná.
8. Obnovit UCS_B_ENABLE0, ostatní RUN0, původní telemetrii/logovací parametry a ověřit závěrečné zdraví. Teprve výsledky dovolí samostatné rozhodnutí o letu.

## 25. Praktický postup a FLIGHT-1/2

Po samostatné HW kvalifikaci a schválení letu, ve skutečném DISARMED STABILIZE, při splněném původním preflightu:

```text
UCS_B_RATE = 10
UCS_B_SECS = 60
UCS_B_ENABLE = 1
UCS_RUN = UCS_F_RUN = UAS_RUN = UNS_RUN = UTT_RUN = 0
```

Počkej na pozorování opt-in v DISARMED. Historické UTB_ENABLE ani UTB_SHADOW nezapínej kvůli tomuto writeru. Původní AP logger musí být zdravý a nastavený pro normální ARM logování. Skutečný ARM spustí jedinou session bez USB/GCS příkazu. DISARM, mode/failsafe nebo limit ji ukončí. Při dalším bootu uložený ENABLE1 opět představuje opt-in; po experimentu jej výslovně vrať na0.

Po letu stáhni celý původní BIN standardním postupem; nové stažení nikdy nemíchej s měřeným oknem. Ve WSL:

```bash
.venv/bin/python Tools/utb/tools/utb_flight_bin_analysis.py /cesta/k/letu.BIN \
  --output tmp/analyza-konkretniho-letu
```

Vznikne summary.json a samples.csv. Neuznávej PASS jen podle překrytých grafů: musí sedět finální čítače, všechny sady i source identity, bez ztrát/invalid/unsupported a mismatch. Kód bez USB nevyžaduje telemetrii, původní AP GCS failsafe konfigurace se tím ale neobchází.

FLIGHT-1 zůstává krátký native-only let, případně s předem kvalifikovaným pasivním loggerem. Pokud BLOCK/stack/timing kvalifikace není hotová, logger při něm zůstane OFF. FLIGHT-2 = native AP + kvalifikovaný pasivní BIN; následně vyhodnotit I/FF, saturaci, resets, source alignment a timing. Aktivní UTB motorové řízení tímto schváleno není.

## 26. Git, závěr a odpovědi

HEAD/větev zachovány. Index nebyl měněn, žádný commit/push/reset/clean/stash/rebase. `checkpoint-verification.json` kontroluje všech7692 původních souborů:0 chybějících,9 očekávaně změněných. Kompletní aktuální status je `git-after.txt`; zahrnuje i starší rozpracované soubory. Zdrojový manifest nezahrnuje dokumentaci a evidence, které nejsou firmware vstupem. Dočasné build environment chyby (PATH překladače/Pythonu) byly odstraněny použitím existujících instalací; nic nebylo doinstalováno.

| Otázka | Odpověď |
|---|---|
|1 Implementováno? | Ano, skutečný queue/worker/BIN/parser, nikoliv pouze návrh. |
|2 Zapnutí? | Compile UTB_FLIGHT_BIN + runtime UCS_B_ENABLE1, před ARM; RATE10/20 a SECS≤120. |
|3 Bez USB/MP? | Kód ano, ARM-edge lifecycle otestován SITL; fyzický samostatný let neověřen. |
|4 Start? | Pozorovaný DISARMED opt-in a následná ARM hrana v native STABILIZE. |
|5 Stop? | COUNT/time, DISARM, mode, původní failsafe, konfigurace, diagnostický fault. |
|6 Skutečné hodnoty? | Ano, existující native/nezávislé UTB snapshots, včetně FF/D_FF a quaternionů. |
|7 Synchronizace? | TimeUS+Seq+source/consume loop a timestamp; nezávislá historie pokračuje mezi kroky. |
|8 Zprávy? | UFPI59, UFRS67, UFAT72, UFLS55 B. |
|9 Frekvence? | 10/20 Hz z400Hz; uložených10Hz finálně9.999942407 Hz. |
|10 Tok? | 4555/9055 B/s včetně přibližně1Hz statusu, plus metadata/page overhead/native logy. |
|11 RAM? | +1288 B staticky; startup worker navíc podle existence původního workeru, viz část17. |
|12 Unit? | 125 relevantních C++,48 Python PASS; další legacy3 PASS/1 SKIP není vydáváno za plnou regrese všech konfigurací. |
|13 ARMED SITL? | Dynamic350, mode319, failsafe170, rearm31 uložených kompletních sad; baseline OFF bez payloadů. |
|14 Numerická shoda? | V podporovaných zaznamenaných sadách max/RMSE0; celou 400Hz matematiku navíc kontroluje dosavadní RAM comparator. |
|15 Motory? | Konečný live audit29006/0 changes, source/symbol guardy a omezený raw OFF/ON audit. |
|16 Přetížení? | Drop celé nové sady při queue full; backend odmítnutí bez retry; stress700 rejected, native pokračoval. |
|17 Main/Rate blokování? | Nový producer nečeká. Sdílený původní AP logger může Main zpozdit nepřímo; nulový dopad netvrdíme. |
|18 BLOCK riziko? | write_sem/sem/dev_sem, StartNewLog, IO/SPI write/erase/WaitReady; skutečný worst-case neověřen. |
|19 Buildy? | H743/SITL BIN OFF i ON PASS, logging-disabled targeted compile guard PASS. |
|20 APJ/SHA? | Konečné dva kandidáty a plné hashe uvádí část21 a firmware/final/artifacts.json. |
|21 HW postup? | Část24; nutné zvlášť schválit upload a jasně zvolený krátký write trigger. |
|22 Co brání letu s BIN? | Fyzická BLOCK write/timing/stack/health kvalifikace tohoto konkrétního APJ a samostatné rozhodnutí o letu. |

**Implementace je připravena k plánování samostatně schválené HW kvalifikace. Není označena za kvalifikovanou pro let.**
