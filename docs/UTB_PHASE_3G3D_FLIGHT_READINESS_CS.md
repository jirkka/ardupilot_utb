# FÁZE 3G-3D – Připravenost pasivního UTB Angle

25. 10. 2026, offline a SITL. **GO k samostatně schválené USB-only kvalifikaci 3G-3E. NO-GO k fyzickému letu.** V této fázi nebyl otevřen COM5, měněna FC, nahrán firmware, proveden fyzický ARM, motorový test ani zápis/mazání DataFlash.

## Závěr

Doporučená cílová varianta pro první omezený pasivní let je **B: současný RAM recorder a standardní AP log**. Nová persistentní UTB infrastruktura zatím není nutná. Doporučení je podmíněné: nejsou uzavřené logger prodlevy, ochrana starých logů před wrapem, Main timing ani bezpečné získání RAM po přistání bez restartu. Samotné „exportovat po letu“ tyto překážky neřeší.

Původní H743 výsledky 3G-3C se nemění: 4000/3999 kroků, 8000 Attitude operací, 84 nulových exportovaných max/RMSE, motor_changes=0, CPU 18,8267 → 28,7278 → 18,7412 %, měřené brackety 190,11525 µs/main, Rate perioda 2453–2550 µs, jitter 1,69685 µs. Nejde o Main WCET. Nové výsledky níže jsou **SITL**, nikoliv další HW měření.

Artefakty: `/home/jirka/ardupilot_utb/tmp/utb-3g3d-readiness-20261025_134513/`. Produkční C++, PID/gains a motorová cesta zůstaly beze změny. Použit existující SITL_FINAL build z 3G-2 se shodným zdrojovým manifestem; přesný SHA je v checkpoint.json. Bez nové sestavy firmware a instalací. Host gcc sestavil pouze izolovanou linuxovou fault fixture `.so`, nikdy nepoužitelnou na H743.

## Časový call graph a mapa bracketů

Pořadí v [Copter.cpp](/home/jirka/ardupilot_utb/ArduCopter/Copter.cpp:113) a [AP_Scheduler.cpp](/home/jirka/ardupilot_utb/libraries/AP_Scheduler/AP_Scheduler.cpp:357):

```text
INS wait → scheduler rsem → sample timestamp / UTT.loop
→ INS update → Chain prepare → native Rate capture / AP PID
→ UTB Rate consume / compare / motor audit
→ native motors_output_main → legacy shadow OFF
→ AHRS/EKF / inertia / reset handling
→ native STABILIZE / Attitude capture / UTB Attitude + reset replay
→ Chain finish / Flight record / případný freeze a publish
→ UTT.finish → zbývající fast a scheduled tasks včetně AP loggeru
→ konec scheduler smyčky → další INS wait
```

Rate(N) spotřebuje Attitude target z N−1. **Pasivní Rate consume je před motor outputem**: nezměněné motorové hodnoty samy neprokazují nezměněnou latenci Rate → aktuátor.

| Úsek a volající | Kontext | Blokování, heap, mutex, DataFlash | Měření / nesting |
|---|---|---|---|
| INS wait, scheduler rsem, sample | Main 180 / boost 182 | původní čekání a rsem | UTT timestamp je až po wait/take; jejich trvání neměří |
| INS update / gyro | Main fast task | původní senzory a driver synchronizace | samostatná execution délka chybí |
| Chain prepare/config/context | run_rate_controller_main | bounded kopie a guardy, bez nových zámků/heapu/DF | část mimo měřený součet |
| Rate pre/post capture, AP PID, audit | AC_AttitudeControl_Multi | UTB kopie a porovnání bez heapu/zámků; native AP setters zachované | native_capture zahrnuje AP; úplný outer capture/audit není pokrytý |
| UTB Rate/PID a event replay | complete_rate | vlastní RAM historie; bez IO/heapu/čekání | rate_sum obsahuje vnořený pid_consume/math |
| motors_output_main | Main fast task | původní AP_Motors/HAL, spool, mixer, DMA synchronizace | není samostatný output bracket |
| AHRS/EKF/inertia/reset | Main fast task | původní AHRS rsem a senzory; možná preempce/čekání | uvnitř Rate → Attitude delay bez atribuce jednotlivé funkce |
| Native STABILIZE pre/post capture | update_flight_mode | AP reference/manual throttle, UTB hodnotové kopie | pre/post oddělené; AP výpočet není UTB režie |
| UTB Attitude/replay/compare | after_attitude | vlastní RAM, bez motorového sinku/logů/heapu | att_consume obsahuje sync/math/compare |
| Chain end, Flight record, freeze | mode.cpp po flightmode->run | pevný ring, RAM param set, release atomic publish; bez param save/DF | record_timing oddělený; end/guard/publish není plně pokrytý |
| UTT hooks | Main/Rate/Attitude/finish | pevná RAM; žádný IO/heap/nový mutex | hook_sum/max nezahrnuje všechny guardy, clock/accounting instrukce |
| Native log tasks, GCS, další úlohy | scheduler Main po finish | logger může synchronně startovat log, čekat na sem/SPI; startup metadata mohou alokovat | UTT.finish není konec scheduleru |
| Logger IO | log_io, priorita 59 | buffer → SPI, chip sem/dev_sem, WaitReady | samostatné vlákno, ale Main může čekat na sdílený backend zámek |
| Frozen MAVFTP export | FTP 58 + GCS | ExpandingString/new, FS a worker synchronizace | mimo měřicí okno, pouze DISARMED; zahrnout stack po exportu |

Zdrojové body: [Attitude.cpp](/home/jirka/ardupilot_utb/ArduCopter/Attitude.cpp:17), [Rate capture](/home/jirka/ardupilot_utb/libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:505), [Attitude capture](/home/jirka/ardupilot_utb/libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:421), [finish](/home/jirka/ardupilot_utb/ArduCopter/mode.cpp:536), [Chain](/home/jirka/ardupilot_utb/libraries/AP_UTB/AP_UTB_ChainShadow.cpp:300), [Flight recorder](/home/jirka/ardupilot_utb/libraries/AP_UTB/AP_UTB_FlightShadow.cpp:102), [AHRS rsem](/home/jirka/ardupilot_utb/libraries/AP_AHRS/AP_AHRS.cpp:373), [FTP worker](/home/jirka/ardupilot_utb/libraries/GCS_MAVLink/GCS_FTP.cpp:49). Tabulka není důkaz auditu každé transitive HAL funkce.

190,11525 µs/main ze 3G-3C je pre + post_without_consume + att_consume + rate_process + record. Chybí úplný outer capture/transport/guards/audit, engine.end, publish a ostatní AP smyčka. Vnořené sync/math/compare se znovu nepřičítají. Native Rate+capture zahrnuje AP. Maxima různých funkcí nelze sečíst na garantovaný nejhorší cyklus.

## Timing, Main deadline a WCET

[AP_UTB_Timing.cpp](/home/jirka/ardupilot_utb/libraries/AP_UTB/AP_UTB_Timing.cpp:222): RUN default 0, COUNT 1–20000, start jen DISARMED a jedna finite session za boot. ARM ji ukončí reason=2. COUNT je počet Main volání, **nikoliv nezávislý wall-time timeout**; při zastavení toku smyčky nelze slíbit ukončení po N/400 sekundách.

Pět metrik: Main start-to-start, Rate entry perioda, první primary Attitude entry perioda, Rate→Attitude delay a změna tohoto delay. Main timestamp je po INS wait a scheduler rsem. **Main execution duration se neměří**: chybí end timestamp po všech scheduler úlohách. Timing.finish po flightmode->run pouze dokončuje diagnostický záznam.

Exportuje min/max/sum/absolute_error/count, počty >2500 a ≥5000 µs a ID/čas min/max. Skupiny ALL/UAS OFF/UAS1/other se řídí **UAS_RUN, nikoliv UCS_F_RUN**; nelze z nich vyrobit per-Flight statistiky. Uchová nejvýše 17 raw řádků kolem největší absolutní Attitude odchylky, nikoliv nutně okolí maximální Main periody. Úplný histogram, individuální periody, P95/P99 dostupné nejsou.

Over_nominal není hard miss; 2501 µs může být běžný jitter. Dvouperiodový interval je významný pozorovaný příznak, ne automatická atribuce viníka. cycle_gaps kontroluje lokální diagnostická ID, nikoliv všechny fyzické IMU vzorky. Hard deadline vyžaduje definovaný okamžik dostupnosti vzorku, přípustný konec outputu a end/output timestampy včetně čekání/preempce. Pozorované maximum není WCET.

[Scheduler task overrun](/home/jirka/ardupilot_utb/libraries/AP_Scheduler/AP_Scheduler.cpp:284) znamená time_taken > max_time_micros. [PM.NLon](/home/jirka/ardupilot_utb/libraries/AP_Scheduler/PerfInfo.cpp:113) používá periodu >1,2×2500=3000 µs (:230). Scheduler load je budget/spare ukazatel, ne samostatný důkaz hard deadline.

Historický H743 3E-4: Timing hooky 6,272 µs/cyklus, max 67 µs/hook; Main 2449–2550, Rate 2448–2553, Attitude 1630–3349 µs. Je to jiný experiment, nikoliv nová režie tohoto firmware. Attitude jitter byl lokalizovaný do variability Rate→Attitude delay, konkrétní funkce nebyla určena.

Nový SITL souběh Flight+Timing prošel 4000 timing cyklů bez missing/duplicate/unsupported a s úplným Flight exportem. Není to důkaz nulového observer effectu na H743. UTT max 20000 nepokryje celé Flight maximum 48000; současný UTT není určen pro ARMED let.

**Minimální doplnění – pouze návrh:** default-OFF RAM bracket sample → konec AP_Scheduler::loop a timestamp dokončení native motor outputu, sum/max/count/ID, frozen export s jedním main writerem. Pro atribuci stage ID před/po loggeru, AHRS a outputu. Bez heapu, textu, DF nebo nového zámku v capture. OFF/ON ověřit observer overhead. WCET dále vyžaduje bounded ISR/preempci/driver waits. Patch ani nový firmware zde nevznikl; jeho implementace/upload vyžadují samostatné rozhodnutí.

## Historické prodlevy a BLOCK logger

Původní důkazy zůstávají: PM maxima 5503/6480 µs, později 5993 a 6070 µs. PM měří Main sample start-to-start přes [scheduler](/home/jirka/ardupilot_utb/libraries/AP_Scheduler/AP_Scheduler.cpp:442), nikoliv samotný logger span. Okna se překrývala s prvním povolením loggeru; přesný okamžik a vnitřní operace chybí. **First-open příčina není prokázána ani opravena.**

TRACE ON v 3D2 zamrzl už při log inventory: perioda 24 238 µs, GCS::update_receive 23 938 µs. Zachytil synchronní listing/hledání/čtení flash v Main; start_new_log count=0. Pozdější 6070µs first-open již nezachytil. Jde o prokázanou blokující cestu DISARMED listingu, ne vysvětlení původních špiček. AP log transfer za armed odmítá. Podrobné původní důkazy: [3D2](/home/jirka/ardupilot_utb/docs/UTB_PHASE_3D2_LOGGER_CS.md), [TRACE](/home/jirka/ardupilot_utb/docs/UTB_LOGGER_TRACE_CS.md).

Konkrétní synchronní cesta: Main log task → WriteBlock → [Backend::WritePrioritisedBlock](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Backend.cpp:442) → StartNewLogOK (jen main) → [Block::start_new_log](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Block.cpp:549) → chip sem → find_last_page/StartRead → [PageToBuffer](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp:229) → WaitReady/dev_sem/SPI.

Také [AP_Arming::arm_checks](/home/jirka/ardupilot_utb/libraries/AP_Arming/AP_Arming.cpp:1795) volá PrepForArming → [PrepForArming_start_logging](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Backend.h:203) → start_new_log před dokončením armování. První open není jen IO operace. Připravení logu v DISARMED může přesunout konkrétní first-open okamžik, ale nekvalifikuje další erase/zápisy.

Uložené HW LOG_BACKEND_TYPE=4. [Hwdef](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD/hwdef.dat:58) používá W25Q128/dataflash na SPI3, BMI270 na SPI1 a SPI4. Logger a gyro tedy nesdílejí stejný SPI bus mutex, sdílejí však CPU, IRQ/DMA/arbitráž. Nulová interference z toho neplyne.

Block má chip sem, write_sem a JEDEC dev_sem. Producer zapisuje do writebuf pod write_sem; nedostatek bufferu způsobí false/drop. [log_io](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger.cpp:1455), priorita 59, zapisuje stránky pod chip sem; stejný zámek používá Main start/listing/čtení. [ChibiOS Semaphore](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/Semaphores.cpp:46) používá chMtxLock a priority inheritance (modules/ChibiOS/os/rt/src/chmtx.c:55). PI neodstraní fyzický SPI/busy delay.

[WaitReady](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp:93) opakuje Busy s delay 100 µs až do timeoutu >5000 ms; wall span zahrnuje preempci a neizoluje flash busy. [SPI transfer](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/SPIDevice.cpp:204) čeká na DMA s timeoutem 20000+32×len µs. Jsou to chybové timeouty, nikoliv prokázaná běžná 5ms příčina nebo přijatelný letový limit.

**Plná flash:** [bufferspace_available](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Block.cpp:77) vrací celkovou kapacitu, protože backend je kruhový. FinishWrite obtáčí adresu na 1 a před dalším blokem volá SectorErase (:90–116). Staré logy se mohou přepsat. chip_full chrání především vlastní hlavičky velmi dlouhého aktuálního logu, ne všechny historické logy. IO recovery může mazat více sektorů pod sem (:876).

W25Q128: teoreticky 16 MiB, stránka 256 B/header 6 B, rezervovaný blok 64 KiB, log payload 65280×250=16 320 000 B před file headers/padding. Inventář 11 885 734 B je payload velikost podle [get_log_info](/home/jirka/ardupilot_utb/libraries/AP_Logger/AP_Logger_Block.cpp:782), nikoliv aktuální bezpečně použitelná souvislá oblast. Prosté odečtení od 16 MiB není kapacitní důkaz. Žádná flash nyní nebyla čtena či mazána.

Také Block::Init (:68–74) může při NeedErase zavolat původní EraseAll kvůli formátu úložiště. Zákaz explicitního erase příkazu proto sám není úplnou ochranou archivů při změně firmware/formátu. Současná fáze tento mechanismus nespouštěla.

Závěr: logger může blokovat Main; first-open i průběžné erase/lock riziko zůstává. **BLOCK není touto fází kvalifikován pro další fyzický let se zachováním starých logů.** Nutná samostatná sonda přesně během prvního startu a následná sector-boundary/recovery kvalifikace. Před cold startem provést zálohu/inventory, po něm žádný listing/FTP/tasks před first-open. Historický TRACE má předčasný freeze; cílená aktivace a IO owner attribution by byly samostatně schvalované změny. Původní logy nepoužít jako spotřební fixture a nenavrhovat automatický formát.

## Strategie záznamu a nároky

| Varianta | Co lze / nelze ověřit | RAM a tok | Ztráta dat / riziko AP | FW a další HW důkazy |
|---|---|---|---|---|
| A: současná RAM | 84 agregací na 400 Hz a 128 decimovaných projekcí; ne úplný 400Hz průběh ani persistenci | recorder 38912 B + existující engine; nulový persistentní UTB tok | restart zničí neexportovaná data, ring overwrite explicitní; UTB Rate je před motor outputem | bez změny FW; nutný same-boot export, dynamická režie a stack |
| B: RAM + AP log | A plus native ATT/ANG/RATE/PID/RC/PM/DSF kontext; nemusí jít o stejný sample | stejná UTB RAM; AP tok podle masky/frekvencí | RAM stejné riziko, AP logger může blokovat/dropnout/wrapovat | bez nové UTB infrastruktury; kvalifikovat BLOCK, kapacitu, napájení, přenos a timestamp mapping |
| C: omezený persistentní UTB přes AP logger | vybrané projekce přežijí reboot, pokud byly flush; ne automaticky úplná sada | návrh 315 B/sada; 10 Hz=3150 B/s, 20 Hz=6300, 50 Hz=15750, 400 Hz=126000 | fronta/drop/částečné sady/FMT/flush; původní mutex a wrap riziko zůstává | nový opt-in FW, metadata, bounded fronta/worker a HW kvalifikace; pouze návrh |

C respektuje nejvýše 16 FMT položek: 3 PID pakety po 65 B (header3 + TimeUS8 + Seq4 + Axis1 + Flags1 + 12 floatů48), rate/gyro/dt paket 57 B a quaternion paket 63 B. Dohromady 315 B v pěti paketech/třech typech. Názvy/ID nejsou rezervované; před implementací prověřit volné ID/FMT/FMTU a délky labels. Návrhová fronta 8×320 B=2560 B + indexy/statistiky (rozpočet 128 B), worker stack asi 4096 B + OS workspace jsou **odhady**, ne linker měření. Page overhead 256/250≈1,024, navíc metadata/padding/native log.

Plná fronta musí dropnout celou sadu s counterem, nikdy nečekat v Main. Worker stále může nepřímo soutěžit s native loggerem. Přímé WriteBlock ve 400Hz cestě není schválené; nevytvářet druhý FS/SPI driver. Varianta C nebyla implementována ani zapnuta na HW.

A/B: pro žádný overwrite musí platit ceil(COUNT/DECIM)≤128. 24000/200 dá 120 řádků při 2 Hz, 48000/400 dá 120 při 1 Hz. 48000/200 znamená 240 selected a 112 overwritten. Agregace stále pokrývají všechny podporované pokusy. COUNT=48000 je nominálně 120 s, ne nezávislý timeout.

**Doporučení B je podmíněné.** A samo neodstraní AP logger riziko, pokud native armed logging zůstane zapnutý. Jeho vypnutí by vyžadovalo samostatné rozhodnutí a ztratilo kontext. C zatím není nutná. Pokud nelze zachovat a exportovat RAM, A/B zůstávají blockerem do ověření komunikačního/napájecího postupu nebo samostatně navržené persistence.

## Zachování RAM po přistání

[dump_flight](/home/jirka/ardupilot_utb/libraries/AP_UTB/AP_UTB_ChainShadow.cpp:351): před publikací pending; po freeze export jen DISARMED. COUNT stop=1, ruční=2, bad=3, conflict=4, unsupported=6, mode exit=7. STABILIZE→LAND/RTL zmrazí před další nepodporovanou operací, native řízení pokračuje. DISARM v nezměněném STABILIZE sám recorder nemaže; změní epoch/reset kontext a session může běžet do COUNT. Návrat do STABILIZE ani další RUN neotevře druhou session ve stejném bootu. Ztráta GCS sama RAM nemaže; native failsafe může změnou módu vyvolat freeze. Reboot data ztratí.

Uložená fyzická konfigurace má MAVLink pouze SERIAL0 USB; SERIAL1_PROTOCOL=23 je RC a SERIAL4_PROTOCOL=5 GPS. CRSF/ELRS RC není doložená náhrada obousměrného MAVFTP kanálu. Bezdrátový exportní kanál není ověřený. Bezpečnost současného USB/LiPo napájení nelze doložit jen hwdefem.

**Podmíněný, dosud HW neověřený postup:** nejprve samostatný bench bez vrtulí dle elektrického zapojení desky ověří USB+LiPo kompatibilitu a přechod zdrojů bez brownoutu/resetu/zpětného napájení. Doložit boot counter, uptime a session identitu. Teprve při úspěchu lze plánovat: po přistání skutečně DISARM, zabránit opětovnému armování, zachovat napájení, bezpečně připojit USB k DISARMED craftu, ověřit stejný boot, sekvenčně exportovat/validovat/hashovat devět souborů a zálohovat native BIN. Napájení odpojit až po ověření úplnosti.

**Nepřipojovat USB k ozbrojenému dronu a neodpojovat LiPo před záchranou RAM.** LiPo/power-handoff bench zde není povolen ani proveden; USB-only 3G-3E jej sám neověří. Alternativou je předem ověřený MAVLink/MAVFTP telemetry kanál s dosahem a propustností. Dnes není doložen. Same-boot získání RAM proto zůstává **blockerem fyzického letu**. Také zahájení finite okna před vzletem vyžaduje plán: dlouhé prearm čekání může vyčerpat COUNT; vzdálený trigger nelze předpokládat bez kanálu.

## Testy a nové SITL důkazy

**154 C++ PASS** v devíti existujících binárkách (47+4+30+20+8+3+5+23+14). **187 Python PASS, 5 skip, 2 subtests PASS**; pytest skutečně spustil funkční Flight testy, samotný unittest discover by je nepokryl. Pět skip potřebuje UTB_AUDIT_EVIDENCE k historickému download incidentu, nejsou PASS. Logy/XML/unit-results zachované v adresáři fáze.

Standardní native STABILIZE let: obě znaménka Roll/Pitch, yaw, combined, neutral/self-level, LAND/DISARM a export ve stejném bootu. FF scénář explicitně ověřil nenulové vlastní FF/D_FF a shaping. Rate/Attitude/Chain live regrese PASS. Aktivní 3G-1 native/UTB ANGLE a ACRO regrese **4/4 PASS** jen v izolovaných SITL instancích, nikdy na H743.

RC/GCS failsafe prokázaly původní AP reason3/5 a RTL/LAND, diagnostický stop7 a explicitní návrat pilota bez retriggeru. Navíc přímé STABILIZE→RTL a LAND PASS. Varianta s FS_GCS_ENABLE=0 a přerušeným GCS heartbeat nezměnila mód: je to důkaz vypnutého mechanismu, nikoliv fyzického failsafe PASS.

Lifecycle PASS: pending export odmítnut analyzátorem, opětovný RUN nezmění frozen session. Úmyslně zahozeno pět FTP odpovědí → přenos selhal a frozen metadata zůstala dostupná. Missing page/mixed-session/truncated page odmítnuty. Skutečný exec reboot stejného PID s persistovanými parametry zvýšil boot1→2, vrátil pending a staré stránky nebyly přijaty jako nová platná session. Samostatný armed-export test odmítl frozen RAM při ARM a po LAND/DISARM správně exportoval. Není to fyzický USB handoff důkaz.

Maximální 48000/400 běh: **48000 attempts, 47999 paired, 49477 valid Attitude, independent_i=46418, 120 raw řádků, overwrite0, stop1, všech84 max/RMSE0, motor_changes0**. Po konci diagnostiky native AP pokračoval k LAND/DISARM. Overflow 8000/1: přesně 7872 overwritten a 128 posledních řádků, bez ztráty numerických agregací.

### Odmítající a zpožděný logger

SITL-only LD_PRELOAD fixture zasahovala jen `.BIN` zápisy owned linuxového arducopter procesu po simulovaném ARM. ENOSPC **20 injekcí**, zpoždění 100 ms/write **56 injekcí**. Obě reálně asertované varianty PASS: native let/přistání/DISARM pokračoval, motor_changes0 a 84 metrik bez chyby, telemetrické dropped nenulové. Max viditelného 16bit dropped 655xx není celkový počet (může wrapovat). Neplnil se skutečný disk. **Jde o FILE backend, ne elektrickou/erase simulaci H743 BLOCK.**

Původní neúspěšné skripty nejsou označené PASS a jejich evidence nebyla přepsána:

- První ENOSPC let i numerika doběhly, ale DFReader selhal na očekávaném 0B BIN. Kopie v2 přeskočí prázdné soubory a ztráty ověřuje z telemetrie; původní coordinator měl také chybu None>0 při chybějícím DSF.
- Lifecycle v1 měl timeout pod MAVFTP idle_detection_time=3,7 s; v2 neobnovil stream po rebootu. Ve v3 helper ponechal `--wipe`, zatímco HAL_SITL z reboot argv vyjímá pouze `-w`. V4 lokální launcher používá `-w` a obnoví stream; skutečný boot1→2 doložen. Produkční kód neměněn.
- Tři aktivní souběžné instance selhaly na společném SERIAL1 portu 5762. Kopie s izolovanými auxiliary porty následně prošly 4/4 scénáře.
- První max běh měl krátké 65s host čekání na LAND: manual-throttle model dosáhl 275,438 m, při sestupu 1,49 m/s potřeboval delší návrat. Další běh s 240s observer timeoutem přistál, ale generic assertion chybně očekával overwrite0 u DECIM200 (správně112). Finální 48000/400 prošel se stejným AP/model/gains, speedup4. Žádný PID tuning ani uvolnění letových guardů.

Přesné čítače, návratové kódy, příkazy a původní chyby: `analysis/flight-counts.csv`, `sitl-results.json`, `active-v2-results.json`, `fixture-v2-results.json`, `lifecycle-v4/result.json`, `armed-export-result.json`, `max-v3-result.json`. Reprodukce používá pouze loopback/owned procesy.

## Zátěž a interpretace dat

Podrobná tabulka je v `analysis/load-comparison.csv`: host CPU pochází z `/proc/PID/stat`, wall čas z monotónních hodin. Native/Chain/Flight mají stejný virtuální interval 61,8 s, speedup2, host okna přibližně 30,8 s. Process CPU 3,61 / 3,73 / 3,56 s; logger-off 3,32 s. Pressure má jiné kratší okno a speedup4. Oddělené a částečně souběžné běhy nejsou statistickým kauzálním měřením ceny UTB.

V pěti celých PM oknech mezi ARM+10s a LAND měly native/Chain/Flight shodně NLon součet24, MaxT3332 µs a PM.Load0. Jsou to **virtuální SITL ukazatele**, ne H743 timing; nelze je automaticky připsat UTB. Logger-off nemá PM/DSF, null není0. Pressure1KiB neměl spolehlivý DSF export; skutečné drops dokládá zvlášť fixture přes telemetrii.

SITL hook brackety mohou vyjít0, protože virtuální čas během CPU výpočtu neběží. Nejsou skutečnou dobou výpočtu. Skutečné host časy/CPU, tlog velikosti, logger drops a dostupné PM jsou zachované; žádný SITL údaj se nevydává za H743 WCET.

Celé uložené tlogy obsahují native 32146, Chain 32245, Flight 33527, logger-off 33554 a pressure 22922 zpráv. Nejde o samotné měřicí okno: zahrnují startup, parametry, let, přistání a export. Rozdělení podle typu a BAD_DATA je v `analysis/message-counts.json` a CSV. Rozdíl počtů není sám důkaz drops; Flight export přidává legitimní FTP provoz.

Grafy skutečných dat: [CPU/wall](/home/jirka/ardupilot_utb/tmp/utb-3g3d-readiness-20261025_134513/analysis/sitl-load.png), [ring accounting](/home/jirka/ardupilot_utb/tmp/utb-3g3d-readiness-20261025_134513/analysis/ring-accounting.png), [native/UTB targety](/home/jirka/ardupilot_utb/tmp/utb-3g3d-readiness-20261025_134513/comparison-flight/analysis/body-targets.png), [PID outputy](/home/jirka/ardupilot_utb/tmp/utb-3g3d-readiness-20261025_134513/comparison-flight/analysis/pid-outputs.png), [FF/D_FF](/home/jirka/ardupilot_utb/tmp/utb-3g3d-readiness-20261025_134513/comparison-flight/analysis/feedforward.png), [timestampy](/home/jirka/ardupilot_utb/tmp/utb-3g3d-readiness-20261025_134513/comparison-flight/analysis/timing-records.png). Bez rekonstrukce chybějících 400Hz raw vzorků.

## Data pro bakalářskou práci

RAM Record má actual/AP/UTB target quaternion, Cmd0..2 po AP RC mapě, native/UTB rate target, gyro, P/I/D/FF/D_FF, regulační/FF output, dt/Seq/SourceUS/TimeUS/Mode/Spool/Limits/ResetFlags/epoch. U Rate řádku je Attitude kontext source(N−1), consume(N); ne všechny veličiny mají stejný instant. Roll/Pitch vypočítat z reálných quaternionů a uvést zdrojový čas. Raw RC nelze dodatečně vymyslet.

[Native logging](/home/jirka/ardupilot_utb/libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp:12): ANG používá loop startTimeUS, RATE gyro timestamp, PID svůj log timestamp. RAM rad/s a RATE deg/s převést. Spojovat jen tentýž boot podle TimeUS/SourceUS a doložit rozdíl časů; nejbližší 10Hz log není důkaz stejného 400Hz samplu. Přesné AP/UTB dvojice v RAM a agregace jsou primární numerický důkaz.

HW maska180222: ATT_FAST0, ATT_MED1, PID1; současné ANG/RATE/PID typicky10Hz, nikoliv400Hz. Struct PID52B/osa, ANG39B a RATE63B, tedy samotné 3PID+ANG+RATE≈258B/sada /2580B/s při10Hz před dalšími zprávami/FMT/page režii. Zvýšení frekvencí by změnilo zátěž a vyžaduje samostatnou kvalifikaci.

Obhajitelné jsou 400Hz agregace/validita, decimované tracking/limit/reset průběhy a native kontext. Neobhajitelná je 400Hz D-dynamika z 1–2Hz RAM, domyšlené řádky, closed-loop UTB účinnost z pasivního letu nebo bitová totožnost všech Attitude stavů z tolerance. Plant stále řídí AP.

## Letové předpoklady a konkrétní blockery

| Oblast | Doloženo | Zbývá před fyzickým letem |
|---|---|---|
| RC | map1/2/3/4, FS_THR_ENABLE1; při USB RC not found | skutečný CRSF/ELRS, kanály/range/neutral, směry, ztráta RC a převzetí pilotem |
| Arming/spool/motory | native AP a SITL; active UTB na H743 compile-out | bezpečný fyzický preflight dle AP, mapping/směry a FRAME_CLASS1/FRAME_TYPE18; USB test neověřil ESC |
| Baterie | nízké USB-only napětí bylo očekávané | skutečné LiPo měření, prahy a battery failsafe |
| Yaw/EKF | AHRS_EKF_TYPE3, EK3_SRC1_YAW1, COMPASS_USE/USE2/USE3=0, AUTODEC1 | skutečný yaw zdroj, heading, inicializace a EKF pod dynamikou; source1 sám neprokazuje magnetickou fúzi |
| Režimy/GPS | native STABILIZE a SITL LAND/RTL | pilotní režimy a bezpečný návrat; GPS/home pokud je zvoleno RTL, STABILIZE sám pozici nedrží |
| GCS failsafe | fyzicky uložené FS_GCS_ENABLE0 | [events.cpp](/home/jirka/ardupilot_utb/ArduCopter/events.cpp:126) při0 vrací bez akce; test se SITL hodnotou1 není fyzický důkaz |
| Logger | BLOCK4, stará metadata | first-open/runtime erase/lock timing, bezpečná kapacita a archivace; bez souhlasu nesmí nový zápis přepsat staré logy |
| Timing | HW Rate a dílčí brackety | Main perioda/end/output latence, preempce, dynamický stack; WCET neuzavřené |
| Uchování dat | bounded RAM a USB export | ověřený same-boot kanál/napájení a zahájení finite okna |

Vypnutý GCS failsafe se zde automaticky nezapíná. Bez doložené zvolené reakce na ztrátu řízení nelze vydat letové GO. Reálné RC/battery/yaw/motor kontroly nyní neproběhly.

## Je nutný nový firmware?

**Pro RAM data a první Main start-to-start kvalifikaci ne.** Preferovat současný APJ:

`/home/jirka/ardupilot_utb/tmp/utb-3g2-20261025_071149/firmware/SkystarsH7HD-bdshot_UTB3G2_PASSIVE_RAM_918dedaaeae9.apj`

SHA-256 `918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7`, 1155815B, board1075/serial200024001251303433333336. Zdrojový/symbol audit active H743 compile-out zachován.

**Pro úplnou Main execution/output duration a kauzální logger stopu současná rozhraní nestačí.** Výše uvedené malé sondy jsou oddělené od regulační matematiky a vyžadují samostatné schválení patche, jeho OFF/ON/unit/SITL/build regresí, RAM/flash/stack overheadu a konkrétního uploadu. Zatím žádný nový produkční patch ani APJ. Persistence se nezavádí pouze kvůli obecnému přání logovat více dat.

## Samostatný návrh 3G-3E – USB-only HW timing

Toto je plán, **nikoliv souhlas ani provedený experiment**. Bez LiPo, vrtulí, ARM a DataFlash zápisu. Použít přesný současný APJ, bez uploadu. Samostatně schvalovat každý běh:

1. Před fyzickým restartem archivovat uzavřenou 3G-3C session; boot111 již Flight znovu nespustí. Identita/health/úplné parametry a původní inventory pouze před měřením. Nový fyzický start provede až obsluha po výslovném schválení, bez automatického rebootu.
2. **E-A native baseline:** nový boot, všechny Flight/UAS/UNS/UCS RUN0, LOG_DISARMED0, SCHED_OPTIONS0, 400Hz/FSTRATE0, bez rate threadu. UTT_COUNT4000, jediný UTT_RUN1. Minimální stejná přednastavená telemetrie; během okna žádný FTP/polling/tasks. Po auto-stop sekvenčně export UTT, paměť/vlákna a health. UTT session je také finite a nelze opakovat v témže bootu.
3. **E-B Flight+Timing:** další samostatně schválený nový boot, stejný FW/config. UTT_COUNT4000 a UCS_F_COUNT4000/DECIM40; jednou UTT_RUN1 a jednou UCS_F_RUN1 těsně za sebou. Dva PARAM_SET nejsou atomický start: uložit příkazové časy a skutečné HW start/stop/počty, přiznat okrajové cykly a netvrdit přesně totožné okno. Během obou sessions jen příjem telemetrie. UAS0 skupina nerozlišuje Flight fáze; právě proto native a ON porovnávat v oddělených bootech.
4. Po dokončení všech RUN0 exportovat sekvenčně, ověřit 84 metrik, pairing, počty, UTT supported a paměť/stack před/po exportu. Obnovit COUNT/DECIM/intervaly, porovnat parametry a log metadata, uzavřít port. Logování nesmí být zapnuté a původní čtyři logy nesmějí být použity pro stress/erase.
5. Návrhová přijímací kritéria: správné counts; žádné missing/invalid/desync/unsupported/mismatch/motor_changes, restart/InternalError/logger drops nebo chyby přenosu. Žádná neobjasněná ≥5000µs dvouperiodová nebo >3000µs PM-threshold událost; při ní zastavit další běhy a vyhodnotit kontext. Tyto detekční hranice pocházejí z nominální periody a původního PM prahu, nejsou nové regulatorní limity. Samotné >2500µs je jitter statistika, ne automatický stop.
6. Porovnat Main/Rate/Attitude min/max/sum/kontext, CPU při stejné telemetrii, FREE/LRG regiony a watermark. Nenastavovat libovolnou „bezpečnou stack rezervu“ podle lokálního832B frame; rozsáhlou změnu watermarku/alokační chybu vyhodnotit jako blocker. Z naměřených maxim nedělat WCET. Main-end údaje zůstávají explicitně neověřené.
7. Stop při neshodě identity, COM chybě, restartu, ARM, poruše senzorů/EKF, InternalError, numerice/motor changes nebo závažné časové/paměťové/exportní anomálii. Bez retry, dalšího RUN, uploadu či rebootu navíc. Další fyzický krok jen po novém rozhodnutí.

USB-only E-A/E-B **neuzavřou logger rizika**. First-open/sector-boundary sonda a případný zásah do logovací oblasti potřebují zvláštní schválenou podfázi a ochranu archivů. LiPo/USB handoff je rovněž samostatný dosud neprovedený bench. Úspěch 3G-3E není letové povolení.

## Přímé odpovědi

1. Výpočetní náročnost: ano pro další omezenou HW kvalifikaci, nikoliv doložený letový WCET.
2. Main/WCET: chybí end/output bracket, úplný observer overhead a bounded preempce/waits; UTT měří post-wait start periody.
3. Logger: může blokovat native Main; příčina 5503/6480µs first-open stále neprokázaná.
4. DataFlash pro další fyzický let: zatím nekvalifikováno; navíc wrap může zničit staré logy.
5. RAM-only první let: podmíněně použitelný, nyní provozní NO-GO kvůli neověřenému získání dat téhož bootu.
6. Data po přistání: nepřerušit napájení, exportovat až DISARMED přes ověřený kanál; fyzický USB/LiPo handoff dnes neověřen.
7. Průběhy: ano pro decimovaný tracking a 400Hz agregace, ne pro plnou rychlou dynamiku/P95/P99.
8. Persistentní UTB: zatím není nutná; C je pouze návrh.
9. RC/yaw/failsafe: při USB chybělo reálné RC/LiPo, magnetické použití0 a heading neověřen, GCS failsafe0. SITL s jinými parametry tyto skutečnosti nenahrazuje.
10. Další USB test: přesné Main/Rate/Attitude start periody s/bez Flight, observer efekt, runtime heap/stack a úplnost exportu; ne let ani DataFlash stress.
11. Další bezpečně vymezená příprava GO; fyzický let NO-GO do odstranění konkrétních blockerů.

## Git a zachování výsledků

HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`, větev `test/skystars-5inch`. Původní čtyři necommitnuté 3G-3/A/B/C reporty a kapitoly/historie.txt zachované bitově. Tracked zdroje a index beze změny, přibyl pouze tento necommitnutý report a ignorované artefakty v novém tmp. Bez commitu/reset/clean/stash/rebase/push a bez kontaktu s FC. Konečné hash kontroly a status jsou v git-final.txt/evidence-sha256.json. Automatické pokračování k 3G-3E neproběhne.
