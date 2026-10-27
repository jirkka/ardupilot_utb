# UTB – Připravenost pasivního Flight SHADOW a prvního letu


> Stav: HW-2B PASS, původní AP motorová autorita zachovaná. V této fázi se nic neuploadovalo a fyzické ARM ani let neproběhly.
>
> Nejbližší krok je FLIGHT-PREP-A/B: skutečné RC, motor mapping a směry, orientace stabilizace, konkrétní LiPo a bateriové reakce, yaw/EKF a vhodný native firmware/logger profil. Motorové testy vyžadují samostatný souhlas.
>
> První let může být pouze na native ArduPilotu s UTB runtime vypnutým. Dokončený vlastní UTB logger, hard WCET, elektrické měření DShot ani position hold nejsou podmínkou tohoto prvního native letu.
>
> Flight SHADOW umí pasivně počítat také při ARMED; ověřeno nyní znovu v SITL, zatím ne při fyzickém letu. Současný Flight recorder ukládá jen do RAM. Palubní BIN logger pro novou větev 3A není implementovaný; níže je konkrétní návrh a jasně oddělené zbývající ověření pro pozdější FLIGHT-2.

## Rozhodnutí a rozsah

HW-2B je uzavřený PASS a archivovaný samostatně; nebudou přidávané další obecné DISARMED časové benchmarky. **První native let je nejbližší provozní cíl, nikoliv aktivní UTB řízení.** Tento report není souhlas k fyzickému letu, LiPo bench, motorovému testu nebo uploadu.

Nebyla nalezena konkrétní chyba vyžadující přepis ověřené PID/Attitude matematiky. Produkční zdroje se v této etapě neměnily. Současný pasivní Flight záznam je RAM-only. Historické UTBC/UTBR/UTBM zaznamenávají starý AP_UTB::evaluate controller, nikoliv UTB_PID/Rate/Attitude z 3A; jejich úspěšný400 Hz logging není důkaz vhodnosti pro nový Flight.

Nový persistentní logger **není implementovaný ani označený jako ověřený**. Níže je nejmenší konkrétní návrh. Volitelný vývoj nového loggeru se nyní nerozšiřoval: oddálil by první native let a bez následného samostatně schváleného uploadu/HW kvalifikace by stejně neopravňoval k prvnímu letu s UTB BIN. Nejkratší cesta využije nejprve native firmware s vypnutým UTB runtime, pak odděleně persistentní pasivní logger. Není potřeba další refactoring nebo PID tuning.

## Původ regulátorů – skutečné zdroje

Základem je ArduPilot Copter4.7.1, původ převzatých funkcí/copyright a GPLv3+ zachovaný v UTB souborech a docs/UTB_PHASE_3A_CS.md. Nejde o wrapper volající native PID podruhé. Kopie UTB_PID::update_all/update_i/get_ff/reset_I/filter alpha mají po normalizaci pouze názvů tříd, whitespace a komentářů přesně stejné tělo jako AC_PID; skutečný výpočet probíhá ve vlastních objektechUTB_PID.

| Oblast | UTB source a funkce | Konkrétní AP reference | Shoda / adaptace |
|---|---|---|---|
| PID | libraries/AP_UTB/UTB_PID.cpp:214 update_all;358 update_i;386 get_ff | libraries/AC_PID/AC_PID.cpp:196/340/368 | P/I/D/FF/D_FF, FLTT/FLTE/FLTD, clamp IMAX, directional anti-windup, PDMAX, pd/i scale převzaté; vlastní shadow_restore:633 |
| Slew | UTB_PID.cpp:512 UTB_PID_SlewLimiter::modifier | libraries/Filter/SlewLimiter.cpp:45 | Převzatý algoritmus; continuous capture nepodporuje SMAX!=0, protože nedováží celý slew stav |
| Rate | UTB_Rate.cpp:533 shadow_rate_step | AC_AttitudeControl_Multi.cpp:452 rate_controller_run_dt, setters505–512 | Stejné pořadí3PID, actuator_sysid a yaw FF scale; adaptérem jsou hodnotové vstupy a inertní ShadowOutput |
| Attitude | UTB_Attitude.cpp:540 input_euler_angle_roll_pitch_euler_rate_yaw_rad;629 attitude_controller_run_quat | AC_AttitudeControl.cpp:421/1063 | AP quaternion/thrust-heading/FF algoritmus; bez opakovaných native capture hooků |
| Shaping | UTB_Attitude.cpp:748 attitude_command_model, lokální utb3_* | AC_AttitudeControl.cpp:1182 a libraries/AP_Math/control.cpp | AP shaping/sqrt/jerk funkce lokálně převzaté, prefix utb3_ a snapshot-backed AHRS adaptér |
| Reset | UTB_Attitude.cpp:832/847/866 reset_target/reset_yaw/inertial_frame_reset | AC_AttitudeControl.cpp:1326/1357/1395 | AP matematika; yaw pole nahrazeno UTB_State::yaw getterem, žádný vlastní EKF |
| State/config | UTB_State.h, AP_UTB_AttitudeShadow.h:15 UTB_AttitudeShadowMath | AC_AttitudeControl::attitude_shadow_sensors/config/history | Vlastní inertní value adapter; gyro/quaternion/yaw převzaté z téhož native volání |
| Comparator | AP_UTB_NativeShadow.cpp:95,AP_UTB_AttitudeShadow.cpp:117 | Native pre/post snapshots, nikoliv druhé spuštění native regulátoru | Vlastní diagnostika, validity/seq/config/reset kontrola, maxima/RMSE; není původní letový algoritmus |
| Chain | AP_UTB_ChainShadow.cpp:46/64 | Copter native Rate před update_flight_mode/Attitude | Vlastní jednocyklické propojení source(N−1)→consume(N), nikoliv nový PID |
| Flight recorder | AP_UTB_FlightShadow.cpp:72–175 | Nemá native regulační analog | Vlastní bounded RAM recorder/epoch/decimace, bez motorové autority |

Podrobná tabulka132 out-of-line funkcí má všechna požadovaná pole (původ, native source/function, vstupy/jednotky, výstupy, vlastní stav a pozorované member assignments, evidence, motorová izolace, ARMED a rizika): `tmp/utb-flight-readiness-20261025_review-01/function-audit.csv`. Automaticky47 normalizovaných těl shodných; ostatní nejsou automaticky prohlášené za stejný algoritmus. `native-body-diffs.txt` ukazuje skutečné adaptace, `static-helper-origin.json` lokální AP_Math helpery. Jednoduché inline getters/empty inertní overrides jsou v AP_UTB_AttitudeShadow.h a UTB_State.h, samostatně od out-of-line inventáře. Textová shoda není formální důkaz ekvivalence compileru nebo všech konfigurací; numerické testy jsou jiný druh důkazu.

## Vstupy, historie a datový tok

```text
INS.wait_for_sample → gyro/AHRS native hodnoty → native Rate/PID
  → native AP_Motors setters → původní motors_output → HAL/ESC
  → ostatní fast tasks → native ModeStabilize/Attitude → target pro další Rate
                 |
                 + readonly pre/post kopie + skutečný dt/gain/limit/reset
                 → vlastní UTB_Attitude historie → source(N−1)
                 → UTB_Rate::shadow_rate_step → vlastní3×UTB_PID
                 → inertní ShadowOutput → comparator → bounded RAM
```

Konkrétní integrační místa: ArduCopter/Attitude.cpp:10–41 prepare/context a skutečný scheduler.get_last_loop_time_s; AC_AttitudeControl_Multi.cpp:481–528 snapshot target/gyro/dt/pd_scale/i_scale/actuator/yaw_ff_scale/limit před/po jediném původním update_all; AC_AttitudeControl.cpp capture kolem skutečné Attitude operace. Gyro/body-rate rad/s, úhly rad, dt s, quaternion jednotkový, motor control normalizovaný AP výstup, gain/limit podle native aktuálního stavu. Native gyro se znovu nesampluje pro porovnání; AP INS filtry/AHRS/EKF se nepřepočítávají.

UTB_PID::shadow_restore mění jen vlastní objekt. Continuous proces importuje historii při synchronizaci, pak pouze skutečné externí i_set/reset_filter události a kontroluje vlastní předchozí historii; neimportuje I každým cyklem skrytě. HW DISARMED přesto často legitimně přebírá externí resetované I (11994 událostí v2B), což není důkaz ARMED nezávislé integrace. Attitude config/dt/angle_scale se předává dle skutečné operace; mode/config změny mají desync/epoch politiku. Rate nové SMAX/notch/gyro-instance/seq/config nesoulady odmítá diagnosticky, nemění AP parametry kvůli nulové chybě.

UTB_AttitudeShadowMath má vlastníUTB_State bez AP_AHRS pointeru v snapshot režimu, tři vlastníPID pro yaw-P limit a prázdné rate_controller_run/set_throttle_out overrides. Neexistuje zde motorový sink. Celá aktivní UTB_Rate/Output/Core cesta je odděleně pouze SITL compile-time. Snapshot AC_PID::shadow_snapshot je const; neexistuje volání native restore/update_all podruhé z této pasivní větve.

AP_Motors limit feedback se kopíruje, nevypočítává se vlastní odlišný allocator. Převzatý anti-windup dovoluje při limitu pouze zmenšovat I ve směru proti chybě, clamp IMAX; FF a D_FF oddělené, yaw FF navíc skutečný native scalar. Smyčku plantu i saturaci stále řídí AP, nikoliv UTB.

## ARMED, disarm, módy a failsafe

Flight AP_UTB_FLIGHT_SHADOW_AVAILABLE je na stávajícímH743 ON. AP_UTB_ChainShadow::prepare nepodmiňuje výpočet DISARMED, ale vyžaduje STABILIZE, main-thread400 Hz, FSTRATE0/no rate thread, bez custom/att_enable controller konfliktu a podporovanou matematiku. AP_UTB_FlightShadow::context mění armed epoch/arm_changes, ne native arm/spool; ARM sám automaticky nezastaví ani nesmaže recorder. Průběh AP disarm/spool resetů zModeStabilize::run (mode_stabilize.cpp:28–48) se zachytí/replayuje do vlastní historie. DISARM nemusí sám uzavřít okno; konecCOUNT/mode exit/fault. COUNT2–48000, maximálně nominálně120s při400 Hz, ring128 záznamů, overwritten explicitní; bez nekonečné RAM akumulace. RAM export jen frozen+DISARMED (ChainShadow.cpp:359–367), reboot ji ztratí.

STABILIZE→LAND/RTL nebo failsafe mění native režim původnímAP; Flight při mode!=0 freeze reason7, dál nepočítá nepodporovanou smyčku, AP normálně pokračuje. Chyba UTB zastaví pouze diagnostiku, žádná náhrada native failsafe vypnutím tahu. Timing/Execution jsou jiné session: loop v AP_UTB_Timing.cpp:225–235 odmítne/stoppuje soft_armed, proto pro první let **UTT_RUN0**; jejich guardy se neodstraňovaly. UAS/UNS/UCS/UB3 a legacyUTB_SHADOW také0, aktivní UTB režimy na H743 compile-out.

Rate/output/target comparator vyžaduje přesnou nulu, Attitude/history toleranci1e−6×max(1,|expected|) s quaternion sign ekvivalencí; ne všechny Attitude vstupní historie jsou bitově stejné. Flight analyzer check_supported je přísnější: k současnému PASS požaduje všech84 max/RMSE0; nevyhlazuje chyby průměrováním.

## Skutečné důkazy a omezení testování

HW-2B:4000 attempts,3999paired,8000Att,12000 motorchecks,0changes,84 × max/RMSE0, CPU28,8%, Main max2516µs, Rate2519µs. Jde o klidný DISARMED rozsah, ne ARMED let.

Nově spuštěný **SITL FF test PASS**, stávající binárka SHA-256 `37d461d53326a3e021cca57a5c672e722b425ce741fd520ccfcdfb19d578c263`: 12000 attempts, 11999 paired, 13471 validAtt, independent_i=10525, shaping=5984, motor_checks=25471, motor_changes0,84 × max/RMSE0, loggerdrops0, ARM/dynamickéR/P/Y/FF/D_FF/LAND/DISARM. Evidence `tmp/utb-flight-readiness-20261025_review-01/sitl-ff/` a `new-sitl-summary.json`. **35 Python parser unit testů PASS** vnovémparser-tests.xml.

Existujících8 úspěšných session (normal/ff/rc/gcs/mode-exit6/mode-exit9/overflow/interrupted) bylo ověřeno proti stejnému executable hash; nejsou vydávané za nově spuštěné: `existing-sitl-evidence.json`. Starší readiness3G3D pokrývá FILE backend ENOSPC/delay, maxCOUNT, ARM-export odmítnutí, reboot/lifecycle; tento report neměnil jejich zdroje ani důkazy. Nejde o výkonovou simulaci SPI BLOCK H743. Nový BIN logger neexistuje, proto žádný unit/SITL/H743 build/stack výsledek pro takový logger není PASS. SoučasnýAPJ build/symbol audit z3G3F zachovaný; nový build nebyl potřebný, žádný kód se neměnil.

## Palubní BIN logger – nejmenší návrh, NEIMPLEMENTOVÁNO

Použít původní AP_Logger/DataFlash, nový compile-time guard defaultOFF a runtime defaultOFF. Producent pouze decimovaná hodnotová kopie stávajících native/UTB výsledků;4položkováSPSC fronta, žádná alokace nebo blokujícíAPI vRate/Attitude, žádný nový zámek motorové cesty. Reuse existujícího utb_log workeru; starý experiment může být konfliktně odmítnut pro jediný sampler. PřímýWriteBlock zfast hooku zakázat; nepřidávatFS/SPI driver nebo MAVLink požadavek letu.

Pevný návrhverze1, názvyUFPI/UFRS/UFLS vaktuálníchsources nenalezené, **ID zatím nerezervovaná** (vybrat při implementaci proti skutečné tabulce a metadata testu, neměnit stávajícíID). Návrh velikostí počítá packed header3B, explicitnítypy, nepřenáší padding:

| Paket | Pole a typy | Packed B | Na sadu |
|---|---|---:|---:|
| UFPI | TimeUS uint64, Seq uint32, Axis uint8, Mode uint8, Status uint16; AP aUTB P/I/D/FF/D_FF po5×float32 |59|3|
| UFRS | stejnýcommon, SourceUS uint32; Dt,Gyro,ATar,UTar,AOut,UOut,AFFOut,UFFOut float32 |55|3|
| UFLS | TimeUS uint64,Version uint8,Mode uint8,Flags uint16; Seq/Requested/Queued/Accepted/Dropped/Invalid/BackendRejected uint32; HWM/QDepth uint16 |47|≤1Hz|

UFPI15FMT polí, UFRS14, UFLS14: limit16 respektovaný. dt s, gyro/targetrad/s, normalizedoutput; Status obsahujeARM/valid/limit/reset/unsupported/mismatch podle protokolu. TimeUS musí mít doložený MCU původ (existující ratecapture je32bitový, bezhlavě jej nepřetypovat na64bitovou absolutníepochu); například zachovat capture low32+source a jednoznačnýboot/high-word anchor, nebo předat jižexistující64bitovýMain čas s přesně popsanýmoffsetem. Toto je implementační položka k ověření wrap testy, ne hotováčasovágarance.

Jedna kompletní decimovanásada342B, výchozí10 Hz=3420B/s plusstatus47=**3467B/s**, experimentálních20 Hz=**6887B/s**.400 Hz by bylo136847B/s a není pro prvnílet odůvodněné; návrh jej nenabízí. Nízkáfrekvence slouží tracking/limity/numerické dvojice, ne detailní400 Hz D dynamika. Rozdíly AP−UTB odvodit přesně offline; nepřidávat redundantnípole. Native PIDinfo je dostupný ze skutečnéhoafter snapshotu, nemusí se odhadovat. Native gyro a reference pocházejí ztéhožframe.

Odhad4×352B interníslot+≤128Bindex/stat=**≤1536B novéstatickéRAM**, beznovéhostacku, nikolivlinkerměření. ExistujícíFlight304×128=38912B užje součástíaktuálníhoimage, nelze jej přičítat znovu nebo beznávrhu recyklovat. Worker má naHW stack3504B, watermark2456B před/po2B; nováserializace musí mít vlastníoffline/HW stackověření. CPU/flashdelta nového loggeru **neměřené**, H743 Main deltaFlight≈91µs předmotor a CPU+9,69pp neobsahují novýBIN export.

Plnáfronta: drop celé nové sady, monotónní counter, nečekat. Backend může odmítnout jednotlivýpaket:6paketůnejsou transakce, Acceptedznamená kompletní přijetí backendem, nikoliv prokázaný trvalýzápisFlash; parser podleboot/Seq/Axis/type zkontroluje kompletní6sad a označí partial/dropped. Logger loss pouzeomezuje observability, nevypíná jinakzdravéAP řízení nebo UTB matematiku. StopCount/fault/modeexit zůstávajífinite; přiARM/DISARM uložit epoch/flag, finishnaCOUNT, stopvLAND/RTL; žádné nekonečné uchovávání. Prvníprofil60s, současnýFlightmax120s; pětiminutovýcapacityrozpočet níže není tvrzení, že dnešní recorder umožňuje300s.

Kód loggeru by minimálně zasáhl AP_UTB_FlightShadow.*, existujícíworker libraries/AP_UTB/Copter/utb.cpp, LogStructure/ID tabulku, buildoption/guard a standaloneBIN parser/test. Novýsender by musel mítunitfixtures proSPSCoverflow/wrap/partialbackend/stop/resets aSITLSTABILIZEARMED R/P/Y/I/FF, LAND/RTL,RC/GCSfailsafe,backendabsent/full/delay, duration/timestamp/BINparse aoff/onmotorcomparison. Pak H743 ON/OFF/logging-disabled build+size/stack a zvlášť schválenýHW logger test. Nyní jde o konkrétní návrh, žádný ztěchto novýchlogger scénářů nebyl proveden.

## BLOCK DataFlash – konkrétní rizika a kapacita

Skutečnýbackend4=BLOCK, JEDECW25Q128,16MiB. AP_Logger_Block.cpp:32–75 přidělujeDMA buffer/writebuf, rezervuje poslední64KiB a může přiNeedErase automatickyEraseAll, jinakvalidate_log_structure/recovery. _WritePrioritisedBlock:140 získáváwrite_sem blokujícímWITH_SEMAPHORE předRAMcopy; samotnéwritebuf-full pouzedropnenestačí k tvrzení neblokujícífunkce. StartNewLog:549 držísem a hledáfind_last_page:677. FinishWrite:91–118 wrapuje nazačátek a maže následujícíblok; starélogy se mohou přepsat. IOtimer:877 dělá recoveryerase,917 zapisuje nejvýše1page. JEDEC WaitReady:93 čeká100µs kroky až5 s poBusy, SPIoperace majídev_sem (page read:246/251; program:276; sectorerase:293). Worker může nepřímo soutěžit sMain přes původnílogger zámky; samotné přesunutíUTBWriteBlock naworker nedokazuje bezpečnoulatenci.

LOG_FILE_BUFSIZE fyzicky172KiB, aleBLOCK jej skutečněořezává na64KiB a může při nedostatkuRAM půlit;parametr172 tedy nedokládá172KiBRAMbuffer. AktuálníHWbuffer allocation nenístřímemoryexportem jednoznačněodečtena, nezaměňovat sregionFREE.

BLOCK_LOG_VALIDATE0, PageHeader6B→250Bpayload/256Bpage. W25Q12865536stránek, reserved256→65280×250=**16320000B teoretického logpayloadu**. Historickéčtyři logy11885734B→rozdíl4434266B pouzehorníorientačníbudget, **ne přesně změřená volnáflash**:headers/padding/fragmentace/rozloženílogů a dalšízápisy mohoučástspotřebovat. bufferspace_available() vracíceloukapacitu ringbackendu, nevolnémísto. HW-2Bnezapisovalo, nevytvořilo novoukapacitnívalidaci.

| Návrhový native tok | UTB10 Hz +status |60s +50kBmetadata |300s +50kBmetadata |
|---:|---:|---:|---:|
|5000B/s|3467B/s|558020B|2590100B|
|10000B/s|3467B/s|858020B|4090100B|
|20000B/s|3467B/s|1458020B|7090100B|

Čísla jsou **rozpočtové předpoklady, ne změřenýletovýnative datovýtok**, pluspagefaktor256/250≈1,024 a headroom musí být zahrnuté. Vyššínative tok/20 HzUTB může300s překročitorientačnízbytek4,43MB a přepsatstarélogy. Předtakovýmtestem znovuověřit skutečnýlogmask/rates a případnésamostatněschválené vyčištěníarchivovanýchlogů. Staré4archivy nesmazané a žádnýerase nepovolen.

Původníprodlevy5503/6480µs **nejsou vysvětlené aniopravené**. TRACE experiment zachytilLOG_REQUEST_LIST/GCSupdate_receive23,938ms/period24,238ms ještě předStartNewLog (count0); pozdějšífirst-openPM6070µs nebylvRAMstopě. docs/UTB_LOGGER_TRACE_CS.md aUTB_PHASE_3D2_LOGGER_CS.md. KvalifikaceRAM-only2A/B neprokazujebezpečnézápisy/erase za letu. Prakticky připravit loggerDISARMED předARM, ověřit health/drops a dostatečnoukapacitu, nikdylisting/download za letu; nenazývatto opravoupříčiny. Nenípotřebnéblokovatprvnínative letnaobecnéhardWCETanalýze, ale nesmí se letět s konkrétnínevyhodnocenouzávadouloggeru: použít letověpoužitelnýnativeFW a bezpečněověřenouloggingkonfiguraci, nebo zvlášťodsouhlasenýloggerOFF profil se ztrátouletovýchdat.

## Přirozený předletový checklist – pouze plán

### FLIGHT-PREP-A: USB-only, bezLiPo/vrtulí

- [ ] Archivovat výsledky, zálohovatparametry, identifikovat konkrétníletovýFW/SHA a zachovatUTBactivecompile-out. DiagnostickýEXECONkandidát neníautomatickyletovýrelease; UTT/UCS_F/UCS/UAS/UNS/UB3/legacySHADOWRUN0.
- [ ] SrovnatFRAME_CLASS1/FRAME_TYPE18 s fyzickým5″rámem; AHRS_ORIENTATION0 ověřitnakloněním:rovina a správnéroll/pitch/yaw směry. Hodnotaparametru sama nenífyzickýdůkaz.
- [ ] Napájení/polarita/kabeláž/senzory, AHRS/EKF, kalibraceaccel/gyro; žádnýbypassprearm. ARMING_SKIPCHK0 zachovat; chybějícíARMING_CHECK neodhadovat podlejinéAPverze.
- [ ] RC CRSF/ELRS (SERIAL1_PROTOCOL23): skutečnýpříjem/vysílač/link, kanályRCMAP1/2/3/4 proRoll/Pitch/Throttle/Yaw, trim/range988–2011 a směrkaždéosy doložitfyzicky, vyhrazenýdisarm; žádný ARMnyní.
- [ ] Ověřitpilotnímode-switch a návratnativeSTABILIZE, neUTBaktivnímódy; STABILIZE nedržípozici/výšku.
- [ ] Yaw: AHRS_EKF_TYPE3/EK3_SRC1_YAW1, COMPASS_USE/USE2/USE3=0. Ověřit skutečnýyawzdroj a headingvhodný prozvolenýrežim; nezařazovatRTLbezověřenéhoGPS/home/yaw. NevyžadovatpositionholdproSTABILIZE.
- [ ] Failsafe: FS_THR_ENABLE1 a reálnouztrátuRC/obnovení ověřitnejprvebezvrtulí. FS_GCS_ENABLE0 jeaktuálnívypnutýGCSfailsafe, nejde oPASS; zvolitpolitiku pro skutečnýtelemetrickýprofil, nezapínatautomaticky.
- [ ] VybratnativeLOGmask/rates, kapacitu, start/stop, drops, zachováníhistorickýchdat a možnostnative-onlyloggerOFF pouzevýslovnýmrozhodnutím. UTB persistentnílogger zatímnepoužívat.
- [ ] RAM/CPU základ s runtimeUTBOFF, žádnéFTP/tasks/listing běhemletu, ověřit provoz po odpojeníUSB a správné napájecízdroje.

### FLIGHT-PREP-B: LiPo, stále bezvrtulí – NOVÉ samostatné schválení

- [ ] Obsluha určískutečnou4S/6Sbaterii, kapacitu/hmotnost a elektrickoukompatibilitu. USBúdaje neprokazujíLiPonapětí; žádnédoporučenípřipojit6Sjenpodleodhadu.
- [ ] Změřit/kalibrovat napětí/proudmonitoru. DnešníBATT_MONITOR4, capacity3300mAh, ARM14,0V/LOW13,6V/CRT13,2V a oběbatteryfailsafeactions0 nejsou automatickysprávné proskutečnoubaterii, zejména6S. Prahy/reakce určit podle článků a zátěže a samostatněschválitzměny.
- [ ] Ověřit ESC/protokol: MOT_PWM_TYPE=6 znamená DShot600 podle AP_Motors_Class.h:367 a AP_MotorsMulticopter.cpp:95, napájení a chyby; nedělatPWM/DShotzměnunaslepo.
- [ ] Po výslovném schváleníkonkrétníhomotorovéhopostupu bezvrtulí ověřitmotor mapping1–4, směrkaždéhomotoru, reakce stabilizace; teprvepoté zvolitodpovídajícívrtule/směry. Tento reportžádnýmotortestnespustil.
- [ ] OvěřitRC/disarm/failsafe/napájení po odpojeníUSB, žádnýbrownout. USB+LiPohandoff vyžadujeověřenízapojení/powerdomains, neprovádětautomaticky.

### FLIGHT-1: nejkratší první native let

- [ ] Kompetentnípilot, vhodnýprostor, počasí, odděleníokolí, mechanickýcheckrámu/upevnění/vrtulí, předchozíA/Bzelené a samostatnéletovépovolení.
- [ ] VýhradněnativeArduPilot, runtimeUTBOFF a Timing/ExecutionOFF, manuálníthrottle/STABILIZE pouze pokud pro nějověřenýpilotnírežim. Krátkývzlet/vis/přistání podlepilotníhozvládnutí, vyhodnotitnativegyro/attitude/RC/baterii/vibrace/logging.
- [ ] Nepodmiňovat hardWCET, elektrickýDShotcompletion, opticalflow/positionhold/indoor, obstacle avoidance, mise nebo vlastníUTBúplnost. Logger musí mít ověřenýprovozníprofil; dalšíobecnýRAMbenchmarknenínutný.

### FLIGHT-2: native řízení + pasivní UTB BIN

- [ ] Implementovat/uvolnit minimálníloggerpodlenávrhu, metadata/BINparser/SITL/offlineH743build a kvalifikovatna H743 předletem.
- [ ] OvěřitARMEDpasivníizolaci, finiteokno60s/max120s, skutečnýloggerthroughput/locks/erase/memory/drops a dostatečnouflashrezervu.
- [ ] SamostatněschválitkonkrétníAPJupload a kvalifikaci; ažpoté záznam naFlight2, stálejenAPmotorováautorita.
- [ ] Pokud bybylprvnípasivníletRAM-onlyalternativa, musí býtověřenýsame-boottrigger a exportnapájení/kanál; tatooblast dnesneníHWověřená. Nenípotřeba tentoRAMhandoffřešitproprvnínative-onlylet.

## Co aktuálně brání letu

NativeFLIGHT1: fyzicky nedoloženéRC/trim/směry/disarm/failsafe, motor mapping/směry/stabilizace, skutečnáLiPo/thresholdy a batteryreakce, yaw/EKFzvolenéhorežimu a určeníletověpoužitelnéhoFW/loggerprofilu. Tyto konkrétníbody nahradit ověřenýmchecklistem, ne dalšímobecným benchmarkem. TentoUSBtestpřiRCnepřítomnosti/bezESCnapájení je nemohlověřit.

PassiveBINFLIGHT2 navíc: logger3Aneexistuje, chybíimplementace ajejíunit/SITL/metadata/H743build/HWčasování/kapacita, runtimevolba/finiteexport. SoučasnáFlightmatematikaARMEDpodporu má a novýSITL nepřineslnumerickouodchylku; fyzickýARMEDnumerickýtestzatímneproběhl.

## Funkční inventář

| Funkce | UTB source | Původ / native reference |
|---|---|---|
| UTB_PID::UTB_PID | libraries/AP_UTB/UTB_PID.cpp:121 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:103 AC_PID::AC_PID |
| UTB_PID::set_filt_T_hz | libraries/AP_UTB/UTB_PID.cpp:151 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:133 AC_PID::set_filt_T_hz |
| UTB_PID::set_filt_E_hz | libraries/AP_UTB/UTB_PID.cpp:157 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:139 AC_PID::set_filt_E_hz |
| UTB_PID::set_filt_D_hz | libraries/AP_UTB/UTB_PID.cpp:163 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:145 AC_PID::set_filt_D_hz |
| UTB_PID::set_slew_limit | libraries/AP_UTB/UTB_PID.cpp:169 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:151 AC_PID::set_slew_limit |
| UTB_PID::set_notch_sample_rate | libraries/AP_UTB/UTB_PID.cpp:176 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:158 AC_PID::set_notch_sample_rate |
| UTB_PID::update_all | libraries/AP_UTB/UTB_PID.cpp:214 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:196 AC_PID::update_all |
| UTB_PID::update_error | libraries/AP_UTB/UTB_PID.cpp:335 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:317 AC_PID::update_error |
| UTB_PID::update_i | libraries/AP_UTB/UTB_PID.cpp:358 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:340 AC_PID::update_i |
| UTB_PID::get_p | libraries/AP_UTB/UTB_PID.cpp:371 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:353 AC_PID::get_p |
| UTB_PID::get_i | libraries/AP_UTB/UTB_PID.cpp:376 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:358 AC_PID::get_i |
| UTB_PID::get_d | libraries/AP_UTB/UTB_PID.cpp:381 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:363 AC_PID::get_d |
| UTB_PID::get_ff | libraries/AP_UTB/UTB_PID.cpp:386 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:368 AC_PID::get_ff |
| UTB_PID::get_ff_component | libraries/AP_UTB/UTB_PID.cpp:391 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:373 AC_PID::get_ff_component |
| UTB_PID::get_dff_component | libraries/AP_UTB/UTB_PID.cpp:396 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:378 AC_PID::get_dff_component |
| UTB_PID::reset_I | libraries/AP_UTB/UTB_PID.cpp:402 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:384 AC_PID::reset_I |
| UTB_PID::load_gains | libraries/AP_UTB/UTB_PID.cpp:409 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:391 AC_PID::load_gains |
| UTB_PID::save_gains | libraries/AP_UTB/UTB_PID.cpp:421 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:403 AC_PID::save_gains |
| UTB_PID::get_filt_T_alpha | libraries/AP_UTB/UTB_PID.cpp:433 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:415 AC_PID::get_filt_T_alpha |
| UTB_PID::get_filt_E_alpha | libraries/AP_UTB/UTB_PID.cpp:439 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:421 AC_PID::get_filt_E_alpha |
| UTB_PID::get_filt_D_alpha | libraries/AP_UTB/UTB_PID.cpp:445 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:427 AC_PID::get_filt_D_alpha |
| UTB_PID::set_integrator | libraries/AP_UTB/UTB_PID.cpp:451 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:433 AC_PID::set_integrator |
| UTB_PID::relax_integrator | libraries/AP_UTB/UTB_PID.cpp:459 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:441 AC_PID::relax_integrator |
| UTB_PID_SlewLimiter::UTB_PID_SlewLimiter | libraries/AP_UTB/UTB_PID.cpp:500 | AP copied body (normalized); libraries/Filter/SlewLimiter.cpp:33 SlewLimiter::SlewLimiter |
| UTB_PID_SlewLimiter::modifier | libraries/AP_UTB/UTB_PID.cpp:512 | AP copied body (normalized); libraries/Filter/SlewLimiter.cpp:45 SlewLimiter::modifier |
| UTB_PID::shadow_snapshot | libraries/AP_UTB/UTB_PID.cpp:609 | AP copied body (normalized); libraries/AC_PID/AC_PID.cpp:451 AC_PID::shadow_snapshot |
| UTB_PID::shadow_restore | libraries/AP_UTB/UTB_PID.cpp:633 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_Rate::UTB_Rate | libraries/AP_UTB/UTB_Rate.cpp:342 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:333 AC_AttitudeControl_Multi::AC_AttitudeControl_Multi |
| UTB_Rate::update_althold_lean_angle_max | libraries/AP_UTB/UTB_Rate.cpp:355 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:345 AC_AttitudeControl_Multi::update_althold_lean_angle_max |
| UTB_Rate::set_throttle_out | libraries/AP_UTB/UTB_Rate.cpp:370 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:360 AC_AttitudeControl_Multi::set_throttle_out |
| UTB_Rate::set_throttle_mix_max | libraries/AP_UTB/UTB_Rate.cpp:386 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:376 AC_AttitudeControl_Multi::set_throttle_mix_max |
| UTB_Rate::get_throttle_boosted | libraries/AP_UTB/UTB_Rate.cpp:394 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:384 AC_AttitudeControl_Multi::get_throttle_boosted |
| UTB_Rate::get_throttle_avg_max | libraries/AP_UTB/UTB_Rate.cpp:415 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:405 AC_AttitudeControl_Multi::get_throttle_avg_max |
| UTB_Rate::update_throttle_gain_boost | libraries/AP_UTB/UTB_Rate.cpp:422 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:412 AC_AttitudeControl_Multi::update_throttle_gain_boost |
| UTB_Rate::update_throttle_rpy_mix | libraries/AP_UTB/UTB_Rate.cpp:435 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:425 AC_AttitudeControl_Multi::update_throttle_rpy_mix |
| UTB_Rate::rate_controller_run_dt | libraries/AP_UTB/UTB_Rate.cpp:462 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:452 AC_AttitudeControl_Multi::rate_controller_run_dt |
| UTB_Rate::rate_controller_run | libraries/AP_UTB/UTB_Rate.cpp:494 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:544 AC_AttitudeControl_Multi::rate_controller_run |
| UTB_Rate::parameter_sanity_check | libraries/AP_UTB/UTB_Rate.cpp:501 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:554 AC_AttitudeControl_Multi::parameter_sanity_check |
| UTB_Rate::set_notch_sample_rate | libraries/AP_UTB/UTB_Rate.cpp:523 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:576 AC_AttitudeControl_Multi::set_notch_sample_rate |
| UTB_Rate::shadow_rate_step | libraries/AP_UTB/UTB_Rate.cpp:533 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_Attitude::get_latest_gyro | libraries/AP_UTB/UTB_Attitude.cpp:479 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:234 AC_AttitudeControl::get_latest_gyro |
| UTB_Attitude::reset_rate_controller_I_terms | libraries/AP_UTB/UTB_Attitude.cpp:491 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:290 AC_AttitudeControl::reset_rate_controller_I_terms |
| UTB_Attitude::reset_rate_controller_I_terms_smoothly | libraries/AP_UTB/UTB_Attitude.cpp:499 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:298 AC_AttitudeControl::reset_rate_controller_I_terms_smoothly |
| UTB_Attitude::rate_controller_target_reset | libraries/AP_UTB/UTB_Attitude.cpp:507 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:306 AC_AttitudeControl::rate_controller_target_reset |
| UTB_Attitude::landed_gain_reduction | libraries/AP_UTB/UTB_Attitude.cpp:517 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:316 AC_AttitudeControl::landed_gain_reduction |
| UTB_Attitude::input_euler_angle_roll_pitch_euler_rate_yaw_rad | libraries/AP_UTB/UTB_Attitude.cpp:540 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:421 AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_rad |
| UTB_Attitude::input_rate_bf_roll_pitch_yaw_2_rads | libraries/AP_UTB/UTB_Attitude.cpp:600 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:715 AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2_rads |
| UTB_Attitude::update_attitude_target | libraries/AP_UTB/UTB_Attitude.cpp:619 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1053 AC_AttitudeControl::update_attitude_target |
| UTB_Attitude::attitude_controller_run_quat | libraries/AP_UTB/UTB_Attitude.cpp:629 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1063 AC_AttitudeControl::attitude_controller_run_quat |
| UTB_Attitude::thrust_heading_rotation_angles | libraries/AP_UTB/UTB_Attitude.cpp:673 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1107 AC_AttitudeControl::thrust_heading_rotation_angles |
| UTB_Attitude::thrust_vector_rotation_angles | libraries/AP_UTB/UTB_Attitude.cpp:696 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1130 AC_AttitudeControl::thrust_vector_rotation_angles |
| UTB_Attitude::attitude_command_model | libraries/AP_UTB/UTB_Attitude.cpp:748 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1182 AC_AttitudeControl::attitude_command_model |
| UTB_Attitude::scale_I_to_angle_P | libraries/AP_UTB/UTB_Attitude.cpp:773 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1229 AC_AttitudeControl::scale_I_to_angle_P |
| UTB_Attitude::ang_vel_limit | libraries/AP_UTB/UTB_Attitude.cpp:786 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1280 AC_AttitudeControl::ang_vel_limit |
| UTB_Attitude::body_to_euler_limit | libraries/AP_UTB/UTB_Attitude.cpp:809 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1303 AC_AttitudeControl::body_to_euler_limit |
| UTB_Attitude::reset_target_and_rate | libraries/AP_UTB/UTB_Attitude.cpp:832 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1326 AC_AttitudeControl::reset_target_and_rate |
| UTB_Attitude::reset_yaw_target_and_rate | libraries/AP_UTB/UTB_Attitude.cpp:847 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1357 AC_AttitudeControl::reset_yaw_target_and_rate |
| UTB_Attitude::inertial_frame_reset | libraries/AP_UTB/UTB_Attitude.cpp:866 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1395 AC_AttitudeControl::inertial_frame_reset |
| UTB_Attitude::euler_derivative_to_body | libraries/AP_UTB/UTB_Attitude.cpp:883 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1428 AC_AttitudeControl::euler_derivative_to_body |
| UTB_Attitude::body_to_euler_derivative | libraries/AP_UTB/UTB_Attitude.cpp:903 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1448 AC_AttitudeControl::body_to_euler_derivative |
| UTB_Attitude::update_ang_vel_target_from_att_error | libraries/AP_UTB/UTB_Attitude.cpp:925 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1470 AC_AttitudeControl::update_ang_vel_target_from_att_error |
| UTB_Attitude::lean_angle_max_rad | libraries/AP_UTB/UTB_Attitude.cpp:957 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1544 AC_AttitudeControl::lean_angle_max_rad |
| UTB_Attitude::pre_arm_checks | libraries/AP_UTB/UTB_Attitude.cpp:962 | AP copied body (normalized); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:1594 AC_AttitudeControl::pre_arm_checks |
| UTB_Attitude::Write_ANG | libraries/AP_UTB/UTB_Attitude.cpp:1045 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_Attitude::Write_Rate | libraries/AP_UTB/UTB_Attitude.cpp:1064 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_Attitude::shadow_history | libraries/AP_UTB/UTB_Attitude.cpp:1121 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_Attitude::shadow_history | libraries/AP_UTB/UTB_Attitude.cpp:1136 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_Attitude::shadow_config | libraries/AP_UTB/UTB_Attitude.cpp:1149 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_Attitude::input_rate_bf_roll_pitch_yaw_rads | libraries/AP_UTB/UTB_Attitude.cpp:1161 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:645 AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_rads |
| UTB_Attitude::relax_attitude_controllers | libraries/AP_UTB/UTB_Attitude.cpp:1196 | AP adapted (see native-body-diffs); libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:247 AC_AttitudeControl::relax_attitude_controllers |
| UTB_RateShadow::start | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:78 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_RateShadow::same_history | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:89 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_RateShadow::process | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:95 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::AP_UTB_NativeShadow | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:264 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::init | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:269 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::freeze | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:273 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::prepare | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:278 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::before | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:319 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::pre_axis | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:325 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::post_axis | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:329 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::complete | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:333 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::consume | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:339 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::finish_motor_audit | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:354 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::finish_unsupported | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:372 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_NativeShadow::dump | libraries/AP_UTB/AP_UTB_NativeShadow.cpp:383 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_AttitudeShadowMath::execute | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:15 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_AttitudeShadowEngine::values | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:34 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_AttitudeShadowEngine::same_history | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:67 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_AttitudeShadowEngine::start | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:113 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_AttitudeShadowEngine::process | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:117 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::AP_UTB_AttitudeShadow | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:222 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::init | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:226 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::freeze | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:231 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::prepare | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:236 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::enabled | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:262 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::before | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:268 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::after | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:287 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::audit | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:299 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::capture_time | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:306 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::finish | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:317 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_AttitudeShadow::dump | libraries/AP_UTB/AP_UTB_AttitudeShadow.cpp:340 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::start | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:30 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::begin | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:35 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::attitude | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:46 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::rate | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:64 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::end | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:131 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::outputs_equal | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:138 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::audit | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:149 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| UTB_ChainShadowEngine::failed | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:153 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::AP_UTB_ChainShadow | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:184 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::init | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:188 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::freeze | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:196 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::enabled | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:206 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::prepare | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:210 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::before_rate | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:271 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::post_axis | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:280 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::complete_rate | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:284 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::before_attitude | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:295 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::after_attitude | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:307 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::capture_time | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:320 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::finish | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:328 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::dump_flight | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:359 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_ChainShadow::dump | libraries/AP_UTB/AP_UTB_ChainShadow.cpp:418 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::AP_UTB_FlightShadow | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:62 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::init | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:66 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::start | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:70 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::stop | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:83 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::context | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:90 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::record | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:102 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::oldest | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:170 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::dump_header | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:174 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |
| AP_UTB_FlightShadow::dump_page | libraries/AP_UTB/AP_UTB_FlightShadow.cpp:186 | UTB integration/adapter; no same-name AP analogue; capture references: AC_AttitudeControl_Multi::rate_controller_run_dt / AC_AttitudeControl::attitude_shadow_compare |

## Git a evidence

HEAD/větev nezměněné, žádnáprodukčnízdrojováoprava/newbuild/upload/instalace/commit. Nové pouze tento report, HWreport a ignorovanéartefakty. Zdrojovéhash,132funkčníinventář/diff/helpermapa, nových35parsertestů a novýARMEDSITL v `tmp/utb-flight-readiness-20261025_review-01`. Celáevidence má vlastníSHAmanifest na koncipráce, odděleněodHWseal.

## Závěrečné odpovědi k celému zadání

| # | Otázka | Odpověď |
|---|---|---|
| 1 | Jak dopadla HW-2B? | PASS, 4000 Timing/Execution kroků a 4000 Flight pokusů; 84 metrik s nulovou max. odchylkou i RMSE. |
| 2 | Posun začátku native motor outputu? | M1 mean +91,198750 µs, rozdíl pozorovaných maxim +73 µs. |
| 3 | Posun návratu motor-output funkce? | M3 mean +90,704750 µs, rozdíl pozorovaných maxim +72 µs. |
| 4 | Délka motorového volání? | M2 mean 31,993500→31,499500 µs; max41 µs v obou bootech. Není to elektrické dokončení DShot. |
| 5 | Skutečný UTB consume čas? | Execution 3999 spanů: min6 / mean82,038760 / max95 µs; zahrnuje první warmup. Flight interní Rate bracket má jiné hranice, mean249942/3999=62,501125 µs. |
| 6 | CPU a scheduler? | CPU active A19,111765→B28,800000 %; Main max2516 a Rate2519 µs, M4 max1637 µs. Neizolovaná pozorování dvou bootů; PM/UBST v tomto RAM-only profilu neověřené. |
| 7 | Numerické kontroly? | Všech84 PASS s max/RMSE0; bez mismatch/desync/invalid/unsupported. |
| 8 | Motorové hodnoty? | 12000 skutečných HW comparator checks, motor_changes0; nezávislý raw TX audit bez ARM/motorových příkazů. |
| 9 | Převzaté a vlastní PID části? | UTB_PID update_all/update_i/FF/LPF/IMAX/anti-windup jsou AP kopie; vlastní jsou value adaptéry, historie import/replay, comparator, propojení a finite RAM recorder. Konkrétní funkce a adaptace v tabulkách/CSV. |
| 10 | Numerické rozdíly při ARMED? | Nový ARMED SITL FF profil má84×max/RMSE0, 11999paired; fyzický ARMED scénář nebyl povolen ani proveden. |
| 11 | ARMED Flight podpora? | Implementovaná pro podporovaný native STABILIZE a omezené okno; SITL důkazy ano. Není tím kvalifikovaný fyzický let ani logger. |
| 12 | Umíme nyní UTB3A do palubního BIN? | Ne. Současný FlightV1 je RAM-only; starý UTBC nepatří této matematické větvi. |
| 13 | Nejmenší změna? | Opt-in decimovaný snapshot do bounded fronty, reuse workeru a původní AP_Logger, verze UFPI/UFRS/UFLS + parser. Návrh níže, zatím bez implementace/uploadu. |
| 14 | Dopad loggeru? | Návrhově≤1536 B nové queue RAM bez nového stacku, 3467 B/s při10Hz či6887 B/s při20Hz. CPU, image delta a BLOCK write/lock dopad nového loggeru dosud neměřené. |
| 15 | Co brání prvnímu native letu? | Skutečné RC/disarm/failsafe, mapping/směry motorů a stabilizace, LiPo/napěťové prahy/reakce, yaw/EKF pro zvolený mód a určení letového FW/logger profilu. |
| 16 | Co navíc brání pasivnímu BIN letu? | Nový logger chybí; vyžaduje implementaci, metadata/unit/SITL/build a samostatně schválenou H743 kvalifikaci zápisu/kapacity/časového dopadu. |
| 17 | Regenerovatelná část projektu? | Přesný návrh 88835 .o/.d +1430 .pyc; ne celé tmp. Většinu obsahu tvoří unikátní měření. |
| 18 | Kolik lze uvolnit? | Po schválení přesného manifestu 2038775808 alokovaných B≈1,90GiB. Dalších48140570624 B≈44,83GiB jen po ověřené archivaci a samostatném rozhodnutí, ne prostým smazáním. |
| 19 | Co nesmazat? | Zdroje/Git/submoduly, current a recovery firmware, manifesty/patche/reporty, 4 původní BIN archivy, frozen A/B/Flight/Timing/Execution a unikátní data bez ověřeného archivu. |
| 20 | Nejkratší cesta k letu? | Dokončit skutečný FLIGHT-PREP-A, pak samostatně schválený LiPo/bezvrtulový FLIGHT-PREP-B a konkrétní motor mapping/direction kontrolu. Poté samostatně povolit krátký native-only FLIGHT-1; pasivní BIN až po kvalifikaci loggeru, aktivní UTB zůstává zakázané. |

## Aktualizace úklidu – 26. 10. 2026

Samostatně schválený přesný úklid dokončen:90265 objektových/dependency/bytecode souborů, 1.899GiB uvolněných bloků, žádné jiné soubory smazané. Kontrola7689 chráněnýchsouborů/Git/firmware/HW evidence PASS. Návrh archivace44,83GiB a VHDXkompakce nebyly provedené. Stav „po schválení lze uvolnit“ v odpovědi18 je tímto aktualizovaný na skutečné provedení; podrobnosti vUTB_STORAGE_AUDIT_CS.md. Původní readiness report je zachovaný v `tmp/utb-storage-cleanup-20261026_approved-01/readiness-report-before.md` s původnímhash. Priorita zůstává FLIGHT-PREP-A/B a první native-onlylet; žádný fyzickýlet ani novýlogger tímto krokem nevznikl.
