# FÁZE 3D-2 – první otevření BLOCK loggeru na H743

Datum: 20. 10. 2026. HEAD `070c189cdfa8357307ee1aebe8f3677cd849d520`,
větev `test/skystars-5inch`. **Problém není vyřešen.** Studený pokus A
znovu zachytil dlouhý cyklus, proto byly další experimenty zastaveny.
Regulační matematika ani firmware se neměnily.

**Doplnění 21. 10. 2026:** připravena a lokálně ověřena samostatná RAM sonda.
H743 ON/OFF i H743 bez loggeru sestaveny; nový firmware nebyl nahrán.
Příčina původní prodlevy dosud není opravená. Podrobnosti implementace
jsou v [popisu sondy](UTB_LOGGER_TRACE_CS.md) a závěrečném doplnění níže.

## Provedené měření a důkazy

[Artefakty](../tmp/utb-3d2-20261020_163314/):
preflight a záloha 1212 parametrů před restartem i po něm;
[výsledek A](../tmp/utb-3d2-20261020_163314/A-cold/result.json),
[telemetrie](../tmp/utb-3d2-20261020_163314/A-cold/telemetry.jsonl),
[analýza](../tmp/utb-3d2-20261020_163314/A-cold/analysis.json),
[metriky](../tmp/utb-3d2-20261020_163314/A-cold/metrics.json).
PM/UBST/UBMT/UBHI/DSF byly exportovány také jako CSV.

- COM5, board 1075, USB serial `200024001251303433333336`, firmware
  Copter 4.7.1/custom `3949c920`, DISARMED/STABILIZE.
- Obsluha potvrdila fyzické odpojení/zapojení USB. BOOTCNT 85 → 86;
  během měření ani downloadu nebyl zaznamenán další restart.
- UB3_RUN=0, UTB_SHADOW=0, FSTRATE_ENABLE=0, SCHED_OPTIONS=0,
  LOG_BACKEND_TYPE=4, SCHED_LOOP_RATE=400.
- Prvních 30 s baseline bez logování, potom 30 s LOG_DISARMED=1.
  Jediné PARAM_SET byly LOG_DISARMED 1 a obnovení 0.
  Jejich host časy jsou 37,141 a 67,250 s od zahájení měřicího skriptu.
- Původní logy č. 1/2: 2 516 746 / 7 924 746 B, stejné velikosti před/po.
  Celkem 10,44 MB před pokusem; krátký pokus měl rezervu proti kapacitě
  16MiB flash i po zohlednění hlaviček stránek. Žádný erase/wrap starých logů
  nebyl vyžádán. Kontrola velikostí není nové úplné hash ověření starých logů.
- Nový log č. 3: **718 496 B**,
  SHA-256 `0d9645be0432a6c4f0cb0cb5952bcfa47e68603635d8ad48a06ba931d7a0c006`.
  [Původní BIN](../tmp/utb-3d2-20261020_163314/A-cold/H743_LOGGER_A.bin).
  Download až po měření, 900B žádosti s 50ms mezerami, 799 žádostí,
  119,594 s včetně kontrol, bez opakování a bez zjištěné poruchy health.

## Naměřené výsledky

| PM export boot time [s] | NL | NLon | MaxT [µs] | PM Load [%] |
| --- | --- | --- | --- | --- |
| 135,365358 | 4000 | **1** | **5993** | 17,8 |
| 145,365323 | 4000 | 0 | 2545 | 17,9 |
| 155,365372 | 4000 | 0 | 2530 | 17,8 |

První PM okno přibližně 125,365–135,365 s překrývá konec no-log baseline
a první zapnutí loggeru (telemetrické boot časy okolo 128–129 s).
Přesný začátek PM okna ani timestamp dlouhého cyklu uložen není.
První UBHI v 129,502573 s již má všechny tři kumulativní události
nejvyššího jitter binu; do 158,546386 s tento počet neroste.
Kumulativní maximum 23127 µs je už v prvním snapshotu a nelze je připsat
tomuto zapnutí loggeru; obsahuje předchozí běh FC od startu.

Rozdíly UBMT/UBST mezi prvním a posledním snapshotem:

| Veličina | Hodnota |
| --- | --- |
| Interval snapshotů | 29,043813 s |
| Nové main cykly | 11617 |
| Průměrná perioda | 2500,000086 µs |
| Průměrný absolutní jitter | 2,677197 µs |
| UBST8 task-budget overruns | 336, tedy 11,5687/s |
| UBST9 perioda nad nominálem | 3761; není počet ztracených cyklů |
| MAIN write_sem acquire max | 58 µs (kumulativní maximum) |
| Worker write_sem hold max | 15 µs (kumulativní maximum) |
| Potvrzené main/worker kolize delta | 0 |
| UNKNOWN attribution delta | 14025 |
| DSF.Dp, PM.InE/ErC a ERR | 0, 0/0, žádný ERR |

Delta okno začíná až po otevření loggeru, takže jeho průměr/jitter
**neobsahují důkaz o velikosti samotné úvodní špičky**. SYS_STATUS mean Load
baseline 17,8 %, logování 18,1931 %; neplést s jinými intervaly PM Load.
Rozdíl přibližně 0,39 procentního bodu patří běžnému logování s existující
instrumentací a telemetrií, nikoliv novému native SHADOW výpočtu.

Dlouhý cyklus byl rozpoznán z prvního PM při čtení již staženého prefixu.
Do té doby byl pokus A dokončen a LOG_DISARMED obnovený na 0.
Následovalo pouze dokončení důkazního downloadu a závěrečné kontroly.
**Warm přepnutí a varianta B se SCHED_OPTIONS bit0 neproběhly.**
Nedostaly PASS; zastavení má přednost před dokončením celé matice.

## Co je prokázáno a co chybí

Prokázána opakovatelnost dlouhé periody ve fázi okolo prvního povolení loggeru
i bez spuštění native benchmarku. Dvě následující úplná PM okna nemají dlouhý
cyklus. Není prokázáno, že událost vzniká jen při otevření nebo že nemůže
nastat během sektorového zápisu/erase či obnovení logování.

58µs write_sem acquire nevysvětluje 5993µs periodu samo o sobě. Nejde však
o důkaz neviny loggeru: chip `sem`, SPI `dev_sem` ani WaitReady jednotlivě
instrumentované nejsou. Nulové potvrzené kolize při velkém UNKNOWN
neznamenají nulové mutexové riziko.

Per-task ID chybí: v A byl SCHED_OPTIONS=0, B byl zastaven. Navíc
[PerfInfo.cpp](../libraries/AP_Scheduler/PerfInfo.cpp):25–35 maže TaskInfo
při každém PM resetu. Původní návrh číst tasks.txt až po 30 s by mohl
ztratit konkrétní úlohu i v B; byl proto v návrhu 3D upřesněn.

Konkrétní místo stále nelze rozlišit mezi:

- synchronním start_new_log/find_last_page/StartRead;
- čekáním nebo preempcí pod chip/SPI zámkem a JEDEC WaitReady;
- obsluhou PARAM_SET/uložení parametru či jinou scheduler úlohou.

Zdrojové cesty: [AP_Logger_Backend.cpp](../libraries/AP_Logger/AP_Logger_Backend.cpp):330–344,442–468;
[AP_Logger_Block.cpp](../libraries/AP_Logger/AP_Logger_Block.cpp):545–595,669–711,836–929;
[AP_Logger_Flash_JEDEC.cpp](../libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp):92–107,224–248,324–329.
Binární hledání poslední stránky skutečně provádí opakované StartRead
v hlavním vlákně pod `sem`; samotný zdroj ale nezměří dobu těchto operací.

## Nejmenší další diagnostika a rozhodnutí o opravě

Doporučený další krok je **samostatná RAM-only časová sonda**, nikoliv
opakování stejného pokusu bez lepšího zachycení. Návrh:

1. Default-OFF compile guard a pevné záznamy oddělené podle vlákna.
   V main záznamu cycle Seq, task ID, TimeUS a několik nested spanů;
   v IO vlastním záznamu flash operace a vlastnictví mutexu.
   Žádný společný nechráněný zapisovaný slot mezi main a IO.
2. Zachytit enter/exit start_new_log, find_last_page, PageToBuffer cache hit/miss,
   WaitReady (počet Busy iterací), sem/dev_sem wait/hold a PARAM_SET obsluhu.
   Dnešní write_sem metriku zachovat, neslučovat ji s ostatními zámky.
3. Uchovat alespoň předchozí a aktuální main cyklus, zmrazit první cyklus
   nad existujícím PM prahem 3000 µs a relevantní IO stopy.
   Trigger je diagnostický; nemění scheduler limity ani AP běh.
4. AP_HAL monotónní hardware čas, žádné logování, text, heap, nové mutexy
   nebo čekání při capture. OFF/ON kontrola vlastní režie, overflow příznak,
   nedokončené spany a preempci nesměšovat se samotnou prací na SPI.
5. Export až po freeze/ukončení okna, s jasnou oddělenou režií. Původní
   průběžné logování během okna zachovat, jinak by se podmínky změnily.

Předpokládané soubory sondy: `AP_Logger_UTBBench.h/.cpp` pro pevná data,
`AP_Logger_Block.cpp`, `AP_Logger_Flash_JEDEC.cpp`, `AP_Scheduler.cpp`;
pro PARAM_SET skutečná obsluha v `GCS_MAVLink` po cíleném auditu její cesty.
Přesný manifest a cenu sondy určit při přípravě patche.
Unit testy musí ověřit nesting, časový wrap, freeze, overflow a publikaci
thread snapshotů; SITL reset/metadata/export a compile OFF. SITL neměří SPI WCET.

Podmíněný návrh minimální opravy: pokud stopa potvrdí právě synchronní
hledání/čtení při prvním otevření, připravit log ve zdravém DISARMED stavu
mimo pravidelnou flight smyčku. Připravenost musí být explicitní a zneplatněná
po rotaci/erase/recovery; nestačí naivně cachovat poslední stránku.
Pokud čtení nelze dokončit bez dlouhého čekání, potřebuje příprava rozdělení
na omezené kroky nebo původní IO kontext s bezpečným předáním výsledku.
Nevkládat stejnou blokaci pouze do jiné běžné scheduler úlohy.

To by odstranilo konkrétní first-open cestu z flight kritického okamžiku,
ale ne automaticky průběžné sektorové a mutexové blokace. Musí následovat
boundary/recovery testy. Při prokázaném WaitReady nebo mutexovém původu
volit opravu podle konkrétního držitele a operace. Bez span důkazu
**nebyl vytvořen spekulativní opravný firmware patch**.

Zdrojový kód ani regulátory změněny nebyly, unit/SITL/build nebyly spuštěny.
Offline analyzátor skutečného BIN úspěšně vytvořil CSV/JSON. Žádná instalace,
upload, ARM, motorové testy, erase, commit nebo push.

## Konečný stav

Po downloadu ověřeno UB3_RUN=0 a LOG_DISARMED=0, DISARMED, EKF attitude
dostupná a senzory/logger zdravé; port byl uzavřen. SCHED_OPTIONS se neměnil
a po pokusu byl přečten jako 0. Následná úplná závěrečná kontrola se
nespustila: Windows už COM5 nenabízelo. Obsluha následně potvrdila úmyslné
odpojení USB a opětovné připojení; tato původní nedostupnost tedy není důkazem
poruchy firmware.

Následná read-only kontrola
([data](../tmp/utb-3d2-20261020_163314/final-reconnected-191726/preflight.json),
[porovnání parametrů](../tmp/utb-3d2-20261020_163314/final-reconnected-191726/parameter-comparison.json))
znovu identifikovala stejnou desku a firmware, DISARMED/STABILIZE, EKF flags=167
a zdravé gyro/accel/AHRS/logger. Přečetla všech 1212 parametrů:
UB3_RUN=0, LOG_DISARMED=0, SCHED_OPTIONS=0 a UTB_SHADOW=0.
Proti začátku pokusu A se liší šest gyro offsetů, dvě gyro kalibrační teploty,
BARO1_GND_PRESS, STAT_BOOTCNT (86→87) a STAT_RUNTIME. Jde o položky spojené
se startovní kalibrací a statistikami; žádné PARAM_SET nebylo při této
kontrole odesláno. Ostatní přečtené hodnoty jsou shodné.

Po přibližně 22 s však čtení skončilo Windows sériovou chybou
`ClearCommError / PermissionError(13), winerror 22`. Celé plánované
pozorovací okno proto neprošlo. Příčina tohoto nového přerušení spojení
není potvrzená; nelze je automaticky připsat firmware ani předchozímu
úmyslnému odpojení. Port byl uzavřen, bez opakování experimentu nebo zápisů.

Obsluha poté výslovně potvrdila, že během této poslední kontroly USB
neodpojovala. Druhé přerušení je tedy neobjasněná závada komunikační cesty;
z dostupných dat nelze rozlišit USB/kabel/Windows od problému na FC.
Samotný ClearCommError neprokazuje restart nebo poruchu firmware.

FÁZE 3D-2 je uzavřena jako **měření zastavené na předepsané stop podmínce,
příčina neobjasněná, oprava neprovedená**. Varianta B a warm porovnání
zůstávají neprovedené. Report a návrh cílené RAM instrumentace jsou hotové;
celá měřicí matice ani odstranění blokace nejsou dokončené.
Před dalším HW experimentem je nutné ověřit stabilitu spojení a připravit
lepší záznam blokující operace. Tímto se nepokračuje k 400Hz SHADOW.

Před 400Hz RAM-only SHADOW nejdříve vyhodnotit RAM span měření loggeru.
Dosavadní výsledek prokazuje opakování problému, nikoliv jeho opravu.

## Opakovaný pokus o spojení

Na další pokyn byl 20. 10. 2026 proveden nový read-only preflight:
[důkazy](../tmp/utb-3d2-repeat-20261020_194621/preflight.json).
USB identita opět souhlasila a byl přijat DISARMED heartbeat ze sysid/compid 1/1.
Už po 0,547 s ale následoval stejný ClearCommError / winerror 22.
Pokus skončil v první health fázi, před PARAM_REQUEST_LIST; přijato 0 parametrů.
Nebyly odeslány PARAM_SET, požadavky na logy, reboot nebo motorové příkazy.
Port byl uzavřen. Měření A/B nebylo znovu spuštěno.

[Windows PnP](../tmp/utb-3d2-repeat-20261020_194621/windows-devices.json)
následně stále uvádělo ArduPilot (COM5), správné USB ID/serial, službu usbser,
Status=OK a ConfigManagerErrorCode=0. To dokládá přítomnost zařízení, nikoliv
funkčnost sériového přenosu. Ve vybraných System událostech Kernel-PnP/USB/
DriverFrameworks za posledních 30 minut nebyl nalezen vysvětlující záznam;
nejde o úplné vyloučení chyby USB.

Opakování potvrzuje aktuální závadu komunikace, ne její příčinu. Není závislé
na spuštění benchmarku nebo novém PARAM_SET; bez měření nelze rozhodnout
mezi kabelem/USB portem/usbser a FC. Další krok: fyzicky ověřit datový kabel
a přímý USB port, potom zopakovat pouze read-only stabilitu spojení.
Teprve stabilní spojení dovolí pokračovat logger experimentem.

Další potvrzené připojení bylo ověřeno v
[recheck 19:49](../tmp/utb-3d2-recheck-20261020_194930/preflight.json).
Identita COM5 opět souhlasila, ale read-only health fáze skončila po 2,625 s
stejným ClearCommError, před čtením parametrů. Následný pokus o čistě pasivní
15s příjem se vůbec nemohl otevřít: očekávané zařízení již nebylo v seznamu.
Výpis Win32_SerialPort potom ukázal pouze COM6/COM7 Bluetooth
([výpis](../tmp/utb-3d2-recheck-20261020_194930/ports-after.json)).
Nelze tedy hodnotit pasivní stabilitu; doložen je výpadek enumerace USB zařízení,
nikoliv jen timeout MAVLink. Příčina USB/FC/host zůstává nerozlišená.
Žádné PARAM_SET, otevření loggeru, nový benchmark ani restart nebyly provedeny.

Při dalším pokusu po oznámení funkčního spojení v Mission Planneru nezačalo
ani otevření sériového portu: pyserial i Windows Get-PnpDevice -PresentOnly
uváděly pouze Bluetooth COM6/COM7. Očekávané USB VID_1209/serial FC ani STM32
VID_0483 nebyly přítomné. Pasivní test proto nebyl proveden a nebyla odeslána
žádná zpráva. Tento pokus nehodnotí chování Mission Planneru; je třeba sladit
aktuální připojení a port s obsluhou.


## Prověření přístupu a nezávislého klienta

Proces běžel jako jirka-nt\jirka ve Windows session 1, stejně jako běžící
MissionPlanner. Přímé otevření COM5 bez kontroly enumerace v Pythonu prošlo,
ale první čtení skončilo ClearCommError. Nejde o doložené odmítnutí Open
z důvodu přístupových práv; příčinu přesto nelze určit jen podle typu výjimky.

Při následném 20s pozorování se v seznamu opakovaně objevily dvě položky COM5.
Registry SERIALCOMM skutečně obsahovalo současně Device/USBSER000 → COM5
 a Device/USBSER001 → COM5. PnP přitom ukazovalo jednu očekávanou ArduPilot
identitu. Jde o konkrétní nejednoznačnost mapování, nikoliv důkaz její příčiny.
Registry ani ovladač nebyly měněny.

Nezávislý .NET SerialPort s DTR/RTS=false a bez odeslání dat otevřel port
 a přijal 4384 B; po 0,554 s však Read skončil OperationCanceledException.
Offline dekódování našlo platné MAVLink zprávy včetně DISARMED heartbeatů.
Rozsah boot timestamps přes 2 s při příjmu kratším než 1 s ukazuje bufferovaná
 data, nikoliv důkaz stabilního živého spojení. Jedna BAD_DATA položka při
připojení uprostřed proudu sama neprokazuje poškození linky.

Tím se nepotvrdila stabilita alternativního klienta ani čistě Python chyba.
Další nejméně invazivní kontrola: úplně ukončit běžící Mission Planner (pouhé
Disconnect nezavírá proces) a znovu ověřit mapování a pasivní .NET příjem.
To pouze izoluje možný souběh klientů; Mission Planner zatím není prokázaný
viník. Žádný firmware nebo logger experiment nebyl spuštěn.

Důkazy: [příjem a dekódování](../tmp/utb-3d2-access-20261020_195847/dotnet-decoded.json),
[.NET výsledek](../tmp/utb-3d2-access-20261020_195847/dotnet-present-result.json),
[časový vývoj portů](../tmp/utb-3d2-access-20261020_195847/port-presence.json).


## Kontrola po úplném ukončení Mission Planneru

Před pokusem ověřeno 0 procesů MissionPlanner a jediné mapování USBSER000
na COM5. Přesto pasivní .NET příjem (DTR/RTS=false, ReadTimeout=250 ms)
po otevření a přijetí 4388 B skončil po 0,389 s OperationCanceledException.
Port byl následně ve Windows stále viditelný. Offline data obsahují platné
DISARMED heartbeaty a boot timestamps 385414–393211 ms; bufferovaný proud
neprokazuje stabilitu živého příjmu po celou tuto dobu.

Druhý omezený pasivní test s ReadTimeout=2000 ms a DTR/RTS=true přijal
5756 B, po 0,164 s zaznamenal zrušení čtení a při další operaci hlásil
„The port is closed“. Skončil po 0,294 s. Nešlo o dosažení 2s timeoutu.
Žádný MAVLink payload, PARAM_SET, restart ani motorový příkaz nebyl odeslán.

Tím je v tomto pokusu vyloučen souběh s běžícím Mission Plannerem jako
nutná podmínka závady; samotná duplicita COM5 rovněž není nutná.
Příčina se stále nedá přiřadit k firmware, USB ovladači nebo prostředí klienta.
Otevření uspělo, takže nejde o doložené odmítnutí přístupových práv k portu.
PnP uvádí ProblemCode=0, ovladač verze 2.0.0.10 / oem118.inf, USB port
Port_#0002.Hub_#0001 a LastArrivalDate 20. 10. 2026 20:04:53.
Tyto údaje samy neprokazují závadu ovladače. Ovladač ani registry se neměnily.

Důkazy: [stav před pokusem](../tmp/utb-3d2-netcheck-20261020_200402/before.json),
[první příjem](../tmp/utb-3d2-netcheck-20261020_200402/result.json),
[delší timeout](../tmp/utb-3d2-netcheck-20261020_200402/timeout-probe.json),
[PnP](../tmp/utb-3d2-netcheck-20261020_200402/device-properties.json).
Logger experiment zůstává zastavený, dokud nebude stabilní komunikační cesta.


## Úspěšné ověření spojení 21. 10. 2026

Další pasivní pokus dokončil 15 s příjmu bez chyby (14 heartbeatů).
Následující aktuální read-only kontrola dokončila 36,219 s bez sériové chyby
 a přečetla 1212/1212 parametrů. Ověřena deska 1075, USB serial
200024001251303433333336, firmware 4.7.1/3949c920, DISARMED/STABILIZE,
zdravé gyro/accel/AHRS/logger ve všech zachycených SYS_STATUS a EKF flags=167
v posledním záznamu. Diagnostické parametry byly přečteny beze změny.
Port byl uzavřen; žádné PARAM_SET, log operace, firmware upload ani motorové
příkazy nebyly provedeny. Komunikace je v tomto pozorování funkční;
příčina předchozích výpadků tím není určená a logger prodleva zůstává nevyřešená.

[Parametry a výsledek](../tmp/utb-com5-verify-20261021_070140/preflight.json),
[telemetrie](../tmp/utb-com5-verify-20261021_070140/preflight-messages.jsonl).


## Lokální diagnostický patch – výsledek 21. 10. 2026

Nový default-OFF `AP_UTB_LOGGER_TRACE_ENABLED` zachytává main-thread spany,
native task-budget overruns a první dlouhou periodu do pevné RAM.
Export po freeze přes read-only `@SYS/utb_logger.txt`; žádný nový DataFlash zápis,
thread, mutex nebo heap v capture. Stávající logger policy a pořadí zámků se nemění.
**Jde o diagnostiku, nikoliv odstranění původního čekání.**

### Přesný manifest tohoto patche

| Soubor | Zásah |
| --- | --- |
| libraries/AP_Logger/AP_Logger_UTBTrace_Recorder.h | Nový pevný ring, souhrny, freeze/publication a armed abort |
| libraries/AP_Logger/AP_Logger_UTBTrace.h | Nový default-OFF přepínač a scoped API |
| libraries/AP_Logger/AP_Logger_UTBTrace.cpp | Nová vazba na AP_HAL hardware čas a neměnný textový export |
| libraries/AP_Logger/tests/test_utb_logger_trace.cpp | Šest nových deterministických testů |
| libraries/AP_Logger/AP_Logger_Block.cpp | Podmíněné start/find/chip/write mutex spany |
| libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp | Podmíněné PageToBuffer, WaitReady a SPI spany včetně status read |
| libraries/AP_Scheduler/AP_Scheduler.cpp | Period trigger a původní čas/ID překračující úlohy bez nového clock read |
| libraries/GCS_MAVLink/GCS_Param.cpp | Podmíněný span main PARAM_SET handleru |
| libraries/AP_Filesystem/AP_Filesystem_Sys.cpp | Podmíněný read-only systémový soubor |
| Tools/scripts/build_options.py | Samostatná build option, default 0 |
| docs/UTB_LOGGER_TRACE_CS.md | Nový protokol, meze a HW postup |
| docs/UTB_PHASE_3D2_LOGGER_CS.md | Toto doplnění stavu a výsledků |

Původní rozpracované docs/UTB_PHASE_3B_CS.md, docs/UTB_PROJECT_STATUS_CS.md,
docs/UTB_PHASE_3C_CS.md a kapitoly/ zachovány. PID, Rate, Attitude, AP_Motors,
HAL/DShot, arming/failsafe a FSTRATE pravidla beze změny.
Checkpoint před implementací zahrnuje 7625 souborů včetně neignorovaných
untracked souborů; manifest a source archive jsou ve složce experimentu.

### Ověření

| Kontrola | Výsledek |
| --- | --- |
| SITL build TRACE ON / OFF | PASS |
| SkystarsH7HD-bdshot TRACE ON / OFF | PASS |
| H743 TRACE ON a Logging=0 | PASS |
| Nové unit testy | 6/6 ON i OFF |
| Původní native AP/UTB oracle | 5/5 ON i OFF; žádné vykazování prázdné suite jako PASS |
| Původní native benchmark unit | 1/1 ON i OFF |
| SITL DISARMED porovnání | 2000 kompletních validních sad, max chyba i RMSE 0, bez chybějících/částečných sad |
| SITL logging suppression | PASS, zastavení experimentu a žádný automatický restart; nejde o kompletní 2000sadový běh |
| SITL export nového @SYS souboru | PASS přes MAVFTP; není to HW ověření JEDEC backendu |
| Publikace snapshotu | 200 souběžných producer/reader iterací s ASan/UBSan bez nálezu; nejde o TSAN ani HW test |
| Compile-out | OFF ELFy nemají UTBLoggerTrace symboly; ON i NoLogging je mají |
| Izolace motorů | H743 bez aktivních UTB_Core/ACRO/ANGLE SITL symbolů, původní regulační soubory shodné s checkpointem |
| Styl | astyle nových C++ souborů, flake8 build_options.py a git diff --check bez nálezu |
| Patch kontrola | git apply --reverse --check úspěšně; žádný apply/reset/staging proveden |

**Známá neúspěšná konfigurace:** SITL + Logging=0 se v této větvi nesestaví:
AC_CustomControl.cpp:190 používá AP::logger bez příslušného guardu.
Stejná chyba vznikla s TRACE ON i OFF a původní soubor je byte-for-byte shodný
s checkpointem. Nejde o prokázanou regresi sondy, chyba nebyla opravována
v nesouvisejícím controlleru. H743 nemá implicitně zapnutý tento SITL custom
controller a jeho kontrola Logging=0 prošla. Tuto SITL kombinaci nelze označit PASS.

Pevný objekt Recorder v H743 ELF má **2784 B**, další started flag **1 B**.
BIN ON 1 268 188 B versus OFF 1 266 756 B: rozdíl **1432 B**.
To jsou naměřené velikosti artefaktů, nikoliv CPU nebo stack overhead.
H743 časové náklady sondy, skutečná stack rezerva, přímá příčina prodlevy,
IO držitel zámku a chování sektorových hranic/recovery zůstávají HW neověřené.

### Artefakty a reprodukce

[Experiment a checkpoint](../tmp/utb-3d2-trace-20261021_070545/),
[patch](../tmp/utb-3d2-trace-20261021_070545/logger-trace.patch),
[source SHA/blob manifest](../tmp/utb-3d2-trace-20261021_070545/source-manifest.json),
[build/test příkazy a výsledky](../tmp/utb-3d2-trace-20261021_070545/v2-results.json),
[upload proposal](../tmp/utb-3d2-trace-20261021_070545/firmware/proposal.json).
Starší prototypy v téže složce nejsou upload kandidátem; rozhodující je
pouze firmware/ a jeho SHA256SUMS.txt.

```sh
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --disable-UTB_RATE_SITL --enable-UTB_NATIVE_BENCH --enable-UTB_BENCH --enable-UTB_LOGGER_TRACE
./waf copter -j6
# OFF: stejné příkazy s --disable-UTB_LOGGER_TRACE
# H743 compile guard: ON konfigurace navíc s --disable-Logging
```

Definitivní diagnostický APJ:
[SkystarsH7HD-bdshot_LOGGER_TRACE_ON_070c189c_c6dd953dfd05.apj](../tmp/utb-3d2-trace-20261021_070545/firmware/SkystarsH7HD-bdshot_LOGGER_TRACE_ON_070c189c_c6dd953dfd05.apj)

- Velikost: **1 144 123 B**, board ID **1075**, USB ID **1209:5741**.
- SHA-256: `c6dd953dfd05530c8849f1fa5cafb408bb6054e9e69230d8d6b43e0ab953e041`.
- APJ git_identity **070c189c**, zdrojový HEAD
  **070c189cdfa8357307ee1aebe8f3677cd849d520**, plus necommitnutý diagnostický patch.
- Dosud instalovaný FW při poslední kontrole: **4.7.1/3949c920**.
- Stav při sestavení kandidáta: APJ ještě nebyl nahrán; následný schválený upload a HW výsledek jsou níže.

Nový upload vyžaduje samostatný souhlas podle zadání 3D-2. Potom je nejmenší
krok nový fyzický start potvrzený obsluhou a jediný omezený DISARMED first-open
pokus podle popisu sondy, s kontrolou kapacity a obnovením parametrů.
Při předčasném freeze nebo poruše zastavit a analyzovat capture; nepokračovat
bez vyhodnocení, nic nemazat a nepokračovat k 400Hz SHADOW.


## Jediný schválený HW experiment TRACE ON – 21. 10. 2026

**Výsledek: původní first-open příčina není určena a není opravena.**
RAM sonda zachytila jinou dlouhou DISARMED periodu před povolením loggeru.
Žádný další běh, restart ani úprava firmware po tomto zjištění nenásledovaly.

### Firmware, identita a kapacita

COM5: USB serial `200024001251303433333336`, board ID `1075`;
bootloader protocol 5, STM32H743/753 revision V. Před uploadem 51,36 s stabilní
read-only komunikace, 1212/1212 parametrů, DISARMED a zdravé IMU/AHRS/logger.
Nahrán pouze schválený APJ **1 144 123 B**, SHA-256
`c6dd953dfd05530c8849f1fa5cafb408bb6054e9e69230d8d6b43e0ab953e041`.
Standardní uploader identifikoval board i serial před programováním a úspěšně
ověřil aplikaci; bootloader, parametry ani logovací flash se nemazaly.

Po uploadu a po potvrzeném fyzickém restartu prvotní kontrola narazila na AHRS
před dokončením inicializace. Následné read-only pozorování ověřilo EKF flags=167
 a zdravé gyro/accel/AHRS/logger; experiment začal až poté. Žádný automatický
restart ani obcházení health kontroly. Firmware hlásí 4.7.1/070c189c,
STAT_BOOTCNT=94. Porovnání všech parametrů před/po uploadu ukazuje pouze provozní
statistiky a automaticky aktualizované gyro offsety/teploty, baro ground pressure
 a compass declination; ostatní konfigurace shodná.

Původní logy 1/2/3: 2 516 746 / 7 924 746 / 718 496 B, zachované podle seznamu
ID/velikostí. Z 16MiB geometrie, 256B stránek a 6B page headerů vycházelo před
měřením přibližně **5 283 840 B** volných stránkových dat, nad rezervou 2,5 MB
pro omezené 30s okno. Jde o výpočet z geometrie/seznamu, ne přímý free-space API.

### Skutečná časová stopa

| Událost | Čas od bootu / wall time |
| --- | --- |
| LOG_REQUEST_LIST při kontrole kapacity před měřením | okolí 132,06 s |
| GCS::update_receive, sloučené scheduler task ID 45 | začátek 132 064 932 µs, délka **23 938 µs** |
| Freeze, další hranice hlavní smyčky | **132 088 881 µs**, perioda **24 238 µs** |
| Poslední SYSTEM_TIME před odesláním LOG_DISARMED=1 | **156 545 000 µs** |
| Zmrazení před povolením loggeru | nejméně **24,456119 s** |
| LOG_DISARMED=1 -> 0 | jednou; přibližně 30 s logování |

Syrová hlavička: `UTBLoggerTraceV1,frozen,trigger=132088881,period=24238,tickets=2722,lost=0,aborted=0,exhausted=0`.

| Operace | Count v capture | Maximum wall time | Začátek maxima (µs) |
| --- | ---: | ---: | ---: |
| start_new_log | **0** | — | — |
| find_last_page | 8 | **2602 µs** | 132083558 |
| PageToBuffer (včetně cache hits) | 175 | 257 µs | 132065635 |
| WaitReady | 162 | 77 µs | 132067040 |
| chip sem wait | 8 | 1 µs | 132065072 |
| chip sem hold body | 8 | 2580 µs | 132065073 |
| SPI dev_sem wait | 324 | 24 µs | 132066730 |
| SPI dev_sem hold body | 324 | 190 µs | 132065701 |
| main PARAM_SET / write_sem wait / hold | 0 / 0 / 0 | — | — |
| scheduler task budget overrun | 1713 | **23938 µs**, task 45 | 132064932 |

Souhrny jsou od aktivace capture, nikoliv součty výhradně posledního cyklu.
Zachovaných 128 detailů má cycle=40826; jde o konec 2722 ticketů, nikoliv
kompletní historii. `lost=0` znamená nulová ztracená dokončení scope,
**nikoliv že ring nepřepsal starší dokončené záznamy**. Vnořené wall spany
nelze sčítat; obsahují preempci a režii sondy. IO držitel mutexu není měřen.

**Doložená zachycená cesta:** LOG_REQUEST_LIST -> AP_Logger::handle_log_request_list
-> get_num_logs / get_log_info -> BLOCK hledání stránek -> JEDEC čtení a SPI.
Task ID bylo ověřeno z přesného TRACE ON ELF: Copter task index 41 plus čtyři
common tasks před prioritou 102 dává sloučené ID 45, GCS::update_receive,
400 Hz, budget 180 µs. Offline mapa a disassembly jsou uložené.

Zdrojové body:
- `libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp:68-100`: synchronní
  get_num_logs a zahájení výpisu; `:259-284`: get_log_info před LOG_ENTRY.
- `libraries/AP_Logger/AP_Logger_Block.cpp:492-509`: get_num_logs volá
  find_last_page dvakrát; `:677-723`: synchronní binární hledání;
  `:727-748`, `:782-799`: hledání konců logu a čtení metadat.
- `libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp:229-259`: WaitReady,
  dev_sem a SPI přenos stránky; `:93-108`: Busy wait; `:182-196`: status read.
- `ArduCopter/Copter.cpp:213`, `libraries/AP_Vehicle/AP_Vehicle.cpp:620-630`
  a `libraries/AP_Scheduler/AP_Scheduler.cpp:201-226`: pořadí sloučených úloh.

U zachyceného 24ms cyklu je doložený synchronní výpis logů a mnoho čtení flash,
nikoliv jeden mnohamilisekundový změřený WaitReady nebo mutex acquire.
Není tím izolována čistá SPI CPU doba ani prokázáno nulové mutex riziko.
Příkazy log transferu se za armed odmítají
(`AP_Logger_MAVLinkLogTransfer.cpp:40-45`); tento DISARMED nález sám
neprokazuje stejnou cestu při letu ani bezpečnost prvního otevření logu při armování.

### Nový BIN: původní symptom přetrvává

Nový log 4: **725 746 B**, stažen celý, bez retry a bez souběhu s měřením;
SHA-256 `333d9e449d51f7a18f632ab275cafd60061357d9b07c02cf95404e0ff7e36c63`.

| PM TimeUS | MaxT | NLon | Interpretace |
| ---: | ---: | ---: | --- |
| 164926439 | **6070 µs** | **1** | 10s interval přibližně 154,926–164,926 s zahrnuje první otevření loggeru |
| 174926480 | 2541 µs | 0 | následující 10s interval |
| 184926481 | 2532 µs | 0 | následující 10s interval |

6070 µs reprodukuje rozsah původního symptomu. PM neposkytuje přesný okamžik
ani vnitřní operaci a sonda v této době už nemohla ukládat. Je doložena
časová souvislost s first-open intervalem, **nikoliv příčina 6070 µs**.
V BIN není žádný ERR, PM.InE/ErC=0 a DSF.Dp=0 ve všech 30 DSF záznamech.

Pozdější stabilní okno 157,990962–186,031043 s:
11 216 period, průměr **2499,999376 µs**, průměr absolutního jitteru
**2,763641 µs**, odvozeno rozdílem kumulativních UBMT Met=5/6.
To nevylučuje dřívější 6070µs cyklus. UBST8 delta=392, **13,97999/s**;
je to budget overrun úloh, ne počet ztracených hlavních cyklů.
SYS_STATUS CPU průměr v 10s baseline **17,89 %**, v logovacím okně
**18,2207 %**. Není to měření samostatné režie sondy: chybí kontrolní OFF
běh a sonda už před oknem zamrzla. Režie capture a stack rezerva nejsou
z těchto dat samostatně určeny.

### Cleanup, konečný stav a další krok

LOG_DISARMED obnoveno na **0**. Při pozdějším čtení `@SYS/tasks.txt` původní
AP automaticky změnil RAM SCHED_OPTIONS 0 -> 1 a vrátil pouze TasksV2;
nejde o statistiky zachyceného cyklu. Zdroj `AP_Scheduler.cpp:499-507`.
Vedlejší účinek zjištěn úplným porovnáním 1212 parametrů a **obnoven na 0**,
MAVLink potvrzeno; neměnil měřicí okno. Jméno úlohy proto ověřeno offline z ELF.

Konečná kontrola: DISARMED, EKF flags=167, gyro/accel/AHRS/logger zdravé,
bez zaznamenaného InternalError nebo restartu, COM5 uzavřen.
UB3_RUN=0, UTB_SHADOW=0, FSTRATE_ENABLE=0, LOG_DISARMED=0, SCHED_OPTIONS=0.
Neproveden ARM, motorový test, další upload, mazání logů ani Git commit/push.
Žádné další balíčky ani ovladače nebyly instalovány.

**Nejmenší další krok k samostatnému schválení:** kapacitu a úplnou zálohu
ověřit před fyzickým cold startem. Po něm již před first-open neprovádět
LOG_REQUEST_LIST/DATA ani @SYS/tasks.txt/FTP; pouze minimální identity/health
 a critical param čtení, baseline a jeden LOG_DISARMED přechod. Tím se odstraní
prokázaný předčasný trigger vyvolaný kontrolou kapacity, bez změny firmware,
prahu nebo automatického resetování sondy. Jiná baseline událost stále může
sondu zmrazit. Nový pokus nebyl proveden ani automaticky schválen.

Změna inicializace loggeru, oprava mutexů nebo nový 400Hz SHADOW zatím
nemají dostatečný důkaz z tohoto experimentu. **Problém není vyřešen.**

Důkazy:
- [Upload, preflight a zachování parametrů](../tmp/utb-3d2-hw-20261021_082016/)
- [Syrová RAM stopa](../tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/utb_logger.txt)
- [Časová stopa CSV](../tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/trace-detail.csv)
- [Souhrny CSV](../tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/trace-summary.csv)
- [BIN](../tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/H743_LOGGER_TRACE.bin)
- [Analýza JSON](../tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/analysis.json)
- [Mapa úlohy z ELF](../tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/compiled-task-map.json)
- [Obnovení parametrů a konečná health](../tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/cleanup-result.json)

Git: `070c189cdfa8357307ee1aebe8f3677cd849d520`, větev test/skystars-5inch,
index prázdný; schválených 10 zdrojových souborů sondy se proti build manifestu
nezměnilo. Zachovány všechny předchozí necommitnuté soubory. V tomto HW kroku
přibyly artefakty v tmp a aktualizovány pouze dva dokumenty tohoto logger testu.
