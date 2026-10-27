# HW-LOG-1 – upload a read-only kvalifikace Flight BIN firmware

**PASS pro jediný schválený upload a omezenou USB-only runtime-OFF kvalifikaci. GO pouze k přípravě samostatně schvalované HW-LOG-2. Žádná ARM ani zápisová BIN session neproběhla.**

Datum26.10.2026; uvedené raw časy jsou UTC, místní Europe/Prague je UTC+1. Evidence `tmp/utb-flight-bin-hwlog1-20261026/`. Aktuální stav: FC DISARMED STABILIZE, UCS_B_ENABLE0, všechny diagnostické RUN0, port zavřený, žádná LiPo/vrtule/motor test, žádné změny zdrojového firmware, build, instalace či Git historie.

## 1. Výchozí Git stav

Větev test/skystars-5inch, HEAD11a518e5857b24cc554b3cb7e6492998013ddd1a. Index prázdný,14 modified a31 untracked položek (včetně adresáře kapitoly) před touto fází. Rozpracované Flight BIN/Execution/Timing a ostatní soubory zachovány. `git-before.txt`, původní source manifest a závěrečná kontrola jsou součástí evidence.

## 2. Offline integrita

Před MAVLink prošlo všech7610 souborů finálního zdrojového manifestu a233 kontrol zapečetěných artefaktů/referencí. V tom kompletní HW-2A104 a HW-2B103 položek, jejich reporty i kritické Timing/Execution/Flight exporty. Zkontrolován původní evidence seal nové implementace, PID/Rate/Attitude zdroje, statický motor isolation audit, ELF/build metadata a runtime defaulty0/10/60. Čtyři host archivy BIN jsou znovu SHA ověřené, dohromady11885734B. Neproběhlo plošné hashování desítek GB historie. Podrobnosti `offline-audit.json`.

## 3. Přesný APJ

`tmp/utb-flight-log1-20261026/firmware/final/SkystarsH7HD-bdshot_UTB_FLIGHT_BIN_ON_11a518e5_efc95cdf5819.apj`

1162543 B; board1075; image1288460 B; SHA-256:

`efc95cdf581920d675af3c39baffa78ce227caf815a48097cabce61b9192b7ff`

APJ decomprese/velikost/board ověřeny. Embedded git11a518e5 sám neidentifikuje dirty build; APJ/hash/source manifest/CRC upload tvoří společnou evidenci. Žádný jiný kandidát ani OFF rollback nebyl použit. .bss proti OFF+1288 B, flash+3872 B, linker heap73728 B jsou build reference, nikoliv nynější runtime FREE.

## 4. Předchozí výsledky

Dříve kvalifikovaný EXEC ON APJ dae4f020db98… prošel HW-2A/B, poslední tehdy přečtený boot116. HW-2B:4000 pokusů,3999paired,84nulových metrik,12000/0 motor audits, Main maximum2516µs a Rate2519µs. Nový logger měl před dneškem pouze unit/SITL/offline důkazy (125C++/48Python;350sad10Hz a319sad20Hz). Tyto údaje nejsou dnešní aktivní flash měření.

## 5. Read-only preflight skutečné FC

Jediné preflight spojení 85.704009s. COM5 vybraná přes1209:5741 a serial200024001251303433333336, board1075. Běžela původní aplikace Copter4.7.1/11a518e5, bez UCS_B parametrů, skutečný boot118. Nárůst116→118 je starší nepozorovaná historie, není připisován chybě. Na začátku/konci uptime5676983→5757911ms, bez poklesu.85heartbeatů, největší RX/HB mezera1.030827s,1476packetů,0parser errors. Gyro/accel/baro/AHRS/logger health a EKF167 ve čtených vzorcích zdravé, InternalError/drops0. Aktuální původní firmware identifikovaný protokolem a historií, ne SHA readbackem celé flash.

Úplná záloha1220/1220 indexů0–1219; srovnání proti HW-2B vysvětlené startup/automatické hodnoty a čítače, jiná letová konfigurace beze změny. Jediný log inventory mimo pasivní pozorování odpovídal původním4souborům. Před uploadem port uzavřený.

## 6. Schvalovací brány

BránaA: „Schvaluji read-only preflight; všechny podmínky platí.“ BránaB po skutečném preflightu: „Schvaluji jeden přesně uvedený upload a read-only kvalifikaci.“ Odpovědi a okamžik jejich zaznamenání v approval-a/b.json. B zahrnovala jediný konkrétní hash, standardní bootloader entry/uploader restart a následnou read-only kvalifikaci. Nešlo o souhlas s ARM, RUN nebo zápisem logů. Žádný další souhlas/retry nebyl využit.

## 7. Fyzické podmínky

Obsluha potvrdila demontované všechny vrtule, odpojenou LiPo, pouze USB, bezpečně položenou FC, nemožnost nechtěného ARM od RC a zavřený Mission Planner/ostatní sériové aplikace. Žádný další fyzický krok po tomto potvrzení nebyl vyžádán. USB napájení a nepřipojená baterie jsou potvrzení obsluhy; elektrické kolejnice a battery-under-load se neměřily. Bateriová PreArm varování při chybějící LiPo nejsou obcházená ani považovaná za letové GO.

## 8. Upload

Použit existující Tools/scripts/uploader.py a ověřený jednorázový host wrapper s přizpůsobeným kandidátem/identitou; žádný nový upload framework. Před zápisem znovu AV/UID/board/boot118/DISARMED/RUN0 a SHA. Jeden standardní PREFLIGHT_REBOOT_SHUTDOWN param1=3, jedna identifikace bootloaderu, jedno upload(fw,force=False,boot_delay=None), no_extf=True. Bootloader rev5, board1075/rev0, serial200024001251303433333336, kapacita1703936B. Erase/Program/Verify100.0% a Rebooting v konzoli, standard_verification_passed=true, program_calls1. Bez rewrite bootloaderu, external flash image, rollbacku či druhého uploadu.

Předwrite timestamp16:10:46.144076UTC; nonce uložen16:10:46.150450. Upload result uložen16:11:30.289865, konzole16:11:30.338443. Mtime je čas zápisu host souboru, ne přesná elektrická MCU reboot hrana. Kompletní uspořádané phase/progress hlášky a rawRX11066B/TX1304408B zachované. Jednotlivé Erase/Program/Verify phase hrany nemají samostatné hardware timestampy; netvrdí se jejich dodatečně odhadnutá přesná délka. `upload-console-readable.txt` pouze zobrazuje původní NUL z bootloader OTP jako\0, původní konzole se neměnila.

CRC Verify potvrzuje programovaný image v aplikační oblasti; samo neověřuje zdravou aplikaci. Ta byla ověřena zvlášť.

## 9. USB po uploadu

Po standardním rebootu byl pasivním výčtem nalezen správný COM5/1209:5741/serial. Nebylo třeba fyzické odpojení ani další reboot; ostatní COM6/7 nebyly otevřené. Nepřetržité PnP pozorování během standardního bootloader přechodu neprobíhalo, proto se netvrdí nulová krátká reenumerace. Dřívější HW-1/HW-1R USB problém se při této post-kontrole neopakoval. Žádné automatické reconnect smyčky. Post-upload samostatné read-only otevření bylo zahrnuto v bráněB.

## 10. Nová aplikace a boot

Copter4.7.1/11a518e5, board1075 a UID2 prvních12B serial200024001251303433333336. Skutečný STAT_BOOTCNT119 na počátku i na konci. Uptime112480→209975ms bez poklesu; žádný nečekaný restart pozorován. Upload+nové parametry+původní rozhraní+nový boot společně dokládají očekávanou aplikaci, nejde o kryptografický readback celé běžící MCU flash.

## 11. Parametry

Nový PARAM_REQUEST_LIST poskytl **1221/1221 unikátních jmen a úplné indexy0–1220**, před i po pozorování. Přibylo UCS_B_ENABLE0. Děti UCS_B_RATE10 a UCS_B_SECS60 jsou při ENABLE0 skryté z enumeration; přímý PARAM_REQUEST_READ je skutečně vrátil s index65535 (validní neindexovaná odpověď). Tento mechanismus odpovídá AP_Param.cpp:1813–1820 a flagu AP_PARAM_FLAG_ENABLE v AP_UTB_FlightBin.cpp. Není to chybějící parametr nebo neúplná záloha.

Úplný indexovaný export je before/final-parameters.json/.param, plus **all-known-parameters.json/.param se1223jmény** včetně obou přímo přečtených dětí. Při kvalifikaci se ENABLE nezapínal jen pro zviditelnění.

| Parametr | Před uploadem | První nový boot |
|---|---:|---:|
| BARO1_GND_PRESS | 102269.1640625 | 102123.671875 |
| COMPASS_DEC | 0.09362976998090744 | 0.09362978488206863 |
| INS_GYR1_CALTEMP | 13.932000160217285 | 28.033000946044922 |
| INS_GYR2OFFS_X | 0.0005479450337588787 | -3.217263292754069e-05 |
| INS_GYR2OFFS_Y | -0.008908922784030437 | 0.001990931574255228 |
| INS_GYR2OFFS_Z | 0.00126131612341851 | 0.0016577440546825528 |
| INS_GYR2_CALTEMP | 13.305000305175781 | 27.334001541137695 |
| INS_GYROFFS_X | 0.0033791055902838707 | -0.010777956806123257 |
| INS_GYROFFS_Y | -6.706670683342963e-05 | -0.0017139564733952284 |
| INS_GYROFFS_Z | 0.00014458455552812666 | 0.002722077537328005 |
| STAT_BOOTCNT | 118.0 | 119.0 |
| STAT_RUNTIME | 207242.0 | 207643.0 |
| UCS_B_ENABLE | None | 0.0 |

Žádná společná native PID/ATC, frame/motor/servo/ESC, RC/failsafe, filtry, EKF/AHRS nebo bateriová konfigurace se nezměnila. Offsety/teploty gyro a baro tlak jsou původní startup kalibrace, COMPASS_DEC původní automatická deklinace; jejich source mechanismus je známý, konkrétní boot kalibrace nebyla instrumentována. Neobnovovaly se starými hodnotami. Mezi první/finální zálohou nového bootu se změnil pouze STAT_RUNTIME207643→207705. Legacy UTB_ENABLE1 se zachoval, UTB_SHADOW0 a ostatní RUN0; legacy enable není aktivní H743 řízení ani zapnutí nového BIN.

## 12. BIN logger runtime OFF

Přímo přečteno UCS_B_ENABLE0/RATE10/SECS60, UTT_RUN/UCS_F_RUN/UCS_RUN/UAS_RUN/UNS_RUN0 a zachované legacy přepínače. Sekvenčně načteny utb_timing.txt, utb_execution.txt, utb_flight.txt, utb_chain_shadow.txt, všechny `pending`. Žádný RUN ani ARM a žádná finite session se nespustila.

**Přímý nový BIN IDLE/counters export v tomto pending stavu neexistuje**: Chain dump vrací pending před bin.dump (AP_UTB_ChainShadow.cpp:461–464). IDLE je konzistentní se zdrojovým defaultem, ENABLE0 a DISARMED historií, nikoliv nezávisle přečtený čítač. Nebyl vymyšlen @SYS/utb_bin soubor. UFPI59/UFRS67/UFAT72/UFLS55B a450B/sada zůstávají source/build ověření; skutečné payloady se v HW-LOG-1 nezapisovaly.

## 13. Health a pozorování

Post spojení 101.844092s,101heartbeatů,2753packetů, RX102307B/TX2978B,0parser errors. Max HB i libovolná RX mezera1.0592624s. Po celou dobu přijaté HB DISARMED STABILIZE. Ve skutečně získaných SYS/EKF vzorcích gyro/accel/baro/AHRS/logger healthy, EKF167, InE/count/drops0. Pozorování zahrnovalo60s receive-only sekci, **během ní ale nepřišel žádný SYS_STATUS**; senzorové/CPU snapshoty jsou před/po, ne400Hz kontinuální health audit. Kritické STATUSTEXT/restart/heartbeat kontroly byly aktivní po celé spojení. Žádné alokační nebo stack chyby pozorované.

## 14. CPU runtime OFF

| Profil | SYS_STATUS N | Min/mean/max % |
|---|---:|---|
| Dnešní původní firmware |4|18.9 /19.275 /19.8|
| Dnešní nový firmware runtime OFF |4|18.3 /18.6 /19.1|
| Historický HW-2B baseline |30|18.2 /18.43 /18.8|

Aktuální4vzorky jsou on-demand při identity/memory/final dotazech, ne pravidelná60s řada. Původní intervaly−1 se neměnily. Rozdíl mean−0.675procentního bodu proti dnešnímu preflightu a+0.17proti historickému HW-2B **neprokazuje cenu/zrychlení firmware**: jiné boot/teplota/uptime, filtrovaný scheduler load a odlišné vzorkování. Přesná změna ustálené CPU baseline je INCONCLUSIVE. Žádná aktivní logger CPU ani flash throughput veličina nebyla změřena.

## 15. Stack – skutečné watermarky

Čísla jsou **pozorované free/allocated B**, nikoliv varovný threshold nebo working-area sizeof. ThreadsV2 vypisuje tuto definici runtime stacku; compile-time pracovní oblast3568B zahrnuje i TCB/alignment a neodporuje reported3504B stacku.

| Thread | Původní FW po dotazech | Nový před60s | Nový po60s |
|---|---:|---:|---:|
| ISR | 1248/1536 | 1288/1536 | 1288/1536 |
| ArduCopter | 4352/7168 | 4352/7168 | 4352/7168 |
| idle | 288/496 | 288/496 | 288/496 |
| UART_RX | 864/1200 | 864/1200 | 864/1200 |
| OTG1 | 352/752 | 352/752 | 352/752 |
| monitor | 1096/1456 | 1096/1456 | 1096/1456 |
| timer | 1544/1968 | 1552/1968 | 1552/1968 |
| rcout | 504/944 | 504/944 | 504/944 |
| rcin | 840/1456 | 912/1456 | 912/1456 |
| io | 1592/2480 | 1592/2480 | 1592/2480 |
| storage | 952/1456 | 960/1456 | 960/1456 |
| UART4 | 328/752 | 344/752 | 344/752 |
| log_io | 1584/2016 | 1528/2016 | 1528/2016 |
| I2C0 | 984/1456 | 984/1456 | 984/1456 |
| OSD | 976/1712 | 976/1712 | 976/1712 |
| led | 280/688 | 280/688 | 280/688 |
| UART1 | 328/752 | 424/752 | 424/752 |
| I2C1 | 1000/1456 | 1000/1456 | 1000/1456 |
| SPI1 | 832/1456 | 880/1456 | 880/1456 |
| SPI4 | 792/1456 | 848/1456 | 848/1456 |
| utb_log | 2448/3504 | 2408/3504 | 2408/3504 |
| FTP | 1616/2992 | 1584/2992 | 1584/2992 |

Main4352/7168, ISR1288/1536, utb_log2408/3504 a FTP1584/2992 zůstaly během nového pozorování stejné. Historické HW-2B Main4312/ISR1240/worker2456/FTP1168 jsou jiný boot s plnými aktivními exporty; lepší některý aktuální watermark není důkaz menšího worst case. Aktuální writer worker frame504B a jeho vnořené logger/driver volání nebyly aktivní, nezaměňovat s dostatečností pro zápis.

## 16. Heap/regiony

Nové před/po snapshoty přesně stejné:

| Region | TYPE/source flag | FREE/LRG před B | FREE/LRG po B |
|---|---|---:|---:|
| 0x30000000 | 8 (ETH_SAFE) | 3688/3640 | 3688/3640 |
| 0x20000000 | 2 (FAST) | 47944/47944 | 47944/47944 |
| 0x24000000 | 4 (AXI_BUS) | 524280/524280 | 524280/524280 |
| 0x00000400 | 2 (FAST) | 64504/64504 | 64504/64504 |
| 0x30040000 | 8 (ETH_SAFE) | 32760/32760 | 32760/32760 |
| 0x38000000 | 1 (DMA_OK) | 54536/53816 | 54536/53816 |

Typy podle AP_HAL_ChibiOS/hwdef/common/malloc.c:39–42, nikoliv záměna všech poolů za jeden heap. Dva MEMINFO freemem32=734840;16bit freemem=65535 je saturace. Globální odhad MEMINFO ani součet rozdílných regionů není univerzální souvislý DMA blok. Největší blok v prvním regionu3640B, dostupné další regiony mají své omezené typy. Snapshoty neprokazují přechodné minimum ani plný absence leaku, jen žádný přetrvávající pokles mezi těmito body. Nebylo pozorované selhání alokace. Runtime FREE nelze izolovaně odečítat jako .bss1288B cenu přes různou historii bootu/exportů.

## 17. utb_log worker a lifecycle

Thread existuje s PRI58 a free2408/3504B. Dále20skutečných existujících `UTB flags=1 calc=0 obs=0 policy=0` zpráv v časech3.761554→98.702118s post spojení dokládá opakovaný průchod workerem. Jediný callsite utb_status_update je v utb_log_thread (utb.cpp:297,519–544); nejde o nový BIN trigger ani novou sondu. Tento přibližně5s status dokládá postup, nikoliv měření každé1ms iterace či per-thread CPU. Žádný další thread nebo alokace nebyly hostem vytvořeny. thread_create failure path zůstala source/unit ověření, ne vyvolané HW selhání.

## 18. DataFlash

LOG_BACKEND_TYPE4 a LOG_DISARMED0 zachovány, logger health bit zdravý, errors_count3/drops0 ve čtených vzorcích. Nebyl poslán LOG_ERASE, StartNewLog ani příkaz ke spuštění nového záznamu. Zdrojová přímá hot path nového produceru bez DataFlash/mutex/heap zůstala beze změny. BLOCK W25Q128/SPI3 může při startupu provádět vlastní recovery; bez readbacku/trace nelze absolutně vyloučit změnu interního markeru nebo libovolného flash bajtu. Žádná taková závada se neprojevila.

Čtyři metadata logů zabírají11885734B. Oproti nominálním16MiB není rozdíl automaticky přesná použitelná volná kapacita: page headers, reserved sektory, geometrie/circular chování jsou další omezení. Přibližný krátký write rozpočet je v HW-LOG-2, aktuální přesná zbylá kapacita není přímo exportovaným read-only údajem. HW-LOG-1 nic nezapisovala pro její měření.

## 19. Původní log inventory

Jeden inventář před uploadem a jeden po něm, oba mimo receive-only okna; žádný nový celý BIN download.

| ID | Bytes | time_utc |
|---|---:|---:|
|1|2516746|0|
|2|7924746|0|
|3|718496|0|
|4|725746|1792564233|

num_logs4/last_log_num4 a všechny LOG_ENTRY údaje se přesně shodují. Archivy/receipts na hostu byly před uploadem ověřeny SHA. **Metadata shoda není byte-for-byte shoda flash obsahu** a nevytváří důkaz fyzického zápisu nových UF zpráv.

## 20. Motorová autorita

Původní AP je jediný vlastník AP_Motors. Ověřený source/ELF audit kandidáta má aktivní Core/Output/Acro/Angle compile-out, writer pouze hodnotové snapshots a SPSC, původní PID/Rate/Attitude/AP_Motors beze změny. Po uploadu native STABILIZE, trvale DISARMED, žádná aktivace UTB modes. V přenosu pre/post pouze read-only REQUEST_MESSAGE/param-read/list/FTP-RO/log inventory; upload navíc pouze schválený bootloader entry. Žádný ARM/servo/motor/PWM/DShot příkaz ani failsafe bypass. Toto je strukturální/protokolový důkaz pro DISARMED stav, nikoliv měření aktivních fyzických motorů nebo elektrického DShot.

## 21. Omezení a nepřepisované nejistoty

- Žádná aktivní10Hz session,50nových sad, skutečný BLOCK throughput/admission/drops nebo aktivní writer CPU/stack nebyly v této fázi testované.
- Přímý pending BIN IDLE/counters export není implementovaný;0/10/60 jsou skutečné parametry, pending a thread průchod skutečné důkazy, IDLE je source inference.
- CPU4vzorky a senzory/EKF snapshoty nepokrývají kontinuálně60s pasivní sekci. Intervaly se nezvyšovaly bez schválení.
- Skutečná Main/Rate perioda/jitter, PM.NLon/MaxT a5–6.5ms anomálie se neměřily. Logger listing byl mimo okno, UTT se nespouštěla. Příčina historických prodlev není tímto vyřešena.
- Stack je watermark runtime OFF, ne aktivního writeru; první pool má omezený souvislý blok. Bez libovolného nového prahu se kvalifikace aktivního zápisu ponechává HW-LOG-2.
- USB napájení nenahrazuje ověření LiPo, RC/disarm, bateriového monitoru, gyro za vibrací, absolutního yaw či letové platformy.
- Detailní upload progress pořadí je zachované; per-phase hardware timestampy/CRC readback celé MCU nejsou k dispozici.

## 22. Rozhodnutí

**PASS – offline integrita, jeden výslovně schválený upload s CRC verify, nový zdravý runtime-OFF boot, parametry/viditelná enumeration i hidden direct reads, worker progress, stabilní pozorované RAM/stack a zachované log metadata.** Žádný konkrétní nový firmware blocker z této read-only kvalifikace nevyšel.

**INCONCLUSIVE/neověřeno – přesná změna ustálené CPU baseline, aktivní flash/worker režie, Main/Rate timing, fyzické ARMED chování a letová způsobilost.** PASS má omezený rozsah, není to schválení zápisového testu ani letu.

## 23. HW-LOG-2 připravenost

Samostatný `UTB_FLIGHT_BIN_HWLOG2_PLAN_CS.md` doporučuje jediný normální ARM s již nahraným firmwarem, **teprve po samostatné elektrické/RC kvalifikaci, odstranění vrtulí, zajištění dronu a výslovném souhlasu s napájením/roztočením motorů**. Aktuální USB-only kvalifikace tento trigger nepovoluje a neprokazuje, že bez LiPo projde PreArm. Nastavení10Hz/5s/ENABLE1 je návrh, nikoliv nynější změna.

Pokud tyto fyzické předpoklady nelze bezpečně splnit, NO-GO proA; variantaB je teprve samostatné možné zadání test-only DISARMED triggeru se stejným engine a pravdivou identitou dat, nikoliv automatický patch. Plán odděluje native logger startup, ustálené native logování a5s UTB zapisování; stávající UTT pouze případně pro DISARMED startup, nikdy při ARMED. Zápisový test se nezahájil.

## 24. Konečný stav a Git

Poslední ověřený boot119, DISARMED STABILIZE, USB-only, runtime BIN OFF, všechny RUN0. Native letové parametry zachované, žádný PARAM_SET/změna intervalů, žádný nový hostem vyžádaný DataFlash záznam ani erase, žádný další reboot nebo upload. COM uzavřený po každém schváleném spojení. Žádný automatický reconnect/retry. Připojení v tomto protokolu: preflight, opětovná app identita v rámci jediného uploadu, standardní bootloader port a následná kvalifikace; nejde o chyby/retry smyčku.

Nově pouze tento report a HW-LOG-2 plán, host evidence v ignorovaném tmp. HEAD/větev/index zachované; žádný commit/push/reset/clean/stash/rebase/mazání ani instalace. Úplný Git status a ověření původních source hashů v závěrečném evidence manifestu.

### Odpovědi na závěrečné otázky

| # | Výsledek |
|---|---|
|1|Offline APJ audit PASS, celý hash/board/image/source/evidence souhlasí.|
|2|Před uploadem4.7.1/11a518e5, bez nových BIN parametrů, boot118; historie odpovídá EXEC ON.|
|3|Samostatná bránaB výslovně schválena.|
|4|Právě1upload,1program call, bez rollbacku.|
|5|Standardní Verify100.0% PASS.|
|6|Nová aplikace naběhla, UID/board/verze/new params a zdraví ověřeny.|
|7|Skutečný boot119, na začátku/konci stejný.|
|8|Po uploadu COM5 dostupná; fyzický reconnect nebyl potřeba. Krátké bootloader reenumerace neměřené.|
|9|Ano0/10/60;1221indexed +2hidden direct-read =1223uložených jmen.|
|10|UCS_B_ENABLE0 na začátku i konci.|
|11|utb_log přítomný a20periodickými legacy statusy doložený postup.|
|12|Main4352/7168,ISR1288/1536,worker2408/3504,FTP1584/2992B; pouze runtimeOFF.|
|13|FREE/LRG po regionech v části16, bez trvalého poklesu ve dvou snapshotech. MEMINFO32=734840není univerzální souvislý heap.|
|14|4řídké vzorky18.3–19.1%;prokazatelný baseline rozdíl nelze určit.|
|15|Společná letová konfigurace zachovaná; změny startup/auto hodnot a STAT čítačů uvedené úplně.|
|16|Ano,4LOG_ENTRY přesně stejné. Nikoli bitový readback flash.|
|17|Žádný nový log ani write/erase požadavek hosta; interní startup marker bez trace nelze absolutně vyloučit.|
|18|Ano strukturálně/guardy a DISARMED native mode; fyzické aktivní motory netestované.|
|19|Neověřené zápisy/propustnost/admission,aktivní timing/stack,ARMED/let; viz část21.|
|20|PlánA bez nového FW po samostatné kvalifikaci RC/elektriky a souhlasu; jinak žádný ARM aB pouze nové výslovné zadání.|
|21|HW-LOG-2 skutečný BIN/stack/timing výsledek plus samostatné fyzické a letové rozhodnutí.|
