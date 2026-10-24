# FÁZE 3F-2 – První HW ověření integrovaného Attitude → Rate/PID SHADOW

Datum: 24. 10. 2026. Skutečná Skystars STM32H743 Rev.V, pouze USB, bez LiPo a všech vrtulí, trvale DISARMED. Jeden upload a jeden experiment, bez opakování.

## 1. Výsledek a rozsah potvrzení

Integrovaný řetězec dokončil **1000 main pokusů, 1 Rate warmup a 999 správně spárovaných Rate kroků**. Attitude provedl **1000 primary STABILIZE + 1000 yaw resetů, všech 2000 operací validních**. Missing/duplicate/invalid/DESYNC/UNSUPPORTED/mismatch a motor_changes **0**; stop reason **1 – dokončený počet**.

Všech **84 exportovaných max. absolutních chyb i RMSE je 0**. Přesnost tohoto tvrzení má dvě úrovně: 58 upstream/Rate veličin navíc podléhá přesnému testu nenulového FP rozdílu; u 26 Attitude veličin je potvrzena nulová exportovaná chyba a žádné překročení původní tolerance, nikoliv nezávislý důkaz bitové shody pod rozlišením textu. Podrobnosti v části 4.

Měřené nepřekrývající se přidané brackety dávají **192,516 µs/main**, **7,70064 % nominální periody 2500 µs**. Neobsahují celý transport ani kompletní capture/audit. CPU load: **18,083 % → 28,200 % → 18,163 %**, Rate capture intervaly **2479–2521 µs**, průměrný absolutní jitter **1,6002 µs**, main stack **4352/7168 B volných před i po**. Úplné WCET a Main perioda nejsou změřené.

**Lze doporučit samostatně schválený 4000krokový USB-only DISARMED experiment podporovaného profilu**, s novým fyzickým startem kvůli uzavřené RAM session, stejnými stop podmínkami a kontrolou stacku/health. Tento výsledek neověřuje aktivní UTB řízení, letovou bezpečnost, dynamický shaping, nenulové I bez resetů ani ozbrojenou saturaci. Další experiment neproběhl.

## 2. Integrita, identita a jediný upload

Repo `/home/jirka/ardupilot_utb`, větev `test/skystars-5inch`, HEAD **89e81d94763f1610b52b6e93510de3df71969af0**, předchozí report commit **90b24237c94d935fbc7907488155d07d87fc3934** zachovaný. Všech 18 souborů zdrojového manifestu 3F-1 odpovídá uloženým SHA-256. `kapitoly/historie.txt` je beze změny, mimo index.

Použit výhradně původní kandidát:

`tmp/utb-3f1-20261024_155715/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`

- Velikost **1 148 707 B**.
- SHA-256 **e4e763e121d0ae5390a9f34d456ca2970bf82ffb1545bd092d1a790d1f94005d**.
- APJ board_id **1075**, git_identity **90b24237**. Build vznikl před implementačním commitem; přesné zdroje jsou svázané manifestem 3F-1, nikoliv tvrzením, že binární verze musí hlásit 89e81d94.
- COM5, VID:PID **1209:5741**, USB serial **200024001251303433333336**, MAVLink 1/1.
- Bootloader protocol **5**, board **1075**, MCU **STM32H743/753 Rev.V**, application capacity **1 703 936 B**, ext flash image **0**.

Před uploadem stabilní komunikace, DISARMED STABILIZE, zdravé gyro/accel/AHRS/logger a EKF attitude, **1215/1215 parametrů** zazálohovaných. Původní firmware Copter 4.7.1 / **95affbde**, STAT_BOOTCNT=104. Obsluha potvrdila USB-only, odpojenou LiPo, demontované vrtule a volný COM5.

Standardní uploader jednou přešel do bootloaderu, před mazáním aplikace znovu ověřil board i serial, zapsal pouze schválenou aplikaci a úspěšně ji verifikoval. Pro protocol 5 se používá porovnání CRC v `Tools/scripts/uploader.py:627`, volané z uploadu. Není to SHA-256 readback MCU flash. Žádný bootloader update, mazání parametrů nebo log flash. Následoval pouze standardní restart uploaderu, žádný další vynucený reboot.

Po uploadu firmware hlásí Copter 4.7.1 / **90b24237**, STAT_BOOTCNT=105. Inicializace a health prošly. Opět úplných **1215/1215 parametrů**. Očekávané změny seznamu: přibyly **UCS_RUN=0, UCS_COUNT=4000**, zmizely **UTT_RUN, UTT_COUNT** vypnuté Timing diagnostiky. Změny hodnot: boot/runtime statistiky, gyro startup offsets/kalibrační teploty a barometrický ground pressure; přesná čísla jsou v `post-upload-parameter-comparison.json`. PID, RC, failsafe, motorové a další letové nastavení se nezměnilo.

Read-only ověřeno SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0, SMAX/NTF/NEF=0 na všech osách, bez rate threadu; UCS/UAS/UNS_RUN=0, UTB_SHADOW=0, LOG_DISARMED=0 a SCHED_OPTIONS=0. UTT parametr není přítomen a nespouštěl se. Nový Chain export byl **UTBChainV1,pending**, ověřený i při vstupu do měřicího runneru.

Logger byl healthy. Původní čtyři logy: **2 516 746, 7 924 746, 718 496, 725 746 B**, celkem **11 885 734 B**. Hwdef uvádí W25Q128 (nominálně 16 MiB), ale inventory není přesné měření použitelného souvislého volného prostoru v BLOCK loggeru. RAM-only test neotevírá nový log a nevyžaduje další DataFlash kapacitu. Inventář před/po je identický; nebyl download celého flash obsahu pro novou bitovou verifikaci.

## 3. Skutečný protokol

Artefakty: **tmp/utb-3f2-hw-20261024_181323/**. Schválení rozsahu a fyzické potvrzení jsou uložené v `approval.json`. `upload/upload-attempt.json` a `experiment-attempt.json` odděleně chrání před opakovaným spuštěním. Žádný nový build nebo úprava produkčního kódu.

Stejné streamy ve všech oknech: SYS_STATUS **2 Hz**, SYSTEM_TIME **1 Hz**, EKF_STATUS_REPORT **1 Hz**; původní intervaly všech tří byly **−1**. Po stack/pending kontrolách následovalo 3s ustálení a **15,016s baseline**.

| Host čas od otevření runneru [s] | Událost |
|---:|---|
| 9,765–24,781 | Baseline, UCS/UAS/UNS=0 |
| 24,781 | UCS_COUNT=1000 |
| 25,859 | Jediný UCS_RUN=1, continuous Chain |
| 25,859–29,937 | Tiché přijímací okno, bez dalších požadavků |
| od 29,937 | Read-only potvrzení automatického UCS_RUN=0, UAS_RUN=0 |
| dalších přibližně 8 s | Stejná telemetrie před exportem |
| poté | FTP frozen snapshotu a analýza |
| 38,234 / 38,281 | Obnovení UCS_RUN=0 / původního UCS_COUNT=4000 |
| do 46,515 | Health, stack po, návrat intervalů na −1, uzavření COM5 |

Audit odchozích packetů v měřicí stage obsahuje **právě jeden PARAM_SET UCS_RUN=1**. Žádný FTP, polling, log listing/download, změna loggeru nebo další diagnostický požadavek v tomto okně. Po experimentu proběhla ještě read-only konečná záloha parametrů, health a inventář logů. COM5 je uzavřený.

## 4. Počty, propojení a numerika

| Veličina | Naměřeno |
|---|---:|
| Main attempts / warmup / paired Rate | **1000 / 1 / 999** |
| Attitude primary seen/valid | **1000 / 1000** |
| RESET_YAW seen/valid | **1000 / 1000** |
| Attitude seen/valid | **2000 / 2000** |
| Rate seen/valid | **999 / 999** |
| Attitude / Rate úplné synchronizace | **1 / 1** |
| Attitude mode/config changes, PID config changes | **0 / 0 / 0** |
| Externí PID I_set události / filter resets | **2994 / 0** |
| Rate sequence first→last / period_count | **1→999 / 998** |
| Poslední source Attitude loop → consumed Rate loop | **70379 → 70380** |
| Missing / duplicate / invalid / unsupported | **0 / 0 / 0 / 0** |
| Attitude/PID DESYNC, mismatch, invalid, unsupported | **vše 0** |
| Upstream / downstream mismatch | **0 / 0** |
| Motor checks / changes | **3000 / 0** |
| Stop reason | **1, dokončený počet** |

### Skutečný vlastní tok dat

`AP_UTB_ChainShadow.cpp:29` ukládá výsledek `_attitude.history()` po skutečném vlastním výpočtu, nikoliv `frame.after`. `rate()` na řádku 47 kontroluje při každé spotřebě `_loop == _source + 1U`. Do Rate inputu přiřadí **`_target.body + společný sysid`** a **vlastní `_target.feedforward`**, teprve potom spustí `_rate.process(frame)`. Native target zůstává srovnávací hodnotou. První main krok má samostatně počítaný warmup; jeho Rate se nevydává za validní výpočet. Poslední nově vytvořený Attitude target zůstává po ukončení okna nespotřebovaný.

Pro každý započítaný Rate krok prošel interní guard N→N+1 a přesná kontrola upstream reference. Snapshot uchovává agregace a poslední pár, nikoliv 999 raw párů. **Nezávislý offline audit všech jednotlivých loop ID nelze provést**, poslední pár kontrolu splňuje a missing/duplicate jsou 0. Závěr o všech párech se opírá o kód guardu a čítače, ne o neexistující raw trasu.

V posledním páru byl skutečný gyro **[0,000612442; 0,001437460; −0,001159354] rad/s**, dt **0,002498 s**, motor flags **1/1/1**. Own/native body target **[−0,06966721; 0,02746463; −0,000212577] rad/s**. Například Roll P **−0,005689635**, D **0,000293608**, output **−0,005396027** v obou větvích. Nejde tedy o porovnání samých nulových výstupů. Úplný poslední kontext v původním snapshotu; zaokrouhlený text neslouží k rekonstrukci všech interních floatů.

Own target quaternion v posledním spotřebovaném kontextu má normu přibližně **1,000000009** z exportovaných hodnot. Finite/unit-history kontroly a shape/reference srovnání prošly. Není to samostatné přesné měření normy každého quaternionu.

| Skupina | Počet metrik | Počet porovnání na metriku | Export max/RMSE | Síla závěru |
|---|---:|---:|---|---|
| Attitude target/error quaternion, Euler/rate/acceleration/body target, thrust/FF | 26 | 2000 | **0 / 0** | Žádné překročení `1e−6 × max(1, abs(AP))`; přesná nula pod textovým rozlišením není dokázaná |
| Spotřebovaný body-rate target + FF scalar | 4 | 999 | **0 / 0** | Nulový přesný FP rozdíl, upstream_mismatch=0 |
| R/P/Y PID/filter/history/P/I/D/FF/D_FF/output/flags | 3×18 | 999 | **0 / 0** | Nulový přesný FP rozdíl porovnávaných projekcí, pid_mismatch=0 |

Přesný seznam a hodnoty všech **84 řádků** je v `analysis/metrics.csv`. Attitude engine počítá nezáporná maxima/squares v RAM, ale export je desetinný text a mismatch používá toleranci. **U těchto 26 řádků proto nelze rozlišit skutečnou nulu od velmi malého podrozlišovacího rozdílu.** U upstream a PID není závěr o numerické rovnosti založený pouze na zaokrouhlených nulách; kód testuje nenulový rozdíl pomocí fpclassify. Nejde o bitovou shodu všech neporovnávaných struktur nebo znaménka nuly.

### Historie a skutečné externí resety

Attitude import **1**, dalších **1999 operací bez plného importu**; mode/config změny 0. Rate import **1**, dalších **998 kroků s vlastní filtrovou historií**, bez opakované úplné obnovy z AP. Původní engine v `AP_UTB_AttitudeShadow.cpp:159` a `AP_UTB_NativeShadow.cpp:172` importuje historii jen při začátku/synchronizaci a běžnou návaznost porovnává, neopravuje kopírováním AP post stavu.

Skutečných **1000 yaw resetů** bylo replayováno do vlastní Attitude instance. U Rate nastalo **2994 I_set replay**, tedy **998 následných kroků × 3 osy**; první krok seeduje celý stav a pending I flag zvlášť nezapočítává. I reset je skutečná externí AP událost při DISARMED, není to tajný úplný import PID. I peak=0, independent_i=0, filter resets=0. **Nezávislá nenulová I akumulace bez resetů tímto HW profilem ověřená není.**

att_dynamic=2000 pouze splňuje interní detektor nenulového body targetu a thrust error; neprokazuje pilotní dynamické buzení. Reference peak=0, shaping=0, FF/D_FF peak=0. `limited=999` jsou shutdown flags v DISARMED, ne ozbrojená saturace. Full reset/relax/inertial operace, ACRO a mode transitions v tomto testu neproběhly.

## 5. Skutečné H743 časování

Časy pocházejí z AP_HAL monotónních MCU hodin. Jsou to wall-time brackety včetně případné preempce, nikoliv čistý instruction count. Časové sumy nebyly nahrazené SITL údaji.

| Bracket | Součet [µs] | Počet | Průměr/volání [µs] | Průměr/main [µs] | Exportované maximum [µs] |
|---|---:|---:|---:|---:|---:|
| Attitude pre capture | 7941 | 2000 | 3,9705 | 7,941 | nedostupné |
| Attitude post capture/audit bez consume | 6808 | 2000 | 3,404 | 6,808 | nedostupné |
| Attitude sync/validace | 37041 | 2000 | 18,5205 | 37,041 | nedostupné |
| Attitude math | 37818 | 2000 | 18,909 | 37,818 | nedostupné |
| Attitude comparison | 19478 | 2000 | 9,739 | 19,478 | nedostupné |
| Attitude celý consume | 115450 | 2000 | 57,725 | 115,450 | **205** |
| Rate PID math | 8009 | 999 | 8,01702 | 8,009 | nedostupné |
| Rate PID consume | 60795 | 999 | 60,85586 | 60,795 | nedostupné |
| Rate process bracket | 62317 | 999 | 62,37938 | 62,317 | **73** |
| Native Rate + capture | 16240 | 999 | 16,25626 | 16,240 | nedostupné |

Sync/math/comparison jsou uvnitř Attitude consume; PID math a consume uvnitř Rate process. Tyto vnořené hodnoty se znovu nepřičítají. **Native Rate + capture obsahuje původní AP výpočet a není přidanou UTB režií.** Maxima 205 a 73 pocházejí z různých volání a nesčítají se jako WCET jednoho main cyklu. Jiné maximální brackety engine může interně sledovat, ale tento frozen export je neposkytuje; firmware se kvůli tomu neměnil.

Nepřekrývající se dostupné přidané části:

`7941 + 6808 + 115450 + 62317 = 192516 µs / 1000 main = 192,516 µs/main`.

**Chybí úplný vnější Chain process/transport span.** Před Rate process timerem se kopíruje kontext a počítá vlastní reference/upstream metriky; po něm se projektují historie, počítají downstream/peak statistiky. Rate snapshoty a přímý motor audit v `AC_AttitudeControl_Multi.cpp` nemají samostatný čistý přidaný bracket. Guardy, fan-out, časový accounting a publikace rovněž nejsou kompletně izolované. Attitude consume už část transportních kopií obsahuje. Proto 192,516 µs/main není kompletní dodatečný overhead ani jeho horní mez.

Systémový load vzrostl o přibližně **10,117 procentního bodu**, zatímco uvedený bracketový součet je 7,701 % nominální periody. Rozdíl se nesmí automaticky vydávat za změřených 60,4 µs konkrétní chybějící funkce: SYS_STATUS je časově průměrovaný systémový údaj a aktivní vzorek je krátký. Zdroj prokazuje neměřené transportní/accounting úseky, ale jejich jednotlivé ceny nejsou dostupné. Bracket Attitude consume/main je proti samostatnému 3E5 vyšší (115,450 vs 94,434 µs); obsahuje navíc integrační transport/uložení vlastního targetu, ale různé buildy a okna nejsou izolovaný profil konkrétní instrukce. Žádná optimalizace nebo změna původních regulátorů se neprovedla.

![Měřené brackety](../tmp/utb-3f2-hw-20261024_181323/analysis/timing-brackets.png)

Rate capture intervaly: **998**, součet **2 494 995 µs**, minimum **2479**, maximum **2521 µs**, průměr **2499,99499 µs**, průměrný absolutní jitter **1,60020 µs**, průměr **400,00080 Hz**. First/last Rate timestamp **178361182 → 180856177 µs**.

To jsou intervaly **before_rate capture**, nikoliv konce výpočtu, Attitude entry nebo Main sample timestamp. Chybí baseline Rate periody bez běžícího enginu, všechny raw periody, histogram/P95/P99 i counter nad konkrétní mezí. Z maxima plyne, že v těchto 998 intervalech není 5ms prodleva. Nelze tvrdit absenci všech hard deadline misses nebo dokázat zlepšení jitteru proti 3E4 s odlišným měřicím bodem. **Main/Attitude perioda, PM.NLon/MaxT a task-budget overruns jsou neověřené**: Timing compile-OFF, logger se neotevřel, tasks.txt se nečetl.

## 6. CPU, stack a health

| SYS_STATUS CPU load | N | Průměr [%] | Min–max [%] |
|---|---:|---:|---:|
| Baseline | 30 | **18,0833** | 17,7–18,4 |
| Vnitřní aktivní okno | 4 | **28,2000** | 28,1–28,4 |
| Po skončení, před FTP | 16 | **18,1625** | 17,8–18,4 |

Okna se přiřazují host časem příkazu, vnitřní ON výsek od +0,5 do +2,2 s v nominálním 2,5s běhu. Load není okamžitý profil ani přesná synchronizace s MCU samplem; aktivní výsek má jen 4 zprávy. Návrat po testu je blízko baseline. Pozdější exportní špičky se nezahrnují. Žádný pozorovaný restart, watchdog, InternalError nebo ztráta gyro/AHRS/EKF/logger health.

![CPU](../tmp/utb-3f2-hw-20261024_181323/analysis/cpu.png)

| Stack watermark, volné/přidělené B | Před | Po včetně exportu |
|---|---:|---:|
| ArduCopter main | **4352/7168** | **4352/7168** |
| ISR | **1288/1536** | **1280/1536** |
| FTP | **1656/2992** | **1432/2992** |
| utb_log (stávající worker, není nový Chain worker) | **2520/3504** | **2520/3504** |

Rate thread nebyl přítomný před ani po uploadu/testu. Stack main je skutečně nově pozorovaný v tomto buildu; nezměněný watermark znamená, že test nepřekonal předchozí startup maximum, nikoliv že nový 832B frame nemá cenu. Známé vlastní statické framy nejsou celá call-chain. Runner před i po ověřil rezervu vyšší než 832+200 B známých Rate framů; jde pouze o nutnou kontrolu známé části, **nikoliv normativní bezpečnostní limit nebo důkaz dostatečné rezervy pro let**. Pokles ISR o 8 B a FTP o 224 B je zaznamenaný, neignorovaný; z těchto agregací nelze připsat každou změnu konkrétnímu okamžiku. Nový plný WCET/watermark za letu není ověřený.

![Stack](../tmp/utb-3f2-hw-20261024_181323/analysis/stack-watermark.png)

## 7. Motorová izolace a konečný stav

Read-only source audit znovu porovnal **2454 chráněných souborů** se stavem před 3F-1: AP_Motors, HAL, arming, AC_PID, vlastní ověřená PID/Rate/Attitude matematika, původní AP Attitude rovnice, scheduler task table a safety zdroje beze změny. Všech 18 implementačních souborů odpovídá 3F-1 manifestu. Binary audit původního identického APJ/ELF zachovává H743 active compile-out; není dostupný UTB_Core/UTB_Output ani aktivní UTB ACRO/ANGLE.

**3000 RAM auditů, motor_changes=0**, včetně native motorových R/P/Y a FF a body targetu. UTB nemá živý AP_Motors ani setter native PID historie; vlastní target je hodnotová kopie. Elektrické výstupy ESC se neměřily, nebyl ARM nebo motorový příkaz. Toto je RAM/source motor audit, nikoliv důkaz fyzické letové bezpečnosti.

Konečná úplná kontrola: board/serial/verze souhlasí, **DISARMED STABILIZE**, STAT_BOOTCNT **105** beze změny během experimentu a závěrečné kontroly, zdravé gyro/AHRS/EKF/logger, bez pozorovaného InternalError. **UCS/UAS/UNS_RUN=0, UCS_COUNT=4000, LOG_DISARMED=0, SCHED_OPTIONS=0**, všechny tři telemetrické intervaly obnovené na −1. Proti post-upload záloze se změnil jen STAT_RUNTIME. Původní čtyři logy mají stejná ID, časy i velikosti; COM5 uzavřený. Counts=4000 neznamená spuštění dalšího experimentu.

Žádná instalace, nový firmware build, změna produkčního C++/regulační matematiky/loggeru/scheduleru, Git commit/reset/push ani druhý experiment. Mimo ignorované tmp vznikl pouze tento nový report. Index a tracked worktree čisté; untracked report a původní kapitoly/historie.txt. HEAD zůstává 89e81d94763f1610b52b6e93510de3df71969af0.

## 8. Artefakty a další rozhodnutí

| Artefakt v tmp/utb-3f2-hw-20261024_181323/ | Obsah |
|---|---|
| source-check.json, approval.json | Integrita kandidáta a schválený rozsah |
| pre-upload/, post-upload/, final-check/ | Identity, health, kompletní parametry a surové zprávy |
| parameters-before-upload.param, parameters-after-upload.param, parameters-after.param | Úplné zálohy |
| pre-upload-extra/, post-upload-extra/ | Log inventory, threads, pending Chain |
| upload/result.json, upload/pre-upload.json, upload/upload-attempt.json | Jediný upload, bootloader identita, úspěšná verifikace |
| post-upload-parameter-comparison.json, final-parameter-changes.json | Přesné změny konfigurace |
| measurement/telemetry.jsonl, measurement/result.json | Telemetrie, odchozí packet audit, časová osa a cleanup |
| measurement/utb_chain_shadow.txt, measurement/shadow.json | Původní frozen snapshot a strojový parse |
| measurement/threads-before.txt, measurement/threads-after.txt | Nové H743 watermarky |
| analysis/metrics.csv, timing-brackets.csv, cpu.csv, stack-watermark.csv, history.csv, summary.json | Numerika, historie a měření |
| analysis/*.png | CPU, timing a stack grafy |
| final-fc-check.json, final-integrity.json, artifact-sha256.json | Konečný FC/source/Git stav a SHA-256 evidence |

1. **Celý řetězec prošel porovnáním v podporovaném klidovém H743 profilu:** 84 exportovaných nulových max/RMSE; Attitude v uvedené toleranci, upstream/Rate přesné numerické projekce. Tvrzení absolutní bitové shody všech Attitude stavů by šlo nad dostupnou evidenci.
2. **999 správně spárovaných kroků**, jeden warmup; každý validní pár podléhá N→N+1 guardu, dostupný raw poslední pár sedí.
3. **Jedna počáteční synchronizace každého enginu**, žádný další plný import. Skutečné yaw a I_set resety se přehrávají; samostatná nenulová I akumulace bez resetů zde není ověřená.
4. **Měřená část přidané režie 192,516 µs/main**, CPU přibližně +10,117 p.b.; celý transport/capture/audit a WCET zatím nejsou měřené.
5. **Main watermark 4352/7168 B volných před i po**, bez pozorovaného nového maxima využití; žádná univerzální garance stacku.
6. **Motorová izolace zachovaná:** 3000 auditů bez změny plus source/binary guardy. Původní AP zůstává jediným vlastníkem motorové cesty.
7. **Doporučený další krok: až po samostatném schválení jeden 4000krokový DISARMED Chain test**, po fyzickém novém startu, se stejným firmwarem a podporovanou konfigurací. Nejprve znovu zkontrolovat pending, health, parametry a stack; porovnat ustálený load delšího okna. Kontinuální 4000krokový test ani let zde neproběhl. Pro plné WCET či rozklad chybějícího transportního času by byla nutná zvlášť posouzená instrumentace; nelze ji nahradit součtem vnořených maxim.
