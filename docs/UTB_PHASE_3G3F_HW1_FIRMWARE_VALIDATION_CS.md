# FÁZE 3G-3F-HW-1 – Kvalifikace diagnostického firmware

**Výsledek: INCONCLUSIVE. Upload a standardní CRC verify PASS; post-upload identita není dostupná. NO-GO k HW-2A.**

Dne2026-10-25 proběhl schválený read-only preflight a právě jeden schválený upload přesného EXEC ON APJ. Uploader dokončil erase/program/Verify100% a standardní reboot. Po čekání30s následný jednorázový read-only runner nenašel očekávanou USB identitu ještě před otevřením portu. Další postup se zastavil: žádný reconnect, další reboot, upload, rollback nebo RUN.

FC po novém bootu nebyla ověřena. Není správné označit úspěšný CRC upload za zdravý start aplikace. Nebyl odeslán ARM, motorový příkaz, PARAM_SET, změna telemetrických intervalů, DataFlash erase/format ani Timing/Flight RUN. Nic se neinstalovalo, produkční zdroje se neměnily.

Evidence `tmp/utb-3g3f-hw1-20261025_offline-01/`; `summary.json` výslovně ponechává neověřené post-upload hodnoty null.

## Offline integrita

Větev `test/skystars-5inch`, HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`. Všech14 souborů původního manifestu souhlasí SHA-256 i Git blob: 7 modified +7 added. Index čistý. Původní untracked reporty a `kapitoly/historie.txt` beze změny. Patch odpovídá současnému git diff, archiv7 nových souborů odpovídá současným obsahům. Původní manifest se nepřepisoval.

SHA-256 původního `tmp/utb-3g3f-20261025_152000/source-manifest.json`:

`80b9e3dfce00d7e437c4f1c97eb014943178a4abfaec5abc420dd4b03ea00191`

Znovu kontrolováno 1955 původních souborů nativních regulátorů/loggeru/motorů/ChibiOS/arming/failsafe vůči checkpointu, všechny beze změny. H743 ELF obsahuje Execution/Timing/Chain/Flight symboly, aktivní UTB_Core/Output/Acro/Angle chybí. Compile command databáze potvrzuje TIMING_EXEC=1, TIMING=1, FLIGHT_SHADOW=1; RATE_SITL není zapnuté. APJ payload obsahuje utb_execution.txt. Původ binárky ověřen proti původnímu evidence manifestu včetně ELF/APJ/build options/source patch/archivu.

Kandidát je výhradně:

`tmp/utb-3g3f-20261025_152000/firmware/SkystarsH7HD-bdshot_UTB3G3F_EXEC_ON_dae4f020db98.apj`

- 1 158 367 B; board1075; Copter4.7.1; embedded git11a518e5 + necommitnuté zdroje identifikované manifestem.
- SHA-256 `dae4f020db98a509b68c71a5b8702dfc7019ed762f69b7d46c8cbf3ae5e9d497`.
- Nový APJ nebyl sestavován. Nový H743 boot, stack a paměť zatím neověřeny.

## Archivované dřívější výsledky

- `tmp/utb-3g3c-hw-20261025_100008/evidence-sha256.json`: všech 98 položek SHA odpovídá.
- `tmp/utb-3g3e-a-20261025_approval-preparation/evidence-sha256.json`: všech 46 položek SHA odpovídá.
- `tmp/utb-3g3e-b-20261025_143139/evidence-sha256.json`: všech 90 položek SHA odpovídá.

Samostatně znovu parsováno 3G-3C Flight4000 a E-B Flight4000, jejich8 stránek/84 metrik; E-A a E-B Timing4000/frozen reason1. Výsledky jsou na hostu a nejsou závislé na zachování RAM při bootu. Při offline auditu byly boot112 a firmwared78e34b3 pouze historické údaje; následný read-only preflight je znovu ověřil před uploadem. Stav po uploadu zůstává neověřený.

## Úplné archivy původních čtyř BIN logů

Nebylo potřeba provádět další download. Nalezeny a ověřeny původní dokončené přenosy; velikost, LOG_ENTRY ID, download success, počet bajtů i původní SHA přesně souhlasí. Ve všech čtyřech download receipts AUTOPILOT_VERSION dokládá board1075 a UID2 odpovídající serialu200024001251303433333336. Parser přečetl soubory bez BAD_DATA a bez výpisu parse chyb.

| ID | Bajtů | Parsovaných záznamů | SHA-256 |
|---:|---:|---:|---|
| 1 | 2516746 | 56451 | `8f719b8e75a3b9e752a06221c21fb31abdbdded3c6a99c7fa76b0528bf031275` |
| 2 | 7924746 | 179495 | `b540dd6883670211251955017c9a6b70d751641259b33bc290000b834ec908be` |
| 3 | 718496 | 18367 | `0d9645be0432a6c4f0cb0cb5952bcfa47e68603635d8ad48a06ba931d7a0c006` |
| 4 | 725746 | 18610 | `333d9e449d51f7a18f632ab275cafd60061357d9b07c02cf95404e0ff7e36c63` |

Umístění a původ:
- Log1: `tmp/utb-3b-measure-20261020_143658/H743_NATIVE_AP_UTB.bin`; zdrojový experiment z data v cestě; ve stejném adresáři `download-result.json` a `download-telemetry.jsonl`. Přesné SHA receipts a file mtime jsou v novém JSON auditu. File mtime není vydáváno za MCU čas vzniku logu.
- Log2: `tmp/utb-3c-20261020_151107/H743_NATIVE_AP_UTB.bin`; zdrojový experiment z data v cestě; ve stejném adresáři `download-result.json` a `download-telemetry.jsonl`. Přesné SHA receipts a file mtime jsou v novém JSON auditu. File mtime není vydáváno za MCU čas vzniku logu.
- Log3: `tmp/utb-3d2-20261020_163314/A-cold/H743_LOGGER_A.bin`; zdrojový experiment z data v cestě; ve stejném adresáři `download-result.json` a `download-telemetry.jsonl`. Přesné SHA receipts a file mtime jsou v novém JSON auditu. File mtime není vydáváno za MCU čas vzniku logu.
- Log4: `tmp/utb-3d2-hw-20261021_082016/cold-start/experiment/H743_LOGGER_TRACE.bin`; zdrojový experiment z data v cestě; ve stejném adresáři `download-result.json` a `download-telemetry.jsonl`. Přesné SHA receipts a file mtime jsou v novém JSON auditu. File mtime není vydáváno za MCU čas vzniku logu.

Celkem11 885 734 B. Jde o úplné kopie historicky stažených logů a důkazy jejich původu. **Aktuální obsah flash nebyl znovu čten**; brána A následně ověřila předupload inventář. Po uploadu se inventář získat nepodařilo. Stejné velikosti/časy logů po uploadu budou jen metadatová kontrola, nikoliv kryptografická bitová shoda s flash.

## DataFlash inicializace – konkrétní audit

Deska dědí `SkystarsH7HD/hwdef.dat:28–29,58–77`: W25Q128 na SPI3, dataflash zařízení, nominálně16MiB. Relevantní je **BLOCK / AP_Logger_Flash_JEDEC**. Lokální `AP_Logger.h:454` definuje BLOCK=4, FILESYSTEM=1; požaduje se ověření skutečného LOG_BACKEND_TYPE, nikoliv předpoklad hodnoty1.

| Cesta | Doložené chování |
|---|---|
| `AP_Logger.cpp:221–291` | Init vybírá backend podle bitmasky; backend Init proběhne před start_io_thread. Watchdog armed větev může runtime vynutit logování DISARMED. |
| `AP_Logger_Flash_JEDEC.cpp:64–85` | SPI device/dev_sem, WaitReady/JEDEC geometrie a následně Block Init. |
| `AP_Logger_Block.cpp:32–74` | DMA buffer, writebuf, sem; NeedErase vede přímo do EraseAll, jinak validate_log_structure. Není zde podmínka LOG_DISARMED. |
| `AP_Logger_Block.cpp:368–380` | Reserved poslední stránka obsahuje marker **0x1901201B**. Jiná přečtená hodnota znamená NeedErase=true. |
| `AP_Logger_Block.cpp:271–301` | EraseAll při DISARMED smaže version sektor a zahájí chip erase. |
| `AP_Logger_Block.cpp:383–425,847–896` | I při správném markeru může validate_log_structure nastavit df_EraseFrom při korupci řetězce; io_timer později maže sektory v rámci recovery. |
| `AP_Logger_Backend.cpp:330,441,471` | Běžný zápis nejprve ShouldLog; DISARMED + LOG_DISARMED0 obvykle vrací false. StartNewLogOK není ochrana init erase. |
| `AP_Arming.cpp:1770–1809`, `AP_Logger_Backend.cpp:516`, `.h:203` | Finální arm_checks volá PrepForArming → PrepForArming_start_logging → start_new_log; žádný ARM požadavek není povolen. |
| `AP_Logger_Block.cpp:549–604` | start_new_log hledá poslední stránku, může znovu použít krátký log a připraví hlavičku do bufferu; nebude hostem vyvolán. |

Oba firmware používají beze změny tentýž logger/driver a stejný formátový marker; nová Execution sonda nepřidává DataFlash formát nebo log IDs. To omezuje riziko migrace, **neprokazuje nulové riziko autoerase/recovery při novém bootu**. Aktuální reserved marker/korupce se MAVLink inventářem neověří. Úplné historické archivy řeší ochranu původních dat na hostu; jejich dostupnost není příslib neměnnosti flash.

Běžný nearmovaný ne-watchdog boot s LOG_DISARMED0 by neměl založit nový běžný letový log. Případný watchdog, vynucené logování, zpráva Chip erase/Log recovery, neočekávaný nový log nebo změna původního inventáře jsou důvodem zastavit. Žádná nová úprava loggeru, workaround nebo erase není povolený.

Read-only LOG_REQUEST_LIST čte přes BLOCK/sem/SPI a může zdržet Main. Proto pouze samostatně, mimo libovolný RUN, se zdravím před/po a bez paralelního FTP/downloadu.

## Brána A – skutečný read-only preflight

Obsluha potvrdila všechny fyzické podmínky a jedno read-only připojení. Záznam `approval-a.json`; fyzické podmínky jsou potvrzením obsluhy. Proveden jeden preflight bez reconnectu, celkem60.657s, vložené40s stabilní pozorování a následné kontroly. Windows COM5/VID1209/PID5741/serial200024001251303433333336, MAVLink1/1/board1075/UID2 souhlasí. Skutečný firmware **Copter4.7.1/d78e34b3**, boot **112**. Senzory včetně gyro/accel/baro/AHRS/logger a EKF attitude zdravé v přijatých zprávách, InternalError/drops0, parser errors0, žádný restart/ARM/mode change. Uptime monotónní. PreArm RC/battery USB-only omezení nejsou letové GO.

Úplná záloha1220/1220, indexy0–1219. Neindexované65535 notifikace se nezapočítaly. Proti E-B finálním parametrům pouze STAT_RUNTIME183966→192845; PID/RC/failsafe/motor/filtry/EKF konfigurace shodná. LOG_BACKEND_TYPE4, LOG_DISARMED0, SCHED_OPTIONS0, SCHED_LOOP_RATE400, FSTRATE_ENABLE0, všechny kontrolované RUN0. Žádný PARAM_SET ani změna intervalů. BIN archivy nebylo třeba znovu stahovat.

Přesný inventář4 logů zůstal stejný:2516746/7924746/718496/725746B, součet11885734B; UTC0/0/0/1792564233. LOG_REQUEST_LIST proběhl samostatně, potom znovu health. Metadata nejsou bitové ověření flash.

Memory snapshot FREE/LRG:

| Region | FREE B | LRG B |
|---|---:|---:|
| 0x30000000 | 7672 | 3128 |
| 0x20000000 | 37936 | 37936 |
| 0x24000000 | 524280 | 524280 |
| 0x00000400 | 64504 | 64504 |
| 0x30040000 | 32760 | 32760 |
| 0x38000000 | 54536 | 53816 |

Stack watermarky před uploadem:

| Vlákno | Volno / přiděleno B |
|---|---:|
| ISR | 1232/1536 |
| ArduCopter | 4344/7168 |
| idle | 288/496 |
| UART_RX | 864/1200 |
| OTG1 | 352/752 |
| monitor | 1096/1456 |
| timer | 1544/1968 |
| rcout | 504/944 |
| rcin | 864/1456 |
| io | 1544/2480 |
| storage | 832/1456 |
| UART4 | 328/752 |
| log_io | 1392/2016 |
| I2C0 | 984/1456 |
| OSD | 976/1712 |
| led | 280/688 |
| UART1 | 328/752 |
| I2C1 | 1000/1456 |
| SPI1 | 808/1456 |
| SPI4 | 808/1456 |
| utb_log | 2448/3504 |
| FTP | 1216/2992 |

Oproti E-B mají některé watermarky téhož dlouhého bootu jinou dosavadní hloubku (např.ISR1232 místo1264, log_io1392 místo1504); Main4344 a FTP1216 B. Nejde o nový firmware ani důkaz jeho spotřeby. Aktuální paměťové snímky nejsou historické minimum, DMA typy se nesčítají do jedné obecné souvislé rezervy. Bez alokační/stack chyby; budoucí runtime změna EXEC ON bude teprve měřena.

CPU pouze3 pasivní SYS_STATUS vzorky:18.2/18.6667/19.0 % min/mean/max. Nejde o aktivní Timing režii ani husté měření zatížení. 1384 přijatých paketů, RX51310B/TX1320B, source audit odchozích:11 REQUEST_MESSAGE,21 PARAM_REQUEST_READ,1 PARAM_REQUEST_LIST,8 read-only FTP a1 LOG_REQUEST_LIST. Upozornění FTP expected100000/got346/1135 odpovídá validnímu EOF krátkých memory/threads souborů; error_code0 a obsah zkontrolovaný.


## Schvalovací brány a skutečný upload

Brána A: „Schvaluji read-only preflight a potvrzuji všechny podmínky“. Brána B: „Schvaluji jeden přesný upload a read-only kontrolu; podmínky platí“. Uloženo approval-a.json/approval-b.json. USB-only, odpojená LiPo, demontované vrtule, bezpečná FC a zavřené ostatní sériové aplikace jsou potvrzení obsluhy, nikoliv nezávisle měřené skutečnosti. Bylo potvrzené i riziko ztráty neexportované RAM při upload bootu. Původní frozen výsledky i čtyři BIN archivy byly předem ověřeny.

Před zápisem se znovu ověřilo APJ SHA, board/source14, USB identita, DISARMED STABILIZE, původní firmware/boot112 a všechny kontrolované RUN0/LOG_DISARMED0. Jediný MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=3 vyžádal standardní bootloader; následovala pouze běžná OS enumerace a jeden bootloader identify, bez sériové reconnect smyčky. Bootloader potvrdil1075/serial200024001251303433333336, protokol5, board_rev0, STM32H743/753 Rev.V, application capacity1703936B.

Použit existující Tools/scripts/uploader.py s no_extfTrue, forceFalse, boot_delayNone, APJ extf_image_size0. Právě jedno volání upload: erase/program aplikace, Verify100%, standardní Rebooting a úspěšný návrat. Nebyl přepisován bootloader ani programován/mazán externí flash obraz. Standardní app flash erase není úplný MCU erase ani LOG_ERASE. Ochrana parametrů a DataFlash po bootu nicméně **není provozně potvrzená**, protože aplikace po uploadu nebyla dostupná.

Upload exit code **0**, application_program_calls1, bootloader_entry_commands1, upload_completedTrue, standard_verification_passedTrue. Prewrite UTC2026-10-25T17:27:21.396877+00:00. Celý stdout/stderr v upload-console.txt, upload/result.json a upload-exit-code.txt; skutečný raw RX10990B, TX1300491B. CRC ověření uploaderu se nevydává za kryptografický SHA-256 readback celé interní flash. Žádný další upload, reset nebo rollback.

## Post-upload stop

Runner post-readonly-once.py nejprve čekal30s na inicializaci. Při prvním Windows USB výčtu nebyl nalezen právě jeden port s požadovaným VID/PID/serial. Nastalo:

```text
AssertionError: USB identity absent or ambiguous
stage=open
UTC2026-10-25T17:29:02.464017+00:00
```

Chyba nastala před serial.open a jakýmkoliv post-upload MAVLink dotazem. Nejde o ClearCommError při již otevřeném spojení ani o prokázanou neshodu numeriky. Funkčnost aplikace/USB po rebootu zůstává neověřená. Původní runner, traceback, sentinel a result.json se zachovaly; žádná úprava firmware ani další pokus nenásledoval. Následná jednorázová kontrola Windows PnP/CIM a existujících System událostí neotevírá COM a není novým pokusem o spojení; její přesný výsledek je v windows-*-after-stop.json. Závěrečný přehled přítomných zařízení obsahuje pouze Bluetooth COM6/COM7; očekávaná FC ani STM32 DFU nejsou v tomto snapshotu přítomné. To neurčuje příčinu zmizení zařízení.

Příčina není prokázaná. Možnosti zahrnují startup aplikace/paměť/USB inicializaci, původní USB/napájecí problém nebo fyzický stav připojení; bez dalších důkazů se žádná nepovažuje za vysvětlení. Úspěšný CRC neprokazuje zdraví aplikace a dřívější USB problémy neprokazují, že nyní jde o stejnou závadu.

## Úplný stav požadovaných kontrol

| Bod | Výsledek |
|---|---|
| Offline Git/source/APJ/board/symbol/provenance | PASS,14 souborů původní fáze shodných |
| Původní frozen výsledky a4 plné BIN archivy | PASS, obsahově/hash ověřené na hostu |
| Logger initialization/autoerase audit | Proveden, zbytkové riziko výše; zdroje/formát beze změny |
| Brány A a B / fyzické podmínky | Výslovně schváleno a potvrzeno |
| Preflight USB/UID/board/firmware/health | PASS;4.7.1/d78e34b3, boot112 |
| Pre-upload parametry |1220/1220, úplné indexy0–1219, letová konfigurace shodná s E-B |
| Pre-upload stack/memory | Skutečné snímky výše, nikoliv nový firmware |
| Pre-upload log inventory |4 /11885734B, původní ID/UTC/velikosti shodné |
| Jediný standardní upload / CRC | PASS, exit0, Verify100%, jeden program pokus |
| Nová aplikace/USB/firmware verze | NEOVĚŘENO, očekávaná USB identita nenalezena |
| Boot counter po uploadu | NEOVĚŘENO, poslední112 před uploadem |
| Parametry po uploadu | NEOVĚŘENO; žádné automatické obnovování |
| Timing/Execution/Flight SYS rozhraní | Offline v kandidátu doloženo, na novém běhu NEOVĚŘENO |
| Pending sessions, všechny RUN0 po bootu | NEOVĚŘENO provozně; default0 doložen source/build, žádný RUN odeslán |
| Post-upload gyro/accel/baro/AHRS/EKF/InternalError/logger | NEOVĚŘENO |
| Post-upload CPU/heap/tři stack snapshoty | NEZMĚŘENO |
| Původní DataFlash po uploadu | NEOVĚŘENO; bezpečné historické kopie jsou na hostu |
| Změny hostem nastavovaných parametrů/intervalů | Žádné, není co obnovovat; autonomní startup změny se nepodařilo načíst |
| Konečný FC stav | Neznámý; poslední skutečný předupload stav DISARMED STABILIZE |
| Konečný COM | Všechny vlastní porty uzavřené, post-upload spojení neotevřeno |
| Neočekávaný rozdíl | Nedostupná očekávaná USB identita po novém bootu |
| HW-2A | NO-GO, žádné další experimenty |

## Odpovědi na závěrečné otázky

1. APJ integrita souhlasila SHA/board/provenance.
2. Původní frozen výsledky jsou bezpečně archivované a znovu parsované.
3. Všechny4 DataFlash logy mají úplné archivní kopie a ověřené receipts/SHA/původ/parser.
4. Při bootu existuje autoerase při neplatném markeru a sektorová recovery při korupci struktury, i při LOG_DISARMED0; neměnil se logger ani marker.
5. Předupload identita FC souhlasila v MAVLink i bootloaderu; po uploadu neověřena.
6. Jeden přesný upload byl samostatně schválen.
7. Právě jeden programovací upload byl proveden.
8. Uploader verify úspěšný podle CRC protokolu.
9. Nově skutečně naběhlou verzi nelze doložit; nahraný kandidát obsahuje4.7.1/11a518e5 s původním manifestem.
10. Nový boot counter neznámý, poslední ověřený112.
11. Zachování parametrů po uploadu není ověřeno; úplná původní záloha existuje.
12. Zachování logů po bootu není ověřeno; původní kopie existují.
13. Execution SYS provozní dostupnost neověřena.
14. Timing pending po novém bootu neověřeno.
15. Flight pending po novém bootu neověřeno.
16. Aktuální DISARMED STABILIZE nelze telemetrií potvrdit; žádný ARM/mode příkaz nebyl odeslán.
17. RUN0 po bootu neověřeno; default0 doložen a žádný RUN odeslán.
18. Nová runtime RAM neznámá; statický přírůstek1712B se nevydává za runtime měření.
19. Nové watermarky neznámé; předupload hodnoty jsou výše.
20. Post-boot chování loggeru nelze určit; nedostupná USB komunikace není důkaz logger závady.
21. Firmware nyní **není kvalifikovaný pro HW-2A**.
22. Nejprve objasnit startup/USB a získat zdravou identitu/konfiguraci/paměť/log inventory. Poté samostatně schválené HW-2A/2B, kvalifikace aktivní pasivní sondy, letového záznamu, dynamického časování/heap/stack, RC/failsafe/yaw a motorové konfigurace. Bez automatického letu či aktivní UTB autority.

## Další nejmenší krok – pouze návrh

Po novém rozhodnutí nejprve posoudit stav USB zařízení na hostu a fyzický stav obsluhou, bez automatického restartu/uploadu. Další fyzické odpojení/připojení, nové otevření portu nebo recovery vyžaduje nový konkrétní souhlas. Žádný rollback se sám nespustí. Historické výsledky i chyby této fáze zůstávají zachované.

Konečný Git: HEAD11a518e5857b24cc554b3cb7e6492998013ddd1a, větev test/skystars-5inch, index čistý. Původních7+7 souborů3G-3F a předchozí reporty/historie beze změny; přidán tento report a ignorované důkazy. Bez commit/push/reset/clean/stash/rebase. Nový build ani instalace neproběhly.
