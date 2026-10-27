# FÁZE 3G-3 – První HW ověření pasivního Flight SHADOW

Datum přípravy: 25. 10. 2026. **Stav: HW preflight zahájen a přerušen chybou Windows USB/COM po13,922s. Upload ani Flight experiment neproběhly.**

## Dosud ověřené skutečnosti

- Větev `test/skystars-5inch`, HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a` odpovídají zadání.
- Všech 15 souborů odpovídá SHA-256 a Git blobům konečného zdrojového manifestu 3G-2.
- `kapitoly/historie.txt` zůstává beze změny a mimo index: SHA-256 `3bcacd44527c67c44ea88b2cdfd2eb7edfd02705d88787c27ead2e74c45c293b`.
- Kandidátní APJ má přesně 1 155 815 B a SHA-256 `918dedaaeae9fe6fe9efdc2486eceedd1d25a78cf9a2c4fd1fee3a80c0f0dba7`.
- APJ board_id=1075, git_identity=d78e34b3, dekomprimovaná aplikace 1 281 164 B. Image SHA-256 `cd5ebe48663746ae5f4b3a0435a544ed9510f6a8b827eab98b0fbf0f474b6320` je kontrola lokálního obrazu, nikoliv readback MCU.
- APJ je totožný s původním H743_FINAL artefaktem. Původní symbol audit obsahuje Flight a Timing, nikoliv aktivní UTB_Core/Output/Angle/Acro.
- Obsluha potvrdila USB-only, odpojenou LiPo, demontované vrtule, žádné ARM/motorové příkazy a zavřený Mission Planner.
- **Schválení konkrétního uploadu a měřicího experimentu dosud neproběhlo.** Bude vyžádáno až po úspěšném živém preflightu.

## Překážka připojení

Windows enumerace sériových portů před potvrzením fyzických podmínek i po něm ukázala pouze COM6 a COM7, oba bez USB VID/PID/serial. COM5 ani USB serial `200024001251303433333336` nebyly nalezeny. Nezávislá kontrola `Get-PnpDevice -PresentOnly` nenalezla očekávané VID:PID1209:5741, daný serial, COM5 ani STM32 DFU0483:DF11.

V této první přípravné části žádný sériový port nebyl otevřen. Obsluha následně oznámila obnovenou detekci; COM5 se objevil se správným VID/PID/serial a poté byl jednou otevřen pro read-only preflight. Nepřítomnost zařízení v první kontrole není diagnóza vadného firmware nebo FC. Žádný softwarový restart nebo recovery nebyl proveden.

Aktuální board ID, běžící firmware a poslední DISARMED/health byly následně zachyceny níže. Boot counter, úplná konfigurace, log inventory, heap a stack zůstaly neověřené. Historická data3F-3 tyto chybějící výsledky nenahrazují.

## Skutečný přerušený read-only preflight

Po obnovení detekce souhlasil COM5, VID:PID1209:5741, USB serial200024001251303433333336. Z přijatého AUTOPILOT_VERSION je board ID1075, Copter **4.7.1 / 90b24237**, UID2 se shodným serialem. Stále běžel původní firmware3F, nikoliv nový kandidát3G-2. Aktuální MCU revision nebyla znovu čtena bootloaderem.

Přijato **96 zpráv**; poslední ve13.469s od zahájení runneru. Poslední heartbeat byl **DISARMED STABILIZE**, uptime55,602s. Pozorované gyro/accel/AHRS/logger health a EKF attitude byly platné; errors_count1/2/3/4 v přijatých SYS_STATUS byly0. Toto není potvrzení stavu po výpadku komunikace.

Preflight se zastavil ve fázi **health-only**, před dokončením40s stabilního pozorování, ještě před PARAM_REQUEST_LIST/zálohou. Chyba po **13.922s**:

```text
serial.serialutil.SerialException:
ClearCommError failed (PermissionError(13, 'Zařízení nezná tento příkaz.', None, 22))
```

Spojení již bylo otevřené a data se přijímala, takže nejde o doložené odmítnutí přístupu při otevření portu. Python výjimka nese název PermissionError, ale skutečná příčina USB/driver/napájení/kabelu/firmware **není určena**. Žádný pozorovaný InternalError nebo restart před chybou nebyl zachycen; po ní je aktuální stav FC neověřený. COM5 byl ve finally uzavřený, bez automatického opakování.

Před výpadkem byly odeslány jen read-only REQUEST_MESSAGE požadavky na identitu/SYS_STATUS/EKF/SYSTEM_TIME. Žádný PARAM_SET, změna intervalů, LOG_REQUEST_LIST, FTP, reboot, upload, ARM nebo motorový příkaz. Parametrů přijato0; nelze tvrdit provedenou zálohu nebo úplné porovnání konfigurace. Žádné logy nebyly čtené nebo mazané a nový firmware nebyl nahrán.

Předběžné CPU pozorování z13 SYS_STATUS: průměr18.223077%, min–max17.7–18.9%. **Nejde o dokončenou15s baseline, režii Flight nebo splněný HW benchmark.** Numerika84 metrik, pairing, raw stránky, časování Flight, stack a heap nebyly měřeny. Všech12 požadovaných závěrečných otázek o novém firmware zůstává neověřených; nelze označit PASS.

Podle stop podmínek není proveden upload ani požadavek na jeho schválení. Nejmenší další krok je samostatně obnovená a stabilní Windows USB/COM komunikace a nový read-only preflight po dalším rozhodnutí obsluhy. Neprovádět automatický další pokus, DFU recovery, nové flashování nebo experiment.

## Připravený protokol po obnovení stabilní USB komunikace

1. Read-only ověření přes Windows COM port s přesným USB serialem; heartbeat1/1, native STABILIZE/DISARMED, AUTOPILOT_VERSION board ID1075 a UID2. Neotevírat jiné neidentifikované porty.
2. Stabilní health pozorování, úplná záloha parametrů a porovnání s koncem3F-3. Startup kalibrace/statistiky odlišit od změn PID/motorů/RC/failsafe. Při skutečné neočekávané změně zastavit, konfiguraci automaticky neopravovat.
3. Před uploadem read-only log inventory, memory.txt, threads.txt a dostupný MEMINFO; žádný tasks.txt. Inventář je pouze metadata, ne bitové ověření obsahu.
4. Předložit původní FW/boot/health a přesný kandidát, následně vyžádat výslovný souhlas s jedním uploadem, standardním restartem uploaderu, jediným experimentem a dočasnou diagnostikou. Bez souhlasu nenahrávat.
5. Po jednom schváleném uploadu ověřit standardní CRC verifikaci uploaderu, zdravou inicializaci, identitu, úplné parametry, RUN0 a pending Flight. CRC nevydávat za kryptografický readback MCU.
6. **Protokol používá UTT_RUN=0.** Stávající Timing může podle kódu běžet souběžně (kombinace byla ověřena v SITL3G-2), ale zde se nepřidává. Main a Attitude entry periody bez jiné dostupné sondy zůstanou neověřené.
7. Nastavit pouze diagnostické UCS_F_COUNT4000 a UCS_F_DECIM40; ponechat všechny ostatní RUN0, LOG_DISARMED0 a SCHED_OPTIONS0. Uložit původní intervaly SYS_STATUS/SYSTEM_TIME/EKF_STATUS_REPORT a dočasně použít2/1/1Hz.
8. Po3s ustálení přibližně15s baseline. Potom jediný PARAM_SET UCS_F_RUN1; během přibližně10s měření pouze příjem přednastavené telemetrie, žádné FTP, polling nebo další požadavky.
9. Po auto-stop ověřit RUN0, DISARMED a health, poté8–10s stejné telemetrie a memory/stack kontrola. Stop při nesouladu či závažné anomálii; žádný další RUN nebo reboot.
10. Sekvenčně stáhnout metadata a osm stránek, vždy jeden otevřený soubor; po každé kontrola přenosu a konzistence identity/stránky/ID. Přesný float32/64 hex decoding původním analyzátorem; neúplný výstup není PASS. Neopakovat neomezeně neúspěšný FTP.
11. Obnovit původní diagnostické hodnoty a intervaly, porovnat plnou konfiguraci a log inventory, ověřit boot/health/DISARMED a uzavřít port.

Při neúspěšném uploadu či běhu žádný automatický další pokus. Nový build, instalace, změny produkčního C++, commit a push jsou mimo tento postup.

## Upřesnění skutečných rozhraní

`g2.att_enable` není samostatný parametr ATT_ENABLE. V `ArduCopter/Parameters.cpp:1123` je tento interní člen registrován jako **FSTRATE_ENABLE**. Podmínka g2.att_enable==0 se tedy ověřuje FSTRATE_ENABLE=0; chybějící ATT_ENABLE není důvod vytvářet nebo měnit parametr. Jde o upřesnění názvu použitého v návrhu3G-2, nikoliv změnu matematických guardů.

AC_CustomControl se posuzuje podle skutečného compile guardu a dostupných parametrů/symbolů kandidáta; nepřítomný CC_TYPE při compile-out není automaticky závada. Rate thread se navíc ověří threads.txt, nikoliv jen hodnotou parametru.

`@SYS/memory.txt` volá `Util::mem_info()` a `chHeapStatus` pro jednotlivé regiony: aktuální FREE a největší blok LRG. `MEMINFO.freemem32` vychází z `hal.util->available_memory()`/mem_available. Ani jedno není historické minimum heapu. Samostatné runtime heap minimum nebylo v tomto rozhraní doloženo. Linker heap76 728B ani statické frame320/832B nejsou naměřená rezerva.

`@SYS/threads.txt` uvádí stack watermarky main/ISR/ostatních vláken. Případně také odečítá/nuluje diagnostické thread-cycle kumulace; čtení se proto plánuje mimo měřicí okno. `tasks.txt` se nepoužije, protože může měnit scheduler diagnostiku.

`SYS_STATUS.errors_count1/2` obsahují masku InternalError, errors_count4 jeho počet. **errors_count3 je počet dropped logger zpráv**, nikoliv další InternalError maska (`GCS_Common.cpp:6021–6061`). Tyto pojmy se nebudou směšovat.

## Naměřené HW výsledky

Zatím **NEPROVEDENO**: 4000 pokusů/3999 paired, všech84 metrik, synchronizace, replay resetů, motor_changes, raw stránky, CPU, Rate jitter, stack, heap, log inventory i zachování parametrů po uploadu. Žádné očekávané nuly se nevydávají za měření. Dynamická ARMED historie zůstává doložená jen předchozím SITL; RAM data po restartu nepřežijí.

Nový hardware ani fyzický let zatím nejsou vyhodnoceny PASS. Další postup závisí nejprve na obnovení stabilní komunikace FC, poté na dokončeném živém preflightu a samostatném schválení uploadu/experimentu.

## Artefakty a Git

Nové ignorované artefakty: `tmp/utb-3g3-hw-20261025_085618/`: offline-integrity.json, source-manifest.json, physical-confirmation.json, windows-ports.json, windows-ports-after-reconnect.json, pre-upload/preflight.json, pre-upload/preflight-messages.jsonl, stop-result.json a přípravný protokol. Původní reporty a historické výsledky zůstávají zachované. Produkční zdroje a HEAD se nezměnily; nový tento HW report je necommitnutý, původní kapitola zůstává necommitnutá a nezměněná. Žádný reset, clean, stash, rebase, commit nebo push.