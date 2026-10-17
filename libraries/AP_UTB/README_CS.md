# AP_UTB: vlastní C++ knihovna a Copter adaptéry

Vlastní C++ řídicí vrstva, Copter adaptéry a C++ testy jsou soustředěné pod
tímto adresářem. Python nástroje a testy jsou oddělené v
[Tools/utb](../../Tools/utb/README_CS.md). [Hlavní přehled projektu](../../docs/UTB_README_CS.md) a
[úplná mapa souborů](../../docs/UTB_FILE_MAP_CS.md) popisují souvislosti.

```text
libraries/AP_UTB/
  AP_UTB.cpp/.h                 SHADOW manager a parametry
  AP_UTB_config.h              compile-time guardy
  AP_UTB_State.*               read-only AP stav
  AP_UTB_RateController.*      vlastní PID matematika
  AP_UTB_MotorMixer.*          pouze diagnostický mixer
  AP_UTB_Reference.*           AP-derived SHADOW reference
  AP_UTB_SyntheticReference.*  DISARMED BENCH reference
  AP_UTB_LogPacket.*           packed SHADOW serializace
  AP_UTB_SITL*.cpp/.h          aktivní adaptér / attitude / allocation, pouze SITL
  Copter/                     původně samostatné UTB soubory z ArduCopter/
  tests/                      C++ unit testy a wscript
```

Copter soubory jsou stále vehicle kód: sestavuje je explicitně
[ArduCopter/wscript](../../ArduCopter/wscript) jako součást vozidla.
Nejsou součástí obecné knihovní kompilace AP_UTB. Původní AP soubory
obsahují registrační a integrační zásahy; jejich těla se nepřesouvají do
této knihovny. Při běhu H743 zůstává UTB SHADOW bez motorové autority.

Tři vlastní benchmarkové source/test soubory zůstávají u
[AP_Logger](../AP_Logger/AP_Logger_UTBBench.h): `AP_Logger_UTBBench.cpp`,
`AP_Logger_UTBBench.h` a `tests/test_utb_bench.cpp`. Instrumentace je
nezávislá na volitelném AP_UTB a musí podporovat i baseline BENCH ON/UTB OFF.
Tím se zabrání zavedení závislosti základního loggeru na volitelné knihovně.
Příslušný testovací `wscript` zůstává rovněž u AP_Logger.

## Komentáře a revize

Vlastní C++ a Python soubory používají [úplnou šablonu](../../docs/komentare.txt).
`@version YYYY-MM-DD` označuje datum poslední revize souboru;
nejde o verzi firmware. `@date` označuje datum této revize.
[Pravidla a viditelnost funkcí](../../docs/UTB_SOURCE_GUIDE_CS.md).

## Spuštění z kořene repository

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
.venv/bin/python ./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --disable-UTB_BENCH
.venv/bin/python ./waf --targets bin/arducopter,tests/test_utb,tests/test_utb_sitl_rate,tests/test_utb_sitl_attitude,tests/test_utb_ap_allocation -j6
UTB_AUDIT_EVIDENCE="$PWD/tmp/utb-1hd-offline-20261012_162639/download-audit/evidence.json" \
  .venv/bin/python -B -m unittest discover -s libraries/AP_UTB/tests/python -p 'test_utb*.py' -v
.venv/bin/python -B Tools/utb/tools/test_utb_active_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-basic --suite basic --speedup 2
.venv/bin/python -B Tools/utb/tools/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-attitude --suite preliminary
```

SITL runnery spouštět po jednom, vždy do nového výstupního adresáře.
Hardware capture/download nástroje v `Tools/utb/tools/` mají vlastní samostatné
podmínky a nejsou součástí běžného SITL ověření. Přesun nezahajuje hardware
experiment a nemění parametry FC. Referenční fyzické konfigurace zůstávají
v [configs/skystars_5inch](../../configs/skystars_5inch/README_CS.md).

Historické reporty, manifesty a příkazy zachycují původní cesty platné při
měření. Aktuální navigační dokumenty používají nové cesty; v historických
reportech se přesměrovaly pouze souborové odkazy, bez změny jejich labelů,
výsledků nebo původních příkazů. Původní převod 48 cest je v `tmp/utb-source-layout-*`. Následné oddělení
Pythonu, záloha a úprava deklarací jsou v `tmp/utb-readable-20261017_084027/`.

## Ověření

Původní ověření přesunu z 16. 10. 2026 je archivované v layout checkpointu.
Úprava deklarací a oddělení Pythonu ze 17. 10. 2026 se ověřuje samostatně
v `tmp/utb-readable-20261017_084027/`; přesné výsledky jsou v jeho logách.
Firmware se nenahrává ani fyzicky netestuje. Pro volbu varianty vždy použít
explicitní configure příkaz, nikoli předpokládat obsah současného build adresáře.

## Ověření úpravy čitelnosti, 17. 10. 2026

Implementováno: oddělené Python nástroje/testy, komentované deklarace
pracovních proměnných a odstranění dvou zbytečných interních ukazatelů.
Pořadí přiřazení a kontrol zůstává zachované; výjimky pro potřebný rozsah
objektů jsou popsané výše a v přehledu zdrojů.

Unit ověřeno: **98 C++ testů** (47 core, 9 allocation, 12 benchmark,
14 attitude, 9 rate, 3 synthetic, 4 matematické) a **128 Python testů**.
Nezávislé matematické porovnání: 30 946 vzorků ve 29 scénářích bez chyby;
CSV výstupy jsou také přesně shodné se starším archivovaným C++ executablem.

SITL ověřeno na finální variantě: DISARMED integrace, roll/pitch rate
2×3200 sad, roll attitude 5600 sad, SHADOW 200/400 Hz a guard/arming kontroly.
Samostatná UTB OFF runtime absence kontrola prošla. SHADOW regrese záměrně
vypíná logger a vyvolává plnou frontu; její drops se neinterpretují jako
výsledek stabilního hardware benchmarku. Naměřená SITL frekvence je
200,0009 / 399,9933 Hz; není to měření fyzického H743 ani jeho execution time.

Build ověřeno: SITL UTB ON/BENCH ON, finální SITL UTB ON/BENCH OFF,
SITL UTB OFF/BENCH OFF, SkystarsH7HD-bdshot UTB ON/BENCH ON a stejný
H743 build s vypnutým logováním. Param/logger metadata, flake8 a diff check
prošly. Astyle změněných C++ částí prošel; původní jednořádkový getter
`sitl_antiwindup()` v AP_UTB.h má stejnou formátovací odchylku jako před úpravou.

H743 ELF zachovává BENCH instrumentaci a neobsahuje aktivní UTB SITL mód
ani jeho controller helpery, i při vynuceném build option. Původních 2600
chráněných AP source souborů zůstalo beze změny. Tři porovnané producer/worker/
consumer funkce mají shodné compiler stack operace. Celkový H743 text je
oproti předchozímu layout buildu větší o 12 B, BSS o 4 B; změna velikosti
funkce je u dřívější konstrukce syntetického vzorku. Skutečný execution time,
WCET, jitter a stack watermark na H743 jsou **hardware neověřeno**.

Výsledky, přesné příkazy, SHA-256 artefaktů a záloha jsou v
`tmp/utb-readable-20261017_084027/`. Firmware nebyl nahrán, žádné parametry FC
se neměnily. Změny se ukládají do navazujícího lokálního checkpointu; původní HEAD zůstává jeho rodičem.
