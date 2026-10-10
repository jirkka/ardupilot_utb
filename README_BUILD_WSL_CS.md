# ArduCopter pro kvadrokoptéru: sestavení a nahrání

Návod pro WSL Ubuntu a tuto kopii projektu zaměřenou na Copter.

Kompletní postup instalace Ubuntu, nastavení Gitu a doplnění build závislostí
je v [návodu k přípravě prostředí](README_ARDUPILOT_UTB_SETUP_BUILD_CS.md).

## 1. Otevři WSL a připrav prostředí

V PowerShellu:

```powershell
wsl -d Ubuntu
```

Další příkazy patří do terminálu Ubuntu. Aktuální umístění projektu je
`/home/jirka/ardupilot_utb` (uživatel `jirka`, nikoli původní `jirkka`):

```bash
cd ~/ardupilot_utb
```

Pokud projekt přesuneš jinam, změň pouze tento `cd`; příkazy `./waf`
spouštěj vždy z kořene této kopie repozitáře.

### Jednorázová příprava po naklonování nebo přesunu na jiný počítač

V této instalaci je ARM GCC 10.2.1 již rozbalený v
`/opt/gcc-arm-none-eabi-10-2020-q4-major`. Původní cesta
`tmp/toolchains/...` zde neexistuje. Ověř překladač:

```bash
/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-g++ --version
```

Pokud soubor chybí, nejprve zajisti tuto ARM toolchain a uprav cestu
v `export PATH` níže podle jejího skutečného umístění.
Hostitelský `g++` je také potřebný pro generátor Lua bindings.

```bash
sudo apt-get update
sudo apt-get install -y make g++ python3-venv pkg-config
python3 -m venv .venv
.venv/bin/python -m pip install empy==3.3.4 intelhex pexpect future lxml pyserial dronecan
```

Balíčky se instalují do lokálního Python prostředí, nikoli systémovým `pip`.
Složka `.venv` není součástí Gitu; na jiném počítači ji vytvoř znovu.
Výše uvedené závislosti pokrývají zde ověřený build obou ChibiOS desek,
nikoli kompletní prostředí SITL/autotest.

Zkontroluj také submoduly:

```bash
git submodule status --recursive
```

Pokud některý řádek začíná `-`, stáhni chybějící obsah na revizích
předepsaných repozitářem:

```bash
git submodule update --init --recursive
```

### Před každým sestavením v novém terminálu

```bash
cd ~/ardupilot_utb
source .venv/bin/activate
export PATH="/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
python --version
arm-none-eabi-g++ --version
```

Tím aktivuješ Python prostředí a ARM překladač. Waf nikdy nespouštěj
přes `sudo`; administrátorská oprávnění jsou potřeba pouze pro `apt-get`.

## 2. Sestav firmware pro svou desku

Spusť pouze blok pro požadovanou desku. `&&` zabrání sestavení,
pokud konfigurace selže. Waf nespouštěj přes sudo.

### Skystars H7HD s obousměrným DShot

```bash
./waf configure --board SkystarsH7HD-bdshot &&
./waf copter -j4
```

### MicoAir H743 V2 45A AIO AM32

```bash
./waf configure --board MicoAir743v2 &&
./waf copter -j4
```

Pro model MicoAir743v2-AIO-45A určuje
[výrobce](https://micoair.com/flightcontroller_micoair743v2_aio_45a/)
cíl `MicoAir743v2`. Podpora obousměrného DShot je v tomto cíli zahrnutá.
Firmware AM32 regulátorů se tímto postupem nevytváří ani neaktualizuje.

Build musí skončit zprávou `'copter' finished successfully`.
Při nedostatku paměti změň `-j4` na `-j2`.
HEX se vytvoří automaticky.

## 3. Najdi výstup ve Windows

Skystars:

```text
\\wsl$\Ubuntu\home\jirka\ardupilot_utb\build\SkystarsH7HD-bdshot\bin
```

MicoAir:

```text
\\wsl$\Ubuntu\home\jirka\ardupilot_utb\build\MicoAir743v2\bin
```

| Soubor | Použití |
| --- | --- |
| `arducopter_with_bl.hex` | První instalace přes DFU, obsahuje bootloader |
| `arducopter.apj` | Aktualizace přes funkční ArduPilot bootloader |

Soubory mají pro obě desky stejná jména. Vždy použij správnou složku.
Po neúspěšném buildu mohou ve složce zůstat staré soubory.

## 4. Nahraj do FC

Před nahráním sundej vrtule a zálohuj stávající parametry.

- **První instalace/přechod z jiného firmwaru:** připoj USB při stisknutém
  BOOT a nahraj `arducopter_with_bl.hex` přes DFU nástroj podporující Intel
  HEX, například STM32CubeProgrammer.
- **Aktualizace již funkčního ArduPilotu:** nahraj `arducopter.apj`
  přes volbu vlastního firmwaru v Mission Planneru.

Po nahrání nastav rám kvadrokoptéry, proveď kalibrace a ověř RC,
senzory, protokol ESC a pořadí i směr motorů bez vrtulí.
Samotný build nenastaví kompletní konfiguraci konkrétní kvadrokoptéry.

### MicoAir AIO 45A: zkontroluj parametry

Společný cíl MicoAir743v2 má některé výchozí hodnoty pro jinou variantu.
Pro AIO 45A výrobce uvádí:

| Parametr | Hodnota |
| --- | --- |
| `BATT_VOLT_MULT` | 21.12 |
| `BATT_AMP_PERVLT` | 14.14 (výchozí hodnota cíle je 40.2) |
| `OSD_TYPE` | 5 při použití digitálního MSP DisplayPort OSD |

Napětí a proud ověř měřením. Tyto hodnoty nejsou automaticky vložené
do vytvořeného firmwaru. AIO nemá vestavěný kompas.

## Další sestavení

Po úpravě kódu zopakuj kroky 1 a 2 pro požadovanou desku.
Nástroje není potřeba znovu instalovat ani běžně provádět `clean`.
Waf si pamatuje poslední desku, proto při přepínání vždy použij `configure`.

Pokud chybí překladač nebo Python modul, ověř aktivaci prostředí v kroku 1.
Pokud chybí HEX, ověř:

```bash
python3 -c 'import intelhex; print("intelhex OK")'
```

Při chybě `ModuleNotFoundError: No module named 'intelhex'` aktivuj
`source .venv/bin/activate` a případně doinstaluj modul pomocí
`python -m pip install intelhex`. Nestačí pouze spouštět Waf příkazem
`.venv/bin/python ./waf`: pomocný skript pro HEX spouští `python3`
z `PATH`, takže i ten musí patřit do `.venv`.

Potom zopakuj konfiguraci a sestavení; již přeložené objekty se využijí znovu.

## Stav ověření

### Aktuální počítač: 6. 10. 2026

Ověřeno v `/home/jirka/ardupilot_utb`, commit `bd9f46956e`, WSL Ubuntu
26.04.1, Python 3.14.4 v `.venv`, ARM GCC 10.2.1 z `/opt`.
Chybějící systémové a Python závislosti byly doplněny podle kroku 1.

| Cíl | Konfigurace a build Copter | APJ/BIN a HEX/bootloader/BIN |
| --- | --- | --- |
| `SkystarsH7HD-bdshot` | Prošlo | Shoduje se |
| `MicoAir743v2` | Prošlo | Shoduje se |

Obě desky byly sestaveny z nového build adresáře. U Skystars první běh
skončil až při tvorbě HEX kvůli neaktivovanému prostředí; po aktivaci
`.venv` a opakování konfigurace/buildu prošel i tento krok.
Rozbalený obraz APJ byl porovnán s BIN. Obsah HEX byl porovnán
s bootloaderem dané desky, výplní do 128 KiB a aplikací BIN
od adresy `0x08020000`.

Logy v této kopii (adresář `tmp/` není součástí Gitu):

- `tmp/configure-Skystars.log`, `tmp/build-Skystars.log`
  a úspěšné dokončení `tmp/build-Skystars-retry.log`
- `tmp/configure-MicoAir.log`, `tmp/build-MicoAir.log`
- `tmp/verify-firmware.log` — porovnání obrazů a SHA-256 výsledných HEX

Nahrání do FC, test na skutečné desce ani SITL/autotest v tomto ověření
neproběhly. Ověřen je postup sestavení a konzistence výstupních souborů.

### Historický záznam z původního počítače

Oba firmware byly 3. 10. 2026 úspěšně sestaveny z commitu `dbe792162d`.
Obsah HEX byl porovnán s příslušným bootloaderem a aplikací BIN;
rozbalený APJ se také shodoval s BIN. Test na skutečném FC neproběhl.

## Zjednodušený repozitář: kde hledat kód kvadrokoptéry

Samostatné adresáře `Rover`, `ArduPlane`, `ArduSub`,
`AntennaTracker` a `Blimp` byly fyzicky odstraněny.
Sdílené knihovny, nástroje a submoduly zůstávají potřebné pro sestavení.

| Umístění | Co v něm hledat |
| --- | --- |
| `ArduCopter/` | Hlavní kód Copteru, režimy letu a parametry |
| `libraries/AC_AttitudeControl/` | Řízení náklonu, rotace a polohy |
| `libraries/AC_WPNav/` | Navigace po trase |
| `libraries/AP_Motors/` | Výstupy motorů a mixování |
| `libraries/AP_InertialSensor/` | Gyroskopy a akcelerometry |
| `libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD-bdshot/` | Definice desky Skystars |
| `libraries/AP_HAL_ChibiOS/hwdef/MicoAir743v2/` | Definice desky MicoAir |
| `Tools/`, `modules/`, `waf`, `wscript` | Nástroje a závislosti sestavení |
| `build/` | Výsledné firmware |

V ArduCopteru zůstávají i podmíněné části pro vrtulníky.
Cíl `./waf copter` volí `FRAME_CONFIG=MULTICOPTER_FRAME`;
vrtulník má samostatný cíl `heli`. Například obsah `ArduCopter/heli.cpp`
je omezen podmínkou `FRAME_CONFIG == HELI_FRAME`.
Jejich ruční vyřezávání ze sdíleného kódu není pro orientaci ani build nutné.

Tato kopie repozitáře nyní slouží pro Copter. Sestavení odstraněných vozidel
a testy vyžadující jejich zdroje už z této kopie neprovedeme.
Smazání je evidované v Gitu. Původní adresáře lze obnovit:

```bash
git restore --source=bd9f46956e^ -- Rover ArduPlane ArduSub AntennaTracker Blimp
```

Po odstranění ostatních vozidel bylo 3. 10. 2026 pro obě desky znovu
provedeno configure, clean a úplné sestavení Copter. Obě sestavení i kontrola
HEX/APJ prošly. SHA-256 obou HEX se shoduje s původními soubory před úklidem.
Původní návod odkazoval na logy `tmp/trim-build-Skystars.log` a
`tmp/trim-build-MicoAir.log`; tyto logy se do aktuální kopie nepřenesly.


## Doplnění: UTB FÁZE 0 (8. 10. 2026)

Pro ověření skeleton patche byly do existující `.venv` doinstalovány
`pymavlink`, `pytest`, `flake8` a jejich závislosti. Do Ubuntu byl doplněn
`astyle`. ARM toolchain se neměnil. `pytest` není nutný k přímému spuštění
níže uvedeného regresního skriptu.

```bash
cd /home/jirka/ardupilot_utb
source .venv/bin/activate
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
# Jednorázově, pokud tyto nástroje ještě chybí:
python -m pip install pymavlink pytest flake8
sudo apt-get install astyle

./waf configure --board SkystarsH7HD-bdshot --enable-UTB
./waf copter -j4
./waf configure --board SkystarsH7HD-bdshot --disable-UTB
./waf copter -j4

./waf configure --board sitl --enable-UTB
./waf copter --targets tests/test_utb -j4
./build/sitl/tests/test_utb
python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 1 --log-dir tmp/utb-sitl-tests-on
./waf configure --board sitl --disable-UTB
./waf copter -j4
python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 0 --log-dir tmp/utb-sitl-tests-off
```

Oba Skystars buildy, oba SITL buildy, všech 6 C++ testů i lokální SITL
regrese prošly. Firmware nebyl nahrán do FC; hardware ani let nebyl ověřen.
Výchozí `AP_UTB_ENABLED=0` feature vyřadí; při zapnutém buildu je výchozí
`UTB_ENABLE=0` a změna tohoto parametru vyžaduje restart. FÁZE 0 poskytuje
pouze disarmovaný diagnostický režim a neobsahuje aktivní UTB řízení motorů.

Podrobný stav, seznam změn a lokální logy jsou popsány v
[UTB_ARCHITEKTURA_CS.md](docs/UTB_ARCHITEKTURA_CS.md).
Firmware obou variant je uložen v `tmp/utb-artifacts/hw-on/` a
`tmp/utb-artifacts/hw-off/`; adresář `tmp/` není součástí Gitu.


## Doplnění: SHADOW FÁZE 1 (8. 10. 2026)

V této fázi nebylo nic doinstalováno. Použita byla existující `.venv`,
`pymavlink`, `flake8`, `astyle`, GTest submodule a stejný ARM toolchain.
Instalační příkazy v předchozí části popisují historickou FÁZI 0.

Implementován je pouze diagnostický shadow PID, BF_X allocator a logování.
AP nadále řídí všechny skutečné motorové výstupy. `UTB_ACRO` nelze armovat
ani zvolit za letu. Výchozí `UTB_ENABLE=0` a `UTB_SHADOW=0`; ENABLE vyžaduje
restart. Runtime shadow podporuje pouze `FRAME_CLASS=1`, `FRAME_TYPE=12`.
To nepotvrzuje konfiguraci fyzického dronu. Typ 18 je odmítnut.

Opakovatelné příkazy v Ubuntu/WSL (vždy bez sudo pro waf):

```bash
cd /home/jirka/ardupilot_utb
source .venv/bin/activate
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
mkdir -p tmp

./waf configure --board SkystarsH7HD-bdshot --enable-UTB
./waf copter -j4
# Před změnou konfigurace případně zkopírovat build/.../bin/arducopter*.
./waf configure --board SkystarsH7HD-bdshot --disable-UTB
./waf copter -j4

./waf configure --board sitl --enable-UTB
./waf copter --targets tests/test_utb -j4
./build/sitl/tests/test_utb --gtest_output=xml:tmp/utb-unit-results.xml
python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 1 --log-dir tmp/utb-new-skeleton-on
python Tools/autotest/test_utb_shadow.py --binary build/sitl/bin/arducopter --output tmp/utb-new-shadow

./waf configure --board sitl --disable-UTB
./waf copter -j4
python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 0 --log-dir tmp/utb-new-skeleton-off

python -m flake8 Tools/autotest/test_utb_skeleton.py Tools/autotest/test_utb_shadow.py
python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter --no-emit
mkdir -p tmp/utb-new-logger-metadata
(cd tmp/utb-new-logger-metadata && python ../../Tools/autotest/logger_metadata/parse.py --vehicle Copter)
git diff --check
```

Pro testovací output adresáře používat nové názvy. Skripty spouštějí lokální
SITL a neotevírají spojení na fyzickou FC. Během dalšího waf configure/build
je potřeba spouštět testy ze zachované kopie SITL binary, ne přepisovaného cíle.

Zachované výsledky této fáze jsou v `tmp/utb-phase1-artifacts/{hw-on,hw-off,sitl-on,sitl-off}/`.
Podrobné výsledky, log formáty, skutečné naměřené frekvence a omezení jsou
v [UTB_CONTROL_THEORY_CS.md](docs/UTB_CONTROL_THEORY_CS.md), část 24.
`tmp/` je ignorované lokální úložiště, nikoli verzovaný důkaz pro vzdálené CI.
H743 CPU/logger load, fyzický frame, hardware a skutečný let nebyly ověřeny.
FÁZE 2 ani aktivní UTB motorový output nebyly implementovány.


## Hardening SHADOW FÁZE 1

Původní FAST_TASK logger consumer nahradil samostatný HAL I/O worker.
Producer nevolá AP_Logger backend/GCS a nikdy na consumer nečeká.
Použita je pevná čtyřpoložková AP SPSC queue; full znamená diagnostický drop.
Primary gyro změna resetuje pouze UTB PID historii. Nic nebylo doinstalováno.
Build příkazy ON/OFF se nemění; test_utb nyní obsahuje 31 testů, shadow script
navíc ověřuje worker counters, primary gyro switch a truncated-set parser.
Pro log rate test script používá SITL speedup=1 (nativní worker a sim time);
startup health timeout45s dovolí skutečné EKF ustálení při této rychlosti.

Aktuální evidence: `tmp/utb-hardening-*`, firmware
`tmp/utb-hardening-artifacts/{hw-on,hw-off,sitl-on,sitl-off}/`.
Podrobný transport, priority, formáty a finální výsledky jsou v teorii,
oddíl25. Původní `utb-phase1-*` jsou historické výsledky před hardeningem.
Hardware a skutečný let zůstávají neověřené; další fáze nebyla zahájena.


Hardening výsledky: 31/31 unit testů, SITL shadow i FÁZE 0 ON/OFF regrese,
všechny čtyři buildy a source/style/metadata audit prošly. Naměřeno v SITL
200.003738 a 399.326397 complete sets/s, nikoli H743 benchmark.
Softwarový stav: SHADOW FÁZE 1 COMPLETE; hardware a skutečný let neověřeny.
## FÁZE 1H-A — diagnostické sestavení

Nic nebylo doinstalováno. Použit stávající WSL Ubuntu, .venv, waf,
ARM GCC 10.2.1, gtest, astyle a flake8. Před C++ změnami byl ověřen checkpoint
včetně untracked souborů a provedeny baseline build/unit/SITL regrese.
Checkpoint: `tmp/utb-1ha-checkpoint/workspace.tar.gz`, SHA-256
`92bcd530009062832ab8595cbfe43b80c7da795d953668ec6ef8f1a52efae51c`.

```sh
cd /home/jirka/ardupilot_utb
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
# A0: runtime UTB_ENABLE=0, bez instrumentace
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --disable-UTB_BENCH
./waf copter -j4
mkdir -p tmp/bench-session/A0
cp build/SkystarsH7HD-bdshot/bin/arducopter* tmp/bench-session/A0/
sha256sum tmp/bench-session/A0/arducopter*

# A/B/C/D: stejný BENCH ON firmware, odlišné parametry
mkdir -p tmp/utb-1ha
printf 'define AP_UTB_BENCH_ENABLED 1\n' > tmp/utb-1ha/bench-on.hwdef
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH \
  --extra-hwdef "$PWD/tmp/utb-1ha/bench-on.hwdef"
./waf copter -j4
mkdir -p tmp/bench-session/BENCH_ON
cp build/SkystarsH7HD-bdshot/bin/arducopter* tmp/bench-session/BENCH_ON/
sha256sum tmp/bench-session/BENCH_ON/arducopter*
```

A: ENABLE=0. B: ENABLE=1/SHADOW=0. C: ENABLE=1/SHADOW=1/LOG_RATE=200.
D: ENABLE=1/SHADOW=1/LOG_RATE=400. Každý profil začít rebootem s uloženými
parametry; worker vzniká pouze při boot ENABLE=1. Vždy explicitní readback parametrů,
FRAME_CLASS=1/FRAME_TYPE=12, LOG_DISARMED=1 a pro Skystars LOG_BACKEND_TYPE=4.
A0 a BENCH ON nezaměnit; každý APJ archivovat s targetem, flagy, velikostí
a SHA-256. UTB OFF a logging-disabled buildy jsou compile guard kontroly,
nikoli profily pro shadow benchmark. MicoAir používá samostatný target
MicoAir743v2 a SDMMC logger typ 1; podrobné rozdíly jsou v bench dokumentu.

Přesné ostatní build/test příkazy a první DISARMED postup bez vrtulí:
[UTB_HARDWARE_BENCH_CS.md](docs/UTB_HARDWARE_BENCH_CS.md).
Výsledky a manifest: [software report](docs/UTB_BENCH_SOFTWARE_REPORT_CS.md).
Implementována je pouze diagnostika. Unit/SITL výsledky jsou oddělené
od dosud neověřených H743 časů, CPU/logger zátěže a stack rezervy.
Firmware nebyl nahrán na FC; aktivní UTB_ACRO a FÁZE 2 nejsou povoleny.
