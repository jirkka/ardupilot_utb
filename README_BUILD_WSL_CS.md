# ArduCopter pro kvadrokoptéru: sestavení a nahrání

Návod pro WSL Ubuntu a tuto kopii projektu zaměřenou na Copter. Vytvořeno s asistencí AI.

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
