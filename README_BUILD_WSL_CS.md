# ArduCopter pro kvadrokoptéru: sestavení a nahrání

Návod pro připravené prostředí v tomto projektu. Vytvořeno s asistencí AI.

## 1. Otevři WSL

V PowerShellu:

```powershell
wsl -d Ubuntu
```

Další příkazy patří do terminálu Ubuntu:

```bash
cd /home/jirkka/ardupilot_utb
source .venv/bin/activate
export PATH="$PWD/tmp/toolchains/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
```

Tím aktivuješ připravený Python a ARM překladač.
Složky `.venv` a `tmp/toolchains` jsou potřebné pro další sestavení.

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
\\wsl$\Ubuntu\home\jirkka\ardupilot_utb\build\SkystarsH7HD-bdshot\bin
```

MicoAir:

```text
\\wsl$\Ubuntu\home\jirkka\ardupilot_utb\build\MicoAir743v2\bin
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

Potom zopakuj konfiguraci a sestavení.

## Stav ověření

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
git restore -- Rover ArduPlane ArduSub AntennaTracker Blimp
```

Po odstranění ostatních vozidel bylo 3. 10. 2026 pro obě desky znovu
provedeno configure, clean a úplné sestavení Copter. Obě sestavení i kontrola
HEX/APJ prošly. SHA-256 obou HEX se shoduje s původními soubory před úklidem.
Logy jsou v `tmp/trim-build-Skystars.log` a `tmp/trim-build-MicoAir.log`.
