# ArduPilot UTB – instalace prostředí, Git a build firmware ve WSL

Tento návod popisuje kompletní zprovoznění projektu `ardupilot_utb` na čistém Windows počítači nebo notebooku.

Závislosti byly doplněny a build ověřen dne 6. 10. 2026.

Cílové prostředí:

- Windows 10/11
- WSL2
- Ubuntu
- Git + SSH přístup na GitHub
- repozitář `jirkka/ardupilot_utb`
- větev `ardupilot-4.7.1-utb`
- GNU Arm Embedded Toolchain `10-2020-q4-major` / GCC 10.2.1
- build ArduCopter firmware přes `waf`

Repozitář vychází z ArduPilot Copter 4.7.1, původně z commitu `dbe792162d06cab66c3475fd5556bf7a120f119e`, nad kterým jsou vlastní UTB změny.

---

# 1. Instalace WSL2 a Ubuntu

Otevři **PowerShell jako správce** a spusť:

```powershell
wsl --install
```

Po dokončení restartuj Windows.

Po restartu ověř stav:

```powershell
wsl -l -v
```

Požadovaný stav je přibližně:

```text
  NAME      STATE           VERSION
* Ubuntu    Stopped         2
```

Pokud by Ubuntu používalo WSL1:

```powershell
wsl --set-version Ubuntu 2
```

WSL spustíš příkazem:

```powershell
wsl
```

Při prvním spuštění Ubuntu vytvoř Linux uživatele a heslo.

---

# 2. Aktualizace Ubuntu a základní nástroje

V Ubuntu spusť:

```bash
sudo apt update
sudo apt upgrade -y
```

Nainstaluj základní balíčky:

```bash
sudo apt install -y \
    git \
    openssh-client \
    wget \
    bzip2 \
    python3 \
    python3-pip \
    python3-venv
```

Ověř:

```bash
git --version
python3 --version
```

Poznámka: Na tomto notebooku byl build obou desek ověřen s Pythonem 3.14.4 v lokálním prostředí `.venv`. Jeho příprava a závislosti jsou uvedené v části 6.

---

# 3. Nastavení Gitu

Nastav autora commitů:

```bash
git config --global user.name "Jiri Kovar"
git config --global user.email "jirikovar215@seznam.cz"
```

Pro Linuxové konce řádků:

```bash
git config --global core.autocrlf input
```

Kontrola:

```bash
git config --global --list
```

`user.name` a `user.email` určují autora commitů. Nejde o přihlašovací jméno a heslo na GitHub.

---

# 4. SSH klíč pro GitHub

Každý nový PC/notebook je vhodné vybavit vlastním SSH klíčem.

Nejdřív ověř, zda už klíč existuje:

```bash
ls ~/.ssh
```

Pokud adresář nebo `id_ed25519` neexistuje, vytvoř nový klíč:

```bash
ssh-keygen -t ed25519 -C "jirikovar215@seznam.cz"
```

Na otázku:

```text
Enter file in which to save the key (.../.ssh/id_ed25519):
```

stačí stisknout **Enter**.

Passphrase může být prázdná, pokud ji nechceš používat.

Veřejný klíč zobraz:

```bash
cat ~/.ssh/id_ed25519.pub
```

Celý řádek začínající `ssh-ed25519` vlož na GitHubu:

```text
Settings → SSH and GPG keys → New SSH key
```

Doporučené názvy:

- PC: `WSL PC`
- notebook: `WSL Notebook`

Typ klíče:

```text
Authentication Key
```

Otestuj spojení:

```bash
ssh -T git@github.com
```

Správný výsledek je přibližně:

```text
Hi jirkka! You've successfully authenticated, but GitHub does not provide shell access.
```

---

# 5. Stažení repozitáře ArduPilot UTB

Repozitář ukládej přímo do Linuxového filesystemu WSL, ne na `C:\`.

```bash
cd ~
git clone --recurse-submodules git@github.com:jirkka/ardupilot_utb.git
cd ~/ardupilot_utb
```

Přepni se na UTB větev:

```bash
git switch ardupilot-4.7.1-utb
```

Aktualizuj submoduly:

```bash
git submodule update --init --recursive
```

Přidej oficiální ArduPilot jako `upstream`:

```bash
git remote add upstream https://github.com/ArduPilot/ardupilot.git
```

Pokud už `upstream` existuje, tento krok vynech.

Kontrola:

```bash
git remote -v
```

Požadovaný stav:

```text
origin    git@github.com:jirkka/ardupilot_utb.git
upstream  https://github.com/ArduPilot/ardupilot.git
```

Ověř větev a pracovní strom:

```bash
git status
git branch --show-current
git log -1 --oneline
git submodule status
```

Aktivní větev musí být:

```text
ardupilot-4.7.1-utb
```

U `git submodule status` by před SHA neměl být znak `+`.

---

# 6. Instalace ArduPilot závislostí

V repozitáři spusť:

```bash
cd ~/ardupilot_utb
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Waf ani instalační skript nespouštěj celý přes `sudo`.

Po dokončení načti profil:

```bash
. ~/.profile
```

Potom znovu synchronizuj submoduly:

```bash
git submodule update --init --recursive
```

## Poznámka pro Ubuntu 26.04

ArduPilot 4.7.1 je starší než Ubuntu 26.04. Instalační skript proto nemusí vždy automaticky nainstalovat přesně požadovaný STM32 toolchain. ARM GCC se proto níže instaluje explicitně.


## Doplnění závislostí na notebooku při ověření 6. 10. 2026

Při prvním pokusu o konfiguraci chyběl `make`. Byly provedeny následující kroky:

```bash
sudo apt-get update
sudo apt-get install -y make g++ python3-venv pkg-config
cd ~/ardupilot_utb
python3 -m venv .venv
.venv/bin/python -m pip install empy==3.3.4 intelhex pexpect future lxml pyserial dronecan
```

Nově se nainstalovaly `make`, hostitelský `g++`, `python3-venv` a jejich systémové
závislosti. `pkg-config` už byl přítomný. Hostitelský překladač je potřebný
pro generátor Lua bindings; firmware překládá ARM GCC.
Python balíčky se nainstalovaly do nové projektové `.venv`, včetně nepřímé
závislosti `ptyprocess`. Systémový `pip` k tomu není potřeba.

ARM GCC 10.2.1 už byl v `/opt/gcc-arm-none-eabi-10-2020-q4-major`;
při tomto ověření se znovu nestahoval ani neinstaloval. Instalační skript
`install-prereqs-ubuntu.sh` nebyl při tomto doplnění spouštěn.
Výše uvedené závislosti byly ověřeny pro obě ChibiOS desky z tohoto návodu,
nikoli jako úplná instalace prostředí SITL/autotest.

`.venv` není součástí Gitu. Na dalším počítači ji vytvoř znovu;
nekopíruj ji mezi PC a notebookem.

## Aktivace před buildem v každém novém terminálu

```bash
cd ~/ardupilot_utb
source .venv/bin/activate
export PATH="/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
python3 --version
arm-none-eabi-g++ --version
```

Pokud generování HEX skončí na `ModuleNotFoundError: No module named 'intelhex'`,
ověř aktivaci prostředí a modul:

```bash
source .venv/bin/activate
python3 -c 'import intelhex; print("intelhex OK")'
```

Pokud modul v `.venv` chybí, doinstaluj jej pomocí `python3 -m pip install intelhex`.
Samotné spuštění `.venv/bin/python ./waf` nestačí: pomocný skript pro HEX
používá `python3` z `PATH`. Po aktivaci znovu spusť konfiguraci a build;
již přeložené objekty se použijí znovu.

---

# 7. Instalace správného ARM GCC

Pro tento projekt používej:

```text
GNU Arm Embedded Toolchain 10-2020-q4-major
GCC 10.2.1
```

Nepoužívej pro tento projekt balíček:

```bash
sudo apt install gcc-arm-none-eabi
```

Na novém Ubuntu může nainstalovat výrazně novější GCC, například GCC 14.x.

## Stažení toolchainu

```bash
cd /tmp
wget https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
```

Pokud `tar` hlásí, že chybí `bzip2`:

```bash
sudo apt install bzip2 -y
```

Rozbal toolchain do `/opt`:

```bash
sudo tar -xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -C /opt
```

Ověř:

```bash
ls -ld /opt/gcc-arm-none-eabi-10-2020-q4-major
```

Přidej compiler do `PATH`:

```bash
echo 'export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH' >> ~/.profile
. ~/.profile
```

Ověř:

```bash
which arm-none-eabi-gcc
arm-none-eabi-gcc --version
```

Požadovaná cesta:

```text
/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gcc
```

Požadovaná verze:

```text
arm-none-eabi-gcc (GNU Arm Embedded Toolchain 10-2020-q4-major) 10.2.1
```

---

# 8. Rozdíl cest PC a notebook

V současném prostředí jsou používány dva Linux uživatelské účty.

## PC

Linux uživatel:

```text
jirkka
```

Repozitář:

```text
/home/jirkka/ardupilot_utb
```

Windows cesta:

```text
\\wsl$\Ubuntu\home\jirkka\ardupilot_utb
```

## Notebook (NT)

Linux uživatel:

```text
jirka
```

Repozitář:

```text
/home/jirka/ardupilot_utb
```

Windows cesta:

```text
\\wsl$\Ubuntu\home\jirka\ardupilot_utb
```

Do Git Extensions otevři přímo tuto WSL cestu. Nevytvářej kvůli Git Extensions druhou kopii repozitáře na `C:\`.

---

# 9. Ověření build prostředí

```bash
cd ~/ardupilot_utb
source .venv/bin/activate
export PATH="/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
```

Ověř:

```bash
git status
git branch --show-current
arm-none-eabi-gcc --version
python3 --version
git submodule status
```

Dále ověř dostupné Skystars desky:

```bash
./waf list_boards | grep -i skystars
```

V projektu jsou mimo jiné:

```text
SkystarsH7HD
SkystarsH7HD-bdshot
SkystarsH7HDv2
```

---

# 10. Build firmware

Waf nespouštěj přes `sudo`.

## Skystars H7HD s obousměrným DShot

```bash
cd ~/ardupilot_utb
source .venv/bin/activate
export PATH="/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board SkystarsH7HD-bdshot &&
./waf copter -j4
```

Pokud je nedostatek RAM:

```bash
./waf copter -j2
```

Build musí skončit zprávou přibližně:

```text
'copter' finished successfully
```

## MicoAir H743 V2 45A AIO AM32

```bash
cd ~/ardupilot_utb
source .venv/bin/activate
export PATH="/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board MicoAir743v2 &&
./waf copter -j4
```

Firmware AM32 regulátorů se tímto postupem nevytváří ani neaktualizuje.

---

# 11. Výstupní firmware

## Skystars

Linux:

```text
~/ardupilot_utb/build/SkystarsH7HD-bdshot/bin/
```

PC přes Windows:

```text
\\wsl$\Ubuntu\home\jirkka\ardupilot_utb\build\SkystarsH7HD-bdshot\bin
```

Notebook přes Windows:

```text
\\wsl$\Ubuntu\home\jirka\ardupilot_utb\build\SkystarsH7HD-bdshot\bin
```

## MicoAir

Linux:

```text
~/ardupilot_utb/build/MicoAir743v2/bin/
```

PC přes Windows:

```text
\\wsl$\Ubuntu\home\jirkka\ardupilot_utb\build\MicoAir743v2\bin
```

Notebook přes Windows:

```text
\\wsl$\Ubuntu\home\jirka\ardupilot_utb\build\MicoAir743v2\bin
```

Hlavní soubory:

| Soubor | Použití |
| --- | --- |
| `arducopter_with_bl.hex` | První instalace / DFU, obsahuje bootloader |
| `arducopter.apj` | Aktualizace přes funkční ArduPilot bootloader |

Po neúspěšném buildu mohou ve složce zůstat staré výstupní soubory. Vždy ověř čas vytvoření výsledného firmware.

---

# 12. Nahrání firmware do FC

Před nahráním:

- sundej vrtule,
- zálohuj parametry,
- ověř správný cílový board.

## První instalace / přechod z jiného firmware

Připoj FC při stisknutém BOOT tlačítku a nahraj:

```text
arducopter_with_bl.hex
```

například přes STM32CubeProgrammer.

## Aktualizace již funkčního ArduPilotu

Nahraj:

```text
arducopter.apj
```

přes vlastní firmware v Mission Planneru.

Po flashi znovu ověř:

- senzory,
- kalibrace,
- RC,
- GPS,
- kompas,
- ESC protokol,
- pořadí motorů,
- směr motorů,
- failsafe.

Kontrolu motorů prováděj bez vrtulí.

---

# 13. Git workflow mezi PC a notebookem

Před začátkem práce:

```bash
cd ~/ardupilot_utb
git pull
git submodule update --init --recursive
```

Po změnách:

```bash
git status
git add .
git commit -m "Popis změny"
git push
```

Na druhém počítači:

```bash
cd ~/ardupilot_utb
git pull
git submodule update --init --recursive
```

`origin` je vlastní UTB repozitář:

```text
git@github.com:jirkka/ardupilot_utb.git
```

`upstream` je oficiální ArduPilot:

```text
https://github.com/ArduPilot/ardupilot.git
```

---

# 14. Po restartu Windows

WSL nemusí běžet trvale na pozadí.

Spusť jej z PowerShellu:

```powershell
wsl
```

Potom:

```bash
cd ~/ardupilot_utb
```

Ověř ARM compiler:

```bash
arm-none-eabi-gcc --version
```

Před dalším buildem aktivuj také Python prostředí:

```bash
source .venv/bin/activate
```

Protože cesta k GCC je zapsaná v `~/.profile`, měla by se načíst automaticky.

Pokud ne:

```bash
. ~/.profile
```

---

# 15. Git Extensions ve Windows

Repozitář není potřeba klonovat znovu ve Windows.

Otevři existující WSL repozitář.

PC:

```text
\\wsl$\Ubuntu\home\jirkka\ardupilot_utb
```

Notebook:

```text
\\wsl$\Ubuntu\home\jirka\ardupilot_utb
```

Git Extensions může sloužit pro:

- prohlížení historie,
- diff,
- stage,
- commit,
- checkout branchí,
- push/pull.

Samotný build firmware prováděj v Ubuntu/WSL.

---

# 16. Zjednodušený repozitář – kde hledat kód

V UTB kopii byly odstraněny samostatné adresáře ostatních vozidel:

- `Rover`
- `ArduPlane`
- `ArduSub`
- `AntennaTracker`
- `Blimp`

Sdílené knihovny, nástroje a submoduly zůstávají potřeba pro build Copteru.

| Umístění | Obsah |
| --- | --- |
| `ArduCopter/` | hlavní kód Copteru, režimy letu a parametry |
| `libraries/AC_AttitudeControl/` | attitude / rate řízení |
| `libraries/AC_WPNav/` | navigace po trase |
| `libraries/AP_Motors/` | motorové výstupy a mixování |
| `libraries/AP_InertialSensor/` | gyroskopy a akcelerometry |
| `libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD-bdshot/` | definice Skystars H7HD bdshot |
| `libraries/AP_HAL_ChibiOS/hwdef/MicoAir743v2/` | definice MicoAir743v2 |
| `Tools/` | build a pomocné nástroje |
| `modules/` | submoduly |
| `waf`, `wscript` | build systém |
| `build/` | výsledné firmware |

Původní odstraněné adresáře lze z historie Gitu obnovit například:

```bash
git restore --source=bd9f46956e^ -- Rover ArduPlane ArduSub AntennaTracker Blimp
```

---

# 17. Rychlý checklist čistého Windows

Na novém počítači je potřeba:

1. WSL2
2. Ubuntu
3. aktualizace Ubuntu
4. Git
5. OpenSSH klient
6. SSH klíč přidaný na GitHub
7. clone `jirkka/ardupilot_utb`
8. checkout `ardupilot-4.7.1-utb`
9. Git submoduly
10. ArduPilot prerequisites
11. `bzip2` + `wget`
12. GNU Arm GCC 10.2.1 v `/opt`
13. GCC cesta v `~/.profile`
14. `make`, hostitelský `g++`, `python3-venv` a `pkg-config`
15. vytvoření `.venv` a instalace Python balíčků podle části 6
16. aktivace `.venv` a kontrola `./waf list_boards`
17. `./waf configure --board ...`
18. `./waf copter`
19. výsledný `.apj` / `.hex` v `build/<board>/bin/`

Po dokončení tohoto checklistu je počítač připravený pro úpravy a lokální build UTB ArduCopter firmware.


---

# 18. Výsledek ověření na notebooku 6. 10. 2026

Prostředí: `/home/jirka/ardupilot_utb`, commit `bd9f46956e`, WSL Ubuntu 26.04.1,
Python 3.14.4 v `.venv`, ARM GCC 10.2.1 z `/opt`.

| Deska | Konfigurace a build Copter | Kontrola výstupů |
| --- | --- | --- |
| `SkystarsH7HD-bdshot` | Prošlo | APJ/BIN a HEX/bootloader/BIN se shodují |
| `MicoAir743v2` | Prošlo | APJ/BIN a HEX/bootloader/BIN se shodují |

Obě desky byly sestaveny z nového build adresáře. U Skystars první build
skončil při tvorbě HEX kvůli neaktivované `.venv`; po aktivaci a opakování
konfigurace/buildu prošel i tento krok. Zdrojový kód ani submoduly se neměnily.

Kontrola zahrnovala porovnání rozbaleného obrazu APJ s BIN a obsahu HEX
s příslušným bootloaderem, výplní do 128 KiB a aplikací BIN od `0x08020000`.
Nahrání do FC, test na skutečné desce ani SITL/autotest neproběhly.

Záznamy jsou lokálně v adresáři `tmp/`, který není součástí Gitu:

- `tmp/configure-Skystars.log` a `tmp/build-Skystars.log`
- `tmp/build-Skystars-retry.log` — úspěšné dokončení Skystars
- `tmp/configure-MicoAir.log` a `tmp/build-MicoAir.log`
- `tmp/verify-firmware.log` — kontrola obrazů a SHA-256 výsledných HEX
