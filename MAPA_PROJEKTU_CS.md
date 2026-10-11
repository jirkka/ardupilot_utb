# Mapa projektu ArduPilot pro kvadrokoptéru

Průvodce aktuální kopií `ardupilot_utb`, ověřený podle místních zdrojů
4. 10. 2026. Vytvořeno s asistencí AI.
Postup kompilace je v [README_BUILD_WSL_CS.md](README_BUILD_WSL_CS.md).

Tento dokument je orientační mapa, nikoli seznam souborů vhodných ke smazání.
Přítomnost knihovny na disku neznamená, že je celá součástí výsledného HEX.
Výběr závisí na cíli, desce a podmínkách kompilace.

## 1. Kde začít číst

Doporučené pořadí:

1. [ArduCopter/Copter.h](ArduCopter/Copter.h) — hlavní třída a její komponenty.
2. [ArduCopter/Copter.cpp](ArduCopter/Copter.cpp) — tabulka úloh a propojení hlavního běhu.
3. [ArduCopter/mode_stabilize.cpp](ArduCopter/mode_stabilize.cpp) — přehledný příklad převodu povelů pilota na požadavky regulátorům.
4. [AC_AttitudeControl_Multi.cpp](libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp) — regulace pro multikoptéru.
5. [AP_MotorsMatrix.cpp](libraries/AP_Motors/AP_MotorsMatrix.cpp) — rozdělení požadavků mezi motory.
6. [RCOutput.cpp](libraries/AP_HAL_ChibiOS/RCOutput.cpp) — realizace výstupů na mikrokontroléru.

Při čtení Stabilize hledej metodu `ModeStabilize::run()`.
V `Copter.cpp` začni u `scheduler_tasks`, nikoli u dlouhého úvodního komentáře.

## 2. Kořen projektu

| Složka nebo soubor | Co obsahuje a kdy sem jít |
| --- | --- |
| `ArduCopter/` | Kód vozidla: režimy letu, armování, obsluha pilota, propojení knihoven. Hlavní místo pro orientaci v Copteru. |
| `libraries/` | Sdílené moduly: regulace, senzory, navigace, protokoly, matematika a hardware. |
| `Tools/` | Sestavení, bootloadery, simulace, testovací a vývojové nástroje. |
| `modules/` | Externí závislosti jako Git submoduly. Neupravovat ručně v rámci tohoto projektu. |
| `build/` | Generované soubory a firmware pro jednotlivé desky. Úpravy zdrojů patří jinam. |
| `tmp/` | Lokální ARM toolchain, logy našich sestavení a kontrolní skripty. Nemaž celý adresář: obsahuje překladač. |
| `.venv/` | Python prostředí potřebné pro zde připravené sestavení. |
| `tests/` | Testy a podpora testování projektu; například Python testy spouštěné přes pytest a `AP_gtest.h`. |
| `benchmarks/` | Výkonnostní měření; další benchmarky mohou být uvnitř knihoven. |
| `docs/` | Podklady a skripty pro generování dokumentace zdrojového kódu pomocí Doxygenu. |
| `.vscode/` | Nastavení a doporučená rozšíření editoru. |
| `.github/` | Automatické kontroly, šablony a pravidla příspěvků na GitHubu. |
| `.git/` | Historie a interní data Gitu. |
| `waf` | Spouštěcí skript sestavovacího systému. |
| `wscript` | Kořenová konfigurace sestavení; propojuje desky, knihovny a cíle. |
| `BUILD.md` | Obecný návod projektu k sestavení. |
| `README.md` | Původní představení projektu ArduPilot. |
| `README_BUILD_WSL_CS.md` | Náš konkrétní postup sestavení a nahrání pro Skystars a MicoAir. |
| `AGENTS.md`, `CODE_OF_CONDUCT.md`, `COPYING.txt` | Pravidla přispívání, chování a licence. |
| `Dockerfile`, `Vagrantfile`, `Makefile` | Alternativní vývojová prostředí a pomocné cíle. Pro náš postup ve WSL je běžně nepotřebuješ otevírat. |
| `.editorconfig`, `.flake8`, `.pre-commit-config.yaml`, `pyproject.toml` | Nastavení stylu a vývojových kontrol. |

Samostatné složky Rover, ArduPlane, ArduSub, AntennaTracker a Blimp jsme
z této pracovní kopie odstranili. Sdílené knihovny a testovací nástroje
proto stále mohou obsahovat odkazy na jiná vozidla.

## 3. ArduCopter: hlavní kód vozidla

V této tabulce jsou názvy relativní ke složce `ArduCopter/`.

| Soubor / skupina | Účel |
| --- | --- |
| `Copter.h` | Deklarace hlavní třídy Copter, členů a vazeb na moduly. |
| `Copter.cpp` | Plánované úlohy, hlavní propojení běhu, instance vozidla a vstup přes HAL. |
| `system.cpp` | Inicializace systémů Copteru. |
| `rate_thread.cpp` | Volitelná rychlá větev zpracování gyra, rate regulace a motorových výstupů. |
| `mode.h`, `mode.cpp` | Rozhraní letových režimů, výběr a přepínání režimů, společné pomocné funkce. |
| `mode_stabilize.cpp` | Stabilizace náklonu; pilot zadává požadavky na náklon, yaw a plyn. |
| `mode_acro.cpp` | Akrobatický režim se zadáváním požadovaných rotačních rychlostí. |
| `mode_althold.cpp` | Režim držení výšky. |
| `mode_loiter.cpp`, `mode_poshold.cpp` | Režimy držení polohy a jejich reakce na vstup pilota. |
| `mode_auto.cpp` | Provádění automatické mise. |
| `mode_guided.cpp`, `mode_guided_nogps.cpp`, `mode_guided_custom.cpp` | Řízení přes externí požadavky v režimech Guided. |
| `mode_rtl.cpp`, `mode_smart_rtl.cpp` | Návrat domů a návrat využívající uloženou cestu. |
| `mode_land.cpp`, `takeoff.cpp` | Přistávání a společná logika vzletu. |
| `mode_autotune.cpp`, `tuning.cpp` | Integrace automatického ladění a ladění parametrů. |
| Ostatní `mode_*.cpp` | Další samostatné režimy: například Brake, Circle, Follow, Turtle a SystemID. |
| `radio.cpp`, `RC_Channel_Copter.*` | Zpracování RC vstupů a funkcí přepínačů pro Copter. |
| `Attitude.cpp`, `autoyaw.cpp` | Pomocné výpočty požadavků na orientaci a automatické řízení yaw. |
| `motors.cpp`, `motor_test.cpp`, `esc_calibration.cpp` | Propojení motorů s Copterem, test motorů a kalibrace ESC tam, kde ji protokol používá. |
| `sensors.cpp`, `inertia.cpp` | Propojení senzorů a inerciální navigace s vozidlem. |
| `navigation.cpp`, `terrain.cpp`, `surface_tracking.cpp` | Navigační obsluha, terén a sledování vzdálenosti od povrchu. |
| `Parameters.h`, `Parameters.cpp` | Parametry specifické pro Copter a registrace skupin parametrů knihoven. |
| `config.h`, `defines.h`, `APM_Config.h` | Překladová konfigurace a definice. Nejde o uložené uživatelské parametry FC. |
| `AP_Arming_Copter.*`, `takeoff_check.cpp` | Kontroly připravenosti, armování a vzletu. |
| `failsafe.cpp`, `events.cpp`, `ekf_check.cpp` | Reakce na selhání a kontrola odhadu stavu. |
| `land_detector.cpp`, `crash_check.cpp` | Rozpoznání přistání a havárie. |
| `fence.cpp`, `avoidance.cpp`, `precision_landing.cpp` | Propojení geofence, vyhýbání a přesného přistání. |
| `GCS_Copter.*`, `GCS_MAVLink_Copter.*` | Komunikace a MAVLink zprávy specifické pro Copter. |
| `Log.cpp` | Logované údaje specifické pro Copter. |
| `UserCode.cpp`, `UserParameters.*`, `UserVariables.h` | Připravená místa pro uživatelské rozšíření podle konfigurace. |
| `heli.cpp`, `mode_*_heli.cpp`, `mode_autorotate.cpp` | Podpora vrtulníků, oddělená podmínkami kompilace. |
| `wscript` | Knihovny vozidla a cíle `copter` / `heli`. |
| `version.h`, `ReleaseNotes.txt` | Identifikace verze a poznámky k vydáním. |

Pozor: parametry knihoven se často registrují přímo v jejich `.cpp`,
nikoli celé v `ArduCopter/Parameters.cpp`.

## 4. Libraries: moduly důležité pro kvadrokoptéru

Názvy v tabulkách odpovídají podsložkám `libraries/`.
`AP_` obvykle označuje obecný modul, `AC_` modul používaný zejména
Copterem a `AR_` moduly pro Rover. Prefix není záruka výhradního použití.

### Regulace, pohyb a motory

| Knihovna | Co zajišťuje |
| --- | --- |
| `AC_AttitudeControl` | Řízení orientace a rotačních rychlostí; obsahuje také `AC_PosControl.*` pro polohu, rychlost a výšku. |
| `AC_PID`, `PID` | Implementace PID regulátorů a souvisejících pomocných tříd. Pro Copter začni u AC_PID. |
| `AC_WPNav` | Navigace multikoptéry po bodech a pomocné řízení Loiter/Circle. |
| `AP_Motors` | Mixování roll/pitch/yaw/throttle a správa motorových výstupů pro různé rámy. |
| `AC_AutoTune` | Algoritmy automatického ladění regulátorů. |
| `AC_CustomControl` | Rozhraní a implementace volitelného vlastního regulátoru. |
| `AC_InputManager` | Pomocné zpracování řídicích vstupů, včetně specifik některých typů rámů. |
| `Filter` | Dolní propusti, notch a další filtry používané při zpracování signálů. |
| `AP_GyroFFT` | Frekvenční analýza gyra používaná například pro řízení notch filtrů. |
| `AP_Quicktune` | Další podpora automatického ladění podle dostupné konfigurace. |
| `SRV_Channel` | Mapování funkcí na výstupní kanály a správa servo/motorových výstupů. |

V `AP_Motors/` jsou pro běžnou kvadrokoptéru nejzajímavější
`AP_MotorsMatrix.*`, `AP_MotorsMulticopter.*` a
`AP_Motors_Thrust_Linearization.*`.
Soubory pro Heli, Tri nebo jiné rámy jsou alternativní implementace.

### Senzory a odhad stavu

| Knihovna | Co zajišťuje |
| --- | --- |
| `AP_InertialSensor` | Gyroskopy a akcelerometry: ovladače IMU, vzorkování a filtrace. |
| `AP_AccelCal` | Kalibrace akcelerometrů. |
| `AP_Compass` | Magnetometry, jejich ovladače a kalibrace. |
| `AP_Baro` | Barometry a zpracování tlaku/výšky. |
| `AP_GPS` | GPS/GNSS přijímače a jejich protokoly. |
| `AP_AHRS` | Rozhraní odhadu orientace a navigačního stavu. |
| `AP_NavEKF3`, `AP_NavEKF2`, `AP_NavEKF` | Implementace a společné části odhadu stavu pomocí EKF. Aktivní varianta závisí na konfiguraci. |
| `AP_InertialNav` | Přístup k inerciálnímu odhadu polohy a rychlosti pro vyšší vrstvy. |
| `AP_RangeFinder`, `AP_SurfaceDistance` | Dálkoměry a zpracování vzdálenosti od povrchu. |
| `AP_OpticalFlow`, `AP_VisualOdom` | Optický tok a vizuální odometrie. |
| `AP_ExternalAHRS`, `AP_Beacon` | Externí navigační jednotky a systémy určování polohy pomocí majáků. |
| `AP_BattMonitor` | Napětí, proud, spotřebovaná kapacita a ovladače bateriových monitorů. |
| `AP_RPM`, `AP_ESC_Telem` | Otáčky a telemetrie regulátorů ESC. |
| `AP_TemperatureSensor`, `AP_TempCalibration` | Teplotní senzory a podpora teplotní kalibrace. |
| `AP_Proximity` | Senzory vzdáleností k překážkám v okolí. |

### Navigace a autonomní funkce

| Knihovna | Co zajišťuje |
| --- | --- |
| `AP_Mission` | Ukládání a zpracování příkazů mise. Provedení příkazů pro Copter propojuje mode_auto. |
| `AC_Fence` | Geografická a výšková omezení prostoru letu. |
| `AC_Avoidance`, `AP_Avoidance` | Různé vrstvy vyhýbání překážkám a kolizím. |
| `AC_PrecLand`, `AP_IRLock` | Přesné přistání a podpora příslušných snímačů/cílů. |
| `AP_Terrain` | Databáze výšky terénu. |
| `AP_SmartRTL` | Uchování a zpracování cesty pro SmartRTL. |
| `AP_Follow`, `AP_Rally` | Sledování cíle a správa rally bodů. |
| `AP_ADSB`, `AP_AIS` | Informace o okolním leteckém/lodním provozu. |
| `AP_ExternalControl` | Rozhraní pro externě zadávané řízení. |

### RC, komunikace a zobrazení

| Knihovna | Co zajišťuje |
| --- | --- |
| `RC_Channel`, `AP_RCMapper` | Zpracování RC kanálů, rozsahů, funkcí a mapování os. |
| `AP_RCProtocol` | Dekódování podporovaných protokolů RC přijímačů. |
| `GCS_MAVLink` | Společná komunikace MAVLink s Mission Plannerem a dalšími systémy. |
| `AP_SerialManager` | Přiřazení protokolů a nastavení sériovým portům. |
| `AP_CRSF`, `AP_RCTelemetry`, `AP_Frsky_Telem`, `AP_Hott_Telem`, `AP_IBus_Telem` | Podpora RC protokolů a telemetrie do vysílaček. |
| `AP_LTM_Telem`, `AP_Devo_Telem`, `AP_NMEA_Output` | Další telemetrické a výstupní formáty. |
| `AP_CANManager`, `AP_DroneCAN` | Správa CAN sběrnic a zařízení DroneCAN. |
| `AP_MSP`, `AP_OSD`, `AP_VideoTX` | MSP/DisplayPort, obrazové menu OSD a komunikace s VTX. |
| `AP_Notify`, `AP_SerialLED` | Signalizace stavů: LED, bzučák a adresovatelné LED. |
| `AP_Networking`, `AP_DDS` | Síťová komunikace a volitelná integrace DDS/ROS 2. |

### Základ systému, ukládání a rozšíření

| Knihovna | Co zajišťuje |
| --- | --- |
| `AP_Vehicle` | Společný základ tříd vozidel. |
| `AP_Scheduler` | Plánování úloh a sledování jejich časování. |
| `AP_Param` | Definice, vyhledávání, načítání a ukládání parametrů. |
| `AP_Arming`, `AP_AdvancedFailsafe` | Společné kontroly armování a volitelné rozšířené failsafe funkce. |
| `AP_Logger` | Palubní logy a jejich úložiště. Nejde o HEX firmware. |
| `AP_Math`, `AP_Common` | Vektory, matice, rotace, základní datové typy a pomocné funkce. |
| `AP_BoardConfig` | Společná konfigurace a inicializace specifik desky. |
| `AP_Filesystem`, `StorageManager` | Souborový systém a členění trvalého úložiště. |
| `AP_FlashIface`, `AP_FlashStorage`, `AP_RAMTRON` | Přístup k nevolatilním pamětem. |
| `AP_ROMFS` | Soubory vložené do obrazu firmwaru. |
| `AP_InternalError`, `AP_CheckFirmware` | Interní diagnostika a kontroly firmwaru. |
| `AP_Scripting` | Lua skripty, vazby na firmware a příklady/applety. |
| `AP_DAL` | Vrstva přístupu k datům využívaná například při přehrávání logů odhadu stavu. |

### Volitelné vybavení a další specializované knihovny

Při prvním čtení je můžeš přeskočit, pokud danou funkci nepoužíváš.

| Skupina | Složky / účel |
| --- | --- |
| Kamera a gimbal | `AP_Camera`, `AP_Mount`, `AP_ONVIF`. |
| Přídavné mechanismy | `AP_Gripper`, `AP_Winch`, `AP_Parachute`, `AP_LandingGear`, `AC_Sprayer`. |
| Spínání a serva | `AP_Relay`, `AP_ServoRelayEvents`, `AP_Servo_Telem`, `AP_SBusOut`, `AP_RobotisServo`, `AP_Volz_Protocol`. |
| ESC protokoly a konfigurace | `AP_BLHeli`, `AP_FETtecOneWire`, `AP_KDECAN`, `AP_PiccoloCAN`. |
| Spalovací motor a zdroje energie | `AP_EFI`, `AP_ICEngine`, `AP_Generator`. |
| Převážně letadla | `APM_Control`, `AP_L1_Control`, `AP_TECS`, `AP_Landing`, `AP_Soaring`. |
| Rover | `AR_Motors`, `AR_WPNav`; `AP_WheelEncoder` poskytuje měření otáčení kol a může být uveden i ve sdílených závislostech. |
| Vrtulníky | `AC_Autorotation` a příslušné implementace ve sdílených knihovnách. |
| Speciální senzory | `AP_Airspeed`, `AP_WindVane`, `AP_LeakDetector`, `AP_RSSI`. |
| Identifikace a pomocné služby | `AP_OpenDroneID`, `AP_RTC`, `AP_Stats`, `AP_Declination`, `AP_CustomRotations`. |
| Formáty a utility | `AP_JSON`, `AP_CSVReader`, `AP_GSOF`, `AP_OLC` (zeměpisné Plus Codes), `AP_MultiHeap`. |

Tohle není úplný katalog všech ovladačů. Zbylé malé knihovny bývají
pojmenované podle konkrétního zařízení, protokolu nebo pomocné služby.

## 5. Hardware: kde se kód potkává s deskou

| Složka | Účel |
| --- | --- |
| `libraries/AP_HAL/` | Obecná rozhraní: čas, GPIO, sériové porty, I2C, SPI, RC vstupy/výstupy. |
| `libraries/AP_HAL_ChibiOS/` | Implementace HAL pro ChibiOS/STM32, kterou používají obě naše desky. |
| `libraries/AP_HAL_ChibiOS/hwdef/` | Popisy podporovaných desek a nástroje pro generování hardwarové konfigurace. |
| `libraries/AP_HAL_SITL/` | HAL pro simulátor běžící na počítači. |
| `libraries/SITL/` | Modely simulovaných vozidel, senzorů a prostředí. |
| Ostatní `AP_HAL_*` | Alternativní platformy: Linux, ESP32, QURT a prázdná implementace Empty. |
| `libraries/AP_IOMCU/` | Komunikace s pomocným I/O procesorem u desek, které jej mají. |

Pro naše desky:

- [SkystarsH7HD-bdshot/hwdef.dat](libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD-bdshot/hwdef.dat)
  zahrnuje základní definici z `../SkystarsH7HD/hwdef.dat` a doplňuje změny varianty.
- [MicoAir743v2/hwdef.dat](libraries/AP_HAL_ChibiOS/hwdef/MicoAir743v2/hwdef.dat)
  definuje piny, sběrnice, senzory, výstupy a hardwarové výchozí hodnoty.
- `hwdef-bl.dat` slouží pro bootloader.
- `hwdef/scripts/` obsahuje generátory konfigurace.
- `AP_HAL_ChibiOS/RCOutput.cpp` je důležité místo pro PWM/DShot výstupy.
- `SPI*Device*`, `I2CDevice*`, `UARTDriver*` obsahují příslušné ovladače sběrnic.

Existence pinu v `hwdef.dat` sama neříká, jakou funkci má uživatel nastavenou
na konkrétním FC. Část chování určuje až konfigurace parametrů.

## 6. Tools: co je k čemu

| Podsložka `Tools/` | Účel |
| --- | --- |
| `ardupilotwaf/` | Logika sestavení ArduPilotu: výběr desky, knihovny a generování výstupů. |
| `bootloaders/` | Připravené bootloadery jednotlivých desek přidávané do `with_bl.hex`. |
| `AP_Bootloader/` | Zdrojový kód bootloaderu a seznam identifikátorů desek. |
| `scripts/` | Vývojové a servisní skripty; například `make_intel_hex.py`, `apj_tool.py`, `build_options.py`. |
| `environment_install/` | Instalace vývojového prostředí. |
| `autotest/` | Spouštění SITL, automatické scénáře a podpůrná data. Pro Copter zejména `arducopter.py` a `ArduCopter_Tests/`. |
| `autotest/param_metadata/` | Zpracování dokumentace parametrů ze zdrojů. |
| `autotest/logger_metadata/` | Zpracování dokumentace logovaných zpráv. |
| `Frame_params/` | Sady parametrů pro konkrétní rámy; nejsou univerzálním nastavením tvé kvadrokoptéry. |
| `Replay/` | Přehrávání zaznamenaných dat pro analýzu, zejména odhadu stavu. |
| `FilterTestTool/` | Nástroje pro práci s filtry. |
| `debug/` | Pomocné nástroje pro ladění. |
| `CodeStyle/`, `gittools/`, `completion/` | Formátování, práce s Gitem a doplňování příkazů. |
| `AP_Periph/` | Samostatný firmware pro periferie; není hlavní firmware FC. |
| `IO_Firmware/` | Firmware pomocných I/O procesorů. |
| `ros2/`, `simulink/`, `mavproxy_modules/` | Integrace s dalšími vývojovými systémy. |
| `geotag/`, `cameras_gimbals/`, `CHDK-Scripts/` | Fotografie, kamery a související nástroje. |
| `terrain-tools/`, `Vicon/`, `Pozyx/`, `UDP_Proxy/` | Terén, externí sledovací systémy a komunikační pomůcky. |

Testy pro odstraněná vozidla zůstávají v Tools/autotest jako součást
společného frameworku; jejich přítomnost je nezahrnuje do firmwaru Copter.

## 7. Modules: převzaté závislosti

| Podsložka `modules/` | Účel |
| --- | --- |
| `ChibiOS` | Operační systém a nízkoúrovňová podpora mikrokontroléru. |
| `waf` | Samotný sestavovací systém Waf. |
| `mavlink` | Definice a nástroje protokolu MAVLink; obsahuje také pymavlink. |
| `DroneCAN` | Knihovny a nástroje pro DroneCAN. |
| `gtest`, `gbenchmark` | Unit testy C++ a benchmarky. |
| `CrashDebug` | Podpora analýzy pádů firmwaru. |
| `Micro-CDR`, `Micro-XRCE-DDS-Client` | Serializace a klient pro DDS. |
| `lwip` | Síťový TCP/IP stack. |
| `littlefs` | Souborový systém pro vestavěná zařízení. |
| `gsoap` | Závislost pro SOAP služby, například při ONVIF integraci. |

Verze těchto závislostí určuje hlavní repozitář pomocí Git submodulů.
Aktualizace na libovolnou novější verzi není běžnou součástí kompilace.

## 8. Jak spolu části souvisejí

Zjednodušený tok řízení pro pochopení vztahů:

```text
RC přijímač / MAVLink / mise
            |
            v
ArduCopter: aktuální letový režim
            |
            v
Požadovaná poloha / rychlost / orientace / rotační rychlost
            |
            v
AC_WPNav + AC_PosControl (podle režimu)
            |
            v
AC_AttitudeControl + AC_PID
            |
            v
AP_Motors: mixování požadavků pro jednotlivé motory
            |
            v
HAL / RCOutput: PWM nebo DShot
            |
            v
ESC -> motory

IMU + další senzory -> filtrace / AHRS / EKF
                         |
                         +---- zpětná vazba pro řízení a navigaci
```

Nejde o přesné pořadí volání v jedné smyčce.
Například Acro nepotřebuje stejnou polohovou vrstvu jako Loiter.
Rozvrh úloh najdeš v `Copter.cpp`; rychlá rate smyčka může podle
konfigurace běžet v samostatném vlákně z `rate_thread.cpp`.

## 9. Chci řešit konkrétní věc: kde začít

| Co hledáš | První místo |
| --- | --- |
| Chování kvadrokoptéry v určitém režimu | `ArduCopter/mode_<režim>.cpp` |
| Rate PID a odezvu na gyro | `AC_AttitudeControl_Multi.cpp`, `AC_PID/`, `rate_thread.cpp` |
| Držení výšky nebo polohy | `AC_PosControl.cpp` a příslušný režim |
| Rozdělení výkonu na čtyři motory | `AP_MotorsMatrix.cpp` |
| DShot a časování výstupů | `AP_HAL_ChibiOS/RCOutput.cpp` |
| Gyro, jeho ovladač nebo filtry | `AP_InertialSensor/`, `Filter/`, `AP_GyroFFT/` |
| Odhad polohy/orientace a EKF | `AP_AHRS/`, `AP_NavEKF3/` |
| Piny UART, SPI, I2C nebo motorů | `hwdef.dat` konkrétní desky |
| Proč nejde armovat | `AP_Arming_Copter.cpp`, `libraries/AP_Arming/` a skutečná pre-arm hláška |
| Failsafe po ztrátě RC nebo baterie | `ArduCopter/failsafe.cpp`, `events.cpp` a příslušný vstupní modul |
| Kde je definován parametr | `Parameters.cpp` nebo `var_info` příslušné knihovny |
| Komunikaci s Mission Plannerem | `GCS_MAVLink_Copter.cpp`, `libraries/GCS_MAVLink/` |
| Obsah palubního logu | `ArduCopter/Log.cpp`, `AP_Logger/`, `LogStructure.h` příslušného modulu |
| Vznik APJ/HEX | `Tools/ardupilotwaf/chibios.py`, `Tools/scripts/make_intel_hex.py` |
| Hotový firmware | `build/<deska>/bin/` |

## 10. Jak číst jednu knihovnu

Typická struktura modulu:

- `AP_Nazev.h` — rozhraní třídy: co modul umí a jaká data poskytuje.
- `AP_Nazev.cpp` — společná implementace a často tabulka parametrů `var_info`.
- `*_Backend.*` — základ pro konkrétní ovladače.
- Soubory s názvy čipů/protokolů — jednotlivé implementace ovladačů.
- `*_config.h` — podmínky zapnutí funkcí při kompilaci.
- `tests/` — unit testy, pokud je knihovna má.
- `examples/` — malé ukázkové programy.
- `LogStructure.h` — definice logovaných zpráv, pokud je knihovna používá.

Ne každá knihovna má všechny tyto soubory.
Rozlišuj **nastavení při kompilaci** (`#if`, `*_ENABLED`, `hwdef.dat`)
a **parametry uložené ve FC** (`AP_Param`, například `BATT_*`).
Změna parametru obvykle nevyžaduje nový HEX; změna C++ kódu ano.

