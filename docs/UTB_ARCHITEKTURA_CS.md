# UTB: návrh paralelní řídicí vrstvy

## Implementovaný stav FÁZE 0 — 7.–8. 10. 2026

Tato část popisuje skutečný skeleton patch. Architektonický návrh níže
zůstává plánem dalších kroků; aktivní řízení z něj není implementované.
Implementace vychází z commitu `36874030af`.

### Implementováno

- `libraries/AP_UTB/`: manager, jediný parametr `UTB_ENABLE`, čtecí AP state
  provider a neaktivní rozhraní rate controlleru a mixeru.
- `AP_UTB_ENABLED` je výchozím stavem 0; Waf nabízí `--enable-UTB`
  a `--disable-UTB`. Copter integrace je omezena na multicopter frame.
- `UTB_ENABLE=0` je runtime default. Hodnota se zachytí při inicializaci;
  změna vyžaduje restart. Při vypnutí manager neodebírá stav ani neřídí.
- Parametr je ve skupině `ParametersG2::var_info2`, nový index 21,
  podskupina `UTB_`, lokální položka `ENABLE` s indexem 1.
  Existující parametrové indexy se nemění.
- `UTB_ACRO=40` je lokální číslo nového disarmovaného diagnostického režimu,
  název v AVAILABLE_MODES `UTB Acro (disarmed)`, krátký název `UTB0`.
  Vstup vyžaduje povolení při bootu, DISARMED a spool `SHUT_DOWN`.
- Běžné armování odmítá `allows_arming()`. Vynucené armování má navíc
  guard v `AP_Arming_Copter::mandatory_checks()`, výhradně pro UTB_ACRO.
  Tento malý zásah je nutný: force-arm obchází běžné `arm_checks()`.
  Původní režimy a jejich arming/failsafe algoritmy se nemění.
- `ModeUTBAcro::run()` pouze čte diagnostický snapshot. Nevolá regulator,
  mixer, motorové settery ani HAL. Původní motorový výstupní řetězec zůstává.
- Snapshot obsahuje pouze quaternion body FRD -> NED a gyro v rad/s,
  časy odečtu a aktualizace INS a validity flags. Kontrola odmítá NaN/Inf,
  nejednotkový quaternion a zastaralé snapshot/INS. Jde o diagnostiku,
  nikoli certifikaci čerstvosti všech podkladových senzorů či letovou health.
- `AP_UTB::healthy()` zůstává false: controller není implementovaný.
  Rate/mixer vždy vrací `valid=false`; neobsahují gains, PID ani alokaci tahu.
- Základní C++ testy validity a neaktivních výstupů a samostatný SITL
  regresní test, který spouští vlastní lokální simulátor v dočasném adresáři.

Není implementováno: aktivní motorový adaptér, PID, nové RC options/backend
resolver, shadow, bumpless transfer/fallback, UTB DataFlash zprávy,
`UTB_LOG_MASK`, attitude/altitude/position/AUTO ani vlastní estimator.
Změny DShot/HAL/AP_Motors nejsou součástí patche. Arming v UTB je zakázané;
nejde o firmware připravený k letu v UTB režimu.

### Přesný seznam souborů skeleton patche

| Soubor | Zásah a důvod |
| --- | --- |
| `libraries/AP_UTB/AP_UTB.h` | Nový manager, boot enable a přístup k diagnostickému stavu |
| `libraries/AP_UTB/AP_UTB.cpp` | AP_Param tabulka, inicializace a čtení snapshotu |
| `libraries/AP_UTB/AP_UTB_config.h` | Výchozí compile-time vypnutí |
| `libraries/AP_UTB/AP_UTB_State.h` | Jednotky/stav a abstrakce čtecího provideru |
| `libraries/AP_UTB/AP_UTB_State.cpp` | Čtení AP AHRS/INS a kontrola validity |
| `libraries/AP_UTB/AP_UTB_RateController.h` | Rozhraní neaktivního regulátoru |
| `libraries/AP_UTB/AP_UTB_RateController.cpp` | Vždy neplatný výsledek; bez PID |
| `libraries/AP_UTB/AP_UTB_MotorMixer.h` | Rozhraní neaktivního mixeru bez motorového API |
| `libraries/AP_UTB/AP_UTB_MotorMixer.cpp` | Vždy neplatný výsledek; bez alokace tahu |
| `libraries/AP_UTB/tests/test_utb.cpp` | Šest testů validity dat a neaktivní kostry |
| `libraries/AP_UTB/tests/wscript` | Registrace C++ testovacího cíle |
| `ArduCopter/mode_utb_acro.cpp` | Nový disarmovaný diagnostický režim |
| `ArduCopter/Copter.h` | Guardované členy manager/režim a přístup nové Mode třídy |
| `ArduCopter/system.cpp` | Inicializace manageru po inicializaci INS |
| `ArduCopter/Parameters.h` | Pointer na novou parametrovou skupinu |
| `ArduCopter/Parameters.cpp` | Registrace UTB_ a metadata lokálního módu 40 |
| `ArduCopter/mode.h` | ID 40, deklarace režimu a zákaz armování |
| `ArduCopter/mode.cpp` | Lookup a dostupnost režimu |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | Registrace v AVAILABLE_MODES |
| `ArduCopter/AP_Arming_Copter.cpp` | Povinný zákaz force-arm výhradně v UTB_ACRO |
| `ArduCopter/wscript` | Přidání AP_UTB do Copter knihoven |
| `Tools/scripts/build_options.py` | Custom-build volba UTB s defaultem 0 |
| `Tools/autotest/test_utb_skeleton.py` | Lokální SITL/MAVLink/RC regresní test |
| `docs/UTB_ARCHITEKTURA_CS.md` | Implementovaný stav, přesný manifest a evidence ověření |
| `README_BUILD_WSL_CS.md` | Doplnění nástrojů a build/test postupu FÁZE 0 |

### Stav ověření tohoto patche

| Kategorie | Stav |
| --- | --- |
| Implementováno | FÁZE 0 dle seznamu výše |
| Build ověřeno | SkystarsH7HD-bdshot s UTB zapnutým i vypnutým prošel |
| C++ testy | Prošlo všech 6 GTest testů v SITL sestavení s UTB zapnutým |
| SITL ověřeno | Build i lokální regresní test prošly s AP_UTB zapnutým i vypnutým |
| Hardware ověřeno | Ne; nebylo nahráno do FC ani provedeno měření na desce |

Ověření dokončeno 8. 10. 2026. SITL test při compile-time zapnutí ověřil
boot s `UTB_ENABLE=0` i `1`, nutnost restartu po změně parametru, vstup
přes GCS/RC, odmítnutí běžného/force/RC armování a odmítnutí vstupu
z armovaného STABILIZE. Motorové servo hodnoty při vstupu do UTB zůstaly
na původních disarmovaných hodnotách. Při runtime vypnutí i compile-time
vypnutí lze zvolit STABILIZE, ACRO a ALT_HOLD. V sestavení s compile-time
vypnutím není parametr UTB_ENABLE ani ELF symbol AP_UTB/ModeUTBAcro.
Tento test není letovým scénářem ani úplnou sadou Copter autotestů.
Zachování všech vlastností původního firmwaru nelze těmito dílčími testy
prohlásit za úplně prokázané.

Evidence v lokálním ignorovaném adresáři `tmp/`:

- `utb-hw-on-config.log`, `utb-hw-on-build.log`,
  `utb-hw-off-config.log`, `utb-hw-off-build.log`: obě HW sestavení.
- `utb-firmware-verify.log`: APJ image ověřen proti HEX s bootloaderem
  a výplní do aplikační adresy 0x08020000, pro obě varianty.
- `utb-sitl-on-config.log`, `utb-sitl-on-build.log`,
  `utb-sitl-off-config.log`, `utb-sitl-off-build.log`: obě SITL sestavení.
- `utb-unit-tests.log`: všech šest GTestů prošlo.
- `utb-sitl-on-test.log`, `utb-sitl-off-test.log`: lokální regrese prošly;
  podrobné konzole simulátorů jsou v `utb-sitl-tests-on/` a `utb-sitl-tests-off/`.
- `utb-artifacts/hw-on/` a `utb-artifacts/hw-off/`: uložené APJ a HEX
  obou variant pro SkystarsH7HD-bdshot. Aktuální `build/SkystarsH7HD-bdshot/`
  obsahuje vypnutou variantu; aktuální `build/sitl/` rovněž vypnutou variantu.

Kontroly `git diff --check`, flake8 změněných Python souborů a astyle
nových C++ souborů prošly. HAL, DShot, AP_Motors ani submoduly se neměnily.

Příkazy se spouštějí z kořene repozitáře s aktivní `.venv` a ARM GCC v PATH:

```bash
./waf configure --board SkystarsH7HD-bdshot --disable-UTB
./waf copter -j4
./waf configure --board SkystarsH7HD-bdshot --enable-UTB
./waf copter -j4
./waf configure --board sitl --enable-UTB
./waf --targets tests/test_utb -j4
./build/sitl/tests/test_utb
./waf copter -j4
python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 1 --log-dir tmp/utb-sitl-tests-on
./waf configure --board sitl --disable-UTB
./waf copter -j4
python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 0 --log-dir tmp/utb-sitl-tests-off
```

SITL regresní skript vyžaduje `pymavlink`. Při této práci byly do `.venv`
doplněny `pymavlink`, `pytest`, `flake8` a jejich závislosti a do Ubuntu
`astyle` pro kontrolu stylu. `pytest` není pro přímé spuštění skriptu nutný.

## Původní schválený architektonický návrh (plán)


Stav této původní části: schválený architektonický návrh. Skutečný rozsah
implementace a ověření je uveden výše; další etapy zůstávají plánem. Zpracováno
7. 10. 2026 nad větví `ardupilot-4.7.1-utb`, commit `36874030af`.
Zdrojem zjištění je skutečný kód této kopie; nelze ji zaměňovat za jinou
verzi upstream ArduPilotu. Při původní architektonické analýze se firmware nepřekládal ani
netestoval v SITL či na desce. Níže uvedené nové názvy API jsou návrhy.

## Rozhodnutí doporučená k odsouhlasení

- Nové režimy `ModeUTBAcro`, později `ModeUTBAngle`, `ModeUTBAltHold`
  a `ModeUTBAuto`; původní režimy zůstanou zachované.
- Knihovna `libraries/AP_UTB/` s hlavní třídou `AP_UTB`, podsložkami
  a názvy souborů podle obvyklého AP patternu. Parametrový prefix `UTB_`.
- Jeden existující motorový objekt a jeden fyzický výstupní řetězec.
  Pro aktivní UTB později úzký adaptér odvozený z `AP_MotorsMatrix`.
- První patch bude kostra bez řízení motorů a bez možnosti armovat UTB.
  Skutečný rate PID a mixer až v následujícím samostatném kroku.
- První aktivní verze pouze quad X, bez rychlého rate vlákna
  (`FSTRATE_ENABLE=0`) a bez běžného přepínání AP/UTB za letu.
- Zákaz změny arming/failsafe algoritmů zachovat. Pokud se ukáže nutná
  malá adaptace jejich vstupů nebo klasifikace režimu, nejprve ji samostatně
  předložit. Bez vyřešení těchto vazeb nepovolit aktivní UTB na hardware.

## A. Současný control flow

Logická cesta:

```text
RC přijímač / AP RC infrastruktura
  -> RC_Channels + RCMAP + kalibrace/deadzone
  -> Copter::read_radio(), rc_loop()
  -> vybraný Mode::run()
  -> reference attitude / rate / throttle
  -> AC_AttitudeControl_Multi
  -> AP_Motors::set_roll/pitch/yaw (+ feedforward), set_throttle
  -> AP_MotorsMulticopter::output()
  -> AP_MotorsMatrix::output_armed_stabilizing() [mixer]
  -> AP_MotorsMatrix::output_to_motors() [spool, linearizace, slew]
  -> AP_Motors::rc_write() -> SRV_Channels
  -> ChibiOS RCOutput -> DShot/DMA -> ESC
```

Není to celá posloupnost uvnitř jednoho cyklu. V
[Copter.cpp](../ArduCopter/Copter.cpp), `scheduler_tasks`, je pořadí:

```text
INS.update
-> run_rate_controller_main
-> volitelný run_custom_controller
-> motors_output_main
-> read_AHRS
-> read_inertia
-> check_ekf_reset
-> update_flight_mode
-> land/crash detektory
```

Rate smyčka tedy používá čerstvé gyro a reference připravené dřívějším
během režimu. `rc_loop` je plánovaný na 250 Hz a auxiliary vstupy na 10 Hz.
Periodu regulace přebírat ze scheduleru; nevkládat pevné `dt=0.0025`.

Druhá cesta je v [rate_thread.cpp](../ArduCopter/rate_thread.cpp): samostatné
vlákno čte gyro vzorky, volá `rate_controller_run_dt()` a `motors_output()`.
`FSTRATE_ENABLE` lze v existujícím kódu měnit za běhu. Pouhý nový callback
v hlavní smyčce by tuto cestu nepokryl a mohl by soutěžit o motorové výstupy.

STABILIZE vytváří reference náklonu a yaw rate, ACRO tělové úhlové rychlosti.
Původní ACRO může používat attitude stabilizaci/trainer; není vždy pouze
prostý rate PID. ALT_HOLD navíc používá vertikální kaskádu `AC_PosControl`;
AUTO dodává navigační reference přes vlastní stavové automaty a `AC_WPNav`.

## B. Konkrétní soubory a třídy

| Oblast | Zdroj a integrační význam |
| --- | --- |
| RC data a failsafe | [radio.cpp](../ArduCopter/radio.cpp), `Copter::read_radio()`; [RC_Channel](../libraries/RC_Channel/RC_Channel.cpp), kalibrace a debounce |
| RC volba režimu | [RC_Channel_Copter.cpp](../ArduCopter/RC_Channel_Copter.cpp), `mode_switch_changed()`, `do_aux_function()` |
| Definice režimu | [mode.h](../ArduCopter/mode.h), `Mode::Number`, virtuální vlastnosti režimu |
| Přechod režimu | [mode.cpp](../ArduCopter/mode.cpp), `mode_from_mode_num()`, `set_mode()`, `exit_mode()`, `update_flight_mode()` |
| Původní manuální módy | [mode_acro.cpp](../ArduCopter/mode_acro.cpp), [mode_stabilize.cpp](../ArduCopter/mode_stabilize.cpp), [mode_althold.cpp](../ArduCopter/mode_althold.cpp) |
| Attitude a rate | [AC_AttitudeControl.cpp](../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp), quaternionová stabilizace; [AC_AttitudeControl_Multi.cpp](../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp), rate PID a zápis motorových vstupů |
| Pozice a navigace | [AC_PosControl.cpp](../libraries/AC_AttitudeControl/AC_PosControl.cpp), [AC_WPNav.cpp](../libraries/AC_WPNav/AC_WPNav.cpp), [mode_auto.cpp](../ArduCopter/mode_auto.cpp) |
| Konstrukce objektů | [system.cpp](../ArduCopter/system.cpp), quad používá `AP_MotorsMatrix`, attitude `AC_AttitudeControl_Multi` |
| Motorový supervisor | [motors.cpp](../ArduCopter/motors.cpp), `motors_output()`: arming delay, interlock, emergency stop, motor test, cork/push |
| Spool a společný řetězec | [AP_MotorsMulticopter.cpp](../libraries/AP_Motors/AP_MotorsMulticopter.cpp), `output()`, `output_logic()`, proudové omezení |
| Mixer a konverze motorů | [AP_MotorsMatrix.cpp](../libraries/AP_Motors/AP_MotorsMatrix.cpp), `output_armed_stabilizing()`, `output_to_motors()` |
| Mapování výstupů | [AP_Motors_Class.cpp](../libraries/AP_Motors/AP_Motors_Class.cpp), `rc_write()`; [SRV_Channel_aux.cpp](../libraries/SRV_Channel/SRV_Channel_aux.cpp), `output_ch()` |
| DShot | [RCOutput.cpp](../libraries/AP_HAL_ChibiOS/RCOutput.cpp), `write()`, `push()`, `dshot_send_groups()`, `dshot_send()`, tvorba paketu a DMA |
| State interface | [AP_AHRS.h](../libraries/AP_AHRS/AP_AHRS.h), [AP_AHRS_View.h](../libraries/AP_AHRS/AP_AHRS_View.h), [AP_NavEKF3](../libraries/AP_NavEKF3/AP_NavEKF3.h) |
| Parametry | [Parameters.cpp](../ArduCopter/Parameters.cpp), [Parameters.h](../ArduCopter/Parameters.h), [AP_Param](../libraries/AP_Param/AP_Param.h) |
| DataFlash | [Log.cpp](../ArduCopter/Log.cpp), [AP_Logger.h](../libraries/AP_Logger/AP_Logger.h) |
| Příklad custom regulace | [AC_CustomControl.cpp](../libraries/AC_CustomControl/AC_CustomControl.cpp), přepis R/P/Y před původním mixerem |

`AC_CustomControl` resetuje integrátory původního regulátoru a zapisuje
motorové požadavky. Není to pasivní shadow výpočet ani hotová architektura
pro vlastní navigaci a mixer. Proto jej nepoužívat jako druhého současného
vlastníka výstupů. S aktivním UTB bude jeho aktivace nepřípustná.

## C. Navrhovaný UTB flow a rozhraní

```text
AP RC -> ModeUTB* -> UTB reference
                     |
AP AHRS/INS -> AP_UTB_StateProvider -> AP_UTB controllers
                     |                    |
                     |                R/P/Y/thrust
                     |                    v
                     |             AP_UTB_MotorMixer
                     |                    |
                     +------> AP_UTB_MotorsMatrix adaptér
                                          |
                        společný AP motorový řetězec
                                          |
                                   SRV/HAL/DShot
```

Copter vlastní manager a rozhoduje o režimu. Manager vlastní stav UTB,
regulátory, health a logování; řadič režimů/RC zůstává na straně Copteru.
Knihovna nesmí zahrnovat `Copter.h` ani přímo volat globální `copter`.
Předávat struktury `PilotInput`, `State`, `ControlCommand`, `MixResult`
a kontext s časem a stavem motorů. To jsou navržené typy, ne existující API.

`run()` režimu připravuje reference; rate PID běží v bodě
`run_rate_controller_main()` před motorovým výstupem. Aktivní režim určuje,
zda se v daném cyklu spustí původní rate regulátor, nebo UTB. Nesmí se
spoléhat na přepis po AP regulátoru bez kontroly jeho vedlejších účinků.
Při přechodu zajistit čerstvou referenci před prvním UTB rate krokem.

### State provider

- Interně SI: rad, rad/s, m, m/s; body FRD a world NED.
- Quaternion body->NED a gyro ze stejného AHRS view jako AP regulátor:
  `get_quat_body_to_ned()` a `get_gyro_latest()`.
- `AP_AHRS::get_velocity_NED()` a `get_relative_position_NED_origin_float()`
  pro rychlost a polohu vůči EKF origin; kontrolovat návratové `bool`.
- Výška kladná nahoru jen jako odvozené `-position_ned.z`; nemíchat home,
  origin, AMSL a terrain. Totéž pro stoupání `-velocity_ned.z`.
- Každé pole má validitu a odpovídající informaci o stáří. Čas načtení
  snapshotu sám nedokazuje čerstvost měření; ta se ověří z INS/AHRS statusu
  a aktualizačního cyklu. Rate ACRO nesmí vyžadovat GPS position fix.
- AHRS může vybrat jiný estimator; označení „EKF3“ platí až po ověření
  aktivního zdroje. Provider čte AHRS, nikoli vnitřní matice EKF3.
- Změny origin/yaw/velocity při EKF resetu ošetřit ve stejné časové doméně
  jako reference. Dnes to pro AP řeší `check_ekf_reset()` a `AC_PosControl`.
- Budoucí provider pro UTB estimator má stejnou smlouvu; volba pouze
  disarmed, zpočátku ideálně s rebootem. První verze žádný nový EKF nemá.

## D. Integrační bod vlastního mixeru

Doporučuji odvozenou třídu `AP_UTB_MotorsMatrix : public AP_MotorsMatrix`
v knihovně AP_UTB. Copter při startu vytvoří jeden tento objekt pro podporovaný
quad, pokud je funkce sestavená a povolená. V AP větvi jeho override volá
původní `AP_MotorsMatrix::output_armed_stabilizing()`; v UTB větvi adaptér
zavolá vlastní čistý mixer a vyplní chráněné `_thrust_rpyt_out[]`.
Nevytvářet druhý `AP_MotorsMatrix`: má singleton a sdílený hardwarový stav.
Za letu nepřehazovat ukazatel `motors` mezi objekty.

Toto je vhodné existující virtuální rozhraní, nikoli důkaz hotové bezpečnosti.
Společná cesta musí vždy projít:

```text
AP_MotorsMulticopter::output()
  update_throttle_filter()
  thr_lin.update_lift_max_from_batt_voltage()
  output_logic()                       [spool/current-limit]
  output_armed_stabilizing()           [jediný přepínaný bod]
  thrust_compensation()
  AP_MotorsMatrix::output_to_motors()   [spool/linearizace/slew]
  zbývající původní výstupní kroky
```

Důležité: `output_logic()` připravuje `_throttle_thrust_max`, ale původní
mixer tento limit skutečně aplikuje. Pouhé vyplnění čtyř hodnot 0..1 by
mohlo obejít část omezení. Adaptér musí číst aktuální limit po spool kroku,
předat kompenzaci a throttle budget mixeru a ověřit jeho výsledek.
Účinek limitu na kolektivní tah a dostupnou autoritu os musí být explicitní
a testovaný; nelze jej nahrazovat tvrzením „clamp 0..1 znamená bezpečno“.

Rozhraní mixeru musí vracet také dosažené požadavky a saturation flags
pro anti-windup. Adaptér udržuje `limit.roll/pitch/yaw/throttle_*`,
`_throttle_out` (používá jej mj. notch a detekce letu), konzistenci
feedforward vstupů, throttle filter/mix a příslušné diagnostiky motorů.
Ve vypnutém/idle stavu zachová omezení nastavená spool logikou.
Proudové omezení, voltage compensation a linearizaci nepoužít dvakrát.

Nedotčené zůstanou implementace původního mixeru, `output_logic()`,
`output_to_motors()`, `output_to_pwm()`, mapování `SERVOn_FUNCTION`,
interlock, emergency stop, motor test, PWM/DShot, DMA a ESC telemetry.
UTB nebude volat přímo `hal.rcout->write()`, `rc_write()` ani
`output_to_motors()` mimo společný řetězec. Ani callback pro thrust
compensation není vhodný skrytý způsob výměny celého mixeru.

### Quad X a pořadí motorů

`SkystarsH7HD-bdshot/hwdef.dat` nastavuje `HAL_FRAME_TYPE_DEFAULT 12`
(Betaflight X). `AP_MotorsMatrix::setup_quad_matrix()` rozlišuje běžné X,
BF_X a další permutace i reverzované směry. UTB musí podporovat explicitně
ověřenou geometrii a mapování; neztotožňovat `motor[0]` s fyzickým pinem 1.
Před armováním ověřit frame class/type a čtyři povolené logické motory.
První aktivní etapa může podporovat X a BF_X jako dvě explicitní permutace
jedné geometrie. Ostatní varianty odmítnout, nikoli tiše přemapovat.

Čistý mixer nemá HAL, arming ani vlastní parametrové úložiště. Pokud převezme
část AP matematiky, uvést původ a zachovat GPL licenci. Vyřezání alokace
neznamená vypuštění limitů potřebných společným výstupem.

## E. AP/UTB backend switch

Autoritativní je úspěšně přijatý `flightmode`, nikoli poloha přepínače.
RC žádost, připravenost UTB a aktivní backend jsou tři odlišné údaje.
Po selhání `set_mode()` se nesmí přepnout mixer ani rate regulátor.
Přechod všech částí se provede atomicky v řídicí smyčce, bez dynamické alokace.

`set_mode()` volá `new_mode->init()` ještě před `old_mode->exit()` a změnou
`flightmode`. Proto `init()` může validovat/připravit, ale nesmí předčasně
přepnout globální výstupní backend. Výběr pro cyklus odvodit až z potvrzeného
režimu. Opuštění UTB kvůli AP LAND/RTL/failsafe musí odpojit UTB mixer i PID.

Zpočátku povolit změnu AP/UTB pouze disarmed a v `SHUT_DOWN`. Výjimkou je
budoucí samostatně ověřený nouzový návrat do AP. Zákaz běžného přepínání
nesmí blokovat `set_mode()` vyvolaný ArduPilot failsafe.

## F. RC přepínače

ARM ponechat existujícímu `RCn_OPTION=153` (`ARMDISARM`) nebo běžnému AP
arming postupu. Žádná pevná čísla kanálů.

Pro manuální třípolohový přepínač použít `FLTMODE_CH` a stávající
`FLTMODE1..6`. Příklad pro vysílač s polohami 1000/1500/2000 us:
`FLTMODE1/2=STABILIZE`, `FLTMODE3/4=ALT_HOLD`, `FLTMODE5/6=ACRO`.
Skutečné PWM a vybrané sloty ověřit; vstup už zpracovává AP debounce.

Dvě nové pojmenované AUX funkce, pracovně `UTB_BACKEND` a `UTB_AUTO`,
použijí existující `RCn_OPTION` mechanismus. Číselná ID přidělit po kontrole
celého enumu a metadat; nerecyklovat `CUSTOM_CONTROLLER=109` ani jiné funkce.
RC knihovna bude znát pouze enum/metadata, nikoli záviset na AP_UTB.

| Backend | AUTO | Manuální volba | Požadovaný režim |
| --- | --- | --- | --- |
| AP | OFF | ANGLE / ALT_HOLD / ACRO | STABILIZE / ALT_HOLD / ACRO |
| UTB | OFF | ANGLE / ALT_HOLD / ACRO | UTB_ANGLE / UTB_ALT_HOLD / UTB_ACRO |
| AP | ON | libovolná | AUTO |
| UTB | ON | libovolná | UTB_AUTO |

Copter resolver nejprve získá konzistentní snapshot všech debouncovaných
poloh, potom vyhodnotí prioritu AUTO a jednou zavolá `set_mode()`.
MIDDLE u dvoupolohových AUX voleb znamená nepřijmout novou žádost,
nikoli automaticky aktivovat UTB/AUTO. Při startu neaktivovat autonomii
jen kvůli vysoké poloze; vyžadovat platný vstup a vědomý přechod OFF->ON.

Resolver je řízený událostmi, ne trvalým vynucováním polohy RC každou smyčku.
Failsafe, LAND/RTL, GCS změna či jiná vyšší autorita nesmí být ihned přepsána
zpět. Po takovém zásahu požadovat nový vědomý RC povel; po obnově signálu
neprovádět automatický návrat. Detekovat konflikt s dalšími mode AUX volbami.

V první etapě bude UTB_ACRO jediná dostupná UTB volba. Ostatní se odmítnou
s hláškou a zachováním aktuálního režimu; nesmějí být prázdnými letovými
placeholdery. `UTB_ENABLE=0` zachová původní chování RC.

## G. Bumpless transfer a shadow

Nulování integrátoru samo o sobě nezaručuje přechod bez skoku. Zpočátku
proto nepovolit běžné přepínání backendu za letu.

Pozdější přechod musí inicializovat cílovou attitude, rate, výšku a pozici
z platného stavu, filtry z aktuálního měření a throttle z dosaženého tahu.
Preload I-termu vychází z posledního dosaženého zásahu po omezení, s limity
integrátoru. Ověřit také FF, D filtr a historii `dt`; AP a UTB gains nejsou
zaměnitelné. Případný blend provádět ve stejné doméně normalizovaného tahu,
před jedinou společnou spool/output cestou, ne blendem DShot paketů.
Testovat zatížení, saturaci a požadavek failsafe během přechodu.

Shadow v prvním směru: AP řídí; UTB má vlastní integrátory, filtry a čistý
mixer a pouze loguje. Žádné zápisy do `motors`, spool state nebo AP PID.
Logy musí sdílet timestamp/reference a uvádět validitu a doménu výstupu.

Opačný směr není zadarmo: AP PID zapisuje do motorů, AP mixer aktualizuje
limity, throttle a detekci ztráty motoru; `AC_CustomControl` také mění
integrátory. Druhé volání `motors->output()` je nepřípustné. Při UTB aktivním
zpočátku AP reference označit jako nedostupnou, nebo logovat jasně označený
pasivní model. Neoznačovat staré hodnoty AP PID za aktuální shadow výsledek.

## H. Safety, fallback, parametry a logování

### Safety a otevřené podmínky aktivního řízení

`healthy()` má vracet také důvod a validitu podle režimu: gyro/attitude,
navíc výška/vertikální rychlost pro ALT_HOLD a poloha/rychlost pro AUTO.
Kontrolovat finite hodnoty, interval `dt`, stáří, meze reference, výstupy
PID, všech motorů, geometrii a deadline výpočtu. Ověřovat před zápisem
motorových vstupů i před potvrzením výsledku mixeru.

| Situace | Požadované chování |
| --- | --- |
| Nepřipravené UTB před vstupem | Odmítnout vstup, nechat dosavadní AP režim |
| Nehotový skeleton | Nelze armovat; nelze do něj vstoupit armed |
| DISARM/interlock/e-stop | Rozhoduje původní AP motorový řetězec |
| RC/battery/EKF failsafe | Původní AP rozhodnutí má prioritu; RC resolver se nepokouší vrátit UTB |
| Chyba pouze UTB, AP state zdravý | Navržený řízený návrat do vhodného AP režimu přes `set_mode()`, s potvrzením a inicializací regulace |
| Neplatný společný estimator | Nelze předpokládat, že AP ALT_HOLD/LAND/RTL bude funkční; použít existující EKF/failsafe politiku podle dostupného stavu |
| Žádný platný výstup nebo neúspěšný fallback | Zablokovat aktivní nasazení, dokud není navržen a fault-injection testy ověřen postup pro tento případ |

Nevkládat NaN do výstupu; dlouhodobě nedržet poslední command a neslibovat,
že automatický DISARM za letu je obecně bezpečný fallback. Pro aktivní fázi
musí být definovaná reakce v rámci konkrétního cyklu, včetně selhání mixeru
po výpočtu spool limitů. Samotná žádost `set_mode()` tuto mezeru neřeší.
První nearmovatelný patch tento nevyřešený nouzový postup nepotřebuje.

Konkrétní provázanosti vyžadující lidské posouzení před aktivní fází:

- `land_detector.cpp` čte AP attitude target/error a throttle mix;
  `crash_check.cpp` čte i AP throttle a yaw integrátor. Zmrazené AP hodnoty
  po převzetí UTB by byly nesprávné. Nutný adaptér diagnostických vstupů
  se zachovanou sémantikou; neobcházet kontrolu nulováním chyby.
- Původní `ModeAcro` má `crash_check_enabled() == false`. Požadavek zachovat
  crash ochranu v UTB_ACRO proto nelze vyřešit slepým zkopírováním ACRO;
  vyjasnit chování pro akrobatické rate reference.
- `should_disarm_on_failsafe()` v `events.cpp` výslovně rozpoznává čísla
  STABILIZE a ACRO. Nové UTB módy spadnou do jiné větve. Pokud se nesmí
  upravit ani klasifikace, nelze tvrdit stejnou failsafe sémantiku.
- `init(ignore_checks)` se volá i disarmed s vypnutými některými kontrolami.
  UTB hardwarová omezení a nearmovatelnost nesmí tímto příznakem zmizet.
- `FSTRATE_ENABLE=0` kontrolovat před vstupem a zabránit aktivaci rychlého
  vlákna v aktivním UTB; samotný jednorázový test parametru nestačí.
  Konkrétní malý guard ve startu vlákna předložit s aktivním patchem.

### Parametry

Pouze AP_Param. Manager má `var_info`; controller a mixer mají podskupiny,
bez vlastní obecné parameter-storage vrstvy. Registrace v `ParametersG2`
přes `AP_SUBGROUPINFO` nebo existující pointer pattern. Aktuální `var_info2`
končí položkou 20, takže index 21 je kandidát pro novou skupinu, po kontrole
historie a souběžných změn; index 62 je rezervovaný pro extension.
Žádná renumerace existujících parametrů.

První patch: `UTB_ENABLE=0` (enable flag, reboot), `UTB_LOG_MASK` a případně
`UTB_SHADOW=0` až když shadow skutečně existuje. PID podskupiny registrují
jen implementované položky; nepřidávat nefunkční budoucí parametry.
Příklady `UTB_RAT_RLL_P/I/D`, `UTB_RAT_PIT_P/I/D`, `UTB_RAT_YAW_P/I/D`
a `UTB_RAT_RLL_IMAX` se vejdou do limitu 16 znaků. Doplnit anotace rozsahů,
jednotek, defaultů a rebootu podle skutečné implementace. Nelze převzít
AP gains bez ověření jednotek, škálování momentu a filtrace.

### DataFlash

Použít `AP::logger().Write()` podle vzoru `AC_CustomControl::log_switch()`
a dokumentaci `@LoggerMessage`. Před přidáním ověřit kolize názvů. Návrh:

| Zpráva | Obsah |
| --- | --- |
| UTBS | TimeUS, požadovaný/aktivní backend, mode, state source, health/reason, sequence |
| UTBR | TimeUS, axis, desired/measured rad/s, error, P/I/D/FF, output, saturation |
| UTBM | TimeUS, backend, R/P/Y/T, čtyři normalizované tahy, limits, valid |
| UTBA | Později attitude desired/measured a rate target |
| UTBP | Později position/velocity desired/measured |

Oddělit pre-linearization tah od skutečného actuator/PWM ekvivalentu.
Párové UTBM záznamy s backend tagem umožní porovnat `M1_AP..M4_AP`
a `M1_UTB..M4_UTB`. RCOU zůstává záznamem skutečných AP výstupů, není to
měření tahu ani samostatný výstup neaktivního AP backendu. Eventy logovat
při změně, průběžná data decimovat a měřit cenu logování. Bez blokování,
textových hlášek či alokací v rate smyčce; bez loggeru se build musí sestavit.

## I. Soubory prvního patche a plán etap

První patch má ověřit hranice, build a vstupy. Neobsahuje létající vlastní
regulátor. `ModeUTBAcro::allows_arming()` vrátí false a `init()` odmítne vstup
armed, včetně GCS cesty. Skeleton nesmí vypadat jako funkční UTB_ACRO.
Rate/mixer kostry vracejí stav „neimplementováno“, nikoli použitelný tah.
Při `UTB_ENABLE=0` se zachová původní chování.

| Přidat | Důvod |
| --- | --- |
| `libraries/AP_UTB/AP_UTB.h/.cpp` | Manager, health/lifecycle, základní var_info |
| `libraries/AP_UTB/AP_UTB_config.h` | `AP_UTB_ENABLED`, výchozí vypnutí a podmínky sestavení |
| `libraries/AP_UTB/AP_UTB_State.h/.cpp` | Typy snapshotu a AP state provider; žádný nový EKF |
| `libraries/AP_UTB/AP_UTB_RateController.h/.cpp` | Rozhraní a reset neaktivní kostry |
| `libraries/AP_UTB/AP_UTB_MotorMixer.h/.cpp` | Čisté typy command/result a neaktivní kostra |
| `ArduCopter/mode_utb_acro.cpp` | Neaktivní režim, kontrola vstupu a armingu |
| `libraries/AP_UTB/tests/` | Smysluplné testy jednotek, validity a stavových přechodů |
| `docs/UTB_ARCHITEKTURA_CS.md` | Tento návrh a pozdější aktualizace ověření |

| Změnit | Důvod a omezení |
| --- | --- |
| `ArduCopter/wscript` | Přidat knihovnu; ostatní vozidla ani submoduly neupravovat |
| `ArduCopter/Copter.h` | Guardované členy manager/režim |
| `ArduCopter/system.cpp` | Inicializace po dostupnosti AHRS/motorů; zatím bez změny motorového objektu |
| `ArduCopter/Parameters.h/.cpp` | Nová skupina a metadata volby režimu; žádné přesouvání indexů |
| `ArduCopter/mode.h/.cpp` | Deklarace režimu, unikátní ID, lookup, dostupnost |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | Seznam AVAILABLE_MODES, konzistentní s mode.cpp |
| `Tools/scripts/build_options.py` | Volba AP_UTB a potřebné dependencies, compile-out |

Číselná mode ID zarezervovat až po kontrole v celé větvi včetně scripting
registrací. Nepoužít rezervované 30 a 127. Režimy držet v rozsahu datových
typů `Mode::Number` a parametrů; seznam dostupných režimů má vlastní 32bit
masku indexů a `static_assert`, nikoli automatickou podporu libovolného počtu.
První patch přidává pouze UTB_ACRO, ostatní jména jsou návrh do dalších etap.

RC resolver a dvě AUX volby přidat jako následující malý patch s testy tabulky
přechodů: nový `ArduCopter/utb_control.cpp`, úzké změny
`RC_Channel_Copter.cpp`, `Copter.h`, `RC_Channel.h` a příslušných RC metadat.
Původní režimy se neupravují. V první kostře lze režim vybrat přes stávající
FLTMODE/GCS infrastrukturu bez nových RC funkcí.

Aktivní FÁZE 1 navíc přidá `AP_UTB_MotorsMatrix.h/.cpp`, konkrétní PID/mixer,
výběr v `system.cpp` a rate dispatch v `Attitude.cpp`, guard rychlého vlákna
a schválené řešení diagnostických/failsafe vazeb. Dokud není toto řešení
schválené, pokračovat jen čistými testy, shadow a simulací.

Pořadí: 0 kostra -> 1 ACRO/PID/quad-X mixer -> 2 ANGLE -> 3 ALT_HOLD
-> 4 position -> 5 AUTO/trajectory/waypoints -> 6 avoidance -> 7 estimator.
V každé etapě zvlášť validita state, limity, logy a testy. Nedoplňovat všechny
budoucí třídy jako prázdné soubory do prvního patche.

### Ověření požadované pro implementaci

- Build SkystarsH7HD-bdshot s UTB vypnutým i zapnutým; následně cílový
  SkystarsH7HD. SITL build Copteru a kontrola logging-off/optional dependencies.
- Stávající AP režimy bez změny; test, že skeleton nelze armovat ani zvolit
  armed přes RC/GCS a že nepíše motorové commandy.
- Při aktivní fázi unit testy os/znamének/permutací, saturace, thrust budget,
  NaN/Inf, anti-windup a časování; stavové testy spool/interlock/e-stop.
- SITL testy odmítnutého přechodu, RC/battery/EKF failsafe, konfliktních AUX,
  stale state, změny FSTRATE za běhu, shadow bez write side effects a fallback.
- Teprve po testech a lidské revizi bench test bez vrtulí; build sám není
  potvrzení bezpečného řízení ani důvod k prvnímu letu.

## J. Hlavní rizika a hranice návrhu

Největší rizika nejsou ve vytvoření nové třídy PID, ale v zachování sémantiky
okolního systému: spool/current-limit vs. alokace, AP diagnostika očekávající
AP controller, čerstvost vstupů, společné singletony a rychlé rate vlákno.

Nezávislost řídicích větví není totéž jako dvě nezávislé instance celého AP.
Jeden scheduler, motorový objekt, bezpečnostní stav a HAL výstup zůstanou
společné. UTB nesmí změnit core knihovnu na závislou na volitelném AP_UTB.

Dědičný adaptér je doporučený bod napojení ověřený existencí potřebných
virtuálních metod a chráněných dat. Jeho aktivní chování zatím ověřené není.
Návrh proto nezaručuje bumpless fallback, správnost nového mixeru ani
bezpečnost letu. Otevřené body v části H jsou podmínkou další aktivní etapy,
ne důvodem oslabit existující kontroly.


## K. Skutečný stav: SHADOW FÁZE 1 (8. 10. 2026)

Tato část doplňuje historický návrh A–J. Schválená a implementovaná FÁZE 1
je **SHADOW ONLY** dle verze 2 `UTB_CONTROL_THEORY_CS.md`. Dřívější formulace
„Aktivní FÁZE 1“ v části I je původní budoucí návrh, nikoli realizovaný stav
nebo povolení aktivního výstupu. Přepínání autority, vlastní motorový adaptér,
aktivní UTB_ACRO, další režimy a FÁZE 2 zůstávají návrhem.

**IMPLEMENTOVÁNO:** AP_UTB samostatný tříosý rate PID (default Ki=0),
SHADOW ONLY directional jednokrokový anti-windup, diagnostický BF_X mixer,
AP-derived UTB shadow reference, parametry SHADOW/LOG_RATE/gains a oddělená
validita state/controller/mixer versus logging/observability. Runtime přijímá
class=1/type=12 nebo18 ve větvi test/skystars-5inch. Neplatné motorové výsledky jsou nuly.
Mixer zachovává poměr R/P/Y společným scale a může posunout collective;
yaw nemá nižší prioritu. Allocator není schválen pro aktivní letový output.

**INTEGRACE:** read-only AP rate target capture před existujícím AP rate
controllerem, shadow fast task po AP motor outputu před AHRS a následným
update_flight_mode. Jeden AP controller run zůstává jeden run. TimeUS/Seq,
AP mode/capture timestamp/sequence/mode a RC timestamp umožňují offline
alignment; AP target a UTB reference nejsou automaticky jeden logický sample.
Čtyři snapshoty ve frontě, maximálně jedna sada na consumer průchod,
UTBS status přibližně 1 Hz, UTBR/M/A/T sada experimentálně 1–400 Hz
(default 200); log rate neřídí controller rate. Při logger výpadku za běhu
matematika pokračuje, drops/missing jsou observability. Bez dostupného loggeru
se nový experiment nespustí. FSTRATE nenulové nebo aktivní rate thread
potlačí shadow. Opakovaný časový overrun potlačí pouze shadow.

**OMEZENÍ AUTORITY:** UTB nemá žádné motorové setters/output/spool/limits,
HAL/DShot zápisy ani druhé spuštění AP controlleru. AP_Motors chování,
arming/failsafe a estimator zůstávají původní. UTB_ACRO je disarmovaná
diagnostika, normální/force/RC arm i armed entry jsou odmítnuty.
Při ENABLE=0 neběží UTB shadow/capture/log/status; compile-out je zachován.
Nové vlastní EKF, position/altitude/AUTO ani UTB rate/expo parametry nevznikly.

| Úroveň | Stav a rozsah |
| --- | --- |
| NÁVRH | Schválená matematika/architektura v2; aktivní výstup a další fáze pouze návrh |
| IMPLEMENTOVÁNO | Výše uvedená SHADOW FÁZE 1, lokální patch |
| UNIT OVĚŘENO | 26 C++ testů; podrobnosti a finální evidence v teorii, část 24 |
| BUILD OVĚŘENO | SkystarsH7HD-bdshot a SITL, UTB on/off; finální tabulka v teorii, část 24 |
| SITL OVĚŘENO | Shadow guards/logging/math, RC map, AP SYSID alignment, steady ground motor isolation a FÁZE 0 regrese |
| HARDWARE OVĚŘENO | NE; firmware nebyl nahrán do FC, frame ani H743 load nebyly změřeny |
| SKUTEČNÝ LET OVĚŘENO | NE; pouze AP řízený simulovaný SYSID let, nikoli uzavřená UTB regulační smyčka |

Přesný seznam souborů/zásahů, logové formáty a otevřené otázky jsou
v [UTB_CONTROL_THEORY_CS.md](UTB_CONTROL_THEORY_CS.md), části 19, 20 a 24.
Příkazy a informace o instalacích jsou v [README_BUILD_WSL_CS.md](../README_BUILD_WSL_CS.md).
Bez dalšího odsouhlasení nepokračovat další fází.


## L. Aktuální hardening transport FÁZE 1

Část K zachycuje první SHADOW patch; její FAST_TASK consumer byl při auditu
shledán blokující přes AP logger backend. Aktuální revize jej odstraňuje.
Flight main obsahuje jen read-only capture a shadow producer. Datová queue
je fixed external-storage AP ByteBuffer/ObjectBuffer SPSC, čtyři položky,
bez heapu/locku/wait. Status mailbox má pouze best-effort try-lock.
Consumer běží v samostatném HAL workeru PRIORITY_IO,0 (ChibiOS 58 < main180),
max čtyři sady/cyklus + sleep1ms. Všechny UTB backend write a GCS status
operace jsou v workeru; producer na loggeru nikdy nečeká. Consumer nesmí
číst main-owned snapshot/capture/controller; používá kopie a atomic feedback.
Full = diagnostics drop, ne zpomalení nebo reset controlleru. Off/on používá
epoch místo cross-thread clear. Při boot ENABLE=0 nevznikne worker ani
UTB capture/control/queue/log činnost. Primary gyro change resetuje vlastní
PID historii s PRIME/PRIMARY_GYRO_CHANGED; AP failover zůstává původní.

PID/BF_X/reference/anti-windup a všechny arming zákazy zůstávají zachovány.
Nový UTBQ ID18 eviduje pipeline counters/epoch/gyro; ID13–17 se nemění.
Skutečné test/build výsledky a omezení jsou v teorii, oddíl25. Hardware,
H743 CPU/logger load, stack reserve a skutečný let nejsou ověřené.
FÁZE 2 a aktivní motor authority nezačaly.


Hardening ověření: **31/31 UNIT PASS**, SITL shadow + primary gyro/SYSID/
RC/isolation PASS, FÁZE 0 regrese ON/OFF PASS, SkystarsH7HD-bdshot a SITL
build ON/OFF PASS, source audit producer/transport/motor/arming PASS.
Stav softwarové SHADOW FÁZE 1 = **COMPLETE**. Hardware a skutečný let
zůstávají **NEOVĚŘENO**; podrobná evidence/omezení v teorii 25.5–25.7.
## M. FÁZE 1H-A — diagnostický hardware benchmark

**Implementováno:** samostatný compile-time `AP_UTB_BENCH_ENABLED`, default 0,
nezávislý na AP_UTB_ENABLED. BENCH OFF odstraní diagnostické třídy, parametry,
log metadata, scheduler probe i exporter. BENCH ON funguje také při runtime
UTB_ENABLE=0; proto lze porovnat A0/BENCH OFF s A/BENCH ON.

Pro SkystarsH7HD-bdshot je relevantní SPI W25Q128 BLOCK logger,
LOG_BACKEND_TYPE=4. Typ 1 je filesystem, který zde není aktivním backendem.
Instrumentuje se jeho původní write_sem, bez změny acquire/release policy.
Filesystem probe je pouze SITL; flash-chip/SPI zámky nejsou měřeny.

Nové logy UBMT/UBHI/UBST/UBEP a omezená channel-0 telemetry pocházejí
ze samostatného diagnostického exporteru. MAIN/UTB_WORKER/OTHER/UNKNOWN
atribuce používá skutečnou identitu vlákna, nikoli prioritu. Snapshoty mají
jednorázové atomic čtení bez retry; nejednoznačná kolize je UNKNOWN.
Worker stack watermark používá místní ChibiOS stack_free API. Zachována
je původní priorita, lifecycle, queue a failure policy shadow workeru.

**Unit/SITL ověření:** konkrétní výsledky, buildy a source audit jsou
v [software reportu](UTB_BENCH_SOFTWARE_REPORT_CS.md).
**Hardware neověřeno:** mutex latence/PI, CPU a logger load, stack rezerva,
exporter overhead, udržitelnost 200/400 Hz a skutečný frame na FC.
Nulové potvrzené kolize neprokazují nulové blocking riziko; diagnostika sama
přidává overhead a exporter je další logger contender.

PID, D filtr, anti-windup, reference, BF_X mixer, arming, AP_Motors,
HAL/DShot, EKF a FSTRATE se nemění. UTB nemá motorovou autoritu.
Plán první session je [hardware bench postup](UTB_HARDWARE_BENCH_CS.md).
MicoAir743v2 je samostatný target s SDMMC filesystem loggerem; současný
BLOCK mutex probe pro jeho hardware neplatí. FÁZE 2 ani aktivní UTB_ACRO
nejsou zahájeny. Firmware se na hardware nenahrával.

## N. SHADOW BF_X_REV — test/skystars-5inch

IMPLEMENTOVÁNO: whitelist class1/type12 a18, jedna matematická implementace allocatoru s opačným yaw sloupcem a konzistentní inverzí pro18. R/P, motor numbering, common scale/collective shift a SHADOW ONLY anti-windup se nemění. Runtime změna geometrie resetuje PID historii a zvýší diagnostickou epoch; PRIMING výpočet nemá platný mixer. TIME_BUDGET/FSTRATE/state/gyro/logger ochrany zůstávají.

UTB nadále nemá fyzický motorový output. AP_Motors, HAL, DShot, původní PID, arming a EKF nejsou dotčeny. Změna existuje pouze v testovací větvi; vývojová větev se nemění. Skystars export class1/type18 a všech1190 hodnot jsou zachovány. C/D overlay jsou oddělené profily pro DISARMED bench, nikdy změna frame nebo globální default.

Unit/SITL/build ověření a jeho omezení viz [report](UTB_BFX_REV_REPORT_CS.md). Hardware timing/log throughput a fyzické zapojení NEOVĚŘENO. Aktivní UTB_ACRO a FÁZE2 nejsou součástí rozšíření.
