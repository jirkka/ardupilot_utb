# FÁZE 2F: pilotní UTB ANGLE pouze v SITL

Stav: implementováno; omezený pilotní kontrakt je unit a skutečně SITL ověřen. Hardware neověřeno; aktivní hardware cesta je compile-out.
Rodič: `9824542e73bbfb193b21fd4388f74e14ba24439c`, větev `test/skystars-5inch`.

Samostatný mode 42, oddělený od disarmovaného mode 40 a automatického mode 41.
Stejný `AP_UTB_RATE_SITL_AVAILABLE` vyžaduje UTB, explicitní active build option
**a skutečný HAL_BOARD_SITL**. H743 nemůže aktivovat ANGLE runtime parametrem.

## Reference a řízení

RCMAP kanály poskytuje původní `init_rc_in()` a `channel_roll/pitch/yaw/throttle`.
Použijí se skutečné `RC_Channel::norm_input_dz()` a AP range/throttle mapování.
Kontrolují se endpointy, trim/deadband, typ kanálu, přítomnost a stáří signálu.
Platnost a stáří používají `RC_Channels::has_valid_input()`, `last_input_ms()`
a původní `get_fs_timeout_ms()`, nikoli novou libovolnou konstantu.

Roll/pitch mají lineární stick-to-angle mapu s kruhovým omezením kombinované
reference; počáteční maximální norma 5°. Kladný roll = pravé křídlo dolů,
kladný pitch = příď nahoru, kladný body yaw = otočení doprava (FRD/NED).
AP SIMPLE/SUPERSIMPLE transformace ani ACRO expo se v této body stick mapě
nepoužívají. Yaw je samostatná body rate, nejvýše 0,1 rad/s; není heading hold.

Vlastní nezměněný attitude P (3/3 s^-1, RP rate limit 0,3 rad/s) předává
body target nezměněnému UTB rate PID. ANGLE používá explicitní konstantní
ověřený SITL preset R/P/Y 0.19/0.16/0.0015/0.19/15,
0.19/0.16/0.001/0.19/20, 0.30/0.60/0/0.19/20.
Dosavadní shared RAT a S_* defaulty ani automatické profily se nemění.

Throttle je skutečná RC páčka zpracovaná `Mode::get_pilot_desired_throttle()`.
Střed expo mapy používá původní `Mode::throttle_hover()` / AP hover parametr,
stejně jako STABILIZE. Výstup průběžně reaguje na RC a není konstantní.
ANGLE nepředefinuje význam polohy páčky novým entry hover mapováním.
Použije se `set_throttle_out(..., false, ...)`, tedy bez angle boost a bez
AP výškové smyčky. AP allocator/spool a motorové pořadí zůstávají původní.
Explicitní manual throttle/RPY mix má výchozí AP manual hodnotu 0,1;
jeho entry hodnota se při odchodu obnoví. Nejde o spuštění AP rate PID.

## Lifecycle, scheduling a ochrany

Pilotní snapshot vzniká v `run()` stejně jako běžné AP pilotní reference,
pro následující rate slot. Úhly i throttle proto patří ke stejnému RC vzorku.
Rate slot znovu kontroluje stáří a validity, vzorkuje AHRS/INS, vykoná právě
jednu vlastní attitude a jednu vlastní rate smyčku a předá R/P/Y původnímu
jedinému motor outputu. Post-output capture/log sdílí TimeUS/Seq a RC timestamp.
AP attitude/rate vlastníci se v této větvi nespouštějí.

Entry/exit resetují oba UTB controllery, reference, feedback, sekvence a AP
přechodovou historii. Arming zůstává standardní AP (RC/MAVLink/AUX), bez force
v testech. Mode klasifikace: manual throttle, non-autopilot, bez XY řízení,
žádný vstup při RC failsafe. Validní AHRS výška/vz jsou nutné jen pro zachování
experimentální obálky ±2 m od entry / |vz|≤0,5 m/s, nikoli pro řízení výšky.
Skutečné roll/pitch do35°, gyro norma do2 rad/s a dosavadní dt/IMU guards platí.

Výpadek RC, stale sample, chybná kalibrace, parametry, FSTRATE nebo controller
fault vyvolají latched fail-stop (RPY/throttle0, SHUT_DOWN) v ANGLE. Obnovení
signálu latch nemaže; je nutný výslovný odchod a nový validní vstup. Původní
AP failsafe může standardně vyžádat LAND/RTL/disarm; takový skutečný mode
transition je oddělený od zakázaného tichého přepnutí PID uvnitř ANGLE.
Default anti-windup politika zůstává saturation stop; volitelná diagnosticky
ověřená alokační zpětná vazba má vlastní ANGLE opt-in.

## Porovnání

Funkční cíl odpovídá ANGLE principu, nikoli kopii Betaflight matematiky.
Oficiální [BF Modes](https://betaflight.com/docs/wiki/guides/current/Modes)
a [BF Controls](https://betaflight.com/docs/wiki/guides/current/Controls)
vymezují ovládání a samonivelaci. Přesné rozdíly shapingu, throttle a saturace
popisuje tabulka níže a validační report. Pro numerický baseline
byl použit skutečný AP STABILIZE, nikoli AP controller podruhé v UTB větvi.

## Parametry a integrační body

| Parametr | AP index | Default | Kontrakt |
|---|---:|---:|---|
| UTB_A_ENABLE | 36 | 0 | Explicitní pilotní opt-in, přesně 1. |
| UTB_A_ANGLE | 37 | 5° | Konečná kladná norma reference nejvýše 5°; dokumentovaný rozsah 0,1–5°. |
| UTB_A_YMAX | 38 | 0,1 rad/s | Konečné kladné maximum nejvýše 0,1; dokumentovaný rozsah 0,01–0,1. |
| UTB_A_AW | 39 | 0 | 0 saturation stop; 1 původní allocation feedback, žádná nová AW matematika. |

Změna limitů/policy za běhu vyvolá CONFIG stop. Změny `RAT_*` netuní nový
pilotní preset. Větší úhel vyžaduje samostatnou revizi a experimenty; samotný
parametr nepřekoná hard guard stávajícího attitude helperu. Původní indexy se nemění.

Nové `AP_UTB_AngleRC.*` řeší pouze pilotní adaptér. Třída `ModeUTBAngleSITL`
a její log metadata jsou v `libraries/AP_UTB/Copter/`. Nativní integrační
body jsou mode enum/lookup/instance, AVAILABLE_MODES, výhradní rate dispatch,
FSTRATE/custom-controller exclusion a jediný post-output log callback.
AP_Motors, HAL/DShot, AHRS/INS/EKF, arming source a původní regulátory se nemění.

Aktivní kód vyžaduje současně AP_UTB_ENABLED, AP_UTB_RATE_SITL_ENABLED,
CONFIG_HAL_BOARD==HAL_BOARD_SITL a multicopter frame. H743 build s násilně
zapnutou active build option nemá ANGLE symboly, název, log UARC ani A_* parametry.
Pouhý runtime parametr proto aktivní hardware autoritu nevytvoří.

## Časové pořadí a logovací overhead

Původní INS → rate dispatch → motor output → post-output capture/log →
`update_flight_mode` zůstává. `run()` produkuje RC snapshot pro další rate slot.
První output po mode entry ještě může obsahovat předchozí AP throttle kontext;
RPY jsou při entry vynulované a vlastní PID nejdříve primuje/resetuje historii.
Throttle/spool jsou původní AP; SHUT_DOWN požadavek neznamená okamžité přepsání
spool fyziky nebo tvrzení, že se každý skutečný PWM výstup ihned rovná nule.

`UARC` používá dosud volné vehicle log ID 30 (původní ID se nemění), 59 B
na záznam, při 400 Hz dalších 23 600 B/s bez metadata/backend overheadu.
UARX/UAMT/UAFB/UALT/UAAT/UAST se použijí z existujících SITL experimentů.
`TimeUS/Seq` spojují RC snapshot, reference, PID a motor output; RCms/Age
zachovávají původní RC timestamp a jeho stáří. Log export neurčuje controller rate.

Nové metriky `Log` a `Run` jsou host `steady_clock` wall µs, nikoli virtuální
SITL perioda ani hardware čas. `Log` je doba **předchozího** capture/log callbacku
(první záznam 0); zahrnuje původní logger API a capture, není izolovaným časem
samotného UARC zápisu. `Run` je poslední pilotní mode run; UAMT.Ex měří vlastní
rate callback. Je nutné odlišit host scheduling preempci od ceny samotné matematiky.

APN/AN=0 jsou deklarace této výhradní source větve, nikoli nový hook ve všech
AP controller funkcích. Důkaz autority spojuje audit návratu před AP PID,
zákaz custom controlleru/rate threadu, skutečné UTB počty a motorové sekvence.
Historické i nové výsledky neobsahují nezávislý profiler celého call graphu.

## Rozsah chování proti Betaflight / AP STABILIZE

| Vlastnost | UTB ANGLE | Srovnání / hranice |
|---|---|---|
| Páčka → náklon, střed → rovina | Lineární kalibrovaný AP vstup, reduced rotation P | Funkční auto-level princip BF ANGLE; žádná kopie BF PID. |
| Yaw | Body rate, střed 0 | Bez heading hold a bez headfree transformace. |
| Throttle | Původní AP manual expo/hover parametr, žádný angle boost | Původní AP STABILIZE angle boost zapíná; BF arming/mixer se nepřebírá. |
| Limity | 5° norma / 0,3 rad/s RP / 0,1 yaw | Širší BF úhlový rozsah není ověřen. |
| Shaping | Bez nové rampy/expo pro RP; skutečný AP deadband/reverse | Původní AP STABILIZE má vlastní attitude input shaping; BF aktuální implementace má další funkce. |
| Saturace | Původní UTB stop nebo explicitní původní feedback | Žádné tvrzení o BF airmode či stejné yaw prioritě aktivního allocatoru. |
| Mode/arming/failsafe | Nativní ArduPilot infrastruktura + experimentální latched stop | BF AUX/arming/failsafe nejsou importovány. |

Literatura/koncepce: oficiální BF Modes popisuje self-level ANGLE; aktuální
[BF pid.c](https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.c)
obsahuje vlastní implementaci, která zde není převzata. ArduPilot source
convention: RC normalizace, throttle, mode/spool/arming. Vlastní engineering
rozhodnutí: nový mode 42, kruh 5°, fixed preset, frozen konfigurace a fail-stop.
Konkrétní výsledky a neověřené body jsou v [SITL validaci](UTB_ANGLE_MODE_SITL_VALIDATION_CS.md).

Původní AP `last_update_ms` může být při přetečení přesně nula; nový adaptér
proto nepoužívá zero timestamp jako další sentinel. Chybějící RC určuje
původní `has_valid_input()` (včetně `has_ever_seen_rc_input`), stáří se počítá
unsigned rozdílem. Unit test pokrývá i tento přesný bod přetečení.

Testovací hover PWM je vlastnost konkrétního SITL modelu a konfigurace,
nikoli nový firmware throttle parametr nebo výšková regulace. Zkoušky se
nesmí vyhodnotit jako PASS, pokud ruční throttle poruší nezměněnou svislou
obálku. Předchozí experimentální mapování/trim pokusy se uchovávají odděleně.

## Nativní klasifikace a omezení failsafe

`ModeUTBAngleSITL` má `has_manual_throttle=true`, `is_autopilot=false`,
`requires_GPS=false`. To neodstraňuje vlastní podmínku validní AHRS výšky
pro experimentální svislou obálku. Původní hover learning v
`ArduCopter/Attitude.cpp` manuální throttle režimy vynechává. Detekce přistání,
arming delay a AP spool zůstávají původní.

`Copter::should_disarm_on_failsafe()` v `ArduCopter/events.cpp` má zvláštní
větev pro STABILIZE/ACRO s nulovým throttle. Mode 42 tuto větev nepřebírá:
platí původní default disarm při landed/arming delay. Původní `events.cpp`
se nemění. Nový režim není automaticky ve všech starých výčtech AP módů.

**Ověřená konfigurace GCS failsafe vyžaduje `FS_GCS_ENABLE=1`,
`FS_OPTIONS=0`.** Zděděná hodnota FS_OPTIONS=16 dovoluje při GCS výpadku
pokračovat v pilotním režimu. To je v konfliktu s UTB politikou, která
při `any_failsafe_triggered()` zastaví tah. Skutečný oddělený test s touto
kombinací vedl k latched stop=7 a pádu simulovaného stroje. Není to bezpečná
podporovaná letová kombinace a nelze ji vydávat za úspěšné přistání.
Nebyla změněna nativní AP failsafe politika ani zmírněn UTB guard.
Nový runner nastavuje FS_OPTIONS=0 explicitně; širší kombinace AP failsafe
voleb zůstávají před dalším rozšířením předmětem samostatného auditu.

MAVLink `RC_CHANNELS_OVERRIDE` v lokální AP implementaci aktualizuje také
čas posledního GCS kontaktu (`GCS_Common.cpp::handle_rc_channels_override`).
GCS test proto zastaví heartbeat i overrides a drží RC odděleným původním
`AP_RCProtocol_UDP` přijímačem, 8 little-endian uint16 kanálů na vlastněném
SITL UDP portu 5501. Runner před vysláním ověřuje vlastnictví socketu PID
svého simulátoru. Žádný fyzický nebo vzdálený endpoint nepřijímá.
