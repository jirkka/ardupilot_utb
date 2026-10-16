# FÁZE 2A: experimentální aktivní rate řízení pouze v SITL

Stav: implementovaný prototyp. Hardware aktivní autorita není implementována.
Větev `test/skystars-5inch`, výchozí HEAD
`687f825d228b12a64d36651072aed19356178217`; změny nejsou commitnuté.
Výsledky aktuálních buildů a experimentů uvádí
[validační report](UTB_ACTIVE_SITL_VALIDATION_CS.md).

## Integrační bod a vlastníci

Původní řetězec je INS update → `Copter::run_rate_controller_main()` →
`AC_AttitudeControl_Multi::rate_controller_run()` → AP_Motors R/P/Y setters →
`motors_output_main()` / `motors_output()` → `Mode::output_to_motors()` →
AP_Motors allocation/spool/linearization/output. AHRS update a
`update_flight_mode()` následují po motorovém outputu. FSTRATE může původní rate
PID a output přesunout do samostatného rate threadu.

Zásah je v `ArduCopter/Attitude.cpp`, přímo v původním rate slotu.
Pouze pro explicitní režim **UTB_RATE_SITL = 41** se slot předá
`ModeUTBRateSITL::rate_control()`. Potom se vrátí; původní AP rate PID není
v této větvi zavolán. Zbývá běžný reset dočasných sysid/scale vstupů.
Ve všech ostatních režimech zůstává původní větev.

Číslo 41 bylo zkontrolováno proti skutečnému `Mode::Number`: původní lokální
UTB_ACRO má 40, ostatní definované módy končí 28; 30 a 127 jsou rezervované.
41 je lokální experimentální číslo tohoto forku, nikoli upstream standard.
Mode registration, instance, parametry i vtable jsou compile-guardované.
Původní `ModeUTBAcro`, jeho vstup pouze DISARMED a zákaz všech arming metod
nebyly změněny.

Vlastník vybraný při vstupu:

| UTB_S_OWNER | Chování v režimu 41 |
|---|---|
| 0 (default) | Vstup odmítnut; žádná aktivní autorita |
| 1 | Jedna instance původního AP_UTB_RateController počítá zásah; AP rate PID se nespouští |
| 2 | Stejný experimentální reference/throttle shell, ale jediný rate PID je původní AP; UTB PID update count je nula |

Vlastníka nelze za letu změnit parametrem. Neshoda proti entry snapshotu
invaliduje experiment. Nový vstup vyžaduje explicitní opuštění a GCS návrat.
`init()` přijímá pouze `ModeReason::GCS_COMMAND`; boot/RC automatický vstup není
povolen, ani při běžném disarmed `ignore_checks=true`.

## Compile-time izolace

`AP_UTB_RATE_SITL_ENABLED` má default 0 a Waf option
`--enable-UTB_RATE_SITL` s dependency UTB. Skutečná dostupnost je:

```cpp
#define AP_UTB_RATE_SITL_AVAILABLE \
    (AP_UTB_ENABLED && AP_UTB_RATE_SITL_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
```

Copter mode/hooky navíc vyžadují MULTICOPTER_FRAME. Aktivní helper nemá
AP_Motors/HAL output závislosti; motor setters jsou jen v guardovaném Copter
adapteru. Na fyzických board typech je availability false i při explicitním
request flagu 1. Nejde pouze o runtime parametr.

Skystars cross-compile s UTB/BENCH/request flag ON byl skutečně proveden:
ELF neobsahuje `ModeUTBRateSITL`, `AP_UTB_SITLRate`, S_OWNER ani jméno módu.
Stejná source podmínka vylučuje všechny ostatní fyzické HAL board typy;
každý z nich nebyl jednotlivě cross-kompilován. Skutečné FC nebylo otevřeno
ani přeflashováno; poslední známý firmware zůstává `f9cc3bcc…`.

## Proč varianta A

Varianta A předává normalizované R/P/Y výsledky UTB PID přes stávající
`set_roll/pitch/yaw`; žádná hypotetická M1..M4 hodnota není použita jako PWM.
Původní AP_Motors nadále provádí vlastní allocation, napěťovou/air-density
compensation, spool, thrust linearization, slew a output. Požadavky FF jsou
v experimentu nulové. Jediný `motors_output_main()` zůstává na původním místě.
Nový post-output callback pouze čte finální simulator PWM/actuator hodnoty
a loguje; nevolá output podruhé.

Varianta B by musela obejít druhé mixování a doložit kompatibilní přímé
thrust/spool/limit rozhraní. Diagnostický UTB MotorMixer není takovým
rozhraním; v této fázi se do aktivní cesty vůbec nezapojuje.
Jeho matematika a SHADOW feedback zůstávají beze změny.

## Saturace a anti-windup kontrakt

AP allocator nemá stejnou priority/achieved definici jako diagnostický UTB
allocator. AP_MotorsMatrix má yaw headroom, throttle tradeoff a motor-loss
politiku; UTB diagnostika zachovává společný poměr R/P/Y a může posunout
collective. Nejsou zaměnitelné.

První aktivní prototyp podporuje **pouze nesaturovanou oblast**:

- Žádné volání `RateController::feedback()` s hypotetickým UTB mixer achieved.
  Residual po resetu zůstává nula; integrál má původní IMAX clamp.
- `abs(raw PID) >= 1` invaliduje výpočet před předáním příkazu.
- Před dalším výpočtem se kontrolují předchozí AP_Motors roll/pitch/yaw
  a throttle-upper flags, po ustálení armed/full-spool kontextu.
- Zjištěná allocation limit invaliduje experiment, resetuje UTB historii a
  ukládá latched SATURATION. Nepokračuje se automaticky do obtížnějšího testu.
- Detekce AP limitů má jednu výstupní iteraci zpoždění; aktuální allocator
  flags vznikají až v následném původním motorovém outputu. Není to okamžitá
  rekonstrukce fyzického achieved momentu ani schválený anti-windup adapter.

Thrust nonlinearities/compensation/slew samy o sobě nejsou měřeným momentem.
Nulové limit flags nedokládají fyzicky dokonalé actuator tracking. Saturating
flight testy zůstávají nepodporované; latched stop se testuje samostatně.
Integrátor v tomto skutečném SITL closed-loop nyní ovlivňuje simulované těleso,
na rozdíl od dřívějšího H743 SHADOW letu. Stále to není hardware validace.

## FSTRATE, custom control a souběh

Entry vyžaduje raw `FSTRATE_ENABLE=0` a `using_rate_thread=false`.
SITL-only větev `Copter::get_fast_rate_type()` vrací během režimu 41 DISABLED,
takže runtime změna parametru nemůže spustit konkurenční thread. Samotný
adapter zároveň čte raw `g2.att_enable`; neshoda se invaliduje jako FAST_RATE.
Pokud rate thread již běží, entry je odmítnuto. HAL/rate thread/DShot kód se
nepřepisuje. Mimo režim 41 getter vrací původní hodnotu.

Fast custom-controller callback se v režimu 41 vrací bez update. Nemůže tedy
po UTB přepsat motorové požadavky. Mimo režim je jeho původní chování zachováno.
Testovací konfigurace nemá aktivní CC backend. Nejde o globální odstranění
původní custom-control funkce.

## Reference, throttle a časování

Reference má oddělený SITL generátor, nepoužívá ani neuvolňuje ochrany
BENCH/DISARMED generátoru. `UTB_S_PROFILE=0..6`, amplitude `UTB_S_AMP=0..0.3`
rad/s, `UTB_S_RUN` pouze startuje finite sequence po arming/full spool.
Nenastavuje arming nebo mód. Sekvence trvá 3200 mode updates při nominal
400 Hz: 8 s, poté zero reference. Další sekvence vyžaduje mode exit/entry.
Profile/amp/gains/delay/owner jsou fixované při vstupu.

Oba vlastníci používají společné
`input_rate_bf_roll_pitch_yaw_2_rads()` pro rate shaping. Není to rate PID ani
trainer/attitude hold. Fast slot čte target z předchozího mode update, stejně
jako původní AP scheduler. Loguje raw reference i skutečný sdílený shaped
target a aktuální AHRS primary gyro; AP controller se nespouští kvůli capture
podruhé. Stav se validuje časem odebraným **po samplingu**, s původními
state/gyro/IMU/quaternion kontrolami. Repeated INS timestamp/změna primary gyro
invalidují kontext.

Throttle je pilotní manual throttle přes stávající AP mapu, bez angle boost.
Při armed entry se mid-throttle reference odvodí z posledního AP throttle
požadavku; při disarmed entry z `get_throttle_hover()`. Hodnota musí být finite
0..1. Spool je původní motor state machine, SHUT_DOWN/GROUND_IDLE mají zero
throttle. Nejde o altitude/position controller ani příslib visu.
Diagnostický collective 0,5 se pro let nepřebírá.

Společná experimentální příprava využívá standardní GUIDED takeoff/hover
**před režimem 41**, s původním AP tune. Teprve po health/position a klidovém
gyro/attitude ověření následuje explicitní GCS přenos. Po dokončeném měřicím
okně je případný LAND opět explicitní cleanup mimo měření, nikoli skrytá
záchrana nestabilního UTB testu.

V UTB větvi se nevykonávají AP rate-loop helpers `update_throttle_gain_boost`
a `update_throttle_rpy_mix`. `set_throttle_out()` stále nastavuje throttle a
throttle_avg_max, ale vnitřní mix stav může být držený z přípravy. Prototyp
proto netvrdí ekvivalenci high-throttle-slew nebo limitovaných režimů; měření
používá konstantní malý throttle pracovní bod a nesaturovaný rozsah.

`UTB_S_DLY=1` vloží pevný jeden cycle transport delay R/P/Y příkazu pro oba
vlastníky, v samostatném Vector3f bufferu; 0 je default. Reset při inactivity,
fault/exit. Je to explicitní simulační adapter delay, nikoli změřený ESC delay,
sensor delay nebo změna HAL/DShot. Throttle se tím nezpožďuje.

## Lifecycle a bezpečný stop

Entry/exit resetují UTB I/D/previous/residual/priming, delay buffer i AP target,
I a filter historii. Arming/full-spool transitions re-prime UTB historii.
Změna zmrazené konfigurace, frame !=1/18, FSTRATE, failsafe, neplatný dt,
state, nonfinite target/output nebo experimentální envelope invalidují režim.
Nominal musí být 400 Hz, dt 0,625–10 ms podle původního PID kontraktu,
age nejvýše 10 ms. Target norm <=0,6 a gyro norm <=2 rad/s jsou bounds tohoto
malého simulačního experimentu, nikoli hardware bezpečnostní limity.

Latched stop nastaví R/P/Y/FF=0, throttle=0 a desired spool SHUT_DOWN;
standardní output chain zůstane jediný. Žádný automatický AP PID fallback.
Neshoda nedovolí opětovné armování v tomto kontextu; nový vstup musí být
explicitní. Spool-down trvá podle původní AP politiky, nejde o nový okamžitý
fyzický emergency stop. První fault detekovaný v rate slotu vysílá STATUSTEXT;
host při jeho přijetí ukončí další testovací požadavky a vlastní SITL proces.
Host má také attitude envelope a konečný časový limit. BIN/tlog se uchovají.

Původní arming a failsafe implementace nejsou změněny. Simulační arming
probíhá standardně s ARMING_CHECK=1, FS_THR/GCS zapnutými, bez force-arm.
Force request je použit jen pro negativní důkaz původního UTB_ACRO zákazu.

## Evidence a omezení logů

Statické SITL-only log IDs 24 UARX, 25 UAMT nekolidují s vehicle IDs ani
common namespace od 32. UTBC=31 zůstává stejné. UARX má tři axis packets:
TimeUS/Seq/owner/fault/armed/spool/dt/raw-ref/used-target/actual-gyro/P/I/D/output.
UAMT čte po výstupu skutečné simulator PWM a normalizované AP actuators,
AP/UTB rate-call counts, motor-call count a host steady-clock rate-slot µs.

Časy jsou oddělené: virtual loop dt/TimeUS versus host steady-clock execution.
SITL PM.Load se může rovnat nule při stopped clock a není H743 CPU měření.
Výpisy RATE/PIDR/PIDP/PIDY při UTB owner mohou obsahovat starou interní AP PID
historii, protože ten není spuštěn. Autoritativní aktivní data jsou UARX/UAMT,
nikoli tyto legacy AP PID logy.

Čtyři fixed-block zápisy jsou pouze v SITL post-output callbacku. Mohou měnit
host execution/pacing a mohou být backendem odmítnuty; pro oba vlastníky je
logging stejný. Není to H743 logging architektura, na něm tato cesta neexistuje.
Původní SHADOW worker/queue/controller/mixer nejsou upraveny.

## Stav ověření

Implementováno a unit/SITL ověřeno: samostatný mode, jedna autorita, původní
motorový řetězec, finite malé reference, detekce faultů a compile-out.
Skystars cross-build a ELF/parameter-string kontrola jsou ověřené.
Aktivní řízení na hardware není implementováno ani ověřeno. Další HAL board
typy jsou vyloučené source guardem, nebyly všechny jednotlivě buildnuté.
Saturace nemá aktivní anti-windup adapter; zůstává stop podmínkou.
