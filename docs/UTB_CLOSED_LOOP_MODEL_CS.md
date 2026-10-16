# FÁZE 1H-D: rotační model a uzavřená UTB smyčka

**SIMULATED – NOT HARDWARE VERIFIED.** Stav 12. 10. 2026, větev
`test/skystars-5inch`, HEAD `687f825d228b12a64d36651072aed19356178217`.
Kód PID/mixeru odpovídá skutečnému firmware `f9cc3bcc`; produkční C++ se mezi
oběma commity nezměnilo. Existující H743 SHADOW dokazuje výpočty nad skutečným
gyrem, ne uzavřenou UTB dynamiku. Níže jde o samostatnou Python simulaci.

**Implementováno:** fyzikální model, scénáře, metriky, SVG grafy, lokální
lineární analýza a replay existujícího C++ unit runneru. **Offline ověřeno:**
geometrie, analytické příklady, invariance, determinismus, konvergence a
matematická shoda. **Hardware neověřeno:** všechny fyzikální parametry,
motorové směry/síly/zpoždění pro tuto konfiguraci a fyzická stabilita.
**Navrženo:** měření fyzikálních parametrů a další samostatně schvalované kroky.

Software nemá MAVLink, serial ani HAL výstup. Produkční RateController,
Reference, MotorMixer, AP_Motors, arming/failsafe/EKF/DShot nebyly upravené.
Nový firmware nebyl sestaven ani nahrán. UTB_ACRO zůstává zakázaný pro armování.

## Původ modelu a rozlišení evidence

- **ArduPilot source convention:** FRD body rates, frame18, ordering a PID/mixer
  podle skutečných zdrojů v této větvi.
- **Odborný matematický základ:** Newton–Euler a stabilita diskrétního lineárního
  systému; návaznost na rovnice a primární zdroje v
  [UTB_CONTROL_THEORY_CS.md](UTB_CONTROL_THEORY_CS.md), oddíly3,4,9,11–14,22.
  Zde použité body rovnice jsou přímo rozepsané z bilance momentu hybnosti;
  nejde o identifikaci parametrů podle starší práce.
- **Engineering decision pro tento model:** normalizovaný actuator stav,
  exponenciální tahová křivka, lineární damping, RK4 a celočíselné delay FIFO.
  To jsou explicitní modelové předpoklady, nikoli vlastnosti změřené na FC.
- **Measured H743:**400Hz matematický controller/log experiment a gains
  použité v DISARMED SHADOW. Žádná fyzikální hodnota J/Fmax/kQ/tau nebyla
  z těchto logů identifikována. Starší3″ model nebyl použit pro numerické parametry.

## Přesná PID a allocator část

`Tools/autotest/utb_closed_loop.py` importuje `Reference` a `f32` z existujícího
`utb_math_reference.py`. PID a allocator nejsou implementovány znovu.
Input dt/reference/gyro/gains jsou quantized float32. Reference počítá s
propagovanými mezemi zaokrouhlení; jeho double střed není bitová emulace všech
C++ operací. Shoda se ověřuje existujícími float32 error bounds a přesnými flags.
Fyzikální integrace je double. Tyto dvě numerické vrstvy jsou oddělené.

Zdrojové rozhodující cesty pod `/home/jirka/ardupilot_utb/`: [PID:26](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_UTB/AP_UTB_RateController.cpp:26>), [mixer:14](<//wsl$/Ubuntu/home/jirka/ardupilot_utb/libraries/AP_UTB/AP_UTB_MotorMixer.cpp:14>).

| Oblast | Soubor a řádky |
| --- | --- |
| Validity/configuration, reset | libraries/AP_UTB/AP_UTB_RateController.cpp:26–65 |
| dt/state a priming | stejný soubor:68–85 |
| P/I/D a clipping | stejný soubor:86–124 |
| Achieved residual | stejný soubor:127–140 |
| Matice, scale, collective projection, achieved | libraries/AP_UTB/AP_UTB_MotorMixer.cpp:14–70 |
| Operational policy a state/INS guard | libraries/AP_UTB/AP_UTB.cpp:272–367 |

Pro osu j:

```math
e[k] = r[k] - y[k]
P[k] = Kp e[k]
deltaI = Ki e[k] dt[k]
block = (residual[k-1] > 1e-5 and deltaI > 0)
     or (residual[k-1] < -1e-5 and deltaI < 0)
I[k] = 0                         if Ki=0
     = clamp(I[k-1] + (block ? 0 : deltaI), -IMAX, +IMAX)
tauD = 1 / (2 pi D_Hz)
beta = dt[k] / (tauD + dt[k])
acc[k] = (1-beta) acc[k-1] + beta (y[k]-y[k-1])/dt[k]   if Kd>0
D[k] = -Kd acc[k]
raw[k] = P[k] + I[k] + D[k]
c[k] = clamp(raw[k], -1, +1)
residual[k] = raw[k] - achieved_diagnostic[k]
```

P pracuje s aktuální chybou; I je obdélníková aktualizace; D je derivace
měření s backward-Euler filtrem, bez derivative kick z reference. První
validní sample jen primuje předchozí gyro a není controller/mixer valid.
Reset vymaže previous, acceleration, I, residual i primed. Změna konfigurace
v referenčním modelu resetuje historii. Původní29-scénářová regrese pokrývá
také reset/invalidation, hraniční dt, frame a změny gains.

Nominal dt je `.0025f`. Wrapper odmítá konfiguraci mimo skutečné meze:
Kp/Ki0–5, Kd0–1, IMAX0–1, cutoff0–200Hz; při Kd>0 cutoff>0 a≤40Hz.
Actual dt je v rozsahu0,25–4×nominal. `dt_5ms` je matematický test actual dt
při stejném nominal400Hz, nikoli schválení jiného skutečného scheduleru.
Jitter ±20% mění actual dt i délku plant kroku, controller má stále nominal400.

Simulace předpokládá čerstvý zdravý gyro state a platný frame1/18. Neobsahuje
EKF/AHRS, RC, arming ani AP_UTB time-budget policy. **Nenahrazuje jejich guards.**
Measurement delay je fázové zpoždění signálu v modelu, nikoli simulace
zastaralého INS timestampu: skutečný State age/new-INS guard by mohl experiment
zamítnout dříve. Analýza dlouhých delay je matematický stress test.

BF_X_REV normalizovaná matice, řádkyM1–M4, sloupceR/P/Y:

```math
B = 0.5 * [-1,-1,+1;
           -1,+1,-1;
           +1,-1,-1;
           +1,+1,+1]
d = B c
scale = min(1, 1/(max(d)-min(d)))      pokud span>0
d_scaled = scale d
t_ach = project(t_command, [-min(d_scaled), 1-max(d_scaled)])
m_i = clamp(t_ach + d_scaled_i, 0, 1)
achieved_RPY = B^T m,   achieved_T = mean(m)
```

Vlastnosti `B^T B=I`, `B^T 1=0`. Firmware má epsilon detaily při vyhodnocení mezí
a flags; ty přebírá existující Reference. Priorita: společný
poměrR/P/Y, zachovat velikost pokud lze, pak collective shift; při span>1
společné škálování. Yaw nemá nižší prioritu. Jde stále o diagnostický allocator.

**Zásadní limit feedback:** achieved je rekonstrukce z diagnostických m_i,
nikoli měření skutečného tahu po omezení/lag/delay/poruše motoru. Aktuální
directional anti-windup tedy nepozná omezení zavedené až v samostatném modelu
aktuátorů. To je zachovaná vlastnost současného kódu, ne skrytá oprava.

## Samostatný převod aktuátorů na fyzikální veličiny

Každý motor má příkaz m_i∈[0,1], strop h_i∈[0,1] a stav x_i:

```math
u_i = clamp(delayed(m_i), 0, h_i)
dot(x_i) = (u_i - x_i) / tau_motor
F_i = Fmax * x_i^gamma                      [N]
```

Při tau_motor=0 x_i=u_i okamžitě. gamma1 představuje lineární převod
normalizované hodnoty na tah; gamma2 je jiná ilustrativní křivka, nikoli
kalibrace ESC/RPM. x_i není skutečné rotorové RPM. Initial x_i=min(0,5,h_i),
body rates0; při priming se virtuální příkaz drží na bias0,5. Tato modelová
initial condition není fyzický motor output ArduPilotu.

Kladný body Z směřuje dolů, rotorová síla je `(0,0,-F_i)`.
Pro vzdálenost středu→motor L a a=L/√2:

| Motor | x [m] | y [m] | Kladný reakční moment Z v modelu |
| --- | ---: | ---: | --- |
| M1 rear-right | −a | +a | +kQ F1 |
| M2 front-right | +a | +a | −kQ F2 |
| M3 rear-left | −a | −a | −kQ F3 |
| M4 front-left | +a | −a | +kQ F4 |

```math
tau = sum_i r_i × (0,0,-F_i) + (0,0,s_i kQ F_i)
tau_x = sum(-y_i F_i)
tau_y = sum(+x_i F_i)
tau_z = kQ (F1-F2-F3+F4)                    [N m]
```

kQ má jednotku m a modeluje poměr reakčního momentu k tahu, nikoli
bezrozměrný mixer yaw coefficient. Pro gamma1, shodné pohony bez omezení,
malé změny při t0,5:

```math
tau_x = sqrt(2) L Fmax achieved_R
tau_y = sqrt(2) L Fmax achieved_P
tau_z = 2 kQ Fmax achieved_Y
```

Pro gamma≠1 je lokální gain navíc násoben gamma*t0^(gamma−1);
mimo malý signál tento lineární převod neplatí. Collective bias není
identifikovaný hover throttle. Celkový tah se spočítá, ale translace, výška,
pozice, AUTO ani attitude outer loop nejsou v modelu.

## Rotační plant

Body angular momentum H=Jω. V inerciálním systému platí dot(H)=tau;
při zápisu v rotujícím body systému se přidá ω×H:

```math
J dot(omega) = tau - omega × (J omega) - diag(Dx,Dy,Dz) omega
omega = [p,q,r],    J=diag(Jx,Jy,Jz)
Jx dot(p) = tau_x - Dx p - (Jz-Jy) q r
Jy dot(q) = tau_y - Dy q - (Jx-Jz) r p
Jz dot(r) = tau_z - Dz r - (Jy-Jx) p q
```

Plný model zahrnuje vazbu os. `coupled=False` ji explicitně odstraní a je
samostatný decoupled scénář. Konstantní kolektiv nemá body moment při
shodných motorech. Bez momentu a damping zachovává model energii
E=½ωᵀJω a normu momentu hybnosti ||Jω||. Testuje se obojí.

J musí být finite/positive a splnit principal inertia triangle inequality
2max(J)≤sum(J). Pozitivita sama by nestačila pro fyzicky možné hlavní momenty.
Nejsou zahrnuté rotorové gyroskopické momenty, pružnost rámu, vibrace,
ground effect, aerodynamické interakce, změna baterie, ESC idle/spool nebo
gyro bias/notch/lowpass ArduPilotu. Těleso má body referenci, nikoli Euler
angle rates; orientace není k uzavření samotné rate smyčky potřeba.

## Parametry: co je změřené a co ilustrativní

| Parametr | Základ modelu | Evidence / sweep |
| --- | --- | --- |
| Nominal controller rate |400Hz | Skutečný SHADOW benchmark; fyzikální closed loop neověřen |
| Gains všech os |Kp0,31; Ki0,23; Kd0,004; IMAX0,19; D_Hz20 | Skutečně použité v DISARMED SHADOW; nejsou schválené flight gains |
| Jx/Jy/Jz |0,002 /0,0025 /0,004 kg m² | ILUSTRATIVNÍ; násobky0,5/1/2 |
| L |0,11m | ILUSTRATIVNÍ, žádné převzetí3″ dronu |
| Fmax |3N/motor | ILUSTRATIVNÍ, není měřený max tah5″ pohonu |
| kQ |0,015m | ILUSTRATIVNÍ |
| Motor tau |30ms | ILUSTRATIVNÍ;0/10/30/80ms |
| Damping všech os |0,003 N m s/rad | ILUSTRATIVNÍ; násobky0/1/3 |
| gamma |1 | ILUSTRATIVNÍ; další scénář2 |
| Collective bias |0,5 | ENGINEERING DECISION, ne hover estimate |
| Actuator ceilings |1 pro každý motor | MODEL; stress všichni0,6 / M1 pouze0,3 / test všechny0 |
| Sensor/compute/actuator delay |0 cyklů navíc | MODEL;5ms a30ms rozdělené FIFO, sensor sweep0–200ms |
| Gyro noise sigma |0 | MODEL; stress independent Gaussian0,02rad/s |
| Jitter |0 | MODEL; uniform±20%; seed1729 |

Intervaly jsou analýza citlivosti, nikoli confidence intervaly pro konkrétní
5″ dron. Fmax/J mohou měnit gain o násobky. Nemáme žádné fyzikální hodnoty,
které by uzavíraly model jako model reálného Skystars5″ stroje.

## Časování a integrace

V kroku k nejprve odečteme ω(t_k) nebo vzorek z sensor FIFO, přidáme šum,
quantize gyro/r/dt, zavoláme Reference a allocator, zařadíme motor command do
FIFO compute+actuator delay a potom integrujeme plant do t_(k+1).
Controller pracuje jednou za krok. Command je mezi kroky ZOH.

Každý delay je celočíselný počet celých controller cyklů. Sensor1/compute1
znamená při nominal dt5ms; sensor4/compute4/actuator4 celkem30ms.
Computation delay není naměřená execution time Pythonu/H743. Při jitter se
fyzická délka FIFO mění; nejde o přesně konstantní sekundové zpoždění.

RK4 integruje současně3 body rates+4 motorové stavy, standardně4 substeps
na400Hz krok. Pro krátký tau se substeps zvýší tak, aby h≤tau/8.
Žádné clipping body rate nezakrývá nestabilitu. Nonfinite/rate>|10000|rad/s
vyvolá numerickou chybu;10000 je diagnostický fail-safe numeriky, nikoli
bezpečnostní limit dronu. V provedených scénářích se neuplatnil.

Plant čas jde ze skutečných použitých dt. Reference skok začne v prvním
vzorku s t≥1s; rise je10–90%, settling je první následný okamžik, po němž
zbývající zaznamenaná odezva zůstane v±2% amplitude. Rozlišení je controller
krok. Nenulové finite-window settling není důkaz stability na nekonečném horizontu.

IAE=∑|r−ω|dt [rad], ITAE=∑t|r−ω|dt [rad s], t je čas od začátku celé
session. Ustálená chyba je průměr poslední1s; nulová střední chyba sama
nevylučuje oscilace, proto se ukládá i peak-to-peak. RMS raw/I maxima,
výstupní clipping, mixer scale/shift a physical actuator limit jsou oddělené.
Measurement noise RMS zahrnuje quantization, sensor-error RMS navíc delay.
Pro sinus a sign reversal se nehlásí nesmyslné jednoskokové settling metriky.

## Reprodukovatelné výsledky

Finální artefakty: `/home/jirka/ardupilot_utb/tmp/utb-1hd-offline-20261012_162639/results/`.
`simulation.json`: každý parametr a metrika,24 scénářů, seed1729.
Každý scénář má CSV se všemi vzorky. Kvůli float32 dt/hranici duration
je skutečná délka dána součtem dt (typicky8,0025s), nikoli přesně požadovanými8s;
JSON ukládá skutečné counts/duration. `pid-replay.txt` je všech75226 vstupů
do existujícího C++ PID/mixer runneru. `stability.json`: skutečné komplexní póly,
81 sensor delay bodů a108 parametrických kombinací. Deset SVG obsahuje všechny
uložené body bez vyhlazení/decimace a viditelné označení simulace.

Následující tabulky jsou automaticky odvozené z finálních JSON, čísla zůstávají
modelová. Přehled zahrnuje zero, skoky všech os/negative, reversal, tři sinusy,
současné osy, saturaci/recovery, delay, jitter/dt, noise, limity aktuátorů,
quadratic curve, inertia/lag/damping a decoupled variantu.

| Osa / step0,3rad/s | Rise10–90% [ms] | Overshoot [%] | Settling±2% [ms] | IAE [rad] | Poslední1s mean error [rad/s] |
| --- | ---: | ---: | ---: | ---: | ---: |
| roll | 37.5 | 3.597 | 87.5 | 0.008974 | 0.00000160 |
| pitch | 40.0 | 3.938 | 107.5 | 0.010678 | -0.00000486 |
| yaw | 277.5 | 0.473 | 482.5 | 0.048283 | -0.00002568 |

| Scénář | max abs rate R/P/Y [rad/s] | IAE součet os [rad] | Common scale [s] | Physical ceiling [s] | Poslední1s mean error R/P/Y [rad/s] |
| --- | --- | ---: | ---: | ---: | --- |
| zero | 0.0000/0.0000/0.0000 | 0.000000 | 0.000 | 0.000 | 0.0000/0.0000/0.0000 |
| roll | 0.3108/0.0000/0.0000 | 0.008974 | 0.000 | 0.000 | 0.0000/0.0000/0.0000 |
| pitch | 0.0000/0.3118/0.0000 | 0.010678 | 0.000 | 0.000 | 0.0000/-0.0000/0.0000 |
| yaw | 0.0000/0.0000/0.3014 | 0.048283 | 0.000 | 0.000 | 0.0000/0.0000/-0.0000 |
| negative | 0.3108/0.0000/0.0000 | 0.008974 | 0.000 | 0.000 | -0.0000/0.0000/0.0000 |
| reversal | 0.3216/0.0000/0.0000 | 0.026847 | 0.000 | 0.000 | -0.0000/0.0000/0.0000 |
| axes | 0.3110/0.3114/0.3023 | 0.069792 | 0.000 | 0.000 | -0.0000/-0.0000/-0.0000 |
| saturation | 15.5022/9.0350/9.4032 | 20.652717 | 3.037 | 0.000 | 0.0047/0.0319/0.0372 |
| sine_0.5 | 0.2997/0.0000/0.0000 | 0.113564 | 0.000 | 0.000 | 0.0009/0.0000/0.0000 |
| sine_2 | 0.2973/0.0000/0.0000 | 0.450518 | 0.000 | 0.000 | 0.0002/0.0000/0.0000 |
| sine_10 | 0.2120/0.0000/0.0000 | 1.813385 | 0.000 | 0.000 | 0.0005/0.0000/0.0000 |
| delay_5ms | 0.3441/0.0000/0.0000 | 0.010372 | 0.000 | 0.000 | 0.0000/0.0000/0.0000 |
| delay_30ms | 5.6922/0.0000/0.0000 | 22.436752 | 0.000 | 0.000 | 0.2140/0.0000/0.0000 |
| jitter20percent | 0.3112/0.3115/0.3023 | 0.069800 | 0.000 | 0.000 | -0.0000/-0.0000/-0.0000 |
| dt_5ms | 0.3172/0.0000/0.0000 | 0.009287 | 0.000 | 0.000 | 0.0000/0.0000/0.0000 |
| noise | 0.0216/0.0189/0.0066 | 0.081303 | 0.000 | 0.000 | 0.0010/0.0003/-0.0009 |
| actuator_ceiling | 16.2934/8.9692/8.9491 | 30.561700 | 3.430 | 3.707 | 0.0084/-0.0079/0.0430 |
| one_motor_weak | 0.8756/0.8629/0.8484 | 12.788054 | 0.000 | 8.002 | -0.5000/-0.5096/0.4979 |
| quadratic_thrust | 0.3086/0.3114/0.3024 | 0.070504 | 0.000 | 0.000 | -0.0000/-0.0000/-0.0000 |
| inertia_half | 0.3048/0.0000/0.0000 | 0.008445 | 0.000 | 0.000 | 0.0000/0.0000/0.0000 |
| inertia_double | 0.3116/0.0000/0.0000 | 0.016870 | 0.000 | 0.000 | -0.0000/0.0000/0.0000 |
| lag_80ms | 0.3735/0.0000/0.0000 | 0.018392 | 0.000 | 0.000 | 0.0000/0.0000/0.0000 |
| no_damping | 0.3187/0.0000/0.0000 | 0.016420 | 0.000 | 0.000 | -0.0000/0.0000/0.0000 |
| decoupled | 14.9603/9.9734/7.7767 | 11.046093 | 0.585 | 0.000 | -0.0204/0.0135/-0.0208 |

| Roll sinus frequency [Hz] | Closed-loop amplitude ratio | Phase [deg] |
| ---: | ---: | ---: |
| 0.5 | 0.998871 | -4.893 |
| 2 | 0.991040 | -19.568 |
| 10 | 0.708542 | -104.194 |

Nominální spectral radius R/P/Y: 0.998149756/0.998145023/0.998130888.

| Sensor delay sweep při ostatních nominal parametrech | Poslední zkoušený stabilní bod | První zkoušený nestabilní bod |
| --- | --- | --- |
| Roll | 15.0ms | 17.5ms, radius1.000252467 |
| Pitch | 20.0ms | 22.5ms, radius1.000737750 |
| Yaw | 200ms | Nenalezen v testovaných0–200ms |

Ve 108 kombinacích J×0,5/1/2, D×0/1/3, lag0/10/30/80ms a sensor delay0/5/20ms je 27 lokálně nestabilních alespoň na jedné ose. Každá kombinace a všechny póly jsou v stability.json.

## Grafy

Všechny grafy: **SIMULATED – NOT HARDWARE VERIFIED**.
CSV slouží k přesnému čtení hodnot, SVG k přehledu.

- [Roll step](../tmp/utb-1hd-offline-20261012_162639/results/roll.svg),
  [Yaw step](../tmp/utb-1hd-offline-20261012_162639/results/yaw.svg),
  [Three axes](../tmp/utb-1hd-offline-20261012_162639/results/axes.svg).
- [Saturation/recovery](../tmp/utb-1hd-offline-20261012_162639/results/saturation.svg),
  [30ms delay](../tmp/utb-1hd-offline-20261012_162639/results/delay_30ms.svg).
- [Noise](../tmp/utb-1hd-offline-20261012_162639/results/noise.svg),
  [Actuator ceiling](../tmp/utb-1hd-offline-20261012_162639/results/actuator_ceiling.svg),
  [Weak motor](../tmp/utb-1hd-offline-20261012_162639/results/one_motor_weak.svg),
  [10Hz sine](../tmp/utb-1hd-offline-20261012_162639/results/sine_10.svg).
- [Discrete poles versus delay](../tmp/utb-1hd-offline-20261012_162639/results/delay-poles.svg).

## Lokální stabilita: podmínky platnosti

V pracovním bodě ω0, shodné pohony x_i=t0,5 se bilineární ω×Jω člen
v první derivaci vynuluje a osy se lokálně rozpojí. U gamma2 je zde slope
gamma*t0^(gamma−1)=1; stejná lokální matice proto nevypovídá o stejné
velkosignálové dynamice.

Pro každou osu se analyticky sestaví diskrétní lineární mapovací funkce:
stav `(ω, actuator moment state, I, D acceleration, previous gyro,
sensor FIFO, command FIFO)`. P/I/D koeficienty odpovídají výše uvedeným gains/dt.
Plant ZOH část je přesná integrace lineárního systému

```math
dot(omega) = -lambda omega + (Ktau/J) x
dot(x) = (u-x)/tau_motor
lambda=D/J
x[k+1] = u + (x[k]-u) exp(-dt/tau_motor)
omega[k+1] = exp(-lambda dt) omega[k]
  + (Ktau/J) [u A + (x[k]-u) B]
A = (1-exp(-lambda dt))/lambda,                  A=dt pokud lambda=0
B = (exp(-dt/tau_motor)-exp(-lambda dt))/(lambda-1/tau_motor)
```

Pro rovné lambda=1/tau je B=dt*exp(-lambda dt); tau0 použije okamžitý
aktuátor. Matice se skládá vyhodnocením mapy na bázových vektorech a NumPy
eigvals. Priming transient, clampI/output, scale/shift, epsilon anti-windup,
noise a jitter switching nejsou v této lokální LTI matici.

Spectral radius<1 znamená asymptotickou stabilitu tohoto **konkrétního
lineárního modelu**. Žádný automatický tuning gains nebyl proveden.
Počty a hranice jsou pro pevné parametry a finite sweep; nejsou robustní
zárukou pro celý kontinuální interval, nelineární/poruchový model nebo hardware.
Fixed-step póly jitter scénáře jsou pouze nominal mapa; stabilita libovolného
switching dt z nich neplyne. Robustnost vůči zvolenému jitter byla ověřena jen
jednou reprodukovatelnou8s trajektorií.

30ms kombinovaný delay dává roll radius1,019097, pitch1,010506,
yaw0,998131. Nelineární roll odezva přesto zůstane konečně omezená saturací:
max|ω|≈5,692rad/s a poslední1s oscilace≈10,750rad/s peak-to-peak.
To je selhání sledování/lokální stability, nikoli úspěch proto, že signál
nepřetekl. Linear instability a nelineární boundedness jsou rozdílné vlastnosti.

## Interpretace anti-windup, vazeb a noise

Saturace všech os `(15,−10,8)rad/s` mezi1–4s a návrat na0 vyvolává společné
škálování≈3,037s a directional I gating na všech osách. Recovery v8s
okně nemá všude nulovou chybu. Decoupled stress má odlišnou dobu common
scale≈0,585s; plná bilineární vazba při vysokých rates je významná. Není
vhodné jeho výsledek automaticky přenést na skutečný dron.

Motor ceiling0,6 je za allocator: controller o něm neví. M1 ceiling0,3
vede k trvalým chybám přibližně±0,5rad/s a integrátor se dostává kIMAX0,19,
přesto diagnostický mixer nemusí hlásit saturaci. Samostatný test všech
aktuátorů0 ukazuje, že I může růst bez integration_blocked, protože normalized
achieved není physical achieved. Závěr je omezení současného feedback modelu,
nikoli důvod změnit firmware SHADOW matematiku bez schválení.

Při sigma0,02rad/s má raw RMS přibližně0,0145/R,0,0145/P,0,0142/Y.
D filtr tlumí derivaci, ale tento test neobsahuje skutečné gyro spektrum,
notch/body resonance/aliasing. Zůstává SHADOW ONLY: H743 integrátor zatím
nikdy neřídil fyzickou UTB uzavřenou smyčku.

Sinusové gain/phase jsou uzavřený přenos r→ω, dominantní harmonická v
posledních4s. Nejde o open-loop gain margin/phase margin; ty se zde nehlásí,
protože bez identifikovaného aktuátor/gyro modelu nemají hardware význam.

## Testy a numerické kontroly

34 nových offline testů PASS bez skips. Původní30946 vektorů/29 scénářů PASS
(4 existující C++ UTBMath testy). Replay75226 kroků/24 scénářů PASS proti
stejnému C++ runneru; všechna kontrolovaná pole v původních forward error bounds,
flags přesně. Flake8 PASS. Detail: unit-confirmed.log,
original-math-regression/comparison.json a cpp-replay-final/comparison.json.


Analytické testy: konstantní torque, damping exponenciála, motor lag exponenciála,
jednotlivé motorové momenty a BF_X_REV→physical gain. Torque-free coupled model
zachovává energii i ||Jω|| v testované numerické toleranci, damping energii sníží.
Test lokální ZOH mapy proti RK4 plant kroku souhlasí na10 desetinných míst.
Convergence kontrola2 versus8 substeps při stejném controller400Hz má rozdíl
body rates<2e−7rad/s. To testuje integraci, ne nezávislou záruku časové
diskretizace controlleru; actual5ms je samostatný stress scénář.

Samostatné testy reset/config/dt historie jsou v původní matematické regresi.
Nové testy kontrolují deterministický seed/noise/jitter, priming, integrátor,
směr mixeru, invalid inertia/dt/gains, nulové gains a úplnou poruchu aktuátorů.
Model/sweep nemá možnost komunikovat s reálným hardwarem.

## Reprodukce bez instalace závislostí

Simulace a unit testy používají existující WSL `.venv`. NumPy a ReportLab jsou
už v dostupném Windows Python runtime; WSL je nemá. Nic nebylo doinstalováno.

```sh
cd /home/jirka/ardupilot_utb
OUT="tmp/utb-1hd-model-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
.venv/bin/python -B Tools/autotest/utb_closed_loop.py --output "$OUT/results"
.venv/bin/python -B Tools/autotest/utb_closed_loop_replay.py \
  --binary tmp/utb-synthetic-reference-20261012_085037/test_utb_math-committed \
  --inputs "$OUT/results/pid-replay.txt" --output "$OUT/cpp-replay"
.venv/bin/python -B Tools/autotest/utb_math_validation.py \
  --binary tmp/utb-synthetic-reference-20261012_085037/test_utb_math-committed \
  --output "$OUT/original-math-regression"
.venv/bin/python -B -m unittest discover \
  -s Tools/autotest/unittest -p test_utb_offline.py -v
```

Poslední příkaz bez UTB_AUDIT_EVIDENCE přeskočí5 archivních kontrol; při
plné ověřené session byla proměnná nastavená a žádná kontrola nebyla skipped.
AdresářOUT/results ze WSL se pro grafy/póly předá následujícím příkazům
jako absolutní Windows UNC cesta. Zobrazený adresář odpovídá skutečné této session:

```powershell
$repo = '\\wsl$\Ubuntu\home\jirka\ardupilot_utb'
$pythonPaths = @(Get-Item "$env:USERPROFILE\.cache\*\*primary-runtime\dependencies\python\python.exe")
if ($pythonPaths.Count -ne 1) { throw 'Vyberte existující Python s NumPy a ReportLab.' }
$py = $pythonPaths[0].FullName
$output = "$repo\tmp\utb-1hd-offline-20261012_162639\results"
$env:PYTHONPATH = "$repo\Tools\autotest"
& $py -B "$repo\Tools\autotest\utb_closed_loop.py" --output $output --stability-only
& $py -B "$repo\Tools\autotest\utb_closed_loop_plots.py" --directory $output
```

CLI odmítne existující directory pro hlavní simulační run, aby nepoškodil
předchozí data. Stability/plot postprocessing zapisuje definované výstupy
do již vytvořené vlastní results directory. C++ runner je archivovaný z předchozí
fáze; SHA-256 jeho binárky a replay vstupů jsou v comparison.json. Nejde o nový
SITL/hardware test. Přesné verze runtime, výsledky a manifest jsou v novémtmp.

## Co chybí pro věrohodný model současného5″ dronu

1. Změřit skutečnou hmotnost, těžiště, motorové pozice vFRD a hlavní momenty
   setrvačnosti J včetně baterie; případně produkty setrvačnosti a transformaci
   principal→body. Ověřit variaci pro baterie/užitečné zatížení.
2. Identifikovat každou konkrétní motor/ESC/vrtuli/baterii: static thrust versus
   command/RPM, reakční torque, napájení, idle/deadzone, min/max, asymetrii,
   forward/inverse křivky a hysteresi. Samostatně případ poruchy/omezení motoru.
3. Změřit aktuátor step response a delay/rise/fall závislost na pracovním
   bodu, skutečném protokolu a zatížení. Propeller data patří na samostatný
   krytovaný tahový stand, nikoli improvizovaný motor test této session.
4. Ověřit fyzickéM1–M4 ordering/spin/prop chirality a směr momentu vůči gyro
   kalibraci/orientaci. FRAME_TYPE18 v parametrech a matematická geometrie
   nejsou samotné potvrzení mechanického zapojení/reakčního koeficientu.
5. Změřit gyro sample/filtrovací transfer, noise/bias/aliasing, IMU+EKF phase
   delay, skutečné computation/scheduling delay/jitter a výpadky. INS timestamp
   age je jiná veličina než fázové zpoždění filtru.
6. Změřit damping versus rate, coupling, rámové rezonance/rotor inertia a
   interference motorů. Bez nich vysokorate stress model není kalibrovaný.
7. Definovat physical achieved/limity a později ověřit anti-windup v uzavřené
   smyčce s kalibrovanými aktuátory; současný diagnostický residual to nenahrazuje.
8. Teprve samostatně navrhnout a odsouhlasit motor authority, bezpečné takeover/
   fallback, arming/failsafe integraci a flight qualification. Nic z toho není
   součástí implementované1H-D.

## Další bezpečný krok

Nejprve dokončit provozní reprodukci download/EKF podle
[samostatného auditu](UTB_LOG_DOWNLOAD_EKF_AUDIT_CS.md), bez souběžného syntetického
experimentu. Host postup a nové read-only časové/health evidence mají přednost
před firmware změnou. Paralelně lze offline dosadit změřené geometrické/J údaje
do Model a znovu provést sweep/testy. Tahové/zpožďovací měření a jakýkoli další
HW experiment vyžadují samostatně schválený postup.

Model poskytl ověřitelný základ a odhalil delay/actuator feedback limity.
Nezakládá schválení aktivního UTB_ACRO ani FÁZE2. Commit/push/release neprovedeny.
