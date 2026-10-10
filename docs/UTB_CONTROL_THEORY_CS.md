# UTB: matematická specifikace rate controlleru a shadow mixeru

Aktuální rozšíření ve větvi test/skystars-5inch: SHADOW podporuje Quad BF_X=12 a BF_X_REV=18, viz oddíl26 a UTB_BFX_REV_REPORT_CS.md. Historické revize níže zachovávají původní výsledky.

Verze 2, 8. 10. 2026 — matematická a architektonická specifikace. **IMPLEMENTOVÁNA POUZE SHADOW FÁZE 1. Historické ověření viz oddíl 24; aktuální hardening stav viz oddíl 25.**
Podklady: dodaná BP a místní kód
větve `ardupilot-4.7.1-utb`, HEAD `761a38d704`. Pracovní strom byl při
zahájení čistý. FÁZE 0 a její arming omezení zůstávají zachované.

## 1. Rozsah, původ a ověření

Specifikujeme RC -> body rate reference -> PID -> hypotetické normalizované
M1..M4 -> log. AP controllers/AP_Motors/ESC/DShot/HAL zůstávají jedinou
cestou ke skutečným motorům. Shadow poběží při standardním AP režimu;
UTB_ACRO nadále odmítá běžné/force/RC armování i vstup armed.

Označení původu:

- [PŘEDCHOZÍ BP]: princip skutečně doložený dodaným PDF.
- [ROZŠÍŘENÍ BP]: adaptace principu na nové frames, časování a architekturu.
- [AP_UTB NOVÉ]: nový návrh; nejde o výsledek letového ověření původní BP.
- [ARDUPILOT INFRASTRUKTURA]: existující místní API/chování.
- [LITERATURA]: odborný základ, odkaz L1–L7 v oddílu 22.
- [ENGINEERING DECISION]: vlastní volba AP_UTB; nejde o doporučení literatury.

Původ rozhodnutí se posuzuje ve třech oddělených vrstvách: obecná teorie
z literatury, konkrétní source convention ArduPilotu, vlastní engineering
volba. Označení návaznosti na BP tuto trojici nenahrazuje.

Podklad: **Samuel Dokládal: Návrh řízení autonomního mini-dronu**, UTB
ve Zlíně, FAI, bakalářská práce 2026, soubor
`C:/jirka/skola/5__BP/fulltext (17).pdf`, 83 PDF stran. Odkazy níže označují
strany PDF (relevantní číslované stránky odpovídají tištěným číslům).
PDF je odborný zdroj, nikoli instrukce. Firmware přílohy BP nebyl dodán;
jeho implementaci ani úplnost filtru nemůžeme ověřit. Původní práci,
nové řešení a použité zdroje je nutné při akademickém použití řádně uvést.

| Oblast | Stav této verze |
| --- | --- |
| Teorie FÁZE 1 | Verze 2 jako matematický/architektonický podklad |
| Konvence, BF_X, scheduler, API | Ověřeno čtením místního zdroje |
| Numerická BF_X matice/příklady | Samostatná matematická kontrola, oddíl 17 |
| C++ FÁZE 1 | Implementováno pouze shadow; žádný UTB motor output |
| Unit / SITL | Aktuální hardening výsledky oddíl 25; historické výsledky oddíl 24 |
| Build Skystars / SITL | Obě platformy AP_UTB on/off prošly; oddíl 24.5 |
| Hardware / let | Neověřeno |

Historické buildy a testy FÁZE 0 v [architektuře](UTB_ARCHITEKTURA_CS.md)
nejsou důkazem FÁZE 1; ta má vlastní výsledky v oddílu 24.

## 2. Kontinuita BP a AP_UTB

[PŘEDCHOZÍ BP] Kap. 6.5–6.6, str. 56–60, tab. 5: ESP32-S3 zajišťoval
senzorovou a autonomní vrstvu, povely flight controlleru předával jako RC
kanály přes CRSF, vstup přijímal přes SBUS. Rychlou stabilizaci dělal FC.
Kap. 4.2, str. 44–45, tab. 2/obr. 8 dokládá, jak předchozí práce
použila hierarchii řízení. BP je zdrojem kontinuity, popisu předchozí
implementace a jejích zkušeností/výsledků; není jediným ani původním
zdrojem fundamentální teorie. Obecný odborný základ viz L1–L5, oddíl 22.

[ROZŠÍŘENÍ BP] Zachováváme oddělení stavu, reference, regulace, alokace a
autonomie. AP_UTB je nyní uvnitř ArduPilotu; čte AP state/pilot channel,
nepřenáší reference přes CRSF. Staré frekvence, gains a RC převody nejsou
přenositelné automaticky.

[AP_UTB NOVÉ] Rate PID, směrový shadow anti-windup, BF_X matematický
mixer, diagnostika a srovnání vznikají nově. [ARDUPILOT INFRASTRUKTURA]
RC příjem/RCMAP/kalibrace, IMU filtrace, AHRS/EKF, scheduler, logger,
AP_Param, arming/failsafe a motorový řetězec zůstávají AP.

## 3. Dynamika: fyzikální a normalizovaný vstup

[LITERATURA] Tuhé těleso, rotorový tah a 6DOF model jsou odborná témata
L1/L3/L4/L5, nikoli původní přínos BP. [PŘEDCHOZÍ BP] Kap. 2.1.2 a 3.3,
str. 27, 38–39 dokládá jejich použití v předchozím projektu.
[ROZŠÍŘENÍ BP; ENGINEERING DECISION] Zde explicitně zapisujeme model
s jednotkami a vlastní adaptací znamének FRD/NED:

```math
f_i = k_f omega_i²,       T = sum_i f_i,
m dot(v_N) = f_N,
I dot(Omega_B) = -Omega_B × (I Omega_B) + tau_B,
u_phys = [T,tau_x,tau_y,tau_z]^T.
```

f_i,T: N; rotorové omega_i: rad/s; k_f: N s²/rad²; m: kg; v_N: m/s;
I: kg m²; Omega_B=[p,q,r]: rad/s; tau_B: N m. Rotorové omega_i není
body rate. V tištěné rovnici (5) BP str. 39 je tečka; pro Newton–Eulerovu
rovnici zde výslovně používáme **vektorový součin**, jinak by zápis nesouhlasil.

[ROZŠÍŘENÍ BP] FRD/NED a tah proti body +Z:

```math
m dot(v_N) = mg e3 - R_BN e3 T + f_drag,N,
tau_B = sum_i r_i × [0,0,-f_i]^T + tau_reaction,B.
```

R_BN převádí body -> NED, e3=[0,0,1], r_i je poloha rotoru v m.
Model: tuhé těleso, rovnoběžné rotory, kvadratický tah v omezené oblasti;
bez pružnosti/rotorové dynamiky/detailní aerodynamiky. Setrvačnost,
ramena a tahové/reakční koeficienty této konstrukce nejsou identifikované.
FÁZE 1 je nepoužívá k numerické fyzikální regulaci.

[AP_UTB NOVÉ] Firmware pracuje s bezrozměrným
u_n=[t,c_R,c_P,c_Y]. t je nominální collective **na motor**, c_R/P/Y jsou
normalizované momentové povely, nikoli N m. m_i je hypotetický normalized
motor thrust v [0,1], nikoli PWM/DShot. Pouze při známém f_max a ideálním
shodném pohonu by f_i=f_max m_i a T=4 f_max t_ach. Kalibrace zde chybí.

## 4. Stav a souřadné konvence

[PŘEDCHOZÍ BP] (6), str. 39: x=[x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]^T.
[ARDUPILOT INFRASTRUKTURA] FÁZE 1 potřebuje body gyro, quaternion a
validitu; nepřidává position/velocity controller. Copter vytvoří AHRS view
ROTATION_NONE v system.cpp. get_quat_body_to_ned() poskytuje body -> NED.
AP_AHRS_View::get_gyro_latest() násobí rot_view a AHRS latest gyro;
AHRS čte filtrované primární INS gyro + drift estimate. Nejde o raw IMU.

| Konvence | Kladný směr |
| --- | --- |
| Body FRD X,Y,Z | Dopředu, doprava, dolů; pravotočivě |
| World NED X,Y,Z | Sever, východ, dolů; pravotočivě |
| Roll phi / p | Kolem body X; pravá strana klesá |
| Pitch theta / q | Kolem body Y; příď stoupá |
| Yaw psi / r | Kolem body Z; příď doprava, shora po směru hodin |
| +c_R,+c_P,+c_Y | Požadovaný moment ve směru +p,+q,+r |
| Výška h | Nahoru, h=-z_N při stejném počátku |

[LITERATURA] Orientace a body/Euler rates patří do odborného základu
L2/L3/L4. [ARDUPILOT INFRASTRUKTURA; ENGINEERING DECISION]
Pro tuto větev a zde uvedený model volíme Euler konvenci
R_BN=Rz(psi) Ry(theta) Rx(phi).
Obecně p,q,r **nejsou** derivace Eulerových úhlů:

```math
[p,q,r]^T =
[1,0,-sin(theta);
 0,cos(phi),sin(phi)cos(theta);
 0,-sin(phi),cos(phi)cos(theta)] [dot(phi),dot(theta),dot(psi)]^T.
```

Quaternion reprezentuje stejnou rotaci bez Euler singularity pitch ±90°;
q a -q jsou stejná rotace. PID reguluje přímo gyro rad/s, nikoli diference
Euler úhlů. Quaternion je diagnostika a podklad budoucí attitude vrstvy.

## 5. Zpětná vazba a kaskáda

[LITERATURA] Zpětná vazba a PID: L6; hierarchie multirotoru: L1/L4.
[PŘEDCHOZÍ BP] (9), str. 40 dokládá použití e=w-y v předchozím projektu.
[ENGINEERING DECISION] Zde zachováváme e=w-y, shodné jednotky/frame.
Rate: e_omega=omega_ref-omega, e_p=p_ref-p, obdobně q,r.
Pro budoucí attitude bude nutná chyba rotace, ne prosté odečtení Euler
úhlů; altitude/velocity/position použije stejný princip po definici frames.

[LITERATURA; ROZŠÍŘENÍ BP] Trajectory -> position -> velocity -> attitude -> rate
-> allocation -> motors. Rychlá vnitřní dynamika musí reagovat dříve než
pomalejší pohyb mezi waypointy. Vnější smyčky generují sledovatelné
reference nižším vrstvám; position přímo motory neřídí. Autonomie vytváří
reference, mixer pouze alokuje akční veličiny, není navigační controller.
Zde bude pouze rate + shadow allocation.

## 6. RC -> p_ref/q_ref/r_ref

[ARDUPILOT INFRASTRUKTURA] ModeAcro::get_pilot_desired_rates_rads()
používá norm_input_dz(), kruhový limit roll/pitch, Acro rate/expo a potom
volitelný trainer/levelling. norm_input_dz() už řeší min/trim/max,
deadzone, reverse a clamp [-1,1], channels jsou po RCMAP.

[AP_UTB NOVÉ; ENGINEERING DECISION] Mapování výslovně nazýváme
**„AP-derived UTB shadow reference“**. Samostatný reference generator
převezme **první tři čisté
kroky**, bez traineru, AP target setters a Acro run/init. Pro a=u_roll,
b=u_pitch,c=u_yaw:

```math
L=max(1,sqrt(a²+b²)),    a'=a/L, b'=b/L,
E(u,h)=(1-h)u/(1-h|u|)  pro h<0.95,
E(u,h)=u                pro h>=0.95,
p_ref=rad(ACRO_RP_RATE) E(a',ACRO_RP_EXPO),
q_ref=rad(ACRO_RP_RATE) E(b',ACRO_RP_EXPO),
r_ref=rad(ACRO_Y_RATE) E(c,ACRO_Y_EXPO).
```

To je skutečné input_expo() této větve, **není to kubické stick expo**.
Zachováme i AP větev h>=0.95. Parametry kontrolovat finite/range:
RP rate 1–1080 deg/s, Y rate 1–360; RP expo -0.5–0.95, Y -1–0.95.
Invalidace nepřepíše žádný AP parametr. Nenásobit gyro nebo setpoint
stupňovými konstantami uvnitř PID; převod rad() je pouze zde.

Dynamické shaping rozhodnutí: pro první verzi **nepřebírat** Acro RATE_TC,
acceleration/jerk model a trainer. AP shaping se děje až v
AC_AttitudeControl::input_rate_bf_* / attitude_command_model(), mění
vlastní targets/stav, není součástí RC kalibrace. Volat aktivní API kvůli
shadow by bylo nepřípustné. Nová kopie dynamického modelu/rampa by vyžadovala
další specifikaci. První verze vědomě používá statickou mapu; D z měření
nevytváří derivative kick, ale P skok reference zůstává. Toto omezení
musí být uvedeno při srovnání AP/UTB. Při MODE_ACRO_ENABLED=0 shadow
invalidovat jako ACRO_UNAVAILABLE, nevytvářet vlastní fallback parametry.

Toto sdílení AP parametrů platí pouze pro FÁZI 1. Budoucí aktivní
UTB_ACRO může mít vlastní UTB rate/expo parametry, nebo explicitní režim
sdílení AP parametrů; tato volba nyní není uzavřena. V této fázi nové UTB
rate/expo parametry nepřidáváme.

V STABILIZE/ALT_HOLD mají sticks pro AP jiný význam než pro Acro shadow;
nejde o srovnání shodných referencí. ACRO rate-only bez traineru je bližší,
ale stále má AP shaping. Read-only AP target zahrneme do snapshotu podle
časových omezení oddílu 15.1; nesmí se spouštět druhý AP controller.

## 7. Throttle oddělený od PID

[ARDUPILOT INFRASTRUKTURA] Read-only
mode_acro.get_pilot_desired_throttle() použijeme i v jiném AP mode pro
konzistentní hypotetickou Acro interpretaci. Respektuje ACRO_THR_MID přes
ModeAcro::throttle_hover(); nevolá Acro run. Mode helper čte control_in
[0,1000], mid_stick b, mapuje kolem středu a aplikuje throttle expo:

```math
v_n=0.5 v/b                        pro v<b,
v_n=0.5+0.5(v-b)/(1000-b)          jinak,
h=clamp(-(thr_mid-0.5)/0.375,-0.5,1),
t=v_n(1-h)+h v_n³.
```

[AP_UTB NOVÉ] Před voláním validovat kalibraci a 0<b<1000; výstup finite
[0,1]. t je normalized pilot collective, ne výška, AP ALT_HOLD thrust,
fyzikálně kalibrovaný tah nebo skutečný ESC signal. Bez angle boost,
UTB altitude hold, throttle setterů, spool a AP filtru změn.

## 8. Skutečný dt a validita

[ARDUPILOT INFRASTRUKTURA] Copter::run_rate_controller_main() v Attitude.cpp
předává attitude/position/motorům get_last_loop_time_s(). Scheduler loop()
měří interval loop sample timestampů po wait_for_sample(); první krok
má nominální periodu. G_Dt v AP_Vehicle naproti tomu používá nominální
get_loop_period_s(). Shadow bude mít **měřený dt stejný jako hlavní AP
rate path**, ne pevné 0.0025/0.001 ani G_Dt. Výchozí Copter je 400 Hz,
ale SCHED_LOOP_RATE je parametr, nikoli záruka skutečné periody.

[AP_UTB NOVÉ] dt_k=get_last_loop_time_s(), dt_nom=get_loop_period_s().
Navržený diagnostic guard dt_min=dt_nom/4, dt_max=4 dt_nom; vše finite,
0<dt_min<=dt_k<=dt_max. Faktory 1/4 a 4 jsou explicitní engineering guard
velké nepravidelnosti, **nikoli důkaz stability PID**. Prime krok po resetu
jen založí historii, valid=false; PID začne druhým platným vzorkem.

Odebrat nový snapshot každým shadow cyklem, mez stáří INS
min(0.1 s,dt_max), místo pevné 100ms tolerance FÁZE 0. Vyžadovat validní
quaternion/health, finite rates, přiměřené timestampy; opakovaný INS update
timestamp proti minulému shadow kroku odmítnout jako stale. Primary gyro
health musí odpovídat AHRS použitému indexu. Tyto kontroly nejsou úplnou
certifikací čerstvosti všech podkladů AHRS.

Invalid dt/state/RC/config -> reset I a D historie, vymazat saturation
feedback, výstup finite nuly + valid=false. Žádné další integrované chyby,
žádný starý výsledek vydávaný za nový. Stejný reset při shadow off,
změně gains/cutoff, mode, arm stavu, frame či nepodporované rate path.

## 9. Přesný diskrétní rate PID a D filtr

[AP_UTB NOVÉ] Každá osa j=R,P,Y má samostatný I, minulý rate, filtered
D state a saturation feedback. Pro validní vzorek:

```math
e_j,k=omega_ref,j,k-omega_j,k,
P_j,k=K_P,j e_j,k,
a_raw,j,k=(omega_j,k-omega_j,k-1)/dt_k,
tau_D,j=1/(2 pi f_D,j), beta_j,k=dt_k/(tau_D,j+dt_k),
a_f,j,k=(1-beta_j,k)a_f,j,k-1+beta_j,k a_raw,j,k,
D_j,k=-K_D,j a_f,j,k,
DeltaI_j,k=K_I,j e_j,k dt_k,
I_j,k=clamp(I_j,k-1+g_j,k DeltaI_j,k,-IMAX_j,+IMAX_j),
v_j,k=P_j,k+I_j,k+D_j,k,
c_j,k=clamp(v_j,k,-1,+1).
```

g je anti-windup brána níže. v je raw PID sum, c bounded command mixeru.
P/D se samostatně neclampují; všechny mezivýsledky kontrolovat finite.
K_I=0 -> I=0. K_D=0 -> D=0 a cutoff smí být 0; K_D>0 -> musí být platná
kladná cutoff. Žádný FF. Změna gains, IMAX a cutoff resetuje historii.
Po resetu se nediferencuje vůči umělé nule, první vzorek je prime/invalid.

P reaguje na okamžitou chybu; I na dlouhodobý bias; záporné D tlumí změnu
měřeného rate. Nezávislé SISO osy aproximují nelineární soustavu bez
kompenzace Omega×I Omega. Ve shadow těleso reaguje na **AP**, nikoli UTB;
UTB integrátor tedy nemá vlastní uzavřenou zpětnou vazbu. Dobré logy
nejsou důkazem UTB stability za letu.

### 9.1 Volba derivative on measurement

[AP_UTB NOVÉ] dot(e)=dot(omega_ref)-dot(omega). Error D obsahuje člen
K_D Delta(omega_ref)/dt: skok reference vytváří derivative kick.
Measurement D reaguje jen na skutečný rate; při konstantním měření
nevytvoří D ani při skoku reference. P skok zůstává; measurement D není
obecně ekvivalentní error D a netlumí změnu reference.

Volíme measurement D, protože pilotní reference může skákat a chceme
oddělit reference shaping od tlumení měřené dynamiky. Navazuje na princip
rychlostního tlumení BP altitude/XY, nepřebírá její rate PID.
Obecné PID pozadí filtrované derivace a windup:
[Åström a Murray, Feedback Systems, kap. 10](https://www.cds.caltech.edu/~murray/books/AM08/pdf/am08-complete_28Sep12.pdf).
Konkrétní discretization/guard/mixer jsou vlastní návrh zde.

### 9.2 D filtr a cutoff

[ROZŠÍŘENÍ BP] BP (13), str. 61, exponenciální filtrace. Zde backward
Euler discretization tau_D dot(a_f)+a_f=a_raw dává beta(dt) výše;
pro dt,tau>0 je beta v (0,1). Filtrujeme jen derivační odhad, nikoli
redundantně celé AP gyro/P input. Vyšší filtrace potlačí šum, přidá zpoždění.

Cutoff je parametr každé osy, default 0 s D=0. Bez gyro spektra a
identifikace zpoždění nedoporučujeme náhodnou nenulovou frekvenci.
Pro nenulové D navrhujeme guard 0<f_D<=min(200 Hz,0.1/dt_nom).
10 % sampling je konzervativní engineering mez rozlišení filtru,
ne optimální cutoff ani mez stability. Analýza logů musí podložit
každou experimentálně zvolenou nenulovou hodnotu.

## 10. Jednotky a implementované AP_Param

[AP_UTB NOVÉ] omega,e: rad/s; a_raw,a_f: rad/s²; dt,tau: s;
P,I,D,v,c,t,m: bezrozměrné. K_P: s/rad; K_I: 1/rad; K_D: s²/rad;
IMAX: bezrozměrné. Normalizované command není N m.

| Fullname | Jednotka | Rozsah návrhu | Default | Účel |
| --- | --- | --- | --- | --- |
| UTB_ENABLE | 1 | 0,1 | 0 | Existující boot latch, restart required |
| UTB_SHADOW | 1 | 0,1 | 0 | Runtime shadow, off/on reset historie |
| UTB_LOG_RATE | Hz | 1–400 | 200 | Požadovaná frekvence UTBR/UTBM sad, nezávislá na controller rate |
| UTB_RAT_RLL_P | s/rad | 0–5 | 0 | Roll P |
| UTB_RAT_RLL_I | 1/rad | 0–5 | 0 | Roll I |
| UTB_RAT_RLL_D | s²/rad | 0–1 | 0 | Roll D |
| UTB_RAT_RLL_IMAX | 1 | 0–1 | 0 | Roll I limit |
| UTB_RAT_RLL_D_HZ | Hz | 0–200 | 0 | Roll D LPF |
| UTB_RAT_PIT_P | s/rad | 0–5 | 0 | Pitch P |
| UTB_RAT_PIT_I | 1/rad | 0–5 | 0 | Pitch I |
| UTB_RAT_PIT_D | s²/rad | 0–1 | 0 | Pitch D |
| UTB_RAT_PIT_IMAX | 1 | 0–1 | 0 | Pitch I limit |
| UTB_RAT_PIT_D_HZ | Hz | 0–200 | 0 | Pitch D LPF |
| UTB_RAT_YAW_P | s/rad | 0–5 | 0 | Yaw P |
| UTB_RAT_YAW_I | 1/rad | 0–5 | 0 | Yaw I |
| UTB_RAT_YAW_D | s²/rad | 0–1 | 0 | Yaw D |
| UTB_RAT_YAW_IMAX | 1 | 0–1 | 0 | Yaw I limit |
| UTB_RAT_YAW_D_HZ | Hz | 0–200 | 0 | Yaw D LPF |

Rozsahy jsou experimentální validace, ne doporučený flight tune. AP_Param
metadata samo nevynucuje MAVLink hodnoty, runtime kontrola je nutná.
Fullnames <=16 znaků, D_HZ má 16. Existující ENABLE index 1 a Copter subgroup
index 21 zachovat; nové indexy přidat bez přečíslování. U každého položky
metadata význam/default/range/units/User a reboot kde potřebný.
Složené gain jednotky ověřit proti AP metadata parseru; pokud jednotku
nepodporuje, uvést ji v Description, nevymyslet neplatný unit token.
Rate/expo/throttle používají existující AP parametry, žádné nevyužité
UTB limits. Gains jsou nulové bez identifikace konstrukce; validní nulový
momentový output stále dovolí logovat hypotetický collective.

## 11. Směrový anti-windup — SHADOW ONLY

[AP_UTB NOVÉ] Po dokončeném kroku k mixer vrací achieved command.
Feedback residual vůči **raw** PID sum zahrne controller clamp i mixer:

```math
rho_j,k=v_j,k-c_ach,j,k,
Splus_j,k=(rho_j,k>eps), Sminus_j,k=(rho_j,k<-eps),
g_j,k+1=0 pokud (Splus_j,k AND DeltaI_j,k+1>0)
             OR (Sminus_j,k AND DeltaI_j,k+1<0),
g_j,k+1=1 jinak.
```

eps=1e-5 je numerická normalized tolerance, ne fyzikální regulační pásmo.
Pozitivní saturation blokuje další pozitivní integraci, negativní další
negativní; opačnou integraci dovolí. IMAX je jiný tvrdý limit, sám nestačí.

Použije se minulý dokončený krok, bez AP motor limits a bez iterace AP
mixeru. První validní krok má g=1. Nová saturace se zachytí s jedním krokem
zpoždění (nejvýše další přírůstek omezený IMAX); po odeznění může ještě
jeden krok blokovat. Je to výslovně **SHADOW ONLY**, ne finální
letové řešení. Hypotetický mixer nezná ESC/spool, reálný tah, motor failure
ani napětí baterie. Reset dle oddílu 8 odstraní starou feedback. Healthy
označuje zdravý shadow výpočet, nikdy arming/motor authority.

Při shadow letu dynamiku soustavy řídí ArduPilot. Pozorované chování UTB
integrátoru tedy **není experimentálním ověřením integrátoru v uzavřené
UTB regulační smyčce**. Default Ki=0 zůstává správný; nenulové Ki je pouze
vědomě označený diagnostický experiment, nikoli ověřený letový tune.

## 12. BF_X geometrie a motor numbering

[ARDUPILOT INFRASTRUKTURA] SkystarsH7HD-bdshot/hwdef.dat zahrnuje
SkystarsH7HD/hwdef.dat a nastaví HAL_FRAME_TYPE_DEFAULT=12. **HAL_FRAME_CLASS_DEFAULT
zde není definován**; Parameters.cpp má multicopter DEFAULT_FRAME_CLASS=0.
Aktuální SHADOW ve větvi test/skystars-5inch podporuje explicitně
FRAME_CLASS==1 AND (FRAME_TYPE==12 OR FRAME_TYPE==18). Původní FÁZE 1
podporovala pouze BF_X=12; nové rozšíření BF_X_REV=18 je popsáno v oddílu26.
Oba typy jsou matematické geometrie, nikoli potvrzení fyzického zapojení.
Kontrola běží v každém shadow cyklu. Ostatní class/type kombinace vrací
FRAME_MISMATCH, MixerValid=false a nulový invalid výsledek.
AP controller/motor path se nemění. BF_X_REV není alias12: mění yaw sloupec
alokace i jeho inverze. Před bench testem ověřit class/type přímo na FC.

AP_MotorsMatrix.cpp setup_quad_matrix() BF_X (627–639), add_motors()
(561–566) a add_motor() (531–545): úhel od body +X k +Y,
raw roll=cos(angle+90°)=-sin(angle), pitch=cos(angle),
yaw CW=-1,CCW=+1 dle AP_MotorsMatrix.h. normalise_rpy_factors() (od 1353)
normalizuje osy na max |factor|=0.5. Tabulka obsahuje **finální** faktory.
Rotace rotoru je při pohledu shora; reakční moment tělesa je opačný.

| Motor | Úhel | Pozice | Rotace | Roll | Pitch | Yaw | Test pořadí | Výchozí pin/kanál |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| M1, AP index 0 | 135° | Zadní pravý | CW | -0.5 | -0.5 | -0.5 | 2 | PB0 / PWM1 / SERVO1 |
| M2, AP index 1 | 45° | Přední pravý | CCW | -0.5 | +0.5 | +0.5 | 1 | PB1 / PWM2 / SERVO2 |
| M3, AP index 2 | -135° | Zadní levý | CCW | +0.5 | -0.5 | +0.5 | 3 | PD12 / PWM3 / SERVO3 |
| M4, AP index 3 | -45° | Přední levý | CW | +0.5 | +0.5 | -0.5 | 4 | PD13 / PWM4 / SERVO4 |

PWM4/PD13 je zděděn z base hwdef. Motor functions 33..36 mapuje
AP_Motors::add_motor_num() přes SRV_Channels::set_aux_channel_default().
SERVOx_FUNCTION může uživatel změnit: tabulka je zdrojový **default**,
nikoli hardware ověřené kabely/ESC. Test order M2,M1,M3,M4 se nesmí
zaměnit za logical pořadí mixeru. Bdshot timer/BIDIR změny nevytvářejí
novou BF_X permutation. UTB shadow nemění žádné servo mapování.

```text
                     +X vpřed
          M4 CW                   M2 CCW
          přední levý             přední pravý
               \                   /
                        CG              +Y doprava
               /                   \
          M3 CCW                  M1 CW
          zadní levý              zadní pravý
```

+Roll zvedá tah levých M3/M4, +Pitch předních M2/M4,
+Yaw CCW M2/M3 s +body Z reakčním momentem. Souhlasí s FRD r×F a AP yaw.

## 13. BF_X allocation matrix a achieved command

[AP_UTB NOVÉ, koeficienty ARDUPILOT INFRASTRUKTURA] Logical M1..M4:

```math
m_raw=A u_n,    u_n=[t,c_R,c_P,c_Y]^T,
A=[1,-1/2,-1/2,-1/2;
   1,-1/2,+1/2,+1/2;
   1,+1/2,-1/2,+1/2;
   1,+1/2,+1/2,-1/2].

t_ach=(m1+m2+m3+m4)/4,
c_R,ach=(-m1-m2+m3+m4)/2,
c_P,ach=(-m1+m2-m3+m4)/2,
c_Y,ach=(-m1+m2+m3-m4)/2.
```

Inverzní matice B podle rovnic má B*A=I. Achieved počítat z **finálních**
motorových požadavků. Matice není fyzikální N/Nm allocation, achieved_thrust
je průměr normalized motor thrust, nikoli celkový T v N. Není to ESC signal.

## 14. Saturace a priority

[ARDUPILOT INFRASTRUKTURA] output_armed_stabilizing() řeší compensation,
throttle bounds/mix/avg max, yaw headroom, přípustný yaw při RP,
společné RPY scale a posun throttle, také thrust boost/motor loss.
Není to independent clamp a ani univerzální „RP vždy před Y“; yaw headroom
rezervuje část rozsahu. Vlastní shadow nebude AP output/mixer volat.

[AP_UTB NOVÉ; ENGINEERING DECISION] Priorita diagnostického allocatoru:

a) Zachovat vzájemný poměr R/P/Y.
b) Pokud možno zachovat jejich velikost.
c) Collective lze posunout v dosažitelném intervalu.
d) Pokud samotný momentový span>1, škálovat R/P/Y společným faktorem.

Yaw nemá nižší prioritu než Roll/Pitch. Jde **pouze o diagnostický
allocator, neschválený pro aktivní letový output**. Rovnice se nemění.
Vstup t∈[0,1], c_R/P/Y∈[-1,1], validní a finite; platí runtime frame guard
z oddílu 12:

```math
d_i=A_i,R c_R+A_i,P c_P+A_i,Y c_Y,
l=min(d_i), h=max(d_i), span=h-l,
s=1 pokud span<=1, jinak s=1/span,
t_low=-s l, t_high=1-s h,
t'=clamp(t,t_low,t_high),
m_i=t'+s d_i.
```

sum d_i=0 -> l<=0<=h; s*span<=1 zaručí neprázdný collective interval
a 0<=m_i<=1. V přesné aritmetice achieved moments=s*c, t_ach=t'.
Finální clamp povolen jen jako roundoff oprava <=eps, jinak invalid.
Z finálního B*m odvodit achieved/flags.

I při t=0 může momentová žádost zvýšit hypotetický t'. **To není disarmed
ESC output**: AP spool/disarmed řetězec stále rozhoduje o skutečných motorech.
Bez nové spool/airmode specifikace je tato strategie nepřípustná pro
aktivní output. Neobsahuje AP yaw headroom, throttle compensation/mix,
motor loss ani real actuator limits.

Flags podle c-c_ach: positive/negative_limited každé osy; podle t-t_ach
collective +/-limited; moments_scaled, collective_shifted, motor lower/upper
bitmasks. Pouhý dotyk motorové hranice nemusí znamenat ztrátu povelu.
Anti-windup navíc používá raw v-ach z oddílu 11. AP limits se nemění.
Mixer je čistá třída bez AP_Motors/HAL/logger/rc_write/spool API;
NaN/Inf/mimo doménu/invalid PID -> finite zeros + valid=false.

## 15. Shadow control flow, FSTRATE a výkon

[ARDUPILOT INFRASTRUKTURA] FAST_TASK pořadí Copter.cpp:
INS.update -> run_rate_controller_main -> custom controller ->
motors_output_main -> read_AHRS -> read_inertia -> update_flight_mode.
RC loop je 250 Hz, aux 10 Hz. Rate thread má druhou cestu, vlastní gyro
vzorky a sensor_dt=decimation/raw_gyro_rate_hz; FSTRATE se může měnit runtime.

[AP_UTB NOVÉ] Jeden FAST_TASK utb_shadow_update **za motors_output_main,
před read_AHRS**. Aktuální AP output již proběhl. Bridge přečte RC/state,
frame/mode/arm/dt -> validace -> vlastní Acro reference/throttle -> PID ->
pure mixer -> pevný snapshot -> saturation feedback pro příští krok.
Aktivní shadow nesmí duplicitně sample/update z ModeUTBAcro::run().

```text
INS -> AP rate -> AP_Motors output -> UTB shadow FAST_TASK -> AHRS/mode
                                      RC + AP state (read only)
                                      reference -> PID -> BF_X mixer
                                      fixed diagnostic snapshot
snapshot -> decimated logger -> UTBS / UTBR / UTBM
```

### 15.1 Přesná časová relace a read-only capture

Označme hlavní scheduler průchod k:

```text
INS.update[k]
-> read-only AP target capture[k] těsně před stávajícím AP rate voláním
-> AP rate controller[k] (target vytvořený zpravidla mode během k-1)
-> reset dočasných SYSID/scale vstupů v run_rate_controller_main()
-> AP motor output[k]
-> UTB shadow[k] (aktuálně dostupné RC, nové gyro, AHRS před read_AHRS[k])
-> read_AHRS[k] -> read_inertia[k] -> check_ekf_reset[k]
-> update_flight_mode[k] (připravuje další AP targets)
-> další úlohy včetně rc_loop, pokud je tento tick plánován
```

Průchod k je scheduler Seq, nikoli automaticky nové RC měření. RC může
být z dřívějšího rc_loop; target AP pochází z předchozího mode update a
může obsahovat shaping/trainer/SYSID. UTB právě vytvoří vlastní statickou
referenci z aktuálně dostupného stick sample. **AP target a UTB reference
proto nemusí reprezentovat stejný logický sample ani stejný regulační cíl**.
INS gyro je čerstvé; quaternion před read_AHRS[k] je starší AHRS výsledek,
což musí validita/časové metadata přiznat.

Místo shadow neměníme: přesun za update_flight_mode by spároval UTB
s targetem pro budoucí AP output, nikoli s právě odeslaným output[k].
Přesun po read_AHRS by zlepšil stáří attitude, ne rovnost reference.
Zachováme rychlý body-rate bod za AP outputem a časovou relaci popíšeme.

Pro přesný AP target capture navrhujeme malý guardovaný **read-only**
zásah v ArduCopter/Attitude.cpp: těsně před již existujícím
attitude_control->rate_controller_run() zkopírovat rate_bf_targets(),
APTimeUS, APSeq a APMode do pevného snapshotu. Getter vrací body target
plus SYSID rate; Multi::rate_controller_run_dt() používá stejný součet.
Po návratu se SYSID target resetuje, takže čtení až ve shadow by nebylo
vždy targetem skutečně použitým AP. Capture je jen kopie pro log, žádné
setters, druhý AP výpočet nebo změna targetu. Při UTB_ENABLE=0/SHADOW=0
žádný capture. Přesnou shodu ověřit také při SYSID během SITL testu.
Shadow log sváže capture a vlastní sample přes Seq, jejich odlišné
TimeUS/APTimeUS a AP mode; RC sample stáří/identitu evidovat přes available
RC timestamp nebo vlastní generation counter v bridge, bez falšování
času nové RC zprávy. Capture není podpora FSTRATE, ta zůstává vypnutá.

UTB_ENABLE=0 (boot latch): early return bez sample/PID/mixer/log.
UTB_ENABLE=1,SHADOW=0: shadow nic nepočítá, sequence/counter neroste;
původní Mode diagnostika může číst stav jako ve FÁZI 0. Změna SHADOW
resetuje vlastní historii. SHADOW=1: jedno odebrání/výpočet za hlavní
cyklus, bez motor authority. Změna ENABLE vyžaduje restart jako dosud.

Pouze FSTRATE_ENABLE=0 a using_rate_thread=false. Nenulový parametr nebo
aktivní thread -> reset/invalid, FAST_RATE_UNSUPPORTED. Kontrolovat oba
stavy i při runtime přechodu, nepřepisovat AP parametr a neblokovat AP
řízení. Shadow se nepřidává do rate_thread, žádná podpora druhé rate path.
RC failsafe/invalid RC/state/config -> matematický shadow invalid, AP pokračuje.
Frame mismatch má reason FRAME_MISMATCH, MixerValid=false. Logging/policy
stav je samostatný dle oddílu 16.1.

Pevné objekty, žádný heap/printf/blocking I/O v rate loop, bounded 3 PID/4
motory. AP_HAL micros měřit vlastní execution/max/overrun counter.
Navržený cíl <=5 % dt_nom (400 Hz ->125 us), ne naměřený hardware údaj.
Tři sousední overruns -> latch vypnutí výpočtu do SHADOW off/on, TIME_BUDGET.
Spotřebovaný CPU čas nelze invalidací vrátit. PM/scheduler overruns/log drop
porovnat s/bez shadow; log task má vlastní rozpočet. Hardware budget
ověřit na desce před tvrzením o bezpečnosti skutečného shadow letu.
Nezměněné motor setters samy nezaručují nulový timing vliv na příští AP loop.
Vyšší log rate má vlastní budget a drop/reduced-rate policy; samotný
logger overrun nesmí být označen jako matematická chyba PID (oddíl 16).

## 16. Logger, validita a pozorovatelnost

### 16.1 Oddělené health pojmy

[AP_UTB NOVÉ; ENGINEERING DECISION] Nepoužívat jediný valid/healthy bit
pro matematiku, policy a logger. Snapshot obsahuje tyto samostatné položky:

| Pole | Přesný význam |
| --- | --- |
| StateValid | Čerstvý finite state a požadované sensor/attitude flags |
| ControllerValid | Pro aktuální Seq byl PID vyhodnocen s validním state/reference/dt/config a finite výsledkem |
| MixerValid | Pro aktuální Seq je platný controller/throttle/frame a alokace/achieved v definované doméně |
| LoggingAvailable | Logging je zkompilován, povolen a existuje připravený zapisovací backend; samo nepotvrzuje uložení každého paketu |
| ShadowHealthy | Aktuální výpočet je vyhodnocený a StateValid AND ControllerValid AND MixerValid; execution/config/frame podmínky splněné; **nezávislé na loggeru** |
| ShadowObservable | LoggingAvailable a aktuální diagnostický proud lze zaznamenávat při evidované skutečné frekvenci; mezery/drop/reduced-rate jsou označené |

Doplnit ControllerEvaluated/MixerEvaluated (nebo ekvivalent enum stavu
NOT_EVALUATED/VALID/INVALID). valid=false při policy skip není důkaz PID
chyby; bez vyhodnocení je matematická health neznámá, nikoli diagnostikovaná
porucha. Zalogovat odděleně CalculationReason a ObservationReason/
ExperimentPolicyReason; jeden kombinovaný reason může být jen souhrn.
FRAME_MISMATCH je konkrétní výpočetní/frame důvod, LOGGING_UNAVAILABLE je
observability/policy důvod. Jedno nesmí přepsat druhé.

Policy FÁZE 1: bez dostupného loggeru experiment nezačne; stav
EXPERIMENT_SUPPRESSED, ControllerEvaluated=false, MixerEvaluated=false.
Nedostupný logger nevytváří matematickou chybu PID/mixeru. Po zahájení
výpadek backendu/drop označí ShadowObservable=false a loss counter;
**sám nemění ControllerValid/MixerValid a neresetuje PID**. Výpočet smí
pokračovat s bounded drop logů, pokud dodrží CPU budget; experimentální
data s mezerou se označí jako neúplná. Samostatný TIME_BUDGET může výpočet
zastavit. Budoucí aktivní UTB musí zachovat toto oddělení a nesmí zdravou
regulaci deaktivovat pouze kvůli výpadku logování.

### 16.2 Logovací frekvence a transport

[ARDUPILOT INFRASTRUKTURA] AP_Logger má WriteBlock i dynamický
WriteStreaming. Pro rychlou cestu použít pevné packed structures a registry
Copter::log_structure[], bez dynamického formátování/alokace. IDs přidat
bez přečíslování existujících. Zápis musí respektovat neblokující AP backend
styl, bez čekání na fyzické médium a bez opakovaných retries v rate loop.

[AP_UTB NOVÉ; ENGINEERING DECISION] Nahrazujeme původní návrh 50 Hz:

- UTBS: změna stavu a heartbeat nejvýše přibližně 1 Hz. Heartbeat limit
  neomezuje důležité stavové přechody; opakovanou totožnou chybu nesypat
  v každém rate cyklu jako další změnu stavu.
- UTBR (tři osy) a UTBM: společné vzorkování diagnostických sad, cílově
  **alespoň 200 sad/s** při 400Hz controlleru.
- Umožnit krátkodobý **400Hz** režim (jedna sada za každý 400Hz krok), pokud
  to reálný scheduler/CPU/backend dovolí. Nelze logovat víc unikátních
  výpočtů než běží controller. Při nižším controller rate uvést jeho mez.
- Controller stále počítá v každém shadow cyklu. Log rate neurčuje dt,
  controller rate ani počet PID update. Změna log rate neresetuje integrátor.

Navržený skutečně využitý AP_Param UTB_LOG_RATE: Hz, rozsah 1–400, default
200; 400 je explicitní experimentální nastavení, ne hardware ověřená
vlastnost. Při CPU/logger přetížení nejdřív snížit nebo dropnout logování,
nikoli blokovat controller. Fyzická H743 měření musí potvrdit nejvyšší
bezpečně udržitelnou frekvenci; když 200/400 nelze udržet, použít nejvyšší
**ověřenou** hodnotu a dokumentovat omezení analýzy D/rychlé dynamiky.

Výpočet publikuje fixed snapshots do malé bounded fronty/kruhového bufferu
bez heapu; výběr sad podle log rate, timestamps a Seq. Samostatný logger
consumer musí umět obsloužit zvolených 200/400 Hz podle skutečné scheduling
kapacity, ne zůstat 50Hz taskem. Hloubku/frontu/RAM určit při implementaci
z naměřené latence; při plné frontě drop + counter, bez blokujícího wait.
400Hz consumer může být FAST_TASK na konci fast task části, aby nový
logger zápis neoddaloval aktuální AP motor output. Přesný consumer bod je
implementační detail podléhající timing kontrole, nikoli přesun controlleru.

### 16.3 Pole a časové zarovnání

UTBR všechny osy a UTBM tvoří sadu se stejným TimeUS/Seq/AP mode;
TimeUS je čas shadow sample, nikoli opožděný okamžik logger zápisu.
AP target má vlastní APTimeUS/APSeq/APMode z read-only capture (15.1).

| Zpráva | Pole |
| --- | --- |
| UTBS | TimeUS, Seq, Enabled, Shadow requested, AP mode, StateValid, ControllerValid, MixerValid, evaluated flags, LoggingAvailable, ShadowHealthy, ShadowObservable, CalculationReason, ObservationReason, PolicyReason, Dt, execution/max, requested/effective log rate, drop/missing counters |
| UTBR | TimeUS, Seq, AP mode, Axis, Desired/Measured/Error rad/s, P/I/D normalized, Raw v, Output c, Dt s, ControllerValid, saturation, read-only AP target rad/s, APTimeUS/APSeq/APMode a capture validity |
| UTBM | TimeUS, Seq, AP mode, R/P/Y/t, M1..4, achieved R/P/Y/t, limit flags, MixerValid, s, collective shift |

Pokud field/packet limit AP loggeru nedovolí všechna pole jedné zprávy,
rozdělit status/metadata do doplňkové pevné zprávy se stejným Seq; pole
nepotichu nevynechat. Normalized units dimensionless, rate rad/s, dt s;
TimeUS/APTimeUS jsou mikrosekundy s AP log metadata převodem. Invalid
výpočet má finite zeros a příslušnou validitu/důvod; žádný valid old output.
UTB_ENABLE=0 negeneruje UTB heartbeat. Respektovat LOG_DISARMED/AP logging.

Ztráta jedné UTBR osy nebo UTBM zneúplní celou sadu v offline analýze.
LoggingAvailable nelze zaměnit s potvrzením persist všech paketů: API/
backend counters a dekódovaný BIN musí ověřit doručené sady, Seq mezery,
časový jitter a efektivní log rate. Bez těchto měření je použitá frekvence
požadovaná/pozorovaná v SITL, ne hardware ověřená.

### 16.4 H743 měření a srovnání

Při budoucím bench testu bez UTB motor authority porovnat 0/200/krátce400Hz
logování při stejném AP logging profilu: CPU load, shadow i logger time
(max a distribuce), scheduler PM/overruns, fronta/max backlog, dropped
packets/sady, backend throughput a skutečná frekvence/time jitter z BIN.
Před testem ověřit FRAME_CLASS/FRAME_TYPE přímo na FC. Udávat délku testu,
backend/médium, konfiguraci a aktivity ovlivňující logger; krátký 400Hz
záznam neprokazuje dlouhodobou udržitelnost. Hardware měření nyní neprovedeno.

Existující RATE/PID/RCOU logy zarovnávat podle času a capture/sample původu,
ne jen podle nejbližšího timestampu nebo očekávané frekvence. UTB je
AP-derived shadow reference, ne identická AP rate reference. AP compensation,
spool, slew, linearizace, filtrace a ESC převod dále odlišují RCOU od M1..4.
Žádný AP controller/mixer/output se nesmí kvůli srovnání spustit podruhé.

## 17. Matematické příklady a plán verifikace

[AP_UTB NOVÉ] Přesné příklady pro navržený allocator:

| [t,R,P,Y] | [M1,M2,M3,M4] |
| --- | --- |
| [0,0,0,0] | [0,0,0,0] |
| [0.5,0,0,0] | [0.5,0.5,0.5,0.5] |
| [0.5,+0.2,0,0] | [0.4,0.4,0.6,0.6] |
| [0.5,0,+0.2,0] | [0.4,0.6,0.4,0.6] |
| [0.5,0,0,+0.2] | [0.4,0.6,0.6,0.4] |
| [0.95,+0.4,0,0] | [0.6,0.6,1,1], t_ach=0.8, s=1 |
| [0.05,+0.4,0,0] | [0,0,0.4,0.4], t_ach=0.2, s=1 |
| [0.5,+1,+1,+1] | [0,1,1,1], t_ach=0.75, s=0.5 |

Dne 8. 10. 2026 prošla nezávislá numerická kontrola: odvození roll/pitch
faktorů z AP úhlů a normalizace, přesné B*A=I v racionální aritmetice,
všech 8 příkladů a 1000 případů se seedem 76138 (bounds a achieved).
Lokální evidence: `tmp/utb-theory-math-check.log` (ignorováno Gitem).
Numerická kontrola tohoto návrhu používá pouze dočasný Python/Fraction
skript; není C++ implementací ani jejími unit/SITL testy.

Po schválení controller testy: zero/positive/negative P; I accumulation;
+IMAX/-IMAX; reset; D response; reference step s konstantním measurement
-> D=0; invalid/variable dt; NaN/Inf; invalid state; saturation anti-windup
v obou směrech, odeznění a jednokrokový delay; nezávislé osy; config change
a priming. Mixer testy: zero/equal thrust; ±R/±P/±Y; mixed R/P;
high/low thrust; +/-saturation; BF_X permutation versus test order;
finite/NaN/Inf; achieved a flags. Property tests BA roundtrip,
bounds, s∈[0,1], achieved=s*c v rámci eps.

SITL: compile on/off, boot enable 0/1, shadow off/on, STABILIZE/ACRO,
ALT_HOLD dostupnost, všechny FÁZE 0 UTB arming/force/RC/armed-entry zákazy;
logy s nenulovými **testovacími**, ne letovými gains; invalid state/dt/RC/
config/FSTRATE -> invalid output/reset recovery. Ověřit FRAME_CLASS!=1
a FRAME_TYPE=18/jiný typ -> FRAME_MISMATCH, MixerValid=false. Testovat
logger unavailable před startem (NOT_EVALUATED/policy skip), runtime drop
bez invalidace zdravého PID, oddělené health flags a 200/400Hz Seq/sample
logování, overflow bounded fronty a skutečný počet kompletních sad.
NaN vstupy injektovat
unit/test-only providerem, nikdy poškodit AP gyro nebo motor path.

Motorová izolace: audit zákazů API plus dvě reprodukovatelné SITL jízdy
stejné firmware/inputs/seed, shadow off/on, RCOU zarovnání simulation času.
Dvě různé ruční jízdy nejsou důkaz. Při jitter nelze slibovat bitovou shodu
volně běžících simulací; dokumentovat seed/timing/toleranci a vyšetřit
každý rozdíl. Porovnat i CPU/PM/log drop. Neprohlašovat testy za hotové.

Build: SkystarsH7HD-bdshot on/off, SITL on/off, C++/Python regrese,
flake8, astyle změněných částí, diff check, param/log metadata.
Hardware/let je zvláštní ověření, neprovedené a neautorizované tímto krokem.

## 18. Předchozí BP: budoucí vrstvy, nyní jen dokumentace

### 18.1 Výška a stavová zpětná vazba

[PŘEDCHOZÍ BP] Kap. 6.8, str. 62–64, (19)–(25):

```math
m ddot(h)=T-mg-d_phys dot(h), T_hover≈mg,
G(s)=K_eff/[s(s+d_model)],
e_h=h_ref-h,
u_h=K_h e_h,filtered-K_vh v_h,
T_cmd=T_hover+u_h.
```

BP označuje výšku z a roste nahoru; zde h pro odlišení NED z.
d_phys má kg/s, d_model=d_phys/m v 1/s při jednoduchém odvození přenosu.
BP označuje obojí d, numerické hodnoty nejsou bez normalizace totožné.
BP str. 63 výslovně uvádí praktický RC plyn místo N; její firmware gains
a hover channel nelze vydávat za SI tah. [ROZŠÍŘENÍ BP] Budoucí altitude
error -> vertical velocity target -> velocity controller -> thrust;
h=-z_N, v_h=-v_D. Finální ALT_HOLD variantu ani gains teď nevolíme.

### 18.2 Error shaping, reference ramp a filtrace

[PŘEDCHOZÍ BP] (25) str. 64: e_n=e/(1+k_e|e|), k_e>=0 inverse error
jednotka. Malá chyba téměř lineární, velká omezená na ±1/k_e; k_e=0
identita. Patří do explicitního ERROR/REFERENCE SHAPING, ne skrytého PID.
Ve FÁZI 1 rate nepoužít automaticky.

[PŘEDCHOZÍ BP; ROZŠÍŘENÍ BP zápisu] Str. 64/66 princip plynulé reference:
ref_next=ref+clamp(target-ref,-rate_limit dt,+rate_limit dt), jednotka
rate_limit reference/s. Budoucí attitude/altitude/position/trajectory;
první Acro statická mapa novou rampu nemá (omezení oddíl 6).

[PŘEDCHOZÍ BP] (13) str. 61: x_f,k=alpha x_f,k-1+(1-alpha)x_k,
0<=alpha<1. Větší alpha -> méně šumu, více zpoždění. Pevné alpha při
proměnném dt neznamená stejnou cutoff; D zde má alpha=1-beta(dt).
AP gyro se redundantně nefiltruje.

### 18.3 XY a znaménka nové soustavy

[PŘEDCHOZÍ BP] Str. 64–65, obr. 15, (26)–(31):
e_x=x_ref-x, e_y=y_ref-y; u_x=K_px e_x-K_vx vx,
u_y=K_py e_y-K_vy vy; theta_ref=k_theta u_x, phi_ref=k_phi u_y.
Poloha m, rychlost m/s, postoj rad; gains závisejí na doméně u_x/u_y
(v BP praktické povely náklonu). Staré RC znaménkové koeficienty neznáme.

[ROZŠÍŘENÍ BP] Při malých náklonech, yaw=0, T≈mg:
a_N≈-g theta, a_E≈g phi. Budoucí yaw transformace do heading frame:
[a_F,a_R]=[cos psi,sin psi;-sin psi,cos psi][a_N,a_E],
theta_ref≈-a_F/g, phi_ref≈a_R/g, plus limity náklonu.
To je ilustrace FRD znamének, ne hotový position controller; velké náklony,
yaw dynamika či jiné T porušují aproximaci. XY se nyní neimplementuje.

### 18.4 Vertikální rychlost, optical flow, estimator

[PŘEDCHOZÍ BP] (12), str. 61: v_h,k=(h_k-h_k-1)/T_s. Diference zesiluje
šum a závisí na timestamp; současné řešení nepřidává vlastní differentiator,
budoucí velocity state primárně validně z AP AHRS/EKF.

(14),(15) optical flow: vx=Delta_x k_flow h/Delta_t, obdobně vy.
Delta obrazové counts/pixely, k_flow kalibrační rad/count měřítko, m/s
výsledek za platné geometrie. Přejmenování k_flow brání záměně s rotor k_f.
Quality, valid height, tilt/rotace a velocity limit nutné. BP dokládá
quality/height/velocity a tilt-dependent důvěru estimatoru, ne úplnou
IMU/flow kalibraci bez source firmware. FÁZE 1 flow neimplementuje.

[PŘEDCHOZÍ BP] (16)–(18), str. 62: stav [x,y,vx,vy], predikce konstantní
rychlosti x_k=x_k-1+vx T_s, y_k=y_k-1+vy T_s. BP popisuje Kalmanův filtr
s korekcí flow, ale nedokládá plné F,H,Q,R/covariance update/observability.
Není ekvivalent EKF3; může být budoucí výukový/reference estimator.
Drift a kvalitu musí další návrh vyřešit. Teď žádný vlastní UTB estimator.

### 18.5 Autonomie, waypointy a překážky

[PŘEDCHOZÍ BP] Kap. 6.10 str. 65–67, obr. 16: autonomie generuje reference,
nemění nízkoúrovňové controllers. Mission -> state machine -> trajectory
-> position -> velocity -> attitude -> rate -> mixer.
TAKEOFF -> HOLD -> MOVE_XY/TRACK -> HOLD -> NEXT WAYPOINT -> LAND -> LANDED.
Splnění waypointu polohová **i rychlostní** tolerance, ne pouhý průlet;
reference plynulé. Obstacle detection dočasně změní trajectory/reference,
po obletu návrat k původnímu cíli; rate/attitude/stabilizace stejné.
Teď pouze teoretický základ budoucí UTB_AUTO, žádná implementace.

## 19. Skutečný manifest SHADOW patche

AP_Motors, HAL, DShot, EKF, submoduly a arming pravidla FÁZE 0 se nemění.
Změna/přidání je proti HEAD `761a38d704`; teorie v2 již existovala jako
nezařazený soubor před implementací. Žádný commit ani push nebyl proveden.

| Soubor | Git druh | Skutečný zásah |
| --- | --- | --- |
| libraries/AP_UTB/AP_UTB.h | Změna | Input/capture/snapshot, oddělené health/reasons, bounded queue a lifecycle API |
| libraries/AP_UTB/AP_UTB.cpp | Změna | 18 parametrů, guards/reset/priming, engine, decimace/fronta/counters, časový latch |
| libraries/AP_UTB/AP_UTB_State.cpp | Změna | Health přes AHRS primary gyro index; původní State.h rozhraní stačilo |
| libraries/AP_UTB/AP_UTB_RateController.h | Změna | Gains, tři stavy, axis terms/priming a vlastní saturation feedback |
| libraries/AP_UTB/AP_UTB_RateController.cpp | Změna | Schválený PID, D backward Euler, finite guards a directional anti-windup |
| libraries/AP_UTB/AP_UTB_MotorMixer.h | Změna | Čistý BF_X result/achieved/flags a supports_frame |
| libraries/AP_UTB/AP_UTB_MotorMixer.cpp | Změna | Matice, common scale/collective shift, inverse a finite invalid zeros |
| libraries/AP_UTB/AP_UTB_Reference.h | Přidání | Čistá static AP-derived reference API |
| libraries/AP_UTB/AP_UTB_Reference.cpp | Přidání | Circular limiting, skutečné input_expo a rad/s/range validace |
| libraries/AP_UTB/tests/test_utb.cpp | Změna | 26 testů v jednom targetu včetně zachovaných state/default FÁZE 0 tests |
| ArduCopter/utb.cpp | Přidání | Readonly AP RC/throttle/state bridge, logger consumer a pomalý GCS status |
| ArduCopter/utb_log.h | Přidání | Packed static UTBS/UTBR/UTBM/UTBA/UTBT a logger metadata |
| ArduCopter/Attitude.cpp | Změna | Read-only AP target capture těsně před jediným existujícím rate voláním |
| ArduCopter/Copter.h | Změna | Deklarace tří UTB callbacks |
| ArduCopter/Copter.cpp | Změna | Shadow za motor outputem, logger na konci FAST_TASK a status 1 Hz |
| ArduCopter/mode_utb_acro.cpp | Změna | Text oznámení již není jen phase 0; arming/vstup/spool guard nezměněn |
| ArduCopter/Log.cpp | Změna | Registrace static packetů |
| ArduCopter/defines.h | Změna | IDs 13–17; původní IDs 0–12 beze změny, lokální zarovnání dotčeného enumu |
| Tools/autotest/test_utb_skeleton.py | Změna | Volitelné extra defaults/model/speedup pro reuse; původní assertions zachovány |
| Tools/autotest/test_utb_shadow.py | Přidání | SITL shadow/health/guards/fronta/rate/RC/SYSID a motor isolation |
| docs/UTB_CONTROL_THEORY_CS.md | Přidání vůči HEAD | Schválená v2 a evidence skutečné implementace; matematické bloky nezměněny |
| docs/UTB_ARCHITEKTURA_CS.md | Změna | Oddělení historického aktivního návrhu od implementované SHADOW FÁZE 1 |
| README_BUILD_WSL_CS.md | Změna | Reprodukovatelné příkazy, evidence a žádné nové instalace |

Config/wscript/build_options z FÁZE 0 poskytují potřebné guards, library a
--enable-UTB/--disable-UTB; nebylo nutné je měnit. Původní návrhový manifest
měl lib header LogStructure.h a tři další testovací soubory: skutečné packed
logy jsou v vehicle headeru `ArduCopter/utb_log.h`, protože IDs/registry patří
Copteru. Test suites jsou v existujícím jediném `test_utb.cpp`/test targetu.
Jde o organizaci souborů; význam rovnic, logů a testovací rozsah se nemění.

## 20. Finální otevřené otázky a rozsah schválení

PID measurement D, backward Euler LPF, AP-derived statická reference bez
traineru/TC, BF_X=12 matematická matice, diagnostické poměrové RPY priority
a SHADOW ONLY directional anti-windup tvoří matematický základ SHADOW FÁZE 1.
Implementace a skutečné výsledky jsou odděleny v oddílu 24. Hardware/let
nebyly ověřeny. Zbývají tyto ověřovací otázky:

1. **FC konfigurace před hardware testem:** skutečné FRAME_CLASS/FRAME_TYPE,
   SERVO mapping, motor wiring a CW/CCW. První implementace je výhradně
   Quad/BF_X=12; odlišný typ vyžaduje další explicitní rozhodnutí/matici.
2. **Log rate na H743:** kolik sad/s se bezpečně uloží s reálným backendem
   a běžnými AP logy? Ověřit 200 a krátce 400 Hz, CPU/logger load, jitter,
   backlog/drop a délku testu; případně použít nejvyšší ověřenou frekvenci.
   Fronta a layouts jsou implementované; jejich rezervu ověřit i na H743.
3. **Časová shoda:** SYSID capture i RC mapping byly ověřeny v SITL;
   hardware časování/freshness zůstává otevřené. V analýze nepárovat
   automaticky aktuální UTB reference s AP targetem z předchozího mode update.
4. **State freshness:** primary gyro failover/health, INS timestamp, starší
   AHRS quaternion před read_AHRS. Před aktivním controllerem budou potřeba
   silnější záruky; tato fáze zůstává diagnostická.
5. **Experimentální gains/cutoff:** nenulové hodnoty podložit gyro/log daty,
   nenazývat je letovým tune. Ki=0 default; integrátor ve shadow není vlastní
   closed-loop validace. Platné nulové defaults ani fronta logů to nemění.
6. **Logger na hardware:** místní API, metadata, IDs, health/fronta a
   decoded BIN byly ověřeny. Doručení na reálné médium/load/drop zůstává
   otevřené; LoggingAvailable a úspěšné enqueue neslibují persist.
7. **Akademická dohledatelnost:** získat a prostudovat konkrétní plné pasáže
   literatury pro konečné stránkové/rovnicové citace bakalářské práce.
   Metadatový/abstraktový záznam není ověřením plného textu; omezení oddíl 22.
8. **Budoucí aktivní UTB reference policy:** vlastní UTB rate/expo versus
   explicitní sdílení AP parametrů. Tato volba je odložena; FÁZE 1 ji nezamyká
   a nepřidává nové UTB rate/expo parametry.

Rozsah této verze je pouze SHADOW FÁZE 1. Nezahrnuje aktivní UTB motor output, armovatelný UTB_ACRO, další letové
režimy, EKF/flow estimator ani DShot/HAL změny.

## 21. Mapa primárních zdrojů

| Tvrzení | Zdroj |
| --- | --- |
| Dynamika/stav/feedback | Literatura L1–L6, vlastní FRD odvození; BP str. 38–41 dokládá použití v předchozím projektu |
| Kaskáda | Literatura L1/L4, BP str. 44–45 dokládá návaznost, ne původ obecné teorie |
| BP architektura | PDF str. 56–60, tab. 5 |
| BP sensing/flow/estimator | PDF str. 61–62, (12)–(18) |
| BP altitude/shaping | PDF str. 62–64, (19)–(25) |
| BP XY | PDF str. 64–65, (26)–(31), obr. 15 |
| BP mise/překážky | PDF str. 65–67, kap. 6.10/obr. 16 |
| RC normalized input | [RC_Channel.cpp](../libraries/RC_Channel/RC_Channel.cpp), norm_input_dz |
| Acro map | [mode_acro.cpp](../ArduCopter/mode_acro.cpp), get_pilot_desired_rates_rads |
| Expo | [control.cpp](../libraries/AP_Math/control.cpp), input_expo |
| AP command shaping | [AC_AttitudeControl.cpp](../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp), input_rate_bf_*, attitude_command_model |
| Throttle/TC | [mode.cpp](../ArduCopter/mode.cpp), get_pilot_desired_throttle, update rate TC |
| Gyro | [AP_AHRS_Backend.cpp](../libraries/AP_AHRS/AP_AHRS_Backend.cpp), get_gyro_latest |
| View/frame | [AP_AHRS_View.cpp](../libraries/AP_AHRS/AP_AHRS_View.cpp), .h; [system.cpp](../ArduCopter/system.cpp) |
| Rate dt | [Attitude.cpp](../ArduCopter/Attitude.cpp); [AP_Scheduler.cpp](../libraries/AP_Scheduler/AP_Scheduler.cpp), loop |
| Task order/FSTRATE | [Copter.cpp](../ArduCopter/Copter.cpp); [rate_thread.cpp](../ArduCopter/rate_thread.cpp) |
| Hardware defaults | [bdshot hwdef](../libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD-bdshot/hwdef.dat), [base hwdef](../libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD/hwdef.dat) |
| Frame/param defaults | [Parameters.cpp](../ArduCopter/Parameters.cpp), DEFAULT_FRAME_CLASS/FRAME_TYPE/ACRO_* |
| BF_X factors/saturation | [AP_MotorsMatrix.cpp](../libraries/AP_Motors/AP_MotorsMatrix.cpp), setup_quad_matrix/add_motor/normalise/output_armed_stabilizing; [header](../libraries/AP_Motors/AP_MotorsMatrix.h) |
| Frame enum | [AP_Motors_Class.h](../libraries/AP_Motors/AP_Motors_Class.h) |
| Motor function mapping | [AP_Motors_Class.cpp](../libraries/AP_Motors/AP_Motors_Class.cpp), add_motor_num; [SRV_Channel_aux.cpp](../libraries/SRV_Channel/SRV_Channel_aux.cpp), set_aux_channel_default |
| Logger API/registration | [AP_Logger.h](../libraries/AP_Logger/AP_Logger.h), WriteBlock; [Log.cpp](../ArduCopter/Log.cpp), log_structure |
| PID background | Åström, K. J.; Murray, R. M.: Feedback Systems, kap. 10, odkaz oddíl 9.1 |

## 22. Odborné zdroje a evidence jejich použití

[LITERATURA] BP slouží především pro kontinuitu projektu, předchozí
implementaci a zkušenosti/výsledky (včetně omezení popsaných v kap. 7).
Fundamentální teorii odkazujeme přímo na původní odborné publikace.
Bibliografii nekopírujeme bez kontroly: některé údaje BP se liší od
vydavatelských záznamů. Níže jsou odkazy na primární publikace/vydavatele
nebo institucionální repozitář a skutečný rozsah ověření 8. 10. 2026.

| ID | Odborný zdroj | Použití a skutečně ověřený rozsah |
| --- | --- | --- |
| L1 | Quan Quan, Xunhua Dai, Shuai Wang: [Multicopter Design and Control Practice](https://link.springer.com/book/10.1007/978-981-15-3138-5), Springer, 2020, DOI 10.1007/978-981-15-3138-5 | BP [3]. Ověřena vydavatelská metadata/obsah a abstrakty kap. 6 Dynamic Modeling Experiment (153–199) a kap. 9 Attitude Controller Design Experiment (249–283). Modelování a návaznost attitude na position jsou odborná témata; plný text kapitol za přístupem nebyl prostudován. |
| L2 | Randal W. Beard, Timothy W. McLain: [Small Unmanned Aircraft: Theory and Practice](https://www.jstor.org/stable/j.ctt7sbc4), Princeton University Press, 2012, ISBN 9780691149219 | BP [4]. Ověřena metadata a vydavatelský popis na distribuční platformě JSTOR. Obecný rámec UAV dynamics/control/guidance, zejména fixed-wing; není zdrojem BF_X motorových znamének nebo potvrzením UTB PID gains. Plný modelový text zde neověřen. |
| L3 | Luis Rodolfo García Carrillo, Alejandro Enrique Dzul López, Rogelio Lozano, Claude Pégard: [Quad Rotorcraft Control: Vision-Based Hovering and Navigation](https://link.springer.com/book/10.1007/978-1-4471-4399-4), Springer, ©2013, DOI 10.1007/978-1-4471-4399-4 | Vydavatelská metadata, popis a obsah ověřeny; vydání uvedeno 2012, copyright 2013. Odborný kontext quadrotor modeling/attitude/navigation. Nezaměňovat tuto knihu automaticky s disertací 2011 uvedenou jako BP [7]. Plné rovnice knihy neověřeny. |
| L4 | Robert Mahony, Vijay Kumar, Peter Corke: [Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor](https://ieeexplore.ieee.org/document/6289431), IEEE Robotics & Automation Magazine 19(3), 2012, 20–32, DOI 10.1109/MRA.2012.2206474 | BP [15]. Ověřen abstrakt a metadata IEEE; [institucionální záznam ANU](https://digitalcollections.anu.edu.au/items/80932d19-935c-4333-9f94-b6bcfd6ef12e/full). Primární odborný základ multirotor modeling/estimation/control; plný PDF nebyl dostupně ověřen, přesná čísla rovnic zde neuvádíme. |
| L5 | Zaid Tahir, Mohsin Jamil, Saad Ali Liaqat, Lubva Mubarak, Waleed Tahir, Syed Omer Gilani: [State Space System Modeling of a Quad Copter UAV](https://indjst.org/articles/state-space-system-modeling-of-a-quad-copter-uav), Indian Journal of Science and Technology 9(27), 2016, 1–5, DOI 10.17485/ijst/2016/v9i27/95239 | BP [10]. Ověřena metadata a abstrakt vydavatele: 6DOF state-space modeling z Newtonových rovnic. Vydavatel uvádí šest autorů a 1–5 stran (BP má zkrácené autory a 1–10). Pokus stáhnout full text neprošel; žádné tvrzení o přečtení jeho plných rovnic. |
| L6 | Karl J. Åström, Richard M. Murray: [Feedback Systems: An Introduction for Scientists and Engineers](https://www.cds.caltech.edu/~murray/books/AM08/pdf/am08-complete_28Sep12.pdf), Princeton University Press, 2008, první vydání, kap. 10 PID Control | Doplněný odborný zdroj. [Autorské výukové materiály](https://murray.cds.caltech.edu/CDS_101/110_-_PID_Control) ověřují PID, actuator saturation a anti-windup jako témata kap. 10. Celé PDF se nyní nepodařilo načíst; zdejší derivace discretization je vlastní a explicitní. U druhého vydání/draftu může být PID kap. 11; nesmí se míchat kapitoly různých vydání. |
| L7 | Daniel Mellinger, Vijay Kumar: [Minimum Snap Trajectory Generation and Control for Quadrotors](https://doi.org/10.1109/ICRA.2011.5980409), ICRA 2011, 2520–2525 | BP [18]. Zahrnuto jako primární kandidát pro pozdější trajectory vrstvu, nikoli zdroj rate PID FÁZE 1. DOI/bibliografická návaznost evidovány; plný text nyní neověřen, žádný minimum-snap algoritmus se nepřebírá. |

Odkazy na konkrétní odborné části L1:
[Dynamic Modeling Experiment](https://link.springer.com/chapter/10.1007/978-981-15-3138-5_6),
[Attitude Controller Design Experiment](https://link.springer.com/chapter/10.1007/978-981-15-3138-5_9).
Nezaměňovat bibliograficky ověřený rozsah kapitoly s tvrzením, že byly
přečteny všechny její stránky. Konečná BP musí doplnit přímé přesné citace
z konkrétního zpřístupněného vydání; nyní je to otevřená citační práce,
ne překážka sepsání tohoto explicitně odvozeného technického návrhu.

### 22.1 Původ konkrétních návrhových rozhodnutí

| Rozhodnutí | Literatura | ArduPilot source convention | Vlastní engineering decision |
| --- | --- | --- | --- |
| Newton–Euler/gyro state | L1–L5 jako odborný modelový rámec | FRD/NED, view a rad/s APIs | Explicitní FRD rovnice a oddělení fyzikální/normalized domény |
| Rate PID/D LPF | L6 obecný PID/filter/anti-windup základ | Filtrované latest gyro a skutečný scheduler dt | Measurement D, backward Euler, cutoff guards a nulové gains nejsou vydávány za převzatý tune |
| Reference | Odborný princip oddělení reference/controlleru | AP calibration/deadzone/circular/input_expo/rate params | AP-derived UTB shadow reference pouze pro FÁZI 1, bez trainer/TC |
| BF_X | Literatura nepředepisuje místní motor order | setup_quad_matrix, raw factors a normalise_rpy_factors | Pouze typ 12, runtime FRAME_MISMATCH guard; žádné potvrzení FC |
| Allocation priority | Obecná omezená alokace akčních veličin | AP saturace analyzována, ale její implementace se nekopíruje | Diagnostická poměrová RPY priorita/collective shift, SHADOW ONLY |
| Anti-windup | L6 obecný windup problém | AP controller/limits se nemění | Minulý-krok directional residual gate, SHADOW ONLY; Ki=0 default |
| Logging/health | Log není matematická zpětná vazba controlleru | Static AP logger/scheduler/read-only target API | 200/400Hz experiment, oddělená validita/observable a CPU policy |

## 23. Záznam této revize vůči verzi 1

1. BF_X=12 je podporovaný matematický typ, fyzická konfigurace nepotvrzena;
   runtime class/type guard a FRAME_MISMATCH, typ 18 nepodporován.
2. 50Hz plán nahrazen 200Hz cílem a krátkým 400Hz experimentem; UTB_LOG_RATE,
   pevná bounded fronta/consumer, H743 měřicí postup a fallback na ověřenou
   frekvenci. PID nadále v každém shadow kroku.
3. Odděleny State/Controller/MixerValid, evaluated, LoggingAvailable,
   ShadowHealthy/Observable, calculation/observation/policy reasons.
4. Přesně popsán scheduler sample alignment; shadow místo zachováno,
   nový read-only AP target capture před existujícím AP rate voláním v
   Attitude.cpp řeší i SYSID reset. Žádný druhý AP controller.
5. Výslovné AP-derived UTB shadow reference a odložení budoucího výběru
   vlastní/sharing rate/expo policy; nyní bez nových UTB rate/expo.
6. Vyjmenovány priority a)–d), yaw rovnocenný RP; diagnostický allocator
   není schválený aktivní output. Matice ani rovnice alokace se nemění.
7. Anti-windup výslovně SHADOW ONLY; shadow I není UTB closed-loop experiment,
   Ki=0 default zachován. Rovnice anti-windup se nemění.
8. Doplněna odborná bibliografie, stav dostupnosti a trojice literatura /
   ArduPilot convention / vlastní engineering decision; BP pro kontinuitu.

Skutečné změny návrhu: logging transport/rate a parametr, health/policy
model a read-only capture pro srovnání. Frame guard, reference a priority
jsou zpřesnění dřívějšího omezení. PID, D filtr, BF_X matice a directional
anti-windup matematika zůstávají. C++ FÁZE 1 nebylo implementováno a buildy
nebyly opakovány při této čistě dokumentační revizi.



## 24. Implementace a evidence SHADOW FÁZE 1 (8. 10. 2026)

Historická evidence před BF_X_REV; aktuální rozšíření ve větvi test/skystars-5inch popisuje oddíl26.

**IMPLEMENTOVÁNO:** reference/PID/diagnostický BF_X allocator, parametry,
read-only AP capture, shadow/consumer tasks, oddělené health/policy,
bounded fronta, static logy a testy. **Žádný aktivní UTB output.**
Arming checks FÁZE 0 nejsou změněny. UTB_ACRO zůstává disarmed-only.
Autoritativní schválené matematické bloky oddílů 9, 11, 13 a 14 se nemění.

### 24.1 Konkrétní runtime/transport volby

- Podporován jen runtime class=1/type=12. Frame mismatch nevyhodnotí mixer,
  MixerEvaluated=false, MixerValid=false, CalculationReason=FRAME_MISMATCH
  a m_i=0. Typ 18 je odmítnut. Simultánní důvody mají pořadí policy,
  fast rate, frame, dostupnost Acro, dt, state, reference, gains/controller.
- StateValid se neposuzuje jako chyba PID: čerstvý state může zůstat validní
  při chybě dt. ShadowObservable může být true u kompletně logovatelného
  invalid výpočtu; není závislé na ShadowHealthy. Startup bez loggeru má
  NOT_EVALUATED a EXPERIMENT_SUPPRESSED. Runtime logger loss nerozpojuje
  matematiku ani neresetuje I/D. Fronta/drop/write failure jsou observability.
- Bridge používá dostupné `RC_Channels::last_input_ms()`. Neoznačuje každý
  400Hz průchod za novou RC zprávu. Throttle calibration/range denominator
  je ověřen před AP helperem. Roll/pitch/yaw používají AP RCMAP channels,
  norm_input_dz a skutečnou static command-model rate/expo konfiguraci.
- Controller má měřený hlavní loop dt a nominal guards dle návrhu. Čas se
  měří od bridge reference/state části včetně enqueue; metadata commit
  výsledné časové hodnoty je malý závěrečný store. Tři sousední >5 % nominal
  overruns latched TIME_BUDGET do SHADOW off/on; další kroky již ani
  nespouštějí state/reference bridge. AP řízení se nemění.
- Fronta má čtyři snapshoty, main-thread producer/consumer. Jeden consumer
  průchod odebere nejvýše jednu sadu, žádný UTB wait/retry. Při nedostupném
  loggeru se fronta nedrainuje; po čtyřech místech další sady drop + counter.
  Při backend write failure se sada neopakuje, missing counter označí možnou
  partial sadu. Úspěch API je enqueue na první backend, nikoli persist.
- SITL sizeof snapshot=328 B, queue=1320 B. Consumer v běžném cyklu odebere
  publikovanou sadu ve stejném průchodu; čtyři místa při 400 Hz představují
  nominální 10ms rezervu. V řízeném výpadku se naplnění/drop ověřilo.
  Skutečnou latenci a potřebnou rezervu na H743 nadále musí změřit bench test.
- Deadline decimace podle TimeUS vybírá 1..400 sad/s, maximálně jednu na
  rate krok; nepoužívá 50Hz consumer a nemění dt/PID. Platná změna LOG_RATE
  neresetuje controller. Eff je průběžný průměr kompletních enqueue sad od
  začátku experimentu; intervalovou persist frekvenci měří decoded BIN.
- GCS status je samostatný pomalý 1Hz scheduled callback: změny a 5s heartbeat
  `UTB flags=... calc=... obs=... policy=...`. Umožňuje ověřit suppressed
  policy bez loggeru. Formátování není ve shadow/rate produceru. ENABLE=0
  neprodukuje UTB status/log/capture/výpočet; SHADOW=0 neinkrementuje Seq.
- Logger používá standardní AP API a jeho existující backend synchronizaci,
  žádné vlastní čekání na médium. To není hard-real-time garance nulové
  latence všech AP backendů; logger maximum/load je předmětem H743 měření.

### 24.2 Skutečný log formát

Místní limity: FMT má 16 format znaků, 64 label znaků, maximální packet
255 B. Proto zůstávají tři 55B UTBR osy; jejich sloučení by překročilo
16 polí. Alignment metadata jsou v UTBT, achieved/limit flags v UTBA.
Žádná požadovaná veličina se potichu nezahodila. Sada sdílí TimeUS/Seq/Md.

| Zpráva/ID/velikost | Skutečné labels |
| --- | --- |
| UTBS / 13 / 55 B | TimeUS,Seq,Md,Flg,Calc,Obs,Pol,Dt,Ex,Max,Req,Eff,Dr,Miss,Ov,LM |
| UTBR / 14 / 55 B, 3 osy | TimeUS,Seq,Md,Ax,Des,Meas,Err,P,I,D,Raw,Out,AP,Val,Sat |
| UTBM / 15 / 59 B | TimeUS,Seq,Md,R,P,Y,T,M1,M2,M3,M4,S,Shift,Val,Lo,Hi |
| UTBA / 16 / 36 B | TimeUS,Seq,Md,R,P,Y,T,Pos,Neg,Scale,Shift |
| UTBT / 17 / 60 B | TimeUS,Seq,Md,APUS,APS,APM,Cap,RCms,Dt,Ex,Max,Req,Eff,Drop,Miss |

Jedna úplná sada je 320 B: nominálně 64 kB/s při 200 Hz a 128 kB/s při
400 Hz, navíc ostatní AP logy a status. To je byte budget, ne naměřený
throughput H743. UTBS posílá změny a heartbeat přibližně nejvýše 1 Hz.
LM je maximum logger consumer času; Ex/Max jsou shadow časy v µs.
APUS/APS/APM/Cap nesou AP capture, RCms je skutečný poslední RC timestamp.
Rate a reference mají rad/s, Dt sekundy, normalizované veličiny jsou
bezrozměrné. Časové integer fields mají AP metadata převod µs/ms na s.
Sat bity 0/1 jsou raw-achieved positive/negative residual, bit 2 gate;
Pos/Neg bity R/P/Y/collective, Lo/Hi bity logical M1..M4.

### 24.3 Ověření a konkrétní omezení

UNIT: 26/26 C++ tests prošlo; XML a stdout v `tmp/utb-phase1-unit-results.*`
a `tmp/utb-phase1-unit-results-verified.log`. Obsah: state/freshness/wrap,
P signs/zero, I accumulation/±IMAX, independent axes, reset/config/prime,
D response/constant-measurement reference step, variable/invalid dt,
NaN/Inf/finite overflow, AW oběma směry/one-step/mixer saturation,
BF_X signed axes/inverse/bounds/9261 combinations, frame/type18 rejection,
reference/input_expo boundary, logger/math separation, FIFO/overflow,
CPU latch/reset. Syntetické časové vstupy unit latch nejsou měřením CPU.

SITL: lokální shadow regrese z finálního binary prošla; evidence v
`tmp/utb-phase1-sitl-verified/`. Obsahuje startup bez loggeru, runtime logger
výpadek/overflow/recovery, class/type/FSTRATE/gains guards, shadow off/on,
STABILIZE/ACRO/ALT_HOLD, UTB_ACRO ordinary/force/RC arm rejection i armed
entry rejection. FÁZE 0 compile/boot latch/mode/arming regrese on/off jsou
v `tmp/utb-phase1-skeleton-on-verified/` a `tmp/utb-phase1-skeleton-off/`.

SYSID: skutečný AP simulovaný let, axis 7, target LPF pro test vypnutý.
2758 přesných timestamp matches PIDR.TimeUS=APUS; maximální odchylka
AP PID targetu a captured targetu 0 rad/s v tomto běhu. Chirp je
nenulový a SIDD přítomné. Žádné druhé spuštění AP controlleru.

Motorová izolace: zdrojový audit `tmp/utb-phase1-audit.json` nenalezl UTB
motor setters/output/HAL/spool/limit calls. Dva lokální běhy stejného binary,
model bfx, stejné fixed RC/defaults/noise konfigurace, liší se pouze SHADOW.
20 servo samples v každé steady disarmed/armed-ground-idle fázi na běh:
0 rozdílů (1000/1100 PWM podle AP fáze). Shadow má nenulový raw demand.
Toto **není** důkaz bitové shody scheduleru, transientů nebo létající soustavy.
Ve scénáři steady RCOU rozdíly nevznikly; odlišné boot/arming časy se
nepovažují za společný logický sample. Neproběhl real-world motor test.

Finální build/metadata/timing evidence je v oddílu 24.5.
Hardware a skutečný let zůstávají **NEOVĚŘENO**.

### 24.4 Expo ověřené proti skutečnému buildu

Izolovaný experiment s běžným g++ původně ukázal jinou hraniční větev,
protože v něm nebyly AP compiler flags. To nebyl rozpor skutečného firmware.
`Tools/ardupilotwaf/boards.py` a compile_commands potvrzují
`-fsingle-precision-constant`: `0.95` i parametr `0.95f` jsou v AP porovnání
stejné float hodnoty. Přímý unit test existujícího `input_expo()` potvrdil
`input=0.5,expo=0.95 -> 0.5`. Schválená mapa/hranice ani AP_Math se nemění.
Žádné rozhodnutí nebo povolení k odchylce není potřeba.


### 24.5 Finální build, metadata a pracovní strom

Všechny čtyři varianty prošly. Zachované binaries/APJ/HEX jsou v
`tmp/utb-phase1-artifacts/`; SHA-256 a logy eviduje
`tmp/utb-phase1-build-results.json`. Velikosti jsou přímo z Waf summary,
BSS není celková runtime RAM spotřeba. U SITL je „flash“ pouze host binary
text+data metrika, nikoli flash H743.

| Platforma | AP_UTB | Build | Total Flash Used B | BSS B | Log v tmp/ |
| --- | --- | --- | --- | --- | --- |
| SkystarsH7HD-bdshot | ON | PASS | 1243268 | 127768 | utb-phase1-hw-on-build-final.log |
| SkystarsH7HD-bdshot | OFF | PASS | 1233328 | 125704 | utb-phase1-hw-off-build-verified.log |
| SITL Copter | ON | PASS | 4632109 | 226144 | utb-phase1-sitl-on-build-verified.log |
| SITL Copter | OFF | PASS | 4610277 | 224096 | utb-phase1-sitl-off-build.log |

Cílový HW ON–OFF rozdíl je 9940 B text+data a 2064 B BSS. SITL `nm -C`
potvrdilo AP_UTB::update přítomné v ON a nepřítomné v OFF. Pro tuto fázi
se nezměnil toolchain ani submoduly a nic se neinstalovalo.

Finální jednotkové spuštění: **26/26 PASS**, 8 suites, souhrn 2 ms;
`tmp/utb-phase1-unit-results-verified.log` a XML
`tmp/utb-phase1-unit-results.xml`. Doba unit suite není runtime benchmark.
`flake8`, AP_Param parser `--no-emit`, Copter logger metadata parser a
`git diff --check` prošly. Log parser rozpoznal všech pět UTB zpráv/IDs
bez kolize; 18 parametrů včetně původního ENABLE má fullname do 16 znaků.
`astyle` dry-run nových/rozšířených UTB souborů a kontrola změněných řádků
původních Copter/State souborů prošly; okolní původní styl nebyl přepsán.
Evidence: `tmp/utb-phase1-{lint,params,logger,diff,astyle}-verified.log`,
`tmp/utb-phase1-astyle-parts-verified.log`, metadata XML v
`tmp/utb-phase1-logger-metadata/` a motor/packet audit JSON.

Git HEAD zůstává `761a38d7042f5a6d91c7dc065043e9cb84b81486`, větev
`ardupilot-4.7.1-utb`. Patch má 17 modifikovaných a 6 nezařazených souborů
(včetně teorie v2, která byla nezařazená již před implementací). Nic není
staged/committed/pushed. Přesný manifest je v oddílu 19; ignorované `tmp/`
obsahuje lokální evidence a není součástí patche.

Schválené matematické bloky byly porovnány s předchozí verzí a nezměnily se.
PID, derivative-on-measurement, backward Euler, BF_X, common R/P/Y priority
ani SHADOW ONLY jednokrokový anti-windup nemají matematickou odchylku.
File layout a pevné doplňkové packets jsou zdokumentované organizační volby.
Po SHADOW FÁZI 1 práce končí; aktivní UTB_ACRO ani FÁZE 2 nezačaly.


### 24.6 Finální SITL log rate, alignment a dostupné časování

Finální script i následná FÁZE 0 regrese s AP_UTB ON skončily exit 0.
Výsledky: `tmp/utb-phase1-sitl-verified/results.json`, stdout
`tmp/utb-phase1-sitl-verified.log` a
`tmp/utb-phase1-skeleton-on-verified.log`. Compile-out regrese OFF prošla
na zachované OFF binary: `tmp/utb-phase1-skeleton-off.log`.

| Požadováno | Pozorováno z úplných sad | Sady | Délka prvního souvislého segmentu |
| --- | --- | --- | --- |
| 200 sad/s | 199.997822 Hz | 5272 | 26.355287 s |
| 400 sad/s | 399.347339 Hz | 3113 | 7.792715 s |

Celkem 36189 kompletních sad, 1 neúplná poslední sada a 561 záměrných
queue drops při řízeném vypnutí loggeru. Poslední neúplná sada
Seq=42062/TimeUS=109781903 má UTBR všech os, UTBM a UTBA, chybí UTBT;
je poslední UTB sada na konci ukončovaného SITL logu. Poslední dostupné
UTBS uvádějí Miss=0. To dokládá omezení enqueue versus persist při ukončení,
ne oprávnění neúplnou sadu použít; offline byla vyloučena. Evidence rozboru
je `tmp/utb-phase1-partial-analysis.json`. Dřívější plný běh měl 0 partial;
neprohlašujeme proto všechny zápisy za vždy doručené.

RC fixture ověřila RCMAP roll=RC2/pitch=RC1, RC2 reverse, DZ=100 PWM,
AP rate/expo mapu na 100 vzorcích každé osy. Očekávaná reference
[-pi/7,0,-pi/14]=[-0.4487989505,0,-0.2243994753] rad/s odpovídala
s tolerancí 1e-6. SYSID měl 2758 přesných APUS/PIDR.TimeUS párů a
max target difference 0 rad/s; filter targetu byl vypnut jen ve fixture.
Motor isolation měla 0 steady RCOU rozdílů v 20 vzorcích každé fáze/běhu,
1000 PWM disarmed a 1100 PWM armed ground idle, bez vzletu.

Měřené dt bylo 0.00249899994 až 0.00333199999 s. Deset PM záznamů:
LR=399–400 Hz, MaxT=3332 µs, NLon=4–5 za přibližně 4000 loopů,
Load=0. UTB shadow Ex mean/max=0 µs, logger LM max=0 µs. Tyto nulové
hodnoty **nejsou** důkaz nulové výpočetní práce: AP_HAL_SITL::micros64()
vrací v loopu scheduler stopped_clock_usec, tedy virtuální čas zamrzlý
během práce tasků. Dostupné PM/dt a log frekvence jsou simulované timing
a observability údaje. Skutečné wall-clock CPU maximum/distribuce ani H743
logger load se jimi neměří. TIME_BUDGET latch byl ověřen syntetickými unit
vstupy; jeho reálnou účinnost/threshold a backend latenci musí ověřit HW.

200 Hz zůstává požadovaný default, 400 Hz krátkodobý experiment.
Bez H743 měření není potvrzen žádný bezpečný hardware log rate limit.
Hardware, fyzický frame a let s vlastní UTB regulací zůstávají NEOVĚŘENO.


## 25. HARDENING REVIZE FÁZE 1 — stav 8. 10. 2026

Historické výsledky; aktuální geometrie a nové ověření jsou v oddílu26.

Oddíl 24 je historická evidence prvního patche. Jeho main-thread consumer
nevyhověl závěrečnému požadavku na absenci blocking logger cesty. Tato revize
jej nahrazuje; staré popisy main-thread fronty/FAST_TASK consumeru a staré
velikosti/statistiky nejsou popisem aktuálního transportu. Matematické bloky
PID, BF_X, reference a anti-windup zůstávají beze změny. Žádná FÁZE 2,
aktivní UTB_ACRO ani změna arming/motor/HAL/DShot nebyla provedena.

### 25.1 Existující AP pattern a priority

`AP_Logger::start_io_thread()` používá HAL thread_create, PRIORITY_IO,+1.
`ArduCopter/rate_thread.cpp` volá Log_Write_Rate/Log_Write_PIDS z workeru.
`AP_Logger_Backend::ShouldLog()` explicitně rozlišuje in_main_thread a
odmítá worker zápisy před dokončením startup messages. File/Block backend
chrání buffer semaphore, MAVLink backend používá take_nonblocking.
To dokládá místní konvenci pro statické WriteBlock z workeru; nejde o
univerzální thread-safety tvrzení o každé metodě AP_Logger.
Internals AP_Logger ani HAL nebyly změněny.

UTB worker vzniká přes thread_create pouze během vehicle startup s boot
ENABLE=1 a HAL_LOGGING_ENABLED. Stack request=3072 B; allocation při
thread startup je mimo flight producer. Selhání vytvoření nepanikuje a
ponechá LoggingAvailable=false; nový experiment zůstane suppressed.
Worker používá PRIORITY_IO,0: ChibiOS priorita 58, main=180, AP log_io=59.
SITL HAL vytváří native pthread a base priority nemapuje na real-time OS
priority; SITL není ověřením H743 priorit/stack rezervy či výkonu.

### 25.2 Producer a pevný transport

Producer = flight critical / non-blocking:
INS -> jediný původní AP rate controller -> AP motor output -> UTB shadow
-> read_AHRS -> ... -> update_flight_mode. Read-only target capture zůstává
před původním controller run. Producer nevolá logger ani GCS_SEND_TEXT,
nečeká na mutex/semaphore, nemá spin-wait a nealokuje heap.

Datová SPSC queue používá **existující AP ByteBuffer + ObjectBuffer**.
Úložiště je vložené do objektu: (CAPACITY+1)*sizeof(Snapshot), CAPACITY=4;
externí ByteBuffer/ObjectBuffer constructors nic nealokují. Head/tail mají
stávající AP std::atomic pořadí (implicitní sequential consistency), žádná
vlastní lock-free implementace. Compile-time assert vyžaduje lock-free
int/char/bool atomics. Jen main publikuje, jen UTB worker popuje.
Snapshot včetně časové metadata je hotový před commit/publikací; již
publikovaná položka se nikdy dodatečně neupravuje.

Status má samostatný pevný mailbox chráněný HAL take_nonblocking.
Contended mailbox znamená ponechat poslední status; nemění datovou sadu.
Plná datová fronta znamená DROP CURRENT SET + producer counter a okamžitý
návrat. PID/I/D/feedback se tím neresetuje. Status při full dostane
QUEUE_FULL a ShadowObservable=false. Log rate je frekvence požadované
publikace, controller nadále běží při každém podporovaném rate kroku.

Off/on reset neprovádí nebezpečné clear SPSC indexů ze dvou vláken.
Main zvýší atomic epoch; worker odmítne staré epoch/disabled experiment
po popu. Rozpracovaný backend zápis může při off doběhnout; žádný main
wait/flush/join není přidán. ENABLE je boot latch: při boot ENABLE=0
nevznikne worker, capture/PID/mixer/queue producer ani UTB data/status zápisy.
Guardované callbacky jen vracejí; identické firmware timing se netvrdí.

### 25.3 Consumer, feedback a counters

Consumer = diagnostic / lower priority / smí čekat na logger.
Každý cyklus vezme kopii statusu, obslouží nejvýše čtyři sady a sleep 1 ms.
Není nekonečný drain loop. Logger/GCS volání nemají status semaphore ani
jiný UTB queue lock. Backend může zablokovat worker; controller na něm
nečeká. Existující AP logger může mít vlastní sdílené locks/priority
inheritance vůči jiným AP logger uživatelům; revize neposkytuje absolutní
izolaci veškerého AP logování ani hard-real-time garanci všech backendů.

Worker nepřistupuje k živému _snapshot, _capture, PID ani AP_Param gains.
Dostává kopie; logging availability/result/counters/request/epoch mají
lock-free atomics. Main jediný vlastní control/capture historii. Selhání
backendu mění ObservationReason a ShadowObservable, nikoli matematiku.
Miss counter uchová i failure následovaný recovery mezi dvěma producer
cykly; failure tedy nezmizí jen rychlým přepsáním bool úspěchem.

Nový UTBQ ID=18, packet 46 B, format QIIIIIIBBIBI:
TimeUS,Seq,ReqN,EnqN,WrN,Drop,Miss,HWM,Depth,Epoch,Gy,Gone.
ReqN=requested publication; EnqN=queue accepted; WrN=complete set accepted
prvním backendem; Drop=producer full; Miss=at least one failed backend
write; Gone=consumer discarded old epoch/disabled sets. HWM je maximum
pozorované/reservované occupancy před publikací; Depth je aktuální
pozorovaná published occupancy. Counters jsou cumulative od bootu,
32bit wrap je možný. Atomics nejsou jeden globálně transakční snapshot;
čtení downstream->upstream zachová WrN<=EnqN<=ReqN bez wrapu.

UTBS/UTBR/UTBM/UTBA/UTBT ID 13–17 a jejich formáty jsou zachované.
UTBQ je přibližně 1 Hz; samotná rate sada zůstává 320 B. Written/enqueue
není důkaz persistence. Všechny sady mají TimeUS/Seq; offline parser
nekompletní sadu vyřadí. Consumer ani flight path nezavádí shutdown flush.

### 25.4 Primary gyro

State zaznamená skutečný AHRS primary gyro index a použije stejný index
pro INS health. Manager porovná index s minulým sample. Změna resetuje
I, previous measurement, D filter a saturation feedback; validní sample
pouze PRIME, ControllerValid/MixerValid=false, reason
PRIMARY_GYRO_CHANGED=11. Následující validní sample pokračuje, stejný
index nevyvolá další reset. Invalid frame/state/dt mají nadále vlastní
precedenci. Gyro change event se pokusí publikovat i mimo běžnou decimaci,
ale při overflow může být diagnostika ztracena stejně jako jiné sady.
AP AHRS/INS failover se nijak nemění.

### 25.5 Ověření

Finální výsledky této revize jsou uvedeny níže.
Dosavadní historické výsledky v oddílu 24 nejsou jejich náhradou.
Hardware/stack/CPU/logger budget a skutečný let zůstávají NEOVĚŘENO.


UNIT: **31/31 PASS**, 8 suites, 5 ms v tomto běhu (nikoli benchmark).
Původních 26 testů plus non-blocking semaphore denial, full queue s
reálnými production drop/request/enqueue counters a pokračujícím I,
backend failure včetně failure/recovery mezi cykly, primary gyro reset
všech historií bez D spike a concurrent coherent copies (20000 publikací).
Evidence `tmp/utb-hardening-unit-verified.log` a `utb-hardening-unit.xml`.
SITL sizeof Snapshot=336 B, Queue=2136 B; ChibiOS runtime stack/heap
rezerva nejsou tímto host sizeof měřením ověřeny.

SITL **ON PASS**: `tmp/utb-hardening-sitl-realtime/results.json`, stdout
`tmp/utb-hardening-sitl-realtime.log`. Source audit je také přímo součástí
regresního skriptu. Guards/frame18/class2/FSTRATE/config/reset/off-on,
logger unavailable start policy, runtime backlog/overflow/recovery,
oddělené health a všechny ordinary/force/RC arm/armed-entry zákazy prošly.
FÁZE 0 ON regrese také prošla (`tmp/utb-hardening-skeleton-on.log`).

| Requested publication | Measured complete persisted sets | Complete sets | Baseline interval |
| --- | --- | --- | --- |
| 200 Hz | 200.003738 Hz | 5244 | 26.214510 s |
| 400 Hz | 399.326397 Hz | 2396 | 5.997600 s |

SITL používá speedup=1. Celkově 24250 úplných sad, 1 neúplná sada a
958 producer drops včetně řízeného výpadku/backlogu. Offline parser
vyřadil nekompletní sadu; navíc umělé odstranění UTBT z úplné reálné sady
snížilo počet complete právě o jednu. Poslední UTBQ (není konec celého
experimentu): ReqN24982, EnqN24024, WrN24021, Drop958, Miss1, HWM4,
Depth1, Epoch1, Gy0, Gone1. Counters odpovídají popsané enqueue semantics,
ne potvrzení všech backendů/persistence. Startup worker write může být
odmítnut AP ShouldLog před dokončením startup messages, aniž selhal PID.

Předběžné speedup=3 běhy ukázaly host service/drop omezení a neprošly
původním testem prvního nepřerušeného segmentu. I při téměř 200Hz průměru
krátké startup mezery tento test přerušily. Finální měření zahrnuje všechny
mezery/ztráty ve baseline intervalu a testuje průměr kompletně doručených
sad, nikoli vybraný nejlepší segment. 1x bylo zvoleno, aby požadované
simulační Hz neznamenaly trojnásobný wall-clock load workeru. 400Hz úspěch
v tomto krátkém SITL intervalu nedokazuje dlouhodobý/H743 throughput.

Primary gyro fixture použila pouze disarmovaný AP EKF3 core switch
(EK3_IMU_MASK3, EK3_PRIMARY1 a zpět0); reálné indexy 0/1 a dvě reason11
prime/invalid události byly zaznamenány, další kroky healthy. UTB failover
neprovádí. SYSID simulated AP flight: 1876 exact PIDR.TimeUS=APUS matches,
max target difference=0 rad/s (target LPF vypnut pouze v testu).
RCMAP/reverse/DZ/reference fixture ověřila 100 samples/axis.
Shadow OFF/ON steady disarmed/armed-ground-idle RCOU: 0 rozdílů,
20 samples/phase/run; žádný důkaz bitové shody transientů či skutečného letu.

Producer dostupné dt=2.499–3.332 ms, Ex mean/max=0 µs, PM LR399–400,
MaxT3332 µs, Load0: jde o SITL stopped-clock údaje. Ex v publikovaném
snapshotu se měří před queue commit, proto nezahrnuje závěrečnou bounded
publikaci. Není to úplný wall-clock producer budget benchmark. Disabled
callback je zdrojově early-return; jeho skutečný H743 overhead nebyl měřen,
SITL virtual clock pro tak malé úseky není vhodný. Skutečný CPU/logger/
stack budget na H743 zůstává NEOVĚŘENO. Blocking backend může čekat pouze
UTB worker v rámci této nové cesty; původní AP logger uživatelé zůstávají
sdíleným systémem a jejich interní locks tato revize neodstraňuje.

### 25.6 Přesný hardening diff manifest

Změny proti stavu bezprostředně před touto hardening revizí (16 souborů):

| Soubor | Zásah |
| --- | --- |
| libraries/AP_UTB/AP_UTB.h | SPSC externí úložiště, status mailbox, atomic feedback/counters/epoch, gyro reason |
| libraries/AP_UTB/AP_UTB.cpp | Transport publikace, counters, oddělený worker feedback a gyro history reset |
| libraries/AP_UTB/AP_UTB_State.h | Primary gyro index v read-only state |
| libraries/AP_UTB/AP_UTB_State.cpp | Index z AHRS a health stejného gyra |
| libraries/AP_UTB/tests/test_utb.cpp | Pět dalších hardening testů, původní testy zachované |
| ArduCopter/utb.cpp | Producer bez logger/GCS; startup HAL worker, bounded drain, copied status a UTBQ |
| ArduCopter/utb_log.h | UTBQ static packet/metadata, původní log formáty nezměněny |
| ArduCopter/Copter.cpp | Odstranění FAST_TASK loggeru a scheduled GCS statusu |
| ArduCopter/Copter.h | Worker/init a copied-status callback declarations |
| ArduCopter/system.cpp | Vytvoření workeru po init UTB během startup |
| ArduCopter/defines.h | Append UTBQ ID18, ostatní IDs nezměněny |
| ArduCopter/mode_utb_acro.cpp | Odstranění GCS zprávy při vstupu kvůli nepřímému main logger zápisu; guards nezměněny |
| Tools/autotest/test_utb_shadow.py | Worker/source audit, pipeline counters, primary gyro switch, truncated parser, 1x rate measurement |
| docs/UTB_CONTROL_THEORY_CS.md | Aktuální architektura, source patterns a hardening evidence |
| docs/UTB_ARCHITEKTURA_CS.md | Oddělení historického consumeru od nové architektury |
| README_BUILD_WSL_CS.md | Nový transport, testy a evidence, žádné instalace |

Matematické zdrojové soubory RateController/MotorMixer/Reference jsou vůči
předchozímu patche byte-for-byte nezměněny. Schválené math blocks také.
Exact local incremental patch/manifest je v ignorovaném `tmp/`;
původní git HEAD se nezměnil a žádný commit/push nebyl proveden.


### 25.7 Finální build/audit stav: SHADOW FÁZE 1 COMPLETE

| Platforma | AP_UTB | Build | Text+data B | BSS B |
| --- | --- | --- | --- | --- |
| SkystarsH7HD-bdshot | ON | PASS | 1244144 | 128580 |
| SkystarsH7HD-bdshot | OFF | PASS | 1233328 | 125704 |
| SITL Copter | ON | PASS | 4633773 | 227040 |
| SITL Copter | OFF | PASS | 4610277 | 224096 |

Firmware je v `tmp/utb-hardening-artifacts/`; logy a SHA-256 eviduje
`tmp/utb-hardening-build-results.json`. BSS není celková runtime RAM;
worker stack/startup allocations nejsou součástí této metriky. SITL
text+data není skutečná flash H743. FÁZE 0 OFF regrese prošla také
(`tmp/utb-hardening-skeleton-off.log`). Symbol audit potvrzuje worker
v ON a jeho nepřítomnost v OFF. Žádné nové instalace ani submodule změny.

Finální source audit: PASS pro absenci UTB backend/GCS call v produceru,
blocking wait/heap v produceru, dynamického post-publication snapshot
zápisu i UTB motor setters/output/spool/limits/HAL/DShot. Queue je bounded
SPSC na místní AP implementation; mailbox je try-lock; worker callbacky
nejsou v scheduleru. Arming guards jsou zachované; jediná úprava ModeUTBAcro
je odstranění informační GCS zprávy při init (nepřímé main logger volání).
Math zdrojové soubory i schválené math blocks nezměněny. PASS pro flake8,
astyle dotčených částí, AP_Param metadata, všech šest logger metadata/IDs
(13–18) a diff check. Explicitní testy/výsledky jsou v 25.5.

Eff je průběžný odhad z asynchronního written counteru a experiment času,
ne synchronně potvrzená persist frekvence. Při off/on může do counteru
po resetu doběhnout jedna předchozí rozpracovaná sada; přesná analýza
aktuálního experimentu musí použít TimeUS/Seq a decoded BIN. Cumulative
UTBQ counters tento odhad nenahrazují globálně transakčním snapshotem.

Git: větev `ardupilot-4.7.1-utb`, HEAD
`761a38d7042f5a6d91c7dc065043e9cb84b81486`. Pracovní strom celkem
20 modified + 6 untracked (kumulativní první patch, odstranění textových
zmínek a tato revize), nic staged/committed/pushed. Šest untracked souborů
je záměrných: ArduCopter/utb.cpp, ArduCopter/utb_log.h,
Tools/autotest/test_utb_shadow.py, docs/UTB_CONTROL_THEORY_CS.md a
libraries/AP_UTB/AP_UTB_Reference.cpp/.h. Exact hardening diff má 16 souborů
z 25.6; plný git status není totéž co incremental hardening manifest.

**NÁVRH:** schválená SHADOW matematika; další aktivní režimy jen návrh.
**IMPLEMENTOVÁNO / UNIT / SITL / BUILD OVĚŘENO:** SHADOW FÁZE 1 COMPLETE
v rozsahu této softwarové revize a uvedených testů/source auditu.
**HARDWARE / SKUTEČNÝ LET:** NEOVĚŘENO. Bench musí ověřit fyzický frame,
H743 priorities/stack/load, logger medium/backend throughput/jitter a
nejvyšší bezpečně udržitelný log rate. Po této revizi se práce zastavuje;
aktivní UTB_ACRO ani FÁZE 2 nebyly zahájeny.

## 26. SHADOW BF_X_REV — větev test/skystars-5inch (10. 10. 2026)

IMPLEMENTOVÁNO: pouze class1/type12 BF_X a class1/type18 BF_X_REV. Jediný allocator vybírá yaw_sign=+1 pro12, −1 pro18; původní BF_X aritmetické pořadí zůstává stejné. Žádná závislost matematických tříd na desce.

```text
G12 = 0.5 * [ -1 -1 -1 ]    G18 = 0.5 * [ -1 -1 +1 ]
            [ -1 +1 +1 ]                [ -1 +1 -1 ]
            [ +1 -1 +1 ]                [ +1 -1 -1 ]
            [ +1 +1 -1 ]                [ +1 +1 +1 ]
```

Literatura: obecné principy alokace a anti-windup dle oddílu22. ArduPilot source convention: quad MotorDef, CW=−1/CCW=+1, logical pořadí a normalise_rpy_factors v AP_MotorsMatrix. Engineering decision: explicitní whitelist12/18, společný diagnostický allocator a epoch změny geometrie. Nejde o změnu fyzikálních rovnic PID.

G18=G12·diag(1,1,−1), GᵀG=I, Gᵀ1=0. Po desaturaci m=T_shift·1+scale·G·c, takže achieved=Gᵀm=scale·c a achieved_thrust=sum(m)/4. Konkrétně Y12=(−m1+m2+m3−m4)/2, Y18=(m1−m2−m3+m4)/2. Collective shift neovlivní momenty. Numerické clamp tolerance zůstávají původní.

Priorita a saturace zůstávají: společný poměr R/P/Y, společný scale při span>1, posun collective a bounds[0,1]; yaw nemá nižší prioritu. SHADOW ONLY anti-windup dostává achieved v téže tělesové soustavě jako raw PID, včetně správného znaménka yaw18. PID, D filtr, reference, anti-windup metoda a Ki=0 default se nemění.

Změna class/type za běhu resetuje všechny I/D/previous/residual historie a první validní cyklus je PRIMING (neplatný controller/mixer). Nově se zvýší diagnostická epoch; staré queued snapshoty si zachovají starou epoch a worker je podle existujícího filtru může zahodit. In-flight write již zahájený workerem může dokončit starou epoch; nepředstavuje zpětnou vazbu nové geometrii ani motorový output. UBEP umožní offline rozlišení epoch při BENCH ON. Snapshot a saturation flags jsou v každém cyklu sestaveny znovu. Změna geometrie neodblokuje TIME_BUDGET latch a neobchází jinou ochranu. FRAME_MISMATCH stále platí pro všechny nepodporované kombinace.

Obě geometrie jsou dostupné pouze pro SHADOW. Žádné změny AP_Motors, HAL/DShot, motor setters, arming, EKF ani FSTRATE. Motorové výstupy nadále zapisuje původní AP. Referenční FRAME_TYPE=18 zůstává zachován.

UNIT/SITL/BUILD evidence a přesný manifest jsou v [reportu rozšíření](UTB_BFX_REV_REPORT_CS.md). Starší oddíly23–25 zachovávají výsledky tehdejší implementace; odmítnutí18 v nich není aktuální podporou této větve. HARDWARE NEOVĚŘENO: timing, CPU/logger load, mutex latence, stack rezerva, udržitelnost200/400Hz, fyzické směry/mapping. Aktivní UTB ani FÁZE2 nejsou implementovány.
