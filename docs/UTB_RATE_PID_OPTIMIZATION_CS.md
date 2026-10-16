# FÁZE 2B: optimalizace skutečného SITL UTB Rate PID

Stav: SOURCE/UNIT a nominální opakované **ACTUAL ARDUPILOT SITL VERIFIED**.
Gains jsou pouze pro testovaný model, ne pro fyzický 5″ dron. Navazující
robustnost a aktivní saturace mají samostatnou výsledkovou část níže.

Nové důkazy: `tmp/utb-2b-20261014_164442`. Branch `test/skystars-5inch`,
HEAD `687f825d228b12a64d36651072aed19356178217`, bez nového commitu.
Checkpoint obsahuje patch i tar všech 39 tehdejších changed/untracked source
souborů a jejich SHA-256. Historické reporty a měření zůstávají zachované.

## Prostředí, regulator a reference

Jde o původní ArduPilot SITL, ne offline plant vydávaný za autopilot.
Mode41 UTB_RATE_SITL, FRAME_CLASS=1/FRAME_TYPE=18, model bfxrev,
400Hz scheduler, FSTRATE=0. UTB_S_OWNER=1 volá skutečný sdílený
AP_UTB_RateController; owner2 původní AP rate PID. Při owner1 platí
APN=0, UN=MN=Seq; při owner2 UN=0, APN=MN=Seq. Jen jeden původní
motor output, žádný UTB diagnostický mixer v aktivní cestě.

Každá session má nový proces a vlastní params/logs. Původní AP GUIDED
provede standardní arming a takeoff do 10 m; experiment začíná po gyro
<0,03 rad/s a roll/pitch <3°, potom 2 s v mode41 s nulovou referencí.
LAND a disarm jsou explicitní cleanup mimo measurement window.
Mode41 nemá vlastní attitude/altitude/position controller.

Normal profiles 1/2/3: 1 s zero, 2 s +0,15 rad/s, 1 s zero,
2 s −0,15 rad/s, 2 s zero. Mode update referenci připravuje až pro následující
fast cycle; tento časový vztah je stejný pro oba owners. Shared AP
`input_rate_bf_roll_pitch_yaw_2` shaping zůstává stejný. IAE/ITAE používají
skutečný shaped Tar, step metrics surový constant plateau Ref.

Po AP přípravě jsou pro obě varianty FLTT/FLTE/FF nulové; PARM také potvrzuje
D_FF=0, SMAX=0, PDMX=0. AP R/P gains 0,135/0,135/0,0036, yaw
0,3/0,02/0; IMAX=0,5. Toto je kontrolované PID-only srovnání v experimentálním
shellu, nikoli porovnání kompletního sériového AP letového režimu.

Host speedup=2 je stejné pro všechny 27 repeat sessions. Virtual 400Hz perioda
není H743 wall timing. Rozšířený logger při speedup5 vytvořil neúplné sady;
archivovaná `preliminary/zero` je FAIL evidence. Nebyly doplňované ani
interpolované chybějící samples. `preliminary-slow` prošel 3x3200 sad.

## Zdrojově a experimentálně doložená příčina

Shared PID: P=Kp*(Tar−gyro), I+=Ki*error*dt s IMAX a directional gate,
D=−Kd*filtered((gyro−gyro_prev)/dt). Backward-Euler D filtr má
beta=dt/(1/(2π*fD)+dt), při 400Hz/20Hz beta≈0,2391. Změna reference
nevytváří přímý UTB derivative kick.

`libraries/AC_PID/AC_PID.cpp:259` naopak derivuje error a filtruje tento
rozdíl. AP D proto při rostoucím targetu obsahuje target derivative, i s FF=0.
Ve FÁZI 2A CSV při roll t≈1,1 s je UTB D≈−0,00154, AP D≈+0,00252
normalized units. Opačný transientní příspěvek je přímo v signálech;
nelze jej zaměnit za odlišné shared reference nebo druhý AP controller.

Roll D-only 0,002→0,001 při starém P/I zkrátil kladný rise na 69,97 ms.
Roll P-only 0,155→0,22 při starém I/D dosáhl 62,47 ms, ale overshoot
2,45 %. Vybraná společná sada dává odlišný kompromis celkové chyby,
překmitu a úsilí. Integrátory R/P jsou řádově 0,0006–0,0008, tedy daleko
od IMAX=0,19; clamp pomalý transient nevysvětluje. D filtr se testuje
oddělenými cutoff 15/20/30Hz při stejných zbývajících gains.

### Yaw

Znaménko targetu, gyro i motor projection je shodné v obou směrech.
Nová capture měří skutečný AP normalized allocated request. V nominálních
yaw bězích nebyl yaw limit flag, normalized residual >1e−5 nebo command
clipping. Reference po shaping dosáhne plateau, integrátor ve staré 2s
sekvenci max≈0,0133 <0,19. Není prokázaný yaw-headroom/throttle/IMAX cap.

Původní yaw s 8s plateau (profile7, 8000 skutečných sad) dosáhl kladně
98,78 % targetu, rise 3,793 s a 2% settling 6,886 s. Záporně dosáhl
97,11 %, rise 4,467 s, settling null. Pouhé prodloužení sekvence tedy
vyvrací pevný 78% ceiling daného allocatoru/modelu.

Yaw I-only 0,115→0,23 (P/D beze změny) dal ve 2s plateau 93,10 %,
rise 1,550 s a IAE 0,333855. D-only 0,002→0 dal 78,57 % a IAE 0,388981,
prakticky jako baseline. Dominantní zjištění je příliš pomalé integrační
dorovnání pro zvolenou yaw dynamiku a 2s plateau; D zde nebyl hlavní příčinou.
Není to tvrzení, že zvýšení I je univerzální oprava fyzického yaw.

`libraries/SITL/SIM_Motor.cpp:72` vytváří rotor torque z yaw_factor,
command a `0,05*diagonal_size*motor_thrust`; následně přidává moment od
ramene a motor momentum drag. Rotační rychlost mění lokální velocity motoru
a tím i drag torque (`SIM_Motor.cpp:59,111`). `SIM_Frame.cpp:695` dělí torque
momenty setrvačnosti a přidává rotační damping. Default model mass=3 kg,
diagonal_size=0,35, Jx=Jy=0,02296875 a Jz=0,0459375 kg*m² podle lokální
fallback konvence. To není identifikace Skystars 5″. Literal refRotRate=120
se použije přímo v radians(400)/120≈0,05818 s⁻¹; není důvod jej samostatně
převádět jako deg/s. Momentum drag je další příspěvek, nelze vše vysvětlit
jen tímto konstantním dampingem.

Pro lokální model J*omega_dot+beta*omega=k*u s PI má charakteristický
polynom J*s²+(beta+k*Kp)*s+k*Ki. Malé Ki může ponechat pomalý closed-loop
pole i bez saturace. To je vysvětlující aproximace/engineering analysis,
nikoli naměřená fyzická identifikace k/beta. Závěr výše stojí na skutečném
long-plateau a I-only experimentu, ne na vymyšleném torque measurement.

## Nominální optimalizace před testem zpoždění

Původní úspěšná sada všech os: P=0,155, I=0,115, D=0,002, IMAX=0,19,
D_HZ=20. Deterministický konečný seznam 5 roll +5 pitch +7 yaw kandidátů,
bez adaptivního posouvání k extrémům. Bounds P 0,1–0,3, I 0,02–0,6,
D 0–0,002, IMAX 0,05–0,19, cutoff 10–30. Všech 17 actual SITL kandidátů
prošlo valid state/dt, envelope, finite math a konečnou sekvenci.

Nominal objective:
J=0,40*RMS(error)/amp +0,25*IAE/(amp*duration)
+0,15*max(abs(tail_error))/amp +0,10*max(overshoot_fraction)
+0,10*RMS(normalized request).

Tail je posledních 0,5 s každého plateau. Rise 10–90 % je null, pokud 90 %
není dosaženo; settling vyžaduje poslední trvalou 2% oblast a alespoň 0,25 s
pozorování. Undershoot má vysokou RMS/IAE/tail penalizaci, nulový overshoot
jej neodměňuje. Stabilita, žádný fault, complete log a správná authority jsou
hard acceptance podmínky před použitím score. Robustnost je samostatná
acceptance kontrola; nebyl hledán globální optimum ani robustness optimum
ve všech kombinacích. Nominal sweep výsledky/score/alternativy jsou v
`nominal-sweep/results.json`, seznam v candidate-plan.json.

| Osa | P | I | D | IMAX | D_HZ |
|---|---:|---:|---:|---:|---:|
| Roll | 0,22 | 0,16 | 0,001 | 0,19 | 20 |
| Pitch | 0,19 | 0,16 | 0,001 | 0,19 | 20 |
| Yaw | 0,30 | 0,60 | 0 | 0,19 | 20 (D vypnutý) |

Parametrové defaulty nebyly změněné; tyto gains nastavuje jen SITL harness.
IMAX v nominální oblasti není aktivní omezení. Yaw s IMAX=0,10 také prošel;
0,19 proto není identifikované jediné optimum. Zůstává jednotná dosavadní
bounded kapacita a oddělený saturation stress; nelze z nesaturujících dat
vymyslet optimální IMAX. D_HZ při yaw D=0 nemá vliv.

Pareto příklady: rychlejší roll P-only má vyšší overshoot; yaw
0,25/0,45/0,001 má menší gains/effort, ale horší tracking než zvolený yaw.
Původní neúspěšná vysoká společná sada 0,31/0,23/0,004 nebyla obnovena.

## První nominální sada: opakovatelnost a srovnání AP

27/27 nezávislých sessions, každý 3200/3200 skutečných control sad a stejný
počet kompletních capture sad. Níže aritmetický průměr tří sessions,
kladný rise/overshoot/settling; null/mixed znamená, že ne ve všech bězích
bylo doloženo settling. Min/max/stdev a záporné plateau v repeat-aggregate.json.

| Osa | Varianta | IAE [rad] | RMS error [rad/s] | Rise [ms] | Overshoot [%] | Settling [s] |
|---|---|---:|---:|---:|---:|---:|
| roll | old | 0.026087 | 0.009730 | 74.14 | 1.044 | null/mixed |
| roll | new | 0.020573 | 0.007210 | 68.31 | 2.068 | null/mixed |
| roll | ap | 0.020196 | 0.006632 | 61.64 | 0.647 | 0.811 |
| pitch | old | 0.025395 | 0.009717 | 73.30 | 1.032 | 1.080 |
| pitch | new | 0.017291 | 0.007545 | 69.14 | 0.883 | 0.283 |
| pitch | ap | 0.020028 | 0.006619 | 59.98 | 1.011 | null/mixed |
| yaw | old | 0.390520 | 0.059720 | null | 0.000 | null/mixed |
| yaw | new | 0.146307 | 0.036060 | 514.79 | 0.066 | 0.965 |
| yaw | ap | 0.262874 | 0.044014 | null | 0.000 | null/mixed |

Mean IAE proti starému UTB: roll −21,14 %, pitch −31,91 %, yaw −62,54 %.
Proti AP: roll +1,87 % (AP nižší), pitch −13,67 %, yaw −44,34 %.
AP má rychlejší roll/pitch náběh. Nový roll má kladný overshoot 1,77–2,26 %
a jeho striktní settling není pro všechny běhy doložený. Tyto vlastnosti
nejsou skryté označením PASS. Yaw nový kladný 2% settling průměr 0,965 s,
kladný overshoot průměr 0,066 %. Old/AP yaw rise a settling v 2s okně null.

Raw/shaped reference porovnané po generator samples: limity 1e−7/1e−5 rad/s,
relative virtual clock phase tolerance 2,5 ms. Všech devět triplets prošlo.
Max initial gyro difference new/AP 0,002065 rad/s; startup tolerance <0,03
rad/s a <3° je součást protokolu. Nejde o identické stochastic initial states.
I při SIM_GYR*_RND=0 existuje INS minimum radians(0,04)*rand_float,
takže výsledky jednotlivých sessions mají měřitelný rozptyl.

![Yaw – actual SITL](../tmp/utb-2b-20261014_164442/plots-final/yaw-comparison.png)
![Roll – actual SITL](../tmp/utb-2b-20261014_164442/plots-final/roll-comparison.png)
![Pitch – actual SITL](../tmp/utb-2b-20261014_164442/plots-final/pitch-comparison.png)

Další PNG/SVG pro každou osu obsahují skutečné P/I/D a request; skript
`make_plots.py` je uložený s důkazy. Grafy jsou scientific exports ze všech
stored samples, nikoli ilustrační syntetické odezvy.

## Meze dosavadního závěru

Jen malé body-rate reference do explicitního envelope, volný attitude drift,
manual inherited hover throttle, tento konkrétní model a scheduler. Úspěch
neprokazuje fyzickou stabilitu, ESC odezvu, H743 closed loop ani univerzální
robustnost nad 2,5ms dodatečný delay. Pro další attitude SITL návrh je třeba
zachovat bounded rate requests a tyto guards. Hardware gains nejsou změněné.

## Audit počátečního hoveru

Všech 27 nominal sessions má nejbližší skutečnou GLOBAL_POSITION_INT
relative altitude 9.996–10.001 m,
vz=0 v rozlišení 0,01 m/s. Max počáteční tilt z ATTITUDE
0.275°. Největší rozdíl telemetry timestampu
od začátku okna 20.569 ms; nejde
o přesný simultaneous attitude sample. Collective z 400Hz capture
Σ(T/G)/4 je 0.363924–0.364226.
Úplné values a časová vzdálenost jsou v initial-state-audit/results.json.

Cutoff-only probe při ostatních vybraných gains: roll J15=0,028936,
J30=0,025793; pitch J15=0,026355, J30=0,025796. J20 repeat mean a
rozptyl jsou v aggregate. 30 Hz je další nominal Pareto kandidát; nemá
tři opakování/robustnost ani noise test se změněným cutoff. Vybraná
20Hz sada má širší ověření. Hodnoty nejsou globální statisticky prokázané
optimum cutoff, a yaw s D=0 cutoff žádnou optimalizaci nepotřebuje.

## Finální sada s ohledem na zpoždění

Po zjištěné citlivosti původního nominal roll vítěze byl před novými
experimenty uložen objective Jbalanced=0,75*Jnominal+0,25*Jdelay1
v pareto-objective.json. Nejde o fyzický safety limit. Osm nových bounded
actual SITL běhů porovnalo D15=0,19/0,16/0,0015/cutoff15 a
D20=0,19/0,16/0,002/cutoff20 pro R/P, s/bez 2,5ms adapter delay.

Roll balanced scores: nominal vítěz 0,033292; D15 0,031257;
D20 0,032249. D15 má horší nominal effort/tracking kompromis než původní
nominal vítěz, ale lepší delayed odezvu. Pitch scores: 0,028206 vs
0,031748/0,032034; původní vybraný pitch proto zůstal. Výběr je
engineering decision z předem definovaného objective, ne globální optimum.

| Osa | P | I | D | IMAX | D_HZ |
|---|---:|---:|---:|---:|---:|
| Roll | 0,19 | 0,16 | 0,0015 | 0,19 | 15 |
| Pitch | 0,19 | 0,16 | 0,001 | 0,19 | 20 |
| Yaw | 0,30 | 0,60 | 0 | 0,19 | 20 (neaktivní D) |

Tato finální kombinace dokončila dalších devět nezávislých sessions:
tři pro každou osu, všechny 3200/3200 sad s APN=0, UN=MN=Seq.
Metadata obsahují skutečné gains všech tří os. Referenční alignment
každého běhu proti odpovídajícímu old/AP repeat prošel stejnými tolerancemi.

| Osa | Finální IAE [rad] | Rise+ [ms] | Overshoot+ [%] | IAE proti 2A | IAE proti AP |
|---|---:|---:|---:|---:|---:|
| roll | 0.021854 | 69.14 | 1.716 | 16.23 % nižší | 8.21 % vyšší |
| pitch | 0.018493 | 69.14 | 0.906 | 27.18 % nižší | 7.67 % nižší |
| yaw | 0.146432 | 515.63 | 0.022 | 62.50 % nižší | 44.30 % nižší |

První tabulka/report mean −21,14/−31,91/−62,54 % patří předchozí
nominal kombinaci s roll P=0,22; není to finální s ohledem na delay.
Finální mean/min/max/stdev, oba směry a tail jsou v final-aggregate.json;
alignment v final-alignment.json, actual traces v repeat-final. AP nadále
nabíhá rychleji na R/P a má nižší roll IAE. Optimized yaw snižuje
steady error a délku transientu proti oběma kontrolovaným baseline.

## Robustnost skutečně ověřeného rozsahu

První nominal sada: 22 complete bounded runs (4 robust +18 robust-rest).
Finální sada: dalších 12 complete targeted runs. Inertia ±10 % na všech
osách, propExpo 0,65→0,715, motor slew150→100, explicitní 1-cycle delay,
kombinované R/P/Y, sine, sign reversal a combined gyro noise byly otestované.
Doplňkový modelový stress slew0,5 prošel. Slew0,1 selhal při původním AP
takeoffu **před mode41**, má failure.json a není vyhodnocený jako UTB
closed-loop test. Žádná arming/health ochrana pro něj nebyla obejita.

Slew100 je rychlý modelový limit při malých signálech; jeho PASS sám
neprokazuje motor-lag margin. Slew0,5 je výrazný diagnostický stress, ne
kalibrace ESC. Gyro SIM_GYR*_RND=0,5 je v deg/s, v INS se násobí throttle
a převádí do rad/s, navíc zůstává minimum0,04deg/s. Nesprávně by bylo
hodnotu interpretovat jako 0,5 rad/s.

Jitter ±20 % byl testován izolovaným C++ helperem se stále platnými senzory
a feedbackem; skutečné SITL traces obsahují vlastní virtual clock period
variace. Není to fyzické H743 CPU jitter měření ani vynucená skutečná
400Hz perioda s ±20% jitterem v autopilot procesu. Timestamp/gyro/primary
invalidity jsou v UNIT a změny config/frame/mode/FSTRATE v integračních
guard regresích; ověřené vrstvy jsou odlišené v summary.

Finální targeted výsledky (pro combined/sine/sign je tabulka roll axis):

| Případ | IAE [rad] | Max signed plateau overshoot [%] | Allocation-limit cycles |
|---|---:|---:|---:|
| roll-inertia-plus | 0.020195 | 1.770 | 0 |
| roll-inertia-minus | 0.021664 | 2.518 | 0 |
| roll-motor-curve | 0.020313 | 1.833 | 0 |
| roll-motor-slew | 0.022536 | 1.507 | 0 |
| roll-motor-slew-05 | 0.023740 | 2.934 | 0 |
| roll-delay | 0.037836 | 9.071 | 0 |
| pitch-delay | 0.024870 | 7.611 | 0 |
| yaw-delay | 0.146432 | 0.000 | 0 |
| combined | 0.021502 | 1.953 | 0 |
| sine | 0.034411 | 0.000 | 0 |
| sign | 0.040583 | 2.123 | 0 |
| combined-noise | 0.025944 | 2.728 | 0 |

Přidané zpoždění není margin guarantee pro libovolnou latenci. R/P
zůstávají citlivější než yaw; finite bounded/stable completion není
totožné s ideálním transientem. Pole, gain/phase margin a velké attitude
reference nebyly změřené. Pro další attitude návrh použít explicitní
SITL-only scope a bounded reference; současná data neautorizují fyzický let.
