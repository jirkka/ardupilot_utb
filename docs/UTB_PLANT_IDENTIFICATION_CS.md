# FÁZE 1H-E: identifikace rotační soustavy současného 5″ dronu

Stav k 13. 10. 2026: **SOFTWARE VERIFIED** pro import, rozšíření původního modelu,
výpočty a offline testy. **UNVERIFIED** pro fyzikální parametry a validaci modelu
skutečného Skystars. Není provedeno měření pohonu, motorový test ani let.

## 1. Existující model a nové rozšíření

[Model FÁZE 1H-D](UTB_CLOSED_LOOP_MODEL_CS.md) a
`Tools/autotest/utb_closed_loop.py` zůstávají základem. PID a allocator jsou stále
převzaty ze stejného `utb_math_reference.py`; jejich matematika se nemění.
Nový simulátor od nuly nevznikl. Produkční C++ UTB ani ArduPilot nebyl změněn.

Původní fyzikální baseline je **ilustrativní**:
J=(0,002; 0,0025; 0,004) kg·m², radiální motorové rameno 0,11 m, max tah 3 N na
motor, reakční poměr 0,015 m, motorový lag 0,03 s, D=(0,003;0,003;0,003)
N·m·s/rad, collective 0,5. Žádná z těchto fyzikálních hodnot nebyla změřena na
současném 5″ dronu. PID/reference/frame convention mají source/hardware
kontinuitu projektu, ale nejsou důkazem těchto fyzikálních parametrů.

Původní model: rigid-body rotační ODE, diagonální J, gyroskopická vazba,
viskózní tlumení, čtyři první řády motorů, normalizovaný actuator stav q,
F=Fmax·q^γ, yaw Q=kQ·F, pravidelná X geometrie. Saturace je zvlášť v allocatoru
a zvlášť jako strop modelového pohonu. Gyro je body rate se zpožděním,
volitelným gaussovským šumem a float32 převodem; nejsou simulovány AHRS/EKF,
aliasing, celé INS filtry nebo vibrace. Explicitní sensor/compute/actuator delay
je fronta celých 400Hz cyklů. RK4 používá substeps a omezuje krok podle lag.

Nově volitelné, bez změny default výsledků:

- Celá symetrická 3×3 matice J a její inverse; kladná definitnost a fyzikální
  realizovatelnost přes Q=(tr(J)/2)I−J ≥0.
- Čtyři FRD polohy motorů vůči těžišti, případně převod z body origin.
- Individuální monotónní tahové a reakční momentové piecewise linear křivky.
- Čtyři motorové τ, samostatné napěťové exponenty tahu a reakčního momentu.
- Import SI hodnot s původem a standardní nejistotou; původní baseline zůstává
  explicitně dostupná přes `Model()`.

Generální trim linearizace/póly dosud implementovány nejsou. `poles()` pro
rozšířenou geometrii/tensor/křivky/per-motor lag hodnotu odmítne. Původní
single-axis poles se nesmí automaticky označit za póly kalibrovaného dronu.
Celkem 24 původních CSV a PID replay vektorů zůstalo bajtově shodných s 1H-D.

## 2. Rovnice, osy a platnost

Body FRD: x dopředu, y doprava, z dolů. Rotační rovnice:

\[
J\dot\omega=\tau-D\omega-\omega\times(J\omega),\qquad
E_k=\tfrac12\omega^TJ\omega.
\]

Tah rovnoběžného motoru míří −z. Pro polohu r_i=(x_i,y_i,z_i):

\[
\tau_i=r_i\times(0,0,-F_i)+ (0,0,s_iQ_i)
       =(-y_iF_i,\ x_iF_i,\ s_iQ_i).
\]

BF_X_REV pořadí modelu: M1 zadní pravý, M2 přední pravý, M3 zadní levý,
M4 přední levý; znaménka reakce s=(+1,−1,−1,+1) jsou převzata ze source
convention současného projektu. Při identifikaci fyzicky ověřit polohu,
pořadí a směr každého motoru; fotografie není povolením motor testu přes FC.
z motoru se při paralelním tahu v roll/pitch momentu neuplatní; při náklonu
osy motoru by bylo nutné rozšíření vektoru tahu, nyní **UNVERIFIED / nepodporováno**.

\[
\dot q_i=(u_i-q_i)/\tau_i,\quad
0\le u_i\le u_{i,max},\quad
F_i=F_{i,ref}(q_i)(V/V_{ref})^{\beta_F},\quad
Q_i=Q_{i,ref}(q_i)(V/V_{ref})^{\beta_Q}.
\]

q je normalizovaný modelový stav pohonu, **není automaticky PWM, DShot ani RPM**.
Naměřená křivka musí mít doloženou mapu příkazu/stavu na q; exponent β=2 je jen
demonstrační předpoklad. Lag se vztahuje k definovanému q, ne bez rozmyslu k tahu
a RPM současně. Fitting z dat musí používat totožnou definici stavu.
Napětí je v jednom běhu konstantní; sag, proud, teplota, ground effect,
motorová gyroskopie, pružnost rámu a aerodynamická interference nejsou modelovány.
Model nemá polohu/altitude ani translaci. Hmotnost se eviduje pro měření,
CG a setrvačnost; do samotné rotační ODE nevstupuje samostatně.

## 3. Inventura neznámých a způsob jejich získání

A = přímé fyzické měření; B = geometrie/CAD; C = provozní logy;
D = literatura/výrobce s omezením přenositelnosti; E = dosud neznámé.
Status současných fyzikálních údajů v tomto dokumentu je E; demo není A.

| Parametr / jednotka | Preferované získání | Co lze bez motorů / co chybí |
|---|---|---|
| Konfigurace, m [kg] | A váha, fotografie a inventura | Celý konkrétní stroj včetně baterie, kamery, kabelů a vrtulí evidovaných zvlášť |
| CG [m], orientace body os | A reakční váhy / zavěšení, B CAD | Staticky bez napájení; z CG se odvozují r_i |
| Jxx,Jyy,Jzz a případné Jxy,Jxz,Jyz [kg·m²] | A bifilár/torzní rig; B komponentní CAD | Bez motorů; produkty vyžadují nakloněné osy / další informace |
| r_i [m], spin a motor order | A měřidlo/fotografie, B CAD | Bez motorů, ověřit vůči skutečnému CG a FRD |
| F_i(q,V), maximum [N] | A samostatný zajištěný thrust stand; D orientační výrobce | Ze SHADOW DISARMED logů nezjistitelné; motor/ESC/vrtule musí být shodné |
| Q_i(q,V) [N·m] | A reakční load cell / rameno standu; D hrubý prior | Neodvozovat přesně jen z tahu nebo KV; yaw gain zvlášť |
| τ_i [s], actuator delay [s] | A společný trigger/RPM/tah na standu | Motor/ESC/prop celek, několik pracovních bodů, repeat; logy C jen s identifikovaným vstupem a time alignment |
| sensor a computation delay [s] | C časování INS/controller; A externí reference | Statický log neobsahuje dostatečnou rotační excitaci; část delay je source convention, fyzická latence zůstává E |
| D_a [N·m·s/rad] | A řízený mechanický decay rig / pozdější identifikace C | Torzní fixture damping není automaticky aerodynamické tlumení dronu při rotorech |
| Saturace/deadzone, q mapping [1] | A stand, D ESC specifikace | AP motor output není fyzikální tahová kalibrace |
| Napětí V,Vref [V], βF,βQ [1] | A voltmetr + stand na více V | V lze staticky; exponenty a sag pod zátěží E bez pohonu |
| Gyro noise [rad/s], vibrace/filtry | C stacionární gyro log a source; A rotační rig | Klidový šum lze bez motorů, motorové vibrace nelze extrapolovat |
| Prop typ, průměr, pitch, hmotnost | A evidence/váha; D specifikace | 3″ předchozí stroj není současný 5″; výměna prop vyžaduje nové F/Q/lag |

Každá konfigurace má samostatné ID: rám, baterie (typ, skutečná hmotnost,
upevnění a poloha), kamera, VTX, přijímač, kabely, šrouby, motor/ESC/prop, datum.
Přesun baterie mění CG i J. Historický 3″ model nebo úhlopříčka rámu samy
neurčují setrvačnost současného 5″ stroje.

## 4. Hmotnost, těžiště a komponentní model

Vážit celý nenapájený stroj a části kalibrovanou váhou; zaznamenat rozlišení,
kalibraci a repeatability. Pro CG položit pevný rig na nejméně tři známé body:

\[
x_{CG}=\frac{\sum R_ix_i}{\sum R_i},\qquad
y_{CG}=\frac{\sum R_iy_i}{\sum R_i}.
\]

R_i jsou svislé reakční síly; všechny v N nebo všechny hmotnostní ekvivalenty
ve stejné jednotce. Odečíst vlastní rig a ověřit, že žádná část neleží mimo
váhy. z_CG určit druhou orientací nebo průsečíkem olovnic při zavěšení z více
bodů. Nezaměnit body origin za CG. Opakovat po přemontování a změně baterie.
Nejistotu polohy i reakcí propagovat; při R součtu ve jmenovateli jsou vstupy
korelované. Použít Jacobian nebo bootstrap jednotlivých nezávislých celých
měření, nikoli jen chybu jednoho dílčího odečtu.

CAD/komponentní součet, s d_i=c_i−c a rotation R_i součásti do FRD:

\[
m=\sum m_i,\quad c=\frac{\sum m_ic_i}{m},\quad
J=\sum\left[R_iJ_{i,local}R_i^T+m_i(\|d_i\|^2I-d_id_i^T)\right].
\]

`cad_mass_properties()` přijímá lokální tensory **již otočené do body FRD**,
vrací m, CG a J s parallel-axis členem. Neaproximuje automaticky součást jako
bod. Pro jednoduché kvádry použít např. Jxx=m(b²+c²)/12; kabely/nekonkrétní
density označit estimated. Váha reálného celku má přednost před CAD nominálem.
Při přesunu baterie znovu spočítat globální CG i všechny d_i, nepřidat pouze
m_battery·delta_distance² ke starému J. Monte Carlo CAD má zahrnout toleranci
poloh, hmotností, orientací a chybějících částí; není zde vydáváno za hotové
měření, protože inventura současného dronu dosud chybí.

## 5. Doporučené bifilární měření setrvačnosti

Vhodný univerzitní přípravek: pevná lehká plošina, dvě stejné dlouhé struny,
pevná horní příčka, měřené rozteče, optické značky, nezávislá kamera/časová báze.
Žádné napájení FC ani motorů není nutné. Vrtule demontovat. Baterii pro váhovou
konfiguraci ponechat mechanicky upevněnou a elektricky odpojenou, nebo použít
doložený dummy se stejnou hmotností, polohou a lokálním J.

Při demontáži vrtulí výsledek odpovídá konfiguraci **bez vrtulí**. Pro model
kompletního stroje přičíst samostatně zjištěné rigid-body J prop včetně parallel
axis; změnu CG přepočítat. Rotorové spin gyroskopické efekty tím nevzniknou.

Označení: m_d hmotnost dronu, m_f plošiny, a horní a b dolní **poloviční**
rozteč strun, h **svislá klidová výška** mezi závěsy, nikoli šikmá délka struny,
T_d tlumená perioda, ζ damping ratio, J_f setrvačnost přípravku kolem stejné osy,
d kolmá vzdálenost měřicí osy od CG dronu. Dvě stejně dlouhé struny a centrovaná
plošina; tilt/sway nesmí nahrazovat čistou torsion oscillation.

Pro malý twist θ geometrie zvedne plošinu přibližně o
Δh=ab·θ²/(2h). Potenciální energie je Mg·Δh, proto κ=Mgab/h,
M=m_d+m_f. Lineární rovnice je J_total·θ¨+c·θ˙+κθ=0:

\[
J_{total}=\frac{(m_d+m_f)gab\,T_d^2(1-\zeta^2)}{4\pi^2h},\qquad
J_{d,CG}=J_{total}-J_f-m_dd^2.
\]

Pro a=b a zanedbatelné tlumení jde o standardní bifilární vztah.
Tlumení odhadnout z obálky: δ=ln(A_k/A_{k+n})/n,
ζ=δ/sqrt(4π²+δ²). Pokud decay není exponenciální / perioda závisí na amplitudě,
model viscous correction není doložen. Rig se měří samostatně, ale jeho J_f
musí být pro stejnou rotační osu a orientaci; prázdná plošina může mít jinou
rovnovážnou výšku, tu měřit znovu. Velké odečítání J_f zvyšuje relativní nejistotu
malého dronu, proto lehký tuhý fixture a nezávislý referenční předmět.

**Praktický protokol, zatím navržený:**

1. Evidovat kompletní konfiguraci a CG, struny/příčku/rig, měřidla a fotografie.
2. Změřit m_d,m_f,a,b,h a d ve výsledné zatížené konfiguraci, před a po měření.
3. Kalibrovat fixture i časovou bázi referenčním objektem známého J; určit J_f.
4. Twist přibližně 3–5° a pustit bez impulzu; rozsah je volba malé amplitudy,
   ne bezpečnostní limit. Porovnat dvě amplitudy: při rozdílu periody mimo
   nejistotu zmenšit amplitudu a vyšetřit geometrii/frikci.
5. Zaznamenat nejméně 10 period jedním záběrem, nejméně 5 nezávislých spuštění
   pro každou osu. Po několika spuštěních rig přemontovat pro kontrolu systematiky.
6. T určit stejnosměrnými průchody nulou; camera frame rate a timestamp bias
   patří do nejistoty. `pendulum_period()` interpoluje crossings; společné
   intervaly sdílejí timestampy, nejsou nezávislá opakování.
7. Pro yaw je body z svisle; roll/pitch vyžadují připevnit stroj tak, aby jeho
   x/y osa byla svislou torzní osou. Každá orientace má vlastní fixture correction.
8. Spočítat J a u(J), ověřit reference, repeatability, amplitudový efekt,
   centrování a fyzikální realizovatelnost celé matice.
9. Opakovat pro relevantní baterie/polohy/kameru; nerecyklovat jediné číslo J
   mezi rozdílnými konfiguracemi.

Jxx/Jyy/Jzz měří pouze diagonalní projekce. Jestliže asymetrická baterie/kamera
může vytvářet produkty, změřit další známé osy n a řešit J_n=nᵀJn. Např.
n=(ex+ey)/√2 dává J_n=(Jxx+Jyy)/2+Jxy v **maticové** konvenci.
Některé CAD systémy reportují produkt ∫xy dm s opačným znaménkem; importovat
skutečnou matici J, nikoli bez kontroly názvů. Nakloněné osy měřit, ne jen
předpokládat nominální úhel děr v rig.

Expertní oporu pro oddělené měření fixture, orientaci os a produkty poskytuje
[Lorenzetti et al., Determining the Products of Inertia for Small Scale UAVs,
NASA, 2017](https://ntrs.nasa.gov/citations/20170000461).
Výše uvedené geometrické odvození a konkrétní protokol jsou engineering návrh
pro tento projekt; nejsou tvrzením o již provedeném měření.

## 6. Alternativní metody a jejich omezení

**Torzní drát:** J_total·θ¨+cθ˙+κθ=0, J_total=κT_d²(1−ζ²)/(4π²).
κ kalibrovat referenčním J_ref se stejným drátem/uchycením a fixture;
κ=4π²(J_ref+J_f)/(T_ref²(1−ζ_ref²)). Potom odečíst J_f a m_d d².
Metoda se vyhne geometrickému h, ale κ závisí na uchycení, materiálu, teplotě
a hysteresi. Opakovat reference před/po, kontrolovat lineární torque-angle
charakteristiku a různé malé amplitudy. u(κ) včetně covariance s J_f vstupuje
do rozboru; používání stejné reference dvakrát není nezávislé měření.

**Fyzické kyvadlo:** o pevné horizontální ose ve vzdálenosti d od CG platí
J_pivot·θ¨+Mg d θ≈0, tedy J_CG=Mg d T²/(4π²)−M d² po odečtení fixture.
Malé d dává špatnou identifikaci restoring torque; velké d vyžaduje odečítat
velký parallel-axis člen. Pivot tření, střed závěsu a kombinovaný CG fixture
musí být změřeny. Kontrolovat na více d; nechybně použít d samotného dronu,
když restoring moment zahrnuje i fixture. Pro malý 5″ dron je bifilár obvykle
praktičtější návrh k ověření, nikoli již prokázaná nejpřesnější metoda.

**CAD a statické váhy** jsou vhodná nezávislá kontrola, ne náhrada všech
nejistot. Metody nepotřebují běžící motory. Samotná úhlopříčka nebo celková
hmotnost neumí určit J. Držení zapnutého stroje v ruce není měřicí metoda.

## 7. Nejistoty a redukce dat

Použít standardní nejistotu (1σ), uvést rozlišení, calibration bias,
repeatability a covariance. Rozlišení rovnoměrně v ±a dává u=a/√3;
nejistotu průměru nezávislých opakování s/√n. Systematická chyba časové báze
nebo rozteče se průměrováním neodstraní. First-order propagace:

\[
u^2(J)=\sum_i(\partial_iJ)^2u_i^2
       +2\sum_{i<j}(\partial_iJ)(\partial_jJ)\operatorname{cov}(x_i,x_j).
\]

Opora: [NIST TN 1297, Appendix A](https://www.nist.gov/pml/nist-technical-note-1297/nist-tn-1297-appendix-law-propagation-uncertainty).
`bifilar()` implementuje analytický Jacobian včetně m_d,m_f,a,b,h,T,J_f,d,ζ,g,
kontroluje covariance bounds a kladnou semidefinitnost, odmítne dvojí zápis
stejného páru a nekladný výsledek po correction. Zanedbané correlations jsou
výslovně považovány za nulu, nikoli prokázány jako nulové.

Bez correction platí přibližně:
u_rel²(J)=(u_M/M)²+(u_a/a)²+(u_b/b)²+(u_h/h)²+(2u_T/T)²+(u_g/g)²
plus covariance. Jediné měření stejné rozteče použité jako a i b je korelované:
nelze podhodnotit chybu jeho dvojího použití. Pro correction používat plný
Jacobian, ne tuto zkrácenou relativní rovnici. g=9,80665 m/s² je konvenční
hodnota, nikoli změřené lokální gravitační zrychlení; při nároku na vyšší
přesnost dodat místní hodnotu a u(g).

**Demonstrační výpočet, nikoli Skystars:** syntetická perioda 0,9 s,
m_d=0,65 kg, m_f=0,05 kg, a=b=0,1 m, h=0,6 m, J_f=0,0002 kg·m²,
d=0,003 m. Výsledek J=0,002141561258 kg·m²,
u(J)=0,000019216359 kg·m². 20 000 nezávislých gaussovských MC vzorků,
seed 1729, dává poměr MC směrodatné odchylky k first-order u **1,001805**.
Je to softwarová kontrola lokální propagace za uvedeného distribučního
předpokladu. Nevypovídá o přesnosti budoucího fyzického rig nebo dronu.
Zdrojové CSV, vstupy a všechny u jsou uloženy v `measurement-demo/`.

## 8. Budoucí pohonové měření

Pouze samostatně schválený laboratorní stand se zajištěným motorem/prop,
krytem, externím emergency odpojením, tahovou a reakční load cell, voltage/current
měřením, RPM a společným timestamp/trigger. **Žádný motor test přes FC.**
V této fázi se nic fyzicky nespouští.

Pro každý motor/ESC/prop a konfiguraci zaznamenat:

1. Nulování a load cell kalibraci před/po; tah F a reakci Q=L·F_cell v SI,
   nejistotu ramene L, křížovou citlivost, teplotu a připojené kabely.
2. Doloženou normalizaci q, deadzone, několik ustálených pracovních bodů a
   několik napětí se stejným příkazem, repeat a vzestup/sestup pro hysteresi.
3. Omezené kroky kolem pracovních bodů, společný t0 příkazu, RPM a síly.
   Fitting y(t)=y∞+(y0−y∞)exp(−(t−L)/τ) až po delay L; residualy mohou ukázat,
   že jeden první řád nestačí. Rise 10–90 % ≈2,197τ jen pro skutečný první řád.
4. Napěťovou závislost při stejné definici stavu: log(F2/F1)/log(V2/V1) může
   dát lokální β, ne univerzální konstantu. Repeat při zatížení a motor warming.
5. Bezpečný dosažitelný actuator ceiling doložený měřením standu a jeho
   omezeními. Marketing max ani nominální baterie nejsou ověřený letový limit.

RC/DISARMED SHADOW BIN neexcituje plant: obsahuje správné matematické výstupy,
ale motory zůstaly neaktivní. Nelze z něj získat absolutní F/Q/τ/D pohonu.
Ani nenulový UTB integrátor v SHADOW není closed-loop UTB identifikace.

## 9. Import a následná kalibrace

`utb_plant_identification.py --dataset ...` přijímá JSON `UTB_PLANT_SI_V1`.
Každá quantity má přesně value/unit/status/u/method/source. Status measured,
estimated, illustrative; unknown a chybějící hodnota se odmítne. Dataset type
demonstration nesmí tvrdit measured; measurement nesmí obsahovat illustrative.
Label measured je deklarace původu, nikoli automatická validace laboratoře.

Jednotky musí přesně odpovídat schema (`kg`, `m`, `kg*m^2`, `N`, `N*m`, `s`,
`N*m*s/rad`, `V`, `rad/s`, `1`). Převod g/mm/g·cm² musí být proveden vědomě
před importem. Všechny hodnoty finite, správné dimenze, J symetrická SPD a
realizovatelná, kladné fyzikální scales, nonnegative lag/delay/damping,
actuator ceilings 0..1. Křivky mají čtyři monotónní řady q=0..1, začínají
F/Q(0)=0, mohou mít deadzone/plateau; neobsahují implicitní PWM mapu.
u je scalar standard uncertainty v jednotce quantity, pro křivky v ordinátě;
matrix/curve correlations se zde automaticky nepropagují do confidence bounds.

FRD origin explicitně `center_of_mass` nebo `body_origin`; ve druhém případě
se odečte cg. Delay se zaokrouhlí nahoru na celé **nominální 2,5ms** cykly,
report obsahuje vstup, nominal rounded, modelový float32 interval i rounding.
Přesně 2,5 ms nesmí kvůli float32 reprezentaci spadnout do dvou cyklů — nový
regresní test to ověřuje. Subcycle delay není dosud implementován; zaokrouhlení
může znatelně měnit odezvu a nesmí být skryto před interpretací.

Původní baseline se nikdy automaticky nekalibruje. Samostatný dataset vytvoří
nový output adresář, parametrický report a sedm CSV: zero, roll, pitch, yaw,
axes, saturation, sine_2. Grafy vytváří `utb_plant_plots.py` pomocí původního
ReportLab plotteru. Demo `utb_1he_demo_plant.json` má všechny hodnoty explicitně
illustrative a configuration ID `DEMO_NOT_SKYSTARS_MEASUREMENT`.

## 10. Nové výsledky citlivosti a identifikovatelnosti

`utb_plant_sensitivity.py` provedl 18 variant téhož 8s axes scénáře:
reference (0,3;−0,3;0,3) rad/s, konstantní PID, stejné deterministické podmínky.
J jednotlivě ±10 %, τ ±25 %, D ±50 %, maximální moment/tah ±20 %, napětí ±10 %,
delay jeden cyklus a společné škálování J,D,F×1,5. Jde o **ilustrativní intervaly
citlivosti**, nikoli skutečně naměřená nejistota současného dronu.

| Změna modelu | RMS rozdíl body rate vůči baseline roll/pitch/yaw [rad/s] | Vybraný důsledek |
|---|---|---|
| Jxx −10 % | 0,000876 / 0,000035 / 0,000179 | Především roll, slabá gyroskopická vazba |
| Jzz −10 % | 0,000070 / 0,000070 / 0,002021 | Především yaw |
| τ +25 % | 0,001513 / 0,001508 / 0,000939 | Roll overshoot 3,68 → 8,40 % |
| D −50 % | 0,000919 / 0,000898 / 0,004541 | Yaw citlivé na tlumení |
| Moment/tah −20 % | 0,001982 / 0,002142 / 0,005000 | Yaw settle 465 → 590 ms |
| V −10 %, β=2 | 0,001869 / 0,002019 / 0,004709 | Moment −19 %, yaw settle 465 → 582,5 ms |
| Sensor delay +2,5 ms | 0,001339 / 0,001179 / 0,000374 | Roll overshoot 3,68 → 7,86 % |
| J,D,F společně ×1,5 | <9e−16 maximální rate rozdíl | Absolutní scale neidentifikovatelný z této odezvy |

RMS je přes celých 8 s, takže ustálená část může maskovat změnu transientu;
tabulka proto uvádí i overshoot/settling a uložené plné CSV/grafy. Pořadí
citlivosti závisí na excitation, PID, konfiguraci a zvoleném intervalu.
Netvrdí univerzálně nejcitlivější fyzikální parametr reálného dronu.
Minulé 17,5/22,5ms body instability sweepu jsou stále jen výsledky ilustrativní
baseline, nikoli limity H743/drone bezpečnosti.

**Strukturální důvody neidentifikovatelnosti:**

- Pokud J,D i všechny momenty násobíme stejným c, celá rotační rovnice se
  zkrátí c. Numerické maximum rozdílu ω **8,8818e−16 rad/s** to potvrzuje.
  Absolutní J musí být ukotveno fyzickým měřením nebo absolutním známým momentem.
- Čistý sensor delay a stejně velký actuator delay jsou pro r→měřené gyro
  ve zdejší baseline nerozlišitelné: maximum rozdílu měřeného gyro **0**.
  Skutečné ω se liší; bez nezávislého trigger/sensoru není poloha delay známá.
- Jediná konfigurace napětí neoddělí Fmax a V^β. β vyžaduje více známých V
  a nezávislou tahovou kalibraci. Stejně tak délka ramene a F gain musí být známy.
- Absolutní setrvačnost a tah nejsou identifikovatelné z nulové DISARMED odezvy;
  pro J tensor/axis coupling je nutná víceosá nezávislá informace.

Lokální finite differences sedmi parametrů (±1 %, normalizace gyro referencí
0,3 rad/s) a NumPy SVD daly singular values
10,9576; 2,68657; 2,13360; 1,62181; 1,03381; 0,000548; 0,000214.
Numerický rank při **analyticky zvoleném relative cutoff 10⁻³** je 5/7.
Jde o local conditioning ilustrativního modelu; cutoff není statistický test
identifikovatelnosti reálného stroje. Malé nenulové hodnoty vznikají také FD
a float32 přesností, neruší výše odvozené strukturální nejednoznačnosti.

## 11. Pořadí měření a validace

1. Bez napájení: konfigurace, m, CG, r_i, motor/prop inventory. Nejmenší riziko,
   levný nezávislý gain/axis základ, zabrání míchání různých baterií/drónů.
2. Bez napájení: reference/fixture a bifilární Jxx/Jyy/Jzz, podle asymetrie
   produkty J; nejistoty a nezávislé CAD srovnání. Ukotví absolutní scale.
3. Odděleně schválené DataFlash idle/LIST/bounded DATA. Nejde o plant tuning,
   ale o problém, který nyní omezuje spolehlivé získání následných důkazů.
4. Samostatně schválený laboratorní pohonový stand: F/Q, mapping, voltage,
   lag/delay/saturace. Žádné aktivní UTB na dronu.
5. Import measured/estimated datasetu, stejné scénáře, sensitivity s reálnými
   uncertainty intervals, porovnání předpovědi s nezávislými daty mimo fitting.
   Až potom případné další návrhy; aktivní UTB/FÁZE 2 tím není schválena.

Software testy zahrnují rozměry rovnic, SPD/realizovatelnost J, FRD znaménka,
rovnováhu, analytické konstantní moment/damping/lag případy, konservaci energie
torque-free, passive damping, saturation, noise reproducibility, step convergence,
baseline consistency a uncertainty/covariance. 27 nových fyzikálních testů PASS;
34 původních testů PASS s archivními regression evidence. Hardware plant validity
z těchto testů neplyne. Kalibrovaný general model zatím nemá vlastní validované
linearized poles, rotor interaction ani experimentální closed-loop důkaz.

## 12. Reprodukovatelné příkazy

Žádné nové závislosti nebyly instalovány. WSL `.venv` má pymavlink a flake8;
existující Windows Python má NumPy a ReportLab. Všechny nové výstupy této session
jsou v `tmp/utb-1he-offline-20261013_151020/`.

```sh
cd /home/jirka/ardupilot_utb
.venv/bin/python -B Tools/autotest/utb_plant_identification.py \
  --dataset Tools/autotest/utb_1he_demo_plant.json --output tmp/utb-1he-demo-NEW
.venv/bin/python -B Tools/autotest/utb_plant_identification.py \
  --pendulum-csv tmp/utb-1he-offline-20261013_151020/measurement-demo/DEMO_pendulum.csv \
  --output tmp/utb-1he-period-NEW
.venv/bin/python -B Tools/autotest/utb_plant_sensitivity.py --output tmp/utb-1he-sensitivity-NEW
.venv/bin/python -B Tools/autotest/utb_closed_loop.py --output tmp/utb-1he-baseline-NEW
UTB_AUDIT_EVIDENCE=tmp/utb-1he-offline-20261013_151020/regression-evidence.json \
  .venv/bin/python -B -m unittest discover -s Tools/autotest/unittest -p test_utb_offline.py -v
.venv/bin/python -B -m unittest discover -s Tools/autotest/unittest -p test_utb_plant_identification.py -v
```

SVD a grafy bez instalace, v PowerShellu z rootu repository:

```powershell
$pythonPaths = @(Get-Item "$env:USERPROFILE\.cache\*\*primary-runtime\dependencies\python\python.exe")
if ($pythonPaths.Count -ne 1) { throw 'Vyberte existující Python s NumPy a ReportLab.' }
$py = $pythonPaths[0].FullName
$root = '\\wsl$\Ubuntu\home\jirka\ardupilot_utb'
& $py -B "$root\Tools\autotest\utb_plant_sensitivity.py" --svd-only `
  --output "$root\tmp\utb-1he-offline-20261013_151020\sensitivity"
# --directory obsahuje demo-calibration/ a sensitivity/; figures/ musí být nové.
& $py -B "$root\Tools\autotest\utb_plant_plots.py" `
  --directory <new-combined-results-directory> `
  --baseline "$root\tmp\utb-1hd-offline-20261012_162639\results"
```

Pro budoucí data zachovat raw measurement CSV, calibration certificates,
configuration evidence a jejich SHA. Importovat nový JSON; pro všechny
estimated values doložit důvod, interval a sensitivity. Nevyplňovat mezery
demonstračními čísly pod označením measured.
