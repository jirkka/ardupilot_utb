# FÁZE 1H-B: validace UTB Rate PID a MotorMixeru

Stav: SOFTWARE-ONLY, 11. 10. 2026. Větev `test/skystars-5inch`,
HEAD `acf04efa99a478a03e1d2d8847f0ddc0b0a4c973`.
Produkční matematika, bridge, arming, AP_Motors, HAL, DShot a EKF nebyly změněny.
Žádné připojení k FC, změna jeho parametrů, upload, reboot, armování ani mazání logů.
Nové závislosti nebyly instalovány. Nový commit, push ani release nevznikl.

Výsledky: `tmp/utb-pid-mixer-validation-20261011_185957/`.
Checkpoint před změnami obsahuje `HEAD.txt`, `tracked.patch`, `untracked.tar.gz`
a původní status. `source-isolation.json` potvrzuje zachování předchozích
rozpracovaných souborů a obsahuje SHA-256 produkčních zdrojů.

## 1. Hodnocení

PASS znamená splnění uvedeného testovaného kontraktu, nikoli důkaz pro všechny
možné float32 vstupy nebo fyzickou stabilitu dronu.

| Vlastnost | Výsledek | Úroveň a konkrétní rozsah |
|---|---|---|
| Znaménka, jednotky, P/I/D rovnice, žádné FF | PASS | SOURCE VERIFIED + UNIT VERIFIED |
| C++ versus nezávislý model | PASS | 30 946 vzorků, 29 scénářů, všechny komponenty a masky |
| Integrace, ±IMAX, směrový anti-windup | PASS | UNIT VERIFIED, včetně 10 000 pokračujících saturovaných kroků na každé znaménko |
| Derivace měření, filtr, priming bez impulsu | PASS | UNIT VERIFIED; uzavřené analytické hodnoty i dynamické průběhy |
| BF_X_REV pořadí, znaménka, alokace, inverse | PASS | SOURCE VERIFIED + UNIT VERIFIED; geometrie FRD a oba typy 12/18 |
| Neplatné vstupy, dt, stáří, opakované INS, wrap | PASS | UNIT VERIFIED, původní health/state testy a nové dynamické testy |
| Nezávislost matematiky na formátu, rate a dostupnosti loggeru po startu | PASS | UNIT VERIFIED, bitová shoda výsledků při 50/200/400 Hz a formátu 0/1 |
| Nenulové gains, RC změny, 400Hz cesta | PASS | SITL VERIFIED, všechny tři osy mají nenulové P/I/D/IMAX/cutoff |
| Healthy → STATE_INVALID → healthy | PASS | SITL VERIFIED, explicitní virtuální AHRS fault; kontroly nebyly obcházeny |
| UTBC dekódování, Seq/Epoch | PASS | 18 654 paketů, kompletní bitové zpětné složení; 9 534 navazujících platných dvojic proti rovnicím |
| Žádný nečekaný TIME_BUDGET latch | PASS s omezením | SITL VERIFIED logická cesta; časování guardu jednotkově ověřeno, virtuální čas neurčuje H743 režii |
| Motorová izolace | PASS | SOURCE VERIFIED + SITL VERIFIED; stejný RC, DISARMED a AP ground idle, 20 výstupních sad na stav/run |
| UTB_ACRO nesmí armovat | PASS | FÁZE 0 SITL regrese: běžné, forced a RC armování odmítnuto; armed vstup odmítnut |
| Staré H743 BIN, nulové gains | PASS | OFFLINE VERIFIED, 4 000/8 000 platných sad; infrastruktura, nikoli nenulová PID dynamika |
| Bezchybná perzistence každého vzorku v novém SITL | UNVERIFIED | Jsou zaznamenané queue/backend ztráty; nebyly zaměněny za chybu PID ani rekonstruovány |
| H743 režie a stack při nenulových gains | UNVERIFIED | HARDWARE UNVERIFIED |
| Uzavřená UTB smyčka, fyzická odezva, letové gains | UNVERIFIED | Žádný model dronu v syntetické validaci, žádná aktivní UTB motorová autorita |

Reprodukovatelná chyba produkční matematiky nebyla nalezena. FAIL požadovaného
matematického kontraktu nebyl zjištěn. Nedokončené fyzické vlastnosti zůstávají
UNVERIFIED a neudělují povolení k FÁZI 2.

## 2. Autoritativní kód a veličiny

Rovnice vycházejí ze skutečného [RateController.cpp](../libraries/AP_UTB/AP_UTB_RateController.cpp),
zejména řádků 26–54, 57–124 a 127–139. Čísla řádků v tomto reportu odkazují
na výše uvedený HEAD; v produkčních souborech se v této etapě nezměnila.

Na každé ose j: `r` je reference v rad/s, `ω` měřená body rate v rad/s,
`h` skutečné dt v sekundách, `h₀` nominální dt. FRD: x dopředu, y doprava,
z dolů; roll/pitch/yaw odpovídají kladné rotaci kolem těchto os.
P, I, D, raw a output jsou bezrozměrné diagnostické požadavky.
Kp má jednotku s/rad, Ki 1/rad, Kd s²/rad, IMAX je bezrozměrný,
cutoff je v Hz, filtrované zrychlení v rad/s².
Nejde o moment v N·m, Newtony tahu ani PWM/DShot povel.

Parametry jsou v [AP_UTB.cpp](../libraries/AP_UTB/AP_UTB.cpp), řádky 40–146.
Každá osa má P/I/D/IMAX/D_HZ; default všech je nula.
Žádný feedforward člen ani FF parametr neexistuje.

### P, I, D a výstup

Pro platný krok po primingu:

```text
e[n] = r[n] - ω[n]
P[n] = Kp · e[n]
ΔI[n] = Ki · e[n] · h[n]
ε = float32(10^-5)
ρ[n-1] = raw[n-1] - achieved[n-1]
blocked[n] = (ρ[n-1] > ε AND ΔI[n] > 0)
          OR (ρ[n-1] < -ε AND ΔI[n] < 0)
I[n] = 0                                              při Ki = 0
I[n] = clip(I[n-1] + (blocked ? 0 : ΔI[n]), -IMAX, IMAX) jinak

a_raw[n] = (ω[n] - ω[n-1]) / h[n]
τ = 1 / (2π · D_HZ)
β[n] = h[n] / (τ + h[n])
a[n] = (1 - β[n]) · a[n-1] + β[n] · a_raw[n]           při Kd > 0
D[n] = -Kd · a[n]                                     při Kd > 0
a[n] = D[n] = 0                                      při Kd = 0
raw[n] = P[n] + I[n] + D[n]
output[n] = clip(raw[n], -1, 1)
```

I používá obdélníkovou integraci s aktuální odchylkou, nikoli trapezoid.
D působí na measurement, nikoli na error: změna samotné reference nevytváří
D kick. Filtr je první řád s backward Euler diskretizací. Při konstantním h
je jeho pól `1-β = τ/(τ+h)` v intervalu (0,1).
Diskrétní přenos D větve je `-Kd·β·(1-z⁻¹)/(h·(1-(1-β)z⁻¹))`.
To je matematika filtru; bez modelu soustavy nejde o fyzickou frekvenční odezvu dronu.

Controller nejprve spočítá všechny tři osy. Historii ω/a/I zapíše až po úspěšné
kontrole konečnosti všech os; chyba vede k resetu a nulovému nevalidnímu výsledku.
I je vždy omezen ±IMAX. `a_raw` se kontroluje i při Kd=0, takže numericky
přetečený rozdíl measurement může zneplatnit výsledek i s vypnutou D větví.

### Anti-windup: SHADOW ONLY

Feedback používá **raw − achieved**, nikoli output − achieved
([RateController.cpp](../libraries/AP_UTB/AP_UTB_RateController.cpp), řádky 127–139).
Zahrnuje proto limitaci samotného PID i mixeru. Brána ovlivňuje následující krok,
nevrací zpětně už provedenou integraci. Při pokračující saturaci ve stejném směru
I zůstává na hodnotě prvního saturovaného kroku; opačné ΔI se může integrovat.
Po zániku residualu se integrace znovu povolí. Není to back-calculation ani
vazba na AP_Motors limit flags. Yaw nemá nižší prioritu.

Tato strategie je vlastní engineering decision pro SHADOW ONLY. Dynamiku
soustavy řídí původní AP. Zaznamenané chování UTB integrátoru proto neověřuje
integrátor v uzavřené UTB smyčce. Nulový default Ki zůstává správný.

### Guards, reset a priming

Gains: Kp/Ki ∈ [0,5], Kd ∈ [0,1], IMAX ∈ [0,1], cutoff ∈ [0,200].
Při Kd>0 musí být cutoff>0 a ≤min(200, 0,1/h₀), tedy při 400 Hz nejvýše 40 Hz.
Konfigurace musí být finite; jakákoli přesná změna gains nebo h₀ resetuje historii.
Žádný epsilon deadband konfigurace se nepoužívá.

Reset nuluje předchozí measurement, filtr, I i residual a ruší primed flag.
První platný vzorek pak pouze nastaví předchozí measurement: výsledek je
`valid=false, priming=true`, všechny výstupní složky jsou nulové i při nenulové
referenci. Další platný krok může mít nenulový P/I, což je očekávaná reakce,
nikoli resetový impuls. Reset nezruší platnou konfiguraci, pokud neuspěla konfigurace samotná.

Platné h musí být finite a včetně hranic `[0,25h₀, 4h₀]`.
[State.cpp](../libraries/AP_UTB/AP_UTB_State.cpp), řádky 19–29, vyžaduje
attitude/rates valid flags, konečný kvaternion s `|q²−1|<0,01`, finite ω,
a stáří sample i INS časové značky nepřesahující `min(0,1 s,4h₀)`.
Unsigned 32bit rozdíl funguje přes přetečení v tomto krátkém časovém horizontu.
Provider na řádcích 38–44 používá AHRS initialised/healthy a health primárního gyra.

[AP_UTB::evaluate](../libraries/AP_UTB/AP_UTB.cpp), řádky 297–351, kontroluje
stáří vůči explicitnímu `validation_time_us` odebranému po samplingu a navíc
vyžaduje nový INS timestamp proti předchozímu cyklu. Samotný RateController
nemá vlastní hodiny ani detekci opakovaného INS timestampu; jeho vnitřní kontrola
používá `sampled_us`. Aktuální stáří snapshotu a novost INS v produkční cestě
ověřuje nadřazený evaluate. Deterministické testy oba kontrakty oddělují.

Mode/armed/frame/primary-gyro změna, nevalidní vstup a nevalidní feedback resetují
historii. Geometry změna navíc inkrementuje Epoch. Epoch není ID každého PID
resetu: například AHRS invalidace a změna gains ho sama neinkrementují.
`reset_shadow()` Epoch inkrementuje; Seq zůstává rostoucím číslem shadow cyklu.

FSTRATE, nepodporovaný frame, neplatná reference/thrust, config a dt mají své
reason codes. Logger musí být dostupný pro start experimentu; následná ztráta
loggeru mění observation health, nikoli matematiku. TIME_BUDGET se po třech
navazujících překročeních 5 % nominální periody zalatchuje, tedy nad 125 µs
při 400 Hz; reset SHADOW latch uvolní. Kontroly ani prahy se nezměnily.

## 3. Reference a zařazení v hlavní smyčce

[Reference.cpp](../libraries/AP_UTB/AP_UTB_Reference.cpp), řádky 14–29:
normalizované AP RC stick vstupy po calibration/deadzone/reverse se pro roll/pitch
dělí `L=max(1,sqrt(sx²+sy²))`; yaw se kruhově neomezuje.
Statická mapa je `r_RP = radians(rate_RP) · expo(s_RP/L)` a
`r_Y = radians(rate_Y) · expo(sY)`. Lokální
[input_expo](../libraries/AP_Math/control.cpp), řádky 760–769, používá
`(1-expo)·s/(1-expo·|s|)` pro `expo<0.95`, jinak s.
Pozor na skutečný float32 parametr: `0.95f` se při porovnání s double literálem
`0.95` stále nachází v první větvi. Původní test source convention tuto hranici ověřuje.

Jde o **AP-derived UTB shadow reference**, bez traineru a bez dynamic RATE_TC.
Převzetí této source convention nefixuje budoucí UTB_ACRO na AP parametry.
Thrust je AP ACRO statická pilotní throttle mapa; zde nebyla upravována.

[Bridge](../ArduCopter/utb.cpp), řádky 16–83, odebírá scheduler dt/h₀,
read-only AP target capture, sampled state a RC reference. `validation_time_us`
je na řádku 47 až za sample. Na [Copter.cpp](../ArduCopter/Copter.cpp),
řádcích 124–129, shadow následuje po AP motor output a před read_AHRS;
update_flight_mode následuje později. Attitude a reference/target proto nemusí
být stejný logický sample. AP controller se nespouští podruhé.
APUS/APS/APM/Cap/TimeUS/Seq umožňují offline zarovnání; přesná shoda AP a UTB
reference není očekávaná například ve STABILIZE nebo SYSID.

## 4. MotorMixer a nezávislé znaménkové odvození

[MotorMixer.cpp](../libraries/AP_UTB/AP_UTB_MotorMixer.cpp), řádky 19–68.
Guard dovoluje pouze FRAME_CLASS=1 a FRAME_TYPE 12/18, control.valid=true,
finite collective T∈[0,1] a každou finite složku c=output v [-1,1].
Neplatná alokace vrací nulový výsledek s valid=false.

Pro FRAME_TYPE=18, v logickém AP motor pořadí M1…M4:

```text
           R     P     Y
B = 1/2 · [-1    -1    +1]  M1 zadní pravý, CCW
          [-1    +1    -1]  M2 přední pravý, CW
          [+1    -1    -1]  M3 zadní levý, CW
          [+1    +1    +1]  M4 přední levý, CCW
```

Odvození nevyužívá opsanou mixovací matici: poloha motoru je `(x,y,0)`, jeho
tah míří proti FRD z. `r×F = (-yF,xF,0)` dává roll/pitch znaménka.
Rotor CW z pohledu shora má kladný moment kolem z, reakce na tělo je záporná;
CCW má opačnou reakci. Souřadnice pro M1…M4 jsou (-,+),(+,+),(-,-),(+,-).
Koeficient 1/2 je zvolená diagnostická normalizace, nikoli fyzická konstanta ramene.

Geometrický kontrakt je nezávisle ověřen proti AP úhlům 135°,45°,-135°,-45°
a spinům v [AP_MotorsMatrix.cpp](../libraries/AP_Motors/AP_MotorsMatrix.cpp),
řádky 640–649. `add_motors()` na řádcích 561–565 používá index pole jako motor_num;
testing_order není číslo řádku motorového výstupu. Yaw constants jsou CW=-1,
CCW=+1 ([AP_MotorsMatrix.h](../libraries/AP_Motors/AP_MotorsMatrix.h), řádky 10–11).
FRAME_TYPE=12 obrací pouze yaw sloupec. Fyzické zapojení konkrétního FC,
rotace skutečných motorů a geometrické rameno tímto software testem potvrzeny nejsou.

### Alokace a inverse

```text
v = B · c
l = min(v), u = max(v), span = u-l
α = 1                         pro span≤1
α = 1/span                    pro span>1
T* = projection(T, [-αl, 1-αu])
shift = T* - T
m_i = clip(T* + αv_i, 0, 1)
achieved = Bᵀ · m
achieved_T = (m1+m2+m3+m4)/4
```

Sloupce B jsou ortonormální a jejich součty nulové. Proto v reálné aritmetice
`achieved=αc` a `achieved_T=T*`; collective shift neovlivní momenty.
Priorita: zachovat poměr R/P/Y, pokud možno velikost; collective lze posunout;
samotný span>1 škáluje všechny tři osy společně. Yaw nemá nižší prioritu.

Lo/Hi indikují blízkost hranic s ε; Pos/Neg bity 0/1/2 popisují residual
`c−achieved`, bit 3 `T−achieved_T`. `scaled` a `shifted` používají také ε,
nikoli detekci každé nenulové změny. UTBR.Sat používá **raw−achieved**, navíc
bit 2 předchozího směrového integration gate. Tyto dvě masky mají odlišný význam.
Před finálním clampem se kontroluje finite a tolerance motorového intervalu
[-ε,1+ε]; platné uložené výstupy jsou vždy [0,1].

Ruční kontrolní body pro typ 18 a T=0,5:

| c | m1,m2,m3,m4 | α | T* |
|---|---|---|---|
| (0,0,0) | (0,5;0,5;0,5;0,5) | 1 | 0,5 |
| (0,25;0;0) | (0,375;0,375;0,625;0,625) | 1 | 0,5 |
| (0;0,25;0) | (0,375;0,625;0,375;0,625) | 1 | 0,5 |
| (0;0;0,25) | (0,625;0,375;0,375;0,625) | 1 | 0,5 |
| (1;1;1) | (0;0;0;1) | 0,5 | 0,25 |

Pro c=(0,5;0;0) je dosažitelný collective [0,25;0,75], proto T=0,01/0,99
vede na 0,25/0,75 bez zmenšení roll požadavku.

AP_Motors se liší: obsahuje compensation, throttle averaging/limits,
yaw headroom a samostatný yaw clamp před společnou RPY alokací
([AP_MotorsMatrix.cpp](../libraries/AP_Motors/AP_MotorsMatrix.cpp), řádky 228–394).
UTB allocator není kopií této letové politiky, ESC převodu ani thrust linearizace.
Je pouze diagnostický a není schválen pro aktivní letový output.

## 5. Nezávislý model a tolerance

[utb_math_reference.py](../Tools/autotest/utb_math_reference.py) používá Python
standard library a reálnou double aritmetiku. Geometrie se vytváří z poloh/spinů,
alokace jako projekce do dosažitelného intervalu, inverse jako transpozice
ortogonálních sloupců, součty přes math.fsum. Nejprve se kvantují pouze vstupy
na skutečné float32, nikoli každá operace jako v kopii C++.
Model nese I/a/residual historii a oddělené absolutní chybové meze.

Osm analytických unittest kontrol modelu je nezávislých na C++ exporteru:
ruční P body/linearita; `I=clip(Ki·e·Σh)`; ±IMAX; pokračující saturace;
pro skok measurement A, `D[n]=-Kd·β·A/h·(1-β)^(n-1)`;
pro rampu se směrnicí b, `D[n]=-Kd·b·(1-(1-β)^n)`;
reference step bez D kick; geometrické a collective ruční body;
float32 cancellation bound. Shoda dvou programů není jediným důkazem.

Pro float32 je unit roundoff `u=2^-24`. Meze zahrnují kvantované vstupy jako
přesná čísla, chybu π, multiplikace/divize, historii filtru/integrátoru,
`γ_n=n·u/(1-n·u)` pro součet absolutních magnitud při libovolném pořadí sčítání,
a absolutní příspěvek `2^-149` pro subnormální hodnoty.
Například násobení nese `|x|E_y+|y|E_x+E_xE_y+u|xy|`; dělení nese
`(E_x+|x/y|E_y)/(|y|-E_y)+u|x/y|`.
Clamp je neexpanzivní; extrema/projekce nesou největší nejistotu větví.
Výsledná mez závisí na konkrétním kroku a komponentě, včetně akumulace I.
Devět platných desetinných číslic C++ CSV dovoluje přesně obnovit float32.
Masky/valid/priming/gate se porovnávají přesně, nikoli s tolerancí.
Float32 konstanty ε a `1-ε` jsou ve modelu kvantovány stejně jako v C++.

Výsledek `math-certified/comparison.json`: 30 946 vzorků, 29 scénářů,
0 mismatches. Největší poměr chyby k vlastní mezi je 0,999878243
(odečtení error); žádná mez nebyla dodatečně rozšířena podle výsledku.

| Veličina | Největší absolutní C++−model odchylka napříč osami |
|---|---:|
| Error | 1,89990×10^-7 rad/s |
| P | 6,83144×10^-8 |
| I | 9,00138×10^-7 |
| D | 2,21006×10^-8 |
| Raw / output | 9,06641×10^-7 |
| Filtrované acceleration | 1,09240×10^-6 rad/s² |
| Motor m_i | 1,59267×10^-6 |
| Achieved R/P/Y | 9,44274×10^-7 |
| Achieved collective | 1,12976×10^-6 |
| Scale | 3,01703×10^-8 |
| Shift | 1,11192×10^-6 |

Dlouhé I akumulace mají větší absolutní chybu než jeden P krok, jak předpovídá
forward error propagation. Masky těsně u zaokrouhlovacího pásma libovolného
složeného výpočtu nejsou globálně dokázané; aktuální vektory mají přesnou shodu,
včetně sousedů float32 EPS/NEAR_ONE pro collective-only alokaci.
FTZ/FPU/compiler chování H743 pro subnormální extrémy zůstává hardware neověřené.

## 6. Scénáře a testy

Syntetická sada: konstantní signál; kladný/záporný reference step; změna znaménka;
rampa; reference sinus; fázově posunutý measurement sinus; krátký impuls;
střídání saturace a lineární oblasti; jitter ±24 µs kolem 2500 µs;
invalidace/recovery/reset a neplatné dt; D measurement step/ramp/reference step
pro cutoff 5/20/40 Hz a změnu dt 2,5→5 ms; ±I s Ki=0/1 a IMAX;
P s Kp=0/0,1/0,2; mixer grid 9³ kombinací ×5 collective ×2 frame types.
Včetně NaN/Inf, sousedů ±1, nejmenších normálních/subnormálních floatů,
hranic Lo/Hi a návratu po invalidaci. Všechny podporované složky jsou při
kombinovaném testu současně nenulové; gains jsou numerické testovací hodnoty.

Nový C++ fixture target přímo linkuje neupravený produkční controller/mixer.
Řádky vstupů/CSV výstupů jsou host-only testovací transport, nikoli nový firmware
logging. Unit varianty neslouží jako motorový backend.

| Ověření | Výsledek a uložený důkaz |
|---|---|
| SITL + BENCH_ON build, původní i nový test target | PASS: build-sitl.log, build-math-formatted.log |
| Původní UTB GTest | 47/47 PASS: regression-0.log |
| Benchmark GTest | 12/12 PASS: regression-1.log |
| Nový UTBMath GTest | 4/4 PASS: math-certified/cpp-test.log |
| Analytický Python model | 8/8 PASS: regression-2.log |
| Dekodér UTBC unit | 6/6 PASS: regression-3.log |
| Nový nenulový PID SITL | PASS: sitl-math-rerun/results.json |
| FÁZE 0 SITL | PASS: sitl-skeleton-final.log |
| Původní SHADOW FÁZE 1 packed SITL | PASS: sitl-shadow-final/results.json |
| Offline archivovaný H743 BIN | PASS: hardware-offline.json |
| flake8, astyle, git diff --check | PASS: final-checks.json a příslušné logy |

Nenulový SITL používá FRAME_TYPE=18, model bfxrev a všechny osy
Kp=0,31, Ki=0,23, Kd=0,004, IMAX=0,19, cutoff=20 Hz. Změny RC vyvolaly
P/I, kladnou/zápornou saturaci a gate; žádné hodnoty nejsou doporučený letový tune.
Validní sousední dvojice se stejným Epoch/mode/Seq+1 se porovnávají po
komponentách s referenční rekurencí. Stav modelu se odvozuje z předchozího logu;
inverze D/Kd zahrnuje jednu násobicí roundoff chybu. Nejde o celou rekonstrukci
historie přes chybějící vzorky. Maximum raw error bylo 2,10984×10^-7,
motor error 1,17695×10^-7; vše uvnitř odvozených mezí.

Runtime STATE_INVALID se vyvolává pouze ve vlastním DISARMED SITL výběrem
existujícího vypnutého EKF2 backendu a návratem na běžící EKF3. Žádná EKF
implementace se neupravuje. FRAME_TYPE 18→1→18 ověřuje frame guard a Epoch 0→1→2.
Log format 1→0→1 a rate 400→200→400 jsou explicitně testovány.
Val=0 výsledky mají nulové motorové alokace. Nenulová D dynamika je hlavně
synteticky ověřena; disarmed gyro SITL se téměř nemění.

Nový SITL zaznamenal poslední pipeline Drop=4, Miss=3, Gone=1; při změně formátu
je dlouhá mezera v UTBC očekávaná (mezitím se ukládá legacy). Snížení rate dává
Seq skoky 2. Některé další mezery jsou skutečná diagnostická ztráta.
Ztracené vzorky nejsou dopočítávány ani zahrnuty do sousedních PID porovnání.
Zkouška proto netvrdí bezeztrátový 400Hz logger ve všech host podmínkách.

Původní packed SHADOW regrese: 25 947 kompletních sad, 0 částečných,
25 068 UTBC paketů; 200/400 Hz ≈199,997/399,993 Hz. Test zároveň záměrně
vyvolává logger loss/queue overflow, cumulative Drop=876 není bezeztrátový bench.
SYSID capture se shodoval v 1 997 PIDR vzorcích, max difference 2,183×10^-11 rad/s.
Primary gyro změna 0↔1, priming a recovery prošly; RC remap/reverse/deadzone
prošly pro 100 vzorků na každou osu.

První pokusy jsou zachovány: jeden souběžný skeleton start narazil na již obsazený
SITL SERIAL1 TCP port 5762; opakování bylo sekvenční a prošlo. První gyro fault
podnět nevyvolal STATE_INVALID, protože stav zůstal zdravý; test ho nepovažuje za
důkaz invalidace. Upravil se pouze virtuální podnět, nikoli safety policy.
Tyto harness pokusy nejsou produkční matematická regrese.

## 7. Motorová izolace

Source audit nenalezl motor setter, HAL rc output, DShot ani AP_Motors závislost
v AP_UTB matematice. Producer nevolá logger ani blocking queue API.
Bridge jen čte armed/frame a ukládá diagnostiku; logger je v samostatném workeru.
Žádný nový kód nebyl přidán do produkční hlavní smyčky.

Nová nenulová SITL dvojice se stejným mode STABILIZE a RC:
DISARMED výstupy (1000,1000,1000,1000), AP armed ground idle
(1100,1100,1100,1100), v obou případech 20/20 identických sad OFF/ON.
Virtuální arming patří původnímu AP controlleru; UTB požadavek je současně
nenulový, ale neovlivní tyto výstupy. Nejde o bitovou ekvivalenci každého
transientu nebo skutečného letu při rozdílném scheduling.

[ModeUTBAcro::init](../ArduCopter/mode_utb_acro.cpp), řádky 15–22, odmítá armed
a jiný spool než SHUT_DOWN; `run()` na řádcích 30–32 pouze vzorkuje stav.
[mode.h](../ArduCopter/mode.h), řádek 446, vrací allows_arming=false.
Původní restrictions zůstaly byte-for-byte beze změny.

## 8. Offline H743 data

Použit výhradně archiv `tmp/utb-400-recovery-20261011_173711/PACKED_CD_FIXED.bin`,
7 654 746 B, SHA-256
`ed08afb62c8dc575353dda96dfe649655178f9d88aca9ea2bfef4f1307a533be`.
Všech 15 gains/IMAX/cutoff parametrů je v tomto BIN nula.

| Okno TimeUS | Sady / Hz | Dt minimum / maximum / průměr |
|---|---|---|
| [317000000,337000000) | 4000 / 200 | 2455 / 2546 / 2501,665 µs |
| [357000000,377000000) | 8000 / 400 | 2438 / 2562 / 2499,999 µs |

200Hz průměr Dt je průměr vybraných logovaných vzorků, není průměr všech cyklů
smyčky (wall-time selection má bias). Při 400 Hz je všech 7999 sousedních Seq
kroků 1; v 200Hz okně jsou kroky 1/2/3. AP capture metadata jsou zarovnaná,
oba epochy v těchto oknech mají jednu konstantní hodnotu. Všech 12 000 paketů
má bitově přesný decode/repack, valid controller/mixer, nulové P/I/D/raw/output
a nulovou chybu collective-only motorů proti modelu.

Reference jsou v obou oknech nulové, gyro přibližně v rozsahu ±0,0065 rad/s.
Přímý důkaz RC škálování při vychýleném sticku z těchto neutrálních oken nevzniká;
ten poskytují source/unit/SITL RC map testy. Raw sampled_us, INS timestamp,
kvaternion a jednotlivé AHRS health podmínky se v UTBC neukládají. Jejich
čerstvost je doložena nepřímo platným výsledkem guardů; nelze ji nezávisle
rozložit z tohoto BIN. Rozdíl TimeUS delta minus Dt dosahuje při Seq+1
zhruba -19 až +18 µs ve 400Hz okně, odpovídá různým místům měření v cyklu;
není sám o sobě stářím IMU vzorku.

## 9. Časová režie a limity

Host-only benchmark používá std::chrono::steady_clock, nikoli AP virtuální čas.
Pět běhů, každý 1 000 000 kroků na variantu, stejné vstupy a build:

| Měřená cesta | Zero gains medián | Nenulové gains medián | Pozorovaný rozdíl |
|---|---:|---:|---:|
| Controller update, bez mixeru a feedbacku | 68,202 ns | 88,986 ns | +20,784 ns |
| Controller + mixer + feedback | 138,153 ns | 168,339 ns | +30,186 ns |

Rozsahy: controller zero 66,958–82,445 ns, active 84,594–93,504 ns;
celá matematika zero 135,020–184,131 ns, active 164,200–180,412 ns.
Includes loop/input/checksum overhead a host scheduling. Oddělené běhy proto
nedávají přesnou izolovanou cenu mixeru odečtením mediánů.
Benchmark active gains jsou (0,3;0,2;0,002;0,4;20) na všech osách.
Výsledek prokazuje aktivní D/I větve s mírně vyšší host cenou; není převod na H743 µs.

Producer, consumer, logger lock waits, scheduler a worker jsou odlišné veličiny.
Host controller benchmark neobsahuje AHRS sampling, queue, packet writer ani logger.
SITL export Ex a producer/consumer instrumentace používají AP_HAL virtuální hodiny;
Ex byl v původní regresi 0. Z toho nelze odvodit reálnou CPU rezervu ani potvrdit
125µs budget na H743. PM v novém SITL hlásí MaxT=3332 µs a NLon=5/interval;
stejné hodnoty má i SHADOW=0 isolation baseline. Nejde o H743 deadline measurement.
Původní SHADOW SITL regrese zaznamenala NLon=4–5/interval.

Existující H743 zero-gain 400Hz bench měl producer průměr ≈39,75 µs,
consumer ≈13,36 µs a stack free minimum 1784 B z actual stack 3504 B;
to jsou historická měření z jiné etapy. Nenulové H743 PID větve, jejich vliv
na producer/consumer, mutex latenci, jitter, PM.NLon a worker stack jsou
**HARDWARE UNVERIFIED**. Zero-gain hodnoty nejsou jejich náhradou.

## 10. Reprodukce

Z rootu repo, existující `.venv`, bez instalace:

```sh
export PATH="$PWD/.venv/bin:$PATH"
./waf configure --board sitl --enable-UTB --enable-UTB_BENCH
./waf copter --targets tests/test_utb,tests/test_utb_math,tests/test_utb_bench -j4
build/sitl/tests/test_utb
build/sitl/tests/test_utb_bench
.venv/bin/python -m unittest discover -s Tools/autotest/unittest -p test_utb_math_reference.py -v
.venv/bin/python -m unittest discover -s Tools/autotest/unittest -p test_utb_log_decode.py -v
.venv/bin/python Tools/autotest/utb_math_validation.py --binary build/sitl/tests/test_utb_math --output tmp/utb-math-new-run
.venv/bin/python Tools/autotest/test_utb_math_sitl.py --binary build/sitl/bin/arducopter --output tmp/utb-math-sitl-new-run
.venv/bin/python Tools/autotest/test_utb_skeleton.py --binary build/sitl/bin/arducopter --compiled 1 --log-dir tmp/utb-skeleton-new-run
.venv/bin/python Tools/autotest/test_utb_shadow.py --binary build/sitl/bin/arducopter --packed --output tmp/utb-shadow-new-run
.venv/bin/python Tools/autotest/utb_math_log_analysis.py tmp/utb-400-recovery-20261011_173711/PACKED_CD_FIXED.bin --window 317000000 337000000 --window 357000000 377000000 --output tmp/utb-offline-new.json
```

Vol nové unikátní output adresáře/soubory; existující artefakty se nepřepisují.
SITL scénáře spouštěj sekvenčně kvůli sdíleným pomocným portům simulátoru.
GTest OfflineVectors bez proměnných UTB_MATH_INPUT/OUTPUT explicitně SKIP;
Python wrapper je nastaví, uloží vstupy, spustí C++ a ověří všechny řádky.

CSV vstup, skutečný C++ výstup a reference jsou v `math-certified/`;
souhrn obsahuje maxima chyb i poměr k odvozené mezi.
[Syntetická saturace a mixer](../tmp/utb-pid-mixer-validation-20261011_185957/math-certified/synthetic-pid-mixer.svg)
a [syntetický sinus s fázovým posunem](../tmp/utb-pid-mixer-validation-20261011_185957/math-certified/synthetic-sine-pid-mixer.svg)
jsou průběhy bez modelu dronu. Nejde o fyzickou step response.

## 11. Přesný manifest této etapy

Přidáno:

- `libraries/AP_UTB/tests/test_utb_math.cpp`: host fixture exporter, dlouhá saturace/recovery, bitová log-policy nezávislost, wall benchmark.
- `Tools/autotest/utb_math_reference.py`: nezávislé matematické rovnice, geometrie a odvozené error bounds.
- `Tools/autotest/utb_math_validation.py`: deterministické vektory, component comparison, CSV/JSON a syntetické SVG.
- `Tools/autotest/unittest/test_utb_math_reference.py`: analytické nezávislé kontrolní body modelu.
- `Tools/autotest/test_utb_math_sitl.py`: výhradně lokální nenulový SHADOW SITL, decode/repack, recurrence, health a motor isolation.
- `Tools/autotest/utb_math_log_analysis.py`: výhradně offline audit existujícího zero-gain H743 BIN.
- `docs/UTB_PID_MIXER_VALIDATION_CS.md`: tento report.

Upraveno:

- `Tools/autotest/test_utb_shadow.py`: isolation přijímá volitelný model simulátoru; default bfx zůstává, nový test používá bfxrev.

V `tmp/utb-pid-mixer-validation-20261011_185957/` jsou checkpoint, vstupní/výstupní
datasety, build/test logy, SITL BIN, JSON, SVG a SHA-256 manifest artefaktů.
Pointer `tmp/utb-pid-mixer-validation-current.txt` identifikuje tuto session.
Tyto dočasné důkazy jsou oddělené od versionovaných zdrojových změn.
Předchozí dirty README a další UTB dokumenty/hardware capture soubory byly
zachovány; nejsou vydávány za změny této etapy. Git HEAD zůstává původní.

## 12. Připravenost

Matematiku současného diagnostického PID a allocatoru lze považovat za
software ověřenou v uvedeném kontraktu a testovaném rozsahu. Nenalezl se podnět
pro opravu produkčních rovnic. Neexistuje důkaz fyzické regulační stability.

Před dalším fyzickým experimentem chybí samostatně schválené DISARMED H743
měření nenulových větví: reálný producer budget, logger/queue ztráty, lock latency,
PM/jitter a stack. Před případnou aktivní regulací navíc chybí model/identifikace
soustavy, uzavřená UTB smyčka, fyzické znaménkové a motorové ověření, validace
letových gains a samostatné posouzení anti-windup/allocation policy.
Žádná z těchto dalších fází zde nezačala a UTB_ACRO zůstává nearmovatelný.
