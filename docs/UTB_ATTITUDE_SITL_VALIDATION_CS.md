# FÁZE 2C: skutečné řízení orientace v ArduPilot SITL

Stav: IMPLEMENTOVÁNO /UNIT a skutečné SITL OVĚŘENO v uvedeném rozsahu.
Všech37 attitude sessions, dvě RATE-only a závěrečné regrese prošly.
Dvě přísnější2% settling metriky zůstávají výslovně nedoložené.
Fyzická FC se nepoužívá. Nejde o hardware tune ani povolení letu.

Větev `test/skystars-5inch`, HEAD
`687f825d228b12a64d36651072aed19356178217`, bez nového commitu/pushe.
Nová evidence výhradně `tmp/utb-2c-20261016_152820/`; historické2A/2B
artefakty nejsou přepsané. Zdrojový checkpoint obsahuje50 dosavadních
changed/untracked souborů, jejich SHA, tar a patch vůči HEAD.

## Skutečná kaskáda a měření

Vlastní `AP_UTB_SITLAttitude` počítá redukovanou rotation error a P
body-rate target; ten ve stejném400Hz slotu přijímá původní jediný UTB
Rate PID. Původní AP allocator/output řídí skutečný SITL proces/model
bfxrev, nikoli Python plant. [Architektura](UTB_ATTITUDE_CONTROLLER_ARCHITECTURE_CS.md)
obsahuje rovnice, frame/yaw pravidla, lifecycle a rozsah limitů.

Každá session má vlastní proces, params, BIN, receive.tlog, console,
events, packets.json, CSV a metrics JSON. Standardní ARM s ARMING_CHECK1,
RC/GCS failsafe1, GUIDED takeoff10m, ustálení gyro<0,03rad/s a náklonu<3°,
explicitní mode41, dvě sekundy zero hold, konečný14s generátor a explicitní
LAND/disarm mimo okno. Žádný force-arm ani skrytý AP fallback v okně.
Virtuální SITL čas/perioda nejsou H743 wall timing ani hard realtime důkaz.
Host speedup2, shodný pro srovnávací běhy. Default bfxrev je3kg model
s diagonal_size0,35m a Jx=Jy≈0,022969 /Jz≈0,045938kg*m² dle lokální
SIM_Frame konvence; není to identifikovaný fyzický5″ dron.

Reference step:1s zero,3s +amplitude,3s zero,3s −amplitude,4s zero.
Každá úplná attitude session musí mít5600 souvislých complete sad;
žádná interpolace ztrát. Jedna sada obsahuje10 mode41 paketů.
Nezávislý double maticový výpočet ověřuje rotation error a skutečný
omezený body target z logované AHRS orientace. Allocation audit nezávisle
rekonstruuje Bᵀ(T/G), CSeq/TimeUS a při AW1 také přírůstky I.

### Gains pouze pro SITL

| Smyčka/osa | P | I | D | IMAX | D_HZ |
|---|---:|---:|---:|---:|---:|
|Vnější roll|3 s⁻¹|0|0|—|—|
|Vnější pitch|3 s⁻¹|0|0|—|—|
|Rate roll|0,19|0,16|0,0015|0,19|15|
|Rate pitch|0,19|0,16|0,001|0,19|20|
|Rate yaw|0,30|0,60|0|0,19|20 (D neaktivní)|

Vnější parametrový default je2; explicitní testovací nastavení je3.
Norm RP targetu nejvýše0,3rad/s, yaw rate default0, S_AW default0.
S_TOFF default0. Throttle je konstantní inherited manual pracovní bod
z GUIDED, zaznamenaný pro každý běh. Mezi sessions není přesně identický.
Neexistuje vlastní ani skrytý AP altitude/position controller v okně.

## Unit, DISARMED a postupný start

79 C++ testů PASS: core47, stávající rate helper9, AP allocation9,
nový attitude helper14 skupin. Nové skupiny pokrývají všech19 požadovaných
oblastí: nulovou/signed/combined error a návrat oběma směry, nenulový yaw,
q/−q a wrap, norm clamp, invalid quaternion/NaN/Inf/dt/health/stáří/INS,
clock wrap, konfiguraci/reset/re-entry, nadlimitní referenci, vnější fault
před inner update, platný outer proti inner FRAME fault a starý RATE profil.
Single-axis expected výsledek je analytický; combined používá samostatný
double321 maticový výpočet, nikoli volání implementované quaternion metody.

128 Python testů PASS, včetně4 nových testů exponenciální odezvy, správné
definice signed návratu, nedosaženého targetu a odmítnutí neúplné evidence.
Před armed testy prošel DISARMED entry/default0, sedm config mutations,
latched stop, re-entry, odmítnutí6° reference a nulové RPY/PWM≤1000.

Pět předběžných skutečných letů: zero hold, roll±1°, pitch±1°, roll±3°,
pitch±3°, každý5600 complete sad bez faultu. Teprve potom následoval sweep
a další obtížnější scénáře. Preliminary K2 měl roll±3° IAE0,108757rad*s
a pitch0,105609rad*s; striktní2% settling nebyl na všech plateau dosažen.
Tato omezení nejsou přejmenovaná na ideální odezvu.

## Konečný sweep a opakovatelnost

Z2B rise≈69ms plyne pouze orientační first-order τ≈31,5ms / bandwidth≈31,8s⁻¹.
Pro kandidáty1,5/2/3s⁻¹ je separace přibližně21/16/11; nejde o změřené
pole nebo gain/phase margin. Rate cap zůstává ve dříve testovaném rozsahu.
Šest actual sessions mění pouze outer P, všechny inner gains/model stejné.
Před sweepem je uložen objective v outer-sweep-objective.json:

`J=.45*IAE/(amplitude*14)+.20*RMS/amplitude+.20*maxOvershootFraction+.15*RMS(rateTarget)/.3`.

| Společné K [s⁻¹] | Průměr J přes R/P |
|---:|---:|
|1,5|0,161770|
|2|0,135724|
|3|0,105154|

Zvolen3, nejlepší z konečného seznamu; nikoli globální optimum.
Následovalo3× roll a3× pitch v nezávislých sessions, všechny5600/5600 sad.

| Osa | IAE [rad*s], mean | ITAE [rad*s²], mean | RMS [rad], mean | Rise+10–90% [s], mean/min/max | Max překmit všech signed/nulových kroků [%] |
|---|---:|---:|---:|---|---:|
|Roll|0,070796|0,411673|0,011638|0,723877 /0,715547 /0,730541|0,705951|
|Pitch|0,069757|0,407314|0,011553|0,705551 /0,700553 /0,710549|0,776807|

Všech24 repeat přechodů má doložený2% settling s alespoň0,25s zbývajícího
pozorování: roll1,117–1,370s, pitch1,115–1,335s. Posledních0,5s návratů
do nuly mělo maximum absolutní chyby0,01871° roll /0,03401° pitch.
Obě znaménka sledují správný směr; návrat je aktivní zásah proti aktuální
nenulové orientaci, nikoli pouhý zero body-rate target.
Mean/min/max/sample stdev a každý přechod jsou v repeat-aggregate.json.

![Roll actual SITL](../tmp/utb-2c-20261016_152820/plots/roll-attitude.png)
![Pitch actual SITL](../tmp/utb-2c-20261016_152820/plots/pitch-attitude.png)
![Roll PID a skutečné AP motorové požadavky](../tmp/utb-2c-20261016_152820/plots/roll-pid-motors.png)
![Pitch PID a skutečné AP motorové požadavky](../tmp/utb-2c-20261016_152820/plots/pitch-pid-motors.png)

## Porovnání AP

Shodný angle generator, model, 400Hz fast slot, manual throttle mechanismus
a preflight. AP má původní outer P4,5, input shaping/accel/sqrt a yaw
management; UTB má jednoduchý P3 bez shaping/FF/heading integrátoru.
Izolovaný comparison používá identický UTB inner PID, plný AP comparison
původní AP inner tune (R/P0,135/0,135/0,0036, yaw0,3/0,02/0).
Po přípravě oba vlastníci mají AP rate FLTT/FLTE/FF0; není to porovnání
celého sériového STABILIZE včetně RC a angle-boost/throttle politiky.
UTB inner stejně jako v2A nevolá AP throttle gain-boost/rpy-mix helpers;
dosavadní mix stav zůstává inherited. Nesaturovaný constant-throttle
comparison neprokazuje ekvivalenci high-throttle-slew nebo limitovaných stavů.

| Osa | Vnější / vnitřní | Angle IAE [rad*s] | Rise+ [s] | Max všech kroků overshoot [%] |
|---|---|---:|---:|---:|
|Roll|UTB /UTB|0,071055|0,730541|0,6508|
|Roll|AP /UTB|0,046429|0,323204|0,8560|
|Roll|AP /AP|0,046796|0,333200|1,2908|
|Pitch|UTB /UTB|0,069679|0,703052|0,5870|
|Pitch|AP /UTB|0,046697|0,333200|0,8161|
|Pitch|AP /AP|0,046552|0,343196|1,2401|

AP je v tomto nastavení rychlejší a má menší IAE. UTB má menší maximální
překmit v této malé sadě; není tím prokázána obecná převaha nebo statistická
významnost, comparison má jednu session na kombinaci/ose. Přesné PARM,
počáteční AHRS/gyro/throttle a kompletní rates/PID/allocation jsou zachované.
Odebrání AP rate shapingu ve vlastní attitude větvi také mění transient
vnitřní smyčky proti starému RATE-only experimentu; jeho staré rise/overshoot
nelze automaticky přenést na nový klesající attitude-derived target.

![Roll comparison](../tmp/utb-2c-20261016_152820/plots/roll-comparison.png)
![Pitch comparison](../tmp/utb-2c-20261016_152820/plots/pitch-comparison.png)

## Závěrečné robustnostní a regresní výsledky

Všech14 scénářů dokončilo5600/5600 sad bez faultu. Inner gains a vybraný
outer K3 zůstaly stejné. Setrvačnost ±10% mění jen J; noise používá
SIM_GYR1/2_RND0,5deg/s (s původním throttle škálováním a INS noise floor),
throttle +0,0002 normalized, motor model pouze slew_max150→100,
delay přesně jeden adapter cycle. Slew100 je mírný, pravděpodobně neaktivní
limit v malém signálu; test neprokazuje velký motor-lag margin ani měření
interního rotor stavu. Každý skutečný model JSON a extra params je archivovaný.

| Scénář | IAE roll /pitch [rad*s] | Max krokový překmit [%] | Omezené outer cycles |
|---|---:|---:|---:|
|inertia-plus|0.069828 /0.070117|0.812|0|
|inertia-minus|0.071307 /0.071070|0.562|0|
|combined|0.071112 /0.071372|0.723|0|
|sine|0.130628 /0.001956|— (sinus)|0|
|sign|0.103819 /0.004358|0.679|19|
|heading30|0.070260 /0.069841|0.900|0|
|yaw-rate|0.072248 /0.071876|2.241|0|
|roll-five|0.116044 /0.003915|0.486|0|
|delay|0.070934 /0.070914|0.570|0|
|noise|0.071367 /0.071305|0.536|0|
|throttle|0.072745 /0.072433|2.043|0|
|motor|0.070393 /0.069302|0.739|0|
|clamp|0.123192 /0.003649|0.615|1185|
|aw|0.069288 /0.069974|0.985|0|

**Omezení přísného settling:** yaw-rate má na prvním záporném pitch plateau
nedoložené2% settling; throttle perturbace má jeden nulový roll návrat bez
trvalého2% settling. Max krokové překmity těchto běhů2,241% a2,043% nejsou
vynechané. Předem stanovené functional tail acceptance0,25° nebylo změněné;
2% settling je samostatná přísnější metrika, nikoli dodatečně uvolněný guard.
P-only vnější smyčka nemá kinematický feedforward pro držení Euler náklonů
při nenulovém body yaw; z těchto dat nelze oddělit celý příspěvek plant/
estimatoru a změny throttle. Přesná sub-2% dynamika v těchto případech
zůstává omezením, ne prokázanou univerzální přesností.

Heading30: skutečný počáteční AHRS yaw30,09°, konečný29,39°, správné signed
roll/pitch a návrat. Yaw-rate0,05rad/s: heading0,285→38,47°; žádný vlastní
heading hold. Sinus0,2Hz: least-squares poměr amplitud0,92568, phase delay
0,31843s v okně3–10s. Jde o jednofrekvenční popis skutečné odezvy, ne
stability margin. Sign case skutečně omezil19 cyklů, RMAX0,05 test1185;
max RP target0,300000029rad/s odpovídá float norm clampu0,3.

Přidaný2,5ms delay zachoval stabilní dokončení; max angle překmit0,570%.
Pomalá vnější smyčka proto v tomto experimentu netrpí stejně velkým angle
překmitem jako dřívější samotný rate step. Neprokazuje to obecnou toleranci
libovolného zpoždění. AW1 nominal dokončil všech5600 sad; nezávislá I
recurrence max chyba3,08e−11. Všechny nové běhy jsou bez allocation limitu,
takže nový2C nominal není opakováním aktivního saturation stressu2B.
Jeho původní kontrakt/testy a matematika zůstaly beze změny.

![Nenulový heading](../tmp/utb-2c-20261016_152820/plots/heading30.png)
![Přidaný jeden cycle delay](../tmp/utb-2c-20261016_152820/plots/delay.png)
![Skutečně aktivní target clamp](../tmp/utb-2c-20261016_152820/plots/clamp.png)

## Autorita, časování a omezení důkazu

37 attitude sessions =207200 complete sad. V každé validní vlastní větvi
platí UAAT.Src1, UAST.UN=Seq, AN0, UAMT.UN=Seq, APN0, MN=Seq.
Při AP outer/UTB inner je outer UN0/AN=Seq a inner UN=Seq/APN0.
Při AP/AP je outer UN0/AN=Seq a inner UN0/APN=Seq. Source dispatch a
nezměněný jediný output hook jsou součástí source-isolation auditu.
Counter neznamená fyzikální torque měření; motorové data jsou původní
simulator PWM/normalized allocation, AHRS je estimator feedback.

Nezávislá rotation/target chyba≤7,73e−8, projection chyba≤5,86e−8.
Shodný angle generator AP/UTB má max rozdíl0 a relative time phase
rozdíl≤833µs; nevyžadují se identické stochastic initial states.
První náklony AP comparison mají abs<0,040°, gyro komponenty<0,0015rad/s,
manual throttle0,363727–0,363968. Přesné hodnoty v overall-analysis.json.

Dosažená frekvence399,99337–400,01718Hz, žádné chybějící sady ve vybraných
oknech. Výška v celém novém attitude souboru9,1587–10,0479m,
max |vz|0,09185m/s, max body gyro norm0,359rad/s. Nejde o AltHold.
Host rate slot Ex měl maximum299µs a nezahrnuje post-output logger/capture;
není to izolovaně změřený overhead vlastního P ani H743 CPU/deadline.
UAAT58B+UAST35B přidávají37200B/s; celkem201200B/s mode41 diagnostic payload.
Původní RATE-only větev tyto dva pakety nezapisuje.

## Neúspěšné pokusy a neověřený rozsah

Žádný nový aktivní controller experiment neskončil faultem nebo envelope
abortem. Počáteční vývojové buildy odmítly přímé float equality a C++11
aggregate inicializaci Config; obojí bylo opravené před unit/DISARMED gate.
První metadata pokus nesprávně rozvinul symlink interpreteru mimo venv;
Windows alternativa narazila na upstream enum parser. Finální původní
metadata nástroje v existujícím venv prošly. Nic nebylo doinstalované.

Hardware aktivní stabilita/CPU, skutečná5″ dynamika, delší delay, velké
náklony, heading hold, disturbance rejection mimo zde uvedené testy,
physical torque a altitude/position hold nejsou ověřené ani implementované.
AP comparison není statisticky opakovaný test všech AP režimů. Float/unit
jitter test není hardware hard deadline validace.
