# FÁZE 2B: AP alokace a directional anti-windup v SITL

Rozsah: výhradně HAL_BOARD_SITL, mode 41, owner 1. Sdílená PID matematika
není změněná. Výchozí UTB_S_AW=0 zachovává původní latched SATURATION stop.
UTB_S_AW=1 je explicitní experimentální politika, frozen při vstupu do mode.
SOURCE a devět nových UNIT testů ověřeno; nominální aktivní SITL capture
ověřena. Výsledky aktivní saturace a robustnosti jsou doplněné níže po měření.

## Skutečná alokační infrastruktura

`AP_MotorsMatrix::output_armed_stabilizing()` násobí R/P/Y compensation gain G,
počítá roll/pitch motorové příspěvky, omezuje yaw podle dostupného prostoru
s minimálním MOT_YAW_HEADROOM (hodnota * 0,001), přidává yaw a případně
škáluje společný R/P/Y span. Collective má vlastní throttle mix a může se
posunout nebo omezit. Limit flags samy neposkytují velikost ani znaménko
ztraceného příspěvku. Samotný throttle_upper neprokazuje ztrátu momentu.

`AP_MotorsMulticopter::output()` zachovává původní output_logic,
output_armed_stabilizing, thrust_compensation, output_to_motors a external
limits. `get_thrust()` vrací inverse-linearized actuator / G. Nejde o
měřený tah nebo fyzikální moment. Slew, boost, spool a scripting mohou
zrušit jednoduchý algebraický kontrakt a musí být rozlišeny.

Diagnostický AP_UTB_MotorMixer s prioritou poměru všech R/P/Y není aktivní
AP allocator. Jeho hypotetické achieved výstupy se zde nepoužívají.

## Podložený kontrakt

BF_X_REV=18: sloupce B pro AP motor ordering jsou
R=(-0,5;-0,5;0,5;0,5), P=(-0,5;0,5;-0,5;0,5),
Y=(0,5;-0,5;-0,5;0,5). Platí BᵀB=I a Bᵀ1=0.
Po jediném původním motor outputu capture vyčte skutečné factors, G,
pre-linearization příkazy T a inverse-linearized post příkazy P.

Rekonstrukce: a = Bᵀ(T/G). Common collective se eliminuje. Je to skutečný
normalizovaný **alokovaný příkaz**, nikoli skutečně dosažený fyzikální moment.
Residual r = u_raw − a zahrnuje UTB command clipping i AP allocator reduction.

Validace odmítá nesprávnou bázi, unequal collective factors, nonfinite/zero G,
T mimo [0;1], boost, neozbrojený stav nebo neplný spool. Musí platit
P≈T/G (1e−5) a Cmd≈clip(u_raw,−1,1) (1e−6). Tolerance jsou numerické
engineering tolerance pro float algebra; nejsou safety limit fyzického dronu.
Enabled scripting je pro politiku 1 odmítnutý při vstupu i za běhu, protože
může měnit external limits a motor overrides. Capture zůstává read-only.

Nový `AP_UTB_SITLAllocation` nemá motor setter, HAL output ani logger.
Existující read-only get_factors/get_thrust_rpyt_out měly definice jen pro
APM_BUILD_UNKNOWN. Jejich compile guard byl rozšířen o HAL_BOARD_SITL;
implementace getterů i allocator zůstaly beze změny. Na H743 se guard chová
stejně jako předtím. Cross-build prokazuje shodný fyzický image s FÁZÍ 2A.

## Integrace, časování a bezpečné odmítnutí

Příští regulační sample použije post-output capture předchozího output cyklu.
Kontroluje output Seq, source PID CSeq a stáří ≤10 ms. Bez adapter delay jde
o one-sample feedback (nominal 2,5 ms). UTB_S_DLY=1 přidává explicitní
2,5ms transportní delay; feedback se váže na příslušný starší PID příkaz,
nikoli na novější neaplikovaný výsledek. CSeq je proto od output Seq odlišné.
Zůstává unsigned rollover-safe kontrola a reálné INS dt.

Původní shared `AP_UTB_RateController::feedback()` uloží residual.
Pro každou osu deltaI=Ki*error*dt. Integrace se blokuje jen při
(r>1e−5 a deltaI>0) nebo (r<−1e−5 a deltaI<0). Opačný přírůstek se dovolí,
aby integrátor mohl unwind. IMAX stále clipuje I. Po návratu residualu do
lineární oblasti pokračuje integrace. Nejde o back-calculation ani o
vlastní změnu PID matematické rovnice.

Unknown/stale/wrong-sequence/boost/slew-mismatched feedback při stabilním
armed/full-spool kontextu vytvoří latched FEEDBACK fault před dalším PID.
Nenásleduje odhad podle limit flags, paralelní AP PID nebo automatický AP
fallback. Fault resetuje PID historii a původní experimentální fail-stop
vypne simulated spool; opětovný start vyžaduje explicitní mode exit/entry.
S_AW=0 nadále zastaví při původních limit flags nebo raw ≥1.

Arming/spool/context změny resetují historii. Priming není platný PID
příkaz; jeho capture proto nelze zaměnit za ready feedback. FSTRATE, frame,
state/INS freshness, gyro identity, envelope, failsafe a configuration latch
jsou zachované. UTB_ACRO=40 zůstává nearmovatelný. H743 aktivní mode41,
S_AW i motor setters z experimentu jsou compile-out.

## Diagnostika a overhead

Nová UAFB ID26 (3 axis packets) a UALT ID27 (1 capture packet) doplňují
původní UARX ID24/UAMT ID25. Max 16 format fields / 64 label bytes dodrženo.
TimeUS a Seq vážou všechny pakety; FBUS zachovává post-output timestamp.
Owner1 Raw je skutečné UTB P+I+D před clippingem, owner2 Raw je AP applied
normalized request, nikoli odhad AP interního unclipped PID. Blk popisuje
gate právě vypočteného PID; Alc/Res popisují následný output stejného cyklu.
Pro ověření gate se proto použije **předchozí** Res.

Fixed capture/result structs, žádná heap alokace, nový mutex nebo wait v
adapteru. Samotná capture CPU latency zatím není samostatně změřená.
UAMT.Ex měří host steady-clock rate slot a nezahrnuje post-output capture
ani logger zápisy. Virtual Dt není host execution time ani H743 timing.

Původní 4 pakety mají 213 B/cyklus; nové 4 pakety přidávají 197 B/cyklus,
celkem 410 B * 400 Hz = 164000 B/s před ostatními AP logy. Tyto zápisy jsou
jen SITL mode41 diagnostika. Pětinásobné zrychlení při rozšířené diagnostice
způsobilo neúplný log; tento běh je FAIL evidence, nikoli validní regulace.
Validovaná srovnání proto používají stejné dvojnásobné host zrychlení a
vyžadují kompletní sady. Fyzický firmware tento overhead nemá.

## UNIT a nominální SITL evidence

`tests/test_utb_ap_allocation`: 9 PASS. Používá skutečný AP_MotorsMatrix
allocator, ne hypotetický UTB mixer. Izolovaný unit fixture má ideal post
stage; aktivní capture navíc čte skutečný inverse actuator. Pokryté:
linear projection/yaw headroom, throttle-only limit, 10000-cycle saturation,
sign reversal, short joint R/P saturation/recovery, unknown boost/slew/NaN/
wrong basis/G, stale/misassigned feedback, delay+jitter a default stop/clockwrap.

`preliminary-slow`: zero, roll a yaw, každý 3200/3200 kompletních sad,
bez faultu, AP calls=0, UN=MN=Seq. Nezávislá analýza ověřuje Bᵀ(T/G),
skutečný Cmd, attribution a deltaI proti předchozímu residualu. Tyto nominální
běhy nesaturují, a proto samotné neprokazují aktivní saturation recovery.

## Neověřené meze kontraktu

Obecné frames, motor loss/boost, aktivní motor slew mismatch, scripting,
multithreaded FSTRATE a skutečné fyzikální torque feedback nejsou podporované.
Je zachované bezpečné odmítnutí. Unit projection není kalibrace aktuátorů
H743 a úspěšný DISARMED SHADOW není aktivní hardware stability důkaz.

## Aktuální aktivní saturation výsledky

Stress plant má motor yaw factors ±0,01 (1 % původní yaw authority),
reference ±0,3 rad/s s 8s plateau a pouze pro stress IMAX=1. Není to
letový kandidát ani identifikace dronu. Ostatní guards zůstaly zapnuté.
Tři samostatné `yaw-long-verified-feedback` sessions (saturation,
saturation-observed, saturation-cleanup) každý dokončily 8000 platných sad.

První session: 1851 gated yaw cycles =4,628 s, I rozsah −0,665964 až
+0,668032, max raw 0,733865; IMAX nebyl dosažen. Long held interval
t=6,915–9,000 s má I přesně 0,663552 bez růstu. Changing collective/P
a další stav permitují mezi blokovanými intervaly jednotlivé legitimní
integral increments; gate není absolutní zmrazení všech budoucích stavů.
Po záporné chybě I unwind pokračuje, druhý signed plateau dosáhne záporné
alokační meze. Nezávislá CSV analýza kontroluje každé deltaI a projectuje
skutečné T/G. V zero tail opět residual ustoupí do linear region; fyzikální
tracking recovery velmi slabého yaw modelu není během 2s tail úplně ustálený.
Není to tvrzení o dosažení fyzikálního targetu při 1% authority.

Příkazy jsou rekonstruované před integer PWM quantization a před dynamikou
SITL motor/prop plant. Inverse actuator consistency kontroluje AP slew
a thrust linearization stage, ne měřený rotor tah/torque ani modelovou
motor rychlost. UAMT současně uchovává skutečný simulator PWM pro pozorování.

`default-stop-final`: stejný stress s S_AW=0 vyvolal fault6 při yaw limit4.
1259 post-fault samples mělo neměnný UN=2883, APN=0. Žádný další UTB PID
nebo automatický AP fallback. Explicitní LAND cleanup je mimo fault
measurement a následný standardní disarm respektuje AP land detector.

První default-stop harness ukončil proces při očekávaném faultu příliš
brzy pro flush logu. Druhý cleanup požádal o disarm před land_complete
a AP jej správně odmítl; třetí očekával ARMED heartbeat, přestože AP po
impactu už disarmoval. Tyto runs jsou zachované jako harness/evidence FAIL,
ne jako úspěšné celý experiment. Finální observer toleruje ARM stav během
explicitního LAND cleanupu, stále vyžaduje standardní disarm a nikdy force.
C++ fault/arming mechanismus kvůli tomu nebyl změněný.

AW-specific DISARMED guards PASS: scripting odmítnuté při entry, enable
za běhu vytvoří CONFIG latch, AP owner nemůže zvolit UTB gating; bez
arming a bez output demand. Nový UNIT test také prokazuje clipping raw>1,
feedback residual včetně tohoto clippingu a povolený reverse unwind.
Active clipping>1 nebyl vyvolán na tuning gains; aktivní stress testuje
AP yaw allocation limit, především jeho directional feedback.

![Actual SITL saturation and gating](../tmp/utb-2b-20261014_164442/plots-final/yaw-saturation.png)
