# FÁZE 2F: validace pilotního UTB ANGLE v SITL

Datum 17. 10. 2026. Rodič pracovních změn `9824542e73bbfb193b21fd4388f74e14ba24439c`.
[Architektura](UTB_ANGLE_MODE_ARCHITECTURE_CS.md) · [Souhrn a návod](UTB_PHASE_2F_SUMMARY_CS.md).

**Rozsah: omezený experimentální RC ANGLE prototyp uvnitř skutečného
ArduPilot SITL, nikoli aktivně ověřený hardware autopilot.** Náklonová
reference má normu nejvýše 5°, RP rate nejvýše 0,3 rad/s, yaw nejvýše
0,1 rad/s. Skutečně letově testovaný yaw target je ±0,05 rad/s.

Důkazy jsou v [samostatném archivu](../tmp/utb-2f-20261017_093117/).
Ignorované tmp soubory nejsou součástí Git commitu a musí se archivovat zvlášť.
Platné finální lety jsou v `runtime-native-trim/`; počáteční pokusy v jiných
adresářích nejsou zaměnitelné za finální výsledky.

## Reprodukovatelnost a čas

Pilotní executable hlavní krokové matice před závěrečným seskupením logových deklarací:
`builds/sitl-angle-native-throttle/arducopter`, 5 772 656 B,
SHA-256 `f744d99afb8cd8b22762aa9cea4db27faa8fa5e494ebbf22537f80f098818d08`.
Model `bfxrev`, rám 1/18, loop 400 Hz, speedup 1, skutečné RC override přes
AP RC_Channels; pro oddělený GCS test nativní simulovaný UDP přijímač.
Žádná reference se nevpichuje přímo do PID.

Každý scénář uchovává BIN, telemetrii, events, marks a dekódované packets.
Analýza spojuje TimeUS/Seq přes UARC, UAAT, UAST, 3×UARX, UAMT, 3×UAFB,
UALT; chybějící sady ani nesprávný gain preset nejsou přijaty.
Hlavní krokové scénáře vznikly před přidáním run-config exportu; jejich
provenience je uvedena souhrnně a shodný executable je zachován.

## Testované vstupy a návrat do roviny

Před lety prošla DISARMED integrace: opt-in, GCS mode 42 a RC5 přepínač,
R/P/Y oba směry, kruhový clamp, throttle sweep 1000–2000, nulové povely
při DISARMED, mode 40 normal/force arm odmítnutí, chybný RC rozsah a latched
stop/reentry. Force příkaz se používá pouze pro negativní test mode 40.
Vlastní letové armování je standardní AP bez force.

Krokové lety obsahují +úhel → 0 → −úhel → 0. Analýza ověřuje při puštění
páčky nulový angle target, opačné znaménko body rate targetu proti náklonu
a následný aktivní PID korekční povel. Nejde pouze o pasivní návrat modelu.

![Skutečné úhlové kroky](../tmp/utb-2f-20261017_093117/figures/angle-steps.png)

![Kombinované vstupy](../tmp/utb-2f-20261017_093117/figures/combined-inputs.png)

Grafy existují také jako SVG a PDF. Číselné metriky a seznam scénářů jsou
v následujících oddílech a strojovém souhrnu archivu.

## Throttle a přechody

Finální režim používá původní AP manual throttle mapu a hover parametr,
nikoli dřívější převzatý pevný kolektiv. V daném modelu se jako testovací
pilotní pracovní bod použilo PWM 1472; krátké kroky1475/1469 proběhly současně
s roll/pitch. To není přenositelná kalibrace fyzického dronu.
Žádný angle boost nebo UTB altitude controller není aktivní.

Ověřené přechody zahrnují DISARMED GCS/RC a za letu STABILIZE → ANGLE →
STABILIZE → ANGLE, i RC switch do STABILIZE. Nový vstup začíná Seq 1,
I/D/output 0; staré PID historie nejsou přeneseny. Spool a throttle kontext
jsou původní AP. Jediný fast-slot motor output se nerozdvojuje.

## Bezpečnost a neúspěšné pokusy

- První speedup3 pokus měl neúplné logy; není použit pro regulační závěry.
  Finální pilotní testy běžely speedup1.
- Původní prototyp s přemapováním entry hover neumožnil srovnatelný přechod
  do STABILIZE. Byl nahrazen původní AP throttle mapou; nešlo o změnu PID.
- Pilotní trim 1470 při delším letu porušil svislou obálku a vyvolal správný
  VERTICAL stop. Finální testovací vstup1472 udržel původní limity; guard se
  nezměnil. Neexistuje automatické držení výšky.
- Test GCS „heartbeat off, override on“ nebyl platným výpadkem: override
  obnovuje GCS timestamp. Zachován jako neúspěšný testovací pokus.
- Skutečná GCS ztráta při FS_OPTIONS=16 způsobila „Continuing Pilot Control“
  a vlastní latched fail-stop 7 s následným pádem modelu. Tato konfigurace
  není podporovaný bezpečný letový kontrakt. Samostatný test používá
  původní nativní RTL politiku s explicitním FS_OPTIONS=0.
- Full SITL logging-disabled build blokuje původní nechráněné
  `AP::logger()` v `libraries/AC_CustomControl/AC_CustomControl.cpp:190`.
  Původní soubor nebyl změněn. Obě nové translation units samostatně
  s HAL_LOGGING_ENABLED0 prošly; celý H743 logging-disabled build prošel.

## Co důkazy neprokazují

APN/AN0 jsou deklarace výhradní dispatch větve, nikoli nezávislý globální
profiler všech AP funkcí. Zdrojový audit a skutečné vlastní controller/motor
počty dokazují testovanou cestu; dynamické hooky do původních AP regulátorů
nebyly přidány. Uvedené počty motor outputů platí pro fast slot, nikoli
například arming `output_min()` mimo něj.

NaN/Inf, PID invalidity a chybné allocation snapshoty jsou cíleně pokryté
unit testy původních helperů. Ne všechny takové chyby byly injektovány
během letu mode 42. Fail-stop není obecná bezpečná letová obnova: může
znamenat pád simulovaného dronu. Nelze jej považovat za hardware failsafe.

Host wall časy a PM/SITL perioda nejsou WCET, CPU load ani hard deadline
výsledky H743. Nový log UARC stojí59 B/cyklus, dalších23,6kB/s při400 Hz.
Post-output logging zůstává pouze v aktivním SITL kontraktu.

Neověřeno: ±10° a širší rozsah, plný manuální RC takeoff/landing,
obecné failsafe kombinace, saturace na hraně motorů v celém letovém rozsahu,
fyzikální shoda dnešního 5″ dronu, aktivní H743 let, vlastní altitude/XY/AUTO.
H743 compile-out je ověřen sestavením a kontrolou ELF/stringů, nikoli letem.

Závěrečné seskupení deklarací pouze v novém logovacím callbacku dalo
executable `builds/sitl-angle-final/arducopter`, SHA-256
`b084edeb3d82c1a8f53dfb6ba8a22a8a45b7d9a707f45d3610420550766cad4e`,
5 772 656 B. Regulátory, pilotní mapa, run a rate callback se tím neměnily.
SITL text narostl o192 B; prolog log_cycle rezervuje0x208 místo0x148 B
(+192 B lokální stack oblasti, bez započtení vnořených volání).
Není to měření stack watermarku. H743 tuto funkci vůbec neobsahuje.
Finální runtime a regrese používají tuto jasně oddělenou binárku;
run-config a strojový souhrn uvádějí provenienci každého scénáře.

Delší GUIDED otočení před vstupem změnilo původním AP hover learningem
mapování throttle: stejná páčka1472 znamenala po této přípravě kolektiv
přibližně0,354 místo potřebného0,364. Test heading45-feedback proto po
2,6s v ANGLE zastavil VERTICAL guard. Tento neúspěch není změnou znamének
attitude matematiky ani důvodem k rozšíření svislé obálky. Samostatný
opakovaný experiment volí jiný explicitní ruční pracovní bod pro tuto
přípravu; nejde o automatické výškové řízení.

## Naměřená hlavní kroková matice

Následující běhy používají executable f744d99a… před čistě logovou úpravou
deklarací. Výsledky posledního executable b084edeb… jsou vedené samostatně.
Pořadí čtyř kroků je +úhel, střed, −úhel, střed. Náběh je10–90 %,
settling2 % vyžaduje zbytek plateau v pásmu a alespoň0,25s dwell. Steady
error je průměr posledních0,5s. Kritérium0,15° je analytická tolerance
sledování malých kroků, nikoli nový bezpečnostní limit firmware.

| Scénář | Úplné sady | Náběh10–90 % [s] | Settling2 % [s] | Největší překmit | Max. absolutní steady error |
|---|---:|---:|---:|---:|---:|
| Roll ±3° + středy | 7202 | 0,667–0,703 | 1,102–1,225 | 0,659 % | 0,0183° |
| Pitch ±3° + středy | 7201 | 0,665–0,706 | 1,102–1,230 | 0,808 % | 0,0162° |
| Roll ±5° + středy | 7202 | 0,687–0,703 | 1,152–1,233 | 0,259 % | 0,0406° |
| Pitch ±5° + středy | 7198 | 0,695–0,711 | 1,200–1,238 | 0,582 % | 0,0079° |

Každý obsahuje oba skutečné návraty do roviny s ověřeným korekčním body
rate targetem a PID povelem. Úplná frekvence400,0118–400,0119 Hz je odhad
z rozdílu logových TimeUS; není to přesné fyzické časování host CPU.
DSF.Dp=0 v těchto scénářích; alokační limit masky0, tedy nejde o test
agresivní saturace. Nový gain preset je kontrolován z identity
P=Kp×(rate target−gyro), nikoli jen z uložených shared RAT parametrů.

Kombinovaný let:7841 úplných sad,400,007 Hz, RP(+3°,−3°) a opačné znaménko,
yaw±0,05rad/s, střed a malé throttle kroky. RCMAP6/7/8/3 a RC6_REVERSED1:
7040 úplných sad,400,0085 Hz; prokázána skutečná změna fyzického kanálu
s normálním AP reverse a návraty do roviny.

## Původní AP STABILIZE baseline

Původní STABILIZE má30° mapu a svůj input shaping/angle boost. Pro3° je
proto normalizovaná páčka0,1, zatímco UTB potřebuje0,6. Nejde o srovnání
stejné polohy páčky. AP ATT a RCIN jsou v původní konfiguraci logované10 Hz;
rekonstrukce raw stick targetu proto má nejistotu přibližně0,1s a není
samplově rovnocenná400 Hz UTB referenci. Samostatné CSV/JSON uchovávají
skutečný čas i integrální/RMS chyby; pro IAE se používají skutečné rozdíly
ATT timestampů. Původní logger nebyl zrychlen, aby se neměnil baseline.

Roll3 baseline:180 ATT vzorků, náběh0,300–0,400s, settling0,500–0,701s,
překmit do 1,313 %. Tyto výsledky samy nejsou důkazem lepšího obecného
regulátoru; shaping a časové rozlišení se liší. Pitch a finální doplňkové
scénáře jsou uvedeny ve strojovém souhrnu.


## Finální executable a doplňkový rozsah

Po seskupení deklarací prošla znovu kompletní DISARMED suite a Roll±5° se
7200 úplnými sadami. Náběh0,685–0,706s, settling1,152–1,220s, překmit do
0,337 %, max. absolutní steady error0,0086°, frekvence400,0119Hz, DSF.Dp=0.
Host producer mean 2,715µs / max 238µs, capture/logger mean 2,750µs / max 98µs.
To jsou wall doby včetně host preempce, nikoli izolovaná cena PID nebo WCET.

Heading45 s explicitním ručním trim 1484 a A_AW=1:7598 úplných sad, body RP
oba směry při měnícím se headingu přibližně44–53°, yaw±0,05rad/s, drops0,
|vz|max0,119m/s, výškový span 0,627m. Jde o funkční sledování a alokační
feedback v malé obálce. **Ne všechny kombinované/krátké plateau dosáhly
2% settling.** Jeden3° roll plateau měl tail přibližně0,061°; jedno-sekundové
2° throttle pulsy jsou kratší než typické ustálení. Tyto výsledky se
nezaměňují za úspěch všech nominálních metrických kritérií.

Původní AP Pitch3 baseline:176 ATT vzorků, náběh přibližně0,300s,
settling0,500–0,601s, překmit do 1,550 %, stejná nejistota timestampů0,1s.
Všechny IAE/RMS a jednotlivé P/I/D/rate výstupy jsou v `evaluation/*.json`
a CSV, ne pouze v grafech.

Ground-spool-gyro v konečném executable: standardní ARM log Method=0
(RUDDER), Forced=0, následný stejný standardní disarm; prošly spool stavy
0/1/2/3/4. Změna EK3_PRIMARY0→1 při DISARMED vyvolala stop 4 ATTITUDE a latch,
nikoli přenos staré historie. Jednotlivé AHRS/gyro health chyby nebyly všechny
injektované zvlášť během letu. Ruční konzolový parser byl skutečně spuštěn
s příkazy status/mode 42/status/mode0/quit za DISARMED; samostatný GUI joystick
se netestoval.

H743 compile-out byl po poslední změně znovu sestaven: APJ1 129 203 B,
SHA-256 `c643a1598ddb4b2526ab3d4d58add280c4ccdea1d4fb46d979dbdac428924e92`.
Sekce text/data/BSS proti předchozímu checkpointu mají shodné velikosti
1 246 044 /5020 /120 088 B. V SITL je proti checkpointu navíc text 19 648 B,
rodata 704 B, data 864 B a BSS 1280 B. Nejde o navýšení hardwarového runtime.

![Finální rate, PID a motorové výstupy](../tmp/utb-2f-20261017_093117/figures/final-rates-motors.png)

Graf používá skutečný finální Roll±5° běh s executable b084edeb…;
první throttle output zachovává předchozí AP kontext, následně je ruční.


Původní SHADOW příkaz nejdříve selhal na statickém počítání textových
výskytů `utb_log_update()`: dvě skutečné definice/volání a jeden Doxygen
popis. C++ bridge je proti HEAD nezměněný. Cílená oprava pouze testovacího
parseru nahrazuje komentáře mezerami při zachování string/char literálů a
řádků; stejné runtime scénáře i všechny bezpečnostní assertions zůstávají.
Dvě nové unit regrese ověřují ignorování komentáře a zachování skutečné
motorové závislosti v kódu. Aktuální Python sada má 135 testů. První chyba
ani doplňkový nezávislý audit se nemažou; původní celý příkaz po opravě prošel se všemi původními runtime assertions.

## Závěrečná regrese a build matice

| Kontrola | Výsledek / přesný rozsah |
|---|---|
| C++ | PASS:47 core +9 allocation +12 bench +14 attitude +9 rate +4 synthetic +4 math +13 angle RC =112. Oba export testy skutečně spuštěné s příslušným environment; žádné přeskočení. Starší součet98 zahrnoval3 synthetic bez volitelného exportu. |
| Python | PASS:135 testů, včetně7 nových; offline audit evidence z historického archivu dodaná explicitně. |
| Nezávislá matematika | PASS:30 946 vzorků/29 scénářů, byte-identický C++ CSV výstup proti ověřenému srovnání. |
| Původní FÁZE0/guards | PASS:DISARMED integrace, old mode40 isolation a původní mode41 config/FSTRATE/lifecycle guards. |
| Původní RATE/ATTITUDE | PASS:3200 roll +3200 pitch RATE sad;5600 attitude roll sad. Původní generátory a matematika nezměněny. |
| Původní AW guards | PASS:scripting/config/owner konflikty při DISARMED. |
| Původní AW saturation | PASS:8000 úplných sad,1853 yaw blocked cycles (4,632s), projekční chyba<9,49e-8, I recurrence chyba<2,98e-8. Default stop: fault6,1259 stopped samples, PID call count se po fault nezvyšuje. Stresový weak-yaw model není identifikovaný skutečný dron. |
| SHADOW úplný původní příkaz | PASS po cílené opravě komentářového parseru:200,0037 Hz/5243 sad a399,9933 Hz/2400 sad; suppression/logger-off, sysid, gyro switch, reference mapping a UTB ON/OFF motor isolation. |
| Metadata | PASS:logger parser i AP_Param parser bez chyb; runtime validate_structures prošel. |
| Flake8 / astyle / diff | PASS:všechny3 upravené Python zdroje,6 nových C++ souborů a git diff check. Nové untracked soubory zvlášť ověřené na LF/trailing whitespace. |
| Nezměněné regulátory | PASS:12 uložených SHA-256 původní matematiky/adaptérů shodných; žádné AP_Motors/HAL/DShot/EKF/arming změny. |

SHADOW regrese úmyslně vyvolává plnou frontu a vypnutý logger. Její queue
drops a HWM4 nejsou nominální ztráty nového pilotního letu ani H743 výsledek.
První spuštění s chybným textovým auditem i obnovený doplňkový běh jsou
zachované vedle čistého konečného spuštění `regressions/shadow-final/`.

| Build | Výsledek / artifact |
|---|---|
| SITL UTB ON / active ON / BENCH ON | PASS:konečný b084edeb…, kontrolní DISARMED a Roll5, heading45/AW1, ruční parser a staré runtime regrese. |
| SITL UTB ON / active OFF / BENCH ON | PASS:9409f89a…; skutečná absence mode41/42 a všech4 ANGLE parametrů; nativní módy0/1/2 dostupné. |
| SITL UTB OFF / active OFF / BENCH OFF | PASS:bbc68a3e…; stejný negativní runtime test. |
| H743 UTB ON / active forced ON / BENCH ON | PASS:finální APJ c643a159…; aktivní ANGLE/rate/attitude symboly a ANGLE parametry/log nejsou přítomné. |
| H743 UTB OFF / BENCH OFF | PASS:APJ5e68a928…; compile isolation. |
| H743 UTB ON / BENCH ON / logging OFF | PASS:APJ424eb2a9…; žádná aktivní autorita. |
| Nové2 translation units při HAL_LOGGING_ENABLED0 | PASS:aktuální source, samostatné objekty mimo Waf build tree. |
| Celý SITL při logging OFF / default custom control ON | FAIL:původní AC_CustomControl.cpp:190, nezměněná nativní logger závislost. Není opraveno mimo scope2F. |
| Průběžný SITL BENCH OFF | Build PASS:d6756933… ze starší pilotní varianty. Není použit jako finální funkční důkaz nového throttle/lifecycle kontraktu. |

OFF varianty byly sestavené před závěrečnou úpravou deklarací v log callbacku;
tato úprava je celá uvnitř jejich compile-out větve. Finální H743 cross-build
byl zopakován po úpravě a má bitově stejný APJ. Přesné build logy a SHA
všech variant jsou v archivu, žádné tvrzení o fyzickém uploadu.

V obou kombinovaných letech nebylo2% settling prokázáno u všech plateau;
roll tail při3° referenci byl přibližně0,061–0,063°. Funkční tolerance0,15°
byla splněna. Tento rozdíl není přepsán na nominální2% PASS.
Samostatná kontrola skutečných dat prokázala yaw průměry+0,04985/−0,05041rad/s
při targetech±0,05. Změna ručního kolektivu0,36437→0,36708/0,36166 zvýšila/
snížila průměrný motorový PWM z1580,09 na1582,04/1577,99µs.
To je konkrétní odezva modelu, nikoli jen přijatý RC příkaz.
