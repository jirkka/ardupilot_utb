# FÁZE 2G: validace pilotního ACRO a přepínání s ANGLE

## Rozsah důkazu

Skutečný ArduPilot SITL, model `bfxrev`, frame 1/18, 400 Hz, reálná AP RC cesta.
Jde o omezenou funkční validaci simulace, nikoli o schválení hardware letu nebo
agresivního ACRO naladění. Fyzický FC nebyl připojen, parametrizován ani flashován.

Evidence: `tmp/utb-2g-20261017_165925/`. Hlavních 20 scénářů v
`verified-runtime-results.json` prošlo na ELF SHA-256
`5834d2df9f675c2b1cd2eccaa2d9b0a1b8f42835627f61c1c58afdc089b763eb`.
Následující změna `Copter::get_available_mode_enabled_mask()` doplnila pouze
GCS cache bitmask pro režimy 42/43. Regulační zdroje zůstaly identické
(`controller-source-equivalence.json`); doplnění šablon komentářů nezměnilo
C++ tokeny (`comment-completion-equivalence.json`). Finální ELF má SHA-256
`3ff4501f80f6736c5e5b7b44684431fe31132540654cacc8beee6866d7cb8ebc`.
Na něm byl zopakován SHA-vázaný DISARMED gate a kombinovaný nenulový přechod,
GCS katalog a následné regrese. Tyto různé hashe nejsou zaměňovány.

## Unit a integrační scénáře

124 unikátních C++ testů: core 47, ACRO RC 12, ANGLE RC 13, AP allocation 9,
BENCH 12, matematika 4, attitude 14, SITL rate 9, synthetic 4.
Samostatné spuštění math přeskočí CSV vstup; opětovný math validační běh jej
skutečně provedl: 30 946 vzorků, 29 scénářů, 0 chyb. Python: 142 PASS.

Hlavní `runtime-verified` scénáře, každý samostatný vlastněný proces:

| Scénář | Ověřený výsledek |
|---|---|
| disarmed | skutečné mapování a oba směry všech os, nulové výstupy, mode40 normal/force-arm zamítnut |
| roll / pitch / yaw | oba směry plného podporovaného stick rozsahu, vystředění, explicitní ANGLE recovery |
| transitions, transitions-pitch, transitions-yaw | přímé GCS/RC přepnutí s drženým R+0,3, P−0,3 nebo Y+0,5 |
| transitions-combined | současně R+0,3/P−0,3/Y+0,5, přímé opakované přepnutí |
| transitions-throttle | malé změny 1475/1469/1472 PWM bez výškového regulátoru |
| rc-loss / gcs-loss | mode43 → původní AP RTL → LAND/disarm, bez automatického UTB návratu |
| angle-rc-loss / angle-gcs-loss | totéž pro mode42; explicitní zdravý DISARMED re-entry má I/D/output=0 |
| guards | neslučitelné FS, FSTRATE, kalibrace, SHADOW, gyro switch; nezávislost nevybraného mode |
| limits | skutečný AP param read/write, limity 0,10/0,08/0,03 a 0,15/0,15/0,05 rad/s |
| ground | standardní rudder arm/spool/disarm přímo v mode43, bez předchozího GUIDED |
| remap-roll | RCMAP 6/7/8/3 a reversed roll na kanálu 6 |
| ap-roll / ap-pitch / ap-yaw | samostatná původní AP ACRO1 se skutečným UTB_ENABLE=0 |

Invalid/stale RC, NaN/Inf target, neplatná alokační zpětná vazba, chybný stav,
PID priming/reset a latched fault pokrývají také zachované unit sady.
SITL gyro změna byla provedena DISARMED; úplné selhání všech IMU/AHRS za letu
není tímto testem doložené a nemá tvrzení o bezpečném zotavení.

## Naměřené reference a gyro

| Osa | Ověřený max absolutní target [rad/s] | Max absolutní gyro [rad/s] | Max RP náklon v ACRO [°] |
|---|---:|---:|---:|
| Roll | 0,15 | 0,178730 | 10,327 |
| Pitch | 0,15 | 0,186405 | 10,388 |
| Yaw | 0,05 | 0,050696 | 0,259 |

Target cap je mez požadavku; není fyzikální mez skutečného gyro překmitu.
0,3 rad/s RP ani 0,1 rad/s yaw pro pilotní ACRO nebyly rozšířeny ani schváleny.
Nominal allocation a integration-block masks byly 0; saturační regrese je
oddělený scénář původního adaptéru. PID matematika ani gains se neměnily.

Po vystředění roll/pitch se target rovná nule, průměrná body rate v závěrečném
0,4s okně je pod 0,015 rad/s, náklon zůstává přibližně ±10,3°. Například
roll+ drží 10,237° s gyro 0,000435 rad/s a drift −0,00998° za 0,4 s.
Přepnutí do ANGLE následně aktivně vyrovná orientaci; ACRO to samo neprovádí.

![Nulová rate při nenulovém náklonu](../tmp/utb-2g-20261017_165925/plots/zero-rate-hold.png)

## Odezvy a srovnání s původním AP

Kontrolované časové okno od měřeného target onset: RP 1,0 s, yaw 1,8 s.
UTB onset je první nenulový target; AP onset je první měřený target nad 1 %
stejné nominální amplitudy. Není to přesné měření latency od prvního RC paketu.
Rise time je interval gyro 10–90 % nominální amplitudy; překmit je gyro peak
nad ní. Settling vyžaduje zůstat v ±2 % aspoň 0,25 s do konce okna.
`nedosaženo` se neinterpretuje jako PASS settling. Steady error je průměr
skutečný controller target minus gyro za posledních 0,5 s.
IAE = ∫|raw nominální target − gyro|dt [rad], ITAE = ∫t|e|dt [rad·s], t od onset.
`rate-comparison.json` navíc uvádí IAE ke skutečnému controller targetu,
P+I+D maximum a I maximum. Starší `analysis.json` má navíc metriky přes celé
mode okno včetně návratu k nule; tyto delší integrály se s tabulkou nemíchají.
Sdílená položka `windows.analysis.angle` v ACRO jen diagnosticky integruje Euler
od nuly. Není to chyba attitude regulátoru ani ACRO tracking metrika; hodnocení
ACRO používá skutečný rate target/gyro a samostatné `acro`/rate-comparison metriky.

| Osa | Řízení | Směr | Rise 10–90 [ms] | Překmit [%] | Settling ±2 % [s] | Steady error [rad/s] | IAE [rad] | ITAE [rad·s] |
|---|---|---|---:|---:|---:|---:|---:|---:|
| roll | UTB | + | 17.493 | 17.730 | nedosaženo | 0.001488 | 0.006417 | 0.001069 |
| roll | UTB | − | 14.994 | 19.154 | nedosaženo | -0.000604 | 0.006245 | 0.000949 |
| roll | AP | + | 62.475 | 3.296 | 0.367 | -0.000230 | 0.008808 | 0.000859 |
| roll | AP | − | 62.475 | 4.179 | 0.506 | 0.000458 | 0.008678 | 0.000802 |
| pitch | UTB | + | 14.994 | 22.224 | nedosaženo | 0.002199 | 0.006447 | 0.001222 |
| pitch | UTB | − | 14.994 | 24.270 | 0.243 | -0.000826 | 0.005387 | 0.000591 |
| pitch | AP | + | 64.974 | 3.494 | 0.367 | -0.000308 | 0.008754 | 0.000749 |
| pitch | AP | − | 63.308 | 5.115 | 0.453 | 0.000407 | 0.008813 | 0.000878 |
| yaw | UTB | + | 544.782 | 0.758 | 0.915 | 0.000198 | 0.012464 | 0.002997 |
| yaw | UTB | − | 482.307 | 1.115 | 0.960 | 0.000259 | 0.011897 | 0.002832 |
| yaw | AP | + | nedosaženo | 0.000 | nedosaženo | 0.012104 | 0.032928 | 0.022396 |
| yaw | AP | − | nedosaženo | 0.000 | nedosaženo | -0.013117 | 0.034248 | 0.023552 |

AP konfigurace: `ACRO_RP_RATE=8.594367 deg/s`, `ACRO_Y_RATE=2.864789 deg/s`,
expo/trainer/balance=0, `ACRO_OPTIONS=2` (původní rate-loop-only volba),
`ACRO_THR_MID=0` (stejná původní hover mapa), rate TC parametry=0.
Původní AP input-rate metoda přesto vytváří měřený přechod targetu; původní
filtry/akcelerační omezení/PID zůstaly aktivní a nebyly napodobovány v UTB.
UTB target je přímo lineární raw=target bez shaping. Proto nejde o izolované
srovnání dvou PID při identickém vstupu. AP yaw v tomto krátkém okně nedosáhlo
90 % nominálního požadavku; z toho nelze obecně usuzovat, že je AP horší.
Skutečné BIN PARM baseline potvrzuje původní AP RP P/I/D=0,135/0,135/0,0036,
IMAX0,5, FLTT/FLTD20Hz; yaw P/I/D=0,30/0,02/0 a FLTE2,5Hz.
UTB má zachovaný pilotní preset: R0,19/0,16/0,0015, P0,19/0,16/0,001,
Y0,30/0,60/0, IMAX0,19, D filtr R15/P20/Y20Hz. Nejsou to stejné gains;
native-baseline-parameters.json uchovává skutečné AP parametry z logu.
Proto zejména rozdíl yaw odezvy nelze připsat jen architektuře.

Každý směr představuje jeden doložený běh, nikoli statistiku opakovaných pokusů.
UTB RP je v těchto bězích rychlejší, ale s překmitem 17–24 % a u tří ze čtyř kroků bez doloženého
2% settling. Zisková sada se v této fázi nezměnila; tuning zůstává otevřený.

![Rates, původní AP a UTB P/I/D](../tmp/utb-2g-20261017_165925/plots/rate-comparison.png)

## Přechody a ruční throttle

Pět hlavních přechodových scénářů obsahuje 22 skutečných přímých hran 42↔43.
Vyhodnocují se skutečné MODE.TimeUS, spárované TimeUS/Seq a první nový priming
vzorek. Sample se stejným časem jako MODE ještě může patřit původnímu mode;
proto interval používá start < sample ≤ další MODE.TimeUS.

- Max bezprostřední PWM rozdíl na hraně: **3 µs**.
- Max jednocyklový PWM rozdíl v okolí ±100 ms: **31 µs**.
- Max jednocyklový rozdíl normalizovaného momentu v témže okolí: **0,034951**.
- Max rozdíl skutečného collective ΣUALT.T/(4·G) na hraně: **2,25·10⁻⁸**.
- Předchozí integrátor dosáhl absolutně **0,006859**; první nový I/D/output je 0.
- Max náklon pěti scénářů **4,148°**, max |vz| **0,1163 m/s**, drops 0.

`UARC.Out` je interní primovaný throttle snapshot a první cyklus může být 0,
ačkoli AP ještě drží předchozí manual požadavek. Pro skutečnou throttle
kontinuitu se proto používá UAST.Thr a rekonstruovaný collective, ne samotný
první UARC.Out. Nejde o dokonale spojitý přenos integrátoru: historie se
resetuje a může vzniknout konečný omezený P krok. Naměřený výsledek připouští
pouze tyto krátké malé přechody, ne libovolný náklon/throttle/vysoké rates.

![Nenulové přechody, gyro, integrátor a motor PWM](../tmp/utb-2g-20261017_165925/plots/nonzero-transitions.png)
První bod throttle grafu je vstup do ANGLE po GUIDED přípravě, ještě při změně
vstupního RC/throttle kontextu (přibližně 0,39→0,364). Není to hrana42↔43.
Uvedená kontinuita se vztahuje na pozdější přímé pilotní přepnutí při shodné páčce.

![Společná manual throttle mapa](../tmp/utb-2g-20261017_165925/plots/throttle-continuity.png)

## Failsafe, opravená re-entry podmínka a otevřená rizika

Původní radio/GCS event handler synchronně přepíná do původního AP RTL před
návratem do hlavní regulační cesty. Ověřeno pro 42 i 43 s FS_GCS_ENABLE=1,
FS_THR_ENABLE=1, FS_OPTIONS=0. GCS loss má samostatný nativní UDP RC; žádné
pokračující MAVLink overrides, které by aktualizovaly GCS last-seen.
Po reconnectu není automatický návrat do UTB, až explicitní zdravý re-entry.
FS_OPTIONS=16 a další neslučitelné konfigurace jsou odmítnuty na vstupu;
runtime mutace latchuje fault, není to letový recovery mechanismus.
Nulový tah při libovolném jiném UTB faultu může způsobit pád.

Původní přísná kontrola raw throttle při DISARMED odmítla návrat po nativním
LAND: původní vertikální PID měl malý záporný raw požadavek, skutečné výstupy 0.
Diagnostika a původní neúspěšný test jsou zachované. Minimální úprava ověřuje
v tomto neaplikovaném kontextu konečnost; při ARMED stále 0..1. Nový unit test
a opakování všech čtyř failsafe scénářů prošly. Hodnota se do UTB nepřenáší.

GCS SET_MODE i RC FLTMODE výběr prošly. Katalog 435 byl ověřen jednotlivými
REQUEST_MESSAGE indexy a monitor 437 mění seq 0→1→2→3→4 při C/A_ENABLE 0/1.
Pokus o hromadný index0 v lokálním nativním streamu vrátil jen první položku;
původní GCS streamer se neopravoval. Konkrétní Mission Planner GUI a jeho
strategie dotazování nejsou ověřené. Původní FLTMODE_GCSBLOCK bitové indexy
nebyly změněny; nové UTB pilotní módy nemají vlastní dokumentovaný block bit.

Buildy a úplné regrese jsou rozepsané v [souhrnu](UTB_PHASE_2G_SUMMARY_CS.md).
Žádný nový log ID, AP_Motors/HAL/DShot/EKF zásah ani hardware autorita.
APN/AN=0 jsou zdrojové deklarace dispatch vlastnictví, nikoli globální profiler.
Host execution časy ze SITL nejsou H743 CPU/worst-case timing důkaz.

## Omezení statického GCS seznamu FLTMODE

Původní `ArduCopter/Parameters.cpp` uvádí ve statickém `FLTMODE1 @Values`
z UTB pouze historický mode40; ani42 ani43 tam nejsou. Runtime AP_Int8 zápis
`FLTMODE2=42`, `FLTMODE3=43` a skutečné RC přepnutí fungují a byly ověřeny.
Klient používající statický seznam nemusí nabídnout nové položky v grafickém
dropdownu. V SITL lze použít číselný zápis v úplném seznamu parametrů nebo
standardní SET_MODE43/42; dynamický AVAILABLE_MODES katalog má položky obou modes.
Globální hardware metadata nebyla rozšířena o nedostupné aktivní SITL režimy.
Konkrétní Mission Planner GUI a automatické načtení jeho seznamu zůstávají neověřené.
