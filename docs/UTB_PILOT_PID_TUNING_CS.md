# Ladění společného pilotního Rate PID a ANGLE P, FÁZE 2H

Stav k 18. 10. 2026. **Aktivní řízení pouze SITL. Hardware tune neověřený.**
FÁZE 2G byla po kontrole všech 26 SHA-256 a Git blob ID uložena do lokálního
commitu `e9a003cb532c65c0acc54b5bd4f5930d4ce0fdd8`.
Výsledky této fáze jsou v `tmp/utb-2h-20261018_093406`; zdrojový manifest a úplný seznam změn
uvádí [souhrn 2H](UTB_PHASE_2H_SUMMARY_CS.md).

## Implementováno

- Samostatných 17 parametrů `UTB_P_*` pro společný Rate PID módů42/43 a
  vnější ANGLE P. Původních 45 AP_GroupInfo položek včetně indexů a defaultů
  je shodných; nové indexy46–62. Mode41 a SHADOW nadále čtou původní RAT_*.
- Gains se načtou při vstupu a za běhu se porovnávají s entry snapshotem.
  Změna vyvolá latched CONFIG stop. Obnovení hodnoty stop neruší; nutný
  explicitní výstup a nový vstup. ACRO nezávisí na nepoužitém ANGLE P.
- Parametrická validace zachovává původní matematické kontroly. Horní
  experimentální meze P0,3/I0,6/D0,004/IMAX0,19/D_HZ40 a ANGLE P1–6
  nejsou důkazem stability celého nastavitelného prostoru. D>0 vyžaduje
  kladný cutoff. Ověřený je konkrétní tune a níže uvedené scénáře.
- PID, D filtr, anti-windup, vlastní attitude matematika, pilotní mapy,
  throttle, AP_Motors, HAL/DShot, EKF, arming a failsafe algoritmy se nemění.
  Žádná nová DataFlash zpráva ani zápis v regulační cestě.
- Historický `AP_UTB_AngleRC::gains()` zůstává jako jednotkový preset2F/2G;
  aktivní pilot čte AP parametry přes `AP_UTB::pilot_gains()`.

## Nové výchozí gains a GCS

| Osa | P | I | D | IMAX | D cutoff [Hz] |
|---|---:|---:|---:|---:|---:|
| RLL | 0.095 | 0.24 | 0.00075 | 0.19 | 25 |
| PIT | 0.1425 | 0.24 | 0.0015 | 0.19 | 35 |
| YAW | 0.3 | 0.6 | 0 | 0.19 | 20 |

Vnější ANGLE P: Roll **3.25**, Pitch **3.25**. Yaw gains se nemění.


P/I/D/IMAX a cutoff se nastavují pomocí `UTB_P_RLL_*`, `UTB_P_PIT_*`,
`UTB_P_YAW_*`; suffixy P/I/D/IMAX/D_HZ. Vnější zisky jsou
`UTB_P_ANG_R` (Roll) a `UTB_P_ANG_P` (Pitch).
Nejprve zvolit původní AP režim, potom upravit hodnoty a znovu vstoupit do
pilotního režimu. Reboot pro změnu těchto gains není nutný. Původní boot/restart požadavek
pro globální `UTB_ENABLE` tím není změněný. Zachované
opt-in parametry mají nadále default0. Reference zůstávají ACRO
0,15/0,15/0,05rad/s, ANGLE kruh5° a normový body-rate cap0,3rad/s.

SITL AP_Param/MAVLink readback a metadata jsou ověřené. Samotné GUI Mission
Planneru nebylo obsluhováno; v běžném úplném seznamu parametrů lze v SITL
vyhledat `UTB_P_`. Na H743 tato pilotní banka vůbec není zkompilovaná.
Existující uložené pilotní hodnoty se nepřepisují automaticky při aktualizaci
firmware; pro reprodukci je nutné porovnat všech17 skutečných hodnot.

## Metoda: regulátor versus reference

Nejprve cutoff D, potom P, D a I při stejné skutečné lineární RC/body-rate
referenci ±0,15rad/s pro každou RP osu. Každý krok má stejný první1s
vyhodnocovací interval. Yaw se neladil. Kandidáti sdílejí stejný executable,
model, RC kalibraci, throttle a guardy. Žádný input shaping se nepřidává:
rate adaptér zůstává `configure(..., false)`. D_HZ filtruje derivaci gyra,
nikoli vstupní referenci. Teprve potom se mění vnější ANGLE P při stejných
krocích ±5° a návratech do roviny.

Koordinátní hledání není globální optimalizace. První hodnoticí funkce
penalizovala překmit>10%, chybějící2% settling, pomalý rise a IAE. Nižší
P/D snížily rate překmit; první I0,16 kandidát ale neposkytl dostatečný
kompromis pro rychlý ANGLE. Následovaly jemnější vnější zisky a systematická
revize I při pevném vnějším P. Celý seznam skutečných pokusů a metrik je
uložený v search-history/joint-* JSON souborech. Nevybrané pokusy se nemažou.

Vnější P přirozeně mění body-rate target. Při větším P může zasáhnout
**dosavadní** normový rate cap0,3rad/s; nejde o zvýšení tohoto limitu ani nový
shaping. To se vyhodnocuje odděleně od motorové saturace. Proti původnímu
baseline se porovnává ANGLE úhlová reference, nikoli identický výstup vnějšího
regulátoru.

Jednotlivé lety odděluje výslovně logovaný původní AP GUIDED návrat na10m
(bez změny UTB výškových guardů). Příprava/takeoff/GUIDED/LAND nepatří do
měřených UTB oken. Není tím implementován UTB position/ALT HOLD. Stejná
procedura platí před i po. Po přepnutí do UTB zůstává ruční throttle.

Rise je10–90%; peak/overshoot má znaménkově správnou normalizaci k
amplitudě. Settling2% vyžaduje zbytek intervalu v pásmu a alespoň0,25s
pobytu; nedoložený settling je `null`, ne PASS. Rate RP okno je1s; pomalejší yaw, ANGLE a
self-level mají2s. Starší pracovní1,8s ANGLE metriky byly znovu vyhodnoceny
ze zachovaných surových vzorků ve stejném2s okně pro všechny kandidáty.
IAE rate má jednotku rad, IAE attitude rad·s; ITAE přidává další sekundu.
Výsledky používají skutečné dt. Grafická interpolace nemění raw metriky.

Překmit ANGLE se posuzuje proti skutečnému opakovanému baseline, nikoli
proti libovolně zvolené dodatečné toleranci. P6 kandidát byl odmítnut,
protože sice podstatně zrychlil odezvu, ale zvýšil překmit.

## Naměřené porovnání v SITL

| Režim/osa | Tune | Rise10–90 [s] | Max překmit [%] | Settling2% [s] | Nedoložené / 6 | IAE průměr |
|---|---|---:|---:|---:|---:|---:|
| ACRO43 Roll | Před | 0.015–0.015 | 20.393 | — | 6 | 0.006480 |
| ACRO43 Roll | Po | 0.030–0.035 | 3.101 | 0.095–0.515 | 0 | 0.005607 |
| ACRO43 Pitch | Před | 0.015–0.015 | 24.853 | 0.190–0.385 | 1 | 0.005574 |
| ACRO43 Pitch | Po | 0.020–0.025 | 4.058 | 0.092–0.316 | 0 | 0.004902 |
| ACRO43 Yaw | Před | 0.495–0.522 | 2.694 | 0.907–1.609 | 0 | 0.012411 |
| ACRO43 Yaw | Po | 0.495–0.567 | 3.423 | 0.905–1.262 | 2 | 0.012864 |
| ANGLE42 Roll | Před | 0.706–0.720 | 0.000 | 1.200–1.284 | 0 | 0.029106 |
| ANGLE42 Roll | Po | 0.611–0.627 | 0.000 | 1.090–1.172 | 0 | 0.026690 |
| ANGLE42 Pitch | Před | 0.700–0.712 | 0.040 | 1.170–1.220 | 0 | 0.028510 |
| ANGLE42 Pitch | Po | 0.605–0.636 | 0.000 | 1.052–1.165 | 0 | 0.026397 |


| Self-level | Tune | Rise [s] | Max překmit [% z5°] | Settling2% [s] | Nedoložené / 6 |
|---|---|---:|---:|---:|---:|
| Roll | Před | 0.705–0.716 | 0.118 | 1.197–1.293 | 0 |
| Roll | Po | 0.613–0.625 | 0.112 | 1.108–1.160 | 0 |
| Pitch | Před | 0.677–0.710 | 0.368 | 1.097–1.228 | 0 |
| Pitch | Po | 0.596–0.642 | 0.014 | 1.015–1.195 | 0 |


Yaw gains zůstaly stejné a zlepšení yaw není prokázané. Ve2s okně nebylo
u2 ze6 finálních yaw kroků doložené2% ustálení; `null` se nenahrazuje
odhadem. Z těchto několika běhů nelze oddělit vliv výchozích stavů a vazeb
plantu od variability měření ani tvrdit obecnou ekvivalenci yaw transientů.
RATE v raw JSON označuje ACRO43 rate krok; legacy RATE41 má samostatné regrese.

Tři nominální běhy každé varianty obsahují obě znaménka RP, yaw±0,05rad/s,
kombinované vstupy, přímé RC i GCS přechody s nenulovými páčkami a návraty
do roviny. Není to statistický důkaz pro celou populaci letových podmínek.
Pásma v grafech jsou min/max tří běhů, nikoli interval spolehlivosti.

[ACRO graf](../tmp/utb-2h-20261018_093406/plots/rate-before-after.png) ·
[ANGLE graf](../tmp/utb-2h-20261018_093406/plots/angle-before-after.png) ·
[self-level](../tmp/utb-2h-20261018_093406/plots/level-before-after.png) ·
[PID a motory](../tmp/utb-2h-20261018_093406/plots/pid-motors-before-after.png).
SVG varianty, úplné raw vzorky a časové značky zůstávají ve stejném archivu.

## Model, saturace a motorové transienty

Použitý původní `bfxrev` model má3kg a diagonálu0,35m. Není to identifikovaný
fyzický5″ Skystars. V `SIM_Frame.cpp` se nominální Jxx/Jyy počítá jako
m×0,25×(d/2)², Jzz dvojnásobně; zde0,02296875/0,02296875/0,0459375kg·m².
Doplňkové modely mění jen všechny tři momenty setrvačnosti na90% a110%.
Hmotnost, motor/prop model, RC reference, gains a bezpečnostní limity zůstávají.
Jsou to omezené modelové perturbace, nikoli robustnost pro skutečný dron.

| Setrvačnost | Max rate překmit R/P [%] | ANGLE42 překmit R/P [%] | Max absolutní VZ [m/s] | DSF.Dp |
|---|---:|---:|---:|---:|
| −10 % | 3.996 / 4.495 | 0.000 / 0.000 | 0.273 | 0 |
| +10 % | 4.586 / 1.419 | 0.000 / 0.000 | 0.276 | 0 |

Perturbace obsahují celý oboustranný a kombinovaný protokol. Nezvýšený překmit je nominální porovnání při stejném plantu; perturbace nemají vlastní párovaný baseline.


| Tune | Přímé přechody | Max PWM krok v±100ms [µs] | Max krok povelu | Max collective skok | Max I před resetem |
|---|---:|---:|---:|---:|---:|
| Před | 42 | 27 | 0.057273 | 0.000000 | 0.006338 |
| Po | 42 | 20 | 0.043005 | 0.000000 | 0.006360 |

`before`: motorové alokační masky [0], blocked masky [0].

`after`: motorové alokační masky [0], blocked masky [0].

`inertia-minus10`: motorové alokační masky [0], blocked masky [0].

`inertia-plus10`: motorové alokační masky [0], blocked masky [0].
Clipping vnější rate reference je uložený zvlášť v ANGLE metrikách.

| Režim/osa | Tune | Max P | Max I | Max D | Max Out | Tail mean error [rad/s rate; rad angle] |
|---|---|---:|---:|---:|---:|---:|
| ACRO43 R | Před | 0.028876 | 0.000705 | 0.009423 | 0.029033 | 0.0005611 |
| ACRO43 R | Po | 0.014320 | 0.001405 | 0.003438 | 0.014720 | 0.0005522 |
| ACRO43 P | Před | 0.028611 | 0.000707 | 0.007117 | 0.028789 | 0.0001815 |
| ACRO43 P | Po | 0.021460 | 0.001308 | 0.009042 | 0.021971 | -0.0000080 |
| ACRO43 Y | Před | 0.015166 | 0.007395 | 0.000000 | 0.015447 | 0.0003028 |
| ACRO43 Y | Po | 0.015270 | 0.007627 | 0.000000 | 0.015565 | -0.0001297 |
| ANGLE42 R | Před | 0.050269 | 0.000839 | 0.016447 | 0.050844 | 0.0000304 |
| ANGLE42 R | Po | 0.027048 | 0.002028 | 0.006264 | 0.027470 | -0.0000710 |
| ANGLE42 P | Před | 0.050214 | 0.000867 | 0.012235 | 0.050421 | 0.0000141 |
| ANGLE42 P | Po | 0.040726 | 0.001937 | 0.016963 | 0.041292 | -0.0000420 |


Primární tuning protokol používá `UTB_C_AW=1` a `UTB_A_AW=1`; původní
default0 se nemění. Samostatné ANGLE regrese používají také AW0. V hlavních
měřeních nebylo blokování integrace ani motorová saturace; úplná pilotní
saturační obálka AW0/AW1 není ověřená.

Motor PWM jsou capture hodnoty původního AP_Motors v SITL, ne měření fyzického
DShot nebo momentu. Přechody musí mít prvníSeq1, nulové I/D/Out historie,
žádný mezilehlý AP controller v přímém přechodu a zachovanou throttle mapu.
Nulové alokační limit masky neprokazují funkci anti-windupu při agresivní
pilotní saturaci. Samostatná dosavadní mode41 saturace a jednotkové testy
ověřují své původní kontrakty; nejde o obecné schválení nového pilotního tune
pro saturující hardware let.

## Ověření a režie

- C++ **128/128**, Python **151/151**, bez přeskočených testů; pět read-only replay kontrol historických logů je zahrnuto.
- Nezávislá matematika **30946 vzorků /29 scénářů /0 neshod**; syntetický CSV byte-identický s2G.
- Nové gain guards a readback všech17 skutečných defaultů; frame/FSTRATE/policy/RC/arming40 a reset historie.
- Sedm feature build variant; po výběru defaultů nový přesný finální-source SITL active a H743 BENCH_ON build. Původní matice a opakované buildy mají oddělené source-hashes.
- Flake8, astyle změněných částí, metadata a Git whitespace kontrola. Nedotčený historický jednořádkový getter má pre-existing astyle rozdíl.
- Pilotní sady: disarmed, roll, pitch, yaw, transitions, transitions-pitch, transitions-yaw, transitions-combined, transitions-throttle, rc-loss, gcs-loss, angle-rc-loss, angle-gcs-loss, guards, limits, ground, remap-roll, ap-roll, ap-pitch, ap-yaw.
- Další úspěšné regrese: legacy-regressions, angle-disarmed, angle-roll5, angle-pitch5, aw-guards, saturation, shadow-final, manual-final.


SITL banky zvětšily `copter` symbol z63912 na64128B (+216B včetně paddingu)
a `AP_UTB::var_info` z1472 na2016B (+544B). H743 `copter`38128B a
var_info352B zůstaly stejné. Za běhu přibývají pevné porovnávací snapshots
15 rate parametrů; žádný heap ani synchronizační zámek. Host wall Ex/Log/Run
obsahují preempce a již existující diagnostiku; nejsou WCET ani H743 CPU cena.

| Tune | Host producer mean [µs] | Host producer max [µs] |
|---|---:|---:|
| Před | 2.663 | 724 |
| Po | 1.338 | 457 |

Tyto rozdíly host wall časů neprokazují snížení výpočetní ceny regulátoru;
srovnání neizoluje zatížení hostu ani vliv preempcí.


4× zrychlení SITL mělo rozsáhlé DataFlash drops a nebylo použito do finálního
porovnání. Neúplné sady zůstaly chybou; neřešilo se to jejich přeskočením.
Platné srovnání používá2×. Původní harness se souvislými rate pulsy bez
height resetu opustil výškovou obálku a byl nahrazen oddělenými GUIDED
přípravami. Původní hardcoded P analytický assert byl nahrazen stejnou
identitou P=Kp×error se skutečným časově platným BIN PARM; všech17 gains se
ověřuje samostatně. Krátké okno bez DSF má neznámou lokální logging health,
ne nulu; celkový drop count se bere z celého logu.

Nebyla instalována žádná další závislost. Grafy a metadata využívají
existující lokální runtime z2F/2G. COM5, upload, změny FC parametrů a fyzické
motorové testy neproběhly. ALT HOLD není implementován.

## Reprodukce a zbývající omezení

```sh
# Z kořene repository; nikoli sudo.
./waf configure --board sitl --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
./waf copter -j6
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_RATE_SITL --enable-UTB_BENCH
./waf copter -j6
```

Přesné varianty, source SHA, SHA-256 a velikosti ELF/APJ obsahují
`compileout-build-results.json`, `active-build-results.json`,
`verified-build-unit-commands.json` a `firmware-artifacts-final.json`.
Porovnání používá zachovaný vývojový ELF `build-initial/arducopter`, stejné
SHA pro všechny baseline/tuning lety a explicitních17 parametrů. Není to
ELF z konečného commitu. Finální-source `builds/verified-sitl-active/arducopter`
má vlastní SHA-bound gate, regrese a readback defaultů bez UTB_P_* override.
Zdrojový obsah dokládá manifest; build předchází druhému commitu, embedded
Git identita proto vychází z2G HEAD a pracovních změn.

Reprodukce: `utb_pilot_tuning.py --binary … --output NOVY_ADRESAR --name disarmed
--stage gate --tune … --speedup 2`, potom `--stage validation` s jedinečným
názvem. JSON model vyžaduje relativní cestu od izolovaného run adresáře,
např. `--model bfxrev:../../models/inertia-minus10.json`. Původní absolutní
cesta skončila před startem simulace; neúspěšný pokus zůstává archivovaný.


- UNIT/SITL/BUILD znamená pouze konkrétní doložené testy. Hardware aktivního
  UTB řízení zůstává neověřený a na H743 není implementovaný.
- Vyšší ACRO rates, větší ANGLE náklony, nové failsafe kombinace, vítr,
  celá bateriová/motorová obálka a jiné typy rámu nejsou tímto schválené.
- FS_GCS_ENABLE1, FS_THR_ENABLE1, FS_OPTIONS0 zůstávají podporovanou policy.
  UTB nulový fault-stop není obecná bezpečná letová obnova.
- Žádný vlastní EKF, nový AP_Motors/HAL/DShot nebo UTB výškový/XY regulátor.
