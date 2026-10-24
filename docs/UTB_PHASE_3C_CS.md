# FÁZE 3C – opakované DISARMED měření AP/UTB na H743

Stav k **20. 10. 2026**: tři skutečná HW opakování dokončena, kód a firmware
beze změny. Navazuje na [FÁZI 3B](UTB_PHASE_3B_CS.md), HEAD
`070c189cdfa8357307ee1aebe8f3677cd849d520`, větev `test/skystars-5inch`.

## Ověřený rozsah

COM5, board ID **1075**, USB serial **200024001251303433333336**, FW **4.7.1/3949c920**.
Stále bez LiPo a vrtulí, DISARMED. Použity existující `UB3_RUN` a
`utb_native_bench_analysis.py` (protokol Ver 2). Žádný ARM, motorové příkazy,
nový build, upload, mazání logů nebo PID tuning.

| Běh | Kompletní/validní sady | Max. rozdíl / nejvyšší RMSE | PM.NLon přes celý RUN | Drops |
| --- | --- | --- | --- | --- |
| 1 | 2000/2000 | 0 / 0 | 0 | 0 |
| 2 | 2000/2000 | 0 / 0 | 0 | 0 |
| 3 | 2000/2000 | 0 / 0 | 0 | 0 |

Celkem **6000 validních sad**, žádné chybějící, částečné, NaN/Inf nebo capture/health
invalid vzorky. V každém běhu 500 sad na každý ze čtyř profilů: PID, Rate/shaping,
Attitude+Rate s fixture gyro a skutečným AHRS, Attitude+Rate se skutečným gyro/AHRS.
Shodné jsou P/I/D/FF/D_FF, výsledné zásahy v RAM, rates i quaternion komponenty.
Výsledek platí pro existující fixture/gains a SMAX=0, nikoliv všechny konfigurace.

## Naměřená režie a časování

| Běh | AP mean / max [µs] | UTB mean / max [µs] | Export mean / max [µs] | Load logged RUN0 → RUN1 [%] |
| --- | --- | --- | --- | --- |
| 1 | 37.124 / 157 | 33.990 / 155 | 100.160 / 440 | 18.411 → 19.202 |
| 2 | 34.675 / 169 | 34.627 / 168 | 100.894 / 210 | 18.270 → 19.145 |
| 3 | 34.602 / 158 | 33.449 / 159 | 103.188 / 214 | 18.216 → 19.037 |

Pooled časy (mean / median / P95 / P99 / max):

- AP: **35,467 / 39 / 91 / 139 / 169 µs**.
- UTB: **34,022 / 37 / 82 / 139 / 168 µs**.
- Logger export: **101,414 / 84 / 183 / 198 / 440 µs**.

Jsou to preemptible wall časy H743, ne izolované CPU cycles. Setup/reset/capture
nejsou v math stopkách; export je měřen zvlášť, poslední export každého běhu
nemá následníka a není pozorovaný. AP běží v páru první; malý rozdíl není důkaz
rychlostní výhody. Úplné percentily pro každý profil jsou v CSV.

50Hz task má frekvence 49,999925 / 50,000433 / 49,999972 Hz a průměrný absolutní
jitter 162,682 / 163,734 / 183,934 µs. Main loop je jiná veličina: průměr
přibližně 2500 µs, delta mean absolute jitter během RUN 2,531 / 2,631 / 2,556 µs.
PM intervaly pokrývají celé RUN; NLon=0 a MaxT nejvýše 2552 / 2577 / 2553 µs.
Žádný InternalError/ERR nebo porucha EKF/loggeru během testů.

## UBST8 a opakovaná časová anomálie

UBST8 měří `time_taken > _task_time_allowed` v
[AP_Scheduler.cpp](../libraries/AP_Scheduler/AP_Scheduler.cpp), řádky 242–285.
Pro běžnou úlohu používá její max_time_micros, pro FAST_TASK nominální periodu.
Wall čas může zahrnovat preempci a dispatch; čítač neukládá ID viníka.
PM.NLon naopak sleduje mezi-sample interval nad 120 % nominálu, tedy 3000 µs
při 400 Hz ([PerfInfo.cpp](../libraries/AP_Scheduler/PerfInfo.cpp), řádky 119–135 a 227–230).
UBST9 počítá už malé překročení 2500 µs. Nulové NLon není důkaz striktního
hard deadline ≤2500 µs pro každý cyklus.

UBST8 RUN0 → RUN1 (událostí/s): **11,088 → 11,676; 13,717 → 13,006;
13,815 → 11,984**. Překračování rozpočtů existuje i bez native výpočtu.
Z toho nelze určit konkrétní běžnou úlohu ani všechny události připsat UTB.
Součet APus[N]+UTus[N]+LogUs[N+1] dokládá nejméně **1 / 0 / 0** native
překročení jeho 300µs budgetu; je to pouze dolní počet, protože chybí
setup/capture/dispatch a poslední export. Nejde o počet ztracených main cyklů.

Před prvním RUN se anomálie zopakovala: **PM.NLon=1, MaxT=6480 µs**.
Její souhrnné okno opět překrývá konec RUN0/no-log a první zapnutí loggeru.
Před RUN2/3 podobná událost není. Původních 5503 µs z 3B nebylo při MCU bootu
(FC už běžela přibližně 18 minut), nových 6480 µs rovněž ne (přibližně 15 minut).
Obě události předcházejí native výpočtu i jeho registraci U3B formátů.
**Přesný čas a viník zůstávají neobjasněné.**

Zdrojově je možná synchronní startovací cesta loggeru v hlavním vlákně:
`AP_Logger_Backend::WritePrioritisedBlock` → `AP_Logger_Block::start_new_log`
→ čtení flash a JEDEC `WaitReady`. To je doložená možnost, ne důkaz příčiny
5503/6480 µs. První velký jitter je již v prvním histogramu po LOG_DISARMED=1;
pak H11 nepřibývá. Souhrny nerozlišují logger start, uložení parametru,
MAVLink, ostatní úlohy nebo mutexovou interakci. Nález byl rozpoznán offline
po dokončení tří pokusů; další pokus po nálezu nebyl spuštěný.

## Data, oddělení záznamů a konečný stav

[Úplný report](../tmp/utb-3c-20261020_151107/REPORT_CS.md),
[souhrnné CSV](../tmp/utb-3c-20261020_151107/summary/runs.csv),
[percentily](../tmp/utb-3c-20261020_151107/summary/timings.csv),
[profily](../tmp/utb-3c-20261020_151107/summary/profiles.csv),
[graf časování](../tmp/utb-3c-20261020_151107/summary/math-and-export.svg),
[scheduler](../tmp/utb-3c-20261020_151107/summary/scheduler.svg),
[CPU](../tmp/utb-3c-20261020_151107/summary/cpu-load.svg).

Původní BIN má **7 924 746 B**, SHA-256
`b540dd6883670211251955017c9a6b70d751641259b33bc290000b834ec908be`.
Oddělení offline bylo výslovně schválené. `derived/run1..3` obsahují každý
jedinou sekvenci, vlastní baseline/after a původní metadata. Zprávy byly
kopírovány jako nezměněné bajty, bez reserializace numerických hodnot;
`provenance.json` uvádí časová okna, SHA a mapu původních offsetů. Kompletní
fyzický zdroj zachován. Nejde o tři soubory vytvořené přímo ve FC.

Po prvním běhu hostitelský skript zastavil na příliš úzkém předpokladu jediného
fyzického souboru: FC po startu před 3C vytvořila nový soubor č. 2 vedle starého
3B logu. Byla upravena jen tato kontrola skriptu v tmp/; první běh se neopakoval,
navázaly pouze běhy 2 a 3. Nešlo o poruchu nebo restart FC během testu.

Závěrečná 36s kontrola přečetla opět **1212/1212 parametrů**. Všechny se shodují
s počátkem 3C kromě přirozeného přírůstku STAT_RUNTIME; BOOTCNT beze změny.
UB3_RUN=0, LOG_DISARMED=0, SHADOW=0, FSTRATE=0, backend=4,
DISARMED/STABILIZE, EKF167. Port uzavřený. Staré i nové logy zachované.
Zdrojový kód a firmware beze změny; žádné instalace, nový commit ani push.

## Rozhodnutí před dalším krokem

- **Numerická opakovatelnost ANO** v tomto fixture a třech opakováních.
- **Steady 50Hz diagnostika prošla**, ale první otevření loggeru zůstává RISK;
  nejde o dlouhodobou nebo cold-start kvalifikaci.
- Lze připravovat **návrh** samostatného pasivního 400Hz SHADOW a jeho měřicí plán.
  Realizaci a let zatím neschvalovat: objasnit startovací špičku, změřit plný
  capture/compute/export rozpočet a propustnost. Současný DISARMED fixture nelze
  jen zrychlit a přesunout do letu. Už samotných 510 B/sadu při 400 Hz znamená
  204000 B/s navíc k AP logování; toto není zde ověřené.
- Další šetření může nejprve využít původní per-task statistiku přes
  SCHED_OPTIONS/@SYS/tasks.txt v samostatné DISARMED session. Její režie a
  alokace musí mít kontrolní baseline. Firmware ani matematiku neopravovat
  naslepo. Původní AP musí nadále držet výhradní motorovou autoritu.

**Žádný výsledek 3C nekvalifikuje aktivní 400Hz řízení nebo letovou bezpečnost.**
