# FÁZE 3G-3F-HW-2B – Flight + Timing + Execution na H743

**PASS pro jeden schválený USB-only DISARMED experiment. Etapa plánovaných DISARMED časových benchmarků uzavřena. Nejde o povolení letu nebo kvalifikaci palubního loggeru.**

## Identita, integrita a schválení

Větev `test/skystars-5inch`, HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`. Původních 7689 sledovaných/nesledovaných souborů a 104 artefaktů HW-2A ověřeno SHA-256. Původních 14 source souborů a APJ shodné. Žádná změna zdrojů před HW-2B.

Stávající APJ `SkystarsH7HD-bdshot_UTB3G3F_EXEC_ON_dae4f020db98.apj`, 1158367 B, SHA-256 `dae4f020db98a509b68c71a5b8702dfc7019ed762f69b7d46c8cbf3ae5e9d497`. Žádný build/upload. COM5 zjištěná podle VID:PID1209:5741 a serialu200024001251303433333336; board1075/MAVLink1/1, Copter4.7.1/11a518e5. APJ hash hostu a upload evidence nejsou kryptografický readback MCU.

Samostatně schválený fyzický reconnect, read-only preflight a jediný společný experiment; odpovědi `approval-physical-reconnect.json`, `approval-a.json`, `approval-b.json`. Nový boot116 oproti A115, bez dalšího restartu. Potvrzeno pouze USB, odpojená LiPo, demontované vrtule, FC v klidu, ostatní sériové aplikace zavřené. Trvale DISARMED STABILIZE. Žádný ARM, motorový příkaz, DataFlash zápis/start/erase, upload nebo automatické opakování.

Preflight: 60,480 s, 1408 RX paketů, parser errors0. Úplná záloha1220 parametrů/0–1219; od HW-2A pouze očekávané startup gyro kalibrace/teploty, BARO ground pressure, COMPASS_DEC a STAT_BOOTCNT/RUNTIME. Původní konfigurace zachovaná. LOG_BACKEND_TYPE4/LOG_DISARMED0, SCHED_LOOP_RATE400/FSTRATE_ENABLE0/SCHED_OPTIONS0, SMAX/NTF/NEF0, bez rate threadu. Flight/Timing/Execution pending, všechny RUN0.

## Průběh

UTT_COUNT již4000, nezapisoval se. UCS_F_COUNT24000→4000 a DECIM200→40. Dočasná telemetrie SYS_STATUS/SYSTEM_TIME/EKF 2/1/1 Hz s GET readback, původní hodnoty−1/−1/−1. Po3s ustálení baseline15,008098 s, bez TX.

Jediný UTT_RUN1 v 2026-10-25T20:05:53.206852+01:00, poté jediný UCS_F_RUN1 v 2026-10-25T20:05:53.207862+01:00. Odstup host perf_counter_ns **1097.500 µs**, nikoliv atomický start nebo přesný čas přijetí MCU. Host QPC rozlišení100ns, MCU HAL tick1µs. Žádné další TX během přibližně20s aktivního/následného pasivního okna. Obě RUN poté automaticky0. Sekvenční export Timing/Execution/Flight a8 stránek až po freeze, memory/threads před a po exportu. Celé spojení 85.454936 s,4135 RX/229242B, parser errors0; rawTX6137B. Port zavřený, žádný další hardware test.

## Validita a numerika

Timing/Execution reason1,4000 cyklů a4000 kompletních povinných Execution sad;3999 Main/Rate/Att period,4000 Rate→Att delay. Missing/duplicate/invalid/gaps/unsupported/phase_changes/missing_finish vše0. Execution session_us322863269 shodné v header/end. Sedm povinných metrik N4000, consume N3999, žádný neúplný pár.

Flight reason1,4000 attempts,3999 paired Rate,1 warmup,8000 valid Attitude operací (4000 STABILIZE +4000 reset),1 Att a1 PID synchronizace,4000 Att resetů,11994 externích I událostí,0 filter resetů. Všech84 metrik **max_abs=RMSE=0** při skutečných tolerancích comparatoru. Motor_checks12000, motor_changes0; upstream/downstream mismatch, PID/Att mismatch/desync/invalid/unsupported0. Bounded recorder100 selected/3900 decimated/0 overwritten, všech8 stránek úplných. CSV všech84 metrik s počty v `analysis/metrics-with-counts.csv`.

Tento DISARMED běh měl limited3999, independent_i0, att_shaping0; neověřuje nenulovou integraci/FF, dynamický shaping nebo uzavřenou UTB smyčku za letu. Tyto scénáře je nutné odlišit od existujících SITL důkazů.

## Překryv a source identity

Flight MCU start322865769/stop332864129 µs, délka9,998360 s. Timing první Main odvozený interval322863221–322863287, poslední332860712–332860778 µs; nejsou vydávané za exportované přesné hranice. Prokázaný společný čas9,994943–9,995009 s. Výpočty z anchorů/min/max/sum jsou v `analysis/summary.json`.

Execution3999 consume zahrnuje první Flight warmup a následné párované kroky uvnitř Timing okna; Flight má vlastní pozdější konec. Jeho první validní PID čas322868272 a poslední332863269 µs. Proto shodná čísla3999 neznamenají stejné množiny platných Rate kroků. Nelze lepit UTT lokální cycle ID a Flight globální Loop ID. Každý zachovaný paired Flight řádek má Loop−Source=1 a SourceUS před TimeUS. Execution bounded kontext cyklů100–116 kolem maxM3 v108 je samostatná sada; Timing má vlastní anchor. Úplná400Hz raw časová řada není exportovaná, P95/P99 nelze doložit.

## M1–M5 a další Execution metriky

Všechny časy µs. A=boot115/native, B=boot116/native+Flight, stejný firmware a telemetrie.

| Metrika | A min/mean/max | B min/mean/max | Δmean B−A | Δpozorované max |
|---|---|---|---|---|
| main_to_motor_begin | 28 / 31.489500 / 62 | 61 / 122.688250 / 135 | 91.198750 | 73 |
| motor_call | 29 / 31.993500 / 41 | 29 / 31.499500 / 41 | -0.494000 | 0 |
| main_to_motor_return | 57 / 63.483000 / 96 | 91 / 154.187750 / 168 | 90.704750 | 72 |
| main_to_scheduler_boundary | 315 / 491.404750 / 1383 | 514 / 738.341000 / 1637 | 246.936250 | 254 |
| rate_to_motor_begin | 13 / 15.368750 / 23 | 14 / 107.117750 / 120 | 91.749000 | 97 |
| main_to_mode_finish | 266 / 408.748250 / 1311 | 462 / 651.911250 / 1357 | 243.163000 | 46 |
| main_to_fast_boundary | 280 / 423.963750 / 1324 | 475 / 671.404000 / 1371 | 247.440250 | 47 |

UTB Rate consume N3999:6 /82,038760 /95 µs, sum328073. V A absent, nikoliv0µs měření. M1 mean+91,198750 a M5+91,749000 µs odpovídají vložení pasivního consume před motor begin plus capture/audit režii. Samotný M2 mean−0,494000 µs, max41 v obou bootech: nejde o prokázané zrychlení motorového backendu. M3 mean+90,704750 µs. Všechny Execution počty>2500/≥5000 jsou0 v A i B; změna počtů0. Součet M3=M1+M2 se ověřuje po cyklech/agregaci, nesčítají se maxima nebo vnořené spany.

M2 zahrnuje softwarové volání i případnou preempci, ne elektrické dokončení DShot. M4 končí na definované hranici scheduleru, nezahrnuje celý AP_Vehicle/HAL ani finalizaci/epilog. Rozdíly dvou bootů nejsou izolovaná deterministická režie nebo hard WCET.

## Timing a CPU

| Metrika | N | Min | Mean | Max | Mean abs.error | >2500 | ≥5000 |
|---|---|---|---|---|---|---|---|
| Main start-to-start | 3999 | 2483 | 2499.997749 | 2516 | 1.227057 | 839 | 0 |
| Rate entry period | 3999 | 2476 | 2499.989747 | 2519 | 1.926232 | 1395 | 0 |
| Attitude primary entry period | 3999 | 1737 | 2499.950488 | 3276 | 211.847962 | 2068 | 0 |
| Rate to Attitude delay | 4000 | 353 | 507.124750 | 1203 | 507.124750 | 0 | 0 |
| Delay change | 3999 | -765 | -0.039260 | 772 | 211.340085 | 0 | 0 |

Main/Rate maximum2516/2519 µs, bez>3000; peak-to-peak33/43 µs proti A48/56. Att max3276 µs, žádný≥5000; přesný počet Att>3000 není exportovaný. Att entry jitter nelze zaměnit za Main/Rate deadline miss. Rate→Att mean507,124750 proti353,816500 µs, posun+153,308250 µs. Periodové identity přesné.

| CPU okno | N | Min % | Mean % | Max % | Host elapsed s |
|---|---|---|---|---|---|
| baseline | 30 | 18.2 | 18.430000 | 18.8 | 26.322592–41.330690 |
| timing_interior_estimate | 18 | 28.6 | 28.800000 | 29.1 | 41.847798–50.844192 |
| after | 18 | 18.2 | 18.377778 | 18.6 | 52.436328–61.436328 |

Active A19,111765→B28,800000 %, rozdíl+9,688235 procentního bodu; baseline A18,516667/B18,430000. Uvnitř B active−baseline+10,37 bodu. Active interior používá host trigger a MCU délky s0,5s okraji, SYS_STATUS scheduler load je filtrovaný a přenos zpožděný. Nevydává se za čistou cenu algoritmu.

Timing brackety sum36754/calls24000/max79 µs,9,188500µs/Main. Execution sum19224/calls23998/max61,4,806000µs/Main; calls=4×4000+2×3999. Flight pre33846, post_without_chain_att26828, att_consume464180/max205, rate_consume249942/max76, record2114/max40 µs; separátní math/sync/compare v `analysis/summary.json`. Math PID33879 a Att149352 µs jsou vnořené úseky. Nemají se přičítat k nadřazenému consume; zachované původní názvy bracketů, žádný součet maxim jako WCET. PM.NLon/MaxT ani UBST/per-task průběh v tomto RAM-only běhu nezískané, null není0.

## Paměť

| Vlákno | Přiděleno B | Před free | Po compute | Po exportu |
|---|---|---|---|---|
| ArduCopter | 7168 | 4312 | 4312 | 4312 |
| ISR | 1536 | 1240 | 1240 | 1240 |
| FTP | 2992 | 1624 | 1600 | 1168 |
| utb_log | 3504 | 2456 | 2456 | 2456 |
| rcout | 944 | 504 | 504 | 504 |
| log_io | 2016 | 1512 | 1512 | 1512 |
| timer | 1968 | 1552 | 1552 | 1552 |
| io | 2480 | 1544 | 1544 | 1544 |

| Region | Před FREE/LRG B | Po compute | Po exportu |
|---|---|---|---|
| 0x30000000 | 4712/4192 | 3688/3640 | 3688/3640 |
| 0x20000000 | 37936/37936 | 47944/47944 | 47944/47944 |
| 0x24000000 | 524280/524280 | 524280/524280 | 524280/524280 |
| 0x00000400 | 64504/64504 | 64504/64504 | 64504/64504 |
| 0x30040000 | 32760/32760 | 32760/32760 | 32760/32760 |
| 0x38000000 | 54536/53816 | 54536/53816 | 54536/53816 |

Main4312 a ISR1240 B volného watermarku stabilní; proti A4344/1256 je to jiný boot, nikoliv izolovaný stack účet. FTP1624→1600→1168 B při úplném exportu; bez selhání a po exportu pozorovaná rezerva1168/2992 B, nejde o bezpečnostní práh. Regiony po výpočtu a exportu stejné, první3688/3640 B. Přípravný snapshot4712/4192 a druhýregion37936/37936 se po výpočtu vrátily na3688/3640 a47944/47944; samotné snapshoty neurčují příčinu dočasných alokací nebo přechodné minimum. Žádné pozorované alokační/stack selhání, žádný trvalý pokles po exportu.

## Izolace, logger a cleanup

Původní source/symbol audit3G-3F zachovaný, APJ/source shodné, H743 active Core/Output/Acro/Angle compile-out. Flight používá inertní hodnotové kopie, motorových kontrol12000 se změnou0. Nejde o elektrické měření výstupů. Native AP jediný vlastník motorů. LOG_DISARMED0 a BLOCK4, drops/InternalError0, žádné nové DataFlash logování. Čtyři historické logy podle úplného inventáře zachované:2516746/7924746/718496/725746B, celkem11885734B; neprováděl se nový byte-for-byte download.

UCS_F_COUNT obnoven24000, DECIM200, intervaly−1/−1/−1 s readbackem. Všechny RUN0, boot116, DISARMED STABILIZE, health v pořádku, úplných1220 parametrů, změnil se pouzeSTAT_RUNTIME. COM uzavřený. Původní rozpracované soubory/HEAD/index zachované. Nový je pouze tento report a ignorované HW artefakty. Žádný commit/push/reset/clean/stash/rebase.

## Grafy a evidence

CSV/JSON/raw RX+TX/approval/parameter backup/exporty v `tmp/utb-3g3f-hw2b-20261025_prepare-01`. Kontrolní SHA manifest `hw-evidence-sha256.json` uzavírá HW část před offline částmi B/C; žádná původní evidence se nemění.

![A/B](../tmp/utb-3g3f-hw2b-20261025_prepare-01/analysis/plots/ab-comparison.png)

![Execution](../tmp/utb-3g3f-hw2b-20261025_prepare-01/analysis/plots/execution-aggregates.png)

![CPU](../tmp/utb-3g3f-hw2b-20261025_prepare-01/analysis/plots/cpu-timeline.png)

Další grafy v analysis/plots:Timing/delay, bounded Execution/Timing kontext, hook brackety, stack,FREE/LRG. Zobrazené body jsou skutečná data.

## Rozhodnutí

**Uzavřít plánovanou DISARMED časovou kvalifikaci.** Žádný konkrétní závažný timing/numerický/motorový blocker v tomto měření nezjištěn. Neplánovat automaticky další stejný benchmark. Pokračovat offline auditem a přímým předletovým plánem. První native let nevyžaduje hard WCET nebo měření elektrického DShot; vyžaduje skutečné napájení/RC/směry motorů a stabilizace/failsafe a letově použitelný firmware/logger. Pasivní persistentní UTB logger zůstává samostatná dosud HW nekvalifikovaná funkce. V této fázi nebyl povolen ani proveden let, LiPo bench nebo upload.
