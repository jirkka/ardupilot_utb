# FÁZE 3F-3 – Kontinuální 400Hz HW ověření integrovaného řetězce

Datum: 24. 10. 2026. Skutečná Skystars STM32H743 Rev.V. Pouze USB, bez LiPo a všech vrtulí, trvale DISARMED. Jeden schválený experiment UCS_RUN=1 / UCS_COUNT=4000, bez opakování, buildu nebo uploadu.

## 1. Závěr a rozsah

Integrovaný UTB_Attitude(N) → UTB_Rate/PID(N+1) dokončil **4000 main pokusů**, **1 Rate warmup**, **3999/3999 spárovaných Rate kroků**, **4000 primary Attitude + 4000 yaw resetů**. Všech **8000 Attitude operací a 3999 Rate kroků validních**. Missing/duplicate/invalid/DESYNC/UNSUPPORTED/mismatch a motor_changes **0**, stop reason **1 – dokončení počtu**.

Všech **84 exportovaných max. absolutních chyb a RMSE je 0**. Upstream/Rate strict compare dokládá přesnou numerickou rovnost porovnávaných projekcí. U Attitude jde o nulový textový export a žádné překročení implementované tolerance; bitovou shodu pod rozlišením exportu neprokazuje.

Jedna počáteční synchronizace Attitude a jedna Rate; žádný následný úplný import historie. Přehráno **4000 skutečných yaw resetů a 11994 externích I_set příznaků**. To potvrzuje vlastní historii s replay skutečných resetů, nikoliv volnou nenulovou I akumulaci nebo yaw historii bez resetů.

Naměřené nepřekrývající se přidané brackety **191,7865 µs/main**, proti krátkému 3F-2 **−0,7295 µs/main (−0,379 %)**. CPU **18,3633 → 28,1722 → 18,3938 %**, Rate capture perioda **2460–2540 µs**, průměr **2500 µs**, průměrný absolutní jitter **1,73487 µs**. Main watermark **4344/7168 B volných před i po**. Není pozorovaný restart, InternalError, porucha senzorů nebo auditovaná změna motorových hodnot.

**Omezenou klidovou DISARMED HW validaci propojeného řetězce lze uzavřít pro tento podporovaný profil.** Neuzavírá se úplná přidaná režie/WCET, Main perioda, dynamické HW/letové chování nebo aktivní UTB řízení. Další experiment se automaticky nespustil.

## 2. Firmware, integrita a fyzický nový start

Repo `/home/jirka/ardupilot_utb`, větev `test/skystars-5inch`, HEAD **89e81d94763f1610b52b6e93510de3df71969af0**. Commit reportů **90b24237c94d935fbc7907488155d07d87fc3934**, report 3F-2 a původní kapitoly/historie.txt zachované. Všech 18 implementačních souborů odpovídá SHA-256 manifestu 3F-1.

Použit již nahraný firmware z 3F-2:

`tmp/utb-3f1-20261024_155715/H743_ON/SkystarsH7HD-bdshot/bin/arducopter.apj`

- SHA-256 **e4e763e121d0ae5390a9f34d456ca2970bf82ffb1545bd092d1a790d1f94005d**.
- Velikost **1 148 707 B**, board_id **1075**, APJ git_identity **90b24237**.
- COM5, VID:PID **1209:5741**, USB serial **200024001251303433333336**, MAVLink 1/1.
- Běžící firmware Copter 4.7.1 / **90b24237**; sestaven před implementačním commitem, zdroje identifikuje původní manifest.
- MCU **STM32H743 Rev.V** je identifikace z předchozího ověřeného bootloader uploadu. V této fázi se nevstupovalo do bootloaderu a znovu se nečetl chip revision registr.

Obsluha potvrdila fyzické odpojení/připojení pouze USB, bez LiPo/vrtulí, FC v klidu a zavřený Mission Planner. **STAT_BOOTCNT=106**, konečný 3F-2 měl 105. Nový boot je potvrzený fyzickým krokem, čítačem a čerstvou session. Žádný softwarový reboot. Lokální SHA a živá identita/verze navazují na verifikovaný upload 3F-2; **nebyl proveden nový kryptografický readback MCU flash**.

## 3. Preflight a schválení

Read-only pozorování **40 s**, poté úplná záloha přibližně **30 s**, celkem **70,734 s**, všech **1215/1215 parametrů**. Identity, DISARMED STABILIZE, gyro/accel/AHRS/logger health a EKF attitude prošly, bez pozorovaného InternalError.

Ověřeno:

- SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0, žádný rate thread.
- UCS_RUN=UAS_RUN=UNS_RUN=0, UCS_COUNT=4000.
- LOG_DISARMED=0, SCHED_OPTIONS=0, UTB_SHADOW=0.
- ATC_RAT_RLL/PIT/YAW_SMAX=0, NTF=0, NEF=0. Zisky, FF, filtry, RC/failsafe/motorové parametry beze změny.
- Timing je compile-OFF, UTT parametr není přítomný ani se neaktivoval.
- Frozen export před měřením byl **UTBChainV1,pending**. Při vstupu runneru se ověřil znovu.

Úplná konfigurace proti konečné 3F-2 má stejný seznam parametrů. Rozdíly jsou pouze STAT_RUNTIME, STAT_BOOTCNT, gyro startup offsets/kalibrační teploty a BARO1_GND_PRESS. Přesné původní/aktuální hodnoty jsou v `parameter-comparison.json`. Automatické startup hodnoty se nevracely na stará čísla; žádné letové parametry se nepřepisovaly.

Původní čtyři logy mají stejná ID, časy a velikosti, celkem **11 885 734 B**. RAM-only test nepotřebuje nové DataFlash záznamy. Logger se nezapnul, BIN se nestahoval a žádný log se nemazal. Inventory je kontrola metadat, nikoliv bitový audit celého obsahu flash.

Main watermark při preflightu **4344/7168 B**, o **8 B méně než v jiném bootu 3F-2**, již před experimentem. Rozdíl byl předložen před schválením; v novém měřicím okně se dále nezměnil. Není doložená konkrétní startup funkce způsobující těchto 8 B.

Po preflightu přišlo samostatné výslovné schválení právě jednoho UCS_RUN=1 / COUNT=4000 experimentu a opětovné potvrzení fyzických podmínek. Záznamy `physical-start.json`, `approval.json` a `experiment-attempt.json`; poslední soubor brání opakovanému triggeru stejného runneru.

## 4. Přesný protokol jediného běhu

Všechny nové artefakty: **tmp/utb-3f3-hw-20261024_192647/**.

Původní intervaly SYS_STATUS/SYSTEM_TIME/EKF_STATUS_REPORT byly **−1**. Dočasně nastavené **2 Hz / 1 Hz / 1 Hz**, stejně pro baseline/ON/po. Po 3s ustálení následovala **15,016s baseline**.

| Host elapsed [s] | Skutečná operace |
|---:|---|
| 9,359–24,375 | Baseline UCS=0 |
| před triggerem | Read-only potvrzení UCS_COUNT=4000; nebylo potřeba měnit COUNT |
| 25,453 | Jediný PARAM_SET UCS_RUN=1 |
| 25,453–36,531 | Tiché přijímací okno, pouze přednastavená telemetrie |
| od 36,531 | Potvrzení automatického UCS_RUN=0, UAS_RUN=0 a health |
| následujících přibližně 8 s | Telemetrie po testu před exportem |
| poté | FTP frozen snapshotu a numerická kontrola |
| 44,797 / 44,859 | UCS_RUN=0 / potvrzení původního UCS_COUNT=4000 při cleanup |
| do 52,937 | Health, stack po, obnovení intervalů na −1 a uzavření COM5 |

Audit odchozích packetů v measurement stage obsahuje **právě jeden PARAM_SET UCS_RUN=1**. Žádný další PARAM_SET, FTP, polling, log operace nebo diagnostický příkaz v aktivním okně. Konečná read-only health/param/log kontrola následovala až po exportu/cleanup a port znovu uzavřela. Nebyl druhý experiment, ARM/motorový příkaz nebo další restart.

MAVFTP hlášení `expected 100000, got ...` odpovídají virtuálnímu @SYS souboru s kratším skutečným EOF. Přenosy skončily error_code=0 a obsah prošel kontrolou. Není to hlášení ztracených shadow sad.

## 5. Úplnost, párování a historie

| Export | Výsledek |
|---|---:|
| Attempts / warmup / paired | **4000 / 1 / 3999** |
| Primary STABILIZE seen/valid | **4000 / 4000** |
| RESET_YAW seen/valid | **4000 / 4000** |
| Ostatní Attitude command seen/valid | **0 / 0** |
| Attitude seen/valid | **8000 / 8000** |
| Rate seen/valid | **3999 / 3999** |
| Attitude synchronizations / mode changes / config changes | **1 / 0 / 0** |
| Rate synchronizations / config changes | **1 / 0** |
| Externí I_set / filter resets | **11994 / 0** |
| Rate first_seq / last_seq / period_count | **1 / 3999 / 3998** |
| Poslední source Attitude loop / consumed Rate loop | **66896 / 66897** |
| Missing / duplicate / invalid / unsupported | **0 / 0 / 0 / 0** |
| Attitude a PID invalid/DESYNC/UNSUPPORTED/mismatch | **vše 0** |
| Upstream / downstream mismatch | **0 / 0** |
| Motor checks / motor changes | **12000 / 0** |
| Stop reason | **1, automaticky dokončený počet** |

### Skutečný target a N→N+1

Zdroj `AP_UTB_ChainShadow.cpp:29` po vlastním Attitude výpočtu ukládá `_attitude.history()`, ne native `frame.after`. Rate větev (`:47`) před každým započítaným krokem kontroluje `_loop == _source + 1U` s unsigned přetečením. Její vstup je **vlastní `_target.body + sysid`** a vlastní `_target.feedforward`. Native target je pouze oracle; fallback se nepoužívá. Gyro/dt/scaling/limit flags a sysid jsou hodnotové kopie skutečného native Rate kontextu, zachycené před původním výpočtem; native PID post stav slouží k porovnání, ne k obnově UTB historie.

Všech 3999 validních kroků prošlo tímto guardem. Poslední exportovaný raw pár **66896→66897** splňuje vztah přímo. Firmware neposkytuje všech 3999 jednotlivých párů: úplnost dokládá runtime guard a agregované čítače bez missing/duplicate, nikoliv nový nezávislý offline přepočet každého raw ID. První Rate má warmup a nepočítá se jako validní porovnání; poslední nově vytvořený Attitude target zůstává mimo konec párovaného okna.

### Co je vlastní a co resetované

- Attitude plný import **1**, dalších **7999 operací bez dalšího plného importu**. Žádný mode/config epoch reset.
- Rate plný import **1**, dalších **3998 kroků s vlastní navazující filtrovou historií**. Změna nepředané historie by způsobila DESYNC, ne skryté překopírování native post stavu.
- Skutečných **4000 RESET_YAW** se přehrálo do vlastní Attitude instance. Jde o změnu vlastní historie podle skutečné AP resetové události.
- **11994 = 3 × (3999 − 1)** I_set replay příznaků. První Rate import seeduje všechny tři osy; v dalších krocích je I nastavované skutečnou externí AP událostí DISARMED. Counter nerozlišuje všechna jednotlivá AP C++ volání, pokud se sloučí do pending flagu.
- Nejde o 3999 úplných PID importů. **I peak=0, independent_i=0**, takže volná nenulová integrace bez resetů není nový HW PASS. Totéž platí pro yaw historii bez pravidelného resetu.

Source opora: `AP_UTB_AttitudeShadow.cpp:159`–170 (import pouze při synchronizaci, jinak kontrola návaznosti), `AP_UTB_NativeShadow.cpp:172`–197 (jediný seed a replay skutečných I/filter reset flags). Matematika se neměnila.

## 6. Numerické výsledky a jejich přesnost

| Skupina | Metrik | Porovnání na metriku | Max/RMSE export | Mismatch |
|---|---:|---:|---|---:|
| Attitude | **26** | **8000** | **0 / 0** | **0** |
| Upstream body target + FF | **4** | **3999** | **0 / 0** | **0** |
| Rate/PID, 3 osy × 18 | **54** | **3999** | **0 / 0** | **0** |

Všech **84 konkrétních řádků** s názvy, osou, počtem porovnání, max. chybou, RMSE a politikou porovnání je v `analysis/metrics.csv`. Exportuje se group mismatch, nikoliv samostatný čítač pro každou metriku; nulová překročení jednotlivých položek jsou odvozená z nulového výsledku příslušné skupiny.

**Attitude:** tolerance `1e−6 × max(1, abs(AP hodnota))`. att_mismatch=0 a nulová desetinná maxima/squares nejsou důkaz bitové shody pod rozlišením exportu. **Upstream a Rate:** strict compare testuje nenulový numerický FP rozdíl, takže nulový mismatch podporuje přesnou numerickou rovnost porovnávaných projekcí i mimo textové zaokrouhlení. Nepotvrzuje bitovou shodu neporovnávaných struktur nebo znaménka nuly.

att_invalid=pid_invalid=0, žádné zachycené selhání finite/unit historie. Samostatný raw seznam výsledků každé quaternionové validace není dostupný; vlastní target quaternion v posledním kontextu má z textových hodnot normu přibližně **1,0000000504**. Není to rekonstrukce všech vzorků.

Příklady skutečného posledního Rate kontextu:

| Veličina | R | P | Y |
|---|---:|---:|---:|
| Body target AP i UTB [rad/s] | −0,07540843 | +0,06530264 | −0,000547151 |
| Gyro [rad/s] | +0,000188978 | −0,000074366 | −0,001357030 |
| P AP i UTB | −0,006123060 | +0,005489584 | −0,000124989 |
| D AP i UTB | −0,000015338 | −0,000247660 | 0 |
| Output AP i UTB | −0,006138398 | +0,005241924 | −0,000124989 |

dt=**0,002500 s**, limit flags=1 na každé ose. Nejde o porovnání samých nulových výstupů, ale ani o řízený dynamický pilotní impuls. Příkazy jsou klidové, gyro peak **0,007337352 rad/s**, command/I/FF/D_FF peak=0, shaping=0. att_dynamic=8000 znamená interní nenulový body target/thrust error, nikoliv 8000 dynamických pilotních operací. limited=3999 jsou DISARMED shutdown flags, ne ozbrojená saturace. Skutečná nenulová FF, shaping, I bez resetů, ACRO/mode transitions a široké reference zůstávají HW neověřené; předchozí dynamický důkaz je SITL, ne tento test.

## 7. H743 časování a srovnání

Skutečné AP_HAL monotónní MCU wall-time brackety včetně případné preempce. Žádný SITL čas nebo host délka se nevydává za H743 compute time.

| Bracket | Součet [µs] | Počet | Průměr/volání [µs] | Průměr/main [µs] | Max z exportu [µs] |
|---|---:|---:|---:|---:|---:|
| Attitude pre capture | 32225 | 8000 | 4,028125 | 8,05625 | nedostupné |
| Attitude post capture/audit bez consume | 26807 | 8000 | 3,350875 | 6,70175 | nedostupné |
| Attitude sync/validace | 149811 | 8000 | 18,726375 | 37,45275 | nedostupné |
| Attitude math | 148749 | 8000 | 18,593625 | 37,18725 | nedostupné |
| Attitude comparison | 78270 | 8000 | 9,78375 | 19,5675 | nedostupné |
| Attitude consume | 459891 | 8000 | 57,486375 | 114,97275 | **203** |
| Rate PID math | 31635 | 3999 | 7,910728 | 7,90875 | nedostupné |
| Rate PID consume | 242154 | 3999 | 60,553638 | 60,53850 | nedostupné |
| Rate process | 248223 | 3999 | 62,071268 | 62,05575 | **74** |
| Native Rate + capture | 65252 | 3999 | 16,317079 | 16,31300 | nedostupné |

Nepřekrývající se dostupné přidané části jsou **pre + post bez Att consume + Att consume + Rate process = 767146 µs / 4000 = 191,7865 µs/main**, přibližně **7,67146 %** nominální periody 2500 µs. Vnořené math/sync/comparison/consume se znovu nepřičítají. Native Rate + capture zahrnuje původní AP a není čistá přidaná UTB režie. Maxima různých operací se nesčítají jako WCET jednoho main cyklu.

**Úplný transport/capture/audit není změřený.** Před Rate process timerem se kopíruje frame a počítá vlastní reference/upstream metriky; po něm se čte vlastní historie a účtují peak/downstream statistiky. Další Rate pre/post snapshoty, motor audit, prepare/finish/fan-out/publikace nemají samostatný čistý bracket. Attitude consume část transportu obsahuje. Součet proto není celý overhead ani jeho horní mez. Ani CPU rozdíl neizoluje konkrétní chybějící funkci; nic se neoptimalizovalo nebo nepřepisovalo.

![Aktuální brackety](../tmp/utb-3f3-hw-20261024_192647/analysis/timing-brackets.png)

### Krátký versus delší Chain test

| Veličina | 3F-2, 1000 main | 3F-3, 4000 main |
|---|---:|---:|
| Dostupný nepřekrývající se součet [µs/main] | 192,516 | **191,7865** |
| Attitude consume mean/max [µs/operaci] | 57,725 / 205 | **57,486375 / 203** |
| Rate process mean/max [µs/volání] | 62,379379 / 73 | **62,071268 / 74** |
| Rate math [µs/volání] | 8,017017 | **7,910728** |
| Rate PID consume [µs/volání] | 60,855856 | **60,553638** |
| Aktivní CPU [%] | 28,2000 | **28,1722** |
| Rate capture min/max [µs] | 2479 / 2521 | **2460 / 2540** |
| Průměrný absolutní Rate capture jitter [µs] | 1,600200 | **1,734867** |

Součet je **−0,7295 µs/main, −0,379 %**, Rate process mean **−0,494 %**, maximum o **1 µs vyšší**. Průměrný jitter je o **0,134667 µs vyšší** a rozsah širší v přibližně čtyřikrát delším okně. Různé boot session a senzorové historie nejsou izolovaný kauzální benchmark nebo důkaz statistické ekvivalence; není doložen progresivní nárůst nákladů, ale agregace neumějí zobrazit vývoj jednotlivých compute časů.

![Srovnání Chain běhů](../tmp/utb-3f3-hw-20261024_192647/analysis/comparison-3f2-3f3.png)

### Srovnání se samostatnými moduly

| Porovnatelný kontext | Samostatná HW fáze | Chain 3F-3 |
|---|---:|---:|
| Attitude math [µs/operaci] | 3E-5: 17,01475 | **18,593625** |
| Attitude consume [µs/operaci] | 3E-5: 47,216875 | **57,486375** |
| Attitude consume [µs/main, 2 operace] | 3E-5: 94,43375 | **114,97275** |
| Rate PID math [µs/volání] | 3D-5: 8,69875 | **7,910728** |
| Rate PID consume [µs/volání] | 3D-5: 63,243 | **60,553638** |

Chain Attitude consume obsahuje také integrační transport/slot a odlišný wrapper; samostatné i propojené math brackety pracují nad jinými skutečnými senzorovými historiemi/booty. Vyšší Attitude a nižší Rate cena nejsou důkaz změny algoritmu; ověřená matematika je beze změny. Starý 3E-5 součet 107,6255 µs/main měřil pouze Attitude, nelze jej porovnávat jako cenu celého řetězce. CPU bylo 3E-5 přibližně 17,92→22,62 %, 3D-5 17,98→21,15 %; jejich systémové load rozdíly nelze sčítat na přesnou režii Chain. Starší exporty poskytovaly další maxima, nynější Chain je neexportuje.

### Perioda

**3998 intervalů**, součet **9 995 000 µs**, first→last timestamp **162154115→172149115 µs**. Min/max **2460/2540 µs**, mean **2500 µs**, průměrný absolutní jitter **1,734867 µs**, frekvence **400 Hz**. Z maxima plyne, že v těchto 3998 Rate capture intervalech není ≥5ms prodleva. Není tím dokázána absence všech deadline misses celé smyčky nebo celého bootu.

Jde o čas v `before_rate` capture. **Main perioda, Attitude entry perioda, PM.NLon/MaxT a task-budget overruns jsou neověřené**, nikoliv nulové. Timing je vypnutý, tasks.txt se nečetl a logger se nezapnul. Neexistuje raw seznam všech period, histogram nebo P95/P99; nevyrobily se z agregací. Starý 3D-5 rate-entry bod a 3E-5 Attitude entry intervaly mají odlišný kontext; nelze z jejich jitteru vyvozovat nové Main měření.

## 8. Scheduler load, stack a zdraví

| SYS_STATUS load okno | N | Průměr [%] | Min–max [%] |
|---|---:|---:|---:|
| Baseline | 30 | **18,363333** | 17,9–18,8 |
| Vnitřní aktivní část | 18 | **28,172222** | 27,8–28,7 |
| Po testu před FTP | 16 | **18,393750** | 17,9–19,1 |

Aktivní host okno +0,5 až +9,5 s od triggeru odstraňuje okraje; baseline trvá 15,016 s. Load je průměrovaný scheduler údaj, ne per-cycle CPU profil nebo cena všech vláken/ISR. Nárůst **+9,808889 p.b.**, proti +10,116667 p.b. v 3F-2; rozdíl přírůstků **−0,307778 p.b.**. Návrat po je **+0,030417 p.b.** proti baseline. Není zjištěné přetrvávající zvýšení load po testu; nelze prohlásit celý neměřený rozdíl proti bracketům za cenu konkrétní kopie nebo mutexu.

![CPU před/během/po](../tmp/utb-3f3-hw-20261024_192647/analysis/cpu.png)

| Watermark, volné/přidělené B | Před oknem | Po včetně exportu |
|---|---:|---:|
| ArduCopter main | **4344/7168** | **4344/7168** |
| ISR | **1280/1536** | **1280/1536** |
| FTP | **1592/2992** | **1456/2992** |

Main má oproti jinému bootu 3F-2 o **8 B méně**, ale bylo to přítomné už v preflightu. Během experimentu se maximum využití main dále nezvýšilo; pokles FTP **136 B** je v intervalu včetně exportu. Nejde o okamžité stack consumption konkrétní funkce. Známý vlastní frame complete_rate **832 B** plus engine Rate frame **200 B** není transitive call-chain; runner použil kontrolu rezervy nad těmito známými 1032 B jen jako nutnou dílčí podmínku, nikoliv schválený bezpečnostní limit. Startup watermark může zastínit pozdější menší špičku. Dostatečná rezerva pro všechny dynamické/letové kombinace se z tohoto pozorování neprokazuje.

![Stack](../tmp/utb-3f3-hw-20261024_192647/analysis/stack-watermark.png)

Boot counter **106** a monotónní uptime zůstaly konzistentní, zachycené health gyro/AHRS/EKF/logger platné, bez pozorovaného InternalError/restartu. Pro tento omezený experiment **nebyl zjištěn časový, stack nebo health blocker**; nevytváří se z toho nový obecný limit CPU nebo letová garance.

## 9. Motorová izolace, konečný FC a Git stav

**12000 RAM auditů, motor_changes=0**. Rate kontroluje native R/P/Y, FF a body-rate target před/po inertním výpočtu; Attitude zachovává svůj původní audit. UTB pracuje nad hodnotovými vstupy/vlastní historií, nemá živý AP_Motors ani nepíše do native PID stavů. H743 aktivní UTB řízení zůstává compile-out.

Read-only audit znovu ověřil **2454 chráněných souborů** proti stavu před 3F-1: AP_Motors, HAL, arming, AC_PID, původní AP attitude matematika, ověřená UTB PID/Rate/Attitude matematika, task table a safety zdroje beze změny. 18souborový manifest 3F-1 stále souhlasí, APJ je identický s binárně ověřeným kandidátem bez aktivních UTB_Core/UTB_Output/ACRO/ANGLE symbolů. Žádný ARM ani elektrické měření ESC. Jde o RAM/source/binary důkaz izolace, ne kvalifikaci fyzického motorového řízení.

Konečná kompletní read-only záloha **1215/1215 parametrů**, stejné keys jako preflight. Proti aktuálnímu preflightu se změnil **pouze STAT_RUNTIME**. UCS/UAS/UNS_RUN=0, UCS_COUNT=4000, LOG_DISARMED=0, SCHED_OPTIONS=0. Tři stream intervaly obnovené na −1. FC DISARMED STABILIZE, zdravá, boot106, COM5 uzavřený. Čtyři staré logy mají identické inventory metadata. COUNT4000 neznamená další trigger.

Žádný nový build/upload, instalace, změna produkčního kódu nebo letových parametrů, Git commit/reset/clean/stash/rebase/push. Nový soubor mimo ignorované tmp je pouze tento report; původní 3F-2 report a kapitoly beze změny. HEAD **89e81d94763f1610b52b6e93510de3df71969af0**, branch test/skystars-5inch, index a tracked tree čisté; untracked reporty 3F-2/3F-3 a kapitoly/historie.txt.

## 10. Artefakty a závěrečná rozhodnutí

Kořen **tmp/utb-3f3-hw-20261024_192647/**:

- `source-check.json`, `physical-start.json`, `approval.json`, `test-plan.md`: integrita, nový fyzický start a konkrétní schválený plán.
- `preflight/`, `preflight-extra/`, `parameter-comparison.json`, `parameters-before.param`: identita, všechny parametry, rozdíly, pending a threads/log inventory.
- `experiment-attempt.json`, `measurement/result.json`, `measurement/telemetry.jsonl`: jediný trigger, raw vstupní/odchozí packet evidence, health, průběh a cleanup.
- `measurement/utb_chain_shadow.txt`, `measurement/shadow.json`: původní frozen snapshot a parse; `threads-before/after.txt`: skutečné nové watermarky.
- `analysis/metrics.csv`, `history.csv`, `timing-brackets.csv`, `cpu.csv`, `stack-watermark.csv`, `summary.json`: všechny dostupné numerické, resetové a časové výsledky. History CSV je agregace, ne vymyšlená per-event časová osa.
- `analysis/comparison.json`, `comparison-3f2-3f3.csv`, `*.png`: srovnání s předchozími skutečnými HW běhy a vizuálně ověřené grafy.
- `final-check/`, `parameters-after.param`, `final-parameter-changes.json`, `final-fc-check.json`, `final-integrity.json`, `artifact-sha256.json`: konečný stav FC/Git a integrita původních i nových dat.

1. **Ano, podporovaný integrovaný řetězec dokončil 4000krokové HW porovnání**, 84 exportovaných max/RMSE0 s výše rozlišenou silou důkazů.
2. **Ano, všech 3999 očekávaných Rate kroků bylo validně spárováno** po jednom warmupu; raw poslední pár a runtime guard/čítače souhlasí. Neexistující úplný offline raw audit se netvrdí.
3. **Ano, vlastní historie pokračovala po jediném plném importu**, s replay skutečných yaw/I_set událostí. Nenulová I a yaw paměť bez resetů na tomto HW stále nejsou potvrzené.
4. **Měřená část režie 191,7865 µs/main, proti 3F-2 −0,379 %**. Plný capture/transport a WCET jsou neověřené, žádné sčítání vnořených spanů.
5. **Pozorovaný load/Rate timing/stack/health bez blockeru pro tento profil**, nikoliv nový obecný bezpečnostní limit. Main/PM/overruns se neměřily.
6. **Motorová izolace zachovaná**, 12000 auditů bez změny plus source/binary evidence; původní AP má výhradní motorovou autoritu.
7. **Omezenou klidovou DISARMED HW validaci celého řetězce lze uzavřít**, včetně kratšího 3F-2 a tohoto delšího 3F-3. Deset sekund není dlouhodobá tepelná nebo letová kvalifikace.
8. **Před jakýmkoliv aktivním HW řízením chybí samostatný návrh/schválení a důkazy**: pasivní HW dynamické reference/FF/shaping/I bez resetů a resetové/mode/failsafe scénáře, relevantní saturace/anti-windup a konfigurace, úplné transportní časování a Main/deadline/stack analýza při realistické zátěži, kvalifikace plánované aktivní motorové/arming/spool/failsafe integrace po etapách. Stávající H743 active compile-out se zde neuvolňuje. Původní logger/startup anomálie nejsou RAM-only testem opravené ani vyvrácené; spolehlivost loggeru/komunikace pro budoucí let je samostatný otevřený bod. Současné výsledky neopravňují k ARM, fyzickému motorovému testu nebo letu a žádný další experiment automaticky nezačal.
