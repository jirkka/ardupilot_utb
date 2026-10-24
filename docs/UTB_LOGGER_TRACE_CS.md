# RAM sonda loggeru pro FÁZI 3D-2

Stav 21. 10. 2026: capture/export sondy **omezeně HW ověřeno**; first-open příčina **neobjasněna**.
Nejde o opravu příčiny prodlevy, nový SHADOW ani změnu řízení. Výsledky buildů
 a testů jsou v závěrečném reportu [3D-2](UTB_PHASE_3D2_LOGGER_CS.md).

## Aktivace a datový tok

Samostatný compile-time přepínač `AP_UTB_LOGGER_TRACE_ENABLED`, default **0**;
Waf `--enable-UTB_LOGGER_TRACE` / `--disable-UTB_LOGGER_TRACE`.
Nezávisí na aktivní UTB letové větvi. H743 active compile-out se nerozšiřuje.
Žádný nový AP parametr, DataFlash message ID nebo motorové rozhraní.

Po 30 s boot času začne pouze DISARMED main thread ukládat pevné RAM záznamy.
30 s je diagnostické vyloučení úvodního bootu, nikoliv požadavek letové bezpečnosti
nebo záruka dokončené inicializace. Gyro/AHRS health musí ověřit obsluha.
První mezi-sample perioda nad nominal + nominal/5 zmrazí předcházející cyklus.
Při 400 Hz jde o práh 3000 µs odpovídající stávajícímu PM prahu.
Práh PM, task budgety ani regulace se nemění. Ostatní frekvence se nekvalifikují
 tímto HW protokolem. Armování před freeze ukončí capture s aborted=1 a period=0; již zmrazený
záznam se nemění.
nevytváří se zákaz armování, změna režimu ani zásah do tahu.

Jednorázový freeze je do restartu neměnný; není příkaz k resetování capture.
Pevný ring má 128 slotů, navíc souhrn count/max pro každý druh operace.
Při přepsání detailu zůstane maximum celé dlouhé operace. Ztracené dokončení
přepsaného slotu se počítá; nelze jej interpretovat jako kompletní stopu.
Task INT16_MIN v souhrnu znamená UNKNOWN při ztrátě původního detailu.

Sledují se:

| Event | Operace |
| --- | --- |
| 0 | start_new_log |
| 1 | find_last_page |
| 2 | PageToBuffer včetně cache-hit cesty |
| 3 | WaitReady |
| 4 / 5 | chip sem wait / tělo držení při startu a hledání stránky |
| 6 / 7 | SPI dev_sem wait / tělo držení při čtení stránky a ReadStatusReg |
| 8 | main handle_param_set; nikoliv fyzické async uložení parametru v IO |
| 9 / 10 | write_sem wait / tělo držení v Block::_WritePrioritisedBlock |
| 11 | native scheduler task-budget overrun, s původním task ID a časem |

Scheduler overrun používá už existující start/time_taken; nepřidává čtení hodin.
Není to počet ztracených 400Hz cyklů. Task ID odpovídá spojenému seznamu úloh
konkrétního buildu; jména lze následně ověřit z původního @SYS/tasks.txt.
Jeho první čtení může aktivovat statistiky/alokaci; provádět až mimo okno.

## Časové a synchronizační meze

Všechny spany jsou preemptible wall time z AP_HAL::micros, ne izolované CPU cycles.
Vnořené intervaly se nesčítají jako nezávislé náklady. Hold-body nezahrnuje
samotné uvolnění původního guardu a část režie sondy při dokončení záznamu.
Není to přesná celková délka fyzického držení mutexu. Wait interval rovněž
obsahuje režii sondy a případnou preempci.

Sonda nepřidává zámky, spin wait, heap, text nebo DataFlash do capture.
Původní pořadí acquire/release se zachová. Každý scope přidává main-thread check,
časové značky a omezené uložení do RAM; IO volání jsou vyloučena.
**Držitel zámku v IO se touto první sondou přímo nezjišťuje.** Dlouhý main wait
lokalizuje zámek, nikoliv automaticky jeho držitele. WaitReady měří celý span,
nezaznamenává přímo jednotlivé návratové hodnoty Busy; dlouhý wall span sám
neprokazuje, kolik času tvořila flash busy oproti preempci.

Jediný writer je main; po dokončení všech jeho operací na hranici smyčky
publikuje freeze release atomikou. Reader nejprve provede acquire a pak čte
jen neměnná pole. Žádné další resetování nebo přepis po freeze. Před freeze
export nečte živá pole a vrací pouze pending. Publikační bool musí být lock-free.
Přetečení 32bit času se vyhodnocuje modulární aritmetikou; span musí být kratší
než celý rozsah hodin. Ticket se při UINT32_MAX vyčerpá s explicitním příznakem.

Ring obsahuje pořadí vložení, **nikoliv vždy pořadí začátků operací**:
scheduler overrun se vloží až po návratu úlohy s jejím původním startem.
Pro analýzu použít cycle a časový offset vůči triggeru, včetně modulo wrap.
Souhrnná maxima zahrnují celý capture od aktivace, nikoliv jen poslední cyklus;
pro časovou souvislost kontrolovat start_us a detailní záznamy.

## Export

Read-only MAVFTP soubor `@SYS/utb_logger.txt`, dostupný jen v TRACE ON buildu.
Před freeze: `UTBLoggerTraceV1,pending`. Po freeze: metadata trigger/period,
tickets/lost/aborted/exhausted, tabulka summary a tabulka jednotlivých spanů.
Za armed stavu export odmítne poskytovat capture. ExpandingString alokuje
stejně jako ostatní @SYS soubory až při požadavku na export; nikoliv při měření.
**Soubor se během měřicího okna nesmí pollovat.** Jeho čtení by změnilo zátěž.

Předchozí BIN/AP logy a původní formáty se nemění. Nová diagnostika nic sama
neposílá do loggeru, USB ani GCS. Proto není hláška o freeze v main cestě.

## První HW session po samostatném schválení uploadu

1. Ověřit USB serial 200024001251303433333336, board 1075, DISARMED,
   bez LiPo/vrtulí, volný port a zdravé senzory/logger. Záloha parametrů.
2. Ověřit identitu a SHA konkrétní TRACE ON APJ; bez samostatného souhlasu
   nenahrávat. Zachovat bootloader, parametry a existující logy.
3. Nový fyzický start provede obsluha; žádný automatický reboot. UB3_RUN=0,
   UTB_SHADOW=0, FSTRATE_ENABLE=0, SCHED_OPTIONS=0, LOG_DISARMED=0.
4. Ověřit dostatečnou kapacitu před povolením logování. Pokud by hrozil wrap,
   zastavit. Zaznamenat časy a health baseline po inicializaci a po 30s aktivaci.
5. Jednou povolit LOG_DISARMED=1. Bez současného list/download/FTP a dalších
   parametrických operací sledovat omezené okno, potom obnovit původní hodnotu.
6. Až po okně stáhnout @SYS/utb_logger.txt a potřebný nový BIN. Pokud freeze
   nastal před zapnutím loggeru, hodnotit zachycenou baseline událost; netvrdit
   first-open příčinu. Při pending nepovažovat měření za zachycenou prodlevu.
7. Z dlouhého cyklu určit nejdelší související span, jeho native task ID a
   intervaly čekání/flash/SPI. Kde stopa nestačí, ponechat přesnou nejistotu.
   Při události nebo poruše zastavit další běhy, obnovit parametry a health.
8. Porovnat ON/OFF se stejnou AP konfigurací pro režii sondy; neschvalovat
   let ani 400Hz SHADOW z tohoto DISARMED měření.

Příprava logu v DISARMED nebo přesun flash operace se bude implementovat až
podle konkrétní stopy. Tato sonda nezkracuje původní čekání a nemaskuje NLon.


## Výsledek prvního TRACE ON hardware pokusu

Schválený firmware byl nahrán a jeden cold-start experiment proveden.
Kontrola kapacity přes LOG_REQUEST_LIST před povolením loggeru zmrazila sondu
na periodě 24 238 µs, úloha GCS::update_receive 23 938 µs.
start_new_log má v capture count=0. Pozdější BIN obsahuje PM.MaxT=6070 µs
a NLon=1 v intervalu prvního otevření logu; tato operace již není v RAM stopě.
**Původní příčina tedy není prokázána a není opravena.**
Podrobná stopa a meze jsou v závěru [reportu 3D-2](UTB_PHASE_3D2_LOGGER_CS.md).

Před případným nově schváleným pokusem ověřit kapacitu a zálohu ještě před
fyzickým cold startem; po něm nedělat listing/download ani FTP před first-open.
@SYS/tasks.txt má navíc vedlejší účinek zapnutí RAM SCHED_OPTIONS bit0;
nekombinovat jej s měřicím oknem. V tomto pokusu byla původní hodnota obnovena.
Tato poznámka není souhlas s opakováním ani s úpravou sondy nebo regulátorů.
