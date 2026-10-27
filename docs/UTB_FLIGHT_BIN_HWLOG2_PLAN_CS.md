# HW-LOG-2: plán jediného krátkého zápisového testu

Stav: **návrh k samostatnému schválení; nespouštěn**. Navazuje na HW-LOG-1 a konkrétní ON APJ SHA-256 `efc95cdf581920d675af3c39baffa78ce227caf815a48097cabce61b9192b7ff`. Tento dokument nepovoluje ARM, LiPo, motorové testy, změny nastavení nebo další upload.

## Doporučený trigger a rozhodovací brána

Doporučenou nejkratší cestou je **varianta A: jediný normální fyzický ARM v původním STABILIZE**, ale až po samostatné elektrické/RC kvalifikaci a výslovném souhlasu s možným roztočením motorů bez vrtulí. Využije přesně kvalifikovaný firmware a skutečnou produkční ARM-edge cestu; nevyžaduje nový kód ani další upload. Pokud níže uvedené předpoklady nejsou doložené, platí NO-GO pro tuto variantu, nikoliv výjimka z PreArm.

Potřebné předpoklady: všechny vrtule demontované, dron mechanicky zajištěný, bezpečný prostor pro motory, funkční RC s ověřeným správným throttle minimum a okamžitým normálním DISARM, ověřený battery monitor/napájení a fyzická možnost okamžitě odpojit energii. USB-only z HW-LOG-1 **nekvalifikuje LiPo, battery-under-load ani RC/disarm**. Pokud bude nutná LiPo, je její připojení a provoz výslovně součástí nového souhlasu. Žádné vypínání arming kontrol, force ARM ani změna motorového mapování/protokolu. Host nebo obsluha musí mít předem určené, kdo provede právě jeden normální ARM a DISARM; nesmí současně působit dva vlastníci řídicích příkazů.

Před spuštěním nový read-only preflight, identita board1075/serial200024001251303433333336, zdraví, správný APJ původ a zachované konfigurace. Nepředpokládat volný COM5 podle čísla; vyhledat USB serialem. Snapshot session musí být dosud nespotřebovaný. Případný nový fyzický boot pouze po samostatném potvrzení; žádný automatický reboot. Původní logy i nová data zachovat, dostatek kapacity ověřit jedním inventářem před měřicími okny, ne během nich. Bez místa zastavit a neprovádět erase.

## Varianta B: DISARMED trigger pouze jako samostatné budoucí rozhodnutí

Pokud nelze bezpečně připravit A, je B fyzicky méně riziková alternativa pro kvalifikaci flash transportu. Vyžadovala by nový výslovně schválený patch/test build a další upload: stejný jediný Chain engine, stejná matematika a fronta/worker, default OFF, explicitní test-only trigger, nejvýše5s, zákaz motorového rozhraní a podmínka trvale DISARMED. Nejde o nový PID benchmark ani druhý controller.

**Tento trigger v současném APJ neexistuje a zde nebyl implementován.** Nestačí obejít ARM podmínku: současný sample vkládá ARM flag a parser požaduje ARMED valid data. Budoucí B musí pravdivě označit DISARMED/test původ sad a odpovídající offline validační profil, nesmí vyrábět zdánlivé ARM důkazy. Musí také vyřešit sdílený finite lifecycle bez poškození FlightV1 exportu. DISARMED reference, I reset a FF mohou být nulové; B by ověřila zápis/backpressure/stack v tomto kontextu, **nenahradila by ARMED chování, vibrace, napájení, motorovou cestu ani letovou kvalifikaci**. Není to automatický fallback po neúspěchu A.

## Přesná konfigurace pro variantu A

Pouze po HW-LOG-1 PASS a novém souhlasu, ještě DISARMED:

```text
UCS_B_RATE   = 10
UCS_B_SECS   = 5
UCS_B_ENABLE = 1
UCS_RUN = UCS_F_RUN = UAS_RUN = UNS_RUN = 0
UTT_RUN = 0 před ARM
```

ENABLE musí být pozorované už v DISARMED před ARM. Nové hodnoty zkontrolovat direct read i tehdy, když Mission Planner skrývá děti vypnuté ENABLE skupiny. Native PID/ATC, motor/servo/frame, RC, failsafe, filtry a bateriové nastavení se nesmějí měnit kvůli dosažení PASS. Původní GCS failsafe respektovat; je-li aktivní, musí probíhat skutečný heartbeat z přijatého SYSID a běžnou konfigurací, nikoliv jeho vypnutí.

Pro rozlišení prvního otevření loggeru od zápisu UTB je vhodné v **téže schválené testovací epizodě** nejprve explicitně povolit LOG_DISARMED=1 a otevřít native log ještě DISARMED, s UCS_B_ENABLE=0. Tato změna sama spouští DataFlash zápis a musí být v budoucím souhlasu; HW-LOG-1 ji neprovedla. Původní LOG_BITMASK zachovat, jeho relevantní PM/DSF dostupnost ověřit předem. Změna masky nebo dalších intervalů by vyžadovala přesné doplnění souhlasu.

## Navržená krátká sekvence

1. Záloha skutečných parametrů, původních intervalů a inventáře; bezpečné fyzické podmínky, jednoznačný vlastník portu. Sekvenční memory/threads snapshot mimo měřicí okna. Připravit offline parser a časovou osu.
2. S vypnutým BIN alespoň15s pasivní baseline bez TX. Pokud existující SYS_STATUS nejsou dostatečné, vyžádat v souhlasu pouze dočasné SYS_STATUS/SYSTEM_TIME/EKF2/1/1Hz s obnovou skutečných původních hodnot. Nikoli automatické zvýšení telemetrie.
3. Zvlášť označit první otevření native loggeru: jeden LOG_DISARMED0→1, zaznamenat host TX a potvrzení. Volitelně použít **stávající DISARMED UTT_COUNT=2000/UTT_RUN=1 pouze pro tuto startup fázi**, pokud to bude výslovně zahrnuté v souhlasu. Zachytí Main/Rate/Execution periody okolo startu loggeru bez dalšího firmware. Počkat na automatické UTT_RUN0 a ustálení native logování, nejméně15s. Žádný listing, download ani FTP v startup okně. UTT nesmí být aktivní při následném ARM.
4. V DISARMED ověřit UCS_B_RATE10/SECS5/ENABLE1, všechny ostatní RUN0, zdraví a dohodnutou normální RC/disarm cestu. Logging už musí běžet ve stejném souboru. Nezkoušet žádný motor test.
5. Po novém fyzickém potvrzení provést **právě jeden normální ARM**. Throttle minimum, páčky v dohodnuté neutrální poloze. Na této hraně vznikne jediná5s BIN session; motory se přitom mohou normálně roztočit.
6. Během aktivních5s pouze příjem již domluvené telemetrie. Žádný PARAM_SET/polling, LOG_REQUEST_LIST, MAVFTP, BIN download, další RUN, změna loggeru či RC experiment. Po přibližně5s nevyvolávat další session. Pokud bude výslovně schválené krátké ARMED post okno, lze pozorovat nejvýše další3s bez SHADOW pro srovnání stejného ARM stavu; jinak ihned normální DISARM. Celkový maximální ARMED čas a odpovědná obsluha musí být ve schválení, nikoliv domyšlené za běhu.
7. Ověřit DISARMED a ukončení session; teprve pak číst frozen Chain/counters a případný startup Timing/Execution export. Žádný další pokus. Pozorovat native logování a zdraví ještě alespoň15s pro zachycení PM/DSF závěru, pak obnovit LOG_DISARMED i všechny změněné diagnostické parametry a intervaly. UCS_B_ENABLE zpět0. Žádný plošný restore kalibrací ani čítačů.
8. Sekvenční memory/threads/MEMINFO po výpočtu a po exportu, s boot/uptime kontrolou. Inventář/download až po ukončených měřicích oknech. BLOCK nemusí soubor zavřít při LOG_DISARMED0; nezavádět automatický reboot. Použít ověřený bezpečný download existujícího logu, případné fyzické odpojení pouze po novém souhlasu, až jsou RAM data uložená.

## Měření a co umožní rozlišit

Oddělit host časy příkazů, MCU TimeUS/Start, source/consume identity a délku session. Jedna sada =3UFPI+3UFRS+UFAT. Nominálně2000 main pokusů, první Rate warmup, přibližně50 vybraných sad; není garantováno50 skutečně persistentních sad. Počty ověřit Req/Que/Acc/Drp/Inv/Uns/Rej/HWM/Dep a skutečnou rekonstrukcí BIN. Kontrolovat normální reset/I_set/saturation, max/RMSE P/I/D/FF/D_FF/rates/quaternionů a full-rate RAM comparator.

Startup native logování, ustálené native logování a ARMED UTB zápis budou tři oddělená časová okna. GCS listing/export jsou až později. To umožní rozlišit časovou souvislost, **nikoliv přímo určit uvnitř backendu WaitReady versus SPI transfer versus write_sem nebo erase**: kandidát nemá zapnutou logger TRACE. Změna ARM zároveň mění native spool a regulaci, takže rozdíl DISARMED/ARMED CPU není izolovaná cena writeru. Krátké ARMED post okno může oddělení zlepšit, ale není hard WCET.

Původní PM/DSF v BIN poskytují agregace přes své intervaly. Vyhodnotit jejich skutečné časové pokrytí přes celé write okno, ne přiřadit jediný MaxT automaticky jednomu paketu. **Aktuální UTT/Execution guard nepodporuje ARMED a nesmí být obejit.** Žádná nová400Hz full-detail telemetrie/logging. Pokud PM/DSF nepokryjí relevantní okno nebo nebude dostupná příslušná Main/Rate latence, napsat INCONCLUSIVE pro tuto veličinu; desetihertzové TimeUS nejsou trace400Hz jitteru. Historické5–6.5ms anomálie nebyly prokázaně vysvětlené; nový výsledek nesmí tuto nejistotu přepsat.

Pro BLOCK zůstávají rizika StartNewLog/init, write_sem, sem, dev_sem, IO thread, sector erase a SPI contention. První otevření v DISARMED samo neprokazuje absenci pozdějšího blokování, recovery nebo erase. Při novém problému nepřipisovat viníka bez příslušných důkazů a nepřidávat automaticky další sondu/patch.

## Datový a paměťový rozpočet

50 sad ×450 B =22500 B. Přibližně1Hz UFLS55 B; konzervativně až7statusů=385 B. Nové FMT/FMTU nejvýše532 B, jsou-li v logu potřeba nově. Celkem přibližně23417 B tohoto diagnostického přídavku, plus společná metadata, **native AP data za baseline/aktivní/post okna**, stránkové hlavičky6/256 a rezervy BLOCK. Největší položkou může být native log, ne UTB. Aktuální volná flash kapacita není jednoduše16MiB minus délky stažených logů; při nejasné rezervě zastavit. Žádný erase bez zvláštního souhlasu.

SPSC má4sady:200ms při20Hz nebo400ms při10Hz kapacity proti nepostupujícímu consumeru, nikoliv garantovaný časový limit celé flash operace. Worker má ještě jednu lokální kopii. Sleduj přijetí backendem odděleně od fyzické persistence. RAM/stack vždy v totožných definicích: reported stack watermark, nikoliv sizeof working area nebo varovný práh. Nesčítat FREE regiony do univerzálního heapu; uvést TYPE/FREE/LRG pro každý region a případnou alokační chybu.

## STOP, cleanup a přijetí

Okamžitě ukončit fyzický experiment běžným DISARM/odpojením energie podle předem dohodnuté odpovědnosti při neočekávané rotaci/chování, ztrátě RC/disarm, InternalError, restartu, selhání senzoru/EKF/loggeru, neshodě, DESYNC, nepodporovaném kontextu, alokační/stack chybě nebo významném zhoršení timing/USB. Při zjištění časové anomálie v pozdějším exportu zastavit další postup. Žádný retry/force ARM/automatický reboot/erase/upload. Při chybě nemazat důkazy.

Přijmout pouze jednoznačný firmware, zdravou FC, schválený trigger, zachovanou native motorovou autoritu a konfiguraci, úplné rekonstruované validní sady, nulové neobjasněné numerické odchylky, ztráty i nečekané motorové změny. Chybějící terminal UFLS či nevysvětlené dlouhé intervaly znamenají neuzavřenou kvalifikaci. Zvláště perioda/prodleva≥5ms vyžaduje vyšetření, nikoliv změnu scheduler limitu. Krátký test není důkaz hard WCET ani několika minut letu.

Po skutečném DISARM zkontrolovat všechny RUN0/UCS_B_ENABLE0, původní letové parametry i intervaly, zdraví a zavření COM. Uložit nový BIN jednou s SHA a receipt; při existenci starých logů nic nemazat. Offline:

```bash
.venv/bin/python Tools/utb/tools/utb_flight_bin_analysis.py /cesta/HWLOG2.BIN \
  --output tmp/utb-flight-bin-hwlog2-analyza
```

Zkontrolovat JSON/CSV, konzistenci všech sedmi paketů, source/consume, uložení skutečných vzorků a soulad s RAM čítači. Pokud chybí poslední status, nelze přijetí backendem vydávat za potvrzený kompletní persistentní log.

## Rozsah potřebného nového souhlasu

Schválení musí jmenovat tento hash firmware, variantuA, skutečné napájení a RC/disarm předpoklady, jeden normální ARM a maximální ARMED čas, změny UCS_B a LOG_DISARMED, případné startup-only UTT_COUNT/RUN a dočasné telemetry intervaly, jeden následný download a cleanup. Nesmí zahrnout force ARM, motor test, novou session, erase ani další upload. VariantaB by vyžadovala samostatné vývojové zadání a nový firmware souhlas.

**HW-LOG-2 dosud neproběhla.** Ani její úspěch sám nepovolí let. Před FLIGHT-1/2 zvlášť rozhodnout o native RC, baterii, motorech/směrech, orientaci, EKF/yaw a vhodnosti platformy. Není-li pasivní logger kvalifikovaný, první schválený native let může být pouze s jeho runtime OFF; aktivní UTB řízení zůstává nedostupné.
