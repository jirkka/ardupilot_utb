# Diagnostika měření baterie Skystars H743 před FLIGHT-1

Datum: 26. 10. 2026. **PARTIAL – příčina na úrovni analogového signálu vymezena, fyzická příčina nepotvrzena. Bateriová kvalifikace pro FLIGHT-1: NO-GO do uzavření níže uvedených bodů.**

## 1. Firmware, hardware a rozsah

Repo `/home/jirka/ardupilot_utb`, větev `test/skystars-5inch`, HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`. SkystarsH7HD-bdshot, STM32H743, board ID1075, USB serial200024001251303433333336. Soubor připraveného/nahraného APJ má 1162543 B a SHA-256 `efc95cdf581920d675af3c39baffa78ce227caf815a48097cabce61b9192b7ff`; hash na hostu znovu ověřen. Živá FC hlásí ArduCopter4.7.1/11a518e5; nejde o nové čtení celé flash a ověření jejího hashe.

Po novém fyzickém potvrzení proběhlo jedno read-only připojení COM5, boot127, 27,2215 s. FC trvale DISARMED, STABILIZE; AHRS/EKF/gyro/logger health prošly, InternalError i logger drops v přijatých SYS_STATUS nulové. USB-only, LiPo odpojená, vrtule sundané. Port je zavřený. Žádné PARAM_SET, změny intervalů, ARM, motorové příkazy, log listing/download, MAVFTP, reboot, upload ani nový benchmark. Existující `UCS_B_ENABLE=1` ponecháno; samotné povolení nezpůsobilo DISARMED experiment. Ostatní přečtené RUN byly0. Nový firmware ani regulátory se neměnily.

## 2. Aktuální parametry – skutečný boot127

Přečteno jednotlivými PARAM_REQUEST_READ, nikoliv odhadnuto z historického exportu. Všechny společné bateriové/ESC parametry souhlasí s PARM v obou BIN; změna STAT_BOOTCNT124/126→127 a runtime je evidovaná zvlášť. Níže jsou skutečně dostupné názvy; žádný domnělý parametr nebyl přidán.

| Parametr | Aktuální hodnota |
|---|---:|
| `BATT_MONITOR` | 4 |
| `BATT_CAPACITY` | 3300 |
| `BATT_SERIAL_NUM` | -1 |
| `BATT_LOW_TIMER` | 10 |
| `BATT_FS_VOLTSRC` | 0 |
| `BATT_LOW_VOLT` | 13.6000004 |
| `BATT_LOW_MAH` | 0 |
| `BATT_CRT_VOLT` | 13.1999998 |
| `BATT_CRT_MAH` | 0 |
| `BATT_FS_LOW_ACT` | 0 |
| `BATT_FS_CRT_ACT` | 0 |
| `BATT_ARM_VOLT` | 14 |
| `BATT_ARM_MAH` | 0 |
| `BATT_OPTIONS` | 0 |
| `BATT_VOLT_PIN` | 10 |
| `BATT_CURR_PIN` | 11 |
| `BATT_VOLT_MULT` | 12.1254702 |
| `BATT_AMP_PERVLT` | 59.5 |
| `BATT_AMP_OFFSET` | 0 |
| `BATT_VLT_OFFSET` | 0 |
| `BATT2_MONITOR` | 0 |
| `BATT3_MONITOR` | 0 |
| `BATT4_MONITOR` | 0 |
| `BATT5_MONITOR` | 0 |
| `BATT6_MONITOR` | 0 |
| `BATT7_MONITOR` | 0 |
| `BATT8_MONITOR` | 0 |
| `BATT9_MONITOR` | 0 |
| `MOT_BAT_VOLT_MAX` | 25.2000008 |
| `MOT_BAT_VOLT_MIN` | 19.7999992 |
| `MOT_BAT_CURR_MAX` | 0 |
| `MOT_BAT_CURR_TC` | 5 |
| `MOT_BAT_IDX` | 0 |

Dále SERIAL3_PROTOCOL=-1 (UART3 ESC telemetrie není nakonfigurovaná), SERVO_BLH_BDMASK=15, SERVO_DSHOT_ESC=0 v obou BIN; poslední položka nebyla v novém připojení samostatně čtena. BATT2–BATT9_MONITOR=0. Kapacita3300mAh není potvrzená kapacita zamýšleného packu. ARM_MAH/LOW_MAH/CRT_MAH=0 znamená vypnuté kapacitní prahy; není důvod svévolně přepsat nominální kapacitu na0.

## 3. Rozbor dodaných BIN a integrita

Po doplnění jsou skutečné soubory pojmenované jinak než v zadání. Použity byly přímo tyto kopie; nebyly přejmenované ani přepsané. Boot124 odpovídá staršímu záznamu bez platného RTC, boot126 novějšímu s koncem ARM testu a platným RTC v závěru. Totožnost původních názvů nelze doložit pouhým novým názvem.

| Soubor | Velikost B | SHA-256 |
|---|---:|---|
| logs/log_utb_01.bin | 636746 | `d913030679a55f4d52f6c45e100abde77f7cb9a17b8c8c417c1489930a91b16d` |
| logs/log_utb_02.bin | 1791996 | `fb0bf84b93c63db3f96bfe2dbb76c40e6b2fdd8148887ab6d8d84bc219201b75` |

| Veličina | log_utb_01 / boot124 | log_utb_02 / boot126 |
|---|---:|---:|
| BAT vzorky instance0 | 153 | 480 |
| BAT.Volt min/max V | 16,631390 /16,703985 | 16,552628 /16,712046 |
| BAT.Curr přesně0 / konečné vzorky | 153/153 | 475/480 |
| BAT.Curr min/max A | 0/0 | 0/0,002996063 |
| BAT.Curr průměr A | 0 | 0,0000112352 |
| CurrTot raw min/max | 0/0 | 0/0,00014984794 |
| EnrgTot min/max Wh | 0/0 | 0/0,00000248113 |
| RemPct | 100 | 99–100 |
| BAT.H | vždy1 | vždy1 |
| BAT.Temp / Res / SH | všechny0 | všechny0 |
| ESC paketů | 6149 | 19200 |
| ESC.RPM maximum | 2900 | 7628 |
| ESC.Volt/Curr/CTot/Temp | všechny0 | všechny0 |
| RCOU C1–C4 maximum | 1100 | 1286 |
| Samostatný RPM paket | žádný | žádný |
| ERR paket | žádný | žádný |

Pět nenulových proudů v novějším logu: 42,110100 s /0,000299606 A; 42,810005 s /0,000299606 A; 43,010228 s /0,001198425 A; 43,310105 s /0,000599213 A; 43,610059 s /0,002996063 A. Není to NaN, chybějící pole ani úplně konstantní0. Zobrazení se dvěma desetinnými místy by všechny tyto proudy zaokrouhlilo na0,00A. Graf záměrně používá mA.

CurrTot je v CSV ponecháno jako raw hodnota dekodéru; `Log_Write_BAT` ukládá interní consumed_mah. Nepoužívá se pro odhad skutečně odebrané kapacity, stejně jako RemPct99–100 není důkaz téměř plné baterie při nefunkčním proudovém měření. BAT.Temp0 u analogového backendu nedokazuje naměřenou teplotu0°C; logger používá0 při chybějící teplotě.

POWR: v obou souborech Vcc i VServo vždyNaN, Flags/AccFlags0, Safety2–6. NaN těchto nepodporovaných rail údajů není automaticky závada napájení; POWR neměří proud hlavní LiPo. Nejsou BAT NaN ani BAT.H0. V uložených MSG není battery failsafe hláška; absence hlášky sama o sobě není test failsafe. Novější log obsahuje dřívější `Arm: Gyros inconsistent` v17,410147 a24,010293 s; následný ARM v32,610186 s byl Forced0. Historická hláška se nezamlčuje ani nepřipisuje baterii.

## 4. Časová osa a graf

![Napětí, proud, RPM a motorové příkazy](/home/jirka/ardupilot_utb/tmp/utb-battery-diagnostic-20261026/battery-timeline.png)

V novějším logu explicitní ARM=1 v32,610186 s, DISARM=0 v58,110127 s; interval25,499941 s. Ve starším je jen DISARM=0 v40,570391 s, proto není počátek ARMED stavu domýšlený. RCOU jsou záznamy původních AP příkazů, nikoliv měření mechanického tahu. RPM jsou převzatá ESC pole při nastavených14 pólech, počet pólů nebyl fyzicky ověřen.

| Okno novějšího logu | BAT vzorků | Napětí min–max V | Proud min–max A |
|---|---:|---:|---:|
| Před ARM | 200 | 16.681395–16.712046 | 0.000000000–0.000000000 |
| První sekunda po ARM | 9 | 16.677181–16.683411 | 0.000000000–0.000000000 |
| Celé ARMED okno | 255 | 16.552628–16.683411 | 0.000000000–0.002996063 |
| První sekunda po DISARM | 10 | 16.645126–16.660940 | 0.000000000–0.000000000 |
| Celé okno po DISARM | 25 | 16.645126–16.664848 | 0.000000000–0.000000000 |

Okna první sekundy jsou podmnožinami celých oken, počty se nesčítají. V235 BAT časech s nedávnými RPM>=1000 je230 nulových a5 nepatrně kladných proudů. V107 BAT časech s RPM4000–8000 je102 nul a5 nepatrně kladných proudů. CSV přiřazuje maximum ESC.RPM jen z předchozích100ms, nikoliv nejbližší budoucí hodnotu; nejde o současné ADC/RPM sample. Napětí při otáčení klesá a po vypnutí roste, proud však nepřináší použitelné měření. Žádný proud se z RPM neodvozuje.

## 5. ADC mapování konkrétní desky

[Základní hwdef](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD/hwdef.dat:92) deklaruje PC0 BATT_VOLTAGE_SENS ADC1 a PC1 BATT_CURRENT_SENS ADC1. Defaulty jsou VOLT_PIN10, CURR_PIN11, VOLT_SCALE11.1, CURR_SCALE59.5. [Bdshot varianta](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD-bdshot/hwdef.dat:4) základ zahrnuje a PC0/PC1 neruší.

Nejde pouze o návrhový hwdef: zachovaný [vygenerovaný hwdef.h použitého ON buildu](/home/jirka/ardupilot_utb/tmp/utb-flight-log1-20261026/H743_ON/SkystarsH7HD-bdshot/hwdef.h:158) obsahuje `{10,10,3.30/4096}` PC0 a `{11,11,3.30/4096}` PC1. MCU [ADC1 mapa](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/hwdef/scripts/STM32H743xx.py:1291) souhlasí. BATT2 default CURR_PIN12 se netýká aktivní instance0; BATT2 je vypnutý a jeho jiné mapování se neaktivuje.

[AnalogIn.cpp](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/AnalogIn.cpp:99) na H7 používá16bit ADC přenosy, přepočet1/16 vůči základnímu12bit škálování; ADC DMA callback a průběžné vzorkování jsou ve stejné třídě. `voltage_average()` v200 převádí průměr ADC na V; `set_pin()` v223 a `valid_analog_pin()` v139 řeší softwarové mapování. `channel()` v799 alokuje AnalogSource při inicializaci; tento audit nic nepřidává do fast path.

## 6. Skutečný backend a význam nul

[AP_BattMonitor_Params.h:18](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_Params.h:18): typ4=ANALOG_VOLTAGE_AND_CURRENT. [AP_BattMonitor.cpp:583](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor.cpp:583) vytváří AP_BattMonitor_Analog. Nečte automaticky proud z ESC. Typ9=BLHeliESC je samostatný backend, nikoliv vhodná náhrada bez důkazu dostupné proudové telemetrie.

[AP_BattMonitor_Analog::read](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_Analog.cpp:151):

```text
BAT.Volt = (voltage_average(PC0) - BATT_VLT_OFFSET) * BATT_VOLT_MULT
BAT.Curr = (voltage_average(PC1) - BATT_AMP_OFFSET) * BATT_AMP_PERVLT
```

V logovaných i aktuálních parametrech OFFSET0, PERVLT59.5. Nejvyšší proud0,002996063 A tedy odpovídá průměru PC1 přibližně50,354µV; většina vzorků odpovídá0V. Je to matematicky rekonstruovaný ADC vstup podle ověřeného backendu, **nikoliv měření multimetrem ani přímý export raw ADC**. Maximum odpovídá přibližně jednomu nominálnímu16bit ADC kroku při3,3V. Kvantizace/přesnost skutečné desky nebyla změřená.

`has_current()` v186 zjišťuje existenci softwarového AnalogSource a zvolený typ, ne fyzickou přítomnost senzoru či kontinuitu kabelu. Healthy vychází ze `set_pin`, nikoliv odezvy na skutečný proud. Odpojený nebo uzemněný platný vstup tedy může mít H1 a měřený proud0. [Log_Write_BAT](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_Logging.cpp:12) při has_current=false zapisujeNaN; při true zapisuje přímo current_amps a spotřebu. Nuly nejsou automatická náhrada nevalidního proudu loggerem. [update_consumed](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_Backend.cpp:358) integruje naměřený proud; [capacity_remaining_pct](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_Backend.cpp:64) používá spotřebu a nominální kapacitu. I nepatrná spotřeba může celočíselný RemPct snížit100→99.

Aktuální USB-only BATTERY_STATUS hlásí přibližně0,05A a0,002V, bez LiPo; to není skutečný odběr odpojené baterie ani kalibrace. Dokládá to, že čtení proudu není ve firmware trvale přepsané na0. Nelze to přenést na analogový signál při napájení ESC.

## 7. Schopnosti FC, ESC a potvrzený datový tok

```text
LiPo → ESC/PDB/proudový senzor [model a vedení NEOVĚŘENÉ]
     → analogový CUR [existence, úroveň a propojení NEOVĚŘENÉ]
     → FC PC1 / ADC1 channel11 [software OVĚŘENO]
     → AP_BattMonitor_Analog, typ4 [PARM + zdroj OVĚŘENO]
     → BAT.Curr / MAVLink BATTERY_STATUS [BIN + aktuální telemetrie OVĚŘENO]
```

Dosavadní dokumenty rozlišují testovací Skystars od budoucí MicoAir45A AIO; model ani proudový senzor této konkrétní ESC nejsou doloženy. Nelze převzít zapojení z jiné podobné desky. Potřeba je označení ESC/revize a konektor/pady CUR→FC CUR, včetně společné země. Fyzická kompatibilita4S/6S FC/ESC/VTX není potvrzena samotným názvem hwdef.

[AP_ESC_Telem.cpp:679](/home/jirka/ardupilot_utb/libraries/AP_ESC_Telem/AP_ESC_Telem.cpp:679) může zapsat ESC paket po aktualizaci pouze RPM; ostatní uložená pole nejsou v tomto paketu doplněna příznakem validity každého pole. ESC.Volt/Curr/Temp0 tedy nedokazuje funkční senzor s nulovým proudem. Ve všech dodaných ESC paketech tato pole zůstávají0. [RCOutput_bdshot.cpp:782](/home/jirka/ardupilot_utb/libraries/AP_HAL_ChibiOS/RCOutput_bdshot.cpp:782) umí další EDT typy pouze za odpovídajícího ESC typu. SERVO_DSHOT_ESC0 a vypnutý UART3 neposkytují důkaz aktivní proudové telemetrie; RPM samotné nestačí. [AP_BattMonitor_ESC::read](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_ESC.cpp:51) potřebuje skutečně validní get_current a voltage, pro které důkaz chybí.

## 8. Varianty příčiny – důkazy PRO a PROTI

| Varianta | PRO | PROTI / co chybí | Závěr |
|---|---|---|---|
| A: senzor vůbec není | PC1 téměř0; ESC proudová data nedoložena | Bez modelu/schématu nelze určit osazení | Otevřená hypotéza |
| B: senzor je, CUR není připojen | Prakticky žádná odezva PC1 během motorů | Chybí kontrola kabelu/padů, společné země a výstupu | Otevřená hypotéza |
| C: chybný ADC pin | Nesprávná revize či fyzická cesta může vést jinam | Aktivní11 přesně odpovídá PC1 hwdef i build artefaktu | Chyba softwarového nastavení nedoložena |
| D: ADC current není dostupný | Nic konkrétního | PC1/ADC1 je v MCU mapě i vygenerovaném buildu | Softwarová nedostupnost vyloučena pro doložený build |
| E: chybný gain/offset | Skutečná ESC kalibrace není známá |59.5 je board default, offset0; pin je u ADC dna, změna gainu neopraví chybějící signál | Nekalibrovat naslepo |
| F: chybný MONITOR | Pokud hardware nemá analog CUR, typ4 by neodpovídal skutečnému vybavení | Pro analog V+I je4 správně; pin10 měří měnící se napětí | Podmíněně správný; fyzická část neověřena |
| G: proud je jen přes ESC telemetrii | Některé ESC to mohou umět | RPM ano, proud/volt/nebo validita proudu nedoloženy; UART3 vypnutý | Typ9 zatím nenavrhovat jako opravu |
| H: nuly z logování/validity | H1 může vzbudit falešný dojem fyzické validity | BAT obsahuje float malé kladné hodnoty, ne NaN; logger nekóduje chybějící proud nulou | Formát/display vysvětluje0,00A, ne chybějící analogový signál |

Zkrat na zem, vadný senzor, nesprávný napájecí režim senzoru nebo proudová cesta obcházející shunt jsou další fyzické možnosti; bez schématu je nelze vybrat.

## 9. Potvrzená příčina a hranice závěru

**Potvrzeno:** používaný analogový proudový kanál PC1 při zaznamenaném testu poskytoval pouze vstup u0V. Známá konfigurace jej lineárně převedla na0 až3mA, přestože RPM i RCOU se měnily. Zobrazení0,00A není důkaz absolutní binární nuly; pět vzorků bylo nenulových. Proud nebyl logem vynechán ani čerpán z ESC RPM.

**Nepotvrzeno:** proč fyzický vstup PC1 neposkytuje použitelné proudové napětí. Osazení senzoru, kabel, zem, revize desky nebo vlastní senzor vyžadují fyzický důkaz. Není poctivé vybrat A/B/C/E jen z tohoto BIN. Nejde o prokázanou chybu UTB, ADC driveru nebo PID.

## 10. Minimální oprava – návrhy bez zápisu

Proudový pin11, MONITOR4, AMP_PERVLT59.5, AMP_OFFSET0 **zatím ponechat**. Nelze doložit žádnou číselnou změnu, která sama obnoví reálné měření. Gain se nemá zvětšovat, aby z šumu vznikl očekávaný odběr. BATT_VOLT_MULT12.125470 je proti board defaultu11.1 vyšší o9,24%; může jít o dřívější kalibraci, nikoliv prokázanou chybu.

Pokud ESC schéma potvrdí analogový proudový výstup, nejmenší oprava může být obnova správného vedení CUR/zem a až potom kalibrace podle výrobce + referenčního měřidla. Při chybějícím senzoru použít vhodný externí power monitor; případně přejít na pouze napěťový monitor typ3 po samostatném schválení, aby systém nepředstíral měření kapacity. Typ3 proud nezměří. Typ9 jen při prokazatelně dostupných validních ESC proudových a napěťových datech. Žádný nový firmware k tomu zatím není zdůvodněn.

## 11. Jedna nejmenší další kontrola a potřebná měření

**Nejbližší jediná kontrola: při odpojené LiPo i USB identifikovat přesný model/revizi ESC a podle jeho vlastního pinoutu zkontrolovat existenci a skutečné vedení CUR z ESC do FC a zem.** Obsluha dodá označení a čitelný konektor/pady, případně ověří kontinuitu odpojeného vedení. Nehádat podle obecného obrázku; nepřipojovat napětí ani nepřepojovat neidentifikované piny. Tento audit novou fyzickou operaci neprovedl.

Teprve po této kontrole navrhnout jeden samostatně schválený LiPo test bez vrtulí, DISARMED, žádné motory: vhodným voltmetrem porovnat napětí packu a MP; bezpečně dostupný CUR vůči GND měřit v režimu napětí podle schématu a povoleného rozsahu. Nezasahovat sondou na drobné MCU piny při napájení. Pokud output CUR nenulový a ADC dále0, lokalizovat vedení/revizi; pokud CUR0, řešit senzor/napájení/proudovou cestu. Neprovádět náhodné přemapování ADC ani injekci napětí.

Kalibrace napětí při nulovém VLT_OFFSET: `nový VOLT_MULT = starý VOLT_MULT * V_multimetr / V_AP`; používat současně odebrané hodnoty a až po ověření stability a bezpečného rozsahu. Pro proud je nutný referenční proud i výstupní napětí senzoru, kalibrační zero a nenulový bod. Případný test zátěže pouze jako další samostatný návrh s vhodným sériovým wattmetrem/proudovou sondou a samostatně schválenou zátěží; ne motory jen kvůli senzoru. **Běžný multimetr v proudovém režimu nikdy paralelně přes LiPo.**

## 12. Napětí a posouzení4S/6S

16,55–16,71V je slučitelné s téměř nabitou4S, ale není nezávislý důkaz počtu článků nebo kalibrace. Není multimetrové porovnání. Z kódu a VOLT_MULT12.12547 vychází pro16,6V PC0 přibližně1,369V; pro25,2V přibližně2,078V. To se vejde do nominálního ADC rozsahu, **neprokazuje však skutečný dělič, toleranci a4S/6S napěťovou odolnost celé sestavy**. Chybný multiplier přímo poměrně zkreslí napětí; default11.1 se nemá obnovovat bez reference.

Předchozí preference je prvnílet4S, později6S; přesný pack/chemie/kapacita ani rating ESC dosud chybí. Následující je pouze podmíněný návrh pro běžnou4,2V/článek LiPo, ne definitivní nastavení pro neznámou baterii:

| Parametr | Nyní | Podmíněný4S návrh | Samostatný6S profil |
|---|---:|---:|---:|
| MOT_BAT_VOLT_MAX |25.2|16.8|25.2|
| MOT_BAT_VOLT_MIN |19.8|13.2|19.8|
| BATT_ARM_VOLT |14.0|14.0 ponechat k posouzení|21.0 ekvivalent stejného per-cell prahu|
| BATT_LOW_VOLT |13.6|13.6 ponechat k posouzení|20.4 ekvivalent|
| BATT_CRT_VOLT |13.2|13.2 ponechat k posouzení|19.8 ekvivalent|

Přesné kandidátní změny4S: `MOT_BAT_VOLT_MAX:25.2→16.8`, `MOT_BAT_VOLT_MIN:19.8→13.2`. Technický důvod jsou dokumentované doporučené násobky4.2/3.3 v [AP_MotorsMulticopter.cpp:63](/home/jirka/ardupilot_utb/libraries/AP_Motors/AP_MotorsMulticopter.cpp:63). Efekt je změna kompenzace tahu, **nikoliv oprava BAT.Curr ani detekce článků**. Ověření: přesný4S pack, rating sestavy, kalibrované napětí a následná samostatná kvalifikace; riziko je změna kompenzace pro dosud neověřený pack. Nic aplikováno nebylo. Současný mix4S alarmů a6S motorové kompenzace je nutné před FLIGHT-1 uzavřít.

Prahy14/13.6/13.2 odpovídají3.5/3.4/3.3V na článek4S, pro6S2.333/2.267/2.2V, a proto nejsou univerzální. Ani jejich prosté násobení není validací chování konkrétního packu pod zátěží. ARM_VOLT14V není doporučení startovat dlouhý test s téměř vybitou4S. Skutečně nabitou baterii a ukončení letu je nutné posoudit samostatně podle konkrétního packu a napěťového poklesu. Kapacitní prahy0 zachovávají neomezení kapacitou; spotřeba/RemPct nyní nejsou důvěryhodné.

## 13. Bateriové failsafe

Aktuálně `BATT_FS_LOW_ACT=0`, `BATT_FS_CRT_ACT=0`: v této verzi **Warn only**, nikoliv automatický LAND/RTL. [Metadata](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp:143). LOW_TIMER10s, FS_VOLTSRC0=raw. [Backend](/home/jirka/ardupilot_utb/libraries/AP_BattMonitor/AP_BattMonitor_Backend.cpp:151) vyhodnocuje také kritické napětí s tímto timeoutem; krátký propad nemusí vyvolat reakci, trvalý nízký signál nevede při akci0 k přistání. Záznam plné baterie tento mechanismus neotestoval.

Napěťové hlídání nepotřebuje funkční proud, ale potřebuje ověřené napětí a správný profil článků. Kompenzovanému napětí/spotřebě nelze při chybném proudu důvěřovat. Jako kandidát pro budoucí krátký STABILIZE test lze posoudit `BATT_FS_LOW_ACT:0→1` a `BATT_FS_CRT_ACT:0→1` (LAND): nezávisí na návratu podle GPS, ale potřebuje zdravý postoj, vertikální odhad a správnou funkci AP LAND; není to prosté pokračování manuálního STABILIZE. [Copter::failsafe_battery_event](/home/jirka/ardupilot_utb/ArduCopter/events.cpp:100) a [LAND akce](/home/jirka/ardupilot_utb/ArduCopter/events.cpp:495) používají původní AP infrastrukturu; na zemi může AP disarmovat. Riziko je změna režimu při falešně nízkém napětí, proto bez kalibrace a samostatného schválení nezapisovat. RTL/SmartRTL automaticky nenavrhovat při dosud neověřené navigační kvalifikaci. Timeout nelze určit libovolně jen podle tohoto testu; ponechat jako neověřenou položku k packu/propadu a požadované reakci.

## 14. Matice ověření a rizika před prvním letem

| Kontrola | Zjištění | Důkaz | Stav |
|---|---|---|---|
| Měření napětí | Mění se16.55–16.71V, bez multimetru | BAT480; PC0 rovnice | PARTIAL |
| ADC voltage mapping |10→PC0/ADC1 | hwdef+generated+aktuální PARM | PASS software |
| Proudový senzor | Osazení a proudová cesta neznámé; užitečný signál neprokázán | BAT0–3mA při otáčení | NEOVĚŘENO hardware |
| ADC current mapping |11→PC1/ADC1, dostupný | hwdef+generated+aktuální PARM | PASS software |
| ESC current output | Nedoložen, RPM jej neprokazují | ESC proud/volt/temp vždy0; bez modelu | NEOVĚŘENO |
| Battery monitor backend |4=analog V+I, nikoliv ESC | živý PARAM_READ+PARM+source | PASS identifikace |
| Kalibrace | Gain59.5 board default, multiplier12.12547 bez reference | aktuální parametry | PARTIAL |
| Battery failsafe | Warn only;4S prahy, ne6S | aktuální parametry+source | NO-GO kvalifikace |

Žádný nový build ani unit/SITL test nebyl potřeba nebo spuštěn: změnila se jen analytická evidence a tento report. Zpracovány oba kompletní soubory dekodérem, BAT/ESC/POWR/RCOU/ARM/EV/MSG/PARM, bez BAD_DATA záznamů. Nepředstírá se tím kalibrační ani letový test. Komunikace aktuální FC27.2s stabilní; její vypnutí či fyzický test nenastal.

## 15. Rozhodnutí a přímé odpovědi

1. **Proč0A?** Analogový PC1 vstup v logu odpovídá0 až50µV a po násobení59.5 dává nulu až3mA. Display zaokrouhlí vše na0,00A. Proč chybí fyzický proudový signál dosud nelze vybrat.
2. **Má sestava funkční proudový senzor?** Funkčnost není prokázaná; osazení bez přesného ESC neznámé.
3. **ADC pin správně?**11 odpovídá PC1 této softwarové desky a buildu; fyzické připojení/revize neověřené.
4. **MONITOR správně?**4 je správný pro analog V+I, podmíněno skutečně osazeným/připojeným senzorem.
5. **Dělič/kalibrace?** Napěťový vstup a výpočet potvrzeny, skutečný poměr a přesnost bez multimetru ne.
6. **Oprava jen parametry?** Zatím žádná doložená. Náhodná kalibrace není řešení signálu u0V.
7. **Fyzický zásah?** Zatím nelze určit; nejprve identifikace ESC a kontrola CUR, bez přepojování naslepo.
8. **Vhodná bateriová konfigurace?** Preferovaná4S vyžaduje přesný pack a rating sestavy; uveden podmíněný4S návrh,6S oddělený profil, ne automatické rozpoznání.
9. **Failsafe připravený?** Neprokázáno; nyní jen varuje, napětí nekalibrované a smíšený profil4S/6S.
10. **Co teď před FLIGHT-1?** Dodat přesný model/revizi ESC a doložit CUR→FC vedení při odpojeném napájení. To je nejmenší další kontrola. Potom samostatně schválit pasivní LiPo/voltmetrový test, vybrat konkrétní4S pack a uzavřít napěťový/failsafe profil. Nový ARM/motorový test ani nový UTB benchmark k diagnóze nyní není zdůvodněn.

Celkem **PARTIAL diagnostika, NO-GO uzavření bateriové kvalifikace FLIGHT-1**. Není vyžadována nová architektura nebo úprava firmware. Stav FC po tomto kroku: DISARMED STABILIZE, USB-only, původní konfigurace zachovaná, COM5 uzavřený. Žádné instalace, commit ani push.

Strojové výsledky, raw RX/TX/telemetrie, aktuální parametry, CSV a PNG/SVG jsou v `/home/jirka/ardupilot_utb/tmp/utb-battery-diagnostic-20261026/`. Původní logy v `/home/jirka/ardupilot_utb/logs/` zůstaly nezměněné. Závěrečná integrita/Git rozdíly: `integrity-check.json` v této analytické složce.

Závěrečná kontrola: HEAD/větev beze změny, index prázdný, všech17 souborů zdrojového manifestu shodných, oba BIN SHA zachované, git diff --check prošel. Jediná nová viditelná Git položka je tento report; dodané BIN a analytické tmp jsou ignorované, nebyly vynuceně přidány. Původní14 modified a33 untracked souborů zachováno.
