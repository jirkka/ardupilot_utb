# FÁZE 3B – pasivní porovnání AP a UTB na H743

Stav k **20. 10. 2026**: diagnostika implementovaná; unit/SITL a cílové buildy
prošly v níže uvedeném rozsahu. **Skutečný H743 DISARMED benchmark dokončen:**
2000/2000 kompletních a validních sad, maximální rozdíl i RMSE AP/UTB 0 ve všech
čtyřech profilech. Během benchmarku PM.NLon=0 a DSF.Dp=0; předchozí baseline
obsahuje jeden neobjasněný dlouhý cyklus 5503 µs. Nejde o kvalifikaci aktivního
400Hz řízení ani letu. Aktivní UTB řízení na H743 zůstává compile-out.
Výchozí3A HEAD `3949c920831c2faab8f13420c12b3a254423e326`, příprava3B commit
`070c189cdfa8357307ee1aebe8f3677cd849d520`, větev `test/skystars-5inch`.

Evidence: `tmp/utb-3b-20261019_194419/`. Zdrojový manifest a checkpoint
obsahují také nové untracked soubory, nejen tracked Git diff.

## Skutečný read-only preflight COM5

Přímý přístup přes Windows Python/pyserial; žádné předpokládané `/dev/ttyS4`.
Byly využité existující Python moduly z WSL `.venv`, nic se neinstalovalo.

- USB VID/PID `1209:5741`, serial `200024001251303433333336`.
  MAVLink UID2 odpovídá stejným dvanácti bajtům; system/component `1/1`.
- FC hlásí Copter **4.7.1**, custom version `f9cc3bcc`, board ID **1075**.
  Lokální commit je `f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f`
  (`AP_UTB: add disarmed internal benchmark references`).
- ID 1075 odpovídá rodině SkystarsH7HD; target `SkystarsH7HD-bdshot` určuje
  zadání a jeho hwdef. Samotná MAVLink identita nerozlišuje všechny varianty
  stejného board ID. Binární SHA aktuálně nahraného firmware nebylo z FC čtené.
- Po celý pozorovaný přenos DISARMED/STABILIZE. EKF flags 167, gyro/accel/AHRS
  a logging sensor bits healthy. Nezdravý enabled bit: souhrnný pre-arm check;
  jeho konkrétní příčina nebyla určena ani obcházená.
- SYS_STATUS load snapshot **18,0 %**; nejde o řízené porovnání CPU zatížení.
  Monitor baterie ukazoval 3 mV, bez LiPo to není měření napájené pohonné soustavy.
- Všech **1211/1211** parametrů uloženo do `parameters-before.param` a JSON.
  SHA-256 PARAM backup: `5255b15ce824f9e39318adcf8430a15dbcba1c68d203ea08fd4e0a0d38601614`.
- `FRAME_CLASS=1`, `FRAME_TYPE=18`, `FSTRATE_ENABLE=0`, `UTB_ENABLE=1`,
  `UTB_SHADOW=0`, `UTB_LOG_RATE=400`, `LOG_BACKEND_TYPE=4`, `LOG_DISARMED=0`.
  `FS_GCS_ENABLE=0`, `FS_THR_ENABLE=1`; nic z toho nebylo změněné.
- `UT3_*` parametry chybí a identifikovaný firmware předchází 3A. Současná FC
  tedy neposkytuje nové porovnání 3A. Logging backend 4 je lokální BLOCK flash,
  nikoliv File/SD backend 1. Healthy bit neprokazuje volnou kapacitu nebo nový zápis.

Přenos trval 36,28 s. Odeslané byly jen čtecí REQUEST_MESSAGE a PARAM_REQUEST_LIST.
Bez heartbeat injection, PARAM_SET, LOG_REQUEST_LIST/DATA, rebootu, erase,
ARM/force ARM, motorových nebo PWM/DShot příkazů. Port je uzavřený.

## Implementovaná diagnostika a izolace

Samostatný compile-time flag `AP_UTB_NATIVE_BENCH_ENABLED`, default 0,
Waf `--enable-UTB_NATIVE_BENCH`. Při OFF jsou diagnostický modul, task a jeho matematické třídy odstraněné.
Obecný konstruktorový opt-out zůstává součástí AP API; OFF image je o 24 B
větší než historický H743 build 3A, bez změny BSS. Není tvrzená bitová shoda firmware.
`AP_UTB_MATH_AVAILABLE` umožní kompilovat skutečné `UTB_PID`, `UTB_Rate`,
`UTB_Attitude`, stavový a output adaptér. **Neumožňuje pilotní autoritu**:
`AP_UTB_RATE_SITL_AVAILABLE` má nezměněnou podmínku platformy SITL.
UTB_Core/Acro/Angle se do H743 nepřidávají.

`AP_UTB_NativeBench` drží dva oddělené controller states: původní
`AC_AttitudeControl_Multi`/`AC_PID` a skutečný `UTB_Rate`/`UTB_PID`/`UTB_Attitude`.
Oba mají vlastní RAM motor sink. Sink není registrovaný jako AP_Motors singleton,
má nulový motor mask a inertní init/output/test virtuální metody. Nikdy se
nevolá jeho output/spool/mixer/HAL. Setters jen ukládají čísla do jeho RAM.
Benchmark nepřijímá od Copteru motorový objekt a nemůže tak omylem navázat živé motory.
AHRS view je sdílený pouze pro čtení; INS ani EKF se nespouštějí podruhé.

Nutný zásah do původního AP: konstruktory `AP_Motors`,
`AP_MotorsMulticopter`, `AC_AttitudeControl`, `AC_AttitudeControl_Multi`
přijímají `register_singleton=true`. Dosavadní volání mají stejnou výchozí
registraci. Pouze diagnostické instance předají false. Inicializace kontroluje
zachování globálních pointerů. Unit canary ověřuje také zachování nenulového
živého I a motorové hodnoty. Motorová logika, PID matematika, HAL, DShot,
arming, failsafe a letové mode dispatch se nemění.

Další změny 3A výpočtových souborů: compile guard pro diagnostickou matematiku
 a `inline` u nepoužívaného vektorového helperu (H743 má -Werror=unused-function).
Žádná změna vzorce nebo PID tuning.

## Průběh experimentu

Jediný nový parametr **`UB3_RUN`**, root key 108, group index 0. Default0,
po bootu je vždy 0. Hodnota 1 spustí jednu sekvenci 2000 kroků, potom se sama vrátí 0;
žádný zápis do EEPROM. Původní indexy parametrů se neposouvají.

- Běžná scheduler úloha **50 Hz**, priorita 255 až po původních úlohách;
  žádný FAST_TASK, žádná druhá invokace živého AP controlleru.
- Rozpočet 300 us je scheduler rezervace pro návrh testu, nikoliv naměřená
  nebo bezpečnostně kvalifikovaná mez H743. Překročení je potřeba vyhodnotit z dat.
- Při armování nebo používání samostatného rate threadu se RUN vrátí 0.
- Nutný otevřený logger, povolené DISARMED zápisy, WritesEnabled a zdravý backend;
  log download/disabled policy/failed logger test ukončí. Po obnově se sám nespustí.
- Session je jednou zkonstruovaný v nulovaném statickém úložišti při bootu.
  V matematickém kroku není heap, blokování nebo manipulace se živým PID.
- Export používá původní AP_Logger streaming API a jeho zámky. **Není tvrzený
  jako neblokující**. Běží jen během DISARMED diagnostiky v nízké prioritě,
  mimo stopky jednotlivých matematických výpočtů. APus/UTus jsou wall time
  včetně případných přerušení/preempce, nikoliv izolovaný počet CPU cyklů.
  Konfigurace/reset a zachycení výsledků leží mimo tyto dvě stopky. Jeho vlastní latence je měřená zvlášť.
  První použití formátů může alokovat metadata v loggeru; není to regulátorová fast path.

| Kroky | Porovnávaný výpočet | Měření a dt |
|---|---|---|
| 0–499 | přímý skutečný PID všech os | deterministický sin/cos gyro fixture; model dt 2,5 ms |
| 500–999 | rate-only shaping + kompletní Rate | stejné syntetické gyro, reference a dt 2,5 ms |
| 1000–1499 | ANGLE shaping/quaternion attitude + Rate | stejné AHRS view, fixture gyro a dt 2,5 ms |
| 1500–1999 | Attitude + Rate se skutečným gyro/AHRS | společný aktuální gyro vzorek a skutečný Wall dt; první krok 2,5 ms |

Každý profil obsahuje nulovou referenci, kladný krok, záporný krok,
I reset na lokálním kroku 200, umělé motor limit flags 250–349 a filter reset 400.
Při ANGLE je generator roll/pitch reference převedený na úhel faktorem 0,1;
yaw zůstává rad/s. CSV `command` zachovává skutečnou předanou hodnotu.
To není uzavřená soustava, model dronu, skutečná saturace motorů ani fyzický let.

Identická pevná testovací konfigurace všech tří os: P/I/D=.135/.135/.0036,
FF=.07, D_FF=.002, IMAX=.5, PDMX=.2, FLTT/FLTE/FLTD=15/7/20 Hz.
Ostatní attitude/shaping defaulty odpovídají konstruktorům AP/3A.
Nejde o změnu letových gains nebo kopírování stávajících ATC hodnot z EEPROM.
**SMAX=0**: časově závislý slew limiter čte skryté AP_HAL::millis, takže jeho
aktivní omezování tento párový fixture nekvalifikuje. Je to výslovné omezení testu.

## Data a vyhodnocení

Protokol **Ver 2** používá dynamicky přidělené logger formáty, bez nových pevných
log IDs: `U3BD`, `U3BF`, `U3BA`, `U3BQ`, `U3BT`. Formáty mají nejvýše 15 položek,
labels pod 64 znaků a unikátní jména. BIN FMT a částečné sady kontroluje decoder.

- BD: TimeUS, Seq(N), profil, osa, dt, generator reference, společné gyro,
  AP/UTB PID target a P/I/D.
- BF: raw FF, raw D_FF, jejich součet, PID output, skutečně předaný FF do RAM
  sinku včetně yaw scalaru a součet PID+applied FF. U přímého PID profilu je
  applied FF definovaný jako raw součet; motorový Rate v tomto profilu neběží.
- BA/BQ: body quaternion, původní požadované body rates a obě target quaternions.
- BT: čas výpočtu AP/UTB, perioda diagnostické úlohy Wall, předchozí export LogUs,
  flags a verze protokolu. První Wall 0 není skutečná perioda; poslední export
  už nemá následující záznam. Časovač je AP_HAL micros/micros64 (na H743 monotónní HW).
- Flags: bit0 synthetic limits, bit1 I reset, bit2 filter reset, bit3 změněný
  asynchronní vstup, bit4 nevyhovující State health. Health/logging nejsou PID error.
  Input/health problém označí profil v offline analýze za neprůkazný do dalšího resetu.

`utb_native_bench_analysis.py` přijímá jen existující místní BIN soubor,
neotevírá zařízení/síť. Vytváří `comparison.csv` a `summary.json`: max absolute
error, RMSE, počet nestejných hodnot i unikátních osových vzorků/sad, neplatnosti,
chybějící sekvence, částečné sady, periody/jitter, math/export latence a původní PM/DSF.
DSF drops patří všemu logování; nejsou automaticky připsané nové diagnostice.
Víc RUN experimentů v jednom souboru decoder odmítne (je nutné oddělit záznamy).
Nezavádí se libovolný numerický ani letový PASS threshold; výsledky jsou vypsané.

## Build a ověření

Příkazy jsou také v README_BUILD_WSL_CS.md. Pro cílový firmware:

```sh
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board SkystarsH7HD-bdshot --enable-UTB \
  --disable-UTB_RATE_SITL --enable-UTB_NATIVE_BENCH --enable-UTB_BENCH
./waf copter -j6
```

Žádný upload příkaz zatím nebyl proveden. Použije se aplikační APJ přes stávající bootloader;
`arducopter_with_bl.hex` není určený pro tento postup. Identita desky se před
uploadem musí znovu shodovat s výše uvedeným USB serial a board ID 1075.

### Skutečně dokončené ověření před uploadem

- Unit: 11 C++ programů; 137 běžných PASS, dva původně přeskočené vector případy
  spuštěné samostatně a PASS (celkem 139 různých případů). Offline math 30 946 vektorů,
  29 scénářů bez chyby; syntetická reference 16 000 vektorů v odvozené toleranci.
  Nový benchmark case ověřuje 2000 kroků, singletony, živý I/motor canary a armed abort.
- Python: **164 testů PASS**, z toho8 nových pro CSV/error statistiky,
  neplatné/vynechané sady, phase taint, duplicity, více RUN a odmítnutí COM endpointu.
- Finální protokol 2 DISARMED SITL: **2000 kompletních sad**, nula partial/missing,
  nula invalid/tainted a nula rozdílných sad. Všechny porovnané PID/FF/applied FF,
  celkové zásahy, rates a quaternion komponenty mají **max abs=0, RMSE=0**.
  Naměřené složky jsou nenulové; nejde o porovnání samých nul.
- Uložená frekvence 50,000206 Hz, diagnostická perioda průměr 19 999,917 us,
  min 19 992 us, max 20 825 us; průměr absolutního jitteru 15,765 us vůči 20 ms.
  **SITL virtuální hodiny vracejí vnitřní math/export časy 0; není to HW výkon.**
- Logging suppression SITL: vypnutí LOG_DISARMED po zahájení vrátí RUN0;
  obnovení logování nezpůsobí automatický restart. Žádné armování.
- Build SITL ON, H743 native-benchmark ON a OFF PASS. Finální H743 image 1 266 756 B,
  BENCH OFF 1 251 932 B; přírůstek 14 824 B flash a 3544 B statické RAM. BSS 132 336 B,
  free application flash 437 180 B. Dynamická logger metadata nejsou tímto BSS měřením zahrnutá.
- Finální ELF audit: původní Copter::motors_output a native rate controller přítomné;
  UTB_Core/Acro/Angle nepřítomné v obou H743 variantách. Nové matematické třídy
  a diagnostika jsou přítomné pouze v ON. V OFF úplně chybějí.
- Parametrická a logger metadata generace PASS; UB3_RUN má default 0 a jméno pod 16 znaků.
  Zdrojové formáty a skutečné BIN FMT mají unikátní jména/IDs a platné délky.
  Flake8, astyle nových souborů a whitespace kontrola jsou součástí finálního auditu.

Připravený diagnostický APJ: `tmp/utb-3b-20261019_194419/firmware/SkystarsH7HD-bdshot_UTB3B_DISARMED_3949c920_123da51058e8.apj`,
**1 142 643 B**, SHA-256:
`123da51058e88b8f61ebf2e9bd3ff5bb424180a8eb4f4505fdb3ac8764a2d259`.
Board ID 1075, MCU STM32H743xx, target SkystarsH7HD-bdshot.
Built z HEAD 3949c920 s necommitnutými 3B změnami; přesná vazba na source je
v `source-manifest.json` a `source-checkpoint.tar.gz`. Binárka je archivovaná
před další konfigurací waf. Poslední obecný build adresář nyní může obsahovat OFF variantu.

Při přípravě19.10. byly hardware numerika, časování a start neověřené.
Aktuální20.10. měření numeriky, wall timing a log completeness je uvedené níže;
nejde o úplnou mutex/stack nebo propustnostní kvalifikaci. Full 400Hz streaming
nové matematiky, aktivní slew limiting, notch fixture, reset při poruše reálných
senzorů a aktivní let jsou také **NEOVĚŘENO**. Logging-disabled compile varianta
nebyla v 3B sestavená; v kódu je compile guard, který v takovém buildu RUN zastaví.
Nejde o skryté PASS pro neprovedené scénáře.

Přípravné chyby (uchované v evidenci): AP_Float vyžaduje set(), přesná float
rovnost vyžaduje bitové porovnání, relativní output cesta selhala před startem
SITL a ARM ELF se jmenuje arducopter bez přípony. H743 unused-helper -Werror byl
odstraněný inline deklarací. Žádná z těchto neúspěšných revizí není vybraná k uploadu.


## První HW session po schválení uploadu

1. Stále bez LiPo a vrtulí, DISARMED. Znovu ověřit jedinou správnou COM5/UID/board1075.
2. Ověřit SHA přesně zvoleného APJ a souhlas před uploadem; zachovat bootloader,
   parametry a log flash. Nemaže se nic. Binární záloha starého firmware z FC není k dispozici.
3. Po startu nejprve jen identita/health/parametry. UB3_RUN=0, režimy 42/43 nedostupné,
   aktuální FRAME 1/18 a failsafe hodnoty nezměněné. Při nezdravém startu zastavit,
   neobcházet kontroly nebo automaticky opakovat reboot.
4. Záloha parametrů už existuje; před testem porovnat stav a kapacitu loggeru.
   LOG_DISARMED dočasně 1, ponechat backend 4 a legacy SHADOW 0. Neprovádět
   download souběžně se záznamem. Bez místa test zastavit, bez souhlasu nic nemazat.
5. Uložit baseline s RUN 0 a původní PM/BENCH statistikou, potom jeden RUN 1
   (přibližně 40 s při 50 Hz), nakonec opět RUN 0. Žádné motorové příkazy.
6. Při armovaném heartbeat, ztrátě komunikace, logger error, neočekávaném InternalError,
   neplatných/nonidentických vstupech či zhoršení původních scheduler ukazatelů test
   ukončit a vyhodnotit. RUN 0 je zastavení diagnostiky, nikoliv motorový stop příkaz.
   Nové libovolné letové meze nebo failsafe pravidlo nebyly přidané.
7. Obnovit LOG_DISARMED na původní 0 a RUN 0; ověřit parametry read-only.
   Teprve potom odděleně a řízeně stáhnout log podle dosavadního download audit postupu.
   Neprovádět neomezené LOG_REQUEST_DATA ani log erase; čtení může ovlivnit EKF/časování.
8. Vyhodnotit CSV a odchylky po profilech; oddělit numeriku od stavu/konfigurace
   a logger losses. CPU hodnoty H743 porovnat s RUN 0 téhož nového firmware,
   nikoliv přičíst celý rozdíl proti starému f9cc jen samotným regulátorům.

Bez LiPo nelze ověřit napájené ESC, PWM/DShot fyzickou odezvu, tah,
bateriovou kompenzaci, skutečné motorové limit flags ani letovou dynamiku.
Pasivní 50Hz výpočet nekvalifikuje aktivní 400Hz řízení nebo let. Žádné takové
scénáře nejsou označené PASS. Aktivní UTB na H743, ARM a hardware motorové testy
zůstávají mimo tuto fázi.

## Stav při lokálním checkpointu

Upload výše identifikované APJ je schválený. Kontrola před uploadem ale
neobjevila COM5 ani USB zařízení 1209:5741; Windows evidoval pouze Bluetooth
COM6/COM7. Kontrola skončila před otevřením FC, rebootem nebo zápisem.
SHA-256 APJ zůstal shodný. Doklady jsou v `upload/result.json`
a `upload-console.txt` v adresáři evidence. Firmware ani parametry FC
nebyly změněné. Po opětovném připojení je nutné znovu ověřit identitu
a DISARMED stav, potom navázat schváleným uploadem a pasivním měřením.
Tento commit neoznačuje hardware část 3B jako dokončenou.

## Skutečný H743 výsledek 20. 10. 2026

Evidence: `tmp/utb-3b-measure-20261020_143658/REPORT_CS.md`, detailní CSV/JSON
v `analysis/`, grafy `ap-utb-curves.svg`, `differences.svg`, `timing.svg`,
`timing-distribution.svg`. Jde výhradně o skutečný FC BIN.

Identita1075/200024001251303433333336/COM5, FW4.7.1/3949c920. Před benchmarkem
byly přesně tři původní logy smazány jedním schváleným MAVLink LOG_ERASE;
Chip erase complete a prázdný seznam ověřeny. Žádný další firmware upload,
MCU erase, přepis bootloaderu nebo změna regulátorů.

| Profil | Kompletní / invalidní sady | Max chyba / RMSE | AP mean / max µs | UTB mean / max µs |
| --- | --- | --- | --- | --- |
| PID, syntetické gyro/reference | 500 / 0 | 0 / 0 | 11,152 / 105 | 10,774 / 50 |
| Rate-only shaping + Rate | 500 / 0 | 0 / 0 | 26,894 / 141 | 25,900 / 121 |
| Attitude+Rate, fixture gyro + skutečný AHRS | 500 / 0 | 0 / 0 | 54,196 / 162 | 51,352 / 160 |
| Attitude+Rate, skutečné gyro/AHRS | 500 / 0 | 0 / 0 | 53,080 / 159 | 51,380 / 158 |

P/I/D, FF/D_FF, PID/total výstupy, rates a quaternion komponenty shodné
v uložených hodnotách;0 missing/partial/NaN/Inf/changed-input/tainted sad.
Wall časy zahrnují preempci, bez setup/reset/capture a bez exportu; AP běží
v páru první, takže nelze z drobného rozdílu průměrů odvodit rychlostní výhodu.
Výpočetní percentily a všechny jednotlivé odchylky jsou v CSV/JSON.

- Dosažená frekvence49,999577 Hz; průměr periody20 000,169 µs,
  min19 127, max20 876, průměr absolutního jitteru311,872 µs. Jde o50Hz task,
  nikoliv400Hz main loop. První3 profily mají model dt2,5ms, poslední wall dt.
- Logger export předchozí sady: průměr101,098 µs, max325 µs; poslední export
  není pozorovaný. Skutečný průměr se čte z `hardware-details.json`.
- SYS_STATUS load: logged baseline18,355 %, benchmark18,854 %,
  po benchmarku18,036 %. Rozdíl přibližně+0,499 procentního bodu; jeden běh.
- PM během benchmarku:4 záznamy, NLon0, MaxT nejvýše2551 µs, InE/ErC0;
  po testu NLon0. Před testem baseline NLon1/MaxT5503 µs, příčina neprokázaná.
  Cumulative since-boot main maximum31 020 µs není maximum benchmarku.
- UBMT delta main period2500,000064 µs; delta mean absolute jitter2,608 µs.
  UBST8 delta477/39,056 s je task budget overrun, nikoliv ztracený main cyklus.
- DSF.Dp0 v celém BIN,0 ERR. UBST17 export-skip delta0 během měřených oken;
  starý kumulativní stav1105 není ztráta nového experimentu.

BIN2 516 746 B, SHA-256
`8f719b8e75a3b9e752a06221c21fb31abdbdded3c6a99c7fa76b0528bf031275`.
Omezený download: nejvýše900 B/request,50ms mezery,0 retries,295 s,
2797 requests, úplná velikost a navazující offsety. Po downloadu read-only
health kontrola prošla: DISARMED/STABILIZE, EKF167, senzory/logger healthy,
UB3_RUN0, LOG_DISARMED0, SHADOW0, FSTRATE0. Port uzavřený, nový log zachován.

Numerická shoda fixture na H743 prokázána. Jeden DISARMED50Hz běh bez ztrát
prošel; dlouhodobá stabilita, další konfigurace, SMAX/notch, skutečné motorové
limity a let nejsou kvalifikované. Před pasivním porovnáváním za letu objasnit
baseline5503µs událost, ověřit opakovatelnost a plný capture/export overhead.
Současný benchmark je DISARMED-only a není live in-flight srovnávač; další
návrh/schválení musí zachovat výhradní původní AP motorovou autoritu.
