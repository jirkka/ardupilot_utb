# UTB packed: hardware session 11. 10. 2026

Firmware byl sestaven a nahrán na SkystarsH7HD-bdshot přes COM5. Všechny pokusy
proběhly DISARMED bez vrtulí; nebylo vydáno armování ani motorový test.

**Výsledek:** 200 Hz ověřeno v 20s stabilním okně včetně platného controlleru
a mixeru. Při požadavku 400 Hz je z flash doložen zápis 400 kompletních sad/s
v dodatečně vybraném 5s úseku, ale všechny s neplatným controllerem/mixerem.
Plánovaný 20s zdravý D test byl zastaven kvůli nezdravému EKF; není PASS.
Aktivní UTB_ACRO, fyzický motorový output a FÁZE 2 zůstávají zakázané.

## Firmware a reprodukce

- Větev: `test/skystars-5inch`.
- Commit zdrojů: `34990b72985ec97e06dc0db9fa6657e901c120b4`.
- APJ/FC identifikace: `34990b72`; board_id 1075, bootloader protocol 5.
- APJ: 1127691 B, rozbalený image 1249992 B.
- SHA-256 APJ: `352c82d61fc0b32b35544bb0dff0a9850d4e4c1bdb211e90d596aa749e72848e`.
- Artefakt: `tmp/utb-packed-hw-20261011_165438/SkystarsH7HD-bdshot-34990b7298-UTB_ON-BENCH_ON-PACKED_V1/arducopter.apj`.
- Konfigurace i build PASS; uploader ověřil board ID a image CRC, bez force a bez
  operací s externím firmware image. Následný AUTOPILOT_VERSION odpovídá APJ.
- BENCH_ON prokázán hwdef, ELF symboly, parametrem UTB_B_WAIT_US a skutečnými
  UBMT/UBHI/UBST v BIN; UTBC ID31/Length242 dekódován z fyzické flash.

```sh
cd /home/jirka/ardupilot_utb
export PATH="$PWD/.venv/bin:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board SkystarsH7HD-bdshot --enable-UTB --enable-UTB_BENCH \
  --extra-hwdef "$PWD/tmp/utb-packed-hw-20261011_165438/bench-on.hwdef"
./waf copter -j4
```

`bench-on.hwdef` obsahuje `define AP_UTB_BENCH_ENABLED 1`. Použitý archivovaný APJ
má výše uvedený hash; poslední soubor v build adresáři neslouží jako identifikace.

V této session se kód neměnil. Hash každého C++/header/Python souboru packed
patche souhlasí s dříve otestovaným zdrojovým manifestem. Znovu spuštěné zachované
unit binárky: UTB 42/42 a BENCH 12/12 PASS; Python decoder unit 6/6 PASS.
SITL/compile-guard matice je předchozí ověření stejného kódu, tentokrát nebyla
znovu spuštěna. Nevydává se za hardware ověření. Nic nebylo doinstalováno.

## Záloha a parametry

Před uploadem archivováno všech 1209 původních parametrů a celý dosavadní log:
`before-upload-log-1.bin`, 16266496 B, SHA-256
`cefd45cbc49e067132189cb689a7ca834c0c8c75ebd7ead1acf8820ba5196f8f`.
Zaplněná flash byla po samostatném povolení vymazána; potvrzeno `Chip erase complete`
a prázdný LOG_ENTRY. Původní soubor ani parametrová záloha nebyly přepsány.

Společný profil: UTB_ENABLE=1, UTB_SHADOW=1, UTB_LOG_FMT=1,
UTB_LOG_RATE=200/400, LOG_DISARMED=1, LOG_BACKEND_TYPE=4 (BLOCK/W25Q128),
LOG_BITMASK=180222, LOG_FILE_BUFSIZE=172 (původní BLOCK clamp 64 KiB),
LOG_BLK_RATEMAX=0, LOG_DARM_RATEMAX=0, FRAME_CLASS=1/FRAME_TYPE=18,
SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0. Parametry byly potvrzeny readbackem.
Původní gains, IMAX a D_HZ zůstaly nulové; test tedy neověřuje chování PID
s nenulovými gains ani jeho nejhorší možnou režii.

Po uploadu byly doplněny nové UTB_LOG_FMT a provozní hodnoty při restartu
(gyro offsets/caltemp, barometr, automatická deklinace a STAT). Ostatní původní
parametry souhlasí se zálohou. Tyto automatické kalibrace nebyly zpětně přepsány.

## Měřené výsledky a rozsah platnosti

| Veličina | C: 200 Hz | D: 400 Hz, zastavený pokus |
|---|---:|---:|
| Vyhodnocené TimeUS okno | [117000000,137000000) | [243000000,248000000), dodatečný 5s úsek |
| Kompletní uložené sady | 4000 | 2000 |
| Platné controller+mixer sady | 4000 | 0 |
| Uložená frekvence přes celé uvedené okno | 200,00 Hz | 400,00 Hz, pouze transport |
| Partial sets | 0 | 0 |
| UTBS.Calc / Flg | 0 / 1023 | 2 / 643 |
| UTBR.Val / UTBM.Val | 1 / 1 | 0 / 0 |
| DSF.Dp | 0, delta 0 | 0 v 5s úseku |
| PM.NLon v uložených PM | 0, 0 | 0 |
| PM.MaxT v uložených PM | 2561 / 2554 µs | 2566 µs |
| PM.Load | 21,2 % / 21,2 % | 21,0 %, výpočet je neplatný |
| Stav plánovaného testu | PASS pro stabilní 20s okno | STOP; 20s okno nebylo dokončeno |

C intervaly sousedních Seq jsou přesně 2, D transport má 2000 kompletů a žádné
partial groups. Validita a úplnost jsou oddělené; samotný počet paketů D neprokazuje
zdravý 400Hz controller. C tok je 70479,65 B/s, z toho ostatní AP logy 20043,45 B/s;
D DSF vykazuje přibližně 119 kB/s. RAM buffer minimum C 50784 B, D 41299 B.

C benchmark metriky jsou rozdíly kumulativních N/Sum mezi exporty
116936017 a 136963622 µs (20,027605 s), nikoli přesně okno packetů:

- Main loop průměr 2500,0011 µs, průměr absolutního jitteru 2,3756 µs.
- Main acquire průměr 0,5841 µs, kumulativní maximum 44 µs.
- Worker hold průměr 4,1729 µs, kumulativní maximum 221 µs.
- Producer průměr 38,0588 µs, kumulativní maximum 60 µs.
- Consumer průměr 11,4278 µs, kumulativní maximum 1224 µs (wall time).
- Export průměr 692,05 µs, kumulativní maximum 1358 µs (wall time).
- Stack free minimum 1784 B při skutečné oblasti 3504 B; požadavek thread_create
  3072 B. Jde o watermark, nikoli potvrzení dostatečné rezervy pro každý scénář.
- Worker/exporter life RUNNING; create failure=0; queue HWM=4.
- Delta queue drops/backend missing/obsolete discard=0 ve stabilním intervalu.
- Potvrzené main/worker kolize=0, UNKNOWN attribution delta=10015.
  Nula confirmed collisions neprokazuje nulové blokování.
- UBST[8] delta=293 (přibližně 14,63/s): jednotlivé task-budget overruns.
  UBST[9] delta=2314: periody nad 2500 µs; není to čítač ztracených cyklů.

Výpočetní režii formátu oproti původnímu firmware nelze z této session izolovat:
chybí současné A0/A a zdravé D, C/D navíc mají rozdílnou matematickou validitu.
Zápis jednoho packed packetu a všechny uvedené mutex/stack hodnoty byly reálně
naměřeny; nejde o důkaz dlouhodobé bezpečnosti nebo běhu s aktivními motory.

## Zastavení, startup a stahování

1. První pokus C po softwarovém rebootu skončil v config_error:
   `Baro: unable to initialise driver`. AP main loop ani benchmark exporter
   se nespustily; zachovaný tlog není rychlostní test. Po úplném odpojení napájení
   je BMP280 a RC zdravý; software reboot nebyl znovu zkoušen.
2. Při spuštění logování před C oknem bylo zaznamenáno queue Drop=57,
   backend Miss=40 a period maximum 6016 µs; PM v 114825264 µs má NLon=1.
   Stabilní C výsledek proto nesmí být rozšířen na celý start session.
3. Po ukončení C a vypnutí SHADOW/logování, během následného MAVLink downloadu,
   export maxima periody vzrostl v 141971 ms na 42236 µs a v 143973 ms na 42515 µs.
   V tlogu se před D objevily `Need Alt Estimate`, `EKF attitude is bad`
   a `AHRS: EKF3 core 0 unhealthy`. Nejde o naměřené maximum 400Hz stabilního okna.
4. D při warmupu hlásilo STATE_INVALID, a proto bylo zastaveno. IMU GH/AH=1,
   BARO.H=1 a AP capture Cap=1 v zachovaném D úseku nezaručují AHRS healthy.
   XKF4 má FS=0, TS=50, SS=32768 na obou cores; samotné tyto agregované hodnoty
   nerozliší všechny interní podmínky EKF health. Explicitní prearm hlášení a
   UTBS se shodují na nezdravém EKF. Příčina jeho přechodu zůstává neprokázaná.
5. Po startu D je kumulativně Drop=151 a Miss=40; nejde o nový bezeztrátový celý
   experiment. V dodatečném 5s úseku jsou Dr/Ms konstantní a DSF.Dp=0.

Source potvrzuje guard v [AP_UTB_State.cpp](../libraries/AP_UTB/AP_UTB_State.cpp#L43),
AHRS přenáší EKF3 healthy v [AP_AHRS.cpp](../libraries/AP_AHRS/AP_AHRS.cpp#L2328).
Interní EKF health má více podmínek v
[AP_NavEKF3_Outputs.cpp](../libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp#L9).
Chyba nulového baro backendu je v [AP_Baro.cpp](../libraries/AP_Baro/AP_Baro.cpp#L753).
BLOCK log download používá synchronní get_log_data/ReadBlock a původní semaphore
v [AP_Logger_Block.cpp](../libraries/AP_Logger/AP_Logger_Block.cpp#L461).
Časová souvislost downloadu a EKF chyby je evidence pro další audit, nikoli důkaz
konkrétní příčiny. Žádný guard, EKF ani backend nebyl upraven nebo obejit.

## Konečný stav a další postup

FC ponecháno DISARMED, firmware 34990b72, UTB_ENABLE=1, UTB_SHADOW=0,
LOG_DISARMED=0, UTB_LOG_FMT=1, UTB_LOG_RATE=400. Nový parametr formátu je výslovný;
staré SHADOW/logging hodnoty nejsou po STOP automaticky obnoveny. Full readback
je uložen. Arming/motor izolace a nezměněné safety sources proti ba793af1 byly
znovu ověřeny; instrumentace/packed zdroje odpovídají předchozí unit/SITL validaci.

Doporučený další experiment po vyřešení/auditu health a odsouhlasení: samostatný
zdravý cold boot pro D, profil nastavit před měřením, vyčkat inicializace a stabilního
stavu, nelogovat download mezi warmupem a měřením; stáhnout BIN až po konci experimentu.
Neměnit AHRS/EKF/gyro/age guardy. Pokud se unhealthy stav nebo dlouhé periody opakují,
test zastavit a analyzovat je odděleně. Nové automatické pokusy už nebyly spuštěny.

Stále **hardware neověřeno**: 20s/long-duration 400 Hz se zdravým state/PID/mixerem,
nenulové gains, opakovaný software reboot, startup ztráty, příčina EKF unhealthy,
vliv full flash/erase/temperature, nejhorší mutex/stack zátěž a jakýkoli letový output.

## Evidence

Adresář `tmp/utb-packed-hw-20261011_165438/` obsahuje APJ/ELF/hwdef, build logy,
unit výsledky, parametry before/final, upload CRC log, erase potvrzení, tlogy,
raw BIN a strojové analýzy. Významné raw soubory:

| Soubor | B | SHA-256 |
|---|---:|---|
| PACKED_C200.bin | 2068246 | `72ce9b52f85ef1e9557e31fefb8a3798553aa3acefa231cca91644fbbef88080` |
| D400-stopped-log-3.bin | 2955746 | `ca3969ff09a644ce1cef179a00b8a0b18a953fa84859cda1270085bb69b673dd` |

`PACKED_C200-analysis.json` a `D400-stopped-analysis.json` oddělují matematickou
validitu, kompletnost, frekvenci a velikost zkoumaného okna. `C200-telemetry-timeline.json`
zachovává změny maxima periody a prearm hlášení kolem downloadu. C raw soubor je
neměnný snapshot před D; pozdější log-3 navíc obsahuje D, jeho okno je vybráno TimeUS.

## Navazující revize plné frekvence

Historické výsledky a STOP výše zůstávají zachované. Nový firmware acf04efa
a oddělení měření od downloadu ověřily 4 000/8 000 platných sad v předem
určených 20s oknech 200/400 Hz. Příčiny, source patch, všechny výsledky a
zbývající startup/download omezení: [navazující report](UTB_400_RECOVERY_20261011_CS.md).
