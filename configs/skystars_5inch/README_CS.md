# Skystars 5″ – referenční konfigurace

Tato složka patří pouze do větve `test/skystars-5inch`. Obsahuje export konkrétního již létaného 5″ dronu Skystars H7 Dual Gyro. Původní provoz ve STABILIZE a ACRO je informace o předchozím dronu, nikoli ověření nového firmware nebo UTB. Profil není univerzální default a není určen pro MicoAir H743 V2 45A AIO AM32.

## Reprodukovatelnost

- Výchozí větev: `ardupilot-4.7.1-utb`, čistý strom.
- Výchozí HEAD: `8dc66c8d18bfe591387333059c35888af5d08f5a`.
- [Původní export](default_config_skystar_funkcni_let.param): 1190 jedinečných parametrů, 20966 bytů.
- SHA-256: `7f39b20c60bc1622a8bb68432408b7f540642e186e762ae6f795db047b71f421`.
- Soubor je kopií původního exportu bez změny jediného bytu. Lokální `.gitattributes` pro tento soubor vypíná normalizaci konců řádků; původní bajty se zachovávají také v Git blobu. Rizikové hodnoty zůstávají zachované.
- [Analýza](ANALYZA_KOMPATIBILITY_CS.md), [obnova a fyzická kontrola](OBNOVA_KONFIGURACE_CS.md), [inventář všech parametrů](parameter_compatibility.json).

## Stav ověření

Implementováno: archiv a analýza, obecná SHADOW podpora Quad BF_X=12/BF_X_REV=18 a minimální DISARMED overlay A0/A/B/C/D. Firmware motorový mapping a původní AP chování nejsou změněny. Přesná unit/SITL/build evidence a omezení jsou v [reportu rozšíření](../../docs/UTB_BFX_REV_REPORT_CS.md).

Hardware neověřeno: skutečný layout, ESC, konkrétní pack/kalibrace, cílový readback, timing/stack/mutex/logger throughput. Dron používá střídavě4S a6S LiPo; žádné battery/failsafe hodnoty nejsou změněny.

## Benchmark overlay

| Varianta | Compile-time firmware | Overlay | Co lze porovnávat |
|---|---|---|---|
| A0 | AP_UTB_ENABLED=1, AP_UTB_BENCH_ENABLED=0 | `bench_A0_A.param` | Původní PM/scheduler, bez nových instrumentovaných metrik |
| A | AP_UTB_ENABLED=1, AP_UTB_BENCH_ENABLED=1 | `bench_A0_A.param` | Overhead instrumentace při UTB_ENABLE=0 |
| B | AP_UTB_ENABLED=1, AP_UTB_BENCH_ENABLED=1 | `bench_B.param` | Zapnutý UTB lifecycle, SHADOW=0 |
| C | AP_UTB_ENABLED=1, AP_UTB_BENCH_ENABLED=1 | `bench_C.param` | Shadow BF_X_REV, požadováno200Hz |
| D | AP_UTB_ENABLED=1, AP_UTB_BENCH_ENABLED=1 | `bench_D.param` | Krátký shadow BF_X_REV experiment, požadováno400Hz |

A0/A/B overlay mění pouze LOG_DISARMED, UTB_ENABLE a UTB_SHADOW. C/D přidávají pouze UTB_LOG_RATE; nedoplňují gains a nezavádějí tune. Vyžadují restart a následný readback. Předem musí platit FRAME_CLASS=1, FRAME_TYPE=18, LOG_BACKEND_TYPE=4, SCHED_LOOP_RATE=400 a FSTRATE_ENABLE=0. Overlay tato nastavení sám nevynucuje. Původní LOG_DISARMED=0 zůstává v archivu.

Měřit nejprve DISARMED, bez vrtulí, stejnou dobu, napájení, teplotu, logger stav a konfiguraci periferií. Zachovat stejné LOG_DISARMED=1 také pro A0, aby rozdíl A0/A neměřil změnu logging policy. Na block loggeru ověřit volnou kapacitu a skutečně uložené záznamy. Zastavit při resetu, ztrátě komunikace, nečekané změně režimu/arming stavu, pohybu motorů, anomálním napájení nebo přehřívání. Žádný číselný bezpečnostní limit ani hardware výsledek zde není nově schválen.

Použít build postup a compile-time přepínače z `docs/UTB_HARDWARE_BENCH_CS.md`, board vždy `SkystarsH7HD-bdshot`; zaznamenat SHA-256 konkrétního firmware a oba přepínače. Parametrický overlay nemění compile-time variantu firmware. Generické profily, které nastavují FRAME_TYPE=12, se pro tento dron nesmí použít. Firmware bez AP_UTB nemá UTB parametry a tyto overlay pro něj nejsou určeny.

Git větev nepřepisuje perzistentní parametry na FC. Původní export neobsahuje UTB_*: jeho pozdější obnova nevymaže dříve uložené UTB hodnoty. Žádný firmware ani parametr nebyl nahrán a žádný motorový test nebyl proveden.

Automatická kontrola všech overlay allowlistů a původního SHA: `.venv/bin/python Tools/autotest/test_utb_profiles.py` z kořene repozitáře. Nové build/SITL výsledky nenahrazují schválení hardware session.
