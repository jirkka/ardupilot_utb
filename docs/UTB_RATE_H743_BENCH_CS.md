# FÁZE 2B: skutečný H743 a rozsah ověření

Stav: **H743 VERIFIED – pouze read-only IDLE preflight**. Nové gains a alokační
anti-windup jsou **SITL ONLY**. Nové SHADOW matematické měření na FC:
**NOT EXECUTED**. Aktivní UTB closed loop na fyzickém dronu: **NOT EXECUTED**.

## Schválený a provedený postup

Dne 14. 10. 2026 byly aktuálně potvrzené sundané vrtule, bezpečně připojená
DISARMED FC a volný COM5. Samostatně schválený plán
`tmp/utb-2b-20261014_164442/hardware-preparation/idle-preflight.json`
obsahoval 30 s baseline a 10 s recovery, maximum 60 s. Canonical SHA-256:
`ba3693a25249d9057d0eb69e0ff665f366b88776ae80f320fd3ac2eeb50e2418`.

Existující `Tools/autotest/utb_download_experiment.py` dokončil session za
40,938 s, výsledek `success=true`. Použil heartbeat, PARAM_REQUEST_READ a
REQUEST_MESSAGE pro verzi, EKF a SYS_STATUS. Nebyl vyslán LOG_REQUEST_LIST,
LOG_REQUEST_DATA ani LOG_REQUEST_END; nebyl PARAM_SET, ARM, RC_OVERRIDE,
motorový příkaz, reboot, erase nebo upload. Port byl uzavřen před SITL běhy.

Důkazy: `hardware-idle/summary.json`, `timeline.jsonl`, wire/RX/TX záznamy a
`read-only-analysis.json` ve výše uvedeném novém adresáři. Původní logy a
parametry nebyly změněné. Session není log-download root-cause experiment.

## Identita a konečný pozorovaný stav

- Board ID 1075, USB vendor 4617, product 22337.
- UID2: `20 00 24 00 12 51 30 34 33 33 33 36 00 00 00 00 00 00` (hex).
- AUTOPILOT_VERSION: flight_sw_version 67568127, source prefix `f9cc3bcc`.
  Osmibajtové pole potvrzuje pouze prefix, nikoli úplný SHA repozitáře.
  Dříve archivovaný firmware má SHA `f9cc3bcc0f13e3b19bcf9b7b04b5151cd57f767f`.
- HEARTBEAT na konci: base_mode 81, custom_mode 0, system_status 3; DISARMED.
- EKF_STATUS_REPORT flags 167, guard session bez health faultu; není to přímé
  měření všech interních AHRS validity predikátů.
- STAT_BOOTCNT 77 na začátku i konci. Žádný pozorovaný restart.
- SYS_STATUS.load poslední 178 = 17,8 %. Není to PM.Load.

Readback začátku: UTB_SHADOW=0, UTB_B_SYNTH=0, LOG_DISARMED=0,
FRAME_CLASS=1, FRAME_TYPE=18, FSTRATE_ENABLE=0, LOG_BACKEND_TYPE=4.
UTB_ENABLE nebyl samostatně vyčten: enable potvrzuje stavový flag a UB00=2
(varianta B). Provedeno nula parameter writes, proto žádný restore nebyl nutný;
nejde o úplnou zálohu nebo nový readback všech parametrů.

## Timing, logger a stack

Všechny UB klíče byly přijaty 41krát. Kumulativní UB29 loop period max zůstal
2566 µs. Z pouhého nezměněného maxima nelze spočítat jitter session ani
rozlišení celého rozdělení periody. PM.NLon nebyl v této telemetrii dostupný:
UNKNOWN, nikoli nula.

UB08 task-budget overruns delta 711, UB09 periods nad nominal delta 5409.
Obě veličiny mají vlastní softwarovou sémantiku; nejsou počtem ztracených
400Hz cyklů. UB27 main acquire max a UB28 worker hold max jsou nula při
vypnutém DISARMED logování. Nejde o důkaz nulového blocking rizika při RUN.

UB10 lifecycle=3 RUNNING, worker loops delta 36339, begin/done timestamp se
aktualizoval. Stack requested 3072 B, ChibiOS allocated area 3504 B,
kumulativní free watermark 2432 B, bez změny během session. Jde o dosavadní
watermark IDLE, nikoli důkaz rezervy pod novou zátěží. Create attempts=1,
failures=0. UB30 producer max 15 µs je dosavadní maximum při SHADOW=0;
nejde o čas nově naladěného PID. Pipeline requested/enqueued/written/drops
zůstaly nula, skutečná frekvence SHADOW sad nebyla měřena.

UB17 export_skips delta 40 při UB36 LoggingAvailable=0: exporter nemohl
zapsat kompletní diagnostiku do vypnutého loggeru. Všechny UB telemetry klíče
přitom dorazily; tento čítač není sám o sobě počet ztracených MAVLink sad.
UB32 exporter max 1427 µs je kumulativní maximum samostatného exporteru,
nikoli main acquire nebo PID execution time. Viz `ArduCopter/utb_bench.cpp`.

DataFlash volná kapacita: **UNKNOWN**. Bez LIST/DATA nebyla nově ověřená;
starý odhad 1,66 MB nebyl použit jako současná kapacita. Žádný nový full-rate
experiment ani download neproběhl. Dřívější download/EKF závada zůstává
UNVERIFIED, zdravý IDLE její příčinu nevysvětluje.

## Proč není potřebný nový hardware PID experiment

Sdílený `AP_UTB_RateController.cpp`, D filtr a directional integrace nebyly
změněné. Nová alokační capture/feedback integrace, parametr UTB_S_AW a mode41
jsou omezené `AP_UTB_RATE_SITL_AVAILABLE`, včetně HAL_BOARD_SITL podmínky.
Fyzická FC tyto změny nemůže vykonávat. H743 kontrola proto sestává z
cross-build/ELF/parameter compile-out důkazů v závěrečném reportu.

Nové gains nebyly přeneseny do H743 parametrů. Jejich stabilita je vlastnost
konkrétního SITL modelu, nikoli kalibrace 5″ dronu. Pro fyzické SHADOW měření
nové konfigurace by byl nutný samostatný konkrétní plán, kapacita loggeru a
souhlas se zápisy. Aktivní fyzický output tím není povolen.
