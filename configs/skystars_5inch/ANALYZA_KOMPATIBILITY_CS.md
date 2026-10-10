# Kompatibilita s místní větví ArduPilot 4.7.1 UTB

Analýza používá zdroje HEAD `8dc66c8d18bfe591387333059c35888af5d08f5a`. Název známý ve zdrojích neznamená přítomnost ve všech sestaveních ani správnou hodnotu pro fyzický dron. Kategorie se překrývají.

## Jmenná a cílová kompatibilita

1190/1190 názvů je známých v místních zdrojích. Generátor ArduCopter metadata rozpoznal 1189; zbývající FLTMODE_GCSBLOCK je registrován přímo v `libraries/AP_Vehicle/AP_Vehicle.cpp` a používán v `ArduCopter/mode.cpp`. Není doložen žádný nepodporovaný název. Kompletní inventář je v `parameter_compatibility.json`. Dostupnost pod compile-time a runtime enable guards je neověřená do získání skutečného seznamu parametrů konkrétního FC. Hodnoty se nesmějí automaticky převést na defaulty ani chybějící názvy tiše ignorovat.

## Frame a motory

Zdroj: `libraries/AP_Motors/AP_Motors_Class.h`, `AP_MotorsMatrix.cpp` (quad BF_X/BF_X_REV, add_motor a normalise_rpy_factors), `AP_MotorsMatrix.h`; UTB `libraries/AP_UTB/AP_UTB_MotorMixer.*` a `AP_UTB.cpp`.

FRAME_CLASS=1, FRAME_TYPE=18 znamená Quad BF_X_REV, nikoli BF_X=12. SERVO1–4_FUNCTION=33/34/35/36 mapuje logické M1–M4 na výstupy 1–4. Zdrojové pořadí motorového testu je 2,1,3,4 a nesmí se zaměnit s číslem výstupu. Následující pozice jsou očekávání AP geometrie při standardní orientaci, nikoli fyzické ověření kabeláže:

| Logický motor / výstup | Úhel | Pozice | BF_X=12 | BF_X_REV=18 | Pořadí testu |
|---|---:|---|---|---|---:|
| M1 / 1 | 135° | zadní pravý | CW | CCW | 2 |
| M2 / 2 | 45° | přední pravý | CCW | CW | 1 |
| M3 / 3 | −135° | zadní levý | CCW | CW | 3 |
| M4 / 4 | −45° | přední levý | CW | CCW | 4 |

AP počítá roll=cos(úhel+90°), pitch=cos(úhel), CW yaw=−1 a CCW yaw=+1, poté normalizuje maximální absolutní faktor každé osy na 0.5. Pro m=T·1+G·[R,P,Y]ᵀ:

```text
G12 = 0.5 * [ -1 -1 -1 ]    G18 = 0.5 * [ -1 -1 +1 ]
            [ -1 +1 +1 ]                [ -1 +1 -1 ]
            [ +1 -1 +1 ]                [ +1 -1 -1 ]
            [ +1 +1 -1 ]                [ +1 +1 +1 ]
```

G18=G12·diag(1,1,−1); roll/pitch a pořadí jsou stejné, yaw je opačný. Pro obě matice GᵀG=I a Gᵀ1=0. Inverze momentu je Gᵀm, collective je sum(m)/4; yaw18=(m1−m2−m3+m4)/2. To popisuje normalizované faktory, nikoli celý AP desaturační allocator nebo fyzikální model tahu.

Současný UTB používá G12 a podporuje výhradně class1/type12. Při class1/type18 vyhodnotí FRAME_MISMATCH a neprovede platný controller/mixer výpočet; při nepřipraveném loggeru může dříve nastoupit EXPERIMENT_SUPPRESSED. Žádná změna guardu ani mixeru není provedena. Samotný FRAME_TYPE nepřepíná fyzické směry ESC.

Varianty dalšího postupu:

- A: zachovat 18 a až po samostatném schválení přidat samostatně matematicky a softwarově ověřenou SHADOW podporu BF_X_REV, včetně inverze, resetů a testů. Tato podpora zde není implementována ani schválena pro aktivní output.
- B: zachovat 18 a nynější UTB; připravit pouze A0/A/B. Doporučený současný postup. Zapnutí shadow na 18 může ověřit odmítnutí frame, nikoli výkon platného mixeru při 200/400 Hz.
- C: případná fyzická rekonfigurace až po ověření motorů, ESC směru a vrtulí. Nepřevádět funkční 18 na 12 pouhou editací parametru.

## Významné parametry a fyzické kontroly

| Oblast | Export | Výsledek / potřebná kontrola |
|---|---|---|
| Výstupy | MOT_PWM_TYPE=6, SERVO_BLH_BDMASK=15, POLES=14 | DShot600, bidirectional na 1–4; ověřit ESC podporu, RPM a počet pólů |
| ESC commands | SERVO_DSHOT_ESC=0, RVMASK=0, 3DMASK=0 | Typ None zakazuje příslušné DShot commands, nikoli běžný DShot motor output; nulová maska nepotvrzuje skutečné směry ESC |
| DShot rate | SERVO_DSHOT_RATE=0 | Lokální metadata: pevný 1 kHz pro nízké loop rates; není automaticky 400 Hz |
| Ostatní výstupy | SERVO9_FUNCTION=120 | NeoPixel LED; ověřit skutečné připojení |
| Tah | SPIN_ARM=.1, MIN=.15, MAX=.95, THST_EXPO=.49, HOVER=.28 | Hodnoty konkrétního pohonu, nepřenášet na jiný dron bez ověření; CURR_MAX=0 bez nastaveného motorového proudového limitu |
| RC mapping | RCMAP_ROLL/PITCH/THROTTLE/YAW=1/2/3/4 | RC2_REVERSED=1; ověřit směr skutečných pohybů |
| RC kalibrace | RC1–4_MIN=988, MAX=2011; TRIM=1500/1502/988/1498; DZ=20/20/30/20 | Ověřit konkrétní vysílač/přijímač, úplný rozsah a nulový plyn |
| Režimy | FLTMODE_CH=7; slots 1–6: STABILIZE/STABILIZE/STABILIZE/ALTHOLD/STABILIZE/ACRO | AltHold je také dosažitelný; předchozí zkušenost se STABILIZE/ACRO nedokládá ověření AltHold |
| Aux | RC5_OPTION=153, RC6=30, RC8=151, RC10=70 | ArmDisarm, LostCopterSound, Turtle, AltHold; fyzicky zkontrolovat všechny polohy přepínačů |
| Turtle | RC8_OPTION=151, SERVO_DSHOT_ESC=0 | `ArduCopter/mode_turtle.cpp`: enabled vyžaduje ESC type != None; nelze tvrdit funkční Turtle ani měnit ESC typ bez nové kontroly |
| Attitude | ATC_ANG_RLL/PIT/YAW_P=4.5; ACC_R/P_MAX=1100, Y_MAX=270 | Zrychlení dle místních metadata deg/s²; zachováno |
| Rate PID | RLL P/I/D=.081/.081/.0012; PIT=.084/.084/.0011; YAW=.18/.018/0; IMAX=.5; FF=0 | Jsou to AP gains, nelze je automaticky použít jako UTB gains |
| Rate filtry | R/P FLTD/FLTT=37.5 Hz, FLTE=0; yaw FLTD=0, FLTE=2, FLTT=37.5 | Původní regulace AP zachována |
| ACRO | RP_RATE=360, Y_RATE=202.5 deg/s; RP_EXPO=.3, Y_EXPO=0; RATE_TC=0 | Statické mapy odpovídají AP-derived UTB shadow reference, nikoli úplné shodě controllerů |
| Trainer | ACRO_TRAINER=2, BAL_ROLL/PITCH=1 | AP ACRO používá trainer; UTB shadow reference jej vynechává |
| INS | GYRO_FILTER=75, ACCEL_FILTER=10, HNTCH/HNTC2/HNTC3_ENABLE=0, FFT_ENABLE=0 | Harmonic notch vypnutý i při BIDIR; fyzicky ověřit vibrace a aliasing, bez automatického přeladění |
| IMU sampling | INS_FAST_SAMPLE=1, GYRO_RATE=1, INS_USE/USE2=1, EK3_IMU_MASK=3 | Fast sampling vybraný pro první IMU, požadavek 2 kHz závisí na driveru; druhá lane nesmí být považována za identickou |
| Kalibrace | INS device IDs, offsets/scales, gyro bias; AHRS_ORIENTATION=0 a trims | Patří konkrétním senzorům a montáži, ne obecně stejnému typu desky |
| EKF | AHRS_EKF_TYPE=3, EK3_ENABLE=1, PRIMARY=0; GPS pos/vel, baro height, yaw source=1 | Ověřit skutečné zdraví lanes a zdroj headingu při nepoužívaném kompasu |
| Kompas | COMPASS_ENABLE=1, EXTERNAL=1, USE/USE2/USE3=0, ORIENT=101 | Není použit pro navigaci; custom rotation a ID vyžadují kontrolu skutečného zařízení; nulové offsets nejsou důkaz kalibrace |
| GPS | GPS1_TYPE=1 Auto, GPS2_TYPE=0, AUTO_CONFIG=1, SAVE_CFG=2 | Ověřit skutečný modul a GPS/EKF stav; obnovení může také způsobit konfiguraci připojeného GPS |
| Logging | LOG_BACKEND_TYPE=4, LOG_DISARMED=0, LOG_BITMASK=180222 | Block backend, původní PM bit3 je zapnutý, PID bit12 a Fast Attitude bit0 vypnuté; fyzická kapacita a zápis neověřeny |
| Scheduler | SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0, DIV=1 | Kompatibilní s FSTRATE guardem; neodstraňuje FRAME mismatch |
| UTB | Žádný UTB_* v exportu | Export neurčuje stávající ani budoucí UTB gains/enable; nutný zvláštní readback |

## Cílový hwdef a periferie

`libraries/AP_HAL_ChibiOS/hwdef/SkystarsH7HD-bdshot/hwdef.dat` zahrnuje rodiče `SkystarsH7HD/hwdef.dat` a přepisuje piny/časovače. PWM1/2 jsou PB0/PB1 na TIM3, PWM3/4 PD12/PD13 na TIM4; BIDIR se vyhodnocuje i na úrovni timer skupin. Nelze odvozovat dostupnost pouze z markeru jednotlivého pinu. Deska má SPI dataflash W25Q128, takže LOG_BACKEND_TYPE=4 je relevantní block logger, nikoli souborový backend=1.

Sériové pořadí cíle: SERIAL0 OTG1 USB, 1 USART1 RX, 2 USART2, 3 USART3 ESC telemetry, 4 UART4 GPS, 5 UART5 VTX, 6 EMPTY, 7 UART7, 8 UART8. Export: SERIAL0_PROTOCOL=2 MAVLink2, SERIAL1_PROTOCOL=23 RCIN, SERIAL4_PROTOCOL=5 GPS; SERIAL2/3/5/7/8_PROTOCOL=−1. SERIAL1_BAUD=115, SERIAL4_BAUD=230 jsou zkrácené baud enumerace (115200/230400), RC driver může provést autodetekci. SERIAL3 vypnutý neznamená vypnutou bidirectional DShot RPM telemetrii. SERIAL6 EMPTY není použitelný fyzický port ani při defaultu DJI v hwdef.

Analogový OSD AT7456E odpovídá OSD_TYPE=1. Dual BMI270 a barometr musí být porovnány s identitou skutečného FC. Analogová baterie BATT_MONITOR=4, VOLT_PIN=10, CURR_PIN=11 odpovídá ADC hwdef; VOLT_MULT=12.12547 a AMP_PERVLT=59.5 vyžadují nezávislé měření. Relay GPIO81/82 a výchozí ON mohou řídit VTX power/video select; ověřit zapojení a chlazení bez přepínání při této analýze. Board default FRAME_TYPE=12 platí pro nové úložiště, nikoli pro uložený FRAME_TYPE=18.

## Bezpečnostní nálezy

BATT_FS_LOW_ACT=0 a BATT_FS_CRT_ACT=0 neurčují žádnou bateriovou failsafe akci. LOW=13.6 V, CRT=13.2 V, ARM_VOLT=14 V, LOW_TIMER=10 s a FS_VOLTSRC=0 raw voltage jsou zachovány; samotné překročení prahů zde není schválenou ochranou RTL/Land. MAH prahy jsou 0, CAPACITY=3300 mAh není ověřená fyzická kapacita.

13.6/13.2/14 V odpovídají aritmeticky 4×3.4/3.3/3.5 V; MOT_BAT_VOLT_MIN/MAX=19.8/25.2 odpovídají 6×3.3/4.2 V. Jde o nesoulad předpokladů, nikoli důkaz 4S nebo 6S akumulátoru. Při 6S by LOW/CRT byly přibližně 2.267/2.2 V na článek; při 4S je i 16.8 V pod nastaveným MOT_BAT_VOLT_MIN. Motorová kompenzace napětí není bateriový failsafe. Chybí skutečný počet článků, chemie, kapacita, limity packu při zatížení, kalibrace napětí/proudu a ověření reakce. Žádné hodnoty nebyly opraveny odhadem.

RC failsafe: FS_THR_ENABLE=1 RTL, FS_THR_VALUE=975, RC_FS_TIMEOUT=1 s; normální minimum plynu 988 ponechává pouze 13 µs rozdíl. Přijímač držící poslední platnou hodnotu nemusí propadnout pod 975; ověřit výpadek rámců/failsafe flag a skutečné chování protokolu. RC_PROTOCOLS=1 dovoluje autodetekci všech protokolů, RC_OPTIONS=32 vyžaduje nulový plyn při armingu a bit pro ignorování receiver failsafe není nastaven. FS_GCS_ENABLE=0; FS_OPTIONS=16 řeší pokračování pilotního řízení při GCS failsafe, není plošné vypnutí RC failsafe. RC_OVERRIDE_TIME=3 s je potřeba zahrnout do kontroly GCS ovládání.

ARMING_SKIPCHK=0 nic nepřeskakuje. ARMING_RUDDER=2 a RC5 ArmDisarm tvoří více cest arm/disarm; zkontrolovat na vysílači. BRD_SAFETY_DEFLT=0 znamená výchozí hardware safety vypnutou; BRD_SAFETY_MASK=16368 umožňuje pohyb výstupům 5–14 mimo safety, nikoli motorovým výstupům 1–4. Nejde o náhradu arming checks. FS_CRASH_CHECK=1, FS_EKF_ACTION=1 Land, FS_VIBE_ENABLE=1, FS_DR_ENABLE=2 RTL a FENCE_ENABLE=0 vyžadují posouzení se skutečným GPS/EKF; pro bench se nesmí vypínat kontroly kvůli dosažení stavu.

Kalibrace a uložené statistiky/formát jsou archiv zařízení. Nejsou důkazem kompatibility po výměně FC ani výsledkem nového testu. Obnova celého exportu může narazit na read-only či podmíněné položky: odmítnutí evidovat a vyšetřit, nikdy nenahrazovat automaticky.

Odborná dokumentace k obecnému chování: [battery failsafe](https://ardupilot.org/copter/docs/failsafe-battery.html), [radio failsafe](https://ardupilot.org/copter/docs/radio-failsafe.html), [napěťová kompenzace](https://ardupilot.org/copter/docs/current-limiting-and-voltage-scaling.html). Význam enumerací a konkrétní guards určuje výše uvedený místní source, nikoli odlišná verze wiki.
