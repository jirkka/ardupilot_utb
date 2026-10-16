# FÁZE 2C: vlastní stabilizace náklonu pouze v SITL

Implementovaný algoritmus: `AP_UTB_SITLAttitude`, experimentální volba
`UTB_S_ATT=1` v existujícím mode41. Výchozí `S_ATT=0` zachovává RATE-only
větev. `S_OWNER=0` nadále odmítá vstup. Výsledky a rozsah ověření uvádí
[validační report](UTB_ATTITUDE_SITL_VALIDATION_CS.md).

## Konvence a vlastní rovnice

Body FRD: X dopředu, Y doprava, Z dolů; navigation NED. Kladný roll
naklání pravou stranu dolů, pitch zvedá příď, yaw otáčí příď doprava.
Quaternion ze stávajícího AHRS view převádí body do NED. Lokální
`Quaternion::from_euler` používá `Rz(yaw) Ry(pitch) Rx(roll)`; jeho operátor
na vektoru odpovídá této matici. Jde o AP source convention, nikoli novou
definici estimatoru.

Vlastní engineering volba je redukovaná chyba rotace osy tahu bez heading
integrátoru. Pro požadované Euler náklony `r*,p*` a aktuální heading `y`:

```text
Rd = Rz(y) Ry(p*) Rx(r*)
v = R(current)^T Rd [0,0,1]^T
s = hypot(v.x,v.y)
theta = atan2(s,v.z)
e = [-v.y,v.x,0] * theta/s        (limit theta/s -> 1 pro s -> 0)
w_xy = [Kroll*e.x, Kpitch*e.y]
w_xy *= min(1, rate_max / norm(w_xy))
w.z = requested_body_yaw_rate
```

To je logaritmus nejkratší rotace mezi směry body Z, vyjádřený v aktuálním
body frame. Není to prostý rozdíl Euler úhlů. Kolem jedné osy a pro malé
náklony dává přesně očekávané znaménko a velikost chyby. Současné změny
headingu vlevo násobí obě rotace stejným `Rz`, které se v součinu ruší.
Quaternion `q` i `-q` dávají totožný výsledek. Euler yaw se používá pouze
pro orientaci požadovaného náklonu; žádná absolutní yaw chyba se nereguluje.

Vnější smyčka je P-only, bez I/D, bez dalšího filtru, feedforwardu nebo
AP rate shapingu. Rate PID z FÁZE2B zůstává jedinou vnitřní instancí.
Nulový yaw body target není přesný heading hold: při současných náklonech
se Euler heading může měnit kinematickým propojením. Body yaw target
není obecně Euler yaw derivative.

Rozsah experimentu: norm úhlové reference nejvýše5°, skutečné Euler
roll/pitch nejvýše35°, RP rate norm limit0,05–0,3 rad/s a body yaw
target nejvýše0,1 rad/s. Antipodální osy/singularita pitch±90° proto
nejsou podporovanou oblastí. Neplatný stav se odmítá před výpočtem.
Omezení nejsou bezpečnostní limity fyzického stroje. Vnitřní původní
target norm≤0,6 / gyro norm≤2 rad/s a další guards zůstávají účinné.

## Scheduling a výhradní autorita

INS update → mode41 rate slot: state capture → vlastní attitude P →
existující UTB Rate PID → původní jediný motor output → capture/log →
AHRS update → update_flight_mode/reference pro další cyklus.

Obě vlastní smyčky běží400Hz. AHRS orientace je z předchozího AHRS
update, gyro je latest z aktuálního INS; nejde o tvrzení jejich dokonalé
fyzické simultánnosti. UAAT/UARX/UAST/UAMT sdílejí TimeUS a Seq.
Srovnávací AP outer je spouštěný ve stejném fast slotu, aby nevznikla další
celá perioda rozdílu mezi vlastníky. AP core ani pořadí scheduleru se nemění.

| S_ATT | S_OWNER | Vnější / vnitřní zdroj |
|---|---|---|
|0|1 nebo2|Původní konečný RATE experiment se společným AP rate shapingem|
|1|1|Vlastní redukovaná rotace P / skutečný UTB Rate PID|
|1|2|Odmítnuto; taková hybridní větev není implementovaná|
|2|1|Výslovně označený AP attitude comparison / stejný UTB Rate PID|
|2|2|AP attitude comparison / původní AP Rate PID|

Pouze S_ATT2 volá `input_euler_angle_roll_pitch_euler_rate_yaw_rad`.
AP outer zachovává vlastní shaping, acceleration/sqrt politiku a heading
management. Není to matematicky stejný regulátor ani celý STABILIZE mode.
Při limitu RP targetu se pro owner1 použije společný norm clamp; owner2
se zastaví, protože jeho PID čte interní AP target, který nelze tímto
read-only adaptérem přepsat. Takový limit nelze skrytě vykázat jako shodu.
Přesné rozdíly srovnání jsou ve validačním reportu.

## Parametry a lifecycle

Nové indexy29–35, pouze pod skutečným SITL guardem:
S_ATT (default0), S_ANG (°), S_ARP/S_APP (s⁻¹), S_RMAX(rad/s),
S_YRATE(rad/s), S_TOFF(normalized). Staré indexy a defaulty se nemění.
Všechny nové hodnoty jsou frozen při entry. Změna v aktivním kontextu
vyvolá CONFIG latch; samotné navrácení parametru jej neodstraní.

Entry/exit resetují attitude context/IMU historii a čítače; vlastní vnější
P nemá integrátor ani filtr. Vnitřní arming/spool re-prime zůstává původní.
Neplatná quaternion norm/NaN/Inf, AHRS/INS health, stáří>10ms, repeated
INS/změna primary gyro, dt mimo0,625–10ms nebo neplatná reference znamenají
attitude fault a přes původní inner latch nulový RPY/throttle/SHUT_DOWN.
Nová fault čísla10 ATTITUDE a11 VERTICAL doplňují původní enum, nemění jej.
Žádný automatický AP fallback v okně. LAND je výslovný host cleanup potom.

Thrust allocation, spool, throttle mapa a default SATURATION stop S_AW0
zůstávají původní. S_AW1 využívá pouze již ověřený SITL allocation feedback.
Současný AP manual throttle vychází z posledního GUIDED pracovního bodu,
bez angle boost/altitude/position řízení. S_TOFF je malý konstantní testovací
přírůstek; nulový default nic nemění. Výška a vertikální rychlost jsou
jen read-only AHRS diagnostika a stop guard.

Výšková obálka je ±2m od entry, |vz|≤0,5m/s. Při přípravě v10m jde o
omezený20% drift a zachování velké vzdálenosti od země; malé combined3°
náklony bez kompenzace mají ideální ztrátu vertikálního zrychlení asi
g*(1-cos3°cos3°)=0,027m/s². Šest sekund plateau proto vysvětluje řád
0,16m/s, nikoli libovolný drift. Tyto engineering bounds jsou podmínky
experimentu, žádný důkaz hardware bezpečnosti. Všechna překročení se
uchovají jako neúspěšné běhy.

## Diagnostika, build a rozšiřování

UAAT ID28 má15 polí (úhly/chyba v rad, rates v rad/s), UAST ID29 má7 polí
(oddělené outer invocation counts, výška/vz/throttle). Vznikají jen pro
S_ATT≠0 po původním outputu; žádné dodatečné spuštění regulátoru kvůli logu.
UAAT má58B a UAST35B včetně3B hlaviček: navíc37200B/s při400Hz,
celkem503B/cyklus =201200B/s pro10 mode41 paketů před dalšími AP logy.
Původní mode41 logy/allocator capture se nemění. Fixed structs, bez heapu,
mutexů nebo nového čekání ve vlastní matematice. DataFlash backend zápisy
jsou původní SITL diagnostický mechanismus; jejich host režie není nula.

Vše používá `AP_UTB_RATE_SITL_AVAILABLE`, tedy UTB enabled AND explicitní
active flag AND skutečný HAL_BOARD_SITL. Na H743 musí být helper, parametry,
logy a režim compile-out i při forced active request. Build důkazy jsou
v novém archivu, oddělené od2A/2B. COM5 se v této fázi nepoužívá.

Samostatný helper lze později vyměnit nebo rozšířit při zachování body-rate
kontraktu, jednotek, validity a limitů. Heading hold/I/D/feedforward,
vlastní altitude nebo fyzická autorita nejsou implementovanou součástí2C.
Nejdříve musí vzniknout jejich samostatná specifikace a testy.
