# Budoucí obnova na Skystars a kontrola před letem

Tento postup popisuje budoucí manuální session. Nyní neproběhl upload, spojení s FC ani motorový test. Git checkout nemění žádné parametry desky.

1. Odmontovat všechny vrtule, zajistit dron, nearmovat. Identifikovat fyzický FC, revizi, zapojení, konkrétní ESC a pack; zajistit bezpečné napájení a chlazení VTX. Nepoužívat profil na MicoAir ani jiném kusu FC bez samostatného posouzení.
2. Před jakýmkoli zápisem uložit úplný aktuální export z FC, verzi firmware, board ID/target, seznam dostupných parametrů, vlastní UTB hodnoty a případné mise/geofence. Zálohu opatřit datem a SHA-256. Ověřit možnost návratu k této konkrétní záloze, nikoli jen k Git větvi.
3. Zkontrolovat vybraný firmware: target SkystarsH7HD-bdshot, commit, SHA-256 a compile-time AP_UTB/AP_UTB_BENCH flags. Identifikace varianty nesmí vycházet pouze z názvu arducopter.apj. Nahrání firmware vyžaduje samostatnou autorizaci; tento postup je upload neprovádí.
4. Ověřit hash referenčního souboru podle README a vytvořit porovnání aktuálního exportu versus všech 1190 archivních hodnot. Označit rozdíly v frame, motorových výstupech, ESC, baterii, RC, kalibraci, EKF a failsafe. Chybějící/read-only parametry řešit jednotlivě. Před přenosem kalibrací ověřit stejné fyzické senzory a jejich device IDs. Archiv nikdy neupravovat.
5. Rozhodnout, které konkrétní rozdíly mají být obnoveny, až po fyzických kontrolách níže. Neprovést hromadné automatické načtení exportu. Baterii ani motorový mapping nepřevádět podle odhadu; nespouštět reset všech parametrů. Podmíněně nedostupné parametry nejprve vysvětlit pomocí cílového sestavení. Zápisy dělat manuálně v GCS do předem zkontrolovaného pracovního profilu, zaznamenat každé odmítnutí.
6. Před experimentem zvlášť přečíst a zkontrolovat UTB parametry, protože archiv je neobsahuje. Zachovat neaktivní UTB a zákazy UTB_ACRO. A0/A: UTB_ENABLE=0, UTB_SHADOW=0; B: ENABLE=1, SHADOW=0. Nesmí se měnit FRAME_TYPE kvůli experimentu.
7. Po všech povolených zápisech provést restart DISARMED a nový úplný readback. Porovnat s odsouhlaseným pracovním profilem, nikoli jen s potvrzením GCS zápisu. Výslovně ověřit FRAME_CLASS=1, FRAME_TYPE=18, SERVO1–4=33–36, ESC/BIDIR, battery/failsafe, RC, EKF a UTB stav. Při rozporu nepokračovat.
8. Teprve při samostatně schválené bench session aplikovat příslušný minimální overlay, restartovat a znovu přečíst LOG_DISARMED, UTB_ENABLE/SHADOW. A0/A/B provádět bez vrtulí DISARMED. C/D nejsou platným shadow benchmarkem tohoto frame. Uchovat logy, záznam varianty firmware, konfigurace a teploty/napájení.
9. Při návratu obnovit jen ověřené rozdíly ze zálohy, včetně LOG_DISARMED a dřívějších UTB hodnot; restart a porovnání celého readbacku zopakovat. Přepnutí Git větve návrat FC neprovádí.

## Co fyzicky ověřit před jakýmkoli dalším letovým testem

- Očíslování všech čtyř výstupů, pozice M1–M4, směry otáčení a vrtule proti BF_X_REV. Skutečný motorový test až jako samostatně povolená činnost bez vrtulí; při této přípravě se neprovádí.
- Verzi a podporu ESC pro DShot600/BIDIR, RPM validitu, počet pólů a skutečné nastavení směru ESC. RVMASK=0 není měřením směru.
- Počet článků a chemii konkrétního packu, kapacitu, napěťové/proudové limity, měření externím přístrojem, ADC kalibraci a konzistentní battery/motor compensation/failsafe politiku. Současný rozpor musí být vyřešen před letem, bez hádání.
- RC rozsahy/směry/deadzones, nulový plyn, kanál 7 a všechny aux přepínače, yaw arm/disarm, skutečný link-loss failsafe včetně RTL požadavků na GPS/home. Kontrolovat DISARMED bez vrtulí, nepoužívat force arm.
- Správnou orientaci desky, akcelerometrů, trim, gyro klid, obě IMU a EKF lanes, vibrace/filtry, heading se zakázaným použitím kompasu, GPS/home/baro health. Statické parametry samy tuto kontrolu nenahrazují.
- Periferie a UART wiring, OSD, VTX/relay chování a chlazení, flash logger funkci, kapacitu, úplnost logů a hardware scheduler/CPU měření A0/A/B.
- Arming checks, hardware safety, failsafe chování a záznam předletové konfigurace po restartu. Nové shadow ani aktivní UTB řízení tímto dokumentem není schváleno.

Otevřené otázky: skutečný pack a kalibrace; fyzický BF_X_REV layout a ESC směr/póly; receiver/protokol/link-loss; dostupnost parametrů konkrétního firmware; heading/EKF s nepoužívaným kompasem; hardware load/block logger výsledky; případné samostatné schválení shadow podpory18. Bez jejich vyřešení nelze referenční archiv označit za nově ověřenou letovou konfiguraci.
