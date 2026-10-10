# Ověření archivu a analýzy

- Původní export versus archiv i Git blob: byte-for-byte PASS; 1190 unikátních jmen, všechny hodnoty konečné; SHA-256 a velikost dle README.
- Jmenná kompatibilita: 1189 metadata + 1 přímá registrace ve zdroji; 1190/1190 známých názvů. Dostupnost na konkrétním FC neověřena.
- Overlay allowlist: PASS, pouze LOG_DISARMED/UTB_ENABLE/UTB_SHADOW; SHADOW vždy 0.
- G12/G18: nezávislý výpočet geometrie, GᵀG=I, Gᵀ1=0, shodné R/P a opačný yaw PASS.
- Spuštěn existující `build/sitl/tests/test_utb`: 31/31 PASS, včetně explicitního odmítnutí class1/type18 a FrameFastRateStaleAndModeResetRecovery. Použit existující unit binární artefakt; neproběhl nový rebuild ani SITL integrační test. Tento výsledek nenahrazuje build nového hardware firmware.
- Zdrojové změny mimo configs/skystars_5inch: žádné. Původní vývojová větev zůstává na 8dc66c8d18bfe591387333059c35888af5d08f5a.
- Hardware, upload, motorové testy, aktivní UTB: neprovedeno. Nic nebylo doinstalováno.

Ověření provedeno 2026-10-10. Bez nových C++/Python zdrojů nejsou nové compile/style regresní testy relevantní; kontrola whitespace provedena před lokálním commitem.
