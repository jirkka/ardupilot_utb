# UTB: Python nástroje a testy

Python je oddělený od [C++ knihovny](../../libraries/AP_UTB/README_CS.md).
[Hlavní přehled](../../docs/UTB_README_CS.md) a
[mapa souborů](../../docs/UTB_FILE_MAP_CS.md) popisují návaznosti.

```text
Tools/utb/
  tools/                     SITL runnery, offline analýza a hardware nástroje
    utb_1he_demo_plant.json   demonstrační dataset, nikoli letová identifikace
  tests/                     Python unit testy
```

Skripty mají jednoznačné `utb_` / `test_utb_` prefixy. Cesty se odvozují
od umístění souboru, všechny příkazy níže se spouštějí z kořene repository.
Stávající prostředí `.venv` obsahuje potřebné závislosti.

```sh
UTB_AUDIT_EVIDENCE="$PWD/tmp/utb-1hd-offline-20261012_162639/download-audit/evidence.json" \
  .venv/bin/python -B -m unittest discover -s Tools/utb/tests -p 'test_utb*.py' -v
.venv/bin/python -B Tools/utb/tools/test_utb_active_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-basic --suite basic --speedup 2
.venv/bin/python -B Tools/utb/tools/utb_attitude_sitl.py --binary build/sitl/bin/arducopter --output tmp/NEW-attitude --suite preliminary
```

Nejdříve sestavit správnou SITL variantu podle C++ přehledu. Runnery spouštět
po jednom a do nového výstupního adresáře. Hardware capture/download nástroje
mají vlastní podmínky a nejsou součástí běžného SITL ověření. Samotný přesun
Python souborů nezahajuje hardware experiment ani nemění parametry FC.

Převod 40 cest, předchozí pracovní strom a výsledky regrese jsou archivované
v `tmp/utb-readable-20261017_084027/`. Historické příkazy v původních reportech
zůstávají záznamem tehdejšího umístění, jejich souborové odkazy vedou na
aktuální zdroje. Komentářová šablona a autor zůstávají zachované;
`@version` a `@date` uvádějí datum revize zdrojového souboru.

## Řízené pilotní ladění2H

`tools/utb_pilot_tuning.py` používá pouze vlastní lokální SITL, skutečné RC,
readback17 gains, oboustranné kroky a SHA-vázaný gate. Nepřijímá COM port.
JSON model má relativní cestu od run adresáře. Chybějící settling zůstává
neověřený. `tests/test_utb_pilot_tuning.py` kontroluje metrické/parametrické
kontrakty bez připojení k vozidlu.
[Metoda, výsledky a reprodukce](../../docs/UTB_PILOT_PID_TUNING_CS.md).
