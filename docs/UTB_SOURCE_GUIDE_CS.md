# UTB: společný index zdrojů a pravidla komentářů

Tento přehled navazuje na lokální commit
`5a32da404dafd19546439bf6886ab9adb831f4c0`. Následné komentáře, uspořádání a úprava čitelnosti jsou
součástí navazujícího lokálního checkpointu. Schválený 62souborový checkpoint zůstává
zachovaný v historii, včetně původních reportů a manifestů.

## Kde jsou soubory pohromadě

| Skupina | Umístění a jednoznačný prefix | Vstupní body |
|---|---|---|
| Vlastní řídicí matematika a SHADOW manager | `libraries/AP_UTB/AP_UTB_*` | [PID](../libraries/AP_UTB/AP_UTB_RateController.cpp), [attitude](../libraries/AP_UTB/AP_UTB_SITLAttitude.cpp), [manager](../libraries/AP_UTB/AP_UTB.cpp) |
| Vlastní knihovní testy | `libraries/AP_UTB/tests/test_utb*` | [Registrace testů](../libraries/AP_UTB/tests/wscript) |
| Copter adaptér a diagnostika | `libraries/AP_UTB/Copter/mode_utb_*`, `libraries/AP_UTB/Copter/utb*` | [Aktivní SITL adaptér](../libraries/AP_UTB/Copter/mode_utb_rate_sitl.cpp), [SHADOW producer/worker](../libraries/AP_UTB/Copter/utb.cpp) |
| Instrumentace loggeru | `libraries/AP_Logger/AP_Logger_UTBBench.*` | [Měřicí kontrakt](../libraries/AP_Logger/AP_Logger_UTBBench.h), [unit test](../libraries/AP_Logger/tests/test_utb_bench.cpp) |
| SITL a offline nástroje | `libraries/AP_UTB/tools/test_utb_*`, `libraries/AP_UTB/tools/utb_*` | [Attitude experiment](../Tools/utb/tools/utb_attitude_sitl.py), [rate experiment](../Tools/utb/tools/utb_rate_optimization.py), [offline model](../Tools/utb/tools/utb_closed_loop.py) |
| Python unit testy | `libraries/AP_UTB/tests/python/test_utb_*` | [Attitude metriky](../Tools/utb/tests/test_utb_attitude_sitl.py), [offline modely](../Tools/utb/tests/test_utb_offline.py) |
| Referenční konfigurace | `configs/skystars_5inch/` | [Přehled profilů](../configs/skystars_5inch/README_CS.md) |
| Dokumentace | `docs/UTB_*_CS.md` | [Rozcestník](UTB_README_CS.md), [úplná mapa souborů](UTB_FILE_MAP_CS.md) |

Vlastní C++ zdroje jsou seskupené pod [libraries/AP_UTB](../libraries/AP_UTB/README_CS.md).
Copter adaptéry mají vlastní podadresář `Copter/` a C++ unit testy `tests/`.
Python nástroje jsou oddělené v [Tools/utb](../Tools/utb/README_CS.md),
s podadresáři `tools/` a `tests/`. Python kořen/importy a odkazy odpovídají
novým cestám. Jediný Python v C++ testovací složce je nativní Waf `wscript`.

Výjimkou jsou dva soubory `AP_Logger_UTBBench.*` a jejich C++ unit test.
Zůstávají u AP_Logger, protože BENCH musí být nezávislý na volitelném AP_UTB.
Původní integrační zásahy do ArduCopter, AP_Scheduler a AP_Motors zůstávají
v nativních souborech ArduPilotu. Dokumentace a fyzické referenční parametry
jsou přístupné z tohoto indexu; jejich přesun není potřeba pro seskupení kódu.

## Jak číst komentáře

Vzor je [komentare.txt](komentare.txt). C++ používá `///` dokumentační
komentáře; Python odpovídající `#` komentáře, aby se nezměnily existující
docstringy ani jejich použití v CLI nápovědě. Souborové hlavičky uvádějí
účel a platformový rozsah, funkce mají popis a relevantní parametry/výstup.
Používá se celý vzor: NOTICE, file, brief, author, version, date, všechny
sekce i jejich ukončení a funkční bloky brief/param/return/details.
Nepoužité sekce jsou označené `None`. Původní licence zůstávají zachované.
Autor `Jiri Kovar` odpovídá Git identitě projektu. Verze
`YYYY-MM-DD` je datum poslední revize souboru, nikoli číslo firmware.
Při současné úpravě se změněné soubory označují `2026-10-17`.
Při další úpravě komentářů se revize může změnit na nové datum.

`Public functions` zahrnuje globální/exportované funkce a veřejné metody.
`Private functions` označuje souborové static funkce, anonymní namespace,
neveřejné metody a v Pythonu vnořené či jménem neveřejné pomocné funkce.
`Private function prototypes` obsahuje jejich samostatné deklarace, pokud
existují. Podrobnost u funkce rozlišuje konkrétní kategorii; například static
veřejná metoda třídy není automaticky souborově lokální funkcí.
Viditelnost Copter metod se čte z původních hlaviček, ty se nekomentují.
Sekce mohou navazovat vícekrát podle rozsahu třídy či modulu; pořadí skutečných
deklarací, definic a inicializací se kvůli komentářům nemění.

Rozsah je 77 vlastních C++/header/Python source a test souborů podle mapy
FÁZE 2E. Komentáře se nevkládají do 22 upravených původních source/build
souborů ArduPilotu. `AP_Motors`, HAL/DShot, původní arming a EKF se nemění.
Parametrové anotace a původní log metadata zůstávají zachované.

Při čtení rozlišovat normalizovaný povel, skutečnou fyzikální veličinu a
diagnostický výsledek. SHADOW mixer nemá motorovou autoritu. Aktivní
attitude/rate adaptér zůstává pouze v SITL. Popis unit testu uvádí ověřovaný
kontrakt; není sám tvrzením, že proběhl nový hardware experiment.

## Lokální proměnné a čitelnost

Pracovní skaláry mají společný blok deklarací na začátku funkce a komentář
s významem a jednotkami. Hodnota se přiřazuje až v místě, kde ji výpočet
potřebuje; pořadí odečtů času, atomických čtení a PID operací zůstává zachované.
Pomocné objekty a reference jsou uvedené na začátku svého pracovního rozsahu.
Reference musí být svázaná s existujícím objektem už při deklaraci.

Rozsah objektů měřících dobu nebo spravujících zámek se nerozšiřuje.
Větší snapshoty a logovací pakety zůstávají za enable/compile guardem nebo
v příslušné větvi, aby se nezvětšoval stack workeru a režie vypnutého UTB.
Loop reference k právě zpracovávané ose se deklaruje na začátku iterace.
Testovací Arrange/Act/Assert bloky si ponechávají vlastní potřebný rozsah.

Zbytečný ukazatel na motorové pole je nahrazený referencí na pevné pole.
Queue používá platnou referenci na zámek mailboxu. Volitelný constructor
argument zůstává ukazatel kvůli původnímu testovacímu rozhraní. Ukazatele
na volitelný AHRS view, RC kanály, singleton a callback identity zůstávají
s kontrolami platnosti; jejich API připouští nepřítomný objekt.

## Kontrola změn

Původní komentářová úprava a přesun měly shodné C++ tokeny a Python AST
po normalizaci doložených cest. Současná úprava deklarací mění C++ tokeny;
nelze ji vydávat pouze za komentářovou změnu. Ověřuje se novou kompilací,
unit testy a SITL regresí. Matematické výrazy, bezpečnostní podmínky a
pořadí volání controllerů zůstávají zachované.

Záloha před úpravou, převod 40 Python/datových cest, přesný diff a výsledky
ověření jsou v `tmp/utb-readable-20261017_084027/`. Původní experimentální
archivy se nepřepisují. Záznamy tohoto ověření pocházejí z pracovního stromu před navazujícím lokálním commitem. Hardware experiment se neprováděl.

## Ověření úpravy čitelnosti, 17. 10. 2026

Implementováno: oddělené Python nástroje/testy, komentované deklarace
pracovních proměnných a odstranění dvou zbytečných interních ukazatelů.
Pořadí přiřazení a kontrol zůstává zachované; výjimky pro potřebný rozsah
objektů jsou popsané výše a v přehledu zdrojů.

Unit ověřeno: **98 C++ testů** (47 core, 9 allocation, 12 benchmark,
14 attitude, 9 rate, 3 synthetic, 4 matematické) a **128 Python testů**.
Nezávislé matematické porovnání: 30 946 vzorků ve 29 scénářích bez chyby;
CSV výstupy jsou také přesně shodné se starším archivovaným C++ executablem.

SITL ověřeno na finální variantě: DISARMED integrace, roll/pitch rate
2×3200 sad, roll attitude 5600 sad, SHADOW 200/400 Hz a guard/arming kontroly.
Samostatná UTB OFF runtime absence kontrola prošla. SHADOW regrese záměrně
vypíná logger a vyvolává plnou frontu; její drops se neinterpretují jako
výsledek stabilního hardware benchmarku. Naměřená SITL frekvence je
200,0009 / 399,9933 Hz; není to měření fyzického H743 ani jeho execution time.

Build ověřeno: SITL UTB ON/BENCH ON, finální SITL UTB ON/BENCH OFF,
SITL UTB OFF/BENCH OFF, SkystarsH7HD-bdshot UTB ON/BENCH ON a stejný
H743 build s vypnutým logováním. Param/logger metadata, flake8 a diff check
prošly. Astyle změněných C++ částí prošel; původní jednořádkový getter
`sitl_antiwindup()` v AP_UTB.h má stejnou formátovací odchylku jako před úpravou.

H743 ELF zachovává BENCH instrumentaci a neobsahuje aktivní UTB SITL mód
ani jeho controller helpery, i při vynuceném build option. Původních 2600
chráněných AP source souborů zůstalo beze změny. Tři porovnané producer/worker/
consumer funkce mají shodné compiler stack operace. Celkový H743 text je
oproti předchozímu layout buildu větší o 12 B, BSS o 4 B; změna velikosti
funkce je u dřívější konstrukce syntetického vzorku. Skutečný execution time,
WCET, jitter a stack watermark na H743 jsou **hardware neověřeno**.

Výsledky, přesné příkazy, SHA-256 artefaktů a záloha jsou v
`tmp/utb-readable-20261017_084027/`. Firmware nebyl nahrán, žádné parametry FC
se neměnily. Změny se ukládají do navazujícího lokálního checkpointu; původní HEAD zůstává jeho rodičem.
