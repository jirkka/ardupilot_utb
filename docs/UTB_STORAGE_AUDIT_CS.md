# Audit diskového prostoru AP_UTB

> **Aktuální stav 26. 10. 2026: schválený úklid dokončen.** Smazáno přesně90265 schválených souborů, uvolněno1,90GiB bloků, zdroje a důkazy zachované. Části původního auditu níže popisují stav před úklidem; skutečné provedení a kontroly jsou v závěrečném oddílu.

## Výsledek

Audit dokončen, **nic nebylo smazáno**, neproběhlGC/reset/clean/stash/rebase, WSLshutdown, TRIM ani kompakceVHDX. HW-2B bylo nejprve dokončené a zapečetěné, diskový audit běžel až poté. Hlavní příčinou velikosti nejsou zbytečné firmware buildy: převládají rozsáhlé unikátní JSON/BIN/CSV důkazy historických SITL experimentů.

`du -B1 -s .`: **63.235 GiB (67897606144 B)** alokovaného místa včetněadresářů; `du --apparent-size`: **62.744 GiB (67370327848 B)**. To je asi67,90GB desetinně, nikoliv původní hrubý odhad56GB. Inventář211620 regular souborů má 63.120 GiB (67774836736 B) alokovanýchfilebloků a 62.697 GiB (67320734689 B) apparent. Rozdíl proti du zahrnuje adresářovébloky a vznik novýchauditníchsouborů během měření (CSV25MB, přesnýlist45,8MB); snapshoty nejsou atomické. Symlinky se nenásledovaly, hardlinky počítané jednou podle dev/inode, žádnéchybyčtení.

## Rozdělení

| Adresář | Alokované regular-file bloky | Souborů |
|---|---|---|
| / | 63.120 GiB (67774836736 B) | 211620 |
| tmp | 60.800 GiB (65284038656 B) | 170667 |
| .git | 0.864 GiB (927219712 B) | 1540 |
| modules | 0.506 GiB (543006720 B) | 17051 |
| build | 0.359 GiB (385581056 B) | 11074 |
| libraries | 0.354 GiB (380260352 B) | 5702 |
| .venv | 0.151 GiB (162467840 B) | 3425 |
| Tools | 0.081 GiB (87506944 B) | 1887 |
| docs | 0.002 GiB (1863680 B) | 87 |
| ArduCopter | 0.002 GiB (1650688 B) | 95 |
| configs | 0.000 GiB (421888 B) | 11 |
| .github | 0.000 GiB (286720 B) | 40 |
| .pytest_cache | 0.000 GiB (40960 B) | 4 |
| .vscode | 0.000 GiB (12288 B) | 3 |
| tests | 0.000 GiB (12288 B) | 3 |
| benchmarks | 0.000 GiB (4096 B) | 1 |
| kapitoly | 0.000 GiB (4096 B) | 1 |

Největšípodadresáře tmp:

| Adresář | Alokováno | Souborů |
|---|---|---|
| tmp/utb-2h-20261018_093406 | 10.571 GiB (11350392832 B) | 1698 |
| tmp/utb-2b-20261014_164442 | 4.820 GiB (5175947264 B) | 2510 |
| tmp/utb-3g2-20261025_071149 | 4.767 GiB (5118734336 B) | 39632 |
| tmp/utb-3a-20261019_142914 | 4.634 GiB (4976103424 B) | 1460 |
| tmp/utb-2g-20261017_165925 | 4.148 GiB (4453584896 B) | 1322 |
| tmp/utb-2i-20261018_204008 | 4.080 GiB (4380721152 B) | 972 |
| tmp/utb-2f-20261017_093117 | 3.069 GiB (3295281152 B) | 4396 |
| tmp/utb-pilot-safety-20261019_083041 | 2.791 GiB (2996924416 B) | 749 |
| tmp/utb-3g1-20261024_210139 | 2.529 GiB (2715557888 B) | 19825 |
| tmp/utb-3g3f-20261025_152000 | 2.486 GiB (2668982272 B) | 15099 |
| tmp/utb-2c-20261016_152820 | 2.357 GiB (2530697216 B) | 906 |
| tmp/utb-3g3d-readiness-20261025_134513 | 2.296 GiB (2465013760 B) | 1167 |
| tmp/utb-3e1-20261023_203945 | 2.220 GiB (2383347712 B) | 25222 |
| tmp/utb-3d3-20261021_191141 | 1.936 GiB (2078265344 B) | 16504 |
| tmp/utb-3f1-20261024_155715 | 1.376 GiB (1477935104 B) | 16971 |

VšechnyH743/SITL/build odpovídajícíkořeny v `build-directory-inventory.json`, úplnámetadata211620souborů vfiles.csv a agregace dohloubky3 vdirectories.json. Výslednébinárky, currentEXECON a defaultbuild nejsou kandidáty k odstranění.

## Co tvoří data

| Přípona | Apparent součet | Počet |
|---|---|---|
| .json | 21.912 GiB (23527860104 B) | 5719 |
| .bin | 21.729 GiB (23331532772 B) | 2672 |
| .csv | 7.971 GiB (8559123732 B) | 2990 |
| .tar.gz | 1.729 GiB (1856714209 B) | 53 |
| .d | 1.415 GiB (1518861536 B) | 54687 |
| [no extension] | 1.273 GiB (1367008844 B) | 3362 |
| .a | 0.928 GiB (996844138 B) | 154 |
| .pack | 0.825 GiB (885601537 B) | 22 |
| .h | 0.682 GiB (732324148 B) | 24999 |
| .tlog | 0.638 GiB (685458764 B) | 888 |
| .o | 0.633 GiB (679552164 B) | 54685 |
| .bundle | 0.610 GiB (655131775 B) | 1 |
| .tar | 0.438 GiB (470077440 B) | 4 |
| .txt | 0.303 GiB (325218316 B) | 2601 |
| .png | 0.201 GiB (215462572 B) | 1618 |
| .hex | 0.144 GiB (154968676 B) | 331 |
| .map | 0.121 GiB (129577890 B) | 446 |
| .apj | 0.117 GiB (125278168 B) | 111 |

JSON23,53GB, BIN23,33GB a CSV8,56GB nejsou automatickyduplikáty. Typickýstarýpackets.json má kolem200MB; největší2Hadresář11,35GB je série validací, jejich unikátnímetadata jsou součástídůkazů. Komprimovanýrepoarchive/bundlecelkem přibližně2,99GB, není tohlavnípříčina.

## Dvacet největších souborů

| Soubor | Apparent B | Alokované B |
|---|---|---|
| .git/objects/pack/pack-a280b7a155117e5f8d35a14ef8946f4997501b12.pack | 655248709 | 655253504 |
| tmp/utb-synthetic-reference-20261012_085037/before-history-cleanup.bundle | 655131775 | 655138816 |
| tmp/utb-2g-20261017_165925/tracked-before.tar | 469821440 | 469823488 |
| tmp/utb-3e1-20261023_203945/checkpoint.tar.gz | 308409568 | 308412416 |
| tmp/utb-2f-20261017_093117/tracked-HEAD.tar.gz | 308394835 | 308396032 |
| tmp/utb-3d3-20261021_191141/checkpoint.tar.gz | 308377449 | 308379648 |
| tmp/utb-3d2-trace-20261021_070545/checkpoint.tar.gz | 308361409 | 308363264 |
| tmp/utb-packed-log-20261011/baseline-head.tar.gz | 308026136 | 308027392 |
| tmp/utb-1ha-checkpoint/workspace.tar.gz | 307731958 | 307732480 |
| tmp/utb-2h-20261018_093406/pre-final-guards/gain-guards/packets.json | 204930433 | 204931072 |
| tmp/utb-2h-20261018_093406/validation-final-attempt2/after-nominal-3/packets.json | 204444046 | 204447744 |
| tmp/utb-2h-20261018_093406/validation-final-attempt2/after-nominal-1/packets.json | 204069465 | 204070912 |
| tmp/utb-2h-20261018_093406/validation-v2/before-nominal-1/packets.json | 203990775 | 203993088 |
| tmp/utb-2h-20261018_093406/validation-final-attempt2/after-nominal-2/packets.json | 203979134 | 203984896 |
| tmp/utb-2h-20261018_093406/validation-v2/before-nominal-2/packets.json | 203968810 | 203972608 |
| tmp/utb-2h-20261018_093406/validation-v2/before-nominal-3/packets.json | 203918338 | 203923456 |
| tmp/utb-2h-20261018_093406/validation-final-attempt2/after-inertia-plus10-relative/packets.json | 203907055 | 203907072 |
| tmp/utb-2h-20261018_093406/validation-final-attempt2/after-inertia-minus10-relative/packets.json | 203896247 | 203898880 |
| tmp/utb-2h-20261018_093406/validation-final/after-nominal-1/packets.json | 203565673 | 203567104 |
| tmp/utb-2h-20261018_093406/validation-v2/after-nominal-1/packets.json | 203443029 | 203444224 |

`.git`pack655MB a historickýbundle655MB se nesmějí smazat jakoobyčejnýcache. Source/header soubory vgeneratedbuild stromech takénavzdoryregenerovatelnostinejsou vdeleteplánu. Filtrovanécore dumpy: 0 přesnýchcore/core.N/.core souborů (`core-dumps.json`); jinédebug/source soubory sřetězcemcore se neoznačují za dump.

## Duplicity, linky a cache

SHA-256 přehrané prostejněvelkéAPJ/ELF/BIN skupiny≥1MB: 32 skupin, teoretickýsoučetnadbytečnýchkopií 0.082 GiB (88114429 B). Přesnécesty+hash vduplicate-apj-elf-bin.json. **Ani tynejsou automatickydeletekandidáti**: různéreporty očekávají různécesty, některéjsou rollbackfirmware. Nešlo oúplnýdedupvšechJSON/CSV; stejnéobsahy vjinémformátu nejsou byte-identickéduplicitní soubory.

2hardlinkovéinode/skupiny po3 cestách existují užuarchivůPACKED_CD_VALID/FIXED a odvozenýchC/Dlogů; nezabírají třikrátfyzickémísto.7symlinků (venvpython/lib64 a2generatornodepackages) ponecháno, nenásledovanévenmimo projekt. `.venv` a submoduly bezezměny. Project-local Pythoncache je vyčíslenávnávrhu; mimoprojektovýccache/OS cache se neprocházel ani nemazal kvůli omezenípráce nakořen projektu. RegenerovatelnéC/C++produkty jsouobjekty/dependency podtmp; žádné tvrzení, že celýbuildnebotmp je cache.

## Přesný návrh úklidu – vyžaduje samostatný souhlas

**Souborovýmanifest** `tmp/utb-storage-20261025_audit-01/cleanup-candidates-files.json`: 90265 přesných položek srelativnícestou/velikostí/SHA-256/důvodem/rizikem. SHA-256 celéhomanifestu:

`3218037c46fea662a890d854f7a6b8a8fddc966a3dd2eef3fc5ac57c9bf8ac84`

- 88835 `.o` a`.d` historickýchbuildů vtmp.
- 1430 `.pyc` mimo.git/modules/.venv; source.py zůstává.
- Celkováalokovanákapacita **1.899 GiB (2038775808 B)**, apparent1.711 GiB (1837114798 B).
- Žádnápoložka není vpůvodním7689souborovémcheckpointu, source.c/cpp/h, `.su`stackproof, APJ/ELF/BIN/tlog, report, patch, manifest nebo Gitmetadata.
- 11199 explicitněreferencovanýchintermediate souborů zachováno. Indexhledal přesnépath reference vMarkdown aSHA/manifest/evidence/checkpointJSON do25MB; neprovádělformálníanalýzu všech libovolnýchskriptů. Deletingbytecode/object nezrušíretainedfinalbinary, dalšíincrementalbuild ale bude musetrekonstruovatpříslušnéobjekty sezdroji/toolchainem. Bitověstejnýbudoucírebuild každéhistorickéverze tímnenígarantovaný.

| Skupina kandidátů | Alokováno | Počet | Důvod/riziko/doporučení |
|---|---|---|---|
| tmp/utb-3g2-20261025_071149 | 0.499 GiB (535265280 B) | 23776 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3e1-20261023_203945 | 0.394 GiB (423383040 B) | 18294 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3g1-20261024_210139 | 0.248 GiB (266330112 B) | 11876 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3f1-20261024_155715 | 0.247 GiB (265076736 B) | 11868 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3e3-20261024_092051 | 0.246 GiB (264507392 B) | 11856 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3d3-20261021_191141 | 0.228 GiB (244895744 B) | 11142 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-2f-20261017_093117 | 0.033 GiB (35024896 B) | 1221 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| Tools | 0.003 GiB (3014656 B) | 162 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-2g-20261017_165925 | 0.001 GiB (724992 B) | 39 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| libraries | 0.000 GiB (286720 B) | 5 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/__pycache__ | 0.000 GiB (73728 B) | 5 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3f3-hw-20261024_192647 | 0.000 GiB (45056 B) | 5 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3g3f-hw2a-20261025_reference-01 | 0.000 GiB (36864 B) | 4 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3g3f-hw2b-20261025_prepare-01 | 0.000 GiB (20480 B) | 2 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-time-fix-20261011 | 0.000 GiB (16384 B) | 4 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3e2-hw-20261024_083723 | 0.000 GiB (16384 B) | 1 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3e4-hw-20261024_101555 | 0.000 GiB (16384 B) | 1 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-packed-hw-20261011_165438 | 0.000 GiB (16384 B) | 1 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3d4-hw-20261022_202457 | 0.000 GiB (12288 B) | 1 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-3f2-hw-20261024_181323 | 0.000 GiB (8192 B) | 1 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |
| tmp/utb-packed-log-20261011 | 0.000 GiB (4096 B) | 1 | Regenerovatelné .o/.d/.pyc; final artifacts zachovat; pouze po schválení manifestu |

Nevykonávatwildcard ani recursive delete. Předkaždýmschválenýmunlinkem ověřit absolutníresolvedpath uvnitř `/home/jirka/ardupilot_utb`, regulárnífile/nonsymlink, SHA/size a případnénovéodkazy. Při neshoděstop; uchovat exactdeleted/skipped log. Ponechatadresáře,finálníbinárky,generatedsource aartefakty. Popřípadnéakci znovuověřitGit,původnísource hash,currentFW,HWA/Bseal,4původníBINarchivy a volnémísto. **Zatím početdeleted=0.**

## Pouze po archivaci – unikátní důkazy

`archive-only-candidates.json` obsahuje 1557 přesnýchvelkýchraw souborů (vybranépackets/eventsJSON, CSV≥10MB, tlog≥10MB aBIN≥10MB uvnitřlogs), alokováno **44.834 GiB (48140570624 B)**. Jde o **unikátnímeasurement/evidence, nikoliv dalšídeletepovolení**. Je nutnézvolitcílovéexternímédium/bezpečnýkomprimovanýarchiv,připravitcelo-souborovéSHA,reálněověřitrestore a zachovatrelativepath+indexreport→archive. Teprvepak schválitodstraněníoriginálů poexplicitnímseznamu. Dnešnímanifesty stáleodkazují napůvodnícesty; nelze jen odstranitdata a nechattvrditúplnoststarýchdůkazů. Existující4historickéHWarchivy acurrentfirmwarezůstávají přímo dostupné.

Tři reálnéprvní1MiBvzorky200MBJSON se gzip6 vRAM stlačily na5,0–5,6% původníhoobjemu (`compression-samples.json`). Je to **vzorkovanýodhadpotenciálu, ne změřenákompresevšechsouborů nebohotovýarchiv**; BIN/CSVse mohoulišit. Plnýkomprimovanýarchivse nevytvářel, originály se neměnily.

## Rozpočet a cílová velikost

| Varianta | Výsledek |
|---|---|
| Nyní | 63.235 GiB (67897606144 B) |
| Potenciál přesně navrženého cache úklidu | 1.899 GiB (2038775808 B) |
| Cíl po samotném schváleném cache úklidu | 61.336 GiB (65858830336 B) |
| Raw data jen po ověřené externí archivaci | 44.834 GiB (48140570624 B) |
| Teoretický projekt po cache+externím přesunu všech vybranýchraw | 16.501 GiB (17718259712 B) |
| Zachovat | Všechny zdroje/Git/submoduly, current+recoveryFW, patche/manifesty/reporty,4archivovanéBIN,frozenA/B/Flight/Timing/Execution aunikátnídůkazy doověřenéarchivace |
| Skutečně uvolněno | 0 B; žádný souhlas smazáním nebyl přijat |

Čísla jsou různékategorie, ne automatickysčítatelnégarance úspor naWindows. Komprimace uvnitřprojektu ušetříneznámýrozdíl proti archivedsize, externípřesunmůžeuvolnitvšechnyvybranébajtyjenpokud jej skutečně dovolíevidence/restoreplán. Auditnívýstupy vznikléběhempráce mírněnavyšujíaktuálnívelikost.

## WSL2, hostitelskýdisk a reclaim

Ubuntu registryVersion2, VHDX `C:\Users\Jirka\AppData\Local\wsl\{1743f9b7-acb1-4c20-a0e3-92280e61d2df}\ext4.vhdx`. Pouzečtenímetadata, žádnáoperace svirtuálnímdiskem. Length/Allocated vposlednímsnapshotu oba **70.756 GiB (75973525504 B)**. Jde o celý Ubuntu disk, ne jen tentoGitprojekt;Linuxfilesystem virtuálně1,081TB s~955GBfree neznamená tolikvolnéhoSSD naWindows. Host C poauditu free **77.108 GiB (82793816064 B)**; prvnísnapshot předHW měl85337821184B. Rozdíl neníúklid: vzniklaanalyzačníevidence a běželyostatníOSprocesy. WSLdiskfree z auditu 889.777 GiB (955391025152 B). Žádné„po smazání“ měřeníneexistuje.

Samostatnýbudoucípostup po schválenémsmazání: ověřitGit/artefakty, znovuzměřitLinuxdu/df aWindowsC/VHDX; zvolitověřenýWSL/VHDXreclaim podle nainstalovanéverze. Předkompakcí zvlášťschválitukončení všechWSLprocesů a zálohovatVHDX mimo aktivníběh, případnéLinuxTRIM a čistéWSLshutdown ažpotvrzeníobsluhy. Pak pouze podporovanákompakce (např. dostupnéOptimize-VHD nebo odpovídajícísystémovýpostup) nadpřesněověřenýmodpojenýmVHDX, nepoužívatdiskpartclean/format/resize/force. ZnovuspustitUbuntu, ověřitfilesystem/Git/SHA a skutreclaim. Dostupnostkonkrétního Hyper-Vcmdletu/WSLcompactsubcommandu se nynínetestovala; žádnýpříkazkompakce se nespustil. SamotnýunlinkuvnitřLinuxu nemusízmenšitVHDXnaWindows.

## Evidence a stav schválení

Rawinventory/files.csv, directories.json,largest20.json, extensions.json, archiveindex, cleanupmanifest+SHA, exactVHDXmetadata,du crosscheck, protectedintermediates, symlink/hardlink acompressionvzorky v `tmp/utb-storage-20261025_audit-01`. Prvnípomalejšíauditníprůchod byl předreferenčníanalýzou zastavenjen ujednoznačněvlastníhoLinuxPID a nahrazenindexovanýmprůchodem; žádnádiskovádata nebohardwareproces nezměněny. Konečnýaudit130,45s, žádnéchybyčtení.

**Čeká samostatné schválení přesného delete manifestu.** Archivace/kompakce mají vlastníbudoucíplán a souhlas; neproběhly. Žádný Gitcommit/push nebo změnahistorie.

## Provedený schválený úklid – 26. 10. 2026

**Dokončeno bez chyby.** Tento oddíl nahrazuje výše uvedený stav návrhu „nic nesmazáno / čeká na schválení“. Původní verze celého reportu je bitově zachovaná v `tmp/utb-storage-cleanup-20261026_approved-01/storage-report-before.md`; její hash odpovídá původnímu auditnímu manifestu. Historické manifesty se nepřepisovaly.

Samostatný souhlas: „Schvaluji pouze tento přesný úklid“, po výslovné pauze pokračování na „pokračuj“. Přesný schválený manifest SHA-256 `3218037c46fea662a890d854f7a6b8a8fddc966a3dd2eef3fc5ac57c9bf8ac84`, beze změny. Dokončení 2026-10-26T15:01:57.273064+01:00.

| Veličina | Skutečný výsledek |
|---|---:|
| Smazané soubory | 90265 (88835 .o/.d + 1430 .pyc) |
| Smazané adresáře | 0 |
| Uvolněné bloky schválených souborů | 1.898758 GiB (2038775808 B) |
| Odstraněná apparent velikost | 1.710946 GiB (1837114798 B) |
| Projekt před (du, včetně nové kontrolní evidence) | 63.248894 GiB (67912982528 B) |
| Projekt po (du) | 61.364758 GiB (65889906688 B) |
| Čistý pokles du projektu | 1.884136 GiB (2023075840 B) |
| Linux filesystem free před | 889.762840 GiB (955375575040 B) |
| Linux filesystem free po | 891.646976 GiB (957398650880 B) |
| Chyby, neshody, přeskočené položky | 0 |

Všechny položky byly ověřeny ještě před prvním smazáním, potom znovu bezprostředně před jednotlivým unlinkem: SHA-256, velikost, regular nonsymlink soubor a resolved absolutní cesta uvnitř repozitáře. Nové přímé reference z kontrolovaných manifestů nebyly nalezené. Nepoužil se plošný delete ani wildcard. `deleted.csv` obsahuje každou skutečně odstraněnou cestu, velikost, bloky a původní hash.

Rozdíl mezi součtem uvolněných bloků a čistým poklesem projektu je kontrolní evidence úklidu. Zůstala zachovaná, nikoliv dodatečně odstraněná pro kosmetické navýšení úspory.

Po smazání nezměněn žádný z ostatních souborů mimo vlastní adresář úklidu (kontrola size/inode/mtime); následné zde popsané aktualizace dvou reportů jsou záměrné a mají archiv původních bytů. Všech7689 původních chráněných souborů ověřeno SHA, currentEXECON APJ a4původníBIN archivy shodné, kompletní evidence HW-2A/HW-2B shodná. Git HEAD `11a518e5857b24cc554b3cb7e6492998013ddd1a`, větev `test/skystars-5inch`, index prázdný a Git status beze změny; diff--check PASS. Produkční zdroje, finální firmware/binary, stack .su a měřenádata se nemažou. Žádný kontakt sFC, žádnýupload/ARM/motorovýtest/Gitcommit nebo push.

44,83GiB archivních kandidátů zůstává zachovaných; jejich archivace nebyla tímto souhlasem povolená. Windows free a VHDX metadata po úklidu v `windows-after.json` / `vhdx-after.json`; žádný TRIM, WSL shutdown, sparse konverze ani kompakce neproběhly. Tento úklid sám negarantuje zmenšení VHDX nebo stejnou úsporu místa na Windows.

Evidence celého provedení: `tmp/utb-storage-cleanup-20261026_approved-01/`, approval/attempt/precheck/ready-to-delete/deleted.csv/result/noncandidate snapshot/critical artifact hashes a vlastní finální SHA manifest. `historical-report-copies.json` mapuje původní reporty na zachované kopie odpovídající jejich původním manifestům.
