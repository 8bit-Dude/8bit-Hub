echo off

set PATH=%PATH%;"..\..\8bit-unity\utils\cc65\bin\"

echo "Lynx Compilation"
cc65 -Cl -O -t lynx --cpu 65SC02 src/hub.c
cl65 -o hub-lynx.lnx -Cl -O -t lynx -C lynx.cfg src/main.c src/hub.s src/comlynx.s

echo "Oric Compilation"
cc65 -Cl -O -t atmos src/hub.c
cl65 -o hub-oric.bin -Cl -O -t atmos -C oric.cfg src/main.c src/hub.s src/VIA.s
utils\oric-header.exe hub-oric.bin hub-oric.tap $0501
del hub-oric.bin

pause