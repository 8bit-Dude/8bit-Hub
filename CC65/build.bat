echo off

set PATH=%PATH%;"..\..\8bit-unity\utils\cc65\bin\";"..\..\8bit-unity\utils\java\bin\"

echo "Apple Compilation"
cc65 -Cl -O -t apple2 src/hub.c
cl65 -o loader.bin -Cl -O -t apple2 src/main.c src/hub.s src/serial.s
copy utils\ProDOS190-140K.po hub-apple.do
java -jar utils/AppleCommander-1.6.0.jar -as hub-apple.do LOADER bin 0x0803 < loader.bin
del loader.bin

echo "Lynx Compilation"
cc65 -Cl -O -t lynx --cpu 65SC02 src/hub.c
cl65 -o hub-lynx.lnx -Cl -O -t lynx -C lynx.cfg src/main.c src/hub.s src/comlynx.s

echo "Oric Compilation"
cc65 -Cl -O -t atmos src/hub.c
cl65 -o hub-oric.bin -Cl -O -t atmos -C oric.cfg src/main.c src/hub.s src/VIA.s
utils\oric-header.exe hub-oric.bin hub-oric.tap $0501
del hub-oric.bin

pause