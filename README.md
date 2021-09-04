# 8bit-Hub
Firmware and Hardware of the 8bit-Hub, a device for 8bit era computers.

Setting up Arduino IDE to program your 8bit-Hub: 

(1) Clone the Repo @ https://github.com/8bit-Dude/8bit-Hub.git/trunk/Firmware

(2) Install the Arduino IDE

(3) File > Preferences > Sketchbook Location  ---> Set to your local copy of "/Firmware" Folder
    (Restart the Arduino IDE for changes to take effect)

(4) File > Preferences > Additional Boards Manager URLs ---> https://github.com/esp8266/Arduino/releases/download/2.7.4/package_esp8266com_index.json

(5) Tools > Board > Boards Manager ---> search esp8266, then click install

----------------------------------------------------------------------

Flash new sketch (Core Part): 
(Switches set to ON ON ON ON OFF OFF OFF OFF)

(1) Tools > Board > optiboot-avr (in sketchbook) > Arduino Mega 2560 (Optiboot)

(2) Load Hub-Mega2560.ino

(3) Edit the code, for example these lines:

    // Override with some defaults
    if (!strlen(ssid) && !strlen(pswd)) {
        strcpy(ssid, "8bit-Unity");
        strcpy(pswd, "0123456789"); 
    }

(4) Connect Hub to PC, then Tools > Port > Select COM number

(5) Upload sketch

(6) Tools > Serial Monitor ---> Set Baud Rate to 115200

"Et Voila"! You can use the serial monitor to get information/debugging messages.
If you want to revert to "official" firmware, set:  char megaVersion[5] = "v0.1";
The Hub will offer you to download latest firmware (e.g. v0.3) next time you boot.

----------------------------------------------------------------------

Flash new sketch (Wifi Part): 
(Switches set to OFF OFF OFF OFF ON ON ON OFF)

(1) Tools > Board > ESP8266 boards > Generic ESP8266 Module

    Tools > Flash Size > 4MB (FS:2MB OTA:~1019KB)
    
    Tools > Erase Flash > All flash contents

(2) Load Hub-ESP8266.ino and edit code

(3) Connect Hub to PC, then Tools > Port > Select COM number

(4) Upload sketch, "Et Voila"!

Important: Set the switches back to ON ON ON ON OFF OFF OFF OFF after flashing the sketch, otherwise the serial monitor will not work.

----------------------------------------------------------------------

Burning the Optiboot boot loader onto a new board:

2 arduino mega boards are needed for this operation (one is used as progammer).

(1) Connect the boards pins as follows, then connect to USB:

    Programmer:  5V  GND  50  51  52  53
    
      Target:    5V  GND  50  51  52  RST

(2) Tools > Board > Arduino AVR Boards > Arduino Mega or Mega 2560

(3) File > Examples > 11.ArduinoISP > ArduinoISP

(4) Change code as follows:
      #define RESET 10                >  #define RESET 53
      //#define USE_OLD_STYLE_WIRING  >  #define USE_OLD_STYLE_WIRING 
      #define PIN_MOSI	11            >  #define PIN_MOSI 51
      #define PIN_MISO	12            >  #define PIN_MISO 50
      #define PIN_SCK	13            >  #define PIN_SCK  52
      
(5) Sketch > Upload

(6) Tools > Programmer > Arduino as ISP

(7) Tools > Board > optiboot-avr (in sketchbook) > Arduino Mega 2560 (Optiboot)

(8) Tools > Burn Loader, "Et Voila"!
