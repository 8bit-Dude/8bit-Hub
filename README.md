# 8bit-Hub
Firmware and Hardware of the 8bit-Hub, a device for 8bit era computers.

Here is a quick HOW-TO to setting up Arduino IDE for programming your 8bit-Hub (Core Part): 

(1) Clone the Repo @ https://github.com/8bit-Dude/8bit-Hub.git/trunk/Firmware

(2) Install the Arduino IDE

(3) File > Preferences > Sketchbook Location  ---> Set to Firmware Folder

(4) Tools > Board > optiboot-avr (in sketchbook) > Arduino Mega 2560 (Optiboot)

(5) Load Hub-Mega2560.ino

(6) Edit the code, for example these lines:

    // Override with some defaults
    if (!strlen(ssid) && !strlen(pswd)) {
        strcpy(ssid, "8bit-Unity");
        strcpy(pswd, "0123456789"); 
    }

(7) Connect Hub to PC, then Tools > Port > Select COM number

(8) Upload sketch, and Voila!

If you want to revert to "official" firmware, change "v0.3" to:       char megaVersion[5] = "v0.2";
The Hub will offer you to download latest firmware (0.3) next time you boot.
